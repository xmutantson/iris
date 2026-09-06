#include "ax25/gfsk.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Gaussian pulse shape for BT=0.5
static float gaussian(float t, float bt) {
    float a = 2.0f * M_PI * bt / std::sqrt(std::log(2.0f));
    return (a / std::sqrt(2.0f * M_PI)) * std::exp(-0.5f * a * a * t * t);
}

static int valid_sample_rate(int sample_rate) {
    if (sample_rate < GFSK_BAUD)
        throw std::invalid_argument("GFSK sample rate must be at least 9600 Hz");
    return sample_rate;
}

GfskModulator::GfskModulator(int sample_rate)
    : sample_rate_(valid_sample_rate(sample_rate)),
      samples_per_bit_(0),
      filter_idx_(0) {
    // Generate Gaussian filter coefficients
    float sum = 0;
    int center = FILTER_TAPS / 2;
    for (int i = 0; i < FILTER_TAPS; i++) {
        float t = (float)(i - center) * GFSK_BAUD / sample_rate_;
        filter_coeffs_[i] = gaussian(t, 0.5f);
        sum += filter_coeffs_[i];
    }
    // Normalize
    for (int i = 0; i < FILTER_TAPS; i++)
        filter_coeffs_[i] /= sum;

    std::memset(filter_buf_, 0, sizeof(filter_buf_));
}

void GfskModulator::reset() {
    samples_per_bit_ = 0;
    filter_idx_ = 0;
    std::memset(filter_buf_, 0, sizeof(filter_buf_));
}

std::vector<float> GfskModulator::modulate(const std::vector<uint8_t>& bits) {
    std::vector<float> samples;
    samples.reserve(bits.size() * (sample_rate_ / GFSK_BAUD + 1));

    for (uint8_t bit : bits) {
        float symbol = bit ? 1.0f : -1.0f;
        int64_t sample_phase = (int64_t)samples_per_bit_ + sample_rate_;
        int samples_this_bit = sample_phase / GFSK_BAUD;
        samples_per_bit_ = sample_phase % GFSK_BAUD;

        for (int s = 0; s < samples_this_bit; s++) {
            // Push symbol into Gaussian filter (sample-rate)
            filter_buf_[filter_idx_] = symbol;
            filter_idx_ = (filter_idx_ + 1) % FILTER_TAPS;

            // Convolve — output is Gaussian-filtered baseband NRZ
            float filtered = 0;
            for (int j = 0; j < FILTER_TAPS; j++) {
                int idx = (filter_idx_ + j) % FILTER_TAPS;
                filtered += filter_buf_[idx] * filter_coeffs_[j];
            }

            // Output baseband signal directly (drives radio FM modulator)
            samples.push_back(filtered);
        }
    }
    return samples;
}

// --- Demodulator ---

GfskDemodulator::GfskDemodulator(int sample_rate)
    : sample_rate_(valid_sample_rate(sample_rate)),
      samples_per_bit_(0),
      clock_phase_(0),
      clock_freq_((float)GFSK_BAUD / sample_rate_),
      prev_sample_(0) {}

void GfskDemodulator::reset() {
    clock_phase_ = 0;
    clock_freq_ = (float)GFSK_BAUD / sample_rate_;
    prev_sample_ = 0;
}

std::vector<uint8_t> GfskDemodulator::demodulate(const float* samples, size_t count) {
    std::vector<uint8_t> bits;

    for (size_t i = 0; i < count; i++) {
        float s = samples[i];

        // 2nd-order clock recovery PLL with damping
        // Proportional (alpha) corrects phase, integral (beta) tracks frequency drift
        constexpr float pll_alpha = 0.15f;   // Phase gain (damped from 0.3)
        constexpr float pll_beta  = 0.005f;  // Frequency gain
        clock_phase_ += clock_freq_;

        // Detect zero crossing for clock adjustment
        if ((prev_sample_ > 0 && s <= 0) || (prev_sample_ <= 0 && s > 0)) {
            float error = clock_phase_ - 0.5f;
            clock_phase_ -= pll_alpha * error;
            clock_freq_ = std::clamp(clock_freq_ - pll_beta * error,
                                     std::numeric_limits<float>::epsilon(),
                                     1.0f);
        }

        if (clock_phase_ >= 1.0f) {
            clock_phase_ -= 1.0f;
            // Baseband signal: positive = 1, negative = 0
            bits.push_back(s >= 0 ? 1 : 0);
        }

        prev_sample_ = s;
    }
    return bits;
}

// --- G3RUH Scrambler ---

void G3ruhScrambler::scramble(std::vector<uint8_t>& bits) {
    for (auto& b : bits) {
        // Feedback from taps at x^17 (bit 16) and x^12 (bit 11)
        uint8_t feedback = ((reg_ >> 16) ^ (reg_ >> 11)) & 1;
        b ^= feedback;
        reg_ = ((reg_ << 1) | b) & 0x1FFFF;  // shift in scrambled bit, keep 17 bits
    }
}

void G3ruhScrambler::descramble(std::vector<uint8_t>& bits) {
    for (auto& b : bits) {
        uint8_t feedback = ((reg_ >> 16) ^ (reg_ >> 11)) & 1;
        uint8_t received = b;
        b ^= feedback;
        reg_ = ((reg_ << 1) | received) & 0x1FFFF;  // shift in received (scrambled) bit
    }
}

} // namespace iris
