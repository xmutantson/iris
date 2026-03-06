#include "ax25/afsk.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// --- Modulator ---

AfskModulator::AfskModulator(int sample_rate)
    : sample_rate_(sample_rate),
      samples_per_bit_(sample_rate / AFSK_BAUD),
      phase_(0.0f),
      phase_inc_mark_(2.0f * M_PI * AFSK_MARK_FREQ / sample_rate),
      phase_inc_space_(2.0f * M_PI * AFSK_SPACE_FREQ / sample_rate) {}

void AfskModulator::reset() {
    phase_ = 0.0f;
}

std::vector<float> AfskModulator::modulate(const std::vector<uint8_t>& bits) {
    std::vector<float> samples;
    samples.reserve(bits.size() * samples_per_bit_);

    for (uint8_t bit : bits) {
        float inc = (bit == 1) ? phase_inc_mark_ : phase_inc_space_;
        for (int s = 0; s < samples_per_bit_; s++) {
            samples.push_back(std::sin(phase_));
            phase_ += inc;
            if (phase_ > 2.0f * M_PI)
                phase_ -= 2.0f * M_PI;
        }
    }
    return samples;
}

// --- Demodulator ---

AfskDemodulator::AfskDemodulator(int sample_rate)
    : sample_rate_(sample_rate),
      samples_per_bit_(sample_rate / AFSK_BAUD),
      mark_i_(0), mark_q_(0), space_i_(0), space_q_(0),
      mark_phase_(0), space_phase_(0),
      clock_phase_(0), prev_sample_(0), prev_decision_(0) {
    // Allocate correlation buffer
    corr_buf_.resize(samples_per_bit_, 0.0f);
    corr_idx_ = 0;
}

void AfskDemodulator::reset() {
    mark_i_ = mark_q_ = space_i_ = space_q_ = 0;
    mark_phase_ = space_phase_ = 0;
    clock_phase_ = 0;
    prev_sample_ = 0;
    prev_decision_ = 0;
    std::fill(corr_buf_.begin(), corr_buf_.end(), 0.0f);
    corr_idx_ = 0;
}

std::vector<uint8_t> AfskDemodulator::demodulate(const float* samples, size_t count) {
    std::vector<uint8_t> bits;

    float mark_inc = 2.0f * M_PI * AFSK_MARK_FREQ / sample_rate_;
    float space_inc = 2.0f * M_PI * AFSK_SPACE_FREQ / sample_rate_;

    for (size_t i = 0; i < count; i++) {
        float s = samples[i];

        // Sliding window correlation over exactly one bit period
        // Mark correlator
        float mi = s * std::cos(mark_phase_);
        float mq = s * std::sin(mark_phase_);
        // Space correlator
        float si = s * std::cos(space_phase_);
        float sq = s * std::sin(space_phase_);

        mark_phase_  += mark_inc;
        space_phase_ += space_inc;
        if (mark_phase_  > 2.0f * M_PI) mark_phase_  -= 2.0f * M_PI;
        if (space_phase_ > 2.0f * M_PI) space_phase_ -= 2.0f * M_PI;

        // Leaky integrator with time constant = 1 bit period
        float alpha = 2.0f / samples_per_bit_;
        mark_i_  += alpha * (mi - mark_i_);
        mark_q_  += alpha * (mq - mark_q_);
        space_i_ += alpha * (si - space_i_);
        space_q_ += alpha * (sq - space_q_);

        float mark_energy  = mark_i_ * mark_i_ + mark_q_ * mark_q_;
        float space_energy = space_i_ * space_i_ + space_q_ * space_q_;

        float decision = mark_energy - space_energy;

        // Clock recovery: detect zero crossings in decision signal
        clock_phase_ += 1.0f / samples_per_bit_;

        // Zero crossing — adjust clock phase
        if ((prev_decision_ > 0 && decision <= 0) ||
            (prev_decision_ <= 0 && decision > 0)) {
            float error = clock_phase_ - 0.5f;
            clock_phase_ -= error * 0.2f;
        }

        if (clock_phase_ >= 1.0f) {
            clock_phase_ -= 1.0f;
            bits.push_back(decision > 0 ? 1 : 0);  // mark=1, space=0
        }

        prev_decision_ = decision;
    }

    return bits;
}

} // namespace iris
