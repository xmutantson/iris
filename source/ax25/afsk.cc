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
// Sliding-window correlation: compute I/Q products with mark and space
// reference tones, sum over exactly one bit period using circular buffers.
// This is a matched filter / DFT bin at each tone frequency.

AfskDemodulator::AfskDemodulator(int sample_rate)
    : sample_rate_(sample_rate),
      samples_per_bit_(sample_rate / AFSK_BAUD),
      mark_phase_(0), space_phase_(0),
      mark_i_sum_(0), mark_q_sum_(0),
      space_i_sum_(0), space_q_sum_(0),
      buf_idx_(0),
      clock_phase_(0), prev_decision_(0) {
    mark_i_buf_.resize(samples_per_bit_, 0.0f);
    mark_q_buf_.resize(samples_per_bit_, 0.0f);
    space_i_buf_.resize(samples_per_bit_, 0.0f);
    space_q_buf_.resize(samples_per_bit_, 0.0f);
}

void AfskDemodulator::reset() {
    mark_phase_ = space_phase_ = 0;
    mark_i_sum_ = mark_q_sum_ = 0;
    space_i_sum_ = space_q_sum_ = 0;
    buf_idx_ = 0;
    clock_phase_ = 0;
    prev_decision_ = 0;
    preemph_prev_ = 0;
    tone_energy_smooth_ = 0;
    tone_energy_peak_ = 0;
    std::fill(mark_i_buf_.begin(), mark_i_buf_.end(), 0.0f);
    std::fill(mark_q_buf_.begin(), mark_q_buf_.end(), 0.0f);
    std::fill(space_i_buf_.begin(), space_i_buf_.end(), 0.0f);
    std::fill(space_q_buf_.begin(), space_q_buf_.end(), 0.0f);
}

std::vector<uint8_t> AfskDemodulator::demodulate(const float* samples, size_t count) {
    std::vector<uint8_t> bits;

    float mark_inc = 2.0f * M_PI * AFSK_MARK_FREQ / sample_rate_;
    float space_inc = 2.0f * M_PI * AFSK_SPACE_FREQ / sample_rate_;
    float block_peak = 0;

    for (size_t i = 0; i < count; i++) {
        // Optional pre-emphasis filter to compensate FM de-emphasis
        float raw = samples[i];
        float s;
        if (preemph_alpha_ > 0.0f) {
            s = raw - preemph_alpha_ * preemph_prev_;
            preemph_prev_ = raw;
        } else {
            s = raw;
        }

        // Compute I/Q correlation products for this sample
        float mi = s * std::cos(mark_phase_);
        float mq = s * std::sin(mark_phase_);
        float si = s * std::cos(space_phase_);
        float sq = s * std::sin(space_phase_);

        mark_phase_  += mark_inc;
        space_phase_ += space_inc;
        if (mark_phase_  > 2.0f * M_PI) mark_phase_  -= 2.0f * M_PI;
        if (space_phase_ > 2.0f * M_PI) space_phase_ -= 2.0f * M_PI;

        // Sliding window: subtract oldest sample, add new sample, advance index
        mark_i_sum_  += mi - mark_i_buf_[buf_idx_];
        mark_q_sum_  += mq - mark_q_buf_[buf_idx_];
        space_i_sum_ += si - space_i_buf_[buf_idx_];
        space_q_sum_ += sq - space_q_buf_[buf_idx_];

        mark_i_buf_[buf_idx_]  = mi;
        mark_q_buf_[buf_idx_]  = mq;
        space_i_buf_[buf_idx_] = si;
        space_q_buf_[buf_idx_] = sq;

        buf_idx_++;
        if (buf_idx_ >= samples_per_bit_) buf_idx_ = 0;

        // Energy = |correlation|^2 for each tone
        float mark_energy  = mark_i_sum_ * mark_i_sum_ + mark_q_sum_ * mark_q_sum_;
        float space_energy = space_i_sum_ * space_i_sum_ + space_q_sum_ * space_q_sum_;

        // Track combined tone energy for DCD
        float tone_e = mark_energy + space_energy;
        if (tone_e > block_peak) block_peak = tone_e;

        float decision = mark_energy - space_energy;

        // Clock recovery: detect zero crossings in decision signal
        clock_phase_ += 1.0f / samples_per_bit_;

        if ((prev_decision_ > 0 && decision <= 0) ||
            (prev_decision_ <= 0 && decision > 0)) {
            // Zero crossing should ideally occur at clock_phase_ = 0.5
            float error = clock_phase_ - 0.5f;
            clock_phase_ -= error * 0.2f;
        }

        if (clock_phase_ >= 1.0f) {
            clock_phase_ -= 1.0f;
            bits.push_back(decision > 0 ? 1 : 0);  // mark=1, space=0
        }

        prev_decision_ = decision;
    }

    // Smooth tone energy: fast attack (~1 block), slow decay (~200ms at 48kHz/1024)
    // This gives us a stable DCD signal that rises quickly when AFSK appears
    // and falls off smoothly when it stops.
    tone_energy_peak_ = block_peak;
    constexpr float attack = 0.8f;
    constexpr float decay = 0.05f;
    if (block_peak > tone_energy_smooth_)
        tone_energy_smooth_ = attack * block_peak + (1.0f - attack) * tone_energy_smooth_;
    else
        tone_energy_smooth_ = decay * block_peak + (1.0f - decay) * tone_energy_smooth_;

    return bits;
}

} // namespace iris
