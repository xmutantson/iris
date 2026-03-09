#ifndef IRIS_AFSK_H
#define IRIS_AFSK_H

#include "common/types.h"
#include <vector>
#include <cstdint>

namespace iris {

// AFSK 1200 baud Bell 202 modulator
// Input: NRZI-encoded bit stream
// Output: audio samples (float, normalized to [-1, 1])
class AfskModulator {
public:
    AfskModulator(int sample_rate = SAMPLE_RATE);

    // Modulate a bit stream into audio samples
    std::vector<float> modulate(const std::vector<uint8_t>& bits);

    void reset();

private:
    int sample_rate_;
    int samples_per_bit_;
    float phase_;
    float phase_inc_mark_;
    float phase_inc_space_;
};

// AFSK 1200 baud Bell 202 demodulator
// Sliding-window correlation demodulator (matched filter over one bit period).
// Same approach as Direwolf / SoundModem.
class AfskDemodulator {
public:
    AfskDemodulator(int sample_rate = SAMPLE_RATE);

    // Process a block of audio samples.
    // Returns decoded NRZI bits.
    std::vector<uint8_t> demodulate(const float* samples, size_t count);

    void reset();
    void set_preemph_alpha(float alpha) { preemph_alpha_ = alpha; }

private:
    int sample_rate_;
    int samples_per_bit_;

    // Reference tone phases (free-running)
    float mark_phase_, space_phase_;

    // Sliding window circular buffers for I/Q correlation products
    // Each has samples_per_bit_ entries; we maintain running sums.
    std::vector<float> mark_i_buf_, mark_q_buf_;
    std::vector<float> space_i_buf_, space_q_buf_;
    float mark_i_sum_, mark_q_sum_;
    float space_i_sum_, space_q_sum_;
    int buf_idx_;

    // Clock recovery
    float clock_phase_;
    float prev_decision_;

    // De-emphasis compensation (pre-emphasis filter)
    float preemph_alpha_ = 0.95f;
    float preemph_prev_ = 0;
};

} // namespace iris

#endif
