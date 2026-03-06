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
// Processes audio samples, outputs data bits via callback or poll
class AfskDemodulator {
public:
    AfskDemodulator(int sample_rate = SAMPLE_RATE);

    // Process a block of audio samples.
    // Returns decoded NRZI bits.
    std::vector<uint8_t> demodulate(const float* samples, size_t count);

    void reset();

private:
    int sample_rate_;
    int samples_per_bit_;

    // Correlator state
    float mark_i_, mark_q_;
    float space_i_, space_q_;
    float mark_phase_, space_phase_;

    // Clock recovery
    float clock_phase_;
    float prev_sample_;
    float prev_decision_;

    // Correlation buffer
    std::vector<float> corr_buf_;
    int corr_idx_;
};

} // namespace iris

#endif
