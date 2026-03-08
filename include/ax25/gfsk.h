#ifndef IRIS_GFSK_H
#define IRIS_GFSK_H

#include "common/types.h"
#include <vector>
#include <cstdint>

namespace iris {

// GFSK 9600 baud G3RUH-compatible modulator
class GfskModulator {
public:
    GfskModulator(int sample_rate = SAMPLE_RATE);

    // Modulate NRZI-encoded bit stream into audio samples
    std::vector<float> modulate(const std::vector<uint8_t>& bits);

    void reset();

private:
    int sample_rate_;
    int samples_per_bit_;
    // Gaussian filter state (BT=0.5, length 4 symbols)
    static constexpr int FILTER_TAPS = 21;  // 4 symbols * 5 samples + 1
    float filter_coeffs_[FILTER_TAPS];
    float filter_buf_[FILTER_TAPS];
    int filter_idx_;
};

// GFSK 9600 baud G3RUH-compatible demodulator
class GfskDemodulator {
public:
    GfskDemodulator(int sample_rate = SAMPLE_RATE);

    // Process audio samples, return decoded NRZI bits
    std::vector<uint8_t> demodulate(const float* samples, size_t count);

    void reset();

private:
    int sample_rate_;
    int samples_per_bit_;
    float clock_phase_;
    float clock_freq_;   // 2nd-order PLL: frequency tracking
    float prev_sample_;
};

} // namespace iris

#endif
