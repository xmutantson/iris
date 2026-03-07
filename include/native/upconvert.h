#ifndef IRIS_UPCONVERT_H
#define IRIS_UPCONVERT_H

#include "common/types.h"
#include <vector>
#include <cstddef>

namespace iris {

// Mode A upconversion: complex baseband IQ -> real audio centered at 1800 Hz
// out[n] = I[n]*cos(2*pi*fc*n/fs) - Q[n]*sin(2*pi*fc*n/fs)
constexpr float MODE_A_CENTER_FREQ = 1800.0f;

class Upconverter {
public:
    Upconverter(float center_freq = MODE_A_CENTER_FREQ, int sample_rate = SAMPLE_RATE);

    // Convert interleaved IQ [I0,Q0,I1,Q1,...] to real audio samples
    std::vector<float> iq_to_audio(const float* iq, size_t iq_count);

    void reset();

private:
    float center_freq_;
    int sample_rate_;
    float phase_;       // oscillator phase (radians)
    float phase_inc_;   // phase increment per sample
};

class Downconverter {
public:
    Downconverter(float center_freq = MODE_A_CENTER_FREQ, int sample_rate = SAMPLE_RATE);

    // Convert real audio samples to interleaved IQ [I0,Q0,I1,Q1,...]
    std::vector<float> audio_to_iq(const float* audio, size_t count);

    void reset();

private:
    float center_freq_;
    int sample_rate_;
    float phase_;
    float phase_inc_;
};

} // namespace iris

#endif
