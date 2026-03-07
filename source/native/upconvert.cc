#include "native/upconvert.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// --- Upconverter ---

Upconverter::Upconverter(float center_freq, int sample_rate)
    : center_freq_(center_freq), sample_rate_(sample_rate),
      phase_(0.0f) {
    phase_inc_ = 2.0f * (float)M_PI * center_freq_ / (float)sample_rate_;
}

void Upconverter::reset() {
    phase_ = 0.0f;
}

std::vector<float> Upconverter::iq_to_audio(const float* iq, size_t iq_count) {
    size_t n_samples = iq_count / 2;
    std::vector<float> audio(n_samples);

    for (size_t i = 0; i < n_samples; i++) {
        float I = iq[2 * i];
        float Q = iq[2 * i + 1];

        // out = I*cos(wt) - Q*sin(wt)
        audio[i] = I * std::cos(phase_) - Q * std::sin(phase_);

        phase_ += phase_inc_;
        if (phase_ > 2.0f * (float)M_PI)
            phase_ -= 2.0f * (float)M_PI;
    }

    return audio;
}

// --- Downconverter ---

Downconverter::Downconverter(float center_freq, int sample_rate)
    : center_freq_(center_freq), sample_rate_(sample_rate),
      phase_(0.0f) {
    phase_inc_ = 2.0f * (float)M_PI * center_freq_ / (float)sample_rate_;
}

void Downconverter::reset() {
    phase_ = 0.0f;
}

std::vector<float> Downconverter::audio_to_iq(const float* audio, size_t count) {
    std::vector<float> iq(count * 2);

    for (size_t i = 0; i < count; i++) {
        float s = audio[i];

        // I = s * cos(wt), Q = -s * sin(wt)
        // (conjugate mix to bring center_freq down to DC)
        iq[2 * i]     =  s * std::cos(phase_);
        iq[2 * i + 1] = -s * std::sin(phase_);

        phase_ += phase_inc_;
        if (phase_ > 2.0f * (float)M_PI)
            phase_ -= 2.0f * (float)M_PI;
    }

    return iq;
}

} // namespace iris
