#include "native/upconvert.h"
#include <cmath>
#include <cstring>

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
      phase_(0.0f), lpf_pos_(0) {
    phase_inc_ = 2.0f * (float)M_PI * center_freq_ / (float)sample_rate_;
    // LPF cutoff: pass baseband signal, reject 2*fc image
    // For 2400 baud RRC alpha=0.2: half-BW = 2400*1.2/2 = 1440 Hz
    // Need flat passband up to 1440 Hz for QAM64/256 constellation accuracy
    // Image starts at 2*fc - BW = 2*1900 - 1440 = 2360 Hz from DC
    // Use center*0.95 for generous passband with 63-tap filter
    float cutoff = std::min(center_freq * 0.95f, 2200.0f);
    design_lpf(cutoff);
    std::memset(lpf_buf_i_, 0, sizeof(lpf_buf_i_));
    std::memset(lpf_buf_q_, 0, sizeof(lpf_buf_q_));
}

void Downconverter::reset() {
    phase_ = 0.0f;
    lpf_pos_ = 0;
    std::memset(lpf_buf_i_, 0, sizeof(lpf_buf_i_));
    std::memset(lpf_buf_q_, 0, sizeof(lpf_buf_q_));
}

void Downconverter::design_lpf(float cutoff_hz) {
    // Windowed sinc FIR lowpass filter
    float fc = cutoff_hz / (float)sample_rate_;
    int M = LPF_TAPS - 1;
    float sum = 0;
    for (int i = 0; i < LPF_TAPS; i++) {
        float n = (float)(i - M / 2);
        if (i == M / 2) {
            lpf_coeffs_[i] = 2.0f * fc;
        } else {
            lpf_coeffs_[i] = std::sin(2.0f * (float)M_PI * fc * n) / ((float)M_PI * n);
        }
        // Hamming window
        float w = 0.54f - 0.46f * std::cos(2.0f * (float)M_PI * i / M);
        lpf_coeffs_[i] *= w;
        sum += lpf_coeffs_[i];
    }
    // Normalize for unity gain at DC
    for (int i = 0; i < LPF_TAPS; i++)
        lpf_coeffs_[i] /= sum;
}

float Downconverter::apply_lpf(float* buf, float sample) {
    buf[lpf_pos_] = sample;
    float out = 0;
    for (int i = 0; i < LPF_TAPS; i++) {
        int idx = (lpf_pos_ - i + LPF_TAPS) % LPF_TAPS;
        out += buf[idx] * lpf_coeffs_[i];
    }
    return out;
}

std::vector<float> Downconverter::audio_to_iq(const float* audio, size_t count) {
    std::vector<float> iq(count * 2);

    for (size_t i = 0; i < count; i++) {
        float s = audio[i];

        // Mix down to baseband
        float i_raw =  s * std::cos(phase_);
        float q_raw = -s * std::sin(phase_);

        // Lowpass filter to remove 2*fc image (I and Q use separate delay lines)
        lpf_buf_i_[lpf_pos_] = i_raw;
        lpf_buf_q_[lpf_pos_] = q_raw;
        float i_filt = 0, q_filt = 0;
        for (int j = 0; j < LPF_TAPS; j++) {
            int idx = (lpf_pos_ - j + LPF_TAPS) % LPF_TAPS;
            i_filt += lpf_buf_i_[idx] * lpf_coeffs_[j];
            q_filt += lpf_buf_q_[idx] * lpf_coeffs_[j];
        }
        iq[2 * i]     = i_filt * 2.0f;  // x2: mixer cos²/sin² averages to 1/2
        iq[2 * i + 1] = q_filt * 2.0f;

        lpf_pos_ = (lpf_pos_ + 1) % LPF_TAPS;

        phase_ += phase_inc_;
        if (phase_ > 2.0f * (float)M_PI)
            phase_ -= 2.0f * (float)M_PI;
    }

    return iq;
}

} // namespace iris
