#include "engine/snr.h"
#include <cmath>
#include <algorithm>

namespace iris {

float estimate_snr(const std::complex<float>* tx_symbols,
                   const std::complex<float>* rx_symbols, int count) {
    if (count <= 0) return 0.0f;

    // Compute channel estimate (1-tap: average ratio of rx/tx)
    std::complex<float> h(0, 0);
    float tx_power = 0;
    for (int i = 0; i < count; i++) {
        h += rx_symbols[i] * std::conj(tx_symbols[i]);
        tx_power += std::norm(tx_symbols[i]);
    }
    if (tx_power > 0) h /= tx_power;

    // Compute signal and noise power
    float signal_power = 0;
    float noise_power = 0;
    for (int i = 0; i < count; i++) {
        std::complex<float> expected = h * tx_symbols[i];
        std::complex<float> error = rx_symbols[i] - expected;
        signal_power += std::norm(expected);
        noise_power += std::norm(error);
    }

    if (noise_power < 1e-12f) return 60.0f;  // Effectively infinite SNR

    float snr_linear = signal_power / noise_power;
    return 10.0f * std::log10(snr_linear);
}

AGC::AGC(float target_rms, float attack, float decay)
    : target_rms_(target_rms), attack_(attack), decay_(decay),
      gain_(1.0f), rms_est_(target_rms) {}

void AGC::reset() {
    gain_ = 1.0f;
    rms_est_ = target_rms_;
}

float AGC::process(float sample) {
    // Track RMS with asymmetric attack/decay
    float abs_sample = std::abs(sample * gain_);
    float rate = (abs_sample > rms_est_) ? attack_ : decay_;
    rms_est_ += rate * (abs_sample - rms_est_);

    // Adjust gain
    if (rms_est_ > 1e-6f) {
        float target_gain = target_rms_ / rms_est_;
        gain_ = std::clamp(target_gain, 0.01f, 100.0f);
    }

    return sample * gain_;
}

void AGC::process_block(float* samples, int count) {
    for (int i = 0; i < count; i++)
        samples[i] = process(samples[i]);
}

} // namespace iris
