#include "native/channel_eq.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

void ChannelEqualizer::reset() {
    configured_ = false;
    taps_.clear();
    delay_line_.clear();
    delay_pos_ = 0;
    eq_curve_db_.clear();
}

void ChannelEqualizer::configure(const ProbeResult& my_rx_result,
                                  const NegotiatedPassband& passband,
                                  int sample_rate,
                                  float max_boost_db) {
    reset();
    sample_rate_ = sample_rate;

    if (!passband.valid) {
        IRIS_LOG("[CH-EQ] Passband not valid, skipping EQ");
        return;
    }

    // Step 1: Collect detected tone powers within the negotiated passband
    struct TonePoint {
        float freq_hz;
        float power_db;
        bool detected;
    };

    std::vector<TonePoint> tones;
    int n_detected_in_band = 0;

    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float freq = probe_tone_freq(k);
        if (freq < passband.low_hz - 50.0f || freq > passband.high_hz + 50.0f)
            continue;

        TonePoint tp;
        tp.freq_hz = freq;
        tp.power_db = my_rx_result.tone_power_db[k];
        tp.detected = my_rx_result.tone_detected[k];
        tones.push_back(tp);

        if (tp.detected) n_detected_in_band++;
    }

    if (n_detected_in_band < 3) {
        IRIS_LOG("[CH-EQ] Only %d tones in passband, need >= 3. Skipping EQ.", n_detected_in_band);
        return;
    }

    // Step 2: Interpolate gaps (undetected tones between detected ones)
    // Find first and last detected tone indices
    int first_det = -1, last_det = -1;
    for (int i = 0; i < (int)tones.size(); i++) {
        if (tones[i].detected) {
            if (first_det < 0) first_det = i;
            last_det = i;
        }
    }

    // Linear interpolation across gaps between detected tones
    for (int i = first_det; i <= last_det; i++) {
        if (tones[i].detected) continue;

        // Find nearest detected neighbors
        int left = -1, right = -1;
        for (int j = i - 1; j >= first_det; j--) {
            if (tones[j].detected) { left = j; break; }
        }
        for (int j = i + 1; j <= last_det; j++) {
            if (tones[j].detected) { right = j; break; }
        }

        if (left >= 0 && right >= 0) {
            float t = (float)(i - left) / (float)(right - left);
            tones[i].power_db = tones[left].power_db * (1.0f - t) +
                                tones[right].power_db * t;
            tones[i].detected = true;  // mark as interpolated
        } else if (left >= 0) {
            tones[i].power_db = tones[left].power_db;
            tones[i].detected = true;
        } else if (right >= 0) {
            tones[i].power_db = tones[right].power_db;
            tones[i].detected = true;
        }
    }

    // Step 3: Compute EQ gains — flatten to median power
    // Using median (not max) avoids amplifying noise at weak edges
    std::vector<float> detected_powers;
    for (int i = first_det; i <= last_det; i++) {
        if (tones[i].detected)
            detected_powers.push_back(tones[i].power_db);
    }
    std::sort(detected_powers.begin(), detected_powers.end());
    float median_power = detected_powers[detected_powers.size() / 2];

    // Build frequency/gain specification for FIR design
    // Only equalize within first_det..last_det range; unity outside
    std::vector<float> freq_hz;
    std::vector<float> gain_linear;

    // DC (0 Hz) — unity gain (passthrough)
    freq_hz.push_back(0.0f);
    gain_linear.push_back(1.0f);

    // Below passband — unity gain
    if (first_det >= 0 && tones[first_det].freq_hz > 100.0f) {
        freq_hz.push_back(tones[first_det].freq_hz - 50.0f);
        gain_linear.push_back(1.0f);
    }

    // EQ region: invert channel response (flatten to median)
    eq_curve_db_.clear();
    for (int i = first_det; i <= last_det; i++) {
        if (!tones[i].detected) continue;

        float eq_db = median_power - tones[i].power_db;
        // Limit boost to avoid noise amplification (RX) or overmodulation (TX).
        // Attenuation capped at -20 dB (extremely loud tones are rare).
        eq_db = std::max(-20.0f, std::min(max_boost_db, eq_db));

        float eq_linear = std::pow(10.0f, eq_db / 20.0f);

        freq_hz.push_back(tones[i].freq_hz);
        gain_linear.push_back(eq_linear);
        eq_curve_db_.push_back(eq_db);
    }

    // Above passband — unity gain
    if (last_det >= 0 && tones[last_det].freq_hz < sample_rate / 2.0f - 100.0f) {
        freq_hz.push_back(tones[last_det].freq_hz + 50.0f);
        gain_linear.push_back(1.0f);
    }

    // Nyquist — unity gain
    freq_hz.push_back((float)(sample_rate / 2));
    gain_linear.push_back(1.0f);

    // Check if EQ range is significant enough to bother
    float max_eq = *std::max_element(eq_curve_db_.begin(), eq_curve_db_.end());
    float min_eq = *std::min_element(eq_curve_db_.begin(), eq_curve_db_.end());
    float eq_range = max_eq - min_eq;

    if (eq_range < 1.0f) {
        IRIS_LOG("[CH-EQ] Channel already flat (range %.1f dB), skipping EQ", eq_range);
        return;
    }

    // Step 4: Design linear-phase FIR via frequency sampling
    constexpr int N_TAPS = 127;  // ~2.6ms at 48kHz, odd for type-I linear phase
    design_fir(freq_hz, gain_linear, N_TAPS);

    // Initialize delay line
    delay_line_.assign(N_TAPS, 0.0f);
    delay_pos_ = 0;
    configured_ = true;

    IRIS_LOG("[CH-EQ] Configured: %d tones, range %.1f-%.1f Hz, EQ range %.1f dB, %d taps",
             n_detected_in_band, tones[first_det].freq_hz, tones[last_det].freq_hz,
             eq_range, N_TAPS);
}

void ChannelEqualizer::design_fir(const std::vector<float>& freq_hz,
                                   const std::vector<float>& gain_linear,
                                   int n_taps) {
    // Frequency-sampling FIR design:
    // 1. Interpolate the sparse freq/gain specification onto a uniform FFT grid
    // 2. IFFT to get impulse response
    // 3. Window with Hamming and normalize

    int n_fft = 1;
    while (n_fft < n_taps * 4) n_fft <<= 1;  // oversample for smooth interpolation



    // Interpolate gain specification onto uniform FFT bins
    std::vector<float> H(n_fft / 2 + 1, 1.0f);

    for (int bin = 0; bin <= n_fft / 2; bin++) {
        float f = (float)bin * (float)sample_rate_ / (float)n_fft;

        // Find bracketing points in the specification
        int lo = 0;
        for (int j = 1; j < (int)freq_hz.size(); j++) {
            if (freq_hz[j] > f) break;
            lo = j;
        }
        int hi = std::min(lo + 1, (int)freq_hz.size() - 1);

        if (lo == hi || freq_hz[hi] <= freq_hz[lo]) {
            H[bin] = gain_linear[lo];
        } else {
            float t = (f - freq_hz[lo]) / (freq_hz[hi] - freq_hz[lo]);
            t = std::max(0.0f, std::min(1.0f, t));
            H[bin] = gain_linear[lo] + t * (gain_linear[hi] - gain_linear[lo]);
        }
    }

    // IFFT of real symmetric magnitude response → linear-phase impulse response
    // For linear phase: H is real and symmetric, so IFFT gives real symmetric h[n]
    // Use cosine series: h[n] = (1/N) * sum_k { H[k] * cos(2*pi*k*n/N) }
    int center = n_taps / 2;
    taps_.resize(n_taps);

    for (int n = 0; n < n_taps; n++) {
        float sum = H[0];  // DC component
        for (int k = 1; k < n_fft / 2; k++) {
            float phase = 2.0f * (float)M_PI * k * (n - center) / (float)n_fft;
            sum += 2.0f * H[k] * std::cos(phase);
        }
        sum += H[n_fft / 2] * std::cos((float)M_PI * (n - center));  // Nyquist
        taps_[n] = sum / (float)n_fft;
    }

    // Apply Hamming window
    for (int n = 0; n < n_taps; n++) {
        float w = 0.54f - 0.46f * std::cos(2.0f * (float)M_PI * n / (n_taps - 1));
        taps_[n] *= w;
    }

    // Normalize: unity gain at passband center (preserve signal level)
    float center_freq = 0;
    float center_gain_target = 0;
    // Find the center of the EQ region
    for (int i = 0; i < (int)freq_hz.size(); i++) {
        if (gain_linear[i] != 1.0f || (i > 0 && i < (int)freq_hz.size() - 1)) {
            center_freq += freq_hz[i];
            center_gain_target += gain_linear[i];
        }
    }
    if (freq_hz.size() > 2) {
        center_freq /= (freq_hz.size() - 2);
        center_gain_target /= (freq_hz.size() - 2);
    }

    // Measure actual filter gain at the passband center
    float actual_gain = 0;
    for (int n = 0; n < n_taps; n++) {
        float phase = 2.0f * (float)M_PI * center_freq * (n - center) / (float)sample_rate_;
        actual_gain += taps_[n] * std::cos(phase);
    }

    if (std::abs(actual_gain) > 1e-6f) {
        float scale = center_gain_target / std::abs(actual_gain);
        for (auto& t : taps_) t *= scale;
    }
}

void ChannelEqualizer::apply(float* audio, int count) {
    if (!configured_ || taps_.empty()) return;

    int n_taps = (int)taps_.size();

    for (int i = 0; i < count; i++) {
        // Write input sample into circular delay line
        delay_line_[delay_pos_] = audio[i];

        // Compute FIR output
        float sum = 0;
        int pos = delay_pos_;
        for (int k = 0; k < n_taps; k++) {
            sum += taps_[k] * delay_line_[pos];
            if (--pos < 0) pos = n_taps - 1;
        }

        audio[i] = sum;

        if (++delay_pos_ >= n_taps) delay_pos_ = 0;
    }
}

} // namespace iris
