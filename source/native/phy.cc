#include "native/phy.h"
#include <cmath>
#include <algorithm>

namespace iris {

PhyConfig mode_a_config() {
    // Default: 1 kHz bandwidth (1200-2200 Hz) → SPS=60, 800 baud, 960 Hz occupied
    return mode_a_config(1000.0f);
}

PhyConfig mode_a_config(float bandwidth_hz) {
    // Find largest SPS where baud = SAMPLE_RATE/SPS fits in bandwidth
    // SPS must evenly divide SAMPLE_RATE for clean timing
    float max_baud = bandwidth_hz / (1.0f + RRC_ALPHA);
    int best_sps = 80;  // fallback: 600 baud
    for (int sps = 6; sps <= 80; sps++) {
        if (SAMPLE_RATE % sps != 0) continue;  // must divide evenly
        int baud = SAMPLE_RATE / sps;
        if (baud <= max_baud) {
            best_sps = sps;
            break;  // smallest SPS (highest baud) that fits
        }
    }
    int baud = SAMPLE_RATE / best_sps;
    return {baud, best_sps, Modulation::BPSK, RRC_ALPHA};
}

PhyConfig mode_b_config() {
    return {4800, SAMPLE_RATE / 4800, Modulation::QPSK, RRC_ALPHA};
}

PhyConfig mode_c_config() {
    // v2 future work: Mode C targets full FM channel bandwidth via SDR.
    // 19200 baud needs 96 kHz sample rate (SPS=5) — no practical VHF/UHF SDR
    // transceivers exist yet. See Mode C-Linear in spec for near-term alternative.
    // Placeholder: 9600 baud with higher-order QAM at 48 kHz.
    return {9600, SAMPLE_RATE / 9600, Modulation::QAM16, RRC_ALPHA};
}

// --- Modulator ---

NativeModulator::NativeModulator(const PhyConfig& config, int sample_rate)
    : config_(config), sample_rate_(sample_rate) {
    rrc_taps_ = rrc_filter(config_.rrc_alpha, RRC_SPAN, config_.samples_per_symbol);
}

std::vector<float> NativeModulator::modulate(const std::vector<uint8_t>& bits) {
    auto symbols = map_bits(bits, config_.modulation);
    return modulate_symbols(symbols);
}

std::vector<float> NativeModulator::modulate_symbols(
    const std::vector<std::complex<float>>& symbols) {

    int sps = config_.samples_per_symbol;

    // Upsample: insert symbols with (sps-1) zeros between them
    std::vector<float> i_up(symbols.size() * sps, 0.0f);
    std::vector<float> q_up(symbols.size() * sps, 0.0f);

    for (size_t k = 0; k < symbols.size(); k++) {
        i_up[k * sps] = symbols[k].real();
        q_up[k * sps] = symbols[k].imag();
    }

    // Pulse shape with RRC filter
    auto i_shaped = fir_filter(i_up, rrc_taps_);
    auto q_shaped = fir_filter(q_up, rrc_taps_);

    // Interleave I/Q and normalize RMS to match AFSK output level (1/sqrt(2))
    // Without this, the RRC pulse-shaped + upsampled signal is ~6 dB quieter
    // than AFSK's constant-envelope sinusoid at the same tx_level setting.
    size_t n = std::min(i_shaped.size(), q_shaped.size());
    std::vector<float> iq(n * 2);
    float energy = 0;
    for (size_t i = 0; i < n; i++) {
        iq[2 * i]     = i_shaped[i];
        iq[2 * i + 1] = q_shaped[i];
        energy += i_shaped[i] * i_shaped[i] + q_shaped[i] * q_shaped[i];
    }
    float rms = std::sqrt(energy / (float)n);
    float target_rms = 1.0f / std::sqrt(2.0f);  // match AFSK sine wave RMS
    if (rms > 1e-6f) {
        float scale = target_rms / rms;
        for (auto& s : iq) s *= scale;
    }

    return iq;
}

// --- Demodulator ---

NativeDemodulator::NativeDemodulator(const PhyConfig& config, int sample_rate)
    : config_(config), sample_rate_(sample_rate),
      mu_(0.0f), mu_gain_(0.01f),
      prev_re_(0), prev_im_(0),
      prev2_re_(0), prev2_im_(0) {
    rrc_taps_ = rrc_filter(config_.rrc_alpha, RRC_SPAN, config_.samples_per_symbol);
}

// Linear interpolation helper
static float interp(float y0, float y1, float mu) {
    return y0 + mu * (y1 - y0);
}

std::vector<uint8_t> NativeDemodulator::demodulate(const float* iq_samples, size_t count) {
    int sps = config_.samples_per_symbol;
    size_t n_samples = count / 2;  // IQ pairs

    // Separate I/Q
    std::vector<float> i_in(n_samples), q_in(n_samples);
    for (size_t i = 0; i < n_samples; i++) {
        i_in[i] = iq_samples[2 * i];
        q_in[i] = iq_samples[2 * i + 1];
    }

    // Matched filter (RRC)
    auto i_filt = fir_filter(i_in, rrc_taps_);
    auto q_filt = fir_filter(q_in, rrc_taps_);
    size_t filt_len = std::min(i_filt.size(), q_filt.size());

    // Symbol extraction with Gardner timing recovery
    symbols_.clear();

    // Combined TX+RX RRC filter delay
    int total_delay = 2 * RRC_SPAN * sps;

    // Extract symbols at sps intervals starting from the combined filter delay
    // Use Gardner TED for fine timing adjustment
    float mu = 0.0f;
    size_t i = total_delay;

    while (i + 1 < filt_len) {
        // Interpolate at current position + fractional offset
        size_t idx = i;
        float frac = mu;
        if (frac < 0) { idx--; frac += 1.0f; }
        if (idx + 1 >= filt_len) break;

        float re = interp(i_filt[idx], i_filt[idx + 1], frac);
        float im = interp(q_filt[idx], q_filt[idx + 1], frac);

        // Gardner TED using midpoint between current and previous symbol
        if (symbols_.size() > 0) {
            size_t mid_idx = (idx >= (size_t)(sps / 2)) ? idx - sps / 2 : 0;
            if (mid_idx + 1 < filt_len) {
                float mid_re = interp(i_filt[mid_idx], i_filt[mid_idx + 1], frac);
                float mid_im = interp(q_filt[mid_idx], q_filt[mid_idx + 1], frac);
                float err = mid_re * (re - prev_re_) + mid_im * (im - prev_im_);
                mu -= mu_gain_ * err;
                mu = std::clamp(mu, -0.5f, 0.5f);
            }
        }

        symbols_.push_back({re, im});
        prev_re_ = re;
        prev_im_ = im;

        i += sps;
    }

    // Demap symbols to bits
    return demap_bits(symbols_, config_.modulation);
}

} // namespace iris
