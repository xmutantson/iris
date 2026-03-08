#include "probe/passband_probe.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// -----------------------------------------------------------------------
// Generate multi-tone probe signal
// -----------------------------------------------------------------------

int probe_generate(float* out, int max_samples, int sample_rate,
                   float amplitude) {
    int n_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate);
    if (n_samples > max_samples) n_samples = max_samples;

    // Per-tone amplitude (divide by sqrt(N) to keep total RMS reasonable)
    float tone_amp = amplitude / std::sqrt((float)PassbandProbeConfig::N_TONES);

    // Hann window for clean edges (no clicks)
    auto hann = [&](int i) -> float {
        return 0.5f * (1.0f - std::cos(2.0 * M_PI * i / (n_samples - 1)));
    };

    for (int i = 0; i < n_samples; i++) {
        float t = (float)i / sample_rate;
        float sample = 0.0f;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float freq = probe_tone_freq(k);
            sample += tone_amp * std::sin(2.0f * (float)M_PI * freq * t);
        }
        out[i] = sample * hann(i);
    }

    return n_samples;
}

// -----------------------------------------------------------------------
// Analyze received probe signal
// -----------------------------------------------------------------------

// Simple power-of-2 FFT (Cooley-Tukey, in-place, radix-2 DIT)
static void fft_inplace(float* re, float* im, int n) {
    // Bit-reversal permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    // Butterfly
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        float wR = std::cos(ang), wI = std::sin(ang);
        for (int i = 0; i < n; i += len) {
            float curR = 1.0f, curI = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                float uR = re[i + j], uI = im[i + j];
                float vR = re[i + j + len / 2] * curR - im[i + j + len / 2] * curI;
                float vI = re[i + j + len / 2] * curI + im[i + j + len / 2] * curR;
                re[i + j] = uR + vR;
                im[i + j] = uI + vI;
                re[i + j + len / 2] = uR - vR;
                im[i + j + len / 2] = uI - vI;
                float tmpR = curR * wR - curI * wI;
                curI = curR * wI + curI * wR;
                curR = tmpR;
            }
        }
    }
}

ProbeResult probe_analyze(const float* samples, int n_samples, int sample_rate) {
    ProbeResult result;

    // Use FFT size = next power of 2 >= n_samples (cap at 8192)
    int fft_n = 4096;
    while (fft_n < n_samples && fft_n < 8192) fft_n <<= 1;
    if (fft_n > 8192) fft_n = 8192;

    std::vector<float> re(fft_n, 0.0f), im(fft_n, 0.0f);

    // Window and copy samples
    int copy_n = std::min(n_samples, fft_n);
    for (int i = 0; i < copy_n; i++) {
        float w = 0.5f * (1.0f - std::cos(2.0 * M_PI * i / (copy_n - 1)));
        re[i] = samples[i] * w;
    }

    fft_inplace(re.data(), im.data(), fft_n);

    // Compute power spectrum (dB) for positive frequencies
    float bin_hz = (float)sample_rate / fft_n;
    int n_pos = fft_n / 2;
    std::vector<float> power_db(n_pos);
    for (int i = 0; i < n_pos; i++) {
        float mag2 = re[i] * re[i] + im[i] * im[i];
        power_db[i] = 10.0f * std::log10(mag2 / (fft_n * fft_n) + 1e-20f);
    }

    // Estimate noise floor: median of all bins in our range
    std::vector<float> range_powers;
    int bin_lo = (int)(PassbandProbeConfig::TONE_LOW_HZ / bin_hz);
    int bin_hi = (int)(PassbandProbeConfig::TONE_HIGH_HZ / bin_hz);
    bin_hi = std::min(bin_hi, n_pos - 1);
    for (int i = bin_lo; i <= bin_hi; i++)
        range_powers.push_back(power_db[i]);
    std::sort(range_powers.begin(), range_powers.end());
    float noise_floor = range_powers.empty() ? -80.0f :
                        range_powers[range_powers.size() / 4];  // 25th percentile

    // Check each expected tone frequency
    float threshold = noise_floor + PassbandProbeConfig::DETECT_THRESHOLD_DB;
    int first_detected = -1, last_detected = -1;

    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float freq = probe_tone_freq(k);
        int bin = (int)(freq / bin_hz + 0.5f);
        if (bin < 0 || bin >= n_pos) {
            result.tone_power_db[k] = -80.0f;
            result.tone_detected[k] = false;
            continue;
        }

        // Peak in +-1 bin neighborhood (handles slight frequency offset)
        float peak = power_db[bin];
        if (bin > 0) peak = std::max(peak, power_db[bin - 1]);
        if (bin < n_pos - 1) peak = std::max(peak, power_db[bin + 1]);

        result.tone_power_db[k] = peak;
        result.tone_detected[k] = (peak >= threshold);

        if (result.tone_detected[k]) {
            if (first_detected < 0) first_detected = k;
            last_detected = k;
            result.tones_detected++;
        }
    }

    if (first_detected >= 0 && last_detected >= 0) {
        result.low_hz = probe_tone_freq(first_detected);
        result.high_hz = probe_tone_freq(last_detected);
        result.valid = (result.tones_detected >= 3);  // Need at least 3 tones
    }

    return result;
}

// -----------------------------------------------------------------------
// Negotiate passband from two probe results
// -----------------------------------------------------------------------

NegotiatedPassband probe_negotiate(const ProbeResult& a_to_b,
                                    const ProbeResult& b_to_a) {
    NegotiatedPassband neg;
    neg.my_tx_their_rx = a_to_b;
    neg.their_tx_my_rx = b_to_a;

    if (!a_to_b.valid || !b_to_a.valid) return neg;

    // Usable band = intersection of both paths
    float low = std::max(a_to_b.low_hz, b_to_a.low_hz);
    float high = std::min(a_to_b.high_hz, b_to_a.high_hz);

    // Add safety margin
    low += PassbandProbeConfig::EDGE_MARGIN_HZ;
    high -= PassbandProbeConfig::EDGE_MARGIN_HZ;

    if (high <= low) return neg;  // No usable overlap

    neg.low_hz = low;
    neg.high_hz = high;
    neg.center_hz = (low + high) / 2.0f;
    neg.bandwidth_hz = high - low;
    neg.valid = true;

    return neg;
}

// -----------------------------------------------------------------------
// Serialize / deserialize
// -----------------------------------------------------------------------

static void put_f32(std::vector<uint8_t>& v, float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    v.push_back((u >>  0) & 0xFF);
    v.push_back((u >>  8) & 0xFF);
    v.push_back((u >> 16) & 0xFF);
    v.push_back((u >> 24) & 0xFF);
}

static float get_f32(const uint8_t* p) {
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

std::vector<uint8_t> probe_result_encode(const ProbeResult& r) {
    std::vector<uint8_t> out;
    out.reserve(18);
    out.push_back(0xBB);              // Magic
    put_f32(out, r.low_hz);           // 4 bytes
    put_f32(out, r.high_hz);          // 4 bytes
    out.push_back((uint8_t)r.tones_detected);  // 1 byte

    // 64-bit bitmap of detected tones (8 bytes)
    for (int byte = 0; byte < 8; byte++) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; bit++) {
            int idx = byte * 8 + bit;
            if (idx < PassbandProbeConfig::N_TONES && r.tone_detected[idx])
                b |= (1 << bit);
        }
        out.push_back(b);
    }

    return out;  // 18 bytes total
}

bool probe_result_decode(const uint8_t* data, size_t len, ProbeResult& r) {
    if (len < 18 || data[0] != 0xBB) return false;

    r.low_hz = get_f32(data + 1);
    r.high_hz = get_f32(data + 5);
    r.tones_detected = data[9];

    for (int byte = 0; byte < 8; byte++) {
        uint8_t b = data[10 + byte];
        for (int bit = 0; bit < 8; bit++) {
            int idx = byte * 8 + bit;
            if (idx < PassbandProbeConfig::N_TONES)
                r.tone_detected[idx] = (b >> bit) & 1;
        }
    }

    r.valid = (r.tones_detected >= 3 && r.high_hz > r.low_hz);
    return true;
}

} // namespace iris
