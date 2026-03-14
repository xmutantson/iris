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

    // 64 tones — sqrt(N) = 8 normalization keeps each tone clean and loud.
    // No clipping needed: peak-to-RMS ratio of N equal tones is sqrt(N),
    // so dividing by sqrt(N) keeps peaks at ±1.
    float norm = std::sqrt((float)PassbandProbeConfig::N_TONES);

    // Hann ramp for first/last 5% to avoid clicks
    int ramp = n_samples / 20;

    for (int i = 0; i < n_samples; i++) {
        float t = (float)i / sample_rate;
        float sample = 0.0f;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float freq = probe_tone_freq(k);
            sample += std::sin(2.0f * (float)M_PI * freq * t);
        }
        sample /= norm;

        // Ramp edges
        float env = 1.0f;
        if (i < ramp) env = (float)i / ramp;
        else if (i > n_samples - ramp) env = (float)(n_samples - i) / ramp;

        out[i] = sample * amplitude * env;
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

    // FFT size: 32768 @ 48kHz = 1.46 Hz bins, plenty for 67 Hz tone spacing.
    // For longer captures, segment-average: split into overlapping windows,
    // FFT each, take MAX power per bin. Max resists temporary fading (a tone
    // that dips in one segment is captured by others) and noise spurs (a spur
    // in one segment doesn't raise the overall floor).
    constexpr int FFT_N = 32768;
    int fft_n = FFT_N;
    if (n_samples < fft_n) {
        fft_n = 4096;
        while (fft_n < n_samples && fft_n < FFT_N) fft_n <<= 1;
    }

    float bin_hz = (float)sample_rate / fft_n;
    int n_pos = fft_n / 2;
    std::vector<float> power_db(n_pos, -200.0f);      // max across segments (tone detection)
    std::vector<float> power_db_mean(n_pos, 0.0f);     // mean across segments (null validation)

    // Segment parameters: 50% overlap
    int hop = fft_n / 2;
    int n_segments = 0;
    std::vector<float> re(fft_n), im(fft_n);

    for (int offset = 0; offset + fft_n <= n_samples; offset += hop) {
        // Window and copy this segment
        std::fill(re.begin(), re.end(), 0.0f);
        std::fill(im.begin(), im.end(), 0.0f);
        for (int i = 0; i < fft_n; i++) {
            float w = 0.5f * (1.0f - std::cos(2.0 * M_PI * i / (fft_n - 1)));
            re[i] = samples[offset + i] * w;
        }

        fft_inplace(re.data(), im.data(), fft_n);

        // Dual-merge: max for tone detection, sum for mean (null validation)
        float norm2 = (float)fft_n * (float)fft_n;
        for (int i = 0; i < n_pos; i++) {
            float mag2 = re[i] * re[i] + im[i] * im[i];
            float db = 10.0f * std::log10(mag2 / norm2 + 1e-20f);
            if (db > power_db[i]) power_db[i] = db;
            power_db_mean[i] += db;
        }
        n_segments++;
    }

    // If capture was shorter than one FFT window, do a single padded FFT
    if (n_segments == 0) {
        std::fill(re.begin(), re.end(), 0.0f);
        std::fill(im.begin(), im.end(), 0.0f);
        int copy_n = std::min(n_samples, fft_n);
        for (int i = 0; i < copy_n; i++) {
            float w = 0.5f * (1.0f - std::cos(2.0 * M_PI * i / (copy_n - 1)));
            re[i] = samples[i] * w;
        }
        fft_inplace(re.data(), im.data(), fft_n);
        float norm2 = (float)fft_n * (float)fft_n;
        for (int i = 0; i < n_pos; i++) {
            float mag2 = re[i] * re[i] + im[i] * im[i];
            power_db[i] = 10.0f * std::log10(mag2 / norm2 + 1e-20f);
            power_db_mean[i] = power_db[i];
        }
        n_segments = 1;
    }

    // Finalize mean spectrum
    if (n_segments > 1) {
        for (int i = 0; i < n_pos; i++)
            power_db_mean[i] /= n_segments;
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

    // Two-pass detection:
    // Pass 1: measure each tone's power and find the peak.
    // Pass 2: detect tones within DETECT_THRESHOLD_DB of the peak.
    // This relative approach works in both zero-noise (VB-Cable) and
    // real-radio environments without needing absolute floor tuning.
    int first_detected = -1, last_detected = -1;

    // Pass 1: measure per-tone power, find peak
    float peak_tone_power = -100.0f;
    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float freq = probe_tone_freq(k);
        int bin = (int)(freq / bin_hz + 0.5f);
        if (bin < 0 || bin >= n_pos) {
            result.tone_power_db[k] = -100.0f;
            continue;
        }

        // Peak in +-1 bin neighborhood (handles slight frequency offset)
        float peak = power_db[bin];
        if (bin > 0) peak = std::max(peak, power_db[bin - 1]);
        if (bin < n_pos - 1) peak = std::max(peak, power_db[bin + 1]);

        result.tone_power_db[k] = peak;
        if (peak > peak_tone_power) peak_tone_power = peak;
    }

    // Pass 2: detect tones within threshold of peak
    float threshold = peak_tone_power - PassbandProbeConfig::DETECT_THRESHOLD_DB;
    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        result.tone_detected[k] = (result.tone_power_db[k] >= threshold);
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

    // Pass 3: Comb validation — verify nulls between adjacent detected tones.
    // Real probe tones have deep nulls between them; broadband noise does not.
    if (result.valid) {
        int null_checks = 0, null_passes = 0;
        for (int k = 0; k < PassbandProbeConfig::N_TONES - 1; k++) {
            if (!result.tone_detected[k] || !result.tone_detected[k + 1])
                continue;
            null_checks++;

            // Midpoint frequency between adjacent tones
            float mid_freq = (probe_tone_freq(k) + probe_tone_freq(k + 1)) / 2.0f;
            int mid_bin = (int)(mid_freq / bin_hz + 0.5f);

            // Peak in ±1 bin neighborhood — use MEAN spectrum so random noise
            // peaks average out instead of accumulating via max-merge
            float mid_power = power_db_mean[mid_bin];
            if (mid_bin > 0) mid_power = std::max(mid_power, power_db_mean[mid_bin - 1]);
            if (mid_bin < n_pos - 1) mid_power = std::max(mid_power, power_db_mean[mid_bin + 1]);

            // Average power of the two flanking tones
            float tone_avg = (result.tone_power_db[k] + result.tone_power_db[k + 1]) / 2.0f;

            if (tone_avg - mid_power >= 10.0f)
                null_passes++;
        }

        // Reject if fewer than 70% of inter-tone nulls are deep enough.
        // Real probe nulls are 20+ dB deep; OFDM broadband has <6 dB variation.
        if (null_checks >= 3 && null_passes * 10 < null_checks * 7) {
            result.valid = false;
            result.tones_detected = 0;
        }
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
    // Wire format: magic(1) + low_hz(4) + high_hz(4) + n_tones_le16(2) + bitmap(N_TONES/8)
    constexpr int BITMAP_BYTES = (PassbandProbeConfig::N_TONES + 7) / 8;  // 8 for 64 tones
    constexpr int TOTAL = 1 + 4 + 4 + 2 + BITMAP_BYTES;                  // 19 bytes

    std::vector<uint8_t> out;
    out.reserve(TOTAL);
    out.push_back(0xBC);              // Magic (0xBC = v2 with 16-bit tone count)
    put_f32(out, r.low_hz);           // 4 bytes
    put_f32(out, r.high_hz);          // 4 bytes
    out.push_back((uint8_t)(r.tones_detected & 0xFF));        // low byte
    out.push_back((uint8_t)((r.tones_detected >> 8) & 0xFF)); // high byte

    // Bitmap of detected tones
    for (int byte = 0; byte < BITMAP_BYTES; byte++) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; bit++) {
            int idx = byte * 8 + bit;
            if (idx < PassbandProbeConfig::N_TONES && r.tone_detected[idx])
                b |= (1 << bit);
        }
        out.push_back(b);
    }

    // Capability flags (2 bytes LE, appended for v2+ peers)
    out.push_back((uint8_t)(r.capabilities & 0xFF));
    out.push_back((uint8_t)((r.capabilities >> 8) & 0xFF));

    return out;
}

bool probe_result_decode(const uint8_t* data, size_t len, ProbeResult& r) {
    constexpr int BITMAP_BYTES = (PassbandProbeConfig::N_TONES + 7) / 8;
    constexpr int TOTAL = 1 + 4 + 4 + 2 + BITMAP_BYTES;
    if (len < (size_t)TOTAL || data[0] != 0xBC) return false;

    r.low_hz = get_f32(data + 1);
    r.high_hz = get_f32(data + 5);
    r.tones_detected = (int)data[9] | ((int)data[10] << 8);

    for (int byte = 0; byte < BITMAP_BYTES; byte++) {
        uint8_t b = data[11 + byte];
        for (int bit = 0; bit < 8; bit++) {
            int idx = byte * 8 + bit;
            if (idx < PassbandProbeConfig::N_TONES)
                r.tone_detected[idx] = (b >> bit) & 1;
        }
    }

    r.valid = (r.tones_detected >= 3 && r.high_hz > r.low_hz);

    // Capability flags (optional, appended by v2+ peers)
    if (len >= (size_t)(TOTAL + 2)) {
        r.capabilities = (uint16_t)data[TOTAL] | ((uint16_t)data[TOTAL + 1] << 8);
    } else {
        r.capabilities = 0;  // Old peer without caps
    }

    return true;
}

} // namespace iris
