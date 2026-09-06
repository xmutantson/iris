#include "probe/passband_probe.h"
#include "ofdm/ofdm_config.h"
#include "common/fft.h"
#include "common/logging.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// -----------------------------------------------------------------------
// Helper: generate a single chirp into a buffer (used by both TX and RX)
// Linear up-chirp from f0 to f1 over n_chirp samples, Tukey windowed.
// -----------------------------------------------------------------------
static void generate_chirp(float* buf, int n_chirp, int sample_rate,
                           float f0, float f1, float amplitude,
                           int output_samples = -1) {
    if (output_samples < 0) output_samples = n_chirp;
    float T = (float)n_chirp / sample_rate;
    float mu = (f1 - f0) / T;  // chirp rate (Hz/s)
    float fs = (float)sample_rate;

    // Tukey window (alpha = 0.1): cosine taper on first/last 5%
    int ramp = (int)(0.05f * n_chirp);  // alpha/2 = 0.05
    if (ramp < 1) ramp = 1;

    for (int i = 0; i < output_samples; i++) {
        float n = (float)i;
        // Phase: 2*pi*(f0*n/fs + mu*n^2 / (2*fs^2))
        float phase = 2.0f * (float)M_PI * (f0 * n / fs + mu * n * n / (2.0f * fs * fs));
        float sample = std::sin(phase);

        // Tukey envelope
        float env = 1.0f;
        if (i < ramp)
            env = 0.5f * (1.0f - std::cos((float)M_PI * i / ramp));
        else if (i >= n_chirp - ramp)
            env = 0.5f * (1.0f - std::cos((float)M_PI * (n_chirp - 1 - i) / ramp));

        buf[i] = amplitude * sample * env;
    }
}

// -----------------------------------------------------------------------
// Generate chirp probe signal: 2 repetitions with gap
// -----------------------------------------------------------------------

int probe_generate(float* out, int max_samples, int sample_rate,
                   float amplitude) {
    int n_chirp = (int)(PassbandProbeConfig::CHIRP_DURATION_S * sample_rate);
    int n_gap = (int)(PassbandProbeConfig::CHIRP_GAP_S * sample_rate);
    int total = n_chirp * PassbandProbeConfig::CHIRP_REPS +
                n_gap * (PassbandProbeConfig::CHIRP_REPS - 1);
    if (total > max_samples) total = max_samples;

    float f0 = PassbandProbeConfig::CHIRP_F0;
    float f1 = PassbandProbeConfig::CHIRP_F1;

    int pos = 0;
    for (int rep = 0; rep < PassbandProbeConfig::CHIRP_REPS; rep++) {
        int chirp_len = std::min(n_chirp, total - pos);
        if (chirp_len <= 0) break;
        generate_chirp(out + pos, n_chirp, sample_rate, f0, f1, amplitude,
                       chirp_len);
        pos += chirp_len;

        // Gap (silence) between repetitions
        if (rep < PassbandProbeConfig::CHIRP_REPS - 1) {
            int gap_len = std::min(n_gap, total - pos);
            if (gap_len > 0) {
                std::memset(out + pos, 0, gap_len * sizeof(float));
                pos += gap_len;
            }
        }
    }

    return pos;
}

// -----------------------------------------------------------------------
// Helper: next power of 2 >= n
// -----------------------------------------------------------------------
static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

// -----------------------------------------------------------------------
// Locate the first chirp onset via reference cross-correlation.
//
// Mirrors probe_analyze()'s correlation stage (FFT xcorr, peak/median gate,
// forward/backward secondary-peak search) but stops at the onset — it does NOT
// do the per-tone deconvolution. The controller uses it to detect that a full
// probe has already landed so it can END the capture window immediately rather
// than waiting out a fixed timer; probe_analyze() still does the authoritative
// pass. Returns the FIRST chirp start sample (the earliest of the two reps), or
// -1 if the peak/median ratio is below the 20 dB accept gate (same as analyze).
// -----------------------------------------------------------------------
int probe_find_chirp_onset(const float* samples, int n_samples, int sample_rate,
                           float* peak_ratio_db_out) {
    if (peak_ratio_db_out) *peak_ratio_db_out = 0.0f;

    int n_chirp = (int)(PassbandProbeConfig::CHIRP_DURATION_S * sample_rate);
    int n_gap   = (int)(PassbandProbeConfig::CHIRP_GAP_S * sample_rate);
    if (n_samples < n_chirp) return -1;

    std::vector<float> ref_chirp(n_chirp);
    generate_chirp(ref_chirp.data(), n_chirp, sample_rate,
                   PassbandProbeConfig::CHIRP_F0, PassbandProbeConfig::CHIRP_F1, 1.0f);

    int fft_n = next_pow2(n_samples + n_chirp);
    std::vector<float> y_re(fft_n, 0.0f), y_im(fft_n, 0.0f);
    for (int i = 0; i < n_samples; i++) y_re[i] = samples[i];
    iris::fft(y_re.data(), y_im.data(), fft_n);

    std::vector<float> x_re(fft_n, 0.0f), x_im(fft_n, 0.0f);
    for (int i = 0; i < n_chirp; i++) x_re[i] = ref_chirp[i];
    iris::fft(x_re.data(), x_im.data(), fft_n);

    std::vector<float> r_re(fft_n), r_im(fft_n);
    for (int i = 0; i < fft_n; i++) {
        r_re[i] = y_re[i] * x_re[i] + y_im[i] * x_im[i];
        r_im[i] = y_im[i] * x_re[i] - y_re[i] * x_im[i];
    }
    iris::ifft(r_re.data(), r_im.data(), fft_n);

    int max_offset = n_samples - n_chirp;
    if (max_offset < 0) max_offset = 0;
    std::vector<float> corr_mag(max_offset + 1);
    float global_peak = 0.0f;
    int global_peak_idx = 0;
    for (int i = 0; i <= max_offset; i++) {
        float mag = std::sqrt(r_re[i] * r_re[i] + r_im[i] * r_im[i]);
        corr_mag[i] = mag;
        if (mag > global_peak) { global_peak = mag; global_peak_idx = i; }
    }
    std::vector<float> corr_sorted(corr_mag.begin(), corr_mag.end());
    std::sort(corr_sorted.begin(), corr_sorted.end());
    float median_mag = corr_sorted.empty() ? 1e-20f : corr_sorted[corr_sorted.size() / 2];
    if (median_mag < 1e-20f) median_mag = 1e-20f;

    float peak_ratio_db = 20.0f * std::log10(global_peak / median_mag);
    if (peak_ratio_db_out) *peak_ratio_db_out = peak_ratio_db;
    if (peak_ratio_db < 20.0f) return -1;

    // Earliest onset: if a strong secondary peak sits ~expected_sep BEFORE the
    // global peak, the global peak is the second rep and the earlier one is the
    // onset. Otherwise the global peak itself is the first rep.
    int expected_sep = n_chirp + n_gap;
    int onset = global_peak_idx;
    int cand_bwd = global_peak_idx - expected_sep;
    if (cand_bwd >= 0) {
        int search_range = sample_rate / 20;  // 50 ms
        int lo = std::max(0, cand_bwd - search_range);
        int hi = std::min(max_offset, cand_bwd + search_range);
        float bwd_mag = 0.0f; int bwd_idx = -1;
        for (int i = lo; i <= hi; i++)
            if (corr_mag[i] > bwd_mag) { bwd_mag = corr_mag[i]; bwd_idx = i; }
        float threshold_10db = median_mag * std::pow(10.0f, 10.0f / 20.0f);
        if (bwd_idx >= 0 && bwd_mag > threshold_10db) onset = bwd_idx;
    }
    return onset;
}

// -----------------------------------------------------------------------
// Analyze received chirp probe signal
//
// Algorithm:
// 1. Generate reference chirp (same formula as TX)
// 2. FFT cross-correlation to find chirp timing (sub-sample precision)
// 3. Extract H(f) = Y(f) / X(f) with Wiener regularization
// 4. Average H(f) from both chirp repetitions
// 5. Sample |H(f)| at 64 tone frequencies → populate ProbeResult
// -----------------------------------------------------------------------

ProbeResult probe_analyze(const float* samples, int n_samples, int sample_rate) {
    ProbeResult result;

    int n_chirp = (int)(PassbandProbeConfig::CHIRP_DURATION_S * sample_rate);
    int n_gap = (int)(PassbandProbeConfig::CHIRP_GAP_S * sample_rate);

    if (n_samples < n_chirp) {
        IRIS_LOG("[PROBE] Audio too short for chirp analysis (%d < %d)", n_samples, n_chirp);
        return result;
    }

    // Generate reference chirp
    std::vector<float> ref_chirp(n_chirp);
    generate_chirp(ref_chirp.data(), n_chirp, sample_rate,
                   PassbandProbeConfig::CHIRP_F0, PassbandProbeConfig::CHIRP_F1, 1.0f);

    // FFT cross-correlation to find chirp start times.
    // Zero-pad both to next power of 2 >= n_samples + n_chirp (avoid circular aliasing).
    int fft_n = next_pow2(n_samples + n_chirp);

    // FFT the received signal
    std::vector<float> y_re(fft_n, 0.0f), y_im(fft_n, 0.0f);
    for (int i = 0; i < n_samples; i++) y_re[i] = samples[i];
    iris::fft(y_re.data(), y_im.data(), fft_n);

    // FFT the reference chirp
    std::vector<float> x_re(fft_n, 0.0f), x_im(fft_n, 0.0f);
    for (int i = 0; i < n_chirp; i++) x_re[i] = ref_chirp[i];
    iris::fft(x_re.data(), x_im.data(), fft_n);

    // Cross-correlation: R(f) = Y(f) * conj(X(f))
    std::vector<float> r_re(fft_n), r_im(fft_n);
    for (int i = 0; i < fft_n; i++) {
        r_re[i] = y_re[i] * x_re[i] + y_im[i] * x_im[i];
        r_im[i] = y_im[i] * x_re[i] - y_re[i] * x_im[i];
    }
    iris::ifft(r_re.data(), r_im.data(), fft_n);

    // Find correlation peaks — expect up to CHIRP_REPS peaks separated by
    // (n_chirp + n_gap) samples. Search the valid range of offsets.
    int expected_sep = n_chirp + n_gap;
    int max_offset = n_samples - n_chirp;
    if (max_offset < 0) max_offset = 0;

    // Compute |r(tau)| and find the global peak
    std::vector<float> corr_mag(max_offset + 1);
    float global_peak = 0.0f;
    int global_peak_idx = 0;
    for (int i = 0; i <= max_offset; i++) {
        float mag = std::sqrt(r_re[i] * r_re[i] + r_im[i] * r_im[i]);
        corr_mag[i] = mag;
        if (mag > global_peak) {
            global_peak = mag;
            global_peak_idx = i;
        }
    }

    // Compute median of correlation magnitude for peak validation
    std::vector<float> corr_sorted(corr_mag.begin(), corr_mag.end());
    std::sort(corr_sorted.begin(), corr_sorted.end());
    float median_mag = corr_sorted.empty() ? 1e-20f :
                       corr_sorted[corr_sorted.size() / 2];
    if (median_mag < 1e-20f) median_mag = 1e-20f;

    float peak_ratio_db = 20.0f * std::log10(global_peak / median_mag);
    if (global_peak <= 0.0f ||
        (max_offset > 0 && peak_ratio_db < 20.0f)) {
        IRIS_LOG("[PROBE] Chirp correlation peak too weak: %.1f dB (need 20 dB)",
                 peak_ratio_db);
        return result;
    }

    // Collect chirp start positions. First peak is global_peak_idx.
    // Search for second peak near expected_sep offset from the first.
    int chirp_starts[2] = { global_peak_idx, -1 };
    int n_found = 1;

    if (PassbandProbeConfig::CHIRP_REPS >= 2) {
        // Determine if the global peak is the first or second chirp.
        // Check both possibilities: peak is chirp 0, or peak is chirp 1.
        int cand_fwd = global_peak_idx + expected_sep;  // next chirp after global peak
        int cand_bwd = global_peak_idx - expected_sep;  // chirp before global peak

        float fwd_mag = 0.0f, bwd_mag = 0.0f;
        int search_range = sample_rate / 20;  // 50ms search window

        // Search forward
        if (cand_fwd >= 0 && cand_fwd <= max_offset) {
            int lo = std::max(0, cand_fwd - search_range);
            int hi = std::min(max_offset, cand_fwd + search_range);
            for (int i = lo; i <= hi; i++) {
                if (corr_mag[i] > fwd_mag) {
                    fwd_mag = corr_mag[i];
                    chirp_starts[1] = i;
                }
            }
        }
        // Search backward
        int bwd_idx = -1;
        if (cand_bwd >= 0 && cand_bwd <= max_offset) {
            int lo = std::max(0, cand_bwd - search_range);
            int hi = std::min(max_offset, cand_bwd + search_range);
            for (int i = lo; i <= hi; i++) {
                if (corr_mag[i] > bwd_mag) {
                    bwd_mag = corr_mag[i];
                    bwd_idx = i;
                }
            }
        }

        // Pick the stronger secondary peak; require it to be at least 10 dB above median
        float threshold_10db = median_mag * std::pow(10.0f, 10.0f / 20.0f);
        if (bwd_mag > fwd_mag && bwd_mag > threshold_10db) {
            // Global peak is the second chirp; backward peak is the first
            chirp_starts[0] = bwd_idx;
            chirp_starts[1] = global_peak_idx;
            n_found = 2;
        } else if (fwd_mag > threshold_10db) {
            // Global peak is the first chirp; forward peak is the second
            chirp_starts[0] = global_peak_idx;
            // chirp_starts[1] already set in forward search
            n_found = 2;
        }
    }

    IRIS_LOG("[PROBE] Chirp correlation: %d peaks found, peak/median %.1f dB, offsets [%d, %d]",
             n_found, peak_ratio_db, chirp_starts[0],
             n_found >= 2 ? chirp_starts[1] : -1);

    // Extract H(f) from each found chirp via frequency-domain deconvolution.
    // H(f) = Y_aligned(f) * conj(X(f)) / (|X(f)|^2 + epsilon)
    //
    // Use an FFT of size n_chirp (or next power of 2) for the deconvolution
    // so we get the response at the chirp's resolution.
    int deconv_n = next_pow2(n_chirp);
    int deconv_pos = deconv_n / 2;

    // Reference chirp FFT (for deconvolution, may differ in size from correlation FFT)
    std::vector<float> xd_re(deconv_n, 0.0f), xd_im(deconv_n, 0.0f);
    for (int i = 0; i < n_chirp; i++) xd_re[i] = ref_chirp[i];
    iris::fft(xd_re.data(), xd_im.data(), deconv_n);

    // Compute |X(f)|^2 and find max for epsilon
    std::vector<float> x_mag2(deconv_n);
    float max_x_mag2 = 0.0f;
    for (int i = 0; i < deconv_n; i++) {
        x_mag2[i] = xd_re[i] * xd_re[i] + xd_im[i] * xd_im[i];
        if (x_mag2[i] > max_x_mag2) max_x_mag2 = x_mag2[i];
    }
    float epsilon = max_x_mag2 * 0.01f;  // -20 dB below peak (Wiener regularization)

    // Accumulate H(f) magnitude across chirp repetitions
    std::vector<float> h_mag_sum(deconv_pos, 0.0f);
    int n_averaged = 0;

    for (int c = 0; c < n_found; c++) {
        int start = chirp_starts[c];
        if (start < 0 || start + n_chirp > n_samples) continue;

        // FFT the aligned received chirp segment
        std::vector<float> yd_re(deconv_n, 0.0f), yd_im(deconv_n, 0.0f);
        for (int i = 0; i < n_chirp; i++) yd_re[i] = samples[start + i];
        iris::fft(yd_re.data(), yd_im.data(), deconv_n);

        // H(f) = Y(f) * conj(X(f)) / (|X(f)|^2 + epsilon)
        for (int i = 0; i < deconv_pos; i++) {
            float num_re = yd_re[i] * xd_re[i] + yd_im[i] * xd_im[i];
            float num_im = yd_im[i] * xd_re[i] - yd_re[i] * xd_im[i];
            float denom = x_mag2[i] + epsilon;
            float h_re = num_re / denom;
            float h_im = num_im / denom;
            h_mag_sum[i] += std::sqrt(h_re * h_re + h_im * h_im);
        }
        n_averaged++;
    }

    if (n_averaged == 0) {
        IRIS_LOG("[PROBE] No valid chirp segments for deconvolution");
        return result;
    }

    // Average and convert to dB
    float bin_hz = (float)sample_rate / deconv_n;
    std::vector<float> h_mag_db(deconv_pos);
    for (int i = 0; i < deconv_pos; i++) {
        float mag = h_mag_sum[i] / n_averaged;
        h_mag_db[i] = 20.0f * std::log10(mag + 1e-20f);
    }

    // (Detection noise floor is estimated AFTER tone sampling, below, from the
    //  REAL in-band tone response — see the ROOT-CAUSE FIX note there.)

    // Sample |H(f)| at the 64 tone frequencies
    float peak_tone_power = -100.0f;
    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float freq = probe_tone_freq(k);
        int bin = (int)(freq / bin_hz + 0.5f);
        if (bin < 0 || bin >= deconv_pos) {
            result.tone_power_db[k] = -100.0f;
            continue;
        }

        // Peak in +-1 bin neighborhood (handles slight frequency offset)
        float peak = h_mag_db[bin];
        if (bin > 0) peak = std::max(peak, h_mag_db[bin - 1]);
        if (bin < deconv_pos - 1) peak = std::max(peak, h_mag_db[bin + 1]);

        result.tone_power_db[k] = peak;
        if (peak > peak_tone_power) peak_tone_power = peak;
    }

    // ------------------------------------------------------------------
    // Detection noise floor — estimated from the REAL in-band channel response.
    //
    // ROOT-CAUSE FIX (was: median of out-of-CHIRP-band bins <150 / >4700 Hz):
    // the reference chirp carries ~zero energy outside [200,4600] Hz, so the
    // deconvolution H(f)=Y*conjX/(|X|^2+eps) there is a numerical artifact
    // (~-170..-210 dB from the eps-regularized 0/0), NOT the channel's noise
    // floor. That artifact put threshold_floor ~100+ dB below every in-band
    // tone, so the OR-branch below re-detected all 64 tones on ANY channel —
    // probe_negotiate returned the full ~4150 Hz band even through a brick-wall
    // bandpass (HW-proven: bandpass on/off/brick-wall -> always 4150 Hz),
    // making the beat-VARA comparison bandwidth-unfair.
    //
    // The honest floor is the level of the tones the channel did NOT pass: in a
    // band-limited channel a stopped tone has Y=noise, so |H|=noise/|X| sits at
    // the true in-band noise floor. A LOW PERCENTILE of the 64 in-band per-tone
    // powers estimates it robustly (a percentile, not the min, is immune to a
    // single deep-null bin). On a genuinely WIDE channel there is no stopband,
    // the percentile rides up near the peak and this floor-branch harmlessly
    // collapses — the peak-relative branch still admits the full band, so
    // wide-channel discovery is preserved.
    float noise_floor;
    {
        std::vector<float> tone_pows;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++)
            if (result.tone_power_db[k] > -100.0f)  // skip out-of-deconv-range sentinel
                tone_pows.push_back(result.tone_power_db[k]);
        if ((int)tone_pows.size() >= 5) {
            std::sort(tone_pows.begin(), tone_pows.end());
            // 20th percentile: below the ~1/3-of-band stopband of a narrow-FM
            // audio filter (300-2900 Hz over the 300-4500 Hz tone grid), above
            // single-bin nulls. On a wide/flat channel it lands near the peak.
            int idx = (int)(0.20f * (tone_pows.size() - 1) + 0.5f);
            noise_floor = tone_pows[idx];
        } else {
            // Degenerate capture — collapse the floor-branch onto the
            // peak-relative branch (floor+margin == peak-DETECT_THRESHOLD).
            noise_floor = peak_tone_power - PassbandProbeConfig::DETECT_THRESHOLD_DB
                                          - PassbandProbeConfig::NOISE_FLOOR_MARGIN_DB;
        }
    }

    // Dual-threshold detection (same as old tone probe):
    // Primary: within DETECT_THRESHOLD_DB of peak
    // Secondary: above noise_floor + NOISE_FLOOR_MARGIN_DB
    int first_detected = -1, last_detected = -1;
    float threshold_peak = peak_tone_power - PassbandProbeConfig::DETECT_THRESHOLD_DB;
    float threshold_floor = noise_floor + PassbandProbeConfig::NOISE_FLOOR_MARGIN_DB;

    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float freq = probe_tone_freq(k);
        int bin = (int)(freq / bin_hz + 0.5f);
        if (bin < 0 || bin >= deconv_pos) continue;
        result.tone_detected[k] = (result.tone_power_db[k] >= threshold_peak ||
                                   result.tone_power_db[k] >= threshold_floor);
        if (result.tone_detected[k]) {
            if (first_detected < 0) first_detected = k;
            last_detected = k;
            result.tones_detected++;
        }
    }

    if (first_detected >= 0 && last_detected >= 0) {
        result.low_hz = probe_tone_freq(first_detected);
        result.high_hz = probe_tone_freq(last_detected);
        result.valid = (result.tones_detected >= 3);
    }

    // Link-SNR estimate (LOCAL-only): median detected-tone power above the
    // out-of-band noise floor.  Physically meaningful — the tones traversed the
    // real channel, so (median tone power − noise floor) is the SNR the OFDM PHY
    // will see.  Consumed by the connect-diet (skip the auto-tune / seed the
    // climb on a clean high-SNR probe).  Median (not mean) is robust to a couple
    // of band-edge tones sitting near the floor.
    {
        std::vector<float> det_db;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++)
            if (result.tone_detected[k]) det_db.push_back(result.tone_power_db[k]);
        if (!det_db.empty()) {
            std::sort(det_db.begin(), det_db.end());
            result.est_snr_db = det_db[det_db.size() / 2] - noise_floor;
        }
    }

    // No comb validation needed for chirp — H(f) is continuous, not discrete tones.

    return result;
}

// -----------------------------------------------------------------------
// Measure the channel's pre-emphasis corner frequency from probe data.
//
// With chirp probe, H(f) directly measures the channel frequency response
// (no TX pre-emphasis shaping to subtract — chirp has flat spectral density).
//
// The response reveals the path characteristic:
//   - Flat (cable/data port): no systematic rolloff → corner = 0
//   - FM mic/speaker: rolloff matching 1/sqrt(1+(f/fc)^2) → fit fc
//
// Method: fit the OFDM de-emphasis model g(f) = -10*log10(1 + (f/fc)^2)
// to the measured dB response using golden-section search over candidate fc.
// -----------------------------------------------------------------------
float probe_detect_preemph_corner(const ProbeResult& probe) {
    // Step 1: Collect detected tones. Chirp H(f) is the true channel response —
    // no TX pre-emphasis to subtract (chirp has flat spectral density).
    struct ToneMeas { float freq; float ch_db; };
    std::vector<ToneMeas> tones;

    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        if (!probe.tone_detected[k]) continue;
        float freq = probe_tone_freq(k);
        float ch_db = probe.tone_power_db[k];
        tones.push_back({freq, ch_db});
    }

    if ((int)tones.size() < 5) {
        IRIS_LOG("[PROBE-SHAPE] only %d tones — preemph disabled",
                 (int)tones.size());
        return 0.0f;
    }

    // Normalize: subtract mean so we fit the shape, not the absolute level.
    float mean_db = 0;
    for (auto& t : tones) mean_db += t.ch_db;
    mean_db /= tones.size();
    for (auto& t : tones) t.ch_db -= mean_db;

    // Step 2: Check if the channel is flat. If peak-to-peak < 4 dB,
    // the path has no significant frequency-dependent rolloff.
    float min_db = 1e9f, max_db = -1e9f;
    for (auto& t : tones) {
        if (t.ch_db < min_db) min_db = t.ch_db;
        if (t.ch_db > max_db) max_db = t.ch_db;
    }
    float range_db = max_db - min_db;

    if (range_db < 4.0f) {
        IRIS_LOG("[PROBE-SHAPE] flat channel (range %.1f dB, %d tones) → preemph disabled",
                 range_db, (int)tones.size());
        return 0.0f;
    }

    // Step 3: Fit fc by minimizing sum-of-squared residuals between
    // the corrected channel response and the model:
    //   model_db(f, fc) = -10*log10(1 + (f/fc)^2) + offset
    // where offset is chosen to minimize residuals (= model's mean matches data mean = 0).
    // Search fc from 100 Hz to 3000 Hz using golden-section.
    auto fit_error = [&](float fc) -> float {
        // Compute model values and their mean
        float model_mean = 0;
        for (auto& t : tones) {
            float model = -10.0f * std::log10(1.0f + (t.freq / fc) * (t.freq / fc));
            model_mean += model;
        }
        model_mean /= tones.size();

        // Sum of squared residuals (model shifted to zero mean)
        float sse = 0;
        for (auto& t : tones) {
            float model = -10.0f * std::log10(1.0f + (t.freq / fc) * (t.freq / fc));
            float residual = t.ch_db - (model - model_mean);
            sse += residual * residual;
        }
        return sse;
    };

    // Golden-section search for minimum SSE
    float lo = 100.0f, hi = 3000.0f;
    constexpr float PHI = 0.6180339887f;
    for (int iter = 0; iter < 30; iter++) {
        float x1 = hi - PHI * (hi - lo);
        float x2 = lo + PHI * (hi - lo);
        if (fit_error(x1) < fit_error(x2))
            hi = x2;
        else
            lo = x1;
    }
    float best_fc = (lo + hi) / 2.0f;
    float best_sse = fit_error(best_fc);
    float rmse = std::sqrt(best_sse / tones.size());

    // Also compute SSE for the flat model (fc → ∞, model = 0 everywhere)
    float flat_sse = 0;
    for (auto& t : tones) flat_sse += t.ch_db * t.ch_db;
    float flat_rmse = std::sqrt(flat_sse / tones.size());

    IRIS_LOG("[PROBE-SHAPE] %d tones, range %.1f dB: fit fc=%.0f Hz (rmse=%.1f dB), flat rmse=%.1f dB",
             (int)tones.size(), range_db, best_fc, rmse, flat_rmse);

    // Decision: use the fitted corner if it explains the data significantly
    // better than the flat model AND the corner is in a physically reasonable range.
    if (rmse < flat_rmse * 0.7f && best_fc >= 100.0f && best_fc <= 2000.0f) {
        IRIS_LOG("[PROBE-SHAPE] FM channel detected → preemph corner %.0f Hz", best_fc);
        return best_fc;
    } else {
        IRIS_LOG("[PROBE-SHAPE] flat or poor fit → preemph disabled (fc=%.0f, rmse=%.1f vs flat=%.1f)",
                 best_fc, rmse, flat_rmse);
        return 0.0f;
    }
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

NegotiatedPassband probe_negotiate_grid(const ProbeResult& my_tx_their_rx,
                                        const ProbeResult& their_tx_my_rx,
                                        bool is_initiator) {
    NegotiatedPassband neg;
    neg.my_tx_their_rx = my_tx_their_rx;
    neg.their_tx_my_rx = their_tx_my_rx;

    // CMD->RSP = the initiator's probe as measured by the responder — the single
    // authoritative band both ends reliably hold (see the header for the full
    // rationale).  On the initiator it arrives via the responder's robustly
    // re-announced RESULT (my_tx_their_rx); on the responder it is the local
    // measurement of the initiator's probe (their_tx_my_rx).
    const ProbeResult& authoritative = is_initiator ? my_tx_their_rx : their_tx_my_rx;

    if (!authoritative.valid) {
        // Authoritative CMD->RSP not available yet (e.g. the initiator's
        // WAITING_RESULT timed out before the responder's RESULT arrived).
        // Fall back to the legacy intersection so the session can still
        // activate; the config-fingerprint guard remains the split-brain net.
        return probe_negotiate(my_tx_their_rx, their_tx_my_rx);
    }

    float low = authoritative.low_hz + PassbandProbeConfig::EDGE_MARGIN_HZ;
    float high = authoritative.high_hz - PassbandProbeConfig::EDGE_MARGIN_HZ;
    if (high <= low) return neg;  // No usable band

    neg.low_hz = low;
    neg.high_hz = high;
    neg.center_hz = (low + high) / 2.0f;
    neg.bandwidth_hz = high - low;
    neg.valid = true;
    return neg;
}

NegotiatedPassband apply_grid_pin(const NegotiatedPassband& neg) {
    // Default-inert: env unset/0 -> the negotiation result passes through
    // byte-identical.  See the header note for the rationale + both-ends
    // requirement.  Never pins an INVALID negotiation (a failed probe stays
    // failed; the pin re-grids a working session, it does not rescue one).
    const char* e = std::getenv("IRIS_GRID_PIN_NARROW");
    if (!(e && std::atoi(e) != 0) || !neg.valid) return neg;
    NegotiatedPassband pinned = neg;      // keep my_tx/their_tx EQ metadata
    NegotiatedPassband nb = narrow_passband();   // 300-3000 Hz, 57 used carriers
    pinned.low_hz = nb.low_hz;
    pinned.high_hz = nb.high_hz;
    pinned.center_hz = nb.center_hz;
    pinned.bandwidth_hz = nb.bandwidth_hz;
    IRIS_LOG("[GRID-PIN] narrow %.0f-%.0f Hz pinned (probe measured %.0f-%.0f Hz)",
             pinned.low_hz, pinned.high_hz, neg.low_hz, neg.high_hz);
    return pinned;
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

    // Per-tone power levels (64 bytes, quantized to 0.5 dB resolution)
    // Enables channel equalization on the receiving side.
    // Encoding: uint8_t val = clamp((power_db + 80) * 2, 0, 255)
    // Range: -80 dB to +47.5 dB in 0.5 dB steps
    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
        float val = (r.tone_power_db[k] + 80.0f) * 2.0f;
        out.push_back((uint8_t)std::max(0.0f, std::min(255.0f, val)));
    }

    // OFDM PHY config (4 bytes, v4 extension)
    // Allows peers to negotiate CP, pilot spacing, etc.
    // Old peers ignore these extra bytes; we detect old peers by len < v4 size.
    out.push_back(r.ofdm_cp_samples);
    out.push_back(r.ofdm_pilot_carrier_spacing);
    out.push_back(r.ofdm_pilot_symbol_spacing);
    out.push_back(r.ofdm_nfft_code);

    return out;
}

bool probe_result_decode(const uint8_t* data, size_t len, ProbeResult& r) {
    constexpr int BITMAP_BYTES = (PassbandProbeConfig::N_TONES + 7) / 8;
    constexpr int TOTAL = 1 + 4 + 4 + 2 + BITMAP_BYTES;
    if (len < (size_t)TOTAL || data[0] != 0xBC) return false;

    r.low_hz = get_f32(data + 1);
    r.high_hz = get_f32(data + 5);
    r.tones_detected = (int)data[9] | ((int)data[10] << 8);

    int bitmap_count = 0;
    int first_detected = -1;
    int last_detected = -1;
    for (int byte = 0; byte < BITMAP_BYTES; byte++) {
        uint8_t b = data[11 + byte];
        for (int bit = 0; bit < 8; bit++) {
            int idx = byte * 8 + bit;
            if (idx < PassbandProbeConfig::N_TONES) {
                r.tone_detected[idx] = (b >> bit) & 1;
                if (r.tone_detected[idx]) {
                    if (first_detected < 0) first_detected = idx;
                    last_detected = idx;
                    bitmap_count++;
                }
            }
        }
    }

    r.valid = (r.tones_detected >= 3 &&
               r.tones_detected == bitmap_count &&
               std::isfinite(r.low_hz) && std::isfinite(r.high_hz) &&
               r.low_hz >= PassbandProbeConfig::TONE_LOW_HZ &&
               r.high_hz <= PassbandProbeConfig::TONE_HIGH_HZ &&
               r.high_hz > r.low_hz &&
               r.low_hz == probe_tone_freq(first_detected) &&
               r.high_hz == probe_tone_freq(last_detected));

    // Capability flags (optional, appended by v2+ peers)
    if (len >= (size_t)(TOTAL + 2)) {
        r.capabilities = (uint16_t)data[TOTAL] | ((uint16_t)data[TOTAL + 1] << 8);
    } else {
        r.capabilities = 0;  // Old peer without caps
    }

    // Per-tone power levels (optional, appended by v3+ peers with EQ support)
    // 64 bytes after caps (TOTAL + 2 + 64)
    constexpr int TONE_POWER_OFFSET = TOTAL + 2;
    if (len >= (size_t)(TONE_POWER_OFFSET + PassbandProbeConfig::N_TONES)) {
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            r.tone_power_db[k] = (float)data[TONE_POWER_OFFSET + k] / 2.0f - 80.0f;
        }
    }
    // If old peer without tone powers, tone_power_db stays at 0 (no EQ applied)

    // OFDM PHY config (optional, appended by v4+ peers)
    // 4 bytes after tone powers: cp, pilot_carrier_spacing, pilot_symbol_spacing, nfft_code
    constexpr int OFDM_CFG_OFFSET = TONE_POWER_OFFSET + PassbandProbeConfig::N_TONES;
    if (len >= (size_t)(OFDM_CFG_OFFSET + 4)) {
        r.ofdm_cp_samples = data[OFDM_CFG_OFFSET];
        r.ofdm_pilot_carrier_spacing = data[OFDM_CFG_OFFSET + 1];
        r.ofdm_pilot_symbol_spacing = data[OFDM_CFG_OFFSET + 2];
        r.ofdm_nfft_code = data[OFDM_CFG_OFFSET + 3];
    }
    // If old peer: ofdm_* fields stay at 0 (use defaults)

    return true;
}

} // namespace iris
