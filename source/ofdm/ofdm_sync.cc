#include "ofdm/ofdm_sync.h"
#include "common/fft.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// ---------------------------------------------------------------------------
// Zadoff-Chu sequence generation
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> generate_zc_sequence(int root, int length)
{
    std::vector<std::complex<float>> seq(length);
    for (int n = 0; n < length; ++n) {
        // x[n] = exp(-j * pi * root * n * (n+1) / length)
        float phase = -(float)M_PI * root * n * (n + 1) / (float)length;
        seq[n] = std::complex<float>(std::cos(phase), std::sin(phase));
    }
    return seq;
}

// ---------------------------------------------------------------------------
// Generate time-domain ZC training symbol (nfft samples, no CP)
// Applies Hermitian symmetry so the IFFT output is REAL-VALUED.
// This is essential because OFDM audio is real — without Hermitian symmetry,
// only the real part of a complex ZC symbol survives through audio, halving
// the effective power at each used carrier and destroying detection/channel est.
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> generate_zc_training_symbol(const OfdmConfig& config,
                                                              int root)
{
    const int nfft = config.nfft;
    const int n_used = config.n_used_carriers;

    // Generate ZC sequence in frequency domain for used carriers
    auto zc_freq = generate_zc_sequence(root, n_used);

    // Place into FFT bins WITH Hermitian symmetry: X[N-k] = conj(X[k])
    // Used carrier bins are all in the positive-frequency half (< nfft/2),
    // so mirrors land in the upper half with no overlap.
    // TX de-emphasis: attenuate higher carriers so radio pre-emphasis
    // produces a flat signal entering the deviation limiter.  Same curve as
    // data symbols and pilots — ensures channel estimate H[k] matches.
    std::vector<std::complex<float>> X(nfft, std::complex<float>(0.0f, 0.0f));
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        // FM TX de-emphasis gain (real scalar, doesn't break Hermitian symmetry)
        float g = 1.0f;
        if (config.fm_preemph_corner_hz > 0.0f) {
            float fhz = (float)bin * (float)config.sample_rate / (float)nfft;
            g = 1.0f / std::sqrt(1.0f + (fhz / config.fm_preemph_corner_hz)
                                       * (fhz / config.fm_preemph_corner_hz));
            if (g < 1.0f / config.fm_preemph_gain_cap)
                g = 1.0f / config.fm_preemph_gain_cap;
        }
        X[bin] = zc_freq[i] * g;
        // Hermitian mirror for real-valued IFFT output
        if (bin > 0 && bin < nfft / 2) {
            X[nfft - bin] = std::conj(zc_freq[i]) * g;
        }
    }

    // IFFT to time domain — output is real-valued due to Hermitian symmetry
    ifft_complex(X.data(), nfft);

    // Scale by nfft to match data symbol amplitude from symbol_to_time().
    // ifft_complex divides by N, so ×N recovers unit-power-per-carrier scaling.
    // This keeps preamble and data at similar amplitudes (preamble also gets
    // PREAMBLE_BOOST in build_ofdm_frame for detection headroom).
    float scale = (float)nfft;
    for (int n = 0; n < nfft; ++n) {
        X[n] *= scale;
    }

    return X;
}

// ---------------------------------------------------------------------------
// Hybrid Schmidl-Cox + ZC frame detection
// ---------------------------------------------------------------------------
// Schmidl-Cox autocorrelation for DETECTION (channel-invariant):
//   Correlates train1 body with train2 body (identical ZC sequences).
//   Both pass through the same channel, so distortion cancels out.
//   Works regardless of radio frequency response differences.
// ZC cross-correlation for TIMING REFINEMENT (sharp peak):
//   After SC detection, ZC xcorr pinpoints exact frame start within ±cp.
// ---------------------------------------------------------------------------

// SC detection threshold.  SC metric for identical training symbols:
//   M ≈ (SNR/(SNR+1))² → ~0.82 at 10 dB, ~0.64 at 5 dB, ~0.25 at 0 dB.
// Noise-only: M ≈ 1/nfft ≈ 0.002.  Data symbols: ~0.01-0.05 (no repetition).
// SC is a COARSE pre-filter — rejects obvious silence/noise.
// FD-ZC (0.50) is the secondary gate.  The sync word symbol after the
// preamble is the real discriminator (deterministic bit-check, no threshold).
// Threshold 0.25 passes signals down to ~0 dB SNR.  False triggers that
// pass SC+FD-ZC are caught by the sync word check (random bits → ~50% BER).
static constexpr float SC_DETECTION_THRESHOLD = 0.25f;  // coarse pre-filter only

// Helper: next power of 2 >= n
static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

OfdmSyncResult ofdm_detect_frame(const std::complex<float>* iq, int n_samples,
                                  const OfdmConfig& config)
{
    OfdmSyncResult result;
    const int nfft = config.nfft;
    const int cp = config.cp_samples;
    const int symbol_len = nfft + cp;

    // Need at least two training symbols for SC detection + CFO
    const int min_samples = 2 * symbol_len;
    if (n_samples < min_samples) {
        IRIS_LOG("[OFDM-SYNC] insufficient samples for detection: %d < %d",
                 n_samples, min_samples);
        return result;
    }

    // -----------------------------------------------------------------------
    // Cache ZC training symbol for timing refinement
    // -----------------------------------------------------------------------
    static std::vector<std::complex<float>> zc_td;
    static float zc_energy = 0.0f;
    static int cached_nfft = 0;
    static int cached_n_used = 0;

    if (cached_nfft != nfft || cached_n_used != config.n_used_carriers) {
        zc_td = generate_zc_training_symbol(config);
        zc_energy = 0.0f;
        for (int n = 0; n < nfft; ++n) {
            zc_energy += std::norm(zc_td[n]);
        }
        cached_nfft = nfft;
        cached_n_used = config.n_used_carriers;
        IRIS_LOG("[OFDM-SYNC] ZC training symbol cached: nfft=%d, n_used=%d, energy=%.1f",
                 nfft, config.n_used_carriers, zc_energy);
    }

    // Search range: d is CP start of train1, need d + 2*symbol_len <= n_samples
    const int search_len = n_samples - 2 * symbol_len;
    if (search_len <= 0) {
        IRIS_LOG("[OFDM-SYNC] search range empty");
        return result;
    }

    // -----------------------------------------------------------------------
    // Phase 1: Schmidl-Cox autocorrelation — coarse search at stride cp
    // -----------------------------------------------------------------------
    // Correlate train1 body [d+cp, d+cp+nfft) with train2 body [d+cp+sym_len, ...)
    // M(d) = |P(d)|² / (A(d) · R(d))   [Cauchy-Schwarz normalized, 0..1]
    // Energy weighting: W(d) = M(d) · (A(d) + R(d))  [rejects silence]

    float sc_best_metric = 0.0f;
    float sc_best_weighted = 0.0f;
    int sc_best_d = -1;

    const int coarse_stride = std::max(1, cp);  // stride = CP width (32 samples typical)

    for (int d = 0; d < search_len; d += coarse_stride) {
        const int body1 = d + cp;
        const int body2 = d + symbol_len + cp;

        std::complex<float> P(0.0f, 0.0f);
        float A = 0.0f;
        float R = 0.0f;

        for (int n = 0; n < nfft; ++n) {
            P += std::conj(iq[body1 + n]) * iq[body2 + n];
            A += std::norm(iq[body1 + n]);
            R += std::norm(iq[body2 + n]);
        }

        float denom = A * R;
        float M = (denom > 1e-20f) ? (std::norm(P) / denom) : 0.0f;
        float W = M * (A + R);  // energy-weighted metric (rejects silence)

        if (W > sc_best_weighted) {
            sc_best_weighted = W;
            sc_best_metric = M;
            sc_best_d = d;
        }
    }

    // -----------------------------------------------------------------------
    // Phase 2: Schmidl-Cox fine search at stride 1 around coarse peak
    // -----------------------------------------------------------------------
    if (sc_best_d >= 0 && sc_best_metric >= SC_DETECTION_THRESHOLD * 0.5f) {
        int fine_lo = std::max(0, sc_best_d - coarse_stride);
        int fine_hi = std::min(search_len - 1, sc_best_d + coarse_stride);

        for (int d = fine_lo; d <= fine_hi; ++d) {
            const int body1 = d + cp;
            const int body2 = d + symbol_len + cp;

            std::complex<float> P(0.0f, 0.0f);
            float A = 0.0f;
            float R = 0.0f;

            for (int n = 0; n < nfft; ++n) {
                P += std::conj(iq[body1 + n]) * iq[body2 + n];
                A += std::norm(iq[body1 + n]);
                R += std::norm(iq[body2 + n]);
            }

            float denom = A * R;
            float M = (denom > 1e-20f) ? (std::norm(P) / denom) : 0.0f;
            float W = M * (A + R);

            if (W > sc_best_weighted) {
                sc_best_weighted = W;
                sc_best_metric = M;
                sc_best_d = d;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Threshold check
    // -----------------------------------------------------------------------
    float peak_metric = std::sqrt(sc_best_metric);  // sqrt for [0,1] range comparable to old ZC metric
    int peak_d = sc_best_d;

    if (peak_d < 0 || sc_best_metric < SC_DETECTION_THRESHOLD) {
        static int no_detect_count = 0;
        no_detect_count++;
        if ((no_detect_count % 200) == 1) {
            IRIS_LOG("[OFDM-SYNC] no detection: peak_sc=%.3f (sqrt=%.3f) at d=%d/%d (threshold=%.2f, %d samples)",
                     sc_best_metric, peak_metric, peak_d, search_len,
                     SC_DETECTION_THRESHOLD, n_samples);
        }
        return result;
    }

    // -----------------------------------------------------------------------
    // Phase 3: ZC cross-correlation for precise timing within ±cp of SC peak
    // -----------------------------------------------------------------------
    // SC has a plateau of width cp (due to cyclic prefix periodicity).
    // ZC xcorr has a sharp peak at the exact body start.
    // Time-domain ZC is used ONLY for timing refinement, not detection gating.
    float best_zc_td = 0.0f;
    {
        int best_zc_d = peak_d;

        int refine_lo = std::max(0, peak_d - cp);
        int refine_hi = std::min(search_len - 1, peak_d + cp);

        for (int d = refine_lo; d <= refine_hi; ++d) {
            int body = d + cp;
            std::complex<float> xc(0.0f, 0.0f);
            float se = 0.0f;
            for (int n = 0; n < nfft; ++n) {
                xc += iq[body + n] * std::conj(zc_td[n]);
                se += std::norm(iq[body + n]);
            }
            float denom = se * zc_energy;
            float m = (denom > 1e-20f) ? std::sqrt(std::norm(xc) / denom) : 0.0f;
            if (m > best_zc_td) {
                best_zc_td = m;
                best_zc_d = d;
            }
        }

        // Use ZC-refined timing if it found a reasonable peak
        if (best_zc_td > 0.1f) {
            peak_d = best_zc_d;
            IRIS_LOG("[OFDM-SYNC] ZC timing refine: d=%d (ZC_td=%.3f, SC=%.3f)",
                     peak_d, best_zc_td, peak_metric);
        }
    }

    // -----------------------------------------------------------------------
    // Phase 4: Frequency-domain ZC verification + CFO estimation
    // -----------------------------------------------------------------------
    // Time-domain ZC cross-correlation has a fundamental flaw on FM channels:
    // the reference zc_td[] has TX de-emphasis gains baked in (g drops from
    // 0.62 at 375 Hz to 0.10 at 3000 Hz).  On flat 9600-baud ports (no RX
    // de-emphasis), the received ZC is ~flat → spectral mismatch gives a
    // theoretical Pearson ceiling of only ~0.86.  FM deviation limiter
    // nonlinearity degrades it further to 0.13-0.35 OTA.
    //
    // Frequency-domain ZC correlation fixes this:
    //   1. Uses the RAW ZC sequence (no de-emphasis) — |ZC[k]|=1 for all k
    //   2. Correlates only at carrier bins — FM intermod on other bins ignored
    //   3. Cauchy-Schwarz normalization handles any spectral tilt
    //   4. Phase relationships survive FM better than time-domain waveform shape
    //
    // This metric is independent of whether the RX port has de-emphasis or not.
    float best_zc_metric = 0.0f;

    result.frame_start = peak_d;
    result.schmidl_metric = peak_metric;

    {
        int train1_body = peak_d + cp;
        int train2_body = peak_d + symbol_len + cp;

        if (train2_body + nfft <= n_samples) {
            // FFT both training symbols (reused for CFO estimation below)
            static std::vector<std::complex<float>> Y1_buf, Y2_buf;
            if ((int)Y1_buf.size() != nfft) {
                Y1_buf.resize(nfft);
                Y2_buf.resize(nfft);
            }
            std::copy(iq + train1_body, iq + train1_body + nfft, Y1_buf.data());
            std::copy(iq + train2_body, iq + train2_body + nfft, Y2_buf.data());
            fft_complex(Y1_buf.data(), nfft);
            fft_complex(Y2_buf.data(), nfft);

            // --- Frequency-domain ZC quality metric ---
            // Correlate received spectrum with de-emphasized ZC — what was
            // actually transmitted.  Previous code used raw ZC (|ZC[k]|=1) but
            // TX applies de-emphasis gains g[k] (0.62→0.10 across band).  The
            // Cauchy-Schwarz metric requires Y[k] ∝ ref[k] at every carrier;
            // without g[k] in the reference, the 6:1 magnitude variation from
            // de-emphasis collapses the metric to 0.12-0.35 on flat-port radios.
            static std::vector<std::complex<float>> zc_freq_cache;
            static float zc_ref_energy_cache = 0.0f;
            static int cached_zc_n_used = 0;
            static float cached_corner_hz = 0.0f;
            const int n_used = config.n_used_carriers;
            if (cached_zc_n_used != n_used ||
                cached_corner_hz != config.fm_preemph_corner_hz) {
                auto zc_raw = generate_zc_sequence(7, n_used);
                zc_freq_cache.resize(n_used);
                zc_ref_energy_cache = 0.0f;
                for (int i = 0; i < n_used; ++i) {
                    int bin = config.used_carrier_bins[i];
                    float g = 1.0f;
                    if (config.fm_preemph_corner_hz > 0.0f) {
                        float fhz = (float)bin * (float)config.sample_rate / (float)config.nfft;
                        g = 1.0f / std::sqrt(1.0f + (fhz / config.fm_preemph_corner_hz)
                                                   * (fhz / config.fm_preemph_corner_hz));
                        if (g < 1.0f / config.fm_preemph_gain_cap)
                            g = 1.0f / config.fm_preemph_gain_cap;
                    }
                    zc_freq_cache[i] = zc_raw[i] * g;
                    zc_ref_energy_cache += g * g;  // |ZC[k]|²=1, so |ref[k]|²=g²
                }
                cached_zc_n_used = n_used;
                cached_corner_hz = config.fm_preemph_corner_hz;
            }

            std::complex<float> fd_corr(0.0f, 0.0f);
            float y_energy = 0.0f;
            for (int i = 0; i < n_used; ++i) {
                int bin = config.used_carrier_bins[i];
                // Use Y1 for the FD-ZC metric (Y2 has CFO phase rotation)
                fd_corr += Y1_buf[bin] * std::conj(zc_freq_cache[i]);
                y_energy += std::norm(Y1_buf[bin]);
            }
            float zc_ref_energy = zc_ref_energy_cache;
            float fd_denom = y_energy * zc_ref_energy;
            best_zc_metric = (fd_denom > 1e-20f)
                ? std::sqrt(std::norm(fd_corr) / fd_denom) : 0.0f;

            // --- ZC quality gate ---
            // OTA 2026-03-22: real frames FD-ZC 0.87-0.96, false detections
            // 0.41-0.60 (originally thought real frames went as low as 0.43,
            // but those were false detections too).  CRC-8 sync word is the
            // deterministic false-positive filter; this threshold pre-filters.
            const float fd_zc_threshold = config.fd_zc_threshold;
            if (best_zc_metric < fd_zc_threshold) {
                IRIS_LOG("[OFDM-SYNC] SC passed (%.3f) but FD-ZC too low (%.3f < %.2f, td=%.3f, n_used=%d) — rejected",
                         peak_metric, best_zc_metric, fd_zc_threshold, best_zc_td, n_used);
                return result;
            }

            // --- CFO estimation ---
            // Y2[k]*conj(Y1[k]) = |H[k]|²*|ZC[k]|² * exp(j*φ_cfo)
            // arg gives the CFO phase rotation over one symbol period.
            std::complex<float> cfo_corr(0.0f, 0.0f);
            for (int i = 0; i < (int)config.used_carrier_bins.size(); ++i) {
                int bin = config.used_carrier_bins[i];
                cfo_corr += Y2_buf[bin] * std::conj(Y1_buf[bin]);
            }
            float cfo_phase = std::arg(cfo_corr);
            result.cfo_hz = cfo_phase / (2.0f * (float)M_PI)
                          * ((float)config.sample_rate / symbol_len);

            IRIS_LOG("[OFDM-SYNC] FD-ZC=%.3f (td=%.3f) CFO: phase=%.4f rad -> %.2f Hz (|corr|=%.1f)",
                     best_zc_metric, best_zc_td, cfo_phase, result.cfo_hz, std::abs(cfo_corr));
        } else {
            IRIS_LOG("[OFDM-SYNC] cannot estimate CFO: train2 out of bounds");
            result.cfo_hz = 0.0f;
        }
    }

    // -----------------------------------------------------------------------
    // SNR estimate from SC metric.
    // SC metric M_sq = |P|²/(A·R) = (SNR/(SNR+1))².
    // peak_metric = sqrt(M_sq) = SNR/(SNR+1).
    // Inversion: SNR = peak_metric / (1 - peak_metric).
    // -----------------------------------------------------------------------
    {
        float snr_linear = (peak_metric < 0.999f) ? (peak_metric / (1.0f - peak_metric)) : 1000.0f;
        result.snr_est = 10.0f * std::log10(std::max(snr_linear, 1e-10f));
    }

    IRIS_LOG("[OFDM-SYNC] detect: SC=%.3f FD-ZC=%.3f at sample %d, CFO=%.1f Hz, SNR~%.1f dB",
             peak_metric, best_zc_metric, peak_d, result.cfo_hz, result.snr_est);

    // CFO sanity check: reject false triggers with implausible CFO.
    // FM radio audio paths introduce real frequency offsets from discriminator
    // tuning error and audio path phase shifts. OTA testing shows 17-25 Hz CFO
    // is normal between FM stations. Subcarrier spacing is 93.75 Hz, so up to
    // ±46 Hz (half-spacing) is correctable. Limit at 35 Hz to allow real FM
    // offsets while rejecting noise triggers.
    constexpr float CFO_MAX_HZ = 35.0f;
    if (std::abs(result.cfo_hz) > CFO_MAX_HZ) {
        IRIS_LOG("[OFDM-SYNC] CFO sanity check FAILED: |%.1f Hz| > %.0f Hz — false trigger rejected",
                 result.cfo_hz, CFO_MAX_HZ);
        result.detected = false;
        return result;
    }

    result.detected = true;
    result.sc_metric = peak_metric;
    result.zc_metric = best_zc_metric;

    // Track detection statistics
    {
        static int detect_count = 0;
        static float metric_sum = 0.0f;
        static float metric_min = 1.0f;
        static float metric_max = 0.0f;
        detect_count++;
        metric_sum += peak_metric;
        metric_min = std::min(metric_min, peak_metric);
        metric_max = std::max(metric_max, peak_metric);
        if (detect_count % 50 == 0) {
            IRIS_LOG("[OFDM-SYNC] detection stats: %d frames, metric avg=%.3f min=%.3f max=%.3f",
                     detect_count, metric_sum / detect_count, metric_min, metric_max);
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// CFO correction (in-place)
// ---------------------------------------------------------------------------
// Complex derotation exp(-jθ) correctly shifts the positive-frequency carriers
// (bins 4-33) that the demodulator extracts.
//
// The input is now an analytic signal (via Hilbert transform in modem.cc),
// so the CFO estimator works correctly: Y2·conj(Y1) is a complex quantity
// whose arg() gives the true CFO phase. Before the Hilbert transform, the
// real-valued input produced Hermitian symmetry that made the correlation
// real-valued and biased the estimator toward zero.
void ofdm_correct_cfo(std::complex<float>* iq, int n_samples,
                       float cfo_hz, int sample_rate)
{
    if (std::abs(cfo_hz) < 1e-6f) return;

    float phase_inc = -2.0f * (float)M_PI * cfo_hz / sample_rate;
    std::complex<float> rot(std::cos(phase_inc), std::sin(phase_inc));
    std::complex<float> phasor(1.0f, 0.0f);
    for (int n = 0; n < n_samples; ++n) {
        iq[n] *= phasor;
        phasor *= rot;
        // Renormalize every 512 samples to prevent magnitude drift
        if ((n & 0x1FF) == 0x1FF) {
            float mag = std::abs(phasor);
            if (mag > 0.0f) phasor /= mag;
        }
    }
}

// ---------------------------------------------------------------------------
// Channel estimation from training symbol (ZC-based)
// ---------------------------------------------------------------------------
OfdmChannelEst ofdm_estimate_channel(const std::complex<float>* iq_symbol,
                                      const OfdmConfig& config)
{
    OfdmChannelEst est;
    const int N = config.nfft;
    const int n_used = config.n_used_carriers;

    if (n_used <= 0) {
        IRIS_LOG("[OFDM-CE] no used carriers in config");
        return est;
    }

    // Cache ZC frequency-domain reference (doesn't change between frames)
    static std::vector<std::complex<float>> cached_zc_freq;
    static int cached_zc_n_used = 0;
    if (cached_zc_n_used != n_used) {
        cached_zc_freq = generate_zc_sequence(7, n_used);
        cached_zc_n_used = n_used;
    }
    const auto& zc_freq = cached_zc_freq;

    // FFT the received training symbol
    std::vector<std::complex<float>> Y(N);
    std::copy(iq_symbol, iq_symbol + N, Y.begin());
    fft_complex(Y.data(), N);

    // Extract H at used carrier bins: H[k] = Y[k] / X[k]
    // where X[k] is the known ZC frequency-domain value
    est.H.resize(n_used);
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        // Division by ZC value: since |ZC[i]| = 1 (unit magnitude),
        // H[i] = Y[bin] * conj(ZC[i]) / |ZC[i]|^2 = Y[bin] * conj(ZC[i])
        est.H[i] = Y[bin] * std::conj(zc_freq[i]);
    }

    // Noise estimation: fit a local quadratic (7-tap window) to H across
    // frequency, then measure residuals. A quadratic tracks FM pre-emphasis
    // slope (~6 dB/octave) and mild curvature without conflating channel
    // shape with noise — the old 3-tap moving average attributed the slope
    // gradient to noise, inflating noise_var at band edges by 6-10 dB.
    std::vector<std::complex<float>> H_smooth(n_used);
    {
        // Local quadratic regression over a window of +/-W carriers.
        // For each carrier i, fit H(x) = a + b*x + c*x^2 to neighbors,
        // evaluate at x=0 to get H_smooth[i]. x is centered at i.
        // Using W=3 (7-tap window) balances noise averaging with slope tracking.
        constexpr int W = 3;
        for (int i = 0; i < n_used; ++i) {
            int lo = std::max(0, i - W);
            int hi = std::min(n_used - 1, i + W);
            int count = hi - lo + 1;

            if (count < 3) {
                // Not enough points for quadratic -- fall back to local mean
                std::complex<float> sum(0.0f, 0.0f);
                for (int j = lo; j <= hi; ++j) sum += est.H[j];
                H_smooth[i] = sum / (float)count;
            } else {
                // Solve normal equations for quadratic fit (real and imag independently)
                // Basis: 1, x, x^2 where x = j - i
                float S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
                float Yr0 = 0, Yr1 = 0, Yr2 = 0;
                float Yi0 = 0, Yi1 = 0, Yi2 = 0;
                for (int j = lo; j <= hi; ++j) {
                    float x = (float)(j - i);
                    float x2 = x * x;
                    S0 += 1.0f;
                    S1 += x;
                    S2 += x2;
                    S3 += x * x2;
                    S4 += x2 * x2;
                    Yr0 += est.H[j].real();
                    Yr1 += est.H[j].real() * x;
                    Yr2 += est.H[j].real() * x2;
                    Yi0 += est.H[j].imag();
                    Yi1 += est.H[j].imag() * x;
                    Yi2 += est.H[j].imag() * x2;
                }
                float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
                if (std::abs(D) > 1e-12f) {
                    float Dr_a = Yr0*(S2*S4-S3*S3) - S1*(Yr1*S4-Yr2*S3) + S2*(Yr1*S3-Yr2*S2);
                    float Di_a = Yi0*(S2*S4-S3*S3) - S1*(Yi1*S4-Yi2*S3) + S2*(Yi1*S3-Yi2*S2);
                    H_smooth[i] = std::complex<float>(Dr_a / D, Di_a / D);
                } else {
                    H_smooth[i] = est.H[i];
                }
            }
        }
    }

    // Noise variance per carrier: |H[k] - H_smooth[k]|^2
    est.noise_var.resize(n_used);
    est.snr_per_carrier.resize(n_used);

    float snr_sum_db = 0.0f;
    float min_snr_db = 100.0f;
    float max_snr_db = -100.0f;
    float mean_H_mag = 0.0f;

    for (int i = 0; i < n_used; ++i) {
        std::complex<float> residual = est.H[i] - H_smooth[i];
        est.noise_var[i] = std::norm(residual);

        // Floor noise variance: 1e-6 caps per-carrier SNR at ~60 dB
        if (est.noise_var[i] < 1e-6f)
            est.noise_var[i] = 1e-6f;

        float signal_power = std::norm(est.H[i]);
        est.snr_per_carrier[i] = signal_power / est.noise_var[i];

        float snr_db = 10.0f * std::log10(std::max(est.snr_per_carrier[i], 1e-10f));
        snr_sum_db += snr_db;
        min_snr_db = std::min(min_snr_db, snr_db);
        max_snr_db = std::max(max_snr_db, snr_db);
        mean_H_mag += std::abs(est.H[i]);
    }

    est.mean_snr_db = snr_sum_db / n_used;
    mean_H_mag /= n_used;

    IRIS_LOG("[OFDM-CE] channel: mean|H|=%.3f, SNR range %.1f-%.1f dB, mean=%.1f dB, %d carriers",
             mean_H_mag, min_snr_db, max_snr_db, est.mean_snr_db, n_used);

    return est;
}

// ---------------------------------------------------------------------------
// Channel update from block-pilot symbol (IIR smoothing)
// ---------------------------------------------------------------------------
void ofdm_update_channel(OfdmChannelEst& est,
                          const std::complex<float>* pilot_symbol_freq,
                          const OfdmConfig& config, float alpha)
{
    const int n_used = config.n_used_carriers;
    if ((int)est.H.size() != n_used) {
        IRIS_LOG("[OFDM-CE] update: H size mismatch (%d vs %d)", (int)est.H.size(), n_used);
        return;
    }

    // Compute new H from block-pilot (all carriers = +1)
    // pilot_symbol_freq is already in frequency domain (NFFT-point FFT output)
    std::vector<std::complex<float>> H_new(n_used);
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        H_new[i] = pilot_symbol_freq[bin];  // Y[k] / (+1) = Y[k]
    }

    // Compute noise variance BEFORE IIR update (residual = H_new - H_old).
    // Computing after IIR would underestimate by factor (1-alpha)^2.
    float snr_sum_db = 0.0f;
    for (int i = 0; i < n_used; ++i) {
        std::complex<float> residual = H_new[i] - est.H[i];
        float new_nv = std::norm(residual);
        est.noise_var[i] = alpha * new_nv + (1.0f - alpha) * est.noise_var[i];
        if (est.noise_var[i] < 1e-6f) est.noise_var[i] = 1e-6f;
    }

    // IIR update: H[k] = alpha * H_new[k] + (1 - alpha) * H_old[k]
    for (int i = 0; i < n_used; ++i) {
        est.H[i] = alpha * H_new[i] + (1.0f - alpha) * est.H[i];
    }

    for (int i = 0; i < n_used; ++i) {

        float signal_power = std::norm(est.H[i]);
        est.snr_per_carrier[i] = signal_power / est.noise_var[i];

        float snr_db = 10.0f * std::log10(std::max(est.snr_per_carrier[i], 1e-10f));
        snr_sum_db += snr_db;
    }

    est.mean_snr_db = snr_sum_db / n_used;
}

// ---------------------------------------------------------------------------
// Pilot interpolation for data symbols
// ---------------------------------------------------------------------------
void ofdm_interpolate_pilots(OfdmChannelEst& est,
                              const std::complex<float>* symbol_freq,
                              const OfdmConfig& config,
                              float alpha)
{
    const int n_used = config.n_used_carriers;
    const int n_pilot = config.n_pilot_carriers;

    if (n_pilot < 2 || n_used <= 0) return;

    // Extract H at pilot positions: H_pilot[k] = Y_pilot[k] / (+1) = Y_pilot[k]
    struct PilotPoint {
        int used_idx;
        std::complex<float> H;
    };
    std::vector<PilotPoint> pilots;
    pilots.reserve(n_pilot);

    for (int i = 0; i < n_used; i += config.pilot_carrier_spacing) {
        int bin = config.used_carrier_bins[i];
        PilotPoint pp;
        pp.used_idx = i;
        pp.H = symbol_freq[bin];
        pilots.push_back(pp);
    }

    if (pilots.size() < 2) return;

    // Linear interpolation between pilot positions to get H_new at all used carriers
    std::vector<std::complex<float>> H_new(n_used);
    int p = 0;

    for (int i = 0; i < n_used; ++i) {
        while (p + 1 < (int)pilots.size() - 1 && i > pilots[p + 1].used_idx) {
            ++p;
        }

        if (i <= pilots[0].used_idx) {
            if (pilots.size() >= 2) {
                float span = (float)(pilots[1].used_idx - pilots[0].used_idx);
                float t = (span > 0) ? (float)(i - pilots[0].used_idx) / span : 0.0f;
                H_new[i] = pilots[0].H + t * (pilots[1].H - pilots[0].H);
            } else {
                H_new[i] = pilots[0].H;
            }
        } else if (i >= pilots.back().used_idx) {
            int last = (int)pilots.size() - 1;
            if (last >= 1) {
                float span = (float)(pilots[last].used_idx - pilots[last - 1].used_idx);
                float t = (span > 0) ? (float)(i - pilots[last - 1].used_idx) / span : 1.0f;
                H_new[i] = pilots[last - 1].H + t * (pilots[last].H - pilots[last - 1].H);
            } else {
                H_new[i] = pilots.back().H;
            }
        } else {
            float span = (float)(pilots[p + 1].used_idx - pilots[p].used_idx);
            float t = (span > 0) ? (float)(i - pilots[p].used_idx) / span : 0.0f;
            H_new[i] = pilots[p].H + t * (pilots[p + 1].H - pilots[p].H);
        }
    }

    // Compute pilot noise_var BEFORE IIR update (residual vs current estimate).
    // M9: Separate noise from channel variation (Doppler).
    std::complex<float> phase_acc(0.0f, 0.0f);
    for (int j = 0; j < (int)pilots.size(); ++j) {
        int idx = pilots[j].used_idx;
        float w = std::norm(est.H[idx]);
        phase_acc += w * (pilots[j].H * std::conj(est.H[idx]));
    }
    float mean_phase_rot = std::arg(phase_acc);
    std::complex<float> phase_correction = std::complex<float>(
        std::cos(-mean_phase_rot), std::sin(-mean_phase_rot));

    std::vector<float> pilot_nv(pilots.size());
    for (int j = 0; j < (int)pilots.size(); ++j) {
        int idx = pilots[j].used_idx;
        std::complex<float> residual = pilots[j].H * phase_correction - est.H[idx];
        pilot_nv[j] = std::norm(residual);
        if (pilot_nv[j] < 1e-6f)
            pilot_nv[j] = std::max(1e-6f, est.noise_var[idx]);
    }

    // IIR blend
    for (int i = 0; i < n_used; ++i) {
        est.H[i] = alpha * H_new[i] + (1.0f - alpha) * est.H[i];
    }

    // Interpolate noise_var across all carriers
    p = 0;
    for (int i = 0; i < n_used; ++i) {
        while (p + 1 < (int)pilots.size() - 1 && i > pilots[p + 1].used_idx) {
            ++p;
        }

        float nv_interp;
        if (i <= pilots[0].used_idx) {
            nv_interp = pilot_nv[0];
        } else if (i >= pilots.back().used_idx) {
            nv_interp = pilot_nv.back();
        } else {
            float span = (float)(pilots[p + 1].used_idx - pilots[p].used_idx);
            float t = (span > 0) ? (float)(i - pilots[p].used_idx) / span : 0.0f;
            nv_interp = pilot_nv[p] + t * (pilot_nv[p + 1] - pilot_nv[p]);
        }

        est.noise_var[i] = nv_interp;
        if (est.noise_var[i] < 1e-6f) est.noise_var[i] = 1e-6f;

        float signal_power = std::norm(est.H[i]);
        est.snr_per_carrier[i] = signal_power / est.noise_var[i];
    }

    // Update mean SNR
    float snr_sum_db = 0.0f;
    for (int i = 0; i < n_used; ++i) {
        snr_sum_db += 10.0f * std::log10(std::max(est.snr_per_carrier[i], 1e-10f));
    }
    est.mean_snr_db = snr_sum_db / n_used;
}

// ---------------------------------------------------------------------------
// Fine CFO estimation from training symbol
// ---------------------------------------------------------------------------
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config)
{
    const int n_used = config.n_used_carriers;
    if (n_used < 4 || (int)est.H.size() != n_used) return 0.0f;

    // CFO manifests as a common phase rotation on all subcarriers:
    // phase(H[k]) = 2*pi * delta_f * nfft / Fs
    // Weighted mean phase (by |H|^2) gives robust estimate.
    float sum_wp = 0.0f;
    float sum_w2 = 0.0f;
    for (int i = 0; i < n_used; i++) {
        float w = std::norm(est.H[i]);
        if (w < 1e-12f) continue;
        sum_wp += w * std::arg(est.H[i]);
        sum_w2 += w;
    }

    if (sum_w2 < 1e-12f) return 0.0f;

    float mean_phase = sum_wp / sum_w2;
    float fine_cfo = mean_phase * config.sample_rate / (2.0f * (float)M_PI * config.nfft);

    IRIS_LOG("[OFDM-SYNC] fine CFO: mean_phase=%.4f rad -> %.2f Hz", mean_phase, fine_cfo);

    return fine_cfo;
}

} // namespace iris
