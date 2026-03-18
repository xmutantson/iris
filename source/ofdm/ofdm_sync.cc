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
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> generate_zc_training_symbol(const OfdmConfig& config,
                                                              int root)
{
    const int nfft = config.nfft;
    const int n_used = config.n_used_carriers;

    // Generate ZC sequence in frequency domain for used carriers
    auto zc_freq = generate_zc_sequence(root, n_used);

    // Place into FFT bins
    std::vector<std::complex<float>> X(nfft, std::complex<float>(0.0f, 0.0f));
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        X[bin] = zc_freq[i];
    }

    // IFFT to time domain (ifft_complex divides by nfft internally)
    ifft_complex(X.data(), nfft);

    // Normalize to unit RMS
    float energy = 0.0f;
    for (int n = 0; n < nfft; ++n) {
        energy += std::norm(X[n]);
    }
    float rms = std::sqrt(energy / nfft);
    if (rms > 1e-10f) {
        float scale = 1.0f / rms;
        for (int n = 0; n < nfft; ++n) {
            X[n] *= scale;
        }
    }

    return X;
}

// ---------------------------------------------------------------------------
// ZC cross-correlation frame detection
// ---------------------------------------------------------------------------

// Detection threshold for ZC cross-correlation peak.
// ZC has near-zero autocorrelation sidelobes, so we can use a lower threshold
// than Schmidl-Cox (which was 0.55). Start at 0.40.
static constexpr float DETECTION_THRESHOLD = 0.40f;

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

    // Need at least two training symbols (train1 + train2) for detection + CFO
    const int min_samples = 2 * symbol_len;
    if (n_samples < min_samples) {
        IRIS_LOG("[OFDM-SYNC] insufficient samples for detection: %d < %d",
                 n_samples, min_samples);
        return result;
    }

    // -----------------------------------------------------------------------
    // Cache ZC training symbol AND its conjugate FFT (padded to seg_size)
    // -----------------------------------------------------------------------
    static std::vector<std::complex<float>> zc_td;
    static float zc_energy = 0.0f;
    static int cached_nfft = 0;
    static int cached_n_used = 0;
    static int seg_size = 0;   // FFT segment size (power of 2 >= 2*nfft)
    static std::vector<std::complex<float>> zc_conj_fft; // conj(FFT(zc_td, seg_size))

    // Incremental search state
    static int last_searched_pos = 0;
    static int last_n_samples = 0;

    if (cached_nfft != nfft || cached_n_used != config.n_used_carriers) {
        zc_td = generate_zc_training_symbol(config);
        zc_energy = 0.0f;
        for (int n = 0; n < nfft; ++n) {
            zc_energy += std::norm(zc_td[n]);
        }

        // Compute FFT segment size: power of 2 >= 2*nfft (for linear correlation)
        seg_size = next_pow2(2 * nfft);
        if (seg_size < 4096) seg_size = 4096; // minimum for efficiency

        // Pad ZC to seg_size, compute FFT, then conjugate
        zc_conj_fft.assign(seg_size, std::complex<float>(0.0f, 0.0f));
        for (int n = 0; n < nfft; ++n) {
            zc_conj_fft[n] = zc_td[n];
        }
        fft_complex(zc_conj_fft.data(), seg_size);
        for (int n = 0; n < seg_size; ++n) {
            zc_conj_fft[n] = std::conj(zc_conj_fft[n]);
        }

        cached_nfft = nfft;
        cached_n_used = config.n_used_carriers;
        last_searched_pos = 0;
        last_n_samples = 0;
        IRIS_LOG("[OFDM-SYNC] ZC training symbol cached: nfft=%d, n_used=%d, energy=%.1f, seg_size=%d",
                 nfft, config.n_used_carriers, zc_energy, seg_size);
    }

    // Search range: d is CP start, body at d+cp, need d+cp+nfft <= n_samples
    // Also leave room for train2: need d + 2*symbol_len <= n_samples
    const int search_len = n_samples - symbol_len - nfft; // leave room for train2
    if (search_len <= 0) {
        IRIS_LOG("[OFDM-SYNC] search range empty");
        return result;
    }

    // Detect buffer shrink/reset: if n_samples decreased, reset incremental pos
    if (n_samples < last_n_samples || last_searched_pos >= search_len) {
        last_searched_pos = 0;
    }
    last_n_samples = n_samples;

    // The correlation maps body position (d+cp) to lag.
    // We search body positions from last_searched_pos+cp to search_len-1+cp.
    // Effective input range: [search_start_body .. search_len-1+cp+nfft-1]
    const int search_start = last_searched_pos; // start d for incremental
    const int body_offset = cp; // body = d + cp

    // -----------------------------------------------------------------------
    // FFT-based cross-correlation in overlapping segments
    // -----------------------------------------------------------------------
    // Each segment of seg_size input samples yields (seg_size - nfft + 1) valid
    // correlation lags. Overlap by nfft-1 to cover all lags.
    const int valid_per_seg = seg_size - nfft + 1;

    float peak_metric = 0.0f;
    int peak_d = -1;
    std::complex<float> peak_xcorr(0.0f, 0.0f);

    // We also need running signal energy for normalization.
    // Compute it per-candidate in the refine pass only (cheap after FFT narrows candidates).

    // Scratch buffer (static to avoid reallocation)
    static std::vector<std::complex<float>> seg_buf;

    if ((int)seg_buf.size() != seg_size) {
        seg_buf.resize(seg_size);
    }

    // Coarse pass: find the best correlation magnitude across all segments.
    // We process overlapping segments of the input starting from body positions.
    // body_pos = search_start + body_offset is where the first segment starts in iq[].
    // Each valid lag l in [0, valid_per_seg) corresponds to d = seg_d_start + l
    // where body = d + cp = seg_input_start + l.

    float coarse_best_mag2 = 0.0f;
    int coarse_best_d = -1;

    for (int seg_d = search_start; seg_d < search_len; seg_d += valid_per_seg) {
        int seg_body_start = seg_d + body_offset; // where this segment reads from iq[]
        int seg_input_len = std::min(seg_size, n_samples - seg_body_start);
        if (seg_input_len < nfft) break; // not enough samples for even one correlation

        // Copy input segment, zero-pad remainder
        for (int i = 0; i < seg_input_len; ++i) {
            seg_buf[i] = iq[seg_body_start + i];
        }
        for (int i = seg_input_len; i < seg_size; ++i) {
            seg_buf[i] = std::complex<float>(0.0f, 0.0f);
        }

        // FFT of input segment
        fft_complex(seg_buf.data(), seg_size);

        // Multiply by conjugate FFT of ZC reference (= correlation in freq domain)
        for (int i = 0; i < seg_size; ++i) {
            seg_buf[i] *= zc_conj_fft[i];
        }

        // IFFT to get correlation lags
        ifft_complex(seg_buf.data(), seg_size);

        // Scan valid lags for peaks (only lags where full nfft overlap exists)
        int n_valid = std::min(valid_per_seg, seg_input_len - nfft + 1);
        // Also constrain to search_len
        n_valid = std::min(n_valid, search_len - seg_d);

        for (int l = 0; l < n_valid; ++l) {
            float mag2 = std::norm(seg_buf[l]);
            if (mag2 > coarse_best_mag2) {
                coarse_best_mag2 = mag2;
                coarse_best_d = seg_d + l;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Refine: sample-by-sample normalized correlation around the coarse peak
    // -----------------------------------------------------------------------
    if (coarse_best_d >= 0) {
        // Compute normalized metric at coarse peak and neighbors (+/- 2 samples)
        // to find the precise peak with proper normalization.
        int refine_lo = std::max(0, coarse_best_d - 2);
        int refine_hi = std::min(search_len - 1, coarse_best_d + 2);

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
            if (m > peak_metric) {
                peak_metric = m;
                peak_d = d;
                peak_xcorr = xc;
            }
        }
    }

    // Update incremental search position (search from here next time)
    last_searched_pos = std::max(0, search_len - nfft);

    if (peak_d < 0 || peak_metric < DETECTION_THRESHOLD) {
        static int no_detect_count = 0;
        no_detect_count++;
        if ((no_detect_count % 200) == 1) {  // log every 200th miss (~5 seconds)
            IRIS_LOG("[OFDM-SYNC] no detection: peak_metric=%.3f at d=%d/%d (threshold=%.2f, %d samples)",
                     peak_metric, peak_d, search_len, DETECTION_THRESHOLD, n_samples);
        }
        return result;
    }

    // Reset incremental position on detection so next call searches fresh
    last_searched_pos = 0;

    result.detected = true;
    result.frame_start = peak_d;
    result.schmidl_metric = peak_metric;

    // -----------------------------------------------------------------------
    // CFO estimation from phase difference between train1 and train2
    // train1 body starts at peak_d + cp
    // train2 body starts at peak_d + symbol_len + cp
    // cfo_phase = angle(sum_n r[train2+n] * conj(r[train1+n])) for n=0..nfft-1
    // cfo_hz = cfo_phase / (2*pi) * (sample_rate / nfft)
    // -----------------------------------------------------------------------
    {
        int train1_body = peak_d + cp;
        int train2_body = peak_d + symbol_len + cp;

        // Bounds check: need train2_body + nfft <= n_samples
        if (train2_body + nfft <= n_samples) {
            std::complex<float> cfo_corr(0.0f, 0.0f);
            for (int n = 0; n < nfft; ++n) {
                cfo_corr += iq[train2_body + n] * std::conj(iq[train1_body + n]);
            }
            float cfo_phase = std::arg(cfo_corr);
            result.cfo_hz = cfo_phase / (2.0f * (float)M_PI) * ((float)config.sample_rate / nfft);
        } else {
            IRIS_LOG("[OFDM-SYNC] cannot estimate CFO: train2 out of bounds");
            result.cfo_hz = 0.0f;
        }
    }

    // -----------------------------------------------------------------------
    // SNR estimate: signal power from correlation peak, noise from residual
    // SNR ~ peak_metric^2 / (1 - peak_metric^2)
    // -----------------------------------------------------------------------
    {
        float m2 = peak_metric * peak_metric;
        float snr_linear = (m2 < 0.999f) ? (m2 / (1.0f - m2)) : 1000.0f;
        result.snr_est = 10.0f * std::log10(std::max(snr_linear, 1e-10f));
    }

    IRIS_LOG("[OFDM-SYNC] ZC detect: metric=%.4f at sample %d, CFO=%.1f Hz, SNR~%.1f dB",
             peak_metric, peak_d, result.cfo_hz, result.snr_est);

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
        // Renormalize every 1024 samples to prevent magnitude drift
        if ((n & 0x3FF) == 0x3FF) {
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
