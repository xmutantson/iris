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
// Schmidl-Cox frame detection
// ---------------------------------------------------------------------------
OfdmSyncResult ofdm_detect_frame(const std::complex<float>* iq, int n_samples,
                                  const OfdmConfig& config)
{
    OfdmSyncResult result;
    const int L = config.nfft / 2;  // Half-symbol length

    // Need at least one full training symbol + CP to search
    const int min_samples = config.cp_samples + config.nfft + L;
    if (n_samples < min_samples) {
        IRIS_LOG("[OFDM-SYNC] insufficient samples for detection: %d < %d",
                 n_samples, min_samples);
        return result;
    }

    // Sliding window computation of P(d) and R(d)
    // P(d) = sum_{m=0}^{L-1} r(d+m) * conj(r(d+m+L))
    // R(d) = sum_{m=0}^{L-1} |r(d+m+L)|^2
    // M(d) = |P(d)|^2 / R(d)^2

    const int search_len = n_samples - 2 * L;
    if (search_len <= 0) {
        IRIS_LOG("[OFDM-SYNC] search range empty");
        return result;
    }

    // Compute initial P, R (second-half energy), and E (first-half energy) for d=0
    std::complex<float> P(0.0f, 0.0f);
    float R = 0.0f;   // sum |r(d+m+L)|^2 for m=0..L-1
    float E = 0.0f;   // sum |r(d+m)|^2   for m=0..L-1
    for (int m = 0; m < L; ++m) {
        P += iq[m] * std::conj(iq[m + L]);
        R += std::norm(iq[m + L]);
        E += std::norm(iq[m]);
    }

    // Detection strategy: find the FIRST position where M exceeds the threshold,
    // then take that as frame_start (left edge of CP plateau). This prevents
    // false detections from data symbols later in the buffer.
    // Threshold 0.70: FM radio processing (pre-emphasis + deviation limiting +
    // de-emphasis) degrades OFDM preamble to M ≈ 0.75-0.90 OTA.
    // Self-hear showed M=0.86 at threshold 0.85 — barely above.
    // False positives at 0.70 are filtered by header CRC check in demodulator.
    // Preamble boost (√2) improves margin by ~3 dB.
    // Noise/tones produce M ≈ 0.2-0.5 (probe remnants up to ~0.65).
    constexpr float DETECTION_THRESHOLD = 0.70f;

    int detect_d = -1;
    float detect_metric = 0.0f;
    std::complex<float> detect_P(0.0f, 0.0f);
    float detect_R = 0.0f;

    // Evaluate at d=0
    {
        float P_mag2 = std::norm(P);
        float denom = E * R;
        float M = (denom > 1e-20f) ? (P_mag2 / denom) : 0.0f;
        if (M >= DETECTION_THRESHOLD) {
            detect_d = 0;
            detect_metric = M;
            detect_P = P;
            detect_R = R;
        }
    }

    if (detect_d < 0) {
        // Slide until we find the first threshold crossing
        for (int d = 1; d < search_len; ++d) {
            P -= iq[d - 1] * std::conj(iq[d - 1 + L]);
            P += iq[d + L - 1] * std::conj(iq[d + L - 1 + L]);
            R -= std::norm(iq[d - 1 + L]);
            R += std::norm(iq[d + L - 1 + L]);
            E -= std::norm(iq[d - 1]);
            E += std::norm(iq[d + L - 1]);

            float P_mag2 = std::norm(P);
            float denom = E * R;
            float M = (denom > 1e-20f) ? (P_mag2 / denom) : 0.0f;

            if (M >= DETECTION_THRESHOLD) {
                detect_d = d;
                detect_metric = M;
                detect_P = P;
                detect_R = R;
                break;
            }
        }
    }

    if (detect_d < 0) {
        IRIS_LOG("[OFDM-SYNC] Schmidl-Cox: no detection (threshold=%.2f)", DETECTION_THRESHOLD);
        return result;
    }

    result.schmidl_metric = detect_metric;
    result.detected = true;
    result.frame_start = detect_d;

    std::complex<float> best_P = detect_P;
    float best_R = detect_R;

    // CFO estimation: cfo_hz = angle(P(d_peak)) * sample_rate / (2 * pi * L)
    float phase = std::arg(best_P);
    result.cfo_hz = phase * config.sample_rate / (2.0f * (float)M_PI * L);

    // SNR estimate from Schmidl-Cox: SNR ~ |P|^2 / (R - |P|)^2
    // More stable form: SNR ~ |P| / (R - |P|)
    float P_mag = std::abs(best_P);
    float denom = best_R - P_mag;
    if (denom > 1e-10f) {
        float snr_linear = P_mag / denom;
        result.snr_est = 10.0f * std::log10(std::max(snr_linear, 1e-10f));
    } else {
        result.snr_est = 40.0f;  // Very high SNR
    }

    IRIS_LOG("[OFDM-SYNC] Schmidl-Cox: M=%.4f at sample %d, CFO=%.1f Hz, SNR~%.1f dB",
             detect_metric, detect_d, result.cfo_hz, result.snr_est);

    // M10: Track detection metric statistics for threshold tuning
    {
        static int detect_count = 0;
        static float metric_sum = 0.0f;
        static float metric_min = 1.0f;
        static float metric_max = 0.0f;
        detect_count++;
        metric_sum += detect_metric;
        metric_min = std::min(metric_min, detect_metric);
        metric_max = std::max(metric_max, detect_metric);
        if (detect_count % 50 == 0) {
            IRIS_LOG("[OFDM-SYNC] detection stats: %d frames, M avg=%.3f min=%.3f max=%.3f",
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

    const float phase_inc = -2.0f * (float)M_PI * cfo_hz / sample_rate;
    float phase = 0.0f;

    for (int n = 0; n < n_samples; ++n) {
        iq[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
        phase += phase_inc;
        // Keep phase in [-pi, pi] to avoid precision loss
        if (phase > (float)M_PI)       phase -= 2.0f * (float)M_PI;
        else if (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
    }
}

// ---------------------------------------------------------------------------
// Channel estimation from training symbol 2
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

    // FFT the received training symbol 2
    std::vector<std::complex<float>> Y(N);
    std::copy(iq_symbol, iq_symbol + N, Y.begin());
    fft_complex(Y.data(), N);

    // Extract H at used carrier bins: H[k] = Y[k] / X[k] = Y[k] (since X=+1)
    est.H.resize(n_used);
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        est.H[i] = Y[bin];
    }

    // Noise estimation: fit a local quadratic (7-tap window) to H across
    // frequency, then measure residuals. A quadratic tracks FM pre-emphasis
    // slope (~6 dB/octave) and mild curvature without conflating channel
    // shape with noise — the old 3-tap moving average attributed the slope
    // gradient to noise, inflating noise_var at band edges by 6-10 dB.
    std::vector<std::complex<float>> H_smooth(n_used);
    {
        // Local quadratic regression over a window of ±W carriers.
        // For each carrier i, fit H(x) = a + b*x + c*x² to neighbors,
        // evaluate at x=0 to get H_smooth[i]. x is centered at i.
        // Using W=3 (7-tap window) balances noise averaging with slope tracking.
        constexpr int W = 3;
        for (int i = 0; i < n_used; ++i) {
            int lo = std::max(0, i - W);
            int hi = std::min(n_used - 1, i + W);
            int count = hi - lo + 1;

            if (count < 3) {
                // Not enough points for quadratic — fall back to local mean
                std::complex<float> sum(0.0f, 0.0f);
                for (int j = lo; j <= hi; ++j) sum += est.H[j];
                H_smooth[i] = sum / (float)count;
            } else {
                // Solve normal equations for quadratic fit (real and imag independently)
                // Basis: 1, x, x² where x = j - i
                // Sums: S0=n, S1=Σx, S2=Σx², S3=Σx³, S4=Σx⁴
                float S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
                float Yr0 = 0, Yr1 = 0, Yr2 = 0;  // real projections
                float Yi0 = 0, Yi1 = 0, Yi2 = 0;  // imag projections
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
                // Normal equations: [S0 S1 S2; S1 S2 S3; S2 S3 S4] * [a;b;c] = [Y0;Y1;Y2]
                // We only need a (the constant term = value at x=0).
                // Cramer's rule for a:
                float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
                if (std::abs(D) > 1e-12f) {
                    float Dr_a = Yr0*(S2*S4-S3*S3) - S1*(Yr1*S4-Yr2*S3) + S2*(Yr1*S3-Yr2*S2);
                    float Di_a = Yi0*(S2*S4-S3*S3) - S1*(Yi1*S4-Yi2*S3) + S2*(Yi1*S3-Yi2*S2);
                    H_smooth[i] = std::complex<float>(Dr_a / D, Di_a / D);
                } else {
                    // Degenerate — fall back to center value
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

        // Floor noise variance: 1e-6 caps per-carrier SNR at ~60 dB,
        // preventing MMSE equalizer from amplifying faded carriers
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
    // Computing after IIR would underestimate by factor (1-alpha)² ≈ 0.49.
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
    // After IIR, residual would be attenuated by (1-alpha), underestimating noise.
    //
    // M9: Separate noise from channel variation (Doppler).
    // Compute mean phase rotation across all pilots weighted by |H_old|²,
    // then remove it before computing residuals. This prevents common Doppler
    // shift from inflating the noise variance estimate.
    std::complex<float> phase_acc(0.0f, 0.0f);
    for (int j = 0; j < (int)pilots.size(); ++j) {
        int idx = pilots[j].used_idx;
        float w = std::norm(est.H[idx]);  // |H_old|² weight
        phase_acc += w * (pilots[j].H * std::conj(est.H[idx]));
    }
    float mean_phase_rot = std::arg(phase_acc);
    std::complex<float> phase_correction = std::complex<float>(
        std::cos(-mean_phase_rot), std::sin(-mean_phase_rot));

    std::vector<float> pilot_nv(pilots.size());
    for (int j = 0; j < (int)pilots.size(); ++j) {
        int idx = pilots[j].used_idx;
        // Remove common phase rotation before computing residual
        std::complex<float> residual = pilots[j].H * phase_correction - est.H[idx];
        pilot_nv[j] = std::norm(residual);
        if (pilot_nv[j] < 1e-6f)
            pilot_nv[j] = std::max(1e-6f, est.noise_var[idx]);
    }

    // IIR blend pilot-interpolated H with existing estimate.
    // alpha=0.3 default: slowly track time-varying channels without
    // destroying the training-based estimate from noisy comb pilots.
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
// Fine CFO estimation from training symbol 2 pilot phases
//
// After coarse CFO correction, H[k] = Y[k]/X[k] = Y[k] (X=+1) should have
// zero phase if CFO=0. Any residual CFO causes a linear phase slope across
// frequency: phase(H[k]) ≈ 2π * Δf * (bin[k] / sample_rate).
// We estimate Δf via weighted linear regression of unwrapped phases.
// ---------------------------------------------------------------------------
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config)
{
    const int n_used = config.n_used_carriers;
    if (n_used < 4 || (int)est.H.size() != n_used) return 0.0f;

    // CFO manifests as a common phase rotation on all subcarriers:
    // phase(H[k]) = 2π * Δf * nfft / Fs
    // Weighted mean phase (by |H|^2) gives robust estimate.
    float sum_wp = 0.0f;
    float sum_w2 = 0.0f;
    for (int i = 0; i < n_used; i++) {
        float w = std::norm(est.H[i]);  // |H|^2
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
