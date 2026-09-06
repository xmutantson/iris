#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"   // get_uniform_tone_map
#include "common/fft.h"
#include "common/logging.h"
#include "native/constellation.h"  // demap_soft
#include "native/nuc_tables.h"     // NucTable, get_nuc_table (for BPS)
#include "native/frame.h"      // crc32, KalmanTrace
#include <cmath>
#include <algorithm>
#include <climits>
#include <cstring>
#include <cstdlib>   // getenv/atof (S2 estimator/EQ toggles)
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// ============================================================================
//  Decision-residual Es/No estimator (FreeDV/Rhizomatica-style)
//
//  For each post-equalization QPSK data symbol, the signal lies along whichever
//  real/imag axis is larger; the orthogonal axis is pure noise. Accumulate
//  Welford-variance of the minor-axis component across the payload, double it
//  (we used one of two orthogonal noise components), compare to signal power.
//
//  Ratio is immune to deep-fade carriers: on a dead carrier both signal and
//  noise drop in lockstep and the carrier contributes nothing to either sum.
//  This is the root fix for Iris's NV_FLOOR/sigma-saturation pathology.
//
//  Reference: github.com/Rhizomatica/mercury modem/freedv/ofdm.c:1967-2001
//             (ofdm_esno_est_calc), which traces to codec2's esno_est.m.
// ============================================================================
// Post-despread effective SINR (dB) from the metric MMSE mean:
//   s = mean(nv_k/(|H_k|^2+nv_k)) = 1-mu  ->  gamma_eff = mu/(1-mu) = (1-s)/s.
// Standard MMSE-FDE SC-FDMA result; clamped to +/-60 dB.
static float metric_gamma_eff_db(float s) {
    if (s < 1e-6f) s = 1e-6f;
    if (s > 1.0f - 1e-6f) s = 1.0f - 1e-6f;
    return 10.0f * std::log10((1.0f - s) / s);
}

static float compute_esno_db_qpsk(const std::complex<float>* syms, int n) {
    if (n < 2) return 0.0f;
    // Decision-directed residual, generic form that works regardless of
    // constellation rotation. For every outer-ring sample, take the distance
    // to the nearest post-EQ decision region on each axis (i.e. the minor
    // axis after auto-alignment: whichever of {re, im} has smaller
    // magnitude, since a clean QPSK symbol has one axis near zero when
    // rotated to grid OR at ±sig_rms on both axes when at 45°). We handle
    // both by picking per-symbol whichever axis is minor, and if both
    // magnitudes are comparable we subtract sig_rms from each and use the
    // smaller residual. This gives a consistent unbiased noise estimator
    // for either QPSK orientation.
    float sig_var = 0.0f;
    const float step = 1.0f / (float)n;
    for (int i = 0; i < n; i++) sig_var += std::norm(syms[i]) * step;
    float sig_rms = std::sqrt(sig_var);

    // Signal along one axis = sig_rms * sqrt(2) (axis-aligned)
    // Signal at 45° = sig_rms on both axes
    // Detect orientation: if ratio of mean |re| to mean |im| is close to 1,
    // it's 45°; otherwise axis-aligned. Iris's DFT-spread IDFT output is
    // empirically axis-aligned (constellation log shows values like
    // (0.47,0.77), (-0.01,-1.17) — strongly one-axis-dominant).
    float sum_abs_re = 0, sum_abs_im = 0;
    for (int i = 0; i < n; i++) {
        sum_abs_re += std::fabs(syms[i].real());
        sum_abs_im += std::fabs(syms[i].imag());
    }
    float ratio = (sum_abs_im > 1e-6f) ? sum_abs_re / sum_abs_im : 1.0f;
    bool diagonal_45 = (ratio > 0.7f && ratio < 1.4f);

    constexpr float INV_SQRT2 = 0.70710678f;
    const std::complex<float> rot(INV_SQRT2, -INV_SQRT2);

    float sum_x = 0.0f, sum_xx = 0.0f;
    int nmin = 0;
    for (int i = 0; i < n; i++) {
        std::complex<float> s = diagonal_45 ? (syms[i] * rot) : syms[i];
        if (std::abs(s) > sig_rms) {
            float minor = (std::fabs(s.real()) > std::fabs(s.imag()))
                          ? s.imag() : s.real();
            sum_x  += minor;
            sum_xx += minor * minor;
            nmin++;
        }
    }
    float noise_var;
    if (nmin > 1)
        noise_var = (nmin * sum_xx - sum_x * sum_x) / (float)(nmin * (nmin - 1));
    else
        noise_var = sig_var;
    noise_var *= 2.0f;

    return 10.0f * std::log10((1e-12f + sig_var) / (1e-12f + noise_var));
}

static int fec_rate_num16(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2: return 8;
        case LdpcRate::RATE_5_8: return 10;
        case LdpcRate::RATE_3_4: return 12;
        case LdpcRate::RATE_7_8: return 14;
        default: return 8;
    }
}

static std::complex<float> nearest_payload_decision(
    std::complex<float> symbol, Modulation modulation, const NucTable* nuc)
{
    if (nuc) {
        if (!nuc->separable) {
            float best_distance = std::numeric_limits<float>::infinity();
            std::complex<float> best(0.0f, 0.0f);
            for (int i = 0; i < nuc->n_points_2d; ++i) {
                const float distance = std::norm(symbol - nuc->points_2d[i]);
                if (distance < best_distance) {
                    best_distance = distance;
                    best = nuc->points_2d[i];
                }
            }
            return best;
        }

        float nearest_i = 0.0f;
        float nearest_q = 0.0f;
        float best_i = std::numeric_limits<float>::infinity();
        float best_q = std::numeric_limits<float>::infinity();
        for (int i = 0; i < nuc->n_axis_1d; ++i) {
            const float di = symbol.real() - nuc->axis_1d[i];
            const float dq = symbol.imag() - nuc->axis_1d[i];
            if (di * di < best_i) {
                best_i = di * di;
                nearest_i = nuc->axis_1d[i];
            }
            if (dq * dq < best_q) {
                best_q = dq * dq;
                nearest_q = nuc->axis_1d[i];
            }
        }
        return {nearest_i, nearest_q};
    }

    uint8_t bits[16] = {};
    demap_symbol(symbol, bits, modulation);
    return map_symbol(bits, modulation);
}

static bool compute_payload_residual_snr_db(
    const std::vector<std::complex<float>>& symbols, const ToneMap& tone_map,
    LdpcRate fec_rate, float& snr_db)
{
    if (symbols.empty() || tone_map.n_data_carriers <= 0)
        return false;

    double signal_power = 0.0;
    double error_power = 0.0;
    std::size_t decisions = 0;
    const int rate_num16 = fec_rate_num16(fec_rate);
    for (std::size_t i = 0; i < symbols.size(); ++i) {
        const int carrier = static_cast<int>(
            i % static_cast<std::size_t>(tone_map.n_data_carriers));
        if (carrier >= static_cast<int>(tone_map.bits_per_carrier.size()))
            continue;
        const int bpc = tone_map.bits_per_carrier[carrier];
        if (bpc <= 0) continue;

        const Modulation modulation = bits_to_modulation(bpc);
        const NucTable* nuc = tone_map.use_nuc && bpc >= 4
            ? get_nuc_table(modulation, rate_num16) : nullptr;
        const auto decision = nearest_payload_decision(symbols[i], modulation,
                                                       nuc);
        const auto residual = symbols[i] - decision;
        signal_power += std::norm(decision);
        error_power += std::norm(residual);
        ++decisions;
    }

    if (decisions < 2 || signal_power <= 0.0 ||
        !std::isfinite(signal_power) || !std::isfinite(error_power))
        return false;
    snr_db = error_power <= 1e-12
        ? 60.0f
        : 10.0f * std::log10(static_cast<float>(signal_power / error_power));
    return std::isfinite(snr_db);
}

void ofdm_authorize_payload_estimator(OfdmDemodResult& result) {
    constexpr float kPayloadResidualCrosscheckDb = 15.0f;
    const float ceiling = result.payload_residual_snr_db +
                          kPayloadResidualCrosscheckDb;
    const bool estimator_crosschecked = result.payload_validated &&
        result.cfo_resolved && result.complete_boundary_validated &&
        result.payload_residual_snr_valid && std::isfinite(ceiling) &&
        std::isfinite(result.snr_db) &&
        std::isfinite(result.mean_channel_snr_db) &&
        std::isfinite(result.effective_snr_db) &&
        result.snr_db <= ceiling &&
        result.mean_channel_snr_db <= ceiling &&
        result.effective_snr_db <= ceiling;
    result.estimator_validity = estimator_crosschecked
        ? OfdmEstimatorValidity::PayloadValidatedForSelectedCfo
        : OfdmEstimatorValidity::PayloadValidatedEstimateWithheld;
}

// ============================================================================
//  Blind Phase Search (BPS) — replaces DD-CPE for QAM16+
//
//  Two-stage search exploiting QAM 4-fold symmetry (±45° unambiguous range).
//  Stage 1: 16 coarse test angles in [-π/4, +π/4)
//  Stage 2: 16 fine angles around stage-1 winner
//  Returns: estimated residual phase error (radians)
//
//  References: Pfau et al. 2009 (coherent optical BPS);
//              S-BPS, IEEE PTL 2014 (low-complexity two-stage variant).
// ============================================================================
static float bps_estimate(const std::complex<float>* eq, int n_data,
                          const ToneMap& tm, int fec_r16)
{
    // Build constellation reference for distance computation
    // Use NUC tables when available (match soft demapper)
    int bpc = 0;
    for (int k = 0; k < tm.n_data_carriers && k < n_data; k++)
        if (tm.bits_per_carrier[k] > bpc) bpc = tm.bits_per_carrier[k];
    if (bpc < 4) return 0.0f;  // BPS only for QAM16+

    Modulation mod = bits_to_modulation(bpc);
    const NucTable* nuc = nullptr;
    if (tm.use_nuc && bpc >= 4)
        nuc = get_nuc_table(mod, fec_r16);

    // Distance to nearest constellation point
    auto min_dist = [&](std::complex<float> sym) -> float {
        if (nuc) {
            float best = 1e30f;
            if (!nuc->separable) {
                for (int s = 0; s < nuc->n_points_2d; s++) {
                    float d = std::norm(sym - nuc->points_2d[s]);
                    if (d < best) best = d;
                }
            } else {
                int side = nuc->n_axis_1d;
                float best_i = 1e30f, best_q = 1e30f;
                for (int s = 0; s < side; s++) {
                    float di = sym.real() - nuc->axis_1d[s];
                    if (di * di < best_i) best_i = di * di;
                    float dq = sym.imag() - nuc->axis_1d[s];
                    if (dq * dq < best_q) best_q = dq * dq;
                }
                best = best_i + best_q;
            }
            return best;
        } else {
            uint8_t bits[16];  // must hold up to 1024QAM (10 bits); bits[8] overflowed
            demap_symbol(sym, bits, mod);
            std::complex<float> ref = map_symbol(bits, mod);
            return std::norm(sym - ref);
        }
    };

    // Stage 1: 16 coarse test angles in [-π/4, +π/4)
    constexpr int B1 = 16;
    constexpr float RANGE = (float)M_PI / 2.0f;  // π/2 = 90° total range
    float coarse_step = RANGE / B1;  // ~5.6°
    float best_angle = 0.0f;
    float best_metric = 1e30f;

    // Always test theta=0 explicitly (critical for clean signals)
    float zero_metric = 0.0f;
    for (int i = 0; i < n_data; i++) {
        int carrier_bpc = (i < tm.n_data_carriers) ? tm.bits_per_carrier[i] : 0;
        if (carrier_bpc < 4) continue;
        zero_metric += min_dist(eq[i]);
    }
    best_metric = zero_metric;

    for (int b = 0; b < B1; b++) {
        float theta = -(float)M_PI / 4.0f + ((float)b + 0.5f) * coarse_step;
        if (std::abs(theta) < 0.01f) continue;  // skip near-zero (tested above)
        std::complex<float> rot(std::cos(theta), std::sin(theta));
        float dist_sum = 0.0f;
        for (int i = 0; i < n_data; i++) {
            int carrier_bpc = (i < tm.n_data_carriers) ? tm.bits_per_carrier[i] : 0;
            if (carrier_bpc < 4) continue;
            dist_sum += min_dist(eq[i] * rot);
        }
        if (dist_sum < best_metric) {
            best_metric = dist_sum;
            best_angle = theta;
        }
    }

    // If zero was best (or within 1% of best), return 0 — no correction needed
    if (zero_metric <= best_metric * 1.01f) return 0.0f;

    // Stage 2: 16 fine angles around coarse winner
    constexpr int B2 = 16;
    float fine_step = coarse_step / B2;  // ~0.35°
    float fine_best_angle = best_angle;
    float fine_best_metric = best_metric;

    for (int b = -B2/2; b <= B2/2; b++) {
        float theta = best_angle + (float)b * fine_step;
        std::complex<float> rot(std::cos(theta), std::sin(theta));
        float dist_sum = 0.0f;
        for (int i = 0; i < n_data; i++) {
            int carrier_bpc = (i < tm.n_data_carriers) ? tm.bits_per_carrier[i] : 0;
            if (carrier_bpc < 4) continue;
            dist_sum += min_dist(eq[i] * rot);
        }
        if (dist_sum < fine_best_metric) {
            fine_best_metric = dist_sum;
            fine_best_angle = theta;
        }
    }

    // Significance gate: BPS must improve on theta=0 by at least 5%
    // Otherwise it's just constellation geometry noise
    if (fine_best_metric >= zero_metric * 0.95f) return 0.0f;

    return fine_best_angle;
}

// CRC-8 (same polynomial as ofdm_mod.cc and native frame header)
static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// Small-N IDFT for DFT-despread OFDM (SC-FDMA).
// O(N²) — fine for N < 100 (our data carrier count).
static void small_idft(const std::complex<float>* in, std::complex<float>* out, int N) {
    float scale = 1.0f / std::sqrt((float)N);
    for (int k = 0; k < N; k++) {
        std::complex<float> sum(0.0f, 0.0f);
        for (int n = 0; n < N; n++) {
            float angle = 2.0f * (float)M_PI * (float)k * (float)n / (float)N;
            sum += in[n] * std::complex<float>(std::cos(angle), std::sin(angle));
        }
        out[k] = sum * scale;
    }
}

// (Header decode removed — config pre-negotiated, Mercury approach)

// ============================================================================
//  Interleave stride constant (must match TX)
// ============================================================================
static constexpr int INTERLEAVE_STRIDE = 41;  // coprime to 1600

// S2 tracking scope (see ofdm_demod.h). E4 extended it to 64QAM (bpc 6);
// E5 extends it DOWN to QPSK (bpc 2) — the QPSK exclusion was tuned on the
// --s2gate proxy (static echo + scalar sine fade, tests_s2gate.cc), whose
// notch never moves, so a preamble-frozen H looked free there. Under a
// walking Watterson notch the frozen H is exactly what fails the QPSK rungs
// (O0-O2), and the s2_lindet detrend already neutralizes the S1 phase-fold
// risk that motivated freezing. BPSK (bpc 1) and 256QAM+ (bpc >= 8) stay out.
// Reverting the lower bound to 4 reproduces the pre-E5 "QPSK frozen" pin.
bool ofdm_s2_track_in_scope(int max_bpc) {
    return max_bpc >= 2 && max_bpc <= 6;
}

// ============================================================================
//  OfdmDemodulator
// ============================================================================

OfdmDemodulator::OfdmDemodulator(const OfdmConfig& config)
    : config_(config)
{
}

// ----------------------------------------------------------------------------
//  extract_data_carriers: from n_used used-carrier values, write data only
//  into pre-allocated output buffer. Returns count of data carriers written.
// ----------------------------------------------------------------------------
int OfdmDemodulator::extract_data_carriers(
    const std::complex<float>* symbol_freq, int n_used,
    std::vector<std::complex<float>>& out)
{
    int count = 0;
    for (int i = 0; i < n_used; i++) {
        // Pilots are at every pilot_carrier_spacing-th position (0, 4, 8, ...)
        if (i % config_.pilot_carrier_spacing == 0) continue;
        out[count++] = symbol_freq[i];
    }
    return count;
}

// ----------------------------------------------------------------------------
//  equalize_mmse: MMSE equalization of data carriers
//
//  Data carrier index d maps to used carrier index via the pilot skip pattern.
//  We need to find the correct H[i] and noise_var[i] for each data carrier.
//  Writes into pre-allocated output buffer (must have capacity >= n_data).
// ----------------------------------------------------------------------------
void OfdmDemodulator::equalize_mmse(
    const std::vector<std::complex<float>>& data_carriers, int n_data,
    const OfdmChannelEst& est,
    std::vector<std::complex<float>>& out)
{
    const int n_used = config_.n_used_carriers;

    // Build mapping from data carrier index -> used carrier index
    int d = 0;
    for (int i = 0; i < n_used && d < n_data; i++) {
        if (i % config_.pilot_carrier_spacing == 0) continue;  // skip pilot

        std::complex<float> Y = data_carriers[d];
        std::complex<float> H = est.H[i];
        float H_mag2 = std::norm(H);
        float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 1e-6f;
        // NV floor: prevent MMSE from over-amplifying noisy carriers.
        // With DFT-spread, the EVM-based sigma_sq (computed after IDFT
        // despreading) captures actual distortion including BPF effects.
        // The per-carrier NV floor here only needs to prevent numerical
        // instability — the real noise estimation happens in dft_sigma_sq.
        // Without DFT-spread, cap at ~33 dB (0.0005) for 1024QAM headroom.
        constexpr float NV_ABS_FLOOR = 0.001f;  // -30 dB absolute
        constexpr float NV_REL_FLOOR = 0.0005f;  // max 33 dB effective SNR per carrier
        float nv_floor = std::max(NV_ABS_FLOOR, NV_REL_FLOOR * H_mag2);
        if (nv < nv_floor) nv = nv_floor;

        if (H_mag2 < 1e-4f * nv) {
            // Deep fade: zero this carrier (numerical safety only)
            out[d] = {0.0f, 0.0f};
        } else {
            // MMSE: X_hat = conj(H) * Y / (|H|^2 + noise_var)
            out[d] = std::conj(H) * Y / (H_mag2 + nv);
        }
        d++;
    }
}

// ----------------------------------------------------------------------------
//  demap_to_llrs: soft-demap equalized data carriers to LLRs
// ----------------------------------------------------------------------------
void OfdmDemodulator::demap_to_llrs(
    const std::vector<std::complex<float>>& eq_carriers, int n_data,
    const ToneMap& tone_map,
    const OfdmChannelEst& est,
    std::vector<float>& llrs,
    float& dft_sigma_sq_out,
    float& metric_sigma_sq_out)
{
    const int n_used = config_.n_used_carriers;

    // Convert LdpcRate to rate_num/16 for NUC table lookup
    int fec_r16 = fec_rate_num16(tone_map.fec_rate);

    // Pre-allocate reusable buffers for demap_soft (Issue 1: avoid per-carrier heap allocs)
    std::vector<std::complex<float>> sym_vec(1);
    std::vector<float> carrier_llrs;
    carrier_llrs.reserve(8);  // max bits per carrier (QAM256)

    // DFT-spread: after IDFT despreading, noise is averaged across all
    // subcarriers.  The IDFT is unitary, so the error covariance matrix is
    // circulant with equal diagonal elements = arithmetic mean of per-subcarrier
    // MMSE error variances.  This is the standard SC-FDMA result (Falconer et al.,
    // IEEE Comm Mag 2002; Lim et al., IEEE Trans Comm 2012).
    //
    // Use ONLY channel-estimator-based noise variance — NOT decision-directed EVM.
    // DD-EVM captures residual CPE, FM limiter distortion, and hard-decision errors
    // that inflate sigma_sq and crush LLR magnitudes.  No surveyed SC-FDMA receiver
    // (srsRAN, MATLAB LTE/5G toolbox) uses DD-EVM for LLR sigma_sq.
    float dft_sigma_sq = 1.0f;
    if (config_.dft_spread && n_data > 1) {
        // Per-carrier noise_var is the training-pair estimate
        // (ofdm_noise_from_training_pair): honest in-band sigma^2(k)
        // including any noise color. The mean of nv/(|H|^2+nv) over the
        // data carriers is 1-mu (mu = mean per-carrier MMSE gain), the
        // quantity the SC-FDMA effective-SINR scaling below consumes.
        float sigma_sum = 0.0f;
        int count = 0;
        for (int i = 0; i < n_used && count < n_data; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float H_mag2 = std::norm(est.H[i]);
            float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 0.001f;
            constexpr float NV_ABS_FLOOR = 0.001f;
            constexpr float NV_REL_FLOOR = 0.0005f;
            float nv_floor = std::max(NV_ABS_FLOOR, NV_REL_FLOOR * H_mag2);
            if (nv < nv_floor) nv = nv_floor;
            sigma_sum += (H_mag2 + nv > 1e-12f) ? (nv / (H_mag2 + nv)) : 1.0f;
            count++;
        }
        dft_sigma_sq = sigma_sum / std::max(1, count);

        // Second metric for the gearshift / effective_snr_db path (NOT for
        // LLR scaling — the per-carrier metric above stays in use for LDPC
        // soft-demap). Same MMSE form, but WITHOUT the absolute floors above:
        // NV_ABS_FLOOR=0.001 would cap the metric at ~30 dB at |H|~1.
        // Source is the honest per-carrier IN-BAND sigma^2(k) from the
        // training pair. The guard-bin scalar (est.noise_var_frame)
        // previously used here is floor-pinned on the live band-limited RX
        // (the band-limit zeroes exactly the bins it averages), which made
        // effective_snr_db read 46-49 dB regardless of the channel — the
        // meter the climb law fell back on was steering blind. (fact doc
        // data-flow-noise-var.md §8)
        float metric_sigma_sum = 0.0f;
        int metric_count = 0;
        for (int i = 0; i < n_used && metric_count < n_data; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float H_mag2 = std::norm(est.H[i]);
            float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 1e-6f;
            if (nv < 1e-9f) nv = 1e-9f;
            metric_sigma_sum += (H_mag2 + nv > 1e-12f)
                ? (nv / (H_mag2 + nv)) : 1.0f;
            metric_count++;
        }
        metric_sigma_sq_out = metric_sigma_sum / std::max(1, metric_count);
        // Per fact doc §9: on flat channels the per-carrier H-smoothness
        // residual confuses signal shape for noise (§6.5a). When
        // llr_use_frame_nv is true, swap the LLR scalar to the frame-wide
        // guard-bin AWGN value (codec2 freedv_700.c:527 uses an analogous
        // bounded scalar EsNo=3.0). Equalizer per-carrier noise_var stays
        // unchanged so deep fades still get correctly notched upstream.
        // climbgate Fix 2a: env override so one binary A/Bs the bias-free frame-nv
        // LLR scalar (IRIS_LLR_FRAME_NV=1) that unlocks 32QAM+ on flat channels —
        // the per-carrier H-smoothness residual reads ~0.46 here (grossly biased
        // high; 16QAM decodes in 1 iter) and crushes the dense-constellation
        // amplitude LLRs. -1 = unset -> honor the config default.
        static const int env_frame_nv = []() {
            const char* e = getenv("IRIS_LLR_FRAME_NV"); return e ? atoi(e) : -1; }();
        bool use_frame_nv = (env_frame_nv >= 0) ? (env_frame_nv != 0)
                                                : config_.llr_use_frame_nv;
        if (use_frame_nv) {
            dft_sigma_sq = metric_sigma_sq_out;
        }
        static int sigma_log_count = 0;
        if (sigma_log_count++ < 5)
            IRIS_LOG("[OFDM-RX] dft_sigma_sq=%.6f (count=%d, sigma_sum=%.4f, frame_nv=%d)",
                     dft_sigma_sq, count, sigma_sum, use_frame_nv ? 1 : 0);
        // Store for demodulate() to compute effective_snr_db
        dft_sigma_sq_out = dft_sigma_sq;
    }

    // SC-FDMA effective-SINR LLR scaling (MMSE FDE + IDFT despread).
    // Per carrier, the MMSE output is Z_k = mu_k d_k + v_k with
    //   mu_k = gamma_k/(1+gamma_k),  Var(v_k) = mu_k(1-mu_k),
    // gamma_k = |H_k|^2/sigma^2_k. After the unitary IDFT despread,
    //   d_hat = mu*d + e,  mu = mean(mu_k) = 1 - dft_sigma_sq,
    // and the TOTAL error (noise + residual self-interference from the
    // spread of mu_k) is exactly E|e|^2 = mean[mu_k(1-mu_k)] + var(mu_k)
    // = mu(1-mu). The post-despread effective SINR is therefore
    //   gamma_eff = mu^2 / (mu(1-mu)) = mu/(1-mu)
    // — the standard MMSE-FDE result for SC-FDMA/DFT-s-OFDM (LTE uplink
    // link-abstraction literature; e.g. Ericsson/3GPP effective-SINR mapping
    // for MMSE receivers). The demapper must see the FULL-scale grid, so
    // undo the shrink (d_hat/mu) and hand it the matching noise variance
    //   Var(e/mu) = mu(1-mu)/mu^2 = (1-mu)/mu = 1/gamma_eff.
    // The previous default skipped the un-shrink and fed sigma^2 = (1-mu)
    // against the full grid; the old opt-in correction divided by mu^2
    // (treating e as if its variance were (1-mu)) — each wrong by a factor
    // of mu at one place. Near mu=1 (clean flat channel) all three coincide,
    // which is why the opt-in was once measured as an O3 no-op at mu~0.96;
    // on colored/tilted channels mu reaches 0.7-0.9 and the difference is
    // real. Default-ON now that the per-carrier sigma^2(k) feeding mu is
    // honest (ofdm_noise_from_training_pair). IRIS_MU_AVG=0 restores the
    // legacy shrunken-symbol scaling for A/B.
    static const int env_mu_avg = []() {
        const char* e = getenv("IRIS_MU_AVG"); return e ? atoi(e) : -1; }();
    bool mu_avg_on = (env_mu_avg >= 0) ? (env_mu_avg != 0) : config_.mu_avg_correct;
    float mu_avg = 1.0f;
    if (mu_avg_on && config_.dft_spread && n_data > 1) {
        mu_avg = 1.0f - dft_sigma_sq;
        if (mu_avg < 0.05f) mu_avg = 0.05f;  // guard against div-by-~0 in deep noise
    }
    const float inv_mu = 1.0f / mu_avg;


    int data_idx = 0;

    for (int i = 0; i < n_used && data_idx < n_data; i++) {
        if (i % config_.pilot_carrier_spacing == 0) continue;  // skip pilot

        int bpc = (data_idx < tone_map.n_data_carriers)
                  ? tone_map.bits_per_carrier[data_idx] : 0;

        if (bpc == 0) {
            data_idx++;
            continue;
        }

        Modulation mod = bits_to_modulation(bpc);

        // Effective noise variance after MMSE equalization.
        // With DFT-spread: use averaged sigma_sq (IDFT mixes noise uniformly).
        // Without: per-carrier sigma_sq as before.
        float sigma_sq;
        if (config_.dft_spread) {
            sigma_sq = dft_sigma_sq;
        } else {
            // NV floors (must match equalize_mmse for consistent scaling).
            float H_mag2 = std::norm(est.H[i]);
            float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 0.001f;
            constexpr float NV_ABS_FLOOR = 0.001f;
            constexpr float NV_REL_FLOOR = 0.0005f;
            float nv_floor = std::max(NV_ABS_FLOOR, NV_REL_FLOOR * H_mag2);
            if (nv < nv_floor) nv = nv_floor;
            sigma_sq = (H_mag2 + nv > 1e-12f) ? (nv / (H_mag2 + nv)) : 1.0f;
        }

        // NUC soft demapping for 16QAM+ when enabled
        const NucTable* nuc = nullptr;
        if (tone_map.use_nuc && bpc >= 4)
            nuc = get_nuc_table(mod, fec_r16);

        // No CSI reliability weighting — MMSE equalization already accounts
        // for per-carrier channel quality via sigma_sq = nv/(|H|²+nv).
        // Adding reliability = |H|²/(|H|²+nv) on top converts MMSE LLRs
        // back to ZF LLRs, negating the MMSE advantage on weak carriers.
        // (Confirmed anti-pattern: 5G NR uses CSI *instead of* correct
        // sigma_sq, not in addition to it.)

        // Undo the MMSE-despread shrink: symbol back to the full-scale grid
        // (d_hat/mu) with the matching error variance (1-mu)/mu = sigma_sq/mu
        // (derivation above — the total post-despread error is mu(1-mu), and
        // dividing the symbol by mu divides the variance by mu^2). inv_mu==1
        // when correction disabled or not DFT-spread (no-op).
        std::complex<float> sym_corr = eq_carriers[data_idx] * inv_mu;
        float sigma_sq_corr = sigma_sq * inv_mu;

        if (nuc) {
            // NUC max-log-MAP soft demapper
            size_t llr_start = llrs.size();
            llrs.resize(llr_start + bpc);
            demap_soft_nuc(sym_corr, sigma_sq_corr, nuc, &llrs[llr_start]);
        } else {
            // Standard uniform QAM soft demapper (reuse pre-allocated buffers)
            sym_vec[0] = sym_corr;
            carrier_llrs = demap_soft(sym_vec, mod, sigma_sq_corr);
            llrs.insert(llrs.end(), carrier_llrs.begin(), carrier_llrs.end());
        }
        data_idx++;
    }
}

// ============================================================================
//  demodulate: main entry point
// ============================================================================
OfdmDemodResult OfdmDemodulator::demodulate(
    const std::complex<float>* iq, int n_samples,
    const ToneMap& tone_map, const OfdmSyncResult* pre_sync,
    const std::vector<std::complex<float>>* genie_H)
{
    OfdmSyncResult principal_sync = pre_sync
        ? *pre_sync
        : ofdm_detect_frame(iq, n_samples, config_, &sync_workspace_);
    if (!principal_sync.detected) {
        IRIS_LOG("[OFDM-RX] no frame detected");
        return {};
    }

    if (principal_sync.cfo_hypothesis_selected) {
        auto result = demodulate_selected(iq, n_samples, tone_map,
                                          principal_sync, genie_H);
        if (!result.channel_estimate.H.empty())
            channel_est_ = result.channel_estimate;
        return result;
    }

    // Component/offline callers do not own RC7's persistent scheduler.  Give
    // them the same complete finite ambiguity search synchronously.  Live RX
    // selects one entry before calling us and persists its CFO cursor instead.
    auto hypotheses = ofdm_cfo_hypotheses(iq, n_samples, config_, principal_sync);
    OfdmDemodResult first_failure;
    bool have_failure = false;
    for (const auto& unresolved_hypothesis : hypotheses) {
        const auto hypothesis = ofdm_refine_cfo_hypothesis(
            iq, n_samples, config_, unresolved_hypothesis);
        IRIS_LOG("[OFDM-CFO] trial m=%d total=%.3f Hz timing=%d ref=%.3f",
                 hypothesis.cfo_ambiguity_index, hypothesis.cfo_hz,
                 hypothesis.frame_start, hypothesis.cfo_training_metric);
        auto trial = demodulate_selected(iq, n_samples, tone_map, hypothesis,
                                         genie_H);
        if (trial.success && trial.cfo_resolved) {
            if (!trial.channel_estimate.H.empty())
                channel_est_ = trial.channel_estimate;
            return trial;
        }
        if (!have_failure) {
            first_failure = std::move(trial);
            have_failure = true;
        }
    }
    if (have_failure) {
        if (!first_failure.channel_estimate.H.empty())
            channel_est_ = first_failure.channel_estimate;
        return first_failure;
    }
    return {};
}

OfdmDemodResult OfdmDemodulator::demodulate_trial(
    const std::complex<float>* iq, int n_samples, const ToneMap& tone_map,
    const OfdmSyncResult& selected_sync)
{
    return demodulate_selected(iq, n_samples, tone_map, selected_sync, nullptr);
}

OfdmDemodResult OfdmDemodulator::demodulate_selected(
    const std::complex<float>* iq, int n_samples,
    const ToneMap& tone_map, const OfdmSyncResult& selected_sync,
    const std::vector<std::complex<float>>* genie_H)
{
    OfdmDemodResult result;
    // These are intentionally local shadows of the historic persistent caches.
    // A hypothesis may refine H on every pilot row, but none of that state is
    // visible outside the returned result until the one post-search commit.
    OfdmChannelEst channel_est_;
    float last_dft_sigma_sq_ = 1.0f;
    float last_metric_sigma_sq_ = 1.0f;
    auto finish_result = [&]() -> OfdmDemodResult {
        result.channel_estimate = channel_est_;
        return std::move(result);
    };
    const int nfft = config_.nfft;
    const int cp = config_.cp_samples;
    const int sym_len = nfft + cp;
    const int n_used = config_.n_used_carriers;
    const int n_data = config_.n_data_carriers;

    auto geometry = checked_ofdm_frame_geometry(config_, tone_map);
    if (!geometry) {
        IRIS_LOG("[OFDM-RX] rejected invalid frame geometry");
        return result;
    }
    const int n_codewords = geometry->codeword_count();
    const int coded_bits_total = static_cast<int>(
        v2::FrameGeometry::kCodedBitsPerCodeword * geometry->codeword_count());
    const int n_data_symbols = static_cast<int>(geometry->data_symbol_count());
    result.fec_rate = geometry->fec();
    result.n_ldpc_blocks = n_codewords;
    result.n_data_symbols = n_data_symbols;
    result.demodulated_level = tone_map.tone_map_id > 0
        ? static_cast<int>(tone_map.tone_map_id) - 1 : -1;
    result.demodulated_n_codewords = n_codewords;

    // ---- 1. Selected unresolved CFO hypothesis ----
    const OfdmSyncResult sync = selected_sync;
    if (!sync.detected || !sync.cfo_hypothesis_selected) return result;

    result.snr_db = sync.snr_est;
    result.effective_snr_db = sync.snr_est;  // default; overridden by dft_sigma_sq for DFT-spread
    result.cfo_hz = sync.cfo_hz;
    result.pair_coherence = sync.pair_coherence > 0.0f
        ? sync.pair_coherence : sync.zc_metric;
    result.estimator_validity =
        OfdmEstimatorValidity::CoherentPairUnresolved;

    int frame_start = sync.frame_start;  // Start of CP of training symbol 1

    // ---- 1b. Complete candidate extent ----
    // frame_start is the first training symbol, so the leading symbol is not
    // part of this input-relative requirement.  Bound scratch to precisely the
    // checked candidate; following frames never enter CFO/FFT work.
    const std::uint64_t required_u64 = ofdm_samples_from_first_training(*geometry);
    if (required_u64 > static_cast<std::uint64_t>(INT_MAX)) return result;
    const int required_samples = static_cast<int>(required_u64);
    int remaining = n_samples - frame_start;
    if (remaining < required_samples) {
        IRIS_LOG("[OFDM-RX] incomplete candidate: %d/%d samples",
                 std::max(0, remaining), required_samples);
        // Don't consume samples — leave preamble in buffer for retry with more data.
        result.completion = OfdmDemodResult::Completion::NeedMoreSamples;
        result.consumed_from_input_start = 0;
        result.additional_samples_required = static_cast<std::uint64_t>(
            required_samples - std::max(0, remaining));
        return result;
    }
    remaining = required_samples;

    // ---- 2. CFO correction ----
    if (remaining <= 0) {
        IRIS_LOG("[OFDM-RX] frame_start beyond buffer");
        return result;
    }

    std::vector<std::complex<float>> iq_corrected(remaining);
    std::copy(iq + frame_start, iq + frame_start + remaining, iq_corrected.data());
    ofdm_correct_cfo(iq_corrected.data(), remaining, sync.cfo_hz, config_.sample_rate);

    // ---- 3. FFT training symbol 1 (for differential-phase CFO) ----
    int pos = 0;
    if (pos + sym_len > remaining) {
        IRIS_LOG("[OFDM-RX] insufficient samples for training symbol 1");
        return result;
    }
    const std::complex<float>* train1_body = iq_corrected.data() + pos + cp;
    std::vector<std::complex<float>> Y1(nfft);
    std::copy(train1_body, train1_body + nfft, Y1.begin());
    fft_complex(Y1.data(), nfft);
    pos += sym_len;  // advance past training symbol 1

    // ---- 4. Channel estimation from training symbol 2 ----
    if (pos + sym_len > remaining) {
        IRIS_LOG("[OFDM-RX] insufficient samples for training symbol 2");
        return result;
    }
    // Skip CP of training symbol 2, then take nfft samples for channel est
    const std::complex<float>* train2_body = iq_corrected.data() + pos + cp;
    channel_est_ = ofdm_estimate_channel(train2_body, config_);
    // Compensate preamble boost: TX applies sqrt(2) to training symbols,
    // so H[k] is overestimated by sqrt(2). Divide out to get true channel.
    // noise_var is NOT divided: it is replaced below (step 4c) by the
    // training-pair estimate, which is true channel noise — boost scales the
    // signal bins of the training symbols, never the noise. (The old code
    // divided the guard-bin floor by boost^2 on the same wrong premise,
    // handing MMSE/LLR a sigma^2 that was 3 dB optimistic.)
    constexpr float PREAMBLE_BOOST = 1.4142f;
    for (auto& h : channel_est_.H)
        h /= PREAMBLE_BOOST;
    pos += sym_len;  // advance past training symbol 2

    // ---- 4b. Fine CFO: frequency-domain correlation between training symbols ----
    // Both training symbols are identical ZC sequences. After coarse CFO correction,
    // the residual CFO appears as a common phase rotation:
    //   Y2[k]*conj(Y1[k]) = |H[k]|²*|ZC[k]|² * exp(j*2π*Δf*T)
    // where T = symbol_len / sample_rate.
    // FFT train2 (we already have Y1 from step 3).
    std::vector<std::complex<float>> Y2(nfft);
    {
        const std::complex<float>* train2_body_ptr = iq_corrected.data()
            + (sym_len)   // skip training symbol 1
            + cp;         // skip CP of training symbol 2
        std::copy(train2_body_ptr, train2_body_ptr + nfft, Y2.begin());
        fft_complex(Y2.data(), nfft);
    }

    std::complex<float> diff_corr(0, 0);
    for (int i = 0; i < n_used; i++) {
        int bin = config_.used_carrier_bins[i];
        diff_corr += Y2[bin] * std::conj(Y1[bin]);
    }
    float diff_phase = std::arg(diff_corr);
    float fine_cfo = diff_phase * config_.sample_rate / (2.0f * (float)M_PI * sym_len);
    IRIS_LOG("[OFDM-RX] fine CFO: diff_phase=%.4f rad -> %.2f Hz", diff_phase, fine_cfo);
    if (std::abs(fine_cfo) > 0.1f) {
        // Apply fine CFO correction to remaining samples
        float total_cfo = sync.cfo_hz + fine_cfo;
        result.cfo_hz = total_cfo;

        // Re-correct from frame start with total CFO
        std::copy(iq + frame_start, iq + frame_start + remaining, iq_corrected.data());
        ofdm_correct_cfo(iq_corrected.data(), remaining, total_cfo, config_.sample_rate);

        // Re-estimate channel with fine-corrected signal
        const std::complex<float>* train2_refined = iq_corrected.data()
            + (sym_len)  // skip training symbol 1
            + cp;        // skip CP of training symbol 2
        channel_est_ = ofdm_estimate_channel(train2_refined, config_);
        // Compensate preamble boost again after re-estimation (H only; see
        // the note at the first boost compensation).
        for (auto& h : channel_est_.H)
            h /= PREAMBLE_BOOST;

        IRIS_LOG("[OFDM-RX] CFO refined: coarse=%.1f Hz + fine=%.2f Hz = %.2f Hz",
                 sync.cfo_hz, fine_cfo, total_cfo);
    }

    // ---- 4c. Per-carrier noise variance from the training pair ----
    // Replace the guard-bin scalar broadcast with the honest in-band per-
    // carrier sigma^2(k) measured from the two identical training symbols
    // (ofdm_noise_from_training_pair). This is what makes the receiver
    // noise-color-agnostic: triangular FM discriminator noise, de-emphasis
    // shaping and any static amplitude tilt reach the MMSE weights and the
    // LLR scalar as measured reliability instead of a flat assumption.
    // Y1/Y2 are refilled from iq_corrected so both reflect the FINAL CFO
    // correction regardless of whether the fine-CFO branch re-ran.
    {
        std::copy(iq_corrected.data() + cp,
                  iq_corrected.data() + cp + nfft, Y1.begin());
        std::copy(iq_corrected.data() + sym_len + cp,
                  iq_corrected.data() + sym_len + cp + nfft, Y2.begin());
        fft_complex(Y1.data(), nfft);
        fft_complex(Y2.data(), nfft);
        channel_est_.noise_var =
            ofdm_noise_from_training_pair(Y1.data(), Y2.data(), config_);

        // Rebuild the derived SNR view on the HONEST absolute scale:
        //   snr_per_carrier[k] = |H[k]|^2 / sigma^2(k)
        // with H at data-symbol scale (preamble boost divided out above) and
        // sigma^2(k) the measured IN-BAND training-pair noise. This is THE
        // meter: mean_snr_db feeds the gearshift climb law
        // (mean_channel_snr_db, modem.cc IRIS_CLIMB_CHSNR default-ON), and
        // the "[OFDM-CE] channel:" line below is what the instrumentation
        // gate (bench_selftest G1) asserts against injected ground truth.
        //
        // History: this view previously multiplied by BOOST_SQ=2 to preserve
        // the legacy guard-bin reporting numbers (+3.0 dB vs truth on white
        // noise). The guard-bin scalar itself is BLIND on the live RX path —
        // the band-limit (modem.cc) zeroes exactly the out-of-band bins it
        // averages, pinning it at the used-carrier leakage floor (~0.08), so
        // the reported SNR sat at 46-49 dB no matter what the channel did
        // (measured residuals +11.5..+26.4 dB vs injected truth). A rate-
        // adaptation law needs an absolute in-band meter; the O-ladder
        // min_snr_db thresholds are true-SNR values, so no table shift
        // accompanies the scale correction. (fact doc data-flow-noise-var.md §8)
        float snr_sum_db = 0.0f, min_db = 1e9f, max_db = -1e9f;
        float mean_H_mag = 0.0f;
        for (int i = 0; i < n_used; i++) {
            float nv = channel_est_.noise_var[i];
            channel_est_.snr_per_carrier[i] =
                std::norm(channel_est_.H[i]) / std::max(nv, 1e-12f);
            float db = 10.0f * std::log10(
                std::max(channel_est_.snr_per_carrier[i], 1e-10f));
            snr_sum_db += db;
            min_db = std::min(min_db, db);
            max_db = std::max(max_db, db);
            mean_H_mag += std::abs(channel_est_.H[i]);
        }
        channel_est_.mean_snr_db = snr_sum_db / std::max(1, n_used);
        mean_H_mag /= std::max(1, n_used);
        // Canonical per-frame meter line (calibration-visible; one per frame,
        // emitted after the fine-CFO branch settles).
        IRIS_LOG("[OFDM-CE] channel: mean|H|=%.3f, SNR range %.1f-%.1f dB, mean=%.1f dB, %d carriers",
                 mean_H_mag, min_db, max_db, channel_est_.mean_snr_db, n_used);
        IRIS_LOG("[OFDM-NV] training-pair noise: nv[0]=%.3g nv[mid]=%.3g nv[last]=%.3g "
                 "(guard-bin floor %.3g, diagnostic only)",
                 channel_est_.noise_var[0], channel_est_.noise_var[n_used / 2],
                 channel_est_.noise_var[n_used - 1], channel_est_.noise_var_frame);
    }

    result.mean_channel_snr_db = channel_est_.mean_snr_db;

    // ---- 4c. Sync word verification (CRC-8 structural check) ----
    // TX places LFSR BPSK on first (n_used-8) carriers + CRC-8 on last 8.
    // RX equalizes, hard-decides all carriers, recomputes CRC-8 over the
    // data bits, and compares to the received CRC bits.  False triggers
    // produce random bits → CRC match probability = 1/256.  Real frames
    // at decodable SNR → CRC always matches.  No threshold.
    if (pos + sym_len <= remaining) {
        std::vector<std::complex<float>> sw_fft(nfft);
        const std::complex<float>* sw_body = iq_corrected.data() + pos + cp;
        std::copy(sw_body, sw_body + nfft, sw_fft.data());
        fft_complex(sw_fft.data(), nfft);

        // Equalize all carriers and hard-decide BPSK bits
        std::vector<uint8_t> rx_bits(n_used, 0);
        int valid_carriers = 0;
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            std::complex<float> H = channel_est_.H[i];
            float H_mag2 = std::norm(H);
            if (H_mag2 < 1e-12f) continue;
            std::complex<float> eq = sw_fft[bin] * std::conj(H) / H_mag2;
            rx_bits[i] = (eq.real() >= 0.0f) ? 1 : 0;
            valid_carriers++;
        }

        // Split into data bits and received CRC bits
        int n_data_bits = n_used - 8;
        if (n_data_bits < 2 || valid_carriers < n_used / 2) {
            // Too few carriers to check — reject
            IRIS_LOG("[OFDM-RX] sync word REJECTED: too few carriers (%d valid, %d needed)",
                     valid_carriers, n_used);
            result.consumed_from_input_start = frame_start + 3 * sym_len;
            return finish_result();
        }

        // Compute CRC-8 over data bits (same packing as TX)
        int n_bytes = (n_data_bits + 7) / 8;
        std::vector<uint8_t> packed(n_bytes, 0);
        for (int i = 0; i < n_data_bits; i++)
            packed[i / 8] |= (rx_bits[i] << (i % 8));
        uint8_t computed_crc = crc8(packed.data(), n_bytes);

        // Extract received CRC from last 8 carriers
        uint8_t received_crc = 0;
        for (int i = 0; i < 8; i++)
            received_crc |= (rx_bits[n_data_bits + i] << i);

        if (computed_crc == received_crc) {
            result.sync_agreement = true;
            IRIS_LOG("[OFDM-RX] sync word OK: CRC-8 match (0x%02X, %d carriers)",
                     computed_crc, valid_carriers);
        } else {
            // CRC mismatch — sync word BPSK is unreliable OTA.
            // FM deviation limiter + phase noise corrupt 1-6 bits even at
            // SC=0.96, SNR=16 dB.  At mean channel SNR 2-4 dB, CRC-8 never
            // passes.  Always proceed to LDPC — CRC-32 is the real arbiter.
            // Cost: false detections waste one LDPC attempt (~ms), but LDPC
            // early-terminates on garbage and CRC-32 catches 100%.
            IRIS_LOG("[OFDM-RX] sync word CRC mismatch (got 0x%02X, expected 0x%02X) "
                     "SC=%.3f FD-ZC=%.3f — proceeding to LDPC (CRC-32 arbiter)",
                     received_crc, computed_crc, sync.sc_metric, sync.zc_metric);
        }
        pos += sym_len;
    }

    // ---- 5. Scale-free channel validity gate ----
    // Absolute |H| depends on capture gain, so it cannot decide admission.
    // Detection already used normalized repetition metrics; here reject only
    // non-finite/degenerate estimates.  LDPC plus every codeword CRC remains
    // the payload authority.
    {
        float h_sum = 0.0f;
        float noise_sum = 0.0f;
        bool finite = !channel_est_.H.empty() &&
                      channel_est_.noise_var.size() == channel_est_.H.size();
        for (size_t i = 0; i < channel_est_.H.size(); ++i) {
            const auto& h = channel_est_.H[i];
            const float nv = channel_est_.noise_var[i];
            finite = finite && std::isfinite(h.real()) && std::isfinite(h.imag()) &&
                     std::isfinite(nv) && nv >= 0.0f;
            h_sum += std::abs(h);
            noise_sum += nv;
        }
        float mean_H = channel_est_.H.empty() ? 0.0f : h_sum / channel_est_.H.size();
        result.mean_H_mag = mean_H;
        const float normalized_energy = h_sum * h_sum /
            std::max(noise_sum * std::max<size_t>(1, channel_est_.H.size()), 1e-30f);
        if (!finite || !(h_sum > 0.0f) || !std::isfinite(normalized_energy)) {
            IRIS_LOG("[OFDM-RX] channel validity gate rejected non-finite/degenerate estimate");
            return finish_result();
        }
    }

    // ---- 6. Use pre-negotiated tone map (no header — Mercury approach) ----
    const ToneMap& active_tone_map = tone_map;
    result.fec_rate = geometry->fec();

    int bps_total = active_tone_map.total_bits_per_symbol;
    if (bps_total <= 0) {
        IRIS_LOG("[OFDM-RX] tone map has 0 bits per symbol");
        return finish_result();
    }

    IRIS_LOG("[OFDM-RX] expecting %d data symbols (%d coded bits, %d bits/sym, %d LDPC block(s))",
             n_data_symbols, coded_bits_total, bps_total, n_codewords);
    IRIS_LOG("[OFDM-RX] frame config: nfft=%d cp=%d sym=%d pilots=%d/%d(spacing=%d) "
             "pilot_row_spacing=%d dft_spread=%d",
             nfft, cp, sym_len, n_used - n_data, n_used,
             config_.pilot_carrier_spacing,
             config_.pilot_row_spacing, config_.dft_spread ? 1 : 0);

    // ---- 8. Receive data symbols (with block pilots) ----
    std::vector<float> all_llrs;
    all_llrs.reserve(coded_bits_total);

    int data_sym_count = 0;  // count of data symbols received so far

    int total_symbols_elapsed = 0;        // total symbols since training (incl. header)

    // ================================================================
    // 3-State Kalman Phase Tracker + BPS (Blind Phase Search)
    //
    // Replaces the old linear-prediction + comb-CPE + DD-CPE chain.
    // H stays immutable after preamble calibration — all phase drift
    // is tracked by Kalman [phase, freq, accel].
    //
    // Measurements:
    //   - Pilot rows (every 8 data sym): all carriers, ~6° RMS, r=0.01
    //   - Block pilots (every 24 data sym): all carriers, ~3° RMS, r=0.01
    //   - Comb pilots (every sym): ~14 carriers (NBFM), ~17° RMS, r=0.09
    //   - BPS (QAM16+, every sym): all data carriers, r=0.05
    //
    // After all symbols: RTS backward smoother, then re-correct stored
    // eq carriers and soft-demap.
    //
    // References: frame.cc 3-state Kalman (production SC-FDMA),
    //   Pfau et al. 2009 (BPS), S-BPS IEEE PTL 2014.
    // ================================================================

    struct OfdmKalmanState {
        float phase = 0, freq = 0, accel = 0;
        float P00 = 0.05f, P01 = 0, P02 = 0;
        float P11 = 1e-4f, P12 = 0;
        float P22 = 1e-7f;
    };

    // Process noise — scaled proportionally to symbol period.
    // Reference: NFFT=512, CP=32 → 544/48000 = 11.33ms.
    // For a Wiener frequency process with σ_f Hz/√s:
    //   Var[Δω] = (2π·σ_f)² · T_sym³  (freq state in rad/sym units)
    // At σ_f=3 Hz/√s, NFFT=1024: Var[Δω] = 4.1e-3 rad²
    // q_freq must match this to keep the Kalman responsive to drift.
    // Phase: q_phase ~ σ_f²·T³/3 ≈ 1.4e-3 at NFFT=1024.
    const float sym_t = config_.symbol_duration_s();
    const float ref_t = 544.0f / 48000.0f;  // reference period (NFFT=512, CP=32)
    const float t_ratio = sym_t / ref_t;
    const float q_phase = 1e-3f * std::sqrt(t_ratio);   // rad²/symbol
    const float q_freq  = 2e-3f * t_ratio;              // rad²/symbol³ (matches 3 Hz/√s drift)
    const float q_accel = 5e-6f * t_ratio * std::sqrt(t_ratio);  // rad²/symbol⁵
    // Measurement noise
    const float r_pilot = 0.01f;   // pilot row / block pilot (~6° RMS)
    const float r_comb  = 0.09f;   // comb pilot CPE (~17° RMS)
    const float r_bps   = 0.05f;   // BPS estimate (QAM16+)
    // STF (Strong Tracking Filter)
    const float stf_rho = 0.90f;   // forgetting factor
    const float stf_max = 3.0f;    // max fading factor

    // FEC rate for NUC table lookup (used by BPS)
    auto fec_to_r16 = [&]() -> int {
        switch (active_tone_map.fec_rate) {
            case LdpcRate::RATE_1_2: return 8;
            case LdpcRate::RATE_5_8: return 10;
            case LdpcRate::RATE_3_4: return 12;
            case LdpcRate::RATE_7_8: return 14;
            default: return 8;
        }
    };
    int fec_r16 = fec_to_r16();

    // Max bits-per-carrier for this tone map (determines BPS vs comb-only)
    int max_bpc = 1;
    for (int k = 0; k < active_tone_map.n_data_carriers; k++)
        if (active_tone_map.bits_per_carrier[k] > max_bpc)
            max_bpc = active_tone_map.bits_per_carrier[k];
    bool use_bps = (max_bpc >= 4);  // BPS for QAM16+

    // Forward Kalman state per data symbol
    std::vector<OfdmKalmanState> kalman_fwd(n_data_symbols);
    std::vector<float> kalman_fwd_lambda(n_data_symbols, 1.0f);
    float stf_Vk = r_comb + q_phase;  // innovation variance tracker
    float max_lambda = 1.0f;
    int gated_count = 0;

    // Store equalized carriers per symbol for RTS re-correction pass
    std::vector<std::vector<std::complex<float>>> stored_eq(n_data_symbols);
    std::vector<int> stored_n_data(n_data_symbols, 0);
    std::vector<float> stored_fwd_phase(n_data_symbols, 0.0f);
    std::vector<int> sym_positions(n_data_symbols, 0);  // byte offset of each data symbol in iq_corrected

    // ================================================================
    // S2 — RX-only continuous per-carrier channel tracking (estimator/EQ).
    //
    // S1 (commit fbedb37) FROZE H at the preamble estimate: it removed the old
    // pilot-driven per-carrier H re-estimate because that update folded an
    // accumulating per-carrier phase residual into H and broke long QPSK/BPSK
    // frames (test_ofdm_longframe_phase). The residual is NOT the common phase
    // (that is already removed by the CPE Kalman) — it is a small per-carrier
    // LINEAR phase (timing/SFO drift + tdf transient) that scalar-CPE removal
    // does NOT reach. S2 re-adds tracking DRIFT-FREE: it derotates the pilot-row
    // observation by the CPE AND removes the residual linear-phase slope (the
    // same weighted regression the comb-pilot path already runs on data symbols)
    // before blending into H — so a pure timing drift produces zero residual and
    // H does not move, while genuine multipath (non-linear in frequency) and
    // flat-fade (amplitude) ARE tracked.
    //
    // Scope: QPSK through 64QAM (max_bpc 2-6, see ofdm_s2_track_in_scope).
    // Measured on this modem (--s2gate, N=32; E5 QPSK numbers from the offline
    // MPG replay harness, N=40):
    //   - 16QAM (O3/O4/O5) SHEDS hard under benign fading (MPG40 40-69% frozen)
    //     AND uses BPS (use_bps = max_bpc>=4), which re-zeros the data-symbol
    //     constellation phase every symbol — so it is IMMUNE to any residual
    //     per-carrier phase the H tracking leaves in H. Tracking recovers it to
    //     ~100% with no regression.
    //   - 32QAM (O6, max_bpc==5, the VARA FM narrow top gear) is bpc==5 so it
    //     ALSO uses BPS (use_bps = max_bpc>=4) — the same per-symbol blind phase
    //     search re-zeros its constellation phase every symbol, making it immune
    //     to the S1 per-carrier phase fold in exactly the way 16QAM is. It needs
    //     the same multipath/flat-fade H tracking (interior FM EVM ~40 dB >> the
    //     ~20 dB 32QAM r5/8 needs, so the shed — not the floor — is what would
    //     drop it). Included so 32QAM tracks fading, not just the static channel.
    //   - QPSK (O0/O1/O2, max_bpc==2, E5) is now IN scope. The old exclusion
    //     rationale ("QPSK does not shed under fading, MPG40/30 = 100% frozen")
    //     was measured on the --s2gate proxy, which is a STATIC echo plus a
    //     scalar sine fade (tests_s2gate.cc) — its spectral notch never moves,
    //     so a preamble-frozen H was free there BY CONSTRUCTION. Under a
    //     walking Watterson notch (the faithful MPG relay) the frozen H is the
    //     decode failure that pins the ladder at O0-O1: the whole frame is
    //     equalized against a channel snapshot the fade has since walked away
    //     from. The S1 phase-fold risk that motivated freezing is handled by
    //     the s2_lindet detrend (test_ofdm_longframe_phase passes with QPSK in
    //     scope), so QPSK gets the same drift-free row update. Tracking also
    //     WINS on AWGN at threshold SNR: the 2-symbol preamble H estimate is
    //     the error floor there, and the pilot-row EMA integrates more pilot
    //     energy (O2 wgn:13 18/40 -> 40/40, wgn:12 0/40 -> 15/40, N=40 paired
    //     seeds) — so the scope is unguarded, no channel-class switch.
    //   - BPSK (max_bpc==1) stays frozen: no per-symbol blind loop, and its
    //     robust-rung role favors the simplest estimator.
    //   - 64QAM (O7/O8, max_bpc==6, E4) is now IN scope. It also uses BPS
    //     (use_bps = max_bpc>=4), so — exactly like 16/32QAM — the per-symbol
    //     blind phase search re-zeros its constellation phase every symbol and it
    //     is immune to the S1 per-carrier phase fold. It rides a multi-frame
    //     burst at O7, so a preamble-frozen H (no per-symbol amplitude refresh)
    //     sheds it across the burst; S2's drift-free row update + decision-FREE
    //     comb-amplitude refine (s2_comb_gain uses the known +1 comb pilots, not
    //     data decisions, so it is sound at any constellation order) track it.
    //     Without this, a preamble-frozen H is the estimator limit that keeps
    //     64QAM from riding a multi-frame burst — independent of the channel's
    //     own EVM floor.
    //   - 256QAM+ (max_bpc>=8) is left frozen — its tiny decision regions cannot
    //     absorb per-carrier EMA noise, and it is not yet on the FM ladder's
    //     supported burst path.
    //
    // Toggles are read per-call from the environment so the --s2gate harness can
    // A/B all configs on ONE binary against IDENTICAL audio (decode-only).
    // Defaults are ON: the fix ships default-ON per the "no proven fix left
    // default-off" discipline.
    auto s2_env_on = [](const char* name, bool dflt) -> bool {
        const char* e = getenv(name);
        return e ? (e[0] == '1') : dflt;
    };
    const bool s2_scope = ofdm_s2_track_in_scope(max_bpc);  // QPSK-64QAM (E5: bpc 2-6)
    const bool s2_e1  = s2_scope && s2_env_on("IRIS_S2_E1", true);   // per-carrier H update at pilot rows
    const bool s2_e2  = s2_scope && s2_env_on("IRIS_S2_E2", true);   // frequency smoothing of the pilot-row obs
    const bool s2_e3  = s2_scope && s2_env_on("IRIS_S2_E3", true);   // per-symbol comb-pilot amplitude refine
    // Linear-phase detrend of the pilot-row observation before the H EMA. This
    // is the anti-S1-fold mechanism; leaving it OFF reproduces the S1 QPSK break
    // (kept as a hidden A/B knob, default ON).
    const bool s2_lindet = s2_env_on("IRIS_S2_LINDET", true);
    // Pilot-row EMA weight. 0.25 measured best (--s2gate alpha sweep): a gentle
    // per-carrier re-center that does NOT inject pilot noise into H on a static
    // channel (WGN30 stays ~100 vs 93.8% at 0.5), while E3's per-symbol comb
    // amplitude does the fast flat-fade tracking (O5 MPG30 25%->100%).
    float s2_alpha = 0.25f;
    if (const char* a = getenv("IRIS_S2_ALPHA")) { float v = (float)atof(a); if (v > 0.0f && v <= 1.0f) s2_alpha = v; }

    // Per-symbol CAUSAL H snapshot: H as it stood when data symbol k was
    // equalized in the forward pass. The soft-demap LLR sigma weighting reads
    // this (not the final, end-of-frame H) so early symbols keep the H that was
    // current for them — the cross-layer invariant that makes live H tracking
    // safe with the deferred soft-demap.
    std::vector<std::vector<std::complex<float>>> H_snap(s2_e1 ? n_data_symbols : 0);

    OfdmKalmanState ks;  // running Kalman state


    // Pre-allocate reusable buffers for the hot loop (avoid per-symbol heap allocs)
    std::vector<std::complex<float>> fft_buf(nfft);          // Issue 2+3: shared FFT buffer
    std::vector<std::complex<float>> used_carriers_buf(n_used);
    std::vector<std::complex<float>> data_carriers_buf(n_data);
    std::vector<std::complex<float>> eq_carriers_buf(n_data);

    // ---- 8a. Preamble-to-data phase calibration ----
    // FM radios introduce a phase discontinuity between the ZC preamble and
    // OFDM data symbols (different spectral content → different group delay,
    // limiter transient, discriminator settling). The magnitude varies by radio:
    // FT-510 data port ~10°, IC-705 mic/speaker ~130°, Baofeng unknown.
    // Fix: FFT the first data symbol, use ALL carriers (pilots + data, BPSK
    // hard-slice) to estimate the phase offset, and rotate H to compensate.
    // This makes the channel estimate's phase valid for data symbols, not just
    // for the preamble that created it. Universal — works on any radio.
    if (pos + sym_len <= remaining) {
        const std::complex<float>* first_sym = iq_corrected.data() + pos + cp;
        std::copy(first_sym, first_sym + nfft, fft_buf.data());
        fft_complex(fft_buf.data(), nfft);

        // Extract used carriers and equalize with current H
        for (int i = 0; i < n_used; i++)
            used_carriers_buf[i] = fft_buf[config_.used_carrier_bins[i]];

        // Phase estimate using PILOT carriers only (known reference values).
        // Data carriers have unknown modulated phases from DFT-spreading that
        // corrupt the weighted average, especially when the true offset is
        // large (>20° from residual CFO). Pilot carriers have known phase
        // so arg(Y_pilot * conj(H)) gives the true common phase offset.
        float cal_num = 0.0f, cal_den = 0.0f;
        for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
            std::complex<float> H = channel_est_.H[i];
            float H_mag2 = std::norm(H);
            float nv = (i < (int)channel_est_.noise_var.size())
                       ? channel_est_.noise_var[i] : 1e-6f;
            if (nv < 1e-12f) nv = 1e-12f;
            float w = H_mag2 / nv;
            if (w < 1e-6f) continue;
            float phase = std::arg(used_carriers_buf[i] * std::conj(H));
            cal_num += w * phase;
            cal_den += w;
        }
        if (cal_den > 0.0f) {
            float cal_phase = cal_num / cal_den;
            // Rotate channel estimate to align with data symbols
            std::complex<float> cal_rot(std::cos(cal_phase), std::sin(cal_phase));
            for (auto& h : channel_est_.H)
                h *= cal_rot;
            IRIS_LOG("[OFDM-RX] preamble-data phase cal: %.1f deg (applied to H)",
                     cal_phase * 180.0f / (float)M_PI);
        }
        // Don't advance pos — first data symbol will be processed normally below.
    }

    // ---- 8b. GENIE-H override (offline diagnostic control only) ----
    // Replace the preamble-derived per-carrier channel estimate with the TRUE
    // channel (captured from a noiseless same-seed companion pass) so the fork
    // can isolate channel-ESTIMATION error from the EQ/diversity/FEC. Overrides
    // AFTER the phase cal so the true-H vector (which carries the clean-pass
    // phase cal) fully replaces the noisy estimate. Everything downstream
    // (Kalman phase tracking, MMSE-EQ, soft-demap, LDPC) is UNCHANGED. The
    // per-carrier noise_var is deliberately LEFT as the noisy-pass estimate
    // (the realistic noise the receiver actually faces) unless IRIS_GENIE_NVFRAME
    // broadcasts the unbiased guard-bin AWGN floor to all carriers.
    if (genie_H != nullptr && (int)genie_H->size() == n_used) {
        channel_est_.H = *genie_H;
        static const bool genie_nvframe = [](){
            const char* e = getenv("IRIS_GENIE_NVFRAME"); return e && e[0] == '1'; }();
        if (genie_nvframe) {
            for (auto& nv : channel_est_.noise_var)
                nv = channel_est_.noise_var_frame;
        }
        IRIS_LOG("[OFDM-RX] GENIE-H: channel estimate OVERRIDDEN with true channel "
                 "(%d carriers, nvframe=%d)", n_used, genie_nvframe ? 1 : 0);
    }

    // ================================================================
    // Kalman helper: predict step (3-state, decoupled STF)
    // ================================================================
    auto kalman_predict = [&](OfdmKalmanState& s, float& lambda_out) {
        s.phase += s.freq + 0.5f * s.accel;
        s.freq  += s.accel;
        float p00=s.P00, p01=s.P01, p02=s.P02;
        float p11=s.P11, p12=s.P12, p22=s.P22;
        // A*P*A' (3-state transition matrix)
        float a00 = p00 + 2*p01 + p02 + p11 + p12 + 0.25f*p22;
        float a01 = p01 + p02 + p11 + 1.5f*p12 + 0.5f*p22;
        float a02 = p02 + p12 + 0.5f*p22;
        float a11 = p11 + 2*p12 + p22;
        float a12 = p12 + p22;
        float a22 = p22;
        // STF fading factor
        float Nk = stf_Vk - r_comb;
        float lambda = (Nk > a00 && a00 > 1e-20f) ? (Nk / a00) : 1.0f;
        if (lambda > stf_max) lambda = stf_max;
        if (lambda > max_lambda) max_lambda = lambda;
        lambda_out = lambda;
        // Decoupled: lambda on phase/freq only, not accel
        s.P00 = lambda*a00 + q_phase;
        s.P01 = lambda*a01;
        s.P02 = a02;
        s.P11 = lambda*a11 + q_freq;
        s.P12 = a12;
        s.P22 = a22 + q_accel;
    };

    // ================================================================
    // Kalman helper: measurement update (Joseph form)
    // ================================================================
    auto kalman_update = [&](OfdmKalmanState& s, float z, float r) {
        float S = s.P00 + r;
        // Innovation gating: 3.5σ
        float gate = 3.5f * 3.5f * S;
        if (z * z > gate) { gated_count++; return; }
        float K0 = s.P00 / S;
        float K1 = s.P01 / S;
        float K2 = s.P02 / S;
        s.phase += K0 * z;
        s.freq  += K1 * z;
        s.accel += K2 * z;
        s.accel = std::clamp(s.accel, -1e-4f, 1e-4f);
        // Update STF innovation variance tracker
        stf_Vk = stf_rho * stf_Vk + (1.0f - stf_rho) * z * z;
        // Joseph form: P=(I-KH)*P*(I-KH)'+K*R*K'
        float np00 = s.P00 - K0*s.P00 + K0*r*K0;
        float np01 = s.P01 - K0*s.P01 + K0*r*K1;
        float np02 = s.P02 - K0*s.P02 + K0*r*K2;
        float np11 = s.P11 - K1*s.P01 + K1*r*K1;
        float np12 = s.P12 - K1*s.P02 + K1*r*K2;
        float np22 = s.P22 - K2*s.P02 + K2*r*K2;
        s.P00=np00; s.P01=np01; s.P02=np02;
        s.P11=np11; s.P12=np12; s.P22=np22;
    };

    // ================================================================
    // Pilot symbol CPE measurement helper (SNR-weighted, all carriers)
    // H is immutable — measurement = arg(Y*conj(H)) gives absolute phase
    // ================================================================
    auto measure_pilot_cpe = [&](const std::complex<float>* fft_data) -> float {
        float num = 0.0f, den = 0.0f;
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            std::complex<float> Y = fft_data[bin];
            std::complex<float> H = channel_est_.H[i];
            float Hm2 = std::norm(H);
            float nv = (i < (int)channel_est_.noise_var.size())
                       ? channel_est_.noise_var[i] : 1e-6f;
            if (nv < 1e-12f) nv = 1e-12f;
            float w = Hm2 / nv;
            if (w < 1e-6f) continue;
            float phase = std::arg(Y * std::conj(H));
            num += w * phase;
            den += w;
        }
        return (den > 0.0f) ? (num / den) : 0.0f;
    };

    // ================================================================
    // Time-Domain Filter (TDF): per-sample phase de-rotation before FFT
    // Removes intra-symbol phase drift that causes ICI.
    // Uses Kalman prediction: phi(n) = freq*(n/nfft) + 0.5*accel*(n/nfft)^2
    // Referenced to symbol center so CPE (mean phase) is preserved.
    // See: Casas et al., IEEE Trans. Broadcasting, 2002.
    // ================================================================
    auto tdf_derotate = [&](std::complex<float>* buf, const OfdmKalmanState& s) {
        // freq/accel are in radians per symbol (nfft samples).
        // Phase ramp relative to center: t = (n - nfft/2) / nfft
        float half = 0.5f * nfft;
        float inv_nfft = 1.0f / (float)nfft;
        // With nfft=1024 this is only 1024 sincos calls — fast enough.
        for (int n = 0; n < nfft; n++) {
            float t = ((float)n - half) * inv_nfft;  // -0.5 to +0.5
            float phi = s.freq * t + 0.5f * s.accel * t * t;
            buf[n] *= std::complex<float>(std::cos(phi), -std::sin(phi));
        }
    };

    // ================================================================
    // S2 E1/E2: refresh per-carrier H from an all-carrier pilot row. Pilot
    // rows transmit a constant-modulus ZC reference (generate_pilot_symbol);
    // the caller derotates the row FFT by conj(zc) BEFORE this update, which
    // leaves the same |X| = deemph-gain reference the preamble H was
    // estimated against (ofdm_estimate_channel divides out the preamble's ZC
    // phase the same way), so after removing the RX phase artifacts, Y[bin]
    // is a fresh (noisy) snapshot of H[i].
    //   Step 1: remove the measured common phase (scalar CPE).
    //   Step 2 (S1 fix): remove the residual LINEAR per-carrier phase (timing/
    //           SFO drift + tdf transient) via a weighted regression of
    //           arg(obs*conj(H)) across carriers — the same estimator the comb
    //           path runs on data symbols. A pure timing drift is fully linear
    //           => zero non-linear residual => H does not move (no S1 fold). What
    //           survives is genuine multipath (non-linear in frequency) + fade.
    //   Step 3 (E2): leak-free 7-tap quadratic frequency smoothing (denoises so
    //           the EMA does not inject noise at low SNR; no band-edge Gibbs
    //           leakage, unlike a subset-tap DFT smoother).
    //   Step 4 (E1): EMA-blend the cleaned observation into H (tracks per-carrier
    //           amplitude + non-linear phase).
    // ================================================================
    std::vector<std::complex<float>> s2_obs_buf(n_used);
    auto s2_update_H = [&](const std::complex<float>* fft_data, float cpe_phase) {
        if (!s2_e1) return;
        // Step 1: scalar common-phase removal.
        std::complex<float> derot(std::cos(-cpe_phase), std::sin(-cpe_phase));
        for (int i = 0; i < n_used; i++)
            s2_obs_buf[i] = fft_data[config_.used_carrier_bins[i]] * derot;

        // Step 2: residual linear-phase (timing/SFO) removal relative to current H.
        if (s2_lindet) {
            float w_sum=0, wx_sum=0, wy_sum=0, wxx_sum=0, wxy_sum=0;
            for (int i = 0; i < n_used; i++) {
                std::complex<float> H = channel_est_.H[i];
                float H_mag2 = std::norm(H);
                float nv = (i < (int)channel_est_.noise_var.size())
                           ? channel_est_.noise_var[i] : 1e-6f;
                if (nv < 1e-12f) nv = 1e-12f;
                float w = H_mag2 / nv;
                if (w < 1e-6f) continue;
                float ph = std::arg(s2_obs_buf[i] * std::conj(H));
                float x = (float)i;
                w_sum += w; wx_sum += w*x; wy_sum += w*ph;
                wxx_sum += w*x*x; wxy_sum += w*x*ph;
            }
            if (w_sum > 0.0f) {
                float center = wx_sum / w_sum;
                float intercept = wy_sum / w_sum;   // residual common phase at centroid
                float denom = w_sum*wxx_sum - wx_sum*wx_sum;
                float slope = 0.0f;
                if (std::abs(denom) > 1e-6f)
                    slope = (w_sum*wxy_sum - wx_sum*wy_sum) / denom;
                for (int i = 0; i < n_used; i++) {
                    float corr = intercept + slope * ((float)i - center);
                    s2_obs_buf[i] *= std::complex<float>(std::cos(-corr), std::sin(-corr));
                }
            }
        }

        // Step 3: leak-free 7-tap quadratic frequency smoothing of the obs.
        if (s2_e2 && n_used >= 5) {
            std::vector<std::complex<float>> in(s2_obs_buf.begin(), s2_obs_buf.end());
            const int W = 3;
            for (int i = 0; i < n_used; i++) {
                int lo = std::max(0, i - W), hi = std::min(n_used - 1, i + W);
                int cnt = hi - lo + 1;
                if (cnt < 3) {
                    std::complex<float> sum(0.0f, 0.0f);
                    for (int j = lo; j <= hi; j++) sum += in[j];
                    s2_obs_buf[i] = sum / (float)cnt;
                    continue;
                }
                float S0=0,S1=0,S2=0,S3=0,S4=0;
                float Yr0=0,Yr1=0,Yr2=0,Yi0=0,Yi1=0,Yi2=0;
                for (int j = lo; j <= hi; j++) {
                    float x = (float)(j - i), x2 = x * x;
                    S0+=1; S1+=x; S2+=x2; S3+=x*x2; S4+=x2*x2;
                    Yr0+=in[j].real();      Yr1+=in[j].real()*x;  Yr2+=in[j].real()*x2;
                    Yi0+=in[j].imag();      Yi1+=in[j].imag()*x;  Yi2+=in[j].imag()*x2;
                }
                float D = S0*(S2*S4-S3*S3) - S1*(S1*S4-S3*S2) + S2*(S1*S3-S2*S2);
                if (std::abs(D) > 1e-12f) {
                    float ar = Yr0*(S2*S4-S3*S3) - S1*(Yr1*S4-Yr2*S3) + S2*(Yr1*S3-Yr2*S2);
                    float ai = Yi0*(S2*S4-S3*S3) - S1*(Yi1*S4-Yi2*S3) + S2*(Yi1*S3-Yi2*S2);
                    s2_obs_buf[i] = std::complex<float>(ar / D, ai / D);
                }
            }
        }

        // Step 4: EMA into H.
        for (int i = 0; i < n_used; i++)
            channel_est_.H[i] = (1.0f - s2_alpha) * channel_est_.H[i]
                              + s2_alpha * s2_obs_buf[i];
    };

    // ================================================================
    // S2 E3: per-symbol amplitude refinement from the comb pilots (known +1 on
    // every data symbol). Between pilot rows the flat-fade amplitude drifts;
    // E1's row update is a zero-order hold. E3 measures the SNR-weighted comb
    // magnitude ratio |Y_comb|/|H_comb| ~ the residual flat gain for THIS symbol
    // and normalizes the equalized carriers by it. Decision-FREE (uses known
    // pilots, not data decisions) so there is no error propagation.
    // ================================================================
    auto s2_comb_gain = [&]() -> float {
        float wy = 0.0f, wh = 0.0f;
        for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
            std::complex<float> H = channel_est_.H[i];
            float H_mag2 = std::norm(H);
            float nv = (i < (int)channel_est_.noise_var.size())
                       ? channel_est_.noise_var[i] : 1e-6f;
            if (nv < 1e-12f) nv = 1e-12f;
            float w = H_mag2 / nv;
            if (w < 1e-6f) continue;
            wy += w * std::abs(used_carriers_buf[i]);
            wh += w * std::sqrt(H_mag2);
        }
        return (wh > 1e-9f) ? (wy / wh) : 1.0f;
    };
    auto s2_apply_comb_gain = [&](int n_data_actual) {
        if (!s2_e3) return;
        float g = s2_comb_gain();
        if (g > 0.2f && g < 5.0f && std::abs(g - 1.0f) > 1e-3f) {
            float inv = 1.0f / g;
            for (int i = 0; i < n_data_actual; i++) eq_carriers_buf[i] *= inv;
        }
    };

    // Pilot-row reference derotation. The TX modulates pilot rows with a
    // constant-modulus ZC sequence (generate_pilot_symbol / ofdm_pilot_zc_root
    // — the all-ones comb was a 19-22 dB PAPR impulse the FM limiter clipped
    // first). |zc| = 1, so multiplying the received bin by conj(zc[i]) turns
    // the observation back into H[i]*e^{j*cpe} — exactly what measure_pilot_cpe
    // and s2_update_H consume. Comb pilots inside data symbols remain +1*g and
    // need no derotation.
    std::vector<std::complex<float>> pilot_row_conj(n_used);
    {
        auto zc = generate_zc_sequence(ofdm_pilot_zc_root(n_used), n_used);
        for (int i = 0; i < n_used; i++)
            pilot_row_conj[i] = std::conj(zc[i]);
    }

    for (int s = 0; s < n_data_symbols; s++) {
        // E3: the block-pilot tier (pilot_symbol_spacing=24) was DELETED on both
        // ends (see ofdm_mod.cc). It emitted a symbol byte-identical to — and at
        // the same position as — the dense pilot row below (block spacing 24 is a
        // multiple of row spacing 8), so it was a pure duplicate. The dense pilot
        // row at that symbol drives the identical Kalman CPE update + drift-free
        // s2_update_H. Removed here in lockstep with the TX so the RX consumes
        // exactly the symbols the TX emits.
        // Check for dense pilot row: every pilot_row_spacing data symbols
        if (config_.pilot_row_spacing > 0 && data_sym_count > 0 &&
            (data_sym_count % config_.pilot_row_spacing) == 0)
        {
            if (pos + sym_len > remaining) {
                IRIS_LOG("[OFDM-RX] insufficient samples for pilot row at data_sym %d", data_sym_count);
                return finish_result();
            }

            const std::complex<float>* pr_body = iq_corrected.data() + pos + cp;
            std::copy(pr_body, pr_body + nfft, fft_buf.data());
            tdf_derotate(fft_buf.data(), ks);
            fft_complex(fft_buf.data(), nfft);

            // Undo the known ZC modulation in place (|zc| = 1): after this,
            // the used bins hold the all-ones-equivalent observation that
            // measure_pilot_cpe and s2_update_H are built around.
            for (int i = 0; i < n_used; i++)
                fft_buf[config_.used_carrier_bins[i]] *= pilot_row_conj[i];

            // Pilot row → Kalman measurement (high quality, all carriers)
            float pr_cpe = measure_pilot_cpe(fft_buf.data());
            float z = pr_cpe - ks.phase;
            while (z > (float)M_PI) z -= 2*(float)M_PI;
            while (z < -(float)M_PI) z += 2*(float)M_PI;
            IRIS_LOG("[OFDM-RX] pilot row %d: meas=%.1f pred=%.1f innov=%.1f deg",
                     data_sym_count / config_.pilot_row_spacing,
                     pr_cpe * 180.0f / (float)M_PI,
                     ks.phase * 180.0f / (float)M_PI,
                     z * 180.0f / (float)M_PI);
            kalman_update(ks, z, r_pilot);

            // Dense pilot row: S2 refreshes per-carrier H here too, drift-free
            // (see the block-pilot note + s2_update_H). Off => H frozen at preamble.
            s2_update_H(fft_buf.data(), pr_cpe);

            pos += sym_len;
            total_symbols_elapsed++;
        }

        // ---- Kalman predict for this data symbol ----
        if (data_sym_count > 0) {
            float lam;
            kalman_predict(ks, lam);
            kalman_fwd_lambda[data_sym_count] = lam;
        }

        // Now receive the actual data symbol
        if (pos + sym_len > remaining) {
            IRIS_LOG("[OFDM-RX] insufficient samples for data symbol %d/%d", s, n_data_symbols);
            return finish_result();
        }

        sym_positions[data_sym_count] = pos;  // store for RTS second pass
        const std::complex<float>* sym_body = iq_corrected.data() + pos + cp;
        std::copy(sym_body, sym_body + nfft, fft_buf.data());
        tdf_derotate(fft_buf.data(), ks);
        fft_complex(fft_buf.data(), nfft);

        // Extract values at used carrier bins
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            used_carriers_buf[i] = fft_buf[bin];
        }

        // ---- Kalman phase correction ----
        // Apply predicted phase to used_carriers_buf (H is immutable)
        float total_applied_phase = ks.phase;  // Track total correction applied to carriers
        if (std::abs(ks.phase) > 1e-6f) {
            std::complex<float> pred_rot(std::cos(-ks.phase), std::sin(-ks.phase));
            for (int i = 0; i < n_used; i++)
                used_carriers_buf[i] *= pred_rot;
        }

        // ---- Comb pilot CPE + ICI phase slope → Kalman measurement ----
        // Weighted linear regression on pilot phases extracts:
        //   - intercept (CPE): common phase error (k=0 term of phase noise DFT)
        //   - slope: linear phase trend across carriers (k=1 ICI term)
        // CPE feeds the Kalman; slope provides per-carrier ICI correction.
        // Ref: 5G NR PT-RS design (Qi et al. 2018), Petrovic et al. 2007.
        {
            float w_sum = 0, wx_sum = 0, wy_sum = 0, wxx_sum = 0, wxy_sum = 0;
            for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
                std::complex<float> Y_pilot = used_carriers_buf[i];
                std::complex<float> H = channel_est_.H[i];
                float H_mag2 = std::norm(H);
                float nv = (i < (int)channel_est_.noise_var.size())
                           ? channel_est_.noise_var[i] : 1e-6f;
                if (nv < 1e-12f) nv = 1e-12f;
                float w = H_mag2 / nv;
                if (w < 1e-6f) continue;
                float phase_diff = std::arg(Y_pilot * std::conj(H));
                float x = (float)i;
                w_sum  += w;
                wx_sum += w * x;
                wy_sum += w * phase_diff;
                wxx_sum += w * x * x;
                wxy_sum += w * x * phase_diff;
            }
            if (w_sum > 0.0f) {
                // Weighted linear regression: phase = cpe + slope*(i - center)
                float center = wx_sum / w_sum;
                float cpe_resid = wy_sum / w_sum;  // intercept at centroid = mean CPE
                float denom = w_sum * wxx_sum - wx_sum * wx_sum;
                float ici_slope = 0.0f;
                if (std::abs(denom) > 1e-6f) {
                    ici_slope = (w_sum * wxy_sum - wx_sum * wy_sum) / denom;
                }

                // Feed CPE to Kalman (same as before — slope is independent)
                kalman_update(ks, cpe_resid, r_comb);

                // Apply CPE + per-carrier ICI slope correction
                for (int i = 0; i < n_used; i++) {
                    float phase_corr = cpe_resid + ici_slope * ((float)i - center);
                    std::complex<float> rot(std::cos(-phase_corr), std::sin(-phase_corr));
                    used_carriers_buf[i] *= rot;
                }
                total_applied_phase += cpe_resid;
                // Note: slope is per-carrier, not tracked in scalar total_applied_phase.
                // RTS smoother adjusts constant phase only; slope correction persists.
            }
        }

        // Diagnostic logging
        if (data_sym_count % 10 == 0 || data_sym_count == n_data_symbols - 1) {
            IRIS_LOG("[OFDM-RX] sym %d/%d: kalman phase=%.1f freq=%.2f accel=%.4f deg "
                     "(P00=%.4f, lambda=%.2f)",
                     data_sym_count, n_data_symbols,
                     ks.phase * 180.0f / (float)M_PI,
                     ks.freq * 180.0f / (float)M_PI,
                     ks.accel * 180.0f / (float)M_PI,
                     ks.P00, kalman_fwd_lambda[data_sym_count]);
        }

        // Extract data carriers (skip pilot positions)
        int n_data_actual = extract_data_carriers(used_carriers_buf.data(), n_used,
                                                  data_carriers_buf);

        // S2: snapshot the causal H used to equalize this symbol, so the deferred
        // soft-demap LLR sigma weighting uses the H that was current here (not the
        // final, end-of-frame H after all pilot-row updates).
        if (s2_e1)
            H_snap[data_sym_count].assign(channel_est_.H.begin(), channel_est_.H.end());

        // MMSE equalize
        equalize_mmse(data_carriers_buf, n_data_actual, channel_est_, eq_carriers_buf);

        // ---- DFT-despread (SC-FDMA IDFT) ----
        if (config_.dft_spread && n_data_actual > 1) {
            std::vector<std::complex<float>> idft_out(n_data_actual);
            small_idft(eq_carriers_buf.data(), idft_out.data(), n_data_actual);
            std::copy(idft_out.begin(), idft_out.end(), eq_carriers_buf.begin());
        }

        // S2 E3: per-symbol comb-pilot amplitude normalization (forward pass).
        s2_apply_comb_gain(n_data_actual);

        // ---- BPS (Blind Phase Search) for QAM16+ ----
        // Replaces DD-CPE. ±45° unambiguous range via QAM 4-fold symmetry.
        // Quadrant ambiguity resolved by Kalman prediction (anchored to pilots).
        // (BPSK/QPSK need no per-symbol blind loop once the pilot-driven H
        //  re-estimate — which was injecting the drift — is removed; the comb/
        //  block/row-pilot CPE Kalman track keeps them aligned. See the pilot
        //  H-update note above.)
        float bps_correction = 0.0f;
        if (use_bps) {
            bps_correction = bps_estimate(eq_carriers_buf.data(), n_data_actual,
                                           active_tone_map, fec_r16);
            if (std::abs(bps_correction) > 1e-4f) {
                // Apply BPS correction to eq carriers
                std::complex<float> bps_rot(std::cos(-bps_correction), std::sin(-bps_correction));
                for (int i = 0; i < n_data_actual; i++)
                    eq_carriers_buf[i] *= bps_rot;
                total_applied_phase += bps_correction;

                // Feed BPS result back to Kalman as measurement
                kalman_update(ks, bps_correction, r_bps);

                if (data_sym_count % 10 == 0 || std::abs(bps_correction) > 0.05f) {
                    IRIS_LOG("[OFDM-RX] BPS sym %d: %.2f deg",
                             data_sym_count, bps_correction * 180.0f / (float)M_PI);
                }
            }
        }

        // Accel mean-reversion (match frame.cc: half-life ~700 symbols)
        ks.accel *= 0.999f;

        // Store forward Kalman state and equalized carriers for RTS pass
        kalman_fwd[data_sym_count] = ks;
        stored_fwd_phase[data_sym_count] = total_applied_phase;
        stored_n_data[data_sym_count] = n_data_actual;
        stored_eq[data_sym_count].assign(eq_carriers_buf.begin(),
                                          eq_carriers_buf.begin() + n_data_actual);

        pos += sym_len;
        data_sym_count++;
        total_symbols_elapsed++;
    }

    // ================================================================
    // RTS Backward Smoother (3-state)
    //
    // Produces optimal interpolated phase estimates between pilot rows.
    // Runs after all symbols collected. Re-corrects stored eq carriers
    // with (smoothed - forward) residual, then soft-demaps.
    // ================================================================
    std::vector<float> smoothed_phase(data_sym_count, 0.0f);
    std::vector<float> smoothed_freq(data_sym_count, 0.0f);
    std::vector<float> smoothed_accel(data_sym_count, 0.0f);
    std::vector<float> smoothed_P00(data_sym_count, 0.0f);

    if (data_sym_count > 0) {
        // Initialize last symbol: smoothed = forward
        int last = data_sym_count - 1;
        smoothed_phase[last] = kalman_fwd[last].phase;
        smoothed_freq[last]  = kalman_fwd[last].freq;
        smoothed_accel[last] = kalman_fwd[last].accel;
        smoothed_P00[last]   = kalman_fwd[last].P00;

        // Smoothed covariance at last symbol
        float sp00 = kalman_fwd[last].P00;
        float sp01 = kalman_fwd[last].P01;
        float sp02 = kalman_fwd[last].P02;
        float sp11 = kalman_fwd[last].P11;
        float sp12 = kalman_fwd[last].P12;
        float sp22 = kalman_fwd[last].P22;

        // Backward sweep
        for (int k = last - 1; k >= 0; k--) {
            auto& fk = kalman_fwd[k];
            float fp = fk.phase, ff = fk.freq, fa = fk.accel;
            float p00=fk.P00, p01=fk.P01, p02=fk.P02;
            float p11=fk.P11, p12=fk.P12, p22=fk.P22;

            // P_pred = lambda * A*P*A' + Q (match forward pass)
            float a00 = p00 + 2*p01 + p02 + p11 + p12 + 0.25f*p22;
            float a01 = p01 + p02 + p11 + 1.5f*p12 + 0.5f*p22;
            float a02 = p02 + p12 + 0.5f*p22;
            float a11 = p11 + 2*p12 + p22;
            float a12 = p12 + p22;
            float a22 = p22;
            float lam = kalman_fwd_lambda[k+1];
            float pp00 = lam*a00 + q_phase;
            float pp01 = lam*a01;
            float pp02 = a02;
            float pp11 = lam*a11 + q_freq;
            float pp12 = a12;
            float pp22 = a22 + q_accel;

            // PA = P * A' (A' is transpose of [[1,1,0.5],[0,1,1],[0,0,1]])
            float pa00=p00+p01+0.5f*p02, pa01=p01+p02,         pa02=p02;
            float pa10=p01+p11+0.5f*p12, pa11=p11+p12,         pa12=p12;
            float pa20=p02+p12+0.5f*p22, pa21=p12+p22,         pa22=p22;

            // Invert P_pred via cofactors
            float det = pp00*(pp11*pp22 - pp12*pp12)
                      - pp01*(pp01*pp22 - pp02*pp12)
                      + pp02*(pp01*pp12 - pp02*pp11);
            if (std::abs(det) < 1e-12f) det = (det < 0 ? -1e-12f : 1e-12f);
            float id = 1.0f / det;
            float i00 = (pp11*pp22 - pp12*pp12) * id;
            float i01 = (pp02*pp12 - pp01*pp22) * id;
            float i02 = (pp01*pp12 - pp02*pp11) * id;
            float i11 = (pp00*pp22 - pp02*pp02) * id;
            float i12 = (pp02*pp01 - pp00*pp12) * id;
            float i22 = (pp00*pp11 - pp01*pp01) * id;

            // C = PA * inv(P_pred) (3x3 matrix multiply)
            float c00=pa00*i00+pa01*i01+pa02*i02;
            float c01=pa00*i01+pa01*i11+pa02*i12;
            float c02=pa00*i02+pa01*i12+pa02*i22;
            float c10=pa10*i00+pa11*i01+pa12*i02;
            float c11=pa10*i01+pa11*i11+pa12*i12;
            float c12=pa10*i02+pa11*i12+pa12*i22;
            float c20=pa20*i00+pa21*i01+pa22*i02;
            float c21=pa20*i01+pa21*i11+pa22*i12;
            float c22=pa20*i02+pa21*i12+pa22*i22;

            // Smoothed covariance: P_s[k] = P[k] + C*(P_s[k+1] - P_pred)*C'
            float d00=sp00-pp00, d01=sp01-pp01, d02=sp02-pp02;
            float d11=sp11-pp11, d12=sp12-pp12, d22=sp22-pp22;
            // E = C * D (symmetric)
            float e00=c00*d00+c01*d01+c02*d02;
            float e01=c00*d01+c01*d11+c02*d12;
            float e02=c00*d02+c01*d12+c02*d22;
            float e10=c10*d00+c11*d01+c12*d02;
            float e11=c10*d01+c11*d11+c12*d12;
            float e12=c10*d02+c11*d12+c12*d22;
            float e20=c20*d00+c21*d01+c22*d02;
            float e21=c20*d01+c21*d11+c22*d12;
            float e22=c20*d02+c21*d12+c22*d22;
            // P_s[k] = P[k] + E*C'
            sp00 = p00 + e00*c00+e01*c10+e02*c20;
            sp01 = p01 + e00*c01+e01*c11+e02*c21;
            sp02 = p02 + e00*c02+e01*c12+e02*c22;
            sp11 = p11 + e10*c01+e11*c11+e12*c21;
            sp12 = p12 + e10*c02+e11*c12+e12*c22;
            sp22 = p22 + e20*c02+e21*c12+e22*c22;

            // Smoothed state: x_s[k] = x_f[k] + C*(x_s[k+1] - A*x_f[k])
            float dp = smoothed_phase[k+1] - (fp + ff + 0.5f*fa);
            float df = smoothed_freq[k+1]  - (ff + fa);
            float da = smoothed_accel[k+1] - fa;
            smoothed_phase[k] = fp + c00*dp + c01*df + c02*da;
            smoothed_freq[k]  = ff + c10*dp + c11*df + c12*da;
            smoothed_accel[k] = fa + c20*dp + c21*df + c22*da;
            smoothed_P00[k]   = sp00;
        }
    }

    // ---- RTS second pass: re-FFT with smoothed tdf for 64QAM+ ----
    // The forward pass tdf uses the predicted (pre-measurement) Kalman state,
    // which lags the actual frequency drift. For 64QAM+, the residual ICI from
    // this prediction error exceeds the decision boundary margin.
    // Solution: re-FFT using RTS-smoothed freq/accel for tdf (better ICI removal),
    // then fresh comb pilot measurement for CPE (not smoothed_phase, which doesn't
    // capture the direct comb/BPS corrections from the forward pass).
    // Cost: one extra FFT per data symbol (N=1024, negligible at ~44 sym/s).
    // Ref: Petrovic, Rave & Fettweis, "Phase Noise Suppression in OFDM", 2007.
    // Gate on Kalman dynamics: only run second pass if freq drift is significant.
    // Check max |smoothed_freq| across data symbols — if < threshold, forward pass
    // is already accurate enough and re-FFT introduces quantization noise.
    float max_rts_freq = 0.0f;
    for (int k = 0; k < data_sym_count; k++)
        max_rts_freq = std::max(max_rts_freq, std::abs(smoothed_freq[k]));
    bool rts_tdf_needed = (max_bpc >= 6 && data_sym_count > 2 &&
                           max_rts_freq > 0.01f);  // ~0.6 deg/sym threshold

    if (rts_tdf_needed) {
        for (int k = 0; k < data_sym_count; k++) {
            int nd = stored_n_data[k];
            if (nd <= 0) continue;

            // Re-read time-domain symbol from iq_corrected
            const auto* sym_body = iq_corrected.data() + sym_positions[k] + cp;
            std::copy(sym_body, sym_body + nfft, fft_buf.data());

            // tdf with RTS-smoothed freq/accel (optimal ICI removal)
            OfdmKalmanState rts_ks;
            rts_ks.freq = smoothed_freq[k];
            rts_ks.accel = smoothed_accel[k];
            tdf_derotate(fft_buf.data(), rts_ks);

            // Re-FFT
            fft_complex(fft_buf.data(), nfft);

            // Extract carriers at used bins
            for (int i = 0; i < n_used; i++)
                used_carriers_buf[i] = fft_buf[config_.used_carrier_bins[i]];

            // Fresh comb pilot CPE + ICI slope (weighted linear regression)
            float w_sum = 0, wx_sum = 0, wy_sum = 0, wxx_sum = 0, wxy_sum = 0;
            for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
                std::complex<float> Y_pilot = used_carriers_buf[i];
                std::complex<float> H = channel_est_.H[i];
                float H_mag2 = std::norm(H);
                float nv = (i < (int)channel_est_.noise_var.size())
                           ? channel_est_.noise_var[i] : 1e-6f;
                if (nv < 1e-12f) nv = 1e-12f;
                float w = H_mag2 / nv;
                if (w < 1e-6f) continue;
                float phase_diff = std::arg(Y_pilot * std::conj(H));
                float x = (float)i;
                w_sum  += w;
                wx_sum += w * x;
                wy_sum += w * phase_diff;
                wxx_sum += w * x * x;
                wxy_sum += w * x * phase_diff;
            }
            if (w_sum > 0.0f) {
                float center = wx_sum / w_sum;
                float cpe2 = wy_sum / w_sum;
                float denom = w_sum * wxx_sum - wx_sum * wx_sum;
                float ici_slope2 = 0.0f;
                if (std::abs(denom) > 1e-6f)
                    ici_slope2 = (w_sum * wxy_sum - wx_sum * wy_sum) / denom;

                // Apply CPE + per-carrier ICI slope correction
                for (int i = 0; i < n_used; i++) {
                    float phase_corr = cpe2 + ici_slope2 * ((float)i - center);
                    std::complex<float> rot(std::cos(-phase_corr), std::sin(-phase_corr));
                    used_carriers_buf[i] *= rot;
                }
            }

            // Extract data carriers (skip pilot positions)
            int n_data_actual = extract_data_carriers(used_carriers_buf.data(), n_used,
                                                      data_carriers_buf);

            // MMSE equalize
            equalize_mmse(data_carriers_buf, n_data_actual, channel_est_, eq_carriers_buf);

            // DFT-despread
            if (config_.dft_spread && n_data_actual > 1) {
                std::vector<std::complex<float>> idft_out(n_data_actual);
                small_idft(eq_carriers_buf.data(), idft_out.data(), n_data_actual);
                std::copy(idft_out.begin(), idft_out.end(), eq_carriers_buf.begin());
            }

            // BPS for QAM16+ (refine phase beyond comb pilot resolution)
            if (use_bps) {
                float bps2 = bps_estimate(eq_carriers_buf.data(), n_data_actual,
                                           active_tone_map, fec_r16);
                if (std::abs(bps2) > 1e-4f) {
                    std::complex<float> bps_rot(std::cos(-bps2), std::sin(-bps2));
                    for (int i = 0; i < n_data_actual; i++)
                        eq_carriers_buf[i] *= bps_rot;
                }
            }

            // Replace forward-pass carriers with RTS-corrected carriers
            stored_eq[k].assign(eq_carriers_buf.begin(),
                                eq_carriers_buf.begin() + n_data_actual);
            stored_n_data[k] = n_data_actual;
        }
    }

    // ---- Soft demapping ----
    result.eq_constellation.clear();
    result.eq_constellation.reserve(data_sym_count * n_data);

    // S2: the forward pass mutated channel_est_.H (live per-carrier tracking).
    // Soft-demap LLR sigma weighting must use the CAUSAL per-symbol H (H_snap[k]),
    // not the final end-of-frame H — otherwise every symbol's LLR confidence is
    // scaled by the last pilot-row's |H|. Save/restore the final H around the loop
    // so no later consumer sees a per-symbol value.
    std::vector<std::complex<float>> H_final;
    if (s2_e1) H_final = channel_est_.H;

    for (int k = 0; k < data_sym_count; k++) {
        int nd = stored_n_data[k];
        if (nd <= 0) continue;

        result.eq_constellation.insert(result.eq_constellation.end(),
                                        stored_eq[k].begin(),
                                        stored_eq[k].begin() + nd);

        // S2: restore the causal per-symbol H for this symbol's LLR sigma.
        if (s2_e1 && !H_snap[k].empty())
            channel_est_.H = H_snap[k];

        // Soft demap
        std::copy(stored_eq[k].begin(), stored_eq[k].begin() + nd,
                  eq_carriers_buf.begin());
        demap_to_llrs(eq_carriers_buf, nd, active_tone_map, channel_est_, all_llrs,
                      last_dft_sigma_sq_, last_metric_sigma_sq_);
    }
    if (s2_e1) channel_est_.H = H_final;

    result.dft_sigma_sq_llr = last_dft_sigma_sq_;

    // This observation is deliberately independent of the repeated training
    // pair: slice each post-equalization payload symbol against its actual
    // uniform/NUC constellation and measure the decision residual.  It is not
    // used for LLRs; after payload CRC validation it is solely the external
    // cross-check that may authorize training-derived SNR for rate control.
    result.payload_residual_snr_valid = compute_payload_residual_snr_db(
        result.eq_constellation, active_tone_map, geometry->fec(),
        result.payload_residual_snr_db);
    if (result.payload_residual_snr_valid) {
        IRIS_LOG("[OFDM-RX] payload decision-residual SNR=%.1f dB (%zu symbols)",
                 result.payload_residual_snr_db,
                 result.eq_constellation.size());
    }

    // ---- Effective SNR for gearshift ----
    // Prior metric was -10*log10(mean(nv/(|H|²+nv))) which saturates at ~7 dB
    // on frequency-selective channels because deep-fade carriers with
    // |H|² < NV_ABS_FLOOR pin sigma_i ≈ 1 regardless of true noise.
    // Replaced with FreeDV/Rhizomatica decision-residual EsNo on the post-EQ
    // data-symbol stream: signal power divided by minor-axis noise variance
    // from outer-ring symbols. Ratio preserves on deep-fade carriers (both
    // sides drop together) and reflects the actual decodability.
    // `last_dft_sigma_sq_` is unchanged and still used for LLR scaling.
    if (!result.eq_constellation.empty()) {
        // QPSK-only heuristic works cleanly when the active tone map is QPSK
        // (bits_per_carrier == 2). For QAM16+ the minor-axis picks are biased
        // by inner constellation points; fall back to the old metric there.
        // Applies regardless of dft_spread since post-EQ symbols are
        // available in both cases.
        int max_bpc = 0;
        for (int i = 0; i < active_tone_map.n_data_carriers; i++)
            if (active_tone_map.bits_per_carrier[i] > max_bpc)
                max_bpc = active_tone_map.bits_per_carrier[i];
        if (max_bpc <= 2) {
            float esno_db = compute_esno_db_qpsk(
                result.eq_constellation.data(),
                (int)result.eq_constellation.size());
            result.effective_snr_db = esno_db;
            IRIS_LOG("[OFDM-RX] decision-residual EsNo=%.1f dB (n=%d syms, QPSK, bpc=%d)",
                     esno_db, (int)result.eq_constellation.size(), max_bpc);
        } else if (config_.dft_spread) {
            // QAM16+ gearshift metric: post-despread effective SINR
            //   gamma_eff = mu/(1-mu),  mu = 1 - last_metric_sigma_sq_
            // with last_metric_sigma_sq_ = mean(nv_k/(|H_k|^2+nv_k)) over the
            // honest in-band per-carrier sigma^2(k) (see demap_to_llrs). This
            // is the SNR the decoder actually experiences after MMSE FDE +
            // IDFT despread, and it matches the QPSK branch above (post-EQ
            // decision-residual EsNo) in kind. Differs from the previous
            // -10log10(s) form by 10log10(mu) — negligible at high SNR,
            // honest at low. Clamped to +/-60 dB.
            result.effective_snr_db = metric_gamma_eff_db(last_metric_sigma_sq_);
        }
        // else: non-DFT-spread QAM16+ keeps whatever default sync.snr_est gave
    } else if (config_.dft_spread) {
        result.effective_snr_db = metric_gamma_eff_db(last_metric_sigma_sq_);
    }

    // ---- Kalman diagnostics ----
    {
        float total_phase = (data_sym_count > 0) ? smoothed_phase[data_sym_count - 1] : 0.0f;
        float final_freq = (data_sym_count > 0) ? smoothed_freq[data_sym_count - 1] : 0.0f;
        IRIS_LOG("[OFDM-RX] Kalman summary: %d syms, phase=%.0f deg, freq=%.2f deg/sym, "
                 "max_lambda=%.2f, gated=%d",
                 data_sym_count,
                 total_phase * 180.0f / (float)M_PI,
                 final_freq * 180.0f / (float)M_PI,
                 max_lambda, gated_count);

        // Residual CFO from Kalman freq state
        float symbol_rate = config_.sample_rate / (float)sym_len;
        result.cpe_drift_hz = final_freq * symbol_rate / (2.0f * (float)M_PI);
    }

    // ---- Populate Kalman trace for GUI 3D viewer + CSV logging ----
    {
        int ds = std::max(1, data_sym_count / 512);
        result.kalman_trace.fwd.clear();
        result.kalman_trace.smoothed.clear();
        result.kalman_trace.total_symbols = data_sym_count;
        result.kalman_trace.downsample_factor = ds;
        for (int k = 0; k < data_sym_count; k += ds) {
            KalmanTracePoint fp;
            fp.phase = kalman_fwd[k].phase;
            fp.freq  = kalman_fwd[k].freq;
            fp.accel = kalman_fwd[k].accel;
            fp.is_pilot = false;  // data symbols
            result.kalman_trace.fwd.push_back(fp);

            KalmanTracePoint sp;
            sp.phase = smoothed_phase[k];
            sp.freq  = smoothed_freq[k];
            sp.accel = smoothed_accel[k];
            sp.is_pilot = false;
            result.kalman_trace.smoothed.push_back(sp);
        }
    }

    // Per-symbol phase variance for HARQ CSI
    result.sym_phase_var.resize(data_sym_count);
    for (int k = 0; k < data_sym_count; k++)
        result.sym_phase_var[k] = smoothed_P00[k];

    // ---- 8b. Validate the codeword-count-bound tail ----
    // Per-block CRCs authenticate block contents, not the number of blocks in
    // this frame. The final known symbol binds both C and the exact boundary;
    // until it matches, decoded blocks are only a prefix hypothesis.
    bool tail_boundary_valid = false;
    float tail_metric = 0.0f;
    if (pos + sym_len <= remaining) {
        const auto* tail_body = iq_corrected.data() + pos + cp;
        std::copy(tail_body, tail_body + nfft, fft_buf.data());
        tdf_derotate(fft_buf.data(), ks);
        fft_complex(fft_buf.data(), nfft);

        const int tail_root = ofdm_tail_zc_root(n_used, n_codewords);
        if (tail_root != 0) {
            const auto tail_ref = generate_zc_sequence(tail_root, n_used);
            std::complex<double> correlation(0.0, 0.0);
            double observed_energy = 0.0;
            double expected_energy = 0.0;
            for (int i = 0; i < n_used; ++i) {
                const std::complex<float> observed =
                    fft_buf[config_.used_carrier_bins[i]];
                const std::complex<float> expected =
                    channel_est_.H[i] * tail_ref[i];
                correlation += static_cast<std::complex<double>>(observed) *
                               std::conj(static_cast<std::complex<double>>(expected));
                observed_energy += static_cast<double>(std::norm(observed));
                expected_energy += static_cast<double>(std::norm(expected));
            }
            const double denom = observed_energy * expected_energy;
            if (denom > 1e-20)
                tail_metric = static_cast<float>(std::norm(correlation) / denom);
            // A wrong data/pilot symbol is approximately 1/n_used, but finite
            // deterministic payloads can correlate above that mean (a measured
            // wrong C=2 boundary reached 0.232).  Complete valid tails in the
            // preservation suite remain above 0.96, leaving a wide margin.
            tail_boundary_valid = tail_metric >= 0.30f;
        }
    }
    IRIS_LOG("[OFDM-RX] tail boundary C=%d: metric=%.3f %s",
             n_codewords, tail_metric, tail_boundary_valid ? "OK" : "REJECT");

    pos += sym_len;

    result.consumed_from_input_start = frame_start + pos;
    result.snr_per_carrier = channel_est_.snr_per_carrier;

    IRIS_LOG("[OFDM-RX] %d LLRs from %d data syms, NUC=%s",
             (int)all_llrs.size(), data_sym_count,
             active_tone_map.use_nuc ? "ON" : "off");

    // ---- 9. Truncate LLRs to exact coded bit count ----
    if ((int)all_llrs.size() > coded_bits_total) {
        all_llrs.resize(coded_bits_total);
    } else if ((int)all_llrs.size() < coded_bits_total) {
        // Pad with zero LLRs (erasures)
        all_llrs.resize(coded_bits_total, 0.0f);
    }

    // ---- 9b. Adaptive LLR clamp ----
    // Literature (ResearchGate: Effect of Saturation on BP Decoding of LDPC)
    // shows LLR clamp of ±6 to ±10 is optimal for low-order modulations.
    // However, 256QAM r7/8 needs more dynamic range — adjacent constellation
    // points are very close, so correct LLRs for inner vs outer bits differ
    // by 10-15×.  ±8 crushes that structure and prevents LDPC convergence.
    //
    // Scale clamp with modulation order:
    //   BPSK/QPSK  (bpc ≤ 2): ±8   — low-order, protect against NV errors
    //   8PSK/16QAM (bpc 3-4): ±12  — moderate dynamic range needed
    //   64QAM      (bpc 5-6): ±16  — wide constellation, need more range
    //   256QAM     (bpc 7-8): ±20  — very dense, full dynamic range required
    {
        int max_bpc_for_clamp = 1;
        for (int k = 0; k < active_tone_map.n_data_carriers; k++)
            if (active_tone_map.bits_per_carrier[k] > max_bpc_for_clamp)
                max_bpc_for_clamp = active_tone_map.bits_per_carrier[k];

        float llr_clamp;
        if (max_bpc_for_clamp <= 2)      llr_clamp = 8.0f;
        else if (max_bpc_for_clamp <= 4) llr_clamp = 12.0f;
        else if (max_bpc_for_clamp <= 6) llr_clamp = 16.0f;
        else                             llr_clamp = 20.0f;

        int n_clamped = 0;
        float max_abs = 0.0f;
        for (auto& l : all_llrs) {
            float a = std::abs(l);
            if (a > max_abs) max_abs = a;
            if (a > llr_clamp) { n_clamped++; l = std::clamp(l, -llr_clamp, llr_clamp); }
        }
        // LLR histogram: bins [0,0.5) [0.5,1) [1,2) [2,4) [4,8) [8+)
        int hist[6] = {};
        int n_positive = 0, n_negative = 0;
        for (auto l : all_llrs) {
            float a = std::abs(l);
            if (l > 0) n_positive++; else if (l < 0) n_negative++;
            if (a < 0.5f) hist[0]++;
            else if (a < 1.0f) hist[1]++;
            else if (a < 2.0f) hist[2]++;
            else if (a < 4.0f) hist[3]++;
            else if (a < 8.0f) hist[4]++;
            else hist[5]++;
        }
        IRIS_LOG("[OFDM-RX] LLR stats: max_abs=%.1f, clamp=±%.0f (bpc=%d), %d/%d clamped",
                 max_abs, llr_clamp, max_bpc_for_clamp, n_clamped, (int)all_llrs.size());
        IRIS_LOG("[OFDM-RX] LLR histogram: <0.5:%d <1:%d <2:%d <4:%d <8:%d 8+:%d | +:%d -:%d",
                 hist[0], hist[1], hist[2], hist[3], hist[4], hist[5],
                 n_positive, n_negative);
    }

    // ---- 10. Descramble LLRs ----
    // The TX scrambled coded bits by XOR with LFSR output.
    // For soft decoding, we flip the LLR sign where the scrambler bit was 1.
    // This is equivalent to descrambling at the LLR level.
    {
        uint16_t lfsr = 0x6959;
        for (size_t i = 0; i < all_llrs.size(); i++) {
            if (lfsr & 1) {
                all_llrs[i] = -all_llrs[i];  // flip LLR sign
            }
            int fb = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
            lfsr = (lfsr >> 1) | ((uint16_t)fb << 14);
        }
    }

    // ---- 10b. Frequency-time de-interleave (reverse of global stride-173) ----
    // TX: out[(i * 173) % N] = in[i]. RX: out[i] = in[(i * 173) % N].
    {
        constexpr int FREQ_TIME_STRIDE = 173;
        int N = (int)all_llrs.size();
        if (N > FREQ_TIME_STRIDE) {
            std::vector<float> tmp(N);
            for (int i = 0; i < N; i++)
                tmp[i] = all_llrs[(i * FREQ_TIME_STRIDE) % N];
            all_llrs = std::move(tmp);
            IRIS_LOG("[OFDM-RX] freq-time de-interleave: stride=%d, %d LLRs", FREQ_TIME_STRIDE, N);
        }
    }

    // ---- 10c. BICM de-interleave (reverse column-row per OFDM symbol) ----
    // TX: out[(i%bpc)*n_carriers + i/bpc] = in[i].
    // RX: out[i] = in[(i%bpc)*n_carriers + i/bpc].
    if (active_tone_map.tone_map_id > 0 && bps_total > 0) {
        int bpc = active_tone_map.bits_per_carrier[0];
        int n_carriers = active_tone_map.n_data_carriers;
        if (bpc >= 4 && bps_total == bpc * n_carriers) {
            int n_syms = 0;
            for (size_t sym_start = 0; sym_start + bps_total <= all_llrs.size();
                 sym_start += bps_total)
            {
                std::vector<float> tmp(bps_total);
                for (int i = 0; i < bps_total; i++) {
                    int row = i % bpc;
                    int col = i / bpc;
                    tmp[i] = all_llrs[sym_start + row * n_carriers + col];
                }
                std::copy(tmp.begin(), tmp.end(), all_llrs.begin() + sym_start);
                n_syms++;
            }
            IRIS_LOG("[OFDM-RX] BICM de-interleave: %d symbols, %d×%d (bpc×carriers)",
                     n_syms, bpc, n_carriers);
        }
    }

    // ---- 11. De-interleave LDPC blocks (reverse stride-41) ----
    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        int n_fec = LdpcCodec::codeword_size(active_tone_map.fec_rate);

        for (int blk_start = 0; blk_start + n_fec <= (int)all_llrs.size();
             blk_start += n_fec)
        {
            // TX interleave: tmp[(i * 41) % n_fec] = coded_bits[blk + i]
            // So coded_bits[blk + i] ended up at position (i * 41) % n_fec
            // To de-interleave: deinterleaved[i] = interleaved[(i * 41) % n_fec]
            std::vector<float> tmp(n_fec);
            for (int i = 0; i < n_fec; i++) {
                tmp[i] = all_llrs[blk_start + (i * INTERLEAVE_STRIDE) % n_fec];
            }
            std::copy(tmp.begin(), tmp.end(), all_llrs.begin() + blk_start);
        }
    }

    // ---- 11c. Generate sym_phase_var for HARQ region selection ----
    // Map per-carrier noise variance to per-coded-bit positions.
    // sym_phase_var[i] = noise_var[k] / |H[k]|² for the carrier that coded bit i maps to.
    {
        // Build per-data-carrier phase variance
        std::vector<float> carrier_pvar;
        carrier_pvar.reserve(config_.n_data_carriers);
        for (int i = 0; i < config_.n_used_carriers; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float nv = (i < (int)channel_est_.noise_var.size()) ? channel_est_.noise_var[i] : 1e-6f;
            float Hm2 = std::norm(channel_est_.H[i]);
            float pvar = (Hm2 > 1e-12f) ? (nv / Hm2) : 10.0f;
            carrier_pvar.push_back(pvar);
        }
        // Expand to per-coded-bit (each carrier contributes bpc LLRs)
        result.sym_phase_var.clear();
        result.sym_phase_var.reserve(coded_bits_total);
        int cpv_idx = 0;
        for (int s = 0; s < n_data_symbols; s++) {
            for (int k = 0; k < active_tone_map.n_data_carriers && cpv_idx < coded_bits_total; k++) {
                int bpc_k = active_tone_map.bits_per_carrier[k];
                float pvar = (k < (int)carrier_pvar.size()) ? carrier_pvar[k] : 1.0f;
                for (int b = 0; b < bpc_k && cpv_idx < coded_bits_total; b++) {
                    result.sym_phase_var.push_back(pvar);
                    cpv_idx++;
                }
            }
        }
    }

    // Save LLRs for HARQ Chase combining (after all de-interleaving)
    result.llrs = all_llrs;

    // Do not spend LDPC work or publish a CRC-valid prefix unless the complete
    // candidate's independently known C/boundary symbol has validated.
    if (!tail_boundary_valid) return finish_result();
    result.complete_boundary_validated = true;

    // ---- 12. LDPC decode (per-block for HARQ) ----
    std::vector<uint8_t> decoded_bits;
    int n_ldpc_blocks = 0;  // number of LDPC blocks decoded (for payload extraction)

    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        // Use per-block decode: continues all blocks even if some fail (for HARQ)
        auto block_results = LdpcCodec::decode_soft_per_block(
            all_llrs, active_tone_map.fec_rate, LdpcDecoder::MIN_SUM, 50);

        result.block_results = block_results;
        n_ldpc_blocks = (int)block_results.size();

        // Check if all blocks converged
        bool all_ok = true;
        int worst_iters = 0;
        for (auto& br : block_results) {
            if (!br.converged) all_ok = false;
            worst_iters = std::max(worst_iters, br.iterations);
        }
        result.worst_ldpc_iters = worst_iters;

        if (!all_ok) {
            int n_failed = 0;
            for (auto& br : block_results)
                if (!br.converged) n_failed++;
            IRIS_LOG("[OFDM-RX] LDPC decode: %d/%d blocks failed (worst=%d iters, snr=%.1f dB)",
                     n_failed, (int)block_results.size(), worst_iters, result.snr_db);
            for (int bi = 0; bi < (int)block_results.size(); bi++) {
                IRIS_LOG("[OFDM-RX]   block %d: %s (%d iters)",
                         bi, block_results[bi].converged ? "OK" : "FAIL",
                         block_results[bi].iterations);
            }
            return finish_result();
        }

        // All blocks OK — collect decoded data bits
        for (auto& br : block_results) {
            decoded_bits.insert(decoded_bits.end(), br.data_bits.begin(), br.data_bits.end());
        }

        IRIS_LOG("[OFDM-RX] LDPC decoded %d data bits (%d blocks), worst-case %d iterations",
                 (int)decoded_bits.size(), (int)block_results.size(), worst_iters);
    } else {
        // No FEC: hard-decision on LLRs
        decoded_bits.reserve(all_llrs.size());
        for (float llr : all_llrs) {
            decoded_bits.push_back(llr >= 0.0f ? 0 : 1);
        }
    }

    // ---- 13-15. Extract payload from each decoded block ----
    // Each block: [len_lo][len_hi][payload_chunk...][CRC32].
    // Multi-codeword: reassemble payload from all blocks in order.
    // Shared with the Chase-combining re-decode (extract_payload_blocks).
    {
        std::vector<uint8_t> full_payload;
        if (!extract_payload_blocks(decoded_bits, geometry->fec(), n_codewords,
                                    static_cast<size_t>(n_codewords) *
                                        static_cast<size_t>(LdpcCodec::block_size(geometry->fec())),
                                    full_payload))
            return finish_result();

        int n_blocks = n_ldpc_blocks;
        if (n_blocks == 0) n_blocks = 1;  // no-FEC fallback

        result.success = true;
        result.completion = OfdmDemodResult::Completion::CompleteValidatedFrame;
        result.payload_validated = true;
        result.cfo_resolved = true;
        // Payload validity resolves the CFO wrap.  SNR publication remains a
        // separate decision and is authorized only by the independently
        // measured post-EQ payload residual above.
        ofdm_authorize_payload_estimator(result);
        result.additional_samples_required = 0;
        result.payload = std::move(full_payload);
        result.payload_len = (uint16_t)result.payload.size();

        IRIS_LOG("[OFDM-RX] frame decoded OK: %d payload bytes (%d blocks), SNR=%.1f dB, mean|H|=%.2f",
                 (int)result.payload.size(), n_blocks, result.snr_db, result.mean_H_mag);
    }

    return finish_result();
}

bool OfdmDemodulator::extract_payload_blocks(const std::vector<uint8_t>& decoded_bits,
                                             LdpcRate rate,
                                             int expected_codeword_count,
                                             size_t exact_information_bit_length,
                                             std::vector<uint8_t>& payload_out) {
    const int k = LdpcCodec::block_size(rate);
    if (k < 48 || expected_codeword_count < 1 ||
        expected_codeword_count > static_cast<int>(v2::FrameGeometry::kMaxCodewordCount))
        return false;
    const size_t expected_bits = static_cast<size_t>(k) *
                                 static_cast<size_t>(expected_codeword_count);
    if (exact_information_bit_length != expected_bits ||
        decoded_bits.size() != exact_information_bit_length)
        return false;
    for (uint8_t bit : decoded_bits)
        if (bit > 1) return false;

    const int max_payload_per_block = k / 8 - 6;
    std::vector<uint8_t> full_payload;
    full_payload.reserve(static_cast<size_t>(max_payload_per_block) *
                         static_cast<size_t>(expected_codeword_count));
    bool saw_empty_trailer = false;

    for (int blk = 0; blk < expected_codeword_count; blk++) {
        const size_t bit_offset = static_cast<size_t>(blk) * static_cast<size_t>(k);
        uint16_t chunk_len = 0;
        for (int i = 0; i < 16; i++)
            chunk_len |= static_cast<uint16_t>(decoded_bits[bit_offset + i]) << i;

        if (static_cast<int>(chunk_len) > max_payload_per_block ||
            (chunk_len == 0 && blk == 0)) {
            IRIS_LOG("[OFDM-RX] block %d: payload_len=%d out of range [%s,%d]",
                     blk, static_cast<int>(chunk_len), blk == 0 ? "1" : "0",
                     max_payload_per_block);
            return false;
        }
        if (saw_empty_trailer && chunk_len != 0) {
            IRIS_LOG("[OFDM-RX] block %d: nonempty block follows empty trailer", blk);
            return false;
        }
        saw_empty_trailer = saw_empty_trailer || chunk_len == 0;

        const int total_bytes = 2 + static_cast<int>(chunk_len) + 4;
        const int total_bits_needed = total_bytes * 8;
        if (total_bits_needed > k) return false;

        std::vector<uint8_t> block_bytes(static_cast<size_t>(total_bytes), 0);
        for (int i = 0; i < total_bits_needed; i++)
            block_bytes[static_cast<size_t>(i / 8)] |=
                decoded_bits[bit_offset + static_cast<size_t>(i)] << (i % 8);

        const int crc_data_len = 2 + static_cast<int>(chunk_len);
        const uint32_t computed_crc = crc32(block_bytes.data(), crc_data_len);
        const uint32_t received_crc =
              static_cast<uint32_t>(block_bytes[crc_data_len + 0])
            | (static_cast<uint32_t>(block_bytes[crc_data_len + 1]) << 8)
            | (static_cast<uint32_t>(block_bytes[crc_data_len + 2]) << 16)
            | (static_cast<uint32_t>(block_bytes[crc_data_len + 3]) << 24);
        if (computed_crc != received_crc) {
            IRIS_LOG("[OFDM-RX] block %d CRC-32 fail: computed=0x%08X received=0x%08X (len=%d)",
                     blk, computed_crc, received_crc, chunk_len);
            return false;
        }

        for (int i = total_bits_needed; i < k; ++i) {
            if (decoded_bits[bit_offset + static_cast<size_t>(i)] != 0) {
                IRIS_LOG("[OFDM-RX] block %d: nonzero residual information bits", blk);
                return false;
            }
        }

        full_payload.insert(full_payload.end(), block_bytes.data() + 2,
                            block_bytes.data() + 2 + chunk_len);
    }

    payload_out = std::move(full_payload);
    return true;
}

} // namespace iris
