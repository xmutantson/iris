#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"   // get_uniform_tone_map
#include "common/fft.h"
#include "common/logging.h"
#include "native/constellation.h"  // demap_soft, bits_to_modulation (from ofdm_mod.h)
#include "native/nuc_tables.h"     // NucTable, get_nuc_table (for BPS)
#include "native/frame.h"      // crc32, KalmanTrace
#include <cmath>
#include <algorithm>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// ============================================================================
//  Blind Phase Search (BPS) — replaces DD-CPE for QAM16+
//
//  Two-stage search exploiting QAM 4-fold symmetry (±45° unambiguous range).
//  Stage 1: 8 coarse test angles in [-π/4, +π/4)
//  Stage 2: 8 fine angles around stage-1 winner
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
            uint8_t bits[8];
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
        // Without DFT-spread, cap at ~27 dB (0.002) for 256QAM headroom.
        constexpr float NV_ABS_FLOOR = 0.001f;  // -30 dB absolute
        constexpr float NV_REL_FLOOR = 0.002f;  // max 27 dB effective SNR per carrier
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
    std::vector<float>& llrs)
{
    const int n_used = config_.n_used_carriers;

    // Convert LdpcRate to rate_num/16 for NUC table lookup
    auto fec_to_rate16 = [](LdpcRate r) -> int {
        switch (r) {
            case LdpcRate::RATE_1_2: return 8;
            case LdpcRate::RATE_5_8: return 10;
            case LdpcRate::RATE_3_4: return 12;
            case LdpcRate::RATE_7_8: return 14;
            default: return 8;
        }
    };
    int fec_r16 = fec_to_rate16(tone_map.fec_rate);

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
        float sigma_sum = 0.0f;
        int count = 0;
        for (int i = 0; i < n_used && count < n_data; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float H_mag2 = std::norm(est.H[i]);
            float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 0.001f;
            constexpr float NV_ABS_FLOOR = 0.001f;
            constexpr float NV_REL_FLOOR = 0.002f;  // 27 dB cap — must match equalize_mmse
            float nv_floor = std::max(NV_ABS_FLOOR, NV_REL_FLOOR * H_mag2);
            if (nv < nv_floor) nv = nv_floor;
            sigma_sum += (H_mag2 + nv > 1e-12f) ? (nv / (H_mag2 + nv)) : 1.0f;
            count++;
        }
        dft_sigma_sq = sigma_sum / std::max(1, count);
    }

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
            constexpr float NV_REL_FLOOR = 0.002f;
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

        if (nuc) {
            // NUC max-log-MAP soft demapper
            size_t llr_start = llrs.size();
            llrs.resize(llr_start + bpc);
            demap_soft_nuc(eq_carriers[data_idx], sigma_sq, nuc, &llrs[llr_start]);
        } else {
            // Standard uniform QAM soft demapper (reuse pre-allocated buffers)
            sym_vec[0] = eq_carriers[data_idx];
            carrier_llrs = demap_soft(sym_vec, mod, sigma_sq);
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
    const ToneMap& tone_map, const OfdmSyncResult* pre_sync)
{
    OfdmDemodResult result;
    const int nfft = config_.nfft;
    const int cp = config_.cp_samples;
    const int sym_len = nfft + cp;
    const int n_used = config_.n_used_carriers;
    const int n_data = config_.n_data_carriers;

    // ---- 1. Frame detection ----
    OfdmSyncResult sync;
    if (pre_sync) {
        // Use pre-computed sync result (avoids redundant Schmidl-Cox)
        sync = *pre_sync;
    } else {
        sync = ofdm_detect_frame(iq, n_samples, config_);
    }
    if (!sync.detected) {
        IRIS_LOG("[OFDM-RX] no frame detected");
        return result;
    }

    result.snr_db = sync.snr_est;
    result.cfo_hz = sync.cfo_hz;

    int frame_start = sync.frame_start;  // Start of CP of training symbol 1

    // ---- 1b. Minimum frame guard ----
    // Need at least: 2 training + 1 sync word + 1 data + 1 tail = 5 symbols
    int min_frame_samples = 5 * sym_len;
    int remaining = n_samples - frame_start;
    if (remaining < min_frame_samples) {
        IRIS_LOG("[OFDM-RX] insufficient samples after detection: %d < %d (need 4 syms)",
                 remaining, min_frame_samples);
        // Don't consume samples — leave preamble in buffer for retry with more data.
        // samples_consumed = 0 signals "frame detected but incomplete, wait for more".
        result.samples_consumed = 0;
        return result;
    }

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
    // noise_var was estimated from boosted training symbols, so residuals are
    // also scaled by sqrt(2) → noise_var is 2x too high. Divide by boost².
    constexpr float PREAMBLE_BOOST = 1.4142f;
    constexpr float BOOST_SQ = PREAMBLE_BOOST * PREAMBLE_BOOST;  // 2.0
    for (auto& h : channel_est_.H)
        h /= PREAMBLE_BOOST;
    for (auto& nv : channel_est_.noise_var) {
        nv /= BOOST_SQ;
        if (nv < 1e-6f) nv = 1e-6f;
    }
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
        // Compensate preamble boost again after re-estimation
        for (auto& h : channel_est_.H)
            h /= PREAMBLE_BOOST;
        for (auto& nv : channel_est_.noise_var) {
            nv /= BOOST_SQ;
            if (nv < 1e-6f) nv = 1e-6f;
        }

        IRIS_LOG("[OFDM-RX] CFO refined: coarse=%.1f Hz + fine=%.2f Hz = %.2f Hz",
                 sync.cfo_hz, fine_cfo, total_cfo);
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
            result.samples_consumed = frame_start + 3 * sym_len;
            return result;
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

    // ---- 5. Mean |H| quality gate (Mercury approach) ----
    // Skip decode if channel estimate is too weak — avoids wasting LDPC cycles.
    {
        float h_sum = 0.0f;
        for (auto& h : channel_est_.H)
            h_sum += std::abs(h);
        float mean_H = channel_est_.H.empty() ? 0.0f : h_sum / channel_est_.H.size();
        result.mean_H_mag = mean_H;
        if (mean_H < 0.05f) {
            IRIS_LOG("[OFDM-RX] quality gate: mean|H|=%.3f < 0.05, skipping decode", mean_H);
            result.samples_consumed = frame_start + 3 * sym_len;  // skip past preamble + sync
            return result;
        }
    }

    // ---- 6. Use pre-negotiated tone map (no header — Mercury approach) ----
    const ToneMap& active_tone_map = tone_map;
    result.fec_rate = active_tone_map.fec_rate;

    int bps_total = active_tone_map.total_bits_per_symbol;
    if (bps_total <= 0) {
        IRIS_LOG("[OFDM-RX] tone map has 0 bits per symbol");
        return result;
    }

    // ---- 7. Calculate number of data symbols (n_codewords LDPC blocks) ----
    // n_codewords comes from the tone_map (set by session based on batch size).
    // Default 1 for backward compatibility.
    int n_codewords = std::max(1, active_tone_map.n_codewords);
    int coded_bits_total;
    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        coded_bits_total = n_codewords * LdpcCodec::codeword_size(active_tone_map.fec_rate);
        result.n_ldpc_blocks = n_codewords;
    } else {
        int k = LdpcCodec::block_size(active_tone_map.fec_rate);
        coded_bits_total = (k > 0) ? k : bps_total;
        result.n_ldpc_blocks = 0;
    }

    int n_data_symbols = (coded_bits_total + bps_total - 1) / bps_total;
    result.n_data_symbols = n_data_symbols;

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
    //   - Pilot rows (every 5 data sym): all carriers, ~6° RMS, r=0.01
    //   - Block pilots (every 24 data sym): all carriers, ~3° RMS, r=0.01
    //   - Comb pilots (every sym): 4-7 carriers, ~17° RMS, r=0.09
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
    // Reference: NFFT=512, CP=32 → 544/48000 = 11.33ms. Values tuned empirically.
    // Phase variance ~ √t (Wiener process), freq drift ~ t, accel ~ t^1.5.
    // Using conservative sqrt scaling to keep Kalman responsive to measurements.
    const float sym_t = config_.symbol_duration_s();
    const float ref_t = 544.0f / 48000.0f;  // reference period (NFFT=512, CP=32)
    const float t_ratio = sym_t / ref_t;
    const float q_phase = 1e-3f * std::sqrt(t_ratio);   // rad²/symbol
    const float q_freq  = 1e-4f * t_ratio;              // rad²/symbol³
    const float q_accel = 1e-6f * t_ratio * std::sqrt(t_ratio);  // rad²/symbol⁵
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

    for (int s = 0; s < n_data_symbols; s++) {
        // Check for block pilot: every pilot_symbol_spacing data symbols
        if (data_sym_count > 0 &&
            (data_sym_count % config_.pilot_symbol_spacing) == 0)
        {
            if (pos + sym_len > remaining) {
                IRIS_LOG("[OFDM-RX] insufficient samples for block pilot at data_sym %d", s);
                return result;
            }

            const std::complex<float>* pilot_body = iq_corrected.data() + pos + cp;
            std::copy(pilot_body, pilot_body + nfft, fft_buf.data());
            tdf_derotate(fft_buf.data(), ks);
            fft_complex(fft_buf.data(), nfft);

            // Block pilot → Kalman measurement (high quality, all carriers)
            float bp_cpe = measure_pilot_cpe(fft_buf.data());
            float z = bp_cpe - ks.phase;
            while (z > (float)M_PI) z -= 2*(float)M_PI;
            while (z < -(float)M_PI) z += 2*(float)M_PI;
            IRIS_LOG("[OFDM-RX] block pilot: meas=%.1f pred=%.1f innov=%.1f deg",
                     bp_cpe * 180.0f / (float)M_PI,
                     ks.phase * 180.0f / (float)M_PI,
                     z * 180.0f / (float)M_PI);
            kalman_update(ks, z, r_pilot);

            pos += sym_len;
            total_symbols_elapsed++;
        }

        // Check for dense pilot row: every pilot_row_spacing data symbols
        if (config_.pilot_row_spacing > 0 && data_sym_count > 0 &&
            (data_sym_count % config_.pilot_row_spacing) == 0)
        {
            if (pos + sym_len > remaining) {
                IRIS_LOG("[OFDM-RX] insufficient samples for pilot row at data_sym %d", data_sym_count);
                return result;
            }

            const std::complex<float>* pr_body = iq_corrected.data() + pos + cp;
            std::copy(pr_body, pr_body + nfft, fft_buf.data());
            tdf_derotate(fft_buf.data(), ks);
            fft_complex(fft_buf.data(), nfft);

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
            return result;
        }

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

        // MMSE equalize
        equalize_mmse(data_carriers_buf, n_data_actual, channel_est_, eq_carriers_buf);

        // ---- DFT-despread (SC-FDMA IDFT) ----
        if (config_.dft_spread && n_data_actual > 1) {
            std::vector<std::complex<float>> idft_out(n_data_actual);
            small_idft(eq_carriers_buf.data(), idft_out.data(), n_data_actual);
            std::copy(idft_out.begin(), idft_out.end(), eq_carriers_buf.begin());
        }

        // ---- BPS (Blind Phase Search) for QAM16+ ----
        // Replaces DD-CPE. ±45° unambiguous range via QAM 4-fold symmetry.
        // Quadrant ambiguity resolved by Kalman prediction (anchored to pilots).
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

    // ---- RTS re-correction + deferred soft demapping ----
    // Apply (smoothed - forward) residual to stored eq carriers, then demap
    result.eq_constellation.clear();
    result.eq_constellation.reserve(data_sym_count * n_data);

    for (int k = 0; k < data_sym_count; k++) {
        int nd = stored_n_data[k];
        if (nd <= 0) continue;

        // Phase correction residual: smoothed minus what was applied in forward pass
        float residual = smoothed_phase[k] - stored_fwd_phase[k];

        if (std::abs(residual) > 1e-6f) {
            std::complex<float> rts_rot(std::cos(-residual), std::sin(-residual));
            for (int i = 0; i < nd; i++)
                stored_eq[k][i] *= rts_rot;
        }

        // Store for constellation GUI
        result.eq_constellation.insert(result.eq_constellation.end(),
                                        stored_eq[k].begin(),
                                        stored_eq[k].begin() + nd);

        // Soft demap
        std::copy(stored_eq[k].begin(), stored_eq[k].begin() + nd,
                  eq_carriers_buf.begin());
        demap_to_llrs(eq_carriers_buf, nd, active_tone_map, channel_est_, all_llrs);
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

    // Skip tail symbol (1 block-pilot)
    pos += sym_len;

    result.samples_consumed = frame_start + pos;
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
        IRIS_LOG("[OFDM-RX] LLR stats: max_abs=%.1f, clamp=±%.0f (bpc=%d), %d/%d clamped",
                 max_abs, llr_clamp, max_bpc_for_clamp, n_clamped, (int)all_llrs.size());
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

    // ---- 11b. Tail bit hardening (deferred) ----
    // Payload length is unknown until after LDPC decode (embedded in data).
    // Light tail hardening: bias last 16 systematic bits toward 0 as likely padding.
    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        int k = LdpcCodec::block_size(active_tone_map.fec_rate);
        if (k > 16) {
            int n_tail = 0;
            for (int i = k - 16; i < k; i++) {
                if (i < (int)all_llrs.size()) {
                    all_llrs[i] = std::max(all_llrs[i], 5.0f);  // weak "0" prior
                    n_tail++;
                }
            }
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
            return result;
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
    {
        int k = LdpcCodec::block_size(active_tone_map.fec_rate);
        int max_payload_per_block = (k > 0) ? (k / 8 - 4 - 2) : (int)decoded_bits.size() / 8;
        int bits_per_block = (k > 0) ? k : (int)decoded_bits.size();

        std::vector<uint8_t> full_payload;
        int n_blocks = n_ldpc_blocks;
        if (n_blocks == 0) n_blocks = 1;  // no-FEC fallback

        for (int blk = 0; blk < n_blocks; blk++) {
            int bit_offset = blk * bits_per_block;
            int bits_avail = (int)decoded_bits.size() - bit_offset;
            if (bits_avail < 16) {
                IRIS_LOG("[OFDM-RX] block %d: decoded bits too short (%d)", blk, bits_avail);
                return result;
            }

            // Extract 2-byte length prefix
            uint16_t chunk_len = 0;
            for (int i = 0; i < 16; i++)
                chunk_len |= ((uint16_t)(decoded_bits[bit_offset + i] & 1)) << i;

            if ((int)chunk_len > max_payload_per_block || (chunk_len == 0 && blk == 0)) {
                IRIS_LOG("[OFDM-RX] block %d: payload_len=%d out of range [%s,%d]",
                         blk, (int)chunk_len, blk == 0 ? "1" : "0", max_payload_per_block);
                return result;
            }

            // Empty trailing blocks are OK (last block may have no data)
            if (chunk_len == 0) continue;

            int total_bytes = 2 + (int)chunk_len + 4;
            int total_bits_needed = total_bytes * 8;
            if (bits_avail < total_bits_needed) {
                IRIS_LOG("[OFDM-RX] block %d: decoded bits insufficient: %d < %d",
                         blk, bits_avail, total_bits_needed);
                return result;
            }

            std::vector<uint8_t> block_bytes(total_bytes, 0);
            for (int i = 0; i < total_bits_needed; i++)
                block_bytes[i / 8] |= (decoded_bits[bit_offset + i] << (i % 8));

            // CRC-32 verification
            int crc_data_len = 2 + (int)chunk_len;
            uint32_t computed_crc = crc32(block_bytes.data(), crc_data_len);
            uint32_t received_crc = (uint32_t)block_bytes[crc_data_len + 0]
                                  | ((uint32_t)block_bytes[crc_data_len + 1] << 8)
                                  | ((uint32_t)block_bytes[crc_data_len + 2] << 16)
                                  | ((uint32_t)block_bytes[crc_data_len + 3] << 24);

            if (computed_crc != received_crc) {
                IRIS_LOG("[OFDM-RX] block %d CRC-32 fail: computed=0x%08X received=0x%08X (len=%d)",
                         blk, computed_crc, received_crc, chunk_len);
                return result;
            }

            full_payload.insert(full_payload.end(),
                                block_bytes.data() + 2,
                                block_bytes.data() + 2 + chunk_len);
        }

        result.success = true;
        result.payload = std::move(full_payload);
        result.payload_len = (uint16_t)result.payload.size();

        IRIS_LOG("[OFDM-RX] frame decoded OK: %d payload bytes (%d blocks), SNR=%.1f dB, mean|H|=%.2f",
                 (int)result.payload.size(), n_blocks, result.snr_db, result.mean_H_mag);
    }

    return result;
}

} // namespace iris
