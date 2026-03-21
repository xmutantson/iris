#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"   // get_uniform_tone_map
#include "common/fft.h"
#include "common/logging.h"
#include "native/constellation.h"  // demap_soft, bits_to_modulation (from ofdm_mod.h)
#include "native/nuc_tables.h"     // NucTable, get_nuc_table (for DD CPE)
#include "native/frame.h"      // crc32
#include <cmath>
#include <algorithm>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

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
    // subcarriers. Measure actual EVM from hard decisions to capture PAPR
    // clipping distortion that the channel estimator cannot see.
    float dft_sigma_sq = 1.0f;
    if (config_.dft_spread && n_data > 1) {
        // First pass: measure EVM from hard decisions
        float evm_sum = 0.0f;
        int evm_count = 0;
        int data_idx_tmp = 0;
        for (int i = 0; i < n_used && data_idx_tmp < n_data; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            int bpc_tmp = (data_idx_tmp < tone_map.n_data_carriers)
                          ? tone_map.bits_per_carrier[data_idx_tmp] : 0;
            if (bpc_tmp > 0) {
                Modulation mod_tmp = bits_to_modulation(bpc_tmp);
                std::complex<float> rx = eq_carriers[data_idx_tmp];
                // Hard-slice to nearest constellation point
                uint8_t bits_tmp[8];
                demap_symbol(rx, bits_tmp, mod_tmp);
                std::complex<float> ref = map_symbol(bits_tmp, mod_tmp);
                evm_sum += std::norm(rx - ref);
                evm_count++;
            }
            data_idx_tmp++;
        }
        float measured_evm = (evm_count > 0) ? (evm_sum / evm_count) : 0.01f;

        // Also compute channel-estimator-based sigma_sq
        float sigma_sum = 0.0f;
        data_idx_tmp = 0;
        for (int i = 0; i < n_used && data_idx_tmp < n_data; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float H_mag2 = std::norm(est.H[i]);
            float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 0.001f;
            constexpr float NV_ABS_FLOOR = 0.001f;
            constexpr float NV_REL_FLOOR = 0.005f;
            float nv_floor = std::max(NV_ABS_FLOOR, NV_REL_FLOOR * H_mag2);
            if (nv < nv_floor) nv = nv_floor;
            sigma_sum += (H_mag2 + nv > 1e-12f) ? (nv / (H_mag2 + nv)) : 1.0f;
            data_idx_tmp++;
        }
        float ch_sigma = sigma_sum / std::max(1, data_idx_tmp);

        // Use the larger of channel estimate and measured EVM
        // This ensures PAPR clipping distortion is properly accounted for
        dft_sigma_sq = std::max(ch_sigma, measured_evm);
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
    // Need at least: 2 training + 1 data + 1 tail = 4 symbols (no header)
    int min_frame_samples = 4 * sym_len;
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
            result.samples_consumed = frame_start + 2 * sym_len;  // skip past preamble
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

    // ---- 8. Receive data symbols (with block pilots) ----
    std::vector<float> all_llrs;
    all_llrs.reserve(coded_bits_total);

    int data_sym_count = 0;  // count of data symbols received so far

    // H8: SFO tracking — save initial channel estimate for phase drift measurement
    std::vector<std::complex<float>> H_initial = channel_est_.H;
    float sfo_ppm_estimate = 0.0f;       // current SFO estimate (ppm)
    int sfo_block_pilot_count = 0;        // number of block pilots seen
    float sfo_cumulative_phase = 0.0f;    // sum of mean phase drifts
    float sfo_cumulative_weight = 0.0f;   // sum of symbol indices (for weighted slope)
    int total_symbols_elapsed = 0;        // total symbols since training (incl. header)

    // CPE: 2nd-order post-FFT phase tracker (PLL).
    // Stage 1: Predict phase from running estimate, pre-correct carriers.
    //   Removes bulk of drift so comb pilots measure only a small residual.
    // Stage 2: Comb-pilot CPE measures residual. DD-CPE refines further.
    // Stage 3: Update tracker with total measured phase (prediction + residuals).
    // Operates post-FFT on frequency-domain carriers. Previous pre-DFT PLL
    // (alpha=0.6, beta=0.15) caused +15 dB phantom noise because PLL error
    // fed back through the FFT. Post-FFT application avoids this.
    float cpe_phase = 0.0f;   // per-symbol residual CPE from comb pilots
    float cpe_total = 0.0f;   // total phase correction (prediction + residual + DD)
    std::vector<float> cpe_history;  // per-symbol TOTAL phase for slope estimation
    cpe_history.reserve(n_data_symbols);
    // Phase tracker state (2nd-order: phase + frequency)
    float phase_pred = 0.0f;  // predicted phase for next symbol (rad)
    float freq_pred = 0.0f;   // phase rate estimate (rad/symbol)
    float last_total = 0.0f;  // total phase from previous symbol


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

    for (int s = 0; s < n_data_symbols; s++) {
        // Check for block pilot: every pilot_symbol_spacing data symbols
        if (data_sym_count > 0 &&
            (data_sym_count % config_.pilot_symbol_spacing) == 0)
        {
            // This position is a block pilot symbol
            if (pos + sym_len > remaining) {
                IRIS_LOG("[OFDM-RX] insufficient samples for block pilot at data_sym %d", s);
                return result;
            }

            // FFT the block pilot (no pre-DFT phase ramp — no PLL)
            const std::complex<float>* pilot_body = iq_corrected.data() + pos + cp;
            std::copy(pilot_body, pilot_body + nfft, fft_buf.data());
            fft_complex(fft_buf.data(), nfft);

            // Block pilot CPE: SNR-weighted phase of all carriers
            {
                float bp_phase_sum = 0.0f;
                float bp_weight_sum = 0.0f;
                for (int i = 0; i < n_used; i++) {
                    int bin = config_.used_carrier_bins[i];
                    std::complex<float> Y_bp = fft_buf[bin];
                    std::complex<float> H = channel_est_.H[i];
                    float Hm2 = std::norm(H);
                    float nv = (i < (int)channel_est_.noise_var.size())
                               ? channel_est_.noise_var[i] : 1e-6f;
                    if (nv < 1e-12f) nv = 1e-12f;
                    float w = Hm2 / nv;
                    if (w < 1e-6f) continue;
                    float phase_err = std::arg(Y_bp * std::conj(H));
                    bp_phase_sum += w * phase_err;
                    bp_weight_sum += w;
                }
                if (bp_weight_sum > 0.0f) {
                    float bp_cpe = bp_phase_sum / bp_weight_sum;
                    IRIS_LOG("[OFDM-RX] block pilot CPE: %.2f deg (applied)",
                             bp_cpe * 180.0f / (float)M_PI);

                    // Apply block pilot CPE to channel estimate
                    std::complex<float> bp_rot(std::cos(bp_cpe), std::sin(bp_cpe));
                    for (auto& h : channel_est_.H)
                        h *= bp_rot;

                    // Update PLL tracker with high-quality block pilot observation
                    float bp_total = phase_pred + bp_cpe;
                    if (data_sym_count > 0) {
                        float measured_slope = bp_total - last_total;
                        while (measured_slope > (float)M_PI) measured_slope -= 2.0f * (float)M_PI;
                        while (measured_slope < -(float)M_PI) measured_slope += 2.0f * (float)M_PI;
                        freq_pred = 0.7f * freq_pred + 0.3f * measured_slope;
                    }
                    last_total = bp_total;
                    phase_pred = bp_total + freq_pred;
                }
            }

            // Block pilot channel update: DISABLED for now.
            // With pre-DFT CFO correction active, block pilots may become usable.
            // TODO: re-enable after OTA validation with pre-DFT CFO.
            // ofdm_update_channel(channel_est_, fft_buf.data(), config_);

            // H8: SFO estimation from block pilot phase drift
            {
                float phase_sum = 0.0f;
                float weight_sum = 0.0f;
                for (int i = 0; i < n_used; i++) {
                    float w = std::norm(H_initial[i]);
                    if (w < 1e-12f) continue;
                    // Phase difference between current H and initial H
                    float dp = std::arg(channel_est_.H[i] * std::conj(H_initial[i]));
                    phase_sum += w * dp;
                    weight_sum += w;
                }
                if (weight_sum > 0.0f) {
                    float mean_phase = phase_sum / weight_sum;
                    // Symbol index relative to training symbol 2
                    int sym_idx = data_sym_count + sfo_block_pilot_count + 1;
                    sfo_block_pilot_count++;
                    sfo_cumulative_phase += mean_phase * sym_idx;
                    sfo_cumulative_weight += (float)(sym_idx * sym_idx);

                    if (sfo_cumulative_weight > 0.0f) {
                        // Linear fit: phase = slope * sym_idx
                        // slope = sum(phase_i * sym_idx_i) / sum(sym_idx_i^2)
                        float slope = sfo_cumulative_phase / sfo_cumulative_weight;
                        // SFO in ppm: slope / (2*pi * sym_len/sample_rate) * 1e-6
                        // Actually: sfo_ppm = slope / (2*pi) * sample_rate / sym_len * 1e6
                        // But: slope is phase per symbol, and each symbol is sym_len samples
                        float sfo_ppm_raw = slope * (float)config_.sample_rate
                                          / (2.0f * (float)M_PI * (float)sym_len) * 1e6f;
                        // Only apply if > 5 ppm threshold (avoid correcting noise)
                        // Cap at 200 ppm — real clock SFO is <100 ppm; anything
                        // larger is a corrupted block pilot (e.g. FM deviation clipping).
                        if (std::abs(sfo_ppm_raw) > 5.0f && std::abs(sfo_ppm_raw) < 200.0f) {
                            sfo_ppm_estimate = sfo_ppm_raw;
                            IRIS_LOG("[OFDM-RX] SFO estimate: %.1f ppm (sym_idx=%d, slope=%.5f rad/sym)",
                                     sfo_ppm_estimate, sym_idx, slope);
                        } else if (std::abs(sfo_ppm_raw) >= 200.0f) {
                            IRIS_LOG("[OFDM-RX] SFO estimate rejected: %.1f ppm (too large, block pilot likely corrupted)", sfo_ppm_raw);
                        }
                    }
                }
            }

            pos += sym_len;
            total_symbols_elapsed++;
            // Do not increment data_sym_count for block pilot
        }

        // Check for dense pilot row: every pilot_row_spacing data symbols.
        // These provide channel re-estimation within long frames, complementing
        // the less-frequent block pilots. Must match TX insertion pattern.
        if (config_.pilot_row_spacing > 0 && data_sym_count > 0 &&
            (data_sym_count % config_.pilot_row_spacing) == 0)
        {
            if (pos + sym_len > remaining) {
                IRIS_LOG("[OFDM-RX] insufficient samples for pilot row at data_sym %d", data_sym_count);
                return result;
            }

            // FFT the pilot row (no pre-DFT phase ramp — no PLL)
            const std::complex<float>* pr_body = iq_corrected.data() + pos + cp;
            std::copy(pr_body, pr_body + nfft, fft_buf.data());
            fft_complex(fft_buf.data(), nfft);

            // Pilot row CPE: SNR-weighted phase of all carriers.
            // Weight by |H|²/noise_var (= SNR) instead of |H|² alone.
            // FM discriminator noise is f²-shaped, so high-frequency carriers
            // have poor SNR even with large |H|. SNR weighting excludes them.
            {
                float pr_phase_sum = 0.0f;
                float pr_weight_sum = 0.0f;
                for (int i = 0; i < n_used; i++) {
                    int bin = config_.used_carrier_bins[i];
                    std::complex<float> Y_pr = fft_buf[bin];
                    std::complex<float> H = channel_est_.H[i];
                    float Hm2 = std::norm(H);
                    float nv = (i < (int)channel_est_.noise_var.size())
                               ? channel_est_.noise_var[i] : 1e-6f;
                    if (nv < 1e-12f) nv = 1e-12f;
                    float w = Hm2 / nv;  // SNR-based weight
                    if (w < 1e-6f) continue;
                    float phase_err = std::arg(Y_pr * std::conj(H));
                    pr_phase_sum += w * phase_err;
                    pr_weight_sum += w;
                }
                if (pr_weight_sum > 0.0f) {
                    float pr_cpe = pr_phase_sum / pr_weight_sum;
                    IRIS_LOG("[OFDM-RX] pilot row %d: cpe=%.2f deg (applied)",
                             data_sym_count / config_.pilot_row_spacing,
                             pr_cpe * 180.0f / (float)M_PI);

                    // Apply dense pilot row CPE to channel estimate (80 carriers
                    // vs 13 comb pilots — 6x more phase information, ~2.5x lower
                    // estimation noise). Rotates H so subsequent data symbols see
                    // corrected phase reference.
                    std::complex<float> pr_rot(std::cos(pr_cpe), std::sin(pr_cpe));
                    for (auto& h : channel_est_.H)
                        h *= pr_rot;

                    // Update PLL tracker with high-quality pilot row observation
                    float pr_total = phase_pred + pr_cpe;
                    if (data_sym_count > 0) {
                        float measured_slope = pr_total - last_total;
                        while (measured_slope > (float)M_PI) measured_slope -= 2.0f * (float)M_PI;
                        while (measured_slope < -(float)M_PI) measured_slope += 2.0f * (float)M_PI;
                        freq_pred = 0.7f * freq_pred + 0.3f * measured_slope;
                    }
                    last_total = pr_total;
                    phase_pred = pr_total + freq_pred;


                }
            }

            pos += sym_len;
            total_symbols_elapsed++;
            // Do not increment data_sym_count for pilot row
        }

        // Now receive the actual data symbol
        if (pos + sym_len > remaining) {
            IRIS_LOG("[OFDM-RX] insufficient samples for data symbol %d/%d", s, n_data_symbols);
            return result;
        }

        // Skip CP, FFT. No pre-DFT phase ramp — CPE is corrected post-FFT
        // per-symbol, matching 802.11/DVB-T2/FreeDV standard practice.
        const std::complex<float>* sym_body = iq_corrected.data() + pos + cp;
        std::copy(sym_body, sym_body + nfft, fft_buf.data());
        fft_complex(fft_buf.data(), nfft);

        // Comb pilot IIR channel update: DISABLED.
        // OTA test (2026-03-19): comb pilots still corrupted by FM channel
        // even with PAPR clipping. DD-CPE corrections 10-28° indicate IIR
        // is blending garbage into H[k]. Preamble estimate is sufficient.
        // ofdm_interpolate_pilots(channel_est_, fft_buf.data(), config_);

        // Extract values at used carrier bins (reuse pre-allocated buffer)
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            used_carriers_buf[i] = fft_buf[bin];
        }

        // H8: Apply SFO correction if estimate exceeds threshold
        if (std::abs(sfo_ppm_estimate) > 5.0f) {
            int sym_idx = total_symbols_elapsed + 1;
            float elapsed_samples = (float)(sym_idx * sym_len);
            float sfo_frac = sfo_ppm_estimate * 1e-6f;  // fractional SFO
            for (int i = 0; i < n_used; i++) {
                int bin = config_.used_carrier_bins[i];
                float phase_corr = -2.0f * (float)M_PI * (float)bin
                                   * sfo_frac * elapsed_samples / (float)nfft;
                std::complex<float> rot(std::cos(phase_corr), std::sin(phase_corr));
                used_carriers_buf[i] *= rot;
            }
        }

        // PLL phase prediction: pre-correct using running estimate before pilots
        // measure the residual. Prediction is extrapolated from previous symbols'
        // total corrections. Pilots see a small residual (~5° vs ~100° uncorrected).
        float sym_prediction = 0.0f;
        if (data_sym_count > 0) {
            sym_prediction = phase_pred;
            std::complex<float> pred_rot(std::cos(-sym_prediction), std::sin(-sym_prediction));
            for (int i = 0; i < n_used; i++)
                used_carriers_buf[i] *= pred_rot;
        }

        // Per-symbol CPE: SNR-weighted phase of comb pilots.
        // Measures residual phase after PLL prediction pre-correction.
        // SNR weighting (|H|²/noise_var) naturally excludes pilots on
        // deep-nulled carriers where FM f²-noise dominates.
        {
            float cpe_num = 0.0f;
            float cpe_den = 0.0f;
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
                cpe_num += w * phase_diff;
                cpe_den += w;
            }
            if (cpe_den > 0.0f) {
                cpe_phase = cpe_num / cpe_den;  // independent per-symbol residual

                // Apply residual CPE correction to all used carriers
                std::complex<float> cpe_rot(std::cos(-cpe_phase), std::sin(-cpe_phase));
                for (int i = 0; i < n_used; i++) {
                    used_carriers_buf[i] *= cpe_rot;
                }
                if (data_sym_count % 20 == 0 || std::abs(cpe_phase) > 0.1f) {
                    IRIS_LOG("[OFDM-RX] CPE sym %d: pred=%.1f resid=%.1f total=%.1f deg",
                             data_sym_count,
                             sym_prediction * 180.0f / (float)M_PI,
                             cpe_phase * 180.0f / (float)M_PI,
                             (sym_prediction + cpe_phase) * 180.0f / (float)M_PI);
                }
            }
        }

        // Extract data carriers (skip pilot positions, reuse pre-allocated buffer)
        int n_data_actual = extract_data_carriers(used_carriers_buf.data(), n_used,
                                                  data_carriers_buf);

        // MMSE equalize (reuse pre-allocated buffer)
        equalize_mmse(data_carriers_buf, n_data_actual, channel_est_, eq_carriers_buf);

        // ---- DFT-despread (SC-FDMA IDFT) ----
        // If DFT-spreading was used on TX, apply IDFT to equalized data
        // carriers to recover QAM symbols. Must happen BEFORE DD-CPE and
        // soft demapping since the equalized carriers are in the DFT domain.
        if (config_.dft_spread && n_data_actual > 1) {
            std::vector<std::complex<float>> idft_out(n_data_actual);
            small_idft(eq_carriers_buf.data(), idft_out.data(), n_data_actual);
            std::copy(idft_out.begin(), idft_out.end(), eq_carriers_buf.begin());
        }

        // Decision-directed CPE refinement: use ALL data carriers (not just
        // 5 comb pilots) to refine the phase estimate. Hard-slice each
        // equalized carrier, compute phase error vs nearest constellation
        // point, weighted-average across all carriers.
        // DISABLED for 64QAM+ (bpc >= 6): FM deviation limiter compresses
        // outer constellation points, causing systematic hard-decision errors
        // that bias the phase estimate. At 64QAM's 13.3° decision boundary,
        // even 2-3° systematic error is destructive. 5 comb pilots provide
        // adequate CPE for 64QAM via the per-symbol correction above.
        float dd_correction_applied = 0.0f;
        {
            int max_bpc = 1;
            for (int k = 0; k < active_tone_map.n_data_carriers; k++)
                if (active_tone_map.bits_per_carrier[k] > max_bpc)
                    max_bpc = active_tone_map.bits_per_carrier[k];
            // DD-CPE after DFT-despread: IDFT recovers time-domain symbols in
            // original constellation domain. BPSK/QPSK hard-slicing is trivially
            // correct post-IDFT. Only disable for 64QAM+ (FM limiter compression).
            bool dd_cpe_enabled = (max_bpc < 6);
          if (dd_cpe_enabled) {
            // FEC rate for NUC table lookup
            auto fec_r16 = [&]() -> int {
                switch (active_tone_map.fec_rate) {
                    case LdpcRate::RATE_1_2: return 8;
                    case LdpcRate::RATE_5_8: return 10;
                    case LdpcRate::RATE_3_4: return 12;
                    case LdpcRate::RATE_7_8: return 14;
                    default: return 8;
                }
            }();

            float dd_num = 0.0f;
            float dd_den = 0.0f;
            int dd_count = 0;
            int data_idx = 0;
            const int n_used = config_.n_used_carriers;

            for (int i = 0; i < n_used && data_idx < n_data_actual; i++) {
                if (i % config_.pilot_carrier_spacing == 0) continue;

                int bpc = (data_idx < active_tone_map.n_data_carriers)
                          ? active_tone_map.bits_per_carrier[data_idx] : 0;
                if (bpc == 0) { data_idx++; continue; }

                std::complex<float> eq = eq_carriers_buf[data_idx];
                Modulation mod = bits_to_modulation(bpc);
                std::complex<float> ref;

                const NucTable* nuc = nullptr;
                if (active_tone_map.use_nuc && bpc >= 4)
                    nuc = get_nuc_table(mod, fec_r16);

                if (nuc) {
                    float best_dist = 1e30f;
                    int best_idx = 0;
                    if (!nuc->separable) {
                        for (int s = 0; s < nuc->n_points_2d; s++) {
                            float d = std::norm(eq - nuc->points_2d[s]);
                            if (d < best_dist) { best_dist = d; best_idx = s; }
                        }
                        ref = nuc->points_2d[best_idx];
                    } else {
                        int side = nuc->n_axis_1d;
                        float best_i_dist = 1e30f, best_q_dist = 1e30f;
                        int best_i = 0, best_q = 0;
                        for (int s = 0; s < side; s++) {
                            float di = (eq.real() - nuc->axis_1d[s]);
                            if (di * di < best_i_dist) { best_i_dist = di * di; best_i = s; }
                            float dq = (eq.imag() - nuc->axis_1d[s]);
                            if (dq * dq < best_q_dist) { best_q_dist = dq * dq; best_q = s; }
                        }
                        ref = std::complex<float>(nuc->axis_1d[best_i], nuc->axis_1d[best_q]);
                    }
                } else {
                    uint8_t bits[8];
                    demap_symbol(eq, bits, mod);
                    ref = map_symbol(bits, mod);
                }

                float ref_mag2 = std::norm(ref);
                if (ref_mag2 > 1e-12f) {
                    float phase_err = std::arg(eq * std::conj(ref));
                    dd_num += ref_mag2 * phase_err;
                    dd_den += ref_mag2;
                    dd_count++;
                }
                data_idx++;
            }

            if (dd_den > 0.0f && dd_count >= 5) {
                float dd_correction = dd_num / dd_den;

                // Clamp: ±5° for 16QAM, ±30° for BPSK/QPSK
                float dd_max_deg = (max_bpc >= 4) ? 5.0f : 30.0f;
                float DD_MAX_RAD = dd_max_deg * (float)M_PI / 180.0f;
                dd_correction = std::clamp(dd_correction, -DD_MAX_RAD, DD_MAX_RAD);

                std::complex<float> dd_rot(std::cos(-dd_correction), std::sin(-dd_correction));
                for (int i = 0; i < n_data_actual; i++)
                    eq_carriers_buf[i] *= dd_rot;
                dd_correction_applied = dd_correction;

                if (data_sym_count % 20 == 0 || std::abs(dd_correction) > 0.03f) {
                    IRIS_LOG("[OFDM-RX] DD-CPE sym %d: %.2f deg (%d carriers)",
                             data_sym_count, dd_correction * 180.0f / (float)M_PI, dd_count);
                }
            }
          } // dd_cpe_enabled
        }

        // Phase tracker update: accumulate total phase and update frequency estimate.
        // Total phase = prediction + pilot residual + DD refinement.
        // DD-CPE is included in PLL state: for BPSK/QPSK, hard decisions are
        // reliable and provide extra phase information that speeds PLL convergence.
        // For 16QAM, DD-CPE is clamped to ±5° (limiting damage from bad decisions).
        // DD-CPE is disabled entirely for 64QAM+ (line above).
        {
            cpe_total = sym_prediction + cpe_phase + dd_correction_applied;
            cpe_history.push_back(cpe_total);

            if (data_sym_count > 0) {
                float measured_slope = cpe_total - last_total;
                while (measured_slope > (float)M_PI) measured_slope -= 2.0f * (float)M_PI;
                while (measured_slope < -(float)M_PI) measured_slope += 2.0f * (float)M_PI;
                float pll_alpha = 0.25f;
                if (std::abs(cpe_phase) > 0.15f)  // >8.6° residual
                    pll_alpha = 0.45f;
                freq_pred = (1.0f - pll_alpha) * freq_pred + pll_alpha * measured_slope;
            }
            last_total = cpe_total;
            phase_pred = cpe_total + freq_pred;
        }

        // Store equalized constellation for GUI scatter plot
        result.eq_constellation.insert(result.eq_constellation.end(),
                                        eq_carriers_buf.begin(),
                                        eq_carriers_buf.begin() + n_data_actual);

        // Soft demap to LLRs
        demap_to_llrs(eq_carriers_buf, n_data_actual, active_tone_map, channel_est_, all_llrs);

        pos += sym_len;
        data_sym_count++;
        total_symbols_elapsed++;
    }

    // ---- CPE slope feedback: estimate residual CFO from per-symbol CPE drift ----
    // Linear regression of CPE(n) vs symbol index n gives slope in rad/symbol.
    // Convert to Hz: residual_cfo = slope * symbol_rate / (2π)
    if (cpe_history.size() >= 3) {
        float symbol_rate = config_.sample_rate / (float)sym_len;
        int N = (int)cpe_history.size();
        // Least-squares slope: slope = Σ((n - n̄)(y - ȳ)) / Σ((n - n̄)²)
        float n_mean = (N - 1) * 0.5f;
        float y_mean = 0.0f;
        for (int i = 0; i < N; i++) y_mean += cpe_history[i];
        y_mean /= N;
        float num = 0.0f, den = 0.0f;
        for (int i = 0; i < N; i++) {
            float dn = (float)i - n_mean;
            num += dn * (cpe_history[i] - y_mean);
            den += dn * dn;
        }
        float slope = (den > 1e-12f) ? (num / den) : 0.0f;  // rad/symbol
        float residual_cfo_hz = slope * symbol_rate / (2.0f * (float)M_PI);
        result.cpe_drift_hz = residual_cfo_hz;
        IRIS_LOG("[OFDM-RX] CPE slope: %.3f deg/sym -> residual CFO %.2f Hz (%d symbols)",
                 slope * 180.0f / (float)M_PI, residual_cfo_hz, N);
    }

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

    // ---- 9b. LLR clamp ±8 ----
    // Literature (ResearchGate: Effect of Saturation on BP Decoding of LDPC)
    // shows LLR clamp of ±6 to ±10 is optimal. At 2 dB SNR, correct LLR
    // magnitude should be ~4-5. Previous ±20 clamp provided no protection
    // against over-confident LLRs from underestimated noise variance.
    {
        constexpr float LLR_CLAMP = 8.0f;
        int n_clamped = 0;
        float max_abs = 0.0f;
        for (auto& l : all_llrs) {
            float a = std::abs(l);
            if (a > max_abs) max_abs = a;
            if (a > LLR_CLAMP) { n_clamped++; l = std::clamp(l, -LLR_CLAMP, LLR_CLAMP); }
        }
        if (n_clamped > 0)
            IRIS_LOG("[OFDM-RX] LLR clamp: %d/%d clamped (max_abs=%.1f before clamp)",
                     n_clamped, (int)all_llrs.size(), max_abs);
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
            IRIS_LOG("[OFDM-RX] LDPC decode: %d/%d blocks failed",
                     n_failed, (int)block_results.size());
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
