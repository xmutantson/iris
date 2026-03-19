#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"   // get_uniform_tone_map
#include "common/fft.h"
#include "common/logging.h"
#include "native/constellation.h"  // demap_soft, bits_to_modulation (from ofdm_mod.h)
#include "native/frame.h"      // crc32
#include <cmath>
#include <algorithm>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

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

        // Get noise variance for this used carrier
        float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 1e-6f;
        float H_mag2 = std::norm(est.H[i]);

        // Effective noise variance after MMSE equalization
        float sigma_sq = (H_mag2 + nv > 1e-12f) ? (nv / (H_mag2 + nv)) : 1.0f;

        // NUC soft demapping for 16QAM+ when enabled
        const NucTable* nuc = nullptr;
        if (tone_map.use_nuc && bpc >= 4)
            nuc = get_nuc_table(mod, fec_r16);

        // Per-carrier reliability weight (CSI): scale LLRs by channel quality.
        // reliability = |H|² / (|H|² + σ²_n), range [0, 1].
        // Low-reliability carriers (deep fades) contribute less to decoding.
        float reliability = (H_mag2 + nv > 1e-12f) ? (H_mag2 / (H_mag2 + nv)) : 0.0f;
        reliability = std::max(0.05f, reliability);  // floor at 5% to avoid zeroing

        if (nuc) {
            // NUC max-log-MAP soft demapper
            size_t llr_start = llrs.size();
            llrs.resize(llr_start + bpc);
            demap_soft_nuc(eq_carriers[data_idx], sigma_sq, nuc, &llrs[llr_start]);
            // Apply reliability weight
            for (int b = 0; b < bpc; b++)
                llrs[llr_start + b] *= reliability;
        } else {
            // Standard uniform QAM soft demapper (reuse pre-allocated buffers)
            sym_vec[0] = eq_carriers[data_idx];
            carrier_llrs = demap_soft(sym_vec, mod, sigma_sq);
            // Apply reliability weight
            for (auto& l : carrier_llrs)
                l *= reliability;
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

    // ---- 7. Calculate number of data symbols (fixed: 1 LDPC block) ----
    int coded_bits_total;
    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        coded_bits_total = LdpcCodec::codeword_size(active_tone_map.fec_rate);
        result.n_ldpc_blocks = 1;
    } else {
        int k = LdpcCodec::block_size(active_tone_map.fec_rate);
        coded_bits_total = (k > 0) ? k : bps_total;
        result.n_ldpc_blocks = 0;
    }

    int n_data_symbols = (coded_bits_total + bps_total - 1) / bps_total;
    result.n_data_symbols = n_data_symbols;

    IRIS_LOG("[OFDM-RX] expecting %d data symbols (%d coded bits, %d bits/sym, 1 LDPC block)",
             n_data_symbols, coded_bits_total, bps_total);

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

    // Pre-allocate reusable buffers for the hot loop (avoid per-symbol heap allocs)
    std::vector<std::complex<float>> fft_buf(nfft);          // Issue 2+3: shared FFT buffer
    std::vector<std::complex<float>> used_carriers_buf(n_used);
    std::vector<std::complex<float>> data_carriers_buf(n_data);
    std::vector<std::complex<float>> eq_carriers_buf(n_data);

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

            // FFT the block pilot (reuse pre-allocated fft_buf)
            const std::complex<float>* pilot_body = iq_corrected.data() + pos + cp;
            std::copy(pilot_body, pilot_body + nfft, fft_buf.data());
            fft_complex(fft_buf.data(), nfft);

            // Block pilot channel update is DISABLED for FM mode.
            // The FM radio's deviation limiter clips OFDM symbols (high PAPR),
            // but the preamble (ZC, near-constant envelope) survives intact.
            // Updating H[k] from a clipped block pilot corrupts the channel
            // estimate, causing CPE to diverge from ~7° to ±110° at symbol 16.
            // The preamble-based estimate + comb pilot CPE tracking is sufficient
            // for FM channels that are static within a frame.
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

        // Now receive the actual data symbol
        if (pos + sym_len > remaining) {
            IRIS_LOG("[OFDM-RX] insufficient samples for data symbol %d/%d", s, n_data_symbols);
            return result;
        }

        // Skip CP, FFT (reuse pre-allocated fft_buf)
        const std::complex<float>* sym_body = iq_corrected.data() + pos + cp;
        std::copy(sym_body, sym_body + nfft, fft_buf.data());
        fft_complex(fft_buf.data(), nfft);

        // Update channel estimate from comb pilots in this data symbol
        ofdm_interpolate_pilots(channel_est_, fft_buf.data(), config_);

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

        // H2: Common Phase Error (CPE) correction from comb pilots
        {
            float cpe_num = 0.0f;   // weighted phase sum
            float cpe_den = 0.0f;   // weight sum
            for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
                // Pilot position: every pilot_carrier_spacing-th used carrier
                std::complex<float> Y_pilot = used_carriers_buf[i];
                std::complex<float> H = channel_est_.H[i];
                float H_mag2 = std::norm(H);
                if (H_mag2 < 1e-12f) continue;

                // Phase difference: arg(Y_pilot * conj(H))
                float phase_diff = std::arg(Y_pilot * std::conj(H));
                cpe_num += H_mag2 * phase_diff;
                cpe_den += H_mag2;
            }
            if (cpe_den > 0.0f) {
                float cpe = cpe_num / cpe_den;
                // Apply conjugate rotation to all used carriers
                std::complex<float> cpe_rot(std::cos(-cpe), std::sin(-cpe));
                for (int i = 0; i < n_used; i++) {
                    used_carriers_buf[i] *= cpe_rot;
                }
                if (data_sym_count % 20 == 0 || std::abs(cpe) > 0.1f) {
                    IRIS_LOG("[OFDM-RX] CPE sym %d: %.4f rad (%.2f deg)",
                             data_sym_count, cpe, cpe * 180.0f / (float)M_PI);
                }
            }
        }

        // Extract data carriers (skip pilot positions, reuse pre-allocated buffer)
        int n_data_actual = extract_data_carriers(used_carriers_buf.data(), n_used,
                                                  data_carriers_buf);

        // MMSE equalize (reuse pre-allocated buffer)
        equalize_mmse(data_carriers_buf, n_data_actual, channel_est_, eq_carriers_buf);

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

    // Skip tail symbol (1 block-pilot)
    pos += sym_len;

    result.samples_consumed = frame_start + pos;
    result.snr_per_carrier = channel_est_.snr_per_carrier;

    // Log per-carrier reliability stats (CSI weighting summary)
    {
        float min_rel = 1.0f, max_rel = 0.0f, sum_rel = 0.0f;
        int n_rel = 0;
        for (int i = 0; i < config_.n_used_carriers; i++) {
            if (i % config_.pilot_carrier_spacing == 0) continue;
            float nv = (i < (int)channel_est_.noise_var.size()) ? channel_est_.noise_var[i] : 1e-6f;
            float Hm2 = std::norm(channel_est_.H[i]);
            float rel = (Hm2 + nv > 1e-12f) ? (Hm2 / (Hm2 + nv)) : 0.0f;
            rel = std::max(0.05f, rel);
            min_rel = std::min(min_rel, rel);
            max_rel = std::max(max_rel, rel);
            sum_rel += rel;
            n_rel++;
        }
        float mean_rel = (n_rel > 0) ? (sum_rel / n_rel) : 0.0f;
        IRIS_LOG("[OFDM-RX] CSI weights: min=%.3f max=%.3f mean=%.3f, NUC=%s, %d LLRs from %d data syms",
                 min_rel, max_rel, mean_rel,
                 active_tone_map.use_nuc ? "ON" : "off",
                 (int)all_llrs.size(), data_sym_count);
    }

    // ---- 9. Truncate LLRs to exact coded bit count ----
    if ((int)all_llrs.size() > coded_bits_total) {
        all_llrs.resize(coded_bits_total);
    } else if ((int)all_llrs.size() < coded_bits_total) {
        // Pad with zero LLRs (erasures)
        all_llrs.resize(coded_bits_total, 0.0f);
    }

    // ---- 9b. LLR clamp ±20 ----
    // Prevents float overflow in LDPC min-sum decoder. Values beyond ±20
    // provide negligible decoding gain but risk numerical instability.
    {
        int n_clamped = 0;
        float max_abs = 0.0f;
        for (auto& l : all_llrs) {
            float a = std::abs(l);
            if (a > max_abs) max_abs = a;
            if (a > 20.0f) { n_clamped++; l = std::max(-20.0f, std::min(20.0f, l)); }
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

    if (active_tone_map.fec_rate != LdpcRate::NONE) {
        // Use per-block decode: continues all blocks even if some fail (for HARQ)
        auto block_results = LdpcCodec::decode_soft_per_block(
            all_llrs, active_tone_map.fec_rate, LdpcDecoder::MIN_SUM, 50);

        result.block_results = block_results;

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

    // ---- 13. Extract 2-byte length prefix from decoded bits ----
    // Headerless frame format: [len_lo][len_hi][payload...][CRC32]
    // All inside the LDPC codeword.
    if ((int)decoded_bits.size() < 16) {
        IRIS_LOG("[OFDM-RX] decoded bits too short for length prefix: %d", (int)decoded_bits.size());
        return result;
    }

    uint16_t extracted_len = 0;
    for (int i = 0; i < 16; i++)
        extracted_len |= ((uint16_t)(decoded_bits[i] & 1)) << i;

    int k = LdpcCodec::block_size(active_tone_map.fec_rate);
    int max_payload = k / 8 - 4 - 2;
    if ((int)extracted_len > max_payload || extracted_len == 0) {
        IRIS_LOG("[OFDM-RX] extracted payload_len=%d out of range [1,%d]",
                 (int)extracted_len, max_payload);
        return result;
    }

    result.payload_len = extracted_len;

    // ---- 14. Convert bits to bytes and verify CRC-32 ----
    int total_bytes = 2 + (int)extracted_len + 4;  // len_prefix + payload + CRC
    int total_bits_needed = total_bytes * 8;

    if ((int)decoded_bits.size() < total_bits_needed) {
        IRIS_LOG("[OFDM-RX] decoded bits insufficient: %d < %d",
                 (int)decoded_bits.size(), total_bits_needed);
        return result;
    }

    std::vector<uint8_t> decoded_bytes(total_bytes, 0);
    for (int i = 0; i < total_bits_needed; i++)
        decoded_bytes[i / 8] |= (decoded_bits[i] << (i % 8));

    // CRC-32 covers [len_lo][len_hi][payload...] = first (2 + extracted_len) bytes
    int crc_data_len = 2 + (int)extracted_len;
    uint32_t computed_crc = crc32(decoded_bytes.data(), crc_data_len);
    uint32_t received_crc = (uint32_t)decoded_bytes[crc_data_len + 0]
                          | ((uint32_t)decoded_bytes[crc_data_len + 1] << 8)
                          | ((uint32_t)decoded_bytes[crc_data_len + 2] << 16)
                          | ((uint32_t)decoded_bytes[crc_data_len + 3] << 24);

    if (computed_crc != received_crc) {
        IRIS_LOG("[OFDM-RX] CRC-32 fail: computed=0x%08X received=0x%08X (payload_len=%d)",
                 computed_crc, received_crc, extracted_len);
        return result;
    }

    // ---- 15. Success — payload starts at byte 2 (after length prefix) ----
    result.success = true;
    result.payload.assign(decoded_bytes.data() + 2, decoded_bytes.data() + 2 + extracted_len);

    IRIS_LOG("[OFDM-RX] frame decoded OK: %d payload bytes, SNR=%.1f dB, mean|H|=%.2f",
             (int)extracted_len, result.snr_db, result.mean_H_mag);

    return result;
}

} // namespace iris
