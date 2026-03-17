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

// ============================================================================
//  CRC-8 (must match TX in ofdm_mod.cc exactly)
//  Polynomial: x^8+x^5+x^4+1 = 0x8C reflected
// ============================================================================
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

// ============================================================================
//  LFSR descrambler (x^15 + x^14 + 1) — must match TX scramble_bits()
//  XOR is self-inverse, so identical to scrambler. XOR before advancing.
// ============================================================================
static void descramble_bits(std::vector<uint8_t>& bits) {
    uint16_t lfsr = 0x6959;  // fixed seed (same as TX)
    for (size_t i = 0; i < bits.size(); i++) {
        bits[i] ^= (lfsr & 1);
        int fb = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
        lfsr = (lfsr >> 1) | ((uint16_t)fb << 14);
    }
}

// ============================================================================
//  FEC field decode (4-bit header field -> LdpcRate, matches TX fec_to_field)
// ============================================================================
static LdpcRate field_to_fec_rate(uint8_t field) {
    switch (field) {
        case 0:  return LdpcRate::NONE;
        case 1:  return LdpcRate::RATE_1_16;
        case 2:  return LdpcRate::RATE_2_16;
        case 3:  return LdpcRate::RATE_3_16;
        case 4:  return LdpcRate::RATE_4_16;
        case 5:  return LdpcRate::RATE_5_16;
        case 6:  return LdpcRate::RATE_6_16;
        case 7:  return LdpcRate::RATE_1_2;
        case 8:  return LdpcRate::RATE_5_8;
        case 9:  return LdpcRate::RATE_3_4;
        case 10: return LdpcRate::RATE_7_8;
        default: return LdpcRate::RATE_1_2;
    }
}

// ============================================================================
//  NFFT mode decode (matches TX nfft_to_mode)
// ============================================================================
static int mode_to_nfft(int mode) {
    switch (mode) {
        case 0: return 512;
        case 1: return 256;
        case 2: return 1024;
        default: return 512;
    }
}

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
//  decode_header: extract metadata from 3 BPSK header OFDM symbols
//
//  Input: header_freq_symbols points to 3 * n_used_carriers complex values.
//  Each group of n_used_carriers is the frequency-domain used carrier values
//  from one header OFDM symbol (already FFT'd and extracted).
// ----------------------------------------------------------------------------
bool OfdmDemodulator::decode_header(
    const std::complex<float>* header_freq_symbols,
    int n_header_symbols,
    uint8_t& tone_map_id, LdpcRate& fec_rate,
    uint16_t& payload_len, int& nfft_mode, bool& harq_flag)
{
    if (n_header_symbols < 1) return false;

    const int n_used = config_.n_used_carriers;
    const int n_data = config_.n_data_carriers;

    // Build pilot mask: pilot at every pilot_carrier_spacing-th used carrier index
    std::vector<bool> is_pilot(n_used, false);
    for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
        is_pilot[i] = true;
    }

    // Collect BPSK hard-decision bits from all header symbols using MMSE.
    // Header bits are sequential across symbols (sym 0 carries bits 0..n_data-1,
    // sym 1 carries bits n_data..2*n_data-1, etc.).
    // MMSE: conj(H)*Y / (|H|² + σ²_n) — avoids noise amplification on faded
    // carriers that naive ZF (Y/H) would cause on FM channels with roll-off.
    std::vector<uint8_t> header_bits;
    header_bits.reserve(n_header_symbols * n_data);

    for (int h = 0; h < n_header_symbols; h++) {
        const std::complex<float>* sym = header_freq_symbols + h * n_used;
        for (int i = 0; i < n_used; i++) {
            if (is_pilot[i]) continue;  // skip pilot positions

            // MMSE equalize: conj(H)*Y / (|H|² + σ²_n)
            std::complex<float> H = channel_est_.H[i];
            float H_mag2 = std::norm(H);
            float nv = (i < (int)channel_est_.noise_var.size())
                        ? channel_est_.noise_var[i] : 1e-6f;

            float denom = H_mag2 + nv;
            std::complex<float> eq;
            if (H_mag2 < nv) {
                // Deep fade: zero this carrier (unreliable)
                eq = {0.0f, 0.0f};
            } else if (denom > 1e-12f) {
                eq = std::conj(H) * sym[i] / denom;
            } else {
                eq = {0.0f, 0.0f};
            }

            // BPSK hard-decision: real > 0 -> bit 0 (+1), else bit 1 (-1)
            header_bits.push_back(eq.real() > 0.0f ? 0 : 1);
        }
    }

    // We need at least 36 bits: [4 tone_map_id][4 fec_rate][12 payload_len]
    //                            [4 nfft_mode][1 harq][3 reserved][8 CRC-8]
    if ((int)header_bits.size() < 36) {
        IRIS_LOG("[OFDM-RX] header too short: %d bits < 36", (int)header_bits.size());
        return false;
    }

    // Parse fields (MSB first, matching TX encode_ofdm_header)
    int pos = 0;

    // tone_map_id: 4 bits
    tone_map_id = 0;
    for (int i = 3; i >= 0; i--)
        tone_map_id |= (header_bits[pos++] << i);

    // fec_rate: 4 bits
    uint8_t fec_field = 0;
    for (int i = 3; i >= 0; i--)
        fec_field |= (header_bits[pos++] << i);
    fec_rate = field_to_fec_rate(fec_field);

    // payload_len: 15 bits (max 32767 bytes)
    payload_len = 0;
    for (int i = 14; i >= 0; i--)
        payload_len |= ((uint16_t)header_bits[pos++] << i);

    // nfft_mode: 4 bits
    nfft_mode = 0;
    for (int i = 3; i >= 0; i--)
        nfft_mode |= (header_bits[pos++] << i);

    // harq_flag: 1 bit
    harq_flag = header_bits[pos++] != 0;

    // CRC-8: 8 bits
    uint8_t received_crc = 0;
    for (int i = 7; i >= 0; i--)
        received_crc |= (header_bits[pos++] << i);

    // Compute CRC-8 over first 28 bits (packed into 4 bytes, same as TX)
    uint8_t header_bytes[4] = {};
    for (int i = 0; i < 28; i++) {
        header_bytes[i / 8] |= header_bits[i] << (7 - (i % 8));
    }
    uint8_t computed_crc = crc8(header_bytes, 4);

    if (received_crc != computed_crc) {
        // Dump channel diagnostics to help debug OTA header failures
        float h_min = 1e9f, h_max = 0.0f, h_sum = 0.0f;
        for (int i = 0; i < (int)channel_est_.H.size(); i++) {
            float mag = std::abs(channel_est_.H[i]);
            h_min = std::min(h_min, mag);
            h_max = std::max(h_max, mag);
            h_sum += mag;
        }
        float h_mean = channel_est_.H.empty() ? 0.0f : h_sum / channel_est_.H.size();
        // Show first 8 header bits for debugging
        uint8_t hdr_preview = 0;
        for (int i = 0; i < 8 && i < (int)header_bits.size(); i++)
            hdr_preview |= (header_bits[i] << (7 - i));
        IRIS_LOG("[OFDM-RX] header CRC-8 fail: rx=0x%02X computed=0x%02X  "
                 "|H| min=%.3f mean=%.3f max=%.3f  first8=0x%02X",
                 received_crc, computed_crc, h_min, h_mean, h_max, hdr_preview);
        return false;
    }

    IRIS_LOG("[OFDM-RX] header OK: tone_map=%d fec=%d payload=%d nfft_mode=%d harq=%d",
             tone_map_id, fec_field, payload_len, nfft_mode, harq_flag ? 1 : 0);
    return true;
}

// ----------------------------------------------------------------------------
//  extract_data_carriers: from n_used used-carrier values, return data only
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmDemodulator::extract_data_carriers(
    const std::complex<float>* symbol_freq, int n_used)
{
    std::vector<std::complex<float>> data;
    data.reserve(config_.n_data_carriers);

    for (int i = 0; i < n_used; i++) {
        // Pilots are at every pilot_carrier_spacing-th position (0, 4, 8, ...)
        if (i % config_.pilot_carrier_spacing == 0) continue;
        data.push_back(symbol_freq[i]);
    }
    return data;
}

// ----------------------------------------------------------------------------
//  equalize_mmse: MMSE equalization of data carriers
//
//  Data carrier index d maps to used carrier index via the pilot skip pattern.
//  We need to find the correct H[i] and noise_var[i] for each data carrier.
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmDemodulator::equalize_mmse(
    const std::vector<std::complex<float>>& data_carriers,
    const OfdmChannelEst& est)
{
    const int n_used = config_.n_used_carriers;
    std::vector<std::complex<float>> eq;
    eq.reserve(data_carriers.size());

    // Build mapping from data carrier index -> used carrier index
    int d = 0;
    for (int i = 0; i < n_used && d < (int)data_carriers.size(); i++) {
        if (i % config_.pilot_carrier_spacing == 0) continue;  // skip pilot

        std::complex<float> Y = data_carriers[d];
        std::complex<float> H = est.H[i];
        float H_mag2 = std::norm(H);
        float nv = (i < (int)est.noise_var.size()) ? est.noise_var[i] : 1e-6f;

        if (H_mag2 < nv) {
            // Deep fade: zero this carrier
            eq.push_back({0.0f, 0.0f});
        } else {
            // MMSE: X_hat = conj(H) * Y / (|H|^2 + noise_var)
            std::complex<float> X_hat = std::conj(H) * Y / (H_mag2 + nv);
            eq.push_back(X_hat);
        }
        d++;
    }

    return eq;
}

// ----------------------------------------------------------------------------
//  demap_to_llrs: soft-demap equalized data carriers to LLRs
// ----------------------------------------------------------------------------
void OfdmDemodulator::demap_to_llrs(
    const std::vector<std::complex<float>>& eq_carriers,
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

    int data_idx = 0;

    for (int i = 0; i < n_used && data_idx < (int)eq_carriers.size(); i++) {
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
            // Standard uniform QAM soft demapper
            std::vector<std::complex<float>> sym_vec(1, eq_carriers[data_idx]);
            std::vector<float> carrier_llrs = demap_soft(sym_vec, mod, sigma_sq);
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
    const ToneMap* tone_map, const OfdmSyncResult* pre_sync)
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
    // Need at least: 2 training + n_header + 1 data + 1 tail = (4 + n_header) symbols
    int min_frame_samples = (4 + config_.n_header_symbols) * sym_len;
    int remaining = n_samples - frame_start;
    if (remaining < min_frame_samples) {
        IRIS_LOG("[OFDM-RX] insufficient samples after detection: %d < %d (need %d syms)",
                 remaining, min_frame_samples, 4 + config_.n_header_symbols);
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
    for (auto& nv : channel_est_.noise_var)
        nv /= BOOST_SQ;
    pos += sym_len;  // advance past training symbol 2

    // ---- 4b. Fine CFO: differential phase between training symbols ----
    // Training sym 1 has PN (+1/-1) on even bins, training sym 2 has all +1.
    // Differential: Y2[k]*conj(Y1[k]) = |H[k]|² * conj(PN[k]) * exp(j*φ_cfo)
    // Multiply by PN[k] to cancel: Y2*conj(Y1)*PN = |H|² * exp(j*φ_cfo)
    // Regenerate same PN sequence as TX (LFSR seed 0xACE1, even bins only).
    std::complex<float> diff_corr(0, 0);
    {
        uint16_t pn = 0xACE1;
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            if (bin % 2 == 0) {
                float pn_val = (pn & 1) ? 1.0f : -1.0f;
                // channel_est_.H[i] ≈ Y2[bin]/boost, Y1[bin] has PN*H*boost
                // diff = H[i]*conj(Y1[bin]) = (Y2/boost)*conj(Y1)
                // Multiply by pn_val to cancel PN sign
                diff_corr += pn_val * channel_est_.H[i] * std::conj(Y1[bin]);
                int fb = ((pn >> 14) ^ (pn >> 13)) & 1;
                pn = (pn >> 1) | ((uint16_t)fb << 14);
            }
        }
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
        for (auto& nv : channel_est_.noise_var)
            nv /= BOOST_SQ;

        IRIS_LOG("[OFDM-RX] CFO refined: coarse=%.1f Hz + fine=%.2f Hz = %.2f Hz",
                 sync.cfo_hz, fine_cfo, total_cfo);
    }

    result.mean_channel_snr_db = channel_est_.mean_snr_db;

    // ---- 5. Decode BPSK header symbols ----
    const int n_hdr_sym = config_.n_header_symbols;
    if (pos + n_hdr_sym * sym_len > remaining) {
        IRIS_LOG("[OFDM-RX] insufficient samples for header symbols");
        return result;
    }

    // Store header used-carrier values: n_hdr_sym * n_used complex values
    std::vector<std::complex<float>> header_used(n_hdr_sym * n_used);

    for (int h = 0; h < n_hdr_sym; h++) {
        // Skip CP, FFT the symbol body
        const std::complex<float>* sym_body = iq_corrected.data() + pos + cp;
        std::vector<std::complex<float>> fft_buf(nfft);
        std::copy(sym_body, sym_body + nfft, fft_buf.data());
        fft_complex(fft_buf.data(), nfft);

        // Extract values at used carrier bins
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            header_used[h * n_used + i] = fft_buf[bin];
        }

        pos += sym_len;
    }

    // Decode header
    uint8_t hdr_tone_map_id = 0;
    LdpcRate hdr_fec_rate = LdpcRate::RATE_1_2;
    uint16_t hdr_payload_len = 0;
    int hdr_nfft_mode = 0;
    bool hdr_harq_flag = false;

    if (!decode_header(header_used.data(), n_hdr_sym,
                       hdr_tone_map_id, hdr_fec_rate, hdr_payload_len,
                       hdr_nfft_mode, hdr_harq_flag)) {
        IRIS_LOG("[OFDM-RX] header decode failed");
        // Consume past the preamble + header so we don't re-detect the same frame.
        // Skip 2 training + n_header symbols worth of samples.
        result.samples_consumed = frame_start + (2 + config_.n_header_symbols) * sym_len;
        return result;
    }

    result.tone_map_id = hdr_tone_map_id;
    result.fec_rate = hdr_fec_rate;
    result.payload_len = hdr_payload_len;
    result.nfft_mode = hdr_nfft_mode;
    result.harq_flag = hdr_harq_flag;

    // Verify NFFT mode matches
    int expected_nfft = mode_to_nfft(hdr_nfft_mode);
    if (expected_nfft != nfft) {
        IRIS_LOG("[OFDM-RX] NFFT mismatch: header says %d, config has %d",
                 expected_nfft, nfft);
        return result;
    }

    // ---- 6. Determine tone map ----
    ToneMap active_tone_map;
    if (hdr_tone_map_id > 0) {
        // Uniform preset from header
        active_tone_map = get_uniform_tone_map(hdr_tone_map_id, config_);
    } else if (tone_map != nullptr) {
        // Waterfill tone map from negotiation
        active_tone_map = *tone_map;
    } else {
        // Fallback: QPSK r1/2
        IRIS_LOG("[OFDM-RX] tone_map_id=0 but no waterfill map provided, fallback QPSK r1/2");
        active_tone_map = get_uniform_tone_map(2, config_);
    }

    // Override FEC rate from header (header is authoritative)
    active_tone_map.fec_rate = hdr_fec_rate;

    int bps_total = active_tone_map.total_bits_per_symbol;
    if (bps_total <= 0) {
        IRIS_LOG("[OFDM-RX] tone map has 0 bits per symbol");
        return result;
    }

    // ---- 7. Calculate number of data symbols needed ----
    // payload_len bytes + 4 CRC bytes -> bits -> LDPC encode -> coded bits
    int payload_with_crc_bits = ((int)hdr_payload_len + 4) * 8;
    int coded_bits_total;

    if (hdr_fec_rate != LdpcRate::NONE) {
        int k = LdpcCodec::block_size(hdr_fec_rate);     // data bits per LDPC block
        int n_fec = LdpcCodec::codeword_size(hdr_fec_rate); // codeword bits (1600)

        if (k <= 0) {
            IRIS_LOG("[OFDM-RX] invalid LDPC block size for FEC rate");
            return result;
        }

        int n_ldpc_blocks = (payload_with_crc_bits + k - 1) / k;
        coded_bits_total = n_ldpc_blocks * n_fec;
        result.n_ldpc_blocks = n_ldpc_blocks;
    } else {
        coded_bits_total = payload_with_crc_bits;
        result.n_ldpc_blocks = 0;
    }

    // Pad to full OFDM symbols
    int n_data_symbols = (coded_bits_total + bps_total - 1) / bps_total;
    result.n_data_symbols = n_data_symbols;

    IRIS_LOG("[OFDM-RX] expecting %d data symbols (%d coded bits, %d bits/sym, %d LDPC blocks)",
             n_data_symbols, coded_bits_total, bps_total, result.n_ldpc_blocks);

    // ---- 8. Receive data symbols (with block pilots) ----
    std::vector<float> all_llrs;
    all_llrs.reserve(coded_bits_total);

    int data_sym_count = 0;  // count of data symbols received so far

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

            // FFT the block pilot
            const std::complex<float>* pilot_body = iq_corrected.data() + pos + cp;
            std::vector<std::complex<float>> pilot_fft(nfft);
            std::copy(pilot_body, pilot_body + nfft, pilot_fft.data());
            fft_complex(pilot_fft.data(), nfft);

            // Update channel estimate from block pilot
            ofdm_update_channel(channel_est_, pilot_fft.data(), config_);

            pos += sym_len;
            // Do not increment data_sym_count for block pilot
        }

        // Now receive the actual data symbol
        if (pos + sym_len > remaining) {
            IRIS_LOG("[OFDM-RX] insufficient samples for data symbol %d/%d", s, n_data_symbols);
            return result;
        }

        // Skip CP, FFT
        const std::complex<float>* sym_body = iq_corrected.data() + pos + cp;
        std::vector<std::complex<float>> sym_fft(nfft);
        std::copy(sym_body, sym_body + nfft, sym_fft.data());
        fft_complex(sym_fft.data(), nfft);

        // Update channel estimate from comb pilots in this data symbol
        ofdm_interpolate_pilots(channel_est_, sym_fft.data(), config_);

        // Extract values at used carrier bins
        std::vector<std::complex<float>> used_carriers(n_used);
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            used_carriers[i] = sym_fft[bin];
        }

        // Extract data carriers (skip pilot positions)
        std::vector<std::complex<float>> data_carriers =
            extract_data_carriers(used_carriers.data(), n_used);

        // MMSE equalize
        std::vector<std::complex<float>> eq_carriers =
            equalize_mmse(data_carriers, channel_est_);

        // Store equalized constellation for GUI scatter plot
        result.eq_constellation.insert(result.eq_constellation.end(),
                                        eq_carriers.begin(), eq_carriers.end());

        // Soft demap to LLRs
        demap_to_llrs(eq_carriers, active_tone_map, channel_est_, all_llrs);

        pos += sym_len;
        data_sym_count++;
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
    if (hdr_fec_rate != LdpcRate::NONE) {
        int n_fec = LdpcCodec::codeword_size(hdr_fec_rate);

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

    // ---- 11b. Tail bit hardening ----
    // The TX encoder pads the last LDPC block with zero bits. Bias those
    // tail LLR positions toward +20 (strong "0" prior) to help the decoder
    // converge on the correct codeword for short payloads.
    if (hdr_fec_rate != LdpcRate::NONE) {
        int k = LdpcCodec::block_size(hdr_fec_rate);
        if (k > 0 && result.n_ldpc_blocks > 0) {
            int total_data_bits = result.n_ldpc_blocks * k;
            int payload_bits = payload_with_crc_bits;
            if (total_data_bits > payload_bits) {
                // Tail bits are in the last LDPC block, positions payload_bits..total_data_bits-1
                // Map data bit index -> codeword LLR index (systematic code: first k bits)
                int last_blk_start_data = (result.n_ldpc_blocks - 1) * k;
                int n_fec = LdpcCodec::codeword_size(hdr_fec_rate);
                int last_blk_start_llr = (result.n_ldpc_blocks - 1) * n_fec;
                int tail_start_in_blk = payload_bits - last_blk_start_data;
                if (tail_start_in_blk < 0) tail_start_in_blk = 0;
                int n_tail = 0;
                for (int i = tail_start_in_blk; i < k; i++) {
                    int llr_idx = last_blk_start_llr + i;
                    if (llr_idx < (int)all_llrs.size()) {
                        all_llrs[llr_idx] = 20.0f;  // strong "0" prior
                        n_tail++;
                    }
                }
                IRIS_LOG("[OFDM-RX] tail hardening: %d bits biased to +20 in last LDPC block",
                         n_tail);
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

    if (hdr_fec_rate != LdpcRate::NONE) {
        // Use per-block decode: continues all blocks even if some fail (for HARQ)
        auto block_results = LdpcCodec::decode_soft_per_block(
            all_llrs, hdr_fec_rate, LdpcDecoder::MIN_SUM, 50);

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

    // ---- 13. Convert bits to bytes ----
    int total_bytes_with_crc = (int)hdr_payload_len + 4;
    int total_bits_needed = total_bytes_with_crc * 8;

    if ((int)decoded_bits.size() < total_bits_needed) {
        IRIS_LOG("[OFDM-RX] decoded bits too short: %d < %d",
                 (int)decoded_bits.size(), total_bits_needed);
        return result;
    }

    std::vector<uint8_t> decoded_bytes(total_bytes_with_crc, 0);
    for (int i = 0; i < total_bits_needed; i++) {
        // LSB-first packing (matching TX: bit j of byte i = (byte >> j) & 1)
        decoded_bytes[i / 8] |= (decoded_bits[i] << (i % 8));
    }

    // ---- 14. Verify CRC-32 ----
    const uint8_t* payload_data = decoded_bytes.data();
    int payload_bytes = (int)hdr_payload_len;

    uint32_t computed_crc = crc32(payload_data, payload_bytes);
    uint32_t received_crc = (uint32_t)decoded_bytes[payload_bytes + 0]
                          | ((uint32_t)decoded_bytes[payload_bytes + 1] << 8)
                          | ((uint32_t)decoded_bytes[payload_bytes + 2] << 16)
                          | ((uint32_t)decoded_bytes[payload_bytes + 3] << 24);

    if (computed_crc != received_crc) {
        IRIS_LOG("[OFDM-RX] CRC-32 fail: computed=0x%08X received=0x%08X",
                 computed_crc, received_crc);
        return result;
    }

    // ---- 15. Success ----
    result.success = true;
    result.payload.assign(payload_data, payload_data + payload_bytes);

    IRIS_LOG("[OFDM-RX] frame decoded OK: %d payload bytes, SNR=%.1f dB, channel SNR=%.1f dB",
             payload_bytes, result.snr_db, result.mean_channel_snr_db);

    return result;
}

} // namespace iris
