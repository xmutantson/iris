#include "native/frame.h"
#include "common/logging.h"
#include <algorithm>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// CRC-32 (ISO 3309 / ITU-T V.42)
uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

// CRC-8 for header protection
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

// Scramble/descramble bits using LFSR PRBS (x^15 + x^14 + 1)
// Self-inverse: applying twice returns original bits
static void scramble_bits(std::vector<uint8_t>& bits) {
    uint16_t lfsr = 0x6959;  // fixed seed (non-zero)
    for (size_t i = 0; i < bits.size(); i++) {
        int fb = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
        bits[i] ^= (lfsr & 1);
        lfsr = (lfsr >> 1) | ((uint16_t)fb << 14);
    }
}

// Soft descramble: flip LLR sign where LFSR bit is 1
// XOR with 1 flips a hard bit; negating the LLR flips soft decision
static void scramble_soft(std::vector<float>& llrs) {
    uint16_t lfsr = 0x6959;
    for (size_t i = 0; i < llrs.size(); i++) {
        int fb = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
        if (lfsr & 1)
            llrs[i] = -llrs[i];
        lfsr = (lfsr >> 1) | ((uint16_t)fb << 14);
    }
}

// Generate 63-symbol m-sequence using LFSR (x^6 + x + 1)
std::vector<std::complex<float>> generate_preamble() {
    std::vector<std::complex<float>> preamble;
    preamble.reserve(IRIS_PREAMBLE_LEN);

    uint8_t lfsr = 0x3F; // all ones initial state
    for (int i = 0; i < IRIS_PREAMBLE_LEN; i++) {
        int bit = lfsr & 1;
        preamble.push_back({bit ? 1.0f : -1.0f, 0.0f});

        int feedback = (lfsr & 1) ^ ((lfsr >> 5) & 1);
        lfsr = (lfsr >> 1) | (feedback << 5);
    }
    return preamble;
}

// 16-symbol sync word (Barker-13 + 3 padding)
std::vector<std::complex<float>> generate_sync_word() {
    static const int barker16[] = {
        1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1
    };
    std::vector<std::complex<float>> sync;
    sync.reserve(IRIS_SYNC_LEN);
    for (int i = 0; i < IRIS_SYNC_LEN; i++) {
        sync.push_back({(float)barker16[i], 0.0f});
    }
    return sync;
}

// Map LdpcRate to 4-bit field for header
static uint8_t fec_to_field(LdpcRate fec) {
    switch (fec) {
        case LdpcRate::NONE:      return 0;
        case LdpcRate::RATE_1_16: return 1;
        case LdpcRate::RATE_2_16: return 2;
        case LdpcRate::RATE_3_16: return 3;
        case LdpcRate::RATE_4_16: return 4;
        case LdpcRate::RATE_5_16: return 5;
        case LdpcRate::RATE_6_16: return 6;
        case LdpcRate::RATE_1_2:  return 7;
        case LdpcRate::RATE_5_8:  return 8;
        case LdpcRate::RATE_3_4:  return 9;
        case LdpcRate::RATE_7_8:  return 10;
    }
    return 0;
}

static LdpcRate field_to_fec(uint8_t field) {
    switch (field & 0x0F) {
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
    }
    return LdpcRate::NONE;
}

std::vector<uint8_t> encode_header(Modulation mod, uint16_t payload_len, LdpcRate fec) {
    // 32 bits: [4 mod][12 payload_len][4 fec][4 reserved][8 crc8]
    uint8_t header_bytes[4];
    header_bytes[0] = ((uint8_t)mod << 4) | ((payload_len >> 8) & 0x0F);
    header_bytes[1] = payload_len & 0xFF;
    header_bytes[2] = (fec_to_field(fec) << 4);
    header_bytes[3] = crc8(header_bytes, 3);

    std::vector<uint8_t> bits(32);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            bits[i * 8 + j] = (header_bytes[i] >> (7 - j)) & 1;
        }
    }
    return bits;
}

bool decode_header(const std::vector<uint8_t>& bits, Modulation& mod, uint16_t& payload_len, LdpcRate& fec) {
    if (bits.size() < 32) return false;

    uint8_t header_bytes[4] = {};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            header_bytes[i] |= (bits[i * 8 + j] & 1) << (7 - j);
        }
    }

    if (crc8(header_bytes, 3) != header_bytes[3])
        return false;

    uint8_t mod_val = (header_bytes[0] >> 4) & 0x0F;
    if (mod_val > (uint8_t)Modulation::QAM256)
        return false;

    mod = (Modulation)mod_val;
    payload_len = ((header_bytes[0] & 0x0F) << 8) | header_bytes[1];
    fec = field_to_fec(header_bytes[2] >> 4);
    return true;
}

std::vector<float> build_native_frame(const uint8_t* payload, size_t len,
                                       const PhyConfig& config,
                                       LdpcRate fec) {
    NativeModulator mod(config);

    auto preamble = generate_preamble();
    auto sync = generate_sync_word();
    auto header_bits = encode_header(config.modulation, (uint16_t)len, fec);

    // Payload + CRC32
    std::vector<uint8_t> payload_with_crc(len + 4);
    for (size_t i = 0; i < len; i++)
        payload_with_crc[i] = payload[i];
    uint32_t crc = crc32(payload, len);
    payload_with_crc[len + 0] = (crc >>  0) & 0xFF;
    payload_with_crc[len + 1] = (crc >>  8) & 0xFF;
    payload_with_crc[len + 2] = (crc >> 16) & 0xFF;
    payload_with_crc[len + 3] = (crc >> 24) & 0xFF;

    // Convert payload bytes to bits
    std::vector<uint8_t> payload_bits;
    for (size_t i = 0; i < payload_with_crc.size(); i++) {
        for (int j = 0; j < 8; j++) {
            payload_bits.push_back((payload_with_crc[i] >> j) & 1);
        }
    }

    // Apply LDPC FEC encoding
    if (fec != LdpcRate::NONE) {
        payload_bits = LdpcCodec::encode(payload_bits, fec);
        // Interleave each LDPC block to spread burst errors (e.g. signal fade
        // at frame tail) across the codeword.  LDPC corrects random errors well
        // but struggles with contiguous bursts in the same parity region.
        int n_fec = LdpcCodec::codeword_size(fec);
        constexpr int INTERLEAVE_STRIDE = 41;  // coprime to 1600
        for (size_t blk = 0; blk + n_fec <= payload_bits.size(); blk += n_fec) {
            std::vector<uint8_t> tmp(n_fec);
            for (int i = 0; i < n_fec; i++)
                tmp[(i * INTERLEAVE_STRIDE) % n_fec] = payload_bits[blk + i];
            std::copy(tmp.begin(), tmp.end(), payload_bits.begin() + blk);
        }
    }

    // Pad to multiple of bits_per_symbol
    int bps = bits_per_symbol(config.modulation);
    while (payload_bits.size() % bps != 0)
        payload_bits.push_back(0);

    // Scramble encoded bits to ensure uniform QAM symbol distribution
    // (LDPC zero-padding maps to high-power corner symbols without scrambling)
    scramble_bits(payload_bits);

    auto payload_symbols = map_bits(payload_bits, config.modulation);

    // Insert mid-frame pilots for phase tracking.
    // QAM16+ always needs pilots.  BPSK/QPSK need them on long frames (>256 symbols)
    // where the decision-directed PLL drifts without known reference points.
    // OTA testing showed 2400-symbol QPSK frames losing phase lock at the tail.
    int ps = pilot_spacing_for((int)config.modulation);
    bool use_pilots = (ps > 0) &&
        (config.modulation >= Modulation::QAM16 || (int)payload_symbols.size() > 256);
    if (use_pilots) {
        std::vector<std::complex<float>> with_pilots;
        with_pilots.reserve(payload_symbols.size() + payload_symbols.size() / ps + 1);
        for (size_t i = 0; i < payload_symbols.size(); i++) {
            if (i > 0 && (i % ps) == 0)
                with_pilots.push_back({1.0f, 0.0f});  // Known +1 pilot
            with_pilots.push_back(payload_symbols[i]);
        }
        payload_symbols = std::move(with_pilots);
    }

    // Concatenate all symbols
    std::vector<std::complex<float>> all_symbols;
    all_symbols.reserve(preamble.size() + sync.size() + 32 + payload_symbols.size());

    for (auto& s : preamble) all_symbols.push_back(s);
    for (auto& s : sync) all_symbols.push_back(s);

    for (int i = 0; i < 32; i++) {
        all_symbols.push_back({header_bits[i] ? -1.0f : 1.0f, 0.0f});
    }

    for (auto& s : payload_symbols) all_symbols.push_back(s);

    return mod.modulate_symbols(all_symbols);
}

// Track best correlation for diagnostics
static float g_last_best_corr = 0;
float detect_best_corr() { return g_last_best_corr; }

static bool g_decode_overflow = false;
bool decode_was_overflow() { return g_decode_overflow; }

static size_t g_decode_consumed_iq = 0;
size_t decode_consumed_iq() { return g_decode_consumed_iq; }

static float g_decode_snr = 0.0f;
float decode_snr_db() { return g_decode_snr; }

int detect_frame_start(const float* iq_samples, size_t count, int sps) {
    auto preamble = generate_preamble();
    auto sync = generate_sync_word();

    std::vector<std::complex<float>> ref;
    for (auto& s : preamble) ref.push_back(s);
    for (auto& s : sync) ref.push_back(s);

    size_t ref_len = ref.size();
    size_t n_samples = count / 2;

    // Require enough buffer for preamble+sync+header decode.
    // This ensures that any detected preamble has enough data to at least
    // decode the header and determine exact frame size, avoiding speculative
    // overflow/retry cycles. Inspired by Direwolf's approach: never attempt
    // decode without sufficient data.
    size_t min_frame_iq = (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN + IRIS_HEADER_LEN) * sps
                         + 2 * RRC_SPAN * sps + 2 * sps;  // header + RRC tail + fine timing margin
    if (n_samples < min_frame_iq + (size_t)sps) { g_last_best_corr = -1; return -1; }

    float best_corr = 0;
    int best_offset = -1;

    // Only search where there's enough data for header decode.
    // Preambles found near the buffer end will be re-detected once
    // more audio accumulates (Direwolf-style streaming approach).
    size_t search_end = n_samples - min_frame_iq;

    for (size_t offset = 0; offset < search_end; offset += sps / 2) {
        float corr_re = 0, corr_im = 0;
        float energy = 0;

        for (size_t k = 0; k < ref_len; k++) {
            size_t idx = offset + k * sps;
            if (idx >= n_samples) break;
            float re = iq_samples[2 * idx];
            float im = iq_samples[2 * idx + 1];

            corr_re += re * ref[k].real() - im * ref[k].imag();
            corr_im += re * ref[k].imag() + im * ref[k].real();
            energy += re * re + im * im;
        }

        float mag = std::sqrt(corr_re * corr_re + corr_im * corr_im);
        // Properly normalize to [0,1]: divide by sqrt(signal_energy * ref_energy)
        // Reference symbols are unit magnitude, so ref_energy = ref_len
        float norm = (energy > 0) ? mag / std::sqrt(energy * (float)ref_len) : 0;

        // Energy gate: reject detections on very low-energy windows.
        // When energy ~ 0, even tiny random correlations normalize to ~1.0.
        // Min energy = 0.01 per symbol (well below any real signal).
        float energy_per_sym = energy / (float)ref_len;
        if (energy_per_sym < 0.01f) norm = 0;

        if (norm > best_corr) {
            best_corr = norm;
            if (norm > 0.5f)
                best_offset = (int)offset;
        }
    }

    g_last_best_corr = best_corr;
    return best_offset;
}

bool decode_native_frame(const float* iq_samples, size_t count,
                          int frame_start, const PhyConfig& default_config,
                          std::vector<uint8_t>& payload) {
    g_decode_overflow = false;
    g_decode_consumed_iq = 0;
    int sps = default_config.samples_per_symbol;
    size_t n_samples = count / 2;

    auto rrc_taps = rrc_filter(default_config.rrc_alpha, RRC_SPAN, sps);

    std::vector<float> i_in(n_samples), q_in(n_samples);
    for (size_t i = 0; i < n_samples; i++) {
        i_in[i] = iq_samples[2 * i];
        q_in[i] = iq_samples[2 * i + 1];
    }

    // Impulse noise blanking: clip passband amplitude to 3.5x RMS.
    // Prevents single loud clicks from corrupting multiple symbols.
    {
        float energy = 0;
        for (size_t i = 0; i < n_samples; i++)
            energy += i_in[i] * i_in[i] + q_in[i] * q_in[i];
        float rms = std::sqrt(energy / (float)n_samples);
        float clip_thresh = 3.5f * rms;
        if (clip_thresh > 1e-6f) {
            for (size_t i = 0; i < n_samples; i++) {
                float mag = std::sqrt(i_in[i] * i_in[i] + q_in[i] * q_in[i]);
                if (mag > clip_thresh) {
                    float scale = clip_thresh / mag;
                    i_in[i] *= scale;
                    q_in[i] *= scale;
                }
            }
        }
    }

    auto i_filt = fir_filter(i_in, rrc_taps);
    auto q_filt = fir_filter(q_in, rrc_taps);
    size_t filt_len = std::min(i_filt.size(), q_filt.size());

    int rx_delay = RRC_SPAN * sps;

    // Fine timing search: try offsets around coarse frame_start,
    // pick the timing that maximizes preamble+sync correlation energy
    auto sync_ref = generate_sync_word();
    auto preamble_ref = generate_preamble();

    // Build combined preamble+sync reference for timing search
    std::vector<std::complex<float>> timing_ref;
    for (auto& s : preamble_ref) timing_ref.push_back(s);
    for (auto& s : sync_ref) timing_ref.push_back(s);
    int timing_ref_len = (int)timing_ref.size();

    int best_timing = frame_start;
    float best_mag = 0;
    int search_range = sps;

    // Store correlation magnitudes for parabolic interpolation
    std::vector<float> corr_mags(2 * search_range + 1, 0);

    for (int dt = -search_range; dt <= search_range; dt++) {
        int trial_start = frame_start + dt;
        if (trial_start < 0) continue;
        int trial_rx = trial_start + rx_delay;

        size_t last_idx = trial_rx + (timing_ref_len - 1) * sps;
        if (last_idx >= filt_len) continue;

        float cr = 0, ci = 0;
        for (int i = 0; i < timing_ref_len; i++) {
            size_t idx = trial_rx + i * sps;
            cr += i_filt[idx] * timing_ref[i].real();
            ci += q_filt[idx] * timing_ref[i].real();
        }
        float mag = cr * cr + ci * ci;
        corr_mags[dt + search_range] = mag;
        if (mag > best_mag) {
            best_mag = mag;
            best_timing = trial_start;
        }
    }

    // Sub-sample timing refinement: parabolic interpolation around the peak
    float timing_frac = 0.0f;  // fractional sample offset from best_timing
    int peak_idx = best_timing - frame_start + search_range;
    if (peak_idx > 0 && peak_idx < (int)corr_mags.size() - 1) {
        float yl = corr_mags[peak_idx - 1];
        float yc = corr_mags[peak_idx];
        float yr = corr_mags[peak_idx + 1];
        float denom = 2.0f * (2.0f * yc - yl - yr);
        if (std::abs(denom) > 1e-10f)
            timing_frac = (yl - yr) / denom;
        timing_frac = std::max(-0.5f, std::min(0.5f, timing_frac));
    }

    int rx_frame_start = best_timing + rx_delay;

    // Estimate carrier phase + frequency offset from preamble (2 halves)
    // Phase at preamble first half midpoint and second half midpoint
    int preamble_start = rx_frame_start;
    int half1_start = 0;
    int half1_end = IRIS_PREAMBLE_LEN / 2;
    int half2_start = half1_end;
    int half2_end = IRIS_PREAMBLE_LEN;

    // Linear interpolation helper for sub-sample timing
    auto interp = [](const std::vector<float>& buf, float pos) -> float {
        int idx = (int)pos;
        float frac = pos - idx;
        if (idx < 0 || idx + 1 >= (int)buf.size()) return (idx >= 0 && idx < (int)buf.size()) ? buf[idx] : 0.0f;
        return buf[idx] + frac * (buf[idx + 1] - buf[idx]);
    };

    float cr1 = 0, ci1 = 0, cr2 = 0, ci2 = 0;
    // Need: preamble end + header + minimum 1 LDPC block for any modulation
    size_t need_for_header = (size_t)(preamble_start +
        (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN + IRIS_HEADER_LEN) * sps + sps + 2);

    for (int i = half1_start; i < half1_end; i++) {
        float pos = preamble_start + i * sps + timing_frac;
        if (pos + 1 >= filt_len) { g_decode_overflow = true; g_decode_consumed_iq = need_for_header; return false; }
        float iv = interp(i_filt, pos);
        float qv = interp(q_filt, pos);
        cr1 += iv * preamble_ref[i].real();
        ci1 += qv * preamble_ref[i].real();
    }
    for (int i = half2_start; i < half2_end; i++) {
        float pos = preamble_start + i * sps + timing_frac;
        if (pos + 1 >= filt_len) { g_decode_overflow = true; g_decode_consumed_iq = need_for_header; return false; }
        float iv = interp(i_filt, pos);
        float qv = interp(q_filt, pos);
        cr2 += iv * preamble_ref[i].real();
        ci2 += qv * preamble_ref[i].real();
    }

    float phase1 = std::atan2(ci1, cr1);
    float phase2 = std::atan2(ci2, cr2);

    // Unwrap phase difference
    float dphase = phase2 - phase1;
    while (dphase > (float)M_PI) dphase -= 2.0f * (float)M_PI;
    while (dphase < -(float)M_PI) dphase += 2.0f * (float)M_PI;

    // Phase rate: dphase per symbol, measured over half-preamble spacing
    int half_spacing = (half2_start + half2_end) / 2 - (half1_start + half1_end) / 2;
    float phase_per_symbol = (half_spacing > 0) ? dphase / half_spacing : 0;

    // Reference point: phase at preamble midpoint
    float phase_ref = (phase1 + phase2) / 2;
    int ref_symbol = IRIS_PREAMBLE_LEN / 2;  // symbol index of reference point

    // Channel gain: |correlation| / num_symbols (ref symbols are unit magnitude)
    float mag1 = std::sqrt(cr1 * cr1 + ci1 * ci1);
    float mag2 = std::sqrt(cr2 * cr2 + ci2 * ci2);
    float channel_gain = (mag1 / (half1_end - half1_start) + mag2 / (half2_end - half2_start)) / 2;
    if (channel_gain < 1e-6f) channel_gain = 1.0f;

    float freq_offset_hz = phase_per_symbol * default_config.baud_rate / (2.0f * (float)M_PI);

    IRIS_LOG("[DECODE] phase=%.1f deg freq=%.1f Hz gain=%.3f timing_adj=%d frac=%.3f",
             phase_ref * 180.0f / (float)M_PI, freq_offset_hz, channel_gain,
             best_timing - frame_start, timing_frac);

    // Compute SNR from phase-corrected, properly-timed preamble symbols.
    // This matches what the decoder actually sees, unlike the modem's external
    // estimator which lacks fine timing and phase/frequency correction.
    {
        float signal_power = 0, noise_power = 0;
        for (int i = 0; i < IRIS_PREAMBLE_LEN; i++) {
            float pos = preamble_start + i * sps + timing_frac;
            if (pos + 1 >= filt_len) break;
            float iv = interp(i_filt, pos);
            float qv = interp(q_filt, pos);
            // Phase correction: rotate by -(phase_ref + phase_per_symbol * (i - ref_symbol))
            float sym_phase = phase_ref + phase_per_symbol * (i - ref_symbol);
            float cc = std::cos(-sym_phase) / channel_gain;
            float ss = std::sin(-sym_phase) / channel_gain;
            float re = iv * cc - qv * ss;
            float im = iv * ss + qv * cc;
            // Known preamble symbol is real-valued (BPSK m-sequence, ±1)
            float expected_re = preamble_ref[i].real();  // ±1
            float err_re = re - expected_re;
            float err_im = im;  // expected Q = 0
            signal_power += expected_re * expected_re;  // = 1.0 per symbol
            noise_power += err_re * err_re + err_im * err_im;
        }
        if (noise_power < 1e-12f)
            g_decode_snr = 60.0f;
        else
            g_decode_snr = 10.0f * std::log10(signal_power / noise_power);
    }

    int header_start = rx_frame_start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN) * sps;

    // Decode header (32 BPSK symbols) with recursive 2nd-order PLL
    // Recursive PLL: running_phase advances by pll_freq each symbol, avoiding
    // the linear extrapolation model (pll_phase + pll_freq * offset) which
    // diverges over long frames (1600+ symbols) due to positive feedback
    // between accumulated pll_freq corrections and growing symbol offsets.
    float pll_freq = phase_per_symbol;  // Initialize from preamble estimate
    float pll_alpha = 0.10f;  // Proportional gain (phase)
    float pll_beta = 0.005f;  // Integral gain (frequency)

    // Advance running_phase from preamble midpoint to header start
    int header_sym_offset = IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN;
    float running_phase = phase_ref + pll_freq * (header_sym_offset - ref_symbol);

    std::vector<uint8_t> header_bits;
    for (int i = 0; i < IRIS_HEADER_LEN; i++) {
        float pos = header_start + i * sps + timing_frac;
        if (pos + 1 >= filt_len) {
            g_decode_overflow = true;
            g_decode_consumed_iq = need_for_header;
            return false;
        }
        float iv = interp(i_filt, pos);
        float qv = interp(q_filt, pos);
        float phase_corr = -running_phase;
        float cc = std::cos(phase_corr) / channel_gain;
        float ss = std::sin(phase_corr) / channel_gain;
        float re = iv * cc - qv * ss;
        float im = iv * ss + qv * cc;
        header_bits.push_back(re < 0 ? 1 : 0);

        // Recursive 2nd-order PLL update: BPSK decision-directed
        float decided_re = re > 0 ? 1.0f : -1.0f;
        float phase_err = std::atan2(im * decided_re, re * decided_re);
        running_phase += pll_alpha * phase_err;
        pll_freq += pll_beta * phase_err;
        running_phase += pll_freq;  // Advance phase by current frequency estimate
    }
    Modulation mod;
    uint16_t payload_len;
    LdpcRate fec;
    if (!decode_header(header_bits, mod, payload_len, fec)) {
        IRIS_LOG("[DECODE] header decode failed (frame_start=%d, header_start=%d)", frame_start, header_start);
        return false;
    }

    // Header decoded: mod, payload_len, fec known. Continue to payload decode.
    IRIS_LOG("[DECODE] RX header: mod=%d fec=%d payload=%d bytes",
             (int)mod, (int)fec, payload_len);

    if (payload_len > NATIVE_MAX_PAYLOAD) {
        IRIS_LOG("[DECODE] payload_len %d too large (max %d)", payload_len, NATIVE_MAX_PAYLOAD);
        return false;
    }

    int payload_start = header_start + IRIS_HEADER_LEN * sps;
    int bps = bits_per_symbol(mod);
    int raw_data_bits = (payload_len + 4) * 8;  // payload + CRC32

    // Compute encoded bit count after FEC
    int encoded_bits;
    if (fec != LdpcRate::NONE) {
        int k = LdpcCodec::block_size(fec);
        int n = LdpcCodec::codeword_size(fec);
        int num_blocks = (raw_data_bits + k - 1) / k;
        encoded_bits = num_blocks * n;
    } else {
        encoded_bits = raw_data_bits;
    }

    int padded_bits = encoded_bits;
    if (padded_bits % bps != 0)
        padded_bits += bps - (padded_bits % bps);
    int data_symbols = padded_bits / bps;

    // Account for mid-frame pilots (must match TX insertion logic)
    int ps = pilot_spacing_for((int)mod);
    bool use_pilots = (ps > 0) &&
        (mod >= Modulation::QAM16 || data_symbols > 256);
    int n_pilots = use_pilots ? ((data_symbols > 0) ? ((data_symbols - 1) / ps) : 0) : 0;
    int payload_symbols = data_symbols + n_pilots;

    // Pre-check: verify buffer has enough data for ALL payload symbols
    // BEFORE extracting any. This avoids mid-decode overflow and ensures
    // we only process frames when the complete signal has arrived.
    {
        float last_pos = payload_start + (payload_symbols - 1) * sps + timing_frac;
        if (last_pos + 2 >= filt_len) {
            g_decode_overflow = true;
            // Exact IQ requirement: payload end + RRC filter tail (no artificial margin)
            g_decode_consumed_iq = (size_t)(payload_start + payload_symbols * sps +
                                            RRC_SPAN * sps);
            IRIS_LOG("[DECODE] need more data: have %zu IQ, need ~%zu (payload_syms=%d, mod=%d, fec=%d)",
                     n_samples, g_decode_consumed_iq, payload_symbols, (int)mod, (int)fec);
            return false;
        }
    }

    // Diagnostic: scan raw IQ energy every 2000 samples to find signal extent
    {
        char ebuf[1024]; int ep = 0;
        ep += snprintf(ebuf+ep, sizeof(ebuf)-ep, "[DECODE] energy scan (n=%zu): ", n_samples);
        for (int pos = 0; pos < (int)n_samples; pos += 2000) {
            float e = 0;
            int cnt = std::min(100, (int)n_samples - pos);
            for (int j = pos; j < pos + cnt; j++)
                e += i_in[j] * i_in[j] + q_in[j] * q_in[j];
            e /= cnt;
            ep += snprintf(ebuf+ep, sizeof(ebuf)-ep, "%d:%.4f ", pos, e);
            if (ep > 900) break;
        }
        IRIS_LOG("%s", ebuf);
    }

    // Adaptive PLL bandwidth: tighten for high-order QAM (more noise-sensitive),
    // widen for low-order (need to track faster without constellation ambiguity).
    float payload_pll_alpha, payload_pll_beta;
    if (mod == Modulation::BPSK || mod == Modulation::QPSK) {
        payload_pll_alpha = 0.15f;  // Wide: FM discriminator phase noise extends to
                                     // 50-100 Hz; previous 0.05 gave ~16 Hz BW at 2086
                                     // baud, too narrow to track.  0.15 → ~50 Hz BW.
                                     // Phase noise penalty: 5° vs 3° at 10 dB SNR,
                                     // well within QPSK ±45° decision boundary.
        payload_pll_beta = 0.001f;  // Increased from 0.0001: allows tracking slow
                                     // frequency drift from FM discriminator.  Random-walk
                                     // risk acceptable — QPSK tolerates ±45° phase error.
    } else if (mod == Modulation::QAM16) {
        payload_pll_alpha = 0.06f;
        payload_pll_beta = 0.0005f; // Reduced: same random-walk concern for long frames
    } else {
        payload_pll_alpha = 0.03f;  // Tight: 64/128/256-QAM need stable phase
        payload_pll_beta = 0.0002f; // Very low: short high-QAM frames, minimal drift
    }

    std::vector<std::complex<float>> symbols;
    std::vector<float> sym_reliability;  // per-symbol magnitude for LLR weighting
    // Diagnostic: track raw IQ phase at sampled points
    char raw_diag[1024]; int rdp = 0;
    rdp += snprintf(raw_diag+rdp, sizeof(raw_diag)-rdp, "[DECODE] raw_phase: ");
    int data_sym_count = 0;  // counts data symbols (excludes pilots)
    int since_pilot = 0;     // symbols since last pilot (resets on pilot extraction)
    for (int k2 = 0; k2 < payload_symbols; k2++) {
        float pos = payload_start + k2 * sps + timing_frac;
        float iv = interp(i_filt, pos);
        float qv = interp(q_filt, pos);

        // Track pre-equalization magnitude for reliability weighting
        float raw_mag = std::sqrt(iv * iv + qv * qv);

        // Log raw IQ phase at sampled positions
        if (k2 < 10 || (k2 % 500 == 0) || k2 == payload_symbols - 1) {
            float raw_phase_deg = std::atan2(qv, iv) * 180.0f / (float)M_PI;
            if (rdp < 900)
                rdp += snprintf(raw_diag+rdp, sizeof(raw_diag)-rdp,
                    "%d:(%.3f,%.3f,%.1f°) ", k2, iv, qv, raw_phase_deg);
        }

        // Recursive 2nd-order PLL: running_phase tracks cumulative phase
        float phase_corr = -running_phase;
        float cc = std::cos(phase_corr) / channel_gain;
        float ss = std::sin(phase_corr) / channel_gain;
        float re = iv * cc - qv * ss;
        float im = iv * ss + qv * cc;

        // Check if this position is a pilot symbol.
        // TX inserts a pilot BEFORE every ps-th data symbol:
        //   for (i=0..N) { if (i>0 && i%ps==0) insert_pilot; push data[i]; }
        // RX mirrors: after ps data symbols, expect one pilot.
        bool is_pilot = false;
        if (use_pilots && since_pilot == ps) {
            is_pilot = true;
        }

        if (is_pilot) {
            // Pilot symbol: known +1 BPSK. Use for PLL update with known reference.
            float phase_err = std::atan2(im, re);
            running_phase += payload_pll_alpha * 2.0f * phase_err;
            pll_freq += payload_pll_beta * 2.0f * phase_err;
            // Update channel gain from pilot magnitude.
            // Smooth tracking to avoid noise-induced gain oscillations.
            // With pilots every 16 symbols at QAM16, we get frequent updates
            // so we can afford to smooth more aggressively.
            float pilot_mag = std::sqrt(re * re + im * im);
            channel_gain = 0.85f * channel_gain + 0.15f * pilot_mag;
            since_pilot = 0;  // reset counter after pilot consumed
        } else {
            symbols.push_back({re, im});
            sym_reliability.push_back(raw_mag);
            data_sym_count++;
            since_pilot++;

            // Decision-directed PLL for BPSK/QPSK (low symbol error rate).
            // For QAM16+, skip DD updates — at ~8 dB SNR, the ~10% SER causes
            // positive-feedback cycle-slips between pilots. With pilots every
            // 16 symbols, pilot-only tracking is sufficient.
            if (mod < Modulation::QAM16) {
                uint8_t dec_bits[8];
                demap_symbol({re, im}, dec_bits, mod);
                auto decided = map_symbol(dec_bits, mod);
                float phase_err = std::atan2(
                    im * decided.real() - re * decided.imag(),
                    re * decided.real() + im * decided.imag());
                running_phase += payload_pll_alpha * phase_err;
                pll_freq += payload_pll_beta * phase_err;
            }
            // QAM16+: phase tracks only on pilots (every 16 symbols).
            // This eliminates DD cycle-slip at the cost of slower tracking,
            // which is acceptable with the denser pilot grid.
        }
        running_phase += pll_freq;  // Advance phase by current frequency estimate
    }
    IRIS_LOG("%s", raw_diag);

    // Dump first and last equalized symbols for diagnostics
    {
        char buf[512]; int bp = 0;
        bp += snprintf(buf+bp, sizeof(buf)-bp, "sym[0..4]: ");
        for (int di = 0; di < 5 && di < (int)symbols.size(); di++)
            bp += snprintf(buf+bp, sizeof(buf)-bp, "(%.2f,%.2f) ",
                           symbols[di].real(), symbols[di].imag());
        int last = (int)symbols.size();
        int lo = std::max(0, last - 5);
        bp += snprintf(buf+bp, sizeof(buf)-bp, "sym[%d..%d]: ", lo, last-1);
        for (int di = lo; di < last && bp < 450; di++)
            bp += snprintf(buf+bp, sizeof(buf)-bp, "(%.2f,%.2f) ",
                           symbols[di].real(), symbols[di].imag());
        IRIS_LOG("[DECODE] %s", buf);
    }

    // Post-equalization decision-directed SNR estimator.
    // Measures |symbol - nearest_constellation_point|² across all decoded
    // payload symbols.  Unlike the preamble estimator (g_decode_snr above),
    // this captures the true SNR *after* PLL tracking and channel EQ —
    // ISI and phase noise that the PLL successfully tracks are excluded.
    // This is critical for gearshift: the preamble estimator biases low
    // by 3-5 dB on FM links because it counts ISI/jitter as thermal noise.
    {
        float sig_pwr = 0, err_pwr = 0;
        for (size_t si = 0; si < symbols.size(); si++) {
            // Hard-decide, re-map to nearest constellation point
            uint8_t dec_bits[8];
            demap_symbol(symbols[si], dec_bits, mod);
            auto ideal = map_symbol(dec_bits, mod);
            float dr = symbols[si].real() - ideal.real();
            float di = symbols[si].imag() - ideal.imag();
            sig_pwr += ideal.real() * ideal.real() + ideal.imag() * ideal.imag();
            err_pwr += dr * dr + di * di;
        }
        float dd_snr;
        if (err_pwr < 1e-12f)
            dd_snr = 60.0f;
        else
            dd_snr = 10.0f * std::log10(sig_pwr / err_pwr);
        IRIS_LOG("[DECODE] SNR preamble=%.1f dB  post-EQ(DD)=%.1f dB  (%d symbols)",
                 g_decode_snr, dd_snr, (int)symbols.size());
        // Use the higher of preamble and DD estimates for gearshift.
        // DD estimate is more accurate on FM (preamble biased low by ISI),
        // but on clean channels both should agree within ~1 dB.
        if (dd_snr > g_decode_snr)
            g_decode_snr = dd_snr;
    }

    std::vector<uint8_t> bits;

    if (fec != LdpcRate::NONE) {
        // Soft-decision path: LLR values give LDPC much more information.
        // Noise-variance-scaled LLRs (sigma_sq) only for QAM16+ where the
        // proper max-log-MAP scaling is critical for constellation geometry.
        // For BPSK/QPSK, the fixed scaling works well with the min-sum decoder
        // (0.75 scaling factor). Applying sigma_sq to BPSK/QPSK makes LLRs
        // ~4x larger at typical SNR, causing over-confident wrong-sign LLRs
        // that the min-sum decoder can't overcome.
        float sigma_sq = 0;
        if (mod >= Modulation::QAM16 && g_decode_snr > 0 && g_decode_snr < 50.0f) {
            float snr_lin = std::pow(10.0f, g_decode_snr / 10.0f);
            sigma_sq = 1.0f / snr_lin;
        }
        auto llrs = demap_soft(symbols, mod, sigma_sq);

        // Reliability weighting: scale each symbol's LLRs by its received magnitude.
        // Symbols with low magnitude (attenuated by VB-Cable, radio echo, fading)
        // get LLRs pushed toward zero = erasure, telling LDPC "I don't know" instead
        // of "I'm confident and wrong". This is critical for VB-Cable and radio paths.
        {
            // Compute median magnitude as reference for "normal" signal level
            std::vector<float> sorted_mags(sym_reliability);
            std::sort(sorted_mags.begin(), sorted_mags.end());
            float median_mag = sorted_mags.size() > 0 ?
                sorted_mags[sorted_mags.size() / 2] : 1.0f;
            if (median_mag < 0.01f) median_mag = 0.01f;

            int n_attenuated = 0;
            for (int si = 0; si < (int)symbols.size(); si++) {
                float weight = std::min(1.0f, sym_reliability[si] / median_mag);
                // Square the weight to more aggressively erase weak symbols
                weight *= weight;
                if (weight < 0.1f) n_attenuated++;
                for (int bi = 0; bi < bps; bi++) {
                    int llr_idx = si * bps + bi;
                    if (llr_idx < (int)llrs.size())
                        llrs[llr_idx] *= weight;
                }
            }
            if (n_attenuated > 0)
                IRIS_LOG("[DECODE] reliability: %d/%d symbols attenuated (median_mag=%.3f)",
                         n_attenuated, (int)symbols.size(), median_mag);
        }

        scramble_soft(llrs);
        if ((int)llrs.size() > encoded_bits)
            llrs.resize(encoded_bits);

        // De-interleave each LDPC block (reverses TX interleaver)
        {
            int n_fec = LdpcCodec::codeword_size(fec);
            constexpr int INTERLEAVE_STRIDE = 41;  // must match TX
            for (size_t blk = 0; blk + n_fec <= llrs.size(); blk += n_fec) {
                std::vector<float> tmp(n_fec);
                for (int i = 0; i < n_fec; i++)
                    tmp[i] = llrs[blk + (i * INTERLEAVE_STRIDE) % n_fec];
                std::copy(tmp.begin(), tmp.end(), llrs.begin() + blk);
            }
        }

        // Clamp LLR magnitude to prevent decoder overflow
        for (auto& l : llrs) {
            l = std::max(-10.0f, std::min(10.0f, l));
        }

        // LDPC tail hardening: bias padding bits toward known-zero value.
        // The encoder pads the last block with zeros, so the decoder can use
        // this prior knowledge to improve convergence on the tail block.
        // This is the #1 cause of OTA LDPC failures (block 12 systematic).
        {
            int k_fec = LdpcCodec::block_size(fec);
            int n_fec = LdpcCodec::codeword_size(fec);
            int total_data_bits = (payload_len + 4) * 8;  // payload + CRC32
            int num_blocks = (total_data_bits + k_fec - 1) / k_fec;
            int padding_bits = num_blocks * k_fec - total_data_bits;
            if (padding_bits > 0 && num_blocks > 0) {
                // Padding is in the systematic (first k) bits of the last block
                int last_block_start = (num_blocks - 1) * n_fec;
                int pad_start = last_block_start + (k_fec - padding_bits);
                for (int i = 0; i < padding_bits; i++) {
                    int idx = pad_start + i;
                    if (idx < (int)llrs.size()) {
                        // Bias toward +1 (bit=0 maps to LLR>0 after scramble/descramble)
                        // Use soft bias (3.0) instead of hard override to let decoder
                        // still correct if the channel flipped a bit
                        llrs[idx] = std::max(llrs[idx], 3.0f);
                    }
                }
                IRIS_LOG("[DECODE] tail hardening: %d padding bits biased in block %d",
                         padding_bits, num_blocks - 1);
            }
        }

        // LLR quality stats: mean |LLR| indicates channel confidence
        {
            float sum_abs = 0;
            for (auto& l : llrs) sum_abs += std::abs(l);
            float mean_llr = llrs.size() > 0 ? sum_abs / llrs.size() : 0;
            IRIS_LOG("[DECODE] LLR stats: mean_abs=%.2f, n=%zu", mean_llr, llrs.size());
        }

        // Pre-LDPC channel quality: count bit errors vs hard decision
        {
            int n_hard_errors = 0;
            std::vector<uint8_t> hard(llrs.size());
            for (size_t di = 0; di < llrs.size(); di++)
                hard[di] = llrs[di] < 0 ? 1 : 0;
            // Check how many LDPC parity equations are satisfied by hard decisions
            int k_fec = LdpcCodec::block_size(fec);
            int n_fec = LdpcCodec::codeword_size(fec);
            int n_unsatisfied = 0;
            for (size_t blk = 0; blk + n_fec <= hard.size(); blk += n_fec) {
                // Quick syndrome check via the same H matrix logic
                for (int j = 0; j < n_fec - k_fec; j++) {
                    int parity = 0;
                    // Pivot: parity bit at k_fec + j
                    parity ^= hard[blk + k_fec + j];
                    // Data columns (approximate: just check data bits near this check)
                    // Use same structure as H matrix: wc=3 layers
                    int p = n_fec - k_fec;
                    for (int layer = 0; layer < 3; layer++) {
                        int col = (j + layer * (k_fec / 3)) % k_fec;
                        parity ^= hard[blk + col];
                    }
                    if (parity) n_unsatisfied++;
                }
            }
            int n_checks = (n_fec - k_fec) * ((int)hard.size() / n_fec);
            IRIS_LOG("[DECODE] pre-LDPC: %d/%d checks unsatisfied (%.1f%%)",
                     n_unsatisfied, n_checks, n_checks > 0 ? 100.0f * n_unsatisfied / n_checks : 0.0f);
        }

        bits = LdpcCodec::decode_soft(llrs, fec);
        if (bits.empty()) {
            IRIS_LOG("[DECODE] LDPC decode failed (did not converge)");
            return false;
        }
    } else {
        // No FEC: hard decision is fine
        bits = demap_bits(symbols, mod);
        scramble_bits(bits);
    }

    int total_bytes = payload_len + 4;
    if ((int)bits.size() < total_bytes * 8) {
        IRIS_LOG("[DECODE] bits too short: %zu < %d", bits.size(), total_bytes * 8);
        return false;
    }

    std::vector<uint8_t> decoded_bytes(total_bytes, 0);
    for (int i = 0; i < total_bytes; i++) {
        for (int j = 0; j < 8; j++) {
            decoded_bytes[i] |= (bits[i * 8 + j] & 1) << j;
        }
    }

    uint32_t rx_crc = (uint32_t)decoded_bytes[payload_len + 0] |
                      ((uint32_t)decoded_bytes[payload_len + 1] << 8) |
                      ((uint32_t)decoded_bytes[payload_len + 2] << 16) |
                      ((uint32_t)decoded_bytes[payload_len + 3] << 24);

    uint32_t calc_crc = crc32(decoded_bytes.data(), payload_len);
    if (rx_crc != calc_crc) {
        IRIS_LOG("[DECODE] CRC mismatch: rx=0x%08x calc=0x%08x (len=%d)", rx_crc, calc_crc, payload_len);
        return false;
    }

    payload.assign(decoded_bytes.begin(), decoded_bytes.begin() + payload_len);

    // Record how many IQ pairs this frame consumed (for buffer drain)
    int last_symbol_pos = payload_start + payload_symbols * sps;
    g_decode_consumed_iq = (size_t)(last_symbol_pos + RRC_SPAN * sps);
    return true;
}

} // namespace iris
