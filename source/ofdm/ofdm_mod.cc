#include "ofdm/ofdm_mod.h"
#include "common/fft.h"
#include "common/logging.h"
#include "native/frame.h"       // crc32()
#include "native/constellation.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
// FM pre-emphasis compensation gain for a given frequency bin.
// Returns 1.0 if pre-emphasis compensation is disabled (corner_hz <= 0).
inline float preemph_gain(int bin, int nfft, int sample_rate,
                          float corner_hz, float gain_cap) {
    if (corner_hz <= 0.0f) return 1.0f;
    float fhz = (float)bin * (float)sample_rate / (float)nfft;
    float g = std::sqrt(1.0f + (fhz / corner_hz) * (fhz / corner_hz));
    return std::min(g, gain_cap);
}
} // anon namespace

namespace iris {

// ============================================================================
//  CRC-8 (same polynomial as native frame header: x^8+x^5+x^4+1 = 0x8C reflected)
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
//  LFSR scrambler (x^15 + x^14 + 1) — identical to native frame.cc
// ============================================================================
static void scramble_bits(std::vector<uint8_t>& bits) {
    uint16_t lfsr = 0x6959;  // fixed seed (non-zero)
    for (size_t i = 0; i < bits.size(); i++) {
        int fb = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
        bits[i] ^= (lfsr & 1);
        lfsr = (lfsr >> 1) | ((uint16_t)fb << 14);
    }
}

// ============================================================================
//  FEC rate <-> 4-bit field (same encoding as native header)
// ============================================================================
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

// ============================================================================
//  NFFT mode encoding for header (2 bits)
// ============================================================================
static int nfft_to_mode(int nfft) {
    switch (nfft) {
        case 256:  return 1;
        case 512:  return 0;  // default
        case 1024: return 2;
    }
    return 0;
}

// ============================================================================
//  Modulation from bits-per-carrier
// ============================================================================
Modulation bits_to_modulation(int bpc) {
    switch (bpc) {
        case 1: return Modulation::BPSK;
        case 2: return Modulation::QPSK;
        case 4: return Modulation::QAM16;
        case 6: return Modulation::QAM64;
        case 8: return Modulation::QAM256;
        default: return Modulation::BPSK;
    }
}

// ============================================================================
//  Uniform tone map presets (Section 6.4 of spec)
// ============================================================================
ToneMap make_uniform_tone_map(uint8_t preset_id, int n_data_carriers, int nfft) {
    ToneMap tm;
    tm.n_data_carriers = n_data_carriers;
    tm.nfft = nfft;
    tm.tone_map_id = preset_id;

    struct Preset {
        int bpc;        // bits per carrier
        LdpcRate fec;
    };
    static const Preset presets[] = {
        {0, LdpcRate::NONE},         // 0 = waterfill placeholder
        {1, LdpcRate::RATE_1_2},     // 1 = BPSK r1/2
        {2, LdpcRate::RATE_1_2},     // 2 = QPSK r1/2
        {2, LdpcRate::RATE_3_4},     // 3 = QPSK r3/4
        {4, LdpcRate::RATE_1_2},     // 4 = 16QAM r1/2
        {4, LdpcRate::RATE_3_4},     // 5 = 16QAM r3/4
        {6, LdpcRate::RATE_3_4},     // 6 = 64QAM r3/4
        {6, LdpcRate::RATE_7_8},     // 7 = 64QAM r7/8
        {8, LdpcRate::RATE_7_8},     // 8 = 256QAM r7/8
    };

    int idx = (preset_id <= 8) ? preset_id : 2;  // default to QPSK r1/2
    if (idx == 0) idx = 2;  // waterfill fallback = QPSK r1/2

    tm.bits_per_carrier.assign(n_data_carriers, (uint8_t)presets[idx].bpc);
    tm.fec_rate = presets[idx].fec;
    tm.total_bits_per_symbol = n_data_carriers * presets[idx].bpc;
    return tm;
}

// ============================================================================
//  OfdmModulator
// ============================================================================

OfdmModulator::OfdmModulator(const OfdmConfig& config)
    : config_(config)
{
}

// ----------------------------------------------------------------------------
//  symbol_to_time: place freq-domain symbols in FFT bins, IFFT, prepend CP
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmModulator::symbol_to_time(
    const std::vector<std::complex<float>>& freq_bins)
{
    int nfft = config_.nfft;

    // Copy to IFFT buffer (caller should have placed symbols at correct bins)
    std::vector<std::complex<float>> buf(freq_bins.begin(), freq_bins.end());
    if ((int)buf.size() != nfft) buf.resize(nfft, {0.0f, 0.0f});

    // Hermitian symmetry: X[N-k] = conj(X[k]) so IFFT output is real-valued.
    // This allows OFDM to generate real passband audio directly (no upconverter).
    // Only mirror bins in [1, N/2-1]; DC (bin 0) and Nyquist (bin N/2) stay as-is.
    for (int k = 1; k < nfft / 2; k++) {
        buf[nfft - k] = std::conj(buf[k]);
    }

    // IFFT: iris::ifft_complex divides by n internally.
    // OFDM convention: we want 1/sqrt(nfft) scaling, but since the IFFT does 1/n,
    // we scale by sqrt(nfft) after to get 1/sqrt(nfft) total (matches demod FFT).
    iris::ifft_complex(buf.data(), nfft);

    // Scale by nfft so that the overall scaling is ×1 (IFFT divides by n,
    // we multiply by n → net ×1; demod FFT will divide by sqrt(n) for matching).
    // Actually, the standard OFDM convention: TX does IFFT with 1/N, and RX does
    // FFT without 1/N, or vice versa. Since iris::ifft divides by N, we scale
    // by N here so that |time sample| ≈ 1 for unit-power constellation points.
    float scale = (float)nfft;
    for (auto& s : buf) s *= scale;

    // Prepend cyclic prefix (last cp_samples of IFFT output)
    int cp = config_.cp_samples;
    std::vector<std::complex<float>> out;
    out.reserve(cp + nfft);
    out.insert(out.end(), buf.end() - cp, buf.end());
    out.insert(out.end(), buf.begin(), buf.end());
    return out;
}

// ----------------------------------------------------------------------------
//  generate_training_symbol_1: Schmidl-Cox (even subcarriers only → identical halves)
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmModulator::generate_training_symbol_1() {
    int nfft = config_.nfft;
    std::vector<std::complex<float>> freq(nfft, {0.0f, 0.0f});

    // PN sequence for even-indexed used subcarriers (BPSK: +1 or -1).
    // Simple LFSR-based: seed = 0xACE1.
    // Pre-emphasis compensation: boost higher carriers to counteract FM
    // de-emphasis (6 dB/octave). This reduces spectral tilt BEFORE the
    // deviation limiter, lowering nonlinear clipping that degrades Schmidl-Cox.
    // Even-bin-only property preserved → halves still identical.
    uint16_t pn = 0xACE1;
    for (int i = 0; i < (int)config_.used_carrier_bins.size(); i++) {
        int bin = config_.used_carrier_bins[i];
        if (bin % 2 == 0) {  // even FFT bins only → identical halves in time domain
            float val = (pn & 1) ? 1.0f : -1.0f;
            float g = preemph_gain(bin, nfft, config_.sample_rate,
                                   config_.fm_preemph_corner_hz,
                                   config_.fm_preemph_gain_cap);
            freq[bin] = {val * g, 0.0f};
            // Advance PN: x^15+x^14+1
            int fb = ((pn >> 14) ^ (pn >> 13)) & 1;
            pn = (pn >> 1) | ((uint16_t)fb << 14);
        }
        // odd FFT bins stay zero → time domain has identical halves
    }

    return symbol_to_time(freq);
}

// ----------------------------------------------------------------------------
//  generate_training_symbol_2: all used subcarriers = +1
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmModulator::generate_training_symbol_2() {
    int nfft = config_.nfft;
    std::vector<std::complex<float>> freq(nfft, {0.0f, 0.0f});

    // Pre-emphasis compensation: same curve as train1, data, header, and pilots.
    // H[k] from channel estimator includes gain[k]. All other symbol types
    // also carry gain[k], so it cancels during equalization.
    for (int bin : config_.used_carrier_bins) {
        float g = preemph_gain(bin, config_.nfft, config_.sample_rate,
                               config_.fm_preemph_corner_hz,
                               config_.fm_preemph_gain_cap);
        freq[bin] = {g, 0.0f};
    }

    return symbol_to_time(freq);
}

// ----------------------------------------------------------------------------
//  generate_data_symbol: map freq_symbols (n_used_carriers) to OFDM symbol
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmModulator::generate_data_symbol(
    const std::vector<std::complex<float>>& freq_symbols)
{
    int nfft = config_.nfft;
    std::vector<std::complex<float>> freq(nfft, {0.0f, 0.0f});

    int n = std::min((int)freq_symbols.size(), (int)config_.used_carrier_bins.size());
    for (int i = 0; i < n; i++) {
        freq[config_.used_carrier_bins[i]] = freq_symbols[i];
    }

    return symbol_to_time(freq);
}

// ----------------------------------------------------------------------------
//  generate_pilot_symbol: all used subcarriers = +1 (block pilot)
// ----------------------------------------------------------------------------
std::vector<std::complex<float>> OfdmModulator::generate_pilot_symbol() {
    // Identical to training symbol 2
    return generate_training_symbol_2();
}

// ----------------------------------------------------------------------------
//  encode_ofdm_header: 36 bits packed into byte array for BPSK mapping
//  Format: [4 tone_map_id][4 fec_rate][15 payload_len][4 nfft_mode]
//          [1 harq_flag][8 CRC-8] = 36 bits
// ----------------------------------------------------------------------------
std::vector<uint8_t> OfdmModulator::encode_ofdm_header(
    uint8_t tone_map_id, LdpcRate fec, uint16_t payload_len,
    int nfft_mode, bool harq_flag)
{
    // Pack into 28 data bits + 8 CRC bits = 36 bits total.
    // Pack MSB-first into bytes for CRC computation.
    // Byte 0: [4 tone_map_id][4 fec_rate]
    // Byte 1: [8 MSB of payload_len (top 4 bits + next 4)]  -> actually [12 bits payload_len] spans bytes 1-2
    // Let's pack bit-by-bit for clarity.

    std::vector<uint8_t> bits(36, 0);
    int pos = 0;

    // tone_map_id: 4 bits
    for (int i = 3; i >= 0; i--)
        bits[pos++] = (tone_map_id >> i) & 1;

    // fec_rate: 4 bits
    uint8_t fec_field = fec_to_field(fec);
    for (int i = 3; i >= 0; i--)
        bits[pos++] = (fec_field >> i) & 1;

    // payload_len: 15 bits (max 32767 bytes)
    for (int i = 14; i >= 0; i--)
        bits[pos++] = (payload_len >> i) & 1;

    // nfft_mode: 4 bits
    for (int i = 3; i >= 0; i--)
        bits[pos++] = (nfft_mode >> i) & 1;

    // harq_flag: 1 bit
    bits[pos++] = harq_flag ? 1 : 0;

    // Now compute CRC-8 over the first 28 bits (packed into 3.5 bytes).
    // Pack first 28 bits into bytes for CRC.
    uint8_t header_bytes[4] = {};
    for (int i = 0; i < 28; i++) {
        header_bytes[i / 8] |= bits[i] << (7 - (i % 8));
    }
    // CRC over 4 bytes (last 4 bits of byte 3 are zero-padded from the reserved field)
    uint8_t crc = crc8(header_bytes, 4);

    // CRC-8: 8 bits
    for (int i = 7; i >= 0; i--)
        bits[pos++] = (crc >> i) & 1;

    return bits;
}

// ============================================================================
//  build_ofdm_frame: complete TX chain
// ============================================================================
std::vector<std::complex<float>> OfdmModulator::build_ofdm_frame(
    const uint8_t* payload, size_t len, const ToneMap& tone_map, LdpcRate fec)
{
    // ---- 1. Prepend 2-byte length prefix, then append CRC-32 ----
    // Headerless frame (Mercury approach): payload length is embedded inside
    // the LDPC-protected data.  Format: [len_lo][len_hi][payload...][CRC32].
    // CRC-32 covers the length prefix + payload.
    if (fec != LdpcRate::NONE) {
        int k = LdpcCodec::block_size(fec);
        int max_payload = k / 8 - 4 - 2;  // minus CRC-32 and length prefix
        if ((int)len > max_payload) {
            IRIS_LOG("[OFDM-TX] payload %zu bytes exceeds capacity %d — rejected", len, max_payload);
            return {};
        }
    }
    uint16_t payload_len_u16 = (uint16_t)len;
    std::vector<uint8_t> payload_with_crc(2 + len + 4);
    payload_with_crc[0] = payload_len_u16 & 0xFF;
    payload_with_crc[1] = (payload_len_u16 >> 8) & 0xFF;
    std::memcpy(payload_with_crc.data() + 2, payload, len);
    uint32_t crc = crc32(payload_with_crc.data(), 2 + len);
    payload_with_crc[2 + len + 0] = (crc >>  0) & 0xFF;
    payload_with_crc[2 + len + 1] = (crc >>  8) & 0xFF;
    payload_with_crc[2 + len + 2] = (crc >> 16) & 0xFF;
    payload_with_crc[2 + len + 3] = (crc >> 24) & 0xFF;

    // ---- 2. Convert to bits ----
    std::vector<uint8_t> data_bits;
    data_bits.reserve(payload_with_crc.size() * 8);
    for (size_t i = 0; i < payload_with_crc.size(); i++) {
        for (int j = 0; j < 8; j++) {
            data_bits.push_back((payload_with_crc[i] >> j) & 1);
        }
    }

    // ---- 3. LDPC encode ----
    std::vector<uint8_t> coded_bits;
    if (fec != LdpcRate::NONE) {
        coded_bits = LdpcCodec::encode(data_bits, fec);

        // Interleave each LDPC block (stride-41, same as native)
        int n_fec = LdpcCodec::codeword_size(fec);
        constexpr int INTERLEAVE_STRIDE = 41;  // coprime to 1600
        for (size_t blk = 0; blk + n_fec <= coded_bits.size(); blk += n_fec) {
            std::vector<uint8_t> tmp(n_fec);
            for (int i = 0; i < n_fec; i++)
                tmp[(i * INTERLEAVE_STRIDE) % n_fec] = coded_bits[blk + i];
            std::copy(tmp.begin(), tmp.end(), coded_bits.begin() + blk);
        }
    } else {
        coded_bits = std::move(data_bits);
    }

    // ---- 3b. Fixed frame: exactly 1 LDPC block ----
    // Both TX and RX compute n_data_symbols from this, so frame size is deterministic.
    if (fec != LdpcRate::NONE) {
        int n_fec_expected = LdpcCodec::codeword_size(fec);
        coded_bits.resize(n_fec_expected, 0);  // pad or truncate to exactly 1 block
    }

    // ---- 4. BICM interleave (column-row per OFDM symbol, QAM16+ uniform) ----
    // For uniform tone maps (all carriers same bpc ≥ 4), spread adjacent coded bits
    // across different carriers. Write column-major into (bpc × n_carriers) matrix,
    // read row-major. This ensures adjacent LDPC bits map to different subcarriers,
    // decorrelating frequency-selective fading.
    // For waterfill (mixed bpc), per-carrier mapping provides implicit BICM.
    if (tone_map.tone_map_id > 0 && tone_map.total_bits_per_symbol > 0) {
        int bpc = tone_map.bits_per_carrier[0];  // uniform: all same
        int n_carriers = tone_map.n_data_carriers;
        int sym_bits = tone_map.total_bits_per_symbol;
        if (bpc >= 4 && sym_bits == bpc * n_carriers) {
            int n_syms = 0;
            for (size_t sym_start = 0; sym_start + sym_bits <= coded_bits.size();
                 sym_start += sym_bits)
            {
                std::vector<uint8_t> tmp(sym_bits);
                for (int i = 0; i < sym_bits; i++) {
                    int row = i % bpc;
                    int col = i / bpc;
                    tmp[row * n_carriers + col] = coded_bits[sym_start + i];
                }
                std::copy(tmp.begin(), tmp.end(), coded_bits.begin() + sym_start);
                n_syms++;
            }
            IRIS_LOG("[OFDM-TX] BICM interleave: %d symbols, %d×%d matrix (bpc×carriers)",
                     n_syms, bpc, n_carriers);
        }
    }

    // ---- 4b. Frequency-time interleave (global stride across all symbols) ----
    // Spread bits from the same LDPC block across different OFDM symbols (time
    // diversity) and different carrier positions (frequency diversity).
    // Uses stride 173 (prime, coprime to any practical frame length).
    {
        constexpr int FREQ_TIME_STRIDE = 173;
        int N = (int)coded_bits.size();
        if (N > FREQ_TIME_STRIDE) {
            std::vector<uint8_t> tmp(N);
            for (int i = 0; i < N; i++)
                tmp[(i * FREQ_TIME_STRIDE) % N] = coded_bits[i];
            coded_bits = std::move(tmp);
            IRIS_LOG("[OFDM-TX] freq-time interleave: stride=%d, %d bits", FREQ_TIME_STRIDE, N);
        }
    }

    // ---- 5. Scramble with LFSR ----
    scramble_bits(coded_bits);

    // ---- 6. Map bits to per-subcarrier symbols ----
    // For each OFDM data symbol, consume total_bits_per_symbol bits from coded_bits.
    // Split by carrier according to tone_map.bits_per_carrier, map each carrier's
    // bits to its constellation point.
    int bps_total = tone_map.total_bits_per_symbol;
    if (bps_total <= 0) {
        IRIS_LOG("[OFDM-TX] ERROR: tone map has 0 bits per symbol");
        return {};
    }

    // Pad coded bits to fill complete OFDM symbols
    while (coded_bits.size() % bps_total != 0)
        coded_bits.push_back(0);

    int n_data_symbols = (int)coded_bits.size() / bps_total;

    // Build per-OFDM-symbol arrays of data carrier constellation points
    std::vector<std::vector<std::complex<float>>> data_sym_carriers(n_data_symbols);
    int bit_pos = 0;
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

    for (int s = 0; s < n_data_symbols; s++) {
        data_sym_carriers[s].resize(tone_map.n_data_carriers);
        for (int k = 0; k < tone_map.n_data_carriers; k++) {
            int bpc = tone_map.bits_per_carrier[k];
            if (bpc == 0) {
                data_sym_carriers[s][k] = {0.0f, 0.0f};
                continue;
            }
            Modulation mod = bits_to_modulation(bpc);

            // NUC: use optimized constellation points (16/64/256QAM only)
            const NucTable* nuc = nullptr;
            if (tone_map.use_nuc && bpc >= 4)  // NUC only for 16QAM+
                nuc = get_nuc_table(mod, fec_r16);

            if (nuc) {
                // Pack bits into symbol index (LSB-first, matching demap_soft_nuc
                // which tests (s >> b) & 1 for bit b — bit 0 is LSB)
                int sym_idx = 0;
                for (int b = 0; b < bpc; b++)
                    sym_idx |= (coded_bits[bit_pos + b] & 1) << b;
                data_sym_carriers[s][k] = map_symbol_nuc(sym_idx, nuc);
            } else {
                data_sym_carriers[s][k] = map_symbol(&coded_bits[bit_pos], mod);
            }
            bit_pos += bpc;
        }
    }

    // ---- 6b. Per-carrier power normalization (M7) ----
    // Different modulations have different average constellation energies
    // (BPSK ~1, QPSK ~1, 16QAM ~10, 64QAM ~42, 256QAM ~170 for integer grids).
    // Normalize each carrier's symbol to unit average power so all carriers
    // contribute equally to total deviation regardless of modulation order.
    for (int s = 0; s < n_data_symbols; s++) {
        for (int k = 0; k < tone_map.n_data_carriers; k++) {
            int bpc = tone_map.bits_per_carrier[k];
            if (bpc == 0) continue;
            // Average energy for standard QAM: E = 2/3 * (M-1) where M = constellation size
            int M_order = 1 << bpc;
            float avg_energy = (M_order <= 2) ? 1.0f : (2.0f/3.0f) * (M_order - 1);
            float norm = std::sqrt(avg_energy);
            data_sym_carriers[s][k] /= norm;
        }
    }

    // ---- 6c. TX pre-emphasis compensation (M1) ----
    // FM de-emphasis causes higher-frequency carriers to lose power at the
    // receiver (6 dB/octave rolloff). Pre-compensate by boosting higher carriers
    // so all carriers arrive at roughly equal power after de-emphasis.
    // Same gain curve as training symbols, header, and pilots — the gain
    // cancels in H[k] during equalization.  Disabled when corner_hz <= 0
    // (flat audio data port connection).
    if (config_.fm_preemph_corner_hz > 0.0f) {
        for (int s = 0; s < n_data_symbols; s++) {
            for (int k = 0; k < tone_map.n_data_carriers; k++) {
                if (tone_map.bits_per_carrier[k] == 0) continue;
                int used_idx = 0;
                int data_count = 0;
                for (int i = 0; i < (int)config_.used_carrier_bins.size(); i++) {
                    bool pilot = (i % config_.pilot_carrier_spacing == 0);
                    if (!pilot) {
                        if (data_count == k) { used_idx = i; break; }
                        data_count++;
                    }
                }
                int bin = config_.used_carrier_bins[used_idx];
                data_sym_carriers[s][k] *= preemph_gain(bin, config_.nfft,
                    config_.sample_rate, config_.fm_preemph_corner_hz,
                    config_.fm_preemph_gain_cap);
            }
        }
    }

    // ---- 7. Insert pilot carriers ----
    // For each OFDM data symbol, build the full n_used_carriers vector:
    // pilot positions get +1, data positions get their constellation point.
    // We need to know which used_carrier index is pilot vs data.

    // Build a lookup: for each used carrier index, is it a pilot?
    int n_used = config_.n_used_carriers;
    std::vector<bool> is_pilot(n_used, false);
    // Pilot positions: every 4th used carrier (indices 0, 4, 8, ...)
    for (int i = 0; i < n_used; i += config_.pilot_carrier_spacing) {
        is_pilot[i] = true;
    }

    auto build_used_carrier_symbols = [&](const std::vector<std::complex<float>>& data_carriers)
        -> std::vector<std::complex<float>>
    {
        std::vector<std::complex<float>> all_used(n_used);
        int data_idx = 0;
        for (int i = 0; i < n_used; i++) {
            if (is_pilot[i]) {
                // Comb pilot: +1 × pre-emphasis gain (must match block pilots / train2)
                int bin = config_.used_carrier_bins[i];
                float g = preemph_gain(bin, config_.nfft, config_.sample_rate,
                                       config_.fm_preemph_corner_hz,
                                       config_.fm_preemph_gain_cap);
                all_used[i] = {g, 0.0f};
            } else {
                if (data_idx < (int)data_carriers.size())
                    all_used[i] = data_carriers[data_idx++];
                else
                    all_used[i] = {0.0f, 0.0f};
            }
        }
        return all_used;
    };

    // ---- 8. Generate training symbols ----
    auto train1 = generate_training_symbol_1();
    auto train2 = generate_training_symbol_2();

    // Preamble boost: √2 (3 dB) extra amplitude for detection headroom.
    // FM radio pre-emphasis + deviation limiting degrades OFDM peaks;
    // boosting the preamble ensures Schmidl-Cox metric stays above threshold.
    constexpr float PREAMBLE_BOOST = 1.4142f;  // sqrt(2)
    for (auto& s : train1) s *= PREAMBLE_BOOST;
    for (auto& s : train2) s *= PREAMBLE_BOOST;

    // ---- 9. (Header removed — config pre-negotiated, Mercury approach) ----

    // ---- 10. Assemble data symbols with block pilots every 8th ----
    std::vector<std::vector<std::complex<float>>> all_data_time;
    int data_sym_count = 0;
    for (int s = 0; s < n_data_symbols; s++) {
        // Insert block pilot every pilot_symbol_spacing data symbols
        if (data_sym_count > 0 &&
            (data_sym_count % config_.pilot_symbol_spacing) == 0)
        {
            all_data_time.push_back(generate_pilot_symbol());
        }
        auto used = build_used_carrier_symbols(data_sym_carriers[s]);
        all_data_time.push_back(generate_data_symbol(used));
        data_sym_count++;
    }

    // ---- 11. Tail symbol (all used subcarriers = +1) ----
    auto tail = generate_pilot_symbol();

    // ---- 12. Concatenate all time-domain symbols ----
    // Frame: [train1][train2][data+block_pilots][tail] — no header symbols
    int total_ofdm_symbols = 2 + (int)all_data_time.size() + 1;
    int sym_len = config_.symbol_samples();
    std::vector<std::complex<float>> frame;
    frame.reserve(total_ofdm_symbols * sym_len);

    // Training
    frame.insert(frame.end(), train1.begin(), train1.end());
    frame.insert(frame.end(), train2.begin(), train2.end());

    // Data + block pilots
    for (auto& ds : all_data_time)
        frame.insert(frame.end(), ds.begin(), ds.end());

    // Tail
    frame.insert(frame.end(), tail.begin(), tail.end());

    // ---- 13. Iterative clip-and-filter for FM PAPR control ----
    // Soft clip at 2.5x RMS (8 dB). The FM radio's audio input IS amplitude-
    // sensitive — its deviation limiter hard-clips uncontrolled peaks, destroying
    // subcarrier orthogonality. Controlled soft clipping is much less harmful
    // than uncontrolled hard clipping.
    // Three iterations of clip-and-filter to better control spectral regrowth.
    //
    // CRITICAL: Only clip data/header/pilot symbols — NOT the training preamble.
    // Clipping destroys the half-symbol repetition property that Schmidl-Cox
    // relies on for frame detection. The preamble has √2 boost and relatively
    // low PAPR (BPSK on even carriers), so it doesn't need clipping.
    int n_clipped = 0;

    constexpr float CLIP_RATIO = 2.5f;  // 8 dB PAPR — keeps signal within FM radio's linear deviation range
    constexpr int CLIP_ITERS = 3;

    // Skip the 2 training symbols (train1 + train2)
    int preamble_samples = 2 * sym_len;
    if ((int)frame.size() > preamble_samples) {
        // Create a view of the data portion only
        std::vector<std::complex<float>> data_part(
            frame.begin() + preamble_samples, frame.end());
        for (int iter = 0; iter < CLIP_ITERS; iter++) {
            n_clipped += soft_clip(data_part, CLIP_RATIO);
        }
        // Copy clipped data back
        std::copy(data_part.begin(), data_part.end(),
                  frame.begin() + preamble_samples);
    }
    if (n_clipped > 0)
        IRIS_LOG("[OFDM-TX] clip-filter: %d iters, ratio=%.1f, %d samples clipped",
                 CLIP_ITERS, CLIP_RATIO, n_clipped);

    // ---- 14. Compute PAPR for logging ----
    float max_power = 0.0f;
    float mean_power = 0.0f;
    for (auto& s : frame) {
        float p = std::norm(s);  // |s|^2
        if (p > max_power) max_power = p;
        mean_power += p;
    }
    mean_power /= (float)frame.size();
    float papr_db = (mean_power > 0.0f)
        ? 10.0f * std::log10(max_power / mean_power) : 0.0f;

    IRIS_LOG("[OFDM-TX] frame: %d OFDM syms, %d bytes, PAPR=%.1f dB, %d clipped, NUC=%s",
             total_ofdm_symbols, (int)len, papr_db, n_clipped,
             tone_map.use_nuc ? "ON" : "off");

    return frame;
}

// ============================================================================
//  soft_clip: limit amplitude to clip_ratio × RMS, preserve phase
// ============================================================================
int OfdmModulator::soft_clip(std::vector<std::complex<float>>& samples, float clip_ratio) {
    if (samples.empty()) return 0;

    // Compute RMS
    float sum_sq = 0.0f;
    for (auto& s : samples)
        sum_sq += std::norm(s);  // |s|^2
    float rms = std::sqrt(sum_sq / (float)samples.size());

    if (rms < 1e-12f) return 0;

    float threshold = clip_ratio * rms;
    int n_clipped = 0;

    for (auto& s : samples) {
        float mag = std::abs(s);
        if (mag > threshold) {
            s *= (threshold / mag);  // scale to threshold, preserve phase
            n_clipped++;
        }
    }

    return n_clipped;
}

} // namespace iris
