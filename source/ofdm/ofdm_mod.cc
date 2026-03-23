#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_papr.h"    // ofdm_papr_clip()
#include "ofdm/ofdm_sync.h"    // generate_zc_training_symbol()
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

// Small-N DFT/IDFT for DFT-spread OFDM (SC-FDMA).
// O(N²) — fine for N < 100 (our data carrier count).
// forward=true: DFT (analysis), forward=false: IDFT (synthesis). Both scale by 1/√N.
static void small_dft(const std::complex<float>* in, std::complex<float>* out,
                       int N, bool forward) {
    float sign = forward ? -1.0f : 1.0f;
    float scale = forward ? 1.0f / std::sqrt((float)N) : 1.0f / std::sqrt((float)N);
    for (int k = 0; k < N; k++) {
        std::complex<float> sum(0.0f, 0.0f);
        for (int n = 0; n < N; n++) {
            float angle = sign * 2.0f * (float)M_PI * (float)k * (float)n / (float)N;
            sum += in[n] * std::complex<float>(std::cos(angle), std::sin(angle));
        }
        out[k] = sum * scale;
    }
}

// FM TX de-emphasis gain: attenuate higher carriers so that AFTER the radio's
// own pre-emphasis the signal is flat entering the deviation limiter, minimising
// nonlinear clipping.  H_deemph(f) = 1 / sqrt(1 + (f/fc)^2).
// Returns 1.0 if disabled (corner_hz <= 0).
// gain_cap controls maximum attenuation ratio (e.g. 10 → −20 dB floor).
inline float fm_tx_deemph_gain(int bin, int nfft, int sample_rate,
                                float corner_hz, float gain_cap) {
    if (corner_hz <= 0.0f) return 1.0f;
    float fhz = (float)bin * (float)sample_rate / (float)nfft;
    float g = 1.0f / std::sqrt(1.0f + (fhz / corner_hz) * (fhz / corner_hz));
    return std::max(g, 1.0f / gain_cap);  // floor at 1/cap (e.g. 0.1 for cap=10)
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
        {0, LdpcRate::NONE},         //  0 = waterfill placeholder
        {1, LdpcRate::RATE_1_2},     //  1 = O0: BPSK r1/2
        {2, LdpcRate::RATE_1_2},     //  2 = O1: QPSK r1/2
        {2, LdpcRate::RATE_3_4},     //  3 = O2: QPSK r3/4
        {4, LdpcRate::RATE_1_2},     //  4 = O3: 16QAM r1/2
        {4, LdpcRate::RATE_5_8},     //  5 = O4: 16QAM r5/8
        {4, LdpcRate::RATE_3_4},     //  6 = O5: 16QAM r3/4
        {6, LdpcRate::RATE_5_8},     //  7 = O6: 64QAM r5/8
        {6, LdpcRate::RATE_3_4},     //  8 = O7: 64QAM r3/4
        {8, LdpcRate::RATE_5_8},     //  9 = O8: 256QAM r5/8
        {8, LdpcRate::RATE_3_4},     // 10 = O9: 256QAM r3/4
    };

    int idx = (preset_id <= 10) ? preset_id : 2;  // default to QPSK r1/2
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
    // TX de-emphasis: attenuate higher carriers so radio pre-emphasis
    // produces a flat signal entering the deviation limiter.
    // Even-bin-only property preserved → halves still identical.
    uint16_t pn = 0xACE1;
    for (int i = 0; i < (int)config_.used_carrier_bins.size(); i++) {
        int bin = config_.used_carrier_bins[i];
        if (bin % 2 == 0) {  // even FFT bins only → identical halves in time domain
            float val = (pn & 1) ? 1.0f : -1.0f;
            float g = fm_tx_deemph_gain(bin, nfft, config_.sample_rate,
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

    // TX de-emphasis: same curve as train1, ZC, data, and pilots.
    // Channel estimator H[k] includes the gain. All symbol types carry the
    // same gain, so it cancels during equalization.
    for (int bin : config_.used_carrier_bins) {
        float g = fm_tx_deemph_gain(bin, config_.nfft, config_.sample_rate,
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
    const uint8_t* payload, size_t len, const ToneMap& tone_map, LdpcRate fec,
    int n_codewords)
{
    if (n_codewords < 1) n_codewords = 1;

    // ---- 1. Prepend 2-byte length prefix, then append CRC-32 per block ----
    // Each LDPC block carries: [len_lo][len_hi][payload_chunk...][CRC32].
    // Multi-codeword: payload is split across blocks, each self-contained.
    int k = (fec != LdpcRate::NONE) ? LdpcCodec::block_size(fec) : 0;
    int max_payload_per_block = (k > 0) ? (k / 8 - 4 - 2) : (int)len;
    int total_capacity = max_payload_per_block * n_codewords;

    if ((int)len > total_capacity) {
        IRIS_LOG("[OFDM-TX] payload %zu bytes exceeds %d-block capacity %d — rejected",
                 len, n_codewords, total_capacity);
        return {};
    }

    // Split payload across blocks
    std::vector<uint8_t> coded_bits;
    size_t payload_offset = 0;

    for (int blk = 0; blk < n_codewords; blk++) {
        // Compute chunk size for this block
        size_t remaining = len - payload_offset;
        size_t chunk = std::min(remaining, (size_t)max_payload_per_block);

        // Build [len][payload_chunk][CRC32]
        uint16_t chunk_len = (uint16_t)chunk;
        std::vector<uint8_t> block_data(2 + chunk + 4);
        block_data[0] = chunk_len & 0xFF;
        block_data[1] = (chunk_len >> 8) & 0xFF;
        if (chunk > 0)
            std::memcpy(block_data.data() + 2, payload + payload_offset, chunk);
        uint32_t crc = crc32(block_data.data(), 2 + chunk);
        block_data[2 + chunk + 0] = (crc >>  0) & 0xFF;
        block_data[2 + chunk + 1] = (crc >>  8) & 0xFF;
        block_data[2 + chunk + 2] = (crc >> 16) & 0xFF;
        block_data[2 + chunk + 3] = (crc >> 24) & 0xFF;
        payload_offset += chunk;

        // Convert to bits
        std::vector<uint8_t> data_bits;
        data_bits.reserve(block_data.size() * 8);
        for (size_t i = 0; i < block_data.size(); i++) {
            for (int j = 0; j < 8; j++) {
                data_bits.push_back((block_data[i] >> j) & 1);
            }
        }

        // LDPC encode this block
        if (fec != LdpcRate::NONE) {
            auto block_coded = LdpcCodec::encode(data_bits, fec);

            // Interleave within block (stride-41)
            int n_fec = LdpcCodec::codeword_size(fec);
            constexpr int INTERLEAVE_STRIDE = 41;
            {
                std::vector<uint8_t> tmp(n_fec);
                for (int i = 0; i < n_fec; i++)
                    tmp[(i * INTERLEAVE_STRIDE) % n_fec] = block_coded[i];
                block_coded = std::move(tmp);
            }

            // Pad/truncate to exact codeword size
            block_coded.resize(n_fec, 0);
            coded_bits.insert(coded_bits.end(), block_coded.begin(), block_coded.end());
        } else {
            coded_bits.insert(coded_bits.end(), data_bits.begin(), data_bits.end());
        }
    }

    // ---- 3b. Frame size: exactly n_codewords LDPC blocks ----
    if (fec != LdpcRate::NONE) {
        int n_fec_expected = n_codewords * LdpcCodec::codeword_size(fec);
        coded_bits.resize(n_fec_expected, 0);
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

    // ---- 6b. Per-carrier power normalization: REMOVED ----
    // map_symbol() / qam_map() already normalize to unit average power.
    // The previous code here divided by sqrt(avg_energy) a SECOND time,
    // which scaled 16QAM+ symbols down by 1/sqrt(10) (16QAM), 1/sqrt(42)
    // (64QAM), etc. BPSK/QPSK survived because their demap only checks
    // sign, but 16QAM+ demap/DD-CPE made wrong hard decisions and LDPC
    // decode failed from incorrect LLRs.

    // ---- 6c. DFT-spread (SC-FDMA precoding) ----
    // Apply N_data-point DFT to each OFDM symbol's data carriers BEFORE
    // de-emphasis and inserting into the OFDM grid. This spreads each QAM
    // symbol across all data subcarriers, making the time-domain signal
    // single-carrier-like with much lower PAPR (~5-7 dB vs ~10-11 dB for
    // plain OFDM). The RX applies IDFT after equalization to recover QAM
    // symbols.
    //
    // IMPORTANT: DFT-spread must happen BEFORE de-emphasis. De-emphasis is a
    // per-carrier frequency-domain operation that the RX channel equalizer
    // can undo (it's part of H[k]). If de-emphasis were applied before
    // DFT-spread, the per-carrier scaling would be baked into the spread
    // signal and the RX IDFT would recover scaled QAM symbols instead of
    // the originals — breaking 16QAM+ demapping (BPSK/QPSK survive because
    // their demappers only check sign, not amplitude).
    if (config_.dft_spread && tone_map.n_data_carriers > 1) {
        int N = tone_map.n_data_carriers;
        std::vector<std::complex<float>> dft_out(N);
        for (int s = 0; s < n_data_symbols; s++) {
            small_dft(data_sym_carriers[s].data(), dft_out.data(), N, true);
            data_sym_carriers[s] = dft_out;
        }
        IRIS_LOG("[OFDM-TX] DFT-spread: %d-point DFT on %d data symbols", N, n_data_symbols);
    }

    // ---- 6d. TX de-emphasis (M1) ----
    // Attenuate higher carriers so that AFTER the radio's own pre-emphasis
    // the signal is flat entering the deviation limiter, minimising
    // nonlinear clipping.  The RX radio de-emphasis restores the tilt, and
    // the equaliser removes it using H[k].  Same gain curve as training
    // symbols and pilots — cancels in H[k] during equalization.
    // Disabled when corner_hz <= 0 (flat audio data port connection).
    // Applied AFTER DFT-spread so the equalizer can cleanly undo it.
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
                data_sym_carriers[s][k] *= fm_tx_deemph_gain(bin, config_.nfft,
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
                // Comb pilot: +1 × de-emphasis gain (must match block pilots / train2)
                int bin = config_.used_carrier_bins[i];
                float g = fm_tx_deemph_gain(bin, config_.nfft, config_.sample_rate,
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

    // ---- 8. Generate ZC training symbols ----
    // Both training symbols are identical Zadoff-Chu sequences (root=7).
    // RX estimates CFO from the phase rotation between them.
    auto zc_sym = generate_zc_training_symbol(config_, /*root=*/7);  // nfft samples, no CP

    // Add cyclic prefix: [last cp_samples of symbol][symbol]
    int cp = config_.cp_samples;
    std::vector<std::complex<float>> train1;
    train1.reserve(cp + config_.nfft);
    train1.insert(train1.end(), zc_sym.end() - cp, zc_sym.end());
    train1.insert(train1.end(), zc_sym.begin(), zc_sym.end());
    auto train2 = train1;  // identical copy for CFO estimation

    // Preamble boost: √2 (3 dB) extra amplitude for detection headroom.
    // Even with TX de-emphasis the radio may clip some peaks; the boost
    // ensures ZC cross-correlation stays above detection threshold.
    constexpr float PREAMBLE_BOOST = 1.4142f;  // sqrt(2)
    for (auto& s : train1) s *= PREAMBLE_BOOST;
    for (auto& s : train2) s *= PREAMBLE_BOOST;

    // ---- 9. Sync word symbol ----
    // Deterministic BPSK symbol after preamble with embedded CRC-8.
    // First (n_used - 8) carriers carry LFSR pseudo-random BPSK bits.
    // Last 8 carriers carry CRC-8 of those bits (one bit per carrier).
    // RX equalizes, hard-decides, recomputes CRC-8 and compares.
    // False triggers: random bits → CRC match probability = 1/256.
    // Real frames: known pattern → CRC always matches (0 errors at decodable SNR).
    // Purely structural check — no threshold, no gray zone.
    std::vector<std::complex<float>> sync_word_sym;
    {
        std::vector<std::complex<float>> sync_freq(config_.nfft, {0.0f, 0.0f});
        int n_data_bits = n_used - 8;  // LFSR bits (data portion)

        // Generate LFSR bits for data carriers
        uint16_t lfsr = 0xACE1;
        std::vector<uint8_t> sw_bits(n_used);
        for (int i = 0; i < n_data_bits; i++) {
            sw_bits[i] = lfsr & 1;
            uint16_t bit = lfsr & 1;
            lfsr >>= 1;
            if (bit) lfsr ^= 0xB400;
        }

        // Compute CRC-8 over data bits (pack into bytes)
        int n_bytes = (n_data_bits + 7) / 8;
        std::vector<uint8_t> packed(n_bytes, 0);
        for (int i = 0; i < n_data_bits; i++)
            packed[i / 8] |= (sw_bits[i] << (i % 8));
        uint8_t crc = crc8(packed.data(), n_bytes);

        // Place CRC-8 in last 8 carriers
        for (int i = 0; i < 8; i++)
            sw_bits[n_data_bits + i] = (crc >> i) & 1;

        // Map all bits to BPSK on carriers with de-emphasis
        for (int i = 0; i < n_used; i++) {
            int bin = config_.used_carrier_bins[i];
            float g = fm_tx_deemph_gain(bin, config_.nfft, config_.sample_rate,
                                   config_.fm_preemph_corner_hz,
                                   config_.fm_preemph_gain_cap);
            float sign = sw_bits[i] ? 1.0f : -1.0f;
            sync_freq[bin] = {sign * g, 0.0f};
        }
        sync_word_sym = symbol_to_time(sync_freq);
    }

    // ---- 10. Assemble data symbols with block pilots + dense pilot rows ----
    std::vector<std::vector<std::complex<float>>> all_data_time;
    int data_sym_count = 0;
    int pilot_rows_inserted = 0;
    for (int s = 0; s < n_data_symbols; s++) {
        // Insert block pilot every pilot_symbol_spacing data symbols
        if (data_sym_count > 0 &&
            (data_sym_count % config_.pilot_symbol_spacing) == 0)
        {
            all_data_time.push_back(generate_pilot_symbol());
        }
        // Insert dense pilot row every pilot_row_spacing data symbols
        // (distinct from block pilots — pilot rows are more frequent and
        // provide full channel re-estimation within long frames)
        if (config_.pilot_row_spacing > 0 && data_sym_count > 0 &&
            (data_sym_count % config_.pilot_row_spacing) == 0)
        {
            all_data_time.push_back(generate_pilot_symbol());
            pilot_rows_inserted++;
        }
        auto used = build_used_carrier_symbols(data_sym_carriers[s]);
        all_data_time.push_back(generate_data_symbol(used));
        data_sym_count++;
    }
    if (pilot_rows_inserted > 0) {
        IRIS_LOG("[OFDM-TX] inserted %d dense pilot rows (every %d data syms)",
                 pilot_rows_inserted, config_.pilot_row_spacing);
    }

    // ---- 11. Tail symbol (all used subcarriers = +1) ----
    auto tail = generate_pilot_symbol();

    // ---- 12. Concatenate all time-domain symbols ----
    // Frame: [noise][train1][train2][sync_word][data+block_pilots][tail]
    // The leading noise symbol absorbs AGC settling, squelch opening, and
    // FM PTT transients — prevents preamble corruption on real radios.
    int total_ofdm_symbols = 1 + 2 + 1 + (int)all_data_time.size() + 1;
    int sym_len = config_.symbol_samples();
    std::vector<std::complex<float>> frame;
    frame.reserve(total_ofdm_symbols * sym_len);

    // Leading noise symbol: noise-like OFDM symbol for AGC/squelch settling.
    // Use the same pilot symbol structure (known +1 on all carriers) — the
    // RX Schmidl-Cox correlator ignores it because it's a single symbol
    // (SC needs two identical symbols). Same amplitude as data symbols.
    auto noise_sym = generate_pilot_symbol();
    frame.insert(frame.end(), noise_sym.begin(), noise_sym.end());

    // Training
    frame.insert(frame.end(), train1.begin(), train1.end());
    frame.insert(frame.end(), train2.begin(), train2.end());

    // Sync word (known BPSK pattern for false-trigger rejection)
    frame.insert(frame.end(), sync_word_sym.begin(), sync_word_sym.end());

    // Data + block pilots
    for (auto& ds : all_data_time)
        frame.insert(frame.end(), ds.begin(), ds.end());

    // Tail
    frame.insert(frame.end(), tail.begin(), tail.end());

    // ---- 13. PAPR reduction (Hilbert clipper) ----
    // Clip time-domain amplitude to 7 dB PAPR, then per-symbol bandpass
    // filter to remove OOB regrowth, iterate 4×. This reduces peaks from
    // ~16-19 dB PAPR to ~7 dB so the signal survives the FM radio's
    // deviation limiter without hard clipping.
    //
    // Unlike the old soft clipper (which corrupted pilots because it didn't
    // filter OOB regrowth), the Hilbert clipper preserves in-band signal
    // structure: pilots and data carriers remain valid at the cost of ~1 dB
    // EVM penalty (acceptable tradeoff vs 15+ dB loss from deviation clipping).
    // Skip preamble (2 training symbols) to keep channel estimate clean.
    // Preamble has inherently lower PAPR (ZC sequence ≈ constant envelope)
    // and doesn't go through the deviation limiter independently.
    int preamble_samples = 4 * sym_len;  // noise + train1 + train2 + sync_word (skip clipper)
    std::vector<std::complex<float>> data_part(frame.begin() + preamble_samples, frame.end());
    // Adaptive PAPR target based on modulation order: higher-order QAM needs
    // gentler clipping to keep EVM low enough for LDPC convergence.
    // With DFT-spread, 16QAM+ should skip clipping entirely — DFT-spread
    // already reduces PAPR to ~5-7 dB, and any residual clipping creates
    // EVM that breaks higher-order constellations.
    int max_bpc = 0;
    for (int k = 0; k < tone_map.n_data_carriers; k++)
        if (tone_map.bits_per_carrier[k] > max_bpc)
            max_bpc = tone_map.bits_per_carrier[k];
    float papr_target;
    bool skip_clip = false;
    if (config_.dft_spread && max_bpc >= 4) {
        skip_clip = true;  // DFT-spread alone is sufficient for 16QAM+
        papr_target = 99.0f;
    } else if (config_.dft_spread) {
        papr_target = 7.0f;   // BPSK/QPSK with DFT-spread
    } else {
        // No DFT-spread: gentle targets
        if (max_bpc >= 8)      papr_target = 15.0f;
        else if (max_bpc >= 6) papr_target = 13.0f;
        else if (max_bpc >= 4) papr_target = 11.0f;
        else                   papr_target = 9.0f;
    }
    int n_clipped = 0;
    float papr_before_db = 0.0f, papr_after_db = 0.0f;
    if (!skip_clip) {
        auto papr_result = ofdm_papr_clip(data_part, config_, papr_target, 4);
        std::copy(data_part.begin(), data_part.end(), frame.begin() + preamble_samples);
        n_clipped = papr_result.n_clipped_samples;
        papr_before_db = papr_result.papr_before_db;
        papr_after_db = papr_result.papr_after_db;
    }

    // ---- 13b. Pilot restoration after PAPR clipping ----
    // The PAPR clipper modifies pilot symbols along with data. Restore pilots
    // to their known values (zero pilot EVM → clean channel estimation/CPE).
    if (n_clipped > 0) {
        int sym_len = config_.symbol_samples();
        int n_data_syms = (int)all_data_time.size();
        for (int sym = 0; sym < n_data_syms; sym++) {
            int sym_start = preamble_samples + sym * sym_len;
            int body_start = sym_start + config_.cp_samples;
            if (body_start + config_.nfft > (int)frame.size()) break;

            // FFT the symbol body
            std::vector<std::complex<float>> fbuf(config_.nfft);
            std::copy(frame.begin() + body_start,
                      frame.begin() + body_start + config_.nfft, fbuf.begin());
            fft_complex(fbuf.data(), config_.nfft);

            // Restore pilot bins to known values
            bool modified = false;
            for (int i = 0; i < n_used; i++) {
                if (!is_pilot[i]) continue;
                int bin = config_.used_carrier_bins[i];
                float g = fm_tx_deemph_gain(bin, config_.nfft, config_.sample_rate,
                                       config_.fm_preemph_corner_hz,
                                       config_.fm_preemph_gain_cap);
                // Pilot value = +g (real), Hermitian mirror = conj
                std::complex<float> known_pilot(g * (float)config_.nfft, 0.0f);
                // The FFT includes the ×nfft scaling from symbol_to_time
                if (std::abs(fbuf[bin] - known_pilot) > 0.01f * std::abs(known_pilot)) {
                    fbuf[bin] = known_pilot;
                    int mirror = config_.nfft - bin;
                    if (mirror > 0 && mirror < config_.nfft)
                        fbuf[mirror] = std::conj(known_pilot);
                    modified = true;
                }
            }

            if (modified) {
                // IFFT back and regenerate CP
                ifft_complex(fbuf.data(), config_.nfft);
                std::copy(fbuf.begin(), fbuf.end(),
                          frame.begin() + body_start);
                // Regenerate CP
                std::copy(frame.begin() + body_start + config_.nfft - config_.cp_samples,
                          frame.begin() + body_start + config_.nfft,
                          frame.begin() + sym_start);
            }
        }
        IRIS_LOG("[OFDM-TX] pilot restoration: restored pilots in %d data symbols", n_data_syms);
    }

    // ---- 13c. TX bandpass filter ----
    // DISABLED — radio already bandpass-filters. Frame-level FFT causes Gibbs leakage.
    if (false) {
        int frame_len = (int)frame.size();
        if (frame_len > 0) {
            // Round up to next power of 2
            int N = 1;
            while (N < frame_len) N <<= 1;

            // Copy frame into zero-padded buffer
            std::vector<std::complex<float>> spectrum(N, {0.0f, 0.0f});
            std::copy(frame.begin(), frame.end(), spectrum.begin());

            iris::fft_complex(spectrum.data(), N);

            // Passband bin range (using padded FFT size N for bin resolution)
            float lo_hz = config_.center_hz - config_.bandwidth_hz / 2.0f;
            float hi_hz = config_.center_hz + config_.bandwidth_hz / 2.0f;
            int bin_lo = freq_to_bin(lo_hz, N, config_.sample_rate);
            int bin_hi = freq_to_bin(hi_hz, N, config_.sample_rate);
            constexpr int TAPER_BINS = 4;  // raised-cosine transition width

            static bool logged_once = false;
            if (!logged_once) {
                IRIS_LOG("[OFDM-TX] BPF: %.0f-%.0f Hz, bins %d-%d/%d, taper=%d bins",
                         lo_hz, hi_hz, bin_lo, bin_hi, N, TAPER_BINS);
                logged_once = true;
            }

            // Apply per-bin gain mask
            for (int k = 0; k < N; k++) {
                // For real-valued passband signal, energy lives at positive bins
                // [bin_lo..bin_hi] and Hermitian mirror [N-bin_hi..N-bin_lo].
                float gain = 0.0f;

                // Positive frequency range
                if (k >= bin_lo && k <= bin_hi) {
                    gain = 1.0f;
                    // Raised-cosine taper at lower edge
                    if (k < bin_lo + TAPER_BINS)
                        gain = 0.5f * (1.0f - (float)std::cos(M_PI * (k - bin_lo + 1) / (TAPER_BINS + 1)));
                    // Raised-cosine taper at upper edge
                    if (k > bin_hi - TAPER_BINS)
                        gain = std::min(gain, 0.5f * (1.0f - (float)std::cos(M_PI * (bin_hi - k + 1) / (TAPER_BINS + 1))));
                }
                // Hermitian mirror range
                else if (k >= (N - bin_hi) && k <= (N - bin_lo)) {
                    int mirror = N - k;  // corresponding positive bin
                    gain = 1.0f;
                    if (mirror < bin_lo + TAPER_BINS)
                        gain = 0.5f * (1.0f - (float)std::cos(M_PI * (mirror - bin_lo + 1) / (TAPER_BINS + 1)));
                    if (mirror > bin_hi - TAPER_BINS)
                        gain = std::min(gain, 0.5f * (1.0f - (float)std::cos(M_PI * (bin_hi - mirror + 1) / (TAPER_BINS + 1))));
                }
                spectrum[k] *= gain;
            }

            // IFFT back to time domain
            // fft_complex (forward, no 1/N) + ifft_complex (backward, 1/N) = identity
            // No additional scaling needed.
            iris::ifft_complex(spectrum.data(), N);

            for (int i = 0; i < frame_len; i++)
                frame[i] = spectrum[i];
        }
    }

    // ---- 14. Log frame stats ----
    IRIS_LOG("[OFDM-TX] frame: %d OFDM syms (%d data), %d bytes, %zu IQ samples (%.0f ms)",
             total_ofdm_symbols, (int)all_data_time.size(), (int)len,
             frame.size(), frame.size() * 1000.0f / config_.sample_rate);
    IRIS_LOG("[OFDM-TX] config: nfft=%d cp=%d sym=%d pilots=%d data=%d spacing=%d dft_spread=%d",
             config_.nfft, config_.cp_samples, sym_len,
             config_.n_pilot_carriers, config_.n_data_carriers,
             config_.pilot_carrier_spacing, config_.dft_spread ? 1 : 0);
    IRIS_LOG("[OFDM-TX] PAPR=%.1f->%.1f dB, %d clipped, NUC=%s%s",
             papr_before_db, papr_after_db,
             n_clipped, tone_map.use_nuc ? "ON" : "off",
             skip_clip ? " (clip skipped)" : "");

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
