#include "native/frame.h"
#include <cmath>
#include <cstring>

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

// Map LdpcRate to 2-bit field for header
static uint8_t fec_to_field(LdpcRate fec) {
    switch (fec) {
        case LdpcRate::NONE:     return 0;
        case LdpcRate::RATE_1_2: return 1;
        case LdpcRate::RATE_3_4: return 2;
        case LdpcRate::RATE_7_8: return 3;
    }
    return 0;
}

static LdpcRate field_to_fec(uint8_t field) {
    switch (field & 0x03) {
        case 0: return LdpcRate::NONE;
        case 1: return LdpcRate::RATE_1_2;
        case 2: return LdpcRate::RATE_3_4;
        case 3: return LdpcRate::RATE_7_8;
    }
    return LdpcRate::NONE;
}

std::vector<uint8_t> encode_header(Modulation mod, uint16_t payload_len, LdpcRate fec) {
    // 32 bits: [4 mod][12 payload_len][2 fec][6 reserved][8 crc8]
    uint8_t header_bytes[4];
    header_bytes[0] = ((uint8_t)mod << 4) | ((payload_len >> 8) & 0x0F);
    header_bytes[1] = payload_len & 0xFF;
    header_bytes[2] = (fec_to_field(fec) << 6);
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
    fec = field_to_fec(header_bytes[2] >> 6);
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
    }

    // Pad to multiple of bits_per_symbol
    int bps = bits_per_symbol(config.modulation);
    while (payload_bits.size() % bps != 0)
        payload_bits.push_back(0);

    auto payload_symbols = map_bits(payload_bits, config.modulation);

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

int detect_frame_start(const float* iq_samples, size_t count, int sps) {
    auto preamble = generate_preamble();
    auto sync = generate_sync_word();

    std::vector<std::complex<float>> ref;
    for (auto& s : preamble) ref.push_back(s);
    for (auto& s : sync) ref.push_back(s);

    size_t ref_len = ref.size();
    size_t n_samples = count / 2;

    if (n_samples < ref_len * (size_t)sps) return -1;

    float best_corr = 0;
    int best_offset = -1;

    size_t search_end = n_samples - ref_len * sps;

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
        float norm = (energy > 0) ? mag / std::sqrt(energy) : 0;

        if (norm > best_corr && norm > 0.5f) {
            best_corr = norm;
            best_offset = (int)offset;
        }
    }

    return best_offset;
}

bool decode_native_frame(const float* iq_samples, size_t count,
                          int frame_start, const PhyConfig& default_config,
                          std::vector<uint8_t>& payload) {
    int sps = default_config.samples_per_symbol;
    size_t n_samples = count / 2;

    auto rrc_taps = rrc_filter(default_config.rrc_alpha, RRC_SPAN, sps);

    std::vector<float> i_in(n_samples), q_in(n_samples);
    for (size_t i = 0; i < n_samples; i++) {
        i_in[i] = iq_samples[2 * i];
        q_in[i] = iq_samples[2 * i + 1];
    }
    auto i_filt = fir_filter(i_in, rrc_taps);
    auto q_filt = fir_filter(q_in, rrc_taps);
    size_t filt_len = std::min(i_filt.size(), q_filt.size());

    int rx_delay = RRC_SPAN * sps;
    int rx_frame_start = frame_start + rx_delay;

    int header_start = rx_frame_start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN) * sps;

    // Decode header (32 BPSK symbols)
    std::vector<uint8_t> header_bits;
    for (int i = 0; i < IRIS_HEADER_LEN; i++) {
        size_t idx = header_start + i * sps;
        if (idx >= filt_len) return false;
        float re = i_filt[idx];
        header_bits.push_back(re < 0 ? 1 : 0);
    }

    Modulation mod;
    uint16_t payload_len;
    LdpcRate fec;
    if (!decode_header(header_bits, mod, payload_len, fec))
        return false;

    if (payload_len > AX25_MAX_FRAME * 4)
        return false;

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
    int payload_symbols = padded_bits / bps;

    std::vector<std::complex<float>> symbols;
    for (int k2 = 0; k2 < payload_symbols; k2++) {
        size_t idx = payload_start + k2 * sps;
        if (idx >= filt_len) return false;
        symbols.push_back({i_filt[idx], q_filt[idx]});
    }

    auto bits = demap_bits(symbols, mod);

    // LDPC decode
    if (fec != LdpcRate::NONE) {
        if ((int)bits.size() > encoded_bits)
            bits.resize(encoded_bits);
        bits = LdpcCodec::decode(bits, fec);
        if (bits.empty())
            return false;
    }

    int total_bytes = payload_len + 4;
    if ((int)bits.size() < total_bytes * 8)
        return false;

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
    if (rx_crc != calc_crc)
        return false;

    payload.assign(decoded_bytes.begin(), decoded_bytes.begin() + payload_len);
    return true;
}

} // namespace iris
