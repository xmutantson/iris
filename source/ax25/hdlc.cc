#include "ax25/hdlc.h"
#include "ax25/crc16.h"
#include <cstring>

namespace iris {

// Emit bits for one byte, LSB first
static void emit_byte_bits(std::vector<uint8_t>& bits, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        bits.push_back((byte >> i) & 1);
    }
}

// Emit bits with bit stuffing (insert 0 after five consecutive 1s)
static void emit_stuffed(std::vector<uint8_t>& bits, uint8_t byte, int& ones) {
    for (int i = 0; i < 8; i++) {
        int b = (byte >> i) & 1;
        bits.push_back(b);
        if (b == 1) {
            ones++;
            if (ones == 5) {
                bits.push_back(0);  // stuff bit
                ones = 0;
            }
        } else {
            ones = 0;
        }
    }
}

std::vector<uint8_t> hdlc_encode(const uint8_t* frame, size_t len,
                                  int preamble_flags, int postamble_flags) {
    std::vector<uint8_t> bits;
    bits.reserve(len * 10 + (preamble_flags + postamble_flags) * 8 + 32);

    // Preamble flags (no bit stuffing)
    for (int i = 0; i < preamble_flags; i++)
        emit_byte_bits(bits, AX25_FLAG);

    // Frame data with bit stuffing
    int ones = 0;
    for (size_t i = 0; i < len; i++)
        emit_stuffed(bits, frame[i], ones);

    // CRC-16 (computed over frame data, sent LSB first with bit stuffing)
    uint16_t crc = crc16_ccitt(frame, len);
    emit_stuffed(bits, crc & 0xFF, ones);
    emit_stuffed(bits, (crc >> 8) & 0xFF, ones);

    // Postamble flags
    for (int i = 0; i < postamble_flags; i++)
        emit_byte_bits(bits, AX25_FLAG);

    // Apply NRZI encoding
    return nrzi_encode(bits);
}

// NRZI encode: 0 -> toggle, 1 -> hold
std::vector<uint8_t> nrzi_encode(const std::vector<uint8_t>& bits) {
    std::vector<uint8_t> out;
    out.reserve(bits.size());
    uint8_t prev = 0;
    for (uint8_t b : bits) {
        if (b == 0)
            prev ^= 1;  // toggle
        // else hold
        out.push_back(prev);
    }
    return out;
}

// NRZI decode: transition -> 0, same -> 1
std::vector<uint8_t> nrzi_decode(const std::vector<uint8_t>& bits) {
    std::vector<uint8_t> out;
    out.reserve(bits.size());
    if (bits.empty()) return out;
    uint8_t prev = 0;
    for (uint8_t b : bits) {
        out.push_back((b == prev) ? 1 : 0);
        prev = b;
    }
    return out;
}

// HDLC Decoder

HdlcDecoder::HdlcDecoder() { reset(); }

void HdlcDecoder::reset() {
    buffer_.clear();
    frame_.clear();
    bit_idx_ = 0;
    ones_count_ = 0;
    shift_reg_ = 0;
    in_frame_ = false;
}

bool HdlcDecoder::push_bit(int bit) {
    // Track consecutive ones for flag/abort detection
    if (bit == 1) {
        ones_count_++;
        if (ones_count_ >= 7) {
            // Abort: 7+ ones
            in_frame_ = false;
            buffer_.clear();
            bit_idx_ = 0;
            ones_count_ = 0;
            shift_reg_ = 0;
            return false;
        }
    } else {
        if (ones_count_ == 6) {
            // Flag detected (0111_1110)
            if (in_frame_ && buffer_.size() >= 2) {
                // End of frame — check CRC
                // Last 2 bytes are CRC
                size_t data_len = buffer_.size() - 2;
                uint16_t rx_crc = buffer_[data_len] | (buffer_[data_len + 1] << 8);
                uint16_t calc_crc = crc16_ccitt(buffer_.data(), data_len);
                if (rx_crc == calc_crc) {
                    frame_.assign(buffer_.begin(), buffer_.begin() + data_len);
                    buffer_.clear();
                    bit_idx_ = 0;
                    ones_count_ = 0;
                    shift_reg_ = 0;
                    in_frame_ = true;  // Stay ready for next frame
                    return true;
                }
            }
            // Start of new frame
            in_frame_ = true;
            buffer_.clear();
            bit_idx_ = 0;
            ones_count_ = 0;
            shift_reg_ = 0;
            return false;
        }
        if (ones_count_ == 5) {
            // Bit stuffing — discard this zero
            ones_count_ = 0;
            return false;
        }
        ones_count_ = 0;
    }

    if (!in_frame_) return false;

    // Accumulate data bits into bytes, LSB first
    shift_reg_ = (shift_reg_ >> 1) | (bit << 7);
    bit_idx_++;
    if (bit_idx_ == 8) {
        buffer_.push_back(shift_reg_);
        shift_reg_ = 0;
        bit_idx_ = 0;
    }

    return false;
}

} // namespace iris
