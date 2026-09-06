#ifndef IRIS_HDLC_H
#define IRIS_HDLC_H

#include "common/types.h"
#include <vector>
#include <cstdint>

namespace iris {

// Encode an AX.25 frame into HDLC bit stream with flags, bit stuffing, NRZI, CRC
// Input: raw AX.25 frame bytes (address + control + PID + info)
// Output: NRZI-encoded bit stream (each element is 0 or 1)
// preamble_flags: number of 0x7E flags before frame
// postamble_flags: number of 0x7E flags after frame
std::vector<uint8_t> hdlc_encode(const uint8_t* frame, size_t len,
                                  int preamble_flags = 4, int postamble_flags = 2);

// Encode HDLC frame to raw bits (pre-NRZI). Append to existing bits vector.
// Use with nrzi_encode() for batching multiple frames into one continuous stream.
void hdlc_encode_raw(std::vector<uint8_t>& bits, const uint8_t* frame, size_t len,
                     int preamble_flags = 4, int postamble_flags = 2);

// HDLC frame receiver — processes one bit at a time (already NRZI-decoded)
class HdlcDecoder {
public:
    HdlcDecoder();

    // Feed one data bit (0 or 1). Returns true if a complete frame is ready.
    bool push_bit(int bit);

    // Get the last decoded frame (valid after push_bit returns true)
    const std::vector<uint8_t>& frame() const { return frame_; }

    void reset();

private:
    std::vector<uint8_t> buffer_;
    std::vector<uint8_t> frame_;
    int bit_idx_;
    int ones_count_;
    uint8_t shift_reg_;
    bool in_frame_;
};

// NRZI encode: toggle on 0, hold on 1
std::vector<uint8_t> nrzi_encode(const std::vector<uint8_t>& bits);

// NRZI decode: output 0 on transition, 1 on same
std::vector<uint8_t> nrzi_decode(const std::vector<uint8_t>& bits);

// Stateful NRZI decoder for streaming use
class NrziDecoder {
public:
    NrziDecoder() : prev_(0) {}
    void reset() { prev_ = 0; }

    std::vector<uint8_t> decode(const std::vector<uint8_t>& bits) {
        std::vector<uint8_t> out;
        out.reserve(bits.size());
        for (uint8_t b : bits) {
            out.push_back((b == prev_) ? 1 : 0);
            prev_ = b;
        }
        return out;
    }

private:
    uint8_t prev_;
};

} // namespace iris

#endif
