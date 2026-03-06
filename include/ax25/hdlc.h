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

} // namespace iris

#endif
