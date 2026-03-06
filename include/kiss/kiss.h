#ifndef IRIS_KISS_H
#define IRIS_KISS_H

#include "common/types.h"
#include <vector>
#include <cstdint>
#include <functional>

namespace iris {

// KISS frame encoder/decoder
// Wraps AX.25 frames in KISS framing for transport over TCP/serial

class KissCodec {
public:
    KissCodec();

    // Encode an AX.25 frame into a KISS frame (FEND + cmd + escaped data + FEND)
    static std::vector<uint8_t> encode(const uint8_t* frame, size_t len,
                                        uint8_t port = 0, uint8_t cmd = KISS_CMD_DATA);

    // Feed raw bytes from transport. Calls callback for each complete KISS frame.
    // Callback receives: port, command, payload data, payload length
    using FrameCallback = std::function<void(uint8_t port, uint8_t cmd,
                                              const uint8_t* data, size_t len)>;
    void set_callback(FrameCallback cb) { callback_ = cb; }

    void feed(const uint8_t* data, size_t len);
    void reset();

private:
    std::vector<uint8_t> buffer_;
    bool in_frame_;
    bool escape_;
    FrameCallback callback_;
};

} // namespace iris

#endif
