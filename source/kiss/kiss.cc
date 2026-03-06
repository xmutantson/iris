#include "kiss/kiss.h"

namespace iris {

KissCodec::KissCodec() : in_frame_(false), escape_(false) {}

void KissCodec::reset() {
    buffer_.clear();
    in_frame_ = false;
    escape_ = false;
}

std::vector<uint8_t> KissCodec::encode(const uint8_t* frame, size_t len,
                                        uint8_t port, uint8_t cmd) {
    std::vector<uint8_t> out;
    out.reserve(len * 2 + 4);

    out.push_back(KISS_FEND);
    out.push_back((port << 4) | (cmd & 0x0F));

    for (size_t i = 0; i < len; i++) {
        switch (frame[i]) {
            case KISS_FEND:
                out.push_back(KISS_FESC);
                out.push_back(KISS_TFEND);
                break;
            case KISS_FESC:
                out.push_back(KISS_FESC);
                out.push_back(KISS_TFESC);
                break;
            default:
                out.push_back(frame[i]);
                break;
        }
    }

    out.push_back(KISS_FEND);
    return out;
}

void KissCodec::feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (b == KISS_FEND) {
            if (in_frame_ && buffer_.size() >= 1 && callback_) {
                uint8_t type_byte = buffer_[0];
                uint8_t port = (type_byte >> 4) & 0x0F;
                uint8_t cmd = type_byte & 0x0F;
                callback_(port, cmd, buffer_.data() + 1, buffer_.size() - 1);
            }
            buffer_.clear();
            in_frame_ = true;
            escape_ = false;
            continue;
        }

        if (!in_frame_) continue;

        if (escape_) {
            escape_ = false;
            if (b == KISS_TFEND)
                buffer_.push_back(KISS_FEND);
            else if (b == KISS_TFESC)
                buffer_.push_back(KISS_FESC);
            else
                buffer_.push_back(b);  // malformed, pass through
            continue;
        }

        if (b == KISS_FESC) {
            escape_ = true;
            continue;
        }

        buffer_.push_back(b);
    }
}

} // namespace iris
