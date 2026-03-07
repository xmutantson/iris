#include "compress/compress.h"
#include <algorithm>
#include <cstring>

namespace iris {

static constexpr int WINDOW_SIZE = 4096;   // 12-bit offset
static constexpr int MIN_MATCH = 3;
static constexpr int MAX_MATCH = 18;       // 4-bit length + 3

std::vector<uint8_t> compress(const uint8_t* data, size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len);

    size_t pos = 0;
    while (pos < len) {
        uint8_t flags = 0;
        size_t flag_pos = out.size();
        out.push_back(0);  // placeholder for flags

        for (int bit = 0; bit < 8 && pos < len; bit++) {
            // Search for best match in window
            int best_off = 0, best_len = 0;
            int search_start = (int)pos - WINDOW_SIZE;
            if (search_start < 0) search_start = 0;

            for (int s = search_start; s < (int)pos; s++) {
                int ml = 0;
                while (ml < MAX_MATCH && pos + ml < len && data[s + ml] == data[pos + ml])
                    ml++;
                if (ml > best_len) {
                    best_len = ml;
                    best_off = (int)pos - s;
                }
            }

            if (best_len >= MIN_MATCH) {
                // Match: offset (12 bits) + length-3 (4 bits) = 2 bytes
                int encoded_len = best_len - MIN_MATCH;
                out.push_back((uint8_t)(best_off >> 4));
                out.push_back((uint8_t)(((best_off & 0x0F) << 4) | encoded_len));
                pos += best_len;
                // flag bit stays 0 (match)
            } else {
                // Literal
                flags |= (1 << bit);
                out.push_back(data[pos++]);
            }
        }
        out[flag_pos] = flags;
    }

    return out;
}

std::vector<uint8_t> decompress(const uint8_t* data, size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len * 2);

    size_t pos = 0;
    while (pos < len) {
        if (pos >= len) break;
        uint8_t flags = data[pos++];

        for (int bit = 0; bit < 8 && pos < len; bit++) {
            if (flags & (1 << bit)) {
                // Literal
                out.push_back(data[pos++]);
            } else {
                // Match
                if (pos + 1 >= len) return out;  // truncated
                uint8_t b1 = data[pos++];
                uint8_t b2 = data[pos++];
                int offset = ((int)b1 << 4) | (b2 >> 4);
                int length = (b2 & 0x0F) + MIN_MATCH;

                if (offset == 0 || offset > (int)out.size()) return {};  // invalid

                int start = (int)out.size() - offset;
                for (int i = 0; i < length; i++)
                    out.push_back(out[start + i]);
            }
        }
    }

    return out;
}

std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len) {
    auto compressed = compress(data, len);
    if (compressed.size() < len) {
        std::vector<uint8_t> out;
        out.reserve(1 + compressed.size());
        out.push_back(0x01);  // compressed
        out.insert(out.end(), compressed.begin(), compressed.end());
        return out;
    }
    // Not worth compressing
    std::vector<uint8_t> out;
    out.reserve(1 + len);
    out.push_back(0x00);  // uncompressed
    out.insert(out.end(), data, data + len);
    return out;
}

std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len) {
    if (len < 1) return {};
    if (data[0] == 0x00) {
        // Uncompressed
        return std::vector<uint8_t>(data + 1, data + len);
    } else if (data[0] == 0x01) {
        // Compressed
        return decompress(data + 1, len - 1);
    }
    return {};  // unknown format
}

} // namespace iris
