#include "compress/compress.h"

#define ZSTD_STATIC_LINKING_ONLY
extern "C" {
#include "zstd.h"
}

#include <algorithm>

namespace iris {

// Use low compression level for speed on small packets
static constexpr int ZSTD_LEVEL = 1;

std::vector<uint8_t> compress(const uint8_t* data, size_t len) {
    if (len == 0) return {};
    size_t bound = ZSTD_compressBound(len);
    std::vector<uint8_t> out(bound);
    size_t result = ZSTD_compress(out.data(), bound, data, len, ZSTD_LEVEL);
    if (ZSTD_isError(result)) return {};
    out.resize(result);
    return out;
}

std::vector<uint8_t> decompress(const uint8_t* data, size_t len) {
    if (len == 0) return {};

    // Get decompressed size from frame header
    unsigned long long decom_size = ZSTD_getFrameContentSize(data, len);
    if (decom_size == ZSTD_CONTENTSIZE_ERROR ||
        decom_size == ZSTD_CONTENTSIZE_UNKNOWN) {
        // Fallback: try with a reasonable upper bound
        decom_size = len * 16;
        if (decom_size > 1024 * 1024) decom_size = 1024 * 1024;  // 1MB cap
    }

    std::vector<uint8_t> out((size_t)decom_size);
    size_t result = ZSTD_decompress(out.data(), out.size(), data, len);
    if (ZSTD_isError(result)) return {};
    out.resize(result);
    return out;
}

std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len) {
    if (len == 0) {
        return {0x00};  // empty uncompressed
    }
    auto compressed = compress(data, len);
    if (!compressed.empty() && compressed.size() < len) {
        std::vector<uint8_t> out;
        out.reserve(1 + compressed.size());
        out.push_back(0x01);  // zstd compressed
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
        return std::vector<uint8_t>(data + 1, data + len);
    } else if (data[0] == 0x01) {
        return decompress(data + 1, len - 1);
    }
    return {};
}

} // namespace iris
