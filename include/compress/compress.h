#ifndef IRIS_COMPRESS_H
#define IRIS_COMPRESS_H

#include <vector>
#include <cstdint>
#include <cstddef>

namespace iris {

// Zstd compression for packet data.
// compress/decompress: raw zstd operations.
// compress_frame/decompress_frame: adds 1-byte header (0x00=raw, 0x01=zstd)
//   and only compresses if it actually saves space.

std::vector<uint8_t> compress(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress(const uint8_t* data, size_t len);

std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len);

} // namespace iris

#endif
