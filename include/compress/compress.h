#ifndef IRIS_COMPRESS_H
#define IRIS_COMPRESS_H

#include <vector>
#include <cstdint>
#include <cstddef>

namespace iris {

// Simple LZ77-style compression for packet data.
// Format: flag bytes + literals/matches
// Flag byte: 8 bits, 1=literal follows, 0=match (offset_hi, offset_lo|len) follows
// Match: 12-bit offset (1-4096), 4-bit length (3-18)

// Compress data. Returns compressed bytes (may be larger than input for random data).
std::vector<uint8_t> compress(const uint8_t* data, size_t len);

// Decompress data. Returns decompressed bytes, or empty on error.
std::vector<uint8_t> decompress(const uint8_t* data, size_t len);

// Compress only if it saves space; returns original with a 1-byte header otherwise.
// Header byte 0x00 = uncompressed follows, 0x01 = compressed follows.
std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len);

} // namespace iris

#endif
