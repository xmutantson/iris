#ifndef IRIS_LDPC_H
#define IRIS_LDPC_H

#include <vector>
#include <cstdint>
#include <cstddef>

namespace iris {

// LDPC code rates supported by Iris
enum class LdpcRate {
    NONE,      // No FEC (passthrough)
    RATE_1_2,  // 1/2 code rate
    RATE_3_4,  // 3/4 code rate
    RATE_7_8,  // 7/8 code rate
};

// Map FEC rate fraction to LdpcRate enum
LdpcRate fec_to_ldpc_rate(int num, int den);

// LDPC encoder/decoder
// Uses systematic encoding: output = [data | parity]
class LdpcCodec {
public:
    LdpcCodec() = default;

    // Encode data bits. Returns codeword (data + parity bits).
    // If data size isn't a multiple of block size, it's padded with zeros.
    static std::vector<uint8_t> encode(const std::vector<uint8_t>& data_bits, LdpcRate rate);

    // Decode received bits (hard decision). Returns decoded data bits.
    // Returns empty vector on decode failure.
    static std::vector<uint8_t> decode(const std::vector<uint8_t>& codeword, LdpcRate rate);

    // Get code parameters
    static int block_size(LdpcRate rate);      // Data bits per block
    static int codeword_size(LdpcRate rate);   // Total codeword bits
    static int parity_bits(LdpcRate rate);     // Parity bits per block

    // Compute code rate as float
    static float code_rate(LdpcRate rate);
};

} // namespace iris

#endif
