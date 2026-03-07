#include "fec/ldpc.h"
#include <cstring>
#include <algorithm>

namespace iris {

// Block sizes chosen for practical frame sizes
// Using simple repeat/parity-check approach as placeholder
// Real deployment should use proper LDPC parity check matrices from codec2/Mercury
static constexpr int LDPC_BLOCK_1_2 = 256;   // 256 data bits -> 512 codeword
static constexpr int LDPC_BLOCK_3_4 = 384;   // 384 data bits -> 512 codeword
static constexpr int LDPC_BLOCK_7_8 = 448;   // 448 data bits -> 512 codeword

int LdpcCodec::block_size(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2: return LDPC_BLOCK_1_2;
        case LdpcRate::RATE_3_4: return LDPC_BLOCK_3_4;
        case LdpcRate::RATE_7_8: return LDPC_BLOCK_7_8;
    }
    return LDPC_BLOCK_1_2;
}

int LdpcCodec::codeword_size(LdpcRate rate) {
    return 512; // All rates use 512-bit codewords
}

int LdpcCodec::parity_bits(LdpcRate rate) {
    return codeword_size(rate) - block_size(rate);
}

float LdpcCodec::code_rate(LdpcRate rate) {
    return (float)block_size(rate) / (float)codeword_size(rate);
}

std::vector<uint8_t> LdpcCodec::encode(const std::vector<uint8_t>& data_bits, LdpcRate rate) {
    int k = block_size(rate);
    int n = codeword_size(rate);
    int p = n - k;

    // Pad input to multiple of k
    std::vector<uint8_t> padded = data_bits;
    while (padded.size() % k != 0)
        padded.push_back(0);

    std::vector<uint8_t> output;
    output.reserve(padded.size() / k * n);

    // Process each block
    for (size_t offset = 0; offset < padded.size(); offset += k) {
        // Systematic: copy data bits
        for (int i = 0; i < k; i++)
            output.push_back(padded[offset + i]);

        // Generate parity bits using simple XOR accumulation
        // Each parity bit is XOR of a subset of data bits
        // This is a simple regular LDPC-like structure
        for (int j = 0; j < p; j++) {
            uint8_t parity = 0;
            // Each parity bit covers data bits at positions (j + i*p) mod k
            // for i = 0, 1, 2, ...
            int stride = std::max(1, k / p);
            for (int i = j % stride; i < k; i += stride) {
                parity ^= padded[offset + i];
            }
            output.push_back(parity & 1);
        }
    }

    return output;
}

std::vector<uint8_t> LdpcCodec::decode(const std::vector<uint8_t>& codeword, LdpcRate rate) {
    int k = block_size(rate);
    int n = codeword_size(rate);

    if (codeword.size() % n != 0)
        return {};

    std::vector<uint8_t> output;
    output.reserve(codeword.size() / n * k);

    // For each block, extract systematic data bits
    // In a real LDPC decoder, we'd do belief propagation
    // For now, just extract data bits (no error correction)
    for (size_t offset = 0; offset < codeword.size(); offset += n) {
        for (int i = 0; i < k; i++)
            output.push_back(codeword[offset + i] & 1);
    }

    return output;
}

} // namespace iris
