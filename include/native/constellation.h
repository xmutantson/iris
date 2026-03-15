#ifndef IRIS_CONSTELLATION_H
#define IRIS_CONSTELLATION_H

#include <cstdint>
#include <complex>
#include <vector>

namespace iris {

enum class Modulation : uint8_t {
    BPSK   = 0,   // 1 bit/symbol
    QPSK   = 1,   // 2 bits/symbol
    QAM16  = 2,   // 4 bits/symbol
    QAM64  = 3,   // 6 bits/symbol
    QAM256 = 4,   // 8 bits/symbol
};

// Bits per symbol for each modulation
int bits_per_symbol(Modulation mod);

// Map bits to complex symbol (Gray-coded)
std::complex<float> map_symbol(const uint8_t* bits, Modulation mod);

// Demap complex symbol to hard-decision bits
void demap_symbol(std::complex<float> sym, uint8_t* bits, Modulation mod);

// Map a sequence of bits to symbols
std::vector<std::complex<float>> map_bits(const std::vector<uint8_t>& bits, Modulation mod);

// Demap symbols to bits
std::vector<uint8_t> demap_bits(const std::vector<std::complex<float>>& symbols, Modulation mod);

// Soft demap: produce LLR values (positive = likely 0, negative = likely 1)
// sigma_sq: noise variance per dimension (from preamble SNR estimate).
// 0 = use fixed normalization (legacy behavior).
std::vector<float> demap_soft(const std::vector<std::complex<float>>& symbols, Modulation mod,
                              float sigma_sq = 0.0f);

} // namespace iris

#endif
