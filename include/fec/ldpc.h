#ifndef IRIS_LDPC_H
#define IRIS_LDPC_H

#include <vector>
#include <cstdint>
#include <cstddef>
#include <atomic>

namespace iris {

// LDPC code rates supported by Iris
// Rates 1/16 through 6/16 are weak-signal modes (heavy redundancy).
// Rates 8/16 through 14/16 are standard modes.
enum class LdpcRate {
    NONE,        // No FEC (passthrough)
    RATE_1_16,   // 1/16 (k=100, p=1500)  — weakest signal
    RATE_2_16,   // 2/16 (k=200, p=1400)
    RATE_3_16,   // 3/16 (k=300, p=1300)
    RATE_4_16,   // 4/16 (k=400, p=1200)
    RATE_5_16,   // 5/16 (k=500, p=1100)
    RATE_6_16,   // 6/16 (k=600, p=1000)
    RATE_1_2,    // 8/16 (k=800, p=800)
    RATE_5_8,    // 10/16 (k=1000, p=600)
    RATE_3_4,    // 12/16 (k=1200, p=400)
    RATE_7_8,    // 14/16 (k=1400, p=200)
};

// Decoder algorithm selection
enum class LdpcDecoder {
    MIN_SUM,     // Min-sum belief propagation (balanced, default)
    SPA,         // Sum-Product Algorithm (highest accuracy, most compute)
    GBF,         // Gradient Bit-Flipping (fastest, hard-decision)
};

// Map FEC rate fraction to LdpcRate enum
LdpcRate fec_to_ldpc_rate(int num, int den);

// LDPC encoder/decoder
// Uses systematic encoding: output = [data | parity]
// Block size: 1600 bits for all rates (N_MAX)
class LdpcCodec {
public:
    LdpcCodec() = default;

    // Encode data bits. Returns codeword (data + parity bits).
    static std::vector<uint8_t> encode(const std::vector<uint8_t>& data_bits, LdpcRate rate);

    // Decode received bits (hard decision). Returns decoded data bits.
    static std::vector<uint8_t> decode(const std::vector<uint8_t>& codeword, LdpcRate rate,
                                        LdpcDecoder algo = LdpcDecoder::MIN_SUM,
                                        int max_iter = 50,
                                        std::atomic<bool>* abort_flag = nullptr);

    // Soft-decision decode. LLRs: positive = likely 0, negative = likely 1.
    static std::vector<uint8_t> decode_soft(const std::vector<float>& llrs, LdpcRate rate,
                                             LdpcDecoder algo = LdpcDecoder::MIN_SUM,
                                             int max_iter = 50,
                                             std::atomic<bool>* abort_flag = nullptr);

    // Get code parameters
    static int block_size(LdpcRate rate);      // Data bits per block (K)
    static int codeword_size(LdpcRate rate);   // Total codeword bits (N=1600)
    static int parity_bits(LdpcRate rate);     // Parity bits per block (P)
    static float code_rate(LdpcRate rate);
};

} // namespace iris

#endif
