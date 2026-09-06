#include "native/constellation.h"
#include "native/nuc_tables.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>

namespace iris {

int bits_per_symbol(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:   return 1;
        case Modulation::QPSK:   return 2;
        case Modulation::QAM16:  return 4;
        case Modulation::QAM32:  return 5;
        case Modulation::QAM64:  return 6;
        case Modulation::QAM256:  return 8;
        case Modulation::QAM1024: return 10;
    }
    IRIS_LOG("[CONSTELLATION] Invalid modulation: %d", (int)mod);
    return 0;
}

// ============================================================================
//  32-QAM cross constellation (VARA FM narrow top gear = 32QAM r5/8).
//
//  Points: the 6x6 grid at odd coords {-5,-3,-1,1,3,5} with the four corners
//  (|I|=5 AND |Q|=5) removed → 32 points, normalized to unit average power
//  (raw avg energy 20 → divide by sqrt(20)=4.47213595). Values 0.2236/0.6708/
//  1.1180 correspond to grid ±1/±3/±5.
//
//  Bit labeling: index the table by the 5-bit symbol value directly, so
//  CROSS32[v] IS the Gray label of value v. The mapping was optimized offline
//  (simulated annealing minimizing total Hamming distance over the 52
//  nearest-neighbor edges → 56 bits total, 1.077 bits/edge, near the cross-32
//  optimum; no perfect Gray code exists for the cross). Correctness of decode
//  does NOT depend on the labeling — the RX uses a full max-log-MAP search over
//  all 32 points (see demap_soft below); the Gray labeling only tightens BER.
//  Bit b of value v is the LSB-first bit (v>>b)&1, matching map_symbol()'s
//  packing and demap_soft_nuc()'s convention.
// ============================================================================
static const float CROSS32_I[32] = {-0.22360680f, 0.22360680f, -0.22360680f, 0.22360680f, -0.22360680f, 0.22360680f, -0.22360680f, 0.22360680f, -0.67082039f, 0.67082039f, -0.67082039f, 0.67082039f, -0.67082039f, 0.67082039f, -0.67082039f, 0.67082039f, -0.22360680f, 0.22360680f, -0.67082039f, 0.67082039f, -0.22360680f, 0.22360680f, -0.67082039f, 0.67082039f, -1.11803399f, 1.11803399f, -1.11803399f, 1.11803399f, -1.11803399f, 1.11803399f, -1.11803399f, 1.11803399f};
static const float CROSS32_Q[32] = {-0.22360680f, -0.22360680f, 1.11803399f, 1.11803399f, 0.22360680f, 0.22360680f, 0.67082039f, 0.67082039f, -0.22360680f, -0.22360680f, 1.11803399f, 1.11803399f, 0.22360680f, 0.22360680f, 0.67082039f, 0.67082039f, -0.67082039f, -0.67082039f, -0.67082039f, -0.67082039f, -1.11803399f, -1.11803399f, -1.11803399f, -1.11803399f, -0.22360680f, -0.22360680f, -0.67082039f, -0.67082039f, 0.22360680f, 0.22360680f, 0.67082039f, 0.67082039f};

// Hard-decision nearest-point index for cross-32 (full search, 32 points).
static int cross32_nearest(std::complex<float> sym) {
    int best = 0;
    float best_d = 1e30f;
    for (int v = 0; v < 32; v++) {
        float di = sym.real() - CROSS32_I[v];
        float dq = sym.imag() - CROSS32_Q[v];
        float d = di * di + dq * dq;
        if (d < best_d) { best_d = d; best = v; }
    }
    return best;
}

// Gray code helpers
static int gray_encode(int n) { return n ^ (n >> 1); }

static int gray_decode(int g) {
    int n = 0;
    for (; g; g >>= 1) n ^= g;
    return n;
}

// Generic square QAM mapper
// M-QAM: sqrt(M) points per axis, Gray-coded per axis
static std::complex<float> qam_map(int bits_val, int order) {
    int side = (int)std::sqrt((float)order);
    int mask = side - 1;

    int i_gray = (bits_val >> (int)(std::log2(side))) & mask;
    int q_gray = bits_val & mask;

    int i_idx = gray_decode(i_gray);
    int q_idx = gray_decode(q_gray);

    // Map to symmetric constellation: -(side-1), -(side-3), ..., (side-3), (side-1)
    float i_val = 2.0f * i_idx - (side - 1);
    float q_val = 2.0f * q_idx - (side - 1);

    // Normalize average power to 1
    float norm = 0;
    for (int k = 0; k < side; k++) {
        float v = 2.0f * k - (side - 1);
        norm += v * v;
    }
    norm = std::sqrt(2.0f * norm / side);

    return std::complex<float>(i_val / norm, q_val / norm);
}

static int qam_demap(std::complex<float> sym, int order) {
    int side = (int)std::sqrt((float)order);

    // Compute normalization factor
    float norm = 0;
    for (int k = 0; k < side; k++) {
        float v = 2.0f * k - (side - 1);
        norm += v * v;
    }
    norm = std::sqrt(2.0f * norm / side);

    // Denormalize
    float i_val = sym.real() * norm;
    float q_val = sym.imag() * norm;

    // Find nearest constellation point index
    int i_idx = std::isnan(i_val) ? 0 :
        (int)std::round(std::clamp((i_val + (side - 1)) / 2.0f,
                                  0.0f, (float)(side - 1)));
    int q_idx = std::isnan(q_val) ? 0 :
        (int)std::round(std::clamp((q_val + (side - 1)) / 2.0f,
                                  0.0f, (float)(side - 1)));

    int i_gray = gray_encode(i_idx);
    int q_gray = gray_encode(q_idx);

    return (i_gray << (int)(std::log2(side))) | q_gray;
}

std::complex<float> map_symbol(const uint8_t* bits, Modulation mod) {
    int bps = bits_per_symbol(mod);
    int val = 0;
    for (int i = 0; i < bps; i++)
        val |= (bits[i] & 1) << i;

    switch (mod) {
        case Modulation::BPSK:
            return std::complex<float>(val ? -1.0f : 1.0f, 0.0f);
        case Modulation::QPSK: {
            // Gray-coded QPSK: 00->+1+j, 01->-1+j, 11->-1-j, 10->+1-j
            float norm = 1.0f / std::sqrt(2.0f);
            int i_bit = bits[0];
            int q_bit = bits[1];
            float i_val = i_bit ? -norm : norm;
            float q_val = q_bit ? -norm : norm;
            return std::complex<float>(i_val, q_val);
        }
        case Modulation::QAM16:   return qam_map(val, 16);
        case Modulation::QAM32:   return std::complex<float>(CROSS32_I[val & 31], CROSS32_Q[val & 31]);
        case Modulation::QAM64:   return qam_map(val, 64);
        case Modulation::QAM256:  return qam_map(val, 256);
        case Modulation::QAM1024: return qam_map(val, 1024);
    }
    return {0.0f, 0.0f};
}

void demap_symbol(std::complex<float> sym, uint8_t* bits, Modulation mod) {
    int bps = bits_per_symbol(mod);
    int val = 0;

    switch (mod) {
        case Modulation::BPSK:
            val = sym.real() < 0 ? 1 : 0;
            break;
        case Modulation::QPSK:
            bits[0] = sym.real() < 0 ? 1 : 0;
            bits[1] = sym.imag() < 0 ? 1 : 0;
            return;
        case Modulation::QAM16:   val = qam_demap(sym, 16);   break;
        case Modulation::QAM32:   val = cross32_nearest(sym); break;
        case Modulation::QAM64:   val = qam_demap(sym, 64);   break;
        case Modulation::QAM256:  val = qam_demap(sym, 256);  break;
        case Modulation::QAM1024: val = qam_demap(sym, 1024); break;
    }

    for (int i = 0; i < bps; i++)
        bits[i] = (val >> i) & 1;
}

std::vector<std::complex<float>> map_bits(const std::vector<uint8_t>& bits, Modulation mod) {
    int bps = bits_per_symbol(mod);
    if (bps == 0) return {};
    size_t nsym = (bits.size() + bps - 1) / bps;
    std::vector<std::complex<float>> symbols;
    symbols.reserve(nsym);
    std::vector<uint8_t> padded_bits(bits);
    padded_bits.resize(nsym * bps, 0);

    for (size_t i = 0; i < nsym; i++) {
        symbols.push_back(map_symbol(&padded_bits[i * bps], mod));
    }
    return symbols;
}

std::vector<uint8_t> demap_bits(const std::vector<std::complex<float>>& symbols, Modulation mod) {
    int bps = bits_per_symbol(mod);
    std::vector<uint8_t> bits(symbols.size() * bps);

    for (size_t i = 0; i < symbols.size(); i++) {
        demap_symbol(symbols[i], &bits[i * bps], mod);
    }
    return bits;
}

// Soft demap: produce LLR values (positive = likely 0, negative = likely 1)
// sigma_sq: noise variance per dimension. When provided, LLR = (d1-d0)/sigma_sq
// which is the correct max-log-MAP expression. Without it, a fixed normalization
// is used which produces over-confident LLRs at low SNR.
std::vector<float> demap_soft(const std::vector<std::complex<float>>& symbols, Modulation mod,
                              float sigma_sq) {
    int bps = bits_per_symbol(mod);
    if (sigma_sq < 0.0f || !std::isfinite(sigma_sq)) {
        IRIS_LOG("[CONSTELLATION] Invalid noise variance: %g", (double)sigma_sq);
        return {};
    }

    std::vector<float> llrs;
    llrs.reserve(symbols.size() * bps);

    for (const auto& sym : symbols) {
        // LLR scaling: if sigma_sq is provided, LLR = (d1-d0)/sigma_sq
        // which is the correct max-log-MAP. Without sigma_sq, use fixed
        // normalization (works at design SNR but over-confident at low SNR).
        switch (mod) {
            case Modulation::BPSK: {
                // bit 0: +1 = 0, -1 = 1. LLR = 2*re/sigma_sq
                float scale = (sigma_sq > 0) ? (2.0f / sigma_sq) : 2.0f;
                llrs.push_back(scale * sym.real());
                break;
            }

            case Modulation::QPSK: {
                // bit 0 from I, bit 1 from Q. Distance between ±1/√2 = √2.
                // Exact LLR = 2 * coord * √2 / sigma_sq
                float norm = std::sqrt(2.0f);
                float scale = (sigma_sq > 0) ? (2.0f * norm / sigma_sq) : (2.0f * norm);
                llrs.push_back(scale * sym.real());
                llrs.push_back(scale * sym.imag());
                break;
            }

            case Modulation::QAM32: {
                // Cross-32: non-separable, so full max-log-MAP over all 32
                // points (identical to demap_soft_nuc's 2D path). The table is
                // unit-power, so distances are on the unit scale and the
                // denominator is sigma_sq directly (NOT scaled by a grid-norm
                // like the square-QAM path below). Bits pushed LSB-first.
                float denom = (sigma_sq > 0) ? sigma_sq : 1.0f;
                for (int b = 0; b < 5; b++) {
                    float min_d0 = 1e30f, min_d1 = 1e30f;
                    for (int v = 0; v < 32; v++) {
                        float di = sym.real() - CROSS32_I[v];
                        float dq = sym.imag() - CROSS32_Q[v];
                        float d = di * di + dq * dq;
                        if ((v >> b) & 1) min_d1 = std::min(min_d1, d);
                        else              min_d0 = std::min(min_d0, d);
                    }
                    llrs.push_back((min_d1 - min_d0) / denom);
                }
                break;
            }

            case Modulation::QAM16:
            case Modulation::QAM64:
            case Modulation::QAM256:
            case Modulation::QAM1024: {
                int order = 1 << bps;
                int side = (int)std::sqrt((float)order);
                int nbits_axis = bps / 2;

                // Compute normalization
                float norm = 0;
                for (int k = 0; k < side; k++) {
                    float v = 2.0f * k - (side - 1);
                    norm += v * v;
                }
                norm = std::sqrt(2.0f * norm / side);

                // Denormalize to integer grid spacing
                float i_val = sym.real() * norm;
                float q_val = sym.imag() * norm;

                // LLR denominator: twice sigma_sq scaled to integer grid, or fixed norm
                // On the integer grid, twice the noise variance = 2 * sigma_sq * norm^2
                float denom = (sigma_sq > 0) ? (2.0f * sigma_sq * norm * norm) : (norm * norm / (float)order);

                // For each axis, compute approximate LLR for each Gray-coded bit
                // by finding min distance to constellation points with bit=0 vs bit=1
                // Q-axis first (low bits), then I-axis (high bits)
                // to match bit packing: val = i_gray << nbits_axis | q_gray
                float axes[2] = {q_val, i_val};
                for (int ax = 0; ax < 2; ax++) {
                    float v = axes[ax];
                    for (int b = 0; b < nbits_axis; b++) {
                        float min_d0 = 1e9f, min_d1 = 1e9f;
                        for (int idx = 0; idx < side; idx++) {
                            float point = 2.0f * idx - (side - 1);
                            float dist_sq = (v - point) * (v - point);
                            int gray = gray_encode(idx);
                            if ((gray >> b) & 1)
                                min_d1 = std::min(min_d1, dist_sq);
                            else
                                min_d0 = std::min(min_d0, dist_sq);
                        }
                        // LLR = (min_d1 - min_d0) / denom — positive means closer to 0
                        float llr = (min_d1 - min_d0) / denom;
                        llrs.push_back(llr);
                    }
                }
                break;
            }
        }
    }

    return llrs;
}

// ============================================================================
//  NUC table lookup
// ============================================================================

// Static NucTable descriptors — one per (order, rate) pair
static const NucTable nuc_table_16_r8  = { 16,  8, false, nuc16_r8_points,  16, nullptr, 0 };
static const NucTable nuc_table_16_r12 = { 16, 12, false, nuc16_r12_points, 16, nullptr, 0 };
static const NucTable nuc_table_16_r14 = { 16, 14, false, nuc16_r14_points, 16, nullptr, 0 };
static const NucTable nuc_table_64_r8  = { 64,  8, false, nuc64_r8_points,  64, nullptr, 0 };
static const NucTable nuc_table_64_r12 = { 64, 12, false, nuc64_r12_points, 64, nullptr, 0 };
static const NucTable nuc_table_64_r14 = { 64, 14, false, nuc64_r14_points, 64, nullptr, 0 };
static const NucTable nuc_table_256_r12 = { 256, 12, true, nullptr, 0, nuc256_r12_axis, 16 };
static const NucTable nuc_table_256_r14 = { 256, 14, true, nullptr, 0, nuc256_r14_axis, 16 };

const NucTable* get_nuc_table(Modulation mod, int fec_rate_num_16) {
    switch (mod) {
        case Modulation::QAM16:
            if (fec_rate_num_16 == 8)  return &nuc_table_16_r8;
            if (fec_rate_num_16 == 12) return &nuc_table_16_r12;
            if (fec_rate_num_16 == 14) return &nuc_table_16_r14;
            break;
        case Modulation::QAM64:
            if (fec_rate_num_16 == 8)  return &nuc_table_64_r8;
            if (fec_rate_num_16 == 12) return &nuc_table_64_r12;
            if (fec_rate_num_16 == 14) return &nuc_table_64_r14;
            break;
        case Modulation::QAM256:
            if (fec_rate_num_16 == 12) return &nuc_table_256_r12;
            if (fec_rate_num_16 == 14) return &nuc_table_256_r14;
            break;
        default:
            break;
    }
    return nullptr;
}

// ============================================================================
//  NUC mapper
// ============================================================================

static float nuc_axis_norm(const NucTable* nuc) {
    float power = 0.0f;
    for (int i = 0; i < nuc->n_axis_1d; i++)
        power += nuc->axis_1d[i] * nuc->axis_1d[i];
    return std::sqrt(2.0f * power / nuc->n_axis_1d);
}

std::complex<float> map_symbol_nuc(int symbol_index, const NucTable* nuc) {
    if (!nuc) return {0.0f, 0.0f};

    if (!nuc->separable) {
        // 2D-NUC: direct lookup
        if (symbol_index < 0 || symbol_index >= nuc->n_points_2d)
            return {0.0f, 0.0f};
        return nuc->points_2d[symbol_index];
    } else {
        // 1D-NUC (256QAM): Cartesian product of axis values
        // symbol_index = i_idx * n_axis + q_idx (upper bits = I, lower bits = Q)
        int side = nuc->n_axis_1d;
        if (symbol_index < 0 || symbol_index >= side * side)
            return {0.0f, 0.0f};
        int i_idx = (symbol_index >> (side > 0 ? (int)std::log2((float)side) : 0)) & (side - 1);
        int q_idx = symbol_index & (side - 1);
        float norm = nuc_axis_norm(nuc);
        return {nuc->axis_1d[i_idx] / norm, nuc->axis_1d[q_idx] / norm};
    }
}

// ============================================================================
//  NUC soft demapper
// ============================================================================

void demap_soft_nuc(const std::complex<float>& sym, float sigma_sq,
                    const NucTable* nuc, float* llrs) {
    if (!nuc) return;

    int nbits = (int)std::log2((float)nuc->order);

    if (!nuc->separable) {
        // ------------------------------------------------------------------
        //  2D-NUC (16-NUC, 64-NUC): full search over all constellation points
        //  max-log-MAP: LLR(b) = min_{s: b=0} |y-s|^2 - min_{s: b=1} |y-s|^2
        //               all divided by sigma_sq
        // ------------------------------------------------------------------
        float denom = (sigma_sq > 0) ? sigma_sq : 1.0f;

        for (int b = 0; b < nbits; b++) {
            float min_d0 = 1e30f;
            float min_d1 = 1e30f;

            for (int s = 0; s < nuc->n_points_2d; s++) {
                float di = sym.real() - nuc->points_2d[s].real();
                float dq = sym.imag() - nuc->points_2d[s].imag();
                float dist_sq = di * di + dq * dq;

                if ((s >> b) & 1)
                    min_d1 = std::min(min_d1, dist_sq);
                else
                    min_d0 = std::min(min_d0, dist_sq);
            }

            // LLR > 0 means bit more likely 0
            llrs[b] = (min_d1 - min_d0) / denom;
        }
    } else {
        // ------------------------------------------------------------------
        //  1D-NUC (256-NUC): separable I/Q axes
        //  Compute per-axis LLRs independently — same complexity as uniform QAM
        // ------------------------------------------------------------------
        int side = nuc->n_axis_1d;          // 16 for 256QAM
        int nbits_axis = nbits / 2;        // 4 bits per axis
        float denom = (sigma_sq > 0) ? sigma_sq : 1.0f;

        float norm = nuc_axis_norm(nuc);
        // I-axis LLRs (upper bits), then Q-axis LLRs (lower bits)
        float axes[2] = {sym.real(), sym.imag()};
        for (int ax = 0; ax < 2; ax++) {
            float v = axes[ax];
            for (int b = 0; b < nbits_axis; b++) {
                float min_d0 = 1e30f;
                float min_d1 = 1e30f;

                for (int idx = 0; idx < side; idx++) {
                    float point = nuc->axis_1d[idx] / norm;
                    float dist_sq = (v - point) * (v - point);

                    if ((idx >> b) & 1)
                        min_d1 = std::min(min_d1, dist_sq);
                    else
                        min_d0 = std::min(min_d0, dist_sq);
                }

                // I-axis bits are the upper half, Q-axis bits are the lower half
                int bit_index = (ax == 0) ? (nbits_axis + b) : b;
                llrs[bit_index] = (min_d1 - min_d0) / denom;
            }
        }
    }
}

} // namespace iris
