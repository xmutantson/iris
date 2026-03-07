#include "fec/ldpc.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

namespace iris {

// All rates use 512-bit codewords, systematic: [data | parity]
static constexpr int LDPC_N = 512;
static constexpr int LDPC_BLOCK_1_2 = 256;   // k=256, p=256
static constexpr int LDPC_BLOCK_3_4 = 384;   // k=384, p=128
static constexpr int LDPC_BLOCK_7_8 = 448;   // k=448, p=64

// H matrix: p rows (check nodes) x n columns (variable nodes)
// Stored as adjacency list per check node.
// Parity bits (columns k..n-1) are assigned so that each parity bit
// appears in exactly one check equation as the "pivot" for encoding.
// Check j's pivot is parity bit j (column k+j).
struct LdpcMatrix {
    int n, k, p;
    std::vector<std::vector<int>> check_nodes; // check_nodes[j] = columns in check j

    LdpcMatrix() : n(0), k(0), p(0) {}

    LdpcMatrix(int n_, int k_) : n(n_), k(k_), p(n_ - k_) {
        build_h();
    }

    void build_h() {
        check_nodes.resize(p);

        // Each check node j connects to:
        //   - Its pivot parity bit at column k+j (for encoding)
        //   - A subset of data bits (columns 0..k-1)
        //
        // We use a regular structure: each data bit participates in wc checks.
        // Column weight for data bits: wc=3 (high rate: wc=2)
        int wc = (p >= 128) ? 3 : 2;

        // First, assign data columns to checks
        for (int j = 0; j < p; j++) {
            check_nodes[j].clear();
            // Always include the pivot parity bit
            check_nodes[j].push_back(k + j);
        }

        // Distribute data bits across checks using wc layers
        for (int layer = 0; layer < wc; layer++) {
            for (int col = 0; col < k; col++) {
                // Map data column to a check node
                int check = (col + layer * (k / std::max(1, wc))) % p;
                check_nodes[check].push_back(col);
            }
        }

        // Sort and deduplicate each check's variable list
        for (int j = 0; j < p; j++) {
            std::sort(check_nodes[j].begin(), check_nodes[j].end());
            check_nodes[j].erase(
                std::unique(check_nodes[j].begin(), check_nodes[j].end()),
                check_nodes[j].end());
        }
    }
};

static LdpcMatrix& get_matrix(LdpcRate rate) {
    static LdpcMatrix m12(LDPC_N, LDPC_BLOCK_1_2);
    static LdpcMatrix m34(LDPC_N, LDPC_BLOCK_3_4);
    static LdpcMatrix m78(LDPC_N, LDPC_BLOCK_7_8);
    switch (rate) {
        case LdpcRate::RATE_3_4: return m34;
        case LdpcRate::RATE_7_8: return m78;
        default: return m12;
    }
}

int LdpcCodec::block_size(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2: return LDPC_BLOCK_1_2;
        case LdpcRate::RATE_3_4: return LDPC_BLOCK_3_4;
        case LdpcRate::RATE_7_8: return LDPC_BLOCK_7_8;
    }
    return LDPC_BLOCK_1_2;
}

int LdpcCodec::codeword_size(LdpcRate) {
    return LDPC_N;
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
    auto& H = get_matrix(rate);

    std::vector<uint8_t> padded = data_bits;
    while (padded.size() % k != 0)
        padded.push_back(0);

    std::vector<uint8_t> output;
    output.reserve(padded.size() / k * n);

    for (size_t offset = 0; offset < padded.size(); offset += k) {
        std::vector<uint8_t> codeword(n, 0);

        // Copy data bits (systematic)
        for (int i = 0; i < k; i++)
            codeword[i] = padded[offset + i] & 1;

        // Compute each parity bit from its check equation:
        // Check j: XOR of all connected columns = 0
        // Pivot is column k+j, so: codeword[k+j] = XOR of all other connected columns
        for (int j = 0; j < H.p; j++) {
            uint8_t syndrome = 0;
            for (int col : H.check_nodes[j]) {
                if (col != k + j) // skip pivot
                    syndrome ^= codeword[col];
            }
            codeword[k + j] = syndrome & 1;
        }

        for (int i = 0; i < n; i++)
            output.push_back(codeword[i]);
    }

    return output;
}

std::vector<uint8_t> LdpcCodec::decode(const std::vector<uint8_t>& codeword, LdpcRate rate) {
    int k = block_size(rate);
    int n = codeword_size(rate);

    if (codeword.size() % n != 0)
        return {};

    auto& H = get_matrix(rate);
    constexpr int MAX_ITER = 50;
    constexpr float SCALE = 0.75f; // min-sum scaling

    std::vector<uint8_t> output;
    output.reserve(codeword.size() / n * k);

    // Build variable-to-check adjacency once
    std::vector<std::vector<int>> var_to_checks(n);
    for (int j = 0; j < H.p; j++) {
        for (int col : H.check_nodes[j]) {
            var_to_checks[col].push_back(j);
        }
    }

    // For fast lookup: position of variable col in check j's list
    // check_var_idx[j][col] = index of col in check_nodes[j], or -1
    std::vector<std::vector<int>> check_var_idx(H.p);
    for (int j = 0; j < H.p; j++) {
        check_var_idx[j].assign(n, -1);
        for (int idx = 0; idx < (int)H.check_nodes[j].size(); idx++)
            check_var_idx[j][H.check_nodes[j][idx]] = idx;
    }

    for (size_t offset = 0; offset < codeword.size(); offset += n) {
        // Channel LLRs: 0 -> +3.0, 1 -> -3.0 (soft-ish from hard bits)
        std::vector<float> llr(n);
        for (int i = 0; i < n; i++)
            llr[i] = (codeword[offset + i] & 1) ? -3.0f : 3.0f;

        // c2v messages: c2v[j][idx] for check j, variable index idx
        std::vector<std::vector<float>> c2v(H.p);
        for (int j = 0; j < H.p; j++)
            c2v[j].assign(H.check_nodes[j].size(), 0.0f);

        std::vector<float> posterior(n);
        bool converged = false;

        for (int iter = 0; iter < MAX_ITER && !converged; iter++) {
            // Check node update (min-sum)
            for (int j = 0; j < H.p; j++) {
                int deg = (int)H.check_nodes[j].size();

                // Compute v2c messages: v2c[idx] = llr[v] + sum c2v from other checks
                std::vector<float> v2c(deg);
                for (int idx = 0; idx < deg; idx++) {
                    int v = H.check_nodes[j][idx];
                    float msg = llr[v];
                    for (int jj : var_to_checks[v]) {
                        if (jj == j) continue;
                        int vi = check_var_idx[jj][v];
                        if (vi >= 0) msg += c2v[jj][vi];
                    }
                    v2c[idx] = msg;
                }

                // Min-sum: c2v[j][idx] = sign_product * min_abs (excluding idx)
                for (int idx = 0; idx < deg; idx++) {
                    float min_abs = 1e9f;
                    int sign = 1;
                    for (int idx2 = 0; idx2 < deg; idx2++) {
                        if (idx2 == idx) continue;
                        float a = std::abs(v2c[idx2]);
                        if (a < min_abs) min_abs = a;
                        if (v2c[idx2] < 0) sign = -sign;
                    }
                    c2v[j][idx] = sign * min_abs * SCALE;
                }
            }

            // Compute posterior LLRs
            for (int i = 0; i < n; i++) {
                posterior[i] = llr[i];
                for (int j : var_to_checks[i]) {
                    int vi = check_var_idx[j][i];
                    if (vi >= 0) posterior[i] += c2v[j][vi];
                }
            }

            // Syndrome check
            converged = true;
            for (int j = 0; j < H.p; j++) {
                int parity = 0;
                for (int col : H.check_nodes[j]) {
                    if (posterior[col] < 0) parity ^= 1;
                }
                if (parity != 0) { converged = false; break; }
            }
        }

        // Extract data bits
        for (int i = 0; i < k; i++)
            output.push_back(posterior[i] < 0 ? 1 : 0);
    }

    return output;
}

} // namespace iris
