#include "fec/ldpc.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

namespace iris {

// All rates use 1600-bit codewords, systematic: [data | parity]
static constexpr int LDPC_N = 1600;

// Data bits (K) for each rate — P = N - K
static int rate_to_k(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_16: return 100;
        case LdpcRate::RATE_2_16: return 200;
        case LdpcRate::RATE_3_16: return 300;
        case LdpcRate::RATE_4_16: return 400;
        case LdpcRate::RATE_5_16: return 500;
        case LdpcRate::RATE_6_16: return 600;
        case LdpcRate::RATE_1_2:  return 800;
        case LdpcRate::RATE_5_8:  return 1000;
        case LdpcRate::RATE_3_4:  return 1200;
        case LdpcRate::RATE_7_8:  return 1400;
        default: return 0;
    }
}

// H matrix: p rows (check nodes) x n columns (variable nodes)
struct LdpcMatrix {
    int n, k, p;
    std::vector<std::vector<int>> check_nodes;

    LdpcMatrix() : n(0), k(0), p(0) {}
    LdpcMatrix(int n_, int k_) : n(n_), k(k_), p(n_ - k_) { build_h(); }

    void build_h() {
        check_nodes.resize(p);
        // Column weight: higher for low-rate codes (more redundancy)
        int wc = (p >= 600) ? 3 : (p >= 200) ? 3 : 2;

        for (int j = 0; j < p; j++) {
            check_nodes[j].clear();
            check_nodes[j].push_back(k + j); // pivot parity bit
        }

        for (int layer = 0; layer < wc; layer++) {
            for (int col = 0; col < k; col++) {
                int check = (col + layer * (k / std::max(1, wc))) % p;
                check_nodes[check].push_back(col);
            }
        }

        for (int j = 0; j < p; j++) {
            std::sort(check_nodes[j].begin(), check_nodes[j].end());
            check_nodes[j].erase(
                std::unique(check_nodes[j].begin(), check_nodes[j].end()),
                check_nodes[j].end());
        }
    }
};

// Lazy-initialized matrix cache
static LdpcMatrix& get_matrix(LdpcRate rate) {
    static LdpcMatrix matrices[10];
    static bool inited[10] = {};

    int idx;
    switch (rate) {
        case LdpcRate::RATE_1_16: idx = 0; break;
        case LdpcRate::RATE_2_16: idx = 1; break;
        case LdpcRate::RATE_3_16: idx = 2; break;
        case LdpcRate::RATE_4_16: idx = 3; break;
        case LdpcRate::RATE_5_16: idx = 4; break;
        case LdpcRate::RATE_6_16: idx = 5; break;
        case LdpcRate::RATE_1_2:  idx = 6; break;
        case LdpcRate::RATE_5_8:  idx = 7; break;
        case LdpcRate::RATE_3_4:  idx = 8; break;
        case LdpcRate::RATE_7_8:  idx = 9; break;
        default: idx = 6; break; // fallback to 1/2
    }

    if (!inited[idx]) {
        int k = rate_to_k(rate);
        if (k <= 0) k = 800;
        matrices[idx] = LdpcMatrix(LDPC_N, k);
        inited[idx] = true;
    }
    return matrices[idx];
}

// --- Public API ---

int LdpcCodec::block_size(LdpcRate rate) {
    int k = rate_to_k(rate);
    return k > 0 ? k : 0;
}

int LdpcCodec::codeword_size(LdpcRate) { return LDPC_N; }

int LdpcCodec::parity_bits(LdpcRate rate) {
    return codeword_size(rate) - block_size(rate);
}

float LdpcCodec::code_rate(LdpcRate rate) {
    int k = block_size(rate);
    return k > 0 ? (float)k / (float)LDPC_N : 0.0f;
}

LdpcRate fec_to_ldpc_rate(int num, int den) {
    if (den == 16) {
        switch (num) {
            case 1: return LdpcRate::RATE_1_16;
            case 2: return LdpcRate::RATE_2_16;
            case 3: return LdpcRate::RATE_3_16;
            case 4: return LdpcRate::RATE_4_16;
            case 5: return LdpcRate::RATE_5_16;
            case 6: return LdpcRate::RATE_6_16;
            case 8: return LdpcRate::RATE_1_2;
            case 10: return LdpcRate::RATE_5_8;
            case 12: return LdpcRate::RATE_3_4;
            case 14: return LdpcRate::RATE_7_8;
        }
    }
    if (num == 1 && den == 2) return LdpcRate::RATE_1_2;
    if (num == 5 && den == 8) return LdpcRate::RATE_5_8;
    if (num == 3 && den == 4) return LdpcRate::RATE_3_4;
    if (num == 7 && den == 8) return LdpcRate::RATE_7_8;
    return LdpcRate::NONE;
}

// --- Encoder ---

std::vector<uint8_t> LdpcCodec::encode(const std::vector<uint8_t>& data_bits, LdpcRate rate) {
    if (rate == LdpcRate::NONE) return data_bits;
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
        for (int i = 0; i < k; i++)
            codeword[i] = padded[offset + i] & 1;

        for (int j = 0; j < H.p; j++) {
            uint8_t syndrome = 0;
            for (int col : H.check_nodes[j]) {
                if (col != k + j)
                    syndrome ^= codeword[col];
            }
            codeword[k + j] = syndrome & 1;
        }

        for (int i = 0; i < n; i++)
            output.push_back(codeword[i]);
    }
    return output;
}

// --- Decoder helpers ---

struct DecoderWorkspace {
    int n, p;
    std::vector<std::vector<int>> var_to_checks;
    std::vector<std::vector<int>> check_var_idx;
    std::vector<std::vector<float>> c2v;
    std::vector<float> posterior;

    void init(const LdpcMatrix& H) {
        n = H.n;
        p = H.p;
        var_to_checks.assign(n, {});
        for (int j = 0; j < p; j++)
            for (int col : H.check_nodes[j])
                var_to_checks[col].push_back(j);

        check_var_idx.resize(p);
        for (int j = 0; j < p; j++) {
            check_var_idx[j].assign(n, -1);
            for (int idx = 0; idx < (int)H.check_nodes[j].size(); idx++)
                check_var_idx[j][H.check_nodes[j][idx]] = idx;
        }

        c2v.resize(p);
        for (int j = 0; j < p; j++)
            c2v[j].assign(H.check_nodes[j].size(), 0.0f);

        posterior.resize(n);
    }

    void reset_messages(const LdpcMatrix&) {
        for (int j = 0; j < p; j++)
            std::fill(c2v[j].begin(), c2v[j].end(), 0.0f);
    }

    bool check_syndrome(const LdpcMatrix& H) {
        for (int j = 0; j < p; j++) {
            int parity = 0;
            for (int col : H.check_nodes[j])
                if (posterior[col] < 0) parity ^= 1;
            if (parity != 0) return false;
        }
        return true;
    }
};

// --- Min-Sum Decoder ---

static int decode_min_sum(const std::vector<float>& llr, const LdpcMatrix& H,
                           DecoderWorkspace& ws, int max_iter,
                           std::atomic<bool>* abort_flag) {
    constexpr float SCALE = 0.75f;
    ws.reset_messages(H);

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        // Check node update
        for (int j = 0; j < H.p; j++) {
            int deg = (int)H.check_nodes[j].size();
            std::vector<float> v2c(deg);
            for (int idx = 0; idx < deg; idx++) {
                int v = H.check_nodes[j][idx];
                float msg = llr[v];
                for (int jj : ws.var_to_checks[v]) {
                    if (jj == j) continue;
                    int vi = ws.check_var_idx[jj][v];
                    if (vi >= 0) msg += ws.c2v[jj][vi];
                }
                v2c[idx] = msg;
            }

            for (int idx = 0; idx < deg; idx++) {
                float min_abs = 1e9f;
                int sign = 1;
                for (int idx2 = 0; idx2 < deg; idx2++) {
                    if (idx2 == idx) continue;
                    float a = std::abs(v2c[idx2]);
                    if (a < min_abs) min_abs = a;
                    if (v2c[idx2] < 0) sign = -sign;
                }
                ws.c2v[j][idx] = sign * min_abs * SCALE;
            }
        }

        // Posterior
        for (int i = 0; i < H.n; i++) {
            ws.posterior[i] = llr[i];
            for (int j : ws.var_to_checks[i]) {
                int vi = ws.check_var_idx[j][i];
                if (vi >= 0) ws.posterior[i] += ws.c2v[j][vi];
            }
        }

        if (ws.check_syndrome(H))
            return iter + 1;
    }
    return max_iter;
}

// --- Sum-Product Algorithm (SPA) Decoder ---

static int decode_spa(const std::vector<float>& llr, const LdpcMatrix& H,
                       DecoderWorkspace& ws, int max_iter,
                       std::atomic<bool>* abort_flag) {
    ws.reset_messages(H);

    // Q messages: Q[j][idx] = variable-to-check message
    std::vector<std::vector<float>> Q(H.p);
    for (int j = 0; j < H.p; j++)
        Q[j].assign(H.check_nodes[j].size(), 0.0f);

    // Initialize Q with channel LLRs
    for (int j = 0; j < H.p; j++) {
        for (int idx = 0; idx < (int)H.check_nodes[j].size(); idx++) {
            Q[j][idx] = llr[H.check_nodes[j][idx]];
        }
    }

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        // Check node update: R[j][idx] = 2 * atanh(product of tanh(Q[j][k]/2) for k!=idx)
        for (int j = 0; j < H.p; j++) {
            int deg = (int)H.check_nodes[j].size();
            for (int idx = 0; idx < deg; idx++) {
                float product = 1.0f;
                for (int k = 0; k < deg; k++) {
                    if (k == idx) continue;
                    float t = tanhf(0.5f * Q[j][k]);
                    // Clamp to avoid singularities
                    if (t > 0.9999999f) t = 0.9999999f;
                    if (t < -0.9999999f) t = -0.9999999f;
                    product *= t;
                }
                if (product > 0.9999999f) product = 0.9999999f;
                if (product < -0.9999999f) product = -0.9999999f;
                ws.c2v[j][idx] = 2.0f * atanhf(product);
            }
        }

        // Variable node update: Q[j][idx] = LLR[v] + sum R[k][v] for k!=j
        // Also compute posterior
        for (int i = 0; i < H.n; i++) {
            float sum_r = 0.0f;
            for (int j : ws.var_to_checks[i]) {
                int vi = ws.check_var_idx[j][i];
                if (vi >= 0) sum_r += ws.c2v[j][vi];
            }
            ws.posterior[i] = llr[i] + sum_r;

            // Update Q messages
            for (int j : ws.var_to_checks[i]) {
                int vi = ws.check_var_idx[j][i];
                if (vi >= 0)
                    Q[j][vi] = ws.posterior[i] - ws.c2v[j][vi];
            }
        }

        if (ws.check_syndrome(H))
            return iter + 1;
    }
    return max_iter;
}

// --- Gradient Bit-Flipping (GBF) Decoder ---

static int decode_gbf(const std::vector<float>& llr, const LdpcMatrix& H,
                       DecoderWorkspace& ws, int max_iter,
                       std::atomic<bool>* abort_flag) {
    constexpr float ETA = 1.0f; // correction rate

    // Initialize LLR estimates
    std::vector<float> llr_est(llr.begin(), llr.end());

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        // Compute syndrome for each check
        std::vector<int> syndrome(H.p);
        bool all_satisfied = true;
        for (int j = 0; j < H.p; j++) {
            int parity = 0;
            for (int col : H.check_nodes[j])
                if (llr_est[col] < 0) parity ^= 1;
            syndrome[j] = parity;
            if (parity != 0) all_satisfied = false;
        }

        if (all_satisfied) {
            for (int i = 0; i < H.n; i++)
                ws.posterior[i] = llr_est[i];
            return iter + 1;
        }

        // Compute gradient delta for each variable
        std::vector<float> delta(H.n, 0.0f);
        for (int j = 0; j < H.p; j++) {
            float contrib = 2.0f * syndrome[j] - 1.0f; // +1 if unsatisfied, -1 if satisfied
            for (int col : H.check_nodes[j])
                delta[col] += contrib;
        }

        // Flip bits with positive delta
        for (int i = 0; i < H.n; i++) {
            if (delta[i] > 0) {
                float sign_flip = (llr_est[i] < 0) ? 1.0f : -1.0f;
                llr_est[i] += sign_flip * delta[i] * ETA;
            }
        }
    }

    // Copy final estimates to posterior
    for (int i = 0; i < H.n; i++)
        ws.posterior[i] = llr_est[i];

    return max_iter;
}

// --- Unified decode entry points ---

std::vector<uint8_t> LdpcCodec::decode(const std::vector<uint8_t>& codeword, LdpcRate rate,
                                         LdpcDecoder algo, int max_iter,
                                         std::atomic<bool>* abort_flag) {
    if (rate == LdpcRate::NONE) return codeword;

    int k = block_size(rate);
    int n = codeword_size(rate);
    if (codeword.size() % n != 0) return {};

    auto& H = get_matrix(rate);
    DecoderWorkspace ws;
    ws.init(H);

    std::vector<uint8_t> output;
    output.reserve(codeword.size() / n * k);

    for (size_t offset = 0; offset < codeword.size(); offset += n) {
        std::vector<float> llr(n);
        for (int i = 0; i < n; i++)
            llr[i] = (codeword[offset + i] & 1) ? -3.0f : 3.0f;

        int result;
        switch (algo) {
            case LdpcDecoder::SPA:
                result = decode_spa(llr, H, ws, max_iter, abort_flag);
                break;
            case LdpcDecoder::GBF:
                result = decode_gbf(llr, H, ws, max_iter, abort_flag);
                break;
            default:
                result = decode_min_sum(llr, H, ws, max_iter, abort_flag);
                break;
        }

        if (result < 0) return {}; // aborted

        for (int i = 0; i < k; i++)
            output.push_back(ws.posterior[i] < 0 ? 1 : 0);
    }
    return output;
}

std::vector<uint8_t> LdpcCodec::decode_soft(const std::vector<float>& llrs, LdpcRate rate,
                                              LdpcDecoder algo, int max_iter,
                                              std::atomic<bool>* abort_flag) {
    if (rate == LdpcRate::NONE) {
        std::vector<uint8_t> out(llrs.size());
        for (size_t i = 0; i < llrs.size(); i++)
            out[i] = llrs[i] < 0 ? 1 : 0;
        return out;
    }

    int k = block_size(rate);
    int n = codeword_size(rate);
    if ((int)llrs.size() % n != 0) return {};

    auto& H = get_matrix(rate);
    DecoderWorkspace ws;
    ws.init(H);

    std::vector<uint8_t> output;
    output.reserve(llrs.size() / n * k);

    for (size_t offset = 0; offset < llrs.size(); offset += n) {
        std::vector<float> llr(llrs.begin() + offset, llrs.begin() + offset + n);

        int result;
        switch (algo) {
            case LdpcDecoder::SPA:
                result = decode_spa(llr, H, ws, max_iter, abort_flag);
                break;
            case LdpcDecoder::GBF:
                result = decode_gbf(llr, H, ws, max_iter, abort_flag);
                break;
            default:
                result = decode_min_sum(llr, H, ws, max_iter, abort_flag);
                break;
        }

        if (result < 0) return {}; // aborted

        for (int i = 0; i < k; i++)
            output.push_back(ws.posterior[i] < 0 ? 1 : 0);
    }
    return output;
}

} // namespace iris
