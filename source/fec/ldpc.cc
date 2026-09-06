#include "fec/ldpc.h"
#include "common/logging.h"
#include "fec/mercury_normal_4_16.h"
#include "fec/mercury_normal_6_16.h"
#include "fec/mercury_normal_8_16.h"
#include "fec/mercury_normal_10_16.h"
#include "fec/mercury_normal_12_16.h"
#include "fec/mercury_normal_14_16.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <mutex>

namespace iris {

// All rates use 1600-bit codewords, systematic: [data | parity]
static constexpr int LDPC_N = 1600;

// Track worst-case LDPC iterations across blocks in last decode_soft call
static std::atomic<int> g_ldpc_max_iters{0};
int ldpc_last_max_iters() { return g_ldpc_max_iters.load(std::memory_order_relaxed); }

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

// Mercury IRA matrix descriptor — points into precomputed tables
struct IraMatrix {
    int n, k, p;
    int cwidth, vwidth;
    const int* QCmatrixC;   // [P][Cwidth] — check-to-variable adjacency
    const int* QCmatrixV;   // [N][Vwidth] — variable-to-check adjacency
    const int* QCmatrixEnc; // [P][Cwidth-1] — encoding matrix (C minus self-parity)

    int c_adj(int check, int idx) const { return QCmatrixC[check * cwidth + idx]; }
    int v_adj(int var, int idx) const { return QCmatrixV[var * vwidth + idx]; }
    int enc_adj(int check, int idx) const { return QCmatrixEnc[check * (cwidth - 1) + idx]; }
};

// Get the IRA matrix for a given rate.
// Rates without a Mercury precomputed table fall back to rate 1/2.
static IraMatrix get_ira_matrix(LdpcRate rate) {
    IraMatrix m;
    m.n = LDPC_N;

    switch (rate) {
        case LdpcRate::RATE_4_16:
            m.k = 400; m.p = 1200;
            m.cwidth = mercury_normal_Cwidth_4_16;
            m.vwidth = mercury_normal_Vwidth_4_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_4_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_4_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_4_16[0][0];
            // L4 (audit §2): per-decode IRIS_LOG (mutex + snprintf + vfprintf)
            // dropped — get_ira_matrix runs on every decode on the capture thread.
            break;
        case LdpcRate::RATE_6_16:
            m.k = 600; m.p = 1000;
            m.cwidth = mercury_normal_Cwidth_6_16;
            m.vwidth = mercury_normal_Vwidth_6_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_6_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_6_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_6_16[0][0];
            // L4: per-decode log dropped (see rate 4/16 note above).
            break;
        case LdpcRate::RATE_1_2:
            m.k = 800; m.p = 800;
            m.cwidth = mercury_normal_Cwidth_8_16;
            m.vwidth = mercury_normal_Vwidth_8_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_8_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_8_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_8_16[0][0];
            break;
        case LdpcRate::RATE_5_8:
            m.k = 1000; m.p = 600;
            m.cwidth = mercury_normal_Cwidth_10_16;
            m.vwidth = mercury_normal_Vwidth_10_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_10_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_10_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_10_16[0][0];
            // L4: per-decode log dropped (see rate 4/16 note above).
            break;
        case LdpcRate::RATE_3_4:
            m.k = 1200; m.p = 400;
            m.cwidth = mercury_normal_Cwidth_12_16;
            m.vwidth = mercury_normal_Vwidth_12_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_12_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_12_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_12_16[0][0];
            break;
        case LdpcRate::RATE_7_8:
            m.k = 1400; m.p = 200;
            m.cwidth = mercury_normal_Cwidth_14_16;
            m.vwidth = mercury_normal_Vwidth_14_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_14_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_14_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_14_16[0][0];
            break;
        default:
            // Fallback to rate 1/2
            m.k = 800; m.p = 800;
            m.cwidth = mercury_normal_Cwidth_8_16;
            m.vwidth = mercury_normal_Vwidth_8_16;
            m.QCmatrixC = &mercury_normal_QCmatrixC_8_16[0][0];
            m.QCmatrixV = &mercury_normal_QCmatrixV_8_16[0][0];
            m.QCmatrixEnc = &mercury_normal_QCmatrixEnc_8_16[0][0];
            break;
    }
    return m;
}

static LdpcRate effective_rate(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_4_16:
        case LdpcRate::RATE_6_16:
        case LdpcRate::RATE_1_2:
        case LdpcRate::RATE_5_8:
        case LdpcRate::RATE_3_4:
        case LdpcRate::RATE_7_8:
            return rate;
        default:
            return LdpcRate::RATE_1_2;
    }
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

// --- Encoder (IRA structure) ---

std::vector<uint8_t> LdpcCodec::encode(const std::vector<uint8_t>& data_bits, LdpcRate rate) {
    if (rate == LdpcRate::NONE) return data_bits;

    LdpcRate eff = effective_rate(rate);
    auto M = get_ira_matrix(eff);
    int k = M.k;
    int n = M.n;
    int p = M.p;
    int enc_width = M.cwidth - 1;

    std::vector<uint8_t> padded = data_bits;
    while (padded.size() % k != 0)
        padded.push_back(0);

    std::vector<uint8_t> output;
    output.reserve(padded.size() / k * n);

    for (size_t offset = 0; offset < padded.size(); offset += k) {
        std::vector<uint8_t> codeword(n, 0);
        for (int i = 0; i < k; i++)
            codeword[i] = padded[offset + i] & 1;

        // IRA encoding using QCmatrixEnc
        for (int i = 0; i < p; i++) {
            uint8_t val = 0;
            for (int j = 0; j < enc_width; j++) {
                int bit_idx = M.enc_adj(i, j);
                if (bit_idx == -1) break;
                val ^= codeword[bit_idx];
            }
            codeword[k + i] = val;
        }

        for (int i = 0; i < n; i++)
            output.push_back(codeword[i]);
    }
    return output;
}

// --- Decoder workspace ---

struct DecoderWorkspace {
    int n, p, cwidth, vwidth;
    std::vector<float> c2v;      // [p * cwidth]
    std::vector<float> posterior; // [n]

    void init(const IraMatrix& M) {
        n = M.n; p = M.p; cwidth = M.cwidth; vwidth = M.vwidth;
        c2v.assign(p * cwidth, 0.0f);
        posterior.resize(n);
    }

    void reset_messages() {
        std::fill(c2v.begin(), c2v.end(), 0.0f);
    }

    bool check_syndrome(const IraMatrix& M) {
        for (int j = 0; j < p; j++) {
            int parity = 0;
            for (int idx = 0; idx < cwidth; idx++) {
                int col = M.c_adj(j, idx);
                if (col == -1) break;
                if (!std::isfinite(posterior[col])) return false;
                if (posterior[col] < 0) parity ^= 1;
            }
            if (parity != 0) return false;
        }
        return true;
    }
};

// L4 / precook (audit §2 L4, §4 R3): persistent decode scratch, sized once by
// Modem::precook() at startup instead of heap-allocated on every decode on the
// audio capture thread under modem_mutex_. Access is serialized so concurrent
// modem instances cannot share in-flight message state.
static std::mutex g_decode_mutex;
static DecoderWorkspace g_decode_ws;
static std::vector<float> g_decode_llr;   // per-block channel LLR scratch (n floats)
static std::vector<float> g_spa_Q;        // SPA-only variable->check messages (p*cwidth)

// Helper: find index of variable `var` in check `chk`'s adjacency list
static inline int find_var_in_check(const IraMatrix& M, int chk, int var) {
    for (int cidx = 0; cidx < M.cwidth; cidx++) {
        int c = M.c_adj(chk, cidx);
        if (c == var) return cidx;
        if (c == -1) break;
    }
    return -1;
}

// --- Layered Min-Sum Decoder ---
// Processes one check-node row at a time, immediately updating variable
// posteriors before moving to the next row.  Information propagates
// multiple hops per sweep — converges in ~15 sweeps vs ~30 flooding iters.
// This frees the iteration budget for marginal cliff frames.

static int decode_min_sum(const std::vector<float>& llr, const IraMatrix& M,
                           DecoderWorkspace& ws, int max_iter,
                           std::atomic<bool>* abort_flag) {
    constexpr float SCALE = 0.80f;
    ws.reset_messages();

    // Initialize posteriors with channel LLRs
    for (int i = 0; i < M.n; i++)
        ws.posterior[i] = llr[i];

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        // Layered sweep: for each check node, update c2v messages and
        // immediately update connected variable posteriors.
        for (int j = 0; j < M.p; j++) {
            int deg = 0;
            float v2c[64];
            int cols[64];

            // Compute v2c = posterior - old c2v (extrinsic info to check node)
            for (int idx = 0; idx < M.cwidth; idx++) {
                int col = M.c_adj(j, idx);
                if (col == -1) break;
                cols[deg] = col;
                v2c[deg] = ws.posterior[col] - ws.c2v[j * M.cwidth + idx];
                deg++;
            }

            // Min-sum check node update
            for (int idx = 0; idx < deg; idx++) {
                float min_abs = 1e9f;
                int sign = 1;
                for (int idx2 = 0; idx2 < deg; idx2++) {
                    if (idx2 == idx) continue;
                    float a = std::abs(v2c[idx2]);
                    if (a < min_abs) min_abs = a;
                    if (v2c[idx2] < 0) sign = -sign;
                }
                float new_c2v = sign * min_abs * SCALE;

                // Immediately update variable posterior:
                // posterior += (new_c2v - old_c2v)
                ws.posterior[cols[idx]] += new_c2v - ws.c2v[j * M.cwidth + idx];
                ws.c2v[j * M.cwidth + idx] = new_c2v;
            }
        }

        // Early bail-out for noise: after iteration 3, count unsatisfied
        // parity checks (syndrome weight).  Real frames at decodable SNR
        // reduce syndrome weight to <30% by iteration 3.  Pure noise stays
        // at ~50% (random bits satisfy half the checks by chance).
        // Bail if syndrome weight > 45% after iter 3 — saves ~47 wasted
        // iterations on false triggers.
        if (iter == 3) {
            int unsatisfied = 0;
            for (int j = 0; j < M.p; j++) {
                int parity = 0;
                for (int idx = 0; idx < M.cwidth; idx++) {
                    int col = M.c_adj(j, idx);
                    if (col == -1) break;
                    if (ws.posterior[col] < 0) parity ^= 1;
                }
                if (parity != 0) unsatisfied++;
            }
            float sw = (float)unsatisfied / M.p;
            if (sw > 0.45f)
                return iter + 1;  // noise — no point continuing
        }

        if (ws.check_syndrome(M))
            return iter + 1;
    }
    return max_iter;
}

// --- Sum-Product Algorithm (SPA) Decoder ---

static int decode_spa(const std::vector<float>& llr, const IraMatrix& M,
                       DecoderWorkspace& ws, int max_iter,
                       std::atomic<bool>* abort_flag) {
    ws.reset_messages();

    for (int i = 0; i < M.n; i++)
        ws.posterior[i] = llr[i];

    std::vector<float>& Q = g_spa_Q;      // L4: persistent (see g_spa_Q note)
    Q.assign(M.p * M.cwidth, 0.0f);

    // Initialize Q with channel LLRs
    for (int j = 0; j < M.p; j++) {
        for (int idx = 0; idx < M.cwidth; idx++) {
            int col = M.c_adj(j, idx);
            if (col == -1) break;
            Q[j * M.cwidth + idx] = llr[col];
        }
    }

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        // Check node update
        for (int j = 0; j < M.p; j++) {
            int deg = 0;
            for (int idx = 0; idx < M.cwidth; idx++) {
                if (M.c_adj(j, idx) == -1) break;
                deg++;
            }

            for (int idx = 0; idx < deg; idx++) {
                float product = 1.0f;
                for (int k = 0; k < deg; k++) {
                    if (k == idx) continue;
                    float t = tanhf(0.5f * Q[j * M.cwidth + k]);
                    if (t > 0.9999999f) t = 0.9999999f;
                    if (t < -0.9999999f) t = -0.9999999f;
                    product *= t;
                }
                if (product > 0.9999999f) product = 0.9999999f;
                if (product < -0.9999999f) product = -0.9999999f;
                ws.c2v[j * M.cwidth + idx] = 2.0f * atanhf(product);
            }
        }

        // Variable node update + posterior
        for (int i = 0; i < M.n; i++) {
            float sum_r = 0.0f;
            for (int vidx = 0; vidx < M.vwidth; vidx++) {
                int chk = M.v_adj(i, vidx);
                if (chk == -1) break;
                int ci = find_var_in_check(M, chk, i);
                if (ci >= 0) sum_r += ws.c2v[chk * M.cwidth + ci];
            }
            ws.posterior[i] = llr[i] + sum_r;

            for (int vidx = 0; vidx < M.vwidth; vidx++) {
                int chk = M.v_adj(i, vidx);
                if (chk == -1) break;
                int ci = find_var_in_check(M, chk, i);
                if (ci >= 0)
                    Q[chk * M.cwidth + ci] = ws.posterior[i] - ws.c2v[chk * M.cwidth + ci];
            }
        }

        if (ws.check_syndrome(M))
            return iter + 1;
    }
    return max_iter;
}

// --- Gradient Bit-Flipping (GBF) Decoder ---

static int decode_gbf(const std::vector<float>& llr, const IraMatrix& M,
                       DecoderWorkspace& ws, int max_iter,
                       std::atomic<bool>* abort_flag) {
    constexpr float ETA = 1.0f;
    std::vector<float> llr_est(llr.begin(), llr.end());

    for (int iter = 0; iter < max_iter; iter++) {
        if (abort_flag && abort_flag->load(std::memory_order_relaxed))
            return -(iter + 1);

        std::vector<int> syndrome(M.p);
        bool all_satisfied = true;
        for (int j = 0; j < M.p; j++) {
            int parity = 0;
            for (int idx = 0; idx < M.cwidth; idx++) {
                int col = M.c_adj(j, idx);
                if (col == -1) break;
                if (llr_est[col] < 0) parity ^= 1;
            }
            syndrome[j] = parity;
            if (parity != 0) all_satisfied = false;
        }

        if (all_satisfied) {
            for (int i = 0; i < M.n; i++)
                ws.posterior[i] = llr_est[i];
            return iter + 1;
        }

        std::vector<float> delta(M.n, 0.0f);
        for (int j = 0; j < M.p; j++) {
            float contrib = 2.0f * syndrome[j] - 1.0f;
            for (int idx = 0; idx < M.cwidth; idx++) {
                int col = M.c_adj(j, idx);
                if (col == -1) break;
                delta[col] += contrib;
            }
        }

        for (int i = 0; i < M.n; i++) {
            if (delta[i] > 0) {
                float sign_flip = (llr_est[i] < 0) ? 1.0f : -1.0f;
                llr_est[i] += sign_flip * delta[i] * ETA;
            }
        }
    }

    for (int i = 0; i < M.n; i++)
        ws.posterior[i] = llr_est[i];
    return max_iter;
}

// --- Unified decode entry points ---

std::vector<uint8_t> LdpcCodec::decode(const std::vector<uint8_t>& codeword, LdpcRate rate,
                                         LdpcDecoder algo, int max_iter,
                                         std::atomic<bool>* abort_flag) {
    if (rate == LdpcRate::NONE) return codeword;

    std::lock_guard<std::mutex> lock(g_decode_mutex);
    LdpcRate eff = effective_rate(rate);
    auto M = get_ira_matrix(eff);
    int k = M.k;
    int n = M.n;
    if (codeword.size() % n != 0) return {};

    DecoderWorkspace& ws = g_decode_ws;   // L4: persistent (see g_decode_ws note)
    ws.init(M);

    std::vector<uint8_t> output;
    output.reserve(codeword.size() / n * k);

    for (size_t offset = 0; offset < codeword.size(); offset += n) {
        std::vector<float>& llr = g_decode_llr;   // L4: persistent
        llr.resize(n);
        for (int i = 0; i < n; i++)
            llr[i] = (codeword[offset + i] & 1) ? -3.0f : 3.0f;

        int result;
        switch (algo) {
            case LdpcDecoder::SPA:
                result = decode_spa(llr, M, ws, max_iter, abort_flag);
                break;
            case LdpcDecoder::GBF:
                result = decode_gbf(llr, M, ws, max_iter, abort_flag);
                break;
            default:
                result = decode_min_sum(llr, M, ws, max_iter, abort_flag);
                break;
        }

        if (result < 0 || !ws.check_syndrome(M)) return {};

        for (int i = 0; i < k; i++)
            output.push_back(ws.posterior[i] < 0 ? 1 : 0);
    }
    return output;
}

std::vector<uint8_t> LdpcCodec::decode_soft(const std::vector<float>& llrs, LdpcRate rate,
                                              LdpcDecoder algo, int max_iter,
                                              std::atomic<bool>* abort_flag) {
    std::lock_guard<std::mutex> lock(g_decode_mutex);
    g_ldpc_max_iters.store(0, std::memory_order_relaxed);  // reset per-frame tracker

    if (rate == LdpcRate::NONE) {
        std::vector<uint8_t> out(llrs.size());
        for (size_t i = 0; i < llrs.size(); i++)
            out[i] = llrs[i] < 0 ? 1 : 0;
        return out;
    }

    LdpcRate eff = effective_rate(rate);
    auto M = get_ira_matrix(eff);
    int k = M.k;
    int n = M.n;
    if ((int)llrs.size() % n != 0) return {};

    DecoderWorkspace& ws = g_decode_ws;   // L4: persistent (see g_decode_ws note)
    ws.init(M);

    std::vector<uint8_t> output;
    output.reserve(llrs.size() / n * k);

    for (size_t offset = 0; offset < llrs.size(); offset += n) {
        std::vector<float>& llr = g_decode_llr;   // L4: persistent
        llr.assign(llrs.begin() + offset, llrs.begin() + offset + n);

        int result;
        switch (algo) {
            case LdpcDecoder::SPA:
                result = decode_spa(llr, M, ws, max_iter, abort_flag);
                break;
            case LdpcDecoder::GBF:
                result = decode_gbf(llr, M, ws, max_iter, abort_flag);
                break;
            default:
                result = decode_min_sum(llr, M, ws, max_iter, abort_flag);
                break;
        }

        if (result < 0) return {};
        if (!ws.check_syndrome(M)) {
            IRIS_LOG("[LDPC] block %zu did not converge after %d iters",
                     offset / n, result);
            g_ldpc_max_iters.store(max_iter, std::memory_order_relaxed);
            return {};
        }
        IRIS_LOG("[LDPC] block %zu converged in %d/%d iters",
                 offset / n, result, max_iter);
        if (result > g_ldpc_max_iters.load(std::memory_order_relaxed))
            g_ldpc_max_iters.store(result, std::memory_order_relaxed);

        for (int i = 0; i < k; i++)
            output.push_back(ws.posterior[i] < 0 ? 1 : 0);
    }
    return output;
}

std::vector<LdpcCodec::BlockResult> LdpcCodec::decode_soft_per_block(
    const std::vector<float>& llrs, LdpcRate rate,
    LdpcDecoder algo, int max_iter,
    std::atomic<bool>* abort_flag) {

    std::lock_guard<std::mutex> lock(g_decode_mutex);
    g_ldpc_max_iters.store(0, std::memory_order_relaxed);

    if (rate == LdpcRate::NONE) {
        // No FEC: single "block" with hard decisions
        BlockResult br;
        br.converged = true;
        br.iterations = 0;
        br.data_bits.resize(llrs.size());
        for (size_t i = 0; i < llrs.size(); i++)
            br.data_bits[i] = llrs[i] < 0 ? 1 : 0;
        return {br};
    }

    LdpcRate eff = effective_rate(rate);
    auto M = get_ira_matrix(eff);
    int k = M.k;
    int n = M.n;
    if ((int)llrs.size() % n != 0) return {};

    DecoderWorkspace& ws = g_decode_ws;   // L4: persistent (see g_decode_ws note)
    ws.init(M);

    size_t num_blocks = llrs.size() / n;
    std::vector<BlockResult> results(num_blocks);

    for (size_t b = 0; b < num_blocks; b++) {
        size_t offset = b * n;
        std::vector<float>& llr = g_decode_llr;   // L4: persistent
        llr.assign(llrs.begin() + offset, llrs.begin() + offset + n);

        int result;
        switch (algo) {
            case LdpcDecoder::SPA:
                result = decode_spa(llr, M, ws, max_iter, abort_flag);
                break;
            case LdpcDecoder::GBF:
                result = decode_gbf(llr, M, ws, max_iter, abort_flag);
                break;
            default:
                result = decode_min_sum(llr, M, ws, max_iter, abort_flag);
                break;
        }

        if (result < 0 || !ws.check_syndrome(M)) {
            results[b].converged = false;
            results[b].iterations = (result < 0) ? max_iter : result;
            IRIS_LOG("[LDPC] block %zu did not converge after %d iters", b, results[b].iterations);
            g_ldpc_max_iters.store(max_iter, std::memory_order_relaxed);
        } else {
            results[b].converged = true;
            results[b].iterations = result;
            results[b].data_bits.resize(k);
            for (int i = 0; i < k; i++)
                results[b].data_bits[i] = ws.posterior[i] < 0 ? 1 : 0;
            IRIS_LOG("[LDPC] block %zu converged in %d/%d iters", b, result, max_iter);
            if (result > g_ldpc_max_iters.load(std::memory_order_relaxed))
                g_ldpc_max_iters.store(result, std::memory_order_relaxed);
        }
    }
    return results;
}

// L4 / precook (audit §4): size the persistent decode scratch once at startup
// (from Modem::precook(), before the capture thread exists) so no decode ever
// heap-allocates workspace/LLR/SPA-Q on the audio callback thread under
// modem_mutex_. Idempotent: init()/assign() grow-only, so repeated calls are
// cheap no-ops. Visiting every real rate leaves each buffer's capacity at the
// cross-rate maximum, so subsequent per-decode init()/assign() never realloc.
void LdpcCodec::precook() {
    std::lock_guard<std::mutex> lock(g_decode_mutex);
    static const LdpcRate rates[] = {
        LdpcRate::RATE_4_16, LdpcRate::RATE_6_16, LdpcRate::RATE_1_2,
        LdpcRate::RATE_5_8,  LdpcRate::RATE_3_4,  LdpcRate::RATE_7_8,
    };
    size_t max_cw = 0;
    for (LdpcRate r : rates) {
        IraMatrix M = get_ira_matrix(r);
        g_decode_ws.init(M);                       // grows c2v/posterior capacity
        size_t cw = (size_t)M.p * (size_t)M.cwidth;
        if (cw > max_cw) max_cw = cw;
    }
    g_decode_llr.assign((size_t)LDPC_N, 0.0f);     // per-block LLR (n floats)
    g_spa_Q.assign(max_cw, 0.0f);                  // SPA messages (max p*cwidth)
}

} // namespace iris
