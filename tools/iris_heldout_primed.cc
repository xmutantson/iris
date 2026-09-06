/*
 * iris_heldout_primed — HELD-OUT Winlink-primed compression measurement (Iris).
 *
 * Measures the PRODUCTION iris::Compressor on each Winlink message (300-3000 B) two
 * ways, per-message (fresh stream each time — NO session carry, NO cross-message
 * dedup Iris would not have on distinct real traffic):
 *
 *   COLD   : streaming on, dict priming OFF, fresh context — today's first-message
 *            regime.
 *   PRIMED : streaming on, dict priming ON (default) — streaming_enable() seeds the
 *            firmware-baked Winlink boilerplate dict (winlink_dict.cc) into BOTH the
 *            PPMd model and the zstd prefix BEFORE the message is compressed. The dict
 *            bytes are NEVER on the wire (baked into both ends). Only the message's own
 *            wire bytes are measured. This is the exact live-session primed path.
 *
 * HELD-OUT DISCIPLINE (C3): the dict is TRAINED on set A = Winlink BOILERPLATE ONLY
 * (headers/form scaffolding/common phrases — winlink_dict_v1.txt, NO message bodies).
 * The messages measured here are set B = real message bodies, DISJOINT from set A. The
 * harness asserts each message's distinctive body content is ABSENT from the dict, so
 * the lift is from shared boilerplate, never memorized bodies (never train-on-test).
 *
 * The PRIMED wire is decompressed by an identically-primed RX to confirm bit-exact
 * round-trip (the RX must hold the same firmware-baked dict).
 *
 * Edge (VARA comparison): edge = VARA_wire / Iris_wire (>1 => Iris sends fewer bytes).
 * VARA_wire = the message's LZHUF .lzh (Winlink B2F, byte-identical to the wl2k-go ref;
 * VARA cannot prime — LZHUF space-fills its 2 KB window, adaptive Huffman cold-starts).
 *
 * Usage: iris_heldout_primed <corpus_dir> <name:plainfile:lzhfile> [<name:plain:lzh> ...]
 * Prints per-message + byte-weighted aggregate. Exit 0 iff every RT is bit-exact AND
 * the held-out disjointness invariant holds.
 *
 * MEASUREMENT tool. No production changes.
 */
#include "compress/compress.h"
#include "compress/winlink_dict.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static std::string read_file(const std::string& path, bool* ok) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { *ok = false; return {}; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); rewind(f);
    std::string s((size_t)sz, 0);
    if (sz > 0 && fread(&s[0], 1, sz, f) != (size_t)sz) { fclose(f); *ok = false; return {}; }
    fclose(f); *ok = true;
    return s;
}

// Compress one message with a fresh streaming compressor (per-message; no carry).
// prime selects dict priming. Round-trips through an identically-(non-)primed RX.
static int compress_one(const std::string& msg, bool prime, bool* rt_ok) {
    iris::Compressor tx; tx.set_dict_priming(prime); tx.init(); tx.streaming_enable();
    iris::Compressor rx; rx.set_dict_priming(prime); rx.init(); rx.streaming_enable();
    static std::vector<uint8_t> out(1 << 20), rt(1 << 20);
    int w = tx.compress_block((const uint8_t*)msg.data(), (int)msg.size(), out.data(), (int)out.size());
    if (w <= 0) { tx.deinit(); rx.deinit(); *rt_ok = false; return -1; }
    int d = rx.decompress_block(out.data(), w, rt.data(), (int)rt.size());
    *rt_ok = (d == (int)msg.size() && memcmp(rt.data(), msg.data(), msg.size()) == 0);
    tx.deinit(); rx.deinit();
    return w;
}

// Longest contiguous byte run of `msg` that also appears verbatim in `dict`.
// Used to assert held-out disjointness: a large run would indicate the dict memorized
// the message body (train-on-test). Boilerplate header lines legitimately share short
// runs; a distinctive body is many hundreds of unique bytes.
static int longest_shared_run(const std::string& msg, const std::string& dict) {
    // O(n*m) is fine for < 3 KB msg / < 16 KB dict.
    int best = 0;
    for (size_t i = 0; i < msg.size(); i++) {
        for (size_t j = 0; j < dict.size(); j++) {
            size_t k = 0;
            while (i + k < msg.size() && j + k < dict.size() && msg[i + k] == dict[j + k]) k++;
            if ((int)k > best) best = (int)k;
        }
    }
    return best;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <corpus_dir> <name:plain:lzh> [...]\n", argv[0]);
        return 2;
    }
    std::string dir = argv[1];
    std::string dict(WINLINK_DICT_RAW, WINLINK_DICT_RAW_LEN);

    // Disjointness threshold: no message may share a run this long with the dict.
    // Set above the longest legitimate boilerplate line but far below any body length.
    const int DISJOINT_MAX_RUN = 120;

    double sum_plain = 0, sum_cold = 0, sum_primed = 0, sum_vara = 0;
    bool all_ok = true, all_disjoint = true;

    printf("%-14s %8s %9s %9s %9s %9s %10s %10s %9s\n",
           "message", "plain", "cold_w", "primed_w", "vara_w",
           "cold_R", "primed_R", "edge_prim", "run/RT");
    printf("--------------------------------------------------------------------------------------------------\n");

    for (int a = 2; a < argc; a++) {
        std::string spec = argv[a];
        size_t c1 = spec.find(':'), c2 = spec.find(':', c1 + 1);
        if (c1 == std::string::npos || c2 == std::string::npos) {
            fprintf(stderr, "bad spec '%s' (want name:plain:lzh)\n", spec.c_str());
            return 2;
        }
        std::string name = spec.substr(0, c1);
        std::string pf = dir + "/" + spec.substr(c1 + 1, c2 - c1 - 1);
        std::string lf = dir + "/" + spec.substr(c2 + 1);
        bool ok1, ok2;
        std::string msg = read_file(pf, &ok1);
        std::string lzh = read_file(lf, &ok2);
        if (!ok1 || !ok2) { fprintf(stderr, "cannot read %s / %s\n", pf.c_str(), lf.c_str()); return 2; }

        bool cold_rt, primed_rt;
        int cold = compress_one(msg, false, &cold_rt);
        int primed = compress_one(msg, true, &primed_rt);
        int shared = longest_shared_run(msg, dict);
        bool disjoint = (shared <= DISJOINT_MAX_RUN);
        bool rt_ok = cold_rt && primed_rt;
        all_ok = all_ok && rt_ok;
        all_disjoint = all_disjoint && disjoint;

        double cr = cold  > 0 ? (double)msg.size() / cold   : 0;
        double pr = primed> 0 ? (double)msg.size() / primed : 0;
        double edge = primed > 0 ? (double)lzh.size() / primed : 0;

        printf("%-14s %8zu %9d %9d %9zu %9.3f %10.3f %10.3f  %3d/%s\n",
               name.c_str(), msg.size(), cold, primed, lzh.size(),
               cr, pr, edge, shared, rt_ok ? "ok" : "BAD");

        sum_plain  += (double)msg.size();
        sum_cold   += cold   > 0 ? cold   : msg.size();
        sum_primed += primed > 0 ? primed : msg.size();
        sum_vara   += (double)lzh.size();
    }

    printf("--------------------------------------------------------------------------------------------------\n");
    double agg_cold_edge   = sum_cold   > 0 ? sum_vara / sum_cold   : 0;
    double agg_primed_edge = sum_primed > 0 ? sum_vara / sum_primed : 0;
    double agg_cold_ratio  = sum_cold   > 0 ? sum_plain / sum_cold  : 0;
    double agg_primed_ratio= sum_primed > 0 ? sum_plain / sum_primed: 0;
    printf("AGGREGATE (byte-weighted): plain=%.0f cold_w=%.0f primed_w=%.0f vara_w=%.0f\n",
           sum_plain, sum_cold, sum_primed, sum_vara);
    printf("  cold_ratio=%.3f  primed_ratio=%.3f\n", agg_cold_ratio, agg_primed_ratio);
    printf("  EDGE (VARA_wire/Iris_wire):  cold=%.3f  PRIMED=%.3f   (beat threshold 1.63)\n",
           agg_cold_edge, agg_primed_edge);
    printf("  E = R_iris_primed / R_vara = primed_ratio / (plain/vara) = %.3f\n",
           (sum_vara > 0) ? (agg_primed_ratio / (sum_plain / sum_vara)) : 0);
    printf("  RT bit-exact all: %s   held-out disjoint all (run<=%d): %s\n",
           all_ok ? "YES" : "NO", DISJOINT_MAX_RUN, all_disjoint ? "YES" : "NO");

    return (all_ok && all_disjoint) ? 0 : 1;
}
