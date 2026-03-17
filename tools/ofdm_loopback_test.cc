// OFDM Loopback Validation Test
// Exercises the complete TX -> AWGN channel -> RX chain at various SNR levels
// for HF configs (2 kHz, CP=64) and FM-optimized configs (5 kHz, CP=16, sparse pilots).
//
// Build (do not run automatically):
//   g++ -std=c++17 -O2 -Iinclude tools/ofdm_loopback_test.cc \
//       source/ofdm/*.cc source/common/*.cc source/fec/*.cc \
//       source/native/constellation.cc source/native/frame.cc \
//       source/probe/passband_probe.cc source/engine/speed_level.cc \
//       source/config/config.cc -o build/ofdm_loopback_test.exe

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_frame.h"
#include "fec/ldpc.h"
#include "probe/passband_probe.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex>
#include <vector>
#include <cstring>

using namespace iris;

// ---------------------------------------------------------------------------
// Box-Muller AWGN generator
// ---------------------------------------------------------------------------
static float randn() {
    static bool has_spare = false;
    static float spare;
    if (has_spare) { has_spare = false; return spare; }
    float u, v, s;
    do {
        u = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        v = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);
    s = std::sqrt(-2.0f * std::log(s) / s);
    spare = v * s;
    has_spare = true;
    return u * s;
}

// ---------------------------------------------------------------------------
// Add AWGN to complex IQ buffer in-place
// ---------------------------------------------------------------------------
static void add_awgn(std::vector<std::complex<float>>& iq, float snr_db) {
    if (iq.empty()) return;
    float signal_power = 0;
    for (auto& s : iq) signal_power += std::norm(s);
    signal_power /= (float)iq.size();

    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float sigma = std::sqrt(noise_power / 2.0f);

    for (auto& s : iq) {
        s += std::complex<float>(sigma * randn(), sigma * randn());
    }
}

// ---------------------------------------------------------------------------
// Random payload generator
// ---------------------------------------------------------------------------
static std::vector<uint8_t> random_payload(int len) {
    std::vector<uint8_t> p(len);
    for (int i = 0; i < len; i++) p[i] = (uint8_t)(rand() & 0xFF);
    return p;
}

// ---------------------------------------------------------------------------
// Preset metadata
// ---------------------------------------------------------------------------
struct PresetInfo {
    uint8_t id;
    const char* name;
};

static const PresetInfo presets[] = {
    { 1, "BPSK r1/2"   },
    { 2, "QPSK r1/2"   },
    { 3, "QPSK r3/4"   },
    { 4, "16QAM r1/2"  },
    { 5, "16QAM r3/4"  },
    { 6, "64QAM r3/4"  },
    { 7, "64QAM r7/8"  },
    { 8, "256QAM r7/8" },
};

// Minimum SNR (dB) where each preset is expected to work in AWGN loopback.
// Below this, failures are informational only, not counted toward pass/fail.
// Includes ~3 dB implementation margin over theoretical limits.
static float preset_min_snr(uint8_t id) {
    switch (id) {
        case 1: return 10;  // BPSK r1/2: detection threshold ~6 dB
        case 2: return 10;  // QPSK r1/2
        case 3: return 10;  // QPSK r3/4
        case 4: return 10;  // 16QAM r1/2
        case 5: return 15;  // 16QAM r3/4
        case 6: return 15;  // 64QAM r3/4
        case 7: return 20;  // 64QAM r7/8
        case 8: return 25;  // 256QAM r7/8
        default: return 10;
    }
}

static const char* preset_name(uint8_t id) {
    for (auto& p : presets) {
        if (p.id == id) return p.name;
    }
    return "???";
}

// ---------------------------------------------------------------------------
// Single trial: TX -> AWGN -> RX.  Returns true if decoded payload matches.
// ---------------------------------------------------------------------------
static bool run_trial(const OfdmConfig& config, uint8_t preset_id,
                      float snr_db, int payload_len, float* est_snr) {
    ToneMap tm = get_uniform_tone_map(preset_id, config);
    if (tm.n_data_carriers == 0) {
        printf("    [ERROR] Invalid tone map for preset %d\n", preset_id);
        *est_snr = 0;
        return false;
    }

    auto payload = random_payload(payload_len);

    OfdmModulator mod(config);
    auto iq = mod.build_ofdm_frame(payload.data(), payload.size(),
                                    tm, tm.fec_rate);
    if (iq.empty()) {
        printf("    [ERROR] build_ofdm_frame returned empty for preset %d, %d bytes\n",
               preset_id, payload_len);
        *est_snr = 0;
        return false;
    }

    add_awgn(iq, snr_db);

    OfdmDemodulator demod(config);
    OfdmDemodResult result = demod.demodulate(iq.data(), (int)iq.size(), &tm);

    *est_snr = result.snr_db;

    if (!result.success) return false;
    if (result.payload.size() != payload.size()) return false;
    return std::memcmp(result.payload.data(), payload.data(), payload.size()) == 0;
}

// ---------------------------------------------------------------------------
// Waterfill trial
// ---------------------------------------------------------------------------
static bool run_waterfill_trial(const OfdmConfig& config, float snr_db,
                                int payload_len, float* est_snr) {
    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    std::vector<float> snr_per_carrier(config.n_data_carriers);
    for (int i = 0; i < config.n_data_carriers; i++) {
        float variation_db = 6.0f * std::sin(2.0f * 3.14159f * i / config.n_data_carriers);
        snr_per_carrier[i] = snr_linear * std::pow(10.0f, variation_db / 10.0f);
    }

    ToneMap tm = compute_waterfill_tone_map(snr_per_carrier, LdpcRate::RATE_3_4, config);
    if (tm.total_bits_per_symbol == 0) {
        printf("    [ERROR] Waterfill produced empty tone map\n");
        *est_snr = 0;
        return false;
    }

    auto payload = random_payload(payload_len);

    OfdmModulator mod(config);
    auto iq = mod.build_ofdm_frame(payload.data(), payload.size(),
                                    tm, tm.fec_rate);
    if (iq.empty()) {
        printf("    [ERROR] build_ofdm_frame returned empty for waterfill\n");
        *est_snr = 0;
        return false;
    }

    add_awgn(iq, snr_db);

    OfdmDemodulator demod(config);
    OfdmDemodResult result = demod.demodulate(iq.data(), (int)iq.size(), &tm);

    *est_snr = result.snr_db;

    if (!result.success) return false;
    if (result.payload.size() != payload.size()) return false;
    return std::memcmp(result.payload.data(), payload.data(), payload.size()) == 0;
}

// ---------------------------------------------------------------------------
// Compute actual throughput from a trial (payload bits / frame duration)
// ---------------------------------------------------------------------------
static float measure_throughput(const OfdmConfig& config, int payload_len, int n_iq_samples) {
    float duration_s = (float)n_iq_samples / config.sample_rate;
    return (duration_s > 0.0f) ? ((float)(payload_len * 8) / duration_s) : 0.0f;
}

// ---------------------------------------------------------------------------
// Run test suite on a config
// ---------------------------------------------------------------------------
static void run_test_suite(const char* label, const OfdmConfig& config,
                           const uint8_t* test_presets, int n_presets,
                           const float* snr_levels, int n_snr,
                           const int* payload_lens, int n_trials,
                           int& total_tests, int& total_pass) {

    printf("=== %s ===\n", label);
    printf("Config: NFFT=%d, CP=%d, BW=%.0f Hz, sym_rate=%.1f, carriers: %d used, %d data, %d pilot, %d hdr syms\n\n",
           config.nfft, config.cp_samples, config.bandwidth_hz,
           config.symbol_rate(),
           config.n_used_carriers, config.n_data_carriers, config.n_pilot_carriers,
           config.n_header_symbols);

    float min_snr[16];
    for (int i = 0; i < n_presets; i++) min_snr[i] = -1.0f;

    for (int pi = 0; pi < n_presets; pi++) {
        uint8_t preset = test_presets[pi];
        printf("Preset %d (%s):\n", preset, preset_name(preset));

        float req_snr = preset_min_snr(preset);

        for (int si = 0; si < n_snr; si++) {
            float snr = snr_levels[si];
            bool below_threshold = (snr < req_snr);
            int pass_count = 0;
            float est_snrs[8];

            for (int t = 0; t < n_trials; t++) {
                bool ok = run_trial(config, preset, snr, payload_lens[t], &est_snrs[t]);
                if (ok) pass_count++;
                if (!below_threshold) total_tests++;
            }

            if (!below_threshold) total_pass += pass_count;

            if (pass_count == n_trials && min_snr[pi] < 0)
                min_snr[pi] = snr;

            const char* status;
            if (pass_count == n_trials) status = "PASS";
            else if (below_threshold) status = "EXPECTED";
            else status = "FAIL";

            printf("  SNR=%2.0f dB: %d/%d %s (est SNR:",
                   snr, pass_count, n_trials, status);
            for (int t = 0; t < n_trials; t++) {
                printf(" %.1f", est_snrs[t]);
                if (t < n_trials - 1) printf("/");
            }
            printf(" dB)\n");
        }
        printf("\n");
    }

    // Summary for this suite
    printf("Minimum SNR for all-pass on each preset:\n");
    for (int pi = 0; pi < n_presets; pi++) {
        uint8_t preset = test_presets[pi];

        // Compute estimated throughput for multi-codeword frames
        ToneMap tm = get_uniform_tone_map(preset, config);
        int k = LdpcCodec::block_size(tm.fec_rate);
        int max_payload_bytes = payload_lens[n_trials - 1];
        int n_cw = ((max_payload_bytes + 4) * 8 + k - 1) / k;

        float tput_1cw = tone_map_throughput(tm, config, 1);
        float tput_ncw = tone_map_throughput(tm, config, n_cw);

        if (min_snr[pi] > 0)
            printf("  Preset %d (%-14s): %2.0f dB  throughput: %5.0f bps (1cw) / %5.0f bps (%dcw)\n",
                   preset, preset_name(preset), min_snr[pi], tput_1cw, tput_ncw, n_cw);
        else
            printf("  Preset %d (%-14s): >40 dB\n", preset, preset_name(preset));
    }
    printf("\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    srand(42);

    int total_tests = 0;
    int total_pass = 0;

    // =====================================================================
    //  Part 1: HF config (2 kHz, CP=64, dense pilots)
    // =====================================================================
    {
        NegotiatedPassband pb;
        pb.low_hz = 700.0f;
        pb.high_hz = 2700.0f;
        pb.center_hz = 1700.0f;
        pb.bandwidth_hz = 2000.0f;
        pb.valid = true;

        OfdmConfig config = ofdm_config_from_probe(pb, 512, 64, 4, 8);

        static const uint8_t test_presets[] = { 1, 2, 4, 5, 6, 8 };
        static const float snr_levels[] = { 5, 10, 15, 20, 25, 30, 40 };
        static const int payload_lens[] = { 32, 64, 128 };

        run_test_suite("HF Config (2 kHz, CP=64, pilot 1:4)",
                       config, test_presets, 6, snr_levels, 7,
                       payload_lens, 3, total_tests, total_pass);

        // Waterfill test
        printf("Waterfill (r3/4 @ 20 dB, sinusoidal SNR variation):\n");
        {
            int wf_pass = 0;
            float wf_est[3];
            for (int t = 0; t < 3; t++) {
                bool ok = run_waterfill_trial(config, 20.0f, payload_lens[t], &wf_est[t]);
                if (ok) wf_pass++;
                total_tests++;
            }
            total_pass += wf_pass;
            printf("  SNR=20 dB: %d/%d %s\n\n", wf_pass, 3,
                   wf_pass == 3 ? "PASS" : "FAIL");
        }
    }

    // =====================================================================
    //  Part 2: FM config (5 kHz, CP=16, sparse pilots)
    // =====================================================================
    {
        NegotiatedPassband pb;
        pb.low_hz = 300.0f;
        pb.high_hz = 5300.0f;
        pb.center_hz = 2800.0f;
        pb.bandwidth_hz = 5000.0f;
        pb.valid = true;

        // FM-optimized: CP=16, pilot every 8th carrier, block pilots every 16th symbol
        OfdmConfig config = ofdm_config_from_probe(pb, 512, 16, 8, 16);

        // FM tests: higher-order presets with multi-codeword payloads
        static const uint8_t test_presets[] = { 2, 4, 6, 7, 8 };
        static const float snr_levels[] = { 10, 15, 20, 25, 30, 40 };
        // Large payloads: these will naturally fill multiple LDPC codewords
        static const int payload_lens[] = { 128, 512, 1024 };

        run_test_suite("FM Config (5 kHz, CP=16, pilot 1:16)",
                       config, test_presets, 5, snr_levels, 6,
                       payload_lens, 3, total_tests, total_pass);

        // Waterfill with large payload
        printf("Waterfill FM (r3/4 @ 25 dB, 1024 bytes):\n");
        {
            float wf_est;
            bool ok = run_waterfill_trial(config, 25.0f, 1024, &wf_est);
            if (ok) total_pass++;
            total_tests++;
            printf("  SNR=25 dB: %s\n\n", ok ? "PASS" : "FAIL");
        }
    }

    // =====================================================================
    //  Part 3: FM Wide config (6 kHz, CP=16, sparse pilots)
    // =====================================================================
    {
        NegotiatedPassband pb;
        pb.low_hz = 200.0f;
        pb.high_hz = 6200.0f;
        pb.center_hz = 3200.0f;
        pb.bandwidth_hz = 6000.0f;
        pb.valid = true;

        OfdmConfig config = ofdm_config_from_probe(pb, 512, 16, 8, 16);

        static const uint8_t test_presets[] = { 4, 6, 7, 8 };
        static const float snr_levels[] = { 15, 20, 25, 30, 40 };
        static const int payload_lens[] = { 256, 512, 1024 };

        run_test_suite("FM Wide Config (6 kHz, CP=16, pilot 1:8)",
                       config, test_presets, 4, snr_levels, 5,
                       payload_lens, 3, total_tests, total_pass);
    }

    // =====================================================================
    //  Part 4: Large-payload (streaming approximation) test
    // =====================================================================
    printf("=== Streaming Throughput Test (FM Wide 6 kHz, large payloads) ===\n");
    {
        NegotiatedPassband pb;
        pb.low_hz = 200.0f;
        pb.high_hz = 6200.0f;
        pb.center_hz = 3200.0f;
        pb.bandwidth_hz = 6000.0f;
        pb.valid = true;

        OfdmConfig config = ofdm_config_from_probe(pb, 512, 16, 8, 16);

        // Test large payloads at 30 dB (safe for 256QAM r7/8)
        static const int large_payloads[] = { 2048, 4096, 8192 };
        static const uint8_t stream_presets[] = { 6, 7, 8 };
        static const float stream_snrs[] = { 20, 30 };

        for (uint8_t preset : stream_presets) {
            ToneMap tm = get_uniform_tone_map(preset, config);
            float req_snr = preset_min_snr(preset);
            printf("  %s:\n", preset_name(preset));
            for (float snr : stream_snrs) {
                for (int plen : large_payloads) {
                    float est_snr;
                    bool ok = run_trial(config, preset, snr, plen, &est_snr);
                    bool below_threshold = (snr < req_snr);
                    if (!below_threshold) {
                        total_tests++;
                        if (ok) total_pass++;
                    }

                    // Compute actual N codewords for this payload
                    int data_bits_per_block;
                    switch (preset) {
                        case 6: data_bits_per_block = 1200; break;  // r3/4
                        case 7: case 8: data_bits_per_block = 1400; break;  // r7/8
                        default: data_bits_per_block = 800; break;
                    }
                    int n_cw = (plen * 8 + data_bits_per_block - 1) / data_bits_per_block;
                    float tput = tone_map_throughput(tm, config, n_cw);

                    const char* s = ok ? "PASS" : (below_threshold ? "EXPECTED" : "FAIL");
                    printf("    SNR=%2.0f dB, %5d bytes (%2d CW): %s  -> %.0f bps\n",
                           snr, plen, n_cw, s, tput);
                }
            }
        }
        printf("\n");
    }

    // =====================================================================
    //  Throughput comparison table
    // =====================================================================
    printf("=== Throughput Comparison (256QAM r7/8, estimated) ===\n");
    printf("%-35s %6s %5s %5s %7s %7s %7s %7s %7s\n",
           "Config", "Data", "HdrS", "SymR", "1-CW", "8-CW", "20-CW", "50-CW", "Stream");

    struct ConfigDef {
        const char* name;
        float low, high, center, bw;
        int cp, pilot_sp, blk_pilot_sp;
    };

    static const ConfigDef configs[] = {
        { "HF 2kHz (CP=64 pilot=1:4)",   700, 2700, 1700, 2000, 64, 4, 8  },
        { "FM 5kHz (CP=16 pilot=1:8)",    300, 5300, 2800, 5000, 16, 8, 16 },
        { "FM 6kHz (CP=16 pilot=1:8)",    200, 6200, 3200, 6000, 16, 8, 16 },
        { "FM 6kHz (CP=8  pilot=1:8)",    200, 6200, 3200, 6000,  8, 8, 16 },
    };

    for (auto& cd : configs) {
        NegotiatedPassband pb;
        pb.low_hz = cd.low;
        pb.high_hz = cd.high;
        pb.center_hz = cd.center;
        pb.bandwidth_hz = cd.bw;
        pb.valid = true;

        OfdmConfig cfg = ofdm_config_from_probe(pb, 512, cd.cp, cd.pilot_sp, cd.blk_pilot_sp);
        ToneMap tm = get_uniform_tone_map(8, cfg); // preset 8 = 256QAM r7/8

        float t1 = tone_map_throughput(tm, cfg, 1);
        float t8 = tone_map_throughput(tm, cfg, 8);
        float t20 = tone_map_throughput(tm, cfg, 20);
        float t50 = tone_map_throughput(tm, cfg, 50);

        // Streaming ceiling: only block pilot overhead, no preamble/header/tail
        // 256QAM r7/8 → code rate = 7/8 = 0.875
        float rate = 0.875f;
        float pilot_eff = (float)cfg.pilot_symbol_spacing / (cfg.pilot_symbol_spacing + 1.0f);
        float stream = (float)tm.total_bits_per_symbol * cfg.symbol_rate() * pilot_eff * rate;

        printf("%-35s %4d  %3d   %4.0f  %6.0f  %6.0f  %6.0f  %6.0f  %6.0f\n",
               cd.name, cfg.n_data_carriers, cfg.n_header_symbols,
               cfg.symbol_rate(), t1, t8, t20, t50, stream);
    }

    printf("\nVARA FM Wide peak: 25,210 bps (level 13, 256QAM, 116 carriers)\n");
    printf("VARA FM Narrow peak: 12,750 bps (level 11, 256QAM, 58 carriers)\n\n");

    // =====================================================================
    //  Overall summary
    // =====================================================================
    printf("=== Overall Summary ===\n");
    printf("Total: %d tests, %d PASS, %d FAIL (%.1f%%)\n",
           total_tests, total_pass, total_tests - total_pass,
           100.0f * total_pass / total_tests);

    return (total_pass == total_tests) ? 0 : 1;
}
