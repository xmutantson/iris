// ============================================================================
//  S2 estimator/EQ gate harness (decode-only, reproducible, identical audio)
//
//  Proves the RX-only continuous channel-tracking fix (E1 per-carrier H update
//  at pilot rows, with drift-free linear-phase detrend; E2 frequency smoothing;
//  E3 per-symbol comb-pilot amplitude refinement) WITHOUT reintroducing the S1
//  long-QPSK phase fold. For each (level, channel-condition, seed) it generates
//  the passband/IQ ONCE, then decodes that same buffer under each E-toggle
//  configuration. Audio generation is toggle-independent, so before/after is
//  fail-before / pass-after on the same samples.
//
//  E-toggles are read per-frame inside OfdmDemodulator::demodulate() from the
//  environment (IRIS_S2_E1/E2/E3, IRIS_S2_LINDET, IRIS_S2_ALPHA). This file
//  flips them via setenv between configs.
//
//  Columns:
//    baseline   E1=0            — frozen H (STEP-1 measurement; current shipped)
//    E1cpe      E1=1,LINDET=0   — scalar-CPE-only update (the prior S2 / the S1
//                                 fold: expected to BREAK long QPSK at zero noise)
//    E1         E1=1,LINDET=1   — drift-free per-carrier H update
//    E1+E2      + freq smoothing
//    E1E2E3     + comb amplitude refine
//
//  Invoke: iris --s2gate   (NOT part of --test; keeps the check-count intact)
//  Run with stdout discarded (IRIS_LOG floods it): iris --s2gate 1>/dev/null
// ============================================================================

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"
#include "probe/passband_probe.h"
#include "common/fft.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <vector>
#include <complex>
#include <random>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace iris;

// IRIS_LOG writes unconditionally to stdout; route this harness's own output to
// stderr so the caller can discard the per-symbol decode logs (1>/dev/null).
#define printf(...) fprintf(stderr, __VA_ARGS__)

namespace {

bool set_toggle(const char* name, const char* val) {
#ifdef _WIN32
    return _putenv_s(name, val ? val : "") == 0;
#else
    return val ? setenv(name, val, 1) == 0 : unsetenv(name) == 0;
#endif
}

bool parse_gate_int(const char* name, int min_value, int max_value, int& value) {
    const char* text = getenv(name);
    if (!text) return true;
    char* end = nullptr;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0' ||
        parsed < min_value || parsed > max_value) {
        printf("invalid %s: %s\n", name, text);
        return false;
    }
    value = (int)parsed;
    return true;
}

// FFT-based analytic signal (Hilbert), mirrors tests.cc hilbert_analytic.
std::vector<std::complex<float>> analytic(const float* audio, int n) {
    int nfft_h = 1;
    while (nfft_h < n) nfft_h <<= 1;
    std::vector<std::complex<float>> buf(nfft_h, {0.0f, 0.0f});
    for (int i = 0; i < n; i++) buf[i] = std::complex<float>(audio[i], 0.0f);
    fft_complex(buf.data(), nfft_h);
    for (int k = 1; k < nfft_h / 2; k++) buf[k] *= 2.0f;
    for (int k = nfft_h / 2 + 1; k < nfft_h; k++) buf[k] = {0.0f, 0.0f};
    ifft_complex(buf.data(), nfft_h);
    std::vector<std::complex<float>> result(n);
    for (int i = 0; i < n; i++) result[i] = buf[i];
    return result;
}

// Parameterized, seeded FM channel. Mirrors tests.cc fm_channel_process
// (pre-emphasis 530us + dev limiter + oscillator drift + f^2 discriminator
// noise + de-emphasis) but adds a per-call SEED, an optional 2-tap static echo
// (frequency-selective |H|) and a slow flat fade (time-varying amplitude) — the
// "benign multipath fading" a frozen-H estimator cannot follow.
void fm_channel(float* audio, int n,
                float noise_amplitude, unsigned seed,
                int echo_delay, float echo_gain,
                float fade_depth, float fade_hz,
                float fs = 48000.0f, float freq_diffusion = 3.0f) {
    const float tau_s = 530e-6f;
    const float tau2_s = 1.0f / (2.0f * (float)M_PI * 15000.0f);
    const float c1 = 2.0f * fs * tau_s;
    const float c2 = 2.0f * fs * tau2_s;
    const float pe_a0 = 1.0f + c2;
    const float pe_b0 = (1.0f + c1) / pe_a0;
    const float pe_b1 = (1.0f - c1) / pe_a0;
    const float pe_a1 = (1.0f - c2) / pe_a0;

    // Pass 1 pre-emphasis (for peak normalization)
    std::vector<float> pe_audio(n);
    float pe_x1 = 0.0f, pe_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = audio[i];
        pe_audio[i] = pe_b0 * x + pe_b1 * pe_x1 - pe_a1 * pe_y1;
        pe_x1 = x; pe_y1 = pe_audio[i];
    }
    float pe_peak = 0;
    for (int i = 0; i < n; i++) if (std::abs(pe_audio[i]) > pe_peak) pe_peak = std::abs(pe_audio[i]);
    const float dev_limit = 0.95f;
    const float target_peak = 0.50f;
    if (pe_peak > 0) {
        float norm = (target_peak * dev_limit) / pe_peak;
        for (int i = 0; i < n; i++) audio[i] *= norm;
    }
    // Re-apply pre-emphasis with correct scaling
    pe_x1 = 0.0f; pe_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = audio[i];
        audio[i] = pe_b0 * x + pe_b1 * pe_x1 - pe_a1 * pe_y1;
        pe_x1 = x; pe_y1 = audio[i];
    }
    // Deviation limiter
    for (int i = 0; i < n; i++) {
        if (audio[i] > dev_limit) audio[i] = dev_limit;
        if (audio[i] < -dev_limit) audio[i] = -dev_limit;
    }

    // Oscillator frequency drift (Wiener process), seeded per-call.
    {
        std::mt19937 pn_rng(1000u + seed);
        float dt = 1.0f / fs;
        float freq_step_std = freq_diffusion * std::sqrt(dt);
        std::normal_distribution<float> pn_dist(0.0f, freq_step_std);
        std::vector<float> phase_traj(n);
        float freq_offset = 0.0f, phase_accum = 0.0f;
        for (int i = 0; i < n; i++) {
            freq_offset += pn_dist(pn_rng);
            phase_accum += 2.0f * (float)M_PI * freq_offset * dt;
            phase_traj[i] = phase_accum;
        }
        auto a = analytic(audio, n);
        for (int i = 0; i < n; i++) {
            std::complex<float> rot(std::cos(phase_traj[i]), std::sin(phase_traj[i]));
            audio[i] = std::real(a[i] * rot);
        }
    }

    // Optional 2-tap static echo (frequency-selective |H|). Delay within the
    // OFDM CP, so it is ISI-free and captured by H[k].
    if (echo_delay > 0 && echo_gain > 0.0f) {
        std::vector<float> src(audio, audio + n);
        for (int i = 0; i < n; i++) {
            float e = (i >= echo_delay) ? echo_gain * src[i - echo_delay] : 0.0f;
            audio[i] = src[i] + e;
        }
    }

    // Optional slow flat fade (time-varying amplitude): the benign fading that a
    // frozen preamble |H| cannot track. Sinusoidal envelope, phase seeded.
    if (fade_depth > 0.0f && fade_hz > 0.0f) {
        std::mt19937 fr(2000u + seed);
        std::uniform_real_distribution<float> ph(0.0f, 2.0f * (float)M_PI);
        float phi0 = ph(fr);
        float dt = 1.0f / fs;
        for (int i = 0; i < n; i++) {
            float env = 1.0f + fade_depth * std::sin(2.0f * (float)M_PI * fade_hz * i * dt + phi0);
            audio[i] *= env;
        }
    }

    // f^2-shaped FM discriminator noise, seeded per-call.
    if (noise_amplitude > 0) {
        std::mt19937 rng(3000u + seed);
        std::normal_distribution<float> dist(0.0f, noise_amplitude);
        float prev_noise = 0.0f;
        for (int i = 0; i < n; i++) {
            float wn = dist(rng);
            float shaped = wn - prev_noise;
            prev_noise = wn;
            audio[i] += shaped;
        }
    }

    // De-emphasis
    const float wc = 1.0f / tau_s;
    const float K = 2.0f * fs;
    const float a = K + wc;
    const float de_b0 = wc / a, de_b1 = wc / a, de_a1 = (wc - K) / a;
    float de_x1 = 0.0f, de_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float de = de_b0 * audio[i] + de_b1 * de_x1 - de_a1 * de_y1;
        de_x1 = audio[i]; de_y1 = de; audio[i] = de;
    }
}

struct Level { int preset; const char* name; LdpcRate fec; int ncw; int payload; };
// noise_amp < -1.5 => CLEAN direct-IQ (no channel, no Hilbert) — mirrors the S1
//                     regression test (test_ofdm_longframe_phase).
// noise_amp in [-1.5,0) (i.e. -1.0) => bypass FM, direct passband via Hilbert.
// noise_amp >= 0 => full FM channel with drift/echo/fade/noise.
struct Cond  { const char* name; float noise_amp; int echo_delay; float echo_gain; float fade_depth; float fade_hz; float drift; };
struct Cfg   { const char* label; int e1, e2, e3, lindet; };

} // namespace

int run_s2_gate() {
    struct SavedToggle {
        const char* name;
        bool present;
        std::string value;
    };
    SavedToggle saved[] = {
        { "IRIS_S2_E1", false, "" }, { "IRIS_S2_E2", false, "" },
        { "IRIS_S2_E3", false, "" }, { "IRIS_S2_LINDET", false, "" },
    };
    for (auto& toggle : saved) {
        if (const char* value = getenv(toggle.name)) {
            toggle.present = true;
            toggle.value = value;
        }
    }
    auto restore_toggles = [&]() {
        bool ok = true;
        for (auto& toggle : saved)
            ok = set_toggle(toggle.name, toggle.present ? toggle.value.c_str() : nullptr) && ok;
        return ok;
    };

    printf("=== S2 estimator/EQ gate (decode-only, identical audio) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    // Same geometry as the S1 regression test test_ofdm_longframe_phase.
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);
    printf("cfg: nfft=%d cp=%d used=%d data=%d comb_sp=%d block_sp=%d row_sp=%d\n",
           cfg.nfft, cfg.cp_samples, cfg.n_used_carriers, cfg.n_data_carriers,
           cfg.pilot_carrier_spacing, cfg.pilot_symbol_spacing, cfg.pilot_row_spacing);

    std::vector<Level> levels = {
        // Regression guards: long QPSK/BPSK frames (ncw8 = 131 syms, well past
        // the S1 ~sym-40 drift threshold). Must stay 100% clean under E1.
        { 1,  "O0 BPSK  r1/2 ncw8", LdpcRate::RATE_1_2, 8, 476 },
        { 2,  "O1 QPSK  r1/2 ncw8", LdpcRate::RATE_1_2, 8, 476 },
        { 3,  "O2 QPSK  r3/4 ncw8", LdpcRate::RATE_3_4, 8, 476 },
        // Shed targets: 16QAM under benign fading.
        { 4,  "O3 16QAM r1/2 ncw8", LdpcRate::RATE_1_2, 8, 400 },
        { 5,  "O4 16QAM r5/8 ncw8", LdpcRate::RATE_5_8, 8, 400 },
        { 6,  "O5 16QAM r3/4 ncw8", LdpcRate::RATE_3_4, 8, 400 },
        // Out-of-scope confirm (max_bpc>5 => S2 inert): must be UNCHANGED.
        // preset 11 = O10 256QAM r3/4 after the 32QAM-rung renumber.
        {11,  "O10 256QAM r3/4 ncw4",LdpcRate::RATE_3_4, 4, 476 },
    };

    std::vector<Cond> conds = {
        { "CLEAN-IQ",  -2.0f,  0, 0.0f,  0.0f, 0.0f, 0.0f },  // regression guard, zero noise
        { "WGN40",      0.001f, 0, 0.0f,  0.0f, 0.0f, 3.0f },
        { "WGN30",      0.007f, 0, 0.0f,  0.0f, 0.0f, 3.0f },
        { "MPG40",      0.001f, 24, 0.40f, 0.25f, 1.5f, 3.0f },
        { "MPG30",      0.006f, 24, 0.40f, 0.25f, 1.5f, 3.0f },
    };

    std::vector<Cfg> cfgs = {
        { "baseline", 0, 0, 0, 1 },
        { "E3only",   0, 0, 1, 1 },   // comb amplitude vs frozen H (fade lever)
        { "E1cpe",    1, 0, 0, 0 },
        { "E1",       1, 0, 0, 1 },
        { "E1+E2",    1, 1, 0, 1 },
        { "E1E2E3",   1, 1, 1, 1 },
    };

    const int N = 96;  // Monte-Carlo trials per cell
    std::vector<uint8_t> payload(4096);
    for (int i = 0; i < 4096; i++) payload[i] = (uint8_t)(i * 37 + 13);
    bool failed = false, abort = false;
    int total_pass = 0;

    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        tm.n_codewords = lv.ncw;

        OfdmModulator mod(cfg);
        const int pbytes = lv.payload;
        std::vector<std::complex<float>> iq =
            mod.build_ofdm_frame(payload.data(), pbytes, tm, lv.fec, lv.ncw);
        if (iq.empty()) {
            printf("[%s] FAIL (cannot build payload=%d)\n", lv.name, pbytes);
            failed = true;
            continue;
        }

        printf("\n### %-18s payload=%d ncw=%d frame_samples=%d\n",
               lv.name, pbytes, lv.ncw, (int)iq.size());
        printf("  %-10s", "cond");
        for (auto& c : cfgs) printf(" | %-8s", c.label);
        printf(" | ~chSNR\n");

        for (auto& cond : conds) {
            // Pre-generate N identical RX buffers (seed = trial index).
            std::vector<std::vector<std::complex<float>>> rx_bufs(N);
            std::vector<OfdmSyncResult> syncs(N);
            std::vector<bool> detected(N, false);
            for (int t = 0; t < N; t++) {
                std::vector<std::complex<float>> rx;
                if (cond.noise_amp < -1.5f) {
                    // CLEAN direct-IQ: peak-normalize to 0.5, no channel, no Hilbert
                    // (exactly mirrors the S1 regression test).
                    rx.assign(iq.begin(), iq.end());
                    float peak = 1e-9f;
                    for (auto& s : rx) peak = std::max(peak, std::abs(s.real()));
                    float g = 0.5f / peak;
                    for (auto& s : rx) s *= g;
                } else {
                    std::vector<float> aud(iq.size());
                    for (size_t i = 0; i < iq.size(); i++) aud[i] = iq[i].real();
                    if (cond.noise_amp >= 0.0f)
                        fm_channel(aud.data(), (int)aud.size(), cond.noise_amp, (unsigned)t,
                                   cond.echo_delay, cond.echo_gain, cond.fade_depth, cond.fade_hz,
                                   48000.0f, cond.drift);
                    rx = analytic(aud.data(), (int)aud.size());
                }
                size_t pad = 48000;
                rx.insert(rx.begin(), pad, std::complex<float>(0, 0));
                rx.insert(rx.end(), pad, std::complex<float>(0, 0));
                auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
                detected[t] = sync.detected;
                syncs[t] = sync;
                rx_bufs[t] = std::move(rx);
            }

            printf("  %-10s", cond.name);
            float ch_snr_report = 0.0f;
            for (auto& c : cfgs) {
                if (!set_toggle("IRIS_S2_E1", c.e1 ? "1" : "0") ||
                    !set_toggle("IRIS_S2_E2", c.e2 ? "1" : "0") ||
                    !set_toggle("IRIS_S2_E3", c.e3 ? "1" : "0") ||
                    !set_toggle("IRIS_S2_LINDET", c.lindet ? "1" : "0")) {
                    printf("\nfailed to set S2 toggles\n");
                    failed = abort = true;
                    break;
                }
                int pass = 0;
                float snr_sum = 0.0f; int snr_n = 0;
                for (int t = 0; t < N; t++) {
                    if (!detected[t]) continue;
                    OfdmDemodulator demod(cfg);
                    auto r = demod.demodulate(rx_bufs[t].data(), (int)rx_bufs[t].size(),
                                              tm, &syncs[t]);
                    bool ok = r.success && r.payload.size() == (size_t)pbytes &&
                              memcmp(r.payload.data(), payload.data(), pbytes) == 0;
                    if (ok) pass++;
                    if (r.mean_channel_snr_db > 0) { snr_sum += r.mean_channel_snr_db; snr_n++; }
                }
                total_pass += pass;
                if (snr_n > 0) ch_snr_report = snr_sum / snr_n;
                printf(" | %6.1f%% ", 100.0f * pass / N);
            }
            if (abort) break;
            printf(" | %.1f dB\n", ch_snr_report);
        }
        if (abort) break;
    }

    if (!restore_toggles()) failed = true;

    printf("\n=== S2 gate done ===\n");
    return failed || total_pass == 0 ? 1 : 0;
}

// ============================================================================
//  32QAM r5/8 rung gate (INCREMENT 1 + pilot-diet A/B for INCREMENT 2)
//
//  Forced-level decode of the NEW 32QAM r5/8 rung (O6) through the same faithful
//  FM channel + drift/echo/fade model as the S2 gate, identical-audio Monte
//  Carlo. Anchors on 16QAM r3/4 (O5) so the S2 cross-layer extension (16QAM +
//  32QAM both tracked) is visible side-by-side, and includes 64QAM r5/8 (O7) as
//  the next rung up for reference.
//
//  Comb pilot spacing is read from IRIS_GATE32_COMB (default 12) so INCREMENT 2's
//  pilot diet (49 -> more data carriers) can be A/B'd on ONE binary against the
//  SAME channel model. N per cell from IRIS_GATE32_N (default 96).
//
//  Invoke: iris --gate32 1>/dev/null   (IRIS_LOG floods stdout)
// ============================================================================
int run_32qam_gate() {
    int comb = 12;  // production default after the pilot diet (52 data carriers)
    int N = 96;
    if (!parse_gate_int("IRIS_GATE32_COMB", 2, 40, comb) ||
        !parse_gate_int("IRIS_GATE32_N", 8, 4096, N)) return 1;

    printf("=== 32QAM r5/8 rung gate (forced-level, identical audio) ===\n");
    printf("comb_spacing=%d  N=%d\n", comb, N);

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, comb, 24);
    printf("cfg: nfft=%d cp=%d used=%d data=%d comb_sp=%d block_sp=%d\n",
           cfg.nfft, cfg.cp_samples, cfg.n_used_carriers, cfg.n_data_carriers,
           cfg.pilot_carrier_spacing, cfg.pilot_symbol_spacing);

    float baud = cfg.symbol_rate();

    // preset, name, fec, ncw, payload, net_num, net_den (for wire-bps math)
    struct GLevel { int preset; const char* name; LdpcRate fec; int ncw; int payload;
                    int bpc; int rn; int rd; };
    std::vector<GLevel> levels = {
        { 6, "O5 16QAM r3/4", LdpcRate::RATE_3_4, 8, 400, 4, 3, 4 },  // S2 anchor
        { 7, "O6 32QAM r5/8", LdpcRate::RATE_5_8, 8, 400, 5, 5, 8 },  // NEW rung (target)
        { 8, "O7 64QAM r5/8", LdpcRate::RATE_5_8, 4, 476, 6, 5, 8 },  // reference (next up)
    };

    std::vector<Cond> conds = {
        { "CLEAN-IQ", -2.0f,   0, 0.0f,  0.0f,  0.0f, 0.0f },
        { "WGN40",     0.001f, 0, 0.0f,  0.0f,  0.0f, 3.0f },
        { "WGN30",     0.007f, 0, 0.0f,  0.0f,  0.0f, 3.0f },
        { "MPG40",     0.001f, 24, 0.40f, 0.25f, 1.5f, 3.0f },
        { "MPG30",     0.006f, 24, 0.40f, 0.25f, 1.5f, 3.0f },
    };

    std::vector<uint8_t> payload(4096);
    for (int i = 0; i < 4096; i++) payload[i] = (uint8_t)(i * 37 + 13);

    // Defaults ON (production): S2 tracking active for 16QAM + 32QAM.
    if (!set_toggle("IRIS_S2_E1", "1") || !set_toggle("IRIS_S2_E2", "1") ||
        !set_toggle("IRIS_S2_E3", "1") || !set_toggle("IRIS_S2_LINDET", "1")) {
        printf("failed to set S2 toggles\n");
        return 1;
    }
    int total_pass = 0;

    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        tm.n_codewords = lv.ncw;

        // Wire-bps math (at this carrier count):
        //   gross = n_data * bpc * (rn/rd) * baud
        //   eff   = tone_map_throughput (accounts for pilot rows + preamble)
        float gross = cfg.n_data_carriers * lv.bpc * ((float)lv.rn / lv.rd) * baud;
        float eff   = tone_map_throughput(tm, cfg, lv.ncw);

        OfdmModulator mod(cfg);
        int pbytes = lv.payload;
        std::vector<std::complex<float>> iq;
        for (int attempt = 0; attempt < 5; attempt++) {
            iq = mod.build_ofdm_frame(payload.data(), pbytes, tm, lv.fec, lv.ncw);
            if (!iq.empty()) break;
            pbytes = pbytes * 3 / 4;
        }
        if (iq.empty()) { printf("[%s] SKIP (no frame)\n", lv.name); continue; }

        printf("\n### %-16s carriers=%d net_bpc=%.3f gross=%.0f eff=%.0f bps  payload=%d ncw=%d\n",
               lv.name, cfg.n_data_carriers, lv.bpc * (float)lv.rn / lv.rd, gross, eff, pbytes, lv.ncw);
        printf("  %-10s | decode%% | ~chSNR\n", "cond");

        for (auto& cond : conds) {
            int pass = 0, det = 0;
            float snr_sum = 0.0f; int snr_n = 0;
            for (int t = 0; t < N; t++) {
                std::vector<std::complex<float>> rx;
                if (cond.noise_amp < -1.5f) {
                    rx.assign(iq.begin(), iq.end());
                    float peak = 1e-9f;
                    for (auto& s : rx) peak = std::max(peak, std::abs(s.real()));
                    float g = 0.5f / peak;
                    for (auto& s : rx) s *= g;
                } else {
                    std::vector<float> aud(iq.size());
                    for (size_t i = 0; i < iq.size(); i++) aud[i] = iq[i].real();
                    if (cond.noise_amp >= 0.0f)
                        fm_channel(aud.data(), (int)aud.size(), cond.noise_amp, (unsigned)t,
                                   cond.echo_delay, cond.echo_gain, cond.fade_depth, cond.fade_hz,
                                   48000.0f, cond.drift);
                    rx = analytic(aud.data(), (int)aud.size());
                }
                size_t pad = 48000;
                rx.insert(rx.begin(), pad, std::complex<float>(0, 0));
                rx.insert(rx.end(), pad, std::complex<float>(0, 0));
                auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
                if (!sync.detected) continue;
                det++;
                OfdmDemodulator demod(cfg);
                auto r = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
                bool ok = r.success && r.payload.size() == (size_t)pbytes &&
                          memcmp(r.payload.data(), payload.data(), pbytes) == 0;
                if (ok) pass++;
                if (r.mean_channel_snr_db > 0) { snr_sum += r.mean_channel_snr_db; snr_n++; }
            }
            total_pass += pass;
            printf("  %-10s | %6.1f%% (%d/%d, det %d) | %.1f dB\n",
                   cond.name, 100.0f * pass / N, pass, N, det,
                   snr_n > 0 ? snr_sum / snr_n : 0.0f);
        }
    }

    printf("\n=== 32QAM gate done ===\n");
    return total_pass == 0 ? 1 : 0;
}

// ============================================================================
//  WIDE-6K flat-port forced-level decode matrix (dominance PHY)
//
//  Stands up the 6 kHz OFDM profile (wide6k_passband(): 300-6300 Hz, ~112 data
//  carriers vs narrow's ~52 — a pure config, same nfft/cp/pilot structure) and a
//  CALIBRATED FLAT 9600-baud DATA-PORT channel, then forced-level decodes the
//  whole O-ladder (BPSK..256QAM) through it. This is the port VARA FM WIDE rides
//  (its wide bar = 2.65x its narrow); the question it answers is which QAM rungs
//  — especially 64QAM (O7)/256QAM (O9-O11) that the narrow emphasis+limiter port
//  cannot carry — decode on the flat port, and the gross wire bps each yields.
//
//  FLAT-PORT CHANNEL MODEL — calibration basis (honest; see FM_SIM_CALIBRATION.md
//  and G3RUH "9600 Baud Packet Radio Modem Design", amsat.org/.../g3ruh/109.html):
//    * SIGNAL PATH IS FLAT. The 9600 data port applies audio DIRECT to the TX
//      varactor (no pre-emphasis, no mic audio processing) and taps the RX
//      DISCRIMINATOR output directly (no de-emphasis). So — unlike the narrow
//      voice port — there is NO pre/de-emphasis and NO syllabic deviation
//      limiter. At a managed drive the port does not clip (hot-drive clip is a
//      real but separate, IONOS-measurable effect). => the emphasis EVM cliff
//      that caps the narrow port at 32QAM is ABSENT.
//    * NOISE IS THE RAW FM DISCRIMINATOR OUTPUT: white -> first-difference
//      differentiator => f^2 ("triangular") PSD (Rice/Carlson/Haykin/Taub &
//      Schilling; f^2 confirmed in FM_SIM_CALIBRATION §4.4). Crucially, with NO
//      de-emphasis to tame it, the noise RISES across the band, so per-carrier
//      S/N falls ~1/f^2 — the honest deviation from a naive "flat AWGN" model
//      (which IRIS_WIDE_FLATNOISE=1 selects as an optimistic sensitivity bound).
//    * BAND LIMIT = the port's occupancy filter: G3RUH's RX "3rd order
//      Butterworth, 6 kHz", plus a 150 Hz high-pass for the AC-coupled LF skirt
//      ("response down to a few Hz"). Applied to signal+noise alike, NO de-emph.
//    * CALIBRATION: noise sigma set so total in-band signal power / total in-band
//      noise power = the target S:N (the VARA-sheet "S:N in <BW>" convention,
//      matching FM_SIM_CALIBRATION §3). Exact by construction (linear); the
//      realized-S:N column reports it. Because the f^2 noise integral is
//      dominated by the top of the band, this band-average S:N sits only a few dB
//      above the worst (top) carrier.
//    * Oscillator drift (Wiener, 3 Hz/rt-s) + optional 2-tap echo + slow flat
//      fade (a VHF multipath PROXY — real VHF FM multipath differs from HF CCIR;
//      the MPG cell is indicative, IONOS-gated for the absolute figure).
//
//  HONESTY BOUND: the model STRUCTURE (flat signal, f^2 noise, ~6.3 kHz occupancy,
//  band-edge rolloff) is grounded in G3RUH's canonical design; the ABSOLUTE dB
//  (a given radio's discriminator noise floor, group-delay ripple, deviation) is
//  radio-specific and NOT knowable from docs => the RELATIVE per-level structure
//  here is believable, the exact crossover dB is IONOS-GATED.
//
//  Invoke: iris --wide6k 1>/dev/null   (IRIS_LOG floods stdout)
//  Env: IRIS_WIDE_N (trials/cell, default 64), IRIS_WIDE_COMB (pilot spacing,
//       default 12), IRIS_WIDE_FLATNOISE=1 (flat AWGN instead of f^2, optimistic).
// ============================================================================

namespace {

struct Biquad2 {
    double b0=1,b1=0,b2=0,a1=0,a2=0,z1=0,z2=0;
    inline float process(float x){
        double w=(double)x - a1*z1 - a2*z2;
        double y=b0*w + b1*z1 + b2*z2;
        z2=z1; z1=w; return (float)y;
    }
};
static Biquad2 make_lp1(double fc,double fs){          // 1st-order LP (bilinear)
    double wc=2.0*M_PI*fc, K=2.0*fs, d=K+wc;
    Biquad2 q; q.b0=wc/d; q.b1=wc/d; q.b2=0; q.a1=(wc-K)/d; q.a2=0; return q;
}
static Biquad2 make_lp2(double fc,double fs,double Q){  // 2nd-order LP (RBJ)
    double w0=2.0*M_PI*fc/fs, c=std::cos(w0), s=std::sin(w0), al=s/(2.0*Q), a0=1.0+al;
    Biquad2 q; q.b0=((1.0-c)/2.0)/a0; q.b1=(1.0-c)/a0; q.b2=((1.0-c)/2.0)/a0;
    q.a1=(-2.0*c)/a0; q.a2=(1.0-al)/a0; return q;
}
static Biquad2 make_hp2(double fc,double fs,double Q){  // 2nd-order HP (RBJ)
    double w0=2.0*M_PI*fc/fs, c=std::cos(w0), s=std::sin(w0), al=s/(2.0*Q), a0=1.0+al;
    Biquad2 q; q.b0=((1.0+c)/2.0)/a0; q.b1=-(1.0+c)/a0; q.b2=((1.0+c)/2.0)/a0;
    q.a1=(-2.0*c)/a0; q.a2=(1.0-al)/a0; return q;
}
// G3RUH flat-port RX occupancy: 3rd-order Butterworth LP @6 kHz (1st + 2nd@Q=1)
// + 2nd-order HP @150 Hz (AC-coupled LF skirt). NO de-emphasis. In place.
static void flat_port_bandlimit(std::vector<float>& x, double fs){
    Biquad2 lp1=make_lp1(6000.0,fs), lp2=make_lp2(6000.0,fs,1.0),
            hp=make_hp2(150.0,fs,0.70710678);
    for(auto& v:x){ float y=hp.process(v); y=lp1.process(y); y=lp2.process(y); v=y; }
}

// Flat 9600-data-port FM channel (see header). Returns the realized in-band
// band-average S:N (dB) it calibrated to.
static float flat_port_channel(float* audio, int n, float target_snr_db, unsigned seed,
                               int echo_delay, float echo_gain,
                               float fade_depth, float fade_hz,
                               float fs, float freq_diffusion, bool flat_noise) {
    // 0. Managed drive: fixed RMS, NO limiter/clip.
    double ms=0; for(int i=0;i<n;i++) ms+=(double)audio[i]*audio[i];
    ms/=(n>0?n:1);
    double rms=std::sqrt(ms>1e-20?ms:1e-20);
    double g=0.25/rms; for(int i=0;i<n;i++) audio[i]=(float)(audio[i]*g);

    // 1. Oscillator drift (Wiener) via analytic rotation.
    {
        std::mt19937 pn(1000u+seed); double dt=1.0/fs;
        double step=freq_diffusion*std::sqrt(dt);
        std::normal_distribution<double> d(0.0,step);
        std::vector<double> ph(n); double f=0,ac=0;
        for(int i=0;i<n;i++){ f+=d(pn); ac+=2.0*M_PI*f*dt; ph[i]=ac; }
        auto a=analytic(audio,n);
        for(int i=0;i<n;i++){ std::complex<float> r((float)std::cos(ph[i]),(float)std::sin(ph[i])); audio[i]=std::real(a[i]*r); }
    }
    // 2. Optional multipath: echo within CP + slow flat fade.
    if(echo_delay>0 && echo_gain>0.0f){
        std::vector<float> src(audio,audio+n);
        for(int i=0;i<n;i++){ float e=(i>=echo_delay)?echo_gain*src[i-echo_delay]:0.0f; audio[i]=src[i]+e; }
    }
    if(fade_depth>0.0f && fade_hz>0.0f){
        std::mt19937 fr(2000u+seed); std::uniform_real_distribution<float> u(0.0f,2.0f*(float)M_PI);
        float phi0=u(fr), dt=1.0f/fs;
        for(int i=0;i<n;i++){ float env=1.0f+fade_depth*std::sin(2.0f*(float)M_PI*fade_hz*i*dt+phi0); audio[i]*=env; }
    }
    // 3. Measure band-limited SIGNAL power (post drift/fade).
    std::vector<float> sigbp(audio,audio+n); flat_port_bandlimit(sigbp,fs);
    double Psig=0; for(float v:sigbp) Psig+=(double)v*v; Psig/=(n>0?n:1);
    // 4. Unit-noise realization: white -> f^2 (differentiator) [or flat].
    std::mt19937 rng(3000u+seed); std::normal_distribution<float> nd(0.0f,1.0f);
    std::vector<float> noise(n);
    if(flat_noise){ for(int i=0;i<n;i++) noise[i]=nd(rng); }
    else { float prev=0; for(int i=0;i<n;i++){ float w=nd(rng); noise[i]=w-prev; prev=w; } }
    std::vector<float> nbp(noise); flat_port_bandlimit(nbp,fs);
    double Pn=0; for(float v:nbp) Pn+=(double)v*v; Pn/=(n>0?n:1);
    // 5. Calibrate sigma to hit target band-average S:N (VARA-sheet convention).
    double snr_lin=std::pow(10.0,(double)target_snr_db/10.0);
    double sigma=(Pn>1e-30)?std::sqrt(Psig/(snr_lin*Pn)):0.0;
    // 6. Inject, then band-limit signal+noise through the one RX chain.
    for(int i=0;i<n;i++) audio[i]+=(float)(sigma*noise[i]);
    std::vector<float> out(audio,audio+n); flat_port_bandlimit(out,fs);
    for(int i=0;i<n;i++) audio[i]=out[i];
    // Realized band-average S:N (in-band signal / in-band injected noise).
    double Pnoise_inj=sigma*sigma*Pn;
    return (Pnoise_inj>1e-30)?(float)(10.0*std::log10(Psig/Pnoise_inj)):999.0f;
}

} // namespace

int run_wide6k_gate() {
    int comb = 12;
    int N = 64;
    if (!parse_gate_int("IRIS_WIDE_COMB", 2, 40, comb) ||
        !parse_gate_int("IRIS_WIDE_N", 8, 4096, N)) return 1;
    const char* flat_noise_value = getenv("IRIS_WIDE_FLATNOISE");
    bool flat_noise = flat_noise_value && strcmp(flat_noise_value, "1") == 0;

    printf("=== WIDE-6K flat-port forced-level decode matrix ===\n");
    printf("comb_spacing=%d  N=%d  noise-model=%s\n", comb, N,
           flat_noise?"FLAT-AWGN (optimistic sensitivity bound)":"f^2 FM-discriminator (honest)");

    NegotiatedPassband pb = wide6k_passband();
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, comb, 24);
    float baud = cfg.symbol_rate();
    float f_lo = cfg.used_carrier_bins.empty()?0.0f:bin_to_freq(cfg.used_carrier_bins.front(),cfg.nfft,cfg.sample_rate);
    float f_hi = cfg.used_carrier_bins.empty()?0.0f:bin_to_freq(cfg.used_carrier_bins.back(),cfg.nfft,cfg.sample_rate);
    printf("profile: band %.0f-%.0f Hz | nfft=%d cp=%d | used=%d data=%d pilot=%d (comb 1:%d) | baud=%.2f | carrier span %.0f-%.0f Hz\n",
           pb.low_hz, pb.high_hz, cfg.nfft, cfg.cp_samples, cfg.n_used_carriers,
           cfg.n_data_carriers, cfg.n_pilot_carriers, cfg.pilot_carrier_spacing, baud, f_lo, f_hi);
    printf("wide bar (VARA FM WIDE, WGN:40) = 18853 bps.  Gross = n_data * bpc * (rn/rd) * baud.\n");

    // preset id = O-level+1; bpc/rn/rd from OFDM_SPEED_LEVELS / kUniformPresets.
    struct GLevel { int preset; const char* name; LdpcRate fec; int bpc; int rn; int rd; };
    std::vector<GLevel> levels = {
        { 1,  "O0  BPSK   r1/2", LdpcRate::RATE_1_2, 1, 1, 2 },
        { 2,  "O1  QPSK   r1/2", LdpcRate::RATE_1_2, 2, 1, 2 },
        { 3,  "O2  QPSK   r3/4", LdpcRate::RATE_3_4, 2, 3, 4 },
        { 4,  "O3  16QAM  r1/2", LdpcRate::RATE_1_2, 4, 1, 2 },
        { 5,  "O4  16QAM  r5/8", LdpcRate::RATE_5_8, 4, 5, 8 },
        { 6,  "O5  16QAM  r3/4", LdpcRate::RATE_3_4, 4, 3, 4 },
        { 7,  "O6  32QAM  r5/8", LdpcRate::RATE_5_8, 5, 5, 8 },
        { 8,  "O7  64QAM  r5/8", LdpcRate::RATE_5_8, 6, 5, 8 },
        { 9,  "O8  64QAM  r3/4", LdpcRate::RATE_3_4, 6, 3, 4 },
        {10,  "O9  256QAM r5/8", LdpcRate::RATE_5_8, 8, 5, 8 },
        {11,  "O10 256QAM r3/4", LdpcRate::RATE_3_4, 8, 3, 4 },
        {12,  "O11 256QAM r7/8", LdpcRate::RATE_7_8, 8, 7, 8 },
    };

    struct WCond { const char* name; float snr; int ed; float eg; float fd; float fh; float dr; bool clean; };
    std::vector<WCond> conds = {
        { "CLEAN-IQ", 0,   0, 0.0f,  0.0f,  0.0f, 0.0f, true  },  // round-trip, no channel
        { "WGN40",    40,  0, 0.0f,  0.0f,  0.0f, 3.0f, false },
        { "WGN30",    30,  0, 0.0f,  0.0f,  0.0f, 3.0f, false },
        { "WGN20",    20,  0, 0.0f,  0.0f,  0.0f, 3.0f, false },
        { "MPG30",    30, 24, 0.40f, 0.25f, 1.5f, 3.0f, false },  // multipath proxy
    };

    const int ncw = 8;
    std::vector<uint8_t> payload(4096);
    for (int i=0;i<4096;i++) payload[i]=(uint8_t)(i*37+13);

    // Production S2 tracking ON.
    if (!set_toggle("IRIS_S2_E1","1") || !set_toggle("IRIS_S2_E2","1") ||
        !set_toggle("IRIS_S2_E3","1") || !set_toggle("IRIS_S2_LINDET","1")) {
        printf("failed to set S2 toggles\n");
        return 1;
    }
    int total_pass = 0;

    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        tm.n_codewords = ncw;
        float gross = cfg.n_data_carriers * lv.bpc * ((float)lv.rn/lv.rd) * baud;
        float eff   = tone_map_throughput(tm, cfg, ncw);

        OfdmModulator mod(cfg);
        int pbytes = 700;
        std::vector<std::complex<float>> iq;
        for (int attempt=0; attempt<6; attempt++){
            iq = mod.build_ofdm_frame(payload.data(), pbytes, tm, lv.fec, ncw);
            if(!iq.empty()) break;
            pbytes = pbytes*3/4;
        }
        if(iq.empty()){ printf("\n### %-16s SKIP (no frame)\n", lv.name); continue; }

        printf("\n### %-16s carriers=%d net_bpc=%.3f gross=%.0f eff=%.0f bps  (%.2fx bar)  payload=%d ncw=%d frame=%d smp\n",
               lv.name, cfg.n_data_carriers, lv.bpc*(float)lv.rn/lv.rd, gross, eff,
               gross/18853.0f, pbytes, ncw, (int)iq.size());
        printf("  %-10s | decode%% | pass/N det | realizedSN | ~estSNR\n","cond");

        for (auto& cd : conds) {
            int pass=0, det=0; float snr_sum=0; int snr_n=0; double real_sum=0; int real_n=0;
            for (int t=0;t<N;t++){
                std::vector<std::complex<float>> rx;
                if (cd.clean) {
                    rx.assign(iq.begin(), iq.end());
                    float peak=1e-9f; for(auto&s:rx) peak=std::max(peak,std::abs(s.real()));
                    float gg=0.5f/peak; for(auto&s:rx) s*=gg;
                } else {
                    std::vector<float> aud(iq.size());
                    for(size_t i=0;i<iq.size();i++) aud[i]=iq[i].real();
                    float rsn = flat_port_channel(aud.data(),(int)aud.size(), cd.snr, (unsigned)t,
                                                  cd.ed, cd.eg, cd.fd, cd.fh, 48000.0f, cd.dr, flat_noise);
                    real_sum+=rsn; real_n++;
                    rx = analytic(aud.data(),(int)aud.size());
                }
                size_t pad=48000;
                rx.insert(rx.begin(),pad,std::complex<float>(0,0));
                rx.insert(rx.end(),pad,std::complex<float>(0,0));
                auto sync = ofdm_detect_frame(rx.data(),(int)rx.size(),cfg);
                if(!sync.detected) continue;
                det++;
                OfdmDemodulator demod(cfg);
                auto r = demod.demodulate(rx.data(),(int)rx.size(),tm,&sync);
                bool ok = r.success && r.payload.size()==(size_t)pbytes &&
                          memcmp(r.payload.data(),payload.data(),pbytes)==0;
                if(ok) pass++;
                if(r.mean_channel_snr_db>0){ snr_sum+=r.mean_channel_snr_db; snr_n++; }
            }
            total_pass += pass;
            char rsn_str[24];
            if (cd.clean) snprintf(rsn_str,sizeof rsn_str,"    n/a ");
            else snprintf(rsn_str,sizeof rsn_str,"%6.1f dB", real_n>0?real_sum/real_n:0.0);
            printf("  %-10s | %6.1f%% | %3d/%3d %3d | %s | %5.1f dB\n",
                   cd.name, 100.0f*pass/N, pass, N, det, rsn_str,
                   snr_n>0?snr_sum/snr_n:0.0f);
        }
    }
    printf("\n=== WIDE-6K gate done ===\n");
    return total_pass == 0 ? 1 : 0;
}
