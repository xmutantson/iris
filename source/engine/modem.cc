#include "engine/modem.h"
#include "ax25/ax25_protocol.h"
#include "native/frame.h"
#include "fec/ldpc.h"
#include "common/fft.h"
#include "common/logging.h"
#include "monocypher.h"
#include <cstring>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <chrono>
#include <limits>
#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

static std::uint64_t sample_position_after(std::uint64_t origin,
                                           std::uint64_t relative) noexcept {
    const std::uint64_t limit = std::numeric_limits<std::uint64_t>::max();
    return relative > limit - origin ? limit : origin + relative;
}

// Hard invariant: the OFDM FFT size MUST equal MfskAck::NFFT.  The MFSK tone
// ACK computes its bin frequencies in NFFT=1024 units (mfsk_ack.cc:18) while
// the caller derives first_bin in ofdm_config_.nfft units (§1.9).  If the two
// differ, every tone ACK lands at the wrong frequency and the link silently
// delivers 0 bytes.  nfft is pinned to 1024 by config + negotiation; this is a
// tripwire against a future regression that reintroduces a non-1024 nfft.
static void assert_mfsk_nfft(int ofdm_nfft) {
    if (ofdm_nfft != MfskAck::NFFT) {
        IRIS_LOG("[FATAL] OFDM nfft=%d != MfskAck::NFFT=%d — tone-ACK bins would "
                 "land at the wrong frequency (silent 0-delivery); aborting activation",
                 ofdm_nfft, (int)MfskAck::NFFT);
        assert(ofdm_nfft == MfskAck::NFFT && "OFDM nfft must equal MfskAck::NFFT");
        std::abort();  // survives -DNDEBUG: fail loud rather than deliver 0
    }
}

static uint8_t next_burst_epoch(uint8_t epoch) {
    return epoch == 0xFE ? 0 : (uint8_t)(epoch + 1);
}

static bool reverse_ack_epoch_matches(uint8_t echo, uint8_t expected) {
    return echo == 0xFF || echo == expected;
}

// #1 reverse-ACK ROOT FIX — LIVE N(R) re-stamp at the single air chokepoint.
// A queued AX.25 S-/I-frame froze its N(R) at build time (send_rr reads vr_ at
// ax25_session.cc:290; send_next_iframe at :421); it then sits in tx_queue_
// behind other traffic and reaches the air STALE. A one-window-stale N(R) is
// indistinguishable from a fresh one mod-8, so it aliases (GBN premise 2 —
// Bertsekas & Gallager, Data Networks §2.4.2) and the peer's V(A)-advance
// consumer destroys never-received frames. Rewrite ctrl-byte bits 5-7 to the
// session's LIVE V(R) here — extending to every reverse carrier the authority
// rule the tone-ACK already applies (modem.cc:3572). All peer I-frames advance
// the shadow V(R) (ax25_session.cc:602-607) BEFORE the KISS pump can emit its
// own RR, so live V(R) is always >= any frozen N(R): the stamp only ever RAISES
// N(R) toward truth (a semantically-valid AX.25 cumulative-ACK bump). Invariant
// I-D (data-flow-tx-queue.md §4): no frame crosses the air with N(R) older than
// session V(R) at the TX instant. U-frames (SABM/UA/DISC/DM/FRMR — ctrl bits
// 1-0 == 11) carry no N(R) field and are skipped.
static void restamp_live_nr(std::vector<uint8_t>& frame, uint8_t live_vr,
                            bool extended = false) {
    // live_vr is 3-bit (mod-8) or 7-bit (wide modulo-128).  For an extended frame
    // the N(R) is re-stamped in octet-2, leaving the 7-bit N(S)/SS in octet-1 intact.
    assert((extended ? live_vr < 128 : live_vr < 8) && "live V(R) out of range for modulus");
    uint8_t old_nr = 0;
    uint8_t before = frame.size() >= 15 ? frame[14] : 0;
    if (ax25_restamp_nr(frame, live_vr, &old_nr, extended) &&
        old_nr != (uint8_t)(live_vr & (extended ? 0x7F : 0x07)))
        IRIS_LOG("[TX-RESTAMP] N(R) %u -> %u (live V(R)%s) ctrl 0x%02X -> 0x%02X",
                 old_nr, (unsigned)(live_vr & (extended ? 0x7F : 0x07)),
                 extended ? " ext" : "", before,
                 extended ? (frame.size() >= 16 ? frame[15] : 0) : frame[14]);
}

// Binary TUNE report: embedded in OFDM ramp frame payload.
// Format: [0xBB] [count:u8] [entry × count]
// Entry:  [index:u8] [iters:i8] [H_hi:u8] [H_lo:u8] [snr_i8]
//   iters: -2=preamble-only, -1=not measured, 0-50=LDPC iters
//   H: unsigned 16-bit fixed-point, H * 100 (range 0-655.35)
//   snr: signed 8-bit, SNR_dB + 30 (range -30 to +97 dB, 0.5 dB res not needed)
static std::vector<uint8_t> tune_build_binary_report(
    const int* iters, const float* H, const float* snr, int count) {
    std::vector<uint8_t> buf;
    buf.push_back(TUNE_REPORT_MAGIC);
    int n = 0;
    for (int i = 0; i < count; i++)
        if (iters[i] != -1) n++;
    buf.push_back((uint8_t)n);
    for (int i = 0; i < count; i++) {
        if (iters[i] == -1) continue;
        buf.push_back((uint8_t)i);              // frame index
        buf.push_back((uint8_t)(int8_t)iters[i]); // iters as signed byte
        uint16_t h16 = (uint16_t)std::min(65535.0f, H[i] * 100.0f);
        buf.push_back((uint8_t)(h16 >> 8));
        buf.push_back((uint8_t)(h16 & 0xFF));
        int8_t s = (int8_t)std::clamp((int)(snr[i] + 30.0f), 0, 127);
        buf.push_back((uint8_t)s);
    }
    return buf;
}

static bool tune_parse_binary_report(const uint8_t* data, size_t len,
    int* out_iters, float* out_H, float* out_snr, int max_entries) {
    if (len < 2 || data[0] != TUNE_REPORT_MAGIC) return false;
    int n = data[1];
    size_t pos = 2;
    for (int i = 0; i < n && pos + 5 <= len; i++) {
        int idx = data[pos];
        int8_t it = (int8_t)data[pos + 1];
        uint16_t h16 = ((uint16_t)data[pos + 2] << 8) | data[pos + 3];
        int8_t s = (int8_t)data[pos + 4];
        pos += 5;
        if (idx < max_entries) {
            out_iters[idx] = it;
            out_H[idx] = h16 / 100.0f;
            out_snr[idx] = s - 30.0f;
        }
    }
    return n > 0;
}

// Codewords per OFDM frame for a given speed level. Packing policy keyed on the
// CONSTELLATION (bits_per_carrier from the single-source ladder), NOT hardcoded
// O-level boundaries — those boundaries went stale when 32QAM was inserted at O6
// (they treated O6=64QAM/O8=256QAM, off by one). This is the ncw both the batch
// sizing (max_payload) AND the TX build read, so they can never disagree.
//   O0 BPSK, O1 QPSK r1/2 : 1 CW  — short frames for weak-signal reliability
//     (2 CW tested OTA fails at O0: LDPC can't ride phase drift over 65 symbols;
//      1 CW = 33 symbols is within the Kalman tracker's capability)
//   O2 QPSK r3/4          : 2 CW
//   16QAM  (O3-O5)        : 4 CW
//   32/64QAM (O6-O8)      : 8 CW  — amortize preamble/pilot at the throughput ceiling
//   256/1024QAM (O9-O13)  : 4 CW  — tight rings can't sustain ncw=8 partial fill on FM
static int ofdm_cw_for_level(int level) {
    return ofdm_level_codeword_count(level);
}

// LDPC code rate for a given OFDM speed level. Delegates to the single-source
// ladder (kUniformPresets, via ofdm_level_fec_rate) so the batch-sizing capacity
// (max_payload) and the TX tone map that get_uniform_tone_map builds resolve the
// SAME fec for every level. The former hardcoded 10-rung table went stale when
// 32QAM was inserted at O6 (O7/O8 swapped, O10-O13 clamped wrong): at O7 the
// sizing used r3/4's k while the frame built at r5/8, oversizing max_payload so
// build_ofdm_frame rejected the frame and the TX silently skipped.
static LdpcRate ofdm_level_to_fec_rate(int level) {
    return ofdm_level_fec_rate(level);
}

// Single-frame OFDM payload capacity (bytes) at a given level: ncw codewords,
// each carrying (block_size/8 - 6) info bytes (2-byte len + 4-byte CRC per
// block). This is the frame-byte capacity the oversized guard checks against.
static int ofdm_capacity_bytes_for_level(int level, const OfdmConfig& config) {
    ToneMap map = make_uniform_tone_map(static_cast<uint8_t>(level + 1),
                                        config.n_data_carriers, config.nfft);
    map.n_codewords = ofdm_cw_for_level(level);
    auto geometry = checked_ofdm_frame_geometry(config, map);
    auto capacity = geometry ? ofdm_payload_capacity_bytes(*geometry) : std::nullopt;
    return capacity && *capacity <= static_cast<std::uint64_t>(INT_MAX)
        ? static_cast<int>(*capacity) : 0;
}

// Largest single-frame OFDM payload capacity across ALL speed levels — the
// drain-time drop threshold. A frame larger than THIS can never be carried at
// any level (a client mis-set its MAX_INFO / paclen) and is dropped LOUD. A
// frame that merely exceeds the CURRENT level's capacity is kept and
// reject-requeued (Modem::requeue_rejected_frames) so it survives a transient
// level fall — the dynamic-MAX_INFO co-dependency safety net.
static int ofdm_abs_max_capacity(const OfdmConfig& config) {
    int m = 0;
    for (int L = 0; L < NUM_OFDM_SPEED_LEVELS; L++)
        m = std::max(m, ofdm_capacity_bytes_for_level(L, config));
    return m;
}

// Fix B (bounded oversize-reject recovery, DATALINK_TAX_DIAGNOSIS.md) — how many
// CONSECUTIVE burst rejects a recoverable (kept) frame may spin before the
// modulated level is forced up to carry it. The burst drain runs at ~10 Hz, so
// this bounds the spin to a few seconds — far below the ~68 s / T1 N2=30 window
// that a capacity mismatch would otherwise ride into a DISCONNECT, yet well above
// any legitimate transient level-fall recovery (which self-heals in a handful of
// bursts and resets the streak on the next successful send).
static constexpr int OFDM_OVERSIZE_REJECT_LIMIT = 32;

// The modulated fit-floor for a stuck frame: once the reject streak crosses the
// limit, return the MIN OFDM speed level whose single-frame capacity carries the
// largest stuck (wrapped) frame, so resolve_ofdm_tx_level floors the level up and
// the frame gets on air at a confirmed-decodable rate (a kept frame fits the
// anchor, so this floor is at/below the peer-ACKed anchor). Returns -1 while the
// streak is still inside the transient-recovery budget (no override). A frame too
// big for even the top level is already drop-drained upstream (> abs_cap+5), so the
// top level is a safe worst-case fallback.
static int ofdm_oversize_fit_floor(int reject_streak, int stuck_frame_bytes,
                                   const OfdmConfig& config) {
    if (reject_streak < OFDM_OVERSIZE_REJECT_LIMIT) return -1;
    for (int L = 0; L < NUM_OFDM_SPEED_LEVELS; L++)
        if (ofdm_capacity_bytes_for_level(L, config) >= stuck_frame_bytes)
            return L;
    return NUM_OFDM_SPEED_LEVELS - 1;
}

// Leg 3 — the dynamic AX.25 MAX_INFO (I-frame info-field limit) for a given
// OFDM speed level: the frame-byte capacity minus the REAL OFDM-KISS framing
// overhead (OFDM_KISS_FRAMING_OVERHEAD = AX.25 header 16 + wrapper 4 = 20 B,
// types.h). SINGLE source replacing the five fixed 75-byte pins; callers pass
// tx_acked_level_ (the CONFIRMED anchor) so MAX_INFO grows with the proven rate
// and shrinks on a peer REJ, letting reverse/forward I-frames fill the larger
// frame the higher level carries. set_max_info clamps to [16,1024]; the O0..O5
// range (74..556) is well inside it.
//
// The subtrahend was "- 19", which under-counted the framing overhead by exactly
// one byte (it omitted the shared epoch byte of the multi-payload wrapper): a
// 75-B O0 info frame then framed to info(75)+AX.25(16)+wrapper(4)=95 > 94-B O0
// capacity, so the burst packer's oversized guard DROPPED every O0 data frame,
// the session timed out to a T1 N2 disconnect, and the whole corpus rode the
// 800-baud AFSK fallback — OFDM never activated (DATALINK_TAX_DIAGNOSIS.md, Fix A).
// Deriving from OFDM_KISS_FRAMING_OVERHEAD makes framed == capacity provably (an
// invariant asserted in tests.cc test_dynamic_max_info) so it can never re-drift.
static int ofdm_max_info_for_level(int level, const OfdmConfig& config) {
    return ofdm_capacity_bytes_for_level(level, config) - OFDM_KISS_FRAMING_OVERHEAD;
}

// Leg 2 pacing kill — burst-MAC in-window continuation DECISION. The 1.5s csma
// re-arm at TX-END is a stop-and-wait timer: it is ACK-clocked (cancelled when
// the peer's RR arrives, modem.cc ~:1239) so the initiator normally re-sends at
// ~RR turnaround, NOT the full 1.5s. Returning true lets the initiator send the
// NEXT burst WITHOUT waiting for that RR while the K=7 window still has room +
// data is queued — filling the window instead of paying one RR turnaround per
// burst. The DCD (energy-busy) gate runs BEFORE the csma gate and still blocks a
// genuinely busy channel, so this never overrides real carrier detect.
//
// *** COLLISION-SAFETY (why `enabled`/Modem::burst_fill_ defaults FALSE): sending
// the next burst before the peer's RR collides with that RR on half-duplex UNLESS
// the responder HOLDS its autonomous RR to burst-end (the coupled RSP T2-hold +
// P=1 trailing solicit). Without the RSP hold this reintroduces the documented
// 25-87 bps RSP-RR collision. The RSP hold is a two-sided turn-taking change that
// cannot be validated in loopback (no collision) or in-process — it needs the
// two-stack/fleet OTA bench. So production keeps the safe ACK-clocked stop-and-
// wait; this mechanism is proven in-process only (test (a)). ***
static bool burst_fill_continue(bool enabled, bool ofdm_kiss_tx, bool session_active,
                                bool we_initiated, bool have_queued_data,
                                int window_used, int window_k) {
    if (!enabled) return false;                       // default: safe stop-and-wait
    if (!(ofdm_kiss_tx && session_active && we_initiated))
        return false;                                 // initiator OFDM-KISS only
    if (!have_queued_data) return false;              // nothing to continue with
    if (window_used >= window_k) return false;        // window FULL -> wait for an ACK
    return true;                                       // room + data -> keep filling
}

// RX PACKAGE, ROOT-2 — retain-vs-consume decision after a failed OFDM decode.
// When CMD is waiting for an ACK it uses a SHORT 1-CW expect-ack frame-length
// gate (fast turnaround). If the incoming frame is actually a LONGER multi-CW
// reverse frame (a reverse I-frame, or a multi-CW REJ/RR — both of which dynamic
// MAX_INFO + burst MAC make common) the short gate fires early and the demod
// runs on a TRUNCATED buffer, so the decode fails. Returning true means "RETAIN
// the RX buffer" — do NOT consume the detected region. The fail branch has
// already cleared ofdm_kiss_rx_confirmed_, so the NEXT pass buffers the FULL
// multi-level frame (the expanded gate) and decodes it, instead of consuming the
// truncated bytes and losing the whole reverse frame (a full airtime-scaled T1
// per loss — the datalink tax this package targets).
//
// Bounded to exactly ONE retry: on the retry ofdm_kiss_rx_confirmed_ is false, so
// the short gate is not taken and short_ack_gate_used is false → normal consume.
// A false positive / quality-gated frame (empty LLRs) returns early upstream and
// never reaches here; requiring llrs_nonempty keeps a genuine noise burst from
// pinning the buffer (the redetect limit backstops it either way).
static inline bool ofdm_root2_retain_on_short_ack_fail(bool short_ack_gate_used,
                                                       bool decode_success,
                                                       bool llrs_nonempty) {
    return short_ack_gate_used && !decode_success && llrs_nonempty;
}

// A2 — the classification-INDEPENDENT OFDM TX level caps, side-effect-free.
// Applies, in order: the no-ACK downshift TARGET (kiss -> O0, arq -> step down),
// the peer-SNR cap, and the TX-feedback ceiling (+ receiver-driven leap). The
// caller applies any control-batch force to `base` BEFORE calling and performs
// the gearshift force side-effect itself. Used by BOTH the batch-sizing pass and
// resolve_ofdm_tx_level() so the two never diverge. `peer_snr_level` = -1 means
// no peer-SNR feedback; `no_ack` means the no-ACK downshift limit was hit.
static int ofdm_apply_tx_level_caps(int base, bool kiss_tx, bool no_ack,
                                    int tx_acked, int tx_proposed,
                                    int peer_snr_level, int leap_max) {
    int level = base;
    if (no_ack && level > 0)
        level = kiss_tx ? 0 : std::max(0, level - 1);
    if (peer_snr_level >= 0 && level > 0 && peer_snr_level < level)
        level = peer_snr_level;
    if (kiss_tx) {
        int ceiling = tx_acked + 1;
        if (tx_proposed >= 0)
            ceiling = std::max(ceiling, std::min(tx_proposed, tx_acked + leap_max));
        if (level > ceiling) level = ceiling;
    }
    // FIXED-GEAR measurement cap (default-inert): env IRIS_TX_LEVEL_CAP=N hard-
    // caps the TX ladder at O<N>.  IRIS_MAX_OFDM_LEVEL caps only the RECEIVER
    // proposal; the sender's own anchor+1 probing still climbs one rung past the
    // proposal cap (measured: 24 O9 slots in a MAX=8 session), which mixes gears
    // into a fixed-gear A/B.  Applied LAST and in this single-sourced cap pass so
    // the sized batch and the modulated frame agree.  Climb below the cap is
    // untouched; unset = today's behavior exactly.  Env is read per call (a few
    // times per burst build — negligible) so the default-inert witness is
    // testable in-process.
    const char* cap_env = std::getenv("IRIS_TX_LEVEL_CAP");
    if (cap_env) {
        int tx_level_cap = std::atoi(cap_env);
        if (tx_level_cap >= 0 && level > tx_level_cap)
            level = tx_level_cap;
    }
    return level;
}

int Modem::apply_tx_level_caps_probe(int base, bool kiss_tx, bool no_ack,
                                     int tx_acked, int tx_proposed,
                                     int peer_snr_level, int leap_max) {
    // Test entry for the file-static single-sourced cap pass (fixed-gear knob).
    return ofdm_apply_tx_level_caps(base, kiss_tx, no_ack, tx_acked, tx_proposed,
                                    peer_snr_level, leap_max);
}

// Normalize one OFDM frame's passband audio (buf[fstart..end)) to the TX
// operating point with ONE scale for the WHOLE frame.
//
// Frame layout (ofdm_mod.cc step 12): [noise][train1][train2][sync][data..][tail].
// The RMS target (0.50 before tx_level) is measured on the DATA region — 4
// symbols in — and the resulting scale is applied to EVERY sample of the
// frame. It must be one scale: train2 is the channel-estimation reference,
// so H has to predict the data amplitude (same scale as data), and train1
// must stay IDENTICAL to train2 on the wire — the training-pair noise
// estimator measures their difference as channel noise, and the Schmidl-Cox
// and fine-CFO correlators assume it too.
//
// HISTORY (the bug this replaces): the old code normalized "data" starting at
// 2 symbols — a boundary that predates the leading noise symbol — which put
// train2 (but not train1) in the data-scaling group and left train1 at the
// raw IFFT level. Measured on a clean -x sim wire: train2 = 0.57 * train1,
// so the pair estimator honestly reported the TX's own mismatch as a ~10 dB
// channel on a noiseless link, and the same poisoned per-carrier sigma^2(k)
// fed the MMSE/LLR path on every live frame (fact doc
// data-flow-noise-var.md §8). Preamble peak safety is preserved by scaling
// the WHOLE frame down if the scaled preamble would hit the hard clip —
// never the preamble alone, which is the ratio-breaking move.
static void ofdm_normalize_tx_frame(std::vector<float>& buf, size_t fstart,
                                    int sym_len, bool eq_on) {
    constexpr float OFDM_TARGET_RMS = 0.50f;
    constexpr float PREAMBLE_PEAK_LIMIT = 0.90f;
    const size_t n = buf.size();
    if (fstart >= n || sym_len <= 0) return;
    const size_t head = (size_t)4 * (size_t)sym_len;  // noise+train1+train2+sync
    const size_t data_start = fstart + std::min(head, n - fstart);

    double sum_sq = 0.0;
    for (size_t i = data_start; i < n; i++)
        sum_sq += (double)buf[i] * buf[i];
    const size_t data_len = n - data_start;
    float rms = (data_len > 0) ? (float)std::sqrt(sum_sq / (double)data_len) : 0.0f;
    float scale = (rms > 1e-6f) ? (OFDM_TARGET_RMS / rms) : 0.1f;

    // If the scaled preamble would exceed the peak limit, back the WHOLE
    // frame off (relative amplitudes preserved).
    float pre_peak = 0.0f;
    for (size_t i = fstart; i < data_start; i++)
        pre_peak = std::max(pre_peak, std::abs(buf[i]));
    if (pre_peak * scale > PREAMBLE_PEAK_LIMIT)
        scale = PREAMBLE_PEAK_LIMIT / pre_peak;

    for (size_t i = fstart; i < n; i++)
        buf[i] *= scale;

    // Hard clip to ±0.95 (soundcard range; residual data peaks above the
    // ACE clipper's target PAPR still land here).
    for (size_t i = fstart; i < n; i++) {
        if (buf[i] > 0.95f) buf[i] = 0.95f;
        else if (buf[i] < -0.95f) buf[i] = -0.95f;
    }

    IRIS_LOG("[TX-OFDM] passband direct: %zu samples, RMS=%.3f->%.3f (scale=%.4f, "
             "whole-frame, preamble peak=%.3f, EQ=%s)",
             n - fstart, rms, rms * scale, scale, pre_peak * scale,
             eq_on ? "on" : "off");
}

// Build RX tone map for a given speed level.
static ToneMap ofdm_rx_tone_map_for_level(int level, const OfdmConfig& config, bool use_nuc) {
    ToneMap tm = get_uniform_tone_map(level + 1, config);
    tm.use_nuc = use_nuc;
    tm.n_codewords = ofdm_cw_for_level(level);
    return tm;
}

// RX TONE-MAP LATCH, prong 2 (data-flow-rx-tonemap.md §5) — the codeword count
// the frame-length gate sizes against. Buffering too FEW symbols is fatal and
// silent (the demod runs on a truncated buffer, deterministic CRC fail at the
// same offset every retransmission); buffering too many costs only latency and
// is recoverable. So in KISS mode the gate is FLOORED at the full data-frame
// codeword count for the current RX level: even if the persistent RX map is
// stale or carries a control frame's 1-CW shape (INV-1 violated), the gate can
// never under-buffer a data frame. The deliberate 1-CW expect-ack fast gate is
// applied AFTER this computation and keeps its own bounded ROOT-2 compensation.
static int ofdm_gate_n_cw(bool kiss_tx, int map_ncw, int rx_level) {
    int n_cw = std::max(1, map_ncw);
    if (kiss_tx)
        n_cw = std::max(n_cw, ofdm_cw_for_level(rx_level));
    return n_cw;
}

// RX TONE-MAP LATCH, prong 3 (data-flow-rx-tonemap.md §5) — blind-detect sweep
// dedupe, keyed on (level, SHAPE) instead of level alone. The current RX level
// counts as "already tried" only when the primary decode used the FULL
// data-frame codeword count for that level (the 1-CW shape is separately tried
// before the sweep). If the persistent map carries a smaller n_codewords, the
// (level, full-CW) shape has NOT been tried yet and must stay reachable —
// otherwise a poisoned map is an absorbing state: the sweep can never re-lock
// the true configuration and every data frame at that level is lost (the
// captured 941 s RX wedge, data-flow-rx-tonemap.md §1).
static bool ofdm_sweep_skip_level(int lvl, int rx_level, int persisted_ncw) {
    return lvl == rx_level && persisted_ncw >= ofdm_cw_for_level(lvl);
}

// Describe an AX.25 frame for GUI logging
static std::string describe_ax25(const uint8_t* data, size_t len) {
    Ax25Frame f;
    if (!ax25_parse(data, len, f))
        return "AX.25 (" + std::to_string(len) + " bytes, unparseable)";

    std::string desc = f.src.to_string() + ">" + f.dst.to_string();
    if (!f.via.empty())
        desc += " via " + ax25_via_to_string(f.via);
    desc += " ";
    switch (f.type()) {
    case Ax25FrameType::I_FRAME:
        desc += "I N(S)=" + std::to_string(f.ns()) + " N(R)=" + std::to_string(f.nr())
              + " (" + std::to_string(f.info.size()) + " bytes)";
        break;
    case Ax25FrameType::S_FRAME:
        switch (f.s_type()) {
        case Ax25SType::RR:   desc += "RR";   break;
        case Ax25SType::RNR:  desc += "RNR";  break;
        case Ax25SType::REJ:  desc += "REJ";  break;
        default:              desc += "S?";    break;
        }
        desc += " N(R)=" + std::to_string(f.nr());
        break;
    case Ax25FrameType::U_FRAME:
        switch (f.u_type()) {
        case Ax25UType::SABM: desc += "SABM"; break;
        case Ax25UType::UA:   desc += "UA";   break;
        case Ax25UType::DISC: desc += "DISC"; break;
        case Ax25UType::DM:   desc += "DM";   break;
        case Ax25UType::UI:   desc += "UI (" + std::to_string(f.info.size()) + " bytes)"; break;
        case Ax25UType::XID:  desc += "XID";  break;
        default:              desc += "U?";    break;
        }
        break;
    }
    if (f.poll_final()) desc += " P/F";
    return desc;
}

static constexpr float CAL_TONE_FREQ = 1000.0f;
static constexpr int   CAL_TONE_DURATION = 48000;
static constexpr float CAL_TARGET_RMS = 0.3f;
static constexpr int RX_MUTE_HOLDOFF_SAMPLES = 9600;   // 200ms at 48kHz

Modem::Modem() = default;
Modem::~Modem() {
    // Callback owners commonly outlive an explicit shutdown but need not outlive
    // member destruction of their enclosing object.  Teardown still snapshots
    // custody; suppress only unsafe destructor-time publication.
    rx_callback_ = nullptr;
    transfer_result_callback_ = nullptr;
    ax25_state_callback_ = nullptr;
    state_callback_ = nullptr;
    shutdown();
}

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    // Validate configuration bounds
    if (config_.sample_rate < 8000 || config_.sample_rate > 192000) {
        IRIS_LOG("ERROR: sample_rate %d out of range [8000, 192000]", config_.sample_rate);
        return false;
    }
    if (config_.band_low_hz < 100.0f || config_.band_low_hz >= config_.band_high_hz) {
        IRIS_LOG("ERROR: invalid band range %.0f-%.0f Hz", config_.band_low_hz, config_.band_high_hz);
        return false;
    }
    if (config_.band_high_hz > config_.sample_rate / 2.0f) {
        IRIS_LOG("ERROR: band_high %.0f Hz exceeds Nyquist (%.0f Hz)",
                 config_.band_high_hz, config_.sample_rate / 2.0f);
        return false;
    }
    if (config_.tx_level < 0.0f || config_.tx_level > 1.0f) {
        IRIS_LOG("WARNING: tx_level %.2f clamped to [0, 1]", config_.tx_level);
        config_.tx_level = std::clamp(config_.tx_level, 0.0f, 1.0f);
    }

    if (config_.mode == "A" || config_.mode == "a") {
        float bandwidth = config_.band_high_hz - config_.band_low_hz;
        phy_config_ = mode_a_config(bandwidth);
        use_upconvert_ = true;

        // Compute center frequency from band config
        float center = config_.center_freq_hz;
        if (center <= 0.0f)
            center = (config_.band_low_hz + config_.band_high_hz) / 2.0f;

        IRIS_LOG("Band: %.0f-%.0f Hz, center %.0f Hz (baud %d, BW %.0f Hz)",
                 config_.band_low_hz, config_.band_high_hz, center,
                 phy_config_.baud_rate,
                 phy_config_.baud_rate * (1.0f + phy_config_.rrc_alpha));

        upconverter_ = Upconverter(center, config_.sample_rate);
        downconverter_ = Downconverter(center, config_.sample_rate);
    } else if (config_.mode == "B" || config_.mode == "b") {
        phy_config_ = mode_b_config();
        use_upconvert_ = false;
    } else {
        phy_config_ = mode_c_config();
        use_upconvert_ = false;
    }

    phy_config_.modulation = Modulation::BPSK;

    // Save original band/PHY for restore on disconnect after probe changes
    orig_band_low_hz_ = config_.band_low_hz;
    orig_band_high_hz_ = config_.band_high_hz;
    orig_phy_config_ = phy_config_;

    afsk_mod_ = AfskModulator(config_.sample_rate);
    afsk_demod_ = AfskDemodulator(config_.sample_rate);
    gfsk_mod_ = GfskModulator(config_.sample_rate);
    gfsk_demod_ = GfskDemodulator(config_.sample_rate);

    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);

    // Speed level cache: persist proven levels to disk
    if (!config_.data_dir.empty())
        gearshift_.set_cache_dir(config_.data_dir);

    afsk_demod_.set_preemph_alpha(config_.preemph_alpha);

    local_cap_.version = XID_VERSION;
    local_cap_.capabilities = CAP_MODE_A | CAP_COMPRESSION | CAP_STREAMING | CAP_HARQ;
    if (config_.mode == "B" || config_.mode == "b")
        local_cap_.capabilities |= CAP_MODE_B;
    if (config_.mode == "C" || config_.mode == "c")
        local_cap_.capabilities |= CAP_MODE_C;
    if (config_.encryption_mode > 0) {
        local_cap_.capabilities |= CAP_ENCRYPTION;
        local_cap_.capabilities |= CAP_PQ_CRYPTO;
    }
    if (config_.b2f_unroll)
        local_cap_.capabilities |= CAP_B2F_UNROLL;
    if (config_.ofdm_enable)
        local_cap_.capabilities |= CAP_OFDM;
    local_cap_.max_modulation = config_.max_modulation;

    int max_mod_idx = (int)config_.max_modulation;
    int max_level = NUM_SPEED_LEVELS - 1;
    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        if ((int)SPEED_LEVELS[i].modulation > max_mod_idx) {
            max_level = (i > 0) ? i - 1 : 0;
            break;
        }
    }
    gearshift_.set_max_level(max_level);

    // Delivered-rate attack A/B toggles (DATALINK_TAX_DIAGNOSIS levers B/C).
    // Both DEFAULT to the shipped safe behavior; an env var opts a single arm
    // into the experimental path so ONE binary drives every cell of the fleet
    // A/B (one-variable-per-increment) without a rebuild.
    //   IRIS_MAX_OFDM_LEVEL — the highest OFDM speed level the RECEIVER may propose
    //     to the sender (modem.h MAX_PROPOSABLE_OFDM_LEVEL, default O5 = the highest
    //     rung RELIABLY sustainable live on WGN:40; O6/O7 are reached but stall/thrash
    //     in the combined terminate stack — see the modem.h note).  Drives the
    //     crossover-recovery A/B: ARM-O5cap = default (no env); ARM-O6/ARM-O7 =
    //     IRIS_MAX_OFDM_LEVEL=6/7 reproduce the pre-fix stall (O6 anchor-stuck spin) /
    //     thrash (O7).  The leap is paced OFDM_LEVEL_LEAP_MAX rungs/step and every
    //     frame is CRC-32 self-rejecting, so an over-cap is honesty-limited, not unsafe.
    if (const char* e = std::getenv("IRIS_MAX_OFDM_LEVEL")) {
        int v = std::atoi(e);
        if (v >= 0 && v < NUM_OFDM_SPEED_LEVELS) {
            max_proposable_ofdm_level_ = v;
            IRIS_LOG("[DELIVRATE] IRIS_MAX_OFDM_LEVEL=%d (receiver-proposal cap O%d)",
                     v, v);
        }
    }
    //   IRIS_REPACK_REFRAG — crossover recovery (Fix B): on a mid-transfer anchor
    //     demote that shrinks max_info below already-packed in-flight frames, RECOVER
    //     the un-ACKed stream bytes and re-fragment at the new size (default-ON, INV-3)
    //     instead of DROPPING them (=0 restores the pre-fix drop_oversized_in_window
    //     behavior that lost stream bytes -> incomplete transfer).  A/B fail-before=0.
    if (const char* e = std::getenv("IRIS_REPACK_REFRAG")) {
        repack_refragment_ = (std::atoi(e) != 0);
        IRIS_LOG("[DELIVRATE] IRIS_REPACK_REFRAG override=%d", (int)repack_refragment_);
    }
    //   IRIS_BURST_FILL — increment 1 (window-fill): enable burst_fill_continue
    //     (initiator sends the next burst without waiting out the ACK-clocked
    //     stop-and-wait csma while the K=7 window has room + data is queued) AND
    //     the COUPLED collision-safety, the RSP autonomous-RR HOLD-to-burst-end
    //     (Ax25Session::set_rr_hold): the responder defers its autonomous RR
    //     until the initiator's burst sequence goes quiet, so the reverse ACK
    //     never collides with the next forward burst on half-duplex. Both sides
    //     read the SAME flag, so a single env var flips the two-sided change.
    if (const char* e = std::getenv("IRIS_BURST_FILL")) {
        burst_fill_ = (std::atoi(e) != 0);
        IRIS_LOG("[DELIVRATE] IRIS_BURST_FILL override=%d", (int)burst_fill_);
    }
    // climbgate Fix 1 (climb-sticky): hold the OFDM level through SNR-estimate
    // jitter while frames still decode. Intended to let the O5->O6 margin-climb
    // accumulate, but the fleet WGN:40 N=3 A/B showed it can HOLD a marginal rung
    // (32QAM is EVM-floored to 23-47% decode here, see the climbgate report) and
    // regress a seed (fix_s2 20819->7509). Since O6 does not reliably decode at
    // this operating point, the sticky hold has nothing to climb TO and only adds
    // risk -> DEFAULT-OFF (quarantined); env IRIS_CLIMB_STICKY=1 to re-arm once the
    // TX-EVM floor is closed and 32QAM becomes reliably decodable.
    {
        bool sticky = false;
        if (const char* e = std::getenv("IRIS_CLIMB_STICKY")) sticky = (std::atoi(e) != 0);
        gearshift_.set_climb_sticky(sticky);
        IRIS_LOG("[CLIMBGATE] climb_sticky=%d", (int)sticky);
    }
    // climbgate Fix A (demote-refine): default-ON; env IRIS_DEMOTE_REFINE=0 opts out.
    demote_refine_ = true;
    if (const char* e = std::getenv("IRIS_DEMOTE_REFINE")) demote_refine_ = (std::atoi(e) != 0);
    IRIS_LOG("[CLIMBGATE] demote_refine=%d", (int)demote_refine_);
    // climbgate experiment lock: force both ends to a fixed OFDM level.
    if (const char* e = std::getenv("IRIS_FORCE_OFDM_LEVEL")) {
        int v = std::atoi(e);
        if (v >= 0 && v < NUM_OFDM_SPEED_LEVELS) {
            force_ofdm_level_ = v;
            IRIS_LOG("[CLIMBGATE] IRIS_FORCE_OFDM_LEVEL=%d (pin TX + RSP proposal to O%d)", v, v);
        }
    }
    // SLOT-COALESCING PROBE (measurement instrument, default-inert): force the
    // on-air slot to coalesce N queued frames per PTT (duty-cycle probe).  Unset
    // => exactly the pre-probe burst bounds (8 frames, adaptive batch airtime).
    // See the modem.h note for the slot-bound rationale (per-frame preamble =>
    // no cross-slot timing drift; 15 s whole-slot hard cap).
    if (const char* e = std::getenv("IRIS_FRAMES_PER_BURST")) {
        int v = std::atoi(e);
        if (v >= 1 && v <= BURST_FRAMES_PROBE_MAX) {
            probe_frames_per_burst_ = v;
            IRIS_LOG("[SLOT-PROBE] IRIS_FRAMES_PER_BURST=%d (default burst cap %d)",
                     v, BURST_FRAMES_DEFAULT_MAX);
        }
    }
    if (const char* e = std::getenv("IRIS_SLOT_AIRTIME_S")) {
        float v = (float)std::atof(e);
        if (v > 0.0f) {
            probe_slot_airtime_s_ = std::min(v, SLOT_AIRTIME_HARD_CAP_S);
            IRIS_LOG("[SLOT-PROBE] IRIS_SLOT_AIRTIME_S=%.1f s (hard cap %.0f s)",
                     probe_slot_airtime_s_, SLOT_AIRTIME_HARD_CAP_S);
        }
    }
    // CONNECT DIET (connshave lever A): skip the post-probe auto-tune on a clean
    // high-SNR probe.  Default-ON; env=0 restores the always-tune fail-before arm.
    if (const char* e = std::getenv("IRIS_CONNECT_DIET"))
        connect_diet_ = (std::atoi(e) != 0);
    if (const char* e = std::getenv("IRIS_TUNE_SKIP_SNR"))
        tune_skip_snr_db_ = (float)std::atof(e);
    IRIS_LOG("[CONNDIET] connect_diet=%d tune_skip_snr=%.1f dB", (int)connect_diet_, tune_skip_snr_db_);
    // FASTER CLIMB (connshave lever B): shorten the OFDM gearshift hold on a clean
    // probe so the climb reaches O5 sooner.  Default-ON; env=0 = fail-before arm.
    if (const char* e = std::getenv("IRIS_CLEAN_CLIMB"))
        clean_climb_ = (std::atoi(e) != 0);
    if (const char* e = std::getenv("IRIS_CLEAN_CLIMB_HOLD")) {
        int v = std::atoi(e);
        if (v >= 1) clean_climb_hold_ = v;
    }
    IRIS_LOG("[CONNDIET] clean_climb=%d clean_climb_hold=%d", (int)clean_climb_, clean_climb_hold_);
    // WIDE WINDOW (turnaround lever): switch the OWNED R2 OFDM transport to AX.25 2.2
    // modulo-128 with a large window K so the sender streams many more full frames
    // before it must stop for a reverse ACK -> the per-window turnaround amortizes
    // toward 0.  Default-ON; IRIS_WIDE_WINDOW=0 restores the mod-8/K=7 fail-before.
    // IRIS_WIDE_WINDOW_K tunes K (clamped [7,127] in Ax25Session::set_wide_window).
    if (const char* e = std::getenv("IRIS_WIDE_WINDOW"))
        wide_window_ = (std::atoi(e) != 0);
    if (const char* e = std::getenv("IRIS_WIDE_WINDOW_K")) {
        int v = std::atoi(e);
        if (v >= 1) wide_window_k_ = v;
    }
    IRIS_LOG("[TURNAROUND] wide_window=%d K=%d", (int)wide_window_, wide_window_k_);

    // Sync the coupled RSP autonomous-RR HOLD to the final burst_fill_ value.
    // burst_fill_ now defaults ON (proven +2.5x delivered B/min, collision-free /
    // zero fatal disconnects across the fleet WGN:40 N=3 A/B); this arms the hold
    // on BOTH ends so the two-sided change stays consistent.
    ax25_session_.set_rr_hold(burst_fill_);
    IRIS_LOG("[DELIVRATE] window-fill=%s (RSP RR-hold %s)",
             burst_fill_ ? "ON" : "off", burst_fill_ ? "armed" : "off");

    // Wire probe controller
    probe_.on_send_audio = [this](const float* audio, int count) {
        // Queue probe audio — appended to tx_buffer_ in process_tx
        // after any pending AFSK messages (RESULT before tones, same PTT cycle)
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        probe_audio_pending_.insert(probe_audio_pending_.end(), audio, audio + count);
    };
    probe_.on_send_msg = [this](const uint8_t* data, size_t len) {
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        // Wrap probe result in AX.25 UI frame, send via AFSK.
        // Use session peer if available, else probe_peer_call_, else broadcast.
        std::string dest = ax25_session_.remote_callsign();
        if (dest.empty()) dest = probe_peer_call_;
        if (dest.empty()) dest = "PROBE";
        auto frame = ax25_build_u(
            ax25_make_addr(dest),
            ax25_make_addr(config_.callsign),
            AX25_CTRL_UI, false, true);
        frame.push_back(AX25_PID_NONE);
        frame.insert(frame.end(), data, data + len);
        constexpr size_t TX_QUEUE_MAX = 32;
        if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
            IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
            ax25_tx_queue_.pop();
        }
        ax25_tx_queue_.push(std::move(frame));
    };

    // Initialize simulated bandpass filter if configured
    if (config_.sim_bandpass_low > 0 && config_.sim_bandpass_high > config_.sim_bandpass_low) {
        sim_bp_enabled_ = true;
        float fs = (float)config_.sample_rate;
        // 8th-order Butterworth = 4 cascaded biquad sections.
        // Q values for 8th-order Butterworth pole pairs:
        const float Q8[4] = {0.5098f, 0.6013f, 0.9000f, 2.5629f};

        // Highpass sections
        {
            float f0 = config_.sim_bandpass_low;
            float w0 = 2.0f * 3.14159265f * f0 / fs;
            float c = std::cos(w0), s = std::sin(w0);
            for (int i = 0; i < 4; i++) {
                float alpha = s / (2.0f * Q8[i]);
                float a0 = 1.0f + alpha;
                sim_bp_hi_[i].b0 = ((1.0f + c) / 2.0f) / a0;
                sim_bp_hi_[i].b1 = -(1.0f + c) / a0;
                sim_bp_hi_[i].b2 = ((1.0f + c) / 2.0f) / a0;
                sim_bp_hi_[i].a1 = (-2.0f * c) / a0;
                sim_bp_hi_[i].a2 = (1.0f - alpha) / a0;
            }
        }
        // Lowpass sections
        {
            float f0 = config_.sim_bandpass_high;
            float w0 = 2.0f * 3.14159265f * f0 / fs;
            float c = std::cos(w0), s = std::sin(w0);
            for (int i = 0; i < 4; i++) {
                float alpha = s / (2.0f * Q8[i]);
                float a0 = 1.0f + alpha;
                sim_bp_lo_[i].b0 = ((1.0f - c) / 2.0f) / a0;
                sim_bp_lo_[i].b1 = (1.0f - c) / a0;
                sim_bp_lo_[i].b2 = ((1.0f - c) / 2.0f) / a0;
                sim_bp_lo_[i].a1 = (-2.0f * c) / a0;
                sim_bp_lo_[i].a2 = (1.0f - alpha) / a0;
            }
        }
        IRIS_LOG("Simulated bandpass filter: %.0f-%.0f Hz (8th order Butterworth)",
                 config_.sim_bandpass_low, config_.sim_bandpass_high);
    }

    // Initialize simulated FM de-emphasis filter if configured
    if (config_.sim_deemph_us > 0) {
        sim_deemph_enabled_ = true;
        // First-order lowpass: H(s) = 1/(1+sτ) → bilinear transform
        float tau = config_.sim_deemph_us * 1e-6f;
        float fs = (float)config_.sample_rate;
        float wc = 1.0f / tau;  // corner angular frequency
        // Bilinear transform: s = (2*fs)*(z-1)/(z+1)
        float K = 2.0f * fs;
        float a = K + wc;
        sim_deemph_.b0 = wc / a;
        sim_deemph_.b1 = wc / a;
        sim_deemph_.b2 = 0;
        sim_deemph_.a1 = (wc - K) / a;
        sim_deemph_.a2 = 0;
        sim_deemph_.z1 = sim_deemph_.z2 = 0;
        float corner_hz = wc / (2.0f * 3.14159265f);
        IRIS_LOG("Simulated FM de-emphasis: %.0f µs (corner %.0f Hz, ~6 dB/octave above)",
                 config_.sim_deemph_us, corner_hz);
    }

    // Wire ARQ session
    arq_.set_callsign(config_.callsign);
    arq_.set_local_capabilities(local_cap_.capabilities);
    probe_.set_local_caps(local_cap_.capabilities);
    // Advertise our preferred OFDM PHY parameters in probe result
    {
        uint8_t nfft_code = 2;  // 1=256, 2=1024, 3=512 (0=ABSENT/old peer)
        if (config_.ofdm_nfft == 512) nfft_code = 3;
        else if (config_.ofdm_nfft == 256) nfft_code = 1;
        probe_.set_local_ofdm_config(
            (uint8_t)config_.ofdm_cp_samples,  // e.g. 64
            12,  // pilot_carrier_spacing (1:12 comb = 52 data carriers; 32QAM-r5/8
                 // parity rung. Estimator-supported thin comb: 32QAM decodes
                 // >=95% at WGN30+MPG30 (5 comb pilots); negotiation min() is
                 // symmetric so a mixed old/new pair converges to 8 (robust).
            24,  // pilot_symbol_spacing
            nfft_code);
    }
    ArqCallbacks arq_cb;
    arq_cb.send_frame = [this](const uint8_t* data, size_t len) {
        // ARQ frames go directly to TX queue (not back through ARQ)
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        constexpr size_t TX_QUEUE_MAX = 32;
        if (tx_queue_.size() >= TX_QUEUE_MAX) {
            IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
            tx_queue_.pop();
        }
        // ARQ (non-KISS) frames are never MFSK-tone-ACK candidates.
        tx_queue_.push(TxFrame(std::vector<uint8_t>(data, data + len)));
    };
    arq_cb.on_data_received = [this](const uint8_t* data, size_t len) {
        // Complete-record seam retained for local integrations and diagnostics.
        accept_native_record(data, len);
    };
    arq_cb.on_data_fragment = [this](const uint8_t* data, size_t len, bool end) {
        return receive_native_fragment(data, len, end);
    };
    arq_cb.on_typed_data_fragment =
        [this](const uint8_t* data, size_t len, bool end,
               ArqRecordType type, uint64_t record_id) {
            return receive_native_fragment(data, len, end, type, record_id);
        };
    arq_cb.on_role_switch = [this]() {
        return resolve_strict_receive_custody("role switch");
    };
    arq_cb.on_state_changed = [this](ArqState state) {
        const char* sn[] = {"IDLE","LISTEN","HAIL","CONNECTING","CONNECTED","TURBO","DISCONNECTING"};
        IRIS_LOG("ARQ state -> %s", sn[(int)state]);
        if (state == ArqState::CONNECTED) {
            const bool fresh_session = !tx_compressor_.is_initialized() &&
                                       !rx_compressor_.is_initialized();
            if (fresh_session) {
                native_rx_record_.clear();
                native_completed_records_.clear();
                native_rx_record_type_ = ArqRecordType::Data;
                native_rx_record_id_ = 0;
                mlkem_rx_held_records_.clear();
            }
            // Init compressors for new session
            tx_compressor_.init();
            rx_compressor_.init();
            if (arq_.negotiated(CAP_STREAMING)) {
                tx_compressor_.streaming_enable();
                rx_compressor_.streaming_enable();
                if (!tx_compressor_.is_healthy() ||
                    !rx_compressor_.is_healthy()) {
                    fail_native_transform("negotiated dictionary initialization");
                    return;
                }
            }

            // Init encryption if negotiated
            if (arq_.negotiated(CAP_ENCRYPTION) && config_.encryption_mode > 0) {
                bool kx_ok = false;
                // Compute X25519 shared secret from peer's public key
                // (exchanged via CONNECT/CONNECT_ACK payloads)
                if (arq_.has_peer_x25519()) {
                    int rc = cipher_.compute_x25519_shared(arq_.peer_x25519_pubkey());
                    if (rc == 0) {
                        IRIS_LOG("[CRYPTO] X25519 DH key exchange complete");
                        kx_ok = true;
                    } else {
                        IRIS_LOG("[CRYPTO] X25519 shared secret computation failed (bad peer key?)");
                        crypto_state_ = 3;  // crypto failure
                    }
                } else {
                    IRIS_LOG("[CRYPTO] WARNING: peer advertised CAP_ENCRYPTION but no X25519 pubkey");
                    crypto_state_ = 1;  // KEY EXCHANGE incomplete
                }

                if (kx_ok) {
                    // Parse PSK from hex (authentication binding, not encryption key material)
                    std::vector<uint8_t> psk;
                    for (size_t i = 0; i + 1 < config_.psk_hex.size(); i += 2) {
                        char byte_str[3] = {config_.psk_hex[i], config_.psk_hex[i+1], 0};
                        psk.push_back((uint8_t)strtol(byte_str, nullptr, 16));
                    }

                    bool is_commander = (arq_.role() == ArqRole::COMMANDER);
                    crypto_direction_ = is_commander ? DIR_CMD_TO_RSP : DIR_RSP_TO_CMD;
                    cipher_.derive_session_key(config_.callsign.c_str(),
                                                arq_.remote_callsign().c_str(),
                                                psk.empty() ? nullptr : psk.data(),
                                                (int)psk.size(), false);
                    cipher_.activate();
                    tx_batch_counter_ = 0;
                    rx_batch_counter_ = 0;
                    crypto_state_ = 2;  // ENCRYPTED (X25519-only, upgrading to hybrid)
                    IRIS_LOG("[CRYPTO] Session encrypted (X25519 ECDH + ChaCha20-Poly1305)");

                    // Start ML-KEM-768 post-quantum upgrade (commander initiates)
                    if (arq_.negotiated(CAP_PQ_CRYPTO)) {
                        mlkem_kx_pending_ = true;
                        arq_.defer_transmit_commit();
                        if (arq_.role() == ArqRole::COMMANDER) {
                            start_mlkem_exchange();
                        }
                        // strict mode: don't release data until hybrid KX done
                        // fast mode: data flows immediately with X25519-only key
                    }
                } else if (config_.encryption_mode == 1) {
                    fail_strict_key_exchange("X25519 key exchange");
                    return;
                }
            } else if (config_.encryption_mode > 0) {
                crypto_state_ = 1;  // KEY EXCHANGE (wanted encryption but peer didn't negotiate)
                if (config_.encryption_mode == 1) {
                    fail_strict_key_exchange("strict encryption negotiation");
                    return;
                }
            }

            // Init B2F handler if negotiated
            if (arq_.negotiated(CAP_B2F_UNROLL)) {
                b2f_handler_.init();
                b2f_handler_.unroll_enabled = true;
            }

            // Native hail: both sides are proven Iris-capable, go native.
            if (config_.native_hail && !native_mode_) {
                native_mode_ = true;
                native_tx_ready_ = true;
                IRIS_LOG("Native hail: native mode active");
                // Cancel AX.25 session if it was still retrying SABM
                if (ax25_session_.state() == Ax25SessionState::AWAITING_CONNECTION) {
                    ax25_session_.reset();
                    IRIS_LOG("Native hail: cancelled AX.25 SABM (native connected)");
                }
            }
        } else if (state == ArqState::IDLE) {
            // This is a real native-link teardown.  A replacement session must
            // not inherit CFO ownership or soft evidence from the old peer.
            reset_ofdm_belief_and_chase();
            tx_compressor_.deinit();
            rx_compressor_.deinit();
            cipher_.wipe();
            crypto_state_ = 0;  // OFF
            mlkem_kx_pending_ = false;
            mlkem_releasing_tx_ = false;
            mlkem_release_buffered_ = false;
            mlkem_release_prepared_count_ = 0;
            mlkem_held_frames_.clear();
            mlkem_rx_held_records_.clear();
            native_rx_record_.clear();
            native_completed_records_.clear();
            native_rx_record_type_ = ArqRecordType::Data;
            native_rx_record_id_ = 0;
            b2f_handler_.deinit();
            b2f_proxy_originals_.clear();

            // Reset native mode so next session starts clean.
            // Guard: if relisten_pending_ is already set, this IDLE came from
            // listen()→reset()→set_state(IDLE) — don't re-trigger or clear buffer.
            if (config_.native_hail && !relisten_pending_) {
                native_mode_ = false;
                native_tx_ready_ = false;
                rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                pending_frame_timeout_ = 0;
                // Defer return to LISTENING (can't call listen() from
                // inside the state callback — would cause recursion).
                relisten_pending_ = true;
            }
        }
        if (state_callback_)
            state_callback_(state, arq_.remote_callsign());
    };
    arq_.set_callbacks(arq_cb);

    // Wire AX.25 connected mode session
    ax25_session_.set_local_callsign(config_.callsign);
    ax25_session_.set_txdelay_ms(config_.ptt_pre_delay_ms);  // Set T1 floor from initial TXDELAY
    ax25_session_.set_send_callback([this](const uint8_t* data, size_t len) {
        // Route session frames: native (OFDM) if upgrade complete, else AFSK.
        // Responder keeps ofdm_kiss_tx_=false until hearing first native frame,
        // so its session frames (RR etc) still go via AFSK during transition.
        // This is safe: the window is short (~1s) and AX.25 retry handles drops.
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        if (ofdm_kiss_tx_) {
            std::vector<uint8_t> frame(data, data + len);
            // Peer SNR feedback: append our RX SNR to S-frames (RR/REJ/RNR)
            // so the peer can cap its TX speed to what we can actually decode.
            // Encoding: 1 byte, SNR * 4 (0.25 dB steps), 0 = no data.
            // Only appended on native PHY — old peers or AFSK ignore extra bytes.
            // Fixed no-via shape (15-byte mod-8 or 16-byte extended, ctrl at 14):
            // a via-carrying S-frame fails this test and simply skips the Iris-only
            // tail — benign, and correct: the native PHY is Iris<->Iris DIRECT
            // (a digipeated session runs the AFSK branch below; endpoint-via
            // over the OFDM transport is deferred, DIGIPEATER_DESIGN.md sec 5.5-2).
            size_t sframe_len = ax25_session_.extended() ? 16 : 15;
            if (len == sframe_len && (data[14] & 0x03) == 0x01) {
                // Use DD (post-Kalman) SNR for peer feedback — with RTS smoother,
                // DD SNR accurately reflects decoded signal quality and is typically
                // higher than preamble SNR on FM links (preamble biased low by ISI)
                uint8_t snr_byte = 0;
                if (snr_db_ > 0)
                    snr_byte = (uint8_t)std::min(255.0f, std::max(1.0f, snr_db_ * 4.0f));
                frame.push_back(snr_byte);
                // Speed level signaling: tell peer our current TX level so they
                // can set their RX tone map without blind detection on every frame.
                // Byte 16 = our TX speed level (0-12).
                frame.push_back(static_cast<uint8_t>(ofdm_speed_level_));
                // Config-identity echo (item 5): bytes 17-18 =
                // 16-bit fingerprint of our RESOLVED OfdmConfig. The peer compares
                // it against its own on RX; a mismatch means the two ends resolved
                // different activation configs (nfft/cp/pilot/carrier-grid, incl.
                // band-edge asymmetry) and OFDM cannot interoperate -> fall back to
                // AFSK instead of delivering 0 bytes over a split-brain link. Live
                // fingerprint, Iris-only tail — legacy AFSK peers never see it.
                if (ofdm_config_.n_used_carriers > 0) {
                    uint16_t fp = ofdm_config_fingerprint(ofdm_config_);
                    frame.push_back((uint8_t)(fp & 0xFF));
                    frame.push_back((uint8_t)((fp >> 8) & 0xFF));
                    // #2 burst-epoch echo (last Iris-only tail byte): echo the
                    // last forward burst-epoch we decoded so the peer can bind THIS
                    // OFDM S-frame ACK to the burst it acks.  0xFF = ABSENT (none
                    // received yet) -> the peer's guard is fail-open.
                    frame.push_back(ofdm_kiss_rx_burst_epoch_ >= 0
                                        ? (uint8_t)ofdm_kiss_rx_burst_epoch_
                                        : (uint8_t)0xFF);
                }
            }
            // THE single tone-ACK-eligible producer: tag only the modem's own
            // autonomous RR (computed from the ORIGINAL 15-byte ctrl, before the
            // SNR/speed tail was appended above).  UA/DISC/DM/SABM/FRMR/RNR/REJ
            // and I-frames emitted through this same callback tag false and go
            // out via OFDM — never swallowed by the tone.  Route through
            // enqueue_native_tx_frame so a connect-recovery go-back-N burst
            // coalesces duplicate re-emissions instead of overflowing the 32-cap
            // queue and drop-oldest evicting a never-sent frame (P0 turnaround).
            bool tone_ack_eligible = tx_is_autonomous_rr(data, len);
            enqueue_native_tx_frame(std::move(frame), tone_ack_eligible);
        } else {
            constexpr size_t TX_QUEUE_MAX = 32;
            if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
                IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
                ax25_tx_queue_.pop();
            }
            ax25_tx_queue_.push(std::vector<uint8_t>(data, data + len));
            // Buffer I-frame info fields during AFSK phase for B2F replay.
            // The B2F handler isn't initialized until OFDM-KISS activates, but
            // the SID/FC/FS exchange happens during AFSK BEFORE the probe.
            // Buffer unconditionally so replay works when handler initializes.
            // Parse (not frame[14]/+16 fixed offsets): session I-frames can now
            // carry a digipeater via path, which shifts control/PID/info by 7
            // bytes per hop — the fixed-offset sniff misread a via frame's
            // address bytes as control (bit 0 of a shifted callsign char is
            // always 0, so EVERY via frame aliased as an I-frame) and fed
            // mid-address garbage into the B2F replay history.
            if (config_.b2f_unroll && len > 16) {
                Ax25Frame bf;
                if (ax25_parse(data, len, bf) &&
                    bf.type() == Ax25FrameType::I_FRAME && !bf.info.empty()) {
                    b2f_afsk_tx_history_.emplace_back(bf.info.begin(), bf.info.end());
                }
            }
        }
    });
    ax25_session_.set_data_callback([this](const uint8_t* data, size_t len) {
        if (rx_callback_) rx_callback_(data, len);
    });
    // Terminate/re-pack RX: the session fires this ONCE per strictly in-order R2
    // I-frame (V(R) is the sole dedup/order gate).  Feed the INFO to the length-
    // delimited reassembler -> re-originate exact records to the local pump.
    ax25_session_.set_native_stream_admission_callback([this](const uint8_t* info, size_t len) {
        if (!native_repack_active())
            return false;
        return repack_rx_ingest(info, len);
    });
    ax25_session_.set_acked_info_callback([this](const uint8_t* info, size_t len) {
        repack_note_air_ack(info, len);
    });
    // DATALINK_INTEGRITY_AUDIT §4/F1: on an unrecoverable N(R) desync, tear custody
    // down cleanly in re-pack mode (the AX.25 re-establish is a dead end there)
    // instead of the ~28 s SABM zombie + silent custody loss.
    ax25_session_.set_desync_callback([this]() { return handle_native_desync(); });
    ax25_session_.set_state_callback([this](Ax25SessionState state, const std::string& remote) {
        IRIS_LOG("AX25 state -> %d (remote=%s)", (int)state, remote.c_str());

        if (state == Ax25SessionState::CONNECTED && v2_live_session_ &&
            !ax25_session_.ordinary_timer_recovery_return())
            v2::CloseTransportOwner::revoke(*this);

        // Probe-after-connect: let SABM/UA complete first so Winlink is happy,
        // then probe while holding I-frames. Only the connection initiator
        // starts the probe; the responder waits for PROBE:START.
        if (state == Ax25SessionState::CONNECTED &&
            !config_.ax25_only && !ofdm_kiss_probing_ &&
            ofdm_kiss_probe_cd_ == 0 && !ofdm_kiss_probe_done_) {
            if (config_.force_ofdm) {
                // --force-ofdm: skip probe entirely, activate OFDM now.
                // Both sides must use --force-ofdm with matching config.
                force_activate_ofdm(remote);
            } else if (try_use_cached_probe(remote)) {
                // Cache hit — activate OFDM immediately.
                IRIS_LOG("Probe cache hit for %s — skipping probe", remote.c_str());
                if (gui_log_) gui_log_("Cached probe for " + remote);
                if (!ax25_session_.we_initiated()) {
                    // Responder: activate OFDM RX but hold off TX until we hear
                    // an OFDM frame from CMD. Without this, the UA gets routed
                    // through OFDM before CMD is ready to decode it.
                    ofdm_kiss_tx_ = false;
                    IRIS_LOG("Responder: OFDM RX active, TX held until CMD OFDM frame received");
                }
            } else if (ax25_session_.we_initiated()) {
                // Cache miss — initiator starts probe.
                // Defer PROBE:START — this callback may fire from queue_tx_frame().
                // Set flag and let tick() send it.
                xid_peer_call_ = remote;
                probe_start_pending_ = true;
                ofdm_kiss_probe_cd_ = 60;  // 3s countdown before tones (3 PROBE:START sends)
                // Pre-limit max_info to the OFDM O0 floor so I-frames queued
                // during probe don't exceed LDPC block size after migration (the
                // level starts at O0 post-migration). leg 3: single-sourced O0
                // floor (ofdm_max_info_for_level(0, ofdm_config_) == 75); grows once the anchor
                // climbs after activation.
                ax25_session_.set_max_info(ofdm_max_info_for_level(0, ofdm_config_));
                probe_connect_timeout_ = 1200;  // 60s overall timeout (READY handshake + AFSK latency)
                IRIS_LOG("Probe-after-connect: probing %s, I-frames held until done", remote.c_str());
                if (gui_log_) gui_log_("Probing " + remote + "...");
            }
        }

        // AWAITING_RELEASE (DISCONNECTING): immediately fall back to AX.25.
        // Disable native/OFDM-KISS so DISC/UA exchange uses AFSK only.
        // Start 30s timeout — if stuck, stop TX'ing entirely.
        if (state == Ax25SessionState::AWAITING_RELEASE) {
            IRIS_LOG("DISCONNECTING: falling back to AX.25 immediately");
            ofdm_kiss_ = false;
            ofdm_kiss_tx_ = false;
            ofdm_kiss_confirmed_ = false;
            ofdm_kiss_probing_ = false;
            ofdm_kiss_probe_cd_ = 0;
            probe_start_pending_ = false;
            native_mode_ = false;
            native_tx_ready_ = false;
            native_rx_gain_ = 1.0f;
            ofdm_phy_active_ = false;
            reset_ofdm_belief_and_chase();
            ofdm_rx_audio_buf_.clear();
            ofdm_acquisition_.reset();
            ofdm_active_candidate_id_ = 0;
            ofdm_rx_lpf_.reset();
            ofdm_rx_lpf_active_ = false;
            ofdm_sync_cached_ = false;
            disconnect_timeout_ticks_ = 600;  // 30s at 50ms/tick
        }

        if (state == Ax25SessionState::DISCONNECTED) {
            disconnect_timeout_ticks_ = 0;
            pending_connect_call_.clear();
            probe_connect_timeout_ = 0;
            if (!b2f_proxy_originals_.empty())
                fail_ofdm_transform("B2F proxy session ended before close proof",
                                    b2f_proxy_originals_);
            const bool had_open_custody = repack_has_open_custody();
            // Terminate/re-pack custody: if the session died with custody-ACKed
            // data still undelivered (air-link death mid-transfer), DISC the local
            // pump so it never reports success on lost bytes.  A clean completion
            // has already drained every buffer, so no teardown fires.  (repack_reset
            // clears state either way for the next session.)
            if (!had_open_custody && repack_engaged_ &&
                (!native_repack_tx_stream_.empty() ||
                 !native_repack_rx_records_.empty() ||
                 !native_repack_rx_stream_.empty() ||
                 ((repack_reorig_vs_ - repack_reorig_va_) & 0x07) != 0)) {
                repack_custody_teardown("session DISCONNECTED with undelivered data");
            } else if (!had_open_custody) {
                repack_reset();
            }
            if (ofdm_kiss_ || ofdm_kiss_tx_ || ofdm_kiss_probe_done_) {
                IRIS_LOG("KISS-over-OFDM disabled (AX.25 disconnected)");
                ofdm_kiss_ = false;
                ofdm_kiss_tx_ = false;
                ofdm_kiss_confirmed_ = false;
                ofdm_kiss_probing_ = false;
                ofdm_kiss_probe_cd_ = 0;
                ofdm_kiss_probe_done_ = false;
                probe_manual_ = false;
                probe_start_pending_ = false;
                probe_.reset();
                ofdm_phy_active_ = false;
                ofdm_mod_.reset();
                ofdm_demod_.reset();
                ofdm_rx_iq_.clear();
                ofdm_rx_audio_buf_.clear();
                ofdm_acquisition_.reset();
                ofdm_active_candidate_id_ = 0;
                ofdm_rx_lpf_.reset();
                ofdm_rx_lpf_active_ = false;
                ofdm_sync_cached_ = false;
                reset_ofdm_belief_and_chase();
                ofdm_txdelay_ms_ = 0;  // Reset adaptive TXDELAY on disconnect
                batch_airtime_s_ = BATCH_AIRTIME_MIN;
                reset_level_state();  // A4: atomic reset of every level-owning var
                // Clean up OFDM-KISS compression and B2F proxy
                if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
                    ofdm_kiss_tx_compressor_.deinit();
                    ofdm_kiss_rx_compressor_.deinit();
                }
                if (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) {
                    ofdm_kiss_b2f_.deinit();
                    b2f_proxy_plaintext_.clear();
                    b2f_proxy_plaintext_.shrink_to_fit();
                }
                b2f_afsk_tx_history_.clear();
                b2f_afsk_rx_history_.clear();
                ofdm_kiss_peer_caps_ = 0;
                rx_channel_eq_.reset();
                tx_channel_eq_.reset();
                b2f_proxy_active_ = false;
                b2f_proxy_rx_active_ = false;
                b2f_proxy_addr_valid_ = false;
                b2f_proxy_vr_ = 0;
            }

            // Restore original band/baud/center if probe changed them
            if (config_.band_low_hz != orig_band_low_hz_ ||
                config_.band_high_hz != orig_band_high_hz_ ||
                phy_config_.baud_rate != orig_phy_config_.baud_rate) {
                IRIS_LOG("Restoring original PHY: band %.0f-%.0f Hz, baud %d (was %.0f-%.0f Hz, baud %d)",
                         orig_band_low_hz_, orig_band_high_hz_, orig_phy_config_.baud_rate,
                         config_.band_low_hz, config_.band_high_hz, phy_config_.baud_rate);
                config_.band_low_hz = orig_band_low_hz_;
                config_.band_high_hz = orig_band_high_hz_;
                phy_config_ = orig_phy_config_;
                if (use_upconvert_) {
                    float center = (orig_band_low_hz_ + orig_band_high_hz_) / 2.0f;
                    upconverter_ = Upconverter(center, config_.sample_rate);
                    downconverter_ = Downconverter(center, config_.sample_rate);
                }
                native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
                native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
            }

            peer_is_iris_ = false;
            native_mode_ = false;
            dcd_inverted_ = false;  // re-detect each connection
            native_tx_ready_ = false;
            native_selfhear_guard_ = 0;  // Don't discard frames from next connection
            rx_overlap_buf_.clear();
            pending_frame_start_ = -1;
            pending_frame_timeout_ = 0;

            // Old-session transport and PHY cleanup must be complete before a
            // terminal callback can reenter and establish a replacement session.
            auto captured_session = v2_live_session_;
            if (had_open_custody &&
                finish_transfer_once(repack_failure_reason_hint_, std::nullopt,
                                     std::move(captured_session))) {
                if (ax25_state_callback_)
                    ax25_state_callback_(state, remote);
                return;
            }
        }

        // Forward all state changes immediately to the GUI/AGW layer.
        if (ax25_state_callback_)
            ax25_state_callback_(state, remote);
    });

    // Initialize FX.25 RS codecs (needed for both TX and RX)
    fx25_init();
    if (config_.fx25_mode > 0) {
        IRIS_LOG("FX.25 enabled: %d RS check bytes per frame", config_.fx25_mode);
    }

    // Digipeater role (config-gated, DEFAULT OFF — [Digipeat] in iris.ini).
    // AFSK->AFSK only.  The active config is logged so the repeater behavior
    // is never silent.
    {
        DigipeatConfig dc;
        dc.enabled = config_.digipeat_enable;
        dc.connected_mode = config_.digipeat_connected;
        dc.ui_mode = config_.digipeat_ui;
        dc.ui_alias = config_.digipeat_ui_alias;
        dc.ui_wide = config_.digipeat_ui_wide;
        dc.dedupe_ttl_s = config_.digipeat_dedupe_ttl_s;
        // Comma-separated call list; empty = the station Callsign-SSID.
        std::string calls = config_.digipeat_mycall;
        if (calls.empty()) {
            calls = config_.callsign;
            if (config_.ssid) calls += "-" + std::to_string(config_.ssid);
        }
        size_t p = 0;
        while (p < calls.size()) {
            size_t c = calls.find(',', p);
            if (c == std::string::npos) c = calls.size();
            std::string one = calls.substr(p, c - p);
            while (!one.empty() && one.front() == ' ') one.erase(one.begin());
            while (!one.empty() && one.back() == ' ') one.pop_back();
            if (!one.empty()) dc.mycalls.push_back(one);
            p = c + 1;
        }
        std::string derr = digipeater_.configure(dc);
        if (!derr.empty())
            IRIS_LOG("[DIGI] config ERROR: %s", derr.c_str());
        if (digipeater_.enabled())
            IRIS_LOG("[DIGI] digipeater ACTIVE %s", digipeater_.describe().c_str());
    }

    state_ = ModemState::IDLE;

    // Precook heavy DSP state (FFT plans + LDPC decode scratch) up front. init()
    // returns before the capture/playback threads are created and TX is gated on
    // activation, so this runs before any audio callback — nothing here ever runs
    // under modem_mutex_ on a real-time thread. See IRIS_STALL_PRECOOK_AUDIT.md §4.
    precook();

    return true;
}

// IRIS_STALL_PRECOOK_AUDIT.md §4 R1/R3 — build persistent DSP state once, off the
// audio callback thread. Idempotent (the pocketfft plan LRU cache and the LDPC
// scratch both no-op on repeat), so safe to call more than once.
void Modem::precook() {
    // R1: warm the pocketfft persistent-plan cache for the hot fixed FFT sizes
    // (1024 = OFDM demod/mod/sync, 512 = spectrum). Building the plan here means
    // the first frame on the capture thread finds a cached plan instead of
    // factorizing 1024 points + rebuilding twiddle tables under the lock. The
    // cache is a program-wide shared static (mutex-guarded), so warming it from
    // this thread warms it for the capture thread. Un-warmed sizes (e.g. the
    // variable Hilbert size) still build once on first use and cache thereafter.
    for (int n : {1024, 512}) {
        std::vector<std::complex<float>> warm(n, std::complex<float>(0.0f, 0.0f));
        fft_complex(warm.data(), n);
        ifft_complex(warm.data(), n);
    }

    // R3: size the persistent LDPC decode scratch for every rate up front.
    LdpcCodec::precook();
}

void Modem::shutdown() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (repack_has_open_custody()) {
        // Put the transport in its terminal state before publication.  Once the
        // terminal callback runs this invocation must not touch reentrant work.
        arq_.reset();
        tx_compressor_.deinit();
        rx_compressor_.deinit();
        b2f_handler_.deinit();
        native_rx_record_.clear();
        mlkem_held_frames_.clear();
        mlkem_rx_held_records_.clear();
        ptt_off();
        state_ = ModemState::IDLE;
        native_mod_.reset();
        native_demod_.reset();
        if (kalman_log_file_) {
            fclose(kalman_log_file_);
            kalman_log_file_ = nullptr;
        }
        finish_transfer_once(v2::TransferResultReason::DeviceStopped);
        return;
    }
    arq_.reset();
    tx_compressor_.deinit();
    rx_compressor_.deinit();
    b2f_handler_.deinit();
    native_rx_record_.clear();
    mlkem_held_frames_.clear();
    mlkem_rx_held_records_.clear();
    ptt_off();
    state_ = ModemState::IDLE;
    native_mod_.reset();
    native_demod_.reset();
    if (kalman_log_file_) {
        fclose(kalman_log_file_);
        kalman_log_file_ = nullptr;
    }
}

void Modem::ptt_on() {
    if (!ptt_active_ && ptt_) {
        ptt_->set_ptt(true);
        ptt_active_ = true;
        rx_muted_ = true;
        // Pause AX.25 timers while we're transmitting (Direwolf pattern).
        // We can't expect an ACK while PTT is keyed.
        ax25_session_.set_channel_busy(true);
    }
}

void Modem::ptt_off() {
    if (ptt_active_ && ptt_) {
        ptt_->set_ptt(false);
        ptt_active_ = false;
        rx_mute_holdoff_ = RX_MUTE_HOLDOFF_SAMPLES;
        rx_raw_rms_ = 0;   // Clear stale DCD reading from our own TX
        dcd_holdoff_ = 0;

        // Post-TX listen window: defer next TX to let peer respond.
        // FM: turnaround is fast (~150ms PTT relay + radio switching).
        // OFDM sync (ZC preamble) handles frame boundaries — no need for
        // HF-style long listen windows.
        if (!loopback_mode_ && ax25_session_.is_active()) {
            static thread_local std::minstd_rand rng(std::random_device{}());
            int base, jitter;
            if (ofdm_kiss_tx_) {
                // FM OFDM: ~150ms radio turnaround + small margin
                jitter = rng() % (config_.sample_rate / 16);  // 0-62ms
                base = ax25_session_.we_initiated()
                    ? config_.sample_rate / 5           // initiator: 200ms
                    : config_.sample_rate / 6;          // responder: 167ms
            } else {
                jitter = rng() % (config_.sample_rate / 4);  // 0-250ms
                base = ax25_session_.we_initiated()
                    ? config_.sample_rate               // initiator: 1.0s
                    : config_.sample_rate * 3 / 4;      // responder: 0.75s
            }
            csma_holdoff_ = std::max(csma_holdoff_.load(), base + jitter);
        }

        // Resume AX.25 timers now that channel is free
        ax25_session_.set_channel_busy(false);
    }
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Deferred relisten: return to LISTENING after failed hail
    // (can't call from inside state callback due to recursion)
    if (relisten_pending_) {
        relisten_pending_ = false;
        arq_listen();
    }

    // Compute raw (pre-AGC) RMS for DCD — must be BEFORE early returns
    // so DCD always has fresh data. Skip during TX/mute: our own signal
    // would falsely trigger DCD.
    if (!ptt_active_ && !rx_muted_) {
        float raw_sq = 0;
        for (int i = 0; i < frame_count; i++)
            raw_sq += rx_audio[i] * rx_audio[i];
        rx_raw_rms_ = std::sqrt(raw_sq / std::max(frame_count, 1));
    }

    // In loopback mode, skip TX mute — the delay buffer handles timing
    if (!loopback_mode_) {
        if (state_ == ModemState::TX_AX25 || state_ == ModemState::TX_NATIVE)
            return;

        // Half-duplex holdoff after TX
        if (rx_muted_) {
            if (rx_mute_holdoff_ > 0) {
                rx_mute_holdoff_ -= frame_count;
                return;
            }
            rx_muted_ = false;
            rx_overlap_buf_.clear();
            ofdm_rx_audio_buf_.clear();  // Clear stale audio from TX period
            ofdm_acquisition_.reset();
            ofdm_active_candidate_id_ = 0;
            ofdm_rx_lpf_.reset();       // Reset filter state (no transient from stale history)
            ofdm_sync_cached_ = false;
            // RX mute expiry is an in-session TX->RX turnaround.  Link belief,
            // Chase identity/storage, and retained owner evidence survive it;
            // only acquisition state tied to self-heard audio is discarded.
            pending_frame_start_ = -1;
            pending_frame_timeout_ = 0;
            if (ofdm_kiss_tx_ && ofdm_expect_ack_) {
                IRIS_LOG("[MAC-DIAG] RX-UNMUTE: listening for RR (guard=%dms remaining)",
                         native_selfhear_guard_.load() * 1000 / config_.sample_rate);
            }
        }
    }

    // Count down self-hear guard (only runs after rx_mute clears — sequential, not parallel)
    if (native_selfhear_guard_ > 0) {
        native_selfhear_guard_ -= frame_count;
        if (native_selfhear_guard_ <= 0) {
            native_selfhear_guard_ = 0;
            // Guard expired: clear any residual audio from countdown granularity
            ofdm_rx_audio_buf_.clear();
            ofdm_acquisition_.reset();
            ofdm_active_candidate_id_ = 0;
            ofdm_rx_lpf_.reset();
            ofdm_sync_cached_ = false;
            if (ofdm_expect_ack_)
                IRIS_LOG("[MAC-DIAG] SELFHEAR-EXPIRED: now listening for RR");
        }
    }

    // Feed probe analyzer after mute guard — never capture our own TX.
    if (ofdm_kiss_probing_ && probe_.state() == ProbeState::LISTENING_PROBE && !ptt_active_)
        probe_.feed_rx(rx_audio, frame_count);

    // Auto-DCD baseline calibration: measure noise floor for 2 seconds at startup.
    // Once baseline is known, detect inverted squelch (signal < noise).
    if (config_.dcd_auto && !dcd_baseline_done_ && !ptt_active_ && !rx_muted_) {
        dcd_baseline_samples_ += frame_count;
        // Exponential moving average of RMS
        float alpha = (dcd_baseline_rms_ == 0) ? 1.0f : 0.1f;
        dcd_baseline_rms_ = dcd_baseline_rms_ * (1.0f - alpha) + rx_raw_rms_ * alpha;
        if (dcd_baseline_samples_ >= config_.sample_rate / 2) {  // 0.5 seconds (was 2s)
            dcd_baseline_done_ = true;
            IRIS_LOG("Auto-DCD: baseline RMS=%.4f (measured over 2s)", dcd_baseline_rms_);
            if (gui_log_) gui_log_("Auto-DCD: baseline " + std::to_string(dcd_baseline_rms_));
        }
    }

    // Pre-allocated RX audio buffer (avoids per-callback heap allocation)
    if (rx_audio_tmp_.size() < (size_t)frame_count)
        rx_audio_tmp_.resize(frame_count);
    std::copy(rx_audio, rx_audio + frame_count, rx_audio_tmp_.data());
    float* audio = rx_audio_tmp_.data();
    // AGC for AX.25 mode only — native mode has preamble-based gain estimation
    // AGC would distort QAM symbols within a frame (gain changes during preamble vs payload)
    if (!native_mode_)
        agc_.process_block(audio, frame_count);

    float sum_sq = 0;
    float peak = 0;
    for (int i = 0; i < frame_count; i++) {
        sum_sq += audio[i] * audio[i];
        float a = std::fabs(audio[i]);
        if (a > peak) peak = a;
    }
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));
    // Peak with slow decay (~1s at 48kHz/1024 frame_count ≈ 47 frames/s)
    if (peak > rx_peak_) rx_peak_ = peak;
    else rx_peak_ *= 0.95f;

    // Periodic RX diagnostic (~every 5 seconds)
    rx_diag_counter_ += frame_count;
    if (rx_diag_counter_ >= config_.sample_rate * 5) {
        rx_diag_counter_ = 0;
        float best = detect_best_corr();
        IRIS_LOG("RX: rms=%.4f overlap=%zu corr=%.3f native=%d",
                 rx_rms_, rx_overlap_buf_.size(), best, native_mode_ ? 1 : 0);
    }

    if (!ptt_active_)
        compute_spectrum(audio, frame_count);

    if (state_ == ModemState::CALIBRATING) {
        process_calibration_rx(audio, frame_count);
        return;
    }

    if (native_mode_) {
        process_rx_native(audio, frame_count);
    } else {
        process_rx_ax25(audio, frame_count);

        // Always run native demod alongside AFSK — it's lightweight (just
        // preamble correlation until a frame arrives) and lets us detect native
        // frames at any point without waiting for the handshake to complete.
        // Self-hear is handled by native_selfhear_guard_.
        if (!config_.ax25_only && native_demod_) {
            process_rx_native(audio, frame_count);
        }
    }
}

// Dispatch a decoded AX.25 frame (shared by HDLC, FX.25, and OFDM-KISS decoders)
void Modem::dispatch_rx_frame(const std::vector<uint8_t>& frame, bool from_fx25, bool from_ofdm) {
    // Dedup: HDLC and FX.25 decoders run in parallel on the same bit stream,
    // so every FX.25 frame also decodes as plain HDLC.  Without dedup the KISS
    // client sees every frame twice, causing AX.25 protocol errors (duplicate
    // UA → reset, duplicate I-frame → REJ).
    if (dedup_cooldown_ > 0 && frame == last_rx_frame_) {
        // Same frame within the dedup window — log it but don't dispatch
        if (packet_log_ && frame.size() >= 14) {
            std::string proto = from_fx25
                ? "FX.25 (" + std::to_string(fx25_decoder_.last_rs_errors()) + " RS corr)"
                : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
            packet_log_(false, proto + " [dup]", describe_ax25(frame.data(), frame.size()));
        }
        return;
    }
    last_rx_frame_ = frame;
    dedup_cooldown_ = config_.sample_rate * 3 / 2;  // 1.5s dedup window (FX.25 with heavy RS can arrive ~1s after HDLC)
    ax25_session_.set_reverse_ack_epoch_ok(true);

    // Session-boundary U-frames are intercepted before carrier selection and
    // raw KISS forwarding.  They cannot bypass the custody terminal decision
    // merely because release fell back from OFDM to AFSK.
    Ax25Frame boundary_frame;
    const bool parsed_boundary =
        ax25_parse(frame.data(), frame.size(), boundary_frame) &&
        boundary_frame.type() == Ax25FrameType::U_FRAME;
    if (parsed_boundary && repack_has_open_custody()) {
        const Ax25UType type = boundary_frame.u_type();
        const bool addressed_to_us =
            boundary_frame.dst.matches(config_.callsign);
        const bool from_current_peer =
            boundary_frame.src.matches(repack_remote_call_);
        if (type == Ax25UType::SABM && addressed_to_us) {
            repack_failure_reason_hint_ =
                v2::TransferResultReason::SessionReplaced;
            ax25_session_.reset();
            return;
        }
        if ((type == Ax25UType::UA || type == Ax25UType::DM) &&
            addressed_to_us && from_current_peer &&
            ax25_session_.state() == Ax25SessionState::AWAITING_RELEASE) {
            // An ordinary AX.25 release response carries no transfer identity,
            // frozen final boundary, challenge, or MatchingCloseProof.  Let the
            // session close, but fail the accepted transfer and consume the raw
            // frame so it cannot masquerade as a success response to the client.
            repack_failure_reason_hint_ = type == Ax25UType::UA
                ? v2::TransferResultReason::ProtocolViolation
                : v2::TransferResultReason::PeerLost;
            ax25_session_.on_frame_received(boundary_frame);
            return;
        }
        if (type == Ax25UType::DISC && addressed_to_us && from_current_peer) {
            const bool rx_open = repack_rx_transfer_ledger_ &&
                !repack_rx_transfer_ledger_->terminal_published;
            if (rx_open) {
                if (!repack_air_disc_pending_) {
                    repack_air_disc_pending_ = true;
                    repack_air_disc_delivered_ = false;
                    repack_air_disc_frame_ = frame;
                    repack_air_disc_hold_ticks_ = 0;
                }
                repack_rx_drain();
                return;
            }
            repack_failure_reason_hint_ = v2::TransferResultReason::PeerLost;
            ax25_session_.on_frame_received(boundary_frame);
            return;
        }
    }

    // A decoded native OFDM frame from the peer positively confirms it activated
    // OFDM — cancel any responder RESULT re-announce still running so we stop
    // spending AFSK airtime once the native session is up (D3).
    if (from_ofdm && probe_.reannounce_active())
        probe_.stop_result_reannounce();

    // When OFDM-KISS is fully bidirectional, suppress AFSK-decoded session
    // frames — only OFDM-decoded frames should feed the AX.25 session.
    // CRITICAL: require ofdm_kiss_confirmed_ (we've heard a native frame from the peer).
    // Without this, the initiator goes deaf to the responder's AFSK frames
    // before the responder has activated native TX.
    if (ofdm_kiss_ && ofdm_kiss_confirmed_ && !from_ofdm) {
        // Check if it's a UI frame — those still need AFSK path for conn header
        bool is_ui = (frame.size() > 15 &&
                      (frame[14] & ~AX25_PF_MASK) == AX25_CTRL_UI);
        if (!is_ui) {
            // Drop non-UI AFSK frames — OFDM handles session traffic now
            return;
        }
    }

    // CSMA: holdoff after frame decode to avoid stepping on a burst.
    // Role-asymmetric: responder ACKs quickly, initiator yields longer.
    // Resets on each frame, so holdoff extends past the LAST frame in a burst.
    //
    // Native data I-frames get a longer holdoff on the responder side:
    // the sender's self-hear guard (300ms) plus FM turnaround (~200ms) means
    // a premature RR arriving within ~500ms of sender's TX end gets discarded.
    // 800ms covers the inter-frame gap in a multi-frame burst and ensures
    // the RR only goes out after the burst is truly done.
    {
        int rx_holdoff;
        bool native_data = from_ofdm && frame.size() > 15 &&
                           (frame[14] & 0x01) == 0;  // I-frame over native
        if (ax25_session_.is_active()) {
            if (ofdm_kiss_tx_) {
                // FM OFDM: burst-done holdoff. The ofdm_sync_cached_ TX guard
                // blocks during frame accumulation. This holdoff covers the brief
                // window between frame decode and next preamble detection.
                rx_holdoff = config_.sample_rate / 5;  // 200ms
            } else if (native_data && !ax25_session_.we_initiated()) {
                // Responder receiving native I-frame: burst holdoff
                // Self-hear guard (300ms) + FM turnaround (~200ms) + margin
                rx_holdoff = config_.sample_rate * 4 / 5;  // 800ms
            } else {
                rx_holdoff = ax25_session_.we_initiated()
                    ? config_.sample_rate * 4 / 5   // initiator: 800ms
                    : config_.sample_rate * 2 / 5;  // responder: 400ms (ACK quickly)
            }
        } else {
            rx_holdoff = config_.sample_rate * 4 / 5;  // no session: 800ms
        }
        csma_holdoff_ = std::max(csma_holdoff_.load(), rx_holdoff);
    }

    frames_rx_++;

    // Auto-DCD inverted squelch detection: when we successfully decode a frame,
    // check if the current RMS is significantly below the baseline noise floor.
    // If so, the radio has inverted squelch (signal present = less noise).
    // Runs for ALL frame types (AFSK and native) so we detect before native upgrade.
    if (config_.dcd_auto && dcd_baseline_done_ && !dcd_inverted_ && rx_raw_rms_ > 0) {
        if (rx_raw_rms_ < dcd_baseline_rms_ * 0.5f) {
            dcd_inverted_ = true;
            IRIS_LOG("Auto-DCD: inverted squelch detected (signal RMS=%.4f < baseline=%.4f)",
                     rx_raw_rms_.load(), dcd_baseline_rms_);
            if (gui_log_) gui_log_("Auto-DCD: inverted squelch detected");
        }
    }

    // Log to packet viewer
    if (packet_log_ && frame.size() >= 14) {
        std::string proto;
        if (from_fx25) {
            int errs = fx25_decoder_.last_rs_errors();
            proto = "FX.25 (" + std::to_string(errs) + " RS corr)";
        } else {
            proto = config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200";
        }
        packet_log_(false, proto, describe_ax25(frame.data(), frame.size()));
    }

    // Try ARQ session first
    {
        ArqState arq_st = arq_.state();
        if (arq_st != ArqState::IDLE) {
            if (arq_.on_frame_received(frame.data(), frame.size()))
                return;
        }
    }

    // Adaptive batch airtime: sniff S-frames before session consumes them.
    // RR = peer ACK'd our batch → grow.  REJ = peer lost a frame → shrink.
    if (ofdm_kiss_ && frame.size() > 14) {
        uint8_t ctrl = frame[14];
        // WIDE WINDOW: an owned OFDM I/S frame carries a 2-octet (modulo-128) control,
        // so N(R) lives in octet-2 (bits 1-7) and the Iris metadata tail (SNR/level/
        // fp/epoch) shifts +1.
        bool is_ext = from_ofdm && ax25_session_.extended() && ((ctrl & 0x03) != 0x03);
        size_t meta_offset = is_ext ? 16 : 15;
        uint8_t nr_sniff = is_ext ? (uint8_t)((frame.size() >= 16 ? frame[15] : 0) >> 1) & 0x7F
                                  : (uint8_t)((ctrl >> 5) & 0x07);
        if ((ctrl & 0x03) == 0x01) {  // S-frame
            auto stype = (Ax25SType)((ctrl >> 2) & 0x03);
            if (stype == Ax25SType::RR) {
                // Only count as real ACK if N(R) advanced (new data acknowledged).
                // Poll responses with unchanged N(R) mean the peer is alive but
                // NOT decoding our data — don't reset the no-ack counter, so the
                // gearshift eventually downshifts on asymmetric decode failures.
                uint8_t nr = nr_sniff;
                if (nr != last_peer_nr_) {
                    float prev = batch_airtime_s_;
                    batch_airtime_s_ = std::min(batch_airtime_s_ + 1.0f, BATCH_AIRTIME_MAX);
                    if (batch_airtime_s_ != prev)
                        IRIS_LOG("Batch airtime: %.0fs -> %.0fs (RR ACK)", prev, batch_airtime_s_);
                    tx_no_ack_count_ = 0;  // peer acknowledged NEW data
                    ofdm_anchor_futility_ = 0;  // forward progress — not futile
                    // A3: raise the anchor to the MAX level among the frames this
                    // N(R) actually ACKs — the outstanding window may span a
                    // climb/leap over 2-3 levels, which the scalar tx_last_level_
                    // (most-recent batch) cannot represent. va() here is the
                    // pre-ACK window base (this sniff runs before on_frame_received).
                    int acked_lvl = ring_acked_and_clear(ax25_session_.va(), nr);
                    if (acked_lvl > tx_acked_level_) {
                        IRIS_LOG("MAC: tx_acked_level O%d -> O%d (peer RR N(R)=%d)",
                                 tx_acked_level_, acked_lvl, nr);
                        tx_acked_level_ = acked_lvl;
                        // leg 3: grow MAX_INFO with the confirmed anchor so larger
                        // I-frames fill the higher level's frame (a GROW never
                        // orphans a window frame — no drop needed).
                        ax25_session_.set_max_info(ofdm_max_info_for_level(tx_acked_level_, ofdm_config_));
                    }
                    last_peer_nr_ = nr;
                    // Cache proven speed level for this peer
                    gearshift_.save_cached_level(ax25_session_.remote_callsign());
                }
                // else: RR with same N(R) = poll response, don't reset no-ack counter
            } else if (stype == Ax25SType::REJ || stype == Ax25SType::RNR) {
                float prev = batch_airtime_s_;
                batch_airtime_s_ = std::max(batch_airtime_s_ / 2.0f, BATCH_AIRTIME_MIN);
                if (batch_airtime_s_ != prev)
                    IRIS_LOG("Batch airtime: %.0fs -> %.0fs (%s)", prev, batch_airtime_s_,
                             stype == Ax25SType::REJ ? "REJ" : "RNR");
                // REJ/RNR = peer couldn't decode our frame. Downshift TX speed.
                // On asymmetric links, local RX SNR doesn't predict peer's RX SNR.
                // Treating peer REJ as a decode failure drives gearshift down so we
                // TX at a rate the peer can actually receive.
                tx_no_ack_count_ = 0;  // peer responded (even if negative)
                // Peer couldn't decode → lower the TX ceiling. Shared with the
                // anchor-futility trigger (demote_tx_anchor).
                // F2 (DATALINK_INTEGRITY_AUDIT §4): DEFER the window mutation onto
                // INV-SEQ-2's quiesce, exactly like the futility path. The anchor +
                // MAX_INFO still shrink INLINE inside demote_tx_anchor (a real REJ
                // must lower the ceiling NOW), but the destructive V(S) rollback /
                // re-fragment is HELD until the in-flight window resolves. This sniff
                // runs BEFORE on_frame_received, so an inline harvest would roll V(S)
                // back to the PRE-ack V(A) — re-using seq numbers the peer already
                // counted (the s00/s02 crossover class) and, at wide K, splicing an
                // old/new slicing into the peer reassembler. Deferring lets the REJ's
                // own N(R) advance V(A) first, then quiesces the flight before any
                // re-slice (consume_pending_window_refrag).
                demote_tx_anchor(stype == Ax25SType::REJ ? "peer REJ" : "peer RNR",
                                 /*defer_window_mutation=*/true);
                gearshift_.report_failure();
                IRIS_LOG("Gearshift: peer %s -> report_failure (level=%d)",
                         stype == Ax25SType::REJ ? "REJ" : "RNR",
                         gearshift_.current_level());
            }
            // Peer SNR feedback: extract appended SNR byte from native S-frames.
            // Standard S-frame is 15 bytes. If 16+ bytes, byte 15 is quantized
            // peer RX SNR (0.25 dB steps). This tells us what the peer measures
            // from our signal — use it to cap TX speed on asymmetric links.
            if (from_ofdm && frame.size() > meta_offset && frame[meta_offset] > 0) {
                float reported_snr = frame[meta_offset] / 4.0f;
                peer_snr_db_ = reported_snr;
                IRIS_LOG("Peer SNR feedback: %.1f dB (from %s)", reported_snr,
                         stype == Ax25SType::RR ? "RR" : "REJ/RNR");
            }
            // Speed level signaling: byte 16 = peer's current TX speed level.
            // Update our RX expectation so the next decode uses the right tone map.
            // This eliminates blind detection during steady state.
            if (from_ofdm && frame.size() > meta_offset + 1) {
                int peer_level = frame[meta_offset + 1];
                if (peer_level >= 0 && peer_level < NUM_OFDM_SPEED_LEVELS) {
                    if (peer_level != ofdm_kiss_rx_level_) {
                        IRIS_LOG("Peer speed level: O%d -> O%d (from %s)",
                                 ofdm_kiss_rx_level_, peer_level,
                                 stype == Ax25SType::RR ? "RR" : "REJ/RNR");
                        ofdm_kiss_rx_level_ = peer_level;
                        ofdm_rx_tone_map_ = ofdm_rx_tone_map_for_level(
                            peer_level, ofdm_config_, config_.ofdm_nuc);
                    }
                    ofdm_kiss_rx_confirmed_ = true;
                }
            }
            // Config-identity echo (item 5): bytes 17-18 carry
            // the peer's resolved-OfdmConfig fingerprint. If it differs from ours,
            // the two ends resolved DIFFERENT activation configs (nfft/cp/pilot/
            // carrier-grid, incl. band-edge asymmetry D5b that the seed fix cannot
            // catch) -> OFDM cannot interoperate. Fall back to AFSK rather than
            // deliver 0 bytes over a split-brain OFDM link. One-shot latch; the
            // local action is safe (stop OFDM TX, don't re-promote) — unacked
            // I-frames retransmit via AFSK and new session frames route via AFSK.
            if (from_ofdm && ofdm_kiss_ && !ofdm_config_mismatch_ &&
                frame.size() > meta_offset + 3 && ofdm_config_.n_used_carriers > 0) {
                uint16_t peer_fp = (uint16_t)frame[meta_offset + 2] |
                                   ((uint16_t)frame[meta_offset + 3] << 8);
                uint16_t local_fp = ofdm_config_fingerprint(ofdm_config_);
                if (peer_fp != 0 && peer_fp != local_fp) {
                    int fb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.front();
                    int lb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.back();
                    IRIS_LOG("[OFDM-CFG] MISMATCH: peer fp=0x%04X != local fp=0x%04X "
                             "(local nfft=%d cp=%d used=%d bins %d-%d) -> AFSK fallback",
                             peer_fp, local_fp, ofdm_config_.nfft, ofdm_config_.cp_samples,
                             ofdm_config_.n_used_carriers, fb, lb);
                    if (gui_log_) gui_log_("[OFDM-CFG] config fingerprint mismatch -> AFSK fallback");
                    ofdm_config_mismatch_ = true;   // latch: prevents RX re-promotion below
                    ofdm_kiss_tx_ = false;          // stop OFDM TX; session frames route via AFSK
                    // Purge the peer's probe-cache entry (memory + disk) so the
                    // next reconnect's try_use_cached_probe (modem.cc:6565) can't
                    // replay the split OfdmConfig AND reset ofdm_config_mismatch_
                    // back to false — otherwise the AFSK fallback never sticks
                    // (D4 cache poison). Forces a fresh probe to re-resolve config.
                    purge_probe_cache(ax25_session_.remote_callsign());
                }
            }
            // #2 burst-epoch guard for the OFDM S-frame reverse-ACK carrier. The
            // last Iris-tail byte echoes the forward burst epoch the peer decoded.
            if (from_ofdm && frame.size() > meta_offset + 4) {
                uint8_t echo = frame[meta_offset + 4];
                bool ep_ok = reverse_ack_epoch_matches(echo, tx_burst_epoch_);
                if (!ep_ok)
                    IRIS_LOG("[REV-ACK] OFDM S-frame epoch MISMATCH: echo=%d expect=%d "
                             "-> advisory (hold V(A))", echo,
                             (int)tx_burst_epoch_);
                ax25_session_.set_reverse_ack_epoch_ok(ep_ok);
            }
            // Event-driven MAC: receiving a valid S-frame from the peer proves
            // the channel is clear. Cancel CMD's post-TX holdoff so the next
            // data burst starts immediately rather than waiting out the timer.
            if (ofdm_kiss_tx_ && ax25_session_.we_initiated()) {
                csma_holdoff_ = 0;
                // Drain stale S-frames (T1 polls) from tx_queue_. T1 may have
                // fired and queued an RR P=1 poll while we were waiting for this
                // ACK. Now that the ACK arrived, the poll is stale — transmitting
                // it wastes 544ms of airtime. This makes T1<holdoff viable.
                {
                    int drained = 0;
                    std::queue<TxFrame> keep;
                    while (!tx_queue_.empty()) {
                        auto& front = tx_queue_.front();
                        // Drop ONLY the modem's own stale autonomous-RR polls
                        // (provenance, not content) so aliasing B2F chunks and
                        // queued REJ/RNR/client/U-frames survive -- see
                        // tx_frame_is_stale_poll() (modem.h).
                        bool is_stale = tx_frame_is_stale_poll(front);
                        if (is_stale) {
                            drained++;
                        } else {
                            keep.push(std::move(front));
                        }
                        tx_queue_.pop();
                    }
                    tx_queue_ = std::move(keep);
                    if (drained > 0)
                        IRIS_LOG("MAC: drained %d stale S-frame(s) from tx_queue_", drained);
                }
                IRIS_LOG("MAC: csma cancelled on peer %s (N(R)=%d)",
                         stype == Ax25SType::RR ? "RR" :
                         stype == Ax25SType::REJ ? "REJ" : "RNR",
                         nr_sniff);
            }
        } else if ((ctrl & 0x01) == 0x00) {  // I-frame
            // I-frames carry N(R) that implicitly acknowledges our data.
            // The S-frame sniff above misses this — if the peer sends data
            // instead of explicit RR, we still need to track their N(R).
            uint8_t nr = nr_sniff;
            if (nr != last_peer_nr_) {
                tx_no_ack_count_ = 0;  // peer acknowledged via I-frame N(R)
                ofdm_anchor_futility_ = 0;  // forward progress — not futile
                int acked_lvl = ring_acked_and_clear(ax25_session_.va(), nr);  // A3
                if (acked_lvl > tx_acked_level_) {
                    IRIS_LOG("MAC: tx_acked_level O%d -> O%d (peer I-frame N(R)=%d)",
                             tx_acked_level_, acked_lvl, nr);
                    tx_acked_level_ = acked_lvl;
                    // leg 3: grow MAX_INFO with the confirmed anchor (I-frame N(R)).
                    ax25_session_.set_max_info(ofdm_max_info_for_level(tx_acked_level_, ofdm_config_));
                }
                last_peer_nr_ = nr;
            }
        }
    }

    // Buffer RX I-frame info fields during AFSK phase for B2F replay.
    // Same reason as TX: SID/FC/FS exchange happens over AFSK before OFDM-KISS activates.
    // Buffer unconditionally (config check only) — ofdm_kiss_ isn't set until after probe.
    // Parse (not frame[14]/+16 fixed offsets): AFSK frames returning through a
    // digipeater carry a via path (H-bits set) that shifts control/info by 7
    // bytes per hop; the fixed-offset sniff misclassified EVERY via frame as an
    // I-frame (bit 0 of a shifted callsign char is 0) and buffered address bytes.
    if (config_.b2f_unroll && !from_ofdm && !ofdm_kiss_b2f_.is_initialized() && frame.size() > 16) {
        Ax25Frame bf;
        if (ax25_parse(frame.data(), frame.size(), bf) &&
            bf.type() == Ax25FrameType::I_FRAME && !bf.info.empty()) {
            b2f_afsk_rx_history_.emplace_back(bf.info.begin(), bf.info.end());
        }
    }

    // B2F proxy: feed incoming I-frame info fields to filter_rx for state tracking.
    // This drives B2F state transitions (detects FS responses, FC proposals, etc.)
    // Must run BEFORE ax25_session consumes the frame.
    if (ofdm_kiss_ && from_ofdm && (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
        ofdm_kiss_b2f_.is_initialized() && frame.size() > 16) {
        Ax25Frame bf;
        if (ax25_parse(frame.data(), frame.size(), bf,
                       ax25_session_.extended()) &&
            bf.type() == Ax25FrameType::I_FRAME && !bf.info.empty()) {
            auto tracked = ofdm_kiss_b2f_.filter_rx_record(
                bf.info.data(), bf.info.size());
            if (tracked.status == v2::TransformStatus::Failed) {
                fail_ofdm_transform("B2F proxy receive state tracking", {frame});
                return;
            }
            // Output ignored — original I-frame delivered to Winlink as-is
        }
    }

    // Try AX.25 connected mode session
    {
        // Terminate/re-pack: engage (absorb held frames + window rollback) the
        // instant OFDM is the transport, BEFORE the first R2 frame is processed, so
        // the reassembler never ingests a verbatim seam frame.
        if (native_repack_active()) repack_maybe_engage();
        bool native_active = (arq_.state() != ArqState::IDLE &&
                              arq_.state() != ArqState::LISTENING);
        if (!native_active || ax25_session_.is_active()) {
            Ax25Frame ax_frame;
            // WIDE WINDOW: owned R2 I/S frames arriving over OFDM carry a 2-octet
            // (modulo-128) control field once the session is extended.  Parse them
            // extended so the 7-bit N(S)/N(R) decode correctly.  U-frames (SABM/UA/
            // DISC) stay 1 octet regardless; non-OFDM (AFSK/local) frames stay mod-8.
            // The session only enters extended mode at repack_engage_at_activation(),
            // which runs (above, same dispatch) BEFORE the first owned frame is parsed.
            bool ext = from_ofdm && ax25_session_.extended();
            if (ax25_parse(frame.data(), frame.size(), ax_frame, ext)) {
                bool consumed = ax25_session_.on_frame_received(ax_frame);
                // #2: reset the epoch guard to its fail-open default so it never
                // leaks to a later frame (it was set only for THIS S-frame's dispatch).
                ax25_session_.set_reverse_ack_epoch_ok(true);
                // Terminate/re-pack: a reverse ACK (M0) may have freed the owned R2
                // window -> drain more of the TX stream.  (RX reassembly + re-orig
                // are driven by the native_stream_rx_ callback fired inside
                // on_frame_received above.)
                if (native_repack_active())
                    repack_tx_drain(false);
                if (consumed)
                    return;
            }
        }
    }

    // Digipeater (config-gated, DEFAULT OFF — [Digipeat] in iris.ini): if this
    // frame asks a station we digipeat for to repeat it — the FIRST via hop
    // whose has-been-repeated (H) bit is still clear matches a configured
    // digipeat callsign — set that H-bit and re-emit on the AFSK TX path.
    // AFSK->AFSK ONLY (no OFDM cross-mode relay; gated on the RX tone-map
    // root fix, DIGIPEATER_DESIGN.md §5.4).  Connected-mode I/S/U frames use
    // the cdigipeater.c rule (exact match, NO dedup — sequence numbers make
    // connected AX.25 self-deduping; a dedup here would eat legitimate
    // I-frame retransmissions); UI frames use the digipeater.c rule (30 s
    // dedup + alias/WIDEn-N).  A frame consumed by the session above is
    // addressed to us (dst match) and so is never a digipeat request —
    // reaching here is the correct gate.  Note the double-decode dedup at the
    // top of this function already keeps the parallel HDLC+FX.25 decode of
    // ONE transmission from being digipeated twice.
    if (!from_ofdm && digipeater_.enabled()) {
        int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        auto rpt = digipeater_.digipeat(frame.data(), frame.size(), now_ms);
        if (!rpt.empty()) {
            constexpr size_t TX_QUEUE_MAX = 32;
            if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
                IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest",
                         ax25_tx_queue_.size());
                ax25_tx_queue_.pop();
            }
            ax25_tx_queue_.push(std::move(rpt));
            // NO early return: like Direwolf, digipeating does not consume the
            // frame — the monitor path below (packet log, KISS promiscuous
            // clients) still sees it.  The UI sniffs below cannot false-match
            // a via-carrying frame (byte-14 proof in the CAL comment below).
        }
    } else if (!from_ofdm) {
        // Digipeat disabled: keep the guardrail loud instead of ignoring a
        // repeat request silently (digipeater design §8).
        Ax25Frame vf;
        if (ax25_parse(frame.data(), frame.size(), vf) && !vf.via.empty()) {
            for (const auto& hop : vf.via) {
                if (hop.repeated) continue;   // already used — not the next hop
                if (hop.addr.matches(config_.callsign)) {
                    IRIS_LOG("[DIGI] digipeater path present, not yet repeated: "
                             "%s>%s via [%s] names us as the next hop — "
                             "digipeating is DISABLED ([Digipeat] Enable=false), "
                             "frame not repeated",
                             vf.src.to_string().c_str(), vf.dst.to_string().c_str(),
                             ax25_via_to_string(vf.via).c_str());
                }
                break;   // only the first unrepeated hop counts (AX.25 repeat rule)
            }
        }
    }

    // Detect CAL: UI frames (ctrl=0x03, PID at offset 15, info at 16+)
    // Fixed no-via offset (ctrl at 14) is safe HERE: CAL/TUNE/PROBE frames are
    // Iris-generated and never carry a via path, and a via-carrying frame can
    // never FALSELY match — with a via present, byte 14 is the first hop's
    // shifted callsign char, and no valid callsign char (A-Z, 0-9, space)
    // shifts to 0x03/0x13, so the UI test below always fails for via frames.
    bool is_cal = false;
    bool is_probe = false;
    if (frame.size() > 20) {
        uint8_t ctrl = frame[14] & ~AX25_PF_MASK;
        Ax25Frame ui_frame;
        if (ctrl == AX25_CTRL_UI && frame.size() > 16 &&
            ax25_parse(frame.data(), frame.size(), ui_frame) &&
            ui_frame.type() == Ax25FrameType::U_FRAME &&
            ui_frame.u_type() == Ax25UType::UI) {
            const uint8_t* info = frame.data() + 16;
            size_t info_len = frame.size() - 16;
            const std::string& remote = ax25_session_.remote_callsign();
            bool expected_source = !remote.empty() && ui_frame.src.matches(remote);
            if (expected_source && ui_frame.dst.matches("CAL") &&
                info_len >= 4 && info[0] == 'C' && info[1] == 'A' &&
                info[2] == 'L' && info[3] == ':') {
                handle_cal_frame(info, info_len);
                is_cal = true;
            }
            // TUNE: UI frames (auto-tune gain calibration)
            if (!is_cal && expected_source && ui_frame.dst.matches("TUNE") &&
                info_len >= 5 && info[0] == 'T' && info[1] == 'U' &&
                info[2] == 'N' && info[3] == 'E' && info[4] == ':') {
                handle_tune_frame(info, info_len);
                is_cal = true;  // reuse flag to skip further processing
            }
            // PROBE:START UI frame — remote wants us to start probe responder.
            if (!is_cal && info_len >= 11 &&
                memcmp(info, "PROBE:START", 11) == 0) {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                if (ui_src != config_.callsign && !ofdm_kiss_probing_ &&
                    ofdm_kiss_probe_cd_ == 0) {
                    IRIS_LOG("[PROBE] Received PROBE:START from %s — starting responder",
                             ui_src.c_str());
                    if (gui_log_) gui_log_("Probe: " + ui_src + " requested probe");
                    probe_peer_call_ = ui_src;
                    // PROBE:START from a peer is always an auto-probe
                    // (apply PHY + enable native on completion).
                    // Manual probe is only triggered by the local Probe button.
                    probe_manual_ = false;
                    ofdm_kiss_probing_ = true;
                    ofdm_kiss_probe_done_ = false;
                    // Pre-limit max_info to the OFDM O0 floor (same as initiator);
                    // leg 3 single-sourced (== 75), grows as the anchor climbs.
                    ax25_session_.set_max_info(ofdm_max_info_for_level(0, ofdm_config_));
                    // Send PROBE:READY so initiator knows we're listening.
                    // New peers wait for this before sending tones (eliminates dead time).
                    // Old peers ignore it and send tones on their fixed countdown.
                    send_probe_ready_ui();
                    send_probe_ready_ui();  // Send twice for reliability
                    // Responder capture window: 8s. With the READY handshake,
                    // initiator sends tones ~2s after we send READY (AFSK delivery).
                    // Tones are 2.25s, so they arrive within 2-4.25s of capture start.
                    // 8s gives plenty of margin.
                    // Old-firmware initiators: tones arrive ~3-5s after their
                    // PROBE:START (fixed 3s countdown). 8s still catches them.
                    probe_.start_responder(config_.sample_rate, 8.0f);
                }
                is_probe = true;
            }
            // PROBE:READY UI frame — responder is listening, send tones now.
            if (!is_cal && info_len >= 11 &&
                memcmp(info, "PROBE:READY", 11) == 0) {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                if (ui_src != config_.callsign &&
                    (ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0)) {
                    IRIS_LOG("[PROBE] Received PROBE:READY from %s", ui_src.c_str());
                    if (ofdm_kiss_probe_cd_ > 0 && !ofdm_kiss_probing_) {
                        // READY arrived during countdown (before controller started).
                        // Start deferred initiator now, then immediately deliver READY.
                        ofdm_kiss_probe_cd_ = 0;
                        probe_.start_initiator_deferred(config_.sample_rate);
                        ofdm_kiss_probing_ = true;
                    }
                    // Forward to probe controller as PROBE_MSG_READY.
                    // The controller transitions from WAITING_READY to LISTENING_PROBE.
                    uint8_t ready_msg = PROBE_MSG_READY;
                    probe_.on_message(&ready_msg, 1);
                }
                is_probe = true;
            }
            // Probe RESULT in UI frame (probe data wrapped in AX.25 UI).
            // Also accept while the responder's RESULT re-announce is running
            // (probe_.reannounce_active()): after RSP finalizes, ofdm_kiss_probe_done_
            // is set, but CMD's corrected RESULT is exactly the confirm that stops
            // the re-announce — the previously-dead D3 heal path.
            if (!is_cal &&
                ((ofdm_kiss_probing_ && !ofdm_kiss_probe_done_) || probe_.reannounce_active()) &&
                info_len >= 1 && info[0] == PROBE_MSG_RESULT) {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                if (ui_src != config_.callsign) {
                    IRIS_LOG("[PROBE] Got probe result in UI frame from %s (%zu bytes)",
                             ui_src.c_str(), info_len);
                    probe_.on_message(info, info_len);
                    is_probe = true;
                }
            }

            // (Connection header removed — probe-first replaces header exchange)
        }
    }

    // Legacy probe detection (raw probe messages, non-OFDM-KISS)
    if (!is_probe && !is_cal && frame.size() >= 2) {
        bool ax25_address_field = frame.size() >= 15 &&
                                  (frame[6] & 0x61) == 0x60 &&
                                  (frame[13] & 0x60) == 0x60;
        for (int i = 0; ax25_address_field && i < 14; i++) {
            if (i == 6 || i == 13) continue;
            char c = (char)(frame[i] >> 1);
            ax25_address_field = (frame[i] & 0x01) == 0 &&
                                 (c == ' ' || (c >= 'A' && c <= 'Z') ||
                                  (c >= '0' && c <= '9'));
        }
        uint8_t first = frame[0];
        if (first == PROBE_MSG_RESULT && !ax25_address_field) {
            probe_.on_message(frame.data(), frame.size());
            is_probe = true;
        }
    }

    if (!is_probe && frame.size() >= 14) {
        std::string rx_src;
        for (int ci = 7; ci < 13; ci++) {
            char c = (char)(frame[ci] >> 1);
            if (c != ' ') rx_src += c;
        }
        if (rx_src != config_.callsign) {
            if (gui_log_)
                gui_log_("[RX] " + describe_ax25(frame.data(), frame.size()));

            // When AX.25 session is active (AGW mode), don't deliver raw frames
            // to rx_callback_ — the session's data callback delivers I-frame data.
            // BUT when KISS-managed, the KISS client (e.g. Winlink) runs its own
            // AX.25 state machine and needs ALL frames (UA, RR, I-frames, etc.).
            // Blocking them causes the KISS client to never see responses, leading
            // to infinite SABM retries and connection failure.
            auto ax_st = ax25_session_.state();
            bool session_active = (ax_st == Ax25SessionState::CONNECTED ||
                                   ax_st == Ax25SessionState::TIMER_RECOVERY ||
                                   ax_st == Ax25SessionState::AWAITING_CONNECTION ||
                                   ax_st == Ax25SessionState::AWAITING_RELEASE);
            // Terminate/re-pack: in terminate mode the far pump receives ONLY the
            // re-originated exact records (repack_rx_drain).  SUPPRESS the verbatim
            // raw-forward of every OFDM-tier frame — the R2 stream I-frames carry
            // length-prefixed bytes (not clean records) and the R2 reverse RRs are
            // Iris<->Iris (not the pump's).  Non-OFDM (AFSK/local) frames still fwd.
            bool suppress_ofdm_fwd = from_ofdm && native_repack_active();
            if ((!session_active || ax25_session_.is_kiss_managed()) && rx_callback_ &&
                !suppress_ofdm_fwd) {
                IRIS_LOG("[KISS-FWD] forwarding %zu byte frame to KISS (ofdm=%d, managed=%d)",
                         frame.size(), from_ofdm ? 1 : 0, ax25_session_.is_kiss_managed() ? 1 : 0);
                rx_callback_(frame.data(), frame.size());
            }
        }
    }
}

void Modem::process_rx_ax25(const float* audio, int count) {
    state_ = ModemState::RX_AX25;

    // Tick dedup cooldown
    if (dedup_cooldown_ > 0) {
        dedup_cooldown_ -= count;
        if (dedup_cooldown_ < 0) dedup_cooldown_ = 0;
    }

    // Feed raw audio to probe controller if it's listening
    // Legacy probe feed (ARQ native mode only — OFDM-KISS feeds from process_rx
    // with !ptt_active_ guard to avoid capturing our own TX).
    if (!ofdm_kiss_ && !ofdm_kiss_probing_ && probe_.state() == ProbeState::LISTENING_PROBE) {
        probe_.feed_rx(audio, count);
    }

    std::vector<uint8_t> rx_nrzi;
    if (config_.ax25_baud == 9600)
        rx_nrzi = gfsk_demod_.demodulate(audio, count);
    else
        rx_nrzi = afsk_demod_.demodulate(audio, count);

    auto rx_bits = nrzi_decoder_.decode(rx_nrzi);
    if (config_.ax25_baud == 9600)
        g3ruh_rx_scrambler_.descramble(rx_bits);

    for (uint8_t b : rx_bits) {
        // Feed to standard HDLC decoder (plain AX.25)
        if (hdlc_decoder_.push_bit(b)) {
            dispatch_rx_frame(hdlc_decoder_.frame());
            // Don't reset() here — push_bit already prepares for the next
            // frame (in_frame_=true). reset() would kill in_frame_, causing
            // the next back-to-back frame to be lost (middle frame in batch).
        }

        // Feed to FX.25 decoder in parallel (always active for backwards compat)
        if (fx25_decoder_.push_bit(b)) {
            dispatch_rx_frame(fx25_decoder_.frame(), true);
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_rx_native(const float* audio, int count) {
    // Don't override state when running as secondary demod (OFDM-KISS or native hail)
    if (!ofdm_kiss_ && native_mode_)
        state_ = ModemState::RX_NATIVE;

    // ============ OFDM PHY RX path ============
    // OFDM bypasses the downconverter — processes raw audio directly.
    // Skip downconvert + rx_overlap_buf when OFDM is active (saves CPU).
    // OFDM uses real passband audio directly (Hermitian-symmetric IFFT),
    // bypassing the downconverter. Buffer raw audio separately.
    if (ofdm_phy_active_ && ofdm_demod_) {
        // Skip OFDM RX during TX — our own signal bleeds back and creates
        // false SC/ZC triggers that waste CPU and corrupt Chase combining.
        if (ptt_active_ || rx_muted_ || native_selfhear_guard_ > 0) {
            return;
        }

        // OFDM bypasses the probe-based channel EQ entirely. The 127-tap FIR
        // has a 63-sample group delay that can exceed the OFDM CP (64 samples
        // default), risking ISI. OFDM handles per-carrier equalization
        // internally via training symbols and block pilots.
        ofdm_rx_audio_buf_.insert(ofdm_rx_audio_buf_.end(),
                                   audio, audio + count);
        // Apply LPF to remove f² discriminator noise from flat radio ports.
        // Runs on newly inserted samples only.
        {
            size_t start = ofdm_rx_audio_buf_.size() - count;
            if (ofdm_rx_lpf_active_) {
                for (size_t i = start; i < ofdm_rx_audio_buf_.size(); i++)
                    ofdm_rx_audio_buf_[i] = ofdm_rx_lpf_.process(ofdm_rx_audio_buf_[i]);
            }
            // Apply RX gain correction
            if (native_rx_gain_ != 1.0f) {
                for (size_t i = start; i < ofdm_rx_audio_buf_.size(); i++)
                    ofdm_rx_audio_buf_[i] *= native_rx_gain_;
            }
        }
        // Retention is bounded by the longest admitted 8-codeword geometry plus
        // acquisition overlap.  Duration and callback partitioning are irrelevant.
        int admitted_min_bits = std::min(ofdm_tone_map_.total_bits_per_symbol,
                                         ofdm_rx_tone_map_.total_bits_per_symbol);
        if (ofdm_rx_geometry_sample_bound_ == 0 ||
            ofdm_rx_geometry_bound_nfft_ != ofdm_config_.nfft ||
            ofdm_rx_geometry_bound_cp_ != ofdm_config_.cp_samples ||
            ofdm_rx_geometry_bound_data_carriers_ != ofdm_config_.n_data_carriers ||
            ofdm_rx_geometry_bound_pilot_rows_ != ofdm_config_.pilot_row_spacing ||
            ofdm_rx_geometry_bound_sample_rate_ != ofdm_config_.sample_rate ||
            ofdm_rx_geometry_bound_min_bits_ != admitted_min_bits) {
            const auto rx_bound = maximum_legal_ofdm_frame_samples(
                ofdm_config_, {ofdm_tone_map_, ofdm_rx_tone_map_});
            if (!rx_bound) {
                IRIS_LOG("[OFDM-RX] invalid admitted geometry set");
                ofdm_rx_audio_buf_.clear();
                ofdm_acquisition_.reset();
                ofdm_active_candidate_id_ = 0;
                ofdm_sync_cached_ = false;
                return;
            }
            ofdm_rx_geometry_sample_bound_ = *rx_bound;
            ofdm_rx_geometry_bound_nfft_ = ofdm_config_.nfft;
            ofdm_rx_geometry_bound_cp_ = ofdm_config_.cp_samples;
            ofdm_rx_geometry_bound_data_carriers_ = ofdm_config_.n_data_carriers;
            ofdm_rx_geometry_bound_pilot_rows_ = ofdm_config_.pilot_row_spacing;
            ofdm_rx_geometry_bound_sample_rate_ = ofdm_config_.sample_rate;
            ofdm_rx_geometry_bound_min_bits_ = admitted_min_bits;
        }
        // A callback may cross the bound while completing an already retained
        // candidate. Preserve that candidate until the geometry gate below gets
        // its one complete evaluation; the demodulator still bounds its scratch
        // to the candidate extent. With no unresolved candidate, keep the usual
        // acquisition window by trimming the searched prefix.
        if (ofdm_rx_audio_buf_.size() > ofdm_rx_geometry_sample_bound_) {
            size_t excess = ofdm_rx_audio_buf_.size() -
                            static_cast<size_t>(ofdm_rx_geometry_sample_bound_);
            const std::uint64_t origin = ofdm_acquisition_.buffer_origin();
            const std::uint64_t overlap = 3U * static_cast<std::uint64_t>(
                ofdm_config_.symbol_samples());
            const std::uint64_t safe =
                ofdm_acquisition_.safe_retirement_watermark(overlap);
            const std::uint64_t safely_retirable = safe > origin
                ? safe - origin : 0;
            if (excess <= safely_retirable) {
                const std::uint64_t retired_end =
                    sample_position_after(origin, excess);
                ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                          ofdm_rx_audio_buf_.begin() + excess);
                ofdm_acquisition_.retire_prefix(retired_end);
                ofdm_sync_cached_ = false;
                IRIS_LOG("[OFDM-RX] geometry sample bound: retired %zu searched samples",
                         excess);
            } else {
                // Capacity loss below the safe watermark invalidates the capture
                // epoch.  Keep the newest bounded suffix, but never claim the
                // discarded prefix was searched or let custody survive the gap.
                const std::uint64_t new_origin =
                    sample_position_after(origin, excess);
                ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                          ofdm_rx_audio_buf_.begin() + excess);
                ofdm_acquisition_.reset(new_origin);
                ofdm_active_candidate_id_ = 0;
                ofdm_sync_cached_ = false;
                ofdm_pending_required_samples_ = 0;
                reset_ofdm_belief_and_chase();
                IRIS_LOG("[OFDM-RX] CAPTURE DISCONTINUITY: capacity exceeded "
                         "safe watermark by %llu samples (discarded=%zu)",
                         static_cast<unsigned long long>(
                             excess - safely_retirable), excess);
                if (repack_has_open_custody())
                    repack_custody_teardown(
                        "OFDM capture capacity discontinuity",
                        v2::TransferResultReason::AudioDiscontinuity);
                return;
            }
        }

        size_t n_samples = ofdm_rx_audio_buf_.size();
        // Need enough samples for a minimum OFDM frame:
        // 2 training + 1 sync word + 1 data + 1 tail = 5 symbols.
        size_t sym_len = (size_t)(ofdm_config_.cp_samples + ofdm_config_.nfft);
        size_t min_samples = sym_len * 5;  // 2 preamble + 1 sync + 1 data + 1 tail
        if (n_samples < min_samples) return;

        // MFSK-first detection: check for MFSK tone ACK BEFORE running the
        // OFDM pipeline. Both sides use MFSK for S-frames (RR/REJ/poll).
        // OFDM sync (SC metric) false-triggers on MFSK tones, consuming the
        // buffer and blocking MFSK detection. Running MFSK first avoids this.
        //
        // Gate: ofdm_expect_ack_ (CMD waiting for RR after data TX).
        // RSP doesn't need MFSK detection — RSP receives OFDM data and sends
        // MFSK RR autonomously via T2. CMD's T1 polls are OFDM-detectable
        // (RSP's enquiry_response handles them via normal S-frame path).
        // Only CMD needs to detect RSP's MFSK ACK response.
        if (ofdm_expect_ack_ && mfsk_ack_.is_initialized() &&
            (int)ofdm_rx_audio_buf_.size() >= mfsk_ack_.min_detect_samples()) {
            auto mfsk = mfsk_ack_.detect(ofdm_rx_audio_buf_.data(),
                                          (int)ofdm_rx_audio_buf_.size());
            if (mfsk.detected && mfsk.n_r >= 0) {
                // A3: capture the pre-ACK window base BEFORE inject_rx_rr advances
                // V(A), so the adopt below can credit the anchor to the MAX level
                // among the frames this MFSK N(R) actually ACKs.
                uint8_t mfsk_prev_va = ax25_session_.va();
                IRIS_LOG("[MAC-DIAG] MFSK ACK received: N(R)=%d PF=%d matched=%d/%d epoch=%d %s",
                         mfsk.n_r, mfsk.pf, mfsk.matched, MfskAck::ACK_LEN, mfsk.epoch,
                         ofdm_expect_ack_ ? "(expected)" : "(unsolicited)");
                // #2 BURST-EPOCH GUARD (the dominant reverse-ACK carrier: RSP MFSK ->
                // CMD).  Bind THIS tone to the forward burst CMD is waiting on.  A
                // stale/buffered tone (the evidence's 4x identical re-detections)
                // echoes an OLD epoch -> ADVISORY: cancel csma so CMD retransmits,
                // but do NOT advance V(A) (session or pump), do NOT adopt a stale
                // rate, and leave T1 running so the missing frame is resent (the
                // cheap miss direction).  A MATCH or an ABSENT echo (mfsk.epoch < 0)
                // is fail-open toward the #1 live-N(R) fix.
                bool mfsk_epoch_ok = (mfsk.epoch < 0) ||
                                     ((mfsk.epoch & 0x07) == (int)(tx_burst_epoch_ & 0x07));
                if (!mfsk_epoch_ok) {
                    IRIS_LOG("[REV-ACK] MFSK epoch MISMATCH: echo=%d expect=%d -> "
                             "advisory (hold V(A), T1 retransmits)",
                             mfsk.epoch, (int)(tx_burst_epoch_ & 0x07));
                    if (ax25_session_.we_initiated()) csma_holdoff_ = 0;
                    ofdm_expect_ack_ = false;
                    int consume = std::min(mfsk.offset + MfskAck::TOTAL_SAMPLES,
                                            (int)ofdm_rx_audio_buf_.size());
                    const std::uint64_t retired_end =
                        sample_position_after(
                            ofdm_acquisition_.buffer_origin(), consume);
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + consume);
                    ofdm_acquisition_.retire_prefix(retired_end);
                    ofdm_sync_cached_ = false;
                    return;
                }
                // Build a properly addressed RR frame from MFSK payload.
                // This frame goes through the full dispatch path:
                // - AX.25 session: V(A) advance, T1/T3, timer recovery exit
                // - KISS forwarding: sender pump sees the ACK, advances its V(A)
                // - MAC: csma cancel, batch airtime, gearshift feedback
                ax25_session_.inject_rx_rr((uint8_t)mfsk.n_r, mfsk.pf != 0);
                // Anchor futility resets ONLY on a real V(A) advance — an MFSK
                // poll response with an unchanged N(R) is "alive but not
                // decoding our data" (same policy as the OFDM RR sniff) and
                // must not mask the futility evidence.
                if (ax25_session_.va() != mfsk_prev_va)
                    ofdm_anchor_futility_ = 0;
                // Forward to KISS client so the pump's V(A) advances too.
                // Build the same frame inject_rx_rr used (dst=us, src=peer).
                {
                    auto rr_frame = ax25_session_.build_rx_rr((uint8_t)mfsk.n_r, mfsk.pf != 0);
                    if (!rr_frame.empty() && rx_callback_)
                        rx_callback_(rr_frame.data(), rr_frame.size());
                }
                // MAC-level: cancel csma holdoff so CMD can TX immediately.
                if (ax25_session_.we_initiated()) {
                    csma_holdoff_ = 0;
                    if (ax25_session_.va() != mfsk_prev_va)
                        tx_no_ack_count_ = 0;
                }
                // Receiver-drives-rate ADOPT: B's absolute forward-level proposal
                // (validated by B's decode-margin climb) is both a positive decode
                // confirmation AND a forward-rate command. The field is self-
                // rejecting in mfsk_ack (2/3 vote + range check); -1 = ABSENT, in
                // which case we HOLD (conservative under-skip, idempotent — the
                // next ACK re-asserts the absolute target with no drift).
                if (mfsk.proposed_level >= 0) {
                    int L_prop = mfsk.proposed_level;
                    // (1) Advance the anchor to what B endorses of what A has
                    //     ACTUALLY sent — never anchor above tx_last_level_ (the
                    //     level A really transmitted). This is what finally lets
                    //     the anchor rise on a unidirectional link.
                    // A3: bound "what A actually sent" by the ring (max level among
                    // the frames B just ACKed) rather than the last-batch scalar,
                    // satisfying the "never anchor above what A really transmitted"
                    // invariant precisely. Falls back to tx_last_level_ when this
                    // ACK advances no new N(S) (poll response) — identical to prior.
                    int ring_lvl = ring_acked_and_clear(mfsk_prev_va, (uint8_t)mfsk.n_r);
                    int sent_ceil = (ring_lvl >= 0) ? ring_lvl : tx_last_level_;
                    int new_anchor = std::max(tx_acked_level_,
                                              std::min(L_prop, sent_ceil));
                    if (new_anchor != tx_acked_level_) {
                        IRIS_LOG("MAC: tx_acked_level O%d -> O%d (MFSK L_prop=O%d, sent=O%d)",
                                 tx_acked_level_, new_anchor, L_prop, sent_ceil);
                        tx_acked_level_ = new_anchor;   // new_anchor >= old (a GROW)
                        // leg 3: grow MAX_INFO with the receiver-endorsed anchor.
                        ax25_session_.set_max_info(ofdm_max_info_for_level(tx_acked_level_, ofdm_config_));
                    }
                    // (2)+(3) Raise the climb target toward L_prop with a BOUNDED
                    //     leap (receiver-driven, so +LEAP_MAX above the confirmed
                    //     anchor is safe; worst-case wrong leap = LEAP_MAX rungs =
                    //     one batch, recovered by no-ACK/REJ downshift). Keep the
                    //     gearshift copy in sync so a later reverse burst doesn't
                    //     fight the adopted level.
                    int leap_target = std::min(L_prop,
                                               tx_acked_level_ + OFDM_LEVEL_LEAP_MAX);
                    gearshift_.force_ofdm_level(leap_target);
                    // (4) Latch the fresh proposal so the TX ceiling (modem.cc
                    //     TX path) transiently widens to admit this leap. It is
                    //     one-shot: consumed by the next data TX, then the ceiling
                    //     decays back to acked+1 (default conservative behavior).
                    tx_proposed_level_ = L_prop;
                    IRIS_LOG("MAC: receiver-driven adopt L_prop=O%d -> gearshift O%d (anchor O%d, +leap%d)",
                             L_prop, leap_target, tx_acked_level_, OFDM_LEVEL_LEAP_MAX);
                }
                ofdm_expect_ack_ = false;
                // Consume detected samples
                int consume = std::min(mfsk.offset + MfskAck::TOTAL_SAMPLES,
                                        (int)ofdm_rx_audio_buf_.size());
                const std::uint64_t retired_end =
                    sample_position_after(
                        ofdm_acquisition_.buffer_origin(), consume);
                ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                          ofdm_rx_audio_buf_.begin() + consume);
                ofdm_acquisition_.retire_prefix(retired_end);
                ofdm_sync_cached_ = false;
                return;
            }
        }

        // A retained candidate already has an exact input-relative geometry
        // requirement. While it is still incomplete, do no growing-buffer
        // analytic FFT at all. Once ready, convert only that candidate extent;
        // any following frame remains raw and untouched in the receive buffer.
        const bool reconsider_cached_timing =
            ofdm_sync_cached_ &&
            ofdm_pending_sync_.status == OfdmSyncStatus::NeedContext;
        size_t analytic_extent = n_samples;
        if (ofdm_sync_cached_ && ofdm_pending_required_samples_ > 0) {
            if (static_cast<std::uint64_t>(n_samples) <
                    ofdm_pending_required_samples_) {
                ofdm_redetect_count_++; // diagnostics only
                return;
            }
            analytic_extent = static_cast<size_t>(ofdm_pending_required_samples_);
        }

        // Convert real audio to analytic signal via Hilbert transform.
        // This enables Schmidl-Cox to extract CFO phase (real-only signals
        // produce real-valued correlation, losing the sign of freq offset).
        // Band-limited to the negotiated OFDM band, fused into the Hilbert
        // positive-frequency selection.  ROOT (confirmed by bisection):
        // on the LIVE continuous real-audio stream the captured passband
        // carries OUT-OF-BAND energy -- sub-band low-freq content below the
        // first carrier and a spectral skirt above the last -- that a clean
        // isolated tx-ofdm frame does not.  That energy inflated the guard-bin
        // noise estimate (a ~20 dB SNR UNDERREAD) and corrupted the channel
        // estimate -> confident-WRONG LLRs, 0 delivered.  The band-limit kills
        // it with zero group delay (no filter memory, so no ISI).  The EDGE of
        // the band-limit is the TAPERED design (raised-cosine transition +
        // reflected guard extensions, see ofdm_analytic_bandlimit): the
        // original zero-margin brick-wall edge rang across the capture buffer
        // and floored the training-pair noise estimate at ~42 dB, capping the
        // SNR meter's top end and falsely refusing the 256QAM gears
        // (data-flow-noise-var.md §8.4).  Offline --rx-ofdm is unchanged (it
        // conditions its own isolated frames); this is the live-RX-only path.
        {
            static const bool RX_BANDLIMIT = [] {
                const char* e = std::getenv("IRIS_RX_BANDLIMIT");
                return !e || std::atoi(e) != 0;   // default ON
            }();
            // IRIS_RX_BL_EDGE=0 restores the legacy zero-margin brick-wall
            // edge (A/B diagnostic only -- reinstates the ~42 dB meter floor).
            static const bool RX_BL_TAPERED = [] {
                const char* e = std::getenv("IRIS_RX_BL_EDGE");
                return !e || std::atoi(e) != 0;   // default tapered
            }();
            const RxBandlimitEdge bl_edge =
                !RX_BANDLIMIT ? RxBandlimitEdge::OFF
                : (RX_BL_TAPERED ? RxBandlimitEdge::TAPERED
                                 : RxBandlimitEdge::BRICKWALL);
            ofdm_rx_iq_ = ofdm_analytic_bandlimit(
                ofdm_rx_audio_buf_.data(), (int)analytic_extent, ofdm_config_, bl_edge);
        }
        n_samples = ofdm_rx_iq_.size();

        // Use cached sync if available (avoids re-detecting same preamble
        // while waiting for data symbols to accumulate in the buffer).
        OfdmSyncResult sync;
        if (ofdm_sync_cached_ && !reconsider_cached_timing) {
            sync = ofdm_pending_sync_;
        } else {
            // A provisional boundary timing is deliberately cached only as
            // receiver-owned work.  Once its requested context arrives, run
            // discovery again; never reuse the provisional frame_start.
            if (reconsider_cached_timing)
                ofdm_sync_cached_ = false;
            // The normal 0.70 FD-ZC threshold is a loose, normalized
            // pre-decoder admission gate. During TUNE, extreme ramp levels
            // degrade coherence further, so measurement acquisition uses 0.30.
            // Complete payload CRC validation remains authoritative for data.
            {
                bool in_tune = (tune_state_ != TuneState::IDLE &&
                                tune_state_ != TuneState::DONE);
                // Normal reception uses loose, normalized admission.  Payload
                // validity is established only by LDPC and complete CRC checks.
                ofdm_config_.fd_zc_threshold = in_tune ? 0.30f : 0.70f;
            }
            sync = ofdm_detect_frame(ofdm_rx_iq_.data(), (int)n_samples,
                                     ofdm_config_, &ofdm_sync_workspace_);
            if (sync.search_examined_through >= 0) {
                ofdm_acquisition_.note_searched_through(
                    sample_position_after(
                        ofdm_acquisition_.buffer_origin(),
                        static_cast<std::uint64_t>(
                            sync.search_examined_through)));
            }
            if (sync.status == OfdmSyncStatus::NeedContext) {
                auto* candidate = ofdm_acquisition_.remember(sync);
                if (!candidate)
                    return;
                ofdm_active_candidate_id_ = candidate->id;
                ofdm_pending_sync_ = sync;
                ofdm_sync_cached_ = true;
                ofdm_pending_required_samples_ = static_cast<std::uint64_t>(
                    sync.required_right_context);
                ofdm_acquisition_.require_context(
                    candidate->id,
                    sample_position_after(ofdm_acquisition_.buffer_origin(),
                                          ofdm_pending_required_samples_));
                ofdm_redetect_count_++;
                return;
            }
            if (!sync.detected) {
                if (reconsider_cached_timing && ofdm_active_candidate_id_ != 0) {
                    ofdm_acquisition_.mark_rejected(ofdm_active_candidate_id_);
                    ofdm_active_candidate_id_ = 0;
                }
                // A chronological SC region that failed normalized FD
                // admission is rejected work, not "no candidate". Retire only
                // its examined timing interval so a following preamble remains
                // available to the next scheduler turn.
                if (sync.candidate_present) {
                    auto* candidate = ofdm_acquisition_.remember(sync);
                    if (!candidate)
                        return; // bounded queue: retain input and yield
                    ofdm_acquisition_.mark_rejected(candidate->id);
                    size_t skip = sync.timing_interval_begin >= 0
                        ? static_cast<size_t>(sync.timing_interval_begin + 1)
                        : static_cast<size_t>(sync.frame_start +
                                              1);
                    skip = std::max<std::size_t>(skip, 1);
                    skip = std::min(skip, ofdm_rx_audio_buf_.size());
                    const std::uint64_t origin =
                        ofdm_acquisition_.buffer_origin();
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + skip);
                    ofdm_acquisition_.retire_prefix(
                        sample_position_after(origin, skip));
                    ofdm_sync_cached_ = false;
                    return;
                }
                // If no detection and buffer is getting large, trim old searched samples.
                // Keep last 4 symbols of margin in case a preamble straddles the trim boundary.
                // When expecting MFSK ACK, keep enough for MFSK detection (12288 samples).
                size_t min_keep = (ofdm_expect_ack_ && mfsk_ack_.is_initialized())
                    ? (size_t)mfsk_ack_.min_detect_samples()
                    : (size_t)(sym_len * 4);
                if (ofdm_rx_audio_buf_.size() > std::max((size_t)(sym_len * 8), min_keep + (size_t)sym_len)) {
                    size_t keep = std::max((size_t)(sym_len * 4), min_keep);
                    size_t trim = ofdm_rx_audio_buf_.size() - keep;
                    const std::uint64_t safe =
                        ofdm_acquisition_.safe_retirement_watermark(3 * sym_len);
                    const std::uint64_t origin = ofdm_acquisition_.buffer_origin();
                    trim = std::min<std::uint64_t>(trim,
                        safe > origin ? safe - origin : 0);
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + trim);
                    ofdm_acquisition_.retire_prefix(
                        sample_position_after(origin, trim));
                    ofdm_sync_cached_ = false;
                }
                return;
            }
            auto* acquisition_candidate = ofdm_acquisition_.remember(sync);
            if (!acquisition_candidate) {
                // Finite work envelope is full: retain every sample and yield.
                return;
            }
            ofdm_active_candidate_id_ = acquisition_candidate->id;
            ofdm_redetect_count_ = 0;

            // The MFSK-first path above is authoritative only after a validated
            // tone detection.  An OFDM candidate admitted at 0.70 always reaches
            // its LDPC/CRC trials, including while the initiator expects an ACK.
        }

        // Self-hear guard: radio plays back TX tones on its audio output.
        // Must clear the ENTIRE buffer — a partial skip (just the preamble)
        // leaves data symbols + subsequent frame preambles intact, which get
        // decoded after the 300ms guard timer expires.
        if (native_selfhear_guard_ > 0) {
            IRIS_LOG("[OFDM-RX] frame detected but DISCARDED (self-hear guard, clearing %zu samples)",
                     ofdm_rx_audio_buf_.size());
            ofdm_rx_audio_buf_.clear();
            ofdm_acquisition_.reset();
            ofdm_active_candidate_id_ = 0;
            ofdm_rx_lpf_.reset();
            ofdm_sync_cached_ = false;
            return;
        }

        // Detection owns only the modulo-Fs/L estimate.  Select exactly one
        // total-CFO hypothesis for this scheduler turn; the candidate persists
        // the cursor so a callback budget yield or a failed shape cannot lose
        // the remaining wraps.  Timing is jointly refined from the full retained
        // capture before any candidate-relative slice is formed.
        OfdmSyncResult principal_sync = sync;
        std::size_t active_cfo_cursor = 0;
        if (auto* candidate =
                ofdm_acquisition_.find(ofdm_active_candidate_id_)) {
            principal_sync = candidate->sync;
            active_cfo_cursor = candidate->cfo_trial_cursor;
        }
        const std::uint64_t acquisition_epoch =
            ofdm_acquisition_.capture_epoch();
        if (ofdm_search_owner_candidate_id_ != ofdm_active_candidate_id_ ||
            ofdm_search_owner_capture_epoch_ != acquisition_epoch) {
            ofdm_search_owner_result_ = {};
            ofdm_search_owner_candidate_id_ = ofdm_active_candidate_id_;
            ofdm_search_owner_capture_epoch_ = acquisition_epoch;
            ofdm_search_owner_chase_attempted_ = false;
            ofdm_search_owner_valid_ = false;
            ofdm_bank_next_shape_samples_ = 0;
        }
        auto cfo_hypotheses = ofdm_cfo_hypotheses(
            ofdm_rx_iq_.data(), static_cast<int>(n_samples), ofdm_config_,
            principal_sync);
        if (active_cfo_cursor >= cfo_hypotheses.size()) {
            ofdm_acquisition_.mark_rejected(ofdm_active_candidate_id_);
            ofdm_sync_cached_ = false;
            return;
        }
        // Select the evidence owner in stable Hz coordinates before shape work
        // so a useful result can survive a bounded-work yield.  The bank index
        // is candidate-relative and may flip at an alias boundary; committed Hz
        // does not. Equal-distance aliases prefer the smaller absolute wrap.
        std::size_t owner_cfo_cursor = 0;
        float owner_distance = std::numeric_limits<float>::infinity();
        int owner_abs_wrap = std::numeric_limits<int>::max();
        for (std::size_t i = 0; i < cfo_hypotheses.size(); ++i) {
            const float distance = std::fabs(
                cfo_hypotheses[i].cfo_hz - ofdm_cfo_committed_hz_);
            const int abs_wrap =
                std::abs(cfo_hypotheses[i].cfo_ambiguity_index);
            if (distance < owner_distance ||
                (distance == owner_distance && abs_wrap < owner_abs_wrap)) {
                owner_cfo_cursor = i;
                owner_distance = distance;
                owner_abs_wrap = abs_wrap;
            }
        }
        const bool evidence_owner = active_cfo_cursor == owner_cfo_cursor;
        OfdmSyncResult trial_sync = ofdm_refine_cfo_hypothesis(
            ofdm_rx_iq_.data(), static_cast<int>(n_samples), ofdm_config_,
            cfo_hypotheses[active_cfo_cursor]);
        IRIS_LOG("[OFDM-RX] CFO hypothesis %zu/%zu: m=%d total=%.3f Hz timing=%d ref=%.3f",
                 active_cfo_cursor + 1, cfo_hypotheses.size(),
                 trial_sync.cfo_ambiguity_index, trial_sync.cfo_hz,
                 trial_sync.frame_start, trial_sync.cfo_training_metric);

        // Exact shape trials below own their own extent checks.  A configured
        // or expanded full-frame requirement must not prevent this wrap from
        // trying a complete shorter shape already present in the buffer.
        bool short_ack_gate_used = false;

        ofdm_acquisition_.mark_ready(ofdm_active_candidate_id_);

#ifndef NDEBUG
        // Fingerprint actual persistent link content, not cooperative write
        // counters.  Acquisition cursors and retained owner search results are
        // intentionally excluded: they are the only state phase C may change.
        auto persistent_link_fingerprint = [&]() {
            std::uint64_t hash = 1469598103934665603ULL;
            auto bytes = [&](const void* data, std::size_t size) {
                const auto* p = static_cast<const unsigned char*>(data);
                for (std::size_t i = 0; i < size; ++i) {
                    hash ^= static_cast<std::uint64_t>(p[i]);
                    hash *= 1099511628211ULL;
                }
            };
            auto scalar = [&](const auto& value) {
                bytes(&value, sizeof(value));
            };
            auto vector = [&](const auto& values) {
                const std::size_t size = values.size();
                scalar(size);
                if (!values.empty())
                    bytes(values.data(), values.size() * sizeof(values[0]));
            };
            auto channel = [&](const OfdmChannelEst& est) {
                vector(est.H);
                vector(est.snr_per_carrier);
                vector(est.noise_var);
                scalar(est.mean_snr_db);
                scalar(est.noise_var_frame);
            };

            if (ofdm_demod_)
                channel(ofdm_demod_->last_channel_estimate());

            vector(ofdm_chase_llrs_);
            scalar(ofdm_chase_combines_);
            scalar(ofdm_chase_candidate_id_);
            scalar(ofdm_chase_capture_epoch_);
            scalar(ofdm_chase_level_);
            scalar(ofdm_chase_n_codewords_);
            scalar(ofdm_chase_fec_rate_);
            scalar(ofdm_cfo_committed_hz_);

            scalar(ofdm_harq_evidence_.any_failed);
            scalar(ofdm_harq_evidence_.all_failed);
            scalar(ofdm_harq_evidence_.num_blocks);
            for (const auto& block : ofdm_harq_evidence_.blocks) {
                scalar(block.converged);
                scalar(block.iterations);
                vector(block.data_bits);
            }
            vector(ofdm_harq_evidence_.stored_llrs);
            vector(ofdm_harq_evidence_.sym_phase_var);
            scalar(ofdm_harq_evidence_.mod);
            scalar(ofdm_harq_evidence_.fec);
            scalar(ofdm_harq_evidence_.payload_len);
            scalar(ofdm_harq_evidence_.harq_flag);
            scalar(ofdm_harq_evidence_.retx_desc.original_seq);
            for (const auto& region :
                    ofdm_harq_evidence_.retx_desc.regions) {
                scalar(region.block_index);
                scalar(region.bit_start);
                scalar(region.bit_count);
            }
            vector(ofdm_harq_evidence_.retx_desc.retx_bits);
            vector(ofdm_harq_evidence_.new_data);
            vector(ofdm_harq_evidence_.payload);
            scalar(ofdm_harq_evidence_candidate_id_);
            scalar(ofdm_harq_evidence_capture_epoch_);
            scalar(ofdm_harq_evidence_valid_);

            scalar(snr_db_);
            scalar(snr_preamble_db_);
            bytes(&gearshift_, sizeof(gearshift_));

            scalar(tune_state_);
            scalar(tune_my_gain_);
            scalar(tune_frames_measured_);
            scalar(tune_peer_gain_);
            for (int i = 0; i < TUNE_RAMP_COUNT; ++i) {
                scalar(tune_rx_frame_iters_[i]);
                scalar(tune_rx_frame_H_[i]);
                scalar(tune_rx_frame_snr_[i]);
                scalar(tune_peer_iters_[i]);
                scalar(tune_peer_H_[i]);
                scalar(tune_peer_snr_[i]);
                scalar(tune_computed_scales_[i]);
            }

            // The ARQ object representation contains its notification/HARQ slot
            // metadata.  Holder payload bytes are hashed explicitly above.
            bytes(&arq_, sizeof(arq_));
            return hash;
        };
        const std::uint64_t search_content_start =
            persistent_link_fingerprint();
        auto assert_search_pure = [&]() {
            assert(persistent_link_fingerprint() == search_content_start);
        };
#endif

        // Demodulate the configured shape.  An incomplete result is cheap and
        // contributes only that shape's exact missing extent; shorter complete
        // shapes are still tried before the scheduler waits.
        // OFDM-KISS: use RX tone map (tracks peer's speed level via signaling).
        // Non-KISS: TX and RX always at same level, use ofdm_tone_map_.
        const ToneMap& decode_map = ofdm_kiss_tx_ ? ofdm_rx_tone_map_ : ofdm_tone_map_;
        int result_level = decode_map.tone_map_id > 0
            ? static_cast<int>(decode_map.tone_map_id) - 1
            : (ofdm_kiss_tx_ ? ofdm_kiss_rx_level_ : ofdm_speed_level_);
        int result_n_codewords = decode_map.n_codewords;
        bool blind_shape_success = false;
        OfdmDemodResult result = ofdm_demod_->demodulate_trial(
            ofdm_rx_iq_.data(), static_cast<int>(n_samples),
            decode_map, trial_sync);
#ifndef NDEBUG
        assert_search_pure();
#endif
        if (result.demodulated_level >= 0)
            result_level = result.demodulated_level;
        result_n_codewords = result.demodulated_n_codewords;

        auto owner_usable_failure = [](const OfdmDemodResult& trial) {
            return !trial.success && trial.complete_boundary_validated &&
                   !trial.llrs.empty();
        };
        auto select_blind_failure_evidence = [&](OfdmDemodResult&& trial,
                                                 int level) {
            if (owner_usable_failure(result) ||
                !owner_usable_failure(trial))
                return false;
            result = std::move(trial);
            result_level = result.demodulated_level >= 0
                ? result.demodulated_level : level;
            result_n_codewords = result.demodulated_n_codewords;
            blind_shape_success = true;
            return true;
        };

        // Blind-shape work is finite but can still be substantial.  Resume its
        // deterministic flat cursor across callbacks rather than monopolizing
        // an audio callback or restarting at shape zero after every yield.
        constexpr std::size_t ACQUISITION_SHAPE_WORK_BUDGET = 24;
        std::size_t resume_untried_shape_cursor = 0;
        std::vector<std::size_t> due_incomplete_shape_cursors;
        if (auto* candidate = ofdm_acquisition_.find(ofdm_active_candidate_id_)) {
            resume_untried_shape_cursor =
                candidate->next_untried_shape_cursor;
            const std::uint64_t available_absolute = sample_position_after(
                ofdm_acquisition_.buffer_origin(), n_samples);
            for (const auto& work : candidate->incomplete_shape_work) {
                if (work.required_right_context <= available_absolute)
                    due_incomplete_shape_cursors.push_back(work.cursor);
            }
        }
        auto shape_trial_due = [&](std::size_t cursor) {
            return cursor >= resume_untried_shape_cursor ||
                std::find(due_incomplete_shape_cursors.begin(),
                          due_incomplete_shape_cursors.end(), cursor) !=
                    due_incomplete_shape_cursors.end();
        };
        std::size_t shape_work_this_callback = 0;
        std::size_t flat_shape_cursor = 0;
        bool shape_budget_yield = false;

        bool longer_shape_incomplete =
            result.completion == OfdmDemodResult::Completion::NeedMoreSamples;
        const std::uint64_t available_shape_samples =
            static_cast<std::uint64_t>(n_samples);
        std::uint64_t required_for_longer_shape = 0;
        if (longer_shape_incomplete &&
            result.additional_samples_required <=
                std::numeric_limits<std::uint64_t>::max() -
                    available_shape_samples) {
            required_for_longer_shape = available_shape_samples +
                result.additional_samples_required;
        }
        auto record_shape_result = [&](const OfdmDemodResult& trial,
                                       std::size_t cursor) {
            if (trial.completion !=
                    OfdmDemodResult::Completion::NeedMoreSamples) {
                ofdm_acquisition_.resolve_shape_trial(
                    ofdm_active_candidate_id_, cursor);
                return;
            }
            longer_shape_incomplete = true;
            const std::uint64_t available = static_cast<std::uint64_t>(n_samples);
            if (trial.additional_samples_required <=
                    std::numeric_limits<std::uint64_t>::max() - available) {
                const std::uint64_t required =
                    available + trial.additional_samples_required;
                required_for_longer_shape = std::max(
                    required_for_longer_shape, required);
                ofdm_acquisition_.remember_incomplete_shape(
                    ofdm_active_candidate_id_, cursor,
                    sample_position_after(ofdm_acquisition_.buffer_origin(),
                                          required));
            }
        };

        // ---- OFDM-KISS blind speed detection ----
        // If primary decode fails and we're in KISS mode, the peer may have
        // shifted to a different speed level OR sent a 1-CW S-frame (ACK).
        // Try candidate levels and CW counts. Cost: ~5ms per decode attempt.
        if (!result.success && ofdm_kiss_tx_ &&
            (!result.llrs.empty() || result.completion ==
                OfdmDemodResult::Completion::NeedMoreSamples)) {
            // First: try 1-CW at the current level (S-frame from peer).
            // S-frames (RR/REJ/RNR) use 1 CW regardless of speed level to
            // minimize ACK airtime. This is the most common blind-detect case.
            if (ofdm_cw_for_level(ofdm_kiss_rx_level_) > 1) {
                const std::size_t cursor = flat_shape_cursor++;
                if (shape_trial_due(cursor)) {
                    ToneMap ack_map = ofdm_rx_tone_map_for_level(
                        ofdm_kiss_rx_level_, ofdm_config_, config_.ofdm_nuc);
                    ack_map.n_codewords = 1;
                    OfdmDemodResult ack_result = ofdm_demod_->demodulate_trial(
                        ofdm_rx_iq_.data(), static_cast<int>(n_samples),
                        ack_map, trial_sync);
#ifndef NDEBUG
                    assert_search_pure();
#endif
                    if (ack_result.completion !=
                            OfdmDemodResult::Completion::NeedMoreSamples)
                        ++shape_work_this_callback;
                    ofdm_acquisition_.set_trial_cursors(
                        ofdm_active_candidate_id_, active_cfo_cursor,
                        flat_shape_cursor);
                    record_shape_result(ack_result, cursor);
                    if (ack_result.success && ack_result.completion ==
                            OfdmDemodResult::Completion::CompleteValidatedFrame) {
                        IRIS_LOG("[OFDM-RX] 1-CW S-frame at O%d (was expecting %d CW)",
                                 ofdm_kiss_rx_level_, ofdm_cw_for_level(ofdm_kiss_rx_level_));
                        result = std::move(ack_result);
                        result_level = result.demodulated_level >= 0
                            ? result.demodulated_level : ofdm_kiss_rx_level_;
                        result_n_codewords = result.demodulated_n_codewords;
                        blind_shape_success = true;
                    } else {
                        select_blind_failure_evidence(
                            std::move(ack_result), ofdm_kiss_rx_level_);
                    }
                }
            }

            // Second: sweep other speed levels (peer may have shifted).
            if (!result.success) {
                int candidates[NUM_OFDM_SPEED_LEVELS];
                int n_candidates = 0;
                // Cap trial levels at config's max_modulation to avoid trying
                // modulations our session didn't negotiate (e.g. 1024QAM at
                // O10-O12 when max_modulation=4/256QAM). Trying unsupported
                // modulations on a mismatched signal has been observed to
                // trigger a stack overflow in the 1024QAM decode path.
                //   Density compare (see the gs_max site): the QAM32=6 enum ordering
                //   would otherwise cap the RX sweep at O5 and stop it demapping the
                //   O6/O7 frames the sender now climbs to. bits_per_symbol keeps
                //   QAM1024 excluded (10>8) so the stack-overflow guard is preserved.
                static const bool density_modcap_rx = []() {
                    const char* e = std::getenv("IRIS_DENSITY_MODCAP");
                    return !e || std::atoi(e) != 0; }();
                int max_mod_level = NUM_OFDM_SPEED_LEVELS - 1;
                for (int lvl = 0; lvl < NUM_OFDM_SPEED_LEVELS; lvl++) {
                    bool over = density_modcap_rx
                        ? (bits_per_symbol(OFDM_SPEED_LEVELS[lvl].modulation)
                             > bits_per_symbol(config_.max_modulation))
                        : ((int)OFDM_SPEED_LEVELS[lvl].modulation
                             > (int)config_.max_modulation);
                    if (over) { max_mod_level = lvl - 1; break; }
                }
                auto add_candidate = [&](int lvl) {
                    if (lvl < 0 || lvl > max_mod_level) return;
                    for (int j = 0; j < n_candidates; j++)
                        if (candidates[j] == lvl) return;  // no duplicates
                    candidates[n_candidates++] = lvl;
                };
                // Coherent RX adoption (Mercury rank 7/9): try the level we are
                // driving A toward FIRST. On the receiver-driven climb this is the
                // level A has just adopted, so the O3/ncw4 frame is demapped at the
                // right (modulation, ncw) on the first trial rather than after a
                // full sweep. CRC-32 gating keeps it self-rejecting.
                add_candidate(ofdm_kiss_rx_proposed_level_);
                int snr_level = ofdm_snr_to_speed_level(result.snr_db);
                add_candidate(snr_level);
                add_candidate(ofdm_kiss_rx_level_);
                add_candidate(0);  // O0 — always try as fallback
                add_candidate(ofdm_kiss_rx_level_ - 1);
                add_candidate(ofdm_kiss_rx_level_ + 1);
                for (int l = NUM_OFDM_SPEED_LEVELS - 1; l >= 0; l--)
                    add_candidate(l);

                for (int ci = 0; ci < n_candidates; ci++) {
                    int trial_level = candidates[ci];
                    // Every C=1..8 is an admitted frame shape.  Visit shortest
                    // first so a complete short frame cannot sit behind an
                    // unavailable configured/full-frame extent.
                    int cw_counts[v2::FrameGeometry::kMaxCodewordCount];
                    int n_cw_tries = 0;
                    for (int c = v2::FrameGeometry::kMinCodewordCount;
                         c <= v2::FrameGeometry::kMaxCodewordCount; ++c)
                        cw_counts[n_cw_tries++] = c;
                    for (int cwi = 0; cwi < n_cw_tries; cwi++) {
                        const std::size_t cursor = flat_shape_cursor++;
                        if (!shape_trial_due(cursor))
                            continue;
                        if (shape_work_this_callback >=
                                ACQUISITION_SHAPE_WORK_BUDGET) {
                            shape_budget_yield = true;
                            break;
                        }
                        ToneMap trial_map = ofdm_rx_tone_map_for_level(
                            trial_level, ofdm_config_, config_.ofdm_nuc);
                        trial_map.n_codewords = cw_counts[cwi];
                        OfdmDemodResult trial_result = ofdm_demod_->demodulate_trial(
                            ofdm_rx_iq_.data(), static_cast<int>(n_samples),
                            trial_map, trial_sync);
#ifndef NDEBUG
                        assert_search_pure();
#endif
                        if (trial_result.completion !=
                                OfdmDemodResult::Completion::NeedMoreSamples)
                            ++shape_work_this_callback;
                        ofdm_acquisition_.set_trial_cursors(
                            ofdm_active_candidate_id_, active_cfo_cursor,
                            flat_shape_cursor);
                        record_shape_result(trial_result, cursor);
                        if (trial_result.success && trial_result.completion ==
                                OfdmDemodResult::Completion::CompleteValidatedFrame) {
                            IRIS_LOG("[OFDM-RX] blind detect: O%d/%dCW -> O%d/%dCW (tried %d candidates)",
                                     ofdm_kiss_rx_level_, ofdm_cw_for_level(ofdm_kiss_rx_level_),
                                     trial_level, cw_counts[cwi], ci + 1);
                            result = std::move(trial_result);
                            result_level = result.demodulated_level >= 0
                                ? result.demodulated_level : trial_level;
                            result_n_codewords = result.demodulated_n_codewords;
                            blind_shape_success = true;
                            break;
                        } else {
                            select_blind_failure_evidence(
                                std::move(trial_result), trial_level);
                        }
                    }
                    if (result.success || shape_budget_yield) break;
                }
            }
        }

        // A failed shorter hypothesis is not authority to retire the preamble
        // while any longer admitted geometry still lacks samples. Keep the
        // original sync and raw capture until the largest requirement observed
        // in this finite enumeration is available.
        if (auto* candidate =
                ofdm_acquisition_.find(ofdm_active_candidate_id_)) {
            if (!candidate->incomplete_shape_work.empty()) {
                longer_shape_incomplete = true;
                const std::uint64_t origin = ofdm_acquisition_.buffer_origin();
                const std::uint64_t required_absolute =
                    candidate->incomplete_shape_required_right_context;
                if (required_absolute > origin) {
                    required_for_longer_shape = std::max(
                        required_for_longer_shape,
                        required_absolute - origin);
                }
            }
            // Incomplete shapes found under earlier CFO wraps are aggregated
            // in the candidate's bank-wide requirement when those wraps yield.
            // They must be retried if every currently ready wrap fails.
            const std::uint64_t available_absolute = sample_position_after(
                ofdm_acquisition_.buffer_origin(), n_samples);
            if (candidate->required_right_context > available_absolute) {
                longer_shape_incomplete = true;
                required_for_longer_shape = std::max(
                    required_for_longer_shape,
                    candidate->required_right_context -
                        ofdm_acquisition_.buffer_origin());
            }
        }
        if (!result.success && shape_budget_yield) {
            if (evidence_owner && owner_usable_failure(result)) {
                ofdm_search_owner_result_ = result;
                ofdm_search_owner_candidate_id_ = ofdm_active_candidate_id_;
                ofdm_search_owner_capture_epoch_ = acquisition_epoch;
                ofdm_search_owner_level_ = result_level;
                ofdm_search_owner_n_codewords_ = result_n_codewords;
                ofdm_search_owner_chase_attempted_ = false;
                ofdm_search_owner_valid_ = true;
            }
            ofdm_pending_sync_ = principal_sync;
            ofdm_sync_cached_ = true;
            // The candidate owns both the earliest incomplete cursor and its
            // required extent.  A budget yield therefore cannot turn a tested
            // NeedMoreSamples hypothesis into skipped work on the next callback.
            // Continue at the separately owned next-untried cursor on the next
            // callback. Tested incomplete cursors remain recorded and are
            // retried only when their own required right context has arrived.
            // Waiting for the largest incomplete extent here would starve a
            // shorter legal shape that lies just beyond this budget boundary.
            ofdm_pending_required_samples_ = 0;
            ofdm_acquisition_.mark_ready(ofdm_active_candidate_id_);
            ofdm_redetect_count_++;
            IRIS_LOG("[OFDM-RX] acquisition work-budget yield after %zu shape trials",
                     shape_work_this_callback);
#ifndef NDEBUG
            assert_search_pure();
#endif
            return;
        }

        // A configured or blind owner result retained before an earlier work-
        // budget yield remains this reception's trial result.  Later failed
        // shapes cannot overwrite its evidence or its owned channel snapshot.
        const bool retained_owner_for_reception =
            ofdm_search_owner_valid_ &&
            ofdm_search_owner_candidate_id_ == ofdm_active_candidate_id_ &&
            ofdm_search_owner_capture_epoch_ == acquisition_epoch;
        if (!result.success && evidence_owner &&
            !owner_usable_failure(result) && retained_owner_for_reception) {
            result = ofdm_search_owner_result_;
            result_level = ofdm_search_owner_level_;
            result_n_codewords = ofdm_search_owner_n_codewords_;
            blind_shape_success =
                result_level != (decode_map.tone_map_id > 0
                    ? static_cast<int>(decode_map.tone_map_id) - 1
                    : result_level) ||
                result_n_codewords != decode_map.n_codewords;
        }
        const bool same_chase_geometry =
            ofdm_chase_level_ == result_level &&
            ofdm_chase_n_codewords_ == result_n_codewords &&
            ofdm_chase_fec_rate_ == result.fec_rate &&
            ofdm_chase_llrs_.size() == result.llrs.size();
        const bool same_chase_reception =
            ofdm_chase_candidate_id_ == ofdm_active_candidate_id_ &&
            ofdm_chase_capture_epoch_ == acquisition_epoch;
        bool chase_attempted = false;

        // Chase probing is part of search: it reads the committed store into the
        // trial-local result but publishes nothing.  Only a distinct reception
        // with identical geometry may combine.  A re-entry of this capture can
        // only refresh its evidence after the bank reaches a final verdict.
        if (!result.success && evidence_owner &&
            result.complete_boundary_validated && !result.llrs.empty() &&
            result.snr_db >= 1.0f && !ofdm_chase_llrs_.empty() &&
            same_chase_geometry && !same_chase_reception) {
            const std::vector<float> fresh_llrs = result.llrs;
            for (std::size_t i = 0; i < result.llrs.size(); ++i)
                result.llrs[i] += ofdm_chase_llrs_[i];
            chase_attempted = true;
            IRIS_LOG("[OFDM-RX] Chase combining attempt #%d (%zu LLRs)",
                     ofdm_chase_combines_ + 1, result.llrs.size());

            auto decoded = LdpcCodec::decode_soft(result.llrs, result.fec_rate,
                                                   LdpcDecoder::MIN_SUM, 50);
            if (!decoded.empty()) {
                std::vector<uint8_t> chase_payload;
                if (OfdmDemodulator::extract_payload_blocks(
                        decoded, result.fec_rate, result.n_ldpc_blocks,
                        static_cast<size_t>(result.n_ldpc_blocks) *
                            static_cast<size_t>(LdpcCodec::block_size(result.fec_rate)),
                        chase_payload)) {
                    result.success = true;
                    result.completion =
                        OfdmDemodResult::Completion::CompleteValidatedFrame;
                    result.payload_validated = true;
                    result.cfo_resolved = true;
                    ofdm_authorize_payload_estimator(result);
                    result.payload = std::move(chase_payload);
                    result.payload_len = static_cast<uint16_t>(result.payload.size());
                }
            }
            if (!result.success)
                result.llrs = fresh_llrs;
        }

        // Acquisition-owned evidence crosses scheduler turns without publishing
        // link state.  The owner overwrites its own same-reception record when
        // more context permits a better re-demodulation.
        if (!result.success && evidence_owner &&
            result.complete_boundary_validated && !result.llrs.empty()) {
            ofdm_search_owner_result_ = result;
            ofdm_search_owner_candidate_id_ = ofdm_active_candidate_id_;
            ofdm_search_owner_capture_epoch_ = acquisition_epoch;
            ofdm_search_owner_level_ = result_level;
            ofdm_search_owner_n_codewords_ = result_n_codewords;
            ofdm_search_owner_chase_attempted_ = chase_attempted;
            ofdm_search_owner_valid_ = true;
        }

#ifndef NDEBUG
        assert_search_pure();
#endif

        // This is the sole post-search commit boundary.  Every path below either
        // publishes one final verdict or mutates acquisition bookkeeping only.
        auto commit_search_result = [&]() {
        auto retain_next_shape_extent = [&]() {
            std::uint64_t next = 0;
            if (auto* candidate =
                    ofdm_acquisition_.find(ofdm_active_candidate_id_)) {
                const std::uint64_t origin =
                    ofdm_acquisition_.buffer_origin();
                for (const auto& work : candidate->incomplete_shape_work) {
                    if (work.required_right_context <= origin) continue;
                    const std::uint64_t relative =
                        work.required_right_context - origin;
                    if (relative <= n_samples) continue;
                    next = next == 0 ? relative : std::min(next, relative);
                }
            }
            // Non-KISS has no blind-shape work list; its configured geometry
            // is the only exact missing extent.
            if (next == 0 && longer_shape_incomplete &&
                required_for_longer_shape > n_samples)
                next = required_for_longer_shape;
            if (next != 0)
                ofdm_bank_next_shape_samples_ =
                    ofdm_bank_next_shape_samples_ == 0
                        ? next
                        : std::min(ofdm_bank_next_shape_samples_, next);
        };

        if (!result.success && active_cfo_cursor + 1 < cfo_hypotheses.size()) {
            // A missing longer shape under this wrap cannot block a complete
            // short frame under another ready wrap.  Preserve the outgoing
            // wrap's aggregate context requirement, reset its wrap-local shape
            // cursor, and give the next CFO hypothesis a scheduler turn before
            // waiting or classifying a no-LLR rejection as a false candidate.
            ofdm_pending_sync_ = principal_sync;
            ofdm_sync_cached_ = true;
            ofdm_pending_required_samples_ = 0;
            retain_next_shape_extent();
            if (longer_shape_incomplete && required_for_longer_shape > 0) {
                ofdm_acquisition_.require_context(
                    ofdm_active_candidate_id_,
                    sample_position_after(ofdm_acquisition_.buffer_origin(),
                                          required_for_longer_shape));
            }
            ofdm_acquisition_.advance_cfo_trial(
                ofdm_active_candidate_id_, active_cfo_cursor + 1);
            IRIS_LOG("[OFDM-RX] CFO hypothesis m=%d exhausted ready shapes; retaining candidate for hypothesis %zu/%zu",
                     trial_sync.cfo_ambiguity_index, active_cfo_cursor + 2,
                     cfo_hypotheses.size());
            return;
        }

        if (!result.success && longer_shape_incomplete) {
            retain_next_shape_extent();
            ofdm_pending_sync_ = principal_sync;
            ofdm_sync_cached_ = true;
            const std::uint64_t next_shape_samples =
                ofdm_bank_next_shape_samples_ != 0
                    ? ofdm_bank_next_shape_samples_
                    : required_for_longer_shape;
            ofdm_pending_required_samples_ = next_shape_samples;
            // All ready wraps have now had a turn.  Restart the finite bank at
            // its principal cursor when the aggregated right context arrives.
            ofdm_acquisition_.advance_cfo_trial(
                ofdm_active_candidate_id_, 0);
            ofdm_acquisition_.require_context(
                ofdm_active_candidate_id_,
                sample_position_after(ofdm_acquisition_.buffer_origin(),
                                      next_shape_samples));
            ofdm_bank_next_shape_samples_ = 0;
            ofdm_redetect_count_++;
            IRIS_LOG("[OFDM-RX] next incomplete shape needs %llu samples; longer extents remain retained",
                     static_cast<unsigned long long>(next_shape_samples));
            return;
        }

        // A short expect-ACK gate is a search retry, not a link verdict.  Decide
        // it before publishing Chase/HARQ, meters, demotion, or a buffer drain.
        if (ofdm_root2_retain_on_short_ack_fail(short_ack_gate_used,
                                                result.success,
                                                !result.llrs.empty())) {
            ofdm_sync_cached_ = false;
            IRIS_LOG("[OFDM-RX] ROOT-2: 1-CW gate truncated a multi-CW reverse frame — "
                     "retaining buffer, expanded gate will re-buffer the full frame");
            return;
        }

        if (ofdm_sync_cached_) {
            IRIS_LOG("[OFDM-RX] frame resolved after %d waits", ofdm_redetect_count_);
        }
        ofdm_sync_cached_ = false;
        ofdm_pending_required_samples_ = 0;
        ofdm_redetect_count_ = 0;
        // climbgate diagnostic: one line per OFDM frame decode attempt so the live
        // vs forced-level 32QAM question is answerable from the log (success, LDPC
        // iters, the effective SNR the gearshift sees, and the biased LLR sigma).
        {
            static const bool climbdiag = std::getenv("IRIS_CLIMB_DIAG") != nullptr;
            if (climbdiag)
                IRIS_LOG("[CLIMBDIAG] rx level=O%d ncw=%d succ=%d iters=%d eff_snr=%.1f "
                         "ch_snr=%.1f sigma_llr=%.4f meanH=%.3f",
                         ofdm_kiss_rx_level_, ofdm_rx_tone_map_.n_codewords,
                         (int)result.success, result.worst_ldpc_iters,
                         result.effective_snr_db, result.mean_channel_snr_db,
                         result.dft_sigma_sq_llr, result.mean_H_mag);
        }

        const bool owner_evidence_for_reception =
            ofdm_search_owner_valid_ &&
            ofdm_search_owner_candidate_id_ == ofdm_active_candidate_id_ &&
            ofdm_search_owner_capture_epoch_ == acquisition_epoch;
        const OfdmDemodResult* owner_evidence = owner_evidence_for_reception
            ? &ofdm_search_owner_result_ : nullptr;

        // Publish only the result selected by the post-search verdict.  Failed
        // trials that ran later cannot replace the owner result's H snapshot.
        const OfdmDemodResult* channel_owner = result.success
            ? &result : owner_evidence;
        if (channel_owner && !channel_owner->channel_estimate.H.empty())
            ofdm_demod_->commit_channel_estimate(
                channel_owner->channel_estimate);

        auto finalized_failure_consumed = [&]() -> std::size_t {
            if (owner_evidence && owner_evidence->complete_boundary_validated &&
                owner_evidence->consumed_from_input_start > 0) {
                return static_cast<std::size_t>(
                    owner_evidence->consumed_from_input_start);
            }
            const std::size_t examined = principal_sync.timing_interval_begin >= 0
                ? static_cast<std::size_t>(
                    principal_sync.timing_interval_begin + 1)
                : static_cast<std::size_t>(principal_sync.frame_start + 1);
            return std::max<std::size_t>(examined, 1);
        };
        if (!result.success && owner_evidence_for_reception) {
            const auto& owner = *owner_evidence;

            // The Chase guard/store is committed only after the whole bank has
            // failed.  Geometry and reception identity are independent keys:
            // equal geometry plus a different reception combines; every other
            // case refreshes without incrementing the combine counter.
            if (owner.snr_db < 1.0f) {
                if (!ofdm_chase_llrs_.empty()) {
                    IRIS_LOG("[OFDM-RX] Chase guard: flushing LLRs (SNR=%.1f dB too low, likely noise)",
                             owner.snr_db);
                    ofdm_chase_llrs_.clear();
                    ofdm_chase_combines_ = 0;
                    ofdm_chase_candidate_id_ = 0;
                    ofdm_chase_capture_epoch_ = 0;
                }
            } else if (owner.complete_boundary_validated && !owner.llrs.empty()) {
                if (ofdm_search_owner_chase_attempted_)
                    ++ofdm_chase_combines_;
                ofdm_chase_llrs_ = owner.llrs;
                ofdm_chase_candidate_id_ = ofdm_active_candidate_id_;
                ofdm_chase_capture_epoch_ = acquisition_epoch;
                ofdm_chase_level_ = ofdm_search_owner_level_;
                ofdm_chase_n_codewords_ = ofdm_search_owner_n_codewords_;
                ofdm_chase_fec_rate_ = owner.fec_rate;
                IRIS_LOG("[OFDM-RX] %s %zu LLRs for Chase combining",
                         ofdm_search_owner_chase_attempted_ ? "refreshed" : "stored",
                         ofdm_chase_llrs_.size());
            }

            if (!owner.block_results.empty()) {
                ofdm_harq_evidence_ = {};
                ofdm_harq_evidence_.any_failed = true;
                ofdm_harq_evidence_.blocks = owner.block_results;
                ofdm_harq_evidence_.stored_llrs = owner.llrs;
                ofdm_harq_evidence_.sym_phase_var = owner.sym_phase_var;
                ofdm_harq_evidence_.fec = owner.fec_rate;
                ofdm_harq_evidence_.payload_len = owner.payload_len;
                ofdm_harq_evidence_.num_blocks = owner.n_ldpc_blocks;
                ofdm_harq_evidence_candidate_id_ = ofdm_active_candidate_id_;
                ofdm_harq_evidence_capture_epoch_ = acquisition_epoch;
                ofdm_harq_evidence_valid_ = true;
            }
        }
        if (!result.success)
            ofdm_search_owner_valid_ = false;

        // Notify ARQ only after the finite CFO bank is exhausted, from the
        // committed-Hz owner retained above.  This precedes no-LLR retirement.
        if (!result.success && !ofdm_kiss_ &&
            arq_.state() != ArqState::IDLE && arq_.negotiated(CAP_HARQ) &&
            ofdm_harq_evidence_candidate_id_ == ofdm_active_candidate_id_ &&
            ofdm_harq_evidence_capture_epoch_ == acquisition_epoch &&
            ofdm_harq_evidence_valid_ &&
            !ofdm_harq_evidence_.blocks.empty()) {
            arq_.on_decode_failed_harq(ofdm_harq_evidence_);
            IRIS_LOG("[OFDM-RX] HARQ: fed %d blocks (%zu LLRs) from committed-CFO owner",
                     ofdm_harq_evidence_.num_blocks,
                     ofdm_harq_evidence_.stored_llrs.size());
            ofdm_harq_evidence_valid_ = false;
        }

        // ---- Secondary gate: false positive ----
        // If no LLRs were produced, normalized detection or scale-free channel
        // validity rejected the hypothesis. Advance narrowly and keep scanning.
        if (!result.success && result.llrs.empty()) {
            // Don't clear ofdm_expect_ack_ on CMD — MFSK tones trigger quality
            // gate (low mean|H|) and we need to keep the MFSK detection gate open.
            if (ofdm_expect_ack_ && ax25_session_.we_initiated()) {
                ofdm_sync_cached_ = false;  // Allow MFSK detection on next callback
            } else {
                ofdm_expect_ack_ = false;
            }
            IRIS_LOG("[OFDM-RX] false positive / quality gate (mean_H=%.3f, M=%.3f) — skipping",
                     result.mean_H_mag, principal_sync.schmidl_metric);
            // A validated owner boundary finalizes this reception: retire its
            // complete input-relative extent so the same waveform cannot be
            // rediscovered under a fresh identity.  Without owner evidence,
            // retain the narrow false-positive retirement policy.
            size_t skip = finalized_failure_consumed();
            skip = std::min(skip, ofdm_rx_audio_buf_.size());
            ofdm_acquisition_.mark_rejected(ofdm_active_candidate_id_);
            const std::uint64_t retired_end =
                sample_position_after(ofdm_acquisition_.buffer_origin(), skip);
            ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                      ofdm_rx_audio_buf_.begin() + skip);
            ofdm_acquisition_.retire_prefix(retired_end);
            ofdm_active_candidate_id_ = 0;
            return;
        }

        if (result.success && result.completion ==
                OfdmDemodResult::Completion::CompleteValidatedFrame) {
            // Payload validation is the only event that re-commits link CFO.
            ofdm_cfo_committed_hz_ = result.cfo_hz;

            if (blind_shape_success) {
                // Persist the peer's modulation/rate, never this frame's
                // codeword count.  Search only selected these values; commit is
                // the first point at which link state changes.
                ofdm_kiss_rx_level_ = result_level;
                ofdm_rx_tone_map_ = ofdm_rx_tone_map_for_level(
                    result_level, ofdm_config_, config_.ofdm_nuc);
                ofdm_kiss_rx_confirmed_ = true;
            }

            if (chase_attempted) {
                ++ofdm_chase_combines_;
                IRIS_LOG("[OFDM-RX] Chase combining SUCCEEDED after %d combines (%zu bytes)",
                         ofdm_chase_combines_, result.payload.size());
            }

            // Clear stale chase LLRs on a successful decode of the SAME LLR
            // shape — the stored frame (or a same-shape sibling) landed, so the
            // store is stale (cross-frame contamination guard). A DIFFERENT
            // shape says nothing about the stored frame: a 20 B / 1-block
            // control frame decoding between retransmissions of a failing
            // multi-block I-frame must NOT flush the I-frame's accumulated
            // copies — that flush is what kept Chase combining from ever
            // accumulating across the measured retx wedge (49 TXes, 6
            // interleaved short OKs, 0 combines to success:
            // data-flow-tx-anchor.md §5 D1).
            const bool same_success_geometry =
                ofdm_chase_level_ == result_level &&
                ofdm_chase_n_codewords_ == result_n_codewords &&
                ofdm_chase_fec_rate_ == result.fec_rate;
            if (!ofdm_chase_llrs_.empty() && same_success_geometry) {
                ofdm_chase_llrs_.clear();
                ofdm_chase_combines_ = 0;
                ofdm_chase_candidate_id_ = 0;
                ofdm_chase_capture_epoch_ = 0;
            }
            ofdm_search_owner_valid_ = false;
            ofdm_expect_ack_ = false;  // ACK (or data) received
            // Re-confirm RX level on successful decode.  A prior failure sets
            // ofdm_kiss_rx_confirmed_=false which expands the frame-length gate
            // to the maximum frame size across all speed levels (~1.8s at O2).
            // The expanded gate keeps ofdm_sync_cached_=true for 1-2s per frame,
            // blocking RSP TX (via the sync_cached guard in process_tx).  This
            // cascades: CMD T1 fires, polls accumulate, session degrades to
            // poll-only mode.  Fix: any successful decode at the current level
            // proves the gate can tighten back to the expected frame size.
            if (ofdm_kiss_tx_ && !ofdm_kiss_rx_confirmed_) {
                ofdm_kiss_rx_confirmed_ = true;
                IRIS_LOG("[OFDM-RX] RX level re-confirmed at O%d (successful decode)",
                         ofdm_kiss_rx_level_);
            }
            // OFDM-KISS payloads are counted by dispatch_rx_frame(); native ARQ
            // payloads have no dispatch hop and are counted here.  Filtered or
            // transform-failed KISS frames are counted at their terminal branch.
            if (!ofdm_kiss_)
                frames_rx_++;
            tx_no_ack_count_ = 0;
            const bool estimator_valid = result.cfo_resolved &&
                result.payload_validated &&
                result.estimator_validity ==
                    OfdmEstimatorValidity::PayloadValidatedForSelectedCfo;
            if (estimator_valid) {
                snr_db_ = result.snr_db;
                snr_preamble_db_ = result.snr_db;
            }

            IRIS_LOG("[OFDM-RX] frame OK: %zu bytes, SNR=%.1f dB, ch_SNR=%.1f dB, %d LDPC blocks",
                     result.payload.size(), result.snr_db, result.mean_channel_snr_db,
                     result.n_ldpc_blocks);

            // TUNE gain measurement from OFDM channel estimate.
            // Power-ramp: track per-frame LDPC iters and mean|H|.
            if (estimator_valid && tune_state_ == TuneState::WAIT_PEER &&
                native_selfhear_guard_ <= 0) {
                float gain = 0.0f;
                const auto& est = result.channel_estimate;
                if (!est.H.empty()) {
                    float sum = 0.0f;
                    for (const auto& h : est.H) sum += std::abs(h);
                    gain = sum / (float)est.H.size();
                }
                if (gain > 0.01f) {
                    int idx = tune_frames_measured_;
                    tune_frames_measured_++;
                    if (idx < TUNE_RAMP_COUNT) {
                        tune_rx_frame_iters_[idx] = result.worst_ldpc_iters;
                        tune_rx_frame_H_[idx] = gain;
                        tune_rx_frame_snr_[idx] = result.snr_db;
                    }
                    if (tune_my_gain_ == 0)
                        tune_my_gain_ = gain;
                    else
                        tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;
                    IRIS_LOG("[TUNE] OFDM ramp frame %d: H=%.3f ldpc=%d iters (avg_H=%.3f)",
                             idx + 1, gain, result.worst_ldpc_iters, tune_my_gain_);
                    tune_audit("MEASURE peer=%s frame=%d raw_gain=%.4f ldpc_iters=%d decode=OK",
                               tune_peer_call_.c_str(), idx + 1, gain, result.worst_ldpc_iters);
                }
                // Extract embedded binary report from responder's ramp payload
                if (!result.payload.empty() && result.payload[0] == TUNE_REPORT_MAGIC) {
                    if (tune_parse_binary_report(result.payload.data(), result.payload.size(),
                            tune_peer_iters_, tune_peer_H_, tune_peer_snr_, TUNE_RAMP_COUNT)) {
                        int rpt_count = 0;
                        for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++)
                            if (tune_peer_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE] Extracted embedded report: %d entries from ramp payload",
                                 rpt_count);
                        tune_audit("EMBEDDED_REPORT entries=%d", rpt_count);
                    }
                }
            }

            // Also check for report in frames received during WAIT_REPORT
            // (initiator's dedicated report frame after ramp)
            if (tune_state_ == TuneState::WAIT_REPORT &&
                !result.payload.empty() && result.payload[0] == TUNE_REPORT_MAGIC) {
                // Debug: log raw report bytes for diagnostics
                {
                    std::string hex;
                    for (size_t bi = 0; bi < std::min(result.payload.size(), (size_t)32); bi++) {
                        char hb[4]; snprintf(hb, sizeof(hb), "%02X ", result.payload[bi]);
                        hex += hb;
                    }
                    IRIS_LOG("[TUNE] report payload (%zu bytes): %s", result.payload.size(), hex.c_str());
                }
                if (tune_parse_binary_report(result.payload.data(), result.payload.size(),
                        tune_peer_iters_, tune_peer_H_, tune_peer_snr_, TUNE_RAMP_COUNT)) {
                    int rpt_count = 0;
                    for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++) {
                        if (tune_peer_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE]   slot[%d]: iters=%d H=%.3f snr=%.1f",
                                 ri, tune_peer_iters_[ri], tune_peer_H_[ri], tune_peer_snr_[ri]);
                    }
                    IRIS_LOG("[TUNE] Received OFDM report frame: %d entries", rpt_count);
                    tune_audit("OFDM_REPORT entries=%d", rpt_count);
                    tune_state_ = TuneState::APPLY;
                }
            }

            // Update OFDM gearshift via Gearshift class (smoothing, LDPC boost,
            // failure downshift, cooldown — same adaptive logic as Mode A).
            // Skip during TUNE: test frames have artificially high preamble SNR
            // (full-power tones, no data) that would cause premature O0→O1 upshift.
            // OFDM-KISS: gearshift drives LOCAL TX level. Peer uses S-frame signaling
            // + blind detection to track our level. Both sides independently ramp
            // toward the optimal level for the measured channel SNR.
            bool in_tune = (tune_state_ != TuneState::IDLE &&
                            tune_state_ != TuneState::DONE);
            if (!in_tune && estimator_valid) {
                gearshift_.feed_ldpc_iters(result.worst_ldpc_iters, 50);
                int old_level = ofdm_speed_level_;
                // UPPER-LADDER ROOT (climb metric): the gearshift upshift decision
                // must see the real CHANNEL SNR. mean_channel_snr_db is the honest
                // in-band meter: per-carrier |H|^2/sigma^2(k) with sigma^2(k)
                // measured from the training-symbol pair (ofdm_demod.cc step 4c) —
                // NOT the guard-bin scalar, which the live RX band-limit blinds
                // (it averages exactly the bins the band-limit zeroes; it read
                // 46-49 dB regardless of the channel, fact doc
                // data-flow-noise-var.md §8). The scale is TRUE in-band SNR
                // (injection-calibrated); the O-ladder min_snr_db thresholds are
                // true-SNR values. The LDPC hard-gate + report_failure() demote
                // find the real ceiling when decode margin, not SNR, binds.
                // effective_snr_db (the fallback when the channel estimate is
                // invalid <=0) is the post-despread gamma_eff from the same
                // in-band sigma^2(k) for QAM16+, or the post-EQ decision-residual
                // EsNo for QPSK — both honest, both lower-or-equal to channel SNR.
                // Env IRIS_CLIMB_CHSNR=0 restores the old residual feed (A/B).
                static const bool climb_chsnr = []() {
                    const char* e = std::getenv("IRIS_CLIMB_CHSNR");
                    return !e || std::atoi(e) != 0; }();
                float climb_snr = (climb_chsnr && result.mean_channel_snr_db > 0.0f)
                    ? result.mean_channel_snr_db
                    : result.effective_snr_db;
                ofdm_speed_level_ = gearshift_.ofdm_update(climb_snr);
                // Feed modem gearshift state to ARQ for conflict detection
                arq_.set_modem_gearshift(ofdm_speed_level_, gearshift_.smoothed_snr(),
                                         gearshift_.cooldown());
                if (ofdm_speed_level_ != old_level) {
                    float eff_tx = ofdm_effective_tx_level();
                    IRIS_LOG("[OFDM-RX] gearshift: O%d -> O%d (frame_SNR=%.1f dB, ch_SNR=%.1f, boost=%.1f, cd=%d, tx=%.3f)",
                             old_level, ofdm_speed_level_, result.snr_db,
                             result.mean_channel_snr_db,
                             gearshift_.boost(), gearshift_.cooldown(), eff_tx);
                    // Speed level changed — stored Chase LLRs are for a different
                    // tone map / FEC rate and would corrupt combining
                    ofdm_chase_llrs_.clear();
                    ofdm_chase_combines_ = 0;
                    ofdm_chase_candidate_id_ = 0;
                    ofdm_chase_capture_epoch_ = 0;
                }
            } else if (in_tune) {
                IRIS_LOG("[OFDM-RX] TUNE frame: skipping gearshift (ch_SNR=%.1f dB)",
                         result.mean_channel_snr_db);
            } else {
                IRIS_LOG("[OFDM-RX] estimator invalid for selected CFO; withholding rate evidence");
            }
            if (estimator_valid)
                arq_.set_local_snr(result.snr_db);

            // Feed equalized constellation to GUI (scatter plot)
            if (!result.eq_constellation.empty()) {
                last_constellation_ = std::move(result.eq_constellation);
            }

            // Feed OFDM Kalman trace to GUI 3D viewer + CSV logging
            if (!result.kalman_trace.fwd.empty()) {
                last_kalman_trace_ = result.kalman_trace;
                kalman_log_trace(last_kalman_trace_, result.success,
                                 result.snr_db, result.mean_H_mag);
            }

            // Waterfilling: DISABLED for FM mode.
            // The channel estimate SNR (from preamble |H|) overestimates link quality
            // because the FM deviation limiter clips high-PAPR data symbols but not
            // the low-PAPR ZC preamble. After one successful QPSK frame, waterfill
            // jumped to 256QAM (222 bits/sym) based on 33.7 dB channel SNR while
            // actual frame SNR was -1.3 dB. All subsequent frames failed.
            // Rate adaptation is handled by gearshift via uniform presets.
            // if (config_.ofdm_waterfill && !result.snr_per_carrier.empty()) { ... }

            // Payload delivery — reuse existing deliver + decompression logic
            std::vector<uint8_t> payload = std::move(result.payload);

            // OFDM-KISS batch decompression
            if (ofdm_kiss_ && payload.size() >= 2 &&
                payload[0] == COMPRESSED_PAYLOAD_MAGIC &&
                (ofdm_kiss_peer_caps_ & CAP_COMPRESSION)) {
                auto decoded = ofdm_kiss_rx_compressor_.decompress_record(
                    payload.data() + 1, payload.size() - 1);
                if (decoded.status == v2::TransformStatus::Produced) {
                    IRIS_LOG("OFDM-KISS decompress: %zu -> %d bytes",
                             payload.size() - 1, (int)decoded.produced_bytes.size());
                    payload = std::move(decoded.produced_bytes);
                } else {
                    ++frames_rx_;
                    fail_ofdm_transform("OFDM receive decompression", {payload});
                    size_t consumed_samples =
                        (result.consumed_from_input_start > 0)
                        ? static_cast<size_t>(result.consumed_from_input_start)
                        : static_cast<size_t>(principal_sync.frame_start +
                                              ofdm_config_.nfft * 4);
                    size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
                    ofdm_acquisition_.mark_validated(ofdm_active_candidate_id_);
                    const std::uint64_t retired_end =
                        sample_position_after(
                            ofdm_acquisition_.buffer_origin(), consumed);
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + consumed);
                    ofdm_acquisition_.retire_prefix(retired_end);
                    ofdm_active_candidate_id_ = 0;
                    rx_overlap_buf_.clear();
                    return;
                }
            }

            // Filter TUNE payloads — not AX.25 data.  Without this filter,
            // "TUNE_TEST_FRAME" (15 bytes) and binary reports (0xBB prefix)
            // are dispatched to the KISS client as garbage AX.25 frames,
            // potentially corrupting AX.25 session state (N(R) stuck at 0).
            {
                static const uint8_t tune_marker[] = "TUNE_TEST_FRAME";
                bool is_tune_test = (payload.size() == sizeof(tune_marker) - 1 &&
                    memcmp(payload.data(), tune_marker, payload.size()) == 0);
                // Validate TUNE report structure: [0xBB] [count] [5-byte entries].
                // A single magic byte is too fragile — compressed AX.25 data can
                // start with 0xBB. Require valid length to avoid false positives.
                bool is_tune_report = false;
                if (payload.size() >= 2 && payload[0] == TUNE_REPORT_MAGIC) {
                    int count = payload[1];
                    size_t expected = 2 + count * 5;
                    is_tune_report = (payload.size() == expected);
                }
                if (is_tune_test || is_tune_report) {
                    if (ofdm_kiss_)
                        ++frames_rx_;
                    IRIS_LOG("[TUNE] Discarded OFDM %s frame (not dispatching to AX.25)",
                             is_tune_test ? "test" : "report");
                    // Drain consumed samples so the same frame isn't re-detected.
                    // Without this, self-hear audio loops: the preamble stays in
                    // ofdm_rx_audio_buf_ and is decoded 100+ times (OTA bug 2026-03-23).
                    size_t consumed_samples =
                        (result.consumed_from_input_start > 0)
                        ? static_cast<size_t>(result.consumed_from_input_start)
                        : static_cast<size_t>(principal_sync.frame_start +
                                              ofdm_config_.nfft * 4);
                    size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
                    ofdm_acquisition_.mark_validated(ofdm_active_candidate_id_);
                    const std::uint64_t retired_end =
                        sample_position_after(
                            ofdm_acquisition_.buffer_origin(), consumed);
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + consumed);
                    ofdm_acquisition_.retire_prefix(retired_end);
                    ofdm_active_candidate_id_ = 0;
                    rx_overlap_buf_.clear();
                    return;
                }
            }

            // Deliver payload(s)
            auto deliver_ofdm = [&](const uint8_t* data, size_t len) {
                if (ofdm_kiss_) {
                    if (!ofdm_kiss_confirmed_) {
                        ofdm_kiss_confirmed_ = true;
                        IRIS_LOG("OFDM-KISS: confirmed (heard OFDM from peer)");
                        if (gui_log_) gui_log_("OFDM-KISS: bidirectional confirmed");
                    }
                    if (!ofdm_kiss_tx_ && !ofdm_config_mismatch_) {
                        ofdm_kiss_tx_ = true;
                        ax25_session_.set_native_active(true);
                        // Restore the responder's T1 backstop for the activation
                        // window (item 6): start_t1_if_unacked() no-ops while
                        // native_active_ is false (ax25_session.cc:352), so
                        // 34a4865's TX-hold left a gap when the Winlink responder
                        // speaks first. Now native_active_ is set — arm T1.
                        ax25_session_.start_t1_if_unacked();
                        IRIS_LOG("OFDM-KISS: TX promoted (responder, OFDM)");
                    }
                    std::vector<uint8_t> ax25_frame(data, data + len);
                    dispatch_rx_frame(ax25_frame, false, true);
                    return;
                }
                auto arq_st = arq_.state();
                if (!loopback_mode_ &&
                    (arq_st == ArqState::CONNECTED || arq_st == ArqState::CONNECTING ||
                     arq_st == ArqState::LISTENING || arq_st == ArqState::HAILING ||
                     arq_st == ArqState::DISCONNECTING)) {
                    if (!arq_.on_frame_received(data, len)) {
                        if (rx_callback_) rx_callback_(data, len);
                    }
                } else {
                    if (rx_callback_) rx_callback_(data, len);
                }
            };

            if (payload.size() >= 4 && payload[0] == MULTI_PAYLOAD_MAGIC) {
                // #2: byte 1 is the sender's forward burst-epoch — remember it so our
                // reverse ACK (MFSK suffix + S-frame tail) echoes it back for binding.
                ofdm_kiss_rx_burst_epoch_ = payload[1];
                size_t pos = 2;  // skip [magic][epoch]
                while (pos + 2 <= payload.size()) {
                    uint16_t sub_len = payload[pos] | ((uint16_t)payload[pos + 1] << 8);
                    pos += 2;
                    if (sub_len == 0 || pos + sub_len > payload.size()) break;
                    deliver_ofdm(&payload[pos], sub_len);
                    pos += sub_len;
                }
            } else {
                deliver_ofdm(payload.data(), payload.size());
            }
        } else {
            crc_errors_++;
            gearshift_.report_failure();
            IRIS_LOG("[OFDM-RX] frame FAIL at offset %d, SNR=%.1f dB (gearshift->O%d)",
                     trial_sync.frame_start, trial_sync.snr_est,
                     gearshift_.current_ofdm_level());
            ofdm_speed_level_ = gearshift_.current_ofdm_level();

            // Reset RX level confirmation so the frame-length gate expands to
            // the maximum possible frame size on the next attempt.  Without this,
            // a peer upshift (e.g. O0->O2) produces longer frames that exceed the
            // confirmed-level gate, causing "insufficient samples" truncation and
            // permanent decode failure on all subsequent frames.
            if (ofdm_kiss_tx_) {
                ofdm_kiss_rx_confirmed_ = false;
                // CMD expecting MFSK ACK: OFDM SC false-triggers on MFSK tones,
                // caches sync, LDPC fails. Clear cached sync so MFSK detection
                // runs on the next callback. Keep ofdm_expect_ack_ true so the
                // MFSK gate stays open until the ACK is found or T1 fires.
                if (ofdm_expect_ack_ && ax25_session_.we_initiated()) {
                    ofdm_sync_cached_ = false;
                    // Don't clear ofdm_expect_ack_ — keep looking for MFSK ACK
                } else {
                    ofdm_expect_ack_ = false;
                }
            }

            // A coherent repeated pair can be a wrong CFO wrap.  Failed payload
            // validation therefore leaves its channel/SNR estimator unresolved;
            // never let it update TUNE/rate measurements.
            if (result.estimator_validity ==
                    OfdmEstimatorValidity::PayloadValidatedForSelectedCfo &&
                tune_state_ == TuneState::WAIT_PEER && native_selfhear_guard_ <= 0
                && !ptt_active_ && !rx_muted_) {
                float gain = 0.0f;
                const auto& est = result.channel_estimate;
                if (!est.H.empty()) {
                    float sum = 0.0f;
                    for (const auto& h : est.H) sum += std::abs(h);
                    gain = sum / (float)est.H.size();
                }
                // Sanity check: reject physically implausible H values.
                // Real OFDM frames produce H in ~0.1-20 range. False detections
                // on AFSK/noise produce extreme or near-zero values.
                if (gain > 0.1f && gain < 50.0f) {
                    // Preamble-only: update H estimate but do NOT count as
                    // measured frame. Only LDPC-decoded frames count toward
                    // WAIT_PEER exit — preamble-only may be false detections.
                    if (tune_my_gain_ == 0)
                        tune_my_gain_ = gain;
                    else
                        tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;
                    IRIS_LOG("[TUNE] OFDM preamble-only: H=%.3f SNR=%.1f dB (avg_H=%.3f, not counted)",
                             gain, trial_sync.snr_est, tune_my_gain_);
                    tune_audit("PREAMBLE_ONLY peer=%s raw_gain=%.4f snr=%.1f (not counted)",
                               tune_peer_call_.c_str(), gain, trial_sync.snr_est);
                } else if (gain > 0.0f) {
                    IRIS_LOG("[TUNE] Rejected preamble-only H=%.3f (outside 0.1-50 range, likely false detection)",
                             gain);
                }
            }

        }

        if (result.success)
            ofdm_acquisition_.mark_validated(ofdm_active_candidate_id_);
        else
            ofdm_acquisition_.mark_rejected(ofdm_active_candidate_id_);

        // Drain consumed samples from OFDM audio buffer
        size_t consumed_samples = 0;
        if (result.success) {
            consumed_samples = (result.consumed_from_input_start > 0)
                ? static_cast<size_t>(result.consumed_from_input_start)
                : static_cast<size_t>(principal_sync.frame_start +
                                      ofdm_config_.nfft * 4);
        } else {
            consumed_samples = finalized_failure_consumed();
        }
        size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
        const std::uint64_t retired_end =
            sample_position_after(ofdm_acquisition_.buffer_origin(), consumed);
        ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                  ofdm_rx_audio_buf_.begin() + consumed);
        ofdm_acquisition_.retire_prefix(retired_end);
        ofdm_active_candidate_id_ = 0;
        return;
        };

        commit_search_result();
        return;
    }

    // ============ Downconvert for native PHY ============
    // (Only reached when OFDM is not active — OFDM path returns above)
    const float* iq_data;
    std::vector<float> iq_buf;
    size_t iq_count;

    if (use_upconvert_) {
        if (rx_channel_eq_.is_configured()) {
            // Pre-allocated EQ scratch buffer (avoids per-callback heap allocation)
            if (rx_eq_tmp_.size() < (size_t)count)
                rx_eq_tmp_.resize(count);
            std::copy(audio, audio + count, rx_eq_tmp_.data());
            rx_channel_eq_.apply(rx_eq_tmp_.data(), count);
            iq_buf = downconverter_.audio_to_iq(rx_eq_tmp_.data(), count);
        } else {
            iq_buf = downconverter_.audio_to_iq(audio, count);
        }
        iq_data = iq_buf.data();
        iq_count = iq_buf.size();
    } else {
        iq_data = audio;
        iq_count = count * 2;
    }

    if (native_rx_gain_ != 1.0f && !iq_buf.empty()) {
        for (auto& s : iq_buf) s *= native_rx_gain_;
        iq_data = iq_buf.data();
    }

    rx_overlap_buf_.insert(rx_overlap_buf_.end(), iq_data, iq_data + iq_count);

    if (rx_overlap_buf_.size() > RX_OVERLAP_MAX) {
        size_t excess = rx_overlap_buf_.size() - RX_OVERLAP_MAX;
        rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                               rx_overlap_buf_.begin() + excess);
    }

    // ============ Legacy single-carrier PHY RX path ============
    // If we have a pending frame waiting for more data, skip detection
    if (pending_frame_start_ >= 0) {
        if (pending_frame_timeout_ <= 0) {
            // Timeout expired — abandon this frame (sender may have stopped)
            IRIS_LOG("RX pending frame TIMEOUT at offset %d (needed %zu IQ, have %zu)",
                     pending_frame_start_, pending_need_floats_ / 2,
                     rx_overlap_buf_.size() / 2);
            pending_frame_start_ = -1;
            pending_frame_timeout_ = 0;
        } else if (rx_overlap_buf_.size() < pending_need_floats_) {
            return;  // Still not enough data, skip expensive work
        } else {
            IRIS_LOG("RX pending retry: buf=%zu floats, need=%zu, start=%d",
                     rx_overlap_buf_.size(), pending_need_floats_, pending_frame_start_);
        }
    }

    int start;
    float det_corr;
    if (pending_frame_start_ >= 0) {
        // Re-use cached detection result
        start = pending_frame_start_;
        det_corr = 0.9f;  // Known good
        pending_frame_start_ = -1;
        pending_frame_timeout_ = 0;
    } else {
        start = detect_frame_start(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                        phy_config_.samples_per_symbol);
        det_corr = detect_best_corr();
    }

    if (start >= 0) {
        // Trim pre-frame silence to maximize buffer space for payload.
        // Keep enough margin for RRC filter priming (RRC_SPAN * SPS samples).
        int rrc_margin = RRC_SPAN * phy_config_.samples_per_symbol + 10;
        if (start > rrc_margin + 50) {
            size_t trim = (size_t)(start - rrc_margin) * 2;
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + trim);
            start = rrc_margin;
        }

        std::vector<uint8_t> payload;
        // Always capture Kalman trace (even on decode failure — essential for debugging)
        bool decode_ok = decode_native_frame(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                              start, phy_config_, payload);
        last_kalman_trace_ = decode_kalman_trace();
        kalman_log_trace(last_kalman_trace_, decode_ok,
                         decode_snr_db(), decode_channel_gain());

        // Auto-tune: capture channel_gain from peer's test frame.
        // channel_gain comes from preamble — valid even on decode failure.
        // Skip self-heard frames: after our own TX, the overlap buffer may
        // still contain our echoed frames with inflated gain.
        if (tune_state_ == TuneState::WAIT_PEER && native_selfhear_guard_ <= 0) {
            float gain = decode_channel_gain();
            if (gain > 0.01f) {
                tune_frames_measured_++;
                if (tune_my_gain_ == 0)
                    tune_my_gain_ = gain;
                else
                    tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;  // EMA
                IRIS_LOG("[TUNE] frame %d: gain=%.4f avg=%.4f (decode=%s)",
                         tune_frames_measured_, gain, tune_my_gain_,
                         decode_ok ? "OK" : "FAIL");
                tune_audit("MEASURE peer=%s frame=%d raw_gain=%.4f ema_gain=%.4f decode=%s",
                           tune_peer_call_.c_str(), tune_frames_measured_, gain,
                           tune_my_gain_, decode_ok ? "OK" : "FAIL");
                // Don't transition here — tick() handles WAIT_PEER→SEND_REPORT
                // after accumulating enough frames or timeout.
            }
        }

        if (decode_ok) {
            // Self-hear guard: discard frames decoded from our own TX echo
            if (native_selfhear_guard_ > 0) {
                IRIS_LOG("RX native frame %zu bytes DISCARDED (self-hear guard, %d samples remaining)",
                         payload.size(), native_selfhear_guard_.load());
                // Skip past this frame so we don't re-decode it
                size_t skip = (size_t)(start + 100) * 2;
                if (skip < rx_overlap_buf_.size())
                    rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                           rx_overlap_buf_.begin() + skip);
                else
                    rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                pending_frame_timeout_ = 0;
                return;
            }
            frames_rx_++;
            // Successful native RX proves the link is alive.  Reset the no-ack
            // counter so gearshift doesn't penalise us for sending S-frame ACKs
            // that the peer acknowledges implicitly via I-frame N(R) — which the
            // S-frame–only sniff in the rx_holdoff path doesn't catch.
            tx_no_ack_count_ = 0;
            IRIS_LOG("RX native frame %zu bytes at offset %d", payload.size(), start);

            {
                // Use SNR computed inside the frame decoder, which has:
                // - RRC-filtered samples
                // - Fine timing (sub-sample interpolation)
                // - Phase + frequency offset correction
                // This is much more accurate than external estimation.
                float snr = decode_snr_db();
                snr_db_ = snr;
                snr_preamble_db_ = decode_snr_preamble_db();
                int ldpc_iters = ldpc_last_max_iters();
                // Don't feed gearshift during auto-tune — tune test frames
                // may decode at higher SNR than actual data (different direction,
                // different gain), causing premature upshift.
                bool in_tune = (tune_state_ != TuneState::IDLE &&
                                tune_state_ != TuneState::DONE);
                if (!in_tune) {
                    gearshift_.feed_ldpc_iters(ldpc_iters, 50);
                    int old_level = gearshift_.current_level();
                    gearshift_.update(snr);
                    int new_level = gearshift_.current_level();
                    IRIS_LOG("[SNR] est=%.1f dB, ldpc_iters=%d, boost=+%.1f, eff=%.1f, gearshift: %d->%d (smoothed=%.1f, cd=%d)",
                             snr, ldpc_iters, gearshift_.boost(),
                             gearshift_.smoothed_snr() + gearshift_.boost(),
                             old_level, new_level, gearshift_.smoothed_snr(), gearshift_.cooldown());
                    // Feed modem gearshift state to ARQ for conflict detection
                    arq_.set_modem_gearshift(new_level, gearshift_.smoothed_snr(),
                                             gearshift_.cooldown());
                } else {
                    IRIS_LOG("[SNR] est=%.1f dB, ldpc_iters=%d (tune frame, gearshift skipped)",
                             snr, ldpc_iters);
                }
                arq_.set_local_snr(snr);
            }

            if (native_demod_)
                last_constellation_ = native_demod_->symbols();

            // Deliver payload(s) — split multi-payload frames
            bool transform_failed = false;
            auto deliver = [&](const uint8_t* data, size_t len) {
                if (transform_failed) return;
                if (ofdm_kiss_) {
                    // OFDM-KISS: payload is a raw AX.25 frame — dispatch through
                    // normal AX.25 path for session handling + KISS forwarding.
                    if (!ofdm_kiss_confirmed_) {
                        ofdm_kiss_confirmed_ = true;
                        IRIS_LOG("OFDM-KISS: native confirmed (heard native from peer)");
                        if (gui_log_) gui_log_("OFDM-KISS: bidirectional confirmed");
                    }
                    if (!ofdm_kiss_tx_ && !ofdm_config_mismatch_) {
                        // Responder: heard first native frame from initiator → promote TX
                        ofdm_kiss_tx_ = true;
                        ax25_session_.set_native_active(true);
                        // Restore the responder's T1 backstop for the activation
                        // window (item 6) — see the OFDM promotion site above.
                        ax25_session_.start_t1_if_unacked();
                        IRIS_LOG("OFDM-KISS: TX promoted to native (responder)");
                        if (gui_log_) gui_log_("OFDM-KISS: native TX active (responder)");
                        // Speed level cache: start at cached level for this peer
                        {
                            int cached = gearshift_.load_cached_level(ax25_session_.remote_callsign());
                            if (cached > 0) {
                                gearshift_.force_level(cached);
                                IRIS_LOG("Gearshift: cached level %d for %s",
                                         cached, ax25_session_.remote_callsign().c_str());
                            }
                        }
                        // Migrate accumulated I-frames from AFSK to native queue.
                        // Drop oversized I-frames (queued before max_info reduction).
                        {
                            int migrated = 0, dropped = 0;
                            int ofdm_capacity =
                                ofdm_capacity_bytes_for_level(0, ofdm_config_);
                            std::queue<std::vector<uint8_t>> keep;
                            while (!ax25_tx_queue_.empty()) {
                                auto fr = std::move(ax25_tx_queue_.front());
                                ax25_tx_queue_.pop();
                                // §3.1 landmine fixed: the old byte-14 sniff
                                // ((fr[14] & 1) == 0) aliased EVERY via-carrying
                                // frame as an I-frame (see the predicate's doc,
                                // modem.h) and stole digipeat re-emits off AFSK.
                                if (migrate_to_native_eligible(fr)) {
                                    if ((int)fr.size() > ofdm_capacity) {
                                        dropped++;
                                        continue;
                                    }
                                    constexpr size_t TX_QUEUE_MAX = 32;
                                    if (tx_queue_.size() >= TX_QUEUE_MAX) {
                                        IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                                        tx_queue_.pop();
                                    }
                                    tx_queue_.push(TxFrame(std::move(fr)));  // I-frame: never tone-eligible
                                    migrated++;
                                } else {
                                    keep.push(std::move(fr));
                                }
                            }
                            ax25_tx_queue_ = std::move(keep);
                            if (migrated > 0 || dropped > 0)
                                IRIS_LOG("OFDM-KISS: migrated %d I-frames to native (responder, %d dropped oversized)",
                                         migrated, dropped);
                        }
                    }

                    // B2F proxy RX: handle B2F_DATA frames (unrolled plaintext from remote)
                    if (len >= 2 && data[0] == B2F_DATA_MAGIC &&
                        (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
                        ofdm_kiss_b2f_.is_initialized()) {
                        auto decoded = ofdm_kiss_rx_compressor_.decompress_record(
                            data + 1, len - 1);
                        if (decoded.status != v2::TransformStatus::Produced) {
                            transform_failed = true;
                            fail_ofdm_transform("B2F proxy receive decompression",
                                {std::vector<uint8_t>(data, data + len)});
                            return;
                        }
                        auto decomp = std::move(decoded.produced_bytes);

                        IRIS_LOG("[B2F-PROXY] RX: received B2F_DATA %zu bytes -> %d decompressed",
                                 len - 1, (int)decomp.size());

                        // Feed decompressed plaintext to filter_rx for rerolling
                        auto filtered = ofdm_kiss_b2f_.filter_rx_record(
                            decomp.data(), decomp.size());
                        if (filtered.status == v2::TransformStatus::Failed) {
                            transform_failed = true;
                            fail_ofdm_transform("B2F proxy receive reroll",
                                {std::vector<uint8_t>(data, data + len)});
                            return;
                        }
                        auto lzhuf_out = std::move(filtered.produced_bytes);
                        int lzhuf_len = static_cast<int>(lzhuf_out.size());

                        if (lzhuf_len > 0 && rx_callback_) {
                            // Construct I-frames with rerolled LZHUF and inject to Winlink
                            b2f_proxy_rx_active_ = true;
                            Ax25Address dst = ax25_make_addr(config_.callsign);
                            Ax25Address src = ax25_make_addr(ax25_session_.remote_callsign());
                            uint8_t ns = ax25_session_.vr();  // Continue from last known N(S)

                            IRIS_LOG("[B2F-PROXY] RX: rerolled %d bytes LZHUF, injecting I-frames (ns=%d)",
                                     lzhuf_len, ns);

                            // Split into MAX_INFO-sized I-frames
                            constexpr int MAX_INFO = 256;
                            int offset = 0;
                            while (offset < lzhuf_len) {
                                int chunk = std::min(MAX_INFO, lzhuf_len - offset);
                                auto iframe = ax25_build_i(dst, src, ns, 0, false,
                                    AX25_PID_NONE,
                                    lzhuf_out.data() + offset, chunk);
                                rx_callback_(iframe.data(), iframe.size());
                                ns = (ns + 1) & 0x07;
                                offset += chunk;
                            }
                        }

                        // Check if B2F handler exited payload transfer (all proposals done)
                        if (!ofdm_kiss_b2f_.is_rx_payload_active()) {
                            b2f_proxy_rx_active_ = false;
                            IRIS_LOG("[B2F-PROXY] RX: payload transfer complete");
                        }
                        return;
                    }

                    // B2F magic is UNCONDITIONAL for from_ofdm frames (item 4): a
                    // 0xCD lead byte lies outside the shifted AX.25 address range
                    // (address octets are ASCII<<1 with bit0==0 except the final
                    // octet), so it can NEVER be a genuine AX.25 frame. Reaching
                    // here with 0xCD means the B2F handler was not ready (caps not
                    // negotiated / handler uninitialized) — DROP loudly rather
                    // than fall through to dispatch_rx_frame and let a compressed
                    // block be parsed as AX.25 control (session poison).
                    if (len >= 1 && data[0] == B2F_DATA_MAGIC) {
                        transform_failed = true;
                        fail_ofdm_transform("B2F proxy receive admission",
                            {std::vector<uint8_t>(data, data + len)});
                        return;
                    }

                    // Filter out tune test frames — they are NOT AX.25 data.
                    // If dispatched, the garbage bytes get parsed as RNR/REJ and
                    // poison the AX.25 session state.
                    static const uint8_t tune_marker[] = "TUNE_TEST_FRAME";
                    if (len == sizeof(tune_marker) - 1 &&
                        memcmp(data, tune_marker, len) == 0) {
                        IRIS_LOG("[TUNE] Discarded decoded test frame (not dispatching to AX.25)");
                        return;
                    }

                    std::vector<uint8_t> ax25_frame(data, data + len);
                    if (packet_log_ && len >= 14)
                        packet_log_(false, "OFDM-KISS", describe_ax25(data, len));
                    dispatch_rx_frame(ax25_frame, false, true);  // from_ofdm=true
                    return;
                }
                auto arq_st = arq_.state();
                if (!loopback_mode_ &&
                    (arq_st == ArqState::CONNECTED ||
                     arq_st == ArqState::CONNECTING ||
                     arq_st == ArqState::LISTENING ||
                     arq_st == ArqState::HAILING ||
                     arq_st == ArqState::DISCONNECTING)) {
                    if (!arq_.on_frame_received(data, len)) {
                        // Not an ARQ frame — pass through to KISS/AGW
                        if (rx_callback_)
                            rx_callback_(data, len);
                    }
                } else {
                    if (rx_callback_)
                        rx_callback_(data, len);
                }
            };

            // OFDM-KISS batch decompression: if peer compressed, decompress first.
            if (ofdm_kiss_ && payload.size() >= 2 &&
                payload[0] == COMPRESSED_PAYLOAD_MAGIC &&
                (ofdm_kiss_peer_caps_ & CAP_COMPRESSION)) {
                auto decoded = ofdm_kiss_rx_compressor_.decompress_record(
                    payload.data() + 1, payload.size() - 1);
                if (decoded.status == v2::TransformStatus::Produced) {
                    IRIS_LOG("OFDM-KISS decompress: %zu -> %d bytes",
                             payload.size() - 1, (int)decoded.produced_bytes.size());
                    payload = std::move(decoded.produced_bytes);
                } else {
                    fail_ofdm_transform("OFDM receive decompression", {payload});
                    rx_overlap_buf_.clear();
                    pending_frame_start_ = -1;
                    pending_frame_timeout_ = 0;
                    return;
                }
            }

            // HARQ frame: strip retx descriptor, apply combining, deliver new data only
            if (decode_last_harq_flag() && arq_.negotiated(CAP_HARQ) && !payload.empty()) {
                HarqRetxDescriptor retx_desc;
                size_t consumed = deserialize_retx_descriptor(
                    payload.data(), payload.size(), retx_desc);
                if (consumed > 0 && consumed <= payload.size()) {
                    // TODO: apply retx bits to stored HARQ RX state for LLR combining
                    // For now, log and deliver the new data portion
                    IRIS_LOG("[HARQ] RX retx descriptor: seq=%d, %zu regions, %zu retx bytes",
                             retx_desc.original_seq, retx_desc.regions.size(),
                             retx_desc.retx_bits.size());
                    std::vector<uint8_t> new_data(payload.begin() + consumed, payload.end());
                    if (!new_data.empty())
                        deliver(new_data.data(), new_data.size());
                } else {
                    deliver(payload.data(), payload.size());
                }
            } else if (payload.size() >= 4 && payload[0] == MULTI_PAYLOAD_MAGIC) {
                // Multi-payload frame: [magic][epoch:1][2-byte LE len][data]...
                // #2: byte 1 is the sender's forward burst-epoch — remember it for
                // the reverse-ACK echo (MFSK suffix + S-frame tail).
                ofdm_kiss_rx_burst_epoch_ = payload[1];
                size_t pos = 2;  // skip [magic][epoch]
                int sub_count = 0;
                while (pos + 2 <= payload.size()) {
                    uint16_t sub_len = payload[pos] | ((uint16_t)payload[pos + 1] << 8);
                    pos += 2;
                    if (sub_len == 0 || pos + sub_len > payload.size())
                        break;
                    deliver(&payload[pos], sub_len);
                    pos += sub_len;
                    sub_count++;
                }
                IRIS_LOG("RX multi-payload: %d sub-frames (epoch=%d)",
                         sub_count, ofdm_kiss_rx_burst_epoch_);
            } else {
                deliver(payload.data(), payload.size());
            }

            // Drain consumed IQ pairs from overlap buffer
            size_t consumed_iq = decode_consumed_iq();
            if (consumed_iq == 0)
                consumed_iq = (size_t)(start + phy_config_.samples_per_symbol * 128);
            size_t consumed = consumed_iq * 2;  // IQ pairs → interleaved floats
            if (consumed > rx_overlap_buf_.size())
                consumed = rx_overlap_buf_.size();
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + consumed);
        } else if (decode_was_overflow()) {
            // Use exact need from decoder (it knows where overflow happened)
            size_t need_iq = decode_consumed_iq();
            if (need_iq == 0) {
                // Fallback: need at least enough for header
                need_iq = (size_t)(start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN +
                    IRIS_HEADER_LEN + 64) * phy_config_.samples_per_symbol +
                    RRC_SPAN * phy_config_.samples_per_symbol);
            }
            pending_frame_start_ = start;
            pending_need_floats_ = need_iq * 2;
            if (pending_need_floats_ > RX_OVERLAP_MAX)
                pending_need_floats_ = RX_OVERLAP_MAX;
            // Timeout: enough time for remaining samples to arrive, plus 50% margin.
            // Replaces fixed 10s timeout that caused 30-40s dead gaps on noisy FM
            // channels when false preamble detections blocked TX for the full duration.
            {
                size_t have = rx_overlap_buf_.size() / 2;  // IQ pairs already buffered
                size_t remaining = (pending_need_floats_ / 2 > have)
                    ? (pending_need_floats_ / 2 - have) : 0;
                int timeout_samples = (int)(remaining * 3 / 2);  // 1.5x margin
                int min_timeout = config_.sample_rate * 1;       // floor: 1s
                int max_timeout = config_.sample_rate * 7;       // cap: 7s
                if (timeout_samples < min_timeout) timeout_samples = min_timeout;
                if (timeout_samples > max_timeout) timeout_samples = max_timeout;
                pending_frame_timeout_ = timeout_samples;
            }
            IRIS_LOG("RX decode: need more data at offset %d (buf=%zu IQ, need ~%zu, timeout=1-7s)",
                     start, rx_overlap_buf_.size() / 2, pending_need_floats_ / 2);
        } else {
            crc_errors_++;
            gearshift_.report_failure();
            IRIS_LOG("RX decode FAIL at offset %d corr=%.3f (crc_errors=%d, gearshift->%d)",
                     start, det_corr, (int)crc_errors_, gearshift_.current_level());
            // NACK: in OFDM-KISS mode, send AX.25 REJ immediately so the peer
            // retransmits without waiting for T1 timeout (~30s).
            if (ofdm_kiss_ && ax25_session_.is_active()) {
                ax25_session_.request_retransmit();
            }
            // Native ARQ NACK: tell commander to retransmit immediately.
            {
                ArqState arq_st = arq_.state();
                if (!ofdm_kiss_ && arq_st != ArqState::IDLE) {
                    if (arq_.negotiated(CAP_HARQ)) {
                        // HARQ: per-block decode with LLR storage + extended NACK
                        auto harq_result = decode_native_frame_harq(
                            rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                            start, phy_config_);
                        arq_.on_decode_failed_harq(harq_result);
                    } else {
                        // Legacy: full Chase combining
                        arq_.on_decode_failed();
                    }
                }
            }
            // Skip past the estimated frame to avoid re-detecting its remnants.
            // Use the consumed_iq from the decoder if available, else estimate.
            size_t consumed_iq = decode_consumed_iq();
            size_t skip;
            if (consumed_iq > 0) {
                skip = consumed_iq * 2;
            } else {
                // Estimate: skip past preamble + sync + header + minimum payload
                skip = (size_t)(start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN +
                    IRIS_HEADER_LEN + 64) * phy_config_.samples_per_symbol) * 2;
            }
            if (skip > rx_overlap_buf_.size()) skip = rx_overlap_buf_.size();
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + skip);
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_tx(float* tx_audio, int frame_count) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Draining: TX buffer exhausted, waiting for audio pipeline to flush
    if (tx_draining_) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        bool done = tx_drain_done_ ? tx_drain_done_() : true;
        if (done) {
            tx_draining_ = false;
            // ptt_off sets holdoff based on TX mode (AFSK vs native)
            ptt_off();
            rx_muted_ = true;
            state_ = ModemState::IDLE;

            // Notify probe controller when probe tone TX finishes draining.
            // CMD (SENT_TONES): starts capture for RSP's response tones.
            // RSP (SENDING_PROBE): analyzes captured CMD tones, sends RESULT.
            if (ofdm_kiss_probing_ &&
                (probe_.state() == ProbeState::SENT_TONES ||
                 probe_.state() == ProbeState::SENDING_PROBE)) {
                probe_.on_tx_complete();
            }
        }
        return;
    }

    if (tx_pos_ < tx_buffer_.size()) {
        // Fresh buffer at pos 0: turn PTT on before sending.
        // Covers externally-filled buffers (tune test frames) that bypass
        // the normal queue→build→ptt_on path below.
        if (tx_pos_ == 0) ptt_on();

        size_t remaining = tx_buffer_.size() - tx_pos_;
        size_t to_copy = std::min((size_t)frame_count, remaining);
        std::memcpy(tx_audio, tx_buffer_.data() + tx_pos_, to_copy * sizeof(float));
        tx_pos_ += to_copy;

        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        // Apply simulated channel effects to TX audio
        if (sim_bp_enabled_) {
            for (int i = 0; i < frame_count; i++) {
                float s = tx_audio[i];
                for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                tx_audio[i] = s;
            }
        }
        if (sim_deemph_enabled_) {
            for (int i = 0; i < frame_count; i++)
                tx_audio[i] = sim_deemph_.process(tx_audio[i]);
        }

        if (tx_pos_ >= tx_buffer_.size()) {
            // Self-hear guard: block OFDM RX to discard self-heard frames.
            // FD-ZC=0.75 for self-hear (passes 0.30 threshold), so guard IS needed
            // for single-carrier (stale audio in rx_overlap_buf_).
            //
            // OFDM-KISS: guard=0. The 200ms rx_mute clears self-heard audio
            // (ALSA round-trip <100ms). Any residual self-hear tail that triggers
            // the SC/ZC correlator after mute expires is rejected by the CRC+FD-ZC
            // phantom gate in ofdm_demod.cc (FD-ZC 0.97 < 0.99 gate + CRC fail).
            // A 167ms guard on top of 200ms mute creates a 367ms blackout that
            // clips RSP's RR preamble arriving ~250ms after TX ends.
            //
            // Event-driven MAC: csma_holdoff is a ceiling, cancelled on RR
            // reception in dispatch_rx_frame(). CMD doesn't wait the full
            // holdoff — it TXs immediately after receiving RSP's response.
            if (state_ == ModemState::TX_NATIVE) {
                // OFDM-KISS: guard=0. CRC+FD-ZC phantom gate handles self-hear.
                // Single-carrier: 1500ms guard (no phantom gate, uses time-based).
                native_selfhear_guard_ = ofdm_kiss_tx_
                    ? 0                                // CRC+FD-ZC phantom gate handles self-hear
                    : config_.sample_rate * 3 / 2;     // 1500ms (single-carrier: full guard)
                // csma_holdoff ceiling: cancelled on RR reception (event-driven).
                // Only matters if RR is lost — T1 fires after ceiling expires.
                // NOTE: 0.8s ceiling was tested and caused CMD to collide with
                // RSP's RR response (25-87 bps). 1.5s is safe — the 0.5s gap
                // between holdoff and T1=2.0s was already tightened by T1=1.0s.
                {
                    int native_listen = (ax25_session_.is_active() && ax25_session_.we_initiated())
                        ? config_.sample_rate * 3 / 2  // initiator: 1.5s ceiling (cancelled on RR)
                        : config_.sample_rate / 2;     // responder: 0.5s (respond ASAP)
                    csma_holdoff_ = std::max(csma_holdoff_.load(), native_listen);
                }
                if (ofdm_kiss_tx_) {
                    IRIS_LOG("[MAC-DIAG] TX-END: guard=%dms mute=200ms csma=%dms expect_ack=%d",
                             native_selfhear_guard_.load() * 1000 / config_.sample_rate,
                             csma_holdoff_.load() * 1000 / config_.sample_rate,
                             ofdm_expect_ack_ ? 1 : 0);
                }
            }
            tx_buffer_.clear();
            tx_pos_ = 0;
            // Mark current pipeline position, then wait for render to catch up
            if (tx_drain_mark_) tx_drain_mark_();
            tx_draining_ = true;
        }
        return;
    }

    if (state_ == ModemState::CALIBRATING && cal_state_ == CalState::TX_TONE) {
        generate_cal_tone(tx_audio, frame_count);
        return;
    }

    // DCD (Data Carrier Detect): defer TX while channel is busy.
    // Tone-based DCD is disabled — on real radios the AFSK correlator can't
    // distinguish signal (~30) from noise (~22), so any threshold either
    // never fires or permanently locks TX.  AFSK turn-taking relies on
    // post-TX holdoff (role-asymmetric) + post-RX burst guard instead.
    // Energy-based DCD is still used in native/OFDM-KISS mode.
    bool dcd_busy = false;
    if (!loopback_mode_ && !ofdm_kiss_probing_ &&
        config_.dcd_threshold > 0 && (native_mode_ || ofdm_kiss_)) {
        if (dcd_inverted_) {
            dcd_busy = rx_raw_rms_ < dcd_baseline_rms_ * 0.5f;
        } else {
            dcd_busy = rx_raw_rms_ > config_.dcd_threshold;
        }
    }
    if (dcd_busy) {
        int holdoff_samples = config_.dcd_holdoff_ms * config_.sample_rate / 1000;
        dcd_holdoff_ = holdoff_samples;
        csma_slot_timer_ = 0;  // reset p-persist slot on new carrier
    }
    if (dcd_holdoff_ > 0) {
        dcd_holdoff_ -= frame_count;
        if (dcd_holdoff_ < 0) dcd_holdoff_ = 0;
    }

    // CSMA guard: wait after last frame decode before starting TX
    // Counted in samples for deterministic timing regardless of audio buffer size.
    if (csma_holdoff_ > 0) {
        csma_holdoff_ -= frame_count;
        if (csma_holdoff_ < 0) csma_holdoff_ = 0;
    }

    // Consolidated channel busy: pause AX.25 T1/T3 whenever TX is blocked.
    // On half-duplex radio we can't expect an acknowledgment while we can't
    // transmit (Direwolf pattern from ax25_link.c:7035-7084).
    // Covers: DCD active, DCD holdoff, CSMA holdoff, PTT (in ptt_on/off),
    // probing, and pending native frame (waiting for more data after preamble).
    bool native_frame_pending = (pending_frame_start_ >= 0 && pending_frame_timeout_ > 0);
    // Channel busy pauses AX.25 T1/T3 timers. Only pause when someone is
    // actually on the air (DCD = peer transmitting, PTT handled separately in
    // ptt_on/ptt_off). CSMA holdoff and probe countdown are TX deferrals —
    // T1 must keep running during those because the peer may be responding.
    bool ofdm_frame_pending = ofdm_sync_cached_ && ofdm_kiss_tx_;
    bool channel_active = dcd_busy || native_frame_pending || ofdm_frame_pending;
    ax25_session_.set_channel_busy(channel_active);

    if (dcd_busy || dcd_holdoff_ > 0) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    if (csma_holdoff_ > 0) {
        // Burst-MAC in-window continuation (leg 2, QUARANTINED default-OFF via
        // burst_fill_ — see burst_fill_continue()). The DCD gate above already
        // blocked a busy channel; if the mechanism is enabled and the K=7 window
        // still has room + data is queued, fall through and send the next burst
        // instead of waiting out the ACK-clocked stop-and-wait timer. Off by
        // default → production keeps the safe stop-and-wait (no collision risk).
        bool inwindow = burst_fill_continue(
            burst_fill_, ofdm_kiss_tx_, ax25_session_.is_active(),
            ax25_session_.we_initiated(), !tx_queue_.empty(),
            ax25_session_.window_used(),
            ax25_session_.window_k());  // live window (mod-8 K=7, or wide modulo-128)
        if (!inwindow) {
            std::memset(tx_audio, 0, frame_count * sizeof(float));
            return;
        }
    }
    // Suppress TX while waiting for a native frame to arrive.
    // We detected a valid preamble and need more audio data to decode
    // the full frame.  Keying PTT would destroy the incoming frame on
    // a half-duplex radio.
    if (native_frame_pending) {
        pending_frame_timeout_ -= frame_count;
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    // OFDM frame-in-progress guard: detected OFDM preamble but waiting for
    // enough samples to decode the full frame. TX now would destroy the
    // incoming data on half-duplex.
    if (ofdm_sync_cached_ && ofdm_kiss_tx_) {
        // Don't block S-frame (ACK) TX on speculative sync detection.
        // False SC triggers on post-burst noise can block RSP's MFSK ACK
        // for up to 6s (redetect limit), causing T1 timeouts on CMD.
        // S-frames are short (235ms MFSK) and critical for throughput.
        // Reconcile the sibling loose &0x01 mask (same bug class): allow TX during
        // speculative sync only for our own autonomous RR ACK (provenance flag),
        // not for any odd-byte14 frame.  Fail-safe — this only PERMITS TX.
        bool tx_is_sframe = !tx_queue_.empty() && tx_queue_.front().tone_ack_eligible;
        if (!tx_is_sframe) {
            std::memset(tx_audio, 0, frame_count * sizeof(float));
            return;
        }
        // S-frame: allow TX, clear stale sync
        ofdm_sync_cached_ = false;
    }
    // Suppress non-probe TX during probe — stray AFSK/native frames
    // would key PTT and prevent hearing the peer's tones.  But let pending
    // probe audio through (it was just queued by start_*() and needs to TX).
    // Also blocks when probe is DONE but tick() hasn't processed completion:
    // prevents accumulated I-frames from blasting out as AFSK before native
    // mode is activated (they'd bury the probe result in a long burst).
    //
    // IMPORTANT: When probe audio IS pending, we skip the AFSK queue drain
    // below (probe_suppress_afsk) so only probe tones go out. Without this,
    // accumulated I-frames (connection SID exchange, etc.) batch into a 30s
    // TX burst WITH probe tones appended at the end — keeping PTT keyed so
    // long that the peer's response tones arrive and vanish before we start
    // listening. The AFSK frames stay in ax25_tx_queue_ and drain naturally
    // after probe completes (gate opens when ofdm_kiss_probing_ = false).
    bool probe_suppress_afsk = false;
    if (ofdm_kiss_probing_ && probe_.state() != ProbeState::IDLE) {
        if (probe_audio_pending_.empty()) {
            if (probe_.state() == ProbeState::WAITING_RESULT && !ax25_tx_queue_.empty()) {
                // Tone exchange complete, waiting for AFSK RESULT exchange.
                // Let ax25_tx_queue_ drain so RESULT frames reach the peer.
                // Without this, both sides deadlock: RESULT frames are queued
                // but probe_suppress_afsk blocks them, so neither side ever
                // receives the other's result and both time out.
                probe_suppress_afsk = false;
            } else {
                // No probe audio and nothing critical to send — suppress TX.
                // This covers: SENT_TONES (CMD waiting for drain), LISTENING_PROBE
                // (capturing), SENDING_RESULT (RSP turnaround gap), SENDING_PROBE
                // (RSP tones queued but not yet in tx_buffer_).
                std::memset(tx_audio, 0, frame_count * sizeof(float));
                return;
            }
        } else {
            // Probe audio pending — skip AFSK queue drain, only send probe tones
            probe_suppress_afsk = true;
        }
    }

    // p-persistent CSMA (AX.25 2.2 Section 6.4.2)
    // Channel is clear (DCD off, holdoffs expired). Use slotted random access
    // to avoid collisions when both sides detect channel-clear simultaneously.
    // Each slottime period: if random < persist, transmit; else wait another slot.
    // Bypass: In a connected AX.25 session (point-to-point), both sides already
    // alternate TX/RX.  DCD + csma_holdoff_ provide sufficient collision avoidance.
    // Random CSMA backoff just adds latency and increases T1 timeouts.
    bool csma_bypass = ax25_session_.is_active();
    if (config_.persist < 255 && config_.slottime_ms > 0 && !loopback_mode_ && !csma_bypass) {
        if (csma_slot_timer_ > 0) {
            csma_slot_timer_ -= frame_count;
            if (csma_slot_timer_ > 0) {
                std::memset(tx_audio, 0, frame_count * sizeof(float));
                return;
            }
        }
        // Slot expired or first entry — roll the dice
        static thread_local std::minstd_rand csma_rng(std::random_device{}());
        int r = csma_rng() % 256;
        if (r >= config_.persist) {
            // Lost the coin toss — wait another slottime
            csma_slot_timer_ = config_.slottime_ms * config_.sample_rate / 1000;
            std::memset(tx_audio, 0, frame_count * sizeof(float));
            return;
        }
        // Won — fall through to transmit. Reset for next time.
        csma_slot_timer_ = 0;
    }

    // Deferred window re-fragment from an anchor-futility demote (see
    // demote_tx_anchor / ofdm_refrag_pending_): this is the clean point —
    // nothing popped, tx_buffer_ drained, PTT off — so the recover + re-slice
    // can roll V(S) back to V(A) without a stale slicing mid-flight.
    consume_pending_window_refrag();

    {
        // Drain forced-AX.25 queue first (probes, etc.),
        // then regular tx_queue_ (native or AX.25 depending on mode).
        bool have_frame = false;
        if (!ax25_tx_queue_.empty() && !probe_suppress_afsk) {
            // Batch all forced-AX.25 frames into one TX burst (same as tx_queue_)
            state_ = ModemState::TX_AX25;
            int preamble_flags = std::max(8, config_.ptt_pre_delay_ms * config_.ax25_baud / 8000);
            std::vector<uint8_t> raw_bits;  // pre-NRZI

            bool first = true;
            while (!ax25_tx_queue_.empty()) {
                auto frame_data = std::move(ax25_tx_queue_.front());
                ax25_tx_queue_.pop();
                IRIS_LOG("TX frame %zu bytes (forced AX.25%s%s)", frame_data.size(),
                         config_.fx25_mode > 0 ? " FX.25" : "",
                         first ? "" : " batched");
                if (packet_log_ && frame_data.size() >= 14) {
                    std::string proto = config_.fx25_mode > 0
                        ? "FX.25-" + std::to_string(config_.fx25_mode)
                        : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
                    packet_log_(true, proto, describe_ax25(frame_data.data(), frame_data.size()));
                }
                int flags = first ? preamble_flags : 2;
                if (config_.fx25_mode > 0) {
                    if (!fx25_encode_raw(raw_bits, frame_data.data(), frame_data.size(),
                                        config_.fx25_mode, flags))
                        hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                } else {
                    hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                }
                first = false;
            }

            if (!raw_bits.empty()) {
                if (config_.ax25_baud == 9600)
                    g3ruh_tx_scrambler_.scramble(raw_bits);
                auto nrzi_bits = nrzi_encode(raw_bits);
                if (config_.ax25_baud == 9600)
                    tx_buffer_ = gfsk_mod_.modulate(nrzi_bits);
                else
                    tx_buffer_ = afsk_mod_.modulate(nrzi_bits);
            }
            have_frame = true;
        } else if (!tx_queue_.empty() && !ofdm_kiss_probing_ &&
                   (tune_state_ == TuneState::IDLE || tune_state_ == TuneState::DONE) &&
                   tune_post_holdoff_ <= 0) {
            // Native hail: use native PHY for ARQ hail/connect frames
            bool native_hail_active = false;
            if (config_.native_hail && !native_mode_) {
                auto arq_st = arq_.state();
                // Only active hail/connect phases — LISTENING is passive (responder
                // only replies via native after receiving a native hail frame).
                native_hail_active = (arq_st == ArqState::HAILING ||
                                       arq_st == ArqState::CONNECTING);
            }
            if ((native_mode_ && native_tx_ready_) || native_hail_active || ofdm_kiss_tx_) {
                // Batch multiple queued frames into one multi-payload frame.
                // Limit total payload based on speed level to cap air time.
                // Adaptive: grows from 6s toward 12s on success, halves on REJ.
                // PHY bps: use OFDM tone map throughput when OFDM active,
                // otherwise legacy single-carrier throughput.
                int phy_bps;
                if (ofdm_phy_active_ && !native_hail_active) {
                    // OFDM: throughput from tone map (accounts for carriers, FEC, CP)
                    // Use streaming estimate (50 CW) for batch sizing
                    phy_bps = (int)tone_map_throughput(ofdm_tone_map_, ofdm_config_);
                } else {
                    int level = gearshift_.current_level();
                    phy_bps = net_throughput(level, phy_config_.baud_rate);
                }
                // Cap payload at actual LDPC block capacity for current speed level.
                // LDPC k bits = data bits per block.  Minus 6 bytes overhead (2 len + 4 CRC).
                // Multi-codeword: pack multiple LDPC blocks per frame to amortize
                // Multi-codeword frames: amortize preamble + pilot overhead.
                // Each LDPC block is independently decodable with per-block SACK.
                // O0-O1: n_cw=1 (keep frames short for weak-signal reliability).
                // O2-O5: n_cw=4. O6-O7 (64QAM): n_cw=8 (best throughput ceiling).
                // O8-O9 (256QAM): n_cw=4 — 256QAM can't sustain ncw=8 partial fill through FM.
                // In practice O7 ncw=8 (~6 kbps) beats O8 ncw=4 (~4.3 kbps).
                // A2: size the batch at the level the OFDM frame will actually be
                // MODULATED at — not the stale ofdm_speed_level_ left over from the
                // previous TX iteration / RX-adopt. The classification-independent
                // caps below fully determine a DATA batch's level; a control batch
                // is only ever forced LOWER (O0) or to the tiny O1 floor and is
                // 1-CW-collapsed, so sizing at the data level never under-sizes a
                // real payload. The FINAL ncw + fec + tone map are re-derived from
                // the single latched level in resolve_ofdm_tx_level() below (which
                // owns BOTH capacities coherently). Retires the modem.h ncw/mod [?].
                int ofdm_cw_per_frame = 1;  // provisional; final set by resolve below
                size_t max_payload;
                if (ofdm_phy_active_ && !native_hail_active) {
                    int peer_snr_level = (peer_snr_db_ > 0)
                        ? ofdm_snr_to_speed_level(peer_snr_db_) : -1;
                    int no_ack_limit = ofdm_kiss_tx_ ? 2 : 6;
                    bool no_ack_hit = (tx_no_ack_count_ + 1 >= no_ack_limit);
                    int sizing_level = ofdm_apply_tx_level_caps(
                        gearshift_.current_ofdm_level(), ofdm_kiss_tx_, no_ack_hit,
                        tx_acked_level_, tx_proposed_level_, peer_snr_level,
                        OFDM_LEVEL_LEAP_MAX);
                    ofdm_cw_per_frame = ofdm_cw_for_level(sizing_level);
                    max_payload = static_cast<size_t>(
                        ofdm_capacity_bytes_for_level(sizing_level, ofdm_config_));
                } else {
                    max_payload = (size_t)NATIVE_MAX_PAYLOAD;
                }
                size_t max_batch = std::min(max_payload,
                                             (size_t)(phy_bps * batch_airtime_s_ / 8));
                // Floor ensures at least one frame fits, but never exceed LDPC capacity
                size_t batch_floor = std::min((size_t)200, max_payload);
                if (max_batch < batch_floor) max_batch = batch_floor;

                std::vector<std::vector<uint8_t>> batch;
                // Originals of the popped frames (provenance preserved) so a build
                // reject re-queues them via requeue_rejected_frames() instead of
                // silently destroying them (leg 2 co-dependency safety net).
                std::vector<TxFrame> taken;
                // Provenance AND: the batch is MFSK-tone-ACK-eligible iff EVERY
                // frame taken was tagged tone_ack_eligible (an autonomous RR).
                // Any U-frame / B2F chunk / client frame in the batch clears this.
                bool batch_all_tone_ack_eligible = true;
                // #2 epoch: does this burst carry any I-frame (forward DATA)?  The
                // burst-epoch counter advances ONLY on data bursts, so a control-only
                // burst (e.g. a T1 RR poll) emitted between a data burst and its ACK
                // does NOT desync the sender's epoch expectation.
                bool batch_has_iframe = false;
                size_t total_bytes = 0;
                const int abs_cap = ofdm_abs_max_capacity(ofdm_config_);
                // [SLOT-TELEM] backlog snapshot at burst-build time (the probe's
                // host-window witness, logged after the slot is assembled below):
                // if the queue runs DRY here every slot, the HOST window (K x
                // paclen) is the coalescing gate; if it stays deep while the
                // slot ends at the frame/airtime bound, the limit is
                // modem-internal and needs no host-interface change first.
                size_t telem_qframes_before = tx_queue_.size();
                size_t telem_qbytes_before = 0;
                {
                    std::queue<TxFrame> scan = tx_queue_;
                    while (!scan.empty()) {
                        telem_qbytes_before += scan.front().data.size();
                        scan.pop();
                    }
                }
                while (!tx_queue_.empty()) {
                    auto& front = tx_queue_.front();
                    // Drop-drain ONLY genuinely un-sendable frames (bigger than ANY
                    // level's capacity — a KISS client with a mis-set MAX_INFO/paclen).
                    // A frame that merely exceeds the CURRENT level's capacity is kept
                    // and reject-requeued below, so a transient level fall (dynamic
                    // MAX_INFO) never destroys a recoverable frame.
                    if (ofdm_phy_active_ && front.data.size() > (size_t)abs_cap + 5) {
                        IRIS_LOG("[TX-OFDM] DROP oversized frame: %zu bytes > %d abs OFDM cap",
                                 front.data.size(), abs_cap);
                        tx_queue_.pop();
                        continue;
                    }
                    // First sub-frame carries the shared wrapper header (magic+epoch)
                    // AND its own 2-byte length; subsequent frames carry only a length.
                    // Must match the wrapper actually built below ([magic][epoch]
                    // [len,data]...) so total_bytes == frame_data.size() exactly and the
                    // oversized guard can never fire on a batch the packer sized as fitting.
                    // The prior "3" omitted the shared epoch byte — the same off-by-one as
                    // ofdm_max_info_for_level (DATALINK_TAX_DIAGNOSIS.md, Fix A).
                    size_t overhead = batch.empty()
                        ? (size_t)(OFDM_KISS_WRAPPER_HDR + OFDM_KISS_SUBFRAME_HDR)  // magic+epoch + len = 4
                        : (size_t)OFDM_KISS_SUBFRAME_HDR;                           // len = 2
                    // Always take the first frame (even if oversized) to avoid empty batches
                    if (!batch.empty() && total_bytes + overhead + front.data.size() > max_batch)
                        break;
                    // #1 ROOT FIX: re-stamp N(R) to the LIVE session V(R) at the air
                    // chokepoint (in place, before both the build copy and the
                    // reject-requeue original) so no S-/I-frame flies with a stale
                    // N(R). Native-only (ofdm_kiss_tx_): pure-KISS/AFSK shadow keeps
                    // its verbatim forwarding. See restamp_live_nr() + invariant I-D.
                    if (ofdm_kiss_tx_)
                        restamp_live_nr(front.data, ax25_session_.current_vr(),
                                        ax25_session_.extended());
                    if (front.data.size() >= 15 && (front.data[14] & 0x01) == 0)
                        batch_has_iframe = true;            // I-frame (data) present
                    total_bytes += overhead + front.data.size();
                    batch_all_tone_ack_eligible = batch_all_tone_ack_eligible && front.tone_ack_eligible;
                    batch.push_back(front.data);            // copy for build
                    taken.push_back(std::move(front));      // original for reject-requeue
                    tx_queue_.pop();
                }

                // Detect if batch contains only ARQ control frames (non-DATA).
                // Control frames at robust rate (~20ms extra) prevents cascading
                // retransmit timeouts when channel has degraded.
                bool batch_is_control = false;
                if (!ofdm_kiss_ && arq_.state() != ArqState::IDLE) {
                    batch_is_control = true;
                    for (auto& sub : batch) {
                        if (!sub.empty() && sub[0] == (uint8_t)ArqType::DATA) {
                            batch_is_control = false;
                            break;
                        }
                    }
                }

                // OFDM-KISS: detect if batch is all AX.25 S-frames (RR/REJ/RNR).
                // S-frames are tiny (~15 bytes) and don't need multi-CW OFDM frames.
                // Using 1 CW saves ~750ms/ACK at O3 (15 vs 48 symbols).
                bool kiss_control_batch = false;
                if (ofdm_kiss_ && !batch.empty()) {
                    // (1) 1-CW SIZE COLLAPSE — CAPACITY-based, content-agnostic —
                    // now lives in resolve_ofdm_tx_level() so it collapses the
                    // FINAL level's ncw (A2), not the stale one. It cannot swallow
                    // or drop anything (pure sizing) and still applies to REJ/RNR/UA
                    // and small B2F/client batches so CMD's 1-CW expect-ack gate
                    // (modem.cc:1726) decodes the RSP's immediate NACK REJ inside
                    // the forward-loss recovery window.

                    // (2) MFSK tone REPLACEMENT trigger — PROVENANCE gate (NOT a
                    // content test). Only the modem's own autonomous RR ACKs
                    // (tagged tone_ack_eligible at the session producer,
                    // modem.cc:590) may be replaced by the MFSK tone-ACK below
                    // and forced to the O1 robust floor. The old `(sub[14] &
                    // 0x01)` content test mis-classified every U-frame
                    // (UA/DISC/DM/SABM/FRMR) and any B2F_DATA proxy chunk whose
                    // byte-14 aliased an odd control byte as an "S-frame",
                    // letting the tone-ACK swallow them with NO retransmit path
                    // (silent data loss — ship-blocker).
                    kiss_control_batch = batch_all_tone_ack_eligible;
                    if (kiss_control_batch) {
                        IRIS_LOG("[MAC-DIAG] S-frame TX: %s batch=%zu csma_was=%dms",
                                 ax25_session_.we_initiated() ? "CMD" : "RSP",
                                 batch.size(),
                                 csma_holdoff_.load() * 1000 / config_.sample_rate);
                    }
                }

                // Log OFDM-KISS frames before payload build (batch elements get moved)
                if (ofdm_kiss_ && packet_log_) {
                    for (auto& sub : batch) {
                        if (sub.size() >= 14)
                            packet_log_(true, "OFDM-KISS", describe_ax25(sub.data(), sub.size()));
                    }
                }

                // #2 burst-epoch: advance the forward counter ONLY on a DATA burst
                // (see batch_has_iframe) so a control-only poll never desyncs it.  On
                // the native path the wrapper ALWAYS carries the epoch (single frames
                // too — the fast path is legacy/AFSK only) so the peer can echo it.
                if (ofdm_kiss_tx_ && batch_has_iframe)
                    tx_burst_epoch_ = next_burst_epoch(tx_burst_epoch_);

                // Build payload: single frame or multi-payload.  Iris-owned native
                // wrapper: [magic][epoch:1][2-byte LE len][data]...  Legacy/AFSK keeps
                // the bare single-frame fast path (no epoch, no wrapper).
                std::vector<uint8_t> frame_data;
                bool use_wrapper = ofdm_kiss_tx_ ||
                                   batch.size() > 1 ||
                                   (batch.size() == 1 && !batch[0].empty() &&
                                    batch[0][0] == MULTI_PAYLOAD_MAGIC);
                if (!use_wrapper) {
                    // Single frame, no wrapping needed (avoid overhead) — legacy/AFSK.
                    frame_data = std::move(batch[0]);
                } else {
                    // Multi-payload wrapper: [magic][epoch][2-byte LE len][data]...
                    frame_data.reserve(total_bytes + 1);
                    frame_data.push_back(MULTI_PAYLOAD_MAGIC);
                    frame_data.push_back(tx_burst_epoch_);
                    for (auto& sub : batch) {
                        uint16_t len = (uint16_t)sub.size();
                        frame_data.push_back(len & 0xFF);
                        frame_data.push_back((len >> 8) & 0xFF);
                        frame_data.insert(frame_data.end(), sub.begin(), sub.end());
                    }
                }

                IRIS_LOG("TX frame %zu bytes (%zu sub-frames, %s)",
                         frame_data.size(), batch.size(),
                         ofdm_kiss_ ? "OFDM-KISS" : "native");

                // Start T1 at TX time for OFDM-KISS frames.
                // notify_outgoing() may have been called before native_active_
                // was set (KISS I-frames queued during AFSK/probe phase).
                if (ofdm_kiss_tx_) {
                    ax25_session_.start_t1_if_unacked();
                }

                // OFDM-KISS batch compression: compress the assembled payload.
                // Prepend COMPRESSED_PAYLOAD_MAGIC so RX knows to decompress.
                // Skip if payload is B2F_DATA (already compressed internally).
                if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) &&
                    frame_data.size() > 20 &&
                    frame_data[0] != B2F_DATA_MAGIC) {  // Don't double-compress B2F_DATA
                    auto encoded = ofdm_kiss_tx_compressor_.compress_record(
                        frame_data.data(), frame_data.size());
                    if (encoded.status != v2::TransformStatus::Produced) {
                        fail_ofdm_transform("OFDM transmit compression", batch);
                        std::memset(tx_audio, 0, frame_count * sizeof(float));
                        return;
                    }
                    const int comp_len = static_cast<int>(encoded.produced_bytes.size());
                    if (comp_len + 1 < (int)frame_data.size()) {
                        IRIS_LOG("OFDM-KISS compress: %zu -> %d bytes (%.0f%%)",
                                 frame_data.size(), comp_len + 1,
                                 100.0 * (comp_len + 1) / frame_data.size());
                        frame_data.clear();
                        frame_data.reserve(1 + comp_len);
                        frame_data.push_back(COMPRESSED_PAYLOAD_MAGIC);
                        frame_data.insert(frame_data.end(), encoded.produced_bytes.begin(),
                                          encoded.produced_bytes.end());
                    }
                }

                state_ = ModemState::TX_NATIVE;

                // Raise T1 floor to account for this frame's airtime.
                float frame_airtime_s = 0;

                // MFSK tone ACK: replace OFDM S-frames with short tone burst.
                // 234ms vs 544ms, non-coherent detection — immune to timing/sync issues.
                bool mfsk_tx_done = false;
                // MFSK ACK TX: only RSP (responder) sends MFSK. CMD (initiator)
                // always sends OFDM — both data and T1 polls — so RSP's existing
                // OFDM demod can decode them without needing MFSK detection.
                // This avoids the MFSK/OFDM coexistence problem on the RSP RX path.
                if (kiss_control_batch && mfsk_ack_.is_initialized() && !batch.empty()
                    && !ax25_session_.we_initiated()) {
                    // Use Iris session's V(R) directly — not the batch S-frame N(R).
                    // The batch may contain RRs from BOTH Iris T2 (correct V(R))
                    // and the KISS pump (pump's own V(R), which wraps independently
                    // and can produce stale/wrong N(R) values). Iris's session V(R)
                    // is authoritative — it tracks every decoded I-frame.
                    int n_r = ax25_session_.current_vr();
                    // P/F: check if any frame in batch has P/F set (enquiry response)
                    int pf_bit = 0;
                    for (auto& f : batch) {
                        if (f.size() >= 15 && (f[14] & 0x10))
                            pf_bit = 1;
                    }
                    // Debug: log batch contents vs session V(R)
                    for (size_t i = 0; i < batch.size(); i++) {
                        if (batch[i].size() >= 15) {
                            uint8_t c = batch[i][14];
                            IRIS_LOG("[MAC-DIAG] MFSK batch[%zu]: ctrl=0x%02X N(R)=%d PF=%d (session V(R)=%d)",
                                     i, c, (c >> 5) & 7, (c >> 4) & 1, n_r);
                        }
                    }
                    // Receiver-drives-rate (SUPER-ACK): B (the data RECEIVER)
                    // proposes the ABSOLUTE forward O-level its own decode-margin
                    // climb has validated it can receive. This is the ONLY signal
                    // that lets A climb on a unidirectional link (B sends A no OFDM
                    // frames, so A's own RX-driven gearshift stays frozen at O0).
                    // Cap the proposal at the confirmed-decodable ceiling: the
                    // margin climb over-proposes O3+, which the RX cannot yet decode
                    // (O2+ LLR-crush bug) and would stall ARQ. Raise when fixed.
                    int proposed_level = std::min(gearshift_.current_ofdm_level(),
                                                  max_proposable_ofdm_level_);
                    // climbgate experiment lock: pin the RSP forward proposal so the
                    // sender runs at exactly O<force> (forced-level decode datapoint).
                    if (force_ofdm_level_ >= 0)
                        proposed_level = force_ofdm_level_;
                    // Coherent RX adoption (Mercury rank 7/9): remember the level we
                    // are driving A toward so B's own RX path follows its proposal.
                    // The frame-length gate widens to admit A's higher/longer frame
                    // (no truncation) and blind-detect tries this level FIRST, so the
                    // O3/ncw4 frame is demapped at the right config deterministically
                    // instead of being rediscovered by a slow sweep (or truncated and
                    // lost). Still CRC-32 self-rejecting — a not-yet-adopted target
                    // just falls through to the blind sweep, never a mis-demap.
                    ofdm_kiss_rx_proposed_level_ = proposed_level;
                    // #2: echo the last forward burst-epoch we decoded so the DATA
                    // sender can bind THIS reverse ACK to the burst it acks (a stale
                    // buffered tone carries the OLD epoch -> the sender holds V(A)).
                    tx_buffer_ = mfsk_ack_.generate(n_r, pf_bit, proposed_level,
                                                    ofdm_kiss_rx_burst_epoch_);
                    IRIS_LOG("[TX-MFSK] ACK burst: N(R)=%d PF=%d L_prop=O%d, %zu samples (%.0fms)",
                             n_r, pf_bit, proposed_level, tx_buffer_.size(),
                             (float)tx_buffer_.size() / config_.sample_rate * 1000.0f);
                    have_frame = true;
                    frame_airtime_s = (float)tx_buffer_.size() / (float)config_.sample_rate;
                    mfsk_tx_done = true;
                }

                std::vector<float> iq;  // Interleaved I/Q output (native PHY)
                std::vector<std::complex<float>> ofdm_iq;  // Complex OFDM samples (real-valued after Hermitian sym)

                if (mfsk_tx_done) {
                    // MFSK ACK already built tx_buffer_ above
                } else if (ofdm_phy_active_ && !native_hail_active) {
                    // ============ OFDM PHY TX path ============
                    // A2: single-source the TX speed level + ncw + tone map. This
                    // re-latches the level from the gearshift, applies the caps in
                    // canonical order (control force → no-ACK downshift → peer-SNR →
                    // ceiling), stamps the A3 per-N(S) ring, builds the tone map, and
                    // derives ncw from that ONE final level (with the 1-CW collapse).
                    ofdm_cw_per_frame = resolve_ofdm_tx_level(kiss_control_batch,
                                                              batch_is_control, total_bytes);
                    IRIS_LOG("[TX-OFDM] speed=O%d %s fec=%d/%d, %zu bytes, %d cw/frame, %d carriers, %d bits/sym",
                             ofdm_speed_level_,
                             ofdm_tone_map_.tone_map_id == 0 ? "waterfill" : "uniform",
                             OFDM_SPEED_LEVELS[ofdm_speed_level_].fec_rate_num,
                             OFDM_SPEED_LEVELS[ofdm_speed_level_].fec_rate_den,
                             frame_data.size(), ofdm_cw_per_frame,
                             ofdm_config_.n_data_carriers,
                             ofdm_tone_map_.total_bits_per_symbol);

                    // Oversized frame guard: if payload exceeds single-block capacity,
                    // skip entire TX. Should not happen — drop_oversized_in_window()
                    // purges pre-OFDM frames, and max_info limits new ones.
                    {
                        auto geometry = checked_ofdm_frame_geometry(
                            ofdm_config_, ofdm_tone_map_);
                        auto geometry_capacity = geometry
                            ? ofdm_payload_capacity_bytes(*geometry) : std::nullopt;
                        int capacity = geometry_capacity &&
                                *geometry_capacity <= static_cast<std::uint64_t>(INT_MAX)
                            ? static_cast<int>(*geometry_capacity) : 0;
                        if (capacity == 0 || (int)frame_data.size() > capacity) {
                            IRIS_LOG("[TX-OFDM] DROPPING oversized frame: %zu bytes > %d capacity",
                                     frame_data.size(), capacity);
                            frame_data.clear();
                        }
                    }

                    // Build OFDM frame (returns complex samples, real-valued via Hermitian symmetry)
                    if (!frame_data.empty()) {
                        ofdm_iq = ofdm_mod_->build_ofdm_frame(
                            frame_data.data(), frame_data.size(), ofdm_tone_map_);
                    }

                    if (ofdm_iq.empty()) {
                        if (!frame_data.empty())
                            IRIS_LOG("[TX-OFDM] build_ofdm_frame rejected %zu bytes — skipping TX",
                                     frame_data.size());
                    }
                } else {
                    // ============ Legacy single-carrier PHY TX path ============
                    int level = gearshift_.current_level();
                    PhyConfig tx_config = phy_config_;

                    // Control frames at robust rate (~20ms extra) prevents cascading
                    // retransmit timeouts when channel has degraded.
                    if (batch_is_control && level > 0) {
                        IRIS_LOG("[TX] control frame batch -> forcing level 0 (robust)");
                        level = 0;
                    }

                    Modulation tx_mod;
                    int tx_fec_n, tx_fec_d;
                    if (native_hail_active) {
                        tx_mod = Modulation::BPSK;
                        tx_fec_n = 1;
                        tx_fec_d = 2;
                        IRIS_LOG("TX native hail frame %zu bytes (BPSK r1/2)", frame_data.size());
                    } else {
                        tx_no_ack_count_++;
                        if (tx_no_ack_count_ >= 6 && level > 0) {
                            gearshift_.report_failure();
                            level = gearshift_.current_level();
                            IRIS_LOG("Gearshift: no peer ACK for %d TX frames -> report_failure (level=%d)",
                                     tx_no_ack_count_, level);
                        }
                        if (peer_snr_db_ > 0 && level > 0) {
                            int peer_max = snr_to_speed_level(peer_snr_db_);
                            if (peer_max < level) {
                                IRIS_LOG("Gearshift: capping TX level %d -> %d (peer SNR %.1f dB)",
                                         level, peer_max, peer_snr_db_);
                                level = peer_max;
                            }
                        }
                        tx_mod = SPEED_LEVELS[level].modulation;
                        tx_fec_n = SPEED_LEVELS[level].fec_rate_num;
                        tx_fec_d = SPEED_LEVELS[level].fec_rate_den;
                        IRIS_LOG("[TX] speed=%s (level=%d) mod=%d fec=%d/%d, %zu bytes",
                                 SPEED_LEVELS[level].name, level,
                                 (int)tx_mod, tx_fec_n, tx_fec_d, frame_data.size());
                    }
                    tx_config.modulation = tx_mod;
                    LdpcRate fec = fec_to_ldpc_rate(tx_fec_n, tx_fec_d);
                    if (arq_.harq_has_pending_retx() && arq_.negotiated(CAP_HARQ)) {
                        iq = build_native_frame_harq(frame_data.data(), frame_data.size(),
                                                      arq_.harq_pending_retx_desc(),
                                                      tx_config, fec);
                        arq_.harq_clear_pending_retx();
                        IRIS_LOG("[TX] HARQ frame: retx piggybacked + %zu bytes new data",
                                 frame_data.size());
                    } else {
                        iq = build_native_frame(frame_data.data(), frame_data.size(), tx_config, fec);
                    }
                }

                if (!mfsk_tx_done && ofdm_phy_active_ && ofdm_mod_ && ofdm_iq.empty()) {
                    // Safety net: payload exceeded the current level's LDPC capacity
                    // (e.g. a dynamic-MAX_INFO fall shrank capacity under a frame
                    // already sized for a higher level). Re-queue the popped frames
                    // (survive) instead of destroying them; a session I-frame also
                    // retransmits from the AX.25 window, a B2F/client frame has ONLY
                    // this path (data-flow-tx-queue.md §6.1).
                    tx_buffer_.clear();
                    requeue_rejected_frames(taken);
                    IRIS_LOG("[TX-OFDM] frame rejected (payload %zu > LDPC capacity) — re-queued",
                             frame_data.size());
                    state_ = ModemState::IDLE;
                    return;
                }

                if (!mfsk_tx_done && ofdm_phy_active_ && ofdm_mod_) {
                    // Fix B: a frame is going on air — the reject spin is broken, so
                    // reset the streak. The fit-floor is NOT cleared here (it is retired
                    // in resolve_ofdm_tx_level once the confirmed anchor reaches it), so a
                    // control/ACK send between two drops of a stuck DATA frame can't strip
                    // the floor before that frame modulates (DATALINK_TAX_DIAGNOSIS.md).
                    ofdm_reject_streak_ = 0;
                    // OFDM: IFFT with Hermitian symmetry produces real-valued audio.
                    // Bypass upconverter — signal is already at audio frequencies.
                    // Normalize RMS to match native single-carrier level (~0.707).
                    // Without this, OFDM with N carriers is √(2N) louder than native
                    // (e.g. 42 carriers → 9.2× → 19 dB overdrive → FM limiter destroys signal).
                    // Extract real passband audio from OFDM IFFT
                    tx_buffer_.resize(ofdm_iq.size());
                    for (size_t i = 0; i < ofdm_iq.size(); i++)
                        tx_buffer_[i] = ofdm_iq[i].real();
                    frame_airtime_s = (float)ofdm_iq.size() / (float)config_.sample_rate;

                    // OFDM bypasses probe-based TX EQ. The 127-tap FIR group
                    // delay can exceed the OFDM CP (64 samples default), risking
                    // ISI. OFDM handles per-carrier equalization internally.

                    // Normalize RMS AFTER EQ to control total power.
                    // Target 0.50 is the "full scale" OFDM output before tx_level
                    // scaling. tx_level (set by TUNE) controls actual drive into FM
                    // transmitter. Default tx_level=0.50 → output RMS≈0.25.
                    // TUNE adjusts tx_level targeting mean|H|≈1.0 at the receiver.
                    // ONE scale for the WHOLE frame (see ofdm_normalize_tx_frame).
                    ofdm_normalize_tx_frame(tx_buffer_, 0,
                        ofdm_config_.nfft + ofdm_config_.cp_samples,
                        tx_channel_eq_.is_configured());

                    // ---- OFDM multi-frame burst: append additional frames ----
                    // First frame is built above.  If queue still has data and
                    // the slot bounds allow (resolve_slot_bounds: pre-probe
                    // defaults, or the IRIS_FRAMES_PER_BURST/IRIS_SLOT_AIRTIME_S
                    // coalescing-probe knobs), build more OFDM frames (each with
                    // own preamble → independent sync + channel est) and
                    // concatenate audio in the same PTT cycle.
                    {
                        auto burst_geometry = checked_ofdm_frame_geometry(
                            ofdm_config_, ofdm_tone_map_);
                        auto burst_capacity = burst_geometry
                            ? ofdm_payload_capacity_bytes(*burst_geometry) : std::nullopt;
                        size_t burst_max_payload = burst_capacity
                            ? static_cast<size_t>(*burst_capacity) : 0;

                        // Unified drain (append_coalesced_slot): each additional
                        // burst frame is built by the SAME helper the
                        // reject-requeue test drives — it pops a batch
                        // (provenance preserved), builds, and on reject re-queues
                        // the popped frames instead of destroying them
                        // (dynamic-MAX_INFO safety net).
                        int burst_frames = append_coalesced_slot(
                            frame_airtime_s, burst_max_payload);

                        if (burst_frames > 1) {
                            IRIS_LOG("[TX-OFDM] burst complete: %d frames, %.1fs, %zu samples",
                                     burst_frames, frame_airtime_s, tx_buffer_.size());
                        }

                        // [SLOT-TELEM] one line per assembled OFDM slot: the
                        // queued-TX-byte witness for the host-window question
                        // (see the snapshot comment at the batch drain above).
                        size_t telem_qbytes_after = 0;
                        {
                            std::queue<TxFrame> scan = tx_queue_;
                            while (!scan.empty()) {
                                telem_qbytes_after += scan.front().data.size();
                                scan.pop();
                            }
                        }
                        IRIS_LOG("[SLOT-TELEM] frames=%d airtime=%.2fs level=O%d "
                                 "q_before=%zuB/%zufr q_after=%zuB/%zufr win=%d/%d",
                                 burst_frames, frame_airtime_s, ofdm_speed_level_,
                                 telem_qbytes_before, telem_qframes_before,
                                 telem_qbytes_after, tx_queue_.size(),
                                 ax25_session_.window_used(),
                                 ax25_session_.window_k());
                    }

                    // After OFDM-KISS TX (data OR poll), expect MFSK response from RSP.
                    // Data burst: RSP sends autonomous T2 MFSK RR.
                    // T1 poll (S-frame): RSP sends enquiry_response MFSK RR.
                    // Both need ofdm_expect_ack_ for CMD's MFSK detector to fire.
                    if (ofdm_kiss_tx_ && ax25_session_.we_initiated()) {
                        ofdm_expect_ack_ = true;
                        IRIS_LOG("[MAC-DIAG] %s TX complete, expecting MFSK RR",
                                 kiss_control_batch ? "poll" : "data burst");
                    }
                } else if (!mfsk_tx_done) {
                    // Native single-carrier PHY: IQ through upconverter as before
                    frame_airtime_s = (float)(iq.size() / 2) / (float)config_.sample_rate;
                    if (use_upconvert_) {
                        tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                        if (tx_channel_eq_.is_configured()) {
                            tx_channel_eq_.apply(tx_buffer_.data(), (int)tx_buffer_.size());
                            for (auto& s : tx_buffer_) {
                                if (s > 0.95f) s = 0.95f + 0.05f * std::tanh((s - 0.95f) / 0.05f);
                                else if (s < -0.95f) s = -0.95f + 0.05f * std::tanh((s + 0.95f) / 0.05f);
                            }
                        }
                    } else {
                        tx_buffer_.resize(iq.size() / 2);
                        for (size_t i = 0; i < iq.size() / 2; i++)
                            tx_buffer_[i] = iq[2 * i];
                    }
                }
                ax25_session_.set_t1_floor_for_airtime(frame_airtime_s);
            } else {
                // AX.25 mode: batch all queued frames into one TX burst
                // to minimize PTT on-time and reduce half-duplex collisions.
                // First frame gets full preamble, subsequent get 2 inter-frame flags.
                state_ = ModemState::TX_AX25;
                int preamble_flags = std::max(8, config_.ptt_pre_delay_ms * config_.ax25_baud / 8000);
                std::vector<uint8_t> raw_bits;  // pre-NRZI

                bool first = true;
                while (!tx_queue_.empty()) {
                    auto frame_data = std::move(tx_queue_.front().data);
                    tx_queue_.pop();
                    IRIS_LOG("TX frame %zu bytes (AX.25%s%s)", frame_data.size(),
                             config_.fx25_mode > 0 ? " FX.25" : "",
                             first ? "" : " batched");
                    if (packet_log_ && frame_data.size() >= 14) {
                        std::string proto = config_.fx25_mode > 0
                            ? "FX.25-" + std::to_string(config_.fx25_mode)
                            : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
                        packet_log_(true, proto, describe_ax25(frame_data.data(), frame_data.size()));
                    }
                    int flags = first ? preamble_flags : 2;
                    if (config_.fx25_mode > 0) {
                        if (!fx25_encode_raw(raw_bits, frame_data.data(), frame_data.size(),
                                            config_.fx25_mode, flags))
                            hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                    } else {
                        hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                    }
                    first = false;
                }

                if (!raw_bits.empty()) {
                    if (config_.ax25_baud == 9600)
                        g3ruh_tx_scrambler_.scramble(raw_bits);
                    auto nrzi_bits = nrzi_encode(raw_bits);
                    if (config_.ax25_baud == 9600)
                        tx_buffer_ = gfsk_mod_.modulate(nrzi_bits);
                    else
                        tx_buffer_ = afsk_mod_.modulate(nrzi_bits);
                }
            }
            have_frame = true;
        }

        // Append any pending probe audio to tx_buffer_ (same PTT cycle).
        // This ensures RESULT AFSK goes out before probe tones in Turn 2.
        // Don't scale here — tx_level is applied to entire buffer below.
        if (!probe_audio_pending_.empty()) {
            tx_buffer_.insert(tx_buffer_.end(),
                              probe_audio_pending_.begin(),
                              probe_audio_pending_.end());
            probe_audio_pending_.clear();
            if (!have_frame) {
                have_frame = true;
                state_ = ModemState::TX_AX25;  // Ensure process_rx skips during probe tone TX
            }
            IRIS_LOG("TX probe audio appended (%zu total samples)", tx_buffer_.size());
        }

        if (have_frame) {
            int air_ms = (int)(tx_buffer_.size() * 1000 / config_.sample_rate);
            IRIS_LOG("TX buffer %zu samples (%d ms)", tx_buffer_.size(), air_ms);
            if (gui_log_)
                gui_log_("[TX] " + std::to_string(air_ms) + " ms on air");

            // Apply tx_level gain control to TX buffer.
            // OFDM: per-O-level tx_level (base from TUNE + offset for modulation).
            //   TUNE calibrates ofdm_tx_base_ for O0, higher modes get dB offsets
            //   to account for tighter constellations (more sensitive to FM clipping).
            // Native: tx_level applied directly to upconverted audio.
            {
                float tx_gain = (ofdm_phy_active_ && ofdm_mod_)
                    ? ofdm_effective_tx_level()
                    : config_.tx_level;
                for (auto& s : tx_buffer_)
                    s *= tx_gain;
            }

            // Pre-TX silence for PTT hardware settle.
            // AX.25: 50ms hardware settle + flag bytes provide the rest of TXDELAY.
            // OFDM: full TXDELAY needed — no flag preamble, training symbols are first.
            int ptt_settle_ms;
            if (ofdm_phy_active_ && ofdm_mod_) {
                // Use adaptive TXDELAY if set (reduced after successful probe/tune)
                ptt_settle_ms = (ofdm_txdelay_ms_ > 0) ? ofdm_txdelay_ms_ : config_.ptt_pre_delay_ms;
                IRIS_LOG("[TX-OFDM] pre-delay: %d ms (TXDELAY%s)", ptt_settle_ms,
                         ofdm_txdelay_ms_ > 0 ? ", adaptive" : "");
            } else {
                ptt_settle_ms = 50;  // native SC / AX.25: preamble handles the rest
            }
            int pre_samples = ptt_settle_ms * config_.sample_rate / 1000;
            if (pre_samples > 0)
                tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);

            // Post-TX tail: keep carrier up so peer's squelch stays open.
            // OFDM needs extra tail because frames are short (~176ms).
            int post_ms = config_.ptt_post_delay_ms;
            if (ofdm_phy_active_ && ofdm_mod_)
                post_ms = std::max(post_ms, 100);  // minimum 100ms tail for OFDM
            int post_samples = post_ms * config_.sample_rate / 1000;
            if (post_samples > 0)
                tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

            tx_pos_ = 0;
            frames_tx_++;
        }
    }
    // ptt_on() calls set_channel_busy() which acquires timer_mutex_.
    // Lock order: modem_mutex_ → timer_mutex_ (consistent with ax25_session_.tick()).
    if (tx_pos_ == 0 && !tx_buffer_.empty()) {
        ptt_on();

        size_t to_copy = std::min((size_t)frame_count, tx_buffer_.size());
        std::memcpy(tx_audio, tx_buffer_.data(), to_copy * sizeof(float));
        tx_pos_ = to_copy;
        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        // Apply simulated channel effects
        if (sim_bp_enabled_) {
            for (int i = 0; i < frame_count; i++) {
                float s = tx_audio[i];
                for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                tx_audio[i] = s;
            }
        }
        if (sim_deemph_enabled_) {
            for (int i = 0; i < frame_count; i++)
                tx_audio[i] = sim_deemph_.process(tx_audio[i]);
        }
        return;
    }

    std::memset(tx_audio, 0, frame_count * sizeof(float));
}

// ===========================================================================
// Terminate/re-pack the native-OFDM AX.25 data plane (beat-VARA keystone).
//
// Shadow mode flies each client I-frame VERBATIM, so the in-flight window is
// K(7) x paclen(74) = 518 B -> ~13% of the O5 window (Step-0 window-BYTE ceiling,
// DATALINK_TAX analysis).  Terminate the client I-frame data plane
// on the Iris<->Iris OFDM tier: (R1) custody-ACK the client + append its INFO as
// a [len_be16][INFO] record to a per-session stream; (R2) drain the stream into
// <=max_info(tx_acked_level_) I-frames over the one owned OFDM sequence (send_data
// fragmentation, DYNAMIC size, bootstraps O0 -> no Step-0 un-sendable deadlock);
// (R3) far Iris reassembles the length-delimited stream to the EXACT original
// records and re-originates one clean I-frame each to its local pump, window-paced.
// The 2-B length prefix makes reassembly bit-exact BY CONSTRUCTION under a mid-
// stream frame-size change (Step-0 integrity root).  Gate native_repack_active();
// AFSK / pure-KISS is byte-identical.
// ===========================================================================
namespace {
// A decoded record length above this can only mean the reassembly stream
// misaligned (client INFO <= paclen 74 here, general <= AX.25 N1 = 1024) -> LOUD
// custody teardown, NEVER a silently-misparsed bogus record (P0 integrity).
constexpr size_t REPACK_MAX_RECORD = 1024;
// R3 re-origination window: the local pump classifies mod-8 fwd-dist from V(R) and
// only accepts fdist 0..RX_WINDOW//2 (=3, kiss_data_pump.py:662).  Keep at most 4
// outstanding (newest at fdist 3) so a record is never dropped as a "behind" dup.
constexpr int REPACK_REORIG_WINDOW = 4;
// Custody backpressure hysteresis on TX stream depth (buffered-but-unflown bytes):
// above HIGH -> RNR (client pauses), below LOW -> RR (resume).  Bounds the buffer.
constexpr size_t REPACK_TX_HIGH_WATER = 16384;
constexpr size_t REPACK_TX_LOW_WATER  = 4096;
// Original custody is retained independently of the scheduling stream.  Refuse
// the record before advancing R1 when this fixed ownership budget is unavailable.
constexpr size_t REPACK_LEDGER_MAX_BYTES = 1024 * 1024;
constexpr size_t REPACK_LEDGER_MAX_RECORDS = 4096;
constexpr size_t REPACK_LEDGER_RECORD_CHARGE = 64;
constexpr size_t REPACK_RX_HIGH_WATER = 16384;
constexpr size_t REPACK_RX_LOW_WATER = 4096;
constexpr int REPACK_R3_RETRY_TICKS = 20;
constexpr int REPACK_R3_MAX_RETRIES = 5;
constexpr int REPACK_R3_DEADLINE_TICKS = 200;
// Flush the stream tail after the client is quiet this many ticks (~50ms each).
// Event/idle-driven; the frame SIZE stays dynamic (Step-0 hard-req (a)).
constexpr int REPACK_FLUSH_IDLE_TICKS = 6;

bool same_transfer(const v2::TransferIdentity& a,
                   const v2::TransferIdentity& b) {
    return a.session_id.bytes == b.session_id.bytes &&
           a.direction == b.direction &&
           a.transfer_id.value == b.transfer_id.value;
}

bool same_r3(const v2::R3DomainIdentity& a,
             const v2::R3DomainIdentity& b) {
    return a.remote_client_connection.bytes == b.remote_client_connection.bytes &&
           a.sequence_space.value == b.sequence_space.value;
}

bool same_final_record(const v2::FrozenFinalRecord& a,
                       const v2::FrozenFinalRecord& b) {
    return a.empty_transfer == b.empty_transfer &&
           a.record_count == b.record_count &&
           a.final_record_id.value == b.final_record_id.value &&
           a.final_record_extent == b.final_record_extent &&
           a.catalog_digest == b.catalog_digest;
}

void append_u64_be(std::vector<uint8_t>& bytes, uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8)
        bytes.push_back(static_cast<uint8_t>(value >> shift));
}

void append_u32_be(std::vector<uint8_t>& bytes, uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8)
        bytes.push_back(static_cast<uint8_t>(value >> shift));
}

v2::FrozenFinalRecord freeze_catalog(const v2::TransferLedger& ledger) {
    v2::FrozenFinalRecord final_record;
    final_record.empty_transfer = ledger.original_records.empty();
    final_record.record_count = ledger.original_records.size();
    if (!ledger.original_records.empty()) {
        const auto& last = ledger.original_records.back();
        final_record.final_record_id = last.record_id;
        final_record.final_record_extent = last.bytes.size();
    }

    std::vector<uint8_t> canonical;
    static constexpr uint8_t prefix[] = {
        'I','R','I','S','-','V','2','-','C','A','T','A','L','O','G'};
    canonical.insert(canonical.end(), std::begin(prefix), std::end(prefix));
    canonical.insert(canonical.end(), ledger.transfer.session_id.bytes.begin(),
                     ledger.transfer.session_id.bytes.end());
    canonical.push_back(static_cast<uint8_t>(ledger.transfer.direction));
    append_u64_be(canonical, ledger.transfer.transfer_id.value);
    append_u32_be(canonical,
                  static_cast<uint32_t>(ledger.original_records.size()));
    for (const auto& record : ledger.original_records) {
        append_u64_be(canonical, record.record_id.value);
        append_u64_be(canonical, record.bytes.size());
    }
    crypto_blake2b(final_record.catalog_digest.data(),
                   final_record.catalog_digest.size(), canonical.data(),
                   canonical.size());
    return final_record;
}

size_t ledger_owned_charge(const v2::TransferLedger& ledger) {
    size_t charge = 0;
    for (const auto& record : ledger.original_records) {
        const size_t add = REPACK_LEDGER_RECORD_CHARGE + 2 + record.bytes.size();
        if (add > REPACK_LEDGER_MAX_BYTES - std::min(charge, REPACK_LEDGER_MAX_BYTES))
            return REPACK_LEDGER_MAX_BYTES + 1;
        charge += add;
    }
    return charge;
}
} // namespace

std::vector<Modem::TransferResultPtr> Modem::retained_transfer_results() const {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    return {repack_retained_transfer_results_.begin(),
            repack_retained_transfer_results_.end()};
}

bool Modem::dispose_transfer_result(const v2::TransferIdentity& transfer) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    auto it = std::find_if(
        repack_retained_transfer_results_.begin(),
        repack_retained_transfer_results_.end(),
        [&](const TransferResultPtr& result) {
            return result && same_transfer(result->transfer, transfer);
        });
    if (it == repack_retained_transfer_results_.end())
        return false;
    repack_retained_transfer_results_.erase(it);
    repack_last_transfer_result_.reset();
    for (const auto& result : repack_retained_transfer_results_) {
        if (!repack_last_transfer_result_ ||
            (repack_last_transfer_result_->outcome == v2::TransferOutcome::Succeeded &&
             result->outcome == v2::TransferOutcome::Failed))
            repack_last_transfer_result_ = result;
    }
    return true;
}

std::shared_ptr<const ArqTransferResult> Modem::last_arq_transfer_result() const {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    return arq_.last_transfer_result();
}

bool Modem::dispose_arq_transfer_result(uint64_t transfer_id) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    return arq_.dispose_transfer_result(transfer_id);
}

bool Modem::accept_remote_close_proof(v2::MatchingCloseProof&& proof) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (repack_transfer_state_ != RepackTransferState::AwaitingRemoteClose)
        return false;
    std::optional<v2::MatchingCloseProof> owned;
    owned.emplace(std::move(proof));
    auto captured_session = v2_live_session_;
    return finish_transfer_once(
        v2::TransferResultReason::RemoteEndpointAcceptedAndClosed,
        std::move(owned), std::move(captured_session));
}

void Modem::notify_local_client_count(int count) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    const int bounded_count = std::max(0, count);
    if (repack_local_client_count_ == 0 && bounded_count > 0) {
        const CryptoKey random = crypto_random_key();
        std::copy_n(random.begin(),
                    repack_local_client_connection_id_.bytes.size(),
                    repack_local_client_connection_id_.bytes.begin());
        repack_local_client_connection_valid_ = true;
    }
    repack_local_client_count_ = bounded_count;
    if (repack_local_client_count_ == 0 && repack_rx_transfer_ledger_ &&
        !repack_rx_transfer_ledger_->terminal_published) {
        repack_r3_no_progress_ticks_ =
            std::max(repack_r3_no_progress_ticks_, REPACK_R3_RETRY_TICKS);
    }
}

void Modem::repack_pack_record(std::vector<uint8_t>& stream,
                               const uint8_t* info, size_t len) {
    stream.push_back((uint8_t)((len >> 8) & 0xFF));
    stream.push_back((uint8_t)(len & 0xFF));
    if (len > 0)
        stream.insert(stream.end(), info, info + len);
}

int Modem::repack_split_records(std::vector<uint8_t>& buf,
                                std::queue<std::vector<uint8_t>>& out) {
    size_t off = 0;
    while (buf.size() - off >= 2) {
        size_t rlen = ((size_t)buf[off] << 8) | (size_t)buf[off + 1];
        if (rlen > REPACK_MAX_RECORD) return -1;   // stream misaligned -> LOUD
        if (buf.size() - off < 2 + rlen) break;    // partial record — wait for more
        out.push(std::vector<uint8_t>(buf.begin() + off + 2,
                                      buf.begin() + off + 2 + rlen));
        off += 2 + rlen;
    }
    if (off > 0) buf.erase(buf.begin(), buf.begin() + off);
    return 0;
}

bool Modem::native_repack_active() const {
    // OFDM is the live transport for a CONNECTED KISS session.  b2f_proxy runs its
    // OWN terminate/re-originate path -> never double-terminate.
    return v2_negotiated_active_ && ofdm_kiss_tx_ &&
           ax25_session_.native_active() &&
           ax25_session_.is_kiss_managed() &&
           (ax25_session_.state() == Ax25SessionState::CONNECTED ||
            ax25_session_.state() == Ax25SessionState::TIMER_RECOVERY) &&
           !b2f_proxy_active_ && !b2f_proxy_rx_active_;
}

bool Modem::repack_has_open_custody() const {
    return (repack_transfer_ledger_ &&
            !repack_transfer_ledger_->terminal_published) ||
           (repack_rx_transfer_ledger_ &&
            !repack_rx_transfer_ledger_->terminal_published);
}

bool Modem::handle_native_desync() {
    // DATALINK_INTEGRITY_AUDIT §4/F1: the Ax25Session just declared an unrecoverable
    // N(R) desync. In the native/re-pack transport the AX.25 FRMR+SABM re-establish
    // is a proven DEAD END (kiss-managed CONNECTED ignores SABM, the OFDM pump
    // suppresses it, T1 never re-sends a session-originated SABM, and the UA-site
    // epoch reset discards un-ACKed custody). Rather than the ~28 s zombie that ends
    // in silent/loud custody loss (s00: tx_buf=91288 lost), surface it NOW.
    if (!native_repack_active()) return false;   // AFSK / owned: real re-establish OK
    // Fold the un-ACKed OWNED window (custody-completed client bytes that were drained
    // out of native_repack_tx_stream_ into I-frames) back into the stream so the
    // teardown log reports EVERY undelivered byte — never a silent loss.
    std::vector<uint8_t> recovered;
    ax25_session_.recover_unacked_stream(recovered);
    if (!recovered.empty())
        native_repack_tx_stream_.insert(native_repack_tx_stream_.begin(),
                                        recovered.begin(), recovered.end());
    IRIS_LOG("[REPACK] N(R) desync in re-pack mode — AX.25 re-establish is unreachable; "
             "immediate custody teardown (no SABM zombie), %zu B undelivered",
             native_repack_tx_stream_.size());
    // Ax25Session owns the remaining reset and DISCONNECTED transition.  Its
    // state callback publishes after session state is latched.
    repack_failure_reason_hint_ = v2::TransferResultReason::ProtocolViolation;
    return true;   // session ends cleanly as DISCONNECTED (no dead T1/SABM wait)
}

void Modem::reset_ofdm_belief_and_chase() {
    ofdm_cfo_committed_hz_ = 0.0f;
    ofdm_chase_llrs_.clear();
    ofdm_chase_combines_ = 0;
    ofdm_chase_candidate_id_ = 0;
    ofdm_chase_capture_epoch_ = 0;
    ofdm_chase_level_ = -1;
    ofdm_chase_n_codewords_ = 0;
    ofdm_chase_fec_rate_ = LdpcRate::RATE_1_2;

    ofdm_search_owner_result_ = {};
    ofdm_search_owner_candidate_id_ = 0;
    ofdm_search_owner_capture_epoch_ = 0;
    ofdm_search_owner_level_ = -1;
    ofdm_search_owner_n_codewords_ = 0;
    ofdm_search_owner_chase_attempted_ = false;
    ofdm_search_owner_valid_ = false;

    ofdm_harq_evidence_ = {};
    ofdm_harq_evidence_candidate_id_ = 0;
    ofdm_harq_evidence_capture_epoch_ = 0;
    ofdm_harq_evidence_valid_ = false;
}

void Modem::repack_reset() {
    // reset_silent() deliberately suppresses the AX.25 state callback, so the
    // re-pack teardown must explicitly end OFDM evidence ownership here.
    reset_ofdm_belief_and_chase();
    native_repack_tx_stream_.clear();
    native_repack_rx_stream_.clear();
    while (!native_repack_rx_records_.empty()) native_repack_rx_records_.pop();
    for (size_t i = 0; i < repack_reorig_window_.size(); ++i) {
        repack_reorig_window_[i].clear();
        repack_reorig_window_active_[i] = false;
        repack_reorig_window_record_id_[i] = {};
    }
    repack_tx_client_ns_ = 0;
    repack_reorig_vs_ = 0;
    repack_reorig_va_ = 0;
    repack_reorig_peer_busy_ = false;
    repack_custody_rnr_ = false;
    repack_tx_idle_ticks_ = 0;
    repack_engaged_ = false;
    repack_activated_ = false;
    repack_client_disc_pending_ = false;
    repack_client_disc_frame_.clear();
    repack_failure_reason_hint_ = v2::TransferResultReason::RetryExhausted;
    repack_transfer_state_ = RepackTransferState::Idle;
    repack_transfer_ledger_.reset();
    repack_rx_transfer_ledger_.reset();
    repack_air_ack_stream_.clear();
    repack_air_ack_record_index_ = 0;
    repack_air_ack_serialized_extent_ = 0;
    repack_air_disc_pending_ = false;
    repack_air_disc_delivered_ = false;
    repack_air_disc_frame_.clear();
    repack_air_disc_hold_ticks_ = 0;
    repack_rx_serialized_bytes_ = 0;
    repack_rx_schedule_record_index_ = 0;
    repack_r3_no_progress_ticks_ = 0;
    repack_r3_retry_ticks_ = 0;
    repack_r3_retries_ = 0;
    repack_terminal_requested_ = false;
    repack_remote_call_.clear();
    repack_window_k_logged_ = -1;
    repack_widen_eligible_ = false;
}

void Modem::repack_begin_transfer() {
    if (repack_transfer_ledger_ &&
        !repack_transfer_ledger_->terminal_published)
        return;

    static std::atomic<uint64_t> next_transfer_id{1};
    static std::atomic<uint64_t> next_sequence_space{1};
    auto ledger = std::make_unique<v2::TransferLedger>();
    CryptoKey random_a = crypto_random_key();
    CryptoKey random_b = crypto_random_key();
    if (repack_local_client_connection_valid_)
        ledger->local_connection_id = repack_local_client_connection_id_;
    else
        std::copy_n(random_a.begin(), 16,
                    ledger->local_connection_id.bytes.begin());
    ledger->transfer.direction = ax25_session_.we_initiated()
        ? v2::TransferDirection::InitiatorToResponder
        : v2::TransferDirection::ResponderToInitiator;
    ledger->transfer.transfer_id.value = next_transfer_id.fetch_add(1);
    ledger->domains.r1.local_client_connection = ledger->local_connection_id;
    ledger->domains.r1.sequence_space.value = next_sequence_space.fetch_add(1);
    ledger->domains.r2.modem_session = ledger->transfer.session_id;
    ledger->domains.r2.sequence_space.value = next_sequence_space.fetch_add(1);
    std::copy_n(random_b.begin(), 16,
                ledger->domains.r3.remote_client_connection.bytes.begin());
    ledger->domains.r3.sequence_space.value = next_sequence_space.fetch_add(1);
    ledger->r1_sequence.custody_cursor.domain = ledger->domains.r1;
    ledger->r3_sequence.endpoint_cursor.domain = ledger->domains.r3;

    if (!v2::CloseTransportOwner::register_origin_transfer(*this, *ledger))
        return;

    repack_transfer_ledger_ = std::move(ledger);
    repack_transfer_state_ = RepackTransferState::Open;
    repack_air_ack_stream_.clear();
    repack_air_ack_record_index_ = 0;
    repack_air_ack_serialized_extent_ = 0;
}

void Modem::repack_begin_rx_transfer() {
    if (repack_rx_transfer_ledger_ &&
        !repack_rx_transfer_ledger_->terminal_published)
        return;

    static std::atomic<uint64_t> next_rx_transfer_id{1};
    static std::atomic<uint64_t> next_rx_sequence_space{1};
    auto ledger = std::make_unique<v2::TransferLedger>();
    CryptoKey random_a = crypto_random_key();
    CryptoKey random_b = crypto_random_key();
    std::copy_n(random_a.begin(), 16,
                ledger->local_connection_id.bytes.begin());
    ledger->transfer.direction = ax25_session_.we_initiated()
        ? v2::TransferDirection::ResponderToInitiator
        : v2::TransferDirection::InitiatorToResponder;
    ledger->transfer.transfer_id.value = next_rx_transfer_id.fetch_add(1);
    ledger->domains.r1.local_client_connection = ledger->local_connection_id;
    ledger->domains.r1.sequence_space.value =
        next_rx_sequence_space.fetch_add(1);
    ledger->domains.r2.modem_session = ledger->transfer.session_id;
    ledger->domains.r2.sequence_space.value =
        next_rx_sequence_space.fetch_add(1);
    if (repack_local_client_connection_valid_) {
        ledger->domains.r3.remote_client_connection =
            repack_local_client_connection_id_;
    } else {
        std::copy_n(random_b.begin(), 16,
                    ledger->domains.r3.remote_client_connection.bytes.begin());
    }
    ledger->domains.r3.sequence_space.value =
        next_rx_sequence_space.fetch_add(1);
    ledger->r1_sequence.custody_cursor.domain = ledger->domains.r1;
    ledger->r3_sequence.endpoint_cursor.domain = ledger->domains.r3;
    if (!v2::CloseTransportOwner::register_receiver_transfer(*this, *ledger))
        return;
    repack_rx_transfer_ledger_ = std::move(ledger);
    repack_r3_no_progress_ticks_ = 0;
    repack_r3_retry_ticks_ = 0;
    repack_r3_retries_ = 0;
}

bool Modem::repack_admit_original(const Ax25Frame& frame) {
    if (frame.info.size() > REPACK_MAX_RECORD ||
        frame.info.size() > REPACK_LEDGER_MAX_BYTES)
        return false;
    repack_begin_transfer();
    if (!repack_transfer_ledger_)
        return false;
    auto& ledger = *repack_transfer_ledger_;
    if (ledger.terminal_published || ledger.frozen_final_record ||
        repack_transfer_state_ != RepackTransferState::Open)
        return false;

    if (ledger.original_records.size() >= REPACK_LEDGER_MAX_RECORDS)
        return false;
    const size_t owned_charge = ledger_owned_charge(ledger);
    const size_t record_charge = REPACK_LEDGER_RECORD_CHARGE + 2 + frame.info.size();
    if (owned_charge > REPACK_LEDGER_MAX_BYTES ||
        record_charge > REPACK_LEDGER_MAX_BYTES - owned_charge)
        return false;

    v2::OriginalRecordId record_id{
        static_cast<uint64_t>(ledger.original_records.size()) + 1};
    v2::LedgerOriginalRecord original;
    original.record_id = record_id;
    original.bytes.assign(frame.info.begin(), frame.info.end());
    ledger.original_records.push_back(std::move(original));

    v2::OriginalRecordCoverage mapping;
    if (frame.info.empty()) {
        ledger.milestones.accepted.completed_empty_records.push_back(record_id);
        mapping.completed_empty_records.push_back(record_id);
    } else {
        v2::OriginalRecordRange range{record_id, 0,
                                      static_cast<uint64_t>(frame.info.size())};
        ledger.milestones.accepted.ranges.push_back(range);
        mapping.ranges.push_back(range);
    }

    v2::ClientRecordSequenceMapping sequence_mapping;
    sequence_mapping.absolute_position =
        ledger.r1_sequence.custody_cursor.next_absolute_position;
    sequence_mapping.transfer = ledger.transfer;
    sequence_mapping.original_mapping = std::move(mapping);
    sequence_mapping.client_payload_byte_count = frame.info.size();
    ledger.r1_sequence.owned_mapping.push_back(std::move(sequence_mapping));
    ledger.r1_sequence.custody_cursor.next_absolute_position++;
    repack_remote_call_ = ax25_session_.remote_callsign();
    return true;
}

void Modem::repack_note_air_ack(const uint8_t* data, size_t len) {
    if (!repack_transfer_ledger_ || repack_transfer_ledger_->terminal_published)
        return;
    repack_air_ack_stream_.insert(repack_air_ack_stream_.end(), data, data + len);
    auto& ledger = *repack_transfer_ledger_;

    // Rebuild exact original-byte coverage from the conclusively ACKed serialized
    // prefix.  Keeping the prefix until terminal publication makes a prefix that
    // ends inside a record durable instead of losing it in repack_reset().
    ledger.milestones.air_acknowledged = {};
    repack_air_ack_record_index_ = 0;
    size_t offset = 0;
    for (const auto& original : ledger.original_records) {
        if (repack_air_ack_stream_.size() - offset < 2)
            break;
        const size_t record_len =
            (static_cast<size_t>(repack_air_ack_stream_[offset]) << 8) |
            static_cast<size_t>(repack_air_ack_stream_[offset + 1]);
        if (record_len != original.bytes.size() || record_len > REPACK_MAX_RECORD) {
            repack_failure_reason_hint_ =
                v2::TransferResultReason::MalformedTransportState;
            return;
        }
        const size_t available = repack_air_ack_stream_.size() - offset - 2;
        const size_t acknowledged_payload = std::min(record_len, available);
        if (!std::equal(original.bytes.begin(),
                        original.bytes.begin() + acknowledged_payload,
                        repack_air_ack_stream_.begin() + offset + 2)) {
            repack_failure_reason_hint_ =
                v2::TransferResultReason::MalformedTransportState;
            return;
        }
        if (record_len == 0) {
            ledger.milestones.air_acknowledged.completed_empty_records.push_back(
                original.record_id);
            repack_air_ack_record_index_++;
            offset += 2;
            continue;
        }
        if (acknowledged_payload != 0) {
            ledger.milestones.air_acknowledged.ranges.push_back(
                v2::OriginalRecordRange{
                    original.record_id, 0,
                    static_cast<uint64_t>(acknowledged_payload)});
        }
        if (acknowledged_payload != record_len)
            break;
        repack_air_ack_record_index_++;
        offset += 2 + record_len;
    }
    repack_air_ack_serialized_extent_ = offset;
    if (offset < repack_air_ack_stream_.size() &&
        repack_air_ack_record_index_ == ledger.original_records.size()) {
        repack_failure_reason_hint_ =
            v2::TransferResultReason::MalformedTransportState;
    }
}

bool Modem::finish_transfer_once(
    v2::TransferResultReason reason,
    std::optional<v2::MatchingCloseProof> proof,
    std::shared_ptr<v2::LiveSession> captured_session) {
    const bool tx_open = repack_transfer_ledger_ &&
                         !repack_transfer_ledger_->terminal_published;
    const bool rx_open = repack_rx_transfer_ledger_ &&
                         !repack_rx_transfer_ledger_->terminal_published;
    if (!tx_open && !rx_open)
        return false;

    // Invalidate the session that owns this terminal transfer before any
    // client/result publication can reenter and install a replacement.
    v2::CloseTransportOwner::revoke(*this, captured_session);

    bool tx_success = tx_open && v2::is_success_reason(reason);
    if (tx_success) {
        auto& ledger = *repack_transfer_ledger_;
        const auto final_record = freeze_catalog(ledger);
        size_t serialized_size = 0;
        for (const auto& record : ledger.original_records)
            serialized_size += 2 + record.bytes.size();
        const bool all_air_acknowledged =
            repack_air_ack_record_index_ == ledger.original_records.size() &&
            repack_air_ack_stream_.size() == serialized_size;
        bool matching_proof = false;
        if (proof) {
            const auto& attestation = proof->remote_attestation();
            const auto& confirmation = proof->confirmation();
            matching_proof =
                same_transfer(attestation.transfer, ledger.transfer) &&
                same_transfer(confirmation.transfer, ledger.transfer) &&
                same_r3(attestation.r3, ledger.domains.r3) &&
                same_r3(confirmation.r3, ledger.domains.r3) &&
                same_final_record(attestation.accepted_final_record, final_record) &&
                same_final_record(confirmation.accepted_final_record, final_record) &&
                attestation.close_id != 0 &&
                attestation.close_id == confirmation.close_id &&
                attestation.challenge.bytes == confirmation.challenge.bytes;
        }
        if (!all_air_acknowledged || !matching_proof) {
            tx_success = false;
            reason = v2::TransferResultReason::ProtocolViolation;
            proof.reset();
        } else {
            ledger.close_proof = std::move(proof);
            ledger.milestones.endpoint_acknowledged = ledger.milestones.accepted;
        }
    }

    std::vector<TransferResultPtr> published;
    auto finish_ledger = [&](v2::TransferLedger& ledger, bool success,
                             v2::TransferResultReason ledger_reason) {
        const auto final_record = freeze_catalog(ledger);
        ledger.frozen_final_record = final_record;
        ledger.outcome = success ? v2::TransferOutcome::Succeeded
                                 : v2::TransferOutcome::Failed;
        ledger.terminal_reason = ledger_reason;

        auto result = std::make_shared<v2::TransferResult>();
        result->local_connection_id = ledger.local_connection_id;
        result->transfer = ledger.transfer;
        result->domains = ledger.domains;
        result->outcome = ledger.outcome;
        result->reason = ledger_reason;
        result->milestones = ledger.milestones;
        result->frozen_final_record = final_record;
        result->preserved_incomplete_serialized_record =
            ledger.incomplete_serialized_record;
        if (success)
            result->close_proof = std::move(ledger.close_proof);

        for (const auto& original : ledger.original_records) {
            result->original_records.push_back(v2::OriginalRecordDescriptor{
                original.record_id,
                static_cast<uint64_t>(original.bytes.size())});
            if (original.bytes.empty()) {
                bool endpoint_done = std::any_of(
                    ledger.milestones.endpoint_acknowledged.completed_empty_records.begin(),
                    ledger.milestones.endpoint_acknowledged.completed_empty_records.end(),
                    [&](const v2::OriginalRecordId& id) {
                        return id.value == original.record_id.value;
                    });
                if (!endpoint_done)
                    result->preserved_unresolved_empty_records.push_back(
                        original.record_id);
                continue;
            }

            uint64_t cursor = 0;
            for (const auto& covered :
                 ledger.milestones.endpoint_acknowledged.ranges) {
                if (covered.record_id.value != original.record_id.value)
                    continue;
                if (covered.offset > cursor) {
                    const size_t begin = static_cast<size_t>(cursor);
                    const size_t end = static_cast<size_t>(covered.offset);
                    result->preserved_unresolved_originals.push_back(
                        v2::PreservedOriginal{
                            {original.record_id, cursor, covered.offset - cursor},
                            std::vector<uint8_t>(original.bytes.begin() + begin,
                                                 original.bytes.begin() + end)});
                }
                cursor = std::max(cursor, covered.offset + covered.length);
            }
            if (cursor < original.bytes.size()) {
                const size_t begin = static_cast<size_t>(cursor);
                result->preserved_unresolved_originals.push_back(
                    v2::PreservedOriginal{
                        {original.record_id, cursor,
                         static_cast<uint64_t>(original.bytes.size()) - cursor},
                        std::vector<uint8_t>(original.bytes.begin() + begin,
                                             original.bytes.end())});
            }
        }

        auto validation = v2::validate_transfer_result(*result, ledger);
        if (validation != v2::TransferResultValidationError::None) {
            // Corrupt acknowledgment metadata can never escape as authority.
            // Fall back to a conservative snapshot that retains every accepted
            // original, then validate that snapshot again before publication.
            IRIS_LOG("[REPACK] terminal snapshot validation failed (%d); "
                     "publishing conservative ProtocolViolation result",
                     static_cast<int>(validation));
            ledger.outcome = v2::TransferOutcome::Failed;
            ledger.terminal_reason = v2::TransferResultReason::ProtocolViolation;
            ledger.close_proof.reset();
            ledger.milestones.air_acknowledged = {};
            ledger.milestones.endpoint_acknowledged = {};
            result->outcome = ledger.outcome;
            result->reason = ledger.terminal_reason;
            result->milestones = ledger.milestones;
            result->close_proof.reset();
            result->preserved_unresolved_originals.clear();
            result->preserved_unresolved_empty_records.clear();
            for (const auto& original : ledger.original_records) {
                if (original.bytes.empty()) {
                    result->preserved_unresolved_empty_records.push_back(
                        original.record_id);
                } else {
                    result->preserved_unresolved_originals.push_back({
                        {original.record_id, 0,
                         static_cast<uint64_t>(original.bytes.size())},
                        original.bytes});
                }
            }
            validation = v2::validate_transfer_result(*result, ledger);
            if (validation != v2::TransferResultValidationError::None) {
                IRIS_LOG("[REPACK] conservative terminal snapshot rejected (%d); "
                         "custody retained and publication refused",
                         static_cast<int>(validation));
                return false;
            }
        }

        ledger.terminal_published = true;
        TransferResultPtr immutable = result;
        repack_retained_transfer_results_.push_back(immutable);
        if (!repack_last_transfer_result_ ||
            repack_last_transfer_result_->outcome == v2::TransferOutcome::Succeeded)
            repack_last_transfer_result_ = immutable;
        published.push_back(std::move(immutable));
        return true;
    };

    if (tx_open) {
        if (!finish_ledger(
                *repack_transfer_ledger_, tx_success,
                tx_success
                    ? v2::TransferResultReason::RemoteEndpointAcceptedAndClosed
                    : reason))
            return true;
        tx_success = repack_transfer_ledger_->outcome ==
                     v2::TransferOutcome::Succeeded;
        if (!tx_success && v2::is_success_reason(reason))
            reason = v2::TransferResultReason::ProtocolViolation;
    }
    if (rx_open) {
        // The receiver's local cumulative R3 RR is conclusive coverage, but an
        // origin-side MatchingCloseProof is not locally manufacturable.  A
        // session terminal without that exchange is therefore an explicit
        // failure snapshot, even if its unresolved complement is empty.
        const auto rx_reason = v2::is_success_reason(reason)
            ? v2::TransferResultReason::ProtocolViolation
            : reason;
        if (!finish_ledger(*repack_rx_transfer_ledger_, false, rx_reason))
            return true;
    }

    const std::string remote_call = repack_remote_call_;
    const bool publish_ax25_disconnected =
        ax25_session_.state() != Ax25SessionState::DISCONNECTED;
    auto ax25_state_callback = ax25_state_callback_;
    if (publish_ax25_disconnected) {
        // Complete session destruction before any terminal callback can reenter.
        // In particular, proof-based success must not leave AWAITING_RELEASE
        // armed to forward a later conventional UA as a duplicate success.
        disconnect_timeout_ticks_ = 0;
        ax25_session_.reset_silent();
    }

    repack_transfer_state_ = tx_success ? RepackTransferState::Succeeded
                                        : RepackTransferState::Failed;
    auto result_callback = transfer_result_callback_;
    auto rx_callback = rx_callback_;
    const auto client_disc = repack_client_disc_frame_;

    std::vector<uint8_t> client_terminal_frame;
    if (tx_success && !client_disc.empty()) {
        Ax25Frame disc;
        if (ax25_parse(client_disc.data(), client_disc.size(), disc))
            client_terminal_frame =
                ax25_build_u(disc.src, disc.dst, AX25_CTRL_UA, true, false);
    } else if (!remote_call.empty()) {
        client_terminal_frame = ax25_build_u(
            ax25_make_addr(config_.callsign), ax25_make_addr(remote_call),
            AX25_CTRL_DISC, true, true);
    }

    IRIS_LOG("[REPACK] transfer terminal: outcome=%s reason=%d results=%zu",
             tx_success ? "success" : "failure", static_cast<int>(reason),
             published.size());

    // Latch, snapshot, retain, and clear every old scheduling reference before
    // invoking external code.  Enclosing callers treat true as a hard return.
    repack_reset();
    if (rx_callback && !client_terminal_frame.empty())
        rx_callback(client_terminal_frame.data(), client_terminal_frame.size());
    if (result_callback) {
        for (auto& result : published)
            result_callback(result);
    }
    if (publish_ax25_disconnected && ax25_state_callback &&
        ax25_session_.state() == Ax25SessionState::DISCONNECTED)
        ax25_state_callback(Ax25SessionState::DISCONNECTED, remote_call);
    return true;
}

void Modem::repack_maybe_engage() {
    if (repack_activated_) return;
    if (!native_repack_active()) return;
    repack_activated_ = true;
    repack_engage_at_activation();
}

void Modem::repack_engage_at_activation() {
    // WIDE WINDOW: switch the OWNED R2 session to the modulo-128 (2-octet control)
    // FORMAT BEFORE any owned I-frame is built (this side) or the far side's first
    // owned frame is parsed (the same repack_maybe_engage() hook fires on BOTH ends
    // before the first R2 frame, so the two sequence spaces + the control format
    // match).  At this point V(A)=V(S)=0 (rolled back below) and V(R) < 8, so the low
    // counters are identical in mod-8 and modulo-128 — the switch is seamless.  The
    // window K starts at 7 (climb cadence); repack_tx_drain grows it to the wide K
    // once the anchor reaches the sustainable O5 (the STEADY-STATE turnaround lever).
    // Re-apply max_info so the first (O0) window reserves the extra control octet.
    if (wide_window_) {
        ax25_session_.set_wide_window(true, Ax25Session::K_WINDOW);
        ax25_session_.set_max_info(ofdm_max_info_for_level(tx_acked_level_, ofdm_config_));
    }
    // Absorb any client I-frames HELD verbatim in the modem TX queue during
    // probe/tune into the length-delimited stream (in N(S) order) + custody-ACK,
    // so NOTHING verbatim flies over R2 (which would misalign the far reassembler).
    // Control / non-client frames stay queued (they still fly).
    std::queue<TxFrame> keep;
    int absorbed = 0;
    while (!tx_queue_.empty()) {
        TxFrame tf = std::move(tx_queue_.front());
        tx_queue_.pop();
        Ax25Frame f;
        bool client_i = tf.data.size() > 14 &&
                        ax25_parse(tf.data.data(), tf.data.size(), f) &&
                        f.type() == Ax25FrameType::I_FRAME &&
                        f.src.matches(config_.callsign) &&
                        f.dst.matches(ax25_session_.remote_callsign());
        if (client_i && f.ns() == repack_tx_client_ns_) {
            if (repack_admit_original(f)) {
                repack_pack_record(native_repack_tx_stream_, f.info.data(), f.info.size());
                repack_tx_client_ns_ = (repack_tx_client_ns_ + 1) & 0x07;
                repack_engaged_ = true;
                absorbed++;
            } else {
                repack_custody_rnr_ = true;
            }
            auto ack = ax25_build_s(
                f.src, f.dst,
                repack_custody_rnr_ ? Ax25SType::RNR : Ax25SType::RR,
                repack_tx_client_ns_, false, false);
            std::optional<v2::TransferIdentity> transfer;
            if (repack_transfer_ledger_)
                transfer = repack_transfer_ledger_->transfer;
            if (rx_callback_) rx_callback_(ack.data(), ack.size());
            if (transfer &&
                (!repack_transfer_ledger_ ||
                 !same_transfer(repack_transfer_ledger_->transfer, *transfer)))
                return;
        } else {
            keep.push(std::move(tf));   // control / out-of-order — keep verbatim
        }
    }
    tx_queue_ = std::move(keep);
    // Roll the owned R2 window back to V(A): the absorbed frames were shadow-counted
    // (notify_outgoing advanced V(S)); their data now flies re-packed from V(A),
    // which == the far end's V(R) (both counted the same ACKed prefix) -> seam-free.
    ax25_session_.rollback_unacked_window();
    IRIS_LOG("[REPACK] engaged: absorbed %d held client I-frame(s), %zu stream B, V(S)->V(A)",
             absorbed, native_repack_tx_stream_.size());
    repack_tx_drain(false);
}

bool Modem::repack_tx_ingest(const uint8_t* frame, size_t len) {
    Ax25Frame f;
    if (!ax25_parse(frame, len, f)) return false;
    if (f.type() != Ax25FrameType::I_FRAME) return false;
    if (!f.src.matches(config_.callsign)) return false;   // only OUR client's data
    if (!f.dst.matches(ax25_session_.remote_callsign())) return false;
    uint8_t ns = f.ns();
    // Dedup client go-back-N retransmits: buffer INFO ONLY for the strictly in-order
    // NEW frame; a dup/retransmit is re-ACKed (custody) but never re-buffered (a
    // duplicate record would corrupt the reassembled SHA stream).
    bool is_new = (ns == repack_tx_client_ns_);
    if (is_new && !repack_admit_original(f)) {
        repack_custody_rnr_ = true;
        auto rnr = ax25_build_s(f.src, f.dst, Ax25SType::RNR,
                                repack_tx_client_ns_, false, false);
        if (rx_callback_) rx_callback_(rnr.data(), rnr.size());
        return true;
    }
    if (!repack_transfer_ledger_)
        return false;
    if (is_new) {
        repack_pack_record(native_repack_tx_stream_, f.info.data(), f.info.size());
        repack_tx_client_ns_ = (repack_tx_client_ns_ + 1) & 0x07;
        repack_engaged_ = true;
        repack_tx_idle_ticks_ = 0;
    }
    // Custody-ACK: cumulative N(R) = next expected -> the client's window advances
    // without waiting for the far end.  Backpressure via RNR/RR hysteresis so the
    // client cannot overrun the stream buffer.
    size_t depth = native_repack_tx_stream_.size();
    if (depth >= REPACK_TX_HIGH_WATER) repack_custody_rnr_ = true;
    else if (depth < REPACK_TX_LOW_WATER) repack_custody_rnr_ = false;
    auto ack = ax25_build_s(f.src, f.dst,
                            repack_custody_rnr_ ? Ax25SType::RNR : Ax25SType::RR,
                            repack_tx_client_ns_, false, false);
    const auto transfer = repack_transfer_ledger_->transfer;
    if (rx_callback_) rx_callback_(ack.data(), ack.size());
    if (!repack_transfer_ledger_ ||
        !same_transfer(repack_transfer_ledger_->transfer, transfer))
        return true;
    repack_tx_drain(false);
    return true;
}

bool Modem::repack_sframe_consume(const uint8_t* frame, size_t len) {
    Ax25Frame f;
    if (!ax25_parse(frame, len, f)) return false;
    bool sframe = f.type() == Ax25FrameType::S_FRAME;
    if (!sframe && f.type() != Ax25FrameType::I_FRAME) return false;
    if (!f.src.matches(config_.callsign)) return false;   // OUR local pump's ACK
    // The local pump's RR/RNR or piggyback N(R) advances the re-orig window.
    if (!f.dst.matches(repack_remote_call_)) return false;
    if (!repack_rx_transfer_ledger_ ||
        repack_rx_transfer_ledger_->terminal_published)
        return false;
    // A supervisory ACK is R3-local and consumed; a piggyback N(R) advances
    // the same local sequence space before its I-frame payload falls through.
    uint8_t nr = f.nr() & 0x07;
    const auto receive_transfer = repack_rx_transfer_ledger_->transfer;
    uint8_t outstanding = (repack_reorig_vs_ - repack_reorig_va_) & 0x07;
    uint8_t fwd = (nr - repack_reorig_va_) & 0x07;
    if (fwd > outstanding) {
        finish_transfer_once(v2::TransferResultReason::ProtocolViolation);
        return true;
    }
    if (fwd > 0 && fwd <= outstanding) {
        uint8_t seq = repack_reorig_va_;
        while (seq != nr) {
            if (repack_rx_transfer_ledger_ &&
                repack_reorig_window_active_[seq]) {
                const auto record_id = repack_reorig_window_record_id_[seq];
                auto it = std::find_if(
                    repack_rx_transfer_ledger_->original_records.begin(),
                    repack_rx_transfer_ledger_->original_records.end(),
                    [&](const v2::LedgerOriginalRecord& record) {
                        return record.record_id.value == record_id.value;
                    });
                if (it != repack_rx_transfer_ledger_->original_records.end()) {
                    if (it->bytes.empty()) {
                        repack_rx_transfer_ledger_->milestones.endpoint_acknowledged
                            .completed_empty_records.push_back(record_id);
                    } else {
                        repack_rx_transfer_ledger_->milestones.endpoint_acknowledged
                            .ranges.push_back({
                                record_id, 0,
                                static_cast<uint64_t>(it->bytes.size())});
                    }
                }
            }
            repack_reorig_window_[seq].clear();
            repack_reorig_window_active_[seq] = false;
            repack_reorig_window_record_id_[seq] = {};
            seq = (seq + 1) & 0x07;
        }
        repack_reorig_va_ = nr;
        if (repack_rx_transfer_ledger_)
            repack_rx_transfer_ledger_->r3_sequence.endpoint_cursor
                .next_absolute_position += fwd;
        repack_r3_no_progress_ticks_ = 0;
        repack_r3_retry_ticks_ = 0;
        repack_r3_retries_ = 0;
    }

    if (sframe) {
        if (f.s_type() == Ax25SType::RNR) {
            repack_reorig_peer_busy_ = true;
        } else {
            repack_reorig_peer_busy_ = false;
        }
        if (f.s_type() == Ax25SType::REJ) {
            // Retransmit the retained window with the original sequence numbers.
            // Build each frame before the callback: an immediate cumulative ACK
            // may reenter and release the underlying slot.
            uint8_t seq = repack_reorig_va_;
            const uint8_t limit = repack_reorig_vs_;
            while (seq != limit) {
                if (repack_reorig_window_active_[seq]) {
                    const auto bytes = repack_reorig_window_[seq];
                    auto iframe = ax25_build_i(
                        ax25_make_addr(config_.callsign),
                        ax25_make_addr(repack_remote_call_), seq, 0, false,
                        AX25_PID_NONE, bytes.data(), bytes.size());
                    if (rx_callback_)
                        rx_callback_(iframe.data(), iframe.size());
                    if (!repack_rx_transfer_ledger_ ||
                        !same_transfer(repack_rx_transfer_ledger_->transfer,
                                       receive_transfer))
                        return true;
                }
                seq = (seq + 1) & 0x07;
            }
        }
    }
    if (!repack_reorig_peer_busy_) {
        repack_rx_drain();
        if (!repack_rx_transfer_ledger_ ||
            repack_rx_transfer_ledger_->terminal_published ||
            !same_transfer(repack_rx_transfer_ledger_->transfer,
                           receive_transfer))
            return true;
    }
    repack_update_rx_backpressure();
    if (!repack_rx_transfer_ledger_ ||
        repack_rx_transfer_ledger_->terminal_published ||
        !same_transfer(repack_rx_transfer_ledger_->transfer,
                       receive_transfer))
        return true;
    return sframe;
}

bool Modem::repack_hold_client_disc(const uint8_t* frame, size_t len) {
    Ax25Frame f;
    if (!ax25_parse(frame, len, f)) return false;
    if (f.type() != Ax25FrameType::U_FRAME) return false;
    if (f.u_type() != Ax25UType::DISC) return false;
    if (!f.src.matches(config_.callsign)) return false;   // our client's DISC
    if (!f.dst.matches(repack_remote_call_)) return false;
    // An accepted ledger remains pending even after every scheduling queue has
    // drained.  Only matching remote close proof can complete it.
    bool pending = repack_transfer_ledger_ &&
                   !repack_transfer_ledger_->terminal_published &&
                   !repack_transfer_ledger_->original_records.empty();
    if (!pending) return false;
    // Duplicate DISC is idempotent: retain the first exact close intent and
    // continue the same transfer; never manufacture early success.
    if (repack_transfer_state_ == RepackTransferState::Draining ||
        repack_transfer_state_ == RepackTransferState::AwaitingRemoteClose)
        return true;
    repack_client_disc_frame_.assign(frame, frame + len);
    repack_client_disc_pending_ = true;
    repack_transfer_state_ = RepackTransferState::Draining;
    IRIS_LOG("[REPACK] client DISC held (custody graceful close): %zu stream B, window=%d",
             native_repack_tx_stream_.size(), ax25_session_.window_used());
    repack_tx_drain(false);
    return true;
}

bool Modem::repack_complete_remote_close(const uint8_t* frame, size_t len) {
    if (!repack_air_disc_pending_ || !repack_air_disc_delivered_ ||
        repack_air_disc_frame_.empty())
        return false;
    Ax25Frame local_close;
    if (!ax25_parse(frame, len, local_close) ||
        local_close.type() != Ax25FrameType::U_FRAME ||
        local_close.u_type() != Ax25UType::UA || !local_close.poll_final() ||
        !local_close.src.matches(config_.callsign) ||
        !local_close.dst.matches(repack_remote_call_))
        return false;

    Ax25Frame held_air_disc;
    if (!ax25_parse(repack_air_disc_frame_.data(),
                    repack_air_disc_frame_.size(), held_air_disc)) {
        repack_failure_reason_hint_ =
            v2::TransferResultReason::MalformedTransportState;
        ax25_session_.reset();
        return true;
    }

    // Consume the client's close exactly once and emit the promised air UA
    // directly.  Feeding the held DISC back into the KISS-managed session only
    // reset shadow state; that branch never generated UA.
    auto air_ua = ax25_build_u(held_air_disc.src, held_air_disc.dst,
                               AX25_CTRL_UA, true, false);
    if (ofdm_kiss_tx_)
        enqueue_native_tx_frame(std::move(air_ua), false);
    else
        ax25_tx_queue_.push(std::move(air_ua));
    repack_air_disc_pending_ = false;
    repack_air_disc_delivered_ = false;
    repack_air_disc_frame_.clear();
    // The conventional close is not the authenticated v2 close exchange.  It
    // ends this shadow session fail-closed; a future RC2 receive boundary must
    // supply MatchingCloseProof for a successful terminal result.
    repack_failure_reason_hint_ = v2::TransferResultReason::ProtocolViolation;
    ax25_session_.reset();
    return true;
}

bool Modem::repack_reorig_drained() const {
    return native_repack_rx_records_.empty() && native_repack_rx_stream_.empty() &&
           ((repack_reorig_vs_ - repack_reorig_va_) & 0x07) == 0;
}

void Modem::repack_graceful_close_tick() {
    // NEAR: the client DISC is held until the buffered stream is fully air-drained
    // (stream empty AND owned window fully air-ACKed) -> then send the real air DISC.
    // A dead air link self-resolves via T1/N2 -> DISCONNECTED -> custody teardown.
    if (repack_client_disc_pending_ &&
        repack_transfer_state_ == RepackTransferState::Draining &&
        native_repack_tx_stream_.empty() &&
        ax25_session_.window_used() == 0) {
        repack_client_disc_pending_ = false;
        repack_transfer_state_ = RepackTransferState::AwaitingRemoteClose;
        repack_failure_reason_hint_ =
            v2::TransferResultReason::RemoteCloseTimeout;
        IRIS_LOG("[REPACK] air-drained -> propagating held client DISC (graceful close)");
        ax25_session_.disconnect();
    }
    // FAR: a received air DISC is held until the re-origination queue has drained to
    // the local pump (bounded backstop for a pump that already exited).
    if (repack_air_disc_pending_) {
        repack_rx_drain();
        repack_air_disc_hold_ticks_++;
        if (!repack_air_disc_delivered_ && repack_reorig_drained()) {
            // Latch before publication: a local client may answer synchronously.
            repack_air_disc_delivered_ = true;
            IRIS_LOG("[REPACK] R3 cumulatively acknowledged; propagating air DISC to pump");
            if (rx_callback_ && !repack_remote_call_.empty()) {
                Ax25Address dst = ax25_make_addr(config_.callsign);
                Ax25Address src = ax25_make_addr(repack_remote_call_);
                auto disc = ax25_build_u(dst, src, AX25_CTRL_DISC, true, true);
                const auto transfer = repack_rx_transfer_ledger_->transfer;
                rx_callback_(disc.data(), disc.size());
                if (!repack_rx_transfer_ledger_ ||
                    !same_transfer(repack_rx_transfer_ledger_->transfer,
                                   transfer))
                    return;
            }
        }
        if (repack_air_disc_hold_ticks_ > 200) {
            repack_failure_reason_hint_ =
                v2::TransferResultReason::RemoteClientLost;
            ax25_session_.reset();
            return;
        }
    }
}

void Modem::repack_tx_drain(bool flush_partial) {
    if (!native_repack_active()) return;
    // WIDE WINDOW steady-state gate.  The widen from K=7 to the wide value was
    // intended to amortize the per-window reverse-ACK turnaround once the anchor
    // reaches the sustainable cap.  MEASURED (WGN:40, 57- and 83-car, n=16 paired,
    // real-audio, dict-off wire parity, gear cap held identical): with the
    // burst-fill continuation ON (default), the wide window STARVES.
    // burst_fill_continue() keeps bypassing the stop-and-wait and firing a slot
    // for every trickle while window_used < window_k, so a wide K never lets the
    // coalescer accumulate a full slot (~1.1-1.5 frames/slot at K=63 vs 3.4-5.1
    // at K=7) and the turnaround is paid per-trickle instead of per-batch.  Net:
    // the K=63 steady state delivers 4.5-5.2x LESS than plain K=7 at the SAME
    // gear cap.  The window-full stop-and-wait that BATCHES K=7 into full slots
    // is exactly what a wide K removes.  Until burst_fill is taught to accumulate
    // a full slot before airing (the owed follow-up that could make a wide window
    // productive AND needs its own two-stack collision re-validation), the widen
    // is DISABLED by defaulting wide_window_k_ to K_WINDOW (7).  IRIS_WIDE_WINDOW_K
    // re-enables a larger K for that follow-up work.  (A demote below the cap
    // shrinks K back to 7 anyway — safe per INV-SEQ-1: window_k_ gates NEW
    // transmissions only; the V(A)-advance guards no longer read K, so a shrink
    // under an outstanding flight can never refuse its honest cumulative ACK.)
    if (wide_window_) {
        bool eligible = (tx_acked_level_ >= max_proposable_ofdm_level_);
        int target_k = eligible ? wide_window_k_ : Ax25Session::K_WINDOW;
        // Fire-proof: log on the rising edge of widen-eligibility (the exact point
        // the pre-fix build grew K to wide_window_k_) AND on any K change, so the
        // held-widen is VISIBLE at the O5 operating point, not just the O0 start.
        if (eligible != repack_widen_eligible_ || target_k != repack_window_k_logged_) {
            IRIS_LOG("[TURNAROUND] window K -> %d (acked=O%d cap=O%d wide_k=%d)%s",
                     target_k, tx_acked_level_, max_proposable_ofdm_level_,
                     wide_window_k_,
                     (eligible && target_k <= Ax25Session::K_WINDOW)
                         ? "  [WIDEN HELD at K=7: a wide K starves under burst-fill]"
                         : (eligible ? "  [widen ENGAGED]" : ""));
            repack_widen_eligible_ = eligible;
            repack_window_k_logged_ = target_k;
        }
        ax25_session_.set_window_k(target_k);
    }
    int mi = ax25_session_.max_info();
    if (mi < 1) mi = 1;
    // Drain full max_info() chunks while the owned R2 window has room; a final
    // SHORT chunk flies only on flush_partial (client idle) so mid-stream frames
    // pack densely.  send_data() re-fragments at max_info_, so a chunk == mi -> one
    // I-frame <= mi <= anchor_cap -> the oversize guard never trips.
    size_t off = 0;
    while (ax25_session_.pending_frames() == ax25_session_.window_used() &&
           ax25_session_.window_used() < ax25_session_.window_k()) {
        size_t avail = native_repack_tx_stream_.size() - off;
        if (avail == 0) break;
        size_t take;
        if (avail >= (size_t)mi)      take = (size_t)mi;
        else if (flush_partial)       take = avail;
        else                          break;   // hold the partial to pack more
        ax25_session_.send_data(native_repack_tx_stream_.data() + off, take);
        off += take;
    }
    if (off > 0)
        native_repack_tx_stream_.erase(native_repack_tx_stream_.begin(),
                                       native_repack_tx_stream_.begin() + off);
}

void Modem::repack_purge_native_data_frames() {
    // Drop OUR-callsign I-frames (re-pack data emitted at the OLD, now-oversize size)
    // from the modem tx_queue_.  In terminate mode the ONLY I-frames the owned session
    // emits are re-pack data (notify_outgoing bypassed, INV-2), and after
    // recover_unacked_stream those bytes are back in the stream to re-emit fresh — so
    // the queued copies are stale and MUST NOT fly (a fit-floor could raise the level
    // to fit one, and it would then decode a DUPLICATE that corrupts the far
    // reassembler).  Control/S-frames and any non-ours frame stay untouched.
    std::queue<TxFrame> keep;
    int purged = 0;
    while (!tx_queue_.empty()) {
        TxFrame tf = std::move(tx_queue_.front());
        tx_queue_.pop();
        Ax25Frame f;
        bool ours_i = tf.data.size() > 14 &&
                      ax25_parse(tf.data.data(), tf.data.size(), f) &&
                      f.type() == Ax25FrameType::I_FRAME &&
                      f.src.matches(config_.callsign);
        if (ours_i) { purged++; continue; }
        keep.push(std::move(tf));
    }
    tx_queue_ = std::move(keep);
    if (purged > 0)
        IRIS_LOG("[REPACK] re-fragment: purged %d stale oversize I-frame(s) from tx_queue_",
                 purged);
}

bool Modem::demote_tx_anchor(const char* reason, bool defer_window_mutation) {
    // Single shared anchor-demote path (data-flow-tx-anchor.md §4). Both live
    // triggers now DEFER the window mutation onto INV-SEQ-2's quiesce:
    //   - anchor futility (TX context via resolve_ofdm_tx_level) — a popped batch
    //     is mid-flight there, and rolling V(S) under it would put two slicings of
    //     the same N(S) on air;
    //   - peer REJ/RNR (RX-context sniff, DATALINK_INTEGRITY_AUDIT §4/F2) — the
    //     sniff runs BEFORE on_frame_received, so an INLINE harvest would roll V(S)
    //     back to the PRE-ack V(A) and re-use peer-counted numbers at wide K.
    // The anchor + MAX_INFO shrink happens INLINE either way (a real REJ still
    // lowers the ceiling now); only the destructive re-fragment/rollback is held.
    // defer_window_mutation=false remains for the non-repack drop path and tests,
    // but no production caller passes it after F2.
    ofdm_anchor_futility_ = 0;
    if (tx_acked_level_ <= 0) return false;
    tx_acked_level_ = std::max(0, tx_acked_level_ - 1);
    IRIS_LOG("MAC: tx_acked_level -> O%d (%s)", tx_acked_level_, reason);
    // leg 3: SHRINK MAX_INFO to the lowered anchor.  In terminate/re-pack mode
    // the un-ACKed window bytes are OWNED transport data the KISS client
    // already custody-completed — dropping them (drop_oversized_in_window) is
    // a silent data loss -> incomplete transfer (the combined-stack 2/8-incomplete
    // stall, INV-3).  Instead RECOVER + re-fragment the un-ACKed stream at the
    // new (smaller) size; the [len] records re-slice bit-exact.  Non-repack/
    // AFSK keeps drop_oversized_in_window (the KISS client retransmits its own
    // frames — B2F/client frames in Modem::tx_queue_ recover via the
    // reject-requeue, leg 2).  A5d: clear the ring either way — a purged/
    // re-fragmented frame must not credit the anchor.
    ax25_session_.set_max_info(ofdm_max_info_for_level(tx_acked_level_, ofdm_config_));
    if (defer_window_mutation) {
        ofdm_refrag_pending_ = true;
        // INV-SEQ-2: hold owned I-frame TX (fills AND re-airs) from this instant —
        // the window is about to be re-sliced, and quiescing the data direction is
        // what makes the peer's next cumulative N(R) its final V(R) for the epoch
        // (consume_pending_window_refrag's rollback-safety proof).  S-frames /
        // T1 polls keep flowing.
        if (repack_refragment_ && native_repack_active())
            ax25_session_.set_iframe_hold(true);
        IRIS_LOG("MAC: window re-fragment DEFERRED to next idle TX tick (max_info=%d)",
                 ax25_session_.max_info());
    } else {
        if (repack_refragment_ && native_repack_active())
            repack_refragment_on_demote();
        else
            ax25_session_.drop_oversized_in_window();
    }
    tx_level_ring_.reset();
    return true;
}

void Modem::consume_pending_window_refrag() {
    if (!ofdm_refrag_pending_) return;
    // INV-SEQ-2 rollback safety (data-flow-owned-shadow-seq.md §5): NEVER harvest /
    // rewind V(S) across an in-flight window.  The peer's V(R) may already have
    // counted those frames while the cumulative RR is still in transit (measured:
    // the honest RR lost the race to this harvest by 0.37-0.46 s in the s00/s02
    // desync specimens); rolling V(S) back re-USES their numbers with re-sliced
    // content -> the peer discards the re-sends as behind-V(R) dups and every later
    // honest N(R) reads invalid at our end -> FRMR/SABM teardown.  Hold until the
    // ACK state RESOLVES:
    //   (a) V(A)==V(S): the whole flight is credited (INV-SEQ-1 accepts the wide
    //       ACK) — the harvest touches only never-aired queued bytes; or
    //   (b) quiesce: I-frame TX held + queued data purged since the hold engaged,
    //       REFRAG_QUIESCE_TICKS elapsed (the peer's demod pipeline drained), and
    //       a valid N(R) APPLIED after that window — the peer's FINAL V(R) for the
    //       epoch, proving frames beyond V(A) were never counted.
    // S-frames (T1 polls / RRs) keep flowing during the hold, so a lost reverse RR
    // is re-elicited; a dead reverse channel ends in T1/N2 teardown — a LOSS,
    // never a desync/corruption.
    if (repack_refragment_ && native_repack_active() &&
        ax25_session_.window_used() > 0) {
        int now = ax25_session_.wall_ticks();
        if (refrag_quiesce_start_tick_ < 0) {
            // First idle tick with the mutation pending: PTT is off and tx_buffer_
            // drained here (this call site's contract), so the air goes quiet NOW —
            // purge any queued data re-airs and start the quiesce clock.
            refrag_quiesce_start_tick_ = now;
            ax25_session_.set_iframe_hold(true);   // idempotent (set at demote)
            repack_purge_native_data_frames();
            IRIS_LOG("[REPACK] re-fragment HELD: %d frame(s) in flight — waiting for "
                     "the cumulative ACK to resolve (quiesce %.1fs)",
                     ax25_session_.window_used(), REFRAG_QUIESCE_TICKS * 0.05);
            return;
        }
        bool drained  = (now - refrag_quiesce_start_tick_) >= REFRAG_QUIESCE_TICKS;
        bool fresh_nr = ax25_session_.last_valid_nr_tick() >=
                        refrag_quiesce_start_tick_ + REFRAG_QUIESCE_TICKS;
        if (!(drained && fresh_nr)) return;   // keep holding
        IRIS_LOG("[REPACK] re-fragment quiesce RESOLVED: post-drain N(R) is the "
                 "peer's final V(R) — %d frame(s) provably uncounted, safe to "
                 "re-slice", ax25_session_.window_used());
    }
    ofdm_refrag_pending_ = false;
    refrag_quiesce_start_tick_ = -1;
    ax25_session_.set_iframe_hold(false);
    if (repack_refragment_ && native_repack_active())
        repack_refragment_on_demote();
    else
        ax25_session_.drop_oversized_in_window();
    // The stuck frame the futility demote freed no longer exists at its old
    // size — a fit-floor computed FOR it must not force the level back up
    // into the wall the demote just stepped away from.
    ofdm_reject_streak_ = 0;
    ofdm_fit_floor_level_ = -1;
}

void Modem::repack_refragment_on_demote() {
    // TERMINATE x DEMOTE re-fragment (crossover recovery, INV-3).  max_info() has just
    // shrunk (anchor demote).  RECOVER the un-ACKed owned-window bytes back into the
    // stream, PREPEND them (they precede the not-yet-drained suffix in byte order),
    // purge their stale oversize copies from the modem tx_queue_, and re-drain at the
    // new size.  No stream byte is dropped; the [len] records re-slice bit-exact by
    // construction.  Cross-layer audit: data-flow-terminate-repack.md §Q5 (INV-3).
    std::vector<uint8_t> recovered;
    ax25_session_.recover_unacked_stream(recovered);   // harvests + rolls window to V(A)
    if (!recovered.empty())
        native_repack_tx_stream_.insert(native_repack_tx_stream_.begin(),
                                        recovered.begin(), recovered.end());
    repack_purge_native_data_frames();
    IRIS_LOG("[REPACK] re-fragment on demote: recovered %zu un-ACKed B -> stream %zu B, "
             "re-slice at max_info=%d", recovered.size(),
             native_repack_tx_stream_.size(), ax25_session_.max_info());
    repack_tx_drain(false);
}

bool Modem::repack_rx_ingest(const uint8_t* info, size_t len) {
    // Admission is transactional: parse and capacity-check a temporary stream,
    // then commit both the receiver ledger and scheduling FIFO before Ax25Session
    // is allowed to advance V(R).
    // RNR is an admission boundary, not merely an advisory bit. Once retained
    // R3 work crosses the high-water mark, keep V(R) pinned until cumulative R3
    // progress drops below the low-water mark and clears own-busy.
    if (ax25_session_.own_busy())
        return false;
    if (len > REPACK_LEDGER_MAX_BYTES -
                  std::min(repack_rx_serialized_bytes_, REPACK_LEDGER_MAX_BYTES)) {
        ax25_session_.set_own_busy(true);
        return false;
    }
    std::vector<uint8_t> candidate = native_repack_rx_stream_;
    candidate.insert(candidate.end(), info, info + len);
    std::queue<std::vector<uint8_t>> admitted_records;
    if (repack_split_records(candidate, admitted_records) < 0) {
        IRIS_LOG("[REPACK-RX] FATAL: record len > max %zu -> stream misaligned",
                 REPACK_MAX_RECORD);
        repack_failure_reason_hint_ =
            v2::TransferResultReason::MalformedTransportState;
        repack_terminal_requested_ = true;
        return false;
    }

    const size_t existing_records = repack_rx_transfer_ledger_
        ? repack_rx_transfer_ledger_->original_records.size() : 0;
    size_t charge = repack_rx_transfer_ledger_
        ? ledger_owned_charge(*repack_rx_transfer_ledger_) : 0;
    auto inspect = admitted_records;
    size_t new_count = 0;
    while (!inspect.empty()) {
        const size_t add = REPACK_LEDGER_RECORD_CHARGE + 2 + inspect.front().size();
        if (existing_records + new_count >= REPACK_LEDGER_MAX_RECORDS ||
            charge > REPACK_LEDGER_MAX_BYTES ||
            add > REPACK_LEDGER_MAX_BYTES - charge) {
            ax25_session_.set_own_busy(true);
            return false;
        }
        charge += add;
        ++new_count;
        inspect.pop();
    }

    repack_begin_rx_transfer();
    if (!repack_rx_transfer_ledger_) {
        ax25_session_.set_own_busy(true);
        return false;
    }
    auto& ledger = *repack_rx_transfer_ledger_;
    native_repack_rx_stream_ = std::move(candidate);
    repack_rx_serialized_bytes_ += len;
    ledger.incomplete_serialized_record = native_repack_rx_stream_;
    while (!admitted_records.empty()) {
        auto record = std::move(admitted_records.front());
        admitted_records.pop();
        v2::OriginalRecordId record_id{
            static_cast<uint64_t>(ledger.original_records.size()) + 1};
        ledger.original_records.push_back({record_id, record});
        if (record.empty()) {
            ledger.milestones.accepted.completed_empty_records.push_back(record_id);
            ledger.milestones.air_acknowledged.completed_empty_records.push_back(record_id);
        } else {
            v2::OriginalRecordRange range{
                record_id, 0, static_cast<uint64_t>(record.size())};
            ledger.milestones.accepted.ranges.push_back(range);
            ledger.milestones.air_acknowledged.ranges.push_back(range);
        }
        native_repack_rx_records_.push(std::move(record));
    }
    repack_engaged_ = true;
    if (repack_remote_call_.empty())
        repack_remote_call_ = ax25_session_.remote_callsign();
    const auto admitted_transfer = ledger.transfer;
    repack_rx_drain();
    if (!repack_rx_transfer_ledger_ ||
        repack_rx_transfer_ledger_->terminal_published ||
        !same_transfer(repack_rx_transfer_ledger_->transfer,
                       admitted_transfer))
        return false;
    repack_update_rx_backpressure(false);
    return true;
}

void Modem::repack_rx_drain() {
    if (!rx_callback_ || repack_reorig_peer_busy_) return;
    Ax25Address dst = ax25_make_addr(config_.callsign);
    Ax25Address src = ax25_make_addr(ax25_session_.remote_callsign());
    while (!native_repack_rx_records_.empty()) {
        uint8_t outstanding = (repack_reorig_vs_ - repack_reorig_va_) & 0x07;
        if (outstanding >= REPACK_REORIG_WINDOW) break;   // pace to pump ahead-window
        uint8_t seq = repack_reorig_vs_;
        auto rec = std::move(native_repack_rx_records_.front());
        native_repack_rx_records_.pop();
        if (!repack_rx_transfer_ledger_ ||
            repack_rx_schedule_record_index_ >=
                repack_rx_transfer_ledger_->original_records.size()) {
            finish_transfer_once(v2::TransferResultReason::MalformedTransportState);
            return;
        }
        const auto record_id = repack_rx_transfer_ledger_
            ->original_records[repack_rx_schedule_record_index_].record_id;
        repack_reorig_window_[seq] = std::move(rec);
        repack_reorig_window_active_[seq] = true;
        repack_reorig_window_record_id_[seq] = record_id;
        v2::OriginalRecordCoverage mapping;
        if (repack_reorig_window_[seq].empty())
            mapping.completed_empty_records.push_back(record_id);
        else
            mapping.ranges.push_back({
                record_id, 0,
                static_cast<uint64_t>(repack_reorig_window_[seq].size())});
        v2::ClientRecordSequenceMapping sequence_mapping;
        sequence_mapping.absolute_position =
            repack_rx_transfer_ledger_->r3_sequence.endpoint_cursor
                .next_absolute_position +
            ((repack_reorig_vs_ - repack_reorig_va_) & 0x07);
        sequence_mapping.transfer = repack_rx_transfer_ledger_->transfer;
        const auto transfer = sequence_mapping.transfer;
        sequence_mapping.original_mapping = std::move(mapping);
        sequence_mapping.client_payload_byte_count =
            repack_reorig_window_[seq].size();
        repack_rx_transfer_ledger_->r3_sequence.owned_mapping.push_back(
            std::move(sequence_mapping));
        ++repack_rx_schedule_record_index_;
        repack_reorig_vs_ = (repack_reorig_vs_ + 1) & 0x07;
        // Re-originate the EXACT original record as a clean I-frame FROM the peer TO
        // the local pump.  Ownership and V(S) advance before the callback so an
        // immediate reentrant RR cannot make this record appear unscheduled and
        // deliver it twice.
        const auto bytes = repack_reorig_window_[seq];
        auto iframe = ax25_build_i(dst, src, seq, 0, false,
                                   AX25_PID_NONE, bytes.data(), bytes.size());
        rx_callback_(iframe.data(), iframe.size());
        if (!repack_rx_transfer_ledger_ ||
            !same_transfer(repack_rx_transfer_ledger_->transfer, transfer))
            return;
    }
    uint8_t outstanding = (repack_reorig_vs_ - repack_reorig_va_) & 0x07;
    ax25_session_.set_own_busy(outstanding >= REPACK_REORIG_WINDOW);
}

void Modem::repack_update_rx_backpressure(bool announce) {
    size_t outstanding_bytes = native_repack_rx_stream_.size();
    auto pending = native_repack_rx_records_;
    while (!pending.empty()) {
        outstanding_bytes += REPACK_LEDGER_RECORD_CHARGE + pending.front().size();
        pending.pop();
    }
    for (size_t i = 0; i < repack_reorig_window_.size(); ++i) {
        if (repack_reorig_window_active_[i])
            outstanding_bytes +=
                REPACK_LEDGER_RECORD_CHARGE + repack_reorig_window_[i].size();
    }
    const uint8_t outstanding =
        (repack_reorig_vs_ - repack_reorig_va_) & 0x07;
    const bool window_full = outstanding >= REPACK_REORIG_WINDOW;
    bool desired = ax25_session_.own_busy();
    if (window_full || outstanding_bytes >= REPACK_RX_HIGH_WATER)
        desired = true;
    else if (outstanding_bytes < REPACK_RX_LOW_WATER)
        desired = false;
    if (announce)
        ax25_session_.set_own_busy(desired);
    else
        ax25_session_.set_own_busy_silent(desired);
}

void Modem::repack_retry_r3_window() {
    if (!rx_callback_ || repack_reorig_peer_busy_)
        return;
    uint8_t seq = repack_reorig_va_;
    const uint8_t limit = repack_reorig_vs_;
    while (seq != limit) {
        if (repack_reorig_window_active_[seq]) {
            const auto bytes = repack_reorig_window_[seq];
            const auto transfer = repack_rx_transfer_ledger_->transfer;
            auto iframe = ax25_build_i(
                ax25_make_addr(config_.callsign),
                ax25_make_addr(repack_remote_call_), seq, 0, false,
                AX25_PID_NONE, bytes.data(), bytes.size());
            rx_callback_(iframe.data(), iframe.size());
            if (!repack_rx_transfer_ledger_ ||
                !same_transfer(repack_rx_transfer_ledger_->transfer, transfer))
                return;
        }
        seq = (seq + 1) & 0x07;
    }
}

void Modem::repack_custody_teardown(
    const char* why, v2::TransferResultReason reason) {
    // The air link died with custody-ACKed data undelivered -> DISC the local pump
    // so it never reports success on lost bytes (standard TNC/VARA custody).
    IRIS_LOG("[REPACK] custody teardown: %s (tx_buf=%zu rx_recs=%zu peer=%s)", why,
             native_repack_tx_stream_.size(), native_repack_rx_records_.size(),
             repack_remote_call_.c_str());
    if (repack_transfer_ledger_ &&
        !repack_transfer_ledger_->terminal_published) {
        finish_transfer_once(reason);
        return;
    }
    if (repack_rx_transfer_ledger_ &&
        !repack_rx_transfer_ledger_->terminal_published) {
        finish_transfer_once(reason);
        return;
    }
    auto callback = rx_callback_;
    std::vector<uint8_t> terminal_frame;
    if (callback && !repack_remote_call_.empty()) {
        Ax25Address dst = ax25_make_addr(config_.callsign);
        Ax25Address src = ax25_make_addr(repack_remote_call_);
        terminal_frame = ax25_build_u(dst, src, AX25_CTRL_DISC, true, true);
    }
    const std::string remote_call = repack_remote_call_;
    const bool publish_ax25_disconnected =
        ax25_session_.state() != Ax25SessionState::DISCONNECTED;
    auto ax25_state_callback = ax25_state_callback_;
    disconnect_timeout_ticks_ = 0;
    ax25_session_.reset_silent();
    repack_reset();
    if (callback && !terminal_frame.empty())
        callback(terminal_frame.data(), terminal_frame.size());
    if (publish_ax25_disconnected && ax25_state_callback &&
        ax25_session_.state() == Ax25SessionState::DISCONNECTED)
        ax25_state_callback(Ax25SessionState::DISCONNECTED, remote_call);
}

bool Modem::test_repack_pack_split_roundtrip() {
    // P0 integrity: records of varied sizes -> one stream -> split under VARYING
    // fragmentation (simulates the dynamic OFDM frame size + a mid-stream change,
    // the Step-0 SHA-mismatch root) -> assert BIT-EXACT recovery + bad-length LOUD.
    std::vector<std::vector<uint8_t>> records;
    const size_t sizes[] = {1, 74, 74, 30, 74, 200, 3, 556, 1024, 55, 74, 2, 400};
    uint8_t seed = 7;
    for (size_t s : sizes) {
        std::vector<uint8_t> r(s);
        for (size_t i = 0; i < s; i++) { seed = (uint8_t)(seed * 31 + 11 + i); r[i] = seed; }
        records.push_back(std::move(r));
    }
    std::vector<uint8_t> stream;
    for (auto& r : records) repack_pack_record(stream, r.data(), r.size());
    // Fragment at varying sizes (dynamic frame size + mid-stream change).
    const size_t frag[] = {74, 74, 200, 556, 3, 1024, 17, 400, 74, 94};
    const size_t nfrag = sizeof(frag) / sizeof(frag[0]);
    std::queue<std::vector<uint8_t>> out;
    std::vector<uint8_t> rxbuf;
    size_t pos = 0, fi = 0;
    while (pos < stream.size()) {
        size_t take = std::min(frag[fi % nfrag], stream.size() - pos);
        fi++;
        rxbuf.insert(rxbuf.end(), stream.begin() + pos, stream.begin() + pos + take);
        pos += take;
        if (repack_split_records(rxbuf, out) < 0) {
            IRIS_LOG("[TEST-REPACK] FAIL: unexpected bad-length during round-trip");
            return false;
        }
    }
    if (!rxbuf.empty()) {
        IRIS_LOG("[TEST-REPACK] FAIL: %zu residual bytes (incomplete split)", rxbuf.size());
        return false;
    }
    if (out.size() != records.size()) {
        IRIS_LOG("[TEST-REPACK] FAIL: recovered %zu records, expected %zu",
                 out.size(), records.size());
        return false;
    }
    for (size_t i = 0; i < records.size(); i++) {
        if (out.front() != records[i]) {
            IRIS_LOG("[TEST-REPACK] FAIL: record %zu bit-mismatch (len %zu)", i, records[i].size());
            return false;
        }
        out.pop();
    }
    // Bad-length: a prefix > REPACK_MAX_RECORD must be rejected LOUD (-1), never a
    // bogus record (P0 — silent data corruption is the failure this guards).
    std::vector<uint8_t> bad{0xFF, 0xFF};   // len = 65535 > REPACK_MAX_RECORD
    bad.resize(12, 0xAA);
    std::queue<std::vector<uint8_t>> tmp;
    if (repack_split_records(bad, tmp) != -1) {
        IRIS_LOG("[TEST-REPACK] FAIL: bad length (65535) not rejected");
        return false;
    }
    IRIS_LOG("[TEST-REPACK] PASS: %zu records round-tripped BIT-EXACT under varying "
             "fragmentation; bad-length rejected", records.size());
    return true;
}

bool Modem::test_repack_refragment_on_demote() {
    // Crossover recovery (Fix B, INV-3): a mid-transfer anchor demote shrinks max_info
    // below already-packed/in-flight frames.  FAIL-BEFORE = drop_oversized_in_window
    // DESTROYS those owned-transport bytes -> incomplete transfer.  PASS-AFTER =
    // recover_unacked_stream returns them LOSSLESS + in order, so re-fragmentation at
    // the smaller MTU is bit-exact.  Drives the ax25 recover/refragment core directly.
    const std::string ME = "N0AAA", PEER = "N0BBB";
    Ax25Address me = ax25_make_addr(ME), peer = ax25_make_addr(PEER);

    // A known [len]INFO record stream (the R2 payload), records of varied sizes incl.
    // ones larger than the demoted MTU (so they MUST re-fragment).
    std::vector<std::vector<uint8_t>> records;
    const size_t sizes[] = {74, 74, 200, 30, 74, 400, 3, 556, 74, 120, 74, 512};
    uint8_t seed = 3;
    for (size_t s : sizes) {
        std::vector<uint8_t> r(s);
        for (size_t i = 0; i < s; i++) { seed = (uint8_t)(seed * 31 + 7 + i); r[i] = seed; }
        records.push_back(std::move(r));
    }
    std::vector<uint8_t> stream;
    for (auto& r : records) repack_pack_record(stream, r.data(), r.size());

    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign(ME);
        s.set_send_callback([](const uint8_t*, size_t) {});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f); s.on_frame_received(f);
        s.set_native_active(true);
    };

    const int BIG = 556;   // O5-class anchor MTU
    const int SMALL = 74;  // O0-class demoted MTU

    // --- FAIL-BEFORE arm: drop_oversized_in_window loses the un-ACKed bytes ---------
    {
        Ax25Session s; bring_up(s);
        s.set_max_info(BIG);
        s.send_data(stream.data(), stream.size());   // window fills to K; rest queued
        int outstanding_before = s.window_used();
        s.set_max_info(SMALL);
        s.drop_oversized_in_window();                 // the pre-fix path
        // Every BIG-sized window frame exceeded SMALL -> dropped; V(S) reset to V(A).
        // Those owned-transport bytes are GONE (the client already custody-completed),
        // so the transfer would ship INCOMPLETE.  Assert the loss is real.
        if (outstanding_before == 0 || s.window_used() != 0) {
            IRIS_LOG("[TEST-REFRAG] FAIL: drop-path setup wrong (out=%d used=%d)",
                     outstanding_before, s.window_used());
            return false;
        }
    }

    // --- PASS-AFTER arm: recover_unacked_stream preserves 100% of the bytes ----------
    {
        Ax25Session s; bring_up(s);
        s.set_max_info(BIG);
        s.send_data(stream.data(), stream.size());
        s.set_max_info(SMALL);
        std::vector<uint8_t> recovered;
        s.recover_unacked_stream(recovered);          // the fix path
        // (1) lossless + in order == the exact un-ACKed stream (nothing ACKed yet).
        if (recovered != stream) {
            IRIS_LOG("[TEST-REFRAG] FAIL: recovered %zu B != stream %zu B (byte loss)",
                     recovered.size(), stream.size());
            return false;
        }
        // (2) the window is rolled back (V(S)=V(A)) ready to re-drive at the new size.
        if (s.window_used() != 0 || s.vs() != s.va()) {
            IRIS_LOG("[TEST-REFRAG] FAIL: window not rolled back (used=%d vs=%d va=%d)",
                     s.window_used(), s.vs(), s.va());
            return false;
        }
        // (3) re-fragment at the SMALL MTU and reassemble by [len] -> bit-exact records
        //     (the whole point: a smaller MTU re-slices the SAME stream losslessly).
        std::queue<std::vector<uint8_t>> out;
        std::vector<uint8_t> rxbuf;
        size_t off = 0;
        while (off < recovered.size()) {
            size_t take = std::min((size_t)SMALL, recovered.size() - off);
            rxbuf.insert(rxbuf.end(), recovered.begin() + off, recovered.begin() + off + take);
            off += take;
            if (repack_split_records(rxbuf, out) < 0) {
                IRIS_LOG("[TEST-REFRAG] FAIL: bad-length during re-fragment reassembly");
                return false;
            }
        }
        if (!rxbuf.empty() || out.size() != records.size()) {
            IRIS_LOG("[TEST-REFRAG] FAIL: reassembled %zu records (+%zu residual), expected %zu",
                     out.size(), rxbuf.size(), records.size());
            return false;
        }
        for (size_t i = 0; i < records.size(); i++) {
            if (out.front() != records[i]) {
                IRIS_LOG("[TEST-REFRAG] FAIL: record %zu bit-mismatch after re-fragment", i);
                return false;
            }
            out.pop();
        }
    }

    IRIS_LOG("[TEST-REFRAG] PASS: drop-path loses un-ACKed bytes; recover_unacked_stream "
             "preserves ALL %zu B and re-slices %zu records BIT-EXACT at the smaller MTU",
             stream.size(), records.size());
    return true;
}

bool Modem::test_anchor_futility_demote() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Anchor-futility demote (data-flow-tx-anchor.md §4). Drives the PRODUCTION
    // resolve_ofdm_tx_level. Scenario = the measured wedge: anchor O5 (credited
    // by smaller frames), full-MTU retransmissions never ACKed, no-ACK demote
    // bottoming at the anchor every round. FAIL-BEFORE: the anchor is a
    // permanent floor (observed 53-84 consecutive clamped demotes, terminal
    // T1 N2 disconnect). PASS-AFTER: the third consecutive bottomed DATA round
    // demotes the anchor by one, shrinks MAX_INFO, and defers the window
    // re-fragment to the idle TX tick.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    demote_refine_ = true;
    reset_level_state();
    const int ANCHOR = 5;
    tx_acked_level_ = ANCHOR;
    ax25_session_.set_max_info(ofdm_max_info_for_level(ANCHOR, ofdm_config_));
    gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
    gearshift_.force_ofdm_level(ANCHOR);
    peer_snr_db_ = 0;
    tx_proposed_level_ = -1;
    int max_info_before = ax25_session_.max_info();

    // (a) CONTROL batches bottomed at the anchor must NOT count as futility
    //     (polls between data retransmissions would otherwise inflate it).
    tx_no_ack_count_ = 2;
    for (int i = 0; i < 5; i++)
        resolve_ofdm_tx_level(/*kiss_control=*/true, /*batch_control=*/false, 20);
    if (ofdm_anchor_futility_ != 0 || tx_acked_level_ != ANCHOR) {
        IRIS_LOG("[TEST-FUTIL] FAIL (a): control batches counted (futil=%d anchor=%d)",
                 ofdm_anchor_futility_, tx_acked_level_);
        return false;
    }

    // (b) DATA rounds 1..LIMIT-1: counter rises, anchor still clamped (the
    //     fail-before behavior is the clamp itself — assert it holds until
    //     the evidence threshold, not a single lost frame).
    for (int i = 1; i < OFDM_ANCHOR_FUTILITY_LIMIT; i++) {
        resolve_ofdm_tx_level(false, false, 500);
        if (ofdm_anchor_futility_ != i || tx_acked_level_ != ANCHOR ||
            ofdm_refrag_pending_) {
            IRIS_LOG("[TEST-FUTIL] FAIL (b): round %d futil=%d anchor=%d pending=%d",
                     i, ofdm_anchor_futility_, tx_acked_level_, (int)ofdm_refrag_pending_);
            return false;
        }
    }

    // (c) an N(R)-advance style reset clears the evidence (forward progress).
    ofdm_anchor_futility_ = 0;
    resolve_ofdm_tx_level(false, false, 500);
    if (ofdm_anchor_futility_ != 1 || tx_acked_level_ != ANCHOR) {
        IRIS_LOG("[TEST-FUTIL] FAIL (c): reset not honored (futil=%d anchor=%d)",
                 ofdm_anchor_futility_, tx_acked_level_);
        return false;
    }

    // (d) rounds to the LIMIT: the anchor demotes ONE rung, MAX_INFO shrinks,
    //     the window mutation is DEFERRED, and the resolved level follows the
    //     new floor.
    int lvl = -1;
    for (int i = ofdm_anchor_futility_; i < OFDM_ANCHOR_FUTILITY_LIMIT; i++) {
        resolve_ofdm_tx_level(false, false, 500);
        lvl = ofdm_speed_level_;
    }
    if (tx_acked_level_ != ANCHOR - 1 || !ofdm_refrag_pending_ ||
        ofdm_anchor_futility_ != 0 ||
        ax25_session_.max_info() != ofdm_max_info_for_level(ANCHOR - 1, ofdm_config_) ||
        ax25_session_.max_info() >= max_info_before || lvl != ANCHOR - 1) {
        IRIS_LOG("[TEST-FUTIL] FAIL (d): anchor=%d pending=%d futil=%d max_info=%d lvl=%d",
                 tx_acked_level_, (int)ofdm_refrag_pending_, ofdm_anchor_futility_,
                 ax25_session_.max_info(), lvl);
        return false;
    }

    // (e) the deferred mutation is consumed exactly once and clears the Fix B
    //     fit-floor (the stuck frame it was computed for no longer exists at
    //     its old size).
    ofdm_fit_floor_level_ = ANCHOR;
    ofdm_reject_streak_ = 7;
    consume_pending_window_refrag();
    if (ofdm_refrag_pending_ || ofdm_fit_floor_level_ != -1 || ofdm_reject_streak_ != 0) {
        IRIS_LOG("[TEST-FUTIL] FAIL (e): pending=%d floor=%d streak=%d",
                 (int)ofdm_refrag_pending_, ofdm_fit_floor_level_, ofdm_reject_streak_);
        return false;
    }

    // (f) repeat futility at the new floor keeps walking down (terminates at O0,
    //     never wedges): three more bottomed data rounds -> anchor O3.
    gearshift_.force_ofdm_level(ANCHOR - 1);
    for (int i = 0; i < OFDM_ANCHOR_FUTILITY_LIMIT; i++)
        resolve_ofdm_tx_level(false, false, 400);
    if (tx_acked_level_ != ANCHOR - 2) {
        IRIS_LOG("[TEST-FUTIL] FAIL (f): second demote missing (anchor=%d)", tx_acked_level_);
        return false;
    }

    IRIS_LOG("[TEST-FUTIL] PASS: %d futile data rounds -> anchor O%d->O%d, MAX_INFO %d->%d, "
             "deferred re-fragment consumed, control rounds ignored",
             OFDM_ANCHOR_FUTILITY_LIMIT, ANCHOR, ANCHOR - 1,
             max_info_before, ofdm_max_info_for_level(ANCHOR - 1, ofdm_config_));
    return true;
}

bool Modem::test_shadow_desync_prevention() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Shadow-desync prevention (data-flow-owned-shadow-seq.md §7).  Replays the
    // captured owned-sequence crossover from the fast-WGN:40 teardown specimens
    // (s00/s02): wide window K=63, a 24-frame in-flight span, an anchor-futility
    // demote that shrinks K to 7 UNDER the flight, and the honest cumulative RR
    // racing the deferred re-fragment.  FAIL-BEFORE (three ways): the RR is
    // silently refused at fwd_dist=24 > K=7 (leg A), the idle-tick harvest rolls
    // V(S) back across the peer-counted numbers so every later honest N(R) reads
    // invalid -> FRMR/re-establish (leg B), and the futility rounds themselves
    // fire spuriously inside the flight's own ACK latency (leg C).
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    v2_negotiated_active_ = true;
    demote_refine_ = true;
    repack_refragment_ = true;
    wide_window_ = true;
    gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);

    const std::string ME = "N0AAA", PEER = "N0BBB";
    const std::string saved_callsign = config_.callsign;
    config_.callsign = ME;
    Ax25Address me = ax25_make_addr(ME), peer = ax25_make_addr(PEER);
    const int ANCHOR = 5;
    const int FLIGHT = 24;                                 // the specimen flight

    // Bring the MEMBER session up as KISS-managed + native + CONNECTED with the
    // wide owned window and a 24-frame un-ACKed flight, plus queued stream bytes.
    auto bring_up_with_flight = [&]() {
        reset_level_state();
        repack_reset();
        ofdm_anchor_futility_ = 0;
        tx_no_ack_count_ = 0;
        while (!tx_queue_.empty()) tx_queue_.pop();
        ax25_session_.reset();
        ax25_session_.set_local_callsign(ME);
        ax25_session_.set_send_callback([](const uint8_t*, size_t) {});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        ax25_session_.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        ax25_session_.on_frame_received(f);
        ax25_session_.set_native_active(true);
        ax25_session_.set_wide_window(true, 63);
        tx_acked_level_ = ANCHOR;
        gearshift_.force_ofdm_level(ANCHOR);
        peer_snr_db_ = 0;
        tx_proposed_level_ = -1;
        ax25_session_.set_max_info(ofdm_max_info_for_level(ANCHOR, ofdm_config_));
        // EFFECTIVE MTU: set_max_info reserves 1 B for the extended (modulo-128)
        // control field, so ask the session (555 at O5 wide), don't recompute.
        const int mtu = ax25_session_.max_info();
        std::vector<uint8_t> flight_bytes((size_t)FLIGHT * mtu);
        for (size_t i = 0; i < flight_bytes.size(); i++)
            flight_bytes[i] = (uint8_t)(i * 31 + 7);
        ax25_session_.send_data(flight_bytes.data(), flight_bytes.size());
        // Queued (never-aired) stream bytes behind the flight.
        native_repack_tx_stream_.assign((size_t)6 * mtu, 0x5A);
        repack_engaged_ = true;
    };
    auto demote_and_shrink = [&]() {
        // The production futility demote: defer the window mutation, shrink
        // MAX_INFO, then the next drain shrinks K 63 -> 7 under the flight
        // (repack_tx_drain's steady-state gate: tx_acked < max_proposable).
        demote_tx_anchor("test: anchor futility", /*defer_window_mutation=*/true);
        ax25_session_.set_window_k(Ax25Session::K_WINDOW);
    };

    // ---- Leg A (INV-SEQ-1): the honest wide ACK beats the harvest (s00/s02) ----
    bring_up_with_flight();
    if (!native_repack_active() || ax25_session_.window_used() != FLIGHT ||
        ax25_session_.vs() != FLIGHT) {
        IRIS_LOG("[TEST-DESYNC] FAIL (setup): repack=%d used=%d vs=%d",
                 (int)native_repack_active(), ax25_session_.window_used(),
                 ax25_session_.vs());
        config_.callsign = saved_callsign;
        return false;
    }
    demote_and_shrink();
    ax25_session_.inject_rx_rr(FLIGHT, false);   // RR N(R)=24, fwd_dist=24 > K=7
    if (ax25_session_.va() != FLIGHT) {
        IRIS_LOG("[TEST-DESYNC] FAIL (A): honest cumulative RR N(R)=%d REFUSED under "
                 "K-shrink (V(A)=%d, want %d) — INV-SEQ-1 violated",
                 FLIGHT, ax25_session_.va(), FLIGHT);
        config_.callsign = saved_callsign;
        return false;
    }
    consume_pending_window_refrag();   // window credited -> harvest queued-only
    if (ofdm_refrag_pending_ || ax25_session_.va() != FLIGHT ||
        ax25_session_.iframe_hold()) {
        IRIS_LOG("[TEST-DESYNC] FAIL (A2): refrag not consumed cleanly "
                 "(pending=%d va=%d hold=%d)", (int)ofdm_refrag_pending_,
                 ax25_session_.va(), (int)ax25_session_.iframe_hold());
        config_.callsign = saved_callsign;
        return false;
    }
    // Re-sliced NEW frames must take FRESH numbers (>= 24), never reuse 0..23.
    if (ax25_session_.window_used() > 0 &&
        ((int)ax25_session_.vs() - (int)ax25_session_.va() + 128) % 128 > 7) {
        IRIS_LOG("[TEST-DESYNC] FAIL (A3): re-slice overfilled the K=7 window "
                 "(va=%d vs=%d)", ax25_session_.va(), ax25_session_.vs());
        config_.callsign = saved_callsign;
        return false;
    }
    for (int i = 0; i < 3; i++)
        ax25_session_.inject_rx_rr(FLIGHT, false);   // honest dups: never a desync
    if (ax25_session_.state() != Ax25SessionState::CONNECTED) {
        IRIS_LOG("[TEST-DESYNC] FAIL (A4): honest dup RRs tore the session down");
        config_.callsign = saved_callsign;
        return false;
    }

    // ---- Leg B (INV-SEQ-2): the idle tick fires FIRST (the pure Step-B race) ----
    bring_up_with_flight();
    demote_and_shrink();
    consume_pending_window_refrag();   // pre-fix: harvests NOW, V(S) 24 -> 0
    if (ax25_session_.vs() != FLIGHT || !ofdm_refrag_pending_ ||
        !ax25_session_.iframe_hold()) {
        IRIS_LOG("[TEST-DESYNC] FAIL (B): harvest did not HOLD under an un-ACKed "
                 "flight (vs=%d pending=%d hold=%d) — V(S) rolled back across "
                 "peer-countable numbers", ax25_session_.vs(),
                 (int)ofdm_refrag_pending_, (int)ax25_session_.iframe_hold());
        config_.callsign = saved_callsign;
        return false;
    }
    consume_pending_window_refrag();   // still no ACK resolution -> still held
    if (!ofdm_refrag_pending_) {
        IRIS_LOG("[TEST-DESYNC] FAIL (B2): hold released without ACK resolution");
        config_.callsign = saved_callsign;
        return false;
    }
    ax25_session_.inject_rx_rr(FLIGHT, false);   // the late honest ACK lands
    consume_pending_window_refrag();             // resolves via V(A)==V(S)
    if (ofdm_refrag_pending_ || ax25_session_.va() != FLIGHT) {
        IRIS_LOG("[TEST-DESYNC] FAIL (B3): late ACK did not resolve the hold "
                 "(pending=%d va=%d)", (int)ofdm_refrag_pending_, ax25_session_.va());
        config_.callsign = saved_callsign;
        return false;
    }
    for (int i = 0; i < 3; i++)
        ax25_session_.inject_rx_rr(FLIGHT, false);
    if (ax25_session_.state() != Ax25SessionState::CONNECTED) {
        IRIS_LOG("[TEST-DESYNC] FAIL (B4): desync fired after a held+resolved refrag");
        config_.callsign = saved_callsign;
        return false;
    }

    // ---- Leg B2 (INV-SEQ-2 quiesce): peer provably stuck -> rollback is SAFE ----
    bring_up_with_flight();
    demote_and_shrink();
    consume_pending_window_refrag();             // arms the quiesce clock
    ax25_session_.inject_rx_rr(0, false);        // frozen N(R) BEFORE the drain window
    consume_pending_window_refrag();
    if (!ofdm_refrag_pending_) {
        IRIS_LOG("[TEST-DESYNC] FAIL (Q1): pre-drain frozen N(R) resolved the hold "
                 "(the peer could still be decoding the flight)");
        config_.callsign = saved_callsign;
        return false;
    }
    for (int i = 0; i <= REFRAG_QUIESCE_TICKS; i++) ax25_session_.tick();
    consume_pending_window_refrag();             // drained, but no POST-drain N(R) yet
    if (!ofdm_refrag_pending_) {
        IRIS_LOG("[TEST-DESYNC] FAIL (Q2): hold released without a post-drain N(R)");
        config_.callsign = saved_callsign;
        return false;
    }
    ax25_session_.inject_rx_rr(0, false);        // post-drain: the peer's FINAL V(R)
    consume_pending_window_refrag();             // quiesce-resolved -> safe rollback
    if (ofdm_refrag_pending_ || ax25_session_.va() != 0 ||
        ax25_session_.iframe_hold()) {
        IRIS_LOG("[TEST-DESYNC] FAIL (Q3): quiesce did not resolve (pending=%d va=%d "
                 "hold=%d)", (int)ofdm_refrag_pending_, ax25_session_.va(),
                 (int)ax25_session_.iframe_hold());
        config_.callsign = saved_callsign;
        return false;
    }

    // ---- Leg C (INV-SEQ-3): futility rounds inside the flight's ACK latency ----
    bring_up_with_flight();
    tx_no_ack_count_ = 2;
    for (int i = 0; i < OFDM_ANCHOR_FUTILITY_LIMIT + 2; i++)
        resolve_ofdm_tx_level(false, false, 500);
    if (ofdm_anchor_futility_ != 0 || tx_acked_level_ != ANCHOR) {
        IRIS_LOG("[TEST-DESYNC] FAIL (C): futility counted inside the flight's own "
                 "ACK latency (futil=%d anchor=%d) — the spurious demote that armed "
                 "the crossover", ofdm_anchor_futility_, tx_acked_level_);
        config_.callsign = saved_callsign;
        return false;
    }
    // Expire the ACK clock (24 frames x 1.2 s) -> genuine futility still demotes.
    for (int i = 0; i <= OFDM_FUTILITY_TICKS_PER_OUTSTANDING * FLIGHT; i++)
        ax25_session_.tick();
    gearshift_.force_ofdm_level(ANCHOR);
    for (int i = 0; i < OFDM_ANCHOR_FUTILITY_LIMIT; i++)
        resolve_ofdm_tx_level(false, false, 500);
    if (tx_acked_level_ != ANCHOR - 1 || !ofdm_refrag_pending_) {
        IRIS_LOG("[TEST-DESYNC] FAIL (C2): genuine futility no longer demotes "
                 "(anchor=%d pending=%d)", tx_acked_level_, (int)ofdm_refrag_pending_);
        config_.callsign = saved_callsign;
        return false;
    }

    // Leave the member session clean for any later in-process test.
    ofdm_refrag_pending_ = false;
    refrag_quiesce_start_tick_ = -1;
    ax25_session_.set_iframe_hold(false);
    ax25_session_.reset();
    repack_reset();
    reset_level_state();
    config_.callsign = saved_callsign;
    IRIS_LOG("[TEST-DESYNC] PASS: wide ACK accepted under K-shrink; harvest held "
             "until ACK resolution (race + quiesce legs); futility honest at wide K");
    return true;
}

bool Modem::test_rej_inflight_deferred() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // DATALINK_INTEGRITY_AUDIT §4/F2.  A peer REJ/RNR arriving mid-flight at wide K
    // must NOT roll V(S) back inline: the sniff runs BEFORE on_frame_received, so an
    // inline harvest rewinds V(S) to the PRE-ack V(A) and re-uses peer-counted seq
    // numbers (the s00/s02 crossover class).  Fail-before (defer=false at the REJ
    // sniff): dispatch_rx_frame(REJ) rolls V(S) 12->0 inline and the REJ's own N(R)
    // reads invalid.  Pass-after (defer=true): the anchor + MAX_INFO still shrink NOW,
    // but the re-fragment is HELD (INV-SEQ-2 quiesce) — V(S) is intact, V(A) advances
    // via the REJ's N(R), and the flight resolves before any re-slice.  Leg B drives
    // the receiver-side re-slice OVERWRITE (a stale old-slicing reorder entry must not
    // win over its re-sliced replacement).
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    v2_negotiated_active_ = true;
    demote_refine_ = true;
    repack_refragment_ = true;
    wide_window_ = true;
    gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);

    const std::string ME = "N0AAA", PEER = "N0BBB";
    const std::string saved_callsign = config_.callsign;
    config_.callsign = ME;
    Ax25Address me = ax25_make_addr(ME), peer = ax25_make_addr(PEER);
    const int ANCHOR = 5;
    const int FLIGHT = 12;                 // > K=7: a genuine wide in-flight span

    auto bring_up_with_flight = [&]() {
        reset_level_state();
        repack_reset();
        ofdm_anchor_futility_ = 0;
        tx_no_ack_count_ = 0;
        while (!tx_queue_.empty()) tx_queue_.pop();
        ax25_session_.reset();
        ax25_session_.set_local_callsign(ME);
        ax25_session_.set_send_callback([](const uint8_t*, size_t) {});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        ax25_session_.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        ax25_session_.on_frame_received(f);
        ax25_session_.set_native_active(true);
        ax25_session_.set_wide_window(true, 63);
        tx_acked_level_ = ANCHOR;
        gearshift_.force_ofdm_level(ANCHOR);
        peer_snr_db_ = 0;
        tx_proposed_level_ = -1;
        ax25_session_.set_max_info(ofdm_max_info_for_level(ANCHOR, ofdm_config_));
        const int mtu = ax25_session_.max_info();
        std::vector<uint8_t> flight_bytes((size_t)FLIGHT * mtu);
        for (size_t i = 0; i < flight_bytes.size(); i++)
            flight_bytes[i] = (uint8_t)(i * 31 + 7);
        ax25_session_.send_data(flight_bytes.data(), flight_bytes.size());
        native_repack_tx_stream_.assign((size_t)6 * mtu, 0x5A);
        repack_engaged_ = true;
        // Mark re-pack ALREADY activated (production engages once at OFDM activation,
        // BEFORE any owned frame flies). Without this, dispatch_rx_frame's
        // repack_maybe_engage() would engage NOW and roll V(S)->V(A) itself — a
        // test-setup artifact that masks what the REJ demote does.
        repack_activated_ = true;
        repack_remote_call_ = PEER;
    };

    // ---- Leg A: REJ mid-flight defers — V(S) stays put, anchor still demotes ----
    bring_up_with_flight();
    if (!native_repack_active() || ax25_session_.vs() != FLIGHT ||
        ax25_session_.va() != 0) {
        IRIS_LOG("[TEST-REJ] FAIL (setup): repack=%d vs=%d va=%d",
                 (int)native_repack_active(), ax25_session_.vs(), ax25_session_.va());
        config_.callsign = saved_callsign;
        return false;
    }
    // A peer REJ with N(R)=4 (partial ack), extended (wide-tier 2-octet control),
    // driven through the REAL production sniff at dispatch_rx_frame (from_ofdm).
    {
        const uint8_t REJ_NR = 4;
        auto rej = ax25_build_s(me, peer, Ax25SType::REJ, REJ_NR, /*pf*/false,
                                /*command*/true, /*extended*/true);
        dedup_cooldown_ = 0;
        dispatch_rx_frame(rej, /*from_fx25*/false, /*from_ofdm*/true);
        // V(S) MUST NOT have rolled back; the anchor + MAX_INFO MUST have shrunk; the
        // re-fragment MUST be deferred+held; the REJ's own N(R) advanced V(A).
        if (ax25_session_.vs() != FLIGHT) {
            IRIS_LOG("[TEST-REJ] FAIL (A): V(S) rolled back inline on peer REJ "
                     "(vs=%d, want %d) — the F2 crossover", ax25_session_.vs(), FLIGHT);
            config_.callsign = saved_callsign;
            return false;
        }
        if (tx_acked_level_ != ANCHOR - 1) {
            IRIS_LOG("[TEST-REJ] FAIL (A2): peer REJ did not demote the anchor "
                     "(tx_acked_level=%d, want %d)", tx_acked_level_, ANCHOR - 1);
            config_.callsign = saved_callsign;
            return false;
        }
        if (!ofdm_refrag_pending_ || !ax25_session_.iframe_hold()) {
            IRIS_LOG("[TEST-REJ] FAIL (A3): REJ demote not deferred/held "
                     "(pending=%d hold=%d)", (int)ofdm_refrag_pending_,
                     (int)ax25_session_.iframe_hold());
            config_.callsign = saved_callsign;
            return false;
        }
        if (ax25_session_.va() != REJ_NR) {
            IRIS_LOG("[TEST-REJ] FAIL (A4): REJ N(R) did not advance V(A) "
                     "(va=%d, want %d)", ax25_session_.va(), REJ_NR);
            config_.callsign = saved_callsign;
            return false;
        }
    }
    // ---- Leg A2: the held flight resolves cleanly, no crossover/desync ----
    ax25_session_.inject_rx_rr(FLIGHT, false);   // peer eventually ACKs the whole flight
    consume_pending_window_refrag();             // V(A)==V(S) -> harvest + re-drain fresh
    // The hold releases, V(A) is credited to the full flight, and the re-slice re-
    // drains the queued stream into FRESH frames from V(A) (window_used <= K=7) —
    // NEVER re-using the peer-counted 0..11.  window_used <= K guarantees no crossover.
    if (ofdm_refrag_pending_ || ax25_session_.iframe_hold() ||
        ax25_session_.va() != FLIGHT || ax25_session_.window_used() > 7) {
        IRIS_LOG("[TEST-REJ] FAIL (A5): held refrag did not resolve cleanly "
                 "(pending=%d hold=%d va=%d vs=%d window_used=%d)",
                 (int)ofdm_refrag_pending_, (int)ax25_session_.iframe_hold(),
                 ax25_session_.va(), ax25_session_.vs(), ax25_session_.window_used());
        config_.callsign = saved_callsign;
        return false;
    }
    for (int i = 0; i < 3; i++)
        ax25_session_.inject_rx_rr(FLIGHT, false);   // honest dups: never a desync
    if (ax25_session_.state() != Ax25SessionState::CONNECTED) {
        IRIS_LOG("[TEST-REJ] FAIL (A6): a desync formed after the deferred REJ resolved");
        config_.callsign = saved_callsign;
        return false;
    }

    // ---- Leg B (F2 leg b): a re-sliced replacement OVERWRITES a stale reorder entry.
    // Selective-repeat receiver: N(S)=1 arrives out of order (buffered), then a
    // DIFFERENT-content N(S)=1 (the sender re-fragmented the un-ACKed window smaller
    // after the REJ) MUST overwrite it, so filling the gap delivers the NEW slicing.
    // Fail-before (the !rx_reorder_present_ guard): the replacement is REFUSED and the
    // stale old-slicing bytes drain -> a spliced byte stream (custody teardown/corrupt).
    {
        ax25_session_.reset();
        ax25_session_.set_local_callsign(ME);
        ax25_session_.set_send_callback([](const uint8_t*, size_t) {});
        std::vector<uint8_t> delivered;
        ax25_session_.set_native_stream_rx_callback(
            [&](const uint8_t* d, size_t n) { delivered.insert(delivered.end(), d, d + n); });
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        ax25_session_.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        ax25_session_.on_frame_received(f);
        ax25_session_.set_native_active(true);   // mod-8 SR window is fine for this leg

        std::vector<uint8_t> zero0(10, 0xC0), old1(20, 0xA1), new1(12, 0xB2);
        auto rx_i = [&](uint8_t ns, const std::vector<uint8_t>& info) {
            auto fr = ax25_build_i(me, peer, ns, 0, false, AX25_PID_NONE,
                                   info.data(), info.size(), false);
            Ax25Frame ff; ax25_parse(fr.data(), fr.size(), ff);
            ax25_session_.on_frame_received(ff);
        };
        rx_i(1, old1);   // out-of-order: buffered at slot 1 (old slicing)
        rx_i(1, new1);   // re-slice replacement at the SAME N(S): must OVERWRITE
        rx_i(0, zero0);  // fill the gap -> deliver 0, then DRAIN slot 1
        std::vector<uint8_t> want;
        want.insert(want.end(), zero0.begin(), zero0.end());
        want.insert(want.end(), new1.begin(), new1.end());
        if (delivered != want) {
            IRIS_LOG("[TEST-REJ] FAIL (B): re-sliced reorder entry not overwritten "
                     "(delivered %zu B, want %zu B) — stale slicing spliced into stream",
                     delivered.size(), want.size());
            config_.callsign = saved_callsign;
            return false;
        }
    }

    // Leave the member session clean for any later in-process test.
    ofdm_refrag_pending_ = false;
    refrag_quiesce_start_tick_ = -1;
    ax25_session_.set_iframe_hold(false);
    ax25_session_.set_native_stream_rx_callback(nullptr);
    ax25_session_.reset();
    repack_reset();
    reset_level_state();
    config_.callsign = saved_callsign;
    IRIS_LOG("[TEST-REJ] PASS: peer REJ mid-flight defers (V(S) intact, anchor demoted, "
             "flight resolves clean); re-sliced reorder entry overwrites the stale copy");
    return true;
}

bool Modem::test_native_desync_custody_teardown() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // DATALINK_INTEGRITY_AUDIT §4/F1.  When an unrecoverable N(R) desync fires in the
    // native/re-pack transport, the AX.25 FRMR+SABM re-establish is a DEAD END, so the
    // pre-fix path burned ~28 s in a zombie AWAITING_CONNECTION and then lost custody-
    // ACKed client data (s00: 91,288 B).  Pass-after: the desync hands off to
    // handle_native_desync(), which DISCs the local pump (custody surfaced, never a
    // silent success) and ends the session deterministically as DISCONNECTED.
    // Fail-before (handle_native_desync neutered to `return false`, i.e. the old
    // nr_error_recovery): the session lands in AWAITING_CONNECTION with NO pump DISC.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    v2_negotiated_active_ = true;
    repack_refragment_ = true;
    wide_window_ = true;
    gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);

    const std::string ME = "N0AAA", PEER = "N0BBB";
    const std::string saved_callsign = config_.callsign;
    config_.callsign = ME;
    Ax25Address me = ax25_make_addr(ME), peer = ax25_make_addr(PEER);
    const int ANCHOR = 5;
    const int FLIGHT = 10;

    // init() wires this in production; a bare test Modem does not run init(), so wire
    // the same desynchronization decision and sole DISCONNECTED completion hook.
    ax25_session_.set_desync_callback([this]() { return handle_native_desync(); });
    ax25_session_.set_state_callback(
        [this](Ax25SessionState state, const std::string&) {
            if (state != Ax25SessionState::DISCONNECTED)
                return;
            if (repack_has_open_custody()) {
                finish_transfer_once(repack_failure_reason_hint_);
            } else if (repack_engaged_ || !native_repack_tx_stream_.empty()) {
                repack_custody_teardown(
                    "native sequence desynchronization",
                    v2::TransferResultReason::ProtocolViolation);
            }
        });

    // Capture the custody signal surfaced to the local pump (a DISC U-frame).
    bool got_pump_disc = false;
    rx_callback_ = [&](const uint8_t* d, size_t n) {
        Ax25Frame f;
        if (ax25_parse(d, n, f) && f.type() == Ax25FrameType::U_FRAME &&
            f.u_type() == Ax25UType::DISC)
            got_pump_disc = true;
    };

    // Bring up native+repack CONNECTED with a wide in-flight window AND undelivered
    // custody bytes (both the un-drained stream and the un-ACKed owned window).
    reset_level_state();
    repack_reset();
    while (!tx_queue_.empty()) tx_queue_.pop();
    ax25_session_.reset();
    ax25_session_.set_local_callsign(ME);
    ax25_session_.set_send_callback([](const uint8_t*, size_t) {});
    {
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        ax25_session_.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        ax25_session_.on_frame_received(f);
    }
    ax25_session_.set_native_active(true);
    ax25_session_.set_wide_window(true, 63);
    tx_acked_level_ = ANCHOR;
    gearshift_.force_ofdm_level(ANCHOR);
    ax25_session_.set_max_info(ofdm_max_info_for_level(ANCHOR, ofdm_config_));
    const int mtu = ax25_session_.max_info();
    std::vector<uint8_t> flight_bytes((size_t)FLIGHT * mtu);
    for (size_t i = 0; i < flight_bytes.size(); i++) flight_bytes[i] = (uint8_t)(i * 17 + 3);
    ax25_session_.send_data(flight_bytes.data(), flight_bytes.size());
    native_repack_tx_stream_.assign((size_t)4 * mtu, 0x5A);   // un-drained custody
    repack_engaged_ = true;
    repack_remote_call_ = PEER;   // so the teardown DISCs the pump

    if (!native_repack_active() || ax25_session_.window_used() != FLIGHT ||
        native_repack_tx_stream_.empty()) {
        IRIS_LOG("[TEST-F1] FAIL (setup): repack=%d used=%d tx_buf=%zu",
                 (int)native_repack_active(), ax25_session_.window_used(),
                 native_repack_tx_stream_.size());
        config_.callsign = saved_callsign;
        return false;
    }

    // Force the desync: consecutive out-of-window N(R)s on the shadow S-frame path
    // (N(R)=100 is well beyond V(S)) until the streak trips (bounded to be robust to
    // the SHADOW_DESYNC_THRESHOLD value; it fires on the 3rd today).
    for (int i = 0; i < 8 && ax25_session_.state() == Ax25SessionState::CONNECTED; i++)
        ax25_session_.inject_rx_rr(100, false);

    // PASS-AFTER: the session ended DETERMINISTICALLY as DISCONNECTED (no dead T1/SABM
    // zombie in AWAITING_CONNECTION), custody was SURFACED to the pump (a DISC), and
    // the re-pack state was torn down (never a silent success on undelivered bytes).
    if (ax25_session_.state() != Ax25SessionState::DISCONNECTED) {
        IRIS_LOG("[TEST-F1] FAIL: desync did not end the session cleanly "
                 "(state=%d, want DISCONNECTED) — dead SABM/T1 zombie",
                 (int)ax25_session_.state());
        config_.callsign = saved_callsign;
        return false;
    }
    if (!got_pump_disc) {
        IRIS_LOG("[TEST-F1] FAIL: no custody DISC surfaced to the pump — undelivered "
                 "bytes lost SILENTLY");
        config_.callsign = saved_callsign;
        return false;
    }
    if (repack_engaged_ || !native_repack_tx_stream_.empty()) {
        IRIS_LOG("[TEST-F1] FAIL: re-pack custody state not torn down "
                 "(engaged=%d tx_buf=%zu)", (int)repack_engaged_,
                 native_repack_tx_stream_.size());
        config_.callsign = saved_callsign;
        return false;
    }

    ax25_session_.set_desync_callback(nullptr);
    ax25_session_.set_state_callback(nullptr);
    rx_callback_ = nullptr;
    ax25_session_.reset();
    repack_reset();
    reset_level_state();
    config_.callsign = saved_callsign;
    IRIS_LOG("[TEST-F1] PASS: native/re-pack desync -> immediate clean custody teardown "
             "(pump DISC, session DISCONNECTED, no SABM zombie, no silent loss)");
    return true;
}

bool Modem::test_chase_multiblock_and_flush() {
    // Chase-combining correctness (data-flow-tx-anchor.md §5).
    // D2: multi-block payload extraction. Build the DECODED data bits of a
    // 2-block frame exactly as the modulator frames them ([2B len][chunk]
    // [4B CRC-32] per block, padded to k bits) and assert the shared extractor
    // returns the FULL payload. FAIL-BEFORE: the old chase reconstruction
    // CRC-validated block 0 alone and returned a TRUNCATED payload as success
    // (reproduced inline below to pin the failure mode).
    const LdpcRate rate = LdpcRate::RATE_1_2;
    int k = LdpcCodec::block_size(rate);          // data bits per block
    int k_bytes = k / 8;
    int cap = k_bytes - 6;                        // max chunk per block
    auto make_block_bits = [&](const std::vector<uint8_t>& chunk,
                               std::vector<uint8_t>& bits) {
        std::vector<uint8_t> block(2 + chunk.size() + 4, 0);
        block[0] = (uint8_t)(chunk.size() & 0xFF);
        block[1] = (uint8_t)((chunk.size() >> 8) & 0xFF);
        std::copy(chunk.begin(), chunk.end(), block.begin() + 2);
        uint32_t crc = crc32(block.data(), 2 + (int)chunk.size());
        block[2 + chunk.size() + 0] = (uint8_t)(crc & 0xFF);
        block[2 + chunk.size() + 1] = (uint8_t)((crc >> 8) & 0xFF);
        block[2 + chunk.size() + 2] = (uint8_t)((crc >> 16) & 0xFF);
        block[2 + chunk.size() + 3] = (uint8_t)((crc >> 24) & 0xFF);
        size_t start = bits.size();
        bits.resize(start + k, 0);
        for (size_t i = 0; i < block.size() * 8; i++)
            bits[start + i] = (block[i / 8] >> (i % 8)) & 1;
    };
    std::vector<uint8_t> chunk0(cap), chunk1(cap / 2);
    uint8_t seed = 11;
    for (auto& b : chunk0) { seed = (uint8_t)(seed * 33 + 5); b = seed; }
    for (auto& b : chunk1) { seed = (uint8_t)(seed * 29 + 3); b = seed; }
    std::vector<uint8_t> decoded_bits;
    make_block_bits(chunk0, decoded_bits);
    make_block_bits(chunk1, decoded_bits);

    std::vector<uint8_t> expect;
    expect.insert(expect.end(), chunk0.begin(), chunk0.end());
    expect.insert(expect.end(), chunk1.begin(), chunk1.end());

    std::vector<uint8_t> got;
    if (!OfdmDemodulator::extract_payload_blocks(decoded_bits, rate, 2,
                                                  decoded_bits.size(), got) ||
        got != expect) {
        IRIS_LOG("[TEST-CHASE] FAIL D2: multi-block extract %zu B != expected %zu B",
                 got.size(), expect.size());
        return false;
    }
    // Pin the fail-before: single-block logic on the same bits "succeeds" with
    // ONLY block 0's chunk — the truncation the shared extractor closes.
    {
        std::vector<uint8_t> bytes(decoded_bits.size() / 8, 0);
        for (size_t i = 0; i < decoded_bits.size(); i++)
            bytes[i / 8] |= (uint8_t)(decoded_bits[i] << (i % 8));
        uint16_t old_len = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
        uint32_t computed = crc32(bytes.data(), 2 + old_len);
        uint32_t received = (uint32_t)bytes[2 + old_len]
                          | ((uint32_t)bytes[2 + old_len + 1] << 8)
                          | ((uint32_t)bytes[2 + old_len + 2] << 16)
                          | ((uint32_t)bytes[2 + old_len + 3] << 24);
        bool old_would_accept_truncated =
            (computed == received) && (old_len == chunk0.size()) &&
            (old_len < expect.size());
        if (!old_would_accept_truncated) {
            IRIS_LOG("[TEST-CHASE] FAIL: fail-before arm no longer reproduces "
                     "(len=%u crc=%d)", old_len, (int)(computed == received));
            return false;
        }
    }
    // A corrupted block 1 must fail the WHOLE frame (no partial success).
    {
        std::vector<uint8_t> bad = decoded_bits;
        bad[k + 40] ^= 1;
        std::vector<uint8_t> out;
        if (OfdmDemodulator::extract_payload_blocks(bad, rate, 2,
                                                     bad.size(), out)) {
            IRIS_LOG("[TEST-CHASE] FAIL D2: corrupted block 1 accepted");
            return false;
        }
    }

    // D1: flush decision. A same-shape success flushes the store; a
    // different-shape success (short control frame between retransmissions of
    // a failing multi-block I-frame) must NOT.
    if (!chase_flush_on_success(12800, 12800) ||   // same shape -> flush
        chase_flush_on_success(12800, 1600)  ||   // 20 B control OK -> keep store
        chase_flush_on_success(0, 1600)) {        // empty store -> nothing to flush
        IRIS_LOG("[TEST-CHASE] FAIL D1: flush decision wrong");
        return false;
    }

    IRIS_LOG("[TEST-CHASE] PASS: multi-block extract full %zu B (old logic truncated "
             "to %zu B), corrupt-block rejected, flush keeps cross-shape store",
             expect.size(), chunk0.size());
    return true;
}

void Modem::fail_native_transform(const char* operation,
                                  ArqTransferResultReason reason) {
    IRIS_LOG("[RC4] %s failed; closing native transfer without fallback", operation);
    native_rx_record_.clear();
    native_rx_record_type_ = ArqRecordType::Data;
    native_rx_record_id_ = 0;
    arq_.fail_active_transfer(reason);
}

void Modem::fail_strict_key_exchange(const char* operation) {
    IRIS_LOG("[RC4] %s failed; preserving strict-mode records", operation);
    // TX originals enter the ARQ custody ledger when first held.  Do not move or
    // clear either strict FIFO here: terminal publication snapshots the ledger
    // first, and ordinary disconnect cleanup happens only afterward.
    mlkem_kx_pending_ = false;
    mlkem_releasing_tx_ = false;
    mlkem_release_buffered_ = false;
    mlkem_release_prepared_count_ = 0;
    arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);
}

bool Modem::resolve_strict_receive_custody(const char* operation) {
    if (!mlkem_rx_held_records_.empty() && mlkem_kx_pending_) {
        fail_strict_key_exchange(operation);
        return false;
    }
    while (!mlkem_rx_held_records_.empty()) {
        const auto& record = mlkem_rx_held_records_.front();
        if (!accept_native_record(record.data(), record.size(),
                                  ArqRecordType::Data, 0))
            return false;
        mlkem_rx_held_records_.erase(mlkem_rx_held_records_.begin());
    }
    const auto outcome = arq_.commit_deferred_receive();
    return outcome.status() == ArqMutationStatus::Applied ||
           outcome.status() == ArqMutationStatus::Queued;
}

void Modem::fail_ofdm_transform(
    const char* operation,
    const std::vector<std::vector<uint8_t>>& originals) {
    IRIS_LOG("[RC4] %s failed; terminating transformed OFDM custody", operation);
    // Preserve the exact intercepted bytes before either transport is allowed to
    // clear scheduling state.  This also covers the B2F proxy, whose local RR may
    // already have transferred custody away from the client.
    for (const auto& original : originals)
        arq_.retain_original(original.data(), original.size());
    if (&originals != &b2f_proxy_originals_) {
        for (const auto& original : b2f_proxy_originals_)
            arq_.retain_original(original.data(), original.size());
    }
    // Once a negotiated wrapper transform fails, no record left in the live
    // scheduling queue may escape through the ordinary AX.25 branch. Preserve
    // untransformed source frames in terminal custody, then empty that queue.
    while (!tx_queue_.empty()) {
        TxFrame candidate = std::move(tx_queue_.front());
        tx_queue_.pop();
        if (!candidate.data.empty() && candidate.data[0] != B2F_DATA_MAGIC)
            arq_.retain_original(candidate.data.data(), candidate.data.size());
    }
    b2f_proxy_originals_.clear();
    arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);

    if (repack_has_open_custody()) {
        finish_transfer_once(v2::TransferResultReason::CodecFailure);
        return;
    }
    repack_failure_reason_hint_ = v2::TransferResultReason::CodecFailure;
    if (ax25_session_.state() != Ax25SessionState::DISCONNECTED)
        ax25_session_.reset();
}

bool Modem::accept_native_record(const uint8_t* data, size_t len,
                                 ArqRecordType type, uint64_t record_id) {
    if ((len > 0 && !data) || len > NATIVE_MAX_ENCODED_RECORD) {
        fail_native_transform("record admission", ArqTransferResultReason::RecordTooLarge);
        return false;
    }

    std::array<uint8_t, 32> digest{};
    crypto_blake2b_ctx digest_ctx;
    crypto_blake2b_init(&digest_ctx, digest.size());
    const uint8_t raw_type = static_cast<uint8_t>(type);
    crypto_blake2b_update(&digest_ctx, &raw_type, 1);
    uint8_t id_bytes[8];
    for (int i = 0; i != 8; ++i)
        id_bytes[i] = static_cast<uint8_t>(record_id >> (56 - i * 8));
    crypto_blake2b_update(&digest_ctx, id_bytes, sizeof(id_bytes));
    if (len > 0) crypto_blake2b_update(&digest_ctx, data, len);
    crypto_blake2b_final(&digest_ctx, digest.data());
    if (record_id != 0)
        for (const auto& completed : native_completed_records_)
            if (completed.record_id == record_id && completed.type == type)
                return true;
    auto remember_completed = [&]() {
        // Legacy DATA has no stable record identity. Its ARQ V(R) window alone
        // handles retransmission; content equality would suppress legitimate,
        // byte-identical records and silently lose the second delivery.
        if (record_id != 0)
            native_completed_records_.push_back({type, record_id, digest});
    };

    // Key traffic is selected only by the transport-owned record type. Data
    // ciphertext is never inspected for key-message magic before authentication.
    if (type == ArqRecordType::MlKemPublicKey ||
        type == ArqRecordType::MlKemCiphertext) {
        const bool public_key = type == ArqRecordType::MlKemPublicKey;
        const size_t expected = public_key ? 1 + MLKEM_PK_SIZE
                                           : 1 + MLKEM_CT_SIZE;
        const uint8_t expected_magic = public_key ? MLKEM_PK_MAGIC : MLKEM_CT_MAGIC;
        if (len != expected || data[0] != expected_magic) {
            fail_native_transform("key-message framing");
            return false;
        }
        handle_mlkem_frame(data, len);
        if (arq_.state() == ArqState::IDLE ||
            arq_.state() == ArqState::DISCONNECTING)
            return false;
        remember_completed();
        return true;
    }

    // Strict hybrid mode adopts complete serialized records into an owned FIFO,
    // but does not mutate B2F or codec state until key exchange is ready.
    if (mlkem_kx_pending_ && config_.encryption_mode == 1) {
        size_t held = 0;
        for (const auto& record : mlkem_rx_held_records_) held += record.size();
        if (len > NATIVE_MAX_ENCODED_RECORD - std::min(held, NATIVE_MAX_ENCODED_RECORD)) {
            fail_native_transform("strict key-exchange hold",
                                  ArqTransferResultReason::RecordTooLarge);
            return false;
        }
        mlkem_rx_held_records_.emplace_back(data, data + len);
        arq_.defer_receive_commit();
        return true;
    }

    const uint8_t* current = data;
    size_t current_len = len;
    std::vector<uint8_t> decrypted;
    if (cipher_.is_active() && current_len > 0) {
        if (current_len <= AUTH_TAG_SIZE) {
            fail_native_transform("authenticated decryption");
            return false;
        }
        decrypted.resize(current_len - AUTH_TAG_SIZE);
        const int decoded = cipher_.decrypt(
            current, static_cast<int>(current_len), decrypted.data(),
            static_cast<int>(decrypted.size()), rx_batch_counter_,
            crypto_direction_ ^ 1, AUTH_TAG_SIZE);
        if (decoded < 0 || static_cast<size_t>(decoded) != decrypted.size()) {
            fail_native_transform("authenticated decryption");
            return false;
        }
        ++rx_batch_counter_; // commit nonce ordering only after authentication
        current = decrypted.data();
        current_len = decrypted.size();
    }

    std::vector<uint8_t> plaintext;
    if (arq_.negotiated(CAP_COMPRESSION) && current_len > 0) {
        auto decoded = rx_compressor_.decompress_record(current, current_len);
        if (decoded.status != v2::TransformStatus::Produced) {
            fail_native_transform("record decompression");
            return false;
        }
        plaintext = std::move(decoded.produced_bytes);
        current = plaintext.data();
        current_len = plaintext.size();
        // RX model history commits exactly once, after the complete record and
        // declared length have validated and before application acceptance.
        rx_compressor_.streaming_commit(current, static_cast<int>(current_len));
    }

    std::vector<uint8_t> rerolled;
    if (arq_.negotiated(CAP_B2F_UNROLL) && b2f_handler_.is_initialized()) {
        auto filtered = b2f_handler_.filter_rx_record(current, current_len);
        if (filtered.status == v2::TransformStatus::Failed) {
            fail_native_transform("B2F reroll");
            return false;
        }
        rerolled = std::move(filtered.produced_bytes);
        while (filtered.status == v2::TransformStatus::NeedOutput &&
               filtered.retained_output_bytes != 0) {
            filtered = b2f_handler_.filter_rx_record(nullptr, 0);
            if (filtered.status == v2::TransformStatus::Failed) {
                fail_native_transform("B2F reroll drain");
                return false;
            }
            rerolled.insert(rerolled.end(), filtered.produced_bytes.begin(),
                            filtered.produced_bytes.end());
        }
        if (filtered.status == v2::TransformStatus::Buffered && rerolled.empty())
        {
            remember_completed();
            return true;
        }
        current = rerolled.data();
        current_len = rerolled.size();
    }

    // Identity commit precedes external publication so callback reentry cannot
    // admit the same completed record a second time.
    remember_completed();
    if (rx_callback_) {
        bytes_rx_ += current_len;
        rx_callback_(current, current_len);
    }
    return true;
}

bool Modem::receive_native_fragment(const uint8_t* data, size_t len, bool end,
                                    ArqRecordType type, uint64_t record_id) {
    if ((len > 0 && !data) || len > NATIVE_MAX_ENCODED_RECORD ||
        native_rx_record_.size() > NATIVE_MAX_ENCODED_RECORD - len) {
        fail_native_transform("bounded record reassembly",
                              ArqTransferResultReason::RecordTooLarge);
        return false;
    }
    if (native_rx_record_.empty()) {
        native_rx_record_type_ = type;
        native_rx_record_id_ = record_id;
    } else if (native_rx_record_type_ != type ||
               native_rx_record_id_ != record_id) {
        fail_native_transform("record identity changed during reassembly");
        return false;
    }
    native_rx_record_.insert(native_rx_record_.end(), data, data + len);

    if (type != ArqRecordType::Data) {
        if (!end) return true;
        std::vector<uint8_t> complete = std::move(native_rx_record_);
        native_rx_record_.clear();
        native_rx_record_type_ = ArqRecordType::Data;
        native_rx_record_id_ = 0;
        return accept_native_record(complete.data(), complete.size(), type, record_id);
    }

    // Ciphertext has no trustworthy length until its complete authenticated
    // record marker. The marker is transport-owned and is repeated on retries.
    if (cipher_.is_active()) {
        if (!end) return true;
        std::vector<uint8_t> complete = std::move(native_rx_record_);
        native_rx_record_.clear();
        native_rx_record_type_ = ArqRecordType::Data;
        native_rx_record_id_ = 0;
        return accept_native_record(complete.data(), complete.size(), type, record_id);
    }

    if (arq_.negotiated(CAP_COMPRESSION)) {
        for (;;) {
            const size_t header = static_cast<size_t>(rx_compressor_.get_header_size());
            if (native_rx_record_.size() < header) {
                if (!end) return true;
                fail_native_transform("codec header reassembly");
                return false;
            }
            size_t encoded_size = 0, original_size = 0;
            if (!rx_compressor_.declared_record_sizes(native_rx_record_.data(),
                                                       native_rx_record_.size(),
                                                       encoded_size, original_size) ||
                encoded_size > NATIVE_MAX_ENCODED_RECORD ||
                original_size > COMPRESS_MAX_RECORD_SIZE) {
                fail_native_transform("codec envelope admission",
                    encoded_size > NATIVE_MAX_ENCODED_RECORD
                        ? ArqTransferResultReason::RecordTooLarge
                        : ArqTransferResultReason::TransformFailure);
                return false;
            }
            if (native_rx_record_.size() < encoded_size) {
                if (!end) return true;
                fail_native_transform("incomplete codec record");
                return false;
            }
            std::vector<uint8_t> complete(native_rx_record_.begin(),
                                          native_rx_record_.begin() + encoded_size);
            native_rx_record_.erase(native_rx_record_.begin(),
                                    native_rx_record_.begin() + encoded_size);
            if (!accept_native_record(complete.data(), complete.size(), type, record_id))
                return false;
            if (native_rx_record_.empty()) return true;
            // Multiple complete records may share an ARQ flight. Each is still
            // inverse-transformed and committed independently in wire order.
        }
    }

    if (!end) return true;
    std::vector<uint8_t> complete = std::move(native_rx_record_);
    native_rx_record_.clear();
    native_rx_record_type_ = ArqRecordType::Data;
    native_rx_record_id_ = 0;
    return accept_native_record(complete.data(), complete.size(), type, record_id);
}

void Modem::queue_tx_frame(const uint8_t* frame, size_t len) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    bytes_tx_ += len;
    if (repack_has_open_custody()) {
        if (repack_complete_remote_close(frame, len)) return;
        if (repack_sframe_consume(frame, len)) return;
        if (repack_hold_client_disc(frame, len)) return;
        Ax25Frame boundary;
        if (ax25_parse(frame, len, boundary) &&
            boundary.type() == Ax25FrameType::U_FRAME &&
            boundary.u_type() == Ax25UType::SABM) {
            repack_failure_reason_hint_ =
                v2::TransferResultReason::SessionReplaced;
            ax25_session_.reset();
            return;
        }
    }
    if (native_mode_ && native_tx_ready_ && arq_.state() == ArqState::CONNECTED) {
        if ((len > 0 && !frame) || len > NATIVE_MAX_ENCODED_RECORD) {
            arq_.fail_active_transfer(ArqTransferResultReason::RecordTooLarge);
            return;
        }
        const bool b2f_payload = arq_.negotiated(CAP_B2F_UNROLL) &&
                                 b2f_handler_.is_tx_payload_active();
        if (arq_.negotiated(CAP_COMPRESSION) &&
            ((!b2f_payload && len > COMPRESS_MAX_RECORD_SIZE) ||
             (b2f_payload &&
              !b2f_handler_.tx_output_within_limit(COMPRESS_MAX_RECORD_SIZE)))) {
            // The codec's legacy envelope has a 16-bit original-size field.
            // Reject before either B2F or the custody ledger accepts the record.
            arq_.fail_active_transfer(ArqTransferResultReason::RecordTooLarge);
            return;
        }
        // Strict mode takes custody of the untouched record before any B2F,
        // compression, model, nonce, or dictionary state can change.
        if (mlkem_kx_pending_ && config_.encryption_mode == 1) {
            size_t held = 0;
            for (const auto& record : mlkem_held_frames_) held += record.size();
            if (len > NATIVE_MAX_ENCODED_RECORD -
                      std::min(held, NATIVE_MAX_ENCODED_RECORD)) {
                fail_strict_key_exchange("strict key-exchange TX admission");
                return;
            }
            // Custody precedes the strict hold. Disconnect, shutdown, or rekey
            // can now only publish through the ledger that owns this original.
            arq_.retain_original(frame, len);
            mlkem_held_frames_.emplace_back(frame, frame + len);
            IRIS_LOG("[CRYPTO] Strict mode: holding %zu-byte record until ML-KEM completes", len);
            return;
        }
        if (!mlkem_releasing_tx_)
            arq_.retain_original(frame, len);

        const uint8_t* cur = frame;
        size_t cur_len = len;
        bool b2f_original_fully_prepared = true;

        // Layer 1: B2F unroll if negotiated (strip LZHUF on TX)
        std::vector<uint8_t> unrolled;
        if (arq_.negotiated(CAP_B2F_UNROLL) && b2f_handler_.is_initialized()) {
            auto filtered = b2f_handler_.filter_tx_record(cur, cur_len);
            if (filtered.status == v2::TransformStatus::Failed) {
                arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);
                return;
            }
            unrolled = std::move(filtered.produced_bytes);
            while (filtered.status == v2::TransformStatus::NeedOutput &&
                   filtered.retained_output_bytes != 0) {
                filtered = b2f_handler_.filter_tx_record(nullptr, 0);
                if (filtered.status == v2::TransformStatus::Failed) {
                    arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);
                    return;
                }
                unrolled.insert(unrolled.end(), filtered.produced_bytes.begin(),
                                filtered.produced_bytes.end());
            }
            if (filtered.status == v2::TransformStatus::Buffered && unrolled.empty()) {
                // B2F owns the input bytes but has not produced transport bytes.
                // Keep strict FIFO custody and prevent ACKs for earlier output
                // from completing the shared outbound transaction.
                if (mlkem_releasing_tx_) {
                    mlkem_release_buffered_ = true;
                    arq_.defer_transmit_commit();
                }
                return;
            }
            b2f_original_fully_prepared =
                filtered.status == v2::TransformStatus::Produced;
            if (mlkem_releasing_tx_ && !b2f_original_fully_prepared) {
                // A completed prefix can be sent, but this FIFO entry also owns
                // a suffix still buffered by B2F.  Withhold preparation credit
                // and prefix retirement until a later entry closes that suffix.
                mlkem_release_buffered_ = true;
                arq_.defer_transmit_commit();
            }
            cur = unrolled.data();
            cur_len = unrolled.size();
        }

        // Layer 2: Compress if negotiated
        std::vector<uint8_t> compressed;
        if (arq_.negotiated(CAP_COMPRESSION) && cur_len > 0) {
            auto encoded = tx_compressor_.compress_record(cur, cur_len);
            if (encoded.status != v2::TransformStatus::Produced) {
                arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);
                return;
            }
            compressed = std::move(encoded.produced_bytes);
            cur = compressed.data();
            cur_len = compressed.size();
        }

        // Layer 3: Encrypt if active
        std::vector<uint8_t> encrypted;
        if (cipher_.is_active() && cur_len > 0) {
            encrypted.resize(cur_len + AUTH_TAG_SIZE);
            int enc_len = cipher_.encrypt(cur, (int)cur_len, encrypted.data(), (int)encrypted.size(),
                                           tx_batch_counter_, crypto_direction_, AUTH_TAG_SIZE);
            if (enc_len > 0) {
                ++tx_batch_counter_;
                cur = encrypted.data();
                cur_len = enc_len;
            } else {
                arq_.fail_active_transfer(ArqTransferResultReason::TransformFailure);
                return;
            }
        }

        // Ordered adoption is the TX commit. The immutable encoded bytes enter
        // transport ownership once, and model history advances once here.
        if (arq_.negotiated(CAP_COMPRESSION) && !compressed.empty())
            tx_compressor_.streaming_commit(unrolled.empty() ? frame : unrolled.data(),
                static_cast<int>(unrolled.empty() ? len : unrolled.size()));
        bool completed_buffered_prefix = false;
        if (mlkem_releasing_tx_) {
            if (b2f_original_fully_prepared)
                arq_.mark_originals_prepared(mlkem_release_prepared_count_);
        } else {
            // A partial result proves the older FIFO prefix but retains bytes
            // from this call's newest original.  Keep that tail outstanding;
            // nonempty output alone is not whole-original preparation.
            arq_.mark_all_originals_prepared(
                b2f_original_fully_prepared ? 0 : 1);
            if (b2f_original_fully_prepared && !mlkem_held_frames_.empty()) {
                // These entries were submitted once by rekey_hybrid() and left
                // in the FIFO only because B2F had not emitted their bytes. The
                // newly produced unit now covers them; ARQ still retains their
                // originals until the unit's eventual ACK.
                mlkem_held_frames_.clear();
                completed_buffered_prefix = true;
            }
        }
        const auto outcome = arq_.send_prepared(
            cur, cur_len, ArqRecordType::Data, 0);
        if (mlkem_releasing_tx_ && outcome)
            mlkem_release_prepared_count_ = 0;
        if (completed_buffered_prefix)
            arq_.commit_deferred_transmit();
        return;
    }

    // Terminate/re-pack the native-OFDM data plane (keystone).  Divert the
    // client's I-frame data plane (custody-ACK + length-delimited stream) and
    // consume the local pump's R3 S-frame ACKs.  Gated on native_repack_active()
    // -> the AFSK / pure-KISS control path below is byte-identical.  SABM/DISC/UA
    // (connect/teardown) fall through to shadow tracking + fly unchanged.
    if (native_repack_active()) {
        repack_maybe_engage();                          // absorb held frames once
        if (repack_tx_ingest(frame, len)) return;       // client data terminated
    }

    // Notify AX.25 session of outgoing frame so it can track KISS-initiated
    // connections (SABM/DISC) without generating duplicate frames.
    // Must be under modem_mutex_ to synchronize with connection header injection.
    ax25_session_.notify_outgoing(frame, len);

    // Buffer KISS client I-frame info fields during AFSK phase for B2F replay.
    // The session send_callback only captures session-generated frames (RR etc),
    // not KISS client data — but the B2F SID/FC/FS exchange comes from the client.
    // Parse (not frame[14]/+16 fixed offsets): a KISS client connecting THROUGH
    // a digipeater (Winlink "Digipeater" connection type) hands us via-carrying
    // frames, which shift control/info by 7 bytes per hop; the fixed-offset
    // sniff misclassified every such frame as an I-frame (bit 0 of a shifted
    // callsign char is 0) and buffered address bytes as B2F history.
    if (config_.b2f_unroll && !ofdm_kiss_tx_ && len > 16) {
        Ax25Frame bf;
        if (ax25_parse(frame, len, bf) &&
            bf.type() == Ax25FrameType::I_FRAME && !bf.info.empty()) {
            b2f_afsk_tx_history_.emplace_back(bf.info.begin(), bf.info.end());
        }
    }

    // B2F proxy TX: sniff outgoing I-frame info fields through B2F handler.
    // During PAYLOAD_TRANSFER (local proposer): intercept LZHUF I-frames,
    // unroll via handler, compress plaintext, send as B2F_DATA over OFDM.
    // Line protocol I-frames (SID, FC, FS, FF, FQ) are forwarded normally.
    // Fixed no-via offsets (ctrl at 14, info at 16; the local-RR rebuild below
    // copies addr bytes 0-13) are safe HERE: gated ofdm_kiss_tx_, and B2F proxy
    // runs only on the Iris<->Iris DIRECT native link — the KISS seam of a
    // digipeated connection never activates it (endpoint-via over OFDM
    // deferred, DIGIPEATER_DESIGN.md sec 5.5-2; re-audit these offsets then).
    if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
        ofdm_kiss_b2f_.is_initialized() && len > 16) {
        uint8_t ctrl = frame[14];
        bool is_iframe = (ctrl & 0x01) == 0;

        if (is_iframe) {
            const uint8_t* info = frame + 16;  // 14 addr + 1 ctrl + 1 pid
            int info_len = (int)(len - 16);

            // Check BEFORE feeding: is handler already in TX payload mode?
            // Transition happens in filter_rx (when FS arrives from remote),
            // so by the time we see LZHUF I-frames here, it's already set.
            // Don't intercept resume transfers — partial LZHUF can't be unrolled
            bool intercepting = ofdm_kiss_b2f_.is_tx_payload_active() &&
                                !ofdm_kiss_b2f_.is_resume_transfer();

            if (intercepting) {
                size_t proxy_charge = len;
                for (const auto& original : b2f_proxy_originals_)
                    proxy_charge += original.size();
                if (proxy_charge > NATIVE_MAX_ENCODED_RECORD) {
                    auto preserved = b2f_proxy_originals_;
                    preserved.emplace_back(frame, frame + len);
                    fail_ofdm_transform("B2F proxy transmit admission", preserved);
                    return;
                }
                b2f_proxy_originals_.emplace_back(frame, frame + len);
                // Feed LZHUF data to filter_tx for unrolling
                auto filtered = ofdm_kiss_b2f_.filter_tx_record(info, info_len);
                if (filtered.status == v2::TransformStatus::Failed) {
                    fail_ofdm_transform("B2F proxy transmit unroll",
                                        b2f_proxy_originals_);
                    return;
                }
                auto b2f_out = std::move(filtered.produced_bytes);
                int out_len = static_cast<int>(b2f_out.size());

                if (out_len > 0) {
                    // Unrolled plaintext ready (one proposal complete)
                    b2f_proxy_plaintext_.insert(b2f_proxy_plaintext_.end(),
                        b2f_out.begin(), b2f_out.begin() + out_len);
                }

                // Check if all proposals done
                if (!ofdm_kiss_b2f_.is_tx_payload_active()) {
                    b2f_proxy_active_ = false;
                    // Flush accumulated plaintext as B2F_DATA frame(s)
                    if (!b2f_proxy_plaintext_.empty()) {
                        IRIS_LOG("[B2F-PROXY] TX: unrolled %zu bytes plaintext, sending B2F_DATA",
                                 b2f_proxy_plaintext_.size());
                        // Segment into chunks that fit in NATIVE_MAX_PAYLOAD
                        // Each chunk: [0xCD][compressed_block]
                        const int CHUNK_SIZE = 3000;  // leaves room for compression overhead
                        size_t offset = 0;
                        while (offset < b2f_proxy_plaintext_.size()) {
                            size_t chunk = std::min((size_t)CHUNK_SIZE,
                                                     b2f_proxy_plaintext_.size() - offset);
                            // Compress this chunk
                            auto encoded = ofdm_kiss_tx_compressor_.compress_record(
                                b2f_proxy_plaintext_.data() + offset, chunk);
                            if (encoded.status != v2::TransformStatus::Produced) {
                                fail_ofdm_transform("B2F proxy transmit compression",
                                                    b2f_proxy_originals_);
                                return;
                            }
                            // Build B2F_DATA frame
                            std::vector<uint8_t> b2f_frame;
                            b2f_frame.push_back(B2F_DATA_MAGIC);
                            b2f_frame.insert(b2f_frame.end(), encoded.produced_bytes.begin(),
                                             encoded.produced_bytes.end());
                            IRIS_LOG("[B2F-PROXY] TX: chunk %zu bytes -> %zu encoded",
                                     chunk, encoded.produced_bytes.size());
                            constexpr size_t TX_QUEUE_MAX = 32;
                            if (tx_queue_.size() >= TX_QUEUE_MAX) {
                                fail_ofdm_transform("B2F proxy output admission",
                                                    b2f_proxy_originals_);
                                return;
                            }
                            tx_queue_.push(TxFrame(std::move(b2f_frame)));  // B2F data: never tone-eligible
                            offset += chunk;
                        }
                        b2f_proxy_plaintext_.clear();
                    }
                }

                // Generate RR ACK back to local Winlink so it doesn't timeout.
                // Construct RR frame: swap src/dst, N(R) = N(S)+1
                if (b2f_proxy_addr_valid_ && len > 14) {
                    uint8_t ns = (ctrl >> 1) & 0x07;
                    b2f_proxy_vr_ = (ns + 1) & 0x07;
                    // Build RR response: [dst(7)][src(7)][ctrl_RR(1)]
                    // dst = original src (our Winlink), src = original dst (remote)
                    std::vector<uint8_t> rr(15);
                    memcpy(rr.data(), frame + 7, 7);      // dst = original src
                    memcpy(rr.data() + 7, frame, 7);      // src = original dst
                    // Fix address extension bits
                    rr[6] &= 0xFE;    // dst: clear end-of-address bit
                    rr[13] |= 0x01;   // src: set end-of-address bit
                    // RR S-frame control: (N(R) << 5) | (F << 4) | 0x01
                    bool poll = (ctrl & 0x10) != 0;
                    rr[14] = (b2f_proxy_vr_ << 5) | (poll ? 0x10 : 0) | 0x01;
                    if (rx_callback_)
                        rx_callback_(rr.data(), rr.size());
                }

                return;  // Don't forward this I-frame
            }

            // Not intercepting: feed for state tracking (line protocol)
            auto tracked = ofdm_kiss_b2f_.filter_tx_record(info, info_len);
            if (tracked.status == v2::TransformStatus::Failed) {
                fail_ofdm_transform("B2F proxy transmit state tracking",
                    {std::vector<uint8_t>(frame, frame + len)});
                return;
            }

            // Check if handler just transitioned to TX payload mode
            // Skip interception for resume transfers (partial LZHUF)
            if (ofdm_kiss_b2f_.is_tx_payload_active() &&
                !ofdm_kiss_b2f_.is_resume_transfer() && !b2f_proxy_active_) {
                b2f_proxy_active_ = true;
                b2f_proxy_vr_ = 0;
                b2f_proxy_plaintext_.clear();
                memcpy(b2f_proxy_addr_, frame, 14);
                b2f_proxy_addr_valid_ = true;
                IRIS_LOG("[B2F-PROXY] TX interception started (local proposer payload)");
            } else if (ofdm_kiss_b2f_.is_resume_transfer() && !b2f_proxy_active_) {
                IRIS_LOG("[B2F-PROXY] Resume transfer — skipping unroll (partial LZHUF)");
            }
            // Fall through: forward line protocol I-frame normally
        }

    }

    // B2F proxy RX ACK suppression (item 2) — HOISTED out of the len>16
    // B2F-unroll gate above, where it was DEAD CODE for its target: host RRs are
    // exactly 15 bytes, so they never entered that gate and the pump's RRs
    // (carrying proxy-local N(R)) leaked OTA and tripped the peer's stale-S
    // drain. While b2f_proxy_rx_active_, the local Winlink's RR/RNR/REJ ACKs for
    // our injected I-frames are meaningless to the remote peer — drop them here.
    // Content typing is SOUND at this site: `frame` is a host-emitted genuine
    // AX.25 frame (from notify_outgoing / the KISS client), NOT the tx_queue_
    // aliasing surface, so (frame[14] & 0x03) == 0x01 reliably means S-frame.
    if (b2f_proxy_rx_active_ && len >= 15 && (frame[14] & 0x03) == 0x01) {
        IRIS_LOG("[B2F-PROXY] suppressing host S-frame (proxy-local N(R)) during B2F RX");
        return;  // Suppress
    }

    std::vector<uint8_t> tx_frame(frame, frame + len);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] queue full (%zu frames), rejecting new frame", tx_queue_.size());
        return;
    }
    // KISS client / passthrough frame (the SHADOW AX.25 deployment path): never
    // tone-eligible — a client's own UA arrives here and must go out via OFDM.
    tx_queue_.push(TxFrame(std::move(tx_frame)));
    if (gui_log_ && len >= 14)
        gui_log_("[TX] " + describe_ax25(frame, len));
}

void Modem::ax25_connect(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Guard: reject if session already active (prevents SABM loop when AGW client
    // retries connect before processing the CONNECTED notification).
    if (ax25_session_.is_active()) {
        IRIS_LOG("AX25 connect to %s REJECTED — session already active (state=%d)",
                 remote_callsign.c_str(), (int)ax25_session_.state());
        return;
    }
    // Reset state for fresh connection
    peer_is_iris_ = false;
    ofdm_kiss_probe_done_ = false;

    // Connect immediately — probe starts after CONNECTED (state callback).
    // This lets the SABM/UA handshake complete first so Winlink doesn't timeout.
    IRIS_LOG("AX25 connect to %s", remote_callsign.c_str());
    ax25_session_.connect(remote_callsign);
}

void Modem::ax25_disconnect() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Cancel probe if in progress
    if (ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0) {
        IRIS_LOG("AX25 disconnect: cancelling probe");
        ofdm_kiss_probing_ = false;
        ofdm_kiss_probe_cd_ = 0;
        probe_connect_timeout_ = 0;
        probe_start_pending_ = false;
        probe_.reset();
    }
    IRIS_LOG("AX25 disconnect");
    ax25_session_.disconnect();
}

void Modem::send_connected_data(const uint8_t* data, size_t len) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (ax25_session_.state() == Ax25SessionState::CONNECTED ||
        ax25_session_.state() == Ax25SessionState::TIMER_RECOVERY) {
        ax25_session_.send_data(data, len);
    } else if (arq_.state() == ArqState::CONNECTED) {
        queue_tx_frame(data, len);
    } else {
        IRIS_LOG("Connected data rejected: no active session");
        if (gui_log_)
            gui_log_("[ERROR] Connected data rejected: no active session");
    }
}

void Modem::arq_connect(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("ARQ connect to %s", remote_callsign.c_str());
    // Generate ephemeral X25519 keypair for DH key exchange
    generate_ephemeral_x25519();
    // Start in AX.25 — upgrade to native after XID negotiation
    arq_.connect(remote_callsign);
}

void Modem::arq_disconnect() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("ARQ disconnect");
    arq_.disconnect();
}

void Modem::arq_listen() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Skip if already listening (prevents recursion from state callbacks)
    if (arq_.state() == ArqState::LISTENING)
        return;
    // When native_hail relisten is pending, let the deferred path handle it
    if (relisten_pending_)
        return;
    // Always start in AX.25 mode. Native upgrade happens via XID negotiation
    // when the remote station proves it supports Iris native PHY.
    // --ax25-only suppresses XID entirely.
    IRIS_LOG("ARQ listen (native_mode=0, ax25_only=%d)", config_.ax25_only ? 1 : 0);
    // Generate ephemeral X25519 keypair for DH key exchange
    generate_ephemeral_x25519();
    arq_.listen();
}

void Modem::generate_ephemeral_x25519() {
    if (config_.encryption_mode == 0) return;
    uint8_t pubkey[X25519_KEY_SIZE];
    if (cipher_.generate_x25519_keypair(pubkey) == 0) {
        arq_.set_local_x25519_pubkey(pubkey);
        IRIS_LOG("[CRYPTO] Generated ephemeral X25519 keypair");
    } else {
        IRIS_LOG("[CRYPTO] WARNING: X25519 keypair generation failed");
    }
}

void Modem::start_mlkem_exchange() {
    // Commander generates ML-KEM-768 keypair and sends encapsulation key
    uint8_t encaps_key[MLKEM_PK_SIZE];
    if (cipher_.generate_mlkem_keypair(encaps_key) != 0) {
        if (config_.encryption_mode == 1)
            fail_strict_key_exchange("ML-KEM keypair generation");
        else
            mlkem_kx_pending_ = false;
        return;
    }
    // Send as ARQ data: [MLKEM_PK_MAGIC][encaps_key(1184)]
    std::vector<uint8_t> kx_frame;
    kx_frame.reserve(1 + MLKEM_PK_SIZE);
    kx_frame.push_back(MLKEM_PK_MAGIC);
    kx_frame.insert(kx_frame.end(), encaps_key, encaps_key + MLKEM_PK_SIZE);
    arq_.send_prepared(kx_frame.data(), kx_frame.size(),
                       ArqRecordType::MlKemPublicKey, 0);
    IRIS_LOG("[CRYPTO] ML-KEM: sent encapsulation key (%d bytes)", MLKEM_PK_SIZE);
}

void Modem::handle_mlkem_frame(const uint8_t* data, size_t len) {
    if (data[0] == MLKEM_PK_MAGIC && len == 1 + MLKEM_PK_SIZE) {
        // Responder received encapsulation key — encapsulate and send ciphertext back
        IRIS_LOG("[CRYPTO] ML-KEM: received encapsulation key (%zu bytes)", len - 1);
        uint8_t ciphertext[MLKEM_CT_SIZE];
        if (cipher_.encapsulate_mlkem(data + 1, ciphertext) != 0) {
            if (config_.encryption_mode == 1)
                fail_strict_key_exchange("ML-KEM encapsulation");
            else
                mlkem_kx_pending_ = false;
            return;
        }
        // Send ciphertext back
        std::vector<uint8_t> ct_frame;
        ct_frame.reserve(1 + MLKEM_CT_SIZE);
        ct_frame.push_back(MLKEM_CT_MAGIC);
        ct_frame.insert(ct_frame.end(), ciphertext, ciphertext + MLKEM_CT_SIZE);
        arq_.send_prepared(ct_frame.data(), ct_frame.size(),
                           ArqRecordType::MlKemCiphertext, 0);
        IRIS_LOG("[CRYPTO] ML-KEM: sent ciphertext (%d bytes)", MLKEM_CT_SIZE);
        // Responder has both shared secrets now — rekey
        rekey_hybrid();
    } else if (data[0] == MLKEM_CT_MAGIC && len == 1 + MLKEM_CT_SIZE) {
        // Commander received ciphertext — decapsulate
        IRIS_LOG("[CRYPTO] ML-KEM: received ciphertext (%zu bytes)", len - 1);
        if (cipher_.decapsulate_mlkem(data + 1) != 0) {
            if (config_.encryption_mode == 1)
                fail_strict_key_exchange("ML-KEM decapsulation");
            else
                mlkem_kx_pending_ = false;
            return;
        }
        // Commander has both shared secrets now — rekey
        rekey_hybrid();
    } else {
        IRIS_LOG("[CRYPTO] ML-KEM: unexpected frame (magic=0x%02X, len=%zu)", data[0], len);
        if (mlkem_kx_pending_ && config_.encryption_mode == 1)
            fail_strict_key_exchange("ML-KEM message validation");
    }
}

void Modem::rekey_hybrid() {
    // Re-derive session key with both X25519 + ML-KEM shared secrets
    std::vector<uint8_t> psk;
    for (size_t i = 0; i + 1 < config_.psk_hex.size(); i += 2) {
        char byte_str[3] = {config_.psk_hex[i], config_.psk_hex[i+1], 0};
        psk.push_back((uint8_t)strtol(byte_str, nullptr, 16));
    }
    cipher_.derive_session_key(config_.callsign.c_str(),
                                arq_.remote_callsign().c_str(),
                                psk.empty() ? nullptr : psk.data(),
                                (int)psk.size(), true);  // mlkem_done=true
    // Reset batch counters — both sides rekey at the same point
    tx_batch_counter_ = 0;
    rx_batch_counter_ = 0;
    mlkem_kx_pending_ = false;
    IRIS_LOG("[CRYPTO] HYBRID REKEY: X25519 + ML-KEM-768 (SNDL-proof)");
    if (gui_log_) gui_log_("[CRYPTO] Post-quantum encryption active");

    // Release strict-mode custody in its original deterministic record order.
    mlkem_releasing_tx_ = true;
    size_t release_index = 0;
    while (release_index < mlkem_held_frames_.size()) {
        const auto& record = mlkem_held_frames_[release_index];
        mlkem_release_buffered_ = false;
        mlkem_release_prepared_count_ = release_index + 1;
        queue_tx_frame(record.data(), record.size());
        if (mlkem_release_buffered_) {
            // The input was consumed into B2F, so do not resubmit it. A later
            // strict FIFO entry may complete the buffered B2F unit; until then
            // every submitted original remains held and TX completion deferred.
            ++release_index;
            continue;
        }
        // queue_tx_frame records an explicit send_prepared receipt by clearing
        // the pending count. Never infer admission from mutable session state.
        if (mlkem_release_prepared_count_ != 0)
            break;
        // Only whole-original coverage authorizes FIFO-prefix retirement. A
        // call that sent a completed prefix while retaining its own suffix sets
        // mlkem_release_buffered_ and leaves the entire submitted prefix owned.
        mlkem_held_frames_.erase(mlkem_held_frames_.begin(),
                                 mlkem_held_frames_.begin() + release_index + 1);
        release_index = 0;
    }
    mlkem_releasing_tx_ = false;
    mlkem_release_buffered_ = false;
    mlkem_release_prepared_count_ = 0;
    // Authentication/decode of held inbound records precedes any deferred TX
    // success. A shared key-exchange ledger cannot publish completion while an
    // inbound transform can still fail or an outbound original remains unacked.
    if (!resolve_strict_receive_custody("strict receive release")) return;
    if (mlkem_held_frames_.empty())
        arq_.commit_deferred_transmit();
}

// After this many AFSK SABM failures, escalate to native BPSK hailing
static constexpr int NATIVE_HAIL_ESCALATION_RETRIES = 3;

void Modem::tick() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    arq_.tick();
    ax25_session_.tick();
    if (repack_terminal_requested_) {
        repack_terminal_requested_ = false;
        if (ax25_session_.state() != Ax25SessionState::DISCONNECTED)
            ax25_session_.reset();
        else
            finish_transfer_once(repack_failure_reason_hint_);
        return;
    }
    probe_.tick();

    // R3 progress is supervised from first admitted receiver record, independent
    // of whether an air DISC has arrived.  A stalled or vanished local client
    // cannot make the modem accept an unbounded R2 stream indefinitely.
    if (repack_rx_transfer_ledger_ &&
        !repack_rx_transfer_ledger_->terminal_published) {
        const uint64_t delivered = repack_rx_transfer_ledger_->r3_sequence
            .endpoint_cursor.next_absolute_position;
        const bool r3_pending = delivered <
            repack_rx_transfer_ledger_->original_records.size();
        if (r3_pending) {
            ++repack_r3_no_progress_ticks_;
            ++repack_r3_retry_ticks_;
            if (repack_r3_retry_ticks_ >= REPACK_R3_RETRY_TICKS &&
                !repack_reorig_peer_busy_) {
                repack_r3_retry_ticks_ = 0;
                ++repack_r3_retries_;
                repack_retry_r3_window();
                if (!repack_has_open_custody())
                    return;
            }
            if (repack_r3_no_progress_ticks_ >= REPACK_R3_DEADLINE_TICKS ||
                repack_r3_retries_ > REPACK_R3_MAX_RETRIES) {
                repack_failure_reason_hint_ =
                    v2::TransferResultReason::RemoteClientLost;
                if (ax25_session_.state() != Ax25SessionState::DISCONNECTED)
                    ax25_session_.reset();
                else
                    finish_transfer_once(repack_failure_reason_hint_);
                return;
            }
        }
        repack_update_rx_backpressure();
    }

    // Terminate/re-pack pacing.  Drain full max_info() chunks whenever the R2
    // window has room; FLUSH the stream tail (a short frame) only after the client
    // has been quiet REPACK_FLUSH_IDLE_TICKS ticks so mid-stream frames stay dense
    // (event/idle-driven — the frame SIZE stays dynamic, Step-0 (a)).  Also re-pump
    // the RX re-origination in case a pump ACK freed the R3 window off-path.
    if (native_repack_active()) {
        repack_maybe_engage();   // absorb held frames the instant OFDM engages
        bool tx_pending = !native_repack_tx_stream_.empty();
        if (tx_pending) repack_tx_idle_ticks_++;
        else            repack_tx_idle_ticks_ = 0;
        bool flush = tx_pending && repack_tx_idle_ticks_ >= REPACK_FLUSH_IDLE_TICKS;
        repack_tx_drain(flush);
        repack_rx_drain();
        repack_graceful_close_tick();
        if (!repack_has_open_custody() &&
            repack_transfer_state_ == RepackTransferState::Failed)
            return;
    }

    // Deferred PROBE:START — queued from state callback.
    if (probe_start_pending_) {
        probe_start_pending_ = false;
        send_probe_start_ui();
        send_probe_start_ui();
    }

    // 30-second disconnect timeout: if stuck in AWAITING_RELEASE, force reset.
    // Winlink gets upset if we keep TX'ing DISC retries forever.
    if (disconnect_timeout_ticks_ > 0) {
        disconnect_timeout_ticks_--;
        if (disconnect_timeout_ticks_ == 0 &&
            ax25_session_.state() == Ax25SessionState::AWAITING_RELEASE) {
            IRIS_LOG("DISCONNECT TIMEOUT: stuck in AWAITING_RELEASE for 30s, forcing reset");
            if (gui_log_) gui_log_("Disconnect timeout — forcing reset");
            repack_failure_reason_hint_ =
                v2::TransferResultReason::RemoteCloseTimeout;
            ptt_off();  // old transport cleanup precedes terminal callbacks
            ax25_session_.reset();  // Force to DISCONNECTED (triggers state callback → PHY restore)
            return;
        }
    }

    // Suspend AX.25 T1 while auto-tune owns the channel (P0 turnaround fix):
    // the held I-frames physically cannot be transmitted until tune completes
    // (can exceed 30s), so an ACK timer against them fires a spurious
    // TIMER_RECOVERY whose go-back-N recovery burst overflows tx_queue_ and
    // silently evicts never-sent frames (the ics213 drop).  Driven LIVE from
    // tune_state_ so it self-clears the instant tune ends; the post-tune first
    // OFDM burst then re-arms T1 with the airtime floor.  set_tx_deferred() is
    // idempotent (cheap early-return when unchanged), so ticking it is fine.
    {
        bool tune_defers_tx = (tune_state_ != TuneState::IDLE &&
                               tune_state_ != TuneState::DONE);
        ax25_session_.set_tx_deferred(tune_defers_tx);
    }

    // OFDM-KISS T1 watchdog: if native mode is active, session is connected,
    // and T1 isn't running despite unacked frames, force-start T1.
    // Belt-and-suspenders for the T1-never-starts bug (OTA 2026-03-22).
    // Runs every tick (50ms) — start_t1_if_unacked() is a fast no-op when
    // T1 is already running, V(A)==V(S), or TX is deferred for auto-tune.
    if (ofdm_kiss_tx_) {
        ax25_session_.start_t1_if_unacked();
    }

    // Auto-tune FSM tick
    if (tune_state_ != TuneState::IDLE && tune_state_ != TuneState::DONE) {
        tune_timeout_--;
        if (tune_timeout_ <= 0) {
            IRIS_LOG("[TUNE] Timeout (state=%d, frames=%d)",
                     (int)tune_state_.load(), tune_frames_measured_);
            tune_audit("TIMEOUT peer=%s role=%s state=%d my_gain=%.4f peer_gain=%.4f frames=%d",
                       tune_peer_call_.c_str(),
                       tune_is_initiator_ ? "initiator" : "responder",
                       (int)tune_state_.load(), tune_my_gain_, tune_peer_gain_,
                       tune_frames_measured_);

            // Unilateral fallback: if we received and measured the peer's ramp
            // frames but never got their report (report exchange collision),
            // apply corrections from our local measurements.  We know our own
            // TX levels and the peer's measured channel gain from the frames we
            // decoded — that's enough to set RX gain.  TX level stays at probe
            // base (no peer report to refine it).
            if (tune_my_gain_ > 0.01f && tune_frames_measured_ >= 3) {
                IRIS_LOG("[TUNE] Timeout fallback: applying local measurements "
                         "(my_gain=%.4f, %d frames)",
                         tune_my_gain_, tune_frames_measured_);
                if (gui_log_) gui_log_("[TUNE] Timeout — applying local measurements");
                tune_apply_corrections();  // sets DONE
            } else {
                // No usable local data — apply probe-calibrated OFDM base as safe fallback.
                // Without this, tx_level stays at the uncalibrated config value
                // (often 0.88+) which overdrives FM deviation and clips the signal.
                if (ofdm_tx_base_ > 0.01f && config_.tx_level > ofdm_tx_base_ * 1.5f) {
                    float old_level = config_.tx_level;
                    config_.tx_level = ofdm_tx_base_;
                    config_.calibrated_tx_level = ofdm_tx_base_;
                    IRIS_LOG("[TUNE] Timeout fallback: tx_level %.3f -> %.3f (probe-calibrated base)",
                             old_level, ofdm_tx_base_);
                }
                if (gui_log_) gui_log_("[TUNE] Timeout — no response from peer");
                tune_state_ = TuneState::IDLE;
            }
        } else {
            TuneState ts = tune_state_;
            if (ts == TuneState::WAIT_READY) {
                // Initiator: wait for TUNE:READY from responder before sending ramp.
                // Resend TUNE:START every 5s in case responder was still in probe.
                tune_ready_resend_cd_--;
                if (tune_ready_resend_cd_ <= 0) {
                    tune_ready_resends_++;
                    if (tune_ready_resends_ <= 8) {  // max 8 retries (40s)
                        char start_msg[32];
                        snprintf(start_msg, sizeof(start_msg), "TUNE:START=%d", TUNE_RAMP_COUNT);
                        send_tune_ui(start_msg);
                        IRIS_LOG("[TUNE] Resending TUNE:START (attempt %d)", tune_ready_resends_);
                    }
                    tune_ready_resend_cd_ = 100;  // 5s at 50ms/tick
                }
                // Transition to SEND_START happens in handle_tune_frame on TUNE:READY
            } else if (ts == TuneState::SEND_START) {
                // Wait for any pending TX to drain, then build and TX test frames.
                // Initiator enters here after receiving TUNE:READY.
                // Responder enters here after measuring initiator's test frames.
                if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
                    IRIS_LOG("[TUNE] %s: TX queue drained, building test frames",
                             tune_is_initiator_ ? "Initiator" : "Responder");
                    if (gui_log_) gui_log_("[TUNE] Sending test frames...");
                    tune_build_and_queue_test_frame();  // builds all frames + sets tx_pos_=0
                    tune_state_ = TuneState::TX_TEST;
                    // ptt_on() happens automatically in process_tx when tx_pos_==0 && !tx_buffer_.empty()
                }
            } else if (ts == TuneState::TX_TEST) {
                // Wait for test frames to finish transmitting.
                // process_tx clears tx_buffer_ and enters drain automatically,
                // so check that we're back to IDLE (drain complete).
                if (state_ == ModemState::IDLE && tx_buffer_.empty()) {
                    if (tune_is_initiator_) {
                        IRIS_LOG("[TUNE] Test frames sent, waiting for peer");
                        if (gui_log_) gui_log_("[TUNE] Test frames sent, waiting for peer...");
                        tune_state_ = TuneState::WAIT_PEER;
                        tune_wait_peer_ticks_ = 0;
                    } else {
                        // Responder: test frames done — wait for initiator's report.
                        // Lockstep: initiator sends first, responder replies.
                        // This eliminates the half-duplex collision where both sides
                        // send AFSK reports simultaneously and neither receives.
                        IRIS_LOG("[TUNE] Responder: test frames sent, waiting for initiator's report");
                        if (gui_log_) gui_log_("[TUNE] Waiting for peer report...");
                        tune_state_ = TuneState::WAIT_REPORT;
                        tune_report_resend_cd_ = 0;
                        tune_report_resends_ = 0;
                    }
                }
            } else if (ts == TuneState::WAIT_PEER) {
                // Accumulate gain measurements from peer's test frames.
                // Exit conditions (lockstep):
                //  1. Initiator: peer's AFSK ramp report received + local frames measured
                //     → respond immediately (3s silence guard). Don't wait for min_wait.
                //  2. Fallback: enough local frames + silence timeout (peer report late)
                tune_wait_peer_ticks_++;
                // Track silence: ticks since last measurement changed
                if (tune_frames_measured_ != tune_last_measured_count_) {
                    tune_last_measured_count_ = tune_frames_measured_;
                    tune_silence_ticks_ = 0;
                } else {
                    tune_silence_ticks_++;
                }

                // Check if peer's AFSK ramp report has arrived
                bool have_peer_report = false;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                    if (tune_peer_iters_[i] != -1) { have_peer_report = true; break; }
                if (!have_peer_report && tune_peer_gain_ > 0.01f)
                    have_peer_report = true;

                int silence_guard = 60;   // 3.0s silence before TX (avoid collision)

                // Lockstep path: peer's report arrived → respond ASAP.
                bool lockstep_ready = have_peer_report &&
                    tune_frames_measured_ >= 1 && tune_silence_ticks_ >= silence_guard;

                // Fallback path: got at least 1 frame + silence (peer ramp is over).
                // Don't require >= 3 frames — on FM, quieter ramp frames may be
                // below noise floor (OTA: only 2/5 frames detected at 26 dB range).
                // Silence guard is the structural exit; frame count is a sanity check.
                int min_wait = 400;       // 20s minimum (covers ramp ~6s + audio latency ~3s + peer processing)
                bool waited_enough = (tune_wait_peer_ticks_ >= min_wait);
                bool fallback_ready = tune_my_gain_ > 0 && waited_enough &&
                    tune_frames_measured_ >= 1 && tune_silence_ticks_ >= silence_guard;

                bool ready = lockstep_ready || fallback_ready;
                if (ready) {
                    IRIS_LOG("[TUNE] WAIT_PEER done: %d/%d frames measured, avg gain=%.4f "
                             "(waited %d ticks, silence %d, peer_report=%s)",
                             tune_frames_measured_, tune_test_frames_target_,
                             tune_my_gain_, tune_wait_peer_ticks_, tune_silence_ticks_,
                             have_peer_report ? "YES" : "no");
                    tune_last_measured_count_ = 0;
                    tune_silence_ticks_ = 0;
                    if (tune_is_initiator_) {
                        tune_state_ = TuneState::SEND_REPORT;
                    } else {
                        // Responder: TX our test frames first, then send report.
                        IRIS_LOG("[TUNE] Responder: sending test frames first, report after");
                        if (gui_log_) gui_log_("[TUNE] Sending test frames...");
                        tune_test_frames_sent_ = 0;
                        tune_state_ = TuneState::SEND_START;
                    }
                }
            } else if (ts == TuneState::SEND_REPORT) {
                // Send report as an OFDM frame (not AFSK — eliminates collision).
                // Wait for TX queue to drain first (same as SEND_START).
                if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
                    if (ofdm_phy_active_ && ofdm_mod_) {
                        auto report = tune_build_binary_report(
                            tune_rx_frame_iters_, tune_rx_frame_H_, tune_rx_frame_snr_,
                            TUNE_RAMP_COUNT);
                        int rpt_count = 0;
                        for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++)
                            if (tune_rx_frame_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE] Sending OFDM report frame (%d entries, %zu bytes)",
                                 rpt_count, report.size());
                        // Build single OFDM frame at safe tx_level
                        ToneMap tune_map = get_uniform_tone_map(1, ofdm_config_);
                        auto ofdm_iq = ofdm_mod_->build_ofdm_frame(
                            report.data(), report.size(), tune_map);
                        if (!ofdm_iq.empty()) {
                            std::vector<float> audio(ofdm_iq.size());
                            for (size_t j = 0; j < ofdm_iq.size(); j++)
                                audio[j] = ofdm_iq[j].real();
                            // Normalize like ramp frames
                            int pre_samp = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
                            int dstart = std::min(pre_samp, (int)audio.size());
                            float ssq = 0.0f;
                            for (int k = dstart; k < (int)audio.size(); k++)
                                ssq += audio[k] * audio[k];
                            int dlen = (int)audio.size() - dstart;
                            float rms = (dlen > 0) ? std::sqrt(ssq / (float)dlen) : 0.0f;
                            float sc = (rms > 1e-6f) ? (0.50f / rms) : 0.1f;
                            for (int k = dstart; k < (int)audio.size(); k++)
                                audio[k] *= sc;
                            float ppeak = 0.0f;
                            for (int k = 0; k < dstart; k++)
                                ppeak = std::max(ppeak, std::abs(audio[k]));
                            if (ppeak > 0.90f) {
                                float ps = 0.90f / ppeak;
                                for (int k = 0; k < dstart; k++)
                                    audio[k] *= ps;
                            }
                            // TX at mid-range level (safe for both FM clipping and noise floor)
                            float report_tx = std::clamp(
                                (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level,
                                0.15f, 0.8f);
                            for (auto& s : audio) s *= report_tx;
                            for (auto& s : audio) s = std::clamp(s, -0.95f, 0.95f);
                            // Pre/post delay for radio settle
                            int pre_samples = config_.ptt_pre_delay_ms * config_.sample_rate / 1000;
                            int post_samples = std::max(config_.ptt_post_delay_ms, 100) * config_.sample_rate / 1000;
                            tx_buffer_.insert(tx_buffer_.end(), pre_samples, 0.0f);
                            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
                            tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);
                        }
                    } else {
                        // Non-OFDM fallback: AFSK report
                        char buf[64];
                        snprintf(buf, sizeof(buf), "TUNE:GAIN=%.4f", tune_my_gain_);
                        IRIS_LOG("[TUNE] Sending AFSK gain report: %.4f", tune_my_gain_);
                        send_tune_ui(buf);
                        send_tune_ui(buf);
                    }
                    // Fast-track: if peer report already received from embedded ramp payload
                    bool have_peer_ramp = false;
                    for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                        if (tune_peer_iters_[i] != -1) { have_peer_ramp = true; break; }
                    if (have_peer_ramp || tune_peer_gain_ > 0.01f) {
                        IRIS_LOG("[TUNE] Fast-track: peer report already received");
                        tune_state_ = TuneState::APPLY;
                    } else {
                        tune_state_ = TuneState::WAIT_REPORT;
                        tune_report_resend_cd_ = 0;
                        tune_report_resends_ = 0;
                    }
                }
            } else if (ts == TuneState::WAIT_REPORT) {
                // Waiting for peer's OFDM report frame.
                // Initiator: already sent its report via OFDM in SEND_REPORT.
                //   Normally fast-tracked to APPLY because responder's report was
                //   embedded in ramp frames. This state is fallback only.
                // Responder: waiting for initiator's dedicated OFDM report frame.
                //   Handled in OFDM RX path (payload with TUNE_REPORT_MAGIC → APPLY).
                // No AFSK resend logic needed — reports travel via OFDM.
            } else if (ts == TuneState::APPLY) {
                tune_apply_corrections();  // sets DONE
            }
            // (responder TX_TEST completion handled in TX_TEST branch above)
        }
    }
    // Reset DONE state after 5 seconds (100 ticks)
    if (tune_state_ == TuneState::DONE) {
        tune_timeout_--;
        if (tune_post_holdoff_ > 0)
            tune_post_holdoff_--;
        if (tune_timeout_ <= 0) {
            tune_state_ = TuneState::IDLE;
            tune_post_holdoff_ = 0;
        }
    }

    // DCD diagnostics: log state every 5s during active session.
    dcd_diag_ticks_++;
    if (ax25_session_.is_active() && dcd_diag_ticks_ >= 100) {  // 100 ticks = 5s
        dcd_diag_ticks_ = 0;
        IRIS_LOG("[DCD] rms=%.4f inv=%d dcd_hold=%d csma_hold=%d ptt=%d",
                 rx_raw_rms_.load(), (int)dcd_inverted_,
                 dcd_holdoff_.load(), csma_holdoff_.load(), (int)ptt_active_);
    }

    // Probe timeout: if probe doesn't complete in 60s, give up and stay AFSK.
    // Connection is already established — just release held I-frames.
    if (probe_connect_timeout_ > 0) {
        probe_connect_timeout_--;
        if (probe_connect_timeout_ == 0) {
            ofdm_kiss_probing_ = false;
            ofdm_kiss_probe_cd_ = 0;
            probe_start_pending_ = false;
            probe_.reset();
            IRIS_LOG("Probe timeout: staying AFSK, releasing held I-frames");
            if (gui_log_) gui_log_("Probe timeout — staying AFSK");
        }
    }

    // Native hail escalation: after a few AFSK SABM failures, switch to
    // native BPSK hailing. Cancel AFSK retries so native hail gets exclusive
    // TX time — interleaved AFSK+native leaves no RX window for the 717ms
    // native response from the remote.
    if (config_.native_hail &&
        ax25_session_.state() == Ax25SessionState::AWAITING_CONNECTION &&
        ax25_session_.retry_count() >= NATIVE_HAIL_ESCALATION_RETRIES &&
        arq_.state() == ArqState::LISTENING) {
        std::string remote = ax25_session_.remote_callsign();
        int retries = ax25_session_.retry_count();
        IRIS_LOG("Native hail: AFSK SABM failed %d times, escalating to native BPSK",
                 retries);
        // Start native ARQ first so state is HAILING when AX.25 DISCONNECTED
        // callback fires (callback guards against HAILING/CONNECTING).
        arq_connect(remote);
        ax25_session_.reset();  // Stop AFSK retries
    }

    // Probe countdown (shared by manual and auto-probe initiator).
    // Responder is started directly by PROBE:START reception, not by countdown.
    // Resend PROBE:START every ~1s during countdown — the initial pair may be
    // lost if the responder is TX'ing (half-duplex collision).  Spreading
    // retransmissions over the countdown window gives multiple chances for delivery.
    if (ofdm_kiss_probe_cd_ > 0) {
        if (ofdm_kiss_probe_cd_ % 20 == 0 && !ofdm_kiss_probing_) {
            send_probe_start_ui();
            IRIS_LOG("[PROBE] Resending PROBE:START (countdown=%d)", ofdm_kiss_probe_cd_);
        }
        ofdm_kiss_probe_cd_--;
        if (ofdm_kiss_probe_cd_ == 0) {
            if (probe_manual_) {
                // Manual probe button: always initiator, generous capture window.
                probe_.start_initiator(config_.sample_rate, 25.0f);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("[PROBE] Manual probe: sending tones now (25s capture)");
            } else {
                // Auto-probe initiator: use deferred mode — wait for PROBE:READY
                // from responder before sending tones. Eliminates dead time where
                // tones are sent before responder is listening.
                // If responder is old firmware (no READY), controller falls back
                // to old timing (35s capture) after 10s timeout.
                probe_.start_initiator_deferred(config_.sample_rate);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("OFDM-KISS probe: initiator waiting for READY (deferred mode)");
            }
        }
    }

    // OFDM-KISS probe completion: apply discovered passband and baud rate.
    // Both sides run probe_negotiate() on the same two ProbeResults, so the
    // negotiated band is deterministic — no protocol exchange needed.
    if (ofdm_kiss_probing_ && probe_.is_done()) {
        // NOTE: ofdm_kiss_probing_ stays true until the end of this block.
        // The TX gate in process_tx checks ofdm_kiss_probing_ — keeping it
        // true prevents process_tx from draining ax25_tx_queue_ as AFSK
        // before migration moves I-frames to the native tx_queue_.
        ofdm_kiss_probe_done_ = true;
        // Clear DCD holdoff accumulated from probe tones so TX resumes immediately
        dcd_holdoff_ = 0;

        if (probe_.has_results() && probe_.negotiated().valid && !probe_manual_) {
            // Auto-probe: apply negotiated band + baud rate
            float low = probe_.negotiated().low_hz;
            float high = probe_.negotiated().high_hz;
            float bandwidth = high - low;
            config_.band_low_hz = low;
            config_.band_high_hz = high;
            float center = (low + high) / 2.0f;

            if (use_upconvert_) {
                upconverter_ = Upconverter(center, config_.sample_rate);
                downconverter_ = Downconverter(center, config_.sample_rate);

                // SPS sweep: highest baud rate that fits in discovered passband.
                // Both sides compute the same answer from the same negotiated band.
                constexpr float MAX_OCCUPIED_BW_HZ = 20000.0f;
                float usable_bw = std::min(bandwidth - 200.0f, MAX_OCCUPIED_BW_HZ);
                constexpr int SPS_MIN = 6;
                constexpr int SPS_MAX = 80;
                int new_sps = -1;
                int new_baud = 0;
                for (int sps = SPS_MIN; sps <= SPS_MAX; sps++) {
                    int baud = config_.sample_rate / sps;
                    float sig_bw = baud * (1.0f + phy_config_.rrc_alpha);
                    if (sig_bw <= usable_bw) {
                        new_sps = sps;
                        new_baud = baud;
                        break;
                    }
                }
                if (new_sps < 0) {
                    IRIS_LOG("WARNING: no valid SPS for usable BW %.0f Hz — keeping default SPS=%d baud=%d",
                             usable_bw, phy_config_.samples_per_symbol, phy_config_.baud_rate);
                    new_sps = phy_config_.samples_per_symbol;
                    new_baud = phy_config_.baud_rate;
                }

                if (new_baud != phy_config_.baud_rate) {
                    phy_config_.baud_rate = new_baud;
                    phy_config_.samples_per_symbol = new_sps;
                    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
                    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
                    IRIS_LOG("Probe: upgraded baud rate to %d (SPS=%d, sig BW=%.0f Hz)",
                             new_baud, new_sps, new_baud * (1.0f + phy_config_.rrc_alpha));
                }
            }

            // Configure channel equalization from probe tone power data.
            // RX EQ: flatten what we receive (their TX → our RX channel response)
            // TX EQ: pre-compensate what we send (our TX → their RX channel response)
            rx_channel_eq_.configure(probe_.their_tx_result(), probe_.negotiated(), config_.sample_rate, 3.0f);
            tx_channel_eq_.configure(probe_.my_tx_result(), probe_.negotiated(), config_.sample_rate, 6.0f);
            if (rx_channel_eq_.is_configured())
                IRIS_LOG("Probe: RX channel EQ active (%d taps)", (int)rx_channel_eq_.taps().size());
            if (tx_channel_eq_.is_configured())
                IRIS_LOG("Probe: TX channel EQ active (%d taps)", (int)tx_channel_eq_.taps().size());

            IRIS_LOG("Probe complete: band %.0f-%.0f Hz (%.0f Hz BW), center %.0f Hz, baud %d",
                     low, high, bandwidth, center, phy_config_.baud_rate);
            if (gui_log_) {
                char buf[128];
                snprintf(buf, sizeof(buf), "Probe: %.0f-%.0f Hz (%.0f Hz BW) baud %d",
                         low, high, bandwidth, phy_config_.baud_rate);
                gui_log_(buf);
            }

            // Probe confirms peer is Iris — enable native mode
            peer_is_iris_ = true;
            ofdm_kiss_ = true;
            ofdm_config_mismatch_ = false;  // fresh session: re-evaluate config echo (item 5)
            // TX-hold invariant (item 6): only the INITIATOR
            // transmits first. The RESPONDER stays OFDM-RX-only until it decodes
            // the initiator's first OFDM/native frame — deliver_ofdm / deliver
            // (modem.cc:2193/:2499) then flip ofdm_kiss_tx_ and arm native mode.
            // This stops both ends racing to TX before either has confirmed the
            // other's resolved config (connect always establishes a definite
            // initiator = the SABM sender, so exactly one side has we_init).
            {
                bool we_init = ax25_session_.we_initiated();
                ofdm_kiss_tx_ = we_init;
                if (we_init) ax25_session_.set_native_active(true);  // T1 polls in native mode
                // Terminate/re-pack: engage synchronously at initiator activation so
                // the held probe/tune I-frames are absorbed (not flown verbatim)
                // before the first OFDM burst can drain tx_queue_.
                if (we_init) repack_maybe_engage();
            }
            // I-frames may have been sent during AFSK phase (before native_active_).
            // Start T1 now if any are unacked — otherwise T1 never starts and the
            // session hangs forever when OFDM fails (OTA bug 2026-03-22).
            ax25_session_.start_t1_if_unacked();

            // Initialize OFDM PHY if enabled (default).
            // Uses the negotiated passband from probe to configure OFDM carriers.
            if (config_.ofdm_enable) {
                NegotiatedPassband ofdm_pb;
                ofdm_pb.low_hz = low;
                ofdm_pb.high_hz = high;
                ofdm_pb.center_hz = center;
                ofdm_pb.bandwidth_hz = bandwidth;
                ofdm_pb.valid = true;

                // Negotiate OFDM PHY parameters from probe exchange.
                // Both sides advertise their config; we use conservative values
                // (max CP, min pilot spacings) so both sides compute identical config.
                // Old peers (v3 or earlier) have ofdm_* = 0, meaning "use defaults."
                int cp = config_.ofdm_cp_samples;
                int carrier_pilot_spacing = 12; // default: 1:12 comb (52 data carriers; 32QAM r5/8 parity rung)
                int block_pilot_spacing = 24;   // default (was 14, widened for throughput)
                int nfft = config_.ofdm_nfft;
                {
                    const ProbeResult& peer = probe_.my_tx_result();  // peer's advertised config
                    if (peer.ofdm_cp_samples > 0) {
                        // Both sides advertised: use max CP (more conservative/robust)
                        cp = std::max(cp, (int)peer.ofdm_cp_samples);
                    }
                    if (peer.ofdm_pilot_carrier_spacing > 0) {
                        // Use min spacing (more pilots = more robust)
                        carrier_pilot_spacing = std::min(carrier_pilot_spacing, (int)peer.ofdm_pilot_carrier_spacing);
                    }
                    if (peer.ofdm_pilot_symbol_spacing > 0) {
                        block_pilot_spacing = std::min(block_pilot_spacing, (int)peer.ofdm_pilot_symbol_spacing);
                    }
                    {
                        // NFFT: code 0 = ABSENT (old peer / unseeded probe) →
                        // keep the LOCAL default.  Do NOT treat 0 as 512 — that
                        // conflation gave the initiator 1024 and the responder
                        // 512 (deterministic split-brain, 0 delivery over a
                        // bit-exact cable — the D2 failure).  Mirror the
                        // cp/pilot ">0" guards above.
                        // nfft_code: 1=256, 2=1024, 3=512.
                        bool has_ofdm_cfg = (peer.ofdm_nfft_code > 0);
                        if (has_ofdm_cfg) {
                            int peer_nfft = nfft;  // unknown code → local default
                            if (peer.ofdm_nfft_code == 1) peer_nfft = 256;
                            else if (peer.ofdm_nfft_code == 2) peer_nfft = 1024;
                            else if (peer.ofdm_nfft_code == 3) peer_nfft = 512;
                            nfft = std::min(nfft, peer_nfft);
                        }
                    }
                    IRIS_LOG("[OFDM-NEG] peer config: cp=%d pilot=%d block=%d nfft_code=%d",
                             peer.ofdm_cp_samples, peer.ofdm_pilot_carrier_spacing,
                             peer.ofdm_pilot_symbol_spacing, peer.ofdm_nfft_code);
                    IRIS_LOG("[OFDM-NEG] negotiated: cp=%d pilot=%d block=%d nfft=%d",
                             cp, carrier_pilot_spacing, block_pilot_spacing, nfft);
                }

                // Auto-train pilot spacing: measure channel flatness from probe tones.
                // Only used when auto_spacing is enabled AND overrides the negotiated value.
                if (config_.ofdm_auto_spacing) {
                    const ProbeResult& rx_probe = probe_.their_tx_result();
                    float sum = 0, sum2 = 0;
                    int n_det = 0;
                    for (int t = 0; t < 64; t++) {
                        if (rx_probe.tone_detected[t]) {
                            sum += rx_probe.tone_power_db[t];
                            sum2 += rx_probe.tone_power_db[t] * rx_probe.tone_power_db[t];
                            n_det++;
                        }
                    }
                    if (n_det > 2) {
                        float mean = sum / n_det;
                        float var = sum2 / n_det - mean * mean;
                        float std_db = std::sqrt(std::max(0.0f, var));
                        if (std_db < 3.0f) {
                            carrier_pilot_spacing = 12;   // flat: 1:12 comb (52 data carriers, 32QAM parity)
                            block_pilot_spacing = 24;
                        } else if (std_db < 6.0f) {
                            carrier_pilot_spacing = 12;   // moderate
                            block_pilot_spacing = 16;
                        } else {
                            carrier_pilot_spacing = 4;    // rough: tighter pilots for equalization
                            block_pilot_spacing = 8;
                        }
                        IRIS_LOG("[OFDM-AUTO] probe flatness: std=%.1f dB → pilot spacing 1:%d (carrier) 1:%d (block)",
                                 std_db, carrier_pilot_spacing, block_pilot_spacing);
                    }
                }
                ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                                       carrier_pilot_spacing, block_pilot_spacing);
                ofdm_config_.clean_channel = config_.ofdm_clean_channel;
                ofdm_config_.skip_papr_clip = config_.ofdm_skip_papr_clip;
                ofdm_config_.llr_use_frame_nv = config_.ofdm_llr_use_frame_nv;

                // Minimum pilot check: ensure at least 4 pilots for reliable channel estimation
                {
                    int n_used = ofdm_config_.n_used_carriers;
                    int n_pilots = n_used / carrier_pilot_spacing + 1;
                    if (n_pilots < 4 && carrier_pilot_spacing > 2) {
                        carrier_pilot_spacing = std::max(2, n_used / 3);  // force at least 4 pilots
                        IRIS_LOG("[OFDM] pilot spacing reduced to 1:%d for minimum 4 pilots", carrier_pilot_spacing);
                        // D5a: recompute with the NEGOTIATED nfft, not the local
                        // config default — otherwise this branch silently reverts
                        // nfft to config_.ofdm_nfft and re-desyncs the two ends
                        // (the D5a failure).
                        ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                                               carrier_pilot_spacing, block_pilot_spacing);
                        ofdm_config_.clean_channel = config_.ofdm_clean_channel;
                ofdm_config_.skip_papr_clip = config_.ofdm_skip_papr_clip;
                ofdm_config_.llr_use_frame_nv = config_.ofdm_llr_use_frame_nv;
                    }
                }

                // Auto-detect FM pre-emphasis from probe frequency response.
                // CLI --no-preemph (corner=0) overrides auto-detection.
                if (config_.ofdm_preemph_corner_hz == 0.0f) {
                    ofdm_config_.fm_preemph_corner_hz = 0.0f;  // explicit flat
                } else {
                    const ProbeResult& rx_probe = probe_.their_tx_result();
                    ofdm_config_.fm_preemph_corner_hz = probe_detect_preemph_corner(rx_probe);
                }
                ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
                ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
                // [GRID] both-ends grid dump (session-reliability diagnostic).
                // The ZC preamble is generated over used_carrier_bins on the TX
                // and correlated over used_carrier_bins on the RX, so if the two
                // ends resolve different bins the correlation collapses
                // ("FD-ZC too low"). Logging role + resolved grid + fingerprint
                // on BOTH ends makes a split grid trivially greppable: a FAILING
                // session is exactly one where CMD's and RSP's [GRID] lines
                // differ. fp is bit-identical iff the grids match.
                {
                    int fb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.front();
                    int lb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.back();
                    IRIS_LOG("[GRID] role=%s nfft=%d cp=%d n_used=%d bins=%d-%d band=%.1f-%.1f Hz fp=0x%04X",
                             ax25_session_.we_initiated() ? "CMD" : "RSP",
                             ofdm_config_.nfft, ofdm_config_.cp_samples,
                             ofdm_config_.n_used_carriers, fb, lb,
                             ofdm_pb.low_hz, ofdm_pb.high_hz,
                             ofdm_config_.n_used_carriers > 0 ? ofdm_config_fingerprint(ofdm_config_) : 0);
                }
                // Don't set ofdm_phy_active_ yet — wait for CAP_OFDM negotiation below
                ofdm_rx_iq_.clear();
                ofdm_rx_audio_buf_.clear();
                ofdm_acquisition_.reset();
                ofdm_active_candidate_id_ = 0;
                ofdm_rx_lpf_.reset();
                ofdm_sync_cached_ = false;

                // Initialize gearshift OFDM level at O0 (BPSK r1/2) — most robust.
                // Gearshift will negotiate up from RX SNR + LDPC convergence.
                // Cap gearshift max at the highest level whose modulation DENSITY is
                // within config_.max_modulation, so we don't try 1024QAM when the
                // session negotiated QAM256-max.
                //   UPPER-LADDER ROOT (density-cap): the Modulation enum orders
                //   QAM32=6 AFTER QAM1024=5 (native-PHY ordering hack,
                //   constellation.h:24), so the old raw enum compare
                //   (mod > max_modulation) wrongly flagged QAM32(6) > QAM256(4) and
                //   HARD-CAPPED the live gearshift at O5 — blocking O6/32QAM (VARA FM
                //   narrow top gear) AND O7/64QAM entirely, regardless of the SNR
                //   metric. Compare by bits_per_symbol (constellation density) instead:
                //   QAM32=5 <= QAM256=8 admits O6/O7, while QAM1024=10 > 8 stays
                //   excluded (preserves the 1024QAM decode stack-overflow guard).
                //   Env IRIS_DENSITY_MODCAP=0 restores the old enum compare (A/B).
                static const bool density_modcap_gs = []() {
                    const char* e = std::getenv("IRIS_DENSITY_MODCAP");
                    return !e || std::atoi(e) != 0; }();
                int gs_max = NUM_OFDM_SPEED_LEVELS - 1;
                for (int lvl = 0; lvl < NUM_OFDM_SPEED_LEVELS; lvl++) {
                    bool over = density_modcap_gs
                        ? (bits_per_symbol(OFDM_SPEED_LEVELS[lvl].modulation)
                             > bits_per_symbol(config_.max_modulation))
                        : ((int)OFDM_SPEED_LEVELS[lvl].modulation
                             > (int)config_.max_modulation);
                    if (over) { gs_max = lvl - 1; break; }
                }
                if (gs_max < 0) gs_max = 0;
                gearshift_.set_max_ofdm_level(gs_max);
                gearshift_.set_kiss_fast_ramp(false);  // Conservative: +1 per hold, fast fallback on failure
                reset_level_state();  // A4: atomic reset (V1-V6 + ring + gearshift O0)
                ofdm_tone_map_ = get_uniform_tone_map(
                    1, ofdm_config_);  // preset 1 = BPSK r1/2 (O0, start at most robust)
                ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
                ofdm_rx_tone_map_ = ofdm_tone_map_;  // RX starts at O0 too

                IRIS_LOG("OFDM PHY: prepared, %d carriers (%d data, %d pilot), CP=%d, BW=%.0f Hz",
                         ofdm_config_.n_used_carriers, ofdm_config_.n_data_carriers,
                         ofdm_config_.n_pilot_carriers, ofdm_config_.cp_samples,
                         ofdm_config_.bandwidth_hz);
            }

            IRIS_LOG("OFDM-KISS: native mode active (probe-first)");
            if (gui_log_) gui_log_("OFDM-KISS: native mode active");

            // Speed level cache: start at cached level for this peer (if available)
            {
                int cached = gearshift_.load_cached_level(ax25_session_.remote_callsign());
                if (cached > 0) {
                    gearshift_.force_level(cached);
                    IRIS_LOG("Gearshift: cached level %d for %s",
                             cached, ax25_session_.remote_callsign().c_str());
                }
            }

            // Negotiate OFDM-KISS capabilities from probe result exchange.
            // Each side embeds its local caps in the probe result message.
            // my_tx_result = what THEY reported about OUR probe (contains THEIR caps).
            // their_tx_result = what WE measured from THEIR probe (contains OUR caps).
            {
                uint16_t peer_caps = probe_.my_tx_result().capabilities;
                ofdm_kiss_peer_caps_ = local_cap_.capabilities & peer_caps;
                IRIS_LOG("OFDM-KISS caps: local=0x%04X peer=0x%04X negotiated=0x%04X",
                         local_cap_.capabilities, peer_caps, ofdm_kiss_peer_caps_);
                if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
                    ofdm_kiss_tx_compressor_.init();
                    ofdm_kiss_rx_compressor_.init();
                    IRIS_LOG("OFDM-KISS: compression enabled (per-block, no streaming)");
                }
                if (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) {
                    ofdm_kiss_b2f_.init();
                    b2f_proxy_plaintext_.reserve(B2F_BUFFER_SIZE);
                    IRIS_LOG("OFDM-KISS: B2F unroll/reroll enabled");

                    // Replay buffered AFSK I-frame info fields so the B2F
                    // handler sees the SID/FC/FS exchange that happened before
                    // OFDM-KISS activated.  Without this, the handler stays in
                    // B2F_IDLE and never enters PAYLOAD_TRANSFER.
                    if (!b2f_afsk_tx_history_.empty() || !b2f_afsk_rx_history_.empty()) {
                        for (auto& info : b2f_afsk_tx_history_) {
                            auto replayed = ofdm_kiss_b2f_.filter_tx_record(
                                info.data(), info.size());
                            if (replayed.status == v2::TransformStatus::Failed) {
                                ofdm_kiss_b2f_.reset();
                                break;
                            }
                        }
                        for (auto& info : b2f_afsk_rx_history_) {
                            auto replayed = ofdm_kiss_b2f_.filter_rx_record(
                                info.data(), info.size());
                            if (replayed.status == v2::TransformStatus::Failed) {
                                ofdm_kiss_b2f_.reset();
                                break;
                            }
                        }
                        IRIS_LOG("OFDM-KISS: B2F replayed %zu TX + %zu RX AFSK I-frames",
                                 b2f_afsk_tx_history_.size(), b2f_afsk_rx_history_.size());
                        b2f_afsk_tx_history_.clear();
                        b2f_afsk_rx_history_.clear();

                        // Safety: if actual LZHUF payload bytes were already sent
                        // as-is during AFSK, we can't start unrolling mid-stream
                        // (remote would get LZHUF + plaintext mix = corrupt).
                        // But if only FS was parsed (no data bytes consumed yet),
                        // it's safe to start intercepting from here.
                        if (ofdm_kiss_b2f_.has_payload_data_in_flight()) {
                            IRIS_LOG("OFDM-KISS: B2F payload bytes already sent during AFSK — "
                                     "disabling unroll for this session");
                            ofdm_kiss_b2f_.deinit();
                            ofdm_kiss_peer_caps_ &= ~CAP_B2F_UNROLL;
                        } else if (ofdm_kiss_b2f_.is_payload_transfer()) {
                            IRIS_LOG("OFDM-KISS: B2F in PAYLOAD_TRANSFER but no data consumed yet — "
                                     "interception safe, keeping unroll enabled");
                        }
                    }
                }
            }

            // Activate OFDM PHY only if both sides negotiated CAP_OFDM.
            // Objects were prepared above; this gate ensures fallback to legacy
            // single-carrier when the peer doesn't support OFDM.
            if (config_.ofdm_enable && ofdm_mod_ && (ofdm_kiss_peer_caps_ & CAP_OFDM)) {
                ofdm_phy_active_ = true;
                // O0: 1 codeword RATE_1_2, capacity 94 bytes. Minus 3 batch + 16 AX.25 = 75.
                // Dynamic MAX_INFO (leg 3): size to the CONFIRMED anchor (just
                // reset to O0 here → 75). Grows as tx_acked_level_ climbs (see the
                // anchor-advance re-calls) so I-frames fill the larger frame.
                int ofdm_max_info = ofdm_max_info_for_level(tx_acked_level_, ofdm_config_);
                ax25_session_.set_max_info(ofdm_max_info);
                ax25_session_.drop_oversized_in_window();
                tx_level_ring_.reset();  // A5d: purged window frames must not credit the anchor
                IRIS_LOG("OFDM PHY: ACTIVE (CAP_OFDM negotiated, %d data carriers, BW=%.0f Hz)",
                         ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz);
                // Initialize MFSK tone ACK: place M=16 tones centered in OFDM band
                {
                    assert_mfsk_nfft(ofdm_config_.nfft);  // tone-ACK bin-unit invariant (§1.9)
                    int center_bin = freq_to_bin(ofdm_config_.center_hz, ofdm_config_.nfft, config_.sample_rate);
                    int first_bin = center_bin - MfskAck::M / 2;
                    if (first_bin < 1) first_bin = 1;
                    mfsk_ack_.init(first_bin, config_.sample_rate);
                }
                if (gui_log_) {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "OFDM PHY: %d carriers, BW=%.0f Hz (%.0f bps ceiling)",
                             ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz,
                             tone_map_throughput(ofdm_tone_map_, ofdm_config_));
                    gui_log_(buf);
                }
            } else if (config_.ofdm_enable && ofdm_mod_) {
                IRIS_LOG("OFDM PHY: peer lacks CAP_OFDM — using legacy single-carrier PHY");
                ofdm_mod_.reset();
                ofdm_demod_.reset();
            }
            v2_negotiated_active_ = ofdm_phy_active_ &&
                (ofdm_kiss_peer_caps_ & CAP_OFDM) != 0;

            // Migrate I-frames from ax25_tx_queue_ (AFSK) to tx_queue_ (native).
            // During the probe, ofdm_kiss_tx_ was false so the send_frame_
            // callback routed I-frames to ax25_tx_queue_.  Now that native mode
            // is active, move them so they go out as native frames — not as a
            // long AFSK burst that would bury the probe result UI frame.
            // Drop oversized I-frames (queued before max_info reduction) — AX.25
            // retry will retransmit them at the correct size.
            {
                int migrated = 0, dropped = 0;
                int ofdm_capacity =
                    ofdm_capacity_bytes_for_level(0, ofdm_config_);  // 94 bytes
                std::queue<std::vector<uint8_t>> keep;
                while (!ax25_tx_queue_.empty()) {
                    auto frame = std::move(ax25_tx_queue_.front());
                    ax25_tx_queue_.pop();
                    // §3.1 landmine fixed: parse-based predicate (modem.h doc) —
                    // the old byte-14 sniff aliased via-carrying frames
                    // (digipeat re-emits, endpoint-via) as I-frames.
                    if (migrate_to_native_eligible(frame)) {
                        if ((int)frame.size() > ofdm_capacity) {
                            dropped++;  // Too large for OFDM — let AX.25 retry at correct size
                            continue;
                        }
                        constexpr size_t TX_QUEUE_MAX = 32;
                        if (tx_queue_.size() >= TX_QUEUE_MAX) {
                            IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                            tx_queue_.pop();
                        }
                        tx_queue_.push(TxFrame(std::move(frame)));  // I-frame: never tone-eligible
                        migrated++;
                    } else {
                        keep.push(std::move(frame));
                    }
                }
                ax25_tx_queue_ = std::move(keep);
                if (migrated > 0 || dropped > 0) {
                    IRIS_LOG("OFDM-KISS: migrated %d I-frames to native queue (%d dropped oversized)",
                             migrated, dropped);
                }
            }

            // Mandatory listen window after probe: don't TX immediately.
            // The peer may be sending its response (UA, held frames) right now.
            // Without this, our TX triggers a self-hear guard that discards the
            // peer's native frame before we can decode it.
            csma_holdoff_ = config_.sample_rate * 3;  // 3s listen before first TX

            // Connection already established before probe.
            // Held I-frames release via native after listen window expires.
            // T1 is already set to 2.0s by set_native_active(true) above —
            // don't override with the AFSK-era 15s value.
            probe_connect_timeout_ = 0;
            IRIS_LOG("Probe done — releasing held I-frames via native");

            // Cache probe result for this peer (24h expiry, skip re-probe on reconnect)
            cache_probe_result(ax25_session_.remote_callsign());

            // Adaptive TXDELAY: after successful probe, reduce OFDM TX pre-delay.
            // Probe success proves radio link works; training symbols handle the rest.
            ofdm_txdelay_ms_ = std::max(50, config_.ptt_pre_delay_ms / 2);
            IRIS_LOG("[TXDELAY] adaptive: %d ms -> %d ms (post-probe)",
                     config_.ptt_pre_delay_ms, ofdm_txdelay_ms_);

            // Pre-calibrate OFDM TX level from probe data.
            //
            // Problem: TUNE needs OFDM frames to decode, but OFDM frames can't
            // decode if tx_level is wrong (too loud → FM clipping destroys ZC
            // preamble). Chicken-and-egg.
            //
            // Solution: probe tones already traversed the channel successfully.
            // Both probe and OFDM go through the same FM audio path, so the path
            // gain cancels — we just need to match output levels with a PAPR
            // correction. TUNE will refine from this starting point.
            //
            // Probe output: 64 continuous sinusoids, PAPR ~3 dB
            //   Total RMS ≈ amplitude/sqrt(2) * tx_level ≈ 0.354 * tx_level
            //   Peak ≈ RMS * 10^(3/20) = 0.354 * tx_level * 1.41
            //
            // OFDM peak matching: OFDM peaks ≤ probe peaks.
            // DFT-spread (always on) reduces PAPR dramatically:
            //   BPSK/QPSK + pilots: ~3 dB → ratio = 0.354*1.41 / (0.50*1.41) = 0.71
            //   16QAM+:             ~5 dB → ratio = 0.354*1.41 / (0.50*1.78) = 0.56
            // Without DFT-spread:   ~8 dB → ratio = 0.354*1.41 / (0.50*2.51) = 0.40
            {
                float ratio = 0.71f;  // DFT-spread default (BPSK/QPSK)
                float ofdm_tx = ratio * config_.tx_level;
                ofdm_tx = std::clamp(ofdm_tx, 0.05f, 0.50f);
                ofdm_tx_base_ = ofdm_tx;

                // Log peer's measured tone levels for diagnostics (path gain info)
                const ProbeResult& peer_heard = probe_.my_tx_result();
                float avg_rx_db = -99.0f;
                int count = 0;
                if (peer_heard.valid && peer_heard.tones_detected > 0) {
                    float sum_rx_db = 0;
                    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                        if (!peer_heard.tone_detected[k]) continue;
                        float freq = probe_tone_freq(k);
                        if (freq < low - 25.0f || freq > high + 25.0f) continue;
                        sum_rx_db += peer_heard.tone_power_db[k];
                        count++;
                    }
                    if (count > 0) avg_rx_db = sum_rx_db / count;
                }

                IRIS_LOG("[PROBE-CAL] OFDM tx_level: %.3f (from %.3f, ratio=%.2f, DFT-spread PAPR)",
                         ofdm_tx, config_.tx_level, ratio);
                IRIS_LOG("[PROBE-CAL]   peer avg tone power: %.1f dBFS (%d tones in band)",
                         avg_rx_db, count);
            }

            // Configure OFDM RX lowpass filter to remove f² discriminator noise.
            // Flat 9600-baud radio ports output raw discriminator with PSD ∝ f².
            // Wideband noise (3-12+ kHz) crushes SC/ZC broadband correlation.
            // 2nd-order Butterworth LPF: ~3 samples GD at cutoff (CP=64 gives huge margin),
            // -12 dB/oct rolloff, -22 dB at 12 kHz.
            {
                float fc = high + 500.0f;  // 500 Hz margin above negotiated band edge
                fc = std::clamp(fc, 2000.0f, 5000.0f);
                const float fs = 48000.0f;
                const float Q = 0.7071f;  // Butterworth
                float w0 = 2.0f * (float)M_PI * fc / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q);
                float a0 = 1.0f + alpha;
                ofdm_rx_lpf_ = {};
                ofdm_rx_lpf_.b0 = ((1.0f - c) / 2.0f) / a0;
                ofdm_rx_lpf_.b1 = (1.0f - c) / a0;
                ofdm_rx_lpf_.b2 = ((1.0f - c) / 2.0f) / a0;
                ofdm_rx_lpf_.a1 = (-2.0f * c) / a0;
                ofdm_rx_lpf_.a2 = (1.0f - alpha) / a0;
                ofdm_rx_lpf_active_ = true;
                IRIS_LOG("[OFDM-LPF] RX lowpass configured: fc=%.0f Hz (band edge %.0f + 500 Hz), 2nd-order Butterworth",
                         fc, high);
            }

            // CONNECT DIET + FASTER CLIMB (connshave): decide once whether the
            // probe found a CLEAN HIGH-SNR link.  Genuine two-sided signal (not a
            // timeout band-aid): the reverse-path SNR we measured of the peer's
            // probe (their_tx_result_.est_snr_db) is at/above the clean threshold
            // AND the forward path the peer measured of our probe survived on most
            // of the in-band tones (my_tx_result_.tones_detected).  low/high are
            // the negotiated band (in scope in this auto-probe branch).
            bool probe_clean = false;
            {
                const ProbeResult& rev = probe_.their_tx_result();  // we heard peer (RX SNR)
                const ProbeResult& fwd = probe_.my_tx_result();      // peer heard us (TX path)
                int band_tones = 0;
                for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                    float f = probe_tone_freq(k);
                    if (f >= low - 25.0f && f <= high + 25.0f) band_tones++;
                }
                int fwd_need = (band_tones * 3) / 4;   // 75% of in-band tones survived fwd
                probe_clean = rev.valid && fwd.valid &&
                              rev.est_snr_db >= tune_skip_snr_db_ &&
                              fwd.tones_detected >= fwd_need;
                IRIS_LOG("[CONNDIET] probe clean=%d: rev_snr=%.1f dB (>= %.1f) "
                         "fwd_tones=%d/%d (need %d)",
                         (int)probe_clean, rev.est_snr_db, tune_skip_snr_db_,
                         fwd.tones_detected, band_tones, fwd_need);
            }

            // LEVER A — connect diet: on a clean probe, skip the unconditional
            // ~40s auto-tune.  Apply the probe-calibrated base as the operating
            // level (bit-identical to the tune's own timeout fallback, :5352-5358)
            // so FM deviation stays safe; ofdm_effective_tx_level() reads
            // ofdm_tx_base_ regardless, so OFDM TX is unaffected by the skip.
            // Auto-tune after probe: gain characteristics change with bandwidth/baud,
            // so recalibrate TX level for the newly discovered passband.
            // Only the probe initiator triggers auto-tune — the responder will
            // enter responder mode when it receives TUNE:START from us.
            // TUNE:START frames queue behind csma_holdoff_ (3s listen window above),
            // and the SEND_START tick handler waits for drain before sending test frames.
            if (ax25_session_.we_initiated() && !config_.skip_tune) {
                std::string peer = ax25_session_.remote_callsign();
                if (!peer.empty()) {
                    if (connect_diet_ && probe_clean) {
                        if (ofdm_tx_base_ > 0.01f) {
                            config_.tx_level = ofdm_tx_base_;
                            config_.calibrated_tx_level = ofdm_tx_base_;
                        }
                        IRIS_LOG("[TUNE] Skipped (connect-diet, clean probe) — tx_level=%.3f",
                                 config_.tx_level);
                        if (gui_log_) gui_log_("[TUNE] Skipped — clean link");
                    } else {
                        IRIS_LOG("[TUNE] Triggering auto-tune after probe (peer=%s)", peer.c_str());
                        start_autotune(peer);
                    }
                }
            } else if (config_.skip_tune) {
                IRIS_LOG("[TUNE] Skipped (--skip-tune), using tx_level=%.3f", config_.tx_level);
            }

            // LEVER B — faster climb: on a clean probe, shorten the OFDM gearshift
            // hold (frames-per-rung) on BOTH ends so the receiver's decode-margin
            // climb proposes the next rung in fewer decoded frames.  The proposal
            // still only advances +1 per step (margin-climb target is capped at
            // ofdm_level+1), so it stays coupled to the sender's actual level — the
            // sender leaps toward it OFDM_LEVEL_LEAP_MAX/round and the RX
            // blind-detect never sees a proposal running rungs ahead of the frame
            // on the air.  reset_level_state() restored the default hold above.
            if (clean_climb_ && probe_clean) {
                gearshift_.set_ofdm_hold_frames(clean_climb_hold_);
                IRIS_LOG("[CONNDIET] clean-climb: OFDM gearshift hold -> %d frames/rung "
                         "(from 5 baseline)", clean_climb_hold_);
            }
        } else if (probe_.has_results() && probe_.negotiated().valid) {
            // Manual probe: log only, no PHY change
            float low = probe_.negotiated().low_hz;
            float high = probe_.negotiated().high_hz;
            float bandwidth = high - low;
            IRIS_LOG("Probe complete (manual): band %.0f-%.0f Hz (%.0f Hz BW), no PHY change",
                     low, high, bandwidth);
            if (gui_log_) {
                char buf[128];
                snprintf(buf, sizeof(buf), "Probe: %.0f-%.0f Hz (%.0f Hz BW)", low, high, bandwidth);
                gui_log_(buf);
            }
        } else {
            IRIS_LOG("Probe complete: no valid results — staying AX.25");
            if (gui_log_) gui_log_("Probe: no valid results");
            probe_connect_timeout_ = 0;
            // Connection already established — I-frames release automatically
            // (ofdm_kiss_probing_ goes false at end of block, TX gate opens)
        }
        // NOW release the TX gate: ofdm_kiss_probing_=false allows process_tx
        // to resume.  Migration (if applicable) already moved I-frames to
        // tx_queue_, so they'll go out as native instead of AFSK.
        ofdm_kiss_probing_ = false;
        probe_manual_ = false;
    }

}

ModemDiag Modem::get_diagnostics() const {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    ModemDiag diag;
    diag.state = state_;
    diag.speed_level = gearshift_.current_level();
    diag.ofdm_speed_level = gearshift_.current_ofdm_level();
    diag.ofdm_active = ofdm_phy_active_;
    diag.snr_db = gearshift_.smoothed_snr();
    diag.agc_gain = agc_.gain();
    diag.tx_level = config_.tx_level;
    diag.native_rx_gain = native_rx_gain_;
    diag.kiss_clients = 0;
    diag.frames_rx = frames_rx_;
    diag.frames_tx = frames_tx_;
    diag.crc_errors = crc_errors_;
    diag.retransmits = arq_.retransmit_count();
    diag.rx_rms = rx_rms_;
    diag.rx_peak = rx_peak_;
    diag.ptt_active = ptt_active_;
    diag.cal_state = cal_state_;
    diag.cal_measured_rms = cal_measured_rms_;
    diag.arq_state = arq_.state();
    diag.arq_role = arq_.role();
    diag.ax25_state = ax25_session_.state();
    {
        bool busy = false;
        if (!ofdm_kiss_probing_ && config_.dcd_threshold > 0 && (native_mode_ || ofdm_kiss_)) {
            if (dcd_inverted_)
                busy = rx_raw_rms_ < dcd_baseline_rms_ * 0.5f;
            else
                busy = rx_raw_rms_ > config_.dcd_threshold;
            busy = busy || (dcd_holdoff_ > 0);
        }
        diag.dcd_busy = busy;
    }
    diag.rx_raw_rms = rx_raw_rms_;
    diag.dcd_tone_energy = afsk_demod_.tone_energy();

    diag.constellation = last_constellation_;
    diag.kalman_trace = last_kalman_trace_;
    diag.spectrum = last_spectrum_;

    // Probe results
    diag.probe_state = probe_.state();
    diag.probe_has_results = probe_.has_results();
    if (diag.probe_has_results) {
        diag.probe_my_tx = probe_.my_tx_result();
        diag.probe_their_tx = probe_.their_tx_result();
        diag.probe_negotiated = probe_.negotiated();
    }

    // Extended diagnostics
    diag.native_mode = native_mode_ || ofdm_kiss_tx_;
    diag.bytes_rx = bytes_rx_;
    diag.bytes_tx = bytes_tx_;
    diag.phy_bps = net_throughput(diag.speed_level, phy_config_.baud_rate);
    diag.app_bps = diag.phy_bps;  // Same as PHY unless compression active
    diag.compression_ratio = tx_compressor_.last_ratio();
    if (diag.compression_ratio > 1.0f)
        diag.app_bps = (int)(diag.phy_bps * diag.compression_ratio);
    diag.encryption_state = crypto_state_;
    diag.band_low_hz = config_.band_low_hz;
    diag.band_high_hz = config_.band_high_hz;
    diag.baud_rate = phy_config_.baud_rate;
    diag.spectrum_low_hz = spectrum_low_hz_;
    diag.spectrum_high_hz = spectrum_high_hz_;

    return diag;
}

void Modem::compute_spectrum(const float* audio, int count) {
    // Downsample 48k→8k then FFT. At 8 kHz, NFFT=512 gives:
    //   15.6 Hz bins, 256 bins covering 0-4 kHz (pocketfft O(N log N)).
    constexpr int DS_RATE = 8000;
    constexpr int NFFT = 512;

    int ratio = config_.sample_rate / DS_RATE;
    int need = NFFT * ratio;  // 3072 samples at 48k

    // Accumulate raw audio
    if ((int)spectrum_buf_.size() < spectrum_buf_pos_ + count)
        spectrum_buf_.resize(spectrum_buf_pos_ + count);
    memcpy(spectrum_buf_.data() + spectrum_buf_pos_, audio, count * sizeof(float));
    spectrum_buf_pos_ += count;

    if (spectrum_buf_pos_ < need) return;

    // Decimate: pick every Nth sample (radio audio is already bandlimited)
    float ds[NFFT];
    const float* src = spectrum_buf_.data() + spectrum_buf_pos_ - need;
    for (int i = 0; i < NFFT; i++)
        ds[i] = src[i * ratio];

    spectrum_buf_pos_ = 0;

    // Hann window + FFT (O(N log N) via pocketfft, replaces O(N²) naive DFT)
    int n_pos = NFFT / 2;
    std::complex<float> fft_buf[NFFT];
    for (int n = 0; n < NFFT; n++) {
        float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * n / (NFFT - 1)));
        fft_buf[n] = std::complex<float>(ds[n] * w, 0.0f);
    }
    fft_complex(fft_buf, NFFT);

    std::vector<float> spec(n_pos, 0.0f);
    for (int k = 0; k < n_pos; k++) {
        float pwr = std::norm(fft_buf[k]) / (float)(NFFT * NFFT);
        spec[k] = 10.0f * std::log10(std::max(pwr, 1e-12f));
    }

    // Caller (process_rx) holds modem_mutex_
    last_spectrum_ = std::move(spec);
    spectrum_low_hz_ = 0;
    spectrum_high_hz_ = DS_RATE / 2.0f;
}

// --- Auto-Tune (bilateral native-frame gain calibration) ---
// Protocol (half-duplex safe — reports sent only when peer is listening):
//   Initiator: clicks Auto Tune → sends TUNE:START UI → TX test frame(s) →
//              waits for responder's test frame(s) → sends report → waits → DONE.
//   Responder: receives TUNE:START → waits for initiator's test frame(s) →
//              TX own test frame(s) → sends report → waits for initiator's report → DONE.
//   Both reports are sent AFTER all TX is complete, avoiding half-duplex collision.
//
// When OFDM is active, test frames are OFDM power-ramp frames (BPSK r1/2) at
// varying TX levels. Peer measures LDPC quality + |H| for each; parabolic fit
// finds optimal drive. Legacy native mode uses single-level gain targeting.

void Modem::send_tune_ui(const char* payload) {
    // Caller holds modem_mutex_
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("TUNE");
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    size_t plen = strlen(payload);
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + plen);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

void Modem::tune_build_and_queue_test_frame() {
    // Build N test frames with known payload into tx_buffer_.
    // When OFDM is active, build OFDM frames (Schmidl-Cox preamble) so the
    // peer's OFDM RX can detect them. Otherwise, build native Mode A frames.
    // Caller must hold modem_mutex_.
    //
    // Responder embeds its measurements of the initiator's ramp frames in the
    // payload of each ramp frame (binary report). This eliminates the AFSK
    // report exchange — the initiator gets the report by decoding the ramp.
    const uint8_t default_payload[] = "TUNE_TEST_FRAME";
    std::vector<uint8_t> report_payload;
    if (!tune_is_initiator_ && tune_frames_measured_ > 0) {
        report_payload = tune_build_binary_report(
            tune_rx_frame_iters_, tune_rx_frame_H_, tune_rx_frame_snr_,
            TUNE_RAMP_COUNT);
        IRIS_LOG("[TUNE] Responder: embedding %d-frame report in ramp payload (%zu bytes)",
                 tune_frames_measured_, report_payload.size());
    }
    const uint8_t* payload_data = report_payload.empty()
        ? default_payload : report_payload.data();
    size_t payload_len = report_payload.empty()
        ? sizeof(default_payload) - 1 : report_payload.size();

    tx_buffer_.clear();

    for (int i = 0; i < tune_test_frames_target_; i++) {
        if (ofdm_phy_active_ && ofdm_mod_) {
            // OFDM power-ramp TUNE: each of the N frames is sent at a different
            // TX level. Peer measures LDPC quality for each and reports which
            // one decoded best. This finds the optimal drive level in a single
            // exchange (5 frames via TUNE_RAMP_COUNT).
            // BPSK r1/2 for TUNE: maximum robustness for TX power calibration.
            // Explicit r1/2 overrides the preset's FEC rate.
            ToneMap tune_map = get_uniform_tone_map(1, ofdm_config_);
            auto ofdm_iq = ofdm_mod_->build_ofdm_frame(
                payload_data, payload_len, tune_map);
            if (ofdm_iq.empty()) {
                IRIS_LOG("[TUNE] Failed to build OFDM test frame %d", i + 1);
                continue;
            }
            // Extract real passband
            std::vector<float> audio(ofdm_iq.size());
            for (size_t j = 0; j < ofdm_iq.size(); j++)
                audio[j] = ofdm_iq[j].real();
            // Normalize data portion to 0.50 RMS, preamble peak to ±0.90
            int pre_samp = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
            int dstart = std::min(pre_samp, (int)audio.size());
            float ssq = 0.0f;
            for (int k = dstart; k < (int)audio.size(); k++)
                ssq += audio[k] * audio[k];
            int dlen = (int)audio.size() - dstart;
            float rms = (dlen > 0) ? std::sqrt(ssq / (float)dlen) : 0.0f;
            float sc = (rms > 1e-6f) ? (0.50f / rms) : 0.1f;
            for (int k = dstart; k < (int)audio.size(); k++)
                audio[k] *= sc;
            float ppeak = 0.0f;
            for (int k = 0; k < dstart; k++)
                ppeak = std::max(ppeak, std::abs(audio[k]));
            if (ppeak > 0.90f) {
                float ps = 0.90f / ppeak;
                for (int k = 0; k < dstart; k++)
                    audio[k] *= ps;
            }

            // Apply power-ramp level for this frame: scales × base.
            // Use probe-calibrated ofdm_tx_base_ if available, else config_.tx_level.
            // Record actual tx_level used so parabolic fit can map back to levels.
            float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
            float ramp_scale = (i < TUNE_RAMP_COUNT) ? tune_computed_scales_[i] : 1.0f;
            float frame_tx = std::clamp(ramp_base * ramp_scale, 0.05f, 1.0f);
            if (i < TUNE_RAMP_COUNT) tune_ramp_tx_levels_[i] = frame_tx;
            for (auto& s : audio) s *= frame_tx;

            // Hard clip to soundcard range
            for (auto& s : audio) {
                if (s > 0.95f) s = 0.95f;
                else if (s < -0.95f) s = -0.95f;
            }
            IRIS_LOG("[TUNE] OFDM ramp frame %d: scale=%.2f tx=%.3f (base=%.3f) %zu samples",
                     i + 1, ramp_scale, frame_tx, ramp_base, audio.size());
            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
            tune_test_frames_sent_++;
        } else {
            // Native Mode A TUNE frame
            auto iq = build_native_frame(payload_data, payload_len,
                                          phy_config_, LdpcRate::RATE_1_2);
            if (iq.empty()) {
                IRIS_LOG("[TUNE] Failed to build native test frame %d", i + 1);
                continue;
            }

            std::vector<float> audio;
            if (use_upconvert_) {
                audio = upconverter_.iq_to_audio(iq.data(), iq.size());
                if (tx_channel_eq_.is_configured()) {
                    tx_channel_eq_.apply(audio.data(), (int)audio.size());
                    for (auto& s : audio) {
                        if (s > 0.95f) s = 0.95f + 0.05f * std::tanh((s - 0.95f) / 0.05f);
                        else if (s < -0.95f) s = -0.95f + 0.05f * std::tanh((s + 0.95f) / 0.05f);
                    }
                }
            } else {
                audio.resize(iq.size() / 2);
                for (size_t j = 0; j < iq.size() / 2; j++)
                    audio[j] = iq[2 * j];
            }
            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
            tune_test_frames_sent_++;
        }
    }

    // Apply tx_level to native TUNE frames only.
    // OFDM ramp frames already have per-frame tx_level baked in above.
    if (!(ofdm_phy_active_ && ofdm_mod_)) {
        for (auto& s : tx_buffer_) s *= config_.tx_level;
    }

    // TUNE uses OFDM frames — need full TXDELAY for radio settle
    int pre_ms = config_.ptt_pre_delay_ms;
    int pre_samples = pre_ms * config_.sample_rate / 1000;
    if (pre_samples > 0)
        tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);
    int post_ms = std::max(config_.ptt_post_delay_ms, 100);  // min 100ms tail
    int post_samples = post_ms * config_.sample_rate / 1000;
    if (post_samples > 0)
        tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

    tx_pos_ = 0;
    state_ = ModemState::TX_NATIVE;
    frames_tx_++;

    IRIS_LOG("[TUNE] Built %d test frames, %zu samples (%d ms, pre=%dms post=%dms)",
             tune_test_frames_sent_, tx_buffer_.size(),
             (int)(tx_buffer_.size() * 1000 / config_.sample_rate),
             pre_ms, post_ms);
}

void Modem::handle_tune_frame(const uint8_t* info, size_t len) {
    std::string payload((const char*)info, len);

    if (payload.find("TUNE:START") != std::string::npos) {
        if (config_.skip_tune) {
            IRIS_LOG("[TUNE] Ignoring TUNE:START (--skip-tune)");
            return;
        }
        // Must have OFDM active to decode ramp frames — ignore if still in probe.
        // Initiator retries TUNE:START every 5s; we'll accept once probe completes.
        if (!ofdm_phy_active_) {
            IRIS_LOG("[TUNE] Ignoring TUNE:START — OFDM not active yet (probe incomplete?)");
            return;
        }
        // Remote (initiator) is starting or restarting tune — enter responder mode.
        // Accept in IDLE or any active TUNE state (initiator may be retrying with
        // more frames, which resets our state machine).
        if (tune_state_ == TuneState::IDLE ||
            (tune_state_ != TuneState::DONE && !tune_is_initiator_)) {
            if (tune_state_ != TuneState::IDLE)
                IRIS_LOG("[TUNE] Received TUNE:START while in state %d — resetting as responder",
                         (int)tune_state_.load());
            else
                IRIS_LOG("[TUNE] Received TUNE:START — entering responder mode");
            if (gui_log_) gui_log_("[TUNE] Remote requested auto-tune");
            tune_audit("=== TUNE START === local=%s role=responder tx_level=%.3f rx_gain=%.3f",
                       config_.callsign.c_str(), config_.tx_level, native_rx_gain_);
            tune_state_ = TuneState::WAIT_PEER;
            tune_is_initiator_ = false;
            tune_peer_call_ = ax25_session_.remote_callsign();
            tune_my_gain_ = 0;
            tune_peer_gain_ = 0;
            tune_test_frames_sent_ = 0;
            tune_frames_measured_ = 0;
            tune_wait_peer_ticks_ = 0;
            tune_last_measured_count_ = 0;
            tune_silence_ticks_ = 0;
            // Parse ramp count from "TUNE:START=N" (default 10 for compat)
            int rc = TUNE_RAMP_COUNT;
            {
                size_t eq = payload.find("TUNE:START=");
                if (eq != std::string::npos) {
                    int parsed = atoi(payload.c_str() + eq + 11);
                    if (parsed >= 1 && parsed <= TUNE_RAMP_COUNT) rc = parsed;
                }
            }
            tune_test_frames_target_ = rc;
            // Compute adaptive ramp scales based on current TX base
            {
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                tune_compute_scales(ramp_base);
            }
            // Send TUNE:READY so initiator knows we're listening
            send_tune_ui("TUNE:READY");
            send_tune_ui("TUNE:READY");  // twice for reliability
            IRIS_LOG("[TUNE] Sent TUNE:READY to initiator");
            // Responder timeout: generous — must cover initiator TX + our TX +
            // 2s stagger delay + reports + retransmits.
            tune_timeout_ = 1200;  // 60s at 50ms/tick
            IRIS_LOG("[TUNE] Ramp count: %d, timeout: 60s", rc);
            for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                tune_rx_frame_iters_[i] = -1;
                tune_rx_frame_H_[i] = 0;
                tune_rx_frame_snr_[i] = -99.0f;
                tune_peer_iters_[i] = -1;
                tune_peer_H_[i] = 0;
                tune_peer_snr_[i] = -99.0f;
                tune_ramp_tx_levels_[i] = 0;
            }
        }
    } else if (payload.find("TUNE:READY") != std::string::npos) {
        // Responder is ready — initiator can start sending ramp frames.
        if (tune_is_initiator_ && tune_state_ == TuneState::WAIT_READY) {
            IRIS_LOG("[TUNE] Received TUNE:READY — responder is listening, starting ramp");
            if (gui_log_) gui_log_("[TUNE] Peer ready, sending test frames...");
            tune_test_frames_sent_ = 0;
            tune_state_ = TuneState::SEND_START;
        } else {
            IRIS_LOG("[TUNE] Ignoring TUNE:READY (state=%d, initiator=%d)",
                     (int)tune_state_.load(), tune_is_initiator_);
        }
    } else if (payload.find("TUNE:R=") != std::string::npos ||
               payload.find("TUNE:RAMP") != std::string::npos) {
        // OFDM power-ramp report. Two formats:
        //   Compact: "TUNE:R=idx:iters:H,idx:iters:H,..."  (only measured entries)
        //   Legacy:  "TUNE:RAMPN=i0,h0,i1,h1,...,iN-1,hN-1" (all N entries)
        int parsed_count = 0;
        size_t cpos = payload.find("TUNE:R=");
        if (cpos != std::string::npos) {
            // Compact format: parse idx:iters:H[:SNR] (3 or 4 fields)
            const char* cp = payload.c_str() + cpos + 7;  // skip "TUNE:R="
            while (*cp) {
                int idx, iters;
                float h, snr = -99.0f;
                int nf = sscanf(cp, "%d:%d:%f:%f", &idx, &iters, &h, &snr);
                if (nf < 3) break;  // need at least idx:iters:H
                if (idx >= 0 && idx < TUNE_RAMP_COUNT) {
                    tune_peer_iters_[idx] = iters;
                    tune_peer_H_[idx] = h;
                    tune_peer_snr_[idx] = (nf >= 4) ? snr : -99.0f;
                    parsed_count++;
                }
                cp = strchr(cp, ',');
                if (cp) cp++; else break;
            }
        } else {
            // Legacy RAMPN format
            size_t rpos = payload.find("TUNE:RAMP");
            const char* p = payload.c_str() + rpos + 9;
            int peer_ramp_count = atoi(p);
            const char* eq = strchr(p, '=');
            if (eq && peer_ramp_count >= 1 && peer_ramp_count <= TUNE_RAMP_COUNT) {
                const char* cp = eq + 1;
                for (int i = 0; i < peer_ramp_count; i++) {
                    int iters; float h;
                    if (sscanf(cp, "%d,%f", &iters, &h) < 2) break;
                    tune_peer_iters_[i] = iters;
                    tune_peer_H_[i] = h;
                    parsed_count++;
                    for (int skip = 0; skip < 2 && *cp; skip++) {
                        cp = strchr(cp, ',');
                        if (cp) cp++; else break;
                    }
                }
            }
        }
        if (parsed_count == 0) {
            IRIS_LOG("[TUNE] Invalid RAMP report format");
            return;
        }
        IRIS_LOG("[TUNE] Peer ramp report: %d entries (state=%d)",
                 parsed_count, (int)tune_state_.load());
        tune_audit("RAMP_REPORT entries=%d state=%d",
                   parsed_count, (int)tune_state_.load());

        // AFSK report fallback (non-OFDM mode only; OFDM uses embedded binary reports)
        if (tune_state_ == TuneState::WAIT_REPORT) {
            tune_state_ = TuneState::APPLY;
        } else if (tune_state_ == TuneState::WAIT_PEER) {
            IRIS_LOG("[TUNE] Stored peer ramp report (still measuring their frames)");
        }
    } else if (payload.find("TUNE:GAIN=") != std::string::npos) {
        // Legacy/Mode A gain report. Also serves as OFDM fallback.
        size_t pos = payload.find("TUNE:GAIN=");
        float remote_gain = 0;
        try {
            remote_gain = std::stof(payload.substr(pos + 10));
        } catch (...) {
            IRIS_LOG("[TUNE] Invalid gain format in payload");
            return;
        }
        if (remote_gain <= 0.01f || remote_gain > 10.0f) {
            IRIS_LOG("[TUNE] Gain out of range: %.4f", remote_gain);
            return;
        }
        IRIS_LOG("[TUNE] Peer reports gain=%.4f from our test frames (state=%d)",
                 remote_gain, (int)tune_state_.load());
        tune_audit("PEER_REPORT peer_gain=%.4f my_gain=%.4f state=%d",
                   remote_gain, tune_my_gain_, (int)tune_state_.load());
        tune_peer_gain_ = remote_gain;

        // AFSK report fallback (non-OFDM mode only)
        if (tune_state_ == TuneState::WAIT_REPORT) {
            tune_state_ = TuneState::APPLY;
        } else if (tune_state_ == TuneState::WAIT_PEER) {
            IRIS_LOG("[TUNE] Stored peer gain (still measuring their frames)");
        }
    }
}

std::string Modem::tune_build_ramp_report() const {
    // Compact report: only include measured entries (iters != -1).
    // Format: "TUNE:R=idx:iters:H:SNR,idx:iters:H:SNR,..."
    // Example: "TUNE:R=0:-2:3.37:2.8,1:-2:3.38:3.1" (~50 bytes)
    // Backward-compatible: old parsers ignore the 4th field, new parsers accept 3 or 4 fields.
    char buf[192];
    int pos = snprintf(buf, sizeof(buf), "TUNE:R=");
    bool first = true;
    for (int i = 0; i < TUNE_RAMP_COUNT && pos < (int)sizeof(buf) - 24; i++) {
        if (tune_rx_frame_iters_[i] == -1) continue;  // not measured
        if (!first) buf[pos++] = ',';
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%d:%d:%.2f:%.1f",
                        i, tune_rx_frame_iters_[i], tune_rx_frame_H_[i],
                        tune_rx_frame_snr_[i]);
        first = false;
    }
    return std::string(buf);
}

// Static constexpr member definitions (C++14 ODR-use)
constexpr float Modem::ofdm_level_offset_db_[10];

void Modem::tune_compute_scales(float base) {
    // Compute TUNE_RAMP_COUNT distinct scales so that base*scale spans
    // [TUNE_SCALE_MIN .. TUNE_SCALE_MAX] uniformly in dB.
    if (base < 0.001f) base = 0.1f;
    float abs_min = TUNE_SCALE_MIN;
    float abs_max = TUNE_SCALE_MAX;
    // Scale range: base*scale_lo = abs_min, base*scale_hi = abs_max
    float scale_lo = abs_min / base;  // may be < 1
    float scale_hi = abs_max / base;  // may be > 1
    // Clamp to physically meaningful range
    if (scale_lo < 0.01f) scale_lo = 0.01f;
    if (scale_hi > 20.0f) scale_hi = 20.0f;
    if (scale_lo >= scale_hi) {
        // Degenerate: base is at or beyond limits, spread around 1.0
        scale_lo = 0.5f;
        scale_hi = 2.0f;
    }
    float db_lo = 20.0f * std::log10(scale_lo);
    float db_hi = 20.0f * std::log10(scale_hi);
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        float db = db_hi - (db_hi - db_lo) * i / (float)(TUNE_RAMP_COUNT - 1);
        tune_computed_scales_[i] = std::pow(10.0f, db / 20.0f);
    }
    IRIS_LOG("[TUNE] Computed %d ramp scales for base=%.3f: [%.3f .. %.3f] (%.1f dB range)",
             TUNE_RAMP_COUNT, base, tune_computed_scales_[TUNE_RAMP_COUNT - 1],
             tune_computed_scales_[0], db_hi - db_lo);
}

float Modem::tune_fit_tx_level() const {
    // SNR-based parabolic fit for TX level optimization.
    // Fit (tx_dB, -SNR) parabola; minimum of -SNR = maximum SNR = optimal TX.
    //
    // FM-specific considerations:
    //   - FM deviation limiter makes |H| invariant across TX levels → H-based fit useless
    //   - Preamble-only SNR (from SC metric on failed frames) is unreliable: it measures
    //     ZC preamble correlation, not data quality. Can be 5-8 dB off from true SNR.
    //   - When ALL frames fail LDPC, we have ZERO reliable SNR measurements.
    //   - Navalekar (2019) optimal FM OFDM backoff: 0.65-0.88 of deviation limiter threshold.

    // Count LDPC-decoded vs preamble-only measurements
    int n_ldpc = 0, n_preamble = 0;
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        if (tune_peer_iters_[i] >= 0) n_ldpc++;
        else if (tune_peer_iters_[i] == -2) n_preamble++;
    }

    // CRITICAL FIX: If zero frames decoded, preamble-only SNR is unreliable.
    // Don't run a parabolic fit on garbage data — it produces absurd results
    // (e.g., tx_level 0.883→0.141, making the station inaudible).
    // Fall back to a conservative FM-appropriate level.
    if (n_ldpc == 0) {
        // No LDPC convergence at any TX level. Two scenarios:
        // 1. FM channel: use 0.75× max ramp level (Navalekar optimal backoff region)
        // 2. Linear channel: keep current level (problem is likely not TX power)
        //
        // Detect FM: |H| nearly invariant across measurements (max/min < 2.0)
        float h_min = 1e9f, h_max = -1e9f;
        int h_count = 0;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
            if (tune_peer_iters_[i] != -1 && tune_peer_H_[i] > 0.01f) {
                h_min = std::min(h_min, tune_peer_H_[i]);
                h_max = std::max(h_max, tune_peer_H_[i]);
                h_count++;
            }
        }
        bool fm_channel = (h_count >= 3 && h_max / h_min < 2.0f);

        if (fm_channel) {
            // FM: use 75% of maximum ramp level (in Navalekar 0.65-0.88 optimal zone).
            // This is conservative enough to avoid clipping but loud enough to be heard.
            float tx_max = 0.0f;
            for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                tx_max = std::max(tx_max, tune_ramp_tx_levels_[i]);
            float tx_fm = 0.75f * tx_max;
            IRIS_LOG("[TUNE] SNR fit: ALL %d frames failed LDPC, FM channel detected "
                     "(H range %.2f-%.2f, ratio %.1f). Using 0.75×max = %.4f",
                     n_preamble, h_min, h_max, h_max / h_min, tx_fm);
            return std::clamp(tx_fm, 0.05f, 1.0f);
        } else {
            // Linear or unknown: keep current level, don't make things worse
            IRIS_LOG("[TUNE] SNR fit: ALL %d frames failed LDPC, non-FM channel. "
                     "Keeping current tx_level=%.4f", n_preamble, config_.tx_level);
            return config_.tx_level;
        }
    }

    // Detect FM channel: |H| nearly invariant across DECODED frames.
    // Only consider frames that decoded (iters >= 0) — preamble-only frames
    // at extreme TX levels can have very different H (below limiter threshold
    // at low TX, clipped at high TX) even on FM channels.
    float h_min = 1e9f, h_max = -1e9f;
    int h_count = 0;
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        if (tune_peer_iters_[i] >= 0 && tune_peer_H_[i] > 0.01f) {
            h_min = std::min(h_min, tune_peer_H_[i]);
            h_max = std::max(h_max, tune_peer_H_[i]);
            h_count++;
        }
    }
    bool fm_channel = (h_count >= 3 && h_max / h_min < 3.0f);

    // FM channel strategy: find the clipping edge, back off one step.
    // The FM deviation limiter creates a plateau — any level below the
    // clipping point works roughly equally well. We don't need the optimal
    // level, we need a SAFE level. Find the highest TX level where a frame
    // decoded (clipping edge), then use the next lower ramp level.
    if (fm_channel) {
        // Ramp frames are ordered highest TX first (index 0 = loudest).
        // Find the first (highest-TX) index that decoded.
        int clip_edge_idx = -1;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
            if (tune_peer_iters_[i] >= 0) {
                clip_edge_idx = i;
                break;
            }
        }

        // Back off one step from the clipping edge
        int use_idx = (clip_edge_idx >= 0) ? std::min(clip_edge_idx + 1, TUNE_RAMP_COUNT - 1) : 0;
        // If the backed-off index didn't decode, use the edge itself
        if (tune_peer_iters_[use_idx] < 0 && clip_edge_idx >= 0)
            use_idx = clip_edge_idx;

        float tx_opt = tune_ramp_tx_levels_[use_idx];

        IRIS_LOG("[TUNE] FM channel (H ratio %.1f): clip edge at ramp[%d] tx=%.3f, "
                 "backed off to ramp[%d] tx=%.3f",
                 h_max / h_min, clip_edge_idx,
                 clip_edge_idx >= 0 ? tune_ramp_tx_levels_[clip_edge_idx] : 0.0f,
                 use_idx, tx_opt);
        return std::clamp(tx_opt, 0.05f, 1.0f);
    }

    // Linear channel: use SNR-based parabolic fit
    float sx[TUNE_RAMP_COUNT], sy[TUNE_RAMP_COUNT];
    int n_snr = 0;
    bool using_preamble_pts = (n_ldpc < 3);

    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        float tx = tune_ramp_tx_levels_[i];
        if (tx < 0.001f) continue;
        if (tune_peer_snr_[i] > -90.0f) {
            if (tune_peer_iters_[i] >= 0 || using_preamble_pts) {
                sx[n_snr] = 20.0f * std::log10(tx);
                sy[n_snr] = -tune_peer_snr_[i];
                n_snr++;
            }
        }
    }

    IRIS_LOG("[TUNE] Linear channel: %d LDPC + %d preamble-only, using %d points",
             n_ldpc, n_preamble, n_snr);

    if (n_snr == 0) {
        IRIS_LOG("[TUNE] SNR fit: no valid data points");
        return config_.tx_level;
    }

    if (n_snr < 3) {
        int best = 0;
        for (int i = 1; i < n_snr; i++) if (sy[i] < sy[best]) best = i;
        float tx_opt = std::pow(10.0f, sx[best] / 20.0f);
        IRIS_LOG("[TUNE] SNR fit: %d points, using best SNR frame at %.1f dB SNR (tx=%.4f)",
                 n_snr, -sy[best], tx_opt);
        return std::clamp(tx_opt, 0.05f, 1.0f);
    }

    // Parabolic fit on (tx_dB, -SNR)
    float snr_min = 1e9f, snr_max = -1e9f;
    for (int i = 0; i < n_snr; i++) {
        snr_min = std::min(snr_min, -sy[i]);
        snr_max = std::max(snr_max, -sy[i]);
    }

    float S0 = (float)n_snr, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    float Sy0 = 0, Sy1 = 0, Sy2 = 0;
    for (int i = 0; i < n_snr; i++) {
        float xi = sx[i], yi = sy[i];
        S1 += xi; S2 += xi*xi; S3 += xi*xi*xi; S4 += xi*xi*xi*xi;
        Sy0 += yi; Sy1 += xi*yi; Sy2 += xi*xi*yi;
    }
    float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
    if (std::abs(D) > 1e-12f && snr_max - snr_min >= 3.0f) {
        float Da = S0*(S2*Sy2 - S3*Sy1) - S1*(S1*Sy2 - S3*Sy0) + S2*(S1*Sy1 - S2*Sy0);
        float Db = S0*(Sy1*S4 - S3*Sy2) - S1*(Sy0*S4 - Sy2*S2) + S2*(Sy0*S3 - Sy1*S2);
        float a = Da / D;
        float b = Db / D;
        if (a > 0.0f) {
            float x_opt = -b / (2.0f * a);
            float tx_opt = std::pow(10.0f, x_opt / 20.0f);
            IRIS_LOG("[TUNE] SNR fit: %d points, a=%.4f b=%.4f x_opt=%.1f dB (tx=%.4f)",
                     n_snr, a, b, x_opt, tx_opt);
            return std::clamp(tx_opt, 0.05f, 1.0f);
        }
    }

    // Fallback: best SNR frame
    int best = 0;
    for (int i = 1; i < n_snr; i++) if (sy[i] < sy[best]) best = i;
    float tx_opt = std::pow(10.0f, sx[best] / 20.0f);
    IRIS_LOG("[TUNE] SNR fit: parabola invalid (spread=%.1f dB), using best SNR=%.1f dB (tx=%.4f)",
             snr_max - snr_min, -sy[best], tx_opt);
    return std::clamp(tx_opt, 0.05f, 1.0f);
}

float Modem::ofdm_effective_tx_level() const {
    if (ofdm_tx_base_ <= 0.0f) return config_.tx_level;
    int level = std::clamp(ofdm_speed_level_, 0, NUM_OFDM_SPEED_LEVELS - 1);
    // ofdm_level_offset_db_ covers O0..O9 only (10 entries). Clamp the index so
    // O10..O13 (dense 256/1024QAM, reachable now that the ladder is unblocked)
    // reuse O9's -3 dB offset instead of reading past the array end.
    int off_idx = std::min(level, 9);
    float offset_db = ofdm_level_offset_db_[off_idx];
    float scale = std::pow(10.0f, offset_db / 20.0f);
    float eff = ofdm_tx_base_ * scale;
    // UPPER-LADDER ROOT (TX deviation auto-cal): the OFDM data portion is
    // normalized to OFDM_TARGET_RMS (0.50, ~modem.cc:4008) and then the whole
    // buffer is scaled by this eff (~modem.cc:4177), so the on-air data RMS =
    // 0.50 * eff. The FM/relay deviation limiter clips above ~0.30 RMS and EVM
    // collapses off that cliff (measured: 0.40->23.5 dB, 0.55->15.4 dB interior;
    // managed 0.10-0.25 -> 36-38 dB). Dense constellations (O6+/32QAM+) are
    // EVM-limited, so when TUNE calibrated ofdm_tx_base_ hot (up to 1.0 -> eff
    // 0.79 at O6's -2 dB -> RMS 0.40, deep on the clip cliff -> live 32QAM
    // decoded only 23-47%), cap eff to hold the O6+ data RMS <= ~0.25 (eff <=
    // 0.50), off the cliff, so O6/O7 decode live as well as the managed
    // forced-level gate32 (100% WGN40). O0..O5 keep the full [0.05,1.0] range
    // for low-SNR reach. Env IRIS_OFDM_TXCAP=0 restores the old clamp (A/B).
    static const bool ofdm_txcap = []() {
        const char* e = std::getenv("IRIS_OFDM_TXCAP");
        return !e || std::atoi(e) != 0; }();
    float eff_hi = (ofdm_txcap && level >= 6) ? 0.50f : 1.0f;
    return std::clamp(eff, 0.05f, eff_hi);
}

void Modem::tune_apply_corrections() {
    // Hybrid TX/RX correction — no double-dip because adjustments are deterministic
    // and each side can predict what the peer will do.
    //
    // Both sides exchange gain reports. Each side knows:
    //   my_gain   = what WE measured from peer's test frames
    //   peer_gain = what the PEER measured from OUR test frames
    //
    // TX rule: if peer_gain > 1, we're overdriving the peer → reduce TX.
    //   Reducing always works (no radio clipping concern). Don't boost TX
    //   (radio deviation limiter makes it unreliable, confirmed OTA).
    //
    // RX rule: if my_gain < 1, peer is too quiet for us → boost RX.
    //   If my_gain > 1, the peer is too loud, but we KNOW they'll reduce
    //   their TX (their peer_gain = our my_gain > 1), so skip RX correction.
    //   This avoids double-dip: TX reduction + RX attenuation don't stack.

    float old_level = config_.tx_level;

    if (native_mode_ || ofdm_kiss_) {
        // OFDM power-ramp TUNE: peer reported (iters, H) for each of our 5 ramp
        // frames at different TX levels. Use parabolic fit to find optimal tx_level.
        //
        // If peer sent RAMP report: fit parabola to find optimal drive level.
        // If peer sent legacy GAIN report: fall back to mean|H| targeting.
        // Check for any ramp data: iters >= 0 (LDPC decoded) or -2 (preamble-only H)
        bool have_ramp = false;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++)
            if (tune_peer_iters_[i] >= -2 && tune_peer_iters_[i] != -1) { have_ramp = true; break; }
        if (have_ramp) {
            float fitted_tx = tune_fit_tx_level();
            config_.tx_level = std::clamp(fitted_tx, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            ofdm_tx_base_ = config_.tx_level;
            {
                int ldpc_pts = 0, h_pts = 0;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_peer_iters_[i] >= 0) ldpc_pts++;
                    else if (tune_peer_iters_[i] == -2) h_pts++;
                }
                IRIS_LOG("[TUNE] OFDM TX: fit → %.3f (was %.3f), %d LDPC + %d preamble-only points",
                         config_.tx_level, old_level, ldpc_pts, h_pts);
            }
        } else if (tune_peer_gain_ > 0.01f) {
            // Fallback: legacy GAIN report from peer (backward compat or no frames decoded)
            constexpr float OFDM_TARGET_H = 1.0f;
            float ratio = OFDM_TARGET_H / tune_peer_gain_;
            config_.tx_level *= ratio;
            config_.tx_level = std::clamp(config_.tx_level, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            ofdm_tx_base_ = config_.tx_level;
            IRIS_LOG("[TUNE] OFDM TX fallback: peer saw H=%.3f, level %.3f → %.3f",
                     tune_peer_gain_, old_level, config_.tx_level);
        } else {
            // No peer report — we don't know how the peer received our frames.
            // But we DO know how WE received the peer's frames (tune_rx_frame_*).
            // On FM, both radios have similar deviation limiters, so the optimal
            // TX level is roughly symmetric.
            // Both sides use identical scale arrays (tune_computed_scales_[]).
            int local_decoded = 0;
            float local_h_min = 1e9f, local_h_max = -1e9f;
            int local_h_count = 0;
            for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                if (tune_rx_frame_iters_[i] >= 0) {
                    local_decoded++;
                    if (tune_rx_frame_H_[i] > 0.01f) {
                        local_h_min = std::min(local_h_min, tune_rx_frame_H_[i]);
                        local_h_max = std::max(local_h_max, tune_rx_frame_H_[i]);
                        local_h_count++;
                    }
                }
            }
            bool local_fm = (local_h_count >= 3 && local_h_max / local_h_min < 3.0f);

            if (local_decoded >= 2 && local_fm) {
                // FM mirror: clip-edge-backoff on local RX (same as peer path).
                // Find highest-TX index (lowest i) that decoded locally.
                int clip_edge_idx = -1;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_rx_frame_iters_[i] >= 0) {
                        clip_edge_idx = i;
                        break;
                    }
                }
                int use_idx = (clip_edge_idx >= 0) ? std::min(clip_edge_idx + 1, TUNE_RAMP_COUNT - 1) : 0;
                if (tune_rx_frame_iters_[use_idx] < 0 && clip_edge_idx >= 0)
                    use_idx = clip_edge_idx;
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                float mirror_tx = std::clamp(ramp_base * tune_computed_scales_[use_idx], 0.05f, 1.0f);
                config_.tx_level = mirror_tx;
                config_.calibrated_tx_level = config_.tx_level;
                ofdm_tx_base_ = config_.tx_level;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, FM mirror clip-edge-backoff: "
                         "clip_edge ramp[%d], using ramp[%d] (scale=%.2f) → tx=%.3f (was %.3f)",
                         clip_edge_idx, use_idx, tune_computed_scales_[use_idx],
                         config_.tx_level, old_level);
            } else if (local_decoded >= 2) {
                // Linear mirror: best SNR index
                int best_rx_idx = -1;
                float best_rx_snr = -1e9f;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_rx_frame_iters_[i] >= 0 && tune_rx_frame_snr_[i] > best_rx_snr) {
                        best_rx_snr = tune_rx_frame_snr_[i];
                        best_rx_idx = i;
                    }
                }
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                float best_scale = tune_computed_scales_[best_rx_idx];
                float mirror_tx = std::clamp(ramp_base * best_scale, 0.05f, 1.0f);
                config_.tx_level = mirror_tx;
                config_.calibrated_tx_level = config_.tx_level;
                ofdm_tx_base_ = config_.tx_level;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, linear mirror: "
                         "best SNR=%.1f dB at ramp[%d] (scale=%.2f) → tx=%.3f (was %.3f)",
                         best_rx_snr, best_rx_idx, best_scale, config_.tx_level, old_level);
            } else if (ofdm_tx_base_ > 0.01f) {
                config_.tx_level = ofdm_tx_base_;
                config_.calibrated_tx_level = ofdm_tx_base_;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, using probe base %.3f (was %.3f)",
                         ofdm_tx_base_, old_level);
            } else {
                IRIS_LOG("[TUNE] OFDM TX: no peer report, keeping level %.3f", config_.tx_level);
                ofdm_tx_base_ = config_.tx_level;
            }
        }

        // RX: MMSE equalizer handles arbitrary gain. No RX correction needed.
        native_rx_gain_ = 1.0f;
        IRIS_LOG("[TUNE] OFDM: base=%.3f (O0), offsets: O3=-1dB O5=-2dB O7=-3dB",
                 ofdm_tx_base_);
    } else {
        // Mode A (single-carrier): apply TX and RX corrections as before.
        // TX: adjust toward peer_gain=1.0.
        //   peer_gain > 1 → we're too loud → reduce TX (always safe).
        //   peer_gain < 1 → we're too quiet → boost TX (capped at 1.0).
        if (tune_peer_gain_ > 0.01f) {
            config_.tx_level /= tune_peer_gain_;
            config_.tx_level = std::clamp(config_.tx_level, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            if (std::abs(config_.tx_level - old_level) > 0.001f) {
                IRIS_LOG("[TUNE] TX: peer_gain=%.3f, level %.3f → %.3f",
                         tune_peer_gain_, old_level, config_.tx_level);
            } else {
                IRIS_LOG("[TUNE] TX: peer_gain=%.3f, level %.3f (no change needed)",
                         tune_peer_gain_, config_.tx_level);
            }
        }

        // RX: normalize so channel_gain ≈ 1.0 (Mode A slicer needs this).
        if (tune_my_gain_ > 0.01f) {
            native_rx_gain_ = 1.0f / tune_my_gain_;
            native_rx_gain_ = std::clamp(native_rx_gain_, 0.1f, 10.0f);
            IRIS_LOG("[TUNE] RX: my_gain=%.3f, rx_gain=%.3f",
                     tune_my_gain_, native_rx_gain_);
        }
    }

    if (gui_log_) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "[TUNE] Done! TX=%.3f RxG=%.2f (we saw %.3f, peer saw %.3f)",
                 config_.tx_level, native_rx_gain_, tune_my_gain_, tune_peer_gain_);
        gui_log_(msg);
    }

    tune_audit("APPLY peer=%s role=%s my_gain=%.4f peer_gain=%.4f "
               "tx_level=%.3f->%.3f rx_gain=%.3f frames_measured=%d",
               tune_peer_call_.c_str(),
               tune_is_initiator_ ? "initiator" : "responder",
               tune_my_gain_, tune_peer_gain_,
               old_level, config_.tx_level, native_rx_gain_,
               tune_frames_measured_);

    tune_state_ = TuneState::DONE;
    tune_timeout_ = 100;  // 5s display before resetting to IDLE

    // Responder defers TX after TUNE so initiator (commander) goes first.
    // Without this, both sides TX simultaneously → half-duplex collision → T1 timeout.
    if (!tune_is_initiator_) {
        tune_post_holdoff_ = 60;  // 3s — enough for initiator to TX one frame
        IRIS_LOG("[TUNE] Responder post-TUNE holdoff: deferring TX for 3s");
    } else {
        tune_post_holdoff_ = 0;
    }
}

void Modem::tune_audit(const char* fmt, ...) {
    // Format and forward to main log via IRIS_LOG
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    IRIS_LOG("[TUNE-AUDIT] %s", buf);
}

void Modem::kalman_log_trace(const KalmanTrace& trace, bool decode_ok,
                              float snr, float gain) {
    if (trace.fwd.empty()) return;

    // Lazy-open kalman_trace_YYYYMMDD_HHMMSS.csv in AppData/Iris/logs/
    if (!kalman_log_file_) {
        time_t now_t = time(NULL);
        struct tm* t = localtime(&now_t);
        char dir[512] = ".";
        char path[700];
#ifdef _WIN32
        char appdata[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, appdata)))
            snprintf(dir, sizeof(dir), "%s\\Iris\\logs", appdata);
#else
        const char* home = getenv("HOME");
        if (home)
            snprintf(dir, sizeof(dir), "%s/.config/iris/logs", home);
#endif
        snprintf(path, sizeof(path), "%s/kalman_trace_%04d%02d%02d_%02d%02d%02d.csv",
                 dir, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
        kalman_log_file_ = fopen(path, "w");
        if (!kalman_log_file_) return;
        fprintf(kalman_log_file_,
                "timestamp,frame_seq,decode,snr_db,gain,total_sym,ds_factor,"
                "sym_idx,fwd_phase,fwd_freq,fwd_accel,smo_phase,smo_freq,smo_accel,pilot\n");
    }

    // Wall-clock timestamp for this frame
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    struct tm* tm = localtime(&t);
    char ts[32];
    snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec, (int)ms);

    static int frame_seq = 0;
    frame_seq++;

    size_t n = std::min(trace.fwd.size(), trace.smoothed.size());
    for (size_t i = 0; i < n; i++) {
        int sym = (int)i * trace.downsample_factor;
        fprintf(kalman_log_file_,
                "%s,%d,%d,%.1f,%.4f,%d,%d,"
                "%d,%.6f,%.6f,%.9f,%.6f,%.6f,%.9f,%d\n",
                ts, frame_seq, decode_ok ? 1 : 0, snr, gain,
                trace.total_symbols, trace.downsample_factor,
                sym,
                trace.fwd[i].phase, trace.fwd[i].freq, trace.fwd[i].accel,
                trace.smoothed[i].phase, trace.smoothed[i].freq, trace.smoothed[i].accel,
                trace.smoothed[i].is_pilot ? 1 : 0);
    }
    fflush(kalman_log_file_);
}

void Modem::start_autotune(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (tune_state_ != TuneState::IDLE && tune_state_ != TuneState::DONE) {
        IRIS_LOG("[TUNE] Already in progress, ignoring");
        return;
    }
    // Defer AX.25 T1 immediately (zero-gap): the probe-completion path may have
    // just armed T1 against the held I-frames (modem.cc probe activation), and
    // auto-tune is about to monopolize the TX for tens of seconds.  Suspend T1
    // now so it cannot fire a spurious TIMER_RECOVERY before the first post-tune
    // OFDM data burst (P0 turnaround drop).  The per-tick tune_state_ sync keeps
    // it deferred and clears it when tune ends.
    ax25_session_.set_tx_deferred(true);
    IRIS_LOG("[TUNE] Starting auto-tune with %s", remote_callsign.c_str());
    if (gui_log_) gui_log_("[TUNE] Starting auto-tune with " + remote_callsign + "...");

    tune_audit("=== TUNE START === local=%s peer=%s role=initiator tx_level=%.3f rx_gain=%.3f",
               config_.callsign.c_str(), remote_callsign.c_str(),
               config_.tx_level, native_rx_gain_);

    tune_peer_call_ = remote_callsign;
    tune_is_initiator_ = true;
    tune_my_gain_ = 0;
    tune_peer_gain_ = 0;
    tune_test_frames_sent_ = 0;
    tune_frames_measured_ = 0;
    tune_wait_peer_ticks_ = 0;
    tune_last_measured_count_ = 0;
    tune_silence_ticks_ = 0;
    tune_timeout_ = 800;  // 40s: 5 frames ~6s TX each side + report exchange + margin
    tune_test_frames_target_ = TUNE_RAMP_COUNT;
    // Compute adaptive ramp scales based on current TX base
    {
        float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
        tune_compute_scales(ramp_base);
    }
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        tune_rx_frame_iters_[i] = -1;
        tune_rx_frame_H_[i] = 0;
        tune_rx_frame_snr_[i] = -99.0f;
        tune_peer_iters_[i] = -1;
        tune_peer_H_[i] = 0;
        tune_peer_snr_[i] = -99.0f;
        tune_ramp_tx_levels_[i] = 0;
    }

    // Send TUNE:START and wait for TUNE:READY from responder before sending ramp.
    // This prevents the ramp from going out while the responder is still in probe.
    tune_state_ = TuneState::WAIT_READY;
    tune_ready_resend_cd_ = 0;  // send immediately on first tick
    tune_ready_resends_ = 0;
    tune_timeout_ = 1200;  // 60s: extra time for READY handshake + ramp + reports
    char start_msg[32];
    snprintf(start_msg, sizeof(start_msg), "TUNE:START=%d", TUNE_RAMP_COUNT);
    send_tune_ui(start_msg);
    send_tune_ui(start_msg);
    IRIS_LOG("[TUNE] Sent TUNE:START, waiting for READY from responder");
}

// --- Calibration ---
// Protocol:
//   Initiator: clicks Auto Cal → sends CAL:START UI frame → tone 1s → WAIT_REPORT
//   Responder: auto-detects CAL:START → measures tone RMS → sends CAL:RMS=X.XXXX
//   Initiator: receives report → adjusts TX level → DONE

// Build and queue a UI frame with cal payload (e.g., "CAL:START" or "CAL:RMS=0.1234")
void Modem::send_cal_ui(const char* payload) {
    // Caller holds modem_mutex_
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("CAL");
    // UI frame: dst(7) + src(7) + ctrl(1) + PID(1) + info
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    size_t plen = strlen(payload);
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + plen);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

// Handle incoming CAL: frame (called from dispatch_rx_frame)
void Modem::handle_cal_frame(const uint8_t* info, size_t len) {
    std::string payload((const char*)info, len);

    if (payload.find("CAL:START") != std::string::npos) {
        // Remote is starting cal — enter RX measurement mode
        if (cal_state_ == CalState::IDLE) {
            IRIS_LOG("[CAL] Received CAL:START — entering measurement mode");
            if (gui_log_) gui_log_("[CAL] Measuring remote tone...");
            state_ = ModemState::CALIBRATING;
            cal_state_ = CalState::RX_TONE;
            cal_rms_accum_ = 0;
            cal_rms_count_ = 0;
            cal_measured_rms_ = 0;
        }
    } else if (payload.find("CAL:RMS=") != std::string::npos) {
        // Remote is reporting our tone's RMS at their end
        size_t pos = payload.find("CAL:RMS=");
        float remote_rms = std::stof(payload.substr(pos + 8));
        IRIS_LOG("[CAL] Remote measured RMS=%.4f", remote_rms);
        if (remote_rms > 0.001f && cal_state_ == CalState::WAIT_REPORT) {
            float correction = CAL_TARGET_RMS / remote_rms;
            config_.tx_level *= correction;
            config_.tx_level = std::clamp(config_.tx_level, 0.01f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            cal_measured_rms_ = remote_rms;
            IRIS_LOG("[CAL] Adjusted TX level to %.3f (correction %.2fx)",
                     config_.tx_level, correction);
            if (gui_log_) {
                char msg[128];
                snprintf(msg, sizeof(msg), "[CAL] Done! TX level = %.3f (remote RMS was %.4f)",
                         config_.tx_level, remote_rms);
                gui_log_(msg);
            }
            cal_state_ = CalState::DONE;
            state_ = ModemState::IDLE;
        }
    }
}

void Modem::start_probe() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0) {
        IRIS_LOG("[PROBE] Already probing, ignoring request");
        return;
    }
    IRIS_LOG("[PROBE] Manual probe: sending PROBE:START, tones in 3s");
    if (gui_log_) gui_log_("Probe: signaling remote, tones in 3s...");
    ofdm_kiss_probe_done_ = false;
    probe_manual_ = true;
    // Send PROBE:START twice for reliability on noisy channel
    send_probe_start_ui();
    send_probe_start_ui();
    // Countdown: 60 ticks = 3s at 50ms/tick.  Gives remote time to decode
    // PROBE:START and start listening before we send tones.
    ofdm_kiss_probe_cd_ = 60;
}

void Modem::send_probe_start_ui() {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("PROBE");
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    const char* payload = "PROBE:START";
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + 11);
    // Caller holds modem_mutex_
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

void Modem::send_probe_ready_ui() {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("PROBE");
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    const char* payload = "PROBE:READY";
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + 11);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

void Modem::start_calibration() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("[CAL] Starting calibration");
    if (gui_log_) gui_log_("[CAL] Sending tone to remote...");
    state_ = ModemState::CALIBRATING;
    cal_state_ = CalState::SEND_CMD;
    cal_tone_samples_ = 0;
    cal_tone_phase_ = 0;
    cal_rms_accum_ = 0;
    cal_rms_count_ = 0;
    cal_measured_rms_ = 0;
    // Queue the CAL:START command frame
    send_cal_ui("CAL:START");
}

void Modem::generate_cal_tone(float* audio, int count) {
    float phase_inc = 2.0f * (float)M_PI * CAL_TONE_FREQ / (float)config_.sample_rate;
    for (int i = 0; i < count; i++) {
        audio[i] = config_.tx_level * std::sin(cal_tone_phase_);
        cal_tone_phase_ += phase_inc;
        if (cal_tone_phase_ > 2.0f * (float)M_PI)
            cal_tone_phase_ -= 2.0f * (float)M_PI;
        cal_tone_samples_++;
    }
    if (cal_tone_samples_ >= CAL_TONE_DURATION) {
        ptt_off();
        cal_state_ = CalState::WAIT_REPORT;
        IRIS_LOG("[CAL] Tone sent, waiting for remote report");
    }
}

void Modem::process_calibration_rx(const float* audio, int count) {
    if (cal_state_ == CalState::SEND_CMD) {
        // Wait for the CAL:START frame to be transmitted
        if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
            // Command sent — start transmitting tone
            cal_state_ = CalState::TX_TONE;
            ptt_on();
            IRIS_LOG("[CAL] CAL:START sent, transmitting tone");
        }
    } else if (cal_state_ == CalState::RX_TONE) {
        // Measure incoming audio RMS (skip first 100ms for PTT settle)
        int skip_samples = config_.sample_rate / 10;
        for (int i = 0; i < count; i++) {
            cal_rms_count_++;
            if (cal_rms_count_ > skip_samples) {
                cal_rms_accum_ += audio[i] * audio[i];
            }
        }
        if (cal_rms_count_ >= CAL_TONE_DURATION + skip_samples) {
            int measure_count = cal_rms_count_ - skip_samples;
            cal_measured_rms_ = std::sqrt(cal_rms_accum_ / measure_count);
            IRIS_LOG("[CAL] Measured RMS = %.4f, sending report", cal_measured_rms_);
            if (gui_log_) {
                char msg[128];
                snprintf(msg, sizeof(msg), "[CAL] Remote RMS = %.4f, sending report",
                         cal_measured_rms_);
                gui_log_(msg);
            }
            // Send report back
            char report[48];
            snprintf(report, sizeof(report), "CAL:RMS=%.4f", cal_measured_rms_);
            send_cal_ui(report);
            cal_state_ = CalState::SEND_REPORT;
        }
    } else if (cal_state_ == CalState::SEND_REPORT) {
        // Wait for report TX to drain, then return to idle
        if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
            IRIS_LOG("[CAL] Report sent, calibration complete (responder)");
            if (gui_log_) gui_log_("[CAL] Report sent.");
            cal_state_ = CalState::IDLE;
            state_ = ModemState::IDLE;
        }
    } else if (cal_state_ == CalState::WAIT_REPORT) {
        // Demodulate AFSK looking for CAL:RMS= frame from remote
        std::vector<uint8_t> rx_nrzi;
        if (config_.ax25_baud == 9600)
            rx_nrzi = gfsk_demod_.demodulate(audio, count);
        else
            rx_nrzi = afsk_demod_.demodulate(audio, count);

        auto rx_bits = nrzi_decoder_.decode(rx_nrzi);
        if (config_.ax25_baud == 9600)
            g3ruh_rx_scrambler_.descramble(rx_bits);
        for (uint8_t b : rx_bits) {
            if (hdlc_decoder_.push_bit(b)) {
                const auto& frame = hdlc_decoder_.frame();
                // UI frame: 7(dst) + 7(src) + 1(ctrl) + 1(PID) = 16 bytes header
                if (frame.size() > 16) {
                    handle_cal_frame(frame.data() + 16, frame.size() - 16);
                }
            }
        }
    }
}

void Modem::cache_probe_result(const std::string& callsign) {
    if (callsign.empty() || !probe_.has_results() || !probe_.negotiated().valid)
        return;

    ProbeCacheEntry entry;
    entry.negotiated = probe_.negotiated();
    entry.my_tx = probe_.my_tx_result();
    entry.their_tx = probe_.their_tx_result();
    entry.timestamp = std::chrono::steady_clock::now();
    probe_cache_[callsign] = entry;

    // Persist to disk for cross-session reuse
    save_probe_to_disk(callsign, entry);

    IRIS_LOG("[PROBE-CACHE] cached result for %s (band %.0f-%.0f Hz, BW=%.0f Hz)",
             callsign.c_str(), entry.negotiated.low_hz, entry.negotiated.high_hz,
             entry.negotiated.bandwidth_hz);
}

int Modem::ring_acked_and_clear(uint8_t prev_va, uint8_t nr) {
    // Bound the credited range to the outstanding window [prev_va, vs): a peer
    // N(R) beyond V(S) (bogus/duplicate) must not over-credit the anchor or
    // over-clear the ring across the sequence wrap.  Uses the SESSION's live
    // modulus (mod-8 or wide modulo-128) so a wide N(R) is not aliased to 3 bits.
    int mod = ax25_session_.seq_mod();
    uint8_t vs = ax25_session_.vs();
    int span  = (int)(uint8_t)(((int)nr - (int)prev_va + mod) % mod);
    int wspan = (int)(uint8_t)(((int)vs - (int)prev_va + mod) % mod);
    if (span == 0 || span > wspan) return -1;  // nothing new acked / nr past window
    int lvl = tx_level_ring_.max_acked(prev_va, nr, mod);
    tx_level_ring_.clear_range(prev_va, nr, mod);
    return lvl;
}

bool Modem::tx_queue_has_identical(const uint8_t* data, size_t len) const {
    // Caller holds modem_mutex_.  tx_queue_ is capped at 32 short frames, so a
    // linear byte-compare scan is cheap.  std::queue exposes no iterator, so scan
    // a shallow copy (the queue is tiny; copies are a handful of small vectors).
    std::queue<TxFrame> scan = tx_queue_;
    while (!scan.empty()) {
        const TxFrame& f = scan.front();
        if (f.data.size() == len && std::memcmp(f.data.data(), data, len) == 0)
            return true;
        scan.pop();
    }
    return false;
}

void Modem::enqueue_native_tx_frame(std::vector<uint8_t>&& frame, bool tone_ack_eligible) {
    // Caller holds modem_mutex_.  THE single producer used by the native AX.25
    // session send_frame_ callback (modem.cc ~:718).  It COALESCES a byte-identical
    // frame already queued (P0 turnaround / C1-storm class-fix): a connect-recovery
    // TIMER_RECOVERY fires a go-back-N that re-emits the SAME outstanding I-frames
    // repeatedly (10x in the P0 re-smoke); queuing every copy overflows the 32-cap
    // tx_queue_ and the drop-oldest guard then SILENTLY evicts a never-sent in-flight
    // frame — an HONEST data loss with no retransmit path (the ics213 drop).  A copy
    // already waiting in the queue fully covers the retransmit, so the duplicate is
    // dropped, not queued.  This bounds the queue to the DISTINCT outstanding frames
    // (<= K=7 I-frames + a few control frames), so drop-oldest can never fire from a
    // recovery burst.  A retransmit of a frame that was already popped+sent is NOT in
    // the queue -> not coalesced -> correctly re-queued.  Byte-identity keeps RRs with
    // different SNR/level tails distinct (both sent) while collapsing true duplicates.
    if (tx_queue_has_identical(frame.data(), frame.size())) {
        IRIS_LOG("[TX] coalesced duplicate frame (%zu B already queued) — recovery-burst dedup",
                 frame.size());
        return;
    }
    // The 32-cap assumed a mod-8/K=7 window (<= 7 I-frames + a few control frames).
    // The WIDE WINDOW streams up to window_k() (default 63, max 127) I-frames before
    // an ACK, all of which can sit in tx_queue_ at once until the burst-build drains
    // them.  A 32-cap drop-oldest would SILENTLY evict never-sent wide-window frames
    // (the exact silent-loss this guard warns about) -> the far side's V(R) freezes.
    // Size the native queue to the live sequence space (mod-8 -> 32 unchanged; wide
    // -> seq_mod + control headroom).  Coalescing above still bounds a recovery burst.
    const size_t TX_QUEUE_MAX = ax25_session_.wide_window()
                                    ? (size_t)(ax25_session_.seq_mod() + 32)   // wide: 160
                                    : (size_t)32;                              // mod-8 (unchanged)
    if (tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
        tx_queue_.pop();
    }
    tx_queue_.push(TxFrame(std::move(frame), tone_ack_eligible));
}

void Modem::requeue_rejected_frames(std::vector<TxFrame>& taken) {
    // Leg 2 co-dependency safety net (data-flow-tx-queue.md §6.1): a frame whose
    // OFDM build was rejected because it exceeds the CURRENT (transiently low)
    // level's capacity but still fits the CONFIRMED anchor is RECOVERABLE — the
    // level will climb back to at least the anchor, so re-queue it to the FRONT
    // (provenance preserved). A frame too big for even the anchor can never be
    // reliably carried (a mis-sized B2F/client frame that bypassed max_info) —
    // drop it LOUD so the queue keeps moving (no head-of-line deadlock); this is
    // a diagnostic drop, NOT the silent destruction the burst loop did before.
    int anchor_cap = ofdm_capacity_bytes_for_level(tx_acked_level_, ofdm_config_);
    std::queue<TxFrame> requeued;
    int kept = 0, dropped = 0;
    int max_kept_bytes = 0;
    for (auto& tf : taken) {
        if ((int)tf.data.size() > anchor_cap + 5) {
            IRIS_LOG("[TX-OFDM] DROP un-sendable frame: %zu B > O%d anchor cap %d",
                     tf.data.size(), tx_acked_level_, anchor_cap);
            dropped++;
        } else {
            max_kept_bytes = std::max(max_kept_bytes, (int)tf.data.size());
            requeued.push(std::move(tf));
            kept++;
        }
    }
    while (!tx_queue_.empty()) {
        requeued.push(std::move(tx_queue_.front()));
        tx_queue_.pop();
    }
    tx_queue_ = std::move(requeued);
    if (kept > 0 || dropped > 0)
        IRIS_LOG("[TX-OFDM] build reject: re-queued %d frame(s), dropped %d un-sendable",
                 kept, dropped);

    // Fix B (DATALINK_TAX_DIAGNOSIS.md): a KEPT frame is being re-queued because the
    // current (transiently low) level can't carry it. Count the consecutive spins;
    // once past OFDM_OVERSIZE_REJECT_LIMIT, force the modulated level up to the min
    // level that fits the largest stuck (wrapped) frame so it gets on air and is
    // ACKed — a capacity mismatch must NEVER ride the re-queue loop into a T1 N2
    // disconnect. The fit-floor is consumed/cleared on the next successful OFDM send.
    if (kept > 0) {
        ofdm_reject_streak_++;
        // Size against the WRAPPED frame (the kept data already includes the AX.25
        // header; add the lone-sub-frame wrapper the burst builder prepends).
        int stuck_wrapped = max_kept_bytes + OFDM_KISS_WRAPPER_HDR + OFDM_KISS_SUBFRAME_HDR;
        int floor = ofdm_oversize_fit_floor(ofdm_reject_streak_, stuck_wrapped, ofdm_config_);
        if (floor >= 0 && floor > ofdm_fit_floor_level_) {
            ofdm_fit_floor_level_ = floor;
            IRIS_LOG("[TX-OFDM] Fix B: reject streak %d >= %d — flooring modulated level to O%d "
                     "to clear a %d-B stuck frame (never spin to N2 disconnect)",
                     ofdm_reject_streak_, OFDM_OVERSIZE_REJECT_LIMIT, floor, stuck_wrapped);
        }
    } else {
        // Nothing recoverable is stuck (all dropped or none) — reset the spin count.
        ofdm_reject_streak_ = 0;
    }
}

void Modem::resolve_slot_bounds(int probe_frames, float probe_airtime_s,
                                float batch_airtime_s,
                                int& max_frames, float& airtime_bound_s) {
    if (probe_frames >= 1) {
        // Probe arm: frame-count-controlled slot.  The airtime bound defaults
        // to the hard cap so the arm is deterministic (N frames decides the
        // slot); an explicit probe airtime can only LOWER it.  Slot length is
        // safe at the cap: every burst frame re-syncs on its own preamble, so
        // timing drift never accumulates across the slot (modem.h note).
        max_frames = std::min(probe_frames, BURST_FRAMES_PROBE_MAX);
        airtime_bound_s = (probe_airtime_s > 0.0f)
            ? std::min(probe_airtime_s, SLOT_AIRTIME_HARD_CAP_S)
            : SLOT_AIRTIME_HARD_CAP_S;
    } else {
        // Default: exactly the pre-probe bounds (8 frames, adaptive AIMD batch
        // airtime) — the probe knobs are inert unless their env is set.
        max_frames = BURST_FRAMES_DEFAULT_MAX;
        airtime_bound_s = (probe_airtime_s > 0.0f)
            ? std::min(probe_airtime_s, SLOT_AIRTIME_HARD_CAP_S)
            : batch_airtime_s;
    }
}

int Modem::append_coalesced_slot(float& frame_airtime_s, size_t burst_max_payload,
                                 std::vector<size_t>* frame_bounds) {
    int burst_frames = 1;   // the main frame is already in tx_buffer_
    int max_burst = BURST_FRAMES_DEFAULT_MAX;
    float slot_airtime_bound_s = batch_airtime_s_;
    resolve_slot_bounds(probe_frames_per_burst_, probe_slot_airtime_s_,
                        batch_airtime_s_, max_burst, slot_airtime_bound_s);
    while (frame_airtime_s < slot_airtime_bound_s && burst_frames < max_burst) {
        float xtime = append_ofdm_burst_frame(burst_max_payload);
        if (xtime < 0.0f) break;   // queue empty or build rejected (re-queued)
        frame_airtime_s += xtime;
        burst_frames++;
        if (frame_bounds) frame_bounds->push_back(tx_buffer_.size());
    }
    return burst_frames;
}

float Modem::append_ofdm_burst_frame(size_t burst_max_payload) {
    // Pop the next batch from tx_queue_ (PROVENANCE preserved in `taken` for a
    // reject-requeue), wrap + compress, build ONE OFDM frame, and append its
    // audio to tx_buffer_. Returns the frame's airtime in seconds, or -1 if the
    // queue is empty or the build was rejected (rejected frames are re-queued,
    // never destroyed). Unifies the main-frame and burst-frame drain so both
    // carry the oversized-drop + reject-requeue (data-flow-tx-queue.md §6).
    if (tx_queue_.empty()) return -1.0f;
    const int abs_cap = ofdm_abs_max_capacity(ofdm_config_);

    std::vector<TxFrame> taken;                       // originals (reject-requeue)
    std::vector<std::vector<uint8_t>> xbatch;         // working copies (build)
    size_t xtotal = 0;
    while (!tx_queue_.empty()) {
        auto& front = tx_queue_.front();
        // Drop-drain only genuinely un-sendable frames (bigger than ANY level's
        // capacity). A frame that merely overflows the current level is kept and
        // reject-requeued below so a transient level fall never destroys it.
        if (front.data.size() > (size_t)abs_cap + 5) {
            IRIS_LOG("[TX-OFDM] DROP oversized frame: %zu B > %d abs OFDM cap",
                     front.data.size(), abs_cap);
            tx_queue_.pop();
            continue;
        }
        // Match the wrapper built below (magic+epoch shared, then per-frame len) so
        // xtotal == xframe.size(); the "3" first-frame accounting omitted the epoch
        // byte (DATALINK_TAX_DIAGNOSIS.md, Fix A).
        size_t overhead = xbatch.empty()
            ? (size_t)(OFDM_KISS_WRAPPER_HDR + OFDM_KISS_SUBFRAME_HDR)  // magic+epoch + len = 4
            : (size_t)OFDM_KISS_SUBFRAME_HDR;                           // len = 2
        if (!xbatch.empty() && xtotal + overhead + front.data.size() > burst_max_payload)
            break;
        // #1 ROOT FIX: same live-N(R) re-stamp on the second air chokepoint (the
        // unified additional-burst-frame drain of the SAME tx_queue_).
        if (ofdm_kiss_tx_)
            restamp_live_nr(front.data, ax25_session_.current_vr(),
                            ax25_session_.extended());
        xtotal += overhead + front.data.size();
        xbatch.push_back(front.data);                 // copy for build
        taken.push_back(std::move(front));            // original for requeue
        tx_queue_.pop();
    }
    if (xbatch.empty()) return -1.0f;

    if (ofdm_kiss_ && packet_log_) {
        for (auto& sub : xbatch)
            if (sub.size() >= 14)
                packet_log_(true, "OFDM-KISS", describe_ax25(sub.data(), sub.size()));
    }

    // #2: this is an ADDITIONAL OFDM frame of the SAME physical burst, so it reuses
    // the current tx_burst_epoch_ (no increment — the main frame already advanced it
    // for a data burst).  Native path always wraps so the epoch travels.
    std::vector<uint8_t> xframe;
    bool xuse_wrapper = ofdm_kiss_tx_ ||
                        xbatch.size() > 1 ||
                        (xbatch.size() == 1 && !xbatch[0].empty() &&
                         xbatch[0][0] == MULTI_PAYLOAD_MAGIC);
    if (!xuse_wrapper) {
        xframe = std::move(xbatch[0]);
    } else {
        xframe.reserve(xtotal + 1);
        xframe.push_back(MULTI_PAYLOAD_MAGIC);
        xframe.push_back(tx_burst_epoch_);
        for (auto& sub : xbatch) {
            uint16_t len = (uint16_t)sub.size();
            xframe.push_back(len & 0xFF);
            xframe.push_back((len >> 8) & 0xFF);
            xframe.insert(xframe.end(), sub.begin(), sub.end());
        }
    }

    if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) &&
        xframe.size() > 20 && xframe[0] != B2F_DATA_MAGIC) {
        auto encoded = ofdm_kiss_tx_compressor_.compress_record(
            xframe.data(), xframe.size());
        if (encoded.status != v2::TransformStatus::Produced) {
            fail_ofdm_transform("OFDM burst transmit compression", xbatch);
            return -1.0f;
        }
        const int clen = static_cast<int>(encoded.produced_bytes.size());
        if (clen + 1 < (int)xframe.size()) {
            xframe.clear();
            xframe.reserve(1 + clen);
            xframe.push_back(COMPRESSED_PAYLOAD_MAGIC);
            xframe.insert(xframe.end(), encoded.produced_bytes.begin(),
                          encoded.produced_bytes.end());
        }
    }

    IRIS_LOG("[TX-OFDM] burst frame: %zu bytes (%zu sub-frames)",
             xframe.size(), xbatch.size());

    auto xiq = ofdm_mod_->build_ofdm_frame(
        xframe.data(), xframe.size(), ofdm_tone_map_);
    if (xiq.empty()) {
        // Reject: re-queue the popped originals instead of destroying them.
        IRIS_LOG("[TX-OFDM] burst frame rejected — re-queuing, stopping burst");
        requeue_rejected_frames(taken);
        return -1.0f;
    }

    // Fix B: this burst frame is going on air — reset the reject spin count (the
    // fit-floor is retired by anchor-catch-up in resolve_ofdm_tx_level, not here).
    ofdm_reject_streak_ = 0;

    size_t bstart = tx_buffer_.size();
    tx_buffer_.resize(bstart + xiq.size());
    for (size_t i = 0; i < xiq.size(); i++)
        tx_buffer_[bstart + i] = xiq[i].real();

    // Same whole-frame normalization as the main frame (one scale; see
    // ofdm_normalize_tx_frame for why a split preamble/data scale is wrong).
    ofdm_normalize_tx_frame(tx_buffer_, bstart,
        ofdm_config_.nfft + ofdm_config_.cp_samples, false);

    return (float)xiq.size() / (float)config_.sample_rate;
}

void Modem::reset_level_state() {
    // A4: reset EVERY OFDM level-owning variable atomically. The four reset sites
    // (disconnect, OFDM-prepare, force-ofdm, cached-probe) were DISJOINT — a
    // (re)connect reaching a prepare site WITHOUT a preceding clean disconnect
    // (force re-entry, cached-probe replay, back-to-back probe) inherited a STALE
    // inflated tx_acked_level_/tx_last_level_/tx_proposed_level_ from the prior
    // session, opening the new session's ceiling too high → over-leap → REJ-thrash.
    ofdm_speed_level_            = 0;     // V1
    ofdm_kiss_rx_level_         = 0;     // V2
    ofdm_kiss_rx_proposed_level_ = -1;    // V3
    tx_acked_level_             = 0;     // V4
    tx_last_level_              = 0;     // V5
    tx_proposed_level_          = -1;    // V6
    tx_no_ack_count_            = 0;
    last_peer_nr_               = 0xFF;
    ofdm_kiss_rx_confirmed_     = false;
    ofdm_expect_ack_            = false;
    ofdm_anchor_futility_       = 0;
    ofdm_refrag_pending_        = false;
    refrag_quiesce_start_tick_  = -1;     // INV-SEQ-2: drop any pending quiesce hold
    ax25_session_.set_iframe_hold(false);
    tx_level_ring_.reset();               // A3 ring
    gearshift_.force_ofdm_level(0);       // V7 (A5a also clears boost/cooldown/easy)
    gearshift_.set_ofdm_hold_frames(0);   // restore default hold; clean-climb re-lowers it after a clean probe
}

int Modem::resolve_ofdm_tx_level(bool kiss_control_batch, bool batch_is_control,
                                 size_t total_bytes) {
    // A2: single-source the OFDM TX speed level, ncw, and tone map from ONE level.
    // Re-latch V1 from the gearshift (V7) so an interleaved RX-adopt that forced
    // the gearshift since the last TX is reflected (invariant I7).
    ofdm_speed_level_ = gearshift_.current_ofdm_level();
    tx_no_ack_count_++;

    // Control frames at robust rate (~20ms extra) prevents cascading retransmit
    // timeouts when the channel has degraded.
    if (batch_is_control && ofdm_speed_level_ > 0) {
        IRIS_LOG("[TX-OFDM] control frame batch -> forcing O0 (robust)");
        ofdm_speed_level_ = 0;
    }
    // KISS S-frames (RR/REJ/RNR): force O1 minimum (QPSK r1/2). At O0 (BPSK) the
    // 1600-bit LDPC codeword needs 39 data symbols (1088ms); at O1 only 20 (544ms)
    // for the same preamble/detection probability. Halves ACK airtime.
    if (kiss_control_batch)
        ofdm_speed_level_ = std::max(1, ofdm_speed_level_);

    // TX-side override: no peer ACK → force downshift (KISS: to O0 immediately;
    // ARQ: step down by 1). The gearshift force makes it stick for one hold window.
    // Order (control/kiss force BEFORE this) is preserved from the pre-A2 inline
    // pipeline so behaviour is byte-identical here.
    {
        int no_ack_limit = ofdm_kiss_tx_ ? 2 : 6;
        if (tx_no_ack_count_ >= no_ack_limit && ofdm_speed_level_ > 0) {
            int old = ofdm_speed_level_;
            // climbgate Fix A (demote-refine): the old KISS behaviour slammed to O0
            // on any no-ACK, so a leap onto a not-yet-decodable rung (e.g. O6) that
            // failed collapsed the whole session to O0 and re-climbed — the
            // demote-cascade the owner flagged as "stuck at O3". Instead refine
            // DOWN ONE step, floored at the CONFIRMED-decodable anchor
            // (tx_acked_level_): settle at the highest rung the peer has actually
            // ACKed and hold there. ARQ path unchanged (already step-down-1).
            if (demote_refine_ && ofdm_kiss_tx_)
                ofdm_speed_level_ = std::max(tx_acked_level_, ofdm_speed_level_ - 1);
            else
                ofdm_speed_level_ = ofdm_kiss_tx_ ? 0 : std::max(0, ofdm_speed_level_ - 1);
            gearshift_.force_ofdm_level(ofdm_speed_level_);
            IRIS_LOG("OFDM Gearshift: no peer ACK for %d TX frames -> O%d (was O%d, anchor O%d)",
                     tx_no_ack_count_, ofdm_speed_level_, old, tx_acked_level_);
            // Anchor futility (data-flow-tx-anchor.md §4): the demote BOTTOMED at
            // the anchor on a DATA batch and the peer still isn't advancing N(R).
            // The anchor's proof was earned by frames sized at the PREVIOUS
            // anchor's MAX_INFO; its own credit grew MAX_INFO, and full-MTU
            // frames at this level can sit at FER~=1 (measured: 53-84 consecutive
            // clamped demotes per session, 0 REJ ever, terminal T1 N2 disconnect).
            // After OFDM_ANCHOR_FUTILITY_LIMIT consecutive bottomed data rounds,
            // demote the anchor itself through the same path a peer REJ takes.
            // Counter resets on any N(R) advance (RR/I-frame/MFSK credit sites).
            if (demote_refine_ && ofdm_kiss_tx_ &&
                !kiss_control_batch && !batch_is_control &&
                old == ofdm_speed_level_ && ofdm_speed_level_ == tx_acked_level_ &&
                tx_acked_level_ > 0) {
                // INV-SEQ-3 (data-flow-owned-shadow-seq.md §5): at a WIDE in-flight
                // window the cumulative-ACK latency of the flight itself (~1 s/frame
                // at O5: a 24-frame mega-burst ACKs 21-24 s after it starts) matches
                // the legacy 3-round patience — a no-advance round is only EVIDENCE
                // of futility once the peer has had TIME to credit the whole flight.
                // Gate the counter on the ACK clock; legacy (K<=7) timing unchanged
                // (a 7-frame drain is well inside 3 rounds).  This is the spurious
                // demote that armed the s00/s02 desync rollback on a CLEAN channel.
                int outstanding = ax25_session_.window_used();
                int stall_ticks = ax25_session_.wall_ticks() -
                                  ax25_session_.last_ack_progress_tick();
                int need_ticks  = OFDM_FUTILITY_TICKS_PER_OUTSTANDING * outstanding;
                if (outstanding > Ax25Session::K_WINDOW && stall_ticks < need_ticks) {
                    IRIS_LOG("MAC: futility round SKIPPED — ACK clock not expired "
                             "(%d in flight, stalled %.1fs < %.1fs)",
                             outstanding, stall_ticks * 0.05, need_ticks * 0.05);
                } else {
                ofdm_anchor_futility_++;
                IRIS_LOG("MAC: retx futile at anchor floor O%d (%d/%d)",
                         tx_acked_level_, ofdm_anchor_futility_,
                         OFDM_ANCHOR_FUTILITY_LIMIT);
                if (ofdm_anchor_futility_ >= OFDM_ANCHOR_FUTILITY_LIMIT &&
                    demote_tx_anchor("anchor futility: peer answers polls, N(R) frozen",
                                     /*defer_window_mutation=*/true)) {
                    gearshift_.report_failure();
                    // Re-resolve this batch against the lowered floor; if it no
                    // longer fits, the oversized guard rejects it and the pump
                    // re-slices the window on the next idle tick.
                    ofdm_speed_level_ = std::max(tx_acked_level_,
                                                 ofdm_speed_level_ - 1);
                    gearshift_.force_ofdm_level(ofdm_speed_level_);
                }
                }   // INV-SEQ-3 ACK-clock gate (else arm)
            }
        }
    }
    // Peer-SNR cap + TX-feedback ceiling (+ receiver-driven leap), single-sourced
    // with the batch-sizing pass via ofdm_apply_tx_level_caps() so the sized batch
    // and the modulated frame can never describe different levels.
    int peer_snr_level = (peer_snr_db_ > 0) ? ofdm_snr_to_speed_level(peer_snr_db_) : -1;
    int before_caps = ofdm_speed_level_;
    ofdm_speed_level_ = ofdm_apply_tx_level_caps(ofdm_speed_level_, ofdm_kiss_tx_,
                                                 /*no_ack=*/false, tx_acked_level_,
                                                 tx_proposed_level_, peer_snr_level,
                                                 OFDM_LEVEL_LEAP_MAX);
    if (ofdm_kiss_tx_ && tx_proposed_level_ >= 0)
        tx_proposed_level_ = -1;  // consume the one-shot receiver-driven leap (decay)
    if (ofdm_speed_level_ != before_caps)
        IRIS_LOG("MAC: TX level capped O%d -> O%d (acked=%d, peerSNR=%.1f)",
                 before_caps, ofdm_speed_level_, tx_acked_level_, peer_snr_db_);

    // climbgate experiment lock: pin the TX level after all caps so both ends run
    // at exactly O<force> through the relay (answers "does 32QAM decode LIVE").
    if (force_ofdm_level_ >= 0 && !batch_is_control && !kiss_control_batch) {
        ofdm_speed_level_ = force_ofdm_level_;
        gearshift_.force_ofdm_level(force_ofdm_level_);
    }

    // Fix B (DATALINK_TAX_DIAGNOSIS.md): a frame that stuck against the current
    // level's capacity for OFDM_OVERSIZE_REJECT_LIMIT consecutive bursts set a
    // sticky fit-floor (requeue_rejected_frames). Honor it as a FLOOR — after all
    // downshift caps — so the stuck frame modulates at a level that carries it and
    // gets ACKed, instead of re-rejecting forever into a T1 N2 disconnect. Once the
    // CONFIRMED anchor (tx_acked_level_) has climbed to the floor, the gearshift
    // carries the frame natively and the floor is retired. The floor must PERSIST
    // across the intervening control/ACK sends (a KISS ACK is forced to O1 and would
    // otherwise clear it before the stuck DATA frame ever modulates at the floor —
    // the live-lock that delivered 0 B while never disconnecting); it is retired here
    // on anchor-catch-up, not on an unrelated send.
    if (ofdm_fit_floor_level_ >= 0) {
        if (tx_acked_level_ >= ofdm_fit_floor_level_) {
            IRIS_LOG("[TX-OFDM] Fix B: anchor O%d reached floor O%d — retiring floor",
                     tx_acked_level_, ofdm_fit_floor_level_);
            ofdm_fit_floor_level_ = -1;
        } else if (ofdm_fit_floor_level_ > ofdm_speed_level_) {
            IRIS_LOG("[TX-OFDM] Fix B: flooring TX level O%d -> O%d to clear a stuck frame",
                     ofdm_speed_level_, ofdm_fit_floor_level_);
            ofdm_speed_level_ = ofdm_fit_floor_level_;
        }
    }

    // Record data TX level for ACK attribution. Don't record S-frame poll level —
    // polls are forced to O1, which would inflate tx_acked_level_ without proving
    // data frames decode at O1. A3: stamp EVERY outstanding N(S) in [va, vs) at
    // this level so a later N(R) credits the ACK anchor per-frame (the window may
    // span a climb/leap over 2-3 levels; the scalar cannot represent that).
    if (!kiss_control_batch) {
        tx_last_level_ = ofdm_speed_level_;
        int rmod = ax25_session_.seq_mod();   // mod-8 or wide modulo-128
        for (uint8_t n = ax25_session_.va(); n != ax25_session_.vs();
             n = (uint8_t)((n + 1) % rmod))
            tx_level_ring_.stamp(n, ofdm_speed_level_);
    }

    // Build the tone map from the SAME final level (preset = level + 1).
    int sl = std::clamp(ofdm_speed_level_, 0, NUM_OFDM_SPEED_LEVELS - 1);
    if (config_.ofdm_waterfill && ofdm_tone_map_.tone_map_id == 0 &&
        ofdm_tone_map_.total_bits_per_symbol > 0) {
        // Waterfill: keep per-carrier bit loading, update FEC from the level.
        ofdm_tone_map_.fec_rate = ofdm_level_to_fec_rate(sl);
    } else {
        // Uniform: all carriers same modulation.
        uint8_t preset = static_cast<uint8_t>(sl + 1);
        ofdm_tone_map_ = get_uniform_tone_map(preset, ofdm_config_);
    }
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;

    // A2: derive ncw from the FINAL level (never a stale prior-iteration level),
    // THEN apply the capacity-based 1-CW collapse (content-agnostic, item 3) —
    // both now use the final fec/level so ncw and modulation stay coherent.
    int ncw = ofdm_cw_for_level(ofdm_speed_level_);
    if (ofdm_kiss_ && total_bytes > 0) {
        ToneMap one_cw_map = ofdm_tone_map_;
        one_cw_map.n_codewords = 1;
        auto one_cw_geometry = checked_ofdm_frame_geometry(ofdm_config_, one_cw_map);
        auto one_cw_capacity = one_cw_geometry
            ? ofdm_payload_capacity_bytes(*one_cw_geometry) : std::nullopt;
        int one_cw_cap = one_cw_capacity &&
                *one_cw_capacity <= static_cast<std::uint64_t>(INT_MAX)
            ? static_cast<int>(*one_cw_capacity) : 0;
        int collapsed = ofdm_collapse_cw(total_bytes, one_cw_cap, ncw);
        if (collapsed != ncw) {
            IRIS_LOG("[TX-OFDM] batch fits 1 CW (%zu B <= %d cap): 1 CW (was %d)",
                     total_bytes, one_cw_cap, ncw);
            ncw = collapsed;
        }
    }
    ofdm_tone_map_.n_codewords = ncw;
    return ncw;
}

int Modem::test_resolve_ofdm_tx_level(int stale_level, int gearshift_level, int max_ofdm,
                                      bool kiss_control_batch, bool batch_is_control,
                                      size_t total_bytes) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Minimal OFDM config so the tone-map build inside resolve is valid.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);  // 1:12 comb (52 data carriers; 32QAM r5/8 parity)
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    gearshift_.set_max_ofdm_level(max_ofdm);
    gearshift_.force_ofdm_level(gearshift_level);
    ofdm_speed_level_  = stale_level;            // leftover from a prior TX/RX-adopt
    tx_no_ack_count_   = 0;
    peer_snr_db_       = 0;
    tx_proposed_level_ = -1;
    tx_acked_level_    = gearshift_level;        // ceiling = acked+1 admits the target
    tx_level_ring_.reset();
    return resolve_ofdm_tx_level(kiss_control_batch, batch_is_control, total_bytes);
}

size_t Modem::test_burst_reject_requeue(int forced_level, int anchor_level,
                                        const std::vector<std::vector<uint8_t>>& frames) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Minimal OFDM setup + a REAL modulator so build_ofdm_frame actually rejects
    // an over-capacity frame (leg 2, test (d)).
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, config_.ofdm_nfft ? config_.ofdm_nfft : 1024,
                                          config_.ofdm_cp_samples ? config_.ofdm_cp_samples : 64,
                                          12, 24);  // 1:12 comb (52 data carriers; 32QAM r5/8 parity)
    ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
    ofdm_phy_active_    = true;
    ofdm_kiss_          = true;
    ofdm_kiss_tx_       = true;
    ofdm_kiss_peer_caps_ = 0;   // no compression: keep the payload oversized
    // Modulate at the LOW forced level (small capacity); build the tone map to it.
    ofdm_speed_level_ = forced_level;
    ofdm_tone_map_ = get_uniform_tone_map(forced_level + 1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    int ncw = ofdm_cw_for_level(forced_level);
    ofdm_tone_map_.n_codewords = ncw;
    // Confirmed anchor HIGHER than the forced level: a frame that fits the anchor
    // but not the forced level is RECOVERABLE and MUST survive (be re-queued).
    tx_acked_level_ = anchor_level;
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (auto& f : frames)
        tx_queue_.push(TxFrame(std::vector<uint8_t>(f)));   // data, tone_ack_eligible=false
    size_t burst_max_payload = (size_t)ofdm_capacity_bytes_for_level(forced_level, ofdm_config_);
    append_ofdm_burst_frame(burst_max_payload);
    return tx_queue_.size();
}

bool Modem::test_coalesced_slot_midloss_recovery(int n_frames, int kill_idx) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Interior loss only: the mid-slot case is what the coalescing probe adds
    // over the single-frame burst (frame 0 loss == the pre-probe case).
    if (n_frames < 2 || kill_idx <= 0 || kill_idx >= n_frames) return false;

    // In-process OFDM PHY at the proven live-RX test geometry
    // (test_rx_tonemap_latch): real modulator -> real audio -> real sync /
    // demod, no audio cards.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
    ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
    ofdm_rx_iq_.clear();
    ofdm_rx_audio_buf_.clear();
    ofdm_acquisition_.reset();
    ofdm_active_candidate_id_ = 0;
    ofdm_rx_lpf_.reset();
    ofdm_sync_cached_ = false;
    ofdm_chase_llrs_.clear();
    ofdm_chase_combines_ = 0;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = false;        // deliver_ofdm falls through to rx_callback_ (byte-exact)
    ofdm_kiss_tx_ = false;     // TX build: no N(R) re-stamp, no wrapper — raw payloads
    ofdm_kiss_peer_caps_ = 0;  // no batch compression: payload boundaries stay exact
    ofdm_expect_ack_ = false;
    ofdm_kiss_rx_proposed_level_ = -1;
    ptt_active_ = false;
    rx_muted_ = false;
    native_selfhear_guard_ = 0;

    // TX and RX pinned to one level (blind-detect is not under test here).
    const int LVL = 2;
    const int NCW = ofdm_cw_for_level(LVL);
    ofdm_speed_level_ = LVL;
    ofdm_tone_map_ = get_uniform_tone_map(LVL + 1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_tone_map_.n_codewords = NCW;
    ofdm_kiss_rx_level_ = LVL;
    ofdm_rx_tone_map_ = ofdm_rx_tone_map_for_level(LVL, ofdm_config_, config_.ofdm_nuc);
    ofdm_kiss_rx_confirmed_ = true;

    // The probe knobs under test (members the env vars set): frame-count-
    // bounded slot at the default (15 s hard cap) airtime bound.
    probe_frames_per_burst_ = n_frames;
    probe_slot_airtime_s_ = -1.0f;

    // n_frames + 2 distinct payloads; burst_max_payload sized so EXACTLY ONE
    // payload fits per OFDM frame -> the frame-count knob (not capacity or
    // airtime) is what bounds the slot, and the 2 excess must stay queued.
    const size_t PAY = 180;   // fits the O2 single-frame capacity (288 B)
    std::vector<std::vector<uint8_t>> payloads;
    for (int f = 0; f < n_frames + 2; f++) {
        std::vector<uint8_t> p(PAY);
        p[0] = 0x55;   // never MULTI_PAYLOAD_MAGIC (0xBB): no RX unwrap attempt
        for (size_t i = 1; i < PAY; i++)
            p[i] = (uint8_t)(i * 7 + f * 31 + 5);
        payloads.push_back(std::move(p));
    }
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (auto& p : payloads)
        tx_queue_.push(TxFrame(std::vector<uint8_t>(p)));

    // Build the slot through the PRODUCTION drain: main frame + the coalescing
    // loop, recording per-frame sample spans.
    bool ok = true;
    tx_buffer_.clear();
    std::vector<size_t> bounds;               // frame k = [bounds[k], bounds[k+1])
    bounds.push_back(0);
    float frame_airtime_s = append_ofdm_burst_frame(PAY);   // the main frame
    if (frame_airtime_s < 0.0f) {
        IRIS_LOG("[TEST-SLOT] main frame build failed");
        return false;
    }
    bounds.push_back(tx_buffer_.size());
    int burst_frames = append_coalesced_slot(frame_airtime_s, PAY, &bounds);

    // (a) The frame-count knob bounds the slot; the excess survives in queue.
    if (burst_frames != n_frames || (int)bounds.size() != n_frames + 1) {
        IRIS_LOG("[TEST-SLOT] FAIL: slot built %d frames (want %d)",
                 burst_frames, n_frames);
        ok = false;
    }
    if (tx_queue_.size() != 2) {
        IRIS_LOG("[TEST-SLOT] FAIL: %zu frames left queued (want 2 — the knob "
                 "must stop the drain)", tx_queue_.size());
        ok = false;
    }
    if (!ok) return false;

    // The mid-slot loss: zero out frame kill_idx's samples entirely.
    for (size_t i = bounds[kill_idx]; i < bounds[kill_idx + 1]; i++)
        tx_buffer_[i] = 0.0f;

    // RX capture.
    std::vector<std::vector<uint8_t>> delivered;
    rx_callback_ = [&](const uint8_t* d, size_t n) {
        delivered.emplace_back(d, d + n);
    };

    // Continuous low-level noise mixed into everything fed (live RX audio is
    // never digitally silent; a zero floor makes the Schmidl correlator fire
    // on partial preambles — same discipline as test_rx_tonemap_latch).
    uint32_t rng = 0x2468ACEu;
    auto noise = [&]() -> float {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return (((rng >> 8) & 0xFFFF) / 32767.5f - 1.0f);
    };
    double acc = 0.0;
    for (size_t i = bounds[0]; i < bounds[1]; i++)
        acc += (double)tx_buffer_[i] * tx_buffer_[i];
    float frame_rms = (float)std::sqrt(acc / (double)(bounds[1] - bounds[0]));
    float noise_amp = frame_rms * 0.0548f;   // ~-30 dB vs frame RMS

    // Feed the WHOLE slot as one contiguous PTT: first frame's preamble region
    // in one call (acquisition context), then live 10 ms callback granularity.
    {
        size_t head = std::min(tx_buffer_.size(),
            (size_t)(8 * (ofdm_config_.nfft + ofdm_config_.cp_samples)));
        std::vector<float> mixed(tx_buffer_.size());
        for (size_t i = 0; i < tx_buffer_.size(); i++)
            mixed[i] = tx_buffer_[i] + noise_amp * noise();
        process_rx_native(mixed.data(), (int)head);
        const size_t CHUNK = 480;
        for (size_t off = head; off < mixed.size(); off += CHUNK) {
            size_t n = std::min(CHUNK, mixed.size() - off);
            process_rx_native(mixed.data() + off, (int)n);
        }
    }
    // Noise-only tail until the deliveries settle.
    {
        std::vector<float> z(480);
        for (int i = 0; i < 320 && delivered.size() < (size_t)(n_frames - 1); i++) {
            for (size_t j = 0; j < z.size(); j++) z[j] = noise_amp * noise();
            process_rx_native(z.data(), 480);
        }
    }

    // (b) Every frame EXCEPT the killed one decodes byte-exact, in order — a
    // mid-slot loss costs ONE frame, never the slot.
    if (delivered.size() != (size_t)(n_frames - 1)) {
        IRIS_LOG("[TEST-SLOT] FAIL: %zu/%d surviving frames delivered",
                 delivered.size(), n_frames - 1);
        ok = false;
    } else {
        for (int j = 0; j < n_frames - 1; j++) {
            int src = (j < kill_idx) ? j : j + 1;
            if (delivered[j] != payloads[src]) {
                IRIS_LOG("[TEST-SLOT] FAIL: surviving frame %d (payload %d) not "
                         "byte-exact", j, src);
                ok = false;
            }
        }
    }

    // (c) The lost frame is INDIVIDUALLY retransmittable: rebuild it alone
    // through the same drain and decode it after a noise gap.
    while (!tx_queue_.empty()) tx_queue_.pop();
    tx_queue_.push(TxFrame(std::vector<uint8_t>(payloads[kill_idx])));
    tx_buffer_.clear();
    // Re-pin the TX map (RX CRC-fail paths may have poked the gearshift level).
    ofdm_speed_level_ = LVL;
    ofdm_tone_map_ = get_uniform_tone_map(LVL + 1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_tone_map_.n_codewords = NCW;
    float rt = append_ofdm_burst_frame(PAY);
    if (rt < 0.0f) {
        IRIS_LOG("[TEST-SLOT] FAIL: retransmit frame build failed");
        return false;
    }
    {
        std::vector<float> z(480);
        for (int i = 0; i < 120; i++) {   // ~1.2 s noise-only gap
            for (size_t j = 0; j < z.size(); j++) z[j] = noise_amp * noise();
            process_rx_native(z.data(), 480);
        }
        size_t head = std::min(tx_buffer_.size(),
            (size_t)(8 * (ofdm_config_.nfft + ofdm_config_.cp_samples)));
        std::vector<float> mixed(tx_buffer_.size());
        for (size_t i = 0; i < tx_buffer_.size(); i++)
            mixed[i] = tx_buffer_[i] + noise_amp * noise();
        process_rx_native(mixed.data(), (int)head);
        const size_t CHUNK = 480;
        for (size_t off = head; off < mixed.size(); off += CHUNK) {
            size_t n = std::min(CHUNK, mixed.size() - off);
            process_rx_native(mixed.data() + off, (int)n);
        }
        for (int i = 0; i < 320 && delivered.size() < (size_t)n_frames; i++) {
            for (size_t j = 0; j < z.size(); j++) z[j] = noise_amp * noise();
            process_rx_native(z.data(), 480);
        }
    }
    if (delivered.size() != (size_t)n_frames ||
        delivered.back() != payloads[kill_idx]) {
        IRIS_LOG("[TEST-SLOT] FAIL: retransmitted frame not delivered byte-exact "
                 "(%zu delivered)", delivered.size());
        ok = false;
    }

    rx_callback_ = nullptr;
    probe_frames_per_burst_ = -1;
    probe_slot_airtime_s_ = -1.0f;
    return ok;
}

bool Modem::test_tx_queue_no_evict_under_dup_burst() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    while (!tx_queue_.empty()) tx_queue_.pop();

    // The never-sent, distinct in-flight frame (the ics213 stand-in) sits at the
    // FRONT — the drop-oldest victim.  It must SURVIVE the recovery burst.
    std::vector<uint8_t> sentinel(48, 0xAB);
    sentinel[0] = 0xCD;   // distinct lead so it matches nothing in the window

    // A small outstanding window (the K<=7 in-flight I-frames) that a stuck
    // receiver's go-back-N re-emits over and over.
    std::vector<std::vector<uint8_t>> window;
    for (int i = 0; i < 6; i++)
        window.emplace_back(std::vector<uint8_t>(60, (uint8_t)(0x10 + i)));

    // Enqueue the never-sent frame first.
    enqueue_native_tx_frame(std::vector<uint8_t>(sentinel), false);

    // Simulate the connect-recovery storm: 12 go-back-N rounds re-emitting the
    // 6-frame window = 72 duplicate pushes into a 32-cap queue.  Pre-fix this is
    // 72 drop-oldest evictions (the sentinel goes first); post-fix every push
    // after the window is first seen is coalesced, so nothing is evicted.
    for (int round = 0; round < 12; round++)
        for (auto& f : window)
            enqueue_native_tx_frame(std::vector<uint8_t>(f), false);

    // The never-sent frame must still be queued.
    std::queue<TxFrame> scan = tx_queue_;
    while (!scan.empty()) {
        if (scan.front().data == sentinel) return true;
        scan.pop();
    }
    return false;
}

// Eligibility of an ax25_tx_queue_ frame for the AFSK->native migration that
// runs when OFDM activates (both migration sites).  Only a genuine NO-VIA
// I-frame — session data the KISS client queued for the peer — may migrate.
// A via-carrying frame MUST stay on the AFSK path: a digipeat re-emit is a
// contract to retransmit on the channel it was heard on, and an endpoint-via
// session runs over AFSK (endpoint-via over OFDM is deferred; see the
// digipeater design §5.5-2).  Replaces the byte-14 sniff
// ((frame[14] & 1) == 0), a §3.1-class landmine: with a via present, byte 14
// is a shifted callsign character whose bit0 is ALWAYS 0, so every
// via-carrying frame (any type) aliased as an I-frame.
bool Modem::migrate_to_native_eligible(const std::vector<uint8_t>& frame) {
    if (frame.size() <= 14) return false;
    Ax25Frame f;
    return ax25_parse(frame.data(), frame.size(), f) &&
           f.type() == Ax25FrameType::I_FRAME && f.via.empty();
}

bool test_burst_epoch_wrap_rejects_delayed_ack_hook() {
    Ax25Address me = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");
    Ax25Session session;
    session.set_local_callsign("N0AAA");
    session.set_send_callback([](const uint8_t*, size_t) {});
    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    session.notify_outgoing(sabm.data(), sabm.size());
    auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    Ax25Frame ua_frame;
    ax25_parse(ua.data(), ua.size(), ua_frame);
    session.on_frame_received(ua_frame);
    session.set_native_active(true);
    session.set_wide_window(true, 63);
    session.set_max_info(16);
    std::vector<uint8_t> data(160, 0x5A);
    session.send_data(data.data(), data.size());

    auto delayed_rr = ax25_build_s(me, peer, Ax25SType::RR, 8, false,
                                   false, true);
    uint8_t current_epoch = 0;
    for (int i = 0; i < 8; i++)
        current_epoch = next_burst_epoch(current_epoch);
    session.set_reverse_ack_epoch_ok(
        reverse_ack_epoch_matches(0, current_epoch));
    Ax25Frame rr_frame;
    ax25_parse(delayed_rr.data(), delayed_rr.size(), rr_frame, true);
    uint8_t va = session.va();
    int window_used = session.window_used();
    session.on_frame_received(rr_frame);
    return current_epoch == 8 && session.va() == va &&
           session.window_used() == window_used;
}

// In-process digipeater dispatch wiring test (see modem.h).  Configures the
// digipeater directly and feeds `frame` through the REAL dispatch_rx_frame
// (from_fx25=false, from_ofdm=false — the AFSK RX path).  Returns the number
// of frames in ax25_tx_queue_ afterwards; copies the queued repeat out via
// `out_frame` when non-null.
size_t Modem::test_digipeat_dispatch(const DigipeatConfig& dcfg,
                                     const std::vector<uint8_t>& frame,
                                     std::vector<uint8_t>* out_frame) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    std::string derr = digipeater_.configure(dcfg);
    if (!derr.empty())
        IRIS_LOG("[DIGI] test config error: %s", derr.c_str());
    while (!ax25_tx_queue_.empty()) ax25_tx_queue_.pop();
    last_rx_frame_.clear();
    dedup_cooldown_ = 0;
    dispatch_rx_frame(frame, false, false);
    if (out_frame && !ax25_tx_queue_.empty())
        *out_frame = ax25_tx_queue_.front();
    return ax25_tx_queue_.size();
}

int Modem::test_ofdm_max_info_for_level(int level) {
    const OfdmConfig config = ofdm_config_from_probe(narrow_passband());
    return ofdm_max_info_for_level(level, config);
}

int Modem::test_ofdm_capacity_bytes_for_level(int level) {
    const OfdmConfig config = ofdm_config_from_probe(narrow_passband());
    return ofdm_capacity_bytes_for_level(level, config);
}

int Modem::test_ofdm_oversize_fit_floor(int reject_streak, int stuck_frame_bytes) {
    const OfdmConfig config = ofdm_config_from_probe(narrow_passband());
    return ofdm_oversize_fit_floor(reject_streak, stuck_frame_bytes, config);
}

bool Modem::test_burst_fill_continue(bool enabled, bool ofdm_kiss_tx,
                                     bool session_active, bool we_initiated,
                                     bool have_queued_data, int window_used,
                                     int window_k) {
    return burst_fill_continue(enabled, ofdm_kiss_tx, session_active, we_initiated,
                               have_queued_data, window_used, window_k);
}

bool Modem::test_ofdm_root2_retain(bool short_ack_gate_used, bool decode_success,
                                   bool llrs_nonempty) {
    return ofdm_root2_retain_on_short_ack_fail(short_ack_gate_used, decode_success,
                                               llrs_nonempty);
}

int Modem::test_ofdm_gate_n_cw(bool kiss_tx, int map_ncw, int rx_level) {
    return ofdm_gate_n_cw(kiss_tx, map_ncw, rx_level);
}

bool Modem::test_ofdm_sweep_skip_level(int lvl, int rx_level, int persisted_ncw) {
    return ofdm_sweep_skip_level(lvl, rx_level, persisted_ncw);
}

// RX tone-map latch regression (data-flow-rx-tonemap.md §6) — drives the LIVE
// process_rx_native() OFDM RX path in-process (real modulator -> real audio ->
// real sync / frame-length gate / demod / blind-detect), no audio cards.
//
// force_poisoned_state == false ("sequence"): from a healthy latched
//   (O7, full-CW) state, a 1-CW control-sized frame at O6 is blind-detected
//   (the RR-poll analog), then an 8-CW O6 data frame arrives. FAIL-BEFORE: the
//   sweep persisted the 1-CW trial map, the gate then released the data frame
//   truncated and it NEVER decoded — the captured 941 s wedge cycle.
//   PASS-AFTER: the persistent map stays full-CW (INV-1) and the data frame
//   primary-decodes byte-exact.
// force_poisoned_state == true ("poisoned"): the wedge state itself is FORCED
//   (persistent map O6/1-CW, confirmed) and the 8-CW O6 data frame is fed.
//   FAIL-BEFORE: absorbing state — (O6, 8 CW) structurally untriable, frame
//   lost every time. PASS-AFTER: the gate floor buffers the full frame, the
//   now-reachable (O6, 8 CW) sweep trial decodes it byte-exact, and the
//   persistent map self-heals to full-CW.
bool Modem::test_rx_tonemap_latch(bool force_poisoned_state) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);

    // In-process OFDM PHY at the proven test geometry (tests.cc blind-detect).
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    ofdm_config_ = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
    ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
    ofdm_rx_iq_.clear();
    ofdm_rx_audio_buf_.clear();
    ofdm_acquisition_.reset();
    ofdm_active_candidate_id_ = 0;
    ofdm_rx_lpf_.reset();
    ofdm_sync_cached_ = false;
    ofdm_chase_llrs_.clear();
    ofdm_chase_combines_ = 0;
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_phy_active_ = true;
    ofdm_kiss_ = false;        // deliver via rx_callback_ (byte-exact capture)
    ofdm_kiss_tx_ = true;      // the KISS RX gate/sweep paths under test
    ofdm_expect_ack_ = false;
    ofdm_kiss_rx_proposed_level_ = -1;
    ptt_active_ = false;
    rx_muted_ = false;
    native_selfhear_guard_ = 0;

    const int LVL_DATA = 6;    // 32QAM: full data shape is 8 CW
    const int FULL_CW = ofdm_cw_for_level(LVL_DATA);
    if (FULL_CW <= 1) {
        IRIS_LOG("[TEST-LATCH] geometry error: O%d not multi-CW", LVL_DATA);
        return false;
    }

    std::vector<std::vector<uint8_t>> delivered;
    rx_callback_ = [&](const uint8_t* d, size_t n) {
        delivered.emplace_back(d, d + n);
    };

    auto build_audio = [&](const std::vector<uint8_t>& payload, int level, int ncw)
        -> std::vector<float> {
        ToneMap tm = ofdm_rx_tone_map_for_level(level, ofdm_config_, config_.ofdm_nuc);
        tm.n_codewords = ncw;
        auto iq = ofdm_mod_->build_ofdm_frame(payload.data(), payload.size(), tm);
        std::vector<float> audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++) audio[i] = iq[i].real();
        return audio;
    };
    // Continuous low-level noise (~30 dB below the frame RMS), mixed into
    // EVERYTHING fed — frames and inter-frame gaps alike. Live RX audio is
    // never digitally silent; against a zero floor the Schmidl correlator
    // fires on a PARTIAL preamble mid-feed (measured M=0.835 on the first few
    // chunks of a clean preamble against pure silence), caching a misaligned
    // sync that then consumes the real frame. The noise floor keeps the
    // detector honest, exactly as on the WGN bench.
    uint32_t rng = 0x1234567u;
    auto noise = [&]() -> float {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return (((rng >> 8) & 0xFFFF) / 32767.5f - 1.0f);  // uniform [-1,1)
    };
    float noise_amp = 0.0f;  // set from the first built frame's RMS below

    // Frame feeding. The PREAMBLE REGION (training pair + sync + margin, 8
    // symbols) is delivered in ONE call so acquisition sees the complete
    // training pair with post-preamble context: when the buffer already holds
    // noise, the per-chunk detector otherwise fires mid-preamble at the buffer
    // edge (measured M=0.834 at exactly training-pair-complete) and caches a
    // biased sync — a real, base-present acquisition weakness that is OUT OF
    // SCOPE here and would mask the latch path under test. The REMAINDER is
    // fed at the live callback granularity (10 ms @ 48 kHz) so the
    // frame-length gate fires exactly as it does live — releasing as soon as
    // ITS notion of the frame length is buffered (the truncation under test).
    auto feed_frame = [&](const std::vector<float>& audio) {
        std::vector<float> mixed(audio.size());
        for (size_t i = 0; i < audio.size(); i++)
            mixed[i] = audio[i] + noise_amp * noise();
        size_t head = std::min(audio.size(),
            (size_t)(8 * (ofdm_config_.nfft + ofdm_config_.cp_samples)));
        process_rx_native(mixed.data(), (int)head);
        const size_t CHUNK = 480;
        for (size_t off = head; off < audio.size(); off += CHUNK) {
            size_t n = std::min(CHUNK, audio.size() - off);
            process_rx_native(mixed.data() + off, (int)n);
        }
    };
    auto feed_silence_until = [&](size_t want_delivered) {
        std::vector<float> z(480);
        for (int i = 0; i < 320 && delivered.size() < want_delivered; i++) {
            for (size_t j = 0; j < z.size(); j++) z[j] = noise_amp * noise();
            process_rx_native(z.data(), 480);
        }
    };
    // Noise-only inter-frame gap (live frames are separated by seconds of
    // noise-only air; the no-detection trim keeps the RX buffer short, so the
    // next preamble is acquired the same way it is on the bench).
    auto feed_gap = [&](int chunks) {
        std::vector<float> z(480);
        for (int i = 0; i < chunks; i++) {
            for (size_t j = 0; j < z.size(); j++) z[j] = noise_amp * noise();
            process_rx_native(z.data(), 480);
        }
    };
    auto rms_of = [](const std::vector<float>& a) -> float {
        double acc = 0.0;
        for (float v : a) acc += (double)v * v;
        return a.empty() ? 0.0f : (float)std::sqrt(acc / a.size());
    };

    std::vector<uint8_t> payload_ctl(20), payload_data(500);
    for (size_t i = 0; i < payload_ctl.size(); i++)
        payload_ctl[i] = (uint8_t)(i * 37 + 13);
    for (size_t i = 0; i < payload_data.size(); i++)
        payload_data[i] = (uint8_t)(i * 151 + 7);

    bool ok = true;
    size_t expect_delivered = 0;

    if (!force_poisoned_state) {
        // Healthy latched state at O7 (full-CW), as before the captured poll.
        ofdm_kiss_rx_level_ = 7;
        ofdm_rx_tone_map_ = ofdm_rx_tone_map_for_level(7, ofdm_config_,
                                                       config_.ofdm_nuc);
        ofdm_kiss_rx_confirmed_ = true;

        // The RR-poll analog: 1-CW control-sized frame at O6, found by the
        // blind-detect sweep (level change O7 -> O6).
        auto a1 = build_audio(payload_ctl, LVL_DATA, 1);
        if (a1.empty()) { IRIS_LOG("[TEST-LATCH] ctl frame build failed"); ok = false; }
        // -30 dB noise floor vs the frame RMS (uniform sigma = amp/sqrt(3)).
        if (noise_amp == 0.0f) noise_amp = rms_of(a1) * 0.0548f;
        feed_frame(a1);
        feed_silence_until(++expect_delivered);
        if (delivered.size() != expect_delivered ||
            delivered.back() != payload_ctl) {
            IRIS_LOG("[TEST-LATCH] FAIL: 1-CW control frame not delivered "
                     "byte-exact (%zu delivered)", delivered.size());
            ok = false;
        }
        if (ofdm_kiss_rx_level_ != LVL_DATA) {
            IRIS_LOG("[TEST-LATCH] FAIL: sweep did not move RX level to O%d "
                     "(at O%d)", LVL_DATA, ofdm_kiss_rx_level_);
            ok = false;
        }
        // THE POISON WITNESS: the persistent map must keep the FULL data-frame
        // codeword count (INV-1). Fails at base: the 1-CW trial map persisted.
        if (ofdm_rx_tone_map_.n_codewords != FULL_CW) {
            IRIS_LOG("[TEST-LATCH] FAIL: persistent RX map poisoned to %d CW "
                     "(want %d) after control-frame blind detect",
                     ofdm_rx_tone_map_.n_codewords, FULL_CW);
            ok = false;
        }
    } else {
        // Force the captured wedge state directly.
        ofdm_kiss_rx_level_ = LVL_DATA;
        ofdm_rx_tone_map_ = ofdm_rx_tone_map_for_level(LVL_DATA, ofdm_config_,
                                                       config_.ofdm_nuc);
        ofdm_rx_tone_map_.n_codewords = 1;   // the poison
        ofdm_kiss_rx_confirmed_ = true;
    }

    // The stuck-I-frame analog: full 8-CW data frame at O6, after a
    // noise-only gap (~1.2 s of air), as on the bench.
    auto a2 = build_audio(payload_data, LVL_DATA, FULL_CW);
    if (a2.empty()) { IRIS_LOG("[TEST-LATCH] data frame build failed"); ok = false; }
    if (noise_amp == 0.0f) noise_amp = rms_of(a2) * 0.0548f;
    feed_gap(120);
    feed_frame(a2);
    feed_silence_until(++expect_delivered);
    if (delivered.size() != expect_delivered ||
        delivered.back() != payload_data) {
        IRIS_LOG("[TEST-LATCH] FAIL: %d-CW data frame not delivered byte-exact "
                 "(%zu/%zu delivered) — the RX wedge",
                 FULL_CW, delivered.size(), expect_delivered);
        ok = false;
    }
    // The map must be (back) at the full data shape either way.
    if (ofdm_kiss_rx_level_ != LVL_DATA ||
        ofdm_rx_tone_map_.n_codewords != FULL_CW) {
        IRIS_LOG("[TEST-LATCH] FAIL: persistent map (O%d, %d CW) after data "
                 "frame (want O%d, %d CW)", ofdm_kiss_rx_level_,
                 ofdm_rx_tone_map_.n_codewords, LVL_DATA, FULL_CW);
        ok = false;
    }

    rx_callback_ = nullptr;
    return ok;
}

void Modem::force_activate_ofdm(const std::string& callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("[FORCE-OFDM] Activating OFDM immediately for %s (no probe)", callsign.c_str());
    if (gui_log_) gui_log_("[FORCE-OFDM] Activating for " + callsign);

    // Native mode flags
    peer_is_iris_ = true;
    ofdm_kiss_ = true;
    ofdm_config_mismatch_ = false;  // fresh session: re-evaluate config echo (item 5)
    ofdm_kiss_probe_done_ = true;
    // TX-hold invariant (item 6): initiator transmits first;
    // responder holds TX until it decodes the initiator's first OFDM frame
    // (modem.cc:2193/:2499). Applies to --force-ofdm too (a lab switch) so it
    // stops racing SABM/UA before either side confirms activation.
    {
        bool we_init = ax25_session_.we_initiated();
        ofdm_kiss_tx_ = we_init;
        if (we_init) ax25_session_.set_native_active(true);
    }
    ax25_session_.start_t1_if_unacked();

    // Synthetic passband from config defaults
    NegotiatedPassband ofdm_pb;
    ofdm_pb.low_hz = config_.band_low_hz;
    ofdm_pb.high_hz = config_.band_high_hz;
    ofdm_pb.center_hz = (config_.band_low_hz + config_.band_high_hz) / 2.0f;
    ofdm_pb.bandwidth_hz = config_.band_high_hz - config_.band_low_hz;
    ofdm_pb.valid = true;

    int nfft = config_.ofdm_nfft;
    int cp = config_.ofdm_cp_samples;
    int carrier_pilot_spacing = 12;  // 1:12 comb (52 data carriers; 32QAM r5/8 parity)
    int block_pilot_spacing = 24;

    ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                           carrier_pilot_spacing, block_pilot_spacing);
    ofdm_config_.fm_preemph_corner_hz = config_.ofdm_preemph_corner_hz;
    ofdm_config_.clean_channel = config_.ofdm_clean_channel;
    ofdm_config_.skip_papr_clip = config_.ofdm_skip_papr_clip;
    ofdm_config_.llr_use_frame_nv = config_.ofdm_llr_use_frame_nv;
    ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
    ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
    ofdm_rx_iq_.clear();
    ofdm_rx_audio_buf_.clear();
    ofdm_acquisition_.reset();
    ofdm_active_candidate_id_ = 0;
    ofdm_rx_lpf_.reset();
    ofdm_sync_cached_ = false;

    gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
    gearshift_.set_kiss_fast_ramp(false);
    reset_level_state();  // A4: atomic reset (V1-V6 + ring + gearshift O0)
    ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
    ofdm_rx_tone_map_ = ofdm_tone_map_;

    // Assume peer has same capabilities (both use --force-ofdm)
    ofdm_kiss_peer_caps_ = local_cap_.capabilities;

    if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
        ofdm_kiss_tx_compressor_.init();
        ofdm_kiss_rx_compressor_.init();
    }

    ofdm_phy_active_ = true;
    // Force mode is a bilateral operator configuration (both endpoints are
    // required to use the same profile), so it supplies the same complete local
    // and peer capability set to the common activation gate.
    v2_negotiated_active_ = (ofdm_kiss_peer_caps_ & CAP_OFDM) != 0;
    // Initialize MFSK tone ACK
    {
        assert_mfsk_nfft(ofdm_config_.nfft);  // tone-ACK bin-unit invariant (§1.9)
        int center_bin = freq_to_bin(ofdm_config_.center_hz, ofdm_config_.nfft, config_.sample_rate);
        int first_bin = center_bin - MfskAck::M / 2;
        if (first_bin < 1) first_bin = 1;
        mfsk_ack_.init(first_bin, config_.sample_rate);
    }
    int ofdm_max_info = ofdm_max_info_for_level(tx_acked_level_, ofdm_config_);  // leg 3 (O0 at reset → 75)
    ax25_session_.set_max_info(ofdm_max_info);
    ax25_session_.drop_oversized_in_window();
    tx_level_ring_.reset();  // A5d: purged window frames must not credit the anchor

    IRIS_LOG("[FORCE-OFDM] ACTIVE: %d carriers (%d data, %d pilot), nfft=%d, BW=%.0f Hz",
             ofdm_config_.n_used_carriers, ofdm_config_.n_data_carriers,
             ofdm_config_.n_pilot_carriers, nfft, ofdm_config_.bandwidth_hz);
}

bool Modem::try_use_cached_probe(const std::string& callsign) {
    auto it = probe_cache_.find(callsign);
    if (it == probe_cache_.end()) {
        // Try loading from disk
        ProbeCacheEntry disk_entry;
        if (load_probe_from_disk(callsign, disk_entry)) {
            probe_cache_[callsign] = std::move(disk_entry);
            it = probe_cache_.find(callsign);
        } else {
            return false;
        }
    }

    auto& entry = it->second;
    auto age = std::chrono::steady_clock::now() - entry.timestamp;
    int age_s = (int)std::chrono::duration_cast<std::chrono::seconds>(age).count();
    if (age_s > PROBE_CACHE_EXPIRY_S) {
        IRIS_LOG("[PROBE-CACHE] expired for %s (%d s old)", callsign.c_str(), age_s);
        probe_cache_.erase(it);
        return false;
    }

    // Replay probe results: apply cached passband, configure PHY, enable native mode.
    // This mirrors the probe completion path in tick() but skips the actual probe.
    ofdm_kiss_probe_done_ = true;
    dcd_holdoff_ = 0;

    float low = entry.negotiated.low_hz;
    float high = entry.negotiated.high_hz;
    float bandwidth = high - low;
    float center = (low + high) / 2.0f;
    config_.band_low_hz = low;
    config_.band_high_hz = high;

    if (use_upconvert_) {
        upconverter_ = Upconverter(center, config_.sample_rate);
        downconverter_ = Downconverter(center, config_.sample_rate);

        constexpr float MAX_OCCUPIED_BW_HZ = 20000.0f;
        float usable_bw = std::min(bandwidth - 200.0f, MAX_OCCUPIED_BW_HZ);
        constexpr int SPS_MIN = 6;
        constexpr int SPS_MAX = 80;
        int new_sps = -1;
        int new_baud = 0;
        for (int sps = SPS_MIN; sps <= SPS_MAX; sps++) {
            int baud = config_.sample_rate / sps;
            float sig_bw = baud * (1.0f + phy_config_.rrc_alpha);
            if (sig_bw <= usable_bw) {
                new_sps = sps;
                new_baud = baud;
                break;
            }
        }
        if (new_sps < 0) {
            new_sps = phy_config_.samples_per_symbol;
            new_baud = phy_config_.baud_rate;
        }

        if (new_baud != phy_config_.baud_rate) {
            phy_config_.baud_rate = new_baud;
            phy_config_.samples_per_symbol = new_sps;
            native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
            native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
        }
    }

    // Channel equalization from cached probe data
    rx_channel_eq_.configure(entry.their_tx, entry.negotiated, config_.sample_rate, 3.0f);
    tx_channel_eq_.configure(entry.my_tx, entry.negotiated, config_.sample_rate, 6.0f);

    // Enable native mode
    peer_is_iris_ = true;
    ofdm_kiss_ = true;
    ofdm_config_mismatch_ = false;  // fresh session: re-evaluate config echo (item 5)
    // TX-hold invariant (item 6): initiator transmits first;
    // responder holds TX until it decodes the initiator's first OFDM frame
    // (modem.cc:2193/:2499) — even on a cached-probe warm start.
    {
        bool we_init = ax25_session_.we_initiated();
        ofdm_kiss_tx_ = we_init;
        if (we_init) ax25_session_.set_native_active(true);
    }

    // OFDM PHY setup from cached passband
    if (config_.ofdm_enable) {
        NegotiatedPassband ofdm_pb;
        ofdm_pb.low_hz = low;
        ofdm_pb.high_hz = high;
        ofdm_pb.center_hz = center;
        ofdm_pb.bandwidth_hz = bandwidth;
        ofdm_pb.valid = true;

        int cp = config_.ofdm_cp_samples;
        int carrier_pilot_spacing = 12;  // 1:12 comb (52 data carriers; 32QAM r5/8 parity)
        int block_pilot_spacing = 24;
        int nfft = config_.ofdm_nfft;

        // Use negotiated params from cached peer result
        if (entry.my_tx.ofdm_cp_samples > 0)
            cp = std::max(cp, (int)entry.my_tx.ofdm_cp_samples);
        if (entry.my_tx.ofdm_pilot_carrier_spacing > 0)
            carrier_pilot_spacing = std::min(carrier_pilot_spacing, (int)entry.my_tx.ofdm_pilot_carrier_spacing);
        if (entry.my_tx.ofdm_pilot_symbol_spacing > 0)
            block_pilot_spacing = std::min(block_pilot_spacing, (int)entry.my_tx.ofdm_pilot_symbol_spacing);
        {
            // nfft_code: code 0 = ABSENT (old peer / unseeded/poisoned cache) →
            // keep the LOCAL default, NOT 512 (D2, clone of
            // the probe-complete guard).  1=256, 2=1024, 3=512.
            bool has_ofdm_cfg = (entry.my_tx.ofdm_nfft_code > 0);
            if (has_ofdm_cfg) {
                int peer_nfft = nfft;  // unknown code → local default
                if (entry.my_tx.ofdm_nfft_code == 1) peer_nfft = 256;
                else if (entry.my_tx.ofdm_nfft_code == 2) peer_nfft = 1024;
                else if (entry.my_tx.ofdm_nfft_code == 3) peer_nfft = 512;
                nfft = std::min(nfft, peer_nfft);
            }
        }

        ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                               carrier_pilot_spacing, block_pilot_spacing);
        ofdm_config_.clean_channel = config_.ofdm_clean_channel;
        ofdm_config_.skip_papr_clip = config_.ofdm_skip_papr_clip;
        ofdm_config_.llr_use_frame_nv = config_.ofdm_llr_use_frame_nv;
        // Auto-detect from cached probe (their_tx = our analysis of their probe)
        if (config_.ofdm_preemph_corner_hz == 0.0f) {
            ofdm_config_.fm_preemph_corner_hz = 0.0f;
        } else {
            ofdm_config_.fm_preemph_corner_hz = probe_detect_preemph_corner(entry.their_tx);
        }
        ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
        ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
        // [GRID] cached-replay grid dump (both ends). Post-fix the cached
        // negotiated band is deterministic, so the two ends' [GRID] fp must
        // match here too; a mismatch means a stale pre-fix (divergent) cache
        // entry — the fingerprint guard then purges it and forces a fresh probe.
        {
            int fb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.front();
            int lb = ofdm_config_.used_carrier_bins.empty() ? -1 : ofdm_config_.used_carrier_bins.back();
            IRIS_LOG("[GRID] (cached) role=%s nfft=%d cp=%d n_used=%d bins=%d-%d band=%.1f-%.1f Hz fp=0x%04X",
                     ax25_session_.we_initiated() ? "CMD" : "RSP",
                     ofdm_config_.nfft, ofdm_config_.cp_samples,
                     ofdm_config_.n_used_carriers, fb, lb,
                     ofdm_pb.low_hz, ofdm_pb.high_hz,
                     ofdm_config_.n_used_carriers > 0 ? ofdm_config_fingerprint(ofdm_config_) : 0);
        }
        ofdm_rx_iq_.clear();
        ofdm_rx_audio_buf_.clear();
        ofdm_acquisition_.reset();
        ofdm_active_candidate_id_ = 0;
        ofdm_rx_lpf_.reset();
        ofdm_sync_cached_ = false;

        gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        gearshift_.set_kiss_fast_ramp(false);
        reset_level_state();  // A4: atomic reset (V1-V6 + ring + gearshift O0)
        ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
        ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
        ofdm_rx_tone_map_ = ofdm_tone_map_;

        // Peer caps from cached probe
        uint16_t peer_caps = entry.my_tx.capabilities;
        ofdm_kiss_peer_caps_ = local_cap_.capabilities & peer_caps;

        if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
            ofdm_kiss_tx_compressor_.init();
            ofdm_kiss_rx_compressor_.init();
        }

        if (config_.ofdm_enable && ofdm_mod_ && (ofdm_kiss_peer_caps_ & CAP_OFDM)) {
            ofdm_phy_active_ = true;
            {
                assert_mfsk_nfft(ofdm_config_.nfft);  // tone-ACK bin-unit invariant (§1.9)
                int center_bin = freq_to_bin(ofdm_config_.center_hz, ofdm_config_.nfft, config_.sample_rate);
                int first_bin = center_bin - MfskAck::M / 2;
                if (first_bin < 1) first_bin = 1;
                mfsk_ack_.init(first_bin, config_.sample_rate);
            }
            int ofdm_max_info = ofdm_max_info_for_level(tx_acked_level_, ofdm_config_);  // leg 3 (O0 at reset → 75)
            ax25_session_.set_max_info(ofdm_max_info);
            ax25_session_.drop_oversized_in_window();
            tx_level_ring_.reset();  // A5d: purged window frames must not credit the anchor
            IRIS_LOG("[PROBE-CACHE] OFDM PHY ACTIVE: %d carriers, BW=%.0f Hz",
                     ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz);
        }
        v2_negotiated_active_ = ofdm_phy_active_ &&
            (ofdm_kiss_peer_caps_ & CAP_OFDM) != 0;
    }

    // Speed level cache
    int cached_level = gearshift_.load_cached_level(callsign);
    if (cached_level > 0) {
        gearshift_.force_level(cached_level);
        IRIS_LOG("[PROBE-CACHE] gearshift: cached level %d", cached_level);
    }

    // Shorter listen window for cached probe (no probe tones to wait for)
    csma_holdoff_ = config_.sample_rate;  // 1s listen (vs 3s for fresh probe)

    // Adaptive TXDELAY for cached probe
    ofdm_txdelay_ms_ = std::max(50, config_.ptt_pre_delay_ms / 2);

    IRIS_LOG("[PROBE-CACHE] applied cached probe for %s (age=%ds, band %.0f-%.0f Hz)",
             callsign.c_str(), age_s, low, high);
    return true;
}

// ---------------------------------------------------------------------------
// Probe cache disk persistence
// ---------------------------------------------------------------------------

static std::string probe_cache_key(const std::string& callsign) {
    std::string key;
    for (char c : callsign) {
        if (c == '-') break;  // strip SSID
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
            key += c;
        else if (c >= 'a' && c <= 'z')
            key += (c - 32);
    }
    return key;
}

static std::string probe_cache_dir(const std::string& data_dir) {
    return data_dir + "/probe_cache";
}

static void ensure_probe_cache_dir(const std::string& data_dir) {
    std::string dir = probe_cache_dir(data_dir);
#ifdef _WIN32
    _mkdir(dir.c_str());
#else
    mkdir(dir.c_str(), 0755);
#endif
}

bool Modem::save_probe_to_disk(const std::string& callsign, const ProbeCacheEntry& entry) {
    if (config_.data_dir.empty() || callsign.empty()) return false;

    std::string key = probe_cache_key(callsign);
    if (key.empty()) return false;

    ensure_probe_cache_dir(config_.data_dir);
    std::string path = probe_cache_dir(config_.data_dir) + "/" + key + ".txt";

    FILE* f = fopen(path.c_str(), "w");
    if (!f) {
        IRIS_LOG("[PROBE-CACHE] failed to write %s", path.c_str());
        return false;
    }

    // Timestamp (wall-clock for cross-session expiry)
    fprintf(f, "timestamp %lld\n", (long long)time(nullptr));

    // Negotiated passband
    fprintf(f, "neg_low %.6f\n", entry.negotiated.low_hz);
    fprintf(f, "neg_high %.6f\n", entry.negotiated.high_hz);
    fprintf(f, "neg_center %.6f\n", entry.negotiated.center_hz);
    fprintf(f, "neg_bw %.6f\n", entry.negotiated.bandwidth_hz);
    fprintf(f, "neg_valid %d\n", entry.negotiated.valid ? 1 : 0);

    // Write a ProbeResult section (my_tx or their_tx)
    auto write_probe = [&](const char* prefix, const ProbeResult& pr) {
        fprintf(f, "%s_low %.6f\n", prefix, pr.low_hz);
        fprintf(f, "%s_high %.6f\n", prefix, pr.high_hz);
        fprintf(f, "%s_tones %d\n", prefix, pr.tones_detected);
        fprintf(f, "%s_valid %d\n", prefix, pr.valid ? 1 : 0);
        fprintf(f, "%s_caps %u\n", prefix, (unsigned)pr.capabilities);
        fprintf(f, "%s_ofdm_cp %u\n", prefix, (unsigned)pr.ofdm_cp_samples);
        fprintf(f, "%s_ofdm_pcs %u\n", prefix, (unsigned)pr.ofdm_pilot_carrier_spacing);
        fprintf(f, "%s_ofdm_pss %u\n", prefix, (unsigned)pr.ofdm_pilot_symbol_spacing);
        fprintf(f, "%s_ofdm_nfft %u\n", prefix, (unsigned)pr.ofdm_nfft_code);

        // Tone arrays (64 entries each)
        for (int i = 0; i < PassbandProbeConfig::N_TONES; i++)
            fprintf(f, "%s_tp %d %.4f\n", prefix, i, pr.tone_power_db[i]);
        for (int i = 0; i < PassbandProbeConfig::N_TONES; i++)
            fprintf(f, "%s_td %d %d\n", prefix, i, pr.tone_detected[i] ? 1 : 0);
    };

    write_probe("my_tx", entry.my_tx);
    write_probe("their_tx", entry.their_tx);

    // Also persist the NegotiatedPassband's embedded copies
    write_probe("neg_my", entry.negotiated.my_tx_their_rx);
    write_probe("neg_their", entry.negotiated.their_tx_my_rx);

    fclose(f);
    IRIS_LOG("[PROBE-CACHE] saved to disk: %s", path.c_str());
    return true;
}

// item 5: drop a peer's cached probe (memory + disk) when its resolved OfdmConfig
// desynced (config-mismatch latch). Without this, try_use_cached_probe replays
// the split config AND clears ofdm_config_mismatch_ on the next reconnect, so the
// AFSK fallback never sticks (D4 cache poison). Both stores must go: the on-disk
// entry is reloaded into memory on a cache miss (try_use_cached_probe :6497).
void Modem::purge_probe_cache(const std::string& callsign) {
    auto it = probe_cache_.find(callsign);
    if (it != probe_cache_.end()) probe_cache_.erase(it);
    if (config_.data_dir.empty() || callsign.empty()) return;
    std::string key = probe_cache_key(callsign);
    if (key.empty()) return;
    std::string path = probe_cache_dir(config_.data_dir) + "/" + key + ".txt";
    if (remove(path.c_str()) == 0)
        IRIS_LOG("[PROBE-CACHE] purged %s (config-mismatch latch)", path.c_str());
}

bool Modem::load_probe_from_disk(const std::string& callsign, ProbeCacheEntry& entry) {
    if (config_.data_dir.empty() || callsign.empty()) return false;

    std::string key = probe_cache_key(callsign);
    if (key.empty()) return false;

    std::string path = probe_cache_dir(config_.data_dir) + "/" + key + ".txt";
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return false;

    long long disk_timestamp = 0;
    char line[256];

    // Helper to parse a ProbeResult from the file
    auto read_probe = [](const char* prefix, const char* key_str, const char* val_str,
                         ProbeResult& pr, int idx) -> bool {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s_low", prefix);
        if (strcmp(key_str, buf) == 0) { pr.low_hz = (float)atof(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_high", prefix);
        if (strcmp(key_str, buf) == 0) { pr.high_hz = (float)atof(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_tones", prefix);
        if (strcmp(key_str, buf) == 0) { pr.tones_detected = atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_valid", prefix);
        if (strcmp(key_str, buf) == 0) { pr.valid = atoi(val_str) != 0; return true; }
        snprintf(buf, sizeof(buf), "%s_caps", prefix);
        if (strcmp(key_str, buf) == 0) { pr.capabilities = (uint16_t)atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_ofdm_cp", prefix);
        if (strcmp(key_str, buf) == 0) { pr.ofdm_cp_samples = (uint8_t)atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_ofdm_pcs", prefix);
        if (strcmp(key_str, buf) == 0) { pr.ofdm_pilot_carrier_spacing = (uint8_t)atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_ofdm_pss", prefix);
        if (strcmp(key_str, buf) == 0) { pr.ofdm_pilot_symbol_spacing = (uint8_t)atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_ofdm_nfft", prefix);
        if (strcmp(key_str, buf) == 0) { pr.ofdm_nfft_code = (uint8_t)atoi(val_str); return true; }
        snprintf(buf, sizeof(buf), "%s_tp", prefix);
        if (strcmp(key_str, buf) == 0 && idx >= 0 && idx < PassbandProbeConfig::N_TONES) {
            pr.tone_power_db[idx] = (float)atof(val_str); return true;
        }
        snprintf(buf, sizeof(buf), "%s_td", prefix);
        if (strcmp(key_str, buf) == 0 && idx >= 0 && idx < PassbandProbeConfig::N_TONES) {
            pr.tone_detected[idx] = atoi(val_str) != 0; return true;
        }
        return false;
    };

    while (fgets(line, sizeof(line), f)) {
        char key_str[64] = {};
        int idx = -1;
        char val_str[64] = {};

        // Try 3-field format first: "prefix_tp <idx> <val>"
        if (sscanf(line, "%63s %d %63s", key_str, &idx, val_str) >= 2) {
            // Check if it's a 2-field line (key value) vs 3-field (key idx value)
            // 2-field lines: the second token is the value, idx parse may have succeeded as int
            char test_key[64] = {};
            char test_val[64] = {};
            int nfields = sscanf(line, "%63s %63s", test_key, test_val);

            if (strcmp(key_str, "timestamp") == 0) {
                disk_timestamp = atoll(test_val);
                continue;
            }
            if (strcmp(key_str, "neg_low") == 0) { entry.negotiated.low_hz = (float)atof(test_val); continue; }
            if (strcmp(key_str, "neg_high") == 0) { entry.negotiated.high_hz = (float)atof(test_val); continue; }
            if (strcmp(key_str, "neg_center") == 0) { entry.negotiated.center_hz = (float)atof(test_val); continue; }
            if (strcmp(key_str, "neg_bw") == 0) { entry.negotiated.bandwidth_hz = (float)atof(test_val); continue; }
            if (strcmp(key_str, "neg_valid") == 0) { entry.negotiated.valid = atoi(test_val) != 0; continue; }

            // Try all probe result prefixes
            // For 3-field lines (_tp, _td), idx and val_str are correct
            // For 2-field lines, idx is actually the value, reparse
            bool is_tone_line = (strstr(key_str, "_tp") || strstr(key_str, "_td"));
            if (!is_tone_line) {
                // 2-field: key=key_str, value=test_val (which is the second token)
                idx = -1;
                snprintf(val_str, sizeof(val_str), "%s", test_val);
            }

            if (nfields >= 2) {
                read_probe("my_tx", key_str, val_str, entry.my_tx, idx);
                read_probe("their_tx", key_str, val_str, entry.their_tx, idx);
                read_probe("neg_my", key_str, val_str, entry.negotiated.my_tx_their_rx, idx);
                read_probe("neg_their", key_str, val_str, entry.negotiated.their_tx_my_rx, idx);
            }
        }
    }
    fclose(f);

    // Check expiry using wall-clock time
    time_t now = time(nullptr);
    int age_s = (int)(now - (time_t)disk_timestamp);
    if (disk_timestamp == 0 || age_s > PROBE_CACHE_EXPIRY_S || age_s < 0) {
        IRIS_LOG("[PROBE-CACHE] disk entry expired for %s (age=%ds), removing", callsign.c_str(), age_s);
        remove(path.c_str());
        return false;
    }

    if (!entry.negotiated.valid) {
        IRIS_LOG("[PROBE-CACHE] disk entry invalid for %s, removing", callsign.c_str());
        remove(path.c_str());
        return false;
    }

    // Set timestamp relative to steady_clock (approximate: now minus age)
    entry.timestamp = std::chrono::steady_clock::now() - std::chrono::seconds(age_s);

    IRIS_LOG("[PROBE-CACHE] loaded from disk: %s (age=%ds, band %.0f-%.0f Hz)",
             callsign.c_str(), age_s, entry.negotiated.low_hz, entry.negotiated.high_hz);
    return true;
}

} // namespace iris
