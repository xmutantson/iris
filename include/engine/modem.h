#ifndef IRIS_MODEM_H
#define IRIS_MODEM_H

#include "config/config.h"
#include "engine/speed_level.h"
#include "engine/gearshift.h"
#include "engine/snr.h"
#include "native/phy.h"
#include "native/frame.h"
#include "native/xid.h"
#include "native/upconvert.h"
#include "native/channel_eq.h"
#include "arq/arq.h"
#include "ax25/ax25_session.h"
#include "ax25/digipeater.h"
#include "compress/compress.h"
#include "crypto/crypto.h"
#include "b2f/b2f_handler.h"
#include "ax25/hdlc.h"
#include "ax25/fx25.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
#include "kiss/kiss_server.h"
#include "radio/rigctl.h"
#include "probe/passband_probe.h"
#include "probe/probe_controller.h"
#include "ofdm/ofdm_config.h"
#include "mfsk/mfsk_ack.h"
#include "v2/transfer_ledger.h"
#include "v2/close_transport.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"
#include <vector>
#include <array>
#include <mutex>
#include <queue>
#include <complex>
#include <memory>
#include <atomic>
#include <unordered_map>
#include <deque>

namespace iris {

class AcceptanceArqHarness;

// Modem operating state
enum class ModemState {
    IDLE,           // Waiting
    RX_AX25,        // Receiving AX.25 (AFSK/GFSK)
    RX_NATIVE,      // Receiving Iris native
    TX_AX25,        // Transmitting AX.25
    TX_NATIVE,      // Transmitting Iris native
    CALIBRATING,    // Auto level calibration in progress
};

// TX-queue element with a local (never-on-wire) provenance flag.
//
// The MFSK tone-ACK REPLACES an entire OFDM-KISS TX batch with a short
// non-coherent tone burst.  It is ONLY safe to do this for the modem's OWN
// autonomous RR ACK — a frame that carries nothing but N(R)+PF, which the tone
// can fully represent.  A batch that also contains a U-frame (UA/DISC/DM/SABM/
// FRMR), RNR/REJ, an I-frame, a B2F_DATA proxy chunk, or a client/passthrough
// frame MUST NOT be swallowed: those carry state or payload the tone cannot
// convey and there is NO retransmit path once the batch is discarded (silent
// data loss — the ship-blocker).
//
// Frame *content* is ambiguous: a compressed B2F chunk's byte-14 can alias an
// RR control byte, and every U-frame passes the old (byte14 & 0x01) "S-frame"
// test.  So eligibility is decided by PROVENANCE, set at the single producer
// that emits the autonomous RR (the session send_frame_ callback), not by
// re-deriving type from bytes.  Every other producer leaves it false.
struct TxFrame {
    std::vector<uint8_t> data;
    bool tone_ack_eligible = false;   // true ONLY for the modem's own autonomous RR
    TxFrame() = default;
    explicit TxFrame(std::vector<uint8_t> d, bool eligible = false)
        : data(std::move(d)), tone_ack_eligible(eligible) {}
};

// Provenance predicate applied ONLY at the session send_frame_ producer.
// True iff the frame is a bare autonomous RR: 15 bytes (7+7 addr + 1 ctrl) and
// control low-nibble == 0x01.  RR ctrl = 0x01|(0<<2)|(nr<<5) so the low nibble
// is 0x01 (N(R) in bits 5-7 and PF in bit 4 lie outside the nibble); RNR
// (0x05), REJ (0x09), every U-frame (UA 0x63/DISC 0x43/DM 0x0F/SABM 0x2F/FRMR
// 0x87) and every I-frame (bit0==0) all fail this test.  Applied only to
// session frames, so arbitrary payload never reaches it.
//
// SCOPE (item 7): matches ANY session-generated RR, INCLUDING P=1 T1 polls —
// the poll/final bit is bit 4, which lies OUTSIDE the low nibble tested here, so
// `RR P=1` (ctrl 0x11|(nr<<5)) still has low-nibble 0x01 and returns true. The
// tone_ack_eligible tag it sets therefore covers the modem's autonomous ACKs AND
// its own stale T1 polls (both are drained by tx_frame_is_stale_poll); it does
// NOT and MUST NOT cover client/B2F/U/S(REJ,RNR) frames.
// NOTE (endpoint-via): the fixed 15-byte no-via shape is deliberate.  A
// session RR built with a digipeater via path is 15+7n bytes and returns
// false here — correctly so: the MFSK tone ACK is a native-PHY (Iris<->Iris
// DIRECT) substitution, while a digipeated session runs over AFSK where the
// RR must fly as real AX.25 bytes for the digi to repeat.
inline bool tx_is_autonomous_rr(const uint8_t* data, size_t len) {
    return len == 15 && (data[14] & 0x0F) == 0x01;
}

// #1 reverse-ACK ROOT FIX helper — rewrite an AX.25 S-/I-frame's N(R) (control
// byte bits 5-7) to `live_vr` IN PLACE, returning the pre-rewrite N(R) via
// `old_nr_out` and true iff the frame carried an N(R) field.  U-frames (control
// bits 1-0 == 11 — SABM/UA/DISC/DM/FRMR) carry no N(R) and are left untouched
// (returns false).  Pure bit-manipulation (no logging) so it is unit-testable;
// modem.cc's restamp_live_nr() wraps it with the diagnostic.  Invariant: no frame crosses the air with an N(R)
// older than the session V(R) at the TX instant.
inline bool ax25_restamp_nr(std::vector<uint8_t>& frame, uint8_t live_vr,
                            uint8_t* old_nr_out = nullptr, bool extended = false) {
    if (frame.size() < 15) return false;
    uint8_t ctrl = frame[14];
    if ((ctrl & 0x03) == 0x03) return false;      // U-frame: no N(R) field
    if (extended) {
        // AX.25 2.2 modulo-128: N(R) lives in octet-2 (bits 1-7), P/F in bit 0.
        // octet-1 (N(S) for I / SS for S) MUST NOT be touched — rewriting its bits
        // 5-7 (as the mod-8 path does) would corrupt a 7-bit N(S).
        if (frame.size() < 16) return false;
        uint8_t c2 = frame[15];
        uint8_t old_nr = (uint8_t)((c2 >> 1) & 0x7F);
        if (old_nr_out) *old_nr_out = old_nr;
        frame[15] = (uint8_t)((c2 & 0x01) | ((uint8_t)(live_vr & 0x7F) << 1));
        return true;
    }
    live_vr &= 0x07;
    uint8_t old_nr = (uint8_t)((ctrl >> 5) & 0x07);
    if (old_nr_out) *old_nr_out = old_nr;
    frame[14] = (uint8_t)((ctrl & 0x1F) | (live_vr << 5));
    return true;
}

// A batch is MFSK-tone-ACK-eligible iff it is non-empty and EVERY frame was
// tagged tone_ack_eligible at push time (i.e. every frame is an autonomous RR).
inline bool tx_batch_tone_ack_eligible(const std::vector<TxFrame>& batch) {
    if (batch.empty()) return false;
    for (const auto& f : batch)
        if (!f.tone_ack_eligible) return false;
    return true;
}

// Stale autonomous-RR-poll predicate for the post-ACK drain (modem.cc:1196).
// When a decoded peer S-frame ACKs our outstanding data, any of the modem's
// OWN autonomous RR polls still sitting in tx_queue_ are now stale (T1 may have
// queued an RR P=1 poll while we waited for the ACK) and may be dropped to save
// ~544 ms of airtime.  PROVENANCE, not content: keyed on tone_ack_eligible (set
// true ONLY at the session send_frame_ producer, modem.cc:590, for a bare
// autonomous RR).  An aliasing B2F_DATA chunk (byte-14 low bits == 01), a queued
// REJ/RNR, and every client/U-frame return false here and SURVIVE the drain --
// closing the byte-14 content-alias silent-data-loss hole the old
// `(data[14] & 0x03) == 0x01` test opened.
inline bool tx_frame_is_stale_poll(const TxFrame& f) {
    return f.tone_ack_eligible;
}

// 1-CW size-collapse decision for OFDM-KISS TX (modem.cc:3209).  CAPACITY-based
// and CONTENT-AGNOSTIC: a batch whose assembled payload fits inside a single
// LDPC codeword is sent as a 1-CW OFDM frame (saves ~750 ms/frame at O3 vs the
// 4-CW default).  This is purely a sizing call -- it can never swallow or drop
// a frame -- so it is applied to REJ/RNR/UA and small B2F/client batches, NOT
// just the modem's own RR ACKs.  It is DISTINCT from the tone-REPLACEMENT
// provenance gate (tx_batch_tone_ack_eligible): the Wave-1 change conflated the
// two, leaving non-RR NACKs multi-CW where CMD's 1-CW expect-ack gate
// (modem.cc:1726) truncates them -> immediate-REJ forward recovery degraded to
// a T1.  Returns the codeword count to use (1 if it fits, else cur_cw).
inline int ofdm_collapse_cw(size_t total_bytes, int one_cw_cap_bytes, int cur_cw) {
    if (cur_cw > 1 && (int)total_bytes <= one_cw_cap_bytes) return 1;
    return cur_cw;
}

// Calibration state machine
// Initiator: SEND_CMD → TX_TONE → WAIT_REPORT → DONE
// Responder (auto): RX_TONE → SEND_REPORT → IDLE
enum class CalState {
    IDLE,
    SEND_CMD,       // Queue CAL:START UI frame, wait for TX drain
    TX_TONE,        // Transmitting test tone
    WAIT_REPORT,    // Waiting for partner's level report
    RX_TONE,        // Measuring partner's test tone RMS
    SEND_REPORT,    // Queue CAL:RMS report, wait for TX drain
    DONE,
};

// Auto-tune state machine (bilateral gain calibration, half-duplex safe)
// Initiator: WAIT_READY → SEND_START → TX_TEST → WAIT_PEER → SEND_REPORT → WAIT_REPORT → APPLY → DONE
// Responder: WAIT_PEER → SEND_START → TX_TEST → SEND_REPORT → WAIT_REPORT → APPLY → DONE
// Initiator sends TUNE:START, waits for TUNE:READY before transmitting ramp.
// Reports are sent AFTER all TX is complete — both sides are listening when reports arrive.
enum class TuneState {
    IDLE,
    WAIT_READY,         // Initiator: sent TUNE:START, waiting for TUNE:READY from responder
    SEND_START,         // Queue test frames, wait for TX drain
    TX_TEST,            // Transmit test frame(s)
    WAIT_PEER,          // Wait to decode peer's test frame(s)
    SEND_REPORT,        // Send measurement report
    WAIT_REPORT,        // Wait for peer's report
    APPLY,              // Apply gain corrections
    DONE,
};

// Diagnostics snapshot for GUI
struct ModemDiag {
    ModemState state;
    int speed_level;
    int ofdm_speed_level = 0;  // O0-O9 OFDM level
    bool ofdm_active = false;  // True when OFDM PHY is active
    float snr_db;
    float agc_gain;
    float tx_level;
    float native_rx_gain;
    int kiss_clients;
    int frames_rx;
    int frames_tx;
    int crc_errors;
    int retransmits;
    float rx_rms;
    float rx_peak = 0;           // Peak sample value (for OVL indicator)
    bool ptt_active;
    CalState cal_state;
    float cal_measured_rms;
    ArqState arq_state;
    ArqRole arq_role;
    Ax25SessionState ax25_state = Ax25SessionState::DISCONNECTED;
    std::vector<std::complex<float>> constellation;  // Last received symbols
    std::vector<float> spectrum;   // Power spectrum for waterfall
    KalmanTrace kalman_trace;      // Kalman filter state trajectory for 3D view

    // Passband probe results
    ProbeState probe_state = ProbeState::IDLE;
    ProbeResult probe_my_tx;       // What they heard from us (A→B)
    ProbeResult probe_their_tx;    // What we heard from them (B→A)
    NegotiatedPassband probe_negotiated;
    bool probe_has_results = false;

    // Extended diagnostics for GUI status bar
    bool native_mode = false;
    uint64_t bytes_rx = 0;         // Cumulative bytes received
    uint64_t bytes_tx = 0;         // Cumulative bytes transmitted
    int phy_bps = 0;               // Current PHY bitrate
    int app_bps = 0;               // Application-level bitrate
    float compression_ratio = 0;   // 0 = off, >1 = active
    int encryption_state = 0;      // 0=off, 1=kx, 2=encrypted, 3=psk_mismatch

    // DCD (carrier detect)
    bool dcd_busy = false;
    float rx_raw_rms = 0;          // Pre-AGC RMS for DCD tuning
    float dcd_tone_energy = 0;     // AFSK tone correlation energy for tone-based DCD

    // Negotiated band info (from probe or config)
    float band_low_hz = 300.0f;
    float band_high_hz = 3500.0f;
    int baud_rate = 2400;

    // Waterfall spectrum range (may differ from operating band)
    float spectrum_low_hz = 0;
    float spectrum_high_hz = 4000.0f;
};

class Modem {
public:
    Modem();
    ~Modem();

    // Initialize with config
    bool init(const IrisConfig& config);
    void shutdown();

    // Main processing (call from audio callback or main loop)
    void process_rx(const float* rx_audio, int frame_count);
    void process_tx(float* tx_audio, int frame_count);

    // Queue a frame for transmission (called by KISS server)
    void queue_tx_frame(const uint8_t* frame, size_t len);

    // --- test hooks (in-process --test only) ---
    // Drive the A2 TX level+ncw resolution directly: seed a STALE ofdm_speed_level_
    // and a gearshift level (cap), then run the production resolve_ofdm_tx_level().
    // Returns the codeword count the frame would be modulated with; the resolved
    // (modulation) level is read via test_ofdm_tx_level(). Fail-before/pass-after
    // for the stale-ncw coherence bug (data-flow-level-vars.md §3/§7).
    int test_resolve_ofdm_tx_level(int stale_level, int gearshift_level, int max_ofdm,
                                   bool kiss_control_batch, bool batch_is_control,
                                   size_t total_bytes);
    int test_ofdm_tx_level() const { return ofdm_speed_level_; }
    // Drive the burst-frame reject-requeue path (leg 2, test (d)) directly:
    // set up a minimal OFDM TX config, force the modulated level LOW (small
    // capacity) while the confirmed anchor sits HIGHER, push `frames` into
    // tx_queue_, and attempt ONE burst-frame build (which rejects the
    // over-capacity frame). Returns tx_queue_ size after — a re-queued frame
    // keeps the size, a destroyed frame shrinks it. Fail-before/pass-after for
    // the burst-loop silent frame destruction (data-flow-tx-queue.md §6).
    size_t test_burst_reject_requeue(int forced_level, int anchor_level,
                                     const std::vector<std::vector<uint8_t>>& frames);

    // Drive the connect-recovery go-back-N burst against the 32-cap tx_queue_
    // through the SAME coalescing producer (enqueue_native_tx_frame) the native
    // session uses: seed a never-sent distinct in-flight frame (the ics213
    // stand-in), then re-emit a small outstanding window many times (the storm).
    // Returns true iff the never-sent frame SURVIVED.  Pre-fix (drop-oldest, no
    // coalesce) the dup burst evicts it -> false; post-fix -> true.  The invariant:
    // a reliable-ARQ tx queue never silently drops UNSENT data (P0 turnaround / C1).
    bool test_tx_queue_no_evict_under_dup_burst();

    // Dynamic per-level OFDM MAX_INFO (leg 3, test (c)): the single-sourced table
    // the five pins + the anchor-advance re-calls all read. Proves MAX_INFO
    // TRACKS the level (268 @ O2, 556 @ O5, ...) instead of a fixed floor pin, and
    // frames to exactly the level capacity (Fix A, DATALINK_TAX_DIAGNOSIS.md).
    static int test_ofdm_max_info_for_level(int level);

    // Slot-coalescing bounds for ONE on-air slot (pure; public for unit tests).
    // Defaults (probe_frames < 1, probe_airtime <= 0) reproduce the pre-probe
    // burst bounds EXACTLY: {BURST_FRAMES_DEFAULT_MAX=8, batch_airtime_s}.
    // With the frame knob set, the airtime bound becomes the 15 s hard cap
    // (deterministic frame-count-controlled slots), overridable LOWER via
    // probe_airtime; the frame knob clamps to BURST_FRAMES_PROBE_MAX=64.
    static void resolve_slot_bounds(int probe_frames, float probe_airtime_s,
                                    float batch_airtime_s,
                                    int& max_frames, float& airtime_bound_s);

    // In-process coalesced-slot mid-loss recovery regression (the probe's
    // retransmit-economics guard): build an n_frames slot through the SAME
    // resolve_slot_bounds + append_coalesced_slot + append_ofdm_burst_frame
    // drain the live packer uses, zero out frame kill_idx's audio (the
    // mid-slot loss), feed the whole slot to the LIVE OFDM RX path, and assert
    // (a) the frame-count knob bounds the slot (excess stays in tx_queue_),
    // (b) every OTHER frame still decodes byte-exact — a mid-slot loss costs
    // ONE frame, never the slot (no all-or-nothing slot), and (c) the lost
    // frame decodes when rebuilt and sent ALONE (individually retransmittable).
    bool test_coalesced_slot_midloss_recovery(int n_frames, int kill_idx);

    // Test entry for the single-sourced OFDM TX level-cap pass (file-static
    // ofdm_apply_tx_level_caps in modem.cc) — exercises the default-inert
    // fixed-gear measurement cap (env IRIS_TX_LEVEL_CAP).
    static int apply_tx_level_caps_probe(int base, bool kiss_tx, bool no_ack,
                                         int tx_acked, int tx_proposed,
                                         int peer_snr_level, int leap_max);

    // ---- Terminate/re-pack framing primitives (pure; public for unit tests) ---
    // pack: append a [len_be16][INFO] record.  split: pop every COMPLETE record
    // from buf into out (advances buf); returns -1 LOUD if a length >
    // REPACK_MAX_RECORD (stream misaligned) — never a bogus record, else 0.
    static void repack_pack_record(std::vector<uint8_t>& stream,
                                   const uint8_t* info, size_t len);
    static int repack_split_records(std::vector<uint8_t>& buf,
                                    std::queue<std::vector<uint8_t>>& out);
    // In-process round-trip integrity test (pack -> fragment at varying sizes ->
    // split -> assert bit-exact; + mid-stream size change; + bad-length LOUD).
    bool test_repack_pack_split_roundtrip();
    // In-process TERMINATE x DEMOTE re-fragment test (crossover recovery): drive an
    // owned session with large I-frames, then RECOVER the un-ACKed stream and assert
    // it re-slices bit-exact at a smaller MTU with ZERO byte loss (fail-before: the
    // drop_oversized_in_window path loses those bytes; pass-after: recovered exact).
    bool test_repack_refragment_on_demote();
    // In-process anchor-futility test (data-flow-tx-anchor.md §4): seed the anchor,
    // drive repeated no-ACK data resolves that bottom AT the anchor, and assert the
    // anchor demotes after OFDM_ANCHOR_FUTILITY_LIMIT futile rounds with MAX_INFO
    // shrunk and the window-mutation deferred (fail-before: the anchor is a
    // permanent floor — 53-84 clamped demotes per session, terminal disconnect).
    bool test_anchor_futility_demote();
    // In-process Chase store/flush test (data-flow-tx-anchor.md §5 D1/D2).
    bool test_chase_multiblock_and_flush();
    // In-process shadow-desync prevention test (data-flow-owned-shadow-seq.md §7):
    // replay the captured owned-sequence crossover — wide window, an in-flight span
    // wider than the post-demote K, the honest cumulative RR racing the deferred
    // re-fragment — and assert (A) the ACK is ACCEPTED despite the K-shrink
    // (INV-SEQ-1), (B) the re-fragment HOLDS until the ACK state resolves so V(S)
    // never rolls back across peer-counted numbers (INV-SEQ-2, no FRMR), and
    // (C) futility rounds under an un-drained wide flight do not demote the anchor
    // (INV-SEQ-3).  Fail-before: the RR is silently refused at fwd_dist > K, the
    // rollback fires, and every later honest N(R) reads invalid -> desync teardown.
    bool test_shadow_desync_prevention();
    // In-process peer-REJ deferral test (DATALINK_INTEGRITY_AUDIT §4/F2): a peer REJ
    // arriving mid-flight at wide K must DEFER the window mutation (INV-SEQ-2 quiesce)
    // — the anchor + MAX_INFO shrink inline, but V(S) must NOT roll back across peer-
    // counted seq numbers.  Fail-before (inline demote): dispatch_rx_frame(REJ) rolls
    // V(S) back and forms the crossover.  Also drives the receiver-side re-slice
    // OVERWRITE (leg b): a stale old-slicing reorder entry must lose to its re-sliced
    // replacement, else the drained stream is spliced (custody teardown / corruption).
    bool test_rej_inflight_deferred();
    // In-process F1 custody-teardown test (DATALINK_INTEGRITY_AUDIT §4/F1): an
    // unrecoverable N(R) desync in native/re-pack mode must end the session cleanly
    // (immediate custody teardown: DISC the pump + surface undelivered bytes,
    // DISCONNECTED) instead of the dead FRMR+SABM re-establish that zombies for ~28 s
    // and loses custody.  Fail-before (handle_native_desync -> false): the session
    // lands in AWAITING_CONNECTION with no pump DISC.
    bool test_native_desync_custody_teardown();

    // Digipeater dispatch wiring (in-process --test): configure the digipeater
    // directly, feed a received frame through the REAL dispatch_rx_frame, and
    // return how many frames landed in ax25_tx_queue_ (the AFSK TX path).
    // Proves the RX->digipeat->TX-queue hook and the config gate (enabled=false
    // must leave the queue empty) without audio.
    size_t test_digipeat_dispatch(const DigipeatConfig& dcfg,
                                  const std::vector<uint8_t>& frame,
                                  std::vector<uint8_t>* out_frame = nullptr);

    // The native-migration eligibility predicate for ax25_tx_queue_ frames
    // (used at BOTH probe-activation migration sites).  Exposed because the
    // pre-fix byte-14 sniff ((frame[14] & 1) == 0) is a §3.1-class no-via
    // landmine: with a via path present, byte 14 is a shifted callsign char
    // (bit0 == 0 always), so EVERY via-carrying frame — a digipeat re-emit or
    // an endpoint-via session frame — aliased as an I-frame and was migrated
    // off the AFSK path into the native OFDM queue.  Fail-before/pass-after
    // locked in tests.cc (test_ax25_digipeater).
    static bool migrate_to_native_eligible(const std::vector<uint8_t>& frame);

    // Single-frame OFDM byte capacity at a level (the oversized-guard threshold).
    // Exposed so the frame-sizing invariant max_info + framing_overhead <= capacity
    // can be proven for EVERY level in tests.cc (Fix A, DATALINK_TAX_DIAGNOSIS.md).
    static int test_ofdm_capacity_bytes_for_level(int level);

    // Fix B decision gate (bounded oversize-reject recovery): given the consecutive
    // reject streak and a stuck (wrapped) frame size, returns the min modulated level
    // that carries it once the streak crosses the limit, else -1. Proven fail-before
    // (streak below limit -> -1, spins) / pass-after (streak >= limit -> a fitting
    // level, gets on air) in-process without a two-stack session.
    static int test_ofdm_oversize_fit_floor(int reject_streak, int stuck_frame_bytes);

    // Burst-MAC in-window continuation DECISION (leg 2, test (a)): does the
    // initiator send the next burst (fill the window) or stop-and-wait for the
    // ACK? Exposes the pure gate predicate so the pacing kill is proven
    // fail-before (mechanism off / window full → stop) / pass-after (on + room →
    // continue) in-process without a two-stack OFDM audio session.
    static bool test_burst_fill_continue(bool enabled, bool ofdm_kiss_tx,
                                         bool session_active, bool we_initiated,
                                         bool have_queued_data, int window_used,
                                         int window_k);

    // RX package, ROOT-2 (RX-side): retain-vs-consume decision after a failed
    // OFDM decode. Returns true when the SHORT 1-CW expect-ack gate truncated a
    // longer multi-CW reverse frame and the buffer must be RETAINED (not consumed)
    // so the expanded gate can re-buffer + decode the full frame next pass. Proves
    // fail-before (HEAD always consumes → truncated reverse frame lost, a T1) /
    // pass-after (retain on short-gate truncation, consume otherwise) in-process.
    static bool test_ofdm_root2_retain(bool short_ack_gate_used, bool decode_success,
                                       bool llrs_nonempty);

    // RX tone-map latch (data-flow-rx-tonemap.md). test_ofdm_gate_n_cw: the
    // frame-length gate's codeword count — in KISS mode floored at the FULL
    // data-frame shape for the current RX level, so a stale/poisoned persistent
    // map can never under-buffer (truncate) a data frame.
    // test_ofdm_sweep_skip_level: the blind-detect dedupe keyed on
    // (level, shape) — the current level is skipped only when the full
    // data-frame shape was already tried, so a 1-CW-poisoned map cannot make
    // the true configuration untriable. test_rx_tonemap_latch drives the LIVE
    // process_rx_native() path end to end (real frames, in-process):
    // false = the poll-then-data poison sequence, true = the forced poisoned
    // wedge state.
    static int test_ofdm_gate_n_cw(bool kiss_tx, int map_ncw, int rx_level);
    static bool test_ofdm_sweep_skip_level(int lvl, int rx_level, int persisted_ncw);
    bool test_rx_tonemap_latch(bool force_poisoned_state);

    // Get diagnostics for GUI
    ModemDiag get_diagnostics() const;

    // Auto level calibration
    void start_calibration();
    bool is_calibrating() const { return state_ == ModemState::CALIBRATING; }

    // Standalone passband probe (debug — TX tones, listen, analyze)
    void start_probe();

    // Auto-tune: bilateral native-frame gain calibration
    // Each side sends a test frame, the other measures channel_gain from preamble,
    // reports back. Both adjust TX level so peer receives at gain ≈ 1.0.
    void start_autotune(const std::string& remote_callsign);

    // ARQ session control (native Iris protocol)
    void arq_connect(const std::string& remote_callsign);
    void arq_disconnect();
    void arq_listen();
    ArqState arq_state() const { return arq_.state(); }

    // AX.25 connected mode (standard protocol, interop with any TNC)
    void ax25_connect(const std::string& remote_callsign);
    void ax25_disconnect();
    void send_connected_data(const uint8_t* data, size_t len);
    Ax25SessionState ax25_state() const { return ax25_session_.state(); }
    int ax25_pending_frames() const { return ax25_session_.pending_frames(); }
    const std::string& ax25_remote_callsign() const { return ax25_session_.remote_callsign(); }
    void set_kiss_passthrough(bool v) {
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        ax25_session_.set_kiss_passthrough(v);
    }
    void set_txdelay_ms(int ms) {
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        ax25_session_.set_txdelay_ms(ms);
    }

    // Periodic tick for ARQ timeouts (call from main loop ~50ms)
    void tick();

    // PTT control
    void set_ptt_controller(std::unique_ptr<PttController> ptt) { ptt_ = std::move(ptt); }
    void set_tx_drain_hooks(std::function<void()> mark, std::function<bool()> done) {
        tx_drain_mark_ = mark; tx_drain_done_ = done;
    }

    // Accessors
    ModemState state() const { return state_; }
    const IrisConfig& config() const { return config_; }
    void set_loopback_mode(bool v) { loopback_mode_ = v; }
    void force_speed_level(int level) { gearshift_.lock_level(level); }

    void set_rx_callback(std::function<void(const uint8_t*, size_t)> cb) {
        rx_callback_ = cb;
    }

    using TransferResultPtr = std::shared_ptr<const v2::TransferResult>;
    void set_transfer_result_callback(
        std::function<void(TransferResultPtr)> cb) {
        transfer_result_callback_ = std::move(cb);
    }
    TransferResultPtr last_transfer_result() const {
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        return repack_last_transfer_result_;
    }
    std::vector<TransferResultPtr> retained_transfer_results() const;
    bool dispose_transfer_result(const v2::TransferIdentity& transfer);
    std::shared_ptr<const ArqTransferResult> last_arq_transfer_result() const;
    bool dispose_arq_transfer_result(uint64_t transfer_id);
    // RC2's protected close receive boundary is the only intended caller.  A
    // conventional AX.25 UA cannot enter this API or mint this move-only type.
    bool accept_remote_close_proof(v2::MatchingCloseProof&& proof);
    void notify_local_client_count(int count);

    // GUI event log callback (frame events, not debug noise)
    void set_gui_log(std::function<void(const std::string&)> cb) { gui_log_ = cb; }

    // Packet log callback (is_tx, protocol, description)
    void set_packet_log(std::function<void(bool, const std::string&, const std::string&)> cb) {
        packet_log_ = cb;
    }


    // Callback when ARQ state changes (for AGW notifications)
    void set_state_callback(std::function<void(ArqState, const std::string&)> cb) {
        state_callback_ = cb;
    }

    // Callback when AX.25 session state changes
    void set_ax25_state_callback(std::function<void(Ax25SessionState, const std::string&)> cb) {
        ax25_state_callback_ = cb;
    }

    // ARQ accessors for AGW flow control
    const std::string& arq_remote_callsign() const { return arq_.remote_callsign(); }
    int arq_pending_frames() const { return arq_.pending_frames(); }

private:
    // Acceptance tests need to inject already-decoded air frames and observe the
    // two real egress seams (air queue and local KISS callback) without replacing
    // any ARQ/custody production logic.
    friend class AcceptanceArqHarness;
    friend class v2::CloseTransportOwner;
    friend class v2::TransportReceiveOwner;

    // Precook (IRIS_STALL_PRECOOK_AUDIT.md §4): build/size all persistent DSP
    // state (FFT plans, LDPC decode scratch) up front, at the END of init() —
    // which returns (main.cc) before the capture thread is created or started —
    // so the audio callback thread never builds a plan or allocates decode
    // scratch under modem_mutex_. Minimal + idempotent.
    void precook();

    void process_rx_ax25(const float* audio, int count);
    void process_rx_native(const float* audio, int count);
    bool receive_native_fragment(const uint8_t* data, size_t len, bool end,
                                 ArqRecordType type = ArqRecordType::Data,
                                 uint64_t record_id = 0);
    bool accept_native_record(const uint8_t* data, size_t len,
                              ArqRecordType type = ArqRecordType::Data,
                              uint64_t record_id = 0);
    void fail_native_transform(const char* operation,
                               ArqTransferResultReason reason =
                                   ArqTransferResultReason::TransformFailure);
    void fail_strict_key_exchange(const char* operation);
    bool resolve_strict_receive_custody(const char* operation);
    void fail_ofdm_transform(const char* operation,
                             const std::vector<std::vector<uint8_t>>& originals = {});
    void dispatch_rx_frame(const std::vector<uint8_t>& frame, bool from_fx25 = false, bool from_ofdm = false);
    // A4 — reset EVERY OFDM level-owning var atomically (single-sourced across the
    // four disjoint reset sites: disconnect, OFDM-prepare, force-ofdm, cached-probe).
    void reset_level_state();
    // A2 — single-source the OFDM TX speed level and its codeword count. Called
    // once per OFDM-KISS TX after the batch is assembled + classified; re-latches
    // the level from the gearshift (V7), applies the caps in canonical order, then
    // derives ncw AND the tone map from that ONE final level. Returns ncw.
    int resolve_ofdm_tx_level(bool kiss_control_batch, bool batch_is_control,
                              size_t total_bytes);
    // A3 — max OFDM level among frames newly ACKed in (prev_va, nr], bounded to the
    // outstanding window so a bogus/duplicate N(R) cannot over-credit; clears those
    // ring slots. Returns -1 if nothing new was acked. Consumed by the three anchor-
    // advance paths (RR N(R), I-frame piggyback N(R), MFSK super-ACK adopt).
    int ring_acked_and_clear(uint8_t prev_va, uint8_t nr);
    // Leg 2 (burst-MAC) — re-queue frames whose OFDM build was rejected so they
    // survive (dynamic-MAX_INFO co-dependency safety net) instead of being
    // silently destroyed. A frame that still fits the CONFIRMED anchor
    // (tx_acked_level_) is recoverable — re-queued to the FRONT, provenance
    // preserved; a frame too big for even the anchor can never be reliably
    // carried and is dropped LOUD (a diagnostic, never a silent loss) so the
    // queue keeps moving (no head-of-line deadlock).
    void requeue_rejected_frames(std::vector<TxFrame>& taken);
    // THE native AX.25 session send_frame_ producer for tx_queue_ (modem.cc ~:718).
    // Coalesces a byte-identical frame already queued so a connect-recovery
    // go-back-N burst (which re-emits the same outstanding I-frames repeatedly)
    // cannot overflow the 32-cap queue and drop-oldest silently evict a never-sent
    // frame (P0 turnaround drop; same mechanism as the fixed C1 storm).
    void enqueue_native_tx_frame(std::vector<uint8_t>&& frame, bool tone_ack_eligible);
    bool tx_queue_has_identical(const uint8_t* data, size_t len) const;

    // ---- Terminate/re-pack the native-OFDM AX.25 data plane (keystone) ------
    // Gate: OFDM is the active transport for a CONNECTED KISS session.  When
    // false EVERY re-pack hook is inert and the legacy verbatim shadow path runs
    // byte-identical (AFSK / pure-KISS control arm).
    bool native_repack_active() const;
    // Ax25Session desync hook (DATALINK_INTEGRITY_AUDIT §4/F1). Called from
    // nr_error_recovery() the instant a desync is declared. Returns true iff we tore
    // custody down here (native/re-pack transport, where the AX.25 re-establish is a
    // dead end) — DISC the local pump + surface every undelivered custody byte — so
    // the session ends cleanly instead of a ~28 s SABM zombie ending in custody loss.
    // Returns false for AFSK / owned sessions so the standard re-establish runs.
    bool handle_native_desync();
    // TX (R1->R2): custody-ACK a client I-frame to the local pump, append its
    // INFO to the length-delimited TX stream, then drain.  Returns true if the
    // frame was terminated (caller must NOT fly it verbatim).
    bool repack_tx_ingest(const uint8_t* frame, size_t len);
    // Consume a local-pump S-frame (RR/RNR) for re-origination flow control
    // (advances repack_reorig_va_); returns true if consumed (do NOT fly it).
    bool repack_sframe_consume(const uint8_t* frame, size_t len);
    // Custody graceful close: hold a client DISC (ack it locally) while buffered
    // data is still air-draining; returns true if held (do NOT propagate yet).
    bool repack_hold_client_disc(const uint8_t* frame, size_t len);
    bool repack_complete_remote_close(const uint8_t* frame, size_t len);
    // Far side: has the re-origination fully delivered (no records/stream pending
    // and the R3 window is empty)?  Gates the far-side air-DISC propagation.
    bool repack_reorig_drained() const;
    // Drive the graceful-close state machine from tick() (propagate held DISCs
    // once the respective queue has drained; bounded far-side backstop).
    void repack_graceful_close_tick();
    // Drain the TX stream into <=max_info() R2 I-frames via send_data while the
    // owned R2 window has room.  flush_partial=true also emits a final short frame.
    void repack_tx_drain(bool flush_partial);
    // TERMINATE x DEMOTE re-fragment (crossover recovery, INV-3): tx_acked_level_ just
    // demoted, shrinking max_info below already-packed in-flight frames.  RECOVER the
    // un-ACKed stream bytes (from the owned window) back into the stream and re-drain
    // at the new (smaller) max_info instead of DROPPING the oversize frames (data loss
    // -> incomplete transfer).  The [len] records re-slice bit-exact by construction.
    void repack_refragment_on_demote();
    // Purge stale OUR-callsign I-frames (re-pack data already emitted at the OLD size)
    // from the modem tx_queue_ so they can neither fit-floor-spin nor fly a duplicate;
    // their bytes are back in the stream and re-emit fresh.  Control/S-frames stay.
    void repack_purge_native_data_frames();
    // Chase-store flush decision on a successful decode (data-flow-tx-anchor.md
    // §5 D1): flush ONLY when the success has the SAME LLR shape as the store —
    // the stored frame (or a same-shape sibling) landed, so the store is stale.
    // A different-shape success (a 20 B / 1-block control frame between
    // retransmissions of a failing multi-block I-frame) says nothing about the
    // stored frame and must NOT flush its accumulated copies.
    static bool chase_flush_on_success(size_t stored_llrs, size_t fresh_llrs) {
        return stored_llrs != 0 && stored_llrs == fresh_llrs;
    }
    // Lower the TX ACK anchor by one and shrink MAX_INFO — the single shared
    // demote path for BOTH triggers: a peer REJ/RNR (explicit "can't decode")
    // and anchor-futility (implicit: the no-ACK demote bottomed at the anchor
    // OFDM_ANCHOR_FUTILITY_LIMIT consecutive data rounds — the peer answers
    // polls but never advances N(R), which modem.cc's RR sniff already reads
    // as "alive but NOT decoding our data"). defer_window_mutation=true postpones
    // the re-fragment/drop to the next idle TX pump tick via ofdm_refrag_pending_
    // (see that member's note) — used by BOTH live triggers now: the TX-path
    // futility demote AND the RX-path peer REJ/RNR (DATALINK_INTEGRITY_AUDIT §4/F2,
    // whose sniff runs before on_frame_received so an inline harvest would roll V(S)
    // back across peer-counted numbers). false (inline) is left only for the
    // non-repack drop path and tests. The anchor + MAX_INFO shrink is INLINE either
    // way. Returns false at the O0 floor.
    bool demote_tx_anchor(const char* reason, bool defer_window_mutation);
    // Consume ofdm_refrag_pending_: recover + re-fragment (repack) or drop
    // oversized (non-repack) the un-ACKed window at the shrunken MAX_INFO.
    void consume_pending_window_refrag();
    // RX (R2->R3): append an in-order R2 INFO chunk to the reassembly stream,
    // split full [len]records into the pending deque (SOLE reassembly producer,
    // fed by ax25_session_ V(R) gate).  Set by set_native_stream_rx_callback.
    bool repack_rx_ingest(const uint8_t* info, size_t len);
    // Re-originate pending records to the local pump, one clean I-frame each,
    // window-paced to the pump's RX ahead-window (never fdist>3).
    void repack_rx_drain();
    void repack_note_air_ack(const uint8_t* data, size_t len);
    bool repack_admit_original(const Ax25Frame& frame);
    void repack_begin_transfer();
    void repack_begin_rx_transfer();
    bool finish_transfer_once(
        v2::TransferResultReason reason,
        std::optional<v2::MatchingCloseProof> proof = std::nullopt,
        std::shared_ptr<v2::LiveSession> captured_session = nullptr);
    bool repack_has_open_custody() const;
    void repack_update_rx_backpressure(bool announce = true);
    void repack_retry_r3_window();
    // Custody teardown: the air link died with custody-ACKed data undelivered ->
    // DISC the local pump so it never reports success on lost bytes.  Clears all
    // re-pack state.  Idempotent.
    void repack_custody_teardown(
        const char* why,
        v2::TransferResultReason reason = v2::TransferResultReason::PeerLost);
    // Reset all re-pack state at (re)connect.
    void repack_reset();
    // Clear all state whose ownership ends with an OFDM link/session.  RX mute
    // expiry is deliberately not a teardown and must never call this helper.
    void reset_ofdm_belief_and_chase();
    // Engage terminate ONCE when OFDM first becomes the transport: absorb any
    // client I-frames HELD verbatim during probe/tune into the stream (so nothing
    // verbatim flies over R2 and misaligns the far reassembler — the seam fix) and
    // roll the owned window back to V(A).  Idempotent via repack_activated_.
    void repack_maybe_engage();
    void repack_engage_at_activation();
    // Build ONE additional OFDM frame from tx_queue_ and append its audio to
    // tx_buffer_ (multi-frame burst, same PTT). Returns the frame's airtime in
    // seconds, or -1 if the queue is empty or the build was rejected (rejected
    // frames are re-queued via requeue_rejected_frames, never destroyed).
    float append_ofdm_burst_frame(size_t burst_max_payload);
    // The coalescing loop of one on-air slot: append additional OFDM burst
    // frames to tx_buffer_ until the slot bounds from resolve_slot_bounds()
    // (frame count AND airtime, probe knobs or the pre-probe defaults) stop it.
    // frame_airtime_s carries the main frame's airtime in and the whole slot's
    // airtime out.  Returns the TOTAL frame count of the slot (main + appended).
    // frame_bounds (test-only, may be null) records tx_buffer_.size() after
    // each appended frame so a test can locate per-frame sample spans.
    int append_coalesced_slot(float& frame_airtime_s, size_t burst_max_payload,
                              std::vector<size_t>* frame_bounds = nullptr);
    void process_calibration_rx(const float* audio, int count);
    void generate_cal_tone(float* audio, int count);
    void send_cal_ui(const char* payload);
    void send_probe_start_ui();
    void send_probe_ready_ui();
    void handle_cal_frame(const uint8_t* info, size_t len);

    void ptt_on();
    void ptt_off();

    // Compute power spectrum for waterfall display
    void compute_spectrum(const float* audio, int count);

    IrisConfig config_;
    // Original band/PHY from init — restored on disconnect after probe changes
    float orig_band_low_hz_ = 0;
    float orig_band_high_hz_ = 0;
    PhyConfig orig_phy_config_;
    std::atomic<ModemState> state_{ModemState::IDLE};
    bool loopback_mode_ = false;  // Skip TX mute when using internal loopback

    // AX.25 modems
    AfskModulator afsk_mod_;
    AfskDemodulator afsk_demod_;
    GfskModulator gfsk_mod_;
    GfskDemodulator gfsk_demod_;
    G3ruhScrambler g3ruh_tx_scrambler_;
    G3ruhScrambler g3ruh_rx_scrambler_;
    HdlcDecoder hdlc_decoder_;
    Fx25Decoder fx25_decoder_;    // FX.25 decoder (runs in parallel with HDLC)
    NrziDecoder nrzi_decoder_;
    std::vector<uint8_t> last_rx_frame_;  // Dedup: last dispatched frame content
    int dedup_cooldown_ = 0;              // Samples remaining for dedup window

    // Native PHY (legacy single-carrier)
    PhyConfig phy_config_;
    std::unique_ptr<NativeModulator> native_mod_;
    std::unique_ptr<NativeDemodulator> native_demod_;

    // OFDM PHY (default, high-throughput multi-carrier)
    bool ofdm_phy_active_ = false;                     // OFDM PHY in use (after probe)
    OfdmConfig ofdm_config_;                            // Current OFDM configuration
    std::unique_ptr<OfdmModulator> ofdm_mod_;           // OFDM modulator
    std::unique_ptr<OfdmDemodulator> ofdm_demod_;       // OFDM demodulator
    ToneMap ofdm_tone_map_;                             // TX tone map (from gearshift)
    ToneMap ofdm_rx_tone_map_;                          // RX tone map (from peer signaling / blind detect)
    int ofdm_speed_level_ = 0;                          // TX OFDM speed level (O0-O12)
    int ofdm_kiss_rx_level_ = 0;                        // Expected peer TX level (signaling / blind detect)
    // Coherent RX adoption (Mercury rank 7/9): the forward O-level THIS station
    // last proposed to the sender via the super-ACK. Because the receiver DRIVES
    // the rate, this is the level A is being commanded toward — so B uses it to
    // (1) keep the frame-length gate wide enough for A's higher/longer frame while
    // the climb is in flight (never truncate), and (2) try that level FIRST in
    // blind-detect. -1 = no outstanding proposal. Self-rejecting: the primary/
    // trial decode is still CRC-32 gated, so a stale target never mis-demaps.
    int ofdm_kiss_rx_proposed_level_ = -1;
    bool ofdm_kiss_rx_confirmed_ = false;               // True after peer speed byte received
    // #2 burst-epoch binding (reverse-ACK aliasing belt-and-braces).
    //   tx_burst_epoch_: this station's 3-bit forward burst counter, stamped into
    //     the Iris-owned MULTI_PAYLOAD wrapper of every native OFDM-KISS burst it
    //     emits (incremented once per burst).  A reverse ACK echoing a DIFFERENT
    //     epoch is a stale/buffered tone -> advisory only (no destructive V(A)
    //     advance); a match (or an ABSENT echo) permits the destructive ack_frames.
    //   ofdm_kiss_rx_burst_epoch_: the last forward burst epoch THIS station
    //     decoded (from the wrapper), which it echoes back in its MFSK tone ACK
    //     suffix + OFDM S-frame tail so the peer can bind the ACK. -1 = none yet.
    uint8_t tx_burst_epoch_ = 0;
    int ofdm_kiss_rx_burst_epoch_ = -1;
    bool ofdm_expect_ack_ = false;                      // True after CMD I-frame burst → expect 1-CW S-frame
    MfskAck mfsk_ack_;                                  // MFSK tone ACK TX/RX for OFDM-KISS
    int ofdm_txdelay_ms_ = 0;                           // Adaptive OFDM TXDELAY (0 = use config default)
    std::vector<std::complex<float>> ofdm_rx_iq_;       // OFDM RX working buffer (complex)
    OfdmSyncWorkspace ofdm_sync_workspace_;             // Per-receiver detector refs/scratch
    OfdmAcquisitionState ofdm_acquisition_;              // Absolute ordered acquisition work
    std::uint64_t ofdm_active_candidate_id_ = 0;
    std::vector<float> ofdm_rx_audio_buf_;              // OFDM RX raw audio buffer (bypass downconverter)
    std::uint64_t ofdm_rx_geometry_sample_bound_ = 0;
    int ofdm_rx_geometry_bound_nfft_ = 0;
    int ofdm_rx_geometry_bound_cp_ = 0;
    int ofdm_rx_geometry_bound_data_carriers_ = 0;
    int ofdm_rx_geometry_bound_pilot_rows_ = -1;
    int ofdm_rx_geometry_bound_sample_rate_ = 0;
    int ofdm_rx_geometry_bound_min_bits_ = -1;

    // 2nd-order Butterworth LPF on OFDM RX path — removes f² discriminator noise
    // from flat 9600-baud radio ports (raw discriminator output, no de-emphasis).
    // LPF only (no HPF): HPF group delay at lowest carriers exceeds CP.
    struct BiquadDF2 {
        float b0=1,b1=0,b2=0,a1=0,a2=0,z1=0,z2=0;
        float process(float x) {
            float w = x - a1*z1 - a2*z2;
            float y = b0*w + b1*z1 + b2*z2;
            z2 = z1; z1 = w;
            return y;
        }
        void reset() { z1 = z2 = 0; }
    };
    BiquadDF2 ofdm_rx_lpf_;
    bool ofdm_rx_lpf_active_ = false;                  // Set after probe configures cutoff

    // OFDM RX search/commit invariant: hypothesis search is side-effect-free.
    // Only a commit mutates persistent state; a commit happens at most once per
    // reception; everything it writes is keyed by an explicit reception
    // identity, owned by the link's committed CFO belief (in Hz), and expressed
    // in one coordinate system.
    std::vector<float> ofdm_chase_llrs_;                // Stored LLRs for OFDM Chase combining
    int ofdm_chase_combines_ = 0;                       // Chase combine counter
    std::uint64_t ofdm_chase_candidate_id_ = 0;
    std::uint64_t ofdm_chase_capture_epoch_ = 0;
    int ofdm_chase_level_ = -1;
    int ofdm_chase_n_codewords_ = 0;
    LdpcRate ofdm_chase_fec_rate_ = LdpcRate::RATE_1_2;
    float ofdm_cfo_committed_hz_ = 0.0f;

    // Owner evidence is search state until the bank reaches a verdict.  The
    // public HARQ holder below is filled only by the single commit step.
    OfdmDemodResult ofdm_search_owner_result_;
    std::uint64_t ofdm_search_owner_candidate_id_ = 0;
    std::uint64_t ofdm_search_owner_capture_epoch_ = 0;
    int ofdm_search_owner_level_ = -1;
    int ofdm_search_owner_n_codewords_ = 0;
    bool ofdm_search_owner_chase_attempted_ = false;
    bool ofdm_search_owner_valid_ = false;

    HarqDecodeResult ofdm_harq_evidence_;
    std::uint64_t ofdm_harq_evidence_candidate_id_ = 0;
    std::uint64_t ofdm_harq_evidence_capture_epoch_ = 0;
    bool ofdm_harq_evidence_valid_ = false;
    // Earliest exact shape extent missing anywhere in the current CFO-bank
    // pass.  The furthest extent remains retained by OfdmAcquisitionState,
    // but cannot be the next wake-up while a shorter shape becomes complete.
    std::uint64_t ofdm_bank_next_shape_samples_ = 0;
    OfdmSyncResult ofdm_pending_sync_;                   // Cached sync for "frame incomplete" retry
    bool ofdm_sync_cached_ = false;                      // Valid cached sync exists
    int ofdm_redetect_count_ = 0;                        // Re-detection counter for diagnostics
    std::uint64_t ofdm_pending_required_samples_ = 0;    // Input-relative end required before retry

    // Mode A upconversion
    Upconverter upconverter_;
    Downconverter downconverter_;
    bool use_upconvert_ = false;

    // Channel equalization (built from probe tone power measurements)
    ChannelEqualizer rx_channel_eq_;   // RX: flatten incoming channel
    ChannelEqualizer tx_channel_eq_;   // TX: pre-equalize outgoing signal

    // Engine
    Gearshift gearshift_;
    AGC agc_;
    float snr_db_ = 0;
    float snr_preamble_db_ = 0;  // Preamble-only SNR (for peer feedback)

    // Peer SNR feedback: what the remote side measures from our TX signal.
    // Used to cap TX speed on asymmetric links where local RX SNR ≠ peer RX SNR.
    // -1 = no peer SNR report received yet.
    float peer_snr_db_ = -1.0f;

    // ARQ session (native Iris protocol)
    ArqSession arq_;

    // AX.25 connected mode session (standard protocol)
    Ax25Session ax25_session_;

    // AFSK->AFSK digipeater role (config-gated, DEFAULT OFF — [Digipeat]).
    // Hooked in dispatch_rx_frame after the session declines the frame;
    // repeats into ax25_tx_queue_.  See ax25/digipeater.h for the Direwolf
    // provenance (cdigipeater.c / digipeater.c / dedupe.c).
    Digipeater digipeater_;

    // Compression (used when ARQ peers negotiate CAP_COMPRESSION)
    Compressor tx_compressor_;
    Compressor rx_compressor_;

    // Encryption (used when ARQ peers negotiate CAP_ENCRYPTION)
    CipherSuite cipher_;
    uint64_t tx_batch_counter_ = 0;
    uint64_t rx_batch_counter_ = 0;
    uint32_t crypto_direction_ = DIR_CMD_TO_RSP;

    // B2F unroll/reroll (used when ARQ peers negotiate CAP_B2F_UNROLL)
    B2fHandler b2f_handler_;

    // Passband probe controller
    ProbeController probe_;

    // Simulated bandpass filter (--bandpass, for testing)
    struct Biquad {
        float b0=1, b1=0, b2=0, a1=0, a2=0;
        float z1=0, z2=0;
        float process(float x) {
            float y = b0*x + z1;
            z1 = b1*x - a1*y + z2;
            z2 = b2*x - a2*y;
            return y;
        }
    };
    Biquad sim_bp_hi_[4], sim_bp_lo_[4];  // 4-stage HP + 4-stage LP = 8th order BP
    bool sim_bp_enabled_ = false;

    // Simulated FM de-emphasis filter (--deemphasis, for testing)
    // Standard 75µs (US) or 50µs (EU) first-order LPF: H(s) = 1/(1+sτ)
    // Produces ~6 dB/octave rolloff above corner frequency (2122 Hz for 75µs)
    Biquad sim_deemph_;
    bool sim_deemph_enabled_ = false;

    // PTT
    std::unique_ptr<PttController> ptt_;
    std::atomic<bool> ptt_active_{false};

    // TX/RX muting for half-duplex
    std::atomic<bool> rx_muted_{false};
    int rx_mute_holdoff_ = 0;  // Samples to remain muted after TX ends
    std::atomic<int> native_selfhear_guard_{0};  // Samples: discard native RX frames (self-hear from pipeline latency)

    // DCD (Data Carrier Detect) — defer TX while channel is busy
    std::atomic<int> dcd_holdoff_{0};        // samples remaining (0 = expired)
    int dcd_diag_ticks_ = 0;                 // DCD diagnostic logging counter

    // CSMA guard: defer TX after last frame decode to avoid stepping on response
    std::atomic<int> csma_holdoff_{0};       // samples remaining (0 = expired)

    // Auto-DCD: detect inverted-squelch radios where static is louder than signal.
    // Calibrates baseline noise level and detects polarity automatically.
    bool dcd_auto_ = false;          // auto-DCD enabled
    float dcd_baseline_rms_ = 0;     // measured noise floor RMS
    int dcd_baseline_samples_ = 0;   // samples collected for baseline
    bool dcd_baseline_done_ = false; // baseline measurement complete
    bool dcd_inverted_ = false;      // true = inverted squelch (signal < noise)


    // p-persistent CSMA (AX.25 2.2 Section 6.4.2)
    // After DCD clears + holdoff + csma_holdoff, enter slotted access:
    // each slottime period, generate random 0-255; if < persist then TX, else wait.
    int csma_slot_timer_ = 0;  // samples remaining in current slot

    // Thread safety: single recursive mutex protecting all shared mutable state.
    // Audio threads (capture delivery, playback producer) are FIFO-buffered from
    // real-time WASAPI threads, so brief blocking (~1-5ms) is safe.
    mutable std::recursive_mutex modem_mutex_;

    // TX queue
    std::queue<TxFrame> tx_queue_;
    std::queue<std::vector<uint8_t>> ax25_tx_queue_;  // forced AX.25 (XID replies)

    // ---- Terminate/re-pack the native-OFDM AX.25 data plane (keystone) ------
    // R1->R2 TX stream: length-delimited ([len_be16][INFO]) concatenation of
    // custody-ACKed client I-frames, drained into <=max_info() R2 I-frames.
    std::vector<uint8_t> native_repack_tx_stream_;
    // R2->R3 RX reassembly buffer + recovered-record FIFO (pending re-origination).
    std::vector<uint8_t> native_repack_rx_stream_;
    std::queue<std::vector<uint8_t>> native_repack_rx_records_;
    // R3 records remain owned after their scheduling queue entry is popped.
    // A slot is released only by the local client's cumulative RR/RNR/REJ.
    std::array<std::vector<uint8_t>, 8> repack_reorig_window_{};
    std::array<bool, 8> repack_reorig_window_active_{};
    std::array<v2::OriginalRecordId, 8> repack_reorig_window_record_id_{};
    // R1 client send-seq dedup: only the strictly-in-order NEW client I-frame is
    // appended to the stream; a go-back-N retransmit is re-ACKed (custody) but
    // never re-buffered (a duplicate record would corrupt the SHA stream).
    uint8_t repack_tx_client_ns_ = 0;
    // R3 (far Iris -> local pump) mod-8 window for re-originated records.
    uint8_t repack_reorig_vs_ = 0;   // next N(S) to emit to the local pump
    uint8_t repack_reorig_va_ = 0;   // last N(R) the local pump ACKed (its V(R))
    bool repack_reorig_peer_busy_ = false;
    // Custody backpressure hysteresis: withhold client custody-ACK (RNR) while the
    // TX stream is deep so the client cannot overrun the buffer.
    bool repack_custody_rnr_ = false;
    // Partial-flush idle counter (ticks since last TX ingest) — event/idle-driven,
    // NOT a fixed frame size; flushes the stream tail after the client goes quiet.
    int repack_tx_idle_ticks_ = 0;
    // Sticky: re-pack has engaged (custody taken) this session -> an unexpected
    // DISCONNECT owes the local pump a custody teardown.
    bool repack_engaged_ = false;
    // Latch: repack_engage_at_activation() has run this session (absorb + window
    // rollback happen exactly once, the instant OFDM becomes the transport).
    bool repack_activated_ = false;
    // TERMINATE x DEMOTE re-fragment (crossover recovery, default-ON).  On a
    // mid-transfer anchor demote that shrinks max_info below already-packed frames,
    // recover+re-fragment the un-ACKed stream instead of dropping it (INV-3).  Env
    // IRIS_REPACK_REFRAG=0 restores the pre-fix drop_oversized_in_window behavior for
    // the fail-before/pass-after A/B.  A member so one binary drives both arms.
    bool repack_refragment_ = true;
    // Peer callsign snapshot for custody teardown (remote_call_ is cleared by
    // reset() BEFORE the DISCONNECTED state callback fires).
    std::string repack_remote_call_;
    // Custody GRACEFUL CLOSE.  The client custody-completes FAST (local ACK) and
    // DISCONNECTs long before the slow OFDM air-delivery of the buffered stream
    // finishes.  Iris took custody, so it MUST finish delivery before tearing the
    // air link: hold the client DISC (ack it locally), keep draining, and only
    // propagate the air DISC once the TX stream is empty AND the owned window is
    // fully air-ACKed.  Far side symmetrically holds a received air DISC until its
    // re-origination queue has drained to the local pump.
    bool repack_client_disc_pending_ = false;   // near: client DISC held, draining
    enum class RepackTransferState {
        Idle,
        Open,
        Draining,
        AwaitingRemoteClose,
        Succeeded,
        Failed,
    };
    RepackTransferState repack_transfer_state_ = RepackTransferState::Idle;
    std::vector<uint8_t> repack_client_disc_frame_;
    v2::TransferResultReason repack_failure_reason_hint_ =
        v2::TransferResultReason::RetryExhausted;
    // The origin-side and receiver-side obligations are distinct directional
    // transfers.  Neither queue is itself an ownership record.
    std::unique_ptr<v2::TransferLedger> repack_transfer_ledger_;
    std::unique_ptr<v2::TransferLedger> repack_rx_transfer_ledger_;
    std::vector<uint8_t> repack_air_ack_stream_;
    size_t repack_air_ack_record_index_ = 0;
    size_t repack_air_ack_serialized_extent_ = 0;
    TransferResultPtr repack_last_transfer_result_;
    std::deque<TransferResultPtr> repack_retained_transfer_results_;
    std::function<void(TransferResultPtr)> transfer_result_callback_;
    bool repack_air_disc_pending_ = false;      // far: air DISC held, reorig draining
    bool repack_air_disc_delivered_ = false;    // far: DISC published to R3 client
    std::vector<uint8_t> repack_air_disc_frame_;
    int  repack_air_disc_hold_ticks_ = 0;       // far: bounded hold (pump-gone backstop)
    size_t repack_rx_serialized_bytes_ = 0;
    size_t repack_rx_schedule_record_index_ = 0;
    int repack_r3_no_progress_ticks_ = 0;
    int repack_r3_retry_ticks_ = 0;
    int repack_r3_retries_ = 0;
    int repack_local_client_count_ = 0;
    v2::ConnectionId repack_local_client_connection_id_{};
    bool repack_local_client_connection_valid_ = false;
    bool repack_terminal_requested_ = false;
    std::vector<float> probe_audio_pending_;          // probe tones waiting to TX
    std::vector<float> tx_buffer_;
    size_t tx_pos_ = 0;
    bool tx_draining_ = false;  // waiting for audio pipeline to flush before PTT release
    std::function<void()> tx_drain_mark_;     // snapshot current pipeline position
    std::function<bool()> tx_drain_done_;     // true when pipeline has flushed past mark

    // Leg 2 pacing kill — burst-MAC in-window continuation. DEFAULT-ON (proven).
    // When true, the initiator sends the NEXT burst without waiting out the
    // ACK-clocked stop-and-wait csma timer while the K=7 window still has room +
    // data is queued (see burst_fill_continue() in modem.cc). The half-duplex
    // RSP-RR collision that kept this OFF is closed by the COUPLED RSP autonomous-
    // RR HOLD-to-burst-end (Ax25Session::set_rr_hold, armed from this flag in
    // Modem::init) — the responder defers its RR until the initiator's burst
    // sequence goes quiet, so the reverse ACK rides the clear post-burst slot.
    // Fleet WGN:40 N=3 A/B (delivrate-attack): burst_fill ON delivered +2.53x
    // (steady-state 2523 -> 6372 B/min) with ZERO fatal disconnects (== the
    // burst-off baseline) and drove the native gearshift climb O2 -> O5 (16QAM).
    // Env IRIS_BURST_FILL still overrides for A/B; RR-hold tracks whatever value
    // wins so the two-sided change stays consistent.
    bool burst_fill_ = true;

    // WIDE WINDOW (terminate turnaround lever, default-ON).  The terminated OFDM tier
    // is Iris<->Iris and owns its own sequence space; the legacy mod-8/K=7 GBN window
    // makes the sender stop for a reverse ACK every 7 frames (= the ~30% steady
    // turnaround at O5).  Switch the OWNED R2 session to AX.25 2.2 modulo-128 with a
    // large window K so the sender streams many more full frames before it must stop
    // -> the per-window turnaround amortizes toward 0.  Applied at native activation
    // (repack_engage_at_activation, both ends flip on the same env).  Env
    // IRIS_WIDE_WINDOW=0 restores mod-8/K=7 (fail-before); IRIS_WIDE_WINDOW_K tunes K.
    bool wide_window_ = true;
    // Default 7 (== K_WINDOW): the widen to a large K is HELD OFF because a wide
    // window starves under the default burst-fill continuation (measured 4.5-5.2x
    // regression vs K=7, WGN:40 57/83-car n=16 paired; see repack_tx_drain).
    // IRIS_WIDE_WINDOW_K re-enables a larger K for the owed burst-fill-accumulate
    // follow-up.  Clamped to [7,127] in Ax25Session::set_wide_window.
    int  wide_window_k_ = 7;
    int  repack_window_k_logged_ = -1;   // last-logged widen target (fire-proof, per session)
    bool repack_widen_eligible_ = false; // last-logged widen-eligibility edge (fire-proof)

    // RX state
    std::atomic<bool> native_mode_{false};      // RX can decode native frames
    std::atomic<bool> native_tx_ready_{false};  // TX may use native PHY
    std::atomic<bool> peer_is_iris_{false};     // Remote confirmed as Iris via probe
    std::atomic<bool> ofdm_kiss_{false};        // OFDM RX enabled (probe complete, native demod active)
    std::atomic<bool> ofdm_kiss_tx_{false};     // OFDM TX enabled
    std::atomic<bool> ofdm_kiss_confirmed_{false}; // Heard native frame from peer (bidirectional)
    std::atomic<bool> ofdm_kiss_probing_{false}; // Probe in progress (suppress data TX)
    std::atomic<bool> ofdm_config_mismatch_{false}; // Peer config fingerprint disagreed -> AFSK fallback latch (item 5)

    // Adaptive batch airtime: grows on successful ACKs, shrinks on REJ/loss.
    // TCP-style AIMD: additive increase (+1s per RR), multiplicative decrease (halve on REJ).
    float batch_airtime_s_ = 6.0f;               // Current batch cap (seconds)
    static constexpr float BATCH_AIRTIME_MIN = 6.0f;   // Enough for 3-4 OFDM frames at O0
    static constexpr float BATCH_AIRTIME_MAX = 12.0f;  // Up to 7-8 frames at O0

    // SLOT-COALESCING PROBE (measurement instrument, default-inert).  The duty
    // model says delivered ~= net_rate x duty, and the measured steady state
    // keys ~one short frame per ACK round-trip; these knobs force the on-air
    // slot to coalesce more queued frames per PTT so the duty prediction is
    // measurable directly.  Env IRIS_FRAMES_PER_BURST=N (1..64) overrides the
    // default 8-frame burst cap (N=1 = strict one-frame stop-and-wait control
    // arm); env IRIS_SLOT_AIRTIME_S=S overrides the slot airtime bound.  Both
    // unset => resolve_slot_bounds() returns exactly the pre-probe bounds
    // (8 frames, adaptive batch_airtime_s_) — nothing changes unless set.
    // SLOT BOUND RATIONALE: the ~50-sample SFO-through-CP timing-slip budget
    // (cp(64) - FP_ADVANCE - jitter; ~52 s at 20 ppm) applies WITHIN one OFDM
    // frame only — every burst frame carries its own full preamble
    // (build_ofdm_frame TX chain: noise ramp + ZC training pair + sync word),
    // so RX re-anchors timing AND channel estimate per frame and drift never
    // accumulates across the slot.  The 15 s hard cap is therefore a
    // conservative whole-slot bound (PTT/thermal/half-duplex fairness), not a
    // sync limit; each frame in the slot stays independently decodable and
    // individually retransmittable (test_coalesced_slot_midloss_recovery).
    int   probe_frames_per_burst_ = -1;    // -1 = default (8-frame burst cap)
    float probe_slot_airtime_s_   = -1.0f; // -1 = default airtime bound
    static constexpr int   BURST_FRAMES_DEFAULT_MAX = 8;    // pre-probe MAX_BURST
    static constexpr int   BURST_FRAMES_PROBE_MAX   = 64;   // knob upper clamp
    static constexpr float SLOT_AIRTIME_HARD_CAP_S  = 15.0f;

    // TX-without-ACK counter: tracks consecutive native TX frames with no RR/REJ.
    // If we TX 3+ frames without any peer acknowledgment, downshift — the peer
    // likely can't decode us (asymmetric link or interference).
    int tx_no_ack_count_ = 0;
    int tx_acked_level_ = 0;       // highest O-level that got peer ACK — TX ceiling
    int tx_last_level_ = 0;        // O-level of most recent TX batch (for ACK attribution)
    // Anchor-futility demote (data-flow-tx-anchor.md §4): the anchor's proof of
    // decodability is earned by frames sized at the PREVIOUS anchor's MAX_INFO,
    // and its own credit GROWS MAX_INFO — so a level whose small frames decoded
    // can carry full-MTU frames whose FER~=1 at the EVM wall. The no-ACK demote
    // (resolve_ofdm_tx_level) floors at the anchor, and the only anchor-lowering
    // trigger was a peer REJ/RNR — which a peer that decodes NOTHING never sends
    // (measured: 53-84 consecutive anchor-clamped demotes per session, 12/12
    // sessions, terminal T1 N2 disconnect with a full window). Count consecutive
    // no-ACK demotes that bottom AT the anchor on a data batch; at the limit,
    // demote the anchor itself through the same path a peer REJ takes
    // (demote_tx_anchor). Reset on any N(R) advance.
    int ofdm_anchor_futility_ = 0;
    static constexpr int OFDM_ANCHOR_FUTILITY_LIMIT = 3;
    // The anchor demote shrinks MAX_INFO and must recover/re-fragment the
    // un-ACKed window — but from resolve_ofdm_tx_level the TX path holds an
    // already-popped batch mid-flight, and mutating the window there would put
    // BOTH the stale slicing and the re-sliced frames on air under the same
    // N(S). Defer the window mutation to the top of the next idle TX pump tick
    // (nothing popped, tx_buffer_ empty). requeue_rejected_frames drops the
    // stale popped copies against the lowered anchor cap in the meantime.
    bool ofdm_refrag_pending_ = false;
    // INV-SEQ-2 rollback safety (data-flow-owned-shadow-seq.md §5): a pending
    // re-fragment may NOT harvest/rewind V(S) across an in-flight window — the
    // peer's V(R) may already have counted those frames while the cumulative RR
    // is still in transit; the rollback would reuse their numbers with re-sliced
    // content and every later honest N(R) would read invalid (the 3-invalid-N(R)
    // desync teardown, 14/16 fast WGN:40 sessions).  consume_pending_window_refrag
    // holds the harvest until (a) V(A)==V(S) (all flight credited) or (b) quiesce:
    // owned I-frame TX held + queued data purged, REFRAG_QUIESCE_TICKS elapsed
    // (the peer's demod pipeline drained), and a valid N(R) APPLIED after that —
    // the peer's FINAL V(R) for the epoch, proving the remainder was never
    // counted.  refrag_quiesce_start_tick_ = session wall tick when the hold
    // engaged (-1 = not holding).  50 ms ticks: 160 = 8.0 s, covering the demod
    // pipeline tail + one autonomous-RR round trip after the air goes quiet.
    int refrag_quiesce_start_tick_ = -1;
    static constexpr int REFRAG_QUIESCE_TICKS = 160;
    // INV-SEQ-3 futility ACK clock: at a wide in-flight window the cumulative-ACK
    // latency of the flight itself (~1 s/frame measured at O5: 24 frames -> the
    // RR lands 21-24 s after the burst starts) exceeds the legacy 3-round
    // patience, so ACK-absence rounds are only EVIDENCE of futility once
    // wall-time-since-ACK-progress exceeds this per-outstanding-frame budget.
    // 24 ticks = 1.2 s/frame (1.2x the measured O5 delivery cadence).  Gated to
    // window_used > K_WINDOW so legacy (K<=7) futility timing is unchanged.
    static constexpr int OFDM_FUTILITY_TICKS_PER_OUTSTANDING = 24;
    // Fix B (bounded oversize-reject recovery, DATALINK_TAX_DIAGNOSIS.md): a frame
    // that fits the confirmed anchor but not the current (transiently downshifted)
    // level is re-queued to survive the level fall. ofdm_reject_streak_ counts how
    // many consecutive bursts that recovery has spun; once it crosses
    // OFDM_OVERSIZE_REJECT_LIMIT the modulated level is floored up to the min level
    // that carries the stuck frame (ofdm_fit_floor_level_, applied in
    // resolve_ofdm_tx_level) so it gets on air and is ACKed instead of spinning to a
    // T1 N2 disconnect. Both reset on any successful OFDM frame send.
    int ofdm_reject_streak_   = 0;
    int ofdm_fit_floor_level_ = -1;   // -1 = inactive
    // Receiver-drives-rate: fresh absolute forward-level proposal from the peer's
    // MFSK tone ACK (-1 = none). Latched in the tone-ACK RX adopt path and
    // consumed one-shot by the next OFDM data TX to transiently widen the TX
    // ceiling; decays back to acked+1 absent a new proposal.
    int tx_proposed_level_ = -1;
    // Max rungs the receiver-driven leap may jump above the confirmed anchor.
    static constexpr int OFDM_LEVEL_LEAP_MAX = 2;
    // Highest O-level the RECEIVER will propose to the sender. Held at O2 pending
    // END-TO-END validation of the O3 climb. The pieces are in place — O3/O4/O5
    // 16QAM ncw=4 decode CLEAN under genie (test_ofdm_genie_ncw4, d32cf5c) and RX
    // coherence during the climb is fixed by coherent adoption of B's own proposal
    // (ofdm_kiss_rx_proposed_level_ drives the frame-length gate + blind-detect
    // ordering, Mercury rank 7/9) — but two datalink gates remain UNPROVEN E2E:
    //   [?] TX-side ncw/modulation split (ofdm_cw_per_frame at modem.cc:3076 from
    //       the STALE pre-leap level vs modulation from the fresh level at :3282) —
    //       the adopt-leap frame can ship 16QAM with O2's ncw (malformed);
    //   [?] a pre-existing AX.25 forward-window deadlock (V(A)>V(S)) caps E2E at one
    //       K=7 window, so the climb never reaches O3 end-to-end to be validated.
    // Raise this cap (toward O5, the genie-validated 16QAM ceiling) only WITH an
    // end-to-end O3 net-win once those two are fixed. Cap is safe (blind-detect is
    // CRC-32 self-rejecting; over-leap self-corrects), so this is honesty, not risk.
    //
    // MEASURED (delivrate-attack, fleet WGN:40 N=3 A/B via IRIS_MAX_OFDM_LEVEL,
    // SUPERSEDED): raising this cap to O6 gave NO delivered-rate win and could STALL
    // the climb (over-propose O4/O5 -> over-leap onto a frame that fails -> demote ->
    // stuck O3). BUT that A/B ran BEFORE the upper-ladder fix: O6 was undecodable
    // live (hot-TX clip cliff) so any leap onto it necessarily failed, and the
    // demote landed at O0/O3 (pre demote_refine_). With O6/O7 now decodable live
    // (TX deviation auto-cal) + a stable channel-SNR climb metric + demote_refine_
    // settling at the confirmed anchor, the stall root is removed and the cap-lift
    // is the enabling change, not a risk. See the ROOT-CAUSE note below.
    //
    // climbgate ROOT-CAUSE (superseded 2026-07-04 by SIM_FIDELITY_EVM_VERDICT +
    // the gate32/s2gate forced-level table + the upper-ladder root-cause below):
    //   The prior conclusion here — "O6 never climbs live because 32QAM is a
    //   fundamental OFDM-over-FM-limiter PAPR floor (forced decode 23-47%), reaching
    //   O6 is deep-DSP not a cap" — is OVERTURNED. The 23-47% forced-live decode was
    //   the RELAY DEVIATION-LIMITER CLIP CLIFF at a HOT TUNE drive (data RMS ~0.40),
    //   NOT a channel law: at a MANAGED drive (RMS 0.10-0.25) the interior post-EQ is
    //   36-38 dB and forced O6 decodes 100% WGN40 / 99% WGN30 / 100% MPG40, O7 94.8%
    //   WGN40 (gate32). And "the RSP gearshift tops at O5" had TWO real roots, both
    //   now fixed: (a) the QAM32=6 enum-ordering density-cap bug hard-capped gs_max at
    //   O5 (see the modem.cc gs_max site); (b) the climb was fed the MMSE-residual
    //   effective_snr_db (18-20 dB) instead of the channel SNR (mean_channel_snr_db
    //   ~40). With (a)+(b) + the TX deviation auto-cal (ofdm_effective_tx_level O6+
    //   RMS<=0.25) + demote_refine_ (over-leap settles at the confirmed anchor, not
    //   O0), the receiver-proposal cap is the last gate on the live climb reaching
    //   O6/32QAM (VARA FM narrow top gear).
    //
    // CROSSOVER RECOVERY (2026-07-04 combined-stack): capped back to O5.
    // The 876a54a upper-ladder fixes are correct at the PHY layer (forced-level gate32
    // decodes O6/O7), but LIVE in the combined terminate stack neither O7 NOR O6 is
    // SUSTAINABLE, so letting the climb reach them REGRESSED delivered goodput from the
    // isolated-terminate 24.2k B/min (measured O5-capped) down to ~0.10-0.16x VARA:
    //   - O7/64QAM: tx_acked reached O7 in 0/8 crossover seeds; TX'd 3/8, demoted
    //     immediately ("no peer ACK for 2 TX frames") -> propose+demote thrash.
    //   - O6/32QAM: REACHED (tx_acked climbs to O6) but NOT sustained.  Measured N=3
    //     WGN:40 264KB fair corpus (recovery A/B): O6-cap delivered 31/19/14 of 288
    //     messages (STALLED) vs O5-cap 287/283/288 (near-complete).  Root: once the
    //     anchor is O6, the RSP's own gearshift collapses on marginal 32QAM decode and
    //     proposes O0; the sender adopts O0 for the modulated level, but tx_acked_level_
    //     is MONOTONIC-UP (receiver-driven adopt only raises it, modem.cc ~1965) so it
    //     stays stuck at O6 -> frames packed at max_info(O6) (~952 B) can't fit the O0
    //     modulation -> ofdm_oversize_fit_floor SPINS (4399x observed) -> near-total
    //     delivery stall on all 3 seeds.
    // The highest RELIABLY-sustainable live rung on WGN:40 is O5/16QAM (== the
    // isolated-terminate config).  Cap the proposal at O5: the anchor never reaches the
    // marginal O6/O7, so neither the O7 thrash nor the O6 anchor-stuck spin can form,
    // and delivered goodput recovers to the isolated 24.2k.  A hard cap removes the
    // thrash/spin source at the root (no hysteresis needed).  O6/O7-live sustainability
    // (the marginal-32/64QAM live-decode + the anchor-monotonic-up spin -> needs an
    // anchor-follows-persistent-collapse demote+refragment) is a SEPARATE frontier item,
    // NOT chased here; the O6/O7 PHY win stays banked (gate32).  Env IRIS_MAX_OFDM_LEVEL
    // overrides (=6 / =7 reproduce the pre-fix stall/thrash for the A/B; the O6+
    // frontier).
    static constexpr int MAX_PROPOSABLE_OFDM_LEVEL = 5;
    // Live receiver-proposal cap (defaults to the constexpr; env IRIS_MAX_OFDM_LEVEL
    // overrides it for the O6cap-vs-O7 A/B — see Modem::init). A member, not the
    // constant, is read at the proposal site so one binary drives every arm.
    int max_proposable_ofdm_level_ = MAX_PROPOSABLE_OFDM_LEVEL;
    // climbgate Fix A (demote-refine): on a KISS no-ACK after a leap onto a
    // not-yet-decodable rung, settle at the highest CONFIRMED-decodable level
    // (tx_acked_level_, one step down at most) instead of slamming to O0 — the
    // demote-cascade-to-O3 root. Env IRIS_DEMOTE_REFINE=1 for the A/B; default-ON
    // once proven (set in Modem::init).
    bool demote_refine_ = false;
    // climbgate experiment lock: env IRIS_FORCE_OFDM_LEVEL=N pins the OFDM TX level
    // AND the RSP forward proposal to N (both ends run at N through the relay), so
    // "does 32QAM decode LIVE through the FM channel" is answerable without the
    // gearshift. -1 = disabled (normal adaptive climb). Diagnostic only.
    int force_ofdm_level_ = -1;
    // CONNECT DIET (connshave lever A): skip the unconditional ~40s post-probe
    // auto-tune when the probe reports a clean high-SNR link — the tune's TX/RX
    // refinement gates nothing there (the probe already peak-matched ofdm_tx_base_,
    // which is the tune's own timeout fallback).  Default-ON; env IRIS_CONNECT_DIET=0
    // restores the always-tune baseline (fail-before arm).
    bool connect_diet_ = true;
    // Clean-probe threshold: the reverse-path probe SNR (their_tx_result_.est_snr_db,
    // median in-band tone power above the OOB noise floor) at/above which the link
    // is "clean" and the tune is skipped + the climb is sped up.  A genuine-need
    // gate, not a timeout band-aid: 20 dB is well above O5's ~18 dB post-EQ EsNo
    // requirement, so a link that can carry the top sustainable rung skips a tune
    // that would only refine an already-safe probe-calibrated level; anything
    // marginal (< 20 dB) still runs the full tune.  The WGN:40 bench measures ~75 dB
    // here, so the skip fires with a large margin.  Env IRIS_TUNE_SKIP_SNR overrides.
    float tune_skip_snr_db_ = 20.0f;
    // FASTER CLIMB (connshave lever B): on a clean probe, shorten the OFDM
    // gearshift hold (frames-per-rung) so the receiver's decode-margin climb
    // proposes the next rung sooner.  The proposal only ever advances +1 per step
    // (margin-climb caps target at ofdm_level+1) so it stays coupled to the
    // sender's actual level — no RX blind-detect poisoning, cannot over-leap to
    // O6/O7 (proposal is separately clamped to max_proposable_ofdm_level_), and a
    // real decode failure still demotes.  Default-ON; env IRIS_CLEAN_CLIMB=0 =
    // fail-before.  clean_climb_hold_ = frames-per-rung on a clean link (env
    // IRIS_CLEAN_CLIMB_HOLD; default 2 vs the OFDM_HOLD_FRAMES=5 baseline).
    bool clean_climb_ = true;
    int  clean_climb_hold_ = 2;
    uint8_t last_peer_nr_ = 0xFF;  // last N(R) from peer RR — 0xFF = no RR yet

    // A3 — per-N(S) TX level ring. Records the OFDM speed level each outstanding
    // I-frame was transmitted at, indexed by N(S) (parallels the AX.25 tx_window_).
    // The ACK anchor (tx_acked_level_) must rise to the MAX level among frames the
    // peer ACTUALLY acknowledged, not merely the most-recent DATA batch: a climb/leap
    // window legitimately spans 2-3 levels, and the scalar tx_last_level_ conflates
    // "most recently sent" with "acked at". -1 = empty.  Sized to the widest sequence
    // modulus (modulo-128, wide-window lever); the iteration modulus is passed in so
    // the ring walks the SAME sequence space as the AX.25 session (mod-8 or mod-128).
    static constexpr int RING_MAX = 128;
    struct TxLevelRing {
        int8_t level[RING_MAX];
        TxLevelRing() { reset(); }
        void reset() { for (int i = 0; i < RING_MAX; i++) level[i] = -1; }
        // Record the level a frame at N(S) was sent at. MIN semantics: a frame that
        // is retransmitted at a LOWER level, OR that sits in the window while NEWER
        // frames go out at a HIGHER level (an in-flight climb), is credited
        // conservatively at the LOWEST level it was actually sent at. The anchor
        // must never rise above a level a frame was genuinely, successfully sent
        // at (invariant I4/I5) — so an in-flight climb only lifts the anchor once
        // the lower-level frames it straddles are ACKed and cleared.
        void stamp(uint8_t ns, int lvl) {
            int8_t& s = level[ns % RING_MAX];
            s = (s < 0) ? (int8_t)lvl : (int8_t)((int)s < lvl ? (int)s : lvl);
        }
        // Max level among stamped N(S) in [from, to) forward in `mod`; -1 if none.
        int max_acked(uint8_t from, uint8_t to, int mod) const {
            int m = -1;
            for (uint8_t n = from; n != to; n = (uint8_t)((n + 1) % mod))
                if ((int)level[n % RING_MAX] > m) m = level[n % RING_MAX];
            return m;
        }
        void clear_range(uint8_t from, uint8_t to, int mod) {
            for (uint8_t n = from; n != to; n = (uint8_t)((n + 1) % mod))
                level[n % RING_MAX] = -1;
        }
    };
    TxLevelRing tx_level_ring_;

    // OFDM-KISS transport-layer compression
    uint16_t ofdm_kiss_peer_caps_ = 0;         // Negotiated caps (intersection of local & peer)
    bool v2_negotiated_active_ = false;
    std::shared_ptr<v2::LiveSession> v2_live_session_;
    Compressor ofdm_kiss_tx_compressor_;
    Compressor ofdm_kiss_rx_compressor_;

    // OFDM-KISS B2F proxy
    B2fHandler ofdm_kiss_b2f_;
    std::vector<uint8_t> b2f_proxy_plaintext_;  // Accumulated plaintext from B2F unroll
    std::vector<std::vector<uint8_t>> b2f_proxy_originals_; // locally ACKed source frames
    bool b2f_proxy_active_ = false;             // TX intercepting B2F payload
    bool b2f_proxy_rx_active_ = false;          // RX reassembling B2F payload
    // B2F AFSK history: buffer I-frame info fields during AFSK phase so the
    // B2F handler can replay SID/FC/FS exchanges when it initializes at OFDM-KISS activation.
    std::vector<std::vector<uint8_t>> b2f_afsk_tx_history_;
    std::vector<std::vector<uint8_t>> b2f_afsk_rx_history_;
    uint8_t b2f_proxy_vr_ = 0;                  // V(R) for generating local RR ACKs
    uint8_t b2f_proxy_addr_[14] = {};            // Cached AX.25 address header (for ACK/I-frame construction)
    bool b2f_proxy_addr_valid_ = false;         // Address header cached
    int  ofdm_kiss_probe_cd_ = 0;   // Tick countdown before initiator starts probe
    bool ofdm_kiss_probe_done_ = false; // Probe completed, PHY reconfigured
    bool probe_manual_ = false;         // Manual probe (button) vs in-session
    int  disconnect_timeout_ticks_ = 0; // 30s timeout: stop TX if stuck in AWAITING_RELEASE
    bool probe_start_pending_ = false;  // Deferred PROBE:START (can't send from state callback)
    std::string probe_peer_call_;       // Peer callsign for standalone probe result addressing
    XidCapability local_cap_;

    // Probe result cache: skip re-probing peers within 24h.
    // Stores the NegotiatedPassband + peer caps + probe results per callsign.
    struct ProbeCacheEntry {
        NegotiatedPassband negotiated;
        ProbeResult my_tx;       // peer's analysis of our probe (contains their caps)
        ProbeResult their_tx;    // our analysis of their probe
        std::chrono::steady_clock::time_point timestamp;
    };
    std::unordered_map<std::string, ProbeCacheEntry> probe_cache_;
    static constexpr int PROBE_CACHE_EXPIRY_S = 86400;  // 24 hours
    void force_activate_ofdm(const std::string& callsign);
    bool try_use_cached_probe(const std::string& callsign);
    void cache_probe_result(const std::string& callsign);
    bool save_probe_to_disk(const std::string& callsign, const ProbeCacheEntry& entry);
    bool load_probe_from_disk(const std::string& callsign, ProbeCacheEntry& entry);
    void purge_probe_cache(const std::string& callsign);  // item 5: drop poisoned entry (memory+disk)

    // Probe-before-connect: AX.25 SABM deferred until probe completes
    std::string pending_connect_call_;   // Remote callsign waiting for probe to finish
    int probe_connect_timeout_ = 0;     // Ticks before giving up on probe and connecting AFSK

    std::string xid_peer_call_;                  // Peer callsign for probe exchange

    // Auto-tune state
    std::atomic<TuneState> tune_state_{TuneState::IDLE};
    std::string tune_peer_call_;           // Remote callsign for tune exchange
    bool tune_is_initiator_ = false;       // true = we started the tune
    float tune_peer_gain_ = 0;            // What peer measured from our test frame
    float tune_my_gain_ = 0;             // What we measured from peer's test frame
    float native_rx_gain_ = 1.0f;        // RX gain correction (1/my_gain from auto-tune)
    int tune_timeout_ = 0;               // Tick countdown for timeout
    int tune_test_frames_sent_ = 0;      // Test frames transmitted
    int tune_test_frames_target_ = 3;    // Send 3 test frames for averaging
    int tune_frames_measured_ = 0;       // Peer test frames we've measured gain from
    int tune_wait_peer_ticks_ = 0;       // Ticks spent in WAIT_PEER (for timeout transition)
    int tune_last_measured_count_ = 0;   // For silence detection: last known frames_measured_
    int tune_silence_ticks_ = 0;         // Ticks since last new measurement
    int tune_ready_resend_cd_ = 0;       // Countdown to resend TUNE:START in WAIT_READY
    int tune_ready_resends_ = 0;         // Number of TUNE:START retransmissions
    int tune_report_resend_cd_ = 0;      // Countdown to resend TUNE report in WAIT_REPORT
    int tune_report_resends_ = 0;        // Number of report retransmissions (max 2)
    int tune_report_delay_cd_ = 0;       // Responder delay before sending report (avoid collision)
    int tune_post_holdoff_ = 0;           // Responder TX holdoff after TUNE DONE (let initiator go first)

    // OFDM power-ramp TUNE: send 10 frames at geometrically-spaced TX levels.
    // Peer measures channel estimate H from preamble (even on LDPC failure)
    // and LDPC iters when decode succeeds. Reports all data points.
    // Sender fits a parabola in dB space to find the optimal drive level.
    //
    // 5 levels spanning the usable range from max to min hardware output.
    // Scales computed dynamically at TUNE start based on current tx_base to
    // guarantee all 5 frames use distinct power levels (no clamping waste).
    // Quality vs tx_dB is U-shaped: too quiet = noise, too loud = FM clipping.
    // OTA testing shows 5-7 of 10 frames typically decode; 5 is sufficient.
    static constexpr int TUNE_RAMP_COUNT = 5;
    static constexpr float TUNE_SCALE_MIN = 0.15f;  // minimum absolute tx_level (OTA: <0.15 below FM noise floor)
    static constexpr float TUNE_SCALE_MAX = 1.0f;   // maximum absolute tx_level
    float tune_computed_scales_[TUNE_RAMP_COUNT] = {};  // computed per-session
    void tune_compute_scales(float base);  // populate tune_computed_scales_
    float tune_ramp_tx_levels_[TUNE_RAMP_COUNT] = {};  // actual tx_level per ramp frame
    // Peer's report: iters and H for each of our ramp frames
    int tune_peer_iters_[TUNE_RAMP_COUNT] = {};
    float tune_peer_H_[TUNE_RAMP_COUNT] = {};
    float tune_peer_snr_[TUNE_RAMP_COUNT] = {};   // peer's reported SNR (dB)
    // RX side: track quality of each received ramp frame from peer
    // H is recorded from preamble on every detection (even LDPC failure).
    // iters = -1 means not decoded, -2 means preamble-only (H valid, no LDPC).
    int tune_rx_frame_iters_[TUNE_RAMP_COUNT] = {};
    float tune_rx_frame_H_[TUNE_RAMP_COUNT] = {};
    float tune_rx_frame_snr_[TUNE_RAMP_COUNT] = {};  // SC-metric SNR (dB) per frame
    // Compute optimal tx_level from parabolic fit of (tx_dB, iters) data
    float tune_fit_tx_level() const;
    std::string tune_build_ramp_report() const;  // builds RAMP3 or RAMP7 payload

    // Per-O-level TX drive offset (dB below O0 baseline).
    // Higher modulations have tighter constellations → more sensitive to FM
    // deviation limiter clipping. Applied multiplicatively at TX time.
    float ofdm_tx_base_ = 0.0f;         // O0 baseline tx_level (set by TUNE)
    // O10..O13 (256/1024QAM) reuse O9's -3 dB offset (index clamped in
    // ofdm_effective_tx_level); the O6+ entries are further hard-capped there so
    // the on-air data RMS stays <= 0.25 (off the deviation-limiter clip cliff).
    static constexpr float ofdm_level_offset_db_[10] = {
         0.0f,   // O0: BPSK r1/2
         0.0f,   // O1: QPSK r1/2
         0.0f,   // O2: QPSK r3/4
        -1.0f,   // O3: 16QAM r1/2
        -1.0f,   // O4: 16QAM r5/8
        -1.0f,   // O5: 16QAM r3/4
        -2.0f,   // O6: 32QAM r5/8 (VARA FM narrow top gear)
        -2.0f,   // O7: 64QAM r5/8
        -3.0f,   // O8: 64QAM r3/4
        -3.0f,   // O9: 256QAM r5/8
    };
    float ofdm_effective_tx_level() const;  // base * 10^(offset/20)

    void send_tune_ui(const char* payload);
    void handle_tune_frame(const uint8_t* info, size_t len);
    void tune_build_and_queue_test_frame();
    void tune_apply_corrections();
    void tune_audit(const char* fmt, ...);   // Audit to main log
    void kalman_log_trace(const KalmanTrace& trace, bool decode_ok,
                          float snr, float gain);  // Dump to kalman_trace.csv
    FILE* kalman_log_file_ = nullptr;

    // Calibration
    std::atomic<CalState> cal_state_{CalState::IDLE};
    float cal_measured_rms_ = 0;
    int cal_tone_samples_ = 0;
    float cal_tone_phase_ = 0;
    float cal_rms_accum_ = 0;
    int cal_rms_count_ = 0;

    // Stats
    std::atomic<int> frames_rx_{0};
    std::atomic<int> frames_tx_{0};
    std::atomic<int> crc_errors_{0};
    std::atomic<uint64_t> bytes_rx_{0};
    std::atomic<uint64_t> bytes_tx_{0};
    int crypto_state_ = 0;  // 0=off, 1=kx, 2=encrypted, 3=psk_mismatch
    void generate_ephemeral_x25519();
    void start_mlkem_exchange();
    void handle_mlkem_frame(const uint8_t* data, size_t len);
    void rekey_hybrid();
    bool mlkem_kx_pending_ = false;   // waiting for ML-KEM exchange to complete
    bool mlkem_releasing_tx_ = false;
    bool mlkem_release_buffered_ = false;
    size_t mlkem_release_prepared_count_ = 0;
    std::vector<std::vector<uint8_t>> mlkem_held_frames_; // original record boundaries retained
    std::vector<std::vector<uint8_t>> mlkem_rx_held_records_;
    std::vector<uint8_t> native_rx_record_;
    ArqRecordType native_rx_record_type_ = ArqRecordType::Data;
    uint64_t native_rx_record_id_ = 0;
    struct NativeCompletedRecord {
        ArqRecordType type = ArqRecordType::Data;
        uint64_t record_id = 0;
        std::array<uint8_t, 32> digest{};
    };
    std::deque<NativeCompletedRecord> native_completed_records_;
    static constexpr size_t NATIVE_MAX_ENCODED_RECORD = 10 * 1024 * 1024;
    float rx_rms_ = 0;
    std::atomic<float> rx_raw_rms_{0};  // Pre-AGC RMS for DCD
    float rx_peak_ = 0;  // Peak sample value (decays over time)
    int rx_diag_counter_ = 0;  // Periodic RX diagnostic counter

    // Pre-allocated RX audio working buffer (avoids per-callback heap allocation)
    std::vector<float> rx_audio_tmp_;
    std::vector<float> rx_eq_tmp_;   // Pre-allocated EQ scratch buffer

    // RX overlap buffer for native mode
    std::vector<float> rx_overlap_buf_;
    static constexpr size_t RX_OVERLAP_MAX = 48000 * 40;  // 20 seconds of IQ (floats=IQ×2)
    int pending_frame_start_ = -1;   // Cached frame start when waiting for more data
    size_t pending_need_floats_ = 0; // Min buffer size (floats) needed before retry
    int pending_frame_timeout_ = 0;  // Samples remaining before abandoning pending frame
    bool relisten_pending_ = false;  // Deferred return to LISTENING after failed hail

    // Waterfall spectrum (protected by modem_mutex_)
    std::vector<std::complex<float>> last_constellation_;
    KalmanTrace last_kalman_trace_;
    std::vector<float> last_spectrum_;
    float spectrum_low_hz_ = 0;
    float spectrum_high_hz_ = 4000.0f;
    std::vector<float> spectrum_buf_;
    int spectrum_buf_pos_ = 0;

    // Callback to deliver received frames
    std::function<void(const uint8_t*, size_t)> rx_callback_;

    // Callback for ARQ state changes (state, remote_callsign)
    std::function<void(ArqState, const std::string&)> state_callback_;

    // Callback for AX.25 session state changes
    std::function<void(Ax25SessionState, const std::string&)> ax25_state_callback_;

    // GUI event log
    std::function<void(const std::string&)> gui_log_;

    // Packet log callback (is_tx, protocol, description)
    std::function<void(bool, const std::string&, const std::string&)> packet_log_;
};

} // namespace iris

#endif
