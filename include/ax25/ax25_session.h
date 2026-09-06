#ifndef IRIS_AX25_SESSION_H
#define IRIS_AX25_SESSION_H

#include "ax25/ax25_protocol.h"
#include "common/types.h"
#include <atomic>
#include <functional>
#include <mutex>
#include <queue>
#include <random>
#include <string>

namespace iris {

// AX.25 2.2 Data Link State Machine (Section 4.3)
// States map to AX.25 2.2 spec state numbers:
//   State 1 = DISCONNECTED
//   State 2 = AWAITING_CONNECTION
//   State 3 = CONNECTED
//   State 4 = TIMER_RECOVERY
//   State 5 = AWAITING_RELEASE
enum class Ax25SessionState {
    DISCONNECTED,           // State 1: no link
    AWAITING_CONNECTION,    // State 2: sent SABM, waiting UA
    CONNECTED,              // State 3: information transfer (ABM)
    TIMER_RECOVERY,         // State 4: T1 expired, polling peer
    AWAITING_RELEASE,       // State 5: sent DISC, waiting UA/DM
};

class Ax25Session {
public:
    using SendFrameFunc = std::function<void(const uint8_t*, size_t)>;
    using DataReceivedFunc = std::function<void(const uint8_t*, size_t)>;
    using StateChangedFunc = std::function<void(Ax25SessionState, const std::string&)>;
    // Terminate/re-pack (native OFDM data plane): fired ONCE per strictly
    // in-order I-frame accepted on the owned R2 sequence (ns==V(R)), carrying the
    // I-frame INFO for stream reassembly.  The modem sets it to feed the
    // length-delimited reassembler; V(R) is the single dedup/order gate (M0 one
    // owned sequence), so the reassembler never sees a duplicate or reorder.
    using NativeStreamRxFunc = std::function<void(const uint8_t*, size_t)>;
    // Returns true only after the custody owner has durably admitted the exact
    // in-order INFO bytes.  V(R) must not advance when admission is refused.
    using NativeStreamAdmissionFunc =
        std::function<bool(const uint8_t*, size_t)>;
    // Reports the exact INFO bytes removed by a validated cumulative N(R).
    // The custody owner uses this to advance record milestones independently
    // of queue occupancy.
    using AckedInfoFunc = std::function<void(const uint8_t*, size_t)>;
    // Unrecoverable N(R) desync hook (DATALINK_INTEGRITY_AUDIT §4/F1). Fired from
    // nr_error_recovery() the instant a desync is declared, BEFORE the FRMR+SABM
    // re-establish. Returns true iff the owner (the modem) handled the desync with a
    // custody terminal cause (in the native/re-pack transport the AX.25 re-establish
    // is a dead end — kiss-managed CONNECTED ignores SABM, the OFDM pump suppresses
    // it, T1 never re-sends it). AX.25 then destroys and latches the old session;
    // the ordinary DISCONNECTED state callback is the sole terminal publication
    // hook. Returns false (or unset) to run normal AX.25 re-establishment where it
    // remains reachable.
    using DesyncFunc = std::function<bool()>;

    Ax25Session();

    void set_local_callsign(const std::string& call) { local_call_ = call; }
    void set_send_callback(SendFrameFunc cb) { send_frame_ = cb; }
    void set_data_callback(DataReceivedFunc cb) { data_received_ = cb; }
    void set_state_callback(StateChangedFunc cb) { state_changed_ = cb; }
    void set_native_stream_rx_callback(NativeStreamRxFunc cb) { native_stream_rx_ = std::move(cb); }
    void set_native_stream_admission_callback(NativeStreamAdmissionFunc cb) {
        native_stream_admission_ = std::move(cb);
    }
    void set_acked_info_callback(AckedInfoFunc cb) { acked_info_ = std::move(cb); }
    void set_desync_callback(DesyncFunc cb) { desync_recovery_ = std::move(cb); }
    // True only for a CONNECTED notification that resumes the same sequence
    // epoch from TIMER_RECOVERY.  A SABM link reset increments the epoch first,
    // so TIMER_RECOVERY -> CONNECTED after re-establishment remains a new link.
    bool ordinary_timer_recovery_return() const noexcept {
        return state_ == Ax25SessionState::CONNECTED &&
               previous_state_ == Ax25SessionState::TIMER_RECOVERY &&
               previous_state_generation_ == session_generation_;
    }
    // Current dynamic I-frame info limit (OFDM capacity anchor) — the re-pack
    // drain slices the stream into <= max_info() chunks so R2 frames never
    // exceed the peer-ACKed level's block capacity.
    int max_info() const { return max_info_; }
    void set_t1_ticks(int ticks) { t1_value_ = ticks; }

    // Update T1 floor based on TNC TXDELAY (accounts for real turnaround time).
    // T1 >= 2*TXDELAY + 1.5s processing/frame-time, but never below FRACK default.
    void set_txdelay_ms(int ms);

    // Update T3 (idle supervision) in ticks
    void set_t3_ticks(int ticks) { t3_value_ = ticks; }

    // Channel busy feedback (Direwolf pattern): pause T1/T3 while DCD or PTT
    // is active.  On half-duplex radio, we can't expect an acknowledgment while
    // the channel is occupied — either by the remote station (DCD) or by our
    // own transmission (PTT).  Prevents false T1 timeouts and unnecessary
    // retransmissions.  Call from modem when DCD or PTT state changes.
    void set_channel_busy(bool busy);

    // Initiate outgoing connection (sends SABM).  `remote_call` may name a
    // digipeater path — "DST via DIGI1,DIGI2" (case-insensitive "via", hops
    // comma- or space-separated) — in which case every session frame is built
    // with that path (H-bits clear; the digipeaters set them hop by hop).
    void connect(const std::string& remote_call);
    // Explicit-path overload (AGW v-connect / CLI callers that already parsed).
    void connect(const std::string& remote_call, const Ax25ViaPath& via);

    // Send data within connection (queues I-frame)
    void send_data(const uint8_t* data, size_t len);

    // Disconnect (sends DISC)
    void disconnect();

    // Handle received parsed AX.25 frame. Returns true if consumed.
    bool on_frame_received(const Ax25Frame& frame);

    // Notify session of an outgoing frame from KISS (track state without
    // generating our own frames — the KISS client already has the frame queued).
    void notify_outgoing(const uint8_t* frame, size_t len);

    // Timer tick — call every ~50ms
    void tick();

    // Reset to disconnected
    void reset();
    // Custody-owner terminal cleanup. Clears the complete old session without
    // invoking state callbacks; the owner publishes only after its ledger latch
    // and all other old transport state are finalized.
    void reset_silent();

    // Queries
    Ax25SessionState state() const { return state_; }
    const std::string& remote_callsign() const { return remote_call_; }
    // Digipeater ("via") path every session-generated frame is built with.
    // ORIGINATOR: the connect request's path as given, H-bits clear.
    // RESPONDER: the REVERSED path of the received SABM, H-bits cleared —
    // replies traverse the digipeaters in reverse order (each direction's
    // digis mark their own H-bits).  Direwolf does exactly this for an
    // incoming connection: get_link_handle() swaps src/dst and reverses the
    // digi path (ax25_link.c:877-890).  Empty = direct (the historical,
    // byte-identical encoding).
    const Ax25ViaPath& remote_via() const { return remote_via_; }
    int pending_frames() const;
    bool is_active() const { return state_ != Ax25SessionState::DISCONNECTED; }
    bool we_initiated() const { return we_initiated_; }
    uint8_t current_vr() const { return vr_; }
    bool is_kiss_managed() const { return kiss_managed_; }
    // Native mode flag: when true, T1 polls are NOT suppressed even in
    // KISS-managed mode. In OFDM-KISS native mode, the session layer is
    // the actual transport and must actively poll on timeout.
    void set_native_active(bool v) { native_active_ = v; if (v) lower_t1_for_native(); }
    bool native_active() const { return native_active_; }
    void set_t1_floor_for_airtime(float airtime_s);  // Raise T1 floor for long native frames
    void set_max_info(int n);  // Limit I-frame info field for OFDM capacity
    void drop_oversized_in_window();  // Discard unacked frames exceeding max_info_
    // Terminate/re-pack engage: the unacked I-frames in [V(A),V(S)) have been
    // ABSORBED into the re-pack stream (their data will fly re-packed), so discard
    // them from the shadow window and roll V(S) back to V(A).  The owned R2
    // sequence then flies from V(A) (== the far end's V(R), which counted the same
    // ACKed prefix) — seam-free.  Keeps the connection + V(A)/V(R) intact.
    void rollback_unacked_window();
    // Terminate/re-pack RE-FRAGMENT (crossover recovery): harvest the INFO bytes of
    // every un-ACKed window frame [V(A),V(S)) in send order, then the not-yet-
    // windowed tx_queue_ frames in order, INTO `out` (== the exact un-ACKed byte
    // suffix of the owned R2 stream), THEN roll the window back to V(A).  The re-pack
    // layer prepends `out` to its stream and re-drains at the (now smaller) max_info
    // -> a mid-transfer demote NEVER drops stream bytes (the [len] records re-slice
    // bit-exact).  Distinct from rollback_unacked_window(), which DISCARDS the window
    // because those bytes still live in the stream; here the window bytes were already
    // erased from the stream when send_data() consumed them, so we RECOVER them first.
    void recover_unacked_stream(std::vector<uint8_t>& out);
    void set_kiss_passthrough(bool v) { kiss_passthrough_ = v; }
    bool is_kiss_passthrough() const { return kiss_passthrough_; }
    // #2 burst-epoch guard.  The modem sets this FALSE immediately before feeding a
    // reverse ACK (MFSK-injected RR or OFDM S-frame) whose echoed burst-epoch does
    // NOT match the burst the sender is waiting to have acked; the shadow S-frame
    // V(A)-advance consumer then treats that N(R) as ADVISORY (no destructive
    // ack_frames — the cheap miss direction) instead of purging the window on a
    // stale/buffered tone.  Default TRUE (fail-open toward the #1 live-N(R) fix);
    // the modem resets it to TRUE after each guarded frame.
    void set_reverse_ack_epoch_ok(bool ok) { reverse_ack_epoch_ok_ = ok; }
    bool reverse_ack_epoch_ok() const { return reverse_ack_epoch_ok_; }
    // Increment 1 (window-fill) collision-safety: HOLD the autonomous T2 RR until
    // the initiator's burst SEQUENCE goes quiet, so the reverse ACK never collides
    // with the next forward burst when the initiator runs burst_fill_continue.
    // Off by default (unchanged stop-and-wait 50ms autonomous RR); the modem turns
    // it on iff IRIS_BURST_FILL is set, so both sides flip on the same flag.
    void set_rr_hold(bool en) { rr_hold_ = en; }
    bool rr_hold() const { return rr_hold_; }
    // WIDE WINDOW (terminate turnaround lever).  The terminated OFDM tier is
    // Iris<->Iris and owns its own sequence space, so the legacy mod-8/K=7 GBN
    // window (a per-7-frame ACK stall = the ~30% steady turnaround at O5) is a
    // client-AX.25 constraint the owned transport does NOT need.  Switch the OWNED
    // R2 session to AX.25 2.2 modulo-128 with a large window K so the sender streams
    // many more full frames before it must stop for a reverse ACK -> the per-window
    // turnaround amortizes toward 0.  The modem calls this at native activation iff
    // IRIS_WIDE_WINDOW is set (both ends flip on the same env), with the SAME modulus
    // (128) and K so the sequence spaces match.  Owned I/S frames are then built and
    // parsed with the 2-octet (extended) control field carrying 7-bit N(S)/N(R).
    // DISABLED => mod-8/K=7 exactly (byte-identical fail-before; the pure-KISS/AFSK
    // client tier is UNCHANGED — this only widens the internal OFDM transport).
    void set_wide_window(bool enable, int k);
    // Grow/shrink ONLY the window cap (K), leaving the modulo-128 wire FORMAT set.
    // The modem keeps K small (=7) during the O0->O5 climb — the receiver-driven
    // climb needs the frequent per-7-frame ACK cadence to ratchet the level up; a
    // wide window at a low level fills 63 slow frames before the first ACK and the
    // climb stalls.  Once the anchor reaches the sustainable O5, the modem grows K to
    // the wide value so the STEADY-STATE per-window turnaround amortizes toward 0.
    // No-ops unless wide (seq_mod_>8); clamps to [K, seq_mod_-1].
    void set_window_k(int k) {
        if (seq_mod_ <= 8) return;
        if (k < K) k = K;
        if (k > seq_mod_ - 1) k = seq_mod_ - 1;
        window_k_ = k;
    }
    bool wide_window() const { return seq_mod_ > 8; }
    bool extended() const { return extended_; }
    int  seq_mod() const { return seq_mod_; }
    int  window_k() const { return window_k_; }
    // Rollback safety (data-flow-owned-shadow-seq.md INV-SEQ-2): while a demote
    // re-fragment is PENDING the modem holds ALL owned I-frame TX (window fills and
    // re-airs; send_next_iframe() is the single funnel).  The held frames are about
    // to be re-sliced anyway, and quiescing the data direction is what makes the
    // peer's next cumulative N(R) its FINAL V(R) for the epoch — the proof that a
    // V(S) rollback reuses no sequence number the peer counted.  S-frames (T1
    // polls, RRs) are unaffected, so ACK resolution keeps flowing during the hold.
    void set_iframe_hold(bool v) { tx_iframe_hold_ = v; }
    bool iframe_hold() const { return tx_iframe_hold_; }
    // 50 ms wall clock + ACK-observation stamps for the modem's re-fragment quiesce
    // gate (INV-SEQ-2) and the anchor-futility ACK clock (INV-SEQ-3).
    int wall_ticks() const { return wall_ticks_; }
    // Wall tick of the last APPLIED valid N(R) (S-frame advance or valid-no-advance,
    // I-frame piggyback).  Epoch-HELD reverse ACKs (stale tones) do NOT stamp.
    int last_valid_nr_tick() const { return last_valid_nr_tick_; }
    // Wall tick of the last V(A) advance, or of the first frame entering an empty
    // window (the flight's ACK clock starts when the flight starts).
    int last_ack_progress_tick() const { return last_ack_progress_tick_; }
    // Inject a received RR as if it arrived over the air. Builds a properly
    // addressed S-frame and feeds it through the full state machine (V(A)
    // advance, T1 reset, timer recovery exit, etc.). Used by MFSK ACK
    // detector which decodes N(R) from tones rather than AX.25 frames.
    void inject_rx_rr(uint8_t nr, bool pf);
    // Build the same RR frame that inject_rx_rr processes (for KISS forwarding).
    std::vector<uint8_t> build_rx_rr(uint8_t nr, bool pf);
    int retry_count() const { return retry_count_; }
    uint8_t vr() const { return vr_; }  // For B2F proxy I-frame injection
    // Window/sequence accessors (test + diagnostics): V(A) <= V(S) is invariant.
    uint8_t vs() const { return vs_; }
    uint8_t va() const { return va_; }
    int window_used() const { return ((int)vs_ - (int)va_ + seq_mod_) % seq_mod_; }
    // T1 remaining ticks (0 = not running).  Test/diagnostic accessor.
    int t1_ticks() const { return t1_; }

    // Flow control: set receiver busy (sends RNR instead of RR to polls).
    // Use during modem reconfiguration (speed change, probe) to pause
    // the peer's I-frame transmissions without dropping the connection.
    void set_own_busy(bool busy);
    void set_own_busy_silent(bool busy) { own_busy_ = busy; }
    bool own_busy() const { return own_busy_; }

    // Request immediate retransmission: sends REJ(V(R)) to tell the peer
    // to retransmit from the last expected sequence number.  Use when the
    // modem detects a native frame decode failure (CRC/LDPC) — much faster
    // than waiting for the AX.25 T1 timeout (2-4s in native mode).
    void request_retransmit();

    // Start T1 if there are unacknowledged frames (V(A) != V(S)).
    // Belt-and-suspenders: called at OFDM TX time to ensure T1 is running
    // even if notify_outgoing() was called before native_active_ was set.
    void start_t1_if_unacked();

    // Suspend/resume T1 while data TX is physically deferred (auto-tune ramp
    // owns the channel — can exceed 30s).  While deferred, the held I-frames
    // CANNOT be transmitted, so an ACK timer against them fires a spurious
    // TIMER_RECOVERY (P0 connect-recovery turnaround drop).  set_tx_deferred(true)
    // stops the running T1 (and any channel-busy-paused remainder) and makes
    // start_t1_if_unacked()/notify_outgoing() no-op; the post-tune first OFDM
    // burst re-arms T1 with the proper airtime floor.  Idempotent.
    void set_tx_deferred(bool deferred);
    bool tx_deferred() const { return tx_deferred_; }

private:
    void set_state(Ax25SessionState s);

    // Frame building and sending
    void send_sabm();
    void send_ua(bool pf);
    void send_disc();
    void send_dm(bool pf);
    void send_rr(bool pf, bool command);
    void send_rnr(bool pf, bool command);
    void send_rej(bool pf, bool command);
    void send_frmr(uint8_t rejected_ctrl, uint8_t vs, uint8_t vr, bool cr,
                   bool w_bit, bool x_bit, bool y_bit, bool z_bit);
    void send_next_iframe();

    // N(R) processing
    bool nr_valid(uint8_t nr) const;
    void ack_frames(uint8_t nr);
    // New sequence epoch (connect / SABM re-establish): zero the counters AND clear
    // every per-sequence artifact of the old epoch (full tx_window_ ring + RX
    // reorder buffer).  A stale reorder entry keyed by an OLD-epoch N(S) must never
    // drain as data under NEW-epoch numbering (silent-corruption hazard), and a
    // stale tx_window_ slot must never retransmit under a reused number.
    void reset_sequence_epoch();
    void retransmit_from(uint8_t nr);
    // Selective single-frame retransmit (native OFDM-KISS cheap-miss): re-send ONLY
    // the frame at `seq`, not the whole outstanding window.  Used on the duplicate-
    // stuck cumulative RR (the "frame at V(A) was lost" signal) once the reordering
    // receiver buffers every OTHER arrived frame, so go-back-N would re-send frames
    // that already flew — the WGN:30 loss amplifier.
    void retransmit_one(uint8_t seq);
    bool in_window(uint8_t ns) const;

    // Internal state management
    void reset_session();
    void establish_data_link();
    void clear_exception_conditions();
    void enquiry_response(bool pf);
    void invoke_retransmission();
    void nr_error_recovery();
    void check_iframe_queued();
    void check_need_for_response(bool pf);
    void select_t1_value();
    void update_srt(int rtt);
    void lower_t1_for_native();

    // AX.25 2.2 system parameters (Section 6)
    // Tick rate: 50ms (main loop sleeps 50ms, modem.tick() calls ax25_session.tick())
    static constexpr int K = 7;             // Default mod-8 window (max outstanding I-frames)
    static constexpr int SEQ_MOD_MAX = 128; // Widest sequence modulus (AX.25 2.2 modulo-128)
    // Live sequence modulus + window.  Default mod-8/K=7 (classic AX.25, byte-identical
    // to the pre-wide-window behavior).  set_wide_window() raises them for the OWNED
    // R2 OFDM transport ONLY.  seq_mod_ is ALSO the tx_window_ ring size in use.
    int seq_mod_   = 8;
    int window_k_  = K;
    bool extended_ = false;                 // build/parse owned I/S with 2-octet control
    // Sequence arithmetic parameterized on the live modulus (mod-8 or modulo-128).
    uint8_t seq_inc(uint8_t s) const { return (uint8_t)((s + 1) % seq_mod_); }
    uint8_t seq_add(uint8_t s, int d) const { return (uint8_t)((s + d) % seq_mod_); }
    // Forward distance a - b in [0, seq_mod_): 0 == equal, up to seq_mod_-1.
    int seq_dist(uint8_t a, uint8_t b) const { return ((int)a - (int)b + seq_mod_) % seq_mod_; }
    static constexpr int N2 = 10;           // Max retries (Direwolf default: 10)
    int n2_effective_ = N2;                 // Dynamic N2 (raised for native OFDM)
public:
    static constexpr int T1_TICKS = 80;     // 4.0s ack timeout (Direwolf FRACK=4)
    static constexpr int K_WINDOW = K;      // max outstanding I-frames (public view of K)
private:
    static constexpr int T2_TICKS = 6;      // 0.3s response delay timer
    static constexpr int T3_TICKS = 6000;   // 300s idle supervision (Direwolf default: 5 min)
    static constexpr int MAX_INFO = 1024;   // Max I-frame info field bytes (N1)
    int max_info_ = MAX_INFO;               // Dynamic limit (OFDM capacity)

    Ax25SessionState state_ = Ax25SessionState::DISCONNECTED;
    Ax25SessionState previous_state_ = Ax25SessionState::DISCONNECTED;
    uint64_t state_generation_ = 0;
    uint64_t previous_state_generation_ = 0;
    std::string local_call_;
    std::string remote_call_;

    // Sequence numbers (mod 8, AX.25 2.2 Section 4.2.4)
    uint8_t vs_ = 0;   // V(S) — send state variable
    uint8_t vr_ = 0;   // V(R) — receive state variable
    uint8_t va_ = 0;   // V(A) — acknowledge state variable

    // TX queue (data waiting to enter window) and window
    std::queue<std::vector<uint8_t>> tx_queue_;
    struct TxIFrame {
        std::vector<uint8_t> info;
        bool sent = false;
    };
    // Sized to the widest modulus (modulo-128).  Only slots [0, seq_mod_) are used at
    // the current modulus; every ring index is a full sequence number (< seq_mod_), so
    // slots are addressed directly (no % needed) and the mod-8 default touches only 0..7.
    TxIFrame tx_window_[SEQ_MOD_MAX];

    // --- Selective-repeat RX reorder buffer (native OFDM-KISS cheap-miss) ---
    // A forward, in-window, OUT-OF-SEQUENCE I-frame is BUFFERED here (indexed by its
    // sequence number) instead of being DROPPED (the pre-fix go-back-N behavior that
    // amplified moderate loss into a stall at WGN:30).  On gap-fill the contiguous
    // buffered frames are drained in strict V(R) order, so V(R) jumps forward past
    // every already-arrived frame and the next cumulative RR acks them all — the
    // sender never re-sends a frame that already flew.  Native-mode only; the slots
    // stay empty/false for legacy AFSK / pure-KISS (byte-identical fail-before).
    std::vector<uint8_t> rx_reorder_buf_[SEQ_MOD_MAX];
    bool rx_reorder_present_[SEQ_MOD_MAX] = {};

    // Timers (counted in ticks, 50ms per tick)
    int t1_value_ = T1_TICKS;  // Configurable T1 value (default 4s, Direwolf FRACK=4)
    int t3_value_ = T3_TICKS;  // Configurable T3 value (default 300s)
    int t1_floor_ = T1_TICKS;  // T1 floor based on TXDELAY (never below FRACK default)
    int t1_ = 0;       // Acknowledgment timer (T1)
    int t2_ = 0;       // Response delay timer (T2)
    int t3_ = 0;       // Idle supervision timer (T3)
    int retry_count_ = 0;  // RC — retry count

    // AX.25 2.2 state flags
    bool peer_busy_ = false;            // Remote station busy (sent RNR)
    bool own_busy_ = false;             // We are busy (would send RNR)
    bool reject_exception_ = false;     // REJ already sent, waiting for correct I-frame
    bool acknowledge_pending_ = false;  // Need to send acknowledgment (T2 driven)
    // C1 implicit-NAK fast-retransmit latch (native OFDM-KISS).  The native
    // responder emits an autonomous RR per received I-frame, so the commander
    // sees a STREAM of forward-advancing cumulative RRs — pipelined ACKs, NOT
    // NAKs.  We go-back-N only on a DUPLICATE RR pinned at V(A) (the responder
    // re-ACKing its stuck V(R) past a gap), at most ONCE per stuck point.  This
    // holds the N(R) we last retransmitted from; kC1NakNone (0xFF) = none.  Reset
    // on any forward progress.  Firing on every advancing RR (the pre-fix bug)
    // was a full-window retransmit STORM that stalled >8-frame transfers.
    static constexpr uint8_t kC1NakNone = 0xFF;
    uint8_t c1_nak_retx_from_ = kC1NakNone;
    // #3 desync backstop (reverse-ACK ROOT FIX).  The kiss-shadow V(A)-advance
    // consumers (S-frame RR/REJ + I-frame piggyback N(R)) used to SILENTLY ignore
    // an out-of-window N(R) from a validly-decoded peer ACK — the deadlock's final
    // leg (peer's honest N(R) rejected forever while V(A) stuck).  Count CONSECUTIVE
    // invalid N(R); SHADOW_DESYNC_THRESHOLD in a row => declare desync LOUD and
    // re-establish (shadow analog of the owned-path nr_error_recovery).  Reset on
    // ANY valid N(R).  With #1 (+#2) live-stamping this state is unreachable; the
    // backstop converts any residual into seconds lost, never session-dead.
    static constexpr int SHADOW_DESYNC_THRESHOLD = 3;
    int shadow_nr_invalid_streak_ = 0;
    // RR-volley guard (native-peer livelock).  enquiry_response() emits an F=1
    // RESPONSE; the native-shadow poll-answer answered ANY pf=1 S-frame — so an
    // F=1 RESPONSE (itself just such an answer) drew another F=1 RESPONSE, and two
    // native peers volleyed RR F=1 rsp at each other every turnaround forever,
    // starving T1 so the C1 selective-retransmit of the one lost frame never
    // re-fired (0 delivery, responder holding N(S)=2..7).  A genuine COMMAND poll
    // is ALWAYS answered, and an F=1 RESPONSE that advanced V(A) this frame is
    // ALWAYS answered (the legitimate fast turnaround that paces bidirectional
    // delivery — each answer's N(R) acks the peer's data, advancing ITS V(A)).
    // Only a no-progress RESPONSE echo repeated past this cap is the pathological
    // volley; suppress further auto-answers until progress resumes.  Counts
    // CONSECUTIVE no-progress response-answers; reset on progress or a command.
    // Kept small: healthy rounds carry progress and never increment it, so it only
    // ever bites a genuinely stuck/idle echo.
    static constexpr int kMaxNoProgressRespAnswers = 2;
    int rr_resp_noprogress_answers_ = 0;
    // #2 burst-epoch guard flag (set by the modem before each reverse ACK; see the
    // set_reverse_ack_epoch_ok() doc).  TRUE = destructive V(A) advance permitted
    // (epoch match or absent echo); FALSE = advisory-only on this reverse ACK.
    bool reverse_ack_epoch_ok_ = true;
    bool srej_enabled_ = false;         // SREJ supported (negotiated via XID)
    uint8_t last_received_ctrl_ = 0;    // For FRMR info field
    bool kiss_managed_ = false;          // Session initiated by KISS client (don't generate SABM/DISC retries)
    bool native_active_ = false;          // OFDM-KISS native mode: enable T1 polls even in KISS mode
    // TX physically deferred (auto-tune ramp owns the channel).  While true, the
    // held I-frames cannot be transmitted, so T1 must NOT run against them — it
    // would fire a spurious TIMER_RECOVERY and the ensuing go-back-N burst can
    // overflow the modem tx_queue_ and evict never-sent frames (P0 turnaround).
    // Driven by the modem from tune_state_; cleared on reset_session().
    bool tx_deferred_ = false;
    bool we_initiated_ = false;          // We sent SABM (true) vs received SABM (false)
    std::atomic<bool> kiss_passthrough_{false}; // KISS client active — don't accept incoming connections
    // Channel busy: pause T1/T3 while DCD or PTT active (Direwolf pattern).
    // Remaining ticks saved on pause, restored on resume.
    bool channel_busy_ = false;
    // Increment 1 (window-fill) RSP autonomous-RR HOLD-to-burst-end. rr_hold_ is
    // set by the modem from IRIS_BURST_FILL. rr_quiet_ticks_ counts consecutive
    // ticks the channel has been idle (channel_busy_ == false); the autonomous RR
    // is deferred while the initiator is on-air OR until the channel has been idle
    // for RR_HOLD_QUIET_TICKS (bridges the initiator's back-to-back re-key gap so
    // the held RR fires only after the burst sequence truly ends, into a clear slot).
    // T1's P=1 poll remains the backstop, so the hold can never deadlock the link.
    bool rr_hold_ = false;
    int  rr_quiet_ticks_ = 0;
    static constexpr int RR_HOLD_QUIET_TICKS = 8;   // 8 ticks ~= 0.4s idle guard
    int t1_paused_remaining_ = 0;       // T1 ticks remaining when paused (0 = not paused)
    int t3_paused_remaining_ = 0;       // T3 ticks remaining when paused

    // Adaptive T1 via Smoothed Round-Trip Time (Direwolf ax25_link.c:6304-6397).
    // Measures actual RTT from SABM→UA and poll→response, then adapts T1.
    // SRT update: first measurement SRT=RTT, subsequent SRT = 7/8*SRT + 1/8*RTT.
    // T1 = max(floor, 2*SRT).
    int wall_ticks_ = 0;                // Always-incrementing tick counter (not paused)
    int rtt_start_ = 0;                 // wall_ticks_ when measurement started
    bool tx_iframe_hold_ = false;       // INV-SEQ-2: hold owned I-frame TX during pending refrag
    int last_valid_nr_tick_ = -1;       // wall tick of the last APPLIED valid N(R)
    int last_ack_progress_tick_ = 0;    // wall tick of last V(A) advance / flight start
    bool measuring_rtt_ = false;        // RTT measurement in progress
    int srt_ticks_ = 0;                 // Smoothed RTT in ticks (0 = not yet measured)

    // T1 retry jitter: breaks synchronized collisions (both sides retrying at
    // exact same interval).  0-25% jitter desynchronizes after 1-2 retries.
    std::minstd_rand rng_{std::random_device{}()};
    int t1_with_jitter() { return t1_value_ + rng_() % (t1_value_ / 4 + 1); }

    // Addresses (built once per connection)
    Ax25Address local_addr_;
    Ax25Address remote_addr_;
    // Digipeater path for session-generated frames (see remote_via() doc).
    Ax25ViaPath remote_via_;
    // Set remote_via_ (originator: as-requested; responder: reversed) with all
    // H-bits cleared, and bump the PRE-MEASUREMENT T1 for the extra
    // store-and-forward hops (Direwolf INIT_T1V_SRT: FRACK × (2m+1), m =
    // number of digipeaters — ax25_link.c:358-364).  SRT is measured end to
    // end THROUGH the path (SABM→UA, poll→response), so once a real RTT
    // lands, select_t1_value() re-derives T1 and this default is superseded.
    void adopt_via_path(const Ax25ViaPath& via, bool reverse);

    // Thread safety: protects timer state (T1/T3/paused/SRT) from concurrent
    // access by audio callback thread (set_channel_busy, on_frame_received)
    // and main loop thread (tick).
    mutable std::mutex timer_mutex_;

    // Callbacks
    SendFrameFunc send_frame_;
    DataReceivedFunc data_received_;
    StateChangedFunc state_changed_;
    NativeStreamRxFunc native_stream_rx_;
    NativeStreamAdmissionFunc native_stream_admission_;
    AckedInfoFunc acked_info_;
    DesyncFunc desync_recovery_;
    // Changes whenever reset/re-establishment replaces the sequence epoch.  An
    // admission callback may reenter and replace the session, so callers must
    // re-check this token before advancing any old-frame sequence state.
    uint64_t session_generation_ = 0;
};

} // namespace iris

#endif // IRIS_AX25_SESSION_H
