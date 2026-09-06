#include "ax25/ax25_session.h"
#include "common/logging.h"
#include <algorithm>
#include <cstdio>

namespace iris {

Ax25Session::Ax25Session() = default;

// ---------------------------------------------------------------------------
// State management
// ---------------------------------------------------------------------------

void Ax25Session::set_state(Ax25SessionState s) {
    if (s == state_) return;
    previous_state_ = state_;
    previous_state_generation_ = state_generation_;
    state_ = s;
    state_generation_ = session_generation_;
    static const char* names[] = {
        "DISCONNECTED", "AWAITING_CONNECTION", "CONNECTED",
        "TIMER_RECOVERY", "AWAITING_RELEASE"
    };
    IRIS_LOG("AX25 state -> %s (remote=%s)", names[(int)s], remote_call_.c_str());
    if (state_changed_) state_changed_(s, remote_call_);
}

void Ax25Session::reset_session() {
    ++session_generation_;
    vs_ = vr_ = va_ = 0;
    t1_ = t2_ = t3_ = 0;
    retry_count_ = 0;
    n2_effective_ = N2;
    peer_busy_ = false;
    own_busy_ = false;
    reject_exception_ = false;
    acknowledge_pending_ = false;
    c1_nak_retx_from_ = kC1NakNone;
    shadow_nr_invalid_streak_ = 0;
    rr_resp_noprogress_answers_ = 0;
    last_received_ctrl_ = 0;
    kiss_managed_ = false;
    // Clear native OFDM-KISS mode: a prior native session must not leave native
    // T1/T2/ACK behavior (native T1 cap 6s vs AFSK 30s, ~18 if(native_active_)
    // consumer branches) armed for a subsequent legacy Direwolf/AFSK-TNC session
    // that never activates native. set_native_active(true) re-arms it on the next
    // peer OFDM decode. AX.25-interop hazard; never cleared before this.
    native_active_ = false;
    tx_iframe_hold_ = false;            // a fresh session never starts TX-held
    last_valid_nr_tick_ = -1;
    last_ack_progress_tick_ = wall_ticks_;   // old-epoch stamps must not leak forward
    tx_deferred_ = false;   // clear auto-tune TX-defer so a reconnect starts clean
    we_initiated_ = false;
    channel_busy_ = false;
    t1_paused_remaining_ = 0;
    t3_paused_remaining_ = 0;
    measuring_rtt_ = false;
    // Keep srt_ticks_ across reconnects — measured RTT is still valid for same path
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (int i = 0; i < SEQ_MOD_MAX; i++) tx_window_[i] = TxIFrame{};
    // Drop any buffered out-of-order RX frames from the prior session.
    for (int i = 0; i < SEQ_MOD_MAX; i++) {
        rx_reorder_buf_[i].clear();
        rx_reorder_present_[i] = false;
    }
    // A fresh session starts mod-8 (the connect handshake + client seam are mod-8);
    // the modem re-widens the OWNED transport at native activation (set_wide_window).
    seq_mod_ = 8;
    window_k_ = K;
    extended_ = false;
    // Drop the digipeater path — a fresh session is direct until a connect
    // request (ours or the peer's) carries one.
    remote_via_.clear();
}

void Ax25Session::adopt_via_path(const Ax25ViaPath& via, bool reverse) {
    // Session-frame reply/request path (see the remote_via() header doc).
    // ORIGINATOR (reverse=false): the request path in path order.
    // RESPONDER (reverse=true): the received SABM's path REVERSED — replies
    // traverse the digipeaters in reverse order.  Direwolf reverses exactly
    // this way for an incoming connection (get_link_handle, ax25_link.c:877-890:
    // "If it came in over the radio, we need to swap source/destination and
    // reverse any digi path").  Either way ALL H-bits are cleared: the frames
    // WE originate have not been repeated yet; each digi sets its own H-bit
    // (the on-air rule ax25_set_h implements, ../direwolf/src/ax25_pad.c:1579).
    remote_via_.clear();
    if (via.empty()) return;
    remote_via_.reserve(via.size());
    if (reverse) {
        for (auto it = via.rbegin(); it != via.rend(); ++it)
            remote_via_.push_back(Ax25ViaHop{it->addr, false});
    } else {
        for (const auto& hop : via)
            remote_via_.push_back(Ax25ViaHop{hop.addr, false});
    }
    // Pre-measurement T1: the default (T1_TICKS = 4s, Direwolf FRACK=4) covers
    // a DIRECT round-trip; each digipeater adds a full store-and-forward of the
    // frame in each direction.  Direwolf's client side initializes T1V to
    // FRACK × (2m+1), m = number of digipeaters (INIT_T1V_SRT,
    // ../direwolf/src/ax25_link.c:358-364) — do the same on top of whatever
    // TXDELAY floor is in force, capped at the 30 s AFSK ceiling
    // select_t1_value() enforces.  A measured SRT (which is end-to-end through
    // the path) supersedes this via select_t1_value(), so only the
    // not-yet-measured default is bumped.
    if (srt_ticks_ == 0) {
        int base = std::max(t1_value_, t1_floor_);
        int mult = 2 * (int)remote_via_.size() + 1;
        int bumped = (int)std::min((long long)base * mult, 600LL);
        if (bumped > t1_value_) {
            IRIS_LOG("AX25 T1 default bumped for %zu-hop digi path: %d -> %d ticks (%.1fs)",
                     remote_via_.size(), t1_value_, bumped, bumped * 0.05);
            t1_value_ = bumped;
        }
    }
    IRIS_LOG("AX25 session via path (%s): [%s]",
             reverse ? "responder, reversed" : "originator",
             ax25_via_to_string(remote_via_).c_str());
}

void Ax25Session::reset_sequence_epoch() {
    // See the header doc: new sequence epoch — counters AND every per-sequence
    // artifact of the old epoch go together.  (Pre-fix, the KISS re-establish
    // paths zeroed only vs_/vr_/va_ and the SABM link-reset cleared just 8 of the
    // SEQ_MOD_MAX tx_window_ slots — stale reorder-buffer entries from the old
    // epoch could then drain as data under new numbering after an FRMR/SABM.)
    ++session_generation_;
    vs_ = vr_ = va_ = 0;
    for (int i = 0; i < SEQ_MOD_MAX; i++) {
        tx_window_[i] = TxIFrame{};
        rx_reorder_buf_[i].clear();
        rx_reorder_present_[i] = false;
    }
    last_ack_progress_tick_ = wall_ticks_;
}

void Ax25Session::set_wide_window(bool enable, int k) {
    // Switch the OWNED R2 transport to AX.25 2.2 modulo-128 with a large window.
    // Called by the modem at native activation (both ends, same env) BEFORE any
    // owned I-frame flies.  vs_/vr_/va_ are all < 8 at this point (the connect
    // handshake carries no sequenced frames; terminate rolls the window to V(A)=0),
    // so the low values are identical in mod-8 and modulo-128 — the switch is seamless.
    if (enable) {
        seq_mod_ = SEQ_MOD_MAX;                                 // modulo-128
        window_k_ = std::max(K, std::min(k, SEQ_MOD_MAX - 1));  // clamp to [7, 127]
        extended_ = true;                                       // 2-octet owned I/S control
    } else {
        seq_mod_ = 8;
        window_k_ = K;
        extended_ = false;
    }
    IRIS_LOG("AX25 wide-window %s: seq_mod=%d K=%d extended=%d [V(A)=%d V(S)=%d V(R)=%d]",
             enable ? "ON" : "off", seq_mod_, window_k_, (int)extended_, va_, vs_, vr_);
}

void Ax25Session::reset() {
    reset_session();
    remote_call_.clear();
    set_state(Ax25SessionState::DISCONNECTED);
}

void Ax25Session::reset_silent() {
    reset_session();
    remote_call_.clear();
    previous_state_ = state_;
    previous_state_generation_ = state_generation_;
    state_ = Ax25SessionState::DISCONNECTED;
    state_generation_ = session_generation_;
}

void Ax25Session::clear_exception_conditions() {
    peer_busy_ = false;
    reject_exception_ = false;
    acknowledge_pending_ = false;
}

void Ax25Session::establish_data_link() {
    clear_exception_conditions();
    retry_count_ = 0;
    send_sabm();
    t1_ = t1_value_;
    t3_ = 0;  // Stop T3
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void Ax25Session::notify_outgoing(const uint8_t* frame, size_t len) {
    // Parse outgoing KISS frame to track session state.
    // The KISS client (e.g., Winlink) manages the connection — we just
    // shadow its state so we don't interfere (e.g., sending DM to a valid UA).
    Ax25Frame f;
    if (!ax25_parse(frame, len, f))
        return;

    // Only care about frames FROM our callsign
    if (!f.src.matches(local_call_))
        return;

    if (state_ != Ax25SessionState::DISCONNECTED && !f.dst.matches(remote_call_))
        return;

    if (f.type() == Ax25FrameType::U_FRAME) {
        Ax25UType ut = f.u_type();
        if (ut == Ax25UType::SABM &&
                   (state_ == Ax25SessionState::DISCONNECTED ||
                    state_ == Ax25SessionState::AWAITING_CONNECTION)) {
            // KISS client initiating connection — track it
            reset_session();
            remote_call_ = f.dst.to_string();
            local_addr_ = ax25_make_addr(local_call_);
            remote_addr_ = f.dst;
            retry_count_ = 0;
            kiss_managed_ = true;
            kiss_passthrough_ = true;  // KISS client active — forward incoming connections
            we_initiated_ = true;
            // Endpoint-through-digis (e.g. the Winlink "Digipeater" connection
            // type): adopt the client's requested path so every session-
            // generated frame carries it, and bump the pre-measurement T1 for
            // the extra hops BEFORE arming it below.  The client's own raw
            // frames already fly with the path verbatim.
            adopt_via_path(f.via, false);   // originator: as requested
            t1_ = t1_value_;
            t3_ = 0;
            set_state(Ax25SessionState::AWAITING_CONNECTION);
            IRIS_LOG("AX25 KISS SABM to %s%s%s — tracking session", remote_call_.c_str(),
                     remote_via_.empty() ? "" : " via ",
                     remote_via_.empty() ? "" : ax25_via_to_string(remote_via_).c_str());
        } else if (ut == Ax25UType::UA &&
                   state_ == Ax25SessionState::DISCONNECTED) {
            // KISS client responding to incoming SABM — responder path.
            // Track the session so connection header exchange works.
            remote_call_ = f.dst.to_string();
            local_addr_ = ax25_make_addr(local_call_);
            remote_addr_ = f.dst;
            // The client owns path reversal on this seam: a conformant client
            // answers a digipeated SABM with the path already reversed in its
            // UA, so adopt it AS BUILT (no second reversal) for the session-
            // generated frames that follow.
            adopt_via_path(f.via, false);
            retry_count_ = 0;
            kiss_managed_ = true;
            kiss_passthrough_ = true;
            we_initiated_ = false;  // We're the responder
            vs_ = vr_ = va_ = 0;
            t1_ = 0;
            t3_ = t3_value_;
            set_state(Ax25SessionState::CONNECTED);
            IRIS_LOG("AX25 KISS UA to %s — tracking as responder", remote_call_.c_str());
        } else if (ut == Ax25UType::DISC &&
                   (state_ == Ax25SessionState::CONNECTED ||
                    state_ == Ax25SessionState::TIMER_RECOVERY)) {
            t1_ = t1_value_;
            t3_ = 0;
            set_state(Ax25SessionState::AWAITING_RELEASE);
            IRIS_LOG("AX25 KISS DISC — tracking disconnect");
        }
    }

    // Shadow V(S): track outgoing I-frames from KISS client.
    // Only advance forward — KISS retransmissions (go-back-N) re-send lower
    // N(S) values which would shrink the nr_valid() window backwards,
    // allowing V(A) to decrement (bug: V(A) 0→7→6→5→...).
    if (f.type() == Ax25FrameType::I_FRAME && kiss_managed_) {
        uint8_t ns = f.ns();
        // Position of this frame within the send window [V(A), V(A)+K).
        // (Shadow/AFSK path — seq_mod_==8 here; terminate diverts client I-frames
        // to repack_tx_ingest BEFORE notify_outgoing, INV-2, so the wide owned
        // window is never advanced from here.  Parameterized for consistency.)
        uint8_t window_used = (uint8_t)seq_dist(vs_, va_);
        uint8_t pos = (uint8_t)seq_dist(ns, va_);  // forward distance of N(S) from V(A)
        // Shadow V(S): advance ONLY for a genuinely-new in-order frame, i.e.
        // N(S) == V(S) (the send pointer) AND the window has room.  Anchoring
        // the decision on V(S) — not on the mod-8 distance from V(A) — keeps
        // the update strictly monotonic.  The previous test scored candidates by
        // distance FROM V(A) (dist_new = (N(S)+1 - V(A)) & 7) and advanced when
        // that beat the old distance; a go-back-N retransmit whose N(S)+1 landed
        // just behind V(A) mapped to the maximal mod-8 distance (7) and dragged
        // V(S) BEHIND V(A), producing the impossible V(A) > V(S) state, a full
        // (window_used == 7) window that never drains, and a hard deadlock after
        // one K=7 window (send_next_iframe stuck at the window-full break).
        // A1 (P0.2): decide "is this a genuinely-new in-order frame?" and STORE
        // it BEFORE advancing V(S).  The eb85fa6 ordering advanced vs_ first,
        // then re-tested `ns == vs_` for the store — but vs_ had already moved
        // past ns, so a brand-new in-order frame (the common case: window empty,
        // ns == vs_, window_used == 0 → pos == 0, `pos < window_used` false)
        // was NEVER stored.  invoke_retransmission() then walked an empty
        // tx_window_ slot and native retransmit silently did nothing → lost
        // frames never recovered → 0 delivery on the KISS/shadow path
        // (the P0.2 failure).
        bool is_new_in_order = (ns == vs_ && window_used < window_k_);
        // Store the I-frame info field for retransmission.  Store only for
        // frames that live in the current send window [V(A), V(S)] (a new frame
        // at V(S) or an in-window retransmit) — never resurrect an already-ACKed
        // slot (a stale re-send has pos > window_used), which would leave a
        // phantom unacked frame in tx_window_ / pending_frames().
        bool in_window = (pos < window_used) || is_new_in_order;
        if (!f.info.empty() && in_window) {
            tx_window_[ns % seq_mod_].info = f.info;
            tx_window_[ns % seq_mod_].sent = true;  // already sent by KISS client
        }
        // Now advance shadow V(S) for the new in-order frame (monotonic; see
        // the go-back-N reasoning above).
        if (is_new_in_order) {
            vs_ = seq_inc(vs_);
        }
        // In native/OFDM mode, Iris owns the transport layer and must
        // supervise the link.  Start T1 so we can poll the peer if the
        // KISS client's frame doesn't get acknowledged.  Without this,
        // recovery depends entirely on the KISS client's retry logic
        // (typically exponential backoff — 35-75s per retry cycle).
        if (native_active_ && t1_ == 0 && va_ != vs_ && !tx_deferred_) {
            t1_ = t1_value_;
            t3_ = 0;
            IRIS_LOG("AX25 T1 started for KISS I-frame in native mode: %d ticks (%.1fs) [V(A)=%d V(S)=%d]",
                     t1_value_, t1_value_ * 0.05, va_, vs_);
        } else if (native_active_) {
            // Diagnostic: log why T1 was NOT started
            IRIS_LOG("AX25 notify_outgoing I-frame: T1 not started [native=%d t1=%d V(A)=%d V(S)=%d chan_busy=%d]",
                     (int)native_active_, t1_, va_, vs_, (int)channel_busy_);
        }
    }
}

void Ax25Session::connect(const std::string& remote_call) {
    // Accept "DST via DIGI1,DIGI2" (case-insensitive " via ", hops separated
    // by commas and/or spaces) — the standard TNC connect-through-digis form.
    // A bare callsign parses to an empty path (byte-identical historical connect).
    std::string dst = remote_call;
    Ax25ViaPath via;
    // Find a standalone " via " token (any case).
    for (size_t i = 0; i + 5 <= remote_call.size(); i++) {
        if (remote_call[i] == ' ' &&
            (remote_call[i + 1] == 'v' || remote_call[i + 1] == 'V') &&
            (remote_call[i + 2] == 'i' || remote_call[i + 2] == 'I') &&
            (remote_call[i + 3] == 'a' || remote_call[i + 3] == 'A') &&
            remote_call[i + 4] == ' ') {
            dst = remote_call.substr(0, i);
            std::string rest = remote_call.substr(i + 5);
            std::string hop;
            for (size_t j = 0; j <= rest.size(); j++) {
                char c = (j < rest.size()) ? rest[j] : ',';
                if (c == ',' || c == ' ') {
                    if (!hop.empty() && via.size() < 8)   // AX.25 2.2: <= 8 digis
                        via.push_back(Ax25ViaHop{ax25_make_addr(hop), false});
                    hop.clear();
                } else {
                    hop += c;
                }
            }
            break;
        }
    }
    connect(dst, via);
}

void Ax25Session::connect(const std::string& remote_call, const Ax25ViaPath& via) {
    reset_session();
    remote_call_ = remote_call;
    local_addr_ = ax25_make_addr(local_call_);
    remote_addr_ = ax25_make_addr(remote_call_);
    we_initiated_ = true;
    adopt_via_path(via, false);   // originator: path as requested, H-bits clear
    establish_data_link();
    set_state(Ax25SessionState::AWAITING_CONNECTION);
}

void Ax25Session::send_data(const uint8_t* data, size_t len) {
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;

    // Fragment into max_info_ chunks (limited by OFDM LDPC capacity)
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = std::min((size_t)max_info_, len - offset);
        tx_queue_.push(std::vector<uint8_t>(data + offset, data + offset + chunk));
        offset += chunk;
    }

    send_next_iframe();
}

void Ax25Session::disconnect() {
    if (state_ == Ax25SessionState::DISCONNECTED) return;

    // Discard pending I-frames
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (int i = 0; i < seq_mod_; i++) tx_window_[i] = TxIFrame{};

    retry_count_ = 0;
    send_disc();
    t1_ = t1_value_;
    t3_ = 0;
    set_state(Ax25SessionState::AWAITING_RELEASE);
}

int Ax25Session::pending_frames() const {
    int count = (int)tx_queue_.size();
    for (int i = 0; i < seq_mod_; i++) {
        if (!tx_window_[i].info.empty()) count++;
    }
    return count;
}

// ---------------------------------------------------------------------------
// N(R) validation (AX.25 2.2 Section 4.3.3.3)
// V(A) <= N(R) <= V(S) (modulo 8)
// ---------------------------------------------------------------------------

bool Ax25Session::nr_valid(uint8_t nr) const {
    // N(R) is valid if V(A) <= N(R) <= V(S) in the live modulus (mod-8 or modulo-128).
    nr = (uint8_t)(nr % seq_mod_);
    for (uint8_t i = va_; ; i = seq_inc(i)) {
        if (i == nr) return true;
        if (i == vs_) return false;
    }
}

// ---------------------------------------------------------------------------
// Frame building and sending
// ---------------------------------------------------------------------------

// Every session-generated frame below is built with remote_via_ (empty for a
// direct link -> byte-identical historical encoding).  The endpoints of a
// digipeated connection carry the FULL path on every frame of the session,
// H-bits clear as transmitted; the digis mark the H-bits on the air.

void Ax25Session::send_sabm() {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_SABM, true, true,
                              remote_via_);
    IRIS_LOG("AX25 TX SABM to %s (P=1)", remote_call_.c_str());
    if (send_frame_) send_frame_(frame.data(), frame.size());
    // Start RTT measurement (SABM → UA)
    rtt_start_ = wall_ticks_;
    measuring_rtt_ = true;
}

void Ax25Session::send_ua(bool pf) {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_UA, pf, false,
                              remote_via_);
    IRIS_LOG("AX25 TX UA to %s (F=%d)", remote_call_.c_str(), pf ? 1 : 0);
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_disc() {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_DISC, true, true,
                              remote_via_);
    IRIS_LOG("AX25 TX DISC to %s (P=1)", remote_call_.c_str());
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_dm(bool pf) {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_DM, pf, false,
                              remote_via_);
    IRIS_LOG("AX25 TX DM to %s (F=%d)",
             remote_call_.empty() ? "?" : remote_call_.c_str(), pf ? 1 : 0);
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_rr(bool pf, bool command) {
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::RR, vr_, pf, command, extended_,
                              remote_via_);
    IRIS_LOG("AX25 TX RR N(R)=%d %s=%d %s", vr_,
             command ? "P" : "F", pf ? 1 : 0,
             command ? "cmd" : "rsp");
    if (send_frame_) send_frame_(frame.data(), frame.size());
    // Start RTT measurement on poll (P=1 command → expect F=1 response)
    if (pf && command && !measuring_rtt_) {
        rtt_start_ = wall_ticks_;
        measuring_rtt_ = true;
    }
}

void Ax25Session::send_rnr(bool pf, bool command) {
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::RNR, vr_, pf, command, extended_,
                              remote_via_);
    IRIS_LOG("AX25 TX RNR N(R)=%d %s=%d %s", vr_,
             command ? "P" : "F", pf ? 1 : 0,
             command ? "cmd" : "rsp");
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_rej(bool pf, bool command) {
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::REJ, vr_, pf, command, extended_,
                              remote_via_);
    IRIS_LOG("AX25 TX REJ N(R)=%d %s=%d %s", vr_,
             command ? "P" : "F", pf ? 1 : 0,
             command ? "cmd" : "rsp");
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_frmr(uint8_t rejected_ctrl, uint8_t vs, uint8_t vr, bool cr,
                              bool w_bit, bool x_bit, bool y_bit, bool z_bit) {
    auto frame = ax25_build_frmr(remote_addr_, local_addr_,
                                  rejected_ctrl, vs, vr, cr, w_bit, x_bit, y_bit, z_bit,
                                  remote_via_);
    IRIS_LOG("AX25 TX FRMR ctrl=0x%02X W=%d X=%d Y=%d Z=%d",
             rejected_ctrl, w_bit, x_bit, y_bit, z_bit);
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::set_own_busy(bool busy) {
    if (own_busy_ == busy) return;
    own_busy_ = busy;
    IRIS_LOG("AX25 own_busy=%d", busy ? 1 : 0);
    // When becoming busy in connected state, send RNR immediately so the peer
    // stops sending I-frames.  When clearing busy, send RR to resume.
    if (state_ == Ax25SessionState::CONNECTED ||
        state_ == Ax25SessionState::TIMER_RECOVERY) {
        if (busy)
            send_rnr(false, false);
        else
            send_rr(false, false);
    }
}

void Ax25Session::request_retransmit() {
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;
    if (reject_exception_) return;  // REJ already pending
    reject_exception_ = true;
    send_rej(false, true);  // Command REJ, no poll bit
    acknowledge_pending_ = false;
}

void Ax25Session::start_t1_if_unacked() {
    if (!native_active_) return;
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;
    std::lock_guard<std::mutex> lock(timer_mutex_);
    // TX deferred by auto-tune: the held I-frames cannot go out until tune
    // completes (>30s), so arming an ACK timer against them would fire a
    // spurious TIMER_RECOVERY (P0 turnaround drop).  The post-tune first OFDM
    // burst re-arms T1 with the airtime floor.  (ax25_session.h set_tx_deferred.)
    if (tx_deferred_) return;
    if (t1_ == 0 && va_ != vs_) {
        t1_ = t1_value_;
        t3_ = 0;
        IRIS_LOG("AX25 T1 started (start_t1_if_unacked): %d ticks (%.1fs) [V(A)=%d V(S)=%d]",
                 t1_value_, t1_value_ * 0.05, va_, vs_);
    }
}

void Ax25Session::set_tx_deferred(bool deferred) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (deferred == tx_deferred_) return;   // idempotent
    tx_deferred_ = deferred;
    if (deferred) {
        // Auto-tune now owns the channel; the held I-frames physically cannot be
        // transmitted until tune completes (>30s).  Suspend T1 so it does not fire
        // a spurious TIMER_RECOVERY against un-sent data.  Clear BOTH the running
        // timer AND any channel-busy-paused remainder — otherwise set_channel_busy()
        // resume would re-arm the stale short T1 the moment PTT drops (the T1 the
        // fired at 26s in the P0 re-smoke was a channel-busy-paused 2.0s timer).
        // The post-tune first OFDM data burst re-arms T1 with the airtime floor
        // (set_t1_floor_for_airtime), which is the correct arming point.
        if (t1_ > 0 || t1_paused_remaining_ > 0) {
            IRIS_LOG("AX25 T1 suspended (TX deferred for auto-tune) [t1=%d paused=%d V(A)=%d V(S)=%d]",
                     t1_, t1_paused_remaining_, va_, vs_);
        }
        t1_ = 0;
        t1_paused_remaining_ = 0;
    } else {
        IRIS_LOG("AX25 TX no longer deferred (auto-tune done) — T1 re-arms on next OFDM burst");
        // Do NOT re-arm T1 here: the burst-build path arms it with the airtime
        // floor when the first post-tune frame is actually transmitted.
    }
}

void Ax25Session::rollback_unacked_window() {
    // See the header doc: discard the ABSORBED unacked window, roll V(S) -> V(A).
    for (uint8_t s = va_; s != vs_; s = seq_inc(s))
        tx_window_[s] = TxIFrame{};
    vs_ = va_;
    while (!tx_queue_.empty()) tx_queue_.pop();
    t1_ = 0;
    retry_count_ = 0;
    c1_nak_retx_from_ = kC1NakNone;
}

void Ax25Session::recover_unacked_stream(std::vector<uint8_t>& out) {
    // See the header doc: harvest the un-ACKed byte suffix of the owned stream (window
    // INFO in send order, then the queued-but-unwindowed frames in order), THEN roll
    // the window back to V(A).  The caller (re-pack demote re-fragment) prepends `out`
    // to its stream and re-drains at the new max_info, so no stream byte is dropped on
    // a mid-transfer demote (data-flow-terminate-repack.md INV-1/INV-3).
    for (uint8_t s = va_; s != vs_; s = seq_inc(s)) {
        auto& info = tx_window_[s].info;
        out.insert(out.end(), info.begin(), info.end());
    }
    while (!tx_queue_.empty()) {
        auto& fr = tx_queue_.front();
        out.insert(out.end(), fr.begin(), fr.end());
        tx_queue_.pop();
    }
    for (uint8_t s = va_; s != vs_; s = seq_inc(s))
        tx_window_[s] = TxIFrame{};
    vs_ = va_;
    t1_ = 0;
    retry_count_ = 0;
    c1_nak_retx_from_ = kC1NakNone;
}

void Ax25Session::send_next_iframe() {
    if (peer_busy_) return;
    // INV-SEQ-2 (data-flow-owned-shadow-seq.md): a pending demote re-fragment holds
    // ALL owned I-frame TX — fills and re-airs — through this single funnel.  The
    // window is about to be re-sliced; quiescing the data direction is what makes
    // the peer's next cumulative N(R) its FINAL V(R) for the epoch.  S-frames
    // (T1 polls, RRs) do not pass through here and keep flowing.
    if (tx_iframe_hold_) return;
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;

    // Fill window from queue
    while (!tx_queue_.empty()) {
        int window_used = seq_dist(vs_, va_);
        if (window_used >= window_k_) break;
        if (window_used == 0)
            last_ack_progress_tick_ = wall_ticks_;   // flight starts: ACK clock rebases

        int slot = vs_;
        tx_window_[slot].info = std::move(tx_queue_.front());
        tx_queue_.pop();
        tx_window_[slot].sent = false;
        vs_ = seq_inc(vs_);
    }

    // Send unsent frames in window
    for (uint8_t seq = va_; seq != vs_; seq = seq_inc(seq)) {
        int slot = seq;
        if (!tx_window_[slot].sent && !tx_window_[slot].info.empty()) {
            auto frame = ax25_build_i(remote_addr_, local_addr_,
                                       seq, vr_, false, AX25_PID_NONE,
                                       tx_window_[slot].info.data(),
                                       tx_window_[slot].info.size(), extended_,
                                       remote_via_);
            IRIS_LOG("AX25 TX I N(S)=%d N(R)=%d (%zu bytes) [V(A)=%d V(S)=%d]",
                     seq, vr_, tx_window_[slot].info.size(), va_, vs_);
            if (send_frame_) send_frame_(frame.data(), frame.size());
            tx_window_[slot].sent = true;
            acknowledge_pending_ = false;  // Piggyback ack on I-frame
            if (t1_ == 0)
                t1_ = t1_value_;  // Start T1 if not running
            t3_ = 0;  // Stop T3 while transmitting
        }
    }
}

void Ax25Session::ack_frames(uint8_t nr) {
    // Acknowledge all frames with seq < nr (in the live modulus).
    if (va_ != nr)
        last_ack_progress_tick_ = wall_ticks_;   // INV-SEQ-3: the futility ACK clock
    while (va_ != nr) {
        int slot = va_;
        if (!tx_window_[slot].info.empty()) {
            IRIS_LOG("AX25 ACK frame N(S)=%d", va_);
            if (acked_info_)
                acked_info_(tx_window_[slot].info.data(),
                            tx_window_[slot].info.size());
        }
        tx_window_[slot] = TxIFrame{};
        va_ = seq_inc(va_);
    }

    if (va_ == vs_) {
        t1_ = 0;           // All acknowledged — stop T1
        t1_paused_remaining_ = 0;
        t3_ = t3_value_;    // Start idle supervision
    } else {
        t1_ = t1_value_;    // Restart T1 for remaining frames
    }
    retry_count_ = 0;
}

void Ax25Session::inject_rx_rr(uint8_t nr, bool pf) {
    // Build a properly addressed RR frame and feed through the state machine.
    // Source = peer (remote_addr_), Dest = us (local_addr_), response (not command).
    // Deliberately built WITHOUT remote_via_: this synthetic frame never touches
    // the air (it is parsed straight back into on_frame_received, which ignores
    // the via list), and the MFSK tone it stands in for is native-mode only —
    // a digipeated (AFSK) session never reaches this path.  Same for
    // build_rx_rr() below (KISS-side delivery of the same synthetic RR).
    // In wide mode the MFSK N(R) is 7-bit, so build+parse EXTENDED so the round-trip
    // preserves the full N(R) (a mod-8 build would alias a wide N(R) to its low 3 bits
    // -> V(A) advances only within an 8-frame window and the wide window stalls).
    auto frame_bytes = ax25_build_s(local_addr_, remote_addr_, Ax25SType::RR, nr, pf, false, extended_);
    Ax25Frame frame;
    if (ax25_parse(frame_bytes.data(), frame_bytes.size(), frame, extended_))
        on_frame_received(frame);
}

std::vector<uint8_t> Ax25Session::build_rx_rr(uint8_t nr, bool pf) {
    return ax25_build_s(local_addr_, remote_addr_, Ax25SType::RR, nr, pf, false);
}

void Ax25Session::retransmit_from(uint8_t nr) {
    // Mark frames from nr to vs_ as unsent for retransmission
    for (uint8_t seq = nr; seq != vs_; seq = seq_inc(seq))
        tx_window_[seq].sent = false;
    send_next_iframe();
}

void Ax25Session::retransmit_one(uint8_t seq) {
    // Selective single-frame retransmit — the cheap miss.  Re-send ONLY `seq`
    // (the known-lost frame at V(A)); leave every other window slot marked sent
    // so send_next_iframe() re-airs just this frame (plus any brand-new queued
    // frames, which is forward progress).  With the reordering RX buffer, `seq`
    // is the ONLY frame the receiver is missing — go-back-N here would re-send
    // the frames it has already buffered (the loss amplifier).  Multi-loss
    // converges one-gap-per-RTT via successive cumulative-RR jumps.
    int slot = seq;
    if (!tx_window_[slot].info.empty()) {
        tx_window_[slot].sent = false;
        send_next_iframe();
    }
}

bool Ax25Session::in_window(uint8_t ns) const {
    for (int i = 0; i < window_k_; i++) {
        if (seq_add(vr_, i) == ns) return true;
    }
    return false;
}

void Ax25Session::invoke_retransmission() {
    // Retransmit all unacknowledged I-frames (from V(A))
    retransmit_from(va_);
}

void Ax25Session::nr_error_recovery() {
    // N(R) error — send FRMR with Z bit (invalid N(R))
    IRIS_LOG("AX25 N(R) error — invalid N(R) received");
    // F1 (DATALINK_INTEGRITY_AUDIT §4): in the native/re-pack transport the AX.25
    // FRMR+SABM re-establish is a DEAD END — the kiss-managed CONNECTED peer ignores
    // SABM/FRMR (only DISC/DM reach a reset), the OFDM pump SUPPRESSES the re-
    // establish SABM, T1 never re-sends a session-originated SABM, and even a
    // successful UA discards un-ACKed custody without harvesting it. The pre-fix path
    // therefore burned ~28 s in a zombie AWAITING_CONNECTION and then lost custody-
    // ACKed client data silently/loudly (s00: 91,288 B). Hand the desync to the owner
    // (the modem) FIRST to stage the terminal cause. We then destroy and latch the
    // old session before the DISCONNECTED state callback publishes custody failure
    // — DISC the local pump and surface every undelivered byte — so callback reentry
    // cannot be erased by a later reset. Only when the owner declines do we run
    // standard FRMR+SABM recovery.
    if (desync_recovery_ && desync_recovery_()) {
        reset_session();
        set_state(Ax25SessionState::DISCONNECTED);
        // DISCONNECTED is the sole completion hook.  Calling a second hook here
        // would run after its terminal callbacks and could terminate custody
        // opened reentrantly by one of those callbacks.
        return;
    }
    send_frmr(last_received_ctrl_, vs_, vr_, false,
              false, false, false, true);
    // Per AX.25 2.2: establish data link
    establish_data_link();
    set_state(Ax25SessionState::AWAITING_CONNECTION);
}

void Ax25Session::enquiry_response(bool pf) {
    // Respond to a P=1 poll (Section 6.2)
    if (own_busy_) {
        send_rnr(pf, false);  // Response
    } else {
        send_rr(pf, false);  // Response
    }
    acknowledge_pending_ = false;
}

void Ax25Session::check_iframe_queued() {
    // If I-frames pending, try to send
    if (!tx_queue_.empty() || va_ != vs_)
        send_next_iframe();
}

void Ax25Session::check_need_for_response(bool pf) {
    // If P/F=1 (poll), must respond
    if (pf) {
        enquiry_response(true);
    } else if (acknowledge_pending_) {
        // T2 will handle delayed ack
    }
}

// ---------------------------------------------------------------------------
// Frame receiving — AX.25 2.2 state machine
// ---------------------------------------------------------------------------

bool Ax25Session::on_frame_received(const Ax25Frame& frame) {
    // Check if addressed to us
    if (!frame.dst.matches(local_call_))
        return false;

    last_received_ctrl_ = frame.control;

    Ax25FrameType ft = frame.type();

    if (state_ != Ax25SessionState::DISCONNECTED &&
        !frame.src.matches(remote_call_))
        return false;

    // KISS-managed sessions: the KISS client handles the full AX.25 session.
    // We only track state transitions (UA→CONNECTED, DISC→DISCONNECTED) and
    // never generate frames or consume I/S frames.
    if (kiss_managed_) {
        if (state_ == Ax25SessionState::AWAITING_CONNECTION) {
            if (ft == Ax25FrameType::U_FRAME) {
                Ax25UType ut = frame.u_type();
                if (ut == Ax25UType::UA) {
                    IRIS_LOG("AX25 KISS RX UA from %s -> CONNECTED", frame.src.to_string().c_str());
                    t1_ = 0;
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 RTT measurement: %d ticks (%.1fs) [KISS SABM->UA]", rtt, rtt * 0.05);
                        update_srt(rtt);
                    }
                    t3_ = t3_value_;
                    reset_sequence_epoch();
                    set_state(Ax25SessionState::CONNECTED);
                } else if (ut == Ax25UType::SABM) {
                    // Simultaneous connect: remote also sent SABM.
                    // KISS client will generate UA — we just track the state.
                    IRIS_LOG("AX25 KISS RX SABM from %s [simultaneous connect -> CONNECTED]",
                             frame.src.to_string().c_str());
                    t1_ = 0;
                    t3_ = t3_value_;
                    reset_sequence_epoch();
                    set_state(Ax25SessionState::CONNECTED);
                } else if (ut == Ax25UType::DM) {
                    IRIS_LOG("AX25 KISS RX DM from %s -> DISCONNECTED", frame.src.to_string().c_str());
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
            }
        } else if (state_ == Ax25SessionState::CONNECTED ||
                   state_ == Ax25SessionState::TIMER_RECOVERY) {
            if (ft == Ax25FrameType::U_FRAME) {
                Ax25UType ut = frame.u_type();
                if (ut == Ax25UType::DISC) {
                    IRIS_LOG("AX25 KISS RX DISC from %s -> DISCONNECTED", frame.src.to_string().c_str());
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                } else if (ut == Ax25UType::DM) {
                    IRIS_LOG("AX25 KISS RX DM from %s -> DISCONNECTED", frame.src.to_string().c_str());
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
            }
            // Shadow V(R): track incoming I-frames for autonomous RR generation.
            // (#4 hygiene: removed the stale cc54fbb rationale that described an
            // advance-V(R)-to-max(V(R),N(S)+1) scheme — that advance-past-out-of-
            // sequence behavior was REVERTED as a data-loss bug and the code below
            // is strict in-sequence; the accurate rationale lives at the next block.)
            if (ft == Ax25FrameType::I_FRAME) {
                uint8_t ns = frame.ns();
                // Shadow V(R): advance STRICTLY in-sequence (N(S) == V(R) ->
                // V(R)+1), mirroring the owned RX path (:987).  This is monotonic
                // and correct across the mod-8 window boundary (V(R) 7 -> 0).
                // The previous guard accepted any N(S) with fwd_from_vr <= K;
                // with K=7 that spanned the ENTIRE ring, so a go-back-N
                // retransmit of an already-delivered low N(S) (e.g. V(R)=7,
                // N(S)=2 -> next=3) dragged V(R) BACKWARD and emitted a bogus
                // N(R) that falsely ACKed never-received frames (data loss).
                // SELECTIVE REPEAT (native OFDM-KISS cheap-miss): an out-of-sequence
                // forward frame within the window is BUFFERED (below), not dropped, so
                // the sender never has to re-send a frame that already flew.  A
                // strictly-behind / duplicate N(S) is still ignored (stale go-back-N /
                // client retransmit) — buffering only forward, in-window sequences.
                auto deliver_native_stream = [&](const std::vector<uint8_t>& info) {
                    // Feed the in-order INFO to the terminate/re-pack stream reassembler.
                    // Called ONLY as V(R) advances (the sole strictly-in-order accept),
                    // so the reassembler shares V(R) as its dedup/order gate (M0 one
                    // owned sequence) — a dup/reorder never advances V(R) and so never
                    // reaches the reassembler. Every nonempty in-order transport
                    // fragment reaches admission; its contents are never metadata.
                    if (native_active_ &&
                        (native_stream_admission_ || native_stream_rx_) &&
                        !info.empty()) {
                        if (native_stream_admission_)
                            return native_stream_admission_(info.data(), info.size());
                        native_stream_rx_(info.data(), info.size());
                    }
                    return true;
                };
                if (ns == vr_) {
                    const uint64_t admission_generation = session_generation_;
                    uint8_t next = seq_inc(vr_);
                    IRIS_LOG("AX25 KISS V(R) %d -> %d (I-frame N(S)=%d, %zu bytes)",
                             vr_, next, ns, frame.info.size());
                    const bool admitted = deliver_native_stream(frame.info);
                    if (session_generation_ != admission_generation)
                        return true;
                    if (!admitted) {
                        // The modem did not accept custody.  Keep V(R) pinned so
                        // no RR can acknowledge these bytes and advertise RNR.
                        set_own_busy(true);
                        acknowledge_pending_ = false;
                        return true;
                    }
                    vr_ = next;
                    // DRAIN: now that the gap filled, deliver every contiguous buffered
                    // out-of-order frame in strict V(R) order.  V(R) jumps forward past
                    // all already-arrived frames, so the next autonomous cumulative RR
                    // acks them and the sender never re-airs a delivered frame.
                    if (native_active_) {
                        while (rx_reorder_present_[vr_]) {
                            const uint64_t drain_generation = session_generation_;
                            std::vector<uint8_t> binfo = std::move(rx_reorder_buf_[vr_]);
                            rx_reorder_buf_[vr_].clear();
                            rx_reorder_present_[vr_] = false;
                            IRIS_LOG("AX25 KISS reorder-drain N(S)=%d -> V(R)=%d (%zu bytes)",
                                     vr_, seq_inc(vr_), binfo.size());
                            const bool admitted = deliver_native_stream(binfo);
                            if (session_generation_ != drain_generation)
                                return true;
                            if (!admitted) {
                                // Retain the exact buffered frame in its sequence
                                // slot.  A later capacity release resumes at V(R).
                                rx_reorder_buf_[vr_] = std::move(binfo);
                                rx_reorder_present_[vr_] = true;
                                set_own_busy(true);
                                break;
                            }
                            vr_ = seq_inc(vr_);
                        }
                    }
                } else if (native_active_ && in_window(ns)) {
                    // Forward, in-window, out-of-sequence: BUFFER instead of dropping
                    // (native selective-repeat cheap-miss).
                    // F2 leg (b) (DATALINK_INTEGRITY_AUDIT §4): if the slot already
                    // holds a frame, a NEW frame with DIFFERENT content is a RE-SLICED
                    // replacement — the sender demoted MAX_INFO on a REJ and re-
                    // fragmented the un-ACKed window at a smaller size, re-using these
                    // same N(S) numbers with new byte boundaries. The stale old-slicing
                    // copy MUST NOT win: draining it after the gap fills would splice a
                    // mismatched byte boundary into the [len] reassembler (custody
                    // teardown, or a chance-valid bogus record = silent corruption).
                    // The sender-side deferred re-fragment quiesce (INV-SEQ-2,
                    // modem.cc consume_pending_window_refrag) waits for the peer's demod
                    // pipeline to drain BEFORE it re-airs the re-slice, so the newer
                    // frame here is always the authoritative one — OVERWRITE. Identical
                    // content (a benign duplicate / go-back-N re-air at the same size) is
                    // a no-op; only a true re-slice mutates the buffer.
                    if (!rx_reorder_present_[ns] || rx_reorder_buf_[ns] != frame.info) {
                        bool replaced = rx_reorder_present_[ns];
                        rx_reorder_buf_[ns] = frame.info;
                        rx_reorder_present_[ns] = true;
                        IRIS_LOG("AX25 KISS reorder-%s N(S)=%d (V(R)=%d, %zu bytes)",
                                 replaced ? "replace" : "buffer",
                                 ns, vr_, frame.info.size());
                    }
                }
                // Shadow V(A) from I-frame N(R) (peer piggyback ack).
                // A5c: advance V(A) via the SAME path the
                // S-frame RR uses (:616) — ack_frames() in native mode — so the
                // piggybacked ack CLEANS tx_window_ and resets T1/retry_count.
                // The old code set va_=nr directly, which advanced V(A) but left
                // the acked slots [old V(A), N(R)) occupied in tx_window_ →
                // pending_frames() over-counted phantom unacked frames and the
                // next invoke_retransmission() walked already-acked slots.  When
                // the reverse ACK rides a piggybacked I-frame (responder has
                // reverse data queued), this is the ONLY V(A)-advance path CMD
                // sees, so the leak stalled the multi-window transfer.
                {
                    uint8_t nr = frame.nr();
                    uint8_t fwd_dist = (uint8_t)seq_dist(nr, va_);
                    // INV-SEQ-1 (data-flow-owned-shadow-seq.md §5): a cumulative
                    // N(R) is validated against the OUTSTANDING SPAN [V(A), V(S)]
                    // (nr_valid) and NEVER against window_k_ — K caps NEW
                    // transmissions, it says nothing about what the peer already
                    // counted.  The old `fwd_dist <= window_k_` conjunct refused
                    // the peer's honest in-flight ACK after a demote shrank K
                    // (63 -> 7) under a wide flight, froze V(A), and set up the
                    // V(S)-rollback sequence crossover behind the 3-invalid-N(R)
                    // desync teardown at speed.
                    if (nr_valid(nr) && nr != va_ && fwd_dist > 0) {
                        if (native_active_ && !reverse_ack_epoch_ok_) {
                            IRIS_LOG("AX25 KISS shadow V(A) HOLD (epoch mismatch): "
                                     "would-be %d -> %d (I-frame N(R)) — advisory",
                                     va_, nr);
                        } else {
                            IRIS_LOG("AX25 KISS shadow V(A) %d -> %d "
                                     "(I-frame N(R), fwd=%d)", va_, nr, fwd_dist);
                            if (native_active_)
                                ack_frames(nr);  // clean tx_window_, reset T1/retry
                            else
                                va_ = nr;
                        }
                    }
                    // #3 backstop: any valid piggyback N(R) is a live ACK from the
                    // peer — reset the desync streak (the desync TRIGGER lives on the
                    // S-frame/RR carrier, the deadlock's final leg).
                    if (nr_valid(nr)) {
                        shadow_nr_invalid_streak_ = 0;
                        if (!native_active_ || reverse_ack_epoch_ok_ || nr == va_)
                            last_valid_nr_tick_ = wall_ticks_;
                    } else if (native_active_ &&
                               ++shadow_nr_invalid_streak_ >= SHADOW_DESYNC_THRESHOLD) {
                        IRIS_LOG("AX25 KISS DESYNC: %d consecutive invalid N(R) on "
                                 "shadow I-frame path [N(R)=%d V(A)=%d V(S)=%d] -> "
                                 "re-establish data link", shadow_nr_invalid_streak_,
                                 nr, va_, vs_);
                        shadow_nr_invalid_streak_ = 0;
                        nr_error_recovery();
                        return false;
                    }
                }
                // Native OFDM-KISS: generate autonomous RR after receiving I-frames.
                // In KISS-managed mode the TNC normally doesn't generate S-frames,
                // but in native mode WE are the transport — the KISS client has no
                // way to generate RR over the OFDM PHY. Without this, CMD must wait
                // for T1 timeout (2s) before polling, halving throughput.
                if (native_active_) {
                    acknowledge_pending_ = true;
                    if (t2_ == 0) t2_ = 1;  // 50ms delay (native mode)
                }
                // Check for connection header (but don't deliver normal data —
                // KISS gets the raw frame via dispatch_rx_frame/rx_callback)
                if (!frame.info.empty() && data_received_ &&
                    frame.info.size() >= 5 &&
                    frame.info[0] == 'I' && frame.info[1] == 'R' &&
                    frame.info[2] == 'I' && frame.info[3] == 'S' &&
                    frame.info[4] == '/') {
                    data_received_(frame.info.data(), frame.info.size());
                }
            }
            // Shadow V(A): track peer acknowledgments via S-frame N(R).
            // Without this, V(A) stays at 0 forever in KISS-managed mode,
            // causing nr_valid() failures and permanent TIMER_RECOVERY.
            if (ft == Ax25FrameType::S_FRAME) {
                uint8_t nr = frame.nr();
                bool pf = frame.poll_final();
                Ax25SType st = frame.s_type();
                uint8_t kiss_prev_va = va_;
                    // Only advance V(A) forward — reject backwards/stale N(R).
                    // Use ack_frames() in native-active mode since it correctly
                    // handles tx_window_ cleanup. Shadow-only for pure KISS.
                {
                    uint8_t fwd_dist = (uint8_t)seq_dist(nr, va_);
                    // INV-SEQ-1: validate against the outstanding span (nr_valid),
                    // never window_k_ — see the I-frame piggyback twin above.  This
                    // is the exact branch that silently refused RR N(R)=81 at
                    // fwd_dist=24 > K=7 in the s00/s02 desync specimens.
                    if (nr_valid(nr) && nr != va_ && fwd_dist > 0) {
                        if (native_active_ && !reverse_ack_epoch_ok_) {
                            // #2 EPOCH MISMATCH: this reverse ACK echoed a burst-epoch
                            // that does NOT match the burst we are waiting on -> a
                            // stale/buffered tone.  Destructively advancing V(A) on it
                            // is the aliasing wedge itself.  ADVISORY only: hold V(A),
                            // let T1/C1 re-drive (the cheap miss direction — one retx
                            // round).  A valid-but-epoch-stale ACK is NOT a desync (the
                            // peer is alive) -> reset the streak.
                            IRIS_LOG("AX25 KISS shadow V(A) HOLD (epoch mismatch): "
                                     "would-be %d -> %d (%s N(R)) — advisory", va_, nr,
                                     st == Ax25SType::RR ? "RR" :
                                     st == Ax25SType::RNR ? "RNR" : "REJ");
                            shadow_nr_invalid_streak_ = 0;
                        } else {
                            IRIS_LOG("AX25 KISS shadow V(A) %d -> %d (%s N(R))",
                                     va_, nr,
                                     st == Ax25SType::RR ? "RR" :
                                     st == Ax25SType::RNR ? "RNR" : "REJ");
                            if (native_active_)
                                ack_frames(nr);  // Clean tx_window_ and update va_
                            else
                                va_ = nr;
                            shadow_nr_invalid_streak_ = 0;   // valid, in-window ACK
                            last_valid_nr_tick_ = wall_ticks_;   // INV-SEQ-2 witness
                        }
                    } else if (nr_valid(nr)) {
                        // Valid N(R) but no advance (duplicate/boundary ACK) — healthy.
                        shadow_nr_invalid_streak_ = 0;
                        last_valid_nr_tick_ = wall_ticks_;   // INV-SEQ-2 witness
                    } else {
                        // #3 DESYNC BACKSTOP: a validly-decoded peer S-frame carried an
                        // OUT-OF-WINDOW N(R) (V(A) <= N(R) <= V(S) violated).  Pre-fix this
                        // was silently dropped here — the deadlock's final leg.  Count
                        // consecutive invalids; the threshold in a row => declare desync
                        // LOUD and re-establish (shadow analog of nr_error_recovery).
                        if (native_active_ &&
                            ++shadow_nr_invalid_streak_ >= SHADOW_DESYNC_THRESHOLD) {
                            IRIS_LOG("AX25 KISS DESYNC: %d consecutive invalid N(R) on "
                                     "shadow S-frame path [N(R)=%d V(A)=%d V(S)=%d] -> "
                                     "re-establish data link", shadow_nr_invalid_streak_,
                                     nr, va_, vs_);
                            shadow_nr_invalid_streak_ = 0;
                            nr_error_recovery();
                            return false;  // session re-established; don't run C1/exit
                        }
                    }
                }
                // Track peer busy state
                if (st == Ax25SType::RNR)
                    peer_busy_ = true;
                else if (st == Ax25SType::RR || st == Ax25SType::REJ)
                    peer_busy_ = false;
                // C1 implicit NAK (RX package) — CORRECTED (P0 root cause).
                // The pre-fix predicate fired go-back-N on EVERY forward-advancing
                // native RR (`va_ != kiss_prev_va`), on the false premise that "the
                // peer heard our ENTIRE burst before it ACKed, so a gap = real
                // loss."  That premise is FALSE for this modem: the native RESPONDER
                // emits an autonomous RR after EACH received I-frame (:603-606), so
                // the commander receives a STREAM of forward-advancing cumulative
                // RRs (N(R)=1,2,3,...).  A forward-advancing cumulative RR is a
                // normal pipelined ACK, NOT a NAK — firing on each turned a healthy
                // ACK stream into a full-window retransmit STORM (~21-27 spurious
                // resends/window) that overflowed the modem tx_queue_ (32-cap,
                // drop-oldest) and starved the mid-window frames into an undecodable
                // burst → the >8-frame / K=7 window-wrap DROP + shadow-V(R) STALL.
                //
                // The genuine loss signal on THIS half-duplex link is a DUPLICATE
                // (non-advancing) cumulative RR pinned at V(A) while frames are
                // still outstanding (V(A) != V(S)): the responder re-ACKs its STUCK
                // V(R) once per out-of-order frame it receives past the gap, so a
                // repeated N(R)==V(A) means "the frame at V(A) was lost."  Go-back-N
                // from V(A) exactly ONCE per stuck point (latch on V(A); reset on any
                // forward progress) — the TCP-style dup-ACK fast retransmit.  This
                // preserves the datalink-tax win (no T1 wait on real loss) WITHOUT
                // the storm.  An explicit REJ (request_retransmit() / decode-fail) is
                // a true NAK and fires immediately.  RNR (peer_busy_) excluded; T1
                // backstops a totally-lost / repeatedly-lost burst.  This is the
                // exact path inject_rx_rr() / the MFSK tone-adopt site
                // (modem.cc:1702) feeds, so the fix is centralized.
                //
                // ACTIVE IN BOTH CONNECTED AND TIMER_RECOVERY (P0 connect-recovery
                // turnaround fix).  The reverse ACK on this half-duplex link is an
                // MFSK tone that reliably carries N(R) but NOT the F(inal) bit
                // (modem.cc mfsk_ack), so the classic TIMER_RECOVERY poll -> F=1
                // response cycle NEVER completes: a stuck receiver re-sends
                // RR N(R)==V(A) with PF=0 forever.  Gating the fast-retransmit to
                // CONNECTED meant that once T1 expired into TIMER_RECOVERY the
                // session could recover ONLY via that dead F=1 path -> permanent
                // wedge (ics213 stalled at V(A)=3; the CMD polled but NEVER
                // retransmitted the missing frame).  The duplicate-stuck-RR is the
                // genuine loss signal in EITHER state, so honor it in both; the T1
                // handler re-arms the latch (kC1NakNone) each recovery cycle so a
                // repeatedly-lost frame can re-fire.  On a fire from TIMER_RECOVERY
                // we return to CONNECTED (below) — the fast-retransmit IS the
                // recovery, so the dead F=1 exit path is bypassed.
                if (native_active_ && !peer_busy_ &&
                    (state_ == Ax25SessionState::CONNECTED ||
                     state_ == Ax25SessionState::TIMER_RECOVERY)) {
                    bool advanced = (va_ != kiss_prev_va);
                    bool c1_fired_retx = false;
                    if (st == Ax25SType::REJ && va_ != vs_) {
                        // Explicit reject = true NAK. REJ acks < N(R) via ack_frames
                        // (V(A) may advance to N(R)), then requests N(R)=V(A): go-
                        // back-N from V(A) now, regardless of advance.
                        IRIS_LOG("AX25 C1 explicit NAK: REJ N(R)=%d at V(A)=%d, "
                                 "%d outstanding — go-back-N", nr, va_,
                                 seq_dist(vs_, va_));
                        c1_nak_retx_from_ = va_;
                        invoke_retransmission();
                        c1_fired_retx = true;
                    } else if (advanced) {
                        // Forward-advancing cumulative RR = pipelined ACK, NOT a
                        // NAK.  Clear the latch so a genuine future stall re-arms;
                        // do NOT retransmit (this kills the storm).
                        c1_nak_retx_from_ = kC1NakNone;
                    } else if (st == Ax25SType::RR && nr == va_ && va_ != vs_ &&
                               c1_nak_retx_from_ != va_) {
                        // Duplicate cumulative RR pinned at V(A) with frames
                        // outstanding: the responder is STUCK waiting for V(A) → the
                        // frame at V(A) was lost.  Fast-retransmit ONCE per stuck
                        // point (latch); repeated dups won't re-fire.
                        //
                        // CHEAP MISS (WGN:30 loss-amplifier fix): re-send ONLY the
                        // frame at V(A), NOT the whole window (go-back-N).  The
                        // reordering RX buffer holds every OTHER frame the responder
                        // already received past this gap, so go-back-N would re-air
                        // frames that already flew — the amplifier.  Successive stuck
                        // points (multi-loss) each fire their own single retransmit as
                        // the cumulative RR jumps forward on gap-fill; T1 stays the
                        // backstop for a fully-lost burst (still go-back-N there).
                        IRIS_LOG("AX25 C1 implicit NAK: duplicate RR N(R)=%d stuck "
                                 "at V(A)=%d, %d outstanding — selective retx",
                                 nr, va_, seq_dist(vs_, va_));
                        c1_nak_retx_from_ = va_;
                        retransmit_one(va_);
                        c1_fired_retx = true;
                    }
                    // A fast-retransmit fired FROM TIMER_RECOVERY: that IS our
                    // recovery on this MFSK-N(R) reverse-ACK link (the F=1 exit path
                    // below is dead for tone ACKs).  Return to CONNECTED so T1/retry
                    // track a live transfer and the next advancing RR drains
                    // normally — instead of the exit block re-sending the whole
                    // window a second time (extra duplicates for the RX pump).
                    if (c1_fired_retx &&
                        state_ == Ax25SessionState::TIMER_RECOVERY) {
                        select_t1_value();
                        retry_count_ = 0;
                        set_state(Ax25SessionState::CONNECTED);
                        IRIS_LOG("AX25 KISS TIMER_RECOVERY -> CONNECTED "
                                 "(C1 fast-retransmit, N(R)=%d)", nr);
                    }
                }
                // Exit TIMER_RECOVERY on: F=1 poll response, OR unsolicited RR
                // that advanced V(A) in native mode (peer's autonomous T2 RR
                // arrives before our T1 poll — don't waste another T1 cycle).
                if (state_ == Ax25SessionState::TIMER_RECOVERY &&
                    (pf || (native_active_ && va_ != kiss_prev_va))) {
                    if (pf) {
                        t1_ = 0;
                        if (measuring_rtt_) {
                            int rtt = wall_ticks_ - rtt_start_;
                            IRIS_LOG("AX25 KISS RTT: %d ticks (%.1fs) [poll->%s]",
                                     rtt, rtt * 0.05,
                                     st == Ax25SType::RR ? "RR" : "RNR");
                            update_srt(rtt);
                        }
                    }
                    select_t1_value();
                    retry_count_ = 0;
                    if (native_active_ && va_ != vs_) {
                        IRIS_LOG("AX25 KISS retransmitting from V(A)=%d to V(S)=%d", va_, vs_);
                        invoke_retransmission();
                    }
                    set_state(Ax25SessionState::CONNECTED);
                    IRIS_LOG("AX25 KISS TIMER_RECOVERY -> CONNECTED (%s)",
                             pf ? "F=1 response" : "unsolicited RR advanced V(A)");
                }
                // Native-active KISS mode: respond to P=1 polls.
                // In pure KISS, the KISS client handles polls. In native mode,
                // the session IS the transport and must respond per AX.25 spec.
                //
                // RR-VOLLEY GUARD (native-peer livelock): enquiry_response() emits
                // an F=1 RESPONSE, and the pre-fix predicate fired on ANY pf=1
                // S-frame — including an F=1 RESPONSE that is itself just such an
                // answer.  Two native peers then volley RR F=1 rsp at each other
                // every turnaround forever; the standing RR keeps resetting T1 so
                // the C1 selective-retransmit of the one lost frame never re-fires
                // (specimen: 0 delivery, responder holding N(S)=2..7 buffered).
                // Per AX.25 2.2 §6.2 an enquiry response answers a COMMAND poll.
                // SURGICAL: a genuine COMMAND poll is ALWAYS answered, and an F=1
                // RESPONSE that advanced V(A) THIS frame is ALWAYS answered — that
                // progress-carrying turnaround is the LEGITIMATE fast pacing (each
                // answer's N(R) acks the peer's data, advancing ITS V(A)).  Only a
                // RESPONSE F=1 that made NO progress, repeated past the cap, is the
                // pathological echo: suppress further auto-answers so the channel
                // quiets and T1/C1 recover the stuck frame.  (Blanket C/R gating
                // here — answer commands only — killed the fast turnaround and
                // forced T1-timeout delivery; this preserves it.  The TIMER_RECOVERY
                // exit above is deliberately left untouched: it emits nothing and is
                // not part of the volley engine.)
                if (pf && native_active_ &&
                    (state_ == Ax25SessionState::CONNECTED ||
                     state_ == Ax25SessionState::TIMER_RECOVERY)) {
                    bool advanced = (va_ != kiss_prev_va);
                    if (frame.is_command() || advanced) {
                        rr_resp_noprogress_answers_ = 0;   // poll or progress
                        enquiry_response(true);
                    } else if (++rr_resp_noprogress_answers_ <=
                               kMaxNoProgressRespAnswers) {
                        // No-progress RESPONSE echo still under the cap: answer it,
                        // so a brief genuine turnaround transient is untouched.
                        enquiry_response(true);
                    } else if (rr_resp_noprogress_answers_ ==
                               kMaxNoProgressRespAnswers + 1) {
                        // Edge-triggered fire-proof at the suppression point.
                        IRIS_LOG("AX25 RR-volley guard: %d consecutive no-progress "
                                 "F=1 RESPONSE answers [N(R)=%d V(A)=%d V(S)=%d] -> "
                                 "suppress auto-answer until progress (T1/C1 recover)",
                                 rr_resp_noprogress_answers_ - 1, nr, va_, vs_);
                    }
                }
            }
            // Reset idle timer on any received frame
            t3_ = t3_value_;
        } else if (state_ == Ax25SessionState::AWAITING_RELEASE) {
            if (ft == Ax25FrameType::U_FRAME) {
                Ax25UType ut = frame.u_type();
                if (ut == Ax25UType::UA || ut == Ax25UType::DM) {
                    IRIS_LOG("AX25 KISS RX %s -> DISCONNECTED",
                             ut == Ax25UType::UA ? "UA" : "DM");
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
            }
        }
        return false;  // Never consume — let frames pass through to KISS
    }

    // -----------------------------------------------------------------------
    // STATE 1: DISCONNECTED
    // -----------------------------------------------------------------------
    if (state_ == Ax25SessionState::DISCONNECTED) {
        // KISS passthrough: KISS client handles incoming connections — don't accept
        if (kiss_passthrough_) {
            IRIS_LOG("AX25 RX frame while DISCONNECTED (KISS passthrough) — forwarding to KISS");
            return false;
        }

        if (ft == Ax25FrameType::U_FRAME) {
            Ax25UType ut = frame.u_type();

            if (ut == Ax25UType::SABM) {
                // Incoming connection request
                IRIS_LOG("AX25 RX SABM from %s (P=%d) [DISCONNECTED -> accept]",
                         frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0);
                reset_session();
                we_initiated_ = false;
                remote_call_ = frame.src.to_string();
                local_addr_ = ax25_make_addr(local_call_);
                remote_addr_ = frame.src;
                // The connect came THROUGH a digi path: the UA and every
                // session frame after it must traverse the digis in REVERSE
                // order, H-bits clear (adopt_via_path; Direwolf
                // ax25_link.c:877-890).  Must precede send_ua().
                adopt_via_path(frame.via, true);
                send_ua(frame.poll_final());
                t3_ = t3_value_;
                set_state(Ax25SessionState::CONNECTED);
                return true;
            }

            if (ut == Ax25UType::UI) {
                // UI frames pass through regardless of state
                return false;
            }

            if (ut == Ax25UType::DISC) {
                // DISC while disconnected — respond DM (F=P), reply path reversed
                IRIS_LOG("AX25 RX DISC from %s while DISCONNECTED",
                         frame.src.to_string().c_str());
                remote_addr_ = frame.src;
                adopt_via_path(frame.via, true);
                send_dm(frame.poll_final());
                return true;
            }

            // Any other command frame while disconnected: respond with DM
            IRIS_LOG("AX25 RX unexpected U-frame (0x%02X) from %s while DISCONNECTED",
                     frame.control, frame.src.to_string().c_str());
            remote_addr_ = frame.src;
            adopt_via_path(frame.via, true);
            send_dm(frame.poll_final());
            return true;
        }

        // I or S frame while disconnected — send DM
        if (ft == Ax25FrameType::I_FRAME || ft == Ax25FrameType::S_FRAME) {
            IRIS_LOG("AX25 RX %s from %s while DISCONNECTED -> DM",
                     ft == Ax25FrameType::I_FRAME ? "I" : "S",
                     frame.src.to_string().c_str());
            remote_addr_ = frame.src;
            adopt_via_path(frame.via, true);
            send_dm(frame.poll_final());
            return true;
        }

        return false;
    }

    // -----------------------------------------------------------------------
    // STATE 2: AWAITING_CONNECTION (sent SABM, waiting UA)
    // -----------------------------------------------------------------------
    if (state_ == Ax25SessionState::AWAITING_CONNECTION) {
        if (ft == Ax25FrameType::U_FRAME) {
            Ax25UType ut = frame.u_type();

            if (ut == Ax25UType::UA) {
                IRIS_LOG("AX25 RX UA from %s (F=%d) [AWAITING_CONNECTION -> CONNECTED]",
                         frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0);
                if (frame.poll_final()) {
                    // Expected UA with F=1
                    t1_ = 0;
                    // Complete RTT measurement (SABM → UA)
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 RTT measurement: %d ticks (%.1fs) [SABM->UA]",
                                 rtt, rtt * 0.05);
                        update_srt(rtt);
                    }
                    t3_ = t3_value_;
                    reset_sequence_epoch();
                    clear_exception_conditions();
                    set_state(Ax25SessionState::CONNECTED);
                    send_next_iframe();  // Send any queued data
                } else {
                    // UA with F=0 — unexpected (spec says ignore)
                    IRIS_LOG("AX25 UA with F=0 ignored (expected F=1)");
                }
                return true;
            }

            if (ut == Ax25UType::DM) {
                IRIS_LOG("AX25 RX DM from %s (F=%d) [AWAITING_CONNECTION -> DISCONNECTED]",
                         frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0);
                if (frame.poll_final()) {
                    t1_ = 0;
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
                // DM with F=0 — ignore
                return true;
            }

            if (ut == Ax25UType::SABM) {
                // Both sides sent SABM simultaneously — accept (collision case)
                // AX.25 2.2 Section 4.3.4.5: send UA AND transition to CONNECTED.
                // Without this, both sides stay in AWAITING_CONNECTION and the
                // UA responses collide on half-duplex radio → infinite SABM loop.
                IRIS_LOG("AX25 RX SABM from %s [simultaneous connect -> CONNECTED]",
                         frame.src.to_string().c_str());
                send_ua(frame.poll_final());
                t1_ = 0;
                t3_ = t3_value_;
                reset_sequence_epoch();
                clear_exception_conditions();
                set_state(Ax25SessionState::CONNECTED);
                return true;
            }

            // Other U-frames ignored in AWAITING_CONNECTION
            IRIS_LOG("AX25 RX U-frame 0x%02X ignored in AWAITING_CONNECTION",
                     frame.control);
            return true;
        }

        // I and S frames ignored in AWAITING_CONNECTION
        return true;
    }

    // -----------------------------------------------------------------------
    // STATE 5: AWAITING_RELEASE (sent DISC, waiting UA/DM)
    // -----------------------------------------------------------------------
    if (state_ == Ax25SessionState::AWAITING_RELEASE) {
        if (ft == Ax25FrameType::U_FRAME) {
            Ax25UType ut = frame.u_type();

            if (ut == Ax25UType::UA) {
                IRIS_LOG("AX25 RX UA from %s (F=%d) [AWAITING_RELEASE -> DISCONNECTED]",
                         frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0);
                if (frame.poll_final()) {
                    t1_ = 0;
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
                return true;
            }

            if (ut == Ax25UType::DM) {
                IRIS_LOG("AX25 RX DM from %s (F=%d) [AWAITING_RELEASE -> DISCONNECTED]",
                         frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0);
                if (frame.poll_final()) {
                    t1_ = 0;
                    reset_session();
                    set_state(Ax25SessionState::DISCONNECTED);
                }
                return true;
            }

            if (ut == Ax25UType::SABM) {
                // SABM while awaiting release — send DM (refuse new connection)
                IRIS_LOG("AX25 RX SABM while AWAITING_RELEASE -> DM");
                send_dm(frame.poll_final());
                return true;
            }

            // Other U-frames ignored
            return true;
        }

        // I and S frames: ignore but respond to polls
        if (ft == Ax25FrameType::I_FRAME || ft == Ax25FrameType::S_FRAME) {
            // Ignore content, but respond to P=1 with DM
            if (frame.poll_final()) {
                send_dm(true);
            }
            return true;
        }

        return true;
    }

    // -----------------------------------------------------------------------
    // STATE 3 & 4: CONNECTED / TIMER_RECOVERY
    // -----------------------------------------------------------------------
    bool in_timer_recovery = (state_ == Ax25SessionState::TIMER_RECOVERY);

    // --- U-frame handling ---
    if (ft == Ax25FrameType::U_FRAME) {
        Ax25UType ut = frame.u_type();

        switch (ut) {
        case Ax25UType::SABM: {
            // Link reset (re-establish connection)
            IRIS_LOG("AX25 RX SABM from %s (P=%d) [%s -> link reset]",
                     frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0,
                     in_timer_recovery ? "TIMER_RECOVERY" : "CONNECTED");
            // Re-adopt the reply path from THIS request (reversed, H clear):
            // the peer may reset the link with a different — or no — digi path,
            // and the reply must mirror the request that established it.
            adopt_via_path(frame.via, true);
            send_ua(frame.poll_final());
            clear_exception_conditions();
            // Discard I-frame queue
            while (!tx_queue_.empty()) tx_queue_.pop();
            reset_sequence_epoch();   // full ring + reorder buf (was 8 of SEQ_MOD_MAX slots)
            t1_ = 0;
            t3_ = t3_value_;
            set_state(Ax25SessionState::CONNECTED);
            return true;
        }

        case Ax25UType::DISC:
            IRIS_LOG("AX25 RX DISC from %s (P=%d) [%s -> DISCONNECTED]",
                     frame.src.to_string().c_str(), frame.poll_final() ? 1 : 0,
                     in_timer_recovery ? "TIMER_RECOVERY" : "CONNECTED");
            send_ua(frame.poll_final());
            t1_ = 0;
            t3_ = 0;
            reset_session();
            set_state(Ax25SessionState::DISCONNECTED);
            return true;

        case Ax25UType::UA:
            // Unsolicited UA in CONNECTED/TIMER_RECOVERY.
            // AX.25 2.2 says re-establish, but this creates an infinite SABM loop
            // during simultaneous connect: both sides go CONNECTED via SABM,
            // then each receives the other's stale UA → re-establish → loop.
            // Direwolf and most implementations simply ignore unsolicited UA.
            IRIS_LOG("AX25 RX unsolicited UA in %s -> ignored (harmless)",
                     in_timer_recovery ? "TIMER_RECOVERY" : "CONNECTED");
            return true;

        case Ax25UType::DM:
            IRIS_LOG("AX25 RX DM in %s -> DISCONNECTED",
                     in_timer_recovery ? "TIMER_RECOVERY" : "CONNECTED");
            t1_ = 0;
            t3_ = 0;
            reset_session();
            set_state(Ax25SessionState::DISCONNECTED);
            return true;

        case Ax25UType::FRMR:
            // FRMR received — re-establish link (AX.25 2.2 Section 4.3.10)
            IRIS_LOG("AX25 RX FRMR from %s -> re-establish link",
                     frame.src.to_string().c_str());
            establish_data_link();
            clear_exception_conditions();
            set_state(Ax25SessionState::AWAITING_CONNECTION);
            return true;

        case Ax25UType::UI:
        case Ax25UType::XID:
            // Pass through to higher layer
            return false;

        default:
            // Unknown U-frame — send FRMR W bit
            IRIS_LOG("AX25 RX unknown U-frame 0x%02X -> FRMR", frame.control);
            send_frmr(frame.control, vs_, vr_, false, true, false, false, false);
            return true;
        }
    }

    // --- I-frame handling (State 3 and 4) ---
    if (ft == Ax25FrameType::I_FRAME) {
        uint8_t ns = frame.ns();
        uint8_t nr = frame.nr();
        bool pf = frame.poll_final();
        IRIS_LOG("AX25 RX I N(S)=%d N(R)=%d P=%d (%zu bytes) [V(R)=%d V(A)=%d V(S)=%d %s]",
                 ns, nr, pf ? 1 : 0, frame.info.size(),
                 vr_, va_, vs_,
                 in_timer_recovery ? "TIMER_RECOVERY" : "CONNECTED");

        // Validate N(R) (AX.25 2.2 Section 4.3.3.3)
        if (!nr_valid(nr)) {
            IRIS_LOG("AX25 I-frame N(R)=%d invalid [V(A)=%d V(S)=%d]", nr, va_, vs_);
            nr_error_recovery();
            return true;
        }

        if (in_timer_recovery) {
            // STATE 4: Timer Recovery
            ack_frames(nr);

            if (ns == vr_) {
                // In-sequence
                vr_ = seq_inc(vr_);
                reject_exception_ = false;

                // Deliver data
                if (!frame.info.empty() && data_received_)
                    data_received_(frame.info.data(), frame.info.size());

                if (pf) {
                    // Response to our poll — leave timer recovery
                    t1_ = 0;
                    select_t1_value();
                    enquiry_response(true);
                    // If all acked, back to CONNECTED; else retransmit
                    if (va_ == vs_) {
                        set_state(Ax25SessionState::CONNECTED);
                        t3_ = t3_value_;
                        send_next_iframe();  // drain any queued data
                    } else {
                        invoke_retransmission();
                        set_state(Ax25SessionState::CONNECTED);
                    }
                } else {
                    acknowledge_pending_ = true;
                    if (t2_ == 0) t2_ = native_active_ ? 1 : T2_TICKS;
                }
            } else {
                // Out-of-sequence
                if (!reject_exception_) {
                    reject_exception_ = true;
                    send_rej(pf, false);  // Response
                    acknowledge_pending_ = false;
                } else {
                    // REJ already sent — discard but respond to P=1
                    if (pf) {
                        send_rr(true, false);
                    }
                }
            }
            return true;
        }

        // STATE 3: Connected (normal information transfer)
        ack_frames(nr);

        if (ns == vr_) {
            // In-sequence frame
            vr_ = seq_inc(vr_);
            reject_exception_ = false;

            // Deliver data to application
            if (!frame.info.empty() && data_received_)
                data_received_(frame.info.data(), frame.info.size());

            if (pf) {
                // Must respond with F=1
                enquiry_response(true);
            } else {
                acknowledge_pending_ = true;
                if (t2_ == 0) t2_ = native_active_ ? 1 : T2_TICKS;
            }

            t3_ = t3_value_;  // Reset idle timer

            // Try sending our own data (piggyback ack)
            send_next_iframe();
        } else if (in_window(ns)) {
            // Out-of-sequence but within window — request retransmission
            if (!reject_exception_) {
                reject_exception_ = true;
                send_rej(pf, pf ? false : true);  // Command if not responding to poll
                acknowledge_pending_ = false;
            } else {
                // REJ already sent — respond to poll if needed
                if (pf) {
                    send_rr(true, false);
                    acknowledge_pending_ = false;
                }
            }
        } else {
            IRIS_LOG("AX25 RX I N(S)=%d outside window [V(R)=%d K=%d]", ns, vr_, K);
            // Discard — respond to poll if needed
            if (pf) {
                enquiry_response(true);
            }
        }
        return true;
    }

    // --- S-frame handling (State 3 and 4) ---
    if (ft == Ax25FrameType::S_FRAME) {
        Ax25SType st = frame.s_type();
        uint8_t nr = frame.nr();
        bool pf = frame.poll_final();

        // Validate N(R)
        if (!nr_valid(nr)) {
            IRIS_LOG("AX25 S-frame N(R)=%d invalid [V(A)=%d V(S)=%d]", nr, va_, vs_);
            nr_error_recovery();
            return true;
        }

        if (in_timer_recovery) {
            // STATE 4: Timer Recovery — S-frame handling
            switch (st) {
            case Ax25SType::RR:
                IRIS_LOG("AX25 RX RR N(R)=%d %s=%d [TIMER_RECOVERY]",
                         nr, pf ? "F" : "P", pf ? 1 : 0);
                peer_busy_ = false;
                { uint8_t prev_va = va_;
                ack_frames(nr);
                if (pf) {
                    // This is the response to our poll — complete RTT measurement
                    t1_ = 0;
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 RTT measurement: %d ticks (%.1fs) [poll->RR]", rtt, rtt * 0.05);
                        update_srt(rtt);
                    }
                    select_t1_value();
                    if (va_ == vs_) {
                        t3_ = t3_value_;
                        set_state(Ax25SessionState::CONNECTED);
                        send_next_iframe();
                    } else {
                        invoke_retransmission();
                        set_state(Ax25SessionState::CONNECTED);
                    }
                } else if (native_active_ && va_ != prev_va) {
                    // Native OFDM-KISS: unsolicited RR (F=0) that advanced V(A).
                    // The peer's autonomous T2 RR arrives before our T1 poll.
                    // Treat as equivalent to poll response — exit timer recovery
                    // and resume data TX. Standard AX.25 stays in timer recovery
                    // for F=0, but that wastes a full T1 cycle on half-duplex links
                    // where the turnaround cost dominates.
                    IRIS_LOG("AX25 unsolicited RR advanced V(A) %d->%d in native mode -> CONNECTED",
                             prev_va, va_);
                    select_t1_value();
                    if (va_ == vs_) {
                        t3_ = t3_value_;
                        set_state(Ax25SessionState::CONNECTED);
                        send_next_iframe();
                    } else {
                        invoke_retransmission();
                        set_state(Ax25SessionState::CONNECTED);
                    }
                }
                } // prev_va scope
                break;

            case Ax25SType::RNR:
                IRIS_LOG("AX25 RX RNR N(R)=%d %s=%d [TIMER_RECOVERY]",
                         nr, pf ? "F" : "P", pf ? 1 : 0);
                peer_busy_ = true;
                ack_frames(nr);
                if (pf) {
                    t1_ = 0;
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 RTT measurement: %d ticks (%.1fs) [poll->RNR]", rtt, rtt * 0.05);
                        update_srt(rtt);
                    }
                    select_t1_value();
                    // Peer busy — go back to CONNECTED but don't retransmit
                    t3_ = t3_value_;
                    set_state(Ax25SessionState::CONNECTED);
                    if (va_ != vs_) t1_ = t1_value_;
                }
                break;

            case Ax25SType::REJ:
                IRIS_LOG("AX25 RX REJ N(R)=%d %s=%d [TIMER_RECOVERY]",
                         nr, pf ? "F" : "P", pf ? 1 : 0);
                peer_busy_ = false;
                ack_frames(nr);
                if (pf) {
                    t1_ = 0;
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 RTT measurement: %d ticks (%.1fs) [poll->REJ]", rtt, rtt * 0.05);
                        update_srt(rtt);
                    }
                    select_t1_value();
                    invoke_retransmission();
                    set_state(Ax25SessionState::CONNECTED);
                } else {
                    // REJ command (P=0): honor retransmission request even in
                    // timer recovery. On half-duplex FM the peer's REJ is always
                    // a command — ignoring it wastes a full T1 cycle.
                    // Stay in TIMER_RECOVERY (still awaiting F=1 poll response).
                    invoke_retransmission();
                }
                break;

            case Ax25SType::SREJ:
                // Treat SREJ as REJ (simplified)
                IRIS_LOG("AX25 RX SREJ N(R)=%d -> treating as REJ [TIMER_RECOVERY]", nr);
                peer_busy_ = false;
                ack_frames(nr);
                if (pf) {
                    t1_ = 0;
                    invoke_retransmission();
                    set_state(Ax25SessionState::CONNECTED);
                }
                break;
            }
            t3_ = t3_value_;
            return true;
        }

        // STATE 3: Connected — S-frame handling
        switch (st) {
        case Ax25SType::RR:
            IRIS_LOG("AX25 RX RR N(R)=%d %s=%d [CONNECTED]",
                     nr, pf ? "P" : "F", pf ? 1 : 0);
            peer_busy_ = false;
            ack_frames(nr);
            if (pf) {
                // Poll — must respond
                enquiry_response(true);
            }
            send_next_iframe();
            break;

        case Ax25SType::RNR:
            IRIS_LOG("AX25 RX RNR N(R)=%d %s=%d [CONNECTED]",
                     nr, pf ? "P" : "F", pf ? 1 : 0);
            peer_busy_ = true;
            ack_frames(nr);
            if (pf) {
                enquiry_response(true);
            }
            // Don't try to send I-frames while peer busy
            break;

        case Ax25SType::REJ:
            IRIS_LOG("AX25 RX REJ N(R)=%d %s=%d [CONNECTED]",
                     nr, pf ? "P" : "F", pf ? 1 : 0);
            peer_busy_ = false;
            ack_frames(nr);
            if (pf) {
                enquiry_response(true);
            }
            retransmit_from(nr);
            break;

        case Ax25SType::SREJ:
            // Treat as REJ (simplified)
            IRIS_LOG("AX25 RX SREJ N(R)=%d -> treating as REJ [CONNECTED]", nr);
            peer_busy_ = false;
            ack_frames(nr);
            retransmit_from(nr);
            break;
        }
        t3_ = t3_value_;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Timer management — AX.25 2.2 Section 6.7
// ---------------------------------------------------------------------------

void Ax25Session::tick() {
    wall_ticks_++;  // Always increment (not affected by channel_busy pause)
    if (state_ == Ax25SessionState::DISCONNECTED) return;

    std::lock_guard<std::mutex> lock(timer_mutex_);

    // Increment 1 (window-fill) RR-hold bookkeeping: track how long the channel
    // has been idle so a held autonomous RR fires only after the initiator's
    // burst sequence truly ends (see the T2 block below). Cheap saturating count.
    if (channel_busy_) rr_quiet_ticks_ = 0;
    else if (rr_quiet_ticks_ < 1000000) rr_quiet_ticks_++;

    // T2: response delay timer (triggers delayed ack)
    if (t2_ > 0 && (state_ == Ax25SessionState::CONNECTED ||
                     state_ == Ax25SessionState::TIMER_RECOVERY)) {
        t2_--;
        if (t2_ == 0 && acknowledge_pending_) {
            // Increment 1 RR-hold (collision-safety for the initiator's burst_fill):
            // while the initiator is still on-air (channel_busy_) OR the channel has
            // only just gone quiet (< RR_HOLD_QUIET_TICKS idle), DEFER the autonomous
            // RR — re-arm t2_ and keep acknowledge_pending_ set. This holds the RR
            // through the initiator's back-to-back bursts and fires it into the clear
            // slot after the sequence ends, so the reverse ACK never collides with
            // the next forward burst on half-duplex. Inert unless rr_hold_ is on, and
            // the initiator's T1 P=1 poll remains a hard backstop (enquiry_response),
            // so the hold cannot stall the link even if the channel never quiets.
            if (rr_hold_ && (channel_busy_ || rr_quiet_ticks_ < RR_HOLD_QUIET_TICKS)) {
                t2_ = 1;   // hold: re-check next tick, keep the ACK pending
            }
            // Piggybacked ACK: if I-frames are queued, suppress standalone RR.
            // The next I-frame's N(R) will implicitly carry the acknowledgment.
            // Saves one full ACK round-trip on bidirectional transfers.
            else if (!tx_queue_.empty()) {
                // Don't clear acknowledge_pending_ — the I-frame send path
                // will clear it when it sends the next I-frame with N(R)=V(R).
            } else {
                acknowledge_pending_ = false;
                // Generate RR if we own the transport (!kiss_managed_) OR
                // if native mode is active (we ARE the transport over OFDM PHY,
                // KISS client can't generate S-frames over native PHY).
                if (!kiss_managed_ || native_active_) {
                    if (own_busy_)
                        send_rnr(false, false);
                    else
                        send_rr(false, false);
                }
            }
        }
    }

    // T1: acknowledgment timer
    if (t1_ > 0) {
        t1_--;
        if (t1_ == 0) {
            retry_count_++;
            if (retry_count_ > n2_effective_) {
                IRIS_LOG("AX25 T1 expired, max retries (%d) exceeded -> DISCONNECTED", n2_effective_);
                reset_session();
                set_state(Ax25SessionState::DISCONNECTED);
                return;
            }

            if (state_ == Ax25SessionState::AWAITING_CONNECTION) {
                if (kiss_managed_) {
                    // KISS client handles SABM retries — just reset T1 and wait.
                    // The KISS client will re-send SABM if needed; each one
                    // resets our retry_count via notify_outgoing().
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — waiting for KISS client retry (%d/%d)", retry_count_, n2_effective_);
                } else {
                    IRIS_LOG("AX25 T1 retry SABM (%d/%d)", retry_count_, n2_effective_);
                    send_sabm();
                    t1_ = t1_with_jitter();
                }
            } else if (state_ == Ax25SessionState::AWAITING_RELEASE) {
                if (kiss_managed_) {
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — waiting for KISS client DISC retry (%d/%d)", retry_count_, n2_effective_);
                } else {
                    IRIS_LOG("AX25 T1 retry DISC (%d/%d)", retry_count_, n2_effective_);
                    send_disc();
                    t1_ = t1_with_jitter();
                }
            } else if (state_ == Ax25SessionState::CONNECTED) {
                if (kiss_managed_ && !native_active_) {
                    // Pure KISS passthrough: client handles its own polling.
                    // Don't count towards N2 — the KISS client manages retries;
                    // T3 idle supervision (5 min) is the safety net for dead links.
                    retry_count_ = 0;
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — skipping poll in CONNECTED");
                } else {
                    // Enter Timer Recovery: send RR poll to probe the peer.
                    // In OFDM-KISS native mode, we MUST poll — the session layer
                    // is the actual transport, and 30s+ silences kill throughput.
                    IRIS_LOG("AX25 T1 timeout -> TIMER_RECOVERY, poll (%d/%d)%s",
                             retry_count_, N2, native_active_ ? " [native]" : "");
                    send_rr(true, true);  // Command with P=1
                    t1_ = t1_with_jitter();
                    t3_ = 0;  // Stop T3 — T1 now supervises the link (AX.25 2.2 §6.4.4)
                    // Re-arm the C1 fast-retransmit for this recovery cycle.  On the
                    // MFSK-N(R) reverse-ACK link the stuck-RR fast-retransmit (NOT the
                    // dead F=1 poll cycle) is what actually recovers, so it must be
                    // allowed to fire once per cycle (P0 connect-recovery turnaround).
                    c1_nak_retx_from_ = kC1NakNone;
                    set_state(Ax25SessionState::TIMER_RECOVERY);
                }
            } else if (state_ == Ax25SessionState::TIMER_RECOVERY) {
                if (kiss_managed_ && !native_active_) {
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — skipping re-poll in TIMER_RECOVERY (%d/%d)", retry_count_, n2_effective_);
                } else {
                    // Already in timer recovery — re-poll
                    IRIS_LOG("AX25 T1 re-poll in TIMER_RECOVERY (%d/%d)",
                             retry_count_, n2_effective_);
                    send_rr(true, true);  // Command with P=1
                    t1_ = t1_with_jitter();
                    c1_nak_retx_from_ = kC1NakNone;   // re-arm C1 fast-retransmit
                }
            }
        }
    }

    // T3: idle supervision timer
    if (t3_ > 0 && (state_ == Ax25SessionState::CONNECTED ||
                     state_ == Ax25SessionState::TIMER_RECOVERY)) {
        t3_--;
        if (t3_ == 0) {
            if (kiss_managed_) {
                t3_ = t3_value_;  // Restart, don't send — KISS client handles idle polling
                IRIS_LOG("AX25 T3 (KISS-managed) — skipping idle poll");
            } else {
                IRIS_LOG("AX25 T3 idle timeout, entering timer recovery");
                retry_count_ = 0;
                send_rr(true, true);  // Command with P=1
                t1_ = t1_value_;
                set_state(Ax25SessionState::TIMER_RECOVERY);
            }
        }
    }
}

void Ax25Session::set_txdelay_ms(int ms) {
    // Compute T1 floor from TXDELAY: round-trip = 2*TXDELAY + 2*frame_time + processing
    // At 1200 baud, max frame ~256 bytes = ~1.7s TX time.  For T1 floor, assume
    // short frames (ACK ~100ms) + generous processing margin.
    // Floor = 2*TXDELAY + 1.5s (frame + turnaround + processing)
    int floor_ms = 2 * ms + 1500;
    int floor_ticks = (floor_ms + 49) / 50;  // Round up to 50ms ticks
    t1_floor_ = std::max(T1_TICKS, floor_ticks);
    // If current T1 value is below the new floor, raise it
    if (t1_value_ < t1_floor_)
        t1_value_ = t1_floor_;
    IRIS_LOG("AX25 T1 floor adjusted for TXDELAY=%dms: floor=%d ticks (%.1fs), T1=%d ticks (%.1fs)",
             ms, t1_floor_, t1_floor_ * 0.05, t1_value_, t1_value_ * 0.05);
}

void Ax25Session::set_channel_busy(bool busy) {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (busy == channel_busy_) return;
    channel_busy_ = busy;

    if (busy) {
        // Channel became busy — pause T1 and T3 by saving remaining ticks
        if (t1_ > 0) {
            t1_paused_remaining_ = t1_;
            t1_ = 0;
        }
        if (t3_ > 0) {
            t3_paused_remaining_ = t3_;
            t3_ = 0;
        }
    } else {
        // Channel became idle — resume T1 and T3 with saved remaining ticks
        if (t1_paused_remaining_ > 0) {
            t1_ = t1_paused_remaining_;
            t1_paused_remaining_ = 0;
        }
        if (t3_paused_remaining_ > 0) {
            t3_ = t3_paused_remaining_;
            t3_paused_remaining_ = 0;
        }
    }
}

void Ax25Session::select_t1_value() {
    // AX.25 2.2 Section 6.3.1: T1 adaptive based on Smoothed RTT.
    // Direwolf formula (ax25_link.c:6304-6397):
    //   First measurement: SRT = RTT
    //   Subsequent: SRT = 7/8 * SRT + 1/8 * RTT
    //   T1 = 2 * SRT (generous margin for half-duplex variation)
    // Floor from TXDELAY ensures T1 is never unreasonably short.
    if (srt_ticks_ > 0) {
        int new_t1 = srt_ticks_ * 2;
        // In native (OFDM) mode, cap T1 at 6s — OFDM frames are fast (~0.5-1s)
        // and the half-duplex turnaround is ~2s. 30s T1 from AFSK-era or
        // contention-inflated RTT measurements kills throughput by limiting
        // retry cycles.  In AFSK mode, keep the original 30s cap.
        int max_t1 = native_active_ ? 120 : 600;  // 6s native, 30s AFSK
        t1_value_ = std::max(t1_floor_, std::min(new_t1, max_t1));
        IRIS_LOG("AX25 adaptive T1: SRT=%d ticks (%.1fs), T1=%d ticks (%.1fs)%s",
                 srt_ticks_, srt_ticks_ * 0.05, t1_value_, t1_value_ * 0.05,
                 native_active_ ? " [native cap]" : "");
    } else {
        // No measurement yet — use floor
        if (t1_value_ < t1_floor_)
            t1_value_ = t1_floor_;
    }
}

void Ax25Session::update_srt(int rtt) {
    if (rtt <= 0) return;
    if (srt_ticks_ == 0) {
        // First measurement
        srt_ticks_ = rtt;
    } else {
        // Exponential smoothing: SRT = 7/8 * SRT + 1/8 * RTT
        srt_ticks_ = (7 * srt_ticks_ + rtt) / 8;
    }
    measuring_rtt_ = false;
    select_t1_value();
}

void Ax25Session::lower_t1_for_native() {
    // Native mode frames are much shorter than 1200 baud AX.25 (~100ms vs ~1.7s).
    // Reset SRT and T1 unconditionally: any AFSK-era RTT measurement is
    // meaningless for OFDM timing (includes probe delay, AFSK frame times,
    // and half-duplex contention artifacts).  Subsequent native-mode
    // poll→response measurements will set SRT accurately.
    // Event-driven MAC: csma_holdoff cancelled on RR reception. T1 is only
    // the safety net when RR is lost. T1=2.0s fires 0.5s after 1.5s csma
    // ceiling expires, triggering an RR poll for fast recovery.
    // T1=1.0s tested with stale poll drain: avg 112 bps (77-138, n=3)
    // vs T1=2.0s avg 126 bps (117-143, n=3). Higher variance, no gain.
    // Drain code remains in modem.cc as defense against edge cases.
    int native_floor = 40;   // 2.0s (40 ticks × 50ms)
    t1_floor_ = native_floor;
    srt_ticks_ = 0;  // Discard AFSK-era SRT — native measurements will replace it
    t1_value_ = native_floor;
    // Native OFDM has a weaker reverse path (ACKs travel through the same
    // lossy channel). Raise N2 from 10 to 30 — with T1=2.0s this gives 60s
    // of tolerance before disconnect, vs 20s at N2=10.  The asymmetric audio
    // path can lose ACKs for 15-20s stretches; N2=10 disconnects prematurely.
    n2_effective_ = 30;
    IRIS_LOG("AX25 T1 reset for native mode: floor=%d ticks (%.1fs), SRT reset, N2=%d",
             t1_floor_, t1_floor_ * 0.05, n2_effective_);
    // Don't start T1 here — the replayed AFSK I-frames will be sent as
    // OFDM in the next process_tx cycle, which calls start_t1_if_unacked
    // with the proper airtime-adjusted floor. Starting T1 at 2.0s default
    // causes a premature timeout before the first OFDM burst completes.
}

void Ax25Session::set_t1_floor_for_airtime(float airtime_s) {
    // T1 is paused during PTT (channel_busy), so the floor does NOT need to
    // cover our own TX airtime — only the peer's decode + response time.
    // OFDM-KISS: peer decodes each frame sequentially (~1s each at O0 NFFT=1024),
    // then holdoff (~0.5s), then ACK TX (~0.24s MFSK or ~0.55s OFDM),
    // then radio turnaround (~0.15s). For a 4-frame burst at O0 (~3.9s airtime),
    // peer response arrives ~4-5s after our last sample. Use airtime×1.5 + 2.0s.
    float margin_s = airtime_s * 1.5f + 2.0f;
    int floor_ticks = (int)(margin_s / 0.05f);
    // Not a ratchet — update floor to match current frame size
    t1_floor_ = floor_ticks;
    if (t1_value_ < t1_floor_)
        t1_value_ = t1_floor_;
    // Also update the running T1 if it's below the new floor.
    // Without this, T1 started by send_next_iframe() uses the old
    // t1_value_, while set_t1_floor_for_airtime raises it later.
    // The running t1_ stays short → premature T1 fire.
    if (t1_ > 0 && t1_ < t1_floor_)
        t1_ = t1_floor_;
}

void Ax25Session::set_max_info(int n) {
    // In wide (modulo-128) mode the control field is 2 octets, so an owned I-frame
    // carries 1 more header byte than mod-8.  Reserve that byte HERE so the framed
    // size (info + header + wrapper) still fits the level's OFDM block capacity
    // EXACTLY (e.g. O0: 73+17+4 == 94; O5: 555+17+4 == 576) — otherwise an extended
    // frame is 1 B oversize and trips the fit-floor/oversize guard.
    if (extended_ && n > 16) n -= 1;
    max_info_ = std::clamp(n, 16, MAX_INFO);
    IRIS_LOG("AX25 MAX_INFO set to %d bytes%s", max_info_, extended_ ? " (ext -1)" : "");
}

void Ax25Session::drop_oversized_in_window() {
    // Discard unacked I-frames in the TX window whose info field exceeds
    // the current max_info_. These frames were queued before OFDM reduced
    // max_info_ and can never be successfully transmitted over OFDM.
    // Resets V(S)=V(A) so the session re-fills the window with correctly-
    // sized frames from the KISS client.
    int dropped = 0;
    for (int i = 0; i < seq_mod_; i++) {
        if (!tx_window_[i].info.empty() && (int)tx_window_[i].info.size() > max_info_) {
            tx_window_[i] = TxIFrame{};
            dropped++;
        }
    }
    if (dropped > 0) {
        vs_ = va_;  // Reset send pointer — window will be refilled
        t1_ = 0;
        retry_count_ = 0;
        IRIS_LOG("AX25 dropped %d oversized frames from window (max_info=%d)", dropped, max_info_);
    }
    // Also drop oversized frames from the TX queue (not yet in window)
    std::queue<std::vector<uint8_t>> keep;
    int q_dropped = 0;
    while (!tx_queue_.empty()) {
        auto fr = std::move(tx_queue_.front());
        tx_queue_.pop();
        if ((int)fr.size() > max_info_) {
            q_dropped++;
        } else {
            keep.push(std::move(fr));
        }
    }
    tx_queue_ = std::move(keep);
    if (q_dropped > 0) {
        IRIS_LOG("AX25 dropped %d oversized frames from TX queue (max_info=%d)", q_dropped, max_info_);
    }
}

} // namespace iris
