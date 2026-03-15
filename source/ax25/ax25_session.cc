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
    state_ = s;
    static const char* names[] = {
        "DISCONNECTED", "AWAITING_CONNECTION", "CONNECTED",
        "TIMER_RECOVERY", "AWAITING_RELEASE"
    };
    IRIS_LOG("AX25 state -> %s (remote=%s)", names[(int)s], remote_call_.c_str());
    if (state_changed_) state_changed_(s, remote_call_);
}

void Ax25Session::reset_session() {
    vs_ = vr_ = va_ = 0;
    t1_ = t2_ = t3_ = 0;
    retry_count_ = 0;
    peer_busy_ = false;
    own_busy_ = false;
    reject_exception_ = false;
    acknowledge_pending_ = false;
    last_received_ctrl_ = 0;
    kiss_managed_ = false;
    we_initiated_ = false;
    channel_busy_ = false;
    t1_paused_remaining_ = 0;
    t3_paused_remaining_ = 0;
    measuring_rtt_ = false;
    // Keep srt_ticks_ across reconnects — measured RTT is still valid for same path
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (int i = 0; i < 8; i++) tx_window_[i] = TxIFrame{};
}

void Ax25Session::reset() {
    reset_session();
    remote_call_.clear();
    set_state(Ax25SessionState::DISCONNECTED);
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
            t1_ = t1_value_;
            t3_ = 0;
            set_state(Ax25SessionState::AWAITING_CONNECTION);
            IRIS_LOG("AX25 KISS SABM to %s — tracking session", remote_call_.c_str());
        } else if (ut == Ax25UType::UA &&
                   state_ == Ax25SessionState::DISCONNECTED) {
            // KISS client responding to incoming SABM — responder path.
            // Track the session so connection header exchange works.
            remote_call_ = f.dst.to_string();
            local_addr_ = ax25_make_addr(local_call_);
            remote_addr_ = f.dst;
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

    // Shadow V(S): track outgoing I-frames from KISS client
    if (f.type() == Ax25FrameType::I_FRAME && kiss_managed_) {
        vs_ = (f.ns() + 1) % 8;
    }
}

void Ax25Session::connect(const std::string& remote_call) {
    reset_session();
    remote_call_ = remote_call;
    local_addr_ = ax25_make_addr(local_call_);
    remote_addr_ = ax25_make_addr(remote_call_);
    we_initiated_ = true;
    establish_data_link();
    set_state(Ax25SessionState::AWAITING_CONNECTION);
}

void Ax25Session::send_data(const uint8_t* data, size_t len) {
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;

    // Fragment into MAX_INFO chunks
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = std::min((size_t)MAX_INFO, len - offset);
        tx_queue_.push(std::vector<uint8_t>(data + offset, data + offset + chunk));
        offset += chunk;
    }

    send_next_iframe();
}

void Ax25Session::disconnect() {
    if (state_ == Ax25SessionState::DISCONNECTED) return;

    // Discard pending I-frames
    while (!tx_queue_.empty()) tx_queue_.pop();
    for (int i = 0; i < 8; i++) tx_window_[i] = TxIFrame{};

    retry_count_ = 0;
    send_disc();
    t1_ = t1_value_;
    t3_ = 0;
    set_state(Ax25SessionState::AWAITING_RELEASE);
}

int Ax25Session::pending_frames() const {
    int count = (int)tx_queue_.size();
    for (int i = 0; i < 8; i++) {
        if (!tx_window_[i].info.empty()) count++;
    }
    return count;
}

// ---------------------------------------------------------------------------
// N(R) validation (AX.25 2.2 Section 4.3.3.3)
// V(A) <= N(R) <= V(S) (modulo 8)
// ---------------------------------------------------------------------------

bool Ax25Session::nr_valid(uint8_t nr) const {
    // N(R) is valid if V(A) <= N(R) <= V(S) in modulo-8 arithmetic
    nr &= 0x07;
    for (uint8_t i = va_; ; i = (i + 1) & 0x07) {
        if (i == nr) return true;
        if (i == vs_) return false;
    }
}

// ---------------------------------------------------------------------------
// Frame building and sending
// ---------------------------------------------------------------------------

void Ax25Session::send_sabm() {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_SABM, true, true);
    IRIS_LOG("AX25 TX SABM to %s (P=1)", remote_call_.c_str());
    if (send_frame_) send_frame_(frame.data(), frame.size());
    // Start RTT measurement (SABM → UA)
    rtt_start_ = wall_ticks_;
    measuring_rtt_ = true;
}

void Ax25Session::send_ua(bool pf) {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_UA, pf, false);
    IRIS_LOG("AX25 TX UA to %s (F=%d)", remote_call_.c_str(), pf ? 1 : 0);
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_disc() {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_DISC, true, true);
    IRIS_LOG("AX25 TX DISC to %s (P=1)", remote_call_.c_str());
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_dm(bool pf) {
    auto frame = ax25_build_u(remote_addr_, local_addr_, AX25_CTRL_DM, pf, false);
    IRIS_LOG("AX25 TX DM to %s (F=%d)",
             remote_call_.empty() ? "?" : remote_call_.c_str(), pf ? 1 : 0);
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_rr(bool pf, bool command) {
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::RR, vr_, pf, command);
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
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::RNR, vr_, pf, command);
    IRIS_LOG("AX25 TX RNR N(R)=%d %s=%d %s", vr_,
             command ? "P" : "F", pf ? 1 : 0,
             command ? "cmd" : "rsp");
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_rej(bool pf, bool command) {
    auto frame = ax25_build_s(remote_addr_, local_addr_, Ax25SType::REJ, vr_, pf, command);
    IRIS_LOG("AX25 TX REJ N(R)=%d %s=%d %s", vr_,
             command ? "P" : "F", pf ? 1 : 0,
             command ? "cmd" : "rsp");
    if (send_frame_) send_frame_(frame.data(), frame.size());
}

void Ax25Session::send_frmr(uint8_t rejected_ctrl, uint8_t vs, uint8_t vr, bool cr,
                              bool w_bit, bool x_bit, bool y_bit, bool z_bit) {
    auto frame = ax25_build_frmr(remote_addr_, local_addr_,
                                  rejected_ctrl, vs, vr, cr, w_bit, x_bit, y_bit, z_bit);
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

void Ax25Session::send_next_iframe() {
    if (peer_busy_) return;
    if (state_ != Ax25SessionState::CONNECTED &&
        state_ != Ax25SessionState::TIMER_RECOVERY)
        return;

    // Fill window from queue
    while (!tx_queue_.empty()) {
        int window_used = (vs_ - va_ + 8) % 8;
        if (window_used >= K) break;

        int slot = vs_ % 8;
        tx_window_[slot].info = std::move(tx_queue_.front());
        tx_queue_.pop();
        tx_window_[slot].sent = false;
        vs_ = (vs_ + 1) % 8;
    }

    // Send unsent frames in window
    for (uint8_t seq = va_; seq != vs_; seq = (seq + 1) % 8) {
        int slot = seq % 8;
        if (!tx_window_[slot].sent && !tx_window_[slot].info.empty()) {
            auto frame = ax25_build_i(remote_addr_, local_addr_,
                                       seq, vr_, false, AX25_PID_NONE,
                                       tx_window_[slot].info.data(),
                                       tx_window_[slot].info.size());
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
    // Acknowledge all frames with seq < nr (mod 8)
    while (va_ != nr) {
        int slot = va_ % 8;
        if (!tx_window_[slot].info.empty()) {
            IRIS_LOG("AX25 ACK frame N(S)=%d", va_);
        }
        tx_window_[slot] = TxIFrame{};
        va_ = (va_ + 1) % 8;
    }

    if (va_ == vs_) {
        t1_ = 0;           // All acknowledged — stop T1
        t3_ = t3_value_;    // Start idle supervision
    } else {
        t1_ = t1_value_;    // Restart T1 for remaining frames
    }
    retry_count_ = 0;
}

void Ax25Session::retransmit_from(uint8_t nr) {
    // Mark frames from nr to vs_ as unsent for retransmission
    for (uint8_t seq = nr; seq != vs_; seq = (seq + 1) % 8)
        tx_window_[seq % 8].sent = false;
    send_next_iframe();
}

bool Ax25Session::in_window(uint8_t ns) const {
    for (int i = 0; i < K; i++) {
        if (((vr_ + i) % 8) == ns) return true;
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
                    vs_ = vr_ = va_ = 0;
                    set_state(Ax25SessionState::CONNECTED);
                } else if (ut == Ax25UType::SABM) {
                    // Simultaneous connect: remote also sent SABM.
                    // KISS client will generate UA — we just track the state.
                    IRIS_LOG("AX25 KISS RX SABM from %s [simultaneous connect -> CONNECTED]",
                             frame.src.to_string().c_str());
                    t1_ = 0;
                    t3_ = t3_value_;
                    vs_ = vr_ = va_ = 0;
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
            // Shadow V(R): track incoming I-frames so we can inject our own
            if (ft == Ax25FrameType::I_FRAME) {
                uint8_t ns = frame.ns();
                if (ns == vr_)
                    vr_ = (vr_ + 1) % 8;
                // Shadow V(A) from I-frame N(R) (peer piggyback ack)
                uint8_t nr = frame.nr();
                if (nr_valid(nr) && nr != va_) {
                    IRIS_LOG("AX25 KISS shadow V(A) %d -> %d (I-frame N(R))", va_, nr);
                    va_ = nr;
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
                if (nr_valid(nr) && nr != va_) {
                    IRIS_LOG("AX25 KISS shadow V(A) %d -> %d (%s N(R))",
                             va_, nr,
                             st == Ax25SType::RR ? "RR" :
                             st == Ax25SType::RNR ? "RNR" : "REJ");
                    va_ = nr;
                }
                // Track peer busy state
                if (st == Ax25SType::RNR)
                    peer_busy_ = true;
                else if (st == Ax25SType::RR || st == Ax25SType::REJ)
                    peer_busy_ = false;
                // F=1 response to our poll: exit TIMER_RECOVERY
                if (pf && state_ == Ax25SessionState::TIMER_RECOVERY) {
                    t1_ = 0;
                    if (measuring_rtt_) {
                        int rtt = wall_ticks_ - rtt_start_;
                        IRIS_LOG("AX25 KISS RTT: %d ticks (%.1fs) [poll->%s]",
                                 rtt, rtt * 0.05,
                                 st == Ax25SType::RR ? "RR" : "RNR");
                        update_srt(rtt);
                    }
                    select_t1_value();
                    retry_count_ = 0;
                    set_state(Ax25SessionState::CONNECTED);
                    IRIS_LOG("AX25 KISS TIMER_RECOVERY -> CONNECTED (F=1 response)");
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
                // DISC while disconnected — respond DM (F=P)
                IRIS_LOG("AX25 RX DISC from %s while DISCONNECTED",
                         frame.src.to_string().c_str());
                remote_addr_ = frame.src;
                send_dm(frame.poll_final());
                return true;
            }

            // Any other command frame while disconnected: respond with DM
            IRIS_LOG("AX25 RX unexpected U-frame (0x%02X) from %s while DISCONNECTED",
                     frame.control, frame.src.to_string().c_str());
            remote_addr_ = frame.src;
            send_dm(frame.poll_final());
            return true;
        }

        // I or S frame while disconnected — send DM
        if (ft == Ax25FrameType::I_FRAME || ft == Ax25FrameType::S_FRAME) {
            IRIS_LOG("AX25 RX %s from %s while DISCONNECTED -> DM",
                     ft == Ax25FrameType::I_FRAME ? "I" : "S",
                     frame.src.to_string().c_str());
            remote_addr_ = frame.src;
            send_dm(frame.poll_final());
            return true;
        }

        return false;
    }

    // For connected states, verify source matches our peer
    if (state_ != Ax25SessionState::DISCONNECTED) {
        if (!frame.src.matches(remote_call_))
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
                    vs_ = vr_ = va_ = 0;
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
                vs_ = vr_ = va_ = 0;
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
            send_ua(frame.poll_final());
            clear_exception_conditions();
            // Discard I-frame queue
            while (!tx_queue_.empty()) tx_queue_.pop();
            for (int i = 0; i < 8; i++) tx_window_[i] = TxIFrame{};
            vs_ = vr_ = va_ = 0;
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
                vr_ = (vr_ + 1) % 8;
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
                    } else {
                        invoke_retransmission();
                        set_state(Ax25SessionState::CONNECTED);
                    }
                } else {
                    acknowledge_pending_ = true;
                    if (t2_ == 0) t2_ = T2_TICKS;
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
            vr_ = (vr_ + 1) % 8;
            reject_exception_ = false;

            // Deliver data to application
            if (!frame.info.empty() && data_received_)
                data_received_(frame.info.data(), frame.info.size());

            if (pf) {
                // Must respond with F=1
                enquiry_response(true);
            } else {
                acknowledge_pending_ = true;
                if (t2_ == 0) t2_ = T2_TICKS;
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
                        // All acknowledged — back to CONNECTED
                        t3_ = t3_value_;
                        set_state(Ax25SessionState::CONNECTED);
                    } else {
                        // Outstanding frames — retransmit and return to CONNECTED
                        invoke_retransmission();
                        set_state(Ax25SessionState::CONNECTED);
                    }
                } else {
                    // Not a response to our poll — stay in timer recovery
                }
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
                    // Stay in timer recovery
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

    // T2: response delay timer (triggers delayed ack)
    if (t2_ > 0 && (state_ == Ax25SessionState::CONNECTED ||
                     state_ == Ax25SessionState::TIMER_RECOVERY)) {
        t2_--;
        if (t2_ == 0 && acknowledge_pending_) {
            acknowledge_pending_ = false;
            if (!kiss_managed_) {
                if (own_busy_)
                    send_rnr(false, false);
                else
                    send_rr(false, false);
            }
        }
    }

    // T1: acknowledgment timer
    if (t1_ > 0) {
        t1_--;
        if (t1_ == 0) {
            retry_count_++;
            if (retry_count_ > N2) {
                IRIS_LOG("AX25 T1 expired, max retries (%d) exceeded -> DISCONNECTED", N2);
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
                    IRIS_LOG("AX25 T1 (KISS-managed) — waiting for KISS client retry (%d/%d)", retry_count_, N2);
                } else {
                    IRIS_LOG("AX25 T1 retry SABM (%d/%d)", retry_count_, N2);
                    send_sabm();
                    t1_ = t1_with_jitter();
                }
            } else if (state_ == Ax25SessionState::AWAITING_RELEASE) {
                if (kiss_managed_) {
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — waiting for KISS client DISC retry (%d/%d)", retry_count_, N2);
                } else {
                    IRIS_LOG("AX25 T1 retry DISC (%d/%d)", retry_count_, N2);
                    send_disc();
                    t1_ = t1_with_jitter();
                }
            } else if (state_ == Ax25SessionState::CONNECTED) {
                if (kiss_managed_ && !native_active_) {
                    // Pure KISS passthrough: client handles its own polling
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — skipping poll in CONNECTED (%d/%d)", retry_count_, N2);
                } else {
                    // Enter Timer Recovery: send RR poll to probe the peer.
                    // In OFDM-KISS native mode, we MUST poll — the session layer
                    // is the actual transport, and 30s+ silences kill throughput.
                    IRIS_LOG("AX25 T1 timeout -> TIMER_RECOVERY, poll (%d/%d)%s",
                             retry_count_, N2, native_active_ ? " [native]" : "");
                    send_rr(true, true);  // Command with P=1
                    t1_ = t1_with_jitter();
                    t3_ = 0;  // Stop T3 — T1 now supervises the link (AX.25 2.2 §6.4.4)
                    set_state(Ax25SessionState::TIMER_RECOVERY);
                }
            } else if (state_ == Ax25SessionState::TIMER_RECOVERY) {
                if (kiss_managed_ && !native_active_) {
                    t1_ = t1_with_jitter();
                    IRIS_LOG("AX25 T1 (KISS-managed) — skipping re-poll in TIMER_RECOVERY (%d/%d)", retry_count_, N2);
                } else {
                    // Already in timer recovery — re-poll
                    IRIS_LOG("AX25 T1 re-poll in TIMER_RECOVERY (%d/%d)",
                             retry_count_, N2);
                    send_rr(true, true);  // Command with P=1
                    t1_ = t1_with_jitter();
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
        // Clamp to reasonable range: floor to 30s max
        t1_value_ = std::max(t1_floor_, std::min(new_t1, 600));
        IRIS_LOG("AX25 adaptive T1: SRT=%d ticks (%.1fs), T1=%d ticks (%.1fs)",
                 srt_ticks_, srt_ticks_ * 0.05, t1_value_, t1_value_ * 0.05);
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
    // Lower T1 floor and value to reduce dead time from T1 waits.
    // Native RTT: ~200-400ms (PTT delay + frame + turnaround + response).
    // T1 = 2.0s gives generous margin while avoiding the 4.0s default stall.
    int native_floor = 40;  // 2.0s (40 ticks × 50ms)
    if (t1_floor_ > native_floor) {
        t1_floor_ = native_floor;
        IRIS_LOG("AX25 T1 floor lowered for native mode: %d ticks (%.1fs)",
                 t1_floor_, t1_floor_ * 0.05);
    }
    if (t1_value_ > native_floor && srt_ticks_ == 0) {
        // Only lower if no RTT measurement yet (adaptive T1 will take over)
        t1_value_ = native_floor;
        IRIS_LOG("AX25 T1 lowered for native mode: %d ticks (%.1fs)",
                 t1_value_, t1_value_ * 0.05);
    }
}

void Ax25Session::set_t1_floor_for_airtime(float airtime_s) {
    // Native data frames can be 5+ seconds of airtime. T1 must not expire
    // during our own transmission. Set floor = airtime + 3s RTT margin.
    // The adaptive SRT measures poll-to-response RTT (~1.5s for tiny frames),
    // which is way too short for data frames. This floor prevents false
    // TIMER_RECOVERY events that waste ~4 seconds each.
    int floor_ticks = (int)((airtime_s + 3.0f) / 0.05f);  // 50ms per tick
    if (floor_ticks > t1_floor_) {
        t1_floor_ = floor_ticks;
        if (t1_value_ < t1_floor_)
            t1_value_ = t1_floor_;
        IRIS_LOG("AX25 T1 floor raised for frame airtime %.1fs: floor=%d ticks (%.1fs), T1=%d ticks (%.1fs)",
                 airtime_s, t1_floor_, t1_floor_ * 0.05f, t1_value_, t1_value_ * 0.05f);
    }
}

// ---------------------------------------------------------------------------
// KISS injection helpers
// ---------------------------------------------------------------------------

// Find control byte offset in raw AX.25 frame (after address field)
} // namespace iris
