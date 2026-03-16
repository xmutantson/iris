#include "arq/arq.h"
#include "native/frame.h"
#include <cstring>
#include <algorithm>
#include <cstdio>

namespace iris {

// --- ArqFrame serialization ---

std::vector<uint8_t> ArqFrame::serialize() const {
    std::vector<uint8_t> out;
    out.push_back((uint8_t)type);
    out.push_back(seq);
    out.push_back(flags);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

bool ArqFrame::deserialize(const uint8_t* data, size_t len, ArqFrame& out) {
    if (len < 3) return false;
    // Validate type byte is a known ARQ frame type
    switch ((ArqType)data[0]) {
        case ArqType::HAIL: case ArqType::HAIL_ACK:
        case ArqType::CONNECT: case ArqType::CONNECT_ACK:
        case ArqType::DATA: case ArqType::ACK: case ArqType::NACK:
        case ArqType::SET_SPEED: case ArqType::SPEED_ACK:
        case ArqType::SET_CONFIG: case ArqType::CONFIG_ACK:
        case ArqType::SWITCH_ROLE: case ArqType::BREAK:
        case ArqType::DISCONNECT: case ArqType::DISCONNECT_ACK:
            break;
        default:
            return false;  // Unknown type — not an ARQ frame
    }
    out.type = (ArqType)data[0];
    out.seq = data[1];
    out.flags = data[2];
    if (len > 3)
        out.payload.assign(data + 3, data + len);
    else
        out.payload.clear();
    return true;
}

// --- ArqSession ---

ArqSession::ArqSession() {
    reset();
}

void ArqSession::reset() {
    role_ = ArqRole::IDLE;
    state_ = ArqState::IDLE;
    speed_level_ = 0;
    remote_snr_ = 0;
    local_snr_ = 0;
    retransmit_count_ = 0;
    chase_disable();
    peer_caps_ = 0;
    ack_timeout_ms_ = ARQ_DEFAULT_ACK_TIMEOUT_MS;
    rtt_sum_ms_ = 0;
    rtt_count_ = 0;
    tx_base_ = 0;
    tx_next_ = 0;
    end_of_data_ = false;
    rx_expected_ = 0;
    connect_retries_ = 0;
    hail_attempts_ = 0;

    batch_data_delivered_ = false;
    last_received_eob_seq_ = -1;

    turbo_phase_ = TurboPhase::NONE;
    turbo_target_speed_ = 0;
    turbo_last_good_ = 0;
    turbo_retries_ = 0;
    turbo_forward_done_ = false;
    turbo_verification_count_ = 0;

    consec_data_acks_ = 0;
    consec_data_nacks_ = 0;
    gearshift_just_applied_ = false;
    gearshift_cooldown_ = 0;

    break_phase_ = BreakPhase::NONE;
    break_recovery_speed_ = 0;
    break_retries_ = 0;
    break_drop_step_ = 1;
    emergency_nack_count_ = 0;

    memset(peer_x25519_pub_, 0, 32);
    has_peer_x25519_ = false;

    rx_mute_ = false;

    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        tx_window_[i] = TxSlot{};
        rx_window_[i] = RxSlot{};
    }

    while (!tx_data_queue_.empty()) tx_data_queue_.pop();
}

void ArqSession::set_state(ArqState s) {
    state_ = s;
    if (callbacks_.on_state_changed)
        callbacks_.on_state_changed(s);
}

void ArqSession::set_speed(int level) {
    level = std::max(0, std::min(level, NUM_SPEED_LEVELS - 1));
    if (level != speed_level_) {
        speed_level_ = level;
        chase_clear();  // Speed change = different frame format, old LLRs invalid
        if (callbacks_.on_speed_changed)
            callbacks_.on_speed_changed(level);
    }
}

void ArqSession::update_ack_timeout(int64_t rtt_ms) {
    rtt_sum_ms_ += rtt_ms;
    rtt_count_++;
    int64_t avg = rtt_sum_ms_ / rtt_count_;
    ack_timeout_ms_ = std::max(ARQ_MIN_ACK_TIMEOUT_MS,
                      std::min(ARQ_MAX_ACK_TIMEOUT_MS, (int)(avg * 5 / 2)));
}

int ArqSession::compute_batch_timeout(int batch_size, int msg_tx_time_ms) const {
    // Mercury-style: covers full TX + ACK round-trip + PTT delays
    int tx_time = batch_size * msg_tx_time_ms;
    int ack_time = msg_tx_time_ms + 200;   // ACK pattern + PTT settle
    int ptt_delay = 400;                    // PTT on/off transitions
    int margin = 500;                       // safety margin

    int timeout = tx_time + ack_time + ptt_delay + margin;

    // Turboshift extension
    if (state_ == ArqState::TURBOSHIFT)
        timeout += 2000;

    return std::max(ARQ_MIN_ACK_TIMEOUT_MS,
           std::min(ARQ_MAX_ACK_TIMEOUT_MS, timeout));
}

void ArqSession::send_arq_frame(const ArqFrame& frame) {
    auto data = frame.serialize();
    if (callbacks_.send_frame)
        callbacks_.send_frame(data.data(), data.size());
    last_send_time_ = Clock::now();
}

// --- Role-switch buffer protection ---

void ArqSession::clear_stale_buffers() {
    // Clear RX window to prevent stale audio from previous direction
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++)
        rx_window_[i] = RxSlot{};
    rx_expected_ = 0;

    // Mute RX for ~200ms to let stale audio drain.
    // VHF FM audio pipeline latency is ~50-100ms; 200ms is sufficient.
    rx_mute_ = true;
    rx_mute_until_ = Clock::now() + std::chrono::milliseconds(200);

    batch_data_delivered_ = false;
    last_received_eob_seq_ = -1;
}

// --- HAIL beacon ---

void ArqSession::listen() {
    reset();
    role_ = ArqRole::LISTENING;
    set_state(ArqState::LISTENING);
}

// --- Connection with HAIL + capability negotiation ---

void ArqSession::connect(const std::string& remote_callsign) {
    reset();
    role_ = ArqRole::COMMANDER;
    remote_callsign_ = remote_callsign;
    hail_attempts_ = 0;

    // Start with HAIL beacon phase
    set_state(ArqState::HAILING);
    connect_time_ = Clock::now();

    ArqFrame frame;
    frame.type = ArqType::HAIL;
    frame.seq = 0;
    frame.flags = 0;
    frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(frame);
    hail_attempts_++;
}

void ArqSession::handle_hail(const ArqFrame& frame) {
    if (state_ != ArqState::LISTENING && state_ != ArqState::IDLE)
        return;

    role_ = ArqRole::RESPONDER;
    remote_callsign_.assign(frame.payload.begin(), frame.payload.end());

    // ACK the HAIL beacon
    ArqFrame ack;
    ack.type = ArqType::HAIL_ACK;
    ack.seq = 0;
    ack.flags = 0;
    ack.payload.insert(ack.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(ack);

    // Wait for CONNECT
    set_state(ArqState::CONNECTING);
    connect_time_ = Clock::now();
}

void ArqSession::handle_hail_ack(const ArqFrame& frame) {
    if (state_ != ArqState::HAILING)
        return;

    // Responder detected — proceed to CONNECT
    if (frame.payload.size() > 0)
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());

    set_state(ArqState::CONNECTING);
    connect_retries_ = 0;
    connect_time_ = Clock::now();

    ArqFrame conn;
    conn.type = ArqType::CONNECT;
    conn.seq = 0;
    conn.flags = (uint8_t)speed_level_;
    conn.payload.push_back((uint8_t)(local_caps_ >> 8));
    conn.payload.push_back((uint8_t)(local_caps_ & 0xFF));
    // Append X25519 public key (32 bytes) for DH key exchange
    if (has_local_x25519_) {
        conn.payload.insert(conn.payload.end(), local_x25519_pub_, local_x25519_pub_ + 32);
    }
    conn.payload.insert(conn.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(conn);
}

void ArqSession::send_data(const uint8_t* data, size_t len) {
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = std::min((size_t)ARQ_MAX_PAYLOAD, len - offset);
        tx_data_queue_.push(std::vector<uint8_t>(data + offset, data + offset + chunk));
        offset += chunk;
    }
    end_of_data_ = false;

    if ((state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT) &&
        role_ == ArqRole::COMMANDER)
        send_next_data();
    else if (state_ == ArqState::CONNECTED && role_ == ArqRole::RESPONDER) {
        // Responder needs to become commander to send data — request role switch
        printf("[ARQ] Responder has data to send, requesting role switch\n");
        fflush(stdout);
        initiate_role_switch();
    }
}

int ArqSession::tx_queue_bytes() const {
    return (int)tx_data_queue_.size() * ARQ_MAX_PAYLOAD;
}

int ArqSession::pending_frames() const {
    // Queued chunks + unacked window slots
    int queued = (int)tx_data_queue_.size();
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (tx_window_[i].sent && !tx_window_[i].acked)
            queued++;
    }
    return queued;
}

void ArqSession::send_next_data() {
    if (role_ != ArqRole::COMMANDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    while (tx_next_ != (tx_base_ + ARQ_WINDOW_SIZE) % (ARQ_WINDOW_SIZE * 2)) {
        int slot = tx_next_ % ARQ_WINDOW_SIZE;

        if (!tx_window_[slot].sent || tx_window_[slot].acked) {
            if (tx_data_queue_.empty()) {
                end_of_data_ = true;
                break;
            }

            tx_window_[slot].data = std::move(tx_data_queue_.front());
            tx_data_queue_.pop();
            tx_window_[slot].acked = false;
            tx_window_[slot].sent = true;
            tx_window_[slot].retries = 0;
            tx_window_[slot].send_time = Clock::now();

            ArqFrame frame;
            frame.type = ArqType::DATA;
            frame.seq = (uint8_t)(tx_next_ % ARQ_WINDOW_SIZE);
            frame.flags = 0;

            if (tx_data_queue_.empty()) {
                frame.flags |= 0x80;  // end-of-batch flag
                end_of_data_ = true;
            }

            frame.payload = tx_window_[slot].data;
            send_arq_frame(frame);
            tx_next_++;
        } else {
            break;
        }
    }
}

// --- Frame dispatch ---

bool ArqSession::on_frame_received(const uint8_t* data, size_t len) {
    // RX mute guard: discard frames during mute window
    if (rx_mute_) {
        if (Clock::now() >= rx_mute_until_)
            rx_mute_ = false;
        else
            return true;  // Muted, consider handled
    }

    ArqFrame frame;
    if (!ArqFrame::deserialize(data, len, frame))
        return false;  // Not an ARQ frame

    switch (frame.type) {
        case ArqType::HAIL:         handle_hail(frame); break;
        case ArqType::HAIL_ACK:     handle_hail_ack(frame); break;
        case ArqType::CONNECT:      handle_connect(frame); break;
        case ArqType::CONNECT_ACK:  handle_connect_ack(frame); break;
        case ArqType::DATA:         handle_data(frame); break;
        case ArqType::ACK:
        case ArqType::NACK:         handle_ack(frame); break;
        case ArqType::SET_SPEED:    handle_set_speed(frame); break;
        case ArqType::SPEED_ACK:    handle_speed_ack(frame); break;
        case ArqType::SET_CONFIG:   handle_set_config(frame); break;
        case ArqType::CONFIG_ACK:   handle_config_ack(frame); break;
        case ArqType::SWITCH_ROLE:  handle_switch_role(frame); break;
        case ArqType::BREAK:        handle_break(frame); break;
        case ArqType::DISCONNECT:   handle_disconnect(frame); break;
        case ArqType::DISCONNECT_ACK:
            if (state_ == ArqState::DISCONNECTING) {
                set_state(ArqState::IDLE);
                role_ = ArqRole::IDLE;
            }
            break;
    }
    return true;
}

// --- Connection handlers with capability exchange ---

void ArqSession::handle_connect(const ArqFrame& frame) {
    if (state_ != ArqState::IDLE && state_ != ArqState::CONNECTING &&
        state_ != ArqState::LISTENING)
        return;

    role_ = ArqRole::RESPONDER;

    if (frame.payload.size() >= 2) {
        peer_caps_ = ((uint16_t)frame.payload[0] << 8) | frame.payload[1];
        // Check for X25519 pubkey: caps(2) + x25519(32) + callsign(1+)
        if ((peer_caps_ & CAP_ENCRYPTION) && frame.payload.size() >= 2 + 32 + 1) {
            memcpy(peer_x25519_pub_, frame.payload.data() + 2, 32);
            has_peer_x25519_ = true;
            remote_callsign_.assign(frame.payload.begin() + 34, frame.payload.end());
        } else {
            has_peer_x25519_ = false;
            remote_callsign_.assign(frame.payload.begin() + 2, frame.payload.end());
        }
    } else {
        peer_caps_ = 0;
        has_peer_x25519_ = false;
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    }

    // Reset per-session state to prevent stale flags from previous session
    batch_data_delivered_ = false;
    last_received_eob_seq_ = -1;

    set_state(ArqState::CONNECTED);
    chase_enable();  // Enable Chase combining for this session

    ArqFrame ack;
    ack.type = ArqType::CONNECT_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    ack.payload.push_back((uint8_t)(local_caps_ >> 8));
    ack.payload.push_back((uint8_t)(local_caps_ & 0xFF));
    // Append X25519 public key for DH key exchange
    if (has_local_x25519_) {
        ack.payload.insert(ack.payload.end(), local_x25519_pub_, local_x25519_pub_ + 32);
    }
    ack.payload.insert(ack.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(ack);
}

void ArqSession::handle_connect_ack(const ArqFrame& frame) {
    if (state_ != ArqState::CONNECTING)
        return;

    if (frame.payload.size() >= 2) {
        peer_caps_ = ((uint16_t)frame.payload[0] << 8) | frame.payload[1];
        // Check for X25519 pubkey: caps(2) + x25519(32) + callsign(1+)
        if ((peer_caps_ & CAP_ENCRYPTION) && frame.payload.size() >= 2 + 32 + 1) {
            memcpy(peer_x25519_pub_, frame.payload.data() + 2, 32);
            has_peer_x25519_ = true;
            remote_callsign_.assign(frame.payload.begin() + 34, frame.payload.end());
        } else {
            has_peer_x25519_ = false;
            remote_callsign_.assign(frame.payload.begin() + 2, frame.payload.end());
        }
    } else {
        peer_caps_ = 0;
        has_peer_x25519_ = false;
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    }

    // Reset per-session state to prevent stale flags from previous session
    batch_data_delivered_ = false;
    last_received_eob_seq_ = -1;

    set_state(ArqState::CONNECTED);
    chase_enable();  // Enable Chase combining for this session
    start_turboshift();
    send_next_data();
}

// --- Data and ACK handlers ---

void ArqSession::handle_data(const ArqFrame& frame) {
    if (role_ != ArqRole::RESPONDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    int seq = frame.seq % ARQ_WINDOW_SIZE;

    int offset = seq - (rx_expected_ % ARQ_WINDOW_SIZE);
    if (offset < 0) offset += ARQ_WINDOW_SIZE;

    if (offset < ARQ_WINDOW_SIZE) {
        int slot = seq;
        if (!rx_window_[slot].received) {
            rx_window_[slot].data = frame.payload;
            rx_window_[slot].received = true;
        }
    }

    // Track end-of-batch
    if (frame.flags & 0x80)
        last_received_eob_seq_ = seq;

    // Deliver in-order data (prevent re-delivery with batch_data_delivered_ flag)
    while (rx_window_[rx_expected_ % ARQ_WINDOW_SIZE].received) {
        int slot = rx_expected_ % ARQ_WINDOW_SIZE;
        if (callbacks_.on_data_received && !rx_window_[slot].data.empty() && !batch_data_delivered_)
            callbacks_.on_data_received(rx_window_[slot].data.data(),
                                         rx_window_[slot].data.size());
        rx_window_[slot] = RxSlot{};
        rx_expected_++;
    }

    // Send ACK with bitmask + SNR
    uint8_t ack_mask = 0;
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        int s = (rx_expected_ + i) % ARQ_WINDOW_SIZE;
        if (rx_window_[s].received)
            ack_mask |= (1 << i);
    }

    ArqFrame ack;
    ack.type = ArqType::ACK;
    ack.seq = (uint8_t)(rx_expected_ % ARQ_WINDOW_SIZE);
    ack.flags = ack_mask;
    int16_t snr_scaled = (int16_t)(local_snr_ * 100.0f);
    ack.payload.push_back((uint8_t)(snr_scaled & 0xFF));
    ack.payload.push_back((uint8_t)((snr_scaled >> 8) & 0xFF));
    send_arq_frame(ack);

    // End of batch check
    if ((frame.flags & 0x80) && rx_expected_ > 0) {
        bool all_done = true;
        for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
            if (rx_window_[i].received) { all_done = false; break; }
        }
        if (all_done) {
            batch_data_delivered_ = true;
            if (callbacks_.on_transfer_complete)
                callbacks_.on_transfer_complete(true);
        }
    }
}

void ArqSession::handle_ack(const ArqFrame& frame) {
    if (role_ != ArqRole::COMMANDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    // Extract SNR
    if (frame.payload.size() >= 2) {
        int16_t snr_scaled = (int16_t)((uint16_t)frame.payload[0] |
                                        ((uint16_t)frame.payload[1] << 8));
        remote_snr_ = snr_scaled / 100.0f;
    }

    int ack_seq = frame.seq;
    uint8_t ack_mask = frame.flags;

    // Measure RTT from oldest unacked frame
    if (tx_base_ < tx_next_) {
        int slot = tx_base_ % ARQ_WINDOW_SIZE;
        auto rtt = std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - tx_window_[slot].send_time).count();
        if (rtt > 0 && rtt < 30000)
            update_ack_timeout(rtt);
    }

    // Advance tx_base
    while ((tx_base_ % ARQ_WINDOW_SIZE) != ack_seq && tx_base_ < tx_next_) {
        int slot = tx_base_ % ARQ_WINDOW_SIZE;
        tx_window_[slot].acked = true;
        tx_base_++;
    }

    // Selective ACK bitmask
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (ack_mask & (1 << i)) {
            int s = (ack_seq + i) % ARQ_WINDOW_SIZE;
            int abs_seq = tx_base_ + i;
            if (abs_seq < tx_next_)
                tx_window_[s].acked = true;
        }
    }

    // Track consecutive acks/nacks
    if (frame.type == ArqType::ACK) {
        consec_data_acks_++;
        consec_data_nacks_ = 0;
        emergency_nack_count_ = 0;

        // Reset BREAK state on successful ACK
        if (break_phase_ != BreakPhase::NONE) {
            break_phase_ = BreakPhase::NONE;
            break_retries_ = 0;
        }
    } else {
        consec_data_nacks_++;
        consec_data_acks_ = 0;

        // Retransmit on NACK — enable Chase combining so the receiver
        // can accumulate LLRs from this retransmission with the previous
        // failed attempt (+3 dB effective SNR per combine).
        int slot = ack_seq % ARQ_WINDOW_SIZE;
        if (tx_window_[slot].sent && !tx_window_[slot].acked) {
            ArqFrame data_frame;
            data_frame.type = ArqType::DATA;
            data_frame.seq = (uint8_t)(ack_seq % ARQ_WINDOW_SIZE);
            data_frame.flags = 0;
            data_frame.payload = tx_window_[slot].data;
            send_arq_frame(data_frame);
            tx_window_[slot].retries++;
            retransmit_count_++;
        }
    }

    // SNR-based gearshift ladder (post-turboshift)
    if (turbo_phase_ == TurboPhase::DONE || turbo_phase_ == TurboPhase::NONE) {
        if (gearshift_cooldown_ > 0)
            gearshift_cooldown_--;

        if (consec_data_acks_ >= TURBO_CONSEC_ACKS_TO_UPSHIFT &&
            speed_level_ < NUM_SPEED_LEVELS - 1 &&
            gearshift_cooldown_ <= 0) {
            // SNR-based: use SNR to suggest target, upshift by 1
            int snr_suggested = snr_to_speed_level(remote_snr_);
            if (snr_suggested > speed_level_) {
                set_speed(speed_level_ + 1);
                consec_data_acks_ = 0;
                gearshift_just_applied_ = true;
            }
        }

        if (consec_data_nacks_ >= 2 && speed_level_ > 0) {
            set_speed(speed_level_ - 1);
            consec_data_nacks_ = 0;
            gearshift_just_applied_ = false;
            gearshift_cooldown_ = 3;  // wait 3 blocks before upshifting again
        }

        // Safety net: first NACK after an upshift triggers BREAK
        if (gearshift_just_applied_ && consec_data_nacks_ >= 1) {
            gearshift_just_applied_ = false;
            initiate_break();
            return;
        }
    }

    // Emergency BREAK on excessive block-level failures
    emergency_nack_count_ += (frame.type == ArqType::NACK) ? 1 : 0;
    if (emergency_nack_count_ >= BREAK_EMERGENCY_NACK_THRESHOLD &&
        break_phase_ == BreakPhase::NONE) {
        initiate_break();
        return;
    }

    // BREAK on excessive consecutive NACKs
    if (consec_data_nacks_ >= BREAK_NACK_THRESHOLD &&
        break_phase_ == BreakPhase::NONE) {
        initiate_break();
        return;
    }

    // Transfer complete check
    if (end_of_data_ && tx_base_ == tx_next_ && tx_data_queue_.empty()) {
        if (callbacks_.on_transfer_complete)
            callbacks_.on_transfer_complete(true);
        return;
    }

    send_next_data();
}

// --- Turboshift ---

void ArqSession::start_turboshift() {
    if (role_ != ArqRole::COMMANDER) return;

    if (remote_snr_ <= 0.0f) {
        turbo_phase_ = TurboPhase::DONE;
        return;
    }

    turbo_phase_ = TurboPhase::FORWARD;
    turbo_last_good_ = speed_level_;
    turbo_retries_ = 0;
    turbo_verification_count_ = 0;

    int suggested = snr_to_speed_level(remote_snr_);
    turbo_target_speed_ = std::max(suggested, speed_level_ + 1);
    turbo_target_speed_ = std::min(turbo_target_speed_, NUM_SPEED_LEVELS - 1);

    set_state(ArqState::TURBOSHIFT);
    turbo_probe_up();
}

void ArqSession::turbo_probe_up() {
    if (turbo_target_speed_ <= turbo_last_good_) {
        // Verification probe at ceiling: test the top config twice
        if (turbo_verification_count_ == 0 && turbo_last_good_ > 0) {
            turbo_verification_count_ = 1;
            ArqFrame frame;
            frame.type = ArqType::SET_SPEED;
            frame.seq = 0;
            frame.flags = (uint8_t)turbo_last_good_;
            send_arq_frame(frame);
            return;
        }
        turbo_settle();
        return;
    }

    ArqFrame frame;
    frame.type = ArqType::SET_SPEED;
    frame.seq = 0;
    frame.flags = (uint8_t)turbo_target_speed_;
    send_arq_frame(frame);
}

void ArqSession::handle_set_speed(const ArqFrame& frame) {
    int new_speed = frame.flags;
    new_speed = std::max(0, std::min(new_speed, NUM_SPEED_LEVELS - 1));
    set_speed(new_speed);

    ArqFrame ack;
    ack.type = ArqType::SPEED_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    send_arq_frame(ack);
}

void ArqSession::handle_speed_ack(const ArqFrame& frame) {
    if (turbo_phase_ == TurboPhase::FORWARD || turbo_phase_ == TurboPhase::REVERSE) {
        int acked_speed = frame.flags;
        set_speed(acked_speed);
        turbo_last_good_ = acked_speed;

        // Verification probe response
        if (turbo_verification_count_ > 0) {
            turbo_verification_count_++;
            if (turbo_verification_count_ >= 2) {
                // Verified — settle
                if (turbo_phase_ == TurboPhase::FORWARD) {
                    turbo_forward_done_ = true;
                    initiate_role_switch();
                } else {
                    turbo_settle();
                }
            }
            return;
        }

        if (acked_speed < NUM_SPEED_LEVELS - 1 && turbo_retries_ < TURBO_PROBE_RETRIES) {
            turbo_target_speed_ = acked_speed + 1;
            turbo_retries_++;
            turbo_probe_up();
        } else {
            if (turbo_phase_ == TurboPhase::FORWARD) {
                turbo_forward_done_ = true;
                initiate_role_switch();
            } else {
                turbo_settle();
            }
        }
    }
}

void ArqSession::turbo_settle() {
    set_speed(turbo_last_good_);
    turbo_phase_ = TurboPhase::DONE;
    set_state(ArqState::CONNECTED);
    consec_data_acks_ = 0;
    consec_data_nacks_ = 0;
    gearshift_just_applied_ = false;
}

// --- Role switching with buffer protection ---

void ArqSession::initiate_role_switch() {
    ArqFrame frame;
    frame.type = ArqType::SWITCH_ROLE;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);
}

void ArqSession::handle_switch_role(const ArqFrame& frame) {
    // Clear stale buffers before swapping roles
    clear_stale_buffers();

    if (role_ == ArqRole::COMMANDER) {
        role_ = ArqRole::RESPONDER;
        turbo_phase_ = TurboPhase::DONE;
        set_state(ArqState::CONNECTED);
    } else if (role_ == ArqRole::RESPONDER) {
        role_ = ArqRole::COMMANDER;
        // Reset TX window for new commander direction
        tx_base_ = 0;
        tx_next_ = 0;
        end_of_data_ = false;
        for (int i = 0; i < ARQ_WINDOW_SIZE; i++)
            tx_window_[i] = TxSlot{};

        if (!tx_data_queue_.empty()) {
            // Data waiting — skip turboshift, send immediately
            printf("[ARQ] Role switch complete, sending queued data (%d chunks)\n",
                   (int)tx_data_queue_.size());
            fflush(stdout);
            turbo_phase_ = TurboPhase::DONE;
            set_state(ArqState::CONNECTED);
            send_next_data();
        } else {
            // No data — turboshift probe (original turboshift path)
            turbo_phase_ = TurboPhase::REVERSE;
            turbo_retries_ = 0;
            turbo_last_good_ = speed_level_;
            turbo_verification_count_ = 0;

            int suggested = snr_to_speed_level(remote_snr_);
            turbo_target_speed_ = std::max(suggested, speed_level_ + 1);
            turbo_target_speed_ = std::min(turbo_target_speed_, NUM_SPEED_LEVELS - 1);

            set_state(ArqState::TURBOSHIFT);
            turbo_probe_up();
        }
    }

    // ACK the role switch
    if (frame.seq == 0) {
        ArqFrame ack;
        ack.type = ArqType::SWITCH_ROLE;
        ack.seq = 1; // seq=1 means ACK
        ack.flags = 0;
        send_arq_frame(ack);
    }
}

// --- Multi-phase BREAK recovery ---

void ArqSession::initiate_break() {
    break_recovery_speed_ = speed_level_;
    break_phase_ = BreakPhase::PHASE_1;
    break_retries_ = 0;
    consec_data_nacks_ = 0;
    emergency_nack_count_ = 0;
    gearshift_just_applied_ = false;

    printf("[ARQ] BREAK initiated at speed %d, dropping to 0\n", speed_level_);
    fflush(stdout);

    break_phase1();
}

void ArqSession::break_phase1() {
    // Phase 1: drop to lowest speed, send SET_CONFIG for sync
    set_speed(0);

    ArqFrame frame;
    frame.type = ArqType::BREAK;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);

    // Also send SET_CONFIG at speed 0 for guaranteed delivery
    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = 0; // config = speed 0
    send_arq_frame(config);
}

void ArqSession::break_phase2() {
    // Phase 2: probe at a recovery target using ladder-based drop
    int target = break_recovery_speed_;
    for (int i = 0; i < break_drop_step_ && target > 0; i++)
        target--;
    if (target < 0) target = 0;

    printf("[ARQ] BREAK phase 2: probing config %d (drop_step=%d)\n",
           target, break_drop_step_);
    fflush(stdout);

    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = (uint8_t)target;
    send_arq_frame(config);

    break_phase_ = BreakPhase::PHASE_2;
}

void ArqSession::break_probe_recovery() {
    // Phase 3: try progressively higher speeds
    int target = speed_level_ + 1;
    if (target >= NUM_SPEED_LEVELS) target = NUM_SPEED_LEVELS - 1;

    if (target <= speed_level_) {
        // At ceiling already, settle
        break_phase_ = BreakPhase::NONE;
        printf("[ARQ] BREAK recovery complete at speed %d\n", speed_level_);
        fflush(stdout);
        return;
    }

    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = (uint8_t)target;
    send_arq_frame(config);

    break_phase_ = BreakPhase::PHASE_3;
}

void ArqSession::break_exhausted() {
    // Fallback: stay at lowest working speed
    printf("[ARQ] BREAK exhausted, settling at speed %d\n", speed_level_);
    fflush(stdout);
    break_phase_ = BreakPhase::NONE;
    break_drop_step_ = std::min(break_drop_step_ * 2, 4);  // escalate for next BREAK
}

void ArqSession::handle_break(const ArqFrame&) {
    // Peer requested emergency fallback
    set_speed(0);

    ArqFrame ack;
    ack.type = ArqType::BREAK;
    ack.seq = 1; // seq=1 means ACK
    ack.flags = 0;
    send_arq_frame(ack);
}

void ArqSession::handle_set_config(const ArqFrame& frame) {
    int new_speed = frame.flags;
    new_speed = std::max(0, std::min(new_speed, NUM_SPEED_LEVELS - 1));
    set_speed(new_speed);

    ArqFrame ack;
    ack.type = ArqType::CONFIG_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    send_arq_frame(ack);
}

void ArqSession::handle_config_ack(const ArqFrame& frame) {
    int acked_speed = frame.flags;
    set_speed(acked_speed);

    switch (break_phase_) {
        case BreakPhase::PHASE_1:
            // Phase 1 ACK received — peer confirmed speed 0, now probe up
            break_phase2();
            break;
        case BreakPhase::PHASE_2:
            // Phase 2 ACK: recovery target confirmed
            break_phase_ = BreakPhase::PHASE_3;
            break_probe_recovery();
            break;
        case BreakPhase::PHASE_3:
            // Probe step confirmed, try higher
            break_probe_recovery();
            break;
        default:
            break;
    }
}

// --- Disconnect ---

void ArqSession::handle_disconnect(const ArqFrame&) {
    ArqFrame ack;
    ack.type = ArqType::DISCONNECT_ACK;
    ack.seq = 0;
    ack.flags = 0;
    send_arq_frame(ack);

    set_state(ArqState::IDLE);
    role_ = ArqRole::IDLE;
}

void ArqSession::disconnect() {
    if (state_ == ArqState::IDLE) return;

    set_state(ArqState::DISCONNECTING);

    ArqFrame frame;
    frame.type = ArqType::DISCONNECT;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);
}

// --- Timer tick ---

void ArqSession::tick() {
    auto now = Clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_send_time_).count();

    // Check rx_mute expiry
    if (rx_mute_ && now >= rx_mute_until_)
        rx_mute_ = false;

    if (state_ == ArqState::HAILING) {
        // HAIL beacon phase: re-send periodically
        if (elapsed_ms > HAIL_INTERVAL_MS) {
            if (hail_attempts_ >= HAIL_MAX_ATTEMPTS) {
                if (callbacks_.on_transfer_complete)
                    callbacks_.on_transfer_complete(false);
                reset();
                return;
            }
            ArqFrame frame;
            frame.type = ArqType::HAIL;
            frame.seq = 0;
            frame.flags = 0;
            frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
            send_arq_frame(frame);
            hail_attempts_++;
        }
    } else if (state_ == ArqState::CONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            connect_retries_++;
            if (connect_retries_ >= ARQ_MAX_RETRIES) {
                if (callbacks_.on_transfer_complete)
                    callbacks_.on_transfer_complete(false);
                reset();
                return;
            }
            ArqFrame frame;
            frame.type = ArqType::CONNECT;
            frame.seq = 0;
            frame.flags = (uint8_t)speed_level_;
            frame.payload.push_back((uint8_t)(local_caps_ >> 8));
            frame.payload.push_back((uint8_t)(local_caps_ & 0xFF));
            frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
            send_arq_frame(frame);
        }
    } else if ((state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT) &&
               role_ == ArqRole::COMMANDER) {

        // Batch-aware timeout
        int timeout = compute_batch_timeout(ARQ_WINDOW_SIZE, 200);
        if (elapsed_ms > timeout && tx_base_ < tx_next_) {
            int slot = tx_base_ % ARQ_WINDOW_SIZE;
            if (tx_window_[slot].sent && !tx_window_[slot].acked) {
                tx_window_[slot].retries++;
                retransmit_count_++;

                if (tx_window_[slot].retries >= ARQ_MAX_RETRIES) {
                    if (break_phase_ == BreakPhase::NONE) {
                        initiate_break();
                    } else {
                        // Already in BREAK and still failing
                        break_retries_++;
                        if (break_retries_ >= BREAK_MAX_RETRIES) {
                            break_exhausted();
                            if (callbacks_.on_transfer_complete)
                                callbacks_.on_transfer_complete(false);
                            reset();
                        } else {
                            break_phase1();
                        }
                    }
                    return;
                }

                ArqFrame frame;
                frame.type = ArqType::DATA;
                frame.seq = (uint8_t)(tx_base_ % ARQ_WINDOW_SIZE);
                frame.flags = 0;
                frame.payload = tx_window_[slot].data;
                send_arq_frame(frame);
            }
        }

        // BREAK phase timeouts
        if (break_phase_ != BreakPhase::NONE && elapsed_ms > ack_timeout_ms_ * 3) {
            break_retries_++;
            if (break_retries_ >= BREAK_MAX_RETRIES) {
                break_exhausted();
            } else {
                break_phase1();  // retry from phase 1
            }
        }

        // Turboshift timeout
        if (state_ == ArqState::TURBOSHIFT && elapsed_ms > ack_timeout_ms_ * 3) {
            turbo_settle();
        }
    } else if (state_ == ArqState::DISCONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            set_state(ArqState::IDLE);
            role_ = ArqRole::IDLE;
        }
    }
}

} // namespace iris
