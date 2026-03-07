#include "arq/arq.h"
#include <cstring>
#include <algorithm>

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
    retransmit_count_ = 0;
    tx_base_ = 0;
    tx_next_ = 0;
    end_of_data_ = false;
    rx_expected_ = 0;
    connect_retries_ = 0;

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

void ArqSession::send_arq_frame(const ArqFrame& frame) {
    auto data = frame.serialize();
    if (callbacks_.send_frame)
        callbacks_.send_frame(data.data(), data.size());
    last_send_time_ = Clock::now();
}

void ArqSession::connect(const std::string& remote_callsign) {
    reset();
    role_ = ArqRole::COMMANDER;
    remote_callsign_ = remote_callsign;
    set_state(ArqState::CONNECTING);
    connect_time_ = Clock::now();
    connect_retries_ = 0;

    ArqFrame frame;
    frame.type = ArqType::CONNECT;
    frame.seq = 0;
    frame.flags = (uint8_t)speed_level_;
    frame.payload.assign(callsign_.begin(), callsign_.end());
    send_arq_frame(frame);
}

void ArqSession::send_data(const uint8_t* data, size_t len) {
    // Fragment into ARQ_MAX_PAYLOAD chunks and queue
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = std::min((size_t)ARQ_MAX_PAYLOAD, len - offset);
        tx_data_queue_.push(std::vector<uint8_t>(data + offset, data + offset + chunk));
        offset += chunk;
    }
    end_of_data_ = false;

    // Start sending if connected
    if (state_ == ArqState::CONNECTED && role_ == ArqRole::COMMANDER)
        send_next_data();
}

int ArqSession::tx_queue_bytes() const {
    int total = 0;
    // Count queued data
    // Can't iterate std::queue without copy, approximate from queue size
    return total + (int)tx_data_queue_.size() * ARQ_MAX_PAYLOAD;
}

void ArqSession::send_next_data() {
    if (state_ != ArqState::CONNECTED || role_ != ArqRole::COMMANDER)
        return;

    // Fill TX window slots
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

            ArqFrame frame;
            frame.type = ArqType::DATA;
            frame.seq = (uint8_t)(tx_next_ % ARQ_WINDOW_SIZE);
            frame.flags = 0;

            // Mark end-of-session if this is the last chunk
            if (tx_data_queue_.empty()) {
                frame.flags |= 0x80;  // end_of_session bit
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

void ArqSession::on_frame_received(const uint8_t* data, size_t len) {
    ArqFrame frame;
    if (!ArqFrame::deserialize(data, len, frame))
        return;

    switch (frame.type) {
        case ArqType::CONNECT:
            handle_connect(frame);
            break;
        case ArqType::CONNECT_ACK:
            handle_connect_ack(frame);
            break;
        case ArqType::DATA:
            handle_data(frame);
            break;
        case ArqType::ACK:
        case ArqType::NACK:
            handle_ack(frame);
            break;
        case ArqType::DISCONNECT:
            handle_disconnect(frame);
            break;
        case ArqType::DISCONNECT_ACK:
            if (state_ == ArqState::DISCONNECTING) {
                set_state(ArqState::IDLE);
                role_ = ArqRole::IDLE;
            }
            break;
    }
}

void ArqSession::handle_connect(const ArqFrame& frame) {
    if (state_ != ArqState::IDLE && state_ != ArqState::CONNECTING)
        return;

    role_ = ArqRole::RESPONDER;
    remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    set_state(ArqState::CONNECTED);

    ArqFrame ack;
    ack.type = ArqType::CONNECT_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    ack.payload.assign(callsign_.begin(), callsign_.end());
    send_arq_frame(ack);
}

void ArqSession::handle_connect_ack(const ArqFrame& frame) {
    if (state_ != ArqState::CONNECTING)
        return;

    remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    set_state(ArqState::CONNECTED);

    // Start sending data
    send_next_data();
}

void ArqSession::handle_data(const ArqFrame& frame) {
    if (role_ != ArqRole::RESPONDER || state_ != ArqState::CONNECTED)
        return;

    int seq = frame.seq % ARQ_WINDOW_SIZE;

    // Check if within expected window
    int offset = seq - (rx_expected_ % ARQ_WINDOW_SIZE);
    if (offset < 0) offset += ARQ_WINDOW_SIZE;

    if (offset < ARQ_WINDOW_SIZE) {
        int slot = seq;
        if (!rx_window_[slot].received) {
            rx_window_[slot].data = frame.payload;
            rx_window_[slot].received = true;
        }
    }

    // Deliver in-order data and advance window
    while (rx_window_[rx_expected_ % ARQ_WINDOW_SIZE].received) {
        int slot = rx_expected_ % ARQ_WINDOW_SIZE;
        if (callbacks_.on_data_received && !rx_window_[slot].data.empty())
            callbacks_.on_data_received(rx_window_[slot].data.data(),
                                         rx_window_[slot].data.size());
        rx_window_[slot] = RxSlot{};
        rx_expected_++;
    }

    // Send ACK with bitmask of received frames
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
    // Include SNR report (scaled by 100)
    int16_t snr_scaled = (int16_t)(remote_snr_ * 100.0f);
    ack.payload.push_back((uint8_t)(snr_scaled & 0xFF));
    ack.payload.push_back((uint8_t)((snr_scaled >> 8) & 0xFF));
    send_arq_frame(ack);

    // If end_of_session flag set and all delivered
    if ((frame.flags & 0x80) && rx_expected_ > 0) {
        // Check if we've received everything up to this point
        bool all_done = true;
        for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
            if (rx_window_[i].received) {
                all_done = false;
                break;
            }
        }
        if (all_done) {
            if (callbacks_.on_transfer_complete)
                callbacks_.on_transfer_complete(true);
        }
    }
}

void ArqSession::handle_ack(const ArqFrame& frame) {
    if (role_ != ArqRole::COMMANDER || state_ != ArqState::CONNECTED)
        return;

    // Extract SNR from payload
    if (frame.payload.size() >= 2) {
        int16_t snr_scaled = (int16_t)((uint16_t)frame.payload[0] |
                                        ((uint16_t)frame.payload[1] << 8));
        remote_snr_ = snr_scaled / 100.0f;
    }

    int ack_seq = frame.seq;  // Next expected by responder
    uint8_t ack_mask = frame.flags;

    // Advance tx_base to ack_seq (all before it are confirmed)
    while ((tx_base_ % ARQ_WINDOW_SIZE) != ack_seq &&
           tx_base_ < tx_next_) {
        int slot = tx_base_ % ARQ_WINDOW_SIZE;
        tx_window_[slot].acked = true;
        tx_base_++;
    }

    // Mark selectively acked frames from bitmask
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (ack_mask & (1 << i)) {
            int s = (ack_seq + i) % ARQ_WINDOW_SIZE;
            // Only mark if within our TX window
            int abs_seq = tx_base_ + i;
            if (abs_seq < tx_next_) {
                tx_window_[s].acked = true;
            }
        }
    }

    // NACK: retransmit specific frame
    if (frame.type == ArqType::NACK) {
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

    // Check if all data transferred
    if (end_of_data_ && tx_base_ == tx_next_ && tx_data_queue_.empty()) {
        if (callbacks_.on_transfer_complete)
            callbacks_.on_transfer_complete(true);
        return;
    }

    // Send more data
    send_next_data();
}

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
    if (state_ == ArqState::IDLE)
        return;

    set_state(ArqState::DISCONNECTING);

    ArqFrame frame;
    frame.type = ArqType::DISCONNECT;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);
}

void ArqSession::tick() {
    auto now = Clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_send_time_).count();

    if (state_ == ArqState::CONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            connect_retries_++;
            if (connect_retries_ >= ARQ_MAX_RETRIES) {
                if (callbacks_.on_transfer_complete)
                    callbacks_.on_transfer_complete(false);
                reset();
                return;
            }
            // Resend CONNECT
            ArqFrame frame;
            frame.type = ArqType::CONNECT;
            frame.seq = 0;
            frame.flags = (uint8_t)speed_level_;
            frame.payload.assign(callsign_.begin(), callsign_.end());
            send_arq_frame(frame);
        }
    } else if (state_ == ArqState::CONNECTED && role_ == ArqRole::COMMANDER) {
        if (elapsed_ms > ARQ_ACK_TIMEOUT_MS && tx_base_ < tx_next_) {
            // Retransmit oldest unacked frame
            int slot = tx_base_ % ARQ_WINDOW_SIZE;
            if (tx_window_[slot].sent && !tx_window_[slot].acked) {
                tx_window_[slot].retries++;
                retransmit_count_++;

                if (tx_window_[slot].retries >= ARQ_MAX_RETRIES) {
                    if (callbacks_.on_transfer_complete)
                        callbacks_.on_transfer_complete(false);
                    reset();
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
    } else if (state_ == ArqState::DISCONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            // Give up waiting for DISCONNECT_ACK
            set_state(ArqState::IDLE);
            role_ = ArqRole::IDLE;
        }
    }
}

} // namespace iris
