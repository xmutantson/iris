#ifndef IRIS_ARQ_H
#define IRIS_ARQ_H

#include "engine/speed_level.h"
#include "fec/ldpc.h"
#include <vector>
#include <cstdint>
#include <functional>
#include <string>
#include <queue>
#include <chrono>

namespace iris {

// ARQ frame types (first byte of ARQ payload)
enum class ArqType : uint8_t {
    CONNECT      = 0x01,  // Connection request
    CONNECT_ACK  = 0x02,  // Connection accepted
    DATA         = 0x10,  // Data frame
    ACK          = 0x20,  // Acknowledgment
    NACK         = 0x21,  // Negative acknowledgment (retransmit request)
    DISCONNECT   = 0x30,  // Disconnect
    DISCONNECT_ACK = 0x31,
};

// ARQ roles
enum class ArqRole {
    IDLE,
    COMMANDER,   // Initiates transfer (sends DATA)
    RESPONDER,   // Receives data (sends ACK)
};

// ARQ connection state
enum class ArqState {
    IDLE,
    CONNECTING,      // Commander: sent CONNECT, awaiting CONNECT_ACK
    CONNECTED,       // Session active
    DISCONNECTING,   // Sent DISCONNECT, awaiting DISCONNECT_ACK
};

// Sliding window size (3-bit sequence, window of 8)
static constexpr int ARQ_WINDOW_SIZE = 8;
static constexpr int ARQ_MAX_RETRIES = 10;
static constexpr int ARQ_CONNECT_TIMEOUT_MS = 3000;
static constexpr int ARQ_ACK_TIMEOUT_MS = 2000;
static constexpr int ARQ_MAX_PAYLOAD = 256;  // Max data bytes per DATA frame

// ARQ control frame layout:
//   [1 byte type][1 byte seq][1 byte flags/ack_mask][N bytes payload]
// CONNECT:       type=0x01, seq=0, flags=speed_level, payload=callsign
// CONNECT_ACK:   type=0x02, seq=0, flags=speed_level
// DATA:          type=0x10, seq=0-7, flags=[end_of_session:1][reserved:7], payload=data
// ACK:           type=0x20, seq=next_expected, flags=ack_bitmask, payload=[2 bytes snr*100]
// NACK:          type=0x21, seq=missing_seq, flags=ack_bitmask
// DISCONNECT:    type=0x30
// DISCONNECT_ACK:type=0x31

struct ArqFrame {
    ArqType type;
    uint8_t seq;
    uint8_t flags;
    std::vector<uint8_t> payload;

    std::vector<uint8_t> serialize() const;
    static bool deserialize(const uint8_t* data, size_t len, ArqFrame& out);
};

// Callbacks for ARQ events
struct ArqCallbacks {
    // Send a frame over the air (caller handles PHY/modulation)
    std::function<void(const uint8_t* data, size_t len)> send_frame;
    // Received application data
    std::function<void(const uint8_t* data, size_t len)> on_data_received;
    // Session state changed
    std::function<void(ArqState state)> on_state_changed;
    // Transfer complete (all data acknowledged)
    std::function<void(bool success)> on_transfer_complete;
};

class ArqSession {
public:
    ArqSession();

    void set_callbacks(const ArqCallbacks& cb) { callbacks_ = cb; }
    void set_callsign(const std::string& cs) { callsign_ = cs; }

    // Commander: start a connection and queue data for transfer
    void connect(const std::string& remote_callsign);
    void send_data(const uint8_t* data, size_t len);

    // Process a received ARQ frame (called by modem when frame arrives)
    void on_frame_received(const uint8_t* data, size_t len);

    // Timer tick — call periodically (~100ms) for retransmit/timeout logic
    void tick();

    // Disconnect gracefully
    void disconnect();

    // Accessors
    ArqRole role() const { return role_; }
    ArqState state() const { return state_; }
    int speed_level() const { return speed_level_; }
    float remote_snr() const { return remote_snr_; }
    int tx_queue_bytes() const;
    int retransmit_count() const { return retransmit_count_; }

    void reset();

private:
    void send_arq_frame(const ArqFrame& frame);
    void send_next_data();
    void process_ack(const ArqFrame& ack);
    void handle_connect(const ArqFrame& frame);
    void handle_connect_ack(const ArqFrame& frame);
    void handle_data(const ArqFrame& frame);
    void handle_ack(const ArqFrame& frame);
    void handle_disconnect(const ArqFrame& frame);

    void set_state(ArqState s);

    ArqCallbacks callbacks_;
    std::string callsign_;
    std::string remote_callsign_;

    ArqRole role_ = ArqRole::IDLE;
    ArqState state_ = ArqState::IDLE;
    int speed_level_ = 0;
    float remote_snr_ = 0;
    int retransmit_count_ = 0;

    // Commander TX window
    struct TxSlot {
        std::vector<uint8_t> data;
        bool acked = false;
        bool sent = false;
        int retries = 0;
    };
    TxSlot tx_window_[ARQ_WINDOW_SIZE];
    int tx_base_ = 0;       // Base sequence number (oldest unacked)
    int tx_next_ = 0;       // Next sequence to send
    bool end_of_data_ = false;  // All data queued

    // Data queue (commander side)
    std::queue<std::vector<uint8_t>> tx_data_queue_;

    // Responder RX window
    struct RxSlot {
        std::vector<uint8_t> data;
        bool received = false;
    };
    RxSlot rx_window_[ARQ_WINDOW_SIZE];
    int rx_expected_ = 0;    // Next expected sequence number

    // Timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_send_time_;
    Clock::time_point connect_time_;
    int connect_retries_ = 0;
};

} // namespace iris

#endif
