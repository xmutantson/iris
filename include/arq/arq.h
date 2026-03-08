#ifndef IRIS_ARQ_H
#define IRIS_ARQ_H

#include "engine/speed_level.h"
#include "common/types.h"
#include <vector>
#include <cstdint>
#include <functional>
#include <string>
#include <queue>
#include <chrono>

namespace iris {

// ARQ frame types (first byte of ARQ payload)
enum class ArqType : uint8_t {
    HAIL         = 0x00,  // Beacon to detect responder
    HAIL_ACK     = 0x01,  // Responder detected beacon
    CONNECT      = 0x02,  // Connection request (+ capabilities)
    CONNECT_ACK  = 0x03,  // Connection accepted (+ capabilities)
    DATA         = 0x10,  // Data frame
    ACK          = 0x20,  // Acknowledgment
    NACK         = 0x21,  // Negative acknowledgment
    SET_SPEED    = 0x25,  // Speed change request (turboshift)
    SPEED_ACK    = 0x26,  // Speed change acknowledged
    SET_CONFIG   = 0x27,  // Config sync for BREAK recovery
    CONFIG_ACK   = 0x28,  // Config sync acknowledged
    SWITCH_ROLE  = 0x2A,  // Role switch for bidirectional turboshift
    BREAK        = 0x2C,  // Emergency fallback to lowest speed
    DISCONNECT   = 0x30,  // Disconnect
    DISCONNECT_ACK = 0x31,
};

// ARQ roles
enum class ArqRole {
    IDLE,
    COMMANDER,   // Initiates transfer (sends DATA)
    RESPONDER,   // Receives data (sends ACK)
    LISTENING,   // Waiting for HAIL beacons
};

// ARQ connection state
enum class ArqState {
    IDLE,
    LISTENING,       // Responder: waiting for HAIL beacon
    HAILING,         // Commander: sending HAIL beacons
    CONNECTING,      // Commander: sent CONNECT, awaiting CONNECT_ACK
    CONNECTED,       // Session active
    TURBOSHIFT,      // Active speed probing
    DISCONNECTING,   // Sent DISCONNECT, awaiting DISCONNECT_ACK
};

// Turboshift phase
enum class TurboPhase {
    NONE,
    FORWARD,    // Commander probing upward
    REVERSE,    // Responder probing upward (after role switch)
    DONE,       // Both sides settled
};

// BREAK recovery phase
enum class BreakPhase {
    NONE,
    PHASE_1,    // Send SET_CONFIG at lowest speed, wait for CONFIG_ACK
    PHASE_2,    // Send SET_CONFIG at probe target, wait for CONFIG_ACK
    PHASE_3,    // Probe-based upward recovery
};

// Sliding window size (3-bit sequence, window of 8)
static constexpr int ARQ_WINDOW_SIZE = 8;
static constexpr int ARQ_MAX_RETRIES = 10;
static constexpr int ARQ_CONNECT_TIMEOUT_MS = 5000;
static constexpr int ARQ_DEFAULT_ACK_TIMEOUT_MS = 2000;
static constexpr int ARQ_MIN_ACK_TIMEOUT_MS = 500;
static constexpr int ARQ_MAX_ACK_TIMEOUT_MS = 10000;
static constexpr int ARQ_MAX_PAYLOAD = 256;

// HAIL beacon constants
static constexpr int HAIL_INTERVAL_MS = 3500;   // Must exceed full round-trip: TX(717ms) + responder accumulate/decode(~800ms) + response TX(717ms)
static constexpr int HAIL_MAX_ATTEMPTS = 10;     // ~35 seconds of hailing

// Turboshift constants
static constexpr int TURBO_PROBE_RETRIES = 2;
static constexpr int TURBO_CONSEC_ACKS_TO_UPSHIFT = 3;

// BREAK recovery constants
static constexpr int BREAK_NACK_THRESHOLD = 3;
static constexpr int BREAK_MAX_RETRIES = 3;
static constexpr int BREAK_EMERGENCY_NACK_THRESHOLD = 2;  // consecutive bad blocks

// Frame layout descriptions:
// HAIL:      [type:1][seq:1][flags:1][callsign:N]
// CONNECT:   [type:1][seq:1][flags:1][caps_hi:1][caps_lo:1][speed:1][callsign:N]
// DATA:      [type:1][seq:1][flags:1][payload:N]  (flags bit7 = end-of-batch)
// ACK:       [type:1][seq:1][ack_mask:1][snr_lo:1][snr_hi:1]
// SET_SPEED: [type:1][seq:1][new_speed:1]
// SET_CONFIG:[type:1][seq:1][config:1]
// BREAK:     [type:1][seq:1][flags:1]

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
    std::function<void(const uint8_t* data, size_t len)> send_frame;
    std::function<void(const uint8_t* data, size_t len)> on_data_received;
    std::function<void(ArqState state)> on_state_changed;
    std::function<void(bool success)> on_transfer_complete;
    std::function<void(int speed_level)> on_speed_changed;
};

class ArqSession {
public:
    ArqSession();

    void set_callbacks(const ArqCallbacks& cb) { callbacks_ = cb; }
    void set_callsign(const std::string& cs) { callsign_ = cs; }
    void set_local_capabilities(uint16_t caps) { local_caps_ = caps; }
    void set_local_snr(float snr) { local_snr_ = snr; }

    // Commander: start a connection and queue data for transfer
    void connect(const std::string& remote_callsign);
    void send_data(const uint8_t* data, size_t len);

    // Responder: start listening for HAIL beacons
    void listen();

    // Process a received ARQ frame (called by modem when frame arrives)
    // Returns true if frame was handled as ARQ, false if not an ARQ frame
    bool on_frame_received(const uint8_t* data, size_t len);

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
    int pending_frames() const;
    int retransmit_count() const { return retransmit_count_; }
    const std::string& remote_callsign() const { return remote_callsign_; }
    uint16_t peer_capabilities() const { return peer_caps_; }
    bool peer_has(uint16_t cap) const { return (peer_caps_ & cap) != 0; }
    bool negotiated(uint16_t cap) const { return (local_caps_ & peer_caps_ & cap) != 0; }
    TurboPhase turbo_phase() const { return turbo_phase_; }
    int ack_timeout_ms() const { return ack_timeout_ms_; }

    void reset();

private:
    void send_arq_frame(const ArqFrame& frame);
    void send_next_data();
    void handle_hail(const ArqFrame& frame);
    void handle_hail_ack(const ArqFrame& frame);
    void handle_connect(const ArqFrame& frame);
    void handle_connect_ack(const ArqFrame& frame);
    void handle_data(const ArqFrame& frame);
    void handle_ack(const ArqFrame& frame);
    void handle_set_speed(const ArqFrame& frame);
    void handle_speed_ack(const ArqFrame& frame);
    void handle_set_config(const ArqFrame& frame);
    void handle_config_ack(const ArqFrame& frame);
    void handle_switch_role(const ArqFrame& frame);
    void handle_break(const ArqFrame& frame);
    void handle_disconnect(const ArqFrame& frame);

    void set_state(ArqState s);
    void set_speed(int level);
    void update_ack_timeout(int64_t rtt_ms);

    // Batch-aware timeout calculation
    int compute_batch_timeout(int batch_size, int msg_tx_time_ms) const;

    // Turboshift logic
    void start_turboshift();
    void turbo_probe_up();
    void turbo_settle();
    void initiate_role_switch();

    // Multi-phase BREAK recovery
    void initiate_break();
    void break_phase1();
    void break_phase2();
    void break_probe_recovery();
    void break_exhausted();

    // Role-switch buffer protection
    void clear_stale_buffers();

    ArqCallbacks callbacks_;
    std::string callsign_;
    std::string remote_callsign_;

    ArqRole role_ = ArqRole::IDLE;
    ArqState state_ = ArqState::IDLE;
    int speed_level_ = 0;
    float remote_snr_ = 0;
    float local_snr_ = 0;
    int retransmit_count_ = 0;

    // Capability negotiation
    uint16_t local_caps_ = 0;
    uint16_t peer_caps_ = 0;

    // Adaptive timeout
    int ack_timeout_ms_ = ARQ_DEFAULT_ACK_TIMEOUT_MS;
    int64_t rtt_sum_ms_ = 0;
    int rtt_count_ = 0;

    // Commander TX window
    using Clock = std::chrono::steady_clock;
    struct TxSlot {
        std::vector<uint8_t> data;
        bool acked = false;
        bool sent = false;
        int retries = 0;
        Clock::time_point send_time;
    };
    TxSlot tx_window_[ARQ_WINDOW_SIZE];
    int tx_base_ = 0;
    int tx_next_ = 0;
    bool end_of_data_ = false;

    std::queue<std::vector<uint8_t>> tx_data_queue_;

    // Responder RX window
    struct RxSlot {
        std::vector<uint8_t> data;
        bool received = false;
    };
    RxSlot rx_window_[ARQ_WINDOW_SIZE];
    int rx_expected_ = 0;

    // Batch tracking
    bool batch_data_delivered_ = false;
    int last_received_eob_seq_ = -1;  // end-of-batch seq, or -1

    // Timing
    Clock::time_point last_send_time_;
    Clock::time_point connect_time_;
    int connect_retries_ = 0;

    // HAIL beacon state
    int hail_attempts_ = 0;

    // Turboshift state
    TurboPhase turbo_phase_ = TurboPhase::NONE;
    int turbo_target_speed_ = 0;
    int turbo_last_good_ = 0;
    int turbo_retries_ = 0;
    bool turbo_forward_done_ = false;
    int turbo_verification_count_ = 0;  // verification probe at ceiling

    // SNR-based gearshift ladder (post-turboshift)
    int consec_data_acks_ = 0;
    int consec_data_nacks_ = 0;
    bool gearshift_just_applied_ = false;  // safety net for upshift failures
    int gearshift_cooldown_ = 0;          // blocks to wait before next upshift

    // Multi-phase BREAK recovery
    BreakPhase break_phase_ = BreakPhase::NONE;
    int break_recovery_speed_ = 0;
    int break_retries_ = 0;
    int break_drop_step_ = 1;        // ladder step (1, 2, 4, 4, ...)
    int emergency_nack_count_ = 0;   // consecutive bad blocks

    // Role-switch protection
    bool rx_mute_ = false;
    Clock::time_point rx_mute_until_;
};

} // namespace iris

#endif
