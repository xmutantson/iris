#ifndef IRIS_AX25_SESSION_H
#define IRIS_AX25_SESSION_H

#include "ax25/ax25_protocol.h"
#include "common/types.h"
#include <functional>
#include <queue>
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

    Ax25Session();

    void set_local_callsign(const std::string& call) { local_call_ = call; }
    void set_send_callback(SendFrameFunc cb) { send_frame_ = cb; }
    void set_data_callback(DataReceivedFunc cb) { data_received_ = cb; }
    void set_state_callback(StateChangedFunc cb) { state_changed_ = cb; }

    // Initiate outgoing connection (sends SABM)
    void connect(const std::string& remote_call);

    // Send data within connection (queues I-frame)
    void send_data(const uint8_t* data, size_t len);

    // Disconnect (sends DISC)
    void disconnect();

    // Handle received parsed AX.25 frame. Returns true if consumed.
    bool on_frame_received(const Ax25Frame& frame);

    // Timer tick — call every ~100ms
    void tick();

    // Reset to disconnected
    void reset();

    // Queries
    Ax25SessionState state() const { return state_; }
    const std::string& remote_callsign() const { return remote_call_; }
    int pending_frames() const;
    bool is_active() const { return state_ != Ax25SessionState::DISCONNECTED; }
    int retry_count() const { return retry_count_; }

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
    void retransmit_from(uint8_t nr);
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

    // AX.25 2.2 system parameters (Section 6)
    static constexpr int K = 7;             // Window size (max outstanding I-frames)
    static constexpr int N2 = 10;           // Max retries
    static constexpr int T1_TICKS = 30;     // 3.0s ack timeout (at 100ms tick rate)
    static constexpr int T2_TICKS = 3;      // 0.3s response delay timer
    static constexpr int T3_TICKS = 300;    // 30s idle supervision
    static constexpr int MAX_INFO = 256;    // Max I-frame info field bytes (N1)

    Ax25SessionState state_ = Ax25SessionState::DISCONNECTED;
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
    TxIFrame tx_window_[8];

    // Timers (counted in ticks)
    int t1_ = 0;       // Acknowledgment timer (T1)
    int t2_ = 0;       // Response delay timer (T2)
    int t3_ = 0;       // Idle supervision timer (T3)
    int retry_count_ = 0;  // RC — retry count

    // AX.25 2.2 state flags
    bool peer_busy_ = false;            // Remote station busy (sent RNR)
    bool own_busy_ = false;             // We are busy (would send RNR)
    bool reject_exception_ = false;     // REJ already sent, waiting for correct I-frame
    bool acknowledge_pending_ = false;  // Need to send acknowledgment (T2 driven)
    bool srej_enabled_ = false;         // SREJ supported (negotiated via XID)
    uint8_t last_received_ctrl_ = 0;    // For FRMR info field

    // Addresses (built once per connection)
    Ax25Address local_addr_;
    Ax25Address remote_addr_;

    // Callbacks
    SendFrameFunc send_frame_;
    DataReceivedFunc data_received_;
    StateChangedFunc state_changed_;
};

} // namespace iris

#endif // IRIS_AX25_SESSION_H
