#ifndef IRIS_AX25_SESSION_H
#define IRIS_AX25_SESSION_H

#include "ax25/ax25_protocol.h"
#include "common/types.h"
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

    Ax25Session();

    void set_local_callsign(const std::string& call) { local_call_ = call; }
    void set_send_callback(SendFrameFunc cb) { send_frame_ = cb; }
    void set_data_callback(DataReceivedFunc cb) { data_received_ = cb; }
    void set_state_callback(StateChangedFunc cb) { state_changed_ = cb; }
    void set_t1_ticks(int ticks) { t1_value_ = ticks; }

    // Update T1 floor based on TNC TXDELAY (accounts for real turnaround time).
    // T1 >= 2*TXDELAY + 2s processing/frame-time, but never below FRACK default.
    void set_txdelay_ms(int ms);

    // Update T3 (idle supervision) in ticks
    void set_t3_ticks(int ticks) { t3_value_ = ticks; }

    // Channel busy feedback (Direwolf pattern): pause T1/T3 while DCD or PTT
    // is active.  On half-duplex radio, we can't expect an acknowledgment while
    // the channel is occupied — either by the remote station (DCD) or by our
    // own transmission (PTT).  Prevents false T1 timeouts and unnecessary
    // retransmissions.  Call from modem when DCD or PTT state changes.
    void set_channel_busy(bool busy);

    // Initiate outgoing connection (sends SABM)
    void connect(const std::string& remote_call);

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

    // Queries
    Ax25SessionState state() const { return state_; }
    const std::string& remote_callsign() const { return remote_call_; }
    int pending_frames() const;
    bool is_active() const { return state_ != Ax25SessionState::DISCONNECTED; }
    bool we_initiated() const { return we_initiated_; }
    bool is_kiss_managed() const { return kiss_managed_; }
    void set_kiss_passthrough(bool v) { kiss_passthrough_ = v; }
    bool is_kiss_passthrough() const { return kiss_passthrough_; }
    int retry_count() const { return retry_count_; }

    // Flow control: set receiver busy (sends RNR instead of RR to polls).
    // Use during modem reconfiguration (speed change, probe) to pause
    // the peer's I-frame transmissions without dropping the connection.
    void set_own_busy(bool busy);
    bool own_busy() const { return own_busy_; }

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
    void update_srt(int rtt);

    // AX.25 2.2 system parameters (Section 6)
    // Tick rate: 50ms (main loop sleeps 50ms, modem.tick() calls ax25_session.tick())
    static constexpr int K = 7;             // Window size (max outstanding I-frames)
    static constexpr int N2 = 10;           // Max retries (Direwolf default: 10)
public:
    static constexpr int T1_TICKS = 80;     // 4.0s ack timeout (Direwolf FRACK=4)
private:
    static constexpr int T2_TICKS = 6;      // 0.3s response delay timer
    static constexpr int T3_TICKS = 6000;   // 300s idle supervision (Direwolf default: 5 min)
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
    bool srej_enabled_ = false;         // SREJ supported (negotiated via XID)
    uint8_t last_received_ctrl_ = 0;    // For FRMR info field
    bool kiss_managed_ = false;          // Session initiated by KISS client (don't generate SABM/DISC retries)
    bool we_initiated_ = false;          // We sent SABM (true) vs received SABM (false)
    bool kiss_passthrough_ = false;      // KISS client active — don't accept incoming connections
    // Channel busy: pause T1/T3 while DCD or PTT active (Direwolf pattern).
    // Remaining ticks saved on pause, restored on resume.
    bool channel_busy_ = false;
    int t1_paused_remaining_ = 0;       // T1 ticks remaining when paused (0 = not paused)
    int t3_paused_remaining_ = 0;       // T3 ticks remaining when paused

    // Adaptive T1 via Smoothed Round-Trip Time (Direwolf ax25_link.c:6304-6397).
    // Measures actual RTT from SABM→UA and poll→response, then adapts T1.
    // SRT update: first measurement SRT=RTT, subsequent SRT = 7/8*SRT + 1/8*RTT.
    // T1 = max(floor, 2*SRT).
    int wall_ticks_ = 0;                // Always-incrementing tick counter (not paused)
    int rtt_start_ = 0;                 // wall_ticks_ when measurement started
    bool measuring_rtt_ = false;        // RTT measurement in progress
    int srt_ticks_ = 0;                 // Smoothed RTT in ticks (0 = not yet measured)

    // T1 retry jitter: breaks synchronized collisions (both sides retrying at
    // exact same interval).  0-25% jitter desynchronizes after 1-2 retries.
    std::minstd_rand rng_{std::random_device{}()};
    int t1_with_jitter() { return t1_value_ + rng_() % (t1_value_ / 4 + 1); }

    // Addresses (built once per connection)
    Ax25Address local_addr_;
    Ax25Address remote_addr_;

    // Thread safety: protects timer state (T1/T3/paused/SRT) from concurrent
    // access by audio callback thread (set_channel_busy, on_frame_received)
    // and main loop thread (tick).
    mutable std::mutex timer_mutex_;

    // Callbacks
    SendFrameFunc send_frame_;
    DataReceivedFunc data_received_;
    StateChangedFunc state_changed_;
};

} // namespace iris

#endif // IRIS_AX25_SESSION_H
