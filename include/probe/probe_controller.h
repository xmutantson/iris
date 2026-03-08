#ifndef IRIS_PROBE_CONTROLLER_H
#define IRIS_PROBE_CONTROLLER_H

#include "probe/passband_probe.h"
#include <vector>
#include <functional>

namespace iris {

// Half-duplex passband probe protocol (3 turns after XID):
//
//   Turn 1 (A→B): probe tones
//   Turn 2 (B→A): RESULT + probe tones
//   Turn 3 (A→B): RESULT
//
// The modem calls start_initiator/start_responder after XID handshake.
// feed_rx() receives captured audio; tick() drives timeouts.
// Callbacks fire when probe audio or protocol messages need to be sent.

class ProbeController {
public:
    // Callbacks
    std::function<void(const float*, int)> on_send_audio;   // Queue probe audio for TX
    std::function<void(const uint8_t*, size_t)> on_send_msg; // Send probe protocol msg

    void start_initiator(int sample_rate);   // Send probe tones + listen
    void start_responder(int sample_rate);   // Listen for probe tones

    // Feed captured RX audio during listening phase
    void feed_rx(const float* audio, int count);

    // Periodic tick (~100ms) to drive timeouts
    void tick();

    // Handle incoming probe protocol messages (first byte = type)
    void on_message(const uint8_t* data, size_t len);

    // State accessors
    ProbeState state() const { return state_; }
    bool is_done() const { return state_ == ProbeState::DONE; }
    bool has_results() const { return has_results_; }
    const ProbeResult& my_tx_result() const { return my_tx_result_; }
    const ProbeResult& their_tx_result() const { return their_tx_result_; }
    const NegotiatedPassband& negotiated() const { return negotiated_; }

    void reset();

private:
    void generate_and_send_probe();
    void analyze_captured();
    void send_result(const ProbeResult& r);
    void try_finalize();

    ProbeState state_ = ProbeState::IDLE;
    int sample_rate_ = 48000;
    bool is_initiator_ = false;

    // Capture buffer for listening phase
    std::vector<float> capture_buf_;
    int capture_samples_ = 0;
    int capture_max_ = 0;
    int capture_skip_ = 0;  // Samples to skip (self-hearing holdoff after TX)

    // Timeout counter (ticks)
    int timeout_ = 0;
    static constexpr int TIMEOUT_TICKS = 100;  // 10 seconds at 100ms/tick

    // Completion tracking (both conditions needed to finalize)
    bool analysis_done_ = false;     // We've analyzed their probe tones
    bool got_peer_result_ = false;   // We've received their analysis of our probe

    // Results
    ProbeResult my_tx_result_;     // What they heard from us
    ProbeResult their_tx_result_;  // What we heard from them
    NegotiatedPassband negotiated_;
    bool has_results_ = false;
};

} // namespace iris

#endif // IRIS_PROBE_CONTROLLER_H
