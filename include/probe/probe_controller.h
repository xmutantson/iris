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

    void start_initiator(int sample_rate, float capture_seconds = 3.0f);   // Send probe tones + listen
    void start_responder(int sample_rate, float capture_seconds = 3.0f);   // Listen for probe tones
    void start_standalone(int sample_rate, float capture_seconds = 3.0f);  // TX + self-listen (debug)

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

    // Local capability flags to embed in probe result
    void set_local_caps(uint16_t caps) { local_caps_ = caps; }

    // OFDM PHY config to embed in probe result for parameter negotiation
    void set_local_ofdm_config(uint8_t cp, uint8_t pilot_carrier, uint8_t pilot_symbol, uint8_t nfft_code) {
        ofdm_cp_ = cp; ofdm_pilot_carrier_ = pilot_carrier;
        ofdm_pilot_symbol_ = pilot_symbol; ofdm_nfft_code_ = nfft_code;
    }

private:
    void generate_and_send_probe();
    void analyze_captured();
    void send_result(const ProbeResult& r);
    void try_finalize();

    ProbeState state_ = ProbeState::IDLE;
    int sample_rate_ = 48000;
    bool is_initiator_ = false;
    bool standalone_ = false;   // Debug mode: no peer, finalize after self-analysis

    // Capture buffer for listening phase
    std::vector<float> capture_buf_;
    int capture_samples_ = 0;
    int capture_max_ = 0;
    int capture_skip_ = 0;  // Samples to skip (self-hearing holdoff after TX)

    // Timeout counter (ticks)
    int timeout_ = 0;
    static constexpr int TIMEOUT_TICKS = 400;  // 20s at 50ms/tick (covers initiator's 10s capture window)

    // Completion tracking (both conditions needed to finalize)
    bool analysis_done_ = false;     // We've analyzed their probe tones
    bool got_peer_result_ = false;   // We've received their analysis of our probe

    // Results
    ProbeResult my_tx_result_;     // What they heard from us
    ProbeResult their_tx_result_;  // What we heard from them
    NegotiatedPassband negotiated_;
    bool has_results_ = false;

    uint16_t local_caps_ = 0;  // Embedded in probe result for peer capability exchange

    // OFDM PHY config (embedded in probe result for parameter negotiation)
    uint8_t ofdm_cp_ = 64;
    uint8_t ofdm_pilot_carrier_ = 4;
    uint8_t ofdm_pilot_symbol_ = 24;
    uint8_t ofdm_nfft_code_ = 2;  // 0=512, 1=256, 2=1024
};

} // namespace iris

#endif // IRIS_PROBE_CONTROLLER_H
