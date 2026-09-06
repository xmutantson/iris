#ifndef IRIS_PROBE_CONTROLLER_H
#define IRIS_PROBE_CONTROLLER_H

#include "probe/passband_probe.h"
#include <vector>
#include <functional>

namespace iris {

// Fixed-timing slot probe protocol (after AFSK READY handshake):
//
// CMD (initiator):
//   t=0:     Send probe tones (2.25s)
//   t=2.25:  TX complete, start capture (4.0s window)
//   t=2.75:  RSP starts tone TX (500ms half-duplex turnaround)
//   t=5.0:   RSP stops TX
//   t=6.25:  CMD capture complete, analyze, send RESULT via AFSK
//
// RSP (responder):
//   t=0:     Start capture (3.0s window, covers CMD's 2.25s tones)
//   t=2.25:  CMD stops TX
//   t=2.75:  RSP sends own probe tones (500ms after CMD stops — fixed gap)
//   t=5.0:   RSP stops TX, analyze CMD tones, send RESULT via AFSK
//
// Key: RSP sends tones at a FIXED offset after CMD tones start.
// Analysis happens after the tone exchange, not during.
// RESULT exchange via AFSK is not time-critical.

class ProbeController {
public:
    // Callbacks
    std::function<void(const float*, int)> on_send_audio;   // Queue probe audio for TX
    std::function<void(const uint8_t*, size_t)> on_send_msg; // Send probe protocol msg

    void start_initiator(int sample_rate, float capture_seconds = 3.0f);   // Send probe tones + listen
    void start_initiator_deferred(int sample_rate);                        // Wait for READY, then send tones
    void start_responder(int sample_rate, float capture_seconds = 3.0f);   // Listen for probe tones
    void start_standalone(int sample_rate, float capture_seconds = 3.0f);  // TX + self-listen (debug)

    // Called by modem when probe tone TX audio has finished draining.
    // CMD: transitions from SENT_TONES to LISTENING_PROBE (start capture).
    // RSP: transitions from SENDING_PROBE to WAITING_RESULT (start listening for RESULT).
    void on_tx_complete();

    // Feed captured RX audio during listening phase
    void feed_rx(const float* audio, int count);

    // Periodic tick (~50ms) to drive timeouts and deferred tone send
    void tick();

    // Handle incoming probe protocol messages (first byte = type)
    void on_message(const uint8_t* data, size_t len);

    // RSP-side RESULT re-announce control. After the responder finalizes
    // (RX-first bring-up), it keeps re-emitting its RESULT on a cadence that
    // spans the initiator's WAITING_RESULT window (the fire-once RESULT lands
    // ~9 s before that window opens — the timing race that leaves ~96% of
    // sessions on AFSK). The modem cancels the re-announce once the peer's
    // activation is positively confirmed (a native OFDM frame decoded).
    bool reannounce_active() const { return reannounce_active_; }
    void stop_result_reannounce() { reannounce_active_ = false; }

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
    // Capture window is full OR a complete probe arrived early: run the RSP
    // deferred-tone / CMD-analyze transition. Idempotent-safe: only acts while
    // in LISTENING_PROBE.
    void on_capture_complete();

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
    static constexpr int TIMEOUT_TICKS = 400;  // 20s at 50ms/tick

    // READY handshake: initiator waits for PROBE_MSG_READY before sending tones.
    // If no READY arrives within READY_TIMEOUT_TICKS, fall back to old timing.
    bool deferred_initiator_ = false;      // True if using start_initiator_deferred()
    int ready_timeout_ = 0;
    static constexpr int READY_TIMEOUT_TICKS = 400;  // 20s at 50ms/tick (AFSK round-trip 8-12s)

    // Capture-window CEILINGS (event-driven early-exit ends capture the instant
    // a complete probe has landed — see feed_rx()/on_capture_complete(); these
    // are the ROBUST FALLBACK for the weak/lost-chirp path, unchanged from the
    // fixed-timing design). CMD: 1.05s chirp + 0.5s turnaround + 1.05s RSP chirp
    // + margin. RSP: 1.05s CMD chirp + margin.
    static constexpr float CMD_CAPTURE_WINDOW_S = 3.0f;
    static constexpr float RSP_CAPTURE_WINDOW_S = 2.0f;
    static constexpr int RSP_TONE_DELAY_TICKS = 10;      // 500ms gap before RSP sends tones (50ms/tick)

    // Event-driven capture early-exit. Once a full probe (probe_samples) plus a
    // short settle tail has been captured AND a chirp with a >=20 dB peak/median
    // ratio is present, END the capture immediately instead of waiting out the
    // fixed window. Weak/absent chirp => the check never fires => the fixed
    // window above still bounds the capture (the low-SNR path is unchanged).
    static constexpr float EARLY_SETTLE_S      = 0.15f;  // trailing margin after the probe
    static constexpr float EARLY_CHECK_STRIDE_S = 0.20f; // min new audio between checks (bounds cost)
    int early_check_at_ = 0;   // capture_samples_ at which the next early check may run

    // RSP deferred tone send: after capture completes, wait for half-duplex
    // turnaround gap before sending own tones.
    int rsp_tone_delay_countdown_ = 0;

    // Completion tracking (both conditions needed to finalize)
    bool analysis_done_ = false;     // We've analyzed their probe tones
    bool got_peer_result_ = false;   // We've received their analysis of our probe

    // RSP RESULT re-announce (closes the RESULT timing race). The responder
    // finalizes immediately (RX-first: it can already decode CMD's OFDM from the
    // symmetric seed) but its single fire-once RESULT is sent ~9 s before CMD's
    // WAITING_RESULT window opens, so CMD misses it and times out -> stays AX.25.
    // After finalizing, re-emit the RESULT on a cadence spanning CMD's window;
    // stop on CMD's corrected RESULT (on_message) or a decoded OFDM frame
    // (modem calls stop_result_reannounce()). Bounded so it self-terminates.
    bool reannounce_active_ = false;
    int  reannounce_ticks_left_ = 0;   // total budget remaining (ticks)
    int  reannounce_countdown_ = 0;    // ticks until next re-send
    static constexpr int RESULT_REANNOUNCE_PERIOD_TICKS = 40;   // ~2.0s at 50ms/tick
    static constexpr int RESULT_REANNOUNCE_SPAN_TICKS   = 700;  // ~35s spans CMD's 23-38s window

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
