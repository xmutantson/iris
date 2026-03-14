#ifndef IRIS_PASSBAND_PROBE_H
#define IRIS_PASSBAND_PROBE_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <functional>

namespace iris {

// Automatic passband discovery via multi-tone probe (Mode A only).
//
// Sends 64 tones from 300-4500 Hz simultaneously. The receiver FFTs the
// captured audio, detects which tones survived the radio's filters, and
// reports back the usable frequency range. Both sides probe, both sides
// report, then the intersection becomes the operating bandwidth.
//
// Mode A only: audio-coupled FM radios have passband within 300-4500 Hz.
// Mode B (9600 baud) has minimal filtering; Mode C is not yet implemented.

struct PassbandProbeConfig {
    static constexpr int N_TONES = 64;
    static constexpr float TONE_LOW_HZ = 300.0f;
    static constexpr float TONE_HIGH_HZ = 4500.0f;
    static constexpr float TONE_SPACING_HZ =
        (TONE_HIGH_HZ - TONE_LOW_HZ) / (N_TONES - 1);  // ~66.7 Hz
    static constexpr float PROBE_DURATION_S = 2.25f;     // 2.25s probe burst (3x for better SNR)
    static constexpr float DETECT_THRESHOLD_DB = 15.0f;  // Below peak tone (wider catches edge rolloff)
    static constexpr float EDGE_MARGIN_HZ = 25.0f;       // Safety margin (was 50 Hz)
};

// Result of analyzing a received probe
struct ProbeResult {
    float low_hz = 0;      // Lowest surviving tone frequency
    float high_hz = 0;     // Highest surviving tone frequency
    int tones_detected = 0; // Number of tones above threshold
    bool valid = false;
    uint16_t capabilities = 0;  // CAP_* flags (appended to wire format, 0 if old peer)

    // Per-tone power (dB, N_TONES entries) — for GUI visualization
    float tone_power_db[PassbandProbeConfig::N_TONES] = {};
    bool tone_detected[PassbandProbeConfig::N_TONES] = {};
};

// Negotiated passband from both directions
struct NegotiatedPassband {
    float low_hz = 0;
    float high_hz = 0;
    float center_hz = 0;
    float bandwidth_hz = 0;
    bool valid = false;

    // Keep individual results for GUI display
    ProbeResult my_tx_their_rx;   // What they heard from us
    ProbeResult their_tx_my_rx;   // What we heard from them
};

// Generate a multi-tone probe signal (all tones superimposed)
// Output: float samples at given sample rate
// Returns number of samples written
int probe_generate(float* out, int max_samples, int sample_rate,
                   float amplitude = 0.5f);

// Analyze received audio for probe tones
// Input: float samples captured during probe window
// Returns ProbeResult with detected frequency range
ProbeResult probe_analyze(const float* samples, int n_samples, int sample_rate);

// Compute the negotiated passband from two probe results
// a_to_b: what B heard from A (A's TX path through B's RX filter)
// b_to_a: what A heard from B (B's TX path through A's RX filter)
NegotiatedPassband probe_negotiate(const ProbeResult& a_to_b,
                                    const ProbeResult& b_to_a);

// Serialize/deserialize ProbeResult for AX.25 transport
// Wire format: magic(1) + low_hz(4) + high_hz(4) + n_tones(2) + bitmap(8) = 19 bytes
//            + caps(2) + tone_power_db(64) = 85 bytes total (v3)
// Old peers (v2): 21 bytes (no tone powers). Backward compatible.
std::vector<uint8_t> probe_result_encode(const ProbeResult& r);
bool probe_result_decode(const uint8_t* data, size_t len, ProbeResult& r);

// Get frequency of tone index i
inline float probe_tone_freq(int i) {
    return PassbandProbeConfig::TONE_LOW_HZ +
           i * PassbandProbeConfig::TONE_SPACING_HZ;
}

// -----------------------------------------------------------------------
// Probe protocol state machine
// -----------------------------------------------------------------------

enum class ProbeState {
    IDLE,
    WAITING_READY,      // Initiator: sent REQUEST, waiting for READY before sending tones
    SENDING_PROBE,      // We are transmitting probe tones
    WAITING_RESULT,     // Waiting for peer's analysis of our probe
    LISTENING_PROBE,    // Peer is transmitting probe tones, we're capturing
    SENDING_RESULT,     // We're sending our analysis back
    DONE                // Both sides have results
};

// Protocol message types (sent as AX.25 I-frame data, first byte = type)
constexpr uint8_t PROBE_MSG_REQUEST  = 0xB0;  // "I want to probe"
constexpr uint8_t PROBE_MSG_READY    = 0xB1;  // "Go ahead, I'm listening"
constexpr uint8_t PROBE_MSG_RESULT   = 0xB2;  // "Here's what I heard from you"
constexpr uint8_t PROBE_MSG_ACK      = 0xB3;  // "Got your result, now my turn"
constexpr uint8_t PROBE_MSG_COMPLETE = 0xB4;  // "Probe done, switching to native"

} // namespace iris

#endif // IRIS_PASSBAND_PROBE_H
