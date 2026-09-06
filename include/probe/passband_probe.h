#ifndef IRIS_PASSBAND_PROBE_H
#define IRIS_PASSBAND_PROBE_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <functional>

namespace iris {

// Automatic passband discovery via chirp probe (Mode A only).
//
// Sends a linear up-chirp (200-6600 Hz, 500ms) repeated twice with a 50ms
// gap. The receiver cross-correlates with a locally generated reference
// chirp to find timing, then extracts the channel frequency response H(f)
// via frequency-domain deconvolution. |H(f)| is sampled at 64 tone
// frequencies (300-6300 Hz) to populate ProbeResult identically to the old
// multi-tone probe. Processing gain: ~35 dB (BT = 3200).
//
// The 300-6300 Hz grid spans BOTH the narrow-FM mic/speaker lane (audio
// filter ~300-2900 Hz) and the wide 6 kHz direct-discriminator (9600 data
// port) lane, so discovery tracks the true filter edge from ~2 kHz up to
// ~6 kHz instead of hard-capping at 4.5 kHz.

struct PassbandProbeConfig {
    static constexpr int N_TONES = 64;
    static constexpr float TONE_LOW_HZ = 300.0f;
    // Grid tops at 6300 Hz to cover BOTH the narrow-FM mic/speaker lane (audio
    // filter ~300-2900 Hz) AND the wide 6 kHz direct-discriminator (9600 data
    // port) lane [dual-bandwidth roadmap]. A grid that stopped at 4500 Hz
    // hard-capped discovery there, UNDER-discovering any channel wider than
    // ~4.5 kHz (a 5-6 kHz data port read back as 4.15 kHz). 64 tones over
    // 300-6300 Hz -> ~95.2 Hz spacing (wire format unchanged: still 64 tones).
    static constexpr float TONE_HIGH_HZ = 6300.0f;
    static constexpr float TONE_SPACING_HZ =
        (TONE_HIGH_HZ - TONE_LOW_HZ) / (N_TONES - 1);  // ~95.2 Hz
    static constexpr float PROBE_DURATION_S = 1.05f;     // 2x 500ms chirp + 50ms gap
    static constexpr float DETECT_THRESHOLD_DB = 15.0f;  // Below peak tone (wider catches edge rolloff)
    static constexpr float NOISE_FLOOR_MARGIN_DB = 10.0f; // Above noise floor (catches FM de-emphasis rolloff)
    static constexpr float EDGE_MARGIN_HZ = 25.0f;       // Safety margin (was 50 Hz)

    // Chirp parameters. The sweep must span slightly beyond the top tone
    // (6300 Hz) so H(f) is well-conditioned across the whole grid; 6600 Hz
    // keeps ~300 Hz of headroom. Well below Nyquist (24 kHz).
    static constexpr float CHIRP_F0 = 200.0f;            // Start frequency (Hz)
    static constexpr float CHIRP_F1 = 6600.0f;           // End frequency (Hz)
    static constexpr float CHIRP_DURATION_S = 0.5f;      // Single chirp duration
    static constexpr float CHIRP_GAP_S = 0.05f;          // Gap between repetitions
    static constexpr int   CHIRP_REPS = 2;               // Number of repetitions
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

    // LOCAL-ONLY (not serialized): median detected in-band tone power above the
    // out-of-band noise floor, in dB — a genuine link-SNR estimate computed by
    // probe_analyze() for the tones WE received (their_tx_result_ = reverse path).
    // Gates the post-probe connect-diet (skip the auto-tune on a clean high-SNR
    // link) + the faster-climb seed.  0 on a peer-supplied result (my_tx_result_).
    float est_snr_db = 0.0f;

    // OFDM PHY parameters (v4 extension, 0 = old peer / use defaults)
    uint8_t ofdm_cp_samples = 0;           // CP length (actual value, e.g. 32 or 64)
    uint8_t ofdm_pilot_carrier_spacing = 0; // Comb pilot spacing (e.g. 4 or 6)
    uint8_t ofdm_pilot_symbol_spacing = 0;  // Block pilot spacing (e.g. 14 or 24)
    uint8_t ofdm_nfft_code = 0;            // 0=ABSENT (old peer/use local default), 1=256, 2=1024, 3=512
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

// Generate a chirp probe signal (linear up-chirp, repeated twice with gap)
// Output: float samples at given sample rate
// Returns number of samples written
int probe_generate(float* out, int max_samples, int sample_rate,
                   float amplitude = 0.5f);

// Analyze received audio for chirp probe
// Cross-correlates with reference chirp, extracts H(f), samples at 64 tone freqs
// Returns ProbeResult with detected frequency range
ProbeResult probe_analyze(const float* samples, int n_samples, int sample_rate);

// Locate the FIRST chirp onset (sample offset of the first repetition) in a
// captured buffer via reference cross-correlation. Cheap relative to a full
// probe_analyze (single correlation, no per-tone deconvolution). Returns the
// onset sample, or -1 if no chirp with a >=20 dB peak/median ratio is present.
// peak_ratio_db_out (optional) receives the measured peak/median ratio in dB.
// Used by the probe controller to END the capture window the instant a
// complete probe has arrived (event-driven) instead of waiting a fixed window.
int probe_find_chirp_onset(const float* samples, int n_samples, int sample_rate,
                           float* peak_ratio_db_out = nullptr);

// Compute the negotiated passband from two probe results
// a_to_b: what B heard from A (A's TX path through B's RX filter)
// b_to_a: what A heard from B (B's TX path through A's RX filter)
NegotiatedPassband probe_negotiate(const ProbeResult& a_to_b,
                                    const ProbeResult& b_to_a);

// Deterministic OFDM-grid band derivation (session-reliability fix).
//
// The OFDM carrier grid (used_carrier_bins) MUST be bit-identical on both ends,
// otherwise the ZC preamble — generated over the TX's bins and correlated over
// the RX's bins — lands on carriers the receiver never looks at, the FD-ZC
// correlation collapses ("FD-ZC too low -> rejected"), and 0 forward frames
// acquire.  The legacy probe_negotiate() computed the band as the min/max
// INTERSECTION of both directions, which makes the band depend on RSP->CMD.
// But RSP->CMD is echoed to the responder only fire-and-forget: when that echo
// is lost the responder falls back to a symmetric seed and computes a WIDER
// band than the initiator's true intersection -> split grid -> blackout.
//
// This function instead derives the band from a SINGLE authoritative
// measurement that BOTH ends reliably hold: the initiator's probe as measured
// by the responder (CMD->RSP).  The responder measures it locally; the
// initiator receives it via the responder's robustly RE-ANNOUNCED RESULT.  The
// weakly-echoed reverse direction is not consulted, so a lost echo can no
// longer desync the grid.  my_tx_their_rx / their_tx_my_rx are preserved for
// GUI/EQ metadata exactly as probe_negotiate() sets them.
//   is_initiator: CMD->RSP is my_tx_their_rx (echoed) on the initiator,
//                 their_tx_my_rx (local) on the responder.
NegotiatedPassband probe_negotiate_grid(const ProbeResult& my_tx_their_rx,
                                        const ProbeResult& their_tx_my_rx,
                                        bool is_initiator);

// GEOMETRY PIN (measurement instrument, default-inert).  Env
// IRIS_GRID_PIN_NARROW=1 forces a VALID negotiated band to the shipped
// voice-port narrow geometry (narrow_passband(): 300-3000 Hz -> nfft-1024
// bins 6..64 -> 57 used carriers) regardless of what the probe discovered.
// WHY: the flat sim bench over-discovers ~325-4275 Hz (83 carriers) because
// the probe's noise floor is a deconvolution artifact (see
// sim_channel_relay.py bandpass note, 2026-07-04) -- every bench rate quoted
// on that grid is inflated ~1.44x vs true narrow.  The pin lets an A/B run
// the SAME channel at the claim-grade 57-carrier grid.  Applied inside the
// grid negotiation choke point on BOTH ends; both ends must therefore share
// the env setting (the honest-baseline runner forwards it to both), or the
// grids split and the session blacks out -- harness knob, never a default.
// An INVALID negotiation is returned unchanged (the pin never fabricates a
// band a failed probe did not earn).
NegotiatedPassband apply_grid_pin(const NegotiatedPassband& neg);

// Serialize/deserialize ProbeResult for AX.25 transport
// Wire format: magic(1) + low_hz(4) + high_hz(4) + n_tones(2) + bitmap(8) = 19 bytes
//            + caps(2) + tone_power_db(64) = 85 bytes (v3)
//            + ofdm_cfg(4) = 89 bytes total (v4: cp, pilot_carrier, pilot_symbol, nfft)
// Old peers: v1=19 bytes, v2=21 bytes, v3=85 bytes. Backward compatible.
std::vector<uint8_t> probe_result_encode(const ProbeResult& r);
bool probe_result_decode(const uint8_t* data, size_t len, ProbeResult& r);

// Detect channel shape from probe tone powers.
// Returns 0.0 for flat path (data port), 300.0 for FM mic/speaker path.
float probe_detect_preemph_corner(const ProbeResult& probe);

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
    SENT_TONES,         // CMD: tones queued, waiting for on_tx_complete() to start capture
    WAITING_RESULT,     // Waiting for peer's analysis of our probe
    LISTENING_PROBE,    // Peer is transmitting probe tones, we're capturing
    SENDING_RESULT,     // RSP: captured CMD tones, waiting for turnaround gap before own tones
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
