#ifndef IRIS_OFDM_SESSION_H
#define IRIS_OFDM_SESSION_H

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_frame.h"
#include "ofdm/ofdm_sync.h"
#include "engine/speed_level.h"
#include "fec/ldpc.h"
#include "probe/passband_probe.h"
#include "config/config.h"
#include <complex>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>

namespace iris {

// OFDM session state
enum class OfdmSessionState {
    IDLE,           // Not active
    NEGOTIATED,     // CAP_OFDM exchanged, waiting for probe
    PROBING,        // Probe in progress
    ACTIVE,         // OFDM PHY active for data
    FAILED          // Fell back to single-carrier
};

// Diagnostics for GUI/logging
struct OfdmSessionDiag {
    OfdmSessionState state = OfdmSessionState::IDLE;
    int speed_level = 0;           // Current O-level (0-3)
    float mean_snr_db = 0;
    float throughput_bps = 0;
    int n_data_carriers = 0;
    int n_frames_tx = 0;
    int n_frames_rx = 0;
    int n_frames_rx_ok = 0;
    int n_frames_rx_fail = 0;
    ToneMap current_tone_map;
};

class OfdmSession {
public:
    OfdmSession();
    ~OfdmSession();

    // Initialize from probe results and config
    void init(const NegotiatedPassband& passband, const IrisConfig& config);

    // Reset to IDLE
    void reset();

    // State transitions
    void on_cap_ofdm_negotiated();   // Both sides have CAP_OFDM
    void on_probe_complete(const ProbeResult& local_rx, const ProbeResult& remote_rx);

    // TX: build OFDM frame from payload bytes
    // Returns baseband IQ samples (complex float) ready for upconversion
    std::vector<std::complex<float>> build_tx_frame(const uint8_t* payload, size_t len);

    // RX: attempt to demodulate OFDM frame from baseband IQ
    // Returns demod result (check .success)
    OfdmDemodResult process_rx_frame(const std::complex<float>* iq, int n_samples);

    // Gearshift: update speed level from channel feedback
    void update_speed_level(float snr_db);

    // Get current tone map (for exchange with peer)
    std::vector<uint8_t> get_serialized_tone_map() const;

    // Set peer's tone map (received from peer)
    void set_peer_tone_map(const uint8_t* data, size_t len);

    // Update tone map from latest channel estimate
    void update_tone_map_from_channel(const std::vector<float>& snr_per_carrier);

    // Diagnostics
    OfdmSessionDiag get_diagnostics() const;

    // State query
    OfdmSessionState state() const { return state_; }
    bool is_active() const { return state_ == OfdmSessionState::ACTIVE; }

    // Current FEC rate (from speed level)
    LdpcRate current_fec_rate() const;

    // Current tone map
    const ToneMap& current_tone_map() const { return tx_tone_map_; }

private:
    mutable std::mutex mutex_;
    OfdmSessionState state_ = OfdmSessionState::IDLE;

    // Config
    OfdmConfig ofdm_config_;
    bool waterfill_enabled_ = true;
    bool nuc_enabled_ = true;

    // TX state
    std::unique_ptr<OfdmModulator> modulator_;
    ToneMap tx_tone_map_;
    int speed_level_ = 0;

    // RX state
    std::unique_ptr<OfdmDemodulator> demodulator_;
    ToneMap rx_tone_map_;       // Peer's tone map (for decoding)

    // Statistics
    int n_frames_tx_ = 0;
    int n_frames_rx_ = 0;
    int n_frames_rx_ok_ = 0;
    int n_frames_rx_fail_ = 0;
    float last_snr_db_ = 0;
};

} // namespace iris
#endif
