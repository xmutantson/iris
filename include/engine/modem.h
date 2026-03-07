#ifndef IRIS_MODEM_H
#define IRIS_MODEM_H

#include "config/config.h"
#include "engine/speed_level.h"
#include "engine/gearshift.h"
#include "engine/snr.h"
#include "native/phy.h"
#include "native/frame.h"
#include "native/xid.h"
#include "ax25/hdlc.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
#include "kiss/kiss_server.h"
#include "radio/rigctl.h"
#include <vector>
#include <mutex>
#include <queue>
#include <complex>
#include <memory>
#include <atomic>

namespace iris {

// Modem operating state
enum class ModemState {
    IDLE,           // Waiting
    RX_AX25,        // Receiving AX.25 (AFSK/GFSK)
    RX_NATIVE,      // Receiving Iris native
    TX_AX25,        // Transmitting AX.25
    TX_NATIVE,      // Transmitting Iris native
    CALIBRATING,    // Auto level calibration in progress
};

// Diagnostics snapshot for GUI
struct ModemDiag {
    ModemState state;
    int speed_level;
    float snr_db;
    float agc_gain;
    float tx_level;
    int kiss_clients;
    int frames_rx;
    int frames_tx;
    int crc_errors;
    float rx_rms;
    bool ptt_active;
    std::vector<std::complex<float>> constellation;  // Last received symbols
};

class Modem {
public:
    Modem();
    ~Modem();

    // Initialize with config
    bool init(const IrisConfig& config);
    void shutdown();

    // Main processing (call from audio callback or main loop)
    // rx_audio: mono float samples from capture
    // tx_audio: mono float samples to fill for playback
    void process_rx(const float* rx_audio, int frame_count);
    void process_tx(float* tx_audio, int frame_count);

    // Queue a frame for transmission (called by KISS server)
    void queue_tx_frame(const uint8_t* frame, size_t len);

    // Get diagnostics for GUI
    ModemDiag get_diagnostics() const;

    // Auto level calibration
    void start_calibration();
    bool is_calibrating() const { return state_ == ModemState::CALIBRATING; }

    // Accessors
    ModemState state() const { return state_; }
    const IrisConfig& config() const { return config_; }

private:
    void process_rx_ax25(const float* audio, int count);
    void process_rx_native(const float* audio, int count);

    IrisConfig config_;
    std::atomic<ModemState> state_{ModemState::IDLE};

    // AX.25 modems
    AfskModulator afsk_mod_;
    AfskDemodulator afsk_demod_;
    GfskModulator gfsk_mod_;
    GfskDemodulator gfsk_demod_;
    HdlcDecoder hdlc_decoder_;

    // Native PHY
    PhyConfig phy_config_;
    std::unique_ptr<NativeModulator> native_mod_;
    std::unique_ptr<NativeDemodulator> native_demod_;

    // Engine
    Gearshift gearshift_;
    AGC agc_;
    float snr_db_ = 0;

    // TX queue
    std::mutex tx_mutex_;
    std::queue<std::vector<uint8_t>> tx_queue_;
    std::vector<float> tx_buffer_;  // Samples being transmitted
    size_t tx_pos_ = 0;

    // RX state
    bool native_mode_ = false;
    XidCapability local_cap_;

    // Stats
    std::atomic<int> frames_rx_{0};
    std::atomic<int> frames_tx_{0};
    std::atomic<int> crc_errors_{0};
    float rx_rms_ = 0;

    // Last constellation for GUI
    mutable std::mutex diag_mutex_;
    std::vector<std::complex<float>> last_constellation_;

    // Callback to deliver received frames
    std::function<void(const uint8_t*, size_t)> rx_callback_;

public:
    void set_rx_callback(std::function<void(const uint8_t*, size_t)> cb) {
        rx_callback_ = cb;
    }
};

} // namespace iris

#endif
