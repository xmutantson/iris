#ifndef IRIS_MODEM_H
#define IRIS_MODEM_H

#include "config/config.h"
#include "engine/speed_level.h"
#include "engine/gearshift.h"
#include "engine/snr.h"
#include "native/phy.h"
#include "native/frame.h"
#include "native/xid.h"
#include "native/upconvert.h"
#include "arq/arq.h"
#include "compress/compress.h"
#include "crypto/crypto.h"
#include "b2f/b2f_handler.h"
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

// Calibration state machine
enum class CalState {
    IDLE,
    TX_TONE,        // Transmitting test tone
    WAIT_REPORT,    // Waiting for partner's level report
    RX_TONE,        // Receiving partner's test tone
    TX_REPORT,      // Sending level report to partner
    DONE,
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
    int retransmits;
    float rx_rms;
    bool ptt_active;
    CalState cal_state;
    float cal_measured_rms;
    ArqState arq_state;
    ArqRole arq_role;
    std::vector<std::complex<float>> constellation;  // Last received symbols
    std::vector<float> spectrum;   // Power spectrum for waterfall
};

class Modem {
public:
    Modem();
    ~Modem();

    // Initialize with config
    bool init(const IrisConfig& config);
    void shutdown();

    // Main processing (call from audio callback or main loop)
    void process_rx(const float* rx_audio, int frame_count);
    void process_tx(float* tx_audio, int frame_count);

    // Queue a frame for transmission (called by KISS server)
    void queue_tx_frame(const uint8_t* frame, size_t len);

    // Get diagnostics for GUI
    ModemDiag get_diagnostics() const;

    // Auto level calibration
    void start_calibration();
    bool is_calibrating() const { return state_ == ModemState::CALIBRATING; }

    // ARQ session control
    void arq_connect(const std::string& remote_callsign);
    void arq_disconnect();
    void arq_listen();
    ArqState arq_state() const { return arq_.state(); }

    // Periodic tick for ARQ timeouts (call from main loop ~100ms)
    void tick();

    // PTT control
    void set_ptt_controller(std::unique_ptr<PttController> ptt) { ptt_ = std::move(ptt); }

    // Accessors
    ModemState state() const { return state_; }
    const IrisConfig& config() const { return config_; }

    void set_rx_callback(std::function<void(const uint8_t*, size_t)> cb) {
        rx_callback_ = cb;
    }

    // Callback when ARQ state changes (for AGW notifications)
    void set_state_callback(std::function<void(ArqState, const std::string&)> cb) {
        state_callback_ = cb;
    }

    // ARQ accessors for AGW flow control
    const std::string& arq_remote_callsign() const { return arq_.remote_callsign(); }
    int arq_pending_frames() const { return arq_.pending_frames(); }

private:
    void process_rx_ax25(const float* audio, int count);
    void process_rx_native(const float* audio, int count);
    void process_calibration_rx(const float* audio, int count);
    void generate_cal_tone(float* audio, int count);

    void ptt_on();
    void ptt_off();

    // Compute power spectrum for waterfall display
    void compute_spectrum(const float* audio, int count);

    IrisConfig config_;
    std::atomic<ModemState> state_{ModemState::IDLE};

    // AX.25 modems
    AfskModulator afsk_mod_;
    AfskDemodulator afsk_demod_;
    GfskModulator gfsk_mod_;
    GfskDemodulator gfsk_demod_;
    HdlcDecoder hdlc_decoder_;
    NrziDecoder nrzi_decoder_;

    // Native PHY
    PhyConfig phy_config_;
    std::unique_ptr<NativeModulator> native_mod_;
    std::unique_ptr<NativeDemodulator> native_demod_;

    // Mode A upconversion
    Upconverter upconverter_;
    Downconverter downconverter_;
    bool use_upconvert_ = false;

    // Engine
    Gearshift gearshift_;
    AGC agc_;
    float snr_db_ = 0;

    // ARQ session
    ArqSession arq_;

    // Compression (used when ARQ peers negotiate CAP_COMPRESSION)
    Compressor tx_compressor_;
    Compressor rx_compressor_;

    // Encryption (used when ARQ peers negotiate CAP_ENCRYPTION)
    CipherSuite cipher_;
    uint64_t tx_batch_counter_ = 0;
    uint64_t rx_batch_counter_ = 0;
    uint32_t crypto_direction_ = DIR_CMD_TO_RSP;

    // B2F unroll/reroll (used when ARQ peers negotiate CAP_B2F_UNROLL)
    B2fHandler b2f_handler_;

    // PTT
    std::unique_ptr<PttController> ptt_;
    bool ptt_active_ = false;

    // TX/RX muting for half-duplex
    std::atomic<bool> rx_muted_{false};
    int rx_mute_holdoff_ = 0;  // Samples to remain muted after TX ends

    // TX queue
    std::mutex tx_mutex_;
    std::queue<std::vector<uint8_t>> tx_queue_;
    std::vector<float> tx_buffer_;
    size_t tx_pos_ = 0;

    // RX state
    bool native_mode_ = false;
    bool xid_sent_ = false;
    XidCapability local_cap_;

    // Calibration
    CalState cal_state_ = CalState::IDLE;
    float cal_measured_rms_ = 0;
    int cal_tone_samples_ = 0;
    float cal_tone_phase_ = 0;
    float cal_rms_accum_ = 0;
    int cal_rms_count_ = 0;

    // Stats
    std::atomic<int> frames_rx_{0};
    std::atomic<int> frames_tx_{0};
    std::atomic<int> crc_errors_{0};
    float rx_rms_ = 0;

    // RX overlap buffer for native mode
    std::vector<float> rx_overlap_buf_;
    static constexpr size_t RX_OVERLAP_MAX = 48000 * 2;

    // Waterfall spectrum
    mutable std::mutex diag_mutex_;
    std::vector<std::complex<float>> last_constellation_;
    std::vector<float> last_spectrum_;

    // Callback to deliver received frames
    std::function<void(const uint8_t*, size_t)> rx_callback_;

    // Callback for ARQ state changes (state, remote_callsign)
    std::function<void(ArqState, const std::string&)> state_callback_;
};

} // namespace iris

#endif
