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
#include "ax25/ax25_session.h"
#include "compress/compress.h"
#include "crypto/crypto.h"
#include "b2f/b2f_handler.h"
#include "ax25/hdlc.h"
#include "ax25/fx25.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
#include "kiss/kiss_server.h"
#include "radio/rigctl.h"
#include "probe/passband_probe.h"
#include "probe/probe_controller.h"
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
// Initiator: SEND_CMD → TX_TONE → WAIT_REPORT → DONE
// Responder (auto): RX_TONE → SEND_REPORT → IDLE
enum class CalState {
    IDLE,
    SEND_CMD,       // Queue CAL:START UI frame, wait for TX drain
    TX_TONE,        // Transmitting test tone
    WAIT_REPORT,    // Waiting for partner's level report
    RX_TONE,        // Measuring partner's test tone RMS
    SEND_REPORT,    // Queue CAL:RMS report, wait for TX drain
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
    float rx_peak = 0;           // Peak sample value (for OVL indicator)
    bool ptt_active;
    CalState cal_state;
    float cal_measured_rms;
    ArqState arq_state;
    ArqRole arq_role;
    Ax25SessionState ax25_state = Ax25SessionState::DISCONNECTED;
    std::vector<std::complex<float>> constellation;  // Last received symbols
    std::vector<float> spectrum;   // Power spectrum for waterfall

    // Passband probe results
    ProbeState probe_state = ProbeState::IDLE;
    ProbeResult probe_my_tx;       // What they heard from us (A→B)
    ProbeResult probe_their_tx;    // What we heard from them (B→A)
    NegotiatedPassband probe_negotiated;
    bool probe_has_results = false;

    // Extended diagnostics for GUI status bar
    bool native_mode = false;
    uint64_t bytes_rx = 0;         // Cumulative bytes received
    uint64_t bytes_tx = 0;         // Cumulative bytes transmitted
    int phy_bps = 0;               // Current PHY bitrate
    int app_bps = 0;               // Application-level bitrate
    float compression_ratio = 0;   // 0 = off, >1 = active
    int encryption_state = 0;      // 0=off, 1=kx, 2=encrypted, 3=psk_mismatch

    // DCD (carrier detect)
    bool dcd_busy = false;
    float rx_raw_rms = 0;          // Pre-AGC RMS for DCD tuning
    float dcd_tone_energy = 0;     // AFSK tone correlation energy for tone-based DCD

    // Negotiated band info (from probe or config)
    float band_low_hz = 300.0f;
    float band_high_hz = 3500.0f;
    int baud_rate = 2400;
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

    // ARQ session control (native Iris protocol)
    void arq_connect(const std::string& remote_callsign);
    void arq_disconnect();
    void arq_listen();
    ArqState arq_state() const { return arq_.state(); }

    // AX.25 connected mode (standard protocol, interop with any TNC)
    void ax25_connect(const std::string& remote_callsign);
    void ax25_disconnect();
    void send_connected_data(const uint8_t* data, size_t len);
    Ax25SessionState ax25_state() const { return ax25_session_.state(); }
    int ax25_pending_frames() const { return ax25_session_.pending_frames(); }
    const std::string& ax25_remote_callsign() const { return ax25_session_.remote_callsign(); }
    void set_kiss_passthrough(bool v) { ax25_session_.set_kiss_passthrough(v); }

    // Periodic tick for ARQ timeouts (call from main loop ~100ms)
    void tick();

    // PTT control
    void set_ptt_controller(std::unique_ptr<PttController> ptt) { ptt_ = std::move(ptt); }
    void set_tx_drain_hooks(std::function<void()> mark, std::function<bool()> done) {
        tx_drain_mark_ = mark; tx_drain_done_ = done;
    }

    // Accessors
    ModemState state() const { return state_; }
    const IrisConfig& config() const { return config_; }
    void set_loopback_mode(bool v) { loopback_mode_ = v; }
    void force_speed_level(int level) { gearshift_.lock_level(level); }

    void set_rx_callback(std::function<void(const uint8_t*, size_t)> cb) {
        rx_callback_ = cb;
    }

    // GUI event log callback (frame events, not debug noise)
    void set_gui_log(std::function<void(const std::string&)> cb) { gui_log_ = cb; }

    // Packet log callback (is_tx, protocol, description)
    void set_packet_log(std::function<void(bool, const std::string&, const std::string&)> cb) {
        packet_log_ = cb;
    }


    // Callback when ARQ state changes (for AGW notifications)
    void set_state_callback(std::function<void(ArqState, const std::string&)> cb) {
        state_callback_ = cb;
    }

    // Callback when AX.25 session state changes
    void set_ax25_state_callback(std::function<void(Ax25SessionState, const std::string&)> cb) {
        ax25_state_callback_ = cb;
    }

    // ARQ accessors for AGW flow control
    const std::string& arq_remote_callsign() const { return arq_.remote_callsign(); }
    int arq_pending_frames() const { return arq_.pending_frames(); }

private:
    void process_rx_ax25(const float* audio, int count);
    void process_rx_native(const float* audio, int count);
    void dispatch_rx_frame(const std::vector<uint8_t>& frame, bool from_fx25 = false, bool from_ofdm = false);
    void process_calibration_rx(const float* audio, int count);
    void generate_cal_tone(float* audio, int count);
    void send_cal_ui(const char* payload);
    void send_conn_header_ui(const std::string& dest_call);
    void send_switch_ui(const std::string& dest_call);
    void handle_cal_frame(const uint8_t* info, size_t len);

    void ptt_on();
    void ptt_off();

    // Compute power spectrum for waterfall display
    void compute_spectrum(const float* audio, int count);

    IrisConfig config_;
    std::atomic<ModemState> state_{ModemState::IDLE};
    bool loopback_mode_ = false;  // Skip TX mute when using internal loopback

    // AX.25 modems
    AfskModulator afsk_mod_;
    AfskDemodulator afsk_demod_;
    GfskModulator gfsk_mod_;
    GfskDemodulator gfsk_demod_;
    HdlcDecoder hdlc_decoder_;
    Fx25Decoder fx25_decoder_;    // FX.25 decoder (runs in parallel with HDLC)
    NrziDecoder nrzi_decoder_;
    std::vector<uint8_t> last_rx_frame_;  // Dedup: last dispatched frame content
    int dedup_cooldown_ = 0;              // Samples remaining for dedup window

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

    // ARQ session (native Iris protocol)
    ArqSession arq_;

    // AX.25 connected mode session (standard protocol)
    Ax25Session ax25_session_;

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

    // Passband probe controller
    ProbeController probe_;
    int probe_fallback_ticks_ = 0;  // Fallback if peer has no probe support

    // Simulated bandpass filter (--bandpass, for testing)
    struct Biquad {
        float b0=1, b1=0, b2=0, a1=0, a2=0;
        float z1=0, z2=0;
        float process(float x) {
            float y = b0*x + z1;
            z1 = b1*x - a1*y + z2;
            z2 = b2*x - a2*y;
            return y;
        }
    };
    Biquad sim_bp_hi_[4], sim_bp_lo_[4];  // 4-stage HP + 4-stage LP = 8th order BP
    bool sim_bp_enabled_ = false;

    // PTT
    std::unique_ptr<PttController> ptt_;
    bool ptt_active_ = false;

    // TX/RX muting for half-duplex
    std::atomic<bool> rx_muted_{false};
    int rx_mute_holdoff_ = 0;  // Samples to remain muted after TX ends
    int native_selfhear_guard_ = 0;  // Samples: discard native RX frames (self-hear from pipeline latency)

    // DCD (Data Carrier Detect) — defer TX while channel is busy
    static constexpr int DCD_HOLDOFF_TICKS = 5;  // ~50-100ms quiet after carrier drops
    int dcd_holdoff_ = 0;

    // CSMA guard: defer TX after last frame decode to avoid stepping on response
    int csma_holdoff_ = 0;  // process_tx ticks remaining

    // TX queue
    std::mutex tx_mutex_;
    std::queue<std::vector<uint8_t>> tx_queue_;
    std::queue<std::vector<uint8_t>> ax25_tx_queue_;  // forced AX.25 (XID replies)
    std::vector<float> probe_audio_pending_;          // probe tones waiting to TX
    std::vector<float> tx_buffer_;
    size_t tx_pos_ = 0;
    bool tx_draining_ = false;  // waiting for audio pipeline to flush before PTT release
    std::function<void()> tx_drain_mark_;     // snapshot current pipeline position
    std::function<bool()> tx_drain_done_;     // true when pipeline has flushed past mark

    // RX state
    bool native_mode_ = false;      // RX can decode native frames
    bool native_tx_ready_ = false;  // TX may use native PHY (delayed after XID)
    int  native_tx_holdoff_ = 0;    // tick() countdown before native TX allowed
    bool xid_sent_ = false;
    bool peer_is_iris_ = false;     // Remote confirmed as Iris via connection header
    bool ofdm_kiss_ = false;        // OFDM RX enabled (SWITCH complete, native demod active)
    bool ofdm_kiss_tx_ = false;     // OFDM TX enabled (initiator: SWITCH complete; responder: heard native)
    bool ofdm_kiss_probing_ = false; // Native probe in progress (suppress data TX)
    int  ofdm_kiss_probe_cd_ = 0;   // Tick countdown before initiator starts probe
    bool ofdm_kiss_probe_done_ = false; // Probe completed, PHY reconfigured
    bool switch_sent_ = false;      // We sent a SWITCH UI frame
    bool switch_received_ = false;  // We received a SWITCH UI frame from peer
    int  switch_fallback_ticks_ = 0; // Timeout: if no SWITCH reply, stay AX.25
    int  conn_hdr_retries_ = 0;              // Remaining connection header UI retransmits
    bool conn_hdr_yield_started_ = false;    // True after first yield (prevents infinite retry reset)
    std::atomic<int> conn_hdr_retry_cd_{0};  // Tick countdown; atomic — read by audio thread
    XidCapability local_cap_;

    // Connection header handshake timing
    int xid_delay_ticks_ = 0;                   // Delay before sending header after CONNECTED
    std::string xid_peer_call_;                  // Peer callsign for delayed header
    int xid_fallback_ticks_ = 0;                // Fallback timer if remote not Iris

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
    std::atomic<uint64_t> bytes_rx_{0};
    std::atomic<uint64_t> bytes_tx_{0};
    int crypto_state_ = 0;  // 0=off, 1=kx, 2=encrypted, 3=psk_mismatch
    float rx_rms_ = 0;
    float rx_raw_rms_ = 0;  // Pre-AGC RMS for DCD
    float rx_peak_ = 0;  // Peak sample value (decays over time)
    int rx_diag_counter_ = 0;  // Periodic RX diagnostic counter

    // RX overlap buffer for native mode
    std::vector<float> rx_overlap_buf_;
    static constexpr size_t RX_OVERLAP_MAX = 48000 * 40;  // 20 seconds of IQ (floats=IQ×2)
    int pending_frame_start_ = -1;   // Cached frame start when waiting for more data
    size_t pending_need_floats_ = 0; // Min buffer size (floats) needed before retry
    bool relisten_pending_ = false;  // Deferred return to LISTENING after failed hail

    // Waterfall spectrum
    mutable std::mutex diag_mutex_;
    std::vector<std::complex<float>> last_constellation_;
    std::vector<float> last_spectrum_;

    // Callback to deliver received frames
    std::function<void(const uint8_t*, size_t)> rx_callback_;

    // Callback for ARQ state changes (state, remote_callsign)
    std::function<void(ArqState, const std::string&)> state_callback_;

    // Callback for AX.25 session state changes
    std::function<void(Ax25SessionState, const std::string&)> ax25_state_callback_;

    // GUI event log
    std::function<void(const std::string&)> gui_log_;

    // Packet log callback (is_tx, protocol, description)
    std::function<void(bool, const std::string&, const std::string&)> packet_log_;
};

} // namespace iris

#endif
