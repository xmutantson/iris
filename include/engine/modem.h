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
#include "native/channel_eq.h"
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

    // Waterfall spectrum range (may differ from operating band)
    float spectrum_low_hz = 0;
    float spectrum_high_hz = 4000.0f;
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

    // Standalone passband probe (debug — TX tones, listen, analyze)
    void start_probe();

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
    void set_txdelay_ms(int ms) { ax25_session_.set_txdelay_ms(ms); }

    // Periodic tick for ARQ timeouts (call from main loop ~50ms)
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
    void send_probe_start_ui();
    void handle_cal_frame(const uint8_t* info, size_t len);

    void ptt_on();
    void ptt_off();

    // Compute power spectrum for waterfall display
    void compute_spectrum(const float* audio, int count);

    IrisConfig config_;
    // Original band/PHY from init — restored on disconnect after probe changes
    float orig_band_low_hz_ = 0;
    float orig_band_high_hz_ = 0;
    PhyConfig orig_phy_config_;
    std::atomic<ModemState> state_{ModemState::IDLE};
    bool loopback_mode_ = false;  // Skip TX mute when using internal loopback

    // AX.25 modems
    AfskModulator afsk_mod_;
    AfskDemodulator afsk_demod_;
    GfskModulator gfsk_mod_;
    GfskDemodulator gfsk_demod_;
    G3ruhScrambler g3ruh_tx_scrambler_;
    G3ruhScrambler g3ruh_rx_scrambler_;
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

    // Channel equalization (built from probe tone power measurements)
    ChannelEqualizer rx_channel_eq_;   // RX: flatten incoming channel
    ChannelEqualizer tx_channel_eq_;   // TX: pre-equalize outgoing signal

    // Engine
    Gearshift gearshift_;
    AGC agc_;
    float snr_db_ = 0;
    float snr_preamble_db_ = 0;  // Preamble-only SNR (for peer feedback)

    // Peer SNR feedback: what the remote side measures from our TX signal.
    // Used to cap TX speed on asymmetric links where local RX SNR ≠ peer RX SNR.
    // -1 = no peer SNR report received yet.
    float peer_snr_db_ = -1.0f;

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

    // Simulated FM de-emphasis filter (--deemphasis, for testing)
    // Standard 75µs (US) or 50µs (EU) first-order LPF: H(s) = 1/(1+sτ)
    // Produces ~6 dB/octave rolloff above corner frequency (2122 Hz for 75µs)
    Biquad sim_deemph_;
    bool sim_deemph_enabled_ = false;

    // PTT
    std::unique_ptr<PttController> ptt_;
    std::atomic<bool> ptt_active_{false};

    // TX/RX muting for half-duplex
    std::atomic<bool> rx_muted_{false};
    int rx_mute_holdoff_ = 0;  // Samples to remain muted after TX ends
    std::atomic<int> native_selfhear_guard_{0};  // Samples: discard native RX frames (self-hear from pipeline latency)

    // DCD (Data Carrier Detect) — defer TX while channel is busy
    std::atomic<int> dcd_holdoff_{0};        // samples remaining (0 = expired)
    int dcd_diag_ticks_ = 0;                 // DCD diagnostic logging counter

    // CSMA guard: defer TX after last frame decode to avoid stepping on response
    std::atomic<int> csma_holdoff_{0};       // samples remaining (0 = expired)

    // Auto-DCD: detect inverted-squelch radios where static is louder than signal.
    // Calibrates baseline noise level and detects polarity automatically.
    bool dcd_auto_ = false;          // auto-DCD enabled
    float dcd_baseline_rms_ = 0;     // measured noise floor RMS
    int dcd_baseline_samples_ = 0;   // samples collected for baseline
    bool dcd_baseline_done_ = false; // baseline measurement complete
    bool dcd_inverted_ = false;      // true = inverted squelch (signal < noise)


    // p-persistent CSMA (AX.25 2.2 Section 6.4.2)
    // After DCD clears + holdoff + csma_holdoff, enter slotted access:
    // each slottime period, generate random 0-255; if < persist then TX, else wait.
    int csma_slot_timer_ = 0;  // samples remaining in current slot

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
    std::atomic<bool> native_mode_{false};      // RX can decode native frames
    std::atomic<bool> native_tx_ready_{false};  // TX may use native PHY
    std::atomic<bool> peer_is_iris_{false};     // Remote confirmed as Iris via probe
    std::atomic<bool> ofdm_kiss_{false};        // OFDM RX enabled (probe complete, native demod active)
    std::atomic<bool> ofdm_kiss_tx_{false};     // OFDM TX enabled
    std::atomic<bool> ofdm_kiss_confirmed_{false}; // Heard native frame from peer (bidirectional)
    std::atomic<bool> ofdm_kiss_probing_{false}; // Probe in progress (suppress data TX)

    // Adaptive batch airtime: grows on successful ACKs, shrinks on REJ/loss.
    // TCP-style AIMD: additive increase (+1s per RR), multiplicative decrease (halve on REJ).
    float batch_airtime_s_ = 3.0f;               // Current batch cap (seconds)
    static constexpr float BATCH_AIRTIME_MIN = 3.0f;
    static constexpr float BATCH_AIRTIME_MAX = 9.0f;

    // TX-without-ACK counter: tracks consecutive native TX frames with no RR/REJ.
    // If we TX 3+ frames without any peer acknowledgment, downshift — the peer
    // likely can't decode us (asymmetric link or interference).
    int tx_no_ack_count_ = 0;
    uint8_t last_peer_nr_ = 0xFF;  // last N(R) from peer RR — 0xFF = no RR yet

    // OFDM-KISS transport-layer compression
    uint16_t ofdm_kiss_peer_caps_ = 0;         // Negotiated caps (intersection of local & peer)
    Compressor ofdm_kiss_tx_compressor_;
    Compressor ofdm_kiss_rx_compressor_;

    // OFDM-KISS B2F proxy
    B2fHandler ofdm_kiss_b2f_;
    std::vector<uint8_t> b2f_proxy_plaintext_;  // Accumulated plaintext from B2F unroll
    bool b2f_proxy_active_ = false;             // TX intercepting B2F payload
    bool b2f_proxy_rx_active_ = false;          // RX reassembling B2F payload
    // B2F AFSK history: buffer I-frame info fields during AFSK phase so the
    // B2F handler can replay SID/FC/FS exchanges when it initializes at OFDM-KISS activation.
    std::vector<std::vector<uint8_t>> b2f_afsk_tx_history_;
    std::vector<std::vector<uint8_t>> b2f_afsk_rx_history_;
    uint8_t b2f_proxy_vr_ = 0;                  // V(R) for generating local RR ACKs
    uint8_t b2f_proxy_addr_[14] = {};            // Cached AX.25 address header (for ACK/I-frame construction)
    bool b2f_proxy_addr_valid_ = false;         // Address header cached
    int  ofdm_kiss_probe_cd_ = 0;   // Tick countdown before initiator starts probe
    bool ofdm_kiss_probe_done_ = false; // Probe completed, PHY reconfigured
    bool probe_manual_ = false;         // Manual probe (button) vs in-session
    int  disconnect_timeout_ticks_ = 0; // 30s timeout: stop TX if stuck in AWAITING_RELEASE
    bool probe_start_pending_ = false;  // Deferred PROBE:START (can't send from state callback — tx_mutex_)
    std::string probe_peer_call_;       // Peer callsign for standalone probe result addressing
    XidCapability local_cap_;

    // Probe-before-connect: AX.25 SABM deferred until probe completes
    std::string pending_connect_call_;   // Remote callsign waiting for probe to finish
    int probe_connect_timeout_ = 0;     // Ticks before giving up on probe and connecting AFSK

    std::string xid_peer_call_;                  // Peer callsign for probe exchange

    // Calibration
    std::atomic<CalState> cal_state_{CalState::IDLE};
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
    void generate_ephemeral_x25519();
    void start_mlkem_exchange();
    void handle_mlkem_frame(const uint8_t* data, size_t len);
    void rekey_hybrid();
    bool mlkem_kx_pending_ = false;   // waiting for ML-KEM exchange to complete
    std::vector<uint8_t> mlkem_held_frames_;  // strict mode: hold data until hybrid KX done
    float rx_rms_ = 0;
    std::atomic<float> rx_raw_rms_{0};  // Pre-AGC RMS for DCD
    float rx_peak_ = 0;  // Peak sample value (decays over time)
    int rx_diag_counter_ = 0;  // Periodic RX diagnostic counter

    // RX overlap buffer for native mode
    std::vector<float> rx_overlap_buf_;
    static constexpr size_t RX_OVERLAP_MAX = 48000 * 40;  // 20 seconds of IQ (floats=IQ×2)
    int pending_frame_start_ = -1;   // Cached frame start when waiting for more data
    size_t pending_need_floats_ = 0; // Min buffer size (floats) needed before retry
    int pending_frame_timeout_ = 0;  // Samples remaining before abandoning pending frame
    bool relisten_pending_ = false;  // Deferred return to LISTENING after failed hail

    // Waterfall spectrum
    mutable std::mutex diag_mutex_;
    std::vector<std::complex<float>> last_constellation_;
    std::vector<float> last_spectrum_;
    float spectrum_low_hz_ = 0;
    float spectrum_high_hz_ = 4000.0f;
    std::vector<float> spectrum_buf_;
    int spectrum_buf_pos_ = 0;

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
