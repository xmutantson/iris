#include "engine/modem.h"
#include "ax25/ax25_protocol.h"
#include "native/frame.h"
#include "fec/ldpc.h"
#include "common/logging.h"
#include <cstring>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Describe an AX.25 frame for GUI logging
static std::string describe_ax25(const uint8_t* data, size_t len) {
    Ax25Frame f;
    if (!ax25_parse(data, len, f))
        return "AX.25 (" + std::to_string(len) + " bytes, unparseable)";

    std::string desc = f.src.to_string() + ">" + f.dst.to_string() + " ";
    switch (f.type()) {
    case Ax25FrameType::I_FRAME:
        desc += "I N(S)=" + std::to_string(f.ns()) + " N(R)=" + std::to_string(f.nr())
              + " (" + std::to_string(f.info.size()) + " bytes)";
        break;
    case Ax25FrameType::S_FRAME:
        switch (f.s_type()) {
        case Ax25SType::RR:   desc += "RR";   break;
        case Ax25SType::RNR:  desc += "RNR";  break;
        case Ax25SType::REJ:  desc += "REJ";  break;
        default:              desc += "S?";    break;
        }
        desc += " N(R)=" + std::to_string(f.nr());
        break;
    case Ax25FrameType::U_FRAME:
        switch (f.u_type()) {
        case Ax25UType::SABM: desc += "SABM"; break;
        case Ax25UType::UA:   desc += "UA";   break;
        case Ax25UType::DISC: desc += "DISC"; break;
        case Ax25UType::DM:   desc += "DM";   break;
        case Ax25UType::UI:   desc += "UI (" + std::to_string(f.info.size()) + " bytes)"; break;
        case Ax25UType::XID:  desc += "XID";  break;
        default:              desc += "U?";    break;
        }
        break;
    }
    if (f.poll_final()) desc += " P/F";
    return desc;
}

static constexpr float CAL_TONE_FREQ = 1000.0f;
static constexpr int   CAL_TONE_DURATION = 48000;
static constexpr float CAL_TARGET_RMS = 0.3f;
static constexpr int RX_MUTE_HOLDOFF_SAMPLES = 9600;   // 200ms at 48kHz

Modem::Modem() = default;
Modem::~Modem() { shutdown(); }

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    // Validate configuration bounds
    if (config_.sample_rate < 8000 || config_.sample_rate > 192000) {
        IRIS_LOG("ERROR: sample_rate %d out of range [8000, 192000]", config_.sample_rate);
        return false;
    }
    if (config_.band_low_hz < 100.0f || config_.band_low_hz >= config_.band_high_hz) {
        IRIS_LOG("ERROR: invalid band range %.0f-%.0f Hz", config_.band_low_hz, config_.band_high_hz);
        return false;
    }
    if (config_.band_high_hz > config_.sample_rate / 2.0f) {
        IRIS_LOG("ERROR: band_high %.0f Hz exceeds Nyquist (%.0f Hz)",
                 config_.band_high_hz, config_.sample_rate / 2.0f);
        return false;
    }
    if (config_.tx_level < 0.0f || config_.tx_level > 1.0f) {
        IRIS_LOG("WARNING: tx_level %.2f clamped to [0, 1]", config_.tx_level);
        config_.tx_level = std::clamp(config_.tx_level, 0.0f, 1.0f);
    }

    if (config_.mode == "A" || config_.mode == "a") {
        float bandwidth = config_.band_high_hz - config_.band_low_hz;
        phy_config_ = mode_a_config(bandwidth);
        use_upconvert_ = true;

        // Compute center frequency from band config
        float center = config_.center_freq_hz;
        if (center <= 0.0f)
            center = (config_.band_low_hz + config_.band_high_hz) / 2.0f;

        IRIS_LOG("Band: %.0f-%.0f Hz, center %.0f Hz (baud %d, BW %.0f Hz)",
                 config_.band_low_hz, config_.band_high_hz, center,
                 phy_config_.baud_rate,
                 phy_config_.baud_rate * (1.0f + phy_config_.rrc_alpha));

        upconverter_ = Upconverter(center, config_.sample_rate);
        downconverter_ = Downconverter(center, config_.sample_rate);
    } else if (config_.mode == "B" || config_.mode == "b") {
        phy_config_ = mode_b_config();
        use_upconvert_ = false;
    } else {
        phy_config_ = mode_c_config();
        use_upconvert_ = false;
    }

    phy_config_.modulation = Modulation::BPSK;

    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);

    afsk_demod_.set_preemph_alpha(config_.preemph_alpha);

    local_cap_.version = XID_VERSION;
    local_cap_.capabilities = CAP_MODE_A | CAP_COMPRESSION | CAP_STREAMING;
    if (config_.mode == "B" || config_.mode == "b")
        local_cap_.capabilities |= CAP_MODE_B;
    if (config_.mode == "C" || config_.mode == "c")
        local_cap_.capabilities |= CAP_MODE_C;
    if (config_.encryption_mode > 0)
        local_cap_.capabilities |= CAP_ENCRYPTION;
    if (config_.b2f_unroll)
        local_cap_.capabilities |= CAP_B2F_UNROLL;
    local_cap_.max_modulation = config_.max_modulation;

    int max_mod_idx = (int)config_.max_modulation;
    int max_level = NUM_SPEED_LEVELS - 1;
    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        if ((int)SPEED_LEVELS[i].modulation > max_mod_idx) {
            max_level = (i > 0) ? i - 1 : 0;
            break;
        }
    }
    gearshift_.set_max_level(max_level);

    // Wire probe controller
    probe_.on_send_audio = [this](const float* audio, int count) {
        // Queue probe audio — appended to tx_buffer_ in process_tx
        // after any pending AFSK messages (RESULT before tones, same PTT cycle)
        std::lock_guard<std::mutex> lock(tx_mutex_);
        probe_audio_pending_.insert(probe_audio_pending_.end(), audio, audio + count);
    };
    probe_.on_send_msg = [this](const uint8_t* data, size_t len) {
        // Send probe messages via forced AX.25 queue (bypasses XID deferral)
        std::lock_guard<std::mutex> lock(tx_mutex_);
        ax25_tx_queue_.push(std::vector<uint8_t>(data, data + len));
    };

    // Initialize simulated bandpass filter if configured
    if (config_.sim_bandpass_low > 0 && config_.sim_bandpass_high > config_.sim_bandpass_low) {
        sim_bp_enabled_ = true;
        float fs = (float)config_.sample_rate;
        // 8th-order Butterworth = 4 cascaded biquad sections.
        // Q values for 8th-order Butterworth pole pairs:
        const float Q8[4] = {0.5098f, 0.6013f, 0.9000f, 2.5629f};

        // Highpass sections
        {
            float f0 = config_.sim_bandpass_low;
            float w0 = 2.0f * 3.14159265f * f0 / fs;
            float c = std::cos(w0), s = std::sin(w0);
            for (int i = 0; i < 4; i++) {
                float alpha = s / (2.0f * Q8[i]);
                float a0 = 1.0f + alpha;
                sim_bp_hi_[i].b0 = ((1.0f + c) / 2.0f) / a0;
                sim_bp_hi_[i].b1 = -(1.0f + c) / a0;
                sim_bp_hi_[i].b2 = ((1.0f + c) / 2.0f) / a0;
                sim_bp_hi_[i].a1 = (-2.0f * c) / a0;
                sim_bp_hi_[i].a2 = (1.0f - alpha) / a0;
            }
        }
        // Lowpass sections
        {
            float f0 = config_.sim_bandpass_high;
            float w0 = 2.0f * 3.14159265f * f0 / fs;
            float c = std::cos(w0), s = std::sin(w0);
            for (int i = 0; i < 4; i++) {
                float alpha = s / (2.0f * Q8[i]);
                float a0 = 1.0f + alpha;
                sim_bp_lo_[i].b0 = ((1.0f - c) / 2.0f) / a0;
                sim_bp_lo_[i].b1 = (1.0f - c) / a0;
                sim_bp_lo_[i].b2 = ((1.0f - c) / 2.0f) / a0;
                sim_bp_lo_[i].a1 = (-2.0f * c) / a0;
                sim_bp_lo_[i].a2 = (1.0f - alpha) / a0;
            }
        }
        IRIS_LOG("Simulated bandpass filter: %.0f-%.0f Hz (8th order Butterworth)",
                 config_.sim_bandpass_low, config_.sim_bandpass_high);
    }

    // Wire ARQ session
    arq_.set_callsign(config_.callsign);
    arq_.set_local_capabilities(local_cap_.capabilities);
    ArqCallbacks arq_cb;
    arq_cb.send_frame = [this](const uint8_t* data, size_t len) {
        // ARQ frames go directly to TX queue (not back through ARQ)
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(std::vector<uint8_t>(data, data + len));
    };
    arq_cb.on_data_received = [this](const uint8_t* data, size_t len) {
        if (!rx_callback_) return;
        const uint8_t* cur = data;
        size_t cur_len = len;

        // Layer 1: Decrypt if encryption active
        std::vector<uint8_t> decrypted;
        if (cipher_.is_active() && cur_len > 0) {
            decrypted.resize(cur_len);
            int dec = cipher_.decrypt(cur, (int)cur_len, decrypted.data(), (int)decrypted.size(),
                                       rx_batch_counter_++, crypto_direction_ ^ 1, AUTH_TAG_SIZE);
            if (dec > 0) {
                cur = decrypted.data();
                cur_len = dec;
            }
        }

        // Layer 2: Decompress if compression negotiated
        std::vector<uint8_t> decompressed;
        if (arq_.negotiated(CAP_COMPRESSION) && cur_len > 0) {
            decompressed.resize(cur_len * 4 + 4096);
            int dec_len = rx_compressor_.decompress_block(cur, (int)cur_len,
                                                          decompressed.data(), (int)decompressed.size());
            if (dec_len > 0) {
                cur = decompressed.data();
                cur_len = dec_len;
            }
        }

        // Layer 3: B2F reroll if negotiated
        std::vector<uint8_t> rerolled;
        if (arq_.negotiated(CAP_B2F_UNROLL) && b2f_handler_.is_initialized() && cur_len > 0) {
            rerolled.resize(cur_len * 2 + 4096);
            int rx_len = b2f_handler_.filter_rx((const char*)cur, (int)cur_len,
                                                 (char*)rerolled.data(), (int)rerolled.size());
            if (rx_len > 0) {
                cur = rerolled.data();
                cur_len = rx_len;
            }
        }

        bytes_rx_ += cur_len;
        rx_callback_(cur, cur_len);
    };
    arq_cb.on_state_changed = [this](ArqState state) {
        const char* sn[] = {"IDLE","LISTEN","HAIL","CONNECTING","CONNECTED","TURBO","DISCONNECTING"};
        IRIS_LOG("ARQ state -> %s", sn[(int)state]);
        if (state == ArqState::CONNECTED) {
            // Init compressors for new session
            tx_compressor_.init();
            rx_compressor_.init();
            if (arq_.negotiated(CAP_STREAMING)) {
                tx_compressor_.streaming_enable();
                rx_compressor_.streaming_enable();
            }

            // Init encryption if negotiated
            if (arq_.negotiated(CAP_ENCRYPTION) && config_.encryption_mode > 0) {
                // Parse PSK from hex
                std::vector<uint8_t> psk;
                for (size_t i = 0; i + 1 < config_.psk_hex.size(); i += 2) {
                    char byte_str[3] = {config_.psk_hex[i], config_.psk_hex[i+1], 0};
                    psk.push_back((uint8_t)strtol(byte_str, nullptr, 16));
                }
                // X25519 key exchange happens via CONNECT/CONNECT_ACK payloads
                // (handled by ARQ session). Here we just derive the session key.
                bool is_commander = (arq_.role() == ArqRole::COMMANDER);
                crypto_direction_ = is_commander ? DIR_CMD_TO_RSP : DIR_RSP_TO_CMD;
                cipher_.derive_session_key(config_.callsign.c_str(),
                                            arq_.remote_callsign().c_str(),
                                            psk.empty() ? nullptr : psk.data(),
                                            (int)psk.size(), false);
                cipher_.activate();
                tx_batch_counter_ = 0;
                rx_batch_counter_ = 0;
                crypto_state_ = 2;  // ENCRYPTED
            } else if (config_.encryption_mode > 0) {
                crypto_state_ = 1;  // KEY EXCHANGE (wanted encryption but peer didn't negotiate)
            }

            // Init B2F handler if negotiated
            if (arq_.negotiated(CAP_B2F_UNROLL)) {
                b2f_handler_.init();
                b2f_handler_.unroll_enabled = true;
            }

            // Native hail: both sides are proven Iris-capable, skip XID
            // and go straight to native mode.
            if (config_.native_hail && !native_mode_) {
                native_mode_ = true;
                native_tx_ready_ = true;
                xid_sent_ = true;  // Suppress XID handshake
                IRIS_LOG("Native hail: skipping XID, native mode active");
                // Cancel AX.25 session if it was still retrying SABM
                if (ax25_session_.state() == Ax25SessionState::AWAITING_CONNECTION) {
                    ax25_session_.reset();
                    IRIS_LOG("Native hail: cancelled AX.25 SABM (native connected)");
                }
            }
        } else if (state == ArqState::IDLE) {
            tx_compressor_.deinit();
            rx_compressor_.deinit();
            cipher_.wipe();
            crypto_state_ = 0;  // OFF
            b2f_handler_.deinit();

            // Reset native mode so next session starts clean.
            // Guard: if relisten_pending_ is already set, this IDLE came from
            // listen()→reset()→set_state(IDLE) — don't re-trigger or clear buffer.
            if (config_.native_hail && !relisten_pending_) {
                native_mode_ = false;
                native_tx_ready_ = false;
                xid_sent_ = false;
                rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                // Defer return to LISTENING (can't call listen() from
                // inside the state callback — would cause recursion).
                relisten_pending_ = true;
            }
        }
        if (state_callback_)
            state_callback_(state, arq_.remote_callsign());
    };
    arq_.set_callbacks(arq_cb);

    // Wire AX.25 connected mode session
    ax25_session_.set_local_callsign(config_.callsign);
    ax25_session_.set_send_callback([this](const uint8_t* data, size_t len) {
        // Route session frames: native (OFDM) if SWITCH complete, else AFSK.
        // Responder keeps ofdm_kiss_tx_=false until hearing first native frame,
        // so its session frames (RR etc) still go via AFSK during transition.
        // This is safe: the window is short (~1s) and AX.25 retry handles drops.
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (ofdm_kiss_tx_) {
            tx_queue_.push(std::vector<uint8_t>(data, data + len));
        } else {
            ax25_tx_queue_.push(std::vector<uint8_t>(data, data + len));
        }
    });
    ax25_session_.set_data_callback([this](const uint8_t* data, size_t len) {
        // Check for Iris connection header in I-frame data
        XidCapability remote_cap;
        if (!config_.ax25_only && conn_header_decode(data, len, remote_cap)) {
            IRIS_LOG("Connection header from peer: v%u caps=%04X mod=%u",
                     remote_cap.version, remote_cap.capabilities, (unsigned)remote_cap.max_modulation);
            auto agreed = negotiate(local_cap_, remote_cap);
            if (agreed.capabilities != 0) {
                int mod_level = std::min((int)agreed.max_modulation,
                                        (int)config_.max_modulation);
                phy_config_.modulation = (Modulation)mod_level;

                if (!xid_sent_) {
                    // Responder: reply with UI frame (no sequence numbers)
                    std::string dest = ax25_session_.remote_callsign().empty()
                        ? xid_peer_call_ : ax25_session_.remote_callsign();
                    send_conn_header_ui(dest);
                    xid_sent_ = true;
                    IRIS_LOG("Connection header reply sent (responder, UI) to %s", dest.c_str());
                } else {
                    conn_hdr_retries_ = 0;
                    conn_hdr_retry_cd_ = 0;  // Stop yielding
                    IRIS_LOG("Connection header I-frame handshake complete (initiator)");
                }
                peer_is_iris_ = true;
                // Don't activate ofdm_kiss_ — wait for SWITCH exchange (Phase 2)
                if (ax25_session_.we_initiated() && !switch_sent_) {
                    std::string dest = ax25_session_.remote_callsign().empty()
                        ? xid_peer_call_ : ax25_session_.remote_callsign();
                    send_switch_ui(dest);
                    switch_sent_ = true;
                    switch_fallback_ticks_ = 100;  // 10s total, retry at 5s
                    IRIS_LOG("SWITCH sent (initiator, I-frame path)");
                }
            }
            return;  // Don't forward header to KISS/AGW
        }

        // Normal I-frame data — deliver to KISS/AGW
        if (rx_callback_) rx_callback_(data, len);
    });
    ax25_session_.set_state_callback([this](Ax25SessionState state, const std::string& remote) {
        IRIS_LOG("AX25 state -> %d (remote=%s)", (int)state, remote.c_str());

        // Connection header: only the INITIATOR (sent SABM) sends the header.
        // The responder only replies after receiving the initiator's header.
        // This prevents synchronized collisions on half-duplex.
        // Queue immediately on CONNECTED — before AX.25 data starts flowing.
        // Use a 1-tick delay so the header drains from ax25_tx_queue_ before
        // the yield gate blocks all TX.
        if (state == Ax25SessionState::CONNECTED &&
            ax25_session_.we_initiated() &&
            !config_.ax25_only && !native_mode_ && !xid_sent_) {
            xid_peer_call_ = remote;
            send_conn_header_ui(remote);
            xid_sent_ = true;
            xid_delay_ticks_ = 15;         // ~1.5s: header TX (880ms) + PTT release + settle
            xid_fallback_ticks_ = 100;      // 10s total before giving up
            IRIS_LOG("Connection header queued for %s (initiator)",
                     remote.c_str());
        }

        // Reset XID state on disconnect so next connection can re-negotiate
        if (state == Ax25SessionState::DISCONNECTED) {
            xid_sent_ = false;
            xid_fallback_ticks_ = 0;
            xid_delay_ticks_ = 0;
            if (ofdm_kiss_ || ofdm_kiss_tx_) {
                IRIS_LOG("KISS-over-OFDM disabled (AX.25 disconnected)");
                ofdm_kiss_ = false;
                ofdm_kiss_tx_ = false;
            }
            peer_is_iris_ = false;
            switch_sent_ = false;
            switch_received_ = false;
            switch_fallback_ticks_ = 0;
            conn_hdr_retries_ = 0;
            conn_hdr_retry_cd_ = 0;
            conn_hdr_yield_started_ = false;
            native_mode_ = false;
            native_tx_ready_ = false;
            rx_overlap_buf_.clear();
            pending_frame_start_ = -1;
        }

        if (ax25_state_callback_)
            ax25_state_callback_(state, remote);
    });

    // Initialize FX.25 RS codecs (needed for both TX and RX)
    fx25_init();
    if (config_.fx25_mode > 0) {
        IRIS_LOG("FX.25 enabled: %d RS check bytes per frame", config_.fx25_mode);
    }

    state_ = ModemState::IDLE;
    return true;
}

void Modem::shutdown() {
    arq_.reset();
    ptt_off();
    state_ = ModemState::IDLE;
    native_mod_.reset();
    native_demod_.reset();
}

void Modem::ptt_on() {
    if (!ptt_active_ && ptt_) {
        ptt_->set_ptt(true);
        ptt_active_ = true;
        rx_muted_ = true;
    }
}

void Modem::ptt_off() {
    if (ptt_active_ && ptt_) {
        ptt_->set_ptt(false);
        ptt_active_ = false;
        rx_mute_holdoff_ = RX_MUTE_HOLDOFF_SAMPLES;
        rx_raw_rms_ = 0;   // Clear stale DCD reading from our own TX
        dcd_holdoff_ = 0;
    }
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
    // Deferred relisten: return to LISTENING after failed hail
    // (can't call from inside state callback due to recursion)
    if (relisten_pending_) {
        relisten_pending_ = false;
        arq_listen();
    }

    // Compute raw (pre-AGC) RMS for DCD — must be BEFORE early returns
    // so DCD always has fresh data. Skip during TX/mute: our own signal
    // would falsely trigger DCD.
    if (!ptt_active_ && !rx_muted_) {
        float raw_sq = 0;
        for (int i = 0; i < frame_count; i++)
            raw_sq += rx_audio[i] * rx_audio[i];
        rx_raw_rms_ = std::sqrt(raw_sq / std::max(frame_count, 1));
    }

    // In loopback mode, skip TX mute — the delay buffer handles timing
    if (!loopback_mode_) {
        if (state_ == ModemState::TX_AX25 || state_ == ModemState::TX_NATIVE)
            return;

        // Half-duplex holdoff after TX
        if (rx_muted_) {
            if (rx_mute_holdoff_ > 0) {
                rx_mute_holdoff_ -= frame_count;
                return;
            }
            rx_muted_ = false;
            rx_overlap_buf_.clear();
            pending_frame_start_ = -1;
        }
    }

    // Count down self-hear guard (continues even during mute)
    if (native_selfhear_guard_ > 0)
        native_selfhear_guard_ -= frame_count;

    std::vector<float> audio(rx_audio, rx_audio + frame_count);
    // AGC for AX.25 mode only — native mode has preamble-based gain estimation
    // AGC would distort QAM symbols within a frame (gain changes during preamble vs payload)
    if (!native_mode_)
        agc_.process_block(audio.data(), frame_count);

    float sum_sq = 0;
    float peak = 0;
    for (int i = 0; i < frame_count; i++) {
        sum_sq += audio[i] * audio[i];
        float a = std::fabs(audio[i]);
        if (a > peak) peak = a;
    }
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));
    // Peak with slow decay (~1s at 48kHz/1024 frame_count ≈ 47 frames/s)
    if (peak > rx_peak_) rx_peak_ = peak;
    else rx_peak_ *= 0.95f;

    // Periodic RX diagnostic (~every 5 seconds)
    rx_diag_counter_ += frame_count;
    if (rx_diag_counter_ >= config_.sample_rate * 5) {
        rx_diag_counter_ = 0;
        float best = detect_best_corr();
        IRIS_LOG("RX: rms=%.4f overlap=%zu corr=%.3f native=%d",
                 rx_rms_, rx_overlap_buf_.size(), best, native_mode_ ? 1 : 0);
    }

    compute_spectrum(audio.data(), frame_count);

    if (state_ == ModemState::CALIBRATING) {
        process_calibration_rx(audio.data(), frame_count);
        return;
    }

    if (native_mode_) {
        process_rx_native(audio.data(), frame_count);
    } else {
        process_rx_ax25(audio.data(), frame_count);

        // KISS-over-OFDM: run native demod only after SWITCH handshake completes.
        // Before SWITCH, both sides use AFSK exclusively. After SWITCH, native
        // demod is the primary decoder (AFSK stays active for protocol frames).
        if (ofdm_kiss_) {
            process_rx_native(audio.data(), frame_count);
        }
        // Native hail: also run native demod during LISTENING/HAILING
        // so we can detect native-mode HAIL frames from peers with native_hail enabled.
        else if (config_.native_hail && !config_.ax25_only) {
            auto arq_st = arq_.state();
            if (arq_st == ArqState::LISTENING || arq_st == ArqState::HAILING ||
                arq_st == ArqState::CONNECTING) {
                process_rx_native(audio.data(), frame_count);
            }
        }
    }
}

// Dispatch a decoded AX.25 frame (shared by HDLC, FX.25, and OFDM-KISS decoders)
void Modem::dispatch_rx_frame(const std::vector<uint8_t>& frame, bool from_fx25, bool from_ofdm) {
    // Dedup: HDLC and FX.25 decoders run in parallel on the same bit stream,
    // so every FX.25 frame also decodes as plain HDLC.  Without dedup the KISS
    // client sees every frame twice, causing AX.25 protocol errors (duplicate
    // UA → reset, duplicate I-frame → REJ).
    if (dedup_cooldown_ > 0 && frame == last_rx_frame_) {
        // Same frame within the dedup window — log it but don't dispatch
        if (packet_log_ && frame.size() >= 14) {
            std::string proto = from_fx25
                ? "FX.25 (" + std::to_string(fx25_decoder_.last_rs_errors()) + " RS corr)"
                : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
            packet_log_(false, proto + " [dup]", describe_ax25(frame.data(), frame.size()));
        }
        return;
    }
    last_rx_frame_ = frame;
    dedup_cooldown_ = config_.sample_rate * 3 / 2;  // 1.5s dedup window (FX.25 with heavy RS can arrive ~1s after HDLC)

    // When OFDM-KISS is active, suppress AFSK-decoded session frames.
    // Only OFDM-decoded frames should feed the AX.25 session and KISS client.
    // Allow UI frames through from AFSK for connection header exchange.
    if (ofdm_kiss_ && !from_ofdm) {
        // Check if it's a UI frame — those still need AFSK path for conn header
        bool is_ui = (frame.size() > 15 &&
                      (frame[14] & ~AX25_PF_MASK) == AX25_CTRL_UI);
        if (!is_ui) {
            // Drop non-UI AFSK frames — OFDM handles session traffic now
            return;
        }
    }

    // CSMA: brief holdoff after frame decode to avoid stepping on a burst.
    // Only set once per burst — don't stack on every frame in a batch.
    if (csma_holdoff_ <= 0)
        csma_holdoff_ = 25;  // ~250ms at 10ms tick rate

    frames_rx_++;

    // Log to packet viewer
    if (packet_log_ && frame.size() >= 14) {
        std::string proto;
        if (from_fx25) {
            int errs = fx25_decoder_.last_rs_errors();
            proto = "FX.25 (" + std::to_string(errs) + " RS corr)";
        } else {
            proto = config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200";
        }
        packet_log_(false, proto, describe_ax25(frame.data(), frame.size()));
    }

    // Try ARQ session first
    {
        ArqState arq_st = arq_.state();
        if (arq_st != ArqState::IDLE) {
            if (arq_.on_frame_received(frame.data(), frame.size()))
                return;
        }
    }

    // Try AX.25 connected mode session
    {
        bool native_active = (arq_.state() != ArqState::IDLE &&
                              arq_.state() != ArqState::LISTENING);
        if (!native_active || ax25_session_.is_active()) {
            Ax25Frame ax_frame;
            if (ax25_parse(frame.data(), frame.size(), ax_frame)) {
                if (ax25_session_.on_frame_received(ax_frame))
                    return;
            }
        }
    }

    // Legacy XID U-frame support (backwards compat with older Iris peers)
    bool is_xid = false;
    if (frame.size() >= 24 && frame[15] == IRIS_PID) {
        std::string src_call;
        for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
            char c = (char)(frame[ci] >> 1);
            if (c != ' ') src_call += c;
        }
        is_xid = true;
        bool is_self = (src_call == config_.callsign);
        XidCapability remote_cap;
        IRIS_LOG("Legacy XID frame from '%s' (self=%d)", src_call.c_str(), is_self ? 1 : 0);
        if (!is_self && xid_decode(&frame[16], frame.size() - 16, remote_cap)) {
            if (!config_.ax25_only) {
                auto agreed = negotiate(local_cap_, remote_cap);
                if (agreed.capabilities != 0) {
                    int mod_level = std::min((int)agreed.max_modulation,
                                            (int)config_.max_modulation);
                    phy_config_.modulation = (Modulation)mod_level;

                    if (!xid_sent_) {
                        // Reply with connection header I-frame (not XID)
                        auto hdr = conn_header_encode(local_cap_);
                        ax25_session_.send_data(hdr.data(), hdr.size());
                        xid_sent_ = true;
                        native_tx_holdoff_ = 10;
                        IRIS_LOG("Legacy XID received, replying with connection header");
                    } else {
                        xid_fallback_ticks_ = 0;
                        if (config_.mode == "A" || config_.mode == "a") {
                            probe_.start_initiator(config_.sample_rate);
                            IRIS_LOG("Legacy XID handshake complete (initiator), starting passband probe");
                        } else {
                            IRIS_LOG("Legacy XID handshake complete (initiator), mode %s — skipping probe", config_.mode.c_str());
                        }
                    }
                }
            }
        }
    }

    // Detect CAL: UI frames (ctrl=0x03, PID at offset 15, info at 16+)
    bool is_cal = false;
    bool is_conn_hdr = false;
    if (!is_xid && frame.size() > 20) {
        uint8_t ctrl = frame[14] & ~AX25_PF_MASK;
        if (ctrl == AX25_CTRL_UI && frame.size() > 16) {
            const uint8_t* info = frame.data() + 16;
            size_t info_len = frame.size() - 16;
            if (info_len >= 4 && info[0] == 'C' && info[1] == 'A' &&
                info[2] == 'L' && info[3] == ':') {
                handle_cal_frame(info, info_len);
                is_cal = true;
            }
            // SWITCH UI frame: Phase 2 of OFDM-KISS upgrade handshake.
            // Initiator sends SWITCH after IRIS header exchange; responder replies.
            // Nobody switches to native unless both SWITCH messages are exchanged.
            bool is_switch = false;
            // Debug: log all non-CAL UI frames to diagnose SWITCH detection
            if (!is_cal && info_len >= 1) {
                IRIS_LOG("UI frame: info[0..5]=%02X %02X %02X %02X %02X %02X len=%zu ax25_only=%d peer_is_iris=%d",
                         info_len >= 1 ? info[0] : 0, info_len >= 2 ? info[1] : 0,
                         info_len >= 3 ? info[2] : 0, info_len >= 4 ? info[3] : 0,
                         info_len >= 5 ? info[4] : 0, info_len >= 6 ? info[5] : 0,
                         info_len, config_.ax25_only ? 1 : 0, peer_is_iris_ ? 1 : 0);
            }
            if (!is_cal && info_len >= 6 && !config_.ax25_only &&
                info[0] == 'S' && info[1] == 'W' && info[2] == 'I' &&
                info[3] == 'T' && info[4] == 'C' && info[5] == 'H') {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                bool ui_is_self = (ui_src == config_.callsign);

                if (!ui_is_self && peer_is_iris_) {
                    switch_received_ = true;
                    IRIS_LOG("SWITCH received from %s (we_initiated=%d, switch_sent=%d)",
                             ui_src.c_str(), ax25_session_.we_initiated() ? 1 : 0,
                             switch_sent_ ? 1 : 0);

                    if (!ax25_session_.we_initiated()) {
                        // Responder: reply with SWITCH, then activate native RX.
                        // TX stays off until we hear initiator's first native frame.
                        if (!switch_sent_) {
                            send_switch_ui(ui_src);
                            switch_sent_ = true;
                        }
                        ofdm_kiss_ = true;
                        ax25_session_.set_t1_ticks(300);  // 15s at 50ms tick (native frames take 6s+)
                        IRIS_LOG("SWITCH complete (responder): native RX active, waiting for native TX from initiator");
                        if (gui_log_) gui_log_("OFDM-KISS: switched to native RX (responder)");
                    } else {
                        // Initiator: both SWITCH messages exchanged — activate native RX + TX.
                        // Initiator transmits first. Brief holdoff for responder's SWITCH reply to clear.
                        ofdm_kiss_ = true;
                        ofdm_kiss_tx_ = true;
                        switch_fallback_ticks_ = 0;
                        csma_holdoff_ = std::max(csma_holdoff_, 100);  // ~1s settle
                        // Native frames are much longer than AFSK — increase T1 to
                        // prevent premature TIMER_RECOVERY during 6s+ native TX.
                        ax25_session_.set_t1_ticks(200);  // 10s (native frame + holdoff + RR round-trip)
                        IRIS_LOG("SWITCH complete (initiator): native RX+TX active");
                        if (gui_log_) gui_log_("OFDM-KISS: switched to native mode (initiator)");

                        // Proactive native ping: queue an RR so the responder hears
                        // a native frame immediately and activates its own native TX.
                        // Without this, responder waits for T3 idle timeout (~17s).
                        {
                            std::lock_guard<std::mutex> lock(tx_mutex_);
                            auto ping = ax25_build_u(
                                ax25_make_addr(ui_src),
                                ax25_make_addr(config_.callsign),
                                AX25_CTRL_UI, false, true);
                            ping.push_back(AX25_PID_NONE);
                            tx_queue_.push(std::move(ping));
                            IRIS_LOG("Native ping queued to %s (responder activation)", ui_src.c_str());
                        }
                    }
                    is_switch = true;
                }
            }

            // Connection header in UI frame: "IRIS/..." (Phase 1 of upgrade)
            if (!is_cal && !is_switch && info_len >= 5 && !config_.ax25_only) {
                // Extract source callsign to filter self-heard frames
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                bool ui_is_self = (ui_src == config_.callsign);

                XidCapability remote_cap;
                if (!ui_is_self && conn_header_decode(info, info_len, remote_cap)) {
                    IRIS_LOG("Connection header UI from %s: v%u caps=%04X mod=%u",
                             ui_src.c_str(), remote_cap.version, remote_cap.capabilities,
                             (unsigned)remote_cap.max_modulation);
                    auto agreed = negotiate(local_cap_, remote_cap);
                    if (agreed.capabilities != 0) {
                        int mod_level = std::min((int)agreed.max_modulation,
                                                (int)config_.max_modulation);
                        phy_config_.modulation = (Modulation)mod_level;

                        if (!ax25_session_.we_initiated()) {
                            // Responder: reply with our header.
                            // Always reply (even if xid_sent_) in case our previous reply was lost.
                            xid_peer_call_ = ui_src;
                            send_conn_header_ui(ui_src);
                            xid_sent_ = true;
                            IRIS_LOG("Connection header UI reply sent to %s (responder%s)",
                                     ui_src.c_str(), peer_is_iris_ ? ", re-reply" : "");
                        } else {
                            // Initiator: IRIS handshake complete.
                            // Now send SWITCH to propose native mode upgrade.
                            xid_fallback_ticks_ = 0;
                            conn_hdr_retries_ = 0;
                            conn_hdr_retry_cd_ = 0;
                            // Brief holdoff to let responder's IRIS reply finish TX
                            csma_holdoff_ = std::max(csma_holdoff_, 50);  // ~500ms
                            if (!switch_sent_) {
                                send_switch_ui(ui_src);
                                switch_sent_ = true;
                                switch_fallback_ticks_ = 100;  // 10s total, retry at 5s for SWITCH reply
                                IRIS_LOG("SWITCH sent to %s (initiator, Phase 2)", ui_src.c_str());
                            }
                        }

                        // Mark peer as Iris (Phase 1 complete).
                        // Don't activate ofdm_kiss_ — wait for SWITCH exchange (Phase 2).
                        peer_is_iris_ = true;
                        xid_peer_call_ = ui_src;
                        IRIS_LOG("Peer %s confirmed as Iris", ui_src.c_str());
                    }
                    is_conn_hdr = true;
                }
            }
        }
    }

    bool is_probe = false;
    if (!is_xid && !is_cal && frame.size() >= 2) {
        uint8_t first = frame[0];
        if (first == PROBE_MSG_RESULT) {
            probe_.on_message(frame.data(), frame.size());
            is_probe = true;
        }
    }

    if (!is_xid && !is_probe && !is_conn_hdr && frame.size() >= 14) {
        std::string rx_src;
        for (int ci = 7; ci < 13; ci++) {
            char c = (char)(frame[ci] >> 1);
            if (c != ' ') rx_src += c;
        }
        if (rx_src != config_.callsign) {
            if (gui_log_)
                gui_log_("[RX] " + describe_ax25(frame.data(), frame.size()));
            // When AX.25 session is active, don't deliver raw frames to rx_callback_.
            // The session's data callback already delivers I-frame user data.
            // Stray frames (UI pings, etc.) must not leak as user data to AGW.
            auto ax_st = ax25_session_.state();
            bool session_active = (ax_st == Ax25SessionState::CONNECTED ||
                                   ax_st == Ax25SessionState::TIMER_RECOVERY ||
                                   ax_st == Ax25SessionState::AWAITING_CONNECTION ||
                                   ax_st == Ax25SessionState::AWAITING_RELEASE);
            if (!session_active && rx_callback_)
                rx_callback_(frame.data(), frame.size());
        }
    }
}

void Modem::process_rx_ax25(const float* audio, int count) {
    state_ = ModemState::RX_AX25;

    // Tick dedup cooldown
    if (dedup_cooldown_ > 0) {
        dedup_cooldown_ -= count;
        if (dedup_cooldown_ < 0) dedup_cooldown_ = 0;
    }

    // Feed raw audio to probe controller if it's listening
    if (probe_.state() == ProbeState::LISTENING_PROBE) {
        probe_.feed_rx(audio, count);
    }

    std::vector<uint8_t> rx_nrzi;
    if (config_.ax25_baud == 9600)
        rx_nrzi = gfsk_demod_.demodulate(audio, count);
    else
        rx_nrzi = afsk_demod_.demodulate(audio, count);

    auto rx_bits = nrzi_decoder_.decode(rx_nrzi);

    for (uint8_t b : rx_bits) {
        // Feed to standard HDLC decoder (plain AX.25)
        if (hdlc_decoder_.push_bit(b)) {
            dispatch_rx_frame(hdlc_decoder_.frame());
            // Don't reset() here — push_bit already prepares for the next
            // frame (in_frame_=true). reset() would kill in_frame_, causing
            // the next back-to-back frame to be lost (middle frame in batch).
        }

        // Feed to FX.25 decoder in parallel (always active for backwards compat)
        if (fx25_decoder_.push_bit(b)) {
            dispatch_rx_frame(fx25_decoder_.frame(), true);
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_rx_native(const float* audio, int count) {
    // Don't override state when running as secondary demod (OFDM-KISS or native hail)
    if (!ofdm_kiss_ && native_mode_)
        state_ = ModemState::RX_NATIVE;

    const float* iq_data;
    std::vector<float> iq_buf;
    size_t iq_count;

    if (use_upconvert_) {
        iq_buf = downconverter_.audio_to_iq(audio, count);
        iq_data = iq_buf.data();
        iq_count = iq_buf.size();
    } else {
        iq_data = audio;
        iq_count = count * 2;
    }

    rx_overlap_buf_.insert(rx_overlap_buf_.end(), iq_data, iq_data + iq_count);

    if (rx_overlap_buf_.size() > RX_OVERLAP_MAX) {
        size_t excess = rx_overlap_buf_.size() - RX_OVERLAP_MAX;
        rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                               rx_overlap_buf_.begin() + excess);
    }

    // If we have a pending frame waiting for more data, skip detection
    if (pending_frame_start_ >= 0) {
        if (rx_overlap_buf_.size() < pending_need_floats_) {
            return;  // Still not enough data, skip expensive work
        }
        IRIS_LOG("RX pending retry: buf=%zu floats, need=%zu, start=%d",
                 rx_overlap_buf_.size(), pending_need_floats_, pending_frame_start_);
    }

    int start;
    float det_corr;
    if (pending_frame_start_ >= 0) {
        // Re-use cached detection result
        start = pending_frame_start_;
        det_corr = 0.9f;  // Known good
        pending_frame_start_ = -1;
    } else {
        start = detect_frame_start(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                        phy_config_.samples_per_symbol);
        det_corr = detect_best_corr();
    }

    if (start >= 0) {
        // Trim pre-frame silence to maximize buffer space for payload.
        // Keep enough margin for RRC filter priming (RRC_SPAN * SPS samples).
        int rrc_margin = RRC_SPAN * phy_config_.samples_per_symbol + 10;
        if (start > rrc_margin + 50) {
            size_t trim = (size_t)(start - rrc_margin) * 2;
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + trim);
            start = rrc_margin;
        }

        std::vector<uint8_t> payload;
        if (decode_native_frame(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                 start, phy_config_, payload)) {
            // Self-hear guard: discard frames decoded from our own TX echo
            if (native_selfhear_guard_ > 0) {
                IRIS_LOG("RX native frame %zu bytes DISCARDED (self-hear guard, %d samples remaining)",
                         payload.size(), native_selfhear_guard_);
                // Skip past this frame so we don't re-decode it
                size_t skip = (size_t)(start + 100) * 2;
                if (skip < rx_overlap_buf_.size())
                    rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                           rx_overlap_buf_.begin() + skip);
                else
                    rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                return;
            }
            frames_rx_++;
            IRIS_LOG("RX native frame %zu bytes at offset %d", payload.size(), start);

            {
                auto preamble_ref = generate_preamble();
                int sps = phy_config_.samples_per_symbol;
                size_t n_iq = rx_overlap_buf_.size() / 2;
                int preamble_syms = (int)preamble_ref.size();
                std::vector<std::complex<float>> rx_preamble;
                for (int i = 0; i < preamble_syms; i++) {
                    size_t idx = start + i * sps;
                    if (idx < n_iq)
                        rx_preamble.push_back({rx_overlap_buf_[2 * idx],
                                                rx_overlap_buf_[2 * idx + 1]});
                }
                if ((int)rx_preamble.size() >= preamble_syms) {
                    float snr = estimate_snr(preamble_ref.data(), rx_preamble.data(),
                                              preamble_syms);
                    snr_db_ = snr;
                    gearshift_.update(snr);
                }
            }

            {
                std::lock_guard<std::mutex> lock(diag_mutex_);
                if (native_demod_)
                    last_constellation_ = native_demod_->symbols();
            }

            // Deliver payload(s) — split multi-payload frames
            auto deliver = [&](const uint8_t* data, size_t len) {
                if (ofdm_kiss_) {
                    // OFDM-KISS: payload is a raw AX.25 frame — dispatch through
                    // normal AX.25 path for session handling + KISS forwarding.
                    // Responder: first native frame received → enable native TX.
                    if (!ofdm_kiss_tx_) {
                        ofdm_kiss_tx_ = true;
                        IRIS_LOG("OFDM-KISS TX enabled (responder heard first native frame)");
                        if (gui_log_) gui_log_("OFDM-KISS: native TX active (responder)");
                    }
                    std::vector<uint8_t> ax25_frame(data, data + len);
                    if (packet_log_ && len >= 14)
                        packet_log_(false, "OFDM-KISS", describe_ax25(data, len));
                    dispatch_rx_frame(ax25_frame, false, true);  // from_ofdm=true
                    return;
                }
                auto arq_st = arq_.state();
                if (!loopback_mode_ &&
                    (arq_st == ArqState::CONNECTED ||
                     arq_st == ArqState::CONNECTING ||
                     arq_st == ArqState::LISTENING ||
                     arq_st == ArqState::HAILING ||
                     arq_st == ArqState::DISCONNECTING)) {
                    if (!arq_.on_frame_received(data, len)) {
                        // Not an ARQ frame — pass through to KISS/AGW
                        if (rx_callback_)
                            rx_callback_(data, len);
                    }
                } else {
                    if (rx_callback_)
                        rx_callback_(data, len);
                }
            };

            if (payload.size() >= 3 && payload[0] == MULTI_PAYLOAD_MAGIC) {
                // Multi-payload frame: [magic][2-byte LE len][data]...
                size_t pos = 1;
                int sub_count = 0;
                while (pos + 2 <= payload.size()) {
                    uint16_t sub_len = payload[pos] | ((uint16_t)payload[pos + 1] << 8);
                    pos += 2;
                    if (sub_len == 0 || pos + sub_len > payload.size())
                        break;
                    deliver(&payload[pos], sub_len);
                    pos += sub_len;
                    sub_count++;
                }
                IRIS_LOG("RX multi-payload: %d sub-frames", sub_count);
            } else {
                deliver(payload.data(), payload.size());
            }

            // Drain consumed IQ pairs from overlap buffer
            size_t consumed_iq = decode_consumed_iq();
            if (consumed_iq == 0)
                consumed_iq = (size_t)(start + phy_config_.samples_per_symbol * 128);
            size_t consumed = consumed_iq * 2;  // IQ pairs → interleaved floats
            if (consumed > rx_overlap_buf_.size())
                consumed = rx_overlap_buf_.size();
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + consumed);
        } else if (decode_was_overflow()) {
            // Use exact need from decoder (it knows where overflow happened)
            size_t need_iq = decode_consumed_iq();
            if (need_iq == 0) {
                // Fallback: need at least enough for header
                need_iq = (size_t)(start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN +
                    IRIS_HEADER_LEN + 64) * phy_config_.samples_per_symbol +
                    RRC_SPAN * phy_config_.samples_per_symbol);
            }
            pending_frame_start_ = start;
            pending_need_floats_ = need_iq * 2;
            if (pending_need_floats_ > RX_OVERLAP_MAX)
                pending_need_floats_ = RX_OVERLAP_MAX;
            IRIS_LOG("RX decode: need more data at offset %d (buf=%zu IQ, need ~%zu)",
                     start, rx_overlap_buf_.size() / 2, pending_need_floats_ / 2);
        } else {
            crc_errors_++;
            IRIS_LOG("RX decode FAIL at offset %d corr=%.3f (crc_errors=%d)",
                     start, det_corr, (int)crc_errors_);
            // Skip past the estimated frame to avoid re-detecting its remnants.
            // Use the consumed_iq from the decoder if available, else estimate.
            size_t consumed_iq = decode_consumed_iq();
            size_t skip;
            if (consumed_iq > 0) {
                skip = consumed_iq * 2;
            } else {
                // Estimate: skip past preamble + sync + header + minimum payload
                skip = (size_t)(start + (IRIS_PREAMBLE_LEN + IRIS_SYNC_LEN +
                    IRIS_HEADER_LEN + 64) * phy_config_.samples_per_symbol) * 2;
            }
            if (skip > rx_overlap_buf_.size()) skip = rx_overlap_buf_.size();
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + skip);
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_tx(float* tx_audio, int frame_count) {
    // Draining: TX buffer exhausted, waiting for audio pipeline to flush
    if (tx_draining_) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        bool done = tx_drain_done_ ? tx_drain_done_() : true;
        if (done) {
            tx_draining_ = false;
            // ptt_off sets holdoff based on TX mode (AFSK vs native)
            ptt_off();
            rx_muted_ = true;
            state_ = ModemState::IDLE;
        }
        return;
    }

    if (tx_pos_ < tx_buffer_.size()) {
        size_t remaining = tx_buffer_.size() - tx_pos_;
        size_t to_copy = std::min((size_t)frame_count, remaining);
        std::memcpy(tx_audio, tx_buffer_.data() + tx_pos_, to_copy * sizeof(float));
        tx_pos_ += to_copy;

        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        // Apply simulated bandpass filter to TX audio
        if (sim_bp_enabled_) {
            for (int i = 0; i < frame_count; i++) {
                float s = tx_audio[i];
                for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                tx_audio[i] = s;
            }
        }

        if (tx_pos_ >= tx_buffer_.size()) {
            // Self-hear guard: after native TX, ignore native RX for 800ms
            // to filter self-heard frames from FM radio audio loopback.
            // Pipeline latency is typically 50-200ms; 800ms covers worst case.
            // Does NOT block RX — just discards native decode results.
            if (state_ == ModemState::TX_NATIVE) {
                native_selfhear_guard_ = config_.sample_rate * 4 / 5;  // 800ms
                // Mandatory listen window: after native TX, defer next TX
                // long enough for the peer to fully transmit its response.
                // A 15-byte RR frame takes ~2.15s at BPSK r1/2. Wait 2.5s
                // to cover response + decode + queue + start.
                csma_holdoff_ = std::max(csma_holdoff_, 250);  // ~2.5s listen
            }
            tx_buffer_.clear();
            tx_pos_ = 0;
            // Mark current pipeline position, then wait for render to catch up
            if (tx_drain_mark_) tx_drain_mark_();
            tx_draining_ = true;
        }
        return;
    }

    if (state_ == ModemState::CALIBRATING && cal_state_ == CalState::TX_TONE) {
        generate_cal_tone(tx_audio, frame_count);
        return;
    }

    // DCD (Data Carrier Detect): defer TX while channel is busy.
    // On half-duplex FM, we must wait for the remote to finish transmitting.
    // Tone-based DCD: detect AFSK 1200/2200 Hz tones via correlator energy.
    // This works regardless of whether the radio's audio level goes up or down
    // with received signal (some radios have inverted squelch audio behavior).
    // Falls back to energy-based DCD in native OFDM mode.
    bool dcd_busy = false;
    if (!loopback_mode_ && config_.dcd_threshold > 0) {
        if (!native_mode_ && !ofdm_kiss_) {
            // AX.25 mode: use AFSK tone correlator energy
            // Threshold is auto-scaled: dcd_threshold 0.05 maps to tone_energy ~50
            // (correlator energy scales with signal amplitude² × window²)
            float tone_thr = config_.dcd_threshold * 1000.0f;
            dcd_busy = afsk_demod_.tone_energy() > tone_thr;
        } else {
            // Native/OFDM mode (including OFDM-KISS): energy-based DCD.
            // AFSK tone DCD would give false positives on native waveforms.
            dcd_busy = rx_raw_rms_ > config_.dcd_threshold;
        }
    }
    if (dcd_busy) {
        dcd_holdoff_ = DCD_HOLDOFF_TICKS;  // wait after carrier drops
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    if (dcd_holdoff_ > 0) {
        dcd_holdoff_--;
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }

    // CSMA guard: wait after last frame decode before starting TX
    if (csma_holdoff_ > 0) {
        csma_holdoff_--;
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }

    {
        std::lock_guard<std::mutex> lock(tx_mutex_);

        // Drain forced-AX.25 queue first (connection headers, probes),
        // then regular tx_queue_ (native or AX.25 depending on mode).
        // During connection header yield, suppress ALL TX so the responder
        // has a clear window to reply.
        bool have_frame = false;
        if (conn_hdr_retry_cd_ > 0) {
            // Yielding airtime — suppress everything so responder can reply
        } else if (!ax25_tx_queue_.empty()) {
            // Batch all forced-AX.25 frames into one TX burst (same as tx_queue_)
            state_ = ModemState::TX_AX25;
            int preamble_flags = std::max(8, config_.ptt_pre_delay_ms * config_.ax25_baud / 8000);
            std::vector<uint8_t> raw_bits;  // pre-NRZI

            bool first = true;
            while (!ax25_tx_queue_.empty()) {
                auto frame_data = std::move(ax25_tx_queue_.front());
                ax25_tx_queue_.pop();
                IRIS_LOG("TX frame %zu bytes (forced AX.25%s%s)", frame_data.size(),
                         config_.fx25_mode > 0 ? " FX.25" : "",
                         first ? "" : " batched");
                if (packet_log_ && frame_data.size() >= 14) {
                    std::string proto = config_.fx25_mode > 0
                        ? "FX.25-" + std::to_string(config_.fx25_mode)
                        : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
                    packet_log_(true, proto, describe_ax25(frame_data.data(), frame_data.size()));
                }
                int flags = first ? preamble_flags : 2;
                if (config_.fx25_mode > 0) {
                    if (!fx25_encode_raw(raw_bits, frame_data.data(), frame_data.size(),
                                        config_.fx25_mode, flags))
                        hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                } else {
                    hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                }
                first = false;
            }

            if (!raw_bits.empty()) {
                auto nrzi_bits = nrzi_encode(raw_bits);
                if (config_.ax25_baud == 9600)
                    tx_buffer_ = gfsk_mod_.modulate(nrzi_bits);
                else
                    tx_buffer_ = afsk_mod_.modulate(nrzi_bits);
            }
            have_frame = true;
        } else if (!tx_queue_.empty()) {
            // Native hail: use native PHY for ARQ hail/connect frames
            bool native_hail_active = false;
            if (config_.native_hail && !native_mode_) {
                auto arq_st = arq_.state();
                // Only active hail/connect phases — LISTENING is passive (responder
                // only replies via native after receiving a native hail frame).
                native_hail_active = (arq_st == ArqState::HAILING ||
                                       arq_st == ArqState::CONNECTING);
            }
            if ((native_mode_ && native_tx_ready_) || native_hail_active || ofdm_kiss_tx_) {
                // Batch multiple queued frames into one multi-payload frame.
                // Limit total payload based on speed level to cap air time at ~3s.
                int level = gearshift_.current_level();
                // PHY bps already accounts for modulation and FEC
                int phy_bps = net_throughput(level, phy_config_.baud_rate);
                // 3 seconds of payload at PHY rate, minus overhead
                size_t max_batch = std::min((size_t)NATIVE_MAX_PAYLOAD,
                                             (size_t)(phy_bps * 3 / 8));
                if (max_batch < 200) max_batch = 200;  // at least one frame

                std::vector<std::vector<uint8_t>> batch;
                size_t total_bytes = 0;
                while (!tx_queue_.empty()) {
                    auto& front = tx_queue_.front();
                    // 3 bytes overhead per sub-frame (2 len + data), plus magic byte
                    size_t overhead = batch.empty() ? 3 : 2;
                    // Always take the first frame (even if oversized) to avoid empty batches
                    if (!batch.empty() && total_bytes + overhead + front.size() > max_batch)
                        break;
                    total_bytes += overhead + front.size();
                    batch.push_back(std::move(front));
                    tx_queue_.pop();
                }

                // Log OFDM-KISS frames before payload build (batch elements get moved)
                if (ofdm_kiss_ && packet_log_) {
                    for (auto& sub : batch) {
                        if (sub.size() >= 14)
                            packet_log_(true, "OFDM-KISS", describe_ax25(sub.data(), sub.size()));
                    }
                }

                // Build payload: single frame or multi-payload
                std::vector<uint8_t> frame_data;
                if (batch.size() == 1 && batch[0].size() > 0 && batch[0][0] != MULTI_PAYLOAD_MAGIC) {
                    // Single frame, no wrapping needed (avoid overhead)
                    frame_data = std::move(batch[0]);
                } else {
                    // Multi-payload: [magic][2-byte LE len][data]...
                    frame_data.reserve(total_bytes);
                    frame_data.push_back(MULTI_PAYLOAD_MAGIC);
                    for (auto& sub : batch) {
                        uint16_t len = (uint16_t)sub.size();
                        frame_data.push_back(len & 0xFF);
                        frame_data.push_back((len >> 8) & 0xFF);
                        frame_data.insert(frame_data.end(), sub.begin(), sub.end());
                    }
                }

                IRIS_LOG("TX frame %zu bytes (%zu sub-frames, %s)",
                         frame_data.size(), batch.size(),
                         ofdm_kiss_ ? "OFDM-KISS" : "native");

                state_ = ModemState::TX_NATIVE;
                PhyConfig tx_config = phy_config_;

                Modulation tx_mod;
                int tx_fec_n, tx_fec_d;
                if (native_hail_active) {
                    // Native hail: BPSK + LDPC rate 1/2 for maximum sensitivity
                    tx_mod = Modulation::BPSK;
                    tx_fec_n = 1;
                    tx_fec_d = 2;
                    IRIS_LOG("TX native hail frame %zu bytes (BPSK r1/2)", frame_data.size());
                } else {
                    tx_mod = SPEED_LEVELS[level].modulation;
                    tx_fec_n = SPEED_LEVELS[level].fec_rate_num;
                    tx_fec_d = SPEED_LEVELS[level].fec_rate_den;
                }
                tx_config.modulation = tx_mod;
                LdpcRate fec = fec_to_ldpc_rate(tx_fec_n, tx_fec_d);
                auto iq = build_native_frame(frame_data.data(), frame_data.size(), tx_config, fec);

                if (use_upconvert_) {
                    tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                } else {
                    tx_buffer_.resize(iq.size() / 2);
                    for (size_t i = 0; i < iq.size() / 2; i++)
                        tx_buffer_[i] = iq[2 * i];
                }
            } else {
                // AX.25 mode: batch all queued frames into one TX burst
                // to minimize PTT on-time and reduce half-duplex collisions.
                // First frame gets full preamble, subsequent get 2 inter-frame flags.
                state_ = ModemState::TX_AX25;
                int preamble_flags = std::max(8, config_.ptt_pre_delay_ms * config_.ax25_baud / 8000);
                std::vector<uint8_t> raw_bits;  // pre-NRZI

                bool first = true;
                while (!tx_queue_.empty()) {
                    auto frame_data = std::move(tx_queue_.front());
                    tx_queue_.pop();
                    IRIS_LOG("TX frame %zu bytes (AX.25%s%s)", frame_data.size(),
                             config_.fx25_mode > 0 ? " FX.25" : "",
                             first ? "" : " batched");
                    if (packet_log_ && frame_data.size() >= 14) {
                        std::string proto = config_.fx25_mode > 0
                            ? "FX.25-" + std::to_string(config_.fx25_mode)
                            : (config_.ax25_baud == 9600 ? "AX.25-9600" : "AX.25-1200");
                        packet_log_(true, proto, describe_ax25(frame_data.data(), frame_data.size()));
                    }
                    int flags = first ? preamble_flags : 2;
                    if (config_.fx25_mode > 0) {
                        if (!fx25_encode_raw(raw_bits, frame_data.data(), frame_data.size(),
                                            config_.fx25_mode, flags))
                            hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                    } else {
                        hdlc_encode_raw(raw_bits, frame_data.data(), frame_data.size(), flags, 4);
                    }
                    first = false;
                }

                if (!raw_bits.empty()) {
                    auto nrzi_bits = nrzi_encode(raw_bits);
                    if (config_.ax25_baud == 9600)
                        tx_buffer_ = gfsk_mod_.modulate(nrzi_bits);
                    else
                        tx_buffer_ = afsk_mod_.modulate(nrzi_bits);
                }
            }
            have_frame = true;
        }

        // Append any pending probe audio to tx_buffer_ (same PTT cycle).
        // This ensures RESULT AFSK goes out before probe tones in Turn 2.
        // Don't scale here — tx_level is applied to entire buffer below.
        if (!probe_audio_pending_.empty()) {
            tx_buffer_.insert(tx_buffer_.end(),
                              probe_audio_pending_.begin(),
                              probe_audio_pending_.end());
            probe_audio_pending_.clear();
            if (!have_frame) have_frame = true;
            IRIS_LOG("TX probe audio appended (%zu total samples)", tx_buffer_.size());
        }

        if (have_frame) {
            int air_ms = (int)(tx_buffer_.size() * 1000 / config_.sample_rate);
            IRIS_LOG("TX buffer %zu samples (%d ms)", tx_buffer_.size(), air_ms);
            if (gui_log_)
                gui_log_("[TX] " + std::to_string(air_ms) + " ms on air");

            for (auto& s : tx_buffer_)
                s *= config_.tx_level;

            // Short silence for PTT hardware settle (50ms), then flag preamble provides
            // the rest of TXDelay for receiver DCD/clock sync
            int ptt_settle_ms = 50;
            int pre_samples = ptt_settle_ms * config_.sample_rate / 1000;
            if (pre_samples > 0)
                tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);

            int post_samples = config_.ptt_post_delay_ms * config_.sample_rate / 1000;
            if (post_samples > 0)
                tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

            tx_pos_ = 0;
            frames_tx_++;
            ptt_on();

            size_t to_copy = std::min((size_t)frame_count, tx_buffer_.size());
            std::memcpy(tx_audio, tx_buffer_.data(), to_copy * sizeof(float));
            tx_pos_ = to_copy;
            if (to_copy < (size_t)frame_count)
                std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

            // Apply simulated bandpass filter
            if (sim_bp_enabled_) {
                for (int i = 0; i < frame_count; i++) {
                    float s = tx_audio[i];
                    for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                    for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                    tx_audio[i] = s;
                }
            }
            return;
        }
    }

    std::memset(tx_audio, 0, frame_count * sizeof(float));
}

void Modem::queue_tx_frame(const uint8_t* frame, size_t len) {
    bytes_tx_ += len;
    if (native_mode_ && native_tx_ready_ && arq_.state() == ArqState::CONNECTED) {
        const uint8_t* cur = frame;
        size_t cur_len = len;

        // Layer 1: B2F unroll if negotiated (strip LZHUF on TX)
        std::vector<uint8_t> unrolled;
        if (arq_.negotiated(CAP_B2F_UNROLL) && b2f_handler_.is_initialized() && cur_len > 0) {
            unrolled.resize(cur_len * 2 + 4096);
            int tx_len = b2f_handler_.filter_tx((const char*)cur, (int)cur_len,
                                                 (char*)unrolled.data(), (int)unrolled.size());
            if (tx_len > 0) {
                cur = unrolled.data();
                cur_len = tx_len;
            }
        }

        // Layer 2: Compress if negotiated
        std::vector<uint8_t> compressed;
        if (arq_.negotiated(CAP_COMPRESSION) && cur_len > 0) {
            compressed.resize(cur_len + COMPRESS_HEADER_SIZE + 256);
            int comp_len = tx_compressor_.compress_block(cur, (int)cur_len,
                                                         compressed.data(), (int)compressed.size());
            if (comp_len > 0) {
                tx_compressor_.streaming_commit(cur, (int)cur_len);
                cur = compressed.data();
                cur_len = comp_len;
            }
        }

        // Layer 3: Encrypt if active
        std::vector<uint8_t> encrypted;
        if (cipher_.is_active() && cur_len > 0) {
            encrypted.resize(cur_len + AUTH_TAG_SIZE);
            int enc_len = cipher_.encrypt(cur, (int)cur_len, encrypted.data(), (int)encrypted.size(),
                                           tx_batch_counter_++, crypto_direction_, AUTH_TAG_SIZE);
            if (enc_len > 0) {
                cur = encrypted.data();
                cur_len = enc_len;
            } else {
                IRIS_LOG("[CRYPTO] CRITICAL: encryption failed (len=%zu), dropping frame to prevent data leak", cur_len);
                return;  // Do NOT send unencrypted data when encryption is active
            }
        }

        arq_.send_data(cur, cur_len);
        return;
    }
    std::lock_guard<std::mutex> lock(tx_mutex_);

    // Notify AX.25 session of outgoing frame so it can track KISS-initiated
    // connections (SABM/DISC) without generating duplicate frames.
    // Must be under tx_mutex_ to synchronize with connection header injection.
    ax25_session_.notify_outgoing(frame, len);

    std::vector<uint8_t> tx_frame(frame, frame + len);
    tx_queue_.push(std::move(tx_frame));
    if (gui_log_ && len >= 14)
        gui_log_("[TX] " + describe_ax25(frame, len));
}

void Modem::ax25_connect(const std::string& remote_callsign) {
    IRIS_LOG("AX25 connect to %s", remote_callsign.c_str());
    ax25_session_.connect(remote_callsign);
}

void Modem::ax25_disconnect() {
    IRIS_LOG("AX25 disconnect");
    ax25_session_.disconnect();
}

void Modem::send_connected_data(const uint8_t* data, size_t len) {
    // Route to AX.25 session if active, otherwise fall back to native ARQ path
    if (ax25_session_.state() == Ax25SessionState::CONNECTED ||
        ax25_session_.state() == Ax25SessionState::TIMER_RECOVERY) {
        ax25_session_.send_data(data, len);
    } else {
        queue_tx_frame(data, len);
    }
}

void Modem::arq_connect(const std::string& remote_callsign) {
    IRIS_LOG("ARQ connect to %s", remote_callsign.c_str());
    // Start in AX.25 — upgrade to native after XID negotiation
    arq_.connect(remote_callsign);
}

void Modem::arq_disconnect() {
    IRIS_LOG("ARQ disconnect");
    arq_.disconnect();
}

void Modem::arq_listen() {
    // Skip if already listening (prevents recursion from state callbacks)
    if (arq_.state() == ArqState::LISTENING)
        return;
    // When native_hail relisten is pending, let the deferred path handle it
    if (relisten_pending_)
        return;
    // Always start in AX.25 mode. Native upgrade happens via XID negotiation
    // when the remote station proves it supports Iris native PHY.
    // --ax25-only suppresses XID entirely.
    IRIS_LOG("ARQ listen (native_mode=0, ax25_only=%d)", config_.ax25_only ? 1 : 0);
    arq_.listen();
}

// After this many AFSK SABM failures, escalate to native BPSK hailing
static constexpr int NATIVE_HAIL_ESCALATION_RETRIES = 3;

void Modem::tick() {
    arq_.tick();
    ax25_session_.tick();
    probe_.tick();

    // Native hail escalation: after a few AFSK SABM failures, switch to
    // native BPSK hailing. Cancel AFSK retries so native hail gets exclusive
    // TX time — interleaved AFSK+native leaves no RX window for the 717ms
    // native response from the remote.
    if (config_.native_hail &&
        ax25_session_.state() == Ax25SessionState::AWAITING_CONNECTION &&
        ax25_session_.retry_count() >= NATIVE_HAIL_ESCALATION_RETRIES &&
        arq_.state() == ArqState::LISTENING) {
        std::string remote = ax25_session_.remote_callsign();
        int retries = ax25_session_.retry_count();
        IRIS_LOG("Native hail: AFSK SABM failed %d times, escalating to native BPSK",
                 retries);
        // Start native ARQ first so state is HAILING when AX.25 DISCONNECTED
        // callback fires (callback guards against HAILING/CONNECTING).
        arq_connect(remote);
        ax25_session_.reset();  // Stop AFSK retries
    }

    // Count down native mode holdoff (set after queuing XID reply)
    // During holdoff, stay in AX.25 mode to decode any remaining AX.25 frames.
    // When holdoff expires, start passband probe (responder side).
    if (native_tx_holdoff_ > 0) {
        native_tx_holdoff_--;
        if (native_tx_holdoff_ == 0) {
            if (config_.mode == "A" || config_.mode == "a") {
                // Don't go native yet — run passband probe first.
                // In the 3-turn protocol the initiator sends tones immediately
                // after XID completes. We (responder) start listening now.
                probe_.start_responder(config_.sample_rate);
                IRIS_LOG("Native holdoff expired, starting passband probe (responder)");
                // Fallback: if probe doesn't complete within 10s,
                // go native anyway (peer may be an older version without probe).
                probe_fallback_ticks_ = 100;  // 10s at 100ms/tick
            } else {
                IRIS_LOG("Native holdoff expired, mode %s — skipping probe, going native", config_.mode.c_str());
            }
        }
    }

    // Probe completion: activate native mode with negotiated band parameters
    if (probe_.is_done() && !native_mode_) {
        native_mode_ = true;
        native_tx_ready_ = true;
        // Apply negotiated passband and baud rate
        if (probe_.has_results() && probe_.negotiated().valid) {
            float low = probe_.negotiated().low_hz;
            float high = probe_.negotiated().high_hz;
            float bandwidth = high - low;
            config_.band_low_hz = low;
            config_.band_high_hz = high;
            float center = (low + high) / 2.0f;

            if (use_upconvert_) {
                upconverter_ = Upconverter(center, config_.sample_rate);
                downconverter_ = Downconverter(center, config_.sample_rate);

                // Auto-negotiate baud rate: pick highest that fits in passband.
                // Signal BW = baud * (1 + alpha). Any integer SPS works.
                // Both sides compute the same answer from the same negotiated band.
                // Sweep SPS from min (fastest) to max (slowest), pick first that fits.
                // Regulatory cap: FCC Part 97 limits 2m to 19.6 kilobaud / 20 kHz BW,
                // 70cm to 56 kilobaud / 100 kHz BW. We cap occupied BW at 20 kHz
                // to stay safe for VHF use (Mode B/C). Mode A (HF) is well below.
                constexpr float MAX_OCCUPIED_BW_HZ = 20000.0f;
                float usable_bw = std::min(bandwidth - 200.0f, MAX_OCCUPIED_BW_HZ);  // 100 Hz margin each edge
                constexpr int SPS_MIN = 6;    // 8000 baud max
                constexpr int SPS_MAX = 80;   // 600 baud min (fits in AFSK-width passband)
                int new_sps = 20;             // default: 2400 baud
                int new_baud = config_.sample_rate / new_sps;
                for (int sps = SPS_MIN; sps <= SPS_MAX; sps++) {
                    int baud = config_.sample_rate / sps;
                    float sig_bw = baud * (1.0f + phy_config_.rrc_alpha);
                    if (sig_bw <= usable_bw) {
                        new_sps = sps;
                        new_baud = baud;
                        break;
                    }
                }

                if (new_baud != phy_config_.baud_rate) {
                    phy_config_.baud_rate = new_baud;
                    phy_config_.samples_per_symbol = new_sps;
                    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
                    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
                    IRIS_LOG("Probe: upgraded baud rate to %d (SPS=%d, sig BW=%.0f Hz)",
                             new_baud, new_sps, new_baud * (1.0f + phy_config_.rrc_alpha));
                }
            }
            IRIS_LOG("Probe complete: band %.0f-%.0f Hz (%.0f Hz), center %.0f Hz, baud %d",
                     low, high, bandwidth, center, phy_config_.baud_rate);
        }
        IRIS_LOG("Native mode active (post-probe)");
    }

    // Probe fallback: if probe didn't complete (old peer or stuck), go native anyway
    if (probe_fallback_ticks_ > 0) {
        probe_fallback_ticks_--;
        if (probe_fallback_ticks_ == 0 && !native_mode_ && !probe_.is_done()) {
            native_mode_ = true;
            native_tx_ready_ = true;
            IRIS_LOG("Probe fallback: no probe received, native mode active");
        }
    }

    // Start yield after connection header has had time to drain from ax25_tx_queue_.
    // On first call (from CONNECTED), set retry count once. On subsequent calls
    // (from retry), retries are already decremented — just start the yield.
    // conn_hdr_yield_started_ prevents resetting retries after they're exhausted.
    if (xid_delay_ticks_ > 0) {
        xid_delay_ticks_--;
        if (xid_delay_ticks_ == 0 && xid_sent_ && !peer_is_iris_) {
            if (!conn_hdr_yield_started_) {
                conn_hdr_yield_started_ = true;
                conn_hdr_retries_ = 2;
            }
            if (conn_hdr_retries_ > 0 || !conn_hdr_yield_started_) {
                conn_hdr_retry_cd_ = 50;    // 5s yield: suppress ALL TX
                IRIS_LOG("Yield started: suppressing all TX for 5s (%d retries remaining)",
                         conn_hdr_retries_);
            }
        }
    }

    // Connection header yield + retry (initiator only)
    // Yield airtime by suppressing ALL TX, giving responder a clear window.
    // When yield expires, check for reply. If none, retry with another yield.
    if (conn_hdr_retry_cd_ > 0) {
        conn_hdr_retry_cd_--;
        if (conn_hdr_retry_cd_ == 0 && !peer_is_iris_ && conn_hdr_retries_ > 0 &&
            ax25_session_.is_active()) {
            // No reply yet — queue retry header, let it drain 1 tick, then yield again
            send_conn_header_ui(xid_peer_call_);
            conn_hdr_retries_--;
            xid_delay_ticks_ = 15;  // ~1.5s: header TX + PTT release + settle
            IRIS_LOG("Connection header retry to %s (%d remaining)",
                     xid_peer_call_.c_str(), conn_hdr_retries_);
        }
    }

    // SWITCH fallback: peer acknowledged IRIS header but didn't reply to SWITCH.
    // After 10s total (tick down from 100), give up and stay AX.25.
    // A retry is sent at the halfway point (50 ticks = 5s).
    if (switch_fallback_ticks_ > 0 && switch_sent_ && !switch_received_ && !ofdm_kiss_) {
        switch_fallback_ticks_--;
        if (switch_fallback_ticks_ == 50 && !xid_peer_call_.empty()) {
            send_switch_ui(xid_peer_call_);
            IRIS_LOG("SWITCH retry to %s (5s elapsed)", xid_peer_call_.c_str());
        }
        if (switch_fallback_ticks_ == 0) {
            IRIS_LOG("SWITCH fallback: no reply after 10s, staying AX.25");
        }
    }

    // Connection header fallback: peer didn't reply — stay AX.25
    if (xid_fallback_ticks_ > 0) {
        xid_fallback_ticks_--;
        if (xid_fallback_ticks_ == 0 && !native_mode_ && !ofdm_kiss_ && !peer_is_iris_) {
            IRIS_LOG("Connection header fallback: remote is not Iris, staying AX.25");
        }
    }
}

ModemDiag Modem::get_diagnostics() const {
    ModemDiag diag;
    diag.state = state_;
    diag.speed_level = gearshift_.current_level();
    diag.snr_db = gearshift_.smoothed_snr();
    diag.agc_gain = agc_.gain();
    diag.tx_level = config_.tx_level;
    diag.kiss_clients = 0;
    diag.frames_rx = frames_rx_;
    diag.frames_tx = frames_tx_;
    diag.crc_errors = crc_errors_;
    diag.retransmits = arq_.retransmit_count();
    diag.rx_rms = rx_rms_;
    diag.rx_peak = rx_peak_;
    diag.ptt_active = ptt_active_;
    diag.cal_state = cal_state_;
    diag.cal_measured_rms = cal_measured_rms_;
    diag.arq_state = arq_.state();
    diag.arq_role = arq_.role();
    diag.ax25_state = ax25_session_.state();
    {
        bool busy = false;
        if (config_.dcd_threshold > 0) {
            if (!native_mode_) {
                float tone_thr = config_.dcd_threshold * 1000.0f;
                busy = afsk_demod_.tone_energy() > tone_thr;
            } else {
                busy = rx_raw_rms_ > config_.dcd_threshold;
            }
            busy = busy || (dcd_holdoff_ > 0);
        }
        diag.dcd_busy = busy;
    }
    diag.rx_raw_rms = rx_raw_rms_;
    diag.dcd_tone_energy = afsk_demod_.tone_energy();

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        diag.constellation = last_constellation_;
        diag.spectrum = last_spectrum_;
    }

    // Probe results
    diag.probe_state = probe_.state();
    diag.probe_has_results = probe_.has_results();
    if (diag.probe_has_results) {
        diag.probe_my_tx = probe_.my_tx_result();
        diag.probe_their_tx = probe_.their_tx_result();
        diag.probe_negotiated = probe_.negotiated();
    }

    // Extended diagnostics
    diag.native_mode = native_mode_ || ofdm_kiss_tx_;
    diag.bytes_rx = bytes_rx_;
    diag.bytes_tx = bytes_tx_;
    diag.phy_bps = net_throughput(diag.speed_level, phy_config_.baud_rate);
    diag.app_bps = diag.phy_bps;  // Same as PHY unless compression active
    diag.compression_ratio = tx_compressor_.last_ratio();
    if (diag.compression_ratio > 1.0f)
        diag.app_bps = (int)(diag.phy_bps * diag.compression_ratio);
    diag.encryption_state = crypto_state_;
    diag.band_low_hz = config_.band_low_hz;
    diag.band_high_hz = config_.band_high_hz;
    diag.baud_rate = phy_config_.baud_rate;

    return diag;
}

void Modem::compute_spectrum(const float* audio, int count) {
    constexpr int NFFT = 512;
    if (count < NFFT) return;

    float bin_hz = (float)config_.sample_rate / NFFT;
    int k_lo = std::max(1, (int)(config_.band_low_hz / bin_hz));
    int k_hi = std::min(NFFT / 2, (int)(config_.band_high_hz / bin_hz) + 1);
    if (k_hi <= k_lo) return;

    std::vector<float> spec(k_hi - k_lo, 0.0f);
    const float* src = audio + count - NFFT;
    for (int k = k_lo; k < k_hi; k++) {
        float re = 0, im = 0;
        for (int n = 0; n < NFFT; n++) {
            float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * n / (NFFT - 1)));
            float angle = -2.0f * (float)M_PI * k * n / NFFT;
            re += src[n] * w * std::cos(angle);
            im += src[n] * w * std::sin(angle);
        }
        float pwr = (re * re + im * im) / (NFFT * NFFT);
        spec[k - k_lo] = 10.0f * std::log10(std::max(pwr, 1e-12f));
    }

    std::lock_guard<std::mutex> lock(diag_mutex_);
    last_spectrum_ = std::move(spec);
}

// --- Calibration ---
// Protocol:
//   Initiator: clicks Auto Cal → sends CAL:START UI frame → tone 1s → WAIT_REPORT
//   Responder: auto-detects CAL:START → measures tone RMS → sends CAL:RMS=X.XXXX
//   Initiator: receives report → adjusts TX level → DONE

// Send connection header as a UI frame (no sequence numbers, no offsetter needed)
void Modem::send_conn_header_ui(const std::string& dest_call) {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr(dest_call);
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    auto hdr = conn_header_encode(local_cap_);
    frame.insert(frame.end(), hdr.begin(), hdr.end());
    IRIS_LOG("Connection header UI to %s (%zu bytes)", dest_call.c_str(), hdr.size());
    std::lock_guard<std::mutex> lock(tx_mutex_);
    ax25_tx_queue_.push(std::move(frame));
}

// Send SWITCH proposal as UI frame (Phase 2 of OFDM-KISS upgrade)
void Modem::send_switch_ui(const std::string& dest_call) {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr(dest_call);
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    const char* payload = "SWITCH\r";
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + 7);
    IRIS_LOG("SWITCH UI sent to %s", dest_call.c_str());
    std::lock_guard<std::mutex> lock(tx_mutex_);
    ax25_tx_queue_.push(std::move(frame));
}

// Build and queue a UI frame with cal payload (e.g., "CAL:START" or "CAL:RMS=0.1234")
void Modem::send_cal_ui(const char* payload) {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("CAL");
    // UI frame: dst(7) + src(7) + ctrl(1) + PID(1) + info
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    size_t plen = strlen(payload);
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + plen);
    std::lock_guard<std::mutex> lock(tx_mutex_);
    ax25_tx_queue_.push(std::move(frame));
}

// Handle incoming CAL: frame (called from dispatch_rx_frame)
void Modem::handle_cal_frame(const uint8_t* info, size_t len) {
    std::string payload((const char*)info, len);

    if (payload.find("CAL:START") != std::string::npos) {
        // Remote is starting cal — enter RX measurement mode
        if (cal_state_ == CalState::IDLE) {
            IRIS_LOG("[CAL] Received CAL:START — entering measurement mode");
            if (gui_log_) gui_log_("[CAL] Measuring remote tone...");
            state_ = ModemState::CALIBRATING;
            cal_state_ = CalState::RX_TONE;
            cal_rms_accum_ = 0;
            cal_rms_count_ = 0;
            cal_measured_rms_ = 0;
        }
    } else if (payload.find("CAL:RMS=") != std::string::npos) {
        // Remote is reporting our tone's RMS at their end
        size_t pos = payload.find("CAL:RMS=");
        float remote_rms = std::stof(payload.substr(pos + 8));
        IRIS_LOG("[CAL] Remote measured RMS=%.4f", remote_rms);
        if (remote_rms > 0.001f && cal_state_ == CalState::WAIT_REPORT) {
            float correction = CAL_TARGET_RMS / remote_rms;
            config_.tx_level *= correction;
            config_.tx_level = std::clamp(config_.tx_level, 0.01f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            cal_measured_rms_ = remote_rms;
            IRIS_LOG("[CAL] Adjusted TX level to %.3f (correction %.2fx)",
                     config_.tx_level, correction);
            if (gui_log_) {
                char msg[128];
                snprintf(msg, sizeof(msg), "[CAL] Done! TX level = %.3f (remote RMS was %.4f)",
                         config_.tx_level, remote_rms);
                gui_log_(msg);
            }
            cal_state_ = CalState::DONE;
            state_ = ModemState::IDLE;
        }
    }
}

void Modem::start_calibration() {
    IRIS_LOG("[CAL] Starting calibration");
    if (gui_log_) gui_log_("[CAL] Sending tone to remote...");
    state_ = ModemState::CALIBRATING;
    cal_state_ = CalState::SEND_CMD;
    cal_tone_samples_ = 0;
    cal_tone_phase_ = 0;
    cal_rms_accum_ = 0;
    cal_rms_count_ = 0;
    cal_measured_rms_ = 0;
    // Queue the CAL:START command frame
    send_cal_ui("CAL:START");
}

void Modem::generate_cal_tone(float* audio, int count) {
    float phase_inc = 2.0f * (float)M_PI * CAL_TONE_FREQ / (float)config_.sample_rate;
    for (int i = 0; i < count; i++) {
        audio[i] = config_.tx_level * std::sin(cal_tone_phase_);
        cal_tone_phase_ += phase_inc;
        if (cal_tone_phase_ > 2.0f * (float)M_PI)
            cal_tone_phase_ -= 2.0f * (float)M_PI;
        cal_tone_samples_++;
    }
    if (cal_tone_samples_ >= CAL_TONE_DURATION) {
        ptt_off();
        cal_state_ = CalState::WAIT_REPORT;
        IRIS_LOG("[CAL] Tone sent, waiting for remote report");
    }
}

void Modem::process_calibration_rx(const float* audio, int count) {
    if (cal_state_ == CalState::SEND_CMD) {
        // Wait for the CAL:START frame to be transmitted
        if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
            // Command sent — start transmitting tone
            cal_state_ = CalState::TX_TONE;
            ptt_on();
            IRIS_LOG("[CAL] CAL:START sent, transmitting tone");
        }
    } else if (cal_state_ == CalState::RX_TONE) {
        // Measure incoming audio RMS (skip first 100ms for PTT settle)
        int skip_samples = config_.sample_rate / 10;
        for (int i = 0; i < count; i++) {
            cal_rms_count_++;
            if (cal_rms_count_ > skip_samples) {
                cal_rms_accum_ += audio[i] * audio[i];
            }
        }
        if (cal_rms_count_ >= CAL_TONE_DURATION + skip_samples) {
            int measure_count = cal_rms_count_ - skip_samples;
            cal_measured_rms_ = std::sqrt(cal_rms_accum_ / measure_count);
            IRIS_LOG("[CAL] Measured RMS = %.4f, sending report", cal_measured_rms_);
            if (gui_log_) {
                char msg[128];
                snprintf(msg, sizeof(msg), "[CAL] Remote RMS = %.4f, sending report",
                         cal_measured_rms_);
                gui_log_(msg);
            }
            // Send report back
            char report[48];
            snprintf(report, sizeof(report), "CAL:RMS=%.4f", cal_measured_rms_);
            send_cal_ui(report);
            cal_state_ = CalState::SEND_REPORT;
        }
    } else if (cal_state_ == CalState::SEND_REPORT) {
        // Wait for report TX to drain, then return to idle
        if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
            IRIS_LOG("[CAL] Report sent, calibration complete (responder)");
            if (gui_log_) gui_log_("[CAL] Report sent.");
            cal_state_ = CalState::IDLE;
            state_ = ModemState::IDLE;
        }
    } else if (cal_state_ == CalState::WAIT_REPORT) {
        // Demodulate AFSK looking for CAL:RMS= frame from remote
        std::vector<uint8_t> rx_nrzi;
        if (config_.ax25_baud == 9600)
            rx_nrzi = gfsk_demod_.demodulate(audio, count);
        else
            rx_nrzi = afsk_demod_.demodulate(audio, count);

        auto rx_bits = nrzi_decoder_.decode(rx_nrzi);
        for (uint8_t b : rx_bits) {
            if (hdlc_decoder_.push_bit(b)) {
                const auto& frame = hdlc_decoder_.frame();
                // UI frame: 7(dst) + 7(src) + 1(ctrl) + 1(PID) = 16 bytes header
                if (frame.size() > 16) {
                    handle_cal_frame(frame.data() + 16, frame.size() - 16);
                }
            }
        }
    }
}

} // namespace iris
