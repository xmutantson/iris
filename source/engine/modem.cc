#include "engine/modem.h"
#include "ax25/ax25_protocol.h"
#include "native/frame.h"
#include "fec/ldpc.h"
#include "common/logging.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <random>

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

    // Save original band/PHY for restore on disconnect after probe changes
    orig_band_low_hz_ = config_.band_low_hz;
    orig_band_high_hz_ = config_.band_high_hz;
    orig_phy_config_ = phy_config_;

    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);

    // Speed level cache: persist proven levels to disk
    if (!config_.data_dir.empty())
        gearshift_.set_cache_dir(config_.data_dir);

    afsk_demod_.set_preemph_alpha(config_.preemph_alpha);

    local_cap_.version = XID_VERSION;
    local_cap_.capabilities = CAP_MODE_A | CAP_COMPRESSION | CAP_STREAMING;
    if (config_.mode == "B" || config_.mode == "b")
        local_cap_.capabilities |= CAP_MODE_B;
    if (config_.mode == "C" || config_.mode == "c")
        local_cap_.capabilities |= CAP_MODE_C;
    if (config_.encryption_mode > 0) {
        local_cap_.capabilities |= CAP_ENCRYPTION;
        local_cap_.capabilities |= CAP_PQ_CRYPTO;
    }
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
        std::lock_guard<std::mutex> lock(tx_mutex_);
        // Wrap probe result in AX.25 UI frame, send via AFSK.
        // Use session peer if available, else probe_peer_call_, else broadcast.
        std::string dest = ax25_session_.remote_callsign();
        if (dest.empty()) dest = probe_peer_call_;
        if (dest.empty()) dest = "PROBE";
        auto frame = ax25_build_u(
            ax25_make_addr(dest),
            ax25_make_addr(config_.callsign),
            AX25_CTRL_UI, false, true);
        frame.push_back(AX25_PID_NONE);
        frame.insert(frame.end(), data, data + len);
        ax25_tx_queue_.push(std::move(frame));
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

    // Initialize simulated FM de-emphasis filter if configured
    if (config_.sim_deemph_us > 0) {
        sim_deemph_enabled_ = true;
        // First-order lowpass: H(s) = 1/(1+sτ) → bilinear transform
        float tau = config_.sim_deemph_us * 1e-6f;
        float fs = (float)config_.sample_rate;
        float wc = 1.0f / tau;  // corner angular frequency
        // Bilinear transform: s = (2*fs)*(z-1)/(z+1)
        float K = 2.0f * fs;
        float a = K + wc;
        sim_deemph_.b0 = wc / a;
        sim_deemph_.b1 = wc / a;
        sim_deemph_.b2 = 0;
        sim_deemph_.a1 = (wc - K) / a;
        sim_deemph_.a2 = 0;
        sim_deemph_.z1 = sim_deemph_.z2 = 0;
        float corner_hz = wc / (2.0f * 3.14159265f);
        IRIS_LOG("Simulated FM de-emphasis: %.0f µs (corner %.0f Hz, ~6 dB/octave above)",
                 config_.sim_deemph_us, corner_hz);
    }

    // Wire ARQ session
    arq_.set_callsign(config_.callsign);
    arq_.set_local_capabilities(local_cap_.capabilities);
    probe_.set_local_caps(local_cap_.capabilities);
    ArqCallbacks arq_cb;
    arq_cb.send_frame = [this](const uint8_t* data, size_t len) {
        // ARQ frames go directly to TX queue (not back through ARQ)
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(std::vector<uint8_t>(data, data + len));
    };
    arq_cb.on_data_received = [this](const uint8_t* data, size_t len) {
        // Intercept ML-KEM key exchange frames BEFORE decryption
        // (KX frames are sent unencrypted over the X25519-encrypted channel)
        if (len > 1 && (data[0] == MLKEM_PK_MAGIC || data[0] == MLKEM_CT_MAGIC)) {
            handle_mlkem_frame(data, len);
            return;
        }

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
                bool kx_ok = false;
                // Compute X25519 shared secret from peer's public key
                // (exchanged via CONNECT/CONNECT_ACK payloads)
                if (arq_.has_peer_x25519()) {
                    int rc = cipher_.compute_x25519_shared(arq_.peer_x25519_pubkey());
                    if (rc == 0) {
                        IRIS_LOG("[CRYPTO] X25519 DH key exchange complete");
                        kx_ok = true;
                    } else {
                        IRIS_LOG("[CRYPTO] X25519 shared secret computation failed (bad peer key?)");
                        crypto_state_ = 3;  // crypto failure
                    }
                } else {
                    IRIS_LOG("[CRYPTO] WARNING: peer advertised CAP_ENCRYPTION but no X25519 pubkey");
                    crypto_state_ = 1;  // KEY EXCHANGE incomplete
                }

                if (kx_ok) {
                    // Parse PSK from hex (authentication binding, not encryption key material)
                    std::vector<uint8_t> psk;
                    for (size_t i = 0; i + 1 < config_.psk_hex.size(); i += 2) {
                        char byte_str[3] = {config_.psk_hex[i], config_.psk_hex[i+1], 0};
                        psk.push_back((uint8_t)strtol(byte_str, nullptr, 16));
                    }

                    bool is_commander = (arq_.role() == ArqRole::COMMANDER);
                    crypto_direction_ = is_commander ? DIR_CMD_TO_RSP : DIR_RSP_TO_CMD;
                    cipher_.derive_session_key(config_.callsign.c_str(),
                                                arq_.remote_callsign().c_str(),
                                                psk.empty() ? nullptr : psk.data(),
                                                (int)psk.size(), false);
                    cipher_.activate();
                    tx_batch_counter_ = 0;
                    rx_batch_counter_ = 0;
                    crypto_state_ = 2;  // ENCRYPTED (X25519-only, upgrading to hybrid)
                    IRIS_LOG("[CRYPTO] Session encrypted (X25519 ECDH + ChaCha20-Poly1305)");

                    // Start ML-KEM-768 post-quantum upgrade (commander initiates)
                    if (arq_.negotiated(CAP_PQ_CRYPTO)) {
                        mlkem_kx_pending_ = true;
                        if (arq_.role() == ArqRole::COMMANDER) {
                            start_mlkem_exchange();
                        }
                        // strict mode: don't release data until hybrid KX done
                        // fast mode: data flows immediately with X25519-only key
                    }
                }
            } else if (config_.encryption_mode > 0) {
                crypto_state_ = 1;  // KEY EXCHANGE (wanted encryption but peer didn't negotiate)
            }

            // Init B2F handler if negotiated
            if (arq_.negotiated(CAP_B2F_UNROLL)) {
                b2f_handler_.init();
                b2f_handler_.unroll_enabled = true;
            }

            // Native hail: both sides are proven Iris-capable, go native.
            if (config_.native_hail && !native_mode_) {
                native_mode_ = true;
                native_tx_ready_ = true;
                IRIS_LOG("Native hail: native mode active");
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
            mlkem_kx_pending_ = false;
            mlkem_held_frames_.clear();
            b2f_handler_.deinit();

            // Reset native mode so next session starts clean.
            // Guard: if relisten_pending_ is already set, this IDLE came from
            // listen()→reset()→set_state(IDLE) — don't re-trigger or clear buffer.
            if (config_.native_hail && !relisten_pending_) {
                native_mode_ = false;
                native_tx_ready_ = false;
                rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                pending_frame_timeout_ = 0;
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
    ax25_session_.set_txdelay_ms(config_.ptt_pre_delay_ms);  // Set T1 floor from initial TXDELAY
    ax25_session_.set_send_callback([this](const uint8_t* data, size_t len) {
        // Route session frames: native (OFDM) if upgrade complete, else AFSK.
        // Responder keeps ofdm_kiss_tx_=false until hearing first native frame,
        // so its session frames (RR etc) still go via AFSK during transition.
        // This is safe: the window is short (~1s) and AX.25 retry handles drops.
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (ofdm_kiss_tx_) {
            std::vector<uint8_t> frame(data, data + len);
            // Peer SNR feedback: append our RX SNR to S-frames (RR/REJ/RNR)
            // so the peer can cap its TX speed to what we can actually decode.
            // Encoding: 1 byte, SNR * 4 (0.25 dB steps), 0 = no data.
            // Only appended on native PHY — old peers or AFSK ignore extra bytes.
            if (len == 15 && (data[14] & 0x03) == 0x01) {  // S-frame (15 bytes, ctrl bit 0 = 1)
                // Use DD (post-Kalman) SNR for peer feedback — with RTS smoother,
                // DD SNR accurately reflects decoded signal quality and is typically
                // higher than preamble SNR on FM links (preamble biased low by ISI)
                uint8_t snr_byte = 0;
                if (snr_db_ > 0)
                    snr_byte = (uint8_t)std::min(255.0f, std::max(1.0f, snr_db_ * 4.0f));
                frame.push_back(snr_byte);
            }
            tx_queue_.push(std::move(frame));
        } else {
            ax25_tx_queue_.push(std::vector<uint8_t>(data, data + len));
            // Buffer I-frame info fields during AFSK phase for B2F replay.
            // The B2F handler isn't initialized until OFDM-KISS activates, but
            // the SID/FC/FS exchange happens during AFSK BEFORE the probe.
            // Buffer unconditionally so replay works when handler initializes.
            if (config_.b2f_unroll && len > 16) {
                uint8_t ctrl = data[14];
                if ((ctrl & 0x01) == 0) {  // I-frame
                    b2f_afsk_tx_history_.emplace_back(data + 16, data + len);
                }
            }
        }
    });
    ax25_session_.set_data_callback([this](const uint8_t* data, size_t len) {
        if (rx_callback_) rx_callback_(data, len);
    });
    ax25_session_.set_state_callback([this](Ax25SessionState state, const std::string& remote) {
        IRIS_LOG("AX25 state -> %d (remote=%s)", (int)state, remote.c_str());

        // Probe-after-connect: let SABM/UA complete first so Winlink is happy,
        // then probe while holding I-frames. Only the connection initiator
        // starts the probe; the responder waits for PROBE:START.
        if (state == Ax25SessionState::CONNECTED &&
            ax25_session_.we_initiated() &&
            !config_.ax25_only && !ofdm_kiss_probing_ &&
            ofdm_kiss_probe_cd_ == 0 && !ofdm_kiss_probe_done_) {
            // Defer PROBE:START — this callback may fire from queue_tx_frame()
            // which holds tx_mutex_. send_probe_start_ui() also needs tx_mutex_.
            // Set flag and let tick() send it (outside the mutex).
            xid_peer_call_ = remote;
            probe_start_pending_ = true;
            ofdm_kiss_probe_cd_ = 20;  // 1s countdown before tones
            probe_connect_timeout_ = 700;  // 35s overall timeout (15s initiator + 12s responder captures)
            IRIS_LOG("Probe-after-connect: probing %s, I-frames held until done", remote.c_str());
            if (gui_log_) gui_log_("Probing " + remote + "...");
        }

        // AWAITING_RELEASE (DISCONNECTING): immediately fall back to AX.25.
        // Disable native/OFDM-KISS so DISC/UA exchange uses AFSK only.
        // Start 30s timeout — if stuck, stop TX'ing entirely.
        if (state == Ax25SessionState::AWAITING_RELEASE) {
            IRIS_LOG("DISCONNECTING: falling back to AX.25 immediately");
            ofdm_kiss_ = false;
            ofdm_kiss_tx_ = false;
            ofdm_kiss_confirmed_ = false;
            ofdm_kiss_probing_ = false;
            ofdm_kiss_probe_cd_ = 0;
            probe_start_pending_ = false;
            native_mode_ = false;
            native_tx_ready_ = false;
            disconnect_timeout_ticks_ = 600;  // 30s at 50ms/tick
        }

        if (state == Ax25SessionState::DISCONNECTED) {
            disconnect_timeout_ticks_ = 0;
            pending_connect_call_.clear();
            probe_connect_timeout_ = 0;
            if (ofdm_kiss_ || ofdm_kiss_tx_ || ofdm_kiss_probe_done_) {
                IRIS_LOG("KISS-over-OFDM disabled (AX.25 disconnected)");
                ofdm_kiss_ = false;
                ofdm_kiss_tx_ = false;
                ofdm_kiss_confirmed_ = false;
                ofdm_kiss_probing_ = false;
                ofdm_kiss_probe_cd_ = 0;
                ofdm_kiss_probe_done_ = false;
                probe_manual_ = false;
                probe_start_pending_ = false;
                probe_.reset();
                batch_airtime_s_ = BATCH_AIRTIME_MIN;
                // Clean up OFDM-KISS compression and B2F proxy
                if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
                    ofdm_kiss_tx_compressor_.deinit();
                    ofdm_kiss_rx_compressor_.deinit();
                }
                if (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) {
                    ofdm_kiss_b2f_.deinit();
                    b2f_proxy_plaintext_.clear();
                    b2f_proxy_plaintext_.shrink_to_fit();
                }
                b2f_afsk_tx_history_.clear();
                b2f_afsk_rx_history_.clear();
                ofdm_kiss_peer_caps_ = 0;
                rx_channel_eq_.reset();
                tx_channel_eq_.reset();
                b2f_proxy_active_ = false;
                b2f_proxy_rx_active_ = false;
                b2f_proxy_addr_valid_ = false;
                b2f_proxy_vr_ = 0;
            }

            // Restore original band/baud/center if probe changed them
            if (config_.band_low_hz != orig_band_low_hz_ ||
                config_.band_high_hz != orig_band_high_hz_ ||
                phy_config_.baud_rate != orig_phy_config_.baud_rate) {
                IRIS_LOG("Restoring original PHY: band %.0f-%.0f Hz, baud %d (was %.0f-%.0f Hz, baud %d)",
                         orig_band_low_hz_, orig_band_high_hz_, orig_phy_config_.baud_rate,
                         config_.band_low_hz, config_.band_high_hz, phy_config_.baud_rate);
                config_.band_low_hz = orig_band_low_hz_;
                config_.band_high_hz = orig_band_high_hz_;
                phy_config_ = orig_phy_config_;
                if (use_upconvert_) {
                    float center = (orig_band_low_hz_ + orig_band_high_hz_) / 2.0f;
                    upconverter_ = Upconverter(center, config_.sample_rate);
                    downconverter_ = Downconverter(center, config_.sample_rate);
                }
                native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
                native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
            }

            peer_is_iris_ = false;
            native_mode_ = false;
            dcd_inverted_ = false;  // re-detect each connection
            native_tx_ready_ = false;
            native_selfhear_guard_ = 0;  // Don't discard frames from next connection
            rx_overlap_buf_.clear();
            pending_frame_start_ = -1;
            pending_frame_timeout_ = 0;
        }

        // Forward all state changes immediately to the GUI/AGW layer.
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
        // Pause AX.25 timers while we're transmitting (Direwolf pattern).
        // We can't expect an ACK while PTT is keyed.
        ax25_session_.set_channel_busy(true);
    }
}

void Modem::ptt_off() {
    if (ptt_active_ && ptt_) {
        ptt_->set_ptt(false);
        ptt_active_ = false;
        rx_mute_holdoff_ = RX_MUTE_HOLDOFF_SAMPLES;
        rx_raw_rms_ = 0;   // Clear stale DCD reading from our own TX
        dcd_holdoff_ = 0;

        // Role-asymmetric post-TX listen window.
        // After transmitting, yield to the peer's response.  Initiator yields
        // longer so the responder can ACK/respond first.  Random jitter
        // desynchronizes after any collision.
        if (!loopback_mode_ && ax25_session_.is_active()) {
            static thread_local std::minstd_rand rng(std::random_device{}());
            int jitter = rng() % (config_.sample_rate / 2);  // 0-500ms random
            int base = ax25_session_.we_initiated()
                ? config_.sample_rate * 3 / 2      // initiator: 1.5s base (yield to response)
                : config_.sample_rate;             // responder: 1.0s base (respond sooner)
            csma_holdoff_ = std::max(csma_holdoff_.load(), base + jitter);
        }

        // Resume AX.25 timers now that channel is free
        ax25_session_.set_channel_busy(false);
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
            pending_frame_timeout_ = 0;
        }
    }

    // Count down self-hear guard (continues even during mute)
    if (native_selfhear_guard_ > 0)
        native_selfhear_guard_ -= frame_count;

    // Feed probe analyzer after mute guard — never capture our own TX.
    if (ofdm_kiss_probing_ && probe_.state() == ProbeState::LISTENING_PROBE && !ptt_active_)
        probe_.feed_rx(rx_audio, frame_count);

    // Auto-DCD baseline calibration: measure noise floor for 2 seconds at startup.
    // Once baseline is known, detect inverted squelch (signal < noise).
    if (config_.dcd_auto && !dcd_baseline_done_ && !ptt_active_ && !rx_muted_) {
        dcd_baseline_samples_ += frame_count;
        // Exponential moving average of RMS
        float alpha = (dcd_baseline_rms_ == 0) ? 1.0f : 0.1f;
        dcd_baseline_rms_ = dcd_baseline_rms_ * (1.0f - alpha) + rx_raw_rms_ * alpha;
        if (dcd_baseline_samples_ >= config_.sample_rate * 2) {  // 2 seconds
            dcd_baseline_done_ = true;
            IRIS_LOG("Auto-DCD: baseline RMS=%.4f (measured over 2s)", dcd_baseline_rms_);
            if (gui_log_) gui_log_("Auto-DCD: baseline " + std::to_string(dcd_baseline_rms_));
        }
    }

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

    if (!ptt_active_)
        compute_spectrum(audio.data(), frame_count);

    if (state_ == ModemState::CALIBRATING) {
        process_calibration_rx(audio.data(), frame_count);
        return;
    }

    if (native_mode_) {
        process_rx_native(audio.data(), frame_count);
    } else {
        process_rx_ax25(audio.data(), frame_count);

        // Always run native demod alongside AFSK — it's lightweight (just
        // preamble correlation until a frame arrives) and lets us detect native
        // frames at any point without waiting for the handshake to complete.
        // Self-hear is handled by native_selfhear_guard_.
        if (!config_.ax25_only && native_demod_) {
            process_rx_native(audio.data(), frame_count);
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

    // When OFDM-KISS is fully bidirectional, suppress AFSK-decoded session
    // frames — only OFDM-decoded frames should feed the AX.25 session.
    // CRITICAL: require ofdm_kiss_confirmed_ (we've heard a native frame from the peer).
    // Without this, the initiator goes deaf to the responder's AFSK frames
    // before the responder has activated native TX.
    if (ofdm_kiss_ && ofdm_kiss_confirmed_ && !from_ofdm) {
        // Check if it's a UI frame — those still need AFSK path for conn header
        bool is_ui = (frame.size() > 15 &&
                      (frame[14] & ~AX25_PF_MASK) == AX25_CTRL_UI);
        if (!is_ui) {
            // Drop non-UI AFSK frames — OFDM handles session traffic now
            return;
        }
    }

    // CSMA: holdoff after frame decode to avoid stepping on a burst.
    // Role-asymmetric: responder ACKs quickly, initiator yields longer.
    // Creates ~800ms exclusive window where only the responder can TX,
    // eliminating the post-decode race where both holdoffs expire together.
    // Resets on each frame, so holdoff extends past the LAST frame in a burst.
    {
        int rx_holdoff;
        if (ax25_session_.is_active()) {
            rx_holdoff = ax25_session_.we_initiated()
                ? config_.sample_rate * 6 / 5   // initiator: 1200ms (yield to response)
                : config_.sample_rate * 2 / 5;  // responder: 400ms (ACK quickly)
        } else {
            rx_holdoff = config_.sample_rate * 4 / 5;  // no session: 800ms
        }
        csma_holdoff_ = std::max(csma_holdoff_.load(), rx_holdoff);
    }

    frames_rx_++;

    // Auto-DCD inverted squelch detection: when we successfully decode a frame,
    // check if the current RMS is significantly below the baseline noise floor.
    // If so, the radio has inverted squelch (signal present = less noise).
    // Runs for ALL frame types (AFSK and native) so we detect before native upgrade.
    if (config_.dcd_auto && dcd_baseline_done_ && !dcd_inverted_ && rx_raw_rms_ > 0) {
        if (rx_raw_rms_ < dcd_baseline_rms_ * 0.5f) {
            dcd_inverted_ = true;
            IRIS_LOG("Auto-DCD: inverted squelch detected (signal RMS=%.4f < baseline=%.4f)",
                     rx_raw_rms_.load(), dcd_baseline_rms_);
            if (gui_log_) gui_log_("Auto-DCD: inverted squelch detected");
        }
    }

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

    // Adaptive batch airtime: sniff S-frames before session consumes them.
    // RR = peer ACK'd our batch → grow.  REJ = peer lost a frame → shrink.
    if (ofdm_kiss_ && frame.size() > 14) {
        uint8_t ctrl = frame[14];
        if ((ctrl & 0x03) == 0x01) {  // S-frame
            auto stype = (Ax25SType)((ctrl >> 2) & 0x03);
            if (stype == Ax25SType::RR) {
                float prev = batch_airtime_s_;
                batch_airtime_s_ = std::min(batch_airtime_s_ + 1.0f, BATCH_AIRTIME_MAX);
                if (batch_airtime_s_ != prev)
                    IRIS_LOG("Batch airtime: %.0fs -> %.0fs (RR ACK)", prev, batch_airtime_s_);
                tx_no_ack_count_ = 0;  // peer acknowledged
                // Cache proven speed level for this peer
                gearshift_.save_cached_level(ax25_session_.remote_callsign());
            } else if (stype == Ax25SType::REJ || stype == Ax25SType::RNR) {
                float prev = batch_airtime_s_;
                batch_airtime_s_ = std::max(batch_airtime_s_ / 2.0f, BATCH_AIRTIME_MIN);
                if (batch_airtime_s_ != prev)
                    IRIS_LOG("Batch airtime: %.0fs -> %.0fs (%s)", prev, batch_airtime_s_,
                             stype == Ax25SType::REJ ? "REJ" : "RNR");
                // REJ/RNR = peer couldn't decode our frame. Downshift TX speed.
                // On asymmetric links, local RX SNR doesn't predict peer's RX SNR.
                // Treating peer REJ as a decode failure drives gearshift down so we
                // TX at a rate the peer can actually receive.
                tx_no_ack_count_ = 0;  // peer responded (even if negative)
                gearshift_.report_failure();
                IRIS_LOG("Gearshift: peer %s -> report_failure (level=%d)",
                         stype == Ax25SType::REJ ? "REJ" : "RNR",
                         gearshift_.current_level());
            }
            // Peer SNR feedback: extract appended SNR byte from native S-frames.
            // Standard S-frame is 15 bytes. If 16+ bytes, byte 15 is quantized
            // peer RX SNR (0.25 dB steps). This tells us what the peer measures
            // from our signal — use it to cap TX speed on asymmetric links.
            if (from_ofdm && frame.size() >= 16 && frame[15] > 0) {
                float reported_snr = frame[15] / 4.0f;
                peer_snr_db_ = reported_snr;
                IRIS_LOG("Peer SNR feedback: %.1f dB (from %s)", reported_snr,
                         stype == Ax25SType::RR ? "RR" : "REJ/RNR");
            }
        }
    }

    // Buffer RX I-frame info fields during AFSK phase for B2F replay.
    // Same reason as TX: SID/FC/FS exchange happens over AFSK before OFDM-KISS activates.
    // Buffer unconditionally (config check only) — ofdm_kiss_ isn't set until after probe.
    if (config_.b2f_unroll && !from_ofdm && !ofdm_kiss_b2f_.is_initialized() && frame.size() > 16) {
        uint8_t ctrl = frame[14];
        if ((ctrl & 0x01) == 0) {  // I-frame
            b2f_afsk_rx_history_.emplace_back(frame.data() + 16, frame.data() + frame.size());
        }
    }

    // B2F proxy: feed incoming I-frame info fields to filter_rx for state tracking.
    // This drives B2F state transitions (detects FS responses, FC proposals, etc.)
    // Must run BEFORE ax25_session consumes the frame.
    if (ofdm_kiss_ && from_ofdm && (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
        ofdm_kiss_b2f_.is_initialized() && frame.size() > 16) {
        uint8_t ctrl = frame[14];
        if ((ctrl & 0x01) == 0) {  // I-frame
            const uint8_t* info = frame.data() + 16;
            int info_len = (int)(frame.size() - 16);
            std::vector<uint8_t> b2f_rx_out(info_len * 2 + 4096);
            ofdm_kiss_b2f_.filter_rx(
                (const char*)info, info_len,
                (char*)b2f_rx_out.data(), (int)b2f_rx_out.size());
            // Output ignored — original I-frame delivered to Winlink as-is
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

    // Detect CAL: UI frames (ctrl=0x03, PID at offset 15, info at 16+)
    bool is_cal = false;
    bool is_probe = false;
    if (frame.size() > 20) {
        uint8_t ctrl = frame[14] & ~AX25_PF_MASK;
        if (ctrl == AX25_CTRL_UI && frame.size() > 16) {
            const uint8_t* info = frame.data() + 16;
            size_t info_len = frame.size() - 16;
            if (info_len >= 4 && info[0] == 'C' && info[1] == 'A' &&
                info[2] == 'L' && info[3] == ':') {
                handle_cal_frame(info, info_len);
                is_cal = true;
            }
            // PROBE:START UI frame — remote wants us to start probe responder.
            if (!is_cal && info_len >= 11 &&
                memcmp(info, "PROBE:START", 11) == 0) {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                if (ui_src != config_.callsign && !ofdm_kiss_probing_ &&
                    ofdm_kiss_probe_cd_ == 0) {
                    IRIS_LOG("[PROBE] Received PROBE:START from %s — starting responder",
                             ui_src.c_str());
                    if (gui_log_) gui_log_("Probe: " + ui_src + " requested probe");
                    probe_peer_call_ = ui_src;
                    // PROBE:START from a peer is always an auto-probe
                    // (apply PHY + enable native on completion).
                    // Manual probe is only triggered by the local Probe button.
                    probe_manual_ = false;
                    ofdm_kiss_probing_ = true;
                    ofdm_kiss_probe_done_ = false;
                    // Responder capture window: must be long enough to catch
                    // initiator's ~2.25s probe tones (which arrive ~3-5s after
                    // PROBE:START), but short enough to reply while the initiator
                    // is still listening (initiator has 12s capture).
                    // Tones arrive ~5-7s after PROBE:START (countdown + PROBE:START
                    // TX + PTT delays). 10s window catches tones with margin,
                    // finishes by t+10, sends reply by t+13 — within initiator's
                    // 12s capture (which starts after tones, not after PROBE:START).
                    probe_.start_responder(config_.sample_rate, 10.0f);
                }
                is_probe = true;
            }
            // Probe RESULT in UI frame (probe data wrapped in AX.25 UI).
            if (!is_cal && ofdm_kiss_probing_ && !ofdm_kiss_probe_done_ &&
                info_len >= 1 && info[0] == PROBE_MSG_RESULT) {
                std::string ui_src;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') ui_src += c;
                }
                if (ui_src != config_.callsign) {
                    IRIS_LOG("[PROBE] Got probe result in UI frame from %s (%zu bytes)",
                             ui_src.c_str(), info_len);
                    probe_.on_message(info, info_len);
                    is_probe = true;
                }
            }

            // (Connection header removed — probe-first replaces header exchange)
        }
    }

    // Legacy probe detection (raw probe messages, non-OFDM-KISS)
    if (!is_probe && !is_cal && frame.size() >= 2) {
        uint8_t first = frame[0];
        if (first == PROBE_MSG_RESULT) {
            probe_.on_message(frame.data(), frame.size());
            is_probe = true;
        }
    }

    if (!is_probe && frame.size() >= 14) {
        std::string rx_src;
        for (int ci = 7; ci < 13; ci++) {
            char c = (char)(frame[ci] >> 1);
            if (c != ' ') rx_src += c;
        }
        if (rx_src != config_.callsign) {
            if (gui_log_)
                gui_log_("[RX] " + describe_ax25(frame.data(), frame.size()));

            // When AX.25 session is active (AGW mode), don't deliver raw frames
            // to rx_callback_ — the session's data callback delivers I-frame data.
            // BUT when KISS-managed, the KISS client (e.g. Winlink) runs its own
            // AX.25 state machine and needs ALL frames (UA, RR, I-frames, etc.).
            // Blocking them causes the KISS client to never see responses, leading
            // to infinite SABM retries and connection failure.
            auto ax_st = ax25_session_.state();
            bool session_active = (ax_st == Ax25SessionState::CONNECTED ||
                                   ax_st == Ax25SessionState::TIMER_RECOVERY ||
                                   ax_st == Ax25SessionState::AWAITING_CONNECTION ||
                                   ax_st == Ax25SessionState::AWAITING_RELEASE);
            if ((!session_active || ax25_session_.is_kiss_managed()) && rx_callback_)
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
    // Legacy probe feed (ARQ native mode only — OFDM-KISS feeds from process_rx
    // with !ptt_active_ guard to avoid capturing our own TX).
    if (!ofdm_kiss_ && !ofdm_kiss_probing_ && probe_.state() == ProbeState::LISTENING_PROBE) {
        probe_.feed_rx(audio, count);
    }

    std::vector<uint8_t> rx_nrzi;
    if (config_.ax25_baud == 9600)
        rx_nrzi = gfsk_demod_.demodulate(audio, count);
    else
        rx_nrzi = afsk_demod_.demodulate(audio, count);

    auto rx_bits = nrzi_decoder_.decode(rx_nrzi);
    if (config_.ax25_baud == 9600)
        g3ruh_rx_scrambler_.descramble(rx_bits);

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
        // Channel equalization: flatten FM de-emphasis before downconversion.
        // Operates on real passband audio so both I and Q see equalized signal.
        if (rx_channel_eq_.is_configured()) {
            std::vector<float> eq_audio(audio, audio + count);
            rx_channel_eq_.apply(eq_audio.data(), count);
            iq_buf = downconverter_.audio_to_iq(eq_audio.data(), count);
        } else {
            iq_buf = downconverter_.audio_to_iq(audio, count);
        }
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
        if (pending_frame_timeout_ <= 0) {
            // Timeout expired — abandon this frame (sender may have stopped)
            IRIS_LOG("RX pending frame TIMEOUT at offset %d (needed %zu IQ, have %zu)",
                     pending_frame_start_, pending_need_floats_ / 2,
                     rx_overlap_buf_.size() / 2);
            pending_frame_start_ = -1;
            pending_frame_timeout_ = 0;
        } else if (rx_overlap_buf_.size() < pending_need_floats_) {
            return;  // Still not enough data, skip expensive work
        } else {
            IRIS_LOG("RX pending retry: buf=%zu floats, need=%zu, start=%d",
                     rx_overlap_buf_.size(), pending_need_floats_, pending_frame_start_);
        }
    }

    int start;
    float det_corr;
    if (pending_frame_start_ >= 0) {
        // Re-use cached detection result
        start = pending_frame_start_;
        det_corr = 0.9f;  // Known good
        pending_frame_start_ = -1;
        pending_frame_timeout_ = 0;
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
                         payload.size(), native_selfhear_guard_.load());
                // Skip past this frame so we don't re-decode it
                size_t skip = (size_t)(start + 100) * 2;
                if (skip < rx_overlap_buf_.size())
                    rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                           rx_overlap_buf_.begin() + skip);
                else
                    rx_overlap_buf_.clear();
                pending_frame_start_ = -1;
                pending_frame_timeout_ = 0;
                return;
            }
            frames_rx_++;
            IRIS_LOG("RX native frame %zu bytes at offset %d", payload.size(), start);

            {
                // Use SNR computed inside the frame decoder, which has:
                // - RRC-filtered samples
                // - Fine timing (sub-sample interpolation)
                // - Phase + frequency offset correction
                // This is much more accurate than external estimation.
                float snr = decode_snr_db();
                snr_db_ = snr;
                snr_preamble_db_ = decode_snr_preamble_db();
                int ldpc_iters = ldpc_last_max_iters();
                gearshift_.feed_ldpc_iters(ldpc_iters, 50);
                int old_level = gearshift_.current_level();
                gearshift_.update(snr);
                arq_.set_local_snr(snr);
                int new_level = gearshift_.current_level();
                IRIS_LOG("[SNR] est=%.1f dB, ldpc_iters=%d, boost=+%.1f, eff=%.1f, gearshift: %d->%d (smoothed=%.1f, cd=%d)",
                         snr, ldpc_iters, gearshift_.boost(),
                         gearshift_.smoothed_snr() + gearshift_.boost(),
                         old_level, new_level, gearshift_.smoothed_snr(), gearshift_.cooldown());
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
                    if (!ofdm_kiss_confirmed_) {
                        ofdm_kiss_confirmed_ = true;
                        IRIS_LOG("OFDM-KISS: native confirmed (heard native from peer)");
                        if (gui_log_) gui_log_("OFDM-KISS: bidirectional confirmed");
                    }
                    if (!ofdm_kiss_tx_) {
                        // Responder: heard first native frame from initiator → promote TX
                        ofdm_kiss_tx_ = true;
                        ax25_session_.set_native_active(true);
                        IRIS_LOG("OFDM-KISS: TX promoted to native (responder)");
                        if (gui_log_) gui_log_("OFDM-KISS: native TX active (responder)");
                        // Speed level cache: start at cached level for this peer
                        {
                            int cached = gearshift_.load_cached_level(ax25_session_.remote_callsign());
                            if (cached > 0) {
                                gearshift_.force_level(cached);
                                IRIS_LOG("Gearshift: cached level %d for %s",
                                         cached, ax25_session_.remote_callsign().c_str());
                            }
                        }
                        // Migrate accumulated I-frames from AFSK to native queue
                        {
                            std::lock_guard<std::mutex> lock(tx_mutex_);
                            int migrated = 0;
                            std::queue<std::vector<uint8_t>> keep;
                            while (!ax25_tx_queue_.empty()) {
                                auto fr = std::move(ax25_tx_queue_.front());
                                ax25_tx_queue_.pop();
                                bool is_iframe = fr.size() > 14 && (fr[14] & 0x01) == 0;
                                if (is_iframe) {
                                    tx_queue_.push(std::move(fr));
                                    migrated++;
                                } else {
                                    keep.push(std::move(fr));
                                }
                            }
                            ax25_tx_queue_ = std::move(keep);
                            if (migrated > 0)
                                IRIS_LOG("OFDM-KISS: migrated %d I-frames to native (responder)", migrated);
                        }
                    }

                    // B2F proxy RX: handle B2F_DATA frames (unrolled plaintext from remote)
                    if (len >= 2 && data[0] == B2F_DATA_MAGIC &&
                        (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
                        ofdm_kiss_b2f_.is_initialized()) {
                        // Decompress the B2F data block
                        std::vector<uint8_t> decomp(len * 4 + 4096);
                        int dec_len = ofdm_kiss_rx_compressor_.decompress_block(
                            data + 1, (int)(len - 1),
                            decomp.data(), (int)decomp.size());
                        if (dec_len <= 0) {
                            // Decompression failed — try as raw uncompressed
                            dec_len = (int)(len - 1);
                            decomp.assign(data + 1, data + len);
                        }

                        IRIS_LOG("[B2F-PROXY] RX: received B2F_DATA %zu bytes -> %d decompressed",
                                 len - 1, dec_len);

                        // Feed decompressed plaintext to filter_rx for rerolling
                        std::vector<uint8_t> lzhuf_out(dec_len * 2 + 4096);
                        int lzhuf_len = ofdm_kiss_b2f_.filter_rx(
                            (const char*)decomp.data(), dec_len,
                            (char*)lzhuf_out.data(), (int)lzhuf_out.size());

                        if (lzhuf_len > 0 && rx_callback_) {
                            // Construct I-frames with rerolled LZHUF and inject to Winlink
                            b2f_proxy_rx_active_ = true;
                            Ax25Address dst = ax25_make_addr(config_.callsign);
                            Ax25Address src = ax25_make_addr(ax25_session_.remote_callsign());
                            uint8_t ns = ax25_session_.vr();  // Continue from last known N(S)

                            IRIS_LOG("[B2F-PROXY] RX: rerolled %d bytes LZHUF, injecting I-frames (ns=%d)",
                                     lzhuf_len, ns);

                            // Split into MAX_INFO-sized I-frames
                            constexpr int MAX_INFO = 256;
                            int offset = 0;
                            while (offset < lzhuf_len) {
                                int chunk = std::min(MAX_INFO, lzhuf_len - offset);
                                auto iframe = ax25_build_i(dst, src, ns, 0, false,
                                    AX25_PID_NONE,
                                    lzhuf_out.data() + offset, chunk);
                                rx_callback_(iframe.data(), iframe.size());
                                ns = (ns + 1) & 0x07;
                                offset += chunk;
                            }
                        }

                        // Check if B2F handler exited payload transfer (all proposals done)
                        if (!ofdm_kiss_b2f_.is_rx_payload_active()) {
                            b2f_proxy_rx_active_ = false;
                            IRIS_LOG("[B2F-PROXY] RX: payload transfer complete");
                        }
                        return;
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

            // OFDM-KISS batch decompression: if peer compressed, decompress first.
            if (ofdm_kiss_ && payload.size() >= 2 &&
                payload[0] == COMPRESSED_PAYLOAD_MAGIC &&
                (ofdm_kiss_peer_caps_ & CAP_COMPRESSION)) {
                std::vector<uint8_t> decomp_buf(payload.size() * 4 + 4096);
                int dec_len = ofdm_kiss_rx_compressor_.decompress_block(
                    payload.data() + 1, (int)(payload.size() - 1),
                    decomp_buf.data(), (int)decomp_buf.size());
                if (dec_len > 0) {
                    IRIS_LOG("OFDM-KISS decompress: %zu -> %d bytes",
                             payload.size() - 1, dec_len);
                    payload.assign(decomp_buf.begin(), decomp_buf.begin() + dec_len);
                } else {
                    IRIS_LOG("OFDM-KISS: decompression failed (%zu bytes), dropping",
                             payload.size());
                    rx_overlap_buf_.clear();
                    pending_frame_start_ = -1;
                    pending_frame_timeout_ = 0;
                    return;
                }
            }

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
            // Timeout: allow enough time for the full frame to arrive, plus margin.
            // At SPS=60, 4949 symbols = 6.2s; use 10s to cover largest frames + radio delays.
            pending_frame_timeout_ = config_.sample_rate * 10;
            IRIS_LOG("RX decode: need more data at offset %d (buf=%zu IQ, need ~%zu, timeout=10s)",
                     start, rx_overlap_buf_.size() / 2, pending_need_floats_ / 2);
        } else {
            crc_errors_++;
            gearshift_.report_failure();
            IRIS_LOG("RX decode FAIL at offset %d corr=%.3f (crc_errors=%d, gearshift->%d)",
                     start, det_corr, (int)crc_errors_, gearshift_.current_level());
            // NACK: in OFDM-KISS mode, send AX.25 REJ immediately so the peer
            // retransmits without waiting for T1 timeout (~30s).
            if (ofdm_kiss_ && ax25_session_.is_active()) {
                ax25_session_.request_retransmit();
            }
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

        // Apply simulated channel effects to TX audio
        if (sim_bp_enabled_) {
            for (int i = 0; i < frame_count; i++) {
                float s = tx_audio[i];
                for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                tx_audio[i] = s;
            }
        }
        if (sim_deemph_enabled_) {
            for (int i = 0; i < frame_count; i++)
                tx_audio[i] = sim_deemph_.process(tx_audio[i]);
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
                // Role-asymmetric: responder responds sooner, initiator yields.
                {
                    int native_listen = (ax25_session_.is_active() && ax25_session_.we_initiated())
                        ? config_.sample_rate * 3      // initiator: 3.0s (yield to response)
                        : config_.sample_rate * 2;     // responder: 2.0s (respond sooner)
                    csma_holdoff_ = std::max(csma_holdoff_.load(), native_listen);
                }
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
    // Tone-based DCD is disabled — on real radios the AFSK correlator can't
    // distinguish signal (~30) from noise (~22), so any threshold either
    // never fires or permanently locks TX.  AFSK turn-taking relies on
    // post-TX holdoff (role-asymmetric) + post-RX burst guard instead.
    // Energy-based DCD is still used in native/OFDM-KISS mode.
    bool dcd_busy = false;
    if (!loopback_mode_ && !ofdm_kiss_probing_ &&
        config_.dcd_threshold > 0 && (native_mode_ || ofdm_kiss_)) {
        if (dcd_inverted_) {
            dcd_busy = rx_raw_rms_ < dcd_baseline_rms_ * 0.5f;
        } else {
            dcd_busy = rx_raw_rms_ > config_.dcd_threshold;
        }
    }
    if (dcd_busy) {
        int holdoff_samples = config_.dcd_holdoff_ms * config_.sample_rate / 1000;
        dcd_holdoff_ = holdoff_samples;
        csma_slot_timer_ = 0;  // reset p-persist slot on new carrier
    }
    if (dcd_holdoff_ > 0) {
        dcd_holdoff_ -= frame_count;
        if (dcd_holdoff_ < 0) dcd_holdoff_ = 0;
    }

    // CSMA guard: wait after last frame decode before starting TX
    // Counted in samples for deterministic timing regardless of audio buffer size.
    if (csma_holdoff_ > 0) {
        csma_holdoff_ -= frame_count;
        if (csma_holdoff_ < 0) csma_holdoff_ = 0;
    }

    // Consolidated channel busy: pause AX.25 T1/T3 whenever TX is blocked.
    // On half-duplex radio we can't expect an acknowledgment while we can't
    // transmit (Direwolf pattern from ax25_link.c:7035-7084).
    // Covers: DCD active, DCD holdoff, CSMA holdoff, PTT (in ptt_on/off),
    // probing, and pending native frame (waiting for more data after preamble).
    bool native_frame_pending = (pending_frame_start_ >= 0 && pending_frame_timeout_ > 0);
    bool tx_blocked = dcd_busy || (dcd_holdoff_ > 0) || (csma_holdoff_ > 0)
                      || ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0
                      || native_frame_pending;
    ax25_session_.set_channel_busy(tx_blocked);

    if (dcd_busy || dcd_holdoff_ > 0) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    if (csma_holdoff_ > 0) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    // Suppress TX while waiting for a native frame to arrive.
    // We detected a valid preamble and need more audio data to decode
    // the full frame.  Keying PTT would destroy the incoming frame on
    // a half-duplex radio.
    if (native_frame_pending) {
        pending_frame_timeout_ -= frame_count;
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }
    // Suppress non-probe TX during probe — stray AFSK/native frames
    // would key PTT and prevent hearing the peer's tones.  But let pending
    // probe audio through (it was just queued by start_*() and needs to TX).
    // Also blocks when probe is DONE but tick() hasn't processed completion:
    // prevents accumulated I-frames from blasting out as AFSK before native
    // mode is activated (they'd bury the probe result in a long burst).
    if (ofdm_kiss_probing_ && probe_.state() != ProbeState::IDLE
        && probe_audio_pending_.empty()) {
        std::memset(tx_audio, 0, frame_count * sizeof(float));
        return;
    }

    // p-persistent CSMA (AX.25 2.2 Section 6.4.2)
    // Channel is clear (DCD off, holdoffs expired). Use slotted random access
    // to avoid collisions when both sides detect channel-clear simultaneously.
    // Each slottime period: if random < persist, transmit; else wait another slot.
    // Bypass: In a connected AX.25 session (point-to-point), both sides already
    // alternate TX/RX.  DCD + csma_holdoff_ provide sufficient collision avoidance.
    // Random CSMA backoff just adds latency and increases T1 timeouts.
    bool csma_bypass = ax25_session_.is_active();
    if (config_.persist < 255 && config_.slottime_ms > 0 && !loopback_mode_ && !csma_bypass) {
        if (csma_slot_timer_ > 0) {
            csma_slot_timer_ -= frame_count;
            if (csma_slot_timer_ > 0) {
                std::memset(tx_audio, 0, frame_count * sizeof(float));
                return;
            }
        }
        // Slot expired or first entry — roll the dice
        static thread_local std::minstd_rand csma_rng(std::random_device{}());
        int r = csma_rng() % 256;
        if (r >= config_.persist) {
            // Lost the coin toss — wait another slottime
            csma_slot_timer_ = config_.slottime_ms * config_.sample_rate / 1000;
            std::memset(tx_audio, 0, frame_count * sizeof(float));
            return;
        }
        // Won — fall through to transmit. Reset for next time.
        csma_slot_timer_ = 0;
    }

    {
        // try_lock: avoid blocking the real-time audio callback.  If another
        // thread is pushing a frame (brief), we output silence this round
        // and encode on the next callback (~5-10ms later).
        std::unique_lock<std::mutex> lock(tx_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            std::memset(tx_audio, 0, frame_count * sizeof(float));
            return;
        }

        // Drain forced-AX.25 queue first (probes, etc.),
        // then regular tx_queue_ (native or AX.25 depending on mode).
        bool have_frame = false;
        if (!ax25_tx_queue_.empty()) {
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
                if (config_.ax25_baud == 9600)
                    g3ruh_tx_scrambler_.scramble(raw_bits);
                auto nrzi_bits = nrzi_encode(raw_bits);
                if (config_.ax25_baud == 9600)
                    tx_buffer_ = gfsk_mod_.modulate(nrzi_bits);
                else
                    tx_buffer_ = afsk_mod_.modulate(nrzi_bits);
            }
            have_frame = true;
        } else if (!tx_queue_.empty() && !ofdm_kiss_probing_) {
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
                // Limit total payload based on speed level to cap air time.
                // Adaptive: grows from 3s toward 9s on success, halves on REJ.
                int level = gearshift_.current_level();
                // PHY bps already accounts for modulation and FEC
                int phy_bps = net_throughput(level, phy_config_.baud_rate);
                size_t max_batch = std::min((size_t)NATIVE_MAX_PAYLOAD,
                                             (size_t)(phy_bps * batch_airtime_s_ / 8));
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

                // OFDM-KISS batch compression: compress the assembled payload.
                // Prepend COMPRESSED_PAYLOAD_MAGIC so RX knows to decompress.
                // Skip if payload is B2F_DATA (already compressed internally).
                if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) &&
                    frame_data.size() > 20 &&
                    frame_data[0] != B2F_DATA_MAGIC) {  // Don't double-compress B2F_DATA
                    std::vector<uint8_t> comp_buf(frame_data.size() + COMPRESS_HEADER_SIZE + 256);
                    int comp_len = ofdm_kiss_tx_compressor_.compress_block(
                        frame_data.data(), (int)frame_data.size(),
                        comp_buf.data(), (int)comp_buf.size());
                    if (comp_len > 0 && comp_len + 1 < (int)frame_data.size()) {
                        IRIS_LOG("OFDM-KISS compress: %zu -> %d bytes (%.0f%%)",
                                 frame_data.size(), comp_len + 1,
                                 100.0 * (comp_len + 1) / frame_data.size());
                        frame_data.clear();
                        frame_data.reserve(1 + comp_len);
                        frame_data.push_back(COMPRESSED_PAYLOAD_MAGIC);
                        frame_data.insert(frame_data.end(),
                                          comp_buf.begin(), comp_buf.begin() + comp_len);
                    }
                }

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
                    // TX-without-ACK guard: if we've sent 6+ native frames with
                    // no peer ACK/REJ, the peer likely can't decode at this speed.
                    // Threshold 6 (not 3): on FM links, turnaround delays and
                    // KISS client processing can delay ACKs for several frames.
                    // Too-aggressive downshift causes unnecessary speed drops.
                    tx_no_ack_count_++;
                    if (tx_no_ack_count_ >= 6 && level > 0) {
                        gearshift_.report_failure();
                        level = gearshift_.current_level();
                        IRIS_LOG("Gearshift: no peer ACK for %d TX frames -> report_failure (level=%d)",
                                 tx_no_ack_count_, level);
                    }
                    // Peer SNR cap: if the peer reported its RX SNR (from our signal),
                    // don't TX at a speed level the peer can't decode.
                    // This handles asymmetric links where our RX SNR is high but
                    // the peer's RX SNR is lower (different TX power, antennas, etc).
                    if (peer_snr_db_ > 0 && level > 0) {
                        int peer_max = snr_to_speed_level(peer_snr_db_);
                        if (peer_max < level) {
                            IRIS_LOG("Gearshift: capping TX level %d -> %d (peer SNR %.1f dB)",
                                     level, peer_max, peer_snr_db_);
                            level = peer_max;
                        }
                    }
                    tx_mod = SPEED_LEVELS[level].modulation;
                    tx_fec_n = SPEED_LEVELS[level].fec_rate_num;
                    tx_fec_d = SPEED_LEVELS[level].fec_rate_den;
                    IRIS_LOG("[TX] speed=%s (level=%d) mod=%d fec=%d/%d, %zu bytes",
                             SPEED_LEVELS[level].name, level,
                             (int)tx_mod, tx_fec_n, tx_fec_d, frame_data.size());
                }
                tx_config.modulation = tx_mod;
                LdpcRate fec = fec_to_ldpc_rate(tx_fec_n, tx_fec_d);
                auto iq = build_native_frame(frame_data.data(), frame_data.size(), tx_config, fec);

                // Raise T1 floor to account for this frame's airtime.
                // IQ samples / 2 = audio samples (interleaved I/Q), / sample_rate = seconds.
                float frame_airtime_s = (float)(iq.size() / 2) / (float)config_.sample_rate;
                ax25_session_.set_t1_floor_for_airtime(frame_airtime_s);

                if (use_upconvert_) {
                    tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                    // TX pre-equalization: compensate for peer's RX de-emphasis
                    if (tx_channel_eq_.is_configured()) {
                        tx_channel_eq_.apply(tx_buffer_.data(), (int)tx_buffer_.size());
                        // Peak limiter: prevent FM overmodulation from EQ boost.
                        // Soft-clip at 0.95 to keep deviation within limits.
                        for (auto& s : tx_buffer_) {
                            if (s > 0.95f) s = 0.95f + 0.05f * std::tanh((s - 0.95f) / 0.05f);
                            else if (s < -0.95f) s = -0.95f + 0.05f * std::tanh((s + 0.95f) / 0.05f);
                        }
                    }
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
                    if (config_.ax25_baud == 9600)
                        g3ruh_tx_scrambler_.scramble(raw_bits);
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
            if (!have_frame) {
                have_frame = true;
                state_ = ModemState::TX_AX25;  // Ensure process_rx skips during probe tone TX
            }
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
        }
    }
    // *** tx_mutex_ released here ***

    // ptt_on() calls set_channel_busy() which acquires timer_mutex_.
    // Must be called OUTSIDE tx_mutex_ to prevent ABBA deadlock with
    // ax25_session_.tick() (holds timer_mutex_, then send_frame_ -> tx_mutex_).
    if (tx_pos_ == 0 && !tx_buffer_.empty()) {
        ptt_on();

        size_t to_copy = std::min((size_t)frame_count, tx_buffer_.size());
        std::memcpy(tx_audio, tx_buffer_.data(), to_copy * sizeof(float));
        tx_pos_ = to_copy;
        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        // Apply simulated channel effects
        if (sim_bp_enabled_) {
            for (int i = 0; i < frame_count; i++) {
                float s = tx_audio[i];
                for (int j = 0; j < 4; j++) s = sim_bp_hi_[j].process(s);
                for (int j = 0; j < 4; j++) s = sim_bp_lo_[j].process(s);
                tx_audio[i] = s;
            }
        }
        if (sim_deemph_enabled_) {
            for (int i = 0; i < frame_count; i++)
                tx_audio[i] = sim_deemph_.process(tx_audio[i]);
        }
        return;
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

        // Strict mode: hold data until post-quantum KX completes
        if (mlkem_kx_pending_ && config_.encryption_mode == 1) {
            // Buffer the original frame — will be flushed after hybrid rekey
            mlkem_held_frames_.insert(mlkem_held_frames_.end(), frame, frame + len);
            IRIS_LOG("[CRYPTO] Strict mode: holding %zu bytes until ML-KEM completes", len);
            return;
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

    // B2F proxy TX: sniff outgoing I-frame info fields through B2F handler.
    // During PAYLOAD_TRANSFER (local proposer): intercept LZHUF I-frames,
    // unroll via handler, compress plaintext, send as B2F_DATA over OFDM.
    // Line protocol I-frames (SID, FC, FS, FF, FQ) are forwarded normally.
    if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) &&
        ofdm_kiss_b2f_.is_initialized() && len > 16) {
        uint8_t ctrl = frame[14];
        bool is_iframe = (ctrl & 0x01) == 0;

        if (is_iframe) {
            const uint8_t* info = frame + 16;  // 14 addr + 1 ctrl + 1 pid
            int info_len = (int)(len - 16);

            // Check BEFORE feeding: is handler already in TX payload mode?
            // Transition happens in filter_rx (when FS arrives from remote),
            // so by the time we see LZHUF I-frames here, it's already set.
            bool intercepting = ofdm_kiss_b2f_.is_tx_payload_active();

            if (intercepting) {
                // Feed LZHUF data to filter_tx for unrolling
                std::vector<uint8_t> b2f_out(info_len * 2 + 4096);
                int out_len = ofdm_kiss_b2f_.filter_tx(
                    (const char*)info, info_len,
                    (char*)b2f_out.data(), (int)b2f_out.size());

                if (out_len > 0) {
                    // Unrolled plaintext ready (one proposal complete)
                    b2f_proxy_plaintext_.insert(b2f_proxy_plaintext_.end(),
                        b2f_out.begin(), b2f_out.begin() + out_len);
                }

                // Check if all proposals done
                if (!ofdm_kiss_b2f_.is_tx_payload_active()) {
                    b2f_proxy_active_ = false;
                    // Flush accumulated plaintext as B2F_DATA frame(s)
                    if (!b2f_proxy_plaintext_.empty()) {
                        IRIS_LOG("[B2F-PROXY] TX: unrolled %zu bytes plaintext, sending B2F_DATA",
                                 b2f_proxy_plaintext_.size());
                        // Segment into chunks that fit in NATIVE_MAX_PAYLOAD
                        // Each chunk: [0xCD][compressed_block]
                        const int CHUNK_SIZE = 3000;  // leaves room for compression overhead
                        size_t offset = 0;
                        while (offset < b2f_proxy_plaintext_.size()) {
                            size_t chunk = std::min((size_t)CHUNK_SIZE,
                                                     b2f_proxy_plaintext_.size() - offset);
                            // Compress this chunk
                            std::vector<uint8_t> comp(chunk + COMPRESS_HEADER_SIZE + 256);
                            int comp_len = ofdm_kiss_tx_compressor_.compress_block(
                                b2f_proxy_plaintext_.data() + offset, (int)chunk,
                                comp.data(), (int)comp.size());
                            // Build B2F_DATA frame
                            std::vector<uint8_t> b2f_frame;
                            b2f_frame.push_back(B2F_DATA_MAGIC);
                            if (comp_len > 0 && comp_len < (int)chunk) {
                                b2f_frame.insert(b2f_frame.end(), comp.begin(), comp.begin() + comp_len);
                                IRIS_LOG("[B2F-PROXY] TX: chunk %zu bytes -> %d compressed",
                                         chunk, comp_len);
                            } else {
                                // Incompressible — send raw with empty compression header
                                b2f_frame.insert(b2f_frame.end(),
                                    b2f_proxy_plaintext_.begin() + offset,
                                    b2f_proxy_plaintext_.begin() + offset + chunk);
                            }
                            tx_queue_.push(std::move(b2f_frame));
                            offset += chunk;
                        }
                        b2f_proxy_plaintext_.clear();
                    }
                }

                // Generate RR ACK back to local Winlink so it doesn't timeout.
                // Construct RR frame: swap src/dst, N(R) = N(S)+1
                if (b2f_proxy_addr_valid_ && len > 14) {
                    uint8_t ns = (ctrl >> 1) & 0x07;
                    b2f_proxy_vr_ = (ns + 1) & 0x07;
                    // Build RR response: [dst(7)][src(7)][ctrl_RR(1)]
                    // dst = original src (our Winlink), src = original dst (remote)
                    std::vector<uint8_t> rr(15);
                    memcpy(rr.data(), frame + 7, 7);      // dst = original src
                    memcpy(rr.data() + 7, frame, 7);      // src = original dst
                    // Fix address extension bits
                    rr[6] &= 0xFE;    // dst: clear end-of-address bit
                    rr[13] |= 0x01;   // src: set end-of-address bit
                    // RR S-frame control: (N(R) << 5) | (F << 4) | 0x01
                    bool poll = (ctrl & 0x10) != 0;
                    rr[14] = (b2f_proxy_vr_ << 5) | (poll ? 0x10 : 0) | 0x01;
                    if (rx_callback_)
                        rx_callback_(rr.data(), rr.size());
                }

                return;  // Don't forward this I-frame
            }

            // Not intercepting: feed for state tracking (line protocol)
            std::vector<uint8_t> b2f_out(info_len * 2 + 4096);
            ofdm_kiss_b2f_.filter_tx(
                (const char*)info, info_len,
                (char*)b2f_out.data(), (int)b2f_out.size());

            // Check if handler just transitioned to TX payload mode
            if (ofdm_kiss_b2f_.is_tx_payload_active() && !b2f_proxy_active_) {
                b2f_proxy_active_ = true;
                b2f_proxy_vr_ = 0;
                b2f_proxy_plaintext_.clear();
                memcpy(b2f_proxy_addr_, frame, 14);
                b2f_proxy_addr_valid_ = true;
                IRIS_LOG("[B2F-PROXY] TX interception started (local proposer payload)");
            }
            // Fall through: forward line protocol I-frame normally
        }

        // B2F proxy RX ACK suppression: during B2F_DATA reception,
        // Winlink sends RR ACKs for our injected I-frames.
        // Suppress them — they're not meaningful to the remote station.
        if (b2f_proxy_rx_active_) {
            uint8_t ctrl2 = frame[14];
            if ((ctrl2 & 0x03) == 0x01) {  // S-frame
                return;  // Suppress
            }
        }
    }

    std::vector<uint8_t> tx_frame(frame, frame + len);
    tx_queue_.push(std::move(tx_frame));
    if (gui_log_ && len >= 14)
        gui_log_("[TX] " + describe_ax25(frame, len));
}

void Modem::ax25_connect(const std::string& remote_callsign) {
    // Guard: reject if session already active (prevents SABM loop when AGW client
    // retries connect before processing the CONNECTED notification).
    if (ax25_session_.is_active()) {
        IRIS_LOG("AX25 connect to %s REJECTED — session already active (state=%d)",
                 remote_callsign.c_str(), (int)ax25_session_.state());
        return;
    }
    // Reset state for fresh connection
    peer_is_iris_ = false;
    ofdm_kiss_probe_done_ = false;

    // Connect immediately — probe starts after CONNECTED (state callback).
    // This lets the SABM/UA handshake complete first so Winlink doesn't timeout.
    IRIS_LOG("AX25 connect to %s", remote_callsign.c_str());
    ax25_session_.connect(remote_callsign);
}

void Modem::ax25_disconnect() {
    // Cancel probe if in progress
    if (ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0) {
        IRIS_LOG("AX25 disconnect: cancelling probe");
        ofdm_kiss_probing_ = false;
        ofdm_kiss_probe_cd_ = 0;
        probe_connect_timeout_ = 0;
        probe_start_pending_ = false;
        probe_.reset();
    }
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
    // Generate ephemeral X25519 keypair for DH key exchange
    generate_ephemeral_x25519();
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
    // Generate ephemeral X25519 keypair for DH key exchange
    generate_ephemeral_x25519();
    arq_.listen();
}

void Modem::generate_ephemeral_x25519() {
    if (config_.encryption_mode == 0) return;
    uint8_t pubkey[X25519_KEY_SIZE];
    if (cipher_.generate_x25519_keypair(pubkey) == 0) {
        arq_.set_local_x25519_pubkey(pubkey);
        IRIS_LOG("[CRYPTO] Generated ephemeral X25519 keypair");
    } else {
        IRIS_LOG("[CRYPTO] WARNING: X25519 keypair generation failed");
    }
}

void Modem::start_mlkem_exchange() {
    // Commander generates ML-KEM-768 keypair and sends encapsulation key
    uint8_t encaps_key[MLKEM_PK_SIZE];
    if (cipher_.generate_mlkem_keypair(encaps_key) != 0) {
        IRIS_LOG("[CRYPTO] ML-KEM keypair generation failed, staying X25519-only");
        mlkem_kx_pending_ = false;
        return;
    }
    // Send as ARQ data: [MLKEM_PK_MAGIC][encaps_key(1184)]
    std::vector<uint8_t> kx_frame;
    kx_frame.reserve(1 + MLKEM_PK_SIZE);
    kx_frame.push_back(MLKEM_PK_MAGIC);
    kx_frame.insert(kx_frame.end(), encaps_key, encaps_key + MLKEM_PK_SIZE);
    arq_.send_data(kx_frame.data(), kx_frame.size());
    IRIS_LOG("[CRYPTO] ML-KEM: sent encapsulation key (%d bytes)", MLKEM_PK_SIZE);
}

void Modem::handle_mlkem_frame(const uint8_t* data, size_t len) {
    if (data[0] == MLKEM_PK_MAGIC && len == 1 + MLKEM_PK_SIZE) {
        // Responder received encapsulation key — encapsulate and send ciphertext back
        IRIS_LOG("[CRYPTO] ML-KEM: received encapsulation key (%zu bytes)", len - 1);
        uint8_t ciphertext[MLKEM_CT_SIZE];
        if (cipher_.encapsulate_mlkem(data + 1, ciphertext) != 0) {
            IRIS_LOG("[CRYPTO] ML-KEM encapsulation failed, staying X25519-only");
            mlkem_kx_pending_ = false;
            return;
        }
        // Send ciphertext back
        std::vector<uint8_t> ct_frame;
        ct_frame.reserve(1 + MLKEM_CT_SIZE);
        ct_frame.push_back(MLKEM_CT_MAGIC);
        ct_frame.insert(ct_frame.end(), ciphertext, ciphertext + MLKEM_CT_SIZE);
        arq_.send_data(ct_frame.data(), ct_frame.size());
        IRIS_LOG("[CRYPTO] ML-KEM: sent ciphertext (%d bytes)", MLKEM_CT_SIZE);
        // Responder has both shared secrets now — rekey
        rekey_hybrid();
    } else if (data[0] == MLKEM_CT_MAGIC && len == 1 + MLKEM_CT_SIZE) {
        // Commander received ciphertext — decapsulate
        IRIS_LOG("[CRYPTO] ML-KEM: received ciphertext (%zu bytes)", len - 1);
        if (cipher_.decapsulate_mlkem(data + 1) != 0) {
            IRIS_LOG("[CRYPTO] ML-KEM decapsulation failed, staying X25519-only");
            mlkem_kx_pending_ = false;
            return;
        }
        // Commander has both shared secrets now — rekey
        rekey_hybrid();
    } else {
        IRIS_LOG("[CRYPTO] ML-KEM: unexpected frame (magic=0x%02X, len=%zu)", data[0], len);
    }
}

void Modem::rekey_hybrid() {
    // Re-derive session key with both X25519 + ML-KEM shared secrets
    std::vector<uint8_t> psk;
    for (size_t i = 0; i + 1 < config_.psk_hex.size(); i += 2) {
        char byte_str[3] = {config_.psk_hex[i], config_.psk_hex[i+1], 0};
        psk.push_back((uint8_t)strtol(byte_str, nullptr, 16));
    }
    cipher_.derive_session_key(config_.callsign.c_str(),
                                arq_.remote_callsign().c_str(),
                                psk.empty() ? nullptr : psk.data(),
                                (int)psk.size(), true);  // mlkem_done=true
    // Reset batch counters — both sides rekey at the same point
    tx_batch_counter_ = 0;
    rx_batch_counter_ = 0;
    mlkem_kx_pending_ = false;
    IRIS_LOG("[CRYPTO] HYBRID REKEY: X25519 + ML-KEM-768 (SNDL-proof)");
    if (gui_log_) gui_log_("[CRYPTO] Post-quantum encryption active");
}

// After this many AFSK SABM failures, escalate to native BPSK hailing
static constexpr int NATIVE_HAIL_ESCALATION_RETRIES = 3;

void Modem::tick() {
    arq_.tick();
    ax25_session_.tick();
    probe_.tick();

    // Deferred PROBE:START — queued from state callback (can't send there,
    // callback fires inside tx_mutex_ via queue_tx_frame → notify_outgoing).
    if (probe_start_pending_) {
        probe_start_pending_ = false;
        send_probe_start_ui();
        send_probe_start_ui();
    }

    // 30-second disconnect timeout: if stuck in AWAITING_RELEASE, force reset.
    // Winlink gets upset if we keep TX'ing DISC retries forever.
    if (disconnect_timeout_ticks_ > 0) {
        disconnect_timeout_ticks_--;
        if (disconnect_timeout_ticks_ == 0 &&
            ax25_session_.state() == Ax25SessionState::AWAITING_RELEASE) {
            IRIS_LOG("DISCONNECT TIMEOUT: stuck in AWAITING_RELEASE for 30s, forcing reset");
            if (gui_log_) gui_log_("Disconnect timeout — forcing reset");
            ax25_session_.reset();  // Force to DISCONNECTED (triggers state callback → PHY restore)
            ptt_off();
        }
    }

    // DCD diagnostics: log state every 5s during active session.
    dcd_diag_ticks_++;
    if (ax25_session_.is_active() && dcd_diag_ticks_ >= 100) {  // 100 ticks = 5s
        dcd_diag_ticks_ = 0;
        IRIS_LOG("[DCD] rms=%.4f inv=%d dcd_hold=%d csma_hold=%d ptt=%d",
                 rx_raw_rms_.load(), (int)dcd_inverted_,
                 dcd_holdoff_.load(), csma_holdoff_.load(), (int)ptt_active_);
    }

    // Probe timeout: if probe doesn't complete in 25s, give up and stay AFSK.
    // Connection is already established — just release held I-frames.
    if (probe_connect_timeout_ > 0) {
        probe_connect_timeout_--;
        if (probe_connect_timeout_ == 0) {
            ofdm_kiss_probing_ = false;
            ofdm_kiss_probe_cd_ = 0;
            probe_start_pending_ = false;
            probe_.reset();
            IRIS_LOG("Probe timeout: staying AFSK, releasing held I-frames");
            if (gui_log_) gui_log_("Probe timeout — staying AFSK");
        }
    }

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

    // Probe countdown (shared by manual and auto-probe initiator).
    // Responder is started directly by PROBE:START reception, not by countdown.
    // Resend PROBE:START every ~1s during countdown — the initial pair may be
    // lost if the responder is TX'ing (half-duplex collision).  Spreading
    // retransmissions over the 3s window gives 3 chances for delivery.
    if (ofdm_kiss_probe_cd_ > 0) {
        if (ofdm_kiss_probe_cd_ % 20 == 0 && !ofdm_kiss_probing_) {
            send_probe_start_ui();
            IRIS_LOG("[PROBE] Resending PROBE:START (countdown=%d)", ofdm_kiss_probe_cd_);
        }
        ofdm_kiss_probe_cd_--;
        if (ofdm_kiss_probe_cd_ == 0) {
            if (probe_manual_) {
                // Manual probe button: always initiator.
                probe_.start_initiator(config_.sample_rate, 12.0f);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("[PROBE] Manual probe: sending tones now (12s capture)");
            } else {
                // Auto-probe initiator: 12s capture. Responder needs 6s capture +
                // ~0.5s analysis + ~3s TX (result + 2.25s probe). Total ~9.5s.
                // 12s provides margin for radio latency.
                probe_.start_initiator(config_.sample_rate, 12.0f);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("OFDM-KISS probe: initiator sending tones (12s capture)");
            }
        }
    }

    // OFDM-KISS probe completion: apply discovered passband and baud rate.
    // Both sides run probe_negotiate() on the same two ProbeResults, so the
    // negotiated band is deterministic — no protocol exchange needed.
    if (ofdm_kiss_probing_ && probe_.is_done()) {
        // NOTE: ofdm_kiss_probing_ stays true until the end of this block.
        // The TX gate in process_tx checks ofdm_kiss_probing_ — keeping it
        // true prevents process_tx from draining ax25_tx_queue_ as AFSK
        // before migration moves I-frames to the native tx_queue_.
        ofdm_kiss_probe_done_ = true;
        // Clear DCD holdoff accumulated from probe tones so TX resumes immediately
        dcd_holdoff_ = 0;

        if (probe_.has_results() && probe_.negotiated().valid && !probe_manual_) {
            // Auto-probe: apply negotiated band + baud rate
            float low = probe_.negotiated().low_hz;
            float high = probe_.negotiated().high_hz;
            float bandwidth = high - low;
            config_.band_low_hz = low;
            config_.band_high_hz = high;
            float center = (low + high) / 2.0f;

            if (use_upconvert_) {
                upconverter_ = Upconverter(center, config_.sample_rate);
                downconverter_ = Downconverter(center, config_.sample_rate);

                // SPS sweep: highest baud rate that fits in discovered passband.
                // Both sides compute the same answer from the same negotiated band.
                constexpr float MAX_OCCUPIED_BW_HZ = 20000.0f;
                float usable_bw = std::min(bandwidth - 200.0f, MAX_OCCUPIED_BW_HZ);
                constexpr int SPS_MIN = 6;
                constexpr int SPS_MAX = 80;
                int new_sps = -1;
                int new_baud = 0;
                for (int sps = SPS_MIN; sps <= SPS_MAX; sps++) {
                    int baud = config_.sample_rate / sps;
                    float sig_bw = baud * (1.0f + phy_config_.rrc_alpha);
                    if (sig_bw <= usable_bw) {
                        new_sps = sps;
                        new_baud = baud;
                        break;
                    }
                }
                if (new_sps < 0) {
                    IRIS_LOG("WARNING: no valid SPS for usable BW %.0f Hz — keeping default SPS=%d baud=%d",
                             usable_bw, phy_config_.samples_per_symbol, phy_config_.baud_rate);
                    new_sps = phy_config_.samples_per_symbol;
                    new_baud = phy_config_.baud_rate;
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

            // Configure channel equalization from probe tone power data.
            // RX EQ: flatten what we receive (their TX → our RX channel response)
            // TX EQ: pre-compensate what we send (our TX → their RX channel response)
            rx_channel_eq_.configure(probe_.their_tx_result(), probe_.negotiated(), config_.sample_rate, 3.0f);
            tx_channel_eq_.configure(probe_.my_tx_result(), probe_.negotiated(), config_.sample_rate, 6.0f);
            if (rx_channel_eq_.is_configured())
                IRIS_LOG("Probe: RX channel EQ active (%d taps)", (int)rx_channel_eq_.taps().size());
            if (tx_channel_eq_.is_configured())
                IRIS_LOG("Probe: TX channel EQ active (%d taps)", (int)tx_channel_eq_.taps().size());

            IRIS_LOG("Probe complete: band %.0f-%.0f Hz (%.0f Hz BW), center %.0f Hz, baud %d",
                     low, high, bandwidth, center, phy_config_.baud_rate);
            if (gui_log_) {
                char buf[128];
                snprintf(buf, sizeof(buf), "Probe: %.0f-%.0f Hz (%.0f Hz BW) baud %d",
                         low, high, bandwidth, phy_config_.baud_rate);
                gui_log_(buf);
            }

            // Probe confirms peer is Iris — enable native mode
            peer_is_iris_ = true;
            ofdm_kiss_ = true;
            ofdm_kiss_tx_ = true;
            ax25_session_.set_native_active(true);  // Enable T1 polls in native mode
            IRIS_LOG("OFDM-KISS: native mode active (probe-first)");
            if (gui_log_) gui_log_("OFDM-KISS: native mode active");

            // Speed level cache: start at cached level for this peer (if available)
            {
                int cached = gearshift_.load_cached_level(ax25_session_.remote_callsign());
                if (cached > 0) {
                    gearshift_.force_level(cached);
                    IRIS_LOG("Gearshift: cached level %d for %s",
                             cached, ax25_session_.remote_callsign().c_str());
                }
            }

            // Negotiate OFDM-KISS capabilities from probe result exchange.
            // Each side embeds its local caps in the probe result message.
            // my_tx_result = what THEY reported about OUR probe (contains THEIR caps).
            // their_tx_result = what WE measured from THEIR probe (contains OUR caps).
            {
                uint16_t peer_caps = probe_.my_tx_result().capabilities;
                ofdm_kiss_peer_caps_ = local_cap_.capabilities & peer_caps;
                IRIS_LOG("OFDM-KISS caps: local=0x%04X peer=0x%04X negotiated=0x%04X",
                         local_cap_.capabilities, peer_caps, ofdm_kiss_peer_caps_);
                if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
                    ofdm_kiss_tx_compressor_.init();
                    ofdm_kiss_rx_compressor_.init();
                    IRIS_LOG("OFDM-KISS: compression enabled (per-block, no streaming)");
                }
                if (ofdm_kiss_peer_caps_ & CAP_B2F_UNROLL) {
                    ofdm_kiss_b2f_.init();
                    b2f_proxy_plaintext_.reserve(B2F_BUFFER_SIZE);
                    IRIS_LOG("OFDM-KISS: B2F unroll/reroll enabled");

                    // Replay buffered AFSK I-frame info fields so the B2F
                    // handler sees the SID/FC/FS exchange that happened before
                    // OFDM-KISS activated.  Without this, the handler stays in
                    // B2F_IDLE and never enters PAYLOAD_TRANSFER.
                    if (!b2f_afsk_tx_history_.empty() || !b2f_afsk_rx_history_.empty()) {
                        std::vector<uint8_t> scratch(16384);
                        for (auto& info : b2f_afsk_tx_history_) {
                            ofdm_kiss_b2f_.filter_tx(
                                (const char*)info.data(), (int)info.size(),
                                (char*)scratch.data(), (int)scratch.size());
                        }
                        for (auto& info : b2f_afsk_rx_history_) {
                            ofdm_kiss_b2f_.filter_rx(
                                (const char*)info.data(), (int)info.size(),
                                (char*)scratch.data(), (int)scratch.size());
                        }
                        IRIS_LOG("OFDM-KISS: B2F replayed %zu TX + %zu RX AFSK I-frames",
                                 b2f_afsk_tx_history_.size(), b2f_afsk_rx_history_.size());
                        b2f_afsk_tx_history_.clear();
                        b2f_afsk_rx_history_.clear();

                        // Safety: if payload transfer already started during AFSK,
                        // some LZHUF bytes were sent as-is.  Can't partially unroll
                        // (remote would get LZHUF + plaintext mix = corrupt).
                        if (ofdm_kiss_b2f_.is_payload_transfer()) {
                            IRIS_LOG("OFDM-KISS: B2F payload already in-flight from AFSK — "
                                     "disabling unroll for this session");
                            ofdm_kiss_b2f_.deinit();
                            ofdm_kiss_peer_caps_ &= ~CAP_B2F_UNROLL;
                        }
                    }
                }
            }

            // Migrate I-frames from ax25_tx_queue_ (AFSK) to tx_queue_ (native).
            // During the probe, ofdm_kiss_tx_ was false so the send_frame_
            // callback routed I-frames to ax25_tx_queue_.  Now that native mode
            // is active, move them so they go out as native frames — not as a
            // long AFSK burst that would bury the probe result UI frame.
            {
                std::lock_guard<std::mutex> lock(tx_mutex_);
                int migrated = 0;
                std::queue<std::vector<uint8_t>> keep;
                while (!ax25_tx_queue_.empty()) {
                    auto frame = std::move(ax25_tx_queue_.front());
                    ax25_tx_queue_.pop();
                    bool is_iframe = frame.size() > 14 && (frame[14] & 0x01) == 0;
                    if (is_iframe) {
                        tx_queue_.push(std::move(frame));
                        migrated++;
                    } else {
                        keep.push(std::move(frame));
                    }
                }
                ax25_tx_queue_ = std::move(keep);
                if (migrated > 0) {
                    IRIS_LOG("OFDM-KISS: migrated %d I-frames from AFSK to native queue", migrated);
                }
            }

            // Mandatory listen window after probe: don't TX immediately.
            // The peer may be sending its response (UA, held frames) right now.
            // Without this, our TX triggers a self-hear guard that discards the
            // peer's native frame before we can decode it.
            csma_holdoff_ = config_.sample_rate;  // 1s listen before first TX

            // Connection already established before probe.
            // Held I-frames release via native after listen window expires.
            ax25_session_.set_t1_ticks(300);
            probe_connect_timeout_ = 0;
            IRIS_LOG("Probe done — releasing held I-frames via native");
        } else if (probe_.has_results() && probe_.negotiated().valid) {
            // Manual probe: log only, no PHY change
            float low = probe_.negotiated().low_hz;
            float high = probe_.negotiated().high_hz;
            float bandwidth = high - low;
            IRIS_LOG("Probe complete (manual): band %.0f-%.0f Hz (%.0f Hz BW), no PHY change",
                     low, high, bandwidth);
            if (gui_log_) {
                char buf[128];
                snprintf(buf, sizeof(buf), "Probe: %.0f-%.0f Hz (%.0f Hz BW)", low, high, bandwidth);
                gui_log_(buf);
            }
        } else {
            IRIS_LOG("Probe complete: no valid results — staying AX.25");
            if (gui_log_) gui_log_("Probe: no valid results");
            probe_connect_timeout_ = 0;
            // Connection already established — I-frames release automatically
            // (ofdm_kiss_probing_ goes false at end of block, TX gate opens)
        }
        // NOW release the TX gate: ofdm_kiss_probing_=false allows process_tx
        // to resume.  Migration (if applicable) already moved I-frames to
        // tx_queue_, so they'll go out as native instead of AFSK.
        ofdm_kiss_probing_ = false;
        probe_manual_ = false;
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
        if (!ofdm_kiss_probing_ && config_.dcd_threshold > 0 && (native_mode_ || ofdm_kiss_)) {
            if (dcd_inverted_)
                busy = rx_raw_rms_ < dcd_baseline_rms_ * 0.5f;
            else
                busy = rx_raw_rms_ > config_.dcd_threshold;
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
    diag.spectrum_low_hz = spectrum_low_hz_;
    diag.spectrum_high_hz = spectrum_high_hz_;

    return diag;
}

void Modem::compute_spectrum(const float* audio, int count) {
    // Downsample 48k→8k then FFT. At 8 kHz, NFFT=512 gives:
    //   15.6 Hz bins, 256 bins covering 0-4 kHz. DFT is 256×512 = 131k ops.
    constexpr int DS_RATE = 8000;
    constexpr int NFFT = 512;

    int ratio = config_.sample_rate / DS_RATE;
    int need = NFFT * ratio;  // 3072 samples at 48k

    // Accumulate raw audio
    if ((int)spectrum_buf_.size() < spectrum_buf_pos_ + count)
        spectrum_buf_.resize(spectrum_buf_pos_ + count);
    memcpy(spectrum_buf_.data() + spectrum_buf_pos_, audio, count * sizeof(float));
    spectrum_buf_pos_ += count;

    if (spectrum_buf_pos_ < need) return;

    // Decimate: pick every Nth sample (radio audio is already bandlimited)
    float ds[NFFT];
    const float* src = spectrum_buf_.data() + spectrum_buf_pos_ - need;
    for (int i = 0; i < NFFT; i++)
        ds[i] = src[i * ratio];

    spectrum_buf_pos_ = 0;

    // DFT — all 256 positive bins cover 0-4 kHz
    int n_pos = NFFT / 2;
    std::vector<float> spec(n_pos, 0.0f);
    for (int k = 0; k < n_pos; k++) {
        float re = 0, im = 0;
        for (int n = 0; n < NFFT; n++) {
            float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * n / (NFFT - 1)));
            float angle = -2.0f * (float)M_PI * k * n / NFFT;
            re += ds[n] * w * std::cos(angle);
            im += ds[n] * w * std::sin(angle);
        }
        float pwr = (re * re + im * im) / (NFFT * NFFT);
        spec[k] = 10.0f * std::log10(std::max(pwr, 1e-12f));
    }

    std::lock_guard<std::mutex> lock(diag_mutex_);
    last_spectrum_ = std::move(spec);
    spectrum_low_hz_ = 0;
    spectrum_high_hz_ = DS_RATE / 2.0f;
}

// --- Calibration ---
// Protocol:
//   Initiator: clicks Auto Cal → sends CAL:START UI frame → tone 1s → WAIT_REPORT
//   Responder: auto-detects CAL:START → measures tone RMS → sends CAL:RMS=X.XXXX
//   Initiator: receives report → adjusts TX level → DONE

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

void Modem::start_probe() {
    if (ofdm_kiss_probing_ || ofdm_kiss_probe_cd_ > 0) {
        IRIS_LOG("[PROBE] Already probing, ignoring request");
        return;
    }
    IRIS_LOG("[PROBE] Manual probe: sending PROBE:START, tones in 3s");
    if (gui_log_) gui_log_("Probe: signaling remote, tones in 3s...");
    ofdm_kiss_probe_done_ = false;
    probe_manual_ = true;
    // Send PROBE:START twice for reliability on noisy channel
    send_probe_start_ui();
    send_probe_start_ui();
    // Countdown: 60 ticks = 3s at 50ms/tick.  Gives remote time to decode
    // PROBE:START and start listening before we send tones.
    ofdm_kiss_probe_cd_ = 60;
}

void Modem::send_probe_start_ui() {
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("PROBE");
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    const char* payload = "PROBE:START";
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + 11);
    std::lock_guard<std::mutex> lock(tx_mutex_);
    ax25_tx_queue_.push(std::move(frame));
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
        if (config_.ax25_baud == 9600)
            g3ruh_rx_scrambler_.descramble(rx_bits);
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
