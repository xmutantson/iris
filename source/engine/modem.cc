#include "engine/modem.h"
#include "ax25/ax25_protocol.h"
#include "native/frame.h"
#include "fec/ldpc.h"
#include "common/fft.h"
#include "common/logging.h"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono>
#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Binary TUNE report: embedded in OFDM ramp frame payload.
// Format: [0xBB] [count:u8] [entry × count]
// Entry:  [index:u8] [iters:i8] [H_hi:u8] [H_lo:u8] [snr_i8]
//   iters: -2=preamble-only, -1=not measured, 0-50=LDPC iters
//   H: unsigned 16-bit fixed-point, H * 100 (range 0-655.35)
//   snr: signed 8-bit, SNR_dB + 30 (range -30 to +97 dB, 0.5 dB res not needed)
static std::vector<uint8_t> tune_build_binary_report(
    const int* iters, const float* H, const float* snr, int count) {
    std::vector<uint8_t> buf;
    buf.push_back(TUNE_REPORT_MAGIC);
    int n = 0;
    for (int i = 0; i < count; i++)
        if (iters[i] != -1) n++;
    buf.push_back((uint8_t)n);
    for (int i = 0; i < count; i++) {
        if (iters[i] == -1) continue;
        buf.push_back((uint8_t)i);              // frame index
        buf.push_back((uint8_t)(int8_t)iters[i]); // iters as signed byte
        uint16_t h16 = (uint16_t)std::min(65535.0f, H[i] * 100.0f);
        buf.push_back((uint8_t)(h16 >> 8));
        buf.push_back((uint8_t)(h16 & 0xFF));
        int8_t s = (int8_t)std::clamp((int)(snr[i] + 30.0f), 0, 127);
        buf.push_back((uint8_t)s);
    }
    return buf;
}

static bool tune_parse_binary_report(const uint8_t* data, size_t len,
    int* out_iters, float* out_H, float* out_snr, int max_entries) {
    if (len < 2 || data[0] != TUNE_REPORT_MAGIC) return false;
    int n = data[1];
    size_t pos = 2;
    for (int i = 0; i < n && pos + 5 <= len; i++) {
        int idx = data[pos];
        int8_t it = (int8_t)data[pos + 1];
        uint16_t h16 = ((uint16_t)data[pos + 2] << 8) | data[pos + 3];
        int8_t s = (int8_t)data[pos + 4];
        pos += 5;
        if (idx < max_entries) {
            out_iters[idx] = it;
            out_H[idx] = h16 / 100.0f;
            out_snr[idx] = s - 30.0f;
        }
    }
    return n > 0;
}

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
    local_cap_.capabilities = CAP_MODE_A | CAP_COMPRESSION | CAP_STREAMING | CAP_HARQ;
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
    if (config_.ofdm_enable)
        local_cap_.capabilities |= CAP_OFDM;
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
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        probe_audio_pending_.insert(probe_audio_pending_.end(), audio, audio + count);
    };
    probe_.on_send_msg = [this](const uint8_t* data, size_t len) {
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
        constexpr size_t TX_QUEUE_MAX = 32;
        if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
            IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
            ax25_tx_queue_.pop();
        }
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
    // Advertise our preferred OFDM PHY parameters in probe result
    {
        uint8_t nfft_code = 2;  // 0=512, 1=256, 2=1024
        if (config_.ofdm_nfft == 512) nfft_code = 0;
        else if (config_.ofdm_nfft == 256) nfft_code = 1;
        probe_.set_local_ofdm_config(
            (uint8_t)config_.ofdm_cp_samples,  // e.g. 64
            4,   // pilot_carrier_spacing
            24,  // pilot_symbol_spacing
            nfft_code);
    }
    ArqCallbacks arq_cb;
    arq_cb.send_frame = [this](const uint8_t* data, size_t len) {
        // ARQ frames go directly to TX queue (not back through ARQ)
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
        constexpr size_t TX_QUEUE_MAX = 32;
        if (tx_queue_.size() >= TX_QUEUE_MAX) {
            IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
            tx_queue_.pop();
        }
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
        std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
            constexpr size_t TX_QUEUE_MAX = 32;
            if (tx_queue_.size() >= TX_QUEUE_MAX) {
                IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                tx_queue_.pop();
            }
            tx_queue_.push(std::move(frame));
        } else {
            constexpr size_t TX_QUEUE_MAX = 32;
            if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
                IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
                ax25_tx_queue_.pop();
            }
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
            // Check probe cache first — skip re-probing within 24h
            if (try_use_cached_probe(remote)) {
                IRIS_LOG("Probe cache hit for %s — skipping probe", remote.c_str());
                if (gui_log_) gui_log_("Cached probe for " + remote);
            } else {
                // Defer PROBE:START — this callback may fire from queue_tx_frame().
                // Set flag and let tick() send it.
                xid_peer_call_ = remote;
                probe_start_pending_ = true;
                ofdm_kiss_probe_cd_ = 60;  // 3s countdown before tones (3 PROBE:START sends)
                probe_connect_timeout_ = 700;  // 35s overall timeout (25s initiator + ~10s responder + analysis)
                IRIS_LOG("Probe-after-connect: probing %s, I-frames held until done", remote.c_str());
                if (gui_log_) gui_log_("Probing " + remote + "...");
            }
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
            native_rx_gain_ = 1.0f;
            ofdm_phy_active_ = false;
            ofdm_chase_llrs_.clear();
            ofdm_chase_combines_ = 0;
            ofdm_rx_audio_buf_.clear();
            ofdm_rx_lpf_.reset();
            ofdm_rx_lpf_active_ = false;
            ofdm_sync_cached_ = false;
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
                ofdm_phy_active_ = false;
                ofdm_mod_.reset();
                ofdm_demod_.reset();
                ofdm_rx_iq_.clear();
                ofdm_rx_audio_buf_.clear();
                ofdm_rx_lpf_.reset();
                ofdm_rx_lpf_active_ = false;
                ofdm_sync_cached_ = false;
                ofdm_chase_llrs_.clear();
                ofdm_chase_combines_ = 0;
                ofdm_txdelay_ms_ = 0;  // Reset adaptive TXDELAY on disconnect
                batch_airtime_s_ = BATCH_AIRTIME_MIN;
                tx_no_ack_count_ = 0;
                last_peer_nr_ = 0xFF;
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    arq_.reset();
    ptt_off();
    state_ = ModemState::IDLE;
    native_mod_.reset();
    native_demod_.reset();
    if (kalman_log_file_) {
        fclose(kalman_log_file_);
        kalman_log_file_ = nullptr;
    }
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

        // Post-TX listen window: defer next TX to let peer respond.
        // FM: turnaround is fast (~150ms PTT relay + radio switching).
        // OFDM sync (ZC preamble) handles frame boundaries — no need for
        // HF-style long listen windows.
        if (!loopback_mode_ && ax25_session_.is_active()) {
            static thread_local std::minstd_rand rng(std::random_device{}());
            int base, jitter;
            if (ofdm_kiss_tx_) {
                // FM OFDM: ~150ms radio turnaround + small margin
                jitter = rng() % (config_.sample_rate / 16);  // 0-62ms
                base = ax25_session_.we_initiated()
                    ? config_.sample_rate / 5           // initiator: 200ms
                    : config_.sample_rate / 6;          // responder: 167ms
            } else {
                jitter = rng() % (config_.sample_rate / 4);  // 0-250ms
                base = ax25_session_.we_initiated()
                    ? config_.sample_rate               // initiator: 1.0s
                    : config_.sample_rate * 3 / 4;      // responder: 0.75s
            }
            csma_holdoff_ = std::max(csma_holdoff_.load(), base + jitter);
        }

        // Resume AX.25 timers now that channel is free
        ax25_session_.set_channel_busy(false);
    }
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
            ofdm_rx_audio_buf_.clear();  // Clear stale audio from TX period
            ofdm_rx_lpf_.reset();       // Reset filter state (no transient from stale history)
            ofdm_sync_cached_ = false;
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

    // Pre-allocated RX audio buffer (avoids per-callback heap allocation)
    if (rx_audio_tmp_.size() < (size_t)frame_count)
        rx_audio_tmp_.resize(frame_count);
    std::copy(rx_audio, rx_audio + frame_count, rx_audio_tmp_.data());
    float* audio = rx_audio_tmp_.data();
    // AGC for AX.25 mode only — native mode has preamble-based gain estimation
    // AGC would distort QAM symbols within a frame (gain changes during preamble vs payload)
    if (!native_mode_)
        agc_.process_block(audio, frame_count);

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
        compute_spectrum(audio, frame_count);

    if (state_ == ModemState::CALIBRATING) {
        process_calibration_rx(audio, frame_count);
        return;
    }

    if (native_mode_) {
        process_rx_native(audio, frame_count);
    } else {
        process_rx_ax25(audio, frame_count);

        // Always run native demod alongside AFSK — it's lightweight (just
        // preamble correlation until a frame arrives) and lets us detect native
        // frames at any point without waiting for the handshake to complete.
        // Self-hear is handled by native_selfhear_guard_.
        if (!config_.ax25_only && native_demod_) {
            process_rx_native(audio, frame_count);
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
    // Resets on each frame, so holdoff extends past the LAST frame in a burst.
    //
    // Native data I-frames get a longer holdoff on the responder side:
    // the sender's self-hear guard (300ms) plus FM turnaround (~200ms) means
    // a premature RR arriving within ~500ms of sender's TX end gets discarded.
    // 800ms covers the inter-frame gap in a multi-frame burst and ensures
    // the RR only goes out after the burst is truly done.
    {
        int rx_holdoff;
        bool native_data = from_ofdm && frame.size() > 15 &&
                           (frame[14] & 0x01) == 0;  // I-frame over native
        if (ax25_session_.is_active()) {
            if (ofdm_kiss_tx_) {
                // FM OFDM: minimal post-RX holdoff. ZC preamble detection
                // handles burst boundaries — timing-based holdoff just wastes air time.
                // Only need enough for radio TX/RX turnaround (~150ms).
                rx_holdoff = config_.sample_rate / 6;  // 167ms for all roles
            } else if (native_data && !ax25_session_.we_initiated()) {
                // Responder receiving native I-frame: burst holdoff
                // Self-hear guard (300ms) + FM turnaround (~200ms) + margin
                rx_holdoff = config_.sample_rate * 4 / 5;  // 800ms
            } else {
                rx_holdoff = ax25_session_.we_initiated()
                    ? config_.sample_rate * 4 / 5   // initiator: 800ms
                    : config_.sample_rate * 2 / 5;  // responder: 400ms (ACK quickly)
            }
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
                // Only count as real ACK if N(R) advanced (new data acknowledged).
                // Poll responses with unchanged N(R) mean the peer is alive but
                // NOT decoding our data — don't reset the no-ack counter, so the
                // gearshift eventually downshifts on asymmetric decode failures.
                uint8_t nr = (ctrl >> 5) & 0x07;
                if (nr != last_peer_nr_) {
                    float prev = batch_airtime_s_;
                    batch_airtime_s_ = std::min(batch_airtime_s_ + 1.0f, BATCH_AIRTIME_MAX);
                    if (batch_airtime_s_ != prev)
                        IRIS_LOG("Batch airtime: %.0fs -> %.0fs (RR ACK)", prev, batch_airtime_s_);
                    tx_no_ack_count_ = 0;  // peer acknowledged NEW data
                    last_peer_nr_ = nr;
                    // Cache proven speed level for this peer
                    gearshift_.save_cached_level(ax25_session_.remote_callsign());
                }
                // else: RR with same N(R) = poll response, don't reset no-ack counter
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
        } else if ((ctrl & 0x01) == 0x00) {  // I-frame
            // I-frames carry N(R) that implicitly acknowledges our data.
            // The S-frame sniff above misses this — if the peer sends data
            // instead of explicit RR, we still need to track their N(R).
            uint8_t nr = (ctrl >> 5) & 0x07;
            if (nr != last_peer_nr_) {
                tx_no_ack_count_ = 0;  // peer acknowledged via I-frame N(R)
                last_peer_nr_ = nr;
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
            // TUNE: UI frames (auto-tune gain calibration)
            if (!is_cal && info_len >= 5 && info[0] == 'T' && info[1] == 'U' &&
                info[2] == 'N' && info[3] == 'E' && info[4] == ':') {
                handle_tune_frame(info, info_len);
                is_cal = true;  // reuse flag to skip further processing
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
                    // is still listening (initiator has 25s capture).
                    // 15s: catches tones arriving at t+3..5 with margin,
                    // finishes by t+15, sends reply by t+18 — well within
                    // initiator's 25s window.
                    probe_.start_responder(config_.sample_rate, 15.0f);
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

    // ============ OFDM PHY RX path ============
    // OFDM bypasses the downconverter — processes raw audio directly.
    // Skip downconvert + rx_overlap_buf when OFDM is active (saves CPU).
    // OFDM uses real passband audio directly (Hermitian-symmetric IFFT),
    // bypassing the downconverter. Buffer raw audio separately.
    if (ofdm_phy_active_ && ofdm_demod_) {
        // Skip OFDM RX during TX — our own signal bleeds back and creates
        // false SC/ZC triggers that waste CPU and corrupt Chase combining.
        if (ptt_active_ || rx_muted_) {
            return;
        }

        // OFDM bypasses the probe-based channel EQ entirely. The 127-tap FIR
        // has a 63-sample group delay that can exceed the OFDM CP (64 samples
        // default), risking ISI. OFDM handles per-carrier equalization
        // internally via training symbols and block pilots.
        ofdm_rx_audio_buf_.insert(ofdm_rx_audio_buf_.end(),
                                   audio, audio + count);
        // Apply LPF to remove f² discriminator noise from flat radio ports.
        // Runs on newly inserted samples only.
        {
            size_t start = ofdm_rx_audio_buf_.size() - count;
            if (ofdm_rx_lpf_active_) {
                for (size_t i = start; i < ofdm_rx_audio_buf_.size(); i++)
                    ofdm_rx_audio_buf_[i] = ofdm_rx_lpf_.process(ofdm_rx_audio_buf_[i]);
            }
            // Apply RX gain correction
            if (native_rx_gain_ != 1.0f) {
                for (size_t i = start; i < ofdm_rx_audio_buf_.size(); i++)
                    ofdm_rx_audio_buf_[i] *= native_rx_gain_;
            }
        }
        // Trim to max buffer size
        // Worst case: O0 BPSK r1/2, 4 codewords, 22 carriers = 365 symbols × 640 = 233600 samples (4.87s).
        // Add margin for sync search window. 6 seconds covers all configurations.
        constexpr size_t OFDM_RX_BUF_MAX = 48000 * 6;
        if (ofdm_rx_audio_buf_.size() > OFDM_RX_BUF_MAX) {
            size_t excess = ofdm_rx_audio_buf_.size() - OFDM_RX_BUF_MAX;
            ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                      ofdm_rx_audio_buf_.begin() + excess);
            ofdm_sync_cached_ = false;
            IRIS_LOG("[OFDM-RX] buffer overflow: trimmed %zu samples", excess);
        }

        size_t n_samples = ofdm_rx_audio_buf_.size();
        // Need enough samples for a minimum OFDM frame:
        // 2 training + 1 sync word + 1 data + 1 tail = 5 symbols.
        size_t sym_len = (size_t)(ofdm_config_.cp_samples + ofdm_config_.nfft);
        size_t min_samples = sym_len * 5;  // 2 preamble + 1 sync + 1 data + 1 tail
        if (n_samples < min_samples) return;

        // Convert real audio to analytic signal via Hilbert transform.
        // This enables Schmidl-Cox to extract CFO phase (real-only signals
        // produce real-valued correlation, losing the sign of freq offset).
        // Method: FFT, zero negative freqs, double positive freqs, IFFT.
        ofdm_rx_iq_.resize(n_samples);
        {
            // Find next power of 2 for FFT
            int nfft_h = 1;
            while (nfft_h < (int)n_samples) nfft_h <<= 1;

            // Copy real audio into complex buffer, zero-pad
            std::vector<std::complex<float>> hbuf(nfft_h, {0.0f, 0.0f});
            for (size_t i = 0; i < n_samples; i++)
                hbuf[i] = std::complex<float>(ofdm_rx_audio_buf_[i], 0.0f);

            fft_complex(hbuf.data(), nfft_h);

            // Keep DC (bin 0) and Nyquist (bin N/2) as-is.
            // Double positive frequencies (bins 1..N/2-1).
            // Zero negative frequencies (bins N/2+1..N-1).
            for (int k = 1; k < nfft_h / 2; k++)
                hbuf[k] *= 2.0f;
            for (int k = nfft_h / 2 + 1; k < nfft_h; k++)
                hbuf[k] = {0.0f, 0.0f};

            ifft_complex(hbuf.data(), nfft_h);

            for (size_t i = 0; i < n_samples; i++)
                ofdm_rx_iq_[i] = hbuf[i];
        }

        // Use cached sync if available (avoids re-detecting same preamble
        // while waiting for data symbols to accumulate in the buffer).
        OfdmSyncResult sync;
        if (ofdm_sync_cached_) {
            sync = ofdm_pending_sync_;
        } else {
            // Lower FD-ZC threshold during TUNE to detect weak ramp frames.
            // FD-ZC threshold: 0.30 for data, 0.15 during TUNE.
            // CRC-8 sync word is the real false-positive filter.
            // During TUNE, extreme TX levels degrade ZC further — use 0.15.
            // Normal operation: 0.30 catches obvious noise without rejecting
            // genuine frames (OTA scores 0.43-0.49 on FM).
            {
                bool in_tune = (tune_state_ != TuneState::IDLE &&
                                tune_state_ != TuneState::DONE);
                ofdm_config_.fd_zc_threshold = in_tune ? 0.15f : 0.30f;
            }
            sync = ofdm_detect_frame(ofdm_rx_iq_.data(), (int)n_samples, ofdm_config_);
            if (!sync.detected) {
                // If no detection and buffer is getting large, trim old searched samples.
                // Keep last 4 symbols of margin in case a preamble straddles the trim boundary.
                if (ofdm_rx_audio_buf_.size() > (size_t)(sym_len * 8)) {
                    size_t keep = sym_len * 4;
                    size_t trim = ofdm_rx_audio_buf_.size() - keep;
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + trim);
                    ofdm_sync_cached_ = false;
                }
                return;
            }
            ofdm_redetect_count_ = 0;
        }

        // Self-hear guard: radio plays back TX tones on its audio output.
        // Must clear the ENTIRE buffer — a partial skip (just the preamble)
        // leaves data symbols + subsequent frame preambles intact, which get
        // decoded after the 300ms guard timer expires.
        if (native_selfhear_guard_ > 0) {
            IRIS_LOG("[OFDM-RX] frame detected but DISCARDED (self-hear guard, clearing %zu samples)",
                     ofdm_rx_audio_buf_.size());
            ofdm_rx_audio_buf_.clear();
            ofdm_rx_lpf_.reset();
            ofdm_sync_cached_ = false;
            return;
        }

        // ---- Frame-length gate: wait until full frame is buffered ----
        // Prevents O(N²) redundant re-demodulation. Previously, demodulate()
        // was called every 10ms callback and re-ran training FFTs, channel est,
        // CFO correction, and all data symbols from scratch each time. For a
        // 34-symbol frame this meant symbol 0 demodulated 34 times, etc.
        // Now we compute the expected frame length and only call demodulate()
        // once when all samples are available.
        {
            int bps = ofdm_tone_map_.total_bits_per_symbol;
            int n_cw = std::max(1, ofdm_tone_map_.n_codewords);
            int coded_bits = (ofdm_tone_map_.fec_rate != LdpcRate::NONE)
                ? n_cw * LdpcCodec::codeword_size(ofdm_tone_map_.fec_rate)
                : bps;
            int n_data_syms = (bps > 0) ? (coded_bits + bps - 1) / bps : 1;
            // Block pilots: one every pilot_symbol_spacing data symbols
            int n_block_pilots = (ofdm_config_.pilot_symbol_spacing > 0)
                ? n_data_syms / ofdm_config_.pilot_symbol_spacing : 0;
            // Dense pilot rows: one every pilot_row_spacing data symbols
            int n_pilot_rows = (ofdm_config_.pilot_row_spacing > 0)
                ? n_data_syms / ofdm_config_.pilot_row_spacing : 0;
            // Total: 2 training + 1 sync word + data + block pilots + pilot rows + 1 tail
            int total_syms = 2 + 1 + n_data_syms + n_block_pilots + n_pilot_rows + 1;
            size_t frame_samples = (size_t)(total_syms * sym_len);
            size_t available = n_samples - (size_t)sync.frame_start;

            if (available < frame_samples) {
                // Not enough samples yet — cache sync and wait
                if (!ofdm_sync_cached_) {
                    IRIS_LOG("[OFDM-RX] frame incomplete: %zu/%zu samples (M=%.3f) — waiting",
                             available, frame_samples, sync.schmidl_metric);
                    ofdm_pending_sync_ = sync;
                    ofdm_sync_cached_ = true;
                }
                ofdm_redetect_count_++;
                if (ofdm_redetect_count_ > 300) {
                    IRIS_LOG("[OFDM-RX] redetect limit reached (%d) — abandoning cached sync", ofdm_redetect_count_);
                    size_t skip = (size_t)(ofdm_pending_sync_.frame_start + ofdm_config_.nfft);
                    skip = std::min(skip, ofdm_rx_audio_buf_.size());
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(), ofdm_rx_audio_buf_.begin() + skip);
                    ofdm_sync_cached_ = false;
                    ofdm_redetect_count_ = 0;
                }
                return;
            }
        }

        // Full frame buffered — demodulate once (no redundant re-demod).
        OfdmSyncResult demod_sync = sync;
        demod_sync.frame_start = 0;
        OfdmDemodResult result = ofdm_demod_->demodulate(
            ofdm_rx_iq_.data() + sync.frame_start,
            (int)(n_samples - sync.frame_start),
            ofdm_tone_map_, &demod_sync);

        // Frame resolved — clear cache.
        if (ofdm_sync_cached_) {
            IRIS_LOG("[OFDM-RX] frame resolved after %d waits", ofdm_redetect_count_);
        }
        ofdm_sync_cached_ = false;
        ofdm_redetect_count_ = 0;

        // ---- Secondary gate: quality gate / false positive ----
        // With headerless frames, the quality gate (mean|H| < 0.50) rejects
        // garbage before LDPC decode.  If no LLRs were produced, this is a
        // false positive or quality-gated frame — skip past and keep scanning.
        if (!result.success && result.llrs.empty()) {
            IRIS_LOG("[OFDM-RX] false positive / quality gate (mean_H=%.3f, M=%.3f) — skipping",
                     result.mean_H_mag, sync.schmidl_metric);
            // Skip past the full frame length (not just nfft) to avoid
            // re-triggering on the same noise burst.  OTA showed clusters of
            // 3-5 false triggers within 100ms on the same noise event.
            int sym_len_skip = ofdm_config_.nfft + ofdm_config_.cp_samples;
            size_t skip = (size_t)(sync.frame_start + sym_len_skip * 3);
            skip = std::min(skip, ofdm_rx_audio_buf_.size());
            ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                      ofdm_rx_audio_buf_.begin() + skip);
            return;
        }

        // ---- OFDM Chase combining ----
        // Guard: flush stored LLRs if this frame looks like a false trigger.
        // FD-ZC gate (0.30) rejects most false triggers before we get here.
        // This guard catches residual garbage with SNR below decodable range.
        if (!result.success && result.snr_db < 1.0f) {
            if (!ofdm_chase_llrs_.empty()) {
                IRIS_LOG("[OFDM-RX] Chase guard: flushing LLRs (SNR=%.1f dB too low, likely noise)",
                         result.snr_db);
                ofdm_chase_llrs_.clear();
                ofdm_chase_combines_ = 0;
            }
        }

        // If decode failed and we have stored LLRs from a previous attempt
        // at the same frame, element-wise add (MRC in LLR domain, +3 dB gain).
        if (!result.success && !result.llrs.empty() &&
            !ofdm_chase_llrs_.empty() &&
            ofdm_chase_llrs_.size() == result.llrs.size())
        {
            // Snapshot fresh LLRs before combining (never store combined version)
            std::vector<float> fresh_llrs = result.llrs;

            // Combine: add stored LLRs to fresh LLRs
            for (size_t i = 0; i < result.llrs.size(); i++) {
                result.llrs[i] += ofdm_chase_llrs_[i];
            }
            ofdm_chase_combines_++;

            IRIS_LOG("[OFDM-RX] Chase combining attempt #%d (%zu LLRs)",
                     ofdm_chase_combines_, result.llrs.size());

            // Re-attempt LDPC decode with combined LLRs
            auto decoded = LdpcCodec::decode_soft(result.llrs, result.fec_rate,
                                                   LdpcDecoder::MIN_SUM, 50);
            if (!decoded.empty()) {
                // Convert bits to bytes (headerless format: [2B len][payload][4B CRC])
                int n_decoded_bytes = (int)decoded.size() / 8;
                std::vector<uint8_t> bytes(n_decoded_bytes, 0);
                for (int i = 0; i < n_decoded_bytes * 8; i++) {
                    bytes[i / 8] |= (decoded[i] << (i % 8));
                }
                // Extract 2-byte length prefix
                if (n_decoded_bytes >= 6) {
                    uint16_t extracted_len = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
                    int crc_data_len = 2 + (int)extracted_len;
                    if (crc_data_len + 4 <= n_decoded_bytes && extracted_len > 0) {
                        uint32_t computed = crc32(bytes.data(), crc_data_len);
                        uint32_t received = (uint32_t)bytes[crc_data_len]
                            | ((uint32_t)bytes[crc_data_len + 1] << 8)
                            | ((uint32_t)bytes[crc_data_len + 2] << 16)
                            | ((uint32_t)bytes[crc_data_len + 3] << 24);
                        if (computed == received) {
                            result.success = true;
                            result.payload_len = extracted_len;
                            result.payload.assign(bytes.data() + 2,
                                                   bytes.data() + 2 + extracted_len);
                            ofdm_chase_llrs_.clear();
                            ofdm_chase_combines_ = 0;
                            IRIS_LOG("[OFDM-RX] Chase combining SUCCEEDED after %d combines",
                                     ofdm_chase_combines_ + 1);
                        }
                    }
                }
            }

            if (!result.success) {
                // Chase failed — store fresh (uncombined) LLRs for next attempt
                ofdm_chase_llrs_ = std::move(fresh_llrs);
                IRIS_LOG("[OFDM-RX] Chase combining failed — stored %zu fresh LLRs",
                         ofdm_chase_llrs_.size());
            }
        } else if (!result.success && !result.llrs.empty()) {
            // First failure — store LLRs for potential Chase combine on retransmit
            ofdm_chase_llrs_ = result.llrs;
            IRIS_LOG("[OFDM-RX] stored %zu LLRs for Chase combining",
                     ofdm_chase_llrs_.size());
        }

        if (result.success) {
            // Clear stale chase LLRs on any successful decode — prevents
            // cross-frame contamination (combining LLRs from frame N with N+1).
            if (!ofdm_chase_llrs_.empty()) {
                ofdm_chase_llrs_.clear();
                ofdm_chase_combines_ = 0;
            }
            frames_rx_++;
            tx_no_ack_count_ = 0;
            snr_db_ = result.snr_db;
            snr_preamble_db_ = result.snr_db;

            IRIS_LOG("[OFDM-RX] frame OK: %zu bytes, SNR=%.1f dB, ch_SNR=%.1f dB, %d LDPC blocks",
                     result.payload.size(), result.snr_db, result.mean_channel_snr_db,
                     result.n_ldpc_blocks);

            // TUNE gain measurement from OFDM channel estimate.
            // Power-ramp: track per-frame LDPC iters and mean|H|.
            if (tune_state_ == TuneState::WAIT_PEER && native_selfhear_guard_ <= 0) {
                float gain = 0.0f;
                const auto& est = ofdm_demod_->last_channel_estimate();
                if (!est.H.empty()) {
                    float sum = 0.0f;
                    for (const auto& h : est.H) sum += std::abs(h);
                    gain = sum / (float)est.H.size();
                }
                if (gain > 0.01f) {
                    int idx = tune_frames_measured_;
                    tune_frames_measured_++;
                    if (idx < TUNE_RAMP_COUNT) {
                        tune_rx_frame_iters_[idx] = result.worst_ldpc_iters;
                        tune_rx_frame_H_[idx] = gain;
                        tune_rx_frame_snr_[idx] = sync.snr_est;
                    }
                    if (tune_my_gain_ == 0)
                        tune_my_gain_ = gain;
                    else
                        tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;
                    IRIS_LOG("[TUNE] OFDM ramp frame %d: H=%.3f ldpc=%d iters (avg_H=%.3f)",
                             idx + 1, gain, result.worst_ldpc_iters, tune_my_gain_);
                    tune_audit("MEASURE peer=%s frame=%d raw_gain=%.4f ldpc_iters=%d decode=OK",
                               tune_peer_call_.c_str(), idx + 1, gain, result.worst_ldpc_iters);
                }
                // Extract embedded binary report from responder's ramp payload
                if (!result.payload.empty() && result.payload[0] == TUNE_REPORT_MAGIC) {
                    if (tune_parse_binary_report(result.payload.data(), result.payload.size(),
                            tune_peer_iters_, tune_peer_H_, tune_peer_snr_, TUNE_RAMP_COUNT)) {
                        int rpt_count = 0;
                        for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++)
                            if (tune_peer_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE] Extracted embedded report: %d entries from ramp payload",
                                 rpt_count);
                        tune_audit("EMBEDDED_REPORT entries=%d", rpt_count);
                    }
                }
            }

            // Also check for report in frames received during WAIT_REPORT
            // (initiator's dedicated report frame after ramp)
            if (tune_state_ == TuneState::WAIT_REPORT &&
                !result.payload.empty() && result.payload[0] == TUNE_REPORT_MAGIC) {
                // Debug: log raw report bytes for diagnostics
                {
                    std::string hex;
                    for (size_t bi = 0; bi < std::min(result.payload.size(), (size_t)32); bi++) {
                        char hb[4]; snprintf(hb, sizeof(hb), "%02X ", result.payload[bi]);
                        hex += hb;
                    }
                    IRIS_LOG("[TUNE] report payload (%zu bytes): %s", result.payload.size(), hex.c_str());
                }
                if (tune_parse_binary_report(result.payload.data(), result.payload.size(),
                        tune_peer_iters_, tune_peer_H_, tune_peer_snr_, TUNE_RAMP_COUNT)) {
                    int rpt_count = 0;
                    for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++) {
                        if (tune_peer_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE]   slot[%d]: iters=%d H=%.3f snr=%.1f",
                                 ri, tune_peer_iters_[ri], tune_peer_H_[ri], tune_peer_snr_[ri]);
                    }
                    IRIS_LOG("[TUNE] Received OFDM report frame: %d entries", rpt_count);
                    tune_audit("OFDM_REPORT entries=%d", rpt_count);
                    tune_state_ = TuneState::APPLY;
                }
            }

            // Update OFDM gearshift via Gearshift class (smoothing, LDPC boost,
            // failure downshift, cooldown — same adaptive logic as Mode A).
            // Skip during TUNE: test frames have artificially high preamble SNR
            // (full-power tones, no data) that would cause premature O0→O1 upshift.
            // The peer then can't decode O1, stops sending data, session stalls.
            bool in_tune = (tune_state_ != TuneState::IDLE &&
                            tune_state_ != TuneState::DONE);
            if (!in_tune) {
                gearshift_.feed_ldpc_iters(result.worst_ldpc_iters, 50);
                int old_level = ofdm_speed_level_;
                // Use frame SNR (post-EQ), not channel SNR (preamble-based).
                // On FM, deviation limiter clips high-PAPR data but not ZC preamble,
                // so ch_SNR overestimates by 15-20 dB → premature upshift → stall.
                ofdm_speed_level_ = gearshift_.ofdm_update(result.snr_db);
                // Feed modem gearshift state to ARQ for conflict detection
                arq_.set_modem_gearshift(ofdm_speed_level_, gearshift_.smoothed_snr(),
                                         gearshift_.cooldown());
                if (ofdm_speed_level_ != old_level) {
                    float eff_tx = ofdm_effective_tx_level();
                    IRIS_LOG("[OFDM-RX] gearshift: O%d -> O%d (frame_SNR=%.1f dB, ch_SNR=%.1f, boost=%.1f, cd=%d, tx=%.3f)",
                             old_level, ofdm_speed_level_, result.snr_db,
                             result.mean_channel_snr_db,
                             gearshift_.boost(), gearshift_.cooldown(), eff_tx);
                    // Speed level changed — stored Chase LLRs are for a different
                    // tone map / FEC rate and would corrupt combining
                    ofdm_chase_llrs_.clear();
                    ofdm_chase_combines_ = 0;
                }
            } else {
                IRIS_LOG("[OFDM-RX] TUNE frame: skipping gearshift (ch_SNR=%.1f dB)",
                         result.mean_channel_snr_db);
            }
            arq_.set_local_snr(result.snr_db);

            // Feed equalized constellation to GUI (scatter plot)
            if (!result.eq_constellation.empty()) {
                last_constellation_ = std::move(result.eq_constellation);
            }

            // Feed OFDM Kalman trace to GUI 3D viewer + CSV logging
            if (!result.kalman_trace.fwd.empty()) {
                last_kalman_trace_ = result.kalman_trace;
                kalman_log_trace(last_kalman_trace_, result.success,
                                 result.snr_db, result.mean_H_mag);
            }

            // Waterfilling: DISABLED for FM mode.
            // The channel estimate SNR (from preamble |H|) overestimates link quality
            // because the FM deviation limiter clips high-PAPR data symbols but not
            // the low-PAPR ZC preamble. After one successful QPSK frame, waterfill
            // jumped to 256QAM (222 bits/sym) based on 33.7 dB channel SNR while
            // actual frame SNR was -1.3 dB. All subsequent frames failed.
            // Rate adaptation is handled by gearshift via uniform presets.
            // if (config_.ofdm_waterfill && !result.snr_per_carrier.empty()) { ... }

            // Payload delivery — reuse existing deliver + decompression logic
            std::vector<uint8_t> payload = std::move(result.payload);

            // OFDM-KISS batch decompression
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
                    IRIS_LOG("OFDM-KISS: decompression failed, dropping");
                    size_t consumed_samples = (result.samples_consumed > 0)
                        ? (size_t)(sync.frame_start + result.samples_consumed)
                        : (size_t)(sync.frame_start + ofdm_config_.nfft * 4);
                    size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + consumed);
                    rx_overlap_buf_.clear();
                    return;
                }
            }

            // Filter TUNE payloads — not AX.25 data.  Without this filter,
            // "TUNE_TEST_FRAME" (15 bytes) and binary reports (0xBB prefix)
            // are dispatched to the KISS client as garbage AX.25 frames,
            // potentially corrupting AX.25 session state (N(R) stuck at 0).
            {
                static const uint8_t tune_marker[] = "TUNE_TEST_FRAME";
                bool is_tune_test = (payload.size() == sizeof(tune_marker) - 1 &&
                    memcmp(payload.data(), tune_marker, payload.size()) == 0);
                bool is_tune_report = (!payload.empty() && payload[0] == TUNE_REPORT_MAGIC);
                if (is_tune_test || is_tune_report) {
                    IRIS_LOG("[TUNE] Discarded OFDM %s frame (not dispatching to AX.25)",
                             is_tune_test ? "test" : "report");
                    // Drain consumed samples so the same frame isn't re-detected.
                    // Without this, self-hear audio loops: the preamble stays in
                    // ofdm_rx_audio_buf_ and is decoded 100+ times (OTA bug 2026-03-23).
                    size_t consumed_samples = (result.samples_consumed > 0)
                        ? (size_t)(sync.frame_start + result.samples_consumed)
                        : (size_t)(sync.frame_start + ofdm_config_.nfft * 4);
                    size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
                    ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                              ofdm_rx_audio_buf_.begin() + consumed);
                    rx_overlap_buf_.clear();
                    return;
                }
            }

            // Deliver payload(s)
            auto deliver_ofdm = [&](const uint8_t* data, size_t len) {
                if (ofdm_kiss_) {
                    if (!ofdm_kiss_confirmed_) {
                        ofdm_kiss_confirmed_ = true;
                        IRIS_LOG("OFDM-KISS: confirmed (heard OFDM from peer)");
                        if (gui_log_) gui_log_("OFDM-KISS: bidirectional confirmed");
                    }
                    if (!ofdm_kiss_tx_) {
                        ofdm_kiss_tx_ = true;
                        ax25_session_.set_native_active(true);
                        IRIS_LOG("OFDM-KISS: TX promoted (responder, OFDM)");
                    }
                    std::vector<uint8_t> ax25_frame(data, data + len);
                    dispatch_rx_frame(ax25_frame, false, true);
                    return;
                }
                auto arq_st = arq_.state();
                if (!loopback_mode_ &&
                    (arq_st == ArqState::CONNECTED || arq_st == ArqState::CONNECTING ||
                     arq_st == ArqState::LISTENING || arq_st == ArqState::HAILING ||
                     arq_st == ArqState::DISCONNECTING)) {
                    if (!arq_.on_frame_received(data, len)) {
                        if (rx_callback_) rx_callback_(data, len);
                    }
                } else {
                    if (rx_callback_) rx_callback_(data, len);
                }
            };

            if (payload.size() >= 3 && payload[0] == MULTI_PAYLOAD_MAGIC) {
                size_t pos = 1;
                while (pos + 2 <= payload.size()) {
                    uint16_t sub_len = payload[pos] | ((uint16_t)payload[pos + 1] << 8);
                    pos += 2;
                    if (sub_len == 0 || pos + sub_len > payload.size()) break;
                    deliver_ofdm(&payload[pos], sub_len);
                    pos += sub_len;
                }
            } else {
                deliver_ofdm(payload.data(), payload.size());
            }
        } else {
            crc_errors_++;
            gearshift_.report_failure();
            IRIS_LOG("[OFDM-RX] frame FAIL at offset %d, SNR=%.1f dB (gearshift->O%d)",
                     sync.frame_start, sync.snr_est, gearshift_.current_ofdm_level());
            ofdm_speed_level_ = gearshift_.current_ofdm_level();

            // TUNE: record preamble H even on LDPC failure.
            // The channel estimate from ZC training is valid regardless of data decode.
            // This gives us H measurements at all TX levels, not just decodable ones.
            // Gate: skip during TX (self-hearing) and reject implausible H values
            // that arise from false preamble detections on AFSK/noise energy.
            if (tune_state_ == TuneState::WAIT_PEER && native_selfhear_guard_ <= 0
                && !ptt_active_ && !rx_muted_) {
                float gain = 0.0f;
                const auto& est = ofdm_demod_->last_channel_estimate();
                if (!est.H.empty()) {
                    float sum = 0.0f;
                    for (const auto& h : est.H) sum += std::abs(h);
                    gain = sum / (float)est.H.size();
                }
                // Sanity check: reject physically implausible H values.
                // Real OFDM frames produce H in ~0.1-20 range. False detections
                // on AFSK/noise produce extreme or near-zero values.
                if (gain > 0.1f && gain < 50.0f) {
                    int idx = tune_frames_measured_;
                    tune_frames_measured_++;
                    if (idx < TUNE_RAMP_COUNT) {
                        tune_rx_frame_iters_[idx] = -2;  // -2 = preamble-only (H valid, no LDPC)
                        tune_rx_frame_H_[idx] = gain;
                        tune_rx_frame_snr_[idx] = sync.snr_est;
                    }
                    if (tune_my_gain_ == 0)
                        tune_my_gain_ = gain;
                    else
                        tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;
                    IRIS_LOG("[TUNE] OFDM ramp frame %d (preamble-only): H=%.3f SNR=%.1f dB (avg_H=%.3f)",
                             idx + 1, gain, sync.snr_est, tune_my_gain_);
                    tune_audit("MEASURE peer=%s frame=%d raw_gain=%.4f ldpc_iters=-2 decode=FAIL snr=%.1f",
                               tune_peer_call_.c_str(), idx + 1, gain, sync.snr_est);
                } else if (gain > 0.0f) {
                    IRIS_LOG("[TUNE] Rejected preamble-only H=%.3f (outside 0.1-50 range, likely false detection)",
                             gain);
                }
            }

            // Feed OFDM decode failure into ARQ HARQ path (per-block region selection)
            if (!ofdm_kiss_ && arq_.state() != ArqState::IDLE &&
                arq_.negotiated(CAP_HARQ) && !result.block_results.empty())
            {
                HarqDecodeResult harq_result;
                harq_result.any_failed = true;
                harq_result.blocks = result.block_results;
                harq_result.stored_llrs = result.llrs;
                harq_result.sym_phase_var = result.sym_phase_var;
                harq_result.fec = result.fec_rate;
                harq_result.payload_len = result.payload_len;
                harq_result.num_blocks = result.n_ldpc_blocks;
                arq_.on_decode_failed_harq(harq_result);
                IRIS_LOG("[OFDM-RX] HARQ: fed %d blocks (%zu LLRs) to ARQ for selective NACK",
                         result.n_ldpc_blocks, result.llrs.size());
            }
        }

        // Drain consumed samples from OFDM audio buffer
        size_t consumed_samples = (result.samples_consumed > 0)
            ? (size_t)(sync.frame_start + result.samples_consumed)
            : (size_t)(sync.frame_start + ofdm_config_.nfft * 4);
        size_t consumed = std::min(consumed_samples, ofdm_rx_audio_buf_.size());
        ofdm_rx_audio_buf_.erase(ofdm_rx_audio_buf_.begin(),
                                  ofdm_rx_audio_buf_.begin() + consumed);
        return;
    }

    // ============ Downconvert for native PHY ============
    // (Only reached when OFDM is not active — OFDM path returns above)
    const float* iq_data;
    std::vector<float> iq_buf;
    size_t iq_count;

    if (use_upconvert_) {
        if (rx_channel_eq_.is_configured()) {
            // Pre-allocated EQ scratch buffer (avoids per-callback heap allocation)
            if (rx_eq_tmp_.size() < (size_t)count)
                rx_eq_tmp_.resize(count);
            std::copy(audio, audio + count, rx_eq_tmp_.data());
            rx_channel_eq_.apply(rx_eq_tmp_.data(), count);
            iq_buf = downconverter_.audio_to_iq(rx_eq_tmp_.data(), count);
        } else {
            iq_buf = downconverter_.audio_to_iq(audio, count);
        }
        iq_data = iq_buf.data();
        iq_count = iq_buf.size();
    } else {
        iq_data = audio;
        iq_count = count * 2;
    }

    if (native_rx_gain_ != 1.0f && !iq_buf.empty()) {
        for (auto& s : iq_buf) s *= native_rx_gain_;
        iq_data = iq_buf.data();
    }

    rx_overlap_buf_.insert(rx_overlap_buf_.end(), iq_data, iq_data + iq_count);

    if (rx_overlap_buf_.size() > RX_OVERLAP_MAX) {
        size_t excess = rx_overlap_buf_.size() - RX_OVERLAP_MAX;
        rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                               rx_overlap_buf_.begin() + excess);
    }

    // ============ Legacy single-carrier PHY RX path ============
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
        // Always capture Kalman trace (even on decode failure — essential for debugging)
        bool decode_ok = decode_native_frame(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                              start, phy_config_, payload);
        last_kalman_trace_ = decode_kalman_trace();
        kalman_log_trace(last_kalman_trace_, decode_ok,
                         decode_snr_db(), decode_channel_gain());

        // Auto-tune: capture channel_gain from peer's test frame.
        // channel_gain comes from preamble — valid even on decode failure.
        // Skip self-heard frames: after our own TX, the overlap buffer may
        // still contain our echoed frames with inflated gain.
        if (tune_state_ == TuneState::WAIT_PEER && native_selfhear_guard_ <= 0) {
            float gain = decode_channel_gain();
            if (gain > 0.01f) {
                tune_frames_measured_++;
                if (tune_my_gain_ == 0)
                    tune_my_gain_ = gain;
                else
                    tune_my_gain_ = 0.7f * tune_my_gain_ + 0.3f * gain;  // EMA
                IRIS_LOG("[TUNE] frame %d: gain=%.4f avg=%.4f (decode=%s)",
                         tune_frames_measured_, gain, tune_my_gain_,
                         decode_ok ? "OK" : "FAIL");
                tune_audit("MEASURE peer=%s frame=%d raw_gain=%.4f ema_gain=%.4f decode=%s",
                           tune_peer_call_.c_str(), tune_frames_measured_, gain,
                           tune_my_gain_, decode_ok ? "OK" : "FAIL");
                // Don't transition here — tick() handles WAIT_PEER→SEND_REPORT
                // after accumulating enough frames or timeout.
            }
        }

        if (decode_ok) {
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
            // Successful native RX proves the link is alive.  Reset the no-ack
            // counter so gearshift doesn't penalise us for sending S-frame ACKs
            // that the peer acknowledges implicitly via I-frame N(R) — which the
            // S-frame–only sniff in the rx_holdoff path doesn't catch.
            tx_no_ack_count_ = 0;
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
                // Don't feed gearshift during auto-tune — tune test frames
                // may decode at higher SNR than actual data (different direction,
                // different gain), causing premature upshift.
                bool in_tune = (tune_state_ != TuneState::IDLE &&
                                tune_state_ != TuneState::DONE);
                if (!in_tune) {
                    gearshift_.feed_ldpc_iters(ldpc_iters, 50);
                    int old_level = gearshift_.current_level();
                    gearshift_.update(snr);
                    int new_level = gearshift_.current_level();
                    IRIS_LOG("[SNR] est=%.1f dB, ldpc_iters=%d, boost=+%.1f, eff=%.1f, gearshift: %d->%d (smoothed=%.1f, cd=%d)",
                             snr, ldpc_iters, gearshift_.boost(),
                             gearshift_.smoothed_snr() + gearshift_.boost(),
                             old_level, new_level, gearshift_.smoothed_snr(), gearshift_.cooldown());
                    // Feed modem gearshift state to ARQ for conflict detection
                    arq_.set_modem_gearshift(new_level, gearshift_.smoothed_snr(),
                                             gearshift_.cooldown());
                } else {
                    IRIS_LOG("[SNR] est=%.1f dB, ldpc_iters=%d (tune frame, gearshift skipped)",
                             snr, ldpc_iters);
                }
                arq_.set_local_snr(snr);
            }

            if (native_demod_)
                last_constellation_ = native_demod_->symbols();

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
                            int migrated = 0;
                            std::queue<std::vector<uint8_t>> keep;
                            while (!ax25_tx_queue_.empty()) {
                                auto fr = std::move(ax25_tx_queue_.front());
                                ax25_tx_queue_.pop();
                                bool is_iframe = fr.size() > 14 && (fr[14] & 0x01) == 0;
                                if (is_iframe) {
                                    constexpr size_t TX_QUEUE_MAX = 32;
                                    if (tx_queue_.size() >= TX_QUEUE_MAX) {
                                        IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                                        tx_queue_.pop();
                                    }
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

                    // Filter out tune test frames — they are NOT AX.25 data.
                    // If dispatched, the garbage bytes get parsed as RNR/REJ and
                    // poison the AX.25 session state.
                    static const uint8_t tune_marker[] = "TUNE_TEST_FRAME";
                    if (len == sizeof(tune_marker) - 1 &&
                        memcmp(data, tune_marker, len) == 0) {
                        IRIS_LOG("[TUNE] Discarded decoded test frame (not dispatching to AX.25)");
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

            // HARQ frame: strip retx descriptor, apply combining, deliver new data only
            if (decode_last_harq_flag() && arq_.negotiated(CAP_HARQ) && !payload.empty()) {
                HarqRetxDescriptor retx_desc;
                size_t consumed = deserialize_retx_descriptor(
                    payload.data(), payload.size(), retx_desc);
                if (consumed > 0 && consumed <= payload.size()) {
                    // TODO: apply retx bits to stored HARQ RX state for LLR combining
                    // For now, log and deliver the new data portion
                    IRIS_LOG("[HARQ] RX retx descriptor: seq=%d, %zu regions, %zu retx bytes",
                             retx_desc.original_seq, retx_desc.regions.size(),
                             retx_desc.retx_bits.size());
                    std::vector<uint8_t> new_data(payload.begin() + consumed, payload.end());
                    if (!new_data.empty())
                        deliver(new_data.data(), new_data.size());
                } else {
                    deliver(payload.data(), payload.size());
                }
            } else if (payload.size() >= 3 && payload[0] == MULTI_PAYLOAD_MAGIC) {
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
            // Timeout: enough time for remaining samples to arrive, plus 50% margin.
            // Replaces fixed 10s timeout that caused 30-40s dead gaps on noisy FM
            // channels when false preamble detections blocked TX for the full duration.
            {
                size_t have = rx_overlap_buf_.size() / 2;  // IQ pairs already buffered
                size_t remaining = (pending_need_floats_ / 2 > have)
                    ? (pending_need_floats_ / 2 - have) : 0;
                int timeout_samples = (int)(remaining * 3 / 2);  // 1.5x margin
                int min_timeout = config_.sample_rate * 1;       // floor: 1s
                int max_timeout = config_.sample_rate * 7;       // cap: 7s
                if (timeout_samples < min_timeout) timeout_samples = min_timeout;
                if (timeout_samples > max_timeout) timeout_samples = max_timeout;
                pending_frame_timeout_ = timeout_samples;
            }
            IRIS_LOG("RX decode: need more data at offset %d (buf=%zu IQ, need ~%zu, timeout=1-7s)",
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
            // Native ARQ NACK: tell commander to retransmit immediately.
            {
                ArqState arq_st = arq_.state();
                if (!ofdm_kiss_ && arq_st != ArqState::IDLE) {
                    if (arq_.negotiated(CAP_HARQ)) {
                        // HARQ: per-block decode with LLR storage + extended NACK
                        auto harq_result = decode_native_frame_harq(
                            rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                            start, phy_config_);
                        arq_.on_decode_failed_harq(harq_result);
                    } else {
                        // Legacy: full Chase combining
                        arq_.on_decode_failed();
                    }
                }
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
        // Fresh buffer at pos 0: turn PTT on before sending.
        // Covers externally-filled buffers (tune test frames) that bypass
        // the normal queue→build→ptt_on path below.
        if (tx_pos_ == 0) ptt_on();

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
            // Self-hear guard: after native TX, ignore native RX for 300ms
            // to filter self-heard frames from FM radio audio loopback.
            // Pipeline latency is typically 50-200ms; 300ms covers worst case
            // without blocking legitimate responses from the peer.
            if (state_ == ModemState::TX_NATIVE) {
                native_selfhear_guard_ = config_.sample_rate * 3 / 10;  // 300ms
                // Mandatory listen window: after native TX, defer next TX
                // long enough for the peer to fully transmit its response.
                // Role-asymmetric: responder responds sooner, initiator yields.
                {
                    int native_listen = (ax25_session_.is_active() && ax25_session_.we_initiated())
                        ? config_.sample_rate * 3 / 2  // initiator: 1.5s (yield to response)
                        : config_.sample_rate;          // responder: 1.0s (respond sooner)
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
    // Channel busy pauses AX.25 T1/T3 timers. Only pause when someone is
    // actually on the air (DCD = peer transmitting, PTT handled separately in
    // ptt_on/ptt_off). CSMA holdoff and probe countdown are TX deferrals —
    // T1 must keep running during those because the peer may be responding.
    bool channel_active = dcd_busy || native_frame_pending;
    ax25_session_.set_channel_busy(channel_active);

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
        } else if (!tx_queue_.empty() && !ofdm_kiss_probing_ &&
                   (tune_state_ == TuneState::IDLE || tune_state_ == TuneState::DONE) &&
                   tune_post_holdoff_ <= 0) {
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
                // Adaptive: grows from 6s toward 12s on success, halves on REJ.
                // PHY bps: use OFDM tone map throughput when OFDM active,
                // otherwise legacy single-carrier throughput.
                int phy_bps;
                if (ofdm_phy_active_ && !native_hail_active) {
                    // OFDM: throughput from tone map (accounts for carriers, FEC, CP)
                    // Use streaming estimate (50 CW) for batch sizing
                    phy_bps = (int)tone_map_throughput(ofdm_tone_map_, ofdm_config_, 50);
                } else {
                    int level = gearshift_.current_level();
                    phy_bps = net_throughput(level, phy_config_.baud_rate);
                }
                // Cap payload at actual LDPC block capacity for current speed level.
                // LDPC k bits = data bits per block.  Minus 6 bytes overhead (2 len + 4 CRC).
                // Multi-codeword: pack multiple LDPC blocks per frame to amortize
                // preamble + pilot overhead. Each block is independently decodable.
                // Single codeword per frame: keeps frames short (~1.3s at O0)
                // for reliable sync+decode. Multi-codeword doubles frame length
                // (2.5s at O0) — too long for the redetect buffer (1.5s limit)
                // and exposes more phase drift on FM. Re-enable for O2+ when
                // throughput > reliability.
                static constexpr int OFDM_CODEWORDS_PER_FRAME = 1;
                size_t max_payload;
                if (ofdm_phy_active_ && !native_hail_active) {
                    // Worst-case per-block capacity (O1 r1/2, K=800) × n_codewords.
                    int k_bytes = LdpcCodec::block_size(LdpcRate::RATE_1_2) / 8;
                    int per_block = std::max(20, k_bytes - 6);
                    max_payload = (size_t)(per_block * OFDM_CODEWORDS_PER_FRAME);
                } else {
                    max_payload = (size_t)NATIVE_MAX_PAYLOAD;
                }
                size_t max_batch = std::min(max_payload,
                                             (size_t)(phy_bps * batch_airtime_s_ / 8));
                // Floor ensures at least one frame fits, but never exceed LDPC capacity
                size_t batch_floor = std::min((size_t)200, max_payload);
                if (max_batch < batch_floor) max_batch = batch_floor;

                std::vector<std::vector<uint8_t>> batch;
                size_t total_bytes = 0;
                while (!tx_queue_.empty()) {
                    auto& front = tx_queue_.front();
                    // First sub-frame: 3 bytes (1 magic + 2 len), subsequent: 2 bytes (2 len)
                    size_t overhead = batch.empty() ? 3 : 2;
                    // Always take the first frame (even if oversized) to avoid empty batches
                    if (!batch.empty() && total_bytes + overhead + front.size() > max_batch)
                        break;
                    total_bytes += overhead + front.size();
                    batch.push_back(std::move(front));
                    tx_queue_.pop();
                }

                // Detect if batch contains only ARQ control frames (non-DATA).
                // Control frames at robust rate (~20ms extra) prevents cascading
                // retransmit timeouts when channel has degraded.
                bool batch_is_control = false;
                if (!ofdm_kiss_ && arq_.state() != ArqState::IDLE) {
                    batch_is_control = true;
                    for (auto& sub : batch) {
                        if (!sub.empty() && sub[0] == (uint8_t)ArqType::DATA) {
                            batch_is_control = false;
                            break;
                        }
                    }
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

                // Start T1 at TX time for OFDM-KISS frames.
                // notify_outgoing() may have been called before native_active_
                // was set (KISS I-frames queued during AFSK/probe phase).
                if (ofdm_kiss_tx_) {
                    ax25_session_.start_t1_if_unacked();
                }

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

                std::vector<float> iq;  // Interleaved I/Q output (native PHY)
                std::vector<std::complex<float>> ofdm_iq;  // Complex OFDM samples (real-valued after Hermitian sym)

                if (ofdm_phy_active_ && !native_hail_active) {
                    // ============ OFDM PHY TX path ============
                    // Start from gearshift's OFDM level (driven by RX SNR + LDPC boost)
                    ofdm_speed_level_ = gearshift_.current_ofdm_level();
                    tx_no_ack_count_++;

                    // Control frames at robust rate (~20ms extra) prevents cascading
                    // retransmit timeouts when channel has degraded.
                    if (batch_is_control && ofdm_speed_level_ > 0) {
                        IRIS_LOG("[TX-OFDM] control frame batch -> forcing O0 (robust)");
                        ofdm_speed_level_ = 0;
                    }

                    // TX-side override: no peer ACK for 6+ frames → force downshift
                    if (tx_no_ack_count_ >= 6 && ofdm_speed_level_ > 0) {
                        ofdm_speed_level_ = std::max(0, ofdm_speed_level_ - 1);
                        gearshift_.force_ofdm_level(ofdm_speed_level_);  // sync gearshift
                        IRIS_LOG("OFDM Gearshift: no peer ACK for %d TX frames -> O%d",
                                 tx_no_ack_count_, ofdm_speed_level_);
                    }
                    // TX-side cap: peer's reported SNR limits our TX speed
                    if (peer_snr_db_ > 0 && ofdm_speed_level_ > 0) {
                        int peer_max = ofdm_snr_to_speed_level(peer_snr_db_);
                        if (peer_max < ofdm_speed_level_) {
                            IRIS_LOG("OFDM Gearshift: capping O%d -> O%d (peer SNR %.1f dB)",
                                     ofdm_speed_level_, peer_max, peer_snr_db_);
                            ofdm_speed_level_ = peer_max;
                        }
                    }

                    // Select tone map: O-levels map 1:1 to uniform presets (preset = level + 1)
                    static const LdpcRate ofdm_level_to_fec[] = {
                        LdpcRate::RATE_1_2,  // O0: BPSK r1/2
                        LdpcRate::RATE_1_2,  // O1: QPSK r1/2
                        LdpcRate::RATE_3_4,  // O2: QPSK r3/4
                        LdpcRate::RATE_1_2,  // O3: 16QAM r1/2
                        LdpcRate::RATE_5_8,  // O4: 16QAM r5/8
                        LdpcRate::RATE_3_4,  // O5: 16QAM r3/4
                        LdpcRate::RATE_5_8,  // O6: 64QAM r5/8
                        LdpcRate::RATE_3_4,  // O7: 64QAM r3/4
                        LdpcRate::RATE_5_8,  // O8: 256QAM r5/8
                        LdpcRate::RATE_3_4,  // O9: 256QAM r3/4
                    };

                    int sl = std::clamp(ofdm_speed_level_, 0, NUM_OFDM_SPEED_LEVELS - 1);
                    if (config_.ofdm_waterfill && ofdm_tone_map_.tone_map_id == 0 &&
                        ofdm_tone_map_.total_bits_per_symbol > 0) {
                        // Waterfill: keep per-carrier bit loading, update FEC from gearshift
                        ofdm_tone_map_.fec_rate = ofdm_level_to_fec[sl];
                    } else {
                        // Uniform: all carriers same modulation
                        uint8_t preset = static_cast<uint8_t>(sl + 1);
                        ofdm_tone_map_ = get_uniform_tone_map(preset, ofdm_config_);
                    }
                    ofdm_tone_map_.use_nuc = config_.ofdm_nuc;
                    ofdm_tone_map_.n_codewords = OFDM_CODEWORDS_PER_FRAME;
                    LdpcRate fec = ofdm_tone_map_.fec_rate;

                    IRIS_LOG("[TX-OFDM] speed=O%d %s fec=%d/%d, %zu bytes, %d cw/frame, %d carriers, %d bits/sym",
                             ofdm_speed_level_,
                             ofdm_tone_map_.tone_map_id == 0 ? "waterfill" : "uniform",
                             OFDM_SPEED_LEVELS[ofdm_speed_level_].fec_rate_num,
                             OFDM_SPEED_LEVELS[ofdm_speed_level_].fec_rate_den,
                             frame_data.size(), OFDM_CODEWORDS_PER_FRAME,
                             ofdm_config_.n_data_carriers,
                             ofdm_tone_map_.total_bits_per_symbol);

                    // Build OFDM frame (returns complex samples, real-valued via Hermitian symmetry)
                    ofdm_iq = ofdm_mod_->build_ofdm_frame(
                        frame_data.data(), frame_data.size(), ofdm_tone_map_, fec,
                        OFDM_CODEWORDS_PER_FRAME);
                    if (ofdm_iq.empty()) {
                        IRIS_LOG("[TX-OFDM] build_ofdm_frame rejected %zu bytes — skipping TX",
                                 frame_data.size());
                    }
                } else {
                    // ============ Legacy single-carrier PHY TX path ============
                    int level = gearshift_.current_level();
                    PhyConfig tx_config = phy_config_;

                    // Control frames at robust rate (~20ms extra) prevents cascading
                    // retransmit timeouts when channel has degraded.
                    if (batch_is_control && level > 0) {
                        IRIS_LOG("[TX] control frame batch -> forcing level 0 (robust)");
                        level = 0;
                    }

                    Modulation tx_mod;
                    int tx_fec_n, tx_fec_d;
                    if (native_hail_active) {
                        tx_mod = Modulation::BPSK;
                        tx_fec_n = 1;
                        tx_fec_d = 2;
                        IRIS_LOG("TX native hail frame %zu bytes (BPSK r1/2)", frame_data.size());
                    } else {
                        tx_no_ack_count_++;
                        if (tx_no_ack_count_ >= 6 && level > 0) {
                            gearshift_.report_failure();
                            level = gearshift_.current_level();
                            IRIS_LOG("Gearshift: no peer ACK for %d TX frames -> report_failure (level=%d)",
                                     tx_no_ack_count_, level);
                        }
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
                    if (arq_.harq_has_pending_retx() && arq_.negotiated(CAP_HARQ)) {
                        iq = build_native_frame_harq(frame_data.data(), frame_data.size(),
                                                      arq_.harq_pending_retx_desc(),
                                                      tx_config, fec);
                        arq_.harq_clear_pending_retx();
                        IRIS_LOG("[TX] HARQ frame: retx piggybacked + %zu bytes new data",
                                 frame_data.size());
                    } else {
                        iq = build_native_frame(frame_data.data(), frame_data.size(), tx_config, fec);
                    }
                }

                // Raise T1 floor to account for this frame's airtime.
                float frame_airtime_s;

                if (ofdm_phy_active_ && ofdm_mod_ && ofdm_iq.empty()) {
                    // Safety net: payload exceeded LDPC capacity despite batch
                    // sizing to worst-case FEC.  ARQ will retransmit on timeout.
                    tx_buffer_.clear();
                    IRIS_LOG("[TX-OFDM] frame rejected (payload %zu > LDPC capacity) — ARQ will retry",
                             frame_data.size());
                    state_ = ModemState::IDLE;
                    return;
                }

                if (ofdm_phy_active_ && ofdm_mod_) {
                    // OFDM: IFFT with Hermitian symmetry produces real-valued audio.
                    // Bypass upconverter — signal is already at audio frequencies.
                    // Normalize RMS to match native single-carrier level (~0.707).
                    // Without this, OFDM with N carriers is √(2N) louder than native
                    // (e.g. 42 carriers → 9.2× → 19 dB overdrive → FM limiter destroys signal).
                    // Extract real passband audio from OFDM IFFT
                    tx_buffer_.resize(ofdm_iq.size());
                    for (size_t i = 0; i < ofdm_iq.size(); i++)
                        tx_buffer_[i] = ofdm_iq[i].real();
                    frame_airtime_s = (float)ofdm_iq.size() / (float)config_.sample_rate;

                    // OFDM bypasses probe-based TX EQ. The 127-tap FIR group
                    // delay can exceed the OFDM CP (64 samples default), risking
                    // ISI. OFDM handles per-carrier equalization internally.

                    // Normalize RMS AFTER EQ to control total power.
                    // Target 0.50 is the "full scale" OFDM output before tx_level
                    // scaling. tx_level (set by TUNE) controls actual drive into FM
                    // transmitter. Default tx_level=0.50 → output RMS≈0.25.
                    // TUNE adjusts tx_level targeting mean|H|≈1.0 at the receiver.
                    // Normalize DATA portion only — preamble (Schmidl-Cox with
                    // sqrt(2) boost) is scaled separately to preserve correlation.
                    int preamble_samples = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
                    int data_start = std::min(preamble_samples, (int)tx_buffer_.size());
                    constexpr float OFDM_TARGET_RMS = 0.50f;

                    // RMS of data portion only
                    float sum_sq = 0.0f;
                    for (int i = data_start; i < (int)tx_buffer_.size(); i++)
                        sum_sq += tx_buffer_[i] * tx_buffer_[i];
                    int data_len = (int)tx_buffer_.size() - data_start;
                    float ofdm_rms = (data_len > 0) ? std::sqrt(sum_sq / (float)data_len) : 0.0f;
                    float ofdm_scale = (ofdm_rms > 1e-6f) ? (OFDM_TARGET_RMS / ofdm_rms) : 0.1f;
                    for (int i = data_start; i < (int)tx_buffer_.size(); i++)
                        tx_buffer_[i] *= ofdm_scale;

                    // Scale preamble so its peak fits within ±0.90 (preserves SC
                    // correlation properties, leaves margin below 0.95 hard clip).
                    constexpr float PREAMBLE_PEAK_LIMIT = 0.90f;
                    float preamble_peak = 0.0f;
                    for (int i = 0; i < data_start; i++)
                        preamble_peak = std::max(preamble_peak, std::abs(tx_buffer_[i]));
                    if (preamble_peak > PREAMBLE_PEAK_LIMIT) {
                        float pscale = PREAMBLE_PEAK_LIMIT / preamble_peak;
                        for (int i = 0; i < data_start; i++)
                            tx_buffer_[i] *= pscale;
                    }

                    // Hard clip to ±0.95 (soundcard range, catches any remaining peaks)
                    for (auto& s : tx_buffer_) {
                        if (s > 0.95f) s = 0.95f;
                        else if (s < -0.95f) s = -0.95f;
                    }

                    IRIS_LOG("[TX-OFDM] passband direct: %zu samples, RMS=%.3f->%.3f (scale=%.4f, preamble=%d samp peak=%.3f, EQ=%s)",
                             tx_buffer_.size(), ofdm_rms, OFDM_TARGET_RMS, ofdm_scale,
                             data_start, preamble_peak,
                             tx_channel_eq_.is_configured() ? "on" : "off");

                    // ---- OFDM multi-frame burst: append additional frames ----
                    // First frame is built above.  If queue still has data and
                    // the batch airtime budget allows, build more OFDM frames
                    // (each with own preamble → independent sync + channel est)
                    // and concatenate audio in the same PTT cycle.
                    {
                        int burst_frames = 1;
                        constexpr int MAX_BURST = 8;
                        size_t burst_max_payload = (size_t)(std::max(20,
                            LdpcCodec::block_size(ofdm_tone_map_.fec_rate) / 8 - 6) * OFDM_CODEWORDS_PER_FRAME);

                        while (!tx_queue_.empty() &&
                               frame_airtime_s < batch_airtime_s_ &&
                               burst_frames < MAX_BURST) {
                            // Batch assembly (one LDPC block)
                            std::vector<std::vector<uint8_t>> xbatch;
                            size_t xtotal = 0;
                            while (!tx_queue_.empty()) {
                                auto& front = tx_queue_.front();
                                size_t overhead = xbatch.empty() ? 3 : 2;
                                if (!xbatch.empty() && xtotal + overhead + front.size() > burst_max_payload)
                                    break;
                                xtotal += overhead + front.size();
                                xbatch.push_back(std::move(front));
                                tx_queue_.pop();
                            }
                            if (xbatch.empty()) break;

                            // Logging
                            if (ofdm_kiss_ && packet_log_) {
                                for (auto& sub : xbatch) {
                                    if (sub.size() >= 14)
                                        packet_log_(true, "OFDM-KISS", describe_ax25(sub.data(), sub.size()));
                                }
                            }

                            // Frame wrapping
                            std::vector<uint8_t> xframe;
                            if (xbatch.size() == 1 && xbatch[0].size() > 0 &&
                                xbatch[0][0] != MULTI_PAYLOAD_MAGIC) {
                                xframe = std::move(xbatch[0]);
                            } else {
                                xframe.reserve(xtotal);
                                xframe.push_back(MULTI_PAYLOAD_MAGIC);
                                for (auto& sub : xbatch) {
                                    uint16_t len = (uint16_t)sub.size();
                                    xframe.push_back(len & 0xFF);
                                    xframe.push_back((len >> 8) & 0xFF);
                                    xframe.insert(xframe.end(), sub.begin(), sub.end());
                                }
                            }

                            // Compression
                            if (ofdm_kiss_tx_ && (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) &&
                                xframe.size() > 20 && xframe[0] != B2F_DATA_MAGIC) {
                                std::vector<uint8_t> cbuf(xframe.size() + COMPRESS_HEADER_SIZE + 256);
                                int clen = ofdm_kiss_tx_compressor_.compress_block(
                                    xframe.data(), (int)xframe.size(),
                                    cbuf.data(), (int)cbuf.size());
                                if (clen > 0 && clen + 1 < (int)xframe.size()) {
                                    xframe.clear();
                                    xframe.reserve(1 + clen);
                                    xframe.push_back(COMPRESSED_PAYLOAD_MAGIC);
                                    xframe.insert(xframe.end(), cbuf.begin(), cbuf.begin() + clen);
                                }
                            }

                            IRIS_LOG("[TX-OFDM] burst frame %d: %zu bytes (%zu sub-frames)",
                                     burst_frames + 1, xframe.size(), xbatch.size());

                            // Build OFDM frame (multi-codeword)
                            auto xiq = ofdm_mod_->build_ofdm_frame(
                                xframe.data(), xframe.size(),
                                ofdm_tone_map_, ofdm_tone_map_.fec_rate,
                                OFDM_CODEWORDS_PER_FRAME);
                            if (xiq.empty()) {
                                IRIS_LOG("[TX-OFDM] burst frame %d rejected — stopping burst",
                                         burst_frames + 1);
                                break;
                            }

                            // Convert to audio + per-frame normalize + append
                            size_t bstart = tx_buffer_.size();
                            tx_buffer_.resize(bstart + xiq.size());
                            for (size_t i = 0; i < xiq.size(); i++)
                                tx_buffer_[bstart + i] = xiq[i].real();

                            int xpreamble = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
                            int xdata_start = (int)bstart + std::min(xpreamble, (int)xiq.size());

                            float xsq = 0.0f;
                            for (int i = xdata_start; i < (int)tx_buffer_.size(); i++)
                                xsq += tx_buffer_[i] * tx_buffer_[i];
                            int xdlen = (int)tx_buffer_.size() - xdata_start;
                            float xrms = (xdlen > 0) ? std::sqrt(xsq / (float)xdlen) : 0.0f;
                            float xscale = (xrms > 1e-6f) ? (OFDM_TARGET_RMS / xrms) : 0.1f;
                            for (int i = xdata_start; i < (int)tx_buffer_.size(); i++)
                                tx_buffer_[i] *= xscale;

                            float xpeak = 0.0f;
                            for (int i = (int)bstart; i < xdata_start; i++)
                                xpeak = std::max(xpeak, std::abs(tx_buffer_[i]));
                            if (xpeak > PREAMBLE_PEAK_LIMIT) {
                                float xps = PREAMBLE_PEAK_LIMIT / xpeak;
                                for (int i = (int)bstart; i < xdata_start; i++)
                                    tx_buffer_[i] *= xps;
                            }

                            for (size_t i = bstart; i < tx_buffer_.size(); i++) {
                                if (tx_buffer_[i] > 0.95f) tx_buffer_[i] = 0.95f;
                                else if (tx_buffer_[i] < -0.95f) tx_buffer_[i] = -0.95f;
                            }

                            float xtime = (float)xiq.size() / (float)config_.sample_rate;
                            frame_airtime_s += xtime;
                            burst_frames++;
                        }

                        if (burst_frames > 1) {
                            IRIS_LOG("[TX-OFDM] burst complete: %d frames, %.1fs, %zu samples",
                                     burst_frames, frame_airtime_s, tx_buffer_.size());
                        }
                    }
                } else {
                    // Native single-carrier PHY: IQ through upconverter as before
                    frame_airtime_s = (float)(iq.size() / 2) / (float)config_.sample_rate;
                    if (use_upconvert_) {
                        tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                        if (tx_channel_eq_.is_configured()) {
                            tx_channel_eq_.apply(tx_buffer_.data(), (int)tx_buffer_.size());
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
                }
                ax25_session_.set_t1_floor_for_airtime(frame_airtime_s);
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

            // Apply tx_level gain control to TX buffer.
            // OFDM: per-O-level tx_level (base from TUNE + offset for modulation).
            //   TUNE calibrates ofdm_tx_base_ for O0, higher modes get dB offsets
            //   to account for tighter constellations (more sensitive to FM clipping).
            // Native: tx_level applied directly to upconverted audio.
            {
                float tx_gain = (ofdm_phy_active_ && ofdm_mod_)
                    ? ofdm_effective_tx_level()
                    : config_.tx_level;
                for (auto& s : tx_buffer_)
                    s *= tx_gain;
            }

            // Pre-TX silence for PTT hardware settle.
            // AX.25: 50ms hardware settle + flag bytes provide the rest of TXDELAY.
            // OFDM: full TXDELAY needed — no flag preamble, training symbols are first.
            int ptt_settle_ms;
            if (ofdm_phy_active_ && ofdm_mod_) {
                // Use adaptive TXDELAY if set (reduced after successful probe/tune)
                ptt_settle_ms = (ofdm_txdelay_ms_ > 0) ? ofdm_txdelay_ms_ : config_.ptt_pre_delay_ms;
                IRIS_LOG("[TX-OFDM] pre-delay: %d ms (TXDELAY%s)", ptt_settle_ms,
                         ofdm_txdelay_ms_ > 0 ? ", adaptive" : "");
            } else {
                ptt_settle_ms = 50;  // native SC / AX.25: preamble handles the rest
            }
            int pre_samples = ptt_settle_ms * config_.sample_rate / 1000;
            if (pre_samples > 0)
                tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);

            // Post-TX tail: keep carrier up so peer's squelch stays open.
            // OFDM needs extra tail because frames are short (~176ms).
            int post_ms = config_.ptt_post_delay_ms;
            if (ofdm_phy_active_ && ofdm_mod_)
                post_ms = std::max(post_ms, 100);  // minimum 100ms tail for OFDM
            int post_samples = post_ms * config_.sample_rate / 1000;
            if (post_samples > 0)
                tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

            tx_pos_ = 0;
            frames_tx_++;
        }
    }
    // ptt_on() calls set_channel_busy() which acquires timer_mutex_.
    // Lock order: modem_mutex_ → timer_mutex_ (consistent with ax25_session_.tick()).
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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

    // Notify AX.25 session of outgoing frame so it can track KISS-initiated
    // connections (SABM/DISC) without generating duplicate frames.
    // Must be under modem_mutex_ to synchronize with connection header injection.
    ax25_session_.notify_outgoing(frame, len);

    // Buffer KISS client I-frame info fields during AFSK phase for B2F replay.
    // The session send_callback only captures session-generated frames (RR etc),
    // not KISS client data — but the B2F SID/FC/FS exchange comes from the client.
    if (config_.b2f_unroll && !ofdm_kiss_tx_ && len > 16) {
        uint8_t ctrl = frame[14];
        if ((ctrl & 0x01) == 0) {  // I-frame
            b2f_afsk_tx_history_.emplace_back(frame + 16, frame + len);
        }
    }

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
            // Don't intercept resume transfers — partial LZHUF can't be unrolled
            bool intercepting = ofdm_kiss_b2f_.is_tx_payload_active() &&
                                !ofdm_kiss_b2f_.is_resume_transfer();

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
                            constexpr size_t TX_QUEUE_MAX = 32;
                            if (tx_queue_.size() >= TX_QUEUE_MAX) {
                                IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                                tx_queue_.pop();
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
            // Skip interception for resume transfers (partial LZHUF)
            if (ofdm_kiss_b2f_.is_tx_payload_active() &&
                !ofdm_kiss_b2f_.is_resume_transfer() && !b2f_proxy_active_) {
                b2f_proxy_active_ = true;
                b2f_proxy_vr_ = 0;
                b2f_proxy_plaintext_.clear();
                memcpy(b2f_proxy_addr_, frame, 14);
                b2f_proxy_addr_valid_ = true;
                IRIS_LOG("[B2F-PROXY] TX interception started (local proposer payload)");
            } else if (ofdm_kiss_b2f_.is_resume_transfer() && !b2f_proxy_active_) {
                IRIS_LOG("[B2F-PROXY] Resume transfer — skipping unroll (partial LZHUF)");
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
    constexpr size_t TX_QUEUE_MAX = 32;
    if (tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
        tx_queue_.pop();
    }
    tx_queue_.push(std::move(tx_frame));
    if (gui_log_ && len >= 14)
        gui_log_("[TX] " + describe_ax25(frame, len));
}

void Modem::ax25_connect(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    // Route to AX.25 session if active, otherwise fall back to native ARQ path
    if (ax25_session_.state() == Ax25SessionState::CONNECTED ||
        ax25_session_.state() == Ax25SessionState::TIMER_RECOVERY) {
        ax25_session_.send_data(data, len);
    } else {
        queue_tx_frame(data, len);
    }
}

void Modem::arq_connect(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("ARQ connect to %s", remote_callsign.c_str());
    // Generate ephemeral X25519 keypair for DH key exchange
    generate_ephemeral_x25519();
    // Start in AX.25 — upgrade to native after XID negotiation
    arq_.connect(remote_callsign);
}

void Modem::arq_disconnect() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    IRIS_LOG("ARQ disconnect");
    arq_.disconnect();
}

void Modem::arq_listen() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    arq_.tick();
    ax25_session_.tick();
    probe_.tick();

    // Deferred PROBE:START — queued from state callback.
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

    // OFDM-KISS T1 watchdog: if native mode is active, session is connected,
    // and T1 isn't running despite unacked frames, force-start T1.
    // Belt-and-suspenders for the T1-never-starts bug (OTA 2026-03-22).
    // Runs every tick (50ms) — start_t1_if_unacked() is a fast no-op when
    // T1 is already running or V(A)==V(S).
    if (ofdm_kiss_tx_) {
        ax25_session_.start_t1_if_unacked();
    }

    // Auto-tune FSM tick
    if (tune_state_ != TuneState::IDLE && tune_state_ != TuneState::DONE) {
        tune_timeout_--;
        if (tune_timeout_ <= 0) {
            IRIS_LOG("[TUNE] Timeout (state=%d, frames=%d)",
                     (int)tune_state_.load(), tune_frames_measured_);
            tune_audit("TIMEOUT peer=%s role=%s state=%d my_gain=%.4f peer_gain=%.4f frames=%d",
                       tune_peer_call_.c_str(),
                       tune_is_initiator_ ? "initiator" : "responder",
                       (int)tune_state_.load(), tune_my_gain_, tune_peer_gain_,
                       tune_frames_measured_);

            // Unilateral fallback: if we received and measured the peer's ramp
            // frames but never got their report (report exchange collision),
            // apply corrections from our local measurements.  We know our own
            // TX levels and the peer's measured channel gain from the frames we
            // decoded — that's enough to set RX gain.  TX level stays at probe
            // base (no peer report to refine it).
            if (tune_my_gain_ > 0.01f && tune_frames_measured_ >= 3) {
                IRIS_LOG("[TUNE] Timeout fallback: applying local measurements "
                         "(my_gain=%.4f, %d frames)",
                         tune_my_gain_, tune_frames_measured_);
                if (gui_log_) gui_log_("[TUNE] Timeout — applying local measurements");
                tune_apply_corrections();  // sets DONE
            } else {
                // No usable local data — apply probe-calibrated OFDM base as safe fallback.
                // Without this, tx_level stays at the uncalibrated config value
                // (often 0.88+) which overdrives FM deviation and clips the signal.
                if (ofdm_tx_base_ > 0.01f && config_.tx_level > ofdm_tx_base_ * 1.5f) {
                    float old_level = config_.tx_level;
                    config_.tx_level = ofdm_tx_base_;
                    config_.calibrated_tx_level = ofdm_tx_base_;
                    IRIS_LOG("[TUNE] Timeout fallback: tx_level %.3f -> %.3f (probe-calibrated base)",
                             old_level, ofdm_tx_base_);
                }
                if (gui_log_) gui_log_("[TUNE] Timeout — no response from peer");
                tune_state_ = TuneState::IDLE;
            }
        } else {
            TuneState ts = tune_state_;
            if (ts == TuneState::SEND_START) {
                // Wait for any pending TX to drain, then build and TX test frames.
                // Initiator enters here after queuing TUNE:START UI.
                // Responder enters here after measuring initiator's test frames.
                if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
                    IRIS_LOG("[TUNE] %s: TX queue drained, building test frames",
                             tune_is_initiator_ ? "Initiator" : "Responder");
                    if (gui_log_) gui_log_("[TUNE] Sending test frames...");
                    tune_build_and_queue_test_frame();  // builds all frames + sets tx_pos_=0
                    tune_state_ = TuneState::TX_TEST;
                    // ptt_on() happens automatically in process_tx when tx_pos_==0 && !tx_buffer_.empty()
                }
            } else if (ts == TuneState::TX_TEST) {
                // Wait for test frames to finish transmitting.
                // process_tx clears tx_buffer_ and enters drain automatically,
                // so check that we're back to IDLE (drain complete).
                if (state_ == ModemState::IDLE && tx_buffer_.empty()) {
                    if (tune_is_initiator_) {
                        IRIS_LOG("[TUNE] Test frames sent, waiting for peer");
                        if (gui_log_) gui_log_("[TUNE] Test frames sent, waiting for peer...");
                        tune_state_ = TuneState::WAIT_PEER;
                        tune_wait_peer_ticks_ = 0;
                    } else {
                        // Responder: test frames done — wait for initiator's report.
                        // Lockstep: initiator sends first, responder replies.
                        // This eliminates the half-duplex collision where both sides
                        // send AFSK reports simultaneously and neither receives.
                        IRIS_LOG("[TUNE] Responder: test frames sent, waiting for initiator's report");
                        if (gui_log_) gui_log_("[TUNE] Waiting for peer report...");
                        tune_state_ = TuneState::WAIT_REPORT;
                        tune_report_resend_cd_ = 0;
                        tune_report_resends_ = 0;
                    }
                }
            } else if (ts == TuneState::WAIT_PEER) {
                // Accumulate gain measurements from peer's test frames.
                // Exit conditions (lockstep):
                //  1. Initiator: peer's AFSK ramp report received + local frames measured
                //     → respond immediately (3s silence guard). Don't wait for min_wait.
                //  2. Fallback: enough local frames + silence timeout (peer report late)
                tune_wait_peer_ticks_++;
                // Track silence: ticks since last measurement changed
                if (tune_frames_measured_ != tune_last_measured_count_) {
                    tune_last_measured_count_ = tune_frames_measured_;
                    tune_silence_ticks_ = 0;
                } else {
                    tune_silence_ticks_++;
                }

                // Check if peer's AFSK ramp report has arrived
                bool have_peer_report = false;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                    if (tune_peer_iters_[i] != -1) { have_peer_report = true; break; }
                if (!have_peer_report && tune_peer_gain_ > 0.01f)
                    have_peer_report = true;

                bool all_received = (tune_frames_measured_ >= tune_test_frames_target_);
                int silence_guard = 60;   // 3.0s silence before TX (avoid collision)

                // Lockstep path: peer's report arrived → respond ASAP.
                bool lockstep_ready = have_peer_report &&
                    tune_frames_measured_ >= 1 && tune_silence_ticks_ >= silence_guard;

                // Fallback path: got at least 1 frame + silence (peer ramp is over).
                // Don't require >= 3 frames — on FM, quieter ramp frames may be
                // below noise floor (OTA: only 2/5 frames detected at 26 dB range).
                // Silence guard is the structural exit; frame count is a sanity check.
                int min_wait = 160;       // 8s minimum (covers 5-frame ramp ~6s + margin)
                bool waited_enough = (tune_wait_peer_ticks_ >= min_wait);
                bool fallback_ready = tune_my_gain_ > 0 && waited_enough &&
                    tune_frames_measured_ >= 1 && tune_silence_ticks_ >= silence_guard;

                bool ready = lockstep_ready || fallback_ready;
                if (ready) {
                    IRIS_LOG("[TUNE] WAIT_PEER done: %d/%d frames measured, avg gain=%.4f "
                             "(waited %d ticks, silence %d, peer_report=%s)",
                             tune_frames_measured_, tune_test_frames_target_,
                             tune_my_gain_, tune_wait_peer_ticks_, tune_silence_ticks_,
                             have_peer_report ? "YES" : "no");
                    tune_last_measured_count_ = 0;
                    tune_silence_ticks_ = 0;
                    if (tune_is_initiator_) {
                        tune_state_ = TuneState::SEND_REPORT;
                    } else {
                        // Responder: TX our test frames first, then send report.
                        IRIS_LOG("[TUNE] Responder: sending test frames first, report after");
                        if (gui_log_) gui_log_("[TUNE] Sending test frames...");
                        tune_test_frames_sent_ = 0;
                        tune_state_ = TuneState::SEND_START;
                    }
                }
            } else if (ts == TuneState::SEND_REPORT) {
                // Send report as an OFDM frame (not AFSK — eliminates collision).
                // Wait for TX queue to drain first (same as SEND_START).
                if (tx_buffer_.empty() && ax25_tx_queue_.empty()) {
                    if (ofdm_phy_active_ && ofdm_mod_) {
                        auto report = tune_build_binary_report(
                            tune_rx_frame_iters_, tune_rx_frame_H_, tune_rx_frame_snr_,
                            TUNE_RAMP_COUNT);
                        int rpt_count = 0;
                        for (int ri = 0; ri < TUNE_RAMP_COUNT; ri++)
                            if (tune_rx_frame_iters_[ri] != -1) rpt_count++;
                        IRIS_LOG("[TUNE] Sending OFDM report frame (%d entries, %zu bytes)",
                                 rpt_count, report.size());
                        // Build single OFDM frame at safe tx_level
                        ToneMap tune_map = get_uniform_tone_map(1, ofdm_config_);
                        auto ofdm_iq = ofdm_mod_->build_ofdm_frame(
                            report.data(), report.size(), tune_map, LdpcRate::RATE_1_2);
                        if (!ofdm_iq.empty()) {
                            std::vector<float> audio(ofdm_iq.size());
                            for (size_t j = 0; j < ofdm_iq.size(); j++)
                                audio[j] = ofdm_iq[j].real();
                            // Normalize like ramp frames
                            int pre_samp = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
                            int dstart = std::min(pre_samp, (int)audio.size());
                            float ssq = 0.0f;
                            for (int k = dstart; k < (int)audio.size(); k++)
                                ssq += audio[k] * audio[k];
                            int dlen = (int)audio.size() - dstart;
                            float rms = (dlen > 0) ? std::sqrt(ssq / (float)dlen) : 0.0f;
                            float sc = (rms > 1e-6f) ? (0.50f / rms) : 0.1f;
                            for (int k = dstart; k < (int)audio.size(); k++)
                                audio[k] *= sc;
                            float ppeak = 0.0f;
                            for (int k = 0; k < dstart; k++)
                                ppeak = std::max(ppeak, std::abs(audio[k]));
                            if (ppeak > 0.90f) {
                                float ps = 0.90f / ppeak;
                                for (int k = 0; k < dstart; k++)
                                    audio[k] *= ps;
                            }
                            // TX at mid-range level (safe for both FM clipping and noise floor)
                            float report_tx = std::clamp(
                                (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level,
                                0.15f, 0.8f);
                            for (auto& s : audio) s *= report_tx;
                            for (auto& s : audio) s = std::clamp(s, -0.95f, 0.95f);
                            // Pre/post delay for radio settle
                            int pre_samples = config_.ptt_pre_delay_ms * config_.sample_rate / 1000;
                            int post_samples = std::max(config_.ptt_post_delay_ms, 100) * config_.sample_rate / 1000;
                            tx_buffer_.insert(tx_buffer_.end(), pre_samples, 0.0f);
                            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
                            tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);
                        }
                    } else {
                        // Non-OFDM fallback: AFSK report
                        char buf[64];
                        snprintf(buf, sizeof(buf), "TUNE:GAIN=%.4f", tune_my_gain_);
                        IRIS_LOG("[TUNE] Sending AFSK gain report: %.4f", tune_my_gain_);
                        send_tune_ui(buf);
                        send_tune_ui(buf);
                    }
                    // Fast-track: if peer report already received from embedded ramp payload
                    bool have_peer_ramp = false;
                    for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                        if (tune_peer_iters_[i] != -1) { have_peer_ramp = true; break; }
                    if (have_peer_ramp || tune_peer_gain_ > 0.01f) {
                        IRIS_LOG("[TUNE] Fast-track: peer report already received");
                        tune_state_ = TuneState::APPLY;
                    } else {
                        tune_state_ = TuneState::WAIT_REPORT;
                        tune_report_resend_cd_ = 0;
                        tune_report_resends_ = 0;
                    }
                }
            } else if (ts == TuneState::WAIT_REPORT) {
                // Waiting for peer's OFDM report frame.
                // Initiator: already sent its report via OFDM in SEND_REPORT.
                //   Normally fast-tracked to APPLY because responder's report was
                //   embedded in ramp frames. This state is fallback only.
                // Responder: waiting for initiator's dedicated OFDM report frame.
                //   Handled in OFDM RX path (payload with TUNE_REPORT_MAGIC → APPLY).
                // No AFSK resend logic needed — reports travel via OFDM.
            } else if (ts == TuneState::APPLY) {
                tune_apply_corrections();  // sets DONE
            }
            // (responder TX_TEST completion handled in TX_TEST branch above)
        }
    }
    // Reset DONE state after 5 seconds (100 ticks)
    if (tune_state_ == TuneState::DONE) {
        tune_timeout_--;
        if (tune_post_holdoff_ > 0)
            tune_post_holdoff_--;
        if (tune_timeout_ <= 0) {
            tune_state_ = TuneState::IDLE;
            tune_post_holdoff_ = 0;
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

    // Probe timeout: if probe doesn't complete in 35s, give up and stay AFSK.
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
                // Manual probe button: always initiator, generous capture window.
                probe_.start_initiator(config_.sample_rate, 25.0f);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("[PROBE] Manual probe: sending tones now (25s capture)");
            } else {
                // Auto-probe initiator: 25s capture. Responder needs 12s capture +
                // ~2s analysis + ~4s TX (result + 2.25s probe). Total ~18s.
                probe_.start_initiator(config_.sample_rate, 25.0f);
                ofdm_kiss_probing_ = true;
                IRIS_LOG("OFDM-KISS probe: initiator sending tones (25s capture)");
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
            // I-frames may have been sent during AFSK phase (before native_active_).
            // Start T1 now if any are unacked — otherwise T1 never starts and the
            // session hangs forever when OFDM fails (OTA bug 2026-03-22).
            ax25_session_.start_t1_if_unacked();

            // Initialize OFDM PHY if enabled (default).
            // Uses the negotiated passband from probe to configure OFDM carriers.
            if (config_.ofdm_enable) {
                NegotiatedPassband ofdm_pb;
                ofdm_pb.low_hz = low;
                ofdm_pb.high_hz = high;
                ofdm_pb.center_hz = center;
                ofdm_pb.bandwidth_hz = bandwidth;
                ofdm_pb.valid = true;

                // Negotiate OFDM PHY parameters from probe exchange.
                // Both sides advertise their config; we use conservative values
                // (max CP, min pilot spacings) so both sides compute identical config.
                // Old peers (v3 or earlier) have ofdm_* = 0, meaning "use defaults."
                int cp = config_.ofdm_cp_samples;
                int carrier_pilot_spacing = 4;  // default: 1:4 comb pilots
                int block_pilot_spacing = 24;   // default (was 14, widened for throughput)
                int nfft = config_.ofdm_nfft;
                {
                    const ProbeResult& peer = probe_.my_tx_result();  // peer's advertised config
                    if (peer.ofdm_cp_samples > 0) {
                        // Both sides advertised: use max CP (more conservative/robust)
                        cp = std::max(cp, (int)peer.ofdm_cp_samples);
                    }
                    if (peer.ofdm_pilot_carrier_spacing > 0) {
                        // Use min spacing (more pilots = more robust)
                        carrier_pilot_spacing = std::min(carrier_pilot_spacing, (int)peer.ofdm_pilot_carrier_spacing);
                    }
                    if (peer.ofdm_pilot_symbol_spacing > 0) {
                        block_pilot_spacing = std::min(block_pilot_spacing, (int)peer.ofdm_pilot_symbol_spacing);
                    }
                    {
                        // NFFT: both must match. Use minimum (most conservative).
                        // nfft_code: 0=512, 1=256, 2=1024
                        int peer_nfft = 512;
                        if (peer.ofdm_nfft_code == 1) peer_nfft = 256;
                        else if (peer.ofdm_nfft_code == 2) peer_nfft = 1024;
                        nfft = std::min(nfft, peer_nfft);
                    }
                    IRIS_LOG("[OFDM-NEG] peer config: cp=%d pilot=%d block=%d nfft_code=%d",
                             peer.ofdm_cp_samples, peer.ofdm_pilot_carrier_spacing,
                             peer.ofdm_pilot_symbol_spacing, peer.ofdm_nfft_code);
                    IRIS_LOG("[OFDM-NEG] negotiated: cp=%d pilot=%d block=%d nfft=%d",
                             cp, carrier_pilot_spacing, block_pilot_spacing, nfft);
                }

                // Auto-train pilot spacing: measure channel flatness from probe tones.
                // Only used when auto_spacing is enabled AND overrides the negotiated value.
                if (config_.ofdm_auto_spacing) {
                    const ProbeResult& rx_probe = probe_.their_tx_result();
                    float sum = 0, sum2 = 0;
                    int n_det = 0;
                    for (int t = 0; t < 64; t++) {
                        if (rx_probe.tone_detected[t]) {
                            sum += rx_probe.tone_power_db[t];
                            sum2 += rx_probe.tone_power_db[t] * rx_probe.tone_power_db[t];
                            n_det++;
                        }
                    }
                    if (n_det > 2) {
                        float mean = sum / n_det;
                        float var = sum2 / n_det - mean * mean;
                        float std_db = std::sqrt(std::max(0.0f, var));
                        if (std_db < 3.0f) {
                            carrier_pilot_spacing = 12;
                            block_pilot_spacing = 24;
                        } else if (std_db > 8.0f) {
                            carrier_pilot_spacing = 4;
                            block_pilot_spacing = 8;
                        }
                        IRIS_LOG("[OFDM-AUTO] probe flatness: std=%.1f dB → pilot spacing 1:%d (carrier) 1:%d (block)",
                                 std_db, carrier_pilot_spacing, block_pilot_spacing);
                    }
                }
                ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                                       carrier_pilot_spacing, block_pilot_spacing);

                // Minimum pilot check: ensure at least 4 pilots for reliable channel estimation
                {
                    int n_used = ofdm_config_.n_used_carriers;
                    int n_pilots = n_used / carrier_pilot_spacing + 1;
                    if (n_pilots < 4 && carrier_pilot_spacing > 2) {
                        carrier_pilot_spacing = std::max(2, n_used / 3);  // force at least 4 pilots
                        IRIS_LOG("[OFDM] pilot spacing reduced to 1:%d for minimum 4 pilots", carrier_pilot_spacing);
                        ofdm_config_ = ofdm_config_from_probe(ofdm_pb, config_.ofdm_nfft, cp,
                                                               carrier_pilot_spacing, block_pilot_spacing);
                    }
                }

                ofdm_config_.fm_preemph_corner_hz = config_.ofdm_preemph_corner_hz;
                ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
                ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
                // Don't set ofdm_phy_active_ yet — wait for CAP_OFDM negotiation below
                ofdm_rx_iq_.clear();
                ofdm_rx_audio_buf_.clear();
                ofdm_rx_lpf_.reset();
                ofdm_sync_cached_ = false;

                // Initialize gearshift OFDM level at O0 (BPSK r1/2) — most robust.
                // Gearshift will negotiate up from RX SNR + LDPC convergence.
                gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
                ofdm_speed_level_ = 0;
                gearshift_.force_ofdm_level(0);
                ofdm_tone_map_ = get_uniform_tone_map(
                    1, ofdm_config_);  // preset 1 = BPSK r1/2 (O0, start at most robust)
                ofdm_tone_map_.use_nuc = config_.ofdm_nuc;

                IRIS_LOG("OFDM PHY: prepared, %d carriers (%d data, %d pilot), CP=%d, BW=%.0f Hz",
                         ofdm_config_.n_used_carriers, ofdm_config_.n_data_carriers,
                         ofdm_config_.n_pilot_carriers, ofdm_config_.cp_samples,
                         ofdm_config_.bandwidth_hz);
            }

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

                        // Safety: if actual LZHUF payload bytes were already sent
                        // as-is during AFSK, we can't start unrolling mid-stream
                        // (remote would get LZHUF + plaintext mix = corrupt).
                        // But if only FS was parsed (no data bytes consumed yet),
                        // it's safe to start intercepting from here.
                        if (ofdm_kiss_b2f_.has_payload_data_in_flight()) {
                            IRIS_LOG("OFDM-KISS: B2F payload bytes already sent during AFSK — "
                                     "disabling unroll for this session");
                            ofdm_kiss_b2f_.deinit();
                            ofdm_kiss_peer_caps_ &= ~CAP_B2F_UNROLL;
                        } else if (ofdm_kiss_b2f_.is_payload_transfer()) {
                            IRIS_LOG("OFDM-KISS: B2F in PAYLOAD_TRANSFER but no data consumed yet — "
                                     "interception safe, keeping unroll enabled");
                        }
                    }
                }
            }

            // Activate OFDM PHY only if both sides negotiated CAP_OFDM.
            // Objects were prepared above; this gate ensures fallback to legacy
            // single-carrier when the peer doesn't support OFDM.
            if (config_.ofdm_enable && ofdm_mod_ && (ofdm_kiss_peer_caps_ & CAP_OFDM)) {
                ofdm_phy_active_ = true;
                // Limit AX.25 I-frame info to fit OFDM frame capacity.
                // Capacity = (k_bytes - 6 overhead) * codewords_per_frame - 16 AX.25 hdr.
                // At O0 BPSK r1/2: (100-6)*2 - 16 = 172 bytes.
                // Use worst-case (r1/2) so I-frames always fit regardless of speed level.
                int ofdm_max_info = (LdpcCodec::block_size(LdpcRate::RATE_1_2) / 8 - 6) * 2 - 16;
                ax25_session_.set_max_info(ofdm_max_info);
                IRIS_LOG("OFDM PHY: ACTIVE (CAP_OFDM negotiated, %d data carriers, BW=%.0f Hz)",
                         ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz);
                if (gui_log_) {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "OFDM PHY: %d carriers, BW=%.0f Hz (%.0f bps ceiling)",
                             ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz,
                             tone_map_throughput(ofdm_tone_map_, ofdm_config_, 50));
                    gui_log_(buf);
                }
            } else if (config_.ofdm_enable && ofdm_mod_) {
                IRIS_LOG("OFDM PHY: peer lacks CAP_OFDM — using legacy single-carrier PHY");
                ofdm_mod_.reset();
                ofdm_demod_.reset();
            }

            // Migrate I-frames from ax25_tx_queue_ (AFSK) to tx_queue_ (native).
            // During the probe, ofdm_kiss_tx_ was false so the send_frame_
            // callback routed I-frames to ax25_tx_queue_.  Now that native mode
            // is active, move them so they go out as native frames — not as a
            // long AFSK burst that would bury the probe result UI frame.
            {
                int migrated = 0;
                std::queue<std::vector<uint8_t>> keep;
                while (!ax25_tx_queue_.empty()) {
                    auto frame = std::move(ax25_tx_queue_.front());
                    ax25_tx_queue_.pop();
                    bool is_iframe = frame.size() > 14 && (frame[14] & 0x01) == 0;
                    if (is_iframe) {
                        constexpr size_t TX_QUEUE_MAX = 32;
                        if (tx_queue_.size() >= TX_QUEUE_MAX) {
                            IRIS_LOG("[TX] queue full (%zu frames), dropping oldest", tx_queue_.size());
                            tx_queue_.pop();
                        }
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
            csma_holdoff_ = config_.sample_rate * 3;  // 3s listen before first TX

            // Connection already established before probe.
            // Held I-frames release via native after listen window expires.
            // T1 is already set to 2.0s by set_native_active(true) above —
            // don't override with the AFSK-era 15s value.
            probe_connect_timeout_ = 0;
            IRIS_LOG("Probe done — releasing held I-frames via native");

            // Cache probe result for this peer (24h expiry, skip re-probe on reconnect)
            cache_probe_result(ax25_session_.remote_callsign());

            // Adaptive TXDELAY: after successful probe, reduce OFDM TX pre-delay.
            // Probe success proves radio link works; training symbols handle the rest.
            ofdm_txdelay_ms_ = std::max(50, config_.ptt_pre_delay_ms / 2);
            IRIS_LOG("[TXDELAY] adaptive: %d ms -> %d ms (post-probe)",
                     config_.ptt_pre_delay_ms, ofdm_txdelay_ms_);

            // Pre-calibrate OFDM TX level from probe data.
            //
            // Problem: TUNE needs OFDM frames to decode, but OFDM frames can't
            // decode if tx_level is wrong (too loud → FM clipping destroys ZC
            // preamble). Chicken-and-egg.
            //
            // Solution: probe tones already traversed the channel successfully.
            // Both probe and OFDM go through the same FM audio path, so the path
            // gain cancels — we just need to match output levels with a PAPR
            // correction. TUNE will refine from this starting point.
            //
            // Probe output: 64 continuous sinusoids, PAPR ~3 dB
            //   Total RMS ≈ amplitude/sqrt(2) * tx_level ≈ 0.354 * tx_level
            //   Peak ≈ RMS * 10^(3/20) = 0.354 * tx_level * 1.41
            //
            // OFDM output: burst symbols, PAPR ~8 dB after clipper
            //   Total RMS = 0.50 * ofdm_tx_level
            //   Peak ≈ RMS * 10^(8/20) = 0.50 * ofdm_tx_level * 2.51
            //
            // Target: OFDM peaks ≤ probe peaks (probe didn't overdrive FM limiter)
            //   0.50 * ofdm_tx * 2.51 = 0.354 * tx_level * 1.41
            //   ofdm_tx = 0.354 * 1.41 / (0.50 * 2.51) * tx_level ≈ 0.40 * tx_level
            {
                float ofdm_tx = 0.40f * config_.tx_level;
                ofdm_tx = std::clamp(ofdm_tx, 0.05f, 0.50f);
                ofdm_tx_base_ = ofdm_tx;

                // Log peer's measured tone levels for diagnostics (path gain info)
                const ProbeResult& peer_heard = probe_.my_tx_result();
                float avg_rx_db = -99.0f;
                int count = 0;
                if (peer_heard.valid && peer_heard.tones_detected > 0) {
                    float sum_rx_db = 0;
                    for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                        if (!peer_heard.tone_detected[k]) continue;
                        float freq = probe_tone_freq(k);
                        if (freq < low - 25.0f || freq > high + 25.0f) continue;
                        sum_rx_db += peer_heard.tone_power_db[k];
                        count++;
                    }
                    if (count > 0) avg_rx_db = sum_rx_db / count;
                }

                IRIS_LOG("[PROBE-CAL] OFDM tx_level: %.3f (from %.3f, ratio=0.40, PAPR safety 5 dB)",
                         ofdm_tx, config_.tx_level);
                IRIS_LOG("[PROBE-CAL]   peer avg tone power: %.1f dBFS (%d tones in band)",
                         avg_rx_db, count);
            }

            // Configure OFDM RX lowpass filter to remove f² discriminator noise.
            // Flat 9600-baud radio ports output raw discriminator with PSD ∝ f².
            // Wideband noise (3-12+ kHz) crushes SC/ZC broadband correlation.
            // 2nd-order Butterworth LPF: ~3 samples GD at cutoff (CP=64 gives huge margin),
            // -12 dB/oct rolloff, -22 dB at 12 kHz.
            {
                float fc = high + 500.0f;  // 500 Hz margin above negotiated band edge
                fc = std::clamp(fc, 2000.0f, 5000.0f);
                const float fs = 48000.0f;
                const float Q = 0.7071f;  // Butterworth
                float w0 = 2.0f * (float)M_PI * fc / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q);
                float a0 = 1.0f + alpha;
                ofdm_rx_lpf_ = {};
                ofdm_rx_lpf_.b0 = ((1.0f - c) / 2.0f) / a0;
                ofdm_rx_lpf_.b1 = (1.0f - c) / a0;
                ofdm_rx_lpf_.b2 = ((1.0f - c) / 2.0f) / a0;
                ofdm_rx_lpf_.a1 = (-2.0f * c) / a0;
                ofdm_rx_lpf_.a2 = (1.0f - alpha) / a0;
                ofdm_rx_lpf_active_ = true;
                IRIS_LOG("[OFDM-LPF] RX lowpass configured: fc=%.0f Hz (band edge %.0f + 500 Hz), 2nd-order Butterworth",
                         fc, high);
            }

            // Auto-tune after probe: gain characteristics change with bandwidth/baud,
            // so recalibrate TX level for the newly discovered passband.
            // Only the probe initiator triggers auto-tune — the responder will
            // enter responder mode when it receives TUNE:START from us.
            // TUNE:START frames queue behind csma_holdoff_ (3s listen window above),
            // and the SEND_START tick handler waits for drain before sending test frames.
            if (ax25_session_.we_initiated()) {
                std::string peer = ax25_session_.remote_callsign();
                if (!peer.empty()) {
                    IRIS_LOG("[TUNE] Triggering auto-tune after probe (peer=%s)", peer.c_str());
                    start_autotune(peer);
                }
            }
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    ModemDiag diag;
    diag.state = state_;
    diag.speed_level = gearshift_.current_level();
    diag.ofdm_speed_level = gearshift_.current_ofdm_level();
    diag.ofdm_active = ofdm_phy_active_;
    diag.snr_db = gearshift_.smoothed_snr();
    diag.agc_gain = agc_.gain();
    diag.tx_level = config_.tx_level;
    diag.native_rx_gain = native_rx_gain_;
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

    diag.constellation = last_constellation_;
    diag.kalman_trace = last_kalman_trace_;
    diag.spectrum = last_spectrum_;

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
    //   15.6 Hz bins, 256 bins covering 0-4 kHz (pocketfft O(N log N)).
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

    // Hann window + FFT (O(N log N) via pocketfft, replaces O(N²) naive DFT)
    int n_pos = NFFT / 2;
    std::complex<float> fft_buf[NFFT];
    for (int n = 0; n < NFFT; n++) {
        float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * n / (NFFT - 1)));
        fft_buf[n] = std::complex<float>(ds[n] * w, 0.0f);
    }
    fft_complex(fft_buf, NFFT);

    std::vector<float> spec(n_pos, 0.0f);
    for (int k = 0; k < n_pos; k++) {
        float pwr = std::norm(fft_buf[k]) / (float)(NFFT * NFFT);
        spec[k] = 10.0f * std::log10(std::max(pwr, 1e-12f));
    }

    // Caller (process_rx) holds modem_mutex_
    last_spectrum_ = std::move(spec);
    spectrum_low_hz_ = 0;
    spectrum_high_hz_ = DS_RATE / 2.0f;
}

// --- Auto-Tune (bilateral native-frame gain calibration) ---
// Protocol (half-duplex safe — reports sent only when peer is listening):
//   Initiator: clicks Auto Tune → sends TUNE:START UI → TX test frame(s) →
//              waits for responder's test frame(s) → sends report → waits → DONE.
//   Responder: receives TUNE:START → waits for initiator's test frame(s) →
//              TX own test frame(s) → sends report → waits for initiator's report → DONE.
//   Both reports are sent AFTER all TX is complete, avoiding half-duplex collision.
//
// When OFDM is active, test frames are OFDM power-ramp frames (BPSK r1/2) at
// varying TX levels. Peer measures LDPC quality + |H| for each; parabolic fit
// finds optimal drive. Legacy native mode uses single-level gain targeting.

void Modem::send_tune_ui(const char* payload) {
    // Caller holds modem_mutex_
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("TUNE");
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    size_t plen = strlen(payload);
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + plen);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

void Modem::tune_build_and_queue_test_frame() {
    // Build N test frames with known payload into tx_buffer_.
    // When OFDM is active, build OFDM frames (Schmidl-Cox preamble) so the
    // peer's OFDM RX can detect them. Otherwise, build native Mode A frames.
    // Caller must hold modem_mutex_.
    //
    // Responder embeds its measurements of the initiator's ramp frames in the
    // payload of each ramp frame (binary report). This eliminates the AFSK
    // report exchange — the initiator gets the report by decoding the ramp.
    const uint8_t default_payload[] = "TUNE_TEST_FRAME";
    std::vector<uint8_t> report_payload;
    if (!tune_is_initiator_ && tune_frames_measured_ > 0) {
        report_payload = tune_build_binary_report(
            tune_rx_frame_iters_, tune_rx_frame_H_, tune_rx_frame_snr_,
            TUNE_RAMP_COUNT);
        IRIS_LOG("[TUNE] Responder: embedding %d-frame report in ramp payload (%zu bytes)",
                 tune_frames_measured_, report_payload.size());
    }
    const uint8_t* payload_data = report_payload.empty()
        ? default_payload : report_payload.data();
    size_t payload_len = report_payload.empty()
        ? sizeof(default_payload) - 1 : report_payload.size();

    tx_buffer_.clear();

    for (int i = 0; i < tune_test_frames_target_; i++) {
        if (ofdm_phy_active_ && ofdm_mod_) {
            // OFDM power-ramp TUNE: each of the N frames is sent at a different
            // TX level. Peer measures LDPC quality for each and reports which
            // one decoded best. This finds the optimal drive level in a single
            // exchange (5 frames via TUNE_RAMP_COUNT).
            // BPSK r1/2 for TUNE: maximum robustness for TX power calibration.
            // Explicit r1/2 overrides the preset's FEC rate.
            ToneMap tune_map = get_uniform_tone_map(1, ofdm_config_);
            auto ofdm_iq = ofdm_mod_->build_ofdm_frame(
                payload_data, payload_len, tune_map, LdpcRate::RATE_1_2);
            if (ofdm_iq.empty()) {
                IRIS_LOG("[TUNE] Failed to build OFDM test frame %d", i + 1);
                continue;
            }
            // Extract real passband
            std::vector<float> audio(ofdm_iq.size());
            for (size_t j = 0; j < ofdm_iq.size(); j++)
                audio[j] = ofdm_iq[j].real();
            // Normalize data portion to 0.50 RMS, preamble peak to ±0.90
            int pre_samp = 2 * (ofdm_config_.nfft + ofdm_config_.cp_samples);
            int dstart = std::min(pre_samp, (int)audio.size());
            float ssq = 0.0f;
            for (int k = dstart; k < (int)audio.size(); k++)
                ssq += audio[k] * audio[k];
            int dlen = (int)audio.size() - dstart;
            float rms = (dlen > 0) ? std::sqrt(ssq / (float)dlen) : 0.0f;
            float sc = (rms > 1e-6f) ? (0.50f / rms) : 0.1f;
            for (int k = dstart; k < (int)audio.size(); k++)
                audio[k] *= sc;
            float ppeak = 0.0f;
            for (int k = 0; k < dstart; k++)
                ppeak = std::max(ppeak, std::abs(audio[k]));
            if (ppeak > 0.90f) {
                float ps = 0.90f / ppeak;
                for (int k = 0; k < dstart; k++)
                    audio[k] *= ps;
            }

            // Apply power-ramp level for this frame: scales × base.
            // Use probe-calibrated ofdm_tx_base_ if available, else config_.tx_level.
            // Record actual tx_level used so parabolic fit can map back to levels.
            float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
            float ramp_scale = (i < TUNE_RAMP_COUNT) ? tune_computed_scales_[i] : 1.0f;
            float frame_tx = std::clamp(ramp_base * ramp_scale, 0.05f, 1.0f);
            if (i < TUNE_RAMP_COUNT) tune_ramp_tx_levels_[i] = frame_tx;
            for (auto& s : audio) s *= frame_tx;

            // Hard clip to soundcard range
            for (auto& s : audio) {
                if (s > 0.95f) s = 0.95f;
                else if (s < -0.95f) s = -0.95f;
            }
            IRIS_LOG("[TUNE] OFDM ramp frame %d: scale=%.2f tx=%.3f (base=%.3f) %zu samples",
                     i + 1, ramp_scale, frame_tx, ramp_base, audio.size());
            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
            tune_test_frames_sent_++;
        } else {
            // Native Mode A TUNE frame
            auto iq = build_native_frame(payload_data, payload_len,
                                          phy_config_, LdpcRate::RATE_1_2);
            if (iq.empty()) {
                IRIS_LOG("[TUNE] Failed to build native test frame %d", i + 1);
                continue;
            }

            std::vector<float> audio;
            if (use_upconvert_) {
                audio = upconverter_.iq_to_audio(iq.data(), iq.size());
                if (tx_channel_eq_.is_configured()) {
                    tx_channel_eq_.apply(audio.data(), (int)audio.size());
                    for (auto& s : audio) {
                        if (s > 0.95f) s = 0.95f + 0.05f * std::tanh((s - 0.95f) / 0.05f);
                        else if (s < -0.95f) s = -0.95f + 0.05f * std::tanh((s + 0.95f) / 0.05f);
                    }
                }
            } else {
                audio.resize(iq.size() / 2);
                for (size_t j = 0; j < iq.size() / 2; j++)
                    audio[j] = iq[2 * j];
            }
            tx_buffer_.insert(tx_buffer_.end(), audio.begin(), audio.end());
            tune_test_frames_sent_++;
        }
    }

    // Apply tx_level to native TUNE frames only.
    // OFDM ramp frames already have per-frame tx_level baked in above.
    if (!(ofdm_phy_active_ && ofdm_mod_)) {
        for (auto& s : tx_buffer_) s *= config_.tx_level;
    }

    // TUNE uses OFDM frames — need full TXDELAY for radio settle
    int pre_ms = config_.ptt_pre_delay_ms;
    int pre_samples = pre_ms * config_.sample_rate / 1000;
    if (pre_samples > 0)
        tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);
    int post_ms = std::max(config_.ptt_post_delay_ms, 100);  // min 100ms tail
    int post_samples = post_ms * config_.sample_rate / 1000;
    if (post_samples > 0)
        tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

    tx_pos_ = 0;
    state_ = ModemState::TX_NATIVE;
    frames_tx_++;

    IRIS_LOG("[TUNE] Built %d test frames, %zu samples (%d ms, pre=%dms post=%dms)",
             tune_test_frames_sent_, tx_buffer_.size(),
             (int)(tx_buffer_.size() * 1000 / config_.sample_rate),
             pre_ms, post_ms);
}

void Modem::handle_tune_frame(const uint8_t* info, size_t len) {
    std::string payload((const char*)info, len);

    if (payload.find("TUNE:START") != std::string::npos) {
        // Remote (initiator) is starting or restarting tune — enter responder mode.
        // Accept in IDLE or any active TUNE state (initiator may be retrying with
        // more frames, which resets our state machine).
        if (tune_state_ == TuneState::IDLE ||
            (tune_state_ != TuneState::DONE && !tune_is_initiator_)) {
            if (tune_state_ != TuneState::IDLE)
                IRIS_LOG("[TUNE] Received TUNE:START while in state %d — resetting as responder",
                         (int)tune_state_.load());
            else
                IRIS_LOG("[TUNE] Received TUNE:START — entering responder mode");
            if (gui_log_) gui_log_("[TUNE] Remote requested auto-tune");
            tune_audit("=== TUNE START === local=%s role=responder tx_level=%.3f rx_gain=%.3f",
                       config_.callsign.c_str(), config_.tx_level, native_rx_gain_);
            tune_state_ = TuneState::WAIT_PEER;
            tune_is_initiator_ = false;
            tune_peer_call_ = ax25_session_.remote_callsign();
            tune_my_gain_ = 0;
            tune_peer_gain_ = 0;
            tune_test_frames_sent_ = 0;
            tune_frames_measured_ = 0;
            tune_wait_peer_ticks_ = 0;
            tune_last_measured_count_ = 0;
            tune_silence_ticks_ = 0;
            // Parse ramp count from "TUNE:START=N" (default 10 for compat)
            int rc = TUNE_RAMP_COUNT;
            {
                size_t eq = payload.find("TUNE:START=");
                if (eq != std::string::npos) {
                    int parsed = atoi(payload.c_str() + eq + 11);
                    if (parsed >= 1 && parsed <= TUNE_RAMP_COUNT) rc = parsed;
                }
            }
            tune_test_frames_target_ = rc;
            // Compute adaptive ramp scales based on current TX base
            {
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                tune_compute_scales(ramp_base);
            }
            // Responder timeout: generous — must cover initiator TX + our TX +
            // 2s stagger delay + reports + retransmits.
            tune_timeout_ = 800;  // 40s at 50ms/tick
            IRIS_LOG("[TUNE] Ramp count: %d, timeout: 40s", rc);
            for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                tune_rx_frame_iters_[i] = -1;
                tune_rx_frame_H_[i] = 0;
                tune_rx_frame_snr_[i] = -99.0f;
                tune_peer_iters_[i] = -1;
                tune_peer_H_[i] = 0;
                tune_peer_snr_[i] = -99.0f;
                tune_ramp_tx_levels_[i] = 0;
            }
        }
    } else if (payload.find("TUNE:R=") != std::string::npos ||
               payload.find("TUNE:RAMP") != std::string::npos) {
        // OFDM power-ramp report. Two formats:
        //   Compact: "TUNE:R=idx:iters:H,idx:iters:H,..."  (only measured entries)
        //   Legacy:  "TUNE:RAMPN=i0,h0,i1,h1,...,iN-1,hN-1" (all N entries)
        int parsed_count = 0;
        size_t cpos = payload.find("TUNE:R=");
        if (cpos != std::string::npos) {
            // Compact format: parse idx:iters:H[:SNR] (3 or 4 fields)
            const char* cp = payload.c_str() + cpos + 7;  // skip "TUNE:R="
            while (*cp) {
                int idx, iters;
                float h, snr = -99.0f;
                int nf = sscanf(cp, "%d:%d:%f:%f", &idx, &iters, &h, &snr);
                if (nf < 3) break;  // need at least idx:iters:H
                if (idx >= 0 && idx < TUNE_RAMP_COUNT) {
                    tune_peer_iters_[idx] = iters;
                    tune_peer_H_[idx] = h;
                    tune_peer_snr_[idx] = (nf >= 4) ? snr : -99.0f;
                    parsed_count++;
                }
                cp = strchr(cp, ',');
                if (cp) cp++; else break;
            }
        } else {
            // Legacy RAMPN format
            size_t rpos = payload.find("TUNE:RAMP");
            const char* p = payload.c_str() + rpos + 9;
            int peer_ramp_count = atoi(p);
            const char* eq = strchr(p, '=');
            if (eq && peer_ramp_count >= 1 && peer_ramp_count <= TUNE_RAMP_COUNT) {
                const char* cp = eq + 1;
                for (int i = 0; i < peer_ramp_count; i++) {
                    int iters; float h;
                    if (sscanf(cp, "%d,%f", &iters, &h) < 2) break;
                    tune_peer_iters_[i] = iters;
                    tune_peer_H_[i] = h;
                    parsed_count++;
                    for (int skip = 0; skip < 2 && *cp; skip++) {
                        cp = strchr(cp, ',');
                        if (cp) cp++; else break;
                    }
                }
            }
        }
        if (parsed_count == 0) {
            IRIS_LOG("[TUNE] Invalid RAMP report format");
            return;
        }
        IRIS_LOG("[TUNE] Peer ramp report: %d entries (state=%d)",
                 parsed_count, (int)tune_state_.load());
        tune_audit("RAMP_REPORT entries=%d state=%d",
                   parsed_count, (int)tune_state_.load());

        // AFSK report fallback (non-OFDM mode only; OFDM uses embedded binary reports)
        if (tune_state_ == TuneState::WAIT_REPORT) {
            tune_state_ = TuneState::APPLY;
        } else if (tune_state_ == TuneState::WAIT_PEER) {
            IRIS_LOG("[TUNE] Stored peer ramp report (still measuring their frames)");
        }
    } else if (payload.find("TUNE:GAIN=") != std::string::npos) {
        // Legacy/Mode A gain report. Also serves as OFDM fallback.
        size_t pos = payload.find("TUNE:GAIN=");
        float remote_gain = 0;
        try {
            remote_gain = std::stof(payload.substr(pos + 10));
        } catch (...) {
            IRIS_LOG("[TUNE] Invalid gain format in payload");
            return;
        }
        if (remote_gain <= 0.01f || remote_gain > 10.0f) {
            IRIS_LOG("[TUNE] Gain out of range: %.4f", remote_gain);
            return;
        }
        IRIS_LOG("[TUNE] Peer reports gain=%.4f from our test frames (state=%d)",
                 remote_gain, (int)tune_state_.load());
        tune_audit("PEER_REPORT peer_gain=%.4f my_gain=%.4f state=%d",
                   remote_gain, tune_my_gain_, (int)tune_state_.load());
        tune_peer_gain_ = remote_gain;

        // AFSK report fallback (non-OFDM mode only)
        if (tune_state_ == TuneState::WAIT_REPORT) {
            tune_state_ = TuneState::APPLY;
        } else if (tune_state_ == TuneState::WAIT_PEER) {
            IRIS_LOG("[TUNE] Stored peer gain (still measuring their frames)");
        }
    }
}

std::string Modem::tune_build_ramp_report() const {
    // Compact report: only include measured entries (iters != -1).
    // Format: "TUNE:R=idx:iters:H:SNR,idx:iters:H:SNR,..."
    // Example: "TUNE:R=0:-2:3.37:2.8,1:-2:3.38:3.1" (~50 bytes)
    // Backward-compatible: old parsers ignore the 4th field, new parsers accept 3 or 4 fields.
    char buf[192];
    int pos = snprintf(buf, sizeof(buf), "TUNE:R=");
    bool first = true;
    for (int i = 0; i < TUNE_RAMP_COUNT && pos < (int)sizeof(buf) - 24; i++) {
        if (tune_rx_frame_iters_[i] == -1) continue;  // not measured
        if (!first) buf[pos++] = ',';
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%d:%d:%.2f:%.1f",
                        i, tune_rx_frame_iters_[i], tune_rx_frame_H_[i],
                        tune_rx_frame_snr_[i]);
        first = false;
    }
    return std::string(buf);
}

// Static constexpr member definitions (C++14 ODR-use)
constexpr float Modem::ofdm_level_offset_db_[10];

void Modem::tune_compute_scales(float base) {
    // Compute TUNE_RAMP_COUNT distinct scales so that base*scale spans
    // [TUNE_SCALE_MIN .. TUNE_SCALE_MAX] uniformly in dB.
    if (base < 0.001f) base = 0.1f;
    float abs_min = TUNE_SCALE_MIN;
    float abs_max = TUNE_SCALE_MAX;
    // Scale range: base*scale_lo = abs_min, base*scale_hi = abs_max
    float scale_lo = abs_min / base;  // may be < 1
    float scale_hi = abs_max / base;  // may be > 1
    // Clamp to physically meaningful range
    if (scale_lo < 0.01f) scale_lo = 0.01f;
    if (scale_hi > 20.0f) scale_hi = 20.0f;
    if (scale_lo >= scale_hi) {
        // Degenerate: base is at or beyond limits, spread around 1.0
        scale_lo = 0.5f;
        scale_hi = 2.0f;
    }
    float db_lo = 20.0f * std::log10(scale_lo);
    float db_hi = 20.0f * std::log10(scale_hi);
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        float db = db_hi - (db_hi - db_lo) * i / (float)(TUNE_RAMP_COUNT - 1);
        tune_computed_scales_[i] = std::pow(10.0f, db / 20.0f);
    }
    IRIS_LOG("[TUNE] Computed %d ramp scales for base=%.3f: [%.3f .. %.3f] (%.1f dB range)",
             TUNE_RAMP_COUNT, base, tune_computed_scales_[TUNE_RAMP_COUNT - 1],
             tune_computed_scales_[0], db_hi - db_lo);
}

float Modem::tune_fit_tx_level() const {
    // SNR-based parabolic fit for TX level optimization.
    // Fit (tx_dB, -SNR) parabola; minimum of -SNR = maximum SNR = optimal TX.
    //
    // FM-specific considerations:
    //   - FM deviation limiter makes |H| invariant across TX levels → H-based fit useless
    //   - Preamble-only SNR (from SC metric on failed frames) is unreliable: it measures
    //     ZC preamble correlation, not data quality. Can be 5-8 dB off from true SNR.
    //   - When ALL frames fail LDPC, we have ZERO reliable SNR measurements.
    //   - Navalekar (2019) optimal FM OFDM backoff: 0.65-0.88 of deviation limiter threshold.

    // Count LDPC-decoded vs preamble-only measurements
    int n_ldpc = 0, n_preamble = 0;
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        if (tune_peer_iters_[i] >= 0) n_ldpc++;
        else if (tune_peer_iters_[i] == -2) n_preamble++;
    }

    // CRITICAL FIX: If zero frames decoded, preamble-only SNR is unreliable.
    // Don't run a parabolic fit on garbage data — it produces absurd results
    // (e.g., tx_level 0.883→0.141, making the station inaudible).
    // Fall back to a conservative FM-appropriate level.
    if (n_ldpc == 0) {
        // No LDPC convergence at any TX level. Two scenarios:
        // 1. FM channel: use 0.75× max ramp level (Navalekar optimal backoff region)
        // 2. Linear channel: keep current level (problem is likely not TX power)
        //
        // Detect FM: |H| nearly invariant across measurements (max/min < 2.0)
        float h_min = 1e9f, h_max = -1e9f;
        int h_count = 0;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
            if (tune_peer_iters_[i] != -1 && tune_peer_H_[i] > 0.01f) {
                h_min = std::min(h_min, tune_peer_H_[i]);
                h_max = std::max(h_max, tune_peer_H_[i]);
                h_count++;
            }
        }
        bool fm_channel = (h_count >= 3 && h_max / h_min < 2.0f);

        if (fm_channel) {
            // FM: use 75% of maximum ramp level (in Navalekar 0.65-0.88 optimal zone).
            // This is conservative enough to avoid clipping but loud enough to be heard.
            float tx_max = 0.0f;
            for (int i = 0; i < TUNE_RAMP_COUNT; i++)
                tx_max = std::max(tx_max, tune_ramp_tx_levels_[i]);
            float tx_fm = 0.75f * tx_max;
            IRIS_LOG("[TUNE] SNR fit: ALL %d frames failed LDPC, FM channel detected "
                     "(H range %.2f-%.2f, ratio %.1f). Using 0.75×max = %.4f",
                     n_preamble, h_min, h_max, h_max / h_min, tx_fm);
            return std::clamp(tx_fm, 0.05f, 1.0f);
        } else {
            // Linear or unknown: keep current level, don't make things worse
            IRIS_LOG("[TUNE] SNR fit: ALL %d frames failed LDPC, non-FM channel. "
                     "Keeping current tx_level=%.4f", n_preamble, config_.tx_level);
            return config_.tx_level;
        }
    }

    // Detect FM channel: |H| nearly invariant across DECODED frames.
    // Only consider frames that decoded (iters >= 0) — preamble-only frames
    // at extreme TX levels can have very different H (below limiter threshold
    // at low TX, clipped at high TX) even on FM channels.
    float h_min = 1e9f, h_max = -1e9f;
    int h_count = 0;
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        if (tune_peer_iters_[i] >= 0 && tune_peer_H_[i] > 0.01f) {
            h_min = std::min(h_min, tune_peer_H_[i]);
            h_max = std::max(h_max, tune_peer_H_[i]);
            h_count++;
        }
    }
    bool fm_channel = (h_count >= 3 && h_max / h_min < 3.0f);

    // FM channel strategy: find the clipping edge, back off one step.
    // The FM deviation limiter creates a plateau — any level below the
    // clipping point works roughly equally well. We don't need the optimal
    // level, we need a SAFE level. Find the highest TX level where a frame
    // decoded (clipping edge), then use the next lower ramp level.
    if (fm_channel) {
        // Ramp frames are ordered highest TX first (index 0 = loudest).
        // Find the first (highest-TX) index that decoded.
        int clip_edge_idx = -1;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
            if (tune_peer_iters_[i] >= 0) {
                clip_edge_idx = i;
                break;
            }
        }

        // Back off one step from the clipping edge
        int use_idx = (clip_edge_idx >= 0) ? std::min(clip_edge_idx + 1, TUNE_RAMP_COUNT - 1) : 0;
        // If the backed-off index didn't decode, use the edge itself
        if (tune_peer_iters_[use_idx] < 0 && clip_edge_idx >= 0)
            use_idx = clip_edge_idx;

        float tx_opt = tune_ramp_tx_levels_[use_idx];

        IRIS_LOG("[TUNE] FM channel (H ratio %.1f): clip edge at ramp[%d] tx=%.3f, "
                 "backed off to ramp[%d] tx=%.3f",
                 h_max / h_min, clip_edge_idx,
                 clip_edge_idx >= 0 ? tune_ramp_tx_levels_[clip_edge_idx] : 0.0f,
                 use_idx, tx_opt);
        return std::clamp(tx_opt, 0.05f, 1.0f);
    }

    // Linear channel: use SNR-based parabolic fit
    float sx[TUNE_RAMP_COUNT], sy[TUNE_RAMP_COUNT];
    int n_snr = 0;
    bool using_preamble_pts = (n_ldpc < 3);

    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        float tx = tune_ramp_tx_levels_[i];
        if (tx < 0.001f) continue;
        if (tune_peer_snr_[i] > -90.0f) {
            if (tune_peer_iters_[i] >= 0 || using_preamble_pts) {
                sx[n_snr] = 20.0f * std::log10(tx);
                sy[n_snr] = -tune_peer_snr_[i];
                n_snr++;
            }
        }
    }

    IRIS_LOG("[TUNE] Linear channel: %d LDPC + %d preamble-only, using %d points",
             n_ldpc, n_preamble, n_snr);

    if (n_snr == 0) {
        IRIS_LOG("[TUNE] SNR fit: no valid data points");
        return config_.tx_level;
    }

    if (n_snr < 3) {
        int best = 0;
        for (int i = 1; i < n_snr; i++) if (sy[i] < sy[best]) best = i;
        float tx_opt = std::pow(10.0f, sx[best] / 20.0f);
        IRIS_LOG("[TUNE] SNR fit: %d points, using best SNR frame at %.1f dB SNR (tx=%.4f)",
                 n_snr, -sy[best], tx_opt);
        return std::clamp(tx_opt, 0.05f, 1.0f);
    }

    // Parabolic fit on (tx_dB, -SNR)
    float snr_min = 1e9f, snr_max = -1e9f;
    for (int i = 0; i < n_snr; i++) {
        snr_min = std::min(snr_min, -sy[i]);
        snr_max = std::max(snr_max, -sy[i]);
    }

    float S0 = (float)n_snr, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
    float Sy0 = 0, Sy1 = 0, Sy2 = 0;
    for (int i = 0; i < n_snr; i++) {
        float xi = sx[i], yi = sy[i];
        S1 += xi; S2 += xi*xi; S3 += xi*xi*xi; S4 += xi*xi*xi*xi;
        Sy0 += yi; Sy1 += xi*yi; Sy2 += xi*xi*yi;
    }
    float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
    if (std::abs(D) > 1e-12f && snr_max - snr_min >= 3.0f) {
        float Da = S0*(S2*Sy2 - S3*Sy1) - S1*(S1*Sy2 - S3*Sy0) + S2*(S1*Sy1 - S2*Sy0);
        float Db = S0*(Sy1*S4 - S3*Sy2) - S1*(Sy0*S4 - Sy2*S2) + S2*(Sy0*S3 - Sy1*S2);
        float a = Da / D;
        float b = Db / D;
        if (a > 0.0f) {
            float x_opt = -b / (2.0f * a);
            float tx_opt = std::pow(10.0f, x_opt / 20.0f);
            IRIS_LOG("[TUNE] SNR fit: %d points, a=%.4f b=%.4f x_opt=%.1f dB (tx=%.4f)",
                     n_snr, a, b, x_opt, tx_opt);
            return std::clamp(tx_opt, 0.05f, 1.0f);
        }
    }

    // Fallback: best SNR frame
    int best = 0;
    for (int i = 1; i < n_snr; i++) if (sy[i] < sy[best]) best = i;
    float tx_opt = std::pow(10.0f, sx[best] / 20.0f);
    IRIS_LOG("[TUNE] SNR fit: parabola invalid (spread=%.1f dB), using best SNR=%.1f dB (tx=%.4f)",
             snr_max - snr_min, -sy[best], tx_opt);
    return std::clamp(tx_opt, 0.05f, 1.0f);
}

float Modem::ofdm_effective_tx_level() const {
    if (ofdm_tx_base_ <= 0.0f) return config_.tx_level;
    int level = std::clamp(ofdm_speed_level_, 0, NUM_OFDM_SPEED_LEVELS - 1);
    float offset_db = ofdm_level_offset_db_[level];
    float scale = std::pow(10.0f, offset_db / 20.0f);
    float eff = ofdm_tx_base_ * scale;
    return std::clamp(eff, 0.05f, 1.0f);
}

void Modem::tune_apply_corrections() {
    // Hybrid TX/RX correction — no double-dip because adjustments are deterministic
    // and each side can predict what the peer will do.
    //
    // Both sides exchange gain reports. Each side knows:
    //   my_gain   = what WE measured from peer's test frames
    //   peer_gain = what the PEER measured from OUR test frames
    //
    // TX rule: if peer_gain > 1, we're overdriving the peer → reduce TX.
    //   Reducing always works (no radio clipping concern). Don't boost TX
    //   (radio deviation limiter makes it unreliable, confirmed OTA).
    //
    // RX rule: if my_gain < 1, peer is too quiet for us → boost RX.
    //   If my_gain > 1, the peer is too loud, but we KNOW they'll reduce
    //   their TX (their peer_gain = our my_gain > 1), so skip RX correction.
    //   This avoids double-dip: TX reduction + RX attenuation don't stack.

    float old_level = config_.tx_level;

    if (native_mode_ || ofdm_kiss_) {
        // OFDM power-ramp TUNE: peer reported (iters, H) for each of our 5 ramp
        // frames at different TX levels. Use parabolic fit to find optimal tx_level.
        //
        // If peer sent RAMP report: fit parabola to find optimal drive level.
        // If peer sent legacy GAIN report: fall back to mean|H| targeting.
        // Check for any ramp data: iters >= 0 (LDPC decoded) or -2 (preamble-only H)
        bool have_ramp = false;
        for (int i = 0; i < TUNE_RAMP_COUNT; i++)
            if (tune_peer_iters_[i] >= -2 && tune_peer_iters_[i] != -1) { have_ramp = true; break; }
        if (have_ramp) {
            float fitted_tx = tune_fit_tx_level();
            config_.tx_level = std::clamp(fitted_tx, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            ofdm_tx_base_ = config_.tx_level;
            {
                int ldpc_pts = 0, h_pts = 0;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_peer_iters_[i] >= 0) ldpc_pts++;
                    else if (tune_peer_iters_[i] == -2) h_pts++;
                }
                IRIS_LOG("[TUNE] OFDM TX: fit → %.3f (was %.3f), %d LDPC + %d preamble-only points",
                         config_.tx_level, old_level, ldpc_pts, h_pts);
            }
        } else if (tune_peer_gain_ > 0.01f) {
            // Fallback: legacy GAIN report from peer (backward compat or no frames decoded)
            constexpr float OFDM_TARGET_H = 1.0f;
            float ratio = OFDM_TARGET_H / tune_peer_gain_;
            config_.tx_level *= ratio;
            config_.tx_level = std::clamp(config_.tx_level, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            ofdm_tx_base_ = config_.tx_level;
            IRIS_LOG("[TUNE] OFDM TX fallback: peer saw H=%.3f, level %.3f → %.3f",
                     tune_peer_gain_, old_level, config_.tx_level);
        } else {
            // No peer report — we don't know how the peer received our frames.
            // But we DO know how WE received the peer's frames (tune_rx_frame_*).
            // On FM, both radios have similar deviation limiters, so the optimal
            // TX level is roughly symmetric.
            // Both sides use identical scale arrays (tune_computed_scales_[]).
            int local_decoded = 0;
            float local_h_min = 1e9f, local_h_max = -1e9f;
            int local_h_count = 0;
            for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                if (tune_rx_frame_iters_[i] >= 0) {
                    local_decoded++;
                    if (tune_rx_frame_H_[i] > 0.01f) {
                        local_h_min = std::min(local_h_min, tune_rx_frame_H_[i]);
                        local_h_max = std::max(local_h_max, tune_rx_frame_H_[i]);
                        local_h_count++;
                    }
                }
            }
            bool local_fm = (local_h_count >= 3 && local_h_max / local_h_min < 3.0f);

            if (local_decoded >= 2 && local_fm) {
                // FM mirror: clip-edge-backoff on local RX (same as peer path).
                // Find highest-TX index (lowest i) that decoded locally.
                int clip_edge_idx = -1;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_rx_frame_iters_[i] >= 0) {
                        clip_edge_idx = i;
                        break;
                    }
                }
                int use_idx = (clip_edge_idx >= 0) ? std::min(clip_edge_idx + 1, TUNE_RAMP_COUNT - 1) : 0;
                if (tune_rx_frame_iters_[use_idx] < 0 && clip_edge_idx >= 0)
                    use_idx = clip_edge_idx;
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                float mirror_tx = std::clamp(ramp_base * tune_computed_scales_[use_idx], 0.05f, 1.0f);
                config_.tx_level = mirror_tx;
                config_.calibrated_tx_level = config_.tx_level;
                ofdm_tx_base_ = config_.tx_level;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, FM mirror clip-edge-backoff: "
                         "clip_edge ramp[%d], using ramp[%d] (scale=%.2f) → tx=%.3f (was %.3f)",
                         clip_edge_idx, use_idx, tune_computed_scales_[use_idx],
                         config_.tx_level, old_level);
            } else if (local_decoded >= 2) {
                // Linear mirror: best SNR index
                int best_rx_idx = -1;
                float best_rx_snr = -1e9f;
                for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
                    if (tune_rx_frame_iters_[i] >= 0 && tune_rx_frame_snr_[i] > best_rx_snr) {
                        best_rx_snr = tune_rx_frame_snr_[i];
                        best_rx_idx = i;
                    }
                }
                float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
                float best_scale = tune_computed_scales_[best_rx_idx];
                float mirror_tx = std::clamp(ramp_base * best_scale, 0.05f, 1.0f);
                config_.tx_level = mirror_tx;
                config_.calibrated_tx_level = config_.tx_level;
                ofdm_tx_base_ = config_.tx_level;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, linear mirror: "
                         "best SNR=%.1f dB at ramp[%d] (scale=%.2f) → tx=%.3f (was %.3f)",
                         best_rx_snr, best_rx_idx, best_scale, config_.tx_level, old_level);
            } else if (ofdm_tx_base_ > 0.01f) {
                config_.tx_level = ofdm_tx_base_;
                config_.calibrated_tx_level = ofdm_tx_base_;
                IRIS_LOG("[TUNE] OFDM TX: no peer report, using probe base %.3f (was %.3f)",
                         ofdm_tx_base_, old_level);
            } else {
                IRIS_LOG("[TUNE] OFDM TX: no peer report, keeping level %.3f", config_.tx_level);
                ofdm_tx_base_ = config_.tx_level;
            }
        }

        // RX: MMSE equalizer handles arbitrary gain. No RX correction needed.
        native_rx_gain_ = 1.0f;
        IRIS_LOG("[TUNE] OFDM: base=%.3f (O0), offsets: O3=-1dB O5=-2dB O7=-3dB",
                 ofdm_tx_base_);
    } else {
        // Mode A (single-carrier): apply TX and RX corrections as before.
        // TX: adjust toward peer_gain=1.0.
        //   peer_gain > 1 → we're too loud → reduce TX (always safe).
        //   peer_gain < 1 → we're too quiet → boost TX (capped at 1.0).
        if (tune_peer_gain_ > 0.01f) {
            config_.tx_level /= tune_peer_gain_;
            config_.tx_level = std::clamp(config_.tx_level, 0.05f, 1.0f);
            config_.calibrated_tx_level = config_.tx_level;
            if (std::abs(config_.tx_level - old_level) > 0.001f) {
                IRIS_LOG("[TUNE] TX: peer_gain=%.3f, level %.3f → %.3f",
                         tune_peer_gain_, old_level, config_.tx_level);
            } else {
                IRIS_LOG("[TUNE] TX: peer_gain=%.3f, level %.3f (no change needed)",
                         tune_peer_gain_, config_.tx_level);
            }
        }

        // RX: normalize so channel_gain ≈ 1.0 (Mode A slicer needs this).
        if (tune_my_gain_ > 0.01f) {
            native_rx_gain_ = 1.0f / tune_my_gain_;
            native_rx_gain_ = std::clamp(native_rx_gain_, 0.1f, 10.0f);
            IRIS_LOG("[TUNE] RX: my_gain=%.3f, rx_gain=%.3f",
                     tune_my_gain_, native_rx_gain_);
        }
    }

    if (gui_log_) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "[TUNE] Done! TX=%.3f RxG=%.2f (we saw %.3f, peer saw %.3f)",
                 config_.tx_level, native_rx_gain_, tune_my_gain_, tune_peer_gain_);
        gui_log_(msg);
    }

    tune_audit("APPLY peer=%s role=%s my_gain=%.4f peer_gain=%.4f "
               "tx_level=%.3f->%.3f rx_gain=%.3f frames_measured=%d",
               tune_peer_call_.c_str(),
               tune_is_initiator_ ? "initiator" : "responder",
               tune_my_gain_, tune_peer_gain_,
               old_level, config_.tx_level, native_rx_gain_,
               tune_frames_measured_);

    tune_state_ = TuneState::DONE;
    tune_timeout_ = 100;  // 5s display before resetting to IDLE

    // Responder defers TX after TUNE so initiator (commander) goes first.
    // Without this, both sides TX simultaneously → half-duplex collision → T1 timeout.
    if (!tune_is_initiator_) {
        tune_post_holdoff_ = 60;  // 3s — enough for initiator to TX one frame
        IRIS_LOG("[TUNE] Responder post-TUNE holdoff: deferring TX for 3s");
    } else {
        tune_post_holdoff_ = 0;
    }
}

void Modem::tune_audit(const char* fmt, ...) {
    // Format and forward to main log via IRIS_LOG
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    IRIS_LOG("[TUNE-AUDIT] %s", buf);
}

void Modem::kalman_log_trace(const KalmanTrace& trace, bool decode_ok,
                              float snr, float gain) {
    if (trace.fwd.empty()) return;

    // Lazy-open kalman_trace_YYYYMMDD_HHMMSS.csv in AppData/Iris/logs/
    if (!kalman_log_file_) {
        time_t now_t = time(NULL);
        struct tm* t = localtime(&now_t);
        char dir[512] = ".";
        char path[700];
#ifdef _WIN32
        char appdata[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, appdata)))
            snprintf(dir, sizeof(dir), "%s\\Iris\\logs", appdata);
#else
        const char* home = getenv("HOME");
        if (home)
            snprintf(dir, sizeof(dir), "%s/.config/iris/logs", home);
#endif
        snprintf(path, sizeof(path), "%s/kalman_trace_%04d%02d%02d_%02d%02d%02d.csv",
                 dir, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
        kalman_log_file_ = fopen(path, "w");
        if (!kalman_log_file_) return;
        fprintf(kalman_log_file_,
                "timestamp,frame_seq,decode,snr_db,gain,total_sym,ds_factor,"
                "sym_idx,fwd_phase,fwd_freq,fwd_accel,smo_phase,smo_freq,smo_accel,pilot\n");
    }

    // Wall-clock timestamp for this frame
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    struct tm* tm = localtime(&t);
    char ts[32];
    snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec, (int)ms);

    static int frame_seq = 0;
    frame_seq++;

    size_t n = std::min(trace.fwd.size(), trace.smoothed.size());
    for (size_t i = 0; i < n; i++) {
        int sym = (int)i * trace.downsample_factor;
        fprintf(kalman_log_file_,
                "%s,%d,%d,%.1f,%.4f,%d,%d,"
                "%d,%.6f,%.6f,%.9f,%.6f,%.6f,%.9f,%d\n",
                ts, frame_seq, decode_ok ? 1 : 0, snr, gain,
                trace.total_symbols, trace.downsample_factor,
                sym,
                trace.fwd[i].phase, trace.fwd[i].freq, trace.fwd[i].accel,
                trace.smoothed[i].phase, trace.smoothed[i].freq, trace.smoothed[i].accel,
                trace.smoothed[i].is_pilot ? 1 : 0);
    }
    fflush(kalman_log_file_);
}

void Modem::start_autotune(const std::string& remote_callsign) {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
    if (tune_state_ != TuneState::IDLE && tune_state_ != TuneState::DONE) {
        IRIS_LOG("[TUNE] Already in progress, ignoring");
        return;
    }
    IRIS_LOG("[TUNE] Starting auto-tune with %s", remote_callsign.c_str());
    if (gui_log_) gui_log_("[TUNE] Starting auto-tune with " + remote_callsign + "...");

    tune_audit("=== TUNE START === local=%s peer=%s role=initiator tx_level=%.3f rx_gain=%.3f",
               config_.callsign.c_str(), remote_callsign.c_str(),
               config_.tx_level, native_rx_gain_);

    tune_peer_call_ = remote_callsign;
    tune_is_initiator_ = true;
    tune_my_gain_ = 0;
    tune_peer_gain_ = 0;
    tune_test_frames_sent_ = 0;
    tune_frames_measured_ = 0;
    tune_wait_peer_ticks_ = 0;
    tune_last_measured_count_ = 0;
    tune_silence_ticks_ = 0;
    tune_timeout_ = 800;  // 40s: 5 frames ~6s TX each side + report exchange + margin
    tune_test_frames_target_ = TUNE_RAMP_COUNT;
    // Compute adaptive ramp scales based on current TX base
    {
        float ramp_base = (ofdm_tx_base_ > 0.01f) ? ofdm_tx_base_ : config_.tx_level;
        tune_compute_scales(ramp_base);
    }
    for (int i = 0; i < TUNE_RAMP_COUNT; i++) {
        tune_rx_frame_iters_[i] = -1;
        tune_rx_frame_H_[i] = 0;
        tune_rx_frame_snr_[i] = -99.0f;
        tune_peer_iters_[i] = -1;
        tune_peer_H_[i] = 0;
        tune_peer_snr_[i] = -99.0f;
        tune_ramp_tx_levels_[i] = 0;
    }

    // Send TUNE:START with ramp count (twice for reliability)
    tune_state_ = TuneState::SEND_START;
    char start_msg[32];
    snprintf(start_msg, sizeof(start_msg), "TUNE:START=%d", TUNE_RAMP_COUNT);
    send_tune_ui(start_msg);
    send_tune_ui(start_msg);
}

// --- Calibration ---
// Protocol:
//   Initiator: clicks Auto Cal → sends CAL:START UI frame → tone 1s → WAIT_REPORT
//   Responder: auto-detects CAL:START → measures tone RMS → sends CAL:RMS=X.XXXX
//   Initiator: receives report → adjusts TX level → DONE

// Build and queue a UI frame with cal payload (e.g., "CAL:START" or "CAL:RMS=0.1234")
void Modem::send_cal_ui(const char* payload) {
    // Caller holds modem_mutex_
    auto src = ax25_make_addr(config_.callsign);
    auto dst = ax25_make_addr("CAL");
    // UI frame: dst(7) + src(7) + ctrl(1) + PID(1) + info
    auto frame = ax25_build_u(dst, src, AX25_CTRL_UI, false, true);
    frame.push_back(AX25_PID_NONE);
    size_t plen = strlen(payload);
    frame.insert(frame.end(), (const uint8_t*)payload, (const uint8_t*)payload + plen);
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
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
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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
    // Caller holds modem_mutex_
    constexpr size_t TX_QUEUE_MAX = 32;
    if (ax25_tx_queue_.size() >= TX_QUEUE_MAX) {
        IRIS_LOG("[TX] AX.25 queue full (%zu frames), dropping oldest", ax25_tx_queue_.size());
        ax25_tx_queue_.pop();
    }
    ax25_tx_queue_.push(std::move(frame));
}

void Modem::start_calibration() {
    std::lock_guard<std::recursive_mutex> lock(modem_mutex_);
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

void Modem::cache_probe_result(const std::string& callsign) {
    if (callsign.empty() || !probe_.has_results() || !probe_.negotiated().valid)
        return;

    ProbeCacheEntry entry;
    entry.negotiated = probe_.negotiated();
    entry.my_tx = probe_.my_tx_result();
    entry.their_tx = probe_.their_tx_result();
    entry.timestamp = std::chrono::steady_clock::now();
    probe_cache_[callsign] = std::move(entry);

    IRIS_LOG("[PROBE-CACHE] cached result for %s (band %.0f-%.0f Hz, BW=%.0f Hz)",
             callsign.c_str(), entry.negotiated.low_hz, entry.negotiated.high_hz,
             entry.negotiated.bandwidth_hz);
}

bool Modem::try_use_cached_probe(const std::string& callsign) {
    auto it = probe_cache_.find(callsign);
    if (it == probe_cache_.end())
        return false;

    auto& entry = it->second;
    auto age = std::chrono::steady_clock::now() - entry.timestamp;
    int age_s = (int)std::chrono::duration_cast<std::chrono::seconds>(age).count();
    if (age_s > PROBE_CACHE_EXPIRY_S) {
        IRIS_LOG("[PROBE-CACHE] expired for %s (%d s old)", callsign.c_str(), age_s);
        probe_cache_.erase(it);
        return false;
    }

    // Replay probe results: apply cached passband, configure PHY, enable native mode.
    // This mirrors the probe completion path in tick() but skips the actual probe.
    ofdm_kiss_probe_done_ = true;
    dcd_holdoff_ = 0;

    float low = entry.negotiated.low_hz;
    float high = entry.negotiated.high_hz;
    float bandwidth = high - low;
    float center = (low + high) / 2.0f;
    config_.band_low_hz = low;
    config_.band_high_hz = high;

    if (use_upconvert_) {
        upconverter_ = Upconverter(center, config_.sample_rate);
        downconverter_ = Downconverter(center, config_.sample_rate);

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
            new_sps = phy_config_.samples_per_symbol;
            new_baud = phy_config_.baud_rate;
        }

        if (new_baud != phy_config_.baud_rate) {
            phy_config_.baud_rate = new_baud;
            phy_config_.samples_per_symbol = new_sps;
            native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
            native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);
        }
    }

    // Channel equalization from cached probe data
    rx_channel_eq_.configure(entry.their_tx, entry.negotiated, config_.sample_rate, 3.0f);
    tx_channel_eq_.configure(entry.my_tx, entry.negotiated, config_.sample_rate, 6.0f);

    // Enable native mode
    peer_is_iris_ = true;
    ofdm_kiss_ = true;
    ofdm_kiss_tx_ = true;
    ax25_session_.set_native_active(true);

    // OFDM PHY setup from cached passband
    if (config_.ofdm_enable) {
        NegotiatedPassband ofdm_pb;
        ofdm_pb.low_hz = low;
        ofdm_pb.high_hz = high;
        ofdm_pb.center_hz = center;
        ofdm_pb.bandwidth_hz = bandwidth;
        ofdm_pb.valid = true;

        int cp = config_.ofdm_cp_samples;
        int carrier_pilot_spacing = 4;
        int block_pilot_spacing = 24;
        int nfft = config_.ofdm_nfft;

        // Use negotiated params from cached peer result
        if (entry.my_tx.ofdm_cp_samples > 0)
            cp = std::max(cp, (int)entry.my_tx.ofdm_cp_samples);
        if (entry.my_tx.ofdm_pilot_carrier_spacing > 0)
            carrier_pilot_spacing = std::min(carrier_pilot_spacing, (int)entry.my_tx.ofdm_pilot_carrier_spacing);
        if (entry.my_tx.ofdm_pilot_symbol_spacing > 0)
            block_pilot_spacing = std::min(block_pilot_spacing, (int)entry.my_tx.ofdm_pilot_symbol_spacing);
        {
            // nfft_code: 0=512, 1=256, 2=1024
            int peer_nfft = 512;
            if (entry.my_tx.ofdm_nfft_code == 1) peer_nfft = 256;
            else if (entry.my_tx.ofdm_nfft_code == 2) peer_nfft = 1024;
            nfft = std::min(nfft, peer_nfft);
        }

        ofdm_config_ = ofdm_config_from_probe(ofdm_pb, nfft, cp,
                                               carrier_pilot_spacing, block_pilot_spacing);
        ofdm_config_.fm_preemph_corner_hz = config_.ofdm_preemph_corner_hz;
        ofdm_mod_ = std::make_unique<OfdmModulator>(ofdm_config_);
        ofdm_demod_ = std::make_unique<OfdmDemodulator>(ofdm_config_);
        ofdm_rx_iq_.clear();
        ofdm_rx_audio_buf_.clear();
        ofdm_rx_lpf_.reset();
        ofdm_sync_cached_ = false;

        gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        ofdm_speed_level_ = 0;
        gearshift_.force_ofdm_level(0);
        ofdm_tone_map_ = get_uniform_tone_map(1, ofdm_config_);
        ofdm_tone_map_.use_nuc = config_.ofdm_nuc;

        // Peer caps from cached probe
        uint16_t peer_caps = entry.my_tx.capabilities;
        ofdm_kiss_peer_caps_ = local_cap_.capabilities & peer_caps;

        if (ofdm_kiss_peer_caps_ & CAP_COMPRESSION) {
            ofdm_kiss_tx_compressor_.init();
            ofdm_kiss_rx_compressor_.init();
        }

        if (config_.ofdm_enable && ofdm_mod_ && (ofdm_kiss_peer_caps_ & CAP_OFDM)) {
            ofdm_phy_active_ = true;
            int ofdm_max_info = (LdpcCodec::block_size(LdpcRate::RATE_1_2) / 8 - 6) * 2 - 16;
            ax25_session_.set_max_info(ofdm_max_info);
            IRIS_LOG("[PROBE-CACHE] OFDM PHY ACTIVE: %d carriers, BW=%.0f Hz",
                     ofdm_config_.n_data_carriers, ofdm_config_.bandwidth_hz);
        }
    }

    // Speed level cache
    int cached_level = gearshift_.load_cached_level(callsign);
    if (cached_level > 0) {
        gearshift_.force_level(cached_level);
        IRIS_LOG("[PROBE-CACHE] gearshift: cached level %d", cached_level);
    }

    // Shorter listen window for cached probe (no probe tones to wait for)
    csma_holdoff_ = config_.sample_rate;  // 1s listen (vs 3s for fresh probe)

    // Adaptive TXDELAY for cached probe
    ofdm_txdelay_ms_ = std::max(50, config_.ptt_pre_delay_ms / 2);

    IRIS_LOG("[PROBE-CACHE] applied cached probe for %s (age=%ds, band %.0f-%.0f Hz)",
             callsign.c_str(), age_s, low, high);
    return true;
}

} // namespace iris
