#include "engine/modem.h"
#include "native/frame.h"
#include <cstring>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

static constexpr float CAL_TONE_FREQ = 1000.0f;
static constexpr int   CAL_TONE_DURATION = 48000;
static constexpr float CAL_TARGET_RMS = 0.3f;
static constexpr int RX_MUTE_HOLDOFF_SAMPLES = 2400;  // 50ms at 48kHz

Modem::Modem() = default;
Modem::~Modem() { shutdown(); }

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    if (config_.mode == "A" || config_.mode == "a") {
        phy_config_ = mode_a_config();
        use_upconvert_ = true;
        upconverter_ = Upconverter(MODE_A_CENTER_FREQ, config_.sample_rate);
        downconverter_ = Downconverter(MODE_A_CENTER_FREQ, config_.sample_rate);
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

        rx_callback_(cur, cur_len);
    };
    arq_cb.on_state_changed = [this](ArqState state) {
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
            }

            // Init B2F handler if negotiated
            if (arq_.negotiated(CAP_B2F_UNROLL)) {
                b2f_handler_.init();
                b2f_handler_.unroll_enabled = true;
            }
        } else if (state == ArqState::IDLE) {
            tx_compressor_.deinit();
            rx_compressor_.deinit();
            cipher_.wipe();
            b2f_handler_.deinit();
        }
        if (state_callback_)
            state_callback_(state, arq_.remote_callsign());
    };
    arq_.set_callbacks(arq_cb);

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
    }
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
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
    }

    std::vector<float> audio(rx_audio, rx_audio + frame_count);
    agc_.process_block(audio.data(), frame_count);

    float sum_sq = 0;
    for (int i = 0; i < frame_count; i++)
        sum_sq += audio[i] * audio[i];
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));

    compute_spectrum(audio.data(), frame_count);

    if (state_ == ModemState::CALIBRATING) {
        process_calibration_rx(audio.data(), frame_count);
        return;
    }

    if (native_mode_) {
        process_rx_native(audio.data(), frame_count);
    } else {
        process_rx_ax25(audio.data(), frame_count);
    }
}

void Modem::process_rx_ax25(const float* audio, int count) {
    state_ = ModemState::RX_AX25;

    std::vector<uint8_t> rx_nrzi;
    if (config_.ax25_baud == 9600)
        rx_nrzi = gfsk_demod_.demodulate(audio, count);
    else
        rx_nrzi = afsk_demod_.demodulate(audio, count);

    auto rx_bits = nrzi_decoder_.decode(rx_nrzi);

    for (uint8_t b : rx_bits) {
        if (hdlc_decoder_.push_bit(b)) {
            const auto& frame = hdlc_decoder_.frame();
            frames_rx_++;

            if (frame.size() >= 24 && frame[15] == IRIS_PID) {
                XidCapability remote_cap;
                if (xid_decode(&frame[16], frame.size() - 16, remote_cap)) {
                    auto agreed = negotiate(local_cap_, remote_cap);
                    if (agreed.capabilities != 0) {
                        native_mode_ = true;
                        int mod_level = std::min((int)agreed.max_modulation,
                                                (int)config_.max_modulation);
                        phy_config_.modulation = (Modulation)mod_level;
                    }
                }
            }

            if (rx_callback_)
                rx_callback_(frame.data(), frame.size());

            hdlc_decoder_.reset();
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_rx_native(const float* audio, int count) {
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

    int start = detect_frame_start(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                    phy_config_.samples_per_symbol);
    if (start >= 0) {
        std::vector<uint8_t> payload;
        if (decode_native_frame(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                 start, phy_config_, payload)) {
            frames_rx_++;

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

            // Route through ARQ in any active session state
            auto arq_st = arq_.state();
            if (arq_st == ArqState::CONNECTED ||
                arq_st == ArqState::CONNECTING ||
                arq_st == ArqState::LISTENING ||
                arq_st == ArqState::HAILING ||
                arq_st == ArqState::DISCONNECTING) {
                arq_.on_frame_received(payload.data(), payload.size());
            } else {
                if (rx_callback_)
                    rx_callback_(payload.data(), payload.size());
            }

            size_t consumed = start + phy_config_.samples_per_symbol * 128;
            if (consumed > rx_overlap_buf_.size())
                consumed = rx_overlap_buf_.size();
            rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                                   rx_overlap_buf_.begin() + consumed);
        } else {
            crc_errors_++;
        }
    }

    if (rx_rms_ < 0.001f)
        state_ = ModemState::IDLE;
}

void Modem::process_tx(float* tx_audio, int frame_count) {
    if (tx_pos_ < tx_buffer_.size()) {
        size_t remaining = tx_buffer_.size() - tx_pos_;
        size_t to_copy = std::min((size_t)frame_count, remaining);
        std::memcpy(tx_audio, tx_buffer_.data() + tx_pos_, to_copy * sizeof(float));
        tx_pos_ += to_copy;

        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        if (tx_pos_ >= tx_buffer_.size()) {
            tx_buffer_.clear();
            tx_pos_ = 0;
            ptt_off();
            state_ = ModemState::IDLE;
        }
        return;
    }

    if (state_ == ModemState::CALIBRATING && cal_state_ == CalState::TX_TONE) {
        generate_cal_tone(tx_audio, frame_count);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (!tx_queue_.empty()) {
            auto frame_data = std::move(tx_queue_.front());
            tx_queue_.pop();

            if (native_mode_) {
                state_ = ModemState::TX_NATIVE;
                int level = gearshift_.current_level();
                PhyConfig tx_config = phy_config_;
                tx_config.modulation = SPEED_LEVELS[level].modulation;
                LdpcRate fec = fec_to_ldpc_rate(SPEED_LEVELS[level].fec_rate_num,
                                                 SPEED_LEVELS[level].fec_rate_den);
                auto iq = build_native_frame(frame_data.data(), frame_data.size(), tx_config, fec);

                if (use_upconvert_) {
                    tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                } else {
                    tx_buffer_.resize(iq.size() / 2);
                    for (size_t i = 0; i < iq.size() / 2; i++)
                        tx_buffer_[i] = iq[2 * i];
                }
            } else {
                state_ = ModemState::TX_AX25;
                auto bits = hdlc_encode(frame_data.data(), frame_data.size(), 8, 4);
                if (config_.ax25_baud == 9600)
                    tx_buffer_ = gfsk_mod_.modulate(bits);
                else
                    tx_buffer_ = afsk_mod_.modulate(bits);
            }

            for (auto& s : tx_buffer_)
                s *= config_.tx_level;

            int pre_samples = config_.ptt_pre_delay_ms * config_.sample_rate / 1000;
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
            return;
        }
    }

    std::memset(tx_audio, 0, frame_count * sizeof(float));
}

void Modem::queue_tx_frame(const uint8_t* frame, size_t len) {
    if (native_mode_ && arq_.state() == ArqState::CONNECTED) {
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
            }
        }

        arq_.send_data(cur, cur_len);
        return;
    }
    std::lock_guard<std::mutex> lock(tx_mutex_);

    // If AX.25 mode and native upgrade allowed, send XID before first user frame
    if (!native_mode_ && !config_.ax25_only && !xid_sent_) {
        auto xid_frame = build_xid_frame(config_.callsign.c_str(), "CQ    ",
                                           local_cap_);
        tx_queue_.push(std::move(xid_frame));
        xid_sent_ = true;
    }

    tx_queue_.push(std::vector<uint8_t>(frame, frame + len));
}

void Modem::arq_connect(const std::string& remote_callsign) {
    native_mode_ = true;
    arq_.connect(remote_callsign);
}

void Modem::arq_disconnect() {
    arq_.disconnect();
}

void Modem::arq_listen() {
    native_mode_ = true;
    arq_.listen();
}

void Modem::tick() {
    arq_.tick();
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
    diag.ptt_active = ptt_active_;
    diag.cal_state = cal_state_;
    diag.cal_measured_rms = cal_measured_rms_;
    diag.arq_state = arq_.state();
    diag.arq_role = arq_.role();

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        diag.constellation = last_constellation_;
        diag.spectrum = last_spectrum_;
    }

    return diag;
}

void Modem::compute_spectrum(const float* audio, int count) {
    constexpr int NFFT = 256;
    if (count < NFFT) return;

    std::vector<float> spec(NFFT / 2, 0.0f);
    const float* src = audio + count - NFFT;
    for (int k = 0; k < NFFT / 2; k++) {
        float re = 0, im = 0;
        for (int n = 0; n < NFFT; n++) {
            float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * n / (NFFT - 1)));
            float angle = -2.0f * (float)M_PI * k * n / NFFT;
            re += src[n] * w * std::cos(angle);
            im += src[n] * w * std::sin(angle);
        }
        float pwr = (re * re + im * im) / (NFFT * NFFT);
        spec[k] = 10.0f * std::log10(std::max(pwr, 1e-12f));
    }

    std::lock_guard<std::mutex> lock(diag_mutex_);
    last_spectrum_ = std::move(spec);
}

// --- Calibration ---

void Modem::start_calibration() {
    state_ = ModemState::CALIBRATING;
    cal_state_ = CalState::TX_TONE;
    cal_tone_samples_ = 0;
    cal_tone_phase_ = 0;
    cal_rms_accum_ = 0;
    cal_rms_count_ = 0;
    cal_measured_rms_ = 0;
    ptt_on();
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
    }
}

void Modem::process_calibration_rx(const float* audio, int count) {
    if (cal_state_ == CalState::RX_TONE) {
        for (int i = 0; i < count; i++) {
            cal_rms_accum_ += audio[i] * audio[i];
            cal_rms_count_++;
        }
        if (cal_rms_count_ >= CAL_TONE_DURATION) {
            cal_measured_rms_ = std::sqrt(cal_rms_accum_ / cal_rms_count_);
            char report[32];
            snprintf(report, sizeof(report), "CAL:RMS=%.4f", cal_measured_rms_);
            {
                std::lock_guard<std::mutex> lock(tx_mutex_);
                tx_queue_.push(std::vector<uint8_t>((uint8_t*)report,
                    (uint8_t*)report + strlen(report)));
            }
            cal_state_ = CalState::TX_REPORT;
        }
    } else if (cal_state_ == CalState::WAIT_REPORT) {
        std::vector<uint8_t> rx_nrzi;
        if (config_.ax25_baud == 9600)
            rx_nrzi = gfsk_demod_.demodulate(audio, count);
        else
            rx_nrzi = afsk_demod_.demodulate(audio, count);

        auto rx_bits = nrzi_decoder_.decode(rx_nrzi);
        for (uint8_t b : rx_bits) {
            if (hdlc_decoder_.push_bit(b)) {
                const auto& frame = hdlc_decoder_.frame();
                if (frame.size() > 16) {
                    std::string payload(frame.begin() + 16, frame.end());
                    size_t pos = payload.find("CAL:RMS=");
                    if (pos != std::string::npos) {
                        float remote_rms = std::stof(payload.substr(pos + 8));
                        if (remote_rms > 0.001f) {
                            float correction = CAL_TARGET_RMS / remote_rms;
                            config_.tx_level *= correction;
                            config_.tx_level = std::clamp(config_.tx_level, 0.01f, 1.0f);
                            config_.calibrated_tx_level = config_.tx_level;
                            cal_measured_rms_ = remote_rms;
                        }
                        cal_state_ = CalState::DONE;
                        state_ = ModemState::IDLE;
                    }
                }
                hdlc_decoder_.reset();
            }
        }
    } else if (cal_state_ == CalState::TX_REPORT) {
        if (state_ != ModemState::TX_AX25 && tx_buffer_.empty()) {
            cal_state_ = CalState::RX_TONE;
            cal_rms_accum_ = 0;
            cal_rms_count_ = 0;
        }
    }
}

} // namespace iris
