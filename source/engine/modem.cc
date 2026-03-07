#include "engine/modem.h"
#include "native/frame.h"
#include <cstring>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Calibration parameters
static constexpr float CAL_TONE_FREQ = 1000.0f;    // Hz
static constexpr int   CAL_TONE_DURATION = 48000;   // 1 second of tone
static constexpr float CAL_TARGET_RMS = 0.3f;       // Target RX RMS for optimal deviation

Modem::Modem() = default;
Modem::~Modem() { shutdown(); }

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    // Configure native PHY based on mode
    if (config_.mode == "A" || config_.mode == "a") {
        phy_config_ = mode_a_config();
        use_upconvert_ = true;  // Mode A needs audio upconversion
        upconverter_ = Upconverter(MODE_A_CENTER_FREQ, config_.sample_rate);
        downconverter_ = Downconverter(MODE_A_CENTER_FREQ, config_.sample_rate);
    } else if (config_.mode == "B" || config_.mode == "b") {
        phy_config_ = mode_b_config();
        use_upconvert_ = false;
    } else {
        phy_config_ = mode_c_config();
        use_upconvert_ = false;
    }

    phy_config_.modulation = Modulation::BPSK; // start conservative

    native_mod_ = std::make_unique<NativeModulator>(phy_config_, config_.sample_rate);
    native_demod_ = std::make_unique<NativeDemodulator>(phy_config_, config_.sample_rate);

    // Set up local capabilities
    local_cap_.version = XID_VERSION;
    local_cap_.capabilities = CAP_MODE_A;
    if (config_.mode == "B" || config_.mode == "b")
        local_cap_.capabilities |= CAP_MODE_B;
    if (config_.mode == "C" || config_.mode == "c")
        local_cap_.capabilities |= CAP_MODE_C;
    local_cap_.max_modulation = config_.max_modulation;

    // Gearshift setup
    int max_mod_idx = (int)config_.max_modulation;
    int max_level = NUM_SPEED_LEVELS - 1;
    for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
        if ((int)SPEED_LEVELS[i].modulation > max_mod_idx) {
            max_level = (i > 0) ? i - 1 : 0;
            break;
        }
    }
    gearshift_.set_max_level(max_level);

    state_ = ModemState::IDLE;
    return true;
}

void Modem::shutdown() {
    ptt_off();
    state_ = ModemState::IDLE;
    native_mod_.reset();
    native_demod_.reset();
}

void Modem::ptt_on() {
    if (!ptt_active_ && ptt_) {
        ptt_->set_ptt(true);
        ptt_active_ = true;
    }
}

void Modem::ptt_off() {
    if (ptt_active_ && ptt_) {
        ptt_->set_ptt(false);
        ptt_active_ = false;
    }
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
    if (state_ == ModemState::TX_AX25 || state_ == ModemState::TX_NATIVE)
        return; // Don't receive while transmitting

    // AGC
    // Work on a copy so we don't modify caller's buffer
    std::vector<float> audio(rx_audio, rx_audio + frame_count);
    agc_.process_block(audio.data(), frame_count);

    // Measure RMS for diagnostics
    float sum_sq = 0;
    for (int i = 0; i < frame_count; i++)
        sum_sq += audio[i] * audio[i];
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));

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
    if (config_.ax25_baud == 9600) {
        rx_nrzi = gfsk_demod_.demodulate(audio, count);
    } else {
        rx_nrzi = afsk_demod_.demodulate(audio, count);
    }

    auto rx_bits = nrzi_decode(rx_nrzi);

    for (uint8_t b : rx_bits) {
        if (hdlc_decoder_.push_bit(b)) {
            const auto& frame = hdlc_decoder_.frame();
            frames_rx_++;

            // Check if this is an XID frame with Iris capabilities
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

            // Deliver to KISS clients
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

    // Append new IQ data to overlap buffer
    rx_overlap_buf_.insert(rx_overlap_buf_.end(), iq_data, iq_data + iq_count);

    // Cap buffer size
    if (rx_overlap_buf_.size() > RX_OVERLAP_MAX) {
        size_t excess = rx_overlap_buf_.size() - RX_OVERLAP_MAX;
        rx_overlap_buf_.erase(rx_overlap_buf_.begin(),
                               rx_overlap_buf_.begin() + excess);
    }

    // Search for frames in the accumulated buffer
    int start = detect_frame_start(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                    phy_config_.samples_per_symbol);
    if (start >= 0) {
        std::vector<uint8_t> payload;
        if (decode_native_frame(rx_overlap_buf_.data(), rx_overlap_buf_.size(),
                                 start, phy_config_, payload)) {
            frames_rx_++;

            // Estimate SNR from preamble and feed gearshift
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

            if (rx_callback_)
                rx_callback_(payload.data(), payload.size());

            // Remove consumed data up to frame end
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
    // If we have buffered TX samples, output them
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

    // Calibration tone generation
    if (state_ == ModemState::CALIBRATING && cal_state_ == CalState::TX_TONE) {
        generate_cal_tone(tx_audio, frame_count);
        return;
    }

    // Check TX queue for new frames
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
                    // Mode A: upconvert IQ to audio
                    tx_buffer_ = upconverter_.iq_to_audio(iq.data(), iq.size());
                } else {
                    // Mode B/C: IQ is baseband, output directly
                    // For real hardware, Mode B outputs just I channel (real baseband)
                    tx_buffer_.resize(iq.size() / 2);
                    for (size_t i = 0; i < iq.size() / 2; i++)
                        tx_buffer_[i] = iq[2 * i]; // I channel only for FM baseband
                }
            } else {
                state_ = ModemState::TX_AX25;
                auto bits = hdlc_encode(frame_data.data(), frame_data.size(), 8, 4);
                if (config_.ax25_baud == 9600) {
                    tx_buffer_ = gfsk_mod_.modulate(bits);
                } else {
                    tx_buffer_ = afsk_mod_.modulate(bits);
                }
            }

            // Apply TX level
            for (auto& s : tx_buffer_)
                s *= config_.tx_level;

            // PTT pre-delay: silence at front for relay engage
            int pre_samples = config_.ptt_pre_delay_ms * config_.sample_rate / 1000;
            if (pre_samples > 0)
                tx_buffer_.insert(tx_buffer_.begin(), pre_samples, 0.0f);

            // PTT post-delay: silence at end for relay tail
            int post_samples = config_.ptt_post_delay_ms * config_.sample_rate / 1000;
            if (post_samples > 0)
                tx_buffer_.insert(tx_buffer_.end(), post_samples, 0.0f);

            tx_pos_ = 0;
            frames_tx_++;

            // Assert PTT
            ptt_on();

            // Output first chunk
            size_t to_copy = std::min((size_t)frame_count, tx_buffer_.size());
            std::memcpy(tx_audio, tx_buffer_.data(), to_copy * sizeof(float));
            tx_pos_ = to_copy;
            if (to_copy < (size_t)frame_count)
                std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));
            return;
        }
    }

    // Nothing to transmit — silence
    std::memset(tx_audio, 0, frame_count * sizeof(float));
}

void Modem::queue_tx_frame(const uint8_t* frame, size_t len) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_queue_.push(std::vector<uint8_t>(frame, frame + len));
}

ModemDiag Modem::get_diagnostics() const {
    ModemDiag diag;
    diag.state = state_;
    diag.speed_level = gearshift_.current_level();
    diag.snr_db = gearshift_.smoothed_snr();
    diag.agc_gain = agc_.gain();
    diag.tx_level = config_.tx_level;
    diag.kiss_clients = 0; // filled by caller
    diag.frames_rx = frames_rx_;
    diag.frames_tx = frames_tx_;
    diag.crc_errors = crc_errors_;
    diag.rx_rms = rx_rms_;
    diag.ptt_active = ptt_active_;
    diag.cal_state = cal_state_;
    diag.cal_measured_rms = cal_measured_rms_;

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        diag.constellation = last_constellation_;
    }

    return diag;
}

// --- Auto Level Calibration ---
// Two-station procedure:
// 1. Station A transmits 1000 Hz test tone at current TX level (1 second)
// 2. Station B measures received RMS level
// 3. Station B reports measured level back (as AX.25 UI frame)
// 4. Station A adjusts TX level: new_level = old_level * (target_rms / measured_rms)
// 5. Roles reverse for Station B calibration

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
        // Done transmitting tone
        ptt_off();
        cal_state_ = CalState::WAIT_REPORT;
    }
}

void Modem::process_calibration_rx(const float* audio, int count) {
    if (cal_state_ == CalState::RX_TONE) {
        // Measure RMS of incoming tone
        for (int i = 0; i < count; i++) {
            cal_rms_accum_ += audio[i] * audio[i];
            cal_rms_count_++;
        }

        // After 1 second of measurement
        if (cal_rms_count_ >= CAL_TONE_DURATION) {
            cal_measured_rms_ = std::sqrt(cal_rms_accum_ / cal_rms_count_);

            // Build calibration report frame
            // Simple format: "CAL:RMS=0.xxx"
            char report[32];
            snprintf(report, sizeof(report), "CAL:RMS=%.4f", cal_measured_rms_);
            queue_tx_frame((const uint8_t*)report, strlen(report));

            cal_state_ = CalState::TX_REPORT;
        }
    } else if (cal_state_ == CalState::WAIT_REPORT) {
        // Look for calibration report in incoming frames
        // Process as AX.25 to receive the report
        std::vector<uint8_t> rx_nrzi;
        if (config_.ax25_baud == 9600)
            rx_nrzi = gfsk_demod_.demodulate(audio, count);
        else
            rx_nrzi = afsk_demod_.demodulate(audio, count);

        auto rx_bits = nrzi_decode(rx_nrzi);
        for (uint8_t b : rx_bits) {
            if (hdlc_decoder_.push_bit(b)) {
                const auto& frame = hdlc_decoder_.frame();
                // Check if it's a calibration report
                // Look for "CAL:RMS=" in the payload (after 16 bytes of AX.25 header)
                if (frame.size() > 16) {
                    std::string payload(frame.begin() + 16, frame.end());
                    size_t pos = payload.find("CAL:RMS=");
                    if (pos != std::string::npos) {
                        float remote_rms = std::stof(payload.substr(pos + 8));
                        if (remote_rms > 0.001f) {
                            // Adjust TX level
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
        // Wait for the report frame to be transmitted
        if (state_ != ModemState::TX_AX25 && tx_buffer_.empty()) {
            // Switch to receiving partner's tone
            cal_state_ = CalState::RX_TONE;
            cal_rms_accum_ = 0;
            cal_rms_count_ = 0;
        }
    }
}

} // namespace iris
