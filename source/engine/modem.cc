#include "engine/modem.h"
#include "native/frame.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace iris {

Modem::Modem() = default;
Modem::~Modem() { shutdown(); }

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    // Configure native PHY based on mode
    if (config_.mode == "A" || config_.mode == "a")
        phy_config_ = mode_a_config();
    else if (config_.mode == "B" || config_.mode == "b")
        phy_config_ = mode_b_config();
    else
        phy_config_ = mode_c_config();

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
    // Map max modulation to max speed level
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
    state_ = ModemState::IDLE;
    native_mod_.reset();
    native_demod_.reset();
}

void Modem::process_rx(const float* rx_audio, int frame_count) {
    if (state_ == ModemState::TX_AX25 || state_ == ModemState::TX_NATIVE)
        return; // Don't receive while transmitting

    // Measure RMS for diagnostics
    float sum_sq = 0;
    for (int i = 0; i < frame_count; i++)
        sum_sq += rx_audio[i] * rx_audio[i];
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));

    if (native_mode_) {
        process_rx_native(rx_audio, frame_count);
    } else {
        process_rx_ax25(rx_audio, frame_count);
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
                        // Update PHY config based on negotiation
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

    // For native mode, audio is IQ pairs (or needs to be downconverted for Mode A)
    // For now, treat as IQ interleaved
    int start = detect_frame_start(audio, count * 2, phy_config_.samples_per_symbol);
    if (start >= 0) {
        std::vector<uint8_t> payload;
        if (decode_native_frame(audio, count * 2, start, phy_config_, payload)) {
            frames_rx_++;

            // Update constellation for GUI
            {
                std::lock_guard<std::mutex> lock(diag_mutex_);
                if (native_demod_) {
                    last_constellation_ = native_demod_->symbols();
                }
            }

            if (rx_callback_)
                rx_callback_(payload.data(), payload.size());
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

        // Zero-fill remainder if frame exhausted
        if (to_copy < (size_t)frame_count)
            std::memset(tx_audio + to_copy, 0, (frame_count - to_copy) * sizeof(float));

        if (tx_pos_ >= tx_buffer_.size()) {
            tx_buffer_.clear();
            tx_pos_ = 0;
            state_ = ModemState::IDLE;
        }
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
                // Build native frame
                int level = gearshift_.current_level();
                PhyConfig tx_config = phy_config_;
                tx_config.modulation = SPEED_LEVELS[level].modulation;
                tx_buffer_ = build_native_frame(frame_data.data(), frame_data.size(), tx_config);
            } else {
                state_ = ModemState::TX_AX25;
                // Build AX.25 frame
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

            tx_pos_ = 0;
            frames_tx_++;

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
    diag.ptt_active = (state_ == ModemState::TX_AX25 || state_ == ModemState::TX_NATIVE);

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        diag.constellation = last_constellation_;
    }

    return diag;
}

void Modem::start_calibration() {
    state_ = ModemState::CALIBRATING;
    // Auto level calibration:
    // 1. Generate test tone at current TX level
    // 2. Partner station measures received level and reports back
    // 3. Adjust TX level to target optimal FM deviation
    // For now, just store that we're in calibration mode.
    // The actual two-station procedure requires ARQ exchange.
}

} // namespace iris
