#include "engine/modem.h"
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

static constexpr float CAL_TONE_FREQ = 1000.0f;
static constexpr int   CAL_TONE_DURATION = 48000;
static constexpr float CAL_TARGET_RMS = 0.3f;
static constexpr int RX_MUTE_HOLDOFF_SAMPLES = 9600;  // 200ms at 48kHz (covers VB-Cable/soundcard latency)

Modem::Modem() = default;
Modem::~Modem() { shutdown(); }

bool Modem::init(const IrisConfig& config) {
    config_ = config;

    if (config_.mode == "A" || config_.mode == "a") {
        phy_config_ = mode_a_config();
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

    // Count down XID reply delay (responder stays in RX to catch trailing data)
    if (xid_reply_delay_samples_ > 0) {
        xid_reply_delay_samples_ -= frame_count;
        if (xid_reply_delay_samples_ <= 0) {
            xid_reply_delay_samples_ = 0;
            if (!pending_xid_reply_.empty()) {
                std::lock_guard<std::mutex> lock(tx_mutex_);
                ax25_tx_queue_.push(std::move(pending_xid_reply_));
                pending_xid_reply_.clear();
                IRIS_LOG("XID reply released to TX queue");
            }
        }
    }

    std::vector<float> audio(rx_audio, rx_audio + frame_count);
    // AGC for AX.25 mode only — native mode has preamble-based gain estimation
    // AGC would distort QAM symbols within a frame (gain changes during preamble vs payload)
    if (!native_mode_)
        agc_.process_block(audio.data(), frame_count);

    float sum_sq = 0;
    for (int i = 0; i < frame_count; i++)
        sum_sq += audio[i] * audio[i];
    rx_rms_ = std::sqrt(sum_sq / std::max(frame_count, 1));

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

            // Try ARQ session first — ARQ frames are raw bytes without
            // AX.25 headers, so they must be handled before the AX.25 checks.
            {
                ArqState arq_st = arq_.state();
                if (arq_st != ArqState::IDLE) {
                    if (arq_.on_frame_received(frame.data(), frame.size())) {
                        hdlc_decoder_.reset();
                        continue;  // ARQ handled it
                    }
                }
            }

            bool is_xid = false;
            if (frame.size() >= 24 && frame[15] == IRIS_PID) {
                // Extract source callsign from AX.25 header (bytes 7-12, shifted)
                std::string src_call;
                for (int ci = 7; ci < 13 && ci < (int)frame.size(); ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') src_call += c;
                }
                // All XID frames are internal protocol — never forward to KISS
                is_xid = true;
                // Ignore our own XID (self-hearing on shared channel)
                bool is_self = (src_call == config_.callsign);
                XidCapability remote_cap;
                if (!is_self && xid_decode(&frame[16], frame.size() - 16, remote_cap)) {
                    is_xid = true;
                    auto agreed = negotiate(local_cap_, remote_cap);
                    if (agreed.capabilities != 0) {
                        int mod_level = std::min((int)agreed.max_modulation,
                                                (int)config_.max_modulation);
                        phy_config_.modulation = (Modulation)mod_level;

                        if (!xid_sent_ && !config_.ax25_only) {
                            // We're the responder. Delay XID reply by ~500ms
                            // so we stay in AX.25 RX mode and can decode any
                            // trailing data frames from the initiator before
                            // we start transmitting (which blocks RX).
                            pending_xid_reply_ = build_xid_frame(
                                config_.callsign.c_str(), "CQ    ", local_cap_);
                            xid_reply_delay_samples_ = config_.sample_rate / 2;  // 500ms
                            xid_sent_ = true;
                            // Native mode switch after holdoff (in tick())
                            native_tx_holdoff_ = 10;  // ~1s
                            IRIS_LOG("XID received (responder): reply delayed 500ms, holdoff=%d", native_tx_holdoff_);
                        } else {
                            // We're the initiator — we already sent XID, remote
                            // is replying. Both sides are Iris-aware, switch now.
                            native_mode_ = true;
                            native_tx_ready_ = true;
                            // Flush deferred data to TX queue (now goes native)
                            {
                                std::lock_guard<std::mutex> lock(tx_mutex_);
                                while (!deferred_tx_queue_.empty()) {
                                    tx_queue_.push(std::move(deferred_tx_queue_.front()));
                                    deferred_tx_queue_.pop();
                                }
                            }
                            xid_fallback_ticks_ = 0;
                            IRIS_LOG("XID handshake complete (initiator), native mode active, flushed deferred");
                        }
                    }
                }
            }

            // Don't forward XID or self-originated frames to KISS clients
            if (!is_xid && rx_callback_ && frame.size() >= 14) {
                // Check source callsign to filter self-hearing
                std::string rx_src;
                for (int ci = 7; ci < 13; ci++) {
                    char c = (char)(frame[ci] >> 1);
                    if (c != ' ') rx_src += c;
                }
                if (rx_src != config_.callsign)
                    rx_callback_(frame.data(), frame.size());
            }

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
            // Always set RX holdoff after TX to prevent self-hearing
            // (VB-Cable and soundcard loopback buffer our own TX)
            rx_muted_ = true;
            rx_mute_holdoff_ = RX_MUTE_HOLDOFF_SAMPLES;
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

        // Drain forced-AX.25 queue first (XID replies must go via AFSK),
        // then regular tx_queue_ (native or AX.25 depending on mode)
        bool have_frame = false;
        if (!ax25_tx_queue_.empty()) {
            auto frame_data = std::move(ax25_tx_queue_.front());
            ax25_tx_queue_.pop();
            IRIS_LOG("TX frame %zu bytes (forced AX.25)", frame_data.size());
            state_ = ModemState::TX_AX25;
            auto bits = hdlc_encode(frame_data.data(), frame_data.size(), 8, 4);
            if (config_.ax25_baud == 9600)
                tx_buffer_ = gfsk_mod_.modulate(bits);
            else
                tx_buffer_ = afsk_mod_.modulate(bits);
            have_frame = true;
        } else if (!tx_queue_.empty()) {
            if (native_mode_ && native_tx_ready_) {
                // Batch multiple queued frames into one multi-payload frame.
                // Limit total payload based on speed level to cap air time at ~3s.
                int level = gearshift_.current_level();
                int bps_sym = bits_per_symbol(SPEED_LEVELS[level].modulation);
                // Max payload bytes ≈ target_air_symbols × bps / 8 × FEC_rate - CRC
                // Simpler: use a lookup. Higher modes = more payload per second of air.
                // At 2400 baud, 3s = 7200 symbols - 111 overhead = 7089 payload symbols
                // BPSK r1/2: 7089 sym → 7089 bits → 7089/16*800 = ~355 data bits → ~44 bytes (too small!)
                // Actually compute from target air time:
                int fec_n = SPEED_LEVELS[level].fec_rate_num;
                int fec_d = SPEED_LEVELS[level].fec_rate_den;
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
                    if (total_bytes + overhead + front.size() > max_batch)
                        break;
                    total_bytes += overhead + front.size();
                    batch.push_back(std::move(front));
                    tx_queue_.pop();
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

                IRIS_LOG("TX frame %zu bytes (%zu sub-frames, native=1)",
                         frame_data.size(), batch.size());

                state_ = ModemState::TX_NATIVE;
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
                auto frame_data = std::move(tx_queue_.front());
                tx_queue_.pop();
                IRIS_LOG("TX frame %zu bytes (native=0)", frame_data.size());
                state_ = ModemState::TX_AX25;
                auto bits = hdlc_encode(frame_data.data(), frame_data.size(), 8, 4);
                if (config_.ax25_baud == 9600)
                    tx_buffer_ = gfsk_mod_.modulate(bits);
                else
                    tx_buffer_ = afsk_mod_.modulate(bits);
            }
            have_frame = true;
        }

        if (have_frame) {
            IRIS_LOG("TX buffer %zu samples", tx_buffer_.size());

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
        // Defer the data frame — don't send it alongside XID.
        // It will be flushed when XID handshake completes (native mode),
        // or after fallback timeout (remote not Iris-aware).
        deferred_tx_queue_.push(std::vector<uint8_t>(frame, frame + len));
        xid_fallback_ticks_ = 40;  // ~3-4 seconds (tick may run faster than 100ms)
        IRIS_LOG("XID sent, data deferred (%zu bytes), fallback=%d ticks", len, xid_fallback_ticks_);
        return;
    }

    // If XID handshake is in progress (sent but not yet complete), defer data
    if (xid_sent_ && !native_mode_ && !config_.ax25_only) {
        deferred_tx_queue_.push(std::vector<uint8_t>(frame, frame + len));
        IRIS_LOG("Data deferred during XID handshake (%zu bytes)", len);
        return;
    }

    tx_queue_.push(std::vector<uint8_t>(frame, frame + len));
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
    // Always start in AX.25 mode. Native upgrade happens via XID negotiation
    // when the remote station proves it supports Iris native PHY.
    // --ax25-only suppresses XID entirely.
    IRIS_LOG("ARQ listen (native_mode=0, ax25_only=%d)", config_.ax25_only ? 1 : 0);
    arq_.listen();
}

void Modem::tick() {
    arq_.tick();

    // Count down native mode holdoff (set after queuing XID reply)
    // During holdoff, stay in AX.25 mode to decode any remaining AX.25 frames.
    // When holdoff expires, switch both RX and TX to native.
    if (native_tx_holdoff_ > 0) {
        native_tx_holdoff_--;
        if (native_tx_holdoff_ == 0) {
            native_mode_ = true;
            native_tx_ready_ = true;
            // Flush deferred data (now goes via native PHY)
            {
                std::lock_guard<std::mutex> lock(tx_mutex_);
                while (!deferred_tx_queue_.empty()) {
                    tx_queue_.push(std::move(deferred_tx_queue_.front()));
                    deferred_tx_queue_.pop();
                }
            }
            IRIS_LOG("Native holdoff expired, native mode active (RX+TX)");
        }
    }

    // XID fallback: if remote didn't reply, flush deferred data as AX.25
    if (xid_fallback_ticks_ > 0) {
        xid_fallback_ticks_--;
        if (xid_fallback_ticks_ == 0 && !native_mode_) {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            int count = 0;
            while (!deferred_tx_queue_.empty()) {
                tx_queue_.push(std::move(deferred_tx_queue_.front()));
                deferred_tx_queue_.pop();
                count++;
            }
            if (count > 0)
                IRIS_LOG("XID fallback: flushed %d deferred frames as AX.25", count);
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
