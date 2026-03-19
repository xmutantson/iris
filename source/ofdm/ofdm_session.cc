#include "ofdm/ofdm_session.h"
#include "config/config.h"
#include "common/logging.h"
#include <algorithm>
#include <cmath>

namespace iris {

// Map speed level to FEC rate (matches OFDM_SPEED_LEVELS[] table).
// O0=BPSK r1/2, O1=QPSK r1/2, O2=QPSK r3/4, O3=16QAM r1/2,
// O4=16QAM r3/4, O5=64QAM r3/4, O6=64QAM r7/8, O7=256QAM r7/8
static LdpcRate speed_level_to_fec(int level) {
    switch (level) {
        case 0:  return LdpcRate::RATE_1_2;   // BPSK r1/2
        case 1:  return LdpcRate::RATE_1_2;   // QPSK r1/2
        case 2:  return LdpcRate::RATE_3_4;   // QPSK r3/4
        case 3:  return LdpcRate::RATE_1_2;   // 16QAM r1/2
        case 4:  return LdpcRate::RATE_3_4;   // 16QAM r3/4
        case 5:  return LdpcRate::RATE_3_4;   // 64QAM r3/4
        case 6:  return LdpcRate::RATE_7_8;   // 64QAM r7/8
        case 7:  return LdpcRate::RATE_7_8;   // 256QAM r7/8
        default: return LdpcRate::RATE_1_2;
    }
}

static const char* state_name(OfdmSessionState s) {
    switch (s) {
        case OfdmSessionState::IDLE:       return "IDLE";
        case OfdmSessionState::NEGOTIATED: return "NEGOTIATED";
        case OfdmSessionState::PROBING:    return "PROBING";
        case OfdmSessionState::ACTIVE:     return "ACTIVE";
        case OfdmSessionState::FAILED:     return "FAILED";
    }
    return "UNKNOWN";
}

static const char* fec_name(LdpcRate r) {
    switch (r) {
        case LdpcRate::RATE_1_2: return "1/2";
        case LdpcRate::RATE_5_8: return "5/8";
        case LdpcRate::RATE_3_4: return "3/4";
        case LdpcRate::RATE_7_8: return "7/8";
        default:                 return "?";
    }
}

OfdmSession::OfdmSession() = default;
OfdmSession::~OfdmSession() = default;

void OfdmSession::init(const NegotiatedPassband& passband, const IrisConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Create OfdmConfig from negotiated passband
    ofdm_config_ = ofdm_config_from_probe(passband, config.ofdm_nfft, config.ofdm_cp_samples);
    ofdm_config_.fm_preemph_corner_hz = config.ofdm_preemph_corner_hz;

    // Create modulator and demodulator
    modulator_ = std::make_unique<OfdmModulator>(ofdm_config_);
    demodulator_ = std::make_unique<OfdmDemodulator>(ofdm_config_);

    // Store config flags
    waterfill_enabled_ = config.ofdm_waterfill;
    nuc_enabled_ = config.ofdm_nuc;

    // Initialize with default tone map: QPSK r1/2 (preset 2)
    speed_level_ = 0;
    tx_tone_map_ = make_uniform_tone_map(2, ofdm_config_.n_data_carriers, ofdm_config_.nfft);
    tx_tone_map_.fec_rate = LdpcRate::RATE_1_2;
    rx_tone_map_ = tx_tone_map_;

    // Reset statistics
    n_frames_tx_ = 0;
    n_frames_rx_ = 0;
    n_frames_rx_ok_ = 0;
    n_frames_rx_fail_ = 0;
    last_snr_db_ = 0;

    state_ = OfdmSessionState::NEGOTIATED;

    IRIS_LOG("[OFDM-SESSION] init: nfft=%d cp=%d bw=%.0f Hz center=%.0f Hz "
             "carriers=%d(data=%d pilot=%d) waterfill=%s nuc=%s -> %s",
             ofdm_config_.nfft, ofdm_config_.cp_samples,
             ofdm_config_.bandwidth_hz, ofdm_config_.center_hz,
             ofdm_config_.n_used_carriers, ofdm_config_.n_data_carriers,
             ofdm_config_.n_pilot_carriers,
             waterfill_enabled_ ? "on" : "off",
             nuc_enabled_ ? "on" : "off",
             state_name(state_));
}

void OfdmSession::reset() {
    std::lock_guard<std::mutex> lock(mutex_);

    OfdmSessionState old_state = state_;
    state_ = OfdmSessionState::IDLE;
    speed_level_ = 0;
    modulator_.reset();
    demodulator_.reset();
    tx_tone_map_ = {};
    rx_tone_map_ = {};
    n_frames_tx_ = 0;
    n_frames_rx_ = 0;
    n_frames_rx_ok_ = 0;
    n_frames_rx_fail_ = 0;
    last_snr_db_ = 0;

    IRIS_LOG("[OFDM-SESSION] reset: %s -> IDLE", state_name(old_state));
}

void OfdmSession::on_cap_ofdm_negotiated() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == OfdmSessionState::IDLE) {
        state_ = OfdmSessionState::NEGOTIATED;
        IRIS_LOG("[OFDM-SESSION] CAP_OFDM negotiated -> NEGOTIATED");
    }
}

void OfdmSession::on_probe_complete(const ProbeResult& local_rx, const ProbeResult& remote_rx) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ != OfdmSessionState::NEGOTIATED && state_ != OfdmSessionState::PROBING) {
        IRIS_LOG("[OFDM-SESSION] on_probe_complete: ignoring in state %s", state_name(state_));
        return;
    }

    LdpcRate fec = speed_level_to_fec(speed_level_);

    if (waterfill_enabled_) {
        // Compute waterfill tone map from probe data
        tx_tone_map_ = compute_tone_map_from_probe(local_rx, remote_rx, fec, ofdm_config_);
        IRIS_LOG("[OFDM-SESSION] waterfill tone map: %d data carriers, %d total bits/sym, fec=%s",
                 tx_tone_map_.n_data_carriers, tx_tone_map_.total_bits_per_symbol, fec_name(fec));
    } else {
        // Uniform tone map based on current speed level.
        // O-levels map 1:1 to uniform presets: O0=1(BPSK r1/2) through O7=8(256QAM r7/8)
        uint8_t preset = static_cast<uint8_t>(speed_level_ + 1);
        tx_tone_map_ = make_uniform_tone_map(preset, ofdm_config_.n_data_carriers, ofdm_config_.nfft);
        tx_tone_map_.fec_rate = fec;
        IRIS_LOG("[OFDM-SESSION] uniform tone map preset %d: %d bits/sym, fec=%s",
                 preset, tx_tone_map_.total_bits_per_symbol, fec_name(fec));
    }

    // Both sides start with the same tone map; peer updates come via set_peer_tone_map
    rx_tone_map_ = tx_tone_map_;

    float throughput = tone_map_throughput(tx_tone_map_, ofdm_config_);

    state_ = OfdmSessionState::ACTIVE;

    IRIS_LOG("[OFDM-SESSION] probe complete -> ACTIVE, O%d, est throughput=%.0f bps",
             speed_level_, throughput);
}

std::vector<std::complex<float>> OfdmSession::build_tx_frame(const uint8_t* payload, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ != OfdmSessionState::ACTIVE || !modulator_) {
        IRIS_LOG("[OFDM-SESSION] build_tx_frame: not active (state=%s)", state_name(state_));
        return {};
    }

    LdpcRate fec = speed_level_to_fec(speed_level_);
    auto iq = modulator_->build_ofdm_frame(payload, len, tx_tone_map_, fec);

    n_frames_tx_++;

    if ((n_frames_tx_ % 100) == 1) {
        IRIS_LOG("[OFDM-SESSION] TX frame #%d: %zu payload bytes, %zu IQ samples, O%d fec=%s",
                 n_frames_tx_, len, iq.size(), speed_level_, fec_name(fec));
    }

    return iq;
}

OfdmDemodResult OfdmSession::process_rx_frame(const std::complex<float>* iq, int n_samples) {
    std::lock_guard<std::mutex> lock(mutex_);

    OfdmDemodResult result{};

    if (state_ != OfdmSessionState::ACTIVE || !demodulator_) {
        IRIS_LOG("[OFDM-SESSION] process_rx_frame: not active (state=%s)", state_name(state_));
        result.success = false;
        return result;
    }

    result = demodulator_->demodulate(iq, n_samples, rx_tone_map_);

    n_frames_rx_++;
    if (result.success) {
        n_frames_rx_ok_++;

        // Update SNR from demod result
        if (result.snr_db > 0) {
            last_snr_db_ = result.snr_db;
        }
    } else {
        n_frames_rx_fail_++;
    }

    if ((n_frames_rx_ % 50) == 1 || !result.success) {
        IRIS_LOG("[OFDM-SESSION] RX frame #%d: %s, snr=%.1f dB, %zu bytes "
                 "[ok=%d fail=%d total=%d]",
                 n_frames_rx_, result.success ? "OK" : "FAIL",
                 result.snr_db, result.payload.size(),
                 n_frames_rx_ok_, n_frames_rx_fail_, n_frames_rx_);
    }

    return result;
}

void OfdmSession::update_speed_level(float snr_db) {
    std::lock_guard<std::mutex> lock(mutex_);

    int new_level = ofdm_snr_to_speed_level(snr_db);
    new_level = std::clamp(new_level, 0, NUM_OFDM_SPEED_LEVELS - 1);

    if (new_level != speed_level_) {
        int old_level = speed_level_;
        speed_level_ = new_level;

        // Rebuild tone map for new speed level (modulation + FEC rate change together)
        uint8_t preset = static_cast<uint8_t>(speed_level_ + 1);
        LdpcRate new_fec = speed_level_to_fec(speed_level_);
        tx_tone_map_ = make_uniform_tone_map(preset, ofdm_config_.n_data_carriers, ofdm_config_.nfft);
        tx_tone_map_.fec_rate = new_fec;

        IRIS_LOG("[OFDM-SESSION] speed level O%d -> O%d (snr=%.1f dB, preset=%d, fec=%s)",
                 old_level, speed_level_, snr_db, preset, fec_name(new_fec));
    }

    last_snr_db_ = snr_db;
}

std::vector<uint8_t> OfdmSession::get_serialized_tone_map() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return serialize_tone_map(tx_tone_map_);
}

void OfdmSession::set_peer_tone_map(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);

    ToneMap new_map;
    if (deserialize_tone_map(data, len, new_map)) {
        rx_tone_map_ = new_map;
        IRIS_LOG("[OFDM-SESSION] peer tone map updated: id=%d, %d carriers, %d bits/sym, fec=%s",
                 rx_tone_map_.tone_map_id, rx_tone_map_.n_data_carriers,
                 rx_tone_map_.total_bits_per_symbol, fec_name(rx_tone_map_.fec_rate));
    } else {
        IRIS_LOG("[OFDM-SESSION] peer tone map deserialize FAILED (%zu bytes)", len);
    }
}

void OfdmSession::update_tone_map_from_channel(const std::vector<float>& snr_per_carrier) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!waterfill_enabled_) {
        // Uniform mode: no per-carrier adaptation, only FEC rate changes via update_speed_level
        return;
    }

    LdpcRate fec = speed_level_to_fec(speed_level_);
    ToneMap new_map = compute_waterfill_tone_map(snr_per_carrier, fec, ofdm_config_);

    // Check if tone map actually changed
    bool changed = (new_map.total_bits_per_symbol != tx_tone_map_.total_bits_per_symbol);
    if (!changed && new_map.bits_per_carrier.size() == tx_tone_map_.bits_per_carrier.size()) {
        for (size_t i = 0; i < new_map.bits_per_carrier.size(); i++) {
            if (new_map.bits_per_carrier[i] != tx_tone_map_.bits_per_carrier[i]) {
                changed = true;
                break;
            }
        }
    }

    if (changed) {
        int old_bits = tx_tone_map_.total_bits_per_symbol;
        tx_tone_map_ = new_map;
        tx_tone_map_.fec_rate = fec;

        float throughput = tone_map_throughput(tx_tone_map_, ofdm_config_);
        IRIS_LOG("[OFDM-SESSION] tone map updated from channel: %d -> %d bits/sym, "
                 "est throughput=%.0f bps",
                 old_bits, tx_tone_map_.total_bits_per_symbol, throughput);
    }
}

OfdmSessionDiag OfdmSession::get_diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex_);

    OfdmSessionDiag diag;
    diag.state = state_;
    diag.speed_level = speed_level_;
    diag.mean_snr_db = last_snr_db_;
    diag.n_data_carriers = ofdm_config_.n_data_carriers;
    diag.n_frames_tx = n_frames_tx_;
    diag.n_frames_rx = n_frames_rx_;
    diag.n_frames_rx_ok = n_frames_rx_ok_;
    diag.n_frames_rx_fail = n_frames_rx_fail_;
    diag.current_tone_map = tx_tone_map_;

    if (state_ == OfdmSessionState::ACTIVE) {
        diag.throughput_bps = tone_map_throughput(tx_tone_map_, ofdm_config_);
    }

    return diag;
}

LdpcRate OfdmSession::current_fec_rate() const {
    return speed_level_to_fec(speed_level_);
}

} // namespace iris
