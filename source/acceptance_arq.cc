// Author: xmutantson
#include "acceptance_manifest.h"
#include "acceptance_rc5surg.h"
#include "arq/arq.h"
#include "ax25/ax25_protocol.h"
#include "engine/modem.h"
#include "probe/passband_probe.h"
#include "v2/close_transport.h"
#include "monocypher.h"
#include "common/fft.h"
#include "ofdm/ofdm_frame.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>
#include <cmath>
#include <array>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace iris {

// Author: xmutantson. RC8 uses real samples, demodulation and ARQ callbacks.
// A turn's log supplies the cumulative combine event that the production
// success path resets before delivery. Missing/unreadable observations fail.
class Rc8TurnLog {
public:
    Rc8TurnLog() {
        std::fflush(stdout);
        file_ = std::tmpfile();
        if (file_) {
            saved_ = dup_fd(fd(stdout));
            valid_ = saved_ >= 0 && dup_to(fd(file_), fd(stdout)) >= 0;
        }
    }
    ~Rc8TurnLog() { finish(); }
    std::string finish() {
        if (!file_) return text_;
        std::fflush(stdout);
        if (saved_ >= 0) {
            valid_ = dup_to(saved_, fd(stdout)) >= 0 && valid_;
            close_fd(saved_);
        }
        std::rewind(file_);
        char bytes[4096];
        size_t n;
        while ((n = std::fread(bytes, 1, sizeof(bytes), file_)) != 0)
            text_.append(bytes, n);
        valid_ = !std::ferror(file_) && valid_;
        std::fclose(file_);
        file_ = nullptr;
        std::fwrite(text_.data(), 1, text_.size(), stdout);
        return text_;
    }
    bool valid() const { return valid_; }
private:
#ifdef _WIN32
    static int fd(FILE* f) { return _fileno(f); }
    static int dup_fd(int f) { return _dup(f); }
    static int dup_to(int a, int b) { return _dup2(a, b); }
    static void close_fd(int f) { _close(f); }
#else
    static int fd(FILE* f) { return fileno(f); }
    static int dup_fd(int f) { return dup(f); }
    static int dup_to(int a, int b) { return dup2(a, b); }
    static void close_fd(int f) { close(f); }
#endif
    FILE* file_ = nullptr;
    int saved_ = -1;
    bool valid_ = false;
    std::string text_;
};

struct Rc8Snapshot {
    float snr = 0;
    // Object representation includes private hold/failure/iteration evidence,
    // not just the displayed gear. Compare the same live instance across turns.
    std::array<unsigned char, sizeof(Gearshift)> gear{};
    std::vector<float> tune, chase, harq;
    bool harq_valid = false;
    int notifications = 0, combines = 0;
    uint64_t origin = 0, candidate = 0, epoch = 0, required = 0;
    size_t cursor = 0;
    bool unresolved = false;
    bool persistent_equal(const Rc8Snapshot& b) const {
        return snr == b.snr && gear == b.gear && tune == b.tune &&
            chase == b.chase && harq == b.harq && harq_valid == b.harq_valid &&
            notifications == b.notifications && combines == b.combines &&
            origin == b.origin;
    }
};

// Content fingerprints deliberately exclude acquisition bookkeeping and the
// hand-maintained production generation counters. Vector storage is serialized
// by value, so reallocations cannot hide changed samples or HARQ metadata.
struct Rc8ContentHash {
    uint64_t value = 1469598103934665603ULL;
    void bytes(const void* ptr, size_t size) {
        const auto* p = static_cast<const unsigned char*>(ptr);
        for (size_t i = 0; i < size; ++i) { value ^= p[i]; value *= 1099511628211ULL; }
    }
    template<class T> void scalar(const T& x) { bytes(&x, sizeof(x)); }
    template<class T> void vector(const std::vector<T>& v) {
        scalar(v.size());
        for (const auto& x : v) scalar(x);
    }
    void channel(const OfdmChannelEst& c) {
        vector(c.H); vector(c.snr_per_carrier); vector(c.noise_var);
        scalar(c.mean_snr_db); scalar(c.noise_var_frame);
    }
    void harq(const HarqDecodeResult& r) {
        scalar(r.any_failed); scalar(r.all_failed); scalar(r.num_blocks);
        scalar(r.blocks.size());
        for (const auto& b : r.blocks) {
            scalar(b.converged); scalar(b.iterations); vector(b.data_bits);
        }
        vector(r.stored_llrs); vector(r.sym_phase_var); scalar(r.mod);
        scalar(r.fec); scalar(r.payload_len); scalar(r.harq_flag);
        scalar(r.retx_desc.original_seq); scalar(r.retx_desc.regions.size());
        for (const auto& region : r.retx_desc.regions) {
            scalar(region.block_index); scalar(region.bit_start); scalar(region.bit_count);
        }
        vector(r.retx_desc.retx_bits); vector(r.new_data); vector(r.payload);
    }
};

struct Rc8LinkState {
    float belief = 0, preamble_snr = 0, tune_gain = 0;
    int level = -1, map_level = -1, chase_level = -1, chase_cw = 0;
    int tune_count = 0;
    uint64_t chase_id = 0, chase_epoch = 0, owner_id = 0, owner_epoch = 0;
    size_t owner_llrs = 0;
    bool owner_valid = false;
    uint64_t channel_hash = 0, content_hash = 0;
};

// This is a transport fixture, not an alternate implementation.  It enters the
// real Modem/AX.25 state machines through queue_tx_frame(), dispatch_rx_frame(),
// and tick(), and records only their externally visible seams: frames delivered
// to the local KISS client, frames queued for air, and public state callbacks.
class AcceptanceArqHarness {
public:
    // Author: xmutantson. Failure publication must retire every TX scheduling
    // reference, including acked slots and metadata invisible to pending_frames.
    static bool surgical_tx_retired(const ArqSession& session) {
        if (session.tx_base_ != session.tx_next_ ||
            !session.tx_data_queue_.empty() || !session.tx_record_end_queue_.empty() ||
            !session.tx_record_type_queue_.empty() || !session.tx_record_id_queue_.empty() ||
            session.harq_pending_retx_ || session.transmit_completion_pending_ ||
            session.transmit_commit_deferred_ || session.unprepared_outgoing_originals_ != 0)
            return false;
        for (int i = 0; i < ARQ_WINDOW_SIZE; ++i) {
            const auto& slot = session.tx_window_[i];
            if (slot.sent || !slot.data.empty() || slot.retries != 0 ||
                session.harq_tx_[i].active || !session.harq_tx_[i].encoded_bits.empty())
                return false;
        }
        return true;
    }

    static constexpr const char* LOCAL = "N0ACC";
    static constexpr const char* PEER = "N0REM";

    bool start(uint16_t advertised_caps, bool responder = false) {
        local_call_ = responder ? PEER : LOCAL;
        peer_call_ = responder ? LOCAL : PEER;
        IrisConfig cfg;
        cfg.callsign = local_call_;
        cfg.force_ofdm = false;
        cfg.b2f_unroll = false;
        cfg.dcd_auto = false;
        cfg.data_dir.clear();
        if (!modem_.init(cfg)) return false;

        modem_.set_rx_callback([this](const uint8_t* data, size_t len) {
            client_frames_.emplace_back(data, data + len);
        });
        modem_.set_ax25_state_callback(
            [this](Ax25SessionState state, const std::string&) {
                states_.push_back(state);
            });

        // Seed the production cached-probe path with capabilities decoded from
        // the peer's wire advertisement.  This avoids manufacturing a parallel
        // negotiation policy in the test.
        Modem::ProbeCacheEntry entry;
        entry.negotiated.low_hz = 300.0f;
        entry.negotiated.high_hz = 3000.0f;
        entry.negotiated.center_hz = 1650.0f;
        entry.negotiated.bandwidth_hz = 2700.0f;
        entry.negotiated.valid = true;
        entry.my_tx.low_hz = entry.their_tx.low_hz = 300.0f;
        entry.my_tx.high_hz = entry.their_tx.high_hz = 3000.0f;
        entry.my_tx.tones_detected = entry.their_tx.tones_detected = 40;
        entry.my_tx.valid = entry.their_tx.valid = true;
        entry.my_tx.capabilities = advertised_caps;
        entry.timestamp = std::chrono::steady_clock::now();
        modem_.probe_cache_[peer_call_] = entry;

        const auto local = ax25_make_addr(local_call_);
        const auto peer = ax25_make_addr(peer_call_);
        if (responder) {
            // The destination KISS client answers the origin's establishment.
            modem_.ax25_session_.set_kiss_passthrough(true);
            auto sabm = ax25_build_u(local, peer, AX25_CTRL_SABM, true, true);
            inject_air(sabm, false);
            auto ua = ax25_build_u(peer, local, AX25_CTRL_UA, true, false);
            modem_.queue_tx_frame(ua.data(), ua.size());
        } else {
            auto sabm = ax25_build_u(peer, local, AX25_CTRL_SABM, true, true);
            modem_.queue_tx_frame(sabm.data(), sabm.size());
            auto ua = ax25_build_u(local, peer, AX25_CTRL_UA, true, false);
            inject_air(ua, false);
        }

        // Cache replay performs the real capability intersection and native
        // activation from the CONNECTED state callback.
        modem_.repack_maybe_engage();
        clear_observations();
        return modem_.ax25_state() == Ax25SessionState::CONNECTED;
    }

    // RC4 transport fixture. CONNECT is decoded through production negotiation;
    // native-mode flags substitute only for successful PHY mode activation.
    bool start_native(uint16_t caps) {
        IrisConfig cfg;
        cfg.callsign = LOCAL;
        cfg.b2f_unroll = (caps & CAP_B2F_UNROLL) != 0;
        cfg.dcd_auto = false;
        cfg.data_dir.clear();
        if (!modem_.init(cfg)) return false;
        modem_.set_rx_callback([this](const uint8_t* data, size_t len) {
            client_frames_.emplace_back(data, data + len);
        });
        modem_.arq_.set_local_capabilities(caps);
        ArqFrame connect{ArqType::CONNECT, 0, 0,
            {static_cast<uint8_t>(caps >> 8), static_cast<uint8_t>(caps),
             'N', '0', 'R', 'E', 'M'}};
        native_frame(connect);
        modem_.native_mode_ = true;
        modem_.native_tx_ready_ = true;
        clear_observations();
        return modem_.arq_state() == ArqState::CONNECTED &&
               (modem_.arq_.peer_capabilities() & caps) == caps;
    }

    void native_frame(const ArqFrame& frame) {
        const auto wire = frame.serialize();
        modem_.arq_.on_frame_received(wire.data(), wire.size());
    }

    bool rc8_start(const OfdmConfig& cfg, const ToneMap& map) {
        if (!start_native(CAP_HARQ)) return false;
        modem_.ofdm_config_ = cfg;
        modem_.ofdm_demod_ = std::make_unique<OfdmDemodulator>(cfg);
        modem_.ofdm_mod_ = std::make_unique<OfdmModulator>(cfg);
        modem_.ofdm_tone_map_ = modem_.ofdm_rx_tone_map_ = map;
        modem_.ofdm_phy_active_ = true;
        modem_.ofdm_kiss_ = modem_.ofdm_kiss_tx_ = false;
        modem_.ofdm_rx_lpf_active_ = false;
        modem_.native_selfhear_guard_ = 0;
        modem_.ptt_active_ = modem_.rx_muted_ = false;
        modem_.ofdm_expect_ack_ = false;
        modem_.tune_state_ = TuneState::WAIT_PEER;
        const auto send = modem_.arq_.callbacks_.send_frame;
        modem_.arq_.callbacks_.send_frame =
            [this, send](const uint8_t* bytes, size_t len) {
                ArqFrame frame;
                if (ArqFrame::deserialize(bytes, len, frame) &&
                    frame.type == ArqType::NACK) {
                    ++rc8_notifications;
                    if (frame.flags & 0x40) {
                        ++rc8_harq_notifications;
                        const int seq = frame.seq % ARQ_WINDOW_SIZE;
                        rc8_notified_llrs = modem_.arq_.harq_rx_[seq].stored_llrs;
                    }
                }
                if (send) send(bytes, len);
            };
        return modem_.arq_.role() == ArqRole::RESPONDER &&
               modem_.arq_.negotiated(CAP_HARQ);
    }

    // Inject detection-time state, as the existing paired-session fixtures
    // inject negotiated mode. The sync itself comes from the real detector.
    // Starting another capture preserves link belief and all Chase/HARQ state.
    void rc8_capture(const OfdmSyncResult& principal) {
        const uint64_t end = modem_.ofdm_acquisition_.buffer_origin() +
                             modem_.ofdm_rx_audio_buf_.size();
        modem_.ofdm_rx_audio_buf_.clear();
        modem_.ofdm_rx_iq_.clear();
        modem_.ofdm_acquisition_.reset(end);
        auto* c = modem_.ofdm_acquisition_.remember(principal);
        modem_.ofdm_active_candidate_id_ = c ? c->id : 0;
        modem_.ofdm_pending_sync_ = principal;
        modem_.ofdm_sync_cached_ = c != nullptr;
        modem_.ofdm_pending_required_samples_ = 0;
    }

    void rc8_turn(const float* samples = nullptr, size_t count = 0) {
        static const float empty = 0;
        const int previous_combines = modem_.ofdm_chase_combines_;
        const int previous_total = rc8_total_combines;
        Rc8TurnLog observation;
        modem_.process_rx_native(samples ? samples : &empty, static_cast<int>(count));
        const std::string log = observation.finish();
        rc8_observable = rc8_observable && observation.valid();
        rc8_last_log = log;
        const std::string marker = "[OFDM-RX] Chase combining attempt #";
        size_t pos = 0;
        while ((pos = log.find(marker, pos)) != std::string::npos) {
            ++rc8_total_combines;
            pos += marker.size();
        }
        // A missing event cannot conceal an observable counter increment.
        rc8_observable = rc8_observable && rc8_total_combines - previous_total >=
            std::max(0, modem_.ofdm_chase_combines_ - previous_combines);
    }

    Rc8Snapshot rc8_snapshot() const {
        Rc8Snapshot s;
        s.snr = modem_.snr_db_;
        std::memcpy(s.gear.data(), &modem_.gearshift_, sizeof(Gearshift));
        s.tune = {modem_.tune_my_gain_, float(modem_.tune_frames_measured_)};
        for (int i = 0; i < Modem::TUNE_RAMP_COUNT; ++i) {
            s.tune.push_back(float(modem_.tune_rx_frame_iters_[i]));
            s.tune.push_back(modem_.tune_rx_frame_H_[i]);
            s.tune.push_back(modem_.tune_rx_frame_snr_[i]);
        }
        s.chase = modem_.ofdm_chase_llrs_;
        s.harq = modem_.ofdm_harq_evidence_.stored_llrs;
        s.harq_valid = modem_.ofdm_harq_evidence_valid_;
        s.combines = modem_.ofdm_chase_combines_;
        s.notifications = rc8_notifications;
        s.origin = modem_.ofdm_acquisition_.buffer_origin();
        s.candidate = modem_.ofdm_active_candidate_id_;
        s.epoch = modem_.ofdm_acquisition_.capture_epoch();
        s.required = modem_.ofdm_pending_required_samples_;
        for (const auto& c : modem_.ofdm_acquisition_.candidates()) {
            if (c.id != s.candidate) continue;
            s.cursor = c.cfo_trial_cursor;
            s.unresolved = c.status == OfdmAcquisitionStatus::Ready ||
                           c.status == OfdmAcquisitionStatus::NeedContext;
        }
        return s;
    }

    Rc8LinkState rc8_link() const {
        Rc8LinkState s;
        s.belief = modem_.ofdm_cfo_committed_hz_;
        s.level = modem_.ofdm_speed_level_;
        s.map_level = int(modem_.ofdm_tone_map_.tone_map_id) - 1;
        s.chase_level = modem_.ofdm_chase_level_;
        s.chase_cw = modem_.ofdm_chase_n_codewords_;
        s.chase_id = modem_.ofdm_chase_candidate_id_;
        s.chase_epoch = modem_.ofdm_chase_capture_epoch_;
        s.owner_id = modem_.ofdm_search_owner_candidate_id_;
        s.owner_epoch = modem_.ofdm_search_owner_capture_epoch_;
        s.owner_valid = modem_.ofdm_search_owner_valid_;
        s.owner_llrs = modem_.ofdm_search_owner_result_.llrs.size();
        s.tune_gain = modem_.tune_my_gain_;
        s.tune_count = modem_.tune_frames_measured_;
        s.preamble_snr = modem_.snr_preamble_db_;
        Rc8ContentHash h;
        if (modem_.ofdm_demod_) h.channel(modem_.ofdm_demod_->last_channel_estimate());
        s.channel_hash = h.value;
        h.vector(modem_.ofdm_chase_llrs_);
        h.scalar(s.chase_id); h.scalar(s.chase_epoch); h.scalar(s.chase_level);
        h.scalar(s.chase_cw); h.scalar(modem_.ofdm_chase_fec_rate_);
        h.scalar(modem_.ofdm_chase_combines_);
        h.harq(modem_.ofdm_harq_evidence_);
        h.scalar(modem_.ofdm_harq_evidence_valid_);
        h.scalar(modem_.ofdm_harq_evidence_candidate_id_);
        h.scalar(modem_.ofdm_harq_evidence_capture_epoch_);
        for (const auto& slot : modem_.arq_.harq_rx_) {
            h.vector(slot.stored_llrs); h.vector(slot.sym_phase_var);
            h.scalar(slot.block_decoded.size());
            for (bool decoded : slot.block_decoded) h.scalar(decoded);
            h.scalar(slot.block_data.size());
            for (const auto& data : slot.block_data) h.vector(data);
            h.scalar(slot.num_blocks); h.scalar(slot.fec); h.scalar(slot.mod);
            h.scalar(slot.payload_len); h.scalar(slot.combine_count);
            h.scalar(slot.retx_attempts); h.scalar(slot.active);
        }
        h.scalar(modem_.snr_db_); h.scalar(s.preamble_snr);
        const auto snapshot = rc8_snapshot();
        h.bytes(snapshot.gear.data(), snapshot.gear.size());
        h.vector(snapshot.tune); h.scalar(modem_.tune_state_.load());
        h.scalar(modem_.tune_peer_gain_);
        for (int i = 0; i < Modem::TUNE_RAMP_COUNT; ++i) {
            h.scalar(modem_.tune_peer_iters_[i]); h.scalar(modem_.tune_peer_H_[i]);
            h.scalar(modem_.tune_peer_snr_[i]); h.scalar(modem_.tune_computed_scales_[i]);
        }
        h.scalar(rc8_notifications); h.scalar(rc8_harq_notifications);
        h.scalar(s.belief); h.scalar(s.level);
        s.content_hash = h.value;
        return s;
    }

    void rc8_level(int level, bool prior_failure = false) {
        modem_.ofdm_speed_level_ = level;
        modem_.gearshift_.reset();
        modem_.gearshift_.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        modem_.gearshift_.force_ofdm_level(level);
        if (prior_failure) modem_.gearshift_.report_failure();
    }
    void rc8_blind(int level, int codewords) {
        modem_.ofdm_kiss_tx_ = modem_.ofdm_kiss_ = true;
        modem_.ofdm_kiss_rx_level_ = level;
        modem_.ofdm_kiss_rx_proposed_level_ = level;
        modem_.ofdm_kiss_rx_confirmed_ = true;
        modem_.ofdm_rx_tone_map_ = get_uniform_tone_map(level + 1, modem_.ofdm_config_);
        modem_.ofdm_rx_tone_map_.n_codewords = codewords;
        // Negotiate only the modulation needed by this fixture. CW=1..8
        // remains the production blind enumeration, including other rates.
        modem_.config_.max_modulation = OFDM_SPEED_LEVELS[level].modulation;
    }
    bool rc8_native_teardown() {
        modem_.arq_.reset();
        return modem_.arq_.state() == ArqState::IDLE;
    }
    bool rc8_ack_turnaround() {
        // Enqueue through the installed native control-frame callback, then
        // actually render and drain TX before entering the public RX unmute.
        modem_.tune_state_ = TuneState::DONE;
        const auto ack = ArqFrame{ArqType::ACK, 0, 0, {}}.serialize();
        modem_.arq_.callbacks_.send_frame(ack.data(), ack.size());
        std::array<float, 4096> samples{};
        bool rendered = false, drained = false, muted = false;
        for (int i = 0; i < 256; ++i) {
            modem_.process_tx(samples.data(), int(samples.size()));
            rendered = rendered || std::any_of(samples.begin(), samples.end(),
                [](float x) { return std::fabs(x) > 1e-7f; });
            if (rendered && modem_.tx_buffer_.empty() && !modem_.tx_draining_) {
                drained = true; muted = modem_.rx_muted_; break;
            }
        }
        samples.fill(0);
        for (int i = 0; i < 64 && (modem_.rx_muted_ || modem_.native_selfhear_guard_ > 0); ++i)
            modem_.process_rx(samples.data(), int(samples.size()));
        return rendered && drained && muted && !modem_.rx_muted_ &&
               modem_.arq_.state() == ArqState::CONNECTED;
    }

    void rc8_decompression_mode() {
        modem_.ofdm_kiss_ = true;
        modem_.ofdm_kiss_peer_caps_ |= CAP_COMPRESSION;
    }
    size_t rc8_frames() const { return modem_.frames_rx_; }
    bool rc8_delivered(const std::vector<uint8_t>& payload) const {
        return std::find(client_frames_.begin(), client_frames_.end(), payload) !=
               client_frames_.end();
    }
    int rc8_notifications = 0, rc8_harq_notifications = 0;
    int rc8_total_combines = 0;
    bool rc8_observable = true;
    std::vector<float> rc8_notified_llrs;
    std::string rc8_last_log;

    void native_data(const std::vector<uint8_t>& bytes, uint8_t seq, bool end) {
        native_frame({ArqType::DATA, seq, static_cast<uint8_t>(end ? 0x80 : 0), bytes});
    }

    // Invoke the installed production inverse-transform callback at its complete
    // record boundary. This isolates model ordering from native batch lifetime.
    // No replacement decoder and no test-side RX commit are installed here.
    void complete_native_record(const std::vector<uint8_t>& bytes) {
        modem_.arq_.callbacks_.on_data_received(bytes.data(), bytes.size());
    }

    std::vector<uint8_t> queue_native_record(const std::vector<uint8_t>& bytes) {
        // This endpoint is still the responder: send_data owns/fragments the
        // encoded record while awaiting role switch. Observe that transport
        // queue, not the original-record custody catalog whose representation
        // RC4 must repair. No queue/window/model state is written by the fixture.
        const size_t before = modem_.arq_.tx_data_queue_.size();
        modem_.queue_tx_frame(bytes.data(), bytes.size());
        auto pending = modem_.arq_.tx_data_queue_;
        for (size_t i = 0; i < before && !pending.empty(); ++i) pending.pop();
        std::vector<uint8_t> adopted;
        while (!pending.empty()) {
            adopted.insert(adopted.end(), pending.front().begin(), pending.front().end());
            pending.pop();
        }
        return adopted;
    }

    std::vector<ArqFrame> take_native_air() {
        std::vector<ArqFrame> frames;
        while (!modem_.tx_queue_.empty()) {
            const auto& wire = modem_.tx_queue_.front().data;
            ArqFrame frame;
            if (ArqFrame::deserialize(wire.data(), wire.size(), frame))
                frames.push_back(std::move(frame));
            modem_.tx_queue_.pop();
        }
        return frames;
    }

    std::vector<uint8_t> delivered_bytes() const {
        std::vector<uint8_t> bytes;
        for (const auto& record : client_frames_)
            bytes.insert(bytes.end(), record.begin(), record.end());
        return bytes;
    }
    size_t deliveries() const { return client_frames_.size(); }
    B2fHandler& native_b2f() { return modem_.b2f_handler_; }
    const Compressor& native_tx_codec() const { return modem_.tx_compressor_; }
    const Compressor& native_rx_codec() const { return modem_.rx_compressor_; }
    static int model_count(const Compressor& codec) { return codec.stream_batch_count_; }
    static std::vector<uint8_t> model_prefix(const Compressor& codec) {
        if (!codec.zstd_prefix_ || codec.zstd_prefix_len_ <= 0) return {};
        return {codec.zstd_prefix_, codec.zstd_prefix_ + codec.zstd_prefix_len_};
    }
    static bool same_model(const Compressor& a, const Compressor& b) {
        return model_count(a) == model_count(b) && model_prefix(a) == model_prefix(b);
    }
    std::vector<std::shared_ptr<const ArqTransferResult>> native_results() const {
        return modem_.arq_.retained_transfer_results();
    }
    bool native_disconnected() const {
        return modem_.arq_state() == ArqState::IDLE ||
               modem_.arq_state() == ArqState::DISCONNECTING;
    }

    // Establish real matching cipher keys for the split-tag receive fixture.
    bool pair_native_cipher(CipherSuite& sender) {
        uint8_t tx_key[X25519_KEY_SIZE], rx_key[X25519_KEY_SIZE];
        if (sender.generate_x25519_keypair(tx_key) != 0 ||
            modem_.cipher_.generate_x25519_keypair(rx_key) != 0 ||
            sender.compute_x25519_shared(rx_key) != 0 ||
            modem_.cipher_.compute_x25519_shared(tx_key) != 0) return false;
        sender.derive_session_key(PEER, LOCAL, nullptr, 0, false);
        modem_.cipher_.derive_session_key(LOCAL, PEER, nullptr, 0, false);
        sender.activate();
        modem_.cipher_.activate();
        modem_.crypto_direction_ = DIR_CMD_TO_RSP ^ 1;
        modem_.rx_batch_counter_ = 0;
        uint8_t tx_confirmation[8], rx_confirmation[8];
        sender.compute_key_confirmation(tx_confirmation);
        modem_.cipher_.compute_key_confirmation(rx_confirmation);
        return std::equal(tx_confirmation, tx_confirmation + 8, rx_confirmation);
    }

    void clear_observations() {
        client_frames_.clear();
        states_.clear();
        while (!modem_.tx_queue_.empty()) modem_.tx_queue_.pop();
        while (!modem_.ax25_tx_queue_.empty()) modem_.ax25_tx_queue_.pop();
    }

    std::vector<uint8_t> queue_client_iframe(uint8_t ns,
                                              const std::vector<uint8_t>& info) {
        auto frame = ax25_build_i(ax25_make_addr(PEER), ax25_make_addr(LOCAL),
                                  ns, 0, false, AX25_PID_NONE,
                                  info.data(), info.size());
        modem_.queue_tx_frame(frame.data(), frame.size());
        return frame;
    }

    void queue_client_disc() {
        auto frame = ax25_build_u(ax25_make_addr(PEER), ax25_make_addr(LOCAL),
                                  AX25_CTRL_DISC, true, true);
        modem_.queue_tx_frame(frame.data(), frame.size());
    }

    void inject_peer_rr(uint8_t nr) {
        auto frame = ax25_build_s(ax25_make_addr(LOCAL), ax25_make_addr(PEER),
                                  Ax25SType::RR, nr, false, false,
                                  modem_.ax25_session_.extended());
        inject_air(frame, true);
    }

    void inject_peer_ua() {
        auto frame = ax25_build_u(ax25_make_addr(LOCAL), ax25_make_addr(PEER),
                                  AX25_CTRL_UA, true, false);
        inject_air(frame, false);
    }

    // Author: xmutantson. Only a validator-produced, move-only proof enters here.
    bool inject_peer_close_proof(v2::MatchingCloseProof&& proof) {
        return modem_.accept_remote_close_proof(std::move(proof));
    }

    v2::LiveSession* close_session() {
        return v2::CloseTransportOwner::session(modem_);
    }

    std::optional<v2::LiveReceiveContext> receive_close_control(
        std::vector<uint8_t> bytes) {
        return v2::TransportReceiveOwner::receive_close_control(
            modem_, std::move(bytes));
    }

    v2::TransferLedger* origin_ledger() {
        return modem_.repack_transfer_ledger_.get();
    }

    const v2::TransferLedger* receiver_ledger() const {
        return modem_.repack_rx_transfer_ledger_.get();
    }

    bool awaiting_remote_close() const {
        return modem_.repack_transfer_state_ ==
            Modem::RepackTransferState::AwaitingRemoteClose;
    }

    std::vector<Modem::TransferResultPtr> retained_results() const {
        return modem_.retained_transfer_results();
    }

    size_t client_count(Ax25UType type) const {
        size_t count = 0;
        for (const auto& bytes : client_frames_) {
            Ax25Frame frame;
            if (ax25_parse(bytes.data(), bytes.size(), frame) &&
                frame.type() == Ax25FrameType::U_FRAME && frame.u_type() == type)
                ++count;
        }
        return count;
    }

    bool client_received_record(const std::vector<uint8_t>& expected) const {
        size_t count = 0;
        for (const auto& bytes : client_frames_) {
            Ax25Frame frame;
            if (ax25_parse(bytes.data(), bytes.size(), frame) &&
                frame.type() == Ax25FrameType::I_FRAME) {
                if (frame.ns() != 0 || frame.info != expected) return false;
                ++count;
            }
        }
        return count == 1;
    }

    size_t deliver_air_data_to(AcceptanceArqHarness& receiver) {
        size_t count = 0;
        // Transport only production-emitted I-frames; leave the held DISC and
        // control exchange under the strict fixture's explicit ordering.
        auto frames = modem_.tx_queue_;
        while (!frames.empty()) {
            const auto& bytes = frames.front().data;
            Ax25Frame frame;
            if (ax25_parse(bytes.data(), bytes.size(), frame,
                           modem_.ax25_session_.extended()) &&
                frame.type() == Ax25FrameType::I_FRAME) {
                receiver.inject_decoded_native_data(bytes);
                ++count;
            }
            frames.pop();
        }
        return count;
    }

    void queue_r3_rr(uint8_t nr) {
        auto frame = ax25_build_s(ax25_make_addr(peer_call_),
                                  ax25_make_addr(local_call_),
                                  Ax25SType::RR, nr, false, false);
        modem_.queue_tx_frame(frame.data(), frame.size());
    }

    void inject_peer_sabm() {
        auto frame = ax25_build_u(ax25_make_addr(LOCAL), ax25_make_addr(PEER),
                                  AX25_CTRL_SABM, true, true);
        inject_air(frame, true);
    }

    void set_short_ack_timeout() { modem_.ax25_session_.set_t1_ticks(1); }

    void tick(int count = 1) {
        for (int i = 0; i < count; ++i) modem_.tick();
    }

    bool client_saw(Ax25UType type) const {
        for (const auto& bytes : client_frames_) {
            Ax25Frame frame;
            if (ax25_parse(bytes.data(), bytes.size(), frame) &&
                frame.type() == Ax25FrameType::U_FRAME && frame.u_type() == type)
                return true;
        }
        return false;
    }

    bool client_saw_rr() const {
        for (const auto& bytes : client_frames_) {
            Ax25Frame frame;
            if (ax25_parse(bytes.data(), bytes.size(), frame) &&
                frame.type() == Ax25FrameType::S_FRAME &&
                frame.s_type() == Ax25SType::RR)
                return true;
        }
        return false;
    }

    bool reached(Ax25SessionState state) const {
        for (Ax25SessionState seen : states_)
            if (seen == state) return true;
        return false;
    }

    bool air_contains_exactly(const std::vector<uint8_t>& expected) const {
        auto native = modem_.tx_queue_;
        while (!native.empty()) {
            if (native.front().data == expected) return true;
            native.pop();
        }
        auto legacy = modem_.ax25_tx_queue_;
        while (!legacy.empty()) {
            if (legacy.front() == expected) return true;
            legacy.pop();
        }
        return false;
    }

private:
    void inject_decoded_native_data(const std::vector<uint8_t>& frame) {
        // This fixture substitutes for the PHY decoder. Carry its native-frame
        // indication across the same responder activation seam as deliver_ofdm
        // in process_rx_native(), before dispatch parses the extended R2 frame.
        // These are transport-mode flags only; custody still enters through RX.
        if (modem_.ofdm_kiss_ && !modem_.ofdm_config_mismatch_) {
            modem_.ofdm_kiss_confirmed_ = true;
            if (!modem_.ofdm_kiss_tx_) {
                modem_.ofdm_kiss_tx_ = true;
                modem_.ax25_session_.set_native_active(true);
                modem_.ax25_session_.start_t1_if_unacked();
            }
        }
        inject_air(frame, true);
    }

    void inject_air(const std::vector<uint8_t>& frame, bool from_ofdm) {
        modem_.dispatch_rx_frame(frame, false, from_ofdm);
    }

    Modem modem_;
    const char* local_call_ = LOCAL;
    const char* peer_call_ = PEER;
    std::vector<std::vector<uint8_t>> client_frames_;
    std::vector<Ax25SessionState> states_;
};

bool acceptance_rc5surg_tx_retired(const ArqSession& session) {
    return AcceptanceArqHarness::surgical_tx_retired(session);
}

} // namespace iris

namespace {

using namespace iris;

void acceptance_check(const char* name, bool ok) {
    acceptance_manifest_record(name, ok);
}

uint16_t decode_peer_caps(bool v2) {
    ProbeResult advertised;
    advertised.low_hz = 300.0f;
    advertised.high_hz = 3000.0f;
    advertised.tones_detected = 40;
    advertised.valid = true;
    advertised.capabilities = v2 ? CAP_OFDM : 0;
    auto wire = probe_result_encode(advertised);

    // A legacy peer ends at the v1 body and has no capability extension.
    if (!v2) wire.resize(19);
    ProbeResult decoded;
    if (!probe_result_decode(wire.data(), wire.size(), decoded)) return 0;
    return decoded.capabilities;
}

// Author: xmutantson. Independent Profile-1 encoding for the exchange oracle.
void close_append_be(std::vector<uint8_t>& bytes, uint64_t value, unsigned width) {
    for (unsigned i = width; i != 0; --i)
        bytes.push_back(static_cast<uint8_t>(value >> ((i - 1) * 8)));
}

void close_append_transfer(std::vector<uint8_t>& bytes,
                           const v2::TransferIdentity& transfer) {
    bytes.insert(bytes.end(), transfer.session_id.bytes.begin(),
                 transfer.session_id.bytes.end());
    bytes.push_back(static_cast<uint8_t>(transfer.direction));
    close_append_be(bytes, transfer.transfer_id.value, 8);
}

v2::FrozenFinalRecord expected_close_boundary(const v2::TransferLedger& ledger) {
    v2::FrozenFinalRecord boundary;
    boundary.empty_transfer = ledger.original_records.empty();
    boundary.record_count = ledger.original_records.size();
    if (!ledger.original_records.empty()) {
        boundary.final_record_id = ledger.original_records.back().record_id;
        boundary.final_record_extent = ledger.original_records.back().bytes.size();
    }
    const char prefix[] = "IRIS-V2-CATALOG";
    std::vector<uint8_t> bytes(prefix, prefix + sizeof(prefix) - 1);
    close_append_transfer(bytes, ledger.transfer);
    // Profile 1 uses u32 for vector counts; individual IDs/extents are u64.
    close_append_be(bytes, ledger.original_records.size(), 4);
    for (const auto& record : ledger.original_records) {
        close_append_be(bytes, record.record_id.value, 8);
        close_append_be(bytes, record.bytes.size(), 8);
    }
    crypto_blake2b(boundary.catalog_digest.data(), boundary.catalog_digest.size(),
                   bytes.data(), bytes.size());
    return boundary;
}

bool close_same_transfer(const v2::TransferIdentity& a,
                          const v2::TransferIdentity& b) {
    return a.session_id.bytes == b.session_id.bytes && a.direction == b.direction &&
           a.transfer_id.value == b.transfer_id.value;
}

bool close_same_r3(const v2::R3DomainIdentity& a, const v2::R3DomainIdentity& b) {
    return a.remote_client_connection.bytes == b.remote_client_connection.bytes &&
           a.sequence_space.value == b.sequence_space.value;
}

bool close_same_boundary(const v2::FrozenFinalRecord& a,
                          const v2::FrozenFinalRecord& b) {
    return a.empty_transfer == b.empty_transfer && a.record_count == b.record_count &&
           a.final_record_id.value == b.final_record_id.value &&
           a.final_record_extent == b.final_record_extent &&
           a.catalog_digest == b.catalog_digest;
}

template<size_t N>
bool close_nonzero(const std::array<uint8_t, N>& bytes) {
    return std::any_of(bytes.begin(), bytes.end(), [](uint8_t b) { return b != 0; });
}

bool close_full_record(const v2::OriginalRecordCoverage& coverage) {
    return coverage.completed_empty_records.empty() && coverage.ranges.size() == 1 &&
           coverage.ranges[0].record_id.value == 1 &&
           coverage.ranges[0].offset == 0 && coverage.ranges[0].length == 5;
}

bool close_no_records(const v2::OriginalRecordCoverage& coverage) {
    return coverage.ranges.empty() && coverage.completed_empty_records.empty();
}

// Serialize the complete peer-emitted object, including its own tag. The
// protected receive validator must independently verify this tag with the owned
// message-sender key; a locally populated IntegrityInfo cannot grant authority.
template<typename Message>
std::vector<uint8_t> close_wire(const Message& message,
                                const v2::FrozenFinalRecord& boundary) {
    std::vector<uint8_t> bytes;
    close_append_transfer(bytes, message.transfer);
    bytes.insert(bytes.end(), message.r3.remote_client_connection.bytes.begin(),
                 message.r3.remote_client_connection.bytes.end());
    close_append_be(bytes, message.r3.sequence_space.value, 8);
    bytes.push_back(boundary.empty_transfer ? 1 : 0);
    close_append_be(bytes, boundary.record_count, 8);
    close_append_be(bytes, boundary.final_record_id.value, 8);
    close_append_be(bytes, boundary.final_record_extent, 8);
    bytes.insert(bytes.end(), boundary.catalog_digest.begin(), boundary.catalog_digest.end());
    close_append_be(bytes, message.close_id, 8);
    bytes.insert(bytes.end(), message.challenge.bytes.begin(), message.challenge.bytes.end());
    bytes.push_back(static_cast<uint8_t>(message.integrity.algorithm));
    bytes.push_back(static_cast<uint8_t>(message.integrity.domain));
    close_append_be(bytes, message.integrity.profile_version, 2);
    close_append_be(bytes, message.integrity.value.size(), 8);
    bytes.insert(bytes.end(), message.integrity.value.begin(), message.integrity.value.end());
    return bytes;
}

template<typename Message>
bool close_message_matches(const Message& message, const v2::CloseRequest& request,
                            v2::ProtectionDomain domain) {
    return close_same_transfer(message.transfer, request.transfer) &&
           close_same_r3(message.r3, request.r3) &&
           close_same_boundary(message.accepted_final_record, request.expected_final_record) &&
           message.close_id == request.close_id &&
           message.challenge.bytes == request.challenge.bytes &&
           message.integrity.algorithm == v2::IntegrityAlgorithm::Blake2b256Keyed &&
           message.integrity.domain == domain &&
           message.integrity.profile_version == v2::kProtectionProfileVersion &&
           message.integrity.value.size() == 32;
}

// Separate fresh transfer: ordinary AX.25 UA must fail even with complete R2.
// Its observation joins the strict check, without registering an eleventh check.
bool strict_ordinary_ua_rejected() {
    AcceptanceArqHarness h;
    const bool ready = h.start(decode_peer_caps(true));
    h.queue_client_iframe(0, {'p', 'r', 'o', 'o', 'f'});
    h.queue_client_disc();
    h.tick(8);
    h.inject_peer_rr(1);
    h.tick();
    const bool awaiting = h.awaiting_remote_close();
    const bool no_early_result = h.retained_results().empty();
    const size_t ua_before = h.client_count(Ax25UType::UA);
    h.inject_peer_ua();
    const auto results = h.retained_results();
    const bool rejected = ready && awaiting && no_early_result && ua_before == 0 &&
        h.client_count(Ax25UType::UA) == 0 && results.size() == 1 &&
        results[0]->outcome == v2::TransferOutcome::Failed &&
        results[0]->reason == v2::TransferResultReason::ProtocolViolation &&
        !results[0]->close_proof;
    std::printf("ACCEPTANCE_OBS name=acc_strict_end_to_end_custody case=ordinary_ua rejected=%d client_ua=%zu retained=%zu\n",
                rejected, h.client_count(Ax25UType::UA), results.size());
    return rejected;
}

void acc_strict_end_to_end_custody() {
    AcceptanceArqHarness h;
    AcceptanceArqHarness peer;
    const bool ready = h.start(decode_peer_caps(true));
    const bool peer_ready = peer.start(decode_peer_caps(true), true);
    const std::vector<uint8_t> payload{'p', 'r', 'o', 'o', 'f'};
    h.queue_client_iframe(0, payload);
    h.queue_client_disc();
    const size_t ua_before_r2 = h.client_count(Ax25UType::UA);
    h.tick(8);
    const size_t ua_before_rr = h.client_count(Ax25UType::UA);
    const size_t air_frames = h.deliver_air_data_to(peer);
    h.inject_peer_rr(1);  // R2 custody only, never selected-client acceptance.
    h.tick();
    const size_t ua_after_r2 = h.client_count(Ax25UType::UA);
    const bool awaiting = h.awaiting_remote_close();
    const bool no_r2_result = h.retained_results().empty();
    int ua_after_attestation = -1;
    int ua_after_receipt = -1;
    bool r3_accepted = false;
    bool request_registered = false;
    bool confirmation_received = false;
    bool proof_validated = false;
    const char* stage = "admitted-catalog";

    const bool exchange_ok = [&]() {
        auto* ledger = h.origin_ledger();
        const auto* remote = peer.receiver_ledger();
        if (!ready || !peer_ready || !awaiting || !ledger || !remote ||
            !h.client_saw_rr() || air_frames != 1 ||
            !peer.client_received_record(payload) ||
            ledger->original_records.size() != 1 ||
            ledger->original_records[0].record_id.value != 1 ||
            ledger->original_records[0].bytes != payload ||
            !close_full_record(ledger->milestones.accepted) ||
            !close_full_record(ledger->milestones.air_acknowledged) ||
            !close_no_records(ledger->milestones.endpoint_acknowledged) ||
            !close_no_records(remote->milestones.endpoint_acknowledged)) return false;
        const auto transfer = ledger->transfer;
        const auto r3 = ledger->domains.r3;
        const auto boundary = expected_close_boundary(*ledger);
        if (boundary.empty_transfer || boundary.record_count != 1 ||
            boundary.final_record_id.value != 1 || boundary.final_record_extent != 5 ||
            !close_nonzero(transfer.session_id.bytes) || transfer.transfer_id.value == 0 ||
            !close_nonzero(r3.remote_client_connection.bytes) || r3.sequence_space.value == 0 ||
            ledger->domains.r2.modem_session.bytes != transfer.session_id.bytes) return false;

        stage = "live-session";
        auto* origin_session = h.close_session();
        auto* receiver_session = peer.close_session();
        if (!origin_session || !receiver_session ||
            !origin_session->active() || !receiver_session->active() ||
            origin_session->identity().session_id.bytes != transfer.session_id.bytes ||
            receiver_session->identity().session_id.bytes != transfer.session_id.bytes ||
            transfer.direction != v2::TransferDirection::InitiatorToResponder) return false;

        stage = "registered-request";
        v2::CloseProofValidationError error = v2::CloseProofValidationError::NoOutstandingRequest;
        auto request = v2::CloseExchangeOwner::begin(*origin_session, *ledger, error);
        if (!request || error != v2::CloseProofValidationError::None ||
            !ledger->frozen_final_record ||
            !close_same_boundary(*ledger->frozen_final_record, boundary) ||
            !close_same_boundary(request->expected_final_record, boundary) ||
            !close_same_transfer(request->transfer, transfer) ||
            !close_same_r3(request->r3, r3) || request->close_id == 0 ||
            !close_nonzero(request->challenge.bytes) ||
            request->integrity.algorithm != v2::IntegrityAlgorithm::Blake2b256Keyed ||
            request->integrity.domain != v2::ProtectionDomain::CloseRequest ||
            request->integrity.profile_version != v2::kProtectionProfileVersion ||
            request->integrity.value.size() != 32) return false;
        request_registered = true;

        stage = "received-request";
        auto request_rx = peer.receive_close_control(close_wire(*request, boundary));
        if (!request_rx) return false;
        auto received_request = v2::CloseReceiveValidator::validate_request(
            *request_rx, *request, error);
        if (!received_request || error != v2::CloseProofValidationError::None ||
            !close_same_transfer(remote->transfer, transfer) ||
            !close_same_r3(remote->domains.r3, r3) ||
            !close_same_r3(remote->r3_sequence.endpoint_cursor.domain, r3) ||
            remote->original_records.size() != 1 ||
            remote->original_records[0].record_id.value != 1 ||
            remote->original_records[0].bytes != payload ||
            !remote->frozen_final_record ||
            !close_same_boundary(*remote->frozen_final_record, boundary) ||
            !close_same_boundary(expected_close_boundary(*remote), boundary)) return false;

        stage = "r3-cumulative-acceptance";
        // Receiving/writing the record alone must not allow an attestation.
        auto premature = v2::CloseExchangeOwner::send_attestation(
            *receiver_session, transfer, error);
        if (premature || error == v2::CloseProofValidationError::None ||
            remote->r3_sequence.endpoint_cursor.next_absolute_position != 0) return false;
        peer.queue_r3_rr(1);  // Actual selected local client -> production R3 window.
        remote = peer.receiver_ledger();
        r3_accepted = remote && close_same_transfer(remote->transfer, transfer) &&
            close_same_r3(remote->r3_sequence.endpoint_cursor.domain, r3) &&
            remote->r3_sequence.endpoint_cursor.next_absolute_position == 1 &&
            close_full_record(remote->milestones.endpoint_acknowledged);
        if (!r3_accepted) return false;

        stage = "received-attestation";
        auto attestation = v2::CloseExchangeOwner::send_attestation(
            *receiver_session, transfer, error);
        if (!attestation || error != v2::CloseProofValidationError::None ||
            !close_message_matches(*attestation, *request,
                                    v2::ProtectionDomain::RemoteCloseAttestation)) return false;
        auto attestation_rx = h.receive_close_control(close_wire(*attestation, boundary));
        if (!attestation_rx) return false;
        auto received_attestation = v2::CloseReceiveValidator::validate_attestation(
            *attestation_rx, *attestation, error);
        if (!received_attestation || error != v2::CloseProofValidationError::None) return false;
        ua_after_attestation = static_cast<int>(h.client_count(Ax25UType::UA));
        if (ua_after_attestation != 0 || !h.retained_results().empty()) return false;

        stage = "received-receipt";
        auto receipt = v2::CloseExchangeOwner::send_receipt(*origin_session, transfer, error);
        if (!receipt || error != v2::CloseProofValidationError::None ||
            !close_message_matches(*receipt, *request,
                                    v2::ProtectionDomain::CloseAttestationReceipt)) return false;
        auto receipt_rx = peer.receive_close_control(close_wire(*receipt, boundary));
        if (!receipt_rx) return false;
        auto received_receipt = v2::CloseReceiveValidator::validate_receipt(
            *receipt_rx, *receipt, error);
        if (!received_receipt || error != v2::CloseProofValidationError::None) return false;
        ua_after_receipt = static_cast<int>(h.client_count(Ax25UType::UA));
        if (ua_after_receipt != 0 || !h.retained_results().empty()) return false;

        stage = "received-confirmation";
        auto confirmation = v2::CloseExchangeOwner::send_confirmation(
            *receiver_session, transfer, error);
        if (!confirmation || error != v2::CloseProofValidationError::None ||
            !close_message_matches(*confirmation, *request,
                                    v2::ProtectionDomain::CloseConfirmation)) return false;
        auto confirmation_rx = h.receive_close_control(close_wire(*confirmation, boundary));
        if (!confirmation_rx) return false;
        auto received_confirmation = v2::CloseReceiveValidator::validate_confirmation(
            *confirmation_rx, *confirmation, error);
        confirmation_received = received_confirmation.has_value() &&
            error == v2::CloseProofValidationError::None;
        if (!confirmation_received) return false;

        stage = "matching-proof";
        auto validated = v2::CloseProofValidator::validate_received_messages(
            *confirmation_rx, std::move(*received_attestation),
            std::move(*received_confirmation));
        proof_validated = validated.error == v2::CloseProofValidationError::None &&
            validated.proof.has_value();
        if (!proof_validated ||
            !close_message_matches(validated.proof->remote_attestation(), *request,
                                    v2::ProtectionDomain::RemoteCloseAttestation) ||
            !close_message_matches(validated.proof->confirmation(), *request,
                                    v2::ProtectionDomain::CloseConfirmation)) return false;
        // No writes to milestones, result queues or client frames in this seam.
        stage = "terminal-publication";
        if (!h.inject_peer_close_proof(std::move(*validated.proof))) return false;
        const auto results = h.retained_results();
        if (h.client_count(Ax25UType::UA) != 1 || results.size() != 1) return false;
        const auto& result = *results[0];
        if (result.outcome != v2::TransferOutcome::Succeeded ||
            result.reason != v2::TransferResultReason::RemoteEndpointAcceptedAndClosed ||
            !close_same_transfer(result.transfer, transfer) ||
            !close_same_r3(result.domains.r3, r3) ||
            !close_same_boundary(result.frozen_final_record, boundary) ||
            result.original_records.size() != 1 ||
            result.original_records[0].record_id.value != 1 ||
            result.original_records[0].size != 5 ||
            !close_full_record(result.milestones.accepted) ||
            !close_full_record(result.milestones.air_acknowledged) ||
            !close_full_record(result.milestones.endpoint_acknowledged) ||
            !result.preserved_unresolved_originals.empty() ||
            !result.preserved_unresolved_empty_records.empty() ||
            !result.preserved_incomplete_serialized_record.empty() || !result.close_proof ||
            !close_message_matches(result.close_proof->remote_attestation(), *request,
                                    v2::ProtectionDomain::RemoteCloseAttestation) ||
            !close_message_matches(result.close_proof->confirmation(), *request,
                                    v2::ProtectionDomain::CloseConfirmation)) return false;
        // Terminal tick reentry cannot publish another UA or retained result.
        h.tick(2);
        stage = "complete";
        return h.client_count(Ax25UType::UA) == 1 && h.retained_results().size() == 1;
    }();

    // Always execute the negative case, even when an earlier RC2 stub failed.
    const bool ordinary_ua_rejected = strict_ordinary_ua_rejected();
    std::printf("ACCEPTANCE_OBS name=acc_strict_end_to_end_custody case=v2_close ready=%d peer_ready=%d air_frames=%zu awaiting=%d stage=%s ua_before_r2=%zu ua_before_rr=%zu ua_after_r2=%zu request_registered=%d r3_accepted=%d ua_after_attestation=%d ua_after_receipt=%d confirmation_received=%d proof_validated=%d client_ua=%zu retained=%zu exchange_ok=%d\n",
                ready, peer_ready, air_frames, awaiting, stage, ua_before_r2, ua_before_rr,
                ua_after_r2, request_registered, r3_accepted, ua_after_attestation,
                ua_after_receipt, confirmation_received, proof_validated,
                h.client_count(Ax25UType::UA), h.retained_results().size(), exchange_ok);
    acceptance_check("acc_strict_end_to_end_custody",
                     ready && peer_ready && awaiting && no_r2_result &&
                     ua_before_r2 == 0 && ua_before_rr == 0 && ua_after_r2 == 0 &&
                     request_registered && r3_accepted && ua_after_attestation == 0 &&
                     ua_after_receipt == 0 && confirmation_received && proof_validated &&
                     exchange_ok && ordinary_ua_rejected);
}

void acc_bounded_timeout_custody() {
    AcceptanceArqHarness h;
    bool ready = h.start(decode_peer_caps(true));
    h.set_short_ack_timeout();
    h.queue_client_iframe(0, {'t', 'i', 'm', 'e', 'o', 'u', 't'});
    bool custody_was_accepted = h.client_saw_rr();
    h.clear_observations();

    // No peer ACK and no pump activity: virtual 50 ms ticks must reach a
    // terminal state and explicitly tell the local client that custody failed.
    h.tick(96);
    const bool disconnected = h.reached(Ax25SessionState::DISCONNECTED);
    const bool saw_disc = h.client_saw(Ax25UType::DISC);
    const bool saw_ua = h.client_saw(Ax25UType::UA);
    std::printf("ACCEPTANCE_OBS name=acc_bounded_timeout_custody ready=%d custody_accepted=%d disconnected=%d client_disc=%d client_ua=%d\n",
                ready, custody_was_accepted, disconnected, saw_disc, saw_ua);
    acceptance_check("acc_bounded_timeout_custody",
                     ready && custody_was_accepted &&
                     disconnected && saw_disc && !saw_ua);
}

void acc_fail_closed_on_epoch_sabm() {
    AcceptanceArqHarness h;
    bool ready = h.start(decode_peer_caps(true));
    h.queue_client_iframe(0, {'e', 'p', 'o', 'c', 'h'});
    h.tick(8);
    h.clear_observations();
    h.inject_peer_sabm();

    const bool saw_disc = h.client_saw(Ax25UType::DISC);
    const bool saw_ua = h.client_saw(Ax25UType::UA);
    std::printf("ACCEPTANCE_OBS name=acc_fail_closed_on_epoch_sabm ready=%d client_disc=%d client_ua=%d\n",
                ready, saw_disc, saw_ua);
    acceptance_check("acc_fail_closed_on_epoch_sabm",
                     ready && saw_disc && !saw_ua);
}

void acc_reset_reports_unsuccessful_custody() {
    ArqSession commander;
    ArqSession responder;
    commander.set_callsign("N0CMD");
    responder.set_callsign("N0RSP");

    std::vector<std::vector<uint8_t>> cmd_to_rsp;
    std::vector<std::vector<uint8_t>> rsp_to_cmd;
    int completions = 0;
    bool completion_success = true;

    ArqCallbacks cmd_cb;
    cmd_cb.send_frame = [&](const uint8_t* data, size_t len) {
        cmd_to_rsp.emplace_back(data, data + len);
    };
    cmd_cb.on_transfer_complete = [&](bool success) {
        ++completions;
        completion_success = success;
    };
    commander.set_callbacks(cmd_cb);

    ArqCallbacks rsp_cb;
    rsp_cb.send_frame = [&](const uint8_t* data, size_t len) {
        rsp_to_cmd.emplace_back(data, data + len);
    };
    responder.set_callbacks(rsp_cb);

    responder.listen();
    commander.connect("N0RSP");
    for (int guard = 0; guard < 20 && commander.state() != ArqState::CONNECTED;
         ++guard) {
        auto forward = std::move(cmd_to_rsp);
        cmd_to_rsp.clear();
        for (const auto& frame : forward)
            responder.on_frame_received(frame.data(), frame.size());
        auto reverse = std::move(rsp_to_cmd);
        rsp_to_cmd.clear();
        for (const auto& frame : reverse)
            commander.on_frame_received(frame.data(), frame.size());
    }

    const uint8_t payload[] = {'r', 'e', 's', 'e', 't'};
    commander.send_data(payload, sizeof(payload));
    bool connected_with_unacked_data = commander.state() == ArqState::CONNECTED &&
                                        !cmd_to_rsp.empty();
    commander.reset();

    std::printf("ACCEPTANCE_OBS name=acc_reset_reports_unsuccessful_custody connected_with_unacked_data=%d completions=%d completion_success=%d\n",
                connected_with_unacked_data, completions, completion_success);
    acceptance_check("acc_reset_reports_unsuccessful_custody",
                     connected_with_unacked_data && completions == 1 &&
                     !completion_success);

    // TODO_ACCEPTANCE(discontinuity): the audio capture/playback layer exposes no
    // discontinuity notification that a test can inject into Modem.  This public
    // reset callback is the closest observable fail-closed contract until that
    // event is plumbed into the session/custody API.
}

void acc_wire_compat_v2_negotiation() {
    const uint16_t legacy_caps = decode_peer_caps(false);
    const uint16_t v2_caps = decode_peer_caps(true);

    AcceptanceArqHarness legacy;
    bool legacy_ready = legacy.start(legacy_caps);
    auto legacy_original = legacy.queue_client_iframe(0, {'l', 'e', 'g', 'a', 'c', 'y'});
    bool legacy_used_custody = legacy.client_saw_rr();
    legacy.tick(8);
    bool legacy_fell_back_verbatim = legacy.air_contains_exactly(legacy_original);

    AcceptanceArqHarness v2;
    bool v2_ready = v2.start(v2_caps);
    v2.queue_client_iframe(0, {'v', '2'});
    bool v2_used_custody = v2.client_saw_rr();

    std::printf("ACCEPTANCE_OBS name=acc_wire_compat_v2_negotiation legacy_ready=%d v2_ready=%d legacy_caps=0x%04X v2_caps=0x%04X legacy_used_custody=%d legacy_verbatim=%d v2_used_custody=%d\n",
                legacy_ready, v2_ready, legacy_caps, v2_caps,
                legacy_used_custody, legacy_fell_back_verbatim,
                v2_used_custody);
    acceptance_check("acc_wire_compat_v2_negotiation",
                     legacy_ready && v2_ready && legacy_caps == 0 &&
                     (v2_caps & CAP_OFDM) != 0 &&
                     !legacy_used_custody && legacy_fell_back_verbatim &&
                     v2_used_custody);

    // TODO_ACCEPTANCE(v2-capability): CAP_OFDM is the closest current wire-level
    // v2 advertisement.  There is no dedicated capability bit for authoritative
    // tone ACK, wide window, and custody, nor a public negotiated-v2 accessor.
}

// Author: xmutantson. RC4 fixtures use ordinary valid codec records. Whole-buffer
// controls establish validity; only production wrapper observations decide gates.
std::vector<uint8_t> rc4_pattern(size_t size) {
    std::vector<uint8_t> bytes(size);
    uint32_t state = 0x4c534255;
    for (auto& byte : bytes) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        byte = static_cast<uint8_t>(state);
    }
    return bytes;
}

std::vector<uint8_t> rc4_encode(Compressor& codec, const std::vector<uint8_t>& plain) {
    std::vector<uint8_t> wire(plain.size() + COMPRESS_HEADER_SIZE + 256);
    const int count = codec.compress_block(plain.data(), static_cast<int>(plain.size()),
                                           wire.data(), static_cast<int>(wire.size()));
    if (count <= 0 || static_cast<size_t>(count) > wire.size()) return {};
    wire.resize(count);
    return wire;
}

bool rc4_decode_control(Compressor& codec, const std::vector<uint8_t>& wire,
                        const std::vector<uint8_t>& plain) {
    std::vector<uint8_t> out(plain.size());
    const int count = codec.decompress_block(wire.data(), static_cast<int>(wire.size()),
                                             out.data(), static_cast<int>(out.size()));
    return count == static_cast<int>(plain.size()) && out == plain;
}

bool rc4_b2f_proposal(B2fHandler& handler, size_t original, size_t encoded) {
    char out[512];
    const char sid[] = "[RC4-1.0-B2F]\r";
    char proposal[128];
    const int proposal_len = std::snprintf(proposal, sizeof(proposal),
                                          "FC EM RC4MSG %zu %zu 0\r", original, encoded);
    uint8_t checksum = 0;
    for (int i = 0; i < proposal_len; ++i)
        checksum -= static_cast<uint8_t>(proposal[i]);
    const int len = proposal_len + std::snprintf(proposal + proposal_len,
        sizeof(proposal) - proposal_len, "F> %02X\r", static_cast<unsigned>(checksum));
    const bool sid_ok = handler.filter_tx(sid, sizeof(sid) - 1, out, sizeof(out)) > 0;
    const bool proposal_ok = handler.filter_tx(proposal, len, out, sizeof(out)) > 0;
    const bool accept_ok = handler.filter_rx("FS +\r", 5, out, sizeof(out)) > 0;
    return sid_ok && proposal_ok && accept_ok && handler.is_tx_payload_active();
}

bool rc4_b2f_drain_case(const std::vector<uint8_t>& plain, const char* fixture) {
    std::vector<uint8_t> lzhuf(30000), control(30000);
    size_t encoded = 0, decoded = 0;
    const bool encoded_ok = lzhuf_encode_buffer(plain.data(), plain.size(),
        lzhuf.data(), lzhuf.size(), &encoded) == 0 && encoded > 0;
    lzhuf.resize(encoded);
    const bool fixture_ok = encoded_ok && lzhuf_decode_buffer(lzhuf.data(), lzhuf.size(),
        control.data(), control.size(), &decoded) == 0 && decoded == plain.size() &&
        std::equal(plain.begin(), plain.end(), control.begin());

    B2fHandler small, large;
    small.init();
    large.init();
    const bool prepared = rc4_b2f_proposal(small, plain.size(), encoded) &&
                          rc4_b2f_proposal(large, plain.size(), encoded);
    const int large_count = large.filter_tx(reinterpret_cast<const char*>(lzhuf.data()),
        static_cast<int>(encoded), reinterpret_cast<char*>(control.data()), control.size());
    const bool large_exact = large_count == static_cast<int>(plain.size()) &&
                            std::equal(plain.begin(), plain.end(), control.begin());
    // Match queue_tx_frame's production-style capacity, and never resubmit input.
    std::vector<char> drain(encoded * 2 + 4096);
    std::vector<uint8_t> delivered;
    bool output_valid = true;
    for (int attempt = 0; attempt != 10; ++attempt) {
        const char* input = attempt == 0 ? reinterpret_cast<const char*>(lzhuf.data()) : "";
        const int n = small.filter_tx(input, attempt == 0 ? static_cast<int>(encoded) : 0,
                                     drain.data(), static_cast<int>(drain.size()));
        if (n < 0 || static_cast<size_t>(n) > drain.size()) output_valid = false;
        else delivered.insert(delivered.end(), drain.begin(), drain.begin() + n);
    }

    AcceptanceArqHarness h;
    const bool ready = h.start_native(CAP_B2F_UNROLL);
    const bool wrapper_prepared = rc4_b2f_proposal(h.native_b2f(), plain.size(), encoded);
    std::vector<uint8_t> adopted = h.queue_native_record(lzhuf);
    for (int attempt = 0; attempt != 9; ++attempt) {
        auto more = h.queue_native_record({});
        adopted.insert(adopted.end(), more.begin(), more.end());
    }
    std::printf("ACCEPTANCE_OBS name=acc_rc4_b2f_small_drain fixture=%s fixture_valid=%d prepared=%d large_exact=%d original=%zu encoded=%zu drain_capacity=%zu drained=%zu wrapper_ready=%d wrapper_prepared=%d wrapper_adopted=%zu exact=%d\n",
        fixture, fixture_ok, prepared, large_exact, plain.size(), encoded, drain.size(), delivered.size(),
        ready, wrapper_prepared, adopted.size(), delivered == plain && adopted == plain);
    return fixture_ok && prepared && large_exact && output_valid && ready &&
           wrapper_prepared && delivered == plain && adopted == plain;
}

void acc_rc4_b2f_small_drain() {
    const bool e08 = rc4_b2f_drain_case(std::vector<uint8_t>(20000, 'A'), "e08");
    // Also cover a complete encapsulated message with the address header, exact
    // body length, separator and trailing CRLF required by winlink.org/B2F.
    const std::string header =
        "Mid: RC4MSG\r\nDate: 2026/09/04 00:00\r\nType: Private\r\n"
        "From: N0ACC\r\nTo: N0REM\r\nSubject: RC4 drain\r\nMbo: N0ACC\r\n"
        "Body: 20000\r\n\r\n";
    std::vector<uint8_t> message(header.begin(), header.end());
    message.insert(message.end(), 20000, 'A');
    message.push_back('\r'); message.push_back('\n');
    const bool encapsulated = rc4_b2f_drain_case(message, "encapsulated");
    acceptance_check("acc_rc4_b2f_small_drain", e08 && encapsulated);
}

void acc_rc4_declared_decode_capacity() {
    const std::vector<uint8_t> plain(20000, 'A');
    Compressor tx, control;
    tx.init();
    control.init();
    const auto wire = rc4_encode(tx, plain);
    const bool fixture_ok = wire.size() >= 5 && wire.size() <= 24 &&
        (wire[3] | (wire[4] << 8)) == 20000 && rc4_decode_control(control, wire, plain);
    AcceptanceArqHarness h;
    const bool ready = h.start_native(CAP_COMPRESSION);
    h.native_data(wire, 0, true);
    const auto delivered = h.delivered_bytes();
    std::printf("ACCEPTANCE_OBS name=acc_rc4_declared_decode_capacity ready=%d fixture_valid=%d encoded=%zu declared=20000 old_capacity=%zu delivered=%zu exact=%d raw_fallback=%d\n",
        ready, fixture_ok, wire.size(), wire.size() * 4 + 4096, delivered.size(),
        delivered == plain, delivered == wire);
    acceptance_check("acc_rc4_declared_decode_capacity", ready && fixture_ok &&
                     delivered == plain && h.deliveries() == 1);
}

void acc_rc4_primed_commit_order() {
    AcceptanceArqHarness tx, rx;
    const bool tx_ready = tx.start_native(CAP_COMPRESSION | CAP_STREAMING);
    const bool rx_ready = rx.start_native(CAP_COMPRESSION | CAP_STREAMING);
    Compressor expected_tx, expected_rx;
    expected_tx.init(); expected_tx.streaming_enable();
    expected_rx.init(); expected_rx.streaming_enable();
    bool valid = expected_tx.dict_primed() && expected_rx.dict_primed() &&
                 tx.native_tx_codec().dict_primed() && rx.native_rx_codec().dict_primed();
    bool tx_once = true, rx_once = true, exact = true;
    bool saw_raw = false, saw_zstd = false;
    const auto seed = rc4_pattern(256);
    for (int block = 1; block <= 4; ++block) {
        std::vector<uint8_t> plain;
        for (int repeat = 0; repeat < block; ++repeat)
            plain.insert(plain.end(), seed.begin(), seed.end());
        const auto expected_wire = rc4_encode(expected_tx, plain);
        expected_tx.streaming_commit(plain.data(), plain.size());
        const auto wire = tx.queue_native_record(plain); // production TX commit
        const bool control_ok = rc4_decode_control(expected_rx, expected_wire, plain);
        if (control_ok) expected_rx.streaming_commit(plain.data(), plain.size());
        valid = valid && control_ok && !wire.empty() && wire == expected_wire;
        if (!wire.empty()) {
            saw_raw = saw_raw || (wire[0] & COMPRESS_ALGO_MASK) == COMPRESS_ALGO_RAW;
            saw_zstd = saw_zstd || (wire[0] & COMPRESS_ALGO_MASK) == COMPRESS_ALGO_ZSTD;
        }
        rx.clear_observations();
        rx.complete_native_record(wire); // production RX; never manually commit
        const bool block_tx_once = AcceptanceArqHarness::same_model(tx.native_tx_codec(), expected_tx);
        const bool block_rx_once = AcceptanceArqHarness::same_model(rx.native_rx_codec(), expected_rx);
        const bool block_exact = rx.delivered_bytes() == plain && rx.deliveries() == 1;
        tx_once = tx_once && block_tx_once;
        rx_once = rx_once && block_rx_once;
        exact = exact && block_exact;
        std::printf("ACCEPTANCE_OBS name=acc_rc4_primed_commit_order block=%d fixture_valid=%d encoded=%zu algo=%d expected_commits=%d tx_commits=%d rx_commits=%d tx_model_once=%d rx_model_once=%d exact=%d\n",
            block, control_ok, wire.size(), wire.empty() ? -1 : wire[0] & COMPRESS_ALGO_MASK,
            AcceptanceArqHarness::model_count(expected_rx),
            AcceptanceArqHarness::model_count(tx.native_tx_codec()),
            AcceptanceArqHarness::model_count(rx.native_rx_codec()),
            block_tx_once, block_rx_once, block_exact);
    }
    acceptance_check("acc_rc4_primed_commit_order", tx_ready && rx_ready && valid &&
                     saw_raw && saw_zstd && tx_once && rx_once && exact);
}

void acc_rc4_fresh_b2f_session() {
    AcceptanceArqHarness h;
    const bool ready = h.start_native(CAP_B2F_UNROLL);
    auto& b2f = h.native_b2f();
    const bool classified = rc4_b2f_proposal(b2f, 20000, 465) && b2f.is_b2f_session();
    b2f.deinit();
    const bool deinitialized = !b2f.is_initialized();
    b2f.init();
    const bool fresh = b2f.is_initialized() && !b2f.is_b2f_session() &&
        !b2f.is_payload_transfer() && !b2f.has_payload_data_in_flight();
    const std::vector<uint8_t> ordinary{'n', 'e', 'w', ' ', 's', 'e', 's', 's', 'i', 'o', 'n', '\r'};
    const auto adopted = h.queue_native_record(ordinary);
    std::printf("ACCEPTANCE_OBS name=acc_rc4_fresh_b2f_session ready=%d classified_before=%d deinitialized=%d fresh=%d classified_after=%d payload_after=%d passthrough_exact=%d\n",
        ready, classified, deinitialized, fresh, b2f.is_b2f_session(),
        b2f.is_payload_transfer(), adopted == ordinary);
    acceptance_check("acc_rc4_fresh_b2f_session", ready && classified && deinitialized &&
                     fresh && adopted == ordinary);
}

void acc_rc4_codec_error_custody() {
    AcceptanceArqHarness tx, rx;
    const bool tx_ready = tx.start_native(CAP_COMPRESSION | CAP_STREAMING);
    const bool rx_ready = rx.start_native(CAP_COMPRESSION | CAP_STREAMING);
    const auto prefix = rc4_pattern(48);
    const auto suffix = rc4_pattern(96);
    const auto first = tx.queue_native_record(prefix);
    const auto second = tx.queue_native_record(suffix);
    // Verify the normal input and only then damage its codec CRC, as an ordinary
    // integrity error after PHY acceptance. The original accepted suffix stays TX-owned.
    Compressor control;
    control.init(); control.streaming_enable();
    const bool first_valid = rc4_decode_control(control, first, prefix);
    if (first_valid) control.streaming_commit(prefix.data(), prefix.size());
    const bool second_valid = rc4_decode_control(control, second, suffix);
    auto damaged = second;
    if (damaged.size() > 6) damaged[5] ^= 1;
    Compressor bad_control;
    bad_control.init(); bad_control.streaming_enable();
    const bool control_prefix = rc4_decode_control(bad_control, first, prefix);
    if (control_prefix) bad_control.streaming_commit(prefix.data(), prefix.size());
    std::vector<uint8_t> out(suffix.size());
    const bool normal_error = bad_control.decompress_block(damaged.data(), damaged.size(),
                                                          out.data(), out.size()) < 0;
    // Use actual ARQ DATA delivery, including the enclosing end-of-batch handler.
    // Do not let an RX callback abort be followed by successful completion/ACK.
    rx.native_data(first, 0, false);
    const bool accepted_prefix = rx.delivered_bytes() == prefix;
    rx.native_data(damaged, 1, true);
    const auto delivered = rx.delivered_bytes();
    const bool no_fallback = delivered == prefix && rx.deliveries() == 1;
    bool success = false;
    for (const auto& result : rx.native_results()) success = success || result->success;
    const bool receiver_disconnected = rx.native_disconnected();
    // TX adopts both records through queue_tx_frame, then processes the real
    // reverse transport outcomes. No reset/disconnect is injected by the fixture.
    tx.native_frame({ArqType::SWITCH_ROLE, 0, 0, {}});
    for (const auto& frame : rx.take_native_air()) tx.native_frame(frame);
    for (const auto& result : tx.native_results()) success = success || result->success;
    bool preserved = false, failed = false;
    for (const auto& result : tx.native_results()) {
        failed = failed || !result->success;
        if (!result->success)
            for (const auto& record : result->preserved_records)
                preserved = preserved || record == suffix;
    }
    const size_t ua = tx.client_count(Ax25UType::UA) + rx.client_count(Ax25UType::UA);
    std::printf("ACCEPTANCE_OBS name=acc_rc4_codec_error_custody tx_ready=%d rx_ready=%d fixture_valid=%d normal_codec_error=%d accepted_prefix=%d disconnected=%d no_raw_fallback=%d delivered=%zu success_result=%d success_ua=%zu retained_failed=%d original_suffix_recoverable=%d\n",
        tx_ready, rx_ready, first_valid && second_valid && control_prefix, normal_error,
        accepted_prefix, receiver_disconnected, no_fallback, delivered.size(), success, ua, failed, preserved);
    acceptance_check("acc_rc4_codec_error_custody", tx_ready && rx_ready && first_valid &&
        second_valid && control_prefix && normal_error && accepted_prefix && receiver_disconnected &&
        no_fallback && !success && ua == 0 && failed && preserved);
}

void acc_rc4_duplicate_record_once() {
    AcceptanceArqHarness h;
    const bool ready = h.start_native(CAP_COMPRESSION | CAP_STREAMING);
    const auto plain = rc4_pattern(96);
    Compressor tx, expected;
    tx.init(); tx.streaming_enable();
    expected.init(); expected.streaming_enable();
    const auto wire = rc4_encode(tx, plain);
    tx.streaming_commit(plain.data(), plain.size());
    const bool valid = rc4_decode_control(expected, wire, plain);
    if (valid) expected.streaming_commit(plain.data(), plain.size());
    h.native_data(wire, 0, true);
    const size_t first_deliveries = h.deliveries();
    const bool first_exact = h.delivered_bytes() == plain && first_deliveries == 1;
    const bool first_commit = AcceptanceArqHarness::same_model(h.native_rx_codec(), expected);
    const int count_before = AcceptanceArqHarness::model_count(h.native_rx_codec());
    const auto prefix_before = AcceptanceArqHarness::model_prefix(h.native_rx_codec());
    const auto results_before = h.native_results().size();
    h.native_data(wire, 0, true); // same identity, complete wire record replay
    const bool unchanged = count_before == AcceptanceArqHarness::model_count(h.native_rx_codec()) &&
        prefix_before == AcceptanceArqHarness::model_prefix(h.native_rx_codec());
    std::printf("ACCEPTANCE_OBS name=acc_rc4_duplicate_record_once ready=%d fixture_valid=%d first_exact=%d first_commit_once=%d commits_before=%d commits_after=%d deliveries_before=%zu deliveries_after=%zu duplicate_model_unchanged=%d\n",
        ready, valid, first_exact, first_commit, count_before,
        AcceptanceArqHarness::model_count(h.native_rx_codec()), first_deliveries,
        h.deliveries(), unchanged);
    acceptance_check("acc_rc4_duplicate_record_once", ready && valid && first_exact && first_commit &&
        unchanged && h.deliveries() == first_deliveries && h.delivered_bytes() == plain &&
        h.native_results().size() == results_before);
}

void acc_rc4_fragmented_record() {
    const auto plain = rc4_pattern(320);
    Compressor tx, control;
    tx.init(); control.init();
    const auto wire = rc4_encode(tx, plain);
    const bool valid = wire.size() > 300 && rc4_decode_control(control, wire, plain);
    bool all_exact = valid;
    // 150 is a production native payload size. In the first case the initial
    // fragment ends inside the codec header; in the second the final cut bisects
    // the authenticated tag. No fragment is ever independently re-encoded.
    for (int encrypted = 0; encrypted != 2; ++encrypted) {
        AcceptanceArqHarness h;
        const bool ready = h.start_native(CAP_COMPRESSION);
        std::vector<uint8_t> record = wire;
        CipherSuite sender;
        bool cipher_ok = true;
        if (encrypted) {
            cipher_ok = h.pair_native_cipher(sender);
            record.resize(wire.size() + AUTH_TAG_SIZE);
            const int n = sender.encrypt(wire.data(), wire.size(), record.data(), record.size(),
                                         0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
            cipher_ok = cipher_ok && n == static_cast<int>(record.size());
            std::vector<uint8_t> decrypted(wire.size());
            const int d = sender.decrypt(record.data(), record.size(), decrypted.data(), decrypted.size(),
                                         0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
            cipher_ok = cipher_ok && d == static_cast<int>(wire.size()) && decrypted == wire;
        }
        size_t offset = 0, fragments = 0;
        bool early_delivery = false, crossed_native = false;
        while (offset < record.size()) {
            size_t chunk = std::min<size_t>(150, record.size() - offset);
            if (!encrypted && offset == 0) chunk = 2; // split 5-byte codec header
            if (encrypted && record.size() - offset <= 150 && record.size() - offset > 4)
                chunk = record.size() - offset - 4; // split tag, retaining last 4 bytes
            crossed_native = crossed_native || chunk == 150;
            std::vector<uint8_t> fragment(record.begin() + offset, record.begin() + offset + chunk);
            offset += chunk;
            h.native_data(fragment, static_cast<uint8_t>(fragments++), offset == record.size());
            if (offset != record.size()) early_delivery = early_delivery || h.deliveries() != 0;
        }
        const bool exact = h.delivered_bytes() == plain && h.deliveries() == 1;
        std::printf("ACCEPTANCE_OBS name=acc_rc4_fragmented_record encrypted=%d ready=%d fixture_valid=%d cipher_valid=%d encoded=%zu fragments=%zu native_size=150 crossed_native=%d split=%s early_delivery=%d delivered=%zu exact=%d\n",
            encrypted, ready, valid, cipher_ok, record.size(), fragments, crossed_native,
            encrypted ? "tag" : "header", early_delivery, h.delivered_bytes().size(), exact);
        all_exact = all_exact && ready && cipher_ok && crossed_native && !early_delivery && exact;
    }
    acceptance_check("acc_rc4_fragmented_record", all_exact);
}

// RC8 sample fixtures deliberately preserve training and pilots. Complementary
// fades suppress half the DATA symbols in each reception. Each one must fail the
// real decoder; their independently demodulated LLR sum must validate all CRCs.
template<class R>
auto rc8_extent(const R& r, int) -> decltype(r.consumed_from_input_start) {
    return r.consumed_from_input_start;
}
template<class R>
auto rc8_extent(const R& r, long) -> decltype(r.samples_consumed) {
    return r.samples_consumed;
}

struct Rc8Capture {
    OfdmConfig cfg = ofdm_config_from_probe(narrow_passband(), 1024, 64, 4, 24);
    ToneMap map;
    std::vector<uint8_t> payload;
    std::vector<float> audio;
    OfdmSyncResult principal;
    Rc8Capture(int fade, float cfo, std::vector<uint8_t> bytes = {}, int lead = 4096,
               int level = 0, int codewords = 1, bool blind_context = false,
               int fade_period = 10, float erased_gain = -1.0f,
               float training_noise = 0.03f) {
        cfg.skip_papr_clip = true;
        map = get_uniform_tone_map(level + 1, cfg);
        map.n_codewords = codewords;
        if (bytes.empty()) {
            bytes.resize(64);
            for (size_t i = 0; i < bytes.size(); ++i)
                bytes[i] = static_cast<uint8_t>(i * 79 + 7);
        }
        payload = std::move(bytes);
        OfdmModulator mod(cfg);
        auto frame = mod.build_ofdm_frame(payload.data(), payload.size(), map,
                                          map.fec_rate, codewords);
        if (frame.empty()) return;
        const int sl = cfg.symbol_samples();
        if (fade >= 0) {
            int data_symbol = 0;
            for (int pos = 4 * sl; pos + sl < int(frame.size()); pos += sl) {
                if (data_symbol > 0 && data_symbol % cfg.pilot_row_spacing == 0) {
                    pos += sl;  // Keep the dense pilot row intact.
                    if (pos + sl >= int(frame.size())) break;
                }
                const bool erase = fade == 0 ? data_symbol % fade_period < fade_period / 2
                                             : data_symbol % fade_period >= fade_period / 2;
                if (erase) {
                    std::vector<std::complex<float>> freq(
                        frame.begin() + pos + cfg.cp_samples, frame.begin() + pos + sl);
                    fft_complex(freq.data(), cfg.nfft);
                    for (int bin : cfg.data_carrier_bins) {
                        const float fade_gain = erased_gain >= 0 ? erased_gain : (cfo == 0 ? 0.0f : 0.001f);
                        freq[bin] *= fade_gain;
                        freq[cfg.nfft - bin] *= fade_gain;
                    }
                    ifft_complex(freq.data(), cfg.nfft);
                    std::copy(freq.begin(), freq.end(), frame.begin() + pos + cfg.cp_samples);
                    std::copy(freq.end() - cfg.cp_samples, freq.end(), frame.begin() + pos);
                }
                ++data_symbol;
            }
        }
        size_t context = 4096;
        if (blind_context) {
            const auto bound = maximum_legal_ofdm_frame_samples(cfg, {map});
            if (!bound || *bound <= size_t(lead) + frame.size()) return;
            context = size_t(*bound) - size_t(lead) - frame.size();
        }
        audio.resize(lead + frame.size() + context);
        for (size_t i = 0; i < frame.size(); ++i) audio[lead + i] = frame[i].real();
        if (blind_context && level > 0) {
            // Later hypotheses extend into a different pilot gain. The true
            // frame's training, data and codeword-bound tail remain intact.
            const auto pilot = mod.generate_pilot_symbol();
            for (size_t pos = lead + frame.size(); pos + pilot.size() < audio.size();
                 pos += pilot.size())
                for (size_t i = 0; i < pilot.size(); ++i)
                    audio[pos + i] = 0.35f * pilot[i].real();
        }
        if (cfo != 0) {
            auto analytic = ofdm_analytic_bandlimit(audio.data(), int(audio.size()),
                                                   cfg, RxBandlimitEdge::OFF);
            for (size_t i = 0; i < audio.size(); ++i) {
                const float phase = 6.283185307179586f * cfo * float(i) / cfg.sample_rate;
                audio[i] = (analytic[i] * std::polar(1.0f, phase)).real();
            }
        }
        double power = 0;
        for (const auto& x : frame) power += std::norm(x);
        const float floor = 0.03f * std::sqrt(power / frame.size());
        uint32_t rng = 0x1234567u;
        for (size_t i = 0; i < audio.size(); ++i) {
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            const float noise = i >= size_t(lead + sl) && i < size_t(lead + 3 * sl)
                ? floor * (training_noise / 0.03f) : floor;
            audio[i] += noise * (float((rng >> 8) & 65535) / 32767.5f - 1);
        }
        auto iq = analytic();
        principal = ofdm_detect_frame(iq.data(), int(iq.size()), cfg);
    }
    std::vector<std::complex<float>> analytic(size_t count = 0) const {
        return ofdm_analytic_bandlimit(audio.data(),
                                       int(count ? std::min(count, audio.size()) : audio.size()),
                                       cfg, RxBandlimitEdge::TAPERED);
    }
    OfdmDemodResult reference(int m, size_t count = 0,
                             OfdmChannelEst* channel = nullptr,
                             OfdmSyncResult* selected = nullptr) const {
        auto iq = analytic(count);
        auto bank = ofdm_cfo_hypotheses(iq.data(), int(iq.size()), cfg, principal);
        for (const auto& trial : bank) {
            if (trial.cfo_ambiguity_index != m) continue;
            const auto sync = ofdm_refine_cfo_hypothesis(iq.data(), int(iq.size()), cfg, trial);
            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(iq.data(), int(iq.size()), map, &sync);
            if (channel) *channel = demod.last_channel_estimate();
            if (selected) *selected = sync;
            return result;
        }
        return {};
    }
    // An interfering tail aligned to m=-2 makes BOTH tails plausible. Its
    // known reference is filtered by the actual wrong-wrap channel estimate.
    // This is waveform construction only; no demod result or RX store is set.
    bool add_alias_tail() {
        OfdmChannelEst channel;
        OfdmSyncResult alias;
        auto result = reference(-2, 0, &channel, &alias);
        const int sl = cfg.symbol_samples();
        const int start = int(rc8_extent(result, 0)) - sl;
        if (channel.H.size() != size_t(cfg.n_used_carriers) || start < 0 ||
            start + sl > int(audio.size())) return false;
        const auto zc = generate_zc_sequence(ofdm_tail_zc_root(cfg.n_used_carriers, 1),
                                            cfg.n_used_carriers);
        std::vector<std::complex<float>> freq(cfg.nfft);
        for (int i = 0; i < cfg.n_used_carriers; ++i)
            freq[cfg.used_carrier_bins[i]] = channel.H[i] * zc[i];
        ifft_complex(freq.data(), cfg.nfft);
        for (int j = 0; j < sl; ++j) {
            int k = (j - cfg.cp_samples + cfg.nfft) % cfg.nfft;
            float phase = 6.283185307179586f * alias.cfo_hz *
                          (start + j - alias.frame_start) / cfg.sample_rate;
            audio[start + j] += 0.75f * (freq[k] * std::polar(1.0f, phase)).real();
        }
        return true;
    }
};

double rc8_correlation(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.empty() || a.size() != b.size()) return -2;
    double aa = 0, bb = 0, ab = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        aa += double(a[i]) * a[i]; bb += double(b[i]) * b[i]; ab += double(a[i]) * b[i];
    }
    return aa > 0 && bb > 0 ? ab / std::sqrt(aa * bb) : -2;
}

bool rc8_failed(const OfdmDemodResult& r) {
    return !r.success && r.complete_boundary_validated && !r.llrs.empty() &&
           !r.block_results.empty() &&
           std::any_of(r.block_results.begin(), r.block_results.end(),
                       [](const LdpcCodec::BlockResult& b) { return !b.converged; });
}

bool rc8_sum_exact(const Rc8Capture& a, const OfdmDemodResult& first,
                   const OfdmDemodResult& second) {
    if (!rc8_failed(first) || !rc8_failed(second) || first.llrs.size() != second.llrs.size())
        return false;
    auto sum = first.llrs;
    for (size_t i = 0; i < sum.size(); ++i) sum[i] += second.llrs[i];
    auto bits = LdpcCodec::decode_soft(sum, first.fec_rate, LdpcDecoder::MIN_SUM, 50);
    std::vector<uint8_t> decoded;
    return OfdmDemodulator::extract_payload_blocks(bits, first.fec_rate, first.n_ldpc_blocks,
        first.n_ldpc_blocks * size_t(LdpcCodec::block_size(first.fec_rate)), decoded) &&
        decoded == a.payload;
}

struct Rc8Run {
    bool finished = false, reentered = false, same_identity = true;
    bool blind_tail_seen = false, discontinuity = false;
    bool pure = true, drain_at_verdict = true, drain_advanced = false;
    int mutations = 0, captures = 0, turns = 0;
    Rc8Snapshot after_reentry;
};

Rc8Run rc8_receive(AcceptanceArqHarness& h, const Rc8Capture& c,
                   size_t first_count = 0, bool trace = false) {
    Rc8Run run;
    h.rc8_capture(c.principal);
    auto before = h.rc8_snapshot();
    const auto identity = std::make_pair(before.candidate, before.epoch);
    const uint64_t origin = before.origin;
    size_t fed = first_count ? std::min(first_count, c.audio.size()) : c.audio.size();
    bool restarting = false;
    for (int turn = 0; turn < 96; ++turn) {
        if (turn == 0) h.rc8_turn(c.audio.data(), fed);
        else if (before.required > 0 && fed < c.audio.size()) {
            h.rc8_turn(c.audio.data() + fed, c.audio.size() - fed);
            fed = c.audio.size();
        } else h.rc8_turn();
        auto after = h.rc8_snapshot();
        run.blind_tail_seen = run.blind_tail_seen ||
            h.rc8_last_log.find("tail boundary C=2:") != std::string::npos;
        run.discontinuity = run.discontinuity ||
            h.rc8_last_log.find("CAPTURE DISCONTINUITY") != std::string::npos;
        ++run.turns;
        const bool changed = !before.persistent_equal(after);
        if ((before.chase != after.chase && !after.chase.empty()) ||
            (before.harq != after.harq && !after.harq.empty())) ++run.captures;
        if (after.unresolved && changed) { run.pure = false; ++run.mutations; }
        if (after.unresolved && after.origin != origin) run.drain_at_verdict = false;
        if (after.unresolved)
            run.same_identity = run.same_identity &&
                std::make_pair(after.candidate, after.epoch) == identity;
        if (after.unresolved && after.cursor == 0 && before.cursor > 0)
            restarting = true;
        if (restarting && after.cursor > 0 && after.unresolved) {
            run.reentered = true;
            run.after_reentry = after;
            restarting = false;
        }
        if (trace)
            std::printf("ACCEPTANCE_RC8_TURN turn=%d cursor=%zu remaining=%d snr=%.3f gear_equal=%d tune_equal=%d notify=%d chase_generation=%d combines=%d drain=%llu changed=%d\n",
                turn, after.cursor, after.unresolved, after.snr, before.gear == after.gear,
                before.tune == after.tune, after.notifications, run.captures, after.combines,
                static_cast<unsigned long long>(after.origin - origin), changed);
        if (!after.unresolved) {
            run.finished = true;
            run.drain_advanced = after.origin > origin;
            // "After re-entry" means after that bank's verdict. Inspecting
            // its first wrap instead would require a mid-search store write,
            // contradicting the purity assertion in T6.
            if (run.reentered) run.after_reentry = after;
            break;
        }
        before = std::move(after);
    }
    return run;
}

std::vector<uint8_t> rc8_ui_payload(size_t size) {
    auto bytes = ax25_build_u(ax25_make_addr(AcceptanceArqHarness::LOCAL),
        ax25_make_addr(AcceptanceArqHarness::PEER), AX25_CTRL_UI, false);
    bytes.push_back(AX25_PID_NONE);
    while (bytes.size() < size) bytes.push_back(uint8_t(bytes.size() * 79 + 7));
    return bytes;
}

// T7--T12: each precondition is measured independently of the asserted seam
// behavior. A fixture that never reaches the trigger fails closed.
void acc_rc8_belief_survives_turnaround() {
    Rc8Capture clean(-1, 150), damaged(0, 150);
    const auto owner = damaged.reference(3);
    const bool fixture = clean.reference(3).success && rc8_failed(owner);
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(clean.cfg, clean.map);
    Rc8Run warmup, failed;
    bool turnaround = false;
    float before = 0, unmuted = 0;
    if (ready && fixture) {
        warmup = rc8_receive(h, clean);
        before = h.rc8_link().belief;
        turnaround = h.rc8_ack_turnaround();
        unmuted = h.rc8_link().belief;
        failed = rc8_receive(h, damaged);
    }
    const auto after = h.rc8_link();
    const double chase = rc8_correlation(h.rc8_snapshot().chase, owner.llrs);
    const double harq = rc8_correlation(h.rc8_notified_llrs, owner.llrs);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_belief_survives_turnaround fixture=%d ready=%d warmup=%d turnaround=%d before=%.3f unmuted=%.3f after=%.3f finished=%d chase_corr=%.6f harq_corr=%.6f callbacks=%d\n",
        fixture, ready, warmup.finished, turnaround, before, unmuted, after.belief,
        failed.finished, chase, harq, h.rc8_harq_notifications);
    acceptance_check("acc_rc8_belief_survives_turnaround", fixture && ready &&
        warmup.finished && turnaround && std::fabs(before - 150) < 3 &&
        std::fabs(unmuted - before) < 0.01f && std::fabs(after.belief - before) < 0.01f &&
        failed.finished && h.rc8_observable && chase >= 0.99 && harq >= 0.99 &&
        h.rc8_harq_notifications == 1);
}

void acc_rc8_belief_resets_on_teardown() {
    Rc8Capture clean(-1, 150), damaged(0, 150);
    const bool fixture = clean.reference(3).success && rc8_failed(damaged.reference(3));
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(clean.cfg, clean.map);
    Rc8Run warmup, failed;
    Rc8LinkState before;
    bool idle = false;
    if (ready && fixture) {
        warmup = rc8_receive(h, clean);
        failed = rc8_receive(h, damaged);
        before = h.rc8_link();
        idle = h.rc8_native_teardown();
    }
    const auto after = h.rc8_link();
    const auto storage = h.rc8_snapshot();
    const bool established = warmup.finished && failed.finished &&
        std::fabs(before.belief - 150) < 3 && before.chase_id != 0 &&
        before.chase_epoch != 0 && before.owner_llrs > 0;
    const bool cleared = after.belief == 0 && storage.chase.empty() &&
        storage.combines == 0 && after.chase_id == 0 && after.chase_epoch == 0 &&
        after.chase_level == -1 && after.chase_cw == 0 && !after.owner_valid &&
        after.owner_id == 0 && after.owner_epoch == 0 && after.owner_llrs == 0;
    std::printf("ACCEPTANCE_OBS name=acc_rc8_belief_resets_on_teardown path=native_idle fixture=%d ready=%d established=%d idle=%d before=%.3f after=%.3f chase_llrs=%zu chase_id=%llu chase_epoch=%llu owner_id=%llu owner_epoch=%llu owner_llrs=%zu cleared=%d\n",
        fixture, ready, established, idle, before.belief, after.belief, storage.chase.size(),
        (unsigned long long)after.chase_id, (unsigned long long)after.chase_epoch,
        (unsigned long long)after.owner_id, (unsigned long long)after.owner_epoch,
        after.owner_llrs, cleared);
    acceptance_check("acc_rc8_belief_resets_on_teardown", fixture && ready &&
        established && idle && cleared && h.rc8_observable);
}

void acc_rc8_rediscovery_not_recombine() {
    Rc8Capture c(0, 0);
    const auto reference = c.reference(0);
    const bool fixture = rc8_failed(reference) && c.principal.detected &&
        c.principal.timing_interval_begin + 1 < c.principal.frame_start;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(c.cfg, c.map);
    Rc8Run first;
    Rc8Snapshot before;
    Rc8LinkState identity;
    int combines = 0, notifications = 0, commits = 0, extra_turns = 0;
    if (ready && fixture) {
        first = rc8_receive(h, c);
        before = h.rc8_snapshot(); identity = h.rc8_link();
        combines = h.rc8_total_combines; notifications = h.rc8_notifications;
        for (int turn = 0; turn < 96; ++turn) {
            h.rc8_turn(); // Deliberately NO rc8_capture(), sync injection or new audio.
            ++extra_turns;
            if (h.rc8_last_log.find("LLRs for Chase combining") != std::string::npos ||
                h.rc8_last_log.find("frame OK:") != std::string::npos) ++commits;
        }
    }
    const auto after = h.rc8_snapshot();
    const auto id_after = h.rc8_link();
    double old_power = 0, new_power = 0;
    for (float x : before.chase) old_power += double(x) * x;
    for (float x : after.chase) new_power += double(x) * x;
    const double ratio = old_power > 0 ? new_power / old_power : -1;
    std::printf("ACCEPTANCE_OBS name=acc_rc8_rediscovery_not_recombine fixture=%d ready=%d finished=%d begin=%d start=%d end=%d drained=%llu extra_turns=%d additional_combines=%d additional_notify=%d additional_commits=%d before_id=%llu after_id=%llu store_equal=%d power_ratio=%.6f\n",
        fixture, ready, first.finished, c.principal.timing_interval_begin,
        c.principal.frame_start, int(rc8_extent(reference, 0)),
        (unsigned long long)before.origin, extra_turns, h.rc8_total_combines - combines,
        h.rc8_notifications - notifications, commits, (unsigned long long)identity.chase_id,
        (unsigned long long)id_after.chase_id, before.chase == after.chase, ratio);
    acceptance_check("acc_rc8_rediscovery_not_recombine", fixture && ready && first.finished &&
        !before.chase.empty() && extra_turns == 96 && h.rc8_observable &&
        h.rc8_total_combines == combines && h.rc8_notifications == notifications && commits == 0 &&
        before.chase == after.chase && before.origin >= uint64_t(rc8_extent(reference, 0)));
}

void acc_rc8_geometry_uses_demod_level() {
    Rc8Capture clean(-1, 150, {}, 4096, 2), first(0, 150, {}, 4096, 2),
        second(1, 150, {}, 4096, 2);
    const auto r1 = first.reference(3), r2 = second.reference(3);
    const auto iq = first.analytic();
    const auto bank = ofdm_cfo_hypotheses(iq.data(), int(iq.size()), first.cfg, first.principal);
    const bool fixture = clean.reference(3).success && rc8_failed(r1) && rc8_failed(r2) &&
        !bank.empty() && bank.back().cfo_ambiguity_index == 3;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(first.cfg, first.map);
    Rc8Run warmup, one, two;
    Rc8LinkState before, demoted, after;
    if (ready && fixture) {
        warmup = rc8_receive(h, clean);
        h.rc8_level(2, true);
        before = h.rc8_link();
        one = rc8_receive(h, first);
        demoted = h.rc8_link();
        two = rc8_receive(h, second);
        after = h.rc8_link();
    }
    const bool transition = before.level == 2 && demoted.level == 1 &&
        demoted.map_level == 2 && demoted.chase_level == 2 && after.map_level == 2;
    const bool exact = h.rc8_frames() == 2 && h.rc8_delivered(first.payload);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_geometry_uses_demod_level fixture=%d ready=%d warmup=%d first=%d second=%d level_before=%d level_demoted=%d map_level=%d stored_level=%d after_stored_level=%d transition=%d combines=%d exact=%d\n",
        fixture, ready, warmup.finished, one.finished, two.finished, before.level,
        demoted.level, demoted.map_level, demoted.chase_level, after.chase_level,
        transition, h.rc8_total_combines, exact);
    acceptance_check("acc_rc8_geometry_uses_demod_level", fixture && ready && warmup.finished &&
        one.finished && two.finished && transition && h.rc8_observable &&
        h.rc8_total_combines == 1 && after.chase_level == 2);
}

void acc_rc8_blind_shape_owner_evidence() {
    // More than one codeword's payload proves that a block-zero-only decoder
    // cannot satisfy the oracle. Both CRC-bearing blocks must be recovered.
    const auto payload = rc8_ui_payload(140);
    Rc8Capture first(0, 0, payload, 2048, 0, 2, true, 2, 0.001f),
        second(1, 0, payload, 2048, 0, 2, true, 2, 0.001f);
    const auto r1 = first.reference(0), r2 = second.reference(0);
    auto configured = first;
    configured.map.n_codewords = 1;
    const auto wrong_shape = configured.reference(0);
    const bool fixture = rc8_sum_exact(first, r1, r2) && r1.n_ldpc_blocks == 2 &&
        !wrong_shape.success && !wrong_shape.complete_boundary_validated;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(first.cfg, configured.map);
    Rc8Run one, two;
    Rc8LinkState stored;
    double corr = -2;
    size_t stored_size = 0;
    if (ready && fixture) {
        h.rc8_blind(0, 1);
        one = rc8_receive(h, first);
        stored = h.rc8_link();
        stored_size = h.rc8_snapshot().chase.size();
        corr = rc8_correlation(h.rc8_snapshot().chase, r1.llrs);
        two = rc8_receive(h, second);
    }
    const bool exact = h.rc8_frames() == 1 && h.rc8_delivered(payload);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_blind_shape_owner_evidence fixture=%d ready=%d configured_boundary=%d blind_boundary=%d blocks=%d sum_exact=%d first=%d second=%d blind_seen=%d discontinuity=%d stored_llrs=%zu expected_llrs=%zu stored_cw=%d corr=%.6f combines=%d exact=%d\n",
        fixture, ready, wrong_shape.complete_boundary_validated, r1.complete_boundary_validated,
        r1.n_ldpc_blocks, rc8_sum_exact(first, r1, r2), one.finished, two.finished,
        one.blind_tail_seen && two.blind_tail_seen, one.discontinuity || two.discontinuity, stored_size, r1.llrs.size(), stored.chase_cw, corr, h.rc8_total_combines, exact);
    acceptance_check("acc_rc8_blind_shape_owner_evidence", fixture && ready && one.finished &&
        two.finished && one.blind_tail_seen && two.blind_tail_seen &&
        !one.discontinuity && !two.discontinuity && h.rc8_observable && stored.chase_cw == 2 && stored.chase_level == 0 &&
        stored_size == r1.llrs.size() && corr >= 0.99 && h.rc8_total_combines == 1 && exact);
}

void acc_rc8_channel_snapshot_owned() {
    const auto payload = rc8_ui_payload(64);
    Rc8Capture first(0, 0, payload, 2048, 1, 1, true, 2, 0.001f, 1.0f),
        second(1, 0, payload, 2048, 1, 1, true, 2, 0.001f, 1.0f);
    OfdmChannelEst owner_channel, later_channel;
    const auto r1 = first.reference(0), r2 = second.reference(0, 0, &owner_channel);
    auto later = second;
    later.map.n_codewords = 8;
    const auto late = later.reference(0, 0, &later_channel);
    Rc8ContentHash owner_hash, later_hash;
    owner_hash.channel(owner_channel); later_hash.channel(later_channel);
    float owner_gain = 0, later_gain = 0;
    for (const auto& x : owner_channel.H) owner_gain += std::abs(x);
    for (const auto& x : later_channel.H) later_gain += std::abs(x);
    if (!owner_channel.H.empty()) owner_gain /= owner_channel.H.size();
    if (!later_channel.H.empty()) later_gain /= later_channel.H.size();
    const bool sum_exact = rc8_sum_exact(first, r1, r2);
    // A separately CRC-validated sum must also pass the existing C1 policy;
    // otherwise TUNE withholding would not demonstrate snapshot ownership.
    auto recovered_reference = r2;
    recovered_reference.payload_validated = recovered_reference.cfo_resolved = sum_exact;
    ofdm_authorize_payload_estimator(recovered_reference);
    const bool tune_admissible = recovered_reference.estimator_validity ==
        OfdmEstimatorValidity::PayloadValidatedForSelectedCfo;
    const bool fixture = sum_exact && tune_admissible && !late.success &&
        !late.llrs.empty() && owner_hash.value != later_hash.value &&
        std::fabs(owner_gain - later_gain) > 0.01f && ofdm_s2_track_in_scope(2);
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(first.cfg, first.map);
    int unresolved_turns = 0, content_changes = 0, channel_changes = 0;
    bool one = false, two = false, later_ran = false, winner_after_blind = false;
    if (ready && fixture) {
        h.rc8_blind(1, 1);
        for (const auto* capture : {&first, &second}) {
            h.rc8_capture(capture->principal);
            for (int turn = 0; turn < 128; ++turn) {
                const auto before = h.rc8_link();
                h.rc8_turn(turn == 0 ? capture->audio.data() : nullptr,
                           turn == 0 ? capture->audio.size() : 0);
                const auto after = h.rc8_link();
                const auto blind_pos = h.rc8_last_log.find("tail boundary C=8:");
                const auto winner_pos = h.rc8_last_log.find("Chase combining SUCCEEDED");
                later_ran = later_ran || blind_pos != std::string::npos;
                winner_after_blind = winner_after_blind ||
                    (blind_pos != std::string::npos && winner_pos != std::string::npos &&
                     blind_pos < winner_pos);
                if (h.rc8_snapshot().unresolved) {
                    ++unresolved_turns;
                    content_changes += before.content_hash != after.content_hash;
                    channel_changes += before.channel_hash != after.channel_hash;
                } else {
                    if (capture == &first) one = true; else two = true;
                    break;
                }
            }
        }
    }
    const auto after = h.rc8_link();
    const bool exact = h.rc8_frames() == 1 && h.rc8_delivered(first.payload);
    const bool owned = after.tune_count == 1 &&
        std::fabs(after.tune_gain - owner_gain) <= 0.00001f * std::max(1.0f, owner_gain);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_channel_snapshot_owned fixture=%d ready=%d first=%d second=%d later_ran=%d winner_after_blind=%d tune_admissible=%d unresolved_turns=%d content_changes=%d channel_changes=%d owner_H=%.6f later_H=%.6f tune_H=%.6f measurements=%d owned=%d combines=%d exact=%d\n",
        fixture, ready, one, two, later_ran, winner_after_blind, tune_admissible, unresolved_turns, content_changes,
        channel_changes, owner_gain, later_gain, after.tune_gain, after.tune_count,
        owned, h.rc8_total_combines, exact);
    acceptance_check("acc_rc8_channel_snapshot_owned", fixture && ready && one && two &&
        later_ran && winner_after_blind && unresolved_turns > 0 && content_changes == 0 && channel_changes == 0 &&
        h.rc8_observable && owned && h.rc8_total_combines == 1 && exact);
}

void acc_rc8_alias_evidence_ownership() {
    Rc8Capture c(0, 0);
    const bool tail = c.add_alias_tail();
    const auto principal = c.reference(0);
    const auto alias = c.reference(-2);
    const size_t split = size_t(rc8_extent(principal, 0)) - 100;
    const auto early_principal = c.reference(0, split);
    const auto early_alias = c.reference(-2, split);
    const bool fixture = c.principal.detected && tail && rc8_failed(principal) &&
        rc8_failed(alias) && rc8_failed(early_alias) &&
        early_principal.completion == OfdmDemodResult::Completion::NeedMoreSamples;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(c.cfg, c.map);
    Rc8Run run;
    if (ready && fixture) run = rc8_receive(h, c, split);
    const double owner_corr = rc8_correlation(h.rc8_notified_llrs, principal.llrs);
    const double alias_corr = rc8_correlation(h.rc8_notified_llrs, alias.llrs);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_alias_evidence_ownership fixture=%d ready=%d finished=%d reentered=%d owner_corr=%.6f alias_corr=%.6f callbacks=%d\n",
        fixture, ready, run.finished, run.reentered, owner_corr, alias_corr, h.rc8_harq_notifications);
    acceptance_check("acc_rc8_alias_evidence_ownership", fixture && ready && run.finished &&
        run.reentered && h.rc8_observable && owner_corr >= 0.99 &&
        alias_corr >= -1 && alias_corr <= 0.5 && h.rc8_harq_notifications == 1);
}

void acc_rc8_reentry_not_retransmission() {
    Rc8Capture c(0, 0);
    auto reference = c.reference(0);
    const size_t split = size_t(rc8_extent(reference, 0)) + 80;
    const bool fixture = c.principal.detected && rc8_failed(reference) &&
        rc8_failed(c.reference(0, split)) && c.reference(3, split).completion ==
            OfdmDemodResult::Completion::NeedMoreSamples;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(c.cfg, c.map);
    Rc8Run run;
    if (ready && fixture) run = rc8_receive(h, c, split);
    const auto& after = run.after_reentry;
    const double corr = rc8_correlation(after.chase, reference.llrs);
    // Correlation alone permits doubled LLRs. Check norm as well.
    double stored_power = 0, reference_power = 0;
    for (float x : after.chase) stored_power += double(x) * x;
    for (float x : reference.llrs) reference_power += double(x) * x;
    const double ratio = reference_power > 0 ? stored_power / reference_power : 0;
    std::printf("ACCEPTANCE_OBS name=acc_rc8_reentry_not_retransmission fixture=%d ready=%d reentered=%d same_identity=%d combines=%d total=%d store_llrs=%zu corr=%.6f power_ratio=%.6f\n",
        fixture, ready, run.reentered, run.same_identity, after.combines,
        h.rc8_total_combines, after.chase.size(), corr, ratio);
    acceptance_check("acc_rc8_reentry_not_retransmission", ready && fixture &&
        run.finished && run.reentered && run.same_identity && h.rc8_observable &&
        after.combines == 0 && h.rc8_total_combines == 0 && corr >= 0.99 &&
        after.chase.size() == reference.llrs.size() && ratio >= 0.95 && ratio <= 1.05);
}

void acc_rc8_real_retransmission_combines() {
    Rc8Capture first(0, 0), second(1, 0);
    auto r1 = first.reference(0), r2 = second.reference(0);
    const size_t split = size_t(rc8_extent(r1, 0)) + 80;
    const bool fixture = first.principal.detected && second.principal.detected &&
        rc8_sum_exact(first, r1, r2) && rc8_failed(first.reference(0, split)) &&
        first.reference(3, split).completion == OfdmDemodResult::Completion::NeedMoreSamples;
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(first.cfg, first.map);
    Rc8Run one, two;
    uint64_t epoch1 = 0, epoch2 = 0;
    if (ready && fixture) {
        one = rc8_receive(h, first, split);
        epoch1 = h.rc8_snapshot().epoch;
        two = rc8_receive(h, second);
        epoch2 = h.rc8_snapshot().epoch;
    }
    const bool exact = h.rc8_delivered(first.payload);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_real_retransmission_combines fixture=%d ready=%d reentered=%d new_epoch=%d exact=%d combines=%d\n",
        fixture, ready, one.reentered, epoch1 != epoch2, exact, h.rc8_total_combines);
    acceptance_check("acc_rc8_real_retransmission_combines", fixture && ready &&
        one.finished && one.reentered && one.same_identity && two.finished && epoch1 != epoch2 &&
        h.rc8_observable && exact && h.rc8_total_combines == 1);
}

void acc_rc8_resolved_cfo_keeps_harq() {
    Rc8Capture clean(-1, 150, std::vector<uint8_t>(32, 0x57));
    Rc8Capture first(0, 150), second(1, 150);
    auto resolved = clean.reference(3), r1 = first.reference(3), r2 = second.reference(3);
    const bool fixture = clean.principal.detected && first.principal.detected &&
        second.principal.detected && resolved.success && resolved.payload == clean.payload &&
        rc8_sum_exact(first, r1, r2);
    AcceptanceArqHarness h;
    const bool ready = h.rc8_start(first.cfg, first.map);
    Rc8Run warmup, one, two;
    bool committed = false, owned = false;
    if (fixture && ready) {
        warmup = rc8_receive(h, clean);
        committed = h.rc8_frames() == 1 && h.rc8_delivered(clean.payload);
        one = rc8_receive(h, first);
        owned = rc8_correlation(h.rc8_snapshot().chase, r1.llrs) >= 0.99 &&
                rc8_correlation(h.rc8_notified_llrs, r1.llrs) >= 0.99;
        two = rc8_receive(h, second);
    }
    const bool exact = h.rc8_frames() == 2 && h.rc8_delivered(first.payload);
    std::printf("ACCEPTANCE_OBS name=acc_rc8_resolved_cfo_keeps_harq fixture=%d ready=%d committed=%d owner_evidence=%d callbacks=%d exact=%d combines=%d\n",
        fixture, ready, committed, owned, h.rc8_harq_notifications, exact, h.rc8_total_combines);
    acceptance_check("acc_rc8_resolved_cfo_keeps_harq", fixture && ready && warmup.finished &&
        one.finished && two.finished && h.rc8_observable && committed && owned && exact &&
        h.rc8_harq_notifications == 1 && h.rc8_total_combines == 1);
}

void acc_rc8_backtoback_frame_drain() {
    bool all = true;
    for (int site = 0; site < 3; ++site) {
        std::vector<uint8_t> a_bytes;
        if (site == 1) {
            const char marker[] = "TUNE_TEST_FRAME";
            a_bytes.assign(marker, marker + sizeof(marker) - 1);
        } else if (site == 2) a_bytes = {COMPRESSED_PAYLOAD_MAGIC, 0xff, 0xff, 0xff};
        Rc8Capture a(-1, 0, a_bytes, 978), b(-1, 0);
        // Detector's calibrated timing is 18 samples before the training CP:
        // 978 + 1088 - 18 = 2048. Fail closed if that fixture moves.
        auto ra = a.reference(0), rb = b.reference(0);
        const size_t extent = size_t(rc8_extent(ra, 0));
        bool fixture = a.principal.detected && b.principal.detected &&
            a.principal.frame_start == 2048 && ra.success && rb.success &&
            ra.payload == a.payload && rb.payload == b.payload && extent <= a.audio.size();
        // Positive control: the exact suffix the correct drain would retain
        // must decode through discovery, without installing B's sync.
        AcceptanceArqHarness suffix;
        bool suffix_exact = false;
        if (fixture && suffix.rc8_start(b.cfg, b.map)) {
            suffix.rc8_turn(b.audio.data() + b.principal.frame_start,
                            b.audio.size() - b.principal.frame_start);
            for (int i = 0; i < 24 && suffix.rc8_frames() == 0; ++i) suffix.rc8_turn();
            suffix_exact = suffix.rc8_frames() == 1 && suffix.rc8_delivered(b.payload);
        }
        if (fixture) {
            // B's calibrated preamble starts exactly at A's input-relative end.
            a.audio.resize(extent);
            a.audio.insert(a.audio.end(), b.audio.begin() + b.principal.frame_start, b.audio.end());
        }
        AcceptanceArqHarness h;
        const bool ready = h.rc8_start(a.cfg, a.map);
        if (site == 2) h.rc8_decompression_mode();
        uint64_t drained = 0;
        size_t decoded_a = 0;
        bool branch = false;
        if (fixture && ready) {
            h.rc8_capture(a.principal);
            const auto origin = h.rc8_snapshot().origin;
            h.rc8_turn(a.audio.data(), a.audio.size());
            drained = h.rc8_snapshot().origin - origin;
            decoded_a = h.rc8_frames();
            branch = site == 0 ? h.rc8_delivered(a.payload) :
                h.rc8_last_log.find(site == 1 ? "Discarded OFDM test frame" :
                    "OFDM receive decompression") != std::string::npos;
            for (int i = 0; i < 24 && h.rc8_frames() < 2; ++i) h.rc8_turn();
        }
        const bool both = decoded_a == 1 && h.rc8_frames() == 2;
        std::printf("ACCEPTANCE_OBS name=acc_rc8_backtoback_frame_drain site=%d fixture=%d suffix_exact=%d ready=%d branch=%d start=%d expected_end=%zu drained=%llu decoded_a=%zu decoded_total=%zu both=%d\n",
            site, fixture, suffix_exact, ready, branch, a.principal.frame_start, extent,
            static_cast<unsigned long long>(drained), decoded_a, h.rc8_frames(), both);
        all = all && fixture && suffix_exact && ready && branch && h.rc8_observable &&
              both && drained == extent;
    }
    acceptance_check("acc_rc8_backtoback_frame_drain", all);
}

void acc_rc8_search_purity() {
    bool all = true;
    for (float cfo : {0.0f, 150.0f, -150.0f}) {
        for (bool split : {false, true}) {
            Rc8Capture c(0, cfo);
            auto iq = c.analytic();
            auto bank = ofdm_cfo_hypotheses(iq.data(), int(iq.size()), c.cfg, c.principal);
            bool fixture = c.principal.detected && bank.size() >= 6;
            bool evidence = false;
            for (const auto& trial : bank) {
                auto r = c.reference(trial.cfo_ambiguity_index);
                fixture = fixture && !r.success &&
                    r.completion != OfdmDemodResult::Completion::NeedMoreSamples;
                evidence = evidence || rc8_failed(r);
            }
            auto principal = c.reference(0);
            AcceptanceArqHarness h;
            const bool ready = h.rc8_start(c.cfg, c.map);
            // Establish this link's belief with a payload-validated reception.
            // A cold zero-Hz belief cannot own a failed nonzero-wrap reception.
            // The measured reception below still fails EVERY bank hypothesis.
            Rc8Capture warmup(-1, cfo, std::vector<uint8_t>(32, 0x57));
            bool primed = false;
            if (ready && warmup.principal.detected) {
                const auto resolved = rc8_receive(h, warmup);
                primed = resolved.finished && h.rc8_frames() == 1 &&
                         h.rc8_delivered(warmup.payload) && h.rc8_notifications == 0;
            }
            Rc8Run run;
            if (fixture && evidence && ready && primed)
                run = rc8_receive(h, c, split ? size_t(rc8_extent(principal, 0)) + 80 : 0, true);
            std::printf("ACCEPTANCE_OBS name=acc_rc8_search_purity cfo=%.0f split=%d fixture=%d ready=%d primed=%d turns=%d finished=%d reentered=%d mutations=%d captures=%d callbacks=%d drain_at_verdict=%d drain_advanced=%d\n",
                cfo, split, fixture && evidence, ready, primed, run.turns, run.finished,
                run.reentered, run.mutations, run.captures, h.rc8_notifications,
                run.drain_at_verdict, run.drain_advanced);
            all = all && fixture && evidence && ready && primed && h.rc8_observable && run.finished &&
                run.turns >= int(bank.size()) && (!split || run.reentered) && run.pure &&
                run.captures <= 1 && h.rc8_notifications == 1 && run.drain_at_verdict &&
                run.drain_advanced;
        }
    }
    acceptance_check("acc_rc8_search_purity", all);
}


} // namespace

void run_acceptance_arq() {
    std::printf("\n--- ARQ/Custody Owner-Decision Acceptance Tests ---\n");
    acc_strict_end_to_end_custody();
    acc_bounded_timeout_custody();
    acc_fail_closed_on_epoch_sabm();
    acc_reset_reports_unsuccessful_custody();
    acc_wire_compat_v2_negotiation();
    std::printf("\n--- RC4 Transactional Transform Acceptance Tests ---\n");
    acc_rc4_b2f_small_drain();
    acc_rc4_declared_decode_capacity();
    acc_rc4_primed_commit_order();
    acc_rc4_fresh_b2f_session();
    acc_rc4_codec_error_custody();
    acc_rc4_duplicate_record_once();
    acc_rc4_fragmented_record();
    acc_rc8_alias_evidence_ownership();
    acc_rc8_reentry_not_retransmission();
    acc_rc8_real_retransmission_combines();
    acc_rc8_backtoback_frame_drain();
    acc_rc8_resolved_cfo_keeps_harq();
    acc_rc8_search_purity();
    acc_rc8_belief_survives_turnaround();
    acc_rc8_belief_resets_on_teardown();
    acc_rc8_rediscovery_not_recombine();
    acc_rc8_geometry_uses_demod_level();
    acc_rc8_blind_shape_owner_evidence();
    acc_rc8_channel_snapshot_owned();
}
