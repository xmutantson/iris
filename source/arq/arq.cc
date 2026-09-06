#include "arq/arq.h"
#include "native/frame.h"
#include <cstring>
#include <algorithm>
#include <cassert>
#include <cstdio>

namespace iris {

namespace {
std::atomic<uint64_t> next_arq_transfer_id{1};
constexpr size_t ARQ_MAX_DRAIN_OPERATIONS = 4096;

void append_u64_be(std::vector<uint8_t>& out, uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8)
        out.push_back(static_cast<uint8_t>(value >> shift));
}

uint64_t read_u64_be(const uint8_t* bytes) {
    uint64_t value = 0;
    for (int i = 0; i != 8; ++i)
        value = (value << 8) | bytes[i];
    return value;
}
}

// --- ArqFrame serialization ---

std::vector<uint8_t> ArqFrame::serialize() const {
    std::vector<uint8_t> out;
    out.push_back((uint8_t)type);
    out.push_back(seq);
    out.push_back(flags);
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

bool ArqFrame::deserialize(const uint8_t* data, size_t len, ArqFrame& out) {
    if (len < 3) return false;
    // Validate type byte is a known ARQ frame type
    switch ((ArqType)data[0]) {
        case ArqType::HAIL: case ArqType::HAIL_ACK:
        case ArqType::CONNECT: case ArqType::CONNECT_ACK:
        case ArqType::DATA: case ArqType::ACK: case ArqType::NACK:
        case ArqType::SET_SPEED: case ArqType::SPEED_ACK:
        case ArqType::SET_CONFIG: case ArqType::CONFIG_ACK:
        case ArqType::SWITCH_ROLE: case ArqType::BREAK:
        case ArqType::DISCONNECT: case ArqType::DISCONNECT_ACK:
            break;
        default:
            return false;  // Unknown type — not an ARQ frame
    }
    out.type = (ArqType)data[0];
    if (out.type == ArqType::DATA && len - 3 > ARQ_MAX_PAYLOAD)
        return false;
    out.seq = data[1];
    out.flags = data[2];
    if (len > 3)
        out.payload.assign(data + 3, data + len);
    else
        out.payload.clear();
    return true;
}

// --- ArqSession ---

ArqSession::ArqSession() {
    reset_state();
}

bool ArqSession::mutations_deferred() const {
    return transition_depth_ != 0 || publication_depth_ != 0 ||
           draining_quiescent_;
}

void ArqSession::record_structure_violation() {
    ++structure_violations_;
    if (structure_violations_ == 1)
        std::fprintf(stderr, "[ARQ] root-rule structure violation\n");
}

ArqMutationOutcome ArqSession::run_transaction(MutationCommand command) {
    if (transition_depth_ != 0) {
        record_structure_violation();
        command_inbox_.push_back(std::move(command));
        return ArqMutationStatus::Queued;
    }
    assert(transition_depth_ == 0);
    ++transition_depth_;
    ArqMutationOutcome outcome = command();
    --transition_depth_;
    assert(transition_depth_ == 0);
    return outcome;
}

ArqMutationOutcome ArqSession::submit_command(MutationCommand command) {
    if (mutations_deferred()) {
        command_inbox_.push_back(std::move(command));
        return ArqMutationStatus::Queued;
    }
    ArqMutationOutcome outcome = run_transaction(std::move(command));
    drain_quiescent();
    return outcome;
}

void ArqSession::queue_publication(Publication publication) {
    if (transition_depth_ == 0)
        record_structure_violation();
    assert(transition_depth_ != 0);
    publish_queue_.push_back(std::move(publication));
}

void ArqSession::retire_pending_frame_publications() {
    publish_queue_.erase(
        std::remove_if(publish_queue_.begin(), publish_queue_.end(),
            [](const Publication& publication) {
                return publication.kind == PublicationKind::Frame;
            }),
        publish_queue_.end());
}

void ArqSession::drain_quiescent() {
    if (draining_quiescent_ || transition_depth_ != 0 || publication_depth_ != 0)
        return;
    draining_quiescent_ = true;
    size_t operations = 0;
    while ((!command_inbox_.empty() || !publish_queue_.empty()) &&
           operations++ < ARQ_MAX_DRAIN_OPERATIONS) {
        // A callback's commands run before effects that belonged to the
        // interrupted continuation. Reset/reconnect can therefore retire those
        // effects before any external code observes them.
        if (!command_inbox_.empty()) {
            MutationCommand command = std::move(command_inbox_.front());
            command_inbox_.pop_front();
            run_transaction(std::move(command));
            continue;
        }
        Publication publication = std::move(publish_queue_.front());
        publish_queue_.pop_front();
        if (transition_depth_ != 0) {
            record_structure_violation();
            publish_queue_.push_front(std::move(publication));
            assert(transition_depth_ == 0);
            break;
        }
        assert(transition_depth_ == 0);
        ++publication_depth_;
        callback_receive_generation_ = publication.receive_generation;
        callback_transmit_generation_ = publication.transmit_generation;
        publication.invoke();
        callback_receive_generation_ = 0;
        callback_transmit_generation_ = 0;
        --publication_depth_;
    }
    draining_quiescent_ = false;
}

bool ArqSession::finish_transfer_once(bool success, bool reset_after,
                                      bool publish_idle_state,
                                      ArqTransferResultReason reason) {
    if (!transfer_open_ || transfer_terminal_latched_)
        return false;

    // Snapshot the callable and close the transfer before invoking user code.
    // A resetting terminal path also clears all transient protocol state first;
    // otherwise a reentrant callback could start new work that the old caller
    // would subsequently erase.
    auto callback = callbacks_.on_transfer_complete;
    auto result_callback = callbacks_.on_transfer_result;
    auto state_callback = callbacks_.on_state_changed;
    ArqState prior_state = state_;
    auto mutable_result = std::make_shared<ArqTransferResult>();
    mutable_result->transfer_id = transfer_id_;
    mutable_result->success = success;
    mutable_result->reason = success ? ArqTransferResultReason::Completed : reason;
    mutable_result->accepted_records = accepted_transfer_records_;
    if (!success)
        mutable_result->preserved_records = accepted_transfer_records_;
    std::shared_ptr<const ArqTransferResult> result = mutable_result;
    retained_transfer_results_.push_back(result);
    if (!last_transfer_result_ || last_transfer_result_->success)
        last_transfer_result_ = result;
    transfer_terminal_latched_ = true;
    transfer_open_ = false;
    transfer_id_ = 0;
    accepted_transfer_records_.clear();
    transmit_bound_record_count_ = 0;
    transmit_prepared_records_pending_ = 0;
    incoming_custody_pending_ = false;
    outgoing_custody_pending_ = false;
    unprepared_outgoing_originals_ = 0;
    if (reset_after)
        reset_state();
    if (reset_after)
        retire_pending_frame_publications();
    if (reset_after && publish_idle_state && state_callback &&
        prior_state != ArqState::IDLE) {
        queue_publication({PublicationKind::State, 0, 0,
            [this, state_callback]() {
                assert(transition_depth_ == 0);
                state_callback(ArqState::IDLE);
            }});
    }
    // The two terminal callbacks are one publication item. Reentrant commands
    // cannot run between them and both callbacks retain the same snapshot.
    if (callback || result_callback) {
        queue_publication({PublicationKind::Terminal,
            receive_deferral_generation_, transmit_deferral_generation_,
            [this, callback, result_callback, success, result]() mutable {
                assert(transition_depth_ == 0);
                if (callback) callback(success);
                if (result_callback) result_callback(std::move(result));
            }});
    }
    return true;
}

bool ArqSession::publish_success_result(
        uint64_t transfer_id,
        const std::vector<std::vector<uint8_t>>& records,
        uint64_t receive_generation,
        uint64_t transmit_generation,
        std::function<void()> after) {
    // Allocation and record copying precede every terminal latch/frontier move.
    auto mutable_result = std::make_shared<ArqTransferResult>();
    mutable_result->transfer_id = transfer_id;
    mutable_result->success = true;
    mutable_result->reason = ArqTransferResultReason::Completed;
    mutable_result->accepted_records = records;
    std::shared_ptr<const ArqTransferResult> result = mutable_result;
    retained_transfer_results_.push_back(result);
    if (!last_transfer_result_ || last_transfer_result_->success)
        last_transfer_result_ = result;
    auto completion = callbacks_.on_transfer_complete;
    auto result_callback = callbacks_.on_transfer_result;
    if (completion || result_callback) {
        queue_publication({PublicationKind::Terminal,
            receive_generation, transmit_generation,
            [this, completion, result_callback, result,
             after = std::move(after)]() mutable {
                assert(transition_depth_ == 0);
                if (completion) completion(true);
                if (result_callback) result_callback(std::move(result));
                if (after) {
                    command_inbox_.push_back(
                        [after = std::move(after)]() mutable {
                            after();
                            return ArqMutationOutcome(ArqMutationStatus::Applied);
                        });
                }
            }});
    } else if (after) {
        command_inbox_.push_back(
            [after = std::move(after)]() mutable {
                after();
                return ArqMutationOutcome(ArqMutationStatus::Applied);
            });
    }
    return true;
}

bool ArqSession::finish_transmit_once() {
    // A transport ACK proves only bytes that were actually prepared. Originals
    // still owned by a buffering transform remain an independent custody
    // obligation and cannot be covered by that cumulative ACK.
    if (unprepared_outgoing_originals_ != 0) {
        transmit_completion_pending_ = true;
        return true;
    }
    outgoing_custody_pending_ = false;
    if (incoming_custody_pending_) return true;
    return finish_transfer_once(true);
}

ArqMutationOutcome ArqSession::reset() {
    return submit_command([this]() { return reset_impl(); });
}

ArqMutationOutcome ArqSession::reset_impl() {
    if (!finish_transfer_once(false, true, true,
                              ArqTransferResultReason::ExplicitReset)) {
        const ArqState prior_state = state_;
        reset_state();
        retire_pending_frame_publications();
        auto callback = callbacks_.on_state_changed;
        if (prior_state != ArqState::IDLE && callback)
            queue_publication({PublicationKind::State, 0, 0,
                [this, callback]() {
                    assert(transition_depth_ == 0);
                    callback(ArqState::IDLE);
                }});
    }
    return ArqMutationStatus::Applied;
}

bool ArqSession::dispose_transfer_result(uint64_t transfer_id) {
    auto it = std::find_if(
        retained_transfer_results_.begin(), retained_transfer_results_.end(),
        [transfer_id](const auto& result) {
            return result && result->transfer_id == transfer_id;
        });
    if (it == retained_transfer_results_.end())
        return false;
    retained_transfer_results_.erase(it);
    last_transfer_result_.reset();
    for (const auto& result : retained_transfer_results_) {
        if (!last_transfer_result_ || last_transfer_result_->success)
            last_transfer_result_ = result;
    }
    return true;
}

void ArqSession::reset_state() {
    role_ = ArqRole::IDLE;
    state_ = ArqState::IDLE;
    speed_level_ = 0;
    remote_snr_ = 0;
    local_snr_ = 0;
    retransmit_count_ = 0;
    chase_disable();
    peer_caps_ = 0;
    ack_timeout_ms_ = ARQ_DEFAULT_ACK_TIMEOUT_MS;
    rtt_sum_ms_ = 0;
    rtt_count_ = 0;
    tx_base_ = 0;
    tx_next_ = 0;
    end_of_data_ = false;
    transfer_open_ = false;
    transfer_terminal_latched_ = false;
    transfer_id_ = 0;
    accepted_transfer_records_.clear();
    rx_expected_ = 0;
    rx_ack_frontier_ = 0;
    connect_retries_ = 0;
    hail_attempts_ = 0;

    receive_commit_deferred_ = false;
    receive_deferral_generation_ = 0;
    receive_ack_pending_ = false;
    deferred_receive_batches_.clear();
    transmit_commit_deferred_ = false;
    transmit_deferral_generation_ = 0;
    transmit_completion_pending_ = false;
    incoming_custody_pending_ = false;
    outgoing_custody_pending_ = false;
    unprepared_outgoing_originals_ = 0;
    rx_expected_fragment_id_ = 0;
    rx_active_record_id_ = 0;
    rx_active_record_type_ = ArqRecordType::Data;
    rx_last_completed_record_id_ = 0;
    rx_custody_charge_ = 0;
    next_tx_record_id_ = 1;
    next_tx_fragment_id_ = 0;
    tx_acknowledged_fragment_boundary_ = 0;
    transmit_batches_.clear();
    transmit_bound_record_count_ = 0;
    transmit_prepared_records_pending_ = 0;

    turbo_phase_ = TurboPhase::NONE;
    turbo_target_speed_ = 0;
    turbo_last_good_ = 0;
    turbo_retries_ = 0;
    turbo_forward_done_ = false;
    turbo_verification_count_ = 0;

    consec_data_acks_ = 0;
    consec_data_nacks_ = 0;
    gearshift_just_applied_ = false;
    gearshift_cooldown_ = 0;
    modem_recommended_level_ = 0;
    modem_recommended_snr_ = 0;
    modem_cooldown_ = 0;

    break_phase_ = BreakPhase::NONE;
    break_recovery_speed_ = 0;
    break_retries_ = 0;
    break_drop_step_ = 1;
    emergency_nack_count_ = 0;

    memset(peer_x25519_pub_, 0, 32);
    has_peer_x25519_ = false;

    rx_mute_ = false;

    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        tx_window_[i] = TxSlot{};
        rx_window_[i] = RxSlot{};
    }

    while (!tx_data_queue_.empty()) tx_data_queue_.pop();
    while (!tx_record_end_queue_.empty()) tx_record_end_queue_.pop();
    while (!tx_record_type_queue_.empty()) tx_record_type_queue_.pop();
    while (!tx_record_id_queue_.empty()) tx_record_id_queue_.pop();

    clear_harq_state();
}

void ArqSession::set_state(ArqState s) {
    state_ = s;
    auto callback = callbacks_.on_state_changed;
    if (callback)
        queue_publication({PublicationKind::State, 0, 0,
            [this, callback, s]() {
                assert(transition_depth_ == 0);
                callback(s);
            }});
}

void ArqSession::set_speed(int level) {
    level = std::max(0, std::min(level, NUM_SPEED_LEVELS - 1));
    if (level != speed_level_) {
        speed_level_ = level;
        chase_clear();  // Speed change = different frame format, old LLRs invalid
        clear_harq_state();
        auto callback = callbacks_.on_speed_changed;
        if (callback)
            queue_publication({PublicationKind::Speed, 0, 0,
                [this, callback, level]() {
                    assert(transition_depth_ == 0);
                    callback(level);
                }});
    }
}

void ArqSession::update_ack_timeout(int64_t rtt_ms) {
    rtt_sum_ms_ += rtt_ms;
    rtt_count_++;
    int64_t avg = rtt_sum_ms_ / rtt_count_;
    ack_timeout_ms_ = std::max(ARQ_MIN_ACK_TIMEOUT_MS,
                      std::min(ARQ_MAX_ACK_TIMEOUT_MS, (int)(avg * 5 / 2)));
}

int ArqSession::compute_batch_timeout(int batch_size, int msg_tx_time_ms) const {
    // Mercury-style: covers full TX + ACK round-trip + PTT delays
    int tx_time = batch_size * msg_tx_time_ms;
    int ack_time = msg_tx_time_ms + 200;   // ACK pattern + PTT settle
    int ptt_delay = 400;                    // PTT on/off transitions
    int margin = 500;                       // safety margin

    int timeout = tx_time + ack_time + ptt_delay + margin;

    // Turboshift extension
    if (state_ == ArqState::TURBOSHIFT)
        timeout += 2000;

    return std::max(ARQ_MIN_ACK_TIMEOUT_MS,
           std::min(ARQ_MAX_ACK_TIMEOUT_MS, timeout));
}

void ArqSession::send_arq_frame(const ArqFrame& frame) {
    auto data = frame.serialize();
    auto callback = callbacks_.send_frame;
    if (callback)
        queue_publication({PublicationKind::Frame, 0, 0,
            [this, callback, data = std::move(data)]() {
                assert(transition_depth_ == 0);
                callback(data.data(), data.size());
            }});
    last_send_time_ = Clock::now();
}

// --- Role-switch buffer protection ---

void ArqSession::clear_stale_buffers() {
    // Clear RX window to prevent stale audio from previous direction
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++)
        rx_window_[i] = RxSlot{};
    rx_expected_ = 0;
    rx_ack_frontier_ = 0;
    rx_active_record_id_ = 0;

    // Mute RX for ~200ms to let stale audio drain.
    // VHF FM audio pipeline latency is ~50-100ms; 200ms is sufficient.
    rx_mute_ = true;
    rx_mute_until_ = Clock::now() + std::chrono::milliseconds(200);

    // Receive deferral belongs to modem-held custody, not the disposable radio
    // window. The role-switch callback resolves it before this cleanup; retaining
    // it here is fail-safe if a future callback elects to defer resolution.
}

// --- HAIL beacon ---

ArqMutationOutcome ArqSession::listen() {
    return submit_command([this]() { return listen_impl(); });
}

ArqMutationOutcome ArqSession::listen_impl() {
    if (transfer_open_) {
        reset_impl();
        return ArqMutationStatus::Applied;
    }
    reset_state();
    role_ = ArqRole::LISTENING;
    set_state(ArqState::LISTENING);
    return ArqMutationStatus::Applied;
}

// --- Connection with HAIL + capability negotiation ---

ArqMutationOutcome ArqSession::connect(const std::string& remote_callsign) {
    return submit_command([this, remote_callsign]() {
        return connect_impl(remote_callsign);
    });
}

ArqMutationOutcome ArqSession::connect_impl(const std::string& remote_callsign) {
    if (transfer_open_) {
        reset_impl();
        return ArqMutationStatus::Rejected;
    }
    reset_state();
    transfer_open_ = true;
    transfer_terminal_latched_ = false;
    transfer_id_ = next_arq_transfer_id.fetch_add(1);
    accepted_transfer_records_.clear();
    role_ = ArqRole::COMMANDER;
    remote_callsign_ = remote_callsign;
    hail_attempts_ = 0;

    // Start with HAIL beacon phase
    set_state(ArqState::HAILING);
    connect_time_ = Clock::now();

    ArqFrame frame;
    frame.type = ArqType::HAIL;
    frame.seq = 0;
    frame.flags = 0;
    frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(frame);
    hail_attempts_++;
    return ArqMutationStatus::Applied;
}

void ArqSession::handle_hail(const ArqFrame& frame) {
    if (state_ != ArqState::LISTENING && state_ != ArqState::IDLE)
        return;

    role_ = ArqRole::RESPONDER;
    remote_callsign_.assign(frame.payload.begin(), frame.payload.end());

    // ACK the HAIL beacon
    ArqFrame ack;
    ack.type = ArqType::HAIL_ACK;
    ack.seq = 0;
    ack.flags = 0;
    ack.payload.insert(ack.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(ack);

    // Wait for CONNECT
    set_state(ArqState::CONNECTING);
    connect_time_ = Clock::now();
}

void ArqSession::handle_hail_ack(const ArqFrame& frame) {
    if (state_ != ArqState::HAILING)
        return;

    // Responder detected — proceed to CONNECT
    if (frame.payload.size() > 0)
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());

    set_state(ArqState::CONNECTING);
    connect_retries_ = 0;
    connect_time_ = Clock::now();

    ArqFrame conn;
    conn.type = ArqType::CONNECT;
    conn.seq = 0;
    conn.flags = (uint8_t)speed_level_;
    conn.payload.push_back((uint8_t)(local_caps_ >> 8));
    conn.payload.push_back((uint8_t)(local_caps_ & 0xFF));
    // Append X25519 public key (32 bytes) for DH key exchange
    if (has_local_x25519_) {
        conn.payload.insert(conn.payload.end(), local_x25519_pub_, local_x25519_pub_ + 32);
    }
    conn.payload.insert(conn.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(conn);
}

ArqMutationOutcome ArqSession::send_data(const uint8_t* data, size_t len) {
    return send_record(data, len, data, len);
}

ArqMutationOutcome ArqSession::send_record(
        const uint8_t* encoded, size_t encoded_len,
        const uint8_t* original, size_t original_len) {
    if ((encoded_len > 0 && !encoded) || (original_len > 0 && !original))
        return ArqMutationStatus::Rejected;
    std::vector<uint8_t> encoded_copy;
    std::vector<uint8_t> original_copy;
    if (encoded_len) encoded_copy.assign(encoded, encoded + encoded_len);
    if (original_len) original_copy.assign(original, original + original_len);
    return submit_command(
        [this, encoded_copy = std::move(encoded_copy),
         original_copy = std::move(original_copy)]() mutable {
            return send_record_impl(encoded_copy, original_copy);
        });
}

ArqMutationOutcome ArqSession::send_record_impl(
        const std::vector<uint8_t>& encoded,
        const std::vector<uint8_t>& original) {
    if (encoded.empty() || original.empty()) return ArqMutationStatus::NoOp;
    retain_original_impl(original);
    return send_prepared_impl(encoded, ArqRecordType::Data, 1);
}

ArqMutationOutcome ArqSession::retain_original(
        const uint8_t* original, size_t original_len) {
    if (original_len == 0 || !original) return ArqMutationStatus::NoOp;
    std::vector<uint8_t> copy(original, original + original_len);
    return submit_command([this, copy = std::move(copy)]() mutable {
        return retain_original_impl(copy);
    });
}

ArqMutationOutcome ArqSession::retain_original_impl(
        const std::vector<uint8_t>& original) {
    if (original.empty()) return ArqMutationStatus::NoOp;
    if (!transfer_open_) {
        transfer_open_ = true;
        transfer_terminal_latched_ = false;
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
        accepted_transfer_records_.clear();
    }
    accepted_transfer_records_.push_back(original);
    ++unprepared_outgoing_originals_;
    outgoing_custody_pending_ = true;
    return ArqMutationStatus::Applied;
}

ArqMutationOutcome ArqSession::mark_originals_prepared(size_t count) {
    return submit_command([this, count]() {
        return mark_originals_prepared_impl(count);
    });
}

ArqMutationOutcome ArqSession::mark_originals_prepared_impl(size_t count) {
    if (count == 0 || unprepared_outgoing_originals_ == 0)
        return ArqMutationStatus::NoOp;
    const size_t applied = std::min(count, unprepared_outgoing_originals_);
    unprepared_outgoing_originals_ -= applied;
    transmit_prepared_records_pending_ += applied;
    return ArqMutationStatus::Applied;
}

ArqMutationOutcome ArqSession::mark_all_originals_prepared(
        size_t retained_tail_count) {
    return submit_command([this, retained_tail_count]() {
        return mark_all_originals_prepared_impl(retained_tail_count);
    });
}

ArqMutationOutcome ArqSession::mark_all_originals_prepared_impl(
        size_t retained_tail_count) {
    // A streaming transform may return transport bytes for a completed prefix
    // while retaining a suffix of the most recently submitted original.  Clear
    // only the proven prefix; the retained tail remains an ACK-independent
    // custody obligation.
    const size_t before = unprepared_outgoing_originals_;
    unprepared_outgoing_originals_ = std::min(retained_tail_count, before);
    transmit_prepared_records_pending_ +=
        before - unprepared_outgoing_originals_;
    return ArqMutationStatus::Applied;
}

ArqMutationOutcome ArqSession::send_prepared(
        const uint8_t* encoded, size_t encoded_len, ArqRecordType type,
        size_t prepared_original_count) {
    if (encoded_len == 0 || !encoded) return ArqMutationStatus::NoOp;
    std::vector<uint8_t> copy(encoded, encoded + encoded_len);
    return submit_command(
        [this, copy = std::move(copy), type, prepared_original_count]() mutable {
            return send_prepared_impl(copy, type, prepared_original_count);
        });
}

ArqMutationOutcome ArqSession::send_prepared_impl(
        const std::vector<uint8_t>& encoded, ArqRecordType type,
        size_t prepared_original_count) {
    if (encoded.empty()) return ArqMutationStatus::NoOp;
    if (type != ArqRecordType::Data &&
        type != ArqRecordType::MlKemPublicKey &&
        type != ArqRecordType::MlKemCiphertext) {
        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
        return ArqMutationStatus::Rejected;
    }
    if (!transfer_open_) {
        // Internal/control producers without an application original still need
        // a terminal transaction, but have no recoverable client bytes to add.
        transfer_open_ = true;
        transfer_terminal_latched_ = false;
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
        accepted_transfer_records_.clear();
    }
    // Bind prepared custody before publishing bytes through send_frame: that
    // callback may synchronously route a peer ACK back into this session.
    mark_originals_prepared_impl(prepared_original_count);
    outgoing_custody_pending_ = true;
    // Adaptive block count: limit payload size based on SNR margin.
    // At marginal SNR, shorter frames (fewer LDPC blocks) have less phase drift
    // and higher decode probability. Tradeoff: more preamble overhead per data bit.
    int max_payload = ARQ_MAX_PAYLOAD - static_cast<int>(ARQ_RECORD_IDENTITY_BYTES);
    if (speed_level_ >= 0 && speed_level_ < NUM_SPEED_LEVELS) {
        float threshold = SPEED_LEVELS[speed_level_].min_snr_db;
        float margin = remote_snr_ - threshold;
        if (margin < 3.0f)       max_payload = 150;   // ~1 LDPC block
        else if (margin < 6.0f)  max_payload = 450;   // ~3 blocks
        else if (margin < 10.0f) max_payload = 750;   // ~5 blocks
        // else full ARQ_MAX_PAYLOAD
    }

    const uint64_t record_id = next_tx_record_id_++;
    TransmitBatch batch;
    const size_t bindable = std::min(
        transmit_prepared_records_pending_,
        accepted_transfer_records_.size() -
            std::min(transmit_bound_record_count_,
                     accepted_transfer_records_.size()));
    if (bindable != 0) {
        batch.records.assign(
            accepted_transfer_records_.begin() + transmit_bound_record_count_,
            accepted_transfer_records_.begin() + transmit_bound_record_count_ + bindable);
        transmit_bound_record_count_ += bindable;
        transmit_prepared_records_pending_ -= bindable;
    }
    size_t offset = 0;
    while (offset < encoded.size()) {
        size_t chunk = std::min((size_t)max_payload, encoded.size() - offset);
        tx_data_queue_.push(std::vector<uint8_t>(encoded.begin() + offset,
                                                 encoded.begin() + offset + chunk));
        offset += chunk;
        tx_record_end_queue_.push(offset == encoded.size());
        tx_record_type_queue_.push(type);
        tx_record_id_queue_.push(record_id);
    }
    batch.final_fragment_id = next_tx_fragment_id_ +
        static_cast<uint64_t>(tx_data_queue_.size()) - 1;
    batch.hold_generation = transmit_commit_deferred_
        ? transmit_deferral_generation_ : 0;
    transmit_batches_.push_back(std::move(batch));
    end_of_data_ = false;

    if ((state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT) &&
        role_ == ArqRole::COMMANDER)
        send_next_data();
    else if (state_ == ArqState::CONNECTED && role_ == ArqRole::RESPONDER) {
        // Responder needs to become commander to send data — request role switch
        printf("[ARQ] Responder has data to send, requesting role switch\n");
        fflush(stdout);
        if (incoming_custody_pending_)
            role_switch_pending_ = true;
        else
            initiate_role_switch();
    }
    return ArqMutationStatus::Applied;
}

ArqMutationOutcome ArqSession::fail_active_transfer(
        ArqTransferResultReason reason) {
    return submit_command([this, reason]() {
        return fail_active_transfer_impl(reason);
    });
}

ArqMutationOutcome ArqSession::fail_active_transfer_impl(
        ArqTransferResultReason reason) {
    const bool published = finish_transfer_once(false, false, false, reason);
    // Terminal failure revokes all scheduling authority before publication.
    tx_base_ = tx_next_;
    for (auto& slot : tx_window_) slot = TxSlot{};
    while (!tx_data_queue_.empty()) tx_data_queue_.pop();
    while (!tx_record_end_queue_.empty()) tx_record_end_queue_.pop();
    while (!tx_record_type_queue_.empty()) tx_record_type_queue_.pop();
    while (!tx_record_id_queue_.empty()) tx_record_id_queue_.pop();
    transmit_commit_deferred_ = false;
    transmit_deferral_generation_ = 0;
    transmit_completion_pending_ = false;
    transmit_batches_.clear();
    role_switch_pending_ = false;
    clear_harq_state();
    if (state_ != ArqState::IDLE && state_ != ArqState::DISCONNECTING)
        disconnect_impl();
    return published ? ArqMutationStatus::Applied : ArqMutationStatus::NoOp;
}

ArqMutationOutcome ArqSession::defer_receive_commit() {
    const uint64_t generation = next_receive_deferral_generation_++;
    return submit_command([this, generation]() {
        receive_commit_deferred_ = true;
        receive_deferral_generation_ = generation;
        for (auto& batch : deferred_receive_batches_)
            batch.hold_generation = generation;
        return ArqMutationOutcome(ArqMutationStatus::Applied);
    });
}

ArqMutationOutcome ArqSession::commit_deferred_receive() {
    const uint64_t generation = publication_depth_ != 0
        ? callback_receive_generation_ : receive_deferral_generation_;
    return submit_command([this, generation]() {
        return commit_deferred_receive_impl(generation);
    });
}

ArqMutationOutcome ArqSession::commit_deferred_receive_impl(
        uint64_t generation) {
    if (!receive_commit_deferred_ || generation == 0 ||
        generation != receive_deferral_generation_)
        return ArqMutationStatus::NoOp;
    if (!deferred_receive_batches_.empty()) {
        if (deferred_receive_batches_.front().hold_generation != generation)
            return ArqMutationStatus::NoOp;
        DeferredReceiveBatch batch =
            std::move(deferred_receive_batches_.front());
        deferred_receive_batches_.pop_front();
        receive_commit_deferred_ = false;
        receive_deferral_generation_ = 0;
        incoming_custody_pending_ = !deferred_receive_batches_.empty();
        publish_success_result(batch.transfer_id, batch.records, generation, 0);
        if (state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT) {
            rx_ack_frontier_ = std::max(rx_ack_frontier_,
                                        batch.ack_fragment_boundary);
            send_receive_ack(batch.ack_fragment_boundary);
        }
        if (role_switch_pending_ &&
            deferred_receive_batches_.empty()) {
            role_switch_pending_ = false;
            initiate_role_switch();
        }
        receive_ack_pending_ = false;
        return ArqMutationStatus::Applied;
    }
    receive_commit_deferred_ = false;
    receive_deferral_generation_ = 0;
    const bool send_ack = receive_ack_pending_;
    receive_ack_pending_ = false;
    // A stamp-matched commit may emit only a captured batch boundary or this
    // consumed frontier, never a boundary re-derived from the live RX cursors.
    if (send_ack && rx_ack_frontier_ > 0 &&
        (state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT))
        send_receive_ack(rx_ack_frontier_);
    return ArqMutationStatus::Applied;
}

ArqMutationOutcome ArqSession::defer_transmit_commit() {
    const uint64_t generation = next_transmit_deferral_generation_++;
    return submit_command([this, generation]() {
        transmit_commit_deferred_ = true;
        transmit_deferral_generation_ = generation;
        for (auto& batch : transmit_batches_)
            if (batch.acknowledged)
                batch.hold_generation = generation;
        return ArqMutationOutcome(ArqMutationStatus::Applied);
    });
}

ArqMutationOutcome ArqSession::commit_deferred_transmit() {
    const uint64_t generation = publication_depth_ != 0
        ? callback_transmit_generation_ : transmit_deferral_generation_;
    return submit_command([this, generation]() {
        return commit_deferred_transmit_impl(generation);
    });
}

ArqMutationOutcome ArqSession::commit_deferred_transmit_impl(
        uint64_t generation) {
    if (!transmit_commit_deferred_ || generation == 0 ||
        generation != transmit_deferral_generation_)
        return ArqMutationStatus::NoOp;
    if (release_transmit_generation(generation))
        return ArqMutationStatus::Applied;
    transmit_commit_deferred_ = false;
    transmit_deferral_generation_ = 0;
    if (transmit_completion_pending_ && end_of_data_ && tx_base_ == tx_next_ &&
        tx_data_queue_.empty()) {
        transmit_completion_pending_ = false;
        return finish_transmit_once() ? ArqMutationStatus::Applied
                                      : ArqMutationStatus::NoOp;
    }
    transmit_completion_pending_ = false;
    send_next_data();
    return ArqMutationStatus::Applied;
}

void ArqSession::retire_transmit_batch_records(const TransmitBatch& batch) {
    const size_t count = std::min(batch.records.size(),
                                  accepted_transfer_records_.size());
    accepted_transfer_records_.erase(accepted_transfer_records_.begin(),
                                      accepted_transfer_records_.begin() + count);
    transmit_bound_record_count_ -=
        std::min(count, transmit_bound_record_count_);
}

bool ArqSession::release_transmit_generation(uint64_t generation) {
    if (transmit_batches_.empty() ||
        !transmit_batches_.front().acknowledged ||
        transmit_batches_.front().hold_generation != generation)
        return false;
    const uint64_t receipt = transfer_id_;
    TransmitBatch batch = std::move(transmit_batches_.front());
    transmit_batches_.pop_front();
    retire_transmit_batch_records(batch);
    outgoing_custody_pending_ = !transmit_batches_.empty() ||
                                unprepared_outgoing_originals_ != 0;
    if (outgoing_custody_pending_ || incoming_custody_pending_) {
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
    } else {
        transfer_open_ = false;
        transfer_terminal_latched_ = true;
        transfer_id_ = 0;
    }
    publish_success_result(
        receipt, batch.records, 0, generation,
        [this, generation]() {
            if (transmit_commit_deferred_ &&
                transmit_deferral_generation_ == generation) {
                if (!release_transmit_generation(generation)) {
                    transmit_commit_deferred_ = false;
                    transmit_deferral_generation_ = 0;
                }
            }
            else if (generation == 0)
                release_transmit_generation(0);
        });
    return true;
}

void ArqSession::acknowledge_transmit_batches(uint64_t fragment_boundary) {
    for (auto& batch : transmit_batches_) {
        if (batch.final_fragment_id >= fragment_boundary) break;
        batch.acknowledged = true;
        batch.hold_generation = transmit_commit_deferred_
            ? transmit_deferral_generation_ : 0;
    }
    if (!transmit_commit_deferred_)
        release_transmit_generation(0);
}

ArqMutationOutcome ArqSession::fail_record(
        const uint8_t* original, size_t original_len,
        ArqTransferResultReason reason) {
    if (original_len > 0 && !original) return ArqMutationStatus::Rejected;
    std::vector<uint8_t> copy;
    if (original_len) copy.assign(original, original + original_len);
    return submit_command([this, copy = std::move(copy), reason]() mutable {
        if (!copy.empty()) retain_original_impl(copy);
        return fail_active_transfer_impl(reason);
    });
}

int ArqSession::tx_queue_bytes() const {
    return (int)tx_data_queue_.size() * ARQ_MAX_PAYLOAD;
}

int ArqSession::pending_frames() const {
    // Queued chunks + unacked window slots
    int queued = (int)tx_data_queue_.size();
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (tx_window_[i].sent && !tx_window_[i].acked)
            queued++;
    }
    return queued;
}

void ArqSession::fill_data_frame(ArqFrame& frame, int slot, uint8_t seq) const {
    const auto& tx = tx_window_[slot];
    frame.type = ArqType::DATA;
    frame.seq = seq;
    // bit5 marks the identity header, bits0-1 carry the explicit record type.
    frame.flags = 0x20 | static_cast<uint8_t>(tx.record_type);
    if (tx.record_end) frame.flags |= 0x40;
    if (tx.batch_end) frame.flags |= 0x80;
    frame.payload.clear();
    frame.payload.reserve(ARQ_RECORD_IDENTITY_BYTES + tx.data.size());
    append_u64_be(frame.payload, tx.record_id);
    append_u64_be(frame.payload, tx.fragment_id);
    frame.payload.insert(frame.payload.end(), tx.data.begin(), tx.data.end());
}

void ArqSession::send_next_data() {
    if (role_ != ArqRole::COMMANDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    while (tx_next_ != (tx_base_ + ARQ_WINDOW_SIZE) % (ARQ_WINDOW_SIZE * 2)) {
        int slot = tx_next_ % ARQ_WINDOW_SIZE;

        if (!tx_window_[slot].sent || tx_window_[slot].acked) {
            if (tx_data_queue_.empty()) {
                end_of_data_ = true;
                break;
            }

            tx_window_[slot].data = std::move(tx_data_queue_.front());
            tx_data_queue_.pop();
            tx_window_[slot].record_end = !tx_record_end_queue_.empty() &&
                                          tx_record_end_queue_.front();
            if (!tx_record_end_queue_.empty()) tx_record_end_queue_.pop();
            tx_window_[slot].record_type = !tx_record_type_queue_.empty()
                ? tx_record_type_queue_.front() : ArqRecordType::Data;
            if (!tx_record_type_queue_.empty()) tx_record_type_queue_.pop();
            tx_window_[slot].record_id = !tx_record_id_queue_.empty()
                ? tx_record_id_queue_.front() : 0;
            if (!tx_record_id_queue_.empty()) tx_record_id_queue_.pop();
            tx_window_[slot].fragment_id = next_tx_fragment_id_++;
            tx_window_[slot].acked = false;
            tx_window_[slot].sent = true;
            tx_window_[slot].retries = 0;
            tx_window_[slot].send_time = Clock::now();

            if (tx_data_queue_.empty()) {
                end_of_data_ = true;
            }
            tx_window_[slot].batch_end = tx_data_queue_.empty();

            ArqFrame frame;
            fill_data_frame(frame, slot,
                            static_cast<uint8_t>(tx_next_ % ARQ_WINDOW_SIZE));
            send_arq_frame(frame);
            tx_next_++;
        } else {
            break;
        }
    }
}

// --- Frame dispatch ---

bool ArqSession::on_frame_received(const uint8_t* data, size_t len) {
    ArqFrame parsed;
    if (!ArqFrame::deserialize(data, len, parsed)) return false;
    std::vector<uint8_t> copy(data, data + len);
    submit_command([this, copy = std::move(copy)]() mutable {
        return on_frame_received_impl(copy)
            ? ArqMutationOutcome(ArqMutationStatus::Applied)
            : ArqMutationOutcome(ArqMutationStatus::Rejected);
    });
    return true;
}

bool ArqSession::on_frame_received_impl(const std::vector<uint8_t>& input) {
    const uint8_t* data = input.data();
    const size_t len = input.size();
    // RX mute suppresses stale data/config audio after a role switch, but terminal
    // control is never suppressible: a peer's fail-closed DISCONNECT must reach
    // the custody owner immediately.
    if (rx_mute_) {
        if (Clock::now() >= rx_mute_until_)
            rx_mute_ = false;
        else if (len < 1 ||
                 (data[0] != static_cast<uint8_t>(ArqType::DISCONNECT) &&
                  data[0] != static_cast<uint8_t>(ArqType::DISCONNECT_ACK)))
            return true;  // Muted, consider handled
    }

    ArqFrame frame;
    if (!ArqFrame::deserialize(data, len, frame))
        return false;  // Not an ARQ frame

    switch (frame.type) {
        case ArqType::HAIL:         handle_hail(frame); break;
        case ArqType::HAIL_ACK:     handle_hail_ack(frame); break;
        case ArqType::CONNECT:      handle_connect(frame); break;
        case ArqType::CONNECT_ACK:  handle_connect_ack(frame); break;
        case ArqType::DATA:         handle_data(frame); break;
        case ArqType::ACK:
        case ArqType::NACK:         handle_ack(frame); break;
        case ArqType::SET_SPEED:    handle_set_speed(frame); break;
        case ArqType::SPEED_ACK:    handle_speed_ack(frame); break;
        case ArqType::SET_CONFIG:   handle_set_config(frame); break;
        case ArqType::CONFIG_ACK:   handle_config_ack(frame); break;
        case ArqType::SWITCH_ROLE:  handle_switch_role(frame); break;
        case ArqType::BREAK:        handle_break(frame); break;
        case ArqType::DISCONNECT:   handle_disconnect(frame); break;
        case ArqType::DISCONNECT_ACK:
            if (state_ == ArqState::DISCONNECTING) {
                if (!finish_transfer_once(
                        false, true, true,
                        ArqTransferResultReason::PeerDisconnected)) {
                    set_state(ArqState::IDLE);
                    role_ = ArqRole::IDLE;
                }
            }
            break;
    }
    return true;
}

// --- Connection handlers with capability exchange ---

void ArqSession::handle_connect(const ArqFrame& frame) {
    if (state_ != ArqState::IDLE && state_ != ArqState::CONNECTING &&
        state_ != ArqState::LISTENING)
        return;

    role_ = ArqRole::RESPONDER;

    if (frame.payload.size() >= 2) {
        peer_caps_ = ((uint16_t)frame.payload[0] << 8) | frame.payload[1];
        // Check for X25519 pubkey: caps(2) + x25519(32) + callsign(1+)
        if ((peer_caps_ & CAP_ENCRYPTION) && frame.payload.size() >= 2 + 32 + 1) {
            memcpy(peer_x25519_pub_, frame.payload.data() + 2, 32);
            has_peer_x25519_ = true;
            remote_callsign_.assign(frame.payload.begin() + 34, frame.payload.end());
        } else {
            has_peer_x25519_ = false;
            remote_callsign_.assign(frame.payload.begin() + 2, frame.payload.end());
        }
    } else {
        peer_caps_ = 0;
        has_peer_x25519_ = false;
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    }

    // Reset per-session state to prevent stale flags from previous session
    receive_commit_deferred_ = false;

    set_state(ArqState::CONNECTED);
    chase_enable();  // Enable Chase combining for this session

    ArqFrame ack;
    ack.type = ArqType::CONNECT_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    ack.payload.push_back((uint8_t)(local_caps_ >> 8));
    ack.payload.push_back((uint8_t)(local_caps_ & 0xFF));
    // Append X25519 public key for DH key exchange
    if (has_local_x25519_) {
        ack.payload.insert(ack.payload.end(), local_x25519_pub_, local_x25519_pub_ + 32);
    }
    ack.payload.insert(ack.payload.end(), callsign_.begin(), callsign_.end());
    send_arq_frame(ack);
}

void ArqSession::handle_connect_ack(const ArqFrame& frame) {
    if (state_ != ArqState::CONNECTING)
        return;

    if (frame.payload.size() >= 2) {
        peer_caps_ = ((uint16_t)frame.payload[0] << 8) | frame.payload[1];
        // Check for X25519 pubkey: caps(2) + x25519(32) + callsign(1+)
        if ((peer_caps_ & CAP_ENCRYPTION) && frame.payload.size() >= 2 + 32 + 1) {
            memcpy(peer_x25519_pub_, frame.payload.data() + 2, 32);
            has_peer_x25519_ = true;
            remote_callsign_.assign(frame.payload.begin() + 34, frame.payload.end());
        } else {
            has_peer_x25519_ = false;
            remote_callsign_.assign(frame.payload.begin() + 2, frame.payload.end());
        }
    } else {
        peer_caps_ = 0;
        has_peer_x25519_ = false;
        remote_callsign_.assign(frame.payload.begin(), frame.payload.end());
    }

    // Reset per-session state to prevent stale flags from previous session
    receive_commit_deferred_ = false;

    set_state(ArqState::CONNECTED);
    chase_enable();  // Enable Chase combining for this session
    start_turboshift();
    send_next_data();
}

// --- Decode failure notification (NACK path for Chase combining) ---

void ArqSession::on_decode_failed() {
    submit_command([this]() {
        on_decode_failed_impl();
        return ArqMutationOutcome(ArqMutationStatus::Applied);
    });
}

void ArqSession::on_decode_failed_impl() {
    // Only responder in an active session should send NACKs
    if (role_ != ArqRole::RESPONDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;
    if (rx_mute_) return;  // Don't NACK during mute window (stale audio)

    // Send NACK for the next expected sequence number.
    // We don't know the actual seq of the failed frame (LDPC failed before
    // we could read the payload), but rx_expected_ is our best guess.
    ArqFrame nack;
    nack.type = ArqType::NACK;
    nack.seq = (uint8_t)(rx_expected_ % ARQ_WINDOW_SIZE);
    nack.flags = 0;  // no selective ACK mask
    int16_t snr_scaled = (int16_t)(local_snr_ * 100.0f);
    nack.payload.push_back((uint8_t)(snr_scaled & 0xFF));
    nack.payload.push_back((uint8_t)((snr_scaled >> 8) & 0xFF));
    send_arq_frame(nack);

    printf("[ARQ] NACK sent for seq %d (decode failed, Chase LLRs stored)\n",
           rx_expected_ % ARQ_WINDOW_SIZE);
    fflush(stdout);
}

// --- Data and ACK handlers ---

void ArqSession::send_receive_ack(uint64_t fragment_boundary) {
    const bool current_boundary = fragment_boundary == UINT64_MAX;
    if (current_boundary)
        fragment_boundary = std::max(rx_expected_fragment_id_,
                                     static_cast<uint64_t>(rx_expected_));
    uint8_t ack_mask = 0;
    if (current_boundary) {
        for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
            int s = (rx_expected_ + i) % ARQ_WINDOW_SIZE;
            if (rx_window_[s].received)
                ack_mask |= (1 << i);
        }
    }

    ArqFrame ack;
    ack.type = ArqType::ACK;
    ack.seq = static_cast<uint8_t>(fragment_boundary % ARQ_WINDOW_SIZE);
    ack.flags = ack_mask;
    int16_t snr_scaled = static_cast<int16_t>(local_snr_ * 100.0f);
    ack.payload.push_back(static_cast<uint8_t>(snr_scaled & 0xFF));
    ack.payload.push_back(static_cast<uint8_t>((snr_scaled >> 8) & 0xFF));
    append_u64_be(ack.payload, fragment_boundary);
    send_arq_frame(ack);
}

void ArqSession::handle_data(const ArqFrame& frame) {
    if (role_ != ArqRole::RESPONDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    int seq = frame.seq % ARQ_WINDOW_SIZE;
    const bool has_identity = (frame.flags & 0x20) != 0;
    ArqRecordType record_type = ArqRecordType::Data;
    uint64_t record_id = 0;
    uint64_t fragment_id = 0;
    const uint8_t* fragment_data = frame.payload.data();
    size_t fragment_size = frame.payload.size();

    if (has_identity) {
        const uint8_t raw_type = frame.flags & 0x03;
        if (frame.payload.size() < ARQ_RECORD_IDENTITY_BYTES || raw_type >
                static_cast<uint8_t>(ArqRecordType::MlKemCiphertext)) {
            fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
            return;
        }
        record_type = static_cast<ArqRecordType>(raw_type);
        record_id = read_u64_be(frame.payload.data());
        fragment_id = read_u64_be(frame.payload.data() + 8);
        fragment_data += ARQ_RECORD_IDENTITY_BYTES;
        fragment_size -= ARQ_RECORD_IDENTITY_BYTES;

        // The absolute fragment identity closes the modulo-8 ambiguity. Old
        // DATA can be acknowledged idempotently but can never refill a wrapped
        // slot or reach inverse transforms again.
        if (record_id == 0) {
            fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
            return;
        }
        if (fragment_id < rx_expected_fragment_id_ ||
            record_id <= rx_last_completed_record_id_) {
            if (receive_commit_deferred_)
                receive_ack_pending_ = true;
            else
                send_receive_ack();
            return;
        }
        if (fragment_id - rx_expected_fragment_id_ >= ARQ_WINDOW_SIZE)
            return;
        if (record_id - rx_last_completed_record_id_ > ARQ_WINDOW_SIZE) {
            fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
            return;
        }
    }

    // Legacy DATA has no absolute identity, so admit only the frame currently
    // named by V(R). Frames behind V(R) are retransmissions; frames ahead will
    // be retransmitted after the cumulative ACK exposes the gap. This keeps a
    // completed EOB replay out of a wrapped slot without suppressing the next
    // legitimate in-sequence record.
    if (!has_identity && seq != rx_expected_ % ARQ_WINDOW_SIZE) {
        if (receive_commit_deferred_)
            receive_ack_pending_ = true;
        else
            send_receive_ack();
        return;
    }

    // Completion is a transaction boundary, not a receive-direction latch.
    // A genuinely new frame opens the next transaction in the same role.
    if (!transfer_open_) {
        transfer_open_ = true;
        transfer_terminal_latched_ = false;
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
        accepted_transfer_records_.clear();
    }
    incoming_custody_pending_ = true;

    int offset = has_identity
        ? static_cast<int>(fragment_id - rx_expected_fragment_id_)
        : seq - (rx_expected_ % ARQ_WINDOW_SIZE);
    if (!has_identity && offset < 0) offset += ARQ_WINDOW_SIZE;

    if (has_identity &&
        seq != (rx_expected_ + offset) % ARQ_WINDOW_SIZE) {
        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
        return;
    }

    if (offset < ARQ_WINDOW_SIZE) {
        int slot = seq;
        if (rx_window_[slot].received && has_identity &&
            (rx_window_[slot].fragment_id != fragment_id ||
             rx_window_[slot].record_id != record_id ||
             rx_window_[slot].record_type != record_type ||
             rx_window_[slot].end != ((frame.flags & (0x40 | 0x80)) != 0) ||
             rx_window_[slot].batch_end != ((frame.flags & 0x80) != 0) ||
             rx_window_[slot].data.size() != fragment_size ||
             !std::equal(rx_window_[slot].data.begin(),
                         rx_window_[slot].data.end(), fragment_data))) {
            fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
            return;
        }
        if (!rx_window_[slot].received) {
            const size_t charge = ARQ_FRAGMENT_CUSTODY_CHARGE + fragment_size;
            if (has_identity) {
                if (rx_active_record_id_ == record_id &&
                    rx_active_record_type_ != record_type) {
                    fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
                    return;
                }
                for (const auto& admitted : rx_window_) {
                    if (admitted.received && admitted.has_identity &&
                        admitted.record_id == record_id &&
                        admitted.record_type != record_type) {
                        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
                        return;
                    }
                }
            }
            if (charge > ARQ_MAX_AGGREGATE_CUSTODY_BYTES -
                    std::min(rx_custody_charge_,
                             ARQ_MAX_AGGREGATE_CUSTODY_BYTES)) {
                fail_active_transfer_impl(ArqTransferResultReason::RecordTooLarge);
                return;
            }
            // Identity, type, and aggregate charge are validated before either
            // the receive window or retained failure ledger copies peer bytes.
            rx_custody_charge_ += charge;
            rx_window_[slot].data.assign(fragment_data,
                                         fragment_data + fragment_size);
            rx_window_[slot].received = true;
            rx_window_[slot].end = (frame.flags & (0x40 | 0x80)) != 0;
            rx_window_[slot].batch_end = (frame.flags & 0x80) != 0;
            rx_window_[slot].has_identity = has_identity;
            rx_window_[slot].record_type = record_type;
            rx_window_[slot].record_id = record_id;
            rx_window_[slot].fragment_id = fragment_id;
        }
    }
    stage_next_delivery();
}

void ArqSession::stage_next_delivery() {
    const int slot = rx_expected_ % ARQ_WINDOW_SIZE;
    if (!rx_window_[slot].received) {
        if (receive_commit_deferred_)
            receive_ack_pending_ = true;
        else
            send_receive_ack();
        return;
    }

    StagedDelivery delivery;
    delivery.bytes = rx_window_[slot].data;
    delivery.end = rx_window_[slot].end;
    delivery.batch_end = rx_window_[slot].batch_end;
    delivery.has_identity = rx_window_[slot].has_identity;
    delivery.type = rx_window_[slot].record_type;
    delivery.record_id = rx_window_[slot].record_id;
    delivery.fragment_id = rx_window_[slot].fragment_id;
    if (delivery.has_identity &&
        (delivery.fragment_id != rx_expected_fragment_id_ ||
         (rx_active_record_id_ == 0 &&
          delivery.record_id != rx_last_completed_record_id_ + 1) ||
         (rx_active_record_id_ != 0 &&
          (delivery.record_id != rx_active_record_id_ ||
           delivery.type != rx_active_record_type_)))) {
        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
        return;
    }
    if (!transfer_open_) {
        transfer_open_ = true;
        transfer_terminal_latched_ = false;
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
        accepted_transfer_records_.clear();
    }
    // Copy the result material before advancing either receive frontier.
    accepted_transfer_records_.push_back(delivery.bytes);
    rx_window_[slot] = RxSlot{};
    ++rx_expected_;
    delivery.seq_boundary = static_cast<uint64_t>(rx_expected_);
    if (delivery.has_identity) {
        if (rx_active_record_id_ == 0) {
            rx_active_record_id_ = delivery.record_id;
            rx_active_record_type_ = delivery.type;
        }
        ++rx_expected_fragment_id_;
    }

    auto typed = callbacks_.on_typed_data_fragment;
    auto fragment = callbacks_.on_data_fragment;
    auto received = callbacks_.on_data_received;
    const uint64_t hold = receive_commit_deferred_
        ? receive_deferral_generation_ : 0;
    queue_publication({PublicationKind::Delivery, hold, 0,
        [this, delivery = std::move(delivery), typed, fragment, received]() mutable {
            assert(transition_depth_ == 0);
            bool accepted = true;
            if (!delivery.bytes.empty()) {
                if (typed)
                    accepted = typed(delivery.bytes.data(), delivery.bytes.size(),
                                     delivery.end, delivery.type,
                                     delivery.record_id);
                else if (fragment)
                    accepted = fragment(delivery.bytes.data(),
                                        delivery.bytes.size(), delivery.end);
                else if (received)
                    received(delivery.bytes.data(), delivery.bytes.size());
            }
            command_inbox_.push_back(
                [this, delivery = std::move(delivery), accepted]() mutable {
                    consume_delivery(std::move(delivery), accepted);
                    return ArqMutationOutcome(ArqMutationStatus::Applied);
                });
        }});
}

void ArqSession::stage_receive_batch(uint64_t ack_fragment_boundary) {
    DeferredReceiveBatch batch;
    batch.transfer_id = transfer_id_;
    batch.hold_generation = receive_commit_deferred_
        ? receive_deferral_generation_ : 0;
    batch.ack_fragment_boundary = ack_fragment_boundary;
    batch.records = accepted_transfer_records_;

    accepted_transfer_records_.clear();
    transmit_bound_record_count_ = 0;
    transmit_prepared_records_pending_ = 0;
    incoming_custody_pending_ = false;
    transfer_open_ = false;
    transfer_terminal_latched_ = true;
    transfer_id_ = 0;

    if (batch.hold_generation != 0) {
        deferred_receive_batches_.push_back(std::move(batch));
        incoming_custody_pending_ = true;
        receive_ack_pending_ = true;
    } else {
        publish_success_result(batch.transfer_id, batch.records);
        rx_ack_frontier_ = std::max(rx_ack_frontier_,
                                    batch.ack_fragment_boundary);
        send_receive_ack(ack_fragment_boundary);
        if (role_switch_pending_) {
            role_switch_pending_ = false;
            initiate_role_switch();
        }
    }

    // A delivery callback may have submitted reverse traffic. It starts a new
    // directional transaction after the receive result is independently fixed.
    if (outgoing_custody_pending_) {
        transfer_open_ = true;
        transfer_terminal_latched_ = false;
        transfer_id_ = next_arq_transfer_id.fetch_add(1);
    }
}

void ArqSession::consume_delivery(StagedDelivery delivery, bool accepted) {
    if (!accepted) {
        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
        return;
    }
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT)
        return;
    if (delivery.has_identity && delivery.end) {
        rx_last_completed_record_id_ = delivery.record_id;
        rx_active_record_id_ = 0;
        rx_active_record_type_ = ArqRecordType::Data;
    }
    if (delivery.batch_end) {
        const uint64_t boundary = delivery.has_identity
            ? delivery.fragment_id + 1
            : static_cast<uint64_t>(rx_expected_);
        stage_receive_batch(boundary);
        if (rx_window_[rx_expected_ % ARQ_WINDOW_SIZE].received)
            stage_next_delivery();
        return;
    }
    rx_ack_frontier_ = std::max(
        rx_ack_frontier_, delivery.has_identity
            ? delivery.fragment_id + 1
            : delivery.seq_boundary);
    stage_next_delivery();
}

void ArqSession::handle_ack(const ArqFrame& frame) {
    if (role_ != ArqRole::COMMANDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;

    // Extract SNR
    if (frame.payload.size() >= 2) {
        int16_t snr_scaled = (int16_t)((uint16_t)frame.payload[0] |
                                        ((uint16_t)frame.payload[1] << 8));
        remote_snr_ = snr_scaled / 100.0f;
    }

    int ack_seq = frame.seq;
    uint8_t ack_mask = frame.flags;
    const bool absolute_ack = frame.type == ArqType::ACK &&
                              frame.payload.size() >= 10;
    uint64_t fragment_boundary = 0;
    if (absolute_ack) {
        fragment_boundary = read_u64_be(frame.payload.data() + 2);
        if (fragment_boundary <= tx_acknowledged_fragment_boundary_ ||
            fragment_boundary > next_tx_fragment_id_)
            return;
    }

    // Measure RTT from oldest unacked frame
    if (tx_base_ < tx_next_) {
        int slot = tx_base_ % ARQ_WINDOW_SIZE;
        auto rtt = std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - tx_window_[slot].send_time).count();
        if (rtt > 0 && rtt < 30000)
            update_ack_timeout(rtt);
    }

    // Advance tx_base
    if (absolute_ack) {
        while (tx_base_ < tx_next_) {
            int slot = tx_base_ % ARQ_WINDOW_SIZE;
            if (!tx_window_[slot].sent ||
                tx_window_[slot].fragment_id >= fragment_boundary)
                break;
            tx_window_[slot].acked = true;
            ++tx_base_;
        }
        tx_acknowledged_fragment_boundary_ = fragment_boundary;
    } else {
        while ((tx_base_ % ARQ_WINDOW_SIZE) != ack_seq && tx_base_ < tx_next_) {
            int slot = tx_base_ % ARQ_WINDOW_SIZE;
            tx_window_[slot].acked = true;
            tx_base_++;
        }
    }

    // Selective ACK bitmask
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (ack_mask & (1 << i)) {
            int s = (ack_seq + i) % ARQ_WINDOW_SIZE;
            int abs_seq = tx_base_ + i;
            if (abs_seq < tx_next_)
                tx_window_[s].acked = true;
        }
    }
    if (absolute_ack)
        acknowledge_transmit_batches(fragment_boundary);

    // Track consecutive acks/nacks
    if (frame.type == ArqType::ACK) {
        consec_data_acks_++;
        consec_data_nacks_ = 0;
        emergency_nack_count_ = 0;

        // Reset BREAK state on successful ACK
        if (break_phase_ != BreakPhase::NONE) {
            break_phase_ = BreakPhase::NONE;
            break_retries_ = 0;
        }
    } else {
        consec_data_nacks_++;
        consec_data_acks_ = 0;

        if ((frame.flags & 0x40) && negotiated(CAP_HARQ)) {
            // HARQ extended NACK: prepare piggybacked retransmit for next frame
            handle_harq_nack(frame);
        } else {
            // Standard NACK: full retransmit with Chase combining
            int slot = ack_seq % ARQ_WINDOW_SIZE;
            if (tx_window_[slot].sent && !tx_window_[slot].acked) {
                ArqFrame data_frame;
                fill_data_frame(data_frame, slot,
                                static_cast<uint8_t>(ack_seq % ARQ_WINDOW_SIZE));
                send_arq_frame(data_frame);
                tx_window_[slot].retries++;
                retransmit_count_++;
            }
        }
    }

    // SNR-based gearshift ladder (post-turboshift)
    if (turbo_phase_ == TurboPhase::DONE || turbo_phase_ == TurboPhase::NONE) {
        if (gearshift_cooldown_ > 0) {
            gearshift_cooldown_--;
            // Conflict: modem recommends upshift but ARQ is in cooldown
            if (modem_recommended_level_ > speed_level_ && gearshift_cooldown_ > 0) {
                printf("[GEARSHIFT] conflict: modem recommends level %d (SNR %.1f dB) but ARQ in cooldown (%d blocks remaining)\n",
                       modem_recommended_level_, modem_recommended_snr_, gearshift_cooldown_);
                fflush(stdout);
            }
        }
        // Conflict: ARQ wants upshift (enough acks) but modem is in cooldown
        if (consec_data_acks_ >= TURBO_CONSEC_ACKS_TO_UPSHIFT &&
            speed_level_ < NUM_SPEED_LEVELS - 1 &&
            gearshift_cooldown_ <= 0 &&
            modem_cooldown_ > 0) {
            printf("[GEARSHIFT] conflict: ARQ ready to upshift (acks=%d) but modem in cooldown (%d frames remaining)\n",
                   consec_data_acks_, modem_cooldown_);
            fflush(stdout);
        }

        if (consec_data_acks_ >= TURBO_CONSEC_ACKS_TO_UPSHIFT &&
            speed_level_ < NUM_SPEED_LEVELS - 1 &&
            gearshift_cooldown_ <= 0) {
            // SNR-based: use SNR to suggest target, upshift by 1
            int snr_suggested = snr_to_speed_level(remote_snr_);
            if (snr_suggested > speed_level_) {
                int old = speed_level_;
                set_speed(speed_level_ + 1);
                consec_data_acks_ = 0;
                gearshift_just_applied_ = true;
                printf("[GEARSHIFT] ARQ upshift: A%d -> A%d (remote_SNR=%.1f, snr_suggested=%d, acks=%d)\n",
                       old, speed_level_, remote_snr_, snr_suggested, TURBO_CONSEC_ACKS_TO_UPSHIFT);
                fflush(stdout);
                // Conflict detection: modem says downshift but ARQ is upshifting
                if (modem_recommended_level_ < old) {
                    printf("[GEARSHIFT] WARNING: conflict: ARQ upshifted to %d but modem recommends level %d (SNR %.1f dB)\n",
                           speed_level_, modem_recommended_level_, modem_recommended_snr_);
                    fflush(stdout);
                }
            }
        }

        if (consec_data_nacks_ >= 2 && speed_level_ > 0) {
            int old = speed_level_;
            set_speed(speed_level_ - 1);
            consec_data_nacks_ = 0;
            gearshift_just_applied_ = false;
            gearshift_cooldown_ = 3;  // wait 3 blocks before upshifting again
            printf("[GEARSHIFT] ARQ downshift: A%d -> A%d (nacks=%d, cooldown=%d)\n",
                   old, speed_level_, 2, gearshift_cooldown_);
            fflush(stdout);
            // Conflict detection: modem says upshift but ARQ is downshifting
            if (modem_recommended_level_ > old) {
                printf("[GEARSHIFT] WARNING: conflict: ARQ downshifted to %d but modem recommends level %d (SNR %.1f dB)\n",
                       speed_level_, modem_recommended_level_, modem_recommended_snr_);
                fflush(stdout);
            }
        }

        // Safety net: first NACK after an upshift triggers BREAK
        if (gearshift_just_applied_ && consec_data_nacks_ >= 1) {
            gearshift_just_applied_ = false;
            initiate_break();
            return;
        }
    }

    // Emergency BREAK on excessive block-level failures
    emergency_nack_count_ += (frame.type == ArqType::NACK) ? 1 : 0;
    if (emergency_nack_count_ >= BREAK_EMERGENCY_NACK_THRESHOLD &&
        break_phase_ == BreakPhase::NONE) {
        initiate_break();
        return;
    }

    // BREAK on excessive consecutive NACKs
    if (consec_data_nacks_ >= BREAK_NACK_THRESHOLD &&
        break_phase_ == BreakPhase::NONE) {
        initiate_break();
        return;
    }

    // Transfer complete check
    if (end_of_data_ && tx_base_ == tx_next_ && tx_data_queue_.empty()) {
        if (!absolute_ack)
            acknowledge_transmit_batches(next_tx_fragment_id_);
        transmit_completion_pending_ = transmit_commit_deferred_ &&
                                       !transmit_batches_.empty();
        return;
    }

    send_next_data();
}

// --- Turboshift ---

void ArqSession::start_turboshift() {
    if (role_ != ArqRole::COMMANDER) return;

    if (remote_snr_ <= 0.0f) {
        turbo_phase_ = TurboPhase::DONE;
        return;
    }

    turbo_phase_ = TurboPhase::FORWARD;
    turbo_last_good_ = speed_level_;
    turbo_retries_ = 0;
    turbo_verification_count_ = 0;

    int suggested = snr_to_speed_level(remote_snr_);
    turbo_target_speed_ = std::max(suggested, speed_level_ + 1);
    turbo_target_speed_ = std::min(turbo_target_speed_, NUM_SPEED_LEVELS - 1);

    set_state(ArqState::TURBOSHIFT);
    turbo_probe_up();
}

void ArqSession::turbo_probe_up() {
    if (turbo_target_speed_ <= turbo_last_good_) {
        // Verification probe at ceiling: test the top config twice
        if (turbo_verification_count_ == 0 && turbo_last_good_ > 0) {
            turbo_verification_count_ = 1;
            ArqFrame frame;
            frame.type = ArqType::SET_SPEED;
            frame.seq = 0;
            frame.flags = (uint8_t)turbo_last_good_;
            send_arq_frame(frame);
            return;
        }
        turbo_settle();
        return;
    }

    ArqFrame frame;
    frame.type = ArqType::SET_SPEED;
    frame.seq = 0;
    frame.flags = (uint8_t)turbo_target_speed_;
    send_arq_frame(frame);
}

void ArqSession::handle_set_speed(const ArqFrame& frame) {
    int new_speed = frame.flags;
    new_speed = std::max(0, std::min(new_speed, NUM_SPEED_LEVELS - 1));
    set_speed(new_speed);

    ArqFrame ack;
    ack.type = ArqType::SPEED_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    send_arq_frame(ack);
}

void ArqSession::handle_speed_ack(const ArqFrame& frame) {
    if (turbo_phase_ == TurboPhase::FORWARD || turbo_phase_ == TurboPhase::REVERSE) {
        int acked_speed = frame.flags;
        set_speed(acked_speed);
        turbo_last_good_ = acked_speed;

        // Verification probe response
        if (turbo_verification_count_ > 0) {
            turbo_verification_count_++;
            if (turbo_verification_count_ >= 2) {
                // Verified — settle
                if (turbo_phase_ == TurboPhase::FORWARD) {
                    turbo_forward_done_ = true;
                    initiate_role_switch();
                } else {
                    turbo_settle();
                }
            }
            return;
        }

        if (acked_speed < NUM_SPEED_LEVELS - 1 && turbo_retries_ < TURBO_PROBE_RETRIES) {
            turbo_target_speed_ = acked_speed + 1;
            turbo_retries_++;
            turbo_probe_up();
        } else {
            if (turbo_phase_ == TurboPhase::FORWARD) {
                turbo_forward_done_ = true;
                initiate_role_switch();
            } else {
                turbo_settle();
            }
        }
    }
}

void ArqSession::turbo_settle() {
    set_speed(turbo_last_good_);
    turbo_phase_ = TurboPhase::DONE;
    set_state(ArqState::CONNECTED);
    consec_data_acks_ = 0;
    consec_data_nacks_ = 0;
    gearshift_just_applied_ = false;
}

// --- Role switching with buffer protection ---

void ArqSession::initiate_role_switch() {
    ArqFrame frame;
    frame.type = ArqType::SWITCH_ROLE;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);
}

void ArqSession::handle_switch_role(const ArqFrame& frame) {
    if (role_ == ArqRole::COMMANDER && outgoing_custody_pending_ &&
        pending_frames() != 0) {
        fail_active_transfer_impl(ArqTransferResultReason::ExplicitReset);
        return;
    }
    // Held strict-mode records must reach an authenticated/decoded terminal
    // decision before receive-window cleanup or a direction change.
    auto callback = callbacks_.on_role_switch;
    const ArqRole expected_role = role_;
    if (callback) {
        const uint64_t hold = receive_commit_deferred_
            ? receive_deferral_generation_ : 0;
        queue_publication({PublicationKind::RoleDecision, hold, 0,
            [this, callback, frame, expected_role]() {
                assert(transition_depth_ == 0);
                const bool accepted = callback();
                command_inbox_.push_back(
                    [this, frame, expected_role, accepted]() {
                        complete_role_switch(frame, expected_role, accepted);
                        return ArqMutationOutcome(ArqMutationStatus::Applied);
                    });
            }});
        return;
    }
    complete_role_switch(frame, expected_role, true);
}

void ArqSession::complete_role_switch(const ArqFrame& frame,
                                      ArqRole expected_role,
                                      bool accepted) {
    if (role_ != expected_role ||
        (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT))
        return;
    if (!accepted || receive_commit_deferred_ ||
        !deferred_receive_batches_.empty()) {
        fail_active_transfer_impl(ArqTransferResultReason::TransformFailure);
        return;
    }
    // Clear stale buffers before swapping roles
    clear_stale_buffers();

    if (role_ == ArqRole::COMMANDER) {
        role_ = ArqRole::RESPONDER;
        turbo_phase_ = TurboPhase::DONE;
        set_state(ArqState::CONNECTED);
    } else if (role_ == ArqRole::RESPONDER) {
        role_ = ArqRole::COMMANDER;
        // Reset TX window for new commander direction
        tx_base_ = 0;
        tx_next_ = 0;
        end_of_data_ = false;
        for (int i = 0; i < ARQ_WINDOW_SIZE; i++)
            tx_window_[i] = TxSlot{};

        if (!tx_data_queue_.empty()) {
            // Data waiting — skip turboshift, send immediately
            printf("[ARQ] Role switch complete, sending queued data (%d chunks)\n",
                   (int)tx_data_queue_.size());
            fflush(stdout);
            turbo_phase_ = TurboPhase::DONE;
            set_state(ArqState::CONNECTED);
            send_next_data();
        } else {
            // No data — turboshift probe (original turboshift path)
            turbo_phase_ = TurboPhase::REVERSE;
            turbo_retries_ = 0;
            turbo_last_good_ = speed_level_;
            turbo_verification_count_ = 0;

            int suggested = snr_to_speed_level(remote_snr_);
            turbo_target_speed_ = std::max(suggested, speed_level_ + 1);
            turbo_target_speed_ = std::min(turbo_target_speed_, NUM_SPEED_LEVELS - 1);

            set_state(ArqState::TURBOSHIFT);
            turbo_probe_up();
        }
    }

    // ACK the role switch
    if (frame.seq == 0) {
        ArqFrame ack;
        ack.type = ArqType::SWITCH_ROLE;
        ack.seq = 1; // seq=1 means ACK
        ack.flags = 0;
        send_arq_frame(ack);
    }
}

// --- Multi-phase BREAK recovery ---

void ArqSession::initiate_break() {
    break_recovery_speed_ = speed_level_;
    break_phase_ = BreakPhase::PHASE_1;
    break_retries_ = 0;
    consec_data_nacks_ = 0;
    emergency_nack_count_ = 0;
    gearshift_just_applied_ = false;

    printf("[ARQ] BREAK initiated at speed %d, dropping to 0\n", speed_level_);
    fflush(stdout);

    break_phase1();
}

void ArqSession::break_phase1() {
    // Phase 1: drop to lowest speed, send SET_CONFIG for sync
    set_speed(0);

    ArqFrame frame;
    frame.type = ArqType::BREAK;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);

    // Also send SET_CONFIG at speed 0 for guaranteed delivery
    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = 0; // config = speed 0
    send_arq_frame(config);
}

void ArqSession::break_phase2() {
    // Phase 2: probe at a recovery target using ladder-based drop
    int target = break_recovery_speed_;
    for (int i = 0; i < break_drop_step_ && target > 0; i++)
        target--;
    if (target < 0) target = 0;

    printf("[ARQ] BREAK phase 2: probing config %d (drop_step=%d)\n",
           target, break_drop_step_);
    fflush(stdout);

    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = (uint8_t)target;
    send_arq_frame(config);

    break_phase_ = BreakPhase::PHASE_2;
}

void ArqSession::break_probe_recovery() {
    // Phase 3: try progressively higher speeds
    int target = speed_level_ + 1;
    if (target >= NUM_SPEED_LEVELS) target = NUM_SPEED_LEVELS - 1;

    if (target <= speed_level_) {
        // At ceiling already, settle
        break_phase_ = BreakPhase::NONE;
        if (speed_level_ < break_recovery_speed_) {
            gearshift_cooldown_ = 10;  // settled below original — suppress rapid re-escalation
            printf("[ARQ] BREAK recovery complete at speed %d (below original %d, cooldown=%d)\n",
                   speed_level_, break_recovery_speed_, gearshift_cooldown_);
        } else {
            printf("[ARQ] BREAK recovery complete at speed %d\n", speed_level_);
        }
        fflush(stdout);
        return;
    }

    ArqFrame config;
    config.type = ArqType::SET_CONFIG;
    config.seq = 0;
    config.flags = (uint8_t)target;
    send_arq_frame(config);

    break_phase_ = BreakPhase::PHASE_3;
}

void ArqSession::break_exhausted() {
    // Fallback: stay at lowest working speed
    break_phase_ = BreakPhase::NONE;
    break_drop_step_ = std::min(break_drop_step_ * 2, 4);  // escalate for next BREAK
    if (speed_level_ < break_recovery_speed_) {
        gearshift_cooldown_ = 10;  // settled below original — suppress rapid re-escalation
        printf("[ARQ] BREAK exhausted, settling at speed %d (below original %d, cooldown=%d)\n",
               speed_level_, break_recovery_speed_, gearshift_cooldown_);
    } else {
        printf("[ARQ] BREAK exhausted, settling at speed %d\n", speed_level_);
    }
    fflush(stdout);
}

void ArqSession::handle_break(const ArqFrame&) {
    // Peer requested emergency fallback
    set_speed(0);

    ArqFrame ack;
    ack.type = ArqType::BREAK;
    ack.seq = 1; // seq=1 means ACK
    ack.flags = 0;
    send_arq_frame(ack);
}

void ArqSession::handle_set_config(const ArqFrame& frame) {
    int new_speed = frame.flags;
    new_speed = std::max(0, std::min(new_speed, NUM_SPEED_LEVELS - 1));
    set_speed(new_speed);

    ArqFrame ack;
    ack.type = ArqType::CONFIG_ACK;
    ack.seq = 0;
    ack.flags = (uint8_t)speed_level_;
    send_arq_frame(ack);
}

void ArqSession::handle_config_ack(const ArqFrame& frame) {
    int acked_speed = frame.flags;
    set_speed(acked_speed);

    switch (break_phase_) {
        case BreakPhase::PHASE_1:
            // Phase 1 ACK received — peer confirmed speed 0, now probe up
            break_phase2();
            break;
        case BreakPhase::PHASE_2:
            // Phase 2 ACK: recovery target confirmed
            break_phase_ = BreakPhase::PHASE_3;
            break_probe_recovery();
            break;
        case BreakPhase::PHASE_3:
            // Probe step confirmed, try higher
            break_probe_recovery();
            break;
        default:
            break;
    }
}

// --- Disconnect ---

void ArqSession::handle_disconnect(const ArqFrame&) {
    ArqFrame ack;
    ack.type = ArqType::DISCONNECT_ACK;
    ack.seq = 0;
    ack.flags = 0;
    send_arq_frame(ack);

    if (!finish_transfer_once(false, true, true,
                              ArqTransferResultReason::PeerDisconnected)) {
        set_state(ArqState::IDLE);
        role_ = ArqRole::IDLE;
    }
}

ArqMutationOutcome ArqSession::disconnect() {
    return submit_command([this]() { return disconnect_impl(); });
}

ArqMutationOutcome ArqSession::disconnect_impl() {
    if (state_ == ArqState::IDLE) return ArqMutationStatus::NoOp;

    set_state(ArqState::DISCONNECTING);

    ArqFrame frame;
    frame.type = ArqType::DISCONNECT;
    frame.seq = 0;
    frame.flags = 0;
    send_arq_frame(frame);
    return ArqMutationStatus::Applied;
}

// --- Timer tick ---

ArqMutationOutcome ArqSession::tick() {
    return submit_command([this]() { return tick_impl(); });
}

ArqMutationOutcome ArqSession::tick_impl() {
    auto now = Clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_send_time_).count();

    // Check rx_mute expiry
    if (rx_mute_ && now >= rx_mute_until_)
        rx_mute_ = false;

    if (state_ == ArqState::HAILING) {
        // HAIL beacon phase: re-send periodically
        if (elapsed_ms > HAIL_INTERVAL_MS) {
            if (hail_attempts_ >= HAIL_MAX_ATTEMPTS) {
                finish_transfer_once(false, true, false,
                                     ArqTransferResultReason::RetryExhausted);
                return ArqMutationStatus::Applied;
            }
            ArqFrame frame;
            frame.type = ArqType::HAIL;
            frame.seq = 0;
            frame.flags = 0;
            frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
            send_arq_frame(frame);
            hail_attempts_++;
        }
    } else if (state_ == ArqState::CONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            connect_retries_++;
            if (connect_retries_ >= ARQ_MAX_RETRIES) {
                finish_transfer_once(false, true, false,
                                     ArqTransferResultReason::RetryExhausted);
                return ArqMutationStatus::Applied;
            }
            ArqFrame frame;
            frame.type = ArqType::CONNECT;
            frame.seq = 0;
            frame.flags = (uint8_t)speed_level_;
            frame.payload.push_back((uint8_t)(local_caps_ >> 8));
            frame.payload.push_back((uint8_t)(local_caps_ & 0xFF));
            frame.payload.insert(frame.payload.end(), callsign_.begin(), callsign_.end());
            send_arq_frame(frame);
        }
    } else if ((state_ == ArqState::CONNECTED || state_ == ArqState::TURBOSHIFT) &&
               role_ == ArqRole::COMMANDER) {

        // Batch-aware timeout
        int timeout = compute_batch_timeout(ARQ_WINDOW_SIZE, 200);
        if (elapsed_ms > timeout && tx_base_ < tx_next_) {
            int slot = tx_base_ % ARQ_WINDOW_SIZE;
            if (tx_window_[slot].sent && !tx_window_[slot].acked) {
                tx_window_[slot].retries++;
                retransmit_count_++;

                if (tx_window_[slot].retries >= ARQ_MAX_RETRIES) {
                    if (break_phase_ == BreakPhase::NONE) {
                        initiate_break();
                    } else {
                        // Already in BREAK and still failing
                        break_retries_++;
                        if (break_retries_ >= BREAK_MAX_RETRIES) {
                            break_exhausted();
                            finish_transfer_once(
                                false, true, false,
                                ArqTransferResultReason::RetryExhausted);
                        } else {
                            break_phase1();
                        }
                    }
                    return ArqMutationStatus::Applied;
                }

                ArqFrame frame;
                fill_data_frame(frame, slot,
                                static_cast<uint8_t>(tx_base_ % ARQ_WINDOW_SIZE));
                send_arq_frame(frame);
            }
        }

        // BREAK phase timeouts
        if (break_phase_ != BreakPhase::NONE && elapsed_ms > ack_timeout_ms_ * 3) {
            break_retries_++;
            if (break_retries_ >= BREAK_MAX_RETRIES) {
                break_exhausted();
                finish_transfer_once(false, true, false,
                                     ArqTransferResultReason::RetryExhausted);
                return ArqMutationStatus::Applied;
            } else {
                break_phase1();  // retry from phase 1
            }
        }

        // Turboshift timeout
        if (state_ == ArqState::TURBOSHIFT && elapsed_ms > ack_timeout_ms_ * 3) {
            turbo_settle();
        }
    } else if (state_ == ArqState::DISCONNECTING) {
        if (elapsed_ms > ARQ_CONNECT_TIMEOUT_MS) {
            if (!finish_transfer_once(
                    false, true, true,
                    ArqTransferResultReason::DisconnectTimeout)) {
                set_state(ArqState::IDLE);
                role_ = ArqRole::IDLE;
            }
        }
    }
    return ArqMutationStatus::Applied;
}

// --- Per-symbol soft HARQ ---

void ArqSession::clear_harq_state() {
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        harq_tx_[i] = HarqTxSlot{};
        harq_rx_[i] = HarqRxSlot{};
    }
    harq_pending_retx_ = false;
    harq_retx_desc_ = HarqRetxDescriptor{};
}

void ArqSession::harq_store_tx(uint8_t seq, const std::vector<uint8_t>& encoded_bits,
                                Modulation mod, LdpcRate fec, uint16_t payload_len) {
    int slot = seq % ARQ_WINDOW_SIZE;
    harq_tx_[slot].encoded_bits = encoded_bits;
    harq_tx_[slot].mod = mod;
    harq_tx_[slot].fec = fec;
    harq_tx_[slot].payload_len = payload_len;
    harq_tx_[slot].active = true;
}

// Select bad symbol regions from phase variance for NACK
std::vector<HarqRetxRegion> ArqSession::select_bad_regions(
    const std::vector<float>& sym_phase_var,
    const std::vector<LdpcCodec::BlockResult>& blocks,
    LdpcRate fec, int bps) {

    std::vector<HarqRetxRegion> regions;
    if (sym_phase_var.empty() || blocks.empty()) return regions;

    int k = LdpcCodec::block_size(fec);
    int n = LdpcCodec::codeword_size(fec);

    // Compute median P00
    std::vector<float> sorted_pvar = sym_phase_var;
    std::sort(sorted_pvar.begin(), sorted_pvar.end());
    float median_p00 = sorted_pvar[sorted_pvar.size() / 2];
    float threshold = std::max(median_p00 * 3.0f, 1e-4f);

    // For each failed block, find contiguous bad-symbol regions
    for (size_t b = 0; b < blocks.size(); b++) {
        if (blocks[b].converged) continue;

        // Map block b's codeword bits to symbol indices
        // Block b spans bits [b*n, (b+1)*n) in the LLR array
        // Each symbol covers `bps` bits, so block b spans symbols
        // [b*n/bps, (b+1)*n/bps)
        int sym_start = (int)(b * n) / bps;
        int sym_end = (int)((b + 1) * n) / bps;
        sym_end = std::min(sym_end, (int)sym_phase_var.size());

        // Find contiguous regions where P00 > threshold
        int region_start = -1;
        for (int s = sym_start; s <= sym_end; s++) {
            bool bad = (s < sym_end) && (s < (int)sym_phase_var.size()) &&
                       (sym_phase_var[s] > threshold);
            if (bad && region_start < 0) {
                region_start = s;
            } else if (!bad && region_start >= 0) {
                // End of bad region — convert to bit coordinates
                HarqRetxRegion r;
                r.block_index = (uint8_t)b;
                r.bit_start = (uint16_t)((region_start - sym_start) * bps);
                int bit_end = (s - sym_start) * bps;
                r.bit_count = (uint16_t)(bit_end - r.bit_start);
                // Round up to multiple of 8
                r.bit_count = ((r.bit_count + 7) / 8) * 8;
                r.bit_count = std::min((uint16_t)128, std::max((uint16_t)8, r.bit_count));
                if (r.bit_start + r.bit_count <= (uint16_t)n)
                    regions.push_back(r);
                region_start = -1;

                if (regions.size() >= 4 * blocks.size()) break;  // max 4 per block
            }
        }
        // Close trailing region
        if (region_start >= 0) {
            HarqRetxRegion r;
            r.block_index = (uint8_t)b;
            r.bit_start = (uint16_t)((region_start - sym_start) * bps);
            int bit_end = (sym_end - sym_start) * bps;
            r.bit_count = (uint16_t)(bit_end - r.bit_start);
            r.bit_count = ((r.bit_count + 7) / 8) * 8;
            r.bit_count = std::min((uint16_t)128, std::max((uint16_t)8, r.bit_count));
            if (r.bit_start + r.bit_count <= (uint16_t)n)
                regions.push_back(r);
        }
    }

    // If no clear bad regions found (uniform P00), request entire failed blocks
    if (regions.empty()) {
        for (size_t b = 0; b < blocks.size(); b++) {
            if (blocks[b].converged) continue;
            HarqRetxRegion r;
            r.block_index = (uint8_t)b;
            r.bit_start = 0;
            r.bit_count = 128;  // max per descriptor field
            regions.push_back(r);
        }
    }

    return regions;
}

void ArqSession::on_decode_failed_harq(const HarqDecodeResult& decode_result) {
    HarqDecodeResult copy = decode_result;
    submit_command([this, copy = std::move(copy)]() mutable {
        on_decode_failed_harq_impl(copy);
        return ArqMutationOutcome(ArqMutationStatus::Applied);
    });
}

void ArqSession::on_decode_failed_harq_impl(
        const HarqDecodeResult& decode_result) {
    if (role_ != ArqRole::RESPONDER) return;
    if (state_ != ArqState::CONNECTED && state_ != ArqState::TURBOSHIFT) return;
    if (rx_mute_) return;

    int seq = rx_expected_ % ARQ_WINDOW_SIZE;
    auto& slot = harq_rx_[seq];

    // Store LLRs and block results
    slot.stored_llrs = decode_result.stored_llrs;
    slot.sym_phase_var = decode_result.sym_phase_var;
    slot.num_blocks = decode_result.num_blocks;
    slot.fec = decode_result.fec;
    slot.mod = decode_result.mod;
    slot.payload_len = decode_result.payload_len;
    slot.active = true;

    // Store per-block decode status
    slot.block_decoded.resize(decode_result.blocks.size(), false);
    slot.block_data.resize(decode_result.blocks.size());
    for (size_t b = 0; b < decode_result.blocks.size(); b++) {
        if (decode_result.blocks[b].converged) {
            slot.block_decoded[b] = true;
            slot.block_data[b] = decode_result.blocks[b].data_bits;
        }
    }

    // Select bad regions
    int bps = bits_per_symbol(decode_result.mod);
    auto regions = select_bad_regions(decode_result.sym_phase_var,
                                       decode_result.blocks,
                                       decode_result.fec, bps);

    // Build extended NACK
    ArqFrame nack;
    nack.type = ArqType::NACK;
    nack.seq = (uint8_t)seq;
    nack.flags = 0x40;  // bit 6 = HARQ extended NACK

    // SNR
    int16_t snr_scaled = (int16_t)(local_snr_ * 100.0f);
    nack.payload.push_back((uint8_t)(snr_scaled & 0xFF));
    nack.payload.push_back((uint8_t)((snr_scaled >> 8) & 0xFF));

    // Block failure mask
    uint8_t block_mask = 0;
    for (size_t b = 0; b < decode_result.blocks.size() && b < 8; b++) {
        if (!decode_result.blocks[b].converged)
            block_mask |= (1 << b);
    }
    nack.payload.push_back(block_mask);

    // Per-failed-block region info
    for (auto& r : regions) {
        nack.payload.push_back(r.block_index);
        uint8_t start_hi = (r.bit_start >> 8) & 0x0F;
        uint8_t count_field = (r.bit_count / 8) - 1;
        nack.payload.push_back((start_hi << 4) | (count_field & 0x0F));
        nack.payload.push_back(r.bit_start & 0xFF);
    }

    send_arq_frame(nack);

    int n_fail = 0;
    for (auto& br : decode_result.blocks) if (!br.converged) n_fail++;
    printf("[ARQ] HARQ NACK sent: seq=%d, %d/%d blocks failed, %zu regions\n",
           seq, n_fail, (int)decode_result.blocks.size(), regions.size());
    fflush(stdout);
}

void ArqSession::handle_harq_nack(const ArqFrame& frame) {
    if (role_ != ArqRole::COMMANDER) return;

    int seq = frame.seq % ARQ_WINDOW_SIZE;
    auto& tx_slot = harq_tx_[seq];

    if (!tx_slot.active || tx_slot.encoded_bits.empty()) {
        // No stored encoded bits for this seq — fall back to full retransmit
        printf("[ARQ] HARQ NACK for seq %d but no stored encoded bits, full retransmit\n", seq);
        fflush(stdout);
        return;
    }

    // Parse extended NACK: [snr:2][block_mask:1][regions...]
    if (frame.payload.size() < 3) return;
    // uint8_t block_mask = frame.payload[2];  // which blocks failed

    // Parse regions
    std::vector<HarqRetxRegion> regions;
    size_t pos = 3;
    while (pos + 3 <= frame.payload.size()) {
        HarqRetxRegion r;
        r.block_index = frame.payload[pos++];
        uint8_t byte2 = frame.payload[pos++];
        uint8_t byte3 = frame.payload[pos++];
        r.bit_count = ((byte2 & 0x0F) + 1) * 8;
        r.bit_start = ((uint16_t)(byte2 >> 4) << 8) | byte3;
        regions.push_back(r);
    }

    // Extract the requested bits from stored encoded bits
    int n = LdpcCodec::codeword_size(tx_slot.fec);
    std::vector<uint8_t> retx_bits;
    for (auto& r : regions) {
        int block_offset = r.block_index * n;
        for (int i = 0; i < r.bit_count; i++) {
            int bit_idx = block_offset + r.bit_start + i;
            if (bit_idx < (int)tx_slot.encoded_bits.size())
                retx_bits.push_back(tx_slot.encoded_bits[bit_idx]);
            else
                retx_bits.push_back(0);
        }
    }

    // Pack bits into bytes
    std::vector<uint8_t> retx_bytes((retx_bits.size() + 7) / 8, 0);
    for (size_t i = 0; i < retx_bits.size(); i++) {
        if (retx_bits[i])
            retx_bytes[i / 8] |= (1 << (i % 8));
    }

    // Build pending retx descriptor for next TX frame
    harq_retx_desc_.original_seq = (uint8_t)seq;
    harq_retx_desc_.regions = regions;
    harq_retx_desc_.retx_bits = retx_bytes;
    harq_pending_retx_ = true;

    printf("[ARQ] HARQ retx prepared: seq=%d, %zu regions, %zu retx bits\n",
           seq, regions.size(), retx_bits.size());
    fflush(stdout);
}

} // namespace iris
