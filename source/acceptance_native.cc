// Author: xmutantson
#include "acceptance_manifest.h"
#include "arq/arq.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <set>
#include <thread>
#include <vector>

namespace {
using namespace iris;
using Bytes = std::vector<uint8_t>;
using Result = std::shared_ptr<const ArqTransferResult>;

// RC5 transport fixture: actual serialized frames, public session entry points,
// and application/terminal callbacks. Queues prevent recursive ACK delivery.
// No private state, replacement protocol, synthetic successful result or reset
// between sends. Zero reported SNR keeps the fixture at the initial speed.
struct NativePair {
    struct Endpoint {
        ArqSession session;
        std::deque<ArqFrame> air;
        std::vector<ArqFrame> sent;
        Bytes received;
        std::vector<Bytes> deliveries;
        std::vector<bool> completions;
        std::vector<Result> results;
        std::vector<size_t> terminal_bytes;
    } a, b;
    bool valid = true;

    NativePair() {
        install(a, "N0CMD");
        install(b, "N0RSP");
    }

    void install(Endpoint& ep, const char* callsign) {
        ep.session.set_callsign(callsign);
        ep.session.set_local_snr(0);
        ArqCallbacks cb;
        cb.send_frame = [this, &ep](const uint8_t* data, size_t len) {
            ArqFrame frame;
            if (!ArqFrame::deserialize(data, len, frame)) { valid = false; return; }
            ep.air.push_back(frame);
            ep.sent.push_back(frame);
        };
        cb.on_data_received = [&ep](const uint8_t* data, size_t len) {
            ep.received.insert(ep.received.end(), data, data + len);
            ep.deliveries.emplace_back(data, data + len);
        };
        cb.on_transfer_complete = [&ep](bool success) { ep.completions.push_back(success); };
        cb.on_transfer_result = [&ep](Result result) {
            ep.results.push_back(std::move(result));
            ep.terminal_bytes.push_back(ep.received.size());
        };
        ep.session.set_callbacks(cb);
    }

    bool deliver(Endpoint& to, const ArqFrame& frame) {
        const auto wire = frame.serialize();
        const bool handled = to.session.on_frame_received(wire.data(), wire.size());
        valid = valid && handled;
        // Preserve queued frames through the production role-switch mute.
        // This is channel scheduling, not a write to the session's clock/state.
        if (frame.type == ArqType::SWITCH_ROLE)
            std::this_thread::sleep_for(std::chrono::milliseconds(210));
        return handled;
    }

    bool pump() {
        for (int guard = 0; guard != 4096; ++guard) {
            if (a.air.empty() && b.air.empty()) return valid;
            if (!a.air.empty()) {
                const auto frame = a.air.front(); a.air.pop_front();
                deliver(b, frame);
            }
            if (!b.air.empty()) {
                const auto frame = b.air.front(); b.air.pop_front();
                deliver(a, frame);
            }
        }
        valid = false;
        return false;
    }

    bool connect() {
        b.session.listen();
        a.session.connect("N0RSP");
        return pump() && connected();
    }

    bool connected() const {
        return a.session.state() == ArqState::CONNECTED &&
               b.session.state() == ArqState::CONNECTED;
    }

    static void send(Endpoint& from, const Bytes& bytes) {
        from.session.send_data(bytes.data(), bytes.size());
    }

    std::vector<ArqFrame> take_data(Endpoint& from) {
        std::vector<ArqFrame> frames;
        while (!from.air.empty()) {
            auto frame = from.air.front(); from.air.pop_front();
            if (frame.type != ArqType::DATA) valid = false;
            else frames.push_back(std::move(frame));
        }
        return frames;
    }

    static bool last_ack(const Endpoint& from, ArqFrame& ack) {
        for (auto it = from.sent.rbegin(); it != from.sent.rend(); ++it) {
            if (it->type == ArqType::ACK) { ack = *it; return true; }
        }
        return false;
    }
};

Bytes pattern(size_t size, unsigned salt = 0) {
    Bytes out(size);
    for (size_t i = 0; i < size; ++i)
        out[i] = static_cast<uint8_t>((i * 73 + (i >> 3) * 29 + salt * 41) & 255);
    return out;
}

Bytes joined(const std::vector<Bytes>& records) {
    Bytes out;
    for (const auto& record : records) out.insert(out.end(), record.begin(), record.end());
    return out;
}

bool successful(const NativePair::Endpoint& ep, size_t count) {
    if (ep.results.size() != count || ep.completions.size() != count) return false;
    std::set<uint64_t> ids;
    for (size_t i = 0; i < count; ++i) {
        const auto& result = ep.results[i];
        if (!ep.completions[i] || !result || !result->success ||
            result->reason != ArqTransferResultReason::Completed ||
            result->transfer_id == 0 || !ids.insert(result->transfer_id).second ||
            !result->preserved_records.empty()) return false;
    }
    return true;
}

bool exact_transfers(const NativePair::Endpoint& tx, const NativePair::Endpoint& rx,
                     const std::vector<Bytes>& records) {
    if (!successful(tx, records.size()) || !successful(rx, records.size()) ||
        tx.session.pending_frames() != 0 || rx.received != joined(records)) return false;
    size_t boundary = 0;
    for (size_t i = 0; i < records.size(); ++i) {
        boundary += records[i].size();
        // Receiver custody may retain fragments in arrival order. Its stream
        // and observed terminal byte boundary are the delivery contract here.
        if (tx.results[i]->accepted_records != std::vector<Bytes>{records[i]} ||
            rx.terminal_bytes[i] != boundary) return false;
    }
    return true;
}

void acc_rc5_two_small_sends() {
    NativePair h;
    const bool ready = h.connect();
    const std::vector<Bytes> records{{'a', 'b', 'c'}, {'D', 'E', 'F'}};
    NativePair::send(h.a, records[0]);
    const bool first = h.pump() && exact_transfers(h.a, h.b, {records[0]});
    NativePair::send(h.a, records[1]);
    const bool drained = h.pump();
    const bool exact = exact_transfers(h.a, h.b, records);
    std::printf("ACCEPTANCE_OBS name=acc_rc5_two_small_sends ready=%d first_exact=%d expected=6 received=%zu tx_results=%zu rx_results=%zu distinct_exact_transfers=%d\n",
        ready, first, h.b.received.size(), h.a.results.size(), h.b.results.size(), exact);
    acceptance_manifest_record("acc_rc5_two_small_sends", ready && first && drained &&
        h.connected() && exact && h.b.deliveries == records);
}

void acc_rc5_duplicate_completed_ack() {
    NativePair h;
    const bool ready = h.connect();
    const auto plain = pattern(3);
    NativePair::send(h.a, plain);
    const bool control = h.pump() && exact_transfers(h.a, h.b, {plain});
    ArqFrame ack{};
    const bool captured = NativePair::last_ack(h.b, ack);
    const auto before = h.a.results;
    if (captured) for (int i = 0; i != 3; ++i) h.deliver(h.a, ack);
    const bool drained = h.pump();
    const bool unchanged = h.a.results == before && exact_transfers(h.a, h.b, {plain});
    std::printf("ACCEPTANCE_OBS name=acc_rc5_duplicate_completed_ack ready=%d control=%d captured_ack=%d replays=3 results_before=%zu results_after=%zu unchanged=%d\n",
        ready, control, captured, before.size(), h.a.results.size(), unchanged);
    acceptance_manifest_record("acc_rc5_duplicate_completed_ack",
        ready && control && captured && drained && unchanged);
}

void acc_rc5_delayed_ack_pending_batch() {
    bool all = true;
    // Immediate later work, and two complete modulo-8 sequence wraps. Replay
    // the actual first completion ACK while the new DATA is held off the wire.
    for (int prior : {2, 16}) {
        NativePair h;
        const bool ready = h.connect();
        std::vector<Bytes> records;
        ArqFrame old_ack{};
        bool control = ready, captured = false;
        for (int i = 0; i < prior; ++i) {
            records.push_back(pattern(3, i));
            NativePair::send(h.a, records.back());
            control = h.pump() && exact_transfers(h.a, h.b, records) && control;
            if (i == 0) captured = NativePair::last_ack(h.b, old_ack);
        }
        const auto before = h.a.results;
        const auto delivered_before = h.b.received;
        records.push_back(pattern(3, prior));
        NativePair::send(h.a, records.back());
        const int pending_before = h.a.session.pending_frames();
        if (captured) h.deliver(h.a, old_ack);
        const bool no_completion = h.a.results == before &&
            h.a.completions.size() == before.size();
        const bool retained = h.a.session.pending_frames() == pending_before;
        const bool held = h.b.received == delivered_before && !h.a.air.empty();
        const bool drained = h.pump();
        const bool eventual = exact_transfers(h.a, h.b, records);
        std::printf("ACCEPTANCE_OBS name=acc_rc5_delayed_ack_pending_batch prior_batches=%d ready=%d control=%d captured_ack=%d held_data=%d pending_before=%d pending_retained=%d no_extra_completion=%d eventual_exact=%d\n",
            prior, ready, control, captured, held, pending_before, retained, no_completion, eventual);
        all = all && ready && control && captured && held && pending_before > 0 &&
              retained && no_completion && drained && eventual;
    }
    acceptance_manifest_record("acc_rc5_delayed_ack_pending_batch", all);
}

void acc_rc5_single_send_3000() {
    NativePair h;
    const bool ready = h.connect();
    const auto plain = pattern(3000);
    NativePair::send(h.a, plain);
    const bool drained = h.pump();
    const bool exact = exact_transfers(h.a, h.b, {plain});
    std::printf("ACCEPTANCE_OBS name=acc_rc5_single_send_3000 ready=%d expected=3000 received=%zu fragment_callbacks=%zu tx_results=%zu rx_results=%zu exact=%d\n",
        ready, h.b.received.size(), h.b.deliveries.size(), h.a.results.size(), h.b.results.size(), exact);
    acceptance_manifest_record("acc_rc5_single_send_3000", ready && drained && h.connected() && exact);
}

void acc_rc5_small_sends_sequence_wraps() {
    NativePair h;
    const bool ready = h.connect();
    std::vector<Bytes> records;
    ArqFrame first_data{};
    bool exact = true, captured = false, duplicates_ignored = true;
    int wraps = 0, previous_seq = -1;
    for (int i = 0; i != 40; ++i) {
        records.push_back(pattern(1 + i % 7, i));
        NativePair::send(h.a, records.back());
        if (h.a.air.size() != 1 || h.a.air.front().type != ArqType::DATA) exact = false;
        else {
            const auto& frame = h.a.air.front();
            if (previous_seq >= 0 && frame.seq < previous_seq) ++wraps;
            previous_seq = frame.seq;
            if (i == 0) { first_data = frame; captured = true; }
        }
        exact = h.pump() && exact_transfers(h.a, h.b, records) && exact;
        if (i >= 8 && captured) {
            const auto before = h.b.results;
            const size_t callbacks = h.b.deliveries.size();
            h.deliver(h.b, first_data);
            duplicates_ignored = h.pump() && h.b.results == before &&
                h.b.deliveries.size() == callbacks && exact_transfers(h.a, h.b, records) &&
                duplicates_ignored;
        }
    }
    std::printf("ACCEPTANCE_OBS name=acc_rc5_small_sends_sequence_wraps ready=%d sends=40 observed_wraps=%d expected=%zu received=%zu tx_results=%zu rx_results=%zu exact=%d old_data_ignored=%d\n",
        ready, wraps, joined(records).size(), h.b.received.size(), h.a.results.size(),
        h.b.results.size(), exact, duplicates_ignored);
    acceptance_manifest_record("acc_rc5_small_sends_sequence_wraps", ready && h.valid &&
        h.connected() && captured && wraps >= 4 && exact && duplicates_ignored);
}

void acc_rc5_lost_final_retry_eob() {
    bool all = true;
    for (bool timeout : {false, true}) {
        NativePair h;
        const bool ready = h.connect();
        const auto plain = pattern(301);
        NativePair::send(h.a, plain);
        const auto frames = h.take_data(h.a);
        const bool fixture = frames.size() >= 2 && (frames.back().flags & 0x80);
        bool prefix = false, retried = false, metadata = false, exact = false;
        if (fixture) {
            for (size_t i = 0; i + 1 < frames.size(); ++i) h.deliver(h.b, frames[i]);
            h.pump();
            prefix = !h.b.received.empty() && h.b.received.size() < plain.size() &&
                std::equal(h.b.received.begin(), h.b.received.end(), plain.begin()) &&
                h.a.results.empty() && h.b.results.empty();
            if (timeout) {
                // Production native timeout is 2.9 s in CONNECTED state.
                std::this_thread::sleep_for(std::chrono::milliseconds(3100));
                h.a.session.tick();
            } else {
                h.b.session.on_decode_failed();
                while (!h.b.air.empty()) {
                    const auto nack = h.b.air.front(); h.b.air.pop_front();
                    h.deliver(h.a, nack);
                }
            }
            const auto retries = h.take_data(h.a);
            retried = retries.size() == 1 && h.a.session.retransmit_count() > 0;
            metadata = retried && retries[0].serialize() == frames.back().serialize();
            for (const auto& frame : retries) h.deliver(h.b, frame);
            exact = h.pump() && exact_transfers(h.a, h.b, {plain});
        }
        std::printf("ACCEPTANCE_OBS name=acc_rc5_lost_final_retry_eob retry=%s ready=%d fixture_valid=%d prefix_without_completion=%d retried=%d immutable_frame_eob=%d received=%zu exact=%d\n",
            timeout ? "timeout" : "nack", ready, fixture, prefix, retried, metadata, h.b.received.size(), exact);
        all = all && ready && h.valid && fixture && prefix && retried && metadata && exact;
    }
    acceptance_manifest_record("acc_rc5_lost_final_retry_eob", all);
}

void acc_rc5_reordered_eob_duplicate_data() {
    NativePair h;
    const bool ready = h.connect();
    const auto plain = pattern(301);
    NativePair::send(h.a, plain);
    const auto frames = h.take_data(h.a);
    const bool fixture = frames.size() >= 3 && (frames.back().flags & 0x80);
    bool held = false, gap = false, exact = false, duplicate = false;
    if (fixture) {
        h.deliver(h.b, frames.back());
        h.deliver(h.b, frames.back()); // duplicate buffered EOB
        held = h.pump() && h.b.received.empty() && h.b.results.empty() && h.a.results.empty();
        h.deliver(h.b, frames.front());
        const auto prefix = h.b.received;
        h.deliver(h.b, frames.front()); // duplicate delivered DATA
        gap = h.pump() && !prefix.empty() && h.b.received == prefix &&
            h.b.results.empty() && h.a.results.empty();
        for (size_t i = 1; i + 1 < frames.size(); ++i) h.deliver(h.b, frames[i]);
        exact = h.pump() && exact_transfers(h.a, h.b, {plain});
        const auto results = h.b.results;
        const auto callbacks = h.b.deliveries;
        for (const auto& frame : frames) h.deliver(h.b, frame);
        duplicate = h.pump() && h.b.results == results && h.b.deliveries == callbacks &&
            exact_transfers(h.a, h.b, {plain});
    }
    std::printf("ACCEPTANCE_OBS name=acc_rc5_reordered_eob_duplicate_data ready=%d fixture_valid=%d eob_held=%d gap_without_completion=%d received=%zu exact=%d replay_unchanged=%d\n",
        ready, fixture, held, gap, h.b.received.size(), exact, duplicate);
    acceptance_manifest_record("acc_rc5_reordered_eob_duplicate_data",
        ready && h.valid && fixture && held && gap && exact && duplicate);
}

void acc_rc5_buffered_next_batch_boundary() {
    NativePair h;
    const bool ready = h.connect();
    const auto first = pattern(3), second = pattern(301, 1);
    NativePair::send(h.a, first);
    const auto batch1 = h.take_data(h.a);
    NativePair::send(h.a, second);
    const auto batch2 = h.take_data(h.a);
    // Two *emitted* EOBs establish the boundaries; do not infer one from an
    // empty RX queue or from the number of application calls alone.
    const bool fixture = batch1.size() == 1 && (batch1[0].flags & 0x80) &&
        batch2.size() >= 2 && (batch2.back().flags & 0x80);
    bool first_boundary = false, two_boundaries = false, exact = false;
    if (fixture) {
        h.deliver(h.b, batch2.back()); // future batch suffix, gap still open
        h.deliver(h.b, batch1[0]);
        first_boundary = h.b.received == first && successful(h.b, 1) &&
            h.b.terminal_bytes[0] == first.size() &&
            joined(h.b.results[0]->accepted_records) == first;
        for (size_t i = 0; i + 1 < batch2.size(); ++i) h.deliver(h.b, batch2[i]);
        const bool drained = h.pump();
        exact = drained && h.b.received == joined({first, second});
        two_boundaries = exact_transfers(h.a, h.b, {first, second});
    }
    std::printf("ACCEPTANCE_OBS name=acc_rc5_buffered_next_batch_boundary ready=%d two_emitted_eobs=%d first_boundary_completed=%d received=%zu exact_bytes=%d tx_results=%zu rx_results=%zu two_distinct_boundaries=%d\n",
        ready, fixture, first_boundary, h.b.received.size(), exact,
        h.a.results.size(), h.b.results.size(), two_boundaries);
    acceptance_manifest_record("acc_rc5_buffered_next_batch_boundary",
        ready && h.valid && fixture && first_boundary && exact && two_boundaries);
}

void acc_rc5_empty_send() {
    NativePair h;
    const bool ready = h.connect();
    // Existing send_data(nullptr, 0) is a no-op, not an explicit finish API.
    h.a.session.send_data(nullptr, 0);
    const bool empty_idle = h.a.air.empty() && h.a.results.empty() && h.b.results.empty();
    const auto plain = pattern(3);
    NativePair::send(h.a, plain);
    const int pending = h.a.session.pending_frames();
    h.a.session.send_data(nullptr, 0);
    const bool empty_pending = h.a.session.pending_frames() == pending && h.a.results.empty();
    const bool control = h.pump() && exact_transfers(h.a, h.b, {plain});
    const auto before = h.a.results;
    h.a.session.send_data(nullptr, 0);
    const bool empty_done = h.pump() && h.a.results == before && exact_transfers(h.a, h.b, {plain});
    std::printf("ACCEPTANCE_OBS name=acc_rc5_empty_send ready=%d idle_noop=%d pending_noop=%d control=%d completed_noop=%d\n",
        ready, empty_idle, empty_pending, control, empty_done);
    acceptance_manifest_record("acc_rc5_empty_send",
        ready && empty_idle && empty_pending && control && empty_done);
}

void acc_rc5_reconnect() {
    NativePair h;
    const bool ready = h.connect();
    const std::vector<Bytes> records{pattern(3), pattern(3, 1)};
    NativePair::send(h.a, records[0]);
    const bool first = h.pump() && exact_transfers(h.a, h.b, {records[0]});
    h.a.session.disconnect();
    const bool closed = h.pump() && h.a.session.state() == ArqState::IDLE &&
        h.b.session.state() == ArqState::IDLE && exact_transfers(h.a, h.b, {records[0]});
    const bool reconnected = h.connect();
    NativePair::send(h.a, records[1]);
    const bool exact = h.pump() && exact_transfers(h.a, h.b, records);
    std::printf("ACCEPTANCE_OBS name=acc_rc5_reconnect ready=%d first_exact=%d closed_without_extra_terminal=%d reconnected=%d received=%zu fresh_distinct_transfers=%d\n",
        ready, first, closed, reconnected, h.b.received.size(), exact);
    acceptance_manifest_record("acc_rc5_reconnect", ready && first && closed && reconnected && exact);
}

void acc_rc5_role_switch_round_trip() {
    NativePair h;
    const bool ready = h.connect();
    const auto first = pattern(3), reverse = pattern(3, 1), last = pattern(3, 2);
    NativePair::send(h.a, first);
    const bool forward = h.pump() && exact_transfers(h.a, h.b, {first});
    NativePair::send(h.b, reverse); // real automatic role request + peer response
    const bool reverse_drained = h.pump();
    const bool reverse_exact = h.a.received == reverse && h.b.received == first &&
        successful(h.a, 2) && successful(h.b, 2) && h.b.session.pending_frames() == 0 &&
        h.a.session.role() == ArqRole::RESPONDER && h.b.session.role() == ArqRole::COMMANDER;
    NativePair::send(h.a, last);
    const bool final_drained = h.pump();
    const bool final_exact = h.a.received == reverse && h.b.received == joined({first, last}) &&
        successful(h.a, 3) && successful(h.b, 3) && h.a.session.pending_frames() == 0 &&
        h.a.session.role() == ArqRole::COMMANDER && h.b.session.role() == ArqRole::RESPONDER;
    std::printf("ACCEPTANCE_OBS name=acc_rc5_role_switch_round_trip ready=%d forward_exact=%d reverse_exact=%d return_exact=%d a_received=%zu b_received=%zu a_results=%zu b_results=%zu\n",
        ready, forward, reverse_exact, final_exact, h.a.received.size(), h.b.received.size(),
        h.a.results.size(), h.b.results.size());
    acceptance_manifest_record("acc_rc5_role_switch_round_trip", ready && h.valid && forward &&
        reverse_drained && reverse_exact && final_drained && final_exact && h.connected());
}

void acc_rc5_role_switch_pending_custody() {
    NativePair h;
    const bool ready = h.connect();
    const auto unresolved = pattern(301), reverse = pattern(3, 1);
    NativePair::send(h.a, unresolved);
    const auto held = h.take_data(h.a); // no peer DATA acceptance or ACK
    const bool pending = !held.empty() && h.a.session.pending_frames() > 0 &&
        h.a.results.empty() && h.b.received.empty();
    NativePair::send(h.b, reverse); // peer requests a real direction change
    const bool request = h.b.air.size() == 1 &&
        h.b.air.front().type == ArqType::SWITCH_ROLE;
    bool failed_once = false, preserved = false;
    if (request) {
        const auto frame = h.b.air.front(); h.b.air.pop_front();
        h.deliver(h.a, frame);
        // RC5 requires unresolved work to reach an explicit terminal decision
        // before role cleanup. Original bytes must remain recoverable there.
        failed_once = h.a.results.size() == 1 && h.a.completions.size() == 1 &&
            !h.a.completions[0] && !h.a.results[0]->success &&
            h.a.results[0]->transfer_id != 0;
        preserved = failed_once &&
            h.a.results[0]->accepted_records == std::vector<Bytes>{unresolved} &&
            h.a.results[0]->preserved_records == std::vector<Bytes>{unresolved};
    }
    std::printf("ACCEPTANCE_OBS name=acc_rc5_role_switch_pending_custody ready=%d unacked_custody=%d peer_role_request=%d explicit_failure_once=%d original_recoverable=%d results=%zu received=%zu\n",
        ready, pending, request, failed_once, preserved, h.a.results.size(), h.b.received.size());
    acceptance_manifest_record("acc_rc5_role_switch_pending_custody",
        ready && h.valid && pending && request && failed_once && preserved);
}
} // namespace

void run_acceptance_native() {
    std::printf("\n--- RC5 Native Batch Acceptance Tests ---\n");
    acc_rc5_two_small_sends();
    acc_rc5_duplicate_completed_ack();
    acc_rc5_delayed_ack_pending_batch();
    acc_rc5_single_send_3000();
    acc_rc5_small_sends_sequence_wraps();
    acc_rc5_lost_final_retry_eob();
    acc_rc5_reordered_eob_duplicate_data();
    acc_rc5_buffered_next_batch_boundary();
    acc_rc5_empty_send();
    acc_rc5_reconnect();
    acc_rc5_role_switch_round_trip();
    acc_rc5_role_switch_pending_custody();
}
