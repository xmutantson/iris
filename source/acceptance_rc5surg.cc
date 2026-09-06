// Author: xmutantson
#include "acceptance_manifest.h"
#include "acceptance_rc5surg.h"
#include "arq/arq.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <functional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace iris;
using Bytes = std::vector<uint8_t>;
using Result = std::shared_ptr<const ArqTransferResult>;

// Serialized public-path fixture. Hooks issue real callback requests; they do
// not emulate a dispatcher, a hold ledger, ACK authority, or terminal results.
// Every scenario bounds routing and keeps callback arguments alive by value.
struct SurgicalPair {
    struct End {
        ArqSession session;
        ArqCallbacks callbacks;
        std::deque<ArqFrame> air;
        std::vector<ArqFrame> wire;
        std::vector<Bytes> deliveries;
        std::vector<Result> results;
        std::vector<bool> completions;
        std::function<void(const ArqFrame&)> frame_hook;
        std::function<bool(bool, ArqRecordType)> delivery_hook;
        std::function<void(bool)> complete_hook;
        std::function<void(Result)> result_hook;
    } a, b;
    bool valid = true;
    int routing = 0;

    SurgicalPair() { install(a, "N0CMD"); install(b, "N0RSP"); }
    void install(End& e, const char* name) {
        e.session.set_callsign(name);
        e.session.set_local_snr(0);
        e.callbacks.send_frame = [this, &e](const uint8_t* p, size_t n) {
            ArqFrame f{};
            if (!ArqFrame::deserialize(p, n, f)) { valid = false; return; }
            e.wire.push_back(f);
            if (e.frame_hook) e.frame_hook(f);
            else e.air.push_back(f);
        };
        e.callbacks.on_typed_data_fragment = [&e](const uint8_t* p, size_t n,
                bool end, ArqRecordType type, uint64_t) {
            e.deliveries.emplace_back(p, p + n);
            return !e.delivery_hook || e.delivery_hook(end, type);
        };
        e.callbacks.on_transfer_complete = [&e](bool ok) {
            e.completions.push_back(ok);
            if (e.complete_hook) e.complete_hook(ok);
        };
        e.callbacks.on_transfer_result = [&e](Result r) {
            e.results.push_back(r);
            if (e.result_hook) e.result_hook(r);
        };
        e.session.set_callbacks(e.callbacks);
    }
    bool deliver(End& e, const ArqFrame& f) {
        if (++routing > 8192) { valid = false; return false; }
        const auto bytes = f.serialize();
        bool ok = e.session.on_frame_received(bytes.data(), bytes.size());
        valid = valid && ok;
        if (f.type == ArqType::SWITCH_ROLE)
            std::this_thread::sleep_for(std::chrono::milliseconds(210));
        return ok;
    }
    bool pump() {
        for (int i = 0; i < 4096; ++i) {
            if (a.air.empty() && b.air.empty()) return valid;
            if (!a.air.empty()) { auto f = a.air.front(); a.air.pop_front(); deliver(b, f); }
            if (!b.air.empty()) { auto f = b.air.front(); b.air.pop_front(); deliver(a, f); }
        }
        return valid = false;
    }
    bool connect() {
        b.session.listen(); a.session.connect("N0RSP");
        return pump() && a.session.state() == ArqState::CONNECTED &&
            b.session.state() == ArqState::CONNECTED;
    }
    static void send(End& e, const Bytes& bytes) {
        e.session.send_data(bytes.data(), bytes.size());
    }
    std::vector<ArqFrame> take(End& e) {
        std::vector<ArqFrame> out(e.air.begin(), e.air.end()); e.air.clear(); return out;
    }
};

Bytes bytes(size_t n, uint8_t tag) { return Bytes(n, tag); }
Bytes joined(const std::vector<Bytes>& parts) {
    Bytes out;
    for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}
size_t frames(const SurgicalPair::End& e, ArqType type, size_t from = 0) {
    return std::count_if(e.wire.begin() + from, e.wire.end(),
        [type](const ArqFrame& f) { return f.type == type; });
}
bool exact(const SurgicalPair::End& e, const std::vector<Bytes>& originals) {
    if (e.results.size() != originals.size() || e.completions.size() != originals.size()) return false;
    std::set<uint64_t> ids;
    for (size_t i = 0; i < originals.size(); ++i) {
        const auto& r = e.results[i];
        if (!r || !r->success || !e.completions[i] || !r->transfer_id ||
            !ids.insert(r->transfer_id).second || !r->preserved_records.empty() ||
            joined(r->accepted_records) != originals[i]) return false;
    }
    return true;
}
// Each predicate is evaluated independently, so one failure cannot skip the
// remainder of the scenario or hide its observations. Missing setup fails too.
struct Check {
    const char* name;
    bool ok = true;
    void require(const char* observation, bool value) {
        std::printf("ACCEPTANCE_OBS name=%s predicate=%s value=%d\n", name, observation, value);
        ok = value && ok;
    }
    void count(const char* observation, size_t value) {
        std::printf("ACCEPTANCE_OBS name=%s count=%s value=%zu\n", name, observation, value);
    }
    void finish() { acceptance_manifest_record(name, ok); }
};

void failure_replacement() {
    Check c{"acc_rc5surg_failure_replacement"};
    for (bool reconnect : {false, true}) {
        SurgicalPair h;
        c.require("connected", h.connect());
        const auto original = bytes(3000, 0x31), replacement = bytes(3, 0x72);
        SurgicalPair::send(h.a, original);
        auto old = h.take(h.a);
        c.require("window_and_queue_populated", old.size() == ARQ_WINDOW_SIZE &&
            h.a.session.tx_queue_bytes() > 0 && h.a.session.pending_frames() > ARQ_WINDOW_SIZE);
        // Capture actual peer feedback for this flight, rather than inventing
        // a cumulative ACK that a future identified wire format might reject.
        if (!old.empty()) h.deliver(h.b, old[0]);
        h.b.session.on_decode_failed();
        const auto feedback = h.take(h.b);
        c.require("peer_ack_and_nack_captured", frames(h.b, ArqType::ACK) > 0 &&
            frames(h.b, ArqType::NACK) > 0);
        // Populate production HARQ storage for an actually emitted slot.
        if (!old.empty()) h.a.session.harq_store_tx(old[0].seq, bytes(64, 1),
            Modulation::BPSK, LdpcRate::NONE, 3);
        bool entered = false;
        size_t after_failure = 0;
        h.a.complete_hook = [&](bool success) {
            if (success || entered) return;
            entered = true;
            c.require("callback_sees_terminal_state", h.a.session.state() == ArqState::DISCONNECTING);
            c.require("callback_sees_retired_schedule", h.a.session.pending_frames() == 0 &&
                h.a.session.tx_queue_bytes() == 0 && !h.a.session.harq_has_pending_retx());
            c.require("all_window_queue_retry_authority_retired", acceptance_rc5surg_tx_retired(h.a.session));
            after_failure = h.a.wire.size();
            if (reconnect) h.a.session.connect("N0NEW");
            SurgicalPair::send(h.a, replacement);
            // Late feedback also enters during publication, when old custody
            // is already final. A NACK must not revive the abandoned flight.
            for (const auto& f : feedback) if (f.type == ArqType::NACK) h.deliver(h.a, f);
        };
        c.require("failure_applied", h.a.session.fail_active_transfer(ArqTransferResultReason::TransformFailure));
        c.require("failure_callback_once", entered && h.a.completions.size() >= 1);
        c.require("original_preserved_once", h.a.results.size() >= 1 &&
            !h.a.results[0]->success && h.a.results[0]->accepted_records == std::vector<Bytes>{original} &&
            h.a.results[0]->preserved_records == std::vector<Bytes>{original});
        for (const auto& f : feedback) h.deliver(h.a, f);
        // Exercise a genuinely expired retry deadline after terminal failure.
        if (!reconnect) std::this_thread::sleep_for(std::chrono::milliseconds(3100));
        h.a.session.tick();
        bool no_old = true;
        for (size_t i = after_failure; i < h.a.wire.size(); ++i) {
            const auto& f = h.a.wire[i];
            // Every original fragment carries the tag, including fragments
            // queued beyond the initial window that have never been emitted.
            if (f.type == ArqType::DATA && f.payload.size() > ARQ_RECORD_IDENTITY_BYTES &&
                std::all_of(f.payload.begin() + ARQ_RECORD_IDENTITY_BYTES, f.payload.end(),
                    [&](uint8_t v) { return v == original[0]; })) no_old = false;
        }
        c.require("no_retired_data_after_feedback", no_old);
        bool rejected = false;
        for (const auto& r : h.a.results)
            rejected = rejected || (!r->success && r->preserved_records == std::vector<Bytes>{replacement} &&
                r->transfer_id != h.a.results.front()->transfer_id);
        c.require("replacement_admitted_or_explicitly_rejected", rejected || (reconnect &&
            h.a.session.state() == ArqState::HAILING && h.a.session.remote_callsign() == "N0NEW"));
        h.a.session.reset();
        size_t original_count = 0, replacement_count = 0;
        std::set<uint64_t> ids;
        for (const auto& r : h.a.results) {
            c.require("unique_terminal_identity", ids.insert(r->transfer_id).second);
            original_count += std::count(r->preserved_records.begin(), r->preserved_records.end(), original);
            replacement_count += std::count(r->preserved_records.begin(), r->preserved_records.end(), replacement);
        }
        c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
            h.b.session.structure_violations() == 0);
        c.require("all_originals_recoverable_once", original_count == 1 && replacement_count == 1);
        c.require("valid_wire", h.valid);
    }
    c.finish();
}

void mlkem_reply() {
    Check c{"acc_rc5surg_rx_mlkem_reply"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto key = bytes(3, 0x19), reply = bytes(3, 0x63);
    h.b.delivery_hook = [&](bool end, ArqRecordType type) {
        c.require("public_key_decision", end && type == ArqRecordType::MlKemPublicKey);
        // Local vector lifetime ends on return, as in the ciphertext producer.
        auto owned = reply;
        h.b.session.send_prepared(owned.data(), owned.size(), ArqRecordType::MlKemCiphertext, 0);
        return true;
    };
    h.a.session.send_prepared(key.data(), key.size(), ArqRecordType::MlKemPublicKey, 0);
    auto forward = h.take(h.a);
    c.require("one_forward_eob", forward.size() == 1 && (forward[0].flags & 0x80));
    const size_t start = h.b.wire.size();
    for (const auto& f : forward) h.deliver(h.b, f);
    c.require("rx_final_before_reverse_ack", exact(h.b, {key}));
    size_t ack = h.b.wire.size(), role = h.b.wire.size();
    for (size_t i = start; i < h.b.wire.size(); ++i) {
        if (h.b.wire[i].type == ArqType::ACK) ack = std::min(ack, i);
        if (h.b.wire[i].type == ArqType::SWITCH_ROLE) role = std::min(role, i);
    }
    c.require("rx_ack_precedes_reply_role_request", ack < role && role < h.b.wire.size());
    const auto before = h.b.results;
    for (const auto& f : forward) h.deliver(h.b, f);
    c.require("below_cursor_no_duplicate", h.b.deliveries == std::vector<Bytes>{key} && h.b.results == before);
    // Hold only the reverse DATA ACK; control frames still use the real pair.
    std::vector<ArqFrame> reverse_acks;
    h.a.frame_hook = [&](const ArqFrame& f) {
        if (f.type == ArqType::ACK) reverse_acks.push_back(f);
        else h.a.air.push_back(f);
    };
    c.require("reply_delivered", h.pump() && joined(h.a.deliveries) == reply);
    c.require("reverse_waits_for_own_ack", h.b.results.size() == 1 && !reverse_acks.empty() &&
        h.b.session.pending_frames() > 0);
    for (const auto& f : reverse_acks) h.deliver(h.b, f);
    c.require("reverse_has_distinct_result", h.b.results.size() == 2 && h.b.completions.size() == 2 &&
        h.b.results[0]->success && h.b.results[1]->success &&
        h.b.results[0]->transfer_id != h.b.results[1]->transfer_id &&
        h.b.results[1]->accepted_records.empty() && h.b.session.pending_frames() == 0);
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("wire_valid", h.valid); c.finish();
}

void acceptance_renewal() {
    Check c{"acc_rc5surg_rx_acceptance_renewal"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto record = bytes(301, 0x27);
    h.b.session.defer_receive_commit();
    bool renewed = false, premature = false;
    h.b.delivery_hook = [&](bool end, ArqRecordType) {
        if (end) {
            const auto count = frames(h.b, ArqType::ACK);
            // Release the old policy, then acquire the new acceptance hold.
            // The decision has not returned: no ACK may include this fragment.
            h.b.session.commit_deferred_receive();
            premature = frames(h.b, ArqType::ACK) != count;
            h.b.session.defer_receive_commit();
            renewed = true;
        }
        return true;
    };
    SurgicalPair::send(h.a, record);
    const auto data = h.take(h.a);
    c.require("fragmented_eob", data.size() == 3 && (data.back().flags & 0x80));
    for (const auto& f : data) h.deliver(h.b, f);
    c.require("renewal_did_not_ack_pending_decision", renewed && !premature);
    c.require("held_without_terminal", h.b.results.empty() && h.a.results.empty());

    // Public entries drain queued commands and publications before returning.
    // Tick outside the acceptance callback, then route every reverse frame via
    // pump()/on_frame_received(): a queued whole-record ACK must reach the
    // sender before we inspect it. A frontier ACK is allowed to retire only
    // earlier fragments, leaving this record pending until its own release.
    const auto drain = [&]() {
        for (int i = 0; i < 4096; ++i) {
            h.b.session.tick();
            h.a.session.tick();
            if (h.a.air.empty() && h.b.air.empty()) return h.valid;
            if (!h.pump()) return false;
        }
        return false;
    };
    c.require("renewal_drained_and_reverse_frames_routed", drain());
    c.count("receive_acks_before_new_release", frames(h.b, ArqType::ACK));
    c.count("sender_completions_before_new_release", h.a.completions.size());
    c.count("sender_results_before_new_release", h.a.results.size());
    c.count("sender_pending_frames_before_new_release", h.a.session.pending_frames());
    c.require("sender_not_completed_after_renewal_ack", h.a.completions.empty());
    c.require("sender_not_finalized_after_renewal_ack", h.a.results.empty() &&
        !h.a.session.last_transfer_result() && h.a.session.retained_transfer_results().empty());
    c.require("sender_still_pending_after_renewal_ack", h.a.session.pending_frames() > 0);
    c.require("renewed_receiver_has_no_completion", h.b.completions.empty());
    c.require("renewed_receiver_has_no_terminal", h.b.results.empty() &&
        !h.b.session.last_transfer_result() && h.b.session.retained_transfer_results().empty());

    const auto delivered = h.b.deliveries;
    const auto ack_count = frames(h.b, ArqType::ACK);
    for (const auto& f : data) h.deliver(h.b, f);
    c.require("held_replay_drained_and_routed", drain());
    c.require("held_replay_inert", h.b.deliveries == delivered && frames(h.b, ArqType::ACK) == ack_count);
    c.require("held_replay_no_duplicate_ack", frames(h.b, ArqType::ACK) == ack_count);
    c.require("held_replay_no_receiver_terminal", h.b.completions.empty() && h.b.results.empty());
    c.require("held_replay_no_sender_terminal", h.a.completions.empty() && h.a.results.empty());
    // No public hold accessor is needed: custody must remain unpublished above,
    // and this later release must apply and publish the retained record below.
    c.require("renewed_hold_own_release_applied",
        h.b.session.commit_deferred_receive().status() == ArqMutationStatus::Applied);
    c.require("new_release_drained_and_routed", drain());
    c.require("new_release_completes_once", h.pump() && exact(h.b, {record}) && exact(h.a, {record}));
    c.require("new_release_receiver_completes_once", exact(h.b, {record}));
    c.require("new_release_sender_completes_once", exact(h.a, {record}));
    c.require("new_release_emits_one_ack", frames(h.b, ArqType::ACK) == ack_count + 1);
    c.require("new_release_retires_sender_frames", h.a.session.pending_frames() == 0);
    const auto released_ack_count = frames(h.b, ArqType::ACK);
    h.b.session.commit_deferred_receive();
    c.require("duplicate_release_drained_and_routed", drain());
    c.require("duplicate_release_no_terminal", exact(h.b, {record}) && exact(h.a, {record}));
    c.require("duplicate_release_no_ack", frames(h.b, ArqType::ACK) == released_ack_count);
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("delivery_once", joined(h.b.deliveries) == record);
    c.require("valid_wire", h.valid);
    c.finish();
}

void two_rx_batches() {
    Check c{"acc_rc5surg_two_rx_batches"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto first = bytes(3, 0x41), second = bytes(3, 0x42);
    h.b.session.defer_receive_commit();
    SurgicalPair::send(h.a, first); const auto a = h.take(h.a);
    SurgicalPair::send(h.a, second); const auto b = h.take(h.a);
    c.require("two_emitted_eobs", a.size() == 1 && b.size() == 1 &&
        (a[0].flags & 0x80) && (b[0].flags & 0x80));
    for (const auto& f : a) h.deliver(h.b, f);
    for (const auto& f : b) h.deliver(h.b, f);
    c.require("both_initially_held", h.b.results.empty() && frames(h.b, ArqType::ACK) == 0);
    bool nested = false;
    h.b.complete_hook = [&](bool ok) {
        if (!ok || nested) return;
        nested = true;
        // Both requests originate during the old release's publication. The
        // recursive release must not acquire authority from a future defer.
        h.b.session.defer_receive_commit();
        h.b.session.commit_deferred_receive();
    };
    h.b.session.commit_deferred_receive();
    c.require("only_a_published", nested && exact(h.b, {first}));
    bool crosses_b = false;
    for (const auto& f : h.b.wire) if (f.type == ArqType::ACK)
        crosses_b = crosses_b || f.seq == 2 || f.flags != 0;
    c.require("no_ack_crosses_b", !crosses_b);
    const auto delivered = h.b.deliveries;
    for (const auto& f : a) h.deliver(h.b, f);
    for (const auto& f : b) h.deliver(h.b, f);
    c.require("old_data_does_not_release_b", exact(h.b, {first}) && h.b.deliveries == delivered);
    h.b.session.commit_deferred_receive();
    c.require("matching_new_release_completes_b", exact(h.b, {first, second}));
    h.b.session.commit_deferred_receive();
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("duplicate_final_release_inert", exact(h.b, {first, second}));
    c.count("rx_results", h.b.results.size()); c.finish();
}

void tx_boundaries() {
    Check c{"acc_rc5surg_tx_boundaries"};
    for (bool prepare : {false, true}) {
        SurgicalPair h; c.require("connected", h.connect());
        const std::vector<Bytes> old{bytes(3, 0x51), bytes(3, 0x52), bytes(3, 0x53)};
        const auto fresh = bytes(3, 0x71), tail = bytes(3, 0x72);
        h.a.session.defer_transmit_commit();
        for (const auto& r : old) { SurgicalPair::send(h.a, r); c.require("acked_batch", h.pump()); }
        c.require("three_acked_boundaries_held", h.a.results.empty() && h.a.session.pending_frames() == 0 &&
            exact(h.b, old));
        bool entered = false;
        h.a.complete_hook = [&](bool success) {
            if (!success || entered) return;
            entered = true;
            h.a.session.defer_transmit_commit();
            if (prepare) {
                h.a.session.retain_original(fresh.data(), fresh.size());
                h.a.session.retain_original(tail.data(), tail.size());
                h.a.session.send_prepared(fresh.data(), fresh.size(), ArqRecordType::Data, 1);
            }
        };
        h.a.session.commit_deferred_transmit();
        c.require("a_only_old_release", entered && exact(h.a, {old[0]}));
        c.require("fresh_not_in_old_result", h.a.results.size() == 1 &&
            h.a.results[0]->accepted_records == std::vector<Bytes>{old[0]});
        if (prepare) {
            // Withhold fresh DATA and its ACK. An old release cannot complete
            // these originals, including the unprepared transform tail.
            c.require("fresh_transport_waiting", h.a.session.pending_frames() > 0);
            h.a.session.commit_deferred_transmit();
            c.require("old_boundaries_retained", exact(h.a, old));
            h.a.session.reset();
            size_t f = 0, t = 0;
            for (const auto& r : h.a.results) if (!r->success) {
                f += std::count(r->preserved_records.begin(), r->preserved_records.end(), fresh);
                t += std::count(r->preserved_records.begin(), r->preserved_records.end(), tail);
            }
            c.require("prepared_prefix_and_tail_recoverable", f == 1 && t == 1);
        } else {
            h.a.session.commit_deferred_transmit();
            c.require("new_release_all_old_boundaries_once", exact(h.a, old));
            h.a.session.commit_deferred_transmit();
            c.require("repeat_release_no_duplicate", exact(h.a, old));
        }
        c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
            h.b.session.structure_violations() == 0);
    }
    c.finish();
}

void role_reset() {
    Check c{"acc_rc5surg_role_reset"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto unresolved = bytes(301, 0x29), reverse = bytes(3, 0x39);
    SurgicalPair::send(h.a, unresolved); h.take(h.a);
    bool reset = false;
    h.a.complete_hook = [&](bool ok) { if (!ok && !reset) { reset = true; h.a.session.reset(); } };
    SurgicalPair::send(h.b, reverse);
    const auto request = h.take(h.b);
    c.require("real_role_request", request.size() == 1 && request[0].type == ArqType::SWITCH_ROLE);
    const size_t before = h.a.wire.size();
    for (const auto& f : request) h.deliver(h.a, f);
    c.require("failure_published_before_switch_cleanup", reset && h.a.results.size() == 1 &&
        !h.a.results[0]->success && h.a.results[0]->preserved_records == std::vector<Bytes>{unresolved});
    c.require("reset_not_overwritten", reset && h.a.session.state() == ArqState::IDLE &&
        h.a.session.role() == ArqRole::IDLE && frames(h.a, ArqType::SWITCH_ROLE, before) == 0);
    c.require("result_retained_after_reset", h.a.results.size() == 1 &&
        h.a.session.retained_transfer_results() == h.a.results);
    // Independent accepted-switch control: unsent responder originals survive
    // and need their actual reverse ACK, with byte-for-byte delivery.
    SurgicalPair control; c.require("control_connected", control.connect());
    SurgicalPair::send(control.a, bytes(3, 0x17)); c.require("forward_control", control.pump());
    SurgicalPair::send(control.b, reverse);
    c.require("reverse_control", control.pump() && joined(control.a.deliveries) == reverse &&
        control.b.results.size() == 2 && control.b.results.back()->success &&
        control.b.results.back()->accepted_records == std::vector<Bytes>{reverse} &&
        control.b.session.pending_frames() == 0);
    SurgicalPair decision; c.require("decision_connected", decision.connect());
    SurgicalPair::send(decision.a, bytes(3, 0x18));
    c.require("decision_completed_control", decision.pump());
    bool decided = false;
    auto cb = decision.a.callbacks;
    cb.on_role_switch = [&]() {
        decided = true;
        decision.a.session.reset();
        decision.a.session.connect("N0NEW");
        return true;
    };
    decision.a.session.set_callbacks(cb);
    SurgicalPair::send(decision.b, reverse);
    const size_t decision_start = decision.a.wire.size();
    for (const auto& f : decision.take(decision.b)) decision.deliver(decision.a, f);
    c.require("role_decision_reconnect_not_overwritten", decided &&
        decision.a.session.state() == ArqState::HAILING &&
        decision.a.session.role() == ArqRole::COMMANDER &&
        decision.a.session.remote_callsign() == "N0NEW");
    c.require("no_old_role_ack_after_reconnect", frames(decision.a, ArqType::SWITCH_ROLE, decision_start) == 0);
    c.require("role_decision_preserves_final_result", exact(decision.a, {bytes(3, 0x18)}));
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("control_no_structure_violations", control.a.session.structure_violations() == 0 &&
        control.b.session.structure_violations() == 0);
    c.require("decision_no_structure_violations", decision.a.session.structure_violations() == 0 &&
        decision.b.session.structure_violations() == 0);
    c.finish();
}

void state_reconnect() {
    Check c{"acc_rc5surg_state_reconnect"};
    SurgicalPair h; c.require("connected", h.connect());
    // Complete initial connection custody before exercising a new CONNECTING
    // callback; its completed result must survive replacement.
    const auto record = bytes(3, 0x22);
    SurgicalPair::send(h.a, record); c.require("completed_control", h.pump() && exact(h.a, {record}));
    if (h.a.results.empty()) {
        c.require("control_result_available", false);
        c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
            h.b.session.structure_violations() == 0);
        c.finish();
        return;
    }
    const auto completed = h.a.results[0];
    bool entered = false;
    auto cb = h.a.callbacks;
    cb.on_state_changed = [&](ArqState state) {
        if (state == ArqState::CONNECTING && !entered) {
            entered = true;
            h.a.session.reset();
            h.a.session.connect("N0NEW");
        }
    };
    h.a.session.set_callbacks(cb);
    h.a.session.connect("N0OLD"); h.take(h.a);
    const size_t before = h.a.wire.size();
    h.deliver(h.a, {ArqType::HAIL_ACK, 0, 0, {'N','0','O','L','D'}});
    c.require("reconnect_honored", entered && h.a.session.state() == ArqState::HAILING &&
        h.a.session.remote_callsign() == "N0NEW");
    c.require("no_old_connect_effect", frames(h.a, ArqType::CONNECT, before) == 0);
    const auto retained = h.a.session.retained_transfer_results();
    c.require("completed_result_survives", completed->success &&
        std::find(retained.begin(), retained.end(), completed) != retained.end());
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.finish();
}

void speed_reset() {
    Check c{"acc_rc5surg_speed_reset"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto original = bytes(3, 0x2b);
    SurgicalPair::send(h.b, original); h.take(h.b);
    bool entered = false;
    auto cb = h.b.callbacks;
    cb.on_speed_changed = [&](int) { if (!entered) { entered = true; h.b.session.reset(); } };
    h.b.session.set_callbacks(cb);
    const size_t before = h.b.wire.size();
    h.deliver(h.b, {ArqType::SET_SPEED, 0, 1, {}});
    c.require("reset_applied", entered && h.b.session.state() == ArqState::IDLE &&
        h.b.session.role() == ArqRole::IDLE && h.b.session.pending_frames() == 0);
    c.require("no_stale_speed_ack", frames(h.b, ArqType::SPEED_ACK, before) == 0);
    c.require("failed_original_survives", h.b.results.size() == 1 && !h.b.results[0]->success &&
        h.b.results[0]->preserved_records == std::vector<Bytes>{original});
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.finish();
}

void synchronous_ack() {
    Check c{"acc_rc5surg_frame_sync_ack"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto first = bytes(3, 0x61), second = bytes(3, 0x62);
    int send_depth = 0;
    bool nested_terminal = false, nested_mutation = false, submitted = false;
    h.a.complete_hook = [&](bool) { nested_terminal = nested_terminal || send_depth > 0; };
    h.a.frame_hook = [&](const ArqFrame& f) {
        if (f.type != ArqType::DATA) { h.a.air.push_back(f); return; }
        // Bound faulty recursive frame dispatch instead of crashing the suite.
        if (h.routing > 100) { h.valid = false; return; }
        ++send_depth;
        const auto pending = h.a.session.pending_frames();
        const auto results = h.a.results.size();
        h.deliver(h.b, f);
        nested_mutation = nested_mutation || pending != h.a.session.pending_frames() ||
            results != h.a.results.size();
        if (!submitted) { submitted = true; SurgicalPair::send(h.a, second); }
        --send_depth;
    };
    h.b.frame_hook = [&](const ArqFrame& f) {
        if (f.type == ArqType::ACK) h.deliver(h.a, f);
        else h.b.air.push_back(f);
    };
    SurgicalPair::send(h.a, first);
    c.require("routing_quiesced", h.pump());
    c.require("nested_input_waits_for_send_return", !nested_terminal && !nested_mutation);
    c.require("both_slots_retired_once", exact(h.a, {first, second}) && h.a.session.pending_frames() == 0);
    c.require("peer_delivered_once", joined(h.b.deliveries) == joined({first, second}) && exact(h.b, {first, second}));
    c.count("tx_results", h.a.results.size()); c.count("pending_frames", h.a.session.pending_frames());
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.finish();
}

void frame_reset_send() {
    Check c{"acc_rc5surg_frame_reset_send"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto old = bytes(3, 0x64), fresh = bytes(3, 0x65);
    bool entered = false, inside = false, nested = false;
    h.a.complete_hook = [&](bool) { nested = nested || inside; };
    h.a.frame_hook = [&](const ArqFrame& f) {
        if (f.type == ArqType::DATA && !entered) {
            entered = true; inside = true;
            h.a.session.reset(); h.a.session.connect("N0NEW"); SurgicalPair::send(h.a, fresh);
            inside = false;
        } else h.a.air.push_back(f);
    };
    SurgicalPair::send(h.a, old);
    c.require("reset_notification_not_nested", entered && !nested);
    c.require("new_session_retained", h.a.session.state() == ArqState::HAILING &&
        h.a.session.remote_callsign() == "N0NEW");
    c.require("old_failed_once", h.a.results.size() == 1 && !h.a.results[0]->success &&
        h.a.results[0]->preserved_records == std::vector<Bytes>{old});
    // Finish the actual new handshake. The old send's post-callback cursor
    // increment must not create a gap before the replacement's first DATA.
    SurgicalPair peer;
    peer.b.session.listen();
    for (int i = 0; i < 64 && (!h.a.air.empty() || !peer.b.air.empty()); ++i) {
        for (const auto& f : h.take(h.a)) peer.deliver(peer.b, f);
        for (const auto& f : peer.take(peer.b)) h.deliver(h.a, f);
    }
    c.require("replacement_delivered_without_cursor_gap", joined(peer.b.deliveries) == fresh &&
        h.a.session.pending_frames() == 0 && h.a.results.size() == 2 &&
        h.a.results.back()->success && h.a.results.back()->accepted_records == std::vector<Bytes>{fresh} &&
        h.a.results[0]->transfer_id != h.a.results[1]->transfer_id);
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("peer_no_structure_violations", peer.a.session.structure_violations() == 0 &&
        peer.b.session.structure_violations() == 0);
    c.require("bounded_valid_wire", h.valid && peer.valid); c.finish();
}

void callback_pair() {
    Check c{"acc_rc5surg_callback_pair"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto first = bytes(3, 0x81), fresh = bytes(3, 0x82);
    std::vector<std::string> events;
    Result captured, observed;
    bool once = false;
    auto cb = h.a.callbacks;
    cb.on_transfer_complete = [&](bool ok) {
        events.push_back("A.complete.begin");
        c.require("a_success", ok);
        if (!once) {
            once = true;
            captured = h.a.session.last_transfer_result();
            c.require("result_installed_before_callback", captured && captured->success);
            if (captured) c.require("dispose_applied", h.a.session.dispose_transfer_result(captured->transfer_id));
            auto replacement = h.a.callbacks;
            replacement.on_transfer_complete = [&](bool success) {
                events.push_back("B.complete"); c.require("b_failed", !success);
            };
            replacement.on_transfer_result = [&](Result r) {
                events.push_back("B.result"); c.require("b_original_owned", r && !r->success &&
                    r->preserved_records == std::vector<Bytes>{fresh});
            };
            h.a.session.set_callbacks(replacement);
            h.a.session.retain_original(fresh.data(), fresh.size());
            h.a.session.reset();
        }
        events.push_back("A.complete.end");
    };
    cb.on_transfer_result = [&](Result r) { events.push_back("A.result"); observed = r; };
    h.a.session.set_callbacks(cb);
    SurgicalPair::send(h.a, first); c.require("wire_drained", h.pump());
    c.require("captured_pair_atomic", events == std::vector<std::string>{
        "A.complete.begin", "A.complete.end", "A.result", "B.complete", "B.result"});
    c.require("same_immutable_result_after_disposal", captured && observed == captured &&
        observed->success && observed->accepted_records == std::vector<Bytes>{first});
    c.require("replacement_reset_final", h.a.session.state() == ArqState::IDLE);
    for (const auto& event : events)
        std::printf("ACCEPTANCE_OBS name=%s event=%s\n", c.name, event.c_str());
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.finish();
}

void transport_matrix() {
    Check c{"acc_rc5surg_transport_matrix"};
    // The large native record exercises nonterminal ACKs, retries and several
    // modulo wraps; duplicates must not reenter application delivery.
    SurgicalPair h; c.require("connected", h.connect());
    const auto large = bytes(3000, 0x91), later = bytes(301, 0x92);
    SurgicalPair::send(h.a, large);
    const auto initial = h.take(h.a);
    c.require("larger_than_window", initial.size() == 8 && h.a.session.tx_queue_bytes() > 0);
    for (const auto& f : initial) h.deliver(h.b, f);
    c.require("nonterminal_progress", h.pump() && exact(h.a, {large}) && exact(h.b, {large}));
    const auto prior = h.b.deliveries;
    for (const auto& f : initial) h.deliver(h.b, f);
    c.require("wrapped_duplicate_inert", h.pump() && h.b.deliveries == prior && exact(h.b, {large}));
    // Two genuine EOBs, with the later EOB buffered before the first arrives.
    const auto small = bytes(3, 0x93);
    SurgicalPair::send(h.a, small); const auto a = h.take(h.a);
    SurgicalPair::send(h.a, later); const auto b = h.take(h.a);
    c.require("two_real_eobs", a.size() == 1 && b.size() == 3 && (a[0].flags & 0x80) && (b.back().flags & 0x80));
    if (a.size() == 1 && b.size() == 3) {
        h.deliver(h.b, b.back()); h.deliver(h.b, a[0]);
        c.require("first_eob_completes_with_later_buffered", exact(h.b, {large, small}));
        for (size_t i = 0; i + 1 < b.size(); ++i) h.deliver(h.b, b[i]);
        c.require("distinct_native_boundaries", h.pump() && exact(h.a, {large, small, later}) && exact(h.b, {large, small, later}));
    }
    SurgicalPair legacy; c.require("legacy_connected", legacy.connect());
    const ArqFrame old{ArqType::DATA, 0, 0x80, bytes(3, 0x94)};
    legacy.deliver(legacy.b, old); legacy.deliver(legacy.b, old);
    c.require("legacy_eob_duplicate_once", exact(legacy.b, {old.payload}) && legacy.b.deliveries.size() == 1);
    SurgicalPair retry; c.require("retry_connected", retry.connect());
    SurgicalPair::send(retry.a, later); const auto emitted = retry.take(retry.a);
    if (emitted.size() == 3) {
        retry.deliver(retry.b, emitted[0]); retry.deliver(retry.b, emitted[1]);
        retry.pump(); retry.b.session.on_decode_failed();
        for (const auto& f : retry.take(retry.b)) retry.deliver(retry.a, f);
        const auto resent = retry.take(retry.a);
        c.require("retry_all_wire_metadata_immutable", resent.size() == 1 && resent[0].serialize() == emitted.back().serialize());
        for (const auto& f : resent) retry.deliver(retry.b, f);
        c.require("retry_completes_once", retry.pump() && exact(retry.a, {later}) && exact(retry.b, {later}));
    } else c.require("retry_fixture", false);
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("legacy_no_structure_violations", legacy.a.session.structure_violations() == 0 &&
        legacy.b.session.structure_violations() == 0);
    c.require("retry_no_structure_violations", retry.a.session.structure_violations() == 0 &&
        retry.b.session.structure_violations() == 0);
    c.finish();
}

void no_structure_violations() {
    Check c{"acc_rc5surg_no_structure_violations"};
    SurgicalPair h; c.require("connected", h.connect());
    const auto record = bytes(3, 0xa1);
    bool sending = false, reentered = false;
    h.a.frame_hook = [&](const ArqFrame& f) {
        if (f.type != ArqType::DATA) { h.a.air.push_back(f); return; }
        sending = true;
        h.deliver(h.b, f);
        sending = false;
    };
    h.b.frame_hook = [&](const ArqFrame& f) {
        if (f.type == ArqType::ACK) {
            reentered = reentered || sending;
            h.deliver(h.a, f);
        } else h.b.air.push_back(f);
    };
    SurgicalPair::send(h.a, record);
    c.require("routing_quiesced", h.pump());
    c.count("a_structure_violations", h.a.session.structure_violations());
    c.count("b_structure_violations", h.b.session.structure_violations());
    c.require("no_structure_violations", h.a.session.structure_violations() == 0 &&
        h.b.session.structure_violations() == 0);
    c.require("ack_reentered_send_callback", reentered);
    c.require("transfer_completed_once", exact(h.a, {record}) && exact(h.b, {record}) &&
        h.a.session.pending_frames() == 0);
    c.finish();
}

// Explicit fail-closed coverage gaps. These are NOT simulated successful
// implementations. Replace each gate with a real public-path scenario when
// the named seam exists; never change the constant to pass without exercising
// stale authority / allocation failure and observing custody plus wire ACKs.
void missing_seams() {
    Check generation{"acc_rc5surg_release_generation"};
    generation.require("missing_capture_and_replay_specific_release_generation", false);
    generation.finish();
    Check preparation{"acc_rc5surg_preparation_binding"};
    preparation.require("missing_original_owner_preparation_token", false);
    preparation.finish();
    Check allocation{"acc_rc5surg_allocation_failure"};
    allocation.require("missing_scoped_snapshot_and_delivery_allocation_failure_injection", false);
    allocation.finish();
}
} // namespace

void run_acceptance_rc5surg() {
    std::printf("\n--- RC5 Surgical Callback Acceptance Tests ---\n");
    failure_replacement();
    mlkem_reply();
    acceptance_renewal();
    two_rx_batches();
    tx_boundaries();
    role_reset();
    state_reconnect();
    speed_reset();
    synchronous_ack();
    frame_reset_send();
    callback_pair();
    transport_matrix();
    no_structure_violations();
    missing_seams();
}
