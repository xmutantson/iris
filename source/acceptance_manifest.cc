#include "acceptance_manifest.h"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {

struct AcceptanceResult {
    const char* name;
    bool passed;
};

std::vector<AcceptanceResult> results;

constexpr const char* required_tests[] = {
    "acc_cfo_150_hz",
    "acc_widegrid_nv_guard",
    "acc_accept_all_frames",
    "acc_earliest_preamble",
    "acc_loose_pre_ldpc_gate",
    "acc_fail_closed_discontinuity",
    "acc_strict_end_to_end_custody",
    "acc_bounded_timeout_custody",
    "acc_fail_closed_on_epoch_sabm",
    "acc_reset_reports_unsuccessful_custody",
    "acc_wire_compat_v2_negotiation",
    "acc_rc4_b2f_small_drain",
    "acc_rc4_declared_decode_capacity",
    "acc_rc4_primed_commit_order",
    "acc_rc4_fresh_b2f_session",
    "acc_rc4_codec_error_custody",
    "acc_rc4_duplicate_record_once",
    "acc_rc4_fragmented_record",
    "acc_rc8_alias_evidence_ownership",
    "acc_rc8_reentry_not_retransmission",
    "acc_rc8_real_retransmission_combines",
    "acc_rc8_backtoback_frame_drain",
    "acc_rc8_resolved_cfo_keeps_harq",
    "acc_rc8_search_purity",
    "acc_rc8_belief_survives_turnaround",
    "acc_rc8_belief_resets_on_teardown",
    "acc_rc8_rediscovery_not_recombine",
    "acc_rc8_geometry_uses_demod_level",
    "acc_rc8_blind_shape_owner_evidence",
    "acc_rc8_channel_snapshot_owned",
    "acc_rc5_two_small_sends",
    "acc_rc5_duplicate_completed_ack",
    "acc_rc5_delayed_ack_pending_batch",
    "acc_rc5_single_send_3000",
    "acc_rc5_small_sends_sequence_wraps",
    "acc_rc5_lost_final_retry_eob",
    "acc_rc5_reordered_eob_duplicate_data",
    "acc_rc5_buffered_next_batch_boundary",
    "acc_rc5_empty_send",
    "acc_rc5_reconnect",
    "acc_rc5_role_switch_round_trip",
    "acc_rc5_role_switch_pending_custody",
    "acc_rc5surg_failure_replacement",
    "acc_rc5surg_rx_mlkem_reply",
    "acc_rc5surg_rx_acceptance_renewal",
    "acc_rc5surg_two_rx_batches",
    "acc_rc5surg_tx_boundaries",
    "acc_rc5surg_role_reset",
    "acc_rc5surg_state_reconnect",
    "acc_rc5surg_speed_reset",
    "acc_rc5surg_frame_sync_ack",
    "acc_rc5surg_frame_reset_send",
    "acc_rc5surg_callback_pair",
    "acc_rc5surg_transport_matrix",
    "acc_rc5surg_no_structure_violations",
    "acc_rc5surg_release_generation",
    "acc_rc5surg_preparation_binding",
    "acc_rc5surg_allocation_failure",
};

}  // namespace

void acceptance_manifest_record(const char* name, bool passed) {
    results.push_back({name, passed});
}

void run_acceptance_gate(int& passed, int& failed) {
    results.clear();
    run_acceptance_iris();
    run_acceptance_arq();
    run_acceptance_native();
    run_acceptance_rc5surg();

    constexpr int required_count =
        static_cast<int>(sizeof(required_tests) / sizeof(required_tests[0]));
    int missing = 0;
    int duplicated = 0;

    std::printf("\n--- Acceptance Manifest ---\n");
    std::printf("ACCEPTANCE_MANIFEST_BEGIN required=%d\n", required_count);
    for (const char* required : required_tests) {
        int registrations = 0;
        bool outcome = true;
        for (const auto& result : results) {
            if (std::strcmp(required, result.name) == 0) {
                ++registrations;
                outcome = outcome && result.passed;
            }
        }

        if (registrations == 0) {
            ++missing;
            std::printf(
                "ACCEPTANCE_MANIFEST name=%s status=MISSING result=NOT_RUN registrations=0\n",
                required);
            continue;
        }

        if (registrations > 1) ++duplicated;
        std::printf(
            "ACCEPTANCE_MANIFEST name=%s status=EXECUTED result=%s registrations=%d\n",
            required, outcome ? "PASS" : "FAIL", registrations);
        if (outcome)
            ++passed;
        else
            ++failed;
    }

    if (missing != 0 || duplicated != 0) {
        std::printf(
            "ACCEPTANCE_INCOMPLETE missing=%d duplicated=%d registered=%zu required=%d\n",
            missing, duplicated, results.size(), required_count);
        ++failed;
    } else {
        std::printf("ACCEPTANCE_COMPLETE executed=%d required=%d\n",
                    required_count, required_count);
    }
}
