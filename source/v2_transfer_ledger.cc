#include "v2/transfer_ledger.h"

#include "monocypher.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace iris::v2 {
namespace {

bool same_transfer(const TransferIdentity& a, const TransferIdentity& b) noexcept {
    return a.session_id.bytes == b.session_id.bytes &&
           a.direction == b.direction &&
           a.transfer_id.value == b.transfer_id.value;
}

bool same_domains(const ReliabilityDomainIdentities& a,
                  const ReliabilityDomainIdentities& b) noexcept {
    return a.r1.local_client_connection.bytes ==
               b.r1.local_client_connection.bytes &&
           a.r1.sequence_space.value == b.r1.sequence_space.value &&
           a.r2.modem_session.bytes == b.r2.modem_session.bytes &&
           a.r2.sequence_space.value == b.r2.sequence_space.value &&
           a.r3.remote_client_connection.bytes ==
               b.r3.remote_client_connection.bytes &&
           a.r3.sequence_space.value == b.r3.sequence_space.value;
}

bool same_final(const FrozenFinalRecord& a,
                const FrozenFinalRecord& b) noexcept {
    return a.empty_transfer == b.empty_transfer &&
           a.record_count == b.record_count &&
           a.final_record_id.value == b.final_record_id.value &&
           a.final_record_extent == b.final_record_extent &&
           a.catalog_digest == b.catalog_digest;
}

void append_u64_be(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

void append_u32_be(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

FrozenFinalRecord freeze_catalog(const TransferIdentity& transfer,
                                 const std::vector<OriginalRecordDescriptor>& catalog) {
    FrozenFinalRecord final_record;
    final_record.empty_transfer = catalog.empty();
    final_record.record_count = catalog.size();
    if (!catalog.empty()) {
        final_record.final_record_id = catalog.back().record_id;
        final_record.final_record_extent = catalog.back().size;
    }

    std::vector<std::uint8_t> canonical;
    static constexpr std::uint8_t prefix[] = {
        'I','R','I','S','-','V','2','-','C','A','T','A','L','O','G'};
    canonical.insert(canonical.end(), std::begin(prefix), std::end(prefix));
    canonical.insert(canonical.end(), transfer.session_id.bytes.begin(),
                     transfer.session_id.bytes.end());
    canonical.push_back(static_cast<std::uint8_t>(transfer.direction));
    append_u64_be(canonical, transfer.transfer_id.value);
    append_u32_be(canonical, static_cast<std::uint32_t>(catalog.size()));
    for (const auto& record : catalog) {
        append_u64_be(canonical, record.record_id.value);
        append_u64_be(canonical, record.size);
    }
    crypto_blake2b(final_record.catalog_digest.data(),
                   final_record.catalog_digest.size(), canonical.data(),
                   canonical.size());
    return final_record;
}

const OriginalRecordDescriptor* find_record(
    const std::vector<OriginalRecordDescriptor>& catalog,
    OriginalRecordId id) noexcept {
    auto it = std::lower_bound(
        catalog.begin(), catalog.end(), id.value,
        [](const OriginalRecordDescriptor& record, std::uint64_t value) {
            return record.record_id.value < value;
        });
    return it != catalog.end() && it->record_id.value == id.value
        ? &*it : nullptr;
}

bool valid_catalog(const std::vector<OriginalRecordDescriptor>& catalog) noexcept {
    std::uint64_t prior = 0;
    for (const auto& record : catalog) {
        if (record.record_id.value == 0 || record.record_id.value <= prior)
            return false;
        prior = record.record_id.value;
    }
    return true;
}

bool valid_coverage(const OriginalRecordCoverage& coverage,
                    const std::vector<OriginalRecordDescriptor>& catalog) noexcept {
    std::uint64_t prior_id = 0;
    std::uint64_t prior_end = 0;
    for (const auto& range : coverage.ranges) {
        const auto* record = find_record(catalog, range.record_id);
        if (!record || range.length == 0 ||
            !is_bounded_half_open_extent(range.offset, range.length, record->size))
            return false;
        if (range.record_id.value < prior_id ||
            (range.record_id.value == prior_id && range.offset <= prior_end))
            return false;
        prior_id = range.record_id.value;
        prior_end = range.offset + range.length;
    }

    prior_id = 0;
    for (const auto id : coverage.completed_empty_records) {
        const auto* record = find_record(catalog, id);
        if (!record || record->size != 0 || id.value <= prior_id)
            return false;
        prior_id = id.value;
    }
    return true;
}

bool same_coverage(const OriginalRecordCoverage& a,
                   const OriginalRecordCoverage& b) noexcept {
    if (a.ranges.size() != b.ranges.size() ||
        a.completed_empty_records.size() != b.completed_empty_records.size())
        return false;
    for (std::size_t i = 0; i < a.ranges.size(); ++i) {
        if (a.ranges[i].record_id.value != b.ranges[i].record_id.value ||
            a.ranges[i].offset != b.ranges[i].offset ||
            a.ranges[i].length != b.ranges[i].length)
            return false;
    }
    for (std::size_t i = 0; i < a.completed_empty_records.size(); ++i) {
        if (a.completed_empty_records[i].value !=
            b.completed_empty_records[i].value)
            return false;
    }
    return true;
}

bool has_empty(const OriginalRecordCoverage& coverage,
               OriginalRecordId id) noexcept {
    return std::binary_search(
        coverage.completed_empty_records.begin(),
        coverage.completed_empty_records.end(), id.value,
        [](const auto& lhs, const auto& rhs) {
            if constexpr (std::is_same_v<std::decay_t<decltype(lhs)>,
                                         OriginalRecordId>)
                return lhs.value < rhs;
            else
                return lhs < rhs.value;
        });
}

bool range_covered(const OriginalRecordCoverage& coverage,
                   const OriginalRecordRange& wanted) noexcept {
    for (const auto& have : coverage.ranges) {
        if (have.record_id.value > wanted.record_id.value)
            break;
        if (have.record_id.value == wanted.record_id.value &&
            have.offset <= wanted.offset &&
            wanted.length <= have.length - (wanted.offset - have.offset))
            return true;
    }
    return false;
}

bool subset_of(const OriginalRecordCoverage& subset,
               const OriginalRecordCoverage& superset) noexcept {
    for (const auto& range : subset.ranges) {
        if (!range_covered(superset, range))
            return false;
    }
    for (const auto id : subset.completed_empty_records) {
        if (!has_empty(superset, id))
            return false;
    }
    return true;
}

bool accepted_is_complete(const OriginalRecordCoverage& accepted,
                          const std::vector<OriginalRecordDescriptor>& catalog) noexcept {
    for (const auto& record : catalog) {
        if (record.size == 0) {
            if (!has_empty(accepted, record.record_id))
                return false;
        } else if (!range_covered(
                       accepted, {record.record_id, 0, record.size})) {
            return false;
        }
    }
    return true;
}

bool preserved_is_exact(const TransferResult& result,
                        const TransferLedger& ledger) noexcept {
    std::size_t preserved_index = 0;
    std::size_t empty_index = 0;
    for (const auto& record : ledger.original_records) {
        if (record.bytes.empty()) {
            if (!has_empty(result.milestones.endpoint_acknowledged,
                           record.record_id)) {
                if (empty_index >= result.preserved_unresolved_empty_records.size() ||
                    result.preserved_unresolved_empty_records[empty_index].value !=
                        record.record_id.value)
                    return false;
                ++empty_index;
            }
            continue;
        }

        std::uint64_t cursor = 0;
        for (const auto& acknowledged :
             result.milestones.endpoint_acknowledged.ranges) {
            if (acknowledged.record_id.value != record.record_id.value)
                continue;
            if (acknowledged.offset > cursor) {
                if (preserved_index >=
                    result.preserved_unresolved_originals.size())
                    return false;
                const auto& preserved =
                    result.preserved_unresolved_originals[preserved_index++];
                const std::uint64_t length = acknowledged.offset - cursor;
                if (preserved.range.record_id.value != record.record_id.value ||
                    preserved.range.offset != cursor ||
                    preserved.range.length != length ||
                    preserved.bytes.size() != length ||
                    !std::equal(preserved.bytes.begin(), preserved.bytes.end(),
                                record.bytes.begin() + static_cast<std::size_t>(cursor)))
                    return false;
            }
            cursor = acknowledged.offset + acknowledged.length;
        }
        if (cursor < record.bytes.size()) {
            if (preserved_index >= result.preserved_unresolved_originals.size())
                return false;
            const auto& preserved =
                result.preserved_unresolved_originals[preserved_index++];
            const auto length = static_cast<std::uint64_t>(record.bytes.size()) - cursor;
            if (preserved.range.record_id.value != record.record_id.value ||
                preserved.range.offset != cursor ||
                preserved.range.length != length ||
                preserved.bytes.size() != length ||
                !std::equal(preserved.bytes.begin(), preserved.bytes.end(),
                            record.bytes.begin() + static_cast<std::size_t>(cursor)))
                return false;
        }
    }
    return preserved_index == result.preserved_unresolved_originals.size() &&
           empty_index == result.preserved_unresolved_empty_records.size() &&
           result.preserved_incomplete_serialized_record ==
               ledger.incomplete_serialized_record;
}

bool proof_matches(const MatchingCloseProof& proof,
                   const TransferResult& result) noexcept {
    const auto& attestation = proof.remote_attestation();
    const auto& confirmation = proof.confirmation();
    return same_transfer(attestation.transfer, result.transfer) &&
           same_transfer(confirmation.transfer, result.transfer) &&
           attestation.r3.remote_client_connection.bytes ==
               result.domains.r3.remote_client_connection.bytes &&
           confirmation.r3.remote_client_connection.bytes ==
               result.domains.r3.remote_client_connection.bytes &&
           attestation.r3.sequence_space.value ==
               result.domains.r3.sequence_space.value &&
           confirmation.r3.sequence_space.value ==
               result.domains.r3.sequence_space.value &&
           same_final(attestation.accepted_final_record,
                      result.frozen_final_record) &&
           same_final(confirmation.accepted_final_record,
                      result.frozen_final_record) &&
           attestation.close_id != 0 &&
           attestation.close_id == confirmation.close_id &&
           attestation.challenge.bytes == confirmation.challenge.bytes;
}

}  // namespace

TransferResultValidationError validate_transfer_result(
    const TransferResult& result, const TransferLedger& owned_ledger) noexcept {
    if (result.outcome == TransferOutcome::Open ||
        owned_ledger.outcome == TransferOutcome::Open)
        return TransferResultValidationError::NotTerminal;
    if (result.local_connection_id.bytes != owned_ledger.local_connection_id.bytes ||
        result.local_connection_id.bytes !=
            result.domains.r1.local_client_connection.bytes ||
        !same_transfer(result.transfer, owned_ledger.transfer) ||
        result.transfer.session_id.bytes != result.domains.r2.modem_session.bytes ||
        !same_domains(result.domains, owned_ledger.domains))
        return TransferResultValidationError::IdentityMismatch;

    if (!valid_catalog(result.original_records) ||
        result.original_records.size() != owned_ledger.original_records.size())
        return TransferResultValidationError::InvalidRecordCatalog;
    for (std::size_t i = 0; i < result.original_records.size(); ++i) {
        if (result.original_records[i].record_id.value !=
                owned_ledger.original_records[i].record_id.value ||
            result.original_records[i].size !=
                owned_ledger.original_records[i].bytes.size())
            return TransferResultValidationError::InvalidRecordCatalog;
    }

    const auto recomputed = freeze_catalog(result.transfer, result.original_records);
    if (!owned_ledger.frozen_final_record ||
        !same_final(recomputed, result.frozen_final_record) ||
        !same_final(result.frozen_final_record,
                    *owned_ledger.frozen_final_record))
        return TransferResultValidationError::InvalidRecordCatalog;

    if (!valid_coverage(result.milestones.accepted, result.original_records) ||
        !valid_coverage(result.milestones.air_acknowledged,
                        result.original_records) ||
        !valid_coverage(result.milestones.endpoint_acknowledged,
                        result.original_records) ||
        !same_coverage(result.milestones.accepted,
                       owned_ledger.milestones.accepted) ||
        !same_coverage(result.milestones.air_acknowledged,
                       owned_ledger.milestones.air_acknowledged) ||
        !same_coverage(result.milestones.endpoint_acknowledged,
                       owned_ledger.milestones.endpoint_acknowledged) ||
        !accepted_is_complete(result.milestones.accepted,
                              result.original_records))
        return TransferResultValidationError::InvalidOrUnnormalizedMilestone;
    if (!subset_of(result.milestones.air_acknowledged,
                   result.milestones.accepted))
        return TransferResultValidationError::AcknowledgedOutsideAccepted;
    if (!subset_of(result.milestones.endpoint_acknowledged,
                   result.milestones.air_acknowledged))
        return TransferResultValidationError::EndpointAcknowledgedOutsideAir;
    if (!preserved_is_exact(result, owned_ledger))
        return TransferResultValidationError::PreservedRangeOrLengthMismatch;

    const bool succeeded = result.outcome == TransferOutcome::Succeeded;
    if (succeeded != is_success_reason(result.reason) ||
        (!succeeded && !is_failure_reason(result.reason)))
        return TransferResultValidationError::InvalidOutcomeReasonCombination;
    if (succeeded) {
        if (!result.close_proof ||
            !same_coverage(result.milestones.endpoint_acknowledged,
                           result.milestones.accepted) ||
            !result.preserved_unresolved_originals.empty() ||
            !result.preserved_unresolved_empty_records.empty() ||
            !result.preserved_incomplete_serialized_record.empty() ||
            !proof_matches(*result.close_proof, result))
            return TransferResultValidationError::InvalidCloseProofCombination;
    } else if (result.close_proof) {
        return TransferResultValidationError::InvalidCloseProofCombination;
    }
    return TransferResultValidationError::None;
}

}  // namespace iris::v2
