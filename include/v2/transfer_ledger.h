#ifndef IRIS_V2_TRANSFER_LEDGER_H
#define IRIS_V2_TRANSFER_LEDGER_H

// Author: xmutantson
#include "v2/identity.h"
#include "v2/integrity.h"
#include "v2/session.h"

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace iris::v2 {

enum class TransferOutcome : std::uint8_t {
    Open = 0,
    Succeeded,
    Failed,
};

enum class TransferResultReason : std::uint8_t {
    None = 0,
    RemoteEndpointAcceptedAndClosed,
    AdmissionCapacityUnavailable,
    PeerLost,
    RetryExhausted,
    RemoteCloseTimeout,
    SessionReplaced,
    MalformedTransportState,
    CodecFailure,
    LocalClientLost,
    RemoteClientLost,
    AudioDiscontinuity,
    DeviceStopped,
    DeviceError,
    LivenessTimeout,
    ExplicitReset,
    ProtocolViolation,
};

constexpr bool is_success_reason(TransferResultReason reason) noexcept {
    return reason == TransferResultReason::RemoteEndpointAcceptedAndClosed;
}

constexpr bool is_failure_reason(TransferResultReason reason) noexcept {
    return reason != TransferResultReason::None && !is_success_reason(reason);
}

struct LedgerOriginalRecord {
    OriginalRecordId record_id{};
    std::vector<std::uint8_t> bytes{};
};

// bytes.size() must equal range.length exactly. Preserved values form the exact
// normalized complement accepted - endpoint_acknowledged at the selected R3
// endpoint; they are never computed from accepted - air_acknowledged.
struct PreservedOriginal {
    OriginalRecordRange range{};
    std::vector<std::uint8_t> bytes{};
};

// Coverage includes normalized half-open ranges AND explicit completed empty IDs.
// air_acknowledged and endpoint_acknowledged are each subsets of accepted.
// endpoint_acknowledged is also a subset of air_acknowledged: R3 authority cannot
// precede receipt across R2. No set is inferred from queue occupancy.
struct TransferMilestones {
    OriginalRecordCoverage accepted{};
    OriginalRecordCoverage air_acknowledged{};
    OriginalRecordCoverage endpoint_acknowledged{};
};

struct CloseChallenge {
    std::array<std::uint8_t, 16> bytes{};
};

// close_id is allocated monotonically and nonzero within TransferIdentity and is
// never reused. challenge is a new nonzero 128-bit CSPRNG value for each request.
// The transfer origin sends CloseRequest only after fixing expected_final_record
// and the selected R3 domain. Profile-1 protection covers all fields in order.
struct CloseRequest {
    TransferIdentity transfer{};
    R3DomainIdentity r3{};
    FrozenFinalRecord expected_final_record{};
    std::uint64_t close_id = 0;
    CloseChallenge challenge{};
    IntegrityInfo integrity{};
};

// The transfer receiver sends this only after receiving a validated CloseRequest
// and cumulative acceptance by the named R3 client reaches its final boundary. It
// echoes the outstanding request identity, R3 domain, close ID and challenge. All
// fields except integrity are covered in declaration order by its protection.
struct RemoteCloseAttestation {
    TransferIdentity transfer{};
    R3DomainIdentity r3{};
    FrozenFinalRecord accepted_final_record{};
    std::uint64_t close_id = 0;
    CloseChallenge challenge{};
    IntegrityInfo integrity{};
};

// After receiving and validating RemoteCloseAttestation, the transfer origin sends
// this receipt. The receiver must receive and validate it against the same live
// exchange before replying. It is not itself sufficient for origin-side success.
struct CloseAttestationReceipt {
    TransferIdentity transfer{};
    R3DomainIdentity r3{};
    FrozenFinalRecord accepted_final_record{};
    std::uint64_t close_id = 0;
    CloseChallenge challenge{};
    IntegrityInfo integrity{};
};

// The transfer receiver sends this final message only after receiving a validated
// CloseAttestationReceipt. The origin must receive and validate it; local creation
// of a structurally equal value is not authority. Profile-1 confirmation protection
// covers every preceding field in declaration order.
struct CloseConfirmation {
    TransferIdentity transfer{};
    R3DomainIdentity r3{};
    FrozenFinalRecord accepted_final_record{};
    std::uint64_t close_id = 0;
    CloseChallenge challenge{};
    IntegrityInfo integrity{};
};

enum class CloseProofValidationError : std::uint8_t {
    None = 0,
    NoOutstandingRequest,
    TransferMismatch,
    R3DomainMismatch,
    FinalBoundaryMismatch,
    CatalogNotFrozen,
    InactiveSession,
    WrongPeerOrRole,
    WrongExchangeStage,
    StaleRevision,
    CloseIdMismatch,
    StaleOrReplayedChallenge,
    AttestationIntegrityFailure,
    ConfirmationIntegrityFailure,
    EvidenceNotAuthoritative,
};

// These wrappers can be created only by the protected receive boundary after a
// keyed tag, current session/direction and message domain verify. They preserve
// peer-receive provenance, so an ordinary locally populated message cannot be
// passed off as a received confirmation.
class ReceivedCloseRequest {
public:
    ReceivedCloseRequest(ReceivedCloseRequest&&) noexcept;
    ReceivedCloseRequest& operator=(ReceivedCloseRequest&&) noexcept;
    ReceivedCloseRequest(const ReceivedCloseRequest&) = delete;
    ReceivedCloseRequest& operator=(const ReceivedCloseRequest&) = delete;
    const CloseRequest& message() const noexcept { return message_; }
private:
    explicit ReceivedCloseRequest(CloseRequest message) : message_(std::move(message)) {}
    CloseRequest message_{};
    std::uint64_t session_generation_ = 0;
    std::uint64_t exchange_revision_ = 0;
    std::uint64_t receive_sequence_ = 0;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
};

class ReceivedRemoteCloseAttestation {
public:
    ReceivedRemoteCloseAttestation(ReceivedRemoteCloseAttestation&&) noexcept;
    ReceivedRemoteCloseAttestation& operator=(ReceivedRemoteCloseAttestation&&) noexcept;
    ReceivedRemoteCloseAttestation(const ReceivedRemoteCloseAttestation&) = delete;
    ReceivedRemoteCloseAttestation& operator=(const ReceivedRemoteCloseAttestation&) = delete;
    const RemoteCloseAttestation& message() const noexcept { return message_; }
private:
    explicit ReceivedRemoteCloseAttestation(RemoteCloseAttestation message)
        : message_(std::move(message)) {}
    RemoteCloseAttestation message_{};
    std::uint64_t session_generation_ = 0;
    std::uint64_t exchange_revision_ = 0;
    std::uint64_t receive_sequence_ = 0;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
};

class ReceivedCloseAttestationReceipt {
public:
    ReceivedCloseAttestationReceipt(ReceivedCloseAttestationReceipt&&) noexcept;
    ReceivedCloseAttestationReceipt& operator=(ReceivedCloseAttestationReceipt&&) noexcept;
    ReceivedCloseAttestationReceipt(const ReceivedCloseAttestationReceipt&) = delete;
    ReceivedCloseAttestationReceipt& operator=(const ReceivedCloseAttestationReceipt&) = delete;
    const CloseAttestationReceipt& message() const noexcept { return message_; }
private:
    explicit ReceivedCloseAttestationReceipt(CloseAttestationReceipt message)
        : message_(std::move(message)) {}
    CloseAttestationReceipt message_{};
    std::uint64_t session_generation_ = 0;
    std::uint64_t exchange_revision_ = 0;
    std::uint64_t receive_sequence_ = 0;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
};

class ReceivedCloseConfirmation {
public:
    ReceivedCloseConfirmation(ReceivedCloseConfirmation&&) noexcept;
    ReceivedCloseConfirmation& operator=(ReceivedCloseConfirmation&&) noexcept;
    ReceivedCloseConfirmation(const ReceivedCloseConfirmation&) = delete;
    ReceivedCloseConfirmation& operator=(const ReceivedCloseConfirmation&) = delete;
    const CloseConfirmation& message() const noexcept { return message_; }
private:
    explicit ReceivedCloseConfirmation(CloseConfirmation message)
        : message_(std::move(message)) {}
    CloseConfirmation message_{};
    std::uint64_t session_generation_ = 0;
    std::uint64_t exchange_revision_ = 0;
    std::uint64_t receive_sequence_ = 0;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
};

// RC1 owns each CloseExchangeState inside RC2's LiveSession registry keyed by
// TransferIdentity. Only begin() may create an origin request: it atomically
// freezes the ledger catalog and installs request/challenge/role/stage/revision.
// Receiver admission independently matches its frozen catalog and session-owned
// R3 ledger. It waits for missing catalog/data before attesting; a truncated local
// catalog is never accepted as equivalent to the requested digest/count/final ID.
// Received attestation advances Requested -> Attested; receipt is sent only then;
// received confirmation requires ReceiptSent. Receiver stages require validated
// request, complete R3 cursor/empty membership, then a validated received receipt
// before confirmation may be sent. Validators advance those owned stages and
// retain receive provenance; keys are selected by message SENDER, not data origin.
// A failed or consumed exchange is a retained tombstone, never re-opened by replay.
struct TransferLedger;
class CloseExchangeOwner {
public:
    static std::optional<CloseRequest> begin(
        LiveSession& session, TransferLedger& ledger,
        CloseProofValidationError& error) noexcept;
    static std::optional<CloseAttestationReceipt> send_receipt(
        LiveSession& session, const TransferIdentity& transfer,
        CloseProofValidationError& error) noexcept;
    static std::optional<RemoteCloseAttestation> send_attestation(
        LiveSession& session, const TransferIdentity& transfer,
        CloseProofValidationError& error) noexcept;
    static std::optional<CloseConfirmation> send_confirmation(
        LiveSession& session, const TransferIdentity& transfer,
        CloseProofValidationError& error) noexcept;
};

class CloseReceiveValidator {
public:
    static std::optional<ReceivedCloseRequest> validate_request(
        LiveReceiveContext& receive, const CloseRequest& decoded,
        CloseProofValidationError& error) noexcept;
    static std::optional<ReceivedRemoteCloseAttestation> validate_attestation(
        LiveReceiveContext& receive, const RemoteCloseAttestation& decoded,
        CloseProofValidationError& error) noexcept;
    static std::optional<ReceivedCloseAttestationReceipt> validate_receipt(
        LiveReceiveContext& receive, const CloseAttestationReceipt& decoded,
        CloseProofValidationError& error) noexcept;
    static std::optional<ReceivedCloseConfirmation> validate_confirmation(
        LiveReceiveContext& receive, const CloseConfirmation& decoded,
        CloseProofValidationError& error) noexcept;
};

// Only CloseProofValidator can construct this type from two separately received
// peer messages. Both messages must match the still-outstanding request and each
// other in transfer/session/direction, R3 domain, frozen final record/catalog,
// close ID and fresh challenge, and both keyed tags must verify. The
// request is consumed on success, so a replay cannot validate another close.
class MatchingCloseProof {
public:
    MatchingCloseProof(MatchingCloseProof&&) noexcept = default;
    MatchingCloseProof& operator=(MatchingCloseProof&&) noexcept = default;
    MatchingCloseProof(const MatchingCloseProof&) = delete;
    MatchingCloseProof& operator=(const MatchingCloseProof&) = delete;
    const RemoteCloseAttestation& remote_attestation() const noexcept {
        return remote_attestation_;
    }
    const CloseConfirmation& confirmation() const noexcept { return confirmation_; }

private:
    MatchingCloseProof(RemoteCloseAttestation attestation,
                       CloseConfirmation confirmation)
        : remote_attestation_(std::move(attestation)),
          confirmation_(std::move(confirmation)) {}
    RemoteCloseAttestation remote_attestation_{};
    CloseConfirmation confirmation_{};
    friend class CloseProofValidator;
};

struct CloseProofValidationResult {
    CloseProofValidationError error = CloseProofValidationError::NoOutstandingRequest;
    std::optional<MatchingCloseProof> proof{};
};

class CloseProofValidator {
public:
    static CloseProofValidationResult validate_received_messages(
        LiveReceiveContext& receive,
        ReceivedRemoteCloseAttestation&& received_attestation,
        ReceivedCloseConfirmation&& received_confirmation) noexcept;
    // Under one live owner transaction, look up the outstanding exchange, recheck
    // session/peer/generation, both provenance records, frozen final identity and
    // catalog, close ID/challenge and required stages; atomically consume it and
    // mint proof. A const or freely constructed request is never an authority.
};

// A TransferResult is an immutable terminal snapshot: outcome is never Open.
// original_records is the complete accepted catalog and bounds every range.
// accepted covers every byte of every catalog record; acknowledged sets are the
// normalized subsets described above. preserved_unresolved_originals is exactly
// accepted - endpoint_acknowledged, with byte-for-byte slices from ledger-owned
// originals. Therefore a close failure retains conclusive acknowledgments and
// does not relabel them undelivered; fully endpoint-acknowledged data may coexist
// with a Failed/RemoteCloseTimeout result and an empty preserved complement.
//
// Failure also freezes the complete admitted catalog before snapshot publication.
// Succeeded iff reason is RemoteEndpointAcceptedAndClosed, endpoint_acknowledged
// equals accepted (including empty membership), preserved is empty, and
// close_proof matches frozen_final_record. Failed requires a non-None nonsuccess
// reason and no close proof.
// local_connection_id must equal domains.r1.local_client_connection;
// transfer.session_id must equal domains.r2.modem_session. Every proof duplicate
// must equal transfer, domains.r3, and frozen_final_record recomputed from catalog.
// Unresolved empty IDs are exactly accepted empty IDs minus endpoint empty IDs;
// their preservation is explicit even though their byte complement is empty.
// Validation against owned ledger originals checks preserved contents, not just size.
struct TransferResult {
    ConnectionId local_connection_id{};
    TransferIdentity transfer{};
    ReliabilityDomainIdentities domains{};
    TransferOutcome outcome = TransferOutcome::Open;
    TransferResultReason reason = TransferResultReason::None;
    std::vector<OriginalRecordDescriptor> original_records{};
    TransferMilestones milestones{};
    std::vector<PreservedOriginal> preserved_unresolved_originals{};
    std::vector<OriginalRecordId> preserved_unresolved_empty_records{};
    // Exact receiver-side R2 bytes accepted between complete record boundaries.
    // They are not silently dropped merely because no catalog entry is complete.
    std::vector<std::uint8_t> preserved_incomplete_serialized_record{};
    FrozenFinalRecord frozen_final_record{};
    std::optional<MatchingCloseProof> close_proof{};
};

enum class TransferResultValidationError : std::uint8_t {
    None = 0,
    NotTerminal,
    IdentityMismatch,
    InvalidRecordCatalog,
    InvalidOrUnnormalizedMilestone,
    AcknowledgedOutsideAccepted,
    EndpointAcknowledgedOutsideAir,
    PreservedRangeOrLengthMismatch,
    IncompleteEndpointAccounting,
    InvalidOutcomeReasonCombination,
    InvalidCloseProofCombination,
};

TransferResultValidationError validate_transfer_result(
    const TransferResult& result, const TransferLedger& owned_ledger) noexcept;

static_assert(!is_failure_reason(TransferResultReason::None),
              "a terminal failure must publish a cause");
static_assert(!is_failure_reason(
                  TransferResultReason::RemoteEndpointAcceptedAndClosed),
              "the success reason cannot label a failed transfer");

// Live ledger records have the same catalog order and byte-size authority as a
// result. accepted may advance only after the record bytes are owned here. The
// single terminal operation validates and snapshots before transient state reset.
struct TransferLedger {
    ConnectionId local_connection_id{};
    TransferIdentity transfer{};
    ReliabilityDomainIdentities domains{};
    std::vector<LedgerOriginalRecord> original_records{};
    TransferMilestones milestones{};
    std::optional<FrozenFinalRecord> frozen_final_record{};
    R1SequenceLedger r1_sequence{};
    R3SequenceLedger r3_sequence{};
    bool terminal_published = false;
    TransferOutcome outcome = TransferOutcome::Open;
    TransferResultReason terminal_reason = TransferResultReason::None;
    std::optional<MatchingCloseProof> close_proof{};
    std::vector<std::uint8_t> incomplete_serialized_record{};
};

// RC1 is sole mutator of TransferLedger. Admission owns exact original bytes
// before R1 custody advancement, fixes selected domains, and refuses after freeze.
// finish snapshots and validates under the owner lock BEFORE any transient reset;
// the terminal latch permits exactly one published immutable snapshot. Reentry
// returns no new result. Retain failed snapshots until explicit client disposition.
// No air ACK releases accepted originals awaiting R3. Proof must be the consumed
// exchange proof in this session's registry, not a copied proof from another ledger.
class TransferLedgerOwner {
public:
    static std::shared_ptr<const TransferResult> finish(
        LiveSession& session, TransferLedger& ledger,
        TransferOutcome outcome, TransferResultReason reason,
        TransferResultValidationError& error) noexcept;
};

}  // namespace iris::v2

#endif  // IRIS_V2_TRANSFER_LEDGER_H
