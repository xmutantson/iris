#ifndef IRIS_V2_IDENTITY_H
#define IRIS_V2_IDENTITY_H

// Author: xmutantson

#include <array>
#include <cstdint>
#include <vector>

namespace iris::v2 {

// RC1 allocates a fresh nonzero 128-bit CSPRNG connection ID on each client
// connect/reconnect, never reusing a live or retained ID. Collision retries before
// admission. Local/remote below always mean transfer-origin/transfer-destination,
// including ResponderToInitiator transfers; they never mean this process/peer.
struct ConnectionId {
    std::array<std::uint8_t, 16> bytes{};
};

struct ConnectionNonce {
    std::array<std::uint8_t, 16> bytes{};
};

struct SessionId {
    std::array<std::uint8_t, 16> bytes{};
};

// Config-prefixed BLAKE2b-256 as defined in session.h over SelectedSessionConfig.
// This type lives with identities so envelopes and frames can bind the selection without a
// dependency on handshake declarations.
struct SelectedConfigFingerprint {
    std::array<std::uint8_t, 32> bytes{};
};

// A direction is defined by the endpoint roles in the live handshake, not by a
// process-local TX/RX point of view. It has the same value at both endpoints.
enum class TransferDirection : std::uint8_t {
    InitiatorToResponder = 0,
    ResponderToInitiator,
};

// TransferId is allocated by the origin endpoint, monotonically in [1, UINT64_MAX]
// within (session_id, direction). Zero is invalid. Values are never reused;
// exhaustion rejects further admission and requires a fresh session. TransferId
// alone is therefore never a key: TransferIdentity is the minimum transfer key.
struct TransferId {
    std::uint64_t value = 0;
};

struct TransferIdentity {
    SessionId session_id{};
    TransferDirection direction = TransferDirection::InitiatorToResponder;
    TransferId transfer_id{};
};

// Record IDs are allocated monotonically in [1, UINT64_MAX] within one
// TransferIdentity. Numeric order is the normative original-record admission
// order. Zero is invalid, IDs are immutable and never reused, and exhaustion
// fails the transfer before accepting another record.
struct OriginalRecordId {
    std::uint64_t value = 0;
};

// Encoded IDs have the same scope, allocation order, invalid-zero convention,
// no-reuse rule, and exhaustion behavior as OriginalRecordId. They identify a
// single immutable representation attempt, not merely equal encoded bytes.
// Adopted encoded records strictly follow original admission order with exactly
// one adopted representation per original. Unadopted attempts earn no ACK credit.
struct EncodedRecordId {
    std::uint64_t value = 0;
};

struct FrameId {
    std::uint64_t value = 0;
};

// Flight and frame IDs are monotonic nonzero values within TransferIdentity and
// are never reused. Exhaustion fails the transfer; modulo wire values are not IDs.
struct FlightId {
    std::uint64_t value = 0;
};

struct R1SequenceSpaceId {
    std::uint64_t value = 0;
};

struct R2SequenceSpaceId {
    std::uint64_t value = 0;
};

struct R3SequenceSpaceId {
    std::uint64_t value = 0;
};

// Fragment ordinals are contiguous starting at 1 within TransferIdentity, in
// encoded-record/byte order. UINT64_MAX is reserved as a one-past limit; the last
// allocatable ordinal is UINT64_MAX-1. Refuse admission before exhaustion, never
// wrap or reuse. A zero-byte encoded record has one zero-length marker fragment,
// so its completion has its own ordinal and cannot vanish from ACK accounting.
struct FragmentOrdinal {
    std::uint64_t value = 0;
};

// This catalog entry is the authority for range bounds and cumulative ordering.
// Entries strictly increase by record_id and include every admitted record,
// including zero-size records. Size and ID are frozen on admission.
struct OriginalRecordDescriptor {
    OriginalRecordId record_id{};
    std::uint64_t size = 0;
};

constexpr bool is_bounded_half_open_extent(std::uint64_t offset,
                                           std::uint64_t length,
                                           std::uint64_t size) noexcept {
    return offset <= size && length <= size - offset;
}

// A range denotes [offset, offset + length) within exactly one catalog record.
// Validate without addition: offset <= size && length <= size - offset. Empty
// ranges are omitted from normalized range sets; empty records remain visible in
// the catalog and in cumulative boundary advancement.
struct OriginalRecordRange {
    OriginalRecordId record_id{};
    std::uint64_t offset = 0;
    std::uint64_t length = 0;
};

enum class RecordBoundaryPosition : std::uint8_t {
    BeforeFirst = 0,
    InRecord,
    AfterFinal,
    AtAdmittedEnd,
};

// Canonical first-not-covered position. BeforeFirst (zero ID/offset) is the
// sole initial boundary of a nonempty catalog, not an alias for (first,0).
// InRecord (first,0) is forbidden; otherwise offset < size, except (empty,0)
// marks that empty record as not yet crossed. Completing A normalizes to the
// next catalog record at zero, crossing empty records only on explicit completion.
// AfterFinal (zero ID/offset) is legal only for a frozen catalog after every
// record, including empty ones, is complete. It is the sole frozen-empty cursor.
// For an open catalog use AtAdmittedEnd (last ID,size), or BeforeFirst when empty;
// appending atomically normalizes AtAdmittedEnd to the new record at zero.
// Freezing normalizes AtAdmittedEnd to AfterFinal; a frozen empty catalog uses
// AfterFinal immediately. These tagged positions distinguish an uncompleted last
// empty record from a completed empty record even though both offsets are zero.
// A cursor never establishes catalog identity: close also requires FrozenFinalRecord.
struct OriginalRecordBoundary {
    RecordBoundaryPosition position = RecordBoundaryPosition::BeforeFirst;
    OriginalRecordId record_id{};
    std::uint64_t next_offset = 0;
};

enum class RecordContractError : std::uint8_t {
    None = 0,
    InvalidIdentity,
    InvalidCatalogOrder,
    RangeOutOfBounds,
    NonCanonicalRangeSet,
    NonCanonicalBoundary,
    ArithmeticOverflow,
};

// Validators use the supplied catalog instead of comparing record IDs or
// boundaries ad hoc. A normalized set is sorted by catalog order/offset,
// contains no empty range or overlap, and merges adjacency within one record.
// Ranges never merge across IDs. Boundary validation also needs frozen state.
RecordContractError validate_original_record_catalog(
    const std::vector<OriginalRecordDescriptor>& catalog) noexcept;
RecordContractError validate_original_record_range(
    const OriginalRecordRange& range,
    const std::vector<OriginalRecordDescriptor>& catalog) noexcept;
RecordContractError validate_normalized_original_ranges(
    const std::vector<OriginalRecordRange>& ranges,
    const std::vector<OriginalRecordDescriptor>& catalog) noexcept;
RecordContractError validate_original_record_boundary(
    const OriginalRecordBoundary& boundary,
    const std::vector<OriginalRecordDescriptor>& catalog, bool catalog_frozen) noexcept;

// Empty records have explicit membership in every milestone/complement. These
// IDs are unique, sorted, and must refer to size-zero entries in the same catalog.
struct OriginalRecordCoverage {
    std::vector<OriginalRecordRange> ranges{};
    std::vector<OriginalRecordId> completed_empty_records{};
};

// Frozen once before close admission. Empty means count=0, final ID=0, extent=0.
// Otherwise final ID/extent exactly equal the last admitted descriptor. The digest
// is BLAKE2b-256("IRIS-V2-CATALOG" || canonical TransferIdentity || canonical
// vector<OriginalRecordDescriptor>), including empty entries. No append afterward.
struct FrozenFinalRecord {
    bool empty_transfer = true;
    std::uint64_t record_count = 0;
    OriginalRecordId final_record_id{};
    std::uint64_t final_record_extent = 0;
    std::array<std::uint8_t, 32> catalog_digest{};
};

// RC1 allocates independent nonzero sequence-space IDs monotonically per client
// ConnectionId, restarting neither counter on SABM nor protocol reset. Each new
// AX.25 establishment/SABM/reset creates a new space, even on the same socket;
// exhaust before wrap and require a new connection. R2's RC2-owned counter has
// the same nonreuse rules within SessionId; the handshake initiator is its sole
// allocator and authenticates the agreed space to the responder. R2 reset revokes
// the session, requiring a fresh handshake before any new space is usable.
// Old-space cursors never acknowledge new data.
struct R1DomainIdentity {
    ConnectionId local_client_connection{};
    R1SequenceSpaceId sequence_space{};
};

struct R2DomainIdentity {
    SessionId modem_session{};
    R2SequenceSpaceId sequence_space{};
};

struct R3DomainIdentity {
    ConnectionId remote_client_connection{};
    R3SequenceSpaceId sequence_space{};
};

struct ReliabilityDomainIdentities {
    R1DomainIdentity r1{};
    R2DomainIdentity r2{};
    R3DomainIdentity r3{};
};

// Absolute client positions start at zero and count I-frame records, including
// empty I-frame records. UINT64_MAX is a boundary only. Wire modulo N(R) is
// reconstructed uniquely against that domain's owned outstanding window, at most
// half its modulus. Each entry binds one I-frame position to an owned ordered
// original-range mapping (including explicit empty records), allowing client frame
// segmentation without losing original identity. Concatenated slices equal the
// exact client I-frame payload; entries never duplicate already mapped bytes or
// empty records. Cumulative ACK advances the normalized union of mappings before
// next_absolute_position. Retransmission allocates no new entry. R1 advances
// only after ledger ownership; R3 advances only on validated remote client ACK,
// never on socket write or R2 ACK. RC1 retains these mappings across retries.
struct ClientRecordSequenceMapping {
    std::uint64_t absolute_position = 0;
    TransferIdentity transfer{};
    OriginalRecordCoverage original_mapping{};
    std::uint64_t client_payload_byte_count = 0;
};
struct R1CumulativeCursor {
    R1DomainIdentity domain{};
    std::uint64_t next_absolute_position = 0;
};
struct R3CumulativeCursor {
    R3DomainIdentity domain{};
    std::uint64_t next_absolute_position = 0;
};
struct R1SequenceLedger {
    R1CumulativeCursor custody_cursor{};
    std::vector<ClientRecordSequenceMapping> owned_mapping{};
};
struct R3SequenceLedger {
    R3CumulativeCursor endpoint_cursor{};
    std::vector<ClientRecordSequenceMapping> owned_mapping{};
};

static_assert(sizeof(ConnectionNonce) == 16, "v2 connection nonces are 128-bit");
static_assert(sizeof(SessionId) == 16, "v2 session IDs are 128-bit");
static_assert(static_cast<std::uint8_t>(TransferDirection::InitiatorToResponder) == 0 &&
                  static_cast<std::uint8_t>(TransferDirection::ResponderToInitiator) == 1,
              "transfer direction has a stable Profile-1 wire value");

}  // namespace iris::v2

#endif  // IRIS_V2_IDENTITY_H
