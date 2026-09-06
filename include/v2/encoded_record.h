#ifndef IRIS_V2_ENCODED_RECORD_H
#define IRIS_V2_ENCODED_RECORD_H

// Author: xmutantson
#include "v2/identity.h"
#include "v2/integrity.h"
#include <array>
#include <cstdint>
#include <vector>

namespace iris::v2 {

inline constexpr std::uint16_t kEncodedEnvelopeVersion = 1; // Only accepted version.

enum class RecordRepresentation : std::uint8_t {
    Opaque = 0, Compressed, Encrypted, CompressedThenEncrypted,
};
enum class CodecId : std::uint16_t {
    None = 0, Zstd = 1, Ppmd = 2, Lzhuf = 3, NegotiatedPrivate = 0x8000,
};
enum class CodecProfile : std::uint16_t {
    OpaqueBytes1 = 1, ZstdIndependent1 = 2, RegisteredExtension = 0x8000,
};
enum class EncryptionAlgorithm : std::uint8_t {
    None = 0, XChaCha20Poly1305 = 1,
};
inline constexpr std::uint32_t kXChaCha20Poly1305NonceBytes = 24;
inline constexpr std::uint32_t kXChaCha20Poly1305TagBytes = 16;

// ID = BLAKE2b-256(u16 format_version || u64 size || exact dictionary bytes).
// The all-zero ID and version zero mean no dictionary (not a hash of empty input).
// Format 1 is a standard zstd full dictionary, including its embedded dictionary ID.
struct DictionaryIdentity {
    std::array<std::uint8_t, 32> content_id{};
    std::uint16_t format_version = 0;
};

// RC4 owns registered codec wire specifications and conformance vectors.
// Digest = BLAKE2b-256(exact published specification bytes); zero is forbidden for
// extensions. Registration must fix codec/version, parameters, framing, model
// initialization/update/reset and dictionary format. Unknown digests fail selection.
// No family name, implementation-local option blob, or matching vendor name suffices.
struct CodecExtensionSpecId {
    std::array<std::uint8_t, 32> digest{};
};

// OpaqueBytes1: codec None, all other fields zero, exact original bytes.
// ZstdIndependent1: codec Zstd, one standard non-skippable zstd frame per record,
// content size present and equal original_size, checksum enabled, no concatenation
// or trailing bytes. Window <= max_decode_window_bytes. Dictionary either absent
// or exactly the selected content-addressed format-1 bytes and embedded ID. Each
// record starts a fresh context; no cross-record prefix/model state. Encoder choice
// of compression level is immaterial to decoding; the encoded attempt is immutable.
// Ppmd/Lzhuf/private require RegisteredExtension and a registered nonzero spec ID;
// built-in profiles require extension zero. No implicit B2F semantic rewriting.
// Dictionary epoch is 0 iff absent, otherwise 1 at transfer start. Model epoch is
// 0 for built-ins; a stateful registered profile starts at 1 in TransferIdentity,
// initialized from the selected dictionary/profile. A new epoch requires a new
// authenticated session/transfer, never unilateral reset. RC4 owns extension state
// commits: ordered TX adoption and validated RX plaintext acceptance, exactly once.
struct CodecEpochBinding {
    CodecId codec = CodecId::None;
    CodecProfile profile = CodecProfile::OpaqueBytes1;
    CodecExtensionSpecId extension{};
    DictionaryIdentity dictionary{};
    std::uint64_t dictionary_epoch = 0;
    std::uint64_t model_epoch = 0;
};

// Every limit is nonzero; empty records remain legal. Aggregate storage reserves
// original output, encoded payload, canonical envelope metadata, and fragment
// registry metadata BEFORE acceptance. RC4 accounts these exact byte budgets;
// count limits also bound zero-byte markers and per-object allocation overhead.
struct EnvelopeAdmissionLimits {
    std::uint64_t max_original_record_bytes = 0;
    std::uint64_t max_encoded_record_bytes = 0;
    std::uint32_t max_fragment_count = 0;
    std::uint64_t max_aggregate_reassembly_bytes = 0;
    std::uint32_t max_incomplete_records = 0;
    std::uint64_t max_envelope_wire_bytes = 0;
    std::uint64_t max_decode_window_bytes = 0;
};
struct NegotiatedCodecSet {
    SelectedConfigFingerprint selected_config{};
    EncryptionAlgorithm encryption = EncryptionAlgorithm::None;
    std::vector<CodecEpochBinding> bindings{};
};
struct EncodedFragmentExtent {
    FragmentOrdinal fragment_ordinal{};
    std::uint64_t encoded_offset = 0;
    std::uint64_t encoded_length = 0;
};

// One envelope transforms exactly one complete original record. Re-encoding under
// another ID cannot earn duplicate credit; RC4's owned catalog identifies the one
// adopted representation. All fields below are immutable after adoption.
struct EncodedRecordEnvelopeIdentity {
    std::uint16_t version = kEncodedEnvelopeVersion;
    TransferIdentity transfer{};
    OriginalRecordId original_record_id{};
    EncodedRecordId encoded_record_id{};
    SelectedConfigFingerprint selected_config{};
    RecordRepresentation representation = RecordRepresentation::Opaque;
    CodecEpochBinding codec{};
    EncryptionAlgorithm encryption = EncryptionAlgorithm::None;
    std::uint64_t original_size = 0;
    std::uint64_t encoded_size = 0;
    std::uint32_t fragment_count = 0;
};

// encoded_size is the reassembled representation byte count, excluding ALL
// envelope metadata/tags. For encrypted records it is 24-byte nonce || ciphertext;
// the separate 16-byte encryption_tag is here. Nonencrypted encryption_tag is empty.
// XChaCha20-Poly1305 is the one-shot crypto_aead_lock/unlock construction in
// third_party/monocypher (24-byte nonce, 32-byte key, 16-byte tag). Encryption key =
// keyed BLAKE2b-256(root, "IRIS-V2-ENCRYPT" || SessionId || u8 transfer.direction).
// The root is the locally configured key named by the activated authentication ID.
// Nonce = canonical u64 transfer_id || u64 encoded_record_id || eight zero bytes;
// IDs cannot repeat under that directional session key. AEAD associated data =
// "IRIS-V2-RECORD-AAD" || canonical identity || canonical fragments vector.
// This binds version, configuration, origin, lengths, codec/epochs and partition;
// it excludes ciphertext, encryption_tag, payload_integrity and integrity.
//
// Extents partition [0,encoded_size) exactly, in contiguous ordinal/offset order;
// length > 0 except encoded_size==0 requires exactly ONE marker extent (offset=0,
// length=0). Duplicate extents, gaps and overlaps inside an envelope are rejected.
// A network duplicate of an identical admitted object is idempotent, not a new
// allocation. Validate offset <= size && length <= size-offset before addition.
//
// payload_integrity uses EnvelopePayload over canonical identity, fragments,
// encryption_tag, then canonical byte string of the full encoded payload.
// integrity uses EncodedRecordEnvelope over preceding fields, INCLUDING the nested
// payload_integrity. Both are keyed Profile 1. Metadata authenticates BEFORE
// storage admission; full payload authenticates BEFORE inverse transform/commit.
struct EncodedRecordEnvelope {
    EncodedRecordEnvelopeIdentity identity{};
    std::vector<EncodedFragmentExtent> fragments{};
    std::vector<std::uint8_t> encryption_tag{};
    IntegrityInfo payload_integrity{};
    IntegrityInfo integrity{};
};

class LiveReceiveContext;
class LiveSession;
class AdmittedEncodedRecord {
public:
    const EncodedRecordEnvelope& envelope() const noexcept;
private:
    AdmittedEncodedRecord() = default;
    EncodedRecordEnvelope envelope_{};
    friend class EncodedRecordAdmission;
};
enum class EnvelopeValidationError : std::uint8_t {
    None = 0, InactiveSession, InvalidTransferIdentity, UnsupportedVersion,
    ConfigMismatch, UnsupportedRepresentation, CodecBindingMismatch,
    RepresentationCodecMismatch, RepresentationEncryptionMismatch,
    OriginalSizeLimitExceeded, EncodedSizeLimitExceeded, FragmentCountLimitExceeded,
    AggregateLimitExceeded, RecordCountLimitExceeded, MetadataLimitExceeded,
    InvalidFragmentExtent, DuplicateOrOverlappingExtent, IncompleteExtentCoverage,
    InvalidEncryptionMetadata, InvalidIntegrity, ArithmeticOverflow,
    ConflictingIdentity, TransformFailure, OriginalLengthMismatch,
};

// RC4's session-owned registry reserves storage atomically and retains the adopted
// envelope, original descriptor, reassembly coverage, completion and model commit
// state across flights. No free peer-supplied catalog/limit/key grants admission.
// Opaque requires None/None; Compressed requires codec; Encrypted requires cipher
// and codec None; CompressedThenEncrypted requires both. Non-None cipher equals
// the selected cipher. Unencrypted records are legal only under selected None.
// Checked budget sums use term <= limit-used, including declared output capacity.
class EncodedRecordAdmission {
public:
    static EnvelopeValidationError receive_metadata(
        LiveReceiveContext& receive, const EncodedRecordEnvelope& decoded) noexcept;
    static EnvelopeValidationError adopt_transmit(
        LiveSession& session, const EncodedRecordEnvelope& prepared,
        std::vector<std::uint8_t> encoded_bytes) noexcept;
    static EnvelopeValidationError validate_complete(
        LiveReceiveContext& receive, const TransferIdentity& transfer,
        EncodedRecordId record) noexcept;
};

// No fragment-to-original proportional mapping exists, even for OpaqueBytes1.
// Original credit is the whole record only after all its fragments (including an
// empty marker), envelope/payload integrity and inverse transform pass at RX.
// Partial transformed records earn zero original-byte or empty-record credit.
struct EncodedRecordProgress {
    EncodedRecordEnvelope envelope{};
    std::vector<EncodedFragmentExtent> acknowledged_extents{};
    bool complete_validated_at_receiver = false;
    bool model_committed = false;
};

enum class TransformStatus : std::uint8_t { Produced = 0, Buffered, NeedOutput, Failed };
// RC4 owns transform/drain implementation. Buffered/NeedOutput retain owned data;
// neither authorizes passthrough. consumed counts this call's input bytes only.
struct RecordTransformResult {
    TransformStatus status = TransformStatus::Buffered;
    std::uint64_t input_bytes_consumed = 0;
    std::vector<std::uint8_t> produced_bytes{};
    std::uint64_t retained_output_bytes = 0;
    EnvelopeValidationError error = EnvelopeValidationError::None;
};

} // namespace iris::v2
#endif // IRIS_V2_ENCODED_RECORD_H
