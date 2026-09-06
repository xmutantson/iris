#ifndef IRIS_V2_SESSION_H
#define IRIS_V2_SESSION_H

// Author: xmutantson
#include "v2/audio_epoch.h"
#include "v2/encoded_record.h"
#include "v2/identity.h"
#include "v2/integrity.h"

#include <array>
#include <cstdint>
#include <optional>
#include <memory>
#include <utility>
#include <vector>

namespace iris::v2 {

// Only (major=2,minor=0) is accepted. No implicit minor compatibility/downgrade.
struct ProtocolVersion {
    std::uint16_t major = 0;
    std::uint16_t minor = 0;
};

enum class V2Capability : std::uint64_t {
    CustodyR1 = 1ULL << 0,
    ProtectedIdentity = 1ULL << 1,
    ExtendedFrame = 1ULL << 2,
    RecordCodec = 1ULL << 3,
    VerifiedDeviceClock = 1ULL << 4,
    TestClock = 1ULL << 5,
};

struct V2CapabilitySet {
    std::uint64_t bits = 0;
};

// AuthenticationKeyId is BLAKE2b-256 over the exact 32-byte configured root;
// the root itself is never serialized in negotiation.
struct AuthenticationKeyId {
    std::array<std::uint8_t, 32> bytes{};
};

struct FrameProfileSetFingerprint {
    std::array<std::uint8_t, 32> bytes{};
};

inline constexpr std::array<std::uint8_t, 14> kConfigDerivationPrefix{{
    'I', 'R', 'I', 'S', '-', 'V', '2', '-', 'C', 'O', 'N', 'F', 'I', 'G'}};
inline constexpr std::array<std::uint8_t, 15> kSessionDerivationPrefix{{
    'I', 'R', 'I', 'S', '-', 'V', '2', '-', 'S', 'E', 'S', 'S', 'I', 'O', 'N'}};
inline constexpr std::array<std::uint8_t, 16> kEvidenceDerivationPrefix{{
    'I', 'R', 'I', 'S', '-', 'V', '2', '-', 'E', 'V', 'I', 'D', 'E', 'N', 'C', 'E'}};

// The selected configuration is a value, not merely a peer-supplied digest.
// Its canonical Profile-1 encoding is in declaration order below. The
// codec_bindings vector is sorted by its complete canonical byte encoding,
// without duplicates, at its declared position. selected_config is
// BLAKE2b-256(kConfigDerivationPrefix || encoding).
// frame_profiles is BLAKE2b-256 over the canonical ordered admitted-profile set.
// Both endpoints must reproduce the value and digest exactly before activation.
struct SelectedSessionConfig {
    ProtocolVersion protocol_version{};
    V2CapabilitySet capabilities{};
    IntegrityAlgorithm evidence_integrity = IntegrityAlgorithm::None;
    IntegrityAlgorithm bulk_integrity = IntegrityAlgorithm::None;
    EncryptionAlgorithm encryption_algorithm = EncryptionAlgorithm::None;
    AuthenticationKeyId authentication_key{};
    EnvelopeAdmissionLimits envelope_limits{};
    std::vector<CodecEpochBinding> codec_bindings{};
    FrameProfileSetFingerprint frame_profiles{};
    std::uint32_t authoritative_sample_rate_hz = 0;
    std::uint8_t max_frame_codeword_count = 0;
    std::uint8_t ack_modulo_bits = 0;
    std::uint32_t max_outstanding_fragments = 0;
};

// A valid selection requires evidence_integrity=Blake2b256Keyed,
// bulk_integrity=Blake2b256Keyed, a nonzero authentication key ID, valid nonzero envelope
// limits, max_frame_codeword_count in [1,8], ack_modulo_bits in [1,31], and
// max_outstanding_fragments in [1,2^(ack_modulo_bits-1)]. Invalid selections do not
// activate any authoritative v2 feature.

// Advertisements describe SUPPORTED sets, not the agreed selection. Vectors are
// canonical byte-sorted, duplicate-free, nonempty (except optional codecs); bits
// outside the known mask are invalid. Limits advertise maxima. RC2 owns the
// capability exchange carrier encoding; its payload is exactly Profile-1 encoding
// of these objects, no separately interpreted negotiation fields.
struct SupportedSessionCapabilities {
    std::vector<ProtocolVersion> versions{};
    V2CapabilitySet capabilities{};
    std::vector<IntegrityAlgorithm> evidence_algorithms{};
    std::vector<IntegrityAlgorithm> bulk_algorithms{};
    std::vector<EncryptionAlgorithm> encryption_algorithms{};
    std::vector<AuthenticationKeyId> authentication_keys{};
    EnvelopeAdmissionLimits envelope_maxima{};
    std::vector<CodecEpochBinding> codec_bindings{};
    std::vector<FrameProfileSetFingerprint> frame_profile_sets{};
    std::vector<std::uint32_t> sample_rates_hz{};
    std::uint8_t max_frame_codeword_count = 0;
    std::vector<std::uint8_t> ack_modulo_bits{};
    std::uint32_t max_outstanding_fragments = 0;
};
enum class EndpointRole : std::uint8_t { Initiator = 0, Responder = 1 };
// Initiator nonce is freshly allocated before offer. Responder echoes it and adds
// its own fresh nonce; offer uses responder_nonce zero. The authenticated selected
// transcript below includes both full offers, preventing offer substitution.
struct V2HandshakeCapabilities {
    EndpointRole sender = EndpointRole::Initiator;
    ConnectionNonce initiator_nonce{};
    ConnectionNonce responder_nonce{};
    SupportedSessionCapabilities supported{};
    IntegrityInfo integrity{}; // HandshakeAdvertisement, configured root key.
};
// Initiator proposes a supported intersection; responder explicitly accepts EXACT
// selected bytes or rejects. No algorithmic tie-breaking is implicit. Scalars and
// each codec/profile/version must be supported by both, limits <= both maxima,
// capability bits a subset of intersection, plus mandatory CustodyR1,
// ProtectedIdentity, ExtendedFrame and exactly one eligible clock capability.
// OpaqueBytes1 is mandatory; RecordCodec is required for any nonopaque codec.
struct HandshakeSelection {
    V2HandshakeCapabilities initiator_offer{};
    V2HandshakeCapabilities responder_offer{};
    SelectedSessionConfig selected{};
    IntegrityInfo integrity{}; // HandshakeSelection, initiator root-key tag.
};
struct HandshakeConfirmation {
    std::array<std::uint8_t, 32> selection_digest{};
    EndpointRole sender = EndpointRole::Responder;
    IntegrityInfo integrity{}; // HandshakeConfirmation, responder root-key tag.
};
// selection_digest = BLAKE2b-256(canonical full HandshakeSelection, tag included).
// Responder sends confirmation; initiator validates it and sends its role-tagged
// confirmation over the same digest. Each activates only after receiving the
// opposite role's confirmation, and retransmits its own idempotently until DATA.

// Session ID is the first 16 bytes of BLAKE2b-256 over
// kSessionDerivationPrefix || initiator_nonce || responder_nonce || selected_config,
// exactly in that role order. Each nonce is 128 random bits from the platform
// CSPRNG, nonzero, new for every handshake, and never restored from cache.
// An all-zero result or collision with any live/retained ID aborts activation for a
// new handshake; identities and evidence are never merged across the collision.
struct V2SessionIdentity {
    SessionId session_id{};
    ProtocolVersion protocol_version{};
    ConnectionNonce initiator_nonce{};
    ConnectionNonce responder_nonce{};
    SelectedConfigFingerprint selected_config{};
};

// The directional evidence key is BLAKE2b-256 keyed with the locally configured
// 32-byte authentication root named by AuthenticationKeyId, over
// kEvidenceDerivationPrefix || session_id || one-byte sender EndpointRole. Absence of
// that root or evidence_integrity != Blake2b256Keyed prevents authoritative-v2
// negotiation rather than falling back to IntegrityAlgorithm::None.
struct DirectionalEvidenceKey {
    EndpointRole sender = EndpointRole::Initiator;
    std::array<std::uint8_t, 32> bytes{};
};

enum class SessionValidationError : std::uint8_t {
    None = 0,
    UnsupportedVersion,
    InvalidNonce,
    ConfigFingerprintMismatch,
    NonCanonicalConfig,
    UnauthenticatedAdvertisement,
    MissingAuthenticationKey,
    SessionIdMismatch,
    SessionIdCollision,
    InvalidNegotiatedBounds,
    UnsupportedSelection,
    HandshakeStageMismatch,
    StaleBackendLease,
    UnsupportedBackend,
    RevokedCapability,
};

// RC2 owns these noncopyable state holders and their out-of-line implementations.
// HandshakeContext owns configured peer/root identity, issued nonce pair, pending
// offers/selection, received confirmations, replay/collision tombstones and the
// current RC3 backend lease. It is never reconstructed from decoded peer values.
class SessionOwner;
class LiveHandshakeContext {
public:
    ~LiveHandshakeContext();
    LiveHandshakeContext(const LiveHandshakeContext&) = delete;
    LiveHandshakeContext& operator=(const LiveHandshakeContext&) = delete;
private:
    LiveHandshakeContext();
    struct State;
    std::unique_ptr<State> state_;
    friend class SessionOwner;
    friend class SessionActivation;
};
class LiveSession {
public:
    ~LiveSession();
    LiveSession(const LiveSession&) = delete;
    LiveSession& operator=(const LiveSession&) = delete;
    const V2SessionIdentity& identity() const noexcept;
    bool active() const noexcept;
private:
    LiveSession();
    struct State;
    std::unique_ptr<State> state_;
    friend class SessionActivation;
    friend class ProtectedReceiveBoundary;
    friend class FragmentValidator;
    friend class AckValidator;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
    friend class CloseExchangeOwner;
    friend class EncodedRecordAdmission;
    friend class FrameShapeValidator;
    friend class SessionOwner;
    friend class TransferLedgerOwner;
    friend class CloseTransportOwner;
};
// State owns the selected config, peer role, secret directional keys, backend
// lease/generation, original and adopted encoded catalogs, fragment registry keyed
// ONLY by (TransferIdentity,FragmentOrdinal), all flight scopes, prior cumulative
// coverage, close exchanges and consumed-evidence tombstones. All are invalidated
// atomically on revocation before callbacks. State survives outstanding handles;
// an inactive handle cannot grant authority. Operations serialize through one
// owner transaction; validator results carry the revision at which they were made.
// Internal generation/revision/receive/authority counters are monotonic nonzero,
// never wrap; exhaustion revokes before reuse. RC1's registered ledger association
// binds the origin-relative R1/R2/R3 domain tuple once per transfer; decoded ACKs
// and close messages may only compare against it, never choose another endpoint.
// RC1 owns the authenticated transfer-domain admission control message; its covered
// payload is exactly TransferIdentity then ReliabilityDomainIdentities (Profile 1).

class SessionActivation {
public:
    static SessionValidationError receive_offer(
        LiveHandshakeContext& live, ByteView canonical_peer_offer) noexcept;
    static SessionValidationError receive_selection(
        LiveHandshakeContext& live, ByteView canonical_peer_selection) noexcept;
    static SessionValidationError receive_confirmation(
        LiveHandshakeContext& live, ByteView canonical_peer_confirmation) noexcept;
    static std::shared_ptr<LiveSession> activate(
        LiveHandshakeContext& live, const AudioBackendLease& current_backend,
        SessionValidationError& error) noexcept;
    static void revoke(LiveSession& live, const DeviceEvent& event) noexcept;
};
// activate verifies both offer tags and confirmations under the SAME locally owned
// root named by selected.authentication_key (also pinned by the handshake owner),
// role/nonces/selection digests, exact (2,0) version, all supported/selected
// bounds, registered codec/profile specs, fresh session ID and CURRENT opened,
// started backend lease. Physical evidence enables VerifiedDeviceClock only;
// DeterministicTestClock enables TestClock only with explicit local test policy.
// Unsupported or stopped backends cannot activate. Backend generation/epoch change
// or capability loss immediately revokes all authority and fails RC1 custody once;
// recovery requires a fresh handshake. Cached support is never current evidence.

// Owned receive transaction, created only by the actual transport receive entry
// point. It pins LiveSession and exact received canonical bytes plus peer role,
// session generation and one-use receive sequence. Validators must compare decoded
// claims to these bytes (or the registered frame subobject byte span), verify the
// peer-sender key, and check active state/replay registry under the owner lock.
// A public decoded object plus a key can never construct this context.
// For v2 DATA the owner stages canonical bytes only after complete block extraction;
// the extractor then compares every decoded field to the staged immutable bytes.
// Staging alone is untrusted and grants no payload, ACK or close side effects.
class LiveReceiveContext {
public:
    ~LiveReceiveContext();
    LiveReceiveContext(LiveReceiveContext&&) noexcept;
    LiveReceiveContext& operator=(LiveReceiveContext&&) noexcept;
    LiveReceiveContext(const LiveReceiveContext&) = delete;
    LiveReceiveContext& operator=(const LiveReceiveContext&) = delete;
private:
    LiveReceiveContext();
    std::shared_ptr<LiveSession> session_;
    std::vector<std::uint8_t> received_bytes_{};
    std::uint64_t generation_ = 0;
    std::uint64_t receive_sequence_ = 0;
    friend class ProtectedReceiveBoundary;
    friend class FragmentValidator;
    friend class AckValidator;
    friend class CloseReceiveValidator;
    friend class CloseProofValidator;
    friend class EncodedRecordAdmission;
    friend class FrameShapeValidator;
    friend class TransportReceiveOwner;
};
// RC2 owns transport-to-ProtectedReceiveBoundary wiring and canonical control
// dispatch; RC6 stages complete block bytes here before protected DATA validation.
class TransportReceiveOwner;
class ProtectedReceiveBoundary {
private:
    static std::optional<LiveReceiveContext> receive(
        std::shared_ptr<LiveSession> session,
        std::vector<std::uint8_t> canonical_received_bytes) noexcept;
    friend class TransportReceiveOwner;
    friend class CompleteFrameExtractor;
};

SessionValidationError validate_selected_session_config(
    const SelectedSessionConfig& config,
    const SelectedConfigFingerprint& fingerprint) noexcept;

struct FragmentIdentity {
    TransferIdentity transfer{};
    FlightId flight_id{};
    FragmentOrdinal absolute_ordinal{};
    EncodedRecordId encoded_record_id{};
    std::uint64_t encoded_offset = 0;
    std::uint64_t encoded_length = 0;
};

// This is decoded, peer-controlled input and grants no retry or ACK authority.
struct DecodedFragmentClaim {
    FragmentIdentity identity{};
    EncodedRecordEnvelope envelope{};
    std::vector<std::uint8_t> payload{};
    IntegrityInfo integrity{};
};

enum class FragmentValidationError : std::uint8_t {
    None = 0,
    TransferMismatch,
    EnvelopeMismatch,
    ExtentMismatch,
    PayloadLengthMismatch,
    FragmentNotDeclared,
    IntegrityNotAuthoritative,
    IntegrityFailure,
    InactiveSession,
    OrdinalOutsideAdmittedFlight,
    IdentityAlreadyBoundToDifferentBytes,
};

// Only FragmentValidator can create this value. Protection Profile 1 covers the
// FragmentIdentity fields, the COMPLETE envelope including both integrity tags,
// payload length, and exact payload bytes, in that order. Its identity offset and
// length must equal the unique envelope extent for absolute_ordinal; payload.size
// equals both lengths. Once admitted, any repeat of the identity must be byte-for-
// byte and envelope-for-envelope equal or is a protocol violation. Implementations
// retain this object (or an immutable ownership reference to it) for every retry.
class ProtectedFragment {
public:
    const FragmentIdentity& identity() const noexcept { return claim_.identity; }
    const EncodedRecordEnvelope& envelope() const noexcept {
        return claim_.envelope;
    }
    const std::vector<std::uint8_t>& payload() const noexcept { return claim_.payload; }
    const IntegrityInfo& integrity() const noexcept { return claim_.integrity; }

private:
    explicit ProtectedFragment(DecodedFragmentClaim claim) : claim_(std::move(claim)) {}
    DecodedFragmentClaim claim_{};
    friend class FragmentValidator;
};

// Admission atomically looks up/inserts the session-owned ordinal registry. Equal
// duplicates return the existing binding without side effects; any change to
// flight, encoded ID, extent, FULL envelope or bytes under an ordinal is fatal.
// The registry retains bindings/tombstones until session disposition; a retry uses
// the same object. TX adoption uses the same registry through RC2's session owner.
class FragmentValidator {
public:
    static std::optional<ProtectedFragment> validate(
        LiveReceiveContext& receive,
        const DecodedFragmentClaim& claim,
        FragmentValidationError& error) noexcept;
};

// A live flight is the nonempty half-open ordinal range [first_fragment,
// fragment_limit); both are nonzero, first < limit, and subtraction (never
// addition) is used to validate its negotiated fragment-count bound.
struct FlightScope {
    TransferIdentity transfer{};
    FlightId flight_id{};
    FragmentOrdinal first_fragment{};
    FragmentOrdinal fragment_limit{};
};

enum class EncodedBoundaryPosition : std::uint8_t {
    BeforeFirst = 0,
    InRecord,
    AfterFinal,
    AtAdmittedEnd,
};

struct EncodedRecordBoundary {
    EncodedBoundaryPosition position = EncodedBoundaryPosition::BeforeFirst;
    EncodedRecordId encoded_record_id{};
    std::uint64_t next_encoded_offset = 0;
};

// Inclusive cursor interval [first_fragment,fragment_limit], although the flight
// itself is half-open. Use the owned complete encoded catalog and prior flights:
// below limit, cursor equals the next fragment start (BeforeFirst instead of
// InRecord(first,0) at the initial transfer boundary); at limit it equals the
// normalized successor after the last fragment (including a zero-length marker).
// Boundary normalization and BeforeFirst/AtAdmittedEnd/AfterFinal rules are the
// original-catalog rules in identity.h applied to encoded sizes/IDs. Never infer
// the next record from the current-flight vector alone. Flights partition the
// allocated contiguous ordinal space; records may cross flights.
struct CumulativeFragmentBoundary {
    FragmentOrdinal next_fragment{};
    EncodedRecordBoundary encoded_boundary{};
};

enum class AckCarrier : std::uint8_t {
    SupervisoryFrame = 0,
    ExtendedSupervisoryFrame,
    InformationFramePiggyback,
    MfskTone,
    V2ControlFrame,
};

enum class AckIdentityCompleteness : std::uint8_t {
    AdvisoryModuloOnly = 0,
    CompleteProtectedIdentity,
};

struct ModuloAckValue {
    std::uint32_t value = 0;
    std::uint8_t modulo_bits = 0;
};

// This is untrusted decoded input. In particular, original_record_mapping is a
// peer claim and cannot validate itself. Acknowledgment Profile-1 protection
// covers every field before integrity in declaration order; optional wire_modulo
// is encoded as a one-byte presence flag followed, when present, by its fields.
struct DecodedAckClaim {
    SessionId session_id{};
    R2DomainIdentity r2{};
    TransferIdentity transfer{};
    FlightScope flight{};
    CumulativeFragmentBoundary cumulative_boundary{};
    OriginalRecordBoundary cumulative_original_boundary{};
    OriginalRecordCoverage original_record_mapping{};
    std::vector<EncodedRecordId> complete_validated_records{};
    AckCarrier carrier = AckCarrier::V2ControlFrame;
    AckIdentityCompleteness identity_completeness =
        AckIdentityCompleteness::AdvisoryModuloOnly;
    std::optional<ModuloAckValue> wire_modulo{};
    IntegrityInfo integrity{};
};

// RC2 stores this state INSIDE LiveSession, populated from owned TX adoption,
// never from peer mappings. Catalogs include prior-flight records; progress is
// monotonic, normalized and retained when a flight retires. A complete-record ACK
// names strictly sorted encoded IDs fully covered by cumulative fragment progress.
// RX emits those IDs only after RC4 complete validation. Derive original credit
// as whole ranges (or explicit empty membership) of those records; partial records
// earn no original credit. Compare the peer's complete cumulative mapping and
// original cursor exactly with that derivation, including prior credit. The
// complete-record list is the full cumulative validated prefix, not only a delta.
// A flight remains outstanding for ACK validation until both its fragment coverage
// and whole-record credit obligations settle; fragment delivery alone cannot retire
// the context needed for a delayed complete-record ACK. Identical fragment cursor
// with newly complete record credit is advancing evidence, not a duplicate.
struct AckProgressSnapshot {
    TransferIdentity transfer{};
    R2DomainIdentity r2{};
    std::vector<OriginalRecordDescriptor> original_catalog{};
    std::vector<EncodedRecordProgress> encoded_catalog{};
    std::vector<FlightScope> flights{};
    CumulativeFragmentBoundary prior_boundary{};
    OriginalRecordCoverage prior_original_credit{};
    std::uint64_t revision = 0;
};
// Owned mutable registry is deliberately not constructible from this diagnostic
// snapshot. Validate below accesses it through the pinned live receive context.

enum class AckValidationError : std::uint8_t {
    None = 0,
    AdvisoryOnly,
    SessionMismatch,
    DirectionOrTransferMismatch,
    R2DomainMismatch,
    FlightNotOutstanding,
    BoundaryOutsideFlight,
    FragmentOffsetDisagreement,
    OriginalBoundaryInvalid,
    OriginalMappingMismatch,
    ModuloWindowTooLarge,
    ModuloReconstructionAmbiguous,
    IntegrityNotAuthoritative,
    IntegrityFailure,
    StaleRevision,
    DuplicateOrRegressiveProgress,
    InactiveSession,
};

// This type is the sole ACK side-effect authority. It can be produced only by
// AckValidator after all duplicate session fields agree, the flight is currently
// outstanding, the cumulative fragment/encoded cursors agree, the original
// mapping equals whole-record credit derived from the owned encoded catalog,
// and Profile-1 authoritative integrity verifies. Insufficiently identified MFSK
// tone ACKs always return AdvisoryOnly and may request status but cannot release
// bytes, update timers/rate, or count as delivery/desynchronization evidence.
class ValidatedAckAuthority {
public:
    ValidatedAckAuthority(ValidatedAckAuthority&&) noexcept;
    ValidatedAckAuthority& operator=(ValidatedAckAuthority&&) noexcept;
    ValidatedAckAuthority(const ValidatedAckAuthority&) = delete;
    ValidatedAckAuthority& operator=(const ValidatedAckAuthority&) = delete;
    const DecodedAckClaim& claim() const noexcept { return claim_; }

private:
    explicit ValidatedAckAuthority(DecodedAckClaim claim) : claim_(std::move(claim)) {}
    DecodedAckClaim claim_{};
    std::uint64_t session_generation_ = 0;
    std::uint64_t expected_revision_ = 0;
    std::uint64_t authority_id_ = 0;
    friend class AckValidator;
};

struct AckValidationResult {
    AckValidationError error = AckValidationError::AdvisoryOnly;
    std::optional<ValidatedAckAuthority> authority{};
};

class AckValidator {
public:
    static AckValidationResult validate(LiveReceiveContext& receive,
                                        const DecodedAckClaim& claim) noexcept;
    // Atomically rechecks generation/revision and consumes authority_id exactly
    // once while updating prior coverage and downstream effects. Copies/replays,
    // retired flights, regressive/identical ACKs produce no new authority. Modulo
    // is a consistency check only: <= half-space, exactly one outstanding boundary,
    // all duplicated fields equal. Modulo-only legacy/tone input stays advisory.
    static AckValidationError apply(LiveSession& session,
                                    ValidatedAckAuthority&& authority) noexcept;
};

}  // namespace iris::v2

#endif  // IRIS_V2_SESSION_H
