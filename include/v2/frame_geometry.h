#ifndef IRIS_V2_FRAME_GEOMETRY_H
#define IRIS_V2_FRAME_GEOMETRY_H

// Author: xmutantson
#include "fec/ldpc.h"
#include "v2/encoded_record.h"
#include "v2/identity.h"
#include "v2/integrity.h"
#include "v2/session.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

namespace iris::v2 {

struct FrameProfileId {
    std::uint16_t value = 0;
};

// Protection Profile 1 serializes LdpcRate with this explicit one-octet mapping;
// it never serializes the compiler-selected underlying representation.
constexpr std::uint8_t canonical_ldpc_rate_id(LdpcRate rate) noexcept {
    switch (rate) {
    case LdpcRate::NONE: return 0;
    case LdpcRate::RATE_1_16: return 1;
    case LdpcRate::RATE_2_16: return 2;
    case LdpcRate::RATE_3_16: return 3;
    case LdpcRate::RATE_4_16: return 4;
    case LdpcRate::RATE_5_16: return 5;
    case LdpcRate::RATE_6_16: return 6;
    case LdpcRate::RATE_1_2: return 8;
    case LdpcRate::RATE_5_8: return 10;
    case LdpcRate::RATE_3_4: return 12;
    case LdpcRate::RATE_7_8: return 14;
    }
    return 0xff;
}

// NFFT is even. Bins use signed, fftshift order [-NFFT/2,NFFT/2), with DC=0;
// all three bin vectors are strictly increasing in that order. pilot and data are
// disjoint, nonempty subsets whose exact union is used; DC is never used. The bit
// loading vector is one-to-one with data bins. tone_map_id is not an arbitrary
// label: it must equal the ID in the matching admitted profile and may not name
// different contents within one selected configuration.
struct FrameToneMap {
    std::vector<std::int32_t> used_carrier_bins{};
    std::vector<std::int32_t> pilot_carrier_bins{};
    std::vector<std::int32_t> data_carrier_bins{};
    std::vector<std::uint8_t> coded_bits_per_data_carrier{};
    std::uint16_t tone_map_id = 0;
    bool use_non_uniform_constellations = false;
};

struct FrameGeometryInput {
    FrameProfileId profile_id{};
    FrameToneMap tone_map{};
    LdpcRate fec = LdpcRate::NONE;
    std::uint8_t codeword_count = 0;
    std::uint32_t nfft = 0;
    std::uint32_t cyclic_prefix_samples = 0;
    std::uint32_t pilot_row_spacing = 0;
};

// A supported profile fixes every waveform field except codeword_count, for which
// it admits the inclusive range [min,max]. Profiles are strictly ordered by
// nonzero unique profile_id. Repeated tone_map_id is permitted only for byte-equal
// tone maps (e.g. different FEC); one ID never names two maps in the admitted set.
// RC6 owns the registered profile table, including exact FEC matrix and constellation
// labeling. Existing LdpcRate values use include/fec/ldpc.h's block_size and matrix;
// unimplemented rates and NONE are rejected for this 1600-bit-codeword profile.
struct AdmittedFrameProfile {
    FrameProfileId profile_id{};
    FrameToneMap tone_map{};
    LdpcRate fec = LdpcRate::NONE;
    std::uint8_t min_codeword_count = 0;
    std::uint8_t max_codeword_count = 0;
    std::uint32_t nfft = 0;
    std::uint32_t cyclic_prefix_samples = 0;
    std::uint32_t pilot_row_spacing = 0;
    std::uint32_t timing_margin_samples = 0;
};

// This is the one admitted set shared by TX, RX hypothesis enumeration, capture
// retention and reporting. authoritative_sample_rate_hz is the sole divisor for
// airtime: duration_seconds = complete_sample_count / rate. All declared
// maxima must equal the checked maximum over every profile at its maximum admitted
// codeword count; a locally chosen smaller subset is not valid for receive bounds.
struct AdmittedFrameProfileSet {
    SelectedConfigFingerprint selected_config{};
    std::uint32_t authoritative_sample_rate_hz = 0;
    std::vector<AdmittedFrameProfile> profiles{};
    std::uint64_t maximum_data_symbol_count = 0;
    std::uint64_t maximum_complete_sample_count = 0;
    std::uint64_t maximum_first_training_remaining_samples = 0;
    std::uint32_t maximum_timing_margin_samples = 0;
    std::uint32_t acquisition_overlap_samples = 0;
};

enum class FrameGeometryError : std::uint8_t {
    None = 0,
    InvalidProfileSet,
    InvalidAuthoritativeSampleRate,
    UnsupportedProfile,
    ToneMapIdMismatch,
    ProfileDefinitionMismatch,
    UnsupportedFec,
    InvalidCodewordCount,
    InvalidFftOrCyclicPrefix,
    InvalidCarrierPlacement,
    CarrierOrderViolation,
    DuplicateCarrier,
    CarrierSetRelationshipViolation,
    ToneMapLengthMismatch,
    UnsupportedConstellation,
    ZeroCodedBitsPerDataSymbol,
    SharedMaximumMismatch,
    ArithmeticOverflow,
};

struct FrameGeometryValidation {
    FrameGeometryError error = FrameGeometryError::None;
};

FrameGeometryValidation validate_admitted_frame_profiles(
    const AdmittedFrameProfileSet& admitted) noexcept;

// Cross-validation requires the profile-set fingerprint, authoritative sample
// rate, maximum codeword count, and derived maxima to agree with the complete
// selected session configuration before any profile is admitted. frame_profiles
// fingerprint = BLAKE2b-256(canonical vector<AdmittedFrameProfile> || u32 rate ||
// u32 acquisition_overlap_samples); excludes selected_config and derived maxima,
// avoiding recursive fingerprints. Every permitted codeword count is enumerated.
// Legacy profiles use this same checked geometry without any session fingerprint:
// selected_config is zero there, and the RC6 table includes EVERY currently legal
// 1..8-codeword shape, not just the active ladder. Receive storage bounds cover the
// union of legacy and active v2 sets, checked maximum over all admitted shapes.
// maximum_first_training_remaining_samples = max((4+D+P)*L + timing_margin).
// Retention additionally reserves acquisition_overlap_samples and retained earlier
// candidate context; no fixed callback or six-second limit substitutes for this.
FrameGeometryValidation validate_selected_frame_profiles(
    const AdmittedFrameProfileSet& admitted,
    const SelectedSessionConfig& selected) noexcept;

// Checked construction is the only way to obtain geometry. validate requires an
// exact admitted-profile match, not merely syntactic validity. The constructor
// throws std::invalid_argument when validate would fail and never exposes partial
// derived fields. Callers that cannot throw must use validate before construction.
class FrameGeometry {
public:
    static constexpr std::uint32_t kCodedBitsPerCodeword = 1600;
    static constexpr std::uint32_t kFixedNonDataSymbols = 5;
    static constexpr std::uint32_t kFixedSymbolsFromFirstTraining = 4;
    static constexpr std::uint8_t kMinCodewordCount = 1;
    static constexpr std::uint8_t kMaxCodewordCount = 8;

    static FrameGeometryValidation validate(
        const FrameGeometryInput& input,
        const AdmittedFrameProfileSet& admitted) noexcept;

    explicit FrameGeometry(const FrameGeometryInput& input,
                           const AdmittedFrameProfileSet& admitted);

    FrameProfileId profile_id() const noexcept { return profile_id_; }
    const FrameToneMap& tone_map() const noexcept { return tone_map_; }
    LdpcRate fec() const noexcept { return fec_; }
    std::uint8_t codeword_count() const noexcept { return codeword_count_; }
    std::uint32_t nfft() const noexcept { return nfft_; }
    std::uint32_t cyclic_prefix_samples() const noexcept {
        return cyclic_prefix_samples_;
    }
    std::uint32_t pilot_row_spacing() const noexcept { return pilot_row_spacing_; }
    std::uint32_t authoritative_sample_rate_hz() const noexcept {
        return authoritative_sample_rate_hz_;
    }
    std::uint64_t coded_bits_per_data_symbol() const noexcept {
        return coded_bits_per_data_symbol_;
    }
    std::uint64_t data_symbol_count() const noexcept { return data_symbol_count_; }
    std::uint64_t pilot_rows() const noexcept { return pilot_rows_; }
    std::uint64_t symbol_length_samples() const noexcept {
        return symbol_length_samples_;
    }
    std::uint64_t complete_sample_count() const noexcept {
        return complete_sample_count_;
    }
    std::uint32_t timing_margin_samples() const noexcept {
        return timing_margin_samples_;
    }

private:
    FrameProfileId profile_id_{};
    FrameToneMap tone_map_{};
    LdpcRate fec_ = LdpcRate::NONE;
    std::uint8_t codeword_count_ = 0;
    std::uint32_t nfft_ = 0;
    std::uint32_t cyclic_prefix_samples_ = 0;
    std::uint32_t pilot_row_spacing_ = 0;
    std::uint32_t authoritative_sample_rate_hz_ = 0;
    std::uint64_t coded_bits_per_data_symbol_ = 0;  // B
    std::uint64_t data_symbol_count_ = 0;            // D = ceil(1600*C/B)
    std::uint64_t pilot_rows_ = 0;                   // P = S?floor((D-1)/S):0
    std::uint64_t symbol_length_samples_ = 0;        // L = NFFT + CP
    std::uint64_t complete_sample_count_ = 0;        // (5 + D + P) * L
    std::uint32_t timing_margin_samples_ = 0;
};

struct EncodedFrameExtent {
    OriginalRecordId original_record_id{};
    std::uint64_t original_total_length = 0;
    EncodedRecordId encoded_record_id{};
    std::uint64_t encoded_offset = 0;
    std::uint64_t encoded_length = 0;
    std::uint64_t encoded_total_length = 0;
};

// Keyed FrameShape protection covers ALL preceding fields in declaration order.
// One v2 DATA frame transports one whole immutable fragment (including an empty
// marker). Extent/flight/ordinal must exactly equal that protected fragment and
// its full envelope. No new fragmentation during retries: another admitted shape
// may carry the SAME object only if frame-fit below succeeds; otherwise fail RC1.
// frame_id is fresh per transmission; reusing it requires byte-identical shape.
// exact_information_bit_length is 8 * COMPLETE TRANSPORT BYTES, not encoded_length.
struct ProtectedFrameShapeClaim {
    TransferIdentity transfer{};
    FrameId frame_id{};
    FlightId flight_id{};
    FragmentOrdinal fragment_ordinal{};
    SelectedConfigFingerprint selected_config{};
    EncodedFrameExtent extent{};
    FrameProfileId profile_id{};
    std::uint16_t tone_map_id = 0;
    LdpcRate fec = LdpcRate::NONE;
    std::uint8_t codeword_count = 0;
    std::uint64_t exact_information_bit_length = 0;
    IntegrityInfo integrity{};
};

// Normative v2 transport bytes: literal octets "IRV2", u16 version=1, canonical
// ProtectedFrameShapeClaim INCLUDING tag, then canonical DecodedFragmentClaim
// INCLUDING complete envelope, envelope payload/metadata tags, nonce in payload,
// encryption tag and fragment tag. No optional omission, out-of-band metadata,
// repeated hidden headers or native struct padding. There are no other fields.
// Concatenation is split into the existing per-LDPC-block format below. The shape
// length covers the entire concatenation including itself; its fixed-width length
// field and fixed tag lengths make sizing nonrecursive. Tags are generated after
// all extents/counts/lengths are fixed. Empty marker payload still has wire metadata.
inline constexpr std::uint16_t kV2FrameTransportVersion = 1;
struct V2FrameTransport {
    ProtectedFrameShapeClaim shape{};
    DecodedFragmentClaim fragment{};
};
struct FrameWireBudget {
    std::uint64_t information_bits_per_codeword = 0; // K = block_size(fec)
    std::uint64_t block_payload_capacity_bytes = 0; // floor(K/8)-6
    std::uint64_t total_transport_capacity_bytes = 0; // C * per-block capacity
    std::uint64_t serialized_shape_bytes = 0; // includes own tag
    std::uint64_t serialized_fragment_metadata_bytes = 0; // all but payload bytes
    std::uint64_t encoded_fragment_payload_bytes = 0;
    std::uint64_t total_transport_bytes = 0; // 6 + all three terms above
};
// RC6 owns this sole TX/RX/frame-fit/MAX_INFO byte accounting entry point. Reject
// overflow, metadata limits, or total > capacity. Payload allowance is capacity
// minus EXACT serialized overhead, with subtraction only after checking overhead.
// RC4 fixes envelope partition against this function before encryption/tagging.
// Six per-block bytes (u16 length + u32 CRC) were already subtracted in capacity;
// the six-byte v2 magic/version prefix is separate and counted exactly once.
std::optional<FrameWireBudget> checked_frame_wire_budget(
    const FrameGeometry& geometry, const V2FrameTransport& transport) noexcept;

enum class FrameShapeValidationError : std::uint8_t {
    None = 0, IdentityMismatch, ConfigMismatch, ExtentOutOfBounds,
    EnvelopeExtentMismatch, GeometryMismatch, InformationLengthOutOfBounds,
    IntegrityNotAuthoritative, IntegrityFailure, InactiveSession,
    FrameIdentityConflict, WireBudgetMismatch,
};
class ValidatedFrameShape {
public:
    const ProtectedFrameShapeClaim& claim() const noexcept { return claim_; }
    const FrameGeometry& geometry() const noexcept { return geometry_; }
private:
    ValidatedFrameShape(ProtectedFrameShapeClaim claim, FrameGeometry geometry)
        : claim_(std::move(claim)), geometry_(std::move(geometry)) {}
    ProtectedFrameShapeClaim claim_{};
    FrameGeometry geometry_;
    friend class FrameShapeValidator;
};
// Decode complete trial geometry first, then validate protected shape against that
// geometry, owned envelope/fragment registry and live peer context. Never require
// a validated on-air declaration to size the first decode (a circular dependency).
class FrameShapeValidator {
public:
    static std::optional<ValidatedFrameShape> validate(
        LiveReceiveContext& receive, const V2FrameTransport& decoded,
        const FrameGeometry& trial_geometry,
        FrameShapeValidationError& error) noexcept;
};

// RC2 owns the framing selector at the session/transport ingress: basic/legacy
// links select Legacy, authenticated active v2 DATA links select ProtectedV2.
// Payload magic alone never upgrades legacy traffic. Failed v2 protection never
// falls back to Legacy on that v2 link; other retained legacy candidates remain.
enum class FramingPath : std::uint8_t { Legacy = 0, ProtectedV2 = 1 };
// RC6 enumerates complete legacy hypotheses independently of v2 negotiation. The
// first-training candidate plus admitted profile, timing/CFO alternative and C
// identifies a hypothesis;
// C is not inferred from a valid shorter payload prefix or CRC-8. This checked
// object requires no session, protected envelope or v2 declaration.
class LegacyFrameHypothesis {
public:
    const FrameGeometry& geometry() const noexcept;
    std::uint64_t expected_decoded_information_bits() const noexcept; // C*K
private:
    explicit LegacyFrameHypothesis(FrameGeometry geometry);
    FrameGeometry geometry_;
    friend class RetainedFrameCandidates;
};

// BOTH paths preserve source/ofdm/ofdm_mod.cc build_ofdm_frame block bytes:
// u16 payload BYTE length LITTLE ENDIAN || payload || u32 CRC LITTLE ENDIAN ||
// zero padding to K information bits. Bytes enter the LDPC information bit vector
// least-significant bit first. CRC-32/ISO-HDLC: poly 0x04c11db7 (reflected
// 0xedb88320), init/xorout 0xffffffff, refin/refout true, over the TWO LENGTH BYTES
// PLUS PAYLOAD. No separate length CRC and no CRC-32C. Every block, INCLUDING an
// empty trailing block, must contain and pass this CRC; empty first block rejects.
// All C blocks have EXACTLY K binary bits, indices 0..C-1, no extra/residual bits.
// Legacy total payload length is learned after all blocks pass; it is not a v2
// declaration. V2 additionally requires summed payload bytes*8 to equal protected
// exact_information_bit_length and the canonical transport to consume all bytes.
struct DecodedInformationBlock {
    std::uint8_t codeword_index = 0;
    std::vector<std::uint8_t> information_bits{};
};
struct ValidatedInformationBlock {
    std::uint8_t codeword_index = 0;
    std::uint16_t payload_byte_length = 0;
    std::vector<std::uint8_t> payload_bytes{};
};
enum class FrameCandidateRejection : std::uint8_t {
    InvalidShape = 0, SampleExtentOverflow, CompleteSampleExtentMissing,
    WrongCodewordCount, MissingOrDuplicateCodeword, PartialInformationBlock,
    NonBinaryInformationBit, BlockLengthOutOfBounds, PayloadCrcFailure,
    NonzeroPadding, ExactInformationLengthMismatch, InvalidV2Protection,
    StaleCandidate, AmbiguousLegacyHypotheses,
};

// RC6 owns retained candidate geometry/completion state; RC7/RC8 own chronological
// timing/CFO hypothesis arbitration through this explicit retained-state seam.
// Handle identity includes input epoch and registry generation, never naked offset.
class RetainedCandidateHandle {
private:
    RetainedCandidateHandle() = default;
    std::uint64_t candidate_id_ = 0;
    std::uint64_t registry_generation_ = 0;
    friend class RetainedFrameCandidates;
    friend class CompleteFrameExtractor;
};
class FrameCandidateOwner;
class RetainedFrameCandidates {
public:
    ~RetainedFrameCandidates();
    RetainedFrameCandidates(const RetainedFrameCandidates&) = delete;
    RetainedFrameCandidates& operator=(const RetainedFrameCandidates&) = delete;
private:
    RetainedFrameCandidates();
    // RC7/RC8 call these through FrameCandidateOwner after acquisition owns the
    // epoch/absolute sample interval; geometry enumeration is exclusively RC6.
    RetainedCandidateHandle retain_first_training(
        AudioStreamEpoch epoch, std::uint64_t absolute_first_training_start,
        const AdmittedFrameProfileSet& admitted, FramingPath path);
    LegacyFrameHypothesis make_legacy_hypothesis(FrameGeometry geometry);
    struct State;
    std::unique_ptr<State> state_;
    friend class CompleteFrameExtractor;
    friend class FrameCandidateOwner;
};
// State owns first-training starts, complete admitted hypothesis sets, required
// right timing margins, full-waveform/tail checks for each geometry, unresolved
// alternatives/next candidates and a safe prefix
// retirement watermark. RC7 creates/populates it through its acquisition owner.
// The reference is the FIRST sample of FIRST TRAINING (after the leading symbol).
// Required input end = first_training_offset + (4+D+P)*L + timing_margin.
// Actual validated frame end excludes right-context margin. Both additions are
// checked. Retention keeps left acquisition overlap too. A noise/work-budget yield
// never discards unresolved candidates. Rejection may retire only the registry's
// safe prefix, not guessed candidate length that could cross a later candidate.

// Every samples_consumed is a count from this call's input BEGIN, <= input size.
// NeedMoreSamples retires only the safe prefix strictly before unresolved samples;
// additional_samples_required = max(0, required input end - available count).
// Zero additional samples means hypotheses still need fair resumable work, not
// permission to discard. RejectedCandidate consumes at most the safe watermark.
// Completion consumes through actual validated frame end; registry must first
// resolve all earlier/overlapping candidate hypotheses that this would retire.
struct NeedMoreSamples {
    std::uint64_t samples_consumed = 0;
    std::uint64_t additional_samples_required = 0;
};
struct RejectedCandidate {
    std::uint64_t samples_consumed = 0;
    FrameCandidateRejection reason = FrameCandidateRejection::InvalidShape;
};
using ValidatedFraming = std::variant<LegacyFrameHypothesis, ValidatedFrameShape>;
class CompleteValidatedFrame {
public:
    std::uint64_t samples_consumed() const noexcept { return samples_consumed_; }
    const ValidatedFraming& framing() const noexcept { return framing_; }
    const std::vector<ValidatedInformationBlock>& blocks() const noexcept;
    const std::vector<std::uint8_t>& transport_bytes() const noexcept;
private:
    CompleteValidatedFrame(std::uint64_t consumed, ValidatedFraming framing,
                           std::vector<ValidatedInformationBlock> blocks,
                           std::vector<std::uint8_t> transport);
    std::uint64_t samples_consumed_ = 0;
    ValidatedFraming framing_;
    std::vector<ValidatedInformationBlock> blocks_{};
    std::vector<std::uint8_t> transport_bytes_{};
    friend class CompleteFrameExtractor;
};
using FrameCompletionResult =
    std::variant<NeedMoreSamples, RejectedCandidate, CompleteValidatedFrame>;
struct FrameExtractionInput {
    AudioStreamEpoch epoch{};
    std::uint64_t input_begin_absolute_sample = 0;
    std::uint64_t available_sample_count = 0;
    // EXACT decoded LDPC information length = C*K, including length/CRC/padding;
    // authoritative C/K come from the retained admitted hypothesis, not this claim.
    std::uint64_t decoded_information_bit_length = 0;
    std::vector<DecodedInformationBlock> decoded_blocks{};
};
class CompleteFrameExtractor {
public:
    static FrameCompletionResult extract_legacy(
        RetainedFrameCandidates& retained, const RetainedCandidateHandle& candidate,
        const FrameExtractionInput& input) noexcept;
    static FrameCompletionResult extract_v2(
        RetainedFrameCandidates& retained, const RetainedCandidateHandle& candidate,
        LiveReceiveContext& receive, const FrameExtractionInput& input) noexcept;
};
// With complete required samples, incomplete/residual blocks REJECT, never publish
// a prefix or wait for a nonexistent extra block. Privately validate all C blocks.
// Legacy successes stay provisional in retained state until every admitted count
// alternative at that candidate is resolved. If unequal complete lengths remain
// valid after full waveform/tail-boundary checks, report ambiguity without
// delivery; a shorter block-CRC prefix alone is not a complete waveform hypothesis.
// Never claim the shortest is known
// complete. V2 also validates the entire transport and shape against trial C.
// CompleteValidatedFrame is PHY completion only: legacy CRC grants no v2 ACK/close
// authority, and v2 frame completion still requires RC4 whole-record transformation.

static_assert(FrameGeometry::kMinCodewordCount == 1, "one-codeword frames are legal");
static_assert(FrameGeometry::kMaxCodewordCount == 8, "eight-codeword frames are legal");

} // namespace iris::v2
#endif // IRIS_V2_FRAME_GEOMETRY_H
