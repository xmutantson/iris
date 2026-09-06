#ifndef IRIS_V2_INTEGRITY_H
#define IRIS_V2_INTEGRITY_H

// Author: xmutantson

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace iris::v2 {

// Iris v2 Protection Profile 1 is the sole normative serialization/protection
// profile for these interfaces. Quoted derivation/AAD strings are ASCII octets
// without a length or NUL; named constexpr arrays use all their explicit octets.
// Unsigned integers and enum underlying values are
// fixed-width, unsigned, network-byte-order values. Arrays are raw octets. A
// variable byte string is u64 length followed by bytes; every other vector is a
// u32 element count followed by elements in declared canonical order. Booleans
// are one octet, 0 or 1. There is no padding, native-layout serialization,
// optional-field omission, or alternate ordering. Signed i32 values use two's
// complement network order. An optional is u8 presence (0/1) then its value when
// present. Nested structs recurse in declaration order; lengths/counts have their
// stated width, checked before allocation. Unknown enum values are rejected.
// Each protected object authenticates kProtectionPrefix || u8 ProtectionDomain ||
// u16 profile_version || u8 IntegrityAlgorithm || canonical covered fields.
// Covered fields exclude only the outer object's own IntegrityInfo. Decoders
// reject noncanonical encodings before verification.
inline constexpr std::array<std::uint8_t, 16> kProtectionPrefix{{
    'I', 'R', 'I', 'S', '-', 'V', '2', '-', 'P', 'R', 'O', 'T', 'E', 'C', 'T', 0}};
inline constexpr std::uint16_t kProtectionProfileVersion = 1;

enum class ProtectionDomain : std::uint8_t {
    HandshakeAdvertisement = 1,
    EncodedRecordEnvelope,
    EncodedFragment,
    Acknowledgment,
    CloseRequest,
    RemoteCloseAttestation,
    CloseAttestationReceipt,
    CloseConfirmation,
    FrameShape,
    HandshakeSelection,
    HandshakeConfirmation,
    EnvelopePayload,
    TransferDomainAdmission,
};

// Algorithm names are complete variants, not families. CRC outputs are emitted
// most-significant byte first. CRC-16/CCITT-FALSE uses poly 0x1021, init 0xffff,
// refin=false, refout=false, xorout=0. CRC-32C uses poly 0x1edc6f41,
// init/xorout=0xffffffff and refin/refout=true. BLAKE2b variants have a 32-byte
// output with standard BLAKE2b parameters, no salt and no personalization. The
// keyed variant uses a 32-byte directional session evidence key and is the only
// Profile-1 algorithm that grants authoritative peer evidence.
enum class IntegrityAlgorithm : std::uint8_t {
    None = 0,
    Crc16CcittFalse,
    Crc32c,
    Blake2b256,
    Blake2b256Keyed,
};

constexpr std::size_t integrity_value_length(IntegrityAlgorithm algorithm) noexcept {
    switch (algorithm) {
    case IntegrityAlgorithm::None:
        return 0;
    case IntegrityAlgorithm::Crc16CcittFalse:
        return 2;
    case IntegrityAlgorithm::Crc32c:
        return 4;
    case IntegrityAlgorithm::Blake2b256:
    case IntegrityAlgorithm::Blake2b256Keyed:
        return 32;
    }
    return 0;
}

constexpr bool is_authoritative_evidence_algorithm(
    IntegrityAlgorithm algorithm) noexcept {
    return algorithm == IntegrityAlgorithm::Blake2b256Keyed;
}

struct IntegrityInfo {
    IntegrityAlgorithm algorithm = IntegrityAlgorithm::None;
    ProtectionDomain domain = ProtectionDomain::EncodedRecordEnvelope;
    std::uint16_t profile_version = kProtectionProfileVersion;
    std::vector<std::uint8_t> value{};
};

struct ByteView {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
};

enum class IntegrityValidationError : std::uint8_t {
    None = 0,
    AlgorithmNotNegotiated,
    NoneNotPermitted,
    WrongDomain,
    WrongProfileVersion,
    WrongValueLength,
    NonCanonicalEncoding,
    VerificationFailed,
};

// Wire IntegrityInfo is u8 algorithm,u8 domain,u16 version,u64 value length,value.
// Nested IntegrityInfo fields ARE covered as ordinary fields of an outer object;
// only the outer object's own integrity is omitted. Every authoritative object
// (handshake, envelope metadata/payload, fragment, frame, ACK, close) requires the
// keyed algorithm. None is never an authoritative or selected bulk algorithm.
// Keys are selected from owned live-session state by actual message sender role;
// transfer.direction identifies data origin, which is opposite the ACK sender.
// The raw helper below verifies cryptography only: it cannot mint receive authority.
// Authoritative ACK and close evidence must use Blake2b256Keyed, the negotiated
// directional evidence key, and a current-session freshness field covered by the
// object (flight/close identity). None, CRC, and unkeyed hashes may protect
// diagnostics or bulk corruption checks but never authorize state mutation.
IntegrityValidationError validate_integrity(
    const IntegrityInfo& integrity,
    ProtectionDomain expected_domain,
    IntegrityAlgorithm negotiated_algorithm,
    ByteView canonical_covered_fields,
    ByteView directional_session_evidence_key,
    bool authoritative_evidence) noexcept;

static_assert(integrity_value_length(IntegrityAlgorithm::Crc16CcittFalse) == 2,
              "Profile-1 CRC-16 length is fixed");
static_assert(integrity_value_length(IntegrityAlgorithm::Blake2b256Keyed) == 32,
              "Profile-1 authentication tag length is fixed");
static_assert(!is_authoritative_evidence_algorithm(IntegrityAlgorithm::None),
              "unprotected evidence is never authoritative");

}  // namespace iris::v2

#endif  // IRIS_V2_INTEGRITY_H
