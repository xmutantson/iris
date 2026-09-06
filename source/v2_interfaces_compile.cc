// Author: xmutantson
#include "v2/contracts.h"

#include <type_traits>

// Compile-only guard for the shared v2 contracts. This translation unit has no
// runtime entry points and intentionally does not wire the definitions into any
// production path.
static_assert(std::is_default_constructible<iris::v2::TransferLedger>::value,
              "the RC1 ledger state must remain a usable value type");
static_assert(std::is_default_constructible<iris::v2::EncodedRecordEnvelope>::value,
              "the RC4 envelope must remain a usable value type");
static_assert(std::is_default_constructible<iris::v2::DecodedAckClaim>::value,
              "decoded ACK claims must remain carrier-neutral value types");
static_assert(!std::is_default_constructible<iris::v2::ValidatedAckAuthority>::value,
              "ACK authority must cross the validator boundary");
static_assert(!std::is_default_constructible<iris::v2::ProtectedFragment>::value,
              "protected fragments must cross the validator boundary");
static_assert(!std::is_default_constructible<iris::v2::MatchingCloseProof>::value,
              "close proof must not be locally manufacturable");
static_assert(!std::is_default_constructible<iris::v2::ReceivedCloseConfirmation>::value,
              "a locally populated close message is not received authority");
static_assert(std::is_default_constructible<iris::v2::DeviceEvent>::value,
              "the RC3 event contract must remain a usable value type");
static_assert(!std::is_default_constructible<iris::v2::FrameGeometry>::value,
              "RC6 geometry must cross its checked construction boundary");

static_assert(!std::is_default_constructible<iris::v2::LiveReceiveContext>::value,
              "receive authority requires the live transport owner");
static_assert(!std::is_copy_constructible<iris::v2::LiveSession>::value,
              "session registries have one owner");
static_assert(!std::is_copy_constructible<iris::v2::ValidatedAckAuthority>::value,
              "ACK authority must be consumed once");
static_assert(!std::is_default_constructible<iris::v2::PayloadEndSampleToken>::value,
              "payload tokens require the actual producer/device mapping");
static_assert(!std::is_default_constructible<iris::v2::CompleteValidatedFrame>::value,
              "frame completion must cross the complete extractor boundary");
static_assert(std::variant_size<iris::v2::FrameCompletionResult>::value == 3,
              "incomplete, rejected and complete frames remain distinct");
static_assert(std::variant_size<iris::v2::ValidatedFraming>::value == 2,
              "legacy and protected v2 framing both have completion paths");
using AckValidationBoundary = iris::v2::AckValidationResult (*)(
    iris::v2::LiveReceiveContext&, const iris::v2::DecodedAckClaim&) noexcept;
static_assert(std::is_same<decltype(&iris::v2::AckValidator::validate),
                           AckValidationBoundary>::value,
              "ACK validation must access owned live receive state");
