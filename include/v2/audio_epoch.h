#ifndef IRIS_V2_AUDIO_EPOCH_H
#define IRIS_V2_AUDIO_EPOCH_H

// Author: xmutantson
#include "v2/identity.h"

#include <cstdint>
#include <optional>
#include <memory>

namespace iris::v2 {

enum class ClockEvidenceKind : std::uint8_t {
    Unsupported = 0,
    PhysicalDevice,
    DeterministicTestClock,
};

enum class AudioBackendCapability : std::uint32_t {
    VerifiedCapturePosition = 1U << 0,
    VerifiedRenderPosition = 1U << 1,
    StreamEpoch = 1U << 2,
    DiscontinuityEvents = 1U << 3,
    DeviceStopEvents = 1U << 4,
    PumpStopEvents = 1U << 5,
    ErrorEvents = 1U << 6,
    TestClock = 1U << 7,
};

// Unsupported requires zero bits. PhysicalDevice forbids TestClock; verified
// position bits require StreamEpoch + discontinuity/device-stop/pump-stop/error
// event bits. DeterministicTestClock requires TestClock and the same verified
// stage/event guarantees. No test evidence may be relabeled PhysicalDevice.
struct AudioBackendCapabilitySet {
    std::uint32_t bits = 0;
    ClockEvidenceKind clock_evidence = ClockEvidenceKind::Unsupported;
};

// RC3 uses ONE shared duplex epoch for a paired capture/render device timeline.
// Allocate nonzero monotonically from a backend-owner lifetime counter on initial
// paired open and every discontinuity/restart of EITHER half. Opening halves
// separately does not activate a lease until both join the SAME new epoch. Any
// half's discontinuity invalidates BOTH, positions, mappings and end tokens.
// Exhaustion fails before reuse; a fresh backend owner gets a fresh ConnectionId.
// Epoch identity is (backend_owner,epoch), not the numeric counter alone.
struct AudioStreamEpoch {
    ConnectionId backend_owner{};
    std::uint64_t value = 0;
};

enum class AudioDirection : std::uint8_t {
    Capture = 0,
    Render,
    Duplex,
};

enum class DevicePositionStage : std::uint8_t {
    Captured = 0,
    Submitted,
    Rendered,
};

enum class DeviceEventType : std::uint8_t {
    CaptureDiscontinuity = 0,
    RenderDiscontinuity,
    DeviceStopped,
    PumpStopped,
    Error,
};

// A device frame is one simultaneous sample for every interleaved device channel,
// not one channel sample. next_frame is an exclusive boundary/count from frame
// zero, the first frame in epoch. Captured means made available by the capture
// device; Submitted means accepted by the render API; Rendered means physically
// played according to the device clock. Only Captured is valid with Capture and
// only Submitted/Rendered with Render. A valid value has nonzero epoch, rates,
// channel count and evidence equal to the live lease. Captured requires
// VerifiedCapturePosition; Rendered requires VerifiedRenderPosition. Submitted
// requires actual successful render-API acceptance in that lease, not a generated
// or queued position; no verified-render capability is inferred from submission.
// Unknown position is represented only by std::nullopt, never by zero or UINT64_MAX.
struct DeviceFramePosition {
    AudioStreamEpoch epoch{};
    AudioDirection direction = AudioDirection::Capture;
    DevicePositionStage stage = DevicePositionStage::Captured;
    std::uint64_t next_frame = 0;
    std::uint32_t frames_per_second = 0;
    std::uint16_t channel_count = 0;
    ClockEvidenceKind evidence = ClockEvidenceKind::Unsupported;
};

// A channel-sample boundary is explicitly distinct from a device-frame boundary.
// next_sample is exclusive from the first modem sample in the epoch. channel_index
// identifies which channel of the mapped device frame supplied/carried it.
struct ChannelSamplePosition {
    AudioStreamEpoch epoch{};
    std::uint64_t next_sample = 0;
    std::uint32_t samples_per_second = 0;
    std::uint16_t channel_index = 0;
};

// invalidated_epoch always names the old epoch affected by the event. A successful
// reopen may supply a distinct nonzero replacement_epoch; absence means none yet.
// All positions are the last verified exclusive progress in invalidated_epoch.
// direction describes the initiating fault; invalidation always affects the pair.
// submitted and rendered progress are separate and never substituted for one
// another. Events travel on a control path independent of the failed callback.
struct DeviceEvent {
    DeviceEventType type = DeviceEventType::Error;
    AudioDirection direction = AudioDirection::Duplex;
    AudioStreamEpoch invalidated_epoch{};
    std::optional<AudioStreamEpoch> replacement_epoch{};
    std::optional<DeviceFramePosition> captured_position{};
    std::optional<DeviceFramePosition> submitted_position{};
    std::optional<DeviceFramePosition> rendered_position{};
    std::int32_t backend_error = 0;
    std::uint64_t monotonic_event_time_ns = 0;
};

// RC3 owns a live, noncopyable backend lease. open/start must succeed on both
// halves; stop or capability loss increments generation and invalidates it before
// notification. Session activation compares the actual lease, not a capabilities
// value copied earlier. Snapshot position evidence must equal this lease's kind.
class AudioBackendOwner;
class AudioBackendLease {
public:
    ~AudioBackendLease();
    const AudioBackendCapabilitySet& capabilities() const noexcept;
    AudioStreamEpoch epoch() const noexcept;
    std::uint64_t generation() const noexcept;
    bool opened_and_started() const noexcept;
    AudioBackendLease(const AudioBackendLease&) = delete;
    AudioBackendLease& operator=(const AudioBackendLease&) = delete;
private:
    AudioBackendLease();
    struct State;
    std::unique_ptr<State> state_;
    friend class AudioBackendOwner;
};

// A committed mapping segment records actual converter/reblocker output ownership,
// including rational resampler phase, origins, filter flush/delay and partial
// writes. Rates alone cannot derive this mapping. Each producer chunk has ordered
// nonoverlapping intervals; output end includes every delayed frame carrying any
// payload contribution, excludes later silence, and is not rounded down. Shared
// boundary frames are permitted across chunks and must be represented explicitly.
// RC3 owns converter-specific phase/filter arithmetic behind this exact-extent API.
struct ProducerDeviceMappingSegment {
    std::uint64_t mapping_id = 0; // monotonic nonzero per shared epoch, no reuse
    AudioStreamEpoch epoch{};
    std::uint64_t producer_begin = 0;
    std::uint64_t producer_end_exclusive = 0;
    std::uint32_t producer_rate_hz = 0;
    std::uint64_t device_begin = 0;
    std::uint64_t device_end_exclusive = 0;
    std::uint32_t device_rate_hz = 0;
    std::uint16_t device_channel_count = 0;
    std::uint16_t device_channel_index = 0;
    bool converter_tail_flushed = false;
};
class ProducerDeviceMappingRegistry;
// RC3 owns the epoch-scoped registry, populated by actual conversion/adoption,
// not by callers computing rate ratios. Tokens can only be issued after adopting
// the current final producer chunk AND flushing its converter tail. Registry
// retains exact segment coverage through submitted and rendered progress.
class PayloadEndSampleToken {
public:
    const TransferIdentity& transfer() const noexcept;
    AudioStreamEpoch epoch() const noexcept;
    std::uint64_t device_frame_end_exclusive() const noexcept;
private:
    PayloadEndSampleToken() = default;
    TransferIdentity transfer_{};
    AudioStreamEpoch epoch_{};
    std::uint64_t mapping_id_ = 0;
    std::uint64_t final_producer_chunk_id_ = 0;
    std::uint64_t modem_sample_end_exclusive_ = 0;
    std::uint64_t device_frame_end_exclusive_ = 0;
    std::uint64_t backend_generation_ = 0;
    friend class ProducerDeviceMappingRegistry;
};

class ProducerDeviceMappingRegistry {
public:
    ~ProducerDeviceMappingRegistry();
    ProducerDeviceMappingRegistry(const ProducerDeviceMappingRegistry&) = delete;
    ProducerDeviceMappingRegistry& operator=(const ProducerDeviceMappingRegistry&) = delete;
    std::optional<PayloadEndSampleToken> seal_payload_end(
        const TransferIdentity& transfer, std::uint64_t final_producer_chunk_id,
        const AudioBackendLease& backend) noexcept;
private:
    ProducerDeviceMappingRegistry();
    struct State; // RC3: actual segments, producer chunk ownership and flush state.
    std::unique_ptr<State> state_;
    friend class AudioBackendOwner;
};

enum class AudioPositionValidationError : std::uint8_t {
    None = 0,
    UnsupportedEvidence,
    MissingCapability,
    InvalidEpoch,
    InvalidDirectionForStage,
    InvalidRateOrChannel,
    EpochMismatch,
    RateOrChannelMismatch,
    PositionBeforePayloadEnd,
    ArithmeticOverflow,
    CapabilityEvidenceMismatch,
    StaleBackendLease,
    MappingMismatch,
    ConverterTailNotFlushed,
};

AudioPositionValidationError validate_device_frame_position(
    const DeviceFramePosition& position,
    const AudioBackendLease& backend) noexcept;
AudioPositionValidationError validate_payload_end_token(
    const PayloadEndSampleToken& token,
    const ProducerDeviceMappingRegistry& mappings,
    const AudioBackendLease& backend) noexcept;
AudioPositionValidationError validate_device_event(
    const DeviceEvent& event,
    const AudioBackendLease& backend) noexcept;

// Rendering is confirmed iff a valid Rendered position has the same epoch,
// evidence, rate and channel geometry as token and
// rendered.next_frame >= token.device_frame_end_exclusive. Submitted progress is
// categorically insufficient for PTT release or response-deadline arming.
AudioPositionValidationError confirm_payload_rendered(
    const PayloadEndSampleToken& token,
    const DeviceFramePosition& rendered,
    const ProducerDeviceMappingRegistry& mappings,
    const AudioBackendLease& backend) noexcept;

// RC3 maps confirmed render time to the shared duplex capture timeline using the
// backend's measured correspondence. Deadline origin is at/after that event and
// at/after the capture boundary already consumed by the scheduler. No pre-render
// capture interval counts as listening. A missing cross-clock correspondence
// withholds timing authority; a watchdog may fail, never synthesize RF progress.
struct CaptureDeadlineOrigin {
    AudioStreamEpoch epoch{};
    std::uint64_t capture_frame_begin = 0;
    std::uint64_t backend_generation = 0;
};

}  // namespace iris::v2

#endif  // IRIS_V2_AUDIO_EPOCH_H
