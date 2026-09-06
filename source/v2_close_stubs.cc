// Author: xmutantson
#include "v2/close_transport.h"
#include "v2/transfer_ledger.h"

#include "engine/modem.h"
#include "monocypher.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace iris::v2 {
namespace {

bool nonzero(const std::array<std::uint8_t, 16>& value) noexcept {
    return std::any_of(value.begin(), value.end(),
                       [](std::uint8_t byte) { return byte != 0; });
}

bool same_transfer(const TransferIdentity& a,
                   const TransferIdentity& b) noexcept {
    return a.session_id.bytes == b.session_id.bytes &&
           a.direction == b.direction &&
           a.transfer_id.value == b.transfer_id.value;
}

bool same_r3(const R3DomainIdentity& a,
             const R3DomainIdentity& b) noexcept {
    return a.remote_client_connection.bytes ==
               b.remote_client_connection.bytes &&
           a.sequence_space.value == b.sequence_space.value;
}

bool same_boundary(const FrozenFinalRecord& a,
                   const FrozenFinalRecord& b) noexcept {
    return a.empty_transfer == b.empty_transfer &&
           a.record_count == b.record_count &&
           a.final_record_id.value == b.final_record_id.value &&
           a.final_record_extent == b.final_record_extent &&
           a.catalog_digest == b.catalog_digest;
}

void append_be(std::vector<std::uint8_t>& out, std::uint64_t value,
               unsigned width) {
    for (unsigned i = width; i != 0; --i)
        out.push_back(static_cast<std::uint8_t>(value >> ((i - 1) * 8)));
}

void append_transfer(std::vector<std::uint8_t>& out,
                     const TransferIdentity& transfer) {
    out.insert(out.end(), transfer.session_id.bytes.begin(),
               transfer.session_id.bytes.end());
    out.push_back(static_cast<std::uint8_t>(transfer.direction));
    append_be(out, transfer.transfer_id.value, 8);
}

void append_r3(std::vector<std::uint8_t>& out,
               const R3DomainIdentity& r3) {
    out.insert(out.end(), r3.remote_client_connection.bytes.begin(),
               r3.remote_client_connection.bytes.end());
    append_be(out, r3.sequence_space.value, 8);
}

void append_boundary(std::vector<std::uint8_t>& out,
                     const FrozenFinalRecord& boundary) {
    out.push_back(boundary.empty_transfer ? 1 : 0);
    append_be(out, boundary.record_count, 8);
    append_be(out, boundary.final_record_id.value, 8);
    append_be(out, boundary.final_record_extent, 8);
    out.insert(out.end(), boundary.catalog_digest.begin(),
               boundary.catalog_digest.end());
}

template <typename Message>
std::vector<std::uint8_t> covered_message(const Message& message,
                                          const FrozenFinalRecord& boundary) {
    std::vector<std::uint8_t> out;
    out.reserve(130);
    append_transfer(out, message.transfer);
    append_r3(out, message.r3);
    append_boundary(out, boundary);
    append_be(out, message.close_id, 8);
    out.insert(out.end(), message.challenge.bytes.begin(),
               message.challenge.bytes.end());
    return out;
}

std::vector<std::uint8_t> covered(const CloseRequest& message) {
    return covered_message(message, message.expected_final_record);
}

template <typename Message>
std::vector<std::uint8_t> covered(const Message& message) {
    return covered_message(message, message.accepted_final_record);
}

void append_integrity(std::vector<std::uint8_t>& out,
                      const IntegrityInfo& integrity) {
    out.push_back(static_cast<std::uint8_t>(integrity.algorithm));
    out.push_back(static_cast<std::uint8_t>(integrity.domain));
    append_be(out, integrity.profile_version, 2);
    append_be(out, integrity.value.size(), 8);
    out.insert(out.end(), integrity.value.begin(), integrity.value.end());
}

template <typename Message>
std::vector<std::uint8_t> full_wire(const Message& message) {
    auto out = covered(message);
    append_integrity(out, message.integrity);
    return out;
}

FrozenFinalRecord freeze_catalog(const TransferLedger& ledger) {
    FrozenFinalRecord boundary;
    boundary.empty_transfer = ledger.original_records.empty();
    boundary.record_count = ledger.original_records.size();
    if (!ledger.original_records.empty()) {
        boundary.final_record_id = ledger.original_records.back().record_id;
        boundary.final_record_extent = ledger.original_records.back().bytes.size();
    }

    std::vector<std::uint8_t> bytes;
    static constexpr std::uint8_t prefix[] = {
        'I','R','I','S','-','V','2','-','C','A','T','A','L','O','G'};
    bytes.insert(bytes.end(), std::begin(prefix), std::end(prefix));
    append_transfer(bytes, ledger.transfer);
    append_be(bytes, ledger.original_records.size(), 4);
    for (const auto& record : ledger.original_records) {
        append_be(bytes, record.record_id.value, 8);
        append_be(bytes, record.bytes.size(), 8);
    }
    crypto_blake2b(boundary.catalog_digest.data(),
                   boundary.catalog_digest.size(), bytes.data(), bytes.size());
    return boundary;
}

bool coverage_complete(const OriginalRecordCoverage& coverage,
                       const TransferLedger& ledger) noexcept {
    for (const auto& record : ledger.original_records) {
        if (record.bytes.empty()) {
            if (std::none_of(
                    coverage.completed_empty_records.begin(),
                    coverage.completed_empty_records.end(),
                    [&](OriginalRecordId id) {
                        return id.value == record.record_id.value;
                    }))
                return false;
            continue;
        }
        const bool covered_all = std::any_of(
            coverage.ranges.begin(), coverage.ranges.end(),
            [&](const OriginalRecordRange& range) {
                return range.record_id.value == record.record_id.value &&
                       range.offset == 0 &&
                       range.length == record.bytes.size();
            });
        if (!covered_all)
            return false;
    }
    return true;
}

EndpointRole origin_role(const TransferIdentity& transfer) noexcept {
    return transfer.direction == TransferDirection::InitiatorToResponder
        ? EndpointRole::Initiator : EndpointRole::Responder;
}

EndpointRole opposite(EndpointRole role) noexcept {
    return role == EndpointRole::Initiator
        ? EndpointRole::Responder : EndpointRole::Initiator;
}

std::size_t role_index(EndpointRole role) noexcept {
    return role == EndpointRole::Initiator ? 0U : 1U;
}

enum class ExchangeStage : std::uint8_t {
    RequestSent = 0,
    RequestReceived,
    AttestationSent,
    AttestationReceived,
    ReceiptSent,
    ReceiptReceived,
    ConfirmationSent,
    ConfirmationReceived,
    Consumed,
};

struct TransferAssociation {
    TransferIdentity transfer{};
    ReliabilityDomainIdentities domains{};
    TransferLedger* origin_ledger = nullptr;
    TransferLedger* receiver_ledger = nullptr;
};

struct ExchangeState {
    TransferAssociation* association = nullptr;
    CloseRequest request{};
    ExchangeStage stage = ExchangeStage::RequestSent;
    std::uint64_t revision = 0;
    std::uint64_t attestation_receive_sequence = 0;
    std::uint64_t confirmation_receive_sequence = 0;
};

struct NegotiatedState {
    std::mutex mutex{};
    std::string initiator_call{};
    std::string responder_call{};
    V2SessionIdentity identity{};
    std::array<DirectionalEvidenceKey, 2> evidence_keys{};
    std::array<bool, 2> endpoint_present{{false, false}};
    std::atomic<bool> active{true};
    std::uint64_t next_close_id = 1;
    std::uint64_t next_revision = 1;
    std::deque<TransferAssociation> transfers{};
    std::deque<ExchangeState> exchanges{};
};

std::mutex broker_mutex;
std::vector<std::weak_ptr<NegotiatedState>> broker_sessions;

std::shared_ptr<NegotiatedState> negotiate_session(
    const std::string& initiator_call, const std::string& responder_call,
    EndpointRole local_role) {
    std::lock_guard<std::mutex> lock(broker_mutex);
    broker_sessions.erase(
        std::remove_if(broker_sessions.begin(), broker_sessions.end(),
                       [](const auto& weak) { return weak.expired(); }),
        broker_sessions.end());

    const std::size_t index = role_index(local_role);
    for (auto& weak : broker_sessions) {
        auto shared = weak.lock();
        if (!shared || !shared->active.load() ||
            shared->initiator_call != initiator_call ||
            shared->responder_call != responder_call ||
            shared->endpoint_present[index])
            continue;
        if (local_role == EndpointRole::Responder &&
            !shared->endpoint_present[role_index(EndpointRole::Initiator)])
            continue;
        shared->endpoint_present[index] = true;
        return shared;
    }

    auto shared = std::make_shared<NegotiatedState>();
    shared->initiator_call = initiator_call;
    shared->responder_call = responder_call;
    shared->endpoint_present[index] = true;
    shared->identity.protocol_version = {2, 0};

    const auto nonce_material = crypto_random_key();
    std::copy_n(nonce_material.begin(), 16,
                shared->identity.initiator_nonce.bytes.begin());
    std::copy_n(nonce_material.begin() + 16, 16,
                shared->identity.responder_nonce.bytes.begin());

    std::vector<std::uint8_t> selected;
    append_be(selected, 2, 2);
    append_be(selected, 0, 2);
    append_be(selected,
              static_cast<std::uint64_t>(V2Capability::CustodyR1) |
              static_cast<std::uint64_t>(V2Capability::ProtectedIdentity) |
              static_cast<std::uint64_t>(V2Capability::ExtendedFrame) |
              static_cast<std::uint64_t>(V2Capability::TestClock), 8);
    selected.push_back(static_cast<std::uint8_t>(
        IntegrityAlgorithm::Blake2b256Keyed));
    std::vector<std::uint8_t> config_input;
    config_input.insert(config_input.end(), kConfigDerivationPrefix.begin(),
                        kConfigDerivationPrefix.end());
    config_input.insert(config_input.end(), selected.begin(), selected.end());
    crypto_blake2b(shared->identity.selected_config.bytes.data(),
                   shared->identity.selected_config.bytes.size(),
                   config_input.data(), config_input.size());

    std::vector<std::uint8_t> session_input;
    session_input.insert(session_input.end(), kSessionDerivationPrefix.begin(),
                         kSessionDerivationPrefix.end());
    session_input.insert(session_input.end(),
                         shared->identity.initiator_nonce.bytes.begin(),
                         shared->identity.initiator_nonce.bytes.end());
    session_input.insert(session_input.end(),
                         shared->identity.responder_nonce.bytes.begin(),
                         shared->identity.responder_nonce.bytes.end());
    session_input.insert(session_input.end(), selected.begin(), selected.end());
    std::array<std::uint8_t, 32> session_hash{};
    crypto_blake2b(session_hash.data(), session_hash.size(),
                   session_input.data(), session_input.size());
    std::copy_n(session_hash.begin(), 16,
                shared->identity.session_id.bytes.begin());

    const auto root = crypto_random_key();
    for (EndpointRole role : {EndpointRole::Initiator,
                              EndpointRole::Responder}) {
        auto& key = shared->evidence_keys[role_index(role)];
        key.sender = role;
        std::vector<std::uint8_t> input;
        input.insert(input.end(), kEvidenceDerivationPrefix.begin(),
                     kEvidenceDerivationPrefix.end());
        input.insert(input.end(), shared->identity.session_id.bytes.begin(),
                     shared->identity.session_id.bytes.end());
        input.push_back(static_cast<std::uint8_t>(role));
        crypto_blake2b_keyed(key.bytes.data(), key.bytes.size(), root.data(),
                             root.size(), input.data(), input.size());
    }

    broker_sessions.push_back(shared);
    return shared;
}

TransferAssociation* find_transfer(NegotiatedState& state,
                                   const TransferIdentity& transfer) noexcept {
    auto it = std::find_if(
        state.transfers.begin(), state.transfers.end(),
        [&](const TransferAssociation& association) {
            return same_transfer(association.transfer, transfer);
        });
    return it == state.transfers.end() ? nullptr : &*it;
}

ExchangeState* find_exchange(NegotiatedState& state,
                             const TransferIdentity& transfer) noexcept {
    auto it = std::find_if(
        state.exchanges.begin(), state.exchanges.end(),
        [&](const ExchangeState& exchange) {
            return exchange.association &&
                   same_transfer(exchange.association->transfer, transfer);
        });
    return it == state.exchanges.end() ? nullptr : &*it;
}

IntegrityInfo protect(ProtectionDomain domain,
                      const std::vector<std::uint8_t>& fields,
                      const DirectionalEvidenceKey& key) {
    IntegrityInfo integrity;
    integrity.algorithm = IntegrityAlgorithm::Blake2b256Keyed;
    integrity.domain = domain;
    integrity.profile_version = kProtectionProfileVersion;
    integrity.value.resize(32);

    std::vector<std::uint8_t> authenticated;
    authenticated.reserve(kProtectionPrefix.size() + 4 + fields.size());
    authenticated.insert(authenticated.end(), kProtectionPrefix.begin(),
                         kProtectionPrefix.end());
    authenticated.push_back(static_cast<std::uint8_t>(domain));
    append_be(authenticated, kProtectionProfileVersion, 2);
    authenticated.push_back(static_cast<std::uint8_t>(
        IntegrityAlgorithm::Blake2b256Keyed));
    authenticated.insert(authenticated.end(), fields.begin(), fields.end());
    crypto_blake2b_keyed(integrity.value.data(), integrity.value.size(),
                         key.bytes.data(), key.bytes.size(), authenticated.data(),
                         authenticated.size());
    return integrity;
}

template <typename Message>
bool message_matches_request(const Message& message,
                             const CloseRequest& request) noexcept {
    return same_transfer(message.transfer, request.transfer) &&
           same_r3(message.r3, request.r3) &&
           same_boundary(message.accepted_final_record,
                         request.expected_final_record) &&
           message.close_id == request.close_id &&
           message.challenge.bytes == request.challenge.bytes;
}

bool authoritative_integrity(const IntegrityInfo& integrity,
                             ProtectionDomain domain) noexcept {
    return integrity.algorithm == IntegrityAlgorithm::Blake2b256Keyed &&
           integrity.domain == domain &&
           integrity.profile_version == kProtectionProfileVersion &&
           integrity.value.size() == 32;
}

}  // namespace

struct LiveSession::State {
    std::shared_ptr<NegotiatedState> negotiated{};
    EndpointRole local_role = EndpointRole::Initiator;
    std::uint64_t generation = 1;
    std::uint64_t next_receive_sequence = 1;
    std::set<std::uint64_t> validated_receive_sequences{};
    Modem* owner = nullptr;
};

IntegrityValidationError validate_integrity(
    const IntegrityInfo& integrity, ProtectionDomain expected_domain,
    IntegrityAlgorithm negotiated_algorithm, ByteView canonical_covered_fields,
    ByteView directional_session_evidence_key,
    bool authoritative_evidence) noexcept {
    try {
        if (integrity.algorithm != negotiated_algorithm)
            return IntegrityValidationError::AlgorithmNotNegotiated;
        if (integrity.algorithm == IntegrityAlgorithm::None)
            return IntegrityValidationError::NoneNotPermitted;
        if (integrity.domain != expected_domain)
            return IntegrityValidationError::WrongDomain;
        if (integrity.profile_version != kProtectionProfileVersion)
            return IntegrityValidationError::WrongProfileVersion;
        if (integrity.value.size() != integrity_value_length(integrity.algorithm))
            return IntegrityValidationError::WrongValueLength;
        if (authoritative_evidence &&
            (!is_authoritative_evidence_algorithm(integrity.algorithm) ||
             directional_session_evidence_key.size != 32))
            return IntegrityValidationError::NoneNotPermitted;
        if (!canonical_covered_fields.data && canonical_covered_fields.size != 0)
            return IntegrityValidationError::NonCanonicalEncoding;

        std::vector<std::uint8_t> authenticated;
        authenticated.insert(authenticated.end(), kProtectionPrefix.begin(),
                             kProtectionPrefix.end());
        authenticated.push_back(static_cast<std::uint8_t>(expected_domain));
        append_be(authenticated, kProtectionProfileVersion, 2);
        authenticated.push_back(static_cast<std::uint8_t>(integrity.algorithm));
        if (canonical_covered_fields.size != 0)
            authenticated.insert(
                authenticated.end(), canonical_covered_fields.data,
                canonical_covered_fields.data + canonical_covered_fields.size);

        std::array<std::uint8_t, 32> expected{};
        if (integrity.algorithm == IntegrityAlgorithm::Blake2b256Keyed) {
            crypto_blake2b_keyed(
                expected.data(), expected.size(),
                directional_session_evidence_key.data,
                directional_session_evidence_key.size, authenticated.data(),
                authenticated.size());
        } else if (integrity.algorithm == IntegrityAlgorithm::Blake2b256) {
            crypto_blake2b(expected.data(), expected.size(), authenticated.data(),
                           authenticated.size());
        } else {
            return IntegrityValidationError::AlgorithmNotNegotiated;
        }
        return crypto_verify32(expected.data(), integrity.value.data()) == 0
            ? IntegrityValidationError::None
            : IntegrityValidationError::VerificationFailed;
    } catch (...) {
        return IntegrityValidationError::VerificationFailed;
    }
}

LiveSession::LiveSession() : state_(std::make_unique<State>()) {}
LiveSession::~LiveSession() = default;

bool LiveSession::active() const noexcept {
    return state_ && state_->negotiated &&
           state_->negotiated->active.load();
}

const V2SessionIdentity& LiveSession::identity() const noexcept {
    static const V2SessionIdentity invalid{};
    return active() ? state_->negotiated->identity : invalid;
}

LiveSession* CloseTransportOwner::session(Modem& modem) noexcept {
    try {
        std::lock_guard<std::recursive_mutex> modem_lock(modem.modem_mutex_);
        if (modem.v2_live_session_ && modem.v2_live_session_->active())
            return modem.v2_live_session_.get();
        if (!modem.v2_negotiated_active_ ||
            (modem.ofdm_kiss_peer_caps_ & CAP_OFDM) == 0)
            return nullptr;

        const EndpointRole local_role = modem.ax25_session_.we_initiated()
            ? EndpointRole::Initiator : EndpointRole::Responder;
        const std::string local = modem.config_.callsign;
        const std::string remote = modem.ax25_session_.remote_callsign();
        if (local.empty() || remote.empty())
            return nullptr;
        const std::string initiator = local_role == EndpointRole::Initiator
            ? local : remote;
        const std::string responder = local_role == EndpointRole::Responder
            ? local : remote;

        auto live = std::shared_ptr<LiveSession>(new LiveSession());
        live->state_->negotiated =
            negotiate_session(initiator, responder, local_role);
        live->state_->local_role = local_role;
        live->state_->owner = &modem;
        if (!nonzero(live->state_->negotiated->identity.session_id.bytes))
            return nullptr;
        modem.v2_live_session_ = std::move(live);
        return modem.v2_live_session_.get();
    } catch (...) {
        return nullptr;
    }
}

bool CloseTransportOwner::register_origin_transfer(
    Modem& modem, TransferLedger& ledger) noexcept {
    try {
        auto* live = session(modem);
        if (!live || !live->active())
            return false;
        auto shared = live->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        ledger.transfer.session_id = shared->identity.session_id;
        ledger.domains.r2.modem_session = shared->identity.session_id;
        if (find_transfer(*shared, ledger.transfer))
            return false;
        shared->transfers.push_back({ledger.transfer, ledger.domains, &ledger,
                                     nullptr});
        return true;
    } catch (...) {
        return false;
    }
}

bool CloseTransportOwner::register_receiver_transfer(
    Modem& modem, TransferLedger& ledger) noexcept {
    try {
        auto* live = session(modem);
        if (!live || !live->active())
            return false;
        auto shared = live->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        const TransferDirection wanted =
            live->state_->local_role == EndpointRole::Responder
            ? TransferDirection::InitiatorToResponder
            : TransferDirection::ResponderToInitiator;
        auto it = std::find_if(
            shared->transfers.begin(), shared->transfers.end(),
            [&](const TransferAssociation& association) {
                return association.transfer.direction == wanted &&
                       association.origin_ledger &&
                       !association.receiver_ledger;
            });
        if (it == shared->transfers.end())
            return false;
        ledger.transfer = it->transfer;
        ledger.domains.r2 = it->domains.r2;
        ledger.domains.r3 = it->domains.r3;
        ledger.r3_sequence.endpoint_cursor.domain = ledger.domains.r3;
        it->receiver_ledger = &ledger;
        return true;
    } catch (...) {
        return false;
    }
}

void CloseTransportOwner::revoke(Modem& modem) noexcept {
    std::lock_guard<std::recursive_mutex> modem_lock(modem.modem_mutex_);
    revoke(modem, modem.v2_live_session_);
}

void CloseTransportOwner::revoke(
    Modem& modem,
    const std::shared_ptr<LiveSession>& captured_session) noexcept {
    std::lock_guard<std::recursive_mutex> modem_lock(modem.modem_mutex_);
    if (!captured_session)
        return;
    if (captured_session->state_ &&
        captured_session->state_->negotiated) {
        std::lock_guard<std::mutex> lock(
            captured_session->state_->negotiated->mutex);
        captured_session->state_->negotiated->active.store(false);
    }
    if (modem.v2_live_session_ == captured_session) {
        modem.v2_live_session_.reset();
        modem.v2_negotiated_active_ = false;
    }
}

LiveReceiveContext::LiveReceiveContext() = default;
LiveReceiveContext::~LiveReceiveContext() = default;

LiveReceiveContext::LiveReceiveContext(LiveReceiveContext&& other) noexcept
    : session_(std::move(other.session_)),
      received_bytes_(std::move(other.received_bytes_)),
      generation_(std::exchange(other.generation_, 0)),
      receive_sequence_(std::exchange(other.receive_sequence_, 0)) {
    other.received_bytes_.clear();
    other.session_.reset();
}

LiveReceiveContext& LiveReceiveContext::operator=(
    LiveReceiveContext&& other) noexcept {
    if (this != &other) {
        session_ = std::move(other.session_);
        received_bytes_ = std::move(other.received_bytes_);
        generation_ = std::exchange(other.generation_, 0);
        receive_sequence_ = std::exchange(other.receive_sequence_, 0);
        other.received_bytes_.clear();
        other.session_.reset();
    }
    return *this;
}

std::optional<LiveReceiveContext> ProtectedReceiveBoundary::receive(
    std::shared_ptr<LiveSession> session,
    std::vector<std::uint8_t> canonical_received_bytes) noexcept {
    try {
        if (!session || !session->active() || canonical_received_bytes.empty() ||
            !session->state_ ||
            session->state_->next_receive_sequence ==
                std::numeric_limits<std::uint64_t>::max())
            return std::nullopt;
        LiveReceiveContext receive;
        receive.session_ = std::move(session);
        receive.received_bytes_ = std::move(canonical_received_bytes);
        receive.generation_ = receive.session_->state_->generation;
        receive.receive_sequence_ =
            receive.session_->state_->next_receive_sequence++;
        return receive;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<LiveReceiveContext> TransportReceiveOwner::receive_close_control(
    Modem& modem, std::vector<std::uint8_t> canonical_received_bytes) noexcept {
    std::lock_guard<std::recursive_mutex> modem_lock(modem.modem_mutex_);
    if (!CloseTransportOwner::session(modem))
        return std::nullopt;
    return ProtectedReceiveBoundary::receive(
        modem.v2_live_session_, std::move(canonical_received_bytes));
}

#define IRIS_MOVE_RECEIVED(Type)                                               \
Type::Type(Type&& other) noexcept                                              \
    : message_(std::move(other.message_)),                                     \
      session_generation_(std::exchange(other.session_generation_, 0)),       \
      exchange_revision_(std::exchange(other.exchange_revision_, 0)),         \
      receive_sequence_(std::exchange(other.receive_sequence_, 0)) {}          \
Type& Type::operator=(Type&& other) noexcept {                                 \
    if (this != &other) {                                                      \
        message_ = std::move(other.message_);                                  \
        session_generation_ = std::exchange(other.session_generation_, 0);     \
        exchange_revision_ = std::exchange(other.exchange_revision_, 0);       \
        receive_sequence_ = std::exchange(other.receive_sequence_, 0);         \
    }                                                                          \
    return *this;                                                              \
}

IRIS_MOVE_RECEIVED(ReceivedCloseRequest)
IRIS_MOVE_RECEIVED(ReceivedRemoteCloseAttestation)
IRIS_MOVE_RECEIVED(ReceivedCloseAttestationReceipt)
IRIS_MOVE_RECEIVED(ReceivedCloseConfirmation)
#undef IRIS_MOVE_RECEIVED

std::optional<CloseRequest> CloseExchangeOwner::begin(
    LiveSession& session, TransferLedger& ledger,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::NoOutstandingRequest;
    try {
        if (!session.active()) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = session.state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (session.state_->local_role != origin_role(ledger.transfer)) {
            error = CloseProofValidationError::WrongPeerOrRole;
            return std::nullopt;
        }
        auto* association = find_transfer(*shared, ledger.transfer);
        if (!association || association->origin_ledger != &ledger) {
            error = CloseProofValidationError::TransferMismatch;
            return std::nullopt;
        }
        if (find_exchange(*shared, ledger.transfer)) {
            error = CloseProofValidationError::WrongExchangeStage;
            return std::nullopt;
        }
        if (ledger.terminal_published || ledger.frozen_final_record ||
            !coverage_complete(ledger.milestones.air_acknowledged, ledger)) {
            error = CloseProofValidationError::CatalogNotFrozen;
            return std::nullopt;
        }
        const auto boundary = freeze_catalog(ledger);
        ledger.frozen_final_record = boundary;
        CloseRequest request;
        request.transfer = ledger.transfer;
        request.r3 = ledger.domains.r3;
        request.expected_final_record = boundary;
        if (shared->next_close_id == 0 ||
            shared->next_close_id == std::numeric_limits<std::uint64_t>::max()) {
            error = CloseProofValidationError::StaleRevision;
            return std::nullopt;
        }
        request.close_id = shared->next_close_id++;
        const auto random = crypto_random_key();
        std::copy_n(random.begin(), 16, request.challenge.bytes.begin());
        if (!nonzero(request.challenge.bytes)) {
            error = CloseProofValidationError::StaleOrReplayedChallenge;
            return std::nullopt;
        }
        request.integrity = protect(
            ProtectionDomain::CloseRequest, covered(request),
            shared->evidence_keys[role_index(session.state_->local_role)]);
        ExchangeState exchange;
        exchange.association = association;
        exchange.request = request;
        exchange.revision = shared->next_revision++;
        shared->exchanges.push_back(std::move(exchange));
        error = CloseProofValidationError::None;
        return request;
    } catch (...) {
        error = CloseProofValidationError::EvidenceNotAuthoritative;
        return std::nullopt;
    }
}

std::optional<RemoteCloseAttestation> CloseExchangeOwner::send_attestation(
    LiveSession& session, const TransferIdentity& transfer,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::NoOutstandingRequest;
    try {
        if (!session.active()) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = session.state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        auto* exchange = find_exchange(*shared, transfer);
        if (!exchange) return std::nullopt;
        if (session.state_->local_role != opposite(origin_role(transfer))) {
            error = CloseProofValidationError::WrongPeerOrRole;
            return std::nullopt;
        }
        if (exchange->stage != ExchangeStage::RequestReceived) {
            error = CloseProofValidationError::WrongExchangeStage;
            return std::nullopt;
        }
        auto* ledger = exchange->association->receiver_ledger;
        if (!ledger || !ledger->frozen_final_record ||
            !same_boundary(*ledger->frozen_final_record,
                           exchange->request.expected_final_record) ||
            !coverage_complete(ledger->milestones.endpoint_acknowledged,
                               *ledger) ||
            !ledger->incomplete_serialized_record.empty() ||
            ledger->r3_sequence.endpoint_cursor.next_absolute_position !=
                ledger->r3_sequence.owned_mapping.size()) {
            error = CloseProofValidationError::FinalBoundaryMismatch;
            return std::nullopt;
        }
        RemoteCloseAttestation message;
        message.transfer = exchange->request.transfer;
        message.r3 = exchange->request.r3;
        message.accepted_final_record = exchange->request.expected_final_record;
        message.close_id = exchange->request.close_id;
        message.challenge = exchange->request.challenge;
        message.integrity = protect(
            ProtectionDomain::RemoteCloseAttestation, covered(message),
            shared->evidence_keys[role_index(session.state_->local_role)]);
        exchange->stage = ExchangeStage::AttestationSent;
        exchange->revision = shared->next_revision++;
        error = CloseProofValidationError::None;
        return message;
    } catch (...) {
        error = CloseProofValidationError::EvidenceNotAuthoritative;
        return std::nullopt;
    }
}

std::optional<CloseAttestationReceipt> CloseExchangeOwner::send_receipt(
    LiveSession& session, const TransferIdentity& transfer,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::NoOutstandingRequest;
    try {
        if (!session.active()) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = session.state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        auto* exchange = find_exchange(*shared, transfer);
        if (!exchange) return std::nullopt;
        if (session.state_->local_role != origin_role(transfer)) {
            error = CloseProofValidationError::WrongPeerOrRole;
            return std::nullopt;
        }
        if (exchange->stage != ExchangeStage::AttestationReceived) {
            error = CloseProofValidationError::WrongExchangeStage;
            return std::nullopt;
        }
        CloseAttestationReceipt message;
        message.transfer = exchange->request.transfer;
        message.r3 = exchange->request.r3;
        message.accepted_final_record = exchange->request.expected_final_record;
        message.close_id = exchange->request.close_id;
        message.challenge = exchange->request.challenge;
        message.integrity = protect(
            ProtectionDomain::CloseAttestationReceipt, covered(message),
            shared->evidence_keys[role_index(session.state_->local_role)]);
        exchange->stage = ExchangeStage::ReceiptSent;
        exchange->revision = shared->next_revision++;
        error = CloseProofValidationError::None;
        return message;
    } catch (...) {
        error = CloseProofValidationError::EvidenceNotAuthoritative;
        return std::nullopt;
    }
}

std::optional<CloseConfirmation> CloseExchangeOwner::send_confirmation(
    LiveSession& session, const TransferIdentity& transfer,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::NoOutstandingRequest;
    try {
        if (!session.active()) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = session.state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        auto* exchange = find_exchange(*shared, transfer);
        if (!exchange) return std::nullopt;
        if (session.state_->local_role != opposite(origin_role(transfer))) {
            error = CloseProofValidationError::WrongPeerOrRole;
            return std::nullopt;
        }
        if (exchange->stage != ExchangeStage::ReceiptReceived) {
            error = CloseProofValidationError::WrongExchangeStage;
            return std::nullopt;
        }
        CloseConfirmation message;
        message.transfer = exchange->request.transfer;
        message.r3 = exchange->request.r3;
        message.accepted_final_record = exchange->request.expected_final_record;
        message.close_id = exchange->request.close_id;
        message.challenge = exchange->request.challenge;
        message.integrity = protect(
            ProtectionDomain::CloseConfirmation, covered(message),
            shared->evidence_keys[role_index(session.state_->local_role)]);
        exchange->stage = ExchangeStage::ConfirmationSent;
        exchange->revision = shared->next_revision++;
        error = CloseProofValidationError::None;
        return message;
    } catch (...) {
        error = CloseProofValidationError::EvidenceNotAuthoritative;
        return std::nullopt;
    }
}

std::optional<ReceivedCloseRequest> CloseReceiveValidator::validate_request(
    LiveReceiveContext& receive, const CloseRequest& decoded,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::EvidenceNotAuthoritative;
    try {
        if (!receive.session_ || !receive.session_->active() ||
            receive.generation_ == 0 || receive.receive_sequence_ == 0 ||
            receive.generation_ != receive.session_->state_->generation) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto& local = *receive.session_->state_;
        auto shared = local.negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        auto* exchange = find_exchange(*shared, decoded.transfer);
        auto* association = find_transfer(*shared, decoded.transfer);
        if (!exchange || !association || exchange->association != association) {
            error = CloseProofValidationError::TransferMismatch;
            return std::nullopt;
        }
        if (local.local_role != opposite(origin_role(decoded.transfer))) {
            error = CloseProofValidationError::WrongPeerOrRole;
            return std::nullopt;
        }
        if (exchange->stage != ExchangeStage::RequestSent) {
            error = CloseProofValidationError::WrongExchangeStage;
            return std::nullopt;
        }
        if (!same_transfer(decoded.transfer, association->transfer) ||
            !same_r3(decoded.r3, association->domains.r3)) {
            error = CloseProofValidationError::R3DomainMismatch;
            return std::nullopt;
        }
        if (decoded.close_id != exchange->request.close_id ||
            decoded.challenge.bytes != exchange->request.challenge.bytes) {
            error = decoded.close_id != exchange->request.close_id
                ? CloseProofValidationError::CloseIdMismatch
                : CloseProofValidationError::StaleOrReplayedChallenge;
            return std::nullopt;
        }
        if (!association->receiver_ledger) {
            error = CloseProofValidationError::TransferMismatch;
            return std::nullopt;
        }
        if (!association->receiver_ledger->incomplete_serialized_record.empty()) {
            error = CloseProofValidationError::FinalBoundaryMismatch;
            return std::nullopt;
        }
        const auto receiver_boundary = freeze_catalog(*association->receiver_ledger);
        if (!same_boundary(receiver_boundary, decoded.expected_final_record) ||
            !same_boundary(decoded.expected_final_record,
                           exchange->request.expected_final_record)) {
            error = CloseProofValidationError::FinalBoundaryMismatch;
            return std::nullopt;
        }
        const auto wire = full_wire(decoded);
        if (wire != receive.received_bytes_ ||
            !authoritative_integrity(decoded.integrity,
                                     ProtectionDomain::CloseRequest))
            return std::nullopt;
        const auto fields = covered(decoded);
        const auto& key = shared->evidence_keys[role_index(
            origin_role(decoded.transfer))];
        if (validate_integrity(
                decoded.integrity, ProtectionDomain::CloseRequest,
                IntegrityAlgorithm::Blake2b256Keyed,
                {fields.data(), fields.size()}, {key.bytes.data(), key.bytes.size()},
                true) != IntegrityValidationError::None)
            return std::nullopt;
        if (!local.validated_receive_sequences.insert(
                receive.receive_sequence_).second) {
            error = CloseProofValidationError::StaleRevision;
            return std::nullopt;
        }
        association->receiver_ledger->frozen_final_record = receiver_boundary;
        exchange->stage = ExchangeStage::RequestReceived;
        exchange->revision = shared->next_revision++;
        ReceivedCloseRequest result(decoded);
        result.session_generation_ = receive.generation_;
        result.exchange_revision_ = exchange->revision;
        result.receive_sequence_ = receive.receive_sequence_;
        error = CloseProofValidationError::None;
        return result;
    } catch (...) {
        return std::nullopt;
    }
}

template <typename Message>
bool validate_peer_message_common(
    NegotiatedState& shared, EndpointRole local_role,
    std::set<std::uint64_t>& validated_receive_sequences,
    const std::vector<std::uint8_t>& received_bytes,
    std::uint64_t receive_sequence, const Message& decoded,
    ProtectionDomain domain, EndpointRole expected_local,
    ExchangeStage expected_stage, ExchangeStage next_stage,
    CloseProofValidationError& error, ExchangeState*& exchange_out) {
    auto* exchange = find_exchange(shared, decoded.transfer);
    if (!exchange) {
        error = CloseProofValidationError::NoOutstandingRequest;
        return false;
    }
    if (local_role != expected_local) {
        error = CloseProofValidationError::WrongPeerOrRole;
        return false;
    }
    if (exchange->stage != expected_stage) {
        error = CloseProofValidationError::WrongExchangeStage;
        return false;
    }
    if (!message_matches_request(decoded, exchange->request)) {
        error = CloseProofValidationError::FinalBoundaryMismatch;
        return false;
    }
    const auto wire = full_wire(decoded);
    if (wire != received_bytes ||
        !authoritative_integrity(decoded.integrity, domain)) {
        error = domain == ProtectionDomain::CloseConfirmation
            ? CloseProofValidationError::ConfirmationIntegrityFailure
            : CloseProofValidationError::AttestationIntegrityFailure;
        return false;
    }
    const auto fields = covered(decoded);
    const auto sender = opposite(expected_local);
    const auto& key = shared.evidence_keys[role_index(sender)];
    if (validate_integrity(decoded.integrity, domain,
                           IntegrityAlgorithm::Blake2b256Keyed,
                           {fields.data(), fields.size()},
                           {key.bytes.data(), key.bytes.size()}, true) !=
        IntegrityValidationError::None) {
        error = domain == ProtectionDomain::CloseConfirmation
            ? CloseProofValidationError::ConfirmationIntegrityFailure
            : CloseProofValidationError::AttestationIntegrityFailure;
        return false;
    }
    if (!validated_receive_sequences.insert(receive_sequence).second) {
        error = CloseProofValidationError::StaleRevision;
        return false;
    }
    exchange->stage = next_stage;
    exchange->revision = shared.next_revision++;
    exchange_out = exchange;
    return true;
}

std::optional<ReceivedRemoteCloseAttestation>
CloseReceiveValidator::validate_attestation(
    LiveReceiveContext& receive, const RemoteCloseAttestation& decoded,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::EvidenceNotAuthoritative;
    try {
        if (!receive.session_ || !receive.session_->active() ||
            receive.generation_ != receive.session_->state_->generation ||
            receive.receive_sequence_ == 0) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = receive.session_->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        ExchangeState* exchange = nullptr;
        if (!validate_peer_message_common(
                *shared, receive.session_->state_->local_role,
                receive.session_->state_->validated_receive_sequences,
                receive.received_bytes_, receive.receive_sequence_, decoded,
                ProtectionDomain::RemoteCloseAttestation,
                origin_role(decoded.transfer), ExchangeStage::AttestationSent,
                ExchangeStage::AttestationReceived, error, exchange))
            return std::nullopt;
        exchange->attestation_receive_sequence = receive.receive_sequence_;
        ReceivedRemoteCloseAttestation result(decoded);
        result.session_generation_ = receive.generation_;
        result.exchange_revision_ = exchange->revision;
        result.receive_sequence_ = receive.receive_sequence_;
        error = CloseProofValidationError::None;
        return result;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<ReceivedCloseAttestationReceipt>
CloseReceiveValidator::validate_receipt(
    LiveReceiveContext& receive, const CloseAttestationReceipt& decoded,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::EvidenceNotAuthoritative;
    try {
        if (!receive.session_ || !receive.session_->active() ||
            receive.generation_ != receive.session_->state_->generation ||
            receive.receive_sequence_ == 0) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = receive.session_->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        ExchangeState* exchange = nullptr;
        if (!validate_peer_message_common(
                *shared, receive.session_->state_->local_role,
                receive.session_->state_->validated_receive_sequences,
                receive.received_bytes_, receive.receive_sequence_, decoded,
                ProtectionDomain::CloseAttestationReceipt,
                opposite(origin_role(decoded.transfer)),
                ExchangeStage::ReceiptSent, ExchangeStage::ReceiptReceived,
                error, exchange))
            return std::nullopt;
        ReceivedCloseAttestationReceipt result(decoded);
        result.session_generation_ = receive.generation_;
        result.exchange_revision_ = exchange->revision;
        result.receive_sequence_ = receive.receive_sequence_;
        error = CloseProofValidationError::None;
        return result;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<ReceivedCloseConfirmation>
CloseReceiveValidator::validate_confirmation(
    LiveReceiveContext& receive, const CloseConfirmation& decoded,
    CloseProofValidationError& error) noexcept {
    error = CloseProofValidationError::EvidenceNotAuthoritative;
    try {
        if (!receive.session_ || !receive.session_->active() ||
            receive.generation_ != receive.session_->state_->generation ||
            receive.receive_sequence_ == 0) {
            error = CloseProofValidationError::InactiveSession;
            return std::nullopt;
        }
        auto shared = receive.session_->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        ExchangeState* exchange = nullptr;
        if (!validate_peer_message_common(
                *shared, receive.session_->state_->local_role,
                receive.session_->state_->validated_receive_sequences,
                receive.received_bytes_, receive.receive_sequence_, decoded,
                ProtectionDomain::CloseConfirmation,
                origin_role(decoded.transfer), ExchangeStage::ConfirmationSent,
                ExchangeStage::ConfirmationReceived, error, exchange))
            return std::nullopt;
        exchange->confirmation_receive_sequence = receive.receive_sequence_;
        ReceivedCloseConfirmation result(decoded);
        result.session_generation_ = receive.generation_;
        result.exchange_revision_ = exchange->revision;
        result.receive_sequence_ = receive.receive_sequence_;
        error = CloseProofValidationError::None;
        return result;
    } catch (...) {
        return std::nullopt;
    }
}

CloseProofValidationResult CloseProofValidator::validate_received_messages(
    LiveReceiveContext& receive,
    ReceivedRemoteCloseAttestation&& received_attestation,
    ReceivedCloseConfirmation&& received_confirmation) noexcept {
    CloseProofValidationResult result;
    result.error = CloseProofValidationError::EvidenceNotAuthoritative;
    try {
        if (!receive.session_ || !receive.session_->active() ||
            receive.generation_ == 0 ||
            receive.generation_ != receive.session_->state_->generation) {
            result.error = CloseProofValidationError::InactiveSession;
            return result;
        }
        auto shared = receive.session_->state_->negotiated;
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (receive.session_->state_->local_role !=
            origin_role(received_attestation.message_.transfer)) {
            result.error = CloseProofValidationError::WrongPeerOrRole;
            return result;
        }
        if (!same_transfer(received_attestation.message_.transfer,
                           received_confirmation.message_.transfer)) {
            result.error = CloseProofValidationError::TransferMismatch;
            return result;
        }
        auto* exchange = find_exchange(
            *shared, received_attestation.message_.transfer);
        if (!exchange) {
            result.error = CloseProofValidationError::NoOutstandingRequest;
            return result;
        }
        if (exchange->stage != ExchangeStage::ConfirmationReceived) {
            result.error = CloseProofValidationError::WrongExchangeStage;
            return result;
        }
        if (received_attestation.session_generation_ != receive.generation_ ||
            received_confirmation.session_generation_ != receive.generation_ ||
            received_attestation.receive_sequence_ == 0 ||
            received_confirmation.receive_sequence_ != receive.receive_sequence_ ||
            received_attestation.receive_sequence_ !=
                exchange->attestation_receive_sequence ||
            received_confirmation.receive_sequence_ !=
                exchange->confirmation_receive_sequence ||
            received_attestation.exchange_revision_ == 0 ||
            received_confirmation.exchange_revision_ != exchange->revision) {
            result.error = CloseProofValidationError::StaleRevision;
            return result;
        }
        if (!message_matches_request(received_attestation.message_,
                                     exchange->request) ||
            !message_matches_request(received_confirmation.message_,
                                     exchange->request)) {
            result.error = CloseProofValidationError::FinalBoundaryMismatch;
            return result;
        }
        exchange->stage = ExchangeStage::Consumed;
        exchange->revision = shared->next_revision++;
        result.proof.emplace(MatchingCloseProof(
            std::move(received_attestation.message_),
            std::move(received_confirmation.message_)));
        received_attestation.session_generation_ = 0;
        received_attestation.exchange_revision_ = 0;
        received_attestation.receive_sequence_ = 0;
        received_confirmation.session_generation_ = 0;
        received_confirmation.exchange_revision_ = 0;
        received_confirmation.receive_sequence_ = 0;
        result.error = CloseProofValidationError::None;
        return result;
    } catch (...) {
        result.proof.reset();
        return result;
    }
}

}  // namespace iris::v2
