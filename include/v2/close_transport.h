#ifndef IRIS_V2_CLOSE_TRANSPORT_H
#define IRIS_V2_CLOSE_TRANSPORT_H

// Author: xmutantson
#include "v2/session.h"

namespace iris {
class Modem;
namespace v2 {

struct TransferLedger;

// RC2 transport integration seams. Lookup returns only the modem's negotiated,
// active session, with its authenticated transfer/domain associations installed.
// It must never create a session from a decoded close or a diagnostic ledger.
// The returned session is borrowed for the current serialized modem operation.
class CloseTransportOwner {
public:
    static LiveSession* session(Modem& modem) noexcept;
    static bool register_origin_transfer(Modem& modem,
                                         TransferLedger& ledger) noexcept;
    static bool register_receiver_transfer(Modem& modem,
                                           TransferLedger& ledger) noexcept;
    static void revoke(Modem& modem) noexcept;
    // Revoke the captured session.  Clear the modem's current slot only when it
    // still names that same session, so teardown cannot revoke a replacement.
    static void revoke(
        Modem& modem,
        const std::shared_ptr<LiveSession>& captured_session) noexcept;
};

// Stage the complete canonical control object received from the peer: all fields
// in declaration order, including its IntegrityInfo trailer (Profile 1).
// The transport owner pins the actual modem session, peer role and receive
// sequence through ProtectedReceiveBoundary. Staging grants no close authority;
// CloseReceiveValidator must verify the bytes, keyed tag and exchange stage.
class TransportReceiveOwner {
public:
    static std::optional<LiveReceiveContext> receive_close_control(
        Modem& modem, std::vector<std::uint8_t> canonical_received_bytes) noexcept;
};

}  // namespace v2
}  // namespace iris

#endif  // IRIS_V2_CLOSE_TRANSPORT_H
