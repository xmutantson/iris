// Author: xmutantson
#pragma once

namespace iris {
class ArqSession;
// Read-only inspection through the existing acceptance friend; no state seeding.
bool acceptance_rc5surg_tx_retired(const ArqSession& session);
}
