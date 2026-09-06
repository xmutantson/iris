#ifndef IRIS_AX25_DIGIPEATER_H
#define IRIS_AX25_DIGIPEATER_H

#include "ax25/ax25_protocol.h"
#include <cstdint>
#include <regex>
#include <string>
#include <vector>

namespace iris {

// AFSK->AFSK AX.25 digipeater (config-gated, DEFAULT OFF — a modem must never
// start repeating other stations' traffic unless the operator opts in).
//
// Ported from Direwolf (../direwolf/src/), which keeps the two digipeat
// flavors in SEPARATE files because they behave differently:
//
//  (A) CONNECTED-MODE digipeat — cdigipeater.c, cdigipeat_match() :238
//      ("a simple digipeater for connected mode AX.25", :199).  Applies to
//      I/S/U session frames (SABM/UA/DISC/DM/I/RR/...).  Rule: the first via
//      hop whose has-been-repeated (H) bit is still clear must EXACTLY match
//      one of our digipeat callsigns; set that H-bit (ax25_set_h,
//      ax25_pad.c:1579 — bit 0x80 of the hop's SSID octet) and re-emit the
//      frame otherwise VERBATIM.  NO duplicate suppression and NO WIDEn-N
//      aliasing: connected mode has its own sequence numbers and
//      retransmission ("APRS digipeating drops duplicates within 30 seconds
//      but we don't do that here", cdigipeater.c:233) — a 30 s dedup here
//      would EAT legitimate I-frame retransmissions and break the link.
//
//  (B) UI / connectionless digipeat — digipeater.c, digipeat_match() :297,
//      with dedupe.c (dedupe_check :238 / dedupe_remember :202, HISTORY_MAX
//      25 :137, default 30 s TTL :134; checksum over source + destination +
//      info but NOT the via path, ax25_dedupe_crc, ax25_pad.c:2777).  Applies
//      ONLY to UI frames (APRS-style beacons).  Rule: first un-repeated hop;
//      exact call OR alias-regex substitution OR WIDEn-N SSID decrement
//      (digipeater.c:525-600); duplicates within the TTL are dropped — a UI
//      digi without dedup storms (dedupe.c:29-73 enumerates the loop cases).
//
// Not ported (v1): preemptive digipeating (digipeater.c:450-523), the ATGP
// hack (:537), per-route packet filters, and the cross-channel routing matrix
// (Iris has ONE AFSK channel today; the from/to matrix collapses to one cell).
struct DigipeatConfig {
    bool enabled = false;          // master gate — DEFAULT OFF (operator opt-in)
    // Callsign-SSIDs we digipeat for (EXACT callsign+SSID match on the next
    // un-repeated via hop).  The FIRST entry is the call substituted into the
    // path on alias/WIDE matches (Direwolf mycall_xmit, digipeater.c:308).
    std::vector<std::string> mycalls;
    bool connected_mode = true;    // path (A): I/S/non-UI-U frames
    bool ui_mode = true;           // path (B): UI frames
    // UI alias regex (digipeater.c:418-432), matched against the hop's
    // "CALL-N" string, case-insensitive.  Empty = no alias.
    std::string ui_alias;
    // UI WIDEn-N regex (digipeater.c:525-600).  Empty = no WIDE handling.
    std::string ui_wide = "^WIDE[1-7]-[1-7]$";
    int dedupe_ttl_s = 30;         // UI duplicate-suppression window (dedupe.c:134)
};

class Digipeater {
public:
    // Compile the config (regexes).  An invalid regex is rejected LOUDLY via
    // the returned error string (pattern disabled, never a silent behavior
    // change); returns empty string on success.
    std::string configure(const DigipeatConfig& cfg);

    bool enabled() const { return cfg_.enabled && !mycall_addrs_.empty(); }

    // The digipeat decision + rebuild.  Input: a received AX.25 frame (FCS
    // already stripped).  Returns the frame bytes to re-emit on the AFSK TX
    // path (H-bit set / path edited per the rules above), or an empty vector
    // when the frame must NOT be repeated.  `now_ms` is a monotonic
    // millisecond clock, caller-supplied so tests control the dedup window.
    // Mirrors Direwolf's contract: the input is never modified, the result is
    // an edited COPY (digipeat_match, digipeater.c:222-225).
    std::vector<uint8_t> digipeat(const uint8_t* data, size_t len, int64_t now_ms);

    // One-line description of the active config for the startup log
    // (config-gated behavior must be visible, never silent).
    std::string describe() const;

    // Why the last digipeat() returned empty (for logs/tests): "", "dup",
    // "not-us", "own-src", "no-hop", "disabled".
    const char* last_drop_reason() const { return last_drop_reason_; }

private:
    // dedupe.c port: fixed ring of {time, checksum} (HISTORY_MAX 25).
    static constexpr int HISTORY_MAX = 25;
    struct HistEntry {
        int64_t time_ms = INT64_MIN;   // never matches until written
        uint16_t crc = 0;
    };

    uint16_t dedupe_crc(const Ax25Frame& f) const;         // ax25_pad.c:2777
    bool dedupe_check(const Ax25Frame& f, int64_t now_ms); // dedupe.c:238
    void dedupe_remember(const Ax25Frame& f, int64_t now_ms); // dedupe.c:202

    DigipeatConfig cfg_;
    std::vector<Ax25Address> mycall_addrs_;  // parsed cfg_.mycalls
    bool has_alias_ = false;
    bool has_wide_ = false;
    std::regex alias_re_;
    std::regex wide_re_;
    HistEntry history_[HISTORY_MAX];
    int insert_next_ = 0;
    const char* last_drop_reason_ = "";
};

} // namespace iris

#endif // IRIS_AX25_DIGIPEATER_H
