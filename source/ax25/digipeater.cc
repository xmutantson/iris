// AFSK->AFSK AX.25 digipeater — Direwolf port.  See digipeater.h for the
// full provenance map (cdigipeater.c connected-mode rule, digipeater.c UI
// rule, dedupe.c duplicate suppression) and the digipeater design
// documentation for the design this implements (§5.1 / §2.2 / §2.3).
#include "ax25/digipeater.h"
#include "ax25/crc16.h"
#include "common/logging.h"

#include <climits>

namespace iris {

// ---------------------------------------------------------------------------
// Wire-offset helpers.  AX.25 address field: dst at 0-6, src at 7-13, via hop
// i at 14+7i; the hop's SSID octet (H-bit 0x80, SSID<<1, extension bit 0x01)
// is its 7th byte.  These edit a COPY of the received bytes so everything the
// rules don't touch stays byte-identical on the air (the interop invariant —
// Direwolf edits its packet objects' frame bytes the same way: ax25_set_h
// ax25_pad.c:1579, ax25_set_addr, ax25_insert_addr).
// ---------------------------------------------------------------------------

static size_t via_off(int hop) { return 14 + 7 * (size_t)hop; }

// ax25_set_h (ax25_pad.c:1579): frame_data[n*7+6] |= 0x80.  Everything else
// verbatim — the connected-mode re-emit is exactly this one bit.
static std::vector<uint8_t> with_h_set(const uint8_t* d, size_t len, int hop) {
    std::vector<uint8_t> v(d, d + len);
    v[via_off(hop) + 6] |= 0x80;
    return v;
}

// Replace hop's address with `a` and set its H-bit (the alias / WIDEn-N
// "substitute MYCALL" rule, digipeater.c:428-433 / :576-585).  The extension
// bit is preserved (the hop keeps its position in the address list); reserved
// bits normalized to 11 (spec default, same as our builders).
static std::vector<uint8_t> with_addr_replaced(const uint8_t* d, size_t len,
                                               int hop, const Ax25Address& a) {
    std::vector<uint8_t> v(d, d + len);
    size_t off = via_off(hop);
    for (int i = 0; i < 6; i++)
        v[off + i] = (uint8_t)(a.callsign[i]) << 1;
    uint8_t ext = v[off + 6] & 0x01;
    v[off + 6] = (uint8_t)(0x80 | 0x60 | ((a.ssid & 0x0F) << 1) | ext);
    return v;
}

// WIDEn-N SSID decrement in place, H-bit left clear (digipeater.c:587-593:
// "Decrement y and don't mark repeater as being used").
static std::vector<uint8_t> with_ssid_decrement(const uint8_t* d, size_t len,
                                                int hop) {
    std::vector<uint8_t> v(d, d + len);
    size_t off = via_off(hop);
    uint8_t ssid = (uint8_t)((v[off + 6] >> 1) & 0x0F);
    v[off + 6] = (uint8_t)((v[off + 6] & ~0x1E) | (((ssid - 1) & 0x0F) << 1));
    return v;
}

// Insert `a` with H set AHEAD of `hop` (WIDEn-N trace insert,
// digipeater.c:595-597: ax25_insert_addr + ax25_set_h).  The inserted hop's
// extension bit is 0 — an address always follows it.
static std::vector<uint8_t> with_hop_inserted(const std::vector<uint8_t>& in,
                                              int hop, const Ax25Address& a) {
    std::vector<uint8_t> v;
    v.reserve(in.size() + 7);
    size_t off = via_off(hop);
    v.insert(v.end(), in.begin(), in.begin() + off);
    for (int i = 0; i < 6; i++)
        v.push_back((uint8_t)(a.callsign[i]) << 1);
    v.push_back((uint8_t)(0x80 | 0x60 | ((a.ssid & 0x0F) << 1)));  // H=1, ext=0
    v.insert(v.end(), in.begin() + off, in.end());
    return v;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

std::string Digipeater::configure(const DigipeatConfig& cfg) {
    cfg_ = cfg;
    mycall_addrs_.clear();
    has_alias_ = has_wide_ = false;
    insert_next_ = 0;
    for (auto& h : history_) h = HistEntry{};
    last_drop_reason_ = "";

    std::string err;
    for (const auto& c : cfg.mycalls) {
        if (c.empty()) continue;
        mycall_addrs_.push_back(ax25_make_addr(c));
    }
    if (cfg.enabled && mycall_addrs_.empty())
        err += "no digipeat callsign configured; ";

    // Compile the UI regexes.  Case-insensitive, like Direwolf's regcomp
    // REG_ICASE (config.c digipeat alias compilation).  An invalid pattern is
    // reported and DISABLED — fail-safe is "don't repeat", never a guess.
    if (!cfg.ui_alias.empty()) {
        try {
            alias_re_ = std::regex(cfg.ui_alias, std::regex::icase);
            has_alias_ = true;
        } catch (const std::regex_error& e) {
            err += "bad UIAlias regex '" + cfg.ui_alias + "' (" + e.what() + "); ";
        }
    }
    if (!cfg.ui_wide.empty()) {
        try {
            wide_re_ = std::regex(cfg.ui_wide, std::regex::icase);
            has_wide_ = true;
        } catch (const std::regex_error& e) {
            err += "bad UIWide regex '" + cfg.ui_wide + "' (" + e.what() + "); ";
        }
    }
    return err;
}

std::string Digipeater::describe() const {
    if (!enabled()) return "off";
    std::string s = "for ";
    for (size_t i = 0; i < mycall_addrs_.size(); i++) {
        if (i) s += ",";
        s += mycall_addrs_[i].to_string();
    }
    s += " (connected=";
    s += cfg_.connected_mode ? "on" : "off";
    s += " ui=";
    s += cfg_.ui_mode ? "on" : "off";
    if (cfg_.ui_mode) {
        s += " alias=" + (has_alias_ ? cfg_.ui_alias : std::string("none"));
        s += " wide=" + (has_wide_ ? cfg_.ui_wide : std::string("none"));
        s += " dedupe=" + std::to_string(cfg_.dedupe_ttl_s) + "s";
    }
    s += ")";
    return s;
}

// ---------------------------------------------------------------------------
// dedupe.c port — UI path ONLY (see header).  Checksum over source +
// destination + info, NOT the via path (ax25_dedupe_crc, ax25_pad.c:2777-2809,
// incl. the v1.3 trailing CR/LF/space trim :2789-2801); fixed 25-entry ring
// with a TTL (dedupe.c:134-163).  Single AFSK channel, so no per-channel key.
// ---------------------------------------------------------------------------

uint16_t Digipeater::dedupe_crc(const Ax25Frame& f) const {
    std::string src = f.src.to_string();
    std::string dst = f.dst.to_string();
    size_t ilen = f.info.size();
    while (ilen >= 1 && (f.info[ilen - 1] == '\r' || f.info[ilen - 1] == '\n' ||
                         f.info[ilen - 1] == ' '))
        ilen--;
    std::vector<uint8_t> buf;
    buf.reserve(src.size() + dst.size() + ilen + 2);
    buf.insert(buf.end(), src.begin(), src.end());
    buf.push_back(0);   // field separators (Direwolf chains three CRC passes;
    buf.insert(buf.end(), dst.begin(), dst.end());
    buf.push_back(0);   // one pass over separated fields is equivalent here)
    buf.insert(buf.end(), f.info.begin(), f.info.begin() + ilen);
    return crc16_ccitt(buf.data(), buf.size());
}

bool Digipeater::dedupe_check(const Ax25Frame& f, int64_t now_ms) {
    uint16_t crc = dedupe_crc(f);
    int64_t ttl_ms = (int64_t)cfg_.dedupe_ttl_s * 1000;
    for (const auto& h : history_) {
        if (h.time_ms != INT64_MIN && h.time_ms >= now_ms - ttl_ms && h.crc == crc)
            return true;
    }
    return false;
}

void Digipeater::dedupe_remember(const Ax25Frame& f, int64_t now_ms) {
    history_[insert_next_].time_ms = now_ms;
    history_[insert_next_].crc = dedupe_crc(f);
    if (++insert_next_ >= HISTORY_MAX) insert_next_ = 0;
}

// ---------------------------------------------------------------------------
// The digipeat decision.
// ---------------------------------------------------------------------------

std::vector<uint8_t> Digipeater::digipeat(const uint8_t* data, size_t len,
                                          int64_t now_ms) {
    last_drop_reason_ = "";
    if (!enabled()) { last_drop_reason_ = "disabled"; return {}; }

    Ax25Frame f;
    if (!ax25_parse(data, len, f) || f.via.empty()) {
        last_drop_reason_ = "no-hop";
        return {};
    }

    // First repeater whose has-been-repeated flag is still clear
    // (ax25_get_first_not_repeated, ax25_pad.c:1648-1662).  None -> nothing
    // to do (digipeat_match :341-345 / cdigipeat_match :283-285).
    int r = -1;
    for (size_t i = 0; i < f.via.size(); i++) {
        if (!f.via[i].repeated) { r = (int)i; break; }
    }
    if (r < 0) { last_drop_reason_ = "no-hop"; return {}; }

    // Never repeat our own transmissions (digipeater.c:380-388).  Applied to
    // BOTH paths here (cdigipeater.c has no self-source check — Direwolf never
    // hears its own TX; Iris can, in loopback and with some sound-card
    // routings, so the guard is cheap insurance on the connected path too).
    for (const auto& a : mycall_addrs_) {
        if (f.src == a) { last_drop_reason_ = "own-src"; return {}; }
    }

    bool exact_match = false;
    for (const auto& a : mycall_addrs_) {
        if (f.via[r].addr == a) { exact_match = true; break; }
    }

    bool is_ui = f.type() == Ax25FrameType::U_FRAME &&
                 f.u_type() == Ax25UType::UI;

    if (!is_ui) {
        // ---- (A) CONNECTED-MODE path: cdigipeat_match, cdigipeater.c:238 ----
        // Exact callsign match on the first clear hop; set H; re-emit
        // VERBATIM.  Deliberately NO dedup, NO alias, NO WIDE (see header).
        if (!cfg_.connected_mode) { last_drop_reason_ = "disabled"; return {}; }
        if (!exact_match) { last_drop_reason_ = "not-us"; return {}; }
        IRIS_LOG("[DIGI] connected-mode digipeat: %s>%s via [%s] hop %d",
                 f.src.to_string().c_str(), f.dst.to_string().c_str(),
                 ax25_via_to_string(f.via).c_str(), r);
        return with_h_set(data, len, r);
    }

    // ---- (B) UI path: digipeat_match, digipeater.c:297 + dedupe.c ----------
    if (!cfg_.ui_mode) { last_drop_reason_ = "disabled"; return {}; }

    // Duplicate suppression FIRST (digipeater.c:391-415).  ONE deliberate
    // ordering deviation from Direwolf: there an exact-mycall match bypasses
    // the dedup history (digipeater.c:356-364, a testing affordance — "I
    // would expect it only for testing purposes").  Here the explicit-call
    // hop IS the normal deployment (via IRISDIGI), and a UI digi without
    // dedup storms (dedupe.c:29-73), so the history applies to every UI
    // repeat, explicit calls included.
    if (dedupe_check(f, now_ms)) {
        last_drop_reason_ = "dup";
        IRIS_LOG("[DIGI] drop redundant UI packet (dup within %d s): %s>%s via [%s]",
                 cfg_.dedupe_ttl_s, f.src.to_string().c_str(),
                 f.dst.to_string().c_str(), ax25_via_to_string(f.via).c_str());
        return {};
    }

    std::vector<uint8_t> out;
    const std::string hopstr = f.via[r].addr.to_string();

    if (exact_match) {
        // Explicit use of my call (digipeater.c:366-378): set H, verbatim.
        out = with_h_set(data, len, r);
    } else if (has_alias_ && std::regex_match(hopstr, alias_re_)) {
        // Alias: unconditionally substitute MYCALL once (digipeater.c:418-433).
        out = with_addr_replaced(data, len, r, mycall_addrs_[0]);
    } else if (has_wide_ && std::regex_match(hopstr, wide_re_)) {
        int ssid = f.via[r].addr.ssid;
        if (ssid == 1) {
            // WIDEn-1: replace with MYCALL, mark used (digipeater.c:576-585).
            out = with_addr_replaced(data, len, r, mycall_addrs_[0]);
        } else if (ssid >= 2 && ssid <= 7) {
            // WIDEn-N: decrement N, leave the hop's H clear, insert MYCALL*
            // ahead for the trace if the address field has room
            // (digipeater.c:587-600; AX25_MAX_REPEATERS = 8).
            out = with_ssid_decrement(data, len, r);
            if (f.via.size() < 8)
                out = with_hop_inserted(out, r, mycall_addrs_[0]);
        }
        // ssid 0 falls through: a spent WIDE hop with a clear H is not ours.
    }

    if (out.empty()) { last_drop_reason_ = "not-us"; return {}; }

    // Remember BEFORE queueing, like Direwolf (digipeater() remembers the
    // ORIGINAL packet right before tq_append — dedupe_remember keys on
    // src+dst+info, which the surgery above never touches).
    dedupe_remember(f, now_ms);
    IRIS_LOG("[DIGI] UI digipeat: %s>%s via [%s] hop %d (%s)",
             f.src.to_string().c_str(), f.dst.to_string().c_str(),
             ax25_via_to_string(f.via).c_str(), r,
             exact_match ? "mycall" : "alias/wide");
    return out;
}

} // namespace iris
