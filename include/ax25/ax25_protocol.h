#ifndef IRIS_AX25_PROTOCOL_H
#define IRIS_AX25_PROTOCOL_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace iris {

// AX.25 frame type classification
enum class Ax25FrameType { I_FRAME, S_FRAME, U_FRAME };

// U-frame subtypes
enum class Ax25UType { SABM, UA, DISC, DM, FRMR, UI, XID, UNKNOWN };

// S-frame subtypes
enum class Ax25SType : uint8_t { RR = 0, RNR = 1, REJ = 2, SREJ = 3 };

// Control field constants (without P/F bit)
constexpr uint8_t AX25_CTRL_SABM  = 0x2F;
constexpr uint8_t AX25_CTRL_UA    = 0x63;
constexpr uint8_t AX25_CTRL_DISC  = 0x43;
constexpr uint8_t AX25_CTRL_DM    = 0x0F;
constexpr uint8_t AX25_CTRL_FRMR  = 0x87;
constexpr uint8_t AX25_CTRL_UI    = 0x03;
constexpr uint8_t AX25_CTRL_XID_U = 0xAF;
constexpr uint8_t AX25_PF_MASK    = 0x10;

// AX.25 address (7 bytes on wire: 6 callsign chars shifted left + SSID byte)
struct Ax25Address {
    char callsign[7];  // 6 chars + null terminator
    uint8_t ssid;

    std::string to_string() const;
    bool matches(const std::string& call) const;

    bool operator==(const Ax25Address& o) const {
        return ssid == o.ssid && strncmp(callsign, o.callsign, 6) == 0;
    }
    bool operator!=(const Ax25Address& o) const { return !(*this == o); }
};

// One digipeater ("via") hop in the AX.25 address field.  `repeated` models the
// H-bit (has-been-repeated): bit 7 of a repeater SSID octet, 0 when the hop has
// not been used yet, set to 1 by the digipeater that repeats the frame.  (The
// same wire bit is the command/response C-bit on the dst/src SSID octets.)
struct Ax25ViaHop {
    Ax25Address addr;
    bool repeated = false;   // H-bit

    bool operator==(const Ax25ViaHop& o) const {
        return repeated == o.repeated && addr == o.addr;
    }
    bool operator!=(const Ax25ViaHop& o) const { return !(*this == o); }
};

// Ordered digipeater path, first hop first.  AX.25 2.2 allows up to 8.
using Ax25ViaPath = std::vector<Ax25ViaHop>;

// "WIDE1-1,WIDE2-1" with '*' appended to hops whose H-bit is set (TNC-2
// monitor notation), e.g. "KG7AAA-2*,WIDE2-1".  For logging.
std::string ax25_via_to_string(const Ax25ViaPath& via);

// Parsed AX.25 frame
struct Ax25Frame {
    Ax25Address dst;
    Ax25Address src;
    // Digipeater ("via") path, in path order, with per-hop H-bit.  Empty for
    // the common direct frame.  Populated by ax25_parse; consumed by the
    // ax25_build_* functions so a via path round-trips byte-identically.
    // NOTE: carrying the path here is a GUARDRAIL — no repeat logic exists yet.
    // See the digipeater design documentation.
    Ax25ViaPath via;
    uint8_t control;      // octet-1 of the control field (frame-type bits + mod-8 N(S)/SS)
    uint8_t pid;          // Valid for I and UI frames
    std::vector<uint8_t> info;
    // EXTENDED (AX.25 2.2 modulo-128) control support — used ONLY by the terminate/
    // re-pack OWNED OFDM transport (Iris<->Iris; a wider window than mod-8/K=7 needs
    // 7-bit N(S)/N(R)).  `extended` is set by ax25_parse() when told the link is
    // modulo-128; `control2` holds the 2nd control octet ((N(R)<<1)|P/F for I/S).
    // Default false -> every legacy/AFSK/mod-8 frame decodes byte-identically.
    bool extended = false;
    uint8_t control2 = 0;
    // Command/Response discrimination (AX.25 2.2 §6.1.2): the C bits ride in
    // bit 7 of the dst/src SSID octets.  Command = dst C=1, src C=0; Response =
    // dst C=0, src C=1 (v2.0 frames set both equal — "unspecified", treated as
    // NOT a command).  Captured by ax25_parse(); the builders already encode it.
    // Lets the session tell a P=1 COMMAND poll (which must be answered F=1) from
    // an F=1 RESPONSE (an answer — answering it back is the RR-volley engine).
    bool cmd_dst = false;   // dst SSID C bit
    bool cmd_src = false;   // src SSID C bit
    bool is_command() const { return cmd_dst && !cmd_src; }

    Ax25FrameType type() const {
        // Frame-type bits live in octet-1 in BOTH mod-8 and modulo-128, width-independent.
        if ((control & 0x01) == 0) return Ax25FrameType::I_FRAME;
        if ((control & 0x03) == 0x01) return Ax25FrameType::S_FRAME;
        return Ax25FrameType::U_FRAME;
    }

    Ax25UType u_type() const {
        // U-frames are ALWAYS a single control octet (modulo-128 widens only I/S).
        uint8_t masked = control & ~AX25_PF_MASK;
        if (masked == AX25_CTRL_SABM)  return Ax25UType::SABM;
        if (masked == AX25_CTRL_UA)    return Ax25UType::UA;
        if (masked == AX25_CTRL_DISC)  return Ax25UType::DISC;
        if (masked == AX25_CTRL_DM)    return Ax25UType::DM;
        if (masked == AX25_CTRL_FRMR)  return Ax25UType::FRMR;
        if (masked == AX25_CTRL_UI)    return Ax25UType::UI;
        if (masked == AX25_CTRL_XID_U) return Ax25UType::XID;
        return Ax25UType::UNKNOWN;
    }

    Ax25SType s_type() const { return (Ax25SType)((control >> 2) & 0x03); }

    // P/F and N(R) live in octet-2 when extended, octet-1 when mod-8.
    bool poll_final() const {
        return extended ? (control2 & 0x01) != 0 : (control & AX25_PF_MASK) != 0;
    }
    uint8_t nr() const {   // N(R) from I or S frames
        return extended ? (uint8_t)((control2 >> 1) & 0x7F) : (uint8_t)((control >> 5) & 0x07);
    }
    uint8_t ns() const {   // N(S) from I frames
        return extended ? (uint8_t)((control >> 1) & 0x7F) : (uint8_t)((control >> 1) & 0x07);
    }
};

// Parse raw AX.25 frame (after HDLC decode, FCS stripped).  `extended` = the link is
// AX.25 2.2 modulo-128: I/S frames carry a 2-octet control field (7-bit N(S)/N(R)).
// U-frames stay 1 octet regardless.  Default false = classic mod-8 (byte-identical to
// every existing caller).
bool ax25_parse(const uint8_t* data, size_t len, Ax25Frame& frame, bool extended = false);

// Build an address from "CALL" or "CALL-N" string
Ax25Address ax25_make_addr(const std::string& callsign, uint8_t ssid = 0);

// Build raw AX.25 frame bytes (ready for HDLC encode)
// command: true = command frame (SABM, DISC, I), false = response (UA, DM)
// `via` (all builders below): optional digipeater path emitted between src and
// the control field, per-hop H-bit preserved.  Default {} keeps the encoding
// byte-identical to the historical two-address form (the interop invariant).
std::vector<uint8_t> ax25_build_u(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t u_ctrl, bool pf, bool command = true,
                                   const Ax25ViaPath& via = {});

// `extended` builds a modulo-128 (2-octet control) S-frame: octet-1 = 0x01|(SS<<2),
// octet-2 = (N(R)<<1)|P/F; N(R) up to 7 bits.  Default false = classic mod-8.
std::vector<uint8_t> ax25_build_s(const Ax25Address& dst, const Ax25Address& src,
                                   Ax25SType stype, uint8_t nr, bool pf,
                                   bool command = true, bool extended = false,
                                   const Ax25ViaPath& via = {});

// `extended` builds a modulo-128 (2-octet control) I-frame: octet-1 = (N(S)<<1),
// octet-2 = (N(R)<<1)|P/F; N(S)/N(R) up to 7 bits each.  Default false = classic mod-8.
std::vector<uint8_t> ax25_build_i(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t ns, uint8_t nr, bool pf, uint8_t pid,
                                   const uint8_t* info, size_t info_len,
                                   bool extended = false,
                                   const Ax25ViaPath& via = {});

// Build FRMR frame with 3-byte info field (modulo 8)
// w=invalid ctrl, x=info when not allowed, y=info too long, z=invalid N(R)
std::vector<uint8_t> ax25_build_frmr(const Ax25Address& dst, const Ax25Address& src,
                                      uint8_t rejected_ctrl, uint8_t vs, uint8_t vr,
                                      bool cr, bool w, bool x, bool y, bool z,
                                      const Ax25ViaPath& via = {});

} // namespace iris

#endif // IRIS_AX25_PROTOCOL_H
