#include "ax25/ax25_protocol.h"
#include <algorithm>

namespace iris {

std::string Ax25Address::to_string() const {
    std::string s;
    for (int i = 0; i < 6; i++) {
        if (callsign[i] != ' ' && callsign[i] != '\0')
            s += callsign[i];
    }
    if (ssid != 0)
        s += "-" + std::to_string(ssid);
    return s;
}

bool Ax25Address::matches(const std::string& call) const {
    // Parse "CALL" or "CALL-N" format
    std::string base = call;
    uint8_t s = 0;
    bool has_ssid = false;
    auto pos = call.find('-');
    if (pos != std::string::npos) {
        std::string ssid_str = call.substr(pos + 1);
        if (!ssid_str.empty() && ssid_str.find_first_not_of("0123456789") == std::string::npos) {
            base = call.substr(0, pos);
            s = (uint8_t)std::stoi(ssid_str);
            has_ssid = true;
        }
        // else: not a numeric SSID, use full string as callsign
    }

    // Compare callsign (padded to 6 with spaces)
    for (int i = 0; i < 6; i++) {
        char expected = (i < (int)base.size()) ? base[i] : ' ';
        if (callsign[i] != expected) return false;
    }
    // If caller specified an explicit SSID (e.g. "KG7VSN-15"), require exact match.
    // If no SSID specified (e.g. "KG7VSN"), accept any SSID — allows responding
    // to frames addressed to any SSID of our callsign (important for Winlink).
    if (has_ssid)
        return ssid == s;
    return true;
}

// Encode a 7-byte AX.25 address into frame bytes.  `hi_bit` is bit 7 of the
// SSID octet: the command/response C-bit on dst/src, the has-been-repeated
// H-bit on digipeater addresses (same wire position, different meaning).
static void encode_addr(std::vector<uint8_t>& out, const Ax25Address& addr,
                         bool hi_bit, bool last) {
    for (int i = 0; i < 6; i++)
        out.push_back((uint8_t)(addr.callsign[i]) << 1);
    uint8_t ssid_byte = (addr.ssid << 1) | 0x60;  // Reserved bits 5-6 set
    if (hi_bit) ssid_byte |= 0x80;  // C-bit (dst/src) or H-bit (digipeater)
    if (last)   ssid_byte |= 0x01;  // Extension bit (last address)
    out.push_back(ssid_byte);
}

// Encode the digipeater ("via") addresses between src and the control field.
// The final hop carries the end-of-address extension bit.
static void encode_via(std::vector<uint8_t>& out, const Ax25ViaPath& via) {
    for (size_t i = 0; i < via.size(); i++)
        encode_addr(out, via[i].addr, via[i].repeated, i + 1 == via.size());
}

std::string ax25_via_to_string(const Ax25ViaPath& via) {
    std::string s;
    for (size_t i = 0; i < via.size(); i++) {
        if (i) s += ",";
        s += via[i].addr.to_string();
        if (via[i].repeated) s += "*";   // TNC-2 monitor notation for H-bit set
    }
    return s;
}

// Decode a 7-byte AX.25 address from frame bytes
static void decode_addr(const uint8_t* data, Ax25Address& addr) {
    for (int i = 0; i < 6; i++)
        addr.callsign[i] = (char)(data[i] >> 1);
    addr.callsign[6] = '\0';
    addr.ssid = (data[6] >> 1) & 0x0F;
}

bool ax25_parse(const uint8_t* data, size_t len, Ax25Frame& frame, bool extended) {
    if (len < 15) return false;  // dst(7) + src(7) + control(1)

    decode_addr(data, frame.dst);
    decode_addr(data + 7, frame.src);
    // Command/Response C bits (bit 7 of each SSID octet, AX.25 2.2 §6.1.2).
    // decode_addr masks them out of .ssid; capture them here so the session layer
    // can tell a P=1 COMMAND (answer F=1) from an F=1 RESPONSE (an answer).  dst
    // SSID is octet 6, src SSID octet 13 — fixed positions ahead of any digis.
    frame.cmd_dst = (data[6] & 0x80) != 0;
    frame.cmd_src = (data[13] & 0x80) != 0;

    // Find control field, CAPTURING the digipeater ("via") addresses on the
    // way instead of skipping them.  Path order and per-hop H-bit preserved so
    // the frame round-trips byte-identically through the ax25_build_* functions.
    frame.via.clear();
    size_t ctrl_offset = 14;
    if ((data[13] & 0x01) == 0) {
        // More addresses follow (digipeaters)
        size_t pos = 14;
        while (pos + 6 < len) {
            Ax25ViaHop hop;
            decode_addr(data + pos, hop.addr);
            hop.repeated = (data[pos + 6] & 0x80) != 0;  // H-bit
            frame.via.push_back(hop);
            if (data[pos + 6] & 0x01) {
                ctrl_offset = pos + 7;
                break;
            }
            pos += 7;
        }
        if (ctrl_offset == 14) {
            // End-of-address bit never found: tolerated-malformed path, control
            // taken at offset 14 exactly as before — the walked bytes were not
            // a well-formed address field, so no via list is kept.
            frame.via.clear();
            if (pos + 6 < len)
                ctrl_offset = pos + 7;
        }
    }

    if (ctrl_offset >= len) return false;
    frame.control = data[ctrl_offset];
    frame.extended = false;
    frame.control2 = 0;

    frame.pid = 0;
    frame.info.clear();

    Ax25FrameType ft = frame.type();
    // AX.25 2.2 modulo-128: I/S frames carry a 2-octet control; U-frames stay 1 octet.
    // The frame-type is decided from octet-1 (frame.control) exactly as in mod-8, so
    // the extended flag only shifts the PID/info offset (I) and captures octet-2 (I/S).
    bool ext_is = extended && (ft == Ax25FrameType::I_FRAME || ft == Ax25FrameType::S_FRAME);
    if (ext_is) {
        if (ctrl_offset + 1 >= len) return false;   // need the 2nd control octet
        frame.extended = true;
        frame.control2 = data[ctrl_offset + 1];
    }
    size_t ctrl_len = ext_is ? 2 : 1;

    if (ft == Ax25FrameType::I_FRAME) {
        if (ctrl_offset + ctrl_len < len) {
            frame.pid = data[ctrl_offset + ctrl_len];
            if (ctrl_offset + ctrl_len + 1 < len)
                frame.info.assign(data + ctrl_offset + ctrl_len + 1, data + len);
        }
    } else if (ft == Ax25FrameType::U_FRAME) {
        uint8_t masked = frame.control & ~AX25_PF_MASK;
        // UI and XID U-frames have PID + info fields
        if (masked == AX25_CTRL_UI || masked == AX25_CTRL_XID_U) {
            if (ctrl_offset + 1 < len) {
                frame.pid = data[ctrl_offset + 1];
                if (ctrl_offset + 2 < len)
                    frame.info.assign(data + ctrl_offset + 2, data + len);
            }
        }
    }

    return true;
}

Ax25Address ax25_make_addr(const std::string& callsign, uint8_t ssid) {
    Ax25Address addr;
    std::string base = callsign;
    uint8_t s = ssid;

    auto pos = callsign.find('-');
    if (pos != std::string::npos) {
        base = callsign.substr(0, pos);
        std::string ssid_str = callsign.substr(pos + 1);
        // Only parse numeric SSIDs (callsigns like "STN-A" are not SSID notation)
        if (!ssid_str.empty() && ssid_str.find_first_not_of("0123456789") == std::string::npos)
            s = (uint8_t)std::stoi(ssid_str);
        else
            base = callsign;  // Not a valid SSID — treat entire string as callsign
    }

    memset(addr.callsign, ' ', 6);
    addr.callsign[6] = '\0';
    size_t n = std::min(base.size(), (size_t)6);
    for (size_t i = 0; i < n; i++)
        addr.callsign[i] = base[i];
    addr.ssid = s;
    return addr;
}

std::vector<uint8_t> ax25_build_u(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t u_ctrl, bool pf, bool command,
                                   const Ax25ViaPath& via) {
    std::vector<uint8_t> frame;
    frame.reserve(15 + 7 * via.size());
    // Command: dst C=1, src C=0. Response: dst C=0, src C=1.
    encode_addr(frame, dst, command, false);
    encode_addr(frame, src, !command, via.empty());
    encode_via(frame, via);
    frame.push_back(pf ? (u_ctrl | AX25_PF_MASK) : u_ctrl);
    return frame;
}

std::vector<uint8_t> ax25_build_s(const Ax25Address& dst, const Ax25Address& src,
                                   Ax25SType stype, uint8_t nr, bool pf,
                                   bool command, bool extended,
                                   const Ax25ViaPath& via) {
    std::vector<uint8_t> frame;
    frame.reserve(16 + 7 * via.size());
    encode_addr(frame, dst, command, false);
    encode_addr(frame, src, !command, via.empty());
    encode_via(frame, via);
    if (extended) {
        // modulo-128 S-frame: octet-1 = 0x01|(SS<<2); octet-2 = (N(R)<<1)|P/F.
        frame.push_back(0x01 | ((uint8_t)stype << 2));
        frame.push_back((uint8_t)(((nr & 0x7F) << 1) | (pf ? 0x01 : 0x00)));
    } else {
        uint8_t ctrl = 0x01 | ((uint8_t)stype << 2) | ((nr & 0x07) << 5);
        if (pf) ctrl |= AX25_PF_MASK;
        frame.push_back(ctrl);
    }
    return frame;
}

std::vector<uint8_t> ax25_build_i(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t ns, uint8_t nr, bool pf, uint8_t pid,
                                   const uint8_t* info, size_t info_len,
                                   bool extended,
                                   const Ax25ViaPath& via) {
    std::vector<uint8_t> frame;
    frame.reserve(18 + info_len + 7 * via.size());
    // I-frames are always commands: dst C=1, src C=0
    encode_addr(frame, dst, true, false);
    encode_addr(frame, src, false, via.empty());
    encode_via(frame, via);
    if (extended) {
        // modulo-128 I-frame: octet-1 = (N(S)<<1) (b0=0 marks I); octet-2 = (N(R)<<1)|P/F.
        frame.push_back((uint8_t)((ns & 0x7F) << 1));
        frame.push_back((uint8_t)(((nr & 0x7F) << 1) | (pf ? 0x01 : 0x00)));
    } else {
        uint8_t ctrl = ((ns & 0x07) << 1) | ((nr & 0x07) << 5);
        if (pf) ctrl |= AX25_PF_MASK;
        frame.push_back(ctrl);
    }
    frame.push_back(pid);
    if (info && info_len > 0)
        frame.insert(frame.end(), info, info + info_len);
    return frame;
}

std::vector<uint8_t> ax25_build_frmr(const Ax25Address& dst, const Ax25Address& src,
                                      uint8_t rejected_ctrl, uint8_t vs, uint8_t vr,
                                      bool cr, bool w, bool x, bool y, bool z,
                                      const Ax25ViaPath& via) {
    std::vector<uint8_t> frame;
    frame.reserve(18 + 7 * via.size());  // 7+7+1+3 (+7 per via hop)
    // FRMR is a response
    encode_addr(frame, dst, false, false);
    encode_addr(frame, src, true, via.empty());
    encode_via(frame, via);
    frame.push_back(AX25_CTRL_FRMR | AX25_PF_MASK);  // Always P/F=1
    // 3-byte info field (AX.25 2.2 Section 4.3.10)
    frame.push_back(rejected_ctrl);
    frame.push_back(((vr & 0x07) << 5) | (cr ? 0x10 : 0) | ((vs & 0x07) << 1));
    uint8_t wxyz_byte = 0;
    if (w) wxyz_byte |= 0x01;
    if (x) wxyz_byte |= 0x02;
    if (y) wxyz_byte |= 0x04;
    if (z) wxyz_byte |= 0x08;
    frame.push_back(wxyz_byte);
    return frame;
}

} // namespace iris
