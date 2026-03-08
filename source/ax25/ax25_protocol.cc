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
    auto pos = call.find('-');
    if (pos != std::string::npos) {
        std::string ssid_str = call.substr(pos + 1);
        if (!ssid_str.empty() && ssid_str.find_first_not_of("0123456789") == std::string::npos) {
            base = call.substr(0, pos);
            s = (uint8_t)std::stoi(ssid_str);
        }
        // else: not a numeric SSID, use full string as callsign
    }

    // Compare (padded to 6 with spaces)
    for (int i = 0; i < 6; i++) {
        char expected = (i < (int)base.size()) ? base[i] : ' ';
        if (callsign[i] != expected) return false;
    }
    return ssid == s;
}

// Encode a 7-byte AX.25 address into frame bytes
static void encode_addr(std::vector<uint8_t>& out, const Ax25Address& addr,
                         bool c_bit, bool last) {
    for (int i = 0; i < 6; i++)
        out.push_back((uint8_t)(addr.callsign[i]) << 1);
    uint8_t ssid_byte = (addr.ssid << 1) | 0x60;  // Reserved bits 5-6 set
    if (c_bit)  ssid_byte |= 0x80;  // Command/Response bit
    if (last)   ssid_byte |= 0x01;  // Extension bit (last address)
    out.push_back(ssid_byte);
}

// Decode a 7-byte AX.25 address from frame bytes
static void decode_addr(const uint8_t* data, Ax25Address& addr) {
    for (int i = 0; i < 6; i++)
        addr.callsign[i] = (char)(data[i] >> 1);
    addr.callsign[6] = '\0';
    addr.ssid = (data[6] >> 1) & 0x0F;
}

bool ax25_parse(const uint8_t* data, size_t len, Ax25Frame& frame) {
    if (len < 15) return false;  // dst(7) + src(7) + control(1)

    decode_addr(data, frame.dst);
    decode_addr(data + 7, frame.src);

    // Find control field (skip digipeater addresses)
    size_t ctrl_offset = 14;
    if ((data[13] & 0x01) == 0) {
        // More addresses follow (digipeaters)
        size_t pos = 14;
        while (pos + 6 < len) {
            if (data[pos + 6] & 0x01) {
                ctrl_offset = pos + 7;
                break;
            }
            pos += 7;
        }
        if (ctrl_offset == 14 && pos + 6 < len)
            ctrl_offset = pos + 7;
    }

    if (ctrl_offset >= len) return false;
    frame.control = data[ctrl_offset];

    frame.pid = 0;
    frame.info.clear();

    Ax25FrameType ft = frame.type();
    if (ft == Ax25FrameType::I_FRAME) {
        if (ctrl_offset + 1 < len) {
            frame.pid = data[ctrl_offset + 1];
            if (ctrl_offset + 2 < len)
                frame.info.assign(data + ctrl_offset + 2, data + len);
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
                                   uint8_t u_ctrl, bool pf, bool command) {
    std::vector<uint8_t> frame;
    frame.reserve(15);
    // Command: dst C=1, src C=0. Response: dst C=0, src C=1.
    encode_addr(frame, dst, command, false);
    encode_addr(frame, src, !command, true);
    frame.push_back(pf ? (u_ctrl | AX25_PF_MASK) : u_ctrl);
    return frame;
}

std::vector<uint8_t> ax25_build_s(const Ax25Address& dst, const Ax25Address& src,
                                   Ax25SType stype, uint8_t nr, bool pf,
                                   bool command) {
    std::vector<uint8_t> frame;
    frame.reserve(15);
    encode_addr(frame, dst, command, false);
    encode_addr(frame, src, !command, true);
    uint8_t ctrl = 0x01 | ((uint8_t)stype << 2) | ((nr & 0x07) << 5);
    if (pf) ctrl |= AX25_PF_MASK;
    frame.push_back(ctrl);
    return frame;
}

std::vector<uint8_t> ax25_build_i(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t ns, uint8_t nr, bool pf, uint8_t pid,
                                   const uint8_t* info, size_t info_len) {
    std::vector<uint8_t> frame;
    frame.reserve(17 + info_len);
    // I-frames are always commands: dst C=1, src C=0
    encode_addr(frame, dst, true, false);
    encode_addr(frame, src, false, true);
    uint8_t ctrl = ((ns & 0x07) << 1) | ((nr & 0x07) << 5);
    if (pf) ctrl |= AX25_PF_MASK;
    frame.push_back(ctrl);
    frame.push_back(pid);
    if (info && info_len > 0)
        frame.insert(frame.end(), info, info + info_len);
    return frame;
}

std::vector<uint8_t> ax25_build_frmr(const Ax25Address& dst, const Ax25Address& src,
                                      uint8_t rejected_ctrl, uint8_t vs, uint8_t vr,
                                      bool cr, bool w, bool x, bool y, bool z) {
    std::vector<uint8_t> frame;
    frame.reserve(18);  // 7+7+1+3
    // FRMR is a response
    encode_addr(frame, dst, false, false);
    encode_addr(frame, src, true, true);
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
