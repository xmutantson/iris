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

// Parsed AX.25 frame
struct Ax25Frame {
    Ax25Address dst;
    Ax25Address src;
    uint8_t control;
    uint8_t pid;          // Valid for I and UI frames
    std::vector<uint8_t> info;

    Ax25FrameType type() const {
        if ((control & 0x01) == 0) return Ax25FrameType::I_FRAME;
        if ((control & 0x03) == 0x01) return Ax25FrameType::S_FRAME;
        return Ax25FrameType::U_FRAME;
    }

    Ax25UType u_type() const {
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

    bool poll_final() const { return (control & AX25_PF_MASK) != 0; }
    uint8_t nr() const { return (control >> 5) & 0x07; }  // N(R) from I or S frames
    uint8_t ns() const { return (control >> 1) & 0x07; }  // N(S) from I frames
};

// Parse raw AX.25 frame (after HDLC decode, FCS stripped)
bool ax25_parse(const uint8_t* data, size_t len, Ax25Frame& frame);

// Build an address from "CALL" or "CALL-N" string
Ax25Address ax25_make_addr(const std::string& callsign, uint8_t ssid = 0);

// Build raw AX.25 frame bytes (ready for HDLC encode)
// command: true = command frame (SABM, DISC, I), false = response (UA, DM)
std::vector<uint8_t> ax25_build_u(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t u_ctrl, bool pf, bool command = true);

std::vector<uint8_t> ax25_build_s(const Ax25Address& dst, const Ax25Address& src,
                                   Ax25SType stype, uint8_t nr, bool pf,
                                   bool command = true);

std::vector<uint8_t> ax25_build_i(const Ax25Address& dst, const Ax25Address& src,
                                   uint8_t ns, uint8_t nr, bool pf, uint8_t pid,
                                   const uint8_t* info, size_t info_len);

// Build FRMR frame with 3-byte info field (modulo 8)
// w=invalid ctrl, x=info when not allowed, y=info too long, z=invalid N(R)
std::vector<uint8_t> ax25_build_frmr(const Ax25Address& dst, const Ax25Address& src,
                                      uint8_t rejected_ctrl, uint8_t vs, uint8_t vr,
                                      bool cr, bool w, bool x, bool y, bool z);

} // namespace iris

#endif // IRIS_AX25_PROTOCOL_H
