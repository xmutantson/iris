#include "native/xid.h"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <charconv>

namespace iris {

std::vector<uint8_t> xid_encode(const XidCapability& cap) {
    std::vector<uint8_t> data(8);
    data[0] = XID_MAGIC[0];  // 'I'
    data[1] = XID_MAGIC[1];  // 'R'
    data[2] = XID_MAGIC[2];  // 'I'
    data[3] = XID_MAGIC[3];  // 'S'
    data[4] = cap.version;
    data[5] = (cap.capabilities >> 8) & 0xFF;
    data[6] = cap.capabilities & 0xFF;
    data[7] = (uint8_t)cap.max_modulation;
    return data;
}

bool xid_decode(const uint8_t* data, size_t len, XidCapability& cap) {
    if (len < 8) return false;

    // Check magic
    if (data[0] != 'I' || data[1] != 'R' || data[2] != 'I' || data[3] != 'S')
        return false;

    uint8_t mod = data[7];
    if (mod > (uint8_t)Modulation::QAM32)
        return false;
    cap.version = data[4];
    cap.capabilities = ((uint16_t)data[5] << 8) | data[6];
    cap.max_modulation = (Modulation)mod;
    return true;
}

std::vector<uint8_t> build_xid_frame(const char* src, const char* dst,
                                      const XidCapability& cap) {
    std::vector<uint8_t> frame;
    frame.reserve(16 + 2 + 8);  // addr(14) + control+PID(2) + XID(8)

    // Destination address (shifted left 1, space-padded)
    for (int i = 0; i < 6; i++) {
        char c = (dst && i < (int)strlen(dst) && dst[i]) ? dst[i] : ' ';
        frame.push_back((uint8_t)c << 1);
    }
    frame.push_back(0xE0);  // SSID byte: command, SSID=0

    // Source address
    for (int i = 0; i < 6; i++) {
        char c = (src && i < (int)strlen(src) && src[i]) ? src[i] : ' ';
        frame.push_back((uint8_t)c << 1);
    }
    frame.push_back(0x61);  // SSID byte: last address, SSID=0

    // Control: XID frame (0xAF for command, 0xAF for response)
    frame.push_back(0xAF);

    // PID: reserved for Iris
    frame.push_back(IRIS_PID);

    // XID capability info
    auto xid_data = xid_encode(cap);
    frame.insert(frame.end(), xid_data.begin(), xid_data.end());

    return frame;
}

XidCapability negotiate(const XidCapability& local, const XidCapability& remote) {
    XidCapability result;
    result.version = std::min(local.version, remote.version);
    result.capabilities = local.capabilities & remote.capabilities;
    result.max_modulation = std::min(local.max_modulation, remote.max_modulation);
    return result;
}

// --- Connection header ---

std::vector<uint8_t> conn_header_encode(const XidCapability& cap) {
    // Format: "IRIS/1 caps=XXXX mod=N\r"
    // caps is 4-digit hex, mod is single digit (0-6)
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "IRIS/%u caps=%04X mod=%u\r",
                     cap.version, cap.capabilities, (unsigned)cap.max_modulation);
    return std::vector<uint8_t>(buf, buf + n);
}

bool conn_header_decode(const uint8_t* data, size_t len, XidCapability& cap) {
    if (len < 5) return false;

    // Must start with "IRIS/"
    if (memcmp(data, CONN_HEADER_PREFIX, 5) != 0) return false;

    // Parse: "IRIS/<version> caps=<hex> mod=<digit>\r"
    const char* p = (const char*)data + 5;
    const char* end = (const char*)data + len;
    unsigned ver = 0, caps = 0, mod = 0;
    auto result = std::from_chars(p, end, ver);
    if (result.ec != std::errc() || end - result.ptr < 6 ||
        memcmp(result.ptr, " caps=", 6) != 0) return false;

    p = result.ptr + 6;
    result = std::from_chars(p, end, caps, 16);
    if (result.ec != std::errc() || end - result.ptr < 5 ||
        memcmp(result.ptr, " mod=", 5) != 0) return false;

    p = result.ptr + 5;
    result = std::from_chars(p, end, mod);
    if (result.ec != std::errc() || result.ptr == end ||
        *result.ptr != '\r' || result.ptr + 1 != end) return false;
    if (ver > 0xFF || caps > 0xFFFF ||
        mod > (unsigned)Modulation::QAM32) return false;

    cap.version = (uint8_t)ver;
    cap.capabilities = (uint16_t)caps;
    cap.max_modulation = (Modulation)mod;
    return true;
}

} // namespace iris
