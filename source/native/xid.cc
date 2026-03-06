#include "native/xid.h"
#include <cstring>
#include <algorithm>

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

    cap.version = data[4];
    cap.capabilities = ((uint16_t)data[5] << 8) | data[6];
    uint8_t mod = data[7];
    if (mod > (uint8_t)Modulation::QAM256)
        return false;
    cap.max_modulation = (Modulation)mod;
    return true;
}

std::vector<uint8_t> build_xid_frame(const char* src, const char* dst,
                                      const XidCapability& cap) {
    std::vector<uint8_t> frame;
    frame.reserve(16 + 2 + 8);  // addr(14) + control+PID(2) + XID(8)

    // Destination address (shifted left 1, space-padded)
    for (int i = 0; i < 6; i++) {
        char c = (dst && dst[i] && i < (int)strlen(dst)) ? dst[i] : ' ';
        frame.push_back((uint8_t)c << 1);
    }
    frame.push_back(0x60);  // SSID byte: command, SSID=0

    // Source address
    for (int i = 0; i < 6; i++) {
        char c = (src && src[i] && i < (int)strlen(src)) ? src[i] : ' ';
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

} // namespace iris
