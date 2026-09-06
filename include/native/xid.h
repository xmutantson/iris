#ifndef IRIS_XID_H
#define IRIS_XID_H

#include "common/types.h"
#include "native/constellation.h"
#include <vector>
#include <cstdint>

namespace iris {

// XID capability frame format (8 bytes):
//   Magic: "IRIS" (4 bytes)
//   Version: 1 byte
//   Capabilities: 2 bytes (bitmask)
//   Max modulation: 1 byte (Modulation enum)

struct XidCapability {
    uint8_t version;
    uint16_t capabilities;
    Modulation max_modulation;
};

// Encode XID capability into an AX.25 XID frame info field
std::vector<uint8_t> xid_encode(const XidCapability& cap);

// Decode XID capability from AX.25 XID frame info field
// Returns true if valid Iris XID frame
bool xid_decode(const uint8_t* data, size_t len, XidCapability& cap);

// Build a complete AX.25 XID frame with Iris capability
// src/dst are 6-char callsigns (padded with spaces)
std::vector<uint8_t> build_xid_frame(const char* src, const char* dst,
                                      const XidCapability& cap);

// Negotiate capabilities between two peers
// Returns the intersection of capabilities and minimum max_modulation
XidCapability negotiate(const XidCapability& local, const XidCapability& remote);

// --- Connection header (replaces XID U-frame) ---
// Sent as I-frame payload: "IRIS/1 caps=XXXX mod=N\r"
// Non-Iris stations display it as harmless text.
// Iris peers parse it and negotiate native mode upgrade.

constexpr const char* CONN_HEADER_PREFIX = "IRIS/";

// Encode capability into a plaintext connection header string
std::vector<uint8_t> conn_header_encode(const XidCapability& cap);

// Try to decode a connection header from I-frame data.
// Returns true if the data starts with "IRIS/" and was successfully parsed.
bool conn_header_decode(const uint8_t* data, size_t len, XidCapability& cap);

} // namespace iris

#endif
