#ifndef IRIS_CRC16_H
#define IRIS_CRC16_H

#include <cstdint>
#include <cstddef>

namespace iris {

// AX.25 uses CRC-CCITT (polynomial 0x8408, init 0xFFFF, reflected)
uint16_t crc16_ccitt(const uint8_t* data, size_t len);

} // namespace iris

#endif
