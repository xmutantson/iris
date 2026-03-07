#ifndef IRIS_TYPES_H
#define IRIS_TYPES_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

namespace iris {

// Sample rate for all audio processing
constexpr int SAMPLE_RATE = 48000;

// AX.25 constants
constexpr uint8_t AX25_FLAG      = 0x7E;
constexpr uint8_t AX25_PID_NONE  = 0xF0;  // No layer 3
constexpr uint8_t IRIS_PID       = 0xBE;   // Reserved PID for Iris native
constexpr int     AX25_MAX_FRAME = 330;    // Max AX.25 frame bytes
constexpr int     AX25_ADDR_LEN  = 7;      // Callsign (6) + SSID (1)

// KISS constants
constexpr uint8_t KISS_FEND  = 0xC0;
constexpr uint8_t KISS_FESC  = 0xDB;
constexpr uint8_t KISS_TFEND = 0xDC;
constexpr uint8_t KISS_TFESC = 0xDD;
constexpr uint8_t KISS_CMD_DATA    = 0x00;
constexpr uint8_t KISS_CMD_TXDELAY = 0x01;
constexpr uint8_t KISS_CMD_P       = 0x02;
constexpr uint8_t KISS_CMD_SLOT    = 0x03;
constexpr uint8_t KISS_CMD_TXTAIL  = 0x04;
constexpr uint8_t KISS_CMD_DUPLEX  = 0x05;
constexpr uint8_t KISS_CMD_RETURN  = 0xFF;

// AFSK 1200 baud parameters
constexpr int AFSK_BAUD       = 1200;
constexpr int AFSK_MARK_FREQ  = 1200;  // Hz
constexpr int AFSK_SPACE_FREQ = 2200;  // Hz
constexpr int AFSK_SAMPLES_PER_BIT = SAMPLE_RATE / AFSK_BAUD;  // 40

// GFSK 9600 baud parameters
constexpr int GFSK_BAUD       = 9600;
constexpr int GFSK_SAMPLES_PER_BIT = SAMPLE_RATE / GFSK_BAUD;  // 5

// Iris native mode parameters
constexpr int IRIS_PREAMBLE_LEN = 63;   // m-sequence symbols
constexpr int IRIS_SYNC_LEN     = 16;   // sync word symbols
constexpr int IRIS_HEADER_LEN   = 32;   // header symbols (BPSK)

// XID capability frame
constexpr uint8_t XID_MAGIC[4] = {'I', 'R', 'I', 'S'};
constexpr uint8_t XID_VERSION  = 0x01;

// Capability bits (negotiated in CONNECT/CONNECT_ACK)
constexpr uint16_t CAP_MODE_A      = 0x0001;
constexpr uint16_t CAP_MODE_B      = 0x0002;
constexpr uint16_t CAP_MODE_C      = 0x0004;
constexpr uint16_t CAP_ENCRYPTION  = 0x0008;
constexpr uint16_t CAP_COMPRESSION = 0x0010;
constexpr uint16_t CAP_B2F_UNROLL  = 0x0020;
constexpr uint16_t CAP_STREAMING   = 0x0040;  // Streaming compression context
constexpr uint16_t CAP_PQ_CRYPTO   = 0x0080;  // Post-quantum ML-KEM-768

} // namespace iris

#endif // IRIS_TYPES_H
