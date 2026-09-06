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
constexpr int     NATIVE_MAX_PAYLOAD = 4000;  // Max native frame payload bytes

// Multi-payload frame: packs multiple KISS payloads into one over-the-air frame
// Format: [0xBB magic][1-byte epoch][2-byte LE len][data]...[2-byte LE len][data]...
// The epoch byte (forward burst-epoch, & 0x07) is shared once per burst; each
// sub-frame then carries its own 2-byte little-endian length prefix.
constexpr uint8_t MULTI_PAYLOAD_MAGIC = 0xBB;

// OFDM-KISS framing overhead — the bytes wrapped around a single AX.25 I-frame's
// info field before it is measured against the LDPC single-block capacity
// (ofdm_capacity_bytes_for_level, source/engine/modem.cc). Naming these makes the
// frame-sizing invariant explicit so it can never silently drift again: a
// max_info-sized I-frame MUST frame to exactly the level capacity, never one byte
// over. The pre-fix "-19" under-counted this by 1 (it omitted the shared epoch
// byte), so every 75-B O0 info frame framed to 95 > 94 and was dropped to the
// 800-baud AFSK fallback.
//   AX.25 I-frame header a session prepends to info:
//     2*AX25_ADDR_LEN (dest+src, no digipeaters) + 1 control + 1 PID = 16 B
//   OFDM-KISS multi-payload wrapper a lone sub-frame carries:
//     1 magic + 1 epoch (shared once per burst) + 2 sub-frame LE length = 4 B
constexpr int AX25_IFRAME_HEADER_BYTES = 2 * AX25_ADDR_LEN + 1 /*control*/ + 1 /*PID*/;  // 16
constexpr int OFDM_KISS_WRAPPER_HDR    = 1 /*magic*/ + 1 /*epoch*/;                       // 2, once per burst
constexpr int OFDM_KISS_SUBFRAME_HDR   = 2 /*LE length*/;                                 // 2, per sub-frame
// Overhead of framing ONE AX.25 I-frame alone (the max_info bound):
constexpr int OFDM_KISS_FRAMING_OVERHEAD =
    OFDM_KISS_WRAPPER_HDR + OFDM_KISS_SUBFRAME_HDR + AX25_IFRAME_HEADER_BYTES;             // 20

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
constexpr uint16_t CAP_HARQ        = 0x0100;  // Per-symbol soft HARQ with piggybacked retx
constexpr uint16_t CAP_OFDM        = 0x0200;  // OFDM PHY capable

// OFDM-KISS transport layer markers
constexpr uint8_t TUNE_REPORT_MAGIC = 0xBB;         // TUNE ramp report embedded in OFDM frame
constexpr uint8_t COMPRESSED_PAYLOAD_MAGIC = 0xCC;  // Batch-compressed OFDM-KISS payload
constexpr uint8_t B2F_DATA_MAGIC = 0xCD;            // B2F proxy data frame
constexpr size_t  B2F_BUFFER_SIZE = 2 * 1024 * 1024; // 2MB B2F proxy buffer

// ML-KEM key exchange frame markers (sent as ARQ DATA payloads)
constexpr uint8_t MLKEM_PK_MAGIC = 0xE1;   // ML-KEM encapsulation key (1184 bytes)
constexpr uint8_t MLKEM_CT_MAGIC = 0xE2;   // ML-KEM ciphertext (1088 bytes)

} // namespace iris

#endif // IRIS_TYPES_H
