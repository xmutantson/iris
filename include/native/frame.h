#ifndef IRIS_FRAME_H
#define IRIS_FRAME_H

#include "native/phy.h"
#include "fec/ldpc.h"
#include "common/types.h"
#include <vector>
#include <cstdint>
#include <complex>

namespace iris {

// CRC-32 for native frames
uint32_t crc32(const uint8_t* data, size_t len);

// Frame structure:
//   [Preamble: 63-symbol m-sequence (BPSK)]
//   [Sync word: 16-symbol Barker-like (BPSK)]
//   [Header: 32 bits BPSK — modulation, payload_len, fec_rate, header_crc8]
//   [Payload: N symbols at configured modulation, LDPC-protected]
//     For 16QAM and above: known BPSK pilot inserted every PILOT_SPACING symbols
//   [CRC-32: 32 bits appended to payload before FEC encoding]

// Mid-frame pilot spacing (symbols between pilots, 0 = disabled)
// Pilots are known +1 BPSK symbols inserted after constellation mapping.
// Active for 16QAM and above where phase drift is critical.
// QAM16+ uses tighter spacing (16) for better phase/gain tracking;
// BPSK/QPSK uses wider spacing (32) when pilots are needed (long frames).
constexpr int PILOT_SPACING_QAM = 16;   // QAM16 and above: 1 pilot every 16 symbols (~6% overhead)
constexpr int PILOT_SPACING_LOW = 32;   // BPSK/QPSK long frames: 1 pilot every 32 symbols

// Returns the pilot spacing for the given modulation
inline int pilot_spacing_for(int mod_int) {
    return (mod_int >= 2) ? PILOT_SPACING_QAM : PILOT_SPACING_LOW;  // mod>=2 is QAM16+
}

// Legacy constant for backward reference
constexpr int PILOT_SPACING = 16;  // default (tightest)

// Preamble m-sequence (length 63, LFSR polynomial x^6+x+1)
std::vector<std::complex<float>> generate_preamble();

// 16-symbol sync word
std::vector<std::complex<float>> generate_sync_word();

// Header: encodes modulation type, payload byte count, and FEC rate
// Format: [4 bits modulation][12 bits payload_len][2 bits fec_rate][6 bits reserved][8 bits CRC-8]
std::vector<uint8_t> encode_header(Modulation mod, uint16_t payload_len, LdpcRate fec = LdpcRate::NONE);
bool decode_header(const std::vector<uint8_t>& bits, Modulation& mod, uint16_t& payload_len, LdpcRate& fec);

// Build a complete native frame from payload bytes
// Returns IQ samples ready for transmission
std::vector<float> build_native_frame(const uint8_t* payload, size_t len,
                                       const PhyConfig& config,
                                       LdpcRate fec = LdpcRate::RATE_1_2);

// Frame detector: correlate against preamble/sync to find frame start
// Returns sample offset of frame start, or -1 if not found
int detect_frame_start(const float* iq_samples, size_t count, int sps);

// Get the best normalized correlation from the last detect_frame_start() call
float detect_best_corr();

// Decode a native frame from IQ samples starting at frame_start
// Returns true if frame decoded successfully, fills payload
bool decode_native_frame(const float* iq_samples, size_t count,
                          int frame_start, const PhyConfig& default_config,
                          std::vector<uint8_t>& payload);

// Returns true if last decode_native_frame failed due to buffer overflow
// (not enough IQ data), as opposed to a real CRC/decode error.
bool decode_was_overflow();

// Returns IQ pairs consumed by last successful decode (for buffer drain)
size_t decode_consumed_iq();

// Returns SNR (dB) estimated from last decoded frame's phase-corrected preamble
float decode_snr_db();

} // namespace iris

#endif
