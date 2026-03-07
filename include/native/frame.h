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
//   [CRC-32: 32 bits appended to payload before FEC encoding]

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

} // namespace iris

#endif
