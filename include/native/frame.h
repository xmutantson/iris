#ifndef IRIS_FRAME_H
#define IRIS_FRAME_H

#include "native/phy.h"
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
//   [Header: 32 bits BPSK — modulation, payload_len, header_crc8]
//   [Payload: N symbols at configured modulation]
//   [CRC-32: 32 bits appended to payload before mapping]

// Preamble m-sequence (length 63, LFSR polynomial x^6+x+1)
std::vector<std::complex<float>> generate_preamble();

// 16-symbol sync word
std::vector<std::complex<float>> generate_sync_word();

// Header: encodes modulation type and payload byte count
// Format: [4 bits modulation][12 bits payload_len][8 bits reserved][8 bits CRC-8]
std::vector<uint8_t> encode_header(Modulation mod, uint16_t payload_len);
bool decode_header(const std::vector<uint8_t>& bits, Modulation& mod, uint16_t& payload_len);

// Build a complete native frame from payload bytes
// Returns IQ samples ready for transmission
std::vector<float> build_native_frame(const uint8_t* payload, size_t len,
                                       const PhyConfig& config);

// Frame detector: correlate against preamble/sync to find frame start
// Returns sample offset of frame start, or -1 if not found
int detect_frame_start(const float* iq_samples, size_t count, int sps);

// Decode a native frame from IQ samples starting at frame_start
// Returns true if frame decoded successfully, fills payload
bool decode_native_frame(const float* iq_samples, size_t count,
                          int frame_start, const PhyConfig& default_config,
                          std::vector<uint8_t>& payload);

} // namespace iris

#endif
