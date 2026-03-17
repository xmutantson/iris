#ifndef IRIS_OFDM_FRAME_H
#define IRIS_OFDM_FRAME_H

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"      // ToneMap struct
#include "probe/passband_probe.h"
#include "fec/ldpc.h"
#include <vector>
#include <cstdint>

namespace iris {

// Compute waterfill tone map from per-subcarrier linear SNR values.
// snr_per_carrier: linear SNR at each data carrier position (from probe or channel est).
// fec_rate: determines the SNR gap (how conservative bit loading is).
ToneMap compute_waterfill_tone_map(const std::vector<float>& snr_per_carrier,
                                    LdpcRate fec_rate, const OfdmConfig& config);

// Compute tone map from probe data (before OFDM training symbols are available).
// Interpolates probe tone powers to OFDM subcarrier frequencies and estimates
// per-carrier SNR from tone power vs. noise floor.
ToneMap compute_tone_map_from_probe(const ProbeResult& local_rx,
                                     const ProbeResult& remote_rx,
                                     LdpcRate fec_rate,
                                     const OfdmConfig& config);

// Get a predefined uniform tone map (all carriers same modulation).
// Preset IDs 1-8. Returns empty ToneMap on invalid ID.
ToneMap get_uniform_tone_map(uint8_t tone_map_id, const OfdmConfig& config);

// Compute total net throughput for a tone map (bits per second).
// n_codewords: LDPC codewords per frame (1=minimum, 8=typical for FM).
float tone_map_throughput(const ToneMap& map, const OfdmConfig& config, int n_codewords = 1);

// Serialize tone map to compact byte representation.
// Header (1 byte): [4-bit tone_map_id][4-bit fec_rate_field]
// For waterfill (id=0): one nibble per carrier (bits/2), two per byte.
// Total: 1 + ceil(n_data_carriers/2) bytes.
std::vector<uint8_t> serialize_tone_map(const ToneMap& map);

// Deserialize tone map from bytes. Returns false on malformed data.
bool deserialize_tone_map(const uint8_t* data, size_t len, ToneMap& map);

} // namespace iris

#endif // IRIS_OFDM_FRAME_H
