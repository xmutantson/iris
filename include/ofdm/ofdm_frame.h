#ifndef IRIS_OFDM_FRAME_H
#define IRIS_OFDM_FRAME_H

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"      // ToneMap struct
#include "probe/passband_probe.h"
#include "fec/ldpc.h"
#include "v2/frame_geometry.h"
#include <optional>
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
// Preset IDs 1-14 (preset = O-level + 1). Returns empty ToneMap on invalid ID.
ToneMap get_uniform_tone_map(uint8_t tone_map_id, const OfdmConfig& config);

// Single source of truth for the OFDM speed-level ladder (kUniformPresets in
// ofdm_frame.cc, kept 1:1 with OFDM_SPEED_LEVELS[]). Both the batch-sizing
// capacity path (modem.cc ofdm_level_to_fec_rate / ofdm_capacity_bytes_for_level)
// and the frame builder resolve FEC/constellation/codeword count from HERE so the
// sized payload and built frame describe the SAME level. A mismatch oversizes max_payload
// -> build_ofdm_frame rejects the frame -> the TX silently skips (the O7+
// 14-rung-vs-10-rung ladder desync). level is 0-based (O0..O13), clamped.
LdpcRate ofdm_level_fec_rate(int level);
int      ofdm_level_bits_per_carrier(int level);
int      ofdm_level_codeword_count(int level);

// Compute total net throughput for a tone map (bits per second).  The complete
// frame shape, including its codeword count, is owned by map.
float tone_map_throughput(const ToneMap& map, const OfdmConfig& config);
inline float tone_map_throughput(const ToneMap& map, const OfdmConfig& config,
                                 int asserted_codeword_count) {
    return asserted_codeword_count == map.n_codewords
        ? tone_map_throughput(map, config) : 0.0f;
}

// Checked bridge from the operational OFDM configuration to the authoritative
// RC6 geometry contract.  The ToneMap owns FEC and codeword count; callers must
// not carry a second, independently mutable copy of either value.
std::optional<v2::FrameGeometry> checked_ofdm_frame_geometry(
    const OfdmConfig& config, const ToneMap& map) noexcept;

// Geometry-derived helpers used by frame fit, live retention and reporting.
std::optional<std::uint64_t> ofdm_payload_capacity_bytes(
    const v2::FrameGeometry& geometry) noexcept;
std::uint64_t ofdm_samples_from_first_training(
    const v2::FrameGeometry& geometry) noexcept;
std::optional<std::uint64_t> maximum_legal_ofdm_frame_samples(
    const OfdmConfig& config,
    const std::vector<ToneMap>& additional_admitted_maps = {}) noexcept;

// Serialize tone map to compact byte representation.
// Header (1 byte): [4-bit tone_map_id][4-bit fec_rate_field]
// For waterfill (id=0): one nibble per carrier (bits/2), two per byte.
// Total: 1 + ceil(n_data_carriers/2) bytes.
std::vector<uint8_t> serialize_tone_map(const ToneMap& map);

// Deserialize tone map from bytes. Returns false on malformed data.
bool deserialize_tone_map(const uint8_t* data, size_t len, ToneMap& map);

} // namespace iris

#endif // IRIS_OFDM_FRAME_H
