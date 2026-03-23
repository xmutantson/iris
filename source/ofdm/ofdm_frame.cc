#include "ofdm/ofdm_frame.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstdio>
#include <cstring>
#include <string>

namespace iris {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// SNR gap (dB) for each FEC rate — determines how conservatively bits are loaded.
// Lower gap = more aggressive loading. Values from CCB analysis with LDPC @ BER 1e-5.
static float snr_gap_db(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2:  return 2.0f;
        case LdpcRate::RATE_5_8:  return 2.5f;
        case LdpcRate::RATE_3_4:  return 3.5f;
        case LdpcRate::RATE_7_8:  return 5.0f;
        default:                  return 3.0f;  // conservative default
    }
}

// FEC effective code rate as a float.
static float fec_code_rate(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2:  return 0.5f;
        case LdpcRate::RATE_5_8:  return 0.625f;
        case LdpcRate::RATE_3_4:  return 0.75f;
        case LdpcRate::RATE_7_8:  return 0.875f;
        default:                  return 0.5f;
    }
}

// Round raw bit count to a valid constellation order: 0,1,2,4,6,8.
// 3 → 2, 5 → 4, 7 → 6.
static uint8_t round_to_valid_bpc(int raw_bits) {
    if (raw_bits <= 0) return 0;
    if (raw_bits == 1) return 1;
    if (raw_bits <= 3) return 2;   // 2 or 3 → QPSK
    if (raw_bits <= 5) return 4;   // 4 or 5 → 16QAM
    if (raw_bits <= 7) return 6;   // 6 or 7 → 64QAM
    return 8;                      // 8+ → 256QAM
}

// Encode FEC rate into a 4-bit field for serialization.
static uint8_t fec_rate_to_field(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2:  return 0;
        case LdpcRate::RATE_5_8:  return 1;
        case LdpcRate::RATE_3_4:  return 2;
        case LdpcRate::RATE_7_8:  return 3;
        default:                  return 0;
    }
}

// Decode 4-bit field back to LdpcRate.
static LdpcRate field_to_fec_rate(uint8_t field) {
    switch (field) {
        case 0:  return LdpcRate::RATE_1_2;
        case 1:  return LdpcRate::RATE_5_8;
        case 2:  return LdpcRate::RATE_3_4;
        case 3:  return LdpcRate::RATE_7_8;
        default: return LdpcRate::RATE_1_2;
    }
}

// Build a compact string representation of bits_per_carrier for logging.
static std::string bits_to_string(const std::vector<uint8_t>& bpc) {
    std::string s;
    for (size_t i = 0; i < bpc.size(); i++) {
        if (i > 0) s += ',';
        s += std::to_string(bpc[i]);
    }
    return s;
}

// FEC rate as N/16 for logging.
static int fec_rate_n16(LdpcRate rate) {
    switch (rate) {
        case LdpcRate::RATE_1_2:  return 8;
        case LdpcRate::RATE_5_8:  return 10;
        case LdpcRate::RATE_3_4:  return 12;
        case LdpcRate::RATE_7_8:  return 14;
        default:                  return 8;
    }
}

// Linear interpolation: given arrays xs[n], ys[n], compute y at position x.
static float lerp_lookup(const float* xs, const float* ys, int n, float x) {
    if (n <= 0) return 0.0f;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    for (int i = 0; i < n - 1; i++) {
        if (x >= xs[i] && x <= xs[i + 1]) {
            float t = (x - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    return ys[n - 1];
}

// ---------------------------------------------------------------------------
// Predefined uniform tone map table
// ---------------------------------------------------------------------------

struct UniformPreset {
    uint8_t id;
    uint8_t bits_per_carrier;  // same for all carriers
    LdpcRate fec_rate;
};

static const UniformPreset kUniformPresets[] = {
    { 1, 1, LdpcRate::RATE_1_2 },  // O0: BPSK r1/2
    { 2, 2, LdpcRate::RATE_1_2 },  // O1: QPSK r1/2
    { 3, 2, LdpcRate::RATE_3_4 },  // O2: QPSK r3/4
    { 4, 4, LdpcRate::RATE_1_2 },  // O3: 16QAM r1/2
    { 5, 4, LdpcRate::RATE_5_8 },  // O4: 16QAM r5/8
    { 6, 4, LdpcRate::RATE_3_4 },  // O5: 16QAM r3/4
    { 7, 6, LdpcRate::RATE_5_8 },  // O6: 64QAM r5/8
    { 8, 6, LdpcRate::RATE_3_4 },  // O7: 64QAM r3/4
    { 9, 8, LdpcRate::RATE_5_8 },  // O8: 256QAM r5/8
    {10, 8, LdpcRate::RATE_3_4 },  // O9: 256QAM r3/4
};
static constexpr int kNumPresets = sizeof(kUniformPresets) / sizeof(kUniformPresets[0]);

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

ToneMap compute_waterfill_tone_map(const std::vector<float>& snr_per_carrier,
                                    LdpcRate fec_rate, const OfdmConfig& config) {
    ToneMap map;
    map.nfft = config.nfft;
    map.fec_rate = fec_rate;
    map.tone_map_id = 0;  // waterfill
    map.n_data_carriers = static_cast<int>(snr_per_carrier.size());
    map.bits_per_carrier.resize(map.n_data_carriers, 0);
    map.total_bits_per_symbol = 0;

    const float gap_db = snr_gap_db(fec_rate);
    const float gap_lin = std::pow(10.0f, gap_db / 10.0f);

    for (int k = 0; k < map.n_data_carriers; k++) {
        float snr = snr_per_carrier[k];
        if (snr <= 0.0f) {
            map.bits_per_carrier[k] = 0;
            continue;
        }
        int raw_bits = static_cast<int>(std::floor(std::log2(1.0f + snr / gap_lin)));
        if (raw_bits > 8) raw_bits = 8;
        map.bits_per_carrier[k] = round_to_valid_bpc(raw_bits);
        map.total_bits_per_symbol += map.bits_per_carrier[k];
    }

    float throughput = tone_map_throughput(map, config);

    IRIS_LOG("[OFDM-WF] waterfill: %d carriers, bits=[%s], total=%d bits/sym, rate=%d/16, throughput=%.0f bps",
             map.n_data_carriers,
             bits_to_string(map.bits_per_carrier).c_str(),
             map.total_bits_per_symbol,
             fec_rate_n16(fec_rate),
             throughput);

    return map;
}

ToneMap compute_tone_map_from_probe(const ProbeResult& local_rx,
                                     const ProbeResult& remote_rx,
                                     LdpcRate fec_rate,
                                     const OfdmConfig& config) {
    // Use the worse of local_rx and remote_rx for each subcarrier (conservative).
    // Probe tones are at probe_tone_freq(i) with spacing ~66.7 Hz.
    // OFDM data subcarriers are at bin_to_freq(data_carrier_bins[k]).
    // Interpolate probe tone power to OFDM subcarrier positions.

    const int n_tones = PassbandProbeConfig::N_TONES;

    // Build probe frequency/power arrays for both directions
    float probe_freq[PassbandProbeConfig::N_TONES];
    float local_power[PassbandProbeConfig::N_TONES];
    float remote_power[PassbandProbeConfig::N_TONES];

    for (int i = 0; i < n_tones; i++) {
        probe_freq[i] = probe_tone_freq(i);
        local_power[i] = local_rx.tone_power_db[i];
        remote_power[i] = remote_rx.tone_power_db[i];
    }

    // Estimate noise floor: median of power in bins between probe tones (the nulls).
    // For each pair of adjacent detected tones, the power at the midpoint frequency
    // approximates the noise floor. We collect these and take the median.
    std::vector<float> noise_samples_local;
    std::vector<float> noise_samples_remote;

    for (int i = 0; i < n_tones - 1; i++) {
        if (local_rx.tone_detected[i] && local_rx.tone_detected[i + 1]) {
            // Midpoint power — approximate as min of adjacent tones minus typical
            // tone-to-noise ratio for a clean probe. In practice the null bin power
            // is ~20-30 dB below the tone power. We use the lower of the two tones
            // minus 20 dB as a conservative noise floor estimate.
            float mid_power = std::min(local_power[i], local_power[i + 1]) - 20.0f;
            noise_samples_local.push_back(mid_power);
        }
        if (remote_rx.tone_detected[i] && remote_rx.tone_detected[i + 1]) {
            float mid_power = std::min(remote_power[i], remote_power[i + 1]) - 20.0f;
            noise_samples_remote.push_back(mid_power);
        }
    }

    // Median noise floor
    auto median = [](std::vector<float>& v) -> float {
        if (v.empty()) return -40.0f;  // default noise floor if no data
        std::sort(v.begin(), v.end());
        size_t mid = v.size() / 2;
        if (v.size() % 2 == 0)
            return (v[mid - 1] + v[mid]) / 2.0f;
        return v[mid];
    };

    float noise_floor_local = median(noise_samples_local);
    float noise_floor_remote = median(noise_samples_remote);

    // Interpolate probe tone power to each OFDM data subcarrier frequency
    // and compute per-carrier linear SNR (minimum of both directions).
    int n_data = config.n_data_carriers;
    std::vector<float> snr_per_carrier(n_data, 0.0f);

    for (int k = 0; k < n_data; k++) {
        float freq = bin_to_freq(config.data_carrier_bins[k], config.nfft, config.sample_rate);

        float local_interp = lerp_lookup(probe_freq, local_power, n_tones, freq);
        float remote_interp = lerp_lookup(probe_freq, remote_power, n_tones, freq);

        float snr_db_local = local_interp - noise_floor_local;
        float snr_db_remote = remote_interp - noise_floor_remote;

        // Use the worse direction (conservative — both TX and RX paths must work)
        float snr_db = std::min(snr_db_local, snr_db_remote);
        if (snr_db < 0.0f) snr_db = 0.0f;

        snr_per_carrier[k] = std::pow(10.0f, snr_db / 10.0f);
    }

    IRIS_LOG("[OFDM-WF] probe→waterfill: %d data carriers, noise floor local=%.1f dB remote=%.1f dB",
             n_data, noise_floor_local, noise_floor_remote);

    return compute_waterfill_tone_map(snr_per_carrier, fec_rate, config);
}

ToneMap get_uniform_tone_map(uint8_t tone_map_id, const OfdmConfig& config) {
    ToneMap map;
    map.nfft = config.nfft;
    map.n_data_carriers = config.n_data_carriers;
    map.tone_map_id = tone_map_id;

    // Find preset
    const UniformPreset* preset = nullptr;
    for (int i = 0; i < kNumPresets; i++) {
        if (kUniformPresets[i].id == tone_map_id) {
            preset = &kUniformPresets[i];
            break;
        }
    }

    if (!preset) {
        IRIS_LOG("[OFDM-WF] invalid uniform tone map id %d, falling back to BPSK r1/2", tone_map_id);
        preset = &kUniformPresets[0];  // fallback to BPSK r1/2
        map.tone_map_id = 1;
    }

    map.fec_rate = preset->fec_rate;
    map.bits_per_carrier.assign(map.n_data_carriers, preset->bits_per_carrier);
    map.total_bits_per_symbol = map.n_data_carriers * preset->bits_per_carrier;

    float throughput = tone_map_throughput(map, config);

    IRIS_LOG("[OFDM-WF] uniform tone map %d: %d carriers × %d bpc, rate=%d/16, throughput=%.0f bps",
             map.tone_map_id, map.n_data_carriers, preset->bits_per_carrier,
             fec_rate_n16(map.fec_rate), throughput);

    return map;
}

float tone_map_throughput(const ToneMap& map, const OfdmConfig& config, int n_codewords) {
    if (map.total_bits_per_symbol <= 0) return 0.0f;

    float symbols_per_sec = config.symbol_rate();

    // Overhead estimation: training(2) + header(n_hdr) + block pilots + tail(1)
    // n_codewords determines how many LDPC blocks per frame (more = less overhead).
    int coded_bits_per_block = 1600;
    int coded_bits_total = n_codewords * coded_bits_per_block;
    int n_data_symbols = (coded_bits_total + map.total_bits_per_symbol - 1) / map.total_bits_per_symbol;

    // Block pilots inserted every pilot_symbol_spacing data symbols
    int n_block_pilots = (n_data_symbols > 0)
        ? (n_data_symbols - 1) / config.pilot_symbol_spacing
        : 0;
    // Dense pilot rows inserted every pilot_row_spacing data symbols
    int n_pilot_rows = (config.pilot_row_spacing > 0 && n_data_symbols > 0)
        ? (n_data_symbols - 1) / config.pilot_row_spacing
        : 0;
    int n_overhead = 2 + config.n_header_symbols + n_block_pilots + n_pilot_rows + 1;
    int n_total = n_data_symbols + n_overhead;

    float data_fraction = static_cast<float>(n_data_symbols) / static_cast<float>(n_total);
    float rate = fec_code_rate(map.fec_rate);

    return static_cast<float>(map.total_bits_per_symbol) * symbols_per_sec * data_fraction * rate;
}

std::vector<uint8_t> serialize_tone_map(const ToneMap& map) {
    std::vector<uint8_t> out;

    // Header byte: [4-bit tone_map_id][4-bit fec_rate_field]
    uint8_t header = static_cast<uint8_t>((map.tone_map_id & 0x0F) << 4)
                   | (fec_rate_to_field(map.fec_rate) & 0x0F);
    out.push_back(header);

    if (map.tone_map_id == 0) {
        // Waterfill: one nibble per carrier (bits/2 fits in 4 bits: 0..4)
        // Pack two nibbles per byte, high nibble first.
        int n = map.n_data_carriers;
        for (int i = 0; i < n; i += 2) {
            uint8_t hi = map.bits_per_carrier[i] / 2;
            uint8_t lo = (i + 1 < n) ? (map.bits_per_carrier[i + 1] / 2) : 0;
            out.push_back(static_cast<uint8_t>((hi << 4) | (lo & 0x0F)));
        }
    }
    // For uniform maps (id 1-8), the header byte alone is sufficient — the
    // receiver reconstructs from the preset table + its own n_data_carriers.

    // Trailing byte: n_codewords (0 = not present / old peer, treated as 1)
    out.push_back(static_cast<uint8_t>(map.n_codewords));

    return out;
}

bool deserialize_tone_map(const uint8_t* data, size_t len, ToneMap& map) {
    if (!data || len < 1) return false;

    uint8_t header = data[0];
    map.tone_map_id = (header >> 4) & 0x0F;
    map.fec_rate = field_to_fec_rate(header & 0x0F);

    if (map.tone_map_id == 0) {
        // Waterfill: nibble-packed carrier data follows
        size_t payload_bytes = len - 1;
        int max_carriers = static_cast<int>(payload_bytes * 2);
        map.bits_per_carrier.clear();
        map.total_bits_per_symbol = 0;

        for (size_t i = 0; i < payload_bytes; i++) {
            uint8_t byte = data[1 + i];
            uint8_t hi_nibble = (byte >> 4) & 0x0F;
            uint8_t lo_nibble = byte & 0x0F;

            uint8_t bpc_hi = static_cast<uint8_t>(hi_nibble * 2);
            map.bits_per_carrier.push_back(bpc_hi);
            map.total_bits_per_symbol += bpc_hi;

            // Second nibble (only if we haven't run past an odd carrier count)
            uint8_t bpc_lo = static_cast<uint8_t>(lo_nibble * 2);
            map.bits_per_carrier.push_back(bpc_lo);
            map.total_bits_per_symbol += bpc_lo;
        }

        map.n_data_carriers = static_cast<int>(map.bits_per_carrier.size());
    } else {
        // Uniform: reconstruct from preset. n_data_carriers must be set
        // by the caller from the OfdmConfig — we leave bits_per_carrier empty
        // to signal that the caller should call get_uniform_tone_map() with
        // the received tone_map_id and their local config.
        map.n_data_carriers = 0;
        map.bits_per_carrier.clear();
        map.total_bits_per_symbol = 0;
    }

    // Trailing byte: n_codewords (optional, old peers don't send it)
    // For waterfill: after header + (n_carriers+1)/2 carrier bytes
    // For uniform: after header byte (1 byte)
    size_t expected_prefix = 1;  // header byte
    if (map.tone_map_id == 0) {
        expected_prefix += (map.n_data_carriers + 1) / 2;  // carrier nibbles
    }
    if (len > expected_prefix) {
        map.n_codewords = data[expected_prefix];
        if (map.n_codewords < 1) map.n_codewords = 1;
    } else {
        map.n_codewords = 1;  // old peer or missing
    }

    return true;
}

} // namespace iris
