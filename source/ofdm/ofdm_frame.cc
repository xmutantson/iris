#include "ofdm/ofdm_frame.h"
#include "ofdm/ofdm_sync.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstdio>
#include <cstring>
#include <limits>
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
        case LdpcRate::RATE_1_16: return 0.0625f;
        case LdpcRate::RATE_2_16: return 0.125f;
        case LdpcRate::RATE_3_16: return 0.1875f;
        case LdpcRate::RATE_4_16: return 0.25f;
        case LdpcRate::RATE_5_16: return 0.3125f;
        case LdpcRate::RATE_6_16: return 0.375f;
        case LdpcRate::RATE_1_2:  return 0.5f;
        case LdpcRate::RATE_5_8:  return 0.625f;
        case LdpcRate::RATE_3_4:  return 0.75f;
        case LdpcRate::RATE_7_8:  return 0.875f;
        default:                  return 0.0f;
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
    if (raw_bits <= 9) return 8;   // 8 or 9 → 256QAM
    return 10;                     // 10+ → 1024QAM
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

// KEEP IN SYNC with OFDM_SPEED_LEVELS[] (speed_level.cc) and presets[] in
// make_uniform_tone_map() (ofdm_mod.cc). preset id = O-level + 1.
static const UniformPreset kUniformPresets[] = {
    { 1, 1, LdpcRate::RATE_1_2 },  // O0: BPSK r1/2
    { 2, 2, LdpcRate::RATE_1_2 },  // O1: QPSK r1/2
    { 3, 2, LdpcRate::RATE_3_4 },  // O2: QPSK r3/4
    { 4, 4, LdpcRate::RATE_1_2 },  // O3: 16QAM r1/2
    { 5, 4, LdpcRate::RATE_5_8 },  // O4: 16QAM r5/8
    { 6, 4, LdpcRate::RATE_3_4 },  // O5: 16QAM r3/4
    { 7, 5, LdpcRate::RATE_5_8 },  // O6: 32QAM r5/8  (NEW: VARA FM narrow top gear)
    { 8, 6, LdpcRate::RATE_5_8 },  // O7: 64QAM r5/8
    { 9, 6, LdpcRate::RATE_3_4 },  // O8: 64QAM r3/4
    {10, 8, LdpcRate::RATE_5_8 },  // O9: 256QAM r5/8
    {11, 8, LdpcRate::RATE_3_4 },  // O10: 256QAM r3/4
    {12, 8, LdpcRate::RATE_7_8 },  // O11: 256QAM r7/8
    {13, 10, LdpcRate::RATE_3_4 }, // O12: 1024QAM r3/4
    {14, 10, LdpcRate::RATE_7_8 }, // O13: 1024QAM r7/8
};
static constexpr int kNumPresets = sizeof(kUniformPresets) / sizeof(kUniformPresets[0]);

// ---------------------------------------------------------------------------
// Single source of truth for the O-level ladder (see ofdm_frame.h). Array index
// == O-level (kUniformPresets[level].id == level + 1). Clamped so out-of-range
// levels resolve to the nearest valid rung rather than reading OOB.
// ---------------------------------------------------------------------------
LdpcRate ofdm_level_fec_rate(int level) {
    int i = std::clamp(level, 0, kNumPresets - 1);
    return kUniformPresets[i].fec_rate;
}

int ofdm_level_bits_per_carrier(int level) {
    int i = std::clamp(level, 0, kNumPresets - 1);
    return kUniformPresets[i].bits_per_carrier;
}

int ofdm_level_codeword_count(int level) {
    int i = std::clamp(level, 0, kNumPresets - 1);
    const int bpc = kUniformPresets[i].bits_per_carrier;
    if (i <= 1) return 1;
    if (bpc <= 2) return 2;
    if (bpc == 4) return 4;
    if (bpc <= 6) return 8;
    return 4;
}

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
        if (!std::isfinite(snr) || snr <= 0.0f) {
            map.bits_per_carrier[k] = 0;
            continue;
        }
        int raw_bits = static_cast<int>(std::floor(std::log2(1.0f + snr / gap_lin)));
        if (raw_bits > 10) raw_bits = 10;
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

    if (!local_rx.valid || !remote_rx.valid ||
        noise_samples_local.empty() || noise_samples_remote.empty()) {
        return compute_waterfill_tone_map(
            std::vector<float>(config.n_data_carriers, 0.0f), fec_rate, config);
    }

    // Median noise floor
    auto median = [](std::vector<float>& v) -> float {
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
    map.n_codewords = ofdm_level_codeword_count(map.tone_map_id - 1);

    float throughput = tone_map_throughput(map, config);

    IRIS_LOG("[OFDM-WF] uniform tone map %d: %d carriers × %d bpc, rate=%d/16, throughput=%.0f bps",
             map.tone_map_id, map.n_data_carriers, preset->bits_per_carrier,
             fec_rate_n16(map.fec_rate), throughput);

    return map;
}

std::optional<v2::FrameGeometry> checked_ofdm_frame_geometry(
    const OfdmConfig& config, const ToneMap& map) noexcept {
    if (config.sample_rate <= 0 || config.nfft <= 0 || config.cp_samples < 0 ||
        config.pilot_carrier_spacing <= 0 || config.pilot_row_spacing < 0 ||
        map.n_codewords < 1 ||
        map.n_codewords > static_cast<int>(v2::FrameGeometry::kMaxCodewordCount) ||
        map.nfft != config.nfft || map.n_data_carriers != config.n_data_carriers ||
        map.total_bits_per_symbol <= 0 ||
        map.bits_per_carrier.size() != config.data_carrier_bins.size() ||
        config.used_carrier_bins.size() != static_cast<size_t>(config.n_used_carriers) ||
        config.pilot_carrier_bins.size() != static_cast<size_t>(config.n_pilot_carriers) ||
        config.data_carrier_bins.size() != static_cast<size_t>(config.n_data_carriers))
        return std::nullopt;

    // The operational waveform treats bins as direct FFT-vector indices and
    // derives comb pilots from positions in used_carrier_bins. Validate that
    // exact emitted layout here before translating it into the v2 contract.
    // Negative fftshift-style bins are not valid indices on this bridge.
    std::vector<int> emitted_pilots;
    std::vector<int> emitted_data;
    emitted_pilots.reserve(config.used_carrier_bins.size());
    emitted_data.reserve(config.used_carrier_bins.size());
    for (size_t i = 0; i < config.used_carrier_bins.size(); ++i) {
        const int bin = config.used_carrier_bins[i];
        if (bin <= 0 || bin >= config.nfft / 2) return std::nullopt;
        if (i != 0 && config.used_carrier_bins[i - 1] >= bin)
            return std::nullopt;
        if ((i % static_cast<size_t>(config.pilot_carrier_spacing)) == 0)
            emitted_pilots.push_back(bin);
        else
            emitted_data.push_back(bin);
    }
    if (emitted_pilots != config.pilot_carrier_bins ||
        emitted_data != config.data_carrier_bins ||
        ofdm_tail_zc_root(config.n_used_carriers, map.n_codewords) == 0)
        return std::nullopt;

    std::uint64_t declared_bits = 0;
    for (std::uint8_t bpc : map.bits_per_carrier) {
        if (declared_bits > std::numeric_limits<std::uint64_t>::max() - bpc)
            return std::nullopt;
        declared_bits += bpc;
    }
    if (declared_bits != static_cast<std::uint64_t>(map.total_bits_per_symbol))
        return std::nullopt;

    v2::FrameToneMap checked_map;
    checked_map.used_carrier_bins.assign(config.used_carrier_bins.begin(),
                                         config.used_carrier_bins.end());
    checked_map.pilot_carrier_bins.assign(config.pilot_carrier_bins.begin(),
                                          config.pilot_carrier_bins.end());
    checked_map.data_carrier_bins.assign(config.data_carrier_bins.begin(),
                                         config.data_carrier_bins.end());
    checked_map.coded_bits_per_data_carrier = map.bits_per_carrier;
    checked_map.tone_map_id = map.tone_map_id;
    checked_map.use_non_uniform_constellations = map.use_nuc;

    v2::AdmittedFrameProfile profile;
    profile.profile_id.value = static_cast<std::uint16_t>(map.tone_map_id) + 1U;
    profile.tone_map = checked_map;
    profile.fec = map.fec_rate;
    profile.min_codeword_count = v2::FrameGeometry::kMinCodewordCount;
    profile.max_codeword_count = v2::FrameGeometry::kMaxCodewordCount;
    profile.nfft = static_cast<std::uint32_t>(config.nfft);
    profile.cyclic_prefix_samples = static_cast<std::uint32_t>(config.cp_samples);
    profile.pilot_row_spacing = static_cast<std::uint32_t>(config.pilot_row_spacing);
    profile.timing_margin_samples = static_cast<std::uint32_t>(config.cp_samples);

    // The admitted singleton still covers every legal C=1..8 for this actual
    // map.  Its shared maxima are therefore calculated at C=8, not at the
    // current ladder choice.
    const std::uint64_t b = static_cast<std::uint64_t>(map.total_bits_per_symbol);
    const std::uint64_t coded = v2::FrameGeometry::kCodedBitsPerCodeword *
                                v2::FrameGeometry::kMaxCodewordCount;
    const std::uint64_t d = (coded + b - 1) / b;
    const std::uint64_t p = config.pilot_row_spacing == 0
        ? 0 : (d - 1) / static_cast<std::uint64_t>(config.pilot_row_spacing);
    const std::uint64_t l = static_cast<std::uint64_t>(config.nfft) +
                            static_cast<std::uint64_t>(config.cp_samples);
    if (d > std::numeric_limits<std::uint64_t>::max() - p -
            v2::FrameGeometry::kFixedNonDataSymbols)
        return std::nullopt;
    const std::uint64_t symbols = v2::FrameGeometry::kFixedNonDataSymbols + d + p;
    if (l != 0 && symbols > std::numeric_limits<std::uint64_t>::max() / l)
        return std::nullopt;

    v2::AdmittedFrameProfileSet admitted;
    admitted.authoritative_sample_rate_hz =
        static_cast<std::uint32_t>(config.sample_rate);
    admitted.profiles.push_back(profile);
    admitted.maximum_data_symbol_count = d;
    admitted.maximum_complete_sample_count = symbols * l;
    admitted.maximum_first_training_remaining_samples =
        (symbols - 1) * l + profile.timing_margin_samples;
    admitted.maximum_timing_margin_samples = profile.timing_margin_samples;
    admitted.acquisition_overlap_samples = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(4 * l, std::numeric_limits<std::uint32_t>::max()));

    v2::FrameGeometryInput input;
    input.profile_id = profile.profile_id;
    input.tone_map = std::move(checked_map);
    input.fec = map.fec_rate;
    input.codeword_count = static_cast<std::uint8_t>(map.n_codewords);
    input.nfft = profile.nfft;
    input.cyclic_prefix_samples = profile.cyclic_prefix_samples;
    input.pilot_row_spacing = profile.pilot_row_spacing;
    if (v2::FrameGeometry::validate(input, admitted).error !=
        v2::FrameGeometryError::None)
        return std::nullopt;
    try {
        return v2::FrameGeometry(input, admitted);
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<std::uint64_t> ofdm_payload_capacity_bytes(
    const v2::FrameGeometry& geometry) noexcept {
    const int k = LdpcCodec::block_size(geometry.fec());
    if (k < 48 || (k % 8) != 0) return std::nullopt;
    const std::uint64_t per_block = static_cast<std::uint64_t>(k / 8 - 6);
    if (per_block > std::numeric_limits<std::uint64_t>::max() /
            geometry.codeword_count()) return std::nullopt;
    return per_block * geometry.codeword_count();
}

std::uint64_t ofdm_samples_from_first_training(
    const v2::FrameGeometry& geometry) noexcept {
    return geometry.complete_sample_count() - geometry.symbol_length_samples() +
           geometry.timing_margin_samples();
}

std::optional<std::uint64_t> maximum_legal_ofdm_frame_samples(
    const OfdmConfig& config,
    const std::vector<ToneMap>& additional_admitted_maps) noexcept {
    std::uint64_t maximum = 0;
    auto admit = [&](ToneMap map) {
        map.n_codewords = v2::FrameGeometry::kMaxCodewordCount;
        auto geometry = checked_ofdm_frame_geometry(config, map);
        if (!geometry) return false;
        maximum = std::max(maximum, geometry->complete_sample_count());
        return true;
    };
    for (int preset = 1; preset <= kNumPresets; ++preset) {
        ToneMap map = make_uniform_tone_map(static_cast<std::uint8_t>(preset),
                                            config.n_data_carriers, config.nfft);
        if (!admit(std::move(map))) return std::nullopt;
    }
    for (const auto& map : additional_admitted_maps)
        if (!admit(map)) return std::nullopt;
    const std::uint64_t overlap = 4U *
        static_cast<std::uint64_t>(config.symbol_samples());
    if (maximum > std::numeric_limits<std::uint64_t>::max() - overlap)
        return std::nullopt;
    return maximum + overlap;
}

float tone_map_throughput(const ToneMap& map, const OfdmConfig& config) {
    auto geometry = checked_ofdm_frame_geometry(config, map);
    auto capacity = geometry ? ofdm_payload_capacity_bytes(*geometry) : std::nullopt;
    if (!geometry || !capacity || geometry->complete_sample_count() == 0 ||
        geometry->authoritative_sample_rate_hz() == 0 ||
        fec_code_rate(geometry->fec()) <= 0.0f)
        return 0.0f;
    const double payload_bits = static_cast<double>(*capacity) * 8.0;
    const double airtime_seconds =
        static_cast<double>(geometry->complete_sample_count()) /
        static_cast<double>(geometry->authoritative_sample_rate_hz());
    return static_cast<float>(payload_bits / airtime_seconds);
}

std::vector<uint8_t> serialize_tone_map(const ToneMap& map) {
    std::vector<uint8_t> out;

    // Header byte: [4-bit tone_map_id][4-bit fec_rate_field]
    uint8_t header = static_cast<uint8_t>((map.tone_map_id & 0x0F) << 4)
                   | (fec_rate_to_field(map.fec_rate) & 0x0F);
    out.push_back(header);

    // Carrier count, little-endian
    uint16_t n = static_cast<uint16_t>(map.n_data_carriers);
    out.push_back(static_cast<uint8_t>(n & 0xFF));
    out.push_back(static_cast<uint8_t>(n >> 8));

    if (map.tone_map_id == 0) {
        // Waterfill: one nibble per carrier
        // Pack two nibbles per byte, high nibble first.
        for (uint16_t i = 0; i < n; i += 2) {
            uint8_t hi = map.bits_per_carrier[i];
            uint8_t lo = (i + 1 < n) ? map.bits_per_carrier[i + 1] : 0;
            out.push_back(static_cast<uint8_t>((hi << 4) | (lo & 0x0F)));
        }
    }

    // Trailing byte: n_codewords
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
        // Upper bound: a single OFDM frame can't plausibly carry more than
        // ~8 LDPC codewords given the 1600-bit codeword size, max bits/sym
        // of 380 (NFFT=1024, QAM256 across all carriers), and ~100 max data
        // symbols. Garbage frame headers (from false-positive sync on
        // legacy PSK audio) were previously setting n_codewords up to 255
        // via the raw byte cast, blowing up downstream buffer allocations
        // and crashing the decoder (stack smashing).
        if (map.n_codewords > 8) map.n_codewords = 8;
    } else {
        map.n_codewords = 1;  // old peer or missing
    }

    return true;
}

} // namespace iris
