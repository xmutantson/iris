#include "ofdm/ofdm_config.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace iris {

int freq_to_bin(float freq_hz, int nfft, int sample_rate) {
    // FFT bin index for a given frequency (round to nearest bin)
    if (!std::isfinite(freq_hz) || sample_rate == 0) return 0;
    double bin = std::round((double)freq_hz * nfft / sample_rate);
    if (bin > std::numeric_limits<int>::max())
        return std::numeric_limits<int>::max();
    if (bin < std::numeric_limits<int>::min())
        return std::numeric_limits<int>::min();
    return (int)bin;
}

float bin_to_freq(int bin, int nfft, int sample_rate) {
    return (float)bin * sample_rate / nfft;
}

NegotiatedPassband narrow_passband() {
    // Voice mic/speaker port — the shipped default (300-3000 Hz, ~2.7 kHz).
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    return pb;
}

NegotiatedPassband wide6k_passband() {
    // Flat 9600-baud data port — ~6 kHz occupancy (300-6300 Hz). Low edge kept
    // at 300 Hz (not DC): G3RUH notes the varactor-in-PLL synthesized radios lose
    // LF response and the channel is AC-coupled "down to a few Hz", so the lowest
    // OFDM carriers must sit above that skirt. High edge 6300 Hz = the port's
    // Nyquist-pulse band limit. Same nfft/cp/pilot structure as narrow — only the
    // carrier grid widens.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 6300.0f;
    pb.center_hz = 3300.0f; pb.bandwidth_hz = 6000.0f; pb.valid = true;
    return pb;
}

OfdmConfig ofdm_config_from_probe(const NegotiatedPassband& passband, int nfft, int cp_samples,
                                   int pilot_carrier_spacing, int pilot_symbol_spacing) {
    OfdmConfig cfg;
    cfg.nfft = nfft > 0 ? nfft : cfg.nfft;
    cfg.cp_samples = std::clamp(cp_samples, 0, cfg.nfft);
    cfg.sample_rate = 48000;
    cfg.pilot_carrier_spacing = std::max(2, pilot_carrier_spacing);
    cfg.pilot_symbol_spacing = pilot_symbol_spacing;

    cfg.bandwidth_hz = passband.bandwidth_hz;
    cfg.center_hz = passband.center_hz;
    cfg.subcarrier_spacing_hz = (float)cfg.sample_rate / cfg.nfft;

    // Compute first and last usable FFT bins from passband edges
    int first_bin = freq_to_bin(passband.low_hz, cfg.nfft, cfg.sample_rate);
    int last_bin  = freq_to_bin(passband.high_hz, cfg.nfft, cfg.sample_rate);

    // Clamp to valid positive bin range (audio passband, no DC/Nyquist issues)
    if (first_bin < 1) first_bin = 1;
    if (last_bin >= cfg.nfft / 2) last_bin = cfg.nfft / 2 - 1;

    // n_total = all bins that fit in the passband
    cfg.n_total_carriers = last_bin >= first_bin ? last_bin - first_bin + 1 : 0;

    // Guard carriers: 1 each edge
    cfg.n_guard_carriers = 2;

    // Used carriers: skip first and last bin (guards)
    int guard_first = first_bin;    // guard low edge
    int guard_last  = last_bin;     // guard high edge

    // Build used_carrier_bins: all bins from first+1 to last-1 (excluding guards)
    cfg.used_carrier_bins.clear();
    for (int bin = guard_first + 1; bin <= guard_last - 1; bin++) {
        cfg.used_carrier_bins.push_back(bin);
    }
    cfg.n_used_carriers = (int)cfg.used_carrier_bins.size();

    // Adaptive pilot spacing: ensure at least 4 pilots for CPE.
    if (cfg.n_used_carriers > 0) {
        int min_pilots = 4;
        int max_spacing = std::max(3, cfg.n_used_carriers / min_pilots);
        if (cfg.pilot_carrier_spacing > max_spacing)
            cfg.pilot_carrier_spacing = max_spacing;
    }

    // Pilot carriers: every Nth entry from used_carrier_bins
    cfg.pilot_carrier_bins.clear();
    cfg.data_carrier_bins.clear();
    for (int i = 0; i < cfg.n_used_carriers; i++) {
        if (i % cfg.pilot_carrier_spacing == 0) {
            cfg.pilot_carrier_bins.push_back(cfg.used_carrier_bins[i]);
        } else {
            cfg.data_carrier_bins.push_back(cfg.used_carrier_bins[i]);
        }
    }
    cfg.n_pilot_carriers = (int)cfg.pilot_carrier_bins.size();
    cfg.n_data_carriers  = (int)cfg.data_carrier_bins.size();

    cfg.n_header_symbols = 0;  // No header — config pre-negotiated (Mercury approach)

    // Log configuration
    IRIS_LOG("[OFDM-CFG] nfft=%d spacing=%.1f Hz BW=%.0f Hz center=%.0f Hz",
             cfg.nfft, cfg.subcarrier_spacing_hz, cfg.bandwidth_hz, cfg.center_hz);

    int first_used = cfg.used_carrier_bins.empty() ? -1 : cfg.used_carrier_bins.front();
    int last_used  = cfg.used_carrier_bins.empty() ? -1 : cfg.used_carrier_bins.back();
    IRIS_LOG("[OFDM-CFG] carriers: %d used, %d pilot (1:%d), %d data, %d hdr syms (bins %d-%d)",
             cfg.n_used_carriers, cfg.n_pilot_carriers, cfg.pilot_carrier_spacing,
             cfg.n_data_carriers, cfg.n_header_symbols, first_used, last_used);

    return cfg;
}

uint16_t ofdm_config_fingerprint(const OfdmConfig& cfg) {
    // FNV-1a over the integer activation parameters. Band-edge asymmetry changes
    // the first/last used bin, so those are included to catch a split grid.
    uint32_t h = 2166136261u;
    auto mix = [&](int v) {
        uint32_t u = (uint32_t)v;
        for (int b = 0; b < 4; b++) { h ^= (u & 0xFFu); h *= 16777619u; u >>= 8; }
    };
    int first_used = cfg.used_carrier_bins.empty() ? -1 : cfg.used_carrier_bins.front();
    int last_used  = cfg.used_carrier_bins.empty() ? -1 : cfg.used_carrier_bins.back();
    mix(cfg.nfft);
    mix(cfg.cp_samples);
    mix(cfg.n_used_carriers);
    mix(cfg.n_pilot_carriers);
    mix(cfg.pilot_carrier_spacing);
    mix(cfg.pilot_symbol_spacing);
    mix(cfg.pilot_row_spacing);
    mix(cfg.dft_spread);
    mix(first_used);
    mix(last_used);
    uint16_t fp = (uint16_t)((h ^ (h >> 16)) & 0xFFFFu);
    if (fp == 0) fp = 1;  // reserve 0 as "absent/unknown"
    return fp;
}

} // namespace iris
