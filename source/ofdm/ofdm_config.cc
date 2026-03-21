#include "ofdm/ofdm_config.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>

namespace iris {

int freq_to_bin(float freq_hz, int nfft, int sample_rate) {
    // FFT bin index for a given frequency (round to nearest bin)
    return (int)std::round(freq_hz * nfft / sample_rate);
}

float bin_to_freq(int bin, int nfft, int sample_rate) {
    return (float)bin * sample_rate / nfft;
}

OfdmConfig ofdm_config_from_probe(const NegotiatedPassband& passband, int nfft, int cp_samples,
                                   int pilot_carrier_spacing, int pilot_symbol_spacing) {
    OfdmConfig cfg;
    cfg.nfft = nfft;
    cfg.cp_samples = cp_samples;
    cfg.sample_rate = 48000;
    cfg.pilot_carrier_spacing = pilot_carrier_spacing;
    cfg.pilot_symbol_spacing = pilot_symbol_spacing;

    cfg.bandwidth_hz = passband.bandwidth_hz;
    cfg.center_hz = passband.center_hz;
    cfg.subcarrier_spacing_hz = (float)cfg.sample_rate / cfg.nfft;

    // Compute first and last usable FFT bins from passband edges
    int first_bin = freq_to_bin(passband.low_hz, nfft, cfg.sample_rate);
    int last_bin  = freq_to_bin(passband.high_hz, nfft, cfg.sample_rate);

    // Clamp to valid positive bin range (audio passband, no DC/Nyquist issues)
    if (first_bin < 1) first_bin = 1;
    if (last_bin >= nfft / 2) last_bin = nfft / 2 - 1;

    // n_total = all bins that fit in the passband
    cfg.n_total_carriers = last_bin - first_bin + 1;

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

} // namespace iris
