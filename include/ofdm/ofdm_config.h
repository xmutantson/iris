#ifndef IRIS_OFDM_CONFIG_H
#define IRIS_OFDM_CONFIG_H

#include "probe/passband_probe.h"
#include <vector>
#include <cstdint>

namespace iris {

struct OfdmConfig {
    // Core parameters
    int nfft = 512;                    // FFT size (256/512/1024)
    int cp_samples = 64;               // Cyclic prefix length in samples
    int sample_rate = 48000;           // Fixed

    // Derived from probe
    float bandwidth_hz = 2000.0f;      // Negotiated bandwidth
    float center_hz = 1700.0f;         // Center frequency
    float subcarrier_spacing_hz;       // = sample_rate / nfft

    // Carrier allocation
    int n_total_carriers;              // Total carriers in bandwidth
    int n_guard_carriers = 2;          // Guard carriers (1 each edge)
    int n_used_carriers;               // n_total - n_guard
    int n_pilot_carriers;              // Every 4th used carrier
    int n_data_carriers;               // n_used - n_pilot

    // Carrier index arrays (indices into FFT bins)
    std::vector<int> used_carrier_bins;    // All used FFT bin indices
    std::vector<int> pilot_carrier_bins;   // Pilot FFT bin indices
    std::vector<int> data_carrier_bins;    // Data FFT bin indices

    // Pilot pattern
    int pilot_carrier_spacing = 4;     // Every Nth used carrier — comb pilots for CPE + interpolation
    int pilot_symbol_spacing = 14;     // Every Mth OFDM symbol is all-pilot (FM: slow fading, 14*12ms=168ms)

    // FM TX de-emphasis: attenuate higher carriers on TX so that after the
    // radio's own pre-emphasis the signal is flat entering the deviation limiter.
    // NBFM mic/speaker: 300 Hz (530us time constant, TIA/EIA-603)
    // FM broadcast (75us): 2120 Hz. Set to 0 to disable (flat audio data port).
    float fm_preemph_corner_hz = 300.0f;
    float fm_preemph_gain_cap = 3.0f;  // Max de-emphasis ratio (3 = −9.5 dB floor)

    // Header
    int n_header_symbols = 0;          // No header — config pre-negotiated (Mercury approach)

    // Computed helpers
    int symbol_samples() const { return nfft + cp_samples; }
    float symbol_duration_s() const { return (float)symbol_samples() / sample_rate; }
    float symbol_rate() const { return 1.0f / symbol_duration_s(); }
    int data_bits_per_symbol_bpsk() const { return n_data_carriers; }
};

// Create OfdmConfig from probe result
OfdmConfig ofdm_config_from_probe(const NegotiatedPassband& passband, int nfft = 512, int cp_samples = 64,
                                   int pilot_carrier_spacing = 4, int pilot_symbol_spacing = 14);

// Get the FFT bin index for a given frequency
int freq_to_bin(float freq_hz, int nfft, int sample_rate);

// Get frequency for a given FFT bin
float bin_to_freq(int bin, int nfft, int sample_rate);

} // namespace iris
#endif
