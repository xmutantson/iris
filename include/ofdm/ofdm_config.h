#ifndef IRIS_OFDM_CONFIG_H
#define IRIS_OFDM_CONFIG_H

#include "probe/passband_probe.h"
#include <vector>
#include <cstdint>

namespace iris {

struct OfdmConfig {
    // Core parameters
    int nfft = 1024;                   // FFT size (256/512/1024) — 46.875 Hz spacing at 48k (~44 baud, matches VARA FM)
    int cp_samples = 64;               // Cyclic prefix length in samples (1.33ms). FM has ~0 delay spread; minimal CP for filter transients.
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
    int pilot_carrier_spacing = 4;     // Every Nth used carrier — comb pilots for CPE. 4 gives ~14 pilots in NBFM (~25% overhead).
    int pilot_symbol_spacing = 24;     // Every Mth OFDM symbol is all-pilot (block pilots for channel tracking)
    int pilot_row_spacing = 8;         // Every Kth data symbol is a dense pilot row (all carriers = known ref)

    // FM TX de-emphasis: attenuate higher carriers on TX so that after the
    // radio's own pre-emphasis the signal is flat entering the deviation limiter.
    // NBFM mic/speaker: 300 Hz (530us time constant, TIA/EIA-603)
    // FM broadcast (75us): 2120 Hz. Set to 0 to disable (flat audio data port).
    float fm_preemph_corner_hz = 300.0f;
    float fm_preemph_gain_cap = 10.0f; // Max de-emphasis ratio (10 = −20 dB floor, handles 6 kHz BW)

    // DFT-spread OFDM (SC-FDMA): DFT-precode data carriers before IFFT.
    // Reduces PAPR from ~10-11 dB to ~5-7 dB (modulation-dependent).
    // All data carriers in a symbol must use the same modulation order.
    // Used in LTE/5G NR uplink for the same reason: FM deviation limiters
    // behave like power amplifier saturators — low PAPR is critical.
    bool dft_spread = true;

    // Clean-channel decode mode (propagated from IrisConfig::ofdm_clean_channel).
    // When true, use broadcast AWGN noise floor for per-carrier noise_var
    // (better on flat channels, worse on frequency-selective ones).
    bool clean_channel = false;
    // PAPR clipper disable (propagated from IrisConfig::ofdm_skip_papr_clip).
    // When true, build_ofdm_frame skips the PAPR clipping pass.
    bool skip_papr_clip = false;
    // LLR-scalar source (propagated from IrisConfig::ofdm_llr_use_frame_nv).
    // When true, dft_sigma_sq for LDPC LLR scaling uses guard-bin nv_frame
    // (frame-wide AWGN) rather than per-carrier H-smoothness residual.
    // See fact doc §9. Default false.
    bool llr_use_frame_nv = false;

    // MMSE-despread bias correction for soft-demap (SC-FDMA).
    // After MMSE-FDE + IDFT despread, the recovered symbol is
    //   d_hat = mu_avg * d + e,   mu_avg = mean(|H|^2/(|H|^2+nv)) = 1 - dft_sigma_sq,
    // with total error variance E|e|^2 = mu_avg*(1-mu_avg) (MMSE noise +
    // residual self-interference — the SC-FDMA effective-SINR identity, so
    // gamma_eff = mu/(1-mu)). The demapper references the FULL-scale grid, so
    // d_hat is scaled by 1/mu_avg and the LLR denominator becomes
    // (1-mu)/mu = dft_sigma_sq/mu_avg. See ofdm_demod.cc demap_to_llrs for
    // the derivation. srsRAN / MATLAB LTE demappers normalize by the post-EQ
    // equalizer gain for the same reason.
    // History: a 2026-07-02 --mu-lens A/B found the correction a no-op for O3
    // decode outcomes — measured at mu~0.95-0.97 on a clean flat channel,
    // where all scalings coincide (and the then-denominator 1/mu^2 was itself
    // off by 1/mu). With the honest per-carrier sigma^2(k) estimator, colored
    // or tilted channels reach mu~0.7-0.9 where the correct scaling matters.
    // Default-ON. IRIS_MU_AVG=0 restores the legacy shrunken-symbol demap.
    bool mu_avg_correct = true;

    // Sync detection thresholds
    // Tight-quality reference used by diagnostics/measurement.  Acquisition
    // caps this at its documented loose admission floor (0.70) before LDPC;
    // complete per-codeword CRC validation remains the acceptance authority.
    float fd_zc_threshold = 0.97f;

    // Header
    int n_header_symbols = 0;          // No header — config pre-negotiated (Mercury approach)

    // Computed helpers
    int symbol_samples() const { return nfft + cp_samples; }
    float symbol_duration_s() const { return (float)symbol_samples() / sample_rate; }
    float symbol_rate() const { return 1.0f / symbol_duration_s(); }
    int data_bits_per_symbol_bpsk() const { return n_data_carriers; }
};

// Create OfdmConfig from probe result
OfdmConfig ofdm_config_from_probe(const NegotiatedPassband& passband, int nfft = 1024, int cp_samples = 64,
                                   int pilot_carrier_spacing = 4, int pilot_symbol_spacing = 24);

// Named band profiles (BW-parameterized; the whole carrier layout follows from
// low_hz/high_hz via ofdm_config_from_probe). These are pure config knobs — the
// TX/RX build a wider carrier grid from them, NOT a forked codepath. Narrow
// (300-3000, the mic/speaker voice port) stays the default everywhere.
//
//   NARROW ("N"):  300-3000 Hz  ~2.7 kHz  -> ~52 data carriers (nfft=1024)
//   WIDE-6K ("W"): 300-6300 Hz  ~6.0 kHz  -> ~112 data carriers
//
// WIDE-6K targets the radio's flat 9600-baud DATA port (direct varactor TX /
// discriminator RX, no pre/de-emphasis, no syllabic deviation limiter). G3RUH's
// canonical 9600 design (amsat.org/amsat/articles/g3ruh/109.html) band-limits
// that port to ~6.3 kHz ("absolutely band limited to 6300 Hz", flat to ~3.3 kHz,
// -4 dB at 4.8 kHz), so a ~6 kHz OFDM occupancy fits it. This is VARA FM WIDE's
// port (its wide bar is 2.65x its narrow). No emphasis EVM cliff there, so the
// upper QAM rungs (64/256QAM) the narrow emphasis port cannot carry become
// viable. PHY-first substrate — no ARQ/negotiation wiring in this pass.
NegotiatedPassband narrow_passband();
NegotiatedPassband wide6k_passband();

// Deterministic 16-bit fingerprint of the ACTIVATION parameters that both ends
// must resolve identically for OFDM interop: FFT size, cyclic prefix, carrier
// grid (first/last used bin + count) and pilot layout.  Integer-only, so two
// ends that resolved the SAME config produce a bit-identical value; a split
// carrier grid (band-edge asymmetry) or an nfft/cp/pilot desync changes it.
// This is a LIVE fingerprint recomputed every session — NOT versioning
// machinery.  0 is reserved as "absent/unknown".
uint16_t ofdm_config_fingerprint(const OfdmConfig& cfg);

// Get the FFT bin index for a given frequency
int freq_to_bin(float freq_hz, int nfft, int sample_rate);

// Get frequency for a given FFT bin
float bin_to_freq(int bin, int nfft, int sample_rate);

} // namespace iris
#endif
