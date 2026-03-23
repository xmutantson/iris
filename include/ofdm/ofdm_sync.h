#ifndef IRIS_OFDM_SYNC_H
#define IRIS_OFDM_SYNC_H

#include "ofdm/ofdm_config.h"
#include <complex>
#include <vector>

namespace iris {

// Result of frame detection
struct OfdmSyncResult {
    bool detected = false;
    int frame_start = -1;        // Sample offset of first training symbol CP start
    float schmidl_metric = 0;    // Peak metric value [0,1] (Schmidl-Cox autocorrelation)
    float cfo_hz = 0;            // Estimated carrier frequency offset
    float snr_est = 0;           // SNR estimate from training symbols (dB)
    float zc_metric = 0;         // ZC cross-correlation peak [0,1]
    float sc_metric = 0;         // Schmidl-Cox autocorrelation peak [0,1]
};

// ---------------------------------------------------------------------------
// Zadoff-Chu preamble generation (shared by TX and RX)
// ---------------------------------------------------------------------------

// Generate a Zadoff-Chu sequence of the given length.
// ZC: x[n] = exp(-j * pi * root * n * (n+1) / length), n = 0..length-1.
// Root must be coprime to length. Default root=7 works well for length~50-100.
std::vector<std::complex<float>> generate_zc_sequence(int root, int length);

// Generate a time-domain ZC training symbol for the given OFDM config.
// Steps:
//   1. Generate ZC sequence of length n_used_carriers
//   2. Place into correct FFT bins (config.used_carrier_bins)
//   3. IFFT to get nfft time-domain samples
//   4. Scale by nfft to match data symbol amplitude
// Returns nfft samples WITHOUT cyclic prefix (caller adds CP).
std::vector<std::complex<float>> generate_zc_training_symbol(const OfdmConfig& config,
                                                              int root = 7);

// Detect OFDM frame using hybrid Schmidl-Cox + ZC detection.
// Schmidl-Cox autocorrelation (train1 vs train2) for channel-invariant detection,
// then ZC cross-correlation for precise timing refinement.
// Returns detection result with timing, CFO, and SNR estimates.
OfdmSyncResult ofdm_detect_frame(const std::complex<float>* iq, int n_samples,
                                  const OfdmConfig& config);

// Correct CFO on a block of IQ samples (in-place rotation).
// For each sample n: iq[n] *= exp(-j * 2*pi * cfo_hz * n / sample_rate)
void ofdm_correct_cfo(std::complex<float>* iq, int n_samples,
                       float cfo_hz, int sample_rate);

// Channel estimation result
struct OfdmChannelEst {
    std::vector<std::complex<float>> H;      // Channel response per used carrier
    std::vector<float> snr_per_carrier;       // SNR estimate per used carrier (linear)
    std::vector<float> noise_var;             // Noise variance per used carrier
    float mean_snr_db = 0;                    // Average SNR across carriers
};

// Estimate channel from training symbol (ZC-based).
// iq_symbol points to the start of the training symbol (after CP removal).
// Performs NFFT-point FFT, divides by known ZC frequency-domain sequence,
// estimates H and noise.
OfdmChannelEst ofdm_estimate_channel(const std::complex<float>* iq_symbol,
                                      const OfdmConfig& config);

// Update channel estimate from a block-pilot symbol (all carriers = +1).
// IIR update: H[k] = alpha * H_new[k] + (1-alpha) * H_old[k].
// Recomputes noise_var from residuals.
void ofdm_update_channel(OfdmChannelEst& est,
                          const std::complex<float>* pilot_symbol_freq,
                          const OfdmConfig& config, float alpha = 0.3f);

// Interpolate channel estimate at data subcarriers from pilot subcarriers.
// For a regular data symbol, extracts pilot carriers (every Nth used),
// computes H at pilot positions, then linear-interpolates to data carriers.
// Uses IIR blending (alpha) to avoid destroying a good initial training estimate.
void ofdm_interpolate_pilots(OfdmChannelEst& est,
                              const std::complex<float>* symbol_freq,
                              const OfdmConfig& config,
                              float alpha = 0.3f);

// Fine CFO estimation from training symbol.
// After coarse CFO correction, the channel estimate H[k] for each used
// subcarrier should align with the known ZC phase. Any residual CFO
// manifests as a common phase rotation across subcarriers.
// Returns residual CFO in Hz (add to coarse CFO for total correction).
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config);

} // namespace iris

#endif // IRIS_OFDM_SYNC_H
