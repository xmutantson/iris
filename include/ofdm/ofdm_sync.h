#ifndef IRIS_OFDM_SYNC_H
#define IRIS_OFDM_SYNC_H

#include "ofdm/ofdm_config.h"
#include <complex>
#include <vector>

namespace iris {

// Result of frame detection
struct OfdmSyncResult {
    bool detected = false;
    int frame_start = -1;        // Sample offset of first training symbol (after CP)
    float schmidl_metric = 0;    // Peak metric value [0,1]
    float cfo_hz = 0;            // Estimated carrier frequency offset
    float snr_est = 0;           // SNR estimate from training symbols (dB)
};

// Detect OFDM frame using Schmidl-Cox on training symbol 1.
// Searches through baseband IQ samples for the training pattern.
// Training symbol 1 has identical first/second halves (each L = NFFT/2 samples).
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

// Estimate channel from training symbol 2 (all used carriers known = +1).
// iq_symbol points to the start of training symbol 2 (after CP removal).
// Performs NFFT-point FFT, extracts used carrier bins, estimates H and noise.
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

// Fine CFO estimation from training symbol 2.
// After coarse CFO correction, the channel estimate H[k] for each used
// subcarrier should be purely real (since TX = +1). Any residual CFO
// manifests as a linear phase slope across subcarriers.
// Returns residual CFO in Hz (add to coarse CFO for total correction).
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config);

} // namespace iris

#endif // IRIS_OFDM_SYNC_H
