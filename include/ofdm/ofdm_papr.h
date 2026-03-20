#ifndef IRIS_OFDM_PAPR_H
#define IRIS_OFDM_PAPR_H

#include "ofdm/ofdm_config.h"
#include <complex>
#include <vector>

namespace iris {

struct PaprClipResult {
    float papr_before_db;
    float papr_after_db;
    int n_clipped_samples;  // total across all iterations
};

// Hilbert PAPR clipper: reduces peak-to-average power ratio of OFDM frame.
//
// Clips time-domain amplitude to target PAPR, then per-symbol bandpass
// filters to remove out-of-band spectral regrowth. Iterates n_iter times
// for convergence. After filtering, regenerates cyclic prefix from the
// filtered symbol body to maintain CP validity.
//
// This is the critical fix for FM radio: without PAPR reduction, the
// FM deviation limiter hard-clips OFDM data symbols (16-19 dB PAPR),
// corrupting pilots and data carriers. The ZC preamble survives (PAPR ~0)
// but data symbols are destroyed.
//
// target_papr_db: desired PAPR in dB (7 dB typical for FM)
// n_iter: number of clip+filter iterations (4 typical)
PaprClipResult ofdm_papr_clip(std::vector<std::complex<float>>& frame,
                                const OfdmConfig& config,
                                float target_papr_db = 7.0f,
                                int n_iter = 4);

// Compute PAPR of a signal in dB
float compute_papr_db(const std::vector<std::complex<float>>& signal);

} // namespace iris

#endif // IRIS_OFDM_PAPR_H
