#include "ofdm/ofdm_papr.h"
#include "common/fft.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>

namespace iris {

float compute_papr_db(const std::vector<std::complex<float>>& signal) {
    if (signal.empty()) return 0.0f;

    float max_power = 0.0f;
    float mean_power = 0.0f;
    for (auto& s : signal) {
        float p = std::norm(s);
        if (p > max_power) max_power = p;
        mean_power += p;
    }
    mean_power /= (float)signal.size();
    if (mean_power < 1e-20f) return 0.0f;

    return 10.0f * std::log10(max_power / mean_power);
}

PaprClipResult ofdm_papr_clip(std::vector<std::complex<float>>& frame,
                                const OfdmConfig& config,
                                float target_papr_db,
                                int n_iter) {
    PaprClipResult result{};
    if (frame.empty()) return result;

    const int nfft = config.nfft;
    const int cp = config.cp_samples;
    const int sym_len = nfft + cp;
    const int n_symbols = (int)frame.size() / sym_len;

    if (n_symbols < 1) return result;

    result.papr_before_db = compute_papr_db(frame);

    // Early exit: already below target
    if (result.papr_before_db <= target_papr_db + 0.5f) {
        result.papr_after_db = result.papr_before_db;
        return result;
    }

    // Build frequency-domain mask: which FFT bins to keep.
    // Keep used carrier bins + their Hermitian mirrors.
    // DC (bin 0) and Nyquist (bin nfft/2) are kept if they're in range.
    std::vector<bool> keep_bin(nfft, false);
    for (int bin : config.used_carrier_bins) {
        if (bin >= 0 && bin < nfft) {
            keep_bin[bin] = true;
            int mirror = nfft - bin;
            if (mirror > 0 && mirror < nfft)
                keep_bin[mirror] = true;
        }
    }

    int total_clipped = 0;
    std::vector<std::complex<float>> fft_buf(nfft);

    for (int iter = 0; iter < n_iter; iter++) {
        // Step 1: Compute mean power and clip threshold
        float mean_power = 0.0f;
        for (auto& s : frame) mean_power += std::norm(s);
        mean_power /= (float)frame.size();

        if (mean_power < 1e-20f) break;

        // Clip threshold in amplitude: sqrt(mean_power * 10^(target/10))
        float threshold = std::sqrt(mean_power * std::pow(10.0f, target_papr_db / 10.0f));

        // Step 2: Clip time-domain amplitude (preserve phase)
        int n_clipped = 0;
        for (auto& s : frame) {
            float mag = std::abs(s);
            if (mag > threshold) {
                s *= (threshold / mag);
                n_clipped++;
            }
        }
        total_clipped += n_clipped;

        if (n_clipped == 0) break;  // Already below target

        // Step 3: Per-symbol bandpass filter to remove OOB spectral regrowth.
        // For each OFDM symbol:
        //   a) FFT the nfft-sample body (skip CP)
        //   b) Zero all bins outside the used carrier range
        //   c) Enforce Hermitian symmetry for real-valued output
        //   d) IFFT back to time domain
        //   e) Regenerate CP from filtered body (last cp samples)
        for (int sym = 0; sym < n_symbols; sym++) {
            int sym_start = sym * sym_len;
            int body_start = sym_start + cp;

            // FFT the symbol body
            std::copy(frame.begin() + body_start,
                      frame.begin() + body_start + nfft,
                      fft_buf.begin());
            fft_complex(fft_buf.data(), nfft);

            // Zero out-of-band bins
            for (int k = 0; k < nfft; k++) {
                if (!keep_bin[k])
                    fft_buf[k] = {0.0f, 0.0f};
            }

            // Enforce Hermitian symmetry: X[N-k] = conj(X[k])
            // This ensures the IFFT output is purely real-valued.
            // After clipping + filtering, tiny imaginary residuals accumulate;
            // this step prevents them from growing across iterations.
            fft_buf[0] = {fft_buf[0].real(), 0.0f};  // DC must be real
            if (nfft > 1)
                fft_buf[nfft / 2] = {fft_buf[nfft / 2].real(), 0.0f};  // Nyquist
            for (int k = 1; k < nfft / 2; k++) {
                auto avg = (fft_buf[k] + std::conj(fft_buf[nfft - k])) * 0.5f;
                fft_buf[k] = avg;
                fft_buf[nfft - k] = std::conj(avg);
            }

            // IFFT back to time domain.
            // fft_complex() + ifft_complex() = identity (no extra scaling needed).
            ifft_complex(fft_buf.data(), nfft);

            // Write filtered body back
            std::copy(fft_buf.begin(), fft_buf.end(),
                      frame.begin() + body_start);

            // Regenerate CP from last cp samples of filtered body
            std::copy(frame.begin() + body_start + nfft - cp,
                      frame.begin() + body_start + nfft,
                      frame.begin() + sym_start);
        }
    }

    result.papr_after_db = compute_papr_db(frame);
    result.n_clipped_samples = total_clipped;

    IRIS_LOG("[OFDM-PAPR] clip: %.1f -> %.1f dB (target %.1f, %d clipped, %d iters, %d syms)",
             result.papr_before_db, result.papr_after_db, target_papr_db,
             total_clipped, n_iter, n_symbols);

    return result;
}

} // namespace iris
