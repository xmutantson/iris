#ifndef IRIS_FFT_H
#define IRIS_FFT_H

#include <complex>
#include <cstddef>

namespace iris {

// Forward FFT (DFT). In-place via pocketfft. Supports arbitrary sizes.
void fft(float* re, float* im, int n);

// Inverse FFT (IDFT). In-place via pocketfft. Supports arbitrary sizes.
void ifft(float* re, float* im, int n);

// Complex interleaved version of forward FFT.
void fft_complex(std::complex<float>* data, int n);

// Complex interleaved version of inverse FFT.
void ifft_complex(std::complex<float>* data, int n);

} // namespace iris

#endif // IRIS_FFT_H
