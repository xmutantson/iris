#include "common/fft.h"
#include "common/logging.h"

#define POCKETFFT_NO_MULTITHREADING
#include "common/pocketfft_hdronly.h"

#include <cmath>
#include <vector>
#include <mutex>
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Track which FFT sizes have been logged to avoid spam.
static std::mutex s_log_mutex;
static std::unordered_set<int> s_fft_logged_sizes;
static std::unordered_set<int> s_ifft_logged_sizes;

static void log_fft_once(int n) {
    std::lock_guard<std::mutex> lock(s_log_mutex);
    if (s_fft_logged_sizes.find(n) == s_fft_logged_sizes.end()) {
        s_fft_logged_sizes.insert(n);
        IRIS_LOG("[FFT] fft(%d) called [pocketfft]", n);
    }
}

static void log_ifft_once(int n) {
    std::lock_guard<std::mutex> lock(s_log_mutex);
    if (s_ifft_logged_sizes.find(n) == s_ifft_logged_sizes.end()) {
        s_ifft_logged_sizes.insert(n);
        IRIS_LOG("[FFT] ifft(%d) called [pocketfft]", n);
    }
}

// Split real/imag forward FFT (used by passband_probe.cc).
void fft(float* re, float* im, int n) {
    log_fft_once(n);

    // Pack into interleaved complex, run pocketfft, unpack
    thread_local std::vector<std::complex<float>> buf;
    buf.resize(n);
    for (int i = 0; i < n; i++)
        buf[i] = std::complex<float>(re[i], im[i]);

    pocketfft::shape_t shape{(size_t)n};
    pocketfft::stride_t stride{(ptrdiff_t)sizeof(std::complex<float>)};
    pocketfft::shape_t axes{0};
    pocketfft::c2c(shape, stride, stride, axes, pocketfft::FORWARD,
                    buf.data(), buf.data(), 1.0f);

    for (int i = 0; i < n; i++) {
        re[i] = buf[i].real();
        im[i] = buf[i].imag();
    }
}

// Split real/imag inverse FFT.
void ifft(float* re, float* im, int n) {
    log_ifft_once(n);

    thread_local std::vector<std::complex<float>> buf;
    buf.resize(n);
    for (int i = 0; i < n; i++)
        buf[i] = std::complex<float>(re[i], im[i]);

    pocketfft::shape_t shape{(size_t)n};
    pocketfft::stride_t stride{(ptrdiff_t)sizeof(std::complex<float>)};
    pocketfft::shape_t axes{0};
    pocketfft::c2c(shape, stride, stride, axes, pocketfft::BACKWARD,
                    buf.data(), buf.data(), 1.0f / (float)n);

    for (int i = 0; i < n; i++) {
        re[i] = buf[i].real();
        im[i] = buf[i].imag();
    }
}

// Complex interleaved forward FFT — zero heap allocation (pocketfft in-place).
void fft_complex(std::complex<float>* data, int n) {
    log_fft_once(n);

    pocketfft::shape_t shape{(size_t)n};
    pocketfft::stride_t stride{(ptrdiff_t)sizeof(std::complex<float>)};
    pocketfft::shape_t axes{0};
    pocketfft::c2c(shape, stride, stride, axes, pocketfft::FORWARD,
                    data, data, 1.0f);
}

// Complex interleaved inverse FFT — divides by n (matches old behavior).
void ifft_complex(std::complex<float>* data, int n) {
    log_ifft_once(n);

    pocketfft::shape_t shape{(size_t)n};
    pocketfft::stride_t stride{(ptrdiff_t)sizeof(std::complex<float>)};
    pocketfft::shape_t axes{0};
    pocketfft::c2c(shape, stride, stride, axes, pocketfft::BACKWARD,
                    data, data, 1.0f / (float)n);
}

} // namespace iris
