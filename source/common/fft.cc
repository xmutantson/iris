#include "common/fft.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <vector>
#include <mutex>

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
        IRIS_LOG("[FFT] fft(%d) called", n);
    }
}

static void log_ifft_once(int n) {
    std::lock_guard<std::mutex> lock(s_log_mutex);
    if (s_ifft_logged_sizes.find(n) == s_ifft_logged_sizes.end()) {
        s_ifft_logged_sizes.insert(n);
        IRIS_LOG("[FFT] ifft(%d) called", n);
    }
}

// Radix-2 Cooley-Tukey DIT FFT, in-place.
void fft(float* re, float* im, int n) {
    log_fft_once(n);

    // Bit-reversal permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }

    // Butterfly stages
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        float wR = std::cos(ang), wI = std::sin(ang);
        for (int i = 0; i < n; i += len) {
            float curR = 1.0f, curI = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                float uR = re[i + j], uI = im[i + j];
                float vR = re[i + j + len / 2] * curR - im[i + j + len / 2] * curI;
                float vI = re[i + j + len / 2] * curI + im[i + j + len / 2] * curR;
                re[i + j] = uR + vR;
                im[i + j] = uI + vI;
                re[i + j + len / 2] = uR - vR;
                im[i + j + len / 2] = uI - vI;
                float tmpR = curR * wR - curI * wI;
                curI = curR * wI + curI * wR;
                curR = tmpR;
            }
        }
    }
}

// Inverse FFT: conjugate, forward FFT, conjugate, scale by 1/n.
void ifft(float* re, float* im, int n) {
    log_ifft_once(n);

    // Conjugate inputs
    for (int i = 0; i < n; i++) im[i] = -im[i];

    // Forward FFT
    fft(re, im, n);

    // Conjugate outputs and scale by 1/n
    float inv_n = 1.0f / n;
    for (int i = 0; i < n; i++) {
        re[i] *= inv_n;
        im[i] = -im[i] * inv_n;
    }
}

// Complex interleaved forward FFT.
void fft_complex(std::complex<float>* data, int n) {
    // std::complex<float> is guaranteed contiguous {re, im} pairs.
    // Extract to separate arrays, run FFT, write back.
    // Using reinterpret_cast on the interleaved layout directly would require
    // stride-2 access; separate arrays are simpler and cache-friendlier for
    // the butterfly.
    std::vector<float> re(n), im(n);
    for (int i = 0; i < n; i++) {
        re[i] = data[i].real();
        im[i] = data[i].imag();
    }

    fft(re.data(), im.data(), n);

    for (int i = 0; i < n; i++) {
        data[i] = std::complex<float>(re[i], im[i]);
    }
}

// Complex interleaved inverse FFT.
void ifft_complex(std::complex<float>* data, int n) {
    std::vector<float> re(n), im(n);
    for (int i = 0; i < n; i++) {
        re[i] = data[i].real();
        im[i] = data[i].imag();
    }

    ifft(re.data(), im.data(), n);

    for (int i = 0; i < n; i++) {
        data[i] = std::complex<float>(re[i], im[i]);
    }
}

} // namespace iris
