#include "common/fft.h"
#include "common/logging.h"
#include <cstdlib>
#include <mutex>
#include <vector>

namespace {

class PocketfftAllocationPool {
    struct Block {
        void* data;
        std::size_t size;
        bool in_use;
    };

public:
    ~PocketfftAllocationPool() {
        for (const auto& block : blocks_)
            std::free(block.data);
    }

    void* allocate(std::size_t size) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& block : blocks_) {
            if (!block.in_use && block.size >= size) {
                block.in_use = true;
                return block.data;
            }
        }

        void* data = std::malloc(size);
        if (data) {
            try {
                blocks_.push_back({data, size, true});
            } catch (...) {
                std::free(data);
                throw;
            }
        }
        return data;
    }

    void deallocate(void* data) {
        if (!data) return;
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& block : blocks_) {
            if (block.data == data) {
                block.in_use = false;
                return;
            }
        }
        std::free(data);
    }

private:
    std::mutex mutex_;
    std::vector<Block> blocks_;
};

PocketfftAllocationPool s_pocketfft_allocations;

} // namespace

namespace pocketfft {
namespace detail {

static void* malloc(std::size_t size) {
    return s_pocketfft_allocations.allocate(size);
}

static void free(void* data) {
    s_pocketfft_allocations.deallocate(data);
}

} // namespace detail
} // namespace pocketfft


// L1 / precook (IRIS_STALL_PRECOOK_AUDIT.md §2, L1 / §4 R1):
// pocketfft's get_plan() rebuilds a fresh N-point plan (factorization + twiddle
// tables + heap, ~14 heap allocs per 1024-pt call) on EVERY fft/ifft when
// POCKETFFT_CACHE_SIZE==0 (its default). Iris calls fft_complex/ifft_complex
// hundreds of times per frame (OFDM demod/mod, sync, chan-est, spectrum) on the
// single audio callback thread under modem_mutex_ — tens of ms/frame of pure
// waste that starves the capture thread. Enable the pocketfft persistent-plan
// LRU cache so each size's plan is built ONCE (in Modem::precook() at startup,
// off the callback thread) and reused thereafter. This is byte-identical: the
// plan for a given length is deterministic; caching only reuses the same object.
// The cache is mutex-protected (thread-safe) — and Iris runs FFTs serialized
// under modem_mutex_ regardless. Only fft.cc includes pocketfft, so there is no
// cross-TU ODR concern with defining the size here. 16 entries (< ~1 MB total)
// covers the hot fixed sizes {1024 OFDM, 512 spectrum} with headroom for the
// occasional variable Hilbert size, with no LRU thrash in OFDM steady state.
#define POCKETFFT_CACHE_SIZE 16

#define POCKETFFT_NO_MULTITHREADING
#include "common/pocketfft_hdronly.h"

#include <cmath>
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

static void transform_complex(std::complex<float>* data, int n,
                              bool forward, float scale) {
    if (n == 0) return;
    auto plan = pocketfft::detail::get_plan<
        pocketfft::detail::pocketfft_c<float>>((size_t)n);
    plan->exec(reinterpret_cast<pocketfft::detail::cmplx<float>*>(data),
               scale, forward);
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

// Complex interleaved forward FFT — zero heap allocation after warm-up.
void fft_complex(std::complex<float>* data, int n) {
    log_fft_once(n);
    transform_complex(data, n, pocketfft::FORWARD, 1.0f);
}

// Complex interleaved inverse FFT — divides by n (matches old behavior).
void ifft_complex(std::complex<float>* data, int n) {
    log_ifft_once(n);
    transform_complex(data, n, pocketfft::BACKWARD, 1.0f / (float)n);
}

} // namespace iris
