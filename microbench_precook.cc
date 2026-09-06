// microbench_precook.cc — CARD-FREE alloc-count + timing micro-bench for the
// Iris precook point-fixes (L1 FFT plan cache, L4 LDPC persistent scratch).
//
// Measures, in steady state (post-warmup, so one-time allocs like the pocketfft
// plan build and stdio buffers are excluded from the per-call figure):
//   - heap allocations per fft_complex()/ifft_complex() call  (L1)
//   - heap allocations per LdpcCodec::decode_soft() call       (L4)
//   - wall-clock per call (the perf win)
//
// FAIL-BEFORE (POCKETFFT_CACHE_SIZE 0, per-call DecoderWorkspace): plan rebuilt
//   every FFT -> allocs/call > 0; scratch alloced every decode.
// PASS-AFTER  (cache enabled, persistent scratch): plan built ONCE in warmup ->
//   0 allocs/FFT call; only the legitimate `output` return vector allocs/decode.
//
// Link against the REAL fft.o + ldpc.o + mercury_normal_*.o objects so this
// exercises the exact production code path.
#include "common/fft.h"
#include "fec/ldpc.h"

#include <atomic>
#include <cstdlib>
#include <cstdio>
#include <new>
#include <vector>
#include <complex>
#include <chrono>

// ---- global allocation counter (counts every ::operator new) ----
static std::atomic<size_t> g_allocs{0};
void* operator new(std::size_t sz) {
    g_allocs.fetch_add(1, std::memory_order_relaxed);
    void* p = std::malloc(sz ? sz : 1);
    if (!p) throw std::bad_alloc();
    return p;
}
void* operator new[](std::size_t sz) {
    g_allocs.fetch_add(1, std::memory_order_relaxed);
    void* p = std::malloc(sz ? sz : 1);
    if (!p) throw std::bad_alloc();
    return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

static size_t allocs() { return g_allocs.load(std::memory_order_relaxed); }

using clk = std::chrono::steady_clock;
static double ms_since(clk::time_point t) {
    return std::chrono::duration<double, std::milli>(clk::now() - t).count();
}

int main() {
    const int WARM = 8;
    const int N = 2000;

    // ===== Plan-built-ONCE probe: a FRESH size never transformed before, so the
    // first call is guaranteed cold (builds+caches the plan if the cache is on).
    // cache OFF  -> every call rebuilds the plan -> allocs constant & high.
    // cache ON   -> call #1 builds the plan (high), calls #2..N reuse it
    //               (constant, lower by exactly the plan-build allocs).
    {
        const int PN = 2048;   // not used elsewhere in this bench
        std::vector<std::complex<float>> b(PN, {0.7f, 0.3f});
        size_t before1 = allocs(); iris::fft_complex(b.data(), PN);
        size_t call1 = allocs() - before1;                       // cold
        size_t before2 = allocs(); iris::fft_complex(b.data(), PN);
        size_t call2 = allocs() - before2;                       // warm
        size_t before3 = allocs(); iris::fft_complex(b.data(), PN);
        size_t call3 = allocs() - before3;                       // warm
        printf("PLAN-PROBE n=%d : call#1(cold)=%zu  call#2=%zu  call#3=%zu allocs "
               "-> plan-build allocs=%zu (happen ONCE iff call#1 > call#2==call#3)\n",
               PN, call1, call2, call3, (call1 > call2) ? (call1 - call2) : 0);
    }

    // ================= L1: FFT plan reuse =================
    for (int size : {1024, 512}) {
        std::vector<std::complex<float>> buf(size, {1.0f, -0.5f});
        // warmup: builds+caches plan (if cache on) / builds+frees (if off)
        for (int i = 0; i < WARM; i++) iris::fft_complex(buf.data(), size);

        size_t a0 = allocs();
        auto t0 = clk::now();
        for (int i = 0; i < N; i++) iris::fft_complex(buf.data(), size);
        double dt_f = ms_since(t0);
        size_t fft_allocs = allocs() - a0;

        for (int i = 0; i < WARM; i++) iris::ifft_complex(buf.data(), size);
        size_t a1 = allocs();
        auto t1 = clk::now();
        for (int i = 0; i < N; i++) iris::ifft_complex(buf.data(), size);
        double dt_i = ms_since(t1);
        size_t ifft_allocs = allocs() - a1;

        printf("FFT  n=%4d : fft_complex  allocs/call=%.4f  us/call=%.3f\n",
               size, (double)fft_allocs / N, dt_f * 1000.0 / N);
        printf("FFT  n=%4d : ifft_complex allocs/call=%.4f  us/call=%.3f\n",
               size, (double)ifft_allocs / N, dt_i * 1000.0 / N);
    }

    // ================= L4: LDPC decode scratch =================
    // Build a length-1600 LLR vector (one codeword). Convergence is irrelevant
    // to the allocation pattern (scratch is allocated regardless of outcome).
    {
        using namespace iris;
        LdpcRate rate = LdpcRate::RATE_1_2;   // 8/16, k=800, p=800 (default gear)
        const int N_LLR = 1600;
        std::vector<float> llrs(N_LLR);
        for (int i = 0; i < N_LLR; i++)
            llrs[i] = (i % 3 == 0) ? -2.5f : 3.0f;   // arbitrary soft values

        // warmup: sizes any persistent scratch + stdio buffers
        for (int i = 0; i < WARM; i++)
            (void)LdpcCodec::decode_soft(llrs, rate, LdpcDecoder::MIN_SUM, 50);

        size_t a0 = allocs();
        auto t0 = clk::now();
        const int ND = 400;
        for (int i = 0; i < ND; i++)
            (void)LdpcCodec::decode_soft(llrs, rate, LdpcDecoder::MIN_SUM, 50);
        double dt = ms_since(t0);
        size_t dec_allocs = allocs() - a0;

        printf("LDPC rate=1/2 : decode_soft allocs/call=%.4f  us/call=%.3f  (1 codeword)\n",
               (double)dec_allocs / ND, dt * 1000.0 / ND);
    }

    printf("TOTAL allocs observed: %zu\n", allocs());
    return 0;
}
