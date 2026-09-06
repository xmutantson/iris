#include "native/rrc.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

std::vector<float> rrc_filter(float alpha, int span, int sps) {
    int len = 2 * span * sps + 1;
    std::vector<float> h(len);
    float norm = 0;

    for (int i = 0; i < len; i++) {
        float t = (float)(i - span * sps) / sps;

        if (std::abs(t) < 1e-6f) {
            // t = 0
            h[i] = (1.0f - alpha + 4.0f * alpha / M_PI);
        } else if (std::abs(std::abs(t) - 1.0f / (4.0f * alpha)) < 1e-6f) {
            // t = ±1/(4*alpha)
            h[i] = (alpha / std::sqrt(2.0f)) *
                   ((1.0f + 2.0f / M_PI) * std::sin(M_PI / (4.0f * alpha)) +
                    (1.0f - 2.0f / M_PI) * std::cos(M_PI / (4.0f * alpha)));
        } else {
            float num = std::sin(M_PI * t * (1.0f - alpha)) +
                        4.0f * alpha * t * std::cos(M_PI * t * (1.0f + alpha));
            float den = M_PI * t * (1.0f - 16.0f * alpha * alpha * t * t);
            h[i] = num / den;
        }
        norm += h[i] * h[i];
    }

    // Normalize to unit energy: matched TX/RX pair gives peak = 1 at symbol center
    // This is the correct normalization for pulse-shaping matched filters.
    float energy = std::sqrt(norm);
    if (energy > 0) {
        for (int i = 0; i < len; i++)
            h[i] /= energy;
    }

    return h;
}

std::vector<float> fir_filter(const std::vector<float>& input,
                               const std::vector<float>& taps) {
    size_t n = input.size();
    size_t m = taps.size();
    std::vector<float> output(n + m - 1, 0.0f);

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            output[i + j] += input[i] * taps[j];
        }
    }
    return output;
}

} // namespace iris
