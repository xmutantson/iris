#ifndef IRIS_RRC_H
#define IRIS_RRC_H

#include <vector>

namespace iris {

// Root-raised-cosine pulse shaping filter
// alpha: rolloff factor (0.2 typical for Iris)
// span: filter length in symbol periods (each side)
// sps: samples per symbol
std::vector<float> rrc_filter(float alpha, int span, int sps);

// Apply FIR filter to input signal
std::vector<float> fir_filter(const std::vector<float>& input,
                               const std::vector<float>& taps);

} // namespace iris

#endif
