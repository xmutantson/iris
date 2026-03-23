#ifndef IRIS_PHY_H
#define IRIS_PHY_H

#include "common/types.h"
#include "native/constellation.h"
#include "native/rrc.h"
#include <complex>
#include <vector>
#include <cstdint>

namespace iris {

// Native PHY parameters
constexpr float RRC_ALPHA = 0.2f;
constexpr int   RRC_SPAN  = 6;      // symbols each side
constexpr int   SPS       = 10;     // samples per symbol (at 48kHz)

// Mode A: 800 baud default (SPS=60, 960 Hz BW), auto-adjusted by probe discovery.
// Mode B: 4800 baud (9600 bps QPSK).
// Mode C: 19200 baud (SDR only).
// Baud rate is configurable; SPS dynamically computed from sample_rate / baud_rate.

struct PhyConfig {
    int baud_rate;          // Symbol rate in baud
    int samples_per_symbol; // = sample_rate / baud_rate
    Modulation modulation;
    float rrc_alpha;
};

// Predefined modes
PhyConfig mode_a_config();                      // 800 baud default (SPS=60, 960 Hz BW)
PhyConfig mode_a_config(float bandwidth_hz);    // baud = BW / (1 + alpha)
PhyConfig mode_b_config();  // 4800 baud, 9600 bps QPSK
PhyConfig mode_c_config();  // 19200 baud, SDR

class NativeModulator {
public:
    NativeModulator(const PhyConfig& config, int sample_rate = SAMPLE_RATE);

    // Modulate bits into baseband IQ samples (I and Q interleaved)
    // Output: vector of float pairs [I0, Q0, I1, Q1, ...]
    std::vector<float> modulate(const std::vector<uint8_t>& bits);

    // Modulate symbols directly (for preamble/sync)
    std::vector<float> modulate_symbols(const std::vector<std::complex<float>>& symbols);

    const PhyConfig& config() const { return config_; }

private:
    PhyConfig config_;
    int sample_rate_;
    std::vector<float> rrc_taps_;
};

class NativeDemodulator {
public:
    NativeDemodulator(const PhyConfig& config, int sample_rate = SAMPLE_RATE);

    // Demodulate baseband IQ samples to bits
    // Input: interleaved [I0, Q0, I1, Q1, ...]
    // Returns demodulated bits
    std::vector<uint8_t> demodulate(const float* iq_samples, size_t count);

    // Get the last demodulated symbols (for diagnostics)
    const std::vector<std::complex<float>>& symbols() const { return symbols_; }

    const PhyConfig& config() const { return config_; }

private:
    PhyConfig config_;
    int sample_rate_;
    std::vector<float> rrc_taps_;

    // Timing recovery state
    float mu_;              // fractional sample offset
    float mu_gain_;         // timing PLL gain
    float prev_re_, prev_im_;
    float prev2_re_, prev2_im_;

    // Output
    std::vector<std::complex<float>> symbols_;
};

} // namespace iris

#endif
