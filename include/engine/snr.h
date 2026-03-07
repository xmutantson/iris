#ifndef IRIS_SNR_H
#define IRIS_SNR_H

#include <complex>
#include <vector>

namespace iris {

// Estimate SNR from known preamble symbols vs received symbols
// Returns SNR in dB
float estimate_snr(const std::complex<float>* tx_symbols,
                   const std::complex<float>* rx_symbols, int count);

// AGC: compute gain to normalize signal to target RMS
class AGC {
public:
    AGC(float target_rms = 0.3f, float attack = 0.1f, float decay = 0.001f);

    // Process one sample, return gain-adjusted sample
    float process(float sample);

    // Process a block
    void process_block(float* samples, int count);

    float gain() const { return gain_; }
    void reset();

private:
    float target_rms_;
    float attack_;
    float decay_;
    float gain_;
    float rms_est_;
};

} // namespace iris

#endif
