#ifndef IRIS_CHANNEL_EQ_H
#define IRIS_CHANNEL_EQ_H

#include "probe/passband_probe.h"
#include <vector>
#include <cstddef>

namespace iris {

// Passband channel equalizer built from probe tone power measurements.
//
// FM radios apply de-emphasis (6 dB/octave rolloff) which creates
// frequency-dependent amplitude distortion across the passband.
// This kills higher-order QAM constellations because the single-tap
// channel estimator interprets the slope as noise.
//
// The probe system already measures per-tone power at 64 frequencies.
// This class uses that data to design a linear-phase FIR that flattens
// the channel response, enabling turboshift to reach A2+ speed levels.

class ChannelEqualizer {
public:
    // Build EQ filter from probe tone power measurements.
    // my_rx_result: what we measured from peer's probe (channel to flatten)
    // passband: negotiated passband (defines frequency range)
    // sample_rate: audio sample rate (48000)
    // max_boost_db: maximum per-tone boost (RX: +3 dB to avoid noise amplification,
    //               TX: +6 dB for FM de-emphasis compensation)
    void configure(const ProbeResult& my_rx_result,
                   const NegotiatedPassband& passband,
                   int sample_rate,
                   float max_boost_db = 3.0f);

    // Apply EQ filter to audio samples in-place.
    // Operates on real passband audio (before downconverter).
    void apply(float* audio, int count);

    bool is_configured() const { return configured_; }

    // Get the EQ response curve (dB) at each probe tone for GUI display
    const std::vector<float>& eq_curve_db() const { return eq_curve_db_; }

    // Get filter taps for diagnostics
    const std::vector<float>& taps() const { return taps_; }

    void reset();

private:
    bool configured_ = false;
    std::vector<float> taps_;
    std::vector<float> delay_line_;
    int delay_pos_ = 0;
    int sample_rate_ = 48000;

    // Cached EQ response for GUI
    std::vector<float> eq_curve_db_;

    // Design FIR from frequency-domain magnitude specification
    void design_fir(const std::vector<float>& freq_hz,
                    const std::vector<float>& gain_linear,
                    int n_taps);
};

} // namespace iris

#endif // IRIS_CHANNEL_EQ_H
