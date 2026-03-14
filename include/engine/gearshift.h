#ifndef IRIS_GEARSHIFT_H
#define IRIS_GEARSHIFT_H

#include "engine/speed_level.h"

namespace iris {

// Adaptive rate controller — selects speed level based on measured SNR
class Gearshift {
public:
    Gearshift();

    // Set maximum allowed speed level (from config/negotiation)
    void set_max_level(int max_level);

    // Feed SNR measurement, returns recommended speed level
    int update(float snr_db);

    // Feed LDPC iteration count from last decoded frame.
    // Low iteration count = channel has margin beyond what SNR says.
    void feed_ldpc_iters(int iters, int max_iters);

    // Force a specific level (e.g., after negotiation)
    void force_level(int level);

    // Lock level — force_level + disable gearshift updates
    void lock_level(int level);

    // Report a decode failure (CRC/LDPC non-convergence).
    // After FAIL_THRESHOLD consecutive failures, downshift one level.
    void report_failure();

    // Get current level
    int current_level() const { return current_level_; }

    // Get smoothed SNR estimate
    float smoothed_snr() const { return snr_avg_; }
    float boost() const { return ldpc_boost_; }
    int cooldown() const { return cooldown_; }

    void reset();

private:
    int current_level_;
    int max_level_;
    bool locked_ = false;
    bool initialized_ = false;
    float snr_avg_;
    int hold_count_;       // Frames to hold at current level before shifting
    int fail_count_;       // Consecutive CRC failures
    float ldpc_boost_;     // SNR bonus from easy LDPC convergence
    int cooldown_;         // Frames to suppress upshift after failure-driven downshift
    static constexpr int HOLD_FRAMES = 4;    // Hold this many frames before upshift
    static constexpr int FAIL_THRESHOLD = 2;  // Downshift after this many failures
    static constexpr int COOLDOWN_FRAMES = 8; // Suppress upshift after failure downshift
    static constexpr float SNR_ALPHA = 0.3f;  // Smoothing factor
};

} // namespace iris

#endif
