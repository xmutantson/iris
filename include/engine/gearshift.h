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

    // Force a specific level (e.g., after negotiation)
    void force_level(int level);

    // Get current level
    int current_level() const { return current_level_; }

    // Get smoothed SNR estimate
    float smoothed_snr() const { return snr_avg_; }

    void reset();

private:
    int current_level_;
    int max_level_;
    float snr_avg_;
    int hold_count_;       // Frames to hold at current level before shifting
    int fail_count_;       // Consecutive CRC failures
    static constexpr int HOLD_FRAMES = 4;    // Hold this many frames before upshift
    static constexpr int FAIL_THRESHOLD = 2;  // Downshift after this many failures
    static constexpr float SNR_ALPHA = 0.3f;  // Smoothing factor
};

} // namespace iris

#endif
