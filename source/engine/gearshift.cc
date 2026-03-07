#include "engine/gearshift.h"
#include <algorithm>

namespace iris {

Gearshift::Gearshift() { reset(); }

void Gearshift::reset() {
    current_level_ = 0;
    max_level_ = NUM_SPEED_LEVELS - 1;
    snr_avg_ = 0;
    hold_count_ = 0;
    fail_count_ = 0;
}

void Gearshift::set_max_level(int max_level) {
    max_level_ = std::clamp(max_level, 0, NUM_SPEED_LEVELS - 1);
    if (current_level_ > max_level_)
        current_level_ = max_level_;
}

void Gearshift::force_level(int level) {
    current_level_ = std::clamp(level, 0, max_level_);
    hold_count_ = 0;
    fail_count_ = 0;
}

int Gearshift::update(float snr_db) {
    // Smooth SNR
    snr_avg_ = SNR_ALPHA * snr_db + (1.0f - SNR_ALPHA) * snr_avg_;

    int target = snr_to_speed_level(snr_avg_);
    target = std::min(target, max_level_);

    if (target > current_level_) {
        // Upshift: hold for stability
        hold_count_++;
        if (hold_count_ >= HOLD_FRAMES) {
            current_level_++;  // Step up one level at a time
            hold_count_ = 0;
            fail_count_ = 0;
        }
    } else if (target < current_level_) {
        // Downshift: immediate
        current_level_ = target;
        hold_count_ = 0;
        fail_count_ = 0;
    } else {
        hold_count_ = 0;
    }

    return current_level_;
}

} // namespace iris
