#include "engine/gearshift.h"
#include <algorithm>

namespace iris {

Gearshift::Gearshift() { reset(); }

void Gearshift::reset() {
    current_level_ = 0;
    max_level_ = NUM_SPEED_LEVELS - 1;
    locked_ = false;
    initialized_ = false;
    snr_avg_ = 0;
    hold_count_ = 0;
    fail_count_ = 0;
    ldpc_boost_ = 0;
    cooldown_ = 0;
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

void Gearshift::lock_level(int level) {
    force_level(level);
    locked_ = true;
}

int Gearshift::update(float snr_db) {
    if (locked_) return current_level_;

    // Initialize smoothed SNR to first measurement (avoid cold-start ramp from 0)
    if (!initialized_) {
        snr_avg_ = snr_db;
        initialized_ = true;
    } else {
        snr_avg_ = SNR_ALPHA * snr_db + (1.0f - SNR_ALPHA) * snr_avg_;
    }

    // LDPC boost: if the decoder converges easily, the channel has more
    // margin than the SNR estimate captures (phase noise, timing jitter
    // inflate the noise estimate but don't hurt LDPC soft decoding).
    int target = snr_to_speed_level(snr_avg_ + ldpc_boost_);
    target = std::min(target, max_level_);

    // Decrement cooldown on each update (each successfully decoded frame)
    if (cooldown_ > 0) cooldown_--;
    // Drain fail_count_ gradually: one success cancels one failure.
    // This allows alternating pass/fail patterns (marginal channel) to
    // accumulate and eventually trigger downshift, instead of the old
    // behavior where any success instantly reset fail_count_ to 0.
    if (fail_count_ > 0) fail_count_--;

    if (target > current_level_ && cooldown_ == 0) {
        // Upshift: hold for stability (suppressed during cooldown)
        hold_count_++;
        if (hold_count_ >= HOLD_FRAMES) {
            current_level_++;  // Step up one level at a time
            hold_count_ = 0;
        }
    } else if (target < current_level_) {
        // Downshift: immediate
        current_level_ = target;
        hold_count_ = 0;
        fail_count_ = 0;
    } else {
        // Target == current: don't reset hold_count_ — a single frame at
        // current level between two "above threshold" frames shouldn't
        // restart the hold counter. Only reset on downshift.
    }

    return current_level_;
}

void Gearshift::report_failure() {
    if (locked_) return;

    fail_count_++;
    // Kill boost immediately — the channel can't handle this speed
    ldpc_boost_ = 0;

    if (fail_count_ >= FAIL_THRESHOLD && current_level_ > 0) {
        current_level_--;
        hold_count_ = 0;
        fail_count_ = 0;
        cooldown_ = COOLDOWN_FRAMES;  // Suppress re-upshift for N frames
    }
}

void Gearshift::feed_ldpc_iters(int iters, int max_iters) {
    if (locked_ || max_iters <= 0) return;

    // Convergence ratio: 1 = converged immediately, 0 = used all iterations
    float ratio = 1.0f - (float)(iters - 1) / (float)max_iters;

    // If LDPC converges in ≤2 iterations, the channel has significant margin.
    // Grant up to 3 dB boost (smoothed to avoid oscillation).
    float target_boost = 0;
    if (ratio > 0.95f)       // 1-2 iters: huge margin
        target_boost = 3.0f;
    else if (ratio > 0.85f)  // 3-7 iters: good margin
        target_boost = 1.5f;
    // else: working hard, no boost

    // Smooth the boost (slow up, fast down)
    if (target_boost > ldpc_boost_)
        ldpc_boost_ += 0.3f * (target_boost - ldpc_boost_);
    else
        ldpc_boost_ = target_boost;  // drop immediately on hard decode
}

} // namespace iris
