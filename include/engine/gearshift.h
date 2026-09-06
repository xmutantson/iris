#ifndef IRIS_GEARSHIFT_H
#define IRIS_GEARSHIFT_H

#include "engine/speed_level.h"
#include <string>

namespace iris {

// Adaptive rate controller — selects speed level based on measured SNR
class Gearshift {
public:
    Gearshift();

    // Set maximum allowed speed level (from config/negotiation)
    void set_max_level(int max_level);
    void set_max_ofdm_level(int max_level);

    // Feed SNR measurement, returns recommended speed level (Mode A)
    int update(float snr_db);

    // Feed SNR measurement for OFDM O-levels (O0-O3). Same adaptive logic
    // (smoothing, LDPC boost, failure downshift, cooldown) but uses OFDM
    // speed level thresholds instead of Mode A.
    int ofdm_update(float snr_db);

    // Feed LDPC iteration count from last decoded frame.
    // Low iteration count = channel has margin beyond what SNR says.
    void feed_ldpc_iters(int iters, int max_iters);

    // Force a specific level (e.g., after negotiation)
    void force_level(int level);
    void force_ofdm_level(int level);

    // Lock level — force_level + disable gearshift updates
    void lock_level(int level);

    // Report a decode failure (CRC/LDPC non-convergence).
    // After FAIL_THRESHOLD consecutive failures, downshift one level.
    void report_failure();

    // Get current level
    int current_level() const { return current_level_; }
    int current_ofdm_level() const { return ofdm_level_; }

    // Get smoothed SNR estimate
    float smoothed_snr() const { return snr_avg_; }
    float boost() const { return ldpc_boost_; }
    int cooldown() const { return cooldown_; }

    void reset();

    // OFDM-KISS fast ramp: jump directly to target level after hold period
    // instead of incrementing by 1. Safe on stable channels (direct cable, good FM).
    void set_kiss_fast_ramp(bool enabled) { kiss_fast_ramp_ = enabled; }

    // FASTER CLIMB (connshave lever B): number of consecutive above-threshold
    // frames the OFDM gearshift holds before stepping +1.  Lowered on a clean
    // high-SNR probe so the receiver's decode-margin climb reaches the next rung
    // in fewer decoded frames.  The proposal still only ever advances +1 per step
    // (margin-climb caps target at ofdm_level+1), so it stays COUPLED to the
    // sender's actual level and never runs ahead to poison the RX blind-detect
    // (unlike a raw force_ofdm_level seed).  n<=0 restores the default.
    void set_ofdm_hold_frames(int n) { ofdm_hold_frames_ = (n > 0) ? n : OFDM_HOLD_FRAMES; }

    // climbgate Fix 1: hold the OFDM level through a transient SNR-estimate dip as
    // long as frames are still DECODING (only report_failure downshifts). The
    // deflated post-EQ EsNo jitters at the O4/O5 boundary and its immediate
    // downshift keeps resetting the O5->O6 margin-climb hold, so 32QAM never
    // accumulates the 5 easy holds it needs. "No success gate reads SNR."
    void set_climb_sticky(bool enabled) { climb_sticky_ = enabled; }

    // Speed level cache: persist proven speed levels to disk so reconnections
    // to the same callsign start near the known-good level instead of A0.
    // Cache dir is %APPDATA%/Iris (Windows) or ~/.config/iris (Linux).
    void set_cache_dir(const std::string& dir) { cache_dir_ = dir; }
    // Load cached level for callsign. Returns -1 if no cache or expired.
    int load_cached_level(const std::string& callsign);
    // Save current level for callsign (call on successful data transfer).
    void save_cached_level(const std::string& callsign);

private:
    int current_level_;
    int max_level_;
    int ofdm_level_;             // Current OFDM O-level (O0-O9)
    int max_ofdm_level_;
    bool locked_ = false;
    bool initialized_ = false;
    float snr_avg_;
    int hold_count_;       // Frames to hold at current level before shifting
    int fail_count_;       // CRC failures (leaky bucket)
    int success_since_drain_ = 0;  // Successes since last fail_count_ drain
    float ldpc_boost_;     // SNR bonus from easy LDPC convergence
    int cooldown_;         // Frames to suppress upshift after failure-driven downshift
    bool ldpc_hard_gate_ = false;  // True when worst LDPC iters > 30 — blocks upshift
    bool last_easy_decode_ = false; // True when last frame decoded with big margin (iters <= MARGIN_ITERS_EASY)
    bool last_decode_ok_ = false;  // True when the last OFDM frame DECODED (feed_ldpc_iters); false on report_failure
    bool climb_sticky_ = false;    // climbgate Fix 1: suppress SNR-dip downshift while decoding
    bool kiss_fast_ramp_ = false;  // OFDM-KISS: jump to target level (not +1 per hold)
    static constexpr int HOLD_FRAMES = 3;    // Hold this many frames before upshift (Mode A)
    static constexpr int OFDM_HOLD_FRAMES = 5; // More conservative for OFDM upshift (FM phase noise)
    int ofdm_hold_frames_ = OFDM_HOLD_FRAMES;  // runtime override (connshave lever B, clean-link climb)
    static constexpr int FAIL_THRESHOLD = 2;  // Downshift after this many failures
    static constexpr int COOLDOWN_FRAMES = 8; // Suppress upshift after failure downshift
    static constexpr float SNR_ALPHA = 0.3f;  // Smoothing factor
    // Decode-margin climb: iters <= this = huge LDPC margin -> propose +1 even
    // when the deflatable post-EQ EsNo sits below the next rung's SNR bar.
    static constexpr int MARGIN_ITERS_EASY = 2;

    std::string cache_dir_;
    static constexpr int CACHE_EXPIRY_HOURS = 24;  // Cache entries expire after 24h
};

} // namespace iris

#endif
