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
    int ofdm_level_;             // Current OFDM O-level (O0-O7)
    int max_ofdm_level_;
    bool locked_ = false;
    bool initialized_ = false;
    float snr_avg_;
    int hold_count_;       // Frames to hold at current level before shifting
    int fail_count_;       // Consecutive CRC failures
    float ldpc_boost_;     // SNR bonus from easy LDPC convergence
    int cooldown_;         // Frames to suppress upshift after failure-driven downshift
    bool ldpc_hard_gate_ = false;  // True when worst LDPC iters > 30 — blocks upshift
    static constexpr int HOLD_FRAMES = 6;    // Hold this many frames before upshift (conservative for FM)
    static constexpr int FAIL_THRESHOLD = 2;  // Downshift after this many failures
    static constexpr int COOLDOWN_FRAMES = 8; // Suppress upshift after failure downshift
    static constexpr float SNR_ALPHA = 0.3f;  // Smoothing factor

    std::string cache_dir_;
    static constexpr int CACHE_EXPIRY_HOURS = 24;  // Cache entries expire after 24h
};

} // namespace iris

#endif
