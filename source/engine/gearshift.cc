#include "engine/gearshift.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#ifdef _WIN32
#include <windows.h>
#endif

namespace iris {

Gearshift::Gearshift() { reset(); }

void Gearshift::reset() {
    current_level_ = 0;
    max_level_ = NUM_SPEED_LEVELS - 1;
    ofdm_level_ = 0;
    max_ofdm_level_ = NUM_OFDM_SPEED_LEVELS - 1;
    locked_ = false;
    initialized_ = false;
    snr_avg_ = 0;
    hold_count_ = 0;
    fail_count_ = 0;
    success_since_drain_ = 0;
    ldpc_boost_ = 0;
    ldpc_hard_gate_ = false;
    cooldown_ = 0;
    last_easy_decode_ = false;
    last_decode_ok_ = false;
}

void Gearshift::set_max_level(int max_level) {
    max_level_ = std::clamp(max_level, 0, NUM_SPEED_LEVELS - 1);
    if (current_level_ > max_level_)
        current_level_ = max_level_;
}

void Gearshift::set_max_ofdm_level(int max_level) {
    max_ofdm_level_ = std::clamp(max_level, 0, NUM_OFDM_SPEED_LEVELS - 1);
    if (ofdm_level_ > max_ofdm_level_)
        ofdm_level_ = max_ofdm_level_;
}

void Gearshift::force_level(int level) {
    current_level_ = std::clamp(level, 0, max_level_);
    hold_count_ = 0;
    fail_count_ = 0;
    success_since_drain_ = 0;
    // A5a: a FORCED level must not be immediately re-climbed by a stale
    // margin/boost carried from the OLD level. reset() already clears these;
    // the two runtime force_* paths (no-ACK downshift, receiver-driven leap,
    // OFDM-prepare) previously leaked them into the next ofdm_update().
    ldpc_boost_ = 0;
    ldpc_hard_gate_ = false;
    last_easy_decode_ = false;
    last_decode_ok_ = false;
    cooldown_ = 0;
}

void Gearshift::force_ofdm_level(int level) {
    ofdm_level_ = std::clamp(level, 0, max_ofdm_level_);
    hold_count_ = 0;
    fail_count_ = 0;
    success_since_drain_ = 0;
    // A5a (see force_level): clear the stale climb drivers so a forced
    // downshift/leap survives at least one hold window.
    ldpc_boost_ = 0;
    ldpc_hard_gate_ = false;
    last_easy_decode_ = false;
    last_decode_ok_ = false;
    cooldown_ = 0;
    if (ofdm_level_ == 0) {
        initialized_ = false;
        snr_avg_ = 0;
    }
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

    // Decrement cooldown on each update (each successfully decoded frame).
    // At A1 (lowest non-base level), drain cooldown 2x faster so recovery
    // from a brief glitch doesn't take 30+ seconds to re-upshift.
    if (cooldown_ > 0) {
        int drain = (current_level_ <= 1) ? 2 : 1;
        cooldown_ = std::max(0, cooldown_ - drain);
    }
    // Drain fail_count_ gradually: one success cancels one failure.
    // This allows alternating pass/fail patterns (marginal channel) to
    // accumulate and eventually trigger downshift, instead of the old
    // behavior where any success instantly reset fail_count_ to 0.
    if (fail_count_ > 0) fail_count_--;

    if (target > current_level_ && cooldown_ == 0 && !ldpc_hard_gate_) {
        // Upshift: hold for stability (suppressed during cooldown + LDPC gate)
        hold_count_++;
        if (hold_count_ >= HOLD_FRAMES) {
            int old = current_level_;
            current_level_++;  // Step up one level at a time
            hold_count_ = 0;
            printf("[GEARSHIFT] modem upshift: A%d -> A%d (SNR=%.1f dB, avg=%.1f, boost=%.1f)\n",
                   old, current_level_, snr_db, snr_avg_, ldpc_boost_);
            fflush(stdout);
        }
    } else if (target < current_level_) {
        // Downshift: immediate
        int old = current_level_;
        current_level_ = target;
        hold_count_ = 0;
        fail_count_ = 0;
        printf("[GEARSHIFT] modem downshift: A%d -> A%d (SNR=%.1f dB, avg=%.1f, target=%d)\n",
               old, current_level_, snr_db, snr_avg_, target);
        fflush(stdout);
    } else {
        // Target == current: don't reset hold_count_ — a single frame at
        // current level between two "above threshold" frames shouldn't
        // restart the hold counter. Only reset on downshift.
    }

    return current_level_;
}

int Gearshift::ofdm_update(float snr_db) {
    // Reuse the same smoothed SNR — it's one physical channel
    if (!initialized_) {
        snr_avg_ = snr_db;
        initialized_ = true;
    } else {
        snr_avg_ = SNR_ALPHA * snr_db + (1.0f - SNR_ALPHA) * snr_avg_;
    }

    int target = ofdm_snr_to_speed_level(snr_avg_ + ldpc_boost_);
    // Decode-margin climb: the post-EQ EsNo (snr_avg_) counts LDPC-correctable
    // FM phase jitter as noise and reads far below the true channel margin, so a
    // frame that LDPC decoded with huge margin (last_easy_decode_) authorizes a
    // one-rung upshift proposal even when snr_avg_ alone would latch the current
    // level. Mirrors Mercury's success/margin-driven climb (no success gate reads
    // SNR). Only PROPOSES +1; the HOLD sustain, single-step increment, ceiling
    // clamp, cooldown and report_failure() downshift below are all unchanged.
    if (last_easy_decode_ && ofdm_level_ < max_ofdm_level_)
        target = std::max(target, ofdm_level_ + 1);
    target = std::min(target, max_ofdm_level_);

    if (cooldown_ > 0) {
        int drain = (ofdm_level_ <= 1) ? 2 : 1;
        cooldown_ = std::max(0, cooldown_ - drain);
    }
    // Leaky bucket: drain fail_count_ every 4th success. On bursty
    // frequency-selective channels (alternating fail/pass), every-success
    // drain prevented fail_count_ from reaching FAIL_THRESHOLD=2.
    if (fail_count_ > 0) {
        success_since_drain_++;
        if (success_since_drain_ >= 4) {
            fail_count_--;
            success_since_drain_ = 0;
        }
    } else {
        success_since_drain_ = 0;
    }

    if (target > ofdm_level_ && cooldown_ == 0 && !ldpc_hard_gate_) {
        hold_count_++;
        if (hold_count_ >= ofdm_hold_frames_) {
            int old = ofdm_level_;
            ofdm_level_ = kiss_fast_ramp_ ? target : ofdm_level_ + 1;
            hold_count_ = 0;
            printf("[GEARSHIFT] modem OFDM upshift: O%d -> O%d (SNR=%.1f dB, avg=%.1f, boost=%.1f%s)\n",
                   old, ofdm_level_, snr_db, snr_avg_, ldpc_boost_,
                   kiss_fast_ramp_ ? ", KISS jump" : "");
            fflush(stdout);
        }
    } else if (target < ofdm_level_ && !(climb_sticky_ && last_decode_ok_)) {
        // climbgate Fix 1: with climb_sticky, a bare SNR-estimate dip does NOT
        // demote a level that is still DECODING — only report_failure does. This
        // kills the O4<->O5 jitter that kept resetting the O5->O6 margin-climb hold.
        int old = ofdm_level_;
        ofdm_level_ = target;
        hold_count_ = 0;
        fail_count_ = 0;
        printf("[GEARSHIFT] modem OFDM downshift: O%d -> O%d (SNR=%.1f dB, avg=%.1f, target=%d)\n",
               old, ofdm_level_, snr_db, snr_avg_, target);
        fflush(stdout);
    }

    return ofdm_level_;
}

void Gearshift::report_failure() {
    fail_count_++;
    hold_count_ = 0;
    // Kill boost immediately — the channel can't handle this speed
    ldpc_boost_ = 0;
    // Revoke the margin-climb authorization: a failure means the last "easy"
    // decode does not justify climbing further.
    last_easy_decode_ = false;
    last_decode_ok_ = false;   // climbgate Fix 1: a real failure re-enables SNR-downshift

    if (fail_count_ >= FAIL_THRESHOLD) {
        int old_level = current_level_;
        int old_ofdm = ofdm_level_;
        bool can_downshift = ((!locked_ && current_level_ > 0) ||
                              ofdm_level_ > 0);
        if (!locked_ && current_level_ > 0) current_level_--;
        if (ofdm_level_ > 0) ofdm_level_--;
        hold_count_ = 0;
        fail_count_ = 0;
        // Only set cooldown if we actually downshifted.  At the floor
        // (A0/O0), cooldown blocks recovery without benefit — there is
        // nowhere lower to go, so preventing re-upshift just delays
        // adaptation after transient failures.
        if (can_downshift)
            cooldown_ = COOLDOWN_FRAMES;
        printf("[GEARSHIFT] modem failure downshift: A%d->A%d O%d->O%d (cooldown=%d frames)\n",
               old_level, current_level_, old_ofdm, ofdm_level_,
               can_downshift ? cooldown_ : 0);
        fflush(stdout);
    }
}

void Gearshift::feed_ldpc_iters(int iters, int max_iters) {
    if (iters <= 0 || max_iters <= 0) return;

    // Hard gate: block upshift when LDPC barely converges (>30 of 50 iters).
    // This prevents upshift to a mode where LDPC will fail outright.
    ldpc_hard_gate_ = (iters > 30);

    // Decode-margin climb signal (consumed by the next ofdm_update, which runs
    // after this per modem.cc): a decode that converged in <= MARGIN_ITERS_EASY
    // iterations means the channel has margin the deflatable post-EQ EsNo misses.
    last_easy_decode_ = (iters <= MARGIN_ITERS_EASY);
    last_decode_ok_ = true;   // climbgate Fix 1: a frame decoded -> hold through SNR dips

    // Convergence ratio: 1 = converged immediately, 0 = used all iterations
    float ratio = 1.0f - (float)(iters - 1) / (float)max_iters;

    // If LDPC converges in ≤2 iterations, the channel has significant margin.
    // Grant up to 2 dB boost (smoothed to avoid oscillation).
    float target_boost = 0;
    if (ratio > 0.95f)       // 1-2 iters: huge margin
        target_boost = 2.0f;
    else if (ratio > 0.85f)  // 3-7 iters: good margin
        target_boost = 1.0f;
    // else: working hard, no boost

    // Smooth the boost (slow up, fast down)
    if (target_boost > ldpc_boost_)
        ldpc_boost_ += 0.3f * (target_boost - ldpc_boost_);
    else
        ldpc_boost_ = target_boost;  // drop immediately on hard decode
}

// --- Speed level cache ---

static std::mutex cache_mutex;

// Normalize callsign: uppercase and remove invalid chars
static std::string cache_key(const std::string& callsign) {
    std::string key;
    for (char c : callsign) {
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-')
            key += c;
        else if (c >= 'a' && c <= 'z')
            key += (c - 32);
    }
    return key;
}

static std::string cache_path(const std::string& dir, const std::string& callsign) {
    return dir + "/speed_cache.txt";
}

int Gearshift::load_cached_level(const std::string& callsign) {
    if (cache_dir_.empty() || callsign.empty()) return -1;
    std::string key = cache_key(callsign);
    if (key.empty()) return -1;

    FILE* f = fopen(cache_path(cache_dir_, key).c_str(), "r");
    if (!f) return -1;

    int result = -1;
    time_t now = time(nullptr);
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char call[64];
        int level;
        long long timestamp;
        if (sscanf(line, "%63s %d %lld", call, &level, &timestamp) == 3) {
            if (cache_key(call) == key) {
                long long age = (long long)now - timestamp;
                // Check expiry
                if (level > 0 && level < NUM_OFDM_SPEED_LEVELS && age >= 0 &&
                    age < (long long)CACHE_EXPIRY_HOURS * 3600) {
                    // Apply one-level safety margin (channel may have degraded)
                    result = level - 1;
                }
            }
        }
    }
    fclose(f);
    return result;
}

void Gearshift::save_cached_level(const std::string& callsign) {
    if (cache_dir_.empty() || callsign.empty()) return;
    std::string key = cache_key(callsign);
    if (key.empty()) return;
    if (ofdm_level_ <= 0) return;  // Don't cache O0 (no value)

    std::string path = cache_path(cache_dir_, key);
    std::lock_guard<std::mutex> lock(cache_mutex);

    // Read existing entries, update or append
    struct Entry { std::string call; int level; long long ts; };
    std::vector<Entry> entries;

    FILE* f = fopen(path.c_str(), "r");
    if (f) {
        char line[256];
        while (fgets(line, sizeof(line), f)) {
            Entry e;
            char call[64];
            if (sscanf(line, "%63s %d %lld", call, &e.level, &e.ts) == 3) {
                e.call = call;
                if (cache_key(call) != key)  // keep other entries
                    entries.push_back(e);
            }
        }
        fclose(f);
    }

    // Add/update our entry
    entries.push_back({key, ofdm_level_, (long long)time(nullptr)});

    // Write back (keep max 50 entries, oldest first)
    if (entries.size() > 50)
        entries.erase(entries.begin(), entries.begin() + (entries.size() - 50));

    std::string temp_path = path + ".tmp";
    f = fopen(temp_path.c_str(), "w");
    if (!f) return;

    bool ok = true;
    for (auto& e : entries) {
        if (fprintf(f, "%s %d %lld\n", e.call.c_str(), e.level, e.ts) < 0) {
            ok = false;
            break;
        }
    }
    if (fflush(f) != 0) ok = false;
    if (fclose(f) != 0) ok = false;
    if (!ok) {
        remove(temp_path.c_str());
        return;
    }
#ifdef _WIN32
    if (!MoveFileExA(temp_path.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        remove(temp_path.c_str());
#else
    if (rename(temp_path.c_str(), path.c_str()) != 0)
        remove(temp_path.c_str());
#endif
}

} // namespace iris
