#include "probe/probe_controller.h"
#include "probe/passband_probe.h"
#include "common/logging.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

void ProbeController::reset() {
    state_ = ProbeState::IDLE;
    capture_buf_.clear();
    capture_samples_ = 0;
    capture_skip_ = 0;
    standalone_ = false;
    timeout_ = 0;
    has_results_ = false;
    analysis_done_ = false;
    got_peer_result_ = false;
    my_tx_result_ = {};
    their_tx_result_ = {};
    negotiated_ = {};
}

// --- Half-duplex probe protocol ---
//
// Turn 1 (A->B): initiator sends probe tones
// Turn 2 (B->A): responder sends RESULT + probe tones
// Turn 3 (A->B): initiator sends RESULT
//
// Both sides capture during their RX window and use the 64-tone comb
// detector to find the probe signal among any other audio.

void ProbeController::start_initiator(int sample_rate, float capture_seconds) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = true;
    capture_max_ = (int)(capture_seconds * sample_rate);

    // Turn 1: send probe tones, then start listening for Turn 2.
    // No capture_skip_ needed — modem's !ptt_active_ guard already prevents
    // feeding our own TX into the capture buffer on real radio.  On VB-Cable
    // loopback there's no PTT guard, but the sliding-window analyzer in
    // analyze_captured() finds the best offset anyway.
    generate_and_send_probe();

    state_ = ProbeState::LISTENING_PROBE;
    capture_buf_.resize(capture_max_, 0.0f);
    capture_samples_ = 0;
    timeout_ = std::max(TIMEOUT_TICKS, (int)(capture_seconds * 20) + 200);
    IRIS_LOG("[PROBE] Initiator: sent probe, listening for response (%.0fs window)", capture_seconds);
}

void ProbeController::start_responder(int sample_rate, float capture_seconds) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = false;
    capture_max_ = (int)(capture_seconds * sample_rate);

    // No message to send — just start listening for initiator's probe tones
    state_ = ProbeState::LISTENING_PROBE;
    capture_buf_.resize(capture_max_, 0.0f);
    capture_samples_ = 0;
    // Timeout must cover the full capture window + exchange time.
    // Main loop ticks at ~20/s (50ms sleep). Add 200 ticks (10s) margin
    // for result exchange after analysis.
    timeout_ = std::max(TIMEOUT_TICKS, (int)(capture_seconds * 20) + 200);
    IRIS_LOG("[PROBE] Responder: listening for probe tones (%.0fs window)", capture_seconds);
}

void ProbeController::start_standalone(int sample_rate, float capture_seconds) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = true;
    standalone_ = true;
    capture_max_ = (int)(capture_seconds * sample_rate);

    // Send probe tones, then listen for them through the audio path.
    int probe_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate);
    generate_and_send_probe();

    state_ = ProbeState::LISTENING_PROBE;
    capture_buf_.resize(capture_max_, 0.0f);
    capture_samples_ = 0;
    // Skip our own TX audio (probe duration + 250ms PTT release margin)
    capture_skip_ = probe_samples + sample_rate / 4;
    timeout_ = (int)(capture_seconds * 20) + 200;
    IRIS_LOG("[PROBE] Standalone: sent probe, listening for loopback (%.0fs window)", capture_seconds);
}

void ProbeController::feed_rx(const float* audio, int count) {
    if (state_ != ProbeState::LISTENING_PROBE) return;

    // Skip self-hearing holdoff (initiator's own TX period)
    if (capture_skip_ > 0) {
        int skip = std::min(count, capture_skip_);
        capture_skip_ -= skip;
        audio += skip;
        count -= skip;
        if (count <= 0) return;
    }

    int space = capture_max_ - capture_samples_;
    int copy = std::min(count, space);
    if (copy > 0) {
        memcpy(capture_buf_.data() + capture_samples_, audio, copy * sizeof(float));
        capture_samples_ += copy;
    }

    // Analyze when buffer is full
    if (capture_samples_ >= capture_max_) {
        analyze_captured();
    }
}

void ProbeController::analyze_captured() {
    int probe_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate_);

    // Two-phase probe finder:
    // Phase 1: Goertzel onset — cheap energy scan at 8 known probe frequencies
    //          to narrow down where probe tones might be.
    // Phase 2: Fine FFT search around the Goertzel onset (±1s window).
    // Fallback: If Phase 2 finds <3 tones, do a full-buffer scan (like pre-Goertzel
    //           code) because the onset may have locked onto AFSK/noise energy
    //           instead of probe tones.

    static const int CHECK_TONES[] = {4, 12, 20, 28, 36, 44, 52, 60};
    static const int N_CHECK = 8;

    int win_samples = sample_rate_ / 20;  // 50ms Goertzel window
    int coarse_step = sample_rate_ / 10;  // 100ms step (cheap, scan fast)
    if (coarse_step < 1) coarse_step = 1;

    // Phase 1: Goertzel energy scan
    int n_positions = 0;
    for (int pos = 0; pos + win_samples <= capture_samples_; pos += coarse_step)
        n_positions++;

    std::vector<float> energies(n_positions, 0.0f);
    int idx = 0;
    for (int pos = 0; pos + win_samples <= capture_samples_; pos += coarse_step) {
        float total_energy = 0;
        for (int t = 0; t < N_CHECK; t++) {
            float freq = probe_tone_freq(CHECK_TONES[t]);
            float k = 0.5f + ((float)win_samples * freq / sample_rate_);
            float w = 2.0f * (float)M_PI * k / win_samples;
            float coeff = 2.0f * std::cos(w);
            float s1 = 0, s2 = 0;
            for (int i = 0; i < win_samples; i++) {
                float s0 = capture_buf_[pos + i] + coeff * s1 - s2;
                s2 = s1; s1 = s0;
            }
            total_energy += s1 * s1 + s2 * s2 - coeff * s1 * s2;
        }
        energies[idx++] = total_energy;
    }

    // Find peak energy position (not onset — we want the strongest region)
    int peak_idx = (int)(std::max_element(energies.begin(), energies.end()) - energies.begin());
    float max_energy = energies[peak_idx];
    int peak_sample = peak_idx * coarse_step;

    IRIS_LOG("[PROBE] Goertzel peak at %.0f ms (energy %.0f)",
             1000.0f * peak_sample / sample_rate_, max_energy);

    // Phase 2: Fine FFT search around the Goertzel peak (±1s window).
    int search_margin = sample_rate_;  // 1 second each side
    int search_start = std::max(0, peak_sample - search_margin);
    int search_end = std::min(capture_samples_ - probe_samples, peak_sample + search_margin);
    int fine_step = sample_rate_ / 20;  // 50ms step
    if (fine_step < 1) fine_step = 1;

    int best_start = 0;
    int best_tones = 0;

    for (int start = search_start; start + probe_samples <= capture_samples_ && start <= search_end; start += fine_step) {
        ProbeResult r = probe_analyze(capture_buf_.data() + start, probe_samples, sample_rate_);
        if (r.tones_detected > best_tones) {
            best_tones = r.tones_detected;
            best_start = start;
        }
    }

    IRIS_LOG("[PROBE] Goertzel-guided search: offset %d (%.0f ms), %d tones",
             best_start, 1000.0f * best_start / sample_rate_, best_tones);

    // Fallback: if Goertzel-guided search found <3 tones, the onset locked
    // onto noise/AFSK instead of probe tones. Scan the entire buffer.
    if (best_tones < 3) {
        IRIS_LOG("[PROBE] Goertzel miss — falling back to full-buffer scan");
        int full_step = sample_rate_ / 10;  // 100ms step
        if (full_step < 1) full_step = 1;
        for (int start = 0; start + probe_samples <= capture_samples_; start += full_step) {
            ProbeResult r = probe_analyze(capture_buf_.data() + start, probe_samples, sample_rate_);
            if (r.tones_detected > best_tones) {
                best_tones = r.tones_detected;
                best_start = start;
            }
        }
        IRIS_LOG("[PROBE] Full scan: offset %d (%.0f ms), %d tones",
                 best_start, 1000.0f * best_start / sample_rate_, best_tones);
    }

    IRIS_LOG("[PROBE] Best window at offset %d (%.0f ms), %d tones matched",
             best_start, 1000.0f * best_start / sample_rate_, best_tones);

    their_tx_result_ = probe_analyze(capture_buf_.data() + best_start, probe_samples, sample_rate_);
    analysis_done_ = true;
    IRIS_LOG("[PROBE] Analyzed: %d tones detected, %.0f-%.0f Hz, valid=%d",
             their_tx_result_.tones_detected, their_tx_result_.low_hz,
             their_tx_result_.high_hz, their_tx_result_.valid ? 1 : 0);

    if (standalone_) {
        // Standalone debug mode: no peer, just show what we captured.
        // Use the captured result as both sides (self-loopback).
        my_tx_result_ = their_tx_result_;
        analysis_done_ = true;
        got_peer_result_ = true;
        try_finalize();
    } else if (is_initiator_) {
        // Initiator: we've analyzed responder's probe from Turn 2.
        // Send our RESULT back (Turn 3). If we already got their RESULT
        // (from the AFSK message in Turn 2), try_finalize completes.
        // Otherwise wait for it.
        send_result(their_tx_result_);
        state_ = ProbeState::WAITING_RESULT;
        timeout_ = TIMEOUT_TICKS;
        try_finalize();
    } else {
        // Responder: we've analyzed initiator's probe from Turn 1.
        // Turn 2: send RESULT + our own probe tones.
        send_result(their_tx_result_);
        generate_and_send_probe();
        state_ = ProbeState::WAITING_RESULT;
        timeout_ = TIMEOUT_TICKS;
        IRIS_LOG("[PROBE] Responder: sent result + probe, waiting for their result");
    }
}

void ProbeController::try_finalize() {
    if (analysis_done_ && got_peer_result_) {
        negotiated_ = probe_negotiate(my_tx_result_, their_tx_result_);
        has_results_ = true;
        state_ = ProbeState::DONE;
        IRIS_LOG("[PROBE] Complete! Negotiated: %.0f-%.0f Hz (%.0f Hz BW), valid=%d",
                 negotiated_.low_hz, negotiated_.high_hz,
                 negotiated_.bandwidth_hz, negotiated_.valid ? 1 : 0);
    }
}

void ProbeController::tick() {
    if (state_ == ProbeState::IDLE || state_ == ProbeState::DONE) return;

    if (timeout_ > 0) {
        timeout_--;
        if (timeout_ == 0) {
            IRIS_LOG("[PROBE] Timeout in state %d, aborting", (int)state_);
            if (their_tx_result_.valid || my_tx_result_.valid) {
                negotiated_ = probe_negotiate(my_tx_result_, their_tx_result_);
                has_results_ = true;
            }
            state_ = ProbeState::DONE;
        }
    }
}

void ProbeController::on_message(const uint8_t* data, size_t len) {
    if (len < 1) return;
    uint8_t type = data[0];

    if (type == PROBE_MSG_RESULT && len >= 2) {
        ProbeResult r;
        if (probe_result_decode(data + 1, len - 1, r)) {
            my_tx_result_ = r;
            got_peer_result_ = true;
            IRIS_LOG("[PROBE] Got peer result: %d tones, %.0f-%.0f Hz",
                     r.tones_detected, r.low_hz, r.high_hz);

            if (is_initiator_) {
                // Initiator got RESULT from responder's Turn 2.
                // If we've also finished our analysis, finalize.
                try_finalize();
            } else {
                // Responder got RESULT from initiator's Turn 3. Done.
                try_finalize();
            }
        }
    }
}

void ProbeController::generate_and_send_probe() {
    int n_max = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate_) + 1;
    std::vector<float> buf(n_max);
    int n = probe_generate(buf.data(), n_max, sample_rate_, 0.5f);
    if (on_send_audio) on_send_audio(buf.data(), n);
    IRIS_LOG("[PROBE] Generated %d samples (%.1f ms)", n, 1000.0f * n / sample_rate_);
}

void ProbeController::send_result(const ProbeResult& r) {
    ProbeResult r_with_caps = r;
    r_with_caps.capabilities = local_caps_;
    auto encoded = probe_result_encode(r_with_caps);
    std::vector<uint8_t> msg;
    msg.reserve(1 + encoded.size());
    msg.push_back(PROBE_MSG_RESULT);
    msg.insert(msg.end(), encoded.begin(), encoded.end());
    if (on_send_msg) on_send_msg(msg.data(), msg.size());
}

} // namespace iris
