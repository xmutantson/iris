#include "probe/probe_controller.h"
#include "probe/passband_probe.h"
#include "common/logging.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Session-reliability fix: derive the OFDM carrier grid from the single
// authoritative CMD->RSP measurement (deterministic on both ends) instead of
// the legacy min/max intersection that could desync on a lost RESULT echo.
// Default ON; IRIS_GRID_AUTHORITATIVE=0 restores the legacy intersection for
// A/B measurement.
static bool grid_authoritative_enabled() {
    const char* e = std::getenv("IRIS_GRID_AUTHORITATIVE");
    return !(e && e[0] == '0');
}

static NegotiatedPassband negotiate_grid_or_legacy(const ProbeResult& my_tx_result,
                                                   const ProbeResult& their_tx_result,
                                                   bool is_initiator) {
    // apply_grid_pin: default-inert geometry pin (IRIS_GRID_PIN_NARROW=1 ->
    // force the claim-grade 57-carrier narrow grid).  Sits at the single
    // choke point every negotiated band flows through — the probe-cache
    // store and every downstream OFDM config derivation see the pinned band,
    // so both ends stay grid-identical as long as they share the env setting
    // (see passband_probe.h).
    if (grid_authoritative_enabled())
        return apply_grid_pin(
            probe_negotiate_grid(my_tx_result, their_tx_result, is_initiator));
    return apply_grid_pin(probe_negotiate(my_tx_result, their_tx_result));
}

void ProbeController::reset() {
    state_ = ProbeState::IDLE;
    capture_buf_.clear();
    capture_samples_ = 0;
    capture_skip_ = 0;
    early_check_at_ = 0;
    standalone_ = false;
    deferred_initiator_ = false;
    ready_timeout_ = 0;
    rsp_tone_delay_countdown_ = 0;
    timeout_ = 0;
    has_results_ = false;
    analysis_done_ = false;
    got_peer_result_ = false;
    reannounce_active_ = false;
    reannounce_ticks_left_ = 0;
    reannounce_countdown_ = 0;
    my_tx_result_ = {};
    their_tx_result_ = {};
    negotiated_ = {};
}

// --- Fixed-timing slot probe protocol ---
//
// After READY handshake (AFSK), the tone exchange is strictly timed:
//
// CMD (initiator):
//   1. Send probe tones (2.25s)
//   2. on_tx_complete() fires → start capture (4.0s window)
//   3. RSP's tones arrive ~500ms after CMD TX ends
//   4. After capture: analyze, send RESULT, wait for peer RESULT
//
// RSP (responder):
//   1. Start capture immediately on PROBE:START (3.0s window, covers CMD's 2.25s)
//   2. After capture: wait 500ms turnaround gap, then send own tones
//   3. on_tx_complete() fires → analyze captured CMD tones, send RESULT
//   4. Wait for peer RESULT
//
// Key: RSP sends tones at a FIXED offset after capture ends, regardless of
// analysis. Analysis happens after the full tone exchange.

void ProbeController::start_initiator(int sample_rate, float capture_seconds) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = true;
    // Store desired capture window — used by on_tx_complete() after TX drain.
    // Manual probe passes a generous window (25s); auto-probe uses CMD_CAPTURE_WINDOW_S.
    capture_max_ = (int)(capture_seconds * sample_rate);

    // Send probe tones immediately. Transition to SENT_TONES and wait for
    // on_tx_complete() before starting capture. No capture_skip_ needed —
    // the modem's rx_mute handles self-hearing, and we don't start capture
    // until TX is fully drained.
    generate_and_send_probe();

    state_ = ProbeState::SENT_TONES;
    timeout_ = std::max(TIMEOUT_TICKS, (int)(capture_seconds * 20) + 200);
    IRIS_LOG("[PROBE] Initiator: sent probe, waiting for TX drain before capture (%.0fs window)", capture_seconds);
}

void ProbeController::start_initiator_deferred(int sample_rate) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = true;
    deferred_initiator_ = true;

    // Don't send tones yet — wait for PROBE_MSG_READY from responder.
    // The modem sends PROBE:START via AFSK; responder replies with PROBE:READY.
    // When we receive READY, we send tones and enter SENT_TONES.
    state_ = ProbeState::WAITING_READY;
    ready_timeout_ = READY_TIMEOUT_TICKS;
    IRIS_LOG("[PROBE] Initiator deferred: waiting for READY before sending tones (timeout %ds)",
             READY_TIMEOUT_TICKS / 20);
}

void ProbeController::start_responder(int sample_rate, float /*capture_seconds*/) {
    reset();
    sample_rate_ = sample_rate;
    is_initiator_ = false;
    capture_max_ = (int)(RSP_CAPTURE_WINDOW_S * sample_rate);

    // Start capturing immediately — CMD is about to send tones (or already is).
    // We capture for RSP_CAPTURE_WINDOW_S (3.0s) to cover CMD's 2.25s tones + margin.
    state_ = ProbeState::LISTENING_PROBE;
    capture_buf_.resize(capture_max_, 0.0f);
    capture_samples_ = 0;
    timeout_ = std::max(TIMEOUT_TICKS, (int)(RSP_CAPTURE_WINDOW_S * 20) + 400);
    IRIS_LOG("[PROBE] Responder: listening for probe tones (%.1fs capture window)", RSP_CAPTURE_WINDOW_S);
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

void ProbeController::on_tx_complete() {
    if (state_ == ProbeState::SENT_TONES && is_initiator_) {
        // CMD: tone TX finished draining. Start capture for RSP's response tones.
        // RSP sends tones ~500ms after CMD stops (half-duplex turnaround).
        // If capture_max_ was pre-set by start_initiator() (e.g. manual probe
        // with generous 25s window), keep it. Otherwise use CMD_CAPTURE_WINDOW_S.
        if (capture_max_ <= 0)
            capture_max_ = (int)(CMD_CAPTURE_WINDOW_S * sample_rate_);
        capture_buf_.resize(capture_max_, 0.0f);
        capture_samples_ = 0;
        capture_skip_ = 0;  // No skip needed — TX is already complete
        state_ = ProbeState::LISTENING_PROBE;
        IRIS_LOG("[PROBE] CMD TX complete — capturing for %.1fs",
                 (float)capture_max_ / sample_rate_);
    } else if (state_ == ProbeState::SENDING_PROBE && !is_initiator_) {
        // RSP: our tone TX finished draining. Now analyze the CMD tones we
        // captured earlier, send our RESULT, and wait for CMD's RESULT.
        analyze_captured();
    } else {
        IRIS_LOG("[PROBE] on_tx_complete() in unexpected state %d (initiator=%d)",
                 (int)state_, is_initiator_ ? 1 : 0);
    }
}

void ProbeController::feed_rx(const float* audio, int count) {
    if (state_ != ProbeState::LISTENING_PROBE) return;

    // Skip self-hearing holdoff (standalone mode only — normal protocol
    // uses on_tx_complete() to start capture after TX drain)
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

    // Event-driven early-exit: end the capture the instant a COMPLETE probe has
    // landed, instead of waiting out the fixed window. Throttled to bound the
    // correlation cost; only fires once enough audio for a full probe + settle
    // tail is in hand AND a chirp with a >=20 dB peak/median ratio is present.
    // If the chirp is weak/absent this never fires and the buffer-full path
    // below (the fixed window) remains the bound — the low-SNR path is unchanged.
    int probe_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate_);
    int settle_samples = (int)(EARLY_SETTLE_S * sample_rate_);
    int min_early = probe_samples + settle_samples;
    if (capture_samples_ < capture_max_ &&
        capture_samples_ >= min_early &&
        capture_samples_ >= early_check_at_) {
        early_check_at_ = capture_samples_ + (int)(EARLY_CHECK_STRIDE_S * sample_rate_);
        float peak_db = 0.0f;
        int onset = probe_find_chirp_onset(capture_buf_.data(), capture_samples_,
                                           sample_rate_, &peak_db);
        if (onset >= 0 && onset + probe_samples + settle_samples <= capture_samples_) {
            IRIS_LOG("[PROBE] Early capture-complete: full probe in hand at %d/%d samples "
                     "(onset %d, peak/med %.1f dB) — ending window early",
                     capture_samples_, capture_max_, onset, peak_db);
            on_capture_complete();
            return;
        }
    }

    // Analyze when buffer is full (fixed-window fallback / weak-signal path).
    if (capture_samples_ >= capture_max_) {
        on_capture_complete();
    }
}

// Capture is complete (window full OR a full probe arrived early). Runs the
// RSP deferred-tone transition or the CMD/standalone analyze. Guarded to
// LISTENING_PROBE so a double-trigger is a no-op.
void ProbeController::on_capture_complete() {
    if (state_ != ProbeState::LISTENING_PROBE) return;
    if (!is_initiator_ && !standalone_) {
        // RSP: capture of CMD tones complete. DON'T analyze yet.
        // First send our own tones after a turnaround gap.
        // The analysis happens in on_tx_complete() after our tones finish.
        state_ = ProbeState::SENDING_RESULT;
        rsp_tone_delay_countdown_ = RSP_TONE_DELAY_TICKS;
        IRIS_LOG("[PROBE] RSP capture complete (%d samples). Waiting %d ticks before sending tones.",
                 capture_samples_, RSP_TONE_DELAY_TICKS);
    } else {
        // CMD or standalone: analyze immediately
        analyze_captured();
    }
}

void ProbeController::analyze_captured() {
    int probe_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate_);

    // Two-phase probe finder:
    // Phase 1: Goertzel onset — cheap energy scan at 8 known probe frequencies
    //          to narrow down where probe tones might be.
    // Phase 2: Fine FFT search around the Goertzel onset (+-1s window).
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

    // Phase 2: Fine FFT search around the Goertzel peak (+-1s window).
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

    // Fallback: if Goertzel-guided search found few tones, the onset locked
    // onto noise/AFSK instead of probe tones. Scan the entire buffer.
    // Threshold 10: a valid probe has 30-64 tones; <10 means the guided
    // search missed the actual tone burst (it's elsewhere in the buffer).
    if (best_tones < 10) {
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
        // CMD: we've analyzed RSP's probe tones from our capture window.
        // FM de-emphasis attenuates RSP's low-frequency tones, so CMD often
        // detects far fewer tones than RSP (8 vs 39). If we got too few
        // tones, use symmetric assumption immediately — the channel is
        // roughly symmetric, so RSP's detection (sent via RESULT) is valid
        // for both directions.
        // FM de-emphasis attenuates RSP's low-frequency tones at CMD's receiver.
        // If CMD detects too few tones, don't send a bad RESULT — wait for RSP's
        // RESULT (which detected 39+ tones from CMD's pre-emphasized signal) and
        // use that as the authoritative passband for both directions.
        if (their_tx_result_.tones_detected < 20) {
            IRIS_LOG("[PROBE] CMD: only %d tones (FM de-emphasis) — waiting for peer RESULT",
                     their_tx_result_.tones_detected);
            // Don't send our RESULT yet — it's wrong. Wait for peer's RESULT,
            // use it as both sides' result (symmetric channel assumption).
            state_ = ProbeState::WAITING_RESULT;
            timeout_ = 300;  // 15s
            // Strand fix (D3): RSP's RESULT may have ALREADY
            // arrived (got_peer_result_) before our own analysis finished —
            // on_message() could not act on it then because analysis_done_ was
            // still false, and this branch previously returned WITHOUT
            // finalizing, stranding a RESULT we already hold until the 15s
            // timeout. Apply the authoritative-peer correction and finalize now.
            if (got_peer_result_ && my_tx_result_.valid) {
                IRIS_LOG("[PROBE] CMD: early peer RESULT already held (%d tones) — finalizing now",
                         my_tx_result_.tones_detected);
                their_tx_result_ = my_tx_result_;  // use peer's analysis for both directions
                send_result(their_tx_result_);      // corrected RESULT = our confirm to RSP
                send_result(their_tx_result_);
                try_finalize();
            }
            return;
        }
        send_result(their_tx_result_);
        send_result(their_tx_result_);
        state_ = ProbeState::WAITING_RESULT;
        // Short timeout: RSP already sent RESULT ~5s ago (AFSK delivery ~5s).
        // If it doesn't arrive in 15s, use symmetric assumption.
        timeout_ = 300;  // 15s
        try_finalize();
    } else {
        // RSP: we've analyzed CMD's probe tones. Send our analysis to CMD.
        // Apply symmetric assumption as initial my_tx_result_ (so the session
        // can upgrade to OFDM immediately without a full round-trip), but
        // remain open to receiving CMD's real analysis via on_message — see
        // line ~416 which updates my_tx_result_ and triggers re-negotiate.
        // This preserves the legacy no-deadlock behavior while letting the
        // asymmetric-channel correction take effect when available.
        send_result(their_tx_result_);
        send_result(their_tx_result_);
        my_tx_result_ = their_tx_result_;  // Symmetric seed — overwritten if CMD sends RESULT
        // Seed capabilities field from local_caps_. Peer runs the same Iris
        // binary with the same caps; assuming them lets OFDM-KISS cap
        // negotiation succeed on first probe without waiting for CMD's
        // RESULT to arrive. Real value (which will be identical) overwrites
        // this if CMD's RESULT does arrive via on_message.
        // Without this: caps=0 propagates to cache, every future reconnect
        // also loads caps=0, session stays on legacy PSK forever. See
        // fact doc §7 for the failure analysis.
        my_tx_result_.capabilities = local_caps_;
        // D1: the v4 OFDM quartet must ALSO carry the LOCAL config, exactly as
        // capabilities does — same-binary assumption.  Without this the seed
        // leaves ofdm_nfft_code=0, which the peer's decode mapped to nfft=512
        // while the initiator resolved 1024 → LDPC fails every frame → 0
        // delivery over a bit-exact cable (the D1 failure).
        my_tx_result_.ofdm_cp_samples = ofdm_cp_;
        my_tx_result_.ofdm_pilot_carrier_spacing = ofdm_pilot_carrier_;
        my_tx_result_.ofdm_pilot_symbol_spacing = ofdm_pilot_symbol_;
        my_tx_result_.ofdm_nfft_code = ofdm_nfft_code_;
        got_peer_result_ = true;
        IRIS_LOG("[PROBE] RSP: sent RESULT, symmetric seed (%d tones, caps=0x%04X assumed); will update on CMD RESULT",
                 their_tx_result_.tones_detected, local_caps_);
        try_finalize();
        // Robust RESULT re-announce (minimal unblock). The single
        // fire-once RESULT above lands ~9s before CMD opens its WAITING_RESULT
        // window; without re-announce CMD misses it and stays AFSK (~96% of
        // sessions). Re-emit on a cadence spanning CMD's window. Finalizing
        // (RX-first) does not stop this — tick() keeps re-announcing past DONE
        // until CMD confirms (on_message) or an OFDM frame decodes (modem).
        reannounce_active_ = true;
        reannounce_ticks_left_ = RESULT_REANNOUNCE_SPAN_TICKS;
        reannounce_countdown_ = RESULT_REANNOUNCE_PERIOD_TICKS;
    }
}

void ProbeController::try_finalize() {
    if (analysis_done_ && got_peer_result_) {
        negotiated_ = negotiate_grid_or_legacy(my_tx_result_, their_tx_result_, is_initiator_);
        has_results_ = true;
        state_ = ProbeState::DONE;
        IRIS_LOG("[PROBE] Complete! Negotiated: %.0f-%.0f Hz (%.0f Hz BW), valid=%d",
                 negotiated_.low_hz, negotiated_.high_hz,
                 negotiated_.bandwidth_hz, negotiated_.valid ? 1 : 0);
    }
}

void ProbeController::tick() {
    // RSP RESULT re-announce runs even after DONE (finalized = native RX active,
    // but CMD may not have received our RESULT yet). Bounded budget; stopped on
    // a positive confirm. Placed before the IDLE/DONE early-return on purpose.
    if (reannounce_active_) {
        if (reannounce_ticks_left_ <= 0) {
            reannounce_active_ = false;
            IRIS_LOG("[PROBE] RSP RESULT re-announce budget exhausted (no confirm)");
        } else {
            reannounce_ticks_left_--;
            if (--reannounce_countdown_ <= 0) {
                reannounce_countdown_ = RESULT_REANNOUNCE_PERIOD_TICKS;
                send_result(their_tx_result_);
                IRIS_LOG("[PROBE] RSP re-announcing RESULT (%d ticks left)", reannounce_ticks_left_);
            }
        }
    }

    if (state_ == ProbeState::IDLE || state_ == ProbeState::DONE) return;

    // READY handshake timeout: if responder doesn't send READY in time,
    // fall back to old timing (send tones immediately with generous capture window).
    // This handles peers running old firmware that don't send PROBE:READY.
    if (state_ == ProbeState::WAITING_READY && ready_timeout_ > 0) {
        ready_timeout_--;
        if (ready_timeout_ == 0) {
            IRIS_LOG("[PROBE] READY timeout — falling back to old timing (35s capture)");
            float fallback_capture = 35.0f;
            capture_max_ = (int)(fallback_capture * sample_rate_);
            generate_and_send_probe();
            state_ = ProbeState::LISTENING_PROBE;
            capture_buf_.resize(capture_max_, 0.0f);
            capture_samples_ = 0;
            // Fallback uses capture_skip_ since we can't rely on on_tx_complete()
            // with old firmware peers.
            int probe_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sample_rate_);
            capture_skip_ = probe_samples + sample_rate_ / 2;
            timeout_ = std::max(TIMEOUT_TICKS, (int)(fallback_capture * 20) + 200);
        }
        return;  // Don't run the main timeout while waiting for READY
    }

    // RSP deferred tone send: wait for turnaround gap after capture ends,
    // then send our own probe tones.
    if (state_ == ProbeState::SENDING_RESULT && rsp_tone_delay_countdown_ > 0) {
        rsp_tone_delay_countdown_--;
        if (rsp_tone_delay_countdown_ == 0) {
            generate_and_send_probe();
            state_ = ProbeState::SENDING_PROBE;
            IRIS_LOG("[PROBE] RSP: turnaround gap done, probe tones queued");
        }
        // Still decrement main timeout while waiting
    }

    if (timeout_ > 0) {
        timeout_--;
        if (timeout_ == 0) {
            IRIS_LOG("[PROBE] Timeout in state %d, aborting", (int)state_);
            // If we analyzed peer's probe but never got their analysis of ours,
            // assume symmetric passband: use our analysis for both directions.
            // This prevents total probe failure when one RESULT frame is lost.
            if (their_tx_result_.valid && !my_tx_result_.valid) {
                IRIS_LOG("[PROBE] Timeout: using symmetric assumption (our analysis for both directions, caps=0x%04X)",
                         local_caps_);
                my_tx_result_ = their_tx_result_;
                my_tx_result_.capabilities = local_caps_;  // same-binary assumption
                // D1: seed the v4 OFDM quartet too (see the symmetric-seed site
                // above) so a lost RESULT frame cannot leave nfft_code=0 → 512
                // split-brain (the D1 failure).
                my_tx_result_.ofdm_cp_samples = ofdm_cp_;
                my_tx_result_.ofdm_pilot_carrier_spacing = ofdm_pilot_carrier_;
                my_tx_result_.ofdm_pilot_symbol_spacing = ofdm_pilot_symbol_;
                my_tx_result_.ofdm_nfft_code = ofdm_nfft_code_;
            }
            if (their_tx_result_.valid || my_tx_result_.valid) {
                negotiated_ = negotiate_grid_or_legacy(my_tx_result_, their_tx_result_, is_initiator_);
                has_results_ = true;
            }
            state_ = ProbeState::DONE;
        }
    }
}

void ProbeController::on_message(const uint8_t* data, size_t len) {
    if (len < 1) return;
    uint8_t type = data[0];

    if (type == PROBE_MSG_READY && state_ == ProbeState::WAITING_READY && is_initiator_) {
        // Responder is listening — send tones now, enter SENT_TONES.
        // on_tx_complete() will start capture after TX drain.
        IRIS_LOG("[PROBE] Got READY from responder — sending tones (fixed-timing slot)");
        ready_timeout_ = 0;
        capture_max_ = (int)(CMD_CAPTURE_WINDOW_S * sample_rate_);
        generate_and_send_probe();
        state_ = ProbeState::SENT_TONES;
        timeout_ = std::max(TIMEOUT_TICKS, (int)(CMD_CAPTURE_WINDOW_S * 20) + 400);
        return;
    }

    if (type == PROBE_MSG_RESULT && len >= 2) {
        ProbeResult r;
        if (probe_result_decode(data + 1, len - 1, r)) {
            my_tx_result_ = r;
            got_peer_result_ = true;
            IRIS_LOG("[PROBE] Got peer result: %d tones, %.0f-%.0f Hz",
                     r.tones_detected, r.low_hz, r.high_hz);

            if (is_initiator_) {
                // CMD got RESULT from RSP.
                // If CMD's own analysis was poor (<20 tones, FM de-emphasis), use
                // RSP's result for both directions (symmetric channel assumption).
                if (analysis_done_ && their_tx_result_.tones_detected < 20) {
                    IRIS_LOG("[PROBE] CMD: using peer result as authoritative (%d tones vs our %d)",
                             r.tones_detected, their_tx_result_.tones_detected);
                    their_tx_result_ = r;  // Use RSP's analysis for "their" side too
                    // Now send our (corrected) RESULT to RSP
                    send_result(their_tx_result_);
                    send_result(their_tx_result_);
                } else if (!analysis_done_ && my_tx_result_.valid) {
                    // Peer RESULT arrived before our own tone-capture analysis ran.
                    // This is the READY-lost / capture-race case: the responder
                    // already finished its probe (RESULT in hand) and has committed
                    // to OFDM, so it no longer emits probe tones — our LISTENING
                    // capture would never complete, and the modem's 60 s probe
                    // guillotine would then strand us on AFSK forever while the
                    // responder sits OFDM-active (TX-OFDM=0 bug). The responder's
                    // analysis is authoritative for a ~symmetric channel, so adopt
                    // it for BOTH directions and finalize now instead of waiting for
                    // a capture that will never arrive. Mirrors the RSP symmetric
                    // seed (analyze_captured RSP branch) and the analyze_captured
                    // early-finalize strand fix, hoisted to fire the instant the
                    // RESULT lands regardless of our capture state.
                    IRIS_LOG("[PROBE] CMD: peer RESULT (%d tones) before own analysis — "
                             "symmetric-adopt + finalize (no capture race)", r.tones_detected);
                    their_tx_result_ = my_tx_result_;  // RSP analysis for both directions
                    analysis_done_ = true;             // we now hold a their_tx_result_
                    send_result(their_tx_result_);     // confirm to RSP — stops its re-announce
                    send_result(their_tx_result_);
                }
                try_finalize();
            } else {
                // RSP got RESULT from CMD — this is CMD's corrected/confirming
                // RESULT, the positive confirm that CMD received ours. Stop the
                // re-announce (D3 heal path) and finalize.
                reannounce_active_ = false;
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
    r_with_caps.ofdm_cp_samples = ofdm_cp_;
    r_with_caps.ofdm_pilot_carrier_spacing = ofdm_pilot_carrier_;
    r_with_caps.ofdm_pilot_symbol_spacing = ofdm_pilot_symbol_;
    r_with_caps.ofdm_nfft_code = ofdm_nfft_code_;
    auto encoded = probe_result_encode(r_with_caps);
    std::vector<uint8_t> msg;
    msg.reserve(1 + encoded.size());
    msg.push_back(PROBE_MSG_RESULT);
    msg.insert(msg.end(), encoded.begin(), encoded.end());
    if (on_send_msg) on_send_msg(msg.data(), msg.size());
}

} // namespace iris
