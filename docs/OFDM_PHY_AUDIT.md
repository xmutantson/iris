# Iris OFDM-over-FM PHY — Full Audit Report

**Date:** 2026-03-17
**Scope:** 38 topics across 8 domains, opus-level analysis, read-only audit
**Build:** a231067b (pre-audit fixes: sync cache, MMSE header, diagnostics, buffer trim, self-hear full clear, noise_var boost compensation, quadratic noise estimator)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [CRITICAL Findings](#critical-findings)
3. [HIGH Findings](#high-findings)
4. [MEDIUM Findings](#medium-findings)
5. [LOW Findings](#low-findings)
6. [Prioritized Fix List](#prioritized-fix-list)
7. [Detailed Analysis by Domain](#detailed-analysis-by-domain)
   - [FM Radio-Specific](#fm-radio-specific)
   - [OFDM Synchronization & Estimation](#ofdm-synchronization--estimation)
   - [OFDM Equalization & Pilots](#ofdm-equalization--pilots)
   - [FEC & Bit Pipeline](#fec--bit-pipeline)
   - [ARQ & Protocol](#arq--protocol)
   - [Numerical Stability](#numerical-stability)
   - [Thread Safety & Memory](#thread-safety--memory)
   - [Platform Audio](#platform-audio)

---

## Executive Summary

| Severity | Count |
|----------|-------|
| CRITICAL | 2 |
| HIGH | 8 |
| MEDIUM | 12 |
| LOW | 16 |
| **Total** | **38** |

The FEC/bit pipeline is clean — all 6 topics rated LOW. Interleavers, scrambler, bit packing, CRC placement, and LDPC decoder are correct and consistent TX↔RX.

The biggest issues are:
- **Thread safety (C1):** The `Modem` class is a single-threaded design called from 3 concurrent threads with no synchronization on shared mutable state.
- **WASAPI sample rate (C2):** No validation that device rate matches expected 48kHz — silent total failure.
- **PAPR clipping disabled (H1):** The FM radio's deviation limiter hard-clips uncontrolled OFDM peaks.
- **No CPE correction (H2):** FM discriminator phase noise accumulates over frames with no per-symbol correction.
- **ACK at data rate (H3):** Control frames sent at up to 256QAM 7/8 — lost ACKs cascade into full batch retransmits.

---

## CRITICAL Findings

### C1. Thread Safety: Shared Mutable State With No Synchronization

**Files:** `modem.cc:638-2507`, `modem.h:273-499`

`process_rx` (delivery thread), `process_tx` (producer thread), and `tick()` (main thread) all operate on the same `Modem` object concurrently. Dozens of non-atomic, non-mutex-protected members are shared:

- `ofdm_rx_audio_buf_` — written in process_rx (line 1114), cleared in process_rx (line 669), inserted by process_rx_native
- `rx_mute_holdoff_` — set by `ptt_off()` from TX thread (line 616), decremented by RX thread (line 664). Non-atomic concurrent read-modify-write
- `rx_overlap_buf_` — written by process_rx_native, cleared by state callbacks from main thread (line 400, 569)
- `pending_frame_start_`, `pending_frame_timeout_` — written by process_rx_native (1830, 1836), read by process_tx (1997, 2017-2020)
- `snr_db_`, `snr_preamble_db_` — written by RX (1279-1280), read by TX path (433)
- `ofdm_sync_cached_` — written by RX (1131, 1188), cleared by state callback from main thread (494)
- `tx_buffer_`, `tx_pos_` — accessed outside `tx_mutex_` scope (2481-2503)
- ARQ session state — `arq_listen()` from RX thread, `arq_.tick()` from main thread
- AX.25 session — ticked from main thread, fed frames from delivery thread

`ofdm_rx_audio_buf_` being cleared by one thread while another inserts is a guaranteed crash under the right timing. `rx_mute_holdoff_` concurrent read-modify-write can produce negative values or skipped holdoff. Formally UB per C++ standard; benign on x86 by accident but breaks on ARM or with aggressive optimizations.

**Recommendation:** Either restructure to single-thread dispatch (queue-based architecture where audio threads post events and a single processing thread handles all state) or add a `modem_mutex_` protecting all shared state accessed from process_rx, process_tx, and tick. The queue-based approach is preferred for a real-time audio system.

---

### C2. No WASAPI Sample Rate Validation

**File:** `audio_wasapi.cc:162-176`

`GetMixFormat()` returns the device's native rate. The code uses it directly without comparing to the expected 48kHz. If the Windows audio engine's shared-mode rate is 44100 Hz or 96000 Hz, every timing calculation is silently wrong: subcarrier spacing, symbol duration, CP, CFO range. The modem produces zero decodes with no error message. The `actual_rate` is logged but never compared to `sample_rate_`.

**Recommendation:** After `GetMixFormat`, compare `actual_rate` to `sample_rate_`. If they differ, reject with a clear error message. Windows lets users set the shared-mode rate in Sound Settings.

---

## HIGH Findings

### H1. PAPR Clipping Disabled — FM Radio Hard-Clips Uncontrolled

**File:** `ofdm_mod.cc:551-571`

```cpp
constexpr float CLIP_RATIO = 99.0f;  // FM is constant-envelope — no PAPR clipping needed
```

This is incorrect. While FM RF output is constant-envelope, the **audio input** to the FM radio is NOT. OFDM with ~30 carriers has ~15 dB PAPR. The radio's deviation limiter hard-clips peaks unpredictably:
- Pre-emphasis boosts high-frequency peaks by 6+ dB
- Different radios clip differently — untestable
- Hard clipping destroys subcarrier orthogonality and creates ICI
- The `modem.cc:2338-2353` normalization targets RMS=0.35 with hard clip at ±0.95 (only 8.7 dB headroom)

**Recommendation:** Re-enable soft clipping at CLIP_RATIO=2.5 (8 dB PAPR). Add a post-clip bandpass FIR matched to the OFDM bandwidth (63-tap) to suppress spectral regrowth. Alternatively, implement iterative clip-and-filter (3-4 iterations).

---

### H2. No Per-Symbol Common Phase Error (CPE) Correction

**File:** `ofdm_demod.cc:596-665` (data symbol loop — no CPE step)

FM discriminator phase noise causes symbol-to-symbol phase wander. No CPE estimation or correction exists. The comb pilot IIR (alpha=0.3) is too slow to track per-symbol rotation — it takes ~3 symbols to track a phase step.

Over long frames (~850ms at low rates), accumulated phase drift can exceed constellation decision boundaries. 256QAM is offered (tone map id 8) but infeasible on real FM radios due to ICI floor (~-45 dB from FM discriminator phase noise).

**Recommendation:** After FFT of each data symbol, compute mean phase rotation of all comb pilots relative to expected values. Apply conjugate to all subcarriers before equalization:
```
cpe = mean_phase(Y_pilot[k] * conj(H[k])) for all pilot k
Y_corrected[k] = Y[k] * exp(-j * cpe)
```
This is simple, low-cost, and critical for 16QAM+. Also cap maximum offered modulation at 64QAM for FM, or gate 256QAM behind a lab-mode flag.

---

### H3. ACK/NAK Sent at Data Rate Instead of Robust Rate

**File:** `modem.cc:2270-2304`

All ARQ control frames (ACK, NACK, SET_SPEED, BREAK, DISCONNECT) are modulated at the current negotiated speed — up to 256QAM 7/8. A 5-byte ACK at 256QAM requires 25 dB SNR. If the channel has degraded even slightly, the ACK fails, causing a full batch retransmit (3-9 seconds of wasted airtime). Self-reinforcing: if ACK fails, the retransmitted data batch also fails.

**Recommendation:** In the `send_arq_frame` callback, tag control frames (ACK, NACK, SET_SPEED, SET_CONFIG, BREAK, SWITCH_ROLE, DISCONNECT) with a "use robust rate" flag. In the TX path, check this flag and force BPSK 1/2. Extra airtime for a 5-byte BPSK ACK is ~20ms — negligible vs 3-9s batch retransmit cost.

---

### H4. Zeroing Gate Discards Usable Soft Information

**File:** `ofdm_demod.cc:266-272` (data), `ofdm_demod.cc:140-142` (header)

```cpp
if (H_mag2 < nv) {
    eq.push_back({0.0f, 0.0f});  // Deep fade: zero this carrier
```

The hard zeroing gate at `H_mag2 < nv` (0 dB carrier SNR) creates a cliff: a carrier at 0.99×nv is completely erased; one at 1.01×nv gets full MMSE treatment. Carriers at 0-3 dB SNR contribute nothing instead of contributing weak-but-useful LLRs to LDPC.

**Note on CSI double-penalty:** Two agents disagreed. The combined `sigma_sq` + `reliability` weight actually produces valid MRC-equivalent LLRs (the FEC agent proved this mathematically). The zeroing gate is the real problem.

**Recommendation:** Remove the zeroing gate entirely, or lower to `H_mag2 < 1e-4f * nv` as a numerical safety floor. With proper MMSE + correct sigma_sq, deeply-faded carriers naturally produce near-zero LLRs without any hard gate.

---

### H5. No WASAPI Audio Format Validation

**File:** `audio_wasapi.cc:220` (capture), `audio_wasapi.cc:464` (playback)

```cpp
float* fdata = (float*)data;  // Blindly casts WASAPI buffer to float*
```

Never verifies `wFormatTag == WAVE_FORMAT_IEEE_FLOAT` or checks the `WAVEFORMATEXTENSIBLE` subformat. On rare audio devices or certain drivers, `GetMixFormat` could return 16-bit PCM or 24-bit packed.

**Recommendation:** After `GetMixFormat`, assert `wBitsPerSample == 32` and format tag is IEEE float. Log error and refuse to start if not.

---

### H6. No Audio Device Disconnect Detection

**File:** `audio_wasapi.cc:200-228` (capture), `audio_wasapi.cc:451-491` (playback)

If a USB soundcard is unplugged, `GetBuffer()` returns `AUDCLNT_E_DEVICE_INVALIDATED`. The code checks `SUCCEEDED(hr)` but only `continue`s on failure, busy-looping with `Sleep(2/5ms)` forever. No reconnection logic, no user notification.

**Recommendation:** Check for `AUDCLNT_E_DEVICE_INVALIDATED` and `AUDCLNT_E_SERVICE_NOT_RUNNING`. On detection, log error, set `running_ = false`, signal modem to attempt recovery or notify user. Add a watchdog: if no capture packets arrive for >2s, log error.

---

### H7. Heap Allocation on Every Audio Callback

**File:** `modem.cc:698`, `main.cc:587, 605`

```cpp
std::vector<float> audio(rx_audio, rx_audio + frame_count);  // ~94 allocs/sec
```

`process_rx` allocates a new `vector<float>` on every callback. `main.cc` mono downmix callbacks also allocate per-invocation. These are on the real-time audio thread path.

**Recommendation:** Pre-allocate a member buffer (e.g., `rx_audio_buf_`) sized to the maximum expected frame_count. Reuse it across callbacks. For mono downmix in main.cc, use a static or thread-local buffer.

---

### H8. No SFO (Sampling Frequency Offset) Tracking

**File:** Architectural gap across `ofdm_demod.cc`

Sample rate mismatch between TX/RX soundcards causes progressive timing drift. Over a long frame (100 symbols, ~1.2s), 100 ppm clock error → ~120 us drift (~10% of CP=16). No tracking mechanism exists. The Schmidl-Cox coarse CFO handles frequency offset but not the time-domain drift from sample clock mismatch.

**Recommendation:** Estimate SFO from the phase slope of block pilot channel estimates over time. Apply fractional sample resampling or adjust the CP stripping offset progressively across the frame. Alternatively, increase CP to provide more margin (see M2).

---

## MEDIUM Findings

### M1. No TX Pre-Emphasis Compensation

**File:** `ofdm_mod.cc` (no pre-emphasis filter exists)

FM radios apply ~6 dB/octave pre-emphasis on TX and de-emphasis on RX. The modem relies on per-carrier channel estimation to absorb the tilt, but this means high-frequency carriers have ~9 dB less received power. Waterfill loads fewer bits on these carriers than it could with compensation.

**Recommendation:** Apply per-carrier TX amplitude tilt based on probe-measured channel response (inverse of measured response, capped at +6 dB) in the frequency domain before IFFT.

---

### M2. CP=16 Runtime Override Despite Spec Analyzing CP=64

**File:** `modem.cc:3172-3173`

```cpp
if (cp <= 0 || cp == 64) cp = 16;  // FM default: minimal multipath
```

The spec (`OFDM_PHY_SPEC.md`) analyzed CP=64 (1.33ms) and concluded it was sufficient. The runtime code uses CP=16 (0.33ms) — 4× shorter. Adequate for RF multipath but marginal for radio IF filter group delay (100-500 us).

**Recommendation:** Raise FM default from CP=16 to CP=32 (0.67ms). Make the override configurable via INI rather than hardcoded.

---

### M3. Pilot Spacing Too Sparse at Narrow Bandwidth

**File:** `modem.cc:3178-3179`

Default carrier pilot spacing 1:8 yields only 2-3 pilots in a 2 kHz bandwidth. A single bad pilot (noise hit) corrupts the entire channel estimate via linear interpolation.

**Recommendation:** Change default to 1:6. Add a minimum pilot count check: if `n_pilot_carriers < 4`, force denser spacing.

---

### M4. noise_var Floor Not Re-Applied After Preamble Boost Compensation

**File:** `ofdm_demod.cc:440-441, 485-486`

After dividing noise_var by BOOST_SQ (2.0), values originally at the 1e-6 floor drop to 5e-7, below the floor. This makes the MMSE equalizer slightly too aggressive on carriers where the original noise estimate was at floor.

**Recommendation:** Re-apply floor after compensation:
```cpp
for (auto& nv : channel_est_.noise_var) {
    nv /= BOOST_SQ;
    if (nv < 1e-6f) nv = 1e-6f;
}
```

---

### M5. ofdm_rx_audio_buf_ Has No Size Cap

**File:** `modem.cc:1114`

Unlike `rx_overlap_buf_` (capped at 20s), `ofdm_rx_audio_buf_` grows without limit if the demodulator fails to consume data (e.g., repeated false-positive preamble detections).

**Recommendation:** Add a cap similar to `RX_OVERLAP_MAX` (e.g., 10 seconds at 48kHz = 480,000 samples). Trim from front on overflow.

---

### M6. tx_queue_ Has No Size Cap

**File:** `modem.h`

`std::queue<vector<uint8_t>>` with no depth limit. If ARQ generates retransmit frames while DCD blocks TX, the queue grows unbounded.

**Recommendation:** Add maximum queue depth (e.g., 32 frames). Drop oldest or block producer on overflow.

---

### M7. Waterfill Power Normalization Starves Low-QAM Carriers

**File:** `ofdm_mod.cc:159-165`, `modem.cc:2338-2353`

Total-power normalization (RMS=0.35) is correct for uniform tone maps. But with waterfill mixed modulations, QAM256 carriers (avg energy ~82) dominate deviation while BPSK carriers (energy ~1) are starved.

**Recommendation:** Normalize each carrier's constellation to unit average power before IFFT. The RX channel estimator already measures per-carrier gain and compensates.

---

### M8. Dual Gearshift Systems Can Conflict

**File:** `gearshift.cc`, `arq.cc:614-659`

Modem-level gearshift (SNR-based, alpha=0.3 EMA) and ARQ-level gearshift (ACK/NACK count-based) operate independently. Modem may suggest upshift based on local RX SNR while ARQ is in cooldown. They can also disagree on asymmetric links.

**Recommendation:** Add diagnostic logging when they disagree. After BREAK recovery that settles below original level, set longer cooldown (10 blocks instead of 3).

---

### M9. Comb Pilot Noise/Channel Conflation

**File:** `ofdm_sync.cc:346-355`

Comb pilot residuals `|H_new - H_old|` capture both actual noise AND channel variation. In mobile scenarios (vehicle-to-vehicle), channel changes fast enough that noise is overestimated by 3-6 dB, making MMSE too conservative.

**Recommendation:** Use variance of pilot residuals after subtracting a linear phase trend (which captures Doppler) rather than raw residuals.

---

### M10. Preamble Detection Margin Tight

**File:** `ofdm_sync.cc:61`

Worst OTA M=0.75 vs threshold 0.70 = only 7% margin. Additional impairments (multipath, co-channel interference) could push below threshold intermittently.

**Recommendation:** Log metric value on every detection for histogram-based threshold tuning. If missed detections become a problem, lower to 0.65 as last resort.

---

### M11. IRIS_LOG Mutex + fflush Blocks Audio Threads

**File:** `logging.h:45-76`

IRIS_LOG acquires a mutex, formats string, calls `fflush(stdout)` and `fflush(file)`. Called 173 times from modem.cc. Disk I/O can take 1-50ms, blocking the audio delivery thread and potentially causing WASAPI capture FIFO overflow.

**Recommendation:** Switch to async logging: push formatted messages to a lock-free ring buffer, drain from a dedicated low-priority logging thread. Or at minimum, remove `fflush` from the hot path and let stdio buffer.

---

### M12. Playback FIFO Uses O(n) Vector Erase

**File:** `audio_wasapi.cc:474`

```cpp
fifo_.erase(fifo_.begin(), fifo_.begin() + copied);  // O(n) shift
```

With 100ms FIFO at 48kHz, this copies up to 9600 floats (~38KB) every 2ms render wakeup.

**Recommendation:** Replace with `std::deque` or a ring buffer with read/write indices.

---

## LOW Findings

| ID | Finding | Domain |
|----|---------|--------|
| L1 | Timing sync left-edge detection adequate for FM delay spreads (<50 us) | OFDM Sync |
| L2 | CFO range ±93.75 Hz sufficient for audio-coupled FM | OFDM Sync |
| L3 | Group delay: CP=1.33ms covers typical FM discriminator delay | FM |
| L4 | Squelch: auto-DCD with inverted-squelch detection works well | FM |
| L5 | Passband: probe-adaptive carrier placement with 1-carrier guard adequate | FM |
| L6 | FM capture effect: correctly handled at radio layer, AGC disabled for OFDM (correct) | FM |
| L7 | CTCSS: probe detection + 300 Hz minimum protect adequately | FM |
| L8 | Repeater courtesy tones: Schmidl-Cox + CRC-8 reject false triggers | FM |
| L9 | FEC pipeline clean: interleavers, scrambler, bit packing all consistent TX↔RX | FEC |
| L10 | LDPC decoder: layered min-sum 0.80 scale, 50 iters is industry-standard | FEC |
| L11 | CRC-8 header + CRC-32 payload: cascading protection gives ~10⁻¹² undetected error rate | FEC |
| L12 | Self-hear guard: dual system (200ms mute + 300ms guard + buffer clear) is robust | ARQ |
| L13 | Timeouts appropriate for FM RTT; min ACK timeout floor (300ms) unreachable in practice | ARQ |
| L14 | DSP division guards: all major denominators have explicit near-zero checks | Numerical |
| L15 | Phase accumulation properly wrapped to [-π,π] | Numerical |
| L16 | SPA decoder tanh/atanh clamped; min-sum posteriors structurally self-limiting | Numerical |

---

## Prioritized Fix List

| Priority | Item | Effort | Impact |
|----------|------|--------|--------|
| 1 | **C2**: Sample rate validation gate | 1 line | Prevents silent total failure |
| 2 | **H1**: Re-enable PAPR clipping at 2.5x RMS | Small | Enables reliable OTA with any radio |
| 3 | **H3**: Force BPSK 1/2 for control frames | Small | Prevents cascading retransmit timeouts |
| 4 | **H2**: Per-symbol CPE correction from pilots | Medium | Unlocks reliable 16QAM+, fixes phase drift |
| 5 | **H4**: Remove zeroing gate or lower to 1e-4×nv | 1 line | Recovers soft info from faded carriers |
| 6 | **M4**: Re-apply noise_var floor after boost | 2 lines | Prevents MMSE over-aggressiveness |
| 7 | **H5+H6**: Format validation + disconnect detection | Medium | Prevents silent audio failures |
| 8 | **M5+M6**: Cap ofdm_rx_audio_buf_ and tx_queue_ | Small | Prevents unbounded memory growth |
| 9 | **H7**: Pre-allocate audio buffers | Medium | Reduces jitter on real-time path |
| 10 | **M2**: Raise CP default to 32 | 1 line | More margin for radio group delay |
| 11 | **M3**: Increase default pilot density to 1:6 | Small | More robust channel interpolation |
| 12 | **M1**: TX pre-emphasis compensation | Medium | Better waterfill capacity |
| 13 | **M7**: Per-carrier power normalization | Medium | Fair deviation budget for waterfill |
| 14 | **M11**: Async logging | Medium | Prevents audio thread blocking |
| 15 | **M12**: Ring buffer for playback FIFO | Small | Reduces render thread jitter |
| 16 | **M8**: Dual gearshift reconciliation | Small | Better diagnostics |
| 17 | **M9**: Comb pilot Doppler separation | Medium | Better noise estimate in mobile |
| 18 | **M10**: Detection metric logging | Small | Data for threshold tuning |
| 19 | **H8**: SFO tracking | Large | Long-frame reliability with mismatched clocks |
| 20 | **C1**: Thread safety overhaul | Large | Eliminates UB, prevents rare crashes |

---

## Detailed Analysis by Domain

### FM Radio-Specific

#### Pre-emphasis/De-emphasis (M1)
The modem does NOT apply any TX pre-emphasis or RX re-emphasis filter. It relies on per-carrier channel estimation to absorb the FM spectral tilt. The quadratic noise estimator (7-tap, recently fixed) tracks the slope well. However, high-frequency carriers receive ~9 dB less power, reducing waterfill capacity. The probe (`passband_probe.cc:31-49`) does compensate for pre-emphasis during measurement, but this information is not used to shape the OFDM TX spectrum.

#### Deviation Limiting / PAPR (H1)
`CLIP_RATIO = 99.0` effectively disables soft clipping. Comment "FM is constant-envelope" is incorrect for the audio input stage. With ~15 dB PAPR and 8.7 dB headroom (RMS=0.35, clip at ±0.95), peaks 6+ dB above headroom are hard-clipped by the radio's deviation limiter. Pre-emphasis makes it worse (+6 dB on high-frequency peaks). PAPR is logged (`ofdm_mod.cc:573-587`) but not controlled.

#### Group Delay (L3)
CP=64 samples = 1.33ms at 48kHz covers typical FM discriminator group delay (0.2-0.5ms passband, 1-2ms at edges). The runtime CP=16 override (M2) reduces this margin to 0.33ms, which is marginal. The MMSE equalizer handles per-carrier phase rotation from group delay.

#### Squelch (L4)
Auto-DCD with baseline calibration (2s at startup), 500ms holdoff, inverted-squelch detection (`modem.cc:817-822`). Post-TX tail of 100ms minimum keeps peer squelch open. No squelch-close detection to abort demodulation early.

#### Passband Boundaries (L5)
Probe sweeps 300-4500 Hz. Edge margin 25 Hz. Guard: 1 carrier each edge (93.75 Hz). For probed band 300-3200 Hz: lowest data carrier at 375 Hz, highest at 3094 Hz. Adequate with waterfill handling edge SNR.

#### Audio Latency (C2-related)
WASAPI shared mode, 5ms capture poll, 2ms render poll. Two-thread FIFO architecture decouples audio I/O from processing. Buffer size 100ms (4800 frames). No sample rate validation (C2). No resampling. Playback hardcodes 50ms buffer ignoring requested `buffer_frames_`.

#### Capture Effect / AGC (L6)
AGC disabled for OFDM (`modem.cc:699-702`) — correct, would distort QAM constellation. Fixed `native_rx_gain_` from auto-tune calibration. FM capture effect is radio-layer, not modem-layer.

#### CTCSS (L7)
CLI warns band_low must be >254 Hz. Default 1200 Hz is safe. Probe starts at 300 Hz with guard. CTCSS 2nd harmonics (up to 508 Hz) are within OFDM range but probe detection would catch elevated noise floor.

#### Repeater Interactions (L8)
No specific handling. Schmidl-Cox metric for single tones ≈ 0.1-0.3 (well below 0.70 threshold). CRC-8 header provides secondary rejection. Consider configurable pre-TX silence for repeater mode.

#### Deviation Control (M7)
Total-power normalization (RMS=0.35) is essential and correct. But waterfill with mixed modulations: QAM256 energy (~82) vs BPSK (~1) means high-QAM carriers dominate deviation. Per-carrier power equalization before IFFT would maximize capacity.

---

### OFDM Synchronization & Estimation

#### Timing Sync (L1)
Schmidl-Cox with first-threshold-crossing (left edge of plateau). Uses improved Minn-Bhargava normalization `|P|²/(E·R)`. Left-edge timing is safe because CP stripping (`pos + cp`) advances past CP before FFT. FM delay spreads (<50 us) are well within CP.

#### CFO Range (L2)
Coarse: ±Fs/NFFT = ±93.75 Hz (from training symbol 1 half-length autocorrelation). Fine: ±48000/(2×576) = ±41.67 Hz (differential phase between training symbols). Total acquisition: ±93.75 Hz. For audio-coupled FM, the only CFO source is soundcard clock mismatch (<2.4 Hz at 50 ppm). More than sufficient.

#### Channel Tracking (M9)
Initial: one-shot LS from training symbol 2. Block pilots: IIR alpha=0.3, every 8-16 data symbols. Comb pilots: IIR alpha=0.3, every symbol. Comb pilot tracking bandwidth ~4 Hz (adequate for FM). Noise/channel conflation in residuals overestimates noise in mobile scenarios.

#### Noise Variance (M4)
Quadratic fit (7-tap, recently implemented) is mathematically correct. Cramer's rule for constant term verified. Edge carrier handling adequate (4+ points at edges). The boost compensation (`÷BOOST_SQ`) is correct but noise_var floor (1e-6) should be re-applied after division.

#### Detection Threshold (M10)
0.70 threshold with first-crossing. OTA M=0.75-0.90. Margin 7% at worst case. CRC-8 header guards false positives (0.39% undetected). Preamble boost (√2) provides ~3 dB detection margin. FM processing (pre-emphasis + deviation limiting + de-emphasis) degrades metric.

---

### OFDM Equalization & Pilots

#### Equalization Strategy (H4)
MMSE with zeroing gate: `H_mag2 < nv → zero`. The MMSE formula `conj(H)*Y/(|H|²+nv)` is correct. The CSI reliability weight `|H|²/(|H|²+nv)` combined with `sigma_sq = nv/(|H|²+nv)` in the demapper produces MRC-equivalent LLRs — mathematically valid. The zeroing gate is the problem: hard cliff at 0 dB carrier SNR discards usable soft information.

#### Guard Interval (M2)
Spec analyzed CP=64 (1.33ms). Runtime overrides to CP=16 (0.33ms). Adequate for RF multipath on VHF/UHF. Marginal for radio IF filter group delay and software audio chain delay. CP=32 (0.67ms) recommended as compromise.

#### Pilot Density (M3)
Default 1:8 carrier, 1:16 block. At 2 kHz BW: only 2-3 carrier pilots. Auto-training adjusts based on probe flatness (1:4 for selective channels). Block pilot spacing adequate (FM channel is quasi-static).

#### PAPR / FM Deviation (H1)
See FM section. Soft clipping disabled. No PAPR reduction. The 15-tap post-clip FIR was removed as too short. No tone reservation. The spec recommends clip at 3.0× RMS but code overrides to 99×.

#### Phase Noise / ICI (H2)
No CPE correction. No ICI cancellation. Spec claims ICI floor ~-45 dB at 93.75 Hz spacing. This supports BPSK/QPSK/16QAM but is marginal for 64QAM and insufficient for 256QAM. The IIR pilot tracking (alpha=0.3) is too slow for symbol-rate phase noise.

---

### FEC & Bit Pipeline

All 6 topics rated **LOW**. No bugs found.

- **LLR quality:** sigma_sq = nv/(|H|²+nv) is correct for MMSE output. Combined with reliability weight produces MRC-equivalent LLRs. ±20 clamp appropriate for min-sum.
- **Interleaver depth:** Three-level interleaving (stride-41 per LDPC block, BICM column-row, stride-173 global) spreads a 5-carrier fade across multiple LDPC blocks with ~40-60 erased bits per block — well within LDPC correction at rate 1/2.
- **LDPC convergence:** Layered min-sum, scale 0.80, 50 iterations. Industry-standard. Tail bit hardening (+20 LLR for known-zero padding) correctly implemented.
- **CRC coverage:** CRC-8 on 28-bit header (1/256 miss rate). CRC-32 on payload before FEC. Cascading: undetected error ≈ 10⁻¹².
- **Scrambler sync:** TX and RX use identical LFSR (x¹⁵+x¹⁴+1, seed 0x6959). Different code ordering is harmless (both read LSB before advancing). Fixed seed per frame is standard.
- **Bit packing:** Header MSB-first (consistent TX↔RX). Payload LSB-first (consistent). CRC-32 little-endian (consistent). Constellation mapping LSB-first (consistent with demapper).

---

### ARQ & Protocol

#### Half-Duplex Turnaround (L12/M-related)
TX drain → PTT release → 200ms RX mute holdoff → buffer clear. Total one-way ~300-450ms. The 200ms mute discards all audio — fast-responding remotes could have their first ~200ms dropped. Configurable holdoff recommended.

#### Self-Hearing (L12)
Dual guard: 200ms RX mute + 300ms self-hear timer + full buffer clear on OFDM detection during guard. Robust. Edge case: radios with >300ms pipeline latency (Bluetooth adapters).

#### ACK Reliability (H3)
Control frames at data rate. 5-byte ACK at 256QAM 7/8 needs 25 dB SNR. Lost ACK → full batch retransmit (3-9s). Self-reinforcing failure. VARA HF always sends control at lowest rate.

#### Gearshift (M8)
Two independent systems: modem-level (SNR EMA, 4-frame hold, 3-frame cooldown) and ARQ-level (3 consecutive ACKs to upshift, 2 NACKs to downshift, BREAK on first NACK after upshift). BREAK escalation (1,2,4,4... step drops) mitigates hunting. Can conflict on asymmetric links.

#### Buffers (M5, M6)
`rx_overlap_buf_`: 20s cap (good). `ofdm_rx_audio_buf_`: no cap (bad). `tx_queue_`: no cap (bad). WASAPI capture FIFO: 10s cap (good). `process_tx` uses `try_to_lock` on tx_mutex_ (good for real-time).

#### Timeouts (L13)
HAIL 3500ms (adequate with 1266ms margin). Default ACK 2000ms (tight but workable). Min ACK floor 300ms (unreachable over FM). No idle keepalive — dead peer takes ~2 minutes to detect via retransmit/BREAK cycle.

---

### Numerical Stability

All major divisions guarded. Phase wrapped. LLRs clamped ±20. Noise_var floored at 1e-6. Schmidl-Cox denominator guarded at 1e-20. SNR computation guarded with log10(max(x, 1e-10)). MMSE denom guarded at 1e-12. AGC gain clamped [0.01, 100]. Quadratic regression determinant guarded at 1e-12.

Key gaps:
- **H8/SFO:** No sample clock tracking for mismatched soundcards.
- **M4:** noise_var floor not re-applied after boost compensation.
- GBF decoder `llr_est` unbounded (non-default decoder).
- Pure noise input causes full Schmidl-Cox scan every buffer (expected, modest CPU).

---

### Thread Safety & Memory

#### Thread Safety (C1)
See CRITICAL section. The Modem class is single-threaded design called from 3 threads. No mutex on shared state. Race on `rx_mute_holdoff_` (concurrent read-modify-write). Race on `ofdm_rx_audio_buf_` (concurrent clear+insert). Race on ARQ/AX.25 session state (tick vs frame dispatch).

#### Real-Time Allocation (H7)
`vector<float> audio(rx_audio, rx_audio + frame_count)` on every RX callback. Mono downmix allocates per-callback. OFDM buffer insert can trigger reallocation. IRIS_LOG mutex + fflush blocks audio thread (M11). Playback FIFO O(n) erase (M12).

#### Resource Cleanup
No exception safety in process_rx/process_tx. COM objects use raw `Release()` (no RAII). Device disconnect causes silent busy-loop. Capture FIFO overflow drops new data (should drop oldest).

---

### Platform Audio

#### WASAPI Behavior (C2, H5, H6)
Shared mode only (appropriate for modem). 5ms capture poll, 2ms render poll. Two-thread FIFO architecture. No sample rate validation (C2). No format validation (H5). No disconnect detection (H6). Buffer size 100ms capture, 50ms playback (hardcoded). VB-Cable adds 5-10ms vs real soundcard — handled by FIFO cushion.

#### Audio Degradation
Capture FIFO overflow drops new data (should drop oldest for freshness). Playback underrun zero-fills (correct). DATA_DISCONTINUITY logged (first 10 only). No AUDCLNT_BUFFERFLAGS_SILENT detection. No input clipping warning. No stall watchdog.
