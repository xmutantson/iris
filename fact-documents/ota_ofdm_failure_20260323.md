# OTA OFDM Failure — 2026-03-23 Test (KG7VSN ↔ W7WEK)

## §1 Test Configuration
- Build: `0b3578ac` on both sides
- NFFT=512 negotiated (wire code 0) — both sides advertise code 0
- 32 used carriers, 8 pilots (spacing=4), pilot_row_spacing=8
- CP=64 (5.3ms guard), symbol=576 samples (12ms), 67 data symbols/frame
- BPSK r=1/2, DFT-spread, PAPR clip 7 dB target
- FM channel (NBFM radios, audio path)
- Initiator: KG7VSN (local, `%APPDATA%\Iris\logs\20260323_102103.log`)
- Responder: W7WEK (peer, `~/Downloads/20260323_102103.log`)

## §2 Session Timeline

| Time | Initiator (KG7VSN) | Responder (W7WEK) |
|------|--------------------|--------------------|
| ~20s | AX.25 connected, probe, native active | AX.25 connected, probe, native active |
| 49s | TX 5 TUNE ramp frames (scales 14.67→2.20) | — |
| 50-57s | — | RX 7 TUNE frames, ALL LDPC fail |
| 51.7s | — | **T1 fires** (fix confirmed: line 232) |
| 55-62s | RX peer TUNE frames 1-6, ALL LDPC fail | T1 re-polls (2/10, 3/10...) |
| **62s** | **Self-hear: decodes own TX as ~134 "frames"** | — |
| 62-65s | TUNE counts 134 frames (all self-hear) | TX 5 ramp frames (T=61.8s) |
| 67-84s | — | T1 re-polls (5/10 through 10/10) |
| 74s | — | Detects frames but CFO=39.5 Hz > 35 Hz limit → rejected |
| **84.6s** | — | **T1 expired, 10 retries → DISCONNECTED** ✓ |
| 101s | TUNE timeout (state=3, 134 frames) | — |
| 101s | Applies self-hear gain: tx_level 0.170→0.750 | 104s: TUNE timeout, applies local gains |
| 103-153s | TX OFDM data frames, all RX LDPC fail | 108s: Restores AFSK, sends AX.25 frames |

## §3 Bug 1: Initiator Self-Hear (CRITICAL)

Starting at T=62s, initiator decodes **134 bit-identical frames** from its own TX buffer:
- All share: d=1510, SC=0.950, FD-ZC=0.752, CFO=-3.74 Hz
- mean|H|=16.837 (legitimate OTA frames had H=5-10)
- SNR=15.7 dB (OTA was 0-6 dB)
- LDPC converges in 5/50 iters, every frame "OK"
- Kalman stable: lambda=1.00, max_lambda=2.63, phase=1219 deg total (18 deg/sym — manageable)
- Same embedded report (5 entries) extracted from every frame

**Evidence it's self-hear, not real signal:**
1. 134 identical frames in ~1 second (62.563-63.595s) — impossible OTA timing
2. H=16.8 vs OTA H=5-10: 10 dB stronger than any real frame
3. Perfect consistency across all 134 frames (same pilot measurements to 0.1 deg)
4. Self-hear guard catches later instances at T=103s: "frame detected but DISCARDED (self-hear guard, clearing N samples)"
5. Sync word CRC still fails (got 0xE8, expected 0xAE) despite LDPC success — sync word was NOT the real TX's sync word [?]

**Impact:** TUNE uses these 134 fake "successful" frames to set tx_level=0.750, which is almost certainly wrong. The responder gets tx_level from its own (failed) measurements.

**Root cause hypothesis:** The initiator's TX audio is being looped back through the sound card or radio's sidetone path. The self-hear guard expects PTT to be active during TX, but the TUNE ramp ends before these 134 frames are processed. The guard only blocks frames detected while `ptt=1`, and by T=62s the PTT is off.

**[?]** Need to verify: does the self-hear guard use PTT state or a time window? The guard at T=103s works ("self-hear guard"), but at T=62s it doesn't — what changed?

## §4 Bug 2: ALL OTA OFDM Frames Fail

Every legitimate OTA frame (both directions) shows:

**Phase drift statistics (from Kalman summaries):**

| Frame | Direction | End phase (deg) | Freq (deg/sym) | max_lambda | gated |
|-------|-----------|----------------|----------------|------------|-------|
| TUNE 1 | W7WEK→KG7VSN | -6357 | 29 | 3.00 | 16 |
| TUNE 2 | W7WEK→KG7VSN | -11065 | -316 | 3.00 | 6 |
| TUNE 3 | W7WEK→KG7VSN | -1057 | 28 | 3.00 | 7 |
| TUNE 4 | W7WEK→KG7VSN | 5818 | 261 | 3.00 | 6 |
| TUNE 5 | W7WEK→KG7VSN | 7743 | 169 | 3.00 | 8 |
| TUNE 6 | W7WEK→KG7VSN | 5187 | 84 | 3.00 | 6 |
| TUNE 1 | KG7VSN→W7WEK | -3270 | -250 | 3.00 | 7 |
| TUNE 2 | KG7VSN→W7WEK | -2427 | -88 | 3.00 | 4 |
| TUNE 3 | KG7VSN→W7WEK | 765 | 22 | 2.82 | 2 |
| TUNE 5 | KG7VSN→W7WEK | 614 | 8 | 2.82 | 8 |

- End phase ranges from -11000 to +7700 degrees over 67 symbols
- Frequency drift 8 to 316 deg/sym (average ~150 deg/sym)
- max_lambda always 2.82-3.00 (STF maxed, Kalman can't keep up)
- Innovation gating on 2-16 measurements per frame

**Other symptoms:**
- Sync word CRC mismatch on EVERY frame
- SNR 0.0-5.7 dB (need ~5 dB for BPSK r=1/2)
- CFO varies: -27 Hz, -1 Hz, +2 Hz, +5 Hz, -21 Hz, -24 Hz between consecutive frames from same TX
  - Real FM offset should be approximately stable
  - This suggests most detections are false sync triggers on noise
- Some frames show FD-ZC right at threshold (0.30-0.38), others below

**Frame 3 (KG7VSN→W7WEK) is interesting:** phase=765 deg, freq=22 deg/sym, max_lambda=2.82, only 2 gated. This is the BEST frame and it still failed LDPC at SNR=3.1 dB. 22 deg/sym ≈ 3 Hz residual CFO — plausible for FM. But 765 deg over 67 symbols ≈ 11.4 deg/sym average — within what pilots + Kalman should handle for BPSK. Yet sync word CRC still failed and LDPC hit 50 iters. [?] This suggests the problem may not be phase alone — SNR is simply too low.

**Frame 5 (KG7VSN→W7WEK) also notable:** freq=8 deg/sym, max_lambda=2.82 — but SNR=0.6 dB, LDPC fails. The phase tracking is actually working here but the signal is buried in noise.

## §5 Root Cause Analysis

### Primary cause: Almost all "OTA frames" are false sync triggers on noise

Evidence from detection metrics across both logs:

**Initiator (KG7VSN) detections of W7WEK frames:**
| Time | SC | FD-ZC | CFO (Hz) | SNR (dB) | Verdict |
|------|-----|-------|----------|----------|---------|
| 55.3s | 0.502 | 0.309 | -21.5 | 0.0 | **False** — FD-ZC barely above 0.30, SNR=0 |
| 56.5s | 0.680 | 0.377 | -26.7 | 3.3 | **Likely false** — CFO jumped 5 Hz |
| 57.5s | 0.530 | 0.520 | -21.0 | 0.5 | **Possible** — FD-ZC decent but SNR=0.5 |
| 58.6s | 0.605 | 0.526 | -18.3 | 1.9 | **Possible** — FD-ZC decent |
| 59.6s | 0.574 | 0.319 | +4.6 | 1.3 | **False** — CFO flipped sign (+4.6 vs -21) |
| 60.6s | 0.508 | 0.329 | -17.0 | 0.1 | **False** — SC/FD-ZC/SNR all marginal |

**Responder (W7WEK) detections of KG7VSN frames:**
| Time | SC | FD-ZC | CFO (Hz) | SNR (dB) | Verdict |
|------|-----|-------|----------|----------|---------|
| 50.2s | 0.788 | 0.151 | -23.5 | 5.7 | **Best candidate** — SC good but FD-ZC=0.151! |
| 51.2s | 0.525 | 0.197 | -23.6 | 0.4 | **False** — FD-ZC=0.197 |
| 52.3s | 0.670 | 0.314 | -1.1 | 3.1 | **False** — CFO jumped from -23.5 to -1.1 |
| 53.4s | 0.507 | 0.395 | +2.0 | 0.1 | **False** — CFO sign flip |
| 54.3s | 0.535 | 0.346 | -1.8 | 0.6 | **False** — inconsistent CFO |

**Self-hear detections (for comparison):**
| Time | SC | FD-ZC | CFO (Hz) | SNR (dB) | Verdict |
|------|-----|-------|----------|----------|---------|
| 61.6s | 0.950 | 0.752 | -3.7 | 12.8 | **Real** (self-hear) |
| 62.6s | 0.950 | 0.752 | -3.7 | 12.8 | **Same frame** (re-scan bug) |

**Key observations:**
1. A **real** ZC detection has SC≈0.95, FD-ZC≈0.75. OTA "detections" have SC=0.5-0.7, FD-ZC=0.15-0.53
2. **CFO is wildly inconsistent**: -27 Hz, -1 Hz, +2 Hz, +5 Hz between "frames" from the **same TX**. Real FM offset is approximately constant. This confirms noise triggers.
3. **FD-ZC threshold lowered to 0.15 during TUNE** (modem.cc:1311), which lets through detections that would be rejected in normal operation (threshold 0.30). The responder's "best" frame at FD-ZC=0.151 is barely above the TUNE threshold.
4. **Sync word CRC fails on 100% of OTA frames** — if even one were a real frame, the 8-bit CRC has 1/256 chance of passing, and it never does.
5. The **316 deg/sym "phase drift"** is meaningless — it's the Kalman tracking noise after a false sync placed the frame window at the wrong position. There is no real signal to track.

### ~~Phase drift from FM discriminator~~ [Corrected]
The extreme phase numbers (up to 316 deg/sym) are NOT evidence of FM channel phase distortion. They result from false sync triggers on noise, where the Kalman has no real signal to lock onto. The one plausible real frame (responder T=50.2s, SC=0.788, SNR=5.7 dB) showed -250 deg/sym — but even this had FD-ZC=0.151, suggesting a marginal detection at best.

### Root cause chain:
1. **NFFT=512 loaded from stale INI** → 32 carriers, ZC preamble length 32 → ZC correlation gain only ~15 dB
2. **Signal too weak for 32-carrier ZC** → real preambles scored FD-ZC=0.15-0.53 (real detection should be >0.7)
3. **TUNE mode lowers FD-ZC threshold to 0.15** → noise triggers pass the gate
4. **Sync word CRC is 8-bit** → with 10 false triggers per session, 10/256 ≈ 4% chance of one passing (none did)
5. **Kalman tracks noise** → accumulates thousands of degrees → reported as "phase drift"

### Remaining question: Was the signal actually present?
The responder's first detection (SC=0.788, FD-ZC=0.151, SNR=5.7 dB, CFO=-23.5 Hz) is the most likely real frame. SC=0.788 is decent but FD-ZC=0.151 is terrible. The ZC correlation gain with only 32 carriers may not be enough to distinguish the signal from noise at 5.7 dB SNR. **With NFFT=1024 (~42 carriers), ZC length increases and correlation gain improves by ~1.2 dB.** This may or may not be enough — the next OTA test will tell.

4. **Self-hear corrupts TUNE**: Initiator's 134 self-hear frames produce avg_H=11.88 and set tx_level=0.750 — wildly incorrect for the actual channel.

### [?] Open questions:
1. Does current codebase default to NFFT=1024? Build 0b3578ac may predate that change. If so, this entire test is on the wrong NFFT.
2. Would NFFT=1024 help? More carriers = more pilots, but 2x longer symbols = more susceptible to phase drift. Net effect unclear.
3. Is the FM audio path simply too noisy/distorted for wideband OFDM? VARA FM works through the same path but uses single-carrier.
4. What is the initiator's TX audio path that causes self-hear? VB-Cable? Radio sidetone? Sound card loopback?

## §6 T1 Timer Fix — CONFIRMED WORKING

Responder log proves the fix:
- Line 64 (T=20s): `AX25 T1 started on native activation: 40 ticks (2.0s) [V(A)=0 V(S)=3]`
- Line 232 (T=51.7s): `AX25 T1 timeout -> TIMER_RECOVERY, poll (1/10) [native]`
- Lines 364-587: T1 re-polls 2/10 through 10/10
- Line 590 (T=84.6s): `AX25 T1 expired, max retries (10) exceeded -> DISCONNECTED`
- Line 594: `Restoring original PHY: band 1200-2200 Hz, baud 800`

The session terminated cleanly. Winlink would not have hung.

## §7 Action Items

1. ~~**Fix self-hear during TUNE**~~ — **FIXED.** Root cause: TUNE frame discard paths (`modem.cc:1664-1676`) cleared `rx_overlap_buf_` but did not drain `ofdm_rx_audio_buf_`, so the same preamble was re-detected and decoded 134 times. Fix: drain `ofdm_rx_audio_buf_` using the same `sync.frame_start + result.samples_consumed` pattern as normal frame processing (`modem.cc:1785-1792`). Also fixed same bug in OFDM-KISS decompression failure path.
2. ~~**Verify NFFT setting**~~ — **FIXED.** Build 0b3578ac loaded NFFT=512 from stale INI (`iris.ini` persisted by older build). Code default is 1024 but `load_config()` overrode it. Fix: removed NFFT and 4 other internal OFDM params (auto_spacing, waterfill, nuc, preemph_corner_hz) from INI load/save entirely. Only `ofdm_enable` remains user-configurable. Also removed OFDM developer controls from GUI.
3. **Research FM OFDM feasibility** — use Opus agent to find how DAB, DRM, or other FM OFDM systems handle the pre-emphasis/de-emphasis + discriminator phase noise problem.
4. ~~Investigate why initiator never fires T1~~ — Initiator is the AX.25 commander and sends data. T1 fires on the responder (which has outstanding I-frames to ack). The initiator doesn't have unacked I-frames in this test because it never successfully sent OFDM data. [Corrected: T1 is started by start_t1_if_unacked(), which checks V(A)!=V(S). Initiator's V(A)=V(S) because its I-frames were delivered via AFSK before probe.]
