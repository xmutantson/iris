# IRIS OFDM PHY Specification

**Revision 2** -- updated to match implementation as of 2026-03-20.

---

## 1. Overview

OFDM PHY for IRIS FM modem, operating alongside the existing single-carrier native PHY.
Goal: beat VARA FM throughput at equivalent SNR by using multicarrier modulation with
per-subcarrier adaptive bit loading (waterfilling) and near-Shannon LDPC FEC.

### Design Principles

1. **Probe-adaptive:** Bandwidth, carrier count, and per-carrier modulation all derive
   from the passband probe. Works on any radio from 2 kHz mic/speaker to 6+ kHz data port.
2. **Waterfilling:** Per-subcarrier bit loading based on measured channel SNR.
3. **Reuse existing infrastructure:** LDPC codec, ARQ/HARQ, probe system, audio I/O,
   upconverter/downconverter, AGW/KISS interfaces.
4. **Coexist with single-carrier:** `CAP_OFDM` capability flag. Either side can disable
   OFDM and fall back to A-levels. Configurable via INI/GUI toggle `OfdmEnable`.
5. **DFT-spread OFDM (SC-FDMA):** Low PAPR critical for FM deviation limiters.
6. **Headerless frames:** Config pre-negotiated via tone map exchange (Mercury approach).

---

## 2. OFDM Parameters

### 2.1 FFT Size and Subcarrier Spacing

Sample rate: 48000 Hz (fixed, `SAMPLE_RATE` in `types.h`).

| NFFT | Subcarrier Spacing | Symbol Duration | Use Case |
|------|-------------------|-----------------|----------|
| 256  | 187.50 Hz | 5.33 ms | Noisy/drifty radios, fallback |
| 512  | 93.75 Hz  | 10.67 ms | **Default** -- good balance |
| 1024 | 46.875 Hz | 21.33 ms | Wide BW (>4 kHz), more carriers |

**Default: NFFT=512, spacing=93.75 Hz.**

### 2.2 Cyclic Prefix

```
cp_samples = 32    (at 48 kHz = 0.67 ms)
```

FM audio channel delay spread < 0.5 ms. Overhead: 32/512 = 6.25%.

Total OFDM symbol: 512 + 32 = 544 samples = 11.33 ms.

### 2.3 Carrier Allocation

Given probed bandwidth `BW_hz` and center frequency `center_hz`:

```
n_total = floor(BW_hz / subcarrier_spacing)
n_guard = 2                    // 1 guard each edge
n_dc = 0                      // no DC carrier (audio passband, not baseband)
n_used = n_total - n_guard
n_pilot = ceil(n_used / pilot_carrier_spacing)
n_data = n_used - n_pilot
```

At NFFT=512, 2 kHz BW: `n_used=19, n_pilot=4, n_data=15`.

Carrier index mapping: carriers placed symmetrically around `center_hz / spacing`
in the FFT bin array. Unused bins set to zero. Hermitian symmetry enforced for
real-valued IFFT output (audio is real).

### 2.4 Pilot Pattern

Three-dimensional pilot grid:

| Dimension | Parameter | Value | Description |
|-----------|-----------|-------|-------------|
| Frequency | `pilot_carrier_spacing` | 6 | Every 6th used carrier is a comb pilot |
| Time (block) | `pilot_symbol_spacing` | 24 | Every 24th OFDM symbol is all-pilot |
| Time (dense) | `pilot_row_spacing` | 5 | Every 5th data symbol is a dense pilot row |

**Comb pilots:** Every 6th used subcarrier carries a known +1 reference (scaled by
de-emphasis gain). Used for per-symbol CPE correction and frequency interpolation.

**Block pilot symbols:** All carriers = known reference. Channel tracking IIR update
(alpha=0.3).

**Dense pilot rows:** Every 5th data symbol, all carriers carry known reference values.
Third pilot dimension beyond comb and block pilots.

---

## 3. Frame Structure

### 3.1 OFDM Frame Layout

```
[Preamble: 2 ZC symbols] [Data: N OFDM symbols] [Tail: 1]
```

No per-frame header. Config (tone map, FEC rate, modulation) is pre-negotiated via
tone map exchange before data transfer begins (Mercury approach).

**Training Symbols 1 & 2 (Zadoff-Chu):**
- Two identical ZC sequences (root=7, length = n_used_carriers).
- ZC placed into used carrier FFT bins with Hermitian symmetry for real-valued output.
- Scaled by NFFT to match data symbol amplitude, plus PREAMBLE_BOOST for detection margin.
- Detection: hybrid Schmidl-Cox autocorrelation (between identical ZC symbols) for
  coarse detection, then ZC cross-correlation for fine timing refinement.
- Channel estimation: divide received ZC by known ZC frequency-domain sequence.
- CFO: phase rotation between the two identical training symbols.

**Data Symbols:**
- Per-subcarrier modulation defined by tone map.
- Dense pilot rows every `pilot_row_spacing` data symbols.
- Block pilot symbols every `pilot_symbol_spacing` OFDM symbols.
- Multi-codeword: `n_codewords > 1` per frame reduces preamble overhead.

**Tail Symbol:**
- All subcarriers carry known +1 reference.
- Anchors backward channel interpolation and frame-end marker.

### 3.2 Frame Timing

At NFFT=512, CP=32, 2 kHz BW (15 data carriers), 1 codeword:

| Component | OFDM Symbols | Duration |
|-----------|-------------|----------|
| Preamble (2x ZC) | 2 | 22.7 ms |
| Data (1 LDPC block, QPSK r1/2) | ~54 | 612 ms |
| Dense pilot rows (every 5th) | ~10 | 113 ms |
| Tail | 1 | 11.3 ms |
| **Total** | **~67** | **~759 ms** |

Multi-codeword frames (n_codewords=8 typical): single preamble + tail amortized over
8 LDPC blocks.

---

## 4. Modulator

### 4.1 TX Signal Processing Chain

```
Payload bytes
  -> CRC-32 append
  -> LDPC encode (rate from tone map)
  -> Bit interleave (stride-41 congruential per 1600-bit block)
  -> BICM interleave (for QAM16+, column-row per block)
  -> Scramble (LFSR x^15+x^14+1, seed=0x6959)
  -> Frequency-time interleave (global stride-173 across all coded bits)
  -> Per-subcarrier symbol mapping (constellation from tone map, NUC optional)
  -> DFT-spread (N-point DFT across data carriers per OFDM symbol)
  -> FM TX de-emphasis (per-subcarrier amplitude shaping)
  -> Insert comb pilot subcarriers (every 6th, +1 * de-emphasis gain)
  -> Insert dense pilot rows (every 5th data symbol)
  -> Insert block pilot symbols (every 24th OFDM symbol)
  -> Prepend 2x ZC training symbols (root=7)
  -> Append tail symbol
  -> IFFT per OFDM symbol (NFFT-point, Hermitian symmetry)
  -> Prepend cyclic prefix (cp_samples=32)
  -> Concatenate all symbols into time-domain baseband
  -> Hilbert PAPR clipper (iterative clip-filter-restore, target ~7 dB PAPR)
  -> Upconvert to center_hz
  -> Output audio samples
```

### 4.2 DFT-Spread OFDM (SC-FDMA)

DFT pre-coding of data carriers before IFFT. Each OFDM symbol's data carriers are
passed through an N-point DFT before insertion into the OFDM frequency grid. This
spreads each QAM symbol across all data subcarriers, producing a single-carrier-like
time-domain signal.

```
PAPR reduction: ~10-11 dB (plain OFDM) -> ~5-7 dB (DFT-spread)
```

Critical for FM deviation limiters, which behave like power amplifier saturators.
Enabled by default (`dft_spread=true` in `OfdmConfig`).

Constraints:
- All data carriers in a symbol must use the same modulation order.
- Disabled for DD-CPE when `max_bpc < 6`.
- DFT-spread must happen BEFORE de-emphasis (de-emphasis is per-carrier and
  undone by the equalizer via H[k]; applying it before DFT would bake scaling
  into the spread signal).

RX applies IDFT after MMSE equalization to recover QAM symbols.

**EVM-based noise variance:** With DFT-spread enabled, the standard ZC-based noise
variance estimate may be inaccurate (DFT-spread changes the per-carrier statistics).
The demodulator computes EVM (Error Vector Magnitude) from hard-sliced DFT-despread
symbols and uses this as noise variance for soft demapping when DFT-spread is active.

### 4.3 FM TX De-Emphasis Compensation

Per-subcarrier amplitude shaping to compensate radio pre-emphasis. Attenuates higher
carriers on TX so that after the radio's pre-emphasis the signal is flat entering the
deviation limiter.

```
fm_preemph_corner_hz = 300 Hz     (NBFM mic/speaker, TIA/EIA-603 530us)
fm_preemph_gain_cap  = 10.0       (max de-emphasis ratio, -20 dB floor)
```

Gain per carrier:
```
g[k] = 1 / sqrt(1 + (f[k] / corner_hz)^2)
g[k] = max(g[k], 1 / gain_cap)
```

Set `fm_preemph_corner_hz = 0` to disable (flat audio data port connection).
Applied to training symbols, comb pilots, and data carriers consistently so that
H[k] from channel estimation accounts for the shaping.

### 4.4 PAPR Reduction (Hilbert Clipper)

Iterative clip-and-filter PAPR reduction targeting ~7 dB PAPR:

1. Measure PAPR of time-domain frame.
2. Hard clip samples exceeding target amplitude.
3. Bandpass filter (passband only) to remove out-of-band spectral regrowth.
4. Restore pilot subcarriers to known values (clipping corrupts them).
5. Repeat steps 1-4 for 4 iterations until convergence.

This replaces simple `3.0x RMS` soft clipping. The iterative approach achieves
lower PAPR while keeping spectral regrowth within the passband. Pilot restoration
ensures channel estimation accuracy is not degraded by clipping.

Implementation: `ofdm_papr.h` (Hilbert clipper class), called from `ofdm_mod.cc`.

### 4.5 Preamble Boost

Training symbols scaled by `PREAMBLE_BOOST = sqrt(2) ≈ 1.414` (+3 dB) relative
to data symbols. Improves detection reliability at low SNR without affecting data
symbol power budget.

RX compensates by dividing received training symbol by `PREAMBLE_BOOST` before
channel estimation, so H[k] reflects the data-symbol-level channel gain.

---

## 5. Demodulator

### 5.1 RX Signal Processing Chain

```
Audio samples
  -> Downconvert from center_hz
  -> Baseband IQ stream
  -> Frame detection (hybrid Schmidl-Cox + ZC correlation)
  -> Coarse CFO estimation (phase between two ZC training symbols)
  -> CFO correction (rotate entire frame)
  -> Strip CP, FFT each OFDM symbol
  -> ZC-based channel estimation: H[k] = Y_zc[k] / X_zc[k]
  -> Noise variance estimation (ZC residuals)
  -> Mean |H| quality gate (skip decode if mean|H| < 0.05)
  -> For each data OFDM symbol:
      -> Per-symbol CPE correction from comb pilots
      -> Extract data carriers (skip pilot positions)
      -> MMSE equalize: d[k] = Y[k] * conj(H[k]) / (|H[k]|^2 + nv[k])
      -> DFT-despread (IDFT if dft_spread enabled)
      -> DD-CPE refinement (BPSK/QPSK/16QAM only)
  -> Collect equalized data symbols
  -> Soft demap per subcarrier (LLR computation with per-carrier sigma^2)
  -> LLR clamp +/-8
  -> Soft descramble (LFSR sign-flip)
  -> Frequency-time de-interleave (reverse stride-173)
  -> BICM de-interleave (QAM16+)
  -> LDPC block de-interleave (reverse stride-41)
  -> Tail bit hardening
  -> Chase combining (if stored LLRs available from HARQ)
  -> LDPC decode (min-sum, 50 iterations)
  -> CRC-32 check
  -> Payload bytes
```

### 5.2 Frame Detection (Hybrid Schmidl-Cox + ZC)

Two identical Zadoff-Chu training symbols replace the original Schmidl-Cox scheme.
Detection uses a two-phase approach:

**Phase 1 -- Schmidl-Cox Autocorrelation (coarse detection):**

Correlates body of train1 with body of train2 (identical ZC sequences). Both pass
through the same channel, so distortion cancels. Channel-invariant.

```
P(d) = sum_{m=0}^{N-1} conj(r(d+cp+m)) * r(d+cp+sym_len+m)
A(d) = sum_{m=0}^{N-1} |r(d+cp+m)|^2
R(d) = sum_{m=0}^{N-1} |r(d+cp+sym_len+m)|^2

M(d) = |P(d)|^2 / (A(d) * R(d))     [Cauchy-Schwarz normalized, bounded 0..1]
W(d) = M(d) * (A(d) + R(d))          [energy-weighted, rejects silence]
```

Coarse search at stride=cp, fine search at stride=1 around peak.
Detection threshold: M >= 0.40.

CFO from SC: `cfo_hz = angle(P(d_peak)) * Fs / (2*pi*N)`

**Phase 2 -- ZC Cross-Correlation (fine timing):**

After SC detection, ZC cross-correlation with known time-domain ZC sequence
pinpoints exact frame start within +/-cp. Frequency-domain cross-correlation
provides ZC quality gate (reject false positives where FD-ZC < 0.30).

### 5.3 Channel Estimation (ZC-Based)

From training symbol (ZC known sequence in frequency domain):

```
H[k] = Y_zc[k] / X_zc[k]
```

where `X_zc[k]` is the known ZC frequency-domain sequence at each used carrier.

Noise variance per subcarrier from ZC residuals (3-tap smoothed H deviation).

Channel estimate updated at block-pilot symbols via IIR (alpha=0.3):
```
H[k] = alpha * H_new[k] + (1-alpha) * H_old[k]
```

Comb pilots provide frequency interpolation with linear interp of H and noise_var.

### 5.4 Per-Symbol CPE Correction (DD-CPE)

Decision-directed common phase error correction per OFDM symbol.

**Pilot-based CPE:** Weighted average of comb pilot phase errors per symbol.
Applied as a common rotation to all carriers:
```
cpe = sum(conj(H_pilot) * Y_pilot) / sum(|H_pilot|^2)
correction: Y[k] *= exp(-j * angle(cpe))
```

**DD refinement:** After equalization and DFT-despread, hard-slice each data carrier
and compute phase error vs nearest constellation point. Weighted average across all
carriers refines the pilot-only CPE estimate.

**Disabled for:**
- 64QAM+ (`max_bpc >= 6`): FM deviation limiter compresses outer constellation points,
  causing systematic hard-decision errors that bias the estimate.
- DFT-spread mode: DFT-spread carriers are in frequency domain before IDFT, DD-CPE
  operates on time-domain symbols.

**CPE slope feedback:** History of per-symbol CPE values estimates residual CFO
(reported as `cpe_drift_hz` in diagnostics).

### 5.5 SFO Tracking

Sampling Frequency Offset (SFO) causes progressive timing drift across the frame.
Block pilot symbols track this drift:

1. At each block pilot, compute mean phase difference from initial channel estimate.
2. Fit linear slope to (symbol_index, phase_drift) across all block pilots seen so far.
3. Apply per-symbol phase correction: `correction[k] = exp(-j * slope * symbol_index * k/N)`.

Dense pilot rows (every 5th data symbol) provide additional phase snapshots for
SFO estimation, improving accuracy for long frames.

### 5.6 MMSE Equalization

Per subcarrier, per OFDM symbol:
```
d[k] = Y[k] * conj(H[k]) / (|H[k]|^2 + nv[k])
```

**Noise variance floors** prevent infinite gain:
```
NV_ABS_FLOOR = 0.001       (-30 dB absolute)
NV_REL_FLOOR = 0.002       (max 27 dB effective SNR per carrier)
nv[k] = max(nv[k], max(NV_ABS_FLOOR, NV_REL_FLOOR * |H[k]|^2))
```

### 5.7 Quality Gates

**Mean |H| gate:** If `mean|H| < 0.05` across all used carriers, skip LDPC decode.
Avoids wasting cycles on noise-only detections.

**ZC quality gate:** In sync, frequency-domain ZC correlation metric must exceed 0.30
to confirm a legitimate frame (vs data-symbol false positive).

### 5.8 LLR Clamp

LLRs clamped to +/-8 (not +/-20). DVB-T2 research shows +/-6 to +/-10 is optimal
for LDPC at low SNR. Previous +/-20 provided no protection against over-confident
LLRs from underestimated noise variance.

### 5.9 CSI Handling

CSI (Channel State Information) reliability weighting is **removed** in the OFDM path.
CSI is an anti-pattern with MMSE equalization -- MMSE already incorporates per-carrier
SNR into the equalizer gain. Native single-carrier path still uses CSI.

---

## 6. Waterfilling / Tone Map

### 6.1 Bit Loading Algorithm

Chow-Cioffi-Bingham (CCB) bit loading:

```
bits[k] = floor(log2(1 + snr[k] / Gamma))
```

SNR gap `Gamma` per FEC rate:
- Rate 1/2: 2.0 dB (1.58 linear)
- Rate 5/8: 2.5 dB (1.78 linear)
- Rate 3/4: 3.5 dB (2.24 linear)
- Rate 7/8: 5.0 dB (3.16 linear)

Clamp to valid set: `{0, 1, 2, 4, 6, 8}`. If `bits[k] < 1`, null the carrier.

### 6.2 Tone Map Structure

```cpp
struct ToneMap {
    std::vector<uint8_t> bits_per_carrier;  // one per data carrier: 0,1,2,4,6,8
    int n_data_carriers;
    int total_bits_per_symbol;              // sum of bits_per_carrier
    LdpcRate fec_rate;
    int nfft;
    uint8_t tone_map_id;                    // 0=waterfill, 1-8=uniform presets
    bool use_nuc;                           // Non-Uniform Constellations
    int n_codewords;                        // LDPC blocks per frame (multi-codeword)
};
```

### 6.3 Tone Map Negotiation

Both sides compute tone maps from exchanged probe data (deterministic: same inputs
produce same output). Tone map serialization for peer exchange:

```
Header (1 byte): [4-bit tone_map_id][4-bit fec_rate_field]
For waterfill (id=0): one nibble per carrier (bits/2), two per byte.
Total: 1 + ceil(n_data_carriers/2) bytes.
```

### 6.4 Uniform Tone Map Presets

| Preset ID | Modulation | FEC Rate | O-Level |
|-----------|-----------|----------|---------|
| 1 | BPSK | 1/2 | O0 |
| 2 | QPSK | 1/2 | O1 |
| 3 | QPSK | 3/4 | O2 |
| 4 | 16QAM | 1/2 | O3 |
| 5 | 16QAM | 3/4 | O4 |
| 6 | 64QAM | 3/4 | O5 |
| 7 | 64QAM | 7/8 | O6 |
| 8 | 256QAM | 7/8 | O7 |

---

## 7. HARQ-IR with Rate-Compatible LDPC

### 7.1 Mother Code

Rate 4/16 (K=400, P=1200, N=1600) as the mother code. Six LDPC rate matrices available:

| Rate | K | P | N | Matrix |
|------|---|------|------|--------|
| 4/16 | 400 | 1200 | 1600 | `mercury_normal_4_16` |
| 6/16 | 600 | 1000 | 1600 | `mercury_normal_6_16` |
| 8/16 (1/2) | 800 | 800 | 1600 | `mercury_normal_8_16` |
| 10/16 (5/8) | 1000 | 600 | 1600 | `mercury_normal_10_16` |
| 12/16 (3/4) | 1200 | 400 | 1600 | `mercury_normal_12_16` |
| 14/16 (7/8) | 1400 | 200 | 1600 | `mercury_normal_14_16` |

### 7.2 Chase Combining

Current implementation uses rate 1/2 Chase combining (existing HARQ infrastructure).
On NACK, retransmit same codeword; receiver LLR-combines with previous attempt.
True incremental redundancy (4/16 mother code puncturing) available as future enhancement.

### 7.3 Integration

The OFDM demodulator produces the same outputs as the native demodulator:
equalized symbols, per-symbol reliability, LLRs, SNR estimate. These feed into
the shared post-equalization pipeline: demap, weighting, de-interleave, LDPC decode.

OFDM `sym_phase_var` maps to per-subcarrier reliability instead of Kalman P00,
but the HARQ region selection logic works identically.

---

## 8. Non-Uniform Constellations (NUC)

ATSC 3.0 optimized constellation points. One lookup table per (order, fec_rate) pair:

- 16-NUC: 2D coordinates, per FEC rate
- 64-NUC: 2D coordinates, per FEC rate
- 256-NUC: 1D (separable I/Q) coordinates, per FEC rate

Gains: 0.2-0.5 dB at 16QAM, 0.5-0.8 dB at 64QAM, 0.8-1.0 dB at 256QAM.

Mapping: `map_symbol_nuc()` and `demap_soft_nuc()` in `constellation.cc`.
For 2D-NUC (16/64QAM): full 2D search over all constellation points.
For 1D-NUC (256QAM): decompose into I/Q axes (same complexity as uniform).

---

## 9. Speed Levels (O-Levels)

Eight O-levels, each a specific modulation + FEC rate pair. Maps 1:1 to uniform
tone map presets (preset_id = level + 1).

| Level | Modulation | FEC Rate | Min SNR | Description |
|-------|-----------|----------|---------|-------------|
| O0 | BPSK | 1/2 | 0 dB | Most robust |
| O1 | QPSK | 1/2 | 6 dB | |
| O2 | QPSK | 3/4 | 8 dB | |
| O3 | 16QAM | 1/2 | 12 dB | |
| O4 | 16QAM | 3/4 | 16 dB | |
| O5 | 64QAM | 3/4 | 20 dB | |
| O6 | 64QAM | 7/8 | 24 dB | |
| O7 | 256QAM | 7/8 | 28 dB | Data port only |

SNR thresholds raised for FM (deviation limiter distortion is not AWGN).
Gearshift adds 1 dB margin on top of these values (picks highest O-level where
SNR exceeds threshold + 1 dB).

O0-O4 validated near theoretical limits (0-13 dB SNR). O5-O7 (64QAM+) broken
on FM mic/speaker path (fundamental limitation of FM deviation limiting on dense
constellations). Data port only.

---

## 10. OFDM-KISS Transport

AX.25 frames tunneled over native OFDM PHY. The AX.25 session layer stays up
during the upgrade from AFSK to native OFDM.

### 10.1 Activation

1. AX.25 connected-mode session established over AFSK 1200.
2. After probe completes and both sides have `CAP_OFDM`, OFDM-KISS RX activates.
3. Split RX/TX activation: RX enables immediately; TX enables after hearing a
   native frame from peer (bidirectional confirmation).
4. Probe coordination: cooldown before initiator starts probe to avoid collision.

### 10.2 Transport Features

- **Compression:** PPMd/zstd batch compression of OFDM-KISS payloads. Marker byte
  `0xCC` (`COMPRESSED_PAYLOAD_MAGIC`).
- **B2F proxy:** Winlink LZHUF unroll/reroll proxied through OFDM transport. Marker
  byte `0xCD` (`B2F_DATA_MAGIC`). B2F AFSK history replayed at activation for SID/FC/FS
  context.
- **Adaptive batch airtime:** TCP-style AIMD. Grows on successful ACKs (+1s per RR),
  halves on REJ/loss.

### 10.3 AX.25 Windowed Flow Control

OFDM-KISS maintains AX.25 sequence numbers (V(R), N(R)) for generating local RR ACKs
and I-frame construction. Cached AX.25 address header for frame assembly.

---

## 11. TUNE Calibration Protocol

Bilateral native-frame gain calibration. Each side sends test frames; the other
measures channel gain from the preamble and reports back. Both adjust TX level so
the peer receives at optimal gain.

### 11.1 Protocol Flow

```
Initiator: SEND_START -> TX_TEST -> WAIT_PEER -> SEND_REPORT -> WAIT_REPORT -> APPLY -> DONE
Responder:              WAIT_PEER -> SEND_REPORT_AND_TEST -> WAIT_REPORT -> APPLY -> DONE
```

### 11.2 Power Ramp

10-frame power ramp at geometrically-spaced TX levels:

```
TUNE_RAMP_COUNT = 10
TUNE_SCALE_MIN  = 0.05   (minimum absolute tx_level)
TUNE_SCALE_MAX  = 1.0    (maximum absolute tx_level)
```

Adaptive ramp: scales computed from current TX base level to span the useful range.
Each ramp frame transmitted at a distinct power level (no clamping waste).

### 11.3 Measurement

Peer measures H magnitude and SNR per frame from ZC preamble (works even when LDPC
decode fails). Reports via compact AFSK format. Sender fits parabola or uses
SNR-based fit (for FM-limited channels where H is invariant across power levels).

3-second responder silence guard prevents report collision.

---

## 12. FM Channel Simulator

Software FM channel model for loopback testing. Applied in the audio loopback path.

### 12.1 Channel Model

```
TX audio
  -> Pre-emphasis (configurable time constant)
  -> Deviation limiting (hard clip at threshold)
  -> De-emphasis
  -> Bandpass filter
  -> f^2-shaped discriminator noise
  -> CFO (carrier frequency offset)
  -> Multipath (2-tap: direct + delayed reflection)
  -> RX audio
```

### 12.2 CLI

```
--loopback --fm-channel              Enable FM channel simulator
--fm-cfo <Hz>                        Add carrier frequency offset
--fm-multipath <delay_ms> <gain>     Add 2-tap multipath (gain 0-1)
```

### 12.3 API

```cpp
void loopback_set_fm_channel(float preemph_us, float bp_low, float bp_high,
                              float cfo_hz, float deviation_limit);
void loopback_set_fm_multipath(float delay_ms, float gain);
```

---

## 13. Capability Flags

Negotiated in CONNECT/CONNECT_ACK, advertised in probe result `capabilities` field.

```cpp
constexpr uint16_t CAP_MODE_A      = 0x0001;  // Mode A capable
constexpr uint16_t CAP_MODE_B      = 0x0002;  // Mode B capable
constexpr uint16_t CAP_MODE_C      = 0x0004;  // Mode C capable
constexpr uint16_t CAP_ENCRYPTION  = 0x0008;  // X25519 + ChaCha20-Poly1305
constexpr uint16_t CAP_COMPRESSION = 0x0010;  // PPMd/zstd compression
constexpr uint16_t CAP_B2F_UNROLL  = 0x0020;  // Winlink B2F unroll/reroll
constexpr uint16_t CAP_STREAMING   = 0x0040;  // Streaming compression context
constexpr uint16_t CAP_PQ_CRYPTO   = 0x0080;  // Post-quantum ML-KEM-768 (FIPS 203)
constexpr uint16_t CAP_HARQ        = 0x0100;  // Per-symbol soft HARQ
constexpr uint16_t CAP_OFDM        = 0x0200;  // OFDM PHY capable
```

ML-KEM key exchange frame markers:
```cpp
constexpr uint8_t MLKEM_PK_MAGIC = 0xE1;   // ML-KEM encapsulation key (1184 bytes)
constexpr uint8_t MLKEM_CT_MAGIC = 0xE2;   // ML-KEM ciphertext (1088 bytes)
```

---

## 14. Integration

### 14.1 Config

In `IrisConfig` (config.h), `[OFDM]` INI section:
```cpp
bool ofdm_enable = true;              // Master OFDM enable
int ofdm_nfft = 512;                  // FFT size (256/512/1024)
int ofdm_cp_samples = 32;            // CP length (samples)
bool ofdm_waterfill = true;           // Per-subcarrier adaptive bit loading
bool ofdm_nuc = true;                 // Non-uniform constellations
```

### 14.2 Mode Switching

```
AFSK AX.25 -> XID/connection -> probe -> OFDM PHY (O-levels)  [both CAP_OFDM]
AFSK AX.25 -> XID/connection -> probe -> native SC (A-levels)  [otherwise]
```

OFDM session state machine: `IDLE -> NEGOTIATED -> PROBING -> ACTIVE -> FAILED`.
Managed by `OfdmSession` class with mutex-protected state transitions.

### 14.3 GUI

Advanced tab: OFDM settings section (enable toggle, NFFT dropdown, waterfilling
checkbox, NUC checkbox). Diagnostics: O-level, mean SNR, throughput, frame counts,
constellation scatter plot.

---

## 15. Logging

All modules log with `IRIS_LOG()` macro and module-specific tags.

| Tag | Content |
|-----|---------|
| `[OFDM-CFG]` | Config creation: NFFT, carriers, BW, center, pilot spacing |
| `[OFDM-TX]` | Frame build: symbols, bits, tone map, PAPR, clip stats, DFT-spread |
| `[OFDM-SYNC]` | Detection: SC metric, ZC metric, timing, CFO |
| `[OFDM-CE]` | Channel estimate: mean/min/max |H|, SNR per carrier |
| `[OFDM-RX]` | Frame decode: LLR count, CPE, de-interleave, NUC state |
| `[OFDM-WF]` | Waterfill: bits per carrier, capacity |
| `[OFDM-FAIL]` | Failure: sync, quality gate, LDPC non-convergence |
| `[OFDM-OK]` | Successful decode: payload size, throughput |
| `[OFDM-SESSION]` | Session state transitions, tone map exchange |
