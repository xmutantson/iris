# IRIS OFDM PHY Specification

**Status:** Implementation spec — agents update this document as they build.
**Last updated by:** (initial spec)

---

## 1. Overview

OFDM PHY for IRIS FM modem, operating alongside the existing single-carrier native PHY.
Goal: beat VARA FM throughput at equivalent SNR by using multicarrier modulation with
per-subcarrier adaptive bit loading (waterfilling) and near-Shannon LDPC FEC.

### Design Principles

1. **Probe-adaptive:** Bandwidth, carrier count, and per-carrier modulation all derive
   from the passband probe. Works on any radio from 2 kHz mic/speaker to 6+ kHz data port.
2. **Waterfilling:** Per-subcarrier bit loading based on measured channel SNR.
   Not fixed constellation per speed level — continuous adaptation.
3. **Reuse existing infrastructure:** LDPC codec, ARQ/HARQ, probe system, audio I/O,
   upconverter/downconverter, AGW/KISS interfaces.
4. **Coexist with single-carrier:** CAP_OFDM capability flag. Either side can disable OFDM
   and fall back to A-levels. Configurable via INI/GUI toggle `OfdmEnable`.
5. **Extensive logging:** Every stage logs tagged `[OFDM-xxx]` for debug. Can be compiled
   out later with `IRIS_LOG` macro.

---

## 2. OFDM Parameters

### 2.1 FFT Size and Subcarrier Spacing

Sample rate: 48000 Hz (fixed, `SAMPLE_RATE` in types.h).

| NFFT | Subcarrier Spacing | Symbol Duration | Use Case |
|------|-------------------|-----------------|----------|
| 256  | 187.50 Hz | 5.33 ms | Noisy/drifty radios, fallback |
| 512  | 93.75 Hz  | 10.67 ms | **Default** — good balance |
| 1024 | 46.875 Hz | 21.33 ms | Wide BW (>4 kHz), more carriers |

**Default: NFFT=512, spacing=93.75 Hz.**

Rationale: 93.75 Hz is wide enough for FM phase noise tolerance (ICI floor ~-45 dB,
supports up to 64QAM reliably), narrow enough for meaningful waterfilling (21+ carriers
in 2 kHz, 64+ in 6 kHz). Power-of-2 FFT for efficiency.

### 2.2 Cyclic Prefix

Default: `CP_SAMPLES = 64` at 48 kHz = **1.33 ms**.

This exceeds FM audio channel impulse response (<1 ms group delay variation from
ceramic IF filters). Overhead: 64/512 = 12.5%.

Total OFDM symbol: 512 + 64 = 576 samples = 12.0 ms.

### 2.3 Carrier Allocation

Given probed bandwidth `BW_hz` and center frequency `center_hz`:

```
n_total = floor(BW_hz / subcarrier_spacing)
n_guard = 2                    // 1 guard each edge
n_dc = 0                      // no DC carrier (we're at audio passband, not baseband)
n_used = n_total - n_guard
n_pilot = ceil(n_used / 4)    // every 4th used carrier is a pilot
n_data = n_used - n_pilot
```

At NFFT=512, 2 kHz BW: `n_used=19, n_pilot=5, n_data=14`.
At NFFT=512, 3 kHz BW: `n_used=30, n_pilot=8, n_data=22`.
At NFFT=512, 6 kHz BW: `n_used=62, n_pilot=16, n_data=46`.

Carrier index mapping: carriers placed symmetrically around `center_hz / spacing`
in the FFT bin array. Unused bins set to zero.

### 2.4 Pilot Pattern

**Frequency domain:** Every 4th used subcarrier is a pilot (comb pattern).
Pilot values: +1 (BPSK, known).

**Time domain:** Every 8th OFDM data symbol is an all-pilot symbol (block pilot).
This gives a 2D pilot grid for channel interpolation:
- Frequency: pilot every 4 carriers × 93.75 Hz = 375 Hz resolution
- Time: pilot every 8 symbols × 12 ms = 96 ms resolution

**Pilot overhead:** ~25% frequency + ~12.5% time ≈ 34% total. This is high; can be
optimized later by reducing to every-6th or every-8th carrier once channel estimation
is validated.

### 2.5 Subcarrier Spacing Auto-Training (Optional)

After tone probe, if `AutoTrainSpacing` config is enabled:
1. TX sends 4 OFDM training symbols at NFFT=256
2. TX sends 4 OFDM training symbols at NFFT=512
3. TX sends 4 OFDM training symbols at NFFT=1024
4. RX measures per-subcarrier SNR for each NFFT, computes:
   `capacity_nfft = sum_k[ log2(1 + SNR_k) ]` for each NFFT
5. Both sides select NFFT that maximizes capacity
6. If capacities within 5%, prefer smaller NFFT (more robust)

Default: disabled. Fixed NFFT=512. Can be enabled in INI/GUI.

---

## 3. Frame Structure

### 3.1 OFDM Frame Layout

```
[Training: 2 OFDM symbols] [Header: 1 OFDM symbol] [Data: N OFDM symbols] [Tail: 1]
```

**Training Symbol 1 (Schmidl-Cox):**
- First half and second half are identical in time domain.
- Only even-indexed subcarriers carry known BPSK; odd subcarriers are zero.
- Used for: coarse timing sync (correlation of first half with second half),
  coarse frequency offset estimation (phase diff between halves).

**Training Symbol 2 (Channel Estimation):**
- All used subcarriers carry known BPSK (+1).
- Used for: fine timing, fine frequency offset, full channel estimate H[k] at every subcarrier.
- Per-subcarrier SNR estimated from deviation of received pilot from expected.

**Header Symbol:**
- All data subcarriers carry BPSK (robust).
- Content (bit-packed, MSB first):
  - [4 bits] tone_map_id (0=uniform, 1-15=waterfill table index)
  - [4 bits] FEC rate (same encoding as native header)
  - [12 bits] payload_len (bytes, max 4095)
  - [4 bits] NFFT mode (0=512, 1=256, 2=1024)
  - [1 bit] HARQ flag
  - [3 bits] reserved
  - [8 bits] CRC-8
  Total: 36 bits. With 14 data carriers at BPSK = 14 bits/symbol → 3 OFDM symbols for header.
  **Revised: header = 3 OFDM symbols, all BPSK.**

**Data Symbols:**
- Per-subcarrier modulation defined by tone map.
- Every 8th data symbol is all-pilot (refresh channel estimate).
- Data interleaved across subcarriers and OFDM symbols (frequency-time interleaver).

**Tail Symbol:**
- All subcarriers carry known +1 BPSK.
- Anchors backward channel estimation interpolation.
- Also serves as frame-end marker.

### 3.2 Frame Timing

At NFFT=512, CP=64, 2 kHz BW (14 data carriers):

| Component | OFDM Symbols | Duration |
|-----------|-------------|----------|
| Training  | 2 | 24.0 ms |
| Header    | 3 | 36.0 ms |
| Data (1 LDPC block, QPSK r1/2) | ~58 | 696 ms |
| Block pilots (every 8th) | ~7 | 84 ms |
| Tail      | 1 | 12.0 ms |
| **Total** | **~71** | **~852 ms** |

One LDPC block at QPSK rate 1/2: 1600 coded bits / (14 carriers × 2 bps) = 57.1 symbols.

---

## 4. Modulator

### 4.1 TX Signal Processing Chain

```
Payload bytes
  → CRC-32 append
  → LDPC encode (rate from tone map)
  → Bit interleave (stride-41 congruential per 1600-bit block)
  → BICM interleave (for QAM16+, column-row per block)
  → Scramble (LFSR x^15+x^14+1, same as native)
  → Per-subcarrier symbol mapping (constellation from tone map)
  → Frequency-time interleave (spread across subcarriers and symbols)
  → Insert pilot subcarriers (every 4th, known +1)
  → Insert block pilot symbols (every 8th OFDM symbol, all +1)
  → Prepend training symbols (Schmidl-Cox + full channel est)
  → Prepend header symbols (3× BPSK)
  → Append tail symbol
  → IFFT per OFDM symbol (NFFT-point)
  → Prepend cyclic prefix (CP_SAMPLES)
  → Concatenate all symbols into time-domain baseband IQ
  → Soft clip at 3.0× RMS
  → Upconvert to center_hz (reuse existing Upconverter)
  → Output audio samples
```

### 4.2 Soft Clipping

After IFFT + CP concatenation, compute RMS of entire frame.
Clip: `if |sample| > 3.0 * rms: sample = 3.0 * rms * sign(sample)`
Then apply a short lowpass to smooth clipping edges (FIR, 15-tap Hamming, cutoff = BW).

Log: `[OFDM-TX] soft clip: %d/%d samples clipped (%.1f%%), PAPR=%.1f dB`

### 4.3 Upconversion

Reuse existing `Upconverter::iq_to_audio()` from `include/native/upconvert.h`.
Center frequency = `center_hz` from probe negotiation (e.g. 1700 Hz for 1200-2200 band).

---

## 5. Demodulator

### 5.1 RX Signal Processing Chain

```
Audio samples
  → Downconvert from center_hz (reuse existing Downconverter)
  → Baseband IQ stream
  → Frame detection (Schmidl-Cox correlation on training symbol)
  → Coarse CFO estimation (phase between training symbol halves)
  → CFO correction (rotate entire frame)
  → Strip CP, FFT each OFDM symbol
  → Channel estimation from training symbol 2: H[k] = Y[k] / X[k]
  → Noise variance estimation: σ²[k] from training symbol residuals
  → For each data OFDM symbol:
      → Extract pilot subcarriers, update channel estimate (linear interpolation)
      → MMSE equalize data subcarriers: d[k] = Y[k] * conj(H[k]) / (|H[k]|² + σ²[k])
      → Compute per-subcarrier reliability (|H[k]|² / (|H[k]|² + σ²[k]))
  → Collect equalized data symbols across all OFDM symbols
  → Frequency-time de-interleave
  → Soft demap per subcarrier (using per-subcarrier σ² for LLR scaling)
  → Per-symbol reliability weighting (same as native: magnitude + CSI)
  → Soft descramble
  → BICM de-interleave (QAM16+)
  → LDPC block de-interleave (stride-41)
  → LLR clamp ±20
  → Tail bit hardening
  → Chase combining (if stored LLRs available)
  → LDPC decode (min-sum)
  → CRC-32 check
  → Payload bytes
```

### 5.2 Frame Detection (Schmidl-Cox)

Training symbol 1 has identical first/second halves (each NFFT/2 = 256 samples).

Correlation metric:
```
P(d) = Σ_{m=0}^{L-1} r(d+m) * conj(r(d+m+L))   where L = NFFT/2
R(d) = Σ_{m=0}^{L-1} |r(d+m+L)|²
M(d) = |P(d)|² / R(d)²
```

Peak of M(d) gives frame timing. Phase of P(d) at peak gives coarse CFO:
```
cfo_hz = angle(P(d_peak)) / (2π * L / sample_rate)
```

Log: `[OFDM-RX] Schmidl-Cox peak: M=%.4f at sample %d, CFO=%.1f Hz`

### 5.3 CFO Correction

Rotate all subsequent samples: `r_corrected(n) = r(n) * exp(-j*2π*cfo*n/Fs)`

Fine CFO refinement from training symbol 2: compare received pilot phases against
expected. Residual CFO = mean phase slope across subcarriers.

Log: `[OFDM-RX] CFO coarse=%.1f Hz, fine=%.2f Hz, total=%.2f Hz`

### 5.4 Channel Estimation

From training symbol 2 (all subcarriers known +1):
```
H[k] = Y_train2[k] / X_train2[k] = Y_train2[k]   (since X=+1)
```

Noise variance per subcarrier (from training symbol residuals):
```
σ²[k] = E[|Y - H·X|²]   (averaged over training + block pilot symbols)
```

Channel estimate updated at each block-pilot symbol (every 8th data symbol):
linear interpolation between block pilots in time, linear interpolation between
comb pilots in frequency.

Log: `[OFDM-RX] channel est: mean |H|=%.3f, SNR range %.1f-%.1f dB across %d carriers`

### 5.5 MMSE Equalization

Per subcarrier, per OFDM symbol:
```
d[k] = Y[k] * conj(H_interp[k]) / (|H_interp[k]|² + σ²[k])
```

This is MMSE (regularized ZF). When |H|² >> σ², it's ZF. When |H|² ≈ σ²,
it attenuates rather than amplifying noise. Same principle as Mercury's MMSE.

Reliability per subcarrier: `reliability[k] = |H[k]|² / (|H[k]|² + σ²[k])`

### 5.6 Interface to Existing LDPC/HARQ Pipeline

The OFDM demodulator produces the same outputs as the native single-carrier demodulator
(step 15 in the decode chain). Specifically:

1. `std::vector<std::complex<float>> symbols` — equalized data symbols, pilots stripped,
   de-interleaved to match TX symbol order
2. `std::vector<float> sym_reliability` — per-symbol magnitude (for soft erasure)
3. `std::vector<float> sym_phase_var` — per-symbol noise variance (for CSI weighting).
   For OFDM: `1.0 / (SNR_subcarrier * reliability[k])` mapped to each symbol
4. `float snr_db` — overall SNR estimate (average across subcarriers)
5. `float channel_gain` — mean |H| across used subcarriers

These feed directly into: `demap_soft()`, per-symbol weighting, scramble_soft, BICM
de-interleave, LDPC block de-interleave, tail hardening, Chase combining, LDPC decode.

**The OFDM demod reuses the ENTIRE post-equalization pipeline from frame.cc.**
We extract the LLR computation, weighting, de-interleaving, and LDPC decode into
shared functions callable by both native and OFDM demodulators.

---

## 6. Waterfilling / Tone Map

### 6.1 Bit Loading Algorithm

Chow-Cioffi-Bingham (CCB) bit loading:

Input: per-subcarrier SNR `snr[k]` (from probe tone power + noise floor estimate,
refined by training symbol channel estimate).

For each data subcarrier k:
```
bits[k] = floor(log2(1 + snr[k] / Γ))
```
where `Γ` is the SNR gap. With LDPC at rate R and BER target 10^-5:
- Rate 1/2: Γ ≈ 2.0 dB (1.58 linear)
- Rate 3/4: Γ ≈ 3.5 dB (2.24 linear)
- Rate 7/8: Γ ≈ 5.0 dB (3.16 linear)

Clamp: `bits[k] = min(bits[k], 8)` (max 256QAM).
If `bits[k] < 1`, null the carrier (don't transmit on it).

Total bits per OFDM data symbol: `B = Σ bits[k]` over data carriers.
Net throughput: `B × baud × fec_rate` where `baud = 1 / T_symbol_total`.

### 6.2 Tone Map Structure

```cpp
struct ToneMap {
    uint8_t bits_per_carrier[MAX_OFDM_CARRIERS];  // 0,1,2,4,6,8
    int n_data_carriers;
    int total_bits_per_symbol;  // sum of bits_per_carrier
    LdpcRate fec_rate;
    int nfft;
};
```

### 6.3 Tone Map Negotiation

Both sides compute tone maps from exchanged probe data (deterministic: same inputs →
same output). No additional exchange needed.

Steps:
1. After probe exchange, both sides have `tone_power_db[64]` for both directions.
2. Intersect detected tone ranges (existing `probe_negotiate()`).
3. Estimate per-subcarrier SNR: `snr[k] = tone_power[k] - noise_floor[k]`.
   Noise floor from probe null analysis (midpoints between tones).
4. Run CCB bit loading.
5. Both sides arrive at identical tone map.

**If the tone map needs refinement** after the training symbols (actual channel
may differ slightly from probe prediction), the TX side includes `tone_map_id`
in the header. ID 0 = computed-from-probe. IDs 1-15 = predefined uniform maps
(all BPSK, all QPSK, etc.) as fallback.

### 6.4 Predefined Uniform Tone Maps (Fallback)

For robustness, define uniform maps that don't require waterfilling:

| ID | Modulation | FEC Rate | Description |
|----|-----------|----------|-------------|
| 0  | (waterfill) | (from map) | Computed from probe |
| 1  | BPSK | 1/2 | Most robust |
| 2  | QPSK | 1/2 | Safe default |
| 3  | QPSK | 3/4 | Good balance |
| 4  | 16QAM | 1/2 | Medium |
| 5  | 16QAM | 3/4 | Medium-high |
| 6  | 64QAM | 3/4 | High SNR |
| 7  | 64QAM | 7/8 | Very high SNR |
| 8  | 256QAM | 7/8 | Maximum rate |

---

## 7. HARQ-IR with Rate-Compatible LDPC

### 7.1 Mother Code

Use Mercury's rate 4/16 (K=400, P=1200, N=1600) as the mother code. This provides
maximum redundancy for incremental retransmission.

Copy from Mercury to Iris:
- `mercury_normal_4_16.h` / `mercury_normal_4_16.cc`
- `mercury_normal_6_16.h` / `mercury_normal_6_16.cc`
- `mercury_normal_10_16.h` / `mercury_normal_10_16.cc`

### 7.2 Puncturing Scheme

First TX: encoder produces all 1200 parity bits (rate 4/16 mother code).
Transmit subset based on target rate:

| Target Rate | Data bits | Parity bits sent | Effective rate |
|------------|-----------|-----------------|----------------|
| 7/8 (14/16) | 1400 | 200 | 0.875 |
| 3/4 (12/16) | 1200 | 400 | 0.75 |
| 5/8 (10/16) | 1000 | 600 | 0.625 |
| 1/2 (8/16)  | 800  | 800 | 0.50 |
| 3/8 (6/16)  | 600  | 1000 | 0.375 |
| 1/4 (4/16)  | 400  | 1200 | 0.25 |

**Important:** The data portion length varies per rate! For HARQ-IR, the mother code
(4/16, K=400) is always used for encoding. First TX sends K=400 data + first 400
parity = rate 1/2 effective. On NACK, send next 400 parity → rate 1/3. On NACK again,
send last 400 parity → rate 1/4.

Alternative: use existing rate-1/2 code (K=800) as base, encode with all 800 parity.
First TX: send all 800 parity (rate 1/2). On NACK, retransmit same (Chase combining).
This is simpler and doesn't require new matrix tables.

**Decision: Start with existing rate 1/2 Chase combining (already implemented in HARQ).
Add true IR with 4/16 mother code as Phase 5 improvement.**

### 7.3 Integration with Existing HARQ

The existing HARQ system (`on_decode_failed_harq`, per-block decode, selective NACK
with region info, piggybacked retransmit) works with OFDM unchanged. The LLR storage
and Chase combining operate at the FEC layer.

For OFDM, `sym_phase_var` maps to per-subcarrier reliability instead of Kalman P00,
but the region selection logic (`select_bad_regions`) works identically — it finds
contiguous regions of low-reliability symbols and requests retransmission.

---

## 8. Non-Uniform Constellations (NUC)

### 8.1 Implementation

Replace standard QAM constellation points with optimized NUC points from ATSC 3.0
(published in ATSC A/322 standard, Table 7.4 through 7.13).

One lookup table per (constellation_order, fec_rate) pair:
- 16-NUC: 2D coordinates for 16 points, per FEC rate
- 64-NUC: 2D coordinates for 64 points, per FEC rate
- 256-NUC: 1D (separable I/Q) coordinates for 256 points, per FEC rate

**Gains:** 0.2-0.5 dB at 16QAM, 0.5-0.8 dB at 64QAM, 0.8-1.0 dB at 256QAM.

### 8.2 Changes

- New file: `include/native/nuc_tables.h` — NUC coordinate tables
- Modify `constellation.cc`: `qam_map()` and `demap_soft()` accept optional NUC table pointer
- When NUC table provided, use NUC coordinates instead of standard QAM grid
- LLR computation: for 1D-NUC (256QAM), decompose into I/Q axes (same complexity as uniform).
  For 2D-NUC (16/64QAM), full 2D search over all constellation points.

---

## 9. FFT Utility

### 9.1 Shared FFT

Extract the existing Cooley-Tukey radix-2 FFT from `passband_probe.cc` into a
shared module:

- `include/common/fft.h`
- `source/common/fft.cc`

Functions:
```cpp
namespace iris {
    // Forward FFT (DFT). In-place, radix-2, requires power-of-2 n.
    void fft(float* re, float* im, int n);

    // Inverse FFT (IDFT). In-place, same requirements.
    void ifft(float* re, float* im, int n);

    // Complex version (interleaved re/im pairs)
    void fft_complex(std::complex<float>* data, int n);
    void ifft_complex(std::complex<float>* data, int n);
}
```

IFFT = conjugate inputs, FFT, conjugate outputs, divide by N.

---

## 10. Integration

### 10.1 Capability Flag

```cpp
constexpr uint16_t CAP_OFDM = 0x0200;  // OFDM PHY capable
```

Added to types.h. Advertised in probe result `capabilities` field.
Both sides must have `CAP_OFDM` to use OFDM PHY.

### 10.2 Config

In `IrisConfig` (config.h):
```cpp
bool ofdm_enable = true;          // Master OFDM enable (GUI toggle)
int ofdm_nfft = 512;              // Default NFFT (256/512/1024)
int ofdm_cp_samples = 64;         // CP length in samples
bool ofdm_auto_spacing = false;   // Auto-train subcarrier spacing
bool ofdm_waterfill = true;       // Per-subcarrier adaptive bit loading
bool ofdm_nuc = true;             // Non-uniform constellations
```

### 10.3 Mode Switching

Current flow:
```
AFSK AX.25 → XID/connection header → probe → native single-carrier (A-levels)
```

New flow (when both sides have CAP_OFDM):
```
AFSK AX.25 → XID/connection header → probe → OFDM PHY (O-levels)
```

When one or both sides lack CAP_OFDM:
```
AFSK AX.25 → XID/connection header → probe → native single-carrier (A-levels)
```

The OFDM PHY slot-in happens in `modem.cc` where `native_mode_` is set. Instead
of calling `process_rx_native()` and using native frame build/decode, the OFDM
variants are called. The switching point is in `process_rx()` and `process_tx()`.

### 10.4 GUI Changes

In the Advanced tab of the Settings window (`gui_imgui.cc`):
```
[x] Enable OFDM mode (auto-upgrade from single-carrier)
    NFFT: [256 | *512* | 1024]  (dropdown)
    [x] Waterfilling (per-subcarrier adaptive modulation)
    [x] Non-uniform constellations (NUC)
    [ ] Auto-train subcarrier spacing
```

---

## 11. Speed Levels (O-levels)

Unlike A-levels (fixed modulation + FEC per level), O-levels are defined by the
FEC rate only. The per-subcarrier modulation is determined by waterfilling.

| Level | FEC Rate | Min SNR | Description |
|-------|---------|---------|-------------|
| O0 | 1/2 | 3 dB | Most robust |
| O1 | 5/8 | 8 dB | |
| O2 | 3/4 | 12 dB | Default |
| O3 | 7/8 | 18 dB | High throughput |

The actual throughput at each level depends on the waterfill result:
```
throughput = Σ_k bits[k] × (1/T_symbol_total) × fec_rate × (data_symbols / total_symbols)
```

Gearshift: varies FEC rate based on per-block LDPC convergence behavior (same
EMA-smoothed SNR + iteration count logic as current gearshift.cc, but with
O-level table instead of A-level table).

---

## 12. File Structure

New files to create:

```
include/ofdm/
    ofdm_config.h       — OfdmConfig struct, factory from probe
    ofdm_mod.h          — OfdmModulator class
    ofdm_demod.h        — OfdmDemodulator class
    ofdm_frame.h        — OFDM frame builder/parser, tone map, waterfill
    ofdm_sync.h         — Schmidl-Cox sync, CFO estimation

source/ofdm/
    ofdm_config.cc      — Config factory, carrier allocation
    ofdm_mod.cc         — IFFT-based modulator, CP, soft clip
    ofdm_demod.cc       — FFT-based demodulator, channel est, MMSE EQ
    ofdm_frame.cc       — Frame build/decode, tone map, waterfill algorithm
    ofdm_sync.cc        — Frame detection, CFO estimation/correction

include/common/
    fft.h               — Shared FFT/IFFT (extracted from probe)

source/common/
    fft.cc              — FFT/IFFT implementation

include/native/
    nuc_tables.h        — Non-uniform constellation tables (ATSC 3.0)
```

Files to modify:
```
include/common/types.h      — Add CAP_OFDM
include/config/config.h     — Add OfdmConfig fields to IrisConfig
source/config/config.cc     — Load/save OFDM settings
source/engine/modem.cc      — OFDM mode switching in process_rx/process_tx
source/engine/speed_level.cc — Add O-level definitions
include/engine/speed_level.h — O-level enum/struct
source/gui/gui_imgui.cc     — OFDM settings UI
source/probe/passband_probe.cc — Extract FFT to shared module
source/native/constellation.cc — NUC table support in map/demap
source/native/frame.cc      — Extract shared LLR/decode pipeline functions
include/native/frame.h      — Shared function declarations
source/fec/ldpc.cc          — Add new rate matrices (4/16, 6/16, 10/16)
include/fec/ldpc.h          — New rate enum entries
```

Mercury LDPC tables to copy:
```
mercury/include/physical_layer/mercury_normal_4_16.h  → iris/include/fec/
mercury/source/physical_layer/mercury_normal_4_16.cc  → iris/source/fec/
mercury/include/physical_layer/mercury_normal_6_16.h  → iris/include/fec/
mercury/source/physical_layer/mercury_normal_6_16.cc  → iris/source/fec/
mercury/include/physical_layer/mercury_normal_10_16.h → iris/include/fec/
mercury/source/physical_layer/mercury_normal_10_16.cc → iris/source/fec/
```

---

## 13. Logging Requirements

Every module must log with a module-specific tag. All logs use `IRIS_LOG()` macro.

| Tag | What to log |
|-----|------------|
| `[OFDM-CFG]` | Config creation: NFFT, n_carriers, n_data, n_pilot, BW, center |
| `[OFDM-TX]` | Frame build: n_symbols, total_bits, tone_map summary, PAPR, clip stats |
| `[OFDM-SYNC]` | Detection: Schmidl-Cox metric, timing, CFO coarse/fine |
| `[OFDM-CE]` | Channel est: mean/min/max |H|, mean/min/max SNR per carrier |
| `[OFDM-EQ]` | Equalization: per-carrier reliability summary |
| `[OFDM-RX]` | Frame decode: header parsed, n_data_symbols, LLR stats |
| `[OFDM-WF]` | Waterfill: bits assigned per carrier, total bits/sym, capacity |
| `[OFDM-NUC]` | NUC: which table loaded, which constellation |
| `[OFDM-FAIL]` | Any failure: sync failed, header CRC fail, LDPC non-convergence |
| `[OFDM-OK]` | Successful decode: payload size, effective throughput |

---

## 14. Execution Plan (Parallelized Agent Tasks)

### Batch 1: Independent Foundation (all parallel)

**Agent A: FFT Utility**
- Extract FFT from passband_probe.cc into include/common/fft.h + source/common/fft.cc
- Add IFFT, complex versions
- Update passband_probe.cc to use shared FFT
- Test: verify FFT round-trip (IFFT(FFT(x)) == x)

**Agent B: OFDM Config + Carrier Allocation**
- Create include/ofdm/ofdm_config.h + source/ofdm/ofdm_config.cc
- OfdmConfig struct, factory from NegotiatedPassband
- Carrier index mapping (which FFT bins map to which carriers)
- Pilot carrier selection
- Logging: [OFDM-CFG]

**Agent C: LDPC Table Import**
- Copy mercury_normal_4_16, 6_16, 10_16 to iris
- Update ldpc.cc get_ira_matrix() to use new tables for RATE_4_16, RATE_6_16, RATE_5_8
- Update ldpc.h effective_rate() to not fall back to 1/2 for these rates
- Test: encode/decode roundtrip at each new rate

**Agent D: NUC Tables**
- Create include/native/nuc_tables.h with ATSC 3.0 constellation coordinates
- Modify constellation.cc to accept NUC table in qam_map() and demap_soft()
- Tables for 16-NUC, 64-NUC at rates 1/2, 3/4, 7/8
- Tables for 256-NUC (1D) at rates 3/4, 7/8
- Logging: [OFDM-NUC]

**Agent E: Config + GUI + Capability**
- Add CAP_OFDM to types.h
- Add OFDM fields to IrisConfig in config.h
- Add load/save in config.cc
- Add OFDM settings section in gui_imgui.cc (Advanced tab)
- Add O-level definitions to speed_level.h/cc

### Batch 2: Core OFDM (depends on Batch 1: A, B)

**Agent F: OFDM Modulator**
- Create include/ofdm/ofdm_mod.h + source/ofdm/ofdm_mod.cc
- Training symbol generation (Schmidl-Cox + full channel est)
- Per-subcarrier symbol mapping from tone map
- IFFT + CP insertion
- Soft clipping
- Uses shared FFT (Agent A) and OfdmConfig (Agent B)
- Logging: [OFDM-TX]

**Agent G: OFDM Sync + Channel Estimation**
- Create include/ofdm/ofdm_sync.h + source/ofdm/ofdm_sync.cc
- Schmidl-Cox frame detection
- Coarse + fine CFO estimation and correction
- Channel estimation from training symbol
- Channel interpolation (time + frequency)
- Noise variance estimation
- Uses shared FFT (Agent A) and OfdmConfig (Agent B)
- Logging: [OFDM-SYNC], [OFDM-CE]

**Agent H: Waterfilling + Tone Map**
- Create waterfill functions in include/ofdm/ofdm_frame.h + source/ofdm/ofdm_frame.cc
- CCB bit-loading algorithm
- ToneMap struct + predefined uniform maps
- Tone map computation from probe per-tone power
- Tone map serialization (for header)
- Logging: [OFDM-WF]

### Batch 3: Integration (depends on Batch 2: F, G, H)

**Agent I: OFDM Demodulator + Frame Decode**
- Create include/ofdm/ofdm_demod.h + source/ofdm/ofdm_demod.cc
- Full RX chain: sync → CFO → FFT → channel est → MMSE EQ → data extraction
- Interface to existing LLR pipeline (extract shared functions from frame.cc)
- Chase combining integration
- HARQ per-block integration
- Build + decode functions for complete OFDM frames
- Logging: [OFDM-EQ], [OFDM-RX], [OFDM-FAIL], [OFDM-OK]

**Agent J: Modem Integration**
- Modify modem.cc: add OFDM mode switching
- After probe + CAP_OFDM negotiation → switch to OFDM PHY
- process_rx_ofdm() / process_tx_ofdm() methods
- Gearshift adaptation for O-levels
- OFDM-KISS transport integration (AX.25 frames over OFDM)

### Batch 4: Validation (depends on Batch 3)

**Agent K: Loopback Test**
- Build and run loopback test: OFDM TX → internal loopback → OFDM RX
- Test all uniform tone maps (O0 through O3)
- Test waterfill mode
- Test with AWGN at various SNR levels
- Measure throughput at each operating point
- Report results, compare to A-level throughput

---

## 15. Agent Instructions Template

Each agent MUST:
1. Read this spec document before starting work.
2. Follow the logging requirements (Section 13).
3. After completing work, **update this document** with:
   - Actual implementation details that differ from spec
   - Any design decisions made that aren't covered by spec
   - Actual file:line references for key functions
   - Status of their task (DONE / PARTIAL / BLOCKED)
4. Ensure code compiles (run `bash build.sh o3` from iris/).
5. Do NOT modify files assigned to other agents in the same batch.

---

## 16. Status Tracker

| Agent | Task | Status | Notes |
|-------|------|--------|-------|
| A | FFT Utility | DONE | `include/common/fft.h` (4 functions: fft, ifft, fft_complex, ifft_complex), `source/common/fft.cc` (radix-2 Cooley-Tukey + log-once per size). `passband_probe.cc` updated to `#include "common/fft.h"` and call `iris::fft()`. No deviations from spec. |
| B | OFDM Config | DONE | Created include/ofdm/ofdm_config.h + source/ofdm/ofdm_config.cc. OfdmConfig struct with carrier allocation from NegotiatedPassband. freq_to_bin()/bin_to_freq() helpers. Pilot every 4th used carrier, 1 guard each edge. Carrier counts match spec Sec 2.3 (e.g. NFFT=512 2kHz: 19 used, 5 pilot, 14 data). [OFDM-CFG] logging. Build passes. |
| C | LDPC Tables | DONE | Copied mercury_normal_{4,6,10}_16 .h/.cc to iris/fec/. Fixed include paths. Added RATE_4_16 (k=400,p=1200), RATE_6_16 (k=600,p=1000), RATE_5_8/10_16 (k=1000,p=600) to get_ira_matrix() and effective_rate(). All 6 rates now have native matrices (4/16, 6/16, 8/16, 10/16, 12/16, 14/16). Build passes. |
| D | NUC Tables | DONE | nuc_tables.h: 8 tables (16/64-NUC 2D + 256-NUC 1D), constellation.cc: map_symbol_nuc + demap_soft_nuc |
| E | Config/GUI/Cap | DONE | CAP_OFDM=0x0200 in types.h, 6 OFDM fields in IrisConfig, [OFDM] INI section, GUI Advanced tab OFDM section, O0-O3 speed levels in OFDM_SPEED_LEVELS[] + ofdm_snr_to_speed_level() |
| F | OFDM Modulator | DONE | Created include/ofdm/ofdm_mod.h + source/ofdm/ofdm_mod.cc. OfdmModulator class with generate_training_symbol_1 (Schmidl-Cox PN even carriers), generate_training_symbol_2 (all +1), generate_data_symbol, generate_pilot_symbol, build_ofdm_frame (full TX chain). ToneMap struct + make_uniform_tone_map() for presets 1-8. TX chain: CRC32 append, LDPC encode, stride-41 interleave, LFSR scramble, per-carrier constellation mapping, pilot insertion (comb every 4th + block every 8th), 3-symbol BPSK header (36 bits: tone_map_id/fec/payload_len/nfft_mode/harq/CRC8), tail symbol, IFFT+CP, soft clip at 3x RMS. Logs [OFDM-TX] with symbol count, payload size, PAPR, clip count. No BICM global interleave (deferred to per-carrier waterfill context). Spec deviation: skipped post-clip FIR smoothing (Section 4.2) — can add later if needed. |
| G | OFDM Sync/CE | DONE | Created include/ofdm/ofdm_sync.h + source/ofdm/ofdm_sync.cc. Schmidl-Cox sliding-window detection (threshold 0.3), CFO from arg(P)*Fs/(2piL), SNR from |P|/(R-|P|). Channel est via NFFT-point FFT of training sym 2, noise_var from 3-tap smoothed H residuals. IIR block-pilot update (alpha=0.3). Comb-pilot interpolation with linear interp of H and noise_var. All per spec Sec 5.2-5.4. Tags: [OFDM-SYNC], [OFDM-CE]. |
| H | Waterfill/ToneMap | DONE | Created `include/ofdm/ofdm_frame.h` + `source/ofdm/ofdm_frame.cc`. CCB waterfilling: SNR gap per FEC rate (r1/2=2.0dB, r5/8=2.5dB, r3/4=3.5dB, r7/8=5.0dB), floor(log2(1+snr/gap)), round to valid {0,1,2,4,6,8}. Probe-to-tonemap: lerp probe tones (66.7Hz) to OFDM subcarriers (93.75Hz), noise floor from median of adj-tone-min minus 20dB, min of both directions (conservative). Uniform presets 1-8 via `get_uniform_tone_map()`. Throughput: accounts for training(2)+header(3)+block pilots+tail(1) overhead. Serialization: 1-byte header [4 id | 4 fec_field] + nibble-packed bpc/2 for waterfill. ToneMap struct reused from ofdm_mod.h (Agent F). Tag: [OFDM-WF]. No spec deviations. |
| I | OFDM Demodulator | DONE | Created `include/ofdm/ofdm_demod.h` + `source/ofdm/ofdm_demod.cc`. OfdmDemodulator class with full RX chain: Schmidl-Cox detection → CFO correction → channel est from training sym 2 → 3-symbol BPSK header decode (ZF, CRC-8 verify) → tone map resolution (uniform preset from header or waterfill from caller) → data symbol reception with block-pilot tracking (every 8th) and comb-pilot interpolation → MMSE equalization per data carrier (zero deep-fade carriers where |H|²<σ²) → per-carrier soft demapping to LLRs (sigma_sq from MMSE noise model) → LLR-level descrambling (sign flip) → stride-41 de-interleave → LDPC soft decode (min-sum, 50 iter) → LSB-first bit-to-byte → CRC-32 verify. OfdmDemodResult returns payload, diagnostics, per-carrier SNR, and raw LLRs for HARQ Chase combining. All TX/RX parameters match exactly (CRC-8 poly, LFSR seed, FEC field encoding, interleave stride, bit packing order). Tag: [OFDM-RX]. |
| J | Modem Integration | DONE | Created `include/ofdm/ofdm_session.h` + `source/ofdm/ofdm_session.cc`. OfdmSession class manages OFDM state machine (IDLE→NEGOTIATED→ACTIVE→FAILED). Init from NegotiatedPassband + IrisConfig, creates OfdmModulator/OfdmDemodulator. State transitions on CAP_OFDM negotiation and probe completion. build_tx_frame() and process_rx_frame() delegate to mod/demod with current tone map. Waterfill from probe or channel feedback, uniform preset fallback. O-level gearshift (O0→RATE_1_2 through O3→RATE_7_8). Tone map serialization for peer exchange. OfdmSessionDiag for GUI. Mutex-protected. Tag: [OFDM-SESSION]. |
| K | Loopback Test | DONE | `tools/ofdm_loopback_test.cc`. Tests 6 uniform presets (BPSK r1/2 through 256QAM r7/8) at 7 SNR levels (5-40 dB), 3 trials each (32/64/128 byte payloads) + 1 waterfill test at 20 dB. Box-Muller AWGN. Results: **123/129 PASS (95.3%)**. All presets BPSK through 64QAM pass at 5 dB. 256QAM r7/8 passes at 15 dB+. Waterfill 3/3 PASS at 20 dB. Only failures: 256QAM at 5/10 dB (expected — insufficient SNR). |
| - | Sync Bugfixes | DONE | Fixed Schmidl-Cox: (1) Proper Cauchy-Schwarz normalization M=|P|²/(E·R) instead of |P|²/R² (prevents M>1 false peaks). (2) First-peak detection (take first threshold crossing, not global max) prevents false detections from data symbols. (3) Left-edge CP plateau snapping avoids FFT window leaking into next symbol's CP. |
