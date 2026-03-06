# Iris — Open Source FM Data Modem

## Overview

High-throughput ARQ data modem for VHF/UHF FM channels. Open-source replacement
for VARA FM. Single-carrier design optimized for the flat, high-SNR FM channel.

Named as companion to Mercury (HF OFDM modem): Mercury carries messages via
skywave, Iris carries them line-of-sight.

License: AGPL-3.0 (matches Mercury, shared code compatible)
Language: C++ (shared codebase with Mercury)

## Design Philosophy

- FM channels are flat and stable — no need for OFDM
- Single-carrier with pulse shaping is simpler, more efficient, lower latency
- Reuse Mercury's proven ARQ engine, LDPC codes, compression, and encryption
- Three transport modes for maximum radio compatibility
- Adaptive modulation from BPSK to 256-QAM based on link quality
- AX.25 compatibility layer for interop with existing packet network
- Transition path: talk to legacy stations today, upgrade to native Iris when both sides have it
- Clean-room AFSK/GFSK implementation (no Direwolf code — GPLv2 incompatible with AGPL)

## Channel Modes

### Mode A — Audio-Coupled (Mic/Speaker)
- Works with any FM radio through audio jacks (including USB audio)
- Usable bandwidth: ~2800 Hz (300–3100 Hz)
- Symbol rate: 2400 baud (RRC alpha=0.2 → 2880 Hz occupied)
- Target: portable/handheld radios, maximum compatibility
- Latency: ~100ms TX/RX turnaround (VOX or CAT PTT)
- Note: USB audio interfaces (e.g., IC-705) give flatter response than analog

### Mode B — 9600 Baud Port (Discriminator/Direct)
- Flat baseband access, bypasses audio filtering
- Usable bandwidth: ~4800 Hz
- Symbol rate: 4800 baud
- Target: base stations, TNC-ready radios (Kenwood TM-D710, IC-9700, etc.)

### Mode C — Channel-Direct/SDR (Full Channel)
- Full FM channel bandwidth via SDR or direct modulation
- 12.5 kHz channel: symbol rate 9600 baud
- 25 kHz channel: symbol rate 19200 baud
- Target: SDR setups (PlutoSDR, LimeSDR, HackRF), digital-native radios
- Single-carrier with fractionally-spaced equalizer (not OFDM — VHF multipath
  is only 2-10% of symbol duration, a 5-tap FSE handles it)

## Modulation & Coding

### Modulation Ladder

| Level | Modulation | Bits/sym | Min SNR (approx) |
|-------|------------|----------|-------------------|
| 0     | BPSK       | 1        | 3 dB              |
| 1     | QPSK       | 2        | 6 dB              |
| 2     | 8PSK       | 3        | 10 dB             |
| 3     | 16QAM      | 4        | 14 dB             |
| 4     | 32QAM      | 5        | 18 dB             |
| 5     | 64QAM      | 6        | 22 dB             |
| 6     | 128QAM     | 7        | 26 dB             |
| 7     | 256QAM     | 8        | 30 dB             |

### LDPC Code Rates
Reuse Mercury's LDPC codes: 1/16, 1/8, 1/4, 1/2, 3/4, 7/8, 14/16.
Adaptive code rate combined with modulation level = fine-grained throughput control.

### Throughput Estimates (Mode A, 2400 baud)

| Config       | Modulation | Code Rate | Throughput |
|--------------|------------|-----------|------------|
| Minimum      | BPSK       | 1/2       | 1,200 bps  |
| Low          | QPSK       | 3/4       | 3,600 bps  |
| Medium       | 16QAM      | 3/4       | 7,200 bps  |
| High         | 64QAM      | 7/8       | 12,600 bps |
| Maximum      | 256QAM     | 7/8       | 16,800 bps |

Mode B doubles these. Mode C (25 kHz) = 8x Mode A rates.
Peak theoretical: 256QAM 7/8 on 25 kHz = **134 kbps**.

### Comparison
| | Direwolf 1200 | Direwolf 9600 | VARA FM | Iris AX.25 | Iris Native A |
|---|---|---|---|---|---|
| Peak throughput | ~1 kbps | ~7 kbps | ~25 kbps | ~7 kbps | ~17 kbps |
| FEC | None | None | Yes | None | LDPC |
| Adaptive | No | No | Yes | No | BPSK→256QAM |
| Encryption | No | No | No | No | ChaCha20 |
| Compression | No | No | No | No | PPMd+zstd |
| AX.25 compat | Yes | Yes | No | Yes | Auto-upgrade |
| Open source | Yes | Yes | No | Yes | Yes |
| Audio-coupled | Yes | Yes | Yes | Yes | Yes |

## Physical Layer

### Pulse Shaping
- Root-raised-cosine (RRC) filter, rolloff alpha = 0.2
- Matched filter at receiver
- Zero ISI at optimal sample point

### Synchronization
- Preamble: known BPSK training sequence (63-bit m-sequence or Barker)
- Timing recovery: Gardner TED or Mueller-Muller
- Carrier recovery: Costas loop (for low-order) / decision-directed PLL (high-order)
- AGC: RMS-based with fast attack / slow decay

### Frame Structure

```
[Preamble 63sym] [Sync Word 16sym] [Header 32sym BPSK] [Payload Nsym] [CRC32]
```

- **Preamble**: m-sequence, always BPSK, used for timing/carrier sync + SNR estimate
- **Sync word**: frame delimiter, Barker-like sequence
- **Header**: modulation level, code rate, payload length, frame type — always BPSK
  so receiver can decode regardless of payload modulation
- **Payload**: variable modulation/coding per header
- **CRC32**: error detection on decoded payload

### Channel Estimation
- Preamble-based SNR measurement (no per-subcarrier estimate needed — flat channel)
- Decision-directed tracking for phase/amplitude drift during payload
- Optional mid-frame pilots for Mode C at high symbol rates

### Equalization
- Simple 1-tap complex gain correction (flat channel, Modes A/B)
- 5-tap fractionally-spaced linear equalizer for Mode C (urban VHF multipath)

## Data Link Layer (Reuse from Mercury)

### ARQ Protocol
- Commander/Responder architecture
- Selective repeat ARQ with NACK
- Adaptive batch sizing

### Gearshift
- SNR-based SUPERSHIFT for fast initial climb
- Ladder gearshift for fine adaptation
- Verification probe at top config
- BREAK recovery with cascading fallback

### Features (inherited from Mercury)
- PPMd8 + zstd streaming compression with capability negotiation
- X25519 + ChaCha20-Poly1305 encryption
- B2F unroll/reroll for Winlink
- SSID addressing
- Passive monitor mode

### Connection Sequence
1. HAIL (BPSK, always decodable)
2. Capability negotiation (compression, encryption, B2F)
3. Key exchange (if encrypted)
4. SUPERSHIFT to optimal modulation
5. Data transfer with ladder gearshift

## AX.25 Compatibility Layer

### Goal
Talk to any existing 1200/9600 baud packet station. An Iris station should be
a drop-in replacement for Direwolf — existing stations don't need to know or
care that the other end is Iris.

### Architecture
```
Application (Winlink, BPQ32, APRS client, Pat)
       |
   KISS TNC interface (TCP :8001 / serial PTY)
       |
  +----+----+
  |         |
AX.25     Iris Native
legacy    (when both sides
mode      support it)
```

### AX.25 Legacy Mode
- Clean-room AFSK 1200 baud (Bell 202) modulator/demodulator
- Clean-room GFSK 9600 baud (G3RUH) modulator/demodulator
- Standard AX.25 HDLC framing, NRZI encoding
- KISS TNC interface on TCP port 8001 and/or serial PTY
- Compatible with Winlink, APRS, BPQ32, Pat, and any KISS-aware application
- Functionally equivalent to Direwolf — same protocols, same framing

### Auto-Upgrade Negotiation
When two Iris stations connect, they can upgrade to native mode mid-session
using a combination of XID frames (connected mode) and reserved PID bytes
(connectionless/APRS).

#### Connected Mode (Winlink, BPQ32, etc.)
1. Station A sends normal AX.25 SABM — works with any TNC
2. Connection established in standard AX.25
3. Station A sends XID frame with Iris capability parameters
4. If peer responds with XID: both sides switch to Iris native framing
5. If peer responds with FRMR or silence: stay in AX.25 mode
6. If native link drops, fall back to AX.25 automatically

XID (Exchange Identification) is an AX.25 2.2 standard frame type designed
specifically for capability negotiation. Legacy TNCs that don't understand XID
respond with FRMR, which cleanly signals "not Iris-capable."

#### XID Capability Frame Format
```
XID Information Field (8 bytes, fixed format):
  Bytes 0-3: Magic "IRIS" (0x49 0x52 0x49 0x53)
  Byte 4:    Version (0x01 = v1)
  Bytes 5-6: Capabilities bitmask (little-endian)
    bit 0: Native Mode A supported
    bit 1: Native Mode B supported
    bit 2: Native Mode C supported
    bit 3: Encryption capable (ChaCha20)
    bit 4: Compression capable (PPMd+zstd)
    bit 5: B2F unroll capable
  Byte 7:    Max modulation level (0-7, maps to modulation ladder)
```

Version byte allows future extension. If version > what receiver understands,
negotiate down to common version.

#### Connectionless Mode (APRS, UI frames)
- Use reserved PID byte (0xBE) for Iris-native UI frames
- Legacy TNCs silently drop frames with unknown PID — no interference
- Iris stations recognize PID 0xBE and decode with native framing
- Standard AX.25 UI frames (PID 0xF0) continue to work for legacy interop

#### Result
- Iris <-> Direwolf: works, AX.25 1200/9600, XID gets FRMR, stays legacy
- Iris <-> Iris: starts AX.25, XID succeeds, auto-upgrades to native
- Upgrade is transparent to the application layer (KISS interface unchanged)
- No channel pollution, no SSID hacks, standards-compliant

### What Auto-Upgrade Enables
| | AX.25 1200 | AX.25 9600 | Iris Native Mode A |
|---|---|---|---|
| Throughput | ~1 kbps | ~7 kbps | ~17 kbps |
| FEC | None | None | LDPC |
| Adaptive | No | No | BPSK→256QAM |
| Encryption | No | No | ChaCha20 |
| Compression | No | No | PPMd+zstd |
| Weak signal | Dies | Dies | Gearshift down |

## Implementation Plan

### What to reuse from Mercury (C++)
- LDPC encoder/decoder (all code rates)
- ARQ state machine (commander/responder)
- Compression engine (PPMd8 + zstd streaming)
- Encryption (Monocypher, key exchange)
- B2F unroll/reroll
- TCP control/data socket interface
- GUI framework (Qt)
- CLI argument parsing
- Audio I/O (WASAPI/ALSA abstraction)

### What to build new
- AFSK 1200 baud modem (Bell 202, clean-room from spec)
- GFSK 9600 baud modem (G3RUH, clean-room from original paper)
- AX.25 HDLC framing, NRZI encoding, KISS TNC interface
- XID-based auto-upgrade negotiation (AX.25 -> Iris native)
- Single-carrier modulator (RRC pulse shaping) for native mode
- Single-carrier demodulator (matched filter + timing recovery)
- Carrier/phase recovery (Costas/PLL)
- Frame sync (correlator on preamble/sync word)
- AGC
- SNR estimator (preamble residual)
- FM radio TX/RX turnaround timing
- PTT control (CAT/CI-V, VOX, GPIO, CM108 HID)

### Estimated Complexity
- AX.25 layer (AFSK/GFSK modem + HDLC + KISS): ~3000-4000 lines
- Iris native PHY (single-carrier): ~2000-3000 lines
- Auto-upgrade negotiation: ~500 lines
- Shared code from Mercury (ARQ, compression, encryption): ~15000 lines (already written)
- Total new code: ~6000-8000 lines

### Build Approach
Fork Mercury, replace PHY layer, keep everything above. Shared ARQ/compression/
encryption code stays in-tree. If both projects diverge significantly, extract
shared code into a library later.

## Audio Interface

### PTT Control
- CAT/CI-V serial commands (preferred, lowest latency)
- VOX (fallback, add TX preamble tone for VOX activation)
- GPIO (Raspberry Pi, embedded)
- CM108 HID (SignaLink, Digirig)

### TX/RX Turnaround
- FM radios have significant TX/RX switching time (~50-200ms)
- ARQ timing must account for this (Mercury assumes ~10ms for SSB)
- Configurable turnaround delay per radio model
- Burst mode: longer transmissions with more data per turn to amortize overhead

### Audio Levels
- TX: configurable deviation control (important for FM — overdeviation = splatter)
- RX: AGC handles variable levels from discriminator

## Frequency Plan

Primary target: VHF/UHF amateur bands where FM data is permitted.
- 2m: 144.900-145.100 MHz (US), or local packet frequencies
- 70cm: 430-440 MHz region
- Works on any FM channel — amateur, GMRS, commercial (with appropriate license)

## Platform Support
- Windows (WASAPI/DirectSound)
- Linux (ALSA/PulseAudio)
- macOS (CoreAudio)
- Raspberry Pi (ALSA + GPIO PTT)

## Decisions Log

| Question | Decision | Rationale |
|----------|----------|-----------|
| Mode C modulation | Single-carrier + FSE | VHF multipath 2-10% of symbol; 5-tap equalizer sufficient |
| Duplex | Deferred to v2 | Doubles audio complexity, ARQ assumes half-duplex |
| Voice/M17 | No | Iris is data-only; M17 handles voice well |
| Winlink/Pat/BPQ32 | Via KISS TNC | All speak KISS; no special integration needed |
| Direwolf DSP reuse | Clean-room | Direwolf GPLv2 incompatible with AGPL-3.0 |
| Config format | Own INI/TOML | Don't clone Direwolf config; KISS interface is what matters |
| License | AGPL-3.0 | Matches Mercury, shared code compatible |
| Language | C++ | Shared codebase with Mercury |
| XID format | Fixed 8-byte | Magic + version + caps bitmask + max modulation |
| Upgrade beacon | Reserved PID 0xBE | Legacy TNCs silently drop unknown PID |

## Future (v2+)
- Full duplex through repeater pairs (simultaneous TX/RX, no turnaround penalty)
- OFDM option for Mode C if FSE proves insufficient in urban environments
- Codec2 voice+data multiplexing if demand exists
- Mesh/digipeater mode with Iris-native routing
