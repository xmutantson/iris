# Iris — Open Source FM Data Modem

## Overview

High-throughput ARQ data modem for VHF/UHF FM channels. Open-source replacement
for VARA FM. Single-carrier design optimized for the flat, high-SNR FM channel.

Named as companion to Mercury (HF OFDM modem): Mercury carries messages via
skywave, Iris carries them line-of-sight.

License: AGPL-3.0 (matches Mercury, shared code compatible)
Language: C++17

## Design Philosophy

- FM channels are flat and stable — no need for OFDM
- Single-carrier with pulse shaping is simpler, more efficient, lower latency
- Cherry-pick algorithms from Mercury (LDPC, compression, encryption) — don't fork
- Three transport modes for maximum radio compatibility
- Adaptive modulation from BPSK to 256-QAM based on link quality
- AX.25 compatibility layer for interop with existing packet network
- Transition path: talk to legacy stations today, upgrade to native Iris when both sides have it
- Clean-room AFSK/GFSK implementation (no Direwolf code — GPLv2 incompatible with AGPL)

## Channel Modes

### Mode A — Audio-Coupled (Mic/Speaker)
- Works with any FM radio through audio jacks (including USB audio)
- Usable bandwidth: ~2800 Hz (300-3100 Hz)
- Symbol rate: 2400 baud (RRC alpha=0.2 -> 2880 Hz occupied)
- Sample rate: 48000 Hz
- Target: portable/handheld radios, maximum compatibility
- Latency: ~100ms TX/RX turnaround (VOX or CAT PTT)
- Note: USB audio interfaces (e.g., IC-705) give flatter response than analog

### Mode B — 9600 Baud Port (Discriminator/Direct)
- Flat baseband access, bypasses audio filtering
- Usable bandwidth: ~4800 Hz
- Symbol rate: 4800 baud
- Sample rate: 48000 Hz
- Target: base stations, TNC-ready radios (Kenwood TM-D710, IC-9700, etc.)

### Mode C — Channel-Direct/SDR (Full Channel)
- Full FM channel bandwidth via SDR or direct modulation
- 12.5 kHz channel: symbol rate 9600 baud
- 25 kHz channel: symbol rate 19200 baud
- Sample rate: 96000 Hz (required for 19200 baud)
- Target: SDR setups (PlutoSDR, LimeSDR, HackRF), digital-native radios
- Single-carrier with fractionally-spaced equalizer (not OFDM — VHF multipath
  is only 2-10% of symbol duration, a 5-tap FSE handles it)

## Modulation & Coding

### Modulation Ladder (square QAM only)

| Level | Modulation | Bits/sym | Min SNR (approx) |
|-------|------------|----------|-------------------|
| 0     | BPSK       | 1        | 3 dB              |
| 1     | QPSK       | 2        | 6 dB              |
| 2     | 16QAM      | 4        | 14 dB             |
| 3     | 64QAM      | 6        | 22 dB             |
| 4     | 256QAM     | 8        | 30 dB             |

Note: 8PSK, 32QAM, 128QAM deliberately omitted. Odd-bit constellations add
implementation complexity (non-square Gray coding) for marginal gain. SNR gaps
between levels are better covered by varying FEC code rate.

### LDPC Code Rates
Extracted from Mercury/codec2: 1/2, 3/4, 7/8.
Combined with modulation level for fine-grained throughput control.

### Speed Levels — Mode A (2400 baud, 48 kHz)

| Level | Mod    | FEC Rate | Net bps | Min SNR |
|-------|--------|----------|---------|---------|
| A0    | BPSK   | 1/2      | 1,200   | ~3 dB   |
| A1    | QPSK   | 1/2      | 2,400   | ~6 dB   |
| A2    | QPSK   | 3/4      | 3,600   | ~9 dB   |
| A3    | 16QAM  | 1/2      | 4,800   | ~14 dB  |
| A4    | 16QAM  | 3/4      | 7,200   | ~17 dB  |
| A5    | 64QAM  | 3/4      | 10,800  | ~22 dB  |
| A6    | 64QAM  | 7/8      | 12,600  | ~25 dB  |
| A7    | 256QAM | 7/8      | 16,800  | ~30 dB  |

Mode B = 2x Mode A rates (4800 baud). Mode C 12.5kHz = 4x (9600 baud).
Mode C 25kHz = 8x (19200 baud, requires 96 kHz sample rate).

Peak: 256QAM 7/8 on 25 kHz Mode C = **134 kbps**.

### Comparison
| | Direwolf 1200 | Direwolf 9600 | VARA FM | Iris AX.25 | Iris Native A |
|---|---|---|---|---|---|
| Peak throughput | ~1 kbps | ~7 kbps | ~25 kbps | ~7 kbps | ~17 kbps |
| FEC | None | None | Yes | None | LDPC |
| Adaptive | No | No | Yes | No | BPSK->256QAM |
| Encryption | No | No | No | No | ChaCha20 |
| Compression | No | No | No | No | PPMd+zstd |
| AX.25 compat | Yes | Yes | No | Yes | Auto-upgrade |
| Open source | Yes | Yes | No | Yes | Yes |

## Physical Layer

### Pulse Shaping
- Root-raised-cosine (RRC) filter, rolloff alpha = 0.2
- Matched filter at receiver
- Zero ISI at optimal sample point

### Mode A Audio Upconversion
- Native PHY produces complex baseband IQ
- Mode A upconverts to real audio: I*cos(wt) - Q*sin(wt) at 1800 Hz center
- Receiver downconverts back to complex baseband before matched filter
- Modes B/C pass baseband directly (no carrier needed)

### Synchronization
- Preamble: known BPSK training sequence (63-bit m-sequence)
- Timing recovery: Gardner TED with interpolation
- Carrier recovery: decision-directed PLL for high-order QAM
- AGC: RMS-based with fast attack / slow decay

### Frame Structure

```
[Preamble 63sym] [Sync Word 16sym] [Header 32sym BPSK] [Payload Nsym] [CRC32]
```

- **Preamble**: m-sequence, always BPSK, used for timing/carrier sync + SNR estimate
- **Sync word**: frame delimiter, Barker-like sequence
- **Header**: modulation level, code rate, payload length, frame type — always BPSK
  so receiver can decode regardless of payload modulation
- **Payload**: variable modulation/coding per header, LDPC-encoded
- **CRC32**: error detection on decoded payload

### Channel Estimation
- Preamble-based SNR measurement (no per-subcarrier estimate needed — flat channel)
- Decision-directed tracking for phase/amplitude drift during payload

### Equalization
- Simple 1-tap complex gain correction (flat channel, Modes A/B)
- 5-tap fractionally-spaced linear equalizer for Mode C (urban VHF multipath)

### FM Deviation Control
- TX audio level directly controls FM deviation — critical for FM modems
- Over-deviation causes adjacent channel splatter and distortion
- Under-deviation wastes SNR
- Auto level calibration: two-station procedure (like VARA FM)
  1. Station A transmits known test tone at configured level
  2. Station B measures received deviation and reports back
  3. Station A adjusts TX level to target optimal deviation
  4. Roles reverse for Station B calibration
- Stored per-radio-profile in config file
- Manual override always available

## Data Link Layer

### ARQ Protocol
- Commander/Responder architecture (concepts from Mercury)
- Selective repeat ARQ with NACK
- Adaptive batch sizing

### Gearshift
- SNR-based rate adaptation
- Start at lowest reliable level, climb based on measured SNR
- Probe higher levels periodically
- Fall back immediately on CRC failure
- Hysteresis prevents oscillation at level boundaries

### Features
- PPMd8 + zstd streaming compression with capability negotiation
- X25519 + ChaCha20-Poly1305 encryption (Monocypher vendored)
- B2F unroll/reroll for Winlink
- SSID addressing

### Connection Sequence
1. HAIL (BPSK, always decodable)
2. Capability negotiation (compression, encryption, B2F)
3. Key exchange (if encrypted)
4. Gearshift to optimal modulation
5. Data transfer with adaptive rate

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
- KISS TNC interface on TCP port 8001
- Compatible with Winlink, APRS, BPQ32, Pat, and any KISS-aware application

### Auto-Upgrade Negotiation
When two Iris stations connect, they can upgrade to native mode mid-session
using XID frames (connected mode) and reserved PID bytes (connectionless).

#### Connected Mode
1. Station A sends normal AX.25 SABM — works with any TNC
2. Connection established in standard AX.25
3. Station A sends XID frame with Iris capability parameters
4. If peer responds with XID: both sides switch to Iris native framing
5. If peer responds with FRMR or silence: stay in AX.25 mode
6. If native link drops, fall back to AX.25 automatically

#### XID Capability Frame Format
```
XID Information Field (8 bytes, fixed format):
  Bytes 0-3: Magic "IRIS" (0x49 0x52 0x49 0x53)
  Byte 4:    Version (0x01 = v1)
  Bytes 5-6: Capabilities bitmask (big-endian)
    bit 0: Native Mode A supported
    bit 1: Native Mode B supported
    bit 2: Native Mode C supported
    bit 3: Encryption capable (ChaCha20)
    bit 4: Compression capable (PPMd+zstd)
    bit 5: B2F unroll capable
  Byte 7:    Max modulation level (0-4, maps to modulation ladder)
```

#### Connectionless Mode (APRS, UI frames)
- Use reserved PID byte (0xBE) for Iris-native UI frames
- Legacy TNCs silently drop frames with unknown PID — no interference
- Standard AX.25 UI frames (PID 0xF0) continue to work for legacy interop

## Implementation Architecture

### Code Organization (standalone C++17 project)
```
iris/
  include/
    common/types.h         - Constants, types
    ax25/                   - CRC16, HDLC, AFSK, GFSK
    kiss/                   - KISS codec
    native/                 - RRC, constellation, PHY, frame, XID
    fec/                    - LDPC encoder/decoder
    audio/                  - Audio I/O abstraction
    radio/                  - rigctl/PTT control
    engine/                 - Modem engine, gearshift, ARQ
    gui/                    - Dear ImGui interface
    config/                 - INI config parser
  source/
    (mirrors include/ structure)
  third_party/
    imgui/                  - Dear ImGui
    monocypher/             - Encryption library
  build.sh
```

### What to extract from Mercury (algorithm-level, not fork)
- LDPC parity check matrices and decoder algorithm
- Compression approach (PPMd8 + zstd streaming pattern)
- Encryption approach (Monocypher X25519 + ChaCha20)
- Gearshift concept (SNR-based adaptation)
- Audio I/O pattern (WASAPI/ALSA abstraction)

### What to build new
- Everything above as clean C++17 with clear interfaces
- Dear ImGui GUI (constellation, waterfall, SNR meter, config)
- rigctl integration for PTT
- KISS TCP server
- Auto level calibration
- FM-specific TX/RX turnaround timing
- Speed level management

## GUI (Dear ImGui)

### Panels
- **Main**: Connect/disconnect, callsign, mode selection, speed level
- **Constellation**: Real-time scatter plot of received symbols
- **Waterfall/Spectrum**: Audio spectrum display
- **SNR Meter**: Bar graph with history, current speed level indicator
- **Config**: Audio device selection, radio model, PTT method, TX level
- **Calibration**: Auto level set wizard (two-station procedure)
- **Log**: Scrolling text log of events, decoded frames

### Radio Control
- rigctl (hamlib) via TCP or direct serial for PTT and frequency
- VOX fallback (add TX preamble tone)
- CM108 HID for SignaLink/Digirig

## Platform Support
- Windows (WASAPI audio, Dear ImGui DirectX/OpenGL backend)
- Linux (ALSA audio, Dear ImGui OpenGL/SDL backend)
- macOS (CoreAudio, Dear ImGui Metal/OpenGL backend)

## Decisions Log

| Question | Decision | Rationale |
|----------|----------|-----------|
| Mode C modulation | Single-carrier + FSE | VHF multipath 2-10% of symbol; 5-tap equalizer sufficient |
| Mode C sample rate | 96 kHz required | 19200 baud needs SPS >= 5; 48kHz only gives 2.5 |
| Duplex | Deferred to v2 | Doubles audio complexity, ARQ assumes half-duplex |
| Voice/M17 | No | Iris is data-only; M17 handles voice well |
| Odd QAM (8PSK etc) | Dropped | Non-square Gray coding complexity; use FEC rate variation instead |
| Mercury code reuse | Extract algorithms, don't fork | Architectures too different; clean C++17 interfaces |
| Direwolf DSP reuse | Clean-room | GPLv2 incompatible with AGPL-3.0 |
| Config format | Own INI | Simple, human-readable |
| License | AGPL-3.0 | Matches Mercury |
| GUI | Dear ImGui | Lightweight, single binary, good for SDR-style displays |
| XID format | Fixed 8-byte | Magic + version + caps bitmask + max modulation |
| Upgrade beacon | Reserved PID 0xBE | Legacy TNCs silently drop unknown PID |
| Auto level cal | Two-station procedure | Like VARA FM; essential for FM deviation control |

## Future (v2+)
- Full duplex through repeater pairs
- OFDM option for Mode C if FSE proves insufficient in urban environments
- Codec2 voice+data multiplexing if demand exists
- Mesh/digipeater mode with Iris-native routing
