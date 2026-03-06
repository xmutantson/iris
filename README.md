# Iris — Open Source FM Data Modem

High-throughput data modem for VHF/UHF FM channels. Open-source replacement for VARA FM with AX.25 backward compatibility.

**Companion to [Mercury](https://github.com/Rhizomatica/mercury)** (HF OFDM modem): Mercury carries messages via skywave, Iris carries them line-of-sight.

## Features

- **AX.25 compatible**: Drop-in Direwolf replacement — works with Winlink, APRS, BPQ32, Pat
- **Auto-upgrade**: Two Iris stations transparently negotiate native mode via AX.25 XID frames
- **Adaptive modulation**: BPSK to 256-QAM with LDPC FEC, SNR-based gearshift
- **Three transport modes**: audio-coupled (any radio), 9600 baud port, SDR/direct
- **Encryption**: X25519 + ChaCha20-Poly1305 end-to-end
- **Compression**: PPMd8 + zstd streaming

## Throughput

| Mode | AX.25 Legacy | Iris Native |
|------|-------------|-------------|
| Mode A (audio, any radio) | 1200 bps | up to 17 kbps |
| Mode B (9600 port) | 9600 bps | up to 34 kbps |
| Mode C (SDR, 25 kHz) | — | up to 134 kbps |

## How It Works

Iris speaks standard AX.25 by default. When it detects another Iris station (via XID capability exchange), both sides transparently upgrade to native Iris framing with LDPC forward error correction and adaptive modulation. Applications using the KISS TNC interface don't need to change — the upgrade happens below the application layer.

```
Your App (Winlink, Pat, APRS)
       |
   KISS TNC (TCP :8001)
       |
  +----+----+
  |         |
AX.25     Iris Native
(legacy)  (auto-upgrade)
```

## Status

Early development. See [DESIGN.md](DESIGN.md) for the full technical specification.

## Building

```bash
# Coming soon
```

## License

AGPL-3.0 — see [LICENSE](LICENSE)
