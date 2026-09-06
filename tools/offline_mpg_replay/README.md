# Offline MPG decode reproduction / A/B harness

Deterministic, ALSA-free, connect-free reproduction of the O0/O1 MPG:40 forward
decode failure. It feeds a known OFDM frame through the **same** MPG Watterson
channel the live real-audio bridge uses (`tools/sim/sim_channel_relay.py:Channel`)
and decodes it with the production demod — so a demod change can be A/B'd on
IDENTICAL waveforms, free of connect/ARQ variance and snd-aloop card wedges.

## Pipeline
1. `iris --tx-ofdm <L> tx.s16`  — build a known-payload OFDM frame (S16LE 48k mono).
2. `chan_apply.py tx.s16 rx.s16 mpg <snr3k> <seed>`  — apply the MPG channel.
3. `iris --rx-ofdm <L> rx.s16`  — sync + demod + LDPC; prints `RESULT: PASS|FAIL`.

Levels (`--tx/rx-ofdm <L>`): 0=O0 BPSK r1/2, 1=O1 QPSK r1/2, 2=O2 QPSK r3/4, …

## Match the LIVE config (important)
The live probe negotiates a **BW=4150 Hz, 87-carrier** OFDM band and a ~94-byte
(21-symbol) O1 frame. Reproduce it with:

```
export IRIS_OFDM_PB_LOW=300 IRIS_OFDM_PB_HIGH=5000   # -> 87 carriers
export IRIS_OFDM_PAYLOAD=94                           # -> ~21-symbol O1 frame
export BASE=/path/to/base/iris FIX=/path/to/fix/iris
bash ab_offline.sh 1 42.4 40                          # O1, SNR3k=42.4 (MPG:40 realized), 40 seeds
```

The default narrow 300–3000 band gives only 57 carriers — the WRONG regime.

## Key measured result (2026-07-04, RESOLVED)
- The old "MPG 42.4/60 dB = 50-57%" boss was the one-pole-IIR Doppler
  infidelity in `tools/sim/sim_channel_relay.py` (intra-frame tap walk ~30x
  the ITU/IONOS Gaussian Watterson): frozen-tap fork (`freeze_apply.py`) =
  **40/40**, faithful Gaussian-FIR tap = **40/40 @42.4 and @60, O0 and O1**.
  The Gaussian tap is now the relay default (workspace branch
  sim/watterson-gaussian-doppler), so plain `chan_apply.py` = faithful arena.
- Receiver fixes shipped default-ON on the way (first-path sync timing +
  white-noise nv, `ofdm_sync.cc`): frozen late-tap-lock EsNo 3.3-9.0 →
  16.8-20.7 dB; WGN +1.0 dB; `--test` 653/0.
- P0/diagnostic tools: `dump_taps.py` (per-seed |g0|,|g1| vs the ledger),
  `freeze_apply.py` (walk-vs-geometry fork).
