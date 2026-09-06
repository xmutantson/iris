#!/usr/bin/env python3
# FROZEN-TAP channel fork (fact doc O0O1_MPG_DECODE_ROOT.md §10.2): apply the
# same 2-tap Watterson geometry as chan_apply.py — same per-seed Rayleigh
# draws, same dtau delay, same AWGN — but with the taps FROZEN at their
# construction values (fd forced to 0 after init). Isolates the intra-frame
# tap WALK from the static multipath geometry:
#   frozen ~100% while walking fails  ->  the walk (channel-model Doppler /
#   H-staleness) is the killer, NOT notch depth / tap order / FEC capacity.
# 2026-07-04: this fork proved the "MPG:40 decode boss" was the one-pole-IIR
# Doppler infidelity in tools/sim/sim_channel_relay.py (frozen 40/40 vs
# walking 24/40 at 60 dB; faithful Gaussian-FIR tap also 40/40).
#   usage: freeze_apply.py tx.s16 rx.s16 <profile:mpg|mpm|mpp> <snr3k_db> <seed>
#   env:   IRIS_SIM_DIR  dir containing sim_channel_relay.py
import sys, os, numpy as np, types
simdir = os.environ.get("IRIS_SIM_DIR", "/dev/shm/iris_matrix/tools/sim")
if not os.path.exists(os.path.join(simdir, "sim_channel_relay.py")):
    simdir = os.path.join(os.path.dirname(__file__), "..", "sim")
sys.path.insert(0, simdir)
from sim_channel_relay import Channel  # noqa: E402

txf, rxf, profile, snr, seed = (sys.argv[1], sys.argv[2], sys.argv[3],
                                float(sys.argv[4]), int(sys.argv[5]))
x = np.fromfile(txf, dtype='<i2').astype(np.float64) / 32768.0
rms = float(np.sqrt(np.mean(x[np.abs(x) > 0.01] ** 2))) if np.any(np.abs(x) > 0.01) else 0.1
a = types.SimpleNamespace(snr=snr, snr_schedule=None, loss=0.0, burst=0,
                          profile=profile, cfo_hz=0.0, phase_noise_deg=0.0, sig_ref=rms)
ch = Channel(a, seed)
# FREEZE: DopplerTap.advance() holds its current gain when fd <= 0 (both the
# legacy IIR and the Gaussian-FIR implementations honor this).
ch.tap0.fd = 0.0
ch.tap1.fd = 0.0
out = []
CH = 1024
for i in range(0, len(x), CH):
    out.extend(ch.process(x[i:i + CH]))
y = np.clip(np.asarray(out, dtype=np.float64), -1.0, 1.0)
(y * 32767.0).astype('<i2').tofile(rxf)
