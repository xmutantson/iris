#!/usr/bin/env python3
# Apply the SAME MPG/WGN Watterson channel the live real-audio bridge uses
# (tools/sim/sim_channel_relay.py:Channel) to a raw S16LE mono@48k OFDM frame,
# deterministically by seed. This is the channel stage of the offline
# capture-replay reproduction (see README.md).
#   usage: chan_apply.py tx.s16 rx.s16 <profile:wgn|mpg|mpm|mpp> <snr3k_db> <seed>
#   env:   IRIS_SIM_DIR  dir containing sim_channel_relay.py
#          (default /dev/shm/iris_matrix/tools/sim, else <repo>/tools/sim)
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
out = []
CH = 1024                     # process in bridge-sized chunks (Doppler/analytic evolve)
for i in range(0, len(x), CH):
    out.extend(ch.process(x[i:i + CH]))
y = np.clip(np.asarray(out, dtype=np.float64), -1.0, 1.0)
(y * 32767.0).astype('<i2').tofile(rxf)
