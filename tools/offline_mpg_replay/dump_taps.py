#!/usr/bin/env python3
# Per-seed Watterson tap-magnitude dump (fact doc §10.1 P0 tool): mirrors the
# EXACT Channel construction + 1024-sample chunk loop chan_apply.py uses (same
# numpy tap RNG order), but records |g0|,|g1| instead of writing audio. Used
# to correlate tap order/depth with the per-seed decode ledger.
#   usage: dump_taps.py tx.s16 <profile> <snr3k_db> <nseeds>
#   out:   seed  |g0|,|g1| mean over the signal extent + over the sync region,
#          and the g1/g0 ratio in dB for each window.
import sys, os, types, numpy as np
simdir = os.environ.get("IRIS_SIM_DIR", "/dev/shm/iris_matrix/tools/sim")
if not os.path.exists(os.path.join(simdir, "sim_channel_relay.py")):
    simdir = os.path.join(os.path.dirname(__file__), "..", "sim")
sys.path.insert(0, simdir)
from sim_channel_relay import Channel  # noqa: E402

txf, profile, snr, nseeds = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
x = np.fromfile(txf, dtype='<i2').astype(np.float64) / 32768.0
rms = float(np.sqrt(np.mean(x[np.abs(x) > 0.01] ** 2))) if np.any(np.abs(x) > 0.01) else 0.1
nz = np.where(np.abs(x) > 0.01)[0]
s0, s1 = (int(nz[0]), int(nz[-1]) + 1) if len(nz) else (0, len(x))
CH = 1024
print("seed g0_sig g1_sig g0_sync g1_sync rdb_sig rdb_sync")
for seed in range(1, nseeds + 1):
    a = types.SimpleNamespace(snr=snr, snr_schedule=None, loss=0.0, burst=0,
                              profile=profile, cfo_hz=0.0, phase_noise_deg=0.0, sig_ref=rms)
    ch = Channel(a, seed)
    g0s, g1s = [], []
    for i in range(0, len(x), CH):
        n = len(x[i:i + CH])
        g0s.append(np.abs(ch.tap0.advance(n) * ch.tap_scale))
        g1s.append(np.abs(ch.tap1.advance(n) * ch.tap_scale))
    g0 = np.concatenate(g0s); g1 = np.concatenate(g1s)
    sync_hi = min(s0 + 3 * 1088, s1)
    g0sig, g1sig = float(g0[s0:s1].mean()), float(g1[s0:s1].mean())
    g0sy, g1sy = float(g0[s0:sync_hi].mean()), float(g1[s0:sync_hi].mean())
    print(f"{seed} {g0sig:.4f} {g1sig:.4f} {g0sy:.4f} {g1sy:.4f} "
          f"{20*np.log10(g1sig/max(g0sig,1e-12)):+.2f} "
          f"{20*np.log10(g1sy/max(g0sy,1e-12)):+.2f}")
