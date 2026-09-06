#!/usr/bin/env python3
"""
iris_connect_tax_decompose.py - decompose the Iris connect-tax into stages from a
honest-baseline session evidence dir (iris_cmd.log + kiss_sender.json + kiss_receiver.json).

Every IRIS_LOG line is prefixed with a monotonic [ssss.mmm] elapsed-seconds stamp
(include/common/logging.h:54), so the connect-tax stages can be decomposed straight from
the INITIATOR (CMD) log without any added instrumentation.  The stage boundaries are the
real modem markers:

  connect (AFSK SABM/UA)   "AX25 KISS SABM to"        -> "RX UA ... -> CONNECTED"
  probe   (chirp BW probe) "Probe-after-connect"      -> "Probe complete: band"
  tune    (TX-gain cal)    "[CONNDIET] probe clean="  -> "[TUNE] Skipped"/"WAIT_PEER done"
                           (0 s when connect-diet skips it on a clean high-SNR probe)
  activate(OFDM native)    "Probe complete"           -> "OFDM PHY: prepared"
  climb   (O0 -> steady)   "OFDM PHY: prepared"        -> first "[TX-OFDM] speed=O<steady>"
                           and, wall-anchored, RX first-verified-byte (kiss_receiver.json)

The connect_probe_climb total = RX first_verified_byte_epoch - sender first_sabm_tx_epoch
(the runner's own line-item), split across the stages above by the CMD-log monotonic deltas.

Usage:  iris_connect_tax_decompose.py <evidence_dir> [<evidence_dir> ...]
        iris_connect_tax_decompose.py --json <dir>       # machine-readable
Bash, not PowerShell.
"""
import json
import os
import re
import sys

TS = re.compile(r"^\[\s*(\d+\.\d+)\]")


def _first(lines, *needles):
    """First (ts, line) whose text contains ANY needle, in log order."""
    for ts, txt in lines:
        if any(n in txt for n in needles):
            return ts, txt
    return None, None


def _load_cmd_lines(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = TS.match(line)
            if m:
                out.append((float(m.group(1)), line.rstrip("\n")))
    return out


def decompose(evdir):
    cmd = _load_cmd_lines(os.path.join(evdir, "iris_cmd.log"))
    snd = _load_json(os.path.join(evdir, "kiss_sender.json"))
    rcv = _load_json(os.path.join(evdir, "kiss_receiver.json"))
    d = {"dir": os.path.basename(evdir.rstrip("/\\"))}
    if not cmd:
        d["error"] = "no timestamped iris_cmd.log"
        return d

    t_sabm, _ = _first(cmd, "AX25 KISS SABM to", "KISS SABM to")
    t_conn, _ = _first(cmd, "-> CONNECTED", "RX UA")
    t_probe_start, _ = _first(cmd, "Probe-after-connect", "Manual probe: sending tones")
    t_probe_done, _ = _first(cmd, "Probe complete: band")
    t_clean, ln_clean = _first(cmd, "[CONNDIET] probe clean=")
    t_tune_skip, _ = _first(cmd, "[TUNE] Skipped")
    t_tune_done, _ = _first(cmd, "[TUNE] WAIT_PEER done", "[TUNE] Test frames sent")
    t_ofdm, _ = _first(cmd, "OFDM PHY: prepared")
    t_first_tx, _ = _first(cmd, "[TX-OFDM] speed=O")

    def dt(a, b):
        return round(b - a, 2) if (a is not None and b is not None) else None

    d["log_markers_s"] = {
        "sabm": t_sabm, "connected": t_conn, "probe_start": t_probe_start,
        "probe_done": t_probe_done, "conndiet_decision": t_clean,
        "tune_skipped": t_tune_skip, "tune_done": t_tune_done,
        "ofdm_prepared": t_ofdm, "first_ofdm_tx": t_first_tx,
    }
    d["stages_s"] = {
        "afsk_connect": dt(t_sabm, t_conn),
        "probe": dt(t_probe_start, t_probe_done),
        "tune": 0.0 if t_tune_skip is not None else dt(t_clean, t_tune_done),
        "ofdm_activate": dt(t_probe_done, t_ofdm),
        "climb_to_first_ofdm_tx": dt(t_ofdm, t_first_tx),
    }
    d["tune_skipped_clean_probe"] = t_tune_skip is not None
    if ln_clean:
        m = re.search(r"rev_snr=([\-\d.]+) dB \(>= ([\-\d.]+)\)", ln_clean)
        if m:
            d["probe_rev_snr_db"] = float(m.group(1))
            d["tune_skip_gate_db"] = float(m.group(2))

    # wall-anchored connect+probe+climb line-item (the runner's own definition).
    t0 = (snd or {}).get("first_sabm_tx_epoch")
    w = (rcv or {}).get("wall") or {}
    fv = w.get("first_verified_byte_epoch")
    d["connect_probe_climb_secs"] = round(fv - t0, 2) if (t0 and fv) else None
    d["delivered_bytes"] = (rcv or {}).get("delivered_bytes")
    return d


def _load_json(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def main(argv):
    as_json = False
    dirs = []
    for a in argv:
        if a == "--json":
            as_json = True
        else:
            dirs.append(a)
    if not dirs:
        print(__doc__)
        return 2
    results = [decompose(d) for d in dirs]
    if as_json:
        print(json.dumps(results, indent=2))
        return 0
    for r in results:
        print(f"\n=== {r['dir']} ===")
        if r.get("error"):
            print("  ", r["error"]); continue
        st = r["stages_s"]
        print(f"  connect(AFSK)={st['afsk_connect']}s  probe={st['probe']}s  "
              f"tune={st['tune']}s  ofdm_activate={st['ofdm_activate']}s  "
              f"climb->first_tx={st['climb_to_first_ofdm_tx']}s")
        print(f"  tune_skipped(clean_probe)={r.get('tune_skipped_clean_probe')}  "
              f"rev_snr={r.get('probe_rev_snr_db')}dB gate={r.get('tune_skip_gate_db')}dB")
        print(f"  connect_probe_climb_secs(wall)={r.get('connect_probe_climb_secs')}  "
              f"delivered_bytes={r.get('delivered_bytes')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
