#!/usr/bin/env python3
"""
test_honest_baseline_integration.py - producer-evidence-binding integration self-test
for the Iris honest-baseline harness (evidence binding + regression follow-up).

The runner's own --self-test drives gate_session/metric_session with SYNTHESISED
fixtures.  The gap this test closes: a LIVE run would push REAL pump-emitted manifests
and a REAL bridge statsfile -- artifacts the fixtures could drift from.  This test
therefore produces GENUINE evidence and runs it through the ACTUAL gate + metric:

  * REAL pump manifests: full KISS-pump sessions over an in-process, perfect
    AX.25-transparent KISS loopback (raw byte forward between the sender/receiver
    client sockets -- no Iris, no fleet, no ALSA), each writing a genuine
    kiss_sender.json + kiss_receiver.json exactly as the fleet would:
      - CORPUS run  (--payload-file corpus.json): the 4 b2f_nodict bodies fly and
        hash-verify -> a clean, corpus-BOUND number.
      - SYNTHETIC run (no --payload-file): the legacy synthetic_NNNNNN fallback flies
        -> the "synthetic-name zero" the metric used to score as a silent 0.0.
      - INTERRUPTED run (SIGTERM mid-session): proves the pump persists its manifest
        incrementally + on teardown (FIX-C).
  * REAL bridge statsfile: `realaudio_bridge_s32.py --dry-run --cell MPG:15` writes a
    genuine [C1] channel_attestation, extracted through the runner's OWN
    _bridge_c1_from_stats (so the folded-in underruns measured field is present).

It asserts the fixed harness on the ORIGINAL holes:
  A. binds the wire-parity numerator to the bytes that flew and anchors the wall on
     the SENDER's first_sabm_tx_epoch, with a populated connect/probe/climb line-item;
  B. converts a synthetic-name delivery into a LOUD HARNESS_INVALID
     (CORPUS_UNKNOWN_MESSAGE), never a gate-clean structural 0.0;
and the regression follow-up holes (each fails at the pre-fix HEAD, passes after):
  A2 (FIX-A) sender manifest ABSENT => HARNESS_INVALID (SENDER_MANIFEST_MISSING),
     never a silent RX-fallback score;
  BB (FIX-B) a DUPLICATE verified record => NO double-count (metric == deduped);
  C  (FIX-C) a SIGTERM mid-session => BOTH manifests preserved (incremental/atexit);
  D1 (FIX-D) a wrong-SNR C1 (realized_snr_offset_db out of tolerance) => REFUSED;
  D2 (FIX-D) a CROSS-BOX binary mismatch => REFUSED (BINARY_MANIFEST_MISMATCH).

The SECOND regression pass (two surviving low-SNR-reach holes) — each fails at the
pre-fix HEAD, passes after:
  E1 (FIX-1) a fast-connect 1-of-4 PARTIAL is scored over the FULL SESSION wall, so it
     is <= a complete slow-connect delivery and CANNOT top the published distribution
     (at HEAD its 306 B over a ~2.5 s truncated wall scores ~7344 B/min and inflates);
  E2 (FIX-2) a genuine CONNECT-FAILURE (real pump vs a deaf KISS listener) persists
     first_sabm_tx_epoch and scores an HONEST 0.0 (MODEM_FAULT) INTO the distribution +
     counted in activation_rate (at HEAD first_sabm_tx_epoch is None => withheld
     HARNESS_INVALID); a TRULY-ABSENT sender manifest still stays HARNESS_INVALID;
  E3 (FIX-2) a SIGTERM DURING connect (before it completes) still persists
     first_sabm_tx_epoch (the worst-moment case-C flagged by the verify pass).

The THIRD regression pass (the recurring partial-inflation ROOT, made robust BY
CONSTRUCTION: one connect-inclusive wall, partial denominator from the PLAN not the
manifest) — each fails at the pre-fix HEAD, passes after:
  F1 a NON-PREFIX verified set {ics213,short_email,batch3} (net_checkin corrupted-early)
     is scored over the connect-inclusive PLAN wall, so it is <= a SLOW complete delivery
     and CANNOT top the distribution (at HEAD it scores 680.8 > a 636 B/min slow complete
     because the partial wall omitted the connect airtime the complete wall included);
  F2 a sender manifest with NO 'duration' field still uses the PLAN wall (no truncated
     last-verified revert): a net_checkin-only partial scores the plan-anchored ~47 B/min,
     never the 7344 B/min the missing field reverted it to at HEAD.

FAIL-AT-HEAD / PASS-AFTER: point IHB_RUNNER + IHB_PUMP at the pre-fix (git HEAD)
copies of the runner + pump and this test FAILS (the new cases exercise gate codes /
runner functions / pump flush behaviour that do not exist at HEAD).  Against the fixed
working tree it PASSES.  Standalone: exit 0 == PASS, 1 == FAIL.

  IHB_RUNNER  override path to iris_honest_baseline_runner.py (default: alongside)
  IHB_PUMP    override path to kiss_data_pump.py (default: outer tools/rpi_scripts)

No PowerShell; Bash-friendly.  Runs on any host with python3 + numpy (bridge dry-run).
"""
import copy
import importlib.util
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
IRIS_REPO = os.path.dirname(HERE)
WORKSPACE = os.path.dirname(IRIS_REPO)

RUNNER_PATH = os.environ.get("IHB_RUNNER", os.path.join(HERE, "iris_honest_baseline_runner.py"))
PUMP_PATH = os.environ.get("IHB_PUMP",
                           os.path.join(WORKSPACE, "tools", "rpi_scripts", "kiss_data_pump.py"))
BRIDGE_PATH = os.path.join(WORKSPACE, "tools", "sim", "realaudio", "realaudio_bridge_s32.py")
CORPUS_JSON = os.path.join(HERE, "corpus", "corpus.json")


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# In-process perfect AX.25-transparent KISS loopback (raw byte forwarder).
# The pump sender + receiver are ordinary KISS TCP CLIENTS; this stands in for the
# whole Iris+air+Iris path with a lossless cable so we exercise the REAL pump codec
# and manifest writer end-to-end.
# ---------------------------------------------------------------------------
def _make_listener(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(1)
    s.settimeout(30.0)
    return s


def _forward(srv_a, srv_b, stop):
    try:
        conn_a, _ = srv_a.accept()
        conn_b, _ = srv_b.accept()
    except OSError:
        return
    conn_a.setblocking(False)
    conn_b.setblocking(False)
    import select
    peers = {conn_a: conn_b, conn_b: conn_a}
    try:
        while not stop.is_set():
            r, _, _ = select.select([conn_a, conn_b], [], [], 0.2)
            for s in r:
                try:
                    data = s.recv(65536)
                except (BlockingIOError, InterruptedError):
                    continue
                except OSError:
                    return
                if not data:
                    return                       # a side closed -> tear down
                try:
                    peers[s].sendall(data)
                except OSError:
                    return
    finally:
        for c in (conn_a, conn_b):
            try:
                c.close()
            except OSError:
                pass


def _pump_argv(mode, callsign, port, out, duration, payload_file, remote=None,
               connect_timeout=30):
    argv = [sys.executable, PUMP_PATH, "--mode", mode, "--callsign", callsign,
            "--kiss-port", str(port), "--duration", str(duration),
            "--connect-timeout", str(connect_timeout), "--output", out]
    if remote:
        argv += ["--remote", remote]
    if payload_file:
        argv += ["--payload-file", payload_file]
    return argv


def run_sender_vs_deaf_listener(port, workdir, connect_timeout, kill_after_s=None,
                                duration=30):
    """Launch ONLY the sender pump against a listener that ACCEPTS the KISS TCP
    connection but NEVER replies with a UA, so the sender transmits its FIRST SABM and
    either (E2) connect() FAILS after connect_timeout, or (E3) is SIGTERM'd mid-connect
    (kill_after_s, well before connect_timeout).  Returns the kiss_sender.json path.

    This is the FIX-2 producer-evidence probe: it proves the pump persists
    first_sabm_tx_epoch the instant the first SABM flies -- even though the connect
    never completes -- so a genuine connect-failure carries a valid TX-anchored wall."""
    srv = _make_listener(port)
    stop = threading.Event()

    def _sink():
        try:
            conn, _ = srv.accept()
        except OSError:
            return
        conn.setblocking(False)
        while not stop.is_set():
            try:
                conn.recv(65536)          # drain the SABM bytes; NEVER reply with a UA
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                break
            time.sleep(0.1)
        try:
            conn.close()
        except OSError:
            pass

    t = threading.Thread(target=_sink, daemon=True)
    t.start()
    send_json = os.path.join(workdir, "kiss_sender.json")
    send = subprocess.Popen(
        _pump_argv("sender", "IRISCMD", port, send_json, duration, None,
                   remote="IRISRSP", connect_timeout=connect_timeout),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        if kill_after_s is not None:
            time.sleep(kill_after_s)
            if send.poll() is None:
                try:
                    send.send_signal(signal.SIGTERM)
                except (OSError, ValueError):
                    send.terminate()
            try:
                send.wait(timeout=20)
            except subprocess.TimeoutExpired:
                send.kill()
        else:
            try:
                send.wait(timeout=connect_timeout + 30)
            except subprocess.TimeoutExpired:
                send.kill()
    finally:
        stop.set()
        t.join(timeout=5)
        try:
            srv.close()
        except OSError:
            pass
    return send_json


def _rx_manifest(R, delivered_names, sabm_rx, first_verified, last_verified):
    """A REAL-schema RX (receiver) C2 manifest for arbitrary delivered corpus names +
    epochs (mirrors kiss_data_pump.build_delivered_manifest).  per_message carries the
    checked-in b2f_nodict bytes + sha256 so the gate's corpus-integrity binding passes."""
    by = {m["name"]: m for m in R.CORPUS}
    per = [{"name": nm, "sha256": by[nm]["sha256"], "verified": True,
            "bytes": by[nm]["b2f_nodict"]} for nm in delivered_names]
    dbytes = sum(by[nm]["b2f_nodict"] for nm in delivered_names)
    return {"mode": "receiver", "delivered_msg_names": list(delivered_names),
            "delivered_bytes": dbytes, "per_message": per,
            "wall": {"sabm_rx_epoch": sabm_rx,
                     "first_verified_byte_epoch": first_verified,
                     "last_verified_byte_epoch": last_verified}}


def _sender_manifest(first_sabm_tx, duration=300):
    """A REAL-schema TX (sender) manifest: TX wall anchor (first_sabm_tx_epoch).  A real
    pump also writes a 'duration' field, but the ROOT FIX makes the metric wall INDEPENDENT
    of it (the partial/zero wall comes from the PLAN); pass duration=None to omit the field
    entirely and prove a missing duration no longer reverts to a truncated wall (F2)."""
    m = {"mode": "sender", "first_sabm_tx_epoch": first_sabm_tx}
    if duration is not None:
        m["duration"] = duration
    return m


def run_pump_pair(port_a, port_b, payload_file, duration, workdir):
    """Bridge port_a<->port_b, launch a REAL receiver + sender pump SUBPROCESS, and
    return (sender_manifest, receiver_manifest) loaded from the genuine JSON files."""
    srv_a = _make_listener(port_a)      # sender (CMD) side
    srv_b = _make_listener(port_b)      # receiver (RSP) side
    stop = threading.Event()
    t = threading.Thread(target=_forward, args=(srv_a, srv_b, stop), daemon=True)
    t.start()

    send_json = os.path.join(workdir, "kiss_sender.json")
    recv_json = os.path.join(workdir, "kiss_receiver.json")

    # Responder listens first (mirrors the runner sequencing), then the initiator hails.
    recv = subprocess.Popen(_pump_argv("receiver", "IRISRSP", port_b, recv_json,
                                       duration, payload_file),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    send = subprocess.Popen(_pump_argv("sender", "IRISCMD", port_a, send_json,
                                       duration, payload_file, remote="IRISRSP"),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        send.wait(timeout=120)
        recv.wait(timeout=120)
    finally:
        for p in (send, recv):
            if p.poll() is None:
                p.kill()
        stop.set()
        t.join(timeout=5)
        for s in (srv_a, srv_b):
            try:
                s.close()
            except OSError:
                pass

    with open(send_json, "r", encoding="utf-8") as f:
        sender_manifest = json.load(f)
    with open(recv_json, "r", encoding="utf-8") as f:
        receiver_manifest = json.load(f)
    return sender_manifest, receiver_manifest


def run_pump_pair_interrupted(port_a, port_b, duration, workdir, kill_after_s=5.0):
    """FIX-C: bridge a CONTINUOUS (synthetic, no --payload-file) session, then SIGTERM
    BOTH pumps mid-session (well before `duration`).  Returns (send_json, recv_json)
    paths WITHOUT loading them -- the caller asserts the files exist + are well-formed,
    proving the manifest was persisted incrementally / on SIGTERM rather than only at
    main() end (which a mid-session kill would lose)."""
    srv_a = _make_listener(port_a)
    srv_b = _make_listener(port_b)
    stop = threading.Event()
    t = threading.Thread(target=_forward, args=(srv_a, srv_b, stop), daemon=True)
    t.start()

    send_json = os.path.join(workdir, "kiss_sender.json")
    recv_json = os.path.join(workdir, "kiss_receiver.json")

    recv = subprocess.Popen(_pump_argv("receiver", "IRISRSP", port_b, recv_json,
                                       duration, None),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    send = subprocess.Popen(_pump_argv("sender", "IRISCMD", port_a, send_json,
                                       duration, None, remote="IRISRSP"),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Let the session connect + deliver a few records (multiple incremental flushes),
    # then SIGTERM both ends MID-session (SIGTERM on POSIX; TerminateProcess on Windows,
    # where the incremental writes alone must have created the files).
    time.sleep(kill_after_s)
    for p in (send, recv):
        if p.poll() is None:
            try:
                p.send_signal(signal.SIGTERM)
            except (OSError, ValueError):
                p.terminate()
    try:
        for p in (send, recv):
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.kill()
    finally:
        stop.set()
        t.join(timeout=5)
        for s in (srv_a, srv_b):
            try:
                s.close()
            except OSError:
                pass
    return send_json, recv_json


def bridge_dry_stats(R, workdir, cell="MPG:15", profile="mpg", seed=1):
    """REAL bridge --dry-run statsfile carrying a genuine [C1] channel_attestation,
    extracted through the runner's OWN _bridge_c1_from_stats so the folded-in
    underruns measured field (FIX-D(1)) is present exactly as a live run would see it."""
    stats = os.path.join(workdir, "bridge_stats.json")
    subprocess.run([sys.executable, BRIDGE_PATH, "--dry-run", "--cell", cell,
                    "--profile", profile, "--seed", str(seed), "--statsfile", stats],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(stats, "r", encoding="utf-8") as f:
        return R._bridge_c1_from_stats(json.load(f))


# ---------------------------------------------------------------------------
# Evidence assembly (real manifests + real attestation + real modem-printf logs).
# ---------------------------------------------------------------------------
def _real_marker_logs():
    """A (cmd_log, rsp_log) pair rendered from the ACTUAL modem.cc printf formats the
    gate regexes target (symmetric OFDM activation)."""
    def side():
        return "\n".join([
            "Native hail: native mode active",
            "Probe complete: band 300-2700 Hz (2400 Hz BW), center 1500 Hz, baud 800",
            "[OFDM-NEG] negotiated: cp=64 pilot=3 block=13 nfft=1024",
            "OFDM PHY: prepared, 31 carriers (25 data, 6 pilot), CP=64, BW=2344 Hz",
            "[TX-OFDM] speed=O2 QAM16 fec=3/4, 269 bytes, 7 cw/frame, 25 carriers, 4 bits/sym",
        ]) + "\n"
    return side(), side()


def _spec():
    # session_secs is the raw data window; the FIXED runner derives the connect-inclusive
    # planned session wall (session_secs + SESSION_WALL_OVERHEAD_S) that a partial/zero
    # delivery is scored over.  Harmless at the pre-fix HEAD (which ignores it and reads the
    # sender-manifest duration instead).
    return {"tag": "MPG15_doff_s00", "cell": "MPG:15", "channel": "MPG", "snr": 15,
            "cohort": "off", "metric": "wire_parity", "profile": "mpg", "seed": 1,
            "session_secs": 300}


def _bin_manifest():
    md5 = "a1b2c3d4" * 4
    head = "d126bb9" + "0" * 33
    return {"CMD": {"md5": md5, "git_head": head}, "RSP": {"md5": md5, "git_head": head}}


def _evidence(cmd_log, rsp_log, attest, receiver_manifest, sender_manifest):
    return {"cmd_log": cmd_log, "rsp_log": rsp_log,
            "launch_cmdlines": [["iris", "--native-hail", "--alsa-raw"]],
            "bridge_attestation": attest,
            "pump_manifest": receiver_manifest,
            "sender_manifest": sender_manifest,
            "binary_manifest": _bin_manifest()}


def main():
    R = _load_module("ihb_runner", RUNNER_PATH)
    fails = []
    cmd_log, rsp_log = _real_marker_logs()
    spec = _spec()
    # summary fields (filled as cases run)
    summ = {}

    with tempfile.TemporaryDirectory(prefix="ihb_integ_") as td:
        attest = bridge_dry_stats(R, td)
        print(f"[integration] bridge C1 attestation: {attest}")

        # ==== A. CORPUS run: real bodies fly, bind numerator + anchor wall on sender ====
        wa = os.path.join(td, "corpus")
        os.makedirs(wa, exist_ok=True)
        snd_a, rcv_a = run_pump_pair(8731, 8732, CORPUS_JSON, duration=30, workdir=wa)
        print(f"[integration] CORPUS sender first_sabm_tx_epoch="
              f"{snd_a.get('first_sabm_tx_epoch')} "
              f"receiver delivered={rcv_a.get('delivered_msg_names')} "
              f"wall={rcv_a.get('wall')}")
        ev_a = _evidence(cmd_log, rsp_log, attest, rcv_a, snd_a)
        row_a = R.score_session(spec, ev_a)
        gate_a = row_a["gate"]
        met_a = row_a.get("metric") or {}

        if gate_a["outcome"] != "SCORED":
            fails.append(f"A: gate outcome {gate_a['outcome']} != SCORED "
                         f"(violations={[v['code'] for v in gate_a['violations']]})")
        if row_a.get("outcome") != "SCORED":
            fails.append(f"A: session outcome {row_a.get('outcome')} != SCORED")
        if met_a.get("value_Bmin") is None or met_a.get("value_Bmin", 0) <= 0:
            fails.append(f"A: metric value {met_a.get('value_Bmin')} not a positive number "
                         f"(numerator unbound / wall broken)")
        # HOLE 2: the wall must be anchored on the SENDER's first_sabm_tx_epoch.
        if met_a.get("wall_anchor") != "sender_first_sabm_tx":
            fails.append(f"A: wall_anchor {met_a.get('wall_anchor')} != sender_first_sabm_tx "
                         f"(HOLE 2: wall not anchored on the sender manifest)")
        # phantom reconciled: cpc line-item is populated from RX first_verified_byte_epoch.
        if met_a.get("connect_probe_climb_secs") is None:
            fails.append("A: connect_probe_climb_secs is None (first_verified_byte_epoch "
                         "phantom NOT reconciled -- pump did not emit it / runner did not read it)")
        planned = {m["name"] for m in R.CORPUS}
        delivered = set(rcv_a.get("delivered_msg_names") or [])
        if delivered != planned:
            fails.append(f"A: delivered set {sorted(delivered)} != planned {sorted(planned)} "
                         f"(loopback under-delivered -- test harness fault)")
        summ["A_corpus"] = {"gate": gate_a["outcome"], "outcome": row_a.get("outcome"),
                            "value_Bmin": met_a.get("value_Bmin"),
                            "wall_anchor": met_a.get("wall_anchor"),
                            "connect_probe_climb_secs": met_a.get("connect_probe_climb_secs")}

        # ==== A2 (FIX-A). sender manifest ABSENT -> loud HARNESS_INVALID ====
        # At HEAD the wall silently falls back to the RX SABM arrival and metric_session
        # SCORES an inflated number; the fixed gate refuses it as HARNESS_INVALID.
        ev_a2 = _evidence(cmd_log, rsp_log, attest, rcv_a, None)
        row_a2 = R.score_session(spec, ev_a2)
        codes_a2 = {v["code"] for v in row_a2["gate"]["violations"]}
        if row_a2["gate"]["outcome"] != "HARNESS_INVALID":
            fails.append(f"A2: sender-absent gate {row_a2['gate']['outcome']} != HARNESS_INVALID "
                         f"(silent RX-fallback score -- FIX-A not closed)")
        if "SENDER_MANIFEST_MISSING" not in codes_a2:
            fails.append(f"A2: expected SENDER_MANIFEST_MISSING, got {sorted(codes_a2)}")
        if row_a2.get("outcome") == "SCORED":
            fails.append(f"A2: session SCORED {row_a2.get('value_Bmin')} on a MISSING sender "
                         f"manifest (inflated RX-fallback wall -- FIX-A not closed)")
        summ["A2_sender_absent"] = {"gate": row_a2["gate"]["outcome"],
                                    "violations": sorted(codes_a2)}

        # ==== BB (FIX-B). DUPLICATE verified record must NOT double-count ====
        met_clean = R.metric_session(spec, rcv_a, snd_a)
        rcv_dup = copy.deepcopy(rcv_a)
        # a duplicate verified record for a name already delivered (same bytes+sha):
        first_pm = dict(rcv_dup["per_message"][0])
        rcv_dup["per_message"].append(first_pm)
        # the gate still SCORES it (validates names as a SET) ...
        gate_dup = R.gate_session(spec, _evidence(cmd_log, rsp_log, attest, rcv_dup, snd_a))
        if gate_dup["outcome"] != "SCORED":
            fails.append(f"BB: duplicate-record gate {gate_dup['outcome']} != SCORED "
                         f"(the duplicate should pass the SET gate)")
        # ... but the metric must credit UNIQUE names once (no inflation).
        met_dup = R.metric_session(spec, rcv_dup, snd_a)
        if met_dup.get("value_Bmin") != met_clean.get("value_Bmin"):
            fails.append(f"BB: duplicate verified record inflated the metric "
                         f"{met_clean.get('value_Bmin')} -> {met_dup.get('value_Bmin')} "
                         f"(FIX-B double-count not closed)")
        summ["BB_duplicate"] = {"gate": gate_dup["outcome"],
                                "value_clean": met_clean.get("value_Bmin"),
                                "value_dup": met_dup.get("value_Bmin")}

        # ==== C (FIX-C). SIGTERM mid-session -> BOTH manifests preserved ====
        wc = os.path.join(td, "sigterm")
        os.makedirs(wc, exist_ok=True)
        send_json, recv_json = run_pump_pair_interrupted(8751, 8752, duration=30,
                                                         workdir=wc, kill_after_s=5.0)
        for role, path in (("sender", send_json), ("receiver", recv_json)):
            if not os.path.exists(path) or os.path.getsize(path) == 0:
                fails.append(f"C: {role} manifest {os.path.basename(path)} absent/empty after "
                             f"a mid-session SIGTERM (FIX-C: pump did not flush "
                             f"incrementally / on SIGTERM)")
                continue
            try:
                with open(path, "r", encoding="utf-8") as f:
                    m = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                fails.append(f"C: {role} manifest not well-formed JSON after SIGTERM ({e})")
                continue
            if m.get("mode") != role:
                fails.append(f"C: {role} manifest mode={m.get('mode')} != {role}")
            if role == "sender" and m.get("first_sabm_tx_epoch") is None:
                fails.append("C: sender manifest missing first_sabm_tx_epoch after SIGTERM "
                             "(connected but wall-anchor not persisted)")
            if role == "receiver" and (m.get("wall") or {}).get("sabm_rx_epoch") is None:
                fails.append("C: receiver manifest missing wall.sabm_rx_epoch after SIGTERM")
        summ["C_sigterm"] = {
            "sender_exists": os.path.exists(send_json) and os.path.getsize(send_json) > 0,
            "receiver_exists": os.path.exists(recv_json) and os.path.getsize(recv_json) > 0}

        # ==== D1 (FIX-D). wrong-SNR C1 (offset out of tolerance) -> REFUSED ====
        bad_c1 = dict(attest)
        bad_c1["realized_snr_offset_db"] = 8.0     # bridge ran a non-standard dial mapping
        gate_d1 = R.gate_session(spec, _evidence(cmd_log, rsp_log, bad_c1, rcv_a, snd_a))
        codes_d1 = {v["code"] for v in gate_d1["violations"]}
        if gate_d1["outcome"] != "HARNESS_INVALID":
            fails.append(f"D1: wrong-SNR C1 gate {gate_d1['outcome']} != HARNESS_INVALID "
                         f"(measured C1 fields ungated -- FIX-D(1) not closed)")
        if "ATTEST_SNR_OFFSET_OUT_OF_TOL" not in codes_d1:
            fails.append(f"D1: expected ATTEST_SNR_OFFSET_OUT_OF_TOL, got {sorted(codes_d1)}")
        summ["D1_wrong_snr"] = {"gate": gate_d1["outcome"], "violations": sorted(codes_d1)}

        # ==== D2 (FIX-D). CROSS-BOX binary mismatch -> REFUSED ====
        if not hasattr(R, "collect_binary_manifest"):
            fails.append("D2: R.collect_binary_manifest missing (per-session cross-box "
                         "attestation not implemented -- FIX-D(2) not closed)")
            summ["D2_cross_box"] = {"error": "collect_binary_manifest missing"}
        else:
            per_box = {"11": ("aa" * 16, "dead" + "0" * 36),
                       "21": ("bb" * 16, "beef" + "0" * 36)}

            def fake_ssh(box, remote_cmd, timeout=60):
                md5, head = per_box[box]
                return (0, f"MD5 {md5}\nHEAD {head}\n", "")

            binmani = R.collect_binary_manifest("/dev/shm/iris/iris", cmd_box="11",
                                                rsp_box="21", iris_repo_on_box="~/iris",
                                                ssh=fake_ssh)
            if binmani["CMD"] == binmani["RSP"]:
                fails.append("D2: cross-box attestation returned identical CMD/RSP "
                             "(tautological -- FIX-D(2) not effective)")
            ev_d2 = _evidence(cmd_log, rsp_log, attest, rcv_a, snd_a)
            ev_d2["binary_manifest"] = binmani
            gate_d2 = R.gate_session(spec, ev_d2)
            codes_d2 = {v["code"] for v in gate_d2["violations"]}
            if gate_d2["outcome"] != "HARNESS_INVALID":
                fails.append(f"D2: cross-box-mismatch gate {gate_d2['outcome']} != HARNESS_INVALID")
            if "BINARY_MANIFEST_MISMATCH" not in codes_d2:
                fails.append(f"D2: expected BINARY_MANIFEST_MISMATCH, got {sorted(codes_d2)}")
            summ["D2_cross_box"] = {"gate": gate_d2["outcome"],
                                    "cmd_head": binmani["CMD"].get("git_head"),
                                    "rsp_head": binmani["RSP"].get("git_head"),
                                    "violations": sorted(codes_d2)}

        # ==== B. SYNTHETIC run: the synthetic-name delivery -> loud HARNESS_INVALID ====
        wb = os.path.join(td, "synth")
        os.makedirs(wb, exist_ok=True)
        snd_b, rcv_b = run_pump_pair(8741, 8742, None, duration=5, workdir=wb)
        synth_names = rcv_b.get("delivered_msg_names") or []
        print(f"[integration] SYNTHETIC receiver delivered {len(synth_names)} msg(s): "
              f"{synth_names[:3]}{'...' if len(synth_names) > 3 else ''}")
        if not synth_names:
            fails.append("B: synthetic run delivered nothing (loopback fault; cannot test "
                         "the synthetic-name zero)")
        ev_b = _evidence(cmd_log, rsp_log, attest, rcv_b, snd_b)
        gate_b = R.gate_session(spec, ev_b)
        codes_b = {v["code"] for v in gate_b["violations"]}
        if gate_b["outcome"] != "HARNESS_INVALID":
            fails.append(f"B: gate outcome {gate_b['outcome']} != HARNESS_INVALID "
                         f"(the synthetic-name delivery was NOT caught -> silent structural 0.0)")
        if "CORPUS_UNKNOWN_MESSAGE" not in codes_b:
            fails.append(f"B: expected CORPUS_UNKNOWN_MESSAGE, got {sorted(codes_b)}")
        # And prove the OLD failure mode: at HEAD this same evidence scores a clean 0.0.
        row_b = R.score_session(spec, ev_b)
        if row_b.get("outcome") == "SCORED" and (row_b.get("value_Bmin") == 0.0):
            fails.append("B: session SCORED a structural 0.0 on synthetic delivery "
                         "(scoring-on-absence inside the metric -- HOLE 1 not closed)")
        summ["B_synthetic"] = {"gate": gate_b["outcome"], "violations": sorted(codes_b)}

        # ==== E1 (FIX-1). partial-delivery inflation: a fast-connect 1-of-4 partial
        #      must be scored over the FULL SESSION wall, never the truncated
        #      last-verified wall (else it out-ranks a complete slow delivery). ====
        t0 = 1000.0
        # complete SLOW-connect: all 4 corpus messages (3710 B) verified 70 s after the
        # TX anchor -> 3710 B over 70 s == 3180.0 B/min (its TRUE finish).
        rx_complete = _rx_manifest(R, [m["name"] for m in R.CORPUS],
                                   sabm_rx=t0 + 6, first_verified=t0 + 20,
                                   last_verified=t0 + 70)
        snd_complete = _sender_manifest(t0, duration=300)
        ev_complete = _evidence(cmd_log, rsp_log, attest, rx_complete, snd_complete)
        row_complete = R.score_session(spec, ev_complete)
        # fast-connect PARTIAL: net_checkin only (306 B), last verified 2.5 s after the
        # TX anchor.  At HEAD this scores 306 B over 2.5 s == 7344 B/min (inflated);
        # after FIX-1 it is scored over the 300 s full session == 61.2 B/min.
        rx_partial = _rx_manifest(R, ["net_checkin"], sabm_rx=t0 + 0.5,
                                  first_verified=t0 + 1.5, last_verified=t0 + 2.5)
        snd_partial = _sender_manifest(t0, duration=300)
        ev_partial = _evidence(cmd_log, rsp_log, attest, rx_partial, snd_partial)
        row_partial = R.score_session(spec, ev_partial)
        met_complete = row_complete.get("metric") or {}
        met_partial = row_partial.get("metric") or {}
        vc = met_complete.get("value_Bmin")
        vp = met_partial.get("value_Bmin")
        print(f"[integration] E1 complete={vc} B/min partial={vp} B/min "
              f"(partial wall_end={met_partial.get('wall_end')})")
        if row_complete.get("outcome") != "SCORED":
            fails.append(f"E1: complete delivery outcome {row_complete.get('outcome')} != SCORED")
        if row_partial.get("outcome") != "MODEM_FAULT":
            fails.append(f"E1: partial delivery outcome {row_partial.get('outcome')} != MODEM_FAULT "
                         f"(a strict-subset delivery is CORPUS_INCOMPLETE)")
        if vp is None or vc is None:
            fails.append(f"E1: metric values missing (partial={vp}, complete={vc})")
        elif vp > vc:
            fails.append(f"E1: partial {vp} B/min > complete {vc} B/min -- truncated-wall "
                         f"inflation NOT closed (FIX-1)")
        if met_partial.get("wall_end") != "full_session_wall":
            fails.append(f"E1: partial wall_end {met_partial.get('wall_end')} != "
                         f"full_session_wall (partial not scored over the full session -- FIX-1)")
        # the partial must NOT top the published distribution (max stays the complete).
        rows_e1 = [R.score_session(spec, ev_complete) for _ in range(3)] + [row_partial]
        agg_e1 = R.aggregate_cell("MPG", 15, "off", rows_e1)
        dmax = (agg_e1.get("distribution_Bmin") or {}).get("max")
        if dmax is None:
            fails.append("E1: distribution max is None (aggregate withheld a clean cell)")
        elif vp is not None and vc is not None and dmax > vc:
            fails.append(f"E1: distribution max {dmax} > complete {vc} -- the partial "
                         f"({vp}) topped the published distribution (FIX-1 not closed)")
        summ["E1_partial_inflation"] = {"complete_Bmin": vc, "partial_Bmin": vp,
                                        "partial_wall_end": met_partial.get("wall_end"),
                                        "dist_max": dmax,
                                        "partial_outcome": row_partial.get("outcome")}

        # ==== E2 (FIX-2). a connect-FAILURE scores an honest 0.0 (MODEM_FAULT) INTO the
        #      distribution + counts in activation_rate; a TRULY-ABSENT sender manifest
        #      stays HARNESS_INVALID. ====
        we = os.path.join(td, "connfail")
        os.makedirs(we, exist_ok=True)
        # real sender pump vs a deaf listener -> connect() FAILS after ~8 s; the pump
        # must have persisted first_sabm_tx_epoch when its first SABM flew (FIX-2).
        snd_cf_path = run_sender_vs_deaf_listener(8761, we, connect_timeout=8)
        with open(snd_cf_path, "r", encoding="utf-8") as f:
            snd_cf = json.load(f)
        print(f"[integration] E2 connect-fail first_sabm_tx_epoch="
              f"{snd_cf.get('first_sabm_tx_epoch')} error={snd_cf.get('error')} "
              f"connected_time={snd_cf.get('connected_time')}")
        if snd_cf.get("first_sabm_tx_epoch") is None:
            fails.append("E2: connect-fail sender manifest first_sabm_tx_epoch is None -- the "
                         "pump did NOT persist the TX anchor on connect failure (FIX-2 not "
                         "closed) -> the gate would withhold this cell as HARNESS_INVALID")
        if snd_cf.get("error") is None:
            fails.append("E2: connect-fail sender manifest has no error (the deaf-listener "
                         "connect should have FAILED; test scenario invalid)")
        # a connect failure delivers nothing and never activates OFDM (AFSK-only logs).
        afsk_log = "AX.25 connected; staying AFSK\n"
        rx_zero = _rx_manifest(R, [], sabm_rx=None, first_verified=None, last_verified=None)
        ev_cf = _evidence(afsk_log, afsk_log, attest, rx_zero, snd_cf)
        row_cf = R.score_session(spec, ev_cf)
        codes_cf = {v["code"] for v in row_cf["gate"]["violations"]}
        if "SENDER_MANIFEST_MISSING" in codes_cf:
            fails.append("E2: connect-fail wrongly raised SENDER_MANIFEST_MISSING -- the TX "
                         "anchor IS present, so this is a MODEM fault, not a harness fault")
        if row_cf["gate"]["outcome"] != "MODEM_FAULT":
            fails.append(f"E2: connect-fail gate {row_cf['gate']['outcome']} != MODEM_FAULT "
                         f"(honest zero into the distribution; violations={sorted(codes_cf)})")
        if row_cf.get("outcome") != "MODEM_FAULT" or row_cf.get("value_Bmin") != 0.0:
            fails.append(f"E2: connect-fail row outcome={row_cf.get('outcome')} "
                         f"value={row_cf.get('value_Bmin')} != MODEM_FAULT/0.0")
        # it must land IN the distribution and be counted in activation_rate.
        rows_e2 = [R.score_session(spec, ev_complete) for _ in range(3)] + [row_cf]
        agg_e2 = R.aggregate_cell("MPG", 15, "off", rows_e2)
        if agg_e2["n_in_distribution"] != 4:
            fails.append(f"E2: n_in_distribution {agg_e2['n_in_distribution']} != 4 "
                         f"(connect-failure not counted into the distribution)")
        if agg_e2["n_modem_fault"] != 1 or agg_e2["n_harness_invalid"] != 0:
            fails.append(f"E2: n_modem_fault={agg_e2['n_modem_fault']} "
                         f"n_harness_invalid={agg_e2['n_harness_invalid']} "
                         f"(connect-failure should be a MODEM fault, not withheld)")
        # 3 activated / 4 in-distribution = 0.75 -- the connect-failure LOWERS activation.
        if agg_e2["activation_rate"] != 0.75:
            fails.append(f"E2: activation_rate {agg_e2['activation_rate']} != 0.75 "
                         f"(connect-failure not in the activation-rate denominator)")
        # a TRULY-ABSENT sender manifest (pump never ran) stays HARNESS_INVALID.
        gate_absent = R.gate_session(spec, _evidence(afsk_log, afsk_log, attest, rx_zero, None))
        codes_absent = {v["code"] for v in gate_absent["violations"]}
        if gate_absent["outcome"] != "HARNESS_INVALID":
            fails.append(f"E2: truly-absent sender gate {gate_absent['outcome']} != HARNESS_INVALID")
        if "SENDER_MANIFEST_MISSING" not in codes_absent:
            fails.append(f"E2: absent-sender expected SENDER_MANIFEST_MISSING, got {sorted(codes_absent)}")
        summ["E2_connect_fail"] = {"first_sabm_tx_epoch": snd_cf.get("first_sabm_tx_epoch"),
                                   "gate": row_cf["gate"]["outcome"],
                                   "value_Bmin": row_cf.get("value_Bmin"),
                                   "activation_rate": agg_e2["activation_rate"],
                                   "n_in_distribution": agg_e2["n_in_distribution"],
                                   "absent_sender_gate": gate_absent["outcome"]}

        # ==== E3 (FIX-2). a SIGTERM DURING connect still persists first_sabm_tx_epoch ====
        wcc = os.path.join(td, "connfail_sigterm")
        os.makedirs(wcc, exist_ok=True)
        # long connect-timeout so the sender is STILL connecting when SIGTERM lands at +4 s.
        snd_sig_path = run_sender_vs_deaf_listener(8771, wcc, connect_timeout=40,
                                                   kill_after_s=4.0)
        if not os.path.exists(snd_sig_path) or os.path.getsize(snd_sig_path) == 0:
            fails.append("E3: sender manifest absent/empty after a SIGTERM during connect "
                         "(the TX-anchor flush did not land pre-connect -- FIX-2)")
            summ["E3_sigterm_during_connect"] = {"error": "manifest absent/empty"}
        else:
            with open(snd_sig_path, "r", encoding="utf-8") as f:
                snd_sig = json.load(f)
            print(f"[integration] E3 SIGTERM-during-connect first_sabm_tx_epoch="
                  f"{snd_sig.get('first_sabm_tx_epoch')} connected_time="
                  f"{snd_sig.get('connected_time')} teardown={snd_sig.get('teardown')}")
            if snd_sig.get("mode") != "sender":
                fails.append(f"E3: manifest mode {snd_sig.get('mode')} != sender")
            if snd_sig.get("first_sabm_tx_epoch") is None:
                fails.append("E3: first_sabm_tx_epoch is None after a SIGTERM DURING connect "
                             "-- the TX anchor must be persisted the instant the first SABM "
                             "flies, before connect completes (FIX-2 not closed)")
            if snd_sig.get("connected_time") is not None:
                fails.append("E3: connected_time is set -- the SIGTERM did not land DURING "
                             "connect (deaf listener should never complete the connect; "
                             "test scenario invalid)")
            summ["E3_sigterm_during_connect"] = {
                "first_sabm_tx_epoch": snd_sig.get("first_sabm_tx_epoch"),
                "connected_time": snd_sig.get("connected_time"),
                "teardown": snd_sig.get("teardown")}

        # ==== F1 (ROOT). a NON-PREFIX verified set is scored over the connect-inclusive
        #      PLAN wall, so it is <= a SLOW complete and CANNOT top the distribution.
        #      At HEAD the partial wall omitted the connect airtime the complete wall
        #      included, letting {ics213,short_email,batch3}=3404 B score 680.8 B/min over
        #      the sender-manifest duration (300 s) and out-rank a 636 B/min slow complete. ====
        plan_wall = getattr(R, "plan_session_wall_secs", None)
        t0f = 2000.0
        # SLOW complete: all 4 (3710 B) verified 350 s after the TX anchor -> 636.0 B/min
        # (its TRUE finish; 350 s <= the fixed plan wall 390 s, so the invariant holds).
        rx_f_complete = _rx_manifest(R, [m["name"] for m in R.CORPUS], sabm_rx=t0f + 6,
                                     first_verified=t0f + 30, last_verified=t0f + 350)
        snd_f_complete = _sender_manifest(t0f, duration=300)
        ev_f_complete = _evidence(cmd_log, rsp_log, attest, rx_f_complete, snd_f_complete)
        row_f_complete = R.score_session(spec, ev_f_complete)
        # NON-PREFIX partial: net_checkin (the FIRST message) corrupted/sha-fails while the
        # later three verify -> {ics213,short_email,batch3}=3404 B, last byte early (t0+25).
        rx_f_partial = _rx_manifest(R, ["ics213", "short_email", "batch3"], sabm_rx=t0f + 6,
                                    first_verified=t0f + 15, last_verified=t0f + 25)
        snd_f_partial = _sender_manifest(t0f, duration=300)
        ev_f_partial = _evidence(cmd_log, rsp_log, attest, rx_f_partial, snd_f_partial)
        row_f_partial = R.score_session(spec, ev_f_partial)
        vfc = (row_f_complete.get("metric") or {}).get("value_Bmin")
        vfp = (row_f_partial.get("metric") or {}).get("value_Bmin")
        wefp = (row_f_partial.get("metric") or {}).get("wall_end")
        print(f"[integration] F1 non-prefix complete={vfc} B/min partial={vfp} B/min "
              f"(partial wall_end={wefp})")
        if row_f_complete.get("outcome") != "SCORED":
            fails.append(f"F1: complete outcome {row_f_complete.get('outcome')} != SCORED")
        if row_f_partial.get("outcome") != "MODEM_FAULT":
            fails.append(f"F1: non-prefix partial outcome {row_f_partial.get('outcome')} "
                         f"!= MODEM_FAULT (a strict subset is CORPUS_INCOMPLETE)")
        if vfp is None or vfc is None:
            fails.append(f"F1: metric values missing (partial={vfp}, complete={vfc})")
        elif vfp > vfc:
            fails.append(f"F1: NON-PREFIX partial {vfp} B/min > slow complete {vfc} B/min -- "
                         f"the partial wall omitted the connect airtime the complete wall "
                         f"included (ROOT not closed)")
        if wefp != "full_session_wall":
            fails.append(f"F1: partial wall_end {wefp} != full_session_wall")
        # the non-prefix partial must NOT top the published distribution.
        rows_f1 = [R.score_session(spec, ev_f_complete) for _ in range(3)] + [row_f_partial]
        agg_f1 = R.aggregate_cell("MPG", 15, "off", rows_f1)
        dmax_f1 = (agg_f1.get("distribution_Bmin") or {}).get("max")
        if dmax_f1 is None:
            fails.append("F1: distribution max is None (aggregate withheld a clean cell)")
        elif vfc is not None and dmax_f1 > vfc:
            fails.append(f"F1: distribution max {dmax_f1} > slow complete {vfc} -- the "
                         f"non-prefix partial ({vfp}) topped the published distribution (ROOT)")
        if plan_wall is not None:                 # exact-value check on the fixed runner
            want_vfp = round(3404 / (plan_wall(spec) / 60.0), 1)
            if vfp != want_vfp:
                fails.append(f"F1: partial value {vfp} != {want_vfp} (3404 B over the "
                             f"{plan_wall(spec):.0f} s connect-inclusive plan wall)")
        summ["F1_nonprefix_partial"] = {"complete_Bmin": vfc, "partial_Bmin": vfp,
                                        "partial_wall_end": wefp, "dist_max": dmax_f1,
                                        "partial_outcome": row_f_partial.get("outcome")}

        # ==== F2 (ROOT). a sender manifest with NO 'duration' field still uses the PLAN
        #      wall -- NOT a truncated last-verified revert.  At HEAD a missing duration
        #      dropped the partial to the else-branch (t1 = last_verified), scoring a
        #      net_checkin-only partial (306 B) over ~2.5 s == 7344 B/min again. ====
        t0g = 3000.0
        rx_g_complete = _rx_manifest(R, [m["name"] for m in R.CORPUS], sabm_rx=t0g + 6,
                                     first_verified=t0g + 30, last_verified=t0g + 350)
        snd_g_complete = _sender_manifest(t0g, duration=300)
        ev_g_complete = _evidence(cmd_log, rsp_log, attest, rx_g_complete, snd_g_complete)
        # fast-connect net_checkin-only partial, sender manifest carries NO duration field.
        rx_g_partial = _rx_manifest(R, ["net_checkin"], sabm_rx=t0g + 0.5,
                                    first_verified=t0g + 1.5, last_verified=t0g + 2.5)
        snd_g_partial = _sender_manifest(t0g, duration=None)   # <-- NO duration field
        if "duration" in snd_g_partial:
            fails.append("F2: test scenario invalid -- sender manifest still carries 'duration'")
        ev_g_partial = _evidence(cmd_log, rsp_log, attest, rx_g_partial, snd_g_partial)
        row_g_partial = R.score_session(spec, ev_g_partial)
        vgc = (R.score_session(spec, ev_g_complete).get("metric") or {}).get("value_Bmin")
        vgp = (row_g_partial.get("metric") or {}).get("value_Bmin")
        wegp = (row_g_partial.get("metric") or {}).get("wall_end")
        print(f"[integration] F2 no-duration partial={vgp} B/min wall_end={wegp} "
              f"(slow complete={vgc} B/min)")
        if row_g_partial.get("outcome") != "MODEM_FAULT":
            fails.append(f"F2: partial outcome {row_g_partial.get('outcome')} != MODEM_FAULT")
        if wegp != "full_session_wall":
            fails.append(f"F2: no-duration partial wall_end {wegp} != full_session_wall "
                         f"(a missing manifest duration reverted to the TRUNCATED last-verified "
                         f"wall -- manifest-duration dependency not removed)")
        if vgp is None or vgc is None:
            fails.append(f"F2: metric values missing (partial={vgp}, complete={vgc})")
        elif vgp > vgc:
            fails.append(f"F2: no-duration partial {vgp} B/min > slow complete {vgc} B/min -- "
                         f"truncated-wall revert on a missing duration field (ROOT not closed)")
        if plan_wall is not None:                 # exact-value check on the fixed runner
            want_vgp = round(306 / (plan_wall(spec) / 60.0), 1)
            if vgp != want_vgp:
                fails.append(f"F2: partial value {vgp} != {want_vgp} (306 B over the "
                             f"{plan_wall(spec):.0f} s plan wall, independent of the manifest)")
        summ["F2_no_duration_partial"] = {"partial_Bmin": vgp, "partial_wall_end": wegp,
                                          "slow_complete_Bmin": vgc,
                                          "partial_outcome": row_g_partial.get("outcome")}

        # ==== G. METRIC-INTRINSIC OVERRUN GUARD.  A COMPLETE delivery whose
        #      MEASURED wall EXCEEDS the planned session wall is a TEARDOWN-CONTRACT VIOLATION
        #      (dispatcher schedule change / cross-box clock skew / stale manifest epoch) -- its
        #      wall is untrustworthy, so it is scored DEFLATED and out-ranked by a fast
        #      non-prefix partial.  The metric must catch this DEFLATION side intrinsically:
        #      UNSCORED (HARNESS_INVALID), never a deflated number into the distribution.
        #      At the pre-guard HEAD the same complete SCORES ~377.3 B/min (3710 B over 590 s)
        #      and a {ics213,short_email,batch3}=3404 B partial out-ranks it at 523.7 B/min. ====
        pw = plan_wall(spec) if plan_wall is not None else None
        t0h = 4000.0
        overrun_wall = (pw + 200.0) if pw is not None else 590.0     # 390 + 200 = 590 s
        # COMPLETE (all 4 corpus, 3710 B) but its last byte verifies 590 s after the TX anchor
        # -- 200 s PAST the 390 s planned wall (teardown contract violated).
        rx_over = _rx_manifest(R, [m["name"] for m in R.CORPUS], sabm_rx=t0h + 0.5,
                               first_verified=t0h + 1.0, last_verified=t0h + overrun_wall)
        snd_over = _sender_manifest(t0h, duration=300)
        ev_over = _evidence(cmd_log, rsp_log, attest, rx_over, snd_over)
        row_over = R.score_session(spec, ev_over)
        met_over = row_over.get("metric") or {}
        would_be = round(3710 / (overrun_wall / 60.0), 1)            # ~377.3 (the deflated rate)
        print(f"[integration] G overrun complete: measured_wall={overrun_wall:.0f}s "
              f"planned_wall={pw}s outcome={row_over.get('outcome')} "
              f"value={row_over.get('value_Bmin')} (would-be deflated rate={would_be})")
        if row_over.get("outcome") != "HARNESS_INVALID":
            fails.append(f"G: overrun complete outcome {row_over.get('outcome')} != "
                         f"HARNESS_INVALID -- a complete measured over {overrun_wall:.0f}s > "
                         f"planned {pw}s was scored (deflated ~{would_be} B/min), which a fast "
                         f"partial out-ranks (overrun guard not closed)")
        if not met_over.get("harness_invalid") or row_over.get("value_Bmin") is not None:
            fails.append(f"G: overrun complete metric harness_invalid="
                         f"{met_over.get('harness_invalid')} value={row_over.get('value_Bmin')} "
                         f"-- must be harness_invalid / UNSCORED, never a deflated number")
        # a fast NON-PREFIX partial (F1 shape) that WOULD out-rank the deflated complete.
        rx_h_partial = _rx_manifest(R, ["ics213", "short_email", "batch3"], sabm_rx=t0h + 6,
                                    first_verified=t0h + 15, last_verified=t0h + 25)
        snd_h_partial = _sender_manifest(t0h, duration=300)
        row_h_partial = R.score_session(spec, _evidence(cmd_log, rsp_log, attest,
                                                        rx_h_partial, snd_h_partial))
        vhp = (row_h_partial.get("metric") or {}).get("value_Bmin")
        # aggregate [overrun complete] + [3 fast partials].  After the guard the overrun
        # complete is WITHHELD (n_harness_invalid == 1), never a deflated number the partials
        # top; at HEAD it SCORES ~377.3 and the 523.7 partials out-rank it (n_harness_invalid 0).
        rows_g = [row_over] + [R.score_session(spec, _evidence(cmd_log, rsp_log, attest,
                                                               rx_h_partial, snd_h_partial))
                               for _ in range(3)]
        agg_g = R.aggregate_cell("MPG", 15, "off", rows_g)
        if agg_g.get("n_harness_invalid") != 1:
            fails.append(f"G: n_harness_invalid {agg_g.get('n_harness_invalid')} != 1 -- the "
                         f"overrun complete was not withheld (it entered the distribution as a "
                         f"deflated number that a fast partial out-ranks)")
        if vhp is not None and would_be is not None and vhp <= would_be:
            fails.append(f"G: test scenario invalid -- the fast partial {vhp} must exceed the "
                         f"deflated complete rate {would_be} to demonstrate the inversion")
        summ["G_overrun_guard"] = {"measured_wall_s": overrun_wall, "planned_wall_s": pw,
                                   "overrun_outcome": row_over.get("outcome"),
                                   "overrun_value": row_over.get("value_Bmin"),
                                   "deflated_rate_would_be": would_be,
                                   "fast_partial_Bmin": vhp,
                                   "n_harness_invalid": agg_g.get("n_harness_invalid")}

    ok = not fails
    print(json.dumps({
        "runner": RUNNER_PATH, "pump": PUMP_PATH,
        **summ,
        "failures": fails, "PASS": ok,
    }, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
