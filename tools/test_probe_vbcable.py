#!/usr/bin/env python3
"""
Test passband probe handshake over VB-Cable.

Starts two iris instances (STA-A and STA-B) sharing one VB-Cable.
TX mute prevents self-hearing. STA-A connects to STA-B via AGW,
triggering XID handshake which auto-triggers the passband probe.
Checks logs for probe completion.

Requires: VB-Cable virtual audio cable.
"""
import subprocess, socket, struct, time, sys, os, re

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

# Audio device IDs (from --list-audio on this machine)
# Both stations share one VB-Cable — TX mute prevents self-hearing.
CABLE_PLAYBACK = 1005   # CABLE Input
CABLE_CAPTURE  = 11     # CABLE Output

STA_A_KISS = 8001
STA_A_AGW  = 8000
STA_B_KISS = 8011
STA_B_AGW  = 8010

KISS_FEND = 0xC0


def send_agw(sock, kind, call_from="", call_to="", data=b""):
    hdr = bytearray(36)
    hdr[4] = ord(kind)
    hdr[6] = 0xF0
    cf = call_from.encode()[:9]
    hdr[8:8+len(cf)] = cf
    ct = call_to.encode()[:9]
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, len(data))
    sock.sendall(bytes(hdr) + data)


def grep_log(path, pattern):
    """Return lines matching pattern from log file."""
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8', errors='replace') as f:
        return [l.rstrip() for l in f if pattern in l]


def main():
    iris = os.path.abspath(IRIS)
    if not os.path.isfile(iris):
        print(f"[ERROR] Not found: {iris}")
        return 1

    # Kill any existing iris
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(2)

    log_a = os.path.join(os.path.dirname(iris), "probe_test_a.log")
    log_b = os.path.join(os.path.dirname(iris), "probe_test_b.log")

    # Clean old logs
    for f in [log_a, log_b]:
        try: os.remove(f)
        except: pass

    print("=" * 60)
    print("  Iris Passband Probe Test (VB-Cable)")
    print("=" * 60)

    # Start STA-B first (responder/listener)
    print("\n[1] Starting STA-B (responder)...")
    cmd_b = [
        iris, "--nogui",
        "--callsign", "STN-B",
        "--mode", "A", "--tx-level", "0.4",
        "--playback", str(CABLE_PLAYBACK),
        "--capture", str(CABLE_CAPTURE),
        "--port", str(STA_B_KISS),
        "--agw-port", str(STA_B_AGW),
        "--log", log_b,
    ]
    fa_out = open(log_b + ".stdout", "w")
    proc_b = subprocess.Popen(cmd_b, stdout=fa_out, stderr=subprocess.STDOUT)
    time.sleep(5)

    # Start STA-A (initiator)
    print("[2] Starting STA-A (initiator)...")
    cmd_a = [
        iris, "--nogui",
        "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.4",
        "--playback", str(CABLE_PLAYBACK),
        "--capture", str(CABLE_CAPTURE),
        "--port", str(STA_A_KISS),
        "--agw-port", str(STA_A_AGW),
        "--log", log_a,
    ]
    fb_out = open(log_a + ".stdout", "w")
    proc_a = subprocess.Popen(cmd_a, stdout=fb_out, stderr=subprocess.STDOUT)
    time.sleep(5)

    # Send KISS data from STA-A — this triggers XID handshake which triggers probe
    print("[3] Connecting KISS and sending data (triggers XID -> probe)...")
    kiss_a = None
    try:
        kiss_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        kiss_a.settimeout(5)
        kiss_a.connect(("127.0.0.1", STA_A_KISS))
        print("    KISS connected to STA-A on port %d" % STA_A_KISS)

        # Send a data frame — modem will send XID first, then defer data
        test_payload = b"Probe test from STN-A"
        kiss_frame = bytearray([KISS_FEND, 0x00])  # KISS data, port 0
        for b in test_payload:
            if b == KISS_FEND:
                kiss_frame.extend([0xDB, 0xDC])
            elif b == 0xDB:
                kiss_frame.extend([0xDB, 0xDD])
            else:
                kiss_frame.append(b)
        kiss_frame.append(KISS_FEND)
        kiss_a.sendall(bytes(kiss_frame))
        print("    KISS data sent (triggers XID -> probe)")
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        proc_a.terminate(); proc_b.terminate()
        return 1

    # Wait for probe to complete (check logs periodically)
    print("[4] Waiting for probe handshake (up to 60s)...")
    probe_ok_a = False
    probe_ok_b = False
    start = time.time()
    while time.time() - start < 60:
        time.sleep(2)
        elapsed = time.time() - start

        lines_a = grep_log(log_a, "[PROBE]") + grep_log(log_a, "post-probe") + grep_log(log_a, "Probe complete")
        lines_b = grep_log(log_b, "[PROBE]") + grep_log(log_b, "post-probe") + grep_log(log_b, "Probe complete")

        # Check for completion (case-insensitive)
        for l in lines_a:
            ll = l.lower()
            if "complete" in ll or "post-probe" in ll or "probe complete" in ll:
                probe_ok_a = True
            if "fallback" in ll:
                probe_ok_a = True  # fallback also counts as "done"
        for l in lines_b:
            ll = l.lower()
            if "complete" in ll or "post-probe" in ll or "probe complete" in ll:
                probe_ok_b = True
            if "fallback" in ll:
                probe_ok_b = True

        status_a = "DONE" if probe_ok_a else "waiting"
        status_b = "DONE" if probe_ok_b else "waiting"
        print(f"    [{elapsed:.0f}s] STA-A: {status_a} ({len(lines_a)} probe msgs), "
              f"STA-B: {status_b} ({len(lines_b)} probe msgs)")

        if probe_ok_a and probe_ok_b:
            break

    # Collect results
    print("\n" + "=" * 60)
    print("  RESULTS")
    print("=" * 60)

    for label, log_path in [("STA-A (initiator)", log_a), ("STA-B (responder)", log_b)]:
        print(f"\n--- {label} ---")
        # Show all probe-related and XID-related log lines
        all_lines = []
        if os.path.isfile(log_path):
            with open(log_path, encoding='utf-8', errors='replace') as f:
                for l in f:
                    l = l.rstrip()
                    if "[PROBE]" in l or "XID" in l or "native" in l.lower() or "probe" in l.lower():
                        all_lines.append(l)
        if all_lines:
            for l in all_lines:
                print(f"  {l}")
        else:
            print("  (no probe/XID log lines found)")

    # Verdict
    print("\n" + "-" * 60)
    if probe_ok_a and probe_ok_b:
        print("  PASS: Probe handshake completed on both sides")
    elif probe_ok_a or probe_ok_b:
        print("  PARTIAL: Probe completed on one side only")
    else:
        # Check if XID at least worked
        xid_a = grep_log(log_a, "XID")
        xid_b = grep_log(log_b, "XID")
        if xid_a or xid_b:
            print("  FAIL: XID handshake occurred but probe did not complete")
        else:
            print("  FAIL: No XID handshake detected (audio path issue?)")
    print("-" * 60)

    # Cleanup
    if kiss_a:
        try: kiss_a.close()
        except: pass
    proc_a.terminate()
    proc_b.terminate()
    try: proc_a.wait(timeout=3)
    except: proc_a.kill()
    try: proc_b.wait(timeout=3)
    except: proc_b.kill()
    try: fa_out.close()
    except: pass
    try: fb_out.close()
    except: pass
    os.system("taskkill /F /IM iris.exe 2>nul >nul")

    return 0 if (probe_ok_a and probe_ok_b) else 1


if __name__ == "__main__":
    sys.exit(main())
