#!/usr/bin/env python3
"""Test XID fallback: Station B runs --ax25-only (simulating a non-Iris TNC like Direwolf).

Station A sends data, triggers XID. B ignores XID (ax25-only).
A should fall back to AX.25 after timeout and data should still flow.
This validates interoperability with standard AX.25 TNCs.
"""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

KISS_FEND  = 0xC0
KISS_FESC  = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

def kiss_encode(data, port=0, cmd=0):
    frame = bytearray([KISS_FEND, (port << 4) | cmd])
    for b in data:
        if b == KISS_FEND:
            frame.extend([KISS_FESC, KISS_TFEND])
        elif b == KISS_FESC:
            frame.extend([KISS_FESC, KISS_TFESC])
        else:
            frame.append(b)
    frame.append(KISS_FEND)
    return bytes(frame)

def kiss_decode(raw):
    frames = []
    in_frame = False
    escaped = False
    got_cmd = False
    cmd = 0
    data = bytearray()
    for b in raw:
        if b == KISS_FEND:
            if in_frame and (len(data) > 0 or got_cmd):
                frames.append((cmd, bytes(data)))
            in_frame = True
            escaped = False
            got_cmd = False
            cmd = 0
            data = bytearray()
        elif not in_frame:
            continue
        elif b == KISS_FESC:
            escaped = True
        elif escaped:
            if b == KISS_TFEND:
                data.append(KISS_FEND)
            elif b == KISS_TFESC:
                data.append(KISS_FESC)
            escaped = False
        elif not got_cmd:
            cmd = b
            got_cmd = True
        else:
            data.append(b)
    return frames

def kiss_recv(sock, timeout=10):
    sock.settimeout(timeout)
    rx_buf = b""
    start = time.time()
    while time.time() - start < timeout:
        try:
            data = sock.recv(4096)
            if data:
                rx_buf += data
                frames = kiss_decode(rx_buf)
                if frames:
                    return frames
        except socket.timeout:
            continue
        except (ConnectionResetError, ConnectionAbortedError, OSError):
            return []
    return []

def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(0.5)

    CAPTURE = "11"
    PLAYBACK = "1005"

    fa = open("xid_fallback_a_stdout.log", "w")
    fb = open("xid_fallback_b_stdout.log", "w")

    # Station A: normal mode (will try XID)
    print("[TEST] Starting Station A (STN-A) — normal mode, will try XID...")
    proc_a = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8001", "--agw-port", "8010",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--log", "xid_fallback_a.log"
    ], stdout=fa, stderr=subprocess.STDOUT)

    # Station B: --ax25-only (simulates Direwolf / standard TNC)
    print("[TEST] Starting Station B (STN-B) — ax25-only (simulating Direwolf)...")
    proc_b = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-B",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8002", "--agw-port", "8011",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--ax25-only",
        "--log", "xid_fallback_b.log"
    ], stdout=fb, stderr=subprocess.STDOUT)

    time.sleep(5)

    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"[ERROR] Station {name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            return 1

    print("[TEST] Connecting KISS clients...")
    try:
        sock_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_a.settimeout(5)
        sock_a.connect(("127.0.0.1", 8001))

        sock_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_b.settimeout(5)
        sock_b.connect(("127.0.0.1", 8002))
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        proc_a.terminate(); proc_b.terminate()
        return 1

    success = True

    # --- Step 1: A sends data (triggers XID + deferred data) ---
    msg = b"Hello from A! Testing XID fallback."
    print(f"\n[1] A sends: {msg.decode()}")
    print("    (XID sent, B ignores it, A will fallback after ~4s timeout)")
    sock_a.sendall(kiss_encode(msg))

    # B should receive the data after A's XID timeout (~4s fallback)
    # Increase timeout to account for XID fallback delay
    print("    Waiting for data on B (expect ~4s delay for XID fallback)...")
    t0 = time.time()
    frames = kiss_recv(sock_b, timeout=15)
    elapsed = time.time() - t0
    if frames:
        for cmd, payload in frames:
            print(f"    B got: {payload} (after {elapsed:.1f}s)")
        if elapsed > 2.0:
            print(f"    *** XID FALLBACK CONFIRMED (delay={elapsed:.1f}s > 2s) ***")
        else:
            print(f"    [WARN] Data arrived too fast ({elapsed:.1f}s), XID may not have been tried")
    else:
        print("    B: nothing received")
        success = False

    time.sleep(2)

    # --- Step 2: B sends reply (B is ax25-only, no XID) ---
    msg_b = b"Reply from B (ax25-only)!"
    print(f"\n[2] B sends: {msg_b.decode()}")
    try:
        sock_b.sendall(kiss_encode(msg_b))
    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"    B send failed: {e}")
        success = False

    frames = kiss_recv(sock_a, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"    A got: {payload}")
    else:
        print("    A: nothing received")
        success = False

    time.sleep(1)

    # --- Step 3: A sends second message (should be AX.25 since no XID response) ---
    msg_a2 = b"Second from A (should be AX.25)!"
    print(f"\n[3] A sends: {msg_a2.decode()}")
    t0 = time.time()
    try:
        sock_a.sendall(kiss_encode(msg_a2))
    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"    A send failed: {e}")
        success = False

    frames = kiss_recv(sock_b, timeout=10)
    elapsed = time.time() - t0
    if frames:
        for cmd, payload in frames:
            print(f"    B got: {payload} (after {elapsed:.1f}s)")
        if elapsed < 3.0:
            print(f"    *** NO XID DELAY — A stayed in AX.25 mode ***")
        else:
            print(f"    [WARN] Unexpected delay ({elapsed:.1f}s)")
    else:
        print("    B: nothing received")
        success = False

    # Cleanup
    print("\n[TEST] Cleaning up...")
    try: sock_a.close()
    except: pass
    try: sock_b.close()
    except: pass
    proc_a.terminate()
    proc_b.terminate()
    try:
        proc_a.wait(timeout=3)
        proc_b.wait(timeout=3)
    except:
        proc_a.kill()
        proc_b.kill()
    fa.close()
    fb.close()

    for name, logfile in [("A", "xid_fallback_a.log"), ("B", "xid_fallback_b.log")]:
        if os.path.isfile(logfile):
            print(f"\n--- Station {name} log (last 20 lines) ---")
            with open(logfile) as f:
                lines = f.readlines()
            for line in lines[-20:]:
                print(f"  {line.rstrip()}")

    print(f"\n{'PASS' if success else 'FAIL'}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
