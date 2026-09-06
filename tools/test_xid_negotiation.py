#!/usr/bin/env python3
"""Test Iris-to-Iris XID negotiation on shared VB-Cable.

Two iris instances (STN-A, STN-B) on the same VB-Cable.
Both start in AX.25 mode. A sends data, which triggers XID.
After XID handshake completes, both should upgrade to native.
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
    """Receive KISS frames with timeout."""
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
            print("  [WARN] connection lost")
            return []
    return []

def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(0.5)

    # VB-Cable: capture=11 (CABLE Output), playback=1005 (CABLE Input)
    # Both stations share the same virtual cable (like a shared radio channel)
    CAPTURE = "11"
    PLAYBACK = "1005"

    # Keep file handles open for the whole test
    fa = open("xid_a_stdout.log", "w")
    fb = open("xid_b_stdout.log", "w")

    # Station A: KISS port 8001, log to xid_a.log
    print("[TEST] Starting Station A (STN-A) on KISS 8001...")
    proc_a = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8001", "--agw-port", "8010",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--log", "xid_a.log"
    ], stdout=fa, stderr=subprocess.STDOUT)

    # Station B: KISS port 8002, log to xid_b.log
    print("[TEST] Starting Station B (STN-B) on KISS 8002...")
    proc_b = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-B",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8002", "--agw-port", "8011",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--log", "xid_b.log"
    ], stdout=fb, stderr=subprocess.STDOUT)

    time.sleep(5)  # Give both instances time to initialize audio

    # Verify both are still running
    if proc_a.poll() is not None:
        print(f"[ERROR] Station A exited with code {proc_a.returncode}")
        proc_b.terminate()
        return 1
    if proc_b.poll() is not None:
        print(f"[ERROR] Station B exited with code {proc_b.returncode}")
        proc_a.terminate()
        return 1

    # Connect KISS to both
    print("[TEST] Connecting KISS clients...")
    try:
        sock_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_a.settimeout(5)
        sock_a.connect(("127.0.0.1", 8001))
        print("  A connected on 8001")

        sock_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_b.settimeout(5)
        sock_b.connect(("127.0.0.1", 8002))
        print("  B connected on 8002")
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        proc_a.terminate()
        proc_b.terminate()
        return 1

    success = True

    # --- Step 1: A sends data (triggers XID + AX.25 data) ---
    msg_a = b"Hello from A!"
    print(f"\n[1] A sends: {msg_a.decode()}")
    sock_a.sendall(kiss_encode(msg_a))

    # B should receive the data frame
    frames = kiss_recv(sock_b, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"  B got: {payload}")
    else:
        print("  B: nothing received")
        success = False

    time.sleep(2)  # Let XID handshake complete

    # --- Step 2: B sends reply ---
    msg_b = b"Reply from B!"
    print(f"\n[2] B sends: {msg_b.decode()}")
    try:
        sock_b.sendall(kiss_encode(msg_b))
    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"  [WARN] B send failed: {e}")
        success = False

    # A should receive B's reply
    frames = kiss_recv(sock_a, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"  A got: {payload}")
    else:
        print("  A: nothing received")
        success = False

    time.sleep(1)

    # --- Step 3: A sends again (should be native if XID completed) ---
    msg_a2 = b"Second from A!"
    print(f"\n[3] A sends: {msg_a2.decode()}")
    try:
        sock_a.sendall(kiss_encode(msg_a2))
    except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
        print(f"  [WARN] A send failed: {e}")
        success = False

    frames = kiss_recv(sock_b, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"  B got: {payload}")
    else:
        print("  B: nothing received")
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

    # Show logs
    for name, logfile in [("A", "xid_a_stdout.log"), ("B", "xid_b_stdout.log")]:
        if os.path.isfile(logfile):
            print(f"\n--- Station {name} stdout (last 15 lines) ---")
            with open(logfile) as f:
                lines = f.readlines()
            for line in lines[-15:]:
                print(f"  {line.rstrip()}")
    for name, logfile in [("A", "xid_a.log"), ("B", "xid_b.log")]:
        if os.path.isfile(logfile):
            print(f"\n--- Station {name} log (last 25 lines) ---")
            with open(logfile) as f:
                lines = f.readlines()
            for line in lines[-25:]:
                print(f"  {line.rstrip()}")

    print(f"\n{'PASS' if success else 'FAIL'}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
