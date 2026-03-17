#!/usr/bin/env python3
"""Test FX.25 encode/decode via Iris loopback on VB-Cable.

Two Iris instances on VB-Cable (one TX with --fx25, one RX).
Sends a KISS frame through Iris-A (FX.25 TX) and checks that Iris-B (plain RX)
receives it via the FX.25 decoder.

Test matrix:
  1. FX.25 TX (16 check) -> FX.25 RX (always-on decoder)
  2. FX.25 TX (32 check) -> FX.25 RX
  3. FX.25 TX (64 check) -> FX.25 RX
  4. Plain AX.25 TX -> still works (backwards compat)
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

def build_ax25_ui(src_call, dst_call, payload):
    """Build a minimal AX.25 UI frame (no digipeaters)."""
    frame = bytearray()
    dst = dst_call.ljust(6)[:6]
    for c in dst:
        frame.append(ord(c) << 1)
    frame.append(0x60)
    src = src_call.ljust(6)[:6]
    for c in src:
        frame.append(ord(c) << 1)
    frame.append(0x61)
    frame.append(0x03)
    frame.append(0xF0)
    frame.extend(payload)
    return bytes(frame)

def run_test(fx25_mode, test_num, total):
    """Run one FX.25 test with given mode (0=plain, 16/32/64=FX.25)."""
    mode_str = f"FX.25-{fx25_mode}" if fx25_mode > 0 else "plain AX.25"
    print(f"\n[{test_num}/{total}] Testing {mode_str}...")

    # Use different ports for each test to avoid TIME_WAIT conflicts
    base_port = 8100 + test_num * 10
    port_a = base_port
    port_b = base_port + 1
    agw_a = base_port + 2
    agw_b = base_port + 3

    # Single cable: A TX -> CABLE -> B RX (one direction only)
    # A: playback=1006 (CABLE Input), capture=2 (dummy)
    # B: capture=12 (CABLE Output), playback=1001 (dummy, CABLE-A Input)
    cmd_a = [
        IRIS, "--nogui", "--callsign", "FX-A",
        "--mode", "A", "--tx-level", "0.5",
        "--port", str(port_a), "--agw-port", str(agw_a),
        "--capture", "2", "--playback", "1006",
        "--ax25-only",
    ]
    if fx25_mode > 0:
        cmd_a.extend(["--fx25", str(fx25_mode)])

    cmd_b = [
        IRIS, "--nogui", "--callsign", "FX-B",
        "--mode", "A", "--tx-level", "0.5",
        "--port", str(port_b), "--agw-port", str(agw_b),
        "--capture", "12", "--playback", "1001",
        "--ax25-only",
    ]

    f_a = open(f"fx25_test_{test_num}_a.log", "w")
    f_b = open(f"fx25_test_{test_num}_b.log", "w")

    proc_a = subprocess.Popen(cmd_a, stdout=f_a, stderr=subprocess.STDOUT)
    proc_b = subprocess.Popen(cmd_b, stdout=f_b, stderr=subprocess.STDOUT)
    time.sleep(3)

    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"  [ERROR] Iris-{name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            f_a.close(); f_b.close()
            return False

    try:
        sock_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_a.settimeout(5)
        sock_a.connect(("127.0.0.1", port_a))

        sock_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_b.settimeout(5)
        sock_b.connect(("127.0.0.1", port_b))
    except Exception as e:
        print(f"  [ERROR] KISS connect failed: {e}")
        proc_a.terminate(); proc_b.terminate()
        f_a.close(); f_b.close()
        return False

    # Send from A to B
    payload = f"FX25 test mode={fx25_mode}".encode()
    ax25_frame = build_ax25_ui("FX-A", "FX-B", payload)
    sock_a.sendall(kiss_encode(ax25_frame))
    print(f"  Sent {len(ax25_frame)} byte AX.25 frame via A (KISS port {port_a})")

    frames = kiss_recv(sock_b, timeout=10)
    success = False
    if frames:
        for cmd, data in frames:
            if payload in data:
                print(f"  B received: payload match! ({len(data)} bytes)")
                success = True
            else:
                print(f"  B received {len(data)} bytes but no payload match")
    else:
        print(f"  B: nothing received")

    # Cleanup
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
    f_a.close()
    f_b.close()

    print(f"  {'PASS' if success else 'FAIL'}")
    return success

def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(1)

    tests = [
        (16, "FX.25 with 16 check bytes"),
        (32, "FX.25 with 32 check bytes"),
        (64, "FX.25 with 64 check bytes"),
        (0,  "Plain AX.25 (backwards compat)"),
    ]

    results = []
    for i, (mode, desc) in enumerate(tests):
        ok = run_test(mode, i + 1, len(tests))
        results.append((desc, ok))
        time.sleep(2)  # Let ports clear

    print("\n" + "=" * 50)
    print("FX.25 Test Results:")
    all_pass = True
    for desc, ok in results:
        status = "PASS" if ok else "FAIL"
        print(f"  {status}: {desc}")
        if not ok:
            all_pass = False

    print(f"\n{'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    return 0 if all_pass else 1

if __name__ == "__main__":
    sys.exit(main())
