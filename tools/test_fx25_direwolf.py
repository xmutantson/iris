#!/usr/bin/env python3
"""Test FX.25 interop between Iris and Direwolf.

Cross-cable setup:
  Iris TX -> CABLE-A Input (1001) -> CABLE-A Output (dev 7) -> Direwolf RX
  Direwolf TX -> CABLE Input (dev 5) -> CABLE Output (dev 11) -> Iris RX

Tests:
  1. Iris FX.25 TX -> Direwolf FX.25 RX (DW auto-decodes FX.25)
  2. Direwolf FX.25 TX -> Iris FX.25 RX (DW sends FX.25, Iris decodes)
  3. Iris plain AX.25 TX -> Direwolf plain RX (backwards compat baseline)
"""
import subprocess, socket, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
DIREWOLF = r"C:\tmp\dw-build\src\direwolf.exe"

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

def kiss_recv(sock, timeout=15):
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

def kiss_connect(host, port, timeout=15):
    """Connect to KISS server, retrying until it's ready. Returns connected socket."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect((host, port))
            return s
        except (ConnectionRefusedError, socket.timeout, OSError):
            try: s.close()
            except: pass
            time.sleep(0.5)
    return None

def write_dw_config(path, kiss_port, fx25_tx=0):
    """Write a Direwolf config. fx25_tx=0 means plain, 16/32/64 enables FX.25 TX."""
    with open(path, "w") as f:
        f.write("MYCALL DWTST\n")
        # Cross-cable: DW captures from CABLE-A Output (dev 7), plays to CABLE Input (dev 5)
        f.write("ADEVICE 7 5\n")
        f.write("ARATE 48000\n")
        f.write("ACHANNELS 1\n")
        f.write("CHANNEL 0\n")
        f.write("MODEM 1200\n")
        f.write(f"KISSPORT {kiss_port}\n")
        f.write("AGWPORT 0\n")
        if fx25_tx > 0:
            f.write(f"FX25TX {fx25_tx}\n")
            f.write("FX25AUTO 0\n")
    return path

def cleanup(*procs, files=None):
    for p in procs:
        try: p.terminate()
        except: pass
    for p in procs:
        try: p.wait(timeout=3)
        except:
            try: p.kill()
            except: pass
    if files:
        for f in files:
            try: f.close()
            except: pass

def show_log(fn, lines=30):
    if os.path.isfile(fn):
        print(f"\n--- {fn} (last {lines} lines) ---")
        with open(fn) as f:
            content = f.readlines()
        for line in content[-lines:]:
            print(f"  {line.rstrip()}")

def run_test(test_num, description, iris_fx25, dw_fx25, tx_from):
    """
    Run one interop test.
    tx_from: 'iris' = Iris sends, DW receives. 'dw' = DW sends, Iris receives.
    """
    # Unique ports per test to avoid TIME_WAIT
    iris_kiss = 8100 + test_num * 10
    iris_agw  = 8100 + test_num * 10 + 1
    dw_kiss   = 8100 + test_num * 10 + 2

    print(f"\n{'=' * 60}")
    print(f"[TEST {test_num}] {description}")
    print(f"  Iris KISS={iris_kiss}, DW KISS={dw_kiss}")
    print(f"{'=' * 60}")

    dw_conf = write_dw_config(
        rf"C:\tmp\direwolf_fx25_test{test_num}.conf",
        kiss_port=dw_kiss, fx25_tx=dw_fx25
    )

    f_iris = open(f"fx25_dw_test{test_num}_iris.log", "w")
    f_dw = open(f"fx25_dw_test{test_num}_dw.log", "w")
    iris_debug = f"fx25_dw_test{test_num}_iris_debug.log"

    # Iris: TX on CABLE-A (1001), RX on CABLE (11)
    iris_cmd = [
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.5",
        "--port", str(iris_kiss), "--agw-port", str(iris_agw),
        "--capture", "12", "--playback", "1001",
        "--ax25-only", "--log", iris_debug
    ]
    if iris_fx25 > 0:
        iris_cmd.extend(["--fx25", str(iris_fx25)])

    proc_iris = subprocess.Popen(iris_cmd, stdout=f_iris, stderr=subprocess.STDOUT)
    time.sleep(2)

    proc_dw = subprocess.Popen(
        [DIREWOLF, "-c", dw_conf, "-t", "0"],
        stdout=f_dw, stderr=f_dw
    )
    time.sleep(3)

    # Check processes alive
    for name, proc in [("Iris", proc_iris), ("Direwolf", proc_dw)]:
        if proc.poll() is not None:
            print(f"  [ERROR] {name} exited with code {proc.returncode}")
            cleanup(proc_iris, proc_dw, files=[f_iris, f_dw])
            return False

    # Connect KISS clients (keep connected — no probe+reconnect race)
    sock_iris = kiss_connect("127.0.0.1", iris_kiss)
    if not sock_iris:
        print("  [ERROR] Iris KISS not ready")
        cleanup(proc_iris, proc_dw, files=[f_iris, f_dw])
        return False
    print("  Iris KISS connected")

    sock_dw = kiss_connect("127.0.0.1", dw_kiss)
    if not sock_dw:
        print("  [ERROR] Direwolf KISS not ready")
        try: sock_iris.close()
        except: pass
        cleanup(proc_iris, proc_dw, files=[f_iris, f_dw])
        return False
    print("  Direwolf KISS connected")

    # Wait for server to register the client connections
    time.sleep(2)

    ok = False
    if tx_from == "iris":
        payload = f"FX25-test{test_num} from Iris".encode()
        ax25_frame = build_ax25_ui("STN-A", "DWTST", payload)
        sock_iris.sendall(kiss_encode(ax25_frame))
        print(f"  Sent {len(ax25_frame)} byte frame via Iris")

        print("  Waiting for Direwolf to decode...")
        frames = kiss_recv(sock_dw, timeout=15)
        if frames:
            for cmd, data in frames:
                print(f"  Direwolf got: {len(data)} bytes")
                if payload in data:
                    print("  *** MATCH ***")
                    ok = True
                else:
                    print(f"  Payload mismatch: {data[:50]}")
        else:
            print("  Direwolf: nothing received via KISS")

    elif tx_from == "dw":
        payload = f"FX25-test{test_num} from Direwolf".encode()
        ax25_frame = build_ax25_ui("DWTST", "STN-A", payload)
        sock_dw.sendall(kiss_encode(ax25_frame))
        print(f"  Sent {len(ax25_frame)} byte frame via Direwolf")

        print("  Waiting for Iris to decode...")
        frames = kiss_recv(sock_iris, timeout=15)
        if frames:
            for cmd, data in frames:
                print(f"  Iris got: {len(data)} bytes")
                if payload in data:
                    print("  *** MATCH ***")
                    ok = True
                else:
                    print(f"  Payload mismatch: {data[:50]}")
        else:
            print("  Iris: nothing received via KISS")

    # Cleanup
    try: sock_iris.close()
    except: pass
    try: sock_dw.close()
    except: pass
    cleanup(proc_iris, proc_dw, files=[f_iris, f_dw])

    # Show logs
    show_log(iris_debug)
    show_log(f"fx25_dw_test{test_num}_dw.log")
    show_log(f"fx25_dw_test{test_num}_iris.log")

    print(f"\n  {'PASS' if ok else 'FAIL'}: {description}")
    return ok

def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1
    if not os.path.isfile(DIREWOLF):
        print(f"[ERROR] Not found: {DIREWOLF}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    os.system("taskkill /F /IM direwolf.exe 2>nul >nul")
    time.sleep(1)

    tests = [
        (1, "Iris FX.25 TX -> Direwolf RX",       16, 0,  "iris"),
        (2, "Direwolf FX.25 TX -> Iris RX",        0,  16, "dw"),
        (3, "Iris plain AX.25 TX -> Direwolf RX",  0,  0,  "iris"),
    ]

    results = []
    for test_num, desc, iris_fx25, dw_fx25, tx_from in tests:
        ok = run_test(test_num, desc, iris_fx25, dw_fx25, tx_from)
        results.append((desc, ok))
        time.sleep(3)

    print(f"\n{'=' * 60}")
    print("FX.25 Direwolf Interop Results:")
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
