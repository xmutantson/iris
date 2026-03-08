#!/usr/bin/env python3
"""Test Iris <-> Direwolf interop over AFSK 1200 on VB-Cable C.

Iris (ax25-only mode) and Direwolf share VB-Cable C.
Both use standard Bell 202 AFSK 1200 with AX.25 HDLC framing.
Data is sent via KISS on both sides.

Test verifies:
  1. Iris -> Direwolf: Iris sends AX.25 frame, Direwolf decodes it
  2. Direwolf -> Iris: Direwolf sends AX.25 frame, Iris decodes it
"""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
DIREWOLF = r"C:\tmp\dw-src\build\src\direwolf.exe"
DIREWOLF_CONF = r"C:\tmp\direwolf_test.conf"

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
    # Destination address (7 bytes: 6 char callsign shifted left + SSID byte)
    dst = dst_call.ljust(6)[:6]
    for c in dst:
        frame.append(ord(c) << 1)
    frame.append(0x60)  # SSID=0, C bit set, last=0
    # Source address
    src = src_call.ljust(6)[:6]
    for c in src:
        frame.append(ord(c) << 1)
    frame.append(0x61)  # SSID=0, last=1 (no digipeaters)
    # Control: UI (0x03)
    frame.append(0x03)
    # PID: No layer 3 (0xF0)
    frame.append(0xF0)
    # Payload
    frame.extend(payload)
    return bytes(frame)

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

    # Cross-cable setup to avoid VB-Cable single-reader conflict:
    # Iris TX -> CABLE-A Input (1001) -> CABLE-A Output -> Direwolf RX
    # Direwolf TX -> CABLE Input -> CABLE Output -> Iris RX (11)
    CAPTURE = "11"    # CABLE Output (receives Direwolf TX)
    PLAYBACK = "1001" # CABLE-A Input (Iris TX goes to Direwolf RX)

    f_iris = open("dw_interop_iris.log", "w")
    f_dw = open("dw_interop_direwolf.log", "w")

    # Start Iris in ax25-only mode (no XID, no native PHY)
    print("[TEST] Starting Iris (STN-A) on KISS 8001, ax25-only mode...")
    proc_iris = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.5",
        "--port", "8001", "--agw-port", "8010",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--ax25-only",
        "--log", "dw_interop_iris_debug.log"
    ], stdout=f_iris, stderr=subprocess.STDOUT)

    time.sleep(2)

    # Start Direwolf
    print("[TEST] Starting Direwolf (DWTST) on KISS 8021...")
    proc_dw = subprocess.Popen([
        DIREWOLF, "-c", DIREWOLF_CONF, "-t", "0"
    ], stdout=f_dw, stderr=f_dw)

    time.sleep(5)

    # Verify both running
    for name, proc in [("Iris", proc_iris), ("Direwolf", proc_dw)]:
        if proc.poll() is not None:
            print(f"[ERROR] {name} exited with code {proc.returncode}")
            proc_iris.terminate(); proc_dw.terminate()
            # Show logs
            f_iris.close(); f_dw.close()
            for fn in ["dw_interop_iris.log", "dw_interop_direwolf.log"]:
                if os.path.isfile(fn):
                    print(f"\n--- {fn} ---")
                    with open(fn) as f:
                        print(f.read()[-2000:])
            return 1

    # Connect KISS clients
    print("[TEST] Connecting KISS clients...")
    try:
        sock_iris = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_iris.settimeout(5)
        sock_iris.connect(("127.0.0.1", 8001))
        print("  Iris KISS connected on 8001")

        sock_dw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_dw.settimeout(5)
        sock_dw.connect(("127.0.0.1", 8021))
        print("  Direwolf KISS connected on 8021")
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        proc_iris.terminate(); proc_dw.terminate()
        f_iris.close(); f_dw.close()
        return 1

    success = True

    # --- Step 1: Iris -> Direwolf ---
    print("\n[1] Iris sends AX.25 UI frame to Direwolf...")
    ax25_frame = build_ax25_ui("STN-A", "DWTST", b"Hello from Iris!")
    sock_iris.sendall(kiss_encode(ax25_frame))
    print(f"    Sent {len(ax25_frame)} byte AX.25 frame via Iris KISS")

    # Wait for Direwolf to decode it
    print("    Waiting for Direwolf to receive...")
    frames = kiss_recv(sock_dw, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"    Direwolf got: {payload}")
            if b"Hello from Iris!" in payload:
                print("    *** IRIS -> DIREWOLF: DATA MATCH ***")
            else:
                print("    Payload didn't match expected content")
    else:
        print("    Direwolf: nothing received via KISS")
        success = False

    time.sleep(2)

    # --- Step 2: Direwolf -> Iris ---
    print("\n[2] Direwolf sends AX.25 UI frame to Iris...")
    ax25_frame_dw = build_ax25_ui("DWTST", "STN-A", b"Hello from Direwolf!")
    sock_dw.sendall(kiss_encode(ax25_frame_dw))
    print(f"    Sent {len(ax25_frame_dw)} byte AX.25 frame via Direwolf KISS")

    print("    Waiting for Iris to receive...")
    frames = kiss_recv(sock_iris, timeout=10)
    if frames:
        for cmd, payload in frames:
            print(f"    Iris got: {payload}")
            if b"Hello from Direwolf!" in payload:
                print("    *** DIREWOLF -> IRIS: DATA MATCH ***")
            else:
                print("    Payload didn't match expected content")
    else:
        print("    Iris: nothing received via KISS")
        success = False

    # Cleanup
    print("\n[TEST] Cleaning up...")
    try: sock_iris.close()
    except: pass
    try: sock_dw.close()
    except: pass
    proc_iris.terminate()
    proc_dw.terminate()
    try:
        proc_iris.wait(timeout=3)
        proc_dw.wait(timeout=3)
    except:
        proc_iris.kill()
        proc_dw.kill()
    f_iris.close()
    f_dw.close()

    # Show logs
    for fn in ["dw_interop_iris_debug.log"]:
        if os.path.isfile(fn):
            print(f"\n--- {fn} (last 20 lines) ---")
            with open(fn) as f:
                lines = f.readlines()
            for line in lines[-20:]:
                print(f"  {line.rstrip()}")

    for fn in ["dw_interop_direwolf.log"]:
        if os.path.isfile(fn):
            print(f"\n--- {fn} (last 30 lines) ---")
            with open(fn) as f:
                lines = f.readlines()
            for line in lines[-30:]:
                print(f"  {line.rstrip()}")

    print(f"\n{'PASS' if success else 'FAIL'}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
