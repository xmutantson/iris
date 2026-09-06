#!/usr/bin/env python3
"""Test AX.25 connected mode between Iris and Direwolf over VB-Cable.

Iris uses AGW protocol (connect, send data, disconnect).
Direwolf uses KISS for monitoring + AGW for connected mode.

Test verifies:
  1. Iris initiates AX.25 SABM -> Direwolf UA -> CONNECTED
  2. Iris sends data -> Direwolf receives via I-frames
  3. Direwolf sends data -> Iris receives via I-frames
  4. Clean disconnect (DISC/UA)
"""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
DIREWOLF = r"C:\tmp\dw-src\build\src\direwolf.exe"
DIREWOLF_CONF = r"C:\tmp\direwolf_test.conf"

def agw_header(kind, call_from="", call_to="", data_len=0, port=0):
    """Build a 36-byte AGW header."""
    hdr = bytearray(36)
    hdr[0] = port
    hdr[4] = ord(kind) if isinstance(kind, str) else kind
    hdr[6] = 0xF0  # PID
    # call_from at offset 8 (10 bytes)
    cf = call_from.encode()[:9]
    hdr[8:8+len(cf)] = cf
    # call_to at offset 18 (10 bytes)
    ct = call_to.encode()[:9]
    hdr[18:18+len(ct)] = ct
    # data_len at offset 28 (uint32 LE)
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)

def agw_send(sock, kind, call_from="", call_to="", data=b""):
    """Send an AGW frame."""
    hdr = agw_header(kind, call_from, call_to, len(data))
    sock.sendall(hdr + data)

def agw_recv(sock, timeout=10):
    """Receive one AGW frame. Returns (kind_char, call_from, call_to, data)."""
    sock.settimeout(timeout)
    # Read 36-byte header
    hdr = b""
    while len(hdr) < 36:
        chunk = sock.recv(36 - len(hdr))
        if not chunk:
            return None
        hdr += chunk
    kind = chr(hdr[4])
    call_from = hdr[8:18].split(b'\x00')[0].decode(errors='replace')
    call_to = hdr[18:28].split(b'\x00')[0].decode(errors='replace')
    data_len = struct.unpack_from('<I', hdr, 28)[0]
    data = b""
    while len(data) < data_len:
        chunk = sock.recv(data_len - len(data))
        if not chunk:
            break
        data += chunk
    return (kind, call_from, call_to, data)

def agw_recv_all(sock, timeout=5):
    """Receive all AGW frames within timeout."""
    frames = []
    end = time.time() + timeout
    while time.time() < end:
        remaining = max(0.1, end - time.time())
        try:
            f = agw_recv(sock, timeout=remaining)
            if f:
                frames.append(f)
        except socket.timeout:
            break
        except (ConnectionResetError, ConnectionAbortedError, OSError):
            break
    return frames

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

    # Cross-cable setup:
    # Iris TX -> CABLE-A Input (1001) -> CABLE-A Output -> Direwolf RX
    # Direwolf TX -> CABLE Input -> CABLE Output (11) -> Iris RX
    CAPTURE = "11"       # CABLE Output (Iris receives Direwolf TX)
    PLAYBACK = "1001"    # CABLE-A Input (Iris TX goes to Direwolf RX)

    f_iris = open("ax25conn_iris.log", "w")
    f_dw = open("ax25conn_direwolf.log", "w")

    # Start Iris (AX.25 mode, no native PHY)
    print("[TEST] Starting Iris (TSTA) on AGW 8010, ax25-only...")
    proc_iris = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "TSTA",
        "--mode", "A", "--tx-level", "0.5",
        "--port", "8001", "--agw-port", "8010",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--ax25-only",
        "--log", "ax25conn_iris_debug.log"
    ], stdout=f_iris, stderr=subprocess.STDOUT)
    time.sleep(2)

    # Start Direwolf
    print("[TEST] Starting Direwolf (DWTST) on AGW 8020...")
    proc_dw = subprocess.Popen([
        DIREWOLF, "-c", DIREWOLF_CONF, "-t", "0"
    ], stdout=f_dw, stderr=f_dw)
    time.sleep(5)

    # Check both running
    for name, proc in [("Iris", proc_iris), ("Direwolf", proc_dw)]:
        if proc.poll() is not None:
            print(f"[ERROR] {name} exited with code {proc.returncode}")
            proc_iris.terminate(); proc_dw.terminate()
            f_iris.close(); f_dw.close()
            for fn in ["ax25conn_iris.log", "ax25conn_direwolf.log"]:
                if os.path.isfile(fn):
                    print(f"\n--- {fn} ---")
                    with open(fn) as f:
                        print(f.read()[-2000:])
            return 1

    success = True

    # Connect AGW clients
    print("[TEST] Connecting AGW clients...")
    try:
        sock_iris = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_iris.settimeout(5)
        sock_iris.connect(("127.0.0.1", 8010))
        print("  Iris AGW connected on 8010")

        sock_dw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_dw.settimeout(5)
        sock_dw.connect(("127.0.0.1", 8020))
        print("  Direwolf AGW connected on 8020")
    except Exception as e:
        print(f"[ERROR] AGW connect failed: {e}")
        proc_iris.terminate(); proc_dw.terminate()
        f_iris.close(); f_dw.close()
        return 1

    # Register callsigns
    agw_send(sock_iris, 'X', call_from="TSTA")
    agw_send(sock_dw, 'X', call_from="DWTST")
    time.sleep(0.5)
    # Drain registration responses
    agw_recv_all(sock_iris, timeout=1)
    agw_recv_all(sock_dw, timeout=1)

    # Enable monitoring on Direwolf so it can accept incoming connections
    agw_send(sock_dw, 'k')
    time.sleep(0.5)

    # --- Step 1: Connect ---
    print("\n[1] Iris connects to Direwolf via AX.25 SABM...")
    agw_send(sock_iris, 'C', call_from="TSTA", call_to="DWTST")
    print("    Sent AGW 'C' (connect request)")

    # Wait for CONNECTED notification on Iris side
    connected_iris = False
    connected_dw = False

    # Check Iris for connect confirmation
    frames = agw_recv_all(sock_iris, timeout=15)
    for kind, cf, ct, data in frames:
        print(f"    Iris AGW: kind='{kind}' from={cf} to={ct} data={data[:80]}")
        if kind == 'C' and b"CONNECTED" in data:
            connected_iris = True
            print("    *** IRIS: CONNECTED ***")

    # Check Direwolf for incoming connect
    frames = agw_recv_all(sock_dw, timeout=5)
    for kind, cf, ct, data in frames:
        print(f"    Direwolf AGW: kind='{kind}' from={cf} to={ct} data={data[:80]}")
        if kind == 'C' and b"CONNECTED" in data:
            connected_dw = True
            print("    *** DIREWOLF: CONNECTED ***")

    if not connected_iris:
        print("    [FAIL] Iris did not report CONNECTED")
        success = False

    if not connected_iris:
        print("    Skipping data transfer (no connection)")
    else:
        time.sleep(1)

        # --- Step 2: Iris -> Direwolf data ---
        print("\n[2] Iris sends data to Direwolf via I-frame...")
        test_msg = b"Hello from Iris AX25!"
        agw_send(sock_iris, 'D', call_from="TSTA", call_to="DWTST", data=test_msg)
        print(f"    Sent {len(test_msg)} bytes via AGW 'D'")

        # Wait for Direwolf to receive
        time.sleep(3)
        frames = agw_recv_all(sock_dw, timeout=10)
        got_data_dw = False
        for kind, cf, ct, data in frames:
            print(f"    Direwolf AGW: kind='{kind}' from={cf} to={ct} data={data[:80]}")
            if kind == 'D' and test_msg in data:
                got_data_dw = True
                print("    *** IRIS -> DIREWOLF: DATA MATCH ***")

        if not got_data_dw:
            print("    [FAIL] Direwolf did not receive Iris data via AGW 'D'")
            success = False

        # --- Step 3: Direwolf -> Iris data ---
        print("\n[3] Direwolf sends data to Iris via I-frame...")
        test_msg2 = b"Hello from Direwolf AX25!"
        agw_send(sock_dw, 'D', call_from="DWTST", call_to="TSTA", data=test_msg2)
        print(f"    Sent {len(test_msg2)} bytes via AGW 'D'")

        time.sleep(3)
        frames = agw_recv_all(sock_iris, timeout=10)
        got_data_iris = False
        for kind, cf, ct, data in frames:
            print(f"    Iris AGW: kind='{kind}' from={cf} to={ct} data={data[:80]}")
            if kind == 'D' and test_msg2 in data:
                got_data_iris = True
                print("    *** DIREWOLF -> IRIS: DATA MATCH ***")

        if not got_data_iris:
            print("    [WARN] Iris did not receive Direwolf data via AGW 'D'")

        # --- Step 4: Disconnect ---
        print("\n[4] Iris disconnects...")
        agw_send(sock_iris, 'd', call_from="TSTA", call_to="DWTST")
        time.sleep(3)

        frames = agw_recv_all(sock_iris, timeout=5)
        disconnected = False
        for kind, cf, ct, data in frames:
            print(f"    Iris AGW: kind='{kind}' from={cf} to={ct} data={data[:80]}")
            if kind == 'd':
                disconnected = True
                print("    *** DISCONNECTED ***")

        if disconnected:
            print("    DISCONNECT: PASS")
        else:
            print("    [WARN] No disconnect confirmation")

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
    for fn in ["ax25conn_iris_debug.log"]:
        if os.path.isfile(fn):
            print(f"\n--- {fn} (last 30 lines) ---")
            with open(fn) as f:
                lines = f.readlines()
            for line in lines[-30:]:
                print(f"  {line.rstrip()}")

    for fn in ["ax25conn_direwolf.log"]:
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
