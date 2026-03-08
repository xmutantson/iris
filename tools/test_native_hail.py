#!/usr/bin/env python3
"""Test native hail and Direwolf interop over VB-Cable.

Test 1: Two Iris instances with --native-hail, verify ARQ CONNECTED via AGW.
Test 2: Iris (ax25-only) <-> Direwolf over AFSK 1200, verify bidirectional decode.

Single VB-Cable shared half-duplex channel:
  CABLE Input (1005) -> CABLE Output (11)
"""
import subprocess, socket, struct, time, sys, os, threading

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
DIREWOLF = r"C:\tmp\dw-src\build\src\direwolf.exe"
if not os.path.isfile(DIREWOLF):
    DIREWOLF = r"C:\tmp\dw-build\src\direwolf.exe"

# VB-Cable device IDs (Iris WASAPI enumeration)
CAPTURE  = "11"    # CABLE Output
PLAYBACK = "1005"  # CABLE Input

KISS_FEND = 0xC0

def kiss_encode(data, port=0, cmd=0):
    frame = bytearray([KISS_FEND, (port << 4) | cmd])
    for b in data:
        if b == KISS_FEND:
            frame.extend([0xDB, 0xDC])
        elif b == 0xDB:
            frame.extend([0xDB, 0xDD])
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
        elif b == 0xDB:
            escaped = True
        elif escaped:
            if b == 0xDC: data.append(KISS_FEND)
            elif b == 0xDD: data.append(0xDB)
            escaped = False
        elif not got_cmd:
            cmd = b
            got_cmd = True
        else:
            data.append(b)
    return frames

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

def recv_agw(sock, timeout=20):
    """Receive AGW frames, return list of (kind, from, to, data)."""
    frames = []
    sock.settimeout(1)
    start = time.time()
    buf = b""
    while time.time() - start < timeout:
        try:
            chunk = sock.recv(4096)
            if chunk:
                buf += chunk
                while len(buf) >= 36:
                    kind = chr(buf[4])
                    call_from = buf[8:18].split(b'\x00')[0].decode(errors='replace')
                    call_to = buf[18:28].split(b'\x00')[0].decode(errors='replace')
                    data_len = struct.unpack_from('<I', buf, 28)[0]
                    if len(buf) < 36 + data_len:
                        break
                    data = buf[36:36+data_len]
                    buf = buf[36+data_len:]
                    frames.append((kind, call_from, call_to, data))
        except socket.timeout:
            if frames:
                return frames
            continue
    return frames

def build_ax25_ui(src, dst, payload):
    frame = bytearray()
    for c in dst.ljust(6)[:6]:
        frame.append(ord(c) << 1)
    frame.append(0x60)
    for c in src.ljust(6)[:6]:
        frame.append(ord(c) << 1)
    frame.append(0x61)
    frame.append(0x03)  # UI
    frame.append(0xF0)  # No L3
    frame.extend(payload)
    return bytes(frame)

def kill_procs():
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    os.system("taskkill /F /IM direwolf.exe 2>nul >nul")
    time.sleep(1)

def tail_log(path, n=25):
    if os.path.isfile(path):
        with open(path) as f:
            lines = f.readlines()
        for line in lines[-n:]:
            print(f"  {line.rstrip()}")

# =====================================================================
# Test 1: Native Hail (Iris <-> Iris)
# =====================================================================
def test_native_hail():
    print("=" * 60)
    print("TEST 1: Native Hail (Iris <-> Iris over VB-Cable)")
    print("  STN-A: ax25-baud 9600 (blocks 1200 AFSK decode, forces native escalation)")
    print("  STN-B: ax25-baud 1200 (default)")
    print("=" * 60)
    kill_procs()

    log_a = "test_nhail_a.log"
    log_b = "test_nhail_b.log"

    # Instance A: listener with native-hail, ax25-baud 9600
    # Using 9600 baud means A won't decode B's 1200 AFSK SABMs,
    # forcing B to escalate to native BPSK hailing after 3 failures.
    print("[A] Starting Iris listener (STN-A) with --native-hail --ax25-baud 9600...")
    fa = open(log_a, "w")
    proc_a = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.5",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--port", "8001", "--agw-port", "8000",
        "--native-hail", "--ax25-baud", "9600",
        "--log", "test_nhail_a_debug.log",
    ], stdout=fa, stderr=subprocess.STDOUT)

    time.sleep(3)

    # Instance B: commander with native-hail (default 1200 baud)
    print("[B] Starting Iris commander (STN-B) with --native-hail...")
    fb = open(log_b, "w")
    proc_b = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-B",
        "--mode", "A", "--tx-level", "0.5",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--port", "8011", "--agw-port", "8010",
        "--native-hail",
        "--log", "test_nhail_b_debug.log",
    ], stdout=fb, stderr=subprocess.STDOUT)

    time.sleep(3)

    # Check both are running
    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"[ERROR] Instance {name} exited with code {proc.returncode}")
            kill_procs()
            fa.close(); fb.close()
            tail_log(log_a)
            tail_log(log_b)
            return False

    # Connect to B's AGW and trigger connect to STN-A
    print("[B] Triggering ARQ connect to STN-A via AGW...")
    try:
        agw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        agw.settimeout(5)
        agw.connect(("127.0.0.1", 8010))
        # Register callsign
        send_agw(agw, 'X', "STN-B")
        time.sleep(0.5)
        try:
            agw.settimeout(0.5)
            agw.recv(4096)
        except:
            pass
        # Connect
        send_agw(agw, 'C', "STN-B", "STN-A")
        print("[B] Connect command sent, waiting for ARQ CONNECTED...")
    except Exception as e:
        print(f"[ERROR] AGW connect failed: {e}")
        kill_procs()
        fa.close(); fb.close()
        return False

    # Wait for connection (look for 'C' frame = connected notification)
    # Longer timeout: 3 AFSK retries (~9s) + native hail escalation (~35s with 3.5s interval)
    connected = False
    frames = recv_agw(agw, timeout=60)
    for kind, cf, ct, data in frames:
        print(f"  AGW: kind='{kind}' from={cf} to={ct} data={data[:50]}")
        if kind == 'C' and b'CONNECTED' in data.upper():
            connected = True
            break

    if connected:
        print("*** NATIVE HAIL: CONNECTED ***")
    else:
        print("*** NATIVE HAIL: FAILED TO CONNECT ***")
        # Check debug logs for clues
        print("\n--- STN-A debug log ---")
        tail_log("test_nhail_a_debug.log", 30)
        print("\n--- STN-B debug log ---")
        tail_log("test_nhail_b_debug.log", 30)

    try:
        agw.close()
    except:
        pass
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

    if not connected:
        print("\n--- STN-A debug log ---")
        tail_log("test_nhail_a_debug.log", 30)
        print("\n--- STN-B debug log ---")
        tail_log("test_nhail_b_debug.log", 30)

    return connected

# =====================================================================
# Test 2: Direwolf Interop (Iris AFSK <-> Direwolf)
# =====================================================================
def test_direwolf_interop():
    print("\n" + "=" * 60)
    print("TEST 2: Direwolf Interop (Iris AFSK <-> Direwolf)")
    print("=" * 60)

    if not os.path.isfile(DIREWOLF):
        print(f"[SKIP] Direwolf not found: {DIREWOLF}")
        return None

    kill_procs()

    # Cross-cable setup to avoid VB-Cable single-reader conflict:
    # Iris TX -> CABLE-A Input (1001) -> CABLE-A Output -> Direwolf RX
    # Direwolf TX -> CABLE Input -> CABLE Output (11) -> Iris RX
    IRIS_CAPTURE  = "11"    # CABLE Output (receives Direwolf TX)
    IRIS_PLAYBACK = "1001"  # CABLE-A Input (Iris TX goes to Direwolf RX)

    # Write minimal Direwolf config
    # Cross-cable: DW captures from CABLE-A Output (Iris TX), plays to CABLE Input (-> Iris RX)
    # Use PortAudio device numbers: 7=CABLE-A Output, 5=CABLE Input
    dw_conf = r"C:\tmp\direwolf_test.conf"
    with open(dw_conf, "w") as f:
        f.write(f"""# Direwolf test config
MYCALL DWTST
ADEVICE 7 5
ARATE 48000
ACHANNELS 1
CHANNEL 0
MODEM 1200
KISSPORT 8021
AGWPORT 0
""")

    log_iris = "test_dw_iris.log"

    # Start Iris in ax25-only mode
    print("[IRIS] Starting Iris (STN-A) in ax25-only mode...")
    fi = open(log_iris, "w")
    proc_iris = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.5",
        "--capture", IRIS_CAPTURE, "--playback", IRIS_PLAYBACK,
        "--port", "8031", "--agw-port", "8030",
        "--ax25-only",
        "--log", "test_dw_iris_debug.log",
    ], stdout=fi, stderr=subprocess.STDOUT)

    time.sleep(2)

    # Start Direwolf
    print("[DW] Starting Direwolf (DWTST)...")
    fd = open("test_dw_direwolf.log", "w")
    proc_dw = subprocess.Popen([
        DIREWOLF, "-c", dw_conf, "-t", "0"
    ], stdout=fd, stderr=fd,
       stdin=subprocess.DEVNULL,
       creationflags=subprocess.CREATE_NO_WINDOW if hasattr(subprocess, 'CREATE_NO_WINDOW') else 0)

    # Wait for Direwolf KISS port to be ready
    print("  Waiting for Direwolf KISS port 8021...")
    for attempt in range(20):
        time.sleep(1)
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            probe.settimeout(1)
            probe.connect(("127.0.0.1", 8021))
            probe.close()
            print(f"  Direwolf KISS ready after {attempt+1}s")
            break
        except:
            if proc_dw.poll() is not None:
                print(f"[ERROR] Direwolf exited with code {proc_dw.returncode}")
                break
    else:
        print("[ERROR] Direwolf KISS port 8021 never opened")

    for name, proc in [("Iris", proc_iris), ("Direwolf", proc_dw)]:
        if proc.poll() is not None:
            print(f"[ERROR] {name} exited with code {proc.returncode}")
            kill_procs()
            fi.close(); fd.close()
            tail_log("test_dw_direwolf.log", 20)
            return False

    # Connect KISS clients
    success_iris_to_dw = False
    success_dw_to_iris = False

    try:
        sock_iris = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_iris.settimeout(5)
        sock_iris.connect(("127.0.0.1", 8031))
        print("  Iris KISS connected")


        sock_dw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_dw.settimeout(5)
        sock_dw.connect(("127.0.0.1", 8021))
        print("  KISS clients connected")
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        kill_procs()
        fi.close(); fd.close()
        return False

    # Test: Iris -> Direwolf
    print("\n[1] Iris -> Direwolf (AX.25 UI frame)...")
    ax25 = build_ax25_ui("STN-A", "DWTST", b"Hello from Iris!")
    sock_iris.sendall(kiss_encode(ax25))

    sock_dw.settimeout(10)
    rx_buf = b""
    start = time.time()
    while time.time() - start < 10:
        try:
            data = sock_dw.recv(4096)
            if data:
                rx_buf += data
                frames = kiss_decode(rx_buf)
                if frames:
                    for cmd, payload in frames:
                        if b"Hello from Iris!" in payload:
                            print("    *** IRIS -> DIREWOLF: PASS ***")
                            success_iris_to_dw = True
                    break
        except socket.timeout:
            continue

    if not success_iris_to_dw:
        print("    IRIS -> DIREWOLF: FAIL (no response)")

    time.sleep(2)

    # Test: Direwolf -> Iris
    print("[2] Direwolf -> Iris (AX.25 UI frame)...")
    ax25_dw = build_ax25_ui("DWTST", "STN-A", b"Hello from Direwolf!")
    sock_dw.sendall(kiss_encode(ax25_dw))

    sock_iris.settimeout(10)
    rx_buf = b""
    start = time.time()
    while time.time() - start < 10:
        try:
            data = sock_iris.recv(4096)
            if data:
                rx_buf += data
                frames = kiss_decode(rx_buf)
                if frames:
                    for cmd, payload in frames:
                        if b"Hello from Direwolf!" in payload:
                            print("    *** DIREWOLF -> IRIS: PASS ***")
                            success_dw_to_iris = True
                    break
        except socket.timeout:
            continue

    if not success_dw_to_iris:
        print("    DIREWOLF -> IRIS: FAIL (no response)")

    try:
        sock_iris.close()
        sock_dw.close()
    except:
        pass

    proc_iris.terminate()
    proc_dw.terminate()
    try:
        proc_iris.wait(timeout=3)
        proc_dw.wait(timeout=3)
    except:
        proc_iris.kill()
        proc_dw.kill()
    fi.close()
    fd.close()

    if not (success_iris_to_dw and success_dw_to_iris):
        print("\n--- Iris debug log ---")
        tail_log("test_dw_iris_debug.log", 20)
        print("\n--- Direwolf log ---")
        tail_log("test_dw_direwolf.log", 20)

    return success_iris_to_dw and success_dw_to_iris


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    r1 = test_native_hail()
    r2 = test_direwolf_interop()

    print("\n" + "=" * 60)
    print("RESULTS:")
    print(f"  Native Hail:      {'PASS' if r1 else 'FAIL'}")
    print(f"  Direwolf Interop: {'PASS' if r2 else 'SKIP' if r2 is None else 'FAIL'}")
    print("=" * 60)

    sys.exit(0 if (r1 and r2 is not False) else 1)
