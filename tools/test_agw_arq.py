#!/usr/bin/env python3
"""Test AGW→ARQ data flow between two Iris instances on VB-Cable.

Station A (commander): AGW connect to B, send data, check for response.
Station B (responder): receives data via AGW, echoes back.
"""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

def build_agw_header(kind, call_from="", call_to="", data_len=0, pid=0xF0):
    hdr = bytearray(36)
    hdr[0] = 0  # port
    hdr[4] = ord(kind)
    hdr[6] = pid
    cf = call_from.encode()[:9]
    ct = call_to.encode()[:9]
    hdr[8:8+len(cf)] = cf
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)

def send_agw(sock, kind, call_from="", call_to="", data=b""):
    hdr = build_agw_header(kind, call_from, call_to, len(data))
    sock.sendall(hdr + data)

def recv_agw(sock, timeout=15):
    """Receive one AGW frame with timeout."""
    sock.settimeout(timeout)
    try:
        hdr = b""
        while len(hdr) < 36:
            chunk = sock.recv(36 - len(hdr))
            if not chunk:
                return None, None, None
            hdr += chunk
        kind = chr(hdr[4])
        data_len = struct.unpack_from('<I', hdr, 28)[0]
        call_from = hdr[8:18].split(b'\0')[0].decode().strip()
        call_to = hdr[18:28].split(b'\0')[0].decode().strip()
        data = b""
        if data_len > 0:
            while len(data) < data_len:
                chunk = sock.recv(data_len - len(data))
                if not chunk:
                    break
                data += chunk
        return kind, (call_from, call_to), data
    except socket.timeout:
        return None, None, None

def wait_for_kind(sock, target_kind, timeout=30):
    """Wait for a specific AGW frame kind."""
    start = time.time()
    while time.time() - start < timeout:
        remaining = timeout - (time.time() - start)
        kind, calls, data = recv_agw(sock, timeout=max(1, remaining))
        if kind is None:
            continue
        print(f"  AGW rx: kind='{kind}' {calls} data={data[:50] if data else b''}")
        if kind == target_kind:
            return kind, calls, data
    return None, None, None


def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(0.5)

    CAPTURE = "11"
    PLAYBACK = "1005"

    fa = open("agw_a_stdout.log", "w")
    fb = open("agw_b_stdout.log", "w")

    print("[TEST] Starting Station A (STN-A) on AGW 8000...")
    proc_a = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-A",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8001", "--agw-port", "8000",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--log", "agw_a.log"
    ], stdout=fa, stderr=subprocess.STDOUT)

    time.sleep(3)

    print("[TEST] Starting Station B (STN-B) on AGW 8010...")
    proc_b = subprocess.Popen([
        IRIS, "--nogui", "--callsign", "STN-B",
        "--mode", "A", "--tx-level", "0.4",
        "--port", "8002", "--agw-port", "8010",
        "--capture", CAPTURE, "--playback", PLAYBACK,
        "--log", "agw_b.log"
    ], stdout=fb, stderr=subprocess.STDOUT)

    time.sleep(4)

    # Verify both running
    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"[ERROR] Station {name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            return 1

    success = True

    # Connect AGW clients
    print("[TEST] Connecting AGW clients...")
    try:
        agw_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        agw_a.connect(("127.0.0.1", 8000))

        agw_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        agw_b.connect(("127.0.0.1", 8010))
    except Exception as e:
        print(f"[ERROR] AGW connect failed: {e}")
        proc_a.terminate(); proc_b.terminate()
        return 1

    # Register callsigns
    print("[TEST] Registering callsigns...")
    send_agw(agw_a, 'X', "STN-A")
    send_agw(agw_b, 'X', "STN-B")
    time.sleep(0.5)

    # A initiates connect to B
    print("\n[1] A connects to B via ARQ...")
    send_agw(agw_a, 'C', "STN-A", "STN-B")

    # Wait for CONNECTED notification on A
    print("  Waiting for CONNECTED on A...")
    kind, calls, data = wait_for_kind(agw_a, 'C', timeout=20)
    if kind == 'C' and data and b"CONNECTED" in data:
        print(f"  A: {data.decode(errors='replace').strip()}")
    else:
        print(f"  A: connect failed (kind={kind}, data={data})")
        success = False

    # Also check B gets CONNECTED notification
    kind_b, calls_b, data_b = wait_for_kind(agw_b, 'C', timeout=5)
    if kind_b == 'C' and data_b and b"CONNECTED" in data_b:
        print(f"  B: {data_b.decode(errors='replace').strip()}")
    else:
        print(f"  B: no connect notification (kind={kind_b})")

    if not success:
        print("[FAIL] Connection not established")
    else:
        # Send data A→B
        print("\n[2] A sends data to B...")
        test_data = b"Hello from STN-A via ARQ!"
        send_agw(agw_a, 'D', "STN-A", "STN-B", test_data)

        # Wait for data on B
        print("  Waiting for data on B...")
        kind, calls, data = wait_for_kind(agw_b, 'D', timeout=20)
        if kind == 'D' and data:
            print(f"  B received: {data}")
            if test_data in data:
                print("  *** DATA MATCH ***")
            else:
                print(f"  Data mismatch: expected {test_data}")
                success = False
        else:
            print("  B: no data received")
            success = False

        # Send data B→A
        print("\n[3] B sends data to A...")
        reply_data = b"Reply from STN-B!"
        send_agw(agw_b, 'D', "STN-B", "STN-A", reply_data)

        # Wait for data on A
        print("  Waiting for data on A...")
        kind, calls, data = wait_for_kind(agw_a, 'D', timeout=20)
        if kind == 'D' and data:
            print(f"  A received: {data}")
            if reply_data in data:
                print("  *** DATA MATCH ***")
            else:
                print(f"  Data mismatch: expected {reply_data}")
                success = False
        else:
            print("  A: no data received")
            success = False

        # Disconnect
        print("\n[4] A disconnects...")
        send_agw(agw_a, 'd', "STN-A", "STN-B")
        kind, calls, data = wait_for_kind(agw_a, 'd', timeout=10)
        if kind == 'd':
            print(f"  A: {data.decode(errors='replace').strip() if data else 'disconnected'}")
        else:
            print("  A: no disconnect notification")

    # Cleanup
    print("\n[TEST] Cleaning up...")
    try: agw_a.close()
    except: pass
    try: agw_b.close()
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
    for name, logfile in [("A", "agw_a.log"), ("B", "agw_b.log")]:
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
