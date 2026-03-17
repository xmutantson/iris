#!/usr/bin/env python3
"""Test native hail frame encode/decode via internal loopback.

Starts a single Iris instance with --loopback and --native-hail,
triggers ARQ connect via AGW, and verifies the native frame round-trip.
This tests the complete native hail TX/RX pipeline without needing
two VB-Cable devices.
"""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

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

def recv_agw(sock, timeout=30):
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

def tail_log(path, n=30):
    if os.path.isfile(path):
        with open(path) as f:
            lines = f.readlines()
        for line in lines[-n:]:
            print(f"  {line.rstrip()}")

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(1)

    log_file = "test_nhail_lb_debug.log"

    print("=" * 60)
    print("Native Hail Loopback Test")
    print("=" * 60)
    print("[1] Starting Iris with --loopback --native-hail...")

    f = open("test_nhail_lb.log", "w")
    proc = subprocess.Popen([
        IRIS, "--loopback", "--nogui", "--callsign", "TEST1",
        "--mode", "A", "--tx-level", "0.5",
        "--port", "8001", "--agw-port", "8000",
        "--native-hail",
        "--log", log_file,
    ], stdout=f, stderr=subprocess.STDOUT)

    time.sleep(3)
    if proc.poll() is not None:
        print(f"[ERROR] Iris exited with code {proc.returncode}")
        f.close()
        tail_log("test_nhail_lb.log")
        return 1

    # Connect AGW and trigger self-connect
    print("[2] Connecting AGW and triggering ARQ connect...")
    try:
        agw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        agw.settimeout(5)
        agw.connect(("127.0.0.1", 8000))
        send_agw(agw, 'X', "TEST1")
        time.sleep(0.5)
        try:
            agw.settimeout(0.5)
            agw.recv(4096)
        except:
            pass
        # Self-connect triggers hail TX -> loopback RX -> decode
        send_agw(agw, 'C', "TEST1", "TEST2")
        print("[3] Connect sent, waiting for native frame decode...")
    except Exception as e:
        print(f"[ERROR] AGW connect failed: {e}")
        proc.terminate()
        f.close()
        return 1

    # Wait for activity — we just need to see frames being sent and decoded
    time.sleep(10)  # Let it run for 10 seconds

    # Check log for native frame activity
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except:
        proc.kill()
    try:
        agw.close()
    except:
        pass
    f.close()

    # Analyze results
    tx_count = 0
    rx_count = 0
    decode_fail = 0
    connected = False

    if os.path.isfile(log_file):
        with open(log_file) as lf:
            for line in lf:
                if "TX native hail frame" in line:
                    tx_count += 1
                if "RX native frame" in line:
                    rx_count += 1
                if "LDPC decode failed" in line:
                    decode_fail += 1
                if "CONNECTED" in line:
                    connected = True

    print(f"\n--- Results ---")
    print(f"  Native TX frames: {tx_count}")
    print(f"  Native RX frames: {rx_count}")
    print(f"  LDPC failures:    {decode_fail}")
    print(f"  Connected:        {connected}")

    if tx_count > 0 and rx_count > 0:
        print(f"\n*** NATIVE HAIL ENCODE/DECODE: PASS ***")
        print(f"    ({rx_count}/{tx_count} frames decoded)")
    elif tx_count > 0 and decode_fail > 0:
        print(f"\n*** NATIVE HAIL: PARTIAL (TX OK, decode failing) ***")
    elif tx_count == 0:
        print(f"\n*** NATIVE HAIL: NO TX (ARQ not hailing) ***")
    else:
        print(f"\n*** NATIVE HAIL: FAIL ***")

    print(f"\n--- Debug log (last 40 lines) ---")
    tail_log(log_file, 40)

    return 0 if (rx_count > 0) else 1


if __name__ == "__main__":
    sys.exit(main())
