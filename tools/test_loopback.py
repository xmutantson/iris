#!/usr/bin/env python3
"""Quick test: send a KISS frame through internal loopback, check if it comes back decoded."""
import subprocess, socket, struct, time, sys, os

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

# KISS framing
KISS_FEND  = 0xC0
KISS_FESC  = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

def kiss_encode(data, port=0, cmd=0):
    """Encode data into a KISS frame."""
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
    """Decode KISS frames from raw bytes. Returns list of (cmd, data)."""
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

def main():
    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    # Kill any existing iris
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(0.5)

    print("[TEST] Starting iris.exe --loopback...")
    proc = subprocess.Popen([
        IRIS, "--loopback", "--nogui", "--callsign", "TEST1",
        "--mode", "A", "--tx-level", "0.5", "--ptt-pre", "100", "--ptt-post", "50",
        "--log", "test_loopback.log"
    ], creationflags=subprocess.CREATE_NEW_CONSOLE)

    time.sleep(3)

    # Connect to KISS port
    print("[TEST] Connecting to KISS port 8001...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect(("127.0.0.1", 8001))
    except Exception as e:
        print(f"[ERROR] KISS connect failed: {e}")
        proc.terminate()
        return 1

    # Send a test payload via KISS
    test_payload = b"Hello Iris Loopback Test! " + bytes(range(256))
    kiss_frame = kiss_encode(test_payload)
    print(f"[TEST] Sending {len(test_payload)} byte KISS frame...")
    sock.sendall(kiss_frame)

    # Wait for it to come back (TX + 2s delay + buffer accumulation + RX decode)
    print("[TEST] Waiting up to 20s for loopback response...")
    sock.settimeout(20)
    rx_buf = b""
    start = time.time()
    got_response = False
    while time.time() - start < 20:
        try:
            data = sock.recv(4096)
            if data:
                rx_buf += data
                frames = kiss_decode(rx_buf)
                if frames:
                    print(f"[TEST] Received {len(frames)} KISS frame(s)!")
                    for i, (cmd, payload) in enumerate(frames):
                        print(f"  Frame {i}: cmd=0x{cmd:02x}, {len(payload)} bytes")
                        if payload[:20] == test_payload[:20]:
                            print(f"  *** MATCH! Loopback verified ***")
                            got_response = True
                    if got_response:
                        break
        except socket.timeout:
            elapsed = time.time() - start
            print(f"  [{elapsed:.1f}s] waiting... (rx_buf={len(rx_buf)} bytes)")
            continue

    if not got_response:
        print("[TEST] No loopback response received.")
        print(f"  rx_buf: {len(rx_buf)} bytes")

    # Read log for diagnostics
    time.sleep(1)
    sock.close()
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except:
        proc.kill()

    if os.path.isfile("test_loopback.log"):
        print("\n--- Modem log (last 30 lines) ---")
        with open("test_loopback.log") as f:
            lines = f.readlines()
        for line in lines[-30:]:
            print(f"  {line.rstrip()}")

    return 0 if got_response else 1


if __name__ == "__main__":
    sys.exit(main())
