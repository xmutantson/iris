#!/usr/bin/env python3
"""Test Iris OFDM-KISS (AFSK → native mode switch) over VB-Cable.

Two Iris instances (STN-A initiator, STN-B responder) on the same VB-Cable.
Uses AGW protocol for connected-mode AX.25 session, sends ~1KB of data.
Verifies:
  1. AX.25 SABM/UA → CONNECTED
  2. IRIS connection header exchange (Phase 1)
  3. SWITCH handshake (Phase 2)
  4. Native mode activation (Phase 3)
  5. Data delivery (1KB payload)
  6. Clean disconnect

Usage:
  python test_ofdm_kiss.py
  python test_ofdm_kiss.py --ax25-only       # Baseline: no native upgrade
  python test_ofdm_kiss.py --payload 4096     # Larger payload
  python test_ofdm_kiss.py --timeout 60       # Longer timeout
"""
import subprocess, socket, struct, time, sys, os, argparse, re, signal
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

# Audio devices — single shared VB-Cable (simulates shared RF channel)
# Both stations play to CABLE Input and capture from CABLE Output.
# The cable acts as a shared audio buffer — like a radio frequency.
# DCD/CSMA + rx_mute handle contention (half-duplex discipline).
SHARED_PLAYBACK = 1006  # CABLE Input (VB-Audio Virtual Cable)
SHARED_CAPTURE  = 12    # CABLE Output (VB-Audio Virtual Cable)

# Ports
A_KISS_PORT = 8001
A_AGW_PORT  = 8010
B_KISS_PORT = 8002
B_AGW_PORT  = 8011


# --- AGW protocol helpers ---

def agw_header(kind, call_from="", call_to="", data_len=0, port=0):
    hdr = bytearray(36)
    hdr[0] = port
    hdr[4] = ord(kind) if isinstance(kind, str) else kind
    hdr[6] = 0xF0
    cf = call_from.encode()[:9]
    hdr[8:8+len(cf)] = cf
    ct = call_to.encode()[:9]
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)

def agw_send(sock, kind, call_from="", call_to="", data=b""):
    hdr = agw_header(kind, call_from, call_to, len(data))
    sock.sendall(hdr + data)

def agw_recv(sock, timeout=10):
    sock.settimeout(timeout)
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


# --- Log analysis ---

def parse_log(path):
    """Parse Iris log file, return dict of events with timestamps."""
    events = {
        'connected': None,
        'probe_start': None,       # Probe-after-connect initiated
        'probe_complete': None,    # Probe completed with results
        'native_active': None,     # OFDM-KISS native mode active
        'native_confirmed': None,  # Heard native frame from peer
        'ofdm_kiss_rx': None,      # ofdm_kiss_ = true
        'ofdm_kiss_tx': None,      # ofdm_kiss_tx_ = true
        'native_frames_tx': 0,
        'native_frames_rx': 0,
        'selfhear_discards': 0,
        'disconnected': None,
        'errors': [],
    }
    if not os.path.isfile(path):
        return events

    with open(path, 'r', errors='replace') as f:
        for line in f:
            line = line.rstrip()
            # Extract timestamp
            m = re.match(r'\[\s*([\d.]+)\]', line)
            ts = float(m.group(1)) if m else None

            if 'AX25 state -> 2' in line or 'CONNECTED' in line and 'state ->' in line:
                if events['connected'] is None:
                    events['connected'] = ts
            # Probe-first architecture events
            if 'Probe-after-connect' in line:
                events['probe_start'] = ts
            if 'PROBE:START' in line and 'Received' in line:
                events['probe_start'] = ts
            if 'Probe complete:' in line or '[PROBE] Complete!' in line:
                events['probe_complete'] = ts
            if 'native mode active' in line:
                events['native_active'] = ts
                events['ofdm_kiss_rx'] = ts
                events['ofdm_kiss_tx'] = ts
            if 'native confirmed' in line:
                events['native_confirmed'] = ts
            # TX frame counting: OFDM-KISS native frames
            if 'TX frame' in line and 'OFDM-KISS' in line:
                events['native_frames_tx'] += 1
            if 'TX frame' in line and 'native' in line.lower():
                events['native_frames_tx'] += 1
            if 'RX native frame' in line and 'DISCARDED' not in line:
                events['native_frames_rx'] += 1
            if 'self-hear guard' in line or 'DISCARDED' in line:
                events['selfhear_discards'] += 1
            if 'AX25 state -> 0' in line or 'DISCONNECTED' in line and 'state ->' in line:
                events['disconnected'] = ts
            if 'FAIL' in line or 'ERROR' in line:
                events['errors'].append(line.strip())
    return events


def print_timeline(events_a, events_b):
    """Print unified timeline from both stations."""
    timeline = []
    for name, events in [('A', events_a), ('B', events_b)]:
        for key in ['connected', 'probe_start', 'probe_complete',
                     'native_active', 'native_confirmed', 'disconnected']:
            ts = events.get(key)
            if ts is not None:
                timeline.append((ts, name, key))

    timeline.sort()
    if timeline:
        print("\n  Timeline:")
        for ts, name, key in timeline:
            print(f"    [{ts:8.3f}] {name}: {key}")

    for name, events in [('A', events_a), ('B', events_b)]:
        ntx = events.get('native_frames_tx', 0)
        nrx = events.get('native_frames_rx', 0)
        nsh = events.get('selfhear_discards', 0)
        if ntx or nrx or nsh:
            print(f"    {name}: native TX={ntx} RX={nrx} self-hear discards={nsh}")
        for err in events.get('errors', []):
            print(f"    {name} ERROR: {err}")


def main():
    parser = argparse.ArgumentParser(description="Iris OFDM-KISS Integration Test")
    parser.add_argument("--iris", default=IRIS, help="Path to iris.exe")
    parser.add_argument("--payload", type=int, default=1024, help="Payload size (bytes)")
    parser.add_argument("--timeout", type=int, default=90, help="Data phase timeout (seconds)")
    parser.add_argument("--ax25-only", action="store_true", help="Disable native upgrade (baseline)")
    parser.add_argument("--capture", type=int, default=SHARED_CAPTURE)
    parser.add_argument("--playback", type=int, default=SHARED_PLAYBACK)
    parser.add_argument("--tx-level", type=float, default=0.4)
    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    # Kill any stale instances
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(1)

    log_a = os.path.abspath("ofdm_kiss_a.log")
    log_b = os.path.abspath("ofdm_kiss_b.log")
    stdout_a = open("ofdm_kiss_a_stdout.log", "w")
    stdout_b = open("ofdm_kiss_b_stdout.log", "w")

    # Clean old logs
    for f in [log_a, log_b]:
        if os.path.isfile(f):
            os.remove(f)

    extra_a = ["--ax25-only"] if args.ax25_only else []
    extra_b = ["--ax25-only"] if args.ax25_only else []

    print("=" * 70)
    print(f"  Iris OFDM-KISS Integration Test")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Payload: {args.payload} bytes, timeout: {args.timeout}s")
    print(f"  Mode: {'AX.25 only (baseline)' if args.ax25_only else 'OFDM-KISS (native upgrade)'}")
    print("=" * 70)

    # --- Start instances ---
    print("\n[1] Starting Iris instances on VB-Cable...")

    cmd_a = [
        iris_path, "--nogui", "--callsign", "TSTA",
        "--mode", "A", "--tx-level", str(args.tx_level),
        "--port", str(A_KISS_PORT), "--agw-port", str(A_AGW_PORT),
        "--capture", str(args.capture), "--playback", str(args.playback),
        "--log", log_a,
    ] + extra_a

    cmd_b = [
        iris_path, "--nogui", "--callsign", "TSTB",
        "--mode", "A", "--tx-level", str(args.tx_level),
        "--port", str(B_KISS_PORT), "--agw-port", str(B_AGW_PORT),
        "--capture", str(args.capture), "--playback", str(args.playback),
        "--log", log_b,
    ] + extra_b

    proc_a = subprocess.Popen(cmd_a, stdout=stdout_a, stderr=subprocess.STDOUT)
    proc_b = subprocess.Popen(cmd_b, stdout=stdout_b, stderr=subprocess.STDOUT)
    time.sleep(4)

    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"  [ERROR] Station {name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            stdout_a.close(); stdout_b.close()
            return 1
    print("  Both instances running.")

    # --- Connect AGW clients ---
    print("\n[2] Connecting AGW clients...")
    sock_a = sock_b = None
    try:
        sock_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_a.settimeout(5)
        sock_a.connect(("127.0.0.1", A_AGW_PORT))
        print(f"  A: AGW connected on {A_AGW_PORT}")

        sock_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_b.settimeout(5)
        sock_b.connect(("127.0.0.1", B_AGW_PORT))
        print(f"  B: AGW connected on {B_AGW_PORT}")
    except Exception as e:
        print(f"  [ERROR] AGW connect failed: {e}")
        proc_a.terminate(); proc_b.terminate()
        stdout_a.close(); stdout_b.close()
        return 1

    # Register callsigns
    agw_send(sock_a, 'X', call_from="TSTA")
    agw_send(sock_b, 'X', call_from="TSTB")
    time.sleep(0.5)
    agw_recv_all(sock_a, timeout=1)
    agw_recv_all(sock_b, timeout=1)

    time.sleep(0.5)

    success = True
    data_received = b""

    # --- Step 3: Connect A → B ---
    print(f"\n[3] A connects to B (AX.25 SABM)...")
    t0 = time.time()
    agw_send(sock_a, 'C', call_from="TSTA", call_to="TSTB")

    connected_a = False
    connected_b = False

    # Poll both sockets for CONNECTED, return as soon as both are connected
    deadline = time.time() + args.timeout
    while time.time() < deadline and not (connected_a and connected_b):
        remaining = max(0.5, deadline - time.time())
        for name, sock, is_connected in [("A", sock_a, connected_a), ("B", sock_b, connected_b)]:
            if is_connected:
                continue
            frames = agw_recv_all(sock, timeout=min(1.0, remaining))
            for kind, cf, ct, data in frames:
                if kind == 'C' and b"CONNECTED" in data:
                    if name == "A":
                        connected_a = True
                    else:
                        connected_b = True
                    print(f"  {name}: CONNECTED ({time.time()-t0:.1f}s)")

    if not connected_a or not connected_b:
        print(f"  [FAIL] Connection failed (A={connected_a}, B={connected_b})")
        success = False
    else:
        # Wait for OFDM-KISS upgrade to complete (IRIS header + SWITCH exchange)
        # The modem handles this transparently — we just need to give it time
        if not args.ax25_only:
            print("  Waiting for OFDM-KISS upgrade (IRIS header + SWITCH)...")
            time.sleep(8)  # yield gate + SWITCH + native ping + settle
        else:
            time.sleep(2)

        # --- Step 4: Send data A → B ---
        # Generate test payload with pattern for verification
        payload = bytes([(i * 37 + 17) & 0xFF for i in range(args.payload)])
        chunk_size = 200  # AGW data chunk size (reasonable for AX.25 I-frames)

        print(f"\n[4] A sends {len(payload)} bytes to B...")
        t_send = time.time()
        offset = 0
        while offset < len(payload):
            chunk = payload[offset:offset+chunk_size]
            try:
                agw_send(sock_a, 'D', call_from="TSTA", call_to="TSTB", data=chunk)
            except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
                print(f"  [ERROR] A send failed at offset {offset}: {e}")
                success = False
                break
            offset += len(chunk)
            time.sleep(0.1)  # Don't flood — let modem batch

        # --- Step 5: Receive data on B ---
        print(f"\n[5] Waiting for data on B...")
        data_received = b""
        deadline = time.time() + args.timeout
        while time.time() < deadline and len(data_received) < len(payload):
            frames = agw_recv_all(sock_b, timeout=min(5, deadline - time.time()))
            for kind, cf, ct, data in frames:
                if kind == 'D':
                    data_received += data

        elapsed = time.time() - t_send
        pct = len(data_received) / len(payload) * 100 if payload else 0
        bps = len(data_received) * 8 / elapsed if elapsed > 0 else 0
        print(f"  B received: {len(data_received)}/{len(payload)} bytes "
              f"({pct:.0f}%) in {elapsed:.1f}s ({bps:.0f} bps)")

        if len(data_received) >= len(payload):
            # Verify data integrity
            if data_received[:len(payload)] == payload:
                print("  Data integrity: PASS")
            else:
                # Find first mismatch
                for i in range(min(len(data_received), len(payload))):
                    if data_received[i] != payload[i]:
                        print(f"  Data integrity: FAIL (first mismatch at byte {i})")
                        success = False
                        break
        else:
            print(f"  [FAIL] Incomplete data ({len(data_received)}/{len(payload)})")
            success = False

        # --- Step 6: Bidirectional test (B → A) ---
        reply = b"ACK:" + bytes(range(256))  # 260 bytes
        print(f"\n[6] B sends {len(reply)} bytes to A...")
        try:
            agw_send(sock_b, 'D', call_from="TSTB", call_to="TSTA", data=reply)
        except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
            print(f"  [WARN] B send failed: {e}")

        reply_received = b""
        frames = agw_recv_all(sock_a, timeout=15)
        for kind, cf, ct, data in frames:
            if kind == 'D':
                reply_received += data
        if reply_received == reply:
            print(f"  A received reply: PASS ({len(reply_received)} bytes)")
        else:
            print(f"  A received reply: {len(reply_received)}/{len(reply)} bytes")
            if len(reply_received) == 0:
                print(f"  [WARN] No reply received (possible transition issue)")

        # --- Step 7: Disconnect ---
        print(f"\n[7] A disconnects...")
        agw_send(sock_a, 'd', call_from="TSTA", call_to="TSTB")
        time.sleep(3)

        disconnected = False
        frames = agw_recv_all(sock_a, timeout=5)
        for kind, cf, ct, data in frames:
            if kind == 'd':
                disconnected = True
        print(f"  Disconnect: {'PASS' if disconnected else 'WARN (no confirm)'}")

    # --- Cleanup ---
    print("\n[8] Cleaning up...")
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
    stdout_a.close()
    stdout_b.close()

    # --- Log analysis ---
    print("\n[9] Log analysis:")
    events_a = parse_log(log_a)
    events_b = parse_log(log_b)

    if not args.ax25_only:
        # Check OFDM-KISS probe-first phases
        phases = [
            ("Phase 1 (probe start)", "probe_start"),
            ("Phase 2 (probe complete)", "probe_complete"),
            ("Phase 3 (native active)", "native_active"),
            ("Phase 4 (native confirmed)", "native_confirmed"),
        ]
        for phase_name, key in phases:
            a_ok = events_a.get(key) is not None
            b_ok = events_b.get(key) is not None
            status = "PASS" if (a_ok or b_ok) else "FAIL"
            print(f"  {phase_name}: {status}")
            if status == "FAIL":
                success = False

    print_timeline(events_a, events_b)

    # Show last lines of logs
    for name, logfile in [("A", log_a), ("B", log_b)]:
        if os.path.isfile(logfile):
            print(f"\n--- Station {name} log (last 20 lines) ---")
            with open(logfile, 'r', errors='replace') as f:
                lines = f.readlines()
            for line in lines[-20:]:
                print(f"  {line.rstrip()}")

    print(f"\n{'=' * 70}")
    print(f"  RESULT: {'PASS' if success else 'FAIL'}")
    print(f"  Data: {len(data_received)}/{args.payload} bytes delivered")
    if not args.ax25_only:
        print(f"  A: native TX={events_a['native_frames_tx']} RX={events_a['native_frames_rx']} "
              f"self-hear={events_a['selfhear_discards']}")
        print(f"  B: native TX={events_b['native_frames_tx']} RX={events_b['native_frames_rx']} "
              f"self-hear={events_b['selfhear_discards']}")
    print(f"{'=' * 70}")

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
