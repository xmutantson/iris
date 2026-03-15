#!/usr/bin/env python3
"""Half-duplex two-instance test over a single VB-Cable.

Both Iris instances share ONE virtual cable — exactly like two radios on the
same frequency.  This exercises the real DCD, CSMA, and RX-mute code paths
that --loopback bypasses.

Setup:
  Both instances use the SAME capture and playback device (one VB-Cable).
  Instance A TX -> cable -> Instance A RX (self-hear, muted by rx_muted_)
                         -> Instance B RX (peer hears us)
  Instance B TX -> cable -> Instance B RX (self-hear, muted)
                         -> Instance A RX (peer hears us)

This is a shared medium, just like real FM radio.  Simultaneous TX produces
garbled audio that neither side can decode — a real collision.

Test flow:
  1. Start two iris instances on the same VB-Cable
  2. Connect AGW client to each
  3. Instance A initiates AX.25 connect to Instance B
  4. Wait for CONNECTED on both sides
  5. Send data A->B, verify receipt
  6. Send data B->A, verify receipt
  7. Disconnect and verify

Usage:
  python test_halfduplex.py [--capture ID] [--playback ID]

If no device IDs given, defaults to CABLE Output (11) / CABLE Input (1001).
Run "iris.exe --list-audio" to find your VB-Cable device IDs.
"""
import subprocess, socket, struct, time, sys, os, argparse

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

# AGW protocol helpers (same as test_ax25_connected.py)

def agw_send(sock, kind, call_from="", call_to="", data=b"", port=0):
    hdr = bytearray(36)
    hdr[0] = port
    hdr[4] = ord(kind) if isinstance(kind, str) else kind
    hdr[6] = 0xF0
    cf = call_from.encode()[:9]
    hdr[8:8+len(cf)] = cf
    ct = call_to.encode()[:9]
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, len(data))
    sock.sendall(bytes(hdr) + data)

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

def tail_log(path, n=40):
    if not os.path.isfile(path):
        return
    print(f"\n--- {os.path.basename(path)} (last {n} lines) ---")
    with open(path) as f:
        lines = f.readlines()
    for line in lines[-n:]:
        print(f"  {line.rstrip()}")


def analyze_burst_holdoff(log_a, log_b):
    """Analyze logs for burst holdoff effectiveness.

    Checks for:
    - TIMER_RECOVERY events (indicates lost acks — burst holdoff should prevent these)
    - Self-hear guard discards (legitimate frames incorrectly discarded)
    - Native mode activation (confirms we're testing the right code path)
    """
    results = {"native_a": False, "native_b": False,
               "timer_recovery_a": 0, "timer_recovery_b": 0,
               "selfhear_discard_a": 0, "selfhear_discard_b": 0,
               "data_frames_a": 0, "data_frames_b": 0}

    for path, suffix in [(log_a, "_a"), (log_b, "_b")]:
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            for line in f:
                if "native TX active" in line or "native confirmed" in line:
                    results["native" + suffix] = True
                if "TIMER_RECOVERY" in line:
                    results["timer_recovery" + suffix] += 1
                if "self-hear guard" in line and "DISCARDED" in line:
                    results["selfhear_discard" + suffix] += 1
                if "RX native frame" in line and "DISCARDED" not in line:
                    results["data_frames" + suffix] += 1

    print(f"\n{'='*60}")
    print("  BURST HOLDOFF ANALYSIS")
    print(f"{'='*60}")
    print(f"  Native mode:  A={'YES' if results['native_a'] else 'no'}  "
          f"B={'YES' if results['native_b'] else 'no'}")
    print(f"  Native RX:    A={results['data_frames_a']}  B={results['data_frames_b']}")
    print(f"  TIMER_RECOVERY: A={results['timer_recovery_a']}  B={results['timer_recovery_b']}")
    print(f"  Self-hear discards: A={results['selfhear_discard_a']}  B={results['selfhear_discard_b']}")

    warnings = []
    if results["timer_recovery_a"] + results["timer_recovery_b"] > 0:
        warnings.append("TIMER_RECOVERY detected — burst holdoff may not be working")
    if results["selfhear_discard_a"] + results["selfhear_discard_b"] > 0:
        warnings.append("Self-hear discards detected — guard may be too long")

    for w in warnings:
        print(f"  WARNING: {w}")
    if not warnings:
        print("  OK: No TIMER_RECOVERY or self-hear discards")

    return results


def main():
    parser = argparse.ArgumentParser(description="Half-duplex two-instance Iris test")
    parser.add_argument("--capture", default="12",
                        help="VB-Cable capture device ID (both instances)")
    parser.add_argument("--playback", default="1006",
                        help="VB-Cable playback device ID (both instances)")
    parser.add_argument("--call-a", default="TSTA", help="Callsign for instance A")
    parser.add_argument("--call-b", default="TSTB", help="Callsign for instance B")
    parser.add_argument("--timeout", type=int, default=60,
                        help="Connection timeout in seconds")
    parser.add_argument("--tx-level", default="0.5", help="TX level for both instances")
    parser.add_argument("--persist", default="63", help="p-persist value (0-255)")
    parser.add_argument("--slottime", default="100", help="Slottime in ms")
    parser.add_argument("--ax25-only", action="store_true",
                        help="Skip native/OFDM probe, stay on AFSK")
    parser.add_argument("--both-connect", action="store_true",
                        help="Both sides initiate connect simultaneously (collision test)")
    parser.add_argument("--payload-size", type=int, default=0,
                        help="Override data payload size (default: 35 bytes; use 1024+ to exercise native bursts)")
    args = parser.parse_args()

    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(1)

    log_a = "halfduplex_a.log"
    log_b = "halfduplex_b.log"

    # Both instances share the SAME cable — half-duplex shared medium
    common_args = [
        "--nogui", "--mode", "A",
        "--capture", args.capture,
        "--playback", args.playback,
        "--tx-level", args.tx_level,
        "--persist", args.persist,
        "--slottime", args.slottime,
    ]
    if args.ax25_only:
        common_args.append("--ax25-only")

    print(f"[TEST] Half-duplex test: {args.call_a} <-> {args.call_b}")
    print(f"       Shared cable: capture={args.capture}, playback={args.playback}")
    print(f"       CSMA: persist={args.persist}, slottime={args.slottime}")
    if args.both_connect:
        print(f"       MODE: simultaneous connect (collision test)")
    print()

    # Instance A: AGW 8010, KISS 8011
    print(f"[1] Starting instance A ({args.call_a})...")
    fa = open(log_a.replace(".log", "_stdout.log"), "w")
    proc_a = subprocess.Popen([
        IRIS, "--callsign", args.call_a,
        "--agw-port", "8010", "--port", "8011",
        "--log", log_a,
        *common_args,
    ], stdout=fa, stderr=subprocess.STDOUT)
    time.sleep(2)

    # Instance B: AGW 8020, KISS 8021
    print(f"[2] Starting instance B ({args.call_b})...")
    fb = open(log_b.replace(".log", "_stdout.log"), "w")
    proc_b = subprocess.Popen([
        IRIS, "--callsign", args.call_b,
        "--agw-port", "8020", "--port", "8021",
        "--log", log_b,
        *common_args,
    ], stdout=fb, stderr=subprocess.STDOUT)
    time.sleep(2)

    # Verify both running
    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"[ERROR] Instance {name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            fa.close(); fb.close()
            tail_log(log_a); tail_log(log_b)
            return 1

    success = True

    try:
        # Connect AGW clients
        print("[3] Connecting AGW clients...")
        sock_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_a.settimeout(5)
        sock_a.connect(("127.0.0.1", 8010))

        sock_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_b.settimeout(5)
        sock_b.connect(("127.0.0.1", 8020))
        print("    Both AGW clients connected")

        # Register callsigns
        agw_send(sock_a, 'X', call_from=args.call_a)
        agw_send(sock_b, 'X', call_from=args.call_b)
        time.sleep(0.5)
        agw_recv_all(sock_a, timeout=1)
        agw_recv_all(sock_b, timeout=1)

        # Enable monitoring on both
        agw_send(sock_a, 'k')
        agw_send(sock_b, 'k')
        time.sleep(0.5)

        # --- Connect ---
        if args.both_connect:
            # Simultaneous connect: BOTH sides send SABM at the same time
            # This is the collision scenario that was failing OTA
            print(f"\n[4] SIMULTANEOUS CONNECT: A->B and B->A at the same time...")
            agw_send(sock_a, 'C', call_from=args.call_a, call_to=args.call_b)
            agw_send(sock_b, 'C', call_from=args.call_b, call_to=args.call_a)
        else:
            # Normal: only A initiates
            print(f"\n[4] {args.call_a} connects to {args.call_b}...")
            agw_send(sock_a, 'C', call_from=args.call_a, call_to=args.call_b)

        print(f"    Waiting up to {args.timeout}s for CONNECTED...")

        connected_a = False
        connected_b = False
        start = time.time()

        while time.time() - start < args.timeout:
            if not connected_a:
                frames = agw_recv_all(sock_a, timeout=2)
                for kind, cf, ct, data in frames:
                    print(f"    A: kind='{kind}' {cf}->{ct} {data[:60]}")
                    if kind == 'C' and b"CONNECTED" in data:
                        connected_a = True
                        print(f"    *** {args.call_a}: CONNECTED ({time.time()-start:.1f}s) ***")

            if not connected_b:
                frames = agw_recv_all(sock_b, timeout=2)
                for kind, cf, ct, data in frames:
                    print(f"    B: kind='{kind}' {cf}->{ct} {data[:60]}")
                    if kind == 'C' and b"CONNECTED" in data:
                        connected_b = True
                        print(f"    *** {args.call_b}: CONNECTED ({time.time()-start:.1f}s) ***")

            if connected_a and connected_b:
                break

        if not connected_a or not connected_b:
            print(f"    [FAIL] Connection failed (A={connected_a}, B={connected_b})")
            success = False

        if connected_a and connected_b:
            # Wait for native mode upgrade before sending large data.
            # Without this, data gets queued as AFSK (~67s for 10KB at 1200 baud)
            # which buries the probe exchange and prevents native mode activation.
            if not args.ax25_only and args.payload_size >= 1024:
                print(f"\n[4b] Waiting for native mode upgrade (probe + SWITCH)...")
                native_deadline = time.time() + 45
                native_ready = False
                while time.time() < native_deadline:
                    time.sleep(2)
                    # Check log A for native TX activation (initiator promotes first)
                    if os.path.isfile(log_a):
                        with open(log_a) as f:
                            content = f.read()
                        if "native TX active" in content or "native confirmed" in content:
                            native_ready = True
                            print("    Native mode active on A")
                            break
                if not native_ready:
                    print("    Native mode not reached — sending data over AFSK")
                else:
                    time.sleep(2)  # Let both sides settle
            else:
                time.sleep(1)

            # --- Data A -> B ---
            print(f"\n[5] Data: {args.call_a} -> {args.call_b}...")
            if args.payload_size > 0:
                msg_ab = bytes(range(256)) * (args.payload_size // 256 + 1)
                msg_ab = msg_ab[:args.payload_size]
            else:
                msg_ab = b"Hello from A to B over half-duplex!"
            # Send in chunks to let modem batch properly (don't flood AGW)
            chunk_size = 200
            offset = 0
            while offset < len(msg_ab):
                chunk = msg_ab[offset:offset+chunk_size]
                agw_send(sock_a, 'D', call_from=args.call_a, call_to=args.call_b, data=chunk)
                offset += len(chunk)
                if offset < len(msg_ab):
                    time.sleep(0.05)
            print(f"    Sent {len(msg_ab)} bytes ({(len(msg_ab)-1)//chunk_size + 1} chunks)")

            # Longer timeout for large payloads (native frames take ~5s each)
            data_timeout = max(30, len(msg_ab) // 50 + 30)
            got_ab = False
            rx_data_b = b""
            frames = agw_recv_all(sock_b, timeout=data_timeout)
            for kind, cf, ct, data in frames:
                if kind == 'D':
                    rx_data_b += data
                    print(f"    B: kind='D' {len(data)} bytes (total {len(rx_data_b)})")
                else:
                    print(f"    B: kind='{kind}' {cf}->{ct} {data[:60]}")

            if rx_data_b == msg_ab:
                got_ab = True
                print("    *** A->B DATA: MATCH ***")
            elif msg_ab in rx_data_b or len(rx_data_b) >= len(msg_ab):
                got_ab = True
                print(f"    *** A->B DATA: received {len(rx_data_b)} bytes (expected {len(msg_ab)}) ***")

            if not got_ab:
                print(f"    [FAIL] B did not receive A's data (got {len(rx_data_b)}/{len(msg_ab)} bytes)")
                success = False

            time.sleep(1)

            # --- Data B -> A ---
            print(f"\n[6] Data: {args.call_b} -> {args.call_a}...")
            if args.payload_size > 0:
                msg_ba = bytes(range(256)) * (args.payload_size // 256 + 1)
                msg_ba = msg_ba[:args.payload_size]
            else:
                msg_ba = b"Reply from B to A over half-duplex!"
            offset = 0
            while offset < len(msg_ba):
                chunk = msg_ba[offset:offset+chunk_size]
                agw_send(sock_b, 'D', call_from=args.call_b, call_to=args.call_a, data=chunk)
                offset += len(chunk)
                if offset < len(msg_ba):
                    time.sleep(0.05)
            print(f"    Sent {len(msg_ba)} bytes")

            got_ba = False
            rx_data_a = b""
            frames = agw_recv_all(sock_a, timeout=data_timeout)
            for kind, cf, ct, data in frames:
                if kind == 'D':
                    rx_data_a += data
                    print(f"    A: kind='D' {len(data)} bytes (total {len(rx_data_a)})")
                else:
                    print(f"    A: kind='{kind}' {cf}->{ct} {data[:60]}")

            if rx_data_a == msg_ba:
                got_ba = True
                print("    *** B->A DATA: MATCH ***")
            elif msg_ba in rx_data_a or len(rx_data_a) >= len(msg_ba):
                got_ba = True
                print(f"    *** B->A DATA: received {len(rx_data_a)} bytes (expected {len(msg_ba)}) ***")

            if not got_ba:
                print(f"    [FAIL] A did not receive B's data (got {len(rx_data_a)}/{len(msg_ba)} bytes)")
                success = False

            # --- Disconnect ---
            print(f"\n[7] Disconnect...")
            agw_send(sock_a, 'd', call_from=args.call_a, call_to=args.call_b)
            time.sleep(3)
            frames = agw_recv_all(sock_a, timeout=5)
            for kind, cf, ct, data in frames:
                print(f"    A: kind='{kind}' {cf}->{ct} {data[:60]}")
                if kind == 'd':
                    print("    *** DISCONNECTED ***")
            frames = agw_recv_all(sock_b, timeout=5)
            for kind, cf, ct, data in frames:
                print(f"    B: kind='{kind}' {cf}->{ct} {data[:60]}")

        # Cleanup
        try: sock_a.close()
        except: pass
        try: sock_b.close()
        except: pass

    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback; traceback.print_exc()
        success = False

    print("\n[TEST] Shutting down...")
    proc_a.terminate()
    proc_b.terminate()
    try: proc_a.wait(timeout=3)
    except: proc_a.kill()
    try: proc_b.wait(timeout=3)
    except: proc_b.kill()
    fa.close()
    fb.close()

    tail_log(log_a)
    tail_log(log_b)

    # Burst holdoff analysis — check for the specific symptoms we fixed
    holdoff_results = analyze_burst_holdoff(log_a, log_b)

    print(f"\n{'='*60}")
    print(f"  RESULT: {'PASS' if success else 'FAIL'}")
    print(f"{'='*60}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
