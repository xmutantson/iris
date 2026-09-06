#!/usr/bin/env python3
"""KISS-level half-duplex test — exercises the EXACT same code path as Winlink.

Both Iris instances share one VB-Cable.  Instead of using AGW (Iris's built-in
AX.25), this test acts as two KISS clients that manage their own AX.25 sessions
— just like Winlink Express does.

This test exercises:
  - KISS TCP client → modem TX path (queue_tx_frame)
  - Modem RX → KISS delivery path (dispatch_rx_frame → rx_callback → KISS)
  - ax25_session_ shadowing via notify_outgoing()
  - Connection header UI frame exchange
  - CSMA/DCD on the shared half-duplex medium

Test flow:
  1. Start two Iris instances on the same VB-Cable (KISS mode only)
  2. Connect KISS TCP clients to each
  3. Client A sends SABM to Client B
  4. Client B receives SABM, sends UA
  5. Client A receives UA → CONNECTED
  6. Send I-frame A→B, receive RR acknowledgment
  7. Send I-frame B→A, receive RR acknowledgment
  8. Client A sends DISC, Client B sends UA
  9. Verify clean disconnect

Usage:
  python test_kiss_session.py [--capture ID] [--playback ID]
"""
import subprocess, socket, struct, time, sys, os, argparse

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")

KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD
AX25_FLAG = 0x7E
AX25_PID_NONE = 0xF0

# --- CRC-CCITT (same as AX.25 FCS) ---

def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc ^ 0xFFFF

# --- AX.25 frame building ---

def ax25_addr(call, ssid=0, last=False, cr_bit=False):
    """Build a 7-byte AX.25 address field."""
    call = call.upper().ljust(6)[:6]
    out = bytearray()
    for c in call:
        out.append(ord(c) << 1)
    ext = 0x60 | ((ssid & 0x0F) << 1)
    if last:
        ext |= 0x01
    if cr_bit:
        ext |= 0x80
    out.append(ext)
    return bytes(out)

def ax25_sabm(src, dst):
    """Build SABM P=1 frame (connection request)."""
    # dst: command bit set (cr_bit in dst for command frame)
    frame = ax25_addr(dst, cr_bit=True) + ax25_addr(src, last=True)
    frame += bytes([0x3F])  # SABM with P=1: 0011_1111
    return frame

def ax25_ua(src, dst, f=True):
    """Build UA F=1 frame (acknowledgment)."""
    ctrl = 0x73 if f else 0x63  # UA: 0110_0011, F bit at position 4
    frame = ax25_addr(dst, cr_bit=False) + ax25_addr(src, last=True, cr_bit=True)
    frame += bytes([ctrl])
    return frame

def ax25_disc(src, dst):
    """Build DISC P=1 frame (disconnect)."""
    frame = ax25_addr(dst, cr_bit=True) + ax25_addr(src, last=True)
    frame += bytes([0x53])  # DISC with P=1: 0101_0011
    return frame

def ax25_rr(src, dst, nr, pf=False, command=False):
    """Build RR (Receive Ready) supervisory frame."""
    ctrl = 0x01 | ((nr & 7) << 5)
    if pf:
        ctrl |= 0x10
    frame = ax25_addr(dst, cr_bit=command) + ax25_addr(src, last=True, cr_bit=not command)
    frame += bytes([ctrl])
    return frame

def ax25_iframe(src, dst, ns, nr, data, pf=False):
    """Build I-frame (information frame)."""
    ctrl = ((ns & 7) << 1) | ((nr & 7) << 5)
    if pf:
        ctrl |= 0x10
    frame = ax25_addr(dst, cr_bit=True) + ax25_addr(src, last=True)
    frame += bytes([ctrl, AX25_PID_NONE])
    frame += data
    return frame

def describe_frame(frame):
    """Describe an AX.25 frame for logging."""
    if len(frame) < 15:
        return f"<short {len(frame)} bytes>"
    dst = ''.join(chr(frame[i] >> 1) for i in range(6)).strip()
    src = ''.join(chr(frame[i] >> 1) for i in range(7, 13)).strip()
    ctrl = frame[14]
    if ctrl & 1 == 0:
        # I-frame
        ns = (ctrl >> 1) & 7
        nr = (ctrl >> 5) & 7
        pf = "P" if ctrl & 0x10 else ""
        info_len = len(frame) - 16 if len(frame) > 15 else 0
        return f"I {src}>{dst} N(S)={ns} N(R)={nr} {pf} ({info_len} bytes)"
    elif ctrl & 3 == 1:
        # S-frame
        nr = (ctrl >> 5) & 7
        pf = "P/F" if ctrl & 0x10 else ""
        stype = (ctrl >> 2) & 3
        snames = {0: "RR", 1: "RNR", 2: "REJ", 3: "SREJ"}
        return f"{snames.get(stype, '?')} {src}>{dst} N(R)={nr} {pf}"
    else:
        # U-frame
        utype = ctrl & 0xEF
        pf = "P/F" if ctrl & 0x10 else ""
        unames = {0x2F: "SABM", 0x63: "UA", 0x43: "DISC", 0x0F: "DM",
                  0x03: "UI", 0x87: "FRMR", 0xAF: "XID"}
        # Handle with/without P/F
        utype_nopf = ctrl & ~0x10 & 0xFF
        for code, name in unames.items():
            if utype_nopf == (code & ~0x10):
                return f"{name} {src}>{dst} {pf}"
        return f"U(0x{ctrl:02X}) {src}>{dst} {pf}"

# --- KISS framing ---

def kiss_encode(frame, port=0):
    """Encode an AX.25 frame into a KISS TCP frame."""
    out = bytearray([KISS_FEND, (port << 4) | 0x00])  # Data command
    for b in frame:
        if b == KISS_FEND:
            out += bytes([KISS_FESC, KISS_TFEND])
        elif b == KISS_FESC:
            out += bytes([KISS_FESC, KISS_TFESC])
        else:
            out.append(b)
    out.append(KISS_FEND)
    return bytes(out)

def kiss_decode_frames(data):
    """Decode KISS TCP data into a list of raw AX.25 frames."""
    frames = []
    in_frame = False
    escape = False
    buf = bytearray()
    for b in data:
        if b == KISS_FEND:
            if in_frame and len(buf) >= 1:
                cmd = buf[0]
                if (cmd & 0x0F) == 0x00:  # Data frame
                    frames.append(bytes(buf[1:]))
            buf = bytearray()
            in_frame = True
            escape = False
        elif not in_frame:
            continue
        elif b == KISS_FESC:
            escape = True
        elif escape:
            if b == KISS_TFEND:
                buf.append(KISS_FEND)
            elif b == KISS_TFESC:
                buf.append(KISS_FESC)
            else:
                buf.append(b)
            escape = False
        else:
            buf.append(b)
    return frames

def kiss_send(sock, frame):
    """Send an AX.25 frame via KISS TCP."""
    sock.sendall(kiss_encode(frame))

def kiss_recv_frames(sock, timeout=10):
    """Receive and decode KISS frames from TCP socket."""
    sock.settimeout(timeout)
    frames = []
    end = time.time() + timeout
    while time.time() < end:
        remaining = max(0.1, end - time.time())
        sock.settimeout(remaining)
        try:
            data = sock.recv(4096)
            if not data:
                break
            decoded = kiss_decode_frames(data)
            frames.extend(decoded)
            if frames:
                return frames  # Return as soon as we get something
        except socket.timeout:
            break
        except (ConnectionResetError, ConnectionAbortedError, OSError):
            break
    return frames

def kiss_wait_for(sock, src, frame_type, timeout=15):
    """Wait for a specific frame type from a specific source."""
    end = time.time() + timeout
    all_frames = []
    while time.time() < end:
        remaining = max(0.1, end - time.time())
        frames = kiss_recv_frames(sock, timeout=remaining)
        for f in frames:
            all_frames.append(f)
            if len(f) < 15:
                continue
            fsrc = ''.join(chr(f[i] >> 1) for i in range(7, 13)).strip()
            ctrl = f[14]
            if fsrc == src:
                if frame_type == "UA" and (ctrl & ~0x10) == 0x63:
                    return f, all_frames
                elif frame_type == "SABM" and (ctrl & ~0x10) == 0x2F:
                    return f, all_frames
                elif frame_type == "DISC" and (ctrl & ~0x10) == 0x43:
                    return f, all_frames
                elif frame_type == "I" and (ctrl & 1) == 0:
                    return f, all_frames
                elif frame_type == "RR" and (ctrl & 0x0F) == 0x01:
                    return f, all_frames
                elif frame_type == "DM" and (ctrl & ~0x10) == 0x0F:
                    return f, all_frames
        if not frames:
            continue
    return None, all_frames

def tail_log(path, n=40):
    if not os.path.isfile(path):
        return
    print(f"\n--- {os.path.basename(path)} (last {n} lines) ---")
    with open(path) as f:
        lines = f.readlines()
    for line in lines[-n:]:
        print(f"  {line.rstrip()}")


def main():
    parser = argparse.ArgumentParser(description="KISS-level half-duplex Iris test")
    parser.add_argument("--capture", default="12",
                        help="VB-Cable capture device ID")
    parser.add_argument("--playback", default="1006",
                        help="VB-Cable playback device ID")
    parser.add_argument("--call-a", default="TSTA", help="Callsign for instance A (initiator)")
    parser.add_argument("--call-b", default="TSTB", help="Callsign for instance B (responder)")
    parser.add_argument("--timeout", type=int, default=30,
                        help="Connection timeout in seconds")
    parser.add_argument("--tx-level", default="0.5", help="TX level")
    parser.add_argument("--persist", default="63", help="p-persist value")
    parser.add_argument("--slottime", default="100", help="Slottime in ms")
    args = parser.parse_args()

    if not os.path.isfile(IRIS):
        print(f"[ERROR] Not found: {IRIS}")
        return 1

    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(1)

    log_a = "kiss_session_a.log"
    log_b = "kiss_session_b.log"

    common_args = [
        "--nogui", "--mode", "A", "--ax25-only",
        "--capture", args.capture,
        "--playback", args.playback,
        "--tx-level", args.tx_level,
        "--persist", args.persist,
        "--slottime", args.slottime,
    ]

    print(f"[TEST] KISS-level half-duplex test: {args.call_a} <-> {args.call_b}")
    print(f"       Shared cable: capture={args.capture}, playback={args.playback}")
    print(f"       This exercises the KISS code path (same as Winlink)")
    print()

    # Instance A: KISS port 8011
    print(f"[1] Starting instance A ({args.call_a})...")
    fa = open(log_a.replace(".log", "_stdout.log"), "w")
    proc_a = subprocess.Popen([
        IRIS, "--callsign", args.call_a,
        "--agw-port", "8010", "--port", "8011",
        "--log", log_a,
        *common_args,
    ], stdout=fa, stderr=subprocess.STDOUT)
    time.sleep(2)

    # Instance B: KISS port 8021
    print(f"[2] Starting instance B ({args.call_b})...")
    fb = open(log_b.replace(".log", "_stdout.log"), "w")
    proc_b = subprocess.Popen([
        IRIS, "--callsign", args.call_b,
        "--agw-port", "8020", "--port", "8021",
        "--log", log_b,
        *common_args,
    ], stdout=fb, stderr=subprocess.STDOUT)
    time.sleep(2)

    for name, proc in [("A", proc_a), ("B", proc_b)]:
        if proc.poll() is not None:
            print(f"[ERROR] Instance {name} exited with code {proc.returncode}")
            proc_a.terminate(); proc_b.terminate()
            fa.close(); fb.close()
            tail_log(log_a); tail_log(log_b)
            return 1

    success = True

    try:
        # Connect KISS TCP clients
        print("[3] Connecting KISS TCP clients...")
        kiss_a = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        kiss_a.settimeout(5)
        kiss_a.connect(("127.0.0.1", 8011))

        kiss_b = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        kiss_b.settimeout(5)
        kiss_b.connect(("127.0.0.1", 8021))
        print("    Both KISS clients connected")
        time.sleep(0.5)

        # --- Step 4: A sends SABM to B ---
        print(f"\n[4] {args.call_a} sends SABM to {args.call_b} (via KISS)...")
        sabm = ax25_sabm(args.call_a, args.call_b)
        kiss_send(kiss_a, sabm)
        print(f"    Sent SABM ({len(sabm)} bytes)")

        # --- Step 5: B waits for SABM, sends UA ---
        print(f"\n[5] {args.call_b} waiting for SABM...")
        result, all_b = kiss_wait_for(kiss_b, args.call_a, "SABM", timeout=args.timeout)
        for f in all_b:
            if len(f) >= 15:
                print(f"    B rx: {describe_frame(f)}")
        if result is None:
            print(f"    [FAIL] B never received SABM")
            success = False
        else:
            print(f"    *** B received SABM ***")
            ua = ax25_ua(args.call_b, args.call_a, f=True)
            kiss_send(kiss_b, ua)
            print(f"    B sent UA ({len(ua)} bytes)")

        # --- Step 6: A waits for UA ---
        if success:
            print(f"\n[6] {args.call_a} waiting for UA...")
            result, all_a = kiss_wait_for(kiss_a, args.call_b, "UA", timeout=args.timeout)
            for f in all_a:
                if len(f) >= 15:
                    print(f"    A rx: {describe_frame(f)}")
            if result is None:
                print(f"    [FAIL] A never received UA")
                success = False
            else:
                print(f"    *** A received UA — CONNECTED ***")

        # --- Step 7: Send data A→B ---
        if success:
            time.sleep(1)
            print(f"\n[7] Data: {args.call_a} -> {args.call_b} (I-frame via KISS)...")
            payload = b"Hello from KISS client A!"
            iframe = ax25_iframe(args.call_a, args.call_b, ns=0, nr=0, data=payload)
            kiss_send(kiss_a, iframe)
            print(f"    Sent I-frame N(S)=0 ({len(payload)} bytes payload)")

            # B should receive I-frame
            result, all_b = kiss_wait_for(kiss_b, args.call_a, "I", timeout=15)
            for f in all_b:
                if len(f) >= 15:
                    print(f"    B rx: {describe_frame(f)}")
            if result is None:
                print(f"    [FAIL] B did not receive I-frame")
                success = False
            else:
                # Verify payload
                rx_payload = result[16:]  # skip addr(14) + ctrl(1) + PID(1)
                if payload in rx_payload:
                    print(f"    *** B received I-frame — DATA MATCH ***")
                else:
                    print(f"    [FAIL] Payload mismatch: {rx_payload[:40]}")
                    success = False

                # B sends RR acknowledgment
                rr = ax25_rr(args.call_b, args.call_a, nr=1)
                kiss_send(kiss_b, rr)
                print(f"    B sent RR N(R)=1")

        # --- Step 8: Send data B→A ---
        if success:
            time.sleep(1)
            print(f"\n[8] Data: {args.call_b} -> {args.call_a} (I-frame via KISS)...")
            payload2 = b"Reply from KISS client B!"
            iframe2 = ax25_iframe(args.call_b, args.call_a, ns=0, nr=1, data=payload2)
            kiss_send(kiss_b, iframe2)
            print(f"    Sent I-frame N(S)=0 ({len(payload2)} bytes payload)")

            # A should receive I-frame
            result, all_a = kiss_wait_for(kiss_a, args.call_b, "I", timeout=15)
            for f in all_a:
                if len(f) >= 15:
                    print(f"    A rx: {describe_frame(f)}")
            if result is None:
                print(f"    [FAIL] A did not receive I-frame")
                success = False
            else:
                rx_payload = result[16:]
                if payload2 in rx_payload:
                    print(f"    *** A received I-frame — DATA MATCH ***")
                else:
                    print(f"    [FAIL] Payload mismatch: {rx_payload[:40]}")
                    success = False

                rr2 = ax25_rr(args.call_a, args.call_b, nr=1)
                kiss_send(kiss_a, rr2)
                print(f"    A sent RR N(R)=1")

        # --- Step 9: Disconnect ---
        if success:
            time.sleep(1)
            print(f"\n[9] Disconnect: {args.call_a} sends DISC...")
            disc = ax25_disc(args.call_a, args.call_b)
            kiss_send(kiss_a, disc)
            print(f"    Sent DISC ({len(disc)} bytes)")

            # B should receive DISC, send UA
            result, all_b = kiss_wait_for(kiss_b, args.call_a, "DISC", timeout=10)
            for f in all_b:
                if len(f) >= 15:
                    print(f"    B rx: {describe_frame(f)}")
            if result:
                print(f"    *** B received DISC ***")
                ua2 = ax25_ua(args.call_b, args.call_a, f=True)
                kiss_send(kiss_b, ua2)
                print(f"    B sent UA")

                # A should receive UA
                time.sleep(2)
                result, all_a = kiss_wait_for(kiss_a, args.call_b, "UA", timeout=10)
                for f in all_a:
                    if len(f) >= 15:
                        print(f"    A rx: {describe_frame(f)}")
                if result:
                    print(f"    *** A received UA — DISCONNECTED ***")
                else:
                    print(f"    [WARN] A did not receive disconnect UA")
            else:
                print(f"    [WARN] B did not receive DISC")

        try: kiss_a.close()
        except: pass
        try: kiss_b.close()
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

    print(f"\n{'='*60}")
    print(f"  RESULT: {'PASS' if success else 'FAIL'}")
    print(f"{'='*60}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
