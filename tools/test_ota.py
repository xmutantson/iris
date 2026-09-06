#!/usr/bin/env python3
"""OTA test: connect to W7WEK via Iris KISS port, observe AX.25 session."""
import socket, time, sys

KISS_FEND = 0xC0
KISS_FESC = 0xDB
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

def build_ax25_sabm(src_call, dst_call):
    """Build AX.25 SABM frame (connection request)."""
    frame = bytearray()
    dst = dst_call.ljust(6)[:6]
    for c in dst:
        frame.append(ord(c) << 1)
    frame.append(0x60)  # dst SSID: command
    src = src_call.ljust(6)[:6]
    for c in src:
        frame.append(ord(c) << 1)
    frame.append(0x61)  # src SSID: last address
    frame.append(0x3F)  # SABM with P bit set
    return bytes(frame)

def build_ax25_disc(src_call, dst_call):
    """Build AX.25 DISC frame (disconnect)."""
    frame = bytearray()
    dst = dst_call.ljust(6)[:6]
    for c in dst:
        frame.append(ord(c) << 1)
    frame.append(0x60)
    src = src_call.ljust(6)[:6]
    for c in src:
        frame.append(ord(c) << 1)
    frame.append(0x61)
    frame.append(0x53)  # DISC with P bit set
    return bytes(frame)

def build_ax25_ui(src_call, dst_call, payload):
    """Build AX.25 UI frame."""
    frame = bytearray()
    dst = dst_call.ljust(6)[:6]
    for c in dst:
        frame.append(ord(c) << 1)
    frame.append(0x60)
    src = src_call.ljust(6)[:6]
    for c in src:
        frame.append(ord(c) << 1)
    frame.append(0x61)
    frame.append(0x03)  # UI control
    frame.append(0xF0)  # no L3
    frame.extend(payload)
    return bytes(frame)

def parse_ax25(data):
    """Parse AX.25 frame, return human-readable description."""
    if len(data) < 14:
        return f"<short frame {len(data)} bytes>"

    dst = ""
    for i in range(6):
        c = chr(data[i] >> 1)
        if c != ' ':
            dst += c
    dst_ssid = (data[6] >> 1) & 0x0F

    src = ""
    for i in range(7, 13):
        c = chr(data[i] >> 1)
        if c != ' ':
            src += c
    src_ssid = (data[13] >> 1) & 0x0F

    if dst_ssid:
        dst += f"-{dst_ssid}"
    if src_ssid:
        src += f"-{src_ssid}"

    ctrl_offset = 14
    # Skip digipeaters
    if not (data[13] & 0x01):
        i = 14
        while i + 6 < len(data):
            if data[i + 6] & 0x01:
                ctrl_offset = i + 7
                break
            i += 7

    if ctrl_offset >= len(data):
        return f"{src}>{dst} <no control>"

    ctrl = data[ctrl_offset]

    # U-frames
    if (ctrl & 0x03) == 0x03:
        masked = ctrl & 0xEF
        pf = "P/F" if (ctrl & 0x10) else ""
        u_types = {
            0x2F: "SABM", 0x43: "DISC", 0x63: "UA",
            0x0F: "DM", 0x87: "FRMR", 0x03: "UI", 0xAF: "XID"
        }
        name = u_types.get(masked, f"U(0x{masked:02X})")
        info = ""
        if ctrl_offset + 1 < len(data):
            pid = data[ctrl_offset + 1]
            if pid == 0xF0 and ctrl_offset + 2 < len(data):
                info = f" [{data[ctrl_offset+2:ctrl_offset+32].decode('ascii', errors='replace')}]"
            elif pid == 0xBE:
                info = " [IRIS XID]"
        return f"{src}>{dst} {name} {pf}{info}".strip()

    # I-frames
    if (ctrl & 0x01) == 0:
        ns = (ctrl >> 1) & 0x07
        nr = (ctrl >> 5) & 0x07
        pf = "P" if (ctrl & 0x10) else ""
        info_len = len(data) - ctrl_offset - 2  # -ctrl -pid
        payload = data[ctrl_offset+2:]
        preview = payload[:40].decode('ascii', errors='replace')
        return f"{src}>{dst} I N(S)={ns} N(R)={nr} {pf} ({info_len} bytes) [{preview}]"

    # S-frames
    s_types = {0x01: "RR", 0x05: "RNR", 0x09: "REJ"}
    s_type = ctrl & 0x0F
    nr = (ctrl >> 5) & 0x07
    pf = "P/F" if (ctrl & 0x10) else ""
    name = s_types.get(s_type, f"S(0x{s_type:02X})")
    return f"{src}>{dst} {name} N(R)={nr} {pf}"

def main():
    host = "127.0.0.1"
    port = 8001
    remote = "W7WEK"
    local = "KG7VSN"
    mode = sys.argv[1] if len(sys.argv) > 1 else "listen"

    print(f"=== OTA Test: {local} -> {remote} via Iris KISS ===")
    print(f"Mode: {mode}")
    print(f"Connecting to KISS server at {host}:{port}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((host, port))
    except Exception as e:
        print(f"[ERROR] Cannot connect to KISS: {e}")
        return 1
    print("Connected to KISS server.")

    if mode == "connect":
        # Send SABM to initiate AX.25 connection
        sabm = build_ax25_sabm(local, remote)
        kiss_frame = kiss_encode(sabm)
        print(f"\nSending SABM to {remote}...")
        sock.send(kiss_frame)
        print(f"  TX: {local}>{remote} SABM P ({len(sabm)} bytes)")
    elif mode == "ui":
        # Send a UI frame
        msg = b"Test from Iris OTA test"
        ui = build_ax25_ui(local, remote, msg)
        kiss_frame = kiss_encode(ui)
        print(f"\nSending UI frame to {remote}...")
        sock.send(kiss_frame)
        print(f"  TX: {local}>{remote} UI ({len(ui)} bytes)")
    elif mode == "disc":
        disc = build_ax25_disc(local, remote)
        kiss_frame = kiss_encode(disc)
        print(f"\nSending DISC to {remote}...")
        sock.send(kiss_frame)
        print(f"  TX: {local}>{remote} DISC P ({len(disc)} bytes)")
    else:
        print(f"\nPassive listen mode (no TX)")

    # Listen for responses
    duration = 90
    print(f"\nListening for frames ({duration} seconds)...")
    print(f"{'='*70}")

    rx_buf = b""
    start = time.time()
    frame_count = 0
    connected = False

    while time.time() - start < duration:
        try:
            sock.settimeout(1.0)
            data = sock.recv(4096)
            if data:
                rx_buf += data
                frames = kiss_decode(rx_buf)
                for cmd, frame_data in frames:
                    frame_count += 1
                    elapsed = time.time() - start
                    desc = parse_ax25(frame_data)
                    print(f"  [{elapsed:5.1f}s] RX #{frame_count}: {desc}")
                    sys.stdout.flush()

                    # Detect connection established
                    if "UA" in desc and not connected:
                        connected = True
                        print(f"  [{elapsed:5.1f}s] *** CONNECTION ESTABLISHED ***")
                if frames:
                    last_fend = rx_buf.rfind(bytes([KISS_FEND]))
                    if last_fend >= 0:
                        rx_buf = rx_buf[last_fend:]
                    else:
                        rx_buf = b""
        except socket.timeout:
            continue
        except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
            print(f"  [ERROR] Connection lost: {e}")
            break

    print(f"{'='*70}")
    print(f"Total frames received: {frame_count}")
    print(f"Connected: {connected}")

    sock.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
