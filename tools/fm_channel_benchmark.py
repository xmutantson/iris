#!/usr/bin/env python3
"""
Iris FM Data Modem -- FM Channel Simulator Benchmark.

Simulates a realistic VHF FM link using --bandpass, --deemphasis, and --noise
flags. Uses AGW protocol to establish a proper connected-mode session, which
triggers the full probe → EQ → gearshift → native PHY pipeline.

Replicates OTA conditions observed on the KG7VSN <-> W7WEK 146.5 MHz link:
  - Passband: 300-3200 Hz (typical FM narrowband radio)
  - De-emphasis: 75 us (US standard, 6 dB/octave above 2122 Hz)
  - SNR: variable (test parameter)

Usage:
  python fm_channel_benchmark.py                      # Default: 60s, no noise
  python fm_channel_benchmark.py --noise 0.01          # Add AWGN
  python fm_channel_benchmark.py --bandpass 300-3000   # Custom bandpass
  python fm_channel_benchmark.py --duration 120        # Longer test
  python fm_channel_benchmark.py --no-deemphasis       # Skip de-emphasis
"""
import subprocess, socket, time, sys, os, argparse, re, struct
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

# AGW/PE protocol constants
AGW_HEADER_SIZE = 36
AGW_VERSION    = ord('R')
AGW_PORT_INFO  = ord('G')
AGW_REGISTER   = ord('X')
AGW_CONNECT    = ord('C')
AGW_DATA       = ord('D')
AGW_DISCONNECT = ord('d')
AGW_OUTSTANDING = ord('Y')


def agw_build_header(kind, call_from="", call_to="", data_len=0):
    """Build a 36-byte AGW header."""
    hdr = bytearray(36)
    hdr[0] = 0        # port
    hdr[4] = kind      # data_kind
    hdr[6] = 0xF0      # pid
    # call_from: bytes 8-17 (10 bytes, null-padded)
    cf = call_from.encode('ascii')[:9]
    hdr[8:8+len(cf)] = cf
    # call_to: bytes 18-27 (10 bytes, null-padded)
    ct = call_to.encode('ascii')[:9]
    hdr[18:18+len(ct)] = ct
    # data_len: bytes 28-31 (uint32 LE)
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)


def agw_parse_header(data):
    """Parse a 36-byte AGW header. Returns dict."""
    if len(data) < 36:
        return None
    kind = data[4]
    call_from = data[8:18].split(b'\x00')[0].decode('ascii', errors='replace').strip()
    call_to = data[18:28].split(b'\x00')[0].decode('ascii', errors='replace').strip()
    data_len = struct.unpack_from('<I', data, 28)[0]
    return {
        'kind': kind,
        'kind_char': chr(kind) if 32 <= kind < 127 else '?',
        'call_from': call_from,
        'call_to': call_to,
        'data_len': data_len,
    }


def agw_recv_frame(sock, timeout=30):
    """Receive one complete AGW frame. Returns (header_dict, data_bytes) or None."""
    sock.settimeout(timeout)
    # Read 36-byte header
    hdr_buf = b""
    while len(hdr_buf) < 36:
        try:
            chunk = sock.recv(36 - len(hdr_buf))
            if not chunk:
                return None
            hdr_buf += chunk
        except socket.timeout:
            return None
    hdr = agw_parse_header(hdr_buf)
    if hdr is None:
        return None
    # Read data payload
    data = b""
    remaining = hdr['data_len']
    while len(data) < remaining:
        try:
            chunk = sock.recv(remaining - len(data))
            if not chunk:
                return None
            data += chunk
        except socket.timeout:
            return None
    return hdr, data


def parse_log(log_path):
    """Parse iris log for gearshift events, probe results, decode stats."""
    info = {
        'gearshift_events': [],
        'peak_level': 0,
        'peak_level_name': 'A0',
        'probe_bandwidth': 0,
        'probe_tones': 0,
        'probe_baud': 0,
        'rx_eq_range': 0,
        'tx_eq_range': 0,
        'crc_errors': 0,
        'decode_failures': 0,
        'frames_decoded': 0,
        'ldpc_iters_avg': 0,
        'compression_events': 0,
        'nack_events': 0,
        'native_activated': False,
        'connection_time': 0,
    }
    ldpc_iters_total = 0
    ldpc_iters_count = 0

    if not os.path.isfile(log_path):
        return info

    with open(log_path, errors='replace') as f:
        for line in f:
            # Gearshift events
            m = re.search(r'gearshift: (\d+)->(\d+)', line)
            if m:
                old, new = int(m.group(1)), int(m.group(2))
                if new != old:
                    info['gearshift_events'].append((old, new))
                if new > info['peak_level']:
                    info['peak_level'] = new
                    info['peak_level_name'] = f'A{new}'

            # Probe results
            m = re.search(r'Probe.*band ([\d.]+)-([\d.]+) Hz \(([\d.]+) Hz BW\)', line)
            if m:
                info['probe_bandwidth'] = float(m.group(3))

            m = re.search(r'(\d+) tones detected', line)
            if m:
                info['probe_tones'] = int(m.group(1))

            m = re.search(r'baud rate to (\d+)', line)
            if m:
                info['probe_baud'] = int(m.group(1))

            # EQ
            m = re.search(r'\[CH-EQ\] Configured.*EQ range ([\d.]+) dB', line)
            if m:
                val = float(m.group(1))
                if info['rx_eq_range'] == 0:
                    info['rx_eq_range'] = val
                else:
                    info['tx_eq_range'] = val

            # LDPC iterations
            m = re.search(r'ldpc_iters=(\d+)', line)
            if m:
                ldpc_iters_total += int(m.group(1))
                ldpc_iters_count += 1

            # Decode failures
            if 'decode FAIL' in line:
                info['decode_failures'] += 1

            # Native frames decoded
            if 'RX native frame' in line and 'DISCARDED' not in line:
                info['frames_decoded'] += 1

            # Native mode activated
            if 'OFDM-KISS' in line and 'activated' in line.lower():
                info['native_activated'] = True
            if 'native=1' in line:
                info['native_activated'] = True

            # Connection established
            m = re.search(r'\[([\d.]+)\].*CONNECTED', line)
            if m and info['connection_time'] == 0:
                info['connection_time'] = float(m.group(1))

            # Compression
            if 'decompress:' in line or 'compress:' in line:
                info['compression_events'] += 1

            # NACK
            if 'AX25 TX REJ' in line:
                info['nack_events'] += 1

    if ldpc_iters_count > 0:
        info['ldpc_iters_avg'] = ldpc_iters_total / ldpc_iters_count

    return info


def run_benchmark(iris_path, duration, bandpass, deemph_us, noise, payload_size, extra_args):
    """Run the FM channel simulator benchmark using AGW protocol."""
    # Kill any existing iris
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(3)

    log_file = os.path.abspath("iris_fm_bench.log")
    if os.path.isfile(log_file):
        try: os.remove(log_file)
        except: pass

    callsign = "BENCH"

    cmd = [
        iris_path, "--loopback", "--nogui",
        "--callsign", callsign,
        "--mode", "A",
        "--log", log_file,
        "--ptt-pre", "50",     # Simulate ~50ms radio keying delay
        "--ptt-post", "50",
    ]

    if bandpass:
        cmd += ["--bandpass", bandpass]
    if deemph_us > 0:
        cmd += ["--deemphasis", str(deemph_us)]
    if noise > 0:
        cmd += ["--noise", str(noise)]
    cmd += extra_args

    print(f"  Command: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)

    # Wait for AGW port
    sock = None
    for attempt in range(15):
        time.sleep(1)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect(("127.0.0.1", 8000))
            sock = s
            break
        except Exception:
            s.close()
    if sock is None:
        print("  AGW connect failed")
        proc.terminate()
        return None

    print("  AGW connected")

    # Register callsign
    hdr = agw_build_header(AGW_REGISTER, call_from=callsign)
    sock.sendall(hdr)
    resp = agw_recv_frame(sock, timeout=5)
    if resp:
        print(f"  Registered callsign: {callsign}")

    # Initiate connection (BENCH -> BENCH, loopback self-connect)
    print(f"  Connecting {callsign} -> {callsign} (loopback)...")
    hdr = agw_build_header(AGW_CONNECT, call_from=callsign, call_to=callsign)
    sock.sendall(hdr)

    # Wait for CONNECTED notification
    connect_start = time.time()
    connected = False
    while time.time() - connect_start < 60:
        resp = agw_recv_frame(sock, timeout=2)
        if resp is None:
            continue
        hdr_info, data = resp
        kind = hdr_info['kind_char']
        if kind == 'C' and data:
            msg = data.decode('ascii', errors='replace')
            if 'CONNECTED' in msg:
                connected = True
                print(f"  CONNECTED ({time.time() - connect_start:.1f}s)")
                break
        elif kind == 'd':
            print(f"  Connection REJECTED")
            break

    if not connected:
        print(f"  Connection failed (timeout after {time.time() - connect_start:.1f}s)")
        sock.close()
        proc.terminate()
        return None

    # Wait for probe + EQ + gearshift to settle
    # The modem does probe -> EQ -> native mode activation automatically
    # after connection is established. Give it time to complete.
    print(f"  Waiting for probe + native mode activation...")
    settle_start = time.time()
    # Drain any incoming frames during settle period
    while time.time() - settle_start < 30:
        resp = agw_recv_frame(sock, timeout=1)
        # Check log periodically for native mode activation
        if time.time() - settle_start > 5:
            log_info = parse_log(log_file)
            if log_info['native_activated'] or log_info['probe_tones'] > 0:
                # Give a bit more time for gearshift to start
                time.sleep(3)
                break

    log_info = parse_log(log_file)
    if log_info['native_activated']:
        print(f"  Native mode ACTIVE (probe: {log_info['probe_tones']} tones, "
              f"BW {log_info['probe_bandwidth']:.0f} Hz, baud {log_info['probe_baud']})")
    else:
        print(f"  Native mode not activated (running on AFSK)")

    # Build test payload
    test_data = bytes([(i * 37 + 17) & 0xFF for i in range(payload_size)])

    print(f"  Running {duration}s measurement...")

    # Measurement phase: send data via AGW 'D' frames, receive via AGW 'D' frames
    tx_count = 0
    rx_bytes = 0
    rx_frames = 0

    start = time.time()
    last_report = start
    sock.settimeout(0.01)

    while time.time() - start < duration:
        elapsed = time.time() - start

        # Send data frames to keep pipeline full
        ahead = tx_count - rx_frames
        to_send = max(0, min(20 - ahead, 5))
        for _ in range(to_send):
            try:
                frame_hdr = agw_build_header(AGW_DATA, call_from=callsign,
                                              call_to=callsign, data_len=len(test_data))
                sock.sendall(frame_hdr + test_data)
                tx_count += 1
            except:
                break

        # Receive data frames
        for _ in range(20):
            try:
                # Try to read header
                hdr_buf = sock.recv(36)
                if not hdr_buf or len(hdr_buf) < 36:
                    # Partial header — save for later
                    break
                hdr_info = agw_parse_header(hdr_buf)
                if hdr_info is None:
                    break
                # Read data
                data = b""
                remaining = hdr_info['data_len']
                sock.settimeout(1)
                while len(data) < remaining:
                    chunk = sock.recv(remaining - len(data))
                    if not chunk:
                        break
                    data += chunk
                sock.settimeout(0.01)

                if hdr_info['kind_char'] == 'D':
                    rx_frames += 1
                    rx_bytes += len(data)
            except socket.timeout:
                break
            except:
                break

        # Progress report every 15s
        if time.time() - last_report >= 15:
            bps_so_far = rx_bytes * 8 / elapsed if elapsed > 0 else 0
            print(f"  [{elapsed:.0f}s] {rx_frames} frames, {rx_bytes} bytes, "
                  f"{bps_so_far:.0f} bps")
            last_report = time.time()

    measure_elapsed = time.time() - start

    # Drain in-flight frames
    drain_timeout = min(30, duration / 2)
    drain_end = time.time() + drain_timeout
    sock.settimeout(0.5)
    while time.time() < drain_end and tx_count > rx_frames:
        try:
            hdr_buf = sock.recv(36)
            if not hdr_buf or len(hdr_buf) < 36:
                continue
            hdr_info = agw_parse_header(hdr_buf)
            if hdr_info is None:
                continue
            data = b""
            remaining = hdr_info['data_len']
            while len(data) < remaining:
                chunk = sock.recv(remaining - len(data))
                if not chunk:
                    break
                data += chunk
            if hdr_info['kind_char'] == 'D':
                rx_frames += 1
                rx_bytes += len(data)
        except socket.timeout:
            continue
        except:
            break

    # Disconnect
    try:
        hdr = agw_build_header(AGW_DISCONNECT, call_from=callsign, call_to=callsign)
        sock.sendall(hdr)
        time.sleep(1)
    except:
        pass

    sock.close()
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except:
        proc.kill()

    # Parse log for detailed stats
    log_info = parse_log(log_file)

    bps = rx_bytes * 8 / measure_elapsed if measure_elapsed > 0 else 0

    return {
        'rx_frames': rx_frames,
        'rx_bytes': rx_bytes,
        'tx_frames': tx_count,
        'duration': measure_elapsed,
        'throughput_bps': bps,
        **log_info,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Iris FM Channel Simulator Benchmark",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # Default FM channel (300-3200 Hz, 75us de-emphasis)
  %(prog)s --noise 0.005                      # Add mild noise
  %(prog)s --noise 0.01                       # Moderate noise (like real VHF)
  %(prog)s --bandpass 500-2700 --noise 0.01   # Narrow radio + noise
  %(prog)s --no-deemphasis                    # Flat channel (no de-emphasis)
  %(prog)s --no-bandpass --no-deemphasis      # Clean loopback (baseline)
        """)
    parser.add_argument("--iris", default=IRIS, help="Path to iris.exe")
    parser.add_argument("--duration", type=int, default=60,
                       help="Measurement duration (default: 60s)")
    parser.add_argument("--payload", type=int, default=200,
                       help="Payload size in bytes (default: 200)")
    parser.add_argument("--bandpass", default="300-3200",
                       help="Simulated radio bandpass (default: 300-3200)")
    parser.add_argument("--no-bandpass", action="store_true",
                       help="Disable bandpass filter")
    parser.add_argument("--deemphasis", type=float, default=75.0,
                       help="De-emphasis time constant in us (default: 75)")
    parser.add_argument("--no-deemphasis", action="store_true",
                       help="Disable de-emphasis filter")
    parser.add_argument("--noise", type=float, default=0.0,
                       help="AWGN noise amplitude (0 = none)")
    parser.add_argument("--tx-level", type=float, default=None)
    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    bandpass = None if args.no_bandpass else args.bandpass
    deemph_us = 0 if args.no_deemphasis else args.deemphasis

    extra = []
    if args.tx_level is not None:
        extra += ["--tx-level", str(args.tx_level)]

    channel_desc = []
    if bandpass:
        channel_desc.append(f"BP {bandpass} Hz")
    if deemph_us > 0:
        channel_desc.append(f"de-emph {deemph_us:.0f} us")
    if args.noise > 0:
        channel_desc.append(f"noise {args.noise}")
    if not channel_desc:
        channel_desc.append("clean loopback")

    print("=" * 70)
    print(f"  Iris FM Data Modem -- FM Channel Simulator Benchmark")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Channel: {', '.join(channel_desc)}")
    print(f"  Duration: {args.duration}s, payload: {args.payload} bytes")
    print("=" * 70)

    result = run_benchmark(
        iris_path, args.duration, bandpass, deemph_us,
        args.noise, args.payload, extra
    )

    if result is None:
        print("\n  BENCHMARK FAILED")
        return 1

    # Results
    print("\n" + "=" * 70)
    print("  RESULTS")
    print("=" * 70)
    print(f"  Throughput:       {result['throughput_bps']:.0f} bps")
    print(f"  Data transferred: {result['rx_bytes']} bytes in {result['duration']:.1f}s")
    print(f"  Frames:           {result['rx_frames']} RX / {result['tx_frames']} TX")
    print()
    print(f"  Probe:")
    print(f"    Tones detected: {result['probe_tones']}")
    print(f"    Bandwidth:      {result['probe_bandwidth']:.0f} Hz")
    print(f"    Baud rate:      {result['probe_baud']}")
    print(f"    RX EQ range:    {result['rx_eq_range']:.1f} dB")
    print(f"    TX EQ range:    {result['tx_eq_range']:.1f} dB")
    print()
    print(f"  Gearshift:")
    print(f"    Peak level:     {result['peak_level_name']}")
    shifts = result['gearshift_events']
    if shifts:
        print(f"    Transitions:    {' -> '.join(f'A{s[1]}' for s in shifts)}")
    else:
        print(f"    Transitions:    none (stayed at A0)")
    print()
    print(f"  Channel quality:")
    print(f"    Native mode:    {'YES' if result['native_activated'] else 'NO (AFSK only)'}")
    print(f"    Frames decoded: {result['frames_decoded']}")
    print(f"    Decode failures:{result['decode_failures']}")
    print(f"    LDPC iters avg: {result['ldpc_iters_avg']:.1f}")
    print(f"    NACK (REJ) sent:{result['nack_events']}")
    print(f"    Compression:    {result['compression_events']} events")

    # Compare vs VARA FM
    vara_bps = 3050
    ratio = result['throughput_bps'] / vara_bps if vara_bps > 0 else 0
    print()
    print(f"  vs VARA FM Narrow ({vara_bps} bps):")
    if ratio >= 1.0:
        print(f"    >>> {ratio:.2f}x VARA FM ({result['throughput_bps']:.0f} vs {vara_bps} bps) <<<")
    else:
        print(f"    {ratio:.2f}x VARA FM ({result['throughput_bps']:.0f} vs {vara_bps} bps)")
    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
