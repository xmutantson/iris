#!/usr/bin/env python3
"""
Iris FM Data Modem -- FM Channel Simulator Benchmark.

Runs TWO iris instances on VB-Cable to create a proper bidirectional link.
Simulates realistic VHF FM conditions using --bandpass, --deemphasis, and --noise.
The full pipeline runs naturally: AFSK connect → probe → EQ → gearshift → native PHY.

Replicates OTA conditions observed on the KG7VSN <-> W7WEK 146.5 MHz link:
  - Passband: 300-3200 Hz (typical FM narrowband radio)
  - De-emphasis: 75 us (US standard, 6 dB/octave above 2122 Hz)
  - SNR: variable (test parameter)

Usage:
  python fm_channel_benchmark.py                      # Default: 60s, mild noise
  python fm_channel_benchmark.py --noise 0.01          # More noise
  python fm_channel_benchmark.py --bandpass 300-3000   # Custom bandpass
  python fm_channel_benchmark.py --duration 120        # Longer test
  python fm_channel_benchmark.py --no-deemphasis       # Skip de-emphasis
  python fm_channel_benchmark.py --no-bandpass --no-deemphasis  # Clean baseline
"""
import subprocess, socket, time, sys, os, argparse, re, struct, json
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

# AGW protocol
AGW_HEADER_SIZE = 36


def agw_build_header(kind, call_from="", call_to="", data_len=0):
    hdr = bytearray(36)
    hdr[0] = 0
    hdr[4] = kind
    hdr[6] = 0xF0
    cf = call_from.encode('ascii')[:9]
    hdr[8:8+len(cf)] = cf
    ct = call_to.encode('ascii')[:9]
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)


def agw_parse_header(data):
    if len(data) < 36:
        return None
    return {
        'kind': data[4],
        'kind_char': chr(data[4]) if 32 <= data[4] < 127 else '?',
        'call_from': data[8:18].split(b'\x00')[0].decode('ascii', errors='replace').strip(),
        'call_to': data[18:28].split(b'\x00')[0].decode('ascii', errors='replace').strip(),
        'data_len': struct.unpack_from('<I', data, 28)[0],
    }


def agw_recv_frame(sock, timeout=30):
    sock.settimeout(timeout)
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


def find_vb_cable(iris_path):
    """Auto-detect VB-Cable device IDs."""
    try:
        result = subprocess.run([iris_path, "--list-audio"],
                                capture_output=True, text=True, timeout=10)
        output = result.stdout + result.stderr
    except Exception as e:
        print(f"  Failed to list audio devices: {e}")
        return None, None

    capture_id = None
    playback_id = None
    for line in output.splitlines():
        # Look for "CABLE Output" (capture) and "CABLE Input" (playback)
        # Prefer plain "CABLE" over "CABLE-A", "CABLE-B", etc.
        m = re.match(r'\s*\[(\d+)\]\s+CABLE Output \(VB-Audio', line)
        if m:
            capture_id = int(m.group(1))
        m = re.match(r'\s*\[(\d+)\]\s+CABLE Input \(VB-Audio', line)
        if m:
            playback_id = int(m.group(1))

    if capture_id is None or playback_id is None:
        print("  VB-Cable not found. Available devices:")
        print(output)
    return capture_id, playback_id


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
            m = re.search(r'gearshift: (\d+)->(\d+)', line)
            if m:
                old, new = int(m.group(1)), int(m.group(2))
                if new != old:
                    info['gearshift_events'].append((old, new))
                if new > info['peak_level']:
                    info['peak_level'] = new
                    info['peak_level_name'] = f'A{new}'

            m = re.search(r'Probe.*band ([\d.]+)-([\d.]+) Hz \(([\d.]+) Hz BW\)', line)
            if m:
                info['probe_bandwidth'] = float(m.group(3))

            m = re.search(r'(\d+) tones detected', line)
            if m:
                info['probe_tones'] = int(m.group(1))

            m = re.search(r'baud rate to (\d+)', line)
            if m:
                info['probe_baud'] = int(m.group(1))

            m = re.search(r'\[CH-EQ\] Configured.*EQ range ([\d.]+) dB', line)
            if m:
                val = float(m.group(1))
                if info['rx_eq_range'] == 0:
                    info['rx_eq_range'] = val
                else:
                    info['tx_eq_range'] = val

            m = re.search(r'ldpc_iters=(\d+)', line)
            if m:
                ldpc_iters_total += int(m.group(1))
                ldpc_iters_count += 1

            if 'decode FAIL' in line:
                info['decode_failures'] += 1

            if 'RX native frame' in line and 'DISCARDED' not in line:
                info['frames_decoded'] += 1

            if 'OFDM-KISS' in line and ('native mode active' in line.lower() or 'activat' in line.lower()):
                info['native_activated'] = True

            m = re.search(r'\[([\d.]+)\].*CONNECTED', line)
            if m and info['connection_time'] == 0:
                info['connection_time'] = float(m.group(1))

            if 'decompress:' in line or 'compress:' in line:
                info['compression_events'] += 1

            if 'AX25 TX REJ' in line:
                info['nack_events'] += 1

    if ldpc_iters_count > 0:
        info['ldpc_iters_avg'] = ldpc_iters_total / ldpc_iters_count

    return info


def run_benchmark(iris_path, duration, bandpass, deemph_us, noise, payload_size,
                  capture_id, playback_id, extra_args):
    """Run two iris instances on VB-Cable and measure throughput."""
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(3)

    log_a = os.path.abspath("iris_fm_bench_A.log")
    log_b = os.path.abspath("iris_fm_bench_B.log")
    for f in [log_a, log_b]:
        if os.path.isfile(f):
            try: os.remove(f)
            except: pass

    # Common args for both instances
    common = [
        "--nogui", "--mode", "A",
        "--capture", str(capture_id),
        "--playback", str(playback_id),
        "--ptt-pre", "50",
        "--ptt-post", "50",
    ]
    if bandpass:
        common += ["--bandpass", bandpass]
    if deemph_us > 0:
        common += ["--deemphasis", str(deemph_us)]
    if noise > 0:
        common += ["--noise", str(noise)]
    common += extra_args

    # Instance A: ALPHA (AGW 8010, KISS 8011)
    cmd_a = [iris_path] + common + [
        "--callsign", "ALPHA",
        "--agw-port", "8010",
        "--kiss-port", "8011",
        "--log", log_a,
    ]

    # Instance B: BRAVO (AGW 8020, KISS 8021)
    cmd_b = [iris_path] + common + [
        "--callsign", "BRAVO",
        "--agw-port", "8020",
        "--kiss-port", "8021",
        "--log", log_b,
    ]

    print(f"  Instance A: {' '.join(cmd_a)}")
    print(f"  Instance B: {' '.join(cmd_b)}")

    proc_a = subprocess.Popen(cmd_a, creationflags=subprocess.CREATE_NEW_CONSOLE)
    time.sleep(1)
    proc_b = subprocess.Popen(cmd_b, creationflags=subprocess.CREATE_NEW_CONSOLE)

    # Connect to AGW on both instances
    sock_a = None
    sock_b = None
    for attempt in range(15):
        time.sleep(1)
        try:
            if sock_a is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3)
                s.connect(("127.0.0.1", 8010))
                sock_a = s
            if sock_b is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3)
                s.connect(("127.0.0.1", 8020))
                sock_b = s
            if sock_a and sock_b:
                break
        except Exception:
            pass
    if sock_a is None or sock_b is None:
        print("  AGW connect failed")
        proc_a.terminate()
        proc_b.terminate()
        return None

    print("  AGW connected to both instances")

    # Register callsigns
    sock_a.sendall(agw_build_header(ord('X'), call_from="ALPHA"))
    agw_recv_frame(sock_a, timeout=5)
    sock_b.sendall(agw_build_header(ord('X'), call_from="BRAVO"))
    agw_recv_frame(sock_b, timeout=5)
    print(f"  Registered ALPHA + BRAVO")

    # Connect ALPHA -> BRAVO
    print(f"  Connecting ALPHA -> BRAVO...")
    sock_a.sendall(agw_build_header(ord('C'), call_from="ALPHA", call_to="BRAVO"))

    # Wait for CONNECTED on both sides
    connect_start = time.time()
    connected_a = False
    connected_b = False
    while time.time() - connect_start < 90:
        if not connected_a:
            resp = agw_recv_frame(sock_a, timeout=1)
            if resp:
                hdr_info, data = resp
                if hdr_info['kind_char'] == 'C' and data and b'CONNECTED' in data:
                    connected_a = True
                    print(f"  ALPHA CONNECTED ({time.time() - connect_start:.1f}s)")
        if not connected_b:
            resp = agw_recv_frame(sock_b, timeout=1)
            if resp:
                hdr_info, data = resp
                if hdr_info['kind_char'] == 'C' and data and b'CONNECTED' in data:
                    connected_b = True
                    print(f"  BRAVO CONNECTED ({time.time() - connect_start:.1f}s)")
        if connected_a and connected_b:
            break
        if not connected_a and not connected_b:
            # Check for rejection
            pass

    if not connected_a:
        print(f"  Connection failed (ALPHA timeout)")
        sock_a.close()
        sock_b.close()
        proc_a.terminate()
        proc_b.terminate()
        return None
    # BRAVO might not get AGW notification — that's OK, modem still accepts
    if not connected_b:
        print(f"  (BRAVO AGW notification not received, continuing)")

    # Wait for probe + native mode
    print(f"  Waiting for probe + native mode...")
    settle_start = time.time()
    native_active = False
    while time.time() - settle_start < 90:
        # Drain any AGW frames from both sockets
        try:
            agw_recv_frame(sock_a, timeout=0.5)
        except: pass
        try:
            agw_recv_frame(sock_b, timeout=0.5)
        except: pass
        # Check logs for native mode activation
        if time.time() - settle_start > 15:
            log_a_info = parse_log(log_a)
            log_b_info = parse_log(log_b)
            if log_a_info['native_activated'] or log_b_info['native_activated']:
                native_active = True
                # Give gearshift time to settle
                time.sleep(5)
                break

    # Re-parse logs after settle
    log_a_info = parse_log(log_a)
    log_b_info = parse_log(log_b)
    probe_info = log_a_info if log_a_info['probe_tones'] > 0 else log_b_info

    if native_active:
        print(f"  Native mode ACTIVE (probe: {probe_info['probe_tones']} tones, "
              f"BW {probe_info['probe_bandwidth']:.0f} Hz, baud {probe_info['probe_baud']})")
    else:
        print(f"  Native mode not activated after {time.time() - settle_start:.0f}s (running on AFSK)")

    # Build test payload
    test_data = bytes([(i * 37 + 17) & 0xFF for i in range(payload_size)])

    print(f"  Running {duration}s measurement...")

    # Measurement phase: bidirectional data flow
    # Send ALPHA>BRAVO via sock_a, BRAVO→ALPHA via sock_b
    # Receive at both ends. This ensures both sides get enough frames for gearshift.
    tx_count_ab = 0  # ALPHA>BRAVO
    tx_count_ba = 0  # BRAVO→ALPHA
    rx_bytes_b = 0   # Received at BRAVO
    rx_frames_b = 0
    rx_bytes_a = 0   # Received at ALPHA
    rx_frames_a = 0

    start = time.time()
    last_report = start
    sock_a.settimeout(0.05)
    sock_b.settimeout(0.05)

    def try_recv_agw(sock):
        """Try to receive one AGW 'D' frame. Returns data length or 0."""
        try:
            hdr_buf = b""
            sock.settimeout(0.05)
            while len(hdr_buf) < 36:
                chunk = sock.recv(36 - len(hdr_buf))
                if not chunk:
                    return 0, False
                hdr_buf += chunk
            if len(hdr_buf) < 36:
                return 0, False
            hdr_info = agw_parse_header(hdr_buf)
            if not hdr_info:
                return 0, False
            data = b""
            remaining = hdr_info['data_len']
            sock.settimeout(2)
            while len(data) < remaining:
                chunk = sock.recv(remaining - len(data))
                if not chunk:
                    break
                data += chunk
            return len(data), hdr_info['kind_char'] == 'D'
        except socket.timeout:
            return 0, False
        except:
            return 0, False

    while time.time() - start < duration:
        elapsed = time.time() - start

        # Send data ALPHA → BRAVO
        ahead_ab = tx_count_ab - rx_frames_b
        to_send = max(0, min(10 - ahead_ab, 3))
        for _ in range(to_send):
            try:
                frame_hdr = agw_build_header(ord('D'), call_from="ALPHA",
                                              call_to="BRAVO", data_len=len(test_data))
                sock_a.sendall(frame_hdr + test_data)
                tx_count_ab += 1
            except: break

        # Send data BRAVO → ALPHA
        ahead_ba = tx_count_ba - rx_frames_a
        to_send = max(0, min(10 - ahead_ba, 3))
        for _ in range(to_send):
            try:
                frame_hdr = agw_build_header(ord('D'), call_from="BRAVO",
                                              call_to="ALPHA", data_len=len(test_data))
                sock_b.sendall(frame_hdr + test_data)
                tx_count_ba += 1
            except: break

        # Receive at BRAVO (data from ALPHA)
        dlen, is_data = try_recv_agw(sock_b)
        if is_data and dlen > 0:
            rx_frames_b += 1
            rx_bytes_b += dlen

        # Receive at ALPHA (data from BRAVO)
        dlen, is_data = try_recv_agw(sock_a)
        if is_data and dlen > 0:
            rx_frames_a += 1
            rx_bytes_a += dlen

        # Progress report every 15s
        total_rx_bytes = rx_bytes_a + rx_bytes_b
        total_rx_frames = rx_frames_a + rx_frames_b
        if time.time() - last_report >= 15:
            bps_so_far = total_rx_bytes * 8 / elapsed if elapsed > 0 else 0
            print(f"  [{elapsed:.0f}s] {total_rx_frames} frames, {total_rx_bytes} bytes, "
                  f"{bps_so_far:.0f} bps  (A>B:{rx_frames_b} B>A:{rx_frames_a})")
            last_report = time.time()

    measure_elapsed = time.time() - start
    rx_bytes = rx_bytes_a + rx_bytes_b
    rx_frames = rx_frames_a + rx_frames_b
    tx_count = tx_count_ab + tx_count_ba

    # Drain in-flight from both sides
    drain_end = time.time() + min(30, duration / 2)
    while time.time() < drain_end and (tx_count_ab > rx_frames_b or tx_count_ba > rx_frames_a):
        dlen, is_data = try_recv_agw(sock_b)
        if is_data and dlen > 0:
            rx_frames_b += 1
            rx_bytes_b += dlen
            rx_frames += 1
            rx_bytes += dlen
        dlen, is_data = try_recv_agw(sock_a)
        if is_data and dlen > 0:
            rx_frames_a += 1
            rx_bytes_a += dlen
            rx_frames += 1
            rx_bytes += dlen

    # Disconnect
    try:
        sock_a.sendall(agw_build_header(ord('d'), call_from="ALPHA", call_to="BRAVO"))
        time.sleep(1)
    except:
        pass

    sock_a.close()
    sock_b.close()

    # Parse final logs
    log_a_final = parse_log(log_a)
    log_b_final = parse_log(log_b)

    proc_a.terminate()
    proc_b.terminate()
    for p in [proc_a, proc_b]:
        try: p.wait(timeout=5)
        except: p.kill()

    # Merge results from both logs
    merged = {}
    for key in log_a_final:
        va, vb = log_a_final[key], log_b_final[key]
        if key == 'gearshift_events':
            merged[key] = va if va else vb
        elif key == 'native_activated':
            merged[key] = va or vb
        elif isinstance(va, (int, float)):
            merged[key] = max(va, vb)
        else:
            merged[key] = va if va else vb

    bps = rx_bytes * 8 / measure_elapsed if measure_elapsed > 0 else 0

    return {
        'rx_frames': rx_frames,
        'rx_bytes': rx_bytes,
        'tx_frames': tx_count,
        'duration': measure_elapsed,
        'throughput_bps': bps,
        **merged,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Iris FM Channel Simulator Benchmark (two-instance VB-Cable)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # Default FM channel (300-3200 Hz, 75us de-emphasis)
  %(prog)s --noise 0.005                      # Mild noise
  %(prog)s --noise 0.01                       # Moderate noise (like real VHF)
  %(prog)s --bandpass 500-2700 --noise 0.01   # Narrow radio + noise
  %(prog)s --no-deemphasis                    # Flat channel (no de-emphasis)
  %(prog)s --no-bandpass --no-deemphasis      # Clean VB-Cable (baseline)
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
    parser.add_argument("--noise", type=float, default=0.005,
                       help="AWGN noise amplitude (default: 0.005)")
    parser.add_argument("--tx-level", type=float, default=None)
    parser.add_argument("--capture", type=int, default=None,
                       help="Audio capture device ID (auto-detect VB-Cable)")
    parser.add_argument("--playback", type=int, default=None,
                       help="Audio playback device ID (auto-detect VB-Cable)")
    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    # Auto-detect VB-Cable if not specified
    capture_id = args.capture
    playback_id = args.playback
    if capture_id is None or playback_id is None:
        print("  Auto-detecting VB-Cable...")
        c, p = find_vb_cable(iris_path)
        if c is None or p is None:
            print("[ERROR] VB-Cable not found. Use --capture and --playback to specify device IDs.")
            print(f"  Run: {iris_path} --list-audio")
            return 1
        if capture_id is None:
            capture_id = c
        if playback_id is None:
            playback_id = p
        print(f"  VB-Cable: capture={capture_id}, playback={playback_id}")

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
        channel_desc.append("clean VB-Cable")

    print("=" * 70)
    print(f"  Iris FM Data Modem -- FM Channel Simulator Benchmark")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Channel: {', '.join(channel_desc)}")
    print(f"  Duration: {args.duration}s, payload: {args.payload} bytes")
    print(f"  Audio: capture={capture_id}, playback={playback_id}")
    print("=" * 70)

    result = run_benchmark(
        iris_path, args.duration, bandpass, deemph_us,
        args.noise, args.payload, capture_id, playback_id, extra
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
