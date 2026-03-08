#!/usr/bin/env python3
"""
Iris FM Data Modem — Internal Loopback Benchmark.

Measures PHY-layer throughput for each speed level by flooding KISS frames
through the internal loopback and counting decoded bytes.

Usage:
  python loopback_benchmark.py                    # All modes, 15s each
  python loopback_benchmark.py --duration 30      # 30s per mode
  python loopback_benchmark.py --level 0          # Single level only
"""
import subprocess, socket, struct, time, sys, os, argparse, signal, select
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

KISS_FEND  = 0xC0
KISS_FESC  = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

# Speed levels (must match speed_level.cc)
SPEED_LEVELS = [
    {"name": "A0", "mod": "BPSK",   "fec": "1/2", "min_snr": 3.0,  "phy_bps": 1200},
    {"name": "A1", "mod": "QPSK",   "fec": "1/2", "min_snr": 6.0,  "phy_bps": 2400},
    {"name": "A2", "mod": "QPSK",   "fec": "3/4", "min_snr": 9.0,  "phy_bps": 3600},
    {"name": "A3", "mod": "16QAM",  "fec": "1/2", "min_snr": 14.0, "phy_bps": 4800},
    {"name": "A4", "mod": "16QAM",  "fec": "3/4", "min_snr": 17.0, "phy_bps": 7200},
    {"name": "A5", "mod": "64QAM",  "fec": "3/4", "min_snr": 22.0, "phy_bps": 10800},
    {"name": "A6", "mod": "64QAM",  "fec": "7/8", "min_snr": 25.0, "phy_bps": 12600},
    {"name": "A7", "mod": "256QAM", "fec": "7/8", "min_snr": 30.0, "phy_bps": 16800},
]


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


def kiss_decode_frames(raw):
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


def run_level(iris_path, level, duration, payload_size=200, extra_args=None):
    """Run a single speed level benchmark. Returns dict with results."""
    extra = extra_args or []

    # Kill any existing iris and wait for full cleanup
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(5)  # Wait for process exit + TIME_WAIT on port 8001

    log_file = os.path.abspath(f"iris_bench_A{level}.log")
    # Remove old log to avoid appending
    if os.path.isfile(log_file):
        try: os.remove(log_file)
        except: pass

    cmd = [
        iris_path, "--loopback", "--nogui",
        "--callsign", "BENCH",
        "--mode", "A",
        "--speed-level", str(level),
        "--log", log_file,
        "--ptt-pre", "0",     # No PTT delay in loopback
        "--ptt-post", "0",
        *extra,
    ]

    proc = subprocess.Popen(cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)

    # Wait for KISS port to become available (retry connect)
    sock = None
    for attempt in range(10):
        time.sleep(1)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect(("127.0.0.1", 8001))
            sock = s
            break
        except Exception:
            s.close()
    if sock is None:
        print(f"    KISS connect failed after 10 attempts")
        proc.terminate()
        return None

    # Build test payload
    test_data = bytes([(i * 37 + 17) & 0xFF for i in range(payload_size)])
    kiss_frame = kiss_encode(test_data)

    # Warmup: send a frame and wait for round-trip
    sock.sendall(kiss_frame)
    print(f"    Warming up (waiting for first round-trip)...")
    sock.settimeout(1)
    rx_buf = b""
    warmup_start = time.time()
    warmup_ok = False
    while time.time() - warmup_start < 20:
        try:
            chunk = sock.recv(4096)
            if chunk:
                rx_buf += chunk
                frames = kiss_decode_frames(rx_buf)
                if frames:
                    warmup_ok = True
                    break
        except socket.timeout:
            continue

    if not warmup_ok:
        print(f"    Warmup failed (no loopback response in 20s)")
        sock.close()
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except:
            proc.kill()
        return None

    warmup_time = time.time() - warmup_start
    print(f"    Round-trip: {warmup_time:.1f}s")

    # Measurement phase: pipeline frames, pacing TX to match PHY rate
    tx_count = 0
    rx_bytes = 0
    rx_frames = 0
    rx_buf = b""

    # Compute TX time per frame from PHY rate
    phy_bps = SPEED_LEVELS[level]["phy_bps"]
    frame_bits = payload_size * 8
    # Time for one frame = payload bits / PHY rate + overhead (preamble, header, FEC)
    # FEC overhead: rate 1/2 = 2x, 3/4 = 1.33x, 7/8 = 1.14x
    fec_str = SPEED_LEVELS[level]["fec"]
    fec_num, fec_den = map(int, fec_str.split("/"))
    baud = 2400
    bps_per_sym = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6, "256QAM": 8}
    mod_bps = bps_per_sym[SPEED_LEVELS[level]["mod"]]
    # Frame duration: (preamble + sync + header + payload_symbols) / baud
    encoded_bits = frame_bits * fec_den / fec_num  # approximate
    # Add CRC32 + LDPC blocks
    raw_bits = (payload_size + 4) * 8
    ldpc_k = {(1,2): 800, (3,4): 1200, (7,8): 1400}[(fec_num, fec_den)]
    n_blocks = (raw_bits + ldpc_k - 1) // ldpc_k
    coded_bits = n_blocks * 1600
    payload_symbols = (coded_bits + mod_bps - 1) // mod_bps
    overhead_symbols = 63 + 16 + 32  # preamble + sync + header
    total_symbols = overhead_symbols + payload_symbols
    frame_time = total_symbols / baud
    # Keep 2 frames in-flight (pipeline depth)
    send_interval = frame_time * 0.95  # slightly faster than TX to keep pipeline full

    print(f"    Frame: {total_symbols} sym, {frame_time:.3f}s, interval: {send_interval:.3f}s")

    start = time.time()
    sock.settimeout(0.01)

    # For multi-payload batching: keep a deep queue of frames so the modem
    # always has data ready to batch when TX fires. We send many more frames
    # than can be decoded in time, but that's fine — we measure RX throughput.
    target_ahead = 100

    while time.time() - start < duration:
        # Send frames to maintain target queue depth
        ahead = tx_count - rx_frames
        to_send = max(0, min(target_ahead - ahead + 10, 50))
        for _ in range(to_send):
            try:
                sock.sendall(kiss_frame)
                tx_count += 1
            except:
                break

        # Drain received frames
        for _ in range(50):
            try:
                chunk = sock.recv(65536)
                if chunk:
                    rx_buf += chunk
                else:
                    break
            except socket.timeout:
                break
            except:
                break

        # Parse complete frames
        if rx_buf:
            frames = kiss_decode_frames(rx_buf)
            rx_buf = b""
            for cmd, payload in frames:
                rx_frames += 1
                rx_bytes += len(payload)

    measure_elapsed = time.time() - start

    # Drain: collect in-flight frames (larger timeout for multi-payload batching)
    in_flight = tx_count - rx_frames
    drain_timeout = min(frame_time * 5 + 5, 30)  # Allow time for batched frames
    drain_end = time.time() + drain_timeout
    sock.settimeout(0.5)
    while time.time() < drain_end and tx_count > rx_frames:
        try:
            chunk = sock.recv(65536)
            if chunk:
                rx_buf += chunk
            else:
                break
        except (socket.timeout, BlockingIOError):
            continue
        except:
            break
        if rx_buf:
            frames = kiss_decode_frames(rx_buf)
            rx_buf = b""
            for cmd, payload in frames:
                rx_frames += 1
                rx_bytes += len(payload)

    if rx_buf:
        frames = kiss_decode_frames(rx_buf)
        for cmd, payload in frames:
            rx_frames += 1
            rx_bytes += len(payload)

    sock.close()
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except:
        proc.kill()

    # Read CRC errors from log
    crc_errors = 0
    if os.path.isfile(log_file):
        with open(log_file, errors='replace') as f:
            for line in f:
                if 'CRC mismatch' in line:
                    crc_errors += 1

    # Use measurement duration for throughput, not elapsed+drain
    bps = rx_bytes * 8 / measure_elapsed if measure_elapsed > 0 else 0

    return {
        'level': level,
        'tx_frames': tx_count,
        'rx_frames': rx_frames,
        'rx_bytes': rx_bytes,
        'duration': measure_elapsed,
        'throughput_bps': bps,
        'crc_errors': crc_errors,
        'payload_size': payload_size,
    }


def main():
    parser = argparse.ArgumentParser(description="Iris Internal Loopback Benchmark")
    parser.add_argument("--iris", default=IRIS, help="Path to iris.exe")
    parser.add_argument("--duration", type=int, default=15,
                       help="Measurement duration per level (default: 15s)")
    parser.add_argument("--level", type=int, default=None,
                       help="Test single speed level (0-7)")
    parser.add_argument("--payload", type=int, default=200,
                       help="Payload size in bytes (default: 200)")
    parser.add_argument("--tx-level", type=float, default=None)
    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    levels = [args.level] if args.level is not None else list(range(len(SPEED_LEVELS)))

    extra = []
    if args.tx_level is not None:
        extra += ["--tx-level", str(args.tx_level)]

    print("=" * 70)
    print(f"  Iris FM Data Modem — Internal Loopback Benchmark")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Duration: {args.duration}s per level, payload: {args.payload} bytes")
    print(f"  Levels: {', '.join(SPEED_LEVELS[l]['name'] for l in levels)}")
    print("=" * 70)

    results = []
    for level in levels:
        sl = SPEED_LEVELS[level]
        print(f"\n[A{level}] {sl['mod']} {sl['fec']} (PHY {sl['phy_bps']} bps)...")

        result = run_level(iris_path, level, args.duration, args.payload, extra)
        if result is None:
            print(f"    FAILED")
            results.append({'level': level, 'throughput_bps': 0, 'rx_frames': 0})
            continue

        eff = result['throughput_bps'] / sl['phy_bps'] * 100 if sl['phy_bps'] > 0 else 0
        print(f"    TX: {result['tx_frames']} frames, "
              f"RX: {result['rx_frames']} frames ({result['rx_bytes']} bytes)")
        print(f"    Throughput: {result['throughput_bps']:.0f} bps "
              f"({eff:.1f}% of PHY {sl['phy_bps']} bps)")
        if result['crc_errors'] > 0:
            print(f"    CRC errors: {result['crc_errors']}")
        results.append(result)

    # Summary table
    print("\n" + "=" * 78)
    print(f"{'Level':<8} {'Mod':<8} {'FEC':<6} {'TX':<6} {'RX':<6} {'Bytes':<8} "
          f"{'bps':<10} {'PHY bps':<10} {'Eff%':<6} {'CRC'}")
    print("-" * 78)
    for r in results:
        level = r['level']
        sl = SPEED_LEVELS[level]
        bps = r.get('throughput_bps', 0)
        eff = bps / sl['phy_bps'] * 100 if sl['phy_bps'] > 0 else 0
        print(f"{sl['name']:<8} {sl['mod']:<8} {sl['fec']:<6} "
              f"{r.get('tx_frames', 0):<6} {r.get('rx_frames', 0):<6} "
              f"{r.get('rx_bytes', 0):<8} {bps:<10.0f} {sl['phy_bps']:<10} "
              f"{eff:<6.1f} {r.get('crc_errors', 0)}")
    print("=" * 78)

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
