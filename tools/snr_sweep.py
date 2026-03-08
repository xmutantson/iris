#!/usr/bin/env python3
"""
Iris FM Data Modem — AWGN SNR Sweep.

Measures throughput at each speed level across a range of SNR values
by injecting calibrated AWGN noise into the internal loopback.

The signal amplitude is fixed at tx_level (default 0.5).
Noise amplitude = signal_rms / 10^(SNR_dB/20).

Usage:
  python snr_sweep.py                        # Default sweep
  python snr_sweep.py --snr-min 0 --snr-max 40 --snr-step 2
  python snr_sweep.py --level 7              # Single level only
"""
import subprocess, socket, struct, time, sys, os, argparse, math, csv
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

KISS_FEND  = 0xC0
KISS_FESC  = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

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

# TX level used by iris (default). Upconverted signal RMS = tx_level / sqrt(2)
TX_LEVEL = 0.5
TX_SIGNAL_RMS = TX_LEVEL / math.sqrt(2)  # ~0.354


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


def snr_to_noise_amplitude(snr_db):
    """Convert desired passband SNR (dB) to AWGN noise amplitude.

    Passband SNR = 20*log10(signal_rms / noise_rms) at 48 kHz sample rate.
    The modem's demodulator achieves higher effective SNR due to processing
    gain from the downconverter LPF (~8 dB) and RRC matched filter (~2 dB).
    """
    signal_rms = 0.08  # Empirical RMS of upconverted signal at tx_level=0.5
    return signal_rms / (10.0 ** (snr_db / 20.0))


def run_test(iris_path, level, snr_db, duration, payload_size=200):
    """Run a single test: fixed speed level at given SNR. Returns throughput bps or None."""
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(2)

    noise_amp = snr_to_noise_amplitude(snr_db)
    log_file = os.path.abspath(f"iris_snr_A{level}_{snr_db:.0f}dB.log")
    if os.path.isfile(log_file):
        try: os.remove(log_file)
        except: pass

    cmd = [
        iris_path, "--loopback", "--nogui",
        "--callsign", "BENCH",
        "--mode", "A",
        "--speed-level", str(level),
        "--noise", f"{noise_amp:.6f}",
        "--log", log_file,
        "--ptt-pre", "0",
        "--ptt-post", "0",
    ]

    proc = subprocess.Popen(cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)

    # Wait for KISS port
    sock = None
    for attempt in range(6):
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
        proc.terminate()
        return None

    test_data = bytes([(i * 37 + 17) & 0xFF for i in range(payload_size)])
    kiss_frame = kiss_encode(test_data)

    # Warmup — send multiple frames and wait for at least one round-trip.
    # The decoder can occasionally miss a frame due to false preamble detection,
    # so we send 3 warmup frames spaced apart to ensure at least one decodes.
    time.sleep(1)
    warmup_ok = False
    rx_buf = b""
    sock.settimeout(1)
    for warmup_attempt in range(3):
        sock.sendall(kiss_frame)
        attempt_start = time.time()
        while time.time() - attempt_start < 8:
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
        if warmup_ok:
            break
        # Send another warmup frame
        time.sleep(1)

    if not warmup_ok:
        sock.close()
        proc.terminate()
        try: proc.wait(timeout=3)
        except: proc.kill()
        return {'bps': 0, 'tx_frames': 0, 'rx_frames': 0, 'crc_errors': 0}

    warmup_time = 2.0  # Drain timeout = min(warmup_time * 3, 8)

    # Measurement
    tx_count = 0
    rx_bytes = 0
    rx_frames = 0
    rx_buf = b""

    # Pacing: estimate frame time
    baud = 2400
    sl = SPEED_LEVELS[level]
    bps_per_sym = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6, "256QAM": 8}
    mod_bps = bps_per_sym[sl["mod"]]
    fec_str = sl["fec"]
    fec_num, fec_den = map(int, fec_str.split("/"))
    raw_bits = (payload_size + 4) * 8
    ldpc_k = {(1,2): 800, (3,4): 1200, (7,8): 1400}[(fec_num, fec_den)]
    n_blocks = (raw_bits + ldpc_k - 1) // ldpc_k
    coded_bits = n_blocks * 1600
    payload_symbols = (coded_bits + mod_bps - 1) // mod_bps
    overhead_symbols = 63 + 16 + 32  # preamble + sync + header
    total_symbols = overhead_symbols + payload_symbols
    frame_time = total_symbols / baud
    send_interval = frame_time * 0.95

    start = time.time()
    last_send = start - send_interval
    sock.settimeout(0.05)

    while time.time() - start < duration:
        now = time.time()
        if now - last_send >= send_interval and tx_count - rx_frames < 8:
            try:
                sock.sendall(kiss_frame)
                tx_count += 1
                last_send = now
            except:
                break

        try:
            chunk = sock.recv(65536)
            if chunk:
                rx_buf += chunk
        except socket.timeout:
            pass
        except:
            break

        if rx_buf:
            frames = kiss_decode_frames(rx_buf)
            rx_buf = b""
            for cmd, payload in frames:
                rx_frames += 1
                rx_bytes += len(payload)

    measure_elapsed = time.time() - start

    # Drain: collect in-flight frames
    in_flight = tx_count - rx_frames
    drain_timeout = min(frame_time * (in_flight + 2), 10)
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

    # Parse any remaining data
    if rx_buf:
        frames = kiss_decode_frames(rx_buf)
        for cmd, payload in frames:
            rx_frames += 1
            rx_bytes += len(payload)

    sock.close()
    proc.terminate()
    try: proc.wait(timeout=3)
    except: proc.kill()

    # Use measurement duration for throughput, not drain time
    bps = rx_bytes * 8 / measure_elapsed if measure_elapsed > 0 else 0

    # Count CRC errors
    crc_errors = 0
    if os.path.isfile(log_file):
        with open(log_file, errors='replace') as f:
            for line in f:
                if 'CRC mismatch' in line:
                    crc_errors += 1

    return {
        'bps': bps,
        'tx_frames': tx_count,
        'rx_frames': rx_frames,
        'crc_errors': crc_errors,
    }


def main():
    parser = argparse.ArgumentParser(description="Iris AWGN SNR Sweep")
    parser.add_argument("--iris", default=IRIS, help="Path to iris.exe")
    parser.add_argument("--duration", type=int, default=15,
                       help="Measurement duration per test point (default: 15s)")
    parser.add_argument("--level", type=int, default=None,
                       help="Test single speed level (0-7)")
    parser.add_argument("--snr-min", type=float, default=0,
                       help="Minimum SNR in dB (default: 0)")
    parser.add_argument("--snr-max", type=float, default=40,
                       help="Maximum SNR in dB (default: 40)")
    parser.add_argument("--snr-step", type=float, default=3,
                       help="SNR step in dB (default: 3)")
    parser.add_argument("--payload", type=int, default=200,
                       help="Payload size in bytes (default: 200)")
    parser.add_argument("--csv", type=str, default=None,
                       help="Output CSV file")
    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    levels = [args.level] if args.level is not None else list(range(len(SPEED_LEVELS)))
    snr_values = []
    snr = args.snr_min
    while snr <= args.snr_max + 0.01:
        snr_values.append(snr)
        snr += args.snr_step

    print("=" * 78)
    print(f"  Iris FM Data Modem — AWGN SNR Sweep")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Duration: {args.duration}s, payload: {args.payload} bytes")
    print(f"  Levels: {', '.join(SPEED_LEVELS[l]['name'] for l in levels)}")
    print(f"  SNR range: {args.snr_min}-{args.snr_max} dB, step {args.snr_step} dB")
    print(f"  SNR type: passband (48 kHz), signal RMS: 0.08")
    print("=" * 78)

    # results[level][snr] = bps
    results = {}

    for level in levels:
        sl = SPEED_LEVELS[level]
        results[level] = {}
        print(f"\n--- {sl['name']} ({sl['mod']} {sl['fec']}, PHY {sl['phy_bps']} bps) ---")

        # Sweep from high to low SNR; stop when decode fails
        for snr_db in reversed(snr_values):
            noise_amp = snr_to_noise_amplitude(snr_db)
            sys.stdout.write(f"  SNR {snr_db:5.1f} dB (noise={noise_amp:.4f}): ")
            sys.stdout.flush()

            result = run_test(iris_path, level, snr_db, args.duration, args.payload)
            if result is None:
                print("CONNECT FAILED")
                results[level][snr_db] = 0
            elif result['rx_frames'] == 0:
                print(f"NO DECODE")
                results[level][snr_db] = 0
                # Lower SNR values will also fail — fill zeros and stop
                for lower_snr in snr_values:
                    if lower_snr < snr_db:
                        results[level][lower_snr] = 0
                break
            else:
                eff = result['bps'] / sl['phy_bps'] * 100 if sl['phy_bps'] > 0 else 0
                crc_str = f" CRC={result['crc_errors']}" if result['crc_errors'] > 0 else ""
                print(f"{result['bps']:7.0f} bps ({eff:5.1f}%) "
                      f"TX={result['tx_frames']} RX={result['rx_frames']}{crc_str}")
                results[level][snr_db] = result['bps']

    # Summary table
    print("\n" + "=" * 78)
    print("THROUGHPUT TABLE (bps)")
    print("=" * 78)

    # Header
    header = f"{'SNR(dB)':>8}"
    for level in levels:
        header += f"  {SPEED_LEVELS[level]['name']:>8}"
    print(header)
    print("-" * (8 + 10 * len(levels)))

    for snr_db in snr_values:
        row = f"{snr_db:8.1f}"
        for level in levels:
            bps = results.get(level, {}).get(snr_db, 0)
            if bps > 0:
                row += f"  {bps:8.0f}"
            else:
                row += f"  {'---':>8}"
        print(row)

    print("=" * (8 + 10 * len(levels)))

    # PHY rate row
    row = f"{'PHY bps':>8}"
    for level in levels:
        row += f"  {SPEED_LEVELS[level]['phy_bps']:>8}"
    print(row)

    # ASCII plot: throughput vs SNR for each level
    print("\n" + "=" * 78)
    print("THROUGHPUT vs SNR")
    print("=" * 78)

    max_bps = max(
        max(results.get(l, {}).values()) if results.get(l, {}).values() else 0
        for l in levels
    )
    if max_bps == 0:
        max_bps = 1

    plot_width = 60
    symbols = "01234567"

    for snr_db in snr_values:
        row = f"{snr_db:5.1f} |"
        # Build bar for each level
        bars = {}
        for level in levels:
            bps = results.get(level, {}).get(snr_db, 0)
            pos = int(bps / max_bps * plot_width) if max_bps > 0 else 0
            if bps > 0:
                bars[pos] = symbols[level % len(symbols)]

        line = list(" " * (plot_width + 1))
        for pos, ch in sorted(bars.items()):
            if pos <= plot_width:
                line[pos] = ch
        row += "".join(line)
        print(row)

    print(f"      +{'-' * (plot_width + 1)}")
    print(f"      0{' ' * (plot_width // 2 - 3)}bps{' ' * (plot_width // 2 - 3)}{max_bps:.0f}")
    print(f"\nLegend: " + " ".join(f"{symbols[l]}={SPEED_LEVELS[l]['name']}" for l in levels))

    # CSV output
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            header_row = ['SNR_dB'] + [SPEED_LEVELS[l]['name'] for l in levels]
            writer.writerow(header_row)
            for snr_db in snr_values:
                row = [snr_db] + [results.get(l, {}).get(snr_db, 0) for l in levels]
                writer.writerow(row)
        print(f"\nCSV written to {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
