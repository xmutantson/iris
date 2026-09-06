#!/usr/bin/env python3
"""
Iris FM Data Modem — VB-Cable Two-Instance Benchmark.

Runs two iris.exe instances (TX and RX) connected via VB-Cable virtual audio,
floods KISS frames into TX, reads decoded frames from RX, measures throughput.

Usage:
  python vbcable_benchmark.py                    # All modes, 15s each
  python vbcable_benchmark.py --level 7          # Single level
  python vbcable_benchmark.py --duration 30      # 30s per mode
"""
import subprocess, socket, time, sys, os, argparse
from datetime import datetime

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS):
    IRIS = r"C:\Program Files\Iris\iris.exe"

# Audio device IDs (from --list-audio)
# Single VB-Cable shared by both instances (like Mercury HF setup):
# Both TX and RX use the same cable. TX mute prevents self-hearing.
CABLE_PLAYBACK = 1005   # CABLE Input (VB-Audio Virtual Cable) — TX plays here
CABLE_CAPTURE  = 11     # CABLE Output (VB-Audio Virtual Cable) — RX captures here

TX_KISS_PORT = 8001
RX_KISS_PORT = 8002

KISS_FEND  = 0xC0
KISS_FESC  = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD

SPEED_LEVELS = [
    {"name": "A0", "mod": "BPSK",   "fec": "1/2", "phy_bps": 1200},
    {"name": "A1", "mod": "QPSK",   "fec": "1/2", "phy_bps": 2400},
    {"name": "A2", "mod": "QPSK",   "fec": "3/4", "phy_bps": 3600},
    {"name": "A3", "mod": "16QAM",  "fec": "1/2", "phy_bps": 4800},
    {"name": "A4", "mod": "16QAM",  "fec": "3/4", "phy_bps": 7200},
    {"name": "A5", "mod": "64QAM",  "fec": "3/4", "phy_bps": 10800},
    {"name": "A6", "mod": "64QAM",  "fec": "7/8", "phy_bps": 12600},
    {"name": "A7", "mod": "256QAM", "fec": "7/8", "phy_bps": 16800},
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


def flush_vbcable(playback_id, capture_id):
    """Flush VB-Cable buffer by playing silence and draining capture."""
    try:
        import numpy as np
        import sounddevice as sd
        # Find WASAPI device IDs corresponding to our playback/capture
        # sounddevice uses its own device numbering
        devs = sd.query_devices()
        play_sd = None
        cap_sd = None
        for i, d in enumerate(devs):
            if 'WASAPI' not in sd.query_hostapis(d['hostapi'])['name']:
                continue
            if 'CABLE Input' in d['name'] and d['max_output_channels'] > 0 and play_sd is None:
                play_sd = i
            if 'CABLE Output' in d['name'] and d['max_input_channels'] > 0 and cap_sd is None:
                cap_sd = i
        if play_sd is not None:
            sd.play(np.zeros((48000*2, 1), dtype=np.float32), samplerate=48000, device=play_sd)
            sd.wait()
        if cap_sd is not None:
            sd.rec(48000, samplerate=48000, channels=1, device=cap_sd)
            sd.wait()
    except Exception:
        # sounddevice not available — just wait
        time.sleep(3)


def run_level(iris_path, level, duration, playback_id, capture_id, payload_size=200):
    """Run a single speed level benchmark over VB-Cable. Returns dict with results."""
    os.system("taskkill /F /IM iris.exe 2>nul >nul")
    time.sleep(3)
    flush_vbcable(playback_id, capture_id)

    sl = SPEED_LEVELS[level]

    # TX plays to VB-Cable, captures from CABLE-A (dummy).
    # RX captures from VB-Cable, plays to CABLE-A (dummy).
    # Each instance gets its own unused-direction device to avoid WASAPI conflicts.
    DUMMY_CAPTURE  = 7      # CABLE-A Output (TX captures from here, unused)
    DUMMY_PLAYBACK = 1001   # CABLE-A Input (RX plays to here, unused)

    tx_log = os.path.join(os.path.dirname(iris_path), f"tx_log_{level}.txt")
    rx_log = os.path.join(os.path.dirname(iris_path), f"rx_log_{level}.txt")

    tx_cmd = [
        iris_path, "--nogui",
        "--callsign", "TX",
        "--mode", "A",
        "--speed-level", str(level),
        "--playback", str(playback_id),
        "--capture", str(DUMMY_CAPTURE),
        "--port", str(TX_KISS_PORT),
        "--ptt-pre", "0",
        "--ptt-post", "0",
        "--log", tx_log,
    ]

    rx_cmd = [
        iris_path, "--nogui",
        "--callsign", "RX",
        "--mode", "A",
        "--speed-level", str(level),
        "--capture", str(capture_id),
        "--playback", str(DUMMY_PLAYBACK),
        "--port", str(RX_KISS_PORT),
        "--ptt-pre", "0",
        "--ptt-post", "0",
        "--log", rx_log,
    ]

    # Start RX first so it drains any residual VB-Cable audio
    rx_proc = subprocess.Popen(rx_cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)
    time.sleep(5)  # Let RX run 5s to consume stale audio
    tx_proc = subprocess.Popen(tx_cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)

    # Connect to TX KISS port
    tx_sock = None
    for attempt in range(8):
        time.sleep(1)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect(("127.0.0.1", TX_KISS_PORT))
            tx_sock = s
            break
        except:
            s.close()
    if tx_sock is None:
        print("    TX KISS connect failed")
        tx_proc.terminate(); rx_proc.terminate()
        return None

    # Connect to RX KISS port
    rx_sock = None
    for attempt in range(8):
        time.sleep(1)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect(("127.0.0.1", RX_KISS_PORT))
            rx_sock = s
            break
        except:
            s.close()
    if rx_sock is None:
        print("    RX KISS connect failed")
        tx_sock.close()
        tx_proc.terminate(); rx_proc.terminate()
        return None

    test_data = bytes([(i * 37 + 17) & 0xFF for i in range(payload_size)])
    kiss_frame = kiss_encode(test_data)

    # Warmup: send frames and wait for first decode on RX
    print("    Warming up...")
    rx_sock.settimeout(0.5)
    rx_buf = b""
    warmup_ok = False
    warmup_start = time.time()
    frames_sent = 0
    while time.time() - warmup_start < 60:
        tx_sock.sendall(kiss_frame)
        frames_sent += 1
        # Check RX for 2 seconds
        check_end = time.time() + 2
        while time.time() < check_end:
            try:
                chunk = rx_sock.recv(4096)
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

    if not warmup_ok:
        print(f"    Warmup failed (no decode after {frames_sent} frames in 60s)")
        tx_sock.close(); rx_sock.close()
        tx_proc.terminate(); rx_proc.terminate()
        return None

    print(f"    Warmup OK. Measuring for {duration}s...")

    # Measurement: flood-feed frames so modem batches them into multi-payload
    # transmissions (one preamble for many sub-frames).
    tx_count = 0
    rx_bytes = 0
    rx_frames = 0
    rx_buf = b""

    # Keep the TX queue fed: send a burst whenever the modem might be ready.
    # The modem batches up to min(4000 bytes, phy_bps*3/8) per transmission.
    phy_bps = sl["phy_bps"]
    max_batch_bytes = min(4000, phy_bps * 3 // 8)
    frames_per_batch = max(1, max_batch_bytes // (payload_size + 3))
    # Estimate air time for a full batch
    fec_str = sl["fec"]
    fec_num, fec_den = map(int, fec_str.split("/"))
    bps_per_sym = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6, "256QAM": 8}
    sym_per_bit = 1.0 / bps_per_sym[sl["mod"]]
    overhead_syms = 63 + 16 + 32  # preamble + sync + header
    batch_data_bits = (max_batch_bytes + 4) * 8
    k = 800
    n = 1600
    num_blocks = (batch_data_bits + k - 1) // k
    coded_bits = num_blocks * n
    payload_syms = int(coded_bits * sym_per_bit + 0.5)
    batch_air_time = (overhead_syms + payload_syms) / 2400.0
    # Feed interval: send a burst of frames every batch_air_time
    feed_interval = max(0.5, batch_air_time * 0.8)

    start = time.time()
    rx_sock.settimeout(0.05)
    last_feed = 0

    while time.time() - start < duration:
        now = time.time()
        # Burst-feed frames so modem can batch them
        if now - last_feed >= feed_interval:
            try:
                for _ in range(frames_per_batch):
                    tx_sock.sendall(kiss_frame)
                    tx_count += 1
                last_feed = now
            except:
                break

        # Drain RX
        try:
            chunk = rx_sock.recv(65536)
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

    # Drain: collect remaining frames from RX
    drain_end = time.time() + 15
    rx_sock.settimeout(0.5)
    while time.time() < drain_end and tx_count > rx_frames:
        try:
            chunk = rx_sock.recv(65536)
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

    tx_sock.close()
    rx_sock.close()
    tx_proc.terminate()
    rx_proc.terminate()
    try: tx_proc.wait(timeout=3)
    except: tx_proc.kill()
    try: rx_proc.wait(timeout=3)
    except: rx_proc.kill()

    bps = rx_bytes * 8 / measure_elapsed if measure_elapsed > 0 else 0

    return {
        'level': level,
        'tx_frames': tx_count,
        'rx_frames': rx_frames,
        'rx_bytes': rx_bytes,
        'duration': measure_elapsed,
        'throughput_bps': bps,
        'payload_size': payload_size,
    }


def main():
    parser = argparse.ArgumentParser(description="Iris VB-Cable Two-Instance Benchmark")
    parser.add_argument("--iris", default=IRIS, help="Path to iris.exe")
    parser.add_argument("--duration", type=int, default=15,
                       help="Measurement duration per level (default: 15s)")
    parser.add_argument("--level", type=int, default=None,
                       help="Test single speed level (0-7)")
    parser.add_argument("--payload", type=int, default=200,
                       help="Payload size in bytes (default: 200)")
    parser.add_argument("--tx-playback", type=int, default=CABLE_PLAYBACK,
                       help=f"TX playback device ID (default: {CABLE_PLAYBACK})")
    parser.add_argument("--rx-capture", type=int, default=CABLE_CAPTURE,
                       help=f"RX capture device ID (default: {CABLE_CAPTURE})")
    args = parser.parse_args()

    playback_id = args.tx_playback
    capture_id = args.rx_capture

    iris_path = os.path.abspath(args.iris)
    if not os.path.isfile(iris_path):
        print(f"[ERROR] Not found: {iris_path}")
        return 1

    levels = [args.level] if args.level is not None else list(range(len(SPEED_LEVELS)))

    print("=" * 70)
    print(f"  Iris FM Data Modem — VB-Cable Benchmark")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Duration: {args.duration}s per level, payload: {args.payload} bytes")
    print(f"  Levels: {', '.join(SPEED_LEVELS[l]['name'] for l in levels)}")
    print(f"  Audio: playback={playback_id}, capture={capture_id}")
    print("=" * 70)

    results = []
    for level in levels:
        sl = SPEED_LEVELS[level]
        print(f"\n[A{level}] {sl['mod']} {sl['fec']} (PHY {sl['phy_bps']} bps)...")

        result = run_level(iris_path, level, args.duration, playback_id, capture_id, args.payload)
        if result is None:
            print(f"    FAILED")
            results.append({'level': level, 'throughput_bps': 0, 'rx_frames': 0})
            continue

        eff = result['throughput_bps'] / sl['phy_bps'] * 100 if sl['phy_bps'] > 0 else 0
        print(f"    TX: {result['tx_frames']} frames, "
              f"RX: {result['rx_frames']} frames ({result['rx_bytes']} bytes)")
        print(f"    Throughput: {result['throughput_bps']:.0f} bps "
              f"({eff:.1f}% of PHY {sl['phy_bps']} bps)")
        results.append(result)

    # Summary table
    print("\n" + "=" * 78)
    print(f"{'Level':<8} {'Mod':<8} {'FEC':<6} {'TX':<6} {'RX':<6} {'Bytes':<8} "
          f"{'bps':<10} {'PHY bps':<10} {'Eff%':<6}")
    print("-" * 78)
    for r in results:
        level = r['level']
        sl = SPEED_LEVELS[level]
        bps = r.get('throughput_bps', 0)
        eff = bps / sl['phy_bps'] * 100 if sl['phy_bps'] > 0 else 0
        print(f"{sl['name']:<8} {sl['mod']:<8} {sl['fec']:<6} "
              f"{r.get('tx_frames', 0):<6} {r.get('rx_frames', 0):<6} "
              f"{r.get('rx_bytes', 0):<8} {bps:<10.0f} {sl['phy_bps']:<10} "
              f"{eff:<6.1f}")
    print("=" * 78)

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
