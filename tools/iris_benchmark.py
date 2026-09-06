#!/usr/bin/env python3
"""
Iris FM Data Modem — VB-Cable Loopback Benchmark.

Launches two Iris instances (commander + responder) cross-wired through
two VB-Cable virtual audio cables, connects via AGW protocol, pumps data,
and measures throughput.

Requires: numpy (optional, for noise injection), two VB-Cable instances

Audio routing:
  CMD TX → CABLE   playback → CABLE   capture → RSP RX
  RSP TX → CABLE-A playback → CABLE-A capture → CMD RX

Usage:
  python iris_benchmark.py [options]
  python iris_benchmark.py --list-audio          # List audio devices
  python iris_benchmark.py --duration 60         # 60 second measurement
  python iris_benchmark.py --gui                 # Show RSP GUI
"""
import subprocess, socket, struct, time, sys, threading, os, argparse
from datetime import datetime

# Defaults
IRIS_DEFAULT = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
if not os.path.isfile(IRIS_DEFAULT):
    IRIS_DEFAULT = r"C:\Program Files\Iris\iris.exe"

# VB-Cable device IDs (from --list-audio)
# Override with --cmd-capture, --cmd-playback, --rsp-capture, --rsp-playback
DEFAULT_CMD_CAPTURE  = 7     # CABLE-A Output (RSP TX arrives here)
DEFAULT_CMD_PLAYBACK = 1005  # CABLE Input    (CMD TX goes here → RSP RX)
DEFAULT_RSP_CAPTURE  = 11    # CABLE Output   (CMD TX arrives here)
DEFAULT_RSP_PLAYBACK = 1001  # CABLE-A Input  (RSP TX goes here → CMD RX)

# AGW ports
CMD_AGW_PORT = 8010
RSP_AGW_PORT = 8000

# KISS ports (must not conflict)
CMD_KISS_PORT = 8011
RSP_KISS_PORT = 8001

# AGW protocol constants
AGW_HEADER_SIZE = 36
CMD_CALLSIGN = "TESTA"
RSP_CALLSIGN = "TESTB"


def build_agw_header(kind, call_from="", call_to="", data_len=0, pid=0xF0):
    """Build a 36-byte AGW header."""
    hdr = bytearray(36)
    hdr[0] = 0          # port
    hdr[4] = ord(kind) if isinstance(kind, str) else kind
    hdr[6] = pid
    # call_from at offset 8 (10 bytes)
    cf = call_from.encode('ascii')[:9]
    hdr[8:8+len(cf)] = cf
    # call_to at offset 18 (10 bytes)
    ct = call_to.encode('ascii')[:9]
    hdr[18:18+len(ct)] = ct
    # data_len at offset 28 (uint32 LE)
    struct.pack_into('<I', hdr, 28, data_len)
    return bytes(hdr)


def parse_agw_header(data):
    """Parse a 36-byte AGW header. Returns dict."""
    if len(data) < 36:
        return None
    return {
        'port': data[0],
        'kind': chr(data[4]),
        'pid': data[6],
        'call_from': data[8:18].split(b'\x00')[0].decode('ascii', errors='replace'),
        'call_to': data[18:28].split(b'\x00')[0].decode('ascii', errors='replace'),
        'data_len': struct.unpack_from('<I', data, 28)[0],
    }


def recv_exact(sock, n, timeout=10):
    """Receive exactly n bytes."""
    sock.settimeout(timeout)
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf


def recv_agw_frame(sock, timeout=10):
    """Receive one AGW frame (header + data). Returns (header_dict, data_bytes)."""
    hdr_bytes = recv_exact(sock, AGW_HEADER_SIZE, timeout)
    hdr = parse_agw_header(hdr_bytes)
    if hdr is None:
        raise ValueError("Invalid AGW header")
    data = b''
    if hdr['data_len'] > 0:
        data = recv_exact(sock, hdr['data_len'], timeout)
    return hdr, data


def send_agw_frame(sock, kind, call_from="", call_to="", data=b""):
    """Send an AGW frame."""
    hdr = build_agw_header(kind, call_from, call_to, len(data))
    sock.sendall(hdr + data)


def tcp_connect_retry(port, timeout=5, retries=15, delay=1):
    """Connect to TCP port with retries."""
    for attempt in range(retries):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect(("127.0.0.1", port))
            return sock
        except (ConnectionRefusedError, socket.timeout, OSError):
            if attempt == retries - 1:
                raise
            time.sleep(delay)
            try:
                sock.close()
            except:
                pass
    raise ConnectionError(f"Failed to connect to port {port}")


def collect_output(proc, lines_list, stop_event):
    """Read stdout lines from a process into a list."""
    try:
        for line in iter(proc.stdout.readline, b''):
            if stop_event.is_set():
                break
            text = line.decode('utf-8', errors='replace').rstrip()
            lines_list.append(text)
    except:
        pass


class IrisSession:
    """Manages a commander + responder Iris modem pair for benchmarking."""

    def __init__(self, iris_path=IRIS_DEFAULT, use_gui=False,
                 cmd_capture=DEFAULT_CMD_CAPTURE, cmd_playback=DEFAULT_CMD_PLAYBACK,
                 rsp_capture=DEFAULT_RSP_CAPTURE, rsp_playback=DEFAULT_RSP_PLAYBACK,
                 cmd_agw_port=CMD_AGW_PORT, rsp_agw_port=RSP_AGW_PORT,
                 cmd_kiss_port=CMD_KISS_PORT, rsp_kiss_port=RSP_KISS_PORT,
                 extra_args=None):
        self.iris_path = os.path.abspath(iris_path)
        self.use_gui = use_gui
        self.cmd_capture = cmd_capture
        self.cmd_playback = cmd_playback
        self.rsp_capture = rsp_capture
        self.rsp_playback = rsp_playback
        self.cmd_agw_port = cmd_agw_port
        self.rsp_agw_port = rsp_agw_port
        self.cmd_kiss_port = cmd_kiss_port
        self.rsp_kiss_port = rsp_kiss_port
        self.extra_args = extra_args or []

        self.procs = []
        self.cmd_lines = []
        self.rsp_lines = []
        self.stop_event = threading.Event()
        self._tx_thread = None
        self._rx_thread = None
        self._rx_bytes = 0
        self._rx_lock = threading.Lock()
        self._rx_events = []   # (timestamp, delta_bytes, total_bytes)
        self._cmd_agw = None
        self._rsp_agw = None

    def start(self):
        """Launch both Iris instances, connect AGW, set up link."""
        print(f"[BENCH] Iris binary: {self.iris_path}")
        if not os.path.isfile(self.iris_path):
            print(f"[ERROR] Iris binary not found: {self.iris_path}")
            return False

        # Kill any existing iris.exe
        os.system("taskkill /F /IM iris.exe 2>nul >nul")
        time.sleep(1)

        # Each process gets its own console + log file
        self._rsp_log = os.path.abspath("iris_bench_rsp.log")
        self._cmd_log = os.path.abspath("iris_bench_cmd.log")

        # --- Launch responder ---
        rsp_cmd = [
            self.iris_path,
            "--callsign", RSP_CALLSIGN,
            "--capture", str(self.rsp_capture),
            "--playback", str(self.rsp_playback),
            "--agw-port", str(self.rsp_agw_port),
            "--port", str(self.rsp_kiss_port),
            "--log", self._rsp_log,
            *self.extra_args,
        ]
        if not self.use_gui:
            rsp_cmd.append("--nogui")

        print(f"[BENCH] RSP: {' '.join(rsp_cmd)}")
        rsp = subprocess.Popen(rsp_cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)
        self.procs.append(rsp)
        time.sleep(5)  # Wait for WASAPI + AGW server init

        # --- Launch commander (always headless) ---
        cmd_cmd = [
            self.iris_path,
            "--callsign", CMD_CALLSIGN,
            "--capture", str(self.cmd_capture),
            "--playback", str(self.cmd_playback),
            "--agw-port", str(self.cmd_agw_port),
            "--port", str(self.cmd_kiss_port),
            "--log", self._cmd_log,
            "--nogui",
            *self.extra_args,
        ]

        print(f"[BENCH] CMD: {' '.join(cmd_cmd)}")
        cmd = subprocess.Popen(cmd_cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)
        self.procs.append(cmd)
        time.sleep(5)  # Wait for WASAPI + AGW server init

        # --- Connect AGW clients ---
        print("[BENCH] Connecting AGW clients...")
        try:
            self._rsp_agw = tcp_connect_retry(self.rsp_agw_port)
            self._cmd_agw = tcp_connect_retry(self.cmd_agw_port)
        except Exception as e:
            print(f"[ERROR] AGW connection failed: {e}")
            return False

        # Register callsigns
        send_agw_frame(self._rsp_agw, 'X', RSP_CALLSIGN)
        time.sleep(0.5)
        self._drain(self._rsp_agw)

        send_agw_frame(self._cmd_agw, 'X', CMD_CALLSIGN)
        time.sleep(0.5)
        self._drain(self._cmd_agw)

        print("[BENCH] AGW clients connected, callsigns registered.")
        return True

    def connect(self, timeout=60):
        """Initiate AGW connection from CMD to RSP. Wait for CONNECTED."""
        print(f"[BENCH] Connecting {CMD_CALLSIGN} -> {RSP_CALLSIGN}...")
        send_agw_frame(self._cmd_agw, 'C', CMD_CALLSIGN, RSP_CALLSIGN)

        # Wait for 'C' frame (CONNECTED notification) on cmd AGW socket
        start = time.time()
        self._cmd_agw.settimeout(2)
        buf = b''
        while time.time() - start < timeout:
            try:
                data = self._cmd_agw.recv(4096)
                if data:
                    buf += data
                    # Look for CONNECTED in any 'C' frames
                    while len(buf) >= AGW_HEADER_SIZE:
                        hdr = parse_agw_header(buf[:AGW_HEADER_SIZE])
                        if hdr is None:
                            buf = buf[1:]  # skip byte
                            continue
                        frame_len = AGW_HEADER_SIZE + hdr['data_len']
                        if len(buf) < frame_len:
                            break  # need more data
                        frame_data = buf[AGW_HEADER_SIZE:frame_len]
                        buf = buf[frame_len:]
                        if hdr['kind'] == 'C':
                            msg = frame_data.decode('ascii', errors='replace')
                            if 'CONNECTED' in msg:
                                print(f"[BENCH] CONNECTED ({time.time()-start:.1f}s)")
                                return True
            except socket.timeout:
                continue
            if not self.is_alive():
                print("[BENCH] Process died while waiting for CONNECTED")
                return False
        print("[BENCH] Timeout waiting for CONNECTED")
        return False

    def start_data_transfer(self):
        """Start TX thread on CMD and RX thread on RSP."""
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        print("[BENCH] Data transfer started (CMD TX -> RSP RX)")

    def _tx_loop(self):
        """Continuously send data via AGW 'D' frames on CMD."""
        # 256 bytes of repeating pattern per frame
        payload = bytes(range(256))
        while not self.stop_event.is_set():
            try:
                # Check outstanding frames (flow control)
                send_agw_frame(self._cmd_agw, 'Y', CMD_CALLSIGN, RSP_CALLSIGN)
                time.sleep(0.05)

                # Send data frame
                send_agw_frame(self._cmd_agw, 'D', CMD_CALLSIGN, RSP_CALLSIGN, payload)
                time.sleep(0.1)  # Throttle to avoid overwhelming the modem
            except (ConnectionError, OSError):
                break

    def _rx_loop(self):
        """Receive data via AGW 'D' frames on RSP."""
        self._rsp_agw.settimeout(2)
        buf = b''
        while not self.stop_event.is_set():
            try:
                data = self._rsp_agw.recv(4096)
                if not data:
                    break
                buf += data

                # Parse complete AGW frames
                while len(buf) >= AGW_HEADER_SIZE:
                    hdr = parse_agw_header(buf[:AGW_HEADER_SIZE])
                    if hdr is None:
                        buf = buf[1:]
                        continue
                    frame_len = AGW_HEADER_SIZE + hdr['data_len']
                    if len(buf) < frame_len:
                        break
                    frame_data = buf[AGW_HEADER_SIZE:frame_len]
                    buf = buf[frame_len:]

                    if hdr['kind'] == 'D':
                        # Data frame received
                        with self._rx_lock:
                            self._rx_bytes += len(frame_data)
                            self._rx_events.append((time.time(), len(frame_data), self._rx_bytes))
            except socket.timeout:
                continue
            except (ConnectionError, OSError):
                break

    def measure_throughput(self, duration_s):
        """Measure RX bytes over a time window. Returns dict with stats."""
        with self._rx_lock:
            start_bytes = self._rx_bytes
        start_time = time.time()

        # Progress updates every 10s
        remaining = duration_s
        while remaining > 0:
            chunk = min(remaining, 10)
            time.sleep(chunk)
            remaining -= chunk
            with self._rx_lock:
                cur = self._rx_bytes - start_bytes
            elapsed = time.time() - start_time
            bps_so_far = cur * 8 / elapsed if elapsed > 0 else 0
            print(f"    [{elapsed:.0f}s] +{cur} bytes ({bps_so_far:.0f} bps)", flush=True)

        end_time = time.time()
        with self._rx_lock:
            end_bytes = self._rx_bytes

        rx = end_bytes - start_bytes
        dt = end_time - start_time
        bps = rx * 8 / dt if dt > 0 else 0

        return {
            'rx_bytes': rx,
            'duration_s': dt,
            'throughput_bps': bps,
            'bytes_per_min': rx * 60 / dt if dt > 0 else 0,
        }

    def get_rx_bytes(self):
        with self._rx_lock:
            return self._rx_bytes

    def _drain(self, sock, timeout=0.5):
        """Drain pending data from a socket."""
        sock.settimeout(timeout)
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
        except socket.timeout:
            pass

    def _read_log_files(self):
        """Read log files into rsp_lines / cmd_lines."""
        for attr, path_attr in [('rsp_lines', '_rsp_log'), ('cmd_lines', '_cmd_log')]:
            path = getattr(self, path_attr, None)
            if path and os.path.isfile(path):
                try:
                    with open(path, 'r', errors='replace') as f:
                        setattr(self, attr, f.read().splitlines())
                except:
                    pass

    def print_modem_output(self, last_n=20):
        """Print recent modem output lines."""
        self._read_log_files()
        print("\n--- RSP recent output ---")
        for line in self.rsp_lines[-last_n:]:
            print(f"  [RSP] {line}")
        print("\n--- CMD recent output ---")
        for line in self.cmd_lines[-last_n:]:
            print(f"  [CMD] {line}")

    def is_alive(self):
        return all(p.poll() is None for p in self.procs)

    def save_log(self, path):
        """Save full modem output to log file."""
        self._read_log_files()
        with open(path, 'w') as f:
            f.write("=== RSP OUTPUT ===\n")
            for line in self.rsp_lines:
                f.write(f"[RSP] {line}\n")
            f.write("\n=== CMD OUTPUT ===\n")
            for line in self.cmd_lines:
                f.write(f"[CMD] {line}\n")
        print(f"[BENCH] Log saved: {path}")

    def stop(self):
        """Stop everything."""
        self.stop_event.set()
        for s in [self._cmd_agw, self._rsp_agw]:
            if s:
                try:
                    s.close()
                except:
                    pass
        for p in self.procs:
            try:
                p.terminate()
                p.wait(timeout=3)
            except:
                try:
                    p.kill()
                except:
                    pass
        os.system("taskkill /F /IM iris.exe 2>nul >nul")
        time.sleep(0.5)
        # Read log files before cleanup
        self._read_log_files()


def list_audio_devices(iris_path):
    """Run iris --list-audio and print results."""
    result = subprocess.run([iris_path, "--list-audio"],
                          capture_output=True, text=True, timeout=10)
    print(result.stdout)
    if result.stderr:
        print(result.stderr)


def main():
    parser = argparse.ArgumentParser(description="Iris FM Data Modem Benchmark")
    parser.add_argument("--iris", default=IRIS_DEFAULT,
                       help="Path to iris.exe")
    parser.add_argument("--duration", type=int, default=30,
                       help="Measurement duration in seconds (default: 30)")
    parser.add_argument("--gui", action="store_true",
                       help="Show GUI on responder")
    parser.add_argument("--list-audio", action="store_true",
                       help="List audio devices and exit")

    # Audio device overrides
    parser.add_argument("--cmd-capture", type=int, default=DEFAULT_CMD_CAPTURE)
    parser.add_argument("--cmd-playback", type=int, default=DEFAULT_CMD_PLAYBACK)
    parser.add_argument("--rsp-capture", type=int, default=DEFAULT_RSP_CAPTURE)
    parser.add_argument("--rsp-playback", type=int, default=DEFAULT_RSP_PLAYBACK)

    # Port overrides
    parser.add_argument("--cmd-agw-port", type=int, default=CMD_AGW_PORT)
    parser.add_argument("--rsp-agw-port", type=int, default=RSP_AGW_PORT)

    # Modem options
    parser.add_argument("--mode", default=None, help="Channel mode (A, B, C)")
    parser.add_argument("--ax25-baud", type=int, default=None, help="AX.25 baud rate")
    parser.add_argument("--max-mod", default=None, help="Max modulation")
    parser.add_argument("--tx-level", type=float, default=None, help="TX level")
    parser.add_argument("--rx-gain", type=float, default=None, help="RX gain")
    parser.add_argument("--log", action="store_true", help="Save modem logs")

    args = parser.parse_args()

    iris_path = os.path.abspath(args.iris)

    if args.list_audio:
        list_audio_devices(iris_path)
        return

    # Build extra args for both instances
    extra = []
    if args.mode:
        extra += ["--mode", args.mode]
    if args.ax25_baud:
        extra += ["--ax25-baud", str(args.ax25_baud)]
    if args.max_mod:
        extra += ["--max-mod", args.max_mod]
    if args.tx_level is not None:
        extra += ["--tx-level", str(args.tx_level)]
    if args.rx_gain is not None:
        extra += ["--rx-gain", str(args.rx_gain)]

    print("=" * 60)
    print(f"  Iris FM Data Modem — VB-Cable Loopback Benchmark")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Duration: {args.duration}s")
    print(f"  GUI: {'RSP' if args.gui else 'disabled'}")
    print(f"  Audio: CMD cap={args.cmd_capture} play={args.cmd_playback}")
    print(f"         RSP cap={args.rsp_capture} play={args.rsp_playback}")
    print("=" * 60)

    session = IrisSession(
        iris_path=iris_path,
        use_gui=args.gui,
        cmd_capture=args.cmd_capture,
        cmd_playback=args.cmd_playback,
        rsp_capture=args.rsp_capture,
        rsp_playback=args.rsp_playback,
        cmd_agw_port=args.cmd_agw_port,
        rsp_agw_port=args.rsp_agw_port,
        extra_args=extra,
    )

    try:
        # Phase 1: Launch
        if not session.start():
            print("[BENCH] Failed to start. Recent output:")
            session.print_modem_output()
            return 1

        # Phase 2: Connect
        if not session.connect(timeout=60):
            print("[BENCH] Connection failed. Recent output:")
            session.print_modem_output()
            return 1

        # Phase 3: Data transfer
        session.start_data_transfer()

        # Wait a few seconds for data flow to stabilize
        print("[BENCH] Warming up (5s)...")
        time.sleep(5)

        # Phase 4: Measure
        print(f"[BENCH] Measuring throughput ({args.duration}s)...")
        result = session.measure_throughput(args.duration)

        # Phase 5: Results
        print("\n" + "=" * 60)
        print("  RESULTS")
        print("=" * 60)
        print(f"  RX bytes:    {result['rx_bytes']}")
        print(f"  Duration:    {result['duration_s']:.1f}s")
        print(f"  Throughput:  {result['throughput_bps']:.0f} bps")
        print(f"  Bytes/min:   {result['bytes_per_min']:.0f}")
        print("=" * 60)

        if result['rx_bytes'] == 0:
            print("\n[WARN] Zero bytes received. Possible issues:")
            print("  - VB-Cable devices not correctly mapped")
            print("  - Audio routing incorrect")
            print("  - Modem not demodulating")
            session.print_modem_output(30)

        # Save logs if requested
        if args.log:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            session.save_log(f"iris_bench_{ts}.log")

        return 0

    except KeyboardInterrupt:
        print("\n[BENCH] Interrupted by user")
        return 1
    finally:
        session.stop()


if __name__ == "__main__":
    sys.exit(main() or 0)
