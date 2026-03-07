#!/usr/bin/env python3
"""Test iris WASAPI audio output through VB-Cable.

Starts iris.exe in CMD mode (which will TX hail frames on CABLE Input),
then captures from CABLE Output with Python to see if audio arrives.
"""
import sounddevice as sd
import numpy as np
import subprocess, time, os, sys, threading

IRIS = os.path.join(os.path.dirname(__file__), "..", "build", "iris.exe")
CABLE_OUTPUT_SD = 12   # sounddevice index for CABLE Output (capture)
CABLE_INPUT_IRIS = 1005  # iris device ID for CABLE Input (playback)
CABLE_OUTPUT_IRIS = 11   # iris device ID for CABLE Output (capture) - unused here

SR = 48000
DURATION = 8  # seconds to capture

print(f"Capture device: {sd.query_devices(CABLE_OUTPUT_SD)['name']}")
print(f"Starting iris.exe with playback on CABLE Input (iris dev {CABLE_INPUT_IRIS})...")

# Start iris in a mode that will TX - connect to a fake callsign triggers hailing
# We use --nogui and just let it run; the AGW connect will make it TX hail frames
proc = subprocess.Popen([
    os.path.abspath(IRIS),
    "--callsign", "TEST1",
    "--capture", str(CABLE_OUTPUT_IRIS),  # doesn't matter, we're testing TX
    "--playback", str(CABLE_INPUT_IRIS),
    "--agw-port", "8020",
    "--port", "8021",
    "--nogui",
    "--log", "iris_audio_test.log",
], creationflags=subprocess.CREATE_NEW_CONSOLE)

print("Waiting 3s for iris to initialize...")
time.sleep(3)

# Now trigger ARQ connect via AGW to make iris TX hail frames
import socket, struct

def send_agw(sock, kind, call_from="", call_to="", data=b""):
    hdr = bytearray(36)
    hdr[4] = ord(kind)
    hdr[6] = 0xF0
    cf = call_from.encode()[:9]
    hdr[8:8+len(cf)] = cf
    ct = call_to.encode()[:9]
    hdr[18:18+len(ct)] = ct
    struct.pack_into('<I', hdr, 28, len(data))
    sock.sendall(bytes(hdr) + data)

try:
    agw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    agw.connect(("127.0.0.1", 8020))
    send_agw(agw, 'X', "TEST1")
    time.sleep(0.5)
    # Drain
    agw.settimeout(0.5)
    try:
        agw.recv(4096)
    except:
        pass
    # Connect to trigger hailing (TX)
    send_agw(agw, 'C', "TEST1", "TEST2")
    print("ARQ connect sent - iris should start TX hailing...")
except Exception as e:
    print(f"AGW connection failed: {e}")
    proc.terminate()
    sys.exit(1)

# Capture from CABLE Output for DURATION seconds
print(f"Capturing {DURATION}s from CABLE Output...")
recording = sd.rec(int(SR * DURATION), samplerate=SR, channels=1,
                   device=CABLE_OUTPUT_SD, dtype='float32')
sd.wait()

# Analyze
rms_total = np.sqrt(np.mean(recording**2))
peak = np.max(np.abs(recording))

# Check 1-second windows
print(f"\nTotal: RMS={rms_total:.6f}, Peak={peak:.6f}")
for i in range(DURATION):
    chunk = recording[i*SR:(i+1)*SR]
    rms = np.sqrt(np.mean(chunk**2))
    pk = np.max(np.abs(chunk))
    print(f"  Second {i}: RMS={rms:.6f}, Peak={pk:.6f}")

if rms_total > 0.001:
    print("\nIris WASAPI TX -> VB-Cable: WORKS!")
else:
    print("\nIris WASAPI TX -> VB-Cable: FAIL (no signal detected)")

# Cleanup
try:
    agw.close()
except:
    pass
proc.terminate()
try:
    proc.wait(timeout=3)
except:
    proc.kill()
os.system("taskkill /F /IM iris.exe 2>nul >nul")
