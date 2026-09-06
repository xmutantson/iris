#!/usr/bin/env python3
"""
fm_channel_relay.py — Iris-owned FM VOICE-CHANNEL relay for the `-x sim` loopback.

Device-free software channel that sits between two Iris processes launched with
`-x sim`. It is a DIRECT DESCENDANT of Mercury's tools/sim_channel_relay.py: the
TCP server, two-peer role handshake, per-direction Channel + reader/scheduler
threads, conservative-PDES window barrier, and the bare 1024xfloat64 wire are
COPIED UNCHANGED from that proven skeleton (so Iris's audio_sim.cc backend
connects with zero modification — audio_sim.cc:77-80 SIM_CHUNK_SAMPLES=1024,
:136-180 role 'A'/'B' handshake, :320 bare SIM_CHUNK_BYTES=8192 recv). The ONLY
thing replaced is Channel.process(): the HF Watterson/OFDM math is swapped for a
faithful, physically-grounded FM voice-channel model.

    peer-A TX audio  --recv--> relay --+
                                        |  FM voice channel (Channel.process)
    peer-B TX audio  --recv--> relay --+
                                        |
    relay --send--> peer-A RX audio  <-+   (B's TX, channel-impaired)
    relay --send--> peer-B RX audio  <-+   (A's TX, channel-impaired)

Wire units = little-endian float64 mono audio passband @48000 Hz (audio_sim.cc),
bare 8192 B/chunk. One Channel per direction; the ONLY DSP point is process().

=============================================================================
FM VOICE-CHANNEL MODEL  (Channel.process, per relayed chunk, in this order)
=============================================================================
This models the AUDIO-in -> AUDIO-out path of a pair of FM radios (the modem
feeds audio into a TX radio's mic, the far radio's discriminator hands audio
back). It ports iris/source/audio/audio_loopback.cc FmChannelState (~:40-323)
and FIXES the six defects wf4 identified in that implementation:

  TX (mic-side) processing
   1. Pre-emphasis (75 us, single source of truth). H(s)=(1+s*tau1)/(1+s*tau2),
      tau1 = PREEMPH_TAU_US, tau2 = stabilizing pole at 15 kHz. Bilinear.
      [FIX] audio_loopback.cc used tau1=530 us AND the batch test (tests.cc:661)
      used yet another (differencing) pre-emphasis => THREE definitions. Here the
      SINGLE constant PREEMPH_TAU_US (= 75 us, CCITT / broadcast-FM) drives BOTH
      pre-emphasis and de-emphasis; de-emphasis is the exact bilinear 1/(1+s*tau)
      of the same tau (audio_loopback.cc:160-166 form).
   2. Deviation limiter: hard clip the pre-emphasized audio to +/-DEV_LIMIT
      (FM radios limit peak deviation; audio_loopback.cc:243-245).

  RF channel (constant-envelope FM -> effects act on CNR, not audio amplitude)
   3. Fading (profile 'mpg'): a Watterson/Rician complex gain H(t) (Gaussian
      Doppler spectrum, ITU-R F.1487) whose envelope |H(t)| modulates the
      CARRIER-to-NOISE ratio. FM is constant-envelope: above threshold the
      recovered audio amplitude does NOT track the fade; a fade instead lowers
      CNR, which RAISES discriminator noise and click rate.
      [FIX] audio_loopback.cc:247-271 MULTIPLIED the audio by the fade gain
      (`pe *= fade_gain`). That is the AM channel, not FM. Here the fade
      modulates NOISE only (steps 5-6); the audio passes at constant envelope.
   4. CFO + intra-symbol phase noise via ANALYTIC ROTATION (Hilbert -> complex
      -> * e^{j(phi_cfo + jitter)} -> Re()). This is the correct single-sideband
      frequency shift the Iris unit tests already use (tests.cc:1043-1056,
      :3497-3502) and the Mercury relay uses.
      [FIX] audio_loopback.cc:318 did `out = bp * cos(cfo_phase)` — a DSB
      multiply. Multiplying a real passband by cos() creates BOTH sidebands
      (sum AND difference) — it is amplitude modulation, NOT a frequency shift.
      The analytic rotation shifts the spectrum by exactly +cfo with no image.
      [ADD] a bounded, zero-mean per-sample phase JITTER (RMS = PHASE_NOISE_DEG)
      is added to the rotation phase. This is oscillator/discriminator phase
      noise. Being an irreducible per-sample phase error, it puts a genuine
      ICI/EVM FLOOR at high S:N (EVM_floor_dB ~ -20*log10(sigma_rad)), which is
      the O2+ high-S:N wall that a noiseless thin sim cannot reproduce (and is
      why the current sim could not reproduce the data-decode failure).

  RX (discriminator + audio) processing
   5. FM discriminator noise: white noise -> first-difference DIFFERENTIATOR
      (|H|^2 = 4 sin^2(pi f/fs) ~ (2 pi f/fs)^2) => the f^2 ("triangular") noise
      PSD characteristic of an FM discriminator above threshold (Carlson,
      "Communication Systems"; Haykin, "Communication Systems"). Injected BEFORE
      de-emphasis (audio_loopback.cc:273-279, tests.cc:3506-3522), so the output
      noise PSD is f^2/(1+(2 pi f tau)^2). The noise AMPLITUDE is CNR-CALIBRATED
      (see "S:N convention" below) and per-sample scaled by 1/|H(t)| so a deep
      fade raises the noise (step 3).
   6. FM threshold / click knee: below the FM capture threshold the discriminator
      emits impulsive "clicks" (2 pi phase slips). Modeled with Rice's click rate
      N ~ (B_if/sqrt(3)) * erfc(sqrt(CNR)) (Rice 1948, "Statistical Properties of
      a Sine Wave plus Random Noise"; Taub & Schilling, "Principles of
      Communication Systems", FM threshold). Clicks are negligible for CNR>>1 and
      rise STEEPLY below ~10 dB CNR — the "knee" absent from the current sim.
      Injected with the discriminator noise (before de-emphasis) so they are
      band-shaped like real clicks. The instantaneous CNR(t)=CNR_nom*|H(t)|^2, so
      fades drive the click rate (step 3).
   7. De-emphasis (75 us, same tau as step 1) then audio bandpass (300-3000 Hz).

  (orthogonal knob) Optional Gilbert-Elliott impulse erasure (--burst/--loss):
   independent lightning/key-click dropout, off by default. NOT the fade proxy.

=============================================================================
S:N CONVENTION  (apples-to-apples with iris/tools/vara_fm_bar_2025.json)
=============================================================================
The VARA FM sheet (vara_fm_bar_2025.json) specifies each cell as WGN or MPG at
"S:N (dB)" in a 3000 Hz bandwidth (the FM voice channel). We adopt EXACTLY that:

   S:N (dB) = 10*log10( P_signal / P_noise )

both powers measured IN THE 3 kHz AUDIO PASSBAND at the modem's input (the relay
output), where:
  * P_signal = the mean-square of the delivered (clean) audio in the passband.
    Measured live from the TX stream with a sticky peak-hold (RF realism: band
    noise does not vanish in silent inter-frame gaps).
  * P_noise  = the TOTAL integrated power of the injected discriminator noise
    over the same 3 kHz passband.

Because the injected noise is f^2-shaped (not flat), we calibrate its TOTAL
in-band power — not a per-Hz density — to P_signal/10^(SN/10). Concretely, the
in-band power GAINS of the signal chain (pre->de->bandpass) and the noise chain
(differentiator->de->bandpass) are computed once from the closed-form filter
frequency responses (G_sig, G_noise), and the white-noise stddev is set to
   sigma = sqrt( P_sig * G_sig / (SNR_lin * G_noise) )
so that E[P_noise_out]/E[P_signal_out] = 1/SNR_lin EXACTLY, regardless of the
f^2 spectral shape. This is what makes the axis apples-to-apples with the sheet:
the sheet's "S:N in 3000 Hz" IS total-signal-power / total-noise-power in the
voice channel, and so is ours. On a fading cell the S:N is the MEAN channel S:N
(E[|H|^2]=1 by construction), matching how the sheet's MPG cells are specified.

NOTE (vs Mercury's relay): Mercury mapped a testbed WGN LABEL to SNR3k with a
+2.4 dB offset (an HF-bench artifact). Iris compares DIRECTLY to the VARA FM
sheet's S:N, so --cell WGN:40 / MPG:20 use the sheet number with NO offset.

=============================================================================
Usage
=============================================================================
    python iris/tools/fm_channel_relay.py --port 52100 --snr 20 --profile mpg
        [--cell WGN:40 | MPG:20]      # convenience: sets --snr and --profile
        [--cfo-hz 0.0] [--phase-noise-deg 0.5]
        [--fade-doppler-hz 2.0] [--rician-k-db 6.0]   # mpg overrides
        [--click-gain 3.0] [--click-bw 3000] [--no-clicks]
        [--dev-limit 0.95] [--preemph-us 75]
        [--seed 1] [--log relay.log]

Then launch two Iris `-x sim` processes with MERCURY_SIM_ROLE A / B and the same
MERCURY_SIM_PORT (audio_sim.cc:136-138). Cells line up 1:1 with the WGN/MPG @
S:N entries of vara_fm_bar_2025.json (narrow_3000Hz).

Self-check:  python iris/tools/fm_channel_relay.py --selfcheck
"""

import argparse
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time

import numpy as np

# --- wire contract: IDENTICAL to Mercury's relay / Iris audio_sim.cc ----------
CHUNK_SAMPLES = 1024          # MUST match SIM_CHUNK_SAMPLES (audio_sim.cc:79)
CHUNK_BYTES = CHUNK_SAMPLES * 8
FS = 48000.0                  # passband wire sample rate (audio_sim.cc)
F_NYQUIST = FS / 2.0          # 24000 Hz

# --- FM voice-channel constants ----------------------------------------------
BP_LOW_HZ = 300.0             # FM voice / Iris audio passband low edge
BP_HIGH_HZ = 3000.0           # ... high edge (the 3 kHz S:N reference band)
PREEMPH_TAU_US = 75.0         # SINGLE SOURCE OF TRUTH: 75 us (CCITT / broadcast
                              # FM North America; ITU-R BS.450). Drives BOTH
                              # pre-emphasis (TX) and de-emphasis (RX).
PREEMPH_STAB_HZ = 15000.0     # pre-emphasis stabilizing pole (audio_loopback:147)
DEV_LIMIT = 0.95              # deviation limiter clip (audio_loopback:56)
PHASE_NOISE_DEG = 0.5         # per-sample oscillator/discriminator phase jitter
                              # RMS (deg). Sets the high-S:N ICI/EVM floor.
FADE_DOPPLER_HZ = 2.0         # 'mpg' Doppler spread (Gaussian PSD), mild VHF
RICIAN_K_DB = 6.0             # 'mpg' Rician K-factor (specular/scatter, dB):
                              # "good" multipath = strong LOS, shallow fades
CLICK_GAIN = 3.0             # FM click amplitude (x sqrt(P_sig); a 2pi spike)
CLICK_BW = 3000.0             # FM IF/click bandwidth for Rice click rate (Hz)

# Channel PROFILES for the VARA FM sheet (vara_fm_bar_2025.json):
#   'wgn' = flat AWGN, NO multipath (pure discriminator-noise reference)
#   'mpg' = multipath-good, Watterson/Rician (ITU-R F.1487, VHF-mild)
PROFILES = {
    "wgn": None,
    "mpg": {"fd": FADE_DOPPLER_HZ, "k_db": RICIAN_K_DB},
}


# ---------------------------------------------------------------------------
# Deterministic Gaussian source (per-direction RNG, reproducible).
# COPIED VERBATIM from tools/sim_channel_relay.py:152-183.
# ---------------------------------------------------------------------------
class Xoshiro:
    """Deterministic Gaussian source (Box-Muller on a splitmix64 stream)."""
    def __init__(self, seed):
        self.s = (seed * 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        self._spare = None

    def _u64(self):
        self.s = (self.s + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = self.s
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        return (z ^ (z >> 31)) & 0xFFFFFFFFFFFFFFFF

    def uniform(self):
        return (self._u64() >> 11) * (1.0 / 9007199254740992.0)

    def gauss(self):
        if self._spare is not None:
            v = self._spare
            self._spare = None
            return v
        u1 = self.uniform()
        if u1 < 1e-15:
            u1 = 1e-15
        u2 = self.uniform()
        mag = math.sqrt(-2.0 * math.log(u1))
        self._spare = mag * math.sin(2.0 * math.pi * u2)
        return mag * math.cos(2.0 * math.pi * u2)

    def seed_np(self):
        """Spin off a numpy Generator seeded from this stream (independent)."""
        return np.random.default_rng(self._u64())


# ---------------------------------------------------------------------------
# Hilbert analytic-signal FIR (causal, continuous across chunks).
# COPIED VERBATIM from tools/sim_channel_relay.py:189-236 — the analytic
# converter used for the CFO/phase-noise rotation (the wf4 CFO fix).
# ---------------------------------------------------------------------------
def _hilbert_fir(numtaps):
    """Type-III FIR Hilbert transformer (odd length, antisymmetric)."""
    if numtaps % 2 == 0:
        numtaps += 1
    m = (numtaps - 1) // 2
    n = np.arange(numtaps) - m
    h = np.zeros(numtaps)
    odd = n % 2 != 0
    h[odd] = 2.0 / (math.pi * n[odd])
    win = np.hanning(numtaps)
    return h * win, m


class AnalyticFilter:
    """Streaming real->analytic converter. Maintains FIR history so output is
    continuous across chunks (no per-chunk edge transients)."""
    def __init__(self, numtaps=129):
        self.h, self.delay = _hilbert_fir(numtaps)
        self.ntaps = len(self.h)
        self.hist = np.zeros(self.ntaps - 1)

    def process(self, x):
        buf = np.concatenate((self.hist, x))
        q = np.convolve(buf, self.h, mode="full")
        start = self.ntaps - 1
        q_chunk = q[start:start + len(x)]
        idx_end = len(buf) - self.delay
        idx_start = idx_end - len(x)
        if idx_start < 0:
            pad = -idx_start
            i_chunk = np.concatenate((np.zeros(pad), buf[0:idx_end]))[:len(x)]
        else:
            i_chunk = buf[idx_start:idx_end]
        self.hist = buf[-(self.ntaps - 1):]
        return i_chunk + 1j * q_chunk


# ---------------------------------------------------------------------------
# IONOS firmware 128-tap Gaussian Doppler FIR (ARSFI HFSim_BFD_2_03.ino.src).
# COPIED VERBATIM from tools/sim_channel_relay.py:247-280. Used to give the
# 'mpg' fade its Gaussian Doppler power spectrum (Watterson, ITU-R F.1487).
# ---------------------------------------------------------------------------
GAUS_FIR_COEFFS = np.array([
    1.1755592671332046e-11, 2.0188004956137427e-10, 1.7236333623946176e-09, 9.815423109243151e-09,
    4.219820040519088e-08, 1.4693429486234634e-07, 4.338503956649552e-07, 1.122118806000393e-06,
    2.604091536729611e-06, 5.522713327023963e-06, 1.0857390465642334e-05, 2.0011412649247494e-05,
    3.4893068706162795e-05, 5.7982477259392286e-05, 9.237678934582388e-05, 0.00014180767284007714,
    0.00021062669000635658, 0.0003037561533907518, 0.0004266051229102419, 0.000584952237938201,
    0.0007847989367901134, 0.001032198204838678, 0.001333065243166745, 0.001692977321877409,
    0.002116970561258896, 0.002609341477645015, 0.003173460865247882, 0.00381160700116864,
    0.004524824309342139, 0.005312812558055807, 0.006173850455688009, 0.007104756211217515,
    0.008100886298035966, 0.00915617235512072, 0.010263194925878348, 0.01141329161173599,
    0.012596696236577472, 0.013802704802828978, 0.015019863385625786, 0.016236172665450826,
    0.01743930354214716, 0.01861681819809535, 0.019756391073966928, 0.020846024470695095,
    0.021874253876569306, 0.02283033861665188, 0.023704434009553566, 0.02448774186995001,
    0.02517263689027796, 0.025752767148979578, 0.026223127704178725, 0.026580106921515002,
    0.02682150583616039, 0.026946531447567135, 0.026955765379786157, 0.02685110980161695,
    0.026635712883536795, 0.026313876369100774, 0.025890948056565766, 0.025373202123371387,
    0.024767710285281262, 0.024082206768594926, 0.02332494999439922, 0.022504583735910823,
    0.02163000032189053, 0.020710208229666707, 0.019754206149457228, 0.018770865316331563,
    0.01776882160593159, 0.016756378583127243, 0.01574142238665159, 0.01473134903422507,
    0.013733004447687326, 0.012752637231260112, 0.011795863992392318, 0.010867646776884902,
    0.009972282000446704, 0.009113400098909145, 0.008293974989634013, 0.007516342337046732,
    0.006782225544921226, 0.006092768355660706, 0.005448572920512081, 0.004849742212182516,
    0.004295925680163602, 0.003786367096475506, 0.003319953602663025, 0.002895265044807468,
    0.002510622769190125, 0.002164137144270543, 0.0018537531721837, 0.001577293652557479,
    0.001332499460868328, 0.001117066600796574, 0.0009286797833839573, 0.0007650423737761393,
    0.0006239026277671727, 0.0005030762143350815, 0.0004004650862100223, 0.0003140728178353742,
    0.00024201657867910556, 0.00018253594974316996, 0.00013399882249807025, 9.49046426887381e-05,
    6.388527699708468e-05, 3.970378899074398e-05, 2.1251412801076355e-05, 7.5430092767428655e-06,
    -2.2887192936932196e-06, -8.999992637884094e-06, -1.3243447148502327e-05, -1.557613368708468e-05,
    -1.6467811426850445e-05, -1.63093528492861e-05, -1.5421097939455242e-05, -1.4061018704922785e-05,
    -1.243257788819571e-05, -1.0692187735418308e-05, -8.956195575428226e-06, -7.3073424710351094e-06,
    -5.8006591112396305e-06, -4.468779263172823e-06, -3.3266653970160216e-06, -2.3757534892377295e-06,
    -1.6075344987453848e-06, -1.0065986371058506e-06, -5.531753923550552e-07, -2.252074190014706e-07,
], dtype=np.float64)

_FIR_SUMSQ = float(np.sum(GAUS_FIR_COEFFS * GAUS_FIR_COEFFS))
_FIR_NH = GAUS_FIR_COEFFS.size


class DopplerTap:
    """Gaussian-Doppler complex tap (Watterson). COPIED VERBATIM from
    tools/sim_channel_relay.py:306-384. Produces a unit-power complex Gaussian
    process with a Gaussian-shaped Doppler power spectrum (fresh innovation
    every FS/(64*fd) samples, pushed through GAUS_FIR_COEFFS, zero-order held)."""
    def __init__(self, fd_hz, rng_np, update=None):
        self.fd = fd_hz
        self.rng = rng_np
        if update is None:
            if fd_hz > 0:
                update = max(1, int(round(FS / (fd_hz * 64.0))))
            else:
                update = CHUNK_SAMPLES
        self.update = update
        if fd_hz > 0:
            self.inno_std = math.sqrt(0.5 / _FIR_SUMSQ)
        else:
            self.inno_std = 0.0
        self.fir_i = np.zeros(_FIR_NH, dtype=np.float64)
        self.fir_q = np.zeros(_FIR_NH, dtype=np.float64)
        if fd_hz > 0:
            for _ in range(_FIR_NH):
                self._fir_update()
            self.g_hold = self._fir_output()
        else:
            self.g_hold = self._static_white()
        self.pos = 0

    def _static_white(self):
        r = self.rng.standard_normal()
        i = self.rng.standard_normal()
        return (r + 1j * i) / math.sqrt(2.0)

    def _fir_update(self):
        self.fir_i = np.roll(self.fir_i, 1)
        self.fir_q = np.roll(self.fir_q, 1)
        self.fir_i[0] = self.rng.standard_normal() * self.inno_std
        self.fir_q[0] = self.rng.standard_normal() * self.inno_std

    def _fir_output(self):
        gi = float(np.dot(GAUS_FIR_COEFFS, self.fir_i))
        gq = float(np.dot(GAUS_FIR_COEFFS, self.fir_q))
        return gi + 1j * gq

    def advance(self, n):
        out = np.empty(n, dtype=np.complex128)
        k = 0
        while k < n:
            if self.fd <= 0:
                out[k:] = self.g_hold
                self.pos = (self.pos + (n - k)) % self.update
                break
            span = self.update - self.pos
            take = min(span, n - k)
            out[k:k + take] = self.g_hold
            k += take
            self.pos += take
            if self.pos >= self.update:
                self.pos = 0
                self._fir_update()
                self.g_hold = self._fir_output()
        return out


# ---------------------------------------------------------------------------
# Streaming IIR filter primitives (state carried across chunks).
# ---------------------------------------------------------------------------
class OnePole:
    """First-order IIR: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]."""
    def __init__(self, b0, b1, a1):
        self.b0, self.b1, self.a1 = b0, b1, a1
        self.x1 = 0.0
        self.y1 = 0.0

    def process(self, x):
        out = np.empty_like(x)
        b0, b1, a1 = self.b0, self.b1, self.a1
        x1, y1 = self.x1, self.y1
        for i in range(x.size):
            xi = x[i]
            yi = b0 * xi + b1 * x1 - a1 * y1
            x1 = xi
            y1 = yi
            out[i] = yi
        self.x1, self.y1 = x1, y1
        return out


class Biquad:
    """Direct-Form-II biquad (matches audio_loopback.cc:73-78)."""
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0, self.b1, self.b2 = b0, b1, b2
        self.a1, self.a2 = a1, a2
        self.z1 = 0.0
        self.z2 = 0.0

    def process(self, x):
        out = np.empty_like(x)
        b0, b1, b2, a1, a2 = self.b0, self.b1, self.b2, self.a1, self.a2
        z1, z2 = self.z1, self.z2
        for i in range(x.size):
            w = x[i] - a1 * z1 - a2 * z2
            out[i] = b0 * w + b1 * z1 + b2 * z2
            z2 = z1
            z1 = w
        self.z1, self.z2 = z1, z2
        return out


# ---- filter coefficient design (SINGLE source of truth = tau) ---------------
def _preemph_coeffs(tau_us, fs):
    """Pre-emphasis shelf H(s)=(1+s*tau1)/(1+s*tau2), bilinear
    (audio_loopback.cc:149-156). Returns (b0, b1, a1)."""
    tau1 = tau_us * 1e-6
    tau2 = 1.0 / (2.0 * math.pi * PREEMPH_STAB_HZ)
    c1 = 2.0 * fs * tau1
    c2 = 2.0 * fs * tau2
    a0 = 1.0 + c2
    return (1.0 + c1) / a0, (1.0 - c1) / a0, (1.0 - c2) / a0


def _deemph_coeffs(tau_us, fs):
    """De-emphasis H(s)=1/(1+s*tau), bilinear (audio_loopback.cc:160-166); the
    SAME tau as pre-emphasis. Returns (b0, b1, a1)."""
    tau = tau_us * 1e-6
    wc = 1.0 / tau
    K = 2.0 * fs
    a = K + wc
    return wc / a, wc / a, (wc - K) / a


def _biquad_hp(f0, fs):
    """2nd-order Butterworth highpass biquad (audio_loopback.cc:176-187)."""
    w0 = 2.0 * math.pi * f0 / fs
    c, s = math.cos(w0), math.sin(w0)
    alpha = s / (2.0 * 0.7071)
    a0 = 1.0 + alpha
    return (((1.0 + c) / 2.0) / a0, -(1.0 + c) / a0, ((1.0 + c) / 2.0) / a0,
            (-2.0 * c) / a0, (1.0 - alpha) / a0)


def _biquad_lp(f0, fs):
    """2nd-order Butterworth lowpass biquad (audio_loopback.cc:190-201)."""
    w0 = 2.0 * math.pi * f0 / fs
    c, s = math.cos(w0), math.sin(w0)
    alpha = s / (2.0 * 0.7071)
    a0 = 1.0 + alpha
    return (((1.0 - c) / 2.0) / a0, (1.0 - c) / a0, ((1.0 - c) / 2.0) / a0,
            (-2.0 * c) / a0, (1.0 - alpha) / a0)


def _freq_resp(b, a, w):
    """Complex H(e^{jw}) for filter coeffs b (numerator), a (denominator)."""
    z1 = np.exp(-1j * w)
    num = np.zeros_like(w, dtype=np.complex128)
    for k, bk in enumerate(b):
        num = num + bk * (z1 ** k)
    den = np.zeros_like(w, dtype=np.complex128)
    for k, ak in enumerate(a):
        den = den + ak * (z1 ** k)
    return num / den


# ---------------------------------------------------------------------------
# Per-direction FM voice channel.
# ---------------------------------------------------------------------------
class Channel:
    """Per-direction FM voice-channel state (independent noise/fade per link).

    Constructor signature (args, rng_seed) is UNCHANGED from the skeleton so the
    main() barrier/scheduler/wire code below is reused verbatim."""

    def __init__(self, args, rng_seed):
        self.snr_db = float(args.snr)          # S:N (dB) in 3 kHz (see header)
        self.profile = args.profile
        self.cfo_hz = float(args.cfo_hz)
        self.loss = float(args.loss)
        self.burst = bool(args.burst)
        self.dev_limit = float(getattr(args, "dev_limit", DEV_LIMIT))
        self.clicks_on = not bool(getattr(args, "no_clicks", False))
        self.click_gain = float(getattr(args, "click_gain", CLICK_GAIN))
        self.click_bw = float(getattr(args, "click_bw", CLICK_BW))
        preemph_us = float(getattr(args, "preemph_us", PREEMPH_TAU_US))

        self.snr_lin = 10.0 ** (self.snr_db / 10.0)

        self.rng = Xoshiro(rng_seed)           # Xoshiro for burst/impulse
        np_rng = self.rng.seed_np()            # numpy RNG for fade/noise/clicks
        self.np_noise = self.rng.seed_np()     # dedicated noise RNG
        self.np_click = self.rng.seed_np()     # dedicated click RNG

        # ---- filter coefficients (single tau) -------------------------------
        pb0, pb1, pa1 = _preemph_coeffs(preemph_us, FS)
        db0, db1, da1 = _deemph_coeffs(preemph_us, FS)
        hp = _biquad_hp(BP_LOW_HZ, FS)
        lp = _biquad_lp(BP_HIGH_HZ, FS)
        self.preemph = OnePole(pb0, pb1, pa1)
        self.deemph = OnePole(db0, db1, da1)
        self.bp_hi = Biquad(*hp)
        self.bp_lo = Biquad(*lp)

        # ---- power gains for the CNR / S:N calibration ----------------------
        # Closed-form filter frequency responses on a uniform w in [0, pi]
        # (maps to f in [0, F_NYQUIST]). White input has a FLAT PSD, so the mean
        # of |H|^2 over a uniform grid IS the output power per unit input
        # variance. Two gains, measured over DIFFERENT supports because the two
        # inputs have different spectra:
        #   * g_sig  = BAND-mean of |Hpre*Hde*Hbp|^2. The modem's audio is
        #     already band-limited (300-3000 Hz OFDM), so its output power is the
        #     in-band average gain times P_sig.
        #   * g_noise = FULL-RANGE mean of |Hdiff*Hde*Hbp|^2. The injected
        #     discriminator noise is generated FULL-BAND white; the bandpass in
        #     the chain is what limits it, so the output noise power is the
        #     full-range average gain times sigma^2. (Averaging g_noise over the
        #     passband only over-counts by ~1/band_fraction and under-injects
        #     noise by ~9 dB — verified via the measured-S:N test.)
        w = np.linspace(1e-4, math.pi, 8192)
        f = w * FS / (2.0 * math.pi)
        band = (f >= BP_LOW_HZ) & (f <= BP_HIGH_HZ)
        Hpre = _freq_resp([pb0, pb1], [1.0, pa1], w)
        Hde = _freq_resp([db0, db1], [1.0, da1], w)
        Hhp = _freq_resp([hp[0], hp[1], hp[2]], [1.0, hp[3], hp[4]], w)
        Hlp = _freq_resp([lp[0], lp[1], lp[2]], [1.0, lp[3], lp[4]], w)
        Hdiff = _freq_resp([1.0, -1.0], [1.0], w)              # differentiator
        Hbp = Hhp * Hlp
        sig_mag2 = np.abs(Hpre * Hde * Hbp) ** 2
        noise_mag2 = np.abs(Hdiff * Hde * Hbp) ** 2
        self.g_sig = float(np.mean(sig_mag2[band]))            # band-limited input
        self.g_noise = float(np.mean(noise_mag2))              # full-band white input
        if self.g_noise <= 0.0:
            self.g_noise = 1e-12

        # ---- live signal-power (P_sig) + noise stddev (sigma_base) ----------
        # P_sig tracks the running MEAN power over ACTIVE chunks (see process()),
        # seeded from sig_ref until the first active audio arrives.
        self.p_sig = max(float(getattr(args, "sig_ref", 0.15)), 1e-6) ** 2
        self.active_ms_sum = 0.0
        self.active_ms_count = 0
        self.silence_ms_floor = 1e-8          # chunk mean-square below this = silence
        self.peak_ms = 0.0                     # diagnostic only (no longer gates P_sig)
        self.peak_diag = 0.0
        self.sigma_base = self._sigma_from_psig(self.p_sig)

        # ---- fading (profile) ----------------------------------------------
        prof = PROFILES.get(self.profile)
        self.fading = prof is not None
        if self.fading:
            fd = float(getattr(args, "fade_doppler_hz", prof["fd"]))
            k_db = float(getattr(args, "rician_k_db", prof["k_db"]))
            k_lin = 10.0 ** (k_db / 10.0)
            # Rician unit-power gain: H = a_los + a_sca*g(t), g unit-power complex.
            #   E[|H|^2] = a_los^2 + a_sca^2 = 1  (a_los^2 = K/(K+1)).
            self.a_los = math.sqrt(k_lin / (k_lin + 1.0))
            self.a_sca = math.sqrt(1.0 / (k_lin + 1.0))
            self.fade_tap = DopplerTap(fd, np_rng)
        else:
            self.fade_tap = None

        # ---- CFO + intra-symbol phase noise (analytic rotation) -------------
        self.analytic = AnalyticFilter(numtaps=129)
        self.phase = 0.0
        self.cfo_w = 2.0 * math.pi * self.cfo_hz / FS
        self.pn_std = math.radians(float(args.phase_noise_deg))
        self.np_pn = np_rng

        # ---- discriminator noise differentiator state -----------------------
        self.noise_prev = 0.0

        # ---- Gilbert-Elliott impulse (orthogonal knob) ----------------------
        self.ge_state = 0
        self.sample_clock = 0

    def _sigma_from_psig(self, p_sig):
        """White-noise stddev so the TOTAL in-band output noise power equals
        P_sig/SNR_lin (CNR/S:N calibration; see header 'S:N convention')."""
        target_noise_power = p_sig * self.g_sig / self.snr_lin
        return math.sqrt(max(target_noise_power / self.g_noise, 0.0))

    def process(self, samples):
        x = np.asarray(samples, dtype=np.float64)
        n = x.size
        if n == 0:
            return []

        # P_sig = running MEAN of per-chunk power over ACTIVE (non-silent) chunks.
        # (Peak-hold was PAPR-pessimistic and drifted downward with session length;
        # the VARA sheet S:N is a mean-power ratio.) Silent inter-frame gaps (idle
        # emits exact zeros) are SKIPPED so the calibrated noise floor persists
        # through gaps (RF realism). A cumulative active-mean is drift-free.
        amax = float(np.max(np.abs(x)))
        if amax > self.peak_diag:
            self.peak_diag = amax
        ms = float(np.mean(x * x))
        if ms > self.peak_ms:
            self.peak_ms = ms                 # diagnostic peak only
        if ms > self.silence_ms_floor:        # active chunk -> update mean P_sig
            self.active_ms_sum += ms
            self.active_ms_count += 1
            new_p_sig = self.active_ms_sum / self.active_ms_count
            if new_p_sig != self.p_sig:
                self.p_sig = new_p_sig
                self.sigma_base = self._sigma_from_psig(self.p_sig)

        # --- 1. TX pre-emphasis (75 us) --------------------------------------
        pe = self.preemph.process(x)

        # --- 2. deviation limiter (hard clip) --------------------------------
        np.clip(pe, -self.dev_limit, self.dev_limit, out=pe)

        # --- 3. RF fading envelope |H(t)| (modulates CNR, NOT the audio) -----
        if self.fading:
            g = self.fade_tap.advance(n)                 # unit-power complex
            H = self.a_los + self.a_sca * g
            env = np.abs(H)
            np.maximum(env, 1e-3, out=env)               # floor: avoid /0
        else:
            env = None                                   # env == 1 everywhere

        # --- 4. CFO + intra-symbol phase noise via ANALYTIC ROTATION --------
        # (Hilbert -> complex -> * e^{j(cfo_ramp + jitter)} -> Re) : the correct
        # single-sideband frequency shift + an irreducible per-sample phase
        # jitter that sets the high-S:N ICI/EVM floor. FIXES the DSB bp*cos()
        # of audio_loopback.cc:318.
        if self.cfo_hz != 0.0 or self.pn_std > 0.0:
            z = self.analytic.process(pe)
            idx = np.arange(n)
            cfo_ph = self.phase + self.cfo_w * (idx + 1)
            if self.pn_std > 0.0:
                jitter = self.np_pn.standard_normal(n) * self.pn_std
            else:
                jitter = 0.0
            z = z * np.exp(1j * (cfo_ph + jitter))
            disc = np.real(z)
            self.phase = float(cfo_ph[-1]) % (2.0 * math.pi)
        else:
            disc = pe

        # --- 5. FM discriminator noise: f^2-shaped, CNR-calibrated ----------
        # white -> first-difference differentiator (f^2 PSD), scaled by 1/|H(t)|
        # so a fade RAISES the noise (fade modulates NOISE, wf4 fix).
        if self.sigma_base > 0.0:
            wn = self.np_noise.standard_normal(n) * self.sigma_base
            if env is not None:
                wn = wn / env                            # fade raises noise
            shaped = np.empty(n, dtype=np.float64)
            prev = self.noise_prev
            # first difference y[k] = wn[k] - wn[k-1]  (carried across chunks)
            shaped[0] = wn[0] - prev
            shaped[1:] = wn[1:] - wn[:-1]
            self.noise_prev = float(wn[-1])
            disc = disc + shaped

        # --- 6. FM threshold / click knee (Rice erfc rate) ------------------
        if self.clicks_on and self.p_sig > 0.0:
            # instantaneous CNR(t) = SNR_lin * |H(t)|^2 (fade drives clicks).
            if env is not None:
                cnr = self.snr_lin * (env * env)
            else:
                cnr = np.full(n, self.snr_lin)
            # per-sample click probability = (B_if/sqrt(3))*erfc(sqrt(CNR))/fs.
            rate_scale = (self.click_bw / math.sqrt(3.0)) / FS
            # erfc(sqrt(cnr)); vectorized via math.erfc (cnr>=0).
            p_click = np.fromiter(
                (rate_scale * math.erfc(math.sqrt(c)) for c in cnr),
                dtype=np.float64, count=n)
            hits = self.np_click.random(n) < p_click
            n_hits = int(np.count_nonzero(hits))
            if n_hits:
                # each click ~ a 2pi discriminator phase-slip impulse; sign random,
                # amplitude ~ click_gain * sqrt(P_sig). Injected pre-de-emphasis.
                amp = self.click_gain * math.sqrt(max(self.p_sig, 0.0))
                signs = np.where(self.np_click.random(n_hits) < 0.5, -1.0, 1.0)
                disc[hits] = disc[hits] + amp * signs

        # --- 7. RX de-emphasis (75 us) then audio bandpass (300-3000) -------
        de = self.deemph.process(disc)
        out = self.bp_hi.process(de)
        out = self.bp_lo.process(out)

        # --- (orthogonal) Gilbert-Elliott impulse erasure -------------------
        if self.burst and self.loss > 0.0:
            p_g2b = self.loss * 0.05
            p_b2g = 0.03
            for i in range(n):
                if self.ge_state == 0:
                    if self.rng.uniform() < p_g2b:
                        self.ge_state = 1
                else:
                    out[i] = 0.0
                    if self.rng.uniform() < p_b2g:
                        self.ge_state = 0
        elif self.loss > 0.0:
            for i in range(n):
                if self.rng.uniform() < self.loss:
                    out[i] = 0.0

        self.sample_clock += n
        return out.tolist()


# ---------------------------------------------------------------------------
# Peer / socket helpers — COPIED VERBATIM from tools/sim_channel_relay.py.
# ---------------------------------------------------------------------------
class Peer:
    def __init__(self, sock, role):
        self.sock = sock
        self.role = role
        self.alive = True


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except OSError:
            return None
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def send_all(sock, data):
    try:
        sock.sendall(data)
        return True
    except OSError:
        return False


def parse_cell(cell):
    """--cell WGN:40 / MPG:20  ->  (snr_db, profile). Bare number -> ('wgn').
    Iris compares DIRECTLY to the VARA FM sheet S:N (no +2.4 offset)."""
    s = cell.strip().upper()
    if ":" in s:
        pfx, val = s.split(":", 1)
        pfx = pfx.strip()
        prof = "mpg" if pfx == "MPG" else "wgn"
        return float(val), prof
    return float(s), None


# ---------------------------------------------------------------------------
# main() — TCP server, handshake, per-direction Channel, conservative-PDES
# barrier, bare 1024xfloat64 wire. COPIED from tools/sim_channel_relay.py:
# only the argparse block, --cell handling, and the profile log line are
# FM-specific; the scheduler/barrier/reader/wire/airtime are UNCHANGED.
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Iris FM voice-channel relay (analytic CFO + f^2 discriminator "
                    "noise + FM threshold clicks + Watterson mpg fade)")
    ap.add_argument("--port", type=int, default=52100)
    ap.add_argument("--snr", type=float, default=20.0,
                    help="channel S:N in dB in a 3 kHz BW (VARA FM sheet basis). "
                         "Overridden by --cell.")
    ap.add_argument("--cell", default=None,
                    help="convenience spec, e.g. WGN:40 or MPG:20 (sets --snr "
                         "and --profile from the VARA FM sheet cell; no offset)")
    ap.add_argument("--profile", choices=list(PROFILES.keys()), default="wgn",
                    help="fading profile: wgn (flat AWGN) or mpg (multipath-good "
                         "Watterson/Rician)")
    ap.add_argument("--sig-ref", type=float, default=0.15,
                    help="initial TX passband RMS reference for the noise floor "
                         "before live TX power is measured (default 0.15)")
    ap.add_argument("--cfo-hz", type=float, default=0.0,
                    help="audio-passband carrier/clock frequency offset in Hz "
                         "(default 0). Applied via analytic rotation. Reverse "
                         "direction gets the opposite sign (--cfo-reverse-sign).")
    ap.add_argument("--cfo-reverse-sign", type=int, default=-1, choices=(-1, 1),
                    help="sign of the CFO applied to the b2a direction relative to "
                         "--cfo-hz (default -1: physically reciprocal). No effect "
                         "when --cfo-hz 0.")
    ap.add_argument("--phase-noise-deg", type=float, default=PHASE_NOISE_DEG,
                    help="per-sample oscillator/discriminator phase-noise stddev "
                         "in degrees (default %(default)s). Sets the high-S:N "
                         "ICI/EVM floor.")
    ap.add_argument("--fade-doppler-hz", type=float, default=FADE_DOPPLER_HZ,
                    help="mpg Doppler spread in Hz (Gaussian PSD). Ignored for wgn.")
    ap.add_argument("--rician-k-db", type=float, default=RICIAN_K_DB,
                    help="mpg Rician K-factor in dB (specular/scatter power ratio; "
                         "higher = shallower 'good' fades). Ignored for wgn.")
    ap.add_argument("--click-gain", type=float, default=CLICK_GAIN,
                    help="FM click amplitude (x sqrt(P_sig)); a 2pi phase-slip spike")
    ap.add_argument("--click-bw", type=float, default=CLICK_BW,
                    help="FM IF/click bandwidth (Hz) for the Rice click rate")
    ap.add_argument("--no-clicks", action="store_true",
                    help="disable the FM threshold/click knee")
    ap.add_argument("--dev-limit", type=float, default=DEV_LIMIT,
                    help="deviation limiter clip level (default %(default)s)")
    ap.add_argument("--preemph-us", type=float, default=PREEMPH_TAU_US,
                    help="pre/de-emphasis time constant in us (default 75, CCITT)")
    ap.add_argument("--loss", type=float, default=0.0,
                    help="impulse/burst erasure fraction (0..1); orthogonal to "
                         "fading (lightning/key-clicks), NOT the fade proxy")
    ap.add_argument("--burst", action="store_true",
                    help="Gilbert-Elliott bursty impulse dropout (else memoryless)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--barrier-k", type=int, default=1,
                    help="conservative-PDES window-barrier credit (chunks). K=1 = "
                         "strict lockstep (default); relax to 4-8 if it throttles.")
    ap.add_argument("--idle-bigstep", type=int, default=1,
                    help="coalesce up to N both-silent idle chunks per forward "
                         "(FTRT speed-up). 1 = strict 1:1 (CONNECT-safe, default).")
    ap.add_argument("--wire-stamp", type=int, default=0, choices=(0, 1),
                    help="prepend the 8-byte <Q per-direction END-sample stamp to "
                         "each chunk (8192->8200). DEFAULT 0 (bare 8192, matches "
                         "Iris audio_sim.cc). Set 1 ONLY for a stamp-aware modem.")
    ap.add_argument("--log", default=None)
    ap.add_argument("--airtime-json", default=None,
                    help="write the per-direction airtime breakdown JSON on exit")
    args = ap.parse_args()
    if args.barrier_k < 1:
        ap.error("--barrier-k must be >= 1")

    if args.cell:
        snr, prof = parse_cell(args.cell)
        args.snr = snr
        if prof is not None:
            args.profile = prof

    logf = open(args.log, "w") if args.log else sys.stdout

    def log(msg):
        ts = time.strftime("%H:%M:%S")
        logf.write(f"[{ts}] {msg}\n")
        logf.flush()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(4)
    prof = PROFILES.get(args.profile)
    prof_s = ("flat AWGN" if prof is None
              else f"Rician fd={args.fade_doppler_hz}Hz K={args.rician_k_db}dB")
    log(f"FM relay listening on 127.0.0.1:{args.port} "
        f"S:N={args.snr:.2f}dB(3kHz) profile={args.profile} ({prof_s}) "
        f"cfo={args.cfo_hz}Hz phase_noise={args.phase_noise_deg}deg "
        f"clicks={'off' if args.no_clicks else 'on'} preemph={args.preemph_us}us "
        f"dev_limit={args.dev_limit} seed={args.seed} barrier_k={args.barrier_k} "
        f"wire={'STAMPED(8200)' if args.wire_stamp else 'BARE(8192)'}")

    peers = {}
    while len(peers) < 2:
        sock, _ = srv.accept()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        tag = recv_exact(sock, 1)
        if tag is None:
            sock.close()
            continue
        role = tag.decode("ascii", "ignore")
        peers[role] = Peer(sock, role)
        log(f"peer connected role='{role}' ({len(peers)}/2)")

    a = peers.get("A")
    b = peers.get("B")
    if a is None or b is None:
        log("ERROR: need both role A and role B; got " + ",".join(peers.keys()))
        return 1

    ch_a2b = Channel(args, args.seed * 2654435761 & 0xFFFFFFFF)
    ch_b2a = Channel(args, args.seed * 40503 + 7 & 0xFFFFFFFF)
    log(f"calib a2b: g_sig={ch_a2b.g_sig:.4f} g_noise={ch_a2b.g_noise:.4f} "
        f"sigma_base={ch_a2b.sigma_base:.6f}")
    # physical reciprocity: reverse direction sees the opposite CFO sign.
    if args.cfo_hz != 0.0 and args.cfo_reverse_sign == -1:
        ch_b2a.cfo_hz = -args.cfo_hz
        ch_b2a.cfo_w = 2.0 * math.pi * ch_b2a.cfo_hz / FS

    stop = threading.Event()
    counters = {"a2b": 0, "b2a": 0}
    sig_chunks = {"a2b": 0, "b2a": 0}
    sil_chunks = {"a2b": 0, "b2a": 0}

    BARRIER_K = max(1, args.barrier_k)
    inq = {"a2b": queue.Queue(maxsize=BARRIER_K + 1),
           "b2a": queue.Queue(maxsize=BARRIER_K + 1)}
    chans = {"a2b": ch_a2b, "b2a": ch_b2a}

    BIGSTEP = max(1, args.idle_bigstep)
    SILENCE_EPS = 1e-12

    def reader(src, key, ch):
        while not stop.is_set():
            raw = recv_exact(src.sock, CHUNK_BYTES)
            if raw is None:
                log(f"{key}: source closed")
                stop.set()
                try:
                    inq[key].put_nowait(None)
                except queue.Full:
                    pass
                break
            samples = list(struct.unpack(f"<{CHUNK_SAMPLES}d", raw))
            silent = True
            for v in samples:
                if v > SILENCE_EPS or v < -SILENCE_EPS:
                    silent = False
                    break
            out = ch.process(samples)
            while not stop.is_set():
                try:
                    inq[key].put((out, silent), timeout=0.2)
                    break
                except queue.Full:
                    continue

    CLOSED, FORWARDED, WOULDBLOCK = -1, 1, 0
    WIRE_STAMP = bool(args.wire_stamp)

    def _account(key, n, silent):
        if silent:
            sil_chunks[key] += n
        else:
            sig_chunks[key] += n

    def _send_stamped(key, out):
        dst = b if key == "a2b" else a
        ch = chans[key]
        stamp = counters[key] * CHUNK_SAMPLES
        payload = struct.pack(f"<{CHUNK_SAMPLES}d", *out)
        packed = (struct.pack("<Q", stamp) + payload) if WIRE_STAMP else payload
        if not send_all(dst.sock, packed):
            log(f"{key}: dest closed")
            stop.set()
            return CLOSED
        if counters[key] % 500 == 0:
            log(f"{key}: {counters[key]} chunks "
                f"({counters[key]*CHUNK_SAMPLES/FS:.1f}s) "
                f"vstamp={stamp} split={counters['a2b']-counters['b2a']:+d} "
                f"P_sig={ch.p_sig:.5f} sigma={ch.sigma_base:.6f} "
                f"txpeak={ch.peak_diag:.4f}")
        return FORWARDED

    def forward_one(key):
        other = "b2a" if key == "a2b" else "a2b"
        if counters[key] - counters[other] >= BARRIER_K:
            return WOULDBLOCK
        try:
            out, silent = inq[key].get_nowait()
        except queue.Empty:
            return WOULDBLOCK
        if out is None:
            stop.set()
            return CLOSED
        counters[key] += 1
        _account(key, 1, silent)
        return _send_stamped(key, out)

    def try_bigstep():
        if BIGSTEP <= 1:
            return 0
        if counters["a2b"] != counters["b2a"]:
            return 0
        try:
            a_out, a_sil = inq["a2b"].get_nowait()
        except queue.Empty:
            return 0
        try:
            b_out, b_sil = inq["b2a"].get_nowait()
        except queue.Empty:
            if a_out is None:
                stop.set(); return -1
            counters["a2b"] += 1
            _account("a2b", 1, a_sil)
            return -1 if _send_stamped("a2b", a_out) == CLOSED else 0
        if a_out is None or b_out is None:
            stop.set(); return -1
        if not (a_sil and b_sil):
            counters["a2b"] += 1
            _account("a2b", 1, a_sil)
            if _send_stamped("a2b", a_out) == CLOSED:
                return -1
            counters["b2a"] += 1
            _account("b2a", 1, b_sil)
            if _send_stamped("b2a", b_out) == CLOSED:
                return -1
            return 0
        a_last, a_n = a_out, 1
        b_last, b_n = b_out, 1
        while a_n < BIGSTEP:
            try:
                o, s = inq["a2b"].get_nowait()
            except queue.Empty:
                break
            if o is None: stop.set(); return -1
            if not s:
                counters["a2b"] += a_n
                _account("a2b", a_n, True)
                if _send_stamped("a2b", a_last) == CLOSED: return -1
                counters["a2b"] += 1
                _account("a2b", 1, False)
                if _send_stamped("a2b", o) == CLOSED: return -1
                a_last, a_n = None, 0
                break
            a_last, a_n = o, a_n + 1
        while b_n < BIGSTEP:
            try:
                o, s = inq["b2a"].get_nowait()
            except queue.Empty:
                break
            if o is None: stop.set(); return -1
            if not s:
                counters["b2a"] += b_n
                _account("b2a", b_n, True)
                if _send_stamped("b2a", b_last) == CLOSED: return -1
                counters["b2a"] += 1
                _account("b2a", 1, False)
                if _send_stamped("b2a", o) == CLOSED: return -1
                b_last, b_n = None, 0
                break
            b_last, b_n = o, b_n + 1
        if a_n > 0:
            counters["a2b"] += a_n
            _account("a2b", a_n, True)
            if _send_stamped("a2b", a_last) == CLOSED: return -1
        if b_n > 0:
            counters["b2a"] += b_n
            _account("b2a", b_n, True)
            if _send_stamped("b2a", b_last) == CLOSED: return -1
        return max(a_n, b_n, 1)

    def scheduler():
        rr = 0
        while not stop.is_set():
            r = try_bigstep()
            if r < 0:
                return
            progressed = (r > 0)
            if not progressed:
                cands = sorted(("a2b", "b2a"),
                               key=lambda k: (counters[k], 0 if k == ("a2b", "b2a")[rr % 2] else 1))
                for key in cands:
                    rf = forward_one(key)
                    if rf == CLOSED:
                        return
                    if rf == FORWARDED:
                        progressed = True
            rr += 1
            if not progressed and not stop.is_set():
                time.sleep(0.0005)

    ta = threading.Thread(target=reader, args=(a, "a2b", ch_a2b), daemon=True)
    tb = threading.Thread(target=reader, args=(b, "b2a", ch_b2a), daemon=True)
    tsched = threading.Thread(target=scheduler, daemon=True)
    ta.start()
    tb.start()
    tsched.start()

    try:
        while not stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop.set()

    for p in (a, b):
        try:
            p.sock.close()
        except OSError:
            pass
    log(f"relay done. a2b={counters['a2b']} b2a={counters['b2a']} chunks "
        f"(final split={counters['a2b']-counters['b2a']:+d}, barrier_k={args.barrier_k})")

    def _airtime_s(key):
        return sig_chunks[key] * CHUNK_SAMPLES / FS

    def _total_s(key):
        return counters[key] * CHUNK_SAMPLES / FS

    for key in ("a2b", "b2a"):
        tot = counters[key]
        frac = (sig_chunks[key] / tot) if tot else 0.0
        log(f"airtime {key}: signal={sig_chunks[key]} silence={sil_chunks[key]} "
            f"total={tot} chunks | signal_airtime={_airtime_s(key):.2f}s "
            f"total_virtual={_total_s(key):.2f}s signal_frac={frac:.3f}")

    if args.airtime_json:
        try:
            with open(args.airtime_json, "w") as af:
                json.dump({
                    "fs": FS,
                    "chunk_samples": CHUNK_SAMPLES,
                    "barrier_k": args.barrier_k,
                    "wire_stamp": int(WIRE_STAMP),
                    "a2b": {
                        "signal_chunks": sig_chunks["a2b"],
                        "silence_chunks": sil_chunks["a2b"],
                        "total_chunks": counters["a2b"],
                        "signal_airtime_s": _airtime_s("a2b"),
                        "total_virtual_s": _total_s("a2b"),
                    },
                    "b2a": {
                        "signal_chunks": sig_chunks["b2a"],
                        "silence_chunks": sil_chunks["b2a"],
                        "total_chunks": counters["b2a"],
                        "signal_airtime_s": _airtime_s("b2a"),
                        "total_virtual_s": _total_s("b2a"),
                    },
                }, af, indent=1)
            log(f"wrote airtime breakdown -> {args.airtime_json}")
        except OSError as e:
            log(f"WARN: could not write airtime-json {args.airtime_json}: {e}")
    return 0


# ---------------------------------------------------------------------------
# Self-check: parse + high-S:N pass-through approximates identity in-band.
# ---------------------------------------------------------------------------
def _selfcheck():
    class _A:
        snr = 80.0; profile = "wgn"; sig_ref = 0.3; cfo_hz = 0.0
        phase_noise_deg = 0.0; fade_doppler_hz = FADE_DOPPLER_HZ
        rician_k_db = RICIAN_K_DB; click_gain = CLICK_GAIN; click_bw = CLICK_BW
        no_clicks = True; dev_limit = 0.95; preemph_us = PREEMPH_TAU_US
        loss = 0.0; burst = False
    ch = Channel(_A(), 1234)
    # 1 kHz in-band tone, low amplitude (no clipping).
    fs = FS
    t = np.arange(20 * CHUNK_SAMPLES) / fs
    tone = 0.1 * np.sin(2.0 * math.pi * 1000.0 * t)
    out = []
    for k in range(0, tone.size, CHUNK_SAMPLES):
        out.extend(ch.process(tone[k:k + CHUNK_SAMPLES]))
    out = np.asarray(out)
    # skip filter warm-up transient; compare steady-state RMS.
    warm = 6 * CHUNK_SAMPLES
    in_rms = float(np.sqrt(np.mean(tone[warm:] ** 2)))
    out_rms = float(np.sqrt(np.mean(out[warm:] ** 2)))
    ratio = out_rms / in_rms if in_rms else 0.0
    # correlation of aligned steady-state (allow small group delay).
    a = tone[warm:tone.size]
    bb = out[warm:out.size]
    m = min(a.size, bb.size)
    corr = float(np.max(np.correlate(bb[:m] / (np.linalg.norm(bb[:m]) + 1e-12),
                                     a[:m] / (np.linalg.norm(a[:m]) + 1e-12),
                                     mode="same")))
    print(f"[selfcheck] g_sig={ch.g_sig:.4f} g_noise={ch.g_noise:.4f}")
    print(f"[selfcheck] in_rms={in_rms:.5f} out_rms={out_rms:.5f} "
          f"ratio={ratio:.4f} peak_corr={corr:.4f}")
    ok_ratio = 0.7 <= ratio <= 1.4
    ok_corr = corr >= 0.9
    # noise-floor sanity: at S:N=80 dB the added noise power must be ~ -80 dB.
    print(f"[selfcheck] identity {'PASS' if (ok_ratio and ok_corr) else 'FAIL'} "
          f"(ratio_ok={ok_ratio} corr_ok={ok_corr})")
    return 0 if (ok_ratio and ok_corr) else 2


if __name__ == "__main__":
    if "--selfcheck" in sys.argv:
        sys.exit(_selfcheck())
    sys.exit(main())
