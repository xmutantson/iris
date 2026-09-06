#!/usr/bin/env python3
"""
iris_honest_baseline_runner.py - honest-baseline runner for Iris FM.

Runs Iris on the DEPLOYMENT PATH (AFSK connect -> --native-hail probe/native
OFDM-KISS, KISS-driven, COLD/purged-cache, NEVER --force-ofdm) through the
faithful real-time snd-aloop IONOS bridge (tools/sim/realaudio/realaudio_bridge_s32.py
+ the raw-S32 `--alsa-raw` Iris backend), on the compute fleet (.11/.21/.31),
across the 8 VARA cells (WGN {40,30,20,15} + MPG {40,30,20,15}, commanded via
--cell so the bridge applies WGN_TO_SNR3K=+2.4: the sheet S:N is the COMMANDED
IONOS dial, and the board delivers ~2.4 dB more SNR3k than the dial reads),
N>=8 COLD sessions per cell, PARALLEL and load-immune, coexisting with the live
Mercury capstone matrix (disjoint atomic snd-aloop card claims).

It emits (per (cell,dict-cohort)) at most ONE number, NEVER MIXED:
  (a) wire-parity B/min  (dict OFF): B2F-compressed bytes/min, bar-comparable to
      vara_fm_bar_2025.json narrow (primed dict does NOT count here);
  (b) plaintext goodput B/min (dict ON): delivered plaintext bytes/min, compared
      to VARA plaintext re-derived from VARA's LZHUF ratio on the identical corpus.
Dict-ON has NO mechanism in Iris today, so the runner defaults to the dict-OFF
cohort only (see --cohorts / preflight dict_primed_backend).

THE GATE IS DEFAULT-DENY (C3).  A session scores a number ONLY on POSITIVE
attestation of ALL of:
  * bridge channel-attestation (C1) present AND matching the plan
    (cell/profile/commanded_snr/seed; passthrough==False);
  * the resolved OFDM config tuple (nfft / band / carriers) present-AND-EQUAL on
    BOTH the CMD and RSP logs;
  * binary md5 + git HEAD recorded (and equal) for both ends;
  * dict-state matching the cohort;
  * pump delivered-manifest (C2) present with hash-VERIFIED delivered bytes.
Missing / mismatched ANYTHING => the cell yields NO number.  Polarity is SPLIT:
  * HARNESS-fault (force-ofdm, warm-cache, C1 attestation mismatch, config
    asymmetry, missing binary/pump evidence) => INVALID, RE-RUN the same seed;
  * MODEM-fault (NO_OFDM, zero-deliver, activation split-brain) => score the
    ACTUAL delivered bytes (usually 0) INTO the distribution, and report the
    per-cell activation-rate as a first-class line-item.
This machine enforcement exists because a human measurement rule was violated once
(every prior win scored on --force-ofdm); a machine rule will not be.

STATUS (2026-07-02): dispatch_fleet() is WIRED (SSH card-claim + 5-process
session + teardown + evidence collection + score + bounded harness re-run) but is
NOT exercised live here -- validate via --dry-run + --self-test (golden-log +
gate-polarity fixtures) ONLY.  A live run additionally refuses to score any
cohort whose delivery dependency is unmet:
  D2. the raw-S32 `--alsa-raw` Iris backend reads per-direction device strings
      from env IRIS_ALSA_RAW_CAPTURE / IRIS_ALSA_RAW_PLAYBACK (audio_alsa_raw.cc
      :44-56; main.cc:343-347 takes ONE optional positional base device).
  D3. the corpus B2F-reroll bit-identity proof + per-message b2f_nodict/b2f_dict
      size table (already filled below).

Run FROM the workspace (Windows or Linux); the fleet work is dispatched over SSH
(`ssh -i ~/.ssh/kameron_fleet kameron@192.168.2.{11,21,31}`).  No fleet-only
Python module (alsaaudio) is imported here; the bridge imports those ON the box.

The implementation enforces the full measurement spec, including the five-question
shared-state audit of the snd-aloop card pool and its associated measurement rules.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import shlex
import statistics
import subprocess
import sys
import time

RUNNER_VERSION = "0.3.0-guards"

# ---------------------------------------------------------------------------
# Repo-relative anchors (this file lives in iris/tools/)
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
# Measurement-integrity guards (tools/measurement_guards.py, same dir).  Four
# machine rules, each motivated by a real 2026-07-09/10 measurement failure:
#   G1 arm provenance (build= content hash on every row; an A/B refuses
#      same-binary arms - the wgn40_auth_arm decoy class);
#   G2 disproof scale (negative verdicts below the message floor demote to
#      INCONCLUSIVE - the 4-message "DISPROVEN");
#   G3 significance (improvement fields refused when the paired effect is not
#      significant - the p=0.69 "1.6x win");
#   G4 citability (every ratio stamped citable true/false with a provenance
#      string - the hand-derived "8281 = 0.155x").
# HARD import: a runner without its guards must not score anything.
sys.path.insert(0, HERE)
import measurement_guards as mg  # noqa: E402
IRIS_REPO = os.path.dirname(HERE)                          # .../iris
WORKSPACE = os.path.dirname(IRIS_REPO)                     # .../hermes and mercury
BRIDGE = os.path.join(WORKSPACE, "tools", "sim", "realaudio", "realaudio_bridge_s32.py")
KISS_PUMP = os.path.join(WORKSPACE, "tools", "rpi_scripts", "kiss_data_pump.py")
VARA_BAR_JSON = os.path.join(HERE, "vara_fm_bar_2025.json")
# Instrument gate (workspace tools/bench_selftest.py).  Per-session it checks
# GRID CONCORDANCE: both ends must have fingerprinted the SAME carrier grid
# ([GRID] fp lines) before any throughput number from the session may exist —
# a split grid puts the ZC training symbol in bins the RX never searches, an
# invisible 0-delivery that would otherwise score as a modem fault.  Byte
# attestation stays with the existing C2 pump-manifest hash gate.
BENCH_SELFTEST = os.path.join(WORKSPACE, "tools", "bench_selftest.py")


def instrument_session_gate(cmd_log, rsp_log):
    """Run the per-session instrument gate; returns its parsed JSON verdict.
    Missing tool => None with a LOUD warning (never a silent pass)."""
    if not os.path.isfile(BENCH_SELFTEST):
        sys.stderr.write("[iris-baseline] WARN: %s missing — instrument gate "
                         "SKIPPED (grid concordance unchecked)\n" % BENCH_SELFTEST)
        return None
    try:
        p = subprocess.run([sys.executable, BENCH_SELFTEST, "session-gate",
                            "--cmd-log", cmd_log, "--rsp-log", rsp_log],
                           capture_output=True, text=True, timeout=60)
        return json.loads(p.stdout)
    except (OSError, ValueError, subprocess.TimeoutExpired) as e:
        return {"pass": False, "reason": "instrument gate unrunnable: %s" % e}


def instrument_gate_positive_split(gate):
    """True when BOTH ends fingerprinted a grid and the fingerprints DIFFER —
    the unambiguous instrument lie that poisons a whole cohort.  (Missing
    fingerprints on a 0-delivery session are a modem/connect failure, scored
    normally, not an abort.)"""
    if gate is None or gate.get("pass"):
        return False
    g3 = (gate.get("parts") or {}).get("grid_concordance") or {}
    return bool(g3.get("cmd_grid")) and bool(g3.get("rsp_grid")) and not g3.get("pass")
# Checked-in corpus BODIES (the b2f_nodict wire bytes that fly) + the materialized
# --payload-file manifest.  These SHIP to the box in the iris checkout; the pump
# reads corpus.json so the delivered bytes ARE the b2f_nodict wire load and the
# numerator is BOUND to them (HOLE 3: name-only crediting could flatter ~1.67x).
CORPUS_DIR = os.path.join(HERE, "corpus")
CORPUS_JSON = os.path.join(CORPUS_DIR, "corpus.json")
# FAIR corpus-scaling (hole-5 methodology).  Each cell gets a per-cell corpus of N DISTINCT
# rerolled Winlink variants (make_scaled_corpus.py) sized so the projected Iris session
# runs ~T_target=900 s -- inside VARA's own 4-120 min envelope -- because bytes/min is
# only size-sensitive through the connect-tax fraction, and the VARA sheet pins no byte
# size to match.  The scaled bodies + manifest live under corpus/scaled/<channel>_<snr>/;
# the runner reads manifest.json (load_scaled_corpus), materializes the per-cell corpus.json
# the pump flies, and binds every delivered record's {bytes,sha256} to the per-variant table.
SCALED_DIR = os.path.join(CORPUS_DIR, "scaled")
# A-priori sizing constants (HOLE5 SCALING RULE), fixed a priori + auditable.
T_TARGET_S = 900              # planned transfer per cell (15 min; inside VARA's [4,120] min)
SESSION_SECS_FACTOR = 1.5     # plan session_secs = 1.5 * T_target so completes finish inside
                             # the teardown contract; oversized payloads become PARTIALS
                             # scored on the plan wall (honest by construction).
N_SETS_FLOOR = 2             # >= 2 sets guarantees a meaningful steady-state span
N_SETS_CAP = 54              # ~200 kB; bounds fleet wall-clock

# ---------------------------------------------------------------------------
# The 8 VARA cells.  channel in {WGN, MPG}; snr is the COMMANDED IONOS dial S:N
# from the sheet (owner 2026-07-02: the sheet S:N is the front-panel command, NOT
# a measured output).  Passed via --cell so the bridge adds WGN_TO_SNR3K=+2.4 ->
# the ACTUAL SNR3k the IONOS produced = what VARA saw.  This is IDENTICAL to
# Mercury's WGN-label testbed (both dial the same IONOS), so the offset applies.
# ---------------------------------------------------------------------------
CELLS = [("WGN", 40), ("WGN", 30), ("WGN", 20), ("WGN", 15),
         ("MPG", 40), ("MPG", 30), ("MPG", 20), ("MPG", 15)]

# dict cohort -> which SINGLE metric that cohort is allowed to emit.  The two
# bases are NEVER produced from one session (that is the "never mixed" rule made
# mechanical): a dict-off session yields ONLY wire-parity; dict-on ONLY plaintext.
# NOTE (item 4): dict-ON has NO mechanism in Iris today, so the default cohort set
# is dict-OFF ONLY.  The "on" entry is kept so the metric machinery is ready when a
# primed-dict backend lands; a live run refuses "on" (preflight dict_primed_backend).
DICT_COHORTS = {
    "off": "wire_parity",     # primed dict DISABLED -> B2F-compressed wire basis
    "on":  "plaintext",       # primed dict ENABLED  -> delivered plaintext basis
}
DEFAULT_COHORTS = "off"       # dict-ON has no mechanism in Iris (item 4)

# sim_channel_relay PROFILES: WGN->wgn (pure AWGN), MPG->mpg (ITU HF "good").
PROFILE_FOR = {"WGN": "wgn", "MPG": "mpg"}

FLEET = {"11": "192.168.2.11", "21": "192.168.2.21", "31": "192.168.2.31"}
SSH_KEY = "~/.ssh/kameron_fleet"
SSH_USER = "kameron"
# The bridge + KISS pump run ON the fleet box (not this Windows host), so they use
# the box python3 + the box checkout of the workspace tools -- NOT this file's local
# sys.executable / WORKSPACE paths.  --fleet-root points at the box workspace root.
FLEET_PY = "python3"
DEFAULT_FLEET_ROOT = "~/hermes-and-mercury"
# Fleet policy: build/scratch on tmpfs, durable results to disk.
FLEET_SCRATCH = "/dev/shm/iris_baseline"
# Atomic snd-aloop card-claim registry (mkdir is atomic on POSIX).  Coexists with
# the live Mercury capstone: we claim only cards that are (a) Loopback, (b) not
# busy (no open PCM substream, so capstone's cards are skipped), (c) not already
# in this registry.  Claim from a HIGH card-base to steer clear of capstone's
# low range as a second line of defence.
FLEET_CLAIM_DIR = "/dev/shm/iris_aloop_claims"
DEFAULT_CARD_BASE = 12       # start high; capstone tends to fill from 0
CARD_SCAN_SPAN = 24          # snd-aloop pool is 24 cards on the R730s

# Per-session process sequencing (mirrors arq_realaudio.py:429-456: bridge first,
# then RSP, then CMD -- the responder must be listening before the initiator hails).
SEQ_AFTER_BRIDGE_S = 2.0
SEQ_AFTER_RSP_S = 3.0
SEQ_AFTER_CMD_S = 3.0
TEARDOWN_GRACE_S = 10.0      # SIGTERM grace before SIGKILL (atexit/log drain)
# FIX-C: event-driven teardown.  The old fixed TEARDOWN_GRACE_S raced the pump's
# manifest write at slow-connect hard cells (the pump wrote its manifest only at
# main() end), systematically WITHHOLDING those cells.  The pump now flushes its
# manifest incrementally + on SIGTERM/atexit, so teardown sends SIGTERM then WAITS
# (bounded) for BOTH manifests to land before SIGKILL, instead of a blind grace.
TEARDOWN_MANIFEST_WAIT_S = 60.0   # bounded wait for the pump manifests after SIGTERM
TEARDOWN_POLL_S = 2.0             # poll interval while waiting for the manifests
DEFAULT_SESSION_SECS = 300   # per-session data window; matched to --session-secs
# The per-session teardown grace the dispatcher SLEEPS after the data window before it
# SIGTERMs the pumps (run_session below).  Named so the metric's planned-session WALL and
# the dispatcher's teardown schedule stay in lockstep.
SESSION_TEARDOWN_GRACE_S = 30
# Connect + drain + teardown airtime that lives OUTSIDE the session_secs DATA window.
# The pump starts its --duration data clock AFTER the AFSK connect (kiss_data_pump.py:789)
# and drains ~15 s past it (kiss_data_pump.py:849); the dispatcher then SIGTERMs at
# ~first_sabm_tx + session_secs + SESSION_TEARDOWN_GRACE_S and lets bytes verify for up to
# TEARDOWN_MANIFEST_WAIT_S more.  So the FULL session wall from the SENDER's first SABM TX
# is bounded by session_secs + this overhead -- and is NEVER bounded by session_secs alone
# (connect + drain + teardown are outside the data window).  This overhead is what makes
# the metric invariant -- a partial/incomplete delivery can NEVER out-rank a complete one
# -- hold BY CONSTRUCTION: the planned-session wall t0 + PLANNED_SESSION_SECONDS is >= any
# complete delivery's true wall (last_verified_byte_epoch - t0).  See plan_session_wall_secs.
SESSION_WALL_OVERHEAD_S = SESSION_TEARDOWN_GRACE_S + TEARDOWN_MANIFEST_WAIT_S   # 90 s
MAX_HARNESS_RERUNS = 3       # bounded re-run of a HARNESS-fault (deterministic
                             # config-asym would loop forever otherwise -> surfaced)

# ---------------------------------------------------------------------------
# Fixed Winlink corpus (HONEST_MEASUREMENT_HARNESS §3.1).  Each message carries
# THREE checked-in sizes so the two metric numerators are honest and never mixed:
#   plaintext   = app bytes the user reads (plaintext-goodput numerator, dict ON)
#   b2f_nodict  = Iris B2F-compressed size with NO primed dict (wire-parity
#                 numerator, dict OFF; bar-comparable to VARA's compressed bytes)
#   b2f_dict    = Iris B2F-compressed size WITH the primed dict (diagnostic)
# b2f_* are None until the B2F-reroll bit-identity tool (risk 6 / C3
# gate) fills them; metric_session() REFUSES the wire-parity number while
# b2f_nodict is None rather than fabricating a compression ratio.
# ---------------------------------------------------------------------------
# DICT-ON PRIMED BASIS (SPEED_ATTACK C2/C3/C4).  b2f_dict is the PER-MESSAGE primed
# wire measured by the PRODUCTION iris::Compressor (tools/iris_heldout_primed.cc) on a
# FRESH stream per message (dict priming ON) -- NO session carry, NO cross-message dedup
# Iris would not have on distinct real traffic.  The dict is TRAINED on set A = Winlink
# BOILERPLATE ONLY (tools/winlink_dict_v1.txt: headers / form scaffolding / GENERIC net
# phrasing, NO message bodies); these messages are set B, DISJOINT from set A (held-out;
# heldout_disjointness_selftest asserts no body memorization).  The originally-published
# 916-B representative dict memorized the net_checkin BODY verbatim (train-on-test, edge
# 3.16 -> inflated); the shipped dict rewords that to generic phrasing, so the HELD-OUT
# per-message edges are net_checkin 2.11 / ics213 1.91 / short_email 1.44 / batch3 1.51
# and the byte-weighted aggregate held-out edge E = 1.600 (was 1.665 with the leaky
# representative dict).  bit-exact RT proven for every message.
CORPUS = [
    # b2f_nodict (dict-off LZHUF wire bytes = what goes on the air, comparable to the VARA
    # bar) measured on the REAL Winlink corpus (_research/b2f_unroll_harness/corpus/) via
    # the production LZHUF -- reroll unroll->reroll byte-identity proven 6/6 (D3)
    # and now a HARD gate (C1, b2f_reroll_shippable).
    #
    # `body` is the checked-in b2f_nodict wire file under CORPUS_DIR; `sha256` is its
    # digest.  The pump sends EXACTLY these bytes (via corpus.json), so delivered
    # per_message {bytes,sha256} must MATCH {b2f_nodict, sha256} here -- the gate
    # binds the wire-parity numerator to the bytes that actually flew (HOLE 3).
    # INVARIANT: bytes == b2f_nodict == len(body file) == what metric_session credits.
    # `plaintext_file`/`plaintext_sha256` are the held-out set-B body (C3 disjointness).
    {"name": "net_checkin",  "plaintext": 387,  "b2f_nodict": 306,  "b2f_dict": 145,
     "body": "net_checkin.b2f",
     "sha256": "a61505369fe4d53a115eb48df9d49e464de469af3e0dac1b514096c4b3e91a7d",
     "plaintext_file": "net_checkin.plain",
     "plaintext_sha256": "ad83e7460c6f257075ed43e770b3acc2e63fb38a10f43dfeb654a6f827f977f1"},
    {"name": "ics213",       "plaintext": 1164, "b2f_nodict": 792,  "b2f_dict": 415,
     "body": "ics213.b2f",
     "sha256": "e52a65f0cccbc449ad71e1073d31b446978d1f3b56c76d406d1020becdbaa463",
     "plaintext_file": "ics213.plain",
     "plaintext_sha256": "183ea77bdd8294c5c208fb5c859cf7a48e33e09b1d18af86a16cb73d63bc7757"},
    {"name": "short_email",  "plaintext": 1384, "b2f_nodict": 910,  "b2f_dict": 632,
     "body": "short_email.b2f",
     "sha256": "c55bea992d9f3fc70c7845c3065d569ef84a7b3d3874e0ab32b5392c1c277cce",
     "plaintext_file": "short_email.plain",
     "plaintext_sha256": "d3bd6bdb707747728c2a92aa532632c1578887baf9e4fd3a26bdc69f8ffbaffd"},
    {"name": "batch3",       "plaintext": 2935, "b2f_nodict": 1702, "b2f_dict": 1127,
     "body": "batch3.b2f",
     "sha256": "8c93e283508ce4425b0f2c38848ee64ae773dae4d2940093b95178cbf824ff47",
     "plaintext_file": "batch3.plain",
     "plaintext_sha256": "07eb6c97d4f3ad3c21648b06ba159eb41d2590ae742f786a149edef9f589febc"},
]
CORPUS_PLAINTEXT_TOTAL = sum(m["plaintext"] for m in CORPUS)      # 5870
CORPUS_B2F_NODICT_TOTAL = sum(m["b2f_nodict"] for m in CORPUS)    # 3710
CORPUS_B2F_DICT_TOTAL = sum(m["b2f_dict"] for m in CORPUS)        # 2319 (held-out primed)
# Firmware-baked priming dict (set A). Held-out disjointness is asserted against it.
DICT_TXT = os.path.join(HERE, "winlink_dict_v1.txt")
# Longest contiguous byte run a held-out set-B body may share with set A (the dict).
# Universal header boilerplate legitimately shares ~96 B; body memorization is ~140 B.
HELDOUT_MAX_SHARED_RUN = 120
# Aggregate plaintext/B2F-LZHUF ratio over the CORPUS above (sum plaintext / sum b2f_nodict
# = 5870/3710 = 1.58), measured with the production LZHUF on the identical real Winlink corpus
# (D3).  Used to re-derive VARA's plaintext rate for the dict-ON comparison.
# CAVEAT: the dict-ON lever uses a 916-B hand-curated boilerplate dict (not a trained/versioned
# zstd dict) -> a trained dict could lift more; this is a conservative floor.
VARA_LZHUF_RATIO = round(CORPUS_PLAINTEXT_TOTAL / CORPUS_B2F_NODICT_TOTAL, 2)  # 1.58

# DELIVERED-PLAINTEXT AXIS (SPEED_ATTACK C4), DERIVED not measured-on-VARA.  This is the
# SECOND, labelled axis: message bytes / session wall, VARA credited its OWN LZHUF ratio
# on the identical corpus.  The primed dict counts HERE and only here (the wire-parity
# dict-OFF headline is UNCHANGED -- b2f_nodict / the narrow bar).  The primed edge is
# measured PER-MESSAGE (fresh stream, no session carry -- CORPUS b2f_dict), so it never
# borrows the streaming-carry / cross-message-dedup lift Iris would not have on distinct
# real traffic (the dict-ON trap).  Iris primed ratio = plaintext / b2f_dict; VARA ratio =
# plaintext / b2f_nodict (LZHUF, cannot prime); E = Iris_primed / VARA = b2f_nodict/b2f_dict.
# All numbers are HELD-OUT (set B disjoint from the set-A dict; heldout_disjointness_selftest).
PRIMED_EDGE_HELDOUT = round(CORPUS_B2F_NODICT_TOTAL / CORPUS_B2F_DICT_TOTAL, 3)  # 3710/2319 = 1.600


def primed_delivered_edge(corpus=None):
    """DERIVED delivered-plaintext edge E = R_iris_primed / R_vara, byte-weighted over the
    corpus and PER-MESSAGE (no session carry).  E = sum(b2f_nodict) / sum(b2f_dict): VARA's
    plaintext ratio is plaintext/b2f_nodict, Iris's primed ratio is plaintext/b2f_dict, so
    their quotient is b2f_nodict/b2f_dict.  Returns {per_class:{name:edge}, aggregate}.
    A published plaintext-axis number MUST come from held-out set B (assert via
    heldout_disjointness_selftest)."""
    corpus = corpus if corpus is not None else CORPUS
    per = {}
    for m in corpus:
        bd = m.get("b2f_dict")
        bn = m.get("b2f_nodict")
        per[m["name"]] = round(bn / bd, 3) if (bd and bn) else None
    agg_bn = sum(m["b2f_nodict"] for m in corpus if m.get("b2f_dict") and m.get("b2f_nodict"))
    agg_bd = sum(m["b2f_dict"] for m in corpus if m.get("b2f_dict") and m.get("b2f_nodict"))
    return {"per_class": per,
            "aggregate": round(agg_bn / agg_bd, 3) if agg_bd else None,
            "basis": "per-message primed (no session carry), held-out set B"}


def _longest_shared_run(msg: bytes, dic: bytes) -> int:
    """Longest contiguous byte run shared by msg and dic (held-out disjointness metric).
    O(n*m); msg < 3 KB, dic < 16 KB.  A large run == the dict memorized the body."""
    best = 0
    mlen, dlen = len(msg), len(dic)
    for i in range(mlen):
        for j in range(dlen):
            k = 0
            while i + k < mlen and j + k < dlen and msg[i + k] == dic[j + k]:
                k += 1
            if k > best:
                best = k
    return best


# ---------------------------------------------------------------------------
# Corpus materialization + integrity (HOLE 1 + HOLE 3).
#
# The pump is driven with --payload-file <corpus.json>; without it the pump sends
# synthetic_NNNNNN records that VERIFY but metric_session filters out by name -> a
# gate-clean structural 0.0 (scoring-on-absence INSIDE the metric).  corpus.json is
# built REPRODUCIBLY from the checked-in b2f_nodict body files so what flies on the
# wire is exactly the tabulated b2f_nodict load, and the gate can bind the numerator
# to it by {bytes,sha256}.
# ---------------------------------------------------------------------------
def _corpus_body_path(m, corpus_dir=CORPUS_DIR):
    return os.path.join(corpus_dir, m["body"])


def read_corpus_bodies(corpus=None, corpus_dir=CORPUS_DIR):
    """Read the checked-in b2f_nodict body files; assert each matches its table entry
    (len == b2f_nodict, sha256 == table).  `corpus`/`corpus_dir` default to the module
    unit table (the 4-template UNIT corpus); the per-cell SCALED corpus passes its own
    variant list + scaled dir.  Returns [(name, payload_bytes)]."""
    corpus = corpus if corpus is not None else CORPUS
    out = []
    for m in corpus:
        with open(_corpus_body_path(m, corpus_dir), "rb") as f:
            payload = f.read()
        if len(payload) != m["b2f_nodict"]:
            raise ValueError(f"corpus body {m['body']}: {len(payload)} B != "
                             f"table b2f_nodict {m['b2f_nodict']} B")
        dig = hashlib.sha256(payload).hexdigest()
        if dig != m["sha256"]:
            raise ValueError(f"corpus body {m['body']}: sha256 {dig} != table {m['sha256']}")
        out.append((m["name"], payload))
    return out


def materialize_corpus_json(path=CORPUS_JSON, corpus=None, corpus_dir=CORPUS_DIR):
    """(Re)generate the --payload-file manifest {messages:[{name,data_b64}]} from the
    checked-in bodies (unit table by default, or a per-cell SCALED variant list).
    Reproducible + tracked: rerunning yields byte-identical JSON (sorted keys, fixed
    message order == corpus order), so a drift shows up as a git diff."""
    import base64
    msgs = [{"name": nm, "data_b64": base64.b64encode(p).decode("ascii")}
            for nm, p in read_corpus_bodies(corpus, corpus_dir)]
    doc = {"messages": msgs}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, sort_keys=True)
        f.write("\n")
    return path


def verify_corpus_json(path=CORPUS_JSON, corpus=None, corpus_dir=CORPUS_DIR):
    """Assert the corpus.json at `path` decodes to bodies MATCHING the corpus table
    (name-set, per-message bytes, sha256).  Returns a list of failures (empty=OK)."""
    import base64
    corpus = corpus if corpus is not None else CORPUS
    fails = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
    except OSError as e:
        return [f"corpus.json unreadable: {e}"]
    items = doc.get("messages") if isinstance(doc, dict) else doc
    by_name = {m["name"]: m for m in corpus}
    seen = []
    for it in (items or []):
        nm = it.get("name")
        seen.append(nm)
        if nm not in by_name:
            fails.append(f"corpus.json has unknown message {nm!r}")
            continue
        try:
            payload = base64.b64decode(it["data_b64"])
        except Exception as e:  # noqa: BLE001
            fails.append(f"{nm}: data_b64 undecodable ({e})")
            continue
        if len(payload) != by_name[nm]["b2f_nodict"]:
            fails.append(f"{nm}: corpus.json {len(payload)} B != table "
                         f"{by_name[nm]['b2f_nodict']} B")
        if hashlib.sha256(payload).hexdigest() != by_name[nm]["sha256"]:
            fails.append(f"{nm}: corpus.json sha256 != table")
    if set(seen) != set(by_name):
        fails.append(f"corpus.json name-set {sorted(set(seen))} != table {sorted(by_name)}")
    return fails


# ---------------------------------------------------------------------------
# Per-cell SCALED corpus (hole-5).  make_scaled_corpus.py writes
# corpus/scaled/<channel>_<snr>/{<name>.b2f, manifest.json}; the runner reads
# manifest.json here and threads the SESSION corpus (NOT the module unit CORPUS)
# through the gate table / metric / dependency checks so every delivered record's
# {bytes,sha256} binds to the per-variant table.  The unit CORPUS stays the a-priori
# UNIT table (the sizing divisor + the fixtures' 4-message shape); a live scaled run
# swaps in the loaded variant list per cell.
# ---------------------------------------------------------------------------
def _cell_channel_snr(cell):
    """Accept 'WGN:40' / 'WGN_40' / (channel, snr) / a spec dict -> (channel, snr)."""
    if isinstance(cell, dict):
        return str(cell["channel"]).upper(), int(cell["snr"])
    if isinstance(cell, (tuple, list)):
        return str(cell[0]).upper(), int(cell[1])
    s = str(cell).replace("_", ":")
    ch, sn = s.split(":")
    return ch.strip().upper(), int(sn)


def scaled_cell_slug(channel, snr):
    return f"{channel}_{snr}"


def scaled_cell_dir(cell, root=SCALED_DIR):
    ch, sn = _cell_channel_snr(cell)
    return os.path.join(root, scaled_cell_slug(ch, sn))


def scaled_manifest_path(cell, root=SCALED_DIR):
    return os.path.join(scaled_cell_dir(cell, root), "manifest.json")


def scaled_corpus_json_path(cell, root=SCALED_DIR):
    return os.path.join(scaled_cell_dir(cell, root), "corpus.json")


def has_scaled_corpus(cell, root=SCALED_DIR):
    return os.path.exists(scaled_manifest_path(cell, root))


def load_scaled_corpus(cell, root=SCALED_DIR):
    """Load the per-cell SCALED corpus written by make_scaled_corpus.py.  Returns
      {corpus, corpus_dir, sizing_basis, manifest}
    where `corpus` is a variant list in the SAME shape as the module CORPUS (each entry
    {name, template, b2f_nodict, plaintext, sha256, body}) so it drops straight into
    gate_session / metric_session / read_corpus_bodies.  Raises if the manifest is absent
    (a live scaled run REQUIRES it; the caller falls back to the unit CORPUS only where a
    scaled corpus is legitimately not yet generated -- e.g. --dry-run before the pilot)."""
    d = scaled_cell_dir(cell, root)
    mpath = os.path.join(d, "manifest.json")
    with open(mpath, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    corpus = []
    for v in manifest.get("variants", []):
        corpus.append({
            "name": v["name"], "template": v.get("template"),
            "b2f_nodict": v["b2f_nodict"], "b2f_dict": v.get("b2f_dict"),
            "plaintext": v.get("plaintext"), "sha256": v["sha256"], "body": v["body"],
        })
    return {"corpus": corpus, "corpus_dir": d,
            "sizing_basis": manifest.get("sizing_basis", {}), "manifest": manifest}


def materialize_scaled_corpus_json(cell, root=SCALED_DIR):
    """(Re)materialize the per-cell pump payload corpus.json from the scaled bodies
    (reproducible; byte-identical to a fresh generation).  Returns its path."""
    sc = load_scaled_corpus(cell, root)
    return materialize_corpus_json(path=os.path.join(sc["corpus_dir"], "corpus.json"),
                                   corpus=sc["corpus"], corpus_dir=sc["corpus_dir"])


def corpus_for_spec(spec, root=SCALED_DIR):
    """Resolve the SESSION corpus + corpus_dir for a plan spec: the per-cell SCALED
    corpus if generated, else the module UNIT CORPUS (dry-run before the pilot).  Returns
    (corpus, corpus_dir, scaled: bool)."""
    if has_scaled_corpus(spec, root):
        sc = load_scaled_corpus(spec, root)
        return sc["corpus"], sc["corpus_dir"], True
    return CORPUS, CORPUS_DIR, False


# ---------------------------------------------------------------------------
# GATE markers (mechanical refusal).  Parsed from the two Iris --log files.
# Every pattern is exercised by golden_log_selftest() against a captured fixture
# line so a DEAD regex fails the suite (the carrier regex was dead at ~:138).
# ---------------------------------------------------------------------------
# resolved OFDM nfft, both sides must be byte-identical.
#   modem.cc:5933  "[OFDM-NEG] negotiated: cp=%d pilot=%d block=%d nfft=%d"
RE_OFDM_NEG_NFFT = re.compile(r"\[OFDM-NEG\] negotiated:.*\bnfft=(\d+)")
# FULL NEGOTIATED grid (the interop contract): cp, pilot-spacing, block, nfft -- the values
# BOTH ends AGREE on during negotiation and MUST match to decode each other.  The gate
# compares this RESOLVED grid (+ the resolved carrier counts below), NOT the raw per-end
# probe band (RE_PROBE_BAND), which is each end's OWN pre-negotiation passband MEASUREMENT
# and legitimately differs per end.  See the CONFIG_ASYMMETRIC gate in gate_session.
#   modem.cc:5933  "[OFDM-NEG] negotiated: cp=%d pilot=%d block=%d nfft=%d"
RE_OFDM_NEG_GRID = re.compile(
    r"\[OFDM-NEG\] negotiated:\s*cp=(\d+)\s*pilot=(\d+)\s*block=(\d+)\s*nfft=(\d+)")
# per-end RAW probe passband -- each end's OWN pre-negotiation measurement (modem.cc:5848).
#   modem.cc:5848  "Probe complete: band %.0f-%.0f Hz (%.0f Hz BW), center ..."
#   NOTE: 'Probe complete (manual):' (modem.cc:6366) does NOT match (colon placement).
# INFORMATIONAL ONLY: recorded in the gate attrs but NOT gated for config symmetry -- the
# two ends legitimately measure slightly-different passbands (e.g. 325-3989 vs 325-4275 Hz)
# yet negotiate the SAME grid; gating the raw band produced spurious CONFIG_ASYMMETRIC_BAND
# HARNESS_INVALID that withheld otherwise-good cells.  The NEGOTIATED grid governs interop.
RE_PROBE_BAND = re.compile(r"Probe complete:\s*band\s*([\d.]+)-([\d.]+)\s*Hz")
# carrier grid -> carrier-count symmetry.  FIXED (was a dead regex at ~:138):
#   modem.cc:4841 "OFDM PHY: prepared, %d carriers (%d data, %d pilot), CP=%d, BW=%.0f Hz"
RE_OFDM_CARRIERS = re.compile(
    r"OFDM PHY:\s*prepared,\s*(\d+)\s*carriers\s*\(\s*(\d+)\s*data,\s*(\d+)\s*pilot\s*\)")
# WARM cache: a cold session must run a full probe; a cache READ hit is a leak.
#   modem.cc:610 "Probe cache hit for %s"; :611 "Cached probe for ..."
#   modem.cc:6601 "[PROBE-CACHE] applied cached probe for %s (age=...)"  (item 2)
#   modem.cc:6798 "[PROBE-CACHE] loaded from disk: %s (age=...)"
# It must NOT match cache-WRITE lines ("[PROBE-CACHE] cached result for" :6343,
# "[PROBE-CACHE] saved to disk" :6687) -- a cold session legitimately WRITES its
# probe cache; only a READ hit is a violation (asserted in golden_log_selftest).
RE_CACHE_HIT = re.compile(
    r"Probe cache hit for|Cached probe for|"
    r"\[PROBE-CACHE\]\s+applied cached probe|\[PROBE-CACHE\]\s+loaded from disk")
# FORCE-OFDM: the non-deployment lab switch.  We NEVER pass --force-ofdm; this is
# the log belt to the cmdline suspenders.  FIXED to match the ACTUAL bracketed
# marker (item 2):
#   modem.cc:6350 "[FORCE-OFDM] Activating OFDM immediately for %s (no probe)"
#   modem.cc:6418 "[FORCE-OFDM] ACTIVE: %d carriers ..."
RE_FORCE_OFDM = re.compile(
    r"\[FORCE-OFDM\]|force[_-]activate[_-]ofdm|--force-ofdm|force_ofdm", re.IGNORECASE)
# PHY attribution: OFDM must actually have activated + carried bytes.  FIXED to the
# real Iris markers:
#   modem.cc:504  "Native hail: native mode active"
#   modem.cc:4841 "OFDM PHY: prepared, ..."
#   modem.cc:3127+ "[TX-OFDM] ..."   modem.cc:1466+ "[OFDM-RX] ..."
RE_OFDM_ACTIVE = re.compile(
    r"Native hail: native mode active|OFDM PHY:\s*prepared|\[TX-OFDM\]|\[OFDM-RX\]")
# DICT-state: a primed-dict ACTIVE marker.  No such mechanism exists in Iris today
# (item 4), so this never fires -> a dict-OFF cohort is trivially attested off, and
# a dict-ON cohort is refused for lack of a backend (not scored on a false marker).
RE_DICT_ACTIVE = re.compile(r"\[DICT\]\s+primed dictionary active|primed-dict:\s*ACTIVE")


# ===========================================================================
# Plan construction
# ===========================================================================
def cell_tag(channel, snr):
    return f"{channel}{snr}"


def cell_str(channel, snr):
    """Canonical cell label echoed by the bridge C1 attestation and matched by the
    gate.  MUST equal what bridge_cmd passes via --cell."""
    return f"{channel}:{snr}"


def _cell_t_target(cell, root=SCALED_DIR):
    """The per-cell planned transfer T_target (s) -- from the scaled manifest sizing_basis
    if generated, else the a-priori default T_TARGET_S (dry-run before the pilot)."""
    if has_scaled_corpus(cell, root):
        try:
            t = load_scaled_corpus(cell, root)["sizing_basis"].get("T_target_s")
            if t:
                return int(t)
        except (OSError, ValueError, KeyError):
            pass
    return T_TARGET_S


def _cell_session_secs(cell):
    """Per-cell data window = SESSION_SECS_FACTOR * T_target (hole-5: 1.5 x T_target so
    completes finish inside the teardown contract; the connect-inclusive plan wall follows
    via SESSION_WALL_OVERHEAD_S)."""
    return int(round(SESSION_SECS_FACTOR * _cell_t_target(cell)))


def build_plan(n, card_base, boxes, cells=None, cohorts=None, session_secs=None,
               card_span=None, card_wrap=True, port_base0=8100):
    """One entry per (cell, dict-cohort, session).  Each session owns ONE card
    (4 snd-aloop cables) and a disjoint TCP port block; sessions are independent
    and load-immune, so any number run concurrently.  Round-robins sessions
    across the given boxes.  Card indices are ASSIGNED AT DISPATCH by the atomic
    claim (claim_card); the plan only fixes the box + port block + seed.

    session_secs=None -> per-cell 1.5 x T_target (hole-5 SCALING RULE); an explicit value
    pins every cell to that fixed window (test / override)."""
    cells = cells or CELLS
    cohorts = cohorts or [DEFAULT_COHORTS]
    plan = []
    idx = 0
    for (channel, snr) in cells:
        sess = session_secs if session_secs is not None else _cell_session_secs((channel, snr))
        for cohort in cohorts:
            for s in range(n):
                box = boxes[idx % len(boxes)]
                port_base = port_base0 + 20 * (idx % 400)   # disjoint per session
                # (port_base0 must ALSO be disjoint across concurrent runner
                # instances on one box -- same collision class as --scratch-tag)
                plan.append({
                    "idx": idx,
                    "channel": channel,
                    "snr": snr,
                    "cell": cell_str(channel, snr),
                    "cohort": cohort,
                    "metric": DICT_COHORTS[cohort],
                    "profile": PROFILE_FOR[channel],
                    "session": s,
                    "seed": s + 1,                     # deterministic per-session
                    "box": box,
                    # FIX-D(2): the CMD and RSP ends are attested per-END (cross-box).
                    # In the single-box snd-aloop topology both live on `box`, but the
                    # fields make the two-end attestation explicit + future-proof.
                    "cmd_box": box,
                    "rsp_box": box,
                    "box_ip": FLEET[box],
                    "session_secs": sess,
                    # Authoritative CONNECT-INCLUSIVE planned session WALL (s) from the
                    # SENDER's first SABM TX to the last moment a byte can verify.  This --
                    # NOT the session_secs data window, NOT the sender-manifest duration --
                    # is the denominator a PARTIAL/incomplete/zero delivery is scored over
                    # (metric_session).  ALWAYS present in the plan; >= any complete
                    # delivery's wall (see SESSION_WALL_OVERHEAD_S), so a partial can never
                    # out-rank a complete.
                    "planned_session_secs": sess + SESSION_WALL_OVERHEAD_S,
                    "kiss_rsp_port": port_base,
                    "agw_rsp_port": port_base + 1,
                    "kiss_cmd_port": port_base + 2,
                    "agw_cmd_port": port_base + 3,
                    "tag": f"{cell_tag(channel, snr)}_d{cohort}_s{s:02d}",
                    "card_base_hint": card_base,
                    "card_span_hint": card_span if card_span is not None else CARD_SCAN_SPAN,
                    "card_wrap": card_wrap,
                })
                idx += 1
    return plan


# ===========================================================================
# Deployment-path command builders (NEVER --force-ofdm)
# ===========================================================================
def iris_cmd(iris_bin, role, spec, card, subs, home):
    """Build ONE Iris deployment-path invocation.  Returns (argv, env).

    role='RSP' (listener) or 'CMD' (initiator).  Uses:
      --native-hail        AFSK SABM/UA connect, then escalate to native OFDM-KISS
                           (the ONLY deployment path; connect stays AFSK -> legacy
                           compatible with the documented AX.25 line).
      --alsa-raw           raw hw: S32 snd-aloop backend.  The per-direction device
                           strings come from env IRIS_ALSA_RAW_CAPTURE /
                           IRIS_ALSA_RAW_PLAYBACK (audio_alsa_raw.cc:44-56); the CLI
                           takes only ONE optional positional base device
                           (main.cc:343-347) -- so we pass NO positional and set the
                           env instead (fixes the old two-positional bug at ~:229-230).
      fresh HOME -> $HOME/.config/iris caches empty == COLD session.
    NEVER appends --force-ofdm; asserts it below.

    snd-aloop cabling (mirrors arq_realaudio topology; card + 4 substreams):
      CMD  play hw:card,0,S0 (TX)   cap hw:card,1,S3 (RX)
      RSP  cap  hw:card,1,S1 (RX)   play hw:card,0,S2 (TX)
    """
    s0, s1, s2, s3 = subs
    if role == "CMD":
        cap = f"hw:{card},1,{s3}"
        play = f"hw:{card},0,{s0}"
        callsign = "IRISCMD"
        kiss_port = spec["kiss_cmd_port"]
        agw_port = spec["agw_cmd_port"]
    else:
        cap = f"hw:{card},1,{s1}"
        play = f"hw:{card},0,{s2}"
        callsign = "IRISRSP"
        kiss_port = spec["kiss_rsp_port"]
        agw_port = spec["agw_rsp_port"]

    cmd = [iris_bin, "--nogui", "--mode", "A",
           "--callsign", callsign,
           "--native-hail",
           # D2: raw-S32 backend, per-direction device via env (below), no positional.
           "--alsa-raw",
           "--sample-rate", "48000",
           "--port", str(kiss_port), "--agw-port", str(agw_port),
           "--log", f"{home}/iris_{role.lower()}.log"]
    assert "--force-ofdm" not in cmd, "deployment path must never force OFDM"
    env = {
        "HOME": home,
        "IRIS_ALSA_RAW_CAPTURE": cap,     # audio_alsa_raw.cc:44 raw_capture_dev()
        "IRIS_ALSA_RAW_PLAYBACK": play,   # audio_alsa_raw.cc:51 raw_playback_dev()
    }
    # Delivered-rate attack A/B toggles (Modem::init): forward IRIS_MAX_OFDM_LEVEL
    # (increment 2 cap-lift) and IRIS_BURST_FILL (increment 1 window-fill + RSP
    # RR-hold) from the runner's own environment onto BOTH iris ends, so ONE
    # binary drives every arm and the two-sided burst_fill change flips together.
    #
    # CONNECT-TAX DIET A/B toggles (Modem::init, modem.cc:520-535): forward the connshave
    # connect-diet levers so a matched fail-before/pass-after can MEASURE the connect-tax
    # each saves.  IRIS_CONNECT_DIET=0 restores the always-tune arm (~40 s post-probe
    # auto-tune runs on every connect); =1 (default in the binary) skips it on a clean
    # high-SNR probe.  IRIS_TUNE_SKIP_SNR sets the clean-probe rev-SNR gate (dB).
    # IRIS_CLEAN_CLIMB / IRIS_CLEAN_CLIMB_HOLD toggle/tune the faster-climb hold.  All are
    # SNR-gated in the binary, so the fast paths never engage on a marginal link.
    # GRID-DERIVATION A/B (session-reliability, probe_controller.cc): forward
    # IRIS_GRID_AUTHORITATIVE so ONE binary drives both arms — =0 restores the
    # legacy min/max band intersection (the split-grid-prone derivation) for a
    # fail-before reproduction with the [GRID] fingerprint dump present; unset/=1
    # is the default authoritative CMD->RSP derivation.
    # COALESCER-PROBE A/B knobs (all default-inert in the binary): forward the
    # slot-coalescing probe (IRIS_FRAMES_PER_BURST / IRIS_SLOT_AIRTIME_S,
    # modem.cc:615-629), the fixed-level lock (IRIS_FORCE_OFDM_LEVEL,
    # modem.cc:603), and the geometry pin (IRIS_GRID_PIN_NARROW,
    # passband_probe.cc apply_grid_pin: 57-carrier true-narrow grid) onto BOTH
    # iris ends — the grid pin in particular MUST reach both ends or the
    # carrier grids split and the session blacks out.
    for _tog in ("IRIS_MAX_OFDM_LEVEL", "IRIS_BURST_FILL",
                 "IRIS_CONNECT_DIET", "IRIS_TUNE_SKIP_SNR",
                 "IRIS_CLEAN_CLIMB", "IRIS_CLEAN_CLIMB_HOLD",
                 "IRIS_GRID_AUTHORITATIVE",
                 "IRIS_FRAMES_PER_BURST", "IRIS_SLOT_AIRTIME_S",
                 "IRIS_FORCE_OFDM_LEVEL", "IRIS_GRID_PIN_NARROW",
                 "IRIS_TX_LEVEL_CAP"):
        if os.environ.get(_tog) is not None:
            env[_tog] = os.environ[_tog]
    return cmd, env


def box_ctx(fleet_root=DEFAULT_FLEET_ROOT):
    """Exec-context for processes that run ON the fleet box: box python3 + the box
    checkout of the bridge/pump (the bridge imports alsaaudio + sim_channel_relay,
    which exist only on the box)."""
    return {"py": FLEET_PY,
            "fleet_root": fleet_root,
            "bridge": f"{fleet_root}/tools/sim/realaudio/realaudio_bridge_s32.py",
            "pump": f"{fleet_root}/tools/rpi_scripts/kiss_data_pump.py",
            # corpus.json ships inside the iris checkout (HOLE 1: the pump MUST be
            # driven with the real corpus or it sends synthetic records the metric
            # filters out -> gate-clean structural 0.0).  This is the UNIT-table default;
            # run_session overrides "corpus" with the per-cell SCALED corpus.json path
            # (scaled_corpus_box_path) whenever a scaled corpus is generated for the cell.
            "corpus": f"{fleet_root}/iris/tools/corpus/corpus.json",
            "scaled_base": f"{fleet_root}/iris/tools/corpus/scaled"}


def scaled_corpus_box_path(fleet_root, cell):
    """Box path to the per-cell SCALED pump payload (corpus.json)."""
    ch, sn = _cell_channel_snr(cell)
    return f"{fleet_root}/iris/tools/corpus/scaled/{scaled_cell_slug(ch, sn)}/corpus.json"


def bridge_cmd(spec, card, subs, statsfile, ctx):
    """The snd-aloop IONOS bridge for this session.  --cell applies the IONOS
    dial-label -> actual-SNR3k offset (WGN_TO_SNR3K=+2.4): the VARA sheet S:N is
    the COMMANDED IONOS front-panel value (owner 2026-07-02), and the IONOS
    delivers MORE SNR in 3 kHz than the dial reads (firmware AdjustS_N, BANDWIDTH:3000).
    HW CONFIRMED (R7 butler test A): the offset is CREST-FACTOR (PAPR)
    dependent because AdjustS_N references the signal PEAK (ppLPInputMeasAvg): a tone
    realizes +3.5 dB (slope 0.994), high-PAPR OFDM realizes ~+2.4 dB. The runner must
    reproduce VARA's actual channel -> use VARA's offset; VARA FM is high-order-QAM
    OFDM (high PAPR) -> ~+2.4. Keeping +2.4 is CONSERVATIVE: it puts Iris at <= VARA's
    actual SNR in all cases (fair if VARA high-PAPR; Iris run 0-1.1 dB HARSHER if VARA
    lower-PAPR) -> it can never INFLATE Iris. Residual axis caveat +-~1.1 dB; a VARA
    capture through test A would pin VARA's exact offset.

    --cell carries the TRUE channel label {channel}:{snr} (fixes the old hardcoded
    'WGN:{snr}' that mis-attested every MPG cell as WGN); the bridge parses only the
    number after ':' for the offset and takes the channel from --profile, so the
    label is free to be honest -- and the bridge's C1 channel_attestation echoes it
    back for the gate to match against the plan (cell/profile/commanded_snr/seed)."""
    s0, s1, s2, s3 = subs
    return [ctx["py"], ctx["bridge"],
            "--fwd-cap", f"hw:{card},1,{s0}", "--fwd-play", f"hw:{card},0,{s1}",
            "--rev-cap", f"hw:{card},1,{s2}", "--rev-play", f"hw:{card},0,{s3}",
            "--cell", spec["cell"],             # {channel}:{snr}; bridge adds +2.4 offset
            "--profile", spec["profile"],
            "--seed", str(spec["seed"]),
            "--statsfile", statsfile]


def kiss_sender_cmd(spec, home, ctx):
    # HOLE 1: --payload-file makes the sender emit the REAL corpus (b2f_nodict
    # bodies) as integrity-framed records; absent it, synthetic_NNNNNN records
    # verify but metric_session filters them out -> a gate-clean structural 0.0.
    return [ctx["py"], ctx["pump"], "--mode", "sender",
            "--callsign", "IRISCMD", "--remote", "IRISRSP",
            "--kiss-port", str(spec["kiss_cmd_port"]),
            "--payload-file", ctx["corpus"],
            "--duration", str(spec["session_secs"]),
            "--output", f"{home}/kiss_sender.json"]


def kiss_receiver_cmd(spec, home, ctx):
    # --payload-file on the receiver is the expected-set early-exit hint (names are
    # on the wire regardless); it also lets the receiver stop as soon as the whole
    # corpus is hash-verified rather than idling to the duration cap.
    return [ctx["py"], ctx["pump"], "--mode", "receiver",
            "--callsign", "IRISRSP",
            "--kiss-port", str(spec["kiss_rsp_port"]),
            "--payload-file", ctx["corpus"],
            "--duration", str(spec["session_secs"]),
            "--output", f"{home}/kiss_receiver.json"]


def session_processes(spec, iris_bin, card, subs, home, ctx):
    """The ordered ~5-process session (bridge + two iris stacks + relay/pump pair).
    Returns a list of dicts {name, argv, env, log, after_s} in LAUNCH order; the
    teardown order is the reverse.  Mirrors arq_realaudio.py:429-456 sequencing."""
    rsp_argv, rsp_env = iris_cmd(iris_bin, "RSP", spec, card, subs, home)
    cmd_argv, cmd_env = iris_cmd(iris_bin, "CMD", spec, card, subs, home)
    return [
        {"name": "bridge", "argv": bridge_cmd(spec, card, subs, f"{home}/bridge_stats.json", ctx),
         "env": {}, "log": f"{home}/bridge.log", "after_s": SEQ_AFTER_BRIDGE_S},
        {"name": "iris_rsp", "argv": rsp_argv, "env": rsp_env,
         "log": f"{home}/iris_rsp.stdout", "after_s": SEQ_AFTER_RSP_S},
        {"name": "iris_cmd", "argv": cmd_argv, "env": cmd_env,
         "log": f"{home}/iris_cmd.stdout", "after_s": SEQ_AFTER_CMD_S},
        {"name": "kiss_receiver", "argv": kiss_receiver_cmd(spec, home, ctx),
         "env": {}, "log": f"{home}/kiss_receiver.stdout", "after_s": 1.0},
        {"name": "kiss_sender", "argv": kiss_sender_cmd(spec, home, ctx),
         "env": {}, "log": f"{home}/kiss_sender.stdout", "after_s": 0.0},
    ]


# ===========================================================================
# Fleet SSH + atomic snd-aloop card claim (coexists with the Mercury capstone)
# ===========================================================================
def card_name(idx):
    """snd-aloop names the card index in UPPERCASE HEX (parallel_spawner.py:73-83):
    idx 0 -> 'Loopback'; 1..9 -> 'Loopback_1'..'Loopback_9'; 10 -> 'Loopback_A';
    16 -> 'Loopback_10'.  A decimal 'Loopback_10' for idx 10 is WRONG -> ENODEV."""
    return "Loopback" if idx == 0 else "Loopback_%X" % idx


def ssh_argv(box, remote_cmd):
    """SSH invocation for a fleet box.  remote_cmd is a single shell string."""
    return ["ssh", "-i", SSH_KEY, "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=no",
            f"{SSH_USER}@{FLEET[box]}", remote_cmd]


def card_claim_script(base, span=CARD_SCAN_SPAN, claim_dir=FLEET_CLAIM_DIR, wrap=True):
    """A POSIX-sh one-liner (run ON the box) that atomically claims ONE snd-aloop
    card and echoes 'CLAIMED <idx>' or 'NONE'.  Five-question audit lives in the
    DESIGN doc §4.  A card is claimable ONLY if ALL hold:
      (a) it is a Loopback card in /proc/asound/cards;
      (b) it is NOT busy -- none of its PCM substreams read RUNNING (so the live
          Mercury capstone's open substreams are skipped even though the capstone
          does not use this registry -> COEXISTENCE);
      (c) mkdir <claim_dir>/card_<idx> succeeds (atomic; EEXIST => already taken).
    Scans from a HIGH base (DEFAULT_CARD_BASE=12) as a second line of defence
    against the capstone filling from 0.

    wrap=False (--card-span/--no-card-wrap) HARD-BOUNDS the scan to [base, base+span):
    the busy-check alone cannot protect a sibling lane's cards BETWEEN its sessions
    (an idle claim window shows no RUNNING substream), so when the operator has been
    assigned an explicit card range the wrap-to-0 fallback must be off -- claiming
    outside the assigned range is never acceptable, even when the range is full."""
    seqs = f'$(seq {base} {base + span - 1})'
    if wrap:
        seqs += f' $(seq 0 {base - 1})'
    return (
        f'set -e; mkdir -p {claim_dir}; got=""; '
        f'for i in {seqs}; do '
        # (a) is card i a Loopback card?
        f'  if ! grep -q "^ *$i .*Loopback" /proc/asound/cards 2>/dev/null; then continue; fi; '
        # (b) busy-check: any substream RUNNING?  -> skip (capstone or a sibling owns it)
        f'  if grep -qs RUNNING /proc/asound/card$i/pcm*/sub*/status 2>/dev/null; then continue; fi; '
        # (c) atomic claim
        f'  if mkdir {claim_dir}/card_$i 2>/dev/null; then got=$i; break; fi; '
        f'done; '
        f'if [ -n "$got" ]; then echo "CLAIMED $got"; else echo "NONE"; fi')


def card_release_script(idx, claim_dir=FLEET_CLAIM_DIR):
    """Release exactly THIS runner's claim (scoped; never rmdir a sibling's)."""
    return f'rmdir {claim_dir}/card_{idx} 2>/dev/null || true'


def binary_attestation_script(iris_bin, repo="~/iris"):
    """Echo 'MD5 <hex>' and 'HEAD <sha>' for the deployed binary + its repo HEAD,
    so the gate can require md5+HEAD for BOTH ends and prove they ran the same
    build (config-coherence)."""
    return (f'md5sum {shlex.quote(iris_bin)} 2>/dev/null | '
            f'awk \'{{print "MD5 " $1}}\'; '
            f'git -C {shlex.quote(repo)} rev-parse HEAD 2>/dev/null | awk \'{{print "HEAD " $1}}\'')


def _parse_attestation(stdout):
    md5 = head = None
    for line in (stdout or "").splitlines():
        line = line.strip()
        if line.startswith("MD5 "):
            md5 = line[4:].strip()
        elif line.startswith("HEAD "):
            head = line[5:].strip()
    return {"md5": md5, "git_head": head}


def collect_binary_manifest(iris_bin, cmd_box, rsp_box, iris_repo_on_box, ssh=None):
    """Attest the CMD end and the RSP end INDEPENDENTLY -- each from its OWN box --
    so the gate's BINARY_MANIFEST_MISMATCH is not vacuous (FIX-D(2)).

    The old dispatch queried ONE attestation per box and assigned it to BOTH ends
    (run_session's box_binmani[box] = {"CMD": att, "RSP": att}), so CMD and RSP were
    identical BY CONSTRUCTION and a genuine build skew between the two ends could
    never fire.  Attesting per-END (cross-box) makes the comparison real; in the
    single-box snd-aloop topology cmd_box == rsp_box, but each end is still queried
    on its own so a mid-run redeploy on one end is caught.

    ssh(box, remote_cmd) -> (rc, stdout, stderr); defaults to the module _ssh."""
    ssh = ssh or _ssh
    script = binary_attestation_script(iris_bin, repo=iris_repo_on_box)
    _, cmd_out, _ = ssh(cmd_box, script)
    _, rsp_out, _ = ssh(rsp_box, script)
    return {"CMD": _parse_attestation(cmd_out), "RSP": _parse_attestation(rsp_out)}


# ===========================================================================
# The mechanical GATE  (C3: DEFAULT-DENY evidence manifest, split polarity)
# ===========================================================================
# Violation polarity:
#   "harness" => the TEST was unfair/inconsistent -> INVALID, re-run the same seed
#                (force-ofdm, warm-cache, C1 attestation mismatch, config asymmetry,
#                 missing binary/pump evidence).  Excluded from the distribution.
#   "modem"   => the test was FAIR but the modem failed -> score the actual
#                delivered bytes (usually 0) INTO the distribution (NO_OFDM,
#                zero-deliver, activation split-brain).  Counts against activation-rate.
POLARITY_HARNESS = "harness"
POLARITY_MODEM = "modem"

# C1 MEASURED-field gate thresholds (FIX-D(1)).  The bridge attests not just the
# cell LABEL (cell/profile/snr/seed) but the channel it ACTUALLY ran -- those
# measured fields were decorative.  Bound them so a bridge that applied a
# non-standard dial->SNR3k mapping, ran a silent/garbage channel, or was
# underrun-corrupted cannot attest a clean label.
EXPECTED_SNR_OFFSET_DB = 2.4   # == WGN_TO_SNR3K (tools/sim/sim_channel_relay.py:148);
                               # the bridge adds this dial->SNR3k offset for every --cell.
SNR_OFFSET_TOL_DB = 1.5        # documented tolerance around the expected offset (the
                               # bridge docstring's +-~1.1 dB PAPR/axis residual, rounded up).
UNDERRUN_MAX = 32              # startup-transient tolerance; sustained snd-aloop ring
                               # underruns drop audio -> corrupt the channel -> reject.
P_SIG_MIN = 1e-6               # a non-passthrough channel must carry measurable signal power.
P_SIG_MAX = 4.0               # sanity ceiling on realized_p_sig (catches NaN/inf/garbage).


def _last(matches):
    return matches[-1] if matches else None


def _scan(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def _warm_cache_violation(log):
    """True iff `log` shows a WARM-START probe-cache violation: a cache-APPLY
    (RE_CACHE_HIT) that is NOT preceded by this session's OWN genuine cold probe
    ("Probe complete: band ...", RE_PROBE_BAND) in the same log.

    Polarity of the two cases (NIT B, the WARM_CACHE false-positive):
      * A cache-apply with NO preceding cold probe == a warm reconnect that SKIPPED
        its probe using a PRIOR-session cache (HONEST_MEASUREMENT_HARNESS §1.3 trap
        #6) -> VIOLATION.
      * A cache-apply AFTER this session's own "Probe complete:" == an in-session
        RE-HAIL (e.g. an AX.25 T1-timeout multi-window reconnect that finishes the
        transfer) re-applying THIS session's own cold probe -> NOT a violation: the
        probe cost was genuinely paid and is in the wall.
    The per-attempt fresh probe-cache dir (run_session) already guarantees no
    prior-session cache can be READ, so after that fix a cache-apply preceded by a
    cold probe can ONLY be the session's own -- the check stays STRICT (a genuinely
    skipped probe is still caught) while retiring the multi-window false positive."""
    ch = RE_CACHE_HIT.search(log or "")
    if not ch:
        return False                       # no cache-apply at all -> cold
    cp = RE_PROBE_BAND.search(log or "")   # this session's genuine cold-probe completion
    if cp is None:
        return True                        # cache-apply, NO cold probe -> probe skipped (warm)
    return ch.start() < cp.start()         # cache-apply BEFORE any cold probe -> warm


def gate_session(spec, evidence, corpus=None):
    """DEFAULT-DENY evidence manifest.  Returns:
      {"outcome": "SCORED"|"MODEM_FAULT"|"HARNESS_INVALID",
       "violations": [{"code","polarity","detail"}...],
       "attrs": {...}, "activated": bool, "delivered_bytes": int|None}

    A session SCORES a number ONLY on positive attestation of ALL evidence.  Any
    deviation appends a violation; the outcome is decided by polarity precedence
    (any harness-fault => HARNESS_INVALID; else any modem-fault => MODEM_FAULT;
    else SCORED).  Text/JSON in, verdict out; no side effects.

    evidence keys:
      cmd_log, rsp_log        : the two Iris --log texts
      launch_cmdlines         : list of argv lists actually launched (force belt)
      bridge_attestation      : C1 dict, or None
      pump_manifest           : C2 receiver (RX) manifest dict, or None
      sender_manifest         : TX (sender) manifest dict, or None -- carries the
                                first_sabm_tx_epoch the wall anchors on (FIX-A)
      binary_manifest         : {"CMD": {md5,git_head}, "RSP": {md5,git_head}}

    `corpus` is the SESSION corpus table (per-cell SCALED variant list, or the module
    UNIT CORPUS by default) the delivered {bytes,sha256} bind against.
    """
    corpus = corpus if corpus is not None else CORPUS
    v = []
    a = {}

    def viol(code, polarity, detail):
        v.append({"code": code, "polarity": polarity, "detail": detail})

    cmd_log = evidence.get("cmd_log", "") or ""
    rsp_log = evidence.get("rsp_log", "") or ""
    cmdlines = evidence.get("launch_cmdlines", []) or []
    attest = evidence.get("bridge_attestation")
    pump = evidence.get("pump_manifest")
    sender = evidence.get("sender_manifest")
    binmani = evidence.get("binary_manifest") or {}

    # -- (1) force-ofdm: cmdline belt + log suspenders (HARNESS) --
    for cl in cmdlines:
        toks = cl if isinstance(cl, (list, tuple)) else [cl]
        if any("--force-ofdm" in str(t) for t in toks):
            viol("FORCE_OFDM_CMDLINE", POLARITY_HARNESS, "--force-ofdm in a launch cmdline")
            break
    if RE_FORCE_OFDM.search(cmd_log) or RE_FORCE_OFDM.search(rsp_log):
        viol("FORCE_OFDM_LOG", POLARITY_HARNESS, "[FORCE-OFDM]/force_activate_ofdm marker in a log")

    # -- (2) warm-cache: a cold session must run its OWN full probe (HARNESS).  Fire
    #    ONLY on a cache-APPLY with no preceding cold probe (a warm reconnect that
    #    SKIPPED its probe using a prior-session cache, trap #6) -- NOT on an in-session
    #    re-hail (T1-timeout multi-window reconnect) re-applying THIS session's own cold
    #    probe (NIT B: that false positive withheld genuinely-cold complete deliveries).
    #    The per-attempt fresh cache dir guarantees no prior-session cache can be read,
    #    so the check stays strict while retiring the multi-window false positive. --
    if _warm_cache_violation(cmd_log) or _warm_cache_violation(rsp_log):
        viol("WARM_CACHE", POLARITY_HARNESS,
             "probe-cache applied WITHOUT a preceding cold probe (session skipped its probe)")

    # -- (3) bridge channel-attestation C1: present AND matches plan (HARNESS) --
    a["bridge_attestation"] = attest
    if not attest:
        viol("ATTEST_MISSING", POLARITY_HARNESS, "bridge C1 channel_attestation absent")
    else:
        if attest.get("passthrough") is not False:
            viol("ATTEST_PASSTHROUGH", POLARITY_HARNESS,
                 f"bridge ran passthrough={attest.get('passthrough')} (perfect cable, not a channel)")
        if str(attest.get("cell")) != str(spec["cell"]):
            viol("ATTEST_CELL_MISMATCH", POLARITY_HARNESS,
                 f"C1 cell={attest.get('cell')} != plan {spec['cell']}")
        if str(attest.get("profile", "")).upper() != str(spec["channel"]).upper():
            viol("ATTEST_PROFILE_MISMATCH", POLARITY_HARNESS,
                 f"C1 profile={attest.get('profile')} != plan {spec['channel']}")
        if _num(attest.get("commanded_snr")) != _num(spec["snr"]):
            viol("ATTEST_SNR_MISMATCH", POLARITY_HARNESS,
                 f"C1 commanded_snr={attest.get('commanded_snr')} != plan {spec['snr']}")
        if _num(attest.get("seed")) != _num(spec["seed"]):
            viol("ATTEST_SEED_MISMATCH", POLARITY_HARNESS,
                 f"C1 seed={attest.get('seed')} != plan {spec['seed']}")

        # -- (3b) C1 MEASURED fields (FIX-D(1)): gate the channel the bridge ACTUALLY
        #    ran, not just its label.  A wrong dial->SNR3k mapping, a silent/garbage
        #    channel, or an underrun-corrupted session must not attest clean. --
        off = _num(attest.get("realized_snr_offset_db"))
        if not isinstance(off, (int, float)):
            viol("ATTEST_OFFSET_MISSING", POLARITY_HARNESS,
                 "C1 realized_snr_offset_db absent/non-numeric")
        elif abs(off - EXPECTED_SNR_OFFSET_DB) > SNR_OFFSET_TOL_DB:
            viol("ATTEST_SNR_OFFSET_OUT_OF_TOL", POLARITY_HARNESS,
                 f"C1 realized_snr_offset_db={off} outside expected "
                 f"{EXPECTED_SNR_OFFSET_DB}+-{SNR_OFFSET_TOL_DB} dB "
                 f"(bridge applied a non-standard dial->SNR3k mapping)")
        psig = _num(attest.get("realized_p_sig"))
        if attest.get("passthrough") is False:
            # a real (non-passthrough) channel must carry measurable signal power.
            if not isinstance(psig, (int, float)) or not (P_SIG_MIN < psig < P_SIG_MAX):
                viol("ATTEST_P_SIG_INSANE", POLARITY_HARNESS,
                     f"C1 realized_p_sig={attest.get('realized_p_sig')} outside sane "
                     f"({P_SIG_MIN}, {P_SIG_MAX}) -> silent/garbage channel")
        und = attest.get("underruns")
        if und is None:
            viol("ATTEST_UNDERRUNS_MISSING", POLARITY_HARNESS,
                 "C1 underruns absent (bridge ring-health not recorded)")
        elif _num(und) > UNDERRUN_MAX:
            viol("ATTEST_UNDERRUNS", POLARITY_HARNESS,
                 f"C1 underruns={und} > {UNDERRUN_MAX} (audio ring drained -> corrupted channel)")

    # -- (4) binary md5 + git HEAD for BOTH ends, present AND equal (HARNESS) --
    cmd_bin = binmani.get("CMD") or {}
    rsp_bin = binmani.get("RSP") or {}
    a["binary_manifest"] = {"CMD": cmd_bin, "RSP": rsp_bin}
    if not (cmd_bin.get("md5") and cmd_bin.get("git_head")
            and rsp_bin.get("md5") and rsp_bin.get("git_head")):
        viol("BINARY_MANIFEST_MISSING", POLARITY_HARNESS,
             "binary md5+HEAD absent for one/both ends")
    elif cmd_bin["md5"] != rsp_bin["md5"] or cmd_bin["git_head"] != rsp_bin["git_head"]:
        viol("BINARY_MANIFEST_MISMATCH", POLARITY_HARNESS,
             f"CMD {cmd_bin['md5'][:8]}/{cmd_bin['git_head'][:8]} != "
             f"RSP {rsp_bin['md5'][:8]}/{rsp_bin['git_head'][:8]}")

    # -- (5) dict-state matches cohort (HARNESS) --
    dict_active = bool(RE_DICT_ACTIVE.search(cmd_log) or RE_DICT_ACTIVE.search(rsp_log))
    a["dict_active"] = dict_active
    if spec["cohort"] == "off" and dict_active:
        viol("DICT_STATE_MISMATCH", POLARITY_HARNESS, "primed-dict ACTIVE marker in a dict-OFF cohort")
    if spec["cohort"] == "on" and not dict_active:
        # dict-ON has no mechanism in Iris (item 4) -> cannot honestly attest.
        viol("DICT_STATE_UNSUPPORTED", POLARITY_HARNESS,
             "dict-ON cohort but no primed-dict backend in Iris (item 4)")

    # -- (6) OFDM activation + config-tuple symmetry --
    ofdm_cmd = bool(RE_OFDM_ACTIVE.search(cmd_log))
    ofdm_rsp = bool(RE_OFDM_ACTIVE.search(rsp_log))
    activated = ofdm_cmd or ofdm_rsp
    a["ofdm_activated"] = {"cmd": ofdm_cmd, "rsp": ofdm_rsp}

    cmd_nfft = _last(RE_OFDM_NEG_NFFT.findall(cmd_log))
    rsp_nfft = _last(RE_OFDM_NEG_NFFT.findall(rsp_log))
    # FULL negotiated grid (cp, pilot-spacing, block, nfft) -- the OFDM interop contract.
    cmd_grid = _last(RE_OFDM_NEG_GRID.findall(cmd_log))   # (cp,pilot,block,nfft) or None
    rsp_grid = _last(RE_OFDM_NEG_GRID.findall(rsp_log))
    cmd_gridcpb = cmd_grid[:3] if cmd_grid else None      # (cp, pilot-spacing, block)
    rsp_gridcpb = rsp_grid[:3] if rsp_grid else None
    cmd_car = _last(RE_OFDM_CARRIERS.findall(cmd_log))    # (total,data,pilot) resolved grid
    rsp_car = _last(RE_OFDM_CARRIERS.findall(rsp_log))
    # per-end RAW probe band -- INFORMATIONAL ONLY (pre-negotiation measurement; NOT gated).
    cmd_band = _last(RE_PROBE_BAND.findall(cmd_log))
    rsp_band = _last(RE_PROBE_BAND.findall(rsp_log))
    a["nfft"] = {"cmd": cmd_nfft, "rsp": rsp_nfft}
    a["ofdm_neg_grid"] = {"cmd": cmd_grid, "rsp": rsp_grid}   # (cp,pilot,block,nfft)
    a["carriers"] = {"cmd": cmd_car, "rsp": rsp_car}
    a["band"] = {"cmd": cmd_band, "rsp": rsp_band,
                 "note": "per-end RAW probe measurement (pre-negotiation); INFORMATIONAL, "
                         "NOT gated -- the negotiated grid (nfft/cp/pilot/block/carriers) "
                         "governs interop"}

    if not activated:
        viol("NO_OFDM", POLARITY_MODEM,
             "OFDM PHY never activated (AFSK-only / 0-deliver; not an OFDM number)")
    else:
        # For a SCORE the RESOLVED-GRID tuples must be present-AND-equal on BOTH sides.
        # These -- nfft, the (cp, pilot-spacing, block) negotiated grid, and the resolved
        # carrier counts (total/data/pilot) -- are what BOTH ends must AGREE on to decode
        # each other's OFDM (nfft/cp/pilot/carrier grid + pilot layout).  The raw per-end
        # probe BAND is deliberately NOT compared here: it is each end's OWN pre-negotiation
        # passband measurement and legitimately differs per end (e.g. CMD 325-3989 vs RSP
        # 325-4275 Hz) even when the negotiated grid is identical -- gating it produced
        # spurious CONFIG_ASYMMETRIC_BAND HARNESS_INVALID that withheld otherwise-good cells.
        # A GENUINE asymmetry (the nfft-desync bug class) still trips CONFIG_ASYMMETRIC_NFFT
        # / _GRID / _CARRIERS below.
        for label, cval, rval in (("NFFT", cmd_nfft, rsp_nfft),
                                  ("GRID", cmd_gridcpb, rsp_gridcpb),
                                  ("CARRIERS", cmd_car, rsp_car)):
            if cval is not None and rval is not None:
                if cval != rval:
                    # both resolved a config and they DIFFER -> config asymmetry.
                    viol(f"CONFIG_ASYMMETRIC_{label}", POLARITY_HARNESS,
                         f"{label} CMD={cval} != RSP={rval}")
            else:
                # activated but the tuple is missing on one/both sides -> one end
                # never brought up a symmetric session (split-brain) -> MODEM fault.
                viol(f"ACTIVATION_SPLIT_BRAIN_{label}", POLARITY_MODEM,
                     f"{label} present cmd={cval} rsp={rval} (one end did not resolve config)")

    # -- (7) pump C2 delivered-manifest: present, hash-verified, corpus-BOUND --
    delivered_bytes = None
    if pump is None:
        viol("PUMP_MANIFEST_MISSING", POLARITY_HARNESS, "pump C2 delivered manifest absent")
    else:
        delivered_bytes = pump.get("delivered_bytes")
        per_msg = pump.get("per_message") or []
        verified_msgs = [m for m in per_msg if m.get("verified")]
        n_verified = len(verified_msgs)
        a["delivered_bytes"] = delivered_bytes
        a["n_verified_messages"] = n_verified
        if delivered_bytes is None:
            viol("PUMP_MANIFEST_CORRUPT", POLARITY_HARNESS, "C2 delivered_bytes missing")
        elif delivered_bytes <= 0 or n_verified == 0:
            viol("ZERO_DELIVER", POLARITY_MODEM,
                 f"C2 delivered_bytes={delivered_bytes}, verified_msgs={n_verified}")
        else:
            # (7b) corpus provenance + integrity binding (HOLE 3).  A verified
            # record only counts if it is a PLANNED corpus message whose delivered
            # {bytes,sha256} EXACTLY match the checked-in table -- so the wire-parity
            # numerator is bound to the bytes that flew and cannot be flattered by a
            # wrong-corpus materialization.  Polarity:
            #   * a FOREIGN name (e.g. synthetic_NNNNNN) or a bytes/sha MISMATCH means
            #     the harness put the wrong payload on the wire -> HARNESS_INVALID
            #     (this converts the old silent 0.0 into a loud re-run signal);
            #   * all-correct but a strict SUBSET of the planned set == honest modem
            #     UNDER-DELIVERY -> MODEM fault (the actual partial bytes are scored
            #     INTO the distribution; never a re-run loop, never a fake pass).
            table = {m["name"]: m for m in corpus}
            planned = set(table)
            delivered_known = set()
            for m in verified_msgs:
                nm = m.get("name")
                if nm not in table:
                    viol("CORPUS_UNKNOWN_MESSAGE", POLARITY_HARNESS,
                         f"verified message {nm!r} is not a planned corpus message "
                         f"(wrong payload materialized -- e.g. synthetic fallback)")
                    continue
                exp = table[nm]
                if _num(m.get("bytes")) != _num(exp["b2f_nodict"]):
                    viol("CORPUS_BYTES_MISMATCH", POLARITY_HARNESS,
                         f"{nm}: delivered {m.get('bytes')} B != table b2f_nodict "
                         f"{exp['b2f_nodict']} B")
                    continue
                if str(m.get("sha256")) != str(exp["sha256"]):
                    viol("CORPUS_SHA_MISMATCH", POLARITY_HARNESS,
                         f"{nm}: delivered sha256 {str(m.get('sha256'))[:12]}.. != "
                         f"table {exp['sha256'][:12]}..")
                    continue
                delivered_known.add(nm)
            a["delivered_known_names"] = sorted(delivered_known)
            a["corpus_planned_names"] = sorted(planned)
            if delivered_known and delivered_known < planned:
                missing = sorted(planned - delivered_known)
                viol("CORPUS_INCOMPLETE", POLARITY_MODEM,
                     f"delivered {sorted(delivered_known)} != planned {sorted(planned)} "
                     f"(missing {missing}); partial goodput scored into the distribution")

    # -- (8) sender (TX) manifest present with the wall-anchor epoch (HARNESS) --
    #    FIX-A: metric_session anchors the wall on the SENDER's first_sabm_tx_epoch
    #    (which INCLUDES the forward AFSK connect airtime + SABM retries).  Absent it,
    #    metric_session SILENTLY falls back to the RX sabm_rx_epoch (SABM ARRIVAL) and
    #    scores an inflated, connect-airtime-excluding number (wall_anchor=
    #    'rx_sabm_fallback').  aggregate_cell additionally withholds any cell whose
    #    scored sessions fell back (defence in depth).
    #
    #    FIX-2 (re-scope): this violation now fires ONLY for TRUE ABSENCE -- no sender
    #    manifest at all, OR a manifest whose first_sabm_tx_epoch is None (the pump
    #    crashed/was killed BEFORE it ever transmitted a SABM, so no TX-anchored wall
    #    can exist).  A genuine CONNECT-FAILURE is NOT this case: the pump now persists
    #    first_sabm_tx_epoch the instant the first SABM flies (kiss_data_pump.py, even
    #    if the connect then fails / is SIGTERM'd), so a connect-failure carries a valid
    #    TX anchor, passes THIS check, and is governed by its ZERO_DELIVER/NO_OFDM
    #    (POLARITY_MODEM) violations => MODEM_FAULT honest-zero INTO the distribution +
    #    counted in activation_rate, instead of being withheld as HARNESS_INVALID (which
    #    biased the median + activation_rate away from exactly the hard reach cells). --
    a["sender_manifest_present"] = sender is not None
    if not sender or sender.get("first_sabm_tx_epoch") is None:
        viol("SENDER_MANIFEST_MISSING", POLARITY_HARNESS,
             "sender (TX) manifest absent, or present but first_sabm_tx_epoch None -> "
             "the pump never transmitted a SABM (crashed/killed pre-connect); no "
             "TX-anchored wall exists (a genuine connect-FAILURE instead carries the "
             "anchor and scores an honest 0.0 MODEM_FAULT)")

    # -- outcome by polarity precedence --
    polarities = {x["polarity"] for x in v}
    if POLARITY_HARNESS in polarities:
        outcome = "HARNESS_INVALID"
    elif POLARITY_MODEM in polarities:
        outcome = "MODEM_FAULT"
    else:
        outcome = "SCORED"
    return {"outcome": outcome, "violations": v, "attrs": a,
            "activated": activated, "delivered_bytes": delivered_bytes}


def _num(x):
    """Coerce an attestation value to a number for tolerant equality (JSON may
    round-trip ints as strings/floats)."""
    try:
        f = float(x)
        return int(f) if f.is_integer() else f
    except (TypeError, ValueError):
        return x


# ===========================================================================
# Per-session metric (emits ONLY the cohort's own basis - never mixed)
# ===========================================================================
def plan_session_wall_secs(spec):
    """The AUTHORITATIVE connect-inclusive planned session WALL (s) -- the denominator a
    PARTIAL / incomplete / zero delivery is scored over (t1 = t0 + PLANNED_SESSION_SECONDS).

    It comes from the PLAN/spec, NOT from the sender manifest.  Prefers an explicit
    spec['planned_session_secs'] (build_plan stamps it = session_secs + SESSION_WALL_OVERHEAD_S);
    else derives it from spec['session_secs'] + SESSION_WALL_OVERHEAD_S so a spec that carries
    only the raw data window still yields the full connect-inclusive wall.

    Returns None ONLY when the plan carries NEITHER field -- a HARNESS CONFIG error the
    caller must surface LOUDLY (HARNESS_INVALID), NEVER a silent truncated-wall fallback.

    INVARIANT (why a partial can never out-rank a complete, BY CONSTRUCTION):
      planned_wall = session_secs + SESSION_WALL_OVERHEAD_S >= (last_verified - t0) for
      EVERY complete delivery, because the dispatcher tears a session down at
      ~t0 + session_secs + SESSION_TEARDOWN_GRACE_S and only lets bytes verify for up to
      TEARDOWN_MANIFEST_WAIT_S more -- so no delivery's true wall can exceed the planned
      wall.  Then for ANY verified subset (prefix, NON-prefix, corrupted-early, or empty):
        partial_bytes / planned_wall  <=  total_bytes / planned_wall
                                      <=  total_bytes / complete_wall  =  complete_rate
      (first step: partial_bytes <= total_bytes; second: complete_wall <= planned_wall).
      partial_never_outranks_selftest() asserts this over every subset."""
    p = spec.get("planned_session_secs")
    if p is not None:
        return float(p)
    s = spec.get("session_secs")
    if s is not None:
        return float(s) + SESSION_WALL_OVERHEAD_S
    return None


def metric_session(spec, pump_manifest, sender_manifest=None, corpus=None):
    """Return {basis, value_Bmin, reason, wall_secs, connect_probe_climb_secs,
    steady_state_Bmin, wall_anchor, wall_end} for THIS session's single allowed metric
    (chosen by dict
    cohort), consuming the RX C2 pump manifest + the TX (sender) manifest.  The
    numerator is the sum over hash-VERIFIED delivered corpus messages of the cohort's
    checked-in size (b2f_nodict for wire-parity, plaintext for goodput); the
    denominator is a SINGLE, connect-inclusive WHOLE-SESSION wall:
        wall = <wall END> - first_sabm_tx_epoch(SENDER)
    (HOLE 2: the wall START is anchored on the SENDER's first SABM TRANSMIT -- which
    INCLUDES the forward AFSK connect airtime + every SABM retry -- NOT the
    receiver's sabm_rx_epoch, which is SABM ARRIVAL and excludes that airtime,
    inflating Iris.)  If the sender manifest is unavailable the wall START falls back
    to the receiver's sabm_rx_epoch (records wall_anchor='rx_sabm_fallback' so the
    slightly-inflating fallback is VISIBLE).  t0 is the SAME anchor for a complete and a
    partial delivery -- there is one wall definition, not two.

    ROOT FIX (partial-delivery inflation, made robust BY CONSTRUCTION): the wall END
    depends on COMPLETENESS, and the PARTIAL wall comes from the PLAN, never the manifest.
      * A COMPLETE delivery (every planned corpus message hash-verified) finishes when
        its LAST verified byte lands -> t1 = last_verified_byte_epoch is its TRUE finish
        (wall_end='last_verified_byte').
      * A PARTIAL / incomplete / zero delivery is scored over the FULL connect-inclusive
        session wall from the PLAN: t1 = t0 + plan_session_wall_secs(spec)
        (wall_end='full_session_wall').  Because the planned wall >= any complete
        delivery's true wall (plan_session_wall_secs INVARIANT), and partial_bytes <=
        total_bytes, a partial's rate can NEVER exceed a complete's -- for ANY verified
        subset (prefix, NON-prefix, corrupted-early, or empty).  The wall NO LONGER
        depends on the sender-manifest 'duration' field: an absent/garbage duration used
        to (A) omit the connect airtime the complete wall included, letting a non-prefix
        subset out-rank a slow complete, or (B) silently revert a partial to the TRUNCATED
        last-verified wall (~7344 B/min).  Both paths are removed: the plan wall is the
        only source, and a plan that carries no session duration FAILS LOUDLY
        (harness_invalid, scored HARNESS_INVALID by score_session) rather than reverting.

    connect/probe/climb is a SEPARATE line-item (SENDER-first-SABM-TX ->
    RX-first-verified-byte), recorded but NOT netted out of the wall.  Refuses
    (value=None + reason) if a size basis or the wall is missing, rather than
    fabricating one.

    steady_state_Bmin (hole-5 REPORTED NUMBERS) is a POST-FIRST-DELIVERY line-item, NEVER
    compared to the VARA bar and NEVER the headline:
        (SUM verified b2f_nodict - the FIRST-verified message's b2f_nodict)
          / ((last_verified_byte_epoch - first_verified_byte_epoch)/60)
    N/A (None) when <2 messages verified; both epochs on the ONE RX clock (skew-immune);
    it excludes connect+probe+climb AND the first message's transfer so numerator and
    denominator cover the identical span.  `corpus` is the SESSION corpus (per-cell SCALED
    variant list, or the module UNIT CORPUS by default)."""
    corpus = corpus if corpus is not None else CORPUS
    by_name = {m["name"]: m for m in corpus}
    pm = pump_manifest or {}
    wall = pm.get("wall") or {}
    t0 = (sender_manifest or {}).get("first_sabm_tx_epoch")
    wall_anchor = "sender_first_sabm_tx"
    if t0 is None:
        t0 = wall.get("sabm_rx_epoch")     # RX arrival (SABM ARRIVAL, not TX)
        wall_anchor = "rx_sabm_fallback"
    basis = spec["metric"]

    # FIX-B: credit each UNIQUE verified corpus name ONCE (first verified wins) so a
    # duplicate verified record cannot double-count the numerator.  The gate validates
    # delivered names as a SET (a duplicate passes SCORED); the metric must match it or
    # a same-name/same-bytes/same-sha duplicate inflates the number (803.2 vs 742.0).
    # Computed BEFORE the wall so COMPLETENESS can pick the honest wall END (FIX-1).
    delivered = []
    seen = set()
    for m in (pm.get("per_message") or []):
        nm = m.get("name")
        if m.get("verified") and nm in by_name and nm not in seen:
            seen.add(nm)
            delivered.append(nm)
    if not delivered:
        # fall back to the flat name list if per_message omitted (still C2).
        for nm in (pm.get("delivered_msg_names") or []):
            if nm in by_name and nm not in seen:
                seen.add(nm)
                delivered.append(nm)

    # ROOT FIX: the wall END is the delivery's TRUE finish for a COMPLETE delivery (last
    # verified byte); ANY partial / incomplete / zero delivery is scored over the FULL
    # connect-inclusive session wall FROM THE PLAN (t0 + plan_session_wall_secs) -- never
    # the sender-manifest duration, never a truncated last-verified fallback.  This makes
    # "a partial can never out-rank a complete" hold BY CONSTRUCTION for ANY verified
    # subset (see plan_session_wall_secs INVARIANT).
    planned = set(by_name)
    delivered_set = set(delivered)
    complete = bool(delivered_set) and delivered_set >= planned
    last_verified = wall.get("last_verified_byte_epoch")
    if complete:
        wall_end = "last_verified_byte"
        t1 = last_verified
        # METRIC-INTRINSIC OVERRUN GUARD: a COMPLETE delivery's TRUE wall
        # (last_verified - t0) can NEVER exceed the planned session wall BY CONSTRUCTION --
        # the dispatcher tears the session down at t0 + session_secs + SESSION_TEARDOWN_GRACE_S
        # and only lets bytes verify for TEARDOWN_MANIFEST_WAIT_S more, so
        #   last_verified - t0 <= session_secs + SESSION_WALL_OVERHEAD_S = plan_session_wall_secs
        # (NON-STRICT because t0 >= launch_end -> NO epsilon).  If the MEASURED complete wall
        # EXCEEDS the planned wall the teardown contract was VIOLATED (dispatcher schedule
        # change / cross-box clock skew / stale manifest epoch); the wall is untrustworthy --
        # a too-slow "complete" is scored DEFLATED and out-ranked by a fast non-prefix partial
        # (the exact partial-out-ranks-complete failure the plan wall closes on the deflation
        # side).  Do NOT clamp (clamping shortens a slow complete's wall and INFLATES its rate
        # -- the cardinal sin) and do NOT accept the honest ranking (that launders a
        # contract-violating session): FAIL LOUDLY as harness_invalid (score_session maps it ->
        # HARNESS_INVALID on BOTH the SCORED-gate path and the activated-MODEM_FAULT path, the
        # WH4 missing-plan-duration wiring reused).
        planned_wall = plan_session_wall_secs(spec)
        if (planned_wall is not None and t0 is not None and last_verified is not None
                and (float(last_verified) - float(t0)) > planned_wall):
            measured = float(last_verified) - float(t0)
            return {"basis": basis, "value_Bmin": None, "harness_invalid": True,
                    "reason": (f"HARNESS_INVALID: complete delivery measured wall "
                               f"({measured:.1f} s) exceeds planned session wall "
                               f"({planned_wall:.1f} s) -- teardown contract violated "
                               f"(dispatcher schedule change / clock skew / stale manifest "
                               f"epoch); re-run"),
                    "wall_secs": None, "connect_probe_climb_secs": None,
                    "wall_anchor": wall_anchor, "wall_end": wall_end}
    else:
        # PARTIAL / incomplete / zero: full connect-inclusive session wall from the PLAN.
        wall_end = "full_session_wall"
        planned_wall = plan_session_wall_secs(spec)
        if planned_wall is None:
            # step 5: the plan carries NO session duration -> HARNESS CONFIG error.  Fail
            # LOUDLY (score_session maps harness_invalid -> HARNESS_INVALID); there is no
            # silent truncated-wall / manifest-duration fallback left.
            return {"basis": basis, "value_Bmin": None, "harness_invalid": True,
                    "reason": "HARNESS_INVALID: plan carries no session duration "
                              "(spec.planned_session_secs / session_secs) -- cannot define "
                              "the full-session wall for a partial/incomplete/zero delivery",
                    "wall_secs": None, "connect_probe_climb_secs": None,
                    "wall_anchor": wall_anchor, "wall_end": wall_end}
        if t0 is None:
            t1 = None                    # no wall START -> caught by the epochs-missing guard
        else:
            t1 = float(t0) + planned_wall
            # never shorten below a byte that verified during drain (past the planned wall);
            # this only LENGTHENS the wall (lowers the rate) -> can never inflate a partial.
            if last_verified is not None and float(last_verified) > t1:
                t1 = float(last_verified)

    if t0 is None or t1 is None:
        return {"basis": basis, "value_Bmin": None,
                "reason": "wall epochs missing (sender first_sabm_tx / RX "
                          "last_verified_byte)",
                "wall_secs": None, "connect_probe_climb_secs": None,
                "wall_anchor": wall_anchor, "wall_end": wall_end}
    wall_secs = float(t1) - float(t0)
    if wall_secs <= 0:
        return {"basis": basis, "value_Bmin": None,
                "reason": f"non-positive wall ({wall_secs:.3f}s)",
                "wall_secs": wall_secs, "connect_probe_climb_secs": None,
                "wall_anchor": wall_anchor, "wall_end": wall_end}
    wall_min = wall_secs / 60.0

    # connect/probe/climb line-item (separate; RX first-verified-byte epoch, now
    # emitted by the pump -- reconciles the old phantom field).
    cpc = None
    first_deliv = wall.get("first_verified_byte_epoch")
    if first_deliv is not None:
        cpc = round(float(first_deliv) - float(t0), 2)

    # steady_state_Bmin (hole-5 REPORTED NUMBERS) -- POST-FIRST-DELIVERY sustained rate,
    # LINE-ITEM ONLY (never the VARA-compared headline).  Always b2f_nodict basis (the wire
    # load), independent of the cohort.  N/A (None) when <2 messages verified.  Both epochs
    # ride the ONE RX clock (skew-immune).  The FIRST-verified message is EPOCH-anchored via
    # the pump's per-message verified_epoch when present (not list-order-inferred), else
    # falls back to delivery order.
    steady_state_Bmin = _steady_state_bmin(delivered, by_name, wall, pm)

    if basis == "wire_parity":                # dict OFF
        total = 0
        for nm in delivered:
            b = by_name[nm].get("b2f_nodict")
            if b is None:
                return {"basis": basis, "value_Bmin": None,
                        "reason": "b2f_nodict size table empty (DEPENDENCY D3)",
                        "wall_secs": wall_secs, "connect_probe_climb_secs": cpc,
                        "steady_state_Bmin": steady_state_Bmin,
                        "wall_anchor": wall_anchor, "wall_end": wall_end}
            total += b
        return {"basis": basis, "value_Bmin": round(total / wall_min, 1),
                "reason": None, "wall_secs": round(wall_secs, 2),
                "connect_probe_climb_secs": cpc,
                "steady_state_Bmin": steady_state_Bmin,
                "wall_anchor": wall_anchor, "wall_end": wall_end}
    else:                                     # dict ON -> plaintext
        total = sum(by_name[nm].get("plaintext", 0) for nm in delivered)
        return {"basis": basis, "value_Bmin": round(total / wall_min, 1),
                "reason": None, "wall_secs": round(wall_secs, 2),
                "connect_probe_climb_secs": cpc,
                "steady_state_Bmin": steady_state_Bmin,
                "wall_anchor": wall_anchor, "wall_end": wall_end}


def _steady_state_bmin(delivered, by_name, wall, pm):
    """hole-5 post-first-delivery sustained rate (b2f_nodict basis).  None (N/A) when <2
    unique verified corpus messages, or the RX epoch span is non-positive/absent.  NEVER
    compared to the VARA bar.  `delivered` is the ORDERED unique-verified name list."""
    if len(delivered) < 2:
        return None
    # first-verified message: epoch-anchored via per-message verified_epoch when present.
    epoch_by_name = {}
    for m in (pm.get("per_message") or []):
        nm = m.get("name")
        ep = m.get("verified_epoch")
        if m.get("verified") and nm in by_name and ep is not None:
            e = float(ep)
            if nm not in epoch_by_name or e < epoch_by_name[nm]:
                epoch_by_name[nm] = e
    if all(nm in epoch_by_name for nm in delivered):
        first_name = min(delivered, key=lambda n: epoch_by_name[n])
    else:
        first_name = delivered[0]                       # delivery-order fallback
    # span: the RX-clock first/last verified epochs (the literal hole-5 denominator).
    span_start = wall.get("first_verified_byte_epoch")
    span_end = wall.get("last_verified_byte_epoch")
    if span_start is None or span_end is None:
        return None
    span_min = (float(span_end) - float(span_start)) / 60.0
    if span_min <= 0:
        return None
    total_b2f = sum((by_name[nm].get("b2f_nodict") or 0) for nm in delivered)
    first_b2f = by_name[first_name].get("b2f_nodict") or 0
    return round((total_b2f - first_b2f) / span_min, 1)


def score_session(spec, evidence, corpus=None):
    """Combine gate + metric into ONE session row for aggregation.  `corpus` is the SESSION
    corpus (per-cell SCALED variant list, or the module UNIT CORPUS by default).

    outcome semantics for the distribution:
      SCORED         -> a real number; goes into the distribution.
      MODEM_FAULT    -> the modem failed a FAIR test -> 0.0 goes into the
                        distribution (the honest denominator INCLUDES zeros), and
                        it counts against the cell activation-rate.
      HARNESS_INVALID-> excluded; re-run the same seed (bounded).
      REFUSED_DEP    -> gate clean but the numerator basis/wall is unavailable
                        (a dependency refusal, not a measurement) -> excluded.
    """
    gate = gate_session(spec, evidence, corpus=corpus)
    row = {"tag": spec["tag"], "cell": spec["cell"], "cohort": spec["cohort"],
           "seed": spec["seed"], "gate": gate, "activated": gate["activated"]}
    # G1 (arm provenance): EVERY scored row carries the binary content hash(es)
    # from the logs (`build=`, main.cc:33), the attested git HEAD/md5, and the
    # corpus fingerprint - so any later A/B over these rows can be refused when
    # the arms are the same binary or different corpora (the wgn40_auth_arm
    # decoy class).  Additive; never changes the outcome.
    row["provenance"] = mg.session_provenance(
        evidence, corpus_table=(corpus if corpus is not None else CORPUS))
    if gate["outcome"] == "HARNESS_INVALID":
        row["outcome"] = "HARNESS_INVALID"
        row["value_Bmin"] = None
        row["metric"] = None
        return row
    if gate["outcome"] == "MODEM_FAULT":
        # C3: score the ACTUAL delivered bytes (usually 0) INTO the distribution.
        # If OFDM never activated (AFSK-only / NO_OFDM) it is not an OFDM number -> 0.
        # If it activated but split-brained, score whatever hash-verified bytes the
        # pump actually delivered (usually 0) so a partial delivery is not inflated
        # away nor credited as a clean number.
        row["outcome"] = "MODEM_FAULT"
        if not gate["activated"]:
            row["value_Bmin"] = 0.0
            row["metric"] = {"basis": spec["metric"], "value_Bmin": 0.0,
                             "reason": "modem-fault: OFDM never activated (not an OFDM number)"}
        else:
            met = metric_session(spec, evidence.get("pump_manifest"),
                                 evidence.get("sender_manifest"), corpus=corpus)
            if met.get("harness_invalid"):
                # a malformed plan (no session duration) is a HARNESS fault, not a silent
                # modem zero -- surface it loudly so the cell is withheld, never scored.
                row["outcome"] = "HARNESS_INVALID"
                row["value_Bmin"] = None
                row["metric"] = met
                return row
            val = met["value_Bmin"] if met["value_Bmin"] is not None else 0.0
            met["reason"] = (met.get("reason")
                             or "modem-fault: activated but split-brain/zero-deliver/incomplete")
            row["value_Bmin"] = val
            row["metric"] = met
        return row
    # SCORED gate -> compute the metric (may still REFUSE on an unmet dep).
    met = metric_session(spec, evidence.get("pump_manifest"),
                         evidence.get("sender_manifest"), corpus=corpus)
    row["metric"] = met
    if met.get("harness_invalid"):
        # step 5: no silent truncated-wall fallback -- a plan with no session duration is
        # a HARNESS CONFIG error, never a scored number.
        row["outcome"] = "HARNESS_INVALID"
        row["value_Bmin"] = None
    elif met["value_Bmin"] is None:
        row["outcome"] = "REFUSED_DEP"
        row["value_Bmin"] = None
    else:
        row["outcome"] = "SCORED"
        row["value_Bmin"] = met["value_Bmin"]
    return row


# ===========================================================================
# VARA comparison + cell aggregation
# ===========================================================================
def load_vara_bar():
    with open(VARA_BAR_JSON, "r", encoding="utf-8") as f:
        return json.load(f)


def vara_compare(channel, snr, cohort, iris_median_Bmin, withhold, withhold_reason):
    """Return {vara_Bmin, ratio, reason}.  Wire-parity (dict OFF) -> the narrow
    VARA bar bytes/min directly; plaintext (dict ON) -> the bar times VARA's LZHUF
    ratio on the identical corpus.  REFUSES a ratio (ratio=None) when the cell has
    no trustworthy distribution (withhold)."""
    if withhold:
        return {"vara_Bmin": None, "ratio": None, "reason": withhold_reason}
    bar = load_vara_bar()
    try:
        base = bar["bars"]["narrow_3000Hz"][channel][str(snr)]
    except KeyError:
        return {"vara_Bmin": None, "ratio": None,
                "reason": f"no VARA narrow bar for {channel}:{snr}"}
    if cohort == "off":                       # wire-parity, bar is compressed B/min
        vara = base
    else:                                     # plaintext, re-derive from VARA LZHUF
        if VARA_LZHUF_RATIO is None:
            return {"vara_Bmin": None, "ratio": None,
                    "reason": "VARA LZHUF ratio on this corpus unmeasured (DEPENDENCY)"}
        vara = base * VARA_LZHUF_RATIO
    if iris_median_Bmin is None:
        return {"vara_Bmin": round(vara, 1), "ratio": None,
                "reason": "no valid Iris median to compare"}
    return {"vara_Bmin": round(vara, 1),
            "ratio": round(iris_median_Bmin / vara, 4) if vara else None,
            "reason": None}


def distribution(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return {"n": 0, "median": None, "min": None, "max": None, "iqr": None}
    vals_sorted = sorted(vals)
    q = statistics.quantiles(vals_sorted, n=4) if len(vals_sorted) >= 2 else None
    return {"n": len(vals), "median": round(statistics.median(vals_sorted), 1),
            "min": vals_sorted[0], "max": vals_sorted[-1],
            "iqr": ([round(q[0], 1), round(q[2], 1)] if q else None)}


def aggregate_cell(channel, snr, cohort, session_rows, corpus=None):
    """Roll N sessions of one (cell,cohort) into a distribution + VARA compare.

    The honest distribution INCLUDES modem-fault zeros (a fair test the modem lost
    is a real 0, not noise).  HARNESS_INVALID rows are excluded (they were re-run);
    any that remain (re-run budget exhausted) are surfaced, not scored.  activation
    -rate is a first-class line-item = fraction of scored-or-modem-fault sessions
    whose OFDM PHY actually activated + delivered.

    `corpus` is the SESSION corpus table for this cell (per-cell SCALED variant
    list or the unit CORPUS); it feeds the G2 scale block + the G1 corpus
    fingerprint.  All guard fields are ADDITIVE: a cohort that scored before
    still scores the identical numbers."""
    scored = [r for r in session_rows if r["outcome"] == "SCORED"]
    modem = [r for r in session_rows if r["outcome"] == "MODEM_FAULT"]
    harness_invalid = [r for r in session_rows if r["outcome"] == "HARNESS_INVALID"]
    refused = [r for r in session_rows if r["outcome"] == "REFUSED_DEP"]
    in_dist = scored + modem                 # the honest denominator (zeros included)
    values = [r["value_Bmin"] for r in in_dist]
    dist = distribution(values)

    n_activated = sum(1 for r in in_dist if r["activated"])
    activation_rate = round(n_activated / len(in_dist), 4) if in_dist else None

    # FIX-A (defence in depth): surface the wall anchor per session, and treat ANY
    # session that scored on the RX SABM-arrival fallback (sender manifest absent) as
    # non-clean.  With the gate's SENDER_MANIFEST_MISSING this should never reach a
    # scored row, but if it does the whole cell is withheld rather than published on a
    # silently-inflated wall.
    def _anchor(r):
        return (r.get("metric") or {}).get("wall_anchor")
    wall_anchors = sorted({_anchor(r) for r in in_dist if _anchor(r)})
    rx_fallback = [r for r in in_dist if _anchor(r) == "rx_sabm_fallback"]

    # Withhold the VARA ratio only when the distribution can't be trusted: no scored
    # -or-modem sessions at all, an unmet numerator dependency, an unresolved
    # harness-fault (re-run budget exhausted), or an RX-fallback wall (-> not clean).
    withhold = False
    reason = None
    if not in_dist:
        withhold, reason = True, "no scored/modem-fault sessions (all harness-invalid or refused)"
    elif refused:
        withhold, reason = True, f"{len(refused)} session(s) refused on an unmet numerator dependency"
    elif harness_invalid:
        withhold, reason = True, (f"{len(harness_invalid)} session(s) remain HARNESS_INVALID after "
                                  f"re-run budget -> cell not clean")
    elif rx_fallback:
        withhold, reason = True, (f"{len(rx_fallback)} session(s) scored on the RX SABM-arrival wall "
                                  f"fallback (sender manifest absent) -> cell not clean (FIX-A)")
    cmp_ = vara_compare(channel, snr, cohort, dist["median"], withhold, reason)

    cell_dict = {
        "cell": f"{channel}:{snr}", "cohort": cohort, "basis": DICT_COHORTS[cohort],
        "n_total": len(session_rows),
        "n_scored": len(scored), "n_modem_fault": len(modem),
        "n_harness_invalid": len(harness_invalid), "n_refused_dep": len(refused),
        "activation_rate": activation_rate,        # first-class line-item
        "n_activated": n_activated, "n_in_distribution": len(in_dist),
        "distribution_Bmin": dist,
        "connect_probe_climb_secs_median": distribution(
            [r["metric"]["connect_probe_climb_secs"] for r in scored
             if r.get("metric") and r["metric"].get("connect_probe_climb_secs") is not None]
        )["median"],
        # hole-5 transparency line-items (NEVER bar-compared): the POST-FIRST-DELIVERY
        # sustained rate and the connect-tax fraction (cpc/wall).  Reported alongside the
        # headline so a skeptic sees how much of the wall was fixed connect overhead.
        "steady_state_Bmin_median": distribution(
            [r["metric"]["steady_state_Bmin"] for r in scored
             if r.get("metric") and r["metric"].get("steady_state_Bmin") is not None]
        )["median"],
        "connect_tax_pct_median": distribution(
            [round(r["metric"]["connect_probe_climb_secs"] / r["metric"]["wall_secs"], 4)
             for r in scored if r.get("metric")
             and r["metric"].get("connect_probe_climb_secs") is not None
             and r["metric"].get("wall_secs")]
        )["median"],
        "steady_state_label": "post-first-delivery sustained rate (line-item; NOT the VARA-compared headline)",
        "gate_violation_codes": sorted({x["code"] for r in session_rows
                                        for x in r["gate"]["violations"]}),
        "wall_anchors": wall_anchors,               # FIX-A: rx_sabm_fallback => not clean
        "vara_Bmin": cmp_["vara_Bmin"], "vs_vara": cmp_["ratio"],
        "compare_reason": cmp_["reason"],
    }
    # G2 (disproof scale): every cell verdict carries the scale it was measured
    # at, so a negative claim built on it can be floor-checked (the 4-message
    # "DISPROVEN" class).
    cell_dict["scale"] = mg.cell_scale_block(session_rows, corpus_table=corpus)
    # G1 (provenance): the binary content hashes + corpus fingerprint the cell's
    # in-distribution rows actually ran (from the per-row stamps).
    cell_dict["provenance"] = {
        "build_hashes": sorted({b for r in in_dist
                                for b in (r.get("provenance") or {}).get("build_hashes", [])}),
        "git_heads": sorted({h for r in in_dist
                             for h in [(r.get("provenance") or {}).get("git_head")] if h}),
        "corpus_sha256": mg.corpus_fingerprint(corpus),
    }
    # G4 (citability): the ONE VARA-comparable ratio gets a citable stamp + a
    # paste-able provenance line; the steady-state line-item is NEVER citable
    # (the hand-derived "8281 = 0.155x" class is refused by construction).
    cell_dict["vs_vara_citability"] = mg.ratio_citability(cell_dict,
                                                          runner_version=RUNNER_VERSION)
    cell_dict["steady_state_citability"] = dict(mg.LINE_ITEM_NEVER_CITABLE)
    return cell_dict


# ===========================================================================
# Dependency preflight (a real run refuses cohorts whose deps are unmet)
# ===========================================================================
def preflight(iris_bin):
    """Report which delivery dependencies are met.  Never fabricates readiness."""
    dep = {}
    help_txt = ""
    try:
        help_txt = subprocess.run([iris_bin, "--help"], capture_output=True,
                                  text=True, timeout=15).stdout or ""
    except Exception as e:  # noqa: BLE001
        help_txt = f"<iris --help failed: {e}>"
    dep["alsa_raw_backend"] = ("--alsa-raw" in help_txt)             # D2
    dep["corpus_b2f_table"] = all(m["b2f_nodict"] is not None for m in CORPUS)  # D3
    # HOLE 1/3: the checked-in bodies + materialized corpus.json must decode to the
    # table (bytes+sha256) or the pump would send the wrong / synthetic payload.
    dep["corpus_bodies_present"] = all(os.path.exists(_corpus_body_path(m)) for m in CORPUS)
    dep["corpus_json_bound"] = (os.path.exists(CORPUS_JSON) and not verify_corpus_json())
    # hole-5: per-cell SCALED corpora.  Absent cells fall back to the unit corpus for
    # --dry-run; a live scaled baseline REQUIRES all 8 generated (pilot -> make_scaled_corpus).
    dep["scaled_corpora_generated"] = sorted(f"{ch}:{sn}" for (ch, sn) in CELLS
                                             if has_scaled_corpus((ch, sn)))
    dep["scaled_corpora_all_present"] = all(has_scaled_corpus((ch, sn)) for (ch, sn) in CELLS)
    dep["vara_lzhuf_ratio"] = VARA_LZHUF_RATIO is not None
    dep["dict_primed_backend"] = False        # item 4: no primed-dict mechanism in Iris
    # D1 (nfft+A1) is not statically probable from Python; the gate catches its
    # failure signature (config-asymmetric / split-brain / 0-deliver) at run time.
    dep["nfft_A1_fix"] = "runtime-gated (see gate_session)"
    return dep


# ===========================================================================
# GOLDEN-LOG + gate-polarity self-test (a dead regex FAILS the suite)
# ===========================================================================
# Real captured/rendered log lines (from the printf formats cited above).  Each
# POSITIVE line MUST match its regex; NEGATIVE lines MUST NOT.
GOLDEN = {
    "RE_OFDM_CARRIERS": {
        "re": RE_OFDM_CARRIERS,
        "pos": ["OFDM PHY: prepared, 31 carriers (25 data, 6 pilot), CP=64, BW=2344 Hz"],
        "groups": ("31", "25", "6"),
        "neg": [],
    },
    "RE_OFDM_NEG_NFFT": {
        "re": RE_OFDM_NEG_NFFT,
        "pos": ["[OFDM-NEG] negotiated: cp=64 pilot=3 block=13 nfft=1024"],
        "groups": ("1024",),
        "neg": ["[OFDM-NEG] peer config: cp=64 pilot=3 block=13 nfft_code=2"],  # code, not resolved nfft
    },
    "RE_OFDM_NEG_GRID": {
        "re": RE_OFDM_NEG_GRID,
        "pos": ["[OFDM-NEG] negotiated: cp=64 pilot=3 block=13 nfft=1024"],
        "groups": ("64", "3", "13", "1024"),
        "neg": ["[OFDM-NEG] peer config: cp=64 pilot=3 block=13 nfft_code=2"],  # not the negotiated line
    },
    "RE_PROBE_BAND": {
        "re": RE_PROBE_BAND,
        "pos": ["Probe complete: band 300-2700 Hz (2400 Hz BW), center 1500 Hz, baud 800"],
        "groups": ("300", "2700"),
        "neg": ["Probe complete (manual): band 300-2700 Hz (2400 Hz BW), no PHY change"],
    },
    "RE_CACHE_HIT": {
        "re": RE_CACHE_HIT,
        "pos": ["Probe cache hit for IRISRSP — skipping probe",
                "[PROBE-CACHE] applied cached probe for IRISRSP (age=42s, band 300-2700 Hz)",
                "[PROBE-CACHE] loaded from disk: /root/.config/iris/probe_cache/IRISRSP (age=42s, band 300-2700 Hz)"],
        "groups": None,
        # cold-session WRITES must NOT read as warm-cache:
        "neg": ["[PROBE-CACHE] cached result for IRISRSP (band 300-2700 Hz, BW=2400 Hz)",
                "[PROBE-CACHE] saved to disk: /root/.config/iris/probe_cache/IRISRSP"],
    },
    "RE_FORCE_OFDM": {
        "re": RE_FORCE_OFDM,
        "pos": ["[FORCE-OFDM] ACTIVE: 31 carriers (25 data, 6 pilot), nfft=1024, BW=2344 Hz",
                "[FORCE-OFDM] Activating OFDM immediately for IRISRSP (no probe)"],
        "groups": None,
        "neg": ["OFDM PHY: prepared, 31 carriers (25 data, 6 pilot), CP=64, BW=2344 Hz"],
    },
    "RE_OFDM_ACTIVE": {
        "re": RE_OFDM_ACTIVE,
        "pos": ["Native hail: native mode active",
                "OFDM PHY: prepared, 31 carriers (25 data, 6 pilot), CP=64, BW=2344 Hz",
                "[TX-OFDM] speed=O2 QAM16 fec=3/4, 269 bytes, 7 cw/frame, 25 carriers, 4 bits/sym",
                "[OFDM-RX] frame OK: 269 bytes, SNR=12.3 dB, ch_SNR=14.1 dB, 7 LDPC blocks"],
        "groups": None,
        "neg": ["AX.25 connected; staying AFSK"],
    },
}


def golden_log_selftest():
    """Run EVERY gate regex against its captured fixture line.  A dead regex (no
    match on its POSITIVE line) or an over-broad regex (a match on a NEGATIVE line)
    FAILS -> returns a list of failures (empty == pass)."""
    fails = []
    for name, g in GOLDEN.items():
        rx = g["re"]
        for line in g["pos"]:
            m = rx.search(line)
            if not m:
                fails.append(f"{name}: POSITIVE line did not match: {line!r}")
                continue
            if g["groups"] is not None and m.groups() != g["groups"]:
                fails.append(f"{name}: groups {m.groups()} != expected {g['groups']} on {line!r}")
        for line in g["neg"]:
            if rx.search(line):
                fails.append(f"{name}: NEGATIVE line wrongly matched: {line!r}")
    return fails


# --- gate-polarity fixtures --------------------------------------------------
def _fixture_spec(cohort="off", channel="MPG", snr=15, seed=1):
    # session_secs is the raw data window; plan_session_wall_secs() derives the
    # connect-inclusive planned wall (session_secs + SESSION_WALL_OVERHEAD_S) that a
    # partial/incomplete/zero delivery is scored over.  Left as the DERIVED path here
    # (no explicit planned_session_secs) so the derivation is exercised; the explicit
    # build_plan field + the missing-both HARNESS_INVALID case are covered separately.
    return {"tag": f"{channel}{snr}_d{cohort}_s00", "cell": cell_str(channel, snr),
            "channel": channel, "snr": snr, "cohort": cohort, "session_secs": DEFAULT_SESSION_SECS,
            "metric": DICT_COHORTS[cohort], "profile": PROFILE_FOR[channel], "seed": seed}


def _fixture_logs(nfft=("1024", "1024"), band=("300-2700", "300-2700"),
                  carriers=(("31", "25", "6"), ("31", "25", "6")),
                  grid=(("64", "3", "13"), ("64", "3", "13")),
                  activate=(True, True), force=False, warm=False):
    """Render a (cmd_log, rsp_log) pair from real marker formats.  `grid` is the
    per-side (cp, pilot-spacing, block) of the negotiated OFDM grid; `band` is the
    per-side RAW probe passband (informational; the gate no longer compares it)."""
    def side(nf, bd, car, gr, act):
        lines = []
        if act:
            lines.append("Native hail: native mode active")
            lines.append(f"Probe complete: band {bd} Hz (2400 Hz BW), center 1500 Hz, baud 800")
            lines.append(f"[OFDM-NEG] negotiated: cp={gr[0]} pilot={gr[1]} block={gr[2]} nfft={nf}")
            lines.append(f"OFDM PHY: prepared, {car[0]} carriers ({car[1]} data, {car[2]} pilot), CP={gr[0]}, BW=2344 Hz")
            lines.append("[TX-OFDM] speed=O2 QAM16 fec=3/4, 269 bytes, 7 cw/frame, 25 carriers, 4 bits/sym")
        if force:
            lines.append("[FORCE-OFDM] ACTIVE: 31 carriers (25 data, 6 pilot), nfft=1024, BW=2344 Hz")
        if warm:
            lines.append("[PROBE-CACHE] applied cached probe for IRISRSP (age=42s, band 300-2700 Hz)")
        return "\n".join(lines) + "\n"
    return (side(nfft[0], band[0], carriers[0], grid[0], activate[0]),
            side(nfft[1], band[1], carriers[1], grid[1], activate[1]))


def _fixture_c1(spec, passthrough=False, **override):
    # underruns folded in by _bridge_c1_from_stats from the bridge fwd/rev stats
    # (FIX-D(1)); a live dry-run/session yields 0 on a healthy ring.
    a = {"cell": spec["cell"], "profile": spec["channel"], "commanded_snr": spec["snr"],
         "realized_snr_offset_db": 2.4, "seed": spec["seed"], "passthrough": passthrough,
         "realized_p_sig": 0.0223, "underruns": 0}
    a.update(override)
    return a


def _fixture_c2(delivered=("net_checkin", "ics213", "short_email", "batch3"),
                verified=True, sabm_rx=101.0, last_verified=400.0, first_deliv=140.0):
    """A REAL-schema RX (receiver) manifest.  per_message carries the b2f_nodict
    bytes + the checked-in sha256 so the gate's corpus-integrity binding passes on
    the clean fixture (and a wrong byte/name/sha would trip it).  The RX wall holds
    sabm_rx_epoch (SABM ARRIVAL) + last/first_verified_byte_epoch; the TX-anchored
    wall start comes from the SEPARATE sender manifest (_fixture_sender)."""
    by = {m["name"]: m for m in CORPUS}
    per = [{"name": nm, "sha256": by[nm]["sha256"], "verified": verified,
            "bytes": by[nm]["b2f_nodict"]} for nm in delivered]
    dbytes = sum(by[nm]["b2f_nodict"] for nm in delivered) if verified else 0
    return {"delivered_msg_names": list(delivered) if verified else [],
            "delivered_bytes": dbytes,
            "per_message": per,
            "wall": {"sabm_rx_epoch": sabm_rx,
                     "last_verified_byte_epoch": last_verified,
                     "first_verified_byte_epoch": first_deliv}}


def _fixture_sender(first_sabm_tx=100.0, duration=DEFAULT_SESSION_SECS):
    """A REAL-schema TX (sender) manifest carrying the TRUE session wall START
    (first_sabm_tx_epoch).  It STILL carries the pump's 'duration' field (a real pump
    writes it), but the metric wall NO LONGER consumes it -- the partial/zero wall now
    comes from the PLAN (plan_session_wall_secs), so a garbage/missing duration cannot
    move the wall.  Pass duration=None to omit the field entirely (proves independence)."""
    m = {"mode": "sender", "first_sabm_tx_epoch": first_sabm_tx}
    if duration is not None:
        m["duration"] = duration
    return m


def _fixture_bin(md5="a1b2c3d4" * 4, head="d126bb9" + "0" * 33):
    return {"CMD": {"md5": md5, "git_head": head}, "RSP": {"md5": md5, "git_head": head}}


def gate_polarity_selftest():
    """Drive gate_session through the full polarity matrix; assert each fixture
    yields the expected outcome.  Returns a list of failures (empty == pass)."""
    fails = []

    def check(label, spec, ev, want_outcome, want_code=None):
        g = gate_session(spec, ev)
        if g["outcome"] != want_outcome:
            fails.append(f"{label}: outcome={g['outcome']} want {want_outcome} "
                         f"(violations={[x['code'] for x in g['violations']]})")
        if want_code and want_code not in {x["code"] for x in g["violations"]}:
            fails.append(f"{label}: expected violation {want_code} not raised "
                         f"({[x['code'] for x in g['violations']]})")

    sp = _fixture_spec()
    clean_cmd, clean_rsp = _fixture_logs()
    clean_ev = {"cmd_log": clean_cmd, "rsp_log": clean_rsp,
                "launch_cmdlines": [["iris", "--native-hail"]],
                "bridge_attestation": _fixture_c1(sp),
                "pump_manifest": _fixture_c2(),
                "sender_manifest": _fixture_sender(),
                "binary_manifest": _fixture_bin()}

    # 1. clean symmetric -> SCORED
    check("clean_symmetric", sp, clean_ev, "SCORED")

    # 1b. corpus provenance: a FOREIGN (synthetic) verified message -> HARNESS_INVALID
    #     (this is the "synthetic-name zero" the metric used to score as a silent 0.0).
    synth = {"delivered_msg_names": ["synthetic_000000"], "delivered_bytes": 512,
             "per_message": [{"name": "synthetic_000000", "sha256": "00" * 32,
                              "verified": True, "bytes": 512}],
             "wall": {"sabm_rx_epoch": 101.0, "last_verified_byte_epoch": 400.0,
                      "first_verified_byte_epoch": 140.0}}
    check("corpus_foreign_name", sp, {**clean_ev, "pump_manifest": synth},
          "HARNESS_INVALID", "CORPUS_UNKNOWN_MESSAGE")

    # 1c. corpus byte-mismatch (right name, wrong size) -> HARNESS_INVALID
    badbytes = _fixture_c2()
    badbytes["per_message"][0]["bytes"] = 999
    check("corpus_bytes_mismatch", sp, {**clean_ev, "pump_manifest": badbytes},
          "HARNESS_INVALID", "CORPUS_BYTES_MISMATCH")

    # 1d. corpus sha-mismatch (right name+size, wrong hash) -> HARNESS_INVALID
    badsha = _fixture_c2()
    badsha["per_message"][1]["sha256"] = "ff" * 32
    check("corpus_sha_mismatch", sp, {**clean_ev, "pump_manifest": badsha},
          "HARNESS_INVALID", "CORPUS_SHA_MISMATCH")

    # 1e. honest UNDER-delivery (correct subset) -> MODEM_FAULT (scored into dist)
    check("corpus_incomplete", sp,
          {**clean_ev, "pump_manifest": _fixture_c2(delivered=("net_checkin", "ics213"))},
          "MODEM_FAULT", "CORPUS_INCOMPLETE")

    # 2. nfft split -> HARNESS_INVALID (config asymmetry; nfft-desync bug class)
    a, b = _fixture_logs(nfft=("1024", "512"))
    check("nfft_split", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "HARNESS_INVALID", "CONFIG_ASYMMETRIC_NFFT")

    # 3. PER-END BAND DIFFERS but the NEGOTIATED GRID AGREES -> SCORED (Task C fix).
    #    Each end reports its OWN raw pre-negotiation probe passband; those legitimately
    #    differ (CMD 325-3989 vs RSP 325-4275 Hz) while nfft/cp/pilot/block/carriers are
    #    identical -> the ends interop and the cell must SCORE.  FAIL-BEFORE: the old gate
    #    compared the raw band and raised CONFIG_ASYMMETRIC_BAND -> HARNESS_INVALID,
    #    spuriously withholding a good cell.  PASS-AFTER: no band gate -> SCORED.
    a, b = _fixture_logs(band=("325-3989", "325-4275"))
    check("band_per_end_differs_scored", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "SCORED")
    # 3a. and CONFIG_ASYMMETRIC_BAND must NOT be among the violations (the code is retired).
    g_band = gate_session(sp, {**clean_ev, "cmd_log": a, "rsp_log": b})
    if "CONFIG_ASYMMETRIC_BAND" in {x["code"] for x in g_band["violations"]}:
        fails.append("band_per_end_differs_scored: CONFIG_ASYMMETRIC_BAND still raised "
                     "(raw per-end band must NOT be gated)")

    # 3b. GENUINE grid asymmetry (cp/pilot/block desync) -> HARNESS_INVALID.  The resolved
    #     grid IS the interop contract, so a real desync must still be caught even when the
    #     raw band matches -- this is the "still catch a genuine asymmetry" requirement.
    a, b = _fixture_logs(grid=(("64", "3", "13"), ("32", "3", "13")))
    check("grid_split", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "HARNESS_INVALID", "CONFIG_ASYMMETRIC_GRID")

    # 4. carrier split -> HARNESS_INVALID
    a, b = _fixture_logs(carriers=(("31", "25", "6"), ("43", "37", "6")))
    check("carrier_split", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "HARNESS_INVALID", "CONFIG_ASYMMETRIC_CARRIERS")

    # 5. IN-SESSION RE-HAIL (cold probe THEN a cache-apply) -> SCORED, NOT WARM_CACHE.
    #    _fixture_logs(warm=True) renders a genuine "Probe complete: band ..." cold probe
    #    followed by a "[PROBE-CACHE] applied cached probe" line -- exactly a T1-timeout
    #    multi-window reconnect re-applying the session's OWN cold probe.  This is the NIT
    #    B false positive: the session is cold (probe paid, in the wall), so it must SCORE.
    a, b = _fixture_logs(warm=True)
    check("in_session_rehail_scored", sp, {**clean_ev, "cmd_log": a, "rsp_log": b}, "SCORED")

    # 5b. GENUINE WARM START (a cache-apply with NO preceding cold probe = probe SKIPPED
    #     via a prior-session cache, trap #6) -> WARM_CACHE -> HARNESS_INVALID.  This is
    #     what the guard MUST still catch (the refinement narrows to the real trap, it does
    #     not weaken it).  No cold-probe/config markers -> extra activation violations are
    #     expected; harness precedence makes the outcome HARNESS_INVALID with WARM_CACHE.
    warm_trap = ("Native hail: native mode active\n"
                 "[PROBE-CACHE] applied cached probe for IRISRSP (age=612s, band 300-2700 Hz)\n"
                 "Probe cache hit for IRISRSP — skipping probe\n")
    check("warm_cache_trap", sp, {**clean_ev, "cmd_log": warm_trap, "rsp_log": warm_trap},
          "HARNESS_INVALID", "WARM_CACHE")

    # 6. force-ofdm in log -> HARNESS_INVALID
    a, b = _fixture_logs(force=True)
    check("force_ofdm_log", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "HARNESS_INVALID", "FORCE_OFDM_LOG")

    # 7. force-ofdm in cmdline -> HARNESS_INVALID
    check("force_ofdm_cmdline", sp,
          {**clean_ev, "launch_cmdlines": [["iris", "--force-ofdm"]]},
          "HARNESS_INVALID", "FORCE_OFDM_CMDLINE")

    # 8. no-OFDM (AFSK only, both sides) -> MODEM_FAULT
    a, b = _fixture_logs(activate=(False, False))
    check("no_ofdm", sp, {**clean_ev, "cmd_log": a, "rsp_log": b},
          "MODEM_FAULT", "NO_OFDM")

    # 9. activation split-brain (CMD activated, RSP did not; 0 delivered) -> MODEM_FAULT
    a, b = _fixture_logs(activate=(True, False))
    check("split_brain", sp,
          {**clean_ev, "cmd_log": a, "rsp_log": b, "pump_manifest": _fixture_c2(verified=False)},
          "MODEM_FAULT")

    # 10. zero-deliver (pump present, nothing verified) -> MODEM_FAULT
    check("zero_deliver", sp,
          {**clean_ev, "pump_manifest": _fixture_c2(verified=False)},
          "MODEM_FAULT", "ZERO_DELIVER")

    # 11. C1 attestation missing -> HARNESS_INVALID
    check("attest_missing", sp, {**clean_ev, "bridge_attestation": None},
          "HARNESS_INVALID", "ATTEST_MISSING")

    # 12. C1 passthrough -> HARNESS_INVALID
    check("attest_passthrough", sp,
          {**clean_ev, "bridge_attestation": _fixture_c1(sp, passthrough=True)},
          "HARNESS_INVALID", "ATTEST_PASSTHROUGH")

    # 13. C1 cell mismatch (wrong channel echoed) -> HARNESS_INVALID
    check("attest_cell_mismatch", sp,
          {**clean_ev, "bridge_attestation": _fixture_c1(sp, cell="WGN:15", profile="WGN")},
          "HARNESS_INVALID", "ATTEST_CELL_MISMATCH")

    # 14. binary manifest missing -> HARNESS_INVALID
    check("binary_missing", sp, {**clean_ev, "binary_manifest": {}},
          "HARNESS_INVALID", "BINARY_MANIFEST_MISSING")

    # 15. binary manifest mismatch (ends ran different builds) -> HARNESS_INVALID
    bad = {"CMD": {"md5": "aa" * 16, "git_head": "d126bb9" + "0" * 33},
           "RSP": {"md5": "bb" * 16, "git_head": "d126bb9" + "0" * 33}}
    check("binary_mismatch", sp, {**clean_ev, "binary_manifest": bad},
          "HARNESS_INVALID", "BINARY_MANIFEST_MISMATCH")

    # 16. pump manifest missing -> HARNESS_INVALID
    check("pump_missing", sp, {**clean_ev, "pump_manifest": None},
          "HARNESS_INVALID", "PUMP_MANIFEST_MISSING")

    # 17. dict-ON cohort with no backend -> HARNESS_INVALID (unsupported)
    sp_on = _fixture_spec(cohort="on")
    check("dict_on_unsupported", sp_on,
          {"cmd_log": clean_cmd, "rsp_log": clean_rsp,
           "launch_cmdlines": [["iris", "--native-hail"]],
           "bridge_attestation": _fixture_c1(sp_on),
           "pump_manifest": _fixture_c2(), "sender_manifest": _fixture_sender(),
           "binary_manifest": _fixture_bin()},
          "HARNESS_INVALID", "DICT_STATE_UNSUPPORTED")

    # 18. sender (TX) manifest absent -> HARNESS_INVALID (FIX-A: the wall would
    #     silently fall back to the RX SABM-arrival, scoring an inflated number).
    check("sender_manifest_missing", sp, {**clean_ev, "sender_manifest": None},
          "HARNESS_INVALID", "SENDER_MANIFEST_MISSING")
    check("sender_first_sabm_none", sp,
          {**clean_ev, "sender_manifest": {"mode": "sender", "first_sabm_tx_epoch": None}},
          "HARNESS_INVALID", "SENDER_MANIFEST_MISSING")

    # 19. C1 measured field: wrong dial->SNR3k offset -> HARNESS_INVALID (FIX-D(1)).
    check("attest_snr_offset", sp,
          {**clean_ev, "bridge_attestation": _fixture_c1(sp, realized_snr_offset_db=8.0)},
          "HARNESS_INVALID", "ATTEST_SNR_OFFSET_OUT_OF_TOL")

    # 20. C1 measured field: underrun-corrupted channel -> HARNESS_INVALID (FIX-D(1)).
    check("attest_underruns", sp,
          {**clean_ev, "bridge_attestation": _fixture_c1(sp, underruns=9999)},
          "HARNESS_INVALID", "ATTEST_UNDERRUNS")

    # 21. C1 measured field: silent (no signal) channel -> HARNESS_INVALID (FIX-D(1)).
    check("attest_p_sig_insane", sp,
          {**clean_ev, "bridge_attestation": _fixture_c1(sp, realized_p_sig=0.0)},
          "HARNESS_INVALID", "ATTEST_P_SIG_INSANE")

    # --- metric + aggregate end-to-end on the clean fixture ---
    # wall = last_verified(RX=400) - first_sabm_tx(SENDER=100) = 300s = 5 min.
    met = metric_session(sp, _fixture_c2(), _fixture_sender())
    if met["value_Bmin"] is None:
        fails.append(f"metric: clean fixture refused ({met['reason']})")
    else:
        # wire-parity numerator = sum b2f_nodict (3710) over wall 300s = 5 min -> 742.0
        want = round(CORPUS_B2F_NODICT_TOTAL / 5.0, 1)
        if met["value_Bmin"] != want:
            fails.append(f"metric: value {met['value_Bmin']} != expected {want}")
    if met.get("connect_probe_climb_secs") != 40.0:
        fails.append(f"metric: connect_probe_climb {met.get('connect_probe_climb_secs')} != 40.0")
    if met.get("wall_anchor") != "sender_first_sabm_tx":
        fails.append(f"metric: wall_anchor {met.get('wall_anchor')} != sender_first_sabm_tx "
                     f"(HOLE 2: wall must anchor on SENDER first_sabm_tx, not RX arrival)")
    # HOLE 2 fallback: without the sender manifest the wall uses the RX sabm_rx_epoch
    # (101) -> a DIFFERENT (slightly-inflating) wall, flagged via wall_anchor.
    met_fb = metric_session(sp, _fixture_c2())
    if met_fb.get("wall_anchor") != "rx_sabm_fallback":
        fails.append(f"metric: fallback wall_anchor {met_fb.get('wall_anchor')} "
                     f"!= rx_sabm_fallback")

    # FIX-B: a duplicate verified record (same name+bytes+sha) must NOT double-count;
    # the metric credits UNIQUE verified names to match the gate SET (742.0, not 803.2).
    dup = _fixture_c2()
    dup["per_message"].append(dict(dup["per_message"][0]))     # duplicate net_checkin
    met_dup = metric_session(sp, dup, _fixture_sender())
    if met_dup.get("value_Bmin") != met.get("value_Bmin"):
        fails.append(f"metric dedupe: duplicate verified record inflated "
                     f"{met.get('value_Bmin')} -> {met_dup.get('value_Bmin')} (FIX-B double-count)")

    # ROOT FIX: a fast-connect PARTIAL (1-of-4) must be scored over the FULL connect-
    # inclusive session wall FROM THE PLAN (t0 + plan_session_wall_secs), NOT the truncated
    # last-verified wall -- else its 306 B over ~2.5 s (~7344 B/min) out-ranks the complete.
    part = _fixture_c2(delivered=("net_checkin",), sabm_rx=100.5,
                       last_verified=102.5, first_deliv=101.5)
    met_part = metric_session(sp, part, _fixture_sender(first_sabm_tx=100.0, duration=300.0))
    planned_wall = plan_session_wall_secs(sp)           # connect-inclusive, from the plan
    if met_part.get("wall_end") != "full_session_wall":
        fails.append(f"FIX-1: partial wall_end {met_part.get('wall_end')} != full_session_wall "
                     f"(partial not scored over the full session)")
    if met_part.get("value_Bmin") is None or met_part["value_Bmin"] > met.get("value_Bmin"):
        fails.append(f"FIX-1: partial value {met_part.get('value_Bmin')} must be <= complete "
                     f"{met.get('value_Bmin')} (truncated-wall inflation not closed)")
    # 306 B over the connect-inclusive planned wall (300 + 90 = 390 s) = 47.1 B/min
    # (NOT 306/(2.5/60)=7344, and NOT the old 306/(300/60)=61.2 that omitted connect airtime).
    want_part = round(306 / (planned_wall / 60.0), 1)
    if met_part.get("value_Bmin") != want_part:
        fails.append(f"FIX-1: partial value {met_part.get('value_Bmin')} != {want_part} "
                     f"(306 B over the {planned_wall:.0f} s full-session wall)")
    # ROOT FIX: the wall is INDEPENDENT of the sender-manifest 'duration' -- a manifest with
    # NO duration field must yield the SAME plan-anchored wall, never a truncated revert.
    met_nodur = metric_session(sp, part, _fixture_sender(first_sabm_tx=100.0, duration=None))
    if met_nodur.get("value_Bmin") != want_part or met_nodur.get("wall_end") != "full_session_wall":
        fails.append(f"FIX-2: partial with NO manifest duration value "
                     f"{met_nodur.get('value_Bmin')}/{met_nodur.get('wall_end')} != "
                     f"{want_part}/full_session_wall (manifest-duration dependency not removed)")
    # step 5: a plan with NO session duration is a HARNESS CONFIG error, not a silent score.
    sp_noplan = {k: v for k, v in sp.items() if k not in ("session_secs", "planned_session_secs")}
    met_noplan = metric_session(sp_noplan, part, _fixture_sender())
    if not met_noplan.get("harness_invalid") or met_noplan.get("value_Bmin") is not None:
        fails.append(f"step5: partial with NO plan session duration must be harness_invalid "
                     f"(got harness_invalid={met_noplan.get('harness_invalid')}, "
                     f"value={met_noplan.get('value_Bmin')}) -- silent truncated-wall fallback")

    rows = [score_session(sp, clean_ev) for _ in range(6)]
    rows += [score_session(sp, {**clean_ev, "cmd_log": _fixture_logs(activate=(False, False))[0],
                                "rsp_log": _fixture_logs(activate=(False, False))[1]})
             for _ in range(2)]              # 2 modem-fault zeros into the distribution
    agg = aggregate_cell("MPG", 15, "off", rows)
    if agg["n_in_distribution"] != 8:
        fails.append(f"aggregate: n_in_distribution {agg['n_in_distribution']} != 8")
    if agg["activation_rate"] != 0.75:
        fails.append(f"aggregate: activation_rate {agg['activation_rate']} != 0.75 "
                     f"(6 scored + 2 modem-zero)")
    if agg["vs_vara"] is None:
        fails.append("aggregate: vs_vara withheld on a clean+modem-zero cell (should compute)")
    return fails


def partial_never_outranks_selftest():
    """Assert the metric INVARIANT (robust BY CONSTRUCTION): a PARTIAL / incomplete / zero
    delivery can NEVER out-rank a COMPLETE one, for EVERY verified subset -- prefix,
    NON-prefix, corrupted-early (an early message sha-fails while later ones verify), and
    empty.  Drives the REAL metric_session over all 2**N corpus subsets against the SLOWEST
    valid complete (complete_wall == planned_wall -> the lowest complete rate) plus several
    faster completes; a single subset scoring > the complete FAILS.  This is the machine
    proof of plan_session_wall_secs's invariant."""
    import itertools
    fails = []
    sp = _fixture_spec()
    names = [m["name"] for m in CORPUS]
    t0 = 1000.0
    planned_wall = plan_session_wall_secs(sp)
    snd = _fixture_sender(first_sabm_tx=t0)
    # A COMPLETE delivery's TRUE wall (last_verified - t0) is bounded by planned_wall BY
    # CONSTRUCTION; test the worst case (== planned_wall, lowest rate) + a few faster ones.
    for complete_wall in (planned_wall, planned_wall * 0.5, 42.0, 3.0):
        rx_c = _fixture_c2(delivered=tuple(names), sabm_rx=t0 + 0.5,
                           last_verified=t0 + complete_wall, first_deliv=t0 + 1.0)
        met_c = metric_session(sp, rx_c, snd)
        vc = met_c.get("value_Bmin")
        if vc is None:
            fails.append(f"invariant: complete reference refused (wall={complete_wall})")
            continue
        for k in range(len(names) + 1):
            for subset in itertools.combinations(names, k):
                if set(subset) >= set(names):
                    continue                        # the complete itself (equal, not out-rank)
                # fast-connect partial (the HEAD-inflation shape): last byte verified early.
                rx_p = _fixture_c2(delivered=subset, verified=bool(subset),
                                   sabm_rx=t0 + 0.5, last_verified=t0 + 2.5,
                                   first_deliv=t0 + 1.5)
                met_p = metric_session(sp, rx_p, snd)
                vp = met_p.get("value_Bmin")
                if not subset or vp is None:
                    continue                        # zero delivery / refusal never inflates
                if met_p.get("wall_end") != "full_session_wall":
                    fails.append(f"invariant: partial {sorted(subset)} wall_end "
                                 f"{met_p.get('wall_end')} != full_session_wall")
                if vp > vc:
                    fails.append(f"invariant: partial {sorted(subset)}={vp} B/min > complete "
                                 f"{vc} B/min (complete_wall={complete_wall}) -- a partial "
                                 f"OUT-RANKED a complete")

    # METRIC-INTRINSIC OVERRUN GUARD: the invariant above holds ONLY while
    # the teardown contract does (complete_wall <= planned_wall).  If a COMPLETE delivery's
    # MEASURED wall EXCEEDS planned_wall (dispatcher schedule change / clock skew / stale
    # manifest epoch) that contract is broken -- the deflated complete would be out-ranked by a
    # fast non-prefix partial.  The metric must catch this DEFLATION side intrinsically: a
    # complete with measured wall 590 s against a planned_wall of 390 s must be UNSCORED
    # (harness_invalid), NOT a deflated ~377.3 B/min.  (Before the guard metric_session scores
    # 3710 B / (590/60) = 377.3, which a fast non-prefix {ics213,short_email,batch3}=3404 B
    # partial out-ranks at 3404 / (390/60) = 523.7 B/min; after the guard it is harness_invalid.)
    overrun_wall = planned_wall + 200.0                       # 390 + 200 = 590 s measured wall
    rx_over = _fixture_c2(delivered=tuple(names), sabm_rx=t0 + 0.5,
                          last_verified=t0 + overrun_wall, first_deliv=t0 + 1.0)
    met_over = metric_session(sp, rx_over, snd)
    if not met_over.get("harness_invalid") or met_over.get("value_Bmin") is not None:
        fails.append(f"overrun-guard: complete measured wall {overrun_wall:.0f} s > planned "
                     f"{planned_wall:.0f} s must be harness_invalid (got "
                     f"harness_invalid={met_over.get('harness_invalid')}, "
                     f"value={met_over.get('value_Bmin')}) -- a deflated complete would be "
                     f"out-ranked by a fast partial (teardown contract violated)")
    # a WITHIN-contract complete (measured wall == planned_wall, the worst valid case) must
    # STILL score -- the guard is non-strict and must not false-trip the boundary.
    rx_edge = _fixture_c2(delivered=tuple(names), sabm_rx=t0 + 0.5,
                          last_verified=t0 + planned_wall, first_deliv=t0 + 1.0)
    met_edge = metric_session(sp, rx_edge, snd)
    if met_edge.get("harness_invalid") or met_edge.get("value_Bmin") is None:
        fails.append(f"overrun-guard: a within-contract complete (measured wall == planned "
                     f"{planned_wall:.0f} s) must SCORE, not trip the guard (got "
                     f"harness_invalid={met_edge.get('harness_invalid')}, "
                     f"value={met_edge.get('value_Bmin')})")
    return fails


def steady_state_selftest():
    """hole-5 steady_state_Bmin (post-first-delivery sustained rate) computes correctly on a
    synthetic multi-message manifest, is N/A for <2 verified, and picks the first-verified
    message EPOCH-anchored (not list-order-inferred).  NEVER a bar-compared number."""
    fails = []
    sp = _fixture_spec()
    by = {m["name"]: m for m in CORPUS}          # 306 / 792 / 910 / 1702
    snd = _fixture_sender(first_sabm_tx=990.0)

    def rx(arrival, first_v, last_v, with_epoch=True):
        """arrival: [(name, verified_epoch), ...] in RX arrival order."""
        per = []
        for (n, e) in arrival:
            rec = {"name": n, "sha256": by[n]["sha256"], "verified": True,
                   "bytes": by[n]["b2f_nodict"]}
            if with_epoch:
                rec["verified_epoch"] = e
            per.append(rec)
        return {"delivered_msg_names": [n for n, _ in arrival],
                "delivered_bytes": sum(by[n]["b2f_nodict"] for n, _ in arrival),
                "per_message": per,
                "wall": {"sabm_rx_epoch": 995.0,
                         "first_verified_byte_epoch": first_v,
                         "last_verified_byte_epoch": last_v}}

    # (1) 4 verified in order: (3710 - 306) / ((1030-1000)/60) = 3404 / 0.5 = 6808.0
    m1 = metric_session(sp, rx([("net_checkin", 1000), ("ics213", 1010),
                                ("short_email", 1020), ("batch3", 1030)], 1000, 1030), snd)
    if m1.get("steady_state_Bmin") != 6808.0:
        fails.append(f"steady_state (4 in-order) {m1.get('steady_state_Bmin')} != 6808.0")

    # (2) <2 verified -> N/A (None)
    m2 = metric_session(sp, rx([("net_checkin", 1000)], 1000, 1000), snd)
    if m2.get("steady_state_Bmin") is not None:
        fails.append(f"steady_state (<2 verified) {m2.get('steady_state_Bmin')} != None")

    # (3) EPOCH-anchored first: the earliest-epoch msg (net_checkin@1000) is NOT list-first
    #     -> its 306 B is the subtracted first, so (306+792+910 - 306)/0.5 = 3404.0
    #     (list-order would wrongly subtract ics213's 792 -> 2432.0).
    m3 = metric_session(sp, rx([("ics213", 1005), ("net_checkin", 1000),
                                ("short_email", 1030)], 1000, 1030), snd)
    if m3.get("steady_state_Bmin") != 3404.0:
        fails.append(f"steady_state (epoch-anchored first) {m3.get('steady_state_Bmin')} "
                     f"!= 3404.0 (not epoch-anchored)")

    # (4) list-order fallback when per-message verified_epoch is absent (older pump).
    m4 = metric_session(sp, rx([("net_checkin", 0), ("ics213", 0), ("short_email", 0)],
                               1000, 1030, with_epoch=False), snd)
    if m4.get("steady_state_Bmin") != 3404.0:
        fails.append(f"steady_state (list-order fallback) {m4.get('steady_state_Bmin')} != 3404.0")

    # (5) zero RX span -> N/A (never divide by zero / inflate)
    m5 = metric_session(sp, rx([("net_checkin", 1000), ("ics213", 1000)], 1000, 1000), snd)
    if m5.get("steady_state_Bmin") is not None:
        fails.append(f"steady_state (zero span) {m5.get('steady_state_Bmin')} != None")
    return fails


def distinct_variant_crediting_selftest():
    """A per-cell SCALED corpus of N DISTINCT rerolled variants credits EACH unique variant
    ONCE (no cross-variant flattering), binds delivered {bytes,sha256} to the SESSION corpus
    (a UNIT name is now FOREIGN), and a duplicate variant record does not double-count."""
    fails = []
    sp = _fixture_spec()          # cohort off -> wire_parity (b2f_nodict basis)
    snd = _fixture_sender(first_sabm_tx=970.0)
    # 8 byte-distinct variants (2 sets x 4 templates); sizes/shas self-consistent.
    scaled = []
    for s in range(2):
        for base in ("net_checkin", "ics213", "short_email", "batch3"):
            nm = f"{base}_s{s:02d}"
            sz = 300 + 17 * len(scaled)                       # distinct per variant
            scaled.append({"name": nm, "template": base, "b2f_nodict": sz,
                           "plaintext": sz + 80,
                           "sha256": hashlib.sha256(nm.encode()).hexdigest(),
                           "body": nm + ".b2f"})
    total = sum(v["b2f_nodict"] for v in scaled)

    def rx(variants):
        per = [{"name": v["name"], "sha256": v["sha256"], "verified": True,
                "bytes": v["b2f_nodict"], "verified_epoch": 1000.0 + i}
               for i, v in enumerate(variants)]
        return {"delivered_msg_names": [v["name"] for v in variants],
                "delivered_bytes": sum(v["b2f_nodict"] for v in variants),
                "per_message": per,
                "wall": {"sabm_rx_epoch": 975.0, "first_verified_byte_epoch": 1000.0,
                         "last_verified_byte_epoch": 1030.0}}

    clean_cmd, clean_rsp = _fixture_logs()

    def ev(pm):
        return {"cmd_log": clean_cmd, "rsp_log": clean_rsp,
                "launch_cmdlines": [["iris", "--native-hail"]],
                "bridge_attestation": _fixture_c1(sp),
                "pump_manifest": pm, "sender_manifest": snd,
                "binary_manifest": _fixture_bin()}

    # all 8 distinct variants delivered -> SCORED (complete); wall = 1030-970 = 60 s = 1 min
    # so the wire-parity value == the full distinct total (every variant credited once).
    full = rx(scaled)
    g = gate_session(sp, ev(full), corpus=scaled)
    if g["outcome"] != "SCORED":
        fails.append(f"distinct: 8-variant gate {g['outcome']} != SCORED "
                     f"({[v['code'] for v in g['violations']]})")
    met = metric_session(sp, full, snd, corpus=scaled)
    if met.get("value_Bmin") != float(total):
        fails.append(f"distinct: value {met.get('value_Bmin')} != total {float(total)} "
                     f"(every distinct variant must be credited once over the 60 s wall)")

    # a DUPLICATE variant record must NOT double-count (mirrors FIX-B for the scaled table).
    dup = rx(scaled)
    dup["per_message"].append(dict(dup["per_message"][0]))
    met_dup = metric_session(sp, dup, snd, corpus=scaled)
    if met_dup.get("value_Bmin") != met.get("value_Bmin"):
        fails.append(f"distinct: duplicate variant inflated {met.get('value_Bmin')} -> "
                     f"{met_dup.get('value_Bmin')} (FIX-B not honored for scaled corpus)")

    # binding uses the SESSION corpus: a UNIT-table name is now FOREIGN -> HARNESS_INVALID.
    foreign = rx(scaled)
    foreign["per_message"].append({"name": "net_checkin", "sha256": CORPUS[0]["sha256"],
                                   "verified": True, "bytes": CORPUS[0]["b2f_nodict"],
                                   "verified_epoch": 1031.0})
    foreign["delivered_msg_names"].append("net_checkin")
    gf = gate_session(sp, ev(foreign), corpus=scaled)
    codes = {v["code"] for v in gf["violations"]}
    if gf["outcome"] != "HARNESS_INVALID" or "CORPUS_UNKNOWN_MESSAGE" not in codes:
        fails.append(f"distinct: a UNIT name delivered under a SCALED corpus must be FOREIGN "
                     f"(got {gf['outcome']} {sorted(codes)})")
    return fails


def measurement_guards_selftest():
    """G1-G4 (tools/measurement_guards.py) + their runner integration.

    The four guard self-tests are keyed to the REAL historical artifacts
    (checked-in harvests under tools/guard_fixtures/): the wgn40_auth_arm
    byte-identical decoy arm (G1), the 4-message-corpus "DISPROVEN" scale
    (G2), the p=0.6875 anchor-fix A/B pairs (G3), and the remeasure verdict
    whose withheld cell leaked "8281 = 0.155x" by hand (G4).  The integration
    half asserts the runner stamps rows/cells with the guard fields WITHOUT
    changing any previously-scored number."""
    fails = []
    for name, fl in mg.run_self_tests().items():
        fails += [f"{name}: {f}" for f in fl]

    # --- runner integration: guard fields are stamped, old numbers unchanged ---
    sp = _fixture_spec()
    clean_cmd, clean_rsp = _fixture_logs()
    clean_ev = {"cmd_log": "[     0.006] Iris FM Data Modem v0.2  build=ab12cd34\n" + clean_cmd,
                "rsp_log": "[     0.006] Iris FM Data Modem v0.2  build=ab12cd34\n" + clean_rsp,
                "launch_cmdlines": [["iris", "--native-hail"]],
                "bridge_attestation": _fixture_c1(sp),
                "pump_manifest": _fixture_c2(),
                "sender_manifest": _fixture_sender(),
                "binary_manifest": _fixture_bin()}
    rows = [score_session(sp, clean_ev) for _ in range(6)]
    a, b = _fixture_logs(activate=(False, False))
    rows += [score_session(sp, {**clean_ev, "cmd_log": a, "rsp_log": b}) for _ in range(2)]
    # G1 row stamp: build= extracted from the log, corpus fingerprint bound.
    prov = rows[0].get("provenance") or {}
    if prov.get("build_hashes") != ["ab12cd34"]:
        fails.append(f"integration: row build_hashes {prov.get('build_hashes')} != "
                     f"['ab12cd34'] (G1 stamp missing/wrong)")
    if prov.get("corpus_sha256") != mg.corpus_fingerprint(CORPUS):
        fails.append("integration: row corpus_sha256 != fingerprint(CORPUS)")
    # pre-guard behavior UNCHANGED: same outcomes, same values, same vs_vara.
    agg = aggregate_cell("MPG", 15, "off", rows, corpus=CORPUS)
    if agg["n_in_distribution"] != 8 or agg["activation_rate"] != 0.75:
        fails.append(f"integration: aggregate outcomes moved (n_in_dist="
                     f"{agg['n_in_distribution']}, act={agg['activation_rate']})")
    if agg["distribution_Bmin"]["median"] != 742.0:
        fails.append(f"integration: clean-fixture median {agg['distribution_Bmin']['median']} "
                     f"!= 742.0 (guards must not move scored numbers)")
    if agg["vs_vara"] is None:
        fails.append("integration: vs_vara withheld on a clean cell (guards must be additive)")
    # G2 cell scale block.
    sc = agg.get("scale") or {}
    if (sc.get("n_sessions") != 8 or sc.get("messages_offered") != len(CORPUS)
            or sc.get("corpus_bytes") != CORPUS_B2F_NODICT_TOTAL):
        fails.append(f"integration: cell scale block wrong ({sc})")
    # G4 citability: clean cell cites, with the denominator named in provenance.
    cit = agg.get("vs_vara_citability") or {}
    if cit.get("citable") is not True:
        fails.append(f"integration: clean cell not citable ({cit.get('reason')})")
    if f"vara_mpg15_Bmin={agg['vara_Bmin']}" not in (cit.get("provenance") or ""):
        fails.append(f"integration: provenance lacks named denominator "
                     f"({cit.get('provenance')})")
    if (agg.get("steady_state_citability") or {}).get("citable") is not False:
        fails.append("integration: steady-state line-item not stamped never-citable")
    # a WITHHELD cell must stamp citable=False with the withhold reason.
    bad = score_session(sp, {**clean_ev, "bridge_attestation": None})   # HARNESS_INVALID
    agg2 = aggregate_cell("MPG", 15, "off", rows + [bad], corpus=CORPUS)
    cit2 = agg2.get("vs_vara_citability") or {}
    if cit2.get("citable") is not False or "HARNESS_INVALID" not in (cit2.get("reason") or ""):
        fails.append(f"integration: withheld cell citability wrong ({cit2})")
    # G2 writer: the runner's own summary must be emittable (has scale) and a
    # scale-less verdict must be REFUSED at write time.
    try:
        mg.emit_verdict(os.devnull, {"outcome": "MEASUREMENT"})
        fails.append("integration: emit_verdict accepted a scale-less verdict")
    except mg.GuardError:
        pass
    return fails


def run_self_tests():
    g = golden_log_selftest()
    p = gate_polarity_selftest()
    inv = partial_never_outranks_selftest()
    return g, p, inv


def corpus_selftest():
    """Assert the checked-in UNIT bodies + corpus.json are internally consistent with the
    CORPUS table (name-set, bytes==b2f_nodict, sha256), then validate EVERY generated
    per-cell SCALED corpus.  A drift FAILS the suite."""
    fails = list(verify_corpus_json())
    try:
        bodies = dict(read_corpus_bodies())
    except (OSError, ValueError) as e:
        return fails + [f"corpus bodies: {e}"]
    for m in CORPUS:
        if m["name"] not in bodies:
            fails.append(f"corpus body missing for {m['name']}")
    fails += corpus_selftest_scaled()
    return fails


def heldout_disjointness_selftest():
    """C3 held-out discipline: assert the shipped priming dict (set A) does NOT memorize any
    set-B message body -- the longest shared byte run between each corpus plaintext and the
    dict must be <= HELDOUT_MAX_SHARED_RUN (universal header boilerplate is ~96 B; body
    memorization is ~140 B).  Also binds each plaintext to its recorded sha256.  A published
    plaintext-axis ratio measured on a body the dict memorized would be train-on-test (the
    original 916-B representative dict did exactly that on net_checkin -> edge inflated 3.16)."""
    fails = []
    try:
        with open(DICT_TXT, "rb") as f:
            dic = f.read()
    except OSError as e:
        return [f"heldout: dict {DICT_TXT} unreadable ({e})"]
    for m in CORPUS:
        pf = m.get("plaintext_file")
        if not pf:
            fails.append(f"heldout: {m['name']} has no plaintext_file (set B undefined)")
            continue
        try:
            with open(os.path.join(CORPUS_DIR, pf), "rb") as f:
                pt = f.read()
        except OSError as e:
            fails.append(f"heldout: {m['name']} plaintext {pf} unreadable ({e})")
            continue
        if len(pt) != m["plaintext"]:
            fails.append(f"heldout: {m['name']} plaintext {len(pt)} B != table {m['plaintext']} B")
        dig = hashlib.sha256(pt).hexdigest()
        if dig != m.get("plaintext_sha256"):
            fails.append(f"heldout: {m['name']} plaintext sha256 {dig} != table {m.get('plaintext_sha256')}")
        run = _longest_shared_run(pt, dic)
        if run > HELDOUT_MAX_SHARED_RUN:
            fails.append(f"heldout: {m['name']} shares a {run} B run with the dict "
                         f"(> {HELDOUT_MAX_SHARED_RUN}) -- TRAIN-ON-TEST, dict memorized the body")
    return fails


def primed_delivered_axis_selftest():
    """C4 delivered-plaintext axis: assert the DERIVED per-message primed edge is honest --
    every b2f_dict is a real shrink over b2f_nodict (primed < nodict) and <= plaintext, the
    aggregate edge equals PRIMED_EDGE_HELDOUT, and the wire-parity dict-OFF headline
    (VARA_LZHUF_RATIO from b2f_nodict) is UNCHANGED by the dict-ON axis."""
    fails = []
    for m in CORPUS:
        bd, bn = m.get("b2f_dict"), m.get("b2f_nodict")
        if not bd or not bn:
            fails.append(f"axis: {m['name']} missing b2f_dict/b2f_nodict")
            continue
        if not (0 < bd <= bn):
            fails.append(f"axis: {m['name']} primed b2f_dict {bd} not a shrink over nodict {bn}")
        if bd > m["plaintext"]:
            fails.append(f"axis: {m['name']} primed b2f_dict {bd} > plaintext {m['plaintext']}")
    edge = primed_delivered_edge()
    if edge["aggregate"] != PRIMED_EDGE_HELDOUT:
        fails.append(f"axis: aggregate edge {edge['aggregate']} != PRIMED_EDGE_HELDOUT {PRIMED_EDGE_HELDOUT}")
    if round(CORPUS_PLAINTEXT_TOTAL / CORPUS_B2F_NODICT_TOTAL, 2) != VARA_LZHUF_RATIO:
        fails.append("axis: wire-parity/VARA_LZHUF_RATIO basis drifted (headline must stay dict-OFF)")
    return fails


def _recompute_n_sets(sb):
    """Recompute N_sets from a manifest sizing_basis (the a-priori clamp) to prove the rule
    was applied honestly -- kept in sync with make_scaled_corpus.compute_n_sets."""
    import math
    unit = sb.get("unit_set_b2f_nodict", 3710)
    floor = sb.get("floor", N_SETS_FLOOR)
    cap = sb.get("cap", N_SETS_CAP)
    span = max(0.0, float(sb["T_target_s"]) - float(sb["C_est"]))
    raw_f = float(sb["R_ss_est"]) * span / float(unit)
    raw = int(math.ceil(raw_f)) if raw_f > 0 else 0
    return max(floor, min(cap, raw)), raw


def corpus_selftest_scaled():
    """Validate every generated per-cell SCALED corpus: bodies match the manifest table
    (len==b2f_nodict, sha256), corpus.json (if materialized) binds, variants are byte-DISTINCT
    (no cross-message dedup can flatter Iris), and the sizing is auditable (N_sets in
    [floor,cap], n_variants==4*N_sets, totals consistent, and N_sets re-derives from the
    recorded R_ss_est/C_est).  No scaled corpora generated yet -> no-op (empty)."""
    fails = []
    for (channel, snr) in CELLS:
        if not has_scaled_corpus((channel, snr)):
            continue
        cell = f"{channel}:{snr}"
        try:
            sc = load_scaled_corpus((channel, snr))
        except (OSError, ValueError, KeyError) as e:
            fails.append(f"scaled {cell}: manifest unreadable ({e})")
            continue
        corpus, cdir, sb = sc["corpus"], sc["corpus_dir"], sc["sizing_basis"]
        # bodies match the manifest table (len + sha256)
        try:
            read_corpus_bodies(corpus, cdir)
        except (OSError, ValueError) as e:
            fails.append(f"scaled {cell}: body/table mismatch ({e})")
        # corpus.json (if present) binds to the same table
        cj = os.path.join(cdir, "corpus.json")
        if os.path.exists(cj):
            fails += [f"scaled {cell}: {x}" for x in verify_corpus_json(cj, corpus, cdir)]
        # byte-DISTINCT across ALL variants (the compression-honesty invariant)
        seen = {}
        for v in corpus:
            if v["sha256"] in seen:
                fails.append(f"scaled {cell}: variants {v['name']} and {seen[v['sha256']]} "
                             f"share a sha256 -- NOT byte-distinct (dedup could flatter Iris)")
            seen[v["sha256"]] = v["name"]
        # sizing audit
        try:
            n_sets = int(sb["N_sets"])
            if not (N_SETS_FLOOR <= n_sets <= N_SETS_CAP):
                fails.append(f"scaled {cell}: N_sets {n_sets} outside [{N_SETS_FLOOR},{N_SETS_CAP}]")
            if len(corpus) != 4 * n_sets:
                fails.append(f"scaled {cell}: n_variants {len(corpus)} != 4*N_sets ({4*n_sets})")
            total = sum(v["b2f_nodict"] for v in corpus)
            declared = sc["manifest"].get("totals", {}).get("b2f_nodict_total")
            if declared is not None and declared != total:
                fails.append(f"scaled {cell}: totals.b2f_nodict_total {declared} != sum {total}")
            recomputed, _raw = _recompute_n_sets(sb)
            if recomputed != n_sets:
                fails.append(f"scaled {cell}: N_sets {n_sets} != clamp-recompute {recomputed} "
                             f"from R_ss_est={sb.get('R_ss_est')} C_est={sb.get('C_est')} "
                             f"(sizing rule not applied honestly)")
        except (KeyError, TypeError) as e:
            fails.append(f"scaled {cell}: sizing_basis malformed ({e})")
    return fails


# ===========================================================================
# Live dispatch (WIRED; not exercised here -- validate via --dry-run/--self-test)
# ===========================================================================
def _ssh(box, remote_cmd, timeout=60):
    """Run a shell string on a fleet box; return (rc, stdout, stderr)."""
    p = subprocess.run(ssh_argv(box, remote_cmd), capture_output=True, text=True,
                       timeout=timeout)
    return p.returncode, p.stdout, p.stderr


def _scp_down(box, remote_path, local_path, timeout=60):
    subprocess.run(["scp", "-i", SSH_KEY, "-o", "BatchMode=yes",
                    "-o", "StrictHostKeyChecking=no",
                    f"{SSH_USER}@{FLEET[box]}:{remote_path}", local_path],
                   capture_output=True, text=True, timeout=timeout)


def _scp_up(box, local_path, remote_path, timeout=120):
    """Upload a local file to a fleet box (mkdir -p the remote dir first)."""
    _ssh(box, f"mkdir -p {shlex.quote(os.path.dirname(remote_path))}")
    return subprocess.run(["scp", "-i", SSH_KEY, "-o", "BatchMode=yes",
                           "-o", "StrictHostKeyChecking=no", local_path,
                           f"{SSH_USER}@{FLEET[box]}:{remote_path}"],
                          capture_output=True, text=True, timeout=timeout)


def stage_scaled_corpus_to_box(box, cell, fleet_root, root=SCALED_DIR):
    """STAGING FIX (hole-5): push THIS orchestrator's checked-in per-cell scaled
    corpus.json to the box path the pump reads (scaled_corpus_box_path), so the bytes
    that FLY are byte-identical to the gate's checked-in table -- REGARDLESS of the box
    checkout's git state.

    The bug this closes: dispatch materialized corpus.json LOCALLY (from the local bodies
    +manifest) but the pump on the box read the BOX's own ~/hermes-and-mercury/iris
    checkout of corpus.json; when the box checkout DIVERGED from this orchestrator's
    checkout the pump sent bytes that failed the gate's {bytes,sha256} binding
    (CORPUS_BYTES/SHA_MISMATCH -> HARNESS_INVALID), which forced the operator back onto the
    unit corpus.  Uploading the EXACT local corpus.json makes box == checkout by
    construction, so a realistically-sized scaled payload can score with the gate intact.

    Verifies the remote sha256 == the local sha256 after the copy; raises RuntimeError on a
    mismatch/failure so a broken stage surfaces LOUDLY (never a silent wrong-payload run).
    Returns the remote path staged."""
    local_cj = scaled_corpus_json_path(cell, root)   # local checked-in corpus.json
    if not os.path.exists(local_cj):
        # materialize it from the checked-in bodies+manifest (idempotent, byte-stable).
        local_cj = materialize_scaled_corpus_json(cell, root)
    remote_cj = scaled_corpus_box_path(fleet_root, cell)
    with open(local_cj, "rb") as f:
        local_sha = hashlib.sha256(f.read()).hexdigest()
    r = _scp_up(box, local_cj, remote_cj)
    if r.returncode != 0:
        raise RuntimeError(f"stage {cell} -> {box}:{remote_cj} scp failed rc={r.returncode}: "
                           f"{(r.stderr or '').strip()[:200]}")
    _, out, _ = _ssh(box, f"sha256sum {shlex.quote(remote_cj)} 2>/dev/null | awk '{{print $1}}'")
    remote_sha = (out or "").strip().split()[0] if (out or "").strip() else ""
    if remote_sha != local_sha:
        raise RuntimeError(f"stage {cell} -> {box}: remote corpus.json sha {remote_sha[:12]}.. "
                           f"!= local {local_sha[:12]}.. (upload corrupt/raced)")
    return remote_cj


def _load_json(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def _bridge_c1_from_stats(stats):
    """Extract the C1 channel_attestation the bridge writes alongside frames/underruns.
    FIX-D(1): the bridge records underruns in the per-direction (fwd/rev) stats, NOT in
    the attestation object -- fold the total INTO the attestation so the gate can bound
    it (a measured field that was previously invisible to the gate)."""
    if not isinstance(stats, dict):
        return None
    att = stats.get("channel_attestation")
    if not isinstance(att, dict):
        return None
    att = dict(att)
    if "underruns" not in att:
        und = 0
        for d in ("fwd", "rev"):
            dd = stats.get(d)
            if isinstance(dd, dict) and isinstance(dd.get("underruns"), int):
                und += dd["underruns"]
        att["underruns"] = und
    return att


def run_session(spec, iris_bin, iris_repo_on_box, outdir, ctx):
    """Execute ONE deployment-path session ON its fleet box and return a scored row.
    Claims a card, sequences the 5 processes, tears down, collects the two logs +
    the bridge C1 stats + the pump C2 + sender manifests, and scores.  On a
    HARNESS-fault it is re-run (up to MAX_HARNESS_RERUNS) with the SAME seed.  NOT
    exercised in the dry-run deliverable (no live fleet)."""
    box = spec["box"]
    # hole-5: resolve THIS cell's SESSION corpus (the per-cell SCALED variant list if
    # generated, else the module UNIT CORPUS) + point the pump at the per-cell corpus.json.
    # The scaled corpus.json is (re)materialized once, single-threaded, by dispatch_fleet
    # before the concurrent sessions launch (never inside this worker thread).
    sess_corpus, _cdir, is_scaled = corpus_for_spec(spec)
    sess_ctx = dict(ctx)
    if is_scaled:
        sess_ctx["corpus"] = scaled_corpus_box_path(
            ctx.get("fleet_root", DEFAULT_FLEET_ROOT), spec)
    # FIX-D(2): attest the CMD end and RSP end INDEPENDENTLY (each on its own box) so
    # the gate's binary-mismatch check is real, not tautological (was one per-box att
    # copied to both ends).
    binmani = collect_binary_manifest(iris_bin, spec.get("cmd_box", box),
                                      spec.get("rsp_box", box), iris_repo_on_box)
    last_row = None
    for attempt in range(1, MAX_HARNESS_RERUNS + 1):
        rc, out, _ = _ssh(box, card_claim_script(
            spec["card_base_hint"],
            span=spec.get("card_span_hint") or CARD_SCAN_SPAN,
            wrap=spec.get("card_wrap", True)))
        card_idx = None
        for line in (out or "").splitlines():
            if line.startswith("CLAIMED "):
                card_idx = int(line.split()[1])
        if card_idx is None:
            last_row = {"tag": spec["tag"], "outcome": "HARNESS_INVALID", "value_Bmin": None,
                        "activated": False, "metric": None,
                        "gate": {"outcome": "HARNESS_INVALID", "activated": False,
                                 "violations": [{"code": "NO_CARD", "polarity": POLARITY_HARNESS,
                                                 "detail": "no free snd-aloop card"}], "attrs": {}}}
            time.sleep(2.0)
            continue
        card = card_name(card_idx)
        subs = [0, 1, 2, 3]
        # NIT B (COLD-CACHE INVARIANT): every attempt is a genuinely COLD session.
        # VARA keeps NO cross-session probe cache, so cold is the only honest basis;
        # a warm probe-cache READ at the single AFSK connect (kiss_data_pump does ONE
        # SABM/UA connect per session -> a cold session must MISS the cache) trips the
        # runner's WARM_CACHE gate.  iris data_dir = $HOME/.config/iris (main.cc:532-537)
        # and the probe cache = $HOME/.config/iris/probe_cache (modem.cc:7344); HOME is
        # set per-END in iris_cmd() (env HOME=home).  Use a UNIQUE per-attempt home AND
        # recreate a FRESH EMPTY probe_cache dir per attempt, so no probe written by a
        # prior attempt / session / stray run can ever be READ at this session's connect
        # (the cross-attempt persistence that made a3 trip WARM_CACHE after a cold a1).
        home = f"{FLEET_SCRATCH}/{spec['tag']}_a{attempt}"
        try:
            _ssh(box, f"rm -rf {home}; mkdir -p {home}/.config/iris/probe_cache")
            procs = session_processes(spec, iris_bin, card, subs, home, sess_ctx)
            pidfiles = []
            for pr in procs:
                envstr = " ".join(f"{k}={shlex.quote(v)}" for k, v in pr["env"].items())
                argv = " ".join(shlex.quote(x) for x in pr["argv"])
                pidf = f"{home}/{pr['name']}.pid"
                pidfiles.append(pidf)
                _ssh(box, f"cd {home}; nohup env {envstr} {argv} > {pr['log']} 2>&1 "
                          f"& echo $! > {pidf}")
                if pr["after_s"]:
                    time.sleep(pr["after_s"])
            # let the session run its data window + a drain margin.  SESSION_TEARDOWN_GRACE_S
            # is the SAME grace the metric's planned-session wall budgets in
            # SESSION_WALL_OVERHEAD_S (kept in lockstep so a partial is scored over a wall
            # that provably upper-bounds any complete delivery's true wall).
            #
            # EARLY-TEARDOWN (wall-clock only, metric-neutral): the RECEIVER pump
            # exits on its own the moment the whole corpus is hash-verified (its
            # --payload-file early-exit), or at its --duration cap for a partial —
            # so a dead receiver PID is the session's true end either way.  Poll
            # it (bounded by the same full window as before) instead of sleeping
            # the fixed window, so a fast completer releases its card in minutes.
            # Scoring anchors on delivery epochs in the manifests, never on
            # teardown time, and a partial still runs its full receiver duration
            # (the receiver only exits early on COMPLETION) — the planned-wall
            # denominator (plan_session_wall_secs) is unchanged.
            _rx_pidf = f"{home}/kiss_receiver.pid"
            _deadline = time.time() + spec["session_secs"] + SESSION_TEARDOWN_GRACE_S
            _min_run = time.time() + 60.0     # never tear down inside the first minute
            while time.time() < _deadline:
                time.sleep(min(20.0, max(1.0, _deadline - time.time())))
                if time.time() < _min_run:
                    continue
                rc_p, out_p, _ = _ssh(box,
                    f'p=$(cat {_rx_pidf} 2>/dev/null); '
                    f'if [ -n "$p" ] && kill -0 $p 2>/dev/null; then echo RX_ALIVE; '
                    f'else echo RX_DONE; fi')
                if "RX_DONE" in (out_p or ""):
                    break
            # graceful teardown (SIGTERM then SIGKILL), reverse order, scoped to pids.
            _t_term = time.time()   # CARD-HYGIENE: floor the SIGTERM->SIGKILL grace below
            for pidf in reversed(pidfiles):
                _ssh(box, f'p=$(cat {pidf} 2>/dev/null); [ -n "$p" ] && kill -TERM $p 2>/dev/null || true')
            # FIX-C: EVENT-DRIVEN teardown.  The pump flushes its manifest on
            # SIGTERM/atexit + incrementally, so WAIT (bounded) for BOTH manifests to
            # land before SIGKILL -- instead of the old fixed grace that raced the
            # write at slow-connect cells and systematically withheld them.
            # Wait for BOTH pump manifests AND the bridge C1 stats to land (all
            # non-empty) before SIGKILL.  The bridge now flushes bridge_stats.json
            # incrementally + on SIGTERM/atexit (OUTER realaudio_bridge_s32, NIT A), so
            # this just keeps the harness from racing that write -> C1 is never lost to
            # the SIGKILL (the ATTEST_MISSING HARNESS_INVALID that withheld good cells).
            manifests = [f"{home}/kiss_receiver.json", f"{home}/kiss_sender.json",
                         f"{home}/bridge_stats.json"]
            checks = " ".join(f'[ -s {shlex.quote(m)} ] || exit 1;' for m in manifests)
            wait_deadline = time.time() + TEARDOWN_MANIFEST_WAIT_S
            while time.time() < wait_deadline:
                rc_w, out_w, _ = _ssh(box, f'{checks} echo MANIFESTS_READY')
                if "MANIFESTS_READY" in (out_w or ""):
                    break
                time.sleep(TEARDOWN_POLL_S)
            # CARD-HYGIENE FLOOR: iris installs a SIGTERM handler (main.cc:606 ->
            # snd_pcm_close) that needs ~10 s to release the snd-aloop substreams
            # cleanly.  The manifest-wait above can break in <10 s once the pump/bridge
            # atexit-flush their JSON, which would SIGKILL iris before its ALSA close and
            # risk an EBUSY snd_pcm_open wedge on the shared card pool.  Guarantee >=14 s
            # from the SIGTERM broadcast to any SIGKILL so iris always closes gracefully.
            _elapsed = time.time() - _t_term
            if _elapsed < 14.0:
                time.sleep(14.0 - _elapsed)
            for pidf in reversed(pidfiles):
                _ssh(box, f'p=$(cat {pidf} 2>/dev/null); [ -n "$p" ] && kill -KILL $p 2>/dev/null || true')
            # collect evidence.
            local = os.path.join(outdir, spec["tag"] + f"_a{attempt}")
            os.makedirs(local, exist_ok=True)
            for remote, lname in ((f"{home}/iris_cmd.log", "iris_cmd.log"),
                                  (f"{home}/iris_rsp.log", "iris_rsp.log"),
                                  (f"{home}/bridge_stats.json", "bridge_stats.json"),
                                  (f"{home}/kiss_receiver.json", "kiss_receiver.json"),
                                  # HOLE 2: the SENDER manifest carries the TRUE
                                  # first_sabm_tx_epoch the wall must anchor on.
                                  (f"{home}/kiss_sender.json", "kiss_sender.json")):
                _scp_down(box, remote, os.path.join(local, lname))
            evidence = {
                "cmd_log": _scan(os.path.join(local, "iris_cmd.log")),
                "rsp_log": _scan(os.path.join(local, "iris_rsp.log")),
                "launch_cmdlines": [pr["argv"] for pr in procs],
                "bridge_attestation": _bridge_c1_from_stats(
                    _load_json(os.path.join(local, "bridge_stats.json"))),
                "pump_manifest": _load_json(os.path.join(local, "kiss_receiver.json")),
                "sender_manifest": _load_json(os.path.join(local, "kiss_sender.json")),
                "binary_manifest": binmani,
            }
            row = score_session(spec, evidence, corpus=sess_corpus)
            # Instrument gate: attach the verdict to EVERY row (auditable), and
            # let dispatch_fleet abort the cohort on a positive grid split.
            row["instrument_gate"] = instrument_session_gate(
                os.path.join(local, "iris_cmd.log"),
                os.path.join(local, "iris_rsp.log"))
            last_row = row
        finally:
            _ssh(box, card_release_script(card_idx))
        if last_row and last_row["outcome"] != "HARNESS_INVALID":
            return last_row               # SCORED / MODEM_FAULT / REFUSED_DEP are terminal
    return last_row                       # exhausted re-runs -> return last (still invalid)


def dispatch_fleet(plan, iris_bin, boxes, outdir, max_parallel, ctx, fleet_root):
    """Run the whole PLAN on the fleet, sessions concurrently (load-immune audio
    clock), score each, aggregate per (cell,cohort), write the summary JSON + DONE
    marker (tools/wait_done.sh idiom).  Returns the summary dict.  WIRED but NOT
    exercised in the dry-run deliverable."""
    os.makedirs(outdir, exist_ok=True)
    iris_repo_on_box = f"{fleet_root}/iris"      # nested iris repo (build source HEAD)
    # hole-5: (re)materialize each cell's SCALED corpus.json ONCE, single-threaded, BEFORE
    # the concurrent sessions launch (idempotent + byte-stable from the committed bodies;
    # avoids a concurrent-write race across worker threads).  Only cells that already have a
    # generated scaled corpus are materialized; the rest fall back to the unit corpus.json.
    #
    # STAGING FIX (hole-5): then UPLOAD that exact local corpus.json to EVERY box the plan
    # runs on, at the pump's scaled_corpus_box_path, and VERIFY the remote sha == local sha.
    # This makes the bytes that fly byte-identical to the gate's checked-in table regardless
    # of the box checkout's git state -- closing the divergence that failed the gate's
    # {bytes,sha256} binding and forced the unit-corpus fallback.  A stage failure raises
    # (surfaced loudly) rather than silently running the wrong / box-stale payload.
    scaled_cells = sorted({(s["channel"], s["snr"]) for s in plan
                           if has_scaled_corpus((s["channel"], s["snr"]))})
    for (channel, snr) in scaled_cells:
        try:
            materialize_scaled_corpus_json((channel, snr))
        except (OSError, ValueError, KeyError) as e:  # noqa: PERF203
            sys.stderr.write(f"[iris-baseline] WARN: could not materialize scaled "
                             f"corpus.json for {channel}:{snr}: {e}\n")
            continue
        for box in boxes:
            staged = stage_scaled_corpus_to_box(box, (channel, snr), fleet_root)
            sys.stderr.write(f"[iris-baseline] staged scaled corpus {channel}:{snr} -> "
                             f"{box}:{staged} (sha-verified == checkout)\n")
    # Per-box HEAD snapshot for the SUMMARY only (informational).  FIX-D(2): the
    # AUTHORITATIVE per-END attestation is gathered per-session INSIDE run_session
    # (cross-box, via collect_binary_manifest), so the gate's BINARY_MANIFEST_MISMATCH
    # is real -- not one per-box att copied to both ends.
    box_binmani = {}
    for box in boxes:
        rc, out, _ = _ssh(box, binary_attestation_script(iris_bin, repo=iris_repo_on_box))
        box_binmani[box] = _parse_attestation(out)
    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_parallel) as ex:
        futs = {ex.submit(run_session, spec, iris_bin, iris_repo_on_box, outdir, ctx): spec
                for spec in plan}
        gate_abort = None
        for fut in concurrent.futures.as_completed(futs):
            r = fut.result()
            rows.append(r)
            if gate_abort is None and instrument_gate_positive_split(
                    r.get("instrument_gate")):
                # A positive grid split means the BENCH is lying, not the
                # modem failing: no number from any sibling session in this
                # cohort is trustworthy.  Abort — do not average a lie.
                gate_abort = {"tag": r.get("tag"),
                              "gate": r.get("instrument_gate")}
                ex.shutdown(wait=False, cancel_futures=True)
                break
    # aggregate per (cell,cohort): group rows back to their plan spec by tag.
    grouped = {}
    group_spec = {}
    tag_to_spec = {s["tag"]: s for s in plan}
    for r in rows:
        s = tag_to_spec.get(r["tag"])
        if not s:
            continue
        key = (s["channel"], s["snr"], s["cohort"])
        grouped.setdefault(key, []).append(r)
        group_spec.setdefault(key, s)
    # G1/G2: each cell aggregates WITH its session corpus so the verdict carries
    # the corpus fingerprint + the scale it was measured at.
    cells = [aggregate_cell(ch, sn, co, rws,
                            corpus=corpus_for_spec(group_spec[(ch, sn, co)])[0])
             for (ch, sn, co), rws in sorted(grouped.items())]
    summary = {"runner_version": RUNNER_VERSION, "mode": "LIVE",
               "iris_bin": iris_bin, "boxes": boxes,
               "box_binary_manifest": box_binmani,
               "n_sessions": len(rows), "cells": cells,
               "instrument_gate_abort": gate_abort,
               # G2 top-level scale block (per-cell scale lives on each cell):
               # messages_offered here is the LARGEST per-session corpus offered,
               # so a negative claim citing this summary floor-checks against the
               # scale the runner actually reached.
               "scale": {
                   "n_sessions": len(rows),
                   "messages_offered": max([c["scale"]["messages_offered"] or 0
                                            for c in cells] or [0]) or None,
                   "corpus_bytes": max([c["scale"]["corpus_bytes"] or 0
                                        for c in cells] or [0]) or None,
                   "wall_s": (round(sum(c["scale"]["wall_s"] or 0 for c in cells), 1)
                              or None),
               },
               "outcome": "MEASUREMENT"}
    out_json = os.path.join(outdir, "IRIS_HONEST_BASELINE.json")
    # G2: the ONLY sanctioned writer - refuses a verdict with no scale fields
    # and demotes under-scale negative claims to INCONCLUSIVE.
    summary = mg.emit_verdict(out_json, summary)
    with open(os.path.join(outdir, "DONE"), "w", encoding="utf-8") as f:
        if gate_abort:
            f.write(f"ABORTED instrument-gate split on {gate_abort['tag']} "
                    f"(cells={len(cells)} sessions={len(rows)})\n")
        else:
            f.write(f"done cells={len(cells)} sessions={len(rows)}\n")
    if gate_abort:
        sys.stderr.write("[iris-baseline] COHORT ABORT: instrument gate found a "
                         "grid split on %s — bench output is untrustworthy\n"
                         % gate_abort["tag"])
        raise SystemExit(3)
    return summary


# ===========================================================================
# main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=8, help="COLD sessions per (cell,cohort); N>=8")
    ap.add_argument("--iris-bin", default="/dev/shm/iris/iris",
                    help="deployment iris binary path ON the fleet box")
    ap.add_argument("--fleet-root", default=DEFAULT_FLEET_ROOT,
                    help="box path to the workspace checkout (bridge/pump + iris repo HEAD)")
    ap.add_argument("--boxes", default="11,21,31",
                    help="comma-separated fleet boxes (subnet 192.168.2.x)")
    ap.add_argument("--card-base", type=int, default=DEFAULT_CARD_BASE,
                    help="snd-aloop card-base hint (claim from HIGH to dodge capstone)")
    ap.add_argument("--card-span", type=int, default=None,
                    help="number of card indices to scan from --card-base "
                         f"(default {CARD_SCAN_SPAN})")
    ap.add_argument("--no-card-wrap", action="store_true",
                    help="HARD-BOUND claims to [card-base, card-base+card-span): never "
                         "fall back to cards below the base (use when this run has an "
                         "assigned card range on a shared box)")
    ap.add_argument("--cohorts", default=DEFAULT_COHORTS,
                    help="dict cohorts: off=wire-parity (default; dict-ON has no mechanism in Iris)")
    ap.add_argument("--cells", default="all",
                    help="'all' or e.g. 'MPG:15,MPG:20' to target the beatable cells first")
    ap.add_argument("--session-secs", type=int, default=None,
                    help="per-session data window (s); default = per-cell 1.5 x T_target "
                         f"(= {int(round(SESSION_SECS_FACTOR * T_TARGET_S))} s at T_target={T_TARGET_S})")
    ap.add_argument("--max-parallel", type=int, default=12,
                    help="concurrent sessions (load-immune; audio HW clock)")
    ap.add_argument("--out", default=os.path.join(WORKSPACE, "_research",
                                                  "IRIS_HONEST_BASELINE.json"))
    ap.add_argument("--dry-run", action="store_true",
                    help="validate plan + print fleet allocation, sample cmdlines, gate "
                         "config; DO NOT dispatch (default-safe while deps land)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the golden-log + gate-polarity + corpus self-tests and exit")
    ap.add_argument("--materialize-corpus", action="store_true",
                    help="(re)generate the UNIT corpus/corpus.json AND every generated per-cell "
                         "scaled/<cell>/corpus.json from the checked-in bodies, then exit")
    ap.add_argument("--allow-live", action="store_true",
                    help="permit real fleet dispatch (requires the fleet + met deps)")
    ap.add_argument("--port-base", type=int, default=8100,
                    help="first TCP port block (20 ports/session). Concurrent "
                         "runner instances on one box (A/B arms) must use "
                         "disjoint bases, e.g. 8100 and 12100.")
    ap.add_argument("--scratch-tag", default="",
                    help="suffix for the per-session scratch namespace "
                         "(<scratch>_<tag>). REQUIRED when two runner instances "
                         "(e.g. the two arms of an A/B) target the same box: "
                         "session homes are keyed by (cell,idx) tag alone, so two "
                         "un-namespaced instances rm -rf each other's LIVE session "
                         "homes (observed 2026-07-15: the second arm deleted the "
                         "first arm's wave-1 homes out from under 8 live "
                         "processes, orphaning the card holders).")
    args = ap.parse_args()
    if args.scratch_tag:
        global FLEET_SCRATCH
        FLEET_SCRATCH = f"{FLEET_SCRATCH}_{args.scratch_tag}"

    if args.materialize_corpus:
        out = []
        path = materialize_corpus_json()
        out.append({"corpus": "unit", "path": path, "verify_failures": verify_corpus_json(path)})
        for (channel, snr) in CELLS:
            if has_scaled_corpus((channel, snr)):
                sc = load_scaled_corpus((channel, snr))
                cj = materialize_scaled_corpus_json((channel, snr))
                out.append({"corpus": f"{channel}:{snr}", "path": cj,
                            "verify_failures": verify_corpus_json(cj, sc["corpus"], sc["corpus_dir"])})
        fails = [f for o in out for f in o["verify_failures"]]
        print(json.dumps({"materialized": out, "PASS": not fails}, indent=2))
        return 0 if not fails else 1

    if args.self_test:
        g, p, inv = run_self_tests()
        c = corpus_selftest()
        ss = steady_state_selftest()
        dv = distinct_variant_crediting_selftest()
        hd = heldout_disjointness_selftest()
        pa = primed_delivered_axis_selftest()
        gu = measurement_guards_selftest()
        ok = not (g or p or inv or c or ss or dv or hd or pa or gu)
        print(json.dumps({"golden_log_failures": g, "gate_polarity_failures": p,
                          "partial_never_outranks_failures": inv,
                          "corpus_failures": c,
                          "steady_state_failures": ss,
                          "distinct_variant_failures": dv,
                          "heldout_disjointness_failures": hd,
                          "primed_delivered_axis_failures": pa,
                          "measurement_guards_failures": gu,
                          "delivered_plaintext_edge": primed_delivered_edge(),
                          "PASS": ok}, indent=2))
        return 0 if ok else 1

    boxes = [b.strip() for b in args.boxes.split(",") if b.strip()]
    cohorts = [c.strip() for c in args.cohorts.split(",") if c.strip()]
    for c in cohorts:
        if c not in DICT_COHORTS:
            ap.error(f"unknown cohort '{c}' (want one of {list(DICT_COHORTS)})")
    if args.cells == "all":
        cells = CELLS
    else:
        cells = []
        for tok in args.cells.split(","):
            ch, sn = tok.split(":")
            cells.append((ch.strip().upper(), int(sn)))

    plan = build_plan(args.n, args.card_base, boxes, cells=cells, cohorts=cohorts,
                      session_secs=args.session_secs,
                      card_span=args.card_span, card_wrap=not args.no_card_wrap,
                      port_base0=args.port_base)
    if args.n < 8:
        sys.stderr.write(f"[WARN] N={args.n} < 8 : distribution will be under-powered "
                         f"(HONEST_MEASUREMENT_HARNESS §1.4 requires N>=8)\n")

    ctx = box_ctx(args.fleet_root)
    if args.dry_run:
        sample = plan[0]
        home = f"{FLEET_SCRATCH}/{sample['tag']}"
        card = "Loopback_C(claimed@dispatch)"
        subs = [0, 1, 2, 3]
        # mirror run_session: the sample pump args show the per-cell SCALED corpus.json
        # when the sample cell has one generated (else the unit fallback).
        sample_ctx = dict(ctx)
        if has_scaled_corpus(sample):
            sample_ctx["corpus"] = scaled_corpus_box_path(args.fleet_root, sample)
        procs = session_processes(sample, args.iris_bin, card, subs, home, sample_ctx)
        # the anti-force invariant, checked against the REAL rendered argv.
        never_force = all("--force-ofdm" not in " ".join(pr["argv"]) for pr in procs)
        report = {
            "runner_version": RUNNER_VERSION,
            "mode": "DRY-RUN (no dispatch)",
            "cells": [f"{c}:{s}" for (c, s) in cells],
            "cohorts": {c: DICT_COHORTS[c] for c in cohorts},
            "cohort_note": "dict-ON has NO mechanism in Iris (item 4); default is dict-OFF only",
            "N_per_cell_cohort": args.n,
            "total_sessions": len(plan),
            "boxes": boxes,
            "snr_basis": "commanded IONOS dial via --cell; bridge applies WGN_TO_SNR3K=+2.4 -> actual SNR3k (owner 2026-07-02)",
            "never_force_ofdm": never_force,
            "cold_cache": "fresh $HOME per session -> $HOME/.config/iris caches empty",
            "alsa_raw_env": {"IRIS_ALSA_RAW_CAPTURE": "hw:<card>,1,<sub>",
                             "IRIS_ALSA_RAW_PLAYBACK": "hw:<card>,0,<sub>",
                             "note": "per-direction via env (audio_alsa_raw.cc:44-56); --alsa-raw takes 1 optional positional only"},
            "card_claim": {"registry": FLEET_CLAIM_DIR, "atomic": "mkdir",
                           "coexist": "busy-check /proc/asound skips capstone RUNNING substreams",
                           "card_base_hint": args.card_base,
                           "scan": f"{args.card_base}..{args.card_base + CARD_SCAN_SPAN - 1} then wrap",
                           "claim_script": card_claim_script(args.card_base)},
            "gate": {
                "polarity": {"harness_fault": "INVALID -> re-run same seed (force/warm/attest/config-asym/missing-evidence)",
                             "modem_fault": "score actual delivered bytes (usually 0) INTO distribution + activation-rate line-item"},
                "default_deny": "scores ONLY on positive attestation of ALL (C1 + config-tuple-equal-both-sides + binary md5/HEAD both + dict-state + C2 verified bytes)",
                "markers": {
                    "force_ofdm": RE_FORCE_OFDM.pattern,
                    "warm_cache": RE_CACHE_HIT.pattern,
                    "nfft": RE_OFDM_NEG_NFFT.pattern,
                    "band": RE_PROBE_BAND.pattern,
                    "carriers": RE_OFDM_CARRIERS.pattern,
                    "ofdm_active": RE_OFDM_ACTIVE.pattern,
                    "dict_active": RE_DICT_ACTIVE.pattern,
                },
            },
            "sample_session": {
                "tag": sample["tag"], "box": sample["box_ip"],
                "processes": [{"name": pr["name"], "env": pr["env"],
                               "cmd": " ".join(pr["argv"])} for pr in procs],
                "teardown": "reverse order: SIGTERM, grace, SIGKILL (scoped to pids)",
            },
            "self_test": dict(zip(("golden_log_failures", "gate_polarity_failures",
                                   "partial_never_outranks_failures"), run_self_tests()),
                              corpus_failures=corpus_selftest(),
                              steady_state_failures=steady_state_selftest(),
                              distinct_variant_failures=distinct_variant_crediting_selftest()),
            "corpus": {"payload_file": ctx["corpus"],
                       "unit_messages": [{"name": m["name"], "b2f_nodict": m["b2f_nodict"],
                                          "sha256": m["sha256"][:12] + ".."} for m in CORPUS],
                       "materialized_local": CORPUS_JSON,
                       "numerator_binding": "delivered {bytes,sha256} must == table (HOLE 3)"},
            "scaled_corpus": {
                "root": SCALED_DIR,
                "sizing_rule": ("N_sets = clamp(ceil(R_ss_est*(T_target-C_est)/3710), "
                                f"{N_SETS_FLOOR}, {N_SETS_CAP}); T_target={T_TARGET_S}s; "
                                f"session_secs = {SESSION_SECS_FACTOR} x T_target"),
                "per_cell": [
                    ({"cell": f"{ch}:{sn}",
                      "generated": True,
                      "N_sets": (sc := load_scaled_corpus((ch, sn)))["sizing_basis"].get("N_sets"),
                      "n_variants": len(sc["corpus"]),
                      "b2f_nodict_total": sum(v["b2f_nodict"] for v in sc["corpus"]),
                      "calibrated": sc["sizing_basis"].get("calibrated"),
                      "cap_applied": sc["sizing_basis"].get("cap_applied"),
                      "session_secs": _cell_session_secs((ch, sn)),
                      "corpus_json_box": scaled_corpus_box_path(args.fleet_root, (ch, sn))}
                     if has_scaled_corpus((ch, sn)) else
                     {"cell": f"{ch}:{sn}", "generated": False,
                      "note": "run make_scaled_corpus.py (after the calibration pilot); "
                              "falls back to the unit corpus until then",
                      "session_secs": _cell_session_secs((ch, sn))})
                    for (ch, sn) in cells],
                "note": ("headline = connect-inclusive whole-session wire-parity B/min (the ONLY "
                         "VARA-compared number); steady_state_Bmin + connect_tax_pct are line-items"),
            },
            "dependencies_preflight": preflight(args.iris_bin)
            if os.path.exists(args.iris_bin) else
            {"note": f"iris bin {args.iris_bin} not present locally; preflight runs on the box"},
            "metric_discipline": {
                "off": "emits ONLY wire_parity_Bmin (dict off); vs VARA narrow bar directly",
                "on":  "emits ONLY plaintext_Bmin (dict on); vs VARA bar * LZHUF ratio (no backend yet)",
                "never_mixed": True,
                "wall": "SENDER-first-SABM-TX -> last-hash-verified-byte (whole session; HOLE 2)",
                "connect_probe_climb": "separate line-item (SENDER-first-SABM-TX -> RX-first-verified-byte)",
            },
        }
        print(json.dumps(report, indent=2))
        return 0

    # --- REAL DISPATCH ---
    if not args.allow_live:
        sys.stderr.write(
            "[iris-baseline] real dispatch is WIRED (dispatch_fleet) but requires the\n"
            "  fleet + met deps and is gated behind --allow-live.  A live Mercury\n"
            "  capstone is running; do NOT collide.  Run --dry-run to validate the plan\n"
            "  or --self-test to validate the gate.  Refusing to dispatch.\n")
        return 3
    outdir = os.path.dirname(os.path.abspath(args.out)) or "."
    summary = dispatch_fleet(plan, args.iris_bin, boxes, outdir, args.max_parallel,
                             ctx, args.fleet_root)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
