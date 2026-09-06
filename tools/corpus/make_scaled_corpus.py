#!/usr/bin/env python3
"""
make_scaled_corpus.py - FAIR corpus-scaling generator for the Iris-vs-VARA honest
baseline (hole-5 corpus-scaling methodology).

WHAT IT DOES (the_change item 1)
--------------------------------
Match the AMORTIZATION axis = session DURATION (not bytes) inside VARA's own 4-120 min
envelope.  Per cell, size the payload so the projected Iris session runs ~T_target=900 s:

    N_sets(cell) = clamp( ceil( R_ss_est(cell) * (T_target - C_est(cell)) / 3710 ), 2, 54 )

where R_ss_est (steady-state bytes/s) and C_est (connect+probe+climb s) come from a one-shot
1x-corpus CALIBRATION pilot per cell (measured on the fleet at BASELINE time; a documented
DEFAULT is used until then so --dry-run works).  3710 B == one UNIT SET (the 4 real Winlink
templates, sum of b2f_nodict).  N_sets sets == 4*N_sets DISTINCT rerolled variants.

Each variant is produced by:
  UNROLL   the real Winlink template (_research/b2f_unroll_harness/corpus/, reroll
           bit-identity 6/6 proven -- the checked-in unit bodies == lzhuf_b2f(template)),
  VARY     ONLY the fields real traffic varies -- Message-ID (Mid:), Date:, subject serial
           -- deterministically per (template, set index).  Byte-DISTINCT plaintext ->
           byte-DISTINCT LZHUF output, so NO cross-message dedup (present or future dict-ON
           PPMd/zstd streaming carry) can flatter Iris on the wire-parity path,
  REROLL   through the PRODUCTION LZHUF dict-off (the same lzhuf_b2f the unit bodies were
           made with -- Encode(..., b2f=1) -> [CRC16:2][uncomp_size:4 LE][bitstream]).

OUTPUT (per cell, iris/tools/corpus/scaled/<channel>_<snr>/)
  <name>.b2f       the dict-off wire body that flies (what metric_session credits)
  manifest.json    { cell, channel, snr, sizing_basis{R_ss_est,C_est,T_target_s,N_sets,
                     cap_applied,...}, unit_templates, variants[{name,template,set,
                     b2f_nodict,plaintext,sha256,body}], totals }
The runner (iris_honest_baseline_runner.py) reads manifest.json via load_scaled_corpus(),
materializes the per-cell corpus.json the pump flies, and binds every delivered record's
{bytes,sha256} to the per-variant table (numerator == bytes that actually flew).

REPRODUCIBLE by construction: the variation is a pure function of (template, set index) with
NO wall-clock / randomness, and lzhuf_b2f is deterministic -> re-generating yields
byte-identical bodies + manifest (a drift shows up as a git diff).  --self-test proves it.

PRE-SHIP (no version bits): formats change directly, both ends rebuild.  The manifest carries
a fixed `schema` label that STAYS at its first value (never bumped) -- a sanity tag, not a
compat switch.

Usage:
  make_scaled_corpus.py --all-cells [--pilot pilot.json]     # generate all 8 VARA cells
  make_scaled_corpus.py --cell WGN:40 --r-ss-est 12.3 --c-est 38.0
  make_scaled_corpus.py --self-test                          # no fleet; reproducibility + distinctness
Bash, not PowerShell.  Needs python3 + the lzhuf_b2f tool (auto-detected in the harness dir).
"""
import argparse
import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
import tempfile

# ---------------------------------------------------------------------------
# Anchors (this file lives in iris/tools/corpus/)
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))          # .../iris/tools/corpus
IRIS_REPO = os.path.dirname(os.path.dirname(HERE))         # .../iris
WORKSPACE = os.path.dirname(IRIS_REPO)                     # .../hermes and mercury
HARNESS_DIR = os.path.join(WORKSPACE, "_research", "b2f_unroll_harness")
TEMPLATES_DIR = os.path.join(HARNESS_DIR, "corpus")
SCALED_ROOT = os.path.join(HERE, "scaled")

# Fixed sanity label; STAYS at its first value (pre-ship: no version machinery).
SCHEMA = "iris_scaled_corpus"

# The 8 VARA cells (mirrors the runner CELLS table).
CELLS = [("WGN", 40), ("WGN", 30), ("WGN", 20), ("WGN", 15),
         ("MPG", 40), ("MPG", 30), ("MPG", 20), ("MPG", 15)]

# The 4 real Winlink UNIT templates -> the runner's unit-table names.  The template
# .plain files reroll bit-identically to the checked-in unit bodies (proven 6/6).
TEMPLATES = [
    # (unit_name, template_file)
    ("net_checkin", "short1.plain"),
    ("ics213",      "ics213.plain"),
    ("short_email", "email1.plain"),
    ("batch3",      "batch_3msg.plain"),
]

# One UNIT SET (the 4 templates) sums to this many dict-off wire bytes.  The a-priori
# sizing divides by it (HOLE5 SCALING RULE).  Asserted against the live rerolls in
# _self_check_unit_identity so a template edit can never silently desync the divisor.
UNIT_SET_B2F_NODICT = 3710      # 306 + 792 + 910 + 1702

# A-priori sizing constants (HOLE5 SCALING RULE).  T_target is FIXED at 900 s (15 min,
# inside VARA's [4,120] min envelope); floor 2 sets guarantees a steady-state span; cap
# 54 sets (~200 kB) bounds fleet wall-clock.
T_TARGET_S = 900
N_SETS_FLOOR = 2
N_SETS_CAP = 54

# Documented DEFAULT pilot values (used until the fleet calibration pilot measures the
# real per-cell R_ss_est / C_est at BASELINE time).  Flat across cells -- the most honest
# placeholder (we have no per-cell Iris rate without the pilot); the manifest records
# calibrated=false so a default-sized corpus is never mistaken for a measured one.  With
# these defaults N_sets = ceil(15*(900-45)/3710) = ceil(3.456) = 4 sets = 16 variants.
DEFAULT_R_SS_EST_BPS = 15.0
DEFAULT_C_EST_S = 45.0

# Deterministic base date the varied Date: headers spread out from.
_DATE_BASE = datetime.datetime(2026, 6, 12, 0, 0, 0)
_MID_ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"   # base-36, Winlink-style 12-char Mid


# ---------------------------------------------------------------------------
# lzhuf_b2f tool discovery (the PRODUCTION dict-off compressor)
# ---------------------------------------------------------------------------
def find_lzhuf(explicit=None):
    """Locate the lzhuf_b2f binary (Encode/Decode with b2f=1 -- the exact blob the unit
    bodies were made with).  Order: --lzhuf / $IRIS_LZHUF_B2F, then the harness dir
    (.exe on Windows, plain on POSIX)."""
    cands = []
    if explicit:
        cands.append(explicit)
    env = os.environ.get("IRIS_LZHUF_B2F")
    if env:
        cands.append(env)
    win = platform.system() == "Windows"
    cands.append(os.path.join(HARNESS_DIR, "lzhuf_b2f.exe" if win else "lzhuf_b2f"))
    cands.append(os.path.join(HARNESS_DIR, "lzhuf_b2f.exe"))   # fallback (wine / mixed host)
    cands.append(os.path.join(HARNESS_DIR, "lzhuf_b2f"))
    for c in cands:
        if c and os.path.exists(c):
            return c
    raise FileNotFoundError(
        "lzhuf_b2f tool not found (looked at --lzhuf, $IRIS_LZHUF_B2F, "
        f"{HARNESS_DIR}/lzhuf_b2f[.exe]).  Build it from "
        f"{HARNESS_DIR}/lzhuf_b2f.c to (re)generate the scaled corpus.")


def lzhuf_encode(plaintext, lzhuf_bin):
    """Compress `plaintext` bytes to a dict-off B2F LZHUF blob via the production tool.
    Deterministic (integer arithmetic); the temp paths never affect the output bytes."""
    with tempfile.TemporaryDirectory(prefix="scaled_lzh_") as td:
        pin = os.path.join(td, "in.plain")
        pout = os.path.join(td, "out.b2f")
        with open(pin, "wb") as f:
            f.write(plaintext)
        rc = subprocess.run([lzhuf_bin, "e", pin, pout],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
        if rc != 0 or not os.path.exists(pout):
            raise RuntimeError(f"lzhuf_b2f encode failed (rc={rc})")
        with open(pout, "rb") as f:
            return f.read()


# ---------------------------------------------------------------------------
# Deterministic field variation (Message-ID / Date / subject serial ONLY)
# ---------------------------------------------------------------------------
def _seed(*parts):
    return hashlib.sha256("|".join(str(p) for p in parts).encode("utf-8")).digest()


def _new_mid(template, set_idx, occ):
    """A deterministic 12-char uppercase-alnum Message-ID (Winlink Mid form)."""
    d = _seed("mid", template, set_idx, occ)
    return "".join(_MID_ALPHABET[b % len(_MID_ALPHABET)] for b in d[:12])


def _new_date(template, set_idx, occ):
    """A deterministic 'YYYY/MM/DD HH:MM' spread out from the base date."""
    d = _seed("date", template, set_idx, occ)
    minutes = int.from_bytes(d[:4], "big") % (60 * 24 * 60)   # within ~60 days
    dt = _DATE_BASE + datetime.timedelta(minutes=minutes)
    return dt.strftime("%Y/%m/%d %H:%M")


def _subject_serial(template, set_idx, occ):
    """A deterministic serial token appended to the subject (real traffic carries one)."""
    return f"[{set_idx:03d}-{occ}]"


def vary_template(raw, template, set_idx):
    """Return the varied plaintext for one (template, set index).  Operates on LF bytes and
    rewrites ONLY the Mid:/Date:/Subject: header lines, preserving every other byte -- so the
    body (and its 'Body: N' count) is untouched.  A batch template carries N sub-messages;
    each Mid/Date/Subject occurrence is varied independently (occ counter)."""
    lines = raw.split(b"\n")
    occ_mid = occ_date = occ_subj = 0
    out = []
    for ln in lines:
        if ln.startswith(b"Mid: "):
            ln = b"Mid: " + _new_mid(template, set_idx, occ_mid).encode("ascii")
            occ_mid += 1
        elif ln.startswith(b"Date: "):
            ln = b"Date: " + _new_date(template, set_idx, occ_date).encode("ascii")
            occ_date += 1
        elif ln.startswith(b"Subject: "):
            ln = ln + b" " + _subject_serial(template, set_idx, occ_subj).encode("ascii")
            occ_subj += 1
        out.append(ln)
    return b"\n".join(out)


# ---------------------------------------------------------------------------
# Sizing (HOLE5 SCALING RULE)
# ---------------------------------------------------------------------------
def compute_n_sets(r_ss_est, c_est, t_target=T_TARGET_S,
                   unit_bytes=UNIT_SET_B2F_NODICT, floor=N_SETS_FLOOR, cap=N_SETS_CAP):
    """N_sets = clamp( ceil( R_ss_est * (T_target - C_est) / unit_bytes ), floor, cap ).
    Returns (n_sets, raw, cap_applied, floor_applied).  Both sizing-error directions
    DEFLATE Iris (HOLE5): too-low -> shorter session -> more connect tax; too-high ->
    partial scored over the full plan wall.  No choice of N can inflate past PHY truth."""
    import math
    span = max(0.0, float(t_target) - float(c_est))
    raw_f = float(r_ss_est) * span / float(unit_bytes)
    raw = int(math.ceil(raw_f)) if raw_f > 0 else 0
    n = max(floor, min(cap, raw))
    return n, raw, (raw > cap), (raw < floor)


def parse_cell(cell):
    """Accept 'WGN:40' / 'WGN_40' / (channel, snr) / a spec dict -> (channel, snr)."""
    if isinstance(cell, dict):
        return str(cell["channel"]).upper(), int(cell["snr"])
    if isinstance(cell, (tuple, list)):
        return str(cell[0]).upper(), int(cell[1])
    s = str(cell).replace("_", ":")
    ch, sn = s.split(":")
    return ch.strip().upper(), int(sn)


def cell_slug(channel, snr):
    return f"{channel}_{snr}"


def cell_dir(channel, snr, root=SCALED_ROOT):
    return os.path.join(root, cell_slug(channel, snr))


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------
def _read_templates(templates_dir=TEMPLATES_DIR):
    out = []
    for name, fn in TEMPLATES:
        p = os.path.join(templates_dir, fn)
        with open(p, "rb") as f:
            out.append((name, fn, f.read()))
    return out


def build_variants(channel, snr, r_ss_est, c_est, t_target, lzhuf_bin,
                   templates_dir=TEMPLATES_DIR, calibrated=False):
    """Produce the in-memory scaled corpus for one cell (no disk writes).  Returns
    (manifest_dict, {body_filename: payload_bytes}).  DISTINCT rerolled variants only."""
    n_sets, raw, cap_applied, floor_applied = compute_n_sets(r_ss_est, c_est, t_target)
    tmpls = _read_templates(templates_dir)
    variants = []
    bodies = {}
    total = 0
    seen_sha = {}
    for s in range(n_sets):
        for (name, fn, raw_bytes) in tmpls:
            vname = f"{name}_s{s:02d}"
            plaintext = vary_template(raw_bytes, name, s)
            payload = lzhuf_encode(plaintext, lzhuf_bin)
            sha = hashlib.sha256(payload).hexdigest()
            if sha in seen_sha:                    # distinctness invariant (must never fire)
                raise RuntimeError(f"variant {vname} collides with {seen_sha[sha]} "
                                   f"(sha {sha[:12]}..) -- byte-distinctness violated")
            seen_sha[sha] = vname
            body_fn = f"{vname}.b2f"
            bodies[body_fn] = payload
            variants.append({
                "name": vname, "template": name, "set": s,
                "b2f_nodict": len(payload), "plaintext": len(plaintext),
                "sha256": sha, "body": body_fn,
            })
            total += len(payload)
    manifest = {
        "schema": SCHEMA,
        "cell": f"{channel}:{snr}",
        "channel": channel,
        "snr": snr,
        "generated_by": "make_scaled_corpus.py",
        "sizing_basis": {
            "R_ss_est": round(float(r_ss_est), 4),
            "C_est": round(float(c_est), 4),
            "T_target_s": int(t_target),
            "N_sets": n_sets,
            "cap_applied": bool(cap_applied),
            "floor_applied": bool(floor_applied),
            "n_sets_raw": raw,
            "unit_set_b2f_nodict": UNIT_SET_B2F_NODICT,
            "floor": N_SETS_FLOOR,
            "cap": N_SETS_CAP,
            "calibrated": bool(calibrated),
        },
        "unit_templates": [name for name, _ in TEMPLATES],
        "variants": variants,
        "totals": {"n_variants": len(variants), "b2f_nodict_total": total},
    }
    return manifest, bodies


def write_cell(channel, snr, manifest, bodies, root=SCALED_ROOT):
    """Write bodies + manifest.json to scaled/<slug>/ (reproducible: sorted keys, fixed
    variant order).  The runner materializes corpus.json from these bodies."""
    d = cell_dir(channel, snr, root)
    os.makedirs(d, exist_ok=True)
    # Remove any stale bodies from a previous (differently-sized) generation so the dir
    # exactly matches the manifest (no orphan .b2f a smaller N_sets would leave behind).
    keep = set(bodies) | {"manifest.json", "corpus.json"}
    for fn in os.listdir(d):
        if fn.endswith(".b2f") and fn not in keep:
            os.remove(os.path.join(d, fn))
    for fn, payload in bodies.items():
        with open(os.path.join(d, fn), "wb") as f:
            f.write(payload)
    with open(os.path.join(d, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    return d


def generate_cell(cell, r_ss_est, c_est, t_target=T_TARGET_S, lzhuf_bin=None,
                  templates_dir=TEMPLATES_DIR, root=SCALED_ROOT, calibrated=False):
    channel, snr = parse_cell(cell)
    lzhuf_bin = lzhuf_bin or find_lzhuf()
    manifest, bodies = build_variants(channel, snr, r_ss_est, c_est, t_target, lzhuf_bin,
                                      templates_dir=templates_dir, calibrated=calibrated)
    d = write_cell(channel, snr, manifest, bodies, root=root)
    return d, manifest


def _pilot_values(cell, pilot):
    """Resolve (R_ss_est, C_est, calibrated) for a cell from a pilot dict, else defaults.
    pilot maps 'WGN:40' (or 'WGN_40') -> {'R_ss_est':..., 'C_est':...}."""
    if pilot:
        channel, snr = parse_cell(cell)
        for key in (f"{channel}:{snr}", f"{channel}_{snr}"):
            if key in pilot:
                p = pilot[key]
                return float(p["R_ss_est"]), float(p["C_est"]), True
    return DEFAULT_R_SS_EST_BPS, DEFAULT_C_EST_S, False


# ---------------------------------------------------------------------------
# Self-test (no fleet): reproducibility + byte-distinctness + sha match
# ---------------------------------------------------------------------------
def _self_check_unit_identity(lzhuf_bin, templates_dir=TEMPLATES_DIR):
    """The set-0-UNVARIED reroll must reproduce the checked-in UNIT bodies, and the 4
    templates must sum to UNIT_SET_B2F_NODICT -- proving the production LZHUF the scaled
    corpus uses is the SAME one the runner's unit table (3710 divisor) is built on."""
    fails = []
    total = 0
    for name, fn in TEMPLATES:
        raw = open(os.path.join(templates_dir, fn), "rb").read()
        body = lzhuf_encode(raw, lzhuf_bin)          # UNVARIED template == unit body
        total += len(body)
        unit = os.path.join(HERE, f"{name}.b2f")
        if os.path.exists(unit):
            want = open(unit, "rb").read()
            if body != want:
                fails.append(f"unit-identity: reroll({fn}) != checked-in {name}.b2f "
                             f"({len(body)} vs {len(want)} B)")
    if total != UNIT_SET_B2F_NODICT:
        fails.append(f"unit-set divisor drift: sum(reroll)={total} != {UNIT_SET_B2F_NODICT}")
    return fails


def self_test():
    fails = []
    try:
        lzhuf_bin = find_lzhuf()
    except FileNotFoundError as e:
        print(json.dumps({"self_test": "make_scaled_corpus", "PASS": False,
                          "failures": [str(e)]}, indent=2))
        return 1

    fails += _self_check_unit_identity(lzhuf_bin)

    # Generate two cells into a temp root; check reproducibility + distinctness + sha bind.
    with tempfile.TemporaryDirectory(prefix="scaled_selftest_") as troot:
        cellA, cellB = ("WGN", 40), ("MPG", 15)
        dA, mA = generate_cell(cellA, DEFAULT_R_SS_EST_BPS, DEFAULT_C_EST_S,
                               lzhuf_bin=lzhuf_bin, root=troot)
        dB, mB = generate_cell(cellB, 8.0, 60.0, lzhuf_bin=lzhuf_bin, root=troot)

        # (1) sizing sanity: N_sets within [floor, cap]; variants == 4*N_sets.
        for m in (mA, mB):
            n = m["sizing_basis"]["N_sets"]
            if not (N_SETS_FLOOR <= n <= N_SETS_CAP):
                fails.append(f"{m['cell']}: N_sets {n} outside [{N_SETS_FLOOR},{N_SETS_CAP}]")
            if m["totals"]["n_variants"] != 4 * n:
                fails.append(f"{m['cell']}: n_variants {m['totals']['n_variants']} != 4*{n}")

        # (2) REPRODUCIBLE: re-generate cell A into a second root; manifest + every body
        #     byte-identical.
        troot2 = os.path.join(troot, "re")
        dA2, mA2 = generate_cell(cellA, DEFAULT_R_SS_EST_BPS, DEFAULT_C_EST_S,
                                 lzhuf_bin=lzhuf_bin, root=troot2)
        if json.dumps(mA, sort_keys=True) != json.dumps(mA2, sort_keys=True):
            fails.append("reproducibility: cell A manifest changed on re-generate")
        for v in mA["variants"]:
            b1 = open(os.path.join(dA, v["body"]), "rb").read()
            b2 = open(os.path.join(dA2, v["body"]), "rb").read()
            if b1 != b2:
                fails.append(f"reproducibility: body {v['body']} changed on re-generate")

        # (3) byte-DISTINCT across ALL variants of a cell + sha256 binds the body file.
        seen = {}
        for v in mA["variants"]:
            p = os.path.join(dA, v["body"])
            payload = open(p, "rb").read()
            if len(payload) != v["b2f_nodict"]:
                fails.append(f"{v['name']}: body {len(payload)} B != manifest {v['b2f_nodict']}")
            dig = hashlib.sha256(payload).hexdigest()
            if dig != v["sha256"]:
                fails.append(f"{v['name']}: body sha256 != manifest sha256")
            if dig in seen:
                fails.append(f"{v['name']}: NOT byte-distinct from {seen[dig]} "
                             f"(cross-message dedup could flatter Iris)")
            seen[dig] = v["name"]

        # (4) the varied bodies must DIFFER from the unvaried unit body (proves the
        #     Mid/Date/subject variation actually rerolled distinct bytes).
        unit_net = open(os.path.join(HERE, "net_checkin.b2f"), "rb").read() \
            if os.path.exists(os.path.join(HERE, "net_checkin.b2f")) else None
        if unit_net is not None:
            v0 = next(v for v in mA["variants"] if v["template"] == "net_checkin")
            if open(os.path.join(dA, v0["body"]), "rb").read() == unit_net:
                fails.append("variation inert: a varied net_checkin body == the unit body")

    ok = not fails
    print(json.dumps({"self_test": "make_scaled_corpus", "lzhuf": lzhuf_bin,
                      "PASS": ok, "failures": fails,
                      "note": "reproducible + byte-distinct + sha-bound rerolled variants"},
                     indent=2))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", help="single cell 'WGN:40'")
    ap.add_argument("--all-cells", action="store_true", help="generate all 8 VARA cells")
    ap.add_argument("--pilot", help="JSON: {'WGN:40': {'R_ss_est':.., 'C_est':..}, ...} "
                                    "(from the fleet calibration pilot); missing cells use defaults")
    ap.add_argument("--r-ss-est", type=float, help="override steady-state bytes/s (single --cell)")
    ap.add_argument("--c-est", type=float, help="override connect+probe+climb s (single --cell)")
    ap.add_argument("--t-target", type=int, default=T_TARGET_S, help="planned transfer s (default 900)")
    ap.add_argument("--out-root", default=SCALED_ROOT, help="scaled corpus root dir")
    ap.add_argument("--lzhuf", help="path to the lzhuf_b2f tool (else auto-detect)")
    ap.add_argument("--templates-dir", default=TEMPLATES_DIR)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    pilot = None
    if args.pilot:
        with open(args.pilot, "r", encoding="utf-8") as f:
            pilot = json.load(f)

    lzhuf_bin = find_lzhuf(args.lzhuf)

    if args.all_cells:
        cells = CELLS
    elif args.cell:
        cells = [parse_cell(args.cell)]
    else:
        ap.error("give --cell, --all-cells, or --self-test")

    out = []
    for cell in cells:
        if args.cell and (args.r_ss_est is not None or args.c_est is not None):
            r = args.r_ss_est if args.r_ss_est is not None else DEFAULT_R_SS_EST_BPS
            c = args.c_est if args.c_est is not None else DEFAULT_C_EST_S
            calibrated = True
        else:
            r, c, calibrated = _pilot_values(cell, pilot)
        d, manifest = generate_cell(cell, r, c, t_target=args.t_target, lzhuf_bin=lzhuf_bin,
                                    templates_dir=args.templates_dir, root=args.out_root,
                                    calibrated=calibrated)
        sb = manifest["sizing_basis"]
        out.append({"cell": manifest["cell"], "dir": d,
                    "N_sets": sb["N_sets"], "n_variants": manifest["totals"]["n_variants"],
                    "b2f_nodict_total": manifest["totals"]["b2f_nodict_total"],
                    "calibrated": sb["calibrated"], "cap_applied": sb["cap_applied"]})
    print(json.dumps({"generated": out, "lzhuf": lzhuf_bin,
                      "note": "runner materializes per-cell corpus.json from these bodies"},
                     indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
