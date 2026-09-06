#!/usr/bin/env python3
"""
measurement_guards.py - machine-enforced measurement-integrity guards for the
iris honest-baseline runner (and any lane that scores iris throughput).

Four guards, each motivated by a REAL measurement failure (2026-07-09/10) that
no artifact caught at action time.  The rule "rules that must bind at action
time live in the artifact that performs the action" is why this module exists:

  G1  ARM PROVENANCE.  An A/B "authoritative arm" tarball turned out to be a
      byte-identical duplicate of its own baseline (same `build=` stamp, same
      SABM epochs to the millisecond, same median 2524.4 B/min).  Scoring
      "baseline vs auth arm" would have FALSELY refuted a real result.  The
      running binary self-identifies: `build=<8hex>` in the iris log is a
      BLAKE2b content hash of the running executable (source/main.cc:33,
      printed at main.cc:603).  G1 harvests it, stamps it on every scored
      row, and REFUSES any A/B comparison whose two arms share a build hash,
      whose corpora differ, or whose evidence files are byte-identical.

  G2  DISPROOF SCALE.  A binding-constraint claim was declared "DISPROVEN" on
      a 4-message / 3,710 B unit corpus; at fair scale (104 messages) the
      modem wedges at ~message 45 - the bug CANNOT manifest at n=4.  G2 makes
      every verdict carry its scale (n_sessions / messages_offered /
      corpus_bytes / wall_s) and demotes any NEGATIVE verdict measured below a
      configurable message floor to INCONCLUSIVE.

  G3  SIGNIFICANCE.  A 2524 -> 3995 B/min "1.6x win" (n=6, seeds shared
      across arms) had paired Wilcoxon p = 0.6875 and a bootstrap CI on the
      median difference spanning zero; one seed collapsed 91 -> 31 messages.
      G3 computes the paired test + bootstrap CI for every improvement claim
      and REFUSES to populate improvement fields when the effect is not
      significant.  Point measurements still publish, with n and IQR.

  G4  CITABILITY.  "8281 B/min = 0.155x VARA" was hand-derived from a 24.5 s
      steady-state window in a cell whose own gate had WITHHELD vs_vara.  G4
      stamps every ratio `citable: true|false` with a reason and a paste-able
      provenance string; a ratio is citable ONLY when the gate passed, the
      denominator is named with its value, and the ratio was computed by the
      runner itself in the same invocation.  Line-item numerators
      (steady_state_Bmin) are NEVER citable against the VARA bar.

Pure functions + a small CLI; no fleet access, no side effects beyond the
explicit emit/harvest entry points.  Self-tests are keyed to the REAL
historical artifacts (checked-in harvested fixtures under tools/guard_fixtures/).

Residual holes are documented in _research/MEASUREMENT_GUARDS.md (workspace).
A guard advertised as catching more than it does is worse than no guard.
"""
import argparse
import glob
import hashlib
import json
import os
import random
import re
import statistics
import subprocess
import sys
import time

GUARDS_VERSION = "1.0.0"
HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES_DIR = os.path.join(HERE, "guard_fixtures")
IRIS_REPO = os.path.dirname(HERE)

# `build=` is the BLAKE2b content hash of the RUNNING executable (first 8 hex
# chars), computed by compute_exe_hash() at source/main.cc:33 and printed at
# startup by main.cc:603 ("Iris FM Data Modem v0.2  build=%s").  Identical
# stamp => byte-identical binary; it cannot be faked by a stale directory name.
RE_BUILD = re.compile(r"\bbuild=([0-9a-f]{8})\b")


class GuardError(Exception):
    """A guard refused.  Refusals are LOUD by design - never swallow this."""


def _read(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def _load_json(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def _md5(path):
    try:
        h = hashlib.md5()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 16), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


# ===========================================================================
# G1 - ARM PROVENANCE
# ===========================================================================
def extract_build_hashes(log_text):
    """All distinct `build=<8hex>` stamps in an iris log (normally exactly one)."""
    return sorted(set(RE_BUILD.findall(log_text or "")))


def corpus_fingerprint(corpus_table):
    """Stable identity of a corpus TABLE: sha256 over the sorted canonical
    (name, sha256, b2f_nodict) triples.  Two runs are same-corpus iff equal."""
    if not corpus_table:
        return None
    canon = sorted((m["name"], m.get("sha256"), m.get("b2f_nodict"))
                   for m in corpus_table)
    return hashlib.sha256(json.dumps(canon, sort_keys=True).encode()).hexdigest()


def session_provenance(evidence, corpus_table=None):
    """The provenance block G1 stamps on EVERY scored session row: the binary
    content hash(es) seen in the two logs, the attested git HEAD + md5 (when
    the two ends agree), and the corpus fingerprint the numerator was bound to."""
    builds = sorted(set(extract_build_hashes(evidence.get("cmd_log", "")) +
                        extract_build_hashes(evidence.get("rsp_log", ""))))
    binmani = evidence.get("binary_manifest") or {}
    cmd_b, rsp_b = binmani.get("CMD") or {}, binmani.get("RSP") or {}
    head = cmd_b.get("git_head") if cmd_b.get("git_head") == rsp_b.get("git_head") else None
    md5 = cmd_b.get("md5") if cmd_b.get("md5") == rsp_b.get("md5") else None
    return {"build_hashes": builds,
            "git_head": head,
            "binary_md5": md5,
            "corpus_sha256": corpus_fingerprint(corpus_table)}


def _session_dirs(arm_dir):
    """Session evidence subdirs of a harvested arm (dirs containing a
    kiss_receiver.json), sorted for determinism."""
    out = []
    for p in sorted(glob.glob(os.path.join(arm_dir, "*"))):
        if os.path.isdir(p) and os.path.exists(os.path.join(p, "kiss_receiver.json")):
            out.append(p)
    return out


def _session_value_Bmin(tx, rx, table, overhead_s=90.0):
    """Whole-session wire-parity B/min recomputed from the raw pump manifests,
    mirroring the runner's metric: unique hash-verified corpus messages summed
    over the connect-inclusive wall (planned wall = duration + overhead for a
    partial; last_verified - t0 for a complete).  Returns (value, n_verified,
    complete) or (None, n, complete) when epochs are missing."""
    t0 = (tx or {}).get("first_sabm_tx_epoch")
    wall = (rx or {}).get("wall") or {}
    seen, total = set(), 0
    for m in (rx or {}).get("per_message") or []:
        nm = m.get("name")
        if not (m.get("verified") and nm in table and nm not in seen):
            continue
        exp = table[nm]
        if m.get("bytes") != exp.get("b2f_nodict") or str(m.get("sha256")) != str(exp.get("sha256")):
            continue                     # corpus-binding mismatch: never credited
        seen.add(nm)
        total += exp["b2f_nodict"]
    complete = bool(seen) and seen >= set(table)
    if t0 is None:
        return None, len(seen), complete
    lv = wall.get("last_verified_byte_epoch")
    if complete:
        if lv is None:
            return None, len(seen), complete
        w = float(lv) - float(t0)
    else:
        dur = (tx or {}).get("duration")
        if dur is None:
            return None, len(seen), complete
        t1 = float(t0) + float(dur) + overhead_s
        if lv is not None and float(lv) > t1:
            t1 = float(lv)
        w = t1 - float(t0)
    if w <= 0:
        return None, len(seen), complete
    return round(total / (w / 60.0), 1), len(seen), complete


def harvest_arm(arm_dir, corpus_table=None, note=None):
    """Harvest one result-arm directory into a small provenance descriptor:
    per-session `build=` stamps, SABM epochs, evidence-file md5s, verified
    (name, sha256) pairs, and (when a corpus table is supplied) the recomputed
    whole-session B/min.  This descriptor is what assert_ab_arms consumes and
    what the checked-in guard fixtures are."""
    sessions = []
    sha_by_name, conflicts, payloads = {}, [], set()
    for d in _session_dirs(arm_dir):
        tx = _load_json(os.path.join(d, "kiss_sender.json")) or {}
        rx = _load_json(os.path.join(d, "kiss_receiver.json")) or {}
        builds = sorted(set(
            extract_build_hashes(_read(os.path.join(d, "iris_cmd.log"))) +
            extract_build_hashes(_read(os.path.join(d, "iris_rsp.log")))))
        for m in rx.get("per_message") or []:
            if not m.get("verified"):
                continue
            nm, sh = m.get("name"), str(m.get("sha256"))
            if nm in sha_by_name and sha_by_name[nm] != sh:
                conflicts.append({"name": nm, "sha_a": sha_by_name[nm], "sha_b": sh})
            sha_by_name.setdefault(nm, sh)
        for man in (tx, rx):
            if man.get("payload_file"):
                payloads.add(man["payload_file"])
        val = n_ver = complete = None
        if corpus_table:
            table = {m["name"]: m for m in corpus_table}
            val, n_ver, complete = _session_value_Bmin(tx, rx, table)
        else:
            n_ver = sum(1 for m in (rx.get("per_message") or []) if m.get("verified"))
        sessions.append({
            "tag": os.path.basename(d),
            "build_hashes": builds,
            "first_sabm_tx_epoch": tx.get("first_sabm_tx_epoch"),
            "duration": tx.get("duration"),
            "kiss_sender_md5": _md5(os.path.join(d, "kiss_sender.json")),
            "kiss_receiver_md5": _md5(os.path.join(d, "kiss_receiver.json")),
            "n_verified": n_ver,
            "complete": complete,
            "value_Bmin": val,
        })
    summary = _load_json(os.path.join(arm_dir, "IRIS_HONEST_BASELINE.json"))
    git_head = None
    if summary:
        heads = {b.get("git_head") for b in
                 (summary.get("box_binary_manifest") or {}).values() if b.get("git_head")}
        git_head = heads.pop() if len(heads) == 1 else None
    return {
        "schema": "measguard-arm-v1",
        "guards_version": GUARDS_VERSION,
        # basename only: absolute paths are machine-specific noise and can leak
        # local directory layout into a tracked fixture.
        "arm_dir": os.path.basename(os.path.normpath(arm_dir)),
        "harvested_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "note": note,
        "n_sessions": len(sessions),
        "build_hashes": sorted({b for s in sessions for b in s["build_hashes"]}),
        "git_head": git_head,
        "payload_files": sorted(payloads),
        "corpus_sha256": corpus_fingerprint(corpus_table),
        "verified_sha_by_name": sha_by_name,
        "verified_sha_conflicts": conflicts,
        "sessions": sessions,
    }


def _git_merge_base(repo_dir, a, b):
    """(merge_base_sha or None, error or None).  Never raises: a repo that
    lacks the commits yields a WARN upstream, not a crash."""
    try:
        p = subprocess.run(["git", "-C", repo_dir, "merge-base", a, b],
                           capture_output=True, text=True, timeout=30)
        if p.returncode == 0 and p.stdout.strip():
            return p.stdout.strip(), None
        return None, (p.stderr or "no merge-base").strip()[:200]
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, str(e)


def assert_ab_arms(arm_a, arm_b, repo_dir=None, git_head_a=None, git_head_b=None):
    """G1: REFUSE an A/B comparison whose arms cannot be a real A/B.

    Refusals (any one blocks the comparison; all are reported):
      SHARED_BUILD_HASH   the two arms ran a byte-identical binary (the
                          wgn40_auth_arm decoy class).  Both hash sets printed.
      IDENTICAL_EVIDENCE  a session evidence file (kiss manifest) in one arm is
                          byte-identical (md5) to one in the other arm - the
                          "arms" are the same run harvested twice.
      CORPUS_TABLE_MISMATCH  both arms carry a corpus-table fingerprint and
                          they differ (comparing across corpora).
      CORPUS_SHA_CONFLICT a message name delivered by BOTH arms carries a
                          different sha256 (rerolled/foreign corpus).
      CORPUS_DISJOINT     the arms share ZERO delivered message names (e.g.
                          unit `net_checkin` vs scaled `net_checkin_s00`).
    Warnings (do not block, must be surfaced):
      GIT_HEAD_UNKNOWN    one/both arms carry no attested git HEAD.
      NO_MERGE_BASE       heads known but share no merge-base in repo_dir.

    Returns {"allowed": bool, "refusals": [...], "warnings": [...],
             "identity": {...}}.  Callers scoring an A/B MUST refuse to emit a
    comparison when allowed is False (raise_on_refusal for the throwing form).
    """
    refusals, warnings = [], []
    ba = set(arm_a.get("build_hashes") or [])
    bb = set(arm_b.get("build_hashes") or [])
    shared = ba & bb
    if not ba or not bb:
        refusals.append({"code": "BUILD_HASH_MISSING",
                         "detail": f"arm A builds={sorted(ba) or 'NONE'}, "
                                   f"arm B builds={sorted(bb) or 'NONE'} - a scored arm "
                                   f"must carry the `build=` stamp (main.cc:33)"})
    elif shared:
        refusals.append({"code": "SHARED_BUILD_HASH",
                         "detail": f"arms share binary content hash(es) {sorted(shared)}: "
                                   f"arm A builds={sorted(ba)} vs arm B builds={sorted(bb)} "
                                   f"- the two arms ran the SAME binary; this is not an A/B "
                                   f"(the wgn40_auth_arm decoy class)"})
    md5s_a = {m for s in arm_a.get("sessions") or []
              for m in (s.get("kiss_sender_md5"), s.get("kiss_receiver_md5")) if m}
    md5s_b = {m for s in arm_b.get("sessions") or []
              for m in (s.get("kiss_sender_md5"), s.get("kiss_receiver_md5")) if m}
    dup = md5s_a & md5s_b
    if dup:
        refusals.append({"code": "IDENTICAL_EVIDENCE",
                         "detail": f"{len(dup)} evidence file(s) byte-identical across the "
                                   f"two arms (md5 {sorted(dup)[:3]}...) - same run "
                                   f"harvested twice, not two arms"})
    fa, fb = arm_a.get("corpus_sha256"), arm_b.get("corpus_sha256")
    if fa and fb and fa != fb:
        refusals.append({"code": "CORPUS_TABLE_MISMATCH",
                         "detail": f"corpus fingerprints differ: A={fa[:12]}.. B={fb[:12]}.. "
                                   f"- comparing across corpora is not an A/B"})
    sa = arm_a.get("verified_sha_by_name") or {}
    sb = arm_b.get("verified_sha_by_name") or {}
    common = set(sa) & set(sb)
    conflicts = sorted(nm for nm in common if sa[nm] != sb[nm])
    if conflicts:
        refusals.append({"code": "CORPUS_SHA_CONFLICT",
                         "detail": f"{len(conflicts)} shared message name(s) with DIFFERENT "
                                   f"payload sha256 (e.g. {conflicts[:3]}) - the corpora are "
                                   f"not the same bytes"})
    elif sa and sb and not common:
        refusals.append({"code": "CORPUS_DISJOINT",
                         "detail": f"zero shared delivered message names "
                                   f"(A e.g. {sorted(sa)[:2]}, B e.g. {sorted(sb)[:2]}) - "
                                   f"different corpora (e.g. unit vs scaled)"})
    ha = git_head_a or arm_a.get("git_head")
    hb = git_head_b or arm_b.get("git_head")
    merge_base = None
    if not ha or not hb:
        warnings.append({"code": "GIT_HEAD_UNKNOWN",
                         "detail": f"git HEAD unattested (A={ha or 'unknown'}, "
                                   f"B={hb or 'unknown'}); arm ancestry unverifiable - "
                                   f"identity rests on the build content hash alone"})
    elif repo_dir:
        merge_base, err = _git_merge_base(repo_dir, ha, hb)
        if merge_base is None:
            warnings.append({"code": "NO_MERGE_BASE",
                             "detail": f"no common merge-base for {ha[:12]} and {hb[:12]} "
                                       f"in {repo_dir} ({err}) - arms may be unrelated trees"})
    return {"allowed": not refusals, "refusals": refusals, "warnings": warnings,
            "identity": {"arm_a_builds": sorted(ba), "arm_b_builds": sorted(bb),
                         "arm_a_git_head": ha, "arm_b_git_head": hb,
                         "merge_base": merge_base,
                         "corpus_sha256_a": fa, "corpus_sha256_b": fb}}


def raise_on_refusal(verdict):
    """Throwing form: a refused comparison is a GuardError, printed loudly."""
    if not verdict["allowed"]:
        lines = [f"  REFUSED [{r['code']}]: {r['detail']}" for r in verdict["refusals"]]
        raise GuardError("A/B comparison REFUSED (G1 arm provenance):\n" + "\n".join(lines))
    return verdict


# ===========================================================================
# G2 - DISPROOF SCALE
# ===========================================================================
# A verdict is NEGATIVE when its outcome asserts absence: a disproof, a
# "no effect", a dead cell.  Negative claims are only as strong as the scale
# they were tested at (the reverse-ACK "DISPROVEN" ran on a 4-message corpus;
# the wedge it denied onsets at ~message 31-45 on the 104-message corpus).
NEGATIVE_OUTCOME_TOKENS = {
    "DISPROOF", "DISPROVEN", "REFUTED", "NO_EFFECT", "NULL_RESULT",
    "CELL_DEAD", "DEAD", "NOT_REPRODUCIBLE", "NOT_BINDING", "EXONERATED",
}
REQUIRED_SCALE_FIELDS = ("n_sessions", "messages_offered", "corpus_bytes", "wall_s")
# Floor on messages_offered (PER-SESSION corpus message count) below which a
# negative verdict is INCONCLUSIVE.  31 is the SMALLEST message count at which
# the at-scale wedge class has ever manifested (das_harvest baseline s01
# wedged after 31/104); 30 is a floor for "the corpus could exhibit the bug",
# NOT a proof of sufficiency - onsets up to 103/104 are on record, so the
# per-claim `disproof_scale.justification` carries the real argument.
DEFAULT_NEGATIVE_FLOOR_MESSAGES = 30


def is_negative_outcome(outcome):
    if not outcome:
        return False
    up = str(outcome).upper().replace("-", "_").replace(" ", "_")
    return any(tok in up for tok in NEGATIVE_OUTCOME_TOKENS)


def _find_scale(verdict):
    """Scale block: verdict['scale'] preferred, top-level fields accepted."""
    src = verdict.get("scale") if isinstance(verdict.get("scale"), dict) else verdict
    return {f: src.get(f) for f in REQUIRED_SCALE_FIELDS}


def guard_verdict(verdict, floor_messages=None):
    """G2: return a guarded copy of a verdict dict.

    * REQUIRED_SCALE_FIELDS must all be present (in `scale` or top-level);
      missing fields are reported in `scale_guard.missing` and, for a NEGATIVE
      verdict, force outcome INCONCLUSIVE (a disproof with unrecorded scale is
      no disproof at all).
    * A NEGATIVE verdict must also carry `disproof_scale` with a one-line
      `justification` that the tested scale could exhibit the effect.
    * A NEGATIVE verdict with messages_offered below the floor is emitted as
      INCONCLUSIVE (original outcome preserved in `original_outcome`).
    Positive/neutral verdicts pass through with the scale audit attached.
    """
    floor = DEFAULT_NEGATIVE_FLOOR_MESSAGES if floor_messages is None else floor_messages
    v = dict(verdict)
    scale = _find_scale(v)
    missing = [f for f in REQUIRED_SCALE_FIELDS if scale.get(f) is None]
    guard = {"guards_version": GUARDS_VERSION, "missing": missing,
             "floor_messages": floor, "demoted": False, "reasons": []}
    if is_negative_outcome(v.get("outcome")):
        ds = v.get("disproof_scale")
        just = (ds or {}).get("justification") if isinstance(ds, dict) else None
        if missing:
            guard["reasons"].append(
                f"negative verdict missing scale field(s) {missing} - a disproof "
                f"with unrecorded scale is no disproof")
        if not just:
            guard["reasons"].append(
                "negative verdict lacks disproof_scale.justification (one line: why "
                "THIS scale could have exhibited the effect)")
        mo = scale.get("messages_offered")
        if mo is not None and mo < floor:
            guard["reasons"].append(
                f"messages_offered={mo} < floor {floor}: the effect class this verdict "
                f"denies has never been observed below ~31 messages; a {mo}-message "
                f"corpus cannot exhibit it")
        if guard["reasons"]:
            guard["demoted"] = True
            v["original_outcome"] = v.get("outcome")
            v["outcome"] = "INCONCLUSIVE"
    v["scale_guard"] = guard
    return v


def emit_verdict(path, verdict, floor_messages=None):
    """The ONLY sanctioned verdict writer.  Applies guard_verdict, REFUSES
    (GuardError) to write any verdict that lacks the scale fields entirely,
    then writes atomically.  Returns the guarded verdict."""
    guarded = guard_verdict(verdict, floor_messages=floor_messages)
    missing = guarded["scale_guard"]["missing"]
    if len(missing) == len(REQUIRED_SCALE_FIELDS):
        raise GuardError(f"emit_verdict REFUSED: verdict carries NO scale fields "
                         f"{REQUIRED_SCALE_FIELDS} - stamp the scale before writing")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(guarded, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)
    return guarded


# ===========================================================================
# G3 - SIGNIFICANCE, NOT VIBES
# ===========================================================================
def _iqr(vals):
    vs = sorted(vals)
    if len(vs) < 2:
        return None
    q = statistics.quantiles(vs, n=4)
    return [round(q[0], 1), round(q[2], 1)]


def wilcoxon_signed_rank_exact(diffs):
    """Exact two-sided paired Wilcoxon signed-rank test.

    Zero differences are dropped (standard practice); ties in |d| get midranks.
    Exact conditional null distribution by enumerating all 2^n sign patterns
    (n <= 20).  Returns (W_plus, p_two_sided, n_effective).
    p = 2 * min(P(W <= w), P(W >= w)) capped at 1 - reproduces the historical
    n=6 anchor-fix A/B exactly: p = 0.6875."""
    d = [x for x in diffs if x != 0]
    n = len(d)
    if n == 0:
        return 0.0, 1.0, 0
    if n > 20:
        raise GuardError(f"exact Wilcoxon capped at n=20 (got {n}); use a larger-n test")
    # midranks over |d|
    order = sorted(range(n), key=lambda i: abs(d[i]))
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j + 1 < n and abs(d[order[j + 1]]) == abs(d[order[i]]):
            j += 1
        mid = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = mid
        i = j + 1
    w_plus = sum(r for x, r in zip(d, ranks) if x > 0)
    # exact null: all sign patterns equally likely
    le = ge = 0
    total = 1 << n
    for mask in range(total):
        w = 0.0
        for i in range(n):
            if mask & (1 << i):
                w += ranks[i]
        if w <= w_plus + 1e-9:
            le += 1
        if w >= w_plus - 1e-9:
            ge += 1
    p = min(1.0, 2.0 * min(le, ge) / total)
    return w_plus, p, n


def bootstrap_ci_median_diff(pre, post, n_boot=20000, seed=20260709, alpha=0.05):
    """Percentile bootstrap CI on median(post) - median(pre), resampling PAIRS
    (arms share seeds, so the pairing is real).  Deterministic via `seed`."""
    n = len(pre)
    if n != len(post) or n == 0:
        raise GuardError(f"paired bootstrap needs equal non-empty arms (got {len(pre)}/{len(post)})")
    rng = random.Random(seed)
    stats_ = []
    for _ in range(n_boot):
        idx = [rng.randrange(n) for _ in range(n)]
        stats_.append(statistics.median([post[i] for i in idx]) -
                      statistics.median([pre[i] for i in idx]))
    stats_.sort()
    lo = stats_[int((alpha / 2) * n_boot)]
    hi = stats_[min(n_boot - 1, int((1 - alpha / 2) * n_boot))]
    return round(lo, 1), round(hi, 1)


def paired_effect(pre, post, n_boot=20000, seed=20260709, alpha=0.05):
    """G3: the ONLY sanctioned way to state an improvement between two arms
    that share seeds.  Returns point stats for BOTH arms unconditionally, and
    populates the `improvement` block ONLY when the paired effect is
    significant (exact Wilcoxon p < alpha AND the bootstrap CI on the median
    difference does not span zero).  Otherwise effect=NOT_SIGNIFICANT and the
    improvement fields are REFUSED (None) with the reason recorded - the point
    measurements still publish, with their n and IQR."""
    if len(pre) != len(post) or not pre:
        raise GuardError(f"paired_effect needs equal non-empty paired arms "
                         f"(got {len(pre)}/{len(post)})")
    diffs = [b - a for a, b in zip(pre, post)]
    w, p, n_eff = wilcoxon_signed_rank_exact(diffs)
    lo, hi = bootstrap_ci_median_diff(pre, post, n_boot=n_boot, seed=seed, alpha=alpha)
    med_pre, med_post = statistics.median(pre), statistics.median(post)
    ci_spans_zero = (lo <= 0.0 <= hi)
    significant = (p < alpha) and not ci_spans_zero
    out = {
        "n": len(pre),
        "pre": {"median": round(med_pre, 1), "iqr": _iqr(pre), "values": list(pre)},
        "post": {"median": round(med_post, 1), "iqr": _iqr(post), "values": list(post)},
        "diffs": [round(d, 1) for d in diffs],
        "paired_test": {"name": "wilcoxon_signed_rank_exact_two_sided",
                        "W_plus": w, "p_value": round(p, 6), "n_effective": n_eff},
        "bootstrap_ci95_median_diff": [lo, hi],
        "effect": "SIGNIFICANT" if significant else "NOT_SIGNIFICANT",
    }
    if significant:
        out["improvement"] = {
            "median_diff_Bmin": round(med_post - med_pre, 1),
            "ratio_of_medians": round(med_post / med_pre, 4) if med_pre else None,
        }
    else:
        out["improvement"] = None
        out["improvement_refused_reason"] = (
            f"effect NOT significant at n={len(pre)}: paired Wilcoxon p={p:.4g}"
            f"{' and' if ci_spans_zero else ','} bootstrap CI on the median difference "
            f"[{lo}, {hi}]{' spans zero' if ci_spans_zero else ''} - improvement fields "
            f"refused; point medians publish with n and IQR only")
    return out


# ===========================================================================
# G4 - CITABILITY
# ===========================================================================
LINE_ITEM_NEVER_CITABLE = {
    "citable": False,
    "reason": ("steady_state_Bmin is a post-first-delivery LINE-ITEM over a short "
               "in-session window; it excludes connect/probe/climb and is NEVER "
               "comparable to the whole-session VARA bar (the 8281-B/min class of "
               "hand-derived ratio is refused by construction)"),
}


def ratio_citability(cell, runner_version=None, bar_source="vara_fm_bar_2025.json narrow_3000Hz"):
    """G4: stamp the ONE VARA-comparable ratio of an aggregated cell.

    citable is True ONLY when, in the SAME runner invocation:
      * the cell's gate passed (vs_vara not withheld: compare_reason is None
        and vs_vara is present),
      * the denominator is named WITH its value (vara_Bmin present), and
      * the ratio was computed by the runner's own vara_compare (this function
        is called from aggregate_cell; anything derived by hand outside the
        runner never receives a stamp and is therefore not citable).
    Returns {"citable", "reason", "provenance"} - provenance is a one-line
    human-readable string suitable for pasting next to the number."""
    cellname = cell.get("cell", "?")
    cohort = cell.get("cohort", "?")
    basis = cell.get("basis", "?")
    dist = cell.get("distribution_Bmin") or {}
    med, n, iqr = dist.get("median"), dist.get("n"), dist.get("iqr")
    vara, ratio = cell.get("vara_Bmin"), cell.get("vs_vara")
    reason_withheld = cell.get("compare_reason")
    denom_name = f"vara_{cellname.replace(':', '').lower()}_Bmin"
    rv = runner_version or "unknown"
    if reason_withheld or ratio is None or vara is None:
        why = reason_withheld or "vs_vara/vara_Bmin absent (gate withheld or no control)"
        return {"citable": False,
                "reason": f"gate did not publish a ratio for {cellname}: {why}",
                "provenance": (f"{cellname} {cohort} {basis} median={med} B/min "
                               f"[n={n} IQR {iqr}] vs VARA: RATIO WITHHELD ({why}) "
                               f"[runner {rv}]")}
    prov = (f"{cellname} {cohort} {basis} median={med} B/min / {denom_name}={vara} "
            f"({bar_source}) = {ratio} [n={n} IQR {iqr}; gate=clean; computed by "
            f"iris_honest_baseline_runner {rv} in-invocation]")
    return {"citable": True,
            "reason": (f"gate passed, control denominator named ({denom_name}={vara}), "
                       f"ratio computed by the runner in the same invocation"),
            "provenance": prov}


def stamp_verdict_citability(verdict, runner_version=None):
    """Stamp citability onto every cell of an EXISTING verdict JSON (e.g. a
    historical IRIS_HONEST_BASELINE.json produced before G4).  Idempotent.
    NOTE: this retro-stamp can only ever confirm what the recorded gate state
    already supports - it cannot make a hand-derived number citable."""
    v = dict(verdict)
    rv = runner_version or v.get("runner_version")
    cells = []
    for cell in v.get("cells") or []:
        c = dict(cell)
        c["vs_vara_citability"] = ratio_citability(c, runner_version=rv)
        c["steady_state_citability"] = dict(LINE_ITEM_NEVER_CITABLE)
        cells.append(c)
    v["cells"] = cells
    return v


def cell_scale_block(session_rows, corpus_table=None):
    """G2 scale block for one aggregated cell, from its session rows + corpus:
    n_sessions (in-distribution), messages_offered (PER-SESSION corpus message
    count - the scale at which a per-session effect can manifest),
    corpus_bytes (per-session planned wire load), wall_s (summed known session
    walls; sessions whose metric carries no wall are counted separately)."""
    in_dist = [r for r in session_rows if r.get("outcome") in ("SCORED", "MODEM_FAULT")]
    walls = [(r.get("metric") or {}).get("wall_secs") for r in in_dist]
    known = [w for w in walls if isinstance(w, (int, float))]
    return {
        "n_sessions": len(in_dist),
        "messages_offered": len(corpus_table) if corpus_table else None,
        "corpus_bytes": (sum(m.get("b2f_nodict") or 0 for m in corpus_table)
                         if corpus_table else None),
        "wall_s": round(sum(known), 1) if known else None,
        "wall_s_sessions_counted": len(known),
    }


# ===========================================================================
# Self-tests - each keyed to the REAL historical artifact that motivated it.
# Fixtures under tools/guard_fixtures/ are harvest_arm() output run against
# the real evidence directories (provenance recorded inside each fixture).
# ===========================================================================
FIX_BASELINE = os.path.join(FIXTURES_DIR, "arm_wgn40_baseline_db99ba99.json")
FIX_DECOY = os.path.join(FIXTURES_DIR, "arm_wgn40_decoy_authextract.json")
FIX_POSTFIX = os.path.join(FIXTURES_DIR, "arm_wgn40_postfix_207b4958.json")
FIX_REMEASURE = os.path.join(FIXTURES_DIR, "arm_remeasure_unit_6e222b47.json")
FIX_REMEASURE_VERDICT = os.path.join(FIXTURES_DIR, "verdict_remeasure_IRIS_HONEST_BASELINE.json")


def _fixture(path, fails):
    d = _load_json(path)
    if d is None:
        fails.append(f"guard fixture missing/unreadable: {path}")
    return d


def arm_provenance_selftest():
    """G1 on the REAL artifacts.
    FAIL-BEFORE: das_harvest baseline vs the wgn40_auth_arm 'authoritative arm'
    both stamp build=db99ba99 with byte-identical kiss manifests - scoring them
    yielded 2524.4 vs 2524.4 and would have falsely refuted the anchor fix.
    The guard must REFUSE, printing both hashes.
    PASS-AFTER: baseline (db99ba99, git 6b0bcd3) vs postfix (207b4958, git
    4c58dff) is the genuine A/B - the guard must ALLOW it, and where both
    commits exist in this repo the merge-base must resolve (6b0bcd3 is an
    ancestor of 4c58dff).
    Plus: baseline (104-msg scaled corpus) vs the remeasure unit arm (4-msg
    corpus) must be REFUSED as a cross-corpus comparison."""
    fails = []
    base = _fixture(FIX_BASELINE, fails)
    decoy = _fixture(FIX_DECOY, fails)
    post = _fixture(FIX_POSTFIX, fails)
    unit = _fixture(FIX_REMEASURE, fails)
    if fails:
        return fails
    # fixture sanity: the REAL stamps, not synthetic ones.
    if base["build_hashes"] != ["db99ba99"]:
        fails.append(f"G1 fixture: baseline builds {base['build_hashes']} != ['db99ba99']")
    if decoy["build_hashes"] != ["db99ba99"]:
        fails.append(f"G1 fixture: decoy builds {decoy['build_hashes']} != ['db99ba99']")
    if post["build_hashes"] != ["207b4958"]:
        fails.append(f"G1 fixture: postfix builds {post['build_hashes']} != ['207b4958']")
    # FAIL-BEFORE witness: the decoy comparison must be REFUSED on BOTH tells.
    v = assert_ab_arms(base, decoy)
    codes = {r["code"] for r in v["refusals"]}
    if v["allowed"]:
        fails.append("G1 FAIL-BEFORE: baseline-vs-decoy (wgn40_auth_arm) was ALLOWED - "
                     "the byte-identical duplicate arm was not refused")
    if "SHARED_BUILD_HASH" not in codes:
        fails.append(f"G1: decoy refusal lacks SHARED_BUILD_HASH (got {sorted(codes)})")
    if "IDENTICAL_EVIDENCE" not in codes:
        fails.append(f"G1: decoy refusal lacks IDENTICAL_EVIDENCE (got {sorted(codes)})")
    det = " ".join(r["detail"] for r in v["refusals"])
    if "db99ba99" not in det:
        fails.append("G1: refusal detail does not print the shared build hash")
    try:
        raise_on_refusal(v)
        fails.append("G1: raise_on_refusal did not raise on the refused decoy comparison")
    except GuardError:
        pass
    # PASS-AFTER witness: the genuine db99ba99-vs-207b4958 A/B must be ALLOWED.
    v2 = assert_ab_arms(base, post, repo_dir=IRIS_REPO,
                        git_head_a="6b0bcd3b12cf03277e9589c72a894900af1e5547",
                        git_head_b="4c58dff")
    if not v2["allowed"]:
        fails.append(f"G1 PASS-AFTER: genuine db99ba99-vs-207b4958 A/B was REFUSED: "
                     f"{v2['refusals']}")
    mb = v2["identity"].get("merge_base")
    if mb is not None and not mb.startswith("6b0bcd3"):
        fails.append(f"G1: merge-base of 6b0bcd3/4c58dff = {mb} (expected 6b0bcd3* - "
                     f"the baseline is an ancestor of the fix)")
    # cross-corpus (104-msg scaled vs 4-msg unit) must be REFUSED.
    v3 = assert_ab_arms(base, unit)
    codes3 = {r["code"] for r in v3["refusals"]}
    if v3["allowed"] or not ({"CORPUS_DISJOINT", "CORPUS_TABLE_MISMATCH"} & codes3):
        fails.append(f"G1: scaled-vs-unit corpus comparison not refused as cross-corpus "
                     f"(allowed={v3['allowed']}, codes={sorted(codes3)})")
    return fails


def disproof_scale_selftest():
    """G2 on the REAL scale.  The 'reverse-ACK binding constraint DISPROVEN'
    claim ran on the 4-message / 3,710 B unit corpus (remeasure_results, build
    6e222b47) - the wedge it denied onsets at >= 31 messages.  FAIL-BEFORE:
    that negative verdict at its true scale must come out INCONCLUSIVE.
    PASS-AFTER: the same claim carried at the 104-message scaled-corpus scale,
    with a justification, survives as a disproof; and a POSITIVE verdict at
    tiny scale is NOT demoted (the floor binds negatives only)."""
    fails = []
    unit = _fixture(FIX_REMEASURE, fails)
    if fails:
        return fails
    walls = [s.get("duration") for s in unit["sessions"] if s.get("duration")]
    # The historical negative claim, reconstructed as a verdict JSON with the
    # REAL scale numbers of the run it was based on (the claim itself was
    # prose in a worklog; the runs are real - see fixture provenance).
    neg = {
        "claim": "reverse-ACK turnaround is NOT the binding constraint (V(A) reaches "
                 "V(S) whenever forward data arrives)",
        "outcome": "DISPROVEN",
        "scale": {"n_sessions": unit["n_sessions"],
                  "messages_offered": 4,            # unit corpus: 4 messages
                  "corpus_bytes": 3710,             # sum b2f_nodict, unit corpus
                  "wall_s": round(sum(w + 90.0 for w in walls), 1)},
    }
    g = guard_verdict(neg)
    if g["outcome"] != "INCONCLUSIVE":
        fails.append(f"G2 FAIL-BEFORE: 4-message disproof outcome={g['outcome']} != "
                     f"INCONCLUSIVE (the wedge cannot manifest at n=4)")
    if g.get("original_outcome") != "DISPROVEN":
        fails.append("G2: demotion must preserve original_outcome=DISPROVEN")
    if not any("messages_offered=4" in r for r in g["scale_guard"]["reasons"]):
        fails.append(f"G2: demotion reasons do not name the scale "
                     f"({g['scale_guard']['reasons']})")
    # PASS-AFTER: at fair scale with a justification, the disproof stands.
    fair = {
        "claim": neg["claim"],
        "outcome": "DISPROVEN",
        "disproof_scale": {"justification": "104-message scaled corpus reaches ~2.3x the "
                                            "largest observed wedge onset (45 msgs)"},
        "scale": {"n_sessions": 6, "messages_offered": 104,
                  "corpus_bytes": 40269, "wall_s": 5940.0},
    }
    g2 = guard_verdict(fair)
    if g2["outcome"] != "DISPROVEN" or g2["scale_guard"]["demoted"]:
        fails.append(f"G2 PASS-AFTER: fair-scale justified disproof was demoted "
                     f"({g2['outcome']}, {g2['scale_guard']})")
    # a negative verdict with NO justification is demoted even at fair scale.
    nojust = dict(fair)
    nojust.pop("disproof_scale")
    if guard_verdict(nojust)["outcome"] != "INCONCLUSIVE":
        fails.append("G2: fair-scale disproof WITHOUT justification must be INCONCLUSIVE")
    # positives are never demoted by the floor.
    pos = {"claim": "x", "outcome": "CONFIRMED",
           "scale": {"n_sessions": 2, "messages_offered": 4, "corpus_bytes": 3710,
                     "wall_s": 900.0}}
    if guard_verdict(pos)["outcome"] != "CONFIRMED":
        fails.append("G2: a POSITIVE verdict must not be demoted by the negative floor")
    # emit_verdict refuses a verdict with NO scale fields at all.
    try:
        emit_verdict(os.devnull, {"outcome": "DISPROVEN"})
        fails.append("G2: emit_verdict wrote a verdict with no scale fields")
    except GuardError:
        pass
    return fails


# The six REAL pre/post pairs (WGN:40, seeds s00..s05, arms share seeds).
# Recomputed independently from the raw kiss manifests + the checked-in
# WGN_40 scaled corpus table (see fixture provenance); they reproduce the
# refutation's numbers exactly: medians 2524.4 -> 3994.65, Wilcoxon p=0.6875.
def _real_pairs(fails):
    base = _fixture(FIX_BASELINE, fails)
    post = _fixture(FIX_POSTFIX, fails)
    if fails:
        return None, None
    pre = [s["value_Bmin"] for s in base["sessions"]]
    pst = [s["value_Bmin"] for s in post["sessions"]]
    if len(pre) != 6 or len(pst) != 6 or None in pre or None in pst:
        fails.append(f"G3 fixture: expected 6 recomputed values per arm "
                     f"(got pre={pre}, post={pst})")
        return None, None
    return pre, pst


def significance_selftest():
    """G3 on the REAL six pre/post session pairs (anchor-fix A/B, WGN:40).
    FAIL-BEFORE: the lane reported 2524 -> 3995 B/min as an improvement; fed
    the actual six pairs, the guard must return effect=NOT_SIGNIFICANT with
    exact p = 0.6875, a CI spanning zero, and REFUSED improvement fields -
    while still publishing both medians with n and IQR.
    PASS-AFTER (positive control): a clearly-shifted paired set must come out
    SIGNIFICANT with the improvement populated (proves the guard can pass)."""
    fails = []
    pre, post = _real_pairs(fails)
    if fails:
        return fails
    eff = paired_effect(pre, post)
    if eff["effect"] != "NOT_SIGNIFICANT":
        fails.append(f"G3 FAIL-BEFORE: real anchor-fix pairs scored "
                     f"effect={eff['effect']} (want NOT_SIGNIFICANT)")
    if abs(eff["paired_test"]["p_value"] - 0.6875) > 1e-9:
        fails.append(f"G3: exact Wilcoxon p={eff['paired_test']['p_value']} != 0.6875 "
                     f"(the historical recompute)")
    lo, hi = eff["bootstrap_ci95_median_diff"]
    if not (lo < 0.0 < hi):
        fails.append(f"G3: bootstrap CI [{lo}, {hi}] does not span zero on the real pairs")
    if eff["improvement"] is not None:
        fails.append("G3: improvement fields were populated on a NOT_SIGNIFICANT effect")
    if "improvement_refused_reason" not in eff:
        fails.append("G3: refused improvement carries no reason")
    if eff["pre"]["median"] != 2524.4 or eff["post"]["median"] != 3994.7:
        fails.append(f"G3: point medians {eff['pre']['median']}/{eff['post']['median']} "
                     f"!= 2524.4/3994.7 (the historical 2524.4 -> 3994.65 pair, rounded; "
                     f"point stats must still publish)")
    if eff["n"] != 6 or not eff["pre"]["iqr"] or not eff["post"]["iqr"]:
        fails.append("G3: n/IQR missing from the published point stats")
    # positive control: all six pairs shift up by ~+1000 with small jitter.
    ctrl_pre = [1000.0, 1100.0, 1200.0, 1300.0, 1400.0, 1500.0]
    ctrl_post = [2010.0, 2090.0, 2210.0, 2320.0, 2390.0, 2515.0]
    eff2 = paired_effect(ctrl_pre, ctrl_post)
    if eff2["effect"] != "SIGNIFICANT" or not eff2["improvement"]:
        fails.append(f"G3 positive control: uniform +~1000 shift scored "
                     f"{eff2['effect']} (want SIGNIFICANT with improvement populated)")
    if eff2["paired_test"]["p_value"] > 0.05:
        fails.append(f"G3 positive control p={eff2['paired_test']['p_value']} > 0.05")
    return fails


def citability_selftest():
    """G4 on the REAL remeasure verdict (the JSON whose WGN:40 cell's gate
    WITHHELD vs_vara while its steady_state_Bmin_median=8280.8 escaped by hand
    as '0.155x VARA').  FAIL-BEFORE: stamping that verdict must mark the
    WGN:40 ratio NOT citable (gate withheld) and the steady-state line-item
    NEVER citable - the 8281-derived ratio cannot acquire a citable stamp.
    PASS-AFTER: the same verdict's MPG:40 cell (gate clean, vara_Bmin=52540
    named, ratio 0.0305 computed in-invocation) must stamp citable with a
    provenance string naming the denominator."""
    fails = []
    verdict = _fixture(FIX_REMEASURE_VERDICT, fails)
    if fails:
        return fails
    stamped = stamp_verdict_citability(verdict)
    by_cell = {c["cell"]: c for c in stamped["cells"]}
    wgn = by_cell.get("WGN:40")
    mpg = by_cell.get("MPG:40")
    if not wgn or not mpg:
        return [f"G4 fixture: expected WGN:40 + MPG:40 cells (got {sorted(by_cell)})"]
    # the real historical facts, asserted so the fixture cannot drift:
    if wgn.get("steady_state_Bmin_median") != 8280.8:
        fails.append(f"G4 fixture: WGN:40 steady_state {wgn.get('steady_state_Bmin_median')} "
                     f"!= 8280.8 (the escaped number)")
    if wgn.get("vs_vara") is not None:
        fails.append("G4 fixture: WGN:40 vs_vara was expected WITHHELD (None)")
    cit = wgn["vs_vara_citability"]
    if cit["citable"] is not False:
        fails.append("G4 FAIL-BEFORE: WGN:40 (gate-withheld cell) stamped citable")
    if "HARNESS_INVALID" not in cit["reason"]:
        fails.append(f"G4: WGN:40 refusal reason does not carry the gate's withhold "
                     f"reason ({cit['reason']})")
    ss = wgn["steady_state_citability"]
    if ss["citable"] is not False or "NEVER" not in ss["reason"]:
        fails.append("G4: steady-state line-item not stamped never-citable")
    # PASS-AFTER: the gate-clean MPG:40 cell stamps citable with provenance.
    cit2 = mpg["vs_vara_citability"]
    if cit2["citable"] is not True:
        fails.append(f"G4 PASS-AFTER: gate-clean MPG:40 not citable ({cit2['reason']})")
    prov = cit2["provenance"]
    for needle in ("vara_mpg40_Bmin=52540", "0.0305", "n=3"):
        if needle not in prov:
            fails.append(f"G4: MPG:40 provenance string lacks '{needle}': {prov}")
    return fails


def run_self_tests():
    """All four guard self-tests; returns {name: [failures]}."""
    return {
        "arm_provenance_G1": arm_provenance_selftest(),
        "disproof_scale_G2": disproof_scale_selftest(),
        "significance_G3": significance_selftest(),
        "citability_G4": citability_selftest(),
    }


# ===========================================================================
# CLI: harvest an arm dir, compare two arms, self-test.
# ===========================================================================
def _load_corpus_table_for_cell(cell):
    """Load the checked-in scaled corpus table for e.g. WGN:40 (variants list),
    or the string 'unit' for the 4-template unit corpus (from the runner)."""
    if cell == "unit":
        sys.path.insert(0, HERE)
        from iris_honest_baseline_runner import CORPUS  # noqa: PLC0415
        return CORPUS
    ch, snr = cell.split(":")
    man = _load_json(os.path.join(HERE, "corpus", "scaled", f"{ch}_{snr}", "manifest.json"))
    if not man:
        raise GuardError(f"no scaled corpus manifest for {cell}")
    return man["variants"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    h = sub.add_parser("harvest", help="harvest an arm evidence dir -> provenance JSON")
    h.add_argument("arm_dir")
    h.add_argument("--cell", default=None,
                   help="corpus for value recompute: 'WGN:40' (scaled) or 'unit'")
    h.add_argument("--note", default=None)
    c = sub.add_parser("ab", help="G1-check two harvested arms (JSON or dirs)")
    c.add_argument("arm_a")
    c.add_argument("arm_b")
    c.add_argument("--cell", default=None)
    c.add_argument("--repo", default=IRIS_REPO)
    sub.add_parser("self-test", help="run the guard self-tests (real-artifact fixtures)")
    args = ap.parse_args(argv)

    if args.cmd == "harvest":
        table = _load_corpus_table_for_cell(args.cell) if args.cell else None
        print(json.dumps(harvest_arm(args.arm_dir, table, note=args.note), indent=2))
        return 0
    if args.cmd == "ab":
        table = _load_corpus_table_for_cell(args.cell) if args.cell else None

        def load(p):
            if os.path.isdir(p):
                return harvest_arm(p, table)
            return _load_json(p)
        v = assert_ab_arms(load(args.arm_a), load(args.arm_b), repo_dir=args.repo)
        print(json.dumps(v, indent=2))
        return 0 if v["allowed"] else 2
    # default: self-test
    res = run_self_tests()
    ok = not any(res.values())
    print(json.dumps({**res, "PASS": ok}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
