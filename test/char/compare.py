#!/usr/bin/env python3
"""Characterization comparator for rnx2rtkp .pos output.

Normalizes the two volatile header lines (program version, absolute input
paths) so golden files are portable across machines/builds, then compares a
current run against a golden byte-for-byte on comments and token-wise (with an
optional absolute+relative float tolerance) on solution rows.

Subcommands:
  normalize <raw>                        write normalized text to stdout
  compare   <golden> <raw> [abstol reltol]  compare; exit 0 if match else 1
  coverage  <cases.txt>                  print PMODE coverage table
"""
import os
import re
import sys

PMODES_ALL = [
    "SINGLE", "DGPS", "KINEMA", "STATIC", "STATIC_START",
    "MOVEB", "FIXED", "PPP_KINEMA", "PPP_STATIC", "PPP_FIXED",
]

_RE_PROGRAM = re.compile(r"^% program\b")
_RE_INPFILE = re.compile(r"^(% inp file\s*:\s*)(.*)$")


def normalize_lines(lines):
    """Mask the two volatile header lines; leave everything else intact."""
    out = []
    for ln in lines:
        ln = ln.rstrip("\n").rstrip()
        if _RE_PROGRAM.match(ln):
            out.append("% program   : <NORMALIZED>")
            continue
        m = _RE_INPFILE.match(ln)
        if m:
            out.append(m.group(1) + os.path.basename(m.group(2).strip()))
            continue
        out.append(ln)
    return out


def read_norm(path):
    with open(path, "r", errors="replace") as f:
        return normalize_lines(f.readlines())


def _as_float(tok):
    try:
        return float(tok)
    except ValueError:
        return None


def _num_close(a, b, abstol, reltol):
    d = abs(a - b)
    return d <= abstol or d <= reltol * max(abs(a), abs(b))


def compare(golden_path, raw_path, abstol, reltol, max_report=6):
    golden = read_norm(golden_path)          # golden is stored already-normalized
    current = normalize_lines(open(raw_path, errors="replace").readlines())
    issues = []
    if len(golden) != len(current):
        issues.append(
            "line count differs: golden=%d current=%d" % (len(golden), len(current))
        )
    for i, (g, c) in enumerate(zip(golden, current), start=1):
        if g == c:
            continue
        g_is_comment = g.startswith("%") or c.startswith("%")
        if g_is_comment:
            issues.append("L%d comment differs:\n    golden : %s\n    current: %s"
                          % (i, g, c))
        else:
            gt, ct = g.split(), c.split()
            if len(gt) != len(ct):
                issues.append("L%d token count %d!=%d\n    golden : %s\n    current: %s"
                              % (i, len(gt), len(ct), g, c))
            else:
                for k, (a, b) in enumerate(zip(gt, ct)):
                    if a == b:
                        continue
                    fa, fb = _as_float(a), _as_float(b)
                    if fa is not None and fb is not None:
                        if not _num_close(fa, fb, abstol, reltol):
                            issues.append("L%d col%d numeric %s vs %s (|d|=%.3e)"
                                          % (i, k + 1, a, b, abs(fa - fb)))
                    else:
                        issues.append("L%d col%d string %r vs %r" % (i, k + 1, a, b))
        if len(issues) >= max_report:
            issues.append("... (truncated at %d issues)" % max_report)
            break
    return issues


def coverage(cases_path):
    seen = {}
    with open(cases_path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split("|", 3)
            if len(parts) < 3:
                continue
            pmode = parts[1].strip()
            seen.setdefault(pmode, 0)
            seen[pmode] += 1
    print("PMODE coverage (cases exercising each mode):")
    covered, missing = [], []
    for m in PMODES_ALL:
        n = seen.get(m, 0)
        mark = "COVERED (%d)" % n if n else "NOT COVERED"
        (covered if n else missing).append(m)
        print("  %-14s %s" % (m, mark))
    unknown = [m for m in seen if m not in PMODES_ALL]
    if unknown:
        print("  UNKNOWN labels: %s" % ", ".join(sorted(unknown)))
    print("Summary: %d/%d PMODEs covered; NOT covered: %s"
          % (len(covered), len(PMODES_ALL), ", ".join(missing) or "none"))
    return 0


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd = argv[1]
    if cmd == "normalize":
        for ln in read_norm(argv[2]):
            print(ln)
        return 0
    if cmd == "compare":
        abstol = float(argv[4]) if len(argv) > 4 else 0.0
        reltol = float(argv[5]) if len(argv) > 5 else 0.0
        issues = compare(argv[2], argv[3], abstol, reltol)
        if issues:
            for it in issues:
                print(it)
            return 1
        return 0
    if cmd == "coverage":
        return coverage(argv[2])
    print("unknown subcommand: %s" % cmd)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
