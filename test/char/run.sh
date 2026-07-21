#!/usr/bin/env bash
# Characterization harness for rnx2rtkp.
#
# Usage:
#   ./run.sh build            build rnx2rtkp deterministically (no LAPACK)
#   ./run.sh capture          run all cases, store normalized golden/
#   ./run.sh verify           run all cases, diff against golden/ (default)
#   ./run.sh mutate           self-test: corrupt one golden, confirm verify FAILs
#
# Determinism note: the default gcc build links no LAPACK/MKL (internal LU),
# which the audit identified as the only cross-build numeric divergence source;
# same binary + same input is bit-stable (no RNG, no threads in the batch path).
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BIN="$REPO/app/consapp/rnx2rtkp/gcc/rnx2rtkp"
DATA="$REPO/test/data"
CONF="$REPO/app/consapp/rnx2rtkp/test"
SCONF="$HERE/conf"
CASES="$HERE/cases.txt"
GOLDEN="$HERE/golden"
WORK="$HERE/work"
CMP="$HERE/compare.py"
ABSTOL="${ABSTOL:-0}"
RELTOL="${RELTOL:-0}"

build() {
    echo "[build] make rnx2rtkp (LDLIBS=-lm, no LAPACK)"
    make -C "$REPO/app/consapp/rnx2rtkp/gcc" clean >/dev/null 2>&1
    make -C "$REPO/app/consapp/rnx2rtkp/gcc" -j"$(nproc)" LDLIBS=-lm
}

# Expand @D@/@C@/@SC@ placeholders in a case's arg string.
#   @D@  = test/data   @C@ = upstream conf dir   @SC@ = this harness's conf/
expand() { echo "$1" | sed -e "s#@D@#$DATA#g" -e "s#@C@#$CONF#g" -e "s#@SC@#$SCONF#g"; }

# Iterate cases, calling handle_case id pmode label rawfile.
# A case may use `-o @OUT@` to write its solution to a file (needed for cases
# that also emit a sibling <out>.stat); otherwise the solution is captured from
# stdout. Any <out>.stat produced is treated as an extra golden artifact.
each_case() {
    local handler="$1"
    while IFS='|' read -r id pmode label args; do
        case "$id" in ''|\#*) continue ;; esac
        id="$(echo "$id" | tr -d '[:space:]')"
        local argline; argline="$(expand "$args")"
        local raw
        # Run inside work/ so any side files the binary writes to cwd (e.g.
        # <out>_events.pos) land in the git-ignored scratch dir, not the source tree.
        if [[ "$argline" == *@OUT@* ]]; then
            raw="$WORK/$id.pos"; argline="${argline//@OUT@/$raw}"
            # shellcheck disable=SC2086
            ( cd "$WORK" && "$BIN" $argline >"$id.stdout" 2>"$id.stderr" )
            echo "$?" >"$WORK/$id.rc"
        else
            raw="$WORK/$id.raw"
            # shellcheck disable=SC2086
            ( cd "$WORK" && "$BIN" $argline >"$id.raw" 2>"$id.stderr" )
            echo "$?" >"$WORK/$id.rc"
        fi
        "$handler" "$id" "$pmode" "$label" "$raw"
    done < "$CASES"
}

cap_one() {
    local id="$1" raw="$4"
    python3 "$CMP" normalize "$raw" > "$GOLDEN/$id.pos"
    cp "$WORK/$id.rc" "$GOLDEN/$id.rc"
    if [ -f "$raw.stat" ]; then python3 "$CMP" normalize "$raw.stat" > "$GOLDEN/$id.stat"; fi
    echo "  captured $id ($3)$([ -f "$raw.stat" ] && echo ' +stat')"
}

ver_one() {
    local id="$1" label="$3" raw="$4"
    local grc crc; grc="$(cat "$GOLDEN/$id.rc" 2>/dev/null || echo NA)"; crc="$(cat "$WORK/$id.rc")"
    local out; out="$(python3 "$CMP" compare "$GOLDEN/$id.pos" "$raw" "$ABSTOL" "$RELTOL" 2>&1)"
    local rc=$?
    if [ "$grc" != "$crc" ]; then
        echo "  FAIL $id ($label): exit code golden=$grc current=$crc"; FAILS=$((FAILS+1)); return
    fi
    if [ $rc -ne 0 ]; then
        echo "  FAIL $id ($label):"; echo "$out" | sed 's/^/      /'; FAILS=$((FAILS+1)); return
    fi
    # If a golden .stat exists for this case, the current run must reproduce it.
    if [ -f "$GOLDEN/$id.stat" ]; then
        if [ ! -f "$raw.stat" ]; then
            echo "  FAIL $id ($label): expected .stat output not produced"; FAILS=$((FAILS+1)); return
        fi
        local sout; sout="$(python3 "$CMP" compare "$GOLDEN/$id.stat" "$raw.stat" "$ABSTOL" "$RELTOL" 2>&1)"
        if [ $? -ne 0 ]; then
            echo "  FAIL $id ($label) [.stat]:"; echo "$sout" | sed 's/^/      /'; FAILS=$((FAILS+1)); return
        fi
        echo "  PASS $id ($label) +stat"; return
    fi
    echo "  PASS $id ($label)"
}

prepare_work() { rm -rf "$WORK"; mkdir -p "$WORK"; }

cmd="${1:-verify}"
case "$cmd" in
  build) build ;;
  capture)
    [ -x "$BIN" ] || build
    mkdir -p "$GOLDEN"; prepare_work
    echo "[capture] -> $GOLDEN"
    each_case cap_one
    echo "[coverage]"; python3 "$CMP" coverage "$CASES"
    ;;
  verify)
    [ -x "$BIN" ] || build
    if [ ! -d "$GOLDEN" ]; then echo "no golden/ yet; run: $0 capture" >&2; exit 2; fi
    prepare_work; FAILS=0
    echo "[verify] abstol=$ABSTOL reltol=$RELTOL"
    each_case ver_one
    echo "[coverage]"; python3 "$CMP" coverage "$CASES"
    if [ "$FAILS" -gt 0 ]; then echo "RESULT: FAIL ($FAILS case(s))"; exit 1; fi
    echo "RESULT: PASS (all cases match golden)"
    ;;
  mutate)
    # Self-test the comparator: perturb one numeric in a golden copy -> must FAIL.
    [ -d "$GOLDEN" ] || { echo "run capture first" >&2; exit 2; }
    tmp="$(mktemp -d)"; cp -r "$GOLDEN"/* "$tmp/"
    victim="$(ls "$tmp"/*.pos | head -1)"
    python3 - "$victim" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().splitlines()
for i, ln in enumerate(lines):
    if ln and not ln.startswith('%') and re.search(r'\d\.\d', ln):
        toks = ln.split()
        for k, t in enumerate(toks):
            try:
                toks[k] = "%.4f" % (float(t) + 0.05)  # inject 5cm shift
                lines[i] = " ".join(toks); break
            except ValueError:
                continue
        break
open(p, "w").write("\n".join(lines) + "\n")
print("mutated:", p)
PY
    # Compare a fresh golden against the mutated one; expect non-zero (caught).
    base="$(basename "$victim")"
    if python3 "$CMP" compare "$tmp/$base" "$GOLDEN/$base" 0 0 >/dev/null 2>&1; then
        echo "MUTATE SELF-TEST: FAIL (comparator did NOT catch a 5cm shift)"; rm -rf "$tmp"; exit 1
    fi
    echo "MUTATE SELF-TEST: PASS (5cm shift detected)"; rm -rf "$tmp"
    ;;
  *) echo "usage: $0 {build|capture|verify|mutate}" >&2; exit 2 ;;
esac
