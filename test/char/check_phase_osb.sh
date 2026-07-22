#!/usr/bin/env bash
# TDD guard for the file-based phase-OSB loader (Bias-SINEX phase biases).
# Builds phase_osb_test.c against the current src/ and runs it against the
# synthetic Bias-SINEX. RED before the three seams exist (compile error:
# nav->pbias / phase2bias absent), GREEN after they are implemented.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$REPO/src"
TMP="$(mktemp -d)"
CC="${CC:-gcc}"
CFLAGS="-std=c99 -I$SRC -DTRACE -DENAGLO -DENAGAL -DENACMP -DENAQZS -DENAIRN -DNFREQ=3 -g"
# readdcb/code2bias/phase2bias live in preceph.c; link the externals it needs.
DEPS="$SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c $SRC/rinex.c $SRC/ephemeris.c $SRC/sbas.c $SRC/geoid.c $SRC/ionex.c"

echo "== build phase_osb_test =="
if ! $CC $CFLAGS "$HERE/phase_osb_test.c" $DEPS -lm -o "$TMP/t" 2>"$TMP/cc.log"; then
    echo "COMPILE FAILED (RED -- phase-OSB seams not yet implemented):"
    sed 's/^/    /' "$TMP/cc.log" | grep -iE 'pbias|phase2bias|error' | head -8
    rm -rf "$TMP"; exit 1
fi
echo "== run =="
if "$TMP/t" "$HERE/data/synthetic_osb.bia"; then
    echo "RESULT: PASS (phase-OSB loader GREEN)"
    rm -rf "$TMP"; exit 0
else
    echo "RESULT: FAIL (assertions RED)"
    rm -rf "$TMP"; exit 1
fi
