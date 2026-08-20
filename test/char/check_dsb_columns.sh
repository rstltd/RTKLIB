#!/usr/bin/env bash
# Regression guard: Bias-SINEX DSB records must survive the fixed-column parse.
# RED if readbiaf() reads OBS2 from the wrong column (records silently dropped).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$REPO/src"
TMP="$(mktemp -d)"
CC="${CC:-gcc}"
CFLAGS="-std=c99 -I$SRC -DTRACE -DENAGLO -DENAGAL -DENACMP -DENAQZS -DENAIRN -DNFREQ=3 -g"
DEPS="$SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c $SRC/rinex.c $SRC/ephemeris.c $SRC/sbas.c $SRC/geoid.c $SRC/ionex.c"

echo "== build dsb_columns_test =="
if ! $CC $CFLAGS "$HERE/dsb_columns_test.c" $DEPS -lm -o "$TMP/t" 2>"$TMP/cc.log"; then
    echo "COMPILE FAILED:"
    sed 's/^/    /' "$TMP/cc.log" | grep -iE 'error' | head -8
    rm -rf "$TMP"; exit 1
fi
echo "== run =="
if "$TMP/t" "$HERE/data/synthetic_dsb.bia"; then
    echo "RESULT: PASS (DSB column parsing GREEN)"
    rm -rf "$TMP"; exit 0
else
    echo "RESULT: FAIL (DSB records dropped)"
    rm -rf "$TMP"; exit 1
fi
