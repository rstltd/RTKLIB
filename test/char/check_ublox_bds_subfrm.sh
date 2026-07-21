#!/usr/bin/env bash
# Prove the u-blox BDS GEO subframe-5 subfrm overrun fix (enlarged subfrm row).
# Builds the regression test twice:
#   1. against the current src (subfrm[MAXSAT][418]) -> must run clean
#   2. against a copy of rtklib_types.h with the row back at 380 -> sentinel wiped
# The overrun is intra-array (invisible to ASan); a sentinel in the neighbor
# subfrm row is the oracle.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$REPO/src"
TMP="$(mktemp -d)"
CC="${CC:-gcc}"
CFLAGS="-std=c99 -fsanitize=address -g -DTRACE -DENAGLO -DENAGAL -DENAQZS -DENACMP -DENAIRN -DNFREQ=3 -DNEXOBS=3"
# ublox.c is #included by the test TU; link rcvraw + the other rcv decoders it
# dispatches to (input_raw references every input_*), plus core deps.
RCV=""; for f in binex crescent javad novatel nvs rt17 septentrio skytraq swiftnav unicore; do RCV="$RCV $SRC/rcv/$f.c"; done
DEPS="$SRC/rcvraw.c $SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c $SRC/rinex.c $SRC/ephemeris.c $SRC/sbas.c $SRC/geoid.c $SRC/ionex.c $RCV"

echo "== build 1/2: current src (subfrm[MAXSAT][418]) =="
$CC $CFLAGS -I"$SRC" "$HERE/ublox_bds_subfrm_test.c" $DEPS -lm -o "$TMP/patched" \
    || { echo "compile(patched) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/patched"; then
    echo "patched: CLEAN (as expected)"
else
    echo "patched: FAILED -- the fix is not clean"; rm -rf "$TMP"; exit 1
fi

echo "== build 2/2: whole src copied with the subfrm row reverted to 380 =="
# A quoted #include "rtklib_types.h" resolves relative to rtklib.h's own dir
# first, so shadowing via -I does not work; build everything from a copy of src
# whose rtklib_types.h is 380, keeping every TU's raw_t layout consistent.
BSRC="$TMP/src"; cp -r "$SRC" "$BSRC"
sed -i 's/uint8_t subfrm\[MAXSAT\]\[418\]/uint8_t subfrm[MAXSAT][380]/' "$BSRC/rtklib_types.h"
if ! grep -q 'subfrm\[MAXSAT\]\[380\]' "$BSRC/rtklib_types.h"; then
    echo "revert sed did not match"; rm -rf "$TMP"; exit 2; fi
BDEPS="${DEPS//$SRC/$BSRC}"
$CC $CFLAGS -I"$BSRC" "$HERE/ublox_bds_subfrm_test.c" $BDEPS -lm -o "$TMP/buggy" \
    || { echo "compile(buggy) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/buggy" >"$TMP/buggy.log" 2>&1; then
    echo "buggy: ran WITHOUT error -- test does not exercise the overrun!"; rm -rf "$TMP"; exit 1
else
    echo "buggy: aborted (overrun reproduced) -- fix is proven necessary"
    grep -m1 -E "OVERRUN|AddressSanitizer" "$TMP/buggy.log" | sed 's/^/    /'
fi

echo "RESULT: PASS (patched clean, buggy caught)"
rm -rf "$TMP"
