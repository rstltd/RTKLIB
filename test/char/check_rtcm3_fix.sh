#!/usr/bin/env bash
# Prove the RTCM3 1007/1008 antenna-descriptor overflow fix under AddressSanitizer.
# Builds the regression test twice:
#   1. against the current (patched) src/rtcm3.c  -> must run clean, assertions pass
#   2. against a temporary buggy variant (clamp lines stripped) -> ASan must abort
# Exit 0 only if the patched build is clean AND the buggy build is caught.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$REPO/src"
TMP="$(mktemp -d)"
CC="${CC:-gcc}"
CFLAGS="-std=c99 -I$SRC -fsanitize=address -g -DTRACE -DENAGLO"
# rtcm3.c is #included by the test TU; link only the externals its functions
# reference (rtkcmn/trace + ephemeris/rinex/sbas for the eph decoders). rtcm.c
# is deliberately omitted (it would drag in decode_rtcm2/encode_rtcm3).
DEPS="$SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c $SRC/rinex.c $SRC/ephemeris.c $SRC/sbas.c $SRC/geoid.c $SRC/ionex.c"

echo "== build 1/2: patched src/rtcm3.c =="
$CC $CFLAGS -DRTCM3_UNDER_TEST="\"$SRC/rtcm3.c\"" "$HERE/rtcm3_antenna_test.c" \
    $DEPS -lm -o "$TMP/patched" || { echo "compile(patched) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/patched"; then
    echo "patched: CLEAN (as expected)"
else
    echo "patched: ASan/assert FAILED -- the fix is not clean"; rm -rf "$TMP"; exit 1
fi

echo "== build 2/2: buggy variant (clamp stripped) =="
# Strip the two/three clamp lines to reconstruct the pre-fix vulnerable code.
grep -v '>=(int)sizeof(' "$SRC/rtcm3.c" > "$TMP/rtcm3_buggy.c"
$CC $CFLAGS -DRTCM3_UNDER_TEST="\"$TMP/rtcm3_buggy.c\"" "$HERE/rtcm3_antenna_test.c" \
    $DEPS -lm -o "$TMP/buggy" || { echo "compile(buggy) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/buggy" >"$TMP/buggy.log" 2>&1; then
    echo "buggy: ran WITHOUT error -- test does not exercise the overflow!"; rm -rf "$TMP"; exit 1
else
    # The overflow is intra-struct (antdes -> antsno -> rectype within one stack
    # object), which ASan cannot see; the sentinel check in the test detects it.
    echo "buggy: aborted (overflow reproduced) -- fix is proven necessary"
    grep -m1 -iE "OVERFLOW|AddressSanitizer" "$TMP/buggy.log" | sed 's/^/    /'
fi

echo "RESULT: PASS (patched clean, buggy caught)"
rm -rf "$TMP"
