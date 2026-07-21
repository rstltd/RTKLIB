#!/usr/bin/env bash
# Prove the RINEX "SYS / # / OBS TYPES" tobs-row overflow fix under AddressSanitizer.
# Builds the regression test twice:
#   1. against the current (patched) src/rinex.c            -> must run clean
#   2. against a copy with the `||!buff[0]` guard reverted  -> ASan must abort
# The OOB is a write to tobs[RNX_NUMSYS] of a stack array, which ASan sees directly.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SRC="$REPO/src"
TMP="$(mktemp -d)"
CC="${CC:-gcc}"
CFLAGS="-std=c99 -I$SRC -fsanitize=address -g -DTRACE -DENAGLO -DENAGAL -DENAQZS -DENACMP -DENAIRN"
# rinex.c is #included by the test TU; link the externals its functions reference.
DEPS="$SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c $SRC/ephemeris.c $SRC/sbas.c $SRC/geoid.c $SRC/ionex.c"

echo "== build 1/2: patched src/rinex.c =="
$CC $CFLAGS -DRINEX_UNDER_TEST="\"$SRC/rinex.c\"" "$HERE/rinex_obsh_test.c" \
    $DEPS -lm -o "$TMP/patched" || { echo "compile(patched) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/patched"; then
    echo "patched: CLEAN (as expected)"
else
    echo "patched: ASan/assert FAILED -- the fix is not clean"; rm -rf "$TMP"; exit 1
fi

echo "== build 2/2: buggy variant (guard reverted) =="
sed 's/if (!p||!buff\[0\])/if (!p)/' "$SRC/rinex.c" > "$TMP/rinex_buggy.c"
if ! grep -q 'if (!p) {' "$TMP/rinex_buggy.c"; then echo "revert sed did not match"; rm -rf "$TMP"; exit 2; fi
$CC $CFLAGS -DRINEX_UNDER_TEST="\"$TMP/rinex_buggy.c\"" "$HERE/rinex_obsh_test.c" \
    $DEPS -lm -o "$TMP/buggy" || { echo "compile(buggy) FAILED"; exit 2; }
if ASAN_OPTIONS=detect_leaks=0 "$TMP/buggy" >"$TMP/buggy.log" 2>&1; then
    echo "buggy: ran WITHOUT error -- test does not exercise the overflow!"; rm -rf "$TMP"; exit 1
else
    echo "buggy: aborted (overflow reproduced) -- fix is proven necessary"
    grep -m1 -E "ERROR: AddressSanitizer|stack-buffer-overflow|global-buffer-overflow" "$TMP/buggy.log" | sed 's/^/    /'
fi

echo "RESULT: PASS (patched clean, buggy caught)"
rm -rf "$TMP"
