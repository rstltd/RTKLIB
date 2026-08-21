#!/bin/sh
# run_utests.sh : build and run the RTKLIB unit tests.
#
#   test/utest/run_utests.sh            all tests
#   test/utest/run_utests.sh t_lambda   just these
#   test/utest/run_utests.sh --list     list them
#
# Gate G4 of plan.md 6.11 for the tests that already exist.  These cover
# rtkcmn.c, lambda.c, rinex.c, ephemeris.c, preceph.c, ionex.c and ppp.c --
# several of which the FGO work touches -- so they need to run on every change,
# not only when someone remembers to configure CMake.
#
# Deliberately independent of CMake: test/utest/CMakeLists.txt links -llapack
# -lblas, and many systems (this one included) ship only the runtime .so.3
# without the development symlink.  RTKLIB has a built-in matrix path for
# exactly that case, so these build against it and need nothing but libm.
#
# Environment variables:
#   FGO_CC      C compiler (default /usr/bin/gcc)
#   FGO_LDLIBS  link libs  (default -lm)
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SRC=$(CDPATH= cd -- "$here/../../src" && pwd)
work=${TMPDIR:-/tmp}/rtklib-utest.$$

cc=${FGO_CC:-/usr/bin/gcc}
ldlibs=${FGO_LDLIBS:--lm}

# see docs/fgo/build_environment.md
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

OPTS="-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=4 -DNEXOBS=3"

# name:extra sources.  rtkcmn, trace and preceph are linked into every one.
TESTS="t_matrix:
t_time:
t_coord:geoid
t_rinex:rinex
t_lambda:lambda
t_atmos:
t_misc:
t_preceph:rinex ephemeris sbas
t_gloeph:rinex ephemeris sbas
t_geoid:geoid
t_ionex:ionex
t_tle:rinex ephemeris sbas tle
t_ppp:ephemeris sbas ionex pntpos ppp ppp_ar lambda tides"

if [ "${1:-}" = "--list" ]; then
    printf '%s\n' "$TESTS" | cut -d: -f1
    exit 0
fi

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"
printf '%s\n' "$TESTS" > "$work/list"

# The requested names go to a file, one per line, straight from "$@".  Passing
# them through an unquoted variable would subject them to word splitting and
# pathname expansion: run from a directory holding built test binaries, an
# argument like t_* would expand into real names, pass validation, and then
# match nothing literally -- restoring the "0 passed, 0 failed" false green
# this validation exists to prevent.
: > "$work/want"
[ $# -gt 0 ] && printf '%s\n' "$@" > "$work/want"
printf '%s\n' "$TESTS" | cut -d: -f1 > "$work/known"

# Validate before building anything.
if [ -s "$work/want" ]; then
    unknown=$(grep -Fxv -f "$work/known" "$work/want" || true)
    if [ -n "$unknown" ]; then
        echo "run_utests.sh: no such test: $(printf '%s' "$unknown" | tr '\n' ' ')" >&2
        echo "               known tests: $(tr '\n' ' ' < "$work/known")" >&2
        exit 2
    fi
fi

pass=0
fail=0
failed=

while IFS= read -r line; do
    [ -n "$line" ] || continue
    name=${line%%:*}
    extra=${line#*:}

    if [ -s "$work/want" ] && ! grep -Fxq "$name" "$work/want"; then
        continue
    fi

    srcs="$SRC/rtkcmn.c $SRC/trace.c $SRC/preceph.c"
    for m in $extra; do srcs="$srcs $SRC/$m.c"; done

    # -w: these are upstream test programs and their warnings are not this
    # project's to fix.  A build failure still fails the gate.
    # shellcheck disable=SC2086
    if ! "$cc" -std=c99 -w -I"$SRC" $OPTS "$here/$name.c" $srcs $ldlibs \
            -o "$work/$name" 2>"$work/$name.build"; then
        printf '  %-12s BUILD FAILED\n' "$name"
        sed 's/^/      /' "$work/$name.build" | head -8
        fail=$((fail+1)); failed="$failed $name"
        continue
    fi

    # several read data files by relative path, as the WORKING_DIRECTORY in
    # CMakeLists arranges; reproduce that here
    if (cd "$here" && timeout 300 "$work/$name" >"$work/$name.out" 2>&1); then
        printf '  %-12s PASS\n' "$name"
        pass=$((pass+1))
    else
        rc=$?
        printf '  %-12s FAIL (rc=%s)\n' "$name" "$rc"
        tail -8 "$work/$name.out" | sed 's/^/      /'
        fail=$((fail+1)); failed="$failed $name"
    fi
done < "$work/list"

echo
if [ "$fail" -gt 0 ]; then
    echo "$pass passed, $fail failed:$failed"
    exit 1
fi
echo "$pass passed, 0 failed"
