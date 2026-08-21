#!/bin/sh
# check_abi.sh : C/C++ boundary checks for the RTKLIB FGO extension.
#
# Covers the two halves of plan.md 6.11 gate G2 that concern the language
# boundary, and the acceptance criterion for the 6.1 M2/M4/M5 visibility
# promotions:
#
#   1. every src/*.c still compiles as strict C99 with -Wall -pedantic.
#      This is the "no C++ leakage" check of invariant I2: as src/fgo/ grows,
#      nothing it introduces may make the shared headers un-compilable as C.
#   2. a C++ translation unit includes rtklib.h, links against the C objects,
#      and calls the promoted helpers -- invariant I3, and the entire point of
#      promoting them.
#
#   test/abi/check_abi.sh
#
# Environment variables:
#   FGO_CC     C compiler   (default /usr/bin/gcc)
#   FGO_CXX    C++ compiler (default /usr/bin/g++)
#   FGO_LDLIBS link libs    (default -lm)
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RTKLIB_ROOT=$(CDPATH= cd -- "$here/../.." && pwd)
work=${TMPDIR:-/tmp}/rtklib-abi.$$

cc=${FGO_CC:-/usr/bin/gcc}
cxx=${FGO_CXX:-/usr/bin/g++}
ldlibs=${FGO_LDLIBS:--lm}

# see docs/fgo/build_environment.md: an activated conda shell exports these
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

# must match app/consapp/rnx2rtkp/gcc/makefile, since the C++ probe links the
# objects that build produces
OPTS="-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=4 -DNEXOBS=3"

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"

# ---- 1. strict C99, no C++ leakage (I2) --------------------------------------
echo "== strict C99 compile of src/*.c =="
n=0
bad=0
for f in "$RTKLIB_ROOT"/src/*.c; do
    n=$((n+1))
    # shellcheck disable=SC2086
    # Only -Isrc: rtkpos.c reaches the FGO header as "fgo/rtklib_fgo_api.h",
    # so no build system needs an extra include directory for it.
    if ! "$cc" -fsyntax-only -std=c99 -Wall -pedantic \
            -Wno-unused-but-set-variable \
            -I"$RTKLIB_ROOT/src" $OPTS "$f" 2>"$work/cc.err"; then
        echo "  FAIL $(basename "$f")"
        sed 's/^/      /' "$work/cc.err" | head -5
        bad=$((bad+1))
    fi
done
if [ "$bad" = 0 ]; then
    echo "  PASS $n files compile as C99"
else
    echo "  $bad of $n files failed"
fi

# ---- 2. C++ can include, link and call (I3) ----------------------------------
echo "== C++ boundary probe =="
src=$RTKLIB_ROOT/app/consapp/rnx2rtkp/gcc
make -B -s -C "$src" CC="$cc" LDLIBS="$ldlibs" >"$work/build.log" 2>&1 || {
    echo "  FAIL could not build the C objects"
    tail -15 "$work/build.log"
    exit 1
}

# every object of the reference build except the app's own main
objs=$(ls "$src"/*.o | grep -v '/rnx2rtkp\.o$' | tr '\n' ' ')

# shellcheck disable=SC2086
"$cxx" -std=c++17 -Wall -Wextra -pedantic -I"$RTKLIB_ROOT/src" \
    -I"$RTKLIB_ROOT/src/fgo" $OPTS \
    "$here/cxx_abi_probe.cpp" $objs $ldlibs -o "$work/cxx_abi_probe" \
    2>"$work/cxx.err" || {
    echo "  FAIL C++ compile/link"
    sed 's/^/      /' "$work/cxx.err" | head -20
    exit 1
}

"$work/cxx_abi_probe" || bad=$((bad+1))

echo
[ "$bad" = 0 ] || { echo "$bad check(s) failed"; exit 1; }
echo "all ABI checks passed"
