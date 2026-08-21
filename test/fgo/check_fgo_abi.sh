#!/bin/sh
# check_fgo_abi.sh : the FGO C ABI boundary and its disabled-build stub.
#
#   test/fgo/check_fgo_abi.sh
#
# Checks, in order:
#   1. rtklib_fgo_api.h contains no C++ and no backend headers -- it is the one
#      file both languages include, so a leak here contaminates the core
#   2. it compiles standalone as strict C99 and as C++17
#   3. fgo_stub.c compiles as strict C99
#   4. the stub behaves the way rtkpos.c will need it to (test/fgo/fgo_abi_probe.c)
#   5. a C++ translation unit links against that C stub, which is what proves
#      the extern "C" guard is right
#
# Environment variables:
#   FGO_CC / FGO_CXX / FGO_LDLIBS  as elsewhere
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RTKLIB_ROOT=$(CDPATH= cd -- "$here/../.." && pwd)
work=${TMPDIR:-/tmp}/rtklib-fgoabi.$$

cc=${FGO_CC:-/usr/bin/gcc}
cxx=${FGO_CXX:-/usr/bin/g++}
ldlibs=${FGO_LDLIBS:--lm}

# see docs/fgo/build_environment.md
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

OPTS="-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=4 -DNEXOBS=3"
INC="-I$RTKLIB_ROOT/src -I$RTKLIB_ROOT/src/fgo"
api=$RTKLIB_ROOT/src/fgo/rtklib_fgo_api.h

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"

bad=0

# ---- 1. nothing from the backend may appear in the shared header -----------
echo "== header purity =="
# Strip comments first, so the documented try/catch template -- which of course
# mentions std::exception -- is not mistaken for C++ in the declarations.
# -fpreprocessed leaves #include and #define alone and does not expand them,
# so this looks at exactly the code the header contributes.
"$cc" -fpreprocessed -E -P -x c "$api" > "$work/api.stripped" 2>/dev/null ||     cp "$api" "$work/api.stripped"
leaks=$(grep -nE '#[[:space:]]*include[[:space:]]*[<"](gtsam|Eigen|boost)' \
        "$work/api.stripped" || true)
leaks="$leaks$(grep -nE '(^|[^[:alnum:]_])(namespace|class|template|nullptr|public:|std::)' \
        "$work/api.stripped" || true)"
if [ -n "$leaks" ]; then
    echo "  FAIL C++ or backend content in rtklib_fgo_api.h"
    printf '%s\n' "$leaks" | sed 's/^/      /'
    bad=$((bad+1))
else
    echo "  PASS no C++ or GTSAM/Eigen/Boost content in the declarations"
fi

# ---- 2/3. both languages, strict ------------------------------------------
echo "== compiles in both languages =="
# shellcheck disable=SC2086
if "$cc" -fsyntax-only -std=c99 -Wall -Wextra -pedantic $INC $OPTS -x c "$api" \
        2>"$work/e" ; then
    echo "  PASS header as strict C99"
else
    echo "  FAIL header as strict C99"; sed 's/^/      /' "$work/e"; bad=$((bad+1))
fi
# shellcheck disable=SC2086
if "$cxx" -fsyntax-only -std=c++17 -Wall -Wextra -pedantic $INC $OPTS -x c++ \
        "$api" 2>"$work/e"; then
    echo "  PASS header as C++17"
else
    echo "  FAIL header as C++17"; sed 's/^/      /' "$work/e"; bad=$((bad+1))
fi
# shellcheck disable=SC2086
if "$cc" -fsyntax-only -std=c99 -Wall -Wextra -pedantic $INC $OPTS \
        "$RTKLIB_ROOT/src/fgo/fgo_stub.c" 2>"$work/e"; then
    echo "  PASS fgo_stub.c as strict C99"
else
    echo "  FAIL fgo_stub.c as strict C99"; sed 's/^/      /' "$work/e"; bad=$((bad+1))
fi

# ---- 4/5. behaviour, from C and from C++ ----------------------------------
src=$RTKLIB_ROOT/app/consapp/rnx2rtkp/gcc
make -B -s -C "$src" CC="$cc" LDLIBS="$ldlibs" >"$work/build.log" 2>&1 || {
    echo "check_fgo_abi.sh: could not build the C objects" >&2
    tail -15 "$work/build.log" >&2
    exit 1
}
objs=$(ls "$src"/*.o | grep -v '/rnx2rtkp\.o$' | tr '\n' ' ')

echo "== stub behaviour (C) =="
# shellcheck disable=SC2086
"$cc" -std=c99 -Wall -Wextra -pedantic -Wno-unused-but-set-variable $INC $OPTS \
    "$here/fgo_abi_probe.c" "$RTKLIB_ROOT/src/fgo/fgo_stub.c" $objs $ldlibs \
    -o "$work/probe" 2>"$work/e" || {
    echo "  FAIL compile/link"; sed 's/^/      /' "$work/e" | head -20; exit 1; }
"$work/probe" || bad=$((bad+1))

echo "== C++ links against the C stub =="
# The stub must be compiled by the C compiler into an object, and only then
# linked with the C++ one.  Handing fgo_stub.c straight to $cxx would compile
# it as C++, so both sides would mangle identically and the test would pass
# even with the extern "C" guard deleted -- which is the one regression it
# exists to catch.
# shellcheck disable=SC2086
"$cc" -c -std=c99 -Wall -Wextra -pedantic $INC $OPTS \
    "$RTKLIB_ROOT/src/fgo/fgo_stub.c" -o "$work/fgo_stub.o" 2>"$work/e" || {
    echo "  FAIL compiling the stub as C"; sed 's/^/      /' "$work/e" | head -20
    exit 1; }
# shellcheck disable=SC2086
"$cxx" -std=c++17 -Wall -Wextra -pedantic $INC $OPTS \
    "$here/fgo_abi_cxx.cpp" "$work/fgo_stub.o" $objs $ldlibs \
    -o "$work/probecxx" 2>"$work/e" || {
    echo "  FAIL C++ cannot link the C-compiled stub"
    echo "       (is the extern \"C\" guard in rtklib_fgo_api.h intact?)"
    sed 's/^/      /' "$work/e" | head -20; bad=$((bad+1)); }
[ -x "$work/probecxx" ] && { "$work/probecxx" || bad=$((bad+1)); }

echo
[ "$bad" = 0 ] || { echo "$bad check(s) failed"; exit 1; }
echo "all FGO ABI checks passed"
