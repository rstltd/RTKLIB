#!/bin/sh
# check_fgo_backend.sh : the ENABLE_FGO=ON build (plan.md 11.3 PR-6).
#
#   test/fgo/check_fgo_backend.sh
#
# Skips cleanly, with exit 0, when GTSAM is not installed.  Most machines that
# touch this repository will never build the backend, and a missing optional
# dependency is not a test failure -- but it is also not a pass, so it says so.
#
# Checks:
#   1. every extern "C" definition in src/fgo/ carries the exception boundary.
#      A C++ exception reaching C is undefined behaviour (plan.md 6.10 RC-9),
#      and GTSAM really does throw -- Symbol on key overflow, and the linear
#      solver on a rank-deficient graph.  Checked mechanically because a review
#      checklist item is only as good as the reviewer's attention.
#   2. the library builds with ENABLE_FGO=ON and exports the C ABI
#   3. ENABLE_FGO=OFF links no C++ runtime at all (invariant I5, risk RC-5)
#   4. the backend behaves, and agrees with the stub wherever it must
#
# Environment variables:
#   FGO_ENV_PREFIX  conda prefix holding GTSAM (default: the rtklib-fgo env)
#   FGO_CC / FGO_CXX
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RTKLIB_ROOT=$(CDPATH= cd -- "$here/../.." && pwd)
work=${TMPDIR:-/tmp}/rtklib-fgobackend.$$

prefix=${FGO_ENV_PREFIX:-$HOME/miniconda3/envs/rtklib-fgo}
cc=${FGO_CC:-/usr/bin/gcc}
cxx=${FGO_CXX:-/usr/bin/g++}
cmake_bin=${FGO_CMAKE:-$prefix/bin/cmake}
ninja_bin=${FGO_NINJA:-$prefix/bin/ninja}

# see docs/fgo/build_environment.md
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"

bad=0

# ---- 1. the exception boundary, checked statically -------------------------
# This one needs no GTSAM, so it runs even when the rest is skipped.
echo "== exception boundary =="
missing=
for f in "$RTKLIB_ROOT"/src/fgo/*.cpp; do
    [ -f "$f" ] || continue
    # fgo_enabled is the one exemption: it returns a constant and executes no
    # C++ that could throw, and wrapping it would make a function documented as
    # never failing able to return an error code.
    names=$(grep -oE '^extern "C" [a-zA-Z_][a-zA-Z0-9_ *]*[ *]([a-zA-Z0-9_]+)\(' "$f" |
            sed -E 's/.*[ *]([a-zA-Z0-9_]+)\($/\1/' | grep -v '^fgo_enabled$' || true)
    for n in $names; do
        # take the function body from its definition to the next one
        body=$(awk -v fn="$n" '
            $0 ~ "^extern \"C\" .*[ *]" fn "\\(" {inside=1}
            inside {print}
            inside && /^}/ {exit}' "$f")
        printf '%s' "$body" | grep -q 'FGO_BOUNDARY_BEGIN' ||
            missing="$missing $(basename "$f"):$n"
    done
done
if [ -n "$missing" ]; then
    echo "  FAIL missing FGO_BOUNDARY:$missing"
    bad=$((bad+1))
else
    echo "  PASS every extern \"C\" definition is wrapped"
fi

# ---- is a backend build possible at all? -----------------------------------
if [ ! -f "$prefix/lib/cmake/GTSAM/GTSAMConfig.cmake" ] || [ ! -x "$cmake_bin" ]; then
    echo
    echo "SKIP: no GTSAM under '$prefix'; the ENABLE_FGO=ON build is not checked."
    echo "      Run tools/fgo/setup_env.sh to enable it."
    [ "$bad" = 0 ] || exit 1
    exit 0
fi

cfg() { # <builddir> <ON|OFF>
    FGO_ENV_PREFIX=$prefix FGO_CC=$cc FGO_CXX=$cxx \
    "$cmake_bin" -S "$RTKLIB_ROOT" -B "$1" -G Ninja \
        -DCMAKE_MAKE_PROGRAM="$ninja_bin" \
        --toolchain "$RTKLIB_ROOT/tools/fgo/fgo-toolchain.cmake" \
        -DCMAKE_BUILD_TYPE=Release -DENABLE_FGO="$2" -DBUILD_TESTING=OFF \
        >"$work/cfg.$2.log" 2>&1
}

lib=$RTKLIB_ROOT/lib/librtklib.so.2.0.0

echo "== ENABLE_FGO=ON build =="
rm -f "$RTKLIB_ROOT"/lib/librtklib.so*
if ! cfg "$work/on" ON; then
    echo "  FAIL configure"; tail -15 "$work/cfg.ON.log" | sed 's/^/      /'; exit 1
fi
if ! FGO_ENV_PREFIX=$prefix "$cmake_bin" --build "$work/on" --target rtklib \
        >"$work/build.on.log" 2>&1; then
    echo "  FAIL build"; grep -E "error" "$work/build.on.log" | head -15 | sed 's/^/      /'
    exit 1
fi
nm -D "$lib" | sed -n 's/.* T \(fgo_[A-Za-z0-9_]*\)$/\1/p' | sort > "$work/sym.on"
echo "  exports $(wc -l < "$work/sym.on" | tr -d ' ') fgo_ symbols"
if [ "$(ldd "$lib" | grep -c gtsam)" -gt 0 ]; then
    echo "  PASS links GTSAM"
else
    echo "  FAIL GTSAM not linked"; bad=$((bad+1))
fi

# ---- 4. behaviour ----------------------------------------------------------
echo "== backend behaviour =="
# The probe MUST be compiled with exactly the macro set the library was.
# ENAGLO and friends size MAXSAT, NFREQ and NEXOBS size obsd_t and ssat_t, so
# a mismatch silently relocates every field of rtk_t.  These are not read from
# a header: they live in add_definitions() at the top of CMakeLists.txt and
# reach the library through the build system alone, so they are read from
# there rather than copied, which would drift.
#
# Getting this wrong is quiet and confusing rather than loud.  Two earlier
# versions of this script guessed: one dropped the defines entirely, moving
# MAXSAT from 208 to 71 and sizeof(rtk_t) from 183 KB to 70 KB, and
# fgo_init() then returned FGO_OK while the caller read rtk->fgo as NULL from
# a different offset; the other kept NEXOBS at 0 against the library's 3.
rtkdefs=$(sed -n 's/^add_definitions(\(-D.*\))$/\1/p' "$RTKLIB_ROOT/CMakeLists.txt" | head -1)
[ -n "$rtkdefs" ] || {
    echo "  FAIL cannot read add_definitions() from CMakeLists.txt" >&2
    exit 1
}
echo "  using library macros: $rtkdefs"
# shellcheck disable=SC2086
# GTSAM's own headers do not survive -Wextra -pedantic cleanly, so they come
# in via -isystem; the probe's own code is still held to the strict flags.
#
# TBB is a transitive dependency: the conda GTSAM is built with it, so header
# code instantiated here (the factor templates) pulls in tbb allocators that
# libgtsam alone does not satisfy.  A CMake consumer gets this automatically
# from the GTSAM target; a hand-built probe has to say it.
"$cxx" -std=c++17 -Wall -Wextra -pedantic \
    -I"$RTKLIB_ROOT/src" -I"$RTKLIB_ROOT/src/fgo" \
    -isystem "$prefix/include" -isystem "$prefix/include/eigen3" $rtkdefs \
    "$here/fgo_backend_probe.cpp" \
    -L"$RTKLIB_ROOT/lib" -Wl,-rpath,"$RTKLIB_ROOT/lib" \
    -L"$prefix/lib" -Wl,-rpath,"$prefix/lib" \
    -lrtklib -lgtsam -lgtsam_unstable -ltbb -ltbbmalloc -lm \
    -o "$work/probe" 2>"$work/e" || {
    echo "  FAIL compile/link"; sed 's/^/      /' "$work/e" | head -20; exit 1; }
data=$RTKLIB_ROOT/test/data/rinex
"$work/probe" "$data/07590920.05o" "$data/30400920.05o" "$data/30400920.05n" ||
    bad=$((bad+1))

# ---- 3. the OFF build must stay free of the C++ runtime --------------------
echo "== ENABLE_FGO=OFF stays pure C =="
rm -f "$RTKLIB_ROOT"/lib/librtklib.so*
if ! cfg "$work/off" OFF || ! FGO_ENV_PREFIX=$prefix "$cmake_bin" \
        --build "$work/off" --target rtklib >"$work/build.off.log" 2>&1; then
    echo "  FAIL build"; tail -10 "$work/build.off.log" | sed 's/^/      /'; exit 1
fi
cxxrt=$(ldd "$lib" | grep -cE 'stdc\+\+|libgtsam' || true)
if [ "$cxxrt" = 0 ]; then
    echo "  PASS no C++ runtime or GTSAM linked"
else
    echo "  FAIL C++ runtime leaked into the disabled build"; bad=$((bad+1))
fi

# The two builds must present the SAME C ABI -- that is the entire premise of
# the stub, and what lets rtkpos.c call these without knowing which is linked.
# Counting symbols would not catch a callback quietly dropped from one side,
# so the sets are compared.
nm -D "$lib" | sed -n 's/.* T \(fgo_[A-Za-z0-9_]*\)$/\1/p' | sort > "$work/sym.off"
if diff -u "$work/sym.off" "$work/sym.on" > "$work/sym.diff"; then
    echo "  PASS ON and OFF export the same $(wc -l < "$work/sym.on" | tr -d ' ') fgo_ symbols"
else
    echo "  FAIL the two builds export different symbol sets"
    sed -n '3,$p' "$work/sym.diff" | sed 's/^/      /' | head -12
    bad=$((bad+1))
fi
rm -f "$RTKLIB_ROOT"/lib/librtklib.so*

echo
[ "$bad" = 0 ] || { echo "$bad check(s) failed"; exit 1; }
echo "all FGO backend checks passed"
