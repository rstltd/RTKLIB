#!/bin/sh
# check_options.sh : gate G3 of plan.md 6.11 -- configuration compatibility.
#
#   test/options/check_options.sh
#
# Builds and runs test/options/opts_compat.c against a real pre-FGO config
# file from data/config/.  See that file for what is checked and why.
#
# Environment variables:
#   FGO_CC      C compiler (default /usr/bin/gcc)
#   FGO_LDLIBS  link libs  (default -lm)
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RTKLIB_ROOT=$(CDPATH= cd -- "$here/../.." && pwd)
work=${TMPDIR:-/tmp}/rtklib-opts.$$

cc=${FGO_CC:-/usr/bin/gcc}
ldlibs=${FGO_LDLIBS:--lm}

# see docs/fgo/build_environment.md
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

# must match the reference build so the compiled NFREQ and MAXSTRPATH agree
OPTS="-DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=4 -DNEXOBS=3"

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"

src=$RTKLIB_ROOT/app/consapp/rnx2rtkp/gcc
make -B -s -C "$src" CC="$cc" LDLIBS="$ldlibs" >"$work/build.log" 2>&1 || {
    echo "check_options.sh: could not build the C objects" >&2
    tail -15 "$work/build.log" >&2
    exit 1
}
objs=$(ls "$src"/*.o | grep -v '/rnx2rtkp\.o$' | tr '\n' ' ')

# shellcheck disable=SC2086
"$cc" -std=c99 -Wall -pedantic -Wno-unused-but-set-variable \
    -I"$RTKLIB_ROOT/src" $OPTS "$here/opts_compat.c" $objs $ldlibs \
    -o "$work/opts_compat" 2>"$work/cc.err" || {
    echo "check_options.sh: compile/link failed" >&2
    sed 's/^/    /' "$work/cc.err" >&2
    exit 1
}

legacy=$RTKLIB_ROOT/data/config/f9p_ppk.conf
[ -f "$legacy" ] || {
    echo "check_options.sh: no legacy config at $legacy" >&2
    exit 1
}

"$work/opts_compat" "$legacy" "$work"
