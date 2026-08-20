#!/bin/sh
# run_regression.sh : byte-diff regression gate for the RTKLIB EKF path.
#
# This is gate G1 of plan.md 6.11.  Design invariant I1 requires the EKF path
# to stay bit-for-bit identical across the whole FGO project, and this is what
# enforces it.  Every PR that touches src/ must pass it -- above all PR-2, the
# ddres() purification, which rewrites core EKF code while intending to change
# nothing about its output.
#
#   test/regression/run_regression.sh              verify every dataset
#   test/regression/run_regression.sh <name> ...   verify only these datasets
#   test/regression/run_regression.sh --update     regenerate the baselines
#   test/regression/run_regression.sh --list       list known datasets
#
# Environment variables:
#   FGO_CC          C compiler for the reference build (default /usr/bin/gcc)
#   FGO_LDLIBS      link libraries                     (default -lm)
#   FGO_RNX2RTKP    use this prebuilt binary instead of building one
#   FGO_DATA_ROOT   root for datasets kept outside the repository
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RTKLIB_ROOT=$(CDPATH= cd -- "$here/../.." && pwd)
ds_dir=$here/datasets
work=${TMPDIR:-/tmp}/rtklib-regression.$$

cc=${FGO_CC:-/usr/bin/gcc}
ldlibs=${FGO_LDLIBS:--lm}

# An activated conda shell exports CFLAGS/LDFLAGS that make would fold into the
# build, changing optimisation and RPATHs underneath a comparison that is
# supposed to be byte-exact.  See docs/fgo/build_environment.md.
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS

mode=verify
datasets=
for a in "$@"; do
    case $a in
    --update) mode=update ;;
    --list)   mode=list ;;
    -*)       echo "run_regression.sh: unknown option $a" >&2; exit 2 ;;
    *)        datasets="$datasets $a" ;;
    esac
done

[ -n "$datasets" ] || datasets=$(cd "$ds_dir" && ls -d */ 2>/dev/null | tr -d /)

if [ "$mode" = list ]; then
    for d in $datasets; do
        desc=$(sed -n 's/^description *= *//p' "$ds_dir/$d/dataset.conf" 2>/dev/null)
        printf '%-24s %s\n' "$d" "$desc"
    done
    exit 0
fi

cleanup() { rm -rf "$work"; }
trap cleanup EXIT INT TERM
mkdir -p "$work"

# ---- reference binary --------------------------------------------------------
# Built with -B on purpose: the makefile leaves its objects in the source tree,
# so an incremental build could silently link objects produced by a different
# compiler than the one this run claims to have used.
if [ -n "${FGO_RNX2RTKP:-}" ]; then
    bin=$FGO_RNX2RTKP
    [ -x "$bin" ] || { echo "run_regression.sh: $bin is not executable" >&2; exit 1; }
    echo "using prebuilt binary: $bin"
else
    src=$RTKLIB_ROOT/app/consapp/rnx2rtkp/gcc
    echo "building rnx2rtkp (CC=$cc LDLIBS=\"$ldlibs\") ..."
    make -B -s -C "$src" CC="$cc" LDLIBS="$ldlibs" >"$work/build.log" 2>&1 || {
        echo "run_regression.sh: build failed" >&2
        tail -20 "$work/build.log" >&2
        exit 1
    }
    bin=$src/rnx2rtkp
fi

# ---- dataset field lookup ----------------------------------------------------
# Only $RTKLIB_ROOT and $FGO_DATA_ROOT are expanded, so a dataset file cannot
# run arbitrary shell.  Large real datasets live outside the repo and are
# referenced through $FGO_DATA_ROOT.
field() {
    v=$(sed -n "s/^$2 *= *//p" "$1" | head -1)
    v=${v%%#*}
    v=$(printf '%s' "$v" | sed -e 's/[[:space:]]*$//')
    printf '%s' "$v" |
        sed -e "s|\${RTKLIB_ROOT}|$RTKLIB_ROOT|g" -e "s|\$RTKLIB_ROOT|$RTKLIB_ROOT|g" \
            -e "s|\${FGO_DATA_ROOT}|${FGO_DATA_ROOT:-}|g" -e "s|\$FGO_DATA_ROOT|${FGO_DATA_ROOT:-}|g"
}

# ---- normalisation -----------------------------------------------------------
# The .pos header records the absolute paths of the inputs and the program
# version, so it is machine- and checkout-dependent and cannot be compared
# verbatim.  Paths collapse to basenames; the version is masked because plan.md
# M19 bumps PATCH_LEVEL in the very PR that adds the FGO fields, which would
# otherwise invalidate every baseline for a reason unrelated to the numbers.
# The version actually used is recorded in the baseline metadata instead.
#
# The .stat file has no header at all and is compared byte for byte untouched.
# It is the sensitive one: it carries per-satellite residuals, so it catches
# differences long before they are visible in the position solution.
normalize_pos() {
    sed -e 's|^\(% inp file  : \).*/|\1|' \
        -e 's|^\(% program   : rnx2rtkp ver\.\).*|\1<masked>|'
}

fail=0
pass=0
for d in $datasets; do
    conf=$ds_dir/$d/dataset.conf
    [ -f "$conf" ] || { echo "SKIP $d (no dataset.conf)"; continue; }

    rover=$(field "$conf" rover)
    base=$(field "$conf" base)
    nav=$(field "$conf" nav)
    args=$(field "$conf" args)
    opts=$ds_dir/$d/$(field "$conf" opts)

    missing=
    for f in "$rover" "$base" "$nav" "$opts"; do
        [ -n "$f" ] && [ -f "$f" ] || missing="$missing $f"
    done
    if [ -n "$missing" ]; then
        echo "SKIP $d (missing:$missing)"
        [ "$mode" = update ] && { echo "  cannot update a dataset with missing inputs" >&2; fail=$((fail+1)); }
        continue
    fi

    out=$work/$d
    mkdir -p "$out"
    # -y 2 emits the solution-status file.  plan.md 6.11 G1 says -x 2, but that
    # sets the trace level and produces a .trace file; the .stat file this gate
    # depends on comes from -y.  The file is named <outfile>.stat.
    # shellcheck disable=SC2086
    "$bin" -k "$opts" -y 2 -o "$out/solution.pos" $args "$rover" "$base" "$nav" \
        >"$out/run.log" 2>&1 || {
        echo "FAIL $d (rnx2rtkp exited non-zero)"
        tail -5 "$out/run.log" >&2
        fail=$((fail+1))
        continue
    }

    for f in solution.pos solution.pos.stat; do
        [ -s "$out/$f" ] || {
            echo "FAIL $d (no $f produced)"
            fail=$((fail+1))
            continue 2
        }
    done

    normalize_pos < "$out/solution.pos" > "$out/solution.pos.norm"

    bdir=$ds_dir/$d/baseline
    if [ "$mode" = update ]; then
        mkdir -p "$bdir"
        cp "$out/solution.pos.norm" "$bdir/solution.pos"
        cp "$out/solution.pos.stat" "$bdir/solution.stat"
        {
            echo "# baseline metadata for dataset '$d'"
            echo "# regenerated by test/regression/run_regression.sh --update"
            echo "git-commit  : $(git -C "$RTKLIB_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
            # only src/ and app/ matter: they are what ends up in the binary.
            # Untracked files count too -- a stray .c in src/ changes the build.
            echo "git-dirty   : $(test -n "$(git -C "$RTKLIB_ROOT" status --porcelain -- src app 2>/dev/null)" && echo YES || echo no)"
            echo "rnx2rtkp    : $("$bin" --version 2>&1 | head -1)"
            echo "cc          : $cc"
            echo "cc-version  : $("$cc" --version | head -1)"
            echo "ldlibs      : $ldlibs"
            echo "uname       : $(uname -srm)"
            echo "epochs      : $(grep -vc '^%' "$bdir/solution.pos")"
            echo "stat-lines  : $(wc -l < "$bdir/solution.stat" | tr -d ' ')"
        } > "$bdir/metadata.txt"
        echo "UPDATED $d ($(grep -vc '^%' "$bdir/solution.pos") epochs)"
        pass=$((pass+1))
        continue
    fi

    if [ ! -f "$bdir/solution.pos" ] || [ ! -f "$bdir/solution.stat" ]; then
        echo "FAIL $d (no baseline; run with --update)"
        fail=$((fail+1))
        continue
    fi

    ok=1
    if ! cmp -s "$out/solution.pos.norm" "$bdir/solution.pos"; then
        echo "FAIL $d (.pos differs from baseline)"
        diff -u "$bdir/solution.pos" "$out/solution.pos.norm" | head -12 | sed 's/^/    /'
        ok=0
    fi
    if ! cmp -s "$out/solution.pos.stat" "$bdir/solution.stat"; then
        echo "FAIL $d (.stat differs from baseline)"
        diff -u "$bdir/solution.stat" "$out/solution.pos.stat" | head -12 | sed 's/^/    /'
        ok=0
    fi
    if [ "$ok" = 1 ]; then
        echo "PASS $d ($(grep -vc '^%' "$bdir/solution.pos") epochs, byte-identical)"
        pass=$((pass+1))
    else
        fail=$((fail+1))
    fi
done

echo
echo "$pass passed, $fail failed"
[ "$fail" = 0 ] || exit 1
