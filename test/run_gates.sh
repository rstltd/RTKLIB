#!/bin/sh
# run_gates.sh : run every mandatory gate for the FGO work.
#
#   test/run_gates.sh           run all gates
#   test/run_gates.sh <name>... run only these
#   test/run_gates.sh --list    list them
#
#   regression  G1  EKF output stays byte-identical (invariant I1)
#   abi         G2  language boundary: strict C99, and C++ can call in (I2/I3)
#   options     G3  config compatibility, FGO stays off by default (RC-3)
#   unit        G4  the existing RTKLIB unit tests
#   fgo-abi         the C ABI header and its disabled-build stub
#
# Every gate is independently runnable; this only sequences them and gives a
# single exit status, so CI and a developer run the same thing.
#
# WHAT THIS DOES NOT COVER.  Passing here is necessary, not sufficient.  Two
# parts of the G2 build matrix cannot run from a single developer machine and
# belong to CI (plan.md 11.1 P1.4):
#
#   - a second compiler.  Everything here builds with one toolchain, pinned by
#     FGO_CC/FGO_CXX; the matrix also requires clang.
#   - ENABLE_FGO=ON.  Refused at configure time until the GTSAM backend lands
#     (plan.md 11.3 PR-6), so there is nothing to build yet.
#
# The closing message says so rather than claiming completeness.
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

gate_regression() { "$here/regression/run_regression.sh"; }
gate_abi()        { "$here/abi/check_abi.sh"; }
gate_options()    { "$here/options/check_options.sh"; }
gate_unit()       { "$here/utest/run_utests.sh"; }
gate_fgo_abi()    { "$here/fgo/check_fgo_abi.sh"; }

ALL="regression abi options unit fgo-abi"

if [ "${1:-}" = "--list" ]; then
    echo "regression  G1  EKF output byte-identical"
    echo "abi         G2  C99 purity and C++ callability"
    echo "options     G3  config compatibility"
    echo "unit        G4  existing RTKLIB unit tests"
    echo "fgo-abi         FGO C ABI header and stub"
    exit 0
fi

[ $# -gt 0 ] && ALL="$*"

failed=
for g in $ALL; do
    echo "########## $g ##########"
    case $g in
    regression) run=gate_regression ;;
    abi)        run=gate_abi ;;
    options)    run=gate_options ;;
    unit)       run=gate_unit ;;
    fgo-abi)    run=gate_fgo_abi ;;
    *)          echo "run_gates.sh: unknown gate '$g'" >&2; exit 2 ;;
    esac
    if $run; then
        echo "---------- $g OK"
    else
        echo "---------- $g FAILED"
        failed="$failed $g"
    fi
    echo
done

if [ -n "$failed" ]; then
    echo "FAILED gates:$failed"
    exit 1
fi
echo "all gates passed"
echo
echo "not covered here (CI, plan.md 11.1 P1.4):"
echo "  - a second compiler; this run used a single pinned toolchain"
echo "  - ENABLE_FGO=ON, refused until the backend lands (plan.md 11.3 PR-6)"
