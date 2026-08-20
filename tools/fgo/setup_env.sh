#!/bin/sh
# setup_env.sh : create (or recreate) the pinned FGO conda environment.
#
#   tools/fgo/setup_env.sh            create from the solved lock (reproducible)
#   tools/fgo/setup_env.sh --resolve  re-solve from environment.yml, then relock
#
# Requires conda on PATH.  Does not need root.
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
env_name=${FGO_ENV_NAME:-rtklib-fgo}

command -v conda >/dev/null 2>&1 || {
    echo "setup_env.sh: conda not found on PATH" >&2
    exit 1
}

# ---- CPU preflight -----------------------------------------------------------
# The SuiteSparse libraries GTSAM links declare `_x86_64-microarch-level >=3`,
# i.e. x86-64-v3 (AVX2 + FMA + BMI2, Haswell 2013 and later).  Restoring an
# @EXPLICIT lock bypasses dependency solving, so conda will happily install
# those binaries on an older CPU and the failure surfaces much later as an
# illegal instruction.  Check up front instead.
if [ -r /proc/cpuinfo ]; then
    missing=
    for f in avx2 bmi2 fma; do
        grep -m1 '^flags' /proc/cpuinfo | grep -qw "$f" || missing="$missing $f"
    done
    if [ -n "$missing" ]; then
        echo "setup_env.sh: this CPU lacks$missing" >&2
        echo "              the locked SuiteSparse/GTSAM builds require" >&2
        echo "              x86-64-v3; re-solve on this host with:" >&2
        echo "                  $0 --resolve" >&2
        exit 1
    fi
fi

if [ "${1:-}" = "--resolve" ]; then
    echo "re-solving $env_name from environment.yml ..."
    conda env remove -y -n "$env_name" >/dev/null 2>&1 || true
    conda env create -y -n "$env_name" -f "$here/environment.yml"
    echo "relocking to conda-linux-64.lock ..."
    conda list -n "$env_name" --explicit > "$here/conda-linux-64.lock"
else
    # `conda create --file <lock>` against an existing prefix installs the
    # locked packages but does not unlink anything already there, so the result
    # would not be the locked package set.  Start from nothing.
    echo "removing any existing $env_name ..."
    conda env remove -y -n "$env_name" >/dev/null 2>&1 || true
    echo "creating $env_name from conda-linux-64.lock ..."
    conda create -y -n "$env_name" --file "$here/conda-linux-64.lock"
fi

prefix=$(conda run -n "$env_name" printenv CONDA_PREFIX)
echo
echo "environment ready: $prefix"
echo "verify it with:  FGO_ENV_PREFIX=$prefix $here/verify_env.sh"
