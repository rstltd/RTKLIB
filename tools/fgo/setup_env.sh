#!/bin/sh
# setup_env.sh : create (or recreate) the pinned FGO conda environment.
#
#   tools/fgo/setup_env.sh            create from the solved lock (reproducible)
#   tools/fgo/setup_env.sh --resolve  re-solve from environment.yml, then relock
#
# Requires conda on PATH.  Does not need root.
#
# Environment variables:
#   FGO_ENV_NAME  environment name             (default: rtklib-fgo)
#   FGO_ARCHSPEC  microarchitecture to solve for on --resolve
#                 (default: x86_64_v3, or the host level if that is lower)
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
env_name=${FGO_ENV_NAME:-rtklib-fgo}
lock=$here/conda-linux-64.lock

command -v conda >/dev/null 2>&1 || {
    echo "setup_env.sh: conda not found on PATH" >&2
    exit 1
}

# ---- host microarchitecture level ---------------------------------------------
# conda-forge splits x86-64 into feature levels; the SuiteSparse libraries GTSAM
# links declare a minimum.  Determine what this host can actually run.
host_level=1
if [ -r /proc/cpuinfo ]; then
    f=$(grep -m1 '^flags' /proc/cpuinfo)
    has() { echo "$f" | grep -qw "$1"; }
    has sse4_2 && has popcnt                       && host_level=2
    [ "$host_level" -ge 2 ] && has avx2 && has bmi2 && has fma && host_level=3
    [ "$host_level" -ge 3 ] && has avx512f          && host_level=4
fi

if [ "${1:-}" = "--resolve" ]; then
    # Solve against a floor rather than against whatever this host happens to
    # be.  Without this, re-solving on a v4 machine bakes a v4 requirement into
    # the lock and silently makes it unusable on v3 hardware.  Never ask for
    # more than the host can run, or the solve produces an env it cannot use.
    arch=${FGO_ARCHSPEC:-x86_64_v3}
    want=${arch##*_v}
    if [ "$host_level" -lt "$want" ]; then
        arch=x86_64_v$host_level
        echo "host is only x86-64-v$host_level; solving for $arch"
        echo "note: the resulting lock will be less portable than the default"
    fi

    echo "re-solving $env_name from environment.yml (archspec $arch) ..."
    conda env remove -y -n "$env_name" >/dev/null 2>&1 || true
    CONDA_OVERRIDE_ARCHSPEC=$arch \
        conda env create -y -n "$env_name" -f "$here/environment.yml"
    echo "relocking to conda-linux-64.lock ..."
    conda list -n "$env_name" --explicit > "$lock"
else
    # Restoring an @EXPLICIT lock bypasses dependency solving, so conda will
    # install binaries this CPU cannot execute without complaint and the
    # failure surfaces much later as an illegal instruction.  Check the level
    # the lock itself demands, so this tracks the lock rather than a hardcoded
    # assumption.
    need=$(sed -n 's/.*_x86_64-microarch-level-\([0-9]\)-.*/\1/p' "$lock" |
           head -1)
    need=${need:-1}
    if [ "$host_level" -lt "$need" ]; then
        echo "setup_env.sh: the lock requires x86-64-v$need but this host is" >&2
        echo "              only x86-64-v$host_level.  Re-solve for this host:" >&2
        echo "                  $0 --resolve" >&2
        exit 1
    fi

    # `conda create --file <lock>` against an existing prefix installs the
    # locked packages but does not unlink anything already there, so the result
    # would not be the locked package set.  Start from nothing.
    echo "removing any existing $env_name ..."
    conda env remove -y -n "$env_name" >/dev/null 2>&1 || true
    echo "creating $env_name from conda-linux-64.lock (needs x86-64-v$need) ..."
    conda create -y -n "$env_name" --file "$lock"
fi

prefix=$(conda run -n "$env_name" printenv CONDA_PREFIX)
echo
echo "environment ready: $prefix"
echo "verify it with:  FGO_ENV_PREFIX=$prefix $here/verify_env.sh"
