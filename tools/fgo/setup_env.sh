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

if [ "${1:-}" = "--resolve" ]; then
    echo "re-solving $env_name from environment.yml ..."
    conda env remove -y -n "$env_name" >/dev/null 2>&1 || true
    conda env create -y -n "$env_name" -f "$here/environment.yml"
    echo "relocking to conda-linux-64.lock ..."
    conda list -n "$env_name" --explicit > "$here/conda-linux-64.lock"
else
    echo "creating $env_name from conda-linux-64.lock ..."
    conda create -y -n "$env_name" --file "$here/conda-linux-64.lock"
fi

prefix=$(conda run -n "$env_name" printenv CONDA_PREFIX)
echo
echo "environment ready: $prefix"
echo "verify it with:  FGO_ENV_PREFIX=$prefix $here/verify_env.sh"
