# FGO Build Environment

Pinned toolchain for the Factor Graph Optimization extension described in
[`plan.md`](../../plan.md).

This document covers **Phase 2 work item P2.1** (GTSAM environment) and
**Phase 1 work item P1.3** (fix and record the build configuration). Both are
prerequisites for everything else: P1.3 in particular gates the byte-diff
regression that protects the EKF path (invariant I1, gate G1).

## Quick start

```sh
tools/fgo/setup_env.sh                                   # create the env
FGO_ENV_PREFIX=$(conda run -n rtklib-fgo printenv CONDA_PREFIX) \
    tools/fgo/verify_env.sh                              # prove it works
```

`verify_env.sh` builds and runs `tools/fgo/gtsam_smoke`, which probes every
GTSAM capability the design depends on, and writes the toolchain metadata that
must accompany any regression baseline.

## What is pinned, and by what

| Concern | Source | Rationale |
|---|---|---|
| C / C++ compilers | **system** `/usr/bin/gcc`, `/usr/bin/g++` (12.2.0) | plan.md §6.9 traps 1–2: the EKF byte-diff baseline must be reproducible, so the compiler must not drift. A conda toolchain changes on every `conda update`. |
| GTSAM, Eigen, Boost, TBB, CMake | conda env `rtklib-fgo` | plan.md §3.5 / D2 requires GTSAM 4.2+; conda-forge gives an exact, root-free, lockable pin. |
| Exact package set | `tools/fgo/conda-linux-64.lock` | Reproducible re-creation without re-solving. |
| Wiring of the two | `tools/fgo/fgo-toolchain.cmake` | Single place that joins system compilers to conda libraries. |

plan.md §11.2 P2.1 suggests a Docker image. A conda environment plus an
explicit lock file gives the same version pinning without requiring a container
runtime; the lock can be baked into a Dockerfile later unchanged.

## Resolved versions

Verified on 2026-08-20, Debian 12 (Linux 6.1.0-51-amd64):

| Package | Version |
|---|---|
| gtsam | 4.2.2 (`py311hf34e0dd_0`) |
| eigen | 5.0.1 |
| libboost-devel | 1.90.0 |
| tbb / tbb-devel | 2023.0.0 |
| metis | 5.1.0 |
| libstdcxx (conda) | 16.1.0 |
| cmake | 4.4.2 |
| ninja | 1.13.2 |
| python | 3.11.15 |
| gcc / g++ (system) | 12.2.0 |

## Capability probe

`tools/fgo/gtsam_smoke` is not an RTKLIB test. It asserts that the *pinned
GTSAM build* actually provides what the design assumes, so a mis-featured GTSAM
is caught at setup time rather than midway through `src/fgo/`:

| Probe | plan.md reference |
|---|---|
| batch Levenberg–Marquardt | §4.5.2 `FGO_SOLVER_BATCH` |
| `IncrementalFixedLagSmoother` | §4.5.4 `FGO_SOLVER_ISAM2` (decision D5) |
| `BatchFixedLagSmoother` | §4.5.3 `FGO_SOLVER_SLIDING` |
| Huber / Cauchy / Tukey weights | §5.5.2 — asserts the documented weight table, including that Huber and Cauchy never reach zero while Tukey does |
| `GncOptimizer` (TLS) | §5.5.3 graduated non-convexity |
| full-covariance Gaussian model | §4.2.1 DD block covariance, decision D7 mode (a) |
| marginal covariance recovery | §4.4.2 LAMBDA integration |
| `numericalDerivative11` | §6.11 gate G4 Jacobian verification |

All eight pass on the pinned environment.

## Findings that affect plan.md

### 1. `gtsam_unstable` must also be linked

`IncrementalFixedLagSmoother` and `BatchFixedLagSmoother` live in
`gtsam_unstable`, **not** in core `gtsam`:

```
include/gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h
include/gtsam_unstable/nonlinear/BatchFixedLagSmoother.h
```

plan.md §6.9 M20 specifies `target_link_libraries(rtklib gtsam)`, which is not
enough for the sliding-window and iSAM2 solvers — the two solvers §8.4
recommends for NRT. M20 has been corrected to link `gtsam gtsam_unstable`.

Being in `gtsam_unstable` also means these classes carry no API-stability
guarantee across GTSAM releases. This sharpens risk **RC-10**: the GTSAM
version pin is load-bearing, not merely advisory.

### 2. Eigen resolves to 5.0.1, not 3.x

plan.md §6.9 writes `find_package(Eigen3 3.3 REQUIRED)`. conda-forge's current
`eigen` is 5.0.1 and still exports the CMake package name `Eigen3`, so the
existing call succeeds — `5.0.1 >= 3.3` in CMake's version comparison. GTSAM
4.2.2 here is built with `GTSAM_USE_SYSTEM_EIGEN`, so `src/fgo/` and GTSAM
share one Eigen; no bundled copy is involved.

### 3. GTSAM 4.2 uses `boost::shared_ptr`

`noiseModel::…::shared_ptr` is `boost::shared_ptr` in 4.2 and becomes
`std::shared_ptr` in 4.3. `src/fgo/` must spell these types as
`SomeType::shared_ptr` rather than naming either library directly.

## Environment traps

Each of these cost real debugging time; they are recorded so nobody pays twice.

### `-lgfortran` breaks the console-app makefiles

`app/consapp/*/gcc/makefile` defaults to `LDLIBS = -lgfortran -lm` in its
*no-LAPACK* configuration, where gfortran is not actually used. On a host
without gfortran the link fails. Build these with:

```sh
make CC=/usr/bin/gcc LDLIBS="-lm"
```

This is an upstream quirk, not something to "fix" casually — see plan.md §6.9
trap 2 on not mixing build-configuration changes into the FGO work.

### conda's compiler shadows the system compiler

With miniconda on `PATH`, bare `make` picks up
`/home/…/miniconda3/bin/x86_64-conda-linux-gnu-cc` (GCC 15.2) instead of the
system GCC 12.2, and its default `-L` paths point at the base env rather than
`rtklib-fgo`. Always pass `CC` explicitly for C builds, and use
`fgo-toolchain.cmake` for CMake builds. Silently switching compilers would
invalidate every byte-diff baseline.

### `boost-cpp` cannot be co-installed with GTSAM 4.2.2

`boost-cpp` is the legacy conda-forge metapackage and conflicts with the
`libboost >=1.90` that `gtsam=4.2.2` requires:

```
package gtsam-4.2.2 requires libboost >=1.90.0,<1.91.0a0,
but none of the providers can be installed
```

Use `libboost-devel`.

### `tbb-devel` is a hidden GTSAM build dependency

The conda GTSAM is built with TBB, so `gtsam/base/types.h` includes
`<tbb/scalable_allocator.h>`. The runtime `tbb` package does not ship headers,
so compiling against GTSAM fails with:

```
fatal error: tbb/scalable_allocator.h: No such file or directory
```

`tbb-devel` is therefore a required dependency, and `-ltbb -ltbbmalloc` must be
resolvable at link time — which `fgo-toolchain.cmake` handles via its
`-L`/`-rpath` flags.

### CMake 4.4 warns about `CMP0167` from `GTSAMConfig.cmake`

GTSAM 4.2.2's config calls `find_dependency(Boost)`, and CMake 4.x has removed
the bundled `FindBoost` module. The warning is emitted by GTSAM's own config
file, is developer-facing only, and does not affect the build. Boost is still
found through `BoostConfig.cmake` in the conda env.

## Verifying the RTKLIB baseline build

The C toolchain is separately checked by building the reference console app,
which is what the byte-diff regression will drive:

```sh
cd app/consapp/rnx2rtkp/gcc && make CC=/usr/bin/gcc LDLIBS="-lm"
```
