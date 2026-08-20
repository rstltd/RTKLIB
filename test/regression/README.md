# Byte-Diff Regression (gate G1)

Enforces design invariant **I1** of [`plan.md`](../../plan.md): the EKF path
must stay bit-for-bit identical for the whole duration of the FGO project.

Every PR that touches `src/` has to pass this. It matters most for **PR-2**,
the `ddres()` purification (plan.md §6.1 M3, risk RC-1) — a ~300-line rewrite
of core EKF code whose entire claim is that it changes nothing. Without a
byte-exact gate that claim is unverifiable.

## Usage

```sh
test/regression/run_regression.sh            # verify every dataset
test/regression/run_regression.sh --list     # list known datasets
test/regression/run_regression.sh <name>     # verify one dataset
test/regression/run_regression.sh --update   # regenerate baselines
```

Exit status is non-zero if any dataset differs from its baseline.

`--update` rewrites the committed baselines. Only run it when the change to
the numbers is **intended and understood**, and say so in the commit message —
a baseline refresh silently smuggled into an unrelated PR defeats the gate.

## Sensitivity

The gate compares two files per dataset:

| File | Compared | Why |
|---|---|---|
| `solution.stat` | byte for byte, unmodified | Per-satellite residuals, ambiguity states and variances. This is the sensitive one. |
| `solution.pos` | after header normalisation | The final position solution. Coarser — a change can move `.stat` without moving `.pos`. |

Measured on the sample dataset: perturbing the return value of `varerr()`
(`src/rtkpos.c`) by **one part in 10¹²** leaves `solution.pos` completely
unchanged but shifts `solution.stat`, and the gate fails. That is the level of
sensitivity a refactor of `ddres()` needs to be held to.

`.pos` needs normalisation because its header records the absolute paths of the
inputs and the program version, neither of which is a property of the numbers.
Paths are reduced to basenames and the version is masked — plan.md §6.6 M19
bumps `PATCH_LEVEL` in the same PR that adds the FGO option fields, and that
must not invalidate every baseline for a reason unrelated to the solution. The
version actually used is recorded in `baseline/metadata.txt` instead.

The header line is `% program   : RTKLIB ver.EX 2.5.1` — the *library* name,
not the tool name. Masking is verified by bumping `PATCH_LEVEL` and confirming
the gate still passes.

`.stat` has no header at all and is therefore compared untouched.

> Note: plan.md §6.11 G1 sketches this gate using `-x 2` and comparing
> `out_new2.stat`. `-x` sets the *trace* level and produces a `.trace` file;
> the solution-status file this gate depends on comes from **`-y`**, and is
> named `<outfile>.stat`. The plan has been corrected.

## Reproducibility

Floating-point results depend on the compiler and its flags, so the runner
pins them (`FGO_CC`, default `/usr/bin/gcc`; `FGO_LDLIBS`, default `-lm`) and
records what it used in `baseline/metadata.txt`. It also unsets
`CFLAGS`/`CXXFLAGS`/`CPPFLAGS`/`LDFLAGS`, which an activated conda shell
exports and `make` would otherwise fold into the build — see
[`docs/fgo/build_environment.md`](../../docs/fgo/build_environment.md).

The binary is always rebuilt with `make -B`. The makefile leaves its objects in
the source tree, so an incremental build could quietly link objects produced by
a different compiler than the run claims to have used. Set `FGO_RNX2RTKP` to
skip the build and use an existing binary.

A baseline is only comparable against a build made the same way. When the
compiler changes, regenerate rather than reinterpret.

## Datasets

```
datasets/<name>/
├── dataset.conf        inputs and arguments
├── opt.conf            rnx2rtkp -k options; part of what the baseline pins
└── baseline/
    ├── solution.pos    normalised
    ├── solution.stat   verbatim
    └── metadata.txt    commit, compiler and versions that produced it
```

`dataset.conf` is `key = value`. Paths may use `$RTKLIB_ROOT` and
`$FGO_DATA_ROOT`; nothing else is expanded, so a dataset file cannot execute
shell code.

### The in-repo dataset is not enough

`sample-2005-static` is the only dataset small enough to live in git: 3.3 km
baseline, GPS L1/L2, one hour at 30 s, 112 of 115 epochs fixed. It is a fast
and sharp guard against accidental numerical change, and nothing more.

plan.md §11.1 **P1.1** asks for at least five sets of ≥ 24 h from real
monitoring stations — open sky, obstructed sky, vegetated slope, urban
multipath, long baseline — with independent ground truth where possible. Those
are what the Phase 2 benefit measurements and the Phase 1 baseline report need;
this sample cannot stand in for them.

To add one, create a sibling directory with the same layout and point
`dataset.conf` at files under `$FGO_DATA_ROOT`:

```
rover = ${FGO_DATA_ROOT}/bridge-site-a/rover.obs
base  = ${FGO_DATA_ROOT}/bridge-site-a/base.obs
nav   = ${FGO_DATA_ROOT}/bridge-site-a/brdc.nav
```

Only the config and the baselines are committed; the observation data stays
outside the repository. Datasets whose files are absent are reported as `SKIP`
rather than failing, so a checkout without `$FGO_DATA_ROOT` still runs the
in-repo dataset. A dataset named explicitly on the command line is different:
if it does not exist the run fails, so a typo in CI cannot pass by testing
nothing.

## Baselines are stored verbatim

`test/regression/.gitattributes` marks the baselines `-text`. The repository
root sets `* text=auto`, which rewrites the CRLF line endings `.pos` uses; a
normalised baseline can never match the program's real output. See that file
for the byte counts observed before the rule existed.
