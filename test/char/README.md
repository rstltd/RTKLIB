# Characterization harness for rnx2rtkp

A golden-master (characterization) test net around the `rnx2rtkp` batch
post-processing binary. Its purpose is to freeze the *current* observable
behaviour of the solver so that later refactoring can be checked for
behaviour-preservation — the binary before a change is the oracle for the
binary after it.

This is deliberately a black-box net at the CLI boundary: the solver core
(`rtkpos`/`pntpos`/`pppos`) currently has zero unit coverage, so the safest
first guard is to pin its end-to-end output before touching any source.

## Layout

| path | role |
|------|------|
| `cases.txt` | corpus of invocations (derived from `app/consapp/CMakeLists.txt` rnx2rtkp cases) |
| `compare.py` | normalizer + token-wise comparator (abs+rel float tolerance) + coverage report |
| `run.sh` | build / capture / verify / mutate driver |
| `golden/` | committed, normalized golden outputs (portable across machines) |
| `work/` | per-run scratch (git-ignored) |
| `rtcm3_antenna_test.c`, `check_rtcm3_fix.sh` | targeted regression test for the RTCM3 1007/1008 overflow fix |

## Usage

```sh
cd test/char
./run.sh build      # deterministic build of rnx2rtkp (LDLIBS=-lm, no LAPACK)
./run.sh capture    # (re)generate golden/ from the current binary
./run.sh verify     # run all cases and diff against golden/  (default)
./run.sh mutate     # self-test: confirm the comparator catches a 5 cm shift
```

`verify` exits non-zero if any case diverges. Tolerance defaults to exact
(`ABSTOL=0 RELTOL=0`), correct for a same-binary self-oracle; a refactor that
legitimately reorders floating-point reductions can be checked with e.g.
`ABSTOL=1e-6 RELTOL=1e-9 ./run.sh verify`.

## Normalization

Only two `.pos` header lines are volatile and get masked before comparison:
`% program : …ver…` (build version) and `% inp file : /abs/path` (reduced to
basename). Everything else — `obs start/end`, column header, every solution row
— is compared verbatim / token-wise.

## Determinism

`run.sh build` links no LAPACK/MKL (internal LU inversion). The audit found that
LAPACK/MKL dispatch is the *only* cross-build numeric divergence source; with it
out, and with no RNG and no threading in the batch path, the same binary on the
same input is bit-stable. `verify` re-running green after a fresh `build`
confirms this.

## Coverage gaps (fail-loud)

`cases.txt` exercises **SINGLE, DGPS, KINEMA, STATIC, STATIC_START** only. The
bundled `test/data` cannot drive **MOVEB, FIXED, PPP_KINEMA, PPP_STATIC,
PPP_FIXED**: no upstream case uses those modes here, and there is no obs file
paired with the precise products (sp3/clk are 2009/2010; the RINEX obs are
2005). The coverage table printed by every run states this explicitly. Closing
the PPP gap requires adding a matched obs + precise-product dataset — do that
before relying on this net to guard any PPP-path refactor.

## RTCM3 overflow regression test

`check_rtcm3_fix.sh` proves the `src/rtcm3.c` type 1007/1008 antenna-descriptor
overflow fix: it builds `rtcm3_antenna_test.c` against the current source (must
run clean) and against a clamp-stripped copy (must be caught). The overflow is
intra-struct (`antdes`→`antsno`→`rectype` inside one `sta_t`), invisible to
AddressSanitizer, so the test uses a sentinel byte in `sta.rectype` — a field a
1008 decode must never write — as the oracle.
