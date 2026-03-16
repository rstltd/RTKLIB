# BLS (Batch Least Squares) Development Guide

## Overview

BatchSolver.java (~900 LOC) implements a DD-parameterized batch least squares solver for RTK static positioning. This document records the development history, key bugs encountered, and architectural decisions for future reference.

### Current Performance (2026-03-16)

Test data: `test/data/static/` (u-blox L1+L5, ~800m baseline)

| Window | BLS (GPS+GLO, L1+L5) | EKF (all systems) | Notes |
|--------|----------------------|---------------------|-------|
| W0 10min | **FIX 1.7cm** | FIX 1.0cm | |
| W1 10min | **FIX 1.1cm** | FIX 0.6cm | |
| W2 10min | **FIX 0.8cm** | FIX 0.7cm | BLS ties EKF |
| W3 10min | FLOAT 19cm | FIX 0.5cm | Huber trade-off |
| W4 10min | FLOAT 18cm | FIX 0.7cm | |
| 30min | **FIX 3.3cm** | — | |

**Important context**: BLS uses GPS+GLO only; EKF uses all 5 systems. The 2/5 FLOAT results reflect unequal input conditions, not a fundamental BLS limitation. With equivalent multi-system input, BLS AR performance should match or exceed EKF.

---

## Architecture

### What RTKLIB code is shared (do NOT modify)

| Component | Purpose | Used by BLS |
|-----------|---------|-------------|
| `satposs()` | Satellite positions/clocks | Via `preprocessEpochs` |
| `zdres()` | SD residuals (geometry, tropo, antenna, Sagnac) | Direct call in `makeDdObs` |
| `testsnr()` | SNR mask filtering | Inside `zdres` |
| `selsat()` | Common satellite selection | In `preprocessEpochs` |
| `lambda()` | LAMBDA integer search | Direct call |
| `pntpos()` | SPP initial position | Via `sppPosition` |
| `varerr()` | Elevation/SNR-dependent variance | Direct call in `makeDdObs` |

### What BLS does independently (the DD layer)

| Component | Method | Purpose |
|-----------|--------|---------|
| DD obs assembly | `makeDdObs()` | Forms DD from zdres SD outputs, per-system ref sat |
| Segment management | `scanDdAmbiguities()` | Gap detection, GF slip detection, ref sat slip propagation |
| Ref sat selection | `chooseRefSats()` | Per (actual system, freq), longest visibility |
| Normal equation | In `solve()` loop | Scalar H'WH accumulation with Huber weighting |
| Schur complement | In `solve()` | Position/ambiguity separation |
| LAMBDA AR | In `solve()` | Marginal Qaa covariance, AR-eligible filtering |
| Fixed solution | In `solve()` | Conditional covariance update |

### Key boundary: zdres output, not ddres

BLS calls `zdres()` and assembles DD observations itself. It does NOT call `ddres()`. This is the most important architectural decision — it avoids:
- SD/DD parameterization confusion
- ddres's testSys grouping (GPS+GAL in m=0) causing ISB
- Coupling with EKF state management (x[IB()], ssat, etc.)

---

## Bug History and Lessons

### Bug #1: matmul dimension confusion

**Symptom**: BLS solution diverges, position corrections > 1000m.
**Root cause**: RTKLIB stores H as J^T (nxBls × nv, column-major). matmul calls assumed nv × nxBls layout.
**Fix**: Replaced matmul-based accumulation with explicit scalar loops.
**Lesson**: For small-dimension normal equations, explicit loops are clearer and less error-prone than matmul chains.

### Bug #2: SD rank deficiency (most costly bug)

**Symptom**: `matinv(Naa)` returns 0 (success) but Naa^{-1} has negative diagonal (-1.37e+10). Post-fit residuals 3.6 cycles (should be 0.01). Position error 48m.
**Root cause**: DD observations only constrain (N_ref - N_j), not N_ref individually. With SD parameterization, one SD bias per (system × freq) group is unobservable → Naa is rank deficient by (n_systems × n_freqs).
**Why hard to find**: `matinv()` doesn't fail — LU decomposition divides by near-zero pivots, producing huge but finite values. dx looks plausible (wrong direction/magnitude but not NaN/Inf). Symptoms mimic sign errors, model errors, and convergence failures.

**First fix**: Zero-constraint on ref sat (Naa[ref,ref] += 1e12). Worked but fragile.
**Final fix**: DD parameterization rewrite. State vector `[pos(3), DD_N_1, ..., DD_N_n]`. Naa inherently full rank.

**Diagnostic that found it**: `v_postfit = v - H * dx`. Should be ≈ 0 for correct BLS. Was 3.6 cycles → confirmed normal equation bug.

**Lesson**: Always check matrix rank before inverting. Print Naa^{-1} diagonal — negative values on a positive-definite matrix = rank deficiency. `v_postfit` is the single strongest self-check tool.

### Bug #3: Multi-system ISB contamination

**Symptom**: Adding GAL makes position 10x worse (0.2m → 1.8m GPS+GAL vs 0.2m GPS-only).
**Root cause**: ddres's `testSys()` groups GPS+GAL+QZS+SBS into m=0. Cross-system DD (GPS ref - GAL sat) contains ISB. BLS's per-system zero-constraint and ddres's per-group DD creation are contradictory.
**Fix**: DD rewrite with per-actual-system ref sat selection. GPS and GAL never form cross-system DD.
**Lesson**: GPS+GLO is safe (GLO is m=1, separate group). GPS+GAL requires BLS to control DD formation itself.

### Bug #4: LAMBDA covariance underestimation

**Symptom**: LAMBDA ratio stuck at ~1.0 despite good float position (3.3cm).
**Root cause**: Passing `Naa^{-1}` to LAMBDA instead of marginal covariance. `Naa^{-1}` assumes position is perfectly known → underestimates ambiguity uncertainty → LAMBDA search space too narrow.

**Correct formula** (used in code):
```
U = Npa · NaaInv          // 3 × nAmb
Qaa = NaaInv + U^T · Qpp · U  // marginal ambiguity covariance
```
This equals `(N_full)^{-1}[amb,amb]` — the ambiguity sub-block of the full normal equation inverse (derivable via matrix inversion lemma).

**Fix**: Compute marginal Qaa and pass to LAMBDA. Ratio jumped from 1.0 to 1.4-1.6. AR achieved.
**Lesson**: For any BLS → LAMBDA pipeline, always use marginal covariance, not conditional.

### Bug #5: Fixed solution matrix mixup

**Symptom**: Wrong fixed position.
**Root cause**: Formula `dx_fix = -Qpa · Qaa^{-1} · da` used raw Naa sub-block instead of `Qaa^{-1}` (inverse of marginal covariance). Naming confusion between NaaAR, QaAR, QaaInvAR.
**Fix**: Consistent naming. `QaaAR` = marginal covariance sub-block, `QaaInvAR` = its inverse.

### Bug #6: navsys filter missing

**Symptom**: nAmb=32 for 9 GPS sats with nf=1. Non-GPS sats (GLO/GAL/BDS) entered ambiguity list with zero H columns.
**Fix**: `if ((sys & opt.navsys) == 0) continue;` in scanAmbiguities.

### Bug #7: Code variance double-counting

**Symptom**: Minimal (code weight already very low).
**Root cause**: `varerr(f)` returns phase variance with eratio factor built in. Multiplying by eratio² again = double-counting.
**Fix**: Call `varerr(f + nf)` for code variance directly.

### Bug #8: Ref sat cycle slip not propagated

**Symptom**: None in test data (no ref sat slips in 10-min windows).
**Risk**: If ref sat slips, ALL DD ambiguities in the group jump, but BLS treats them as same segment → position pulled off.
**Fix**: Three-phase detection: (1) GF slip for all sats including ref, (2) propagate ref slip to all active DD pairs, (3) segment creation.

### Bug #9: Huber IRLS vs AR trade-off

**Symptom**: W3 ratio drops from 1.5 (no Huber) to 1.3 (Huber k=4.0), falling below adaptive threshold.
**Root cause**: Huber changes the effective observation weighting → marginal Qaa eigenvalues shift → LAMBDA search space shape changes → borderline ratio drops.
**Trade-off**: Huber improves robustness for noisy production data at cost of marginal AR rate on clean data. Correct choice for 200+ station automation.
**Future fix**: Bootstrapping success rate is less sensitive to covariance perturbation than ratio test.

---

## Key Lessons

1. **SD params + DD observations = guaranteed rank deficiency**. Always use DD parameterization for BLS.
2. **Check rank before inverting**. Print N^{-1} diagonal. Negative = rank deficient.
3. **`v_postfit = v - H*dx` is the strongest self-check**. Should be ≈ 0.
4. **Marginal covariance is decisive for AR**. `Naa^{-1}` vs `Qaa` is the difference between ratio 1.0 and 1.5.
5. **Share zdres, bypass ddres**. The clean boundary is at the SD residual level.
6. **Match input conditions before comparing**. GPS-only BLS vs all-system EKF is not a valid comparison.
7. **BLS is more sensitive to preprocessing than EKF**. SNR mask off → 48m error. EKF has dynamic filtering (lock count, arfilter) that BLS must replicate upfront.

---

## Roadmap

```
Current state (2026-03-16)
  GPS+GLO dual-freq, Huber IRLS, FIX 0.8-1.7cm on 10-min windows
  ↓
Multi-system dual-freq (next)
  Add GAL+BDS+QZS via per-system DD (already architected)
  → Equal input conditions for BLS vs EKF comparison
  ↓
Bootstrapping AR validation
  Replace ratio test with success rate from Qaa
  → More stable AR under covariance perturbation (fixes Huber trade-off)
  ↓
Production pipeline integration
  200+ station batch processing, BLS vs EKF cross-validation
  zdres result caching (P1 performance), findDdAmbIdx lookup table
  ↓
Extended capabilities
  Doppler constraint (velocity → position for short windows)
  Post-fix residual validation
  A-F quality grading calibrated for BLS
```
