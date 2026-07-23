# PPP-AR in RTKLIB-EX: implementation and empirical findings (WTZR, GFZ MGX, 2023-152)

Status: **negative result, with a correct implementation.** The revived `ppp_ar()` (IFLC
wide-lane/narrow-lane cascade) is algebraically correct and does fix ambiguities, but under the
products actually available here (GFZ MGX rapid: orbits + clocks + OSB, **no atmosphere**) it
delivers **no measurable value** — neither better 24 h static accuracy nor faster convergence.
The missing ingredient is the PPP-RTK atmosphere layer, exactly as the earlier PPP modernity
audit predicted.

## 1. What was implemented (`src/ppp_ar.c`)

The upstream stub (`return 0`) was replaced by a wide-lane/narrow-lane two-step on the
**ionosphere-free (`IONOOPT_IFLC`)** float PPP:

- IFLC has ONE clean ambiguity state per satellite, `IB(s,0) = Lc - Pc = lam_N*N1 +
  lam_N*(f2/(f1-f2))*N_WL` (metres). No ionosphere states, so no rank deficiency.
- **Step 1 — wide lane:** round the time-averaged, OSB-corrected Melbourne-Wubbena
  single-difference ambiguity (geometry-and-ionosphere-free, `lam_W ~= 0.86 m`, safe).
- **Step 2 — narrow lane:** `N1_SD = B_IF_SD/lam_N - (f2/(f1-f2))*N_WL` is integer; fix jointly by
  LAMBDA + ratio test. `B_IF_SD` is the single-difference of the metre IF state, so the design
  matrix `D` carries `+1/lam_N` on the satellite IF state and `-1/lam_N` on the reference.
- **Step 3 — remove-restore:** condition the float state on `a == N1_fixed` in place
  (`x -= P*D'*(D*P*D')^-1*(a - N1)`), one IF state per satellite.

Between-satellite single differencing against a (held) reference cancels the receiver IF phase
bias, which RTKLIB does not estimate. Scope: GPS, `nf>=2`, `IONOOPT_IFLC`. The routine self-gates
(returns 0 unless `mode>=PMODE_PPP_KINEMA && modear!=OFF && ionoopt==IFLC`), so an AR-off run is
byte-identical to float.

A prior attempt on the `IONOOPT_EST` (uncombined) states failed structurally: those states absorb
the ionosphere (rank deficiency), so the IF combination `c1*IB0 + c2*IB1` carried a systematic
datum bias and LAMBDA fixed a *shifted* integer grid — high ratio, wrong position. See
`est_ar_research.md`. Switching to IFLC removed that contamination.

## 2. Acceptance criterion (the load-bearing lesson)

**RTKLIB's Q code is not a valid PPP-AR success criterion.** `pppos()` sets `SOLQ_PPP` (Q=6) for
float and promotes to `SOLQ_FIX` (Q=1) when `ppp_ar()` returns non-zero and a position-std gate
(`MAX_STD_FIX = 0.15 m`) passes. But the remove-restore *shrinks* the covariance, so wrong fixes
sail through the gate: reaching Q=1 proves nothing. The narrow-lane ratio test is also blind to a
whole-grid datum shift (it only ranks competing integer candidates).

The only valid criterion is **external quality against ground truth**: does the fixed solution sit
closer to the true ITRF position, and scatter less, than the float?

Ground truth used: **ITRF2020-IGS-TRF.SSC**, WTZR segment 3 (epoch 2015.0), propagated with the
station velocity to the data epoch 2023.4137:

```
WTZR ITRF2020 @ 2023.42  =  (4075580.256, 931854.113, 4801568.314) m
```

## 3. The PPP configuration had to be fixed first (or the comparison lies)

An early comparison suggested the AR was a "6x breakthrough" (fixed 3.1 mm precision vs float
19 mm). **That was an artifact of a mis-configured float.** The conf was missing
`ant1-anttype=*`, so `setpcv()` never pulled the receiver antenna from the RINEX header and the
LEIAR25.R3 choke-ring receiver PCV (~150 mm vertical) was never applied. The float carried a
~190 mm vertical error and ~19 mm noise; the fixed only looked good against that noisy baseline.

The **correct** configuration (float now agrees with ITRF2020 to 12 mm):

```
pos1-ionoopt   = dual-freq        # IONOOPT_IFLC
pos1-tropopt   = est-ztd
pos1-posopt1   = on               # satellite antenna PCV
pos1-posopt2   = on               # receiver antenna PCV
pos1-posopt3   = on               # phase windup
pos1-tidecorr  = 1                # solid-earth tide (this option is a BITMASK: 1 solid, 2 otl, 4 spole)
ant1-anttype   = *                # REQUIRED: take receiver antenna + delta from the RINEX header
file-satantfile / file-rcvantfile = igs20.atx   # igs20 for 2023 data, not igs14
```

## 4. Result under the correct configuration

Static WTZR, full day, GPS-only, GFZ MGX rapid products, vs the ITRF2020 truth above. Converged
window (12:00-24:00) for a fair comparison:

| solution     | n    | 3D err vs truth | precision (scatter) |
|--------------|------|-----------------|---------------------|
| float (Q=6)  | 1346 | **12.2 mm**     | **2.6 mm**          |
| fixed (Q=1)  | 94   | 14.3 mm         | 3.8 mm              |

**The well-configured float already wins.** The AR fixes are comparable at best, slightly worse in
3D, and no more precise. The value evaporated the moment the baseline was correct.

## 5. Convergence test (the scenario where AR *should* help)

For a 24 h static solution the float fully converges, so AR cannot improve the final accuracy — its
real value is faster convergence and short/kinematic sessions. Testing convergence directly:

- float-only and AR-on convergence curves are **identical for the first 75 minutes**; the first
  fix does not occur until **75 min**, by which point the float has already converged to ~30-50 mm.
- The bottleneck is **not** wide-lane averaging: lowering `MW_NMIN` 40 -> 16 leaves the first fix
  at 75 min. It is the **narrow lane**: lowering the ratio threshold 2.5 -> 1.5 still yields zero
  fixes before 75 min, because the float IF ambiguity is not precise enough for LAMBDA to separate
  integer candidates (ratio < 1.5) until it has converged. `lam_N = 0.107 m` needs the float IF
  ambiguity determined to << 5 cm, which takes ~75 min here.

So the narrow lane must wait for a converged float — and once the float has converged, there is
nothing left for AR to accelerate. AR provides **no convergence benefit** with these products.

## 6. Root cause and conclusion

Two independent reasons the AR adds no value here:

1. **No atmosphere corrections.** GFZ MGX rapid provides orbits, clocks, and biases but no
   ionosphere/troposphere SSR. Fast (seconds-to-minutes) PPP-AR convergence relies on external
   atmosphere corrections (CLAS / SPARTN / a regional network) that collapse the float almost
   immediately so the ambiguities can be fixed at once. Without them the float converges on its
   own ~75 min timescale and AR can only ride behind it. This is precisely the gap the PPP
   modernity audit flagged ("no PPP-RTK atmosphere -> convergence stuck at 20-30 min").
2. **Marginal narrow-lane integer recovery.** Even at convergence the NL fractional RMS is ~0.28
   (near-uniform), so many candidate fixes are borderline; the standard clock + OSB combination
   does not make the NL cleanly integer for this receiver's observables.

**Conclusion.** The IFLC WL/NL implementation is correct and complete, but PPP-AR delivers no
practical benefit under standard MGX rapid products. To make it worthwhile requires either the
PPP-RTK atmosphere layer (a substantial new subsystem: SSR atmosphere decode + regional
corrections) or integer-recovery-/decoupled-clock products that make the narrow lane cleanly
fixable. Neither is available in-tree today.

## 7. Reproducibility notes

- Ground truth: `ITRF2020-IGS-TRF.SSC` from `itrf.ign.fr/ftp/pub/itrf/itrf2020/`; propagate the
  station's position with its velocity from epoch 2015.0 to the data epoch. IGS station metadata
  JSON coordinates are reference-epoch (unpropagated) and rounded — do not use them as cm truth.
- Antenna: use `igs20.atx` (`files.igs.org/pub/station/general/igs20.atx`) for 2023 data, and set
  `ant1-anttype=*` or the receiver PCV silently will not be applied.
- `pos1-tidecorr` is an integer bitmask, not a keyword; `=solid` is a no-op.
- Do not judge PPP-AR by the Q code; judge by external agreement with an ITRF coordinate at the
  correct epoch.
