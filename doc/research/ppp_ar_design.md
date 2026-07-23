# Reviving `ppp_ar()` in RTKLIB-EX — Implementable Design

Branch inspected: `fix/qzss-navic-ppp-bugs`. All file:line citations verified against source this
session. Where the five studies disagreed, I re-ran the grep myself and record the resolution.

---

## 0. LOAD-BEARING CORRECTION TO THE TASK PREMISE (read first)

The task states, as landed-and-verified prior work: *"a file phase-OSB loader (`corr_phase_bias_file`)
now pre-corrects obs.L[] … the integer-recoverable bias substrate the theory map calls L1 is in
place."*

**This is false in this checkout.** Verified directly:

- `grep -rn corr_phase_bias_file src/` → **zero hits**. The function does not exist.
- `readbiaf()` still hard-skips phase records: `src/preceph.c:474` — `if (obs1[0]!='C') continue;
  /* skip phase biases for now */`. Only code ("C…") OSBs reach `nav->cbias[]`.
- The only phase-bias→`obs.L[]` path is `corr_phase_bias_ssr()` (postpos.c:418, called 465) /
  `corr_phase_bias()` (rtksvr.c:515, called 705). Both read **only** `nav->ssr[sat-1].pbias[]`,
  which is written **only** by the RTCM3-SSR decoder (rtcm3.c:2017). In RINEX + SP3/CLK +
  Bias-SINEX post-processing, `pbias[]` is all-zero, so the correction is a **silent no-op**.
- The repo's **own** theory map already says this: `doc/research/ppp_theory_map.md:191-204` (§3.A.3)
  — *"the file-based loader `readbiaf()` skips phase … Only the real-time RTCM-SSR stream ever
  populates pbias."*

Two of the five studies (`ppp-state-model`, `ar-forks-qc`) caught this; two (`inert-plumbing`,
`lambda-api`, and the task header) accepted the premise. My grep confirms the skeptics. This matches
the project MEMORY note that the Debian checkout tracks a clean upstream mirror and lacks Windows-side
customizations — the loader may exist on a Windows checkout, but **it is not here, on any branch this
box can see.**

**Consequence:** the phase-OSB substrate is **not** in place. Building `ppp_ar()` on top of it is
building the roof before the wall. The phase-OSB file loader becomes **Phase 0** of this plan — an
explicit prerequisite, not an assumption. (Alternative that avoids Phase 0 entirely: drive the
characterization fixture from a **recorded RTCM3-SSR phase-bias stream** instead of a Bias-SINEX file,
which populates `pbias[]` through the existing, working path. See §5.)

---

## 1. FORK CHOICE — **Fork B (IONOOPT_EST + LAMBDA on per-frequency IB(s,f))**

### Decision: Fork B first. Fork A deferred.

**Why B, from the code:**

1. **Zero new persistent state for the fixing step.** Fork B fixes the per-frequency ambiguity
   states `IB(s,f)` that the EKF already carries (`ppp.c:104-118`, `IB(s,f,opt)=NR+MAXSAT*f+s-1`).
   No new `ssat_t`/`rtk_t` field is needed to *fix*; `ppp_ar()` slices `xp`/`Pp` with the existing
   `IB()` macro.
2. **Fork A needs net-new machinery that does not exist.** Fork A (IFLC) must externally reconstruct
   WL (Melbourne-Wübbena) then NL, because the single IFLC ambiguity `IB(s,0)=Lc-Pc`
   (`ppp.c:758-761`) is a real-valued combination `κ1·N1·λ1+κ2·N2·λ2` (κ1≈2.546, κ2≈-1.546) — **not
   an integer of any wavelength**, so LAMBDA on the raw IFLC state is meaningless (confirmed
   `ppp.c:929-932`, IFLC has no iono state). WL reconstruction needs a **persistent per-satellite
   MW/WL accumulator** (running mean + count + slip-reset). Neither `ssat_t.mw[NFREQ-1]` (a
   single-epoch snapshot overwritten every epoch purely for slip detection, `ppp.c:496-518`) nor
   `ambc_t.LC/LCv/n/epoch` (shaped for exactly this but **dead** — zero reads/writes anywhere in
   `src/*.c`, `rtklib_types.h:629-636`) is usable as-is. Fork A is greenfield.
3. **Architectural grain already matches B.** Every existing OSB-consumption path is per-signal, not
   IF-combined: `corr_meas()` applies code OSB per `nf` loop iteration; `corr_phase_bias_ssr()` loops
   `j=0..NFREQ-1` subtracting `pbias[code-1]` per signal (`postpos.c:418-430`). There is **no** code
   path that forms an IF-combined bias or a decoupled phase-clock (`dts_phase`) — the products Fork A
   would need. RTKLIB follows the OSB/observation-correction convention (§3.A.3), which is exactly
   Fork B's input assumption.
4. **The float model under EST is genuinely iono-clean.** In `ppp_res` under IONOOPT_EST the
   ionosphere lives in a **separate** state `x[II(sat)]` (`ppp.c:923-928`, `1031-1037`) and the phase
   residual is `y-(r+cdtr-c·dts+dtrp+C·dion+dcb+bias)` (`ppp.c:1042-1047`). The converged `IB(s,f)`
   is therefore `N_f·λ_f + residual-hardware-bias`, the same shape RTK's float ambiguity has and
   `resamb_LAMBDA` fixes.

**What Fork A would need if resurrected later** (deferred, tracked):
- New `ssat_t` fields: `wl_mean[NFREQ-1]`, `wl_var`, `wl_n`, `wl_epoch` (or repurpose the dead
  `ambc_t.LC/LCv/n/epoch` — but that struct is untouched and its semantics unproven, so prefer new
  named fields).
- WL fix by rounding (λ_W≈0.86 m, safe), then NL fix, using `N_IF=N_N+(f2/(f1-f2))·N_W`
  (theory map §3.A.2.1).
- A **decoupled-clock or IF-OSB product ingest** if the decoupled-clock family is chosen — new
  `nav_t`/`ssr_t` schema, not just a loader tweak.

---

## 2. MINIMAL FIRST SCOPE — the smallest `ppp_ar()` that yields a real `SOLQ_FIX`

**IN the first cut:**
- **GPS only.** (Single well-behaved constellation; avoids GLONASS FDMA inter-channel bias and
  BDS/QZSS/NavIC OSB-table gaps.)
- **IONOOPT_EST, nf=2 (L1/L2).**
- **Static** session (or short kinematic), converged float solution.
- **Between-satellite single-difference (SD)** of `IB(s,f)` against one reference satellite per
  frequency — see §3 for *why this is mandatory, not optional*.
- Stack L1-SD and L2-SD ambiguities into one `lambda()` call, `m=2`.
- **Ratio test only**, fixed threshold `s[1]/s[0] ≥ 3.0`. (FF-RT deferred.)
- `ppp_ar()` self-gates on `opt->modear != ARMODE_OFF` (see §4 gating note).

**OUT of the first cut (explicitly):**
- Fork A / IFLC / WL-MW accumulator.
- FF-RT critical-value QC (ratio test is the placeholder).
- Partial AR / round-robin exclusion / `arfilter` (`manage_amb_LAMBDA` heuristics, rtkpos.c:1832+).
- Multi-GNSS mixing; L5 / nf=3 (the shared `ND(opt)` L5-DCB state, `ppp.c:117`, entangles the
  3rd-frequency ambiguity — scope to nf=2 to avoid it).
- Decoupled-clock product ingest.
- Any change to the downstream hold mechanism (`test_hold_amb`, the hard-`matcpy` feedback) — it is
  inherited as-is and only fires when `modear==ARMODE_FIXHOLD`.
- Fixing `test_hold_amb`'s hardcoded `fix[0]/fix[1]`-only inspection (it structurally cannot see a
  3rd frequency — irrelevant at nf=2, flagged for later).

---

## 3. STEP SEQUENCE — the algorithm inside `ppp_ar(rtk,obs,n,exc,nav,azel,x,P)`

`x` and `P` are `xp`/`Pp` — the converged float state/covariance, full length `rtk->nx`
(`rtk->na==rtk->nx` for PPP, `rtkpos.c:2255-2256`, so there is **no** reduced-`na` target; we fix and
back-substitute in place in the same arrays).

RTK's `resamb_LAMBDA` (`rtkpos.c:1706-1832`) is the template. Reusable-verbatim vs PPP-specific is
called out per step.

### Step 0 — gate (PPP-specific)
`if (opt->modear==ARMODE_OFF) return 0;` — because `pppos()` calls `ppp_ar()` **unconditionally**
whenever the float converges (no `modear` check at the call site, unlike RTK at rtkpos.c:1846). The
stub returning 0 is the only thing keeping AR-off behavior today. `ppp_ar` **must** self-gate.

### Step 1 — select candidate satellites & pick a reference per (system,freq) [PPP-specific]
Mirror `ddidx()`'s **reference-pick mechanics** (`rtkpos.c:1531-1544`) but with **one differencing
axis** (between-satellite), not two (RTK also differences between receivers). For GPS, for each
frequency f∈{0,1}:
- Eligible sat = GPS, `x[IB(s,f)]!=0`, `ssat[s-1].vsat[f]`, not excluded (`exc`), `azel[1] >=
  opt->elmaskar` (use `rtk->ssat[s-1].azel[1]`, populated for all modes by the `pntpos()` pre-pass
  `rtkpos.c:2376-2377` — **not** the obs-indexed `azel` parameter).
- **Reference** = highest-elevation eligible sat (RTK uses "first eligible"; highest-elevation is a
  cheap improvement worth taking for PPP's longer arcs). Hold it fixed across epochs if still
  eligible, to keep the SD datum continuous.

### Step 2 — WHY single-difference: remove the receiver phase bias [the crux, PPP-specific]
After satellite-OSB correction, the undifferenced float ambiguity is
`IB(s,f) ≈ N_f(s)·λ_f + β_f(receiver) + ε`, where `β_f` is a **receiver-side per-frequency phase
bias common to all satellites** that RTKLIB has **no estimated state for** (there is a receiver code
DCB state `ID(opt)`, `ppp.c:117`, but no receiver *phase*-bias state). `β_f` is generally
non-integer ⇒ the raw `IB(s,f)` is **not** an integer of `λ_f`. Between-satellite SD cancels `β_f`:
`SD(s,ref,f) = IB(s,f) - IB(ref,f) = (N_f(s)-N_f(ref))·λ_f + ε` → **integer×λ**. This is the single
step that makes est-mode ambiguities integer-recoverable, and it is why "OSB applied" alone is *not*
sufficient (see Risk R2).

### Step 3 — assemble float ambiguity vector `a` (cycles) and covariance `Q` [algebra reused, indices PPP-specific]
For each non-reference eligible sat s and freq f:
- `a[k] = (x[IB(s,f)] - x[IB(ref,f)]) / λ_f`  (convert meters→cycles; `λ_f=CLIGHT/sat2freq(...)`).
- Build `Q` (n×n, column-major) as the SD covariance: for SD pairs k,l,
  `Q[k,l] = P[is,is]-P[is,ir]-P[ir,is]+P[ir,ir]` scaled by `1/(λ·λ)`, where `is=IB(s,f)`,
  `ir=IB(ref,f)`. This is `D·Pp·Dᵀ` with the between-satellite D-matrix — same subtraction pattern
  as RTK's `y[i]=x[ix[2i]]-x[ix[2i+1]]` and `Qb` build (`rtkpos.c:1728-1744`), one axis instead of
  two. **Must include** the `II(s)–IB(s,f)` cross-correlation baked into `Pp` (Risk R3): don't
  assume iono-decorrelation — just slice `Pp` honestly.
- Also build cross-covariance `Qab` (na×nb) between the **full state block** and the SD ambiguity
  block, for back-substitution (`rtkpos.c:1742-1744` analog).
- **Prune** any SD state whose `Q` diagonal is ~0 / not-yet-observed before calling `lambda()` —
  `LD()` hard-fails on a non-positive pivot (`lambda.c:35`, returns -1). RTK does this via
  `ddidx`/`minfixsats`; replicate the intent.

### Step 4 — integer search [REUSED VERBATIM]
`info = lambda(nb, 2, a, Q, F, s);` (`lambda.c:180`, declared `rtklib_api.h:465`). `lambda()` is
fully standalone/reentrant, no RTK/PPP coupling, returns `F` sorted ascending by `s`
(`F[:,0]`/`s[0]` = best). **No change to lambda.c.**

### Step 5 — validation [ratio now, FF-RT later; QC written fresh in ppp_ar.c]
`if (info || s[0]<=0.0 || s[1]/s[0] < AR_THRES_PPP) { return 0; }` with `AR_THRES_PPP=3.0` for the
first cut. **Do not** reuse `ar_poly_coeffs` (rtkpos.c:96-99): it is a curve-fit to a TU-Delft
LAMBDA-toolbox example (comment rtkpos.c:94-95), has **no** failure-rate grounding, and is `static`
to rtkpos.c. FF-RT is a clean later upgrade — swap the ratio comparison for a critical value
`μ=f(P_f,n)` from a lookup table; the surrounding machinery is unchanged. Record `rtk->sol.ratio =
s[1]/s[0]` (field exists, unused by ppp.c today — free diagnostic).

### Step 6 — back-substitution (remove-restore) [ALGEBRA REUSED, target PPP-specific]
Apply Teunissen `x̌ = x̂ - Qab·Qb⁻¹·(â-ǎ)` and `P̌ = P̂ - Qab·Qb⁻¹·Qabᵀ` — the exact formula at
`rtkpos.c:1796-1804` (`matinv(Qb,nb)` then two `matmul`/`matmulm` chains). **Difference from RTK:**
RTK writes into separate `rtk->xa`/`rtk->Pa` of reduced size `na<nx` and then `restamb()` rebuilds SD
biases; **PPP has no reduced target** — write the corrected values **directly back into the same
`x`(=xp) and `P`(=Pp)** slots. There is no `restamb()` step to replicate. `â-ǎ` = SD-float minus
SD-fixed (in meters, `·λ_f`). This shrinks the position-block covariance `Pp[0..2]` — which is what
must happen for the caller's `norm(sqrt(diag(Pp[0:3]))) < MAX_STD_FIX(0.15)` gate (`ppp.c:1262`) to
promote `SOLQ_FIX`.

### Step 7 — set fix flags [PPP-specific, MANDATORY]
For every SD-fixed sat s at freq f: `rtk->ssat[s-1].fix[f]=2;` (also the reference sat, since it
participates). **Nothing else in the PPP path ever writes `fix[]=2`** (verified grep: ppp.c only
resets to 0 at 1200, demotes 2→1 at 1160, reads at 1173/1175). Do not leave `fix[]=2` on any sat not
in the accepted set (pppos zeroes all `fix[]` at epoch top, so per-epoch scope is automatic).

### Step 8 — return
`return nb;` (nonzero = validated fix). Do **not** touch `rtk->x/P/xa/Pa` — pppos owns those copies
(`ppp.c:1258-1259` into xa/Pa; `1271-1272` into x/P only on hold).

**Interaction with `test_hold_amb`:** none required from `ppp_ar` beyond correct `fix[]=2`. It is
already gated on `modear==ARMODE_FIXHOLD` and `minfix` (`ppp.c:1168,1184`); it sets `ambc[].flags`
on the fly (`ppp.c:1176`); `udbias_ppp` clears those flags on slip (`ppp.c:799`). `ppp_ar` runs
before it (call at 1256, hold at 1270), so flags are correct as long as `fix[]=2` is set first.

---

## 4. THE CONTRACT — what must mutate/return so the inert plumbing lights up

`ppp_ar(rtk, obs, n, exc, nav, azel, x=xp, P=Pp)` must, on a validated fix:

| # | Requirement | Enforced by | Source |
|---|---|---|---|
| C1 | Mutate `x`(xp) in place to the fixed state | ppp_res(9) revalidation reads `bias=x[IB]` from mutated xp | ppp.c:1043,1257 |
| C2 | Mutate `P`(Pp) so position-block std shrinks < 0.15 m | `norm(sqrt(diag(Pp[0:3])))<MAX_STD_FIX` promotes SOLQ_FIX **on mutated Pp, independent of return value** | ppp.c:1262-1263 |
| C3 | Return nonzero iff validated | `&&`-chain gate; 0 → `else{nfix=0}`, stays SOLQ_PPP silently | ppp.c:1256,1264-1265 |
| C4 | Set `ssat[s-1].fix[f]=2` for every fixed (s,f) | only writer in PPP path; consumed by update_stat demote + test_hold_amb | ppp.c:1160,1173 |
| C5 | Never write rtk->x/P/xa/Pa directly | caller does those matcpy's on `&&` success | ppp.c:1258-1259,1271-1272 |
| C6 | Survive ppp_res(9) post-fit re-check (THRES_REJECT=4σ, one-shot worst-outlier reject, no retry) | 2nd `&&` term; one bad residual discards the whole epoch's fix | ppp.c:1257, ppp_res post>0 branch |
| C7 | Self-gate on modear (no external gate) | pppos calls ppp_ar unconditionally on float convergence | ppp.c:1256 |

**Gating decision (design call, not answered by source):** `modear==4:ppp-ar` in the struct comment
(`rtklib_types.h:464`) is **vestigial** — not in `options.c` ARMOPT string, `grep modear==4` = zero
hits. `ssat.fix==4:ppp` (rtkpos.c docstring) is likewise never assigned. For the first cut, gate on
`opt->modear != ARMODE_OFF` (reuse the existing CONT/INST/FIXHOLD enum). The clean long-term fix is
a dedicated `pos2-pppar` boolean option in `sysopts` + an `ARMODE_PPPAR`/separate flag — recommended
but **out of first-cut scope**; do not assume value 4 is live.

**Inherited risk to flag (not fixed here):** the hold feedback (`ppp.c:1271-1276`) is a raw `matcpy`
overwrite of the live `rtk->x/P` — no noise-damped Kalman blend like RTK's `holdamb()`
(`rtkpos.c:1649-1655`). A wrong fix injects its full shrunk covariance into the running filter with
zero cushioning. This is a strong argument for **conservative QC** (favor FF-RT / a high ratio
threshold early) before `ppp_ar` ever returns success.

---

## 5. CHARACTERIZATION + TDD — fencing before/while building

### Phase 0 (prerequisite): build the phase-OSB file loader — OR sidestep it
Because §0: the substrate is missing. Two options.

**Option 0a (build the loader — needed for a Bias-SINEX product workflow):**
1. Add a phase-OSB table to `nav_t` parallel to `cbias[]` (e.g. `pbias_osb[MAXSAT][MAXCODE]`).
2. Extend `readbiaf()` (`preceph.c:459-500`) to route `obs1[0]=='L'` records into it (remove/branch
   the `preceph.c:474` skip).
3. Add `corr_phase_bias_file()` running at the **same point** `corr_phase_bias_ssr()` runs
   (`postpos.c:465`, on `obs_ptr` before `rtkpos()`), applying `L[j] -= B_O·freq/CLIGHT`. Preserve
   the invariant that `udbias_ppp`/`ppp_res`/`corr_meas` see `obs->L[]` **only after** pre-correction
   (they never bias-correct L themselves — verified `ppp.c:409-453`).
4. **Characterization tell (assert this!):** phase-OSB correction moves the **$SAT ambiguity values
   but NOT the float position.** Integer-recoverable bias is absorbed into the float ambiguity; until
   *fixing* happens, the float solution is invariant. So Phase-0-done looks like "ambiguities
   shifted, position identical." This cleanly separates "loader works" from "fixing works." ⚠️ Do
   **not** read the ambiguities via the $SAT status line — `rtkoutstat()` uses rtkpos.c's own
   `IB()/NR()` macros for PPP mode, which omit `NC(opt)=NSYS(=6)+ND` states (`rtkpos.c:317` vs
   `ppp.c:104-118`), so $SAT bias columns read the wrong `rtk->x` slot for PPP. Read `rtk->x[IB()]`
   directly via `trace()` or a test hook.

**Option 0b (avoid Phase 0): drive the fixture from a recorded RTCM3-SSR phase-bias stream.** This
populates `nav->ssr[].pbias[]` through the existing, working `corr_phase_bias_ssr()` path with **no
new loader code**. Faster to a first green fix; the file loader can follow as a separate deliverable.
**Recommended for the very first end-to-end fix** to decouple the AR engine from loader work.

### Phase 1: unit tests (red→green), no fixture needed
- `t_lambda.c` already exercises `lambda()` — keep as regression.
- **New `t_ppp_ar.c`:** synthetic `(a,Q)` with a known integer truth; assert the §3 SD-assembly +
  §6 remove-restore reproduce the fixed vector and shrink `P`. Include a **tiny-n case (n=1,2)** to
  probe the unverified `search()` s[] fill risk (Risk R4).
- **Guard test:** with `modear==ARMODE_OFF`, `ppp_ar` returns 0 and `xp/Pp` are untouched (proves
  C7 + AR-off byte-identity to today's stub).

### Phase 2: golden-master characterization (`test/char/`)
- Existing PPP cases 28-30 run **IONOOPT_BRDC** (`prcopt_default.ionoopt=1`), i.e. neither fork's
  iono branch — **zero regression cover** for the code being changed. Capture golden **now** so the
  stub-behavior baseline is frozen: with AR disabled, output must stay byte-identical after the
  `ppp_ar` rebuild lands.
- Add a **new EST-mode case** (ionoopt via `.conf`, since `rnx2rtkp.c` has no `-y`/ionoopt CLI flag)
  using the fixture below.

### Phase 3: integration on the WTZR 2023-152 + CODE OSB fixture
⚠️ The fixture is **not** in `test/data`/`test/char` today (grep/ls this session) — must be sourced
externally (RINEX obs for WTZR DOY 152/2023 + CODE MGEX SP3+CLK + CODE phase-OSB, as Bias-SINEX for
0a or RTCM3-SSR replay for 0b). Assertions, once a fix is reachable:
- **Reachable `SOLQ_FIX`:** at least one epoch reports `sol.stat==SOLQ_FIX` (the whole point — proves
  the dead branch at `ppp.c:1258-1263` is alive).
- **Position moves toward truth on fix:** fixed position is **closer to WTZR's published ITRF
  coordinate than the float** solution. This is the payoff assertion — phase-OSB moved the
  ambiguities but *not* float position (§Phase 0); **fixing is where position finally changes.**
- **Ratio:** `sol.ratio ≥ 3.0` on accepted epochs.
- **Fix rate:** report % of converged epochs fixed over the session (diagnostic, not a hard pin
  first — real data carries legitimate extremes; make thresholds data-adaptive, not gut absolutes).

### Red-to-green summary
1. (0b) fixture replays SSR → float PPP runs, ambiguities OSB-corrected, position unchanged. 2.
`t_ppp_ar` synthetic red→green. 3. Wire §3 into `ppp_ar.c`, gate on modear. 4. Integration: first
`SOLQ_FIX` epoch (red until back-substitution + fix flags correct). 5. Assert fixed-vs-float position
improvement. 6. Freeze new golden case.

---

## 6. RISKS / OPEN QUESTIONS (carried from studies, ranked)

- **R1 — Phase-OSB substrate absent (§0).** HARD blocker; the premise's "already landed" is false in
  this checkout. Mitigation: Phase 0 (build loader) or Option 0b (SSR replay). Reconcile with
  whoever supplied the premise in case a Windows checkout differs.
- **R2 — est-mode ambiguities may be non-integer even after OSB.** The undifferenced `IB(s,f)`
  carries an unmodeled **receiver per-frequency phase bias** `β_f` (no estimated state exists).
  Mitigation is **mandatory between-satellite SD** (§3 Step 2), which cancels `β_f`. Residual open
  question (theory-level, only real data answers): after OSB + SD, is a residual fractional/systematic
  part left that still needs a WL-style decorrelation before LAMBDA reaches useful success rates?
  This is the deepest unknown; if SD-fixing fails at low ratio on the fixture despite correct
  algebra, this is the first suspect — and it would push toward Fork A's WL two-step after all.
- **R3 — `II(s)–IB(s,f)` covariance coupling.** `Pp` couples the per-sat iono state to the ambiguity
  in the same EKF row (`ppp.c:1036` vs `1044`) even though model_iono injects `var=0`. The SD `Q`/`Qab`
  slice must reflect this honestly; do not assume iono-decorrelated ambiguities.
- **R4 — `lambda()` s[] fill for tiny n.** `search()` case-2 (`lambda.c:122-140`) may not fill
  `s[1]` for very small n before exiting; untested. Cover with the tiny-n unit test before trusting
  the ratio in early-convergence / few-satellite epochs.
- **R5 — uncushioned hold feedback (§4).** Raw `matcpy` into the live filter, no damping. Argues for
  conservative QC before returning success. Do not weaken the ratio threshold early.
- **R6 — post-slip transient.** `udbias_ppp` seeds `f==0` with `ion=0` (`ppp.c:766-767`); the very
  first fix attempt after a cycle-slip, before `II(s)` re-converges, may be poisoned. Consider a
  post-reinit hold-off (N epochs / elevation gate) before admitting a sat to AR.
- **R7 — weak/untested EST mode.** Fork B rides `IONOOPT_EST`, the less-exercised iono mode (char
  cases use BRDC). New golden cover is part of the plan, not optional.
- **R8 — `rtkoutstat()` $SAT bias indexing bug for PPP** (`rtkpos.c:317` uses wrong macros). Not in
  scope to fix, but **do not trust $SAT bias columns** when debugging; read `rtk->x[IB()]`/`sol.stat`/
  `sol.ratio` directly, or budget the fix into characterization if $SAT-based debugging is wanted.
- **R9 — `test_hold_amb` sees only `fix[0]/fix[1]`** (hardcoded, `ppp.c:1172`), not `opt->nf`.
  Irrelevant at nf=2; if nf=3 AR is ever added, a 3rd-freq fix is invisible to hold-continuity —
  extending that loop is out of first-cut scope (touches existing plumbing).
- **R10 — reference-satellite continuity.** Re-pick every epoch (stateless, RTK-style) vs hold fixed
  across epochs? Recommended: hold the highest-elevation reference while eligible for a stable SD
  datum over PPP's long arcs; re-pick only on loss. Validate on the fixture.
- **Open — fixture availability:** WTZR/CODE OSB set not present in-repo; must be sourced.
- **Open — activation option:** should `ppp_ar` get a dedicated `pos2-pppar` option (recommended)
  vs reuse `modear!=ARMODE_OFF` (first cut) vs wire vestigial value 4? Design call, flagged.
