# Modern-PPP Theory Map for RTKLIB-EX

**Purpose.** An implementation-oriented synthesis of six sourced theory briefs, cross-checked
for consistency, mapping the *theory* of modern PPP onto RTKLIB-EX's concrete code seams. This
is a **theory dependency map**, not a coding plan. It exists to answer, for each of RTKLIB's
three modern-PPP gaps — PPP-AR (integer ambiguity resolution), PPP-RTK (atmosphere ingestion +
fast convergence), and HAS/B2b (new free-SIS correction decoders) — *what mathematics must be
true, which design forks a rebuild must consciously choose, and where in the tree each piece lands.*

> **Confidence discipline.** Every claim below inherits the confidence its source brief gave it.
> High-confidence = read from a primary ICD/spec or from RTKLIB source this session.
> Medium = paper-level or secondary-source-verified. Low / UNCONFIRMED = training-knowledge or
> AI-mediated extraction not verified against the typeset primary. Section 7 is the master
> "must verify against primary source" list. **Low-confidence items are flagged inline as such and
> are NOT laundered into confident prose.**

> ⚠️ **VERIFICATION STATUS (2026-07-22 primary-source pass — see companion `ppp_theory_verification.md`).**
> The §7 open list was run against primary sources: **14/18 reached items CONFIRMED-primary, 3 PARTIAL,
> 1 STILL-UNCONFIRMED** (item 18 not covered). **Two equations in THIS map were found WRONG — read the
> companion before quoting them:**
> 1. **§3.A.2 point 2 (decoupled clock)** — `dts_phase = dts_code + b_N^s/f_N` is **not in Laurichesse
>    2009 or Collins 2010**; both estimate phase- and code-clock as *separate* parameters. Treat that
>    equation as a pedagogical simplification, not a sourced formula. (Concept unchanged.)
> 2. **§3.B / §7 #13 (Odijk 2002 iono weighting)** — Odijk's distance model is **LINEAR** (`σ=β·d`,
>    β=0.3–3 mm/km, orig. Schaffrin & Bock 1988), **not the Gaussian** decay this map used as a stand-in.
> Also refined by the pass: item 9 `swl` is **2-bit / 3 WL groups** (not 1-bit); Galileo GGTO rides
> **Word Type 10** (not 16); **HAS phase bias still NOT operational** (Full Service target Q1–Q2 2027, so
> HAS real-time PPP-AR is infeasible until ~2027); RTCM **1265-1270 must NOT be called "ratified"**
> (draft/proposal-phase); **NavIC OSBs confirmed ABSENT** from current MGEX products; item 3 DOI fix
> `10.1007/s00190-007-0187-4`. Every SUBTRACT/offset sign checked (B2b, BDT, GGTO, URA) held; RTKLIB
> `var_urassr()` is bit-exact vs the IGS-SSR ICD. Full scoreboard + verbatim quotes: companion §1–§3.

---

## 1. Domain overview — how the three gaps interlock at the theory level

RTKLIB-EX's `pppos()` is a competent **float** PPP engine: forward-only iterated EKF, per-system
receiver clock, ZWD(+gradients), optional per-satellite slant iono, per-frequency float
ambiguities. Everything modern that it *lacks* is one of three tightly-coupled capabilities:

| Gap | One-sentence theory statement | Blocking dependency |
|-----|-------------------------------|---------------------|
| **PPP-AR** | The undifferenced float ambiguity is `B = λN + b_r + b^s` — an integer plus a receiver and a satellite fractional phase bias; it is only integer-recoverable once `b^s` (and effectively `b_r`) are removed by an external product. | Needs a **satellite phase-bias product** consumed consistently, then an **integer-fixing engine** (`ppp_ar.c` is a dead stub). |
| **PPP-RTK** | A regional network estimates the atmosphere (STEC per satellite, ZTD per region) as **state-space** quantities and broadcasts them so a single undifferenced user gets RTK-like *fast* convergence. | Needs an **ingestion path** that turns an external iono/tropo correction into a *weighted pseudo-observation* on the existing iono/ZWD states — plus the products to consume. |
| **HAS / B2b** | Galileo HAS (E6-B) and BDS-3 PPP-B2b are free SIS orbit/clock/bias streams; they are the *delivery formats* that would feed the two capabilities above. | Needs new **decoders** (transport + message parsers) that populate `ssr_t`, respecting each service's sign convention. |

**The interlock.** These are not three independent features — they share one algebraic spine
(Section 2) and one bias formalism (OSB, Section 4):

- PPP-AR is **useless without a phase-bias product**, and the phase-bias product is only
  *delivered* by SSR/HAS/B2b-class streams (Gap 3) or by static Bias-SINEX files. So Gap 1's AR
  engine sits directly downstream of Gap 3's decoders (and of a file-OSB loader RTKLIB lacks).
- PPP-RTK's atmosphere corrections ride the **same SSR transport** (IGS-SSR VTEC message IM201;
  CSSR grid for QZSS CLAS) as the orbit/clock/bias corrections — Gap 2 and Gap 3 share wire format.
- Teunissen & Khodabandeh (2015) unify all of it: **single-receiver PPP-RTK integer ambiguities
  are, algebraically, double-differenced ambiguities** — the network correction supplies the
  "other receiver." This is why PPP-AR and PPP-RTK are the same estimability problem viewed at two
  radii, and why the correction product's *S-basis/datum convention* is load-bearing for both.

The practical consequence for sequencing: **the functional model and the bias substrate come
first; the AR engine and atmosphere ingestion are peers built on top; the decoders are the supply
chain that makes either usable in the real world.** (Formalized in Section 6.)

---

## 2. Core functional model — the shared foundation

Everything builds on the **raw (uncombined), undifferenced multi-frequency** code/phase equations.
For receiver `r`, satellite `s`, frequency `j` (all terms in metres unless noted):

```
E[p^s_{r,j}] = m^s_r·τ_r + dt_r − dt^s + μ_j·l^s_r + d_{r,j} − d^s_j
E[φ^s_{r,j}] = m^s_r·τ_r + dt_r − dt^s − μ_j·l^s_r + δ_{r,j} − δ^s_j + λ_j·z^s_{r,j}
```

| Symbol | Meaning |
|--------|---------|
| `m^s_r·τ_r` | tropo mapping × zenith wet delay (+gradients) |
| `dt_r, dt^s` | receiver / satellite clock (metres) |
| `μ_j = (f_1/f_j)²` | ionosphere frequency-scaling (μ_1=1); scales **one** slant-iono state `l^s_r` to all freqs |
| `l^s_r` | slant ionospheric delay on the reference frequency (STEC×40.3/f_1²) |
| `d_{r,j}, d^s_j` | receiver / satellite **code** hardware bias |
| `δ_{r,j}, δ^s_j` | receiver / satellite **phase** hardware bias (cycles) |
| `λ_j·z^s_{r,j}` | integer ambiguity |

**Sign of ionosphere:** code is *delayed* (+μ_j·l), phase is *advanced* (−μ_j·l). Confirmed
consistent across all briefs and against RTKLIB (`C = (FREQL1/freq)²`, sign flips per obs type).

### 2.1 RTKLIB's residual template as an instance of this model

`ppp_res()` (ppp.c ~:1042):
```
res = y − (r + cdtr − CLIGHT·dts + dtrp + C·dion + dcb + bias)
```
Mapping to theory: `cdtr = dt_r` (one shared state per system, **no `dt^s` state** — satellite
clock is an *external* correction `dts[]`, not estimated); `C·dion = μ_j·l^s_r` (only if
`IONOOPT_EST`); `dcb` = the single L5 receiver-DCB term (only if `nf≥3`); `bias = x[IB(s,f)]`, the
float ambiguity that **silently absorbs both `λ_j·z` and any uncorrected `δ_{r,j}−δ^s_j`** because
neither phase bias is separately parameterized or corrected. *This absorption is exactly why float
ambiguities cannot be rounded — the physics/estimability gap is structural, not a bug.*

### 2.2 The two measurement-model branches (the architectural fork RTKLIB already has)

- **`IONOOPT_IFLC`** — strict dual-freq ionosphere-free LC (`seliflc()` rtkcmn.c:3741, hardcoded to
  exactly 2 freqs). `NF(opt)=1`: ONE combined ambiguity per satellite that is **not an integer
  number of cycles of anything** → cannot run LAMBDA directly; needs external WL/NL reconstruction.
  IF-LC amplifies noise (~3× for GPS L1/L2 code), needs the same 2 freqs every epoch, and destroys
  raw per-frequency info. (Zumberge 1997; Kouba & Héroux 2001.)
- **`IONOOPT_EST`** ("est-stec") — raw obs + per-satellite slant-iono state `II(s,opt)`.
  `NF(opt)=opt->nf`: genuine per-frequency float ambiguities, each integer-recoverable **after OSB
  correction**. This is the architecturally-correct branch to complete for modern PPP.

**Cross-brief agreement (high confidence):** RTKLIB's single-`l^s_r`-per-LOS iono parameterization
is the *correct* uncombined model — "RTKLIB got this part right." The gap is the absence of an
external-correction ingestion path, not the parameterization.

### 2.3 State layout and rank deficiency (S-system theory)

State vector (pppnx/NX macros, ppp.c:104-118):
`[pos 3|9][clk NSYS][ZWD(+2 grad)][slant-iono MAXSAT if EST][one L5 rcv-DCB if nf≥3][NF·MAXSAT float ambiguities]`.

The raw multi-freq network model is **rank-deficient**; S-system theory (Odijk et al. 2016; Zhang
et al. 2022) enumerates the singularities and resolves them by choosing an **S-basis** (parameters
fixed to a datum). For the code-plus-phase category: 8 singularity groups (sizes `n, m, n, m, 1,
f−2, fm, fn`) coupling {clock, code-IF-bias, phase-bias}, {iono, code-GF-bias, phase-bias},
extra-frequency code biases, and {phase-bias, ambiguity}. **Result: estimable ambiguities are
double-differenced** (Teunissen & Khodabandeh 2015). Phase-only processing adds one more
deficiency of size `(n−1)(m−1)`, forcing a non-native-wavelength estimable ambiguity.

> ⚠️ **Confidence flag.** The exact 8-group enumeration and S-basis subscripts come from an
> AI-mediated HTML→text extraction of Zhang et al. 2022 (SpringerLink was cookie-gated) and from
> Odijk et al. 2016 *as cited through* Zhang (the original was CAPTCHA-blocked). **Concepts are
> high-confidence** (they match the well-established Odijk/Teunissen S-system literature); **exact
> symbol subscripts and the phase-only wavelength-ratio `w_{1,j}/w_{2,j}` are medium** — verify
> against the typeset PDF before byte-quoting in a spec. (Brief 1 openQuestions 1-2; Brief 4 oQ 1-3.)

**Extra-frequency (j>2) residual bias.** Each frequency past 2 introduces exactly one new
inestimable receiver- and satellite-side bias pair: `d̄_{r,j} = d_{r,j} − d_{r,IF} − μ_j·d_{r,GF}`.
RTKLIB's `ND(opt)=(nf≥3?1:0)` carves out **exactly one** such state (the L5 receiver-DCB,
receiver-side only). A rigorous quad-freq rebuild needs `ND` generalized to `(nf≥3?nf−2:0)` **and**
satellite-side handling — a single shared ND state does not generalize.

**IFCB caveat (do not conflate with the L5 DCB).** GPS Block IIF/III show a real, *time-varying*
(10–40 cm) satellite-side L1/L2-vs-L5 apparent-clock discrepancy (Montenbruck et al. 2012). This is
`IFCB^s(t)`, applied to L5 phase — **satellite-side and time-varying**, structurally distinct from
RTKLIB's `ND/ID(opt)` term which is **receiver-side and constant**. A triple-freq rebuild needs
*both*; they are not interchangeable.

---

## 3. Per-component theory

### 3.A PPP-AR = bias products + AR engine + quality control

#### 3.A.1 Why float ambiguities are non-integer (the prerequisite)
A standard **code-consistent** satellite clock (SP3/CLK, broadcast, SSR) uses the ionosphere-free
code combination as its implicit "Satellite Bias Datum" — code hardware delays are absorbed into
the clock. Carrier phase has its *own* independent instrumental delays that do **not** cancel
against that datum. Hence `B^s_r = λN^s_r + b_r + b^s`, and only after `b_r`, `b^s` are externally
supplied and subtracted can the remainder be rounded. (ESA Navipedia; Ge et al. 2008 lineage.)

#### 3.A.2 Three families of integer-recoverable bias products (all mathematically equivalent)
1. **UPD/FCB two-step (Ge et al. 2008).** Estimate wide-lane satellite+receiver FCB first (from the
   geometry-and-iono-free Melbourne-Wübbena combination), fix `N_W` by rounding (λ_W≈0.86 m makes
   this safe); then narrow-lane FCB, fix `N_N`.
   - MW combination (verified against RTKLIB `mwmeas()` ppp.c:395, and ESA Navipedia):
     `B_W = Φ_W − R_N = λ_W·N_W + b_W + ε`, `Φ_W=(f_1L_1−f_2L_2)/(f_1−f_2)`,
     `R_N=(f_1P_1+f_2P_2)/(f_1+f_2)`, `λ_W=c/(f_1−f_2)`.
   - IF/WL/NL relation: `N_IF = N_N + (f_2/(f_1−f_2))·N_W`.
   - ⚠️ **Medium confidence** on the exact Ge/Laurichesse equation *numbering/notation* — full texts
     were paywalled; the equations are training-reconstructions cross-checked against fetched primary
     specs. Verify before byte-quoting.
2. **Decoupled clock / Integer Recovery Clock (Laurichesse & Mercier 2009; Collins 2010).** Fold the
   satellite NL-FCB directly into a **second clock product**: `dts_phase = dts_code + b_N^s/f_N`.
   Applying `dts_phase` to phase (and `dts_code` to code) makes the NL ambiguity directly
   integer-recoverable. Corrects the **model** (clock) instead of the **observation**.
   ⚠️ Medium confidence (paper-level, full text not fetched).
3. **OSB (Observable-Specific Bias) — the modern unifying format (IGS Bias-SINEX v1.00, HIGH
   confidence, spec read directly).** One pseudo-absolute bias per (constellation, RINEX3 signal),
   for **code (ns) and phase (cyc)**. Corrects the **observation**: `O' = O − B_O`.
   - Sign convention: `bias = observation − true`, so `corrected = observed − bias`.
   - Transforms to legacy products: `B_D(c1,c2)=B_O(c1)−B_O(c2)`;
     `B_I(c1,c2)=κ1·B_O(c1)+κ2·B_O(c2)`, `κ1=f_1²/(f_1²−f_2²)=2.546`, `κ2=−f_2²/(...)=−1.546`
     (GPS L1/L2). Inverse: `B_O(c1)=B_I+κ2·B_D`, `B_O(c2)=B_I−κ1·B_D`.

**Design fork the rebuild must choose:** does RTKLIB consume the **OSB/observation-correction**
convention (which it already follows — see 3.A.3) or additionally the **decoupled/phase-clock**
convention? These are **not numerically interchangeable**; `nav_t`/`ssr_t` schema must decide which
(or both). No source mandates one; genuine architecture choice.

#### 3.A.3 What RTKLIB *already* does (reconciling a cross-brief conflict)
> **Consistency note — resolved by reading source this session.** Brief 2 states a phase-bias
> consumption path **exists**; Briefs 5, 6 and the parent audit state pbias is **never consumed**.
> **Both are right about different files.** Verified directly:
> `corr_phase_bias_ssr()` (postpos.c:418-430) and `corr_phase_bias()` (rtksvr.c:515-526) DO apply
> `obs[i].L[j] -= ssr[sat-1].pbias[code-1]·freq/CLIGHT` **before** `pppos()` runs — this is the
> textbook-correct OSB observation-correction, and in post-processing it runs **by default** (gated
> `if (!strstr(popt->pppopt,"-ENA_FCB"))`). The "never consumed" claim is true only **inside ppp.c**
> (grep of ppp.c for `pbias` = 0 hits). **The real gaps are therefore narrower and sharper:**
> 1. The **file-based** loader `readbiaf()` skips phase (`if(obs1[0]!='C') continue`, preceph.c:473),
>    so for SP3/CLK/Bias-SINEX post-processing the correction is a **silent no-op** (pbias empty).
>    Only the **real-time RTCM-SSR** stream ever populates pbias.
> 2. Even when pbias *is* applied, `ppp_ar()` is a dead stub → nothing exploits the now-quasi-integer
>    ambiguity. Consumption math is fine; the **loader and the fixing engine** are what's missing.

Code bias is applied *inside* `corr_meas()` (ppp.c:409, `P[i] += cbias[code]−cbias[ref]` for
EPHOPT_SSRAPC/SSRCOM; else `P[i] −= code2bias(...)` from the file DCB/OSB table). Satellite `dts`
stays an ordinary code-consistent clock (ephemeris.c `satpos_ssr()`) — RTKLIB followed the
**OSB/observation-correction** convention, not decoupled-clock.

#### 3.A.4 The AR engine (integer estimation theory — Teunissen 2003, HIGH confidence, read in full)
Model `E{y}=Aa+Bb`, `a∈Z^n`. Float → integer → fixed (remove-restore):
`b̌ = b̂ − Q_{b̂â}Q_â^{-1}(â−ǎ)` (exactly RTKLIB's `resamb_LAMBDA` back-substitution, rtkpos.c:1784).

Estimator hierarchy **I ⊂ IA ⊂ IE**:
- **Rounding** `ǎ_R=[â]` (unit-cube pull-in) — used for WL because λ_W≈0.86 m makes it safe.
- **Bootstrapping** — sequential conditional rounding via `Q_â=L^T·D·L`; has a **closed-form
  success rate** `P_{s,IB}=∏(2Φ(1/(2σ_{i|I}))−1)`. This is the cheap pre-check RTKLIB *should* use
  and currently substitutes with an ad-hoc position-variance threshold.
- **ILS** `ǎ_LS=argmin‖â−z‖²_{Q_â}` — ML-optimal under Gaussian (Theorem 1). **This is LAMBDA**,
  fully implemented in `src/lambda.c` (LD + reduction/Z-decorrelation + MLAMBDA search) and
  unit-tested (`t_lambda.c`) — but **orphaned from the PPP path**.

**Validation / QC (the currency of AR):** success/failure/undecided probabilities `P_S, P_F, P_U`.
- **Ratio test** (an Integer-Aperture estimator): accept if `s[1]/s[0] ≥ 1/ρ` (RTKLIB `thres`,
  nominal ≈3.0). **No failure-rate guarantee.**
- **Difference test** — close-to-optimal IA when float pdf is peaked; theoretically preferable.
- **FF-RT (Fixed-Failure-Rate Ratio Test)** — critical value `μ = f_μ(P_{f,ILS}; n)` from lookup
  tables, so realized `P_F ≤` tolerance regardless of geometry. **This directly contradicts
  RTKLIB's fixed/adaptive-polynomial `ar_poly_coeffs` heuristic** (rtkpos.c:96-99), which is an
  empirical curve-fit to satellite-pair count with **no stated failure-rate bound**.

**Partial AR (PAR).** RTKLIB's `arfilter/mindropsats/excsat` round-robin exclusion is a reactive,
RTK-tuned heuristic, **not** grounded in bootstrapped-success-rate subset selection (Verhagen et al.
2011). ⚠️ Low confidence on the exact principled-subset algorithm (paper not fetched); the
*existence/motivation* of PAR theory is well-established.

**RTKLIB integration fork (from Brief 3, high confidence — read source):** two structurally
different `ppp_ar()` rebuild paths — **(A)** IFLC + external WL/NL reconstruction (needs a new
persistent per-satellite MW/WL accumulator in `ssat_t` — does not exist today), or **(B)**
IONOOPT_EST + LAMBDA directly on per-frequency `IB(s,f)` states after per-signal OSB correction
(reuses existing state layout; lower-effort, but on the currently weaker/less-tested iono mode).
The choice must be explicit.

### 3.B PPP-RTK = atmosphere ingestion + fast convergence

**Core idea (Wübbena et al. 2005).** A regional network estimates atmosphere as **state-space**
quantities and broadcasts them; the undifferenced user applies them directly → RTK-like fast AR.
RTKLIB structurally *cannot* reach PPP-RTK convergence no matter how the local filter is tuned —
the missing piece is **architectural** (a correction-consuming layer), not parameter tuning.

**Ionosphere as a weighted pseudo-observation (Psychas & Verhagen 2020, HIGH confidence,
open-access, read in full).** Do **not** hard-substitute the network value; **append a row**:
```
E(δι) = ι_u^s − ι_{net→u}^s ,   D(δι) = σ²_{δι}·I_m
```
so the filter down-weights a poor correction. This is the single biggest structural difference from
RTKLIB: `model_iono()` (ppp.c:918-923) sets `var = 0.0` hardcoded, and `udiono_ppp()` only ever
**widens** the iono variance (random walk `P_jj += (prn[1]/sin el)²·|dt|`), never narrows it toward
an external value. RTKLIB's only "reconvergence" is a **cold restart** after `GAP_RESION` (120 ep):
zero the state, re-init from raw pseudorange with `VAR_IONO=60²` — the *opposite* of a warm start.

**Correction variance is distance/quality-dependent, not constant.** Gaussian spatial-covariance
`h_ij = c_ι²·exp(−(l_ij/l_0)²)` + BLUP/kriging interpolate correction *and its uncertainty* to the
user; `σ_{δι}` is empirically calibrated against independent "truth." RTKLIB has **no field
anywhere** (ssr_t or otherwise) to carry a correction quality indicator through decode → filter.

**Ingestion changes estimability, not just adds a constraint.** The network iono correction is
biased by (pivot-receiver and satellite) geometry-free DCBs, so introducing it makes the user's own
receiver DCB **estimable**: `E(ι̂_{net→u}^s) = ι_u^s − d^{s,GF}`. A naive "just subtract" is biased.
Relevant to generalizing RTKLIB's existing L5-only `ID(opt)` DCB machinery.

**Convergence numbers (sourced, replaces the vague "20-30 min → seconds"):** float PPP ~28.5 min
(P50) / 68.5 min (P90) to 10 cm; with dense (68 km) network iono, P90 < 6 min; sparse (237 km) P90
~20 min. **Benefit scales ~linearly with network density** — not a universal instantaneous fix.

**Reality check (Brief 4/5, medium confidence).** Atmosphere support is the *exception*: only QZSS
CLAS (Compact SSR / L6D, ~300 CORS, Japan) and commercial SPARTN carry it operationally. IGS-SSR
defines a standalone VTEC message (RTCM 1264 / IGS-SSR **IM201**) but troposphere has **no**
standardized RTCM/IGS-SSR message. Galileo HAS SL1 and BDS-3 PPP-B2b carry **no atmosphere**.
→ A rebuild's realistic near-term target is **CLAS/CSSR or a self-hosted regional product**, not a
global free service.

### 3.C SSR / CSSR message structure (the supply chain)

**Two families, same content (HIGH confidence, IGS-SSR v1.00 read directly; RTKLIB source read).**
Native RTCM-SSR uses dedicated message numbers (1057-1068 GPS/GLO ratified); IGS-SSR wraps
everything in RTCM-proprietary **MT4076** with an internal subtype field (IM021-027 GPS … IM101-107
BDS, IM121-127 SBAS, **IM201 VTEC**). RTKLIB's `decode_type4076()` implements both via shared
`decode_ssr1..7()`, cross-checked 1:1 against the spec's subtype table — a strong template for how a
HAS/B2b decoder should be structured (message-specific bit parser → protocol-agnostic `ssr_t` →
shared `satpos_ssr()` consumption).

**Orbit/clock model (verified sign-for-sign against `satpos_ssr()`):**
- Triad `e_along=ṙ/|ṙ|`, `e_cross=(r×ṙ)/|r×ṙ|`, `e_radial=e_along×e_cross`.
- `δO(t)=[δO]+[δȮ]·(t−t0)`; `X_orbit = X_broadcast − [e_r e_a e_c]·δO` (**SUBTRACT**).
- `δC(t)=C0+C1(t−t0)+C2(t−t0)²[+HighRate]`; `t_sat = t_broadcast − δC/c`. Relativistic
  `−2·r·v/c²` still applied separately from broadcast — SSR clock never subsumes it.

**The critical sign trap (HIGH confidence, both ICDs read):**

| Service | Orbit | Clock | Code bias | Reuse `satpos_ssr()`? |
|---------|-------|-------|-----------|------------------------|
| IGS-SSR / RTCM-SSR | SUBTRACT | SUBTRACT | `P += cbias−cbias_ref` | Yes (native) |
| **Galileo HAS** | **ADD** (`X̃=x+δX`) | **ADD** (`dt̃=dt+Δt_r+δC/c`) | **ADD** (`P̃=P+d̃`, replaces BGD/TGD) | Yes **iff HAS populator negates δR/δC** before storing |
| **BDS-3 PPP-B2b** | SUBTRACT | SUBTRACT (**C0 only**, no C1/C2) | SUBTRACT (`l̃=l−DCB`) | Yes (no sign flip) |

A HAS decoder naively reusing the subtract-convention `satpos_ssr()` would **double-sign** the
error. B2b is sign-compatible; its single-coefficient clock re-transmits fresh C0 every epoch.

**VTEC / ionosphere SSR (IGS-SSR IM201).** Spherical-harmonic `VTEC(φ_PP,λ_PP)=ΣΣ(C_nm cos mλ_S +
S_nm sin mλ_S)P_nm(sin φ_PP)`, evaluated at the pierce point, mapped `STEC=VTEC/sin(E+ψ_PP)`,
converted `δ=±(40.3/f²)·STEC` (+code / −phase). **Absent from RTKLIB** — no VTEC decoder, no
pierce-point/SH evaluator, no `ssr_t` field to store coefficients. This is the concrete named
missing message behind "NO PPP-RTK atmosphere ingestion."

**Phase-bias integrity flags (the "is this bias integer-safe" contract).** IGS-SSR phase-bias
messages carry per-signal **Signal Integer Indicator (sii)**, **Wide-Lane Integer Indicator (swl)**,
**Signal Discontinuity Counter (sdc)**, and per-header **Dispersive-** and **MW-Consistency
Indicators (dispe/mw)**. RTKLIB's `decode_ssr7()` decodes all five as **local variables and discards
them** — `ssr_t` has no fields for them. Without these, no AR engine can tell an integer-recoverable
bias from a float-only one, or detect a bias-arc discontinuity that should force an ambiguity reset.
⚠️ **Medium-low confidence** on the *exact* RTCM 10403.x/IGS-SSR semantics of these bits (the
standard text was not fetched; treat as training-inference until verified). The *grep fact* that
RTKLIB discards them is high-confidence.

**HAS transport (HIGH confidence, ICD read).** Completely different stack: E6-B C/NAV pages, HPVRS
Reed-Solomon RS(255,32,224) recovery across satellites, MT1 mask/orbit/clock/bias blocks. **No
incremental extension of rtcm3.c reaches HAS** — it needs a new subsystem fed from raw E6-B symbols
(which `rcv/ublox.c` RXM-SFRBX currently *receives and discards*, ublox.c:1225). B2b: LDPC-coded
B2b-I navigation message, MT1-7, no phase-bias type at all → **B2b is structurally float-only PPP**.

---

## 4. Multi-GNSS / bias substrate (what §3 depends on)

**Per-system receiver clock / ISB (Kouba 2009; Odijk & Teunissen 2013, HIGH confidence, read).**
One master `dt_r,GPS` + one ISB per extra system: `ISB_sys = dt_r,sys − dt_r,GPS`. RTKLIB implements
exactly this: `IC(s,opt)` states, dispatched by `switch(sys)` (ppp.c:1009-1015): GLO=1, GAL=2,
CMP=3, IRN=4, default(GPS/QZS/SBS)=0.
- **Phase-vs-code asymmetry:** phase DISB `δ_o^AB = (δ_o^B−δ_o^A) + z_o` carries an **unknown
  integer** → recoverable only as a fractional cycle; code DISB is real-valued/unambiguous. Same
  reason PPP-AR needs a fractional-bias framework.
- ISB is **receiver-pair-specific but stable over hours-days** → should be a slowly-varying
  (random-walk) state. **RTKLIB hard-resets every clock/ISB state to white noise every epoch**
  (`udclk_ppp()` ppp.c:612-631) — cannot exploit ISB stability.
- BDS-2/BDS-3 sub-ISB and GPS-QZSS ISB are non-negligible (medium confidence, MDPI 2023;
  GPS-Solutions 2016). RTKLIB folds QZSS into GPS and gives BDS-2/BDS-3 one shared state — defensible
  simplifications, documented approximations, not laws.

**GLONASS FDMA inter-frequency bias (IFB) (HIGH confidence, Sleewaegen 2012 + Wanninger 2012 read).**
`f_L1^k = 1602 + k·0.5625 MHz`. IFB's dominant source is the **digital correlator** (not analog RF):
a fixed time delay `dt_CP = dt_C − dt_φ − dt_PPS` → phase bias **linear in carrier frequency, hence
linear in channel k**, numerically **equal on L1/L2 in length units**. It is a **deterministic,
calibratable bias** (one slope per receiver), not noise. **RTKLIB models it as pure variance
inflation** (`VAR_GLO_IFB=0.6²`, applied only to GLONASS *code*, ppp.c:1053) — protects against
blunders but never removes the systematic part, and gives phase **zero** compensation. Three real
calibration options exist (per-brand lookup; single estimated slope state; firmware-level fix); the
natural RTKLIB fit is a **single estimated `dt_CP`/slope state** (the FCN plumbing via `sat2freq()`
already exists). ⚠️ Open: whether modern firmware auto-compensates IFB, making variance-inflation a
more reasonable simplification for today's receiver population.

**Terminology collision to document (HIGH confidence).** "ISB" means **inter-system
(receiver-clock) bias** in the PPP-filtering literature (RTKLIB's `IC()`), but **ionosphere-free
signal bias** in a Bias-SINEX file (`B_I = κ1·B_O(c1)+κ2·B_O(c2)`). A rebuild's docs must call this
out explicitly.

**DCB/OSB signal coverage (the silent-failure substrate).** RTKLIB's `code_bias_ix[NSYS][MAXCODE]`
(preceph.c:62-100) has **no entries for QZSS, NavIC, or BDS-3 B1C/B2a** → `code2bias()` indexes with
`code_ix=−1` (out-of-bounds read, undefined behavior, silently wrong bias). Also `corr_meas()`'s SSR
reference-code selector `ix` is only assigned for GPS/GLO/GAL → BDS/QZS/IRN silently use table index
0 as reference in real-time SSR mode. Both must be fixed *and* a defensive `code_ix<0 → return 0.0`
bounds-check added. ⚠️ Low confidence on *current* MGEX OSB coverage for new signals (moving target,
no live Bias-SINEX header fetched). ⚠️ Needs verification: exact `CODE_` enum names for QZSS/NavIC/
BDS-3 signals in `code2freq_*()` before writing table entries.

**Time-system alignment (medium confidence).** GPST-TAI=−19 s; GST designed GPST-aligned (residual
in broadcast GGTO); **BDT = GPST − 14 s (fixed)**; GLONASST = UTC(SU)+3h (implements leap seconds →
drifts vs GPST). The **SSR product supply chain resolves these before broadcast** (ssr_t time-tags
everything in GPST); only the residual (receiver hardware + imperfect product alignment) is left for
the ISB state. HAS explicitly warns of a residual GPS-clock common offset the *user* must absorb
(ICD §7.3) — concrete proof that per-system ISB states catch supply-chain residuals, not just
receiver hardware. ⚠️ The 14 s BDT-GPST figure and GGTO field layout were search-synthesized, not
read from a primary ICD table this session.

---

## 5. RTKLIB integration seams (theory piece → file:symbol → what to build)

| Theory piece | Existing seam | Gap | What to build |
|---|---|---|---|
| Uncombined state layout | `ppp.c:104-118` NF/NI/ND/IB macros | ND fixed at 1 for nf≥3; no sat-side bias states; ambiguities absorb residual phase bias | Generalize `ND→(nf≥3?nf−2:0)`; add sat-side recombined-bias handling (state+S-basis, or external OSB) |
| Phase-OSB **observation** correction | `postpos.c:418-430` `corr_phase_bias_ssr()`, `rtksvr.c:515-526` `corr_phase_bias()` | **Works for real-time SSR**; no-op for file products (pbias empty) | (No change to consumption math) |
| Phase-OSB **file loader** | `preceph.c:473` `readbiaf()` `if(obs1[0]!='C')continue` | Bias-SINEX PHASE OSBs (unit cyc) never loaded → **file-based PPP-AR impossible** | Add `nav_t` phase-OSB table (mirror `cbias`); route `obs1[0]=='L'` records; feed same pre-correction |
| Integer AR engine | `ppp_ar.c` stub → `ppp.c:1251` short-circuits | No integer estimation; `SOLQ_FIX` unreachable | Rebuild `ppp_ar()`: assemble (nb, y, Qb) for PPP ambiguities → `lambda()` → validate → back-substitute into xp/Pp → set `ssat[s].fix[f]=2` |
| LAMBDA primitive | `lambda.c` `lambda()` | Orphaned from PPP; already unit-tested | Reuse unchanged |
| AR back-substitute + PAR template | `rtkpos.c:1706-1946` `resamb_LAMBDA`/`manage_amb_LAMBDA` | RTK-only (DD); not shared with ppp.c | Factor LAMBDA-call+validate+back-substitute into shared helper, or write `ppp_resamb_LAMBDA()` |
| Phase-bias integrity flags | `rtcm3.c` `decode_ssr7()` (sii/swl/sdc/dispe/mw as locals) | All 5 discarded; `ssr_t` has no fields | Add fields; gate AR on `sii&&mw`; force ambiguity reset on `sdc` change |
| QC / failure-rate | `ppp.c:1258` `MAX_STD_FIX=0.15` position gate; `ar_poly_coeffs` | No `P_S/P_F/P_U`; ratio has no failure-rate bound | Ratio→**FF-RT** inside ppp_ar; keep MAX_STD_FIX as secondary/defensive only |
| Iono pseudo-observation | `ppp.c:918-923` `model_iono()` EST (`var=0.0`); `udiono_ppp()` random-walk only | No external-correction ingestion; variance only widens | Add pseudo-obs branch `v=x[II(s)]−ι_corr`, `H=e_{II(s)}`, `R=σ_corr²` |
| Iono warm-start | `ppp.c:668-696` `GAP_RESION` cold restart | Always cold-restarts from pseudorange | On outage-reset, `initx(..,ι_corr,σ_corr²,..)` if a valid external correction exists |
| Atmosphere container | `rtklib_types.h` `ssr_t` | No stec/vtec/ztd/grid fields; no quality field | Add SH-VTEC coeff struct (mirror IONEX `nav->tec`) + per-value std-dev |
| VTEC / atmosphere decode | `rtcm3.c` `decode_type4076()` (stops at orbit/clock/bias); outer switch stops at 1263 | IM201 VTEC + 1264/1265-1270 never dispatched | Add IM201 SH parser; add 1264/1265-1270 cases |
| ISB propagation | `ppp.c:612-631` `udclk_ppp()` white-noise reset | Cannot exploit ISB stability | Random-walk process noise (mirror `udtrop_ppp`) |
| QZSS/NavIC/BDS-3 code bias | `preceph.c:62-100` `code_bias_ix`; `corr_meas()` `ix` selector | −1/OOB reads; SSR ref-code unset for BDS/QZS/IRN | Populate table rows; add BDS/QZS/IRN ref-code branches; bounds-check `code_ix<0` |
| GLONASS IFB | `ppp.c:87/1053` `VAR_GLO_IFB` | Variance-inflation only; phase uncompensated | Add per-receiver `dt_CP`/slope state; subtract `slope·k` in residual (FCN via `sat2freq()`) |
| HAS decoder | `rcv/ublox.c:1225` E6 CNAV discarded | No HPVRS RS, no C/NAV reassembly, no MT1 parser | New module: RS(255,32,224) recovery → MT1 parse → `ssr_t` with **negated** δR/δC |
| B2b decoder | `rcv/unicore.c`,`binex.c` (ranging only) | No LDPC nav-message decode, no MT1-7 parser | New module: LDPC + MT1-7 → `ssr_t` (subtract, C0-only clock) |
| SSR yaw consistency | `ssr_t.yaw_ang/yaw_rate` decoded, never read | Wind-up always uses homegrown `yaw_angle()` | When pbias consumed, switch to provider yaw during eclipse to stay bias-consistent |

---

## 6. Dependency-ordered rebuild sequence (THEORY dependency graph)

Ordering by *what mathematics/data must exist before the next layer is meaningful* — not effort.

```
L0  FOUNDATION (must be solid before anything downstream is even well-posed)
    ├─ Uncombined functional model + state layout (§2)         [have it, IONOOPT_EST]
    └─ OSB/DSB/ISB bias algebra + sign convention (§3.A.2, §4)  [have code side]

L1  BIAS SUBSTRATE (what L2 consumes)
    ├─ Per-signal code-bias coverage: QZSS/NavIC/BDS-3 + bounds-check (§4)
    ├─ Phase-OSB storage in nav_t + file loader (readbiaf phase) (§3.A.3 gap 1)
    └─ Phase-bias integrity flags persisted in ssr_t (sii/mw/sdc) (§3.C)
         │  (Teunissen-Khodabandeh: PPP-AR ambiguities are DD → the S-basis/datum
         │   of these products is load-bearing; must be consistent before fixing)
         ▼
L2  TWO PEER CAPABILITIES (both stand on L0+L1; independent of each other)
    ├─ PPP-AR ENGINE                          ├─ PPP-RTK ATMOSPHERE INGESTION
    │   ├─ ppp_ar(): assemble→lambda()→        │   ├─ atmosphere container in ssr_t/nav_t
    │   │   validate(FF-RT)→back-substitute     │   ├─ iono pseudo-obs branch (var≠0)
    │   ├─ set ssat.fix=2 → test_hold_amb       │   └─ warm-start replaces GAP_RESION cold restart
    │   └─ choose fork A(IFLC+WL/NL) or         │        (needs external STEC + its σ)
    │        B(EST+per-freq LAMBDA)             │
    │  gated on L1 phase-bias + integrity flags │  gated on L1 (needs a correction to ingest)

L3  SUPPLY CHAIN (delivers the products L2 needs, in the real world)
    ├─ IGS-SSR IM201 VTEC decoder  ──────────────▶ feeds PPP-RTK
    ├─ Galileo HAS decoder (transport+MT1, negate signs) ─▶ feeds PPP-AR (SL1 code/clock now;
    │                                                       phase-bias when operational)
    └─ BDS-3 PPP-B2b decoder (LDPC+MT1-7) ───────▶ feeds float PPP only (no phase bias)

L4  REFINEMENTS (measurable but not blocking)
    ├─ ISB random-walk propagation (exploit stability)
    ├─ GLONASS IFB estimated slope state (retire variance-inflation)
    ├─ Triple-freq: ND→nf-2, satellite-side IFCB
    └─ SSR-provider yaw for eclipse wind-up consistency
```

**Key ordering insights:**
- **L1 before L2, always.** An AR engine on top of missing/inconsistent phase biases fixes to
  *wrong* integers; ingesting an atmosphere correction with no quality field can't be weighted. The
  substrate is the gate, echoing the T&K-2015 result that the *datum consistency* of the product is
  what makes AR sound.
- **PPP-AR and PPP-RTK are peers, not sequential.** They share L0+L1 but neither needs the other.
- **L3 decoders are the supply chain, not the capability.** They can be built in parallel with L2;
  L2 can be developed/tested against *file* products (post-processing) before any real-time decoder
  exists — which is exactly why the **file phase-OSB loader (L1)** is the highest-leverage single
  addition (it unblocks post-processed PPP-AR without touching any decoder).

---

## 7. Open questions / primary-source verification list

Carried forward verbatim from all six briefs — **must verify against primary source before treated
as ground truth in an implementation spec.** Grouped by artifact to fetch.

**Peer-reviewed papers not read in full (equations are secondary/training-reconstructed):**
1. **Zhang, Hou, Zha & Liu (2022), Sat. Nav. 5:5** — the 8-group rank-deficiency enumeration,
   S-basis subscripts, phase-only wavelength-ratio `w_{1,j}/w_{2,j}`. Extracted via r.jina.ai
   reader, not the typeset PDF. *Concepts high; exact subscripts medium.* (NCBI/PDF mirror not tried.)
2. **Odijk, Zhang, Khodabandeh, Odolinski & Teunissen (2016), J.Geod. 90:15-44** — the original
   S-system paper; cited *through* Zhang 2022, never fetched (CAPTCHA). Read for canonical eq numbering.
3. **Ge et al. (2008), J.Geod. 82:389-399** & **Laurichesse & Mercier (2009), Navigation 56:135-149**
   & **Collins et al. (2010), Navigation 57:123-135** — WL/NL FCB-separation and decoupled/phase-clock
   equations are training-reconstructions cross-checked against ESA Navipedia; success-rate figures
   ("~80%", "27% repeatability") not re-derived. Paywalled.
4. **Teunissen & Khodabandeh (2015), J.Geod. 89:217-240** — the "single-receiver ambiguities are DD"
   concept is high-confidence and consensus, but the *exact S-basis equations* were not verified this
   session (Springer IDP wall). The Curtin PDF of Teunissen (2003) *was* read in full and is solid.
5. **Khodabandeh & Teunissen (2015), J.Geod. 89:1109-1132** — correction-precision→user-AR-success
   propagation; 403/paywall, abstract only.
6. **Verhagen et al. (2011) "which subset to fix"** — principled PAR subset selection; search-summary
   only (low confidence on exact algorithm).

**ICDs / format specs to fetch (message-field/bit-level unverified):**
7. **Galileo HAS operational phase-bias status** — ICD *defines* phase biases but SDD (Jan 2023) says
   "not yet provided" under Initial Service SL1. **Check current GSC-Europa HAS Info Note / SDD
   revision / NAGU** before scoping HAS-based PPP-AR as usable *today*. (gsc-europa.eu)
8. **BDS-3 PPP-B2b ICD (BDS-SIS-ICD-PPP-B2b-1.0)** — MT3 code-bias bit-width/scale/sign, MT1-7 field
   layout, LDPC matrix. Primary PDF (en.beidou.gov.cn) refused connection twice; only secondary
   characterization obtained. Verify before writing a decoder.
9. **RTCM 10403.x** — (a) whether dedicated phase-bias message numbers **1265-1270** are ratified
   (only store-listing/community sources; RTKLIB uses tentative 11-14 + MT4076); (b) exact bit-level
   **URA** encoding vs `var_urassr()`; (c) exact semantics of **sii/swl/sdc/dispe/mw** integrity flags.
   Standard is paywalled — the *grep facts* about RTKLIB (discards flags, no 1265-1270 cases) are solid.
10. **IGS-SSR format PDF** — the exact **IM201 VTEC subtype number inside MT4076** (WebFetch returned
    unparseable byte-stream; only a secondary igs.org table used). Parse with a real PDF text extractor
    before writing the VTEC decoder.
11. **IS-QZSS-L6 (CLAS/CSSR)** — atmosphere grid/functional-model bit layout (pp.53,55) only
    characterized qualitatively via a secondary comparison paper. Needed if targeting CLAS.
12. **Galileo OS SIS ICD (GGTO) + BeiDou B1I/B3I ICD (BDT-GPST=14 s)** — read the primary time-offset
    tables to lock down the alignment figures (search-synthesized this session).
13. **Odijk (2002) TU Delft thesis** — original distance-dependent iono weighting; PDF returned binary.
    Substituted the verified Psychas & Verhagen 2020 Gaussian form; fetch the original only for
    historical/comparative context.

**Code reads to do before implementing (this checkout):**
14. `rtkcmn.c` `code2freq_BDS/QZS/IRN()` — confirm exact `CODE_` enum constants before writing
    `code_bias_ix[]` rows for QZSS/NavIC/BDS-3 B1C/B2a (§4, §5).
15. Whether `decode_ssr7_head()`'s `dispe/mw` header bits are used anywhere downstream (appeared
    unused in the excerpt read) — trace before relying on RTKLIB to respect consistency flags.
16. Whether `ar_poly_coeffs` was ever calibrated against FF-RT theory or is pure empirical curve-fit
    (rtklibexplorer forum/commit archaeology) — decides whether FF-RT is a clean upgrade.
17. Whether RTKLIB's PPP receiver-side phase-bias nuisance is already absorbed by the `IC()` clock
    states (making between-satellite SD redundant) or needs a separate parameter — trace
    `corr_meas()`/`ppp_res()` before choosing the ppp_ar assembly.

**Empirical / currency questions (out of literature-only scope):**
18. Adequacy of `VAR_GLO_IFB` vs a fully-parameterized per-receiver GLONASS IFB (needs zero/short-
    baseline data); whether modern firmware auto-compensates IFB.
19. Live MGEX/IGS Bias-SINEX coverage for BDS-3 B1C/B2a, Galileo E6, modernized QZSS, NavIC (fetch a
    current file header; low confidence today).

---

## 8. Key references (consolidated, deduplicated)

**Primary specs / ICDs (read in full or fetched this session — HIGH confidence):**
- IGS **SINEX_BIAS v1.00** (Schaer) — OSB/DSB/ISB, sign convention, κ transforms.
  https://files.igs.org/pub/data/format/sinex_bias_100.pdf
- IGS **State Space Representation (SSR) Format v1.00** (2020) — MT4076 subtypes, orbit/clock/bias/
  URA/VTEC, phase-bias integrity flags. https://files.igs.org/pub/data/format/igs_ssr_v1.pdf
- **Galileo HAS SIS ICD v1.0** (May 2022) — HPVRS RS, MT1 blocks, ADD-convention eqs 18-29.
  https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_HAS_SIS_ICD_v1.0.pdf
- **Galileo HAS SDD v1.0** (Jan 2023) — SL1 vs SL2; "phase biases not yet provided."
  https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-SDD_v1.0.pdf
- **BDS-3 PPP-B2b ICD v1.0** (2020-07) — MT1-7, DCB/URA eqs 7-1..7-11 (PDF blocked; verify #8).
  http://en.beidou.gov.cn/SYSTEMS/ICD/202008/P020231201538195573144.pdf
- **Teunissen (2003), "Towards a unified theory of GNSS AR", JGPS 2(1):1-12** — full text read; all
  integer-estimation theory. https://gnss.curtin.edu.au/wp-content/uploads/sites/21/2016/04/Teunissen2003Towards.pdf

**Primary papers (read/fetched — HIGH):**
- **Psychas & Verhagen (2020), Sensors 20(11):3012** — iono pseudo-obs, σ calibration, BLUP,
  convergence numbers. https://pmc.ncbi.nlm.nih.gov/articles/PMC7309063/
- **Odijk & Teunissen (2013), "Estimation of DISBs"** — ISB/DISB, phase-vs-code asymmetry.
  https://gnss.curtin.edu.au/wp-content/uploads/sites/21/2016/04/Odijk2013Estimation.pdf
- **Sleewaegen et al. (2012), InsideGNSS** — GLONASS DSP-induced IFB, linear-in-k model.
  https://www.insidegnss.com/auto/mayjune12-Sleewaegen.pdf
- **Montenbruck et al. (2012), GPS Solut. 16:303-313** — GPS IIF L5 IFCB.
  https://doi.org/10.1007/s10291-011-0232-x
- **FF-RT implementation, PMC4969832**; **bootstrapped success rate, PMC6164463**.

**Secondary / paper-level (MEDIUM — verify per §7):**
- Ge et al. 2008 (10.1007/s00190-007-0208-3); Laurichesse & Mercier 2009; Collins et al. 2010;
  Teunissen & Khodabandeh 2015 (10.1007/s00190-014-0771-3); Khodabandeh & Teunissen 2015;
  Zhang et al. 2022 (10.1186/s43020-022-00064-4); Odijk et al. 2016 (10.1007/s00190-015-0854-9);
  Geng et al. 2010; Wanninger 2012; MDPI RS 15(9):2252 (2023); GPS Solut. 2016 (QZSS ISB);
  NAVIGATION 68(4):759 (2021, open-format comparison); Zumberge et al. 1997 (10.1029/96JB03860).

**Tertiary (cross-check only):** ESA Navipedia (Ambiguity Fixing; Melbourne-Wübbena; Time References);
IGS RTS formats page; QZSS CLAS overview; Wübbena et al. 2005 (PPP-RTK origin).

**RTKLIB source (this checkout — PRIMARY, read this session):** `src/ppp.c`, `src/ppp_ar.c`,
`src/lambda.c`, `src/rtkpos.c`, `src/rtcm3.c`, `src/ephemeris.c`, `src/preceph.c`, `src/postpos.c`,
`src/rtksvr.c`, `src/rtklib.h`/`rtklib_types.h`, `src/rcv/ublox.c`.
