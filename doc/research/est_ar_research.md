# Uncombined / est-mode PPP-AR: does Fork B's recipe exist, does the CODE OSB fit, and what is the minimal correct fix?

**Context.** Fork B = integer AR directly on RTKLIB's per-frequency undifferenced phase-bias states `IB(s,f)` under `IONOOPT_EST` (uncombined, free per-epoch slant ionosphere), via between-satellite single-difference (SD) against a reference satellite, then LAMBDA. Phase OSBs from CODE `COD0OPSRAP` Bias-SINEX are applied to carrier phase before the filter. Real data (WTZR 2023-06-01): SD ambiguities do **not** approach integers — fracRMS ~0.29 (uniform-random ≈ 0.289), ratio ~1.0–1.2, **zero fixes/day**; `QbDiagMean` (variance) converges but fracRMS does not fall.

**Source legend.** [SPEC] = SINEX_BIAS v1.00 format spec, read directly. [PAPER] = peer-reviewed. [REPO] = software/docs. [TK] = training-knowledge, not re-verified. [UNCONF] = named/on-topic but full text not accessed this session.

---

## 1. Is Fork B a real recipe? — **No, not as built. It is a known-degenerate parameterization.**

**Verdict: HIGH confidence.** No surveyed mature implementation recovers integers by applying a satellite phase OSB to raw per-frequency phase and then LAMBDA-fixing the raw (or raw-SD) per-frequency ambiguity directly. Every one of them inserts an ionosphere-eliminating recombination — wide-lane/narrow-lane (WL/NL) cascade — or folds the bias into a decoupled/integer clock:

- **PRIDE-PPPAR** (Wuhan/Geng): float filter is uncombined per-frequency, but AR is the classic **MW wide-lane first → narrow-lane second** cascade; "all-frequency" means flexible *choice of IF-combination pair*, not raw-frequency fixing. It ingests CODE `COM*.BIA` directly — but through the WL/NL path. [REPO: PRIDE-PPPAR wiki Fundamentals/Technical-Aspects; PAPER: Geng et al. 2019 GPS Sol 10.1007/s10291-019-0888-1]
- **GREAT-PVT** (Wuhan): AR config block (EWL/WL/NL cascade + UPD mode) is **identical for both its IF and uncombined (UC) PPP modes** — i.e. even in UC mode it fixes via WL→NL cascade, not direct per-frequency LAMBDA. Its own UC functional-model derivation (Eq. 14/17) shows the raw per-frequency ambiguity retains an uncorrected pseudorange-hardware/IFB cross-term for any frequency outside the clock's defining pair — a *structural* non-integer residual beyond phase-OSB correction, needing a separate IFB parameter. [REPO: GREAT-PVT manual §B.2–B.3]
- **CSRS-PPP** (NRCan, Banville et al. 2021): a *production* uncombined PPP-AR system with an explicitly estimated per-satellite slant ionosphere — structurally the same est-mode family as Fork B — **but it does not layer a Bias-SINEX OSB on a standard clock.** It uses the **decoupled-clock / "integer clock" model** (Collins et al. 2010) + DCB, i.e. the phase bias is folded *into* per-frequency satellite clocks. Even so, "successful ambiguity validation requires multiple epochs" because code-only ionosphere estimation is noisy. [PAPER: NAVIGATION 68(2):433]
- **Geng et al. 2022** (J Geod, all-frequency OSB): states directly that code/phase OSBs **cannot** be applied to make raw per-frequency ambiguities integer-recoverable — they are rank-deficient with clocks, ionosphere, and ambiguities; users must still form WL (MW) and IF/NL combinations after OSB correction, and the paper's fractional-ambiguity checks are done in the WL/NL domain, never on raw per-frequency ambiguities. [UNCONF full text; consistent across two independent briefs]

**The one apparent counter-example is thin.** Zhao et al. 2020 (Remote Sensing 12(14):2310) claims direct LAMBDA fixing of undifferenced/uncombined per-frequency GPS ambiguities using self-generated SCB/SPB/IFCB biases. But it (a) self-generates biases from 142 MGEX stations rather than consuming an operational AC product, (b) never explains how single-receiver rank deficiency/datum is handled, (c) reports **no fix-rate or fractional-ambiguity statistics** — only TTFF. It is the least-corroborated source in the survey and cannot be read as validating Fork B's recipe with an operational product. [PAPER, UNCONF full text — low confidence]

**Conclusion:** Fork B's *bias-correction domain* is legitimate (OSB is designed to be subtracted from the raw signal — see §2), but its *ambiguity-resolution step* — direct LAMBDA on raw per-frequency SD ambiguities against a free-floating ionosphere — is precisely the case estimability theory flags as rank-deficient. The missing ingredient is architectural (a WL/NL step or an ionosphere constraint), not a different bias file.

---

## 2. Does the CODE OSB product fit? — **Right family, but not sufficient by itself for direct raw AR.**

**Two things are simultaneously true, and both are spec-grounded (HIGH confidence).**

**(a) OSB is genuinely a raw-signal correction.** SINEX_BIAS v1.00 defines application by direct subtraction from the raw observable: `O'(G,C1) = O(G,C1C) − B_O(G,C1C)` (spec Eq. 14). So applying phase OSB to raw L1/L2 before the filter is the *intended* domain — Fork B is not misusing the product at the correction step. CODE's clock is the standard **Common Clock (CC)** family (single IF-code-consistent IGS clock) + a separate OSB layer — the same SP3/CLK every RTKLIB PPP user already consumes. This is *not* a mismatch of clock family. [SPEC; PAPER: Schaer et al. 2021 J Geod 95:81, 10.1007/s00190-021-01521-9]

**(b) But OSB carries no information that can break the iono/ambiguity degeneracy.** The spec proves individual OSBs are **not independently estimable**; only two combinations are — ISB (ionosphere-free, from clock analysis) and DSB (geometry-free, from ionosphere analysis). Per-frequency OSB is *recovered by inverting a 2×2 system* built from ISB+DSB (spec Eqs. 11–13), and is only **"pseudo-absolute"**: `B = B_O + ΔB`, ΔB an arbitrary datum offset fixed by the AC's chosen reference observables. Real CODE headers reproduced in the spec declare `DETERMINATION_METHOD = COMBINED_ANALYSIS` and `SATELLITE_CLOCK_REFERENCE_OBSERVABLES = "G C1W C2W" / "R C1P C2P"`. The spec warns: *"the selection of the reference observables is absolutely essential"* and OSB-corrected observations are only consistent with a clock product sharing the same convention. [SPEC — verbatim]

So the CODE OSB is a **reparameterization of an IF-clock + geometry-free-ionosphere decomposition**, re-expressed per observable for flexible frequency handling. It supplies exactly the information already in the IF+geometry-free decomposition — nothing that independently constrains a *free-running per-epoch slant ionosphere* in an autonomous single-receiver filter.

**Is product-method mismatch a plausible cause of zero fixes, separate from iono coupling?** Partly, and it is the best explanation for one specific anomaly:

- For the **raw per-frequency SD ambiguities**, the dominant cause is the structural iono-ambiguity rank deficiency (§4), *not* the product — no satellite-only bias product (OSB, IF-FCB, or otherwise) can fix that alone.
- For the **MW/wide-lane cross-check getting *worse* with OSB** (fracRMS 0.249 without OSB → 0.306/0.326/0.316 with phase/code/both), a datum/convention issue in how the two per-frequency OSBs combine into the WL bias (the DSB relation, spec Eq. 10/12b) — or C2W multipath dominance at WTZR — is the leading suspect. WL is exactly the combination the theory says should be iono-free and behave *best*; its degradation is the strongest hint of a genuine product-consistency issue and should be treated as a **separate** problem from the raw-SD rank deficiency.

**Important caveat: no source experimentally validates CODE's `COD0OPSRAP` for uncombined-model AR of any kind.** PRIDE-PPPAR's *paired/recommended* product is WUM (`WUM0MGXRAP`); the 2025 multi-product comparison (Adv. Space Res. S0273117725005071) tested WUM-rapid/final and CNES-gbm but **not** CODE, citing only hearsay that CODE "performed best." So CODE OSB is architecturally suitable but empirically unproven for this mode.

---

## 3. Minimal correct recipe — ranked by evidence × effort

The est-mode uncombined model has a formal rank deficiency of size `(n−1)(m−1)` between slant-ionosphere and per-frequency ambiguity parameters (Zhang et al. 2022, Sat. Nav. 3:3). Satellite phase bias alone does **not** restore rank for freqs 1–2; the literature offers three escapes: (i) an ionosphere-eliminating recombination (WL/NL), (ii) an ionosphere constraint (weighted/fixed), or (iii) a third frequency. Between-satellite SD (Fork B already does this) only removes *receiver* clock/hardware delay — it is orthogonal to the iono/ambiguity coupling and does nothing to fix it.

| Rank | Fix | Evidence | Effort | Notes |
|---|---|---|---|---|
| **1** | **WL/NL two-step *inside the filter's own SD state space*** — form SD MW wide-lane from the SD float ambiguities (OSB-corrected code+phase), fix WL (λ≈86 cm, iono-free & geometry-free); reconstruct NL/IF ambiguity from float + fixed-WL, fix that; recover per-frequency integers algebraically from fixed WL+NL. | **Strongest.** This is literally what PRIDE-PPPAR, GREAT-PVT, CSRS-PPP, Geng 2022, and the 2025 comparison study all do. [multiple PAPER+REPO] | **Medium–High.** Restructure ambiguity parameterization in `ppp.c`; stock RTKLIB-EX has *no* WL/NL AR path (MW is used only for cycle-slip detection, ~lines 394/495/738; `ppp_ar.c` is a no-op stub). | Keeps CODE OSB and the uncombined float filter as-is. Highest-payoff structural fix. |
| **2** | **Add an ionosphere constraint** (ionosphere-weighted or -fixed pseudo-observation on the slant-iono states). | Strong theory: converts the rank-deficient ionosphere-float model to an estimable one; enables raw per-frequency AR directly. [PAPER: Zhang 2021 GPS Sol 10.1007/s10291-021-01169-0; Zhang 2022] | **Medium** in code, but **High** operationally — needs a *regional/local* reference-network ionosphere stream. Global CODE rapid products don't supply this. | Only helps with an added correction source; a different architecture than anything in Fork B. |
| **3** | **Switch to a decoupled/integer-clock product** (Collins 2010 lineage) — bias folded into per-frequency clocks; most directly enables literal raw per-frequency AR. | Strong — this is exactly how CSRS-PPP (a production est-mode AR system) works. [PAPER: Banville 2021] | **High + blocked.** Decoupled/integer-clock products are a non-standard family; unclear any AC publishes one openly (NRCan generates internally). Would also replace RTKLIB's SP3/CLK handling. | Biggest architectural change; product availability is the gating unknown. |
| **4** | **Swap OSB product** (WUM `WUM0MGXRAP` or CNES-gbm) with the *same* Fork B recipe. | Weak for the core problem — doesn't touch the rank deficiency. Only worth it to isolate whether CODE's product is a *contributing* factor to the MW anomaly. | **Low.** Just a different bias file. | Useful controlled A/B *after* the WL/NL fix, not a standalone cure. |

**Bottom line on §3:** the smallest change that would actually fix ambiguities is **candidate 1 (WL/NL two-step)**. It is a re-architecture of the AR step, not a product swap or a one-line iono tweak — but it is the one with overwhelming convergent evidence.

---

## 4. Root-cause verdict for fracRMS ≈ 0.29

**Primary cause: structural ionosphere–ambiguity rank deficiency of the est-mode (ionosphere-float) uncombined model. — HIGH confidence.**

The symptom is textbook: near-uniform fracRMS (~0.29 vs uniform-random 0.289) with `QbDiagMean` (variance) shrinking but fracRMS *not* falling. That is the signature of a **structurally ill-posed / rank-deficient** estimation, not slow convergence — the covariance collapses around a value that is *not* the true integer because the observation geometry genuinely cannot separate the free per-epoch slant ionosphere from the per-frequency ambiguity. LAMBDA is being asked to decorrelate a null space, not noise. Estimability theory (Zhang 2022; Geng 2022; Villiger/Geng) says a satellite-only bias product — OSB, IF-FCB, or otherwise — *cannot* resolve this by itself. This also matches the existing project lesson `memory/feedback_dd_vs_sd.md` ("SD params + DD obs = rank deficiency") — a sibling instance of the same failure mode.

**Secondary/compounding cause: a possible product-datum/convention inconsistency — MEDIUM confidence, and probably a *separate* problem.** The MW-wide-lane getting *worse* with OSB is unexpected under the rank-deficiency story alone (WL should be iono-free and behave best). That points to a convention issue in the per-frequency-OSB→WL-bias combination (DSB relation), a reference-observable/sign mismatch (spec: "reference observables absolutely essential"), a receiver-DCB/ISB datum gap not separately estimated, or plain C2W multipath dominance at WTZR. No source this session explained it.

**What is *not* the cause:** the OSB correction domain itself (OSB is designed for raw signals) and the clock family (CODE CC clock is standard). Those are correct.

**Confidence summary:** *iono coupling is the dominant root cause of the raw-SD non-integer result (HIGH); a distinct product/datum inconsistency likely drives the MW anomaly (MEDIUM); a residual filter-parameterization bug cannot be fully excluded (LOW)* — RTKLIB's implicit S-basis for its IB/iono states was never checked against S-system theory, so it is unconfirmed whether the system is even full-rank before AR is attempted.

---

## 5. Recommendation — what to do next

**Do this in order. The first two steps are cheap and decide everything after.**

**Step A — Local diagnostic, no web research, ~1 hr (do first).**
Grep the actual downloaded `COD0OPSRAP` `.BIA` header for `DETERMINATION_METHOD` and `SATELLITE_CLOCK_REFERENCE_OBSERVABLES` (plain-text SINEX keywords). Expect `COMBINED` + a `C1W/C2W` reference pair. This confirms the OSB is IF/geometry-free-combination-consistent by construction and tells you the datum convention Fork B must match.

**Step B — Isolate the mechanism, ~half day (do second).**
From your OSB-corrected raw L1/L2, explicitly form CODE's own **IF combination** and **geometry-free/DSB (wide-lane-shaped)** combination and check *their* fracRMS/integer-closeness **in isolation from the Kalman filter's ionosphere estimate**. Decision rule:
- If IF and WL combos round near-integer but the `IONOOPT_EST` SD-IB states do not → confirms the problem is the **ionosphere-datum / rank-deficiency** coupling (candidate fix = §3 #1), not LAMBDA/SD mechanics or a bad file.
- If even the WL/GF combo is non-integer → the product-convention/DSB issue (§4 secondary) is real and must be fixed *before* any AR can work — debug OSB sign/order applied to C1W/C2W and whether **code** OSB was applied consistently to the pseudoranges feeding MW.

**Step C — Implement the fix (§3 candidate 1: WL/NL two-step).** This is the real work: restructure the AR step in `ppp.c` to fix SD MW wide-lane first, then NL, then recover per-frequency integers — instead of direct LAMBDA on raw SD-IB. Keep CODE OSB and the uncombined float filter unchanged. Medium–high effort; highest evidence; the only path all mature systems actually use.

**Step D — Only if C still underperforms:** controlled substitution of WUM `WUM0MGXRAP` or CNES-gbm OSB (cheap, low effort) to rule the CODE product in/out; and, further out, consider a decoupled-clock product (CSRS-PPP route) if you want direct raw AR without WL/NL — but that is a large architecture change gated by product availability.

**Cost summary:** A ≈ 1 hr; B ≈ half day (both pure diagnostics, no new code paths). C ≈ the substantive engineering (new AR parameterization). D ≈ low (file swap) to high (new product family). Do **not** pour more effort into tuning the direct-per-frequency LAMBDA path or chasing the noisy obs-level MW check — the evidence says that path is structurally degenerate, and PPP-AR being hard here is consistent with the wider ecosystem (even rtklibexplorer's own PPPLib fork ships PPP-AR as explicitly unfinished).

**One honest caveat:** the strongest on-point papers (Banville 2020 J Geod 10.1007/s00190-019-01335-w; Geng 2022 10.1007/s00190-022-01602-3, CC-BY) could not be read in full this session. If you have library access, read those two — they are the cross-AC IRC/DC/OSB interoperability and all-frequency-OSB references that would harden every "MEDIUM/UNCONF" verdict above.
