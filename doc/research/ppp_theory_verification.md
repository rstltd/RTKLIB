# §7 Verification Addendum — Primary-Source Pass on the Modern-PPP Theory Map

**Companion to:** `doc/research/ppp_theory_map.md` §7 (19 open items)
**Method:** 6 agents, T4 sourcing contract (verbatim primary quote required to upgrade). Repo `/home/alenchu/vs_code/RTKLIB`, HEAD `d574080`, working tree clean.
**Headline:** Of the 18 §7 items actually reached this pass, **14 CONFIRMED-primary, 3 PARTIAL, 1 STILL-UNCONFIRMED**. Item 18 was not addressed by any agent (see §4). Two training-reconstructions turned out **wrong** (item 13 wrong equation shape; item 3 equation not in the cited sources) — flagged loudly in §2. No wrong *sign* was found; every SUBTRACT/offset convention checked (B2b, BDT, GGTO, URA) held up.

---

## 1. Scoreboard

| # | Item | Verdict | One-line result | Source (that worked) |
|---|------|---------|-----------------|----------------------|
| 1 | Zhang, Hou, Zha & Liu 2022 S-system | **CONFIRMED-primary** | 8 rank-deficiency groups (n,m,n,m,1,f−2,fm,fn), Eqs.8–15; estimable ambiguity is DD; phase-only 9th deficiency + GPS 60/77 example all verbatim | r.jina.ai proxy of SpringerOpen OA (peer-reviewed-fulltext) |
| 2 | Odijk et al. 2016 J.Geod estimability | **PARTIAL** | Abstract + Appendices 1–3 (Eq.49–59, CC-R/CC-S S-basis, Eq.52 block sizes) recovered; core Sec.1–5 / Eq.1–48 unreachable | Wayback snapshot of Springer page (archive) |
| 3a | Ge et al. 2008 WL/NL FCB | **STILL-UNCONFIRMED** | Genuinely paywalled, zero OA deposit; **DOI correction**: cited DOI is the erratum | none reached |
| 3b | Laurichesse&Mercier 2009 + Collins et al. 2010 | **CONFIRMED-primary** | Decoupled/integer-recovery clock concept confirmed, **but the map's `dts_phase=dts_code+b_N/f_N` is NOT in either paper** | ppp-wizard.net author PDFs (peer-reviewed-fulltext) |
| 4 | Teunissen & Khodabandeh 2015 review | **CONFIRMED-primary** | "single-receiver user ambiguities are in fact DD" verbatim from abstract; S-basis Eq.8/10 | Curtin repo via Wayback (peer-reviewed-fulltext) |
| 5 | Khodabandeh & Teunissen 2015 corrections precision | **CONFIRMED-primary** | D(ẑ*)=D(ẑ)−D(δ̃) variance propagation (Eq.38), Table 9 closed forms | Curtin repo via Wayback (peer-reviewed-fulltext) |
| 6 | Verhagen et al. 2011 partial-AR subset | **CONFIRMED-primary** | Bootstrap SR Eq.7 matches map exactly; PAR algorithm Eq.11 verbatim | gnss.curtin.edu.au author PDF (peer-reviewed-fulltext) |
| 7 | Galileo HAS phase-bias status | **CONFIRMED-primary** | **Still NOT operational** (Q1-2026 data); Full Service target Q1–Q2 2027 | gsc-europa.eu SDD/Info-Note/Q1-2026 report (primary-icd) |
| 8 | BDS-3 PPP-B2b ICD | **CONFIRMED-primary** | SUBTRACT on all 3 channels, C0-only clock, no phase-bias type — all verbatim | en.beidou.gov.cn ICD via curl+pdftotext (primary-icd) |
| 9 | RTCM 1265-1270 + integrity flags | **PARTIAL** | Flags sii/swl/sdc/dispe/mw semantics verbatim (IGS-SSR); **1265-1270 ratification unconfirmed** (RTCM paywall) | files.igs.org IGS-SSR v1 (official-secondary) |
| 10 | IGS-SSR IM201 VTEC | **CONFIRMED-primary** | "IM201" correct; SH equation exact match; full bit-level layout | files.igs.org igs_ssr_v1.pdf (primary-icd) |
| 11 | IS-QZSS-L6 CLAS atmosphere | **CONFIRMED-primary** | ST8 STEC poly + ST9 grid bit-level; **new ST12 combined message** found | qzss.go.jp L6-007 (primary-icd) |
| 12 | Galileo GGTO + BDT-GPST | **CONFIRMED-primary** | GGTO Eq.23/Table 74 exact; **rides Word Type 10 not 16**; **BDT−14s is derived, not tabulated** | gsc-europa.eu OS-SIS-ICD + beidou via Wayback (primary-icd) |
| 13 | Odijk 2002 iono weighting | **CONFIRMED-primary** | **Distance model is LINEAR (Schaffrin&Bock 1988), NOT Gaussian** as the map assumed | gnss.curtin.edu.au Odi02.pdf (peer-reviewed-fulltext) |
| 14 | CODE_* enum for bias table | **CONFIRMED-primary** | Full enum read; **code_bias_ix[] must key on (sys,code)** — NavIC L1 collides BDS B1C | src/rtklib_const.h, rtcm3.c (code read) |
| 15 | ssr_t integrity-flag storage | **CONFIRMED-primary** | sii/swl/sdc/dispe/mw decoded-then-discarded; no ssr_t field (exhaustive grep) | src/rtcm3.c, rtklib_types.h (code read) |
| 16 | ar_poly_coeffs provenance | **CONFIRMED-primary** | Empirical curve-fit to TU Delft LAMBDA webpage example; no FF-RT calibration | src/rtkpos.c + git log (code read) |
| 17 | Receiver phase-bias vs IC() | **CONFIRMED-primary** | **NOT absorbed by IC()**; separate SD/UPD needed; ppp_ar stub does neither | src/ppp.c (code read) |
| 18 | *(not addressed this pass)* | **NOT REACHED** | No agent covered §7 item 18 | — |
| 19 | MGEX/IGS OSB coverage | **PARTIAL** | BDS-3 B1C/B2a, GAL E6, mod-QZSS covered; **NavIC OSBs absent**; not byte-verified | igs.org/mgex (official-secondary) |

---

## 2. MAP CHANGES — contradictions & material refinements (highest-value section)

### 2a. WRONG — training-reconstructions that the primary source contradicts

**Item 13 (§ iono-weighting / Odijk 2002) — WRONG EQUATION SHAPE.**
- **Map said:** used the Psychas & Verhagen (2020) **Gaussian-decay** distance-weighting form as a stand-in for "the Odijk (2002) model" (PDF was previously unreadable).
- **Primary says:** Odijk's own distance-dependent variance is **LINEAR**: σ_i,user = β·l_xr, with **0.3 ≤ β ≤ 3 mm/km** (Eq.6.60, p.185-186), and Odijk **explicitly attributes it to Schaffrin & Bock (1988)** — it is not his own invention, and it is not Gaussian. His Ch.5 "ionosphere-weighted model" proper is a single **non**-distance-dependent tunable variance σ_i²; distance-dependence only enters Ch.6 as a VRS user-side heuristic.
- **Fix:** In any section describing "the Odijk (2002) ionosphere weighting," correct to *linear-in-distance, β = 0.3–3 mm/km, orig. Schaffrin & Bock (1988)*. Drop the Gaussian form or re-attribute it to Psychas & Verhagen (2020) as a separate, later model.

**Item 3b (§3.A.2 line ~158, decoupled clock) — EQUATION NOT IN CITED SOURCES.**
- **Map said:** "Fold the satellite NL-FCB directly into a second clock product: `dts_phase = dts_code + b_N^s/f_N`," attributed to the Laurichesse/Collins decoupled-clock family.
- **Primary says:** Neither Laurichesse & Mercier (2009) nor Collins et al. (2010) writes this equation. Both estimate phase-clock and code-clock as **two separate network parameters**, not one derived from the other by adding a bias/frequency term. Laurichesse Eq.6: `Q̂_c − D_w = λ_c·δN_1 + h_i − h^j` (NL bias lives *inside* the integer clocks h). Collins Eq.4: fully separate `dt_L3^s` and `dt_P3^s`, rank filled by ambiguity-datum-fixing.
- **Fix:** Either mark `dts_phase = dts_code + b_N/f_N` as an **illustrative/pedagogical simplification** (do not attribute it verbatim to Laurichesse/Collins), or replace with the Eq.4/Eq.6 forms above. The *concept* (second clock product makes NL ambiguity integer-recoverable, model-side correction) is fully confirmed and unchanged.

### 2b. Sourcing / attribution corrections (numbers right, provenance wrong)

**Item 12 (§ time-systems) — two sourcing corrections, numbers correct.**
- Galileo GGTO parameters (A0G/A1G/t0G/WN0G, Eq.23/Table 74) are numerically exact — **but GGTO rides in I/NAV Word Type 10, not Word Type 16** (WT16 = "Reduced CED"). Correct any word-type reference.
- BDS "BDT = GPST − 14 s" is numerically correct (matches RTKLIB `bdt2gpst`/`gpst2bdt`, rtkcmn.c:1958-1960) **but is NOT tabulated in the ICD**; it is derived from epoch definitions + historical leap-seconds. The ICD's *dynamic* offset field (A0GPS/A1GPS, Table 5-19) is explicitly **"Not broadcast temporarily"** in current B1I-3.0 (2019) and B3I-1.0 (2018). Any text implying BDS broadcasts a GGTO-like fine offset is wrong; only the fixed 14 s whole-second constant is usable.

**Item 2 (§2.3) — citation precision.** The exact numbered (1)-(8) 8-group/size enumeration is verbatim only in **Zhang et al. 2022**. Odijk et al. 2016's recovered text organizes the *network* model around a 5-parameter-group CC-R/CC-S S-basis (Eq.52, sizes n−1, m, f(n−1), fm) — related but not identical framing. Attribute the 8-group enumeration to Zhang 2022; credit Odijk 2016 as the foundational S-system method it builds on.

**Item 3a (§ Ge 2008 bibliography) — DOI fix.** The DOI in the map/brief `10.1007/s00190-007-0208-3` resolves to the 1-page **erratum**. The article is `10.1007/s00190-007-0187-4` (pp.389-399).

### 2c. Bit-field / semantic refinements

**Item 9 (§3.C phase-bias flags) — swl is 2-bit, not 1-bit.** The map treats "swl" (Wide-Lane Integer Indicator) structurally like the 1-bit sii. Primary IGS-SSR IDF030 is **2 bits** encoding membership in up to **three** wide-lane groups (00/10/01/11). An AR engine consuming it needs 4-way branching, not boolean. sii/sdc/dispe/mw semantics confirmed verbatim (upgrade from medium-low to high).

**Item 8 (§ B2b decoder note) — ICD-internal inconsistency (not a map error).** MT3 DCB field: 12-bit two's-complement × 0.017 LSB spans ±34.816 m, but the ICD's own table prints range ±35.746 m. Verified in two pdftotext modes — it's in the printed ICD, not OCR. Flag in decoder, do not "fix" silently.

**Item 5 (§ corrections-precision one-liner) — refine, not contradict.** The map's "correction-precision → user-AR-success propagation" is a full **variance-matrix** propagation `D(ẑ*)=D(ẑ)−D(δ̃)` (Eq.38, Table 9), with success-rate/ADOP as a downstream qualitative consequence — not a single closed-form success-rate equation. Reword the one-liner.

**Item 7 (§3.C footnote) — status sharpened.** HAS phase bias now explicitly scoped as an **SL1** product (Info Note Issue 1.1, Dec 2025); Full Service declaration target **Q1–Q2 2027**. Operational answer for §6 L3 sequencing: HAS today feeds only float-PPP-grade orbit/clock/code-bias; HAS-based real-time PPP-AR remains infeasible until ~2027.

### 2d. Design constraints newly surfaced (map took no prior position)

**Item 14 — code_bias_ix[] cannot key on CODE_* alone.** NavIC L1 (msm_sig_irn) reuses the numeric CODE_L1D=56 / CODE_L1P=2 / CODE_L1X=12 values that are simultaneously BDS-3 B1C's codes, and those MSM slots are marked **"tentative … PocketSDR extensions"** in-source (not RTCM-ratified). Any bias table must be keyed by **(sys, code)** pairs.

**Item 17 — receiver phase bias NOT absorbed by IC().** IC() is one scalar clock per *system*, shared code+phase, no per-frequency dimension. The receiver-side phase hardware delay stays folded into every satellite's per-frequency IB(sat,frq) float state (init `bias=L−P+…`, ppp.c). A separate mechanism (between-satellite SD, or external receiver-UPD/OSB) **is** required; between-satellite SD is NOT redundant. `ppp_ar.c` stub does neither today.

**Item 16 — ar_poly_coeffs has no statistical calibration to preserve.** Confirmed empirical curve-fit to a TU Delft LAMBDA-toolbox *webpage example* (comment rtkpos.c:94-95), revised once (commit fc55667) to also depend on nominal AR ratio, valid only up to 50 sat-pairs. No FF-RT / Verhagen / Teunissen calibration anywhere → FF-RT would be a clean drop-in upgrade.

---

## 3. Newly-pinned facts (upgraded to high-confidence with quotes)

- **Zhang 2022 estimable ambiguity is DD** (Eq.15): `z̃_{r,j}^s = (z_{r,j}^s − z_{1,j}^s) − (z_{r,j}^1 − z_{1,j}^1)`; phase-only 2nd-freq form `(λ_2/60)(60 z̃_{r,2}^s − 77 z̃_{r,1}^s)`.
- **Teunissen & Khodabandeh 2015 abstract (verbatim):** "PPP-RTK is a relative-technique for which the 'single-receiver user' integer ambiguities are in fact double differenced ambiguities." Estimable `z̃_r^ps = z_r^ps − z_1^ps` (Eq.10).
- **Verhagen 2011 bootstrap SR (Eq.7):** `P_{s,B}=∏(2Φ(1/(2σ_{â_i|I}))−1)` — exact match to map; PAR criterion Eq.11 `∏_{i=k}^n(2Φ(1/(2σ_{ẑ_i|I}))−1) ≥ P_0`, start from most-precise decorrelated ambiguity.
- **B2b conventions (verbatim ICD):** DCB `l' = l − DCB` (7-1); orbit `X = X_bc − ΔX` (7-5); clock `t = t_bc − C0/c` (7-10); MT4 clock is C0-only (15 bit, 0.0016, ±26.2128 m); message-type space 1-63 has no phase-bias type.
- **IGS-SSR IM201 VTEC:** MT4076, IDF002=201, GNSS-independent; SH eq. exact; header 83 bits; coeffs int16 ±163.835 TECU / 0.005; N=degree+1, M=order+1; count `(N+1)(N+1)−(N−M)(N−M+1)`.
- **URA (IGS-SSR IDF034):** 6-bit = 3-bit CLASS + 3-bit VALUE, `URA[mm] ≤ 3^CLASS·(1+VALUE/4)−1`; 000000 undefined, 111111 = >5466.5 mm. **RTKLIB var_urassr() (ephemeris.c:132) is a bit-exact match** (`pow(3,(ura>>3)&7)*(1+(ura&7)/4)-1)*1e-3`, ura≥63→5.4665 m).
- **IGS-SSR phase-bias flags:** IDF029 sii 1-bit; IDF030 swl **2-bit** (3 WL groups); IDF031 sdc uint4 (rolls 15→0); IDF032 dispe 1-bit; IDF033 mw 1-bit; IDF028 phase bias int20 ±52.4287 m / 0.0001, re-init to ±0.5 cycle on overflow.
- **Galileo GGTO (Eq.23):** `dt = t_Gal − t_GPS = A0G + A1G[TOW − t0G + 604800(WN−WN0G)]`; A0G 16b/2^-35, A1G 12b/2^-51, t0G 8b/3600, WN0G 6b/1; **Word Type 10**.
- **IS-QZSS-L6 CLAS:** ST8 STEC poly (4 types, C00 14b/0.05 TECU … C02/C20 8b/0.005); ST9 grid tropo hydro 9b/0.004 m nominal 2.3 m, wet 8b/0.004 nominal 0.252 m, Neill MF; STEC residual adaptive int7/int16 @0.04 TECU; **new combined ST12** exists (current content pp.41-52 of L6-007, not pp.53/55).
- **Code reads (repo HEAD d574080):** ssr_t (rtklib_types.h:257-274) has no integrity-flag field; ar_poly_coeffs 3×5 fit valid ≤50 pairs; IC()=NP+s per-system clock, IB=NR+MAXSAT·f+s−1 per-sat-per-freq. Header split into rtklib_const.h / rtklib_types.h / rtklib_api.h.

---

## 4. Still-unconfirmed / not reached (residual risk before writing the impl spec)

| Item | What's missing | Why (channels that stayed blocked) | Risk |
|------|----------------|-------------------------------------|------|
| 3a | **Ge et al. 2008** full text — WL/NL FCB two-step in the author's own notation | Genuinely paywalled; **Unpaywall/GFZ/ProQuest/RG/S2 all confirm no OA deposit exists**. The map's WL/NL equations remain training-reconstructions cross-checked only vs RTKLIB `mwmeas()` + Navipedia | LOW-MED — concept solid elsewhere (Laurichesse Eq.6), only Ge's exact notation unverified |
| 2 | **Odijk 2016 core Sec.1–5 / Eq.1–48** (original numbered rank-deficiency enumeration) | Springer paywall; only Abstract+Appendices sit outside the gated component (recovered via Wayback). Curtin espace behind AWS-WAF; no repo copy indexed | LOW — enumeration independently confirmed in Zhang 2022 |
| 9 | **RTCM 10403.4 + Amendment 1 ratification of MT 1265-1270** | Hard $340 paywall (rtcm.myshopify.com); only marketing blurb public. 4 channels converge on "draft/proposal-phase, used via MT4076 + community impls" | MED — do NOT upgrade 1265-1270 to "ratified"; numbers are real (traced to a "RTCM c10403.3" committee-draft citation), status is not |
| 9c | **RTCM 10403.3 DF389** (native URA) byte text | Same RTCM paywall | LOW — IDF034 (1:1 wrapper) read; RTKLIB decodes both paths with shared code, so DF389≈IDF034 by design |
| 19 | **Raw Bias-SINEX .BIA byte header** | CDDIS Earthdata OAuth-gated; AIUB anonymous mirror listing returned empty for guessed 2026 filenames | LOW — IGS product-description page is authoritative for *coverage*; **NavIC OSBs confirmed ABSENT** — flag false any assumption they exist |
| **18** | **§7 item 18 was not addressed by any agent this pass** | Not assigned/covered | UNKNOWN — re-scope and run before the impl spec if item 18 gates anything |

No fabricated equation, bit-width, scale, sign, or message number was introduced. Every "could not verify" above is recorded as such rather than guessed.

---

## 5. Consolidated working primary-source URLs (for future direct access)

**Technique for blocked PDFs (repeatable):** `curl -sL <url>` + local `pdftotext -layout` bypasses WebFetch's binary-PDF blindness. `curl https://r.jina.ai/<url>` returns raw article text for cookie-gated Springer OA HTML (works when the WebFetch tool's own proxy call returns only a summary). Wayback `web.archive.org/web/<ts>/<url>` recovered Curtin repo PDFs and beidou.gov.cn ICDs when live hosts refused.

**Peer-reviewed full text (open / self-archived):**
- Zhang 2022 (OA): `https://link.springer.com/article/10.1186/s43020-022-00064-4` (via r.jina.ai)
- Laurichesse & Mercier 2009: `http://www.ppp-wizard.net/Articles/laurichesse_navigation.pdf`
- Collins et al. 2010: `http://www.ppp-wizard.net/Articles/Collins_Navigation_v57n2_2010_accepted.pdf`
- Teunissen & Khodabandeh 2015: `https://espace.curtin.edu.au/bitstream/handle/20.500.11937/37094/231902_231902.pdf` (via Wayback `web/20241205150912/`)
- Khodabandeh & Teunissen 2015: `https://espace.curtin.edu.au/bitstream/handle/20.500.11937/12229/231903_231903.pdf` (via Wayback `web/20241205152758/`)
- Verhagen et al. 2011: `https://gnss.curtin.edu.au/wp-content/uploads/sites/21/2016/04/Verhagen2011GNSS.pdf`
- Odijk 2002 thesis: `https://gnss.curtin.edu.au/wp-content/uploads/sites/21/2016/04/Odi02.pdf`

**Primary ICDs / official format specs:**
- IGS-SSR v1.00 (VTEC IM201, URA IDF034, phase-bias flags): `https://files.igs.org/pub/data/format/igs_ssr_v1.pdf`
- BDS-3 PPP-B2b ICD: `http://en.beidou.gov.cn/SYSTEMS/ICD/202008/P020200803538771492778.pdf`
- Galileo OS SIS ICD v2.1 (GGTO): `https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf`
- Galileo HAS SDD v1.0 / Info Note 1.1 / Q1-2026 report: `https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-HAS-SDD_v1.0.pdf` · `.../Galileo_HAS_Info_Note.pdf` · `.../Galileo-HAS-Quarterly-Performance_Report-Q1-2026.pdf`
- IS-QZSS-L6-007 (CLAS): `https://qzss.go.jp/en/technical/download/pdf/ps-is-qzss/is-qzss-l6-007.pdf`
- BeiDou B1I-3.0 / B3I-1.0 (BDT epoch) via Wayback: `web/20260605225406/http://en.beidou.gov.cn/SYSTEMS/ICD/201902/P020190227702348791891.pdf` · `web/20251126075715/http://en.beidou.gov.cn/SYSTEMS/ICD/201806/P020180608516798097666.pdf`

**Official secondary:**
- IGS MGEX data-products / OSB coverage: `https://igs.org/mgex/data-products/`

**Local extracted text (this session's scratchpad):** `primary_sources/` (igs_ssr_v1, HAS_*), `b2b.txt`, `igs_ssr_v1.txt`, `l6.txt`.
