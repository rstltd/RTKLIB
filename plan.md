# RTKLIB Factor Graph Optimization (FGO) Extension
## Architecture Design Document / Technical Plan

| 項目 | 內容 |
|---|---|
| **文件類型** | RFC / Architecture Design Document (ADD) |
| **版本** | 0.1 (Draft for Review) |
| **日期** | 2026-08-20 |
| **狀態** | Draft — 待技術評審 |
| **目標程式庫** | RTKLIB-EX (demo5 lineage, RTKLIB 2.4.3 b34 base)，`rstltd/RTKLIB` fork |
| **程式碼基準** | branch `chore/sync-upstream-20260820` @ `512ed8ac`（已含上游 RTKLIB-EX 2.5.1 合併 `a8062992`）。見下方「行號基準」說明 |
| **授權基線** | RTKLIB: BSD-2-Clause；GTSAM: BSD-3-Clause（相容，見 §12.4） |
| **適用範圍** | GNSS Near Real-Time (NRT) 變形監測 — 橋梁、邊坡、建物沉陷、結構健康監測 |

### 行號基準（重要）

本文件引用的所有 `檔案:行號` 皆已對照 branch `chore/sync-upstream-20260820` 的 commit **`512ed8ac`**（本文件加入前的分支頂端）逐項驗證。該分支已包含 `a8062992`「Merge upstream rtklibexplorer/RTKLIB main (2.5.1) into rstltd main」。

已驗證正確的檔案（行號可直接使用）：

| 檔案 | 行數 | 抽查驗證 |
|---|---|---|
| `src/rtkpos.c` | 2,549 | `varerr`:406、`zdres`:1049、`ddcov`:1128、`constbl`:1145、`ddres`:1240、`manage_amb_LAMBDA`:1909、`relpos`:2068、`rtkpos`:2438 全部相符 |
| `src/ppp.c` | 1,318 | `ppp_res`:974、`pppos`:1226 相符 |
| `src/pntpos.c` | 715 | `varerr`:51、`rescode`:277 相符 |
| `src/rtkcmn.c` | 4,301 | `filter`:1479 相符 |
| `src/lambda.c` | 267 | `lambda`:180 相符 |
| `src/rtklib_const.h` | 504 | `NFREQ`:83、`MAXSAT`:172、`MAXOBS`:177、`PMODE_*`:314、`SOLQ_*`:332 相符 |
| `src/rtklib_api.h` | 554 | `chisqr[]`:8 相符 |
| `src/options.c` | 578 | `sysopts[]`:67 相符 |
| `src/rtklib_types.h` | 824 | 已於本版重新校正（見下） |

**已於本版修正的行號**：`src/rtklib_types.h` 因上游合併新增內容，`obsd_t`(:13) 之後的所有結構定義位移約 +12 行。本文件所有相關引用（`prcopt_t`、`rtk_t`、`ssat_t`、`sol_t`、`nav_t`、`solopt_t`、`ambc_t`、`snrmask_t`、`opt_t`、`void *solstat`）已全部更新為現行位置。

**使用提醒**：

1. 行號僅供快速定位，**閱讀時請以函式／結構名稱為準**。上游持續合併會使行號再次位移，但函式名稱與職責是穩定的。
2. §4.2.1「要點 1」與 §6.1 的 M3（凍結配對模式）所依據的行為——`ddres()` 會動態選擇最小變異數的參考衛星（`refvar`/`minvar` 邏輯）——**已確認存在於現行分支的已提交程式碼中**，M3 的設計前提成立。
3. `src/rtklib.h` 現為 91 行的純門面檔（`:51-52` 為 `extern "C"` guard，`:85-87` 引入三個拆分檔），§2.1 描述的標頭拆分結構已完成。

---

## 目錄

- [1. Executive Summary](#1-executive-summary)
- [2. Baseline Analysis — 現況程式庫盤點](#2-baseline-analysis--現況程式庫盤點)
- [3. System Architecture](#3-system-architecture)
- [4. Detailed Design](#4-detailed-design)
- [5. Error Model Design](#5-error-model-design)
- [6. RTKLIB Modification Plan](#6-rtklib-modification-plan)
- [7. 新增模組設計 src/fgo/](#7-新增模組設計-srcfgo)
- [8. Solver 評估](#8-solver-評估)
- [9. GNSS Monitoring 應用](#9-gnss-monitoring-應用)
- [10. AI Insight Integration](#10-ai-insight-integration)
- [11. Implementation Roadmap](#11-implementation-roadmap)
- [12. Risks and Mitigations](#12-risks-and-mitigations)
- [13. Future Extensions](#13-future-extensions)
- [14. Open Questions — 待決議事項](#14-open-questions--待決議事項)
- [Appendix A — 關鍵原始碼索引](#appendix-a--關鍵原始碼索引)
- [Appendix B — 建議新增設定項全表](#appendix-b--建議新增設定項全表)
- [Appendix C — 名詞與符號對照](#appendix-c--名詞與符號對照)

---

# 1. Executive Summary

## 1.1 背景

現行 GNSS NRT 監測系統以 RTKLIB 為 DGNSS / RTK 解算核心，狀態估計採用 Extended Kalman Filter（`src/rtkpos.c` 的 `relpos()` → `rtkcmn.c` 的 `filter()`）。EKF 在即時性與計算成本上表現優異，但對於**長期變形監測**這一應用領域，存在三項結構性限制：

1. **單次線性化（one-shot linearization）**：`filter()` 在每個 epoch 只做一次量測更新，線性化點固定為預測狀態。在多路徑或大殘差情境下，線性化誤差無法透過迭代修正。RTKLIB 雖有 `opt->niter`（`pos2-niter`）可重複迭代，但那是重複整個 EKF 更新，並非真正的非線性最佳化。
2. **不可回溯（no re-linearization of the past）**：一旦某 epoch 的量測被吸收進 `rtk->P`，該 epoch 的資訊即被邊緣化。事後才判定為 outlier 的觀測量（例如經過數個 epoch 才確認的 cycle slip）無法被移除或重新加權。
3. **硬性離群值處理**：現行機制為門檻式硬拒絕（`opt->maxinno[]`，見 `rtkpos.c` 的 `valpos()`）與 AR filtering。這是 0/1 的權重函數，對「輕度污染」的觀測量缺乏中間地帶。

Factor Graph Optimization（FGO）在同一份觀測資料上做**批次或滑動視窗的非線性最小平方**，天然支援重複線性化、robust kernel、以及多感測器異質約束的加入。這正是變形監測（mm 等級、長時序、多測站、未來需融合 IMU/傾斜儀/InSAR）所需要的估計框架。

## 1.2 核心結論

> **推薦架構：Architecture A′ — Residual Callback Hybrid（架構 A 的可重複線性化精煉版），並以 Phase 2 → Phase 3 分階段落地。**

三項關鍵判斷：

**（一）架構 A 與 B 皆不宜直接採用，正確答案是 A 的精煉版 A′。**

- 需求文件描述的**架構 A**（RTKLIB → Residual → GTSAM → FGO Solution）若字面實作，會匯出「在固定線性化點 `x0` 上計算的殘差 `v` 與 Jacobian `H`」。GTSAM 拿到的是一組**已經線性化的 factor**，本質上退化為一個 sparse linear least squares／資訊濾波器。FGO 相對於 EKF 最重要的優勢——**重複線性化**——完全喪失。
- **架構 B**（RTKLIB Native FGO，以 C 自行實作圖最佳化）需要在 C99 中重建 sparse Cholesky、變數消去順序、Bayes tree、robust kernel 與自動微分基礎設施。這是 GTSAM/Ceres 累積十餘年的工程量，重寫的技術風險與維護成本遠高於收益，且與「盡可能重用 RTKLIB 現有觀測模型、避免重寫」的專案目標背道而馳。
- **架構 A′**：將 RTKLIB 的 `zdres()` / `ddres()` / `rescode()` / `ppp_res()` 重構為**可重入的純函式（re-entrant pure function）**，透過一層薄 C ABI（`src/fgo/rtklib_fgo_api.h`）暴露；GTSAM factor 的 `evaluateError()` 在每次線性化時**回呼**這些函式，以當前估計值 `X` 為輸入重新計算殘差與 Jacobian。RTKLIB 仍是觀測模型的**唯一真相來源（single source of truth）**，GTSAM 只負責圖結構與最佳化。既保有重複線性化，又完全不重寫 RTCM decoder / RINEX parser / ephemeris / 大氣模型 / 座標轉換。

**（二）本 fork 的三項既有條件對此架構特別有利。**

| 條件 | 證據 | 意義 |
|---|---|---|
| 頂層 CMake 已啟用 C++ | `CMakeLists.txt:12` — `project(rtklib LANGUAGES C CXX VERSION 2.4.3)` | 加入 `src/fgo/*.cpp` 不需改動 build system 的語言設定 |
| `rtklib.h` 已具 C++ linkage guard | `rtklib.h:51-52` — `#ifdef __cplusplus` / `extern "C" {` | C++ TU 可直接 `#include "rtklib.h"`，無須額外 wrapper |
| `rtk_t` 已有 opaque context 指標先例 | `rtklib_types.h:672` — `void *solstat;  /* statout_t*, bound by rtkinit */` | 加入 `void *fgo;` 遵循本 fork 既有慣例，ABI 影響可控、且審查者已熟悉此模式 |

**（三）Double-Difference 的相關性協方差與 robust kernel 存在根本張力，必須在設計期就決策。**

`ddcov()`（`rtkpos.c:1128`）產生的 `R` 是**分塊稠密**矩陣（同一參考衛星的所有 DD 彼此相關），結構為 `R_b = σ_ref² · 1·1ᵀ + diag(σ_j²)`。GTSAM 的 `noiseModel::Robust` 要求 base noise model 為 isotropic/diagonal 才能正確運作。二者不能同時成立。本文件在 §4.2.1 與 §5.6 提出明確的分階段解法（Phase 2 用 block Gaussian + 區塊層級卡方檢定；Phase 3 改用 undifferenced/SD 表述 + 顯式接收機鐘差以恢復獨立性，從而啟用逐觀測量 robust kernel）。

## 1.3 關鍵決策摘要

| # | 決策 | 選擇 | 理由 | 章節 |
|---|---|---|---|---|
| D1 | 整體架構 | **A′ Residual Callback Hybrid** | 保有重複線性化，同時零重寫 | §3.4 |
| D2 | 圖最佳化後端 | **GTSAM 4.2+** | BSD-3、iSAM2 成熟、GNSS 社群驗證 | §3.5, §12.4 |
| D3 | EKF 路徑 | **完全不動，位元級等價** | 相容性硬性不變量 I1 | §3.1, §6 |
| D4 | 模式選擇 | 新增 `pos1-solver` 選項，非新增 `PMODE_*` | `PMODE_*` 語意為「定位模式」，solver 是正交維度 | §6.4 |
| D5 | NRT 主力 solver | **iSAM2 + Fixed-Lag Smoother** | 有界記憶體、可預測延遲 | §8 |
| D6 | 每日再處理 solver | **Batch FGO (LM + Robust)** | 最高精度，離線無延遲限制 | §8, §9 |
| D7 | DD 相關性處理 | Phase 2 block Gaussian；Phase 3 轉 SD + 顯式鐘差 | 兼顧統計正確性與 robust kernel | §4.2.1, §5.6 |
| D8 | 模糊度 | 沿用 `lambda.c` + `manage_amb_LAMBDA()`，FGO 提供更佳浮點解 | 避免重寫已驗證的 AR 邏輯 | §4.4 |
| D9 | GTSAM 為選用相依 | `-DENABLE_FGO=ON/OFF`，OFF 時完全不編譯 `src/fgo/` | 無 GTSAM 環境仍可建置 | §3.1 (I5), §6.9 |

## 1.4 Non-Goals（本階段明確不做）

- **不取代 RTKLIB**。FGO 是新增的第二引擎，EKF 永遠是預設值與回歸基準。
- **不重寫** RTCM decoder（`rtcm*.c`）、RINEX parser（`rinex.c`）、ephemeris engine（`ephemeris.c` / `preceph.c`）、大氣模型（`rtkcmn.c` 的 `tropmodel()`/`ionmodel()`、`ionex.c`）、座標轉換（`rtkcmn.c`）。
- **不在 Phase 1–3 導入 IMU/傾斜儀/InSAR/氣象**。Phase 4 才處理；但 Phase 3 的介面設計必須為其預留（§13）。
- **不改動任何 GUI**（`app/qtapp/`、`app/winapp/`）於 Phase 2；Phase 3 僅加入 solver 下拉選單。
- **不追求 kinematic 高動態場景最佳化**。本專案目標是準靜態監測測站。
- **不做 PPP-FGO** 於 Phase 2–3；`ppp.c` 的 `ppp_res()` 僅做純函式化準備（§6.2），實際 PPP-FGO 留待 Phase 4。

---

# 2. Baseline Analysis — 現況程式庫盤點

> 本章的所有敘述皆已對照 `chore/sync-upstream-20260820` @ `512ed8ac` 的原始碼逐項驗證。行號基準見文件開頭說明。

## 2.1 Fork 血統與結構特徵

本程式庫並非 upstream RTKLIB 2.4.3，而是 **RTKLIB-EX（前稱 demo5，rtklibexplorer）** 分支，再由 `rstltd` 二次分支。與 upstream 的關鍵差異，對本專案有影響者：

| 特徵 | 說明 | 對 FGO 的影響 |
|---|---|---|
| 標頭檔已拆分 | `rtklib.h` 為門面，實際內容在 `rtklib_const.h`(504行) / `rtklib_types.h`(824行) / `rtklib_api.h`(554行)，後三者含 `#error "include rtklib.h, not ..."` 防護 | 新增型別/常數/API 需修改對應的拆分檔，而非 `rtklib.h` 本體 |
| `varerr()` 已擴充 SNR 與接收機標準差項 | `rtkpos.c:406-452`，使用 `opt->err[5]`(snr_max)、`err[6]`(snr)、`err[7]`(rcv_std) 與 `obs->Pstd/Lstd` | 誤差模型比 upstream 豐富，FGO 可直接繼承，不需自建 |
| AR 邏輯大幅改寫 | `manage_amb_LAMBDA()` (`rtkpos.c:1909`)、`arfilter`、partial AR、`excsat` | 應完整重用，不重寫（§4.4） |
| GLONASS IC bias 自動校正 | `GLO_ARMODE_AUTOCAL`，狀態 `IL(f,opt)` | FGO 狀態向量需鏡射此設計 |
| `rtk_t` 已有 opaque context 欄位 | `rtklib_types.h:672` `void *solstat;` | 新增 `void *fgo;` 的先例已存在 |
| 已有 PPP phase-OSB loader 與 est-stec 相關近期開發 | commits `d220b8a0`, `4b376f51`, `505d12cc` | 表示團隊已在 `ppp.c` 有活躍修改；FGO 對 `ppp.c` 的改動需與之協調 |

**命名更正（重要）**：需求文件提及的 `resph()` 在本程式庫中**不存在**。PPP 的殘差函式實際名稱為 **`ppp_res()`**，位於 `src/ppp.c:974`，簽章為：

```c
static int ppp_res(int post, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *var_rs, const int *svh,
                   const double *dr, int *exc, const nav_t *nav,
                   const double *x, rtk_t *rtk, double *v, double *H, double *R,
                   double *azel);
```

（upstream RTKLIB 2.4.2 中曾名為 `res_ppp`。本文件後續一律使用 `ppp_res()`。）

## 2.2 解算主流程與 Hook 點

```
rtksvr.c:727  rtkpos(&svr->rtk, obs.data, obs.n, &svr->nav)     <- NRT 即時路徑
postpos.c:486 rtkpos(rtk, obs_ptr, n, &navs)                    <- 後處理路徑
                    |
                    v
        rtkpos()  src/rtkpos.c:2438        * 唯一的模式派發點
                    |
    +---------------+----------------+------------------+
    v               v                v                  v
 pntpos()      PMODE_SINGLE     pppos()            relpos()
 pntpos.c:646  -> return 1      ppp.c:1226         rtkpos.c:2068
    |                                |                  |
 rescode()                      ppp_res()          +----+-----+
 pntpos.c:277                   ppp.c:974          |          |
 estpos() / raim_fde()          udstate_ppp()   udstate()  zdres() x n
 resdop() / estvel()                            rtkpos.c:957  rtkpos.c:1049
                                                   |          |
                                                   |       ddres()
                                                   |       rtkpos.c:1240
                                                   |          |
                                                   |       filter()
                                                   |       rtkcmn.c:1479
                                                   |          |
                                                   |    manage_amb_LAMBDA()
                                                   |       rtkpos.c:1909
                                                   |          |
                                                   |       lambda()
                                                   |       lambda.c:180
```

**FGO 的注入點（injection point）**：`rtkpos()` (`rtkpos.c:2438`) 中，在 `pntpos()` 取得初始解之後、`relpos()` 呼叫之前。這是唯一需要新增分支的位置。理由：

- 該處已具備 `rtk->sol.rr`（SPP 初始解，作為 FGO 的 initial guess）與 `rtk->tt`（epoch 時間差）。
- `nu` / `nr`（rover/base 觀測數）已分離。
- 基站座標 `rtk->rb[]` 已設定。
- `PMODE_SINGLE` 與 `PMODE_PPP_*` 的早期返回已在其後，不受影響。

## 2.3 EKF 狀態向量佈局

定義於 `src/rtkpos.c:79-92`：

```c
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)   /* 頻率數 */
#define NP(opt)     ((opt)->dynamics==0?3:9)                     /* pos / pos+vel+acc */
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)       /* 電離層 */
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NL(opt)     ((opt)->glomodear!=GLO_ARMODE_AUTOCAL?0:NFREQGLO)
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))   /* 相位偏差 */
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
#define NX(opt)     (NR(opt)+NB(opt))

#define II(s,opt)   (NP(opt)+(s)-1)                  /* 電離層 (s: satno) */
#define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r))  /* 對流層 (r: 0=rov,1=ref) */
#define IL(f,opt)   (NP(opt)+NI(opt)+NT(opt)+(f))    /* GLO 接收機硬體偏差 */
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)       /* 相位偏差 (s:satno, f:freq) */
```

**關鍵性質，FGO 設計必須鏡射**：

1. **相位偏差狀態為「站間單差（single-differenced between receivers）」，非雙差**。`ddres()` (`rtkpos.c:1379-1394`) 中明確以 `v[nv] -= CLIGHT/freqi*x[ii] - CLIGHT/freqj*x[jj]` 形成雙差。這意味著 SD 偏差集合存在秩虧（rank deficiency，每個 system-frequency 群組有一個不可觀測的共同偏移），EKF 因為初始協方差有限而不會發散，但**在 factor graph 中會導致 Hessian 奇異**——必須顯式錨定（§4.4.2）。
2. `filter()` (`rtkcmn.c:1479`) 會壓縮掉 `x[i]==0 || P[i][i]<=0` 的狀態，實際參與更新的維度 `k` 遠小於 `nx`。典型 `k ≈ 60–120`。
3. 過程雜訊僅加在加速度區塊（`udpos()`，`rtkpos.c:566-571`）：`Q_ENU = diag(σ_ah², σ_ah², σ_av²) · |Δt|`，其中 `σ_ah = prn[3]`、`σ_av = prn[4]`，再經 `covecef(pos, Q, Qv)` 旋轉至 ECEF。**FGO 的 Acceleration Factor 必須使用完全相同的公式**，否則兩引擎不可比較。

## 2.4 可重用資產盤點

| RTKLIB 資產 | 位置 | FGO 重用方式 | 需否修改 |
|---|---|---|---|
| `satposs()` / `satpos()` | `ephemeris.c` | 直接呼叫，每 epoch 一次（與線性化點無關） | 否 |
| `zdres()` | `rtkpos.c:1049` | 純函式化後由 factor 回呼；輸入 `rr` 改為當前估計 | 是（§6.1） |
| `zdres_sat()` | `rtkpos.c:988` | 隨 `zdres()` | 否 |
| `ddres()` | `rtkpos.c:1240` | 純函式化後由 factor 回呼；`v` → error、`H` row → Jacobian | 是（§6.1） |
| `ddcov()` | `rtkpos.c:1128` | 直接呼叫產生 block covariance | 否 |
| `varerr()` (RTK) | `rtkpos.c:406` | 提升為 `EXPORT`，供 FGO 誤差模型使用 | 是（§6.1） |
| `varerr()` (SPP) | `pntpos.c:51` | 同上 | 是（§6.3） |
| `rescode()` | `pntpos.c:277` | undifferenced PR factor 的殘差來源 | 是（§6.3） |
| `ppp_res()` | `ppp.c:974` | Phase 4 PPP-FGO；Phase 2–3 僅做純函式化準備 | 是（§6.2） |
| `constbl()` | `rtkpos.c:1145` | **Site Constraint Factor 的直接前身**，公式可完整沿用 | 否（作為參考） |
| `prectrop()` | `rtkpos.c:1185` | 對流層 factor 的 mapping function + Jacobian | 否 |
| `lambda()` / `lambda_reduction()` / `lambda_search()` | `lambda.c:180/214/247` | FGO 浮點解 → LAMBDA 固定 | 否 |
| `manage_amb_LAMBDA()` / `resamb_LAMBDA()` / `ddidx()` / `restamb()` / `holdamb()` | `rtkpos.c:1909/1770/1563/1633/1661` | 完整重用 AR pipeline | 否（見 §4.4） |
| `tropmodel()` / `tropmapf()` / `ionmodel()` / `ionmapf()` | `rtkcmn.c` | 由 `zdres`/`ddres` 內部呼叫 | 否 |
| `ecef2pos()` / `pos2ecef()` / `covecef()` / `ecef2enu()` / `xyz2enu()` | `rtkcmn.c` | 座標與協方差旋轉 | 否 |
| `geodist()` / `satazel()` | `rtkcmn.c` | 幾何距離與 LOS 單位向量 | 否 |
| `tidedisp()` | `tides.c` | 地球潮汐改正（監測應用必要） | 否 |
| RTCM / RINEX / stream I/O | `rtcm*.c`, `rinex.c`, `stream.c`, `rtksvr.c` | 完全不動 | 否 |

**盤點結論**：需要修改的核心解算檔案僅 5 個（`rtkpos.c`、`pntpos.c`、`ppp.c`、`options.c`、`rtkcmn.c`）加上 3 個標頭檔與 2 個 CMake 檔，且絕大多數修改屬於「**純函式化 + 提升可見性**」這類低風險重構，而非邏輯變更。這證實了 A′ 架構的可行性。

## 2.5 現行 EKF 的量化基準（Phase 1 需建立）

在動任何一行程式碼之前，必須先建立可重現的基準。目前程式庫尚無此基準：`test/utest/` 僅有單元測試（`t_matrix.c`、`t_lambda.c`、`t_atmos.c`、`t_rinex.c`、`t_ppp.c` 等），無端到端解算回歸測試。Phase 1 的核心交付即為此基準（§11.1）。

---

# 3. System Architecture

## 3.1 設計不變量（Invariants）

以下五項為硬性約束。任何違反其中一項的設計提案應被否決。

| ID | 不變量 | 驗證方式 |
|---|---|---|
| **I1** | **EKF 路徑位元級等價**。當 `solver == FGO_SOLVER_EKF`（預設）時，程式行為與改動前完全相同 | 回歸測試：改動前後對同一資料集的 `.pos` 輸出做 byte-diff；`rtkoutstat()` 輸出亦須一致 |
| **I2** | **單向語言相依**。`src/*.c`（C99）不得 `#include` 任何 C++ 標頭；跨界僅透過 `src/fgo/rtklib_fgo_api.h` 的 C ABI | CI 檢查：以純 C compiler 編譯 `src/*.c` 必須成功 |
| **I3** | **觀測模型單一真相來源**。FGO 的 GNSS 殘差與 Jacobian 必須由 RTKLIB 函式產生，不得在 `src/fgo/` 中重新實作幾何距離、大氣改正、天線相位中心等 | Code review checklist；`src/fgo/` 中禁止出現 `geodist`、`tropmodel` 等的替代實作 |
| **I4** | **純函式化優先於複製**。若要重用某 static 函式，優先重構為無副作用、參數化的形式再提升可見性；禁止 copy-paste 到 `src/fgo/` | Code review；`git log` 需顯示為 refactor commit 而非 add commit |
| **I5** | **GTSAM 為選用相依**。`cmake -DENABLE_FGO=OFF`（預設）時，整個專案的建置結果與現況相同，且不需系統上存在 GTSAM | CI matrix：`ENABLE_FGO=OFF` 與 `ON` 兩種組態皆須通過 |

## 3.2 架構 A — RTKLIB Residual Export

### 資料流

```
[epoch k]
 RTKLIB: satposs() -> zdres() -> ddres()
          |
          +--> 匯出 (v_k, H_k, R_k, vflg_k) 至 IPC / 檔案 / 共享記憶體
                                |
                                v
                        GTSAM: 以 (v_k, H_k, R_k) 建構 JacobianFactor
                                |
                                v
                        LM / iSAM2 求解 -> 位置解
```

### 實作形態

- RTKLIB 端只新增「殘差傾印」功能（可視為 `rtkoutstat()` 的擴充版），輸出 `v`、`H`、`R`、`vflg`、`azel`、`sat` 等。
- GTSAM 端為獨立行程（Python 或 C++），讀取傾印檔或透過 ZeroMQ/共享記憶體接收。
- 兩者間**沒有函式呼叫關係**，只有資料契約。

### 優點

| 面向 | 說明 |
|---|---|
| **零侵入性** | RTKLIB 核心幾乎不需修改（僅新增輸出），I1 天然滿足，相容性風險趨近於零 |
| **開發速度最快** | GTSAM 端可用 Python (`gtsam` pip package) 快速原型，一週內可有第一版結果 |
| **完全解耦** | FGO 崩潰不影響即時 RTK 產線；可在既有系統旁路運行 |
| **易於研究與調參** | 資料一旦落地，可反覆嘗試不同 factor 組合、robust kernel、window 長度，無需重編譯 C 程式 |
| **語言自由** | 可用 Python/Julia/MATLAB 做演算法探索 |

### 缺點

| 面向 | 說明 | 嚴重度 |
|---|---|---|
| **無法重複線性化（致命）** | `H_k` 是在 `x0` 處算的固定值。GTSAM 迭代時若更新 `X`，殘差不會隨之重算，最佳化在第一次迭代後即無實質進展。等價於一次 sparse Gauss-Newton step | 高 |
| **非線性優勢喪失** | 大初始誤差（例如冷啟動、長時間失鎖後重收斂）情境下與 EKF 無異 | 高 |
| **Robust kernel 效果受限** | Robust kernel 需要在迭代中依殘差大小重新加權（IRLS）。線性化點固定時，權重更新後殘差不變，收斂行為退化 | 中高 |
| **資料量大** | 每 epoch 的 `H` 為 `nv × nx` 稠密陣列（典型 `nv≈40`, `nx≈400`），未壓縮下約 128 KB/epoch。1 Hz 連續運作為 11 GB/天/測站 | 中 |
| **延遲與同步** | IPC 序列化、行程切換造成不確定延遲，NRT 難以保證 | 中 |
| **雙重維護** | 觀測模型的知識被切成兩半（RTKLIB 算殘差、GTSAM 算圖），介面契約需嚴格版本管理 | 中低 |

### 適用情境

**架構 A 是優秀的 Phase 2 研究平台，但不是最終產品架構。** 它的價值在於：以最低風險快速回答「FGO 對我們的監測資料是否真有幫助、幫助多大」這個問題，並產生調參經驗與驗證資料集。

## 3.3 架構 B — RTKLIB Native FGO

### 資料流

```
[epoch k]
 RTKLIB: satposs() -> zdres() -> ddres()
          |
          v
 src/fgo/*.c  (以 C99 自行實作)
   - 圖結構管理 (variable/factor container)
   - 稀疏矩陣組裝 (CSC/CSR)
   - 變數排序 (COLAMD/AMD)
   - 稀疏 Cholesky 分解
   - Levenberg-Marquardt 迭代
   - Robust kernel (IRLS)
   - 增量更新 (Bayes tree 或 periodic re-batch)
          |
          v
     Solution -> rtk->sol
```

### 優點

| 面向 | 說明 |
|---|---|
| **零外部相依** | 不需 GTSAM/Eigen/Boost/TBB。部署簡單，嵌入式友善（監測站的邊緣裝置可能是 ARM SBC） |
| **建置系統不變** | 純 C，`src/CMakeLists.txt` 的 `aux_source_directory` 自動涵蓋 |
| **完全掌控** | 可精細調整記憶體配置、避免動態配置、達成硬即時（hard real-time）保證 |
| **授權單純** | 全部 BSD-2，無混合授權議題 |
| **可重複線性化** | 由於在行程內，可任意呼叫 `zdres`/`ddres` 重算 |

### 缺點

| 面向 | 說明 | 嚴重度 |
|---|---|---|
| **工程量巨大（致命）** | 稀疏 Cholesky + AMD ordering + Bayes tree 增量更新是 GTSAM 十餘年的核心資產。保守估計 8,000–15,000 行高難度數值程式碼 | 高 |
| **數值穩健性風險** | 稀疏分解的 pivoting、fill-in 控制、病態系統處理，是極易出錯且極難除錯的領域。GNSS 問題本身條件數就差（模糊度與位置強相關） | 高 |
| **與專案目標衝突** | 需求明確要求「盡可能重用、避免重寫」。自建最佳化器是最大規模的重寫 | 高 |
| **iSAM2 難以自行實作** | Bayes tree 的部分重線性化與變數重排序，是本專案 NRT 需求的關鍵，也是最難自行實作的部分 | 高 |
| **無法受惠於上游改進** | GTSAM 持續有新 solver、新 robust kernel、GPU 加速。自建即與社群脫節 | 中 |
| **人力風險** | 需要具備數值線性代數與 SLAM 背景的工程師；此類人才稀缺，離職即形成單點失效 | 中 |
| **驗證成本高** | 自建 solver 需與參考實作逐項比對，本身就是一個子專案 | 中 |

### 適用情境

架構 B 在以下條件同時成立時才合理：(a) 部署環境**絕對禁止** C++ 或第三方函式庫；(b) 有硬即時延遲上限；(c) 團隊具備專職的數值最佳化工程師。本專案不符合這些條件。

**但架構 B 的一個受限變體是有價值的**：僅實作 **Batch Gauss-Newton with fixed sliding window**（不做 iSAM2、不做增量），使用固定大小的稠密或簡單稀疏求解。這在 window 很小（例如 10 epochs）時可行，約 1,500–2,500 行，作為「GTSAM 不可用時的 fallback solver」。列為 §14 的 Open Question OQ-3。

## 3.4 架構 A′ — Residual Callback Hybrid（推薦）

### 核心概念

> RTKLIB 保留**觀測模型**的所有權；GTSAM 取得**圖結構與最佳化**的所有權。二者透過一層薄 C ABI 在**同一行程內**耦合，且該 ABI 是**可重入的（re-entrant）**——GTSAM 每次線性化都會回呼進 RTKLIB。

這解決了架構 A 的致命缺陷（線性化點固定），同時避免架構 B 的致命缺陷（重寫最佳化器）。

### 資料流

```
                      rtkpos()  [src/rtkpos.c:2438]
                            |
                  solver == FGO_SOLVER_EKF ?
                     /                    \
                  yes                      no
                   |                        |
              relpos()              fgo_process_epoch()   [src/fgo/fgo_solver.cpp]
           (完全不變)                        |
                              +-------------+-------------+
                              |                           |
                    (1) 每 epoch 一次的前處理        (2) 建圖
                        satposs()                    加入 GnssDDFactor,
                        selsat()                     MotionFactor,
                        cycle-slip 偵測               VelocityFactor,
                        ddcov() -> R                 AccelFactor,
                              |                      SiteConstraintFactor
                              |                           |
                              +-------------+-------------+
                                            |
                                            v
                              (3) GTSAM 最佳化 (LM / iSAM2)
                                            |
                        +-------------------+-------------------+
                        |          每次線性化迭代                |
                        |               |                       |
                        |               v                       |
                        |   GnssDDFactor::evaluateError(X)      |
                        |               |                       |
                        |               v                       |
                        |   rtklib_fgo_dd_residual(ctx, X, ...) |  <- C ABI 回呼
                        |               |                       |
                        |               v                       |
                        |     zdres_pure() + ddres_pure()       |  <- RTKLIB 純函式
                        |     [src/rtkpos.c, 重構後]             |
                        |               |                       |
                        |               v                       |
                        |      回傳 (error, Jacobian)            |
                        +-------------------+-------------------+
                                            |
                                            v
                              (4) 取出解與邊緣協方差
                                  -> rtk->x, rtk->P, rtk->sol
                                            |
                                            v
                              (5) manage_amb_LAMBDA()  [重用既有 AR]
                                            |
                                            v
                              (6) fgo_export_insight()  -> AI Insight JSON
```

### 關鍵機制：純函式化（Purification）

`zdres()` 目前已接近純函式（輸入 `rr`，輸出 `y`/`e`/`azel`/`freq`，無全域狀態），只需提升可見性。

`ddres()` 則有副作用：它寫入 `rtk->ssat[].resp/resc`、`rtk->ssat[].vsat`、並讀取 `rtk->x` / `rtk->sol.time` / `rtk->rb`。重構方案（§6.1）為抽出一個 `ddres_core()`，其所有輸入透過參數傳入、所有輸出寫入呼叫端提供的緩衝區，`ddres()` 則成為薄包裝以維持 I1：

```c
/* 重構後（示意）*/
static int ddres_core(const ddres_ctx_t *ctx, const double *x,
                      double *v, double *H, double *R, int *vflg,
                      ddres_stat_t *stat_out);   /* 純函式，無 rtk_t 副作用 */

static int ddres(rtk_t *rtk, ...)                /* 薄包裝，行為完全不變 */
{
    ddres_ctx_t ctx; ddres_stat_t st;
    /* 從 rtk 填充 ctx ... */
    int nv = ddres_core(&ctx, x, v, H, R, vflg, &st);
    /* 將 st 寫回 rtk->ssat[] ... */
    return nv;
}
```

`ddres_core()` 是 FGO 回呼的目標。此重構滿足 I4（純函式化而非複製）與 I1（EKF 行為不變）。

### 誤差與 Jacobian 的符號約定（重要）

RTKLIB 的慣例：`v = z - h(x)`（innovation），`H` 以狀態為列儲存（`H[state + meas*nx]`），亦即 `H` 的第 `nv` 欄是 `∂h/∂x` 的第 `nv` 列。

GTSAM 的慣例：`NoiseModelFactor::evaluateError()` 回傳 `e(X) = h(X) - z`，線性化為 `A·δ + b`，其中 `b = e(X0)`，`A = ∂e/∂δ`。

因此對應關係為：

```
GTSAM error      e  = -v                              (RTKLIB innovation 取負)
GTSAM Jacobian   A  = H 的對應欄轉置                    (逐元素相同，僅儲存佈局轉換)
```

以位置區塊驗證（`rtkpos.c:1350-1354`）：RTKLIB 寫 `Hi[k] = -e[k+iu[i]*3] + e[k+iu[j]*3]`，其中 `i` 為參考衛星、`j` 為對象衛星。由於 `geodist()` 回傳的 `e` 為接收機指向衛星的單位向量，`∂ρ/∂r_rcv = -e`，故 DD 幾何距離對測站座標的偏導為 `-e_i + e_j`，與 RTKLIB 完全一致。**FGO 端不需重新推導，直接取用即可。**

### 優點

| 面向 | 說明 |
|---|---|
| **保有重複線性化** | 每次 LM/GN 迭代都以最新 `X` 重算殘差與 Jacobian，這是 FGO 的核心價值 |
| **零觀測模型重寫** | 幾何、大氣、天線、潮汐、頻率處理全數由 RTKLIB 提供（滿足 I3） |
| **Robust kernel 完全有效** | IRLS 的權重更新配合真實的重線性化，收斂行為正確 |
| **無 IPC 開銷** | 同行程函式呼叫，NRT 延遲可控 |
| **可重用整套 AR** | `manage_amb_LAMBDA()` 直接吃 FGO 的 `x`/`P`，不需改 |
| **可與 EKF 並行比較** | 同一 epoch 可同時跑兩者，輸出差異供 AI Insight 做交叉驗證 |
| **Phase 2 成果可直接演進** | Phase 2 架構 A 的 factor 設計與調參結果，在 A′ 中完全沿用；只是殘差來源從「讀檔」變成「回呼」 |

### 缺點與緩解

| 缺點 | 緩解 |
|---|---|
| 需要重構 `ddres()` 為純函式（觸碰核心解算碼） | I1 回歸測試（byte-diff）+ 薄包裝設計；重構本身有獨立的 milestone 與 review |
| 引入 GTSAM 相依（Eigen/Boost/TBB） | `ENABLE_FGO=OFF` 預設關閉（I5）；提供 Docker 建置環境 |
| 回呼開銷：每次線性化重算全部殘差 | 每 epoch 觀測數有限（`nv≈40`），`zdres+ddres` 成本遠低於 Cholesky；實測後若成為瓶頸，可快取 `satposs()` 結果（本已與 `X` 無關） |
| C/C++ 邊界的錯誤處理（例外不可跨 C ABI） | 所有 C ABI 函式回傳錯誤碼；`src/fgo/` 的 C++ 例外在 ABI 邊界以 `try/catch(...)` 攔截並轉為錯誤碼 |

## 3.5 決策矩陣與推薦

評分：5 = 最佳，1 = 最差。權重反映本專案（NRT 監測）的優先序。

| 準則 | 權重 | A (Export) | B (Native) | **A′ (Callback)** |
|---|---|---|---|---|
| 重複線性化能力（FGO 核心價值） | 25% | 1 | 5 | **5** |
| 觀測模型重用程度 | 20% | 4 | 4 | **5** |
| 實作與維護工作量（越低越好） | 15% | 5 | 1 | **3** |
| NRT 即時性與延遲可控性 | 15% | 2 | 5 | **4** |
| 對既有 EKF 的相容性風險（越低越好） | 10% | 5 | 3 | **4** |
| Robust kernel 有效性 | 10% | 2 | 4 | **5** |
| 多感測器擴充性（Phase 4） | 5% | 3 | 2 | **5** |
| **加權總分** | 100% | **2.85** | **3.70** | **4.45** |

### 推薦

> **採用架構 A′，但以架構 A 作為 Phase 2 的研究跳板。**

分階段落地的理由：

1. **Phase 2 先做架構 A**（殘差匯出 + 離線 GTSAM 原型）。此階段不觸碰核心解算碼，風險極低，卻能在 4–6 週內回答最重要的問題：*FGO 對我們的實際監測資料，相較 EKF 有多少改善？* 若答案是「幾乎沒有」，則可在投入大量工程前終止或調整方向。此階段同時產出：factor 設計驗證、robust kernel 參數、window 長度、以及一份可重複的驗證資料集。
2. **Phase 3 升級為 A′**（純函式化 + 行程內回呼 + iSAM2）。此時 factor 設計已驗證，工程風險集中在重構與整合，而非演算法未知數。
3. 架構 A 的產出（殘差傾印格式、離線分析腳本）在 Phase 3 之後**仍保留**，作為除錯與離線再處理的工具鏈。這不是被丟棄的工作。

## 3.6 目標模組結構與相依方向

```
                    +---------------------------------------+
                    |  app/  (consapp, qtapp, winapp)       |
                    |  rtkrcv / rnx2rtkp / rtknavi ...      |
                    +------------------+--------------------+
                                       | (無變更 / Phase 3 僅加選項)
                    +------------------v--------------------+
                    |  src/  RTKLIB core (C99)              |
                    |                                       |
                    |  rtkpos.c  <- 新增 solver 分支         |
                    |  pntpos.c  <- rescode 純函式化         |
                    |  ppp.c     <- ppp_res 純函式化 (P4)    |
                    |  options.c <- 新增 fgo-* 選項          |
                    |  rtkcmn.c  <- 新增數值輔助函式          |
                    |                                       |
                    |  (rtcm*.c rinex.c ephemeris.c         |
                    |   preceph.c tides.c lambda.c          |
                    |   stream.c rtksvr.c  == 完全不變)      |
                    +------+-------------------------^------+
                           |                         |
      C ABI (單向宣告)      |                         | C ABI 回呼
      rtklib_fgo_api.h     |                         | (re-entrant)
                           v                         |
                    +------+-------------------------+------+
                    |  src/fgo/  (C++17)                    |
                    |                                       |
                    |  fgo_config.h    設定與型別            |
                    |  fgo_solver.cpp  生命週期與排程        |
                    |  fgo_graph.cpp   圖與變數管理          |
                    |  fgo_factor.cpp  factor 實作           |
                    |  fgo_gtsam.cpp   GTSAM 後端封裝        |
                    |  fgo_insight.cpp AI Insight 輸出       |
                    +------------------+--------------------+
                                       |
                    +------------------v--------------------+
                    |  GTSAM 4.2+ / Eigen 3.3+ / Boost      |
                    +---------------------------------------+
```

**相依規則（對應 I2）**：

- `src/*.c` → `src/fgo/rtklib_fgo_api.h`：僅函式宣告，純 C 標頭，無 C++ 內容。
- `src/fgo/*.cpp` → `src/rtklib.h`：合法，因 `rtklib.h:51` 已有 `extern "C"` guard。
- `src/fgo/*.cpp` → GTSAM：僅在 `.cpp` 內，不洩漏至任何被 C 引入的標頭。
- **禁止**：`src/*.c` 引入任何 GTSAM/Eigen 標頭。

---

# 4. Detailed Design

## 4.1 變數（Variables）與 GTSAM Key Schema

Factor graph 的節點即待估參數。設計原則：**鏡射 RTKLIB 的狀態定義**，使 FGO 解可直接寫回 `rtk->x` / `rtk->P` 並被既有的 AR、輸出、統計程式碼消費。

### 4.1.1 變數型錄

| 變數 | 符號 | GTSAM 型別 | 維度 | 對應 RTKLIB 狀態 | 出現頻率 |
|---|---|---|---|---|---|
| 測站位置 | `P(k)` | `gtsam::Point3` | 3 | `x[0..2]` (ECEF) | 每 epoch |
| 測站速度 | `V(k)` | `gtsam::Vector3` | 3 | `x[3..5]` | 每 epoch（`dynamics≥1`） |
| 測站加速度 | `A(k)` | `gtsam::Vector3` | 3 | `x[6..8]` | 每 epoch（`dynamics=2`） |
| 站間單差相位偏差 | `B(sat,f)` | `gtsam::Vector1` | 1 | `x[IB(s,f,opt)]` | 隨衛星弧段 |
| 對流層 ZTD (rover) | `T(k,0)` | `gtsam::Vector1` 或 `Vector3` | 1 或 3 | `x[IT(0,opt)]` | 每 N epoch（分段常數） |
| 對流層 ZTD (base) | `T(k,1)` | 同上 | 1 或 3 | `x[IT(1,opt)]` | 同上 |
| 電離層 STEC | `I(k,sat)` | `gtsam::Vector1` | 1 | `x[II(s,opt)]` | 每 epoch（`ionoopt=EST`） |
| GLONASS IC bias | `L(f)` | `gtsam::Vector1` | 1 | `x[IL(f,opt)]` | 全域（單一節點） |

### 4.1.2 Key 編碼

GTSAM `Key` 為 64-bit。使用 `gtsam::Symbol(char c, uint64_t j)`（8-bit char + 56-bit index）。

```cpp
// src/fgo/fgo_config.h  （示意）
namespace fgo {

inline gtsam::Key keyPos (uint64_t k)            { return gtsam::Symbol('p', k); }
inline gtsam::Key keyVel (uint64_t k)            { return gtsam::Symbol('v', k); }
inline gtsam::Key keyAcc (uint64_t k)            { return gtsam::Symbol('a', k); }
inline gtsam::Key keyTrop(uint64_t k, int rcv)   { return gtsam::Symbol('t', k*2 + rcv); }
inline gtsam::Key keyIono(uint64_t k, int sat)   { return gtsam::Symbol('i', k*MAXSAT + (sat-1)); }
inline gtsam::Key keyGloIcb(int f)               { return gtsam::Symbol('g', f); }

/* 相位偏差以「弧段 (arc)」為單位，而非 (sat,freq) —— 見 4.1.3 */
inline gtsam::Key keyBias(uint64_t arc_id)       { return gtsam::Symbol('b', arc_id); }

} // namespace fgo
```

### 4.1.3 相位偏差的「弧段（arc）」語意 — 關鍵設計

RTKLIB 的 EKF 對 cycle slip 的處理是**就地重設狀態**（`udbias()`，`rtkpos.c:831`，呼叫 `initx()` 給予大變異數）。在 factor graph 中**不能這樣做**——圖是累積的，重設一個既有節點會破壞歷史 factor 的意義。

正確作法：**cycle slip 產生一個新的變數節點**。

```
衛星 G05 / L1 的偏差變數演進：

epoch:   0    1    2    3    4 (slip!)  5    6    7
節點:   b#17 -----------------→        b#41 ----------→
        （弧段 A，epoch 0-3）           （弧段 B，epoch 4-7）
```

實作上維護一個對照表 `arc_map[sat][freq] -> arc_id`，在偵測到 slip（重用 `detslp_ll()` / `detslp_gf()` / `detslp_dop()` / `detslp_code()`，`rtkpos.c:660/680/729/758`）時遞增 `arc_id`。

這帶來一個 EKF 沒有的能力：**若後續判定該 slip 為誤報，可在圖中加入一條連接 `b#17` 與 `b#41` 的軟約束 factor（`|b#41 - b#17| ~ N(0, σ_slip²)`），把兩段弧「縫合」回去**，恢復被誤報打斷的長期連續性。這對長時間靜態監測的 mm 級穩定度有直接助益，是 FGO 對監測應用最實質的優勢之一（見 §9.1）。

### 4.1.4 時間索引與 marginalization

`k` 為 epoch 序號（`rtk->epoch`，`rtkpos.c:2542` 已在遞增）。滑動視窗中，超出 window 的變數以 GTSAM 的 `IncrementalFixedLagSmoother` 自動邊緣化，產生的 marginal factor 保留其對 window 內變數的資訊。**不可直接刪除節點**——那等同丟棄資訊，會使解退化。

---

## 4.2 Factor 型錄

以下每個 factor 皆給出：定義、殘差、Jacobian、協方差、實作要點。

### 4.2.1 GNSS Double-Difference Factor（主力）

#### 定義

連接：`P(k)`，以及該 epoch 內所有參與 DD 的相位偏差節點 `B(arc)`、對流層 `T(k,·)`、電離層 `I(k,·)`、GLONASS IC bias `L(f)`。

**這是一個 n 元（n-ary）、向量值（vector-valued）factor**，維度等於該 (系統, 頻率, code/phase) 區塊內的 DD 個數 `nb`。設計成向量值而非逐 DD 拆分，是為了正確承載 `ddcov()` 的相關性協方差（見下方「協方差」）。

#### 觀測模型

對參考衛星 `i`、對象衛星 `j`、rover `u`、base `r`：

```
h_DD = (ρ_u^i - ρ_r^i) - (ρ_u^j - ρ_r^j)
       + 電離層項  (ionoopt == IONOOPT_EST)
       + 對流層項  (tropopt >= TROPOPT_EST)
       + 相位偏差項 (phase, mode > PMODE_DGPS)
       + GLONASS IC bias 項 (glomodear == AUTOCAL)
```

其中 `ρ` 由 `zdres()` → `zdres_sat()` 產生，已包含：衛星鐘差、Sagnac 效應（`geodist()`）、乾對流層（`tropmodel()` + `tropmapf()`）、接收機天線相位中心（`antmodel()`）、固體潮/海潮/極潮（`tidedisp()`，若 `opt->tidecorr`）。

#### 殘差（Residual）

```cpp
// src/fgo/fgo_factor.cpp（示意）
gtsam::Vector GnssDDFactor::evaluateError(
        const gtsam::Point3& p,            /* 當前位置估計 */
        const std::vector<double>& others, /* bias/trop/iono/glo_icb */
        boost::optional<gtsam::Matrix&> Hp,
        boost::optional<std::vector<gtsam::Matrix>&> Hother) const
{
    /* 1. 將 GTSAM 變數打包成 RTKLIB 的扁平狀態向量 x[] */
    pack_state(p, others, x_buf_);

    /* 2. 回呼 RTKLIB：以當前 x 重新計算 zdres + ddres */
    int nv = rtklib_fgo_dd_eval(ctx_, x_buf_,
                                v_buf_, H_buf_, /*R=*/nullptr, vflg_buf_);
    if (nv != dim_) return gtsam::Vector::Zero(dim_);  /* 觀測結構改變，見下 */

    /* 3. 符號轉換：GTSAM error = -v */
    gtsam::Vector e(dim_);
    for (int m = 0; m < dim_; ++m) e(m) = -v_buf_[block_ofs_ + m];

    /* 4. Jacobian：由 RTKLIB 的 H (state-major) 取出對應欄，轉為 GTSAM 的 (dim x 3) 等 */
    if (Hp)     unpack_jacobian_pos  (H_buf_, block_ofs_, dim_, *Hp);
    if (Hother) unpack_jacobian_other(H_buf_, block_ofs_, dim_, *Hother);

    return e;
}
```

**要點 1 — 觀測結構固定性**：一個 factor 建立後，其涉及的衛星集合與 DD 配對必須固定。`ddres()` 內部會**動態選擇最小變異數的參考衛星**（`rtkpos.c:1285-1330`）。若在迭代中參考衛星改變，factor 的維度與意義會漂移。**解法**：`rtklib_fgo_dd_eval()` 接受一個「凍結的配對表」參數（`sat_ref[]`, `sat_obs[]`, `freq_idx[]`, `is_code[]`），該表在 epoch 建圖時決定一次，之後迭代中不再重選。這需要 `ddres_core()` 支援「外部指定參考衛星」模式（§6.1 修改項 M3）。

**要點 2 — 重新線性化的正確性**：因為 `zdres()` 會以新的 `rr` 重算 `geodist()`、`satazel()`、`tropmapf()`、`antmodel()`，這是**完整的非線性重估**，不是僅更新線性項。這正是 A′ 相對 A 的價值所在。

#### Jacobian

| 對象變數 | 偏導 | RTKLIB 出處 |
|---|---|---|
| 位置 `P(k)` (3) | `∂h/∂r = -e_i + e_j` | `rtkpos.c:1352` `Hi[k] = -e[k+iu[i]*3] + e[k+iu[j]*3]` |
| 電離層 `I(k,sat_i)` | `+didx_i`，`didx_i = (code?-1:+1)·m_i·(f_L1/f_i)²` | `rtkpos.c:1360-1367` |
| 電離層 `I(k,sat_j)` | `-didx_j` | 同上 |
| 對流層 `T(k,0)` (rover) | `+(dtdx_u[i] - dtdx_u[j])` | `rtkpos.c:1370-1376`；`dtdx` 來自 `prectrop()` (`rtkpos.c:1185`) |
| 對流層 `T(k,1)` (base) | `-(dtdx_r[i] - dtdx_r[j])` | 同上 |
| 相位偏差 `B(arc_i)` | `+c/f_i`（非 IFLC）或 `+1`（IFLC） | `rtkpos.c:1383-1392` |
| 相位偏差 `B(arc_j)` | `-c/f_j`（非 IFLC）或 `-1`（IFLC） | 同上 |
| GLONASS IC bias `L(f)` | `df = (f_i - f_j)/Δf_GLO` | `rtkpos.c:1398-1400` |

**所有 Jacobian 皆直接取自 RTKLIB 既有程式碼，FGO 端零推導、零重寫**（滿足 I3）。

**近似的量化（已實測，2026-08-21）**：RTKLIB 的位置 Jacobian 是**純幾何**的（`-e_i + e_j`），但 `zdres()` 中另有兩項也依賴測站位置，且未被納入 Jacobian。以 `numericalDerivative` 對照解析 Jacobian，逐項移除後量測到的相對誤差為：

| 被忽略的項 | 相對誤差 |
|---|---|
| 對流層乾延遲對測站高度的偏導 | **4.1e-4** |
| `geodist()` 內的 Sagnac 項對測站位置的偏導 | 6.0e-6 |
| （兩者皆移除後的殘餘＝數值極限） | ~6e-6 |

對流層項為主因。**注意 `zdres()` 是無條件套用 `mapfh*zhd` 的**——`opt->tropopt` 只決定 `ddres()` 是否**估計**對流層狀態，並不控制模型延遲是否套用，因此把 `tropopt` 設為 `off` 並不會移除此項（這一點在調查中一度造成誤判）。ZHD 隨高度約以 −2.9e-4 m/m 衰減，而高低仰角衛星的 mapping function 差異使 DD 無法抵消。

**這對 FGO 是可接受的**：Gauss-Newton 使用近似 Jacobian 只影響收斂**速率**，不影響收斂**位置**，因為殘差本身是精確的。實務影響是每個 epoch 可能多一兩次迭代。

**但它界定了 §6.11 G4 的合理門檻**：Jacobian 對數值微分的相對誤差不可能優於 ~4e-4，訂在 1e-6 是不可達成的。`test/fgo/check_fgo_backend.sh` 採用 2e-3。若未來要追求 mm 等級並在意收斂速率，補上 `∂(mapfh·zhd)/∂r` 是明確的改進點。

#### 協方差（Covariance）

由 `varerr()` (`rtkpos.c:406`) 產生每個 SD 的變異數 `σ²`，再由 `ddcov()` (`rtkpos.c:1128`) 組成 DD 協方差矩陣。`ddcov()` 的實際公式為：

```c
R[k+i + (k+j)*nv] = Ri[k+i] + (i==j ? Rj[k+i] : 0.0);
```

由於區塊 `b` 內所有 DD 共用同一參考衛星，`Ri[k+i]` 在區塊內為常數 `σ_ref²`。故：

```
R_b = σ_ref² · 1·1ᵀ  +  diag(σ_1², σ_2², ..., σ_nb²)
      \_____________/    \___________________________/
       參考衛星共用誤差            各對象衛星獨立誤差
       (rank-1)                    (diagonal)
```

這是「對角 + rank-1」結構。三種處理方式：

| 方式 | 作法 | 優點 | 缺點 | 採用 |
|---|---|---|---|---|
| **(a) Block Gaussian** | 單一向量值 factor，`noiseModel::Gaussian::Covariance(R_b)`，GTSAM 內部做 Cholesky 白化 | 統計嚴格正確 | 無法套用逐 DD 的 robust kernel | **Phase 2–3 預設** |
| **(b) 對角近似** | 忽略相關性，`R ≈ diag(σ_ref² + σ_j²)`，逐 DD 建立 1 維 factor | 可用 robust kernel；實作簡單 | 低估相關性，協方差過度樂觀（形式協方差偏小），需經驗性膨脹因子 | 可選（`fgo-ddcov=diag`） |
| **(c) 顯式參考誤差變數** | 為每個區塊引入潛在變數 `ε_ref ~ N(0, σ_ref²)`，DD factor 改為連接 `ε_ref`，殘差間即獨立 | 統計正確 **且** 可用 robust kernel | 增加變數數量；`ε_ref` 隨參考衛星切換而換節點 | **Phase 3 進階選項** |

> **設計決策 D7**：Phase 2 採 (a)；離群值處理在**區塊層級**以卡方檢定（`chisqr[]` 表已存在於 `rtklib_api.h:8`）與 RTKLIB 既有的 `maxinno` 硬門檻進行。Phase 3 評估 (c)，或改用 §4.2.2/§4.2.3 的非差表述以獲得完全獨立的殘差。

利用「對角 + rank-1」結構，反矩陣有 Sherman–Morrison 閉式解，複雜度 `O(nb)` 而非 `O(nb³)`：

```
R_b⁻¹ = D⁻¹ - (σ_ref² · D⁻¹ 1 1ᵀ D⁻¹) / (1 + σ_ref² · 1ᵀ D⁻¹ 1),   D = diag(σ_j²)
```

若 `nb` 大（>30）且效能敏感，可在 `fgo_gtsam.cpp` 中提供自訂 `noiseModel` 覆寫預設 Cholesky。列為效能最佳化選項，非 Phase 2 必要。

---

### 4.2.2 GNSS Pseudorange Factor（非差 / undifferenced）

#### 用途

1. Single-point / DGNSS 模式下的 FGO。
2. Phase 3 的「獨立殘差」表述，以啟用逐觀測量 robust kernel。
3. RTK 冷啟動時提供絕對位置約束，避免 DD 的相對性造成整體平移。

#### 連接

`P(k)`、接收機鐘差 `C(k, sys)`（每個 GNSS 系統一個，對應 `sol.dtr[0..5]`）、`T(k,0)`、`I(k,sat)`。

#### 殘差

直接來自 `rescode()` (`pntpos.c:277`)：

```
e = -v_rescode
  = ρ_computed - P_observed
  = |r_sat - r_rcv| + Sagnac - c·dt_sat + c·dt_rcv + T_trop + I_iono + TGD - P_obs
```

`rescode()` 已處理：`prange()` 的雙頻無電離層組合與 TGD (`pntpos.c:110`)、`ionocorr()` (`pntpos.c:204`)、`tropcorr()` (`pntpos.c:251`)、SNR mask (`snrmask()`)、衛星健康與排除 (`satexclude()`)。

#### Jacobian

```
∂e/∂r_rcv    = -e_los            (pntpos.c:322 附近，H[j+i*NX] = -e[j])
∂e/∂c·dt_rcv = +1                (對應系統的鐘差欄)
∂e/∂ZTD      = mapping function (若估計對流層)
∂e/∂STEC     = ±(f_L1/f_i)²      (code 為 +，phase 為 −)
```

#### 協方差

`varerr()` (`pntpos.c:51`)：

```
var = fact² · eratio[0]² · [ err[1]² + err[2]²/sin(el) + err[6]²·10^(0.1·(err[5]-SNR)) ]
      + (err[7] · Pstd)²
      × 3²  (若 IONOOPT_IFLC)
```

**注意與 RTK 版 `varerr()` 的差異**：SPP 版用 `1/sin(el)`，RTK 版用 `1/sin²(el)`（`rtkpos.c:438` — `b*b/sinel/sinel`）。這不是筆誤，而是兩種常見的仰角加權模型。FGO 必須依 factor 型別選用對應版本，不可混用。

#### Robust kernel

**此 factor 的殘差彼此獨立**（每顆衛星一個 1 維 factor），因此 `noiseModel::Robust` 可直接套用。這是 Phase 3 遷移至非差表述的主要動機。

---

### 4.2.3 GNSS Carrier Phase Factor（非差 / 站間單差）

#### 用途

Phase 3 的高精度表述；亦是 TDCP factor（§4.2.5）的基礎。

#### 殘差

```
e = |r_sat - r_rcv| + Sagnac - c·dt_sat + c·dt_rcv + T_trop - I_iono
    + λ_f · (N_sat,f + b_rcv,f + b_sat,f)
    + λ_f · φ_windup                        (model_phw(), ppp.c:296)
    - λ_f · L_observed
```

站間單差（SD）表述可消去衛星鐘差與衛星硬體偏差，且保留獨立殘差結構（不同衛星的 SD 互不相關，因為它們不共用參考衛星）。**這是 Phase 3 推薦的表述**：兼具 DD 的誤差消除能力與非差的獨立性，代價是需顯式估計接收機間鐘差 `Δdt_rb`（每 epoch 每系統一個變數，共 1–6 個）。

| 表述 | 殘差獨立性 | 需估計的額外變數 | 消除的誤差 | Robust kernel |
|---|---|---|---|---|
| 非差 (UD) | 是 — 完全獨立 | 接收機鐘差、衛星鐘差殘差、衛星偏差 | 少 | 是 |
| **站間單差 (SD)** | 是 — 完全獨立 | 接收機間鐘差 `Δdt_rb`（1–6） | 衛星鐘差、衛星硬體偏差、大部分大氣（短基線） | 是 |
| 雙差 (DD) | 否 — 區塊相關 | 無 | 上述 + 接收機鐘差與偏差 | 否（需 §4.2.1 (c)） |

> **設計建議**：Phase 2 用 DD（最貼近現行 RTKLIB，驗證成本最低）；Phase 3 提供 SD 選項（`fgo-obsmodel=sd`）以啟用完整 robust 能力。兩者共用同一套 `zdres()` 輸出，切換成本可控。

#### 週波未定值（Ambiguity）處理

見 §4.4。

---

### 4.2.4 Motion Factor（位置連續性約束）

#### 用途

相鄰 epoch 位置的平滑約束。在監測應用中，這是把「測站是準靜態的」這一先驗知識注入估計的主要管道，也是 FGO 相對於單 epoch 最小平方的核心增益來源。

#### 連接

`P(k)` ↔ `P(k+1)`（靜態模式）；或 `P(k)`, `V(k)`, `P(k+1)`（等速模式）。

#### 殘差

```
靜態模式 (mode == PMODE_STATIC / STATIC_START):
    e = p_{k+1} - p_k                                            (dim 3)

等速模式 (dynamics >= 1):
    e = p_{k+1} - p_k - v_k · Δt                                 (dim 3)

等加速模式 (dynamics == 2):
    e = p_{k+1} - p_k - v_k · Δt - 0.5 · a_k · Δt²               (dim 3)
```

第三式的 `0.5·Δt²` 項與 RTKLIB `udpos()` 的狀態轉移矩陣一致（`rtkpos.c:544` — `F[i+(i+6)*nx] = (tt>=0?1:-1)*SQR(tt)/2.0`）。**注意 RTKLIB 對負 `tt`（後向濾波）的符號處理，FGO 需一併鏡射。**

#### Jacobian

```
∂e/∂p_k     = -I₃
∂e/∂p_{k+1} = +I₃
∂e/∂v_k     = -Δt · I₃
∂e/∂a_k     = -0.5 · Δt² · I₃
```

全部為常數矩陣，無需回呼 RTKLIB。可用 `gtsam::BetweenFactor<Point3>` 的變體或自訂 factor。

#### 協方差

**必須在 ENU 座標系定義後旋轉至 ECEF**——GNSS 監測的水平與垂直精度差異約 2–3 倍，且變形的物理方向性也在 ENU 中才有意義。

```cpp
double Δt   = timediff(t_{k+1}, t_k);
double σ_h  = opt->prn[5];   /* pos process noise, 水平 (m/√s) */
double σ_v  = opt->prn[5] * fgo->pos_vratio;   /* 預設 vratio = 3.0 */

double Q_enu[9] = {0};
Q_enu[0] = Q_enu[4] = SQR(σ_h) * fabs(Δt);
Q_enu[8]           = SQR(σ_v) * fabs(Δt);

double pos[3], Q_ecef[9];
ecef2pos(p_k, pos);
covecef(pos, Q_enu, Q_ecef);      /* rtkcmn.c，與 udpos() 完全相同的旋轉 */
```

隨機遊走（random walk）假設下變異數正比於 `Δt`；標準差正比於 `√Δt`。

#### 監測應用的參數建議

| 場景 | `σ_h` (m/√s) | `σ_v` (m/√s) | 說明 |
|---|---|---|---|
| 建物沉陷（極穩定測站） | 1e-6 | 3e-6 | 幾乎完全剛性；日變形 < 1 mm |
| 邊坡（緩慢潛變） | 1e-5 | 3e-5 | 允許 mm/day 級的持續位移 |
| 橋梁（溫度效應 + 載重） | 1e-4 | 3e-4 | 需容納數 cm 的日週期擺動 |
| 橋梁（結構健康監測，高頻） | 1e-3 | 3e-3 | 需容納模態振動；需 ≥10 Hz 取樣 |

> 注意：過緊的 `σ_h/σ_v` 會使 FGO 對真實變形不敏感（把變形當作雜訊濾掉），這是監測應用最危險的誤用。建議在 AI Insight 中持續監控 Motion Factor 的正規化殘差（§10.4），若長期偏離 1，即代表過程雜訊設定與實際運動不符。

---

### 4.2.5 Velocity Factor（速度連續性 / 速度觀測）

分為三個子類型，各有不同來源與精度。

#### (a) Velocity Continuity Factor（連續性約束）

```
連接： V(k) ↔ V(k+1)
殘差： e = v_{k+1} - v_k - a_k · Δt            (dim 3)
協方差：Q_enu = diag(prn[3]², prn[3]², prn[4]²) · |Δt|  然後 covecef() 旋轉
```

`prn[3]` = `stats-prnaccelh`、`prn[4]` = `stats-prnaccelv`。**這與 `udpos()` (`rtkpos.c:566-571`) 完全相同**，確保兩引擎可比較。

#### (b) Doppler Velocity Factor（都卜勒速度觀測）

```
連接： V(k)、接收機鐘漂 D(k)
來源： resdop() / estvel()  (pntpos.c:549 / 598)
殘差： e = (v_sat - v_rcv)·e_los + c·(dts_drift - dtr_drift) - λ·D_observed
Jacobian： ∂e/∂v_rcv = -e_los ；∂e/∂(c·dtr_drift) = -1
協方差： σ_D ≈ 0.1–0.5 m/s（接收機相依）
```

都卜勒速度是**每 epoch 獨立的絕對速度觀測**，對 FGO 的價值在於它把速度變數固定住，防止「位置漂移被速度吸收」的退化模式。但精度僅 cm/s–dm/s 級，對 mm 級變形監測的直接貢獻有限。

#### (c) TDCP Factor（時間差分載波相位）— 監測應用的關鍵

Time-Differenced Carrier Phase 是**跨兩個 epoch 的相位差**，在無週波跳脫的前提下，未定值被完全消去：

```
連接： P(k) ↔ P(k+1)  （直接連接位置，不經速度）

殘差：
    e = [ρ(p_{k+1}, sat, t_{k+1}) - ρ(p_k, sat, t_k)]
        - λ·[L_{k+1} - L_k]
        - c·[dtr_{k+1} - dtr_k]
        + Δ(大氣項) + Δ(相位纏繞)

Jacobian：
    ∂e/∂p_k     = +e_los(k)
    ∂e/∂p_{k+1} = -e_los(k+1)

協方差：
    σ_TDCP ≈ √2 · σ_phase ≈ √2 × 3 mm ≈ 4 mm
    （雙差 TDCP 則為 2 × σ_phase）
```

**為什麼這對監測特別重要：**

1. **精度**：TDCP 的位置增量精度可達 mm 級，遠優於都卜勒的 cm/s 級。
2. **EKF 難以自然表達**：TDCP 本質上耦合兩個時刻的狀態。EKF 只能透過狀態增廣（把 `p_k` 也放進狀態向量）來處理，代價高且笨拙。**在 factor graph 中，這只是一條連接兩個節點的邊——這是 FGO 結構性優於 EKF 的最清楚例子。**
3. **對變形趨勢的敏感度**：TDCP 直接約束位移增量，因此對「緩慢但持續的變形」有累積敏感度，正是邊坡潛變與建物沉陷的訊號特徵。
4. **免疫於未定值誤差**：TDCP 不含 `N`，因此即使 AR 失敗（float 解），位移增量仍精確。這在都市峽谷、遮蔽嚴重的橋下等 AR 困難場景極有價值。

**實作要點**：TDCP factor 必須與該衛星該頻率的**弧段 ID 一致**（§4.1.3）。若 `arc_id(k) != arc_id(k+1)`，表示中間有 slip，該 TDCP factor **不可加入**。這個檢查是 TDCP 正確性的唯一關鍵，必須有單元測試覆蓋。

---

### 4.2.6 Acceleration Factor（加速度平滑約束）

#### 連接

`A(k)` ↔ `A(k+1)`。僅在 `dynamics == 2` 時啟用。

#### 殘差

```
e = a_{k+1} - a_k                                    (dim 3)
```

（一階隨機遊走。若需更平滑，可改為 jerk-limited 的二階形式，見下。）

#### Jacobian

```
∂e/∂a_k     = -I₃
∂e/∂a_{k+1} = +I₃
```

#### 協方差

**與 `udpos()` 逐項相同**（`rtkpos.c:566-571`）：

```c
Q_enu[0] = Q_enu[4] = SQR(opt->prn[3]) * fabs(Δt);   /* acch */
Q_enu[8]            = SQR(opt->prn[4]) * fabs(Δt);   /* accv */
ecef2pos(x, pos);
covecef(pos, Q_enu, Q_ecef);
```

#### 變體：Jerk-limited Smoothness

對於橋梁 SHM，若要抑制高頻雜訊而保留結構模態，可加入三點平滑 factor：

```
e = a_{k+1} - 2·a_k + a_{k-1}                       (dim 3, 離散二階差分 ≈ jerk·Δt)
σ_jerk = prn[3] / τ_smooth,  τ_smooth 為平滑時間常數（建議 5–30 s）
```

注意：此 factor 會**主動抑制真實的高頻結構響應**。僅適用於變形監測（低頻趨勢），**不可用於模態辨識**。設定項 `fgo-jerkconst` 預設為 off，並在文件與 GUI 中標註此風險。

---

### 4.2.7 Site Constraint Factor（測站先驗約束）— 監測應用專屬

這是本專案相對於通用 GNSS-FGO 的差異化價值：**把工程結構的物理先驗編碼進圖**。

RTKLIB 已有一個此類約束的先例：`constbl()` (`rtkpos.c:1145`)，以基線長度為約束，殘差 `v = opt->baseline[0] - |b|`、Jacobian `H[i] = b[i]/|b|`、協方差 `SQR(opt->baseline[1])`。Site Constraint Factor 是它的泛化。

#### 統一形式

```
e = W · (p_k - p_ref)  -  d_expected                (dim 依約束型別)
```

其中 `W` 為投影矩陣（把 ECEF 位移投影到約束子空間），`p_ref` 為測站參考基準（首日平均解或已知控制點）。

#### 子類型 A — Planar / Directional Constraint（橋梁）

橋梁的位移主要在**垂直**與**橫向（transverse）**，沿橋軸（longitudinal）方向受支承約束，位移小得多。

```
設 û_long 為橋軸單位向量（ENU 座標，由橋梁幾何給定）

e = û_longᵀ · R_enu · (p_k - p_ref)                 (dim 1)
∂e/∂p_k = û_longᵀ · R_enu                            (1×3)
σ_long: 依支承型式，固定支承端 2–5 mm；活動支承端 20–50 mm（需容納溫度伸縮）
```

**進階：溫度耦合。** 橋梁縱向位移與溫度呈近線性關係 `Δu_long ≈ α·L·ΔT`（`α ≈ 1.2e-5 /°C`，`L` 為伸縮縫間距）。若有橋面溫度感測器，可改為：

```
e = û_longᵀ·R_enu·(p_k - p_ref) - α·L·(T_k - T_ref)          (dim 1)
```

這是 Phase 4「Weather integration」的具體落地形式之一，也是 Digital Twin 的直接介面。

#### 子類型 B — Dip-Direction Constraint（邊坡）

邊坡位移主要沿**傾向（dip direction）**向下，垂直於滑動面的位移應接近零。

```
設 n̂ 為滑動面法向量（ENU），d̂ 為傾向單位向量

(B1) 面外約束（強）：
     e = n̂ᵀ · R_enu · (p_k - p_ref)                  (dim 1)
     σ_n ≈ 3–10 mm

(B2) 單向潛變先驗（軟，單邊）：
     s_k = d̂ᵀ · R_enu · (p_k - p_ref)                 沿傾向的位移量
     e = max(0, -s_k) / σ_creep                       僅懲罰「逆傾向」位移
     σ_creep ≈ 10 mm
```

(B2) 的單邊 hinge 形式編碼了「邊坡只會往下滑，不會自己爬回去」這一物理事實。GTSAM 中以自訂 `NoiseModelFactor` 加上不對稱 loss 實作。**這是 EKF 完全無法表達的約束型別**（EKF 的量測更新必須是線性高斯的），是 FGO 對邊坡監測的獨特價值。

注意：使用時機警示：若邊坡發生**回彈或膨脹**（例如降雨後的孔隙水壓變化、或凍融循環），(B2) 會產生系統性偏差。建議僅在已確認為單調潛變的邊坡上啟用，且必須在 AI Insight 中監控此 factor 的殘差是否持續為正（代表約束與實際不符）。

#### 子類型 C — Vertical-Dominant Constraint（建物沉陷）

建物沉陷以垂直為主，水平位移應接近零。

```
e = [ê_E; ê_N]ᵀ · R_enu · (p_k - p_ref)              (dim 2, 水平分量)
σ_h ≈ 2–5 mm     （垂直方向不加約束，讓資料說話）
```

#### 子類型 D — Multi-Station Rigid-Body Constraint（建物 / 橋梁多測站）

當同一結構上有 `M ≥ 3` 個測站時，它們的運動應近似為剛體平移 + 微小轉動：

```
連接： P_1(k), P_2(k), ..., P_M(k)

對每一對 (a,b)：
e_ab = |p_a(k) - p_b(k)| - L_ab_ref                  (dim 1，基線長度不變)
σ_L ≈ 1–3 mm

或完整剛體形式（引入額外變數 R(k) ∈ SO(3), t(k) ∈ R³）：
e_a = p_a(k) - [R(k)·p_a_ref + t(k)]                 (dim 3, 每測站)
```

完整剛體形式讓 AI Insight 可以直接輸出**結構的整體平移與旋轉（傾斜）**，這正是 Digital Twin 需要的量。同時，任何顯著偏離剛體假設的殘差即為**局部變形或損傷的指標**——這是結構健康監測的核心訊號。

> **這是本架構最強的監測價值主張**：多測站剛體約束在 EKF 中需要把所有測站放進同一個狀態向量（維度爆炸、耦合複雜）；在 factor graph 中只是加幾條邊。

#### 子類型 E — Datum / Prior Factor（基準錨定）

```
e = p_k - p_datum                                    (dim 3)
σ: 若 p_datum 為已知控制點，1–5 mm；若為首日平均解，10–50 mm
```

用於：(1) 消除 DD 表述的整體平移自由度；(2) 提供 batch FGO 的絕對基準；(3) 定期以外部量測（水準測量、全站儀）校正 GNSS 長期漂移。

#### Site Constraint 設定介面

建議以獨立設定檔提供（而非塞進 `prcopt_t`），格式草案見 Appendix B.3。

---

### 4.2.8 其他 Factor

| Factor | 用途 | 殘差 | 備註 |
|---|---|---|---|
| **Prior Factor** | 初始化第一個節點 | `e = X - X_0` | 位置用 SPP 解 + `VAR_POS`(`rtkpos.c:66`) |
| **Marginalization Factor** | 滑動視窗邊界 | GTSAM 自動產生 | 不可手動刪節點 |
| **Bias Random-Walk Factor** | 相位偏差在弧段內的緩慢變化 | `e = b_{k+1} - b_k` | `σ` 極小（1e-5 cycle/√s）；多數情況可省略（弧段內視為常數） |
| **Arc-Stitch Factor** | 縫合誤報的 cycle slip | `e = b_arcB - b_arcA` | 見 §4.1.3；`σ_stitch` 設為 0.1 cycle，僅在事後判定 slip 為誤報時加入 |
| **Trop Random-Walk Factor** | ZTD 時間變化 | `e = ztd_{k+1} - ztd_k` | `σ = prn[2]·√Δt`，與 `udtrop()` (`rtkpos.c:606`) 一致 |
| **Iono Random-Walk Factor** | STEC 時間變化 | `e = stec_{k+1} - stec_k` | `σ = prn[1]·√Δt`·(仰角相依)，與 `udion()` (`rtkpos.c:576`) 一致 |

---

## 4.3 Residual / Jacobian / Covariance 建立方式總表

| Factor | 維度 | 殘差來源 | Jacobian 來源 | 協方差來源 | 需回呼 RTKLIB？ |
|---|---|---|---|---|---|
| GNSS DD | `nb` | `ddres_core()` → `v` (取負) | `ddres_core()` → `H` 欄 | `varerr()`+`ddcov()` | 是 — 每次線性化 |
| GNSS PR (UD) | 1 | `rescode_core()` → `v` (取負) | `rescode_core()` → `H` 欄 | `varerr()` (pntpos) | 是 — 每次線性化 |
| GNSS CP (SD) | 1 | `zdres()` 差分 | 解析式（`-e_los`, `λ`） | `varerr()` (rtkpos) ×2 | 是 — 每次線性化 |
| TDCP | 1 | `zdres()` 於兩 epoch | 解析式 | `√2·σ_phase` | 是 — 每次線性化 |
| Motion | 3 | 解析式 | 常數 `±I₃`, `-Δt·I₃` | `prn[5]`+`covecef()` | 否 |
| Velocity (cont.) | 3 | 解析式 | 常數 | `prn[3,4]`+`covecef()` | 否 |
| Velocity (Doppler) | 1 | `resdop()` | 解析式 | 接收機規格 | 是 |
| Acceleration | 3 | 解析式 | 常數 `±I₃` | `prn[3,4]`+`covecef()` | 否 |
| Site Constraint | 1–3 | 解析式 | 常數（投影矩陣） | 設定檔 | 否 |
| Prior | 3 | 解析式 | `I₃` | `VAR_POS` 等 | 否 |

**觀察**：只有 GNSS 觀測類 factor 需要回呼，且回呼成本為 `O(n_sat)` 的浮點運算，而每次 GTSAM 迭代的 Cholesky 成本為 `O(n^1.5)` 以上。回呼**不會**成為效能瓶頸。這一點在 Phase 2 需以實測確認（§11.2 驗收準則）。

---

## 4.4 Ambiguity 處理與 LAMBDA 整合

### 4.4.1 整體策略

> **FGO 負責產生更好的浮點解與更可靠的協方差；LAMBDA 固定的邏輯完全沿用 RTKLIB。**

理由：`manage_amb_LAMBDA()` (`rtkpos.c:1909`) 及其呼叫的 `resamb_LAMBDA()` (`rtkpos.c:1770`)、`ddidx()` (`rtkpos.c:1563`)、`restamb()` (`rtkpos.c:1633`)、`holdamb()` (`rtkpos.c:1661`) 是本 fork 相對 upstream 的主要改進之一（partial AR、`arfilter`、`excsat` 輪替、動態 ratio 門檻），已在大量實地資料上驗證。重寫這部分是純粹的風險，沒有收益。

### 4.4.2 介面設計

FGO 求解後，將結果寫回 `rtk->x` 與 `rtk->P`，然後直接呼叫既有 AR pipeline：

```c
/* src/fgo/fgo_solver.cpp -> 透過 C ABI 回到 rtkpos.c */

/* 1. FGO 求解 */
fgo_optimize(fgo_ctx);

/* 2. 取出當前 epoch 的邊緣分布，寫回 RTKLIB 狀態格式 */
fgo_marginals_to_rtk(fgo_ctx, rtk->x, rtk->P, rtk->nx);
/*    - 位置 -> x[0..2]
 *    - SD 相位偏差（依 arc_map 反查 sat/freq）-> x[IB(s,f,opt)]
 *    - 對流層 -> x[IT(r,opt)] ；電離層 -> x[II(s,opt)]
 *    - P 為對應變數的 joint marginal covariance
 *      必須是 joint（含互相關），不可只取對角，否則 LAMBDA 的 Q 錯誤     */

/* 3. 沿用既有 AR */
stat = manage_amb_LAMBDA(rtk, bias, xa, sat, nf, ns);
```

**關鍵風險 R-AR1**：GTSAM 的 `Marginals::jointMarginalCovariance()` 對大量變數的聯合協方差計算成本高（需回代 Bayes tree）。若 `nb` 個相位偏差的聯合協方差每 epoch 都要算，可能成為瓶頸。緩解：
- 僅計算「參與本次 AR 的子集」的聯合協方差（`ddidx()` 已篩選過）。
- 使用 `gtsam::Marginals::Factorization::CHOLESKY`（較 QR 快）。
- Phase 2 需實測此步驟耗時，若 > 30% 總時間，考慮改用 iSAM2 的 `marginalCovariance()` 增量介面。

### 4.4.3 SD 偏差的秩虧與錨定

如 §2.3 所述，RTKLIB 的相位偏差狀態是**站間單差**，DD 由 `ddres()` 內部形成。這造成：

> 對任意常數 `c`，將某個 (系統, 頻率) 群組內所有 SD 偏差同時加上 `c`，所有 DD 殘差不變。

EKF 中這不致命，因為 `initx()` 給的有限初始變異數（`opt->std[0]`）本身就是一個隱含的先驗，正則化了問題。但在 factor graph 中，若不顯式加入此先驗，**資訊矩陣在該方向為奇異，Cholesky 分解失敗或產生數值垃圾**。

**必須採取的措施**（三選一，建議同時採用 1 與 3）：

1. **對每個 (系統, 頻率) 群組的參考衛星偏差加 Prior Factor**：
   `e = b_ref - b_ref_init`，`σ = opt->std[0]`（`stats-stdbias`，預設 30 m）。這精確重現 EKF 的隱含先驗，兩引擎行為一致。
2. **改用 DD 未定值變數**：直接以 DD 為變數，無秩虧。但需重寫 `ddidx()`/`restamb()` 的索引邏輯，違反 §4.4.1 策略。不建議。
3. **對所有偏差變數加極弱 Prior**（`σ = 1e4` m）：作為數值保險，成本可忽略。即使措施 1 因某些邊界情況失效，也能保證矩陣非奇異。

**驗收測試 T-AR1**：以人為構造的完美無雜訊 DD 資料，驗證加入錨定前 Cholesky 失敗、加入後成功且解正確。這個測試必須在 Phase 2 就建立。

### 4.4.4 FGO 對 AR 的實質增益

| 增益 | 機制 |
|---|---|
| 更小的浮點解變異數 → 更高 fix rate | 滑動視窗累積多 epoch 資訊，且 robust kernel 移除污染觀測 |
| 更可靠的協方差 → ratio test 更有意義 | EKF 的 `P` 常因過程雜訊調校不當而過度樂觀；FGO 的邊緣協方差來自完整非線性重估 |
| **可回溯修正錯誤的 fix** | EKF 一旦 `holdamb()` 錯誤地把整數固定住，錯誤會傳播且難以恢復。FGO 中可移除該 factor 並重解視窗 |
| Cycle slip 誤報的修復 | 弧段縫合 factor（§4.1.3） |

---

## 4.5 Solver 設計

### 4.5.1 三種 solver 模式

```c
/* src/fgo/fgo_config.h */
typedef enum {
    FGO_SOLVER_EKF          = 0,  /* 現行 RTKLIB EKF（預設，不進入 FGO 路徑）*/
    FGO_SOLVER_BATCH        = 1,  /* 全批次 Levenberg-Marquardt */
    FGO_SOLVER_SLIDING      = 2,  /* 固定視窗 batch smoother */
    FGO_SOLVER_ISAM2        = 3   /* iSAM2 增量 + fixed-lag */
} fgo_solver_t;
```

### 4.5.2 Batch FGO

```cpp
gtsam::LevenbergMarquardtParams p;
p.setMaxIterations(30);
p.setRelativeErrorTol(1e-6);
p.setAbsoluteErrorTol(1e-8);
p.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
p.setOrderingType("COLAMD");
gtsam::LevenbergMarquardtOptimizer opt(graph_, initial_, p);
gtsam::Values result = opt.optimize();
```

- 全部 epoch 一次解算。適用於**每日再處理**與**事件後鑑識分析**。
- 記憶體與時間隨 epoch 數成長。24 小時 @ 1 Hz = 86,400 epochs，需分段處理或降取樣（監測應用通常 30 s 取樣即足夠 → 2,880 epochs，完全可行）。

### 4.5.3 Sliding Window FGO

```cpp
gtsam::BatchFixedLagSmoother smoother(window_sec_);   /* 例如 300 s */
smoother.update(newFactors, newValues, newTimestamps);
gtsam::Values est = smoother.calculateEstimate();
```

- 每次 update 對整個視窗重解。視窗外的變數自動邊緣化為 linear factor。
- 計算量固定可預測；適合對延遲抖動敏感的 NRT。
- 視窗長度是精度/成本的主要旋鈕：`fgo-window`（預設 300 s）。

### 4.5.4 iSAM2

```cpp
gtsam::ISAM2Params p;
p.relinearizeThreshold = 0.05;      /* 變數變化超過此值才重線性化 */
p.relinearizeSkip      = 1;
p.factorization        = gtsam::ISAM2Params::CHOLESKY;
p.cacheLinearizedFactors = true;
gtsam::IncrementalFixedLagSmoother smoother(window_sec_, p);
```

- 只重線性化受影響的子樹，攤銷成本接近常數。
- **風險**：當有大更新（例如 AR 固定成功造成位置跳變）時，重線性化會級聯至整棵 Bayes tree，造成延遲尖峰。這對 NRT 是真實風險。
- **緩解**：(a) 監控每 epoch 耗時，超過門檻時降級為僅輸出預測解並標記低信心；(b) 在 §4.7 的雙執行緒設計中，FGO 延遲不阻塞資料接收。

### 4.5.5 Solver 選擇的執行期策略（建議）

```
NRT 主線 (rtksvr)         : FGO_SOLVER_ISAM2, window = 300 s
每日再處理 (排程作業)      : FGO_SOLVER_BATCH,  full day, robust = Cauchy→Tukey
事件觸發鑑識 (地震/暴雨後) : FGO_SOLVER_BATCH,  事件前後 ±2 h, 全 factor
基準/回歸                 : FGO_SOLVER_EKF
```

「NRT 快解 + 每日精解」的雙軌設計，是變形監測的標準作業模式，也是 FGO 架構最自然的落地方式：**同一份圖、同一套 factor，只換 solver。**

---

## 4.6 C ABI 介面設計

```c
/* src/fgo/rtklib_fgo_api.h
 * 純 C 標頭。由 src/*.c 與 src/fgo/*.cpp 共同引入。
 * 不得包含任何 C++ 或 GTSAM 內容。                                    */
#ifndef RTKLIB_FGO_API_H
#define RTKLIB_FGO_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtklib.h"

/* ---- 錯誤碼 ---- */
#define FGO_OK              0
#define FGO_ERR_DISABLED   -1   /* 建置時未啟用 FGO */
#define FGO_ERR_NOMEM      -2
#define FGO_ERR_SINGULAR   -3   /* Cholesky 失敗 */
#define FGO_ERR_NOTCONV    -4   /* 未收斂 */
#define FGO_ERR_INTERNAL   -5   /* C++ 例外已於邊界攔截 */
#define FGO_ERR_TIMEOUT    -6   /* 超過 NRT 延遲上限 */

/* ---- 生命週期 ---- */
EXPORT int  fgo_init (rtk_t *rtk, const prcopt_t *opt);   /* 綁定 rtk->fgo */
EXPORT void fgo_free (rtk_t *rtk);
EXPORT void fgo_reset(rtk_t *rtk);                        /* 清空圖，保留設定 */

/* ---- 每 epoch 主入口（由 rtkpos.c 呼叫）---- */
EXPORT int  fgo_process_epoch(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                              const nav_t *nav);

/* ---- 由 GTSAM factor 回呼進 RTKLIB 的殘差評估（於 rtkpos.c 實作）---- */
typedef struct fgo_dd_ctx_tag fgo_dd_ctx_t;   /* opaque，內容在 rtkpos.c */

EXPORT int  fgo_dd_ctx_create (fgo_dd_ctx_t **ctx, rtk_t *rtk,
                               const obsd_t *obs, int nu, int nr,
                               const nav_t *nav);
EXPORT void fgo_dd_ctx_destroy(fgo_dd_ctx_t *ctx);

/* 凍結 DD 配對（參考衛星選定後不再變動），供整個 epoch 的所有迭代使用 */
EXPORT int  fgo_dd_freeze_pairs(fgo_dd_ctx_t *ctx);

/* 核心回呼：以任意狀態向量 x 重新計算殘差與 Jacobian。
 * 純函式：不修改 ctx、不修改 rtk。可安全地被多執行緒同時呼叫（不同 ctx）。 */
EXPORT int  fgo_dd_eval(const fgo_dd_ctx_t *ctx, const double *x, int nx,
                        double *v, double *H, double *R, int *vflg);

/* 非差偽距版本（Phase 3） */
EXPORT int  fgo_pr_eval(const fgo_pr_ctx_t *ctx, const double *x, int nx,
                        double *v, double *H, double *R);

/* ---- 誤差模型（供 src/fgo/ 建構 noise model）---- */
EXPORT double fgo_obsvar_rtk(int sat, int sys, double el,
                             double snr_rover, double snr_base,
                             double bl, double dt, int f,
                             const prcopt_t *opt, const obsd_t *obs);
EXPORT double fgo_obsvar_spp(const prcopt_t *opt, const obsd_t *obs,
                             double el, int sys);

/* ---- 結果輸出 ---- */
EXPORT int  fgo_insight_json(const rtk_t *rtk, char *buff, int size);

#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_FGO_API_H */
```

**C++ 例外的邊界處理**（每個 `EXPORT` 函式的 C++ 實作）：

```cpp
extern "C" int fgo_process_epoch(rtk_t *rtk, const obsd_t *obs,
                                 int nu, int nr, const nav_t *nav)
{
    try {
        return fgo::Solver::from(rtk)->processEpoch(obs, nu, nr, nav);
    } catch (const std::bad_alloc&)     { trace(1,"fgo: oom\n");  return FGO_ERR_NOMEM; }
    catch (const std::exception& e)     { trace(1,"fgo: %s\n", e.what()); return FGO_ERR_INTERNAL; }
    catch (...)                         { trace(1,"fgo: unknown\n"); return FGO_ERR_INTERNAL; }
}
```

**絕對規則**：C++ 例外**不可**跨越 C ABI 邊界（未定義行為）。此 try/catch 樣板必須出現在每一個 `extern "C"` 函式中，並列入 code review checklist。

當 `ENABLE_FGO=OFF` 時，提供一份 stub 實作（`fgo_stub.c`，純 C），所有函式回傳 `FGO_ERR_DISABLED`。這樣 `rtkpos.c` 的呼叫點不需 `#ifdef`，程式碼更乾淨（滿足 I5 且不污染核心）。

---

## 4.7 執行緒與即時性設計

### 4.7.1 現況

`rtksvr.c:727` 的 `rtkpos()` 在 `rtksvrthread()` 內同步呼叫。若 FGO 耗時超過 epoch 間隔，會阻塞資料接收，造成串流緩衝溢位。

### 4.7.2 建議設計（Phase 3）

```
+---------------------+       lock-free ring        +----------------------+
|  rtksvr thread      |  ---- (obs, nav snapshot) -->  FGO worker thread   |
|                     |                              |                      |
|  - 串流讀取          |                              | - 建圖               |
|  - RTCM/RINEX 解碼   |                              | - iSAM2 update       |
|  - EKF 解 (永遠執行)  |  <--- (fgo_sol, insight) ---  | - 邊緣協方差         |
|  - 立即輸出 EKF 解    |                              | - AI Insight JSON    |
|  - FGO 解到達後補發   |                              |                      |
+---------------------+                              +----------------------+
```

**設計要點**：

1. **EKF 永遠同步執行並立即輸出**。這保證即使 FGO 完全失效，產線仍有解。這是 I1 的實務延伸，也是產品風險的最低保障。
2. FGO 在 worker thread 非同步產出，延遲數秒可接受（NRT 而非 hard real-time）。
3. FGO 解到達時，以「修正解（revised solution）」形式輸出，帶有明確的 `revision` 標記與延遲時間。下游 AI Insight 需能處理「同一 epoch 有兩個解」的情況。
4. 佇列滿時**丟棄最舊的**（監測應用中，最新資料最重要），並在 insight 中記錄丟棄率。
5. `nav_t` 需做快照（星曆會被主執行緒更新）。可用讀寫鎖或雙緩衝。

### 4.7.3 延遲預算（Phase 3 驗收目標）

| 項目 | 目標 | 說明 |
|---|---|---|
| EKF 路徑延遲 | 不變（< 20 ms） | I1 要求 |
| FGO iSAM2 每 epoch（p50） | < 200 ms | 30 s 取樣下有充裕餘裕 |
| FGO iSAM2 每 epoch（p99） | < 2 s | 涵蓋重線性化級聯 |
| FGO 解相對 epoch 時間的總延遲 | < 5 s | NRT 定義 |
| 佇列丟棄率 | < 0.1% | 連續 7 天測試 |

---

# 5. Error Model Design

## 5.1 設計原則

> **FGO 的誤差模型必須是 RTKLIB `varerr()` 的嚴格超集（superset）。** 在所有 robust kernel 關閉、所有加權選項設為現行值的組態下，FGO 使用的 `σ²` 必須與 EKF 逐位元相同。

理由：若兩引擎的誤差模型不同，任何精度差異都無法歸因——不知道是 FGO 的功勞還是誤差模型調校的功勞。這會使 Phase 2 的驗證失去意義。

實作上，`fgo_obsvar_rtk()` 與 `fgo_obsvar_spp()`（§4.6）就是 `varerr()` 的 `EXPORT` 版本，**同一份程式碼**，不是重新實作。

## 5.2 Measurement Noise Model — 基礎模型

### 5.2.1 RTK / DD 版本（`rtkpos.c:406-452`）

```
fact = (code ? eratio[frq] : eratio[frq]/eratio[0]) × EFACT_<sys>

a = fact · err[1]                     基礎項 (m)
b = fact · err[2]                     仰角項 (m)
c = err[3] · bl / 1e4                 基線項 (m)，bl 為基線長 (m)
d = CLIGHT · sclkstab · dt            衛星鐘穩定度項 (m)

var = 2·(a² + b²/sin²(el) + c²) + d²

若 err[6] > 0:   e_snr = fact · err[6]
                 var += e_snr² · [10^(0.1·max(err[5]-SNR_rover,0))
                                + 10^(0.1·max(err[5]-SNR_base ,0))]

若 err[7] > 0:   var += (err[7] · Pstd[frq])²          [code]
                 var += (err[7] · Lstd[frq] · 0.2)²    [phase]

若 IONOOPT_IFLC: var ×= 3²
```

**注意事項（FGO 實作必須遵守）**：

- 前置係數 `2.0` 已內含「兩台接收機的單差」效應。DD 的第二次差分則由 `ddcov()` 處理。**FGO 不可再乘 2**。
- `dt` 是基站與移動站觀測的時間差（差分年齡），對監測應用（基站通常穩定）通常很小，但在 NTRIP 斷線重連時會突增，此項會自動提高變異數——這是有用的自適應行為，必須保留。
- `EFACT_*` 是各系統的誤差係數常數（定義於 `rtklib_const.h`），FGO 直接繼承。

### 5.2.2 SPP 版本（`pntpos.c:51-76`）

```
var = fact² · { eratio[0]² · [ err[1]² + err[2]²/sin(el)
                             + err[6]²·10^(0.1·max(err[5]-SNR,0)) ]
              + (err[7]·Pstd[0])² }
若 IONOOPT_IFLC: var ×= 3²
```

**與 RTK 版的三項差異，實作時必須注意，勿混用**：
1. 仰角項為 `1/sin(el)`（RTK 版為 `1/sin²(el)`）
2. 無前置 `2.0`（非差，只有一台接收機）
3. 無基線項與衛星鐘穩定度項
4. `MIN_EL` 下限鉗制（`pntpos.c:65`）

## 5.3 Elevation-based Weighting

### 5.3.1 現行模型的行為

以預設值 `err[1]=0.003 m`（基礎）、`err[2]=0.003 m`（仰角）計算 RTK 相位的 `σ`（不含 SNR 與基線項）：

| 仰角 | `1/sin²(el)` | RTK 相位 σ (mm) | 相對 90° 的倍數 |
|---|---|---|---|
| 90° | 1.00 | 6.0 | 1.00× |
| 45° | 2.00 | 7.3 | 1.22× |
| 30° | 4.00 | 9.5 | 1.58× |
| 20° | 8.55 | 13.1 | 2.18× |
| 15° | 14.9 | 16.9 | 2.82× |
| 10° | 33.2 | 24.8 | 4.13× |
| 5° | 131.6 | 48.9 | 8.14× |

（`σ = sqrt(2·(a² + b²/sin²el))`，`a = b = 0.003 m`，`EFACT_GPS = 1`，忽略基線、鐘穩定度與 SNR 項。實際值依 `eratio`、系統與 SNR 而異。）

低仰角衛星的權重急遽下降，符合實際的大氣殘差與多路徑分布。

### 5.3.2 FGO 的仰角加權策略

**Phase 2**：完全沿用，不做任何修改。這是驗證 FGO 本身價值的必要控制條件。

**Phase 3 可選增強**（設定項 `fgo-elwmodel`）：

| 模型 | 公式 | 適用 |
|---|---|---|
| `rtklib`（預設） | 現行 `a² + b²/sin²(el)` | 相容基準 |
| `exp` | `a²·(1 + b·exp(-el/el0))²`，`el0 ≈ 10°` | 對極低仰角更保守；文獻中對都市環境表現較佳 |
| `sitemap` | 由該測站歷史殘差統計出的 az/el 網格加權（見 §5.4.3） | **監測應用最佳**：測站固定，可長期學習 |

`sitemap` 是監測應用相對於一般 GNSS 的獨特優勢：**測站不動，因此可以為每個測站學習專屬的方位-仰角誤差圖**。這在移動載體上是做不到的。

## 5.4 Multipath Mitigation

多路徑是變形監測中 mm 級精度的**主要限制因素**，且因為測站固定，多路徑具有高度可重複性——這既是問題（系統性偏差不會被平均掉）也是機會（可預測、可建模）。

分四層處理，由淺至深：

### 5.4.1 第一層：既有機制（Phase 2 直接沿用）

| 機制 | 位置 | 設定項 |
|---|---|---|
| SNR mask（依仰角的 SNR 門檻） | `pntpos.c:96` `snrmask()`；`snrmask_t` (`rtklib_types.h:464`) | `pos1-snrmask_r/_b/_L1/_L2/_L5/_L6` |
| SNR 加權項 | `varerr()` `err[5]`,`err[6]` | `stats-errsnrmax`, `stats-errsnr` |
| 接收機回報標準差加權 | `varerr()` `err[7]` × `obs->Pstd/Lstd` | `stats-errrcvstd` |
| 仰角遮罩 | `opt->elmin` | `pos1-elmask` |
| 幾何無關（GF）滑脫偵測 | `detslp_gf()` (`rtkpos.c:729`) | `pos2-slipthres` |
| 都卜勒滑脫偵測 | `detslp_dop()` (`rtkpos.c:758`) | `pos2-dopthres` |

### 5.4.2 第二層：Robust Kernel（Phase 2 新增）

見 §5.5。這是對**隨機、非重複性**多路徑（例如經過的車輛、臨時遮蔽）最有效的手段。

### 5.4.3 第三層：測站多路徑圖（Site Multipath Map，Phase 3）

**原理**：對固定測站，多路徑誤差是衛星方位角與仰角的函數 `m(az, el)`，且在數天內穩定。

**建立流程**：
1. 以 batch FGO 處理 N 天（建議 ≥ 7 天）資料，取得高精度位置解。
2. 對每個觀測量計算事後殘差 `r_i`。
3. 將 `r_i` 依 `(az, el)` 分箱（建議 2° × 2° 網格）。
4. 每格計算殘差的均值 `μ(az,el)` 與標準差 `s(az,el)`。
5. 套用時：`P_corrected = P_observed - μ(az,el)`；`σ² += s(az,el)²`。

**風險**：若在建圖期間測站已在變形，變形訊號會被錯誤地吸收進多路徑圖。**緩解**：建圖前先以 §10.3 的趨勢分析扣除線性趨勢；且多路徑圖需定期（每季）重建並比對前後差異。

### 5.4.4 第四層：Sidereal Filtering（Phase 3，選用）

**原理**：GPS 衛星軌道週期約為一個恆星日（23h 56m 04s），因此多路徑幾何在次日同一「恆星時」重現。將前一恆星日的殘差時間序列平移後扣除，可壓抑重複性多路徑。

**注意**：
- 各系統的重複週期不同：GPS ≈ 23h55m57s（實測值略有差異）、Galileo ≈ 10 恆星日 / 17 圈、BDS MEO ≈ 7 恆星日 / 13 圈、BDS GEO/IGSO 為靜地/傾斜同步（幾何幾乎不變，多路徑近乎恆定但也最難分離）。**必須逐系統處理**。
- 對監測應用，sidereal filtering 可能把「日週期的真實變形」（例如溫度造成的橋梁日擺動）一併濾掉。**必須先扣除已知的溫度模型再做 sidereal filtering**，否則會抹除真實訊號。這是嚴重誤用風險，需在文件與程式中明確警示。

> **建議**：第三層（測站多路徑圖）優先於第四層（sidereal filtering）。前者較穩健、較易解釋，且不會混淆日週期訊號。第四層列為 Phase 3 的選用實驗項。

### 5.4.5 Code-Minus-Carrier 監測（Phase 2 即可實作，低成本高價值）

MP 組合（雙頻）：

```
MP1 = P1 - (1 + 2/(γ-1))·L1 + (2/(γ-1))·L2,     γ = (f1/f2)²
```

MP1 在無週波跳脫的弧段內僅含**多路徑 + 雜訊 + 常數偏差**。逐弧段扣除均值後，其 RMS 即為該衛星該時段的多路徑強度指標。

**用法**：作為 `varerr()` 的自適應膨脹因子：

```
σ²_adaptive = σ²_varerr × max(1, (MP_rms / MP_nominal)²)
```

`MP_nominal` 為該測站的長期中位數。這讓誤差模型自動反應「今天這顆衛星特別髒」。

**這是低成本高價值的改進**：計算量可忽略（每個觀測量幾次浮點運算），不需外部資料，且直接輸出給 AI Insight 作為訊號品質指標（§10.4）。建議在 Phase 2 就實作。

## 5.5 Robust Kernel

### 5.5.1 三種 kernel 的比較

以正規化殘差 `u = r/σ` 表示。`ρ(u)` 為 loss，`w(u) = ρ'(u)/u` 為 IRLS 權重。

| Kernel | `ρ(u)` | 權重 `w(u)` | 調節常數 | 凸性 | Redescending |
|---|---|---|---|---|---|
| **L2** | `u²/2` | `1` | — | 凸 | 否 |
| **Huber** | `u²/2` (·`\|u\|≤δ`)；`δ(\|u\|-δ/2)` | `1` / `δ/\|u\|` | `δ = 1.345` | **凸** | 否 |
| **Cauchy** | `(c²/2)·ln(1+(u/c)²)` | `1/(1+(u/c)²)` | `c = 2.3849` | 非凸 | 是（漸近） |
| **Tukey** | `(c²/6)[1-(1-(u/c)²)³]` (·`\|u\|≤c`)；`c²/6` | `(1-(u/c)²)²` / `0` | `c = 4.6851` | 非凸 | **是（完全）** |

調節常數為對高斯分布達 95% 漸近效率的標準值。

### 5.5.2 行為差異（關鍵）

```
權重 w(u) 對正規化殘差 u：

 u:      0     1     2     3     5     10    20
L2:     1.00  1.00  1.00  1.00  1.00  1.00  1.00   <- 離群值完全污染解
Huber:  1.00  1.00  0.67  0.45  0.27  0.13  0.07   <- 下降但永不歸零
Cauchy: 1.00  0.85  0.59  0.39  0.19  0.05  0.01   <- 快速下降，漸近歸零
Tukey:  1.00  0.91  0.66  0.32  0.01  0.00  0.00   <- c=4.685 以外完全歸零
```

- **Huber 是非 redescending 的**：即使 `u = 100`，權重仍為 `δ/100 ≈ 0.013`，仍會拉扯解。對「多重嚴重離群值」（如都市峽谷的 NLOS）不夠。
- **Tukey 完全歸零**：`|u| > c` 的觀測量對梯度貢獻為零。這是最強的離群值抑制，但也意味著**若初始估計太差，正確的觀測量可能被誤判為離群值而永久排除，最佳化陷入錯誤的局部極小值**。
- **Cauchy 居中**：權重快速下降但永不精確為零，保留了「若解移動過來，這個觀測量還能回來」的可能性。

### 5.5.3 推薦策略：Graduated Non-Convexity（GNC）

> **不要直接用 Tukey 作為第一輪。** 使用逐步收緊的多輪最佳化。

```
Pass 1: L2                → 取得初始解（或直接用 EKF 解作為初值，更好）
Pass 2: Huber (δ=1.345)   → 凸，保證收斂，壓制中度離群值
Pass 3: Cauchy (c=2.3849) → 進一步壓制
Pass 4: Tukey (c=4.6851)  → 僅離線 batch；NRT 不做
```

實作上，GTSAM 的 `noiseModel::Robust::Create()` 可在每輪重建 factor 的 noise model，或使用 GTSAM 的 `GncOptimizer`（4.2+ 提供，自動化 GNC 排程）。

### 5.5.4 各 Solver 模式的建議組態

| Solver | Kernel 策略 | 理由 |
|---|---|---|
| `EKF` | 無（沿用 `maxinno` 硬拒絕） | I1 要求不變 |
| `SLIDING` (NRT) | Huber → Cauchy（2 輪） | 凸起步保證收斂；延遲可控 |
| `ISAM2` (NRT) | Huber only | iSAM2 的增量特性與非凸 kernel 交互作用複雜；保守起見僅用凸 kernel |
| `BATCH` (離線) | GNC 全序列至 Tukey | 無延遲限制，追求最高精度 |

### 5.5.5 與 RTKLIB 既有硬拒絕機制的關係

RTKLIB 現有兩層硬拒絕：
- `ddres()` 中的 `maxinno[0]`（相位）/ `maxinno[1]`（code）逐觀測量門檻。
- `valpos()` (`rtkpos.c:2039`) 的事後驗證。

Robust kernel 是這兩者的**平滑推廣**（硬門檻等價於一個 0/1 的 kernel）。

**建議組合**：
1. **保留硬門檻，但放寬**：FGO 路徑下將 `maxinno` 提高至現值的 3 倍，僅用於攔截「明顯錯誤」（如解碼錯誤造成的 1000 m 殘差），避免這類極端值破壞 robust kernel 的尺度估計。設定項 `fgo-maxinno-scale`（預設 3.0）。
2. **細粒度加權交給 robust kernel**。
3. **不要兩者都設緊**——那等於硬拒絕主導，robust kernel 無用武之地。

### 5.5.6 尺度估計（Scale Estimation）— 常被忽略的關鍵

Robust kernel 的調節常數是相對於 `σ` 定義的。若 `varerr()` 給出的 `σ` 本身系統性偏小（過度樂觀），則所有殘差的 `u` 都偏大，robust kernel 會把**正常觀測量誤判為離群值**，導致解退化。

**必須實作的自適應尺度估計**：每 epoch（或每 N epoch）以 MAD（Median Absolute Deviation）估計實際尺度：

```
s = 1.4826 × median(|r_i - median(r)|)          /* 1.4826 使 MAD 對高斯分布無偏 */
scale_factor = clamp(s / median(σ_i), 0.5, 5.0)
σ_effective = σ_varerr × scale_factor
```

`clamp` 上下限防止在觀測量過少（`n < 8`）或全部被污染時的失控。並將 `scale_factor` 輸出至 AI Insight——**持續偏離 1.0 是誤差模型調校不當的直接證據**。

## 5.6 白化與相關性

### 5.6.1 白化（Whitening）

GTSAM 內部對每個 factor 做 `Σ^{-1/2}` 白化：`ẽ = L⁻¹e`、`Ã = L⁻¹A`，其中 `Σ = LLᵀ`。

- `noiseModel::Isotropic::Sigma(dim, σ)` → `L⁻¹ = I/σ`，最快。
- `noiseModel::Diagonal::Sigmas(v)` → 逐元素除法。
- `noiseModel::Gaussian::Covariance(Σ)` → 完整 Cholesky（DD block 用此）。

### 5.6.2 Robust kernel 與相關性協方差的不相容

> **`noiseModel::Robust` 的 `ρ` 作用在白化後殘差的範數上。若 base model 是完整協方差（相關），白化會把不同觀測量的誤差混合，使得「哪一顆衛星是離群值」的資訊被抹除——robust kernel 只能整塊接受或整塊降權。**

三種解法，對應 §4.2.1 的三種協方差處理：

| 解法 | Robust 粒度 | 統計正確性 | 建議階段 |
|---|---|---|---|
| (a) Block Gaussian + 區塊層級卡方檢定 | 整個區塊 | 是 — 完全正確 | **Phase 2** |
| (b) 對角近似 + 逐 DD robust kernel | 逐 DD | 注意 — 協方差低估（需膨脹因子 ~1.4） | 可選 |
| (c) 顯式 `ε_ref` 潛在變數 + 逐 DD robust | 逐 DD | 是 — 完全正確 | **Phase 3** |
| (d) 改用 SD 表述（§4.2.3） | 逐衛星 | 是 — 完全正確 | **Phase 3 推薦** |

**Phase 2 的 (a) 具體作法**：對每個 DD 區塊計算 Mahalanobis 距離

```
d² = eᵀ · R_b⁻¹ · e     ~  χ²(nb)  在虛無假設下
```

若 `d² > chisqr[nb]`（表已存在：`rtklib_api.h:8` `EXPORT extern const double chisqr[]`，α=0.001），則對整個區塊降權或以留一法（leave-one-out）逐一嘗試移除，找出貢獻最大的 DD。留一法成本為 `O(nb)` 次 rank-1 更新，可接受。

## 5.7 參數預設值總表

| 參數 | 設定項 | 預設 | 監測建議 | 說明 |
|---|---|---|---|---|
| 相位基礎誤差 | `stats-errphase` (`err[1]`) | 0.003 m | 0.003 | 沿用 |
| 相位仰角誤差 | `stats-errphaseel` (`err[2]`) | 0.003 m | 0.003 | 沿用 |
| 相位基線誤差 | `stats-errphasebl` (`err[3]`) | 0 m/10km | 依基線長 | 短基線監測可為 0 |
| SNR 上限 | `stats-errsnrmax` (`err[5]`) | 52 dBHz | 依接收機 | 需依實測 SNR 分布調整 |
| SNR 誤差項 | `stats-errsnr` (`err[6]`) | 0 | **0.003** | 建議啟用，多路徑抑制有效 |
| 接收機標準差項 | `stats-errrcvstd` (`err[7]`) | 0 | **1.0** | 若接收機回報 Pstd/Lstd 則啟用 |
| Code/Phase 比 | `stats-eratio1/2/3` | 300 | 100–300 | 高品質測地型接收機可降至 100 |
| 位置過程雜訊 | `stats-prnpos` (`prn[5]`) | — | 見 §4.2.4 表 | **監測應用最關鍵的旋鈕** |
| 加速度過程雜訊(H) | `stats-prnaccelh` (`prn[3]`) | 1.0 m/s²/√s | 1e-4 – 1e-3 | 準靜態測站應極小 |
| 加速度過程雜訊(V) | `stats-prnaccelv` (`prn[4]`) | 0.1 m/s²/√s | 3e-4 – 3e-3 | 通常為 H 的 3 倍 |
| — 以下為 FGO 新增 — | | | | |
| Robust kernel | `fgo-robust` | `huber` | `huber`(NRT) / `gnc`(batch) | §5.5.4 |
| Huber δ | `fgo-huber-delta` | 1.345 | 1.345 | 標準值 |
| Cauchy c | `fgo-cauchy-c` | 2.3849 | 2.3849 | 標準值 |
| Tukey c | `fgo-tukey-c` | 4.6851 | 4.6851 | 標準值 |
| 尺度估計 | `fgo-scale-est` | `mad` | `mad` | §5.5.6 |
| 尺度上下限 | `fgo-scale-clamp` | 0.5,5.0 | 0.5,5.0 | 防失控 |
| 硬門檻放寬倍數 | `fgo-maxinno-scale` | 3.0 | 3.0 | §5.5.5 |
| DD 協方差模式 | `fgo-ddcov` | `block` | `block` | §5.6.2 |
| 仰角加權模型 | `fgo-elwmodel` | `rtklib` | `rtklib`→`sitemap` | §5.3.2 |
| MP 自適應加權 | `fgo-mp-adaptive` | `off` | **`on`** | §5.4.5 |

---

# 6. RTKLIB Modification Plan

> **總覽**：需修改 5 個 `.c`、3 個 `.h`、2 個 `CMakeLists.txt`。約 **85%** 的改動為「純函式化重構 + 提升可見性」，屬於行為不變的機械性修改；僅 `rtkpos.c` 的 solver 分支與 `options.c` 的選項新增是真正的功能新增。

修改分類與風險等級定義：

| 類別 | 定義 | 風險 |
|---|---|---|
| **T1 純函式化** | 抽出無副作用核心，原函式改為薄包裝 | 中（觸碰核心，但有 byte-diff 回歸保護） |
| **T2 可見性提升** | `static` → `EXPORT`，簽章不變 | 低 |
| **T3 新增分支** | 新增 `if` 分支，預設不進入 | 低 |
| **T4 結構欄位新增** | struct 尾端新增欄位 | 中（ABI 破壞，見 §6.10） |
| **T5 新增檔案** | 全新檔案，不影響既有 | 極低 |

---

## 6.1 `src/rtkpos.c`（2,549 行）— 主要修改對象

### M1 — 新增 solver 派發分支 【T3】

**位置**：`rtkpos()` (`rtkpos.c:2438`)，在 `pntpos()` 呼叫之後、`relpos()` 呼叫之前。

```c
    /* Relative positioning */
    if (opt->fgo_solver != FGO_SOLVER_EKF) {
        int ret = fgo_process_epoch(rtk, obs, nu, nr, nav);
        if (ret != FGO_OK) {
            errmsg(rtk, "fgo error (%d), falling back to ekf\n", ret);
            relpos(rtk, obs, nu, nr, nav);        /* 降級：永遠有解 */
        }
    } else {
        relpos(rtk, obs, nu, nr, nav);            /* 原路徑，完全不變 */
    }
    rtk->epoch++;
    outsolstat(rtk, nav);
```

**影響範圍**：`rtkpos()` 一個函式，約 +10 行。
**相容性風險**：極低。`opt->fgo_solver` 預設 `FGO_SOLVER_EKF`，所有既有設定檔與 GUI 皆走原路徑。
**降級設計**：FGO 失敗時自動 fallback 至 EKF，保證產線不中斷（這與 §4.7.2 的「EKF 永遠執行」策略在 Phase 3 會合併為非同步設計）。

### M2 — `zdres()` 提升可見性 【T2】

**位置**：`rtkpos.c:1049`。

```c
/* 由 static 改為 EXPORT，簽章完全不變 */
EXPORT int zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const double *var, const int *svh,
                 const nav_t *nav, const double *rr, const prcopt_t *opt,
                 double *y, double *e, double *azel, double *freq);
```

**分析**：`zdres()` 已是實質純函式——輸入 `rr`，輸出 `y`/`e`/`azel`/`freq`，唯一的外部相依是 `opt->pcvr`、`opt->odisp`、`nav->erp`，皆為唯讀。**不需重構，只需提升可見性並在 `rtklib_api.h` 加宣告。**

**影響範圍**：`rtkpos.c` 1 行 + `rtklib_api.h` 1 行。
**相容性風險**：極低。唯一風險是符號名稱 `zdres` 過於通用，可能與其他函式庫衝突。**緩解**：改名為 `rtk_zdres()` 並在 `rtkpos.c` 內以 `#define zdres rtk_zdres` 保持內部呼叫不變。建議採用。

### M3 — `ddres()` 純函式化 【T1】— 最高風險修改

**位置**：`rtkpos.c:1240-1507`（268 行）。

**現況的副作用清單**（重構前必須完整盤點）：

| 副作用 | 行號 | 處理方式 |
|---|---|---|
| 清空 `rtk->ssat[i].resp/resc` | 1263-1265 | 移至包裝函式 |
| 寫入 `rtk->ssat[].resp[frq]` / `resc[frq]` | ~1420 | 輸出至 `ddres_stat_t` |
| 寫入 `rtk->ssat[].vsat[frq]` | ~1425 | 同上 |
| 讀取 `rtk->opt` | 全域 | 改由 `ctx` 傳入 `const prcopt_t*` |
| 讀取 `rtk->rb[]` | 1256 | 改由 `ctx` 傳入 |
| 讀取 `rtk->sol.time` | 1276 | 改由 `ctx` 傳入 |
| 讀取 `rtk->nx` | H 的 stride | 改由參數傳入 |
| 讀取 `rtk->ssat[].sys/slip/lock/snr_*` | 多處 | 改由 `ctx` 傳入 `const ssat_t*` |
| 動態選擇參考衛星 | 1285-1330 | **新增「凍結配對」模式**（見下） |
| `errmsg(rtk, ...)` | 多處 | 改為回傳錯誤碼 + 由包裝函式輸出 |

**重構後結構**：

```c
/* 新增：DD 評估上下文（唯讀輸入的集合）*/
typedef struct {
    const prcopt_t *opt;
    const obsd_t   *obs;
    const ssat_t   *ssat;        /* 唯讀快照 */
    const double   *rb;          /* 基站座標 */
    gtime_t         soltime;
    double          dt;          /* 差分年齡 */
    const double   *y, *e, *azel, *freq;
    const int      *sat, *iu, *ir;
    int             ns, nx;
    /* 凍結的 DD 配對表；NULL 表示動態選擇（EKF 模式）*/
    const int      *frozen_ref;  /* [m][f] -> 參考衛星在 sat[] 中的索引，-1 表無 */
} ddres_ctx_t;

/* 輸出的衛星狀態（原本直接寫入 rtk->ssat）*/
typedef struct {
    double resp[MAXSAT][NFREQ];
    double resc[MAXSAT][NFREQ];
    uint8_t vsat[MAXSAT][NFREQ];
    int     ref_idx[NSYS][NFREQ*2];   /* 本次選中的參考衛星，供凍結使用 */
} ddres_stat_t;

/* 純函式核心：無任何副作用，可重入 */
EXPORT int ddres_core(const ddres_ctx_t *ctx, const double *x, const double *P,
                      double *v, double *H, double *R, int *vflg,
                      ddres_stat_t *st);

/* 薄包裝：維持原簽章與原行為（I1）*/
static int ddres(rtk_t *rtk, const obsd_t *obs, double dt, const double *x,
                 const double *P, const int *sat, double *y, double *e,
                 double *azel, double *freq, const int *iu, const int *ir,
                 int ns, double *v, double *H, double *R, int *vflg)
{
    ddres_ctx_t ctx; ddres_stat_t st;
    int i, j, nv;

    fill_ddres_ctx(&ctx, rtk, obs, dt, sat, y, e, azel, freq, iu, ir, ns);
    ctx.frozen_ref = NULL;                    /* EKF 用動態選擇，行為不變 */
    memset(&st, 0, sizeof(st));

    nv = ddres_core(&ctx, x, P, v, H, R, vflg, &st);

    /* 將 st 寫回 rtk->ssat，重現原副作用 */
    for (i = 0; i < MAXSAT; i++) for (j = 0; j < NFREQ; j++) {
        rtk->ssat[i].resp[j] = st.resp[i][j];
        rtk->ssat[i].resc[j] = st.resc[i][j];
        rtk->ssat[i].vsat[j] = st.vsat[i][j];
    }
    return nv;
}
```

**「凍結配對」模式的必要性**：如 §4.2.1 所述，若參考衛星在迭代中改變，factor 維度會漂移。`frozen_ref != NULL` 時，`ddres_core()` 跳過 `rtkpos.c:1285-1330` 的最小變異數搜尋，直接使用給定的參考衛星。這是 `ddres_core()` **唯一的行為新增**，且在 `frozen_ref == NULL`（EKF 路徑）時完全不觸發。

**影響範圍**：`rtkpos.c` 約 ±300 行（重構，非新增邏輯）。
**相容性風險**：**本計畫最高風險項**。緩解措施：
1. 此重構獨立成一個 PR，**不含任何 FGO 程式碼**，可單獨審查與回退。
2. 強制 byte-diff 回歸（§6.11）在合併前通過。
3. `ddres_stat_t` 為堆疊上的大結構（`MAXSAT × NFREQ × 8 × 2 ≈ 9 KB`）。若堆疊空間受限（嵌入式），改為由呼叫端提供緩衝區。需檢查目標平台的 thread stack size（`rtksvr.c` 建立執行緒時未指定 stack size，使用系統預設，Linux 通常 8 MB，安全）。
4. 加入單元測試 `test/utest/t_ddres.c`：以固定輸入驗證 `ddres_core()` 與重構前 `ddres()` 的輸出逐位元相同。

### M4 — `varerr()` 提升可見性 【T2】

**位置**：`rtkpos.c:406`。改為 `EXPORT double rtk_varerr(...)`，簽章不變。`rtkpos.c` 內部以 `#define varerr rtk_varerr` 保持呼叫點不變。

**相容性風險**：極低。

### M5 — `ddcov()` 提升可見性 【T2】

**位置**：`rtkpos.c:1128`。改為 `EXPORT void rtk_ddcov(...)`。FGO 需直接呼叫以建構 block covariance。

### M6 — 新增 FGO 結果寫回輔助函式 【T5/T3】

新增一個 `EXPORT` 函式，供 `src/fgo/` 把 FGO 解寫回 `rtk_t`：

```c
EXPORT int rtk_set_fgo_solution(rtk_t *rtk, const double *x, const double *P,
                                int nx, int stat, int ns, double ratio);
```

集中所有「寫回 `rtk->sol`」的邏輯（座標、協方差 `qr[]` 的 ECEF→上三角打包、`sol.stat`、`sol.ns`、`sol.age`、`sol.ratio`），避免 `src/fgo/` 直接操作 `rtk_t` 內部欄位。

### M7 — `constbl()` 提升可見性 【T2】（選用）

`rtkpos.c:1145`。Site Constraint Factor 可參考其實作；若要直接重用基線長度約束，需提升可見性。優先度低，Phase 3 再議。

### `rtkpos.c` 修改總計

| 項目 | 類別 | 估計行數 | 風險 |
|---|---|---|---|
| M1 solver 分支 | T3 | +10 | 低 |
| M2 `zdres` 可見性 | T2 | ±3 | 低 |
| M3 `ddres` 純函式化 | T1 | ±300 | **中高（本計畫最高）** |
| M4 `varerr` 可見性 | T2 | ±3 | 低 |
| M5 `ddcov` 可見性 | T2 | ±3 | 低 |
| M6 結果寫回 | T5 | +60 | 低 |
| M7 `constbl` 可見性 | T2 | ±3 | 低 |
| **合計** | — | **約 ±380 行** | **中高** |

---

## 6.2 `src/ppp.c`（1,318 行）

> **Phase 2–3 不修改。Phase 4 才動。**

### 規劃中的修改（Phase 4）

**M8 — `ppp_res()` 純函式化** 【T1】（`ppp.c:974`，注意：非需求文件所稱的 `resph()`）

與 M3 同型的重構。`ppp_res()` 的副作用更多：寫入 `rtk->ssat[].vsat/resp/resc/snr`、`rtk->ssat[].rejc`、內部有離群值拒絕迴圈（`ve[]`/`obsi[]`/`frqi[]` 與 `vmax` 邏輯，`ppp.c:1090` 附近）會**改變 `exc[]` 並重新呼叫自己**。

**重要判斷**：`ppp_res()` 的離群值拒絕迴圈在 FGO 中應由 robust kernel 取代，因此純函式化時應**拆出兩層**：
- `ppp_res_core()`：單純計算殘差/Jacobian/協方差，無拒絕邏輯。
- `ppp_res()`：`core` + 拒絕迴圈 + 副作用（EKF 用，行為不變）。

**M9 — `pppos()` 加 solver 分支** 【T3】（`ppp.c:1226`）

與 M1 同型。

**相容性風險**：中。`ppp.c` 近期有活躍開發（`d220b8a0`、`4b376f51`、`505d12cc` 三個 commit 涉及 phase-OSB loader 與 `IONOOPT_QZS` 處理）。Phase 4 動工前需與該工作線協調，避免合併衝突。**建議 Phase 4 開工前先確認 `ppp.c` 的 OSB/est-stec 工作已穩定落地。**

**Phase 2–3 的唯一動作**：不改程式碼，僅在 `docs/` 記錄 `ppp_res()` 的副作用盤點（作為 Phase 4 的前置分析）。

---

## 6.3 `src/pntpos.c`（715 行）

### M10 — `varerr()` 提升可見性 【T2】

`pntpos.c:51` → `EXPORT double spp_varerr(...)`。供非差 PR factor 使用。

### M11 — `rescode()` 純函式化 【T1】

`pntpos.c:277-370`。副作用較 `ddres()` 少：寫入 `vsat[]`、`resp[]`、`azel[]`（皆為呼叫端提供的陣列，本已參數化）。實際上 `rescode()` **已經接近純函式**——它接受 `x` 作為輸入、輸出 `v`/`H`/`var`/`azel`/`vsat`/`resp`。

主要工作只是：(a) 提升可見性；(b) 確認 `opt` 相依皆為唯讀；(c) 加入 `EXPORT` 宣告。

**影響範圍**：約 ±20 行。
**相容性風險**：低。

### M12 — `resdop()` 提升可見性 【T2】（選用）

`pntpos.c:549`。供 Doppler Velocity Factor 使用。優先度中。

### `pntpos.c` 修改總計：約 ±40 行，風險 低。

---

## 6.4 `src/options.c`（578 行）

### M13 — 新增 FGO 選項至 `sysopts[]` 【T3】

在 `sysopts[]` 表（`options.c:67` 起）新增 FGO 區段。**關鍵設計決策 D4：不擴充 `PMODE_*`，而是新增獨立的 `pos1-solver`。**

理由：`PMODE_*`（`rtklib_const.h:314-323`）的語意是「定位模式」（single/dgps/kinematic/static/moving-base/fixed/ppp-*），描述的是**觀測組合與測站運動假設**。Solver 是**正交的維度**——「static + FGO」與「kinematic + FGO」都應合法。若把 FGO 塞進 `PMODE_*`，會造成組合爆炸（`PMODE_FGO_STATIC`、`PMODE_FGO_KINEMA`...），且破壞所有既有的 `opt->mode <= PMODE_DGPS`、`opt->mode >= PMODE_PPP_KINEMA` 之類的比較邏輯（這類比較在 `rtkpos.c`、`ppp.c`、`postpos.c` 中散布數十處）。**這是一個必須避免的設計錯誤。**

```c
/* options.c，新增字串常數 */
#define SLVOPT  "0:ekf,1:fgo-batch,2:fgo-sliding,3:fgo-isam2"
#define RBSTOPT "0:none,1:huber,2:cauchy,3:tukey,4:gnc"
#define DDCVOPT "0:block,1:diag,2:latent"
#define ELWOPT  "0:rtklib,1:exp,2:sitemap"

/* sysopts[] 新增條目 */
    {"pos1-solver",       3, (void *)&prcopt_.fgo_solver,   SLVOPT },
    {"fgo-window",        1, (void *)&prcopt_.fgo_window,   "s"    },
    {"fgo-maxiter",       0, (void *)&prcopt_.fgo_maxiter,  ""     },
    {"fgo-robust",        3, (void *)&prcopt_.fgo_robust,   RBSTOPT},
    {"fgo-huber-delta",   1, (void *)&prcopt_.fgo_kparam[0],""     },
    {"fgo-cauchy-c",      1, (void *)&prcopt_.fgo_kparam[1],""     },
    {"fgo-tukey-c",       1, (void *)&prcopt_.fgo_kparam[2],""     },
    {"fgo-ddcov",         3, (void *)&prcopt_.fgo_ddcov,    DDCVOPT},
    {"fgo-elwmodel",      3, (void *)&prcopt_.fgo_elwmodel, ELWOPT },
    {"fgo-mp-adaptive",   3, (void *)&prcopt_.fgo_mpadapt,  SWTOPT },
    {"fgo-scale-est",     3, (void *)&prcopt_.fgo_scaleest, SWTOPT },
    {"fgo-maxinno-scale", 1, (void *)&prcopt_.fgo_innoscale,""     },
    {"fgo-tdcp",          3, (void *)&prcopt_.fgo_tdcp,     SWTOPT },
    {"fgo-siteconst",     2, (void *)fgo_sitefile_,         "file" },
    {"fgo-insight-out",   2, (void *)fgo_insightfile_,      "file" },
```

（完整清單見 Appendix B.1。）

### M14 — `resetsysopts()` 補上 FGO 預設值 【T3】

`options.c` 的 `resetsysopts()` 需為新欄位設定預設。**若遺漏，讀取舊設定檔時 FGO 欄位為未初始化值，可能意外啟用 FGO——這是必須避免的嚴重 bug。** 加入回歸測試：讀取一份不含任何 `fgo-*` 條目的舊設定檔，驗證 `fgo_solver == FGO_SOLVER_EKF`。

**影響範圍**：`options.c` 約 +30 行。
**相容性風險**：低，但 M14 是必要的防護。舊設定檔缺少 `fgo-*` 條目時，`loadopts()` 會保留 `resetsysopts()` 的值，因此預設值正確即可向下相容。新設定檔含 `fgo-*` 條目而被舊版程式讀取時，`searchopt()` 找不到該名稱會忽略——向上相容也成立。

---

## 6.5 `src/rtkcmn.c`（4,301 行）

> **修改極少。這是好消息——`rtkcmn.c` 是所有模組的共用基礎，改動風險最高。**

### M15 — 新增數值輔助函式 【T5】

FGO 需要少數 `rtkcmn.c` 尚未提供的工具：

```c
/* 對稱正定矩陣的 Cholesky 分解（DD block covariance 白化用）*/
EXPORT int  chol(const double *A, int n, double *L);

/* 對角 + rank-1 矩陣的反矩陣（Sherman-Morrison，§4.2.1）*/
EXPORT int  dpr1inv(const double *d, double sigma_ref2, int n, double *Ainv);

/* MAD 尺度估計（§5.5.6）*/
EXPORT double madscale(const double *v, int n);
```

**注意**：`rtkcmn.c` 已有兩套 `matinv()` 實作（`rtkcmn.c:1164` 與 `:1386`，分別對應內建與 LAPACK 路徑，以 `#ifdef` 切換）。新增函式必須遵循同樣的雙路徑慣例，否則在某一種建置組態下會缺符號。**這是容易踩到的陷阱，需在 code review checklist 中列明。**

### M16 — `filter()` 不修改

明確記錄：**`filter()` (`rtkcmn.c:1479`) 不做任何修改**。FGO 走完全獨立的求解路徑。這保證 EKF 的數值行為 100% 不受影響。

**影響範圍**：約 +120 行（純新增函式）。
**相容性風險**：極低（純新增）。唯一風險是新符號名稱衝突；`chol` 較通用，建議命名為 `rtk_chol`。

---

## 6.6 `src/rtklib_types.h`（812 行）【T4】

### M17 — `prcopt_t` 新增 FGO 欄位

**必須加在 struct 尾端**（`rtklib_types.h:532`，`char pppopt[256];` 之後），以最小化 ABI 影響：

```c
typedef struct {        /* processing options type */
    ...
    char pppopt[256];   /* ppp option */
    /* ---- FGO extension (added 2026-08) ---- */
    int    fgo_solver;      /* solver (FGO_SOLVER_???) */
    int    fgo_robust;      /* robust kernel type */
    int    fgo_ddcov;       /* DD covariance mode */
    int    fgo_elwmodel;    /* elevation weighting model */
    int    fgo_maxiter;     /* max optimizer iterations */
    int    fgo_tdcp;        /* enable TDCP factors */
    int    fgo_mpadapt;     /* enable MP-adaptive weighting */
    int    fgo_scaleest;    /* enable MAD scale estimation */
    double fgo_window;      /* sliding window length (s) */
    double fgo_kparam[4];   /* kernel params {huber_d, cauchy_c, tukey_c, rsv} */
    double fgo_innoscale;   /* maxinno relaxation factor */
    double fgo_scaleclamp[2]; /* MAD scale clamp {min,max} */
    char   fgo_sitefile[MAXSTRPATH];    /* site constraint config file */
    char   fgo_insightfile[MAXSTRPATH]; /* AI insight output path */
} prcopt_t;
```

### M18 — `rtk_t` 新增 FGO 上下文指標

```c
typedef struct {        /* RTK control/result type */
    ...
    void *solstat;      /* solution-status output context (statout_t*) */
    void *fgo;          /* FGO context (fgo::Solver*), bound by fgo_init() */
} rtk_t;
```

**遵循本 fork 既有慣例**（`solstat` 欄位，`rtklib_types.h:672`）：opaque `void*`，由 `rtkinit()`/`rtkfree()` 週期管理，C 端不需知道其內部結構。

### M19 — `ddres_ctx_t` / `ddres_stat_t` / `fgo_dd_ctx_t` 型別定義

置於 `rtklib_types.h` 尾端。

**ABI 風險分析（重要）**：

`prcopt_t` 與 `rtk_t` 是**跨模組**結構，被 `src/`、`app/consapp/`、`app/qtapp/`、`app/winapp/` 共同使用，且 `librtklib.so` 是 **shared library**（`src/CMakeLists.txt:38` — `add_library(rtklib SHARED ...)`）。改變 struct 大小會**破壞 ABI**：

| 情境 | 風險 | 緩解 |
|---|---|---|
| 全專案一起重編 | 無風險 | CMake 的相依追蹤會自動全部重編 |
| 使用者以舊版 `librtklib.so` 搭配新版 app（或反之） | 記憶體損毀 | 提升 `VER_RTKLIB`/`PATCH_LEVEL`；`install(TARGETS rtklib ...)` 加上 SOVERSION |
| 第三方程式碼直接 `sizeof(prcopt_t)` | 中 | Release note 明確標示 ABI break |
| 序列化的 `prcopt_t`（若有） | 中 | 確認無二進位序列化；設定檔為文字格式（`options.c`），安全 |

**行動項**：在同一個 PR 中提升 `rtklib.h` 的 `PATCH_LEVEL`（頂層 `CMakeLists.txt:5-9` 會讀取），並在 release note 標註 ABI 變更。

---

## 6.7 `src/rtklib_const.h`（500 行）【T3】

新增 FGO 常數：

```c
/* FGO solver types */
#define FGO_SOLVER_EKF      0
#define FGO_SOLVER_BATCH    1
#define FGO_SOLVER_SLIDING  2
#define FGO_SOLVER_ISAM2    3

/* FGO robust kernel types */
#define FGO_ROBUST_NONE     0
#define FGO_ROBUST_HUBER    1
#define FGO_ROBUST_CAUCHY   2
#define FGO_ROBUST_TUKEY    3
#define FGO_ROBUST_GNC      4

/* FGO DD covariance modes */
#define FGO_DDCOV_BLOCK     0
#define FGO_DDCOV_DIAG      1
#define FGO_DDCOV_LATENT    2

/* FGO defaults */
#define FGO_DEF_WINDOW      300.0   /* sliding window (s) */
#define FGO_DEF_MAXITER     30
#define FGO_DEF_HUBER_D     1.345
#define FGO_DEF_CAUCHY_C    2.3849
#define FGO_DEF_TUKEY_C     4.6851
#define FGO_DEF_INNOSCALE   3.0
```

**相容性風險**：極低（純新增宏）。

---

## 6.8 `src/rtklib_api.h`（552 行）【T2】

新增被提升可見性之函式的宣告：

```c
/* FGO support: exposed residual/error-model functions ------------------------*/
EXPORT int    rtk_zdres(int base, const obsd_t *obs, int n, const double *rs,
                        const double *dts, const double *var, const int *svh,
                        const nav_t *nav, const double *rr, const prcopt_t *opt,
                        double *y, double *e, double *azel, double *freq);
EXPORT int    ddres_core(const ddres_ctx_t *ctx, const double *x, const double *P,
                         double *v, double *H, double *R, int *vflg,
                         ddres_stat_t *st);
EXPORT void   rtk_ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                        int nv, double *R);
EXPORT double rtk_varerr(int sat, int sys, double el, double snr_rover,
                         double snr_base, double bl, double dt, int f,
                         const prcopt_t *opt, const obsd_t *obs);
EXPORT double spp_varerr(const prcopt_t *opt, const obsd_t *obs, double el, int sys);
EXPORT int    rtk_set_fgo_solution(rtk_t *rtk, const double *x, const double *P,
                                   int nx, int stat, int ns, double ratio);
EXPORT int    rtk_chol(const double *A, int n, double *L);
EXPORT int    dpr1inv(const double *d, double sigma_ref2, int n, double *Ainv);
EXPORT double madscale(const double *v, int n);
```

---

## 6.9 建置系統

### M20 — `src/CMakeLists.txt` 【T3】

現況 `src/CMakeLists.txt:26` 使用 `aux_source_directory(. DIR_SRCS_RTKLIB)`，**只會收集 `src/*.c`，不會遞迴進 `src/fgo/`**。需顯式加入：

```cmake
option(ENABLE_FGO "Build Factor Graph Optimization solver (requires GTSAM)" OFF)

aux_source_directory(. DIR_SRCS_RTKLIB)
# ... 既有 DIR_SRCS_RTKLIB_RCV ...

if(ENABLE_FGO)
    find_package(GTSAM 4.2 REQUIRED)
    find_package(Eigen3 REQUIRED)   # 不可寫 3.3，見下方陷阱 5
    set(DIR_SRCS_FGO
        fgo/fgo_solver.cpp
        fgo/fgo_graph.cpp
        fgo/fgo_factor.cpp
        fgo/fgo_gtsam.cpp
        fgo/fgo_insight.cpp
    )
    list(APPEND DIR_SRCS ${DIR_SRCS_FGO})
    add_definitions(-DENABLE_FGO)
else()
    list(APPEND DIR_SRCS fgo/fgo_stub.c)   # 純 C stub，回傳 FGO_ERR_DISABLED
endif()

list(APPEND DIR_SRCS ${DIR_SRCS_RTKLIB} ${DIR_SRCS_RTKLIB_RCV})
add_library(rtklib SHARED ${DIR_SRCS} rtklib.h)

set_property(TARGET rtklib PROPERTY C_STANDARD 99)
if(ENABLE_FGO)
    set_property(TARGET rtklib PROPERTY CXX_STANDARD 17)
    set_property(TARGET rtklib PROPERTY CXX_STANDARD_REQUIRED ON)
    target_link_libraries(rtklib gtsam gtsam_unstable)
    target_include_directories(rtklib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/fgo)
endif()
```

**陷阱 1**：`src/CMakeLists.txt:11` 有

```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable ...")   # <- 注意：此行覆蓋而非附加
```

第二行**沒有** `${CMAKE_C_FLAGS}`，會覆蓋第一行（包括 `-O3`）。這是既有的疑似 bug，不在本專案範圍內，但**不可因為加入 C++ 而意外「修正」它**——那會改變 C 程式碼的最佳化等級，破壞 I1 的 byte-diff 驗證。**必須保持原樣，並在 PR 說明中註記。**

**陷阱 2（已驗證）**：上述 C flags **實際上從未生效**。`src/CMakeLists.txt:10` 與 `:16` 的條件為 `if (GCC)`，但變數 `GCC` 在整個專案的任何 `CMakeLists.txt` 中皆**未被 `set()`**（已以 `grep -rn "GCC" --include=CMakeLists.txt .` 確認，只有這兩處使用、零處定義）。CMake 沒有內建的 `GCC` 變數，因此該條件恆為假，`-std=c99 -Wall -O3 -ansi -pedantic` 與 Release 的 `-O3 -fno-signed-zeros ...` 皆未套用。實際使用的是 CMake 對 `CMAKE_BUILD_TYPE` 的預設 flags。

**這對本專案有兩個直接後果**：
1. **byte-diff 基準必須建立在實際生效的 flags 上**。建立基準前先以 `make VERBOSE=1` 或 `cmake -LAH` 記錄真實的編譯命令，並將其存入基準資料集的 metadata。
2. **不可順手「修正」這個 bug**。修正它會改變最佳化等級與浮點語意（特別是 `-fno-signed-zeros -fno-math-errno`），使所有既有基準失效，且會把「FGO 專案」與「編譯設定變更」兩件事的影響混在一起，無法歸因。**若要修正，應為獨立的 PR，且在 FGO 專案開始前或結束後進行。** 已列為 §14 的 OQ-5。

**陷阱 3**：`add_library(rtklib SHARED ...)` 混合 C 與 C++ 物件檔時，CMake 會自動改用 C++ linker（因為有 CXX 來源）。這會連結 `libstdc++`。對純 C 的下游使用者（`app/consapp/`）無影響，但**靜態連結情境下需注意**。

**陷阱 5（已驗證，2026-08-21）**：`find_package(Eigen3 3.3 REQUIRED)` 會**拒絕**現行安裝的 Eigen。`Eigen3ConfigVersion.cmake` 採 same-major-version 語意，指定 3.3 只接受 3.x；conda-forge 現行版本為 5.0.1，configure 會直接失敗（`The version found is not compatible with the version requested`）。應如 GTSAM 自身的做法寫成 `find_dependency(Eigen3 REQUIRED)` 不加版本——GTSAM 是對著現場的 Eigen 編譯的，與 GTSAM 一致才是重點，自訂下限反而有害。

**陷阱 4（已驗證，2026-08-20）**：`IncrementalFixedLagSmoother` 與 `BatchFixedLagSmoother` 位於 **`gtsam_unstable`**，不在核心 `gtsam` 中（標頭為 `gtsam_unstable/nonlinear/*FixedLagSmoother.h`）。上方 `target_link_libraries` 已據此更正為 `gtsam gtsam_unstable`。§8.4 推薦給 NRT 的兩種 solver（SLIDING / ISAM2）皆依賴這兩個類別，若只連結 `gtsam` 會在連結期失敗。**另一層意義**：`gtsam_unstable` 不保證跨版本 API 穩定，這使 RC-10 的版本鎖定成為硬性需求而非建議。詳見 `docs/fgo/build_environment.md`。

### M21 — 頂層 `CMakeLists.txt` 【T3】

- `project(... LANGUAGES C CXX ...)` **已存在**（`CMakeLists.txt:12`），無需修改。
- 加入 `option(ENABLE_FGO ...)` 的頂層轉發（或直接在 `src/` 定義）。
- `test/CMakeLists.txt` 新增 FGO 單元測試子目錄（`ENABLE_FGO` 時才加入）。

---

## 6.10 相容性風險總表

| 風險 | 描述 | 機率 | 衝擊 | 緩解 | 負責階段 |
|---|---|---|---|---|---|
| **RC-1** | `ddres()` 重構改變 EKF 數值行為 | 中 | 高 | byte-diff 回歸 + 獨立 PR + `t_ddres.c` 單元測試 | Phase 3 |
| **RC-2** | `prcopt_t`/`rtk_t` ABI 破壞 | **高（必然發生）** | 中 | 欄位加在尾端 + 提升 PATCH_LEVEL + release note + SOVERSION | Phase 3 |
| **RC-3** | `resetsysopts()` 遺漏 FGO 預設值 → 舊設定檔意外啟用 FGO | 中 | 高 | M14 + 專門的回歸測試 | Phase 3 |
| **RC-4** | 符號名稱衝突（`zdres`/`varerr`/`chol`） | 中 | 中 | 一律加 `rtk_`/`spp_` 前綴 | Phase 3 |
| **RC-5** | C++ linker 引入 `libstdc++` 影響部署 | 低 | 低 | `ENABLE_FGO=OFF` 預設；提供靜態連結建置選項 | Phase 3 |
| **RC-6** | `rtkcmn.c` 新函式未實作雙路徑（LAPACK/內建） | 中 | 中 | Code review checklist；CI 測試兩種組態 | Phase 3 |
| **RC-7** | `ppp.c` 修改與進行中的 OSB/est-stec 工作衝突 | 中 | 中 | Phase 4 開工前確認該工作已落地 | Phase 4 |
| **RC-8** | `ddres_stat_t` 堆疊使用量（~9 KB） | 低 | 低 | 檢查 thread stack size；必要時改為呼叫端配置 | Phase 3 |
| **RC-9** | C++ 例外跨 C ABI 邊界 | 中 | 高 | 每個 `extern "C"` 函式強制 try/catch 樣板 + review checklist | Phase 3 |
| **RC-10** | GTSAM 版本相依（API 在 4.0→4.2 有變動） | 中 | 中 | `find_package(GTSAM 4.2 REQUIRED)` 鎖定最低版本；Docker 固定版本 | Phase 2 |

## 6.11 相容性驗證策略

### 強制關卡 G1 — Byte-Diff 回歸（每個 PR）

**已實作**：`test/regression/run_regression.sh`（見 `test/regression/README.md`）。

```bash
test/regression/run_regression.sh            # 驗證全部資料集
test/regression/run_regression.sh --update   # 重新產生基準（僅在變更為預期時）
```

**原草案的兩項更正（已驗證）**：

1. **`.stat` 由 `-y` 產生，不是 `-x`**。`rnx2rtkp` 的 `-x <level>` 設定的是 trace 等級，產生 `.trace` 檔；solution status 檔來自 `-y <level>`，且檔名為 `<outfile>.stat`（例如 `-o out.pos -y 2` 產生 `out.pos.stat`）。原草案的 `cmp out_new2.stat` 會比對到不存在的檔案。
2. **`.pos` 不能直接 `cmp`**。其標頭含輸入檔的**絕對路徑**與程式版本號，兩者皆非數值性質。必須先正規化（路徑取 basename、版本號遮蔽）再比對。特別是 §6.6 M19 會在新增 FGO 欄位的同一個 PR 中提升 `PATCH_LEVEL`，若不遮蔽版本號將使全部基準無故失效。`.stat` 無標頭，可逐位元直接比對。

**敏感度（實測）**：將 `varerr()`（`rtkpos.c`）的回傳值擾動 **1e-12 相對量**，`solution.pos` **完全不變**，但 `solution.stat` 改變並被關卡攔截。這證實了「`.stat` 較敏感」的判斷，也是 M3（`ddres()` 純函式化）所需的驗證強度。

**注意**：浮點運算對編譯器最佳化等級敏感。建立基準時必須固定 compiler 版本、最佳化等級與 `-ffast-math` 相關 flags（見 §6.9 陷阱 1、2）。基準檔應與 compiler 版本一起記錄——`run_regression.sh` 已將其寫入各資料集的 `baseline/metadata.txt`。

### 強制關卡 G2 — 建置矩陣（CI）

| 組態 | C compiler | 必須通過 |
|---|---|---|
| `ENABLE_FGO=OFF` (預設) | gcc / clang | 建置 + 全部 utest + G1 |
| `ENABLE_FGO=ON` | gcc / clang + GTSAM 4.2 | 建置 + 全部 utest + G1 + FGO utest |
| `ENABLE_FGO=OFF`，純 C 編譯檢查 | `gcc -std=c99 -Wall` 逐檔編譯 `src/*.c` | 無 C++ 洩漏（驗證 I2） |

### 強制關卡 G3 — 設定檔向下相容

**已實作**：`test/options/check_options.sh`（測試本體 `test/options/opts_compat.c`）。

以 `data/config/f9p_ppk.conf`（真實的 FGO 前設定檔，不含任何 `fgo-*` 條目）驗證：

1. `loadopts()` 不報錯。
2. `prcopt_.fgo_solver == FGO_SOLVER_EKF`。
3. 全部 FGO 預設值與 Appendix B.1 相符。
4. 明確設定 `fgo-*` 時確實生效。
5. `saveopts()` 輸出的新設定檔可被再次讀入且語意不變。
6. **未知選項名稱被忽略而非拒絕**——這是「新設定檔可被舊版程式讀取」的前提（向上相容）。

**預設值必須寫在 `prcopt_default`（`rtkcmn.c`）而非 `resetsysopts()`**。原草案的 M14 只提到後者，但 `resetsysopts()` 僅執行 `prcopt_=prcopt_default;`，而 `rnx2rtkp` 等呼叫端是直接複製 `prcopt_default`——只寫在 `resetsysopts()` 的預設值對它們而言會是 0。已以負向測試確認：移除 `prcopt_default` 中的 `fgo_window`/`fgo_maxiter` 預設後，本關卡失敗（`window=0.0 maxiter=0`）。

### 強制關卡 G4 — 單元測試（新增至 `test/utest/`）

| 測試檔 | 驗證內容 |
|---|---|
| `t_ddres.c` | `ddres_core()` 與重構前 `ddres()` 輸出逐位元相同（固定輸入向量） |
| `t_fgo_factor.cpp` | 各 factor 的 Jacobian 對數值微分的一致性（GTSAM 有 `numericalDerivative` 工具） |
| `t_fgo_anchor.cpp` | §4.4.3 的秩虧測試：未錨定時 Cholesky 失敗，錨定後成功 |
| `t_fgo_tdcp.cpp` | 弧段不連續時 TDCP factor 正確地不被加入 |
| `t_fgo_robust.cpp` | 各 kernel 的 `ρ`/`w` 函式數值正確性；GNC 排程收斂性 |
| `t_fgo_ddcov.cpp` | `dpr1inv()` 對照 `matinv()` 的一致性 |

**Jacobian 數值驗證是最重要的一項**——解析 Jacobian 寫錯是 FGO 最常見且最難察覺的 bug（會導致收斂變慢或錯誤收斂，但不會崩潰）。GTSAM 的 `gtsam::numericalDerivative11/21/31` 應用於每一個自訂 factor。

---

# 7. 新增模組設計 `src/fgo/`

## 7.1 檔案結構

```
src/fgo/
├── rtklib_fgo_api.h     [C]     C ABI 介面（唯一被 src/*.c 引入的標頭）
├── fgo_config.h         [C++]   內部設定、Key schema、型別
├── fgo_stub.c           [C]     ENABLE_FGO=OFF 時的 stub 實作
├── fgo_solver.cpp       [C++]   生命週期、排程、solver 派發
├── fgo_graph.cpp        [C++]   圖與變數管理、弧段追蹤、邊緣化
├── fgo_factor.cpp       [C++]   所有 factor 的實作
├── fgo_gtsam.cpp        [C++]   GTSAM 後端封裝（隔離所有 GTSAM API）
├── fgo_insight.cpp      [C++]   AI Insight 輸出與異常指標
└── CMakeLists.txt               （若採 add_subdirectory 方式）
```

## 7.2 各檔案責任

### `rtklib_fgo_api.h` — C ABI 邊界

| 項目 | 內容 |
|---|---|
| **責任** | 定義 `src/*.c` 與 `src/fgo/*.cpp` 之間的唯一契約 |
| **語言** | 純 C（含 `extern "C"` guard） |
| **不得包含** | 任何 C++ 語法、GTSAM/Eigen/Boost 標頭、C++ 型別 |
| **內容** | 錯誤碼、生命週期函式、`fgo_process_epoch()`、回呼型別、誤差模型函式、insight 輸出 |
| **穩定性要求** | **這是最需要穩定的介面**。變更需同步更新 C 與 C++ 兩側，且會觸發全專案重編 |
| **完整草案** | §4.6 |

### `fgo_config.h` — 內部設定與型別

| 項目 | 內容 |
|---|---|
| **責任** | FGO 內部使用的設定結構、GTSAM Key schema、列舉、常數 |
| **語言** | C++17 |
| **內容** | `fgo::Config`（從 `prcopt_t` 轉換而來的內部設定）；`keyPos()`/`keyVel()`/`keyBias()` 等 Key 函式；`fgo::ArcId`、`fgo::EpochId` 型別；編譯期常數 |
| **設計要點** | 把 `prcopt_t` 的扁平欄位轉換為結構化的 C++ 設定物件，讓其餘檔案不直接依賴 `prcopt_t` 的欄位命名。這降低了 §6.6 ABI 變更的擴散範圍 |

```cpp
// fgo_config.h（示意）
namespace fgo {

struct RobustConfig {
    int    type      = FGO_ROBUST_HUBER;
    double huberD    = FGO_DEF_HUBER_D;
    double cauchyC   = FGO_DEF_CAUCHY_C;
    double tukeyC    = FGO_DEF_TUKEY_C;
    bool   scaleEst  = true;
    double scaleMin  = 0.5, scaleMax = 5.0;
};

struct Config {
    int          solver      = FGO_SOLVER_ISAM2;
    double       windowSec   = FGO_DEF_WINDOW;
    int          maxIter     = FGO_DEF_MAXITER;
    int          ddCovMode   = FGO_DDCOV_BLOCK;
    bool         enableTdcp  = true;
    bool         mpAdaptive  = false;
    RobustConfig robust;
    std::string  siteFile, insightFile;

    static Config fromPrcopt(const prcopt_t& opt);   /* 唯一的轉換點 */
};

} // namespace fgo
```

### `fgo_stub.c` — 停用時的替身

| 項目 | 內容 |
|---|---|
| **責任** | `ENABLE_FGO=OFF` 時提供所有 C ABI 函式的空實作，全部回傳 `FGO_ERR_DISABLED` |
| **語言** | 純 C |
| **價值** | 讓 `rtkpos.c` 的呼叫點**不需要 `#ifdef`**，核心程式碼保持乾淨；同時滿足 I5 |
| **行數** | < 50 |

### `fgo_solver.cpp` — 生命週期與排程

| 項目 | 內容 |
|---|---|
| **責任** | FGO 的「主控制器」。管理 `fgo::Solver` 物件的生命週期；接收每 epoch 的呼叫；協調前處理→建圖→最佳化→輸出的流程；solver 模式派發；Phase 3 的 worker thread 管理 |
| **不負責** | 不直接呼叫 GTSAM API（透過 `fgo_gtsam.cpp`）；不建構 factor（透過 `fgo_factor.cpp`）；不管理圖結構（透過 `fgo_graph.cpp`） |
| **關鍵 API** | `Solver::init()`, `Solver::processEpoch()`, `Solver::reset()`, `Solver::from(rtk_t*)` |
| **狀態** | 持有 `Config`、`Graph`、`GtsamBackend`、`InsightExporter`、epoch 計數、`fgo_dd_ctx_t*` 的生命週期 |

```cpp
// fgo_solver.cpp（流程骨架）
int Solver::processEpoch(const obsd_t* obs, int nu, int nr, const nav_t* nav)
{
    /* 1. 前處理：呼叫 RTKLIB 取得與線性化點無關的量 */
    auto ctx = DdContext::create(rtk_, obs, nu, nr, nav);   /* satposs, selsat */
    ctx->freezePairs();                                     /* 凍結參考衛星 */

    /* 2. 更新弧段對照（cycle slip -> 新變數）*/
    graph_.updateArcs(rtk_->ssat, epoch_);

    /* 3. 建圖：加入本 epoch 的所有 factor 與初值 */
    graph_.addGnssFactors  (ctx, epoch_);
    graph_.addMotionFactors(epoch_, rtk_->tt);
    graph_.addTdcpFactors  (ctx, epoch_);
    graph_.addSiteFactors  (epoch_);
    graph_.addInitialValues(epoch_, rtk_->sol.rr);

    /* 4. 最佳化 */
    auto result = backend_.optimize(graph_, cfg_);
    if (!result.ok) return FGO_ERR_NOTCONV;

    /* 5. 寫回 RTKLIB 狀態 + AR */
    backend_.marginalsToRtk(result, rtk_->x, rtk_->P, rtk_->nx);
    /*    manage_amb_LAMBDA() 由 rtkpos.c 端呼叫 */

    /* 6. AI Insight */
    insight_.emit(result, epoch_);

    return FGO_OK;
}
```

### `fgo_graph.cpp` — 圖與變數管理

| 項目 | 內容 |
|---|---|
| **責任** | 維護 factor graph 的結構與變數初值。弧段（arc）追蹤與 cycle slip → 新變數的轉換（§4.1.3）。滑動視窗的時間戳管理與邊緣化排程。變數 ↔ RTKLIB 狀態索引的雙向對照 |
| **關鍵資料結構** | `NonlinearFactorGraph`（新增待送出的 factor）；`Values`（初值）；`arc_map_[MAXSAT][NFREQ] -> ArcId`；`timestamps_`（fixed-lag 用）；`key_to_rtkidx_` 對照表 |
| **關鍵 API** | `updateArcs()`, `addGnssFactors()`, `addMotionFactors()`, `addTdcpFactors()`, `addSiteFactors()`, `addInitialValues()`, `takeNewFactors()` |
| **設計要點** | 圖的**增量交付**：每 epoch 只把「新增的」factor 與 values 交給 iSAM2，不重送舊的。這是 iSAM2 效能的前提 |

### `fgo_factor.cpp` — Factor 實作

| 項目 | 內容 |
|---|---|
| **責任** | 實作 §4.2 的所有 factor 類別。GNSS 類 factor 負責呼叫 C ABI 回呼並做符號/佈局轉換（RTKLIB `v`/`H` ↔ GTSAM `error`/`Jacobian`）。幾何類 factor（Motion/Velocity/Accel/Site）以解析式實作 |
| **類別清單** | `GnssDDFactor`, `GnssPRFactor`, `GnssCPFactor`, `TdcpFactor`, `MotionFactor`, `VelocityContinuityFactor`, `DopplerFactor`, `AccelSmoothFactor`, `SiteDirectionalFactor`, `SiteDipConstraintFactor`, `SiteRigidBodyFactor`, `ArcStitchFactor` |
| **設計要點** | 全部繼承 `gtsam::NoiseModelFactorN`；每個都必須有對應的 `numericalDerivative` 單元測試（§6.11 G4） |
| **效能要點** | `evaluateError()` 是熱路徑，每次線性化每個 factor 呼叫一次。避免在其中做記憶體配置——緩衝區（`v_buf_`, `H_buf_`）在建構時預先配置 |

### `fgo_gtsam.cpp` — GTSAM 後端封裝

| 項目 | 內容 |
|---|---|
| **責任** | **隔離所有 GTSAM API 的呼叫**。建立與設定 optimizer（LM / BatchFixedLagSmoother / IncrementalFixedLagSmoother）。執行最佳化。計算邊緣協方差。noise model 的建構（含 robust kernel 包裝與 block covariance） |
| **設計價值** | 若未來要換後端（Ceres、g2o、自建），理論上只需重寫此檔。實務上 factor 定義也會受影響，但隔離仍大幅降低耦合 |
| **關鍵 API** | `optimize()`, `marginalsToRtk()`, `makeNoiseModel()`, `makeRobustModel()`, `getJointMarginal()` |
| **關鍵實作** | §4.5 的三種 solver 設定；§5.5 的 GNC 排程；§4.4.2 的邊緣協方差擷取 |

### `fgo_insight.cpp` — AI Insight 輸出

| 項目 | 內容 |
|---|---|
| **責任** | 從最佳化結果萃取 §10 定義的所有量（position/velocity/acceleration/deformation rate/confidence/anomaly flags），序列化為 JSON，寫入檔案或串流 |
| **內部狀態** | 用於趨勢估計的滑動歷史緩衝區（位置時間序列）；統計累積器（NIS/NEES、robust 降權比例、MP 指標） |
| **設計要點** | 不使用外部 JSON 函式庫（避免新增相依），以簡單的手寫序列化實作。Schema 見 §10.2 |

## 7.3 相依方向與封裝

```
        rtklib_fgo_api.h  <-- 唯一的 C/C++ 邊界
              ^
              |  (實作)
        fgo_solver.cpp
         /    |     \
        /     |      \
 fgo_graph  fgo_factor  fgo_insight
   .cpp       .cpp        .cpp
        \     |      /
         \    |     /
        fgo_gtsam.cpp  <-- 唯一接觸 GTSAM API 的檔案
              |
            GTSAM
```

**規則**：
- 只有 `fgo_gtsam.cpp` 可 `#include <gtsam/...>` 中的 optimizer/marginals 標頭。
- `fgo_factor.cpp` 需要 `gtsam::NoiseModelFactor` 基底類別，這是必要的例外（factor 必須繼承 GTSAM 型別）。
- `fgo_solver.cpp`、`fgo_graph.cpp`、`fgo_insight.cpp` **不得**直接建立 optimizer。

## 7.4 程式碼規模估計

| 檔案 | 估計行數 | 難度 |
|---|---|---|
| `rtklib_fgo_api.h` | 150 | 低 |
| `fgo_config.h` | 200 | 低 |
| `fgo_stub.c` | 50 | 極低 |
| `fgo_solver.cpp` | 600 | 中 |
| `fgo_graph.cpp` | 800 | 中高（弧段管理與邊緣化） |
| `fgo_factor.cpp` | 1,200 | **高**（Jacobian 正確性） |
| `fgo_gtsam.cpp` | 700 | 中高（GTSAM API 細節） |
| `fgo_insight.cpp` | 500 | 中 |
| 單元測試 | 800 | 中 |
| **合計** | **約 5,000 行** | |

對照：架構 B（Native FGO）估計 8,000–15,000 行，且集中在最高難度的數值線性代數。**A′ 的 5,000 行中，最難的部分（sparse Cholesky、Bayes tree）為零。**

---

# 8. Solver 評估

## 8.1 記號與假設

| 符號 | 意義 | 監測應用典型值 |
|---|---|---|
| `n` | 單 epoch 的狀態維度 | 60–120（`filter()` 壓縮後的實際維度） |
| `m` | 單 epoch 的觀測數（DD 個數） | 30–60 |
| `N` | 總 epoch 數 | 2,880（24 h @ 30 s）／86,400（24 h @ 1 s） |
| `W` | 滑動視窗 epoch 數 | 10（300 s @ 30 s）／300（300 s @ 1 s） |
| `s` | 每 epoch 的「持久」狀態維度（位置+速度+加速度+對流層） | 3–11 |

**取樣率的重要說明**：變形監測通常採 **30 s 或 1 min** 取樣（趨勢訊號的頻寬遠低於 1 Hz），而非 1 Hz。這使 batch FGO 的可行性大幅提升——這是監測應用相對於車載/機載 GNSS 的關鍵差異，也是本評估的重要前提。SHM 的模態辨識需要 10–100 Hz，但那是不同的產品線（見 §9.3）。

## 8.2 綜合比較表

| 評估項目 | EKF | Batch FGO | Sliding Window FGO | iSAM2 |
|---|---|---|---|---|
| **Precision（精度）** | 基準 | **最高** | 高 | 高 |
| — 短期（單 epoch） | 基準 | +（重複線性化） | + | + |
| — 長期趨勢（日/週） | 弱（無回溯） | **強（全域一致）** | 中（受限於視窗） | 中（受限於視窗） |
| — 離群值韌性 | 弱（0/1 硬拒絕） | **強（GNC 至 Tukey）** | 中強（Huber→Cauchy） | 中（僅 Huber） |
| — 收斂性（大初始誤差） | 弱（單次線性化） | **強** | 強 | 中（增量特性限制） |
| **Computational Cost** | **最低** | 最高 | 中 | 中低（攤銷） |
| — 每 epoch 複雜度 | `O(n·m² + n²m)`；`filter_()` 壓縮後實測 < 20 ms | 全圖重解 | `O((W·s)^1.5)` 稀疏 | 攤銷近 `O(1)`；最差 `O((W·s)^1.5)` |
| — 24 h 總量（30 s 取樣，N=2880） | ~60 s | ~10–30 min | ~5–15 min | ~3–8 min |
| — 延遲可預測性 | **最佳（固定）** | N/A（離線） | **良好（固定視窗）** | 差（重線性化尖峰） |
| **Memory Usage** | **最低** | 最高 | 中 | 中 |
| — 峰值 | `O(n²)` ≈ 幾百 KB | `O(N·(s+m))` 稀疏；N=2880 時約 0.5–2 GB | `O(W·(s+m))` ≈ 幾 MB | `O(W·(s+m))` + Bayes tree ≈ 10–50 MB |
| — 是否有界 | 是 | 否 — 隨時間線性成長 | 是 | 是 |
| **Real-time Capability** | **是 — 優異** | 否 — 不適用 | 是 — 良好 | 是 — 良好（需延遲監控） |
| — 適用於 NRT (< 5 s) | 是 | 否 | 是 | 是 |
| — 適用於 hard real-time | 是 | 否 | 注意 — 需上限保護 | 否 — 尖峰不可預測 |
| **Monitoring Capability** | 低 | **最高** | 高 | 高 |
| — 事後修正錯誤 fix/slip | 否 | **是 — 全期間** | 是 — 視窗內 | 是 — 視窗內 |
| — 弧段縫合（§4.1.3） | 否 | **是** | 是 | 是 |
| — TDCP 跨 epoch factor | 否（需狀態增廣） | 是 | 是 | 是 |
| — 多測站剛體約束 | 注意 — 需巨大聯合狀態 | **是 — 自然** | 是 | 是 |
| — 單邊/非高斯約束（邊坡潛變） | 否 — 結構上不可能 | **是** | 是 | 注意 — 非凸與增量交互複雜 |
| — 完整殘差歷史可查 | 否（已邊緣化） | **是** | 視窗內 | 視窗內 |
| **Digital Twin Suitability** | 低 | **最高** | 中高 | 高 |
| — 異質感測器融合 | 需大幅改寫 | **是 — 加 factor 即可** | 是 | 是 |
| — 物理模型作為約束 | 否 | **是** | 是 | 是 |
| — 不確定性完整傳遞至孿生體 | 部分（僅當前 `P`） | **是 — 全期聯合協方差** | 視窗內聯合 | 視窗內聯合 |
| — 假設檢驗／what-if 重解 | 否 | **是 — 改 factor 重解** | 注意 — 受視窗限制 | 注意 — 受視窗限制 |
| — 增量更新（孿生體即時同步） | 是 | 否 | 中 | **是 — 最佳** |

> **表中所有計算成本與記憶體數字為基於複雜度分析的工程估計，非實測值。Phase 2 的驗收準則之一即為以實際資料建立這些數字（§11.2）。** 標示為「估計」的欄位在 Phase 2 報告中須以實測取代。

## 8.3 逐 Solver 評析

### 8.3.1 EKF

**保留的理由，不只是相容性。** EKF 有三項 FGO 無法取代的特性：

1. **延遲完全可預測**。監測系統的告警路徑（例如邊坡位移超限 → 觸發疏散）不能有延遲抖動。
2. **失效模式簡單**。EKF 發散是可偵測的（協方差爆炸）；FGO 的失效模式（收斂到錯誤局部極小、robust kernel 誤殺、圖結構錯誤）更隱蔽。
3. **是 FGO 的初值來源與交叉驗證基準**。EKF 解與 FGO 解的差異本身就是一個重要的品質指標（§10.4）。

**因此：EKF 永遠執行，不是「舊模式」，而是產品的安全基線。**

### 8.3.2 Batch FGO

**最高精度，但只能離線。**

- **殺手級應用：每日再處理。** 監測系統的核心產出不是即時位置，而是**變形趨勢**。趨勢是統計量，晚幾小時完全可以接受。每天凌晨對前一日全部資料做一次 batch FGO + GNC 至 Tukey，產出的日解精度可顯著優於 NRT 解。
- **第二應用：事件鑑識。** 地震、颱風、暴雨後，對事件前後 ±2 小時做高精度重解，是判定結構是否受損的關鍵證據。
- **限制**：記憶體隨 `N` 成長。1 Hz 資料需分段（例如每小時一段，段間以 marginal factor 銜接）。30 s 取樣則單日單段可行。

### 8.3.3 Sliding Window FGO

**NRT 的保守選擇。**

- 每次 update 對整個視窗重解，計算量固定，**延遲可預測性遠優於 iSAM2**。
- 對延遲抖動敏感、或部署在計算資源受限的邊緣裝置時，這是比 iSAM2 更好的選擇。
- 視窗長度 `fgo-window` 是主要旋鈕。過短（< 60 s）則 FGO 相對 EKF 的優勢有限；過長則成本上升。**建議起始值 300 s，並在 Phase 2 以實測資料做敏感度分析。**

### 8.3.4 iSAM2

**NRT 的效能最佳選擇，但需要延遲保護。**

- 增量更新使平均成本遠低於 sliding window。
- **主要風險：重線性化級聯**。當某個變數變化超過 `relinearizeThreshold`，其所在的 Bayes tree 子樹需重新線性化並可能觸發變數重排序。最壞情況接近全圖重解。
- 在監測應用中，這種尖峰的觸發時機是可預期的：AR 從 float 跳到 fix 時的位置躍變、長時間失鎖後重收斂、大量衛星同時進出視野。
- **緩解**：
  1. 設定每 epoch 的計算時間上限；超時則放棄本 epoch 的 FGO 解，輸出 EKF 解並標記。
  2. `relinearizeThreshold` 調校：太小 → 頻繁重線性化；太大 → 線性化誤差累積。建議位置 0.05 m、偏差 0.05 cycle。
  3. 定期（例如每小時）主動重建圖，避免 Bayes tree 病態化。

## 8.4 推薦組態

| 部署情境 | Solver | 視窗 | Robust | 取樣 | 理由 |
|---|---|---|---|---|---|
| **NRT 主線（標準測站）** | `isam2` | 300 s | Huber | 30 s | 效能最佳，延遲可接受 |
| **NRT（邊緣裝置 / 資源受限）** | `sliding` | 180 s | Huber | 30 s | 延遲可預測，記憶體有界 |
| **NRT（告警關鍵路徑）** | `ekf` | — | — | 1 s | 延遲確定性優先；FGO 解作為補充 |
| **每日再處理** | `batch` | 全日 | GNC→Tukey | 30 s | 最高精度，產出官方日解 |
| **事件鑑識** | `batch` | ±2 h | GNC→Tukey | 1 s | 最高時間解析度 + 最高精度 |
| **回歸基準** | `ekf` | — | — | 依資料集 | I1 驗證 |

## 8.5 EKF vs FGO 的精度增益 — 誠實的預期

> **必須說明：本文件不對「FGO 比 EKF 精確 X%」做承諾。** 增益高度依賴：測站環境（多路徑強度）、基線長度、衛星可見度、實際變形量級。文獻中 GNSS-FGO 相對 EKF 的改善在都市峽谷等惡劣環境可達 30–50%，在開闊天空的良好環境則可能僅 5–10%（因為 EKF 本身已接近 Cramér–Rao 下界）。

**變形監測測站的特徵是「環境固定但常有部分遮蔽」**（橋下、邊坡植被、建物立面），介於兩者之間。

**Phase 2 的核心任務就是量測這個數字。** 若在貴司的實際測站上增益 < 10%，則應重新評估是否值得推進 Phase 3——這是一個明確的 go/no-go 決策點（§11.2 里程碑 M2.4）。

**但精度不是唯一價值**。即使定位精度增益有限，以下能力仍可能單獨證成本專案：
- 弧段縫合帶來的長期時間序列連續性（§4.1.3）
- 多測站剛體約束帶來的結構級解釋（§4.2.7-D）
- 單邊約束帶來的物理先驗注入（§4.2.7-B）
- 完整不確定性傳遞至 Digital Twin（§8.2）
- 事後可修正錯誤 fix 的能力

這些是 EKF **結構上做不到**的事，與精度百分比無關。**建議在 Phase 2 的 go/no-go 決策中，將這些能力與精度增益一併評估。**

---

# 9. GNSS Monitoring 應用

## 9.1 共通效益

FGO 對所有監測型態的共通效益，依價值高低排序：

### (1) 時間序列連續性（價值：高）

變形監測的產出是**時間序列**，而時間序列最怕的不是雜訊，而是**不連續**（跳階）。跳階的來源：

| 來源 | EKF 的行為 | FGO 的行為 |
|---|---|---|
| Cycle slip 誤報 | `udbias()` 重設狀態，解跳變數 cm | 產生新弧段；若事後判定為誤報，以 stitch factor 縫合，跳階消失 |
| AR 從 float 跳到 fix | 位置瞬間躍變 | 視窗內重解，躍變被分攤／平滑 |
| AR 錯誤固定（wrong fix） | 錯誤傳播且 `holdamb()` 會鎖住錯誤 | 事後可移除該 factor 並重解視窗／全期 |
| 衛星進出視野 | 幾何突變造成解跳動 | 多 epoch 資訊平滑此躍變 |

**對 mm 級變形監測而言，消除 cm 級的假跳階，比降低 mm 級的隨機雜訊更有價值。** 這是 FGO 最實質、最容易驗證的效益。

### (2) 物理先驗的注入（價值：高）

EKF 的量測更新必須是線性高斯的。FGO 可以加入任意的非線性、非高斯、甚至單邊的約束（§4.2.7）。這讓工程知識能直接進入估計器，而不只是事後的資料後處理。

### (3) 多測站聯合解算（價值：中）

同一結構上的多個測站，其運動有強烈的相關性（剛體、模態）。FGO 中加入跨測站 factor 是自然的；EKF 中需要巨大的聯合狀態向量。

### (4) 不確定性的完整表達（價值：中）

FGO 提供視窗內（或全期）的**聯合**協方差，而非只有當前 epoch 的邊緣協方差。這對「這個 5 mm 的位移在統計上顯著嗎？」這類問題至關重要——答案依賴於位移前後兩個時刻估計的相關性。

### (5) 離群值韌性（價值：中）

Robust kernel 對突發多路徑、NLOS、短暫干擾的抑制。

## 9.2 Bridge Monitoring（橋梁監測）

### 場景特徵

| 項目 | 說明 |
|---|---|
| 位移量級 | 溫度效應：縱向數 cm/日；活載：垂向 mm–cm；長期：mm/年 |
| 主要週期 | 日週期（溫度）、季週期（溫度）、瞬時（車輛載重） |
| 環境挑戰 | 橋塔／纜索遮蔽、水面多路徑（強且隨潮位變化）、橋面震動 |
| 測站配置 | 主跨中央、1/4 跨、塔頂、橋台（參考站） |
| 取樣率 | 變形監測 30 s；載重響應 1 Hz；模態辨識 10–100 Hz |

### FGO 帶來的效益

| 效益 | 機制 | 價值 |
|---|---|---|
| **水面多路徑抑制** | Robust kernel + 測站多路徑圖（§5.4.3）。水面多路徑隨潮位變化，是準週期的，可被 az/el 圖部分捕捉 | 高 |
| **溫度-位移耦合建模** | §4.2.7-A 的溫度耦合 Site Factor，把 `Δu = α·L·ΔT` 直接寫進圖 | 高 直接產出「扣除溫度後的殘餘變形」，這才是結構健康的指標 |
| **多測站剛體/模態約束** | §4.2.7-D。橋面各點的相對位移受結構剛度約束 | 高 偏離剛體的殘差 = 局部損傷指標 |
| **纜索遮蔽下的連續性** | 弧段縫合 + TDCP | 中 |
| **支承狀態診斷** | 縱向約束 factor 的殘差若持續增大 → 支承卡死或失效 | 中 這是一個新的、FGO 特有的診斷產品 |

### 建議 Factor 組合

```
每 epoch:  GnssDDFactor(全頻全系統)
           MotionFactor       σ_h=1e-4, σ_v=3e-4  (m/√s)
           TdcpFactor         (所有連續弧段)
           SiteDirectionalFactor(縱向，σ 依支承型式 5–50 mm)
           SiteTemperatureFactor(若有溫度感測器)
跨測站:    SiteRigidBodyFactor(橋面各點)
每日:      DatumPriorFactor(橋台參考站)
```

## 9.3 Slope Monitoring（邊坡監測）

### 場景特徵

| 項目 | 說明 |
|---|---|
| 位移量級 | 潛變：mm–cm/月；加速期：cm–m/日；破壞：瞬時 |
| 主要驅動 | 降雨（孔隙水壓）、地震、人為開挖、凍融 |
| 環境挑戰 | 植被遮蔽（樹冠衰減與多路徑）、地形遮蔽、供電與通訊困難 |
| 測站配置 | 滑動體上多點 + 穩定區參考站 |
| 取樣率 | 常態 5–30 min；預警期 30 s–1 min |

### FGO 帶來的效益

| 效益 | 機制 | 價值 |
|---|---|---|
| **單向潛變先驗** | §4.2.7-B(B2) 的單邊 hinge 約束 | 高 **EKF 結構上不可能表達**。這是最能展現 FGO 價值的場景 |
| **滑動面幾何約束** | (B1) 面外位移約束 | 高 把地質調查的滑動面資訊直接注入估計 |
| **植被遮蔽下的韌性** | Robust kernel + 弧段縫合。植被遮蔽造成頻繁短暫失鎖與 slip 誤報 | 高 邊坡測站的 slip 誤報率遠高於開闊測站 |
| **加速度趨勢偵測** | FGO 直接估計加速度狀態；批次解的加速度估計遠比 EKF 穩定 | 高 **邊坡破壞預警的核心指標是位移加速度**（Fukuzono / Saito 倒數速度法）。EKF 的加速度估計雜訊太大而難以實用 |
| **低取樣率下的精度** | 5–30 min 取樣時每 epoch 的觀測獨立性高，FGO 的多 epoch 平滑增益顯著 | 中 |

> **加速度估計是邊坡監測的關鍵差異化。** 倒數速度法（inverse velocity）預測破壞時間需要穩定的速度與加速度估計。EKF 的加速度狀態雜訊大（過程雜訊必須設得夠大才不發散），而 batch FGO 對整段時間序列同時求解，加速度估計的統計效率高得多。這值得作為 Phase 2 的一個獨立驗證項目。

### 建議 Factor 組合

```
每 epoch:  GnssDDFactor
           MotionFactor       σ_h=1e-5, σ_v=3e-5
           VelocityContinuityFactor
           AccelSmoothFactor  (啟用，dynamics=2)
           TdcpFactor
           SiteDipConstraintFactor(B1: 面外, σ=5mm)
           SiteCreepPriorFactor(B2: 單向, σ_creep=10mm)   ← 需審慎啟用
每日:      DatumPriorFactor(穩定區參考站)
```

## 9.4 Structural Health Monitoring（結構健康監測）

### 場景特徵與 GNSS 的定位

SHM 的完整需求（模態頻率、阻尼比、模態振型）通常需要 10–100 Hz 的加速度計。**GNSS 在 SHM 中的角色是提供「絕對位移」**，這是加速度計無法提供的（二次積分會漂移）。

| 項目 | 說明 |
|---|---|
| 頻寬 | GNSS 實用上限約 10–20 Hz（受多路徑與接收機雜訊限制） |
| 位移解析度 | 動態情境下 mm–cm 級 |
| 與加速度計的關係 | **互補**：GNSS 提供低頻絕對位移（DC–1 Hz），加速度計提供高頻（0.1–100 Hz）。融合後可得全頻段位移 |

### FGO 帶來的效益

| 效益 | 機制 | 價值 |
|---|---|---|
| **GNSS + 加速度計融合** | Phase 4 的 IMU pre-integration factor。這是 FGO 最成熟的應用領域（源自 VIO/SLAM） | 高 這是 Phase 4 最有把握的擴充，因為 GTSAM 的 `ImuFactor`/`CombinedImuFactor` 已高度成熟 |
| **絕對位移的無漂移約束** | GNSS factor 為加速度計的二次積分提供絕對錨定 | 高 |
| **時間同步的圖表達** | 不同取樣率的感測器在 factor graph 中自然共存（各自連到最近的位置節點，或以插值 factor 連接） | 中 EKF 中處理多速率量測需要複雜的排程 |
| **模態參數的不確定性** | 從聯合協方差傳遞至模態辨識結果 | 中 |

**警示**：§4.2.6 的 jerk-limited smoothness factor 與 §4.2.4 的緊 Motion Factor **會主動抹除結構的動態響應**。SHM 組態必須使用寬鬆的過程雜訊（`σ_h ≈ 1e-3`），且 `fgo-jerkconst` 必須關閉。**建議在設定驗證中加入檢查：若 `fgo-jerkconst=on` 且取樣率 > 1 Hz，發出警告。**

## 9.5 Deformation Monitoring（廣義變形監測）

### 場景

大壩、隧道、地層下陷、火山、地熱區、礦區沉陷、老舊建物。共通特徵：**極緩慢的變形（mm/年 至 cm/年），監測期以年計**。

### FGO 帶來的效益

| 效益 | 機制 | 價值 |
|---|---|---|
| **長期趨勢的統計效率** | Batch FGO 對整段時間同時求解，趨勢估計的變異數遠小於「先逐 epoch 定位再回歸」 | 高 **這是最重要的一項**。變形速率是最終產品，而不是位置 |
| **多路徑的系統性偏差消除** | 測站多路徑圖（§5.4.3）。對年變形率 1 mm/yr 的偵測，mm 級的系統性偏差是致命的 | 高 |
| **基準框架的一致性** | 全期 batch 解可統一參考框架，避免逐日解的框架漂移 | 高 |
| **與 InSAR 的融合** | Phase 4。InSAR 提供高空間密度但低時間頻率與相對量測；GNSS 提供高時間頻率的絕對量測。二者在 factor graph 中融合是自然的 | 高 見 §13.3 |
| **季節性訊號的分離** | 可加入週期性 factor（年週期、半年週期）明確建模熱脹冷縮與地下水位效應，使殘餘趨勢更純淨 | 中 |

### 週期性 Factor（Deformation 專用）

```
位置的分解模型：
  p(t) = p_0 + v·(t-t_0) + A_1·cos(ω_y t + φ_1) + A_2·cos(2ω_y t + φ_2) + ε(t)

在 factor graph 中，把 p_0, v, A_1, φ_1, A_2, φ_2 作為全域變數，
加入 factor:  e = p_k - [p_0 + v·Δt_k + 週期項(t_k)]
```

這讓**變形速率 `v` 成為圖中的一個顯式變數，並直接獲得其協方差**——這正是 §10.3 的 `deformation_rate` 及其信心區間的最嚴謹來源，遠優於對位置時間序列做事後回歸（後者忽略了位置估計間的相關性，會低估速率的不確定性）。

> **這是一個強烈建議在 Phase 3 實作的功能**：把「變形速率」從後處理的統計量提升為圖中的一級變數。對監測產品而言，這直接改善了最終交付物的品質與可信度。

---

# 10. AI Insight Integration

## 10.1 設計目標

> 從 FGO 結果產出一份**自足（self-contained）、帶不確定性、帶品質指標**的結構化輸出，讓下游的 AI 分析（異常偵測、趨勢預測、告警決策）不需回頭查詢 GNSS 領域細節。

三個原則：

1. **一切量都帶不確定性**。單純的數值對 AI 分析毫無用處——`5.0 mm ± 0.8 mm` 與 `5.0 mm ± 12 mm` 是完全不同的事實。
2. **區分「量測」與「推論」**。位置是量測；變形速率是推論。兩者的可信度來源不同，必須分開標示。
3. **暴露內部診斷量**。AI 異常偵測最有價值的特徵，往往不是位置本身，而是解算過程的健康指標（殘差分布、降權比例、fix 率）。這些在傳統 `.pos` 輸出中完全遺失。

## 10.2 輸出 Schema

每 epoch 一筆 JSON（NDJSON 格式，一行一筆，便於串流與追加）：

```jsonc
{
  "schema_version": "1.0",
  "site_id": "BRIDGE-P03",
  "epoch": 128473,
  "time": {
    "gpst": "2026-08-20T04:15:30.000Z",
    "week": 2432, "tow": 274530.0
  },
  "solver": {
    "type": "isam2",              // ekf | batch | sliding | isam2
    "window_sec": 300.0,
    "iterations": 4,
    "converged": true,
    "latency_ms": 143.2,
    "revision": 0                 // 0 = 首解；>0 = 視窗內被後續 epoch 修正過
  },

  // ---- 一級量測 ----
  "position": {
    "frame": "ENU",
    "datum": { "type": "site_ref", "epoch_ref": "2026-01-01",
               "ecef": [-2956123.4567, 4864321.8901, 2681234.5678] },
    "enu_m": [0.00312, -0.00847, -0.01523],
    "cov_enu_m2": [[6.4e-6, 1.1e-6, 2.0e-6],
                   [1.1e-6, 7.1e-6, 2.6e-6],
                   [2.0e-6, 2.6e-6, 3.9e-5]],
    "sigma_enu_mm": [2.53, 2.66, 6.24],
    "ecef_m": [-2956123.4536, 4864321.8817, 2681234.5526]
  },
  "velocity": {
    "enu_mps": [1.2e-8, -3.4e-8, -5.9e-8],
    "sigma_enu_mmps": [0.0021, 0.0024, 0.0058],
    "source": "state"             // state | tdcp | doppler | finite_diff
  },
  "acceleration": {
    "enu_mps2": [2.1e-11, -4.0e-11, -7.2e-11],
    "sigma_enu_mmps2": [1.4e-5, 1.6e-5, 3.8e-5],
    "source": "state"
  },

  // ---- 推論量 ----
  "deformation": {
    "window_days": 30,
    "n_epochs_used": 86400,
    "rate_enu_mm_per_year": [0.42, -1.87, -4.31],
    "rate_sigma_mm_per_year": [0.18, 0.21, 0.55],
    "rate_source": "graph_variable",   // graph_variable | post_regression
    "cumulative_enu_mm": [3.12, -8.47, -15.23],
    "trend_significant": [false, true, true],   // |rate| > 2*sigma
    "seasonal": {
      "annual_amp_enu_mm": [0.8, 1.2, 3.4],
      "annual_phase_deg":  [45.2, 132.7, 201.4],
      "detrended_residual_rms_mm": [1.1, 1.3, 3.2]
    },
    "inverse_velocity_days": null   // 邊坡專用；破壞時間預測，null 表不適用
  },

  // ---- 品質與信心 ----
  "confidence": {
    "score": 0.87,                // [0,1] 綜合分數，見 §10.5
    "grade": "A",                 // A>=0.85, B>=0.70, C>=0.50, D<0.50
    "components": {
      "formal_precision": 0.92,
      "redundancy":       0.88,
      "residual_health":  0.91,
      "ambiguity":        0.95,
      "robustness":       0.79,
      "continuity":       0.83
    },
    "limiting_factor": "robustness"
  },

  "quality": {
    "n_sat_used": 24, "n_sat_visible": 31,
    "n_obs_dd": 46, "dof": 41,
    "gdop": 1.62, "pdop": 1.41, "hdop": 0.78, "vdop": 1.17,
    "ar_status": "fix",           // fix | float | none
    "ar_ratio": 14.7, "ar_threshold": 3.0,
    "fix_rate_1h": 0.964,
    "chi2": 44.8, "chi2_dof": 41, "chi2_pvalue": 0.31,
    "nis": 1.093,                 // normalized innovation squared, 理想 ~1.0
    "residual_rms_phase_mm": 4.2,
    "residual_rms_code_m": 0.51,
    "robust_downweight_frac": 0.087,   // 權重 < 0.5 的觀測量比例
    "robust_rejected_frac": 0.021,     // 權重 < 0.1
    "scale_factor_mad": 1.14,          // §5.5.6，理想 ~1.0
    "mp_index_mean": 0.34,             // §5.4.5 CMC RMS (m)
    "snr_mean_dbhz": 44.6,
    "n_slips_1h": 3, "n_arcs_active": 24, "n_arcs_stitched_1h": 1,
    "base_age_s": 1.2, "base_baseline_m": 842.3
  },

  // ---- 異常旗標 ----
  "anomalies": [
    { "code": "ROBUST_HIGH_REJECT", "severity": "warn",
      "detail": "rejected_frac=0.021 exceeds 7d baseline 0.008 by 2.6x",
      "sats": ["C21","C24"] }
  ],

  // ---- 交叉驗證 ----
  "cross_check": {
    "ekf_enu_m": [0.00298, -0.00861, -0.01489],
    "fgo_minus_ekf_mm": [0.14, 0.14, -0.34],
    "consistent": true            // 差異 < 3 * combined sigma
  },

  // ---- Site constraint 診斷 ----
  "site_constraints": [
    { "name": "bridge_longitudinal", "type": "directional",
      "residual_mm": 2.1, "sigma_mm": 5.0, "normalized": 0.42, "status": "ok" },
    { "name": "rigid_body_P02_P03", "type": "baseline_length",
      "residual_mm": 0.7, "sigma_mm": 2.0, "normalized": 0.35, "status": "ok" }
  ]
}
```

## 10.3 各輸出量的定義與計算

### Position

- **來源**：FGO 最佳化結果中 `keyPos(k)` 的值。
- **協方差**：`Marginals::marginalCovariance(keyPos(k))`，ECEF → ENU 以 `covenu()`（`rtkcmn.c`）旋轉。
- **Datum**：ENU 的原點必須是一個**明確且穩定的參考**。建議為首次部署後第一週的 batch 解平均，並在 metadata 中固定記錄。**不可使用「前一日平均」之類的滑動基準**——那會把趨勢從輸出中減掉。

### Velocity

三種來源，優先序：

1. `graph state`（`keyVel(k)`）— 若 `dynamics ≥ 1`。最佳。
2. `tdcp` — 由 TDCP factor 隱含的位移增量除以 `Δt`。精度高但需連續弧段。
3. `finite_diff` — 對位置時間序列做差分。最差（放大雜訊），僅作 fallback。

**必須在輸出中標示來源**，因為三者的精度差 1–2 個數量級。

### Acceleration

- `graph state`（`keyAcc(k)`）— 若 `dynamics == 2`。
- 對邊坡預警至關重要（§9.3）。
- 注意：若 `dynamics < 2`，此欄應為 `null` 而非 0——**不可輸出一個看起來是零加速度的假值**，那會讓下游 AI 誤判為「結構完全靜止」。

### Deformation Rate

兩種計算方式，**優先使用第一種**：

**(1) `graph_variable`（推薦，Phase 3）**：把速率作為圖中的顯式變數（§9.5 的週期性 factor）。速率的協方差直接來自 marginals，**正確地考慮了所有位置估計之間的相關性**。

**(2) `post_regression`（Phase 2 過渡）**：對位置時間序列做加權最小平方回歸：

```
p_i = p_0 + v·(t_i - t_0) + ε_i,    W = Σ_p^{-1}
[p_0; v] = (AᵀWA)^{-1} AᵀW p
Cov([p_0; v]) = (AᵀWA)^{-1}
```

**此法會低估 `v` 的不確定性**，因為它假設 `ε_i` 獨立，但 FGO（與 EKF）的相鄰 epoch 位置估計是**強烈正相關**的。建議做法：以殘差的自相關估計等效獨立樣本數 `N_eff`，並將 `σ_v` 乘以 `sqrt(N/N_eff)` 加以修正。**必須在輸出中標示 `rate_source`，讓下游知道該打多少折扣。**

### Confidence Score

見 §10.5。

## 10.4 異常偵測的特徵

以下量是異常偵測最有價值的輸入。**它們在傳統 RTKLIB 輸出中完全不存在**，是 FGO 整合帶來的新資訊。

| 特徵 | 正常範圍 | 異常意義 |
|---|---|---|
| `nis`（正規化新息平方） | 0.7 – 1.3 | > 1.5：誤差模型過度樂觀，或有未建模的誤差源（多路徑增強、大氣擾動）<br>< 0.5：誤差模型過度保守，資訊被浪費 |
| `chi2_pvalue` | > 0.01 | < 0.01：模型與資料不符 |
| `scale_factor_mad` | 0.8 – 1.3 | 持續偏離 1：`varerr()` 參數需重新調校 |
| `robust_rejected_frac` | < 0.02 | 突增：環境改變（新增遮蔽物、施工、天線移位） |
| `mp_index_mean` | 站點相依 | 突增：多路徑環境改變 |
| `n_slips_1h` | 站點相依 | 突增：天線／接收機故障、強干擾、劇烈震動 |
| `fix_rate_1h` | > 0.9 | 下降：衛星幾何、大氣活動、或設備劣化 |
| `fgo_minus_ekf_mm` | < 3σ | 超出：兩引擎不一致，其中之一有問題。**這是最有價值的自我診斷** |
| Site constraint `normalized` | < 2 | > 2：物理先驗與實際不符 → **可能是真實的異常變形，也可能是先驗設錯**。需人工研判 |
| `detrended_residual_rms_mm` | 站點相依 | 突增：非模型化的變形開始 |

> **Site constraint 殘差是最直接的結構異常指標。** 例如橋梁縱向約束的殘差持續增大，代表結構的縱向位移超出支承設計容許——這是一個可直接告警的物理事件，而不需要 AI 學習。**建議把 site constraint 殘差列為告警規則的一級輸入。**

### 建議的異常偵測分層

```
第 0 層（規則）: site constraint 殘差超限、位移超設計值、fix_rate 崩潰
                 -> 立即告警，不需 AI
第 1 層（統計）: 各品質指標對其 7/30 日基線的 z-score
                 -> 偏離 > 3σ 觸發 warn
第 2 層（AI）  : 多變量異常偵測（Isolation Forest / autoencoder）
                 輸入 = §10.4 全部特徵 + 位移
                 -> 偵測「單看每個指標都正常，但組合異常」的狀況
第 3 層（AI）  : 時序預測（趨勢外推、破壞時間預測）
                 輸入 = deformation.rate 時間序列 + 環境資料（雨量、溫度）
```

**第 2 層是 FGO 整合的獨特價值**：只有當品質指標豐富且可信時，多變量異常偵測才有意義。

## 10.5 Confidence Score 設計

### 設計原則

- 輸出 `[0, 1]`，越高越可信。
- **由可解釋的分量組成**，且輸出 `limiting_factor` 指出瓶頸——一個不透明的分數對工程師無用。
- 使用**幾何平均**而非算術平均：任一分量崩潰時總分應顯著下降（木桶效應），這符合定位品質的實際特性。

### 公式

```
score = Π_i (c_i ^ w_i)          ,  Σ w_i = 1        (加權幾何平均)
```

| 分量 `c_i` | 權重 `w_i` | 定義 | 理由 |
|---|---|---|---|
| `formal_precision` | 0.25 | `clamp(σ_target / σ_3d, 0, 1)`，`σ_3d = sqrt(tr(Σ_enu))`，`σ_target` 為該站的規格值（例如 5 mm） | 形式精度是最基本的指標 |
| `redundancy` | 0.15 | `clamp((dof - dof_min) / (dof_nom - dof_min), 0, 1)`，`dof_min=4`, `dof_nom=25` | 自由度不足時，形式精度不可信（無法檢核） |
| `residual_health` | 0.20 | `exp(-0.5·((nis - 1)/0.4)²)` — 以 `nis=1` 為中心的高斯 | **雙邊懲罰**：過大過小都是問題 |
| `ambiguity` | 0.15 | `fix`: `clamp(ratio/threshold/2, 0.7, 1.0)`；`float`: 0.45；`none`: 0.15 | AR 狀態對精度影響巨大 |
| `robustness` | 0.15 | `1 - clamp(rejected_frac / 0.10, 0, 0.8)` | 大量離群值代表環境或設備問題 |
| `continuity` | 0.10 | `1 - clamp(n_slips_1h / 20, 0, 0.6)`；若本 epoch 有弧段重啟再乘 0.9 | 時間序列連續性 |

### 校準與注意事項

**上述權重與參數是基於工程判斷的初始值，不是經過驗證的模型。** 必須：

1. **在 Phase 2 以已知真值的資料集校準**。方法：取一批有獨立真值（例如強制位移試驗、或全站儀同步觀測）的資料，計算實際誤差，檢驗 `score` 與實際誤差的相關性。
2. **驗收準則**：`score` 與 `|實際誤差|` 的 Spearman 相關係數應 < -0.6（負相關，分數高則誤差小）。若達不到，需重新設計分量或權重。
3. **不可讓 confidence score 成為唯一的告警依據**。它是一個輔助指標，最終決策應同時看原始的 `sigma_enu_mm` 與 `quality` 區塊。

### 輸出 `limiting_factor`

```cpp
limiting_factor = argmin_i (c_i)     /* 最低的分量 */
```

這讓維運人員能直接知道「今天這個測站解不好，是因為 AR 一直 float」而非「分數是 0.62」。**這個欄位的實用價值可能高於分數本身。**

## 10.6 輸出通道

| 通道 | 用途 | 實作 |
|---|---|---|
| NDJSON 檔案 | 歸檔、批次分析 | `fgo-insight-out` 設定路徑；按日輪替 |
| TCP/HTTP 串流 | 即時儀表板、告警系統 | 重用 `stream.c` 的 `STR_TCPSVR`（已存在，不需新寫） |
| 時序資料庫 | 長期趨勢查詢 | 由外部 agent 讀取 NDJSON 寫入（InfluxDB/TimescaleDB）；**不在 RTKLIB 內實作 DB client** |
| 既有 `.pos` / `.stat` | 相容性 | 完全不變 |

**設計決策**：不在 RTKLIB 內加入 HTTP client、DB driver 或 JSON 函式庫相依。輸出 NDJSON 到檔案或 TCP，由外部程式負責轉發。這保持 RTKLIB 的輕量與可移植性。

---

# 11. Implementation Roadmap

## 11.0 總覽

| Phase | 名稱 | 主要交付 | 複雜度 | 對核心碼的侵入 | Go/No-Go 決策點 |
|---|---|---|---|---|---|
| **1** | RTKLIB + EKF（基線建立） | 回歸測試框架、量化基準、殘差傾印 | 低 | 無（僅新增） | — |
| **2** | RTKLIB + Residual Export + GTSAM | 離線 FGO 原型、效益量化報告 | 中 | 極低 | **G2：FGO 效益是否足以證成 Phase 3** |
| **3** | RTKLIB Native FGO（架構 A′） | 行程內 FGO、NRT 可用、AI Insight | **高** | 中高 | **G3：NRT 品質與穩定度是否達產品標準** |
| **4** | Multi-Sensor FGO Platform | IMU/傾斜儀/InSAR/氣象/Digital Twin | 高 | 中 | 逐感測器獨立決策 |

**複雜度定義**：低 = 主要為既有技術的組裝；中 = 需要設計決策但路徑清楚；高 = 含未知數，需要探索與迭代。

**本文件不提供以「週」或「人月」為單位的時程估計**，因為那高度依賴投入人力、團隊既有的 GTSAM/SLAM 經驗、以及可用的測試資料量——這些資訊目前未知。各 Phase 的工作項目已切分到可獨立估點的粒度，建議由實際執行團隊據此估算。工作項目間的相依關係已在下方標示。

---

## 11.1 Phase 1 — RTKLIB + EKF（基線建立）

> **目標：在動任何 FGO 程式碼之前，建立「改動前後可比較」的能力。** 這個 Phase 不產出任何 FGO 功能，但它決定了後續所有階段能否被正確驗證。**跳過此階段是本專案最容易犯的錯誤。**

### 工作項目

| ID | 工作項目 | 相依 | 複雜度 | 說明 |
|---|---|---|---|---|
| **P1.1** | 建立回歸資料集 | — | 低 | 從實際監測測站蒐集 N ≥ 5 組代表性資料（各 ≥ 24 h）：開闊天空、橋下遮蔽、邊坡植被、都市多路徑、長基線。每組含 rover obs、base obs、nav、以及使用的設定檔 |
| **P1.2** | 建立 byte-diff 回歸腳本 | P1.1 | 低 | §6.11 G1。含 `.pos` 與 `.stat` 雙重比對 |
| **P1.3** | 固定並記錄建置組態 | — | 低 | §6.9 陷阱 1、2。記錄實際生效的 compiler flags 至基準 metadata |
| **P1.4** | 建立 CI 建置矩陣 | P1.3 | 低 | §6.11 G2 |
| **P1.5** | 量化現行 EKF 基線 | P1.1, P1.2 | 中 | 對每組資料計算：定位重複性（static 測站的位置 RMS）、fix rate、收斂時間、每 epoch 耗時、殘差 RMS、slip 次數 |
| **P1.6** | 建立「真值」參考 | P1.1 | 中 | 對 static 測站，以長時段（≥7 天）後處理平均解作為位置真值；理想上另有全站儀/水準測量的獨立驗證 |
| **P1.7** | 殘差傾印功能 | P1.3 | 中 | 新增輸出：每 epoch 的 `v`/`H`/`R`/`vflg`/`azel`/`sat`/`freq`。以獨立的 `-x` level 或新選項啟用，**不影響既有輸出路徑** |
| **P1.8** | 傾印格式定義與工具 | P1.7 | 低 | 二進位格式（節省空間）+ Python 讀取器。格式需版本化 |
| **P1.9** | CMC / MP 指標計算 | P1.7 | 低 | §5.4.5。低成本，先做可為 Phase 2 提供資料品質標籤 |

### 交付物

1. `test/data/regression/` — 版本化的回歸資料集（大檔案以 Git LFS 或外部儲存管理）。
2. `test/regression/run_regression.sh` — 一鍵回歸腳本。
3. `docs/fgo/baseline_report.md` — 現行 EKF 的量化基線報告。
4. `src/` 的殘差傾印功能（純新增，`ENABLE_FGO` 無關）。
5. `tools/fgo/read_residual_dump.py` — 傾印檔讀取器。
6. CI 設定（GitHub Actions 或等效）。

### 技術風險

| 風險 | 機率 | 衝擊 | 緩解 |
|---|---|---|---|
| 缺乏具獨立真值的資料 | 中 | 高 — 無法驗證精度改善 | 優先在有控制點的測站部署；或以長時段平均解作為代理真值並明確標示其限制 |
| 浮點結果因平台/編譯器而異，byte-diff 過於嚴格 | 中 | 中 | 基準綁定特定 compiler 版本；提供 tolerance-based 比對作為次要關卡 |
| 回歸資料集過大難以版本控制 | 高 | 低 | Git LFS 或外部物件儲存 + manifest 檔入 git |

### 里程碑與驗收準則

| 里程碑 | 驗收準則 |
|---|---|
| **M1.1** 回歸框架就緒 | 對未修改的程式碼執行回歸腳本，5/5 資料集全部通過（byte-identical） |
| **M1.2** 基線報告完成 | 報告含全部 5 組資料的量化指標；static 測站的位置重複性有明確數字 |
| **M1.3** 殘差傾印可用 | Python 讀取器可正確重建 `v`/`H`/`R`；以 `H·δx` 與有限差分交叉驗證通過 |

---

## 11.2 Phase 2 — RTKLIB + Residual Export + GTSAM

> **目標：以最低工程風險，量化回答「FGO 對我們的資料值不值得」。** 產出是一份**決策報告**，不是產品程式碼。

### 工作項目

| ID | 工作項目 | 相依 | 複雜度 | 說明 |
|---|---|---|---|---|
| **P2.1** | GTSAM 環境建置 | — | 低 | Docker 映像固定 GTSAM 4.2.x + Eigen + Boost 版本 |
| **P2.2** | 傾印 → GTSAM 圖的轉換器 | P1.8, P2.1 | 中 | Python（`gtsam` pip）或 C++。讀傾印檔建圖 |
| **P2.3** | GNSS DD Factor（線性版） | P2.2 | 中 | 以傾印的 `H`/`v` 建 `JacobianFactor`。**明知此版無法重複線性化，但足以驗證圖結構、協方差處理與 robust kernel 的框架** |
| **P2.4** | 相位偏差錨定 | P2.3 | 中 | §4.4.3。**必須先做，否則後續全部無法求解** |
| **P2.5** | Motion / Velocity / Accel Factor | P2.2 | 低 | 解析式，不需傾印資料 |
| **P2.6** | Site Constraint Factor | P2.5 | 中 | §4.2.7 全部子類型；設定檔格式定案 |
| **P2.7** | DD block covariance 處理 | P2.3 | 中 | §4.2.1 (a)(b) 兩種模式；比較其影響 |
| **P2.8** | Robust kernel + GNC | P2.3 | 中 | §5.5 全部 kernel；MAD 尺度估計 |
| **P2.9** | Batch / Sliding / iSAM2 三種 solver | P2.3–P2.8 | 中 | §4.5 |
| **P2.10** | TDCP Factor | P2.2 | 中高 | 需弧段追蹤。**這是最可能單獨證成專案的功能，優先度高** |
| **P2.11** | 弧段管理與縫合 | P2.10 | 中高 | §4.1.3 |
| **P2.12** | AI Insight 輸出（原型） | P2.9 | 中 | §10.2 schema；Python 實作即可 |
| **P2.13** | 有限差分 Jacobian 驗證 | P2.3–P2.6 | 中 | §6.11 G4。**每個 factor 都要做** |
| **P2.14** | 對照實驗與效益量化 | P2.9, P1.5 | **高** | 見下方「實驗設計」 |
| **P2.15** | Confidence score 校準 | P2.12, P1.6 | 中 | §10.5；需真值資料 |
| **P2.16** | 決策報告 | P2.14, P2.15 | 中 | Go/No-Go 的依據 |

### 實驗設計（P2.14）— 本 Phase 的核心

對每組回歸資料，跑以下組態並交叉比較：

| 組態 | 目的 |
|---|---|
| EKF（基線） | 對照組 |
| FGO batch，無 robust，無 site constraint | 隔離「批次求解」本身的貢獻 |
| FGO batch + robust (GNC) | 隔離 robust kernel 的貢獻 |
| FGO batch + robust + TDCP | 隔離 TDCP 的貢獻 |
| FGO batch + 全部 factor | 上限效益 |
| FGO sliding (300 s) + 全部 | NRT 可達效益 |
| FGO iSAM2 (300 s) + 全部 | NRT 效能組態 |

**量測指標**（對照 P1.5 的基線）：

| 指標 | 為何重要 |
|---|---|
| Static 測站位置重複性（3D RMS，mm） | 最直接的精度指標 |
| 對真值的偏差（若有） | 精度而非精密度 |
| 24 h 時間序列的跳階次數與幅度 | §9.1(1)，可能比 RMS 更重要 |
| 變形速率估計的標準差 | §9.5，最終產品的品質 |
| 加速度估計的雜訊水準 | §9.3，邊坡預警的關鍵 |
| Fix rate | AR 改善 |
| 每 epoch 計算時間（p50/p99） | NRT 可行性 |
| 記憶體峰值 | 部署可行性 |
| 回呼佔總時間的比例 | 驗證 §4.3 的「回呼不是瓶頸」假設 |

### 技術風險

| 風險 | 機率 | 衝擊 | 緩解 |
|---|---|---|---|
| 線性版 factor（無重複線性化）使 Phase 2 效益被系統性低估，導致錯誤的 No-Go | **高** | **高** | **關鍵緩解**：對 batch 模式實作一個「外迴圈重線性化」——在 Python 中呼叫 RTKLIB 命令列工具，以上一輪 FGO 解作為新的線性化點重新產生傾印，重複 3–5 輪。這近似 A′ 的行為，可用於估計 A′ 的真實效益上限。**此項應列為 P2.14 的必要組態** |
| 相位偏差錨定（P2.4）處理不當導致全面無法收斂 | 中 | 高 | 優先實作並以 §6.11 G4 的 `t_fgo_anchor` 測試守護 |
| Jacobian 符號/佈局轉換錯誤 | 中 | 高 | P2.13 有限差分驗證；先在合成資料上驗證 |
| GTSAM 學習曲線 | 中 | 中 | 團隊訓練；GTSAM 官方範例；預留學習時間 |
| 效益不顯著（真實結果） | 中 | — | **這不是風險，這是 Phase 2 要回答的問題**。應以開放心態接受 No-Go 結論 |

### 里程碑與驗收準則

| 里程碑 | 驗收準則 |
|---|---|
| **M2.1** 圖可求解 | 對至少 1 組資料，FGO 產出的解與 EKF 解的差異 < 5 cm（證明沒有結構性錯誤） |
| **M2.2** Jacobian 全數驗證 | 每個 factor 的解析 Jacobian 與有限差分的相對誤差 < 1e-6 |
| **M2.3** 全組態實驗完成 | 7 種組態 × 5 組資料 = 35 次執行全部完成並產出指標表 |
| **M2.4 (Gate G2)** | **決策報告完成，含明確的 Go/No-Go 建議與依據**。建議 Go 的條件（任一成立）：<br>(a) static 位置重複性改善 ≥ 15%；或<br>(b) 24 h 跳階次數減少 ≥ 50%；或<br>(c) 變形速率估計標準差改善 ≥ 20%；或<br>(d) 加速度估計雜訊改善 ≥ 30%；或<br>(e) §8.5 所列的結構性能力中，至少一項被證實對業務有明確價值 |

---

## 11.3 Phase 3 — RTKLIB Native FGO（架構 A′）

> **目標：把 Phase 2 驗證過的演算法，變成 NRT 產線可用的行程內功能。** 工程風險集中在重構與整合，演算法未知數已在 Phase 2 消除。

### 工作項目

| ID | 工作項目 | 相依 | 複雜度 | 風險 |
|---|---|---|---|---|
| **P3.1** | `zdres()`/`varerr()`/`ddcov()` 可見性提升 | Phase 1 | 低 | 低 |
| **P3.2** | **`ddres()` 純函式化（M3）** | P3.1 | **高** | **中高（RC-1）** |
| **P3.3** | 「凍結配對」模式 | P3.2 | 中 | 中 |
| **P3.4** | `rescode()` 純函式化 | Phase 1 | 中 | 低 |
| **P3.5** | `rtklib_fgo_api.h` C ABI 定案 | P3.2, P3.3 | 中 | 中 |
| **P3.6** | `fgo_stub.c` + CMake `ENABLE_FGO` | P3.5 | 低 | 低 |
| **P3.7** | `prcopt_t`/`rtk_t` 欄位新增 + `options.c` | — | 低 | 中（RC-2, RC-3） |
| **P3.8** | `fgo_config.h` + `fgo_solver.cpp` 骨架 | P3.5, P3.6 | 中 | 低 |
| **P3.9** | `fgo_graph.cpp`（含弧段管理） | P3.8 | 高 | 中 |
| **P3.10** | `fgo_factor.cpp`（全部 factor，回呼版） | P3.5, P3.9 | **高** | 中高 |
| **P3.11** | `fgo_gtsam.cpp`（三種 solver + robust + marginals） | P3.10 | 高 | 中 |
| **P3.12** | AR 整合（`manage_amb_LAMBDA()` 銜接） | P3.11 | 中高 | 中（R-AR1） |
| **P3.13** | `fgo_insight.cpp` | P3.11 | 中 | 低 |
| **P3.14** | `rtkpos.c` solver 分支（M1）+ fallback | P3.8 | 低 | 低 |
| **P3.15** | 單元測試套件（G4 全部） | P3.10–P3.13 | 中高 | 低 |
| **P3.16** | 非同步 worker thread 設計 | P3.14 | 高 | 中高 |
| **P3.17** | 效能剖析與最佳化 | P3.16 | 中 | 中 |
| **P3.18** | GUI solver 選項（Qt） | P3.7 | 低 | 低 |
| **P3.19** | 長時穩定度測試（≥ 30 天連續） | P3.16, P3.17 | 中 | 中 |
| **P3.20** | 週期性 factor（變形速率作為圖變數） | P3.11 | 中高 | 中 |
| **P3.21** | 測站多路徑圖（§5.4.3） | P3.11 | 中高 | 中 |
| **P3.22** | 文件與維運手冊 | 全部 | 中 | 低 |

### 建議的 PR 切分（降低審查風險）

| PR | 內容 | 可否獨立回退 |
|---|---|---|
| PR-1 | P3.1（可見性提升，無行為變更） | 是 |
| PR-2 | **P3.2 + P3.3（`ddres()` 純函式化）— 不含任何 FGO 程式碼** | 是 |
| PR-3 | P3.4（`rescode()` 純函式化） | 是 |
| PR-4 | P3.7（選項與結構欄位，預設全部關閉） | 是 |
| PR-5 | P3.5 + P3.6（C ABI + stub + CMake） | 是 |
| PR-6 | P3.8–P3.11（FGO 主體） | 是 |
| PR-7 | P3.12–P3.14（AR 整合 + insight + 分支） | 是 |
| PR-8 | P3.15（測試） | 是 |
| PR-9 | P3.16–P3.17（非同步 + 效能） | 是 |
| PR-10+ | P3.18–P3.21（GUI、進階功能） | 是 |

**PR-2 是最關鍵的一個**——它單獨觸碰核心 EKF 路徑且不帶來任何新功能。**它必須通過 byte-diff 回歸才能合併，且應獨立審查、獨立部署一段時間確認無問題後，才繼續後續 PR。**

### 技術風險

| 風險 | 機率 | 衝擊 | 緩解 |
|---|---|---|---|
| RC-1 `ddres()` 重構改變 EKF 行為 | 中 | 高 | PR-2 獨立化 + byte-diff + `t_ddres.c` |
| RC-2 ABI 破壞造成部署問題 | 高（必然） | 中 | 版本號提升 + release note + 全專案同步部署 |
| RC-9 C++ 例外跨 ABI | 中 | 高 | try/catch 樣板強制 + review checklist |
| R-AR1 邊緣協方差計算成為瓶頸 | 中 | 中 | 只算 AR 需要的子集；Cholesky factorization |
| iSAM2 重線性化尖峰破壞 NRT | 中 | 中 | 逾時保護 + EKF fallback + P3.16 非同步 |
| 記憶體洩漏（C/C++ 混合生命週期） | 中 | 中 | `rtkinit()`/`rtkfree()` 對稱管理；Valgrind/ASan 納入 CI |
| 長時運行的圖退化（Bayes tree 病態） | 中 | 中 | 定期重建圖；P3.19 長時測試 |
| 團隊 C++/GTSAM 能力不足 | 中 | 高 | Phase 2 已累積經驗；必要時外部技術支援 |

### 里程碑與驗收準則

| 里程碑 | 驗收準則 |
|---|---|
| **M3.1** 重構安全 | PR-2 合併，byte-diff 回歸 5/5 通過，`t_ddres.c` 通過 |
| **M3.2** FGO 端到端可跑 | `pos1-solver=fgo-sliding` 可產出解，與 Phase 2 離線結果差異 < 1 mm（驗證行程內實作與離線原型等價） |
| **M3.3** 重複線性化生效 | 相同資料下，A′ 的解優於 Phase 2 的線性版（證明重複線性化確實有貢獻）；且回呼佔總時間 < 30% |
| **M3.4** AR 整合完成 | FGO 路徑的 fix rate ≥ EKF 路徑；`t_fgo_anchor` 通過 |
| **M3.5** NRT 延遲達標 | §4.7.3 的延遲預算全部達成 |
| **M3.6 (Gate G3)** | **30 天連續 NRT 運行**：無崩潰、無記憶體洩漏、佇列丟棄率 < 0.1%、FGO 解可用率 > 99%、與 EKF 的一致性檢查通過率 > 99.5% |

---

## 11.4 Phase 4 — Multi-Sensor FGO Platform

> **目標：把 factor graph 從「GNSS 解算器」擴展為「多感測器監測平台」。** 每個感測器是一個獨立的、可分別決策的子專案。

### 感測器整合的優先序

依「技術成熟度 × 業務價值 ÷ 工程成本」排序：

| 順位 | 感測器 | 成熟度 | 業務價值 | 成本 | 理由 |
|---|---|---|---|---|---|
| 1 | **傾斜儀 (Tilt)** | 高 | 高 | **低** | Factor 極簡單（直接約束姿態或相對位移）；感測器便宜；與 GNSS 高度互補（GNSS 弱在垂直，傾斜儀強在角度） |
| 2 | **氣象 (Weather)** | 高 | 中高 | **低** | 溫度直接進 Site Factor（§4.2.7-A）；氣壓/溼度改善對流層建模；感測器便宜 |
| 3 | **IMU / 加速度計** | **極高** | 高 | 中 | GTSAM 的 `ImuFactor`/`CombinedImuFactor` 是最成熟的資產；SHM 必需 |
| 4 | **InSAR** | 中 | **極高** | 高 | 空間密度無可取代，但時間對齊、相位解纏、大氣改正、參考點選擇皆為研究級難題 |
| 5 | **Digital Twin 雙向耦合** | 低 | 極高 | 極高 | 需要前四項就位；且需要結構模型 |

### 工作項目（依感測器）

#### P4-A 傾斜儀

| ID | 項目 | 複雜度 |
|---|---|---|
| P4A.1 | 傾斜儀資料介面（串流/檔案） | 低 |
| P4A.2 | `TiltFactor`：約束測站姿態或相鄰測站的相對高程差 | 中 |
| P4A.3 | 時間同步與插值 factor | 中 |
| P4A.4 | 傾斜儀零漂與溫度係數的線上估計（作為圖變數） | 中高 |

#### P4-B 氣象

| ID | 項目 | 複雜度 |
|---|---|---|
| P4B.1 | 氣象資料介面 | 低 |
| P4B.2 | 溫度耦合 Site Factor（§4.2.7-A） | 中 |
| P4B.3 | 以實測氣壓/溫度/溼度取代 `tropmodel()` 的標準大氣假設 | 中 |
| P4B.4 | 降雨-邊坡位移的耦合先驗（邊坡專用） | 中高 |

#### P4-C IMU

| ID | 項目 | 複雜度 |
|---|---|---|
| P4C.1 | IMU 資料介面與時間同步（需硬體 PPS 或事件標記） | 中高 |
| P4C.2 | 引入 `gtsam::NavState`（位置+速度+姿態）取代純 `Point3` | **高** — 這是狀態表述的重大變更 |
| P4C.3 | `CombinedImuFactor` 整合（含 bias 隨機遊走） | 中 |
| P4C.4 | GNSS-IMU 桿臂（lever arm）標定 | 中 |
| P4C.5 | 高頻位移重建（GNSS + IMU 融合輸出 10–100 Hz 位移） | 中高 |

**注意**：P4C.2 會改變位置變數的型別，影響所有既有 factor。**這是 Phase 4 最大的架構風險。** 建議在 Phase 3 的 factor 設計中就以介面隔離位置變數的存取，為此變更預留空間（例如 factor 內部透過 `getPosition(Values&, Key)` 取值，而非直接 `values.at<Point3>()`）。

#### P4-D InSAR

| ID | 項目 | 複雜度 |
|---|---|---|
| P4D.1 | InSAR 產品介面（PS-InSAR / SBAS 時序） | 中 |
| P4D.2 | 時空對齊：InSAR 的 LOS 位移 → GNSS 的 ENU | 中高 |
| P4D.3 | `InsarLosFactor`：約束 GNSS 測站與 InSAR 散射體間的相對位移 | 高 |
| P4D.4 | InSAR 參考點與 GNSS 基準的統一 | **高** — InSAR 是相對量測，需 GNSS 提供絕對基準。這其實是 GNSS 對 InSAR 的最大貢獻 |
| P4D.5 | 大氣相位屏（APS）的聯合估計 | 高 |

**InSAR 融合的正確定位**：不是「InSAR 改善 GNSS 精度」，而是**「GNSS 為 InSAR 提供絕對基準與時間密度，InSAR 為 GNSS 提供空間密度」**。融合的產品是一張**時空連續的變形場**，這是 Digital Twin 的直接輸入。這個定位必須在專案溝通中講清楚，避免期待落差。

#### P4-E Digital Twin

| ID | 項目 | 複雜度 |
|---|---|---|
| P4E.1 | 變形場輸出介面（時空網格 + 不確定性） | 中 |
| P4E.2 | 結構模型（FEM）的模態/剛度作為 factor | 極高 |
| P4E.3 | 雙向耦合：孿生體預測 → factor 先驗；量測 → 模型參數更新 | 極高 |
| P4E.4 | What-if 情境重解 | 中高 |

### 技術風險（Phase 4）

| 風險 | 機率 | 衝擊 | 緩解 |
|---|---|---|---|
| P4C.2 的狀態表述變更破壞既有 factor | 高 | 高 | Phase 3 就做介面隔離；P4C 獨立分支開發 |
| 多感測器時間同步誤差 | 高 | 高 | 硬體層 PPS 同步；圖中顯式估計時間偏移作為變數 |
| InSAR 的參考框架與 GNSS 不一致 | 高 | 高 | P4D.4 專門處理；需要地面控制點 |
| 感測器故障造成整個圖污染 | 中 | 高 | 每個感測器的 factor 群組獨立的 robust kernel 與健康監控；可執行期停用 |
| FEM 模型不準確反而劣化估計 | 中 | 中高 | 模型 factor 使用寬鬆 σ；持續監控其殘差；可停用 |
| 範圍蔓延（scope creep） | **高** | 高 | 每個感測器獨立立項、獨立 Go/No-Go |

### 里程碑

| 里程碑 | 驗收準則 |
|---|---|
| **M4.1** | 傾斜儀整合上線，與 GNSS 垂直分量的一致性通過驗證 |
| **M4.2** | 氣象整合，溫度-位移耦合模型的殘差顯著小於未耦合版本 |
| **M4.3** | IMU 整合，可輸出 ≥ 10 Hz 的位移時序，與獨立參考（如雷射位移計）比對通過 |
| **M4.4** | InSAR 融合產出第一張時空變形場，GNSS 測站處的一致性 < 5 mm |
| **M4.5** | Digital Twin 介面上線，孿生體可消費帶不確定性的變形場 |

---

# 12. Risks and Mitigations

## 12.1 風險登記表

風險等級 = 機率 × 衝擊。優先處理「高」等級者。

| ID | 類別 | 風險描述 | 機率 | 衝擊 | 等級 | 緩解措施 | 負責階段 |
|---|---|---|---|---|---|---|---|
| **R-01** | 技術 | `ddres()` 純函式化改變 EKF 數值行為，破壞既有產線 | 中 | 高 | **高** | 獨立 PR、byte-diff 回歸、`t_ddres.c`、部署後觀察期 | P3 |
| **R-02** | 技術 | Jacobian 符號或佈局轉換錯誤，導致收斂慢或錯誤收斂（且不會崩潰，難察覺） | 中 | 高 | **高** | 每個 factor 的有限差分驗證（G4）；合成資料端到端測試 | P2, P3 |
| **R-03** | 技術 | SD 相位偏差秩虧未妥善處理，Cholesky 失敗或解無意義 | 中 | 高 | **高** | §4.4.3 三重錨定；`t_fgo_anchor` 專用測試 | P2 |
| **R-04** | 產品 | Phase 2 的線性版低估 FGO 效益，導致錯誤的 No-Go | 高 | 高 | **高** | 實作「外迴圈重線性化」組態近似 A′（P2.14 必要項） | P2 |
| **R-05** | 技術 | C++ 例外跨 C ABI 邊界造成未定義行為 | 中 | 高 | **高** | 每個 `extern "C"` 強制 try/catch 樣板；review checklist；ASan CI | P3 |
| **R-06** | 產品 | Site constraint 的先驗設錯，把真實變形當作雜訊壓抑掉 | 中 | 高 | **高** | 持續監控 site constraint 的正規化殘差（§10.4）；預設 σ 寬鬆；文件明確警示 | P2, P3 |
| **R-07** | 工程 | ABI 破壞造成新舊版本混用時記憶體損毀 | 高 | 中 | **高** | 欄位加尾端、提升版本號、release note、SOVERSION、全專案同步部署 | P3 |
| **R-08** | 技術 | iSAM2 重線性化尖峰破壞 NRT 延遲保證 | 中 | 中 | 中 | 逾時保護 + EKF fallback + 非同步 worker + 定期重建圖 | P3 |
| **R-09** | 工程 | `resetsysopts()` 遺漏預設值，舊設定檔意外啟用 FGO | 中 | 高 | **高** | M14 + 專門回歸測試 G3 | P3 |
| **R-10** | 技術 | 邊緣協方差計算成為效能瓶頸 | 中 | 中 | 中 | 只算 AR 子集；Cholesky；實測後決定 | P3 |
| **R-11** | 資源 | 團隊缺乏 GTSAM/factor graph 經驗 | 中 | 高 | **高** | Phase 2 作為學習期（低風險環境）；訓練預算；必要時外部支援 | P1, P2 |
| **R-12** | 資源 | 關鍵人員離職造成知識斷層 | 中 | 高 | **高** | 文件化（本文件 + 程式碼註解）；至少 2 人熟悉 FGO 模組；避免單點 |
| **R-13** | 產品 | 效益不足以證成成本（真實結果） | 中 | 高 | 中 | 這是 Phase 2 設計要回答的問題；接受 No-Go；Phase 1 的投資（回歸框架）獨立有價值 | P2 |
| **R-14** | 工程 | 範圍蔓延（Phase 4 無限擴張） | 高 | 中 | **高** | 每個感測器獨立立項與 Go/No-Go；Phase 3 完成前不啟動 Phase 4 |
| **R-15** | 技術 | 長時運行的記憶體洩漏或圖退化 | 中 | 中 | 中 | Valgrind/ASan CI；30 天連續測試（M3.6）；定期重建圖 | P3 |
| **R-16** | 相依 | GTSAM 版本升級造成 API 破壞 | 中 | 中 | 中 | 鎖定版本；`fgo_gtsam.cpp` 隔離所有 GTSAM API；Docker 固定環境 | P2–P4 |
| **R-17** | 部署 | GTSAM 相依（Boost/TBB/Eigen）在目標邊緣裝置上難以部署 | 中 | 中 | 中 | `ENABLE_FGO=OFF` 預設；提供靜態連結建置；評估 sliding window 的輕量替代（OQ-3） |
| **R-18** | 技術 | 多路徑圖／sidereal filtering 誤把真實變形當作多路徑消除 | 中 | 高 | **高** | 建圖前先扣除趨勢；定期重建並比對；文件明確警示；不預設啟用 | P3 |
| **R-19** | 產品 | Confidence score 未經校準即被下游當作可信指標使用 | 中 | 中 | 中 | §10.5 的校準驗收準則；未校準前在 schema 中標示 `"calibrated": false` | P2 |
| **R-20** | 技術 | 浮點/編譯器差異使 byte-diff 回歸不可行 | 中 | 中 | 中 | 綁定 compiler 版本；提供 tolerance 比對作為次要關卡；記錄實際 flags | P1 |

## 12.2 最高優先的五項

若資源有限，以下五項的緩解措施**不可妥協**：

1. **R-04（Phase 2 低估效益）** — 這會導致專案在錯誤的資訊上被終止。「外迴圈重線性化」組態必須實作。
2. **R-01（`ddres()` 重構）** — 這會破壞既有產線。byte-diff 回歸與 PR 獨立化必須執行。
3. **R-02（Jacobian 錯誤）** — 這是靜默失敗，會浪費大量除錯時間。有限差分驗證必須對每個 factor 執行。
4. **R-03（秩虧）** — 這會讓 Phase 2 在起步就卡住。必須最早處理。
5. **R-06 / R-18（先驗/多路徑圖抹除真實變形）** — 這是**業務層面最嚴重的失敗模式**：系統看起來運作正常、數字漂亮，但漏報了真實的結構危險。必須有獨立的監控機制。

> **R-06 與 R-18 值得特別強調**：監測系統的失敗模式不對稱。誤報造成成本；**漏報可能造成人命損失**。所有會「讓資料看起來更平滑」的機制（緊的 Motion Factor、Site Constraint、多路徑圖、sidereal filtering、jerk 平滑）都有壓抑真實訊號的風險。**設計原則：這類機制一律預設關閉或設寬鬆，且必須有殘差監控。**

## 12.3 回退策略

| 情境 | 回退動作 |
|---|---|
| FGO 單 epoch 失敗 | 自動 fallback 至 EKF（M1 已設計） |
| FGO 持續失敗（> N epoch） | 記錄告警；持續使用 EKF；不影響產線 |
| FGO 造成 NRT 延遲超標 | 切換 `pos1-solver=ekf`（純設定變更，不需重新部署） |
| 發現 FGO 解有系統性偏差 | 同上；並以 batch 模式離線重解確認 |
| 需完全移除 FGO | `cmake -DENABLE_FGO=OFF` 重建；核心 C 程式碼的重構（PR-1~PR-4）保留（它們本身無害且改善了程式碼結構） |

**設計上的重要性質**：即使 FGO 整體被放棄，Phase 1 的回歸框架與 Phase 3 的純函式化重構仍是對程式庫的淨改善。**這降低了專案的整體風險——不存在「全輸」的結局。**

## 12.4 授權相容性分析

| 元件 | 授權 | 相容性 |
|---|---|---|
| RTKLIB / RTKLIB-EX | BSD-2-Clause | 基線 |
| GTSAM | BSD-3-Clause | **相容**。BSD-3 = BSD-2 + 非背書條款。二者皆為寬鬆授權，可同時分發，但需在散布物中同時保留兩份著作權聲明與免責聲明 |
| Eigen | MPL2（部分 LGPL，可以 `EIGEN_MPL2_ONLY` 排除） | **相容**（MPL2 為檔案級 copyleft，僅在修改 Eigen 本身時才需開源該檔案）。建議定義 `EIGEN_MPL2_ONLY` 以完全避免 LGPL 部分 |
| Boost | Boost Software License 1.0 | **相容**（寬鬆） |
| TBB（GTSAM 選用） | Apache-2.0 | **相容**。注意 Apache-2.0 含專利條款，與 BSD 混合分發沒有問題 |

**行動項**：
1. 在 release 產物中加入 `THIRD_PARTY_LICENSES.txt`，收錄 GTSAM、Eigen、Boost、TBB 的授權全文。
2. 定義 `EIGEN_MPL2_ONLY` 編譯巨集。
3. 若產品為閉源商業散布，上述組合仍可行（無 GPL/AGPL 成分），但**建議在 Phase 2 結束前由法務確認**，避免後期才發現問題。

---

# 13. Future Extensions

> 本章描述 Phase 4 之後的擴充方向。這些**不在當前規劃範圍內**，但架構設計必須為其預留空間。

## 13.1 IMU / 加速度計

**Factor**：`gtsam::CombinedImuFactor`（GTSAM 內建，含 IMU bias 隨機遊走）。

**架構影響**：位置變數需從 `Point3` 升級為 `NavState`（位置 + 速度 + 姿態，SE(3) 流形上的變數）。這會改變所有既有 factor 的變數型別。

**預留設計（Phase 3 必須做）**：
```cpp
/* fgo_factor.cpp 中，所有 factor 透過此介面取位置，不直接 values.at<Point3>() */
inline gtsam::Point3 getPosition(const gtsam::Values& v, gtsam::Key k);
/* Phase 4 只需改此函式的實作為 v.at<NavState>(k).position() */
```

**價值**：SHM 的高頻位移重建（§9.4）；GNSS 短暫失鎖期間的位置保持；姿態量測（對傾斜監測直接有用）。

## 13.2 傾斜儀 / 傾角感測器

**Factor**：`TiltFactor`，約束測站的姿態，或約束多個測站間的相對高程變化率。

```
單站姿態約束（需 NavState 或獨立姿態變數）：
    e = [roll, pitch]_measured - [roll, pitch](R_k)

雙站相對傾斜約束（僅需 Point3，Phase 3 即可實作）：
    e = [(p_a(k) - p_b(k)) - (p_a(0) - p_b(0))]_vertical / L_ab  -  Δtilt_measured
```

**第二式的價值**：它**不需要 NavState**，可以在 Phase 3 的架構下直接實作。這使傾斜儀成為**最容易整合的第二感測器**，建議作為 Phase 4 的第一個項目。

## 13.3 InSAR

**融合的正確架構**：

```
             GNSS 測站 (少數、絕對、高時間頻率)
                      |
                      | 提供絕對基準與時間內插
                      v
         +------------------------------+
         |   聯合 factor graph          |
         |   變數: GNSS 測站位置 p_i(k) |
         |         InSAR 散射體位移 d_j(k)|
         |         大氣相位屏 APS(k)     |
         +------------------------------+
                      ^
                      | 提供空間密度
                      |
         InSAR 散射體 (大量、相對、低時間頻率)
```

**關鍵 Factor**：

```
InsarLosFactor:
    e = ŝ_LOSᵀ · [d_j(k) - d_j(k')]  -  Δφ_LOS_measured · λ/(4π)
    其中 ŝ_LOS 為衛星視線方向（ENU）

GnssInsarTieFactor（把 InSAR 錨定到 GNSS）：
    e = d_j(k) - p_i(k)      對於位於 GNSS 測站附近的散射體 j
    σ 依散射體與測站的距離而定
```

**主要難點**：時空對齊、相位解纏的模糊性（`2π` 整數倍，本質上與 GNSS 的週波未定值是同一類問題，可用類似的整數估計方法處理——**這是一個有趣的架構共通性**）、大氣相位屏、參考點選擇。

## 13.4 氣象

| 資料 | 用途 | Factor |
|---|---|---|
| 溫度 | 橋梁熱脹冷縮建模 | §4.2.7-A 的溫度耦合 Site Factor |
| 氣壓、溫度、溼度 | 取代 `tropmodel()` 的標準大氣 | 修改對流層 factor 的先驗與 mapping |
| 降雨 | 邊坡孔隙水壓 → 位移的耦合先驗 | `RainfallCreepFactor`：以累積雨量預測潛變速率的先驗 |
| 風速 | 橋梁動態響應 | 作為 SHM 分析的解釋變數（不一定進圖） |

**最低成本、最高效益者：溫度**。溫度感測器極便宜，而溫度是橋梁與建物短期位移的**主要驅動因子**。扣除溫度效應後的殘餘變形，才是結構健康的真實指標。**建議在 Phase 4 最優先實作。**

## 13.5 Digital Twin

**FGO 與 Digital Twin 的天然契合點**：

| Digital Twin 需求 | FGO 提供 |
|---|---|
| 帶不確定性的狀態 | 聯合協方差（不只對角） |
| 物理模型作為約束 | FEM 模態/剛度可直接編碼為 factor |
| 假設檢驗（what-if） | 修改 factor 並重解 |
| 多源資料融合 | 圖的本質 |
| 時空連續的變形場 | GNSS + InSAR 融合輸出 |
| 反演模型參數 | 把 FEM 參數作為圖變數 |

**雙向耦合的願景**：

```
孿生體（FEM）  --預測位移場-->  factor graph 的先驗 factor
      ^                                    |
      |                                    | 估計的變形場
      +---- 更新模型參數（剛度、邊界條件）----+
```

這是一個**聯合狀態-參數估計（joint state-parameter estimation）**問題，在 factor graph 中是自然的（把 FEM 參數當作變數即可）。

**現實評估**：這是研究級的目標，複雜度極高（P4E.2/P4E.3 標為「極高」）。**不應在 Phase 4 的初期承諾此項。** 建議先完成 P4E.1（單向：FGO → 孿生體的變形場輸出），累積經驗後再評估雙向耦合。

## 13.6 其他可能方向

| 方向 | 說明 | 成熟度 |
|---|---|---|
| **PPP-FGO** | 以 PPP 取代 RTK，免除基站。對偏遠監測站（邊坡、山區）價值高 | 中（需 §6.2 的 `ppp_res()` 純函式化） |
| **PPP-RTK / SSR** | 網路改正 + FGO | 中 |
| **多測站網解** | 把整個監測網當作一張圖聯合解算 | 中高（§4.2.7-D 的自然延伸） |
| **深度學習的多路徑分類** | 以 NN 判定 LOS/NLOS，輸出至 robust kernel 的權重 | 中（文獻活躍） |
| **學習式誤差模型** | 以歷史資料學習 `varerr()` 的替代模型 | 中低 |
| **GNSS 反射測量（GNSS-R）** | 以反射訊號量測水位/雪深，作為額外的環境資料 | 低（與監測的關聯較弱） |

---

# 14. Open Questions — 待決議事項

以下問題需要在對應階段開始前由專案關係人決議。**它們不阻塞本文件的定稿，但會影響實作細節。**

| ID | 問題 | 影響 | 需決議時點 | 建議 |
|---|---|---|---|---|
| **OQ-1** | 監測測站的實際取樣率為何？30 s、1 min，還是 1 Hz？ | 直接決定 batch FGO 的可行性與 §8.2 的成本估計 | Phase 1 | 若為 30 s 以上，batch 全日解完全可行，應列為主要產品 |
| **OQ-2** | 是否有具獨立真值的測站（控制點、全站儀、水準測量）？ | 決定 Phase 2 能否驗證「精度」而非只有「精密度」；決定 confidence score 能否校準 | **Phase 1（最優先）** | 若無，應優先建置至少一處；否則 Phase 2 的結論會有本質限制 |
| **OQ-3** | 目標部署平台是否能承載 GTSAM（Boost/TBB/Eigen）？ | 若邊緣裝置無法部署，需考慮 §3.3 的受限架構 B 變體作為 fallback | Phase 2 | 先確認實際的邊緣裝置規格與 OS |
| **OQ-4** | NRT 的延遲要求具體為何？告警路徑是否有硬性上限？ | 決定 solver 選擇與是否需要非同步設計 | Phase 2 | §4.7.3 的預算為建議值，需業務確認 |
| **OQ-5** | `src/CMakeLists.txt` 的 `if (GCC)` 失效問題（§6.9 陷阱 2）是否要修正？何時修正？ | 修正會改變最佳化等級，使所有基準失效 | **Phase 1 開始前** | 建議：Phase 1 建立基準前先決定。若要修，就在 Phase 1 最開始修，然後才建基準 |
| **OQ-6** | 是否需要維持與 upstream RTKLIB-EX 的同步能力？ | 決定修改的侵入程度可以到什麼地步；`ddres()` 重構會使 merge 變困難 | Phase 3 開始前 | 若需持續同步，應盡量把 FGO 邏輯放在 `src/fgo/`，核心修改壓到最小 |
| **OQ-7** | 多測站聯合解算是否在範圍內？ | §4.2.7-D 的剛體約束需要跨測站的資料匯流，這會改變系統架構（單站解算 → 中心化解算） | Phase 3 | 這是很高價值的功能，但需要架構層級的決策 |
| **OQ-8** | Confidence score 的消費者是誰？告警系統會如何使用？ | 決定 §10.5 的校準目標與嚴謹度 | Phase 2 | 若用於自動告警，校準要求遠高於僅供人工參考 |
| **OQ-9** | 資料保留政策為何？batch 再處理需要保留多久的原始觀測？ | 決定儲存架構與 batch FGO 的可行窗口 | Phase 2 | 建議至少保留 90 天原始 RINEX/RTCM |
| **OQ-10** | Phase 4 的感測器優先序是否與 §11.4 的建議一致？ | 決定 Phase 4 的規劃 | Phase 3 結束前 | §11.4 的排序基於技術成熟度；業務優先序可能不同 |
| **OQ-11** | 上游 RTKLIB-EX 的合併節奏為何？下一次上游合併預計何時？ | 上游合併會位移本文件的行號，且可能改動 `zdres()`/`ddres()` 的內容，影響 §6.1 的重構方案 | Phase 1 開始前 | 建議在一次上游合併剛完成後才啟動 Phase 3 的 `ddres()` 重構，以取得最長的穩定窗口；並確認是否需長期維持與上游同步（見 OQ-6） |

---

# Appendix A — 關鍵原始碼索引

> 行號取自 branch `chore/sync-upstream-20260820` @ `512ed8ac`，已逐項驗證（詳見文件開頭「行號基準」）。上游後續合併會使行號位移，**請以函式名稱為準**。

## A.1 解算入口與流程

| 函式 | 位置 | 說明 |
|---|---|---|
| `rtkpos()` | `src/rtkpos.c:2438` | 唯一的模式派發點；FGO 注入點 |
| `relpos()` | `src/rtkpos.c:2068` | RTK 相對定位主流程（EKF） |
| `pntpos()` | `src/pntpos.c:646` | 單點定位 |
| `pppos()` | `src/ppp.c:1226` | PPP 主流程 |
| `rtkinit()` / `rtkfree()` | `src/rtkpos.c:2332` / `:2371` | `rtk_t` 生命週期；FGO context 綁定點 |
| `rtkpos()` 呼叫點（NRT） | `src/rtksvr.c:727` | 即時串流路徑 |
| `rtkpos()` 呼叫點（後處理） | `src/postpos.c:486` | 後處理路徑 |

## A.2 觀測模型與殘差

| 函式 | 位置 | 說明 |
|---|---|---|
| `zdres()` | `src/rtkpos.c:1049` | 零差殘差；已近純函式 |
| `zdres_sat()` | `src/rtkpos.c:988` | 單顆衛星的零差殘差 |
| `ddres()` | `src/rtkpos.c:1240` | 雙差殘差 + Jacobian + 協方差；純函式化目標 |
| `ddcov()` | `src/rtkpos.c:1128` | DD 協方差（對角 + rank-1） |
| `rescode()` | `src/pntpos.c:277` | SPP 偽距殘差 |
| `resdop()` | `src/pntpos.c:549` | 都卜勒殘差 |
| `ppp_res()` | `src/ppp.c:974` | PPP 殘差（**非** `resph()`） |
| `constbl()` | `src/rtkpos.c:1145` | 基線長度約束；Site Constraint Factor 的前身 |
| `prectrop()` | `src/rtkpos.c:1185` | 精密對流層 + Jacobian |
| `validobs()` | `src/rtkpos.c:1115` | 觀測有效性檢查 |
| `valpos()` | `src/rtkpos.c:2039` | 事後殘差驗證 |
| `test_sys()` | `src/rtkpos.c:1210` | 系統分組（m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN） |

## A.3 誤差模型

| 函式 | 位置 | 說明 |
|---|---|---|
| `varerr()` (RTK) | `src/rtkpos.c:406` | DD 觀測變異數；`1/sin²(el)` |
| `varerr()` (SPP) | `src/pntpos.c:51` | SPP 觀測變異數；`1/sin(el)` |
| `varerr()` (PPP) | `src/ppp.c:336` | PPP 觀測變異數 |
| `snrmask()` | `src/pntpos.c:96` | SNR 遮罩 |
| `gettgd()` | `src/pntpos.c:78` | 群延遲 |

## A.4 狀態更新（EKF）

| 函式 | 位置 | 說明 |
|---|---|---|
| `udstate()` | `src/rtkpos.c:957` | 狀態時間更新總入口 |
| `udpos()` | `src/rtkpos.c:488` | 位置/速度/加速度；過程雜訊在 `:566-571` |
| `udion()` | `src/rtkpos.c:576` | 電離層 |
| `udtrop()` | `src/rtkpos.c:606` | 對流層 |
| `udrcvbias()` | `src/rtkpos.c:635` | GLONASS 接收機偏差 |
| `udbias()` | `src/rtkpos.c:831` | 相位偏差；cycle slip 重設 |
| `initx()` | `src/rtkpos.c:461` | 狀態初始化（inline） |

## A.5 週波跳脫偵測

| 函式 | 位置 |
|---|---|
| `detslp_code()` | `src/rtkpos.c:660` |
| `detslp_ll()` | `src/rtkpos.c:680` |
| `detslp_gf()` | `src/rtkpos.c:729` |
| `detslp_dop()` | `src/rtkpos.c:758` |

## A.6 模糊度解算

| 函式 | 位置 | 說明 |
|---|---|---|
| `manage_amb_LAMBDA()` | `src/rtkpos.c:1909` | AR 總管（partial AR、arfilter） |
| `resamb_LAMBDA()` | `src/rtkpos.c:1770` | LAMBDA 解算 |
| `ddidx()` | `src/rtkpos.c:1563` | DD 索引表 |
| `restamb()` | `src/rtkpos.c:1633` | 還原固定解 |
| `holdamb()` | `src/rtkpos.c:1661` | fix-and-hold |
| `lambda()` | `src/lambda.c:180` | LAMBDA 主函式 |
| `lambda_reduction()` | `src/lambda.c:214` | Z 變換降相關 |
| `lambda_search()` | `src/lambda.c:247` | 整數搜尋 |

## A.7 數值運算

| 函式 | 位置 | 說明 |
|---|---|---|
| `filter()` | `src/rtkcmn.c:1479` | Kalman 量測更新（含零狀態壓縮） |
| `filter_()` | `src/rtkcmn.c:1459` | 核心實作 |
| `smoother()` | `src/rtkcmn.c:1520` | 前後向平滑 |
| `lsq()` | `src/rtkcmn.c:1428` | 最小平方 |
| `matinv()` | `src/rtkcmn.c:1164` / `:1386` | **兩份實作**（內建 / LAPACK），以 `#ifdef` 切換 |
| `matmul()` | `src/rtkcmn.c:1125` / `:1203` | 同上，雙路徑 |

## A.8 狀態索引巨集

| 巨集 | 位置 |
|---|---|
| `NF/NP/NI/NT/NL/NB/NR/NX` | `src/rtkpos.c:79-86` |
| `II/IT/IL/IB` | `src/rtkpos.c:89-92` |

## A.9 型別與常數

| 項目 | 位置 |
|---|---|
| `obsd_t` | `src/rtklib_types.h:13` |
| `nav_t` | `src/rtklib_types.h:287` |
| `sol_t` | `src/rtklib_types.h:347` |
| `prcopt_t` | `src/rtklib_types.h:469` |
| `solopt_t` | `src/rtklib_types.h:535` |
| `ssat_t` | `src/rtklib_types.h:617` |
| `ambc_t` | `src/rtklib_types.h:642` |
| `rtk_t` | `src/rtklib_types.h:651` |
| `snrmask_t` | `src/rtklib_types.h:464` |
| `opt_t` | `src/rtklib_types.h:457` |
| `PMODE_*` | `src/rtklib_const.h:314-323` |
| `SOLQ_*` | `src/rtklib_const.h:332-339` |
| `MAXSAT` / `MAXOBS` / `NFREQ` | `src/rtklib_const.h:172` / `:177` / `:83` |
| `VAR_POS` / `VAR_VEL` / `VAR_ACC` | `src/rtkpos.c:66-69` |
| `chisqr[]` | `src/rtklib_api.h:8` |
| `sysopts[]` | `src/options.c:67`（宣告於 `rtklib_api.h:14`） |

## A.10 建置系統

| 項目 | 位置 | 註記 |
|---|---|---|
| `project(... LANGUAGES C CXX ...)` | `CMakeLists.txt:12` | C++ 已啟用 |
| 版本讀取 | `CMakeLists.txt:3-9` | 從 `rtklib.h` 的 `VER_RTKLIB`/`PATCH_LEVEL` |
| `aux_source_directory(.)` | `src/CMakeLists.txt:26` | 只收集 `src/*.c`，不含子目錄 |
| `add_library(rtklib SHARED ...)` | `src/CMakeLists.txt:38` | 共享函式庫，ABI 相關 |
| `if (GCC)` | `src/CMakeLists.txt:10`, `:16` | **`GCC` 從未被 `set()`，條件恆為假** |
| `extern "C"` guard | `src/rtklib.h:51-52`（`extern "C" {`）；`:85-87` 引入三個拆分檔 | C++ 可直接引入 |

---

# Appendix B — 建議新增設定項全表

## B.1 `prcopt_t` 對應的設定項（`options.c` 的 `sysopts[]`）

| 設定項名稱 | 型別 | 對應欄位 | 預設 | 說明 |
|---|---|---|---|---|
| `pos1-solver` | enum | `fgo_solver` | `0:ekf` | `0:ekf,1:fgo-batch,2:fgo-sliding,3:fgo-isam2` |
| `fgo-window` | double | `fgo_window` | 300.0 | 滑動視窗長度（s） |
| `fgo-maxiter` | int | `fgo_maxiter` | 30 | 最佳化器最大迭代次數 |
| `fgo-robust` | enum | `fgo_robust` | `1:huber` | `0:none,1:huber,2:cauchy,3:tukey,4:gnc` |
| `fgo-huber-delta` | double | `fgo_kparam[0]` | 1.345 | Huber 調節常數 |
| `fgo-cauchy-c` | double | `fgo_kparam[1]` | 2.3849 | Cauchy 調節常數 |
| `fgo-tukey-c` | double | `fgo_kparam[2]` | 4.6851 | Tukey 調節常數 |
| `fgo-ddcov` | enum | `fgo_ddcov` | `0:block` | `0:block,1:diag,2:latent` |
| `fgo-elwmodel` | enum | `fgo_elwmodel` | `0:rtklib` | `0:rtklib,1:exp,2:sitemap` |
| `fgo-mp-adaptive` | enum | `fgo_mpadapt` | `0:off` | CMC 自適應加權（§5.4.5） |
| `fgo-scale-est` | enum | `fgo_scaleest` | `1:on` | MAD 尺度估計（§5.5.6） |
| `fgo-scale-min` | double | `fgo_scaleclamp[0]` | 0.5 | 尺度下限 |
| `fgo-scale-max` | double | `fgo_scaleclamp[1]` | 5.0 | 尺度上限 |
| `fgo-maxinno-scale` | double | `fgo_innoscale` | 3.0 | 硬門檻放寬倍數（§5.5.5） |
| `fgo-tdcp` | enum | `fgo_tdcp` | `1:on` | TDCP factor |
| `fgo-jerkconst` | enum | `fgo_jerk` | `0:off` | Jerk 平滑（§4.2.6，**謹慎使用**） |
| `fgo-arcstitch` | enum | `fgo_stitch` | `0:off` | 弧段縫合（§4.1.3） |
| `fgo-relin-thres` | double | `fgo_relinthres` | 0.05 | iSAM2 重線性化門檻（m） |
| `fgo-timeout-ms` | int | `fgo_timeout` | 2000 | 單 epoch 逾時保護（ms） |
| `fgo-async` | enum | `fgo_async` | `0:off` | 非同步 worker thread（§4.7.2） |
| `fgo-siteconst` | string | `fgo_sitefile` | "" | Site constraint 設定檔路徑 |
| `fgo-insight-out` | string | `fgo_insightfile` | "" | AI Insight NDJSON 輸出路徑 |
| `fgo-mpmap` | string | `fgo_mpmapfile` | "" | 測站多路徑圖檔路徑（§5.4.3） |

**更正（已驗證）**：原草案的 `fgo-scale-clamp` 型別為「double×2」，但 `options.c` 的 `opt_t` 沒有這種型別——`format=1` 只以 `atof()` 讀取單一 double（`options.c` 的 `str2opt()`）。既有慣例是拆成兩個具名選項：`prcopt_.baseline[2]` 即拆為 `pos2-baselen` / `pos2-basesig`。因此本表改為 `fgo-scale-min` / `fgo-scale-max`。

## B.2 相關的既有設定項（FGO 會使用，不新增）

| 設定項 | 欄位 | FGO 的使用方式 |
|---|---|---|
| `pos1-posmode` | `mode` | 決定 Motion Factor 的形式（static/kinematic） |
| `pos1-dynamics` | `dynamics` | 決定是否有 `V(k)`、`A(k)` 變數 |
| `pos1-ionoopt` | `ionoopt` | 決定是否有 `I(k,sat)` 變數 |
| `pos1-tropopt` | `tropopt` | 決定是否有 `T(k,r)` 變數 |
| `stats-err*` | `err[]` | 誤差模型（§5.2） |
| `stats-prn*` | `prn[]` | Motion/Velocity/Accel Factor 的協方差（§4.2.4–4.2.6） |
| `stats-stdbias` | `std[0]` | SD 偏差錨定 prior 的 σ（§4.4.3） |
| `pos2-*` (AR 相關) | `modear` 等 | AR pipeline 完全沿用（§4.4） |
| `pos2-rejphase/rejcode` | `maxinno[]` | 乘以 `fgo-maxinno-scale` 後作為預過濾（§5.5.5） |

## B.3 Site Constraint 設定檔格式（草案）

獨立的 INI 風格檔案，由 `fgo-siteconst` 指定。**不塞進 `prcopt_t`**，因為結構複雜且測站相依。

```ini
# fgo_site.conf  —  Site Constraint 設定
# 座標系：所有方向向量以 ENU 表示（單位向量，會自動正規化）

[site]
id          = BRIDGE-P03
type        = bridge                  # bridge | slope | building | generic
datum_mode  = first_week_mean         # first_week_mean | fixed_ecef | control_point
datum_ecef  = -2956123.4567, 4864321.8901, 2681234.5678   # datum_mode=fixed_ecef 時使用
datum_epoch = 2026-01-01T00:00:00Z

# --- 方向性約束（橋梁縱向、邊坡面外）---
[constraint.longitudinal]
enable      = 1
type        = directional
direction   = 0.866, 0.500, 0.000     # ENU 單位向量（橋軸方向）
sigma_mm    = 20.0                     # 活動支承端；固定支承端用 3.0
# 溫度耦合（可選，需 fgo-weather 資料）
temp_couple = 1
temp_alpha  = 1.2e-5                   # 熱膨脹係數 (1/degC)
temp_length = 120.0                    # 伸縮縫間距 (m)
temp_ref_c  = 20.0

# --- 邊坡：面外約束 ---
[constraint.slope_normal]
enable      = 0
type        = directional
direction   = 0.643, -0.766, 0.000     # 滑動面法向量
sigma_mm    = 5.0

# --- 邊坡：單向潛變先驗（單邊 hinge loss）---
# 警告：僅在已確認為單調潛變的邊坡啟用。降雨回彈或凍融會造成系統性偏差。
[constraint.slope_creep]
enable      = 0
type        = one_sided
direction   = -0.643, 0.766, -0.259    # 傾向（向下）
sigma_mm    = 10.0

# --- 建物：水平主導約束 ---
[constraint.horizontal]
enable      = 0
type        = planar
plane_normal= 0.0, 0.0, 1.0            # 約束垂直於此法向量的分量（即水平面內）
sigma_mm    = 3.0

# --- 基準先驗 ---
[constraint.datum_prior]
enable      = 1
type        = prior
sigma_mm    = 30.0, 30.0, 60.0         # E, N, U

# --- 多測站剛體約束 ---
[rigid_body]
enable      = 1
peers       = BRIDGE-P01, BRIDGE-P02, BRIDGE-P04
mode        = baseline_length          # baseline_length | full_se3
sigma_mm    = 2.0
# full_se3 模式會額外估計結構的整體平移與旋轉

# --- 監控門檻（供 AI Insight 的 status 欄位）---
[monitor]
warn_normalized  = 2.0                 # 正規化殘差 > 此值 -> status="warn"
alarm_normalized = 4.0                 # -> status="alarm"
```

---

# Appendix C — 名詞與符號對照

## C.1 縮寫

| 縮寫 | 英文 | 中文 |
|---|---|---|
| FGO | Factor Graph Optimization | 因子圖最佳化 |
| EKF | Extended Kalman Filter | 擴展卡爾曼濾波器 |
| GNC | Graduated Non-Convexity | 漸進非凸化 |
| IRLS | Iteratively Reweighted Least Squares | 迭代重加權最小平方 |
| DD | Double Difference | 雙差 |
| SD | Single Difference | 單差 |
| UD | Undifferenced | 非差 |
| TDCP | Time-Differenced Carrier Phase | 時間差分載波相位 |
| CMC | Code Minus Carrier | 碼相減載波 |
| MP | Multipath | 多路徑 |
| AR | Ambiguity Resolution | 模糊度解算 |
| ZTD | Zenith Total Delay | 天頂總延遲 |
| STEC | Slant Total Electron Content | 傾斜總電子含量 |
| LOS | Line of Sight | 視線方向 |
| NLOS | Non-Line-of-Sight | 非視線 |
| MAD | Median Absolute Deviation | 中位數絕對偏差 |
| NIS | Normalized Innovation Squared | 正規化新息平方 |
| NEES | Normalized Estimation Error Squared | 正規化估計誤差平方 |
| SHM | Structural Health Monitoring | 結構健康監測 |
| NRT | Near Real-Time | 近即時 |
| APS | Atmospheric Phase Screen | 大氣相位屏 |
| ABI | Application Binary Interface | 應用程式二進位介面 |

## C.2 符號

| 符號 | 意義 |
|---|---|
| `p_k` | 第 `k` 個 epoch 的測站位置（ECEF, m） |
| `v_k`, `a_k` | 速度、加速度 |
| `b` | 站間單差相位偏差（cycle 或 m） |
| `N` | 整數週波未定值 |
| `e_los` | 接收機指向衛星的單位向量 |
| `ρ` | 幾何距離 + 改正項（computed range） |
| `v` | RTKLIB 的新息（innovation），`v = z - h(x)` |
| `H` | RTKLIB 的設計矩陣（狀態為列儲存） |
| `A` | GTSAM 的 Jacobian（量測為列） |
| `R`, `Σ` | 量測協方差 |
| `Q` | 過程雜訊協方差 |
| `σ` | 標準差 |
| `u` | 正規化殘差 `r/σ` |
| `ρ(u)` | Robust loss 函式 |
| `w(u)` | IRLS 權重 |
| `Δt` | epoch 間時間差 |
| `nb` | DD 區塊內的觀測數 |
| `nv` | 總觀測數 |
| `nx` | 狀態向量維度 |

## C.3 本文件與 RTKLIB 命名的對照

| 本文件用語 | RTKLIB 對應 | 備註 |
|---|---|---|
| PPP 殘差函式 | `ppp_res()` | 需求文件稱 `resph()`，本程式庫中不存在 |
| 站間單差相位偏差 | `x[IB(s,f,opt)]` | 非 DD 未定值 |
| 弧段（arc） | 無直接對應 | FGO 新增概念；EKF 以 `udbias()` 就地重設取代 |
| DD 協方差 | `ddcov()` 產生的 `R` | 對角 + rank-1 結構 |
| 測站約束 factor | `constbl()` 的泛化 | 基線長度約束是其特例 |
| Solver 模式 | `prcopt_t.fgo_solver`（新增） | **不是** `PMODE_*` |

---

## 文件結束

**修訂記錄**

| 版本 | 日期 | 變更 | 作者 |
|---|---|---|---|
| 0.1 | 2026-08-20 | 初版草稿 | — |

**待辦：本文件定稿前需完成**

1. 由專案關係人回答 §14 的 Open Questions（特別是 OQ-1、OQ-2、OQ-5，這三項影響 Phase 1 的執行）。
2. 確認 §11 各 Phase 的資源投入與時程（本文件刻意未提供人月估計）。
3. 法務確認 §12.4 的授權組合（若為閉源商業散布）。
4. 技術評審會議：重點審查 §3.5 的架構推薦、§4.2.1 的 DD 協方差處理、§6.1 的 `ddres()` 重構方案。

