# RTKLIB BLS (Batch Least Squares) 解算核心完整解說

本文件說明 Java RTKLIB RTK static 的 BLS 解算管線，與 EKF 管線文件
（`RTK_EKF_Pipeline.md`）互為對照。BLS 原始碼位於 git history 的
`2cb4a85^:rtklib-java/src/main/java/com/gnss/rtklib/positioning/BatchSolver.java`
（2076 LOC，18 commits 開發歷史）。

---

## 一、BLS vs EKF：架構差異總覽

| 面向 | EKF (Rtkpos) | BLS (BatchSolver) |
|------|-------------|-------------------|
| 處理模式 | Sequential（逐 epoch） | Batch（所有 epoch 同時） |
| 狀態向量 | SD bias（MAXSAT×NF 固定大小） | DD bias（動態大小，per-arc） |
| 參數化 | x = [pos, iono, trop, hwbias, SD_N] | x = [pos(3), DD_N₁, DD_N₂, ...] |
| Ref sat | 每 epoch 可變（仰角最高） | 全域固定（可見時間最長） |
| 法方程 | 隱式（P⁻¹ 逐步更新） | 顯式累加 N, b |
| AR 方法 | LAMBDA + ratio test | Connected Component WL/NL + PAR |
| 固定回饋 | Fix-and-hold（注入 EKF） | 無（一次性求解） |
| Outlier 處理 | 單 epoch maxinno 閾值 | Huber IRLS + post-fix iterative recovery |
| 求解方式 | Kalman filter update | Schur complement + block-diagonal inverse |

---

## 二、狀態向量佈局

```
x = [ pos(3) | DD_N₁ | DD_N₂ | ... | DD_Nₙ ]
     ├─ 3 ──┤├────────── nAmb ──────────────┤
                                        nx = 3 + nAmb
```

### 與 EKF 的關鍵差異

**EKF**：每顆衛星佔一個固定 slot（`IB(sat, f, opt)`），nx=207 固定大小，
但只有 ~70 個活躍狀態（其餘 x=0, P=0）。

**BLS**：只為實際觀測到的 DD pair 建立參數，每個參數是一個 `AmbParam`：

```java
class AmbParam {
    int refSat;      // reference satellite (1-based)
    int sat;         // non-reference satellite (1-based)
    int freq;        // frequency index (0=L1, 2=L5)
    int segment;     // segment number (cycle slip 後遞增)
    int startEpoch;  // 此段的起始 epoch
    int endEpoch;    // 此段的結束 epoch
}
```

同一顆衛星如果有 cycle slip，會產生多個 `AmbParam`（不同 segment），
每段獨立估計。典型 3h@1Hz：nAmb ≈ 30-60 個 DD 參數。

### DD 參數化 vs SD 參數化

| | SD (EKF) | DD (BLS) |
|---|---|---|
| 狀態數 | MAXSAT×NF = 204（固定） | 動態，取決於 DD pair 數 |
| Rank deficiency | 有（接收機端整數不確定） | 無（DD 天然 full-rank） |
| Ref sat 切換 | 透明（不影響狀態向量） | 不切換（全域固定 ref sat） |
| Cycle slip | 重設單顆衛星 | 新增獨立 segment |

---

## 三、BLS 完整管線流程

```
BatchSolver.solve()
│
├─ 1. SPP: sppPosition()                    ← 粗略位置 (L136)
│
├─ 2. 預處理: preprocessEpochs()             ← 匹配 rover/base，計算衛星位置 (L142)
│   ├─ satposs()                             ← 共用 EKF 的衛星位置計算
│   ├─ zdres(base)                           ← 共用 EKF 的非差殘差
│   └─ selsat()                              ← 共用 EKF 的共視選星
│
├─ 3. 全域 ref sat 選擇: chooseRefSats()      ← 依最長可見時間 (L150)
│
├─ 4. DD ambiguity 掃描: scanDdAmbiguities()  ← Cycle slip 偵測 + arc 管理 (L153)
│
├─ 5. 初始 ambiguity 估計: initDdAmbFromZdres() ← Code-phase 差 (L175)
│
├─ 6. Gauss-Newton 迭代 (最多 10 次)          ← (L185-399)
│   │
│   │  對每個 epoch:
│   │  ├─ zdres(rover) + zdres(base)          ← 共用 EKF 的非差殘差
│   │  ├─ makeDdObs()                         ← 建構 DD 觀測 (BLS 專用)
│   │  └─ 累加法方程 N += Hᵀ W H, b += Hᵀ W v
│   │
│   │  累加完成後:
│   │  ├─ Schur complement 求解               ← 分離 pos 和 amb
│   │  └─ 收斂檢查: |Δpos| < 0.1mm 且 |Δamb| < 0.01 cycle
│   │
├─ 7. 計算協方差                              ← (L405-453)
│   ├─ Block-diagonal Naa⁻¹                  ← 利用 Connected Component 結構
│   ├─ Schur complement → Qpp (位置協方差)
│   └─ Marginal Qaa (含位置不確定度的 ambiguity 協方差)
│
├─ 8. Connected Component WL/NL AR           ← (L523-762)
│   ├─ 分組: findArComponents()              ← Union-Find by (refSat, freq, epoch overlap)
│   ├─ 每組獨立 AR: runComponentAr()
│   │   ├─ 品質門檻: fractional screening
│   │   ├─ WL fix: LAMBDA on (N_L1 - N_L5)
│   │   ├─ WL 約束 → 條件協方差收縮
│   │   ├─ NL fix: LAMBDA on (N_L1 + N_L5) + single-freq
│   │   └─ PAR with LD sorting (fallback)
│   ├─ Per-component post-fix validation
│   └─ 固定解位置修正 (條件協方差)
│
├─ 9. Fallback: 單步 PAR                     ← (L765-891)
│   └─ LD conditional variance sorting → 逐步縮減 subset
│
└─ 回傳 BatchResult {pos, qr, stat, ratio, ...}
```

---

## 四、核心步驟詳解

### 4.1 全域 Ref Sat 選擇 — chooseRefSats() (L1275)

**與 EKF 的關鍵差異**：EKF 每 epoch 選仰角最高的衛星作為 ref sat，
BLS 在全部 epoch 上選**可見時間最長**的衛星，確保 DD pair 的時間連續性。

```
對每個星座 sys、每個頻率 f:
  1. 統計每顆衛星在所有 epoch 的可見次數 visCount[sat][f]
  2. 優先選 dual-freq (L1+L5) 可見時間最長的衛星作為所有頻率的 ref
  3. 如果沒有 dual-freq 衛星，每個頻率獨立選 ref
```

穩定的 ref sat 最大化了每個 DD pair 的 segment 長度，
提供更強的幾何約束。代價是 ref sat 不一定是每個 epoch 仰角最高的，
但長時間平均下來更穩定。

### 4.2 DD Ambiguity 掃描 — scanDdAmbiguities() (L1377)

三階段逐 epoch 掃描：

**Phase 1: GF Cycle Slip 偵測** (L1394-1421)

```
對每顆衛星:
  GF = SD_L1(m) - SD_L2(m)          ← Geometry-Free combination
  如果 |GF - GF_prev| > 0.05m:
    slipMap[sat][f] = true            ← 標記 cycle slip
```

GF 組合消除了幾何距離變化，只反映電離層和 ambiguity 的變化。
短時段內電離層變化 < 0.01m/epoch，所以 0.05m 的閾值可靠偵測 cycle slip。

**Phase 2: Ref Sat Slip 傳播** (L1423-1441)

```
如果 ref sat 在頻率 f 上有 slip:
  該星座所有使用此 ref sat 的 DD pair 都標記為需要新 segment
```

這是 BLS 獨有的問題——EKF 用 SD 參數化不受 ref sat slip 影響，
但 BLS 的 DD 參數直接包含 ref sat 的 phase，ref sat slip 等同所有 DD pair 同時 slip。

**Phase 3: 建立/更新 Segment** (L1443-1510)

```
對每顆衛星 sat、每個頻率 f:
  如果是新出現、自身 slip、或 ref sat slip:
    結束當前 AmbParam（設定 endEpoch）
    建立新 AmbParam（新 segment number）
  如果衛星消失:
    結束當前 AmbParam
```

### 4.3 法方程累加 — solve() 內部迴圈 (L185-295)

**觀測方程**（與 EKF 的 ddres 相同的數學模型）：

```
DD 載波: v = DD_phase - hPos·δr - λ·N_DD
DD 偽距: v = DD_code  - hPos·δr

hPos = -e_ref + e_sat                ← LOS 方向差（與 EKF ddres L822-825 相同）
```

**法方程累加**：

```
對每個 DD 觀測 dd:
  w = 1/var                           ← 量測權重（DD 方差的倒數）

  // Huber IRLS（iter ≥ 1 時啟用）
  如果 |v|/σ > 4.0:  w *= 4.0σ/|v|    ← 下調離群觀測的權重

  // 位置-位置區塊
  N[0:3, 0:3] += hPos × w × hPosᵀ
  b[0:3]      += hPos × w × v

  // 如果是載波觀測（有 ambiguity）:
  N[0:3, amb]  += hPos × w × λ
  N[amb, 0:3]  += λ × w × hPosᵀ
  N[amb, amb]  += λ × w × λ
  b[amb]       += λ × w × v
```

**DD 方差處理**：

```java
obs.var = varRef + varJ;    // DD phase variance (L1626)
```

注意：這是**對角近似**——忽略了同一 ref sat 的 DD 之間的相關性。
EKF 的 `ddcov()` 處理了這個相關性。BLS 選擇簡化處理，
依賴 Huber 權重和 post-fix 驗證來補償。

上次開發記錄中的教訓：
> *Sherman-Morrison DD full covariance is a clear win: +1.6% fix rate*

但最終版本的主迴圈用的是對角近似 + Huber（L244-260），
Sherman-Morrison 可能只在協方差計算階段使用。

### 4.4 Schur Complement 求解 (L321-398)

法方程分塊：

```
[ Npp  Npa ] [ δpos ] = [ bp ]
[ Nap  Naa ] [ δamb ]   [ ba ]
```

Schur complement 消去 ambiguity：

```
1. Naa⁻¹  (block-diagonal inverse，利用 Connected Component 結構)
2. tmp    = Npa × Naa⁻¹                         (3 × nAmb)
3. Nred   = Npp - tmp × Npaᵀ                     (3 × 3，reduced normal equation)
4. bred   = bp  - tmp × ba                        (3 × 1)
5. Nred⁻¹                                         (3 × 3 inverse)
6. δpos   = Nred⁻¹ × bred                         (3 × 1，位置更新)
7. δamb   = Naa⁻¹ × (ba - Napᵀ × δpos)           (nAmb × 1，ambiguity 更新)
```

**Block-diagonal Naa⁻¹** (L338-355)：

不同 Connected Component 之間的 ambiguity 互相獨立
（不共享任何 DD 觀測），所以 Naa 是 block-diagonal 的。
每個 block 獨立求逆，比完整 nAmb×nAmb 求逆高效得多。

例如：GPS 組 20 個 amb + GAL 組 10 個 amb →
求逆成本 O(20³ + 10³) = 9000 vs O(30³) = 27000。

### 4.5 協方差計算 (L405-511)

**位置協方差**（Schur complement）：

```
Qpp = (Npp - Npa × Naa⁻¹ × Npaᵀ)⁻¹            (3 × 3)
```

**位置-ambiguity 交叉協方差**：

```
Qpa = -Qpp × Npa × Naa⁻¹                       (3 × nAmb)
```

**Marginal ambiguity 協方差**（含位置不確定度）：

```
Qaa = Naa⁻¹ + (Npa × Naa⁻¹)ᵀ × Qpp × (Npa × Naa⁻¹)    (nAmb × nAmb)
```

Qaa 比單純的 Naa⁻¹ 大——因為位置不確定度會「傳播」到 ambiguity 的不確定度中。
LAMBDA 需要這個 marginal Qaa 才能正確評估整數解的信心。

### 4.6 Connected Component WL/NL AR — runComponentAr() (L1005)

**分為三層 fallback**：

```
Layer 1: WL/NL 兩步 AR (dual-freq 衛星)
  ├─ 品質門檻: avg fractional < 0.25, bad frac (>0.35) < 30%
  ├─ WL fix: LAMBDA on (N_L1 - N_L5)        ← Wide-Lane，長波長 ~86cm
  ├─ WL 約束 → 條件協方差收縮 Qaa
  ├─ NL fix: LAMBDA on (N_L1 + N_L5)        ← Narrow-Lane，短波長 ~10cm
  └─ NL1/NL5 還原: N_L1 = (WL+NL)/2, N_L5 = (NL-WL)/2

Layer 2: 單步 PAR with LD sorting (component-level fallback)
  ├─ LD 分解 Qaa → 條件方差 D[i] 排序
  └─ 從全部到 minAR 逐步縮減，找到 ratio ≥ threshold 的子集

Layer 3: 全域單步 PAR (L765-891)
  └─ 跨所有 component 的 PAR fallback
```

**WL/NL 兩步的數學原理**：

```
WL = N_L1 - N_L5    λ_WL ≈ 86cm (寬巷)
NL = N_L1 + N_L5    λ_NL ≈ 11cm (窄巷)

WL 波長長 → 容易固定（對未建模誤差容忍度高）
WL 固定後 → 施加約束 → NL 的不確定度收縮
NL 在收縮後的協方差下更容易固定

最後還原:
N_L1 = (WL + NL) / 2
N_L5 = (NL - WL) / 2
要求 WL + NL 為偶數（L1197-1199 有 parity correction）
```

**WL 約束的協方差收縮**（條件協方差）(L1082-1098)：

```
T = [I, -I] (WL = L1 - L5 的線性組合矩陣)
QaaC = Qaa - (T×Qaa)ᵀ × (T×Qaa×Tᵀ)⁻¹ × (T×Qaa)
```

WL 固定後，L1 和 L5 ambiguity 的協方差顯著收縮，
使得 NL 的 LAMBDA 搜索空間大幅縮小。

### 4.7 Post-fix 驗證 (L567-624, L689-761)

**兩層驗證**：

**Layer 1: Per-component 驗證** (L585-608)

```
對每個 connected component:
  compRms = computePostFixPhaseRms(固定解, 此 component 的 ambiguity)
  如果 compRms > pfThreshold (0.015m for 1Hz):
    拒絕整個 component → 回退到 float
```

**Layer 2: 全域驗證 + Iterative Recovery** (L689-758)

```
計算全域 postFixRms
如果 postFixRms > pfThreshold:
  迴圈最多 MAX_RECOVERY 次:
    找殘差最大的固定 ambiguity (findWorstFixedAmb)
    回退為 float → 重算固定解 → 重算 RMS
    直到 RMS ≤ threshold 或 subset 太小
```

Iterative recovery 的直覺：如果一兩個 ambiguity 固定錯了，
找到它們、回退為 float，其餘正確的固定解仍然可用。

### 4.8 固定解位置修正（條件協方差）

```
δa = float_amb - fixed_int                      ← ambiguity 修正量
pos_fix = pos_float - Qpa × Qaa⁻¹ × δa          ← 位置修正
Q_fix = Qpp - Qpa × Qaa⁻¹ × Qpaᵀ               ← 修正後位置協方差
```

與 EKF 的 `resambLAMBDA()` (L1245-1257) 數學完全相同，
但 BLS 用 Qpa / Qaa（顯式計算），EKF 用 P 的子塊。

**安全檢查** (L669-676)：

```
如果 |pos_fix - pos_float| > 50mm:
  拒絕固定解（修正量太大，疑似 wrong fix）
```

---

## 五、共用基礎設施分析

### 已共用的（可直接復用）

| 模組 | 呼叫位置 | 用途 |
|------|---------|------|
| `Spp.pntpos()` | L1925 | SPP 初始位置 |
| `EphemerisCalc.satposs()` | L1986 | 衛星位置/鐘差 |
| `Rtkpos.zdres()` | L204, L225, L1718, L1737, ... | 非差殘差 |
| `Rtkpos.selsat()` | L2026 | 共視衛星選擇 |
| `Rtkpos.varerr()` | L1570, L1596 | SD 量測方差 |
| `Rtkpos.baseline()` | L232 | 基線長度 |
| `Spp.testsnr()` | L1564 | SNR mask |
| `Lambda.lambda()` | L804, L1073, L1170, L1244 | LAMBDA 整數搜索 |
| `Lambda.LD()` | L774, L1145, L1224 | LDL' 分解 |
| `Rtkpos.computeAdaptiveArThreshold()` | L810, L1176, L1250 | FFRT 自適應閾值 |
| `MatrixUtil.matmul/matinv` | 全域 | 矩陣運算 |

### BLS 專用模組（無法共用）

| 模組 | LOC | 原因 |
|------|----:|------|
| `chooseRefSats()` | 76 | 全域穩定 ref sat（EKF 每 epoch 選） |
| `scanDdAmbiguities()` | 141 | DD arc tracking + GF slip + ref slip propagation |
| `makeDdObs()` | 136 | DD 觀測建構（不同於 ddres 的 H 矩陣佈局） |
| `findDdAmbIdx()` | 12 | DD 參數查找 |
| `initDdAmbFromZdres()` | 91 | DD ambiguity 初始化（平均 code-phase 差） |
| `solve()` 主邏輯 | ~350 | Gauss-Newton + Schur complement |
| `findAmbiguityComponents()` | 23 | Union-Find 分組 |
| `findArComponents()` | 25 | AR-eligible 分組 |
| `runComponentAr()` | 259 | WL/NL 兩步 AR |
| `computePostFixPhaseRms()` | 53 | Post-fix 驗證 |
| `findWorstFixedAmb()` | 59 | Iterative recovery |
| `preprocessEpochs()` | 93 | Epoch 匹配和預處理 |
| **合計 BLS 專用** | **~1318** | |

---

## 六、BLS 與 EKF 的觀測方程對比

### 相同部分

```
DD 殘差:  v = (y_rover_ref - y_base_ref) - (y_rover_j - y_base_j)
位置偏導: hPos = -e_ref + e_j
DD 方差:  var = varRef + varJ (來自 varerr())
```

兩者都用 `zdres()` 計算非差殘差 y，用 `varerr()` 計算 SD 方差。

### 不同部分

| | EKF (ddres) | BLS (makeDdObs) |
|---|---|---|
| 殘差中的 ambiguity 項 | `v -= (c/f_ref)·x[IB(ref)] - (c/f_j)·x[IB(j)]` | `v -= λ · ambValues[ddAmbIdx]` |
| H 矩陣 ambiguity 列 | `H[IB(ref)] = c/f_ref`, `H[IB(j)] = -c/f_j` | `H[3+ddAmbIdx] = λ` |
| DD 協方差 | `ddcov()` 處理 ref sat 相關性 | 對角近似 `var = varRef + varJ` |
| Iono/trop 項 | 有（如啟用估計） | 無（假設短基線消除） |
| Outlier rejection | `maxinno` 硬閾值 | Huber IRLS（soft downweight） |

**關鍵觀察**：BLS 把每個 DD pair 的 ambiguity 建模為一個獨立參數（λ·N），
而 EKF 把 ref sat 和 non-ref sat 的 SD bias 分別建模為兩個獨立參數。
DD 參數化的好處是 full-rank，壞處是 ref sat 切換需要新 segment。

---

## 七、資料流圖

```
全部觀測資料 (rover + base, 所有 epoch)
    │
    ▼
┌─────────────────────────────────────────┐
│  SPP: 粗略位置 rr                        │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  preprocessEpochs                        │
│  ├─ satposs (共用)                       │
│  ├─ zdres(base) (共用)                   │
│  └─ selsat (共用)                        │
│  → List<EpochData>                       │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  chooseRefSats: 全域穩定 ref sat          │
│  → refSatMap[sys][freq]                  │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  scanDdAmbiguities: GF slip + arc 管理   │
│  → List<AmbParam> (DD pair 描述)         │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  initDdAmbFromZdres: code-phase 差初始化  │
│  → ambValues[] (float DD ambiguity)      │
└─────────────┬───────────────────────────┘
              ▼
      ┌────── Gauss-Newton 迭代 ──────┐
      │                                │
      │  對每個 epoch:                  │
      │  ├─ zdres(rover) (共用)        │
      │  ├─ zdres(base) (共用)         │
      │  ├─ makeDdObs (BLS 專用)       │
      │  └─ N += HᵀWH, b += HᵀWv     │
      │                                │
      │  Schur complement 求解:         │
      │  ├─ Naa⁻¹ (block-diagonal)    │
      │  ├─ δpos = (Npp-Npa·Naa⁻¹·Npaᵀ)⁻¹·bred
      │  └─ δamb = Naa⁻¹·(ba-Nap·δpos) │
      │                                │
      │  pos += δpos, amb += δamb       │
      └────────────┬───────────────────┘
                   ▼
┌─────────────────────────────────────────┐
│  協方差: Qpp, Qpa, Qaa (marginal)       │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  Connected Component AR                  │
│  ├─ findArComponents (Union-Find)        │
│  ├─ 每組: runComponentAr                 │
│  │  ├─ 品質門檻 (fractional screening)    │
│  │  ├─ WL fix (LAMBDA)                   │
│  │  ├─ WL 約束 → Qaa 條件收縮            │
│  │  ├─ NL fix (LAMBDA)                   │
│  │  └─ L1/L5 還原 + parity correction    │
│  ├─ Per-component post-fix 驗證           │
│  └─ Iterative recovery (worst-amb 回退)   │
│                                          │
│  Fallback: 單步 PAR (LD sorting)          │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  固定解位置修正                            │
│  pos_fix = pos - Qpa × Qaa⁻¹ × δa       │
│  Q_fix = Qpp - Qpa × Qaa⁻¹ × Qpaᵀ      │
│                                          │
│  安全檢查: |fix_corr| < 50mm             │
└─────────────┬───────────────────────────┘
              ▼
         BatchResult
         {pos, qr, stat=FIX/FLOAT, ratio}
```

---

## 八、設計決策與直覺

### 為什麼 BLS 用 DD 參數化？

EKF 用 SD 是為了避免 ref sat 切換的 pivot 問題。
BLS 用 DD 是因為：

1. **Full-rank**：DD 參數化天然沒有 rank deficiency，
   不需要額外的 datum constraint
2. **Ref sat 全域固定**：BLS 可以選全域最穩定的 ref sat，
   不像 EKF 需要每 epoch 選最高仰角
3. **Segment 自然處理 slip**：每段 arc 是獨立的 DD 參數，
   slip 不影響其他 segment

### 為什麼用 Schur Complement 而不是直接求逆 N？

```
直接求逆: O(nx³) = O((3 + nAmb)³)
Schur:    O(nAmb³) + O(3³) + O(3 × nAmb²)
```

nAmb ≈ 40 時差異不大，但 Schur 的好處是：
- Naa 是 block-diagonal → 可以按 component 分別求逆
- 自然分離位置和 ambiguity 的協方差
- 條件協方差公式直接可用（AR 需要）

### 為什麼需要 Connected Component？

不同星座（GPS、GAL、BDS）的 DD 使用不同的 ref sat，
它們的 ambiguity 之間沒有直接觀測關聯 → Naa 的跨 component 區塊為零。

利用這個結構：
1. **Naa⁻¹** 可以分 block 求逆（效率更高、數值更穩定）
2. **AR** 可以分 component 進行（GPS 固定但 GAL 未固定 → GPS 仍有效）
3. **Post-fix 驗證** 分 component 進行（一個 component 錯不影響其他）

### 為什麼 DD 方差用對角近似？

完整的 DD 協方差（如 EKF 的 ddcov）需要對每個 epoch 的所有 DD 建構 R 矩陣，
然後計算 Hᵀ R⁻¹ H。這在 BLS 中成本更高（累加數千個 epoch）。

上次開發記錄提到 Sherman-Morrison 可以高效處理，但最終版本選擇
對角近似 + Huber IRLS，犧牲少量精度換取簡潔性。

如果要改進，恢復 Sherman-Morrison DD 協方差預計可以提升 1-2% fix rate。

### 為什麼 Huber IRLS 不是所有迭代都啟用？

```java
if (iter >= 1) {  // 只在第二次迭代後啟用 Huber
```

第一次迭代時 ambiguity 初始值來自 code-phase 差（精度 ~3m），
殘差很大但不是 outlier。如果第一次就下調權重，會阻礙 LS 收斂。
等到第二次迭代，ambiguity 已經接近真值，大殘差才是真的 outlier。

---

## 九、歷史性能數據

來源：`memory/project_bls_solver.md`

### 3h@1Hz

| 配置 | Fix rate | Median error | Max error |
|------|---------|-------------|-----------|
| Baseline | 97.6% | 6.6mm | 50.5mm |
| + SM covariance | 99.2% | 5.0mm | 22.8mm |
| + QZS merge + adaptive | **98.4%** | **4.8mm** | **22.8mm** |

### 10min@1Hz

| 配置 | Fix rate | Median error |
|------|---------|-------------|
| Baseline | 65.7% | 43.1mm |
| + SM covariance | 74.3% | 14.0mm |
| + QZS merge + adaptive | **93.6%** | **11.2mm** |

### 對比當前 EKF

| | EKF (修正後) | BLS (最佳) |
|---|---|---|
| 3h fix rate | **99.6%** | 98.4% |
| 3h RMS | **4.6mm** | 4.8mm |
| Wrong fix | 未測量 | **0%** |
| 10min fix rate | 未測量 | **93.6%** |

---

## 十、復用評估

### 可直接復用的程式碼

| 模組 | 狀態 | 備註 |
|------|------|------|
| `chooseRefSats()` | 可直接復用 | 穩定成熟 |
| `scanDdAmbiguities()` | 可直接復用 | 含 GF slip + ref slip propagation |
| `makeDdObs()` | 可直接復用 | 使用 zdres 輸出 |
| `initDdAmbFromZdres()` | 可直接復用 | 但 zdres 重複計算可優化 |
| `findAmbiguityComponents()` | 可直接復用 | Union-Find，穩定 |
| `findArComponents()` | 可直接復用 | 同上 |
| `runComponentAr()` | 可直接復用 | WL/NL 核心，最複雜也最關鍵 |
| Schur complement 求解 | 可直接復用 | 數學正確，已驗證 |
| Post-fix 驗證 | 可直接復用 | Per-component + iterative recovery |
| `preprocessEpochs()` | 可復用但需重構 | 大量 arraycopy 可簡化 |

### 需要新增/改造的部分

| 需求 | 說明 |
|------|------|
| 增量累積模式 | 新增：支援逐 epoch 累加 N, b，而非一次性讀入全部 epoch |
| 滑動窗口管理 | 新增：epoch 過期時從 N, b 中減去該 epoch 的貢獻 |
| Cycle slip 的增量處理 | 改造：slip 時清除受影響 segment 的 N, b 貢獻 |
| PostProcessor 接入 | 改造：在 PostProcessor 中新增 BLS 呼叫路徑 |
| Sherman-Morrison 恢復 | 可選：恢復完整 DD 協方差處理 |

### 估計工作量

| 項目 | LOC |
|------|----:|
| 原始 BatchSolver 復用 | ~1800（已有） |
| 增量累積 wrapper | ~200 |
| 滑動窗口管理 | ~150 |
| PostProcessor 接入 | ~50 |
| 測試 | ~300 |
| **合計新增** | **~700** |
