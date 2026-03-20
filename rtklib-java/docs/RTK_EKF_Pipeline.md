# RTKLIB RTK Static EKF 解算核心完整解說

本文件說明 Java RTKLIB RTK static 模式的 EKF 解算管線，涵蓋狀態向量佈局、
每 epoch 處理流程、核心數學推導、以及關鍵設計決策背後的直覺。

所有行號引用對應 `rtklib-java/src/main/java/com/gnss/rtklib/positioning/Rtkpos.java`，
除非另行標註。

---

## 一、狀態向量佈局

EKF 狀態向量 `x[nx]` 與協方差矩陣 `P[nx×nx]` 皆為 column-major 儲存。
狀態分為「真實狀態」（位置、電離層、對流層、硬體偏差）與「模糊度狀態」（phase bias）。

```
x = [ Position | Ionosphere | Troposphere | GLO_HWbias | PhaseBias          ]
     ├── NP ──┤├── NI ─────┤├── NT ──────┤├── NL ────┤├── NB ─────────────┤
     └────────────── na (= NR = NP+NI+NT+NL) ────────┘
                                                        nx = na + NB
```

### 各區塊維度

| 區塊 | 符號 | 維度公式 | 典型值 (static, IFLC, nf=1) |
|------|------|---------|---------------------------|
| 位置 | NP | `dynamics ? 9 : 3` | 3 (ECEF XYZ) |
| 電離層 | NI | `ionoopt==EST ? MAXSAT : 0` | 0 |
| 對流層 | NT | `tropopt>=EST ? 2~6 : 0` | 0 |
| GLO 硬體偏差 | NL | `glomodear==AUTOCAL ? 2 : 0` | 0 |
| Phase bias | NB | `MAXSAT × NF` | 204 (MAXSAT×1) |
| **合計** | nx | na + NB | **207** |

### 狀態索引函數

定義於 `RtkState.java`：

| 函數 | 公式 | 用途 |
|------|------|------|
| `IB(sat, f, opt)` | `NR + MAXSAT×f + sat-1` | 衛星 sat 在頻率 f 的 phase bias 索引 |
| `II(sat, opt)` | `NP + sat-1` | 衛星 sat 的電離層狀態索引 |
| `IT(rcv, opt)` | `NP + NI + NT/2 × rcv` | 接收機 rcv 的對流層狀態索引 |
| `IL(f, opt)` | `NP + NI + NT + f` | GLO 硬體偏差索引 |

### 協方差矩陣

`P[nx×nx]` 是 column-major dense 矩陣：`P[i + j*nx]` = Cov(x[i], x[j])。

```
P = ┌─────────┬─────────┐
    │ P_rr    │ P_ra    │   P_rr: 真實狀態的自協方差 (na×na)
    │ (na×na) │ (na×NB) │   P_ra: 真實-模糊度交叉協方差
    ├─────────┼─────────┤   P_aa: 模糊度自協方差 (NB×NB)
    │ P_ar    │ P_aa    │
    │ (NB×na) │ (NB×NB) │   注意: 只有可見衛星的子塊非零
    └─────────┴─────────┘
```

---

## 二、每 Epoch 管線流程

```
rtkpos()                                    ← 入口 (L1623)
│
├─ 1. SPP                                   ← 粗略定位 (L1611)
│
├─ 2. relpos()                              ← RTK 主邏輯 (L1631)
│   │
│   ├─ 2a. satposs()                        ← 衛星位置/鐘差 (L1413)
│   ├─ 2b. zdres(base)                      ← 基站非差殘差 (L1431)
│   ├─ 2c. selsat()                         ← 選共視衛星 (L1448)
│   ├─ 2d. udstate()                        ← EKF 時間更新 (L1452)
│   │   ├─ udpos()                          ← 位置狀態
│   │   ├─ udion() / udtrop()               ← 電離層/對流層 (如啟用)
│   │   ├─ udrcvbias()                      ← GLO 硬體偏差 (如 AUTOCAL)
│   │   └─ udbias()                         ← phase bias 時間更新
│   │
│   ├─ 2e. 迭代 EKF 觀測更新                  ← niter=2~3 次 (L1483)
│   │   ├─ zdres(rover)                     ← rover 非差殘差
│   │   ├─ ddres()                          ← 雙差殘差 + H + R
│   │   └─ filter()                         ← EKF 觀測更新
│   │
│   ├─ 2f. Post-fit 殘差檢查                  ← (L1511)
│   ├─ 2g. 整數模糊度解算                      ← LAMBDA + ratio test (L1541)
│   │   ├─ manageAmbLAMBDA()               ← AR 管理 + satellite dropout
│   │   ├─ 固定解驗證                        ← zdres(xa) + ddres(xa)
│   │   └─ holdamb()                        ← fix-and-hold 回饋
│   │
│   └─ 2h. 儲存結果 + 更新計數器              ← (L1585)
│
└─ 回傳 solution
```

以下逐步詳解。

---

## 三、Step 1：SPP 單點定位

**程式碼**：`Spp.pntpos()` (L1611)

用 pseudorange 對所有可見衛星做最小二乘，估計 rover 的粗略 ECEF 位置和接收機鐘差。
精度 ~2-10m，足以作為 EKF 的線性化參考點。

**副產品**：每顆衛星的方位角 (azimuth) 和仰角 (elevation)，供後續仰角遮罩和權重計算使用。

---

## 四、Step 2a-2c：衛星位置、基站殘差、共視選星

### 2a. satposs()

用廣播星曆（或精密星曆）計算每顆衛星在信號發射時刻的 ECEF 座標和鐘差。
包含地球自轉修正（Sagnac 效應）和相對論修正。

### 2b. zdres() — 非差殘差 (L686)

對每顆衛星 i，計算觀測值與模型值的差（zero-difference residual）：

```
ρᵢ = |rsᵢ - rr|                     ← 幾何距離 (Geometry.geodist)
ρᵢ -= c × dtsᵢ                      ← 衛星鐘差修正
ρᵢ += Mh(el) × ZHD                  ← 對流層乾延遲先驗修正 (Saastamoinen)

載波殘差:  y[i]    = Lᵢ × (c/fᵢ) - ρᵢ      (公尺)
偽距殘差:  y[i+nf] = Pᵢ - ρᵢ               (公尺)
```

殘差中仍包含：接收機鐘差（被 SD 消除）、電離層延遲（被 DD 或估計消除）、
phase bias（EKF 估計）。

**IFLC 模式**（iono-free linear combination）：用雙頻組合消除電離層一階效應：

```
L_IF = C₁·L₁ + C₂·L₂      其中 C₁ = f₁²/(f₁²-f₂²), C₂ = -f₂²/(f₁²-f₂²)
P_IF = C₁·P₁ + C₂·P₂
```

### 2c. selsat()

找 rover 和 base 都觀測到的衛星，回傳：
- `sat[]` — 共視衛星 PRN 列表
- `iu[]` — 各衛星在 rover 觀測陣列中的索引
- `ir[]` — 各衛星在 base 觀測陣列中的索引
- `ns` — 共視衛星數

---

## 五、Step 2d：EKF 時間更新 (Prediction)

**程式碼**：`udstate()` (L522)

### 5.1 位置更新 udpos() (L362)

| 模式 | 行為 |
|------|------|
| Static | 首 epoch 用 SPP 初始化 `x[0:3]`，之後**不動**（無 process noise） |
| Kinematic | 每 epoch 用 SPP 重新初始化（無動態模型） |
| Dynamic | 狀態轉移：位置 += 速度×dt，速度 += 加速度×dt，加上 process noise |

Static 模式下位置的 process noise 為零，意味著 P 的位置區塊只會因為觀測更新而收縮，
永遠不會膨脹。

### 5.2 Phase Bias 更新 udbias() (L397)

這是最複雜的時間更新步驟，分為五個階段：

#### 階段 1：清除 cycle slip 標誌 (L402-407)

```java
rtk.ssat[sat[i] - 1].slip[k] &= 0xFC;   // 保留高位，清除 LLI_SLIP | LLI_HALFC
```

#### 階段 2：Cycle slip 偵測 (L410-418)

四種偵測機制平行運作：

| 偵測器 | 原理 | 靈敏度 |
|--------|------|--------|
| `detslpDop()` | Doppler 預測的相位變化 vs 實際相位變化 | 中（受 Doppler 噪聲限制）|
| `detslpCode()` | signal code 變化（如 L1C→L1W）| 高（code 變化必然 slip）|
| `detslpLl()` | 接收機報告的 LLI flag | 依接收機品質 |
| `detslpGf()` | Geometry-free (L1-L2) 相位跳變 | 高（消除幾何變化後的跳變）|

任一偵測器觸發都會設置 `slip[k] |= LLI_SLIP`。

#### 階段 3：Outage 處理 (L428-446)

```
對每顆衛星 i、每個頻率 k:
  outc[k]++                               ← 觀測消失計數器遞增
  如果 outc > maxout 且 x[j] ≠ 0:
    initx(0, 0, j)                        ← 完全清除 bias 狀態
    outc = 0
```

長時間消失的衛星 bias 被完全歸零，確保重新出現時從乾淨的初始估計開始。

#### 階段 4：Process noise 注入 + Slip 重置 (L448-465)

```
P[j,j] += σ²_phase × |dt|               ← bias 方差隨時間增長

如果偵測到 cycle slip 或 rejc ≥ 2:
  x[j] = 0                               ← 清除 bias
  lock[k] = -minlock                      ← 進入冷卻期（例如 -20）
```

#### 階段 5：初始 bias 估計 (L467-515)

對新衛星（x[j]==0）初始化 bias：

```
bias[i] = SD_phase(cycles) - SD_code × f/c     ← code-phase 差估計 bias

offset = mean(bias[i] - x[IB(i)]) for active sats  ← 計算系統偏移
x[j] += offset/cnt  for all active sats              ← 修正所有活躍 bias

新衛星:
  initx(bias[i], σ²_code, IB(i))                ← 用 code 精度初始化方差
  lock[k] = -minlock                             ← 進入冷卻期
```

偏移修正（offset correction）確保新衛星的 bias 與已有的 bias 在同一個 datum 上。

### 5.3 lock 計數器機制

```
衛星初始化 → lock = -minlock (例如 -20)
每 epoch:
  如果 lock < 0 → lock++                  ← 冷卻中
  如果 lock ≥ 0 → 可參與 AR (ddidx 檢查)
  如果 fix 成功且 fix[f]≥2 → lock++       ← 累積信心

20 epoch 的冷卻期讓 EKF 有時間收斂 bias 估計，
避免尚未收斂的 bias 污染 LAMBDA 搜索。
```

---

## 六、Step 2e：迭代 EKF 觀測更新 (Measurement Update)

**程式碼**：L1483-1508，迴圈 `niter` 次（通常 2-3 次）。

每次迭代包含三個子步驟：zdres(rover) → ddres → filter。

### 6.1 ddres() — 雙差殘差與設計矩陣 (L740)

#### 雙差的構成

對每個星座 m、每個觀測類型（phase / code）、每個頻率 f：

1. **選 reference satellite**：該星座中仰角最高的可見衛星（L791-797）
2. **對每顆其他衛星 j**：計算 DD 殘差

```
v_DD = (y_rover_ref - y_base_ref) - (y_rover_j - y_base_j)
     = SD_rover(ref-j) - SD_base(ref-j)
```

#### 雙差消除的量

| 誤差源 | SD (rover-base) | DD (ref-sat) |
|--------|:---:|:---:|
| 接收機鐘差 | 消除 | — |
| 衛星鐘差 | — | 消除 |
| 電離層（短基線） | ~消除 | ~消除 |
| 對流層（短基線） | ~消除 | ~消除 |

#### 觀測方程（DD 載波相位）

```
v = DD_obs - h(x)

h(x) = DD_geometric + DD_iono + DD_trop + DD_bias

DD_geometric = (e_ref - e_j) · δr          δr = rover 位置偏差
DD_iono = s_ref·I_ref - s_j·I_j            s = ±(f₁/f)² 電離層係數
DD_trop = (m_rover_ref - m_rover_j)·ZWD_rover - (m_base_ref - m_base_j)·ZWD_base
DD_bias = (c/f_ref)·N_ref - (c/f_j)·N_j   N = SD phase bias (cycles)
```

#### 設計矩陣 H（Jacobian）

每一行 `H[nv, :]` 是一個 DD 觀測對所有 nx 個狀態的偏導：

```
H[nv, 0:3]        = -e_ref + e_j           ← ∂v/∂pos (LOS 方向差)
H[nv, II(ref)]     = +s_ref                 ← ∂v/∂I_ref (電離層)
H[nv, II(j)]       = -s_j                   ← ∂v/∂I_j
H[nv, IT(0)]       = mf_rover_ref - mf_rover_j  ← ∂v/∂ZWD_rover
H[nv, IT(1)]       = -(mf_base_ref - mf_base_j) ← ∂v/∂ZWD_base
H[nv, IB(ref,f)]   = +c/f_ref              ← ∂v/∂N_ref (phase bias)
H[nv, IB(j,f)]     = -c/f_j                ← ∂v/∂N_j
```

其餘元素皆為 0。H 是稀疏的：每行只有 ~5-7 個非零元素。

#### 量測協方差 R — ddcov()

DD 量測之間存在相關性（共用 reference satellite 的 SD 觀測）：

```
R_DD[i,j] = {  Ri[i] + Rj[i]     if i == j        (方差)
            {  Ri[i]              if same ref, i≠j  (協方差)
            {  0                  otherwise
```

其中 `Ri[i]` 是 reference sat 的 SD 方差，`Rj[i]` 是 satellite j 的 SD 方差。
SD 方差由 `varerr()` 計算，考慮仰角、SNR、基線長度、頻率等。

#### Outlier rejection (L890-893)

```
如果 |v[nv]| > maxinno × threshadj:
  vsat[f] = 0                  ← 標記為無效
  rejc[f]++                    ← 拒絕計數器 (≥2 時觸發 bias 重置)
  跳過此觀測                    ← 不進入 EKF
```

`threshadj = 10` 如果 bias 剛初始化（P = σ²_code），放寬閾值讓新衛星有機會進入。

### 6.2 filter() — EKF 觀測更新 (MatrixUtil.java L482)

#### 壓縮機制

```java
for (int i = 0; i < n; i++) {
    if (x[i] != 0.0 && P[i + i * n] > 0.0) ix[k++] = i;
}
```

只保留活躍狀態。典型 n=207 壓縮到 k≈70（3 pos + ~65 可見衛星 bias）。
壓縮後的矩陣運算量：O(70³) vs O(207³) = **26 倍加速**。

#### 核心運算 filter_()

標準 Kalman filter 觀測更新的 7 個步驟：

```
輸入: x(k×1), P(k×k), H(k×m), v(m×1), R(m×m)
      k = 壓縮後活躍狀態數, m = DD 觀測數

Step 1:  F  = P × H              (k×m)    F = 協方差投影到觀測空間
Step 2:  Q  = Hᵀ × F + R         (m×m)    Q = 新息協方差 (innovation cov.)
Step 3:  Q⁻¹                              LU 分解求逆
Step 4:  K  = F × Q⁻¹            (k×m)    K = Kalman 增益
Step 5:  x⁺ = x + K × v          (k×1)    狀態更新
Step 6:  I_ = I - K × Hᵀ         (k×k)    更新因子
Step 7:  P⁺ = I_ × P             (k×k)    協方差更新
```

**計算量分析**（k=70, m=20）：

| 步驟 | 運算 | FLOP 數 | 佔比 |
|------|------|---------|------|
| Step 1: P×H | k²m | 98K | 18% |
| Step 2: H'×F | m²k | 28K | 5% |
| Step 4: F×Q⁻¹ | km² | 28K | 5% |
| Step 6: K×H' | k²m | 98K | 18% |
| Step 7: I_×P | k³ | 343K | **64%** |
| **合計** | | ~595K | |

Step 7 佔 64% 的計算量——這就是 matmul cache 優化的主要目標。

#### Kalman 增益的直覺意義

```
K = P × H × (Hᵀ × P × H + R)⁻¹

K[i,j] 代表: 第 j 個觀測的新息 (v[j]) 對第 i 個狀態的修正量。

如果狀態 i 的方差大（P 大）且觀測 j 對狀態 i 敏感（H 大）:
  → K 大 → 觀測對狀態影響大
如果觀測雜訊大（R 大）:
  → K 小 → 觀測影響小，信任現有估計
```

### 6.3 為什麼要迭代？

觀測方程 `ρ = |rs - rr|` 是非線性的。`ddres()` 對位置做 Taylor 展開只取一階：

```
ρ(rr + δr) ≈ ρ(rr) + e · δr       e = LOS 單位向量
```

EKF 更新後位置改變，線性化點偏移。迭代讓線性化點逐步逼近真值。
Static 基線通常 2 次就收斂（位置變化量 < mm 級）。

---

## 七、Step 2f：Post-fit 殘差檢查

**程式碼**：L1511-1537

用更新後的位置 `xp` 重新計算 zdres + ddres，驗證殘差是否合理。

```
valpos(rtk, v, R, vflg, nv, 4.0)
```

目前 `valpos()` 始終回傳 true（與 C 行為一致），但殘差仍被計算並儲存在
`ssat[i].resp[f]` / `ssat[i].resc[f]` 中，供除錯使用。

---

## 八、Step 2g：整數模糊度解算 (AR)

### 8.1 manageAmbLAMBDA() — AR 管理 (L1272)

入口條件檢查：

```
跳過 AR 如果:
  - mode ≤ DGPS
  - modear == OFF
  - thresar[0] < 1.0 (ratio threshold 無效)
  - posvar > thresar[1] (位置方差太大)
```

#### Satellite dropout (round-robin) (L1287-1299)

如果上一 epoch 未能固定 (`prevRatio2 < thres`) 且衛星數足夠 (`nb_ar ≥ mindropsats`)：

```
找下一顆 AR-eligible 衛星排除 (round-robin)
lock[f] = -nb_ar          ← 從 AR 中移除足夠長的時間
```

Round-robin 策略：從上次排除位置開始掃描，逐 epoch 換不同衛星嘗試，
最終能定位到造成問題的衛星。

#### AR filter (L1313-1333)

如果新衛星加入後 ratio 惡化：

```
如果 prevRatio2 ≥ thres 且 (ratio < thres 或 ratio 大幅下降):
  對每顆 lock == 0 的衛星（剛通過冷卻期的新衛星）:
    lock = -minlock - dly   ← 延遲重新加入，stagger 避免同時加入
    dly += 2
  重新跑 LAMBDA
```

### 8.2 resambLAMBDA() — LAMBDA 解算 (L1182)

#### Step 1: 建立 DD 索引 ddidx() (L945)

```
對每個星座 m、每個頻率 f:
  找 reference satellite: 第一個 lock≥0 且無 half-cycle 的衛星
  對其他合格衛星建立 DD pair: ix[nb*2] = ref, ix[nb*2+1] = sat_j

合格條件: lock[f]≥0, 無 half-cycle slip, elevation ≥ elmaskar, vsat≠0
```

#### Step 2: 提取 DD 模糊度和協方差

```
y[i] = x[ix[i*2]] - x[ix[i*2+1]]         ← DD float ambiguity (SD_ref - SD_j)

DP[i,j] = P[ix[i*2], na+j] - P[ix[i*2+1], na+j]     ← D×P_amb
Qb[i,j] = DP[i, ix[j*2]-na] - DP[i, ix[j*2+1]-na]   ← D×P_amb×D' (DD 協方差)
Qab[i,j] = P[i, ix[j*2]] - P[i, ix[j*2+1]]           ← P_pos_amb × D'
```

#### Step 3: LAMBDA 搜索

```
Lambda.lambda(nb, 2, y, Qb, b, s)

1. LDL' 分解 Qb
2. Z-transform 降相關 → 使搜索空間更「球形」
3. ILS (Integer Least Squares) 搜索:
   找到使 (y - b)' × Qb⁻¹ × (y - b) 最小的整數向量 b
4. 回傳最佳 b[0:nb] 和次佳 b[nb:2nb]
5. 殘差: s[0] = 最佳, s[1] = 次佳
```

#### Step 4: Ratio test

```
ratio = s[1] / s[0]                         ← 次佳 / 最佳
thres = computeAdaptiveArThreshold(nb)       ← FFRT: 自適應閾值 (基於 nb)

如果 ratio ≥ thres → 固定成功
如果 ratio < thres → 保持 float
```

Ratio test 的直覺：如果最佳整數解遠優於次佳，ratio 大，表示整數解是唯一的。
如果兩者接近，ratio 小，表示整數解不確定。

自適應閾值 (FFRT) 隨 nb 減小而增大，因為衛星越少，偶然通過的機率越高。

#### Step 5: 條件協方差更新

固定成功後，用條件分佈更新「真實」狀態（位置等）：

```
// 殘差: float ambiguity - fixed integer
δy = y - b

// 條件均值: 扣除 ambiguity 已知後的位置修正
xa = x[0:na] - Qab × Qb⁻¹ × δy

// 條件協方差: ambiguity 已知後位置不確定度收縮
Pa = P[0:na, 0:na] - Qab × Qb⁻¹ × Qab'
```

這就是 RTK 高精度的數學本質：**phase bias 被固定為整數後，位置的不確定度從 ~dm 級收縮到 ~cm 級**。

#### Step 6: restamb() (L1003)

將 DD 固定的整數模糊度還原回 SD bias：

```
xa[ref_index] = x[ref_index]               ← reference sat 保持 float 值
xa[sat_index] = xa[ref_index] - b[i]       ← 其他衛星 = ref - DD_integer
```

### 8.3 固定解驗證 (L1543-1561)

用固定解 `xa` 重新算 zdres + ddres，檢查殘差是否合理。
通過驗證後更新 nfix 計數器。

### 8.4 holdamb() — Fix-and-Hold 回饋 (L1040)

**程式碼**：L1040-1067

連續 `minfix`（例如 20）次固定成功後，將固定的 ambiguity 作為虛擬觀測回饋到 float EKF：

```
對每個 DD pair (ref, sat_j):
  v[i] = (xa[ref] - xa[j]) - (x[ref] - x[j])     ← 固定值 vs float 值的差
  H[i, IB(ref)] = 1
  H[i, IB(j)]   = -1
  R = diag(varholdamb)                              ← 很小的方差（強約束）

filter(x, P, H, v, R, nx, nv)                       ← 注入 float EKF
```

效果：float bias 被強力拉向整數值。後續 epoch 的 float 解更接近整數，
ratio 更高，形成正向循環。

**注意**：`minfix=20` 的門檻是為了避免錯誤固定（wrong fix）污染 EKF。
一旦 wrong fix 被 hold 進去，EKF 需要很長時間才能脫離。

---

## 九、Step 2h：計數器更新

**程式碼**：L1585-1601

```java
// 儲存相位觀測（供下 epoch cycle slip 偵測用）
ssat[sat].pt[rcv][f] = obs.time;      // 觀測時間
ssat[sat].ph[rcv][f] = obs.L[f];      // 載波相位值

// slip 計數器
if (slip & LLI_SLIP) slipc[f]++;

// lock 計數器更新
if (vsat[f] == 0) continue;           // 未被使用的衛星不更新
if (lock[f] < 0) lock[f]++;           // 冷卻中: 遞增趨近 0
else if (nfix > 0 && fix[f] >= 2)     // fix 成功且參與 AR
    lock[f]++;                         // 累積信心
```

---

## 十、資料流圖

```
觀測資料 (L, P, D, SNR)
    │
    ▼
┌─────────────────────────────────────────┐
│  SPP: pseudorange least-squares          │
│  → 粗略位置 rr, 衛星 azel               │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  satposs: 衛星 ECEF + 鐘差               │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  zdres(base): 基站非差殘差               │
│  y_base = obs - model                    │──→ y_base[]
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  udstate: EKF 時間更新                    │
│  ├─ udpos:  static 不動                  │
│  ├─ udbias: slip偵測 → bias重置/初始化    │
│  └─ P += Q×dt (process noise)            │
└─────────────┬───────────────────────────┘
              ▼
      ┌────── 迭代 2~3 次 ──────┐
      │                          │
      │  zdres(rover)            │
      │  → y_rover = obs - model │
      │                          │
      │  ddres                   │
      │  → v = DD(y_rover, y_base) - h(x)
      │  → H = ∂v/∂x (Jacobian) │
      │  → R = DD 量測協方差      │
      │                          │
      │  filter (EKF update)     │
      │  → K = PH(H'PH+R)⁻¹    │
      │  → x += Kv, P = (I-KH')P│
      │                          │
      └────────────┬─────────────┘
                   ▼
┌─────────────────────────────────────────┐
│  Post-fit: 驗證更新後殘差                 │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  LAMBDA: float ambiguity → integer       │
│  ├─ DD 索引 (ddidx)                      │
│  ├─ LDL' + Z-transform + ILS 搜索        │
│  ├─ ratio test (自適應閾值)               │
│  └─ 條件協方差: xa, Pa                   │
└─────────────┬───────────────────────────┘
              ▼
┌─────────────────────────────────────────┐
│  holdamb: 固定解回饋 float EKF            │
│  (連續 minfix 次成功後啟動)               │
│  → bias 被拉向整數 → 正向循環             │
└─────────────┬───────────────────────────┘
              ▼
         輸出 solution
         stat = FIX (xa) 或 FLOAT (x)
```

---

## 十一、關鍵設計決策與直覺

### 為什麼用 SD bias 而不是 DD bias 作為 EKF 狀態？

DD bias 在 reference satellite 切換時需要重新參數化整個 ambiguity 子空間
（pivot 問題）。SD bias 避免了這個問題：ref sat 切換只影響 `ddidx()` 的索引，
EKF 狀態向量本身不需要任何變動。代價是 SD 參數化有一個 rank deficiency
（接收機端的整數不確定性），但這不影響 DD 層級的求解。

### 為什麼 filter 要壓縮？

`nx=207` 但典型只有 ~15 顆衛星可見，對應 ~65 個活躍 bias + 3 pos ≈ 70 個活躍狀態。
其餘 137 個狀態的 x[i]=0 且 P[i,i]=0，對 EKF 更新沒有任何貢獻。
壓縮讓核心矩陣運算從 O(207³) ≈ 8.9M 降到 O(70³) ≈ 343K，加速 **26 倍**。

### Fix-and-hold 的正向循環

```
首次固定成功
  → holdamb 把 float bias 拉向整數
    → 下一 epoch float 解更接近整數
      → ratio 更高 → 更容易固定
        → 更多 hold → bias 更準 → ...
```

這是 RTK 能長時間保持 fix 的核心機制。但也是雙面刃：
如果首次固定錯誤（wrong fix），hold 會把 EKF 拉向錯誤的整數，
需要等 bias 方差自然膨脹才能脫離。`minfix=20` 的門檻就是為了降低這個風險。

### 迭代的必要性

觀測方程 `ρ = |rs - rr|` 是非線性的。EKF 只做一階 Taylor 展開：

```
ρ(rr + δr) ≈ ρ(rr) + e · δr
```

如果線性化點（上一 epoch 的位置）與真值相差甚遠，一階近似不夠準確。
迭代讓線性化點逐步逼近真值。Static 基線通常 2 次收斂，
kinematic 可能需要 3 次。

### Outage reset vs variance inflation

衛星長時間消失後（`outc > maxout`），bias 的真實值可能已經完全改變
（接收機重啟、相位跳變等）。完全歸零（`initx(0,0,j)`）確保衛星
重新出現時從乾淨的 code-phase 估計重新開始，而不是沿用可能已錯誤的舊值。
這是 Java 版本達到 99.6% fix rate 的關鍵修正。
