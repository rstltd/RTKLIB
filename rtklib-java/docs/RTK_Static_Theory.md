# RTKLIB RTK Static Baseline Post-Processing: Theory and Implementation

## Table of Contents

1. [Overview](#1-overview)
2. [Observation Model](#2-observation-model)
3. [EKF State Vector](#3-ekf-state-vector)
4. [Time Update (Prediction)](#4-time-update-prediction)
5. [Measurement Update](#5-measurement-update)
6. [Measurement Noise Model](#6-measurement-noise-model)
7. [Ambiguity Resolution (LAMBDA)](#7-ambiguity-resolution-lambda)
8. [Fix-and-Hold Strategy](#8-fix-and-hold-strategy)
9. [Post-Processing Pipeline](#9-post-processing-pipeline)
10. [Quality Control](#10-quality-control)
11. [Code Reference Map](#11-code-reference-map)
12. [Parameter Reference](#12-parameter-reference)

---

## 1. Overview

RTKLIB 的 RTK static baseline 後處理使用 **Extended Kalman Filter (EKF)** 對雙差 (Double-Difference, DD)
觀測量進行遞推估計，核心流程為：

```
每個 epoch:
  1. Time Update   — 狀態預測（position hold, bias process noise）
  2. zdres()       — 計算零差殘差（幾何距離、衛星鐘差、對流層、天線）
  3. ddres()       — 形成雙差殘差、設計矩陣 H、量測協方差 R
  4. filter()      — EKF 量測更新（float solution）
  5. manageAmbLAMBDA() — 模糊度解算（LAMBDA + ratio test）
  6. holdamb()     — Fix-and-hold 回饋（若連續 fix >= minfix）
```

後處理支援三種模式：forward-only、backward-only、**combined**（forward + backward + smoother fusion）。

---

## 2. Observation Model

### 2.1 Undifferenced Phase and Code

對衛星 $s$，接收機 $r$ 在頻率 $f$ 上：

$$
\phi_r^s = \rho_r^s + c(\delta t_r - \delta t^s) + T_r^s - I_r^s / f^2 + \lambda_f N_r^s + \epsilon_\phi
$$

$$
P_r^s = \rho_r^s + c(\delta t_r - \delta t^s) + T_r^s + I_r^s / f^2 + \epsilon_P
$$

| 符號 | 意義 |
|------|------|
| $\rho_r^s$ | 幾何距離 (satellite position - receiver position) |
| $\delta t_r, \delta t^s$ | 接收機、衛星鐘差 |
| $T_r^s$ | 對流層延遲（乾延遲 + 濕延遲，經 mapping function 投影） |
| $I_r^s$ | 電離層延遲（與頻率平方成反比） |
| $N_r^s$ | 載波相位模糊度（整數 cycles） |
| $\lambda_f$ | 波長 = c/f |

### 2.2 Single-Difference (SD)

同一顆衛星 $s$，rover ($r$) 與 base ($b$) 之間做差，消除衛星鐘差：

$$
\Delta \phi^s = \phi_r^s - \phi_b^s = \Delta\rho^s + c\cdot\delta t_{rb} + \Delta T^s - \Delta I^s / f^2 + \lambda_f \Delta N^s
$$

### 2.3 Double-Difference (DD)

再對兩顆衛星 $s$ (reference) 與 $k$ 做差，消除接收機鐘差：

$$
\nabla\Delta\phi_f^{sk} = \Delta\phi_f^s - \Delta\phi_f^k = \nabla\Delta\rho^{sk} + \nabla\Delta T^{sk} - \nabla\Delta I^{sk}/f^2 + \lambda_f \nabla\Delta N^{sk}
$$

**DD 消除的誤差源**：
- 接收機鐘差（完全消除）
- 衛星鐘差（完全消除）
- 對流層延遲（短基線近似消除）
- 電離層延遲（短基線近似消除）
- 接收機硬體延遲（消除）

**殘留的誤差源**：
- 幾何距離差（與 rover 位置有關 → 待估參數）
- 相位模糊度差（整數 → LAMBDA 解算）
- 量測噪聲

### 2.4 Reference Satellite Selection

C: `rtkpos.c:1258-1267` / Java: `Rtkpos.java:738-746`

每個衛星系統（GPS/GLO/GAL/BDS/QZS/IRN）、每個頻率獨立選擇：

1. 按系統分組
2. 要求 rover 與 base 同時有效觀測
3. **優先選擇無 cycle slip** 的衛星
4. 在有效衛星中選 **仰角最高** 者（幾何最穩定、噪聲最低）

---

## 3. EKF State Vector

### 3.1 State Composition

C: `rtkpos.c:79-92` / Java: `RtkState.java:71-128`

```
x = [position | velocity | acceleration | ionosphere | troposphere | GLO_bias | phase_biases]
     ├── NP ──┤├── (opt) ──┤├── (opt) ────┤├── NI ────┤├── NT ────┤├── NL ──┤├── NB ────────┤
```

| 分量 | 維度 | 索引 | 條件 | 說明 |
|------|------|------|------|------|
| Position (x,y,z) | 3 | 0-2 | Always | ECEF 座標 (m) |
| Velocity | 3 | 3-5 | dynamics=1 | ECEF 速度 (m/s) |
| Acceleration | 3 | 6-8 | dynamics=1 | ECEF 加速度 (m/s^2) |
| Ionosphere | MAXSAT | II(s) | ionoopt=EST | 每顆衛星電離層延遲 (m) |
| Troposphere | 2 or 6 | IT(r) | tropopt>=EST | 天頂濕延遲 (m)，r=0:rover,1:base |
| GLO H/W Bias | NFREQ | IL(f) | glomodear=AUTOCAL | GLONASS 接收機偏差 |
| Phase Biases | MAXSAT*NF | IB(s,f) | mode>DGPS | 每顆衛星、每頻率 SD 模糊度 (cycles) |

**典型 static baseline 配置**（nf=2, no dynamics, no iono/trop estimation）：

```
NP=3, NI=0, NT=0, NL=0, NB=MAXSAT*2
NR=3 (real parameters), NX=3+MAXSAT*2 (total)
```

### 3.2 State Dimensions

```
NF(opt) = (ionoopt == IFLC) ? 1 : nf
NP(opt) = (dynamics == 0) ? 3 : 9
NI(opt) = (ionoopt != EST) ? 0 : MAXSAT
NT(opt) = (tropopt < EST) ? 0 : (tropopt < ESTG) ? 2 : 6
NL(opt) = (glomodear != AUTOCAL) ? 0 : NFREQGLO
NB(opt) = (mode <= DGPS) ? 0 : MAXSAT * NF
NR(opt) = NP + NI + NT + NL          (real parameters, for fixed solution)
NX(opt) = NR + NB                    (total, for float solution)
```

---

## 4. Time Update (Prediction)

C: `rtkpos.c:udstate()` / Java: `Rtkpos.java:udstate()`

### 4.1 Position (`udpos`)

C: line 484 / Java: line 348

| 模式 | 行為 |
|------|------|
| **STATIC** | 第一個 epoch 用 SPP 初始化，之後 **不更新**（位置不隨時間變） |
| FIXED | 設為已知座標，極小方差 (1e-8 m^2) |
| KINEMATIC (no dynamics) | 每 epoch 重新初始化為 SPP 解 |
| KINEMATIC (dynamics) | 狀態轉移 $x_k = F \cdot x_{k-1}$，CV/CA 模型 |

Static baseline 的關鍵：**position 的 time update 是 identity**（P 不增長），位置精度完全靠量測更新逐步收斂。

### 4.2 Ionosphere (`udion`)

C: line 572 / Java: line 531

- 初始值：$1 \times 10^{-6}$ m
- 初始方差：$(\text{std}[1] \times bl/10^4)^2$（與基線長度成正比）
- Process noise：$(\text{prn}[1] \times bl/10^4 \times \cos(el))^2 \times |dt|$
- 衛星斷觀測超過 `GAP_RESION` (120s) 後 reset

### 4.3 Troposphere (`udtrop`)

C: line 602 / Java: line 555

- 初始值：`INIT_ZWD` = 0.15 m（天頂濕延遲）
- 初始方差：$\text{std}[2]^2$
- Process noise：$\text{prn}[2]^2 \times |dt|$（random walk）
- 梯度 process noise：$(\text{prn}[2] \times 0.3)^2 \times |dt|$

### 4.4 Phase Bias (`udbias`)

C: line 806 / Java: line 383

這是最複雜的 time update：

**Cycle Slip Detection**（多重檢測）：
1. **Doppler method** (`detslp_dop`)：比較相位變化率與 Doppler
2. **LLI flag** (`detslp_ll`)：利用 RINEX LLI 標記
3. **Geometry-free** (`detslp_gf`)：L1-L2 相位差跳變
4. **Code-phase divergence** (`detslp_code`)：碼相位差變化異常

**Bias 更新流程**：

```
For each frequency f:
  1. 超過 maxout epoch 未觀測 → reset bias to 0
  2. 現有 bias 加 process noise: P[j,j] += prn[0]^2 * |dt|
  3. Cycle slip detected → reset bias, set lock = -minlock
  4. 計算初始 bias: bias = SD_phase - SD_code * freq/c
  5. 校正 bias offset（確保新舊 bias 一致）
  6. 新衛星初始化: initx(bias, std[0]^2)
```

**初始 bias 估計**：

$$
\hat{N}_{SD} = \phi_{SD} - P_{SD} \cdot f / c
$$

取 SD phase (cycles) 減去 SD code 轉換成 cycles，得到粗略的模糊度估計。

---

## 5. Measurement Update

### 5.1 Zero-Difference Residuals (`zdres`)

C: `rtkpos.c:1023-1087` / Java: `Rtkpos.java:640-684`

對每顆衛星計算：

$$
y_\phi = L \cdot c/f - (\rho - c \cdot dt^s + T_{hyd} \cdot m_h + d_{ant})
$$

$$
y_P = P - (\rho - c \cdot dt^s + T_{hyd} \cdot m_h + d_{ant})
$$

其中：
- $\rho$：`geodist()` 幾何距離
- $dt^s$：衛星鐘差
- $T_{hyd} \cdot m_h$：乾對流層（Saastamoinen 模型 + 投影函數）
- $d_{ant}$：天線相位中心改正

### 5.2 Double-Difference Formation (`ddres`)

C: `rtkpos.c:1214-1450` / Java: `Rtkpos.java:690-894`

**DD 殘差**：

$$
v = (y_{rover}^{ref} - y_{base}^{ref}) - (y_{rover}^{k} - y_{base}^{k}) - H \cdot x
$$

**設計矩陣 H**：

位置偏導數（$e$ = 衛星到接收機的 LOS 單位向量）：
$$
\frac{\partial v}{\partial pos} = -(e^{ref} - e^k)
$$

電離層（若估計）：
$$
\frac{\partial v}{\partial I^s} = \pm (f_1/f_s)^2 \quad (\text{phase: +, code: -})
$$

對流層（若估計）：
$$
\frac{\partial v}{\partial T} = m^{ref} - m^k \quad (m = \text{mapping function})
$$

相位模糊度：
$$
\frac{\partial v}{\partial N^{ref}} = c/f^{ref}, \quad \frac{\partial v}{\partial N^k} = -c/f^k
$$

### 5.3 Kalman Filter Update

C: `rtkpos.c:relpos()` / Java: `Rtkpos.java:rtkpos()`

標準 EKF 量測更新：

$$
K = P^- H^T (H P^- H^T + R)^{-1}
$$
$$
x^+ = x^- + K \cdot v
$$
$$
P^+ = (I - KH) P^-
$$

RTKLIB 使用 `filter()` 函數（sequential/batch Kalman filter update）。

---

## 6. Measurement Noise Model

C: `rtkpos.c:varerr()` line 402 / Java: `Rtkpos.java:varerr()` line 172

### 6.1 Variance Formula

$$
\sigma^2 = 2 \left( a^2 + \frac{b^2}{\sin^2(el)} + c^2 \right) + d^2
$$

| 項 | 公式 | 意義 |
|----|------|------|
| $a$ | $F \cdot \text{err}[1]$ | 基本誤差（與仰角無關） |
| $b$ | $F \cdot \text{err}[2]$ | 仰角相關誤差：低仰角噪聲大 |
| $c$ | $\text{err}[3] \times bl / 10^4$ | 基線長度相關：大氣殘差 |
| $d$ | $c \cdot \text{sclkstab} \times dt$ | 接收機鐘穩定度 |
| 前面的 2 | — | SD → DD 方差倍增（兩次差分） |

### 6.2 Scale Factor F

$$
F = \text{eratio}[f] \times \text{EFACT}_{sys}
$$

**Code/Phase ratio** (`eratio`)：
- Phase（載波相位）：基本噪聲 ~mm 級
- Code（偽距）：噪聲 = phase × eratio，通常 100~300 倍

**系統 factor** (`EFACT`)：
| 系統 | EFACT | 說明 |
|------|-------|------|
| GPS | 1.0 | 基準 |
| GLONASS | 1.5 | FDMA 額外噪聲 |
| Galileo | 1.0 | — |
| BeiDou | 1.0 | — |
| SBAS | 3.0 | GEO 衛星品質較差 |
| QZSS | 1.0 | — |
| IRNSS | 1.5 | — |

### 6.3 Elevation Weighting

$$
\sigma^2_{el} \propto \frac{1}{\sin^2(el)}
$$

- 天頂 (90deg)：$\sin(el)=1$，噪聲最小
- 15deg 仰角：$\sin(15°)=0.26$，噪聲 ~15 倍
- 5deg 仰角：$\sin(5°)=0.087$，噪聲 ~130 倍

### 6.4 DD Covariance Matrix R (`ddcov`)

C: `rtkpos.c:1102` / Java: `Rtkpos.java:215`

R 矩陣是 **按系統分塊的對角矩陣**：

$$
R_{ij} = \begin{cases}
\sigma^2_{ref} + \sigma^2_{j} & i = j \text{ (diagonal)} \\
\sigma^2_{ref} & i \neq j, \text{ same system (off-diagonal)} \\
0 & \text{different system}
\end{cases}
$$

同系統內的 DD 共享 reference satellite 的方差（off-diagonal 非零），不同系統之間獨立。

---

## 7. Ambiguity Resolution (LAMBDA)

### 7.1 Integer Least Squares Formulation

給定 float ambiguity $\hat{a}$ 和其協方差 $Q_{\hat{a}}$：

$$
\check{a} = \arg\min_{a \in \mathbb{Z}^n} (\hat{a} - a)^T Q_{\hat{a}}^{-1} (\hat{a} - a)
$$

### 7.2 LAMBDA Algorithm

C: `lambda.c` / Java: `Lambda.java`

**Step 1: LD Factorization**

$$
Q_{\hat{a}} = L^T D L
$$

**Step 2: Z-Transform (Decorrelation)**

整數 Gauss 變換 + 排列，降低模糊度之間的相關性：

$$
z = Z^T \hat{a}, \quad Q_z = Z^T Q_{\hat{a}} Z
$$

**Step 3: MLAMBDA Search**

在 Z-transform 後的空間中搜索兩個最佳整數候選解 $\check{z}_1, \check{z}_2$。

**Step 4: Inverse Transform**

$$
\check{a} = Z^{-T} \check{z}
$$

**Step 5: Ratio Test**

$$
\text{ratio} = \frac{s_2}{s_1} = \frac{\|\hat{a} - \check{a}_2\|^2_{Q^{-1}}}{\|\hat{a} - \check{a}_1\|^2_{Q^{-1}}}
$$

ratio > threshold → **FIX**（最佳解顯著優於次佳解）。

### 7.3 FFRT Adaptive Threshold

C: `rtkpos.c:ar_poly_coeffs` / Java: `Rtkpos.computeAdaptiveArThreshold()`

Fixed Failure-Rate Test (Verhagen & Teunissen 2009)：DD 數量越多，所需 threshold 越低（等失敗率）。

| nb (DD count) | threshold (nom=3.0) |
|---------------|-------------------|
| 8 | ~3.0 (anchor) |
| 18 | ~1.98 |
| 38 | ~1.30 |

### 7.4 Conditional Covariance Update

AR 成功後，用固定模糊度更新 position：

$$
\hat{x}_a = \hat{x} - Q_{ab} Q_b^{-1} (\hat{b} - \check{b})
$$

$$
P_a = P - Q_{ab} Q_b^{-1} Q_{ab}^T
$$

這是 fix solution 精度遠高於 float 的數學原因：**模糊度不確定性被完全移除**。

### 7.5 Full AR Pipeline (`manageAmbLAMBDA`)

C: `rtkpos.c:1827-1943` / Java: `Rtkpos.java:1133-1260`

```
1. Position variance check → skip AR if P too large
2. Satellite dropout → 逐一排除可疑衛星（若上次 AR 失敗）
3. Initial AR → resamb_LAMBDA(gps=1, glo=depends, sbs=depends)
4. AR filter → 若 ratio 突然下降，排除新加入的衛星（lock=0）
5. GLO two-pass → 若仍失敗且用 FIXHOLD，重試不含 GLONASS
6. Dropout restore → 若排除衛星沒有改善，恢復
7. Update ratio history → prevRatio1, prevRatio2
```

---

## 8. Fix-and-Hold Strategy

C: `rtkpos.c:holdamb()` line 1604 / Java: `Rtkpos.java:holdamb()` line 978

### 8.1 Mechanism

當連續 `minfix` 個 epoch 達到 FIX：

1. 計算 DD pseudo-innovation：

$$
v_i = (\hat{x}_{a,ref} - \hat{x}_{a,i}) - (\hat{x}_{ref} - \hat{x}_i)
$$

2. 以小方差 (`varholdamb`) 作為 pseudo-measurement 回饋 EKF：

$$
H = \begin{bmatrix} 1 & 0 & \cdots & -1 & \cdots \end{bmatrix}, \quad R = \text{varholdamb} \cdot I
$$

3. Kalman filter update：**將 float ambiguity 拉向整數值**

### 8.2 GLONASS ICB 更新

Fix-and-hold 模式下，將 phase bias 的小數部分移到 inter-channel bias：

$$
\Delta = \text{gainholdamb} \times (\hat{N}_{DD} - \text{round}(\hat{N}_{DD}))
$$

$$
x[IB(j)] \mathrel{-}= \Delta, \quad \text{icbias}[j] \mathrel{+}= \Delta
$$

這讓 GLONASS 在後續 epoch 中可以用 ICB 補償 FDMA 偏差後參與 AR。

### 8.3 Effect on Static Baseline

Fix-and-hold 對 static baseline 特別有效：
- 初始 epoch float → ambiguity 逐步收斂
- 一旦 FIX → hold 鎖住模糊度
- 後續 epoch 幾乎 100% FIX
- 新衛星加入時，已 hold 的衛星提供穩定的 reference frame

---

## 9. Post-Processing Pipeline

C: `postpos.c` / Java: `PostProcessor.java`

### 9.1 Forward / Backward / Combined

| soltype | 行為 |
|---------|------|
| 0 (forward) | epoch 1→N，單次 EKF |
| 1 (backward) | epoch N→1，單次 EKF |
| 2 (combined) | forward + reset + backward + smoother fusion |

### 9.2 Combined Solution (`combres`)

C: `postpos.c:553` / Java: `PostProcessor.java:607`

對每個 epoch，比較 forward 解 $x_f, P_f$ 與 backward 解 $x_b, P_b$：

**Case 1**: 時間不匹配 → 用存在的那個

**Case 2**: 品質不同 → 取品質較好者（FIX > FLOAT > DGPS > SINGLE）

**Case 3**: 品質相同 → **Fixed-interval smoother fusion**：

$$
x_{fused} = x_f + P_f (P_f + P_b)^{-1} (x_b - x_f)
$$

$$
P_{fused} = P_f - P_f (P_f + P_b)^{-1} P_f
$$

等價於：$P_{fused}^{-1} = P_f^{-1} + P_b^{-1}$（information fusion）。

### 9.3 Validation (`valcomb`)

C: `postpos.c:527` / Java: `PostProcessor.java:591`

Kinematic FIX 解的一致性檢驗（**4-sigma test**）：

$$
\forall i: \quad (x_{f,i} - x_{b,i})^2 \leq 16 \cdot (\sigma^2_{f,i} + \sigma^2_{b,i})
$$

若不通過 → 降級為 FLOAT。

---

## 10. Quality Control

### 10.1 Outlier Detection in `ddres`

C: `rtkpos.c:1366-1382` / Java: `Rtkpos.java:829-842`

$$
|v_{DD}| > \text{maxinno} \times \text{threshadj} \quad \Rightarrow \quad \text{reject}
$$

- `maxinno[0]` (phase) 預設 ~0.03 m
- `maxinno[1]` (code) 預設 ~0.3 m
- `threshadj = 10` 若 bias 剛初始化（方差 = std[0]^2），放寬門檻

### 10.2 Solution Status

```
SOLQ_FIX    (1): AR 成功，模糊度固定
SOLQ_FLOAT  (2): float solution，模糊度未固定
SOLQ_DGPS   (4): DGPS 模式（無模糊度）
SOLQ_SINGLE (5): SPP（無基站）
```

### 10.3 Cycle Slip Detection

四重檢測機制：

| 方法 | 原理 | 靈敏度 |
|------|------|--------|
| LLI flag | RINEX 標記 | 依賴接收機 |
| Doppler | $\Delta\phi$ vs Doppler 預測 | ~0.5 cycle |
| Geometry-free (L1-L2) | 消除幾何，看電離層跳變 | ~0.05 cycle |
| Code-phase divergence | code-phase 差變化 | ~數 cycle |

---

## 11. Code Reference Map

### C Source (`src/rtkpos.c`)

| Function | Lines | Purpose |
|----------|-------|---------|
| `udstate()` | 931-960 | Master time update dispatcher |
| `udpos()` | 484-570 | Position state update |
| `udion()` | 572-600 | Ionosphere state update |
| `udtrop()` | 602-629 | Troposphere state update |
| `udbias()` | 806-929 | Phase bias update + cycle slip |
| `zdres()` | 1023-1087 | Zero-difference residuals |
| `ddres()` | 1214-1450 | Double-difference residuals + H, R |
| `varerr()` | 402-448 | Measurement noise model |
| `ddcov()` | 1102-1117 | DD covariance matrix |
| `ddidx()` | 1506-1574 | DD index selection |
| `resamb_LAMBDA()` | 1701-1806 | LAMBDA + conditional covariance |
| `manage_amb_LAMBDA()` | 1827-1943 | Full AR pipeline |
| `holdamb()` | 1604-1699 | Fix-and-hold feedback |
| `restamb()` | 1576-1602 | Restore SD from DD fixed |
| `relpos()` | 1975-2232 | Main RTK engine |

### Java Port (`positioning/Rtkpos.java`)

| Method | Lines | Purpose |
|--------|-------|---------|
| `udstate()` | 505-525 | Master time update |
| `udpos()` | 348-377 | Position state update |
| `udion()` | 531-549 | Ionosphere state update |
| `udtrop()` | 555-573 | Troposphere state update |
| `udbias()` | 383-501 | Phase bias update + cycle slip |
| `zdres()` | 640-684 | Zero-difference residuals |
| `ddres()` | 690-894 | DD residuals + H, R |
| `varerr()` | 172-201 | Measurement noise model |
| `ddcov()` | 215-226 | DD covariance matrix |
| `ddidx()` | 893-945 | DD index selection |
| `resambLAMBDA()` | 1067-1152 | LAMBDA + conditional covariance |
| `manageAmbLAMBDA()` | 1133-1260 | Full AR pipeline |
| `holdamb()` | 978-1036 | Fix-and-hold feedback |
| `rtkpos()` | — | Main entry (calls relpos) |

### LAMBDA (`lambda.c` / `Lambda.java`)

| Function | C Lines | Java Lines | Purpose |
|----------|---------|------------|---------|
| `LD()` | 28-44 | 33-50 | LD factorization |
| `reduction()` | 74-89 | 95-110 | Z-transform decorrelation |
| `search()` | 97-167 | 116-187 | MLAMBDA search |
| `lambda()` | 180-206 | 204-225 | Public API |

---

## 12. Parameter Reference

### Processing Options (`prcopt_t` / `ProcessingOptions`)

| Parameter | Field | Default | Description |
|-----------|-------|---------|-------------|
| Mode | `mode` | STATIC | STATIC / KINEMA / FIXED |
| Frequencies | `nf` | 2 | 1-3 (L1, L1+L2, L1+L2+L5) |
| Nav systems | `navsys` | GPS | GPS\|GLO\|GAL\|BDS\|QZS\|IRN |
| Elevation mask | `elmin` | 15 deg | Minimum satellite elevation |
| Ionosphere | `ionoopt` | BRDC | OFF / BRDC / IFLC / EST |
| Troposphere | `tropopt` | SAAS | OFF / SAAS / EST / ESTG |
| AR mode | `modear` | 3 | 0:off, 1:continuous, 2:instantaneous, 3:fix-and-hold |
| GLO AR mode | `glomodear` | 1 | 0:off, 1:fix-and-hold, 2:autocal |
| Dynamics | `dynamics` | 0 | 0:off, 1:on (velocity/acceleration estimation) |

### Error Model Parameters

| Parameter | Field | Default | Unit | Meaning |
|-----------|-------|---------|------|---------|
| Base error (phase) | `err[1]` | 0.003 | m | Constant term |
| Elev error | `err[2]` | 0.003 | m | b/sin(el) term |
| Baseline error | `err[3]` | 0.0 | m/10km | Baseline-dependent |
| Code/phase ratio | `eratio[f]` | 100-300 | - | Code noise / phase noise |
| Clock stability | `sclkstab` | 5e-12 | s/s | Receiver clock Allan deviation |

### State Initial / Process Noise

| Parameter | Field | Default | Unit | Meaning |
|-----------|-------|---------|------|---------|
| Bias initial std | `std[0]` | 30.0 | m | Phase bias initial sigma |
| Iono initial std | `std[1]` | 0.03 | m/10km | Ionosphere initial sigma |
| Trop initial std | `std[2]` | 0.3 | m | ZWD initial sigma |
| Bias process noise | `prn[0]` | 1e-4 | m/sqrt(s) | Phase bias random walk |
| Iono process noise | `prn[1]` | 1e-3 | m/10km/sqrt(s) | Iono random walk |
| Trop process noise | `prn[2]` | 1e-4 | m/sqrt(s) | ZWD random walk |
| Accel H process noise | `prn[3]` | 1.0 | m/s^2/sqrt(s) | Horizontal accel |
| Accel V process noise | `prn[4]` | 0.1 | m/s^2/sqrt(s) | Vertical accel |

### AR Parameters

| Parameter | Field | Default | Meaning |
|-----------|-------|---------|---------|
| Ratio threshold | `thresar[0]` | 3.0 | LAMBDA ratio test threshold |
| Max pos variance | `thresar[1]` | 900 | Skip AR if position variance > this (m^2) |
| Min fix count | `minfix` | 10 | Consecutive fixes to trigger hold |
| Min lock count | `minlock` | 0 | Min epochs locked to use in AR |
| Min fix sats | `minfixsats` | 4 | Min DD pairs for AR attempt |
| Min hold sats | `minholdsats` | 3 | Min sats to hold |
| Min drop sats | `mindropsats` | 5 | Min ARs before dropout attempt |
| AR elevation mask | `elmaskar` | 15 deg | Elevation cutoff for AR |
| Hold elevation mask | `elmaskhold` | 15 deg | Elevation cutoff for hold |
| Hold variance | `varholdamb` | 0.01 | Pseudo-measurement variance (m^2) |
| Hold gain | `gainholdamb` | 0.01 | ICB extraction gain |
| AR filter | `arfilter` | 1 | Enable satellite AR rejection filter |
| Max outlier count | `maxout` | 5 | Reset bias after this many outages |
