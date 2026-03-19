# RTKLIB PPP Post-Processing: Theory and Implementation

## Table of Contents

1. [Overview](#1-overview)
2. [Observation Model](#2-observation-model)
3. [Error Sources and Corrections](#3-error-sources-and-corrections)
4. [EKF State Vector](#4-ekf-state-vector)
5. [Time Update (Prediction)](#5-time-update-prediction)
6. [Measurement Update](#6-measurement-update)
7. [Measurement Noise Model](#7-measurement-noise-model)
8. [Cycle Slip Detection](#8-cycle-slip-detection)
9. [PPP-AR (Ambiguity Resolution)](#9-ppp-ar-ambiguity-resolution)
10. [PPP vs RTK Comparison](#10-ppp-vs-rtk-comparison)
11. [Code Reference Map](#11-code-reference-map)
12. [Parameter Reference](#12-parameter-reference)

---

## 1. Overview

PPP (Precise Point Positioning) 使用 **單一接收機** 搭配精密星曆（SP3）和精密鐘差（CLK），
達到 cm 級定位精度。與 RTK 不同，PPP 不需要基站，但需要精確建模所有誤差源。

核心流程：

```
每個 epoch:
  1. 精密星曆/鐘差插值 → 衛星位置和鐘差
  2. Time Update   — 狀態預測（position, clock white noise, trop, bias）
  3. 改正量計算   — 潮汐、天線 PCV、相位纏繞、對流層、電離層
  4. ppp_res()    — 殘差、設計矩陣 H、量測協方差 R
  5. filter()     — EKF 量測更新
  6. Post-fit 殘差檢驗 → 剔除最大 outlier → 迭代（最多 8 次）
  7. PPP-AR       — 模糊度解算（stub，未實作）
```

**與 RTK 的根本差異**：PPP 不做差分，所有誤差源（衛星鐘差、天線、相位纏繞、相對論等）必須顯式建模。

---

## 2. Observation Model

### 2.1 Undifferenced Phase and Code

對衛星 $s$，接收機 $r$ 在頻率 $f$ 上：

$$
\phi_r^s = \rho_r^s + c(\delta t_r - \delta t^s) + T_r^s - I_r^s/f^2 + \lambda_f N_r^s + d_{\phi,ant} + d_{phw} + \epsilon_\phi
$$

$$
P_r^s = \rho_r^s + c(\delta t_r - \delta t^s) + T_r^s + I_r^s/f^2 + d_{P,ant} + d_{cbias} + \epsilon_P
$$

| 符號 | 意義 | PPP 處理方式 |
|------|------|-------------|
| $\rho_r^s$ | 幾何距離 | SP3 精密星曆 |
| $\delta t^s$ | 衛星鐘差 | CLK 精密鐘差 |
| $\delta t_r$ | 接收機鐘差 | **估計**（每系統獨立，white noise） |
| $T_r^s$ | 對流層延遲 | **估計**（ZTD + 梯度） |
| $I_r^s$ | 電離層延遲 | IFLC 消除 或 **估計** |
| $N_r^s$ | 相位模糊度 | **估計**（float bias in meters） |
| $d_{ant}$ | 天線改正 | ANTEX PCV 模型 |
| $d_{phw}$ | 相位纏繞 | 衛星姿態模型 |
| $d_{cbias}$ | 碼偏差 | SSR 或檔案改正 |

### 2.2 Ionosphere-Free Linear Combination (IFLC)

C: `ppp.c:449-457` / Java: `Pppos.java:638-647`

$$
C_1 = \frac{f_1^2}{f_1^2 - f_2^2}, \quad C_2 = \frac{-f_2^2}{f_1^2 - f_2^2}
$$

$$
L_c = C_1 L_1 + C_2 L_2, \quad P_c = C_1 P_1 + C_2 P_2
$$

GPS L1/L2 ($f_1$=1575.42 MHz, $f_2$=1227.60 MHz)：$C_1=2.546$, $C_2=-1.546$

**效果**：完全消除一階電離層延遲，但量測噪聲放大 $\sim$3 倍（方差 $\times 9$）。

### 2.3 PPP 殘差方程

C: `ppp.c:1046` / Java: `Pppos.java:834`

**相位**：
$$
v_\phi = L_c - (\rho + c \cdot \delta t_r - c \cdot \delta t^s + d_{trop} + B_\phi)
$$

**偽距**：
$$
v_P = P_c - (\rho + c \cdot \delta t_r - c \cdot \delta t^s + d_{trop} + d_{cbias})
$$

其中 $L_c, P_c$ 已經過天線 PCV、相位纏繞、碼偏差改正。

---

## 3. Error Sources and Corrections

### 3.1 Corrections Pipeline

C: `ppp.c:936-1095` (ppp_res) / Java: `Pppos.java:660-885`

```
Raw observation
  │
  ├── 1. Geometric range (geodist)          ← SP3 精密星曆
  ├── 2. Satellite clock (-c·dts)           ← CLK 精密鐘差
  ├── 3. Solid Earth tides (tidedisp)       ← IERS 模型
  ├── 4. Satellite antenna PCV (satantpcv)  ← ANTEX
  ├── 5. Receiver antenna PCV (antmodel)    ← ANTEX
  ├── 6. Phase wind-up (model_phw)          ← 衛星姿態模型
  ├── 7. Troposphere (model_trop)           ← Saastamoinen + mapping function
  ├── 8. Code bias (corr_meas)             ← SSR 或檔案
  └── 9. Ionosphere (model_iono)            ← IFLC 消除 或 estimated
```

### 3.2 Satellite Antenna Phase Center

C: `ppp.c:837-854` / Java: `Pppos.java:289-331`

衛星天線偏移在 **衛星固定座標系** (satellite body frame) 中定義，需要轉換到 ECEF：

1. 計算太陽位置 → 建立衛星 body frame ($e_x, e_y, e_z$)
2. 讀取 ANTEX 中的 L1、L2 偏移
3. IFLC 組合：$d_{ant} = C_1 \cdot d_1 + C_2 \cdot d_2$
4. 投影到 LOS 方向

### 3.3 Receiver Antenna Phase Center

C: `rtkcmn.c:antmodel()` / Java: `Pppos.java:341-357`

$$
d_{ant,f} = -(off_f \cdot e_{AEZ}) + PCV_f(90° - el)
$$

其中 $e_{AEZ}$ = [sin(az)cos(el), cos(az)cos(el), sin(el)]，$off_f$ = 天線偏移 (NEU)。

### 3.4 Phase Wind-up

C: `ppp.c:290-328` / Java: `Pppos.java:237-278`

載波相位受衛星與接收機天線相對旋轉影響，最大可達 0.5 cycle：

1. 計算衛星 yaw 姿態 → body frame $e_{xs}, e_{ys}$
2. 計算接收機 antenna frame $e_{xr}, e_{yr}$
3. 有效偶極子向量：

$$
d_s = e_{xs} - \hat{k}(\hat{k} \cdot e_{xs}) - \hat{k} \times e_{ys}
$$

$$
d_r = e_{xr} - \hat{k}(\hat{k} \cdot e_{xr}) + \hat{k} \times e_{yr}
$$

4. 纏繞角（cycles）：

$$
\phi_{phw} = \frac{1}{2\pi} \arccos\left(\frac{d_s \cdot d_r}{|d_s||d_r|}\right) \times \text{sign}(\hat{k} \cdot (d_s \times d_r))
$$

5. 保持連續性：$\phi_{phw} = \phi + \lfloor \phi_{prev} - \phi + 0.5 \rfloor$

### 3.5 Troposphere Model

C: `ppp.c:856-906` / Java: `Pppos.java:894-920`

**Precise model**（TROPOPT_EST/ESTG）：

$$
d_{trop} = m_h \cdot ZHD + m_w \cdot (ZTD - ZHD)
$$

其中 $m_h, m_w$ 為乾、濕 mapping function（如 GMF/VMF），$ZHD$ 由 Saastamoinen 模型計算，
$ZTD$ 為待估參數。

**梯度模型**（TROPOPT_ESTG）：

$$
d_{trop} = m_h \cdot ZHD + m_w \cdot ZWD + m_w \cot(el) \cdot (G_N \cos(az) + G_E \sin(az))
$$

偏導數：
- $\partial d / \partial ZTD = m_w$
- $\partial d / \partial G_N = m_w \cot(el) \cos(az)$
- $\partial d / \partial G_E = m_w \cot(el) \sin(az)$

### 3.6 Solid Earth Tides

C: `rtkcmn.c:tidedisp()` / Java: `TideCorrection.tidedisp()`

站座標的潮汐改正（ECEF 位移）：
- **固體潮**：日月引力造成，徑向最大 ~30 cm
- **海洋潮汐負荷** (OTL)：站點附近海洋質量重分佈
- **極潮**：地球極軸擺動

### 3.7 Code Bias

C: `ppp.c:428-447` / Java: `Pppos.java:627-635`

偽距受接收機和衛星硬體延遲影響，需要 code bias 改正：
- SSR 模式：使用即時 SSR 碼偏差
- 檔案模式：使用 `nav.cbias[]` 查表

---

## 4. EKF State Vector

### 4.1 State Composition

C: `ppp.c:104-118` / Java: `PppState.java:49-116`

```
x = [position | clock(×NSYS) | troposphere | ionosphere | DCB | phase_biases]
     ├── NP ──┤├── NC=7 ────┤├── NT(0/1/3)┤├── NI ─────┤├ND─┤├── NB ────────┤
```

| 分量 | 維度 | 索引 | 說明 |
|------|------|------|------|
| Position (x,y,z) | 3 | 0-2 | ECEF (m) |
| Velocity | 3 | 3-5 | dynamics=1 時 |
| Acceleration | 3 | 6-8 | dynamics=1 時 |
| **Clock (per system)** | **NSYS=7** | IC(s) | GPS/GLO/GAL/BDS/QZS/IRN/LEO |
| Troposphere | 1 or 3 | IT() | ZTD [+ G_N + G_E] |
| Ionosphere | MAXSAT | II(s) | 每顆衛星垂直電離層 (ionoopt=EST) |
| L5 DCB | 0 or 1 | ID() | L5 接收機差分碼偏差 (nf>=3) |
| Phase Biases | NF×MAXSAT | IB(s,f) | 每衛星每頻率 (m, not cycles) |

**PPP vs RTK State Vector 關鍵差異**：

| | PPP | RTK |
|--|-----|-----|
| Clock | **7 states**（每系統） | **無**（DD 消除） |
| Ionosphere | MAXSAT or IFLC 消除 | 0 or MAXSAT |
| GLO HW Bias | 不估計 | NL (autocal) |
| Phase bias 單位 | **meters** | **meters** (SD) |
| Typical NX | 3+7+1+MAXSAT = ~260 | 3+MAXSAT*2 = ~473 |

### 4.2 Dimension Formulas

```
NF(opt) = (ionoopt == IFLC) ? 1 : nf
NP(opt) = dynamics ? 9 : 3
NC()    = NSYS = 7
NT(opt) = tropopt < EST ? 0 : (tropopt == EST ? 1 : 3)
NI(opt) = (ionoopt == EST) ? MAXSAT : 0
ND(opt) = (nf >= 3) ? 1 : 0
NR(opt) = NP + NC + NT + NI + ND        (real parameters)
NB(opt) = NF × MAXSAT                   (phase biases)
NX(opt) = NR + NB                       (total)
```

---

## 5. Time Update (Prediction)

C: `ppp.c:udstate_ppp()` line 811 / Java: `Pppos.java:udstatePpp()` line 372

### 5.1 Position (`udpos_ppp`)

C: line 525 / Java: line 382

| 模式 | 行為 |
|------|------|
| **PPP_STATIC** | 第一 epoch 用 SPP 初始化 (var=900 m²)，之後加微小 process noise：$P_{ii} += \text{prn}[5]^2 |dt|$ |
| PPP_KINEMA | 每 epoch 重設為 SPP 解 |
| PPP_FIXED | 設為已知座標，$\sigma^2 = 10^{-8}$ m² |

### 5.2 Clock (`udclk_ppp`) — White Noise

C: line 617 / Java: line 410

$$
x[IC(s)] = c \cdot \delta t_r, \quad P[IC(s), IC(s)] = \text{VAR\_CLK} = 3600 \text{ m}^2
$$

**每個 epoch 完全重新初始化**（white noise model）。這是因為接收機鐘差在 epoch 間沒有可靠的動態模型。

### 5.3 Troposphere (`udtrop_ppp`)

C: line 638 / Java: line 423

- 初始值：SBAS MOPS 模型估計的 ZTD
- Process noise（random walk）：$P_{ii} += \text{prn}[2]^2 |dt|$
- 梯度 process noise：$P_{ii} += (\text{prn}[2] \times 0.1)^2 |dt|$

### 5.4 Phase Bias (`udbias_ppp`)

C: line 722 / Java: line 451

**初始值估計**（IFLC 模式）：

$$
\hat{B} = L_c - P_c \quad \text{(meters)}
$$

**非 IFLC 模式**（需消除電離層影響）：

$$
\hat{I} = \frac{P_1 - P_f}{1 - (f_1/f)^2}, \quad \hat{B}_f = L_f - P_f + 2 \hat{I} (f_1/f)^2
$$

**Phase-code jump detection**：若平均偏移 > 0.15 mm（0.0005c），校正所有 bias。

**Process noise**：$P_{ii} += \text{prn}[0]^2 |dt|$

---

## 6. Measurement Update

### 6.1 Design Matrix H

C: `ppp.c:1007-1043` / Java: `Pppos.java:774-833`

每個量測的 H 行：

| 分量 | Phase ($v_\phi$) | Code ($v_P$) |
|------|---------|--------|
| Position | $-e_k$ (LOS unit vector) | $-e_k$ |
| Clock IC(sys) | 1.0 | 1.0 |
| Troposphere IT | $\partial d_{trop}/\partial ZTD = m_w$ | $m_w$ |
| Trop gradient | $\partial d_{trop}/\partial G_{N,E}$ | same |
| Ionosphere II(sat) | $-(f_1/f)^2 \cdot m_{ion}$ | $+(f_1/f)^2 \cdot m_{ion}$ |
| DCB ID | 0 | 1.0 (L5 code only) |
| Phase bias IB(sat,f) | **1.0** | **0** |

**IFLC 模式**：電離層項為 0（已消除），NF=1，phase bias = IFLC bias。

### 6.2 Kalman Filter

C: `rtkcmn.c:filter_()` line 1459 / 同 RTK

$$
K = P^- H^T (H P^- H^T + R)^{-1}
$$
$$
x^+ = x^- + K v
$$
$$
P^+ = (I - KH) P^-
$$

**非零狀態壓縮**：RTKLIB 只處理 $x_i \neq 0$ 且 $P_{ii} > 0$ 的狀態，大幅減少計算量（NX ~260 但活躍狀態通常 <50）。

### 6.3 Iterative Update

C: `ppp.c:1223-1244` / Java: `Pppos.java:109-135`

```
for iter = 0 to MAX_ITER(=8):
    1. Pre-fit residuals + outlier rejection (|v| > maxinno)
    2. EKF measurement update
    3. Post-fit residuals
    4. 找最大 post-fit outlier (> 4σ) → 排除 → 重跑
    5. 若無 outlier → break (converged)
```

---

## 7. Measurement Noise Model

C: `ppp.c:varerr()` line 330 / Java: `Pppos.java:varerr()` line 953

### 7.1 Variance Formula

$$
\sigma^2 = a^2 + \frac{b^2}{\sin^2(el)}
$$

| 項 | 公式 | 意義 |
|----|------|------|
| $a$ | $F \cdot \text{err}[1]$ | 基本誤差 |
| $b$ | $F \cdot \text{err}[2]$ | 仰角相關誤差 |

**注意**：PPP 的 `varerr` 與 RTK 不同，沒有基線長度項和鐘穩定度項，因為是單站。

### 7.2 Scale Factor F

$$
F = \begin{cases}
\text{eratio}[f] \times \text{EFACT}_{sys} & \text{code} \\
\text{EFACT}_{sys} & \text{phase}
\end{cases}
$$

**系統 factor**：GPS=1.0, GLO=1.5, GAL=1.0, BDS=1.0, SBS=3.0, QZS=1.0, IRN=1.5

**L5 特殊處理**：GPS/QZS L5 額外乘以 EFACT_GPS_L5 = 10.0

### 7.3 Additional Variance Components

$$
\sigma^2_{total} = \sigma^2_{obs} + \sigma^2_{trop} + C^2 \sigma^2_{iono} + \sigma^2_{eph}
$$

- $\sigma^2_{trop}$：對流層模型方差
- $\sigma^2_{iono}$：電離層模型方差（IFLC 為 0）
- $\sigma^2_{eph}$：精密星曆方差（SP3 中提供）

### 7.4 IFLC Noise Amplification

$$
\sigma^2_{IFLC} = 9 \times \sigma^2_{single-freq}
$$

因為 $C_1^2 + C_2^2 \approx 9$（GPS L1/L2）。

### 7.5 R Matrix Structure

PPP 的 R 矩陣是 **對角矩陣**（各量測獨立），不像 RTK 的 DD covariance 有 off-diagonal。

---

## 8. Cycle Slip Detection

### 8.1 Three Detection Methods

C: `ppp.c:udbias_ppp()` / Java: `Pppos.java:udbiasPpp()`

| 方法 | 原理 | 公式 |
|------|------|------|
| **LLI flag** | RINEX 標記 | `obs.LLI[f] & 1` |
| **Geometry-free** | L1-L2 相位差跳變 | $\Delta(L_1 - L_2) > \text{threshold}$ |
| **Melbourne-Wubbena** | 寬巷組合（電離層無關） | 見下文 |

### 8.2 Melbourne-Wubbena Combination

$$
MW = \frac{f_1 L_1 - f_2 L_2}{f_1 - f_2} \cdot c - \frac{f_1 P_1 + f_2 P_2}{f_1 + f_2}
$$

MW 組合消除幾何距離、鐘差、對流層、電離層，理論值為常數（= 寬巷模糊度 $\times \lambda_{WL}$）。
MW 跳變 → cycle slip。PPP 使用 MW 作為最可靠的 cycle slip detector。

---

## 9. PPP-AR (Ambiguity Resolution)

### 9.1 Current Status

C: `ppp_ar.c:14-22` — **Stub implementation**，`ppp_ar()` 直接 return 0。

Java: **未實作**。

PPP 目前只輸出 **float solution** (SOLQ_PPP)。

### 9.2 PPP-AR 理論框架（未實作但已知）

PPP-AR 與 RTK AR 的關鍵差異：

| | RTK AR | PPP-AR |
|--|--------|--------|
| 模糊度性質 | DD → 整數 | Undifferenced → **非整數**（含硬體延遲） |
| 前提 | 基站消除硬體偏差 | 需要外部 **FCB (Fractional Cycle Bias)** 改正 |
| 方法 | LAMBDA on DD | WL fixing → NL fixing → LAMBDA |
| 難點 | 幾何穩定性 | FCB 品質、收斂時間 |

**WL (Wide-Lane) Fixing**：
$$
N_{WL} = \text{round}(MW / \lambda_{WL})
$$

$\lambda_{WL} \approx 86$ cm，容易固定。

**NL (Narrow-Lane) Fixing**：
利用已固定的 WL，形成 NL 模糊度後用 LAMBDA 搜索：

$$
N_{NL} = N_1 - N_{WL}
$$

$\lambda_{NL} \approx 10.7$ cm，需要較高精度的 float solution。

### 9.3 C RTKLIB PPP-AR 調用點

C: `ppp.c:1251`

```c
if (ppp_ar(rtk,obs,n,exc,nav,azel,xp,Pp) && ...) {
    if (norm(std,3) < MAX_STD_FIX)  // MAX_STD_FIX = 0.15 m
        stat = SOLQ_FIX;
}
```

若 PPP-AR 成功且 3D 位置標準差 < 0.15 m → SOLQ_FIX。

---

## 10. PPP vs RTK Comparison

| 面向 | PPP | RTK |
|------|-----|-----|
| **定位方式** | 絕對定位（單站） | 相對定位（基線） |
| **基站需求** | 不需要 | 必要 |
| **精密產品** | SP3 + CLK + ANTEX 必要 | 不需要（廣播星曆即可） |
| **鐘差處理** | 估計（每系統獨立，white noise） | DD 消除 |
| **電離層** | IFLC 消除 或 估計 | DD 消除 或 IFLC |
| **對流層** | 估計（ZTD + 梯度） | DD 大部分消除，可估計 |
| **天線改正** | 必要（衛星+接收機 PCV） | 可選（基線短時差分消除） |
| **相位纏繞** | 必要 | 不需要（差分消除） |
| **潮汐改正** | 必要（~30 cm 級效應） | 短基線不需要 |
| **模糊度** | Float（meters），PPP-AR 需 FCB | DD → 整數，LAMBDA |
| **收斂時間** | **30-60 分鐘** | **數十秒** |
| **穩態精度** | ~cm (float), ~mm (PPP-AR) | ~mm (fix) |
| **適用場景** | 偏遠地區、全球覆蓋 | 短~中基線 (<50 km) |
| **State dimension** | ~260 (3+7+1+MAXSAT) | ~473 (3+MAXSAT*2) |
| **R matrix** | 對角（各量測獨立） | Block-diagonal（DD 共享 ref） |

---

## 11. Code Reference Map

### C Source (`src/ppp.c`)

| Function | Lines | Purpose |
|----------|-------|---------|
| `udstate_ppp()` | 811-835 | Master time update dispatcher |
| `udpos_ppp()` | 525-615 | Position state update |
| `udclk_ppp()` | 617-636 | Clock white noise reset |
| `udtrop_ppp()` | 638-663 | Troposphere update |
| `udbias_ppp()` | 722-809 | Phase bias update + cycle slip |
| `udiono_ppp()` | 665-721 | Ionosphere update (EST mode) |
| `uddcb_ppp()` | — | L5 DCB update |
| `ppp_res()` | 936-1095 | Residuals, H matrix, R matrix |
| `varerr()` | 330-373 | Measurement noise model |
| `corr_meas()` | 409-458 | Antenna, wind-up, code bias corrections |
| `model_trop()` | 884-906 | Troposphere model dispatch |
| `trop_model_prec()` | 856-881 | Precise trop with mapping + gradients |
| `model_iono()` | 908-934 | Ionosphere model dispatch |
| `model_phw()` | 290-328 | Phase wind-up computation |
| `satantpcv()` | 837-854 | Satellite antenna PCV |
| `pppos()` | 1173-1275 | Main PPP entry (iteration loop) |
| `detslp_gf()` | 481-502 | Geometry-free cycle slip detection |
| `detslp_mw()` | 503-524 | Melbourne-Wubbena cycle slip detection |
| `detslp_ll()` | 476-480 | LLI flag cycle slip detection |

### C Source (`src/ppp_ar.c`)

| Function | Lines | Purpose |
|----------|-------|---------|
| `ppp_ar()` | 14-22 | **Stub** — always returns 0 |

### Java Port (`positioning/Pppos.java`)

| Method | Lines | Purpose |
|--------|-------|---------|
| `pppos()` | 75-155 | Main entry (SPP + EKF iteration) |
| `udstatePpp()` | 372-379 | Master time update |
| `udposPpp()` | 382-407 | Position update |
| `udclkPpp()` | 410-420 | Clock white noise |
| `udtropPpp()` | 423-448 | Troposphere update |
| `udbiasPpp()` | 451-522 | Phase bias update + cycle slip |
| `pppRes()` | 660-885 | Residuals + H + R |
| `varerr()` | 953-986 | Measurement noise |
| `corrMeas()` | 599-648 | Measurement corrections |
| `modelTrop()` | 894-920 | Troposphere model |
| `modelPhw()` | 237-278 | Phase wind-up |
| `satantoff()` | 289-331 | Satellite antenna IFLC offset |
| `antmodel()` | 341-357 | Receiver antenna model |
| `detslpGf()` | — | Geometry-free slip detection |
| `detslpMw()` | — | Melbourne-Wubbena slip detection |

---

## 12. Parameter Reference

### PPP-Specific Options

| Parameter | Field | Default | Description |
|-----------|-------|---------|-------------|
| Mode | `mode` | PPP_STATIC | PPP_KINEMA / PPP_STATIC / PPP_FIXED |
| Satellite ephemeris | `sateph` | EPHOPT_PREC | Must be PREC for PPP |
| Ionosphere | `ionoopt` | IONOOPT_IFLC | IFLC (default) / EST / BRDC |
| Troposphere | `tropopt` | TROPOPT_EST | EST (ZTD) / ESTG (ZTD+grad) |
| Tide correction | `tidecorr` | 0 | bit0: solid, bit1: OTL, bit2: pole |
| Sat antenna PCV | `posopt[0]` | 0 | 0: off, 1: on |
| Rcv antenna PCV | `posopt[1]` | 0 | 0: off, 1: on |
| Phase wind-up | `posopt[2]` | 0 | 0: off, 1: on |

### Measurement Noise

| Parameter | Field | Default | Unit | Meaning |
|-----------|-------|---------|------|---------|
| Base error | `err[1]` | 0.003 | m | Phase base noise |
| Elev error | `err[2]` | 0.003 | m | b/sin(el) term |
| Code/phase ratio | `eratio[f]` | 100-300 | - | Code noise / phase noise |

### State Initial / Process Noise

| Parameter | Field | Default | Unit | Meaning |
|-----------|-------|---------|------|---------|
| Position initial σ | — | 30 | m | VAR_POS = 900 m² |
| Clock initial σ | — | 60 | m | VAR_CLK = 3600 m² |
| Bias initial σ | — | 60 | m | VAR_BIAS = 3600 m² |
| Iono initial σ | — | 60 | m | VAR_IONO = 3600 m² |
| Bias process noise | `prn[0]` | 1e-4 | m/√s | Phase bias random walk |
| Iono process noise | `prn[1]` | 1e-3 | m/√s | Ionosphere random walk |
| Trop process noise | `prn[2]` | 1e-4 | m/√s | ZTD random walk |
| Accel H process noise | `prn[3]` | 1.0 | m/s²/√s | Kinematic horizontal |
| Accel V process noise | `prn[4]` | 0.1 | m/s²/√s | Kinematic vertical |
| Position process noise | `prn[5]` | 0 | m/√s | Static position drift |

### PPP Solution Quality

| Parameter | Value | Meaning |
|-----------|-------|---------|
| MAX_ITER | 8 | Maximum EKF iterations per epoch |
| MAX_STD_FIX | 0.15 m | 3D σ threshold for FIX (PPP-AR) |
| THRES_REJECT | 4.0 | Post-fit outlier rejection (σ multiplier) |
| MIN_NSAT_SOL | 4 | Minimum satellites for valid solution |
