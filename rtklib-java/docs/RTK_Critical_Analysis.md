# RTKLIB RTK Static: Critical Algorithm Analysis

> Deployment context: GeoGuard mountain monitoring, u-blox F9P/F9T (L1+L5),
> GPS/GLO/BDS/GAL/QZSS, short baselines (<30km), 10-min sessions (30s epoch interval, ~20 epochs/session)
> with 3hr sliding window, Taiwan mountain sites (vegetation, terrain reflection).

---

## 1. Ratio Test — 固定門檻 3.0

### RTKLIB 做法

LAMBDA 搜索兩個最佳整數解，$\text{ratio} = s_2/s_1 > 3.0$ 則 FIX。
C: `rtkpos.c:1751` / Java: `Rtkpos.java:1108`

### 批判

固定門檻沒有統計基礎，無法控制實際失敗率。但在 GeoGuard 的五星座場景下，
**模型強度通常很高（nb=15~25 DDs）**，固定 3.0 門檻的問題不是「弱模型誤 fix」，
而是**過度保守導致不必要的 FLOAT**。FFRT 在此場景的主要價值是 **提升 fix rate**。

驗算：nb=20, nom=3.0 → FFRT threshold ≈ 1.69。大量原本 ratio=2.0~2.9 被拒的 epoch
可以通過自適應門檻，且等失敗率保證下安全性不降。

> **Verhagen & Teunissen (2009)** "The GNSS ambiguity ratio-test revisited" *Survey Review* 41(312)
> — critical value 應取決於模型強度。固定 3.0 在不同場景失敗率從 <0.1% 到 >5%。

> **Verhagen & Teunissen (2013)** "The ratio test for future GNSS AR" *GPS Solutions* 17(4)
> — FFRT 通過 lookup table 自適應門檻，同正確率下 fix rate 提升達 30%。

### 現狀

Java EKF 已啟用 FFRT (`Rtkpos.computeAdaptiveArThreshold()`)。
Station B (nb=20) 門檻從 3.0 降至 1.69，fix rate 從 65.6% 升至 68.8%。

**Note**: fix rate 僅提升 3.2 個百分點，遠低於文獻宣稱的 30%。原因是 10 分鐘 session
僅 ~20 個 epoch（30s interval），filter 收斂時間本身是瓶頸——早期 epoch 的 float
精度不足導致 ratio 低，FFRT 降低門檻也救不回來。FFRT 的 30% 改善是在長時段
連續 RTK（filter 已充分收斂）場景下的數字。

---

## 2. Fix-and-Hold — 錯誤整數鎖定風險

### RTKLIB 做法

連續 `minfix` (預設 10) 個 epoch FIX 後，`holdamb()` 注入 pseudo-measurement：

$$
v_i = (x_{a,ref} - x_{a,i}) - (x_{ref} - x_i), \quad R = \text{varholdamb} \cdot I
$$

**關鍵數值**：`varholdamb` **預設 0.1 cycle²**。

C: `rtkpos.c:1645` / Java: `Rtkpos.java:1016`, `ProcessingOptions.java:135`

這意味著 hold 注入的約束 σ = 0.316 cycle ≈ 6 cm (L1)。一旦注入，filter 極難偏離整數值。

### 批判

**一旦鎖定錯誤整數，0.1 cycle² 的約束讓 filter 幾乎無法自我修正**。
錯誤整數產生 >1 波長（~19 cm L1）的位置偏差，且 propagate 到所有後續 epoch。

> **Teunissen (2005)** "Integer aperture bootstrapping" *Journal of Geodesy* 79(6-7)
> — 錯誤整數導致 >λ 的位置誤差。IA estimation 在 fixing 可靠度不足時保持 float。

> **Psychas, Verhagen & Teunissen (2020)** "Precision analysis of partial AR" *Adv. Space Res.* 66(12)
> — 錯誤固定的解比 float 更差。Partial AR（只固定可靠子集）更安全。

### 緩解機制

RTKLIB 有兩道防線，但都不是根本解法：

**arfilter** (`Rtkpos.java:1216-1238`)：若 ratio 突然下降（新衛星加入後），
排除 `lock=0` 的新衛星並重跑 AR。觸發條件：
- 上一 epoch ratio >= threshold 且
- 本 epoch ratio < threshold 或 ratio < 1.1×threshold 且 < 0.5×prevRatio

**satellite dropout** (`Rtkpos.java:1173-1204`)：若連續 AR 失敗（prevRatio2 < threshold），
逐一排除衛星重試。觸發條件：nb_ar >= `mindropsats` (預設 5)。

### GeoGuard 場景特定評估

**風險較低**：10 分鐘 session 獨立初始化，每個 session 約 20 個 epoch（30s interval）。
即使某個 session 鎖定錯誤整數，不會跨 session 傳播。
與長時間連續 RTK（數小時不重啟）相比，fix-and-hold 的風險被 session 設計有效隔離。

**`minfix` 在此場景的意義**：30s epoch interval 下，`minfix=10` 意味著需要連續
10 個 epoch（5 分鐘）fix 才觸發 hold，佔整個 session 的 50%。這已經相當保守——
如果一個 session 前半都能 fix，整數正確的可信度很高。進一步提高 `minfix`
會導致多數 session 根本無法觸發 hold，失去 fix-and-hold 的收斂加速效果。
**結論**：在 30s interval / 20 epoch session 下，`minfix=10` 是合理的平衡點。

---

## 3. Standard EKF — 無 Robust Estimation

### RTKLIB 做法

標準 EKF，假設高斯噪聲。Outlier 靠 `maxinno` 硬門檻二元剔除（全部接受/全部拒絕）。
C: `rtkpos.c:1366-1382` / Java: `Rtkpos.java:829-842`

### 批判

**台灣山區站點有植被遮蔽和地形反射，不是典型開闊環境**。
multipath 和 NLOS 產生非高斯、heavy-tailed 誤差。
硬門檻剔除的問題：剛好低於門檻的壞觀測完全進入 filter，剛好高於的觀測完全丟失。

> **Crespillo et al. (2021)** "Robust Filtering Techniques for RTK" *Sensors* 21(4)
> — M-estimation（Huber/Tukey weight function）連續降權而非 binary reject。

> **Wen & Hsu (2021)** "Towards Robust GNSS Positioning Using FGO" *ICRA 2021*
> — EKF 對 outlier 敏感。Robust kernel 在城市峽谷顯著優於 EKF。

### GeoGuard 場景評估

山區站點介於「開闊環境」和「城市峽谷」之間：植被衍射、地形反射產生中等程度的 multipath，
但不如城市 NLOS 嚴重。**Robust EKF（Huber weighting on innovation）的邊際收益可觀**，
尤其對低仰角衛星穿過樹冠層時的觀測。

實作路線：在 `ddres()` 的 innovation 上加 Huber weight function，
不需改 filter 核心，只需修改 R 矩陣的 diagonal entry（動態放大壞觀測的方差）。

---

## 4. Elevation Weighting — $1/\sin^2(el)$ 模型

### RTKLIB 做法

$$
\sigma^2 = 2(a^2 + b^2/\sin^2(el) + c^2) + d^2
$$

純仰角函數，不考慮 SNR。`err[6]=0` (SNR weighting off), `err[7]=0` (receiver std off)。

C: `rtkpos.c:varerr()` line 402 / Java: `Rtkpos.java:varerr()` line 172

### 批判

$1/\sin^2(el)$ 是平均統計近似，**無法捕捉站點特定的 multipath pattern**。
山區站點的問題：
- 高仰角衛星穿過樹冠 → 衍射噪聲增大，但 $1/\sin^2$ 給它最低權重
- 低仰角衛星在特定方位可能無遮蔽 → 過度降權浪費觀測

> **Luo et al. (2014)** "A Realistic Weighting Model for GPS Phase Observations" *IEEE TGRS* 52(10)
> — EXPZ 模型：AR 成功率 +10%，對流層估計 +40%，座標重複性改善最多 2.3 mm (50%)。

> **Brunner, Hartinger & Troyer (1999)** "GPS signal diffraction modelling" *J. Geodesy* 73(5)
> — SNR-based weighting 解決了 10% 更多的模糊度。

### GeoGuard 場景評估

**高優先級改善，且實作成本極低**。u-blox F9P/F9T 報告 per-satellite C/N0，
直接可用於 SNR weighting。RTKLIB 已有 `err[6]` 框架：

$$
\sigma^2 += e^2 \cdot (10^{0.1 \cdot \max(SNR_0 - SNR_{rover}, 0)} + 10^{0.1 \cdot \max(SNR_0 - SNR_{base}, 0)})
$$

**只需設定 `err[5]`（SNR 門檻, ~35 dBHz）和 `err[6]`（SNR factor, ~0.003）即可啟用**。
山區站點受益高於一般開闊環境。

---

## 5. Cycle Slip Detection — 最關鍵的遺漏議題

### RTKLIB 做法

RTK 使用 **四重檢測**（按呼叫順序）：

| 方法 | 函數 | 原理 | 對 L1+L5 的適用性 |
|------|------|------|-------------------|
| Doppler | `detslpDop()` :286 | $\Delta\phi/\Delta t$ vs $-D_f$ | 需要有效 Doppler |
| Code change | `detslpCode()` :328 | 信號碼類型切換 | 檢測 tracking mode 變化 |
| LLI flag | `detslpLl()` :233 | RINEX LLI 標記 | 依賴接收機報告 |
| Geometry-free | `detslpGf()` :266 | $\Delta(L_1 - L_k)$ 跳變 | **L1-L5 可用** |

### 批判

**RTK 沒有 Melbourne-Wubbena 檢測**（PPP 才有 `detslp_mw()`），
而 MW 恰好是最可靠的電離層無關 cycle slip detector。

更嚴重的問題：**Geometry-free (GF) 檢測對 L1+L5 是雙面刃**。

GF 組合量測的是電離層延遲的頻率差異部分：
- $L_1 - L_2$：電離層項 $= I \cdot (f_1^2/f_2^2 - 1) \cdot \lambda_1$，其中 $f_1^2/f_2^2 = 1.647$
- $L_1 - L_5$：電離層項 $= I \cdot (f_1^2/f_5^2 - 1) \cdot \lambda_1$，其中 $f_1^2/f_5^2 = 1.793$

$f_5 < f_2$，所以 L1-L5 的電離層項比 L1-L2 **更大**（係數 0.793 vs 0.647）。
這意味著**電離層時變造成的 GF 背景波動也更大**——cycle slip 的信號固然更大，
但噪聲底也更高，**slip 的 signal-to-noise ratio 不一定改善**。

因此 GF 門檻 (`thresslip` 預設 0.05 m) **不能直接沿用**，
需要用實際 L1+L5 資料校準，考慮電離層活躍度對 GF 背景的影響。

**u-blox F9T 特有問題**：
- F9T 在 L5 上偶爾出現 half-cycle slip（LLI flag 未必可靠）
- Doppler 在 L5 可能不如 L1 穩定

### MW 在 L1+L5 的限制

MW combination 移植到 RTK 需注意：MW 精度直接受 code noise 影響。

- L1/L2 wide-lane: $\lambda_{WL}$ = c/(f1-f2) ≈ **0.862 m**
- L1/L5 wide-lane: $\lambda_{WL}$ = c/(f1-f5) ≈ **0.751 m**

u-blox F9P/F9T 在山區環境下的 pseudorange noise 約 1~3 m（multipath），
而 L1/L5 的 wide-lane 波長只有 0.751 m——**code noise 接近甚至超過一個 WL 波長**。
因此 MW 在 L1+L5 上的 cycle slip detection 靈敏度 **不如 L1+L2 組合**，
single-epoch MW 可能不可靠，需要多 epoch 平滑後使用。

### 建議

1. **將 MW combination 引入 RTK cycle slip detection**（從 `ppp.c:detslp_mw()` 移植），
   但需加入多 epoch 平滑（running average）以應對 L1+L5 的 code noise
2. 校準 `thresslip` 門檻，用實際 L1+L5 資料驗證 GF 靈敏度和 SNR
3. 監控 F9T 的 L5 half-cycle 問題，必要時加入 half-cycle ambiguity 處理

---

## 6. Forward+Backward 組合 ≠ RTS Smoother

### RTKLIB 做法

C: `postpos.c:combres()` / Java: `PostProcessor.java:combres()`

```
foreach epoch:
  if (quality_fwd > quality_bwd) → use forward
  if (quality_bwd > quality_fwd) → use backward
  if (quality_fwd == quality_bwd) → smoother fusion
```

品質比較基於 `stat` 欄位：FIX(1) > FLOAT(2) > DGPS(4) > SINGLE(5)。
同品質時：$P_{fused}^{-1} = P_f^{-1} + P_b^{-1}$。

### 批判

**不是統計最優的 fixed-interval smoother**。RTS smoother 的正確公式是：

$$
x_s = x_f + G_k(x_{s,k+1} - x_{p,k+1}), \quad G_k = P_f F^T P_{p,k+1}^{-1}
$$

RTKLIB 的簡單 fusion **double-count process noise**，因為 forward 和 backward
各自獨立加入了 process noise。

> **Vaclavovic & Dousa (2015)** "Backward smoothing for precise GNSS applications" *Adv. Space Res.* 56(8)
> — RTS smoother 在資料邊緣改善最大。

> **Banville et al. (2021)** "Enabling ambiguity resolution in CSRS-PPP" *Navigation* 68(2)
> — RTS 有數值不穩定性（協方差減法失去正定性），CSRS-PPP v3 改用 square-root information smoother。

### 邊界 epoch 行為

品質比較的 heuristic 在 **剛 fix 或剛 lose fix** 的邊界 epoch 行為值得注意：
- Forward 方向在 epoch 15 首次 FIX，backward 方向在同 epoch 仍是 FLOAT
  → 取 forward FIX（正確）
- Forward 方向在 epoch 80 lose fix（cycle slip），backward 方向仍 FIX
  → 取 backward FIX（正確，這正是 combined 的價值）
- 但若兩方向在同 epoch **都是 FIX 但固定到不同整數組合**
  → fusion 產生 **平均值**，既不是正確的 forward 也不是正確的 backward

**品質控制建議**：在短基線 static 下，forward/backward 都 FIX 但位置差 > 數 cm
幾乎必然意味著其中一個方向固定了錯誤整數。正確處理不是 fusion，
而是 **flag 該 epoch 為可疑**，降級為 FLOAT。這可以作為後處理 pipeline 的 QC 指標。

### GeoGuard 場景評估

10 分鐘 session：forward 和 backward 各自都有充分時間收斂，
smoother 邊際改善小。但 combined mode 的主要價值是處理 **session 中間的 cycle slip**
——forward 在 slip 後重收斂，backward 在 slip 前已收斂，combined 取較好者。

---

## 7. Ambiguity Process Noise — 設為 0 的隱含假設

### RTKLIB 做法

Phase bias 的 process noise `prn[0]`：

```java
rtk.P[j + j * rtk.nx] += rtk.opt.prn[0] * rtk.opt.prn[0] * Math.abs(tt);
```

`prn[0]` **預設為非零**（~1e-4 m/√s），但極小。量化其效果：

在 30s epoch interval 下，一個 epoch 的 bias random walk：
$$\sigma_{1epoch} = 10^{-4} \times \sqrt{30} \approx 5.5 \times 10^{-4} \text{ m} \approx 0.55 \text{ mm}$$

若漏檢一個完整 cycle slip（L1 ≈ 19 cm），filter 靠 process noise 自然 drift 回來需要：
$$(0.19 / 5.5 \times 10^{-4})^2 \approx 119{,}000 \text{ epochs} \approx 41 \text{ days}$$

**結論：process noise 作為 cycle slip 安全網完全無效**。
RTKLIB 完全依賴 cycle slip detection 來決定是否 reset bias——
如果 detection 漏掉一個 slip，filter 會在錯誤的 bias 上一直跑到 session 結束。

### 批判

對 u-blox L1+L5，cycle slip detection 的可靠性直接決定 bias 估計品質。
如果 GF/Doppler/LLI 三重檢測都漏掉一個 slip（例如 half-cycle slip），
後續所有 epoch 的該衛星 bias 都是錯的，且 process noise 太小無法自然修正。

> **Mohamed & Schwarz (1999)** "Adaptive Kalman Filtering for INS/GPS" *J. Geodesy* 73(4)
> — 固定 Q 矩陣在動態變化環境下次優。Innovation-based adaptive estimation 改善 ~20%。

> **Zhang et al. (2018)** "Adaptive KF based on VCE for ionospheric delay prediction" *J. Geodesy* 92(11)
> — Helmert VCE 自適應調整 process noise，避免 over/under-smoothing。

### GeoGuard 場景評估

山區站點、低成本接收機、L5 tracking 不穩定——**cycle slip 漏檢是實際風險**。
比起自適應 process noise（實作複雜度高），更直接的改善是 **強化 cycle slip detection**
（引入 MW combination、校準 L1+L5 GF 門檻）。

---

## 改善優先級（GeoGuard 場景）

| # | 改善方向 | 影響度 | 難度 | 驗證方法 | 狀態 |
|---|----------|--------|------|----------|------|
| 1 | **FFRT adaptive threshold** | 高 | 低 | fix rate 前後比較 | **已完成** |
| 2 | **SNR-based weighting** (err[6]) | 中高 | **極低** | 座標重複性、fix rate 比較 | config 調參 |
| 3 | **MW cycle slip detection** | 高 | 低~中 | 已知 slip 的 detection rate | 待實作 |
| 4 | **L1+L5 GF threshold 校準** | 中 | 低 | GF 時序圖 + false alarm rate | 待驗證 |
| 5 | **Robust EKF** (Huber) | 中 | 中 | multipath epoch 的位置偏差比較 | 待評估 |
| 6 | **Fwd+Bwd FIX 不一致 QC flag** | 中 | 低 | flagged epoch 數量與座標離群分析 | 待實作 |
| 7 | **Partial AR** | 高 | **高** | 錯誤 fix rate (需已知基準座標) | 需改 LAMBDA |
| 8 | **RTS smoother** | 低~中 | 中 | 邊緣 epoch 精度比較 | PPP 優先 |
| 9 | **Adaptive process noise** (VCE) | 低 | 高 | innovation consistency test | 長期目標 |
