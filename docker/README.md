# RTKLIB 解算服務

把 RINEX 觀測檔丟給它，回傳定位解。以 HTTP 提供，包成單一 Docker image，供內網使用。

---

## 一分鐘上手

```bash
# 建置並啟動（預設只綁在本機）
docker compose -f docker/docker-compose.yml up -d --build

# 送一組資料進去
curl -X POST http://127.0.0.1:8000/solve \
     -F "rover=@rover.obs" \
     -F "base=@base.obs" \
     -F "nav=@brdc.nav" \
     -F "preset=static"
```

回傳 JSON，其中 `pos` 欄位就是 RTKLIB 的 `.pos` 內容：

```json
{
  "job": "8b21e579560c",
  "solver": "ekf",
  "preset": "static",
  "epochs": 115,
  "quality": { "fix": 112, "float": 3 },
  "pos": "% program   : RTKLIB ver.EX 2.5.2\n..."
}
```

---

## 先講清楚：這個服務用的是 EKF，不是 FGO

這個 repository 正在開發 Factor Graph Optimization（見 [`plan.md`](../plan.md)），而
**FGO 目前還不能解算**。`fgo_process_epoch()` 回傳 `FGO_ERR_NOTCONV`，`rtkpos.c`
也還沒有 solver 分支（那是 PR-7）。

所以：

- `/solve` 產出的**所有**解都來自 RTKLIB 原本的 Extended Kalman Filter。
- image 裡確實編入了 FGO 的 C ABI 並連結 GTSAM 4.2.2，`/capabilities` 會如實回報。
- 若指定 `solver=fgo-batch` / `fgo-sliding` / `fgo-isam2`，服務會回 **HTTP 400 拒絕**，
  而不是默默改用 EKF 給你一個數字。這是刻意的：拿到解的人會合理假設那就是他點的
  solver。

想確認現況，直接問服務：

```bash
curl -s http://127.0.0.1:8000/capabilities | jq .fgo
```

---

## API

### `POST /solve`

`multipart/form-data`：

| 欄位 | 必填 | 說明 |
|---|---|---|
| `rover` | ✔ | 移動站（或待測站）觀測檔 RINEX |
| `base` | | 基站觀測檔。**省略即為單點定位**（無差分） |
| `nav` | ✔ | 導航電文 RINEX |
| `preset` | | 選項組合名稱，預設 `static`。見下方 |
| `solver` | | 只接受 `ekf`（預設）。其他值回 400 |
| `stat` | | `true` 時另外回傳 `.stat`（逐衛星殘差，檔案相當大） |

成功回 `200`，內容如上。

**失敗的回應**，以及各代表什麼：

| 狀態 | 意思 |
|---|---|
| `400` | 參數有問題——例如指定了不存在的 solver，或 preset 名稱含非法字元 |
| `404` | preset 不存在。用 `/capabilities` 看有哪些 |
| `413` | 上傳總量超過上限（預設 256 MB） |
| `422` | 解算失敗，或**解算成功但一個 epoch 都沒有**。後者通常是 rover/base 沒有共同時段或共同衛星。回應中含 `stderr` 節錄 |
| `504` | 超過時間上限（預設 900 秒） |

### `GET /capabilities`

回報這個 image 實際的能力：RTKLIB 版本與 ABI、編譯巨集、可用 solver、FGO 狀態、
可用 preset、上傳與逾時上限。**部署後第一件該做的事就是打這支**。

### `GET /presets/{name}`

回傳該 preset 的原始 `.conf` 內容。想知道 `static` 到底設了什麼，看這裡，不必猜。

### `GET /health`

給 load balancer 與 healthcheck 用。它會真的去執行一次 solver（`rnx2rtkp --version`），
所以動態連結壞掉會被抓出來，不只是「行程還活著」。

### `GET /docs`

FastAPI 自動產生的互動式文件，可直接在瀏覽器上傳檔案試打。

---

## Preset

| 名稱 | 用途 |
|---|---|
| `static` | 靜態測站，連續模糊度解算。**變形監測用這個** |
| `kinematic` | 移動載體 |
| `single` | 單點定位，不需要 base |
| `iflc` | 無電離層組合，適合長基線雙頻 |

四者都設為 GPS+GLONASS+Galileo+BeiDou、仰角遮罩 15°。要調整就改
`docker/conf/*.conf` 後重新建置；檔名即 preset 名稱。

---

## 部署到內網

image 已在本機驗證可跑。推送與上線由你執行：

```bash
# 1. 標記並推到內網 registry
docker tag rtklib-solve:local <registry>/rtklib-solve:2.5.2
docker push <registry>/rtklib-solve:2.5.2

# 2. 在目標主機上用既有的 image，不要在那邊重建
RTKLIB_IMAGE=<registry>/rtklib-solve:2.5.2 \
RTKLIB_BIND=0.0.0.0 \
docker compose -f docker/docker-compose.yml up -d
```

### 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `RTKLIB_IMAGE` | `rtklib-solve:local` | 要跑哪個 image |
| `RTKLIB_BIND` | `127.0.0.1` | 綁定介面。**改成 `0.0.0.0` 前請先讀下一節** |
| `RTKLIB_PORT` | `8000` | 對外 port |
| `RTKLIB_MAX_UPLOAD_MB` | `256` | 單次請求上傳總量上限 |
| `RTKLIB_SOLVE_TIMEOUT_S` | `900` | 單次解算逾時 |
| `RTKLIB_CPUS` / `RTKLIB_MEMORY` | `2.0` / `2g` | 資源上限 |
| `RTKLIB_TMPFS` | `2g` | 暫存空間，**必須大於上傳上限** |

### 這個服務沒有身分驗證

刻意如此——它不知道你內網的驗證方式。預設只綁 `127.0.0.1`，就是為了逼人在打開之前
先想清楚。放到內網前，至少做到其中一項：

- 擺在已有驗證的反向代理（nginx / Traefik）後面；
- 用防火牆限制來源 IP；
- 放進只有應用層可達的內部網段。

**別直接把 `RTKLIB_BIND=0.0.0.0` 加上去就上線。** 任何能連到它的人都能送任意檔案
進來消耗 CPU。

已經做到的加固：非 root 執行、唯讀根檔案系統、丟棄全部 Linux capability、
`no-new-privileges`、CPU 與記憶體上限，以及上傳落在 tmpfs——請求結束後不留任何
使用者資料在磁碟上。

---

## 關於解算結果的兩點說明

**與開發用的回歸基準不會逐位元相同。** image 由 CMake 建置，用的是
`-DNFREQ=3 -DNEXOBS=3` 加多星系巨集；而 `test/regression/` 的基準是由 console
makefile 以 `-DNFREQ=4` 建的。兩者都正確，但編譯期巨集會改變結果，所以請不要拿
image 的輸出去比對 baseline。`/capabilities` 的 `rtklib.macros` 會列出這個 image
實際用的巨集。

**`epochs: 0` 不算成功。** RTKLIB 在找不到共同衛星時仍會回傳 0 並產生只有標頭的
`.pos`。服務會把這種情況轉成 `422`，因為對呼叫端而言那是失敗。

---

## 疑難排解

| 症狀 | 先看這裡 |
|---|---|
| `422` 且 stderr 提到 `no common satellite` | rover 與 base 的時段沒有重疊，或星系設定排除了共同衛星 |
| 全部 epoch 都是 `single` | 沒有給 `base`，或 preset 用了 `single` |
| 沒有 `fix`、只有 `float` | 資料時間太短、基線太長，或 preset 的頻率設定與資料不符 |
| `504` | 資料量大就調高 `RTKLIB_SOLVE_TIMEOUT_S`，並確認 `RTKLIB_CPUS` 夠用 |
| 容器起不來 | `docker logs rtklib-solve`。`/health` 會真的執行 solver，動態連結問題會顯示在這裡 |

---

## 這個 image 裡有什麼

多階段建置。builder 用 `tools/fgo/conda-linux-64.lock`（與開發環境同一份鎖定檔）
取得 GTSAM，以 CMake 建置；runtime 只複製執行檔、`librtklib.so.3` 以及 `ldd` 判定
實際需要的共用函式庫。最終 image 約 **174 MB**（`ENABLE_FGO=OFF` 為 163 MB），不含 conda。

編譯器刻意使用系統 gcc 而非 conda 的，理由見
[`docs/fgo/build_environment.md`](../docs/fgo/build_environment.md)：EKF 的
byte-diff 基準必須可重現。

想跳過 GTSAM 讓建置快很多（服務行為完全相同，只有 `/capabilities` 顯示不同）：

```bash
docker build -f docker/Dockerfile --build-arg ENABLE_FGO=OFF -t rtklib-solve:nofgo .
```
