# RTKLIB 解算 API — 使用說明

送 RINEX 進去，拿定位解出來。這份是給**呼叫這個 API 的人**看的；要自己架設請看
[`README.md`](README.md)。

服務位址請向維運人員索取，以下一律以 `http://rtklib:8000` 代表。

---

## 先確認兩件事

**一、這個服務用的是 EKF。** 本 repository 正在開發 Factor Graph Optimization，但
FGO 目前還不能解算，所有的解都來自 RTKLIB 原本的 Extended Kalman Filter。若你指定
`solver=fgo-*`，服務會回 400 拒絕，不會默默改用 EKF 給你一個數字。

**二、公分級是「相對於基站」的。** 沒有基站就只能單點定位（公尺級），這是 GNSS 本身
的限制。但有了基站也**不代表絕對座標就準**——服務直接採用基站 RINEX 標頭裡的
`APPROX POSITION XYZ`（preset 的 `ant2-postype=rinexhead`），而那個值往往只是接收機
自己算的概略解，可能差好幾公尺。

結果是：`fix` 的解相對於基站是公分級，但**整組解會連同基站的誤差一起平移**。

所以做控制點測量或變形監測時：

- 基站標頭裡必須是**實測過的精確座標**，不能是接收機的概略解；
- 變形監測若只關心「隨時間的變化量」，基站的絕對誤差會被抵消，影響不大；
- 但要輸出絕對座標，先確認基站座標從何而來。

不確定的話，看一眼基站檔的標頭：

```bash
grep "APPROX POSITION" base.obs
```

---

## 最短路徑

```bash
curl -X POST http://rtklib:8000/solve \
     -F "rover=@rover.obs" \
     -F "base=@base.obs" \
     -F "nav=@brdc.nav" \
     -F "preset=static" \
     -o result.json
```

拿到的 JSON 裡，`pos` 就是 RTKLIB 的 `.pos` 內容，可直接存成檔案：

```bash
python3 -c "import json;print(json.load(open('result.json'))['pos'])" > result.pos
```

先看一眼結果好不好：

```bash
python3 -c "
import json; d=json.load(open('result.json'))
print('epochs :', d['epochs'])
print('quality:', d['quality'])"
```

輸出類似：

```
epochs : 115
quality: {'fix': 112, 'float': 3}
```

`fix` 越多越好——那是模糊度已解算、公分級的 epoch。

---

## 選哪個 preset

| preset | 什麼時候用 | 需要 base |
|---|---|---|
| `static` | 測站不動。**變形監測、控制點測量用這個** | 是 |
| `kinematic` | 測站在移動（車載、機載、手持） | 是 |
| `iflc` | 長基線（>20 km）雙頻資料，消去電離層 | 是 |
| `single` | 只有一台接收機，或只需要公尺級 | 否 |

不確定就從 `static`（靜態）或 `kinematic`（移動）開始。

想知道某個 preset 到底設了什麼，直接要原始檔：

```bash
curl http://rtklib:8000/presets/static
```

---

## API

### `POST /solve`

`multipart/form-data`：

| 欄位 | 型別 | 必填 | 說明 |
|---|---|---|---|
| `rover` | 檔案 | ✔ | 待測站觀測檔（RINEX OBS） |
| `base` | 檔案 | | 基站觀測檔。差分模式必填 |
| `nav` | 檔案 | ✔ | 導航電文（RINEX NAV） |
| `preset` | 字串 | | 預設 `static` |
| `solver` | 字串 | | 只接受 `ekf`（預設） |
| `stat` | 布林 | | `true` 時額外回傳逐衛星殘差，檔案很大 |

成功回 `200`：

```json
{
  "job": "8b21e579560c",
  "solver": "ekf",
  "preset": "static",
  "epochs": 115,
  "quality": { "fix": 112, "float": 3 },
  "pos": "% program   : RTKLIB ver.EX 2.5.2\n1316 518400.000  35.160874407 ..."
}
```

| 欄位 | 意思 |
|---|---|
| `job` | 這次請求的識別碼，回報問題時附上 |
| `epochs` | 解出來的 epoch 數 |
| `quality` | 各品質等級的 epoch 數，見下表 |
| `pos` | `.pos` 檔完整內容 |
| `stat` | 只有在 `stat=true` 時出現 |

品質等級：

| 名稱 | RTKLIB Q | 意義 | 大致精度 |
|---|---|---|---|
| `fix` | 1 | 模糊度已固定 | 公分級 |
| `float` | 2 | 模糊度浮動解 | 分米級 |
| `dgps` | 4 | 差分碼解 | 次公尺 |
| `single` | 5 | 單點定位 | 公尺級 |

### `GET /capabilities`

回報這個服務實際的能力：版本、可用 solver、preset 清單、大小與逾時上限。
**接上新服務時先打這支。**

### `GET /presets/{name}`

回傳該 preset 的原始 `.conf`，純文字。

### `GET /health`

正常回 `200`，solver 有問題回 `503`。

### `GET /docs`

瀏覽器打開就能直接上傳檔案試打，不用寫程式。

---

## 錯誤怎麼看

| 狀態 | 意思 | 怎麼辦 |
|---|---|---|
| `400` | 參數有問題 | 看 `detail`。最常見是**沒給 `base` 卻用了差分 preset**，回應會列出可用的單點 preset |
| `404` | preset 不存在 | 打 `/capabilities` 看有哪些 |
| `413` | 上傳太大 | 見下方大小限制 |
| `422` | 解算失敗，或解出 0 個 epoch | 看 `detail.stderr` |
| `504` | 超過時間上限 | 資料太大，請維運人員調高上限 |

`422` 最常見的兩種原因：

**`no common satellite`** — rover 和 base 的時間區間沒有重疊，或兩者共同觀測到的
衛星被仰角/星系設定濾掉了。先確認兩個檔案是同一時段。

**`epochs: 0`** — 解算跑完但一個 epoch 都沒有。服務把這種情況當失敗回 `422`，因為
對你來說那不是成功。通常同上。

---

## 大小與時間限制

上限預設 **256 MB**，限制的是**整個 request**——三個檔案加起來，不是每個檔案。
超過回 `413`。

實際參考：

| 資料 | 大約大小 |
|---|---|
| 24 小時、30 秒取樣、多星系 | 20–50 MB |
| 24 小時、1 Hz | 數百 MB |

單次解算時間上限預設 **900 秒**。兩者都可以請維運人員調整，實際值打
`/capabilities` 就知道。

檔案太大時，先想想是不是可以：
- 用 `teqc` 或 `gfzrnx` 抽出需要的時段
- 降取樣（30 秒對靜態監測通常足夠）
- 分段送，之後再合併

---

## 程式範例

### Python

```python
import requests

with open("rover.obs", "rb") as rover, \
     open("base.obs", "rb") as base, \
     open("brdc.nav", "rb") as nav:
    r = requests.post(
        "http://rtklib:8000/solve",
        files={"rover": rover, "base": base, "nav": nav},
        data={"preset": "static"},
        timeout=1200,          # 要大於服務端的解算上限
    )

if r.status_code != 200:
    raise SystemExit(f"solve failed [{r.status_code}]: {r.json().get('detail')}")

result = r.json()
print(f"{result['epochs']} epochs, {result['quality']}")

with open("result.pos", "w") as f:
    f.write(result["pos"])
```

### Shell（含錯誤處理）

```bash
#!/bin/sh
set -eu
resp=$(mktemp)
code=$(curl -s -o "$resp" -w "%{http_code}" \
    -X POST http://rtklib:8000/solve \
    -F "rover=@$1" -F "base=@$2" -F "nav=@$3" -F "preset=static")

if [ "$code" != "200" ]; then
    echo "solve failed [$code]" >&2
    python3 -c "import json,sys;print(json.load(open('$resp')).get('detail'),file=sys.stderr)"
    rm -f "$resp"; exit 1
fi
python3 -c "import json;print(json.load(open('$resp'))['pos'])"
rm -f "$resp"
```

---

## 常見狀況

**全部都是 `single`** — 沒給 `base`，或 preset 用了 `single`。要公分級就兩者都要
補上。

**只有 `float` 沒有 `fix`** — 模糊度解不開。常見原因：觀測時間太短（靜態至少
15–30 分鐘）、基線太長（超過 20 km 建議改 `iflc`）、資料只有單頻但 preset 設了
雙頻、或環境多路徑嚴重。

**epoch 數比預期少** — 仰角遮罩（預設 15°）濾掉了低仰角衛星，或部分時段共同衛星
不足。`/presets/{name}` 可以看到實際設定。

**想調參數但 preset 都不合用** — preset 是服務端的檔案，請維運人員新增；`.conf`
就是標準的 RTKLIB 設定格式。

---

## 回報問題時請附上

- **`X-Job-Id` 回應標頭**——成功或失敗都有，維運可以用它對到伺服器日誌
- HTTP 狀態碼與 `detail` 內容
- `/capabilities` 的輸出
- 用了哪個 preset、有沒有給 base

取得 job id：

```bash
curl -s -D - -o /dev/null -X POST http://rtklib:8000/solve \
     -F "rover=@rover.obs" -F "nav=@brdc.nav" | grep -i x-job-id
```

成功的回應另外也會在 JSON 的 `job` 欄位帶同一組值。

有這些通常一次就能定位。
