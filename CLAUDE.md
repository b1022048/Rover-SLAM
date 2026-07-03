# Rover-SLAM 專案指南

ORB-SLAM3 的 fork，把 ORB 特徵抽取換成 SuperPoint（ONNX Runtime GPU 推論），
研究用途：使用者正在逐行理解程式碼、跑 EuRoC 實驗、用 evo 評估軌跡誤差。

## 鐵律（違反前必須先問使用者）

1. **預設不修改任何 src/、include/、Examples/ 下的程式碼。** 使用者明確要求過「只讀」。
   例外：使用者在**當次訊息**明確要求修改某處，才可以改，且只改被點名的地方。
   （在程式碼中新增中文註解也算修改——同樣需要當次明確要求。）
2. 可以自由新增/更新 `.md` 文件（本檔、GLOSSARY.md、根目錄的說明檔）。
3. 不要手動編輯或刪除 `build/`、`lib/`、`Vocabulary/`、`onnxmodel/` 的內容
   （經 `make` 編譯自然產生的 build/、lib/ 變動不在此限）。

## 建置與執行

```bash
source env_rover.sh            # 必須先做：OpenCV 3.4.13、onnxruntime-gpu-1.16.3、CUDA 11.8 路徑、CUDA_VISIBLE_DEVICES=2
cd build && make -j12          # 增量編譯（改碼後驗證用這個）
./build.sh                     # 只有 Thirdparty 或 CMakeLists 變動時才需要全量重建
```

- EuRoC 資料集：`/media/lab405/Windows/data/Euroc/`（外接 Windows 分割區，**沒掛載時路徑會消失**，
  找不到先檢查掛載）。序列目錄 `MH_01_easy` … `V2_03_difficult`，各含 `mav0/`。
- 實跑範例（repo 根目錄，先 `source env_rover.sh`；短序列驗證建議 V1_01_easy）：
  ```bash
  ./Examples/Stereo-Inertial/stereo_inertial_euroc \
      Vocabulary/voc_binary_tartan_8u_6.yml.gz \
      Examples/Stereo-Inertial/EuRoC.yaml \
      /media/lab405/Windows/data/Euroc/V1_01_easy \
      Examples/Stereo-Inertial/EuRoC_TimeStamps/V101.txt \
      V101_si
  ```
  注意：TimeStamps 檔名沒有底線與難度字尾（`MH_01_easy` 對 `MH01.txt`、`V1_01_easy` 對 `V101.txt`）。
- 軌跡輸出：最後一個參數是輸出名（如上會得到 `f_V101_si.txt` 與 `kf_V101_si.txt`）；
  不給輸出名（只用 4 個參數）則輸出 repo 根目錄的 `CameraTrajectory.txt`、`KeyFrameTrajectory.txt`。
- evo 評估指令：見根目錄 `evo_使用指令.md`（檔名含中文，shell 中請加引號）。
- 舊版評估腳本：`evaluation/`（associate.py、evaluate_ate_scale.py）。

## 熱點檔案地圖（行數為 2026-07 快照，僅供判斷「這是大檔」）

| 檔案 | 行數 | 內容 |
|---|---|---|
| src/Tracking.cc | 5189 | 追蹤主流程（GrabImageStereo、Track、TrackStereo） |
| src/LocalMapping.cc | 2144 | 局部建圖；951-952 行把 matchmean 回寫給 extractor 的 lastmatchnum（950 行是寫給 tracker 的 lastmatchtrack） |
| src/Frame.cc | 1789 | Frame 建構、立體匹配（ComputeStereoMatches）、格子分配 |
| src/Extractors/SPextractor.cc | 713 | SuperPoint 抽取器外殼、金字塔、lastmatch 轉交 |
| src/Extractors/superpoint_onnx.cc | 291 | ONNX 推論、自適應閾值（192-209 行） |
| include/Settings.h | 235 | 設定解析 |

## 回答程式碼問題的規約（使用者最常見的用法）

1. 使用者貼一段碼提問 → **先用 Grep 在 repo 定位到實際 file:line**，
   再 Read 該處前後 ±40 行，以檔案內容為準作答，不要只憑貼文猜。
2. 大檔（>800 行，見上表）**只能用 offset/limit 讀片段**，禁止整檔 Read。
   需要跨多檔追資料流時，派 Explore subagent，主對話只收結論
   （規則見 ~/.claude/rules/model-dispatch.md）。
3. 回答時一律附 `file:line` 引用。
4. 答完一個「這變數是什麼」型的問題後，把定義**追記到 `GLOSSARY.md`**（先查該檔避免重複）。
   下次被問到先查 GLOSSARY.md，有就直接引用並附原始碼位置。
5. 使用者問「這是不是錯了/幫我確認」→ 這是判斷題，適用
   ~/.claude/rules/judgment.md 的證據等級規則：沒讀到實際碼行與資料形狀，不得下結論，
   更不得附和使用者的假設。

## 修改程式碼後的完成定義（若使用者要求改碼）

`source env_rover.sh && cd build && make -j12` 編譯通過，才可以說「完成」。
只改註解也要編譯（註解破壞編碼/字元的事故存在）。
實跑驗證用上面「建置與執行」的 V1_01_easy 指令；資料集路徑不存在（碟未掛載）或跑不了時，
明說「只驗證了編譯」。
