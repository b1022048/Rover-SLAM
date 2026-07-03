# Rover-SLAM 變數與術語表

> 用法（給每個 session）：回答「這變數/術語是什麼」之前先查這裡；答完新術語就追記一條。
> 格式：`## 術語` ＋ 一段定義 ＋ 原始碼位置。按字母/筆畫排序不強求，加在對應分類底下即可。
> 行號會隨程式碼變動——引用時以「檔案＋可 grep 的符號」為主，行號僅供參考。

## 立體視覺（ORB-SLAM3 標準）

### mbf
基線長度 × 焦距（baseline * fx），單位是「公尺×像素」。用途：深度 z = mbf / 視差。
來源有兩條路徑：
- **新路徑（本 repo 的 EuRoC.yaml 走這條）**：yaml 給 `Stereo.T_c1_c2`（左右相機外參），
  `Settings.cc`（`readCamera2` 內，約 394-400 行）取平移向量的 norm 當基線 `b_`，
  再算 `bf_ = b_ * fx`；需要立體校正時改用校正後焦距 `bf_ = b_ * P1(0,0)`（約 583 行）。
  若相機是 Rectified 型，yaml 直接給 `Stereo.b`（基線，公尺）。
- **舊路徑（僅舊格式 yaml）**：`Tracking.cc` 直接讀 `Camera.bf`（約 4881 行）。本 repo 的 EuRoC.yaml
  沒有這個鍵。

### mb
基線長度（公尺）。`mb = mbf / fx`（見 `src/Frame.cc`）。Frame 拿到的是 mbf（見上），
所以在 Frame 層要用基線本身時得除回 fx。

### mvuRight
對每個左影像特徵點，記錄其在（校正後）右影像中匹配點的 u 座標；無匹配為 -1。
由 `Frame::ComputeStereoMatches()`（`src/Frame.cc`）填入。單目點也是 -1。

### SAD（Sum of Absolute Differences）
滑動視窗絕對差和，立體匹配時比較左右 patch 相似度的代價函數；越小越像。
用於 `ComputeStereoMatches()` 的精匹配與亞像素插值階段。

## SuperPoint / 本專案特有

### lastmatch / lastmatchnum / lastmatchtrack
自適應閾值回饋鏈：`LocalMapping.cc`（`mpTracker->mpExtractorLeft->lastmatchnum = matchmean`，約 951-952 行；950 行另把 matchmean 寫給 tracker 的 lastmatchtrack）
→ `SPextractor.cc`（`featureExtractor->lastmatch = lastmatchnum`，約 597 行）
→ `superpoint_onnx.cc` 的閾值公式（約 209 行）：
`threshold = mean - 0.6*sqrt(variance) - 0.02 / (1 + exp(-0.02*(lastmatch-270)))`。
意義：上一輪匹配數越多，閾值越高（抽的點越少），反之降低閾值多抽點。初始值 0。
開關：`superpoint_onnx.cc` 中 `bool adaptivethresold`（約 192 行，寫死在程式碼裡，改了要重編譯）。

### adaptivethresold（原作者拼字，非 typo 修正對象）
上述自適應閾值機制的開關，2026-07-01 使用者打開試驗中。
