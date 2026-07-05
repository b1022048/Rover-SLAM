# Rover-SLAM 變數與術語表

> 用法（給每個 session）：回答「這變數/術語是什麼」之前先查這裡；答完新術語就追記一條。
> 格式：`## 術語` ＋ 一段定義 ＋ 原始碼位置。按字母/筆畫排序不強求，加在對應分類底下即可。
> 行號會隨程式碼變動——引用時以「檔案＋可 grep 的符號」為主，行號僅供參考。

## 立體視覺（ORB-SLAM3 標準）

### mbf
基線長度 × 焦距（baseline * fx），單位是「公尺×像素」。用途：深度 z = mbf / 視差。
完整鏈路（本 repo EuRoC.yaml，Pinhole + IMU_STEREO 走的路徑）：
1. `Settings::readCamera2()`（`Settings.cc:392-400`）：yaml 給 `Stereo.T_c1_c2`（左右相機外參），
   取平移向量 norm 當基線 `b_ = Tlr_.translation().norm()`，先算一次
   `bf_ = b_ * calibration1_->getParameter(0)`（此時 fx 還是「原始未校正」的焦距）。
2. `Settings::precomputeRectificationMaps()`（`Settings.cc:583`）：做完 `cv::stereoRectify` 後，
   用**校正後**的等效焦距覆寫：`bf_ = b_ * P1.at<double>(0,0)`——這個函式在建構時排在
   `readCamera2` 之後執行，所以**這才是最終真正用到的 `bf_`**。
3. `Tracking.cc:604`：`mbf = settings->bf();`（`Settings::bf()` getter，`Settings.h:75`，只回傳 `bf_`）。
4. `Frame` 建構子把 `mbf` 當參數 `bf` 傳入，初始化列表 `mbf(bf)`（如 `Frame.cc:107`）。
若相機是 `Rectified` 型，yaml 直接給 `Stereo.b`（見 `readCamera2` 的 if 分支，`Settings.cc:388-390`），
不用取 `Tlr_` 的 norm。舊格式 yaml 路徑：`Tracking.cc:4881` 直接讀 `Camera.bf`，本 repo 的
EuRoC.yaml 沒有這個鍵，不會走到。

### mb
基線長度（公尺）。`mb = mbf / fx`（`src/Frame.cc:214`，另 330/458/1587 行同），
Frame 拿到的是 mbf（見上），所以在 Frame 層要用基線本身時得除回 fx。

**這裡的 `fx` 確認是矯正（rectify）後的焦距，不是 yaml 原始 `Camera1.fx`**：
`Frame::fx`（靜態成員，`Frame.cc:201` 等處 `fx = K.at<float>(0,0)`）← `Tracking::mK`
（`Tracking.cc:580`：`mK.at<float>(0,0) = mpCamera->getParameter(0)`）← `mpCamera =
settings->camera1()` 也就是 `calibration1_`。而 `calibration1_` 的參數 0 早在 `Settings`
建構時就被 `precomputeRectificationMaps()`（`Settings.cc:577`：
`calibration1_->setParameter(P1.at<double>(0,0), 0)`）覆寫成校正後的值，這一步在
`Settings::Settings()` 建構子裡（`Settings.cc:205-208`，`if(bNeedToRectify_)`）比 `Tracking`
或任何 `Frame` 都早執行。EuRoC.yaml 是 `PinHole` 型，`readCamera2()` 會把
`bNeedToRectify_` 設 true（`Settings.cc:327`），所以一定會跑到這段覆寫。
`mbf` 最終值也是用同一個 `P1.at<double>(0,0)`（`Settings.cc:583`）算的，兩邊焦距一致，
`mb = mbf/fx` 才能把焦距完全消掉、乾淨還原成純基線；如果兩邊焦距不一致（例如一個用
校正前一個用校正後），這個除法會算出錯的基線值。

### mvuRight
對每個左影像特徵點，記錄其在（校正後）右影像中匹配點的 u 座標；無匹配為 -1。
由 `Frame::ComputeStereoMatches()`（`src/Frame.cc`）填入。單目點也是 -1。

### SAD（Sum of Absolute Differences）
滑動視窗絕對差和，立體匹配時比較左右 patch 相似度的代價函數；越小越像。
用於 `ComputeStereoMatches()` 的精匹配與亞像素插值階段。

### Frame::N
左圖（僅左圖）目前抽到的特徵點數量，`int N;` 宣告於 `include/Frame.h`。
來源鏈（雙目建構子，`Frame::Frame(imLeft, imRight, ...)`，`src/Frame.cc:106` 起）：
`Frame.cc:156 N = mvKeys.size()` ← `mvKeys` 由 `ExtractKeyPoints`（`Frame.cc:546`）內
`(*mpExtractorLeft)(im, mvKeys, mDescriptors)` 呼叫填入 ← 也就是 `SPextractor::operator()`
（`src/Extractors/SPextractor.cc:516`）。用途：`Tracking::StereoInitialization()`
（`Tracking.cc:2742`）要求 `mCurrentFrame.N>500` 才建圖初始化。
受 `EuRoC.yaml` 的 `ORBextractor.nFeatures`（上限）與目前金字塔失效只跑單層抽取
（`nLevels: 1`，見金字塔相關落差分析）影響，特徵點數變少時這道門檻更容易卡住。

## 慣性導航（IMU）初始化

### avgA
`IMU::Preintegrated` 的成員（`include/ImuTypes.h:221`），該次預積分區間內加速度計讀數的加權平均，
在 `IntegrateNewMeasurement` 累加更新（`src/ImuTypes.cc:271`：
`avgA = (dT*avgA + dR*acc*dt)/(dT+dt)`）。

### mpImuPreintegratedFrame vs mpImuPreintegrated（兩個不同的累積器，容易搞混）
- `mpImuPreintegratedFrame`：**從上一幀到當前幀**這一小段時間的 IMU 預積分（`Tracking.cc:1915` 賦值）。
- `mpImuPreintegrated`：**從上一個關鍵幀累積到現在**的 IMU 預積分（`mpImuPreintegratedFromLastKF` 是它的來源，見下）。

### 「加速度激勵不足」初始化門檻
`Tracking::StereoInitialization()`（`Tracking.cc:2744-2763`，只在 IMU_STEREO/IMU_RGBD 模式跑）
在真正建第一個關鍵幀前，依序檢查：
1. `mCurrentFrame.mpImuPreintegrated` / `mLastFrame.mpImuPreintegrated` 是否已存在（IMU 資料流是否已到位）。
2. 除非 `mFastInit`（yaml 的 `IMU.fastInit`，`Tracking.h:175`）開啟，否則要求
   `(mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() >= 0.5`：
   載體幾乎靜止/等速時，加速度計無法可靠分辨重力方向與真實運動，初始化易解壞，故中止等下一幀。
3. 通過後，重置 `mpImuPreintegratedFromLastKF`（歸零偏置 `IMU::Bias()` 重新建）並掛回
   `mCurrentFrame.mpImuPreintegrated`，作為即將誕生的第一個關鍵幀之後的累積起點。
這只是初始化前的把關，跟 `LocalMapping::InitializeIMU` 正式的重力/尺度/偏置優化是兩回事。

### mTbc / mTcb（IMU::Calib 的相機↔IMU 外參）
來源鏈：yaml `IMU.T_b_c1`（`Examples/Stereo-Inertial/EuRoC.yaml:67`，相機→IMU/body 剛體變換）
→ `Settings::readIMU()` 讀成 `Tbc_`（`Settings.cc:479-480`）
→ 若為 IMU_STEREO，套用立體校正修正 `Tbc_ = Tbc_ * T_r1_u1.inverse()`（`Settings.cc:591`，
  因為校正後左相機座標系旋轉了 R_r1_u1，外參要跟著校正後的影像對齊）
→ `Tracking.cc:655,665`：`mpImuCalib = new IMU::Calib(settings->Tbc(), ...)`
→ `Calib::Set`（`ImuTypes.cc:574-575`）：`mTbc = sophTbc; mTcb = mTbc.inverse();`（兩者互為反變換）
→ `Frame` 建構時用拷貝建構子整份複製到 `mCurrentFrame.mImuCalib`。
即：`mTcb`/`mTbc` 是常數（來自 yaml），不是估計值。

**命名陷阱**：`StereoInitialization()`（`Tracking.cc:2769-2770`）裡把 `mImuCalib.mTcb` 的
旋轉/平移存進叫做 `Rwb0`/`twb0`（world→body）的變數，但算的其實是 `mTcb`（camera→body）。
不是筆誤——只有在初始化這一刻，world 座標系被定義成跟第一個相機幀重合（單位旋轉、零平移，
見 `Tracking.cc:2776` 的 `SetPose(Sophus::SE3f())` 分支），所以 t=0 時 camera→body 數值上
剛好等於 world→body；之後幀就不再有這個等價關係。

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
