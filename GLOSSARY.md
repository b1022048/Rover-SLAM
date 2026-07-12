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
在 `IntegrateNewMeasurement` 累加更新（`src/ImuTypes.cc:285`：
`avgA = (dT*avgA + dR*acc*dt)/(dT+dt)`）。
公式＝時間加權平均的遞推寫法（分子 `dT*avgA`＝把舊平均還原成舊總和；此刻 dT 尚未 += dt，恰為舊總時間）。
乘 `dR`（此刻＝ΔR_{i,j-1}，尚未更新）是把各筆讀值翻到共同座標系（段初機體系）再平均——
向量相加必須同座標系；不乘的話「靜止原地旋轉」轉一圈讀值正負抵消、平均趨零，會騙過初始化的激勵檢查。
對照：avgW（:272）沒乘 dR，各時刻機體系的 ω 直接平均（粗略統計，用途只是初始化把關）。

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

### Nga / NgaWalk（與其來源 Calib::Cov / Calib::CovWalk）
IMU 預積分用的雜訊協方差矩陣，型別 `Eigen::DiagonalMatrix<float,6>`（宣告 `include/ImuTypes.h:214`）。
- `Nga`（**N**oise-**g**yro-**a**cc）：IMU **量測白雜訊**協方差，對角線 `[ng², ng², ng², na², na², na²]`（前三陀螺、後三加速度計）。
- `NgaWalk`：IMU **bias 隨機游走（random walk）**協方差，對角線 `[ngw², ngw², ngw², naw², naw², naw²]`。

來源鏈：EuRoC.yaml 的四個雜訊參數 `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk`（給的是**標準差 σ**）
→ `Calib::Set()`（`src/ImuTypes.cc:565-580`）**平方**成變異數存進 `Cov`/`CovWalk`（`Cov.diagonal() << ng2,...`，協方差 = σ²）
→ `Preintegrated` 建構子（`src/ImuTypes.cc:161-162`）`Nga = calib.Cov; NgaWalk = calib.CovWalk;` 拷進預積分物件。

用途：預積分除了算均值 `dR/dV/dP`，還要傳遞不確定度（協方差 `C`／資訊矩陣 `Info`）。
每次 `IntegrateNewMeasurement` 把 `Nga` 疊進協方差傳遞（誤差隨積分步數累積）；`NgaWalk` 當 bias 在關鍵幀間漂移的先驗。
累積出的協方差最終成為**後端優化中這條 IMU 約束的權重**（雜訊越大權重越低）。

### IMU bias 生命週期（誕生 → 首次估計 → 傳遞 → 迭代）
bias 沒有離線標定值，完全線上估計，流程四階段：
1. **誕生（全零）**：`Bias()` 預設建構六分量全 0（`include/ImuTypes.h:73`）；系統啟動的第一個預積分器用全零 bias（`Tracking.cc:667`），追蹤丟失重啟同樣歸零（`Tracking.cc:1484/2761/2909/3191`）。
2. **首次估計**：`LocalMapping::InitializeIMU`（`LocalMapping.cc:1828`）→ `Optimizer::InertialOptimization`：
   拿純視覺軌跡當基準，把重力方向/尺度/速度/**整段共用的一組 bias 頂點**（`Optimizer.cc:3738-3745`）一起優化；
   bias 另有拉向 0 的先驗邊 `EdgePriorGyro/Acc`（`Optimizer.cc:3759-3768`，權重 priorG=1e2/priorA=1e10）。
   優化後逐 KF `SetNewBias`，新舊差 >0.01 加做 `Reintegrate()`（`Optimizer.cc:3871-3877`）。
3. **日常傳遞（繼承制）**：幀對幀預積分用上一幀 bias 當線性化點（`Tracking.cc:1849`）；
   `PredictStateIMU` 地圖剛更新用上一 KF 的 bias、否則用上一幀的並直接繼承（`Tracking.cc:1960-1990`）；
   建新 KF 時寫入 KF 並起算下一段預積分（`Tracking.cc:3956/3973`）；
   優化後的新 bias 經 `UpdateFrameIMU` 的 `mLastBias` 中繼推回前後幀（`Tracking.cc:4930-4935`）。
4. **迭代更新**：bias 是 g2o 頂點（`VertexGyroBias`/`VertexAccBias`），在每幀 `PoseInertialOptimization*`
   （`Optimizer.cc:451-459`，回寫 864-868）、每 KF 的 `LocalInertialBA`（窗口內每 KF 各一組頂點，
   `Optimizer.cc:2375-2420`，回寫 2769-2773）、及 `FullInertialBA` 中反覆重估。
   約束來源：`EdgeInertial`（視覺-慣性殘差，靠 GetDelta* 感知 bias）＋ `EdgeGyroRW`/`EdgeAccRW`
   （random walk 邊，誤差＝相鄰 KF bias 之差，`G2oTypes.h:746-751`；資訊矩陣取自預積分協方差 `C` 的
   (9,9)/(12,12) 區塊——即 [[Nga / NgaWalk]] 條目中 `NgaWalk` 累加進 `C` 的部分，yaml 的
   GyroWalk/AccWalk 在此閉環：它決定優化器允許 bias 漂多快）。

### b / bu / db / JRg,JVg,JVa,JPg,JPa（預積分的 bias 一階修正機制）
預積分 `dR/dV/dP` 是在**線性化點 `b`**（舊 bias）下積出來的（積分時扣 `b`，`ImuTypes.cc:280-282`）。
優化器改了 bias 後不必重積幾百筆 IMU，用一階近似跟上（Forster 預積分論文做法）：
- `JRg/JVg/JVa/JPg/JPa`：dR/dV/dP 對 bias 的敏感度（Jacobian），在 `IntegrateNewMeasurement`
  裡遞推累積（`ImuTypes.cc:306-309, 333`）。
- `bu`＝最新 bias，`db = bu - b`：由 `SetNewBias` 更新（`ImuTypes.cc:373-384`）。
- `GetDeltaRotation/Velocity/Position(b_new)`（`ImuTypes.cc:402-441`）：
  `dR·exp(JRg·dbg)`、`dV + JVg·dbg + JVa·dba`、`dP + JPg·dbg + JPa·dba`——泰勒一階修正，
  幾次矩陣乘法就得到新 bias 下的積分值；`EdgeInertial::computeError`（`G2oTypes.cc:594-613`）就靠這三個函式算殘差。
- 一階近似失效的保險：bias 改動 >0.01 觸發 `Reintegrate()`（`ImuTypes.cc:231-238`）——
  用快取的原始量測 `mvMeasurements` 以新 bias 全部重積，並重設線性化點 `b = bu`。
  這也是預積分物件要保留原始 IMU 讀數的原因。

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

## C++ 語言與並行（多 thread）機制

### std::mutex（互斥鎖本體）
保護共享資料的鎖，保證同一時間只有一個 thread 能進入被保護的區段。本 repo 例：
`mMutexImuQueue`（宣告於 `include/Tracking.h:247`）保護 IMU 佇列 `mlQueueImuData`——
IMU 資料由感測器 thread 塞入、tracking thread 取出，兩條 thread 同時碰同一 queue 會出錯，故上鎖。

### std::unique_lock（RAII 鎖包裝）／RAII
`unique_lock<mutex> lock(m)`：**建構時自動對 m 上鎖，解構時（變數離開最近的一對 `{ }` 作用域）自動解鎖**。
這就是 RAII（Resource Acquisition Is Initialization，資源取得即初始化）：資源的生命週期綁在物件生命週期上，
不用手動 unlock，即使中途 `break`/`return`/丟例外，解構子照跑、鎖一定會被放掉（裸 `mutex.lock()/unlock()` 中途跳出容易忘記解鎖而死鎖）。
**鎖的作用域 = 宣告它的那對 `{ }`，不是某個 if/else 的 `}`。**
例：`Tracking.cc:1806` 的 `lock` 宣告在 1805 開的匿名區塊裡，一直活到 **1835** 的 `}` 才解鎖
（不是 `else` 的 `}`＝1834，差一行）。

### 縮小臨界區（critical section）／匿名 `{ }` 區塊
臨界區＝被鎖保護、同時只允許一個 thread 進入的程式碼範圍。用一對額外的匿名 `{ }` 把
`unique_lock` 框住，可精準控制解鎖時機、把臨界區壓到最小。
例：`Tracking.cc:1805-1835` 特意多包一層 `{ }`，讓「碰 queue」一做完就解鎖，
避免在 1837 的 `usleep(500)` 睡覺時還握著鎖，擋住產生 IMU 的那條 thread 塞資料。

### mbImuPreintegrated / setIntegrated()（「處理完畢」旗標，跨 thread 同步用）
`Frame.h:332` 宣告，`Frame.cc:1514-1517` 的 `setIntegrated()` 上鎖後設為 true，
`Frame.cc:1505-1508` 的 `imuIsPreintegrated()` 讀取。
語義是「這一幀的預積分**處理流程已結束**」，不是「預積分成功算出來了」——
所以 `PreintegrateIMU()` 的三個出口（無上一幀 `Tracking.cc:1787`、queue 空 `1797`、正常算完 `1919`）
都要呼叫它。為什麼不能直接 return：LocalMapping thread 在 IMU 初始化時會呼叫
`Tracking::UpdateFrameIMU()`（`Tracking.cc:4900`），裡面 `Tracking.cc:4937-4941` 用
`while(!mCurrentFrame.imuIsPreintegrated()) usleep(500);` 空轉等待 tracking thread 把當前幀
預積分做完。若提前 return 不標記，該幀旗標永遠是 false，LocalMapping 會卡在這個迴圈裡不出來。

### mlQueueImuData vs mvImuFromLastFrame（IMU 資料的兩層容器）
- `mlQueueImuData`（list，`Tracking.h` 宣告，受 `mMutexImuQueue` 保護）：感測器端 `GrabImuData()`
  （`Tracking.cc:1771-1775`）推進來的**所有還沒被消化的** IMU 資料。時間範圍不是「上一幀到當前幀」，
  而是「大約上一幀時間戳（上一輪留下的跨幀那筆）～ 最新收到的一筆（通常已超過當前幀，
  因為 IMU 200Hz 跑在影像處理前面）」。
- `mvImuFromLastFrame`（vector，每幀清空重建）：`PreintegrateIMU()` 的篩選迴圈
  （`Tracking.cc:1801-1838`）從 queue 抽出的「上一幀～當前幀」區間，才是真正拿去積分的資料。
  篩選規則（容忍 mImuPer=1ms）：太早的丟棄；區間內的複製後 pop；
  第一筆 ≥ 當前幀時間的**複製但不 pop**（`1826` 只 push 沒 pop）→ 留在 queue 裡當下一幀的起點。
  補充：跨幀那筆沒有「超過多久」的上限（`1823` else 分支照收），但積分時它只當線性內插的端點，
  `tstep` 截在當前幀時間（`Tracking.cc:1897`），積分總時長恆為上一幀→當前幀，不會多算。
  另注意 `1843-1846` 的 `n==0` 提前 return 是唯一**沒有** setIntegrated 的出口（原版 ORB-SLAM3 亦然）。

### tini / tab / tend（預積分首末段的時間補償變數，PreintegrateIMU 迴圈內）
積分段的邊界永遠對齊影像幀時刻；邊界上沒有 IMU 實測值，就用相鄰兩筆線性內插補出來。
- `tab`＝相鄰兩筆 IMU 的間隔（`Tracking.cc:1867、1891`），內插的分母。
- `tini`＝IMU0 離**上一幀時刻**（首段的前端邊界）多遠（`1869`，可正可負）；
  首段用 `a0-(a1-a0)*(tini/tab)` 反推上一幀時刻的值（`1874-1875`），tstep 從上一幀起算（`1879`）。
- `tend`＝末筆 IMU 超出**當前幀時刻**（末段的後端邊界）多少（`1892`）；
  末段內插出當前幀時刻的值（`1893-1896`），tstep 截在當前幀（`1897`）。
中間段兩端剛好都是 IMU 實測值，直接平均（`1881-1887`）。

### IMU::Preintegrated 狀態量（dR/dV/dP、C、J 系列，ImuTypes.h:212-220）
一個物件＝「幀 i → 幀 j 的相對運動包裹」，與起點絕對位姿/速度/重力無關（重力在使用端如
PredictStateIMU 的 Gz 項才加回）。`IntegrateNewMeasurement()`（ImuTypes.cc:247-338）每小段累積一次：
- `dR/dV/dP`：起點座標系下的相對旋轉/速度變化/位移；更新順序固定 P→V→R，
  因為每行只能用「該小段開始時」的狀態，被依賴者最晚更新（252-254 行註解）。
- `C`（15×15）：前 9 維 [δθ,δv,δp] 協方差，經 `C=A·C·Aᵀ+B·Nga·Bᵀ` 傳遞（311 行）；
  後 6 維 bias，每步 `+= NgaWalk`（隨機游走，313 行）。求逆後當優化的資訊矩陣（權重）。
- `JRg/JVg/JVa/JPg/JPa`：dR/dV/dP 對 bias 的雅可比，優化微調 bias 時做一階修正、免重積分；
  bias 大改才用 `Reintegrate()`（231-238 行）拿 `mvMeasurements` 原始資料全部重積。
- `avgA/avgW`：加權平均加/角速度；avgA 用在 Tracking.cc:2787 檢查初始化前加速度激勵是否足夠。
- `NormalizeRotation`（301 行）：連乘浮點誤差會讓 dR 漂離正交，定期拉回合法旋轉矩陣。

### 變數命名前綴（ORB-SLAM 匈牙利式命名，全 repo 通用）
開頭小寫字母是型別/身分縮寫，不是單字：`m`=成員變數、`v`=vector、`l`=list、`p`=指標、
`b`=bool、`n`=整數，可疊加。例：`mvMeasurements`（成員+vector，ImuTypes.h:248）、
`mlQueueImuData`（成員+list）、`mpPrevFrame`（成員+指標）、`mbImuPreintegrated`（成員+bool）、
`mnId`（成員+整數）、`mvpMapPoints`（成員+vector+指標）。無 `m` 開頭者多為區域變數或參數。

### IMU::Bias / bias（零偏、零飄）與扣除位置
誤差模型：讀值＝真值＋bias＋白噪聲。bias 是慢變的系統性偏移，不扣會被積分放大
（速度誤差∝t、位置∝t²），所以是待估計量，優化器會更新（配合 J 系列一階修正／Reintegrate）。
`IMU::Bias`（ImuTypes.h:62-86）＝六個 float：`bax/bay/baz`（加速度計）、`bwx/bwy/bwz`（陀螺儀，w=ω）。
扣除位置：加速度在 `IntegrateNewMeasurement` 的 `acc`（ImuTypes.cc:281）；
陀螺儀實際在 `IntegratedRotation` 建構子內（ImuTypes.cc:128-130），
282 行的 `accW`（名字誤導，是角速度）只供 avgW 統計用。
`acc << x,y,z` 的 `<<` 是 Eigen comma initializer，非位移運算。
補充（Preintegrated 內的三兄弟，ImuTypes.h:217-229）：`b`＝積分時假設的 bias（一階修正的基準點）、
`bu`＝優化後更新的 bias（SetNewBias 寫入）、`db`＝bu−b 的 6 維小量；
db 太大時 `Reintegrate()` 以 bu 為新基準重積（ImuTypes.cc:235 `Initialize(bu)`）。

### SearchByBoWSP（命名陷阱：名字有 BoW，內部是 LightGlue）
`SPmatcher::SearchByBoWSP`（SPmatcher.cc:1524 KF-KF 版、:1670 KF-Frame 版）**不用 BoW 分桶**——
把兩幀全部特徵點＋描述子經 `MatchingPoints_onnx`（:374）→ `featureMatcher->Matcher_Inference`
送 LightGlue ONNX（onnxmodel/lightglue_sim.onnx）全對全匹配；名字只是沿用舊介面方便 drop-in 替換。
呼叫點：TrackReferenceKeyFrame（Tracking.cc:3294）、Relocalization（:4530）、回環驗證（LoopClosing.cc:1279）。
BoW 在本 fork 的現役職責只剩**候選幀檢索**：ComputeBoW3（Frame.cc:1046，SP 詞典 mBow3Vec）→
KeyFrameDatabase 倒排索引（KeyFrameDatabase.cc:49-71）→ DetectRelocalizationCandidates（Tracking.cc:4492）
／DetectNBestCandidates_sp（LoopClosing.cc:611）。幀對幀的 FeatureVector 分桶匹配已退役。
舊制 mBowVec 管線（ORB 詞典）在本 fork **整組斷電**：ComputeBoW() 呼叫點全註解（Tracking.cc:4487、
LocalMapping.cc:405，mBowVec 恆為空）；吃 mBowVec 的舊檢索函數零呼叫；DetectCommonRegionsFromBoW
（內含真 BoW 分桶的 SearchByBoW，LoopClosing.cc:867）呼叫點 :632/:642 已註解，現役為 _sp 版（:633/:643）。
另：Tracking.cc:3284 的 ComputeBoW3 是化石——SearchByBoWSP 不用 BoW、KF 建構子不拷 mBow3Vec（KeyFrame.cc:52
只拷舊制向量），該次計算無消費者。

### integrable / mvMeasurements（預積分的「原料倉」）
`Preintegrated::integrable`（ImuTypes.h:231-246）＝純資料 struct：`a`（加速度計讀值）、`w`（陀螺讀值，皆未扣 bias）、`t`（該筆 dt）。
建構子只做成員初始化列表打包，無運算。`IntegrateNewMeasurement` 開頭（ImuTypes.cc:250）每筆先存進
`mvMeasurements` 再積分——dR/dV/dP 是加工品、這裡是原料：bias 大改時 `Reintegrate()`（ImuTypes.cc:236）、
刪 KF 併段時 `MergePrevious()`（ImuTypes.cc:359）都靠它逐筆回放重積。`serialize` 供 boost 存地圖用。

### 軸角向量（axis-angle）φ = ω·dt
歐拉旋轉定理：任何 3D 旋轉＝繞某軸 u（單位向量）轉某角 θ。軸角向量把兩者打包：`φ = θ·u`
——長度 `|φ|`＝角度、方向 `φ/|φ|`＝軸（u 長度恆為 1，「長度欄位」空著剛好存 θ）；右手定則定轉向。
ω 用同一打包（方向＝瞬時轉軸、長度＝rad/s），dt 內視 ω 不變 → 這段的旋轉軸角＝`ω·dt`。
對應 `IntegratedRotation` 建構子：ImuTypes.cc:128-130 的 x,y,z＝φ 分量、133-134 的 `d`＝解包出的 θ。
陷阱：兩個軸角**不可相加**合成旋轉（除非同軸），要各自 Exp 成矩陣後相乘（`dR·Exp(φ)`）；小角度才近似可加。

### 預積分的數學骨架（IntegrateNewMeasurement 內，ImuTypes.cc:247-338）
三件工具推出全部公式：①`hat(a)b=a×b`（交換帶負號 `hat(a)b=−hat(b)a`）
——hat 把 v=(x,y,z) 排成反對稱矩陣 `[[0,−z,y],[z,0,−x],[−y,x,0]]`（Wᵀ=−W），
目的：叉積矩陣化才能自乘（Rodrigues 的 W²）、才能把 δφ 提出來求 Jacobian（Wacc 的由來，ImuTypes.cc:295）。
名詞注意：叉積＝台灣教材的「外積」a×b（輸出向量）；但英文 outer product 是 abᵀ（輸出矩陣），內積 aᵀb 輸出純量，三者勿混。
單位軸 u 有 `hat(u)²=uuᵀ−I`，故 Rodrigues 的 W² 版（程式碼用）與 uuᵀ 版（部分文獻用）等價；
②Exp/Rodrigues（149 行）把軸角變旋轉矩陣，右雅可比 `Jr`＝「軸角加法擾動→群上乘法擾動」的匯率
（`Exp(θ+δ)≈Exp(θ)·Exp(Jr·δ)`）；③一階泰勒 `Exp(δφ)≈I+hat(δφ)`、移位 `R·Exp(φ)·Rᵀ=Exp(Rφ)`。
- 本體＝運動學＋座標翻譯：`dP+=dV·dt+½dR·acc·dt²`、`dV+=dR·acc·dt`、`dR←dR·Exp((ω−bg)dt)`（右乘＝機體系接旋轉）。無重力，使用端才補 g。
- 誤差態 η=[δφ,δv,δp]：`η←A·η+B·n`，A(0,0)=ΔRᵀ（誤差換座標）、A(3,0)=−dR·dt·Wacc（姿態誤差×加速度滲入速度）、
  A(6,3)=dt·I；B(0,0)=Jr·dt、B(3,3)=dR·dt。協方差 `C←A·C·Aᵀ+B·Nga·Bᵀ`（線性系統的方差傳遞），bias 塊每步 +NgaWalk（隨機游走）。
- J 系列＝∂(dR,dV,dP)/∂bias 的遞推（一階修正免重積分）：`JVa−=dR·dt`、`JVg−=dR·dt·Wacc·JRg`（鏈式經 dR）、
  `JRg←ΔRᵀ·JRg−Jr·dt`。所有遞推右邊只用「段初」值 → 更新順序 dP→dV、JP→JV→（dR）→JRg 不可換。
文獻：Forster《On-Manifold Preintegration》eq.35-38（本體）、eq.62-63（協方差）；ORB_SLAM3 issue #212（變形）。
