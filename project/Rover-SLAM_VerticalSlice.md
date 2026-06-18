# Rover-SLAM 垂直切片（Vertical Slice）分析報告

> 以「一張輸入影像 frame」為主線，追蹤它從進入程式到輸出 camera pose / trajectory / map update 的完整流程。
>
> 本專案是以 **ORB-SLAM3** 為骨架，把傳統 ORB 特徵前端換成 **深度學習前端（SuperPoint 抽特徵 + LightGlue 匹配，使用 ONNX Runtime 推論）**。
>
> 程式碼基準：`src/`、`include/`、`Examples/`。所有引用皆標註 `file:line`。

---

## 0. 名詞與符號

| 符號 | 意義 |
| ---- | ---- |
| `Tcw` | World → Camera 的位姿（`Sophus::SE3f`），即相機在世界座標系下的姿態 |
| KF | KeyFrame 關鍵幀 |
| MP | MapPoint 地圖點 |
| LBA | Local Bundle Adjustment 局部光束法平差 |
| SP | SuperPoint（深度特徵抽取器） |
| LG | LightGlue（深度特徵匹配器） |
| Atlas | 多地圖管理器（ORB-SLAM3 特性，可同時持有多張子地圖） |

---

## 1. 程式入口（Entry Point）

### 1.1 可執行檔

`CMakeLists.txt:172-207` 定義了多個範例執行檔；README 主要示範的是 **Monocular**：

| 執行檔 | 對應 main | 感測器型態 |
| ------ | --------- | ---------- |
| `mono_euroc` | `Examples/Monocular/mono_euroc.cc` | `System::MONOCULAR` |
| `mono_inertial_euroc` | `Examples/Monocular-Inertial/...` | `System::IMU_MONOCULAR` |
| `stereo_inertial_euroc` | `Examples/Stereo-Inertial/stereo_inertial_euroc.cc` | `System::IMU_STEREO` |

> 註：本程式碼庫的 Stereo 純雙目（非 inertial）入口在 `Examples` 中未單獨提供 `.cc`，但 `System::TrackStereo()`（`src/System.cc:283`）仍完整支援 `STEREO` 與 `IMU_STEREO`。

### 1.2 main() 做的事（以 `mono_euroc.cc` 為例）

```text
main()  (Examples/Monocular/mono_euroc.cc:33)
 ├─ 解析參數: argv[1]=Vocabulary, argv[2]=settings.yaml, argv[3]=影像資料夾, argv[4]=時間戳檔
 ├─ LoadImages()           (:206)  讀入影像路徑清單 + 時間戳
 ├─ 建立 SLAM 系統          (:83)   ORB_SLAM3::System SLAM(voc, settings, MONOCULAR, true)
 ├─ imageScale = SLAM.GetImageScale()  (:84)
 └─ for 每張影像 (:95):
       im = cv::imread(...)               (:99)   ← 一張 frame 從這裡進入程式
       (可選) cv::resize(im, ...)          (:120)
       SLAM.TrackMonocular(im, tframe)    (:140)  ← 送入 SLAM 系統
 ── 迴圈結束後 ──
 ├─ SLAM.Shutdown()                        (:187)
 ├─ SLAM.SaveTrajectoryEuRoC(f_file)       (:194/199)   ← 輸出完整相機軌跡
 └─ SLAM.SaveKeyFrameTrajectoryEuRoC(kf)   (:195/200)   ← 輸出關鍵幀軌跡
```

### 1.3 讀了哪些設定檔 / 模型

`System` 建構子 `src/System.cc:50-282`：

1. **settings.yaml**（`fsSettings`，`:74`）：相機內參、解析度、特徵數、IMU 標定等。新版檔（`File.version == "1.0"`）會走 `new Settings(...)`（`:86`），舊版走 `ParseCamParamFile` / `ParseORBParamFile` / `ParseIMUParamFile`。
2. **Vocabulary（詞袋）**：`mpVocabulary_sp = new SPVocabulary(); mpVocabulary_sp->load(strVocFile)`（`src/System.cc:131-132`）。這是 **SuperPoint 描述子的 DBoW 詞典**（取代了原本 ORB 的 `ORBVocabulary`）。
3. **ONNX 模型**：不是在 System 載入，而是在 **Tracking 建構特徵抽取器時** 載入（見 §4）：
   - `onnxmodel/superpoint.onnx`
   - `onnxmodel/lightglue_sim.onnx`

### 1.4 建立了哪些核心物件（System 建構子）

| 物件 | 建立位置 | 角色 |
| ---- | -------- | ---- |
| `Settings* settings_` | `System.cc:86` | 設定檔解析 |
| `SPVocabulary* mpVocabulary_sp` | `System.cc:131` | SuperPoint 詞袋 |
| `KeyFrameDatabase* mpKeyFrameDatabase` | `System.cc:149` | 關鍵幀資料庫（迴環/重定位用） |
| `Atlas* mpAtlas` | `System.cc:154` | 多地圖管理 |
| `FrameDrawer* mpFrameDrawer` | `System.cc:215` | 影像視覺化 |
| `MapDrawer* mpMapDrawer` | `System.cc:216` | 3D 地圖/軌跡視覺化 |
| `Tracking* mpTracker` | `System.cc:222` | **追蹤主線（跑在主執行緒）** |
| `LocalMapping* mpLocalMapper` | `System.cc:227` | 局部建圖 |
| `LoopClosing* mpLoopCloser` | `System.cc:250` | 迴環檢測 / 地圖融合 |
| `Viewer* mpViewer` | `System.cc:271` | 顯示（可選） |

### 1.5 啟動了哪些執行緒

| 執行緒 | 啟動位置 | 進入點 |
| ------ | -------- | ------ |
| **Tracking** | 不另開 thread，跑在 **main 主執行緒** | 每次 `TrackXXX()` 呼叫 |
| **LocalMapping** | `mptLocalMapping = new thread(&LocalMapping::Run, ...)` `System.cc:229` | `LocalMapping::Run()` |
| **LoopClosing** | `mptLoopClosing = new thread(&LoopClosing::Run, ...)` `System.cc:251` | `LoopClosing::Run()` |
| **Viewer** | `mptViewer = new thread(&Viewer::Run, ...)` `System.cc:272` | `Viewer::Run()` |

執行緒之間的指標互設於 `System.cc:255-262`（`SetLocalMapper / SetLoopClosing / SetTracker / SetLoopCloser`）。

### 1.6 影像是在哪裡被送進 SLAM

`main` 迴圈 → `SLAM.TrackMonocular(im, t)`（對應 Stereo 為 `SLAM.TrackStereo(...)` `src/System.cc:283`）。
`System::TrackXXX()` 內部會做：模式切換檢查 → reset 檢查 → IMU 資料灌入 → 呼叫 `mpTracker->GrabImageXXX()`。

---

## 2. 一張 frame 的完整生命週期（Vertical Slice 主線）

以 **Stereo** 為例（Monocular 流程同構，差別在初始化與深度）：

```text
Image input (cv::Mat imLeft, imRight)
 → System::TrackStereo()              src/System.cc:283
 → Tracking::GrabImageStereo()        src/Tracking.cc:1539
 → Frame 建構 (含特徵抽取)            src/Frame.cc:106  → ExtractKeyPoints (:544)
 → Tracking::Track()                  src/Tracking.cc:2012
      ├─ 初始化 StereoInitialization  src/Tracking.cc:2739   (僅第一幀)
      ├─ 幀間追蹤 TrackWithMotionModel / TrackReferenceKeyFrame
      ├─ 局部地圖追蹤 TrackLocalMap   src/Tracking.cc:3564
      ├─ NeedNewKeyFrame              src/Tracking.cc:3728
      └─ CreateNewKeyFrame            src/Tracking.cc:3940
 → (新 KF) LocalMapping::InsertKeyFrame  src/LocalMapping.cc:368
 → return Tcw (mCurrentFrame.GetPose())  src/Tracking.cc:1626
```

### 2.1 函式呼叫鏈逐段表

| 步驟 | File:Line | Class::Function | 主要輸入 | 主要輸出 | 下一步 |
| ---- | --------- | --------------- | -------- | -------- | ------ |
| 影像進入 | `System.cc:283` | `System::TrackStereo` | imLeft, imRight, timestamp | `Tcw` | `GrabImageStereo` |
| 前處理 | `System.cc:291-307` | rectify / resize / clone | 原始影像 | `imLeftToFeed` | — |
| 模式/重置檢查 | `System.cc:311-348` | 旗標檢查 | mb* flags | — | — |
| 灰階+建幀 | `Tracking.cc:1539` | `Tracking::GrabImageStereo` | 左右影像 | `mCurrentFrame` | `Frame()` |
| 特徵抽取 | `Frame.cc:544` | `Frame::ExtractKeyPoints` | 灰階影像 | `mvKeys`,`mDescriptors` | SuperPoint |
| 追蹤主流程 | `Tracking.cc:2012` | `Tracking::Track` | `mCurrentFrame` | `mState`,`Tcw` | 見 §3 |
| 回傳位姿 | `Tracking.cc:1626` | `GetPose()` | — | `Sophus::SE3f Tcw` | main |

### 2.2 GrabImageStereo 細節（`src/Tracking.cc:1539-1627`）

1. 左右影像存入 `mImGray` / `imGrayRight`，必要時 `cvtColor` 轉灰階（`:1548-1561`）。
2. 依感測器型態與相機模型，建構 `mCurrentFrame = Frame(...)`（`:1580-1606`）。
3. 設定 `mCurrentFrame.mNameFile / mnDataset`（`:1611-1612`）。
4. 呼叫 `Track()`（`:1621`）。
5. `return mCurrentFrame.GetPose()`（`:1626`）。

---

## 3. Tracking 主流程與狀態機

### 3.1 狀態機（State Machine）

定義於 `include/Tracking.h:121-129`：

```text
SYSTEM_NOT_READY = -1
NO_IMAGES_YET    =  0   // 還沒收到影像
NOT_INITIALIZED  =  1   // 收到影像但地圖未建立
OK               =  2   // 正常追蹤中
RECENTLY_LOST    =  3   // 剛跟丟，嘗試用 IMU / 重定位救回
LOST             =  4   // 徹底跟丟
OK_KLT           =  5   // (保留)
```

狀態轉移（`src/Tracking.cc:2012` 起的 `Track()`）：

```text
NO_IMAGES_YET ──(收到第一張)──► NOT_INITIALIZED
NOT_INITIALIZED ──Stereo/MonoInitialization 成功──► OK
OK ──幀間+局部地圖追蹤成功──► OK
OK ──局部地圖追蹤失敗──► RECENTLY_LOST  (純視覺) / RECENTLY_LOST(IMU)
RECENTLY_LOST ──IMU 預測 or 重定位成功──► OK
RECENTLY_LOST ──超時(>5s IMU / >2s 視覺)──► LOST
LOST ──KF<10──► ResetActiveMap (重置當前地圖)
LOST ──KF>10──► CreateMapInAtlas (開新子地圖)
```

### 3.2 Track() 逐步拆解（`src/Tracking.cc:2012-2732`）

| Step | 行號 | 做什麼 |
| ---- | ---- | ------ |
| 1 | 2025 | LocalMapping 報告 IMU 壞掉 → ResetActiveMap |
| 2 | 2040 | 時間戳異常檢查（倒退 / 跳變 >1s）→ 開新地圖或重置 |
| 3 | 2089 | IMU 模式設定 bias 初值 |
| 4 | 2099 | IMU 預積分 `PreintegrateIMU()` |
| — | 2120 | **鎖住地圖** `lock(pCurrentMap->mMutexMapUpdate)` |
| 5 | 2135 | **初始化**：`StereoInitialization()`(2739) 或 `MonocularInitialization()`(2879) |
| 6 | 2176 | **幀間位姿估計**（見 3.3） |
| 7 | 2446 | **局部地圖追蹤** `TrackLocalMap()` |
| 8 | 2483 | 依結果更新 `mState`（OK / RECENTLY_LOST / LOST） |
| 9 | 2571 | 更新速度模型、清理 MP、`NeedNewKeyFrame()` + `CreateNewKeyFrame()` |
| 10 | 2658 | LOST 處理：重置或開新地圖 |
| 11 | 2697 | 記錄相對位姿到 `mlRelativeFramePoses`（軌跡輸出用） |

### 3.3 位姿估計（Pose Estimation）

**幀間追蹤（兩兩匹配求初始位姿）**，`Track()` Step 6（`:2197-2223`）：

- `TrackReferenceKeyFrame()`（`Tracking.cc:3245`）：用 **詞袋匹配** `mspmatcher.SearchByBoWSP`（`:3259`）→ `Optimizer::PoseOptimization`（`:3276`）。
  - 觸發條件：無速度模型且 IMU 未初始化，或剛重定位完 < 2 幀。
- `TrackWithMotionModel()`（`Tracking.cc:3427`）：恆速模型給初值 → **投影匹配** `mspmatcher.SearchBySP`（`:3465`）→ `Optimizer::PoseOptimization`（`:3504`）。
  - 失敗會 fallback 回 `TrackReferenceKeyFrame()`（`:2217`）。

**局部地圖追蹤（精修位姿）**，`TrackLocalMap()`（`Tracking.cc:3564`）：
1. `UpdateLocalMap()`（`:4190`）：收集局部關鍵幀 + 局部地圖點。
2. `SearchLocalPoints()`（`:4093`）→ `mspmatcher.SearchByProjection1`（`:4178`）把局部 MP 投影匹配到當前幀。
3. `Optimizer::PoseOptimization`（`:3593`）或 IMU 版 `PoseInertialOptimizationLastKeyFrame`（`:3617`）。

**重定位（Relocalization）**，`Tracking.cc:4446`：
1. `mpKeyFrameDB->DetectRelocalizationCandidates()`（`:4457`）用詞袋找候選 KF。
2. 對每個候選 `mspmatcher.SearchByBoWSP`（`:4495`）。
3. `MLPnPsolver::iterate`（`:4549`）RANSAC 求位姿 → `Optimizer::PoseOptimization`（`:4587`）→ 不夠內點再投影補匹配（`:4604/4624`）。

### 3.4 是否建立新 KeyFrame（`NeedNewKeyFrame()` `Tracking.cc:3728`）

核心條件（簡化）：
- LocalMapping 是否閒置（`mpLocalMapper->AcceptKeyFrames()`）。
- 距離上一個 KF 已超過 `mMaxFrames`，或追蹤內點數低於閾值（與參考 KF 觀測比例）。
- 非單目時若 LocalMapping 隊列 `KeyframesInQueue() < 3` 才允許插入（`:3909`）。

通過後呼叫 `CreateNewKeyFrame()`（`:3940`），雙目/RGB-D 會同時用深度反投影產生新 MP。

---

## 4. 深度特徵前端（SuperPoint / LightGlue / ONNX）

> 這是 Rover-SLAM 相對於原版 ORB-SLAM3 最核心的改動：**抽特徵與匹配全部換成深度模型**。

### 4.1 SuperPoint 抽取器在哪裡初始化

- 建立：`Tracking::ParseORBParamFile` / `newParameterLoader` 中
  `mpExtractorLeft = new SPextractor(nFeatures, ...)`（`src/Tracking.cc:645/648/651`，舊版 `:1343/1346/1349`）。
  > 變數名仍叫 `mpExtractorLeft`，但型別已是 `SPextractor`（不是 `ORBextractor`）。
- `SPextractor` 建構子（`src/Extractors/SPextractor.cc:84`）內：
  `featureExtractor = new SuperPointOnnxRunner(); featureExtractor->InitOrtEnv(cfg);`（`:95-96`）
  → 載入 `superpoint.onnx`，建立 `Ort::Session`。

### 4.2 ONNX 推論在哪裡執行

`SuperPointOnnxRunner`（`src/Extractors/superpoint_onnx.cc`）：

| 階段 | 函式 | 行號 |
| ---- | ---- | ---- |
| 初始化環境 | `InitOrtEnv` | `:4`（`Ort::Session` 建立於 `:35`，CUDA 選項 `:27`） |
| 前處理 | `Extractor_PreProcess`（含 `NormalizeImage`） | `:68` |
| **推論** | `Extractor_Inference`（`ExtractorSession->Run`） | `:88`（Run 在 `:135`） |
| 後處理 | `Extractor_PostProcess` | `:165` |

### 4.3 keypoints / scores / descriptors 存在哪

- 抽取入口：`Frame::ExtractKeyPoints`（`src/Frame.cc:544`）呼叫
  `(*mpExtractorLeft)(im, mvKeys, mDescriptors)` → `SPextractor::operator()`（`SPextractor.cc:516`）。
- `operator()` 依金字塔層數決定 `ExtractSingleLayer`（`:592`）或 `ExtractMultiLayers`（`:619`）。
  本專案常用單層：
  ```
  Extractor_Inference(cfg, inputImage);
  Extractor_PostProcess(..., vKeyPoints, Descriptors);   // SPextractor.cc:599-600
  ```
- 結果存放在 **`Frame` 的成員**：
  - `mvKeys`（`std::vector<cv::KeyPoint>`）：keypoint 座標（score 放在 `KeyPoint.response`）。
  - `mDescriptors`（`cv::Mat`，**256 維 float** 每列一個描述子）。

### 4.4 LightGlue 匹配器在哪裡呼叫

- 匹配器物件：`Tracking::mspmatcher`（型別 `SPmatcher`，`include/Tracking.h:130`）。
- `SPmatcher` 建構子（`src/Matchers/SPmatcher.cc:23`）：
  `featureMatcher = new LightGlueDecoupleOnnxRunner();` → 載入 `lightglue_sim.onnx`。
- **核心匹配函式** `SPmatcher::MatchingPoints_onnx(Frame&, Frame&, vnMatches12)`（`SPmatcher.cc:457`）：
  ```
  normal_kpts = featureMatcher->Matcher_PreProcess(kpts, rows, cols);   // :492-493
  output      = featureMatcher->Matcher_Inference(kpts0,kpts1,desc0,desc1); // :528
  size        = featureMatcher->Matcher_PostProcess_fused(output, ..., vnMatches12); // :530
  ```
  LightGlue 的 ONNX `Run` 在 `src/Matchers/lightglue_onnx.cpp:213`。
- 包裝給 Tracking 用的高階介面（都在 `SPmatcher.cc`）：
  - `SearchForInitialization`（`:681`）— 單目初始化匹配。
  - `SearchByBoWSP`（`:1524/1670`）— 與 KF 的匹配（取代 ORB 的 `SearchByBoW`）。
  - `SearchBySP`（`:967/996/1050`）— 幀間/投影匹配。
  - `SearchByProjection1`（被 `TrackLocalMap` 用，`Tracking.cc:4178`）。

### 4.5 deep feature 如何接進 pose estimation

```text
Image (cv::Mat)
 → NormalizeImage (前處理)              SPextractor.cc:597
 → SuperPoint ONNX 推論                  superpoint_onnx.cc:88  (Run @135)
 → keypoint(mvKeys) + descriptor(mDescriptors, 256-d)  PostProcess @165
 → 存入 Frame                            Frame.cc:548
 → LightGlue 前處理(歸一化座標)          SPmatcher.cc:492
 → LightGlue ONNX 推論                   lightglue_onnx.cpp:213
 → matched pairs (vnMatches12)           SPmatcher.cc:530
 → 建立 2D-3D 對應 (Frame.mvpMapPoints)
 → Optimizer::PoseOptimization           Tracking.cc:3276/3504/3593
 → Tcw
```

> 取代/補強關係：原 ORB 流程的 `ORBextractor` → `SPextractor`、`ORBmatcher.SearchByBoW/Projection` → `SPmatcher.SearchByBoWSP/SearchBySP`。
> 程式中仍保留 `ORBextractor`/`ORBmatcher`（檔案還在），但 Tracking 實際走的是 SP/LG 路徑（舊呼叫多被註解掉，如 `Tracking.cc:3258` vs `:3259`）。
> 詞袋（loop/reloc）也改用 **SuperPoint 描述子訓練的 `SPVocabulary`**。

---

## 5. KeyFrame 與 MapPoint 的生命週期

```text
Frame ──(滿足條件)──► KeyFrame ──► 進入 LocalMapping ──► 三角化產生 MapPoint
  │                       │                                      │
  │                       └─ 進入 LoopClosing 隊列                ▼
  └─ 普通幀用完即丟                                       Local Map / BA 優化 / culling
```

### 5.1 Frame 何時建立
每張影像都會建立一個 `Frame`（`Frame.cc:106` stereo 建構子等），抽特徵、去畸變、（雙目）計算 stereo 匹配與深度（`ComputeStereoMatches`）。普通 Frame 用完即被 `mLastFrame = Frame(mCurrentFrame)` 覆蓋（`Tracking.cc:2684`）。

### 5.2 Frame → KeyFrame 的條件
`NeedNewKeyFrame()` 通過後，`CreateNewKeyFrame()`（`Tracking.cc:3940`）：
1. 用 `mCurrentFrame` 建 `KeyFrame* pKF = new KeyFrame(mCurrentFrame, map, KFDB)`。
2. 設為當前幀的參考 KF。
3. 雙目/RGB-D：對有正深度的點反投影建立新 MP。
4. `mpLocalMapper->InsertKeyFrame(pKF)`（送進 LocalMapping 隊列）。

### 5.3 MapPoint 何時建立
- **初始化時**：`StereoInitialization`（`Tracking.cc:2789-2798`，深度反投影）/ `CreateInitialMapMonocular`（`:2994`，三角化）。
- **追蹤時**：`CreateNewKeyFrame` 對雙目新點建 MP。
- **建圖時**：`LocalMapping::CreateNewMapPoints`（`LocalMapping.cc:517`）對相鄰 KF 三角化新點。

### 5.4 MapPoint 何時被刪 / 保留
`LocalMapping::MapPointCulling`（`LocalMapping.cc:462`）：新建 MP 在觀測不足（被追蹤比例過低、被觀測 KF 數不足）時剔除；通過考核才長期保留。

### 5.5 KeyFrame culling
`LocalMapping::KeyFrameCulling`（`LocalMapping.cc:1310`）：若某 KF 有 90% 以上地圖點能被其他 KF 觀測到，視為冗餘並刪除。

### 5.6 Local BA
`Optimizer::LocalBundleAdjustment`（`LocalMapping.cc:214`）或 IMU 版 `LocalInertialBA`（`:205`）。優化局部 KF 位姿 + 局部 MP 座標（+ IMU 參數）。

---

## 6. LocalMapping 流程（`LocalMapping::Run` `src/LocalMapping.cc:95`）

```text
Receive new KeyFrame (CheckNewKeyFrames)  :111
 → ProcessNewKeyFrame      :121 / 定義 :390
 → MapPointCulling         :132 / 定義 :462
 → CreateNewMapPoints      :143 / 定義 :517
 → SearchInNeighbors       :156 / 定義 :1048   (隊列清空時才做)
 → LocalBundleAdjustment   :214 / (IMU: LocalInertialBA :205)
 → (IMU 初始化 InitializeIMU / VIBA1 / VIBA2 / ScaleRefinement)  :243-317
 → KeyFrameCulling         :258 / 定義 :1310
 → mpLoopCloser->InsertKeyFrame  :325   (送進迴環隊列)
```

| 步驟 | File:Line | 輸入 | 輸出 | 對地圖的影響 |
| ---- | --------- | ---- | ---- | ------------ |
| ProcessNewKeyFrame | `LocalMapping.cc:390` | 隊列中的 KF | 更新 MP 觀測、計算詞袋、更新共視圖 | KF 正式插入 Map |
| MapPointCulling | `:462` | 近期新建 MP | 剔除壞點 | 移除低品質 MP |
| CreateNewMapPoints | `:517` | 當前 KF + 共視 KF | 三角化新 MP | 增加地圖點 |
| SearchInNeighbors | `:1048` | 兩級相鄰 KF | 融合重複 MP | 減少冗餘、增加觀測 |
| LocalBundleAdjustment | `Optimizer.cc` | 局部 KF/MP | 優化後位姿/座標 | 提升局部一致性 |
| KeyFrameCulling | `:1310` | 共視 KF | 刪冗餘 KF | 控制地圖規模 |

> 與 Tracking 的介面：Tracking 透過 `InsertKeyFrame`（`:368`）把 KF 推進 `mlNewKeyFrames` 隊列；LocalMapping 在 `Run()` 主迴圈逐個 pop 處理。

---

## 7. LoopClosing / Relocalization 流程

### 7.1 LoopClosing（`LoopClosing::Run` `src/LoopClosing.cc:100`）

```text
CheckNewKeyFrames                :115   (隊列來自 LocalMapping::InsertKeyFrame)
 → NewDetectCommonRegions        :128 / 定義 :387
       └─ KeyFrameDatabase::DetectNBestCandidates_sp / DetectCommonRegionsFromBoW_sp
       └─ 幾何驗證 (Sim3Solver, 投影匹配 FindMatchesByProjection :1692)
 → if mbMergeDetected:  MergeLocal()/MergeLocal2()   :214-216  (跨地圖融合)
 → if mbLoopDetected:   CorrectLoop()                :312→ :1781 (同地圖迴環矯正)
       └─ Pose Graph Optimization + RunGlobalBundleAdjustment :3377
```

- **候選搜尋**：`KeyFrameDatabase::DetectNBestCandidates_sp`（`KeyFrameDatabase.cc:658`），基於 **SuperPoint 詞袋**。
- **特徵匹配**：對候選做 `SPmatcher::SearchByBoWSP` + Sim3 求解（`Sim3Solver`）。
- **幾何驗證**：`Sim3Solver` RANSAC + `FindMatchesByProjection`（`LoopClosing.cc:1692`）累積一致觀測（`mnLoopNumCoincidences`）。
- **迴環矯正**：`CorrectLoop()`（`:1781`）做 Essential Graph / Pose Graph 優化，再觸發 `RunGlobalBundleAdjustment`（`:3377`）。
- **地圖融合**：若候選屬於不同子地圖 → `MergeLocal`（`:2098`）把兩張地圖焊接成一張（ORB-SLAM3 Atlas 特性）。

### 7.2 Relocalization（追蹤丟失時，`Tracking::Relocalization` `src/Tracking.cc:4446`）

```text
Tracking 進入 RECENTLY_LOST/LOST (純視覺)
 → Relocalization()                         Tracking.cc:4446
 → KeyFrameDatabase::DetectRelocalizationCandidates   :4457
 → SPmatcher::SearchByBoWSP (對每個候選)     :4495
 → MLPnPsolver::iterate (RANSAC 求位姿)      :4549
 → Optimizer::PoseOptimization              :4587
 → 內點不足則投影補匹配再優化               :4604/4624
 → 成功 → mState = OK, 記錄 mnLastRelocFrameId
```

> 觸發點在 `Track()`：純視覺 RECENTLY_LOST（`:2284`）與純定位模式 LOST（`:2329`）。

---

## 8. Thread / Queue / 共享資料結構

### 8.1 模組總覽表

| Thread / Module | Input | Output | Shared Data | Important Functions |
| --------------- | ----- | ------ | ----------- | ------------------- |
| **Tracking**（主執行緒） | 影像 Frame（+IMU） | `Tcw`、新 KF | `Atlas`/當前 `Map`、`KeyFrameDatabase` | `GrabImageStereo`(1539), `Track`(2012), `TrackLocalMap`(3564), `Relocalization`(4446) |
| **LocalMapping** | `mlNewKeyFrames` 隊列 | 優化後地圖、新 MP | `Atlas`、共視圖 | `Run`(95), `ProcessNewKeyFrame`(390), `CreateNewMapPoints`(517), `KeyFrameCulling`(1310) |
| **LoopClosing** | `mlpLoopKeyFrameQueue` 隊列 | 矯正後位姿、融合地圖 | `Atlas`、`KeyFrameDatabase` | `Run`(100), `NewDetectCommonRegions`(387), `CorrectLoop`(1781), `MergeLocal`(2098) |
| **Viewer** | `FrameDrawer`/`MapDrawer` | 畫面顯示 | 當前幀、地圖 | `Run` |

### 8.2 Queue（執行緒間傳遞 KF）

```text
Tracking ──InsertKeyFrame──► LocalMapping.mlNewKeyFrames      (LocalMapping.cc:368)
LocalMapping ──InsertKeyFrame──► LoopClosing.mlpLoopKeyFrameQueue (LoopClosing.cc:365 / 呼叫於 LocalMapping.cc:325)
```

### 8.3 共享資料與鎖（mutex）

| Mutex | 保護對象 | 位置 |
| ----- | -------- | ---- |
| `Map::mMutexMapUpdate` | 整張地圖更新（Tracking/LocalMapping/Loop 互斥） | `Tracking.cc:2120` 取得 |
| `LocalMapping::mMutexNewKFs` | `mlNewKeyFrames` 隊列 | `LocalMapping.cc:368/381` |
| `LocalMapping::mMutexStop` | 停止/釋放（純定位、迴環暫停 LM） | `LocalMapping.cc:1204/1232/1250` |
| `LoopClosing::mMutexLoopQueue` | 迴環 KF 隊列 | `LoopClosing.cc:365/375` |
| `System::mMutexMode` | `mbActivateLocalizationMode` 等 | `System.cc:312` |
| `System::mMutexReset` | `mbReset` / `mbResetActiveMap` | `System.cc:336` |
| `System::mMutexState` | `mTrackingState` 等對外查詢狀態 | `System.cc:327`（TrackStereo 尾） |

### 8.4 同步要點
- Tracking 在 `Track()` 全程持有 `mMutexMapUpdate`，確保用地圖時 LocalMapping/Loop 不會改它。
- 純定位模式切換需先 `RequestStop()` 並 **忙等** `isStopped()`（`System.cc:317-322`），確認 LocalMapping 真停下才切換，避免地圖狀態不一致。
- 迴環矯正 / 地圖融合期間，LoopClosing 會 `RequestStop` LocalMapping，並暫停 Global BA 既有執行緒。

---

## 9. 報告用整理

### A. Overall Code Flow

```text
main()  (mono_euroc.cc:33 / TrackStereo 同構)
 → System 初始化 (System.cc:50)  ── 載入 voc + onnx，建立 Atlas/KFDB，啟動 3 條背景執行緒
 → 影像輸入迴圈 (mono_euroc.cc:95)
 → System::TrackXXX (System.cc:283) → Tracking::GrabImageXXX (Tracking.cc:1539)
 → Tracking::Track (Tracking.cc:2012)
       初始化 / 幀間追蹤 / 局部地圖追蹤 / 位姿優化
 → KeyFrame 插入 (CreateNewKeyFrame → LocalMapping 隊列)
 → LocalMapping (Run:95)  三角化 + LBA + culling
 → LoopClosing  (Run:100) 迴環檢測 + 矯正 + 地圖融合 + Global BA
 → Trajectory 輸出 (SaveTrajectoryEuRoC, mono_euroc.cc:194)
```

### B. Frame-level Vertical Slice

```text
Image (cv::Mat)
 → Frame                (Frame.cc:106 建構, ExtractKeyPoints:544)
 → Features             (SuperPoint ONNX: superpoint_onnx.cc:88 → mvKeys/mDescriptors 256-d)
 → Matches              (LightGlue ONNX: lightglue_onnx.cpp:213 → SPmatcher.cc:530 vnMatches12)
 → Pose                 (Optimizer::PoseOptimization: Tracking.cc:3276/3504/3593)
 → KeyFrame             (CreateNewKeyFrame: Tracking.cc:3940)
 → MapPoint             (StereoInit:2789 / CreateNewMapPoints: LocalMapping.cc:517)
 → Optimized Map        (LocalBundleAdjustment / LoopClosing GBA)
 → Trajectory           (mlRelativeFramePoses: Tracking.cc:2705 → SaveTrajectoryEuRoC)
```

### C. Function Call Chain 表

| Step | File | Class / Function | Purpose | Input | Output |
| ---- | ---- | ---------------- | ------- | ----- | ------ |
| 1 | System.cc:283 | System::TrackStereo | 影像入口、前處理、模式檢查 | imLeft/Right, t | Tcw |
| 2 | Tracking.cc:1539 | Tracking::GrabImageStereo | 灰階化、建 Frame | 左右影像 | mCurrentFrame |
| 3 | Frame.cc:544 | Frame::ExtractKeyPoints | 抽 SuperPoint 特徵 | 灰階影像 | mvKeys, mDescriptors |
| 4 | superpoint_onnx.cc:88 | SuperPointOnnxRunner::Extractor_Inference | ONNX 抽特徵 | normalized image | tensors |
| 5 | Tracking.cc:2012 | Tracking::Track | 追蹤主流程/狀態機 | mCurrentFrame | mState, Tcw |
| 6 | Tracking.cc:3427 | Tracking::TrackWithMotionModel | 恆速模型幀間追蹤 | last/cur frame | bOK, 初值 Tcw |
| 7 | SPmatcher.cc:457 | SPmatcher::MatchingPoints_onnx | LightGlue 匹配 | 2 frames | vnMatches12 |
| 8 | Tracking.cc:3564 | Tracking::TrackLocalMap | 局部地圖精修 | local map | 精修 Tcw |
| 9 | Optimizer.cc | Optimizer::PoseOptimization | g2o 位姿優化 | 2D-3D 對應 | 優化 Tcw |
| 10 | Tracking.cc:3940 | Tracking::CreateNewKeyFrame | 建 KF + 新 MP | mCurrentFrame | KeyFrame |
| 11 | LocalMapping.cc:95 | LocalMapping::Run | 建圖/LBA/culling | KF 隊列 | 優化地圖 |
| 12 | LoopClosing.cc:100 | LoopClosing::Run | 迴環/融合/GBA | KF 隊列 | 全域一致地圖 |

### D. Data Structure Lifecycle

```text
Image
 └─► Frame              每幀建立；普通幀用完被 mLastFrame 覆蓋丟棄
      └─► KeyFrame      NeedNewKeyFrame 通過才升級；送入 LocalMapping，可被 KeyFrameCulling 刪
           └─► MapPoint 由深度反投影 / 三角化建立；MapPointCulling 剔除壞點
                └─► Map  KF 與 MP 的容器；屬於 Atlas 中某一張子地圖
                     └─► Atlas  多地圖管理；LOST 時開新子地圖，迴環時可融合子地圖
```

### E. Thread Interaction Diagram

```text
            影像 Frame
                │
         ┌──────▼───────┐  InsertKeyFrame   ┌──────────────┐
         │   Tracking   │ ────────────────► │ LocalMapping │
         │ (主執行緒)   │                    │  (Run thread)│
         └──────┬───────┘ ◄──────────────── └──────┬───────┘
                │   (替換 MP / Release)             │ InsertKeyFrame
        讀/寫(鎖 mMutexMapUpdate)                   ▼
                │                            ┌──────────────┐
                ├───────────────────────────│ LoopClosing  │
                │     共享 Atlas / KFDB       │  (Run thread)│
         ┌──────▼───────┐                    └──────┬───────┘
         │  Atlas / Map │ ◄─── 迴環矯正 / 地圖融合 / GBA ─┘
         │  KeyFrameDB  │
         └──────────────┘
                ▲
                │ (顯示)
         ┌──────┴───────┐
         │    Viewer    │
         └──────────────┘
```

---

## 10. 最值得報告的核心函式（Top 8）

| # | 函式 | File:Line | 為什麼重要 |
| - | ---- | --------- | ---------- |
| 1 | `Tracking::Track` | `Tracking.cc:2012` | 整個追蹤的狀態機與排程中樞，所有追蹤決策都在這 |
| 2 | `Tracking::GrabImageStereo` | `Tracking.cc:1539` | 影像 → Frame 的轉換點，特徵前端的觸發處 |
| 3 | `SPextractor::operator()` → `SuperPointOnnxRunner::Extractor_Inference` | `SPextractor.cc:516` / `superpoint_onnx.cc:88` | **深度抽特徵的核心**，取代 ORB |
| 4 | `SPmatcher::MatchingPoints_onnx` | `SPmatcher.cc:457` | **LightGlue 匹配核心**，取代 ORB 匹配 |
| 5 | `Tracking::TrackLocalMap` | `Tracking.cc:3564` | 把幀間初值精修成穩定位姿，決定追蹤是否成功 |
| 6 | `Optimizer::PoseOptimization` | 由 Tracking 多處呼叫 | 所有位姿估計的數值核心（g2o） |
| 7 | `LocalMapping::CreateNewMapPoints` + `LocalBundleAdjustment` | `LocalMapping.cc:517` / `:214` | 地圖增長與局部一致性優化 |
| 8 | `LoopClosing::NewDetectCommonRegions` + `CorrectLoop` | `LoopClosing.cc:387` / `:1781` | 迴環/融合，消除累積漂移 |

---

## 附註：需要進一步確認的點

1. **金字塔層數**：`SPextractor::operator()` 同時有單層（`ExtractSingleLayer`）與多層（`ExtractMultiLayers`）路徑，但多層的 `mModel->infer` 區塊被註解掉（`SPextractor.cc:629-635`）。實際執行時 `nlevels` 的設定值需從 settings.yaml 確認；若 >1 但多層推論被註解，行為需實測確認。
2. **純雙目（非 inertial）入口**：`Examples/` 未提供獨立 `stereo_*.cc`，`TrackStereo` 由誰呼叫需確認（可能透過 ROS 節點 `Examples/ROS/ORB_SLAM3`）。
3. **`mpVocabulary`（ORB 詞袋）**：`LoopClosing` 建構子傳入的是 `mpVocabulary`（`System.cc:250`）而非 `mpVocabulary_sp`，但實際候選搜尋走 `*_sp` 版本，這個傳參一致性需確認是否為殘留。
4. 報告中 `Optimizer.cc` 內各優化函式的精確行號未一一列出，若簡報需要可再補。
