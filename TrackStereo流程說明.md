# `System::TrackStereo` 完整流程說明

> 對象函式：
> ```cpp
> Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
>                                  const double &timestamp,
>                                  const vector<IMU::Point>& vImuMeas, string filename)
> ```
> 位置：`src/System.cc:284 ~ 367`
> 角色：雙目（或雙目+IMU）的**對外入口**。把左右影像（前處理後）交給 Tracking，回傳相機位姿 `Sophus::SE3f Tcw`。

---

## 0. 名詞 / 符號約定

| 符號 | 意義 |
|------|------|
| `mp開頭` | member pointer（類別的成員指標），如 `mpTracker`、`mpLocalMapper` |
| `m開頭` | member（類別的成員變數），如 `mSensor`、`mCurrentFrame` |
| 黑盒子 | 第三方函式庫的 API（OpenCV / Sophus / 系統）。只標注「屬於誰」，不追內部實作 |
| 位置 | `檔名:行號`（可點擊跳轉） |

本文件追蹤深度：**TrackStereo 內所有呼叫 → 一層展開**；對巨大的主流程 `Track()` 只說明職責與位置（其內部另成體系，篇幅過大不全展開）。

---

## 1. 最上層流程圖（TrackStereo 本體）

```
TrackStereo(imLeft, imRight, timestamp, vImuMeas, filename)   [System.cc:284]
│
├─① 感測器型別檢查                                            [System.cc:287]
│     若 mSensor 不是 STEREO / IMU_STEREO → 報錯 exit(-1)
│
├─② 影像前處理：產生 imLeftToFeed / imRightToFeed             [System.cc:293~310]
│     ├─ 需要校正 needToRectify()？  → cv::remap 套校正表
│     ├─ 需要縮放 needToResize()？   → cv::resize
│     └─ 都不需要                    → imLeft.clone()
│
├─③ 模式切換檢查（純定位模式 開/關）                          [System.cc:312~334]
│     ├─ 啟用純定位：LocalMapper 停止 + Tracking 只追蹤
│     └─ 停用純定位：恢復正常 SLAM
│
├─④ 重置檢查                                                  [System.cc:336~350]
│     ├─ mbReset          → mpTracker->Reset()
│     └─ mbResetActiveMap → mpTracker->ResetActiveMap()
│
├─⑤ 餵 IMU 資料（僅 IMU_STEREO）                              [System.cc:352~354]
│     for 每筆 vImuMeas → mpTracker->GrabImuData(...)
│
├─⑥ ★核心★ 影像追蹤                                          [System.cc:357]
│     Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename)
│
├─⑦ 更新對外查詢狀態（上鎖）                                  [System.cc:361~364]
│     mTrackingState / mTrackedMapPoints / mTrackedKeyPointsUn
│
└─⑧ return Tcw                                                [System.cc:366]
```

---

## 2. 各步驟詳解 + 呼叫的跨檔函式

### ① 感測器型別檢查 — `System.cc:287`
```cpp
if(mSensor!=STEREO && mSensor!=IMU_STEREO){ ... exit(-1); }
```
確保呼叫者用對了介面。EuRoC stereo-inertial 範例傳的是 `IMU_STEREO`，通過。

---

### ② 影像前處理 — `System.cc:293~310`

目的：把唯讀的原始影像 `imLeft/imRight`，加工成「可餵給追蹤器」的 `imLeftToFeed/imRightToFeed`。三選一：

#### 2-A 需要校正（立體校正 rectify）
```cpp
if(settings_ && settings_->needToRectify()){      // System.cc:294
    cv::Mat M1l = settings_->M1l();               // System.cc:295
    cv::Mat M2l = settings_->M2l();               // System.cc:296
    cv::Mat M1r = settings_->M1r();               // System.cc:297
    cv::Mat M2r = settings_->M2r();               // System.cc:298
    cv::remap(imLeft,  imLeftToFeed,  M1l, M2l, cv::INTER_LINEAR);  // System.cc:299
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);  // System.cc:300
}
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `settings_->needToRectify()` | **`include/Settings.h:85`** | getter，`return bNeedToRectify_;` |
| └ `bNeedToRectify_` 真正設值處 | **`src/Settings.cc:327`** | `readCamera2()` 中，若 Camera2 是 `PinHole` → `= true` |
| `settings_->M1l()` | **`include/Settings.h:120`** | getter，回傳左圖 x 映射表 `M1l_` |
| `settings_->M2l()` | **`include/Settings.h:121`** | getter，回傳左圖 y 映射表 `M2l_` |
| `settings_->M1r()` | **`include/Settings.h:122`** | getter，回傳右圖 x 映射表 `M1r_` |
| `settings_->M2r()` | **`include/Settings.h:123`** | getter，回傳右圖 y 映射表 `M2r_` |
| └ 四張表的計算來源 | **`src/Settings.cc:549` `precomputeRectificationMaps()`** | 初始化時算一次（由 `Settings.cc:207` 呼叫） |
| `cv::remap` | **OpenCV 黑盒子** | 套用映射表做逐像素重映射（去畸變+對齊） |
| `cv::INTER_LINEAR` | **OpenCV 黑盒子（常數）** | 雙線性插值旗標 |

> `precomputeRectificationMaps()` 內部流程（僅供參考，位置 `Settings.cc:549~593`）：
> 取 K1/K2 → 取左右相對位姿拆出 R12/t12 → `cv::stereoRectify`（OpenCV 黑盒子）→ `cv::initUndistortRectifyMap`（OpenCV 黑盒子）產出 M1l_/M2l_/M1r_/M2r_ → 更新校正後內參與基線 bf_ →（IMU 模式）修正 Tbc_。

#### 2-B 需要縮放
```cpp
else if(settings_ && settings_->needToResize()){  // System.cc:302
    cv::resize(imLeft,  imLeftToFeed, settings_->newImSize());  // System.cc:303
    cv::resize(imRight, imRightToFeed, settings_->newImSize()); // System.cc:304
}
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `settings_->needToResize()` | **`include/Settings.h:84`** | getter，`return bNeedToResize1_;` |
| └ `bNeedToResize1_` 設值處 | **`src/Settings.cc:418`** | `readImageInfo()` 中，若 yaml 有 `Camera.newHeight` → `= true` |
| `settings_->newImSize()` | **`include/Settings.h:81`** | getter，回傳目標尺寸 `newImSize_` |
| `cv::resize` | **OpenCV 黑盒子** | 影像縮放 |

#### 2-C 都不需要
```cpp
else{
    imLeftToFeed  = imLeft.clone();   // System.cc:308
    imRightToFeed = imRight.clone();  // System.cc:309
}
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `cv::Mat::clone()` | **OpenCV 黑盒子** | 深拷貝一份影像（避免共用記憶體 / 動到唯讀輸入） |

---

### ③ 模式切換檢查 — `System.cc:312~334`

用 `mMutexMode` 上鎖後，檢查兩個旗標：

```cpp
if(mbActivateLocalizationMode){                   // System.cc:315  啟用純定位
    mpLocalMapper->RequestStop();                 // System.cc:317
    while(!mpLocalMapper->isStopped()){ usleep(1000); }  // System.cc:320~323
    mpTracker->InformOnlyTracking(true);          // System.cc:325
    mbActivateLocalizationMode = false;           // System.cc:326
}
if(mbDeactivateLocalizationMode){                 // System.cc:328  停用純定位
    mpTracker->InformOnlyTracking(false);         // System.cc:330
    mpLocalMapper->Release();                      // System.cc:331
    mbDeactivateLocalizationMode = false;         // System.cc:332
}
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `mpLocalMapper->RequestStop()` | **`src/LocalMapping.cc:1204`** | 設 `mbStopRequested=true`、`mbAbortBA=true`（只是「發出請求」） |
| `mpLocalMapper->isStopped()` | **`src/LocalMapping.cc:1232`** | 回傳 `mbStopped`，用來等待 LocalMapping 真的停下 |
| `usleep(1000)` | **系統（unistd）黑盒子** | 等待 1ms 再輪詢 |
| `mpTracker->InformOnlyTracking(bool)` | **`src/Tracking.cc:4879`** | 設 `mbOnlyTracking = flag` |
| `mpLocalMapper->Release()` | **`src/LocalMapping.cc:1250`** | 設 `mbStopped/mbStopRequested=false`、清空緩衝關鍵影格，恢復運行 |

---

### ④ 重置檢查 — `System.cc:336~350`

用 `mMutexReset` 上鎖後：

```cpp
if(mbReset){                            // System.cc:339  完整重置
    mpTracker->Reset();                 // System.cc:341
    mbReset = false; mbResetActiveMap = false;
}
else if(mbResetActiveMap){             // System.cc:345  只重置當前活動地圖
    mpTracker->ResetActiveMap();        // System.cc:347
    mbResetActiveMap = false;
}
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `mpTracker->Reset()` | **`src/Tracking.cc:4673`** | 重置整個追蹤系統 |
| `mpTracker->ResetActiveMap()` | **`src/Tracking.cc:4738`** | 只重置當前活動地圖 |

---

### ⑤ 餵 IMU 資料（僅 IMU_STEREO）— `System.cc:352~354`

```cpp
if (mSensor == System::IMU_STEREO)
    for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        mpTracker->GrabImuData(vImuMeas[i_imu]);   // System.cc:354
```

把「兩張影像之間累積的整批 IMU 量測」逐筆交給 Tracking 暫存（IMU 取樣率遠高於影像）。

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `mpTracker->GrabImuData(const IMU::Point&)` | **`src/Tracking.cc:1771`** | 上鎖後 `mlQueueImuData.push_back(...)`，只存不算 |

> `mlQueueImuData` 型別：`std::list<IMU::Point>`，宣告於 **`include/Tracking.h:243`**。
> `IMU::Point` 成員：`Eigen::Vector3f a`（加速度）、`Eigen::Vector3f w`（角速度）、`double t`（時間戳），定義於 **`include/ImuTypes.h:46~59`**。

---

### ⑥ ★核心★ 影像追蹤 — `System.cc:357`

```cpp
Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);
```

| 呼叫 | 位置 |
|------|------|
| `mpTracker->GrabImageStereo(...)` | **`src/Tracking.cc:1539`** |

#### ⑥ 展開：`Tracking::GrabImageStereo`（`Tracking.cc:1539~1627`）

```
GrabImageStereo(imRectLeft, imRectRight, timestamp, filename)   [Tracking.cc:1539]
│
├─ 影像暫存：mImGray=左, imGrayRight=右, mImRight=右   [Tracking.cc:1543~1545]
│
├─ Step1 轉灰階（若為 3 或 4 通道）                     [Tracking.cc:1548~1576]
│     cvtColor(...)   ← OpenCV 黑盒子
│
├─ 依感測器/相機組態建立 mCurrentFrame（4 個分支）     [Tracking.cc:1580~1606]
│     ├ STEREO     且無 camera2 → Frame(...,mpCamera)              [1581 → Frame.cc:106]
│     ├ STEREO     且有 camera2 → Frame(...,mpCamera2,mTlr)        [1594 → Frame.cc:1520]
│     ├ IMU_STEREO 且無 camera2 → Frame(...,&mLastFrame,*mpImuCalib)[1597 → Frame.cc:106]   ★EuRoC 走這條★
│     └ IMU_STEREO 且有 camera2 → Frame(...,camera2,Tlr,lastFrame,ImuCalib)[1604 → Frame.cc:1520]
│
├─ 記錄檔名 / dataset 編號                              [Tracking.cc:1611~1612]
│
├─ Step2 Track()  ← 追蹤主流程                          [Tracking.cc:1621 → Tracking.cc:2012]
│
└─ return mCurrentFrame.GetPose()                       [Tracking.cc:1626 → Frame.h:151]
```

| 呼叫 | 位置 | 說明 |
|------|------|------|
| `cvtColor(...)` | **OpenCV 黑盒子** | RGB/BGR/RGBA/BGRA → 灰階 |
| `mImGray.channels()` | **OpenCV 黑盒子**（cv::Mat） | 取通道數 |
| `Frame(...)` 雙目/IMU雙目（無 camera2） | **`src/Frame.cc:106`**（宣告 `Frame.h:68`） | 雙目 Frame 建構子；EuRoC stereo-inertial 走此 |
| `Frame(...)` 含 camera2 | **`src/Frame.cc:1520`**（宣告 `Frame.h:357/358`） | 雙相機模型 Frame 建構子 |
| `Track()` | **`src/Tracking.cc:2012`** | 追蹤主流程（見下方 ⑥-Track） |
| `mCurrentFrame.GetPose()` | **`include/Frame.h:151`** | inline，`return mTcw;`（Track 算完後寫入的位姿） |

#### ⑥-Frame：`Frame::Frame`（`src/Frame.cc:106~239`，雙目/IMU雙目建構子）

把原始左右影像 → 結構化的一幀（特徵點、深度、網格）。主要步驟與內部呼叫：

| 步驟 | 內容 | 內部呼叫 / 位置 |
|------|------|------------------|
| 初始化清單 | 存參數、`mK(K.clone())`、`mDistCoef(distCoef.clone())` 深拷貝 | `Frame.cc:107~109` |
| Step1 幀 ID | `mnId = nNextId++` | `Frame.cc:113` |
| Step2 金字塔參數 | `mpExtractorLeft->GetLevels()` 等 | `SPextractor`（專案類別）；呼叫於 `Frame.cc:118~134` |
| Step3 提特徵（雙執行緒） | `Frame::ExtractKeyPoints` | **`src/Frame.cc:544`**；開執行緒於 `Frame.cc:142,144` |
| Step4 特徵數檢查 | `N=mvKeys.size()`（156）；空則 return（159） | `Frame.cc:156~159` |
| Step5a 去畸變 | `UndistortKeyPoints()` | **`src/Frame.cc:1060`**；呼叫於 `Frame.cc:163` |
| Step5b 雙目匹配算深度 | `ComputeStereoMatches()` | **`src/Frame.cc:1159`**；呼叫於 `Frame.cc:171` |
| 第一幀專屬 | `ComputeImageBounds(imLeft)` | **`src/Frame.cc:1115`**；呼叫於 `Frame.cc:191` |
| Step6 特徵分配網格 | `AssignFeaturesToGrid()` | **`src/Frame.cc:488`**；呼叫於 `Frame.cc:238` |

> `thread` / `.join()` 屬 **C++ 標準函式庫 `<thread>` 黑盒子**。

#### ⑥-Track：`Tracking::Track`（`src/Tracking.cc:2012`）

整個 SLAM 的**主流程**（函式極長，自成體系，本文件不逐行展開，僅列職責與位置）：
- IMU 預積分（`PreintegrateIMU()`，**`src/Tracking.cc:1780`**）
- 地圖初始化 / 追蹤上一幀或參考關鍵影格 / 追蹤局部地圖
- 關鍵影格判定與插入
- 把估計出的位姿寫入 `mCurrentFrame`（之後由 `GetPose()` 取出）

> 若需要，可另外針對 `Track()` 再做一份專屬流程說明。

---

### ⑦ 更新對外查詢狀態 — `System.cc:361~364`

```cpp
unique_lock<mutex> lock2(mMutexState);
mTrackingState      = mpTracker->mState;                     // System.cc:362
mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints; // System.cc:363
mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;     // System.cc:364
```
上鎖後，把追蹤狀態、地圖點、去畸變特徵點複製到 System 成員，供外部執行緒查詢。
（`mState`、`mCurrentFrame` 為 Tracking 成員，宣告於 `include/Tracking.h`。）

---

### ⑧ 回傳 — `System.cc:366`
```cpp
return Tcw;   // Sophus::SE3f，相機位姿（旋轉+平移）
```

---

## 3. 跨檔案位置總表（速查）

| 函式 / 符號 | 位置 | 歸屬 |
|-------------|------|------|
| `System::TrackStereo` | `src/System.cc:284` | 專案 |
| `Settings::needToRectify` | `include/Settings.h:85` | 專案 |
| `Settings::needToResize` | `include/Settings.h:84` | 專案 |
| `Settings::newImSize` | `include/Settings.h:81` | 專案 |
| `Settings::M1l/M2l/M1r/M2r` | `include/Settings.h:120/121/122/123` | 專案 |
| `Settings::precomputeRectificationMaps` | `src/Settings.cc:549` | 專案 |
| `bNeedToRectify_` 設值 | `src/Settings.cc:327`（`readCamera2`） | 專案 |
| `bNeedToResize1_` 設值 | `src/Settings.cc:418`（`readImageInfo`） | 專案 |
| `LocalMapping::RequestStop` | `src/LocalMapping.cc:1204` | 專案 |
| `LocalMapping::isStopped` | `src/LocalMapping.cc:1232` | 專案 |
| `LocalMapping::Release` | `src/LocalMapping.cc:1250` | 專案 |
| `Tracking::InformOnlyTracking` | `src/Tracking.cc:4879` | 專案 |
| `Tracking::Reset` | `src/Tracking.cc:4673` | 專案 |
| `Tracking::ResetActiveMap` | `src/Tracking.cc:4738` | 專案 |
| `Tracking::GrabImuData` | `src/Tracking.cc:1771` | 專案 |
| `Tracking::GrabImageStereo` | `src/Tracking.cc:1539` | 專案 |
| `Tracking::Track` | `src/Tracking.cc:2012` | 專案 |
| `Frame::Frame`（雙目/IMU雙目） | `src/Frame.cc:106` | 專案 |
| `Frame::Frame`（含 camera2） | `src/Frame.cc:1520` | 專案 |
| `Frame::ExtractKeyPoints` | `src/Frame.cc:544` | 專案 |
| `Frame::UndistortKeyPoints` | `src/Frame.cc:1060` | 專案 |
| `Frame::ComputeStereoMatches` | `src/Frame.cc:1159` | 專案 |
| `Frame::ComputeImageBounds` | `src/Frame.cc:1115` | 專案 |
| `Frame::AssignFeaturesToGrid` | `src/Frame.cc:488` | 專案 |
| `Frame::GetPose` | `include/Frame.h:151` | 專案 |
| `IMU::Point` | `include/ImuTypes.h:46` | 專案 |
| `Tracking::mlQueueImuData` | `include/Tracking.h:243` | 專案 |

### 黑盒子函式（只標注歸屬，不追實作）

| 函式 / 符號 | 歸屬 |
|-------------|------|
| `cv::remap`、`cv::resize`、`cv::Mat::clone`、`cv::Mat::channels`、`cvtColor`、`cv::INTER_LINEAR` | **OpenCV** |
| `cv::stereoRectify`、`cv::initUndistortRectifyMap`（在 precompute 內） | **OpenCV** |
| `Sophus::SE3f` 及其運算 | **Sophus** |
| `Eigen::Vector3f` 等 | **Eigen** |
| `usleep` | **系統 `<unistd.h>`** |
| `std::thread` / `.join()`、`std::unique_lock` / `std::mutex` | **C++ 標準函式庫** |

---

## 4. 一句話總覽

`TrackStereo`（`System.cc:284`）= **前處理影像**（校正/縮放/複製，靠 `Settings` 的映射表與 OpenCV）→ **處理模式與重置**（操作 `LocalMapping`、`Tracking`）→ **餵 IMU**（`GrabImuData`）→ **交給 `GrabImageStereo`**（`Tracking.cc:1539`，內部建 `Frame` 並跑 `Track()`）→ **取回位姿並更新狀態** → 回傳 `Sophus::SE3f Tcw`。
