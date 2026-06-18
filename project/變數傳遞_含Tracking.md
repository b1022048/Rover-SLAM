# 變數傳遞分析（二）：含 Tracking.cc

> **閱讀範圍**：`stereo_inertial_euroc.cc` + `src/System.cc` + `src/Tracking.cc`
>
> 本版本在「系統介面層」之上，再往內延伸到 **Tracking 內部的變數傳遞**：IMU 進佇列、影像建 Frame、Frame 進 Track()、結果回傳，以及 Tracking 把 KeyFrame 往下游送的「出口」。
>
> ⚠️ **誠實邊界**：LocalMapping / LoopClosing 兩個執行緒的**內部**處理仍不在範圍內。本版只講到 Tracking 把資料**交出去的那一刻**（呼叫 `mpLocalMapper->InsertKeyFrame`）為止。

---

## 0. 一張總圖（延伸到 Tracking）

```text
main (stereo_inertial_euroc.cc)
  │  ① 建構參數                          ② vImuMeas 組裝
  ▼
System 建構子 (System.cc:50) ──④ 發配 mpAtlas/mpKFDB/settings_ + 互設指標──► [LocalMapping][LoopClosing]
  │  ③ TrackStereo(imL,imR,t,vImuMeas)
  ▼
System::TrackStereo (System.cc:283)
  │  ⑤a GrabImuData(逐筆)        ⑤b GrabImageStereo(影像)
  ▼                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Tracking (主執行緒)                                                │
│  ⑤a' GrabImuData → mlQueueImuData 佇列 (鎖 mMutexImuQueue) :1771   │
│  ⑤b' GrabImageStereo:1539                                         │
│        ├─ 建 mCurrentFrame = Frame(影像, 抽特徵, ...)  :1580       │
│        ├─ Track() :1621                                           │
│        │    ├─ PreintegrateIMU() 取用 mlQueueImuData :1780        │
│        │    ├─ 狀態機 / 追蹤 / 優化 → 算出 mCurrentFrame 位姿      │
│        │    ├─ 軌跡記錄 mlRelativeFramePoses :2705                 │
│        │    ├─ mLastFrame = Frame(mCurrentFrame) :2684 (當前→上一) │
│        │    └─ ⑦ CreateNewKeyFrame → mpLocalMapper->InsertKeyFrame │
│        │         :4080  ←── Tracking 的「出口」(交給下游)          │
│        └─ return mCurrentFrame.GetPose() :1626                    │
└─────────────────────────────────────────────────────────────────┘
  ⑥ 回到 System：寫回 mTrackingState/mTrackedMapPoints (鎖 mMutexState)
  ▼  回傳 Tcw → main
```

---

## A. 系統介面層（與第一版相同，摘要）

1. **main → System 建構子**：組態參數 → 成員變數（`System.cc:118` `mStrVocabularyFilePath = strVocFile`）。
2. **main 內部 IMU 組裝**：`vAcc/vGyro` → `IMU::Point`（`:179`）→ `vImuMeas`。
3. **main → TrackStereo**：`imLeft, imRight, tframe, vImuMeas`（`:192`）→ 回傳 `Tcw`。
4. **System 發配共享物件**：`mpAtlas / mpKeyFrameDatabase / settings_` 傳入三模組建構子（`System.cc:222/227/250`）+ 互設指標（`:255-262`）。

詳見「變數傳遞（一）」。以下為**本版新增的 Tracking 內部**。

---

## B. ⭐ IMU 路徑：GrabImuData → 佇列 → 預積分

### B-1. System 把 vImuMeas 拆筆交給 Tracking
```cpp
for(...) mpTracker->GrabImuData(vImuMeas[i_imu]);   // System.cc:350-352
```

### B-2. Tracking 把單筆 IMU 放進「佇列」（不馬上算）
```cpp
void Tracking::GrabImuData(const IMU::Point &imuMeasurement)   // Tracking.cc:1771
{
    unique_lock<mutex> lock(mMutexImuQueue);   // 上鎖：IMU 與影像可能來自不同 thread
    mlQueueImuData.push_back(imuMeasurement);  // 只是暫存，還沒計算
}
```
- 變數落腳處：`mlQueueImuData`（IMU 等待佇列），用 `mMutexImuQueue` 保護。

### B-3. 影像進來後才取用佇列做預積分
`PreintegrateIMU()`（`Tracking.cc:1780`）在 `Track()` 內被呼叫，從 `mlQueueImuData` 取出「上一幀～當前幀」之間的 IMU（`:1792` 用 `mlQueueImuData.size()`），做中值積分，結果寫進 `mCurrentFrame` 的預積分成員。

**講法**：IMU 變數路徑是「**逐筆入佇列（GrabImuData）→ 影像觸發時整段取出（PreintegrateIMU）→ 積分結果掛到當前 Frame**」，典型的生產者-消費者緩衝。

---

## C. ⭐ 影像路徑：GrabImageStereo → Frame → Track()

### C-1. System 把影像交給 Tracking
```cpp
Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename); // System.cc:354
```

### C-2. Tracking 把影像「包成」一個 Frame 物件
`Tracking::GrabImageStereo`（`Tracking.cc:1539`）：
```cpp
mImGray = imRectLeft;  imGrayRight = imRectRight;     // :1543-1545 影像存成員/區域
// 依感測器型態建構當前幀（建構過程中就抽好特徵）
mCurrentFrame = Frame(mImGray, imGrayRight, timestamp,
                      mpExtractorLeft, mpExtractorRight, mpSPVocabulary,
                      mK, mDistCoef, mbf, mThDepth, mpCamera, ...);   // :1580-1606
mCurrentFrame.mNameFile = filename;                  // :1611
mCurrentFrame.mnDataset = mnNumDataset;              // :1612
```
- **關鍵變數誕生**：`mCurrentFrame`（當前幀）。影像 + 特徵 + 內參 + IMU 標定全部封裝進這個物件，成為後續所有處理的載體。

### C-3. 呼叫 Track()，再回傳位姿
```cpp
Track();                                  // Tracking.cc:1621  主流程
return mCurrentFrame.GetPose();           // :1626  回傳 Tcw
```

**講法**：影像變數路徑是「**原始影像 → 封裝成 `mCurrentFrame` → 交給 Track() 處理 → 從 Frame 取出位姿回傳**」。`mCurrentFrame` 是 Tracking 內部最核心的「資料中樞」。

---

## D. Track() 內部的幾個關鍵變數傳遞（可講的層級）

> Track() 演算法細節很多，但**變數傳遞層級**有幾個清楚可講的點：

| 變數動作 | 程式碼 | 意義 |
| -------- | ------ | ---- |
| 鎖住共享地圖 | `lock(pCurrentMap->mMutexMapUpdate)` `Tracking.cc:2120` | Track 全程獨占地圖，防止 LocalMapping/Loop 同時改 |
| 預積分取佇列 | `PreintegrateIMU()` `:2105` | 取 `mlQueueImuData` → 掛到 `mCurrentFrame` |
| 當前幀 → 上一幀 | `mLastFrame = Frame(mCurrentFrame)` `:2684` | 處理完把當前幀「下移」成上一幀，給下一輪用 |
| 速度模型 | `mVelocity = mCurrentFrame.GetPose() * LastTwc` `:2579` | 由前後兩幀位姿算相對運動，傳給下一幀當初值 |
| 軌跡記錄 | `mlRelativeFramePoses.push_back(Tcr_)` `:2705` | 每幀相對位姿存入串列，供最後 SaveTrajectory 用 |

**講法**：Tracking 用 `mCurrentFrame` / `mLastFrame` 兩個成員「接力」——當前幀算完就變上一幀，位姿差變成下一幀的速度初值；同時把每幀位姿累積進 `mlRelativeFramePoses`（這就是最後 `SaveTrajectoryEuRoC` 的資料來源）。

---

## E. ⭐ Tracking 的「出口」：把 KeyFrame 交給下游

這是 Tracking 與其他兩個執行緒之間**唯一你目前能看到的傳遞點**（呼叫端在 Tracking.cc，看得到「交出去」的動作）：

```cpp
// Track() 判斷需要新關鍵幀後：
bool bNeedKF = NeedNewKeyFrame();           // Tracking.cc:2621
if(bNeedKF && ...) CreateNewKeyFrame();     // :2632

// CreateNewKeyFrame() 內：
KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB); // 由當前幀產生 KF
mpLocalMapper->InsertKeyFrame(pKF);         // Tracking.cc:4080  ←── 交給 LocalMapping
```

也包含初始化時的交付（`Tracking.cc:2840 / 3120 / 3121`）。

**講法**：
- Tracking 把 `mCurrentFrame` **升級**成一個 `KeyFrame`（new 在共享 Atlas 的當前 Map 裡）。
- 然後把這個 `KeyFrame*` **指標**透過 `mpLocalMapper->InsertKeyFrame()` 交給 LocalMapping。
> ✅ 本版可講到「Tracking 在這裡把 KF 指標交出去」。
> ❌ `InsertKeyFrame` 之後 LocalMapping 在佇列 `mlNewKeyFrames` 裡怎麼處理 → 屬於再下一版（需讀 LocalMapping.cc）。

### Tracking 對 LocalMapping 的幾個「狀態查詢」（也看得到）
在 `NeedNewKeyFrame()` 內，Tracking 會先問 LocalMapping 忙不忙再決定要不要插：
```cpp
mpLocalMapper->AcceptKeyFrames()      // Tracking.cc:3790/3945  LM 是否閒置
mpLocalMapper->KeyframesInQueue()<3   // :3910                  佇列是否太滿
mpLocalMapper->InterruptBA()          // :3903                  叫 LM 中斷 BA
mpLocalMapper->SetNotStop(true)       // :3947                  插 KF 期間禁止 LM 停
```
這些是「Tracking 讀/寫 LocalMapping 旗標」的傳遞，呼叫端在 Tracking.cc，可講。

---

## F. 回傳路徑：Tracking → System → main

```cpp
// Tracking.cc:1626
return mCurrentFrame.GetPose();        // Frame 內取出位姿 Tcw

// 回到 System::TrackStereo 尾 (System.cc)
unique_lock<mutex> lock2(mMutexState);
mTrackingState      = mpTracker->mState;                      // 讀 Tracking 成員 → 寫 System 成員
mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
return Tcw;                                                   // 回傳 main
```

**講法**：結果從 `mCurrentFrame` 取出 → 回到 System → 快照進 System 成員（鎖保護）→ 回傳 main。System 同時直接讀了 `mpTracker->mState` 等 Tracking 公開成員，這也是一種跨層變數傳遞。

---

## G. 本版可講的「變數總清單」

| 變數 | 屬於 | 角色 | 傳遞路線 |
| ---- | ---- | ---- | -------- |
| `vImuMeas` | main | 一幀的 IMU 段 | main → TrackStereo → GrabImuData |
| `mlQueueImuData` | Tracking | IMU 等待佇列 | GrabImuData 寫入 → PreintegrateIMU 取用 |
| `mCurrentFrame` | Tracking | 當前幀資料中樞 | GrabImageStereo 建立 → Track 處理 → 取位姿 |
| `mLastFrame` | Tracking | 上一幀 | `mLastFrame = Frame(mCurrentFrame)` 接力 |
| `mVelocity` | Tracking | 速度模型 | 前後幀位姿差 → 下一幀初值 |
| `mlRelativeFramePoses` | Tracking | 軌跡串列 | 每幀累積 → SaveTrajectory |
| `KeyFrame* pKF` | Tracking→共享 | 關鍵幀 | CreateNewKeyFrame → InsertKeyFrame（交出口） |
| `Tcw` | 回傳鏈 | 位姿結果 | Frame → Tracking → System → main |
| `mTrackingState` 等 | System | 對外狀態快照 | Tracking 成員 → System 成員（鎖 mMutexState） |

---

## H. 報告用「誠實邊界」聲明

> 本報告涵蓋從 **應用層 → System → Tracking** 的變數傳遞：IMU 如何入佇列並在影像觸發時預積分、影像如何封裝成 `mCurrentFrame` 並貫穿整個追蹤、結果如何回傳，以及 Tracking 如何把 `KeyFrame` 指標交給 LocalMapping（交出口）。**至於 LocalMapping / LoopClosing 收到 KeyFrame 之後的內部處理，屬於後續工作。**
