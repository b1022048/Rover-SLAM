# ORB-SLAM2/3 改良型論文調查筆記（2026-07-05，已依使用者要求排除 MDPI 來源）

> 硬性限制：本筆記排除所有 MDPI 出版品（Sensors、Remote Sensing、Applied Sciences、Electronics、
> Machines 等）。原本收集到的 YPR-SLAM（Sensors）、ADM-SLAM（Sensors）、
> 《An Improved Visual SLAM Method with Adaptive Feature Extraction》（Applied Sciences）
> 已從「具體論文例子」中移除，換成下方的 IEEE / Elsevier / Wiley / Taylor & Francis 論文。
> arXiv preprint 可用但已標註「未過審」。

## 具體論文清單（貢獻拆解）

### 1. PLE-SLAM
- 來源：arXiv:2401.01081v2（2024-01，**preprint，未過審**）。搜尋結果有片段聲稱正式發表於
  IEEE Transactions on Instrumentation and Measurement，但回傳的卷期資訊（Vol. 14, No. 8, August 2021）
  與 arXiv 掛出時間（2024）矛盾，**判斷是摘要生成器誤引，未查證屬實**——引用時只確定 arXiv 版本存在，
  期刊收錄狀態标示為「未確認」。
- 基於 ORB-SLAM3，四執行緒：tracking / local mapping / loop closing / dynamic feature elimination
- 貢獻：
  1. Point-line feature 整合（合併不穩定線段、抑制短線段）
  2. 高效 IMU 初始化（陀螺儀 bias 用迭代優化 2D 觀測，加速度計 bias／重力方向用解析解）
  3. YOLOv5+DeepSORT 動態特徵剔除
  4. SuperPoint+SuperGlue 用於迴環偵測
- 資料集：EuRoC、OpenLORIS-Scene、TUM-VI
- 即時性：DNN 模組用 TensorRT 加速
- 具體 ATE 數字：原文摘要中未提供（只說 state-of-the-art）

### 2. DynaTM-SLAM
- Robotics and Autonomous Systems (Elsevier)，2024
- 貢獻：
  1. YOLOv7 物件偵測找動態物體候選區
  2. 滑動視窗內做 template matching，快速篩掉真正的動態特徵點（比純語意分割快）
  3. 建立線上物件資料庫維持靜態物體的資料關聯一致性，供帶語意約束的 BA 使用
- 資料集：TUM RGB-D
- 主打軸：強健性（動態場景）＋即時性（template matching 比逐幀語意分割快）
- 具體 ATE/RPE 數字：搜尋摘要未提供精確百分比，需查原文（ScienceDirect 全文未 fetch 到，僅摘要）

### 2b. YoloV8-SLAM（原文標題：Advancing real-world visual SLAM: Integrating adaptive segmentation with dynamic object detection...）
- Expert Systems with Applications (Elsevier)，2024，Volume 255
- 基於 ORB-SLAM3
- 貢獻：
  1. 自適應分割：融合連續幀的幾何運動資訊分割動態物體
  2. Enhanced Multi-View Geometry
  3. 高速運動資訊擷取演算法
- 量化結果：ATE 降低 38–45%，RPE 降低 66–72%（對比 baseline，資料集細節未查證）
- 主打軸：精度＋強健性（動態場景）

### 3. AFE-ORB-SLAM
- Journal of Intelligent & Robotic Systems (Springer)，2022（早於 2023，僅作背景參考）
- 貢獻：adaptive FAST threshold ＋ 影像增強，應對複雜光照環境的單目 VSLAM

### 4. PLPF-VSLAM
- Journal of Field Robotics，2024，vol 41, pp 50-67
- 貢獻：point-line-plane 自適應融合——依場景紋理豐富度自動切換用 point-only 或 point+line/plane
- 資料集：TUM RGB-D、ICL-NUIM（皆為 RGB-D，非 stereo-inertial）
- 量化結果：對比 ORB-SLAM2 精度提升約 11.29%；速度比 PL(P)-VSLAM 快約 21.57%

### 5. DOE（Dynamic Object Elimination scheme）
- Connection Science (Taylor & Francis)，2023
- 貢獻：幾何約束＋語意約束整合進 ORB-SLAM3，鄰近幀之間的幾何一致性進一步剔除動態特徵點
- 量化結果（TUM 資料集，對比 ORB-SLAM3）：ATE 最高改善 99.01%，RPE 最高改善 95.12%
- 注意：全文被 403 擋下，數字僅來自搜尋摘要，未讀原文查證細節（資料集子集、是否所有序列都達到此數字未知）

### 6. SuperPoint-SLAM3（重要：與使用者現有系統高度重疊，需提醒）
- arXiv:2506.13089（2025）
- 基於 ORB-SLAM3，貢獻：SuperPoint 取代 ORB＋adaptive NMS（均勻化關鍵點分布）＋NetVLAD 學習型迴環偵測
- KITTI Odometry：平移誤差 4.15%→0.34%
- **提醒使用者**：這篇跟 Rover-SLAM 現有架構（ORB-SLAM3 + SuperPoint + LightGlue GPU）非常像。「換學習型特徵＋改迴環」這個組合已經有人發表（雖然是 arXiv 非正式期刊），代表這條路線的「新穎性存量」正在被別人用掉，越晚投風險越高。

### 7. 醫院物流機器人改良 ORB-SLAM3
- Discover Applied Sciences (Springer)，2025
- 貢獻：多尺度金字塔＋自適應閾值＋雙閾值方法應對光照變化＋深度學習輔助

### 8. IRAF-SLAM
- arXiv:2507.07752（2025）
- 貢獻：非深度學習的影像前處理管線（Gaussian filtering → adaptive gamma correction → sharpening）＋自適應特徵剔除（illumination-robust adaptive feature culling）
- 符合使用者「非深度學習光照穩健前處理」的需求方向

### 9.（已移除）ADM-SLAM 原為 Sensors (MDPI) 來源，依排除規則刪除，未找到非 MDPI 替代版本。

### 10. ALGD-ORB
- PLOS ONE，2023（非 MDPI）
- 貢獻：自適應閾值＋local gray difference 特徵偵測＋改良 quadtree 均勻化分布
- 註：僅涉及特徵提取單一軸，非多重貢獻組合型論文，列在此作為「自適應特徵數量/分布控制」這條候選優化的佐證，不算入 Q1 的「組合型論文」計數。

### 11.（已移除）原引用 Applied Sciences (MDPI) 論文，依排除規則刪除。量化參考數字（ATE 降 13.88%）失去來源，不再引用。

## Q3：常見強健性資料集

- **TUM-VI**：手持 fisheye 立體＋IMU，涵蓋快速手持運動、AR/VR 場景；ORB-SLAM3 原論文報告在此資料集房間序列可達 9mm 精度（快速運動下）。
- **UMA-VI**：搜尋沒有找到直接相關結果，未查證，不確定其在 stereo-inertial 強健性論文中的使用頻率。
- **Hilti SLAM Challenge Dataset**（arXiv:2109.11316，IEEE RAL）：室內辦公室/實驗室＋室外工地/停車場，無紋理區域與光照劇烈變化。搜尋結果證實 ORB-SLAM2 在部分序列完全失敗；「degenerate and dynamic scenes 中 SOTA 演算法普遍效能大幅下降」，但沒有找到指名 ORB-SLAM3 具體哪個序列失敗的直接證據。
- **4Seasons**（IJCV 2024，arXiv:2301.01147）：跨季節自動駕駛，350+ km，日夜與天氣變化。證實：ORB-SLAM3 不開 IMU 在夜間會直接失敗；開 IMU 時夜間表現優於 Basalt（光流法），但仍有明顯效能下降。

## Q4：實驗章節標配（歸納，非直接引用）

從以上論文歸納出的共同模式（**這是我的歸納，不是某篇論文的明文規定**）：
1. 對比表：至少對比原始 baseline（ORB-SLAM2 或 ORB-SLAM3）＋ 1-2 個同類 SOTA 方法。
2. 指標：ATE RMSE 為主，常搭配 RPE（平移＋旋轉）；跨資料集多序列列成表格，附百分比改善。
3. 消融表：每個提出的模組單獨開關做排列組合（如 DynaTM-SLAM／YoloV8-SLAM 把偵測模組與幾何模組分開驗證），證明每個模組都有貢獻。
4. 即時性表：每幀處理時間（ms）或 fps，證明沒有為了精度犧牲即時性太多。
5. 定性圖：估計軌跡 vs ground truth 疊圖，或動態場景的特徵點剔除視覺化。
6. 部分論文（如涉及室內/低紋理場景）額外附上失敗案例分析（baseline 在哪個序列直接跟丟）。
