# evo 軌跡評估指令速查

`evo` 是 SLAM 軌跡評估的標準工具，用來比對「你的估計軌跡」和「ground truth」，計算 ATE / RPE 等誤差，並檢查軌跡的完整性（配對點數、時間範圍）。

---

## 安裝

```bash
pip install evo --upgrade
```

---

## 1. evo_traj —— 先看兩條軌跡各自的「數量、時長」

```bash
evo_traj tum your_trajectory.txt --ref groundtruth.txt
```

輸出會列出每條軌跡的統計：

```
your_trajectory.txt
    poses: 1487              # 你的軌跡有幾個位姿
    path length: 58.3m
    duration: 74.2s          # 時間範圍
    t_start / t_end: ...     # 起訖時間戳

groundtruth.txt
    poses: 14820             # GT 有幾個位姿（通常多很多）
    duration: 74.5s
```

**一眼對比兩者的位姿數和時長** —— 如果你的 duration 明顯短於 GT，或 poses 少一大截，就代表軌跡有缺失。

視覺化看軌跡有沒有斷掉：

```bash
evo_traj tum your_trajectory.txt --ref groundtruth.txt --plot
```

---

## 2. evo_ape —— 算 ATE 誤差，用 -v 看「配對了幾個點」

```bash
evo_ape tum groundtruth.txt your_trajectory.txt -va --plot
```

關鍵是 **`-v`（verbose）**，它會印出配對資訊：

```
--------------------------------------------------------------------------------
Found 1487 of maximum 1487 possible matching timestamps between...
with max. time diff 0.01 (s) and time offset 0.0 (s).
--------------------------------------------------------------------------------
Compared 1487 absolute pose pairs.       # 實際配對了幾對
...
APE w.r.t. translation part (m)
      rmse    0.052341                    # ATE RMSE（你要的誤差）
      mean    0.048...
```

- **`Found X of maximum Y matching timestamps`**：配對成功數 / 可能的最大配對數
- **`Compared X absolute pose pairs`**：實際算誤差用了幾對

> **這行是抓 cherry-pick（只存準的點）的關鍵** —— 如果軌跡被砍到只剩少數點，這裡的數字會很小，一看就知道。

### 各 flag 意思

| flag | 作用 |
|------|------|
| `-v` | verbose，印出配對細節（**最重要**） |
| `-a` | align，用 Umeyama 對齊座標系 |
| `-s` | align + scale，單目要加（有尺度不確定性） |
| `--plot` | 畫圖 |
| `--t_max_diff` | 時間戳配對容忍度（秒），預設 0.01 |

---

## 3. 控制時間戳配對的容忍度

```bash
evo_ape tum groundtruth.txt your_trajectory.txt -v --t_max_diff 0.01
```

- `--t_max_diff 0.01`：時間戳差在 0.01 秒內才算配對成功
- 設太小 → 配對數變少；設太大 → 可能配到不對的點
- 預設 0.01 秒通常合理

---

## 4. 完整檢查「覆蓋率」的組合指令

要判斷軌跡有沒有缺失，跑這兩個對照：

```bash
# 先看兩條軌跡的原始規模
evo_traj tum your_trajectory.txt --ref groundtruth.txt

# 再看實際配對數 + 誤差
evo_ape tum groundtruth.txt your_trajectory.txt -va --plot
```

**判斷方法：**

```
GT 時長 74.5s，你的軌跡時長只有 40s？   → 缺了一半，可疑
GT 有 14820 點，配對只成功 800 對？      → 你的軌跡太稀疏/缺失
```

---

## 5. EuRoC 格式（GT 常是 euroc 格式）

EuRoC 的 ground truth 是特殊格式（`data.csv`），格式參數改成 `euroc`：

```bash
evo_ape euroc groundtruth_data.csv your_trajectory.txt -va
```

或先轉格式：

```bash
evo_traj euroc groundtruth_data.csv --save_as_tum   # 轉成 tum 格式
```

---

## 實務建議的檢查流程

```bash
# 1. 看規模對不對（抓缺失）
evo_traj tum your_traj.txt --ref gt.txt

# 2. 算誤差 + 看配對數（-v 是關鍵）
evo_ape tum gt.txt your_traj.txt -va --plot
```

檢查輸出裡的：

- `Found X of maximum Y` → 配對率
- `Compared X pairs` → 實際比較點數
- `duration` 對不對 → 有沒有缺一段

---

## 補充：為什麼要看配對數 / 覆蓋率

ATE 的意義是反映系統在**整段軌跡**的真實表現。如果只保留「確定準」的位姿（cherry-picking），ATE 會變小，但那不代表系統的真實表現 —— 等於考試撕掉答錯的題目。

透過 `evo_traj` 的位姿數/時長對比、`evo_ape -v` 的配對點數，就能檢查軌跡是否完整、有沒有藏掉跟丟或漂移大的片段。這也是複現/檢驗他人結果時該用的方法。

---

## 常見軌跡格式參考

| 格式 | 說明 | 每行內容 |
|------|------|----------|
| `tum` | TUM 格式（最常用） | `timestamp tx ty tz qx qy qz qw` |
| `euroc` | EuRoC ground truth | `timestamp,px,py,pz,qw,qx,qy,qz,...`（csv） |
| `kitti` | KITTI 格式 | 3×4 位姿矩陣攤平（無時間戳） |
