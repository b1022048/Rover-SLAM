# IMU 預積分學習筆記（IntegrateNewMeasurement 完整攻略）

> 整理自 2026-07-07 的學習對話。行號以「加完中文註解後」的 `src/ImuTypes.cc` 為準（會漂移，查證請用可 grep 的符號）。
> 文獻：Forster et al., *On-Manifold Preintegration for Real-Time Visual–Inertial Odometry*, IEEE T-RO 2017（下稱 Forster；附錄 A＝Eq 59-63、附錄 B＝bias 一階修正）。

---

## 第一部分：先備知識（六堂課）

### 第 0 課：IMU 量什麼
- 陀螺儀：角速度 ω（rad/s，機體系）。加速度計：**比力**＝真實加速度−重力（機體系）——靜止讀 9.8 朝上、自由落體讀 0。這決定了「預積分不碰重力、使用端才補」。
- 頻率：IMU 200Hz vs 相機 20Hz → 兩幀之間夾約 10 筆 IMU，`IntegrateNewMeasurement` 每筆呼叫一次。
- 參數 `dt` 是**相鄰兩筆 IMU** 的間隔（≈5ms），不是幀間隔（`ImuTypes.cc:245` 的註解誤導；證據：`Reintegrate()` 逐筆傳 `aux[i].t`）。

### 第 1 課：量測模型
讀值＝真值＋bias＋白雜訊。bias 慢變（random walk），是待估計量；白雜訊扣不掉，只能追蹤不確定度。對應 `Nga`（白雜訊強度）/`NgaWalk`（bias 游走強度），詳見 GLOSSARY。

### 第 2 課：旋轉不能用加法——軸角、hat、Exp
- **軸角向量** `φ = θ·u`：方向＝轉軸、長度＝角度。ω 本身就是同一種打包（方向＝瞬時轉軸、長度＝轉速），dt 內視 ω 不變 → 這段的旋轉軸角＝`ω·dt`。
- **hat**：`hat(v)` 把 3 維向量排成反對稱矩陣 `[[0,−z,y],[z,0,−x],[−y,x,0]]`，使 `hat(a)b = a×b`。目的不是「算叉積」（Eigen 有 `cross()`），是讓叉積**取得矩陣身分**：能自乘（W²）、能塞進大矩陣、能把 δφ 提出來求 Jacobian。
- **Exp（Rodrigues）**：`Exp(φ) = I + (sinθ/θ)W + ((1−cosθ)/θ²)W²`（`IntegratedRotation` 建構子，ImuTypes.cc:149 附近）。小角度退化為 `I+W`。
- 陀螺積分＝`R ← R·Exp((ω−bg)dt)`，右乘＝接在機體系。
- 名詞：叉積＝台灣教材「外積」a×b（輸出向量）；英文 outer product 是 abᵀ（輸出矩陣），勿混。

### 第 3 課：right Jacobian（Jr）
`Exp(φ+δ) ≈ Exp(φ)·Exp(Jr(φ)·δ)`——「軸角加法小量 → 群上乘法小量」的匯率。出場兩處：B(0,0)=Jr·dt（陀螺雜訊換算進旋轉誤差）、JRg 遞推。

### 第 4-5 課：為什麼要「預積分」
- 普通積分依賴初始世界狀態；後端優化每輪迭代都改狀態 → 每改一次要重積所有 IMU，代價爆炸。
- Forster 解法：定義只依賴讀數與 bias 的相對量 **ΔR/ΔV/ΔP**（＝程式的 dR/dV/dP）——「以第 i 幀機體系為臨時世界」的積分。連接公式（使用端才出現重力）：
  `v_j = v_i + gΔt + R_i·ΔV`、`p_j = p_i + v_iΔt + ½gΔt² + R_i·ΔP`。
- bias 改變用一階修正（五個 Jacobian：JRg/JVg/JVa/JPg/JPa＝∂Δ量/∂bias），太大（>0.01）才 `Reintegrate()`。

### 第 6 課：不確定度傳遞
誤差狀態 η=[δφ,δv,δp]（9 維），逐筆線性遞推 `η_new = A·η_old + B·n`，協方差 `C ← A·C·Aᵀ + B·Nga·Bᵀ`（線性變換的變異數規則）。C 最終＝後端優化中這條 IMU 約束的權重（ORB-SLAM3 論文公式 4 的 Σ）。
**重點**：η 本身永遠不可知（知道就扣掉了），被傳遞的是它的分布 C；A/B 存的是「係數/規則」，不是誤差值。

---

## 第二部分：IntegrateNewMeasurement 逐段（ImuTypes.cc:247-338）

執行順序＝「遞推公式右邊一律用段初值」這條紀律的展開：

| 步驟 | 行（快照） | 內容 |
|---|---|---|
| 存原料 | :250 | `integrable(acc,ω,dt)` 打包存進 `mvMeasurements`（供 Reintegrate/MergePrevious 回放；純資料 struct，無運算） |
| 扣 bias | :281-282 | `acc`＝扣 bias 的加速度；`accW` 名字誤導（是角速度，只供 avgW） |
| 統計 | :285-286 | avgA（乘 dR 翻共同座標系再加權平均）、avgW（不乘，見下） |
| 更新均值 | :290-291 | `dP += dV·dt + ½dR·acc·dt²`；`dV += dR·acc·dt`（都用**舊 dR**——順序即公式） |
| 填 A/B 平移格 | :295-301 | Wacc=hat(acc)；五個格子＝Eq(60)(61) 的係數（見第三部分） |
| bias Jacobian | :306-309 | JPa/JPg/JVa/JVg 遞推（Forster 附錄 B） |
| 更新旋轉 | :313-315 | `dRi=Exp((ω−bg)dt)`（Rodrigues）；`dR ← Normalize(dR·dRi)`（SVD 拉回正交） |
| 填 A/B 旋轉格 | :319-320 | `A(0,0)=dRiᵀ`、`B(0,0)=Jr·dt`（Eq 59 的係數；必須等 dRi 出生） |
| 協方差 | :325-327 | `C ← A·C·Aᵀ + B·Nga·Bᵀ`（Eq 63）；bias 塊 `+= NgaWalk` |
| JRg | :333 | `JRg ← dRiᵀ·JRg − Jr·dt`（回答「為何最後更新」：dV 遞推用舊 JRg 本來就正確） |
| 總時間 | :337 | `dT += dt` |

一句話總結：每收一筆 IMU 做三件平行的事——遞推**均值**（dR/dV/dP）、遞推**不確定度**（A/B→C）、遞推**bias 斜率**（J 系列），三者用同一組線性化（所以 `−dR·dt·Wacc` 在 A 和 JVg 裡長一樣）。

### dR 專題（最容易迷路的部分）
- **身分**：段落內的「旋轉里程表」——出發歸 I、路上當翻譯機（`dR·acc` 把機體系讀值翻回基準系）、到站當交付品（`R_j = R_i·dR`）。
- **基準**＝「此物件被 new/Initialize 的那一瞬間」的機體姿態。幀對幀積分器（Tracking.cc 的 `new IMU::Preintegrated(mLastFrame.mImuBias,…)`，PreintegrateIMU 內）基準＝上一**影像幀時刻**（不是第一筆 IMU——邊界用內插切齊）；KF 累積器（mpImuPreintegratedFromLastKF）基準＝上一關鍵幀時刻。
- **dR=I 不是錯**：起點那一刻「現在的機體系＝基準系」，零旋轉是唯一正確值（里程表歸零）。
- **不跨段繼承**：段落間銜接靠幀/KF 的世界姿態，dR 每段從 I 重來；bias 才是被繼承的（拿上一幀的當初值）。
- **左乘 vs 右乘**（只在旋轉×旋轉時有意義）：右乘＝接在鏈的機體端（輸入用機體系語言）、左乘＝接在基準端（輸入用基準系語言）。恆等式 `dR·Exp(ω·dt) = Exp((dR·ω)dt)·dR`（由換座標引理 `R·Exp(φ)·Rᵀ=Exp(Rφ)` 推出）。`dR*acc` 是矩陣作用於向量——沒有左右問題（向量只能被左乘），要選的是方向（dR vs dRᵀ）。

### avgW 為什麼不乘 dR
1. **數學的幸運**：角速度向量的方向＝轉軸，而旋轉不動自己的軸（`Exp(uθ)u=u`，±u 同一條線都不動）→ 只要段內轉軸線穩定，機體系與基準系讀出同一向量。繞圈（ω 恆指天上）、8 字形（+z↔−z 同軸線）、原地掉頭全都零誤差；平面運動（地面車）永遠精確。會失效的只有三維翻滾（軸線改向），50ms 內做到需要 3600°/s，物理上不可能。
2. **用途的不講究**：avgW 在本 repo **沒有任何消費者**（grep 驗證過）；avgA 唯一消費者是 Tracking.cc 的初始化激勵檢查（norm 差 ≥0.5），且 avgA 有乘 dR、是對的。
- avgA 乘 dR 的反例：靜止原地自轉，讀值方向隨機身掃動，不翻譯平均會抵消成零、騙過激勵檢查。
- avg 公式本身＝時間加權平均的遞推寫法：`avg_新 = (dT·avg_舊 + 貢獻·dt)/(dT+dt)`（dT 此刻尚未 += dt，恰為舊總時間）。

---

## 第三部分：A、B 矩陣完全解析

`η_new = A·η_old + B·n`。列（收款人）都是狀態：0-2=δφ、3-5=δv、6-8=δp。
**行（付款人）不同**：A 的行是狀態（9×9）；B 的行是雜訊（9×6）：0-2=ng（陀螺）、3-5=na（加計）。
（`B.block(3,3)` 不是「速度對速度」——B 的行 3 是 na 的第一分量！）

```
A =  ⎡  dRiᵀ            0      0 ⎤   :319   舊旋轉誤差換座標到新機體系（Eq 59）
     ⎢ −dR·dt·Wacc      I      0 ⎥   :297   姿態歪→加速度轉錯→滲進速度（Eq 60）
     ⎣ −½dR·dt²·Wacc  dt·I     I ⎦   :298,:299   同上進位置；δv·dt→δp（Eq 61）

B =  ⎡  Jr·dt      0     ⎤   :320   陀螺雜訊×匯率→旋轉誤差（Eq 59）
     ⎢   0       dR·dt   ⎥   :300   加計雜訊搭 dR·acc·dt 便車→速度（Eq 60）
     ⎣   0      ½dR·dt²  ⎦   :301   同上走 ½dt² 通道→位置（Eq 61）
```

- 對角線 I 來自 `setIdentity`（舊誤差原樣保留）；A 右上 0＝因果鏈單向「旋轉→速度→位置」（下三角）；B 左下 0＝陀螺雜訊當步只進 δφ，下一步才經 A 間接滲透。
- `block<3,3>(r,c)`：3×3 子塊，**(r,c) 是左上角**（非中心），等號＝整塊覆蓋（非乘上原值）；`DiagonalMatrix(dt,dt,dt)`＝dt·I 的建構寫法。
- 為何 :319-320 不跟 :297-301 寫在一起：297-301 必須在 dR 更新（:315）**前**（要用舊 dR）；319-320 必須在 dRi 建構（:313）**後**。技術上可把 dRi 提前合併（等價），但會犧牲「dR 更新點前皆舊值」的可審計紀律。

### 推導（所有格子的來源）
手法：真值版更新式 − 估計版更新式，工具三件（`Exp(δ)≈I+hat(δ)`、`hat(a)b=−hat(b)a`、二階小量捨去）。以 δv 為例：
```
dV_真new = (dV+δv) + dR·Exp(δφ)·(acc−na)·dt
→ δv_new = δv + (−dR·dt·Wacc)·δφ + (dR·dt)·na      ← 三個係數對號入座
```
δp 同法（多 δv·dt 項、係數換 ½dt²）。B 的正負號無所謂（進 B·Nga·Bᵀ 平方掉；Forster 的雜訊慣例直接給正號，與程式碼一字不差）。

### 與論文的對照（Forster 附錄 A，p.15）
論文手法：總和形式 → 拆出最後一項 → 認出遞推（「Iterative Noise Propagation」）。程式碼只實作遞推那行，逐筆呼叫自然滾出總帳。

| Forster | 管什麼 | 程式行（快照） |
|---|---|---|
| Eq (59) | δφ 遞推 | :319 + :320 |
| Eq (60) | δv 遞推 | :297 + :300 |
| Eq (61) | δp 遞推 | :298、:299、:301 |
| Eq (62) | η=Aη+Bn 矩陣形式 | （概念） |
| Eq (63) | Σ=AΣAᵀ+BΣηBᵀ | :325（初始 Σ_ii=0 ↔ Initialize 的 C.setZero） |
| 附錄 B Eq(64-69) | bias 一階修正 | :306-309、:333、GetDelta*（:402-441） |

### 與 ORB-SLAM3 論文（J21 PDF）的關係
- 公式 2＝**慣性殘差**（消費端）：三行分別把連接公式移項，`R_iᵀ` 把世界座標差翻回第 i 幀機體系（dR 的基準），**重力 g 在此出現**。實作：`EdgeInertial::computeError`（G2oTypes.cc）。
- 公式 4 的 `‖r‖²_{Σ⁻¹}`：Σ＝A/B 攢出來的 C——積越久越不準、殘差話語權越低。
- 上下游一句話：`IntegrateNewMeasurement` 是工廠（均值＋不確定度＋bias 斜率三條生產線），公式 2 吃均值（經 GetDelta* 做 bias 修正）、公式 4 吃 C。

---

## 第四部分：資料流（vImuMeas → 預積分）

- 主程式（stereo_inertial_euroc.cc）：`vImuMeas` 每幀 `clear()` 後裝「(上一幀, 當前幀] 的 IMU」，EuRoC 200/20Hz → 每幀 ~10 筆（首幀 0 筆）。**不會隨時間變大**（clear ＋ 游標單向前進）。
- `first_imu[seq]`＝每序列一個的**書籤**（陣列下標），什麼都不知道；「對應哪一幀」由「兩串都按時間排序＋書籤只前進」的雙指標掃描自動湧現——while 結束時書籤剛好停在下一幀的第一筆。
- 載入時「找第一幀前最近的 IMU」（:118-120）：①丟掉相機開錄前的積壓（無幀可錨定）；②`first_imu--` 刻意留幀界左側那筆當**內插原料**（PreintegrateIMU 的 tini 補償需要）。
- 多序列（seq≥1）：所有容器與游標都以 `[seq]` 索引、完全隔離，行為同 seq=0；序列交界不接 IMU（多地圖機制的入口）。
- 「IMU 時間戳憑什麼跟幀時間戳比較」：EuRoC 硬體同步（實測 cam0/imu0 第一筆時間戳**逐奈秒相同**）；9-11 筆不是被保證的，是取樣率算出來的；下游對筆數不敏感（逐段算 tstep，n==0 有防呆）。**自錄資料必須先做時間同步標定（如 Kalibr），否則錯得無聲**。
- Tracking 端雙保險：每筆自帶時間戳，`PreintegrateIMU` 用前後幀時間戳重新切窗（早於上一幀−1ms 丟、區間內收、到達當前幀收尾筆後 break 留餘給下一幀）。
- 呼叫鏈到 Initialize：`Track()` → Step4 `PreintegrateIMU()` → `new IMU::Preintegrated(...)`（每幀）→ 建構子 → `Initialize()`（dR=I 錨定基準）。KF 累積器只在建新 KF 時重建。

---

## 第五部分：怎麼驗證公式不是編的（四道防線）

1. **溯源**：Forster T-RO 2017（程式註解自標「第 63 個公式」）；GTSAM/VINS-Mono/OKVIS 獨立實作同一公式可交叉比對。
2. **自己推**：只需三個可手算的工具（見第三部分推導）。
3. **數值實驗**（最強）：`verify_preintegration.py`（repo 根目錄）——①hat vs numpy cross（誤差 1e-15）②Rodrigues vs scipy expm（1e-15）③Jr 恆等式二次收斂（δ 縮 10 倍誤差縮 100 倍）④蒙地卡羅 2 萬次 vs A/B 傳遞的 C：九個對角分量比值 0.976-1.009，落在統計漲落內。改參數可重跑。
4. **對 AI 回答的通用防護**：要求 file:line 並點開驗證、要求標 [已驗證]/[推測]、數字公式要求跑給你看。

---

## 常用符號速查

| 程式 | 論文 | 意義 |
|---|---|---|
| dR, dV, dP | ΔR̃_ij, Δṽ_ij, Δp̃_ij | 預積分均值（第 i 幀機體系為基準） |
| Wacc | (ã−bᵃ)^∧ | 扣 bias 加速度的 hat 矩陣 |
| dRi / dRi.rightJ | Exp((ω̃−bᵍ)Δt) / J_r | 這一步的小旋轉 / 其 right Jacobian |
| C | Σ_ij | 15×15 協方差（前 9 誤差態＋後 6 bias） |
| Nga / NgaWalk | Σ_η / — | 量測白雜訊 / bias 游走強度（離散變異數） |
| JRg,JVg,JVa,JPg,JPa | ∂Δ/∂b | bias 一階修正的斜率 |
| b / bu / db | b̄_i / b̂_i / δb | 線性化點 / 更新後 bias / 差量 |
