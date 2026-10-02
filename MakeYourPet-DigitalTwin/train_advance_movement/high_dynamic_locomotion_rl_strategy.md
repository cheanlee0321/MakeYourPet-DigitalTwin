# 六足機器人高動態運動與跳躍強化學習策略分析
## 殘差網路在極限動作中的影響、瓶頸與架構演進方向

---

## 1. 摘要與現狀回顧 (Executive Summary)

在當前的「Make Your Pet Hexapod」專案中，步態控制器廣泛採用了**殘差強化學習 (Residual Reinforcement Learning)**：
$$\mathbf{a}_t = \mathbf{a}_{\text{ref}}(t) + \Delta \mathbf{a}_t$$
其中 $\mathbf{a}_{\text{ref}}(t)$ 是由預編程軌跡（Pre-programmed Nominal Gait，如三角步態逆運動學或跳躍 FSM）提供的基準關節目標，而神經網路則學習補償殘差 $\Delta \mathbf{a}_t$ 以適應地面接觸、動態震動與姿態微調。

當面臨以下更高階的高動態目標時：
1. **極限高跳 (Max-Height Jumping)**
2. **高速移動 (High-Speed Sprinting / Gait Transition)**
3. **跑動中起跳 (Running Jump / Parkour / Dynamic Obstacle Clearing)**

**核心問題**：*我們是否需要移除殘差學習中的參考條件 $\mathbf{a}_{\text{ref}}$？*

**結論**：
* **平地巡航與微調**：保留殘差架構具備極高的樣本效率與安全性。
* **跳更高與高速移動**：不一定立刻完全移除，但必須**解除參數與動作幅度的硬約束**。
* **跑動中起跳等跨拓撲動作**：強烈建議**移除單一連續步態參考**，轉向**分層專項技能 (Hierarchical Skill Switching)** 或 **端到端強化學習 (End-to-End RL)**。

---

## 2. 殘差架構在高動態運動中的雙面刃分析

### 2.1 殘差學習的優勢（為什麼現階段成功）
* **探索空間壓縮**：預先鎖定了合理的六足相鄰腿支撐相位，避免了網路在初期無效踢腿、摔倒或自我碰撞。
* **物理安全約束**：將關節動作限制在正常工作空間，保護實體舵機與連桿。
* **超高樣本效率**：在數萬步內即可收斂出平穩走姿與受控起跳。

### 2.2 在極限高動態下的三大瓶頸（為什麼會成為限制）

```
[預編程參考軌跡 a_ref]
        │
        ├──> (1) 拓撲衝突：週期性步態 vs 非週期瞬時爆發
        ├──> (2) 動作截斷：Action Clipping 限制了極限力矩與衝程釋放
        └──> (3) 局部最優：誘導偏置 (Inductive Bias) 形成「重力井」，阻礙全域最佳發力
```

1. **相位與動作拓撲衝突 (Topological & Phase Conflict)**：
   * 行走是**週期性、連續接地的滾動平滑運動**。
   * 跳躍則是**非週期性、高階瞬態爆發**（深蹲蓄力 $\to$ 全力伸展蹬地 $\to$ 騰空伸足迎地 $\to$ 屈膝吸震緩衝）。
   * 若強行在週期性步態參考上疊加殘差，神經網路必須花費大量權重來「抵抗並覆蓋」預設指令，反而造成訓練失穩或能量浪費。
2. **殘差範圍截斷限制 (Action Clipping Bottleneck)**：
   * 為了維持穩定，通常將殘差截斷在較小區間（例如 $\Delta a \in [-0.2, 0.2]\ \text{rad}$）。
   * 但極限起跳需要關節瞬時打到最大極限角與最高動態做功衝程（如 Femur/Tibia 大行程伸展），硬約束會直接鎖死機器人的爆發高度上限。
3. **歸納偏置引發的局部最優 (Inductive Bias Trap)**：
   * 預編程的軌跡如同一個「力學重力井」。神經網路傾向在預設動作附近進行局部梯度下降，很難主動探索出動態力學上更優秀但反直覺的動作（例如：利用前後腿延遲蹬伸以增加騰空俯仰穩定性）。

---

## 3. 三大高動態行為的具體訓練策略

### 3.1 極限跳高 (Max-Height Jumping)

六足機器人要跳得更高，核心在於**垂直衝量最大化**：
$$J_z = \int_0^{t_{\text{liftoff}}} (F_{z,\text{total}} - mg) \, dt$$

* **瓶頸分析**：目前基於 FSM 的深蹲-蹬地時序（如 $t_{\text{crouch}}=0.16s, t_{\text{thrust}}=0.10s$）是人為設定的固定切換點，並非動力學上的最優做功時序。
* **策略演進**：
  1. **動態時間扭曲 / 變時序 (Variable Timing)**：允許網路不僅輸出關節角，還能控制各階段的爆發時長與蹬地切換點。
  2. **端到端無參考跳躍 (End-to-End Jump Policy)**：
     * **觀測空間**：當前關節角速度、IMU 線加速度、底盤高度、足端接觸感測。
     * **獎勵設計**：
       $$R_{\text{jump}} = w_1 \cdot h_{\text{apex}} + w_2 \cdot (\mathbf{v}_z^{\text{liftoff}})^2 - w_3 \cdot \|\boldsymbol{\omega}_{\text{air}}\|^2 + w_4 \cdot R_{\text{landing\_stability}}$$
     * **課程學習 (Curriculum)**：從目標高度 20cm 逐步加碼至 60cm、80cm，避免初期因落地摔毀而得不到正向回饋。

---

### 3.2 高速移動 (High-Speed Sprinting)

要突破六足機器人的傳統三角步態速度極限：

* **瓶頸分析**：傳統預編程的步態頻率 $f$ 與步幅 $L$ 是固定的。當速度要求超過臨界點，三角步態（3 腿支撐、3 腿擺動）會引發嚴重的地面衝擊與機身震顫。生物六足昆蟲在高速奔馳時會自然過渡到二足彈跳（Bipedal Running）或滑跑步態。
* **策略演進**：
  1. **相位條件化殘差 (Phase-Conditioned Residuals)**：
     * 讓神經網路具備修改步頻 $f_t$ 與左右步相位的自由度，而不僅僅是關節角偏差。
  2. **完全端到端速度跟隨 (End-to-End Velocity Tracking)**：
     * 輸入目標速度向量 $\mathbf{v}_{\text{cmd}} = [v_x, v_y, \omega_z]$。
     * 逐步提升指令速度（$0.2\ \text{m/s} \to 0.8\ \text{m/s} \to 1.5\ \text{m/s}$）。
     * 引入足端滑移懲罰（Penalize Foot Slip）與觸地衝擊懲罰，引導網路自發湧現出高步頻、帶有微小騰空相（Flight Phase）的高速步態。

---

### 3.3 跑動中起跳 (Running Jump / Parkour)

這是難度最高的高動態動作，需要處理**水平動量向垂直動量的轉化**：
$$\frac{1}{2} m v_x^2 + \frac{1}{2} m v_z^2 \implies \text{最大化飛行距離與過障高度}$$

* **瓶頸分析**：在高速跑動時，如果直接切入靜態的深蹲蓄力，六足會因為慣性向前翻滾（Trip & Pitch Over）。必須在起跳前 1~2 步進行「步點調整（Stutter Step）」與傾角預補償。
* **策略演進**：
  1. **不應使用固定軌跡殘差**：此處預編程幾乎不可能人工設計出適應各種跑速的銜接曲線。
  2. **雙階段預備動作 (Run-Up & Plant Phase)**：
     * 前足主動前伸制動（Plant / Braking Step），將水平衝量轉化為抬頭力矩。
     * 中後足全力後蹬做功。
  3. **強化學習實現方式**：
     * 採用**視覺/障礙物距離引導 (Terrain/Obstacle-Aware Policy)**。
     * 給予起跳觸發信號或依據前方障礙物邊界的距離，由網路自主決定「在第幾步踏出制動腳、在第幾步合力起跳」。

---

## 4. 四大架構演進技術路線對比

| 方案路線 | 核心架構 | 優點 | 缺點 / 代價 | 推薦適用場景 |
| :--- | :--- | :--- | :--- | :--- |
| **路線 A：完全端到端強化學習 (End-to-End RL)** | 移除 $a_{\text{ref}}$，直接輸出 PD Target，依賴獎勵與 Curriculum | 極限性能上限最高，能發現物理最佳解 | 訓練極易局部發散，需要嚴格的領域隨機化與課程設計 | 極限跳高、跑酷跨障、極速衝刺 |
| **路線 B：分層專項技能庫 (Hierarchical FSM + Policy)** | 平常保留殘差走姿；收到跳躍/衝刺指令時，切換至端到端專項 Policy | 工業落地最穩健，不影響平地的高安全巡航穩定性 | 狀態切換瞬間（Transition）需要平滑過渡控制 | 實際產品落地、多模態行為切換 |
| **路線 C：粗糙/參數化軌跡引導 (Coarse Parameterized Reference)** | 僅提供低精度的粗糙相位草圖（蓄力-爆發-收腿），殘差負責全身協調 | 訓練收斂極快，保留殘差的引導效果 | 動作上限仍受限於粗糙草圖的時間切分 | 研發初期的跳躍功能快速原型驗證 |
| **路線 D：殘差退火機制 (Residual Annealing / Warm-up)** | $a_t = \alpha_k a_{\text{ref}} + a_{\text{RL}}$，隨訓練週期將 $\alpha_k \to 0$ | 解決端到端冷啟動摔倒問題，平滑過渡到自由探索 | 退火曲線需要仔細調參，可能在斷奶期出現回縮 | 想追求端到端上限但初期探索困難 |

---

## 5. 針對「Make Your Pet Hexapod」的具體實施路徑 (Actionable Roadmap)

### Phase 1：現有殘差架構極限壓榨（短期實踐）
* **放寬殘差上限 (Action Scaling)**：將起跳期間的 $\Delta a$ 限制從 $\pm 0.2\ \text{rad}$ 逐步放大至全量程（或移除 Clipping，僅保留物理 Joint Limit）。
* **參數化預編程 (Parameterized Reference)**：將 `jump_controller.py` 中的 `power`、蓄力時長 `t_crouch` 與蹬地時長 `t_thrust` 改為可被 RL Policy 動態調節的參數（RL controls parameters of the trajectory generator）。

### Phase 2：分層技能切換架構（中期推薦）
* 保持平地巡航使用優秀的**殘差行走 Policy**。
* 開闢獨立的 `hexapod_jump_env.py`，訓練專門的 **End-to-End High Jump Policy**（完全移除 $a_{\text{ref}}$，全靠獎勵引導關節輸出）。
* 構建切換過渡控制器（Blender / Interpolator），確保由行走狀態切入起跳狀態時力矩不突波。

### Phase 3：跑動中起跳與跑酷（長期挑戰）
* 引入前方障礙感知（高度、距離標籤或 Elevation Map）。
* 訓練具備預備步態調節能力（Run-up adjustment）的綜合運動模型，自主決定跑動轉起跳的發力點。

---

> **結論摘要**：
> 殘差學習是通往「穩健行走」的最佳跳板，但在通往「極限爆發與跑酷」的道路上，預編程參考會逐漸轉變為枷鎖。
> **未來的演進不是非黑即白地全盤拋棄，而是走向「平地殘差保障穩定，極限技能端到端釋放潛能」的分層融合體系。**
