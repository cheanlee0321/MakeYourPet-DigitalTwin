# 🎮 Make Your Pet 六足機器人 Joystick 遙控規則控制表格手冊
### Complete Joystick & Gamepad Control Rules & Mapping Tables

> **專案儲存庫**：`Make Your Pet Digital Twin`  
> **涵蓋模組**：
> 1. **MuJoCo 3D 數位孿生工作台** (`MakeYourPet-DigitalTwin/demo.py`)
> 2. **雙手機戰術 Web 觸控遙控器** (`MakeYourPet-DigitalTwin-Server/pilot_web/app.js` & `pilot_server.py`)
> 3. **實體機 Wi-Fi 遙控器直連腳本** (`chica-server-main/tools/chica_gamepad_teleop.py`)  
> **更新日期**：2026-10-02  
> **版本**：v1.0 (全模組對齊)

---

## 📑 目錄

1. [三大遙控系統架構矩陣](#1-三大遙控系統架構矩陣)
2. [系統 A：MuJoCo 數位孿生工作台 (`demo.py`)](#2-系統-amujoco-數位孿生工作台-demopy)
   - [2.1 BetaFPV 穿越機遙控器對應表](#21-betafpv-穿越機遙控器對應表-mode-2)
   - [2.2 標準遊戲手把 (Xbox / PS / Switch) 對應表](#22-標準遊戲手把-xbox--ps--switch-對應表)
   - [2.3 速度檔位與姿態參數表](#23-速度檔位與姿態參數表)
   - [2.4 雙控混控與死區防抖規則](#24-雙控混控與死區防抖規則)
3. [系統 B：雙手機戰術 Web 觸控遙控系統 (`pilot_web` & `pilot_server`)](#3-系統-b雙手機戰術-web-觸控遙控系統-pilot_web--pilot_server)
   - [3.1 雙虛擬搖桿操控表格](#31-雙虛擬搖桿操控表格)
   - [3.2 輔助戰術功能鍵表格](#32-輔助戰術功能鍵表格)
   - [3.3 封包格式與雙重 Deadman 看門狗規則](#33-封包格式與雙重-deadman-看門狗規則)
4. [系統 C：實體機 Wi-Fi 直連手把遙控 (`chica_gamepad_teleop.py`)](#4-系統-c實體機-wi-fi-直連手把遙控-chica_gamepad_teleoppy)
   - [4.1 搖桿軸向與速度檔位修飾表](#41-搖桿軸向與速度檔位修飾表)
   - [4.2 動作按鈕對應表](#42-動作按鈕對應表)
   - [4.3 安全煞車邏輯](#43-安全煞車邏輯)
5. [全系統橫向參數對比總表](#5-全系統橫向參數對比總表)
6. [手把調試與常見問題排除 (FAQ)](#6-手把調試與常見問題排除-faq)

---

## 1. 三大遙控系統架構矩陣

工作區中目前共有三個獨立但功能互補的 Joystick / 遙控器控制系統：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       Make Your Pet 遙控架構生態系                           │
└─────────────────────────────────────────────────────────────────────────────┘
       │                                │                                │
       ▼                                ▼                                ▼
【系統 A: 數位孿生工作台】       【系統 B: 雙手機戰術 Web】       【系統 C: 實體機直連手把】
• 核心：demo.py                 • 核心：pilot_web + pilot_server • 核心：chica_gamepad_teleop.py
• 介面：MuJoCo 3D 物理引擎      • 介面：HTML5 Canvas 戰術 HUD    • 介面：終端文字即時 HUD
• 設備：BetaFPV / USB 手把      • 設備：手機觸控虛擬雙搖桿       • 設備：Xbox / PS4 / PS5 手把
• 通訊：本地 Pygame 直接驅動     • 通訊：WebSocket 25Hz -> TCP   • 通訊：Wi-Fi TCP 18711 (25Hz)
• 輸出：67 維觀測 / 18 維動作   • 輸出：walkonnx: / walk2: 指令  • 輸出：walk2: / torque / sit
```

---

## 2. 系統 A：MuJoCo 數位孿生工作台 (`demo.py`)

本系統是**高動態模擬與 AI 步態驗證工作台**，支援使用實體鍵盤、標準電玩手把以及 **BetaFPV 穿越機遙控器 (ELRS / EdgeTX / OpenTX)**。鍵盤與搖桿**完全並行運作**。

### 2.1 BetaFPV 穿越機遙控器對應表 (Mode 2)

當偵測到名稱包含 `betafpv`、`radio`、`edgetx`、`opentx`、`taranis`、`elrs` 或 `frsky` 且軸數大於 4 時，自動啟動專屬撥桿對齊：

| 控制元件 | 實體通道 / 軸向 | 操作動作 | 控制行為與物理響應 | 數值範圍 / 單位 |
| :--- | :--- | :--- | :--- | :--- |
| **右搖桿 (上下)** | CH2 / Axis 1 (Pitch) | **推前** (Forward) | 線性無段前進速度 ($v_x$) | $0 \sim +v_{x,\max}$ (依檔位) |
| | | **拉後** (Backward) | 線性無段後退速度 ($v_x$) | $0 \sim -v_{x,\text{back}}$ |
| **右搖桿 (左右)** | CH1 / Axis 0 (Roll) | **推左** (Strafe L) | 側向左向平移橫移步態 ($v_y$) | $0 \sim +0.75 \cdot v_{x,\max}$ |
| | | **推右** (Strafe R) | 側向右向平移橫移步態 ($v_y$) | $0 \sim -0.75 \cdot v_{x,\max}$ |
| **左搖桿 (左右)** | CH4 / Axis 3 (Yaw) | **推左** (Turn Left) | 原地逆時針旋轉轉向 ($\omega_z$) | $0 \sim +\omega_{z,\max}$ rad/s |
| | | **推右** (Turn Right) | 原地順時針旋轉轉向 ($\omega_z$) | $0 \sim -\omega_{z,\max}$ rad/s |
| **搖桿回彈** | 雙搖桿 | **放手回彈置中** | **即刻煞車立定站穩** (死區過濾後指令歸零) | $v_x=0, v_y=0, \omega_z=0$ |
| **SA 撥桿 (左外)** | CH8 / Axis 7 (兩段) | **向下撥動** | **Reset**：將機器人姿態與座標重置回起點中央 | 邊緣觸發 ($\Delta > 0.4$) |
| **SB 撥桿 (左內)** | CH7 / Axis 6 (三段) | **撥至上方** ($\text{val} < -0.33$) | 切換至 **1 檔 ECO** (慢步微調/爬坡，1.0 Hz) | 狀態 1 |
| | | **撥至中間** ($-0.33 \le \text{val} \le 0.33$) | 切換至 **2 檔 NORMAL** (標準巡航，1.5 Hz) | 狀態 2 (預設) |
| | | **撥至下方** ($\text{val} > 0.33$) | 切換至 **3 檔 TURBO** (極速狂飆，2.5 Hz) | 狀態 3 |
| **SC 撥桿 (右內)** | CH6 / Axis 5 (三段) | **撥至上方** ($\text{val} < -0.33$) | 切換至 **一般基準姿態** (自然基準拱步，標準離地高度) | 狀態 1 |
| | | **撥至中間** ($-0.33 \le \text{val} \le 0.33$) | **無 Mapping** (未指派獨立動作，閒置保持當前姿態) | 狀態 2 (中立) |
| | | **撥至下方** ($\text{val} > 0.33$) | 切換至 **OFFROAD 越野挺身姿態** (超高離地挺身避障，大腿/小腿各 +10°) | 狀態 3 |
| **SD 撥桿 (右外)** | CH5 / Axis 4 | 撥動 | **空白保留** (未指派，安全閒置無動作) | N/A |

---

### 2.2 標準遊戲手把 (Xbox / PS / Switch) 對應表

當連接通用 PC USB/藍牙手把時，系統自動套用標準搖桿與動作按鈕映射：

| 按鍵 / 搖桿 | 實體編號 | 動作形式 | 功能描述 | 備註 |
| :--- | :--- | :--- | :--- | :--- |
| **左搖桿 (上下)** | Axis 1 (Pitch) | 類比推拉 | 前進 / 倒退 (推前值為負，程式自動反轉極性) | 含死區過濾 |
| **左搖桿 (左右)** | Axis 0 (Roll) | 類比推拉 | 左右側向平移 (Strafe) | 速度上限為前進之 75% |
| **右搖桿 (左右)** | Axis 3 (或 2) (Yaw) | 類比推拉 | 原地向左 / 向右轉彎旋轉 | 依手把型號自適應軸號 |
| **A 鍵 / ✕ 鍵** | Button 0 | 單擊 (上升緣) | **循環切換速度檔位**：1檔 $\rightarrow$ 2檔 $\rightarrow$ 3檔 $\rightarrow$ 1檔 | 循環循環換檔 |
| **B 鍵 / ○ 鍵** | Button 1 | 單擊 (上升緣) | **Toggle 越野挺身姿態**：底盤超高挺身 / 一般姿態互切 | 防連點單擊觸發 |
| **X 鍵 / □ 鍵** | Button 2 | 單擊 (上升緣) | **鏡頭跟隨切換**：開啟 / 關閉攝影機跟隨機器人軀幹 | Tracking ON / Free |
| **Y 鍵 / △ 鍵** | Button 3 | 單擊 (上升緣) | **重置起點**：姿態與環境座標重置回原點 | 同鍵盤 R 鍵 |
| **立定跳躍** | N/A | — | *未指派於手把* (手把操作專注行走，跳躍由鍵盤 5/6 鍵觸發防誤觸) | 避免手把誤碰摔機 |

---

### 2.3 速度檔位與姿態參數表

`demo.py` 定義了三檔速度配置與兩大姿態模式，搖桿輸入將在這些上限內進行**百分比線性縮放**：

#### 速度檔位 (`SPEED_PROFILES`)

| 檔位 | 檔位名稱 | 步態頻率 (`freq`) | 最大前進速度 (`vx`) | 最大後退速度 (`back`) | 最大轉向角速度 (`yaw`) | Shift 衝刺倍率 |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **1** | **ECO 慢步微調/爬坡** | 1.0 Hz | 0.15 m/s | 0.10 m/s | 0.35 rad/s | 1.35x |
| **2** | **NORMAL 標準巡航** (預設) | 1.5 Hz | 0.25 m/s | 0.18 m/s | 0.50 rad/s | 1.40x |
| **3** | **TURBO 極速狂飆** | 2.5 Hz | 0.45 m/s | 0.25 m/s | 0.75 rad/s | 1.20x |

#### 姿態模式 (`POSTURE_PROFILES`)

| 模式 | 姿態名稱 | 髖關節偏置 (`offset_hip`) | 小腿偏置 (`offset_tibia`) | 物理特性與適用場景 |
| :---: | :--- | :---: | :---: | :--- |
| **False** | **一般基準姿態** | $0.0^\circ$ (0.0 rad) | $0.0^\circ$ (0.0 rad) | 基準自然拱步，重心平穩，耗電低，巡航穩定 |
| **True** | **OFFROAD 越野挺身姿態** | $+10.0^\circ$ (0.1745 rad) | $+10.0^\circ$ (0.1745 rad) | 大腿與小腿下撐，大幅拉高機身底盤高度，越過碎石/台階 |

---

### 2.4 雙控混控與死區防抖規則

#### 1. 死區平滑公式 (Deadzone Filtering)
為避免搖桿微幅抖動造成機體原地抽搐，設定死區門檻 $\delta = 0.08$：
$$
v_{\text{filtered}} = \begin{cases} 0.0 & \text{若 } |v| < \delta \\ \text{sign}(v) \cdot \dfrac{|v| - \delta}{1.0 - \delta} & \text{若 } |v| \ge \delta \end{cases}
$$
當搖桿置中時，輸出嚴格為 0.0；越過死區後自 0.0 起平滑過渡至 1.0，無突跳感。

#### 2. 鍵盤與搖桿無縫混控規則
鍵盤按鍵（`↑`/`↓`/`←`/`→`）與搖桿信號在同一迴圈中進行最大絕對值仲裁（Max-Absolute Arbitration）：
```python
if abs(js_vx) > abs(target_vx):
    target_vx = js_vx
if abs(js_vy) > abs(target_vy):
    target_vy = js_vy
if abs(js_wz) > abs(target_wz):
    target_wz = js_wz
```
- 使用者可在按住鍵盤前進時隨時微撥手把轉向，兩者互不衝突干擾。

---

## 3. 系統 B：雙手機戰術 Web 觸控遙控系統 (`pilot_web` & `pilot_server`)

本系統是針對**雙手機野外自主作業**設計，操作者手持 Phone A 透過瀏覽器操作 HTML5 Canvas 觸控雙搖桿，透過 WebSocket 傳送至中繼端轉發實體機 (Phone B)。

### 3.1 雙虛擬搖桿操控表格

| 虛擬搖桿 | 觸控軸向 | 方向動作 | 物理控制量 | 計算公式 | 數值限制 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **左搖桿 (Left Stick)** | **垂直軸 (Y)** | **向上推** | 前進速度 ($v_x$) | $v_x = Y_{\text{norm}} \times 0.35 \times \text{SpeedGain}$ | 最大 $+0.35 \times \text{Gain}$ m/s |
| | | **向下拉** | 倒退速度 ($v_x$) | $v_x = Y_{\text{norm}} \times 0.35 \times \text{SpeedGain}$ | 最大 $-0.35 \times \text{Gain}$ m/s |
| | **水平軸 (X)** | **向左推** | 橫向平移左 ($v_y$) | $v_y = X_{\text{norm}} \times 0.20 \times \text{SpeedGain}$ *(需開 Crab)* | 最大 $+0.20 \times \text{Gain}$ m/s |
| | | **向右推** | 橫向平移右 ($v_y$) | $v_y = X_{\text{norm}} \times 0.20 \times \text{SpeedGain}$ *(需開 Crab)* | 最大 $-0.20 \times \text{Gain}$ m/s |
| **右搖桿 (Right Stick)** | **水平軸 (X)** | **向左推** | 原地左轉旋轉 ($\omega_z$) | $\text{Yaw} = -X_{\text{norm}} \times 0.65 \times \text{SpeedGain}$ | 最大 $+0.65 \times \text{Gain}$ rad/s |
| | | **向右推** | 原地右轉旋轉 ($\omega_z$) | $\text{Yaw} = -X_{\text{norm}} \times 0.65 \times \text{SpeedGain}$ | 最大 $-0.65 \times \text{Gain}$ rad/s |
| | **垂直軸 (Y)** | 上下推 | 空白保留 | N/A | 無動作 |
| **雙搖桿共通** | 徑向回彈 | **放手彈回中心** | **Deadman 煞車** | 手指脫離 Canvas 即刻發送 `stop` (`walkclear`) | 立即立定站穩 |

---

### 3.2 輔助戰術功能鍵表格

| 介面按鈕 | 功能標籤 | 操作方式 | 下發 Chica 指令 | 系統行為 |
| :--- | :--- | :--- | :--- | :--- |
| `btn-mode-onnx` | **ONNX AI 步態** | 點擊切換 | `onnx on` | 切換為 18-DOF 神經網路殘差增強步態 (自適應崎嶇地形) |
| `btn-mode-tripod` | **TRIPOD 幾何步態**| 點擊切換 | `onnx off` | 切換為傳統三角逆運動學解析步態 |
| `btn-torque` | **RELAY 舵機電源** | 點擊切換 | `torque` | 切換 Servo 2040 實體 Relay 電源供電開關 (P0) |
| `btn-stand` | **POSE 站立/坐下** | 點擊切換 | `sit` | 切換機器人蹲坐休眠姿態 / 站立待命姿態 |
| `btn-crab` | **CRAB 橫行步態** | 點擊切換 | `crab` | 解鎖左搖桿 X 軸側向平移功能 (橫著走) |
| `btn-clearance` | **CLEARANCE 高底盤**| 點擊切換 | `clearance on/off` | 提高機身腹部離地間隙避開地面障礙 |
| `btn-estop` | **E-STOP 緊急急停** | 點擊觸發 | `walkclear` + `estop` | 強制關閉行走、鎖定煞車並切斷動力輸出 |
| `btn-calibrate` | **CALIBRATE 校準** | 點擊確認 | `calibrate` | 觸發六足觸地壓力/接觸感測器基準歸零校準 |
| `speed-slider` | **SPEED 倍率滑桿** | 滑動調整 | 本地增益調整 | 調整速度倍率 $\text{SpeedGain} \in [0.5\text{x}, 2.0\text{x}]$ (預設 1.0x) |

---

### 3.3 封包格式與雙重 Deadman 看門狗規則

#### 1. 控制封包傳輸協議 (25 Hz / 40 ms 週期)
- **移動中 (In Motion)**：
  ```json
  {
    "type": "walk",
    "mode": "onnx",
    "forward": 0.350,
    "strafe": 0.000,
    "turn": -0.420,
    "crab": false
  }
  ```
  `pilot_server.py` 轉發至 Phone B TCP 18711：
  - ONNX 模式：`walkonnx:<turn|strafe>,<forward>,0\n`
  - Tripod 模式：`walk2:<turn|strafe>,<forward>,0\n`
- **停止時 (Stopped)**：
  ```json
  { "type": "stop" }
  ```
  `pilot_server.py` 轉發：`walkclear\n`

#### 2. 雙重安全看門狗機制 (Double Deadman Protection)
1. **第一重（前端即時保護）**：只要雙搖桿回到死區內或使用者手指離開觸控螢幕，前端立即停止定時廣播，並發送單次 `stop` 封包至後端。
2. **第二重（後端獨立守護緒）**：`pilot_server.py` 內建獨立計時守護緒 (`_deadman_watchdog`)：
   - 監控逾時門檻：`DEADMAN_TIMEOUT_SEC = 0.35` (350 ms)。
   - 若發生 Wi-Fi 斷線、手機瀏覽器崩潰或封包遺失，超過 350 ms 未收到搖桿指令且當前狀態為移動中，後端**強制主動向機器人發送 `walkclear\n`**，徹底防範暴衝。

---

## 4. 系統 C：實體機 Wi-Fi 直連手把遙控 (`chica_gamepad_teleop.py`)

本系統用於工程師持 PC 或筆電，透過藍牙/USB 接上手把，透過 Wi-Fi TCP 18711 **直連機器人機載手機**進行硬體調試。

### 4.1 搖桿軸向與速度檔位修飾表

| 控制元件 | 軸向 / 按鍵 | 動作形式 | 物理控制量 | 輸出範圍 | 備註 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **左搖桿 (垂直)** | Axis 1 (Pitch) | 推前 / 拉後 | 前進 / 倒退速度 | $[-1.0, 1.0] \times \text{MaxSpeed}$ | 極性自動修正 (`-raw_pitch` 或 FPV) |
| **右搖桿 (水平)** | Axis 2 / 3 (Yaw) | 推左 / 推右 | 原地旋轉角速度 | $[-1.0, 1.0] \times \text{MaxSpeed}$ | 自動適應不同手把右搖桿軸號 |
| **LB / L1 鍵** | Button 4 (肩鍵) | **按住不放** | **Precision Crawl 微調爬行檔** | 最大速度上限限制為 **0.3 (30%)** | 適合狹窄空間微調對位 |
| **RB / R1 鍵** | Button 5 (肩鍵) | **按住不放** | **Sprint 極速衝刺檔** | 最大速度上限提升為 **1.0 (100%)** | 適合開闊地全力狂奔 |
| **預設狀態** | 無按肩鍵 | 放開 | **Normal 標準檔** | 最大速度上限為 **0.6 (60%)** | 平衡續航與操控性 |

---

### 4.2 動作按鈕對應表

| 手把按鍵 | 實體編號 | 下發指令 | 實體機器人響應動作 |
| :--- | :--- | :--- | :--- |
| **A / ✕ 鍵** | Button 0 | `sit` | **站立 / 蹲坐切換**：機器人起立進入站姿，或蹲下休眠 |
| **B / ○ 鍵** | Button 1 | `walkclear` | **即刻煞車停步**：清除所有行走指令，原地六足踏地站穩 |
| **X / □ 鍵** | Button 2 | `level` | **主動姿態調平**：利用手機 IMU 閉環保持機身水平 |
| **Y / △ 鍵** | Button 3 | `torque` | **舵機電源開關**：切換 Servo 2040 板載電源繼電器 |

---

### 4.3 安全煞車邏輯

- **死區設定**：`deadzone = 0.12`。
- **發送頻率**：25 Hz (`interval = 40ms`)。
- **指令格式**：`walk2:<turn>,<forward>,0\n`。
- **置中自動煞車**：當搖桿置中歸零時，若連續判定 **2 個週期（約 80 ms）** 為 0，腳本即刻自動發送 `walkclear\n` 並標記 `is_walking = False`，釋放舵機負載。

---

## 5. 全系統橫向參數對比總表

| 評比項目 | 系統 A: MuJoCo 模擬器 (`demo.py`) | 系統 B: 雙手機 Web (`pilot_web`) | 系統 C: 實體直連 (`chica_gamepad`) |
| :--- | :---: | :---: | :---: |
| **適用設備** | PC 模擬 (鍵盤 + BetaFPV / 手把) | 手機 A 觸控瀏覽器 (雙虛擬搖桿) | PC/筆電直連 (實體 USB/藍牙手把) |
| **底層通訊** | Pygame Direct Event | WebSocket (8081) $\rightarrow$ TCP (18711) | Wi-Fi TCP Socket (18711) |
| **更新頻率** | 模擬器即時更新 (~50 Hz) | 25 Hz (40 ms) 定時廣播 | 25 Hz (40 ms) 定時迴圈 |
| **死區閥值 (Deadzone)**| **0.08** (線性補償拉伸) | **0.05** (徑向距離閥值) | **0.12** (平滑過渡補償) |
| **側向平移 (Strafe)** | ✅ 原生支援 (右搖桿 Roll 橫滾) | ✅ 支援 (需開啓 Crab 橫行模式) | ❌ 僅支援 Forward + Turn |
| **前進最大速度** | 0.15 / 0.25 / 0.45 m/s (三檔) | $0.35 \times \text{Gain}$ m/s (滑桿可調) | 0.3 / 0.6 / 1.0 比例檔 (LB/RB修飾) |
| **旋轉最大速度** | 0.35 / 0.50 / 0.75 rad/s (三檔) | $-0.65 \times \text{Gain}$ rad/s | 1.0 (等比例縮放) |
| **姿態挺身切換** | ✅ 支援 (SC 三段撥桿: 上=基準/中=無Mapping/下=越野挺身，手把 B 鍵 Toggle) | ✅ 支援 (`clearance on/off` 按鈕) | ❌ (支援 `level` 主動調平) |
| **失能安全煞車** | 放手即停 (指令立即歸零) | 雙重保護 (前端放手即送 + 後端 350ms 看門狗) | 雙幀歸零自動發送 `walkclear` |
| **立定跳躍觸發** | 鍵盤 5/6 鍵專屬 (手把鎖定防誤觸) | 尚未開放 (保留未來擴充) | 尚未開放 |

---

## 6. 手把調試與常見問題排除 (FAQ)

### Q1: 連接 BetaFPV 遙控器後，向前推搖桿機器人反而倒退？
- **原因**：航模遙控器與一般電玩手把的垂直軸 (Pitch) 極性定義不同。
- **程式機制**：`demo.py` 與 `chica_gamepad_teleop.py` 已內建自動偵測名稱：
  ```python
  is_fpv = any(k in joystick.get_name().lower() for k in ["betafpv", "radio", "edgetx", "opentx", "taranis", "elrs", "frsky"])
  pitch_fwd = raw_pitch if is_fpv else -raw_pitch
  ```
- **排查方法**：若使用自訂名稱遙控器（如 DIY OpenTX），請確認其識別名稱或於 EdgeTX 遙控器模型設定中將 CH2 (Pitch) 設定為 Reverse (反向)。

### Q2: 手把放開時機器人仍微微移動，無法完全停步？
- **原因**：搖桿機械彈簧老化或電位器存在硬體中心偏置。
- **解決方法**：可於對應腳本中微調 `deadzone` 數值：
  - `demo.py`：調整 `deadzone=0.08` 為 `0.10` 或 `0.12`。
  - `pilot_web/app.js`：調整 `Math.hypot(this.x, this.y) < 0.05` 為 `0.08`。
  - `chica_gamepad_teleop.py`：啟動參數加入 `--deadzone 0.15`。

### Q3: 為什麼手把上找不到「立定跳躍」按鍵？
- **設計考量**：六足機器人立定跳躍（50cm / 70cm）屬於極高瞬間功率動作（全足舵機極限爆發）。為防止操作手在搖桿走位或巡航時因手指誤觸導致摔機或撞牆，目前刻意將跳躍動作保留為**鍵盤數字鍵 5 與 6 專屬觸發**。

---
> 💡 *本手冊已整合至 Make Your Pet Hexapod 專案文檔系統，可隨時依硬體更新擴充新通道映射。*
