# 📱 Make Your Pet 六足機器人：雙手機與 AI 步態 (ONNX) 控制系統軟體設計計畫書
### Dual-Phone & ONNX Locomotion Control System Architecture Plan

> **專案儲存庫**：`Make Your Pet Digital Twin`  
> **硬體平台**：[MakeYourPet Hexapod (Chica / Chipo)](https://github.com/MakeYourPet/hexapod)  
> **主控板型號**：Pimoroni Servo 2040 (RP2040 MCU)  
> **最新修訂**：2026-09-29  
> **狀態**：架構選型定案與實施計畫 (Architecture Finalized & Implementation Ready)

---

## 目錄 (Table of Contents)

1. [系統背景與設計目標](#1-系統背景與設計目標)
2. [重大架構選型決策 (Architecture Decision Record, ADR) ⭐](#2-重大架構選型決策-architecture-decision-record-adr-)
   - 2.1 [核心決策：基於 chica-server 原生 Android 擴展 + DigitalTwin-Server 駕駛中繼](#21-核心決策基於-chica-server-原生-android-擴展--digitaltwin-server-駕駛中繼)
   - 2.2 [否決方案分析：為何不從零重寫全新 Android Server？](#22-否決方案分析為何不從零重寫全新-android-server)
   - 2.3 [工作區目錄職責清晰劃分](#23-工作區目錄職責清晰劃分)
3. [MakeYourPet 機板與馬達控制深度調查](#3-makeyourpet-機板與馬達控制深度調查)
   - 3.1 [核心基板：Pimoroni Servo 2040](#31-核心基板pimoroni-servo-2040)
   - 3.2 [18 軸 PWM 馬達控制原理 (RP2040 PIO)](#32-18-軸-pwm-馬達控制原理-rp2040-pio)
   - 3.3 [電源開關與 Relay 繼電器保護機制 (GPIO 26 / A0)](#33-電源開關與-relay-繼電器保護機制-gpio-26--a0)
   - 3.4 [USB CDC 虛擬串列埠二進位通訊協定全解構 (SET 0xD3, GET 0xC7)](#34-usb-cdc-虛擬串列埠二進位通訊協定全解構-set-0xd3-get-0xc7)
   - 3.5 [實體硬體插針走線與關節重映射 (Joint-to-Pin Mapping) ⭐](#35-實體硬體插針走線與關節重映射-joint-to-pin-mapping-)
   - 3.6 [感測回傳 (Telemetry 上行協定) 與 ADC 轉換公式 ⭐](#36-感測回傳-telemetry-上行協定與-adc-轉換公式-)
   - 3.7 [欠壓與過流硬體安全保護機制 (Under-voltage & Over-current Cutoff)](#37-欠壓與過流硬體安全保護機制-under-voltage--over-current-cutoff)
   - 3.8 [機械組裝零位偏移與方向校準 (Mechanical Calibration & Attach Angles)](#38-機械組裝零位偏移與方向校準-mechanical-calibration--attach-angles)
4. [機載手機與機板實體通訊與電源安全](#4-機載手機與機板實體通訊與電源安全)
   - 4.1 [USB-OTG 串列直連與 Android 自動授權 (USB_DEVICE_ATTACHED)](#41-usb-otg-串列直連與-android-自動授權-usb_device_attached)
   - 4.2 [硬體電源隔離注意事項 (防止高壓反灌燒機) ⚠️](#42-硬體電源隔離注意事項-防止高壓反灌燒機-️)
5. [雙手機 + ONNX 系統整體軟體架構](#5-雙手機--onnx-系統整體軟體架構)
   - 5.1 [系統拓撲與雙向資料流 (Mermaid 架構圖)](#51-系統拓撲與雙向資料流-mermaid-架構圖)
   - 5.2 [手機 A：駕駛操控終端與即時狀態儀表板 (HUD)](#52-手機-a駕駛操控終端與即時狀態儀表板-hud)
   - 5.3 [手機 B：機載大腦 (ChicaController + ONNX Runtime Mobile)](#53-手機-b機載大腦-chicacontroller--onnx-runtime-mobile)
   - 5.4 [67 維觀測狀態向量對齊與建構規範](#54-67-維觀測狀態向量對齊與建構規範)
   - 5.5 [殘差融合、EMA 濾波與軟體機械限位](#55-殘差融合ema-濾波與軟體機械限位)
6. [核心程式碼實作示範](#6-核心程式碼實作示範)
   - 6.1 [手機 B 原生端：ONNX Runtime 整合骨架 (`OnnxLocomotionRunner.java`)](#61-手機-b-原生端onnx-runtime-整合骨架-onnxlocomotionrunnerjava)
   - 6.2 [手機 B 原生端：Gradle 依賴設定 (`app/build.gradle`)](#62-手機-b-原生端gradle-依賴設定-appbuildgradle)
   - 6.3 [手機 A 駕駛端：Web 虛擬雙搖桿與 Telemetry 伺服器 (`pilot_server.py`)](#63-手機-a-駕駛端web-虛擬雙搖桿與-telemetry-伺服器-pilot_serverpy)
   - 6.4 [底層驅動與 PC 模擬測試工具 (`servo2040_driver.py`)](#64-底層驅動與-pc-模擬測試工具-servo2040_driverpy)
7. [實作里程碑與驗證計畫 (Milestones)](#7-實作里程碑與驗證計畫-milestones)

---

## 1. 系統背景與設計目標

本計畫旨在為 **MakeYourPet 六足仿生機器人 (Hexapod)** 構建現代化的雙手機邊緣 AI 控制系統：
1. **輸入成果**：載入本數位孿生專案訓練完成並匯出的強化學習策略模型 `hexapod_policy.onnx`（約 1.9 KB）。
2. **操作體驗**：使用 **手機 A** 作為手持遙控器，提供電玩等級雙虛擬搖桿與即時狀態儀表板（電池電壓、負載電流、足端觸地反饋，支援放手即停）。
3. **運算中樞**：使用載於機身的 **手機 B** 作為機器人邊緣計算大腦，接收手機 A 的運動指令，實時運行解析前饋運動學與 ONNX 殘差神經網路，並讀取手機內建 IMU 感測器進行閉環姿態維持。
4. **實體執行**：手機 B 透過 USB-OTG 連接機器人主控板（Pimoroni Servo 2040），以 50Hz 頻率將 18 個關節的角度目標即時發送至金屬伺服舵機。

---

## 2. 重大架構選型決策 (Architecture Decision Record, ADR) ⭐

### 2.1 核心決策：基於 chica-server 原生 Android 擴展 + DigitalTwin-Server 駕駛中繼

系統整體架構採用 **分層模組化** 策略：
* **機載大腦端 (手機 B)**：**直接採用開源 `chica-server-main` 作為原生底座進行功能擴充**。在現有 Android 專案中導入微軟官方 `onnxruntime-android` 函式庫，將 `hexapod_policy.onnx` 模型放入 `assets/`，在原有的 50Hz 控制迴圈內實現「IMU 感測 + 67 維觀測構建 + ONNX 殘差疊加」。
* **手持操控與中繼端 (手機 A)**：在 **`MakeYourPet-DigitalTwin-Server/`** 構建專屬的 Web 觸控遙控儀表板 (Pilot HUD) 與通訊橋接服務，提供跨平台的虛擬搖桿操控與即時 Telemetry 顯示。

### 2.2 否決方案分析：為何不從零重寫全新 Android Server？

經過深度技術評估，**明確否決「在 MakeYourPet-DigitalTwin-Server 從零重寫全新 Android Server」的方案**，原因如下：

| 評估維度 | `chica-server-main` 現成能力 | 從零重寫之代價與致命風險 |
| :--- | :--- | :--- |
| **USB-OTG 免確認授權** | 透過 `AndroidManifest.xml` 與 `device_filter.xml` 綁定 RP2040 VID(`0x2e8a`)/PID(`0x000a`)，**插線即開機無感授權**。 | Android 系統對 USB 權限控管極嚴格，從零實作極易遇到插拔後權限遺失或每次跳彈窗的問題，嚴重破壞機器人即插即用體驗。 |
| **超低延遲串列驅動** | 內建經過嚴格時序驗證的 **7ms 非同步輪詢線程** (`UsbSerialServoBackend.java`) 與 39-byte 防碰撞二進位編碼。 | 自行重寫串列通訊容易遭遇 I/O 阻塞、執行緒競爭或 Baudrate 溢位，導致舵機反應延遲甚至高頻抽搐。 |
| **手機 IMU 姿態融合** | 完整封裝 Gravity + Magnetic 旋轉向量融合 (`OriginalOrientationSensor.java`)，即時輸出 Roll, Pitch 與角速度。 | Android 各品牌機型感測器坐標系映射與四元數轉尤拉角存在大量設備相容性問題，重新除錯耗費大量精力。 |
| **硬體電源守護機制** | 內建**電壓 $<6.0\text{V}$ 欠壓保護**、**電流 $>10\text{A}$ 堵轉斷電**、繼電器總閥控制與 `ToneGenerator` 聲響提示。 | 2S 鋰電池只要過放一次便會永久報廢；18 顆金屬舵機堵轉 2 秒即可燒毀驅動板。從零編寫若守護邏輯有微小瑕疵，將造成實體硬體損毀。 |
| **系統休眠管理 (Doze)** | 內建全螢幕 HUD 與 `WakeLock` 保持 CPU 常駐，防止後台被 Android OS 殺死。 | Android 9+ 嚴格的電池優化常在螢幕變暗或背景運行數十秒後殺死自寫的 Socket 服務。 |

> **結論**：`chica-server-main` 的底層通訊與硬體防護是 100% 穩定可用的生產級成果，站在成熟底座上直接擴展 ONNX 殘差神經網路，可省下數週重複造輪子的時間。

### 2.3 工作區目錄職責清晰劃分

為確保工程維護乾淨整潔，工作區目錄分工定義如下：

```text
Make Your Pet Digital Twin/
│
├── chica-server-main/                # 【手機 B 本體原生 App】
│   └── 專注於 Android 原生 APK 編譯：
│       ├── USB-OTG 串列通訊 (Pimoroni Servo 2040)
│       ├── 手機 IMU 感測器融合 (OrientationSensor)
│       ├── 嵌入 ONNX Runtime Mobile (hexapod_policy.onnx 殘差推論)
│       └── TCP 18711 遙控指令接收與遙測廣播
│
├── MakeYourPet-DigitalTwin-Server/   # 【手機 A / 外部中繼與駕駛端】
│   ├── pilot_web/                    # 手機 A 專用 Web 儀表板與虛擬雙搖桿 (HTML5/Canvas)
│   ├── pilot_server.py               # (可選) 轉發 WebSocket 指令至 chica-server TCP 18711 的中繼器
│   └── tools/                        # PC 端離線測試、模擬器與協定驗證腳本
│
└── MakeYourPet-DigitalTwin/          # 【數位孿生與 RL 訓練中樞】
    ├── models/hexapod_policy.onnx    # 訓練產出的神經網路權重
    └── train_walking/                # MuJoCo 模擬環境與 PPO 訓練
```

---

## 3. MakeYourPet 機板與馬達控制深度調查

### 3.1 核心基板：Pimoroni Servo 2040
MakeYourPet 原版（Chica 與 Chipo）推薦的主控板為 **Pimoroni Servo 2040**：
* **核心晶片**：Raspberry Pi **RP2040**（Dual ARM Cortex-M0+ @ 133MHz，264KB SRAM）。
* **介面配置**：
  * 18 組 3-pin 伺服舵機插針（支援同步控制 18 自由度六足機器人）。
  * 內建高精度分流電阻與 ADC 電流監測（可讀取全機總耗電電流）。
  * 內建電阻分壓 ADC 電池電壓監測。
  * 6 組類比感測器擴展通道（可接六足足端觸地開關 TS1~TS6）。
  * 獨立 USB-C 連接埠（支援 USB CDC 虛擬串列埠）。

### 3.2 18 軸 PWM 馬達控制原理 (RP2040 PIO)
* **舵機規範**：常規 25kg/35kg 金屬齒輪數位舵機（如 ZOSKAY DS3235, MG996R 等）。
* **PWM 訊號參數**：
  * 控制週期：$20\text{ ms}$（頻率 **$50\text{ Hz}$**）。
  * 脈衝寬度：$1000\ \mu s \sim 2000\ \mu s$（中央基準點為 $1500\ \mu s$ 對應 $0^\circ$）。
* **PIO 硬體狀態機**：
  RP2040 晶片原生僅有 16 個硬體 PWM 輸出，但透過其獨特的 **PIO (Programmable I/O)** 技術，板端韌體編寫了微秒級的精確狀態機，能無 CPU 負擔地並行產生 18 路亞微秒解析度的 PWM 脈衝。
* **物理角度轉脈寬公式**：
  $$\text{PWM}(\mu s) = 1500 - \left(\frac{\theta_{\text{rad}}}{\pi} \times 180\right) \times 11.11$$
  * 基節（Coxa, Hip Yaw）：範圍約 $\pm 45^\circ$（對應 $1000 \sim 2000\ \mu s$）
  * 大腿（Femur, Hip Pitch）：範圍約 $\pm 45^\circ$（對應 $1000 \sim 2000\ \mu s$）
  * 小腿（Tibia, Knee Pitch）：行程最大約 $\pm 60^\circ$（對應 $833 \sim 2167\ \mu s$）

### 3.3 電源開關與 Relay 繼電器保護機制 (GPIO 26 / A0)
機板在 **A0 接腳 (GPIO 26)** 連接了一組電源繼電器（Relay）或高功率 MOSFET 開關：
* **上電安全防護**：
  當板端剛上電時，A0 預設為 LOW，舵機供電與 PWM 輸出均被硬體與軟體雙重切斷（防止開機時舵機暴衝、夾手或齒輪受損）。
* **啟用條件**：
  只有當外部主控（手機 B）發送指令將 A0 設為 `1` 時，韌體才會執行 `servos.enable_all()` 啟動 PWM 輸出並接通舵機主電源。
* **緊急煞車 (E-Stop)**：
  發送指令將 A0 設為 `0` 時，韌體會執行 `servos.disable_all()` 立即切斷動力與洩力。

### 3.4 USB CDC 虛擬串列埠二進位通訊協定全解構 (SET 0xD3, GET 0xC7)
官方韌體（`EddieCarrera/chica-servo2040-simpleDriver`）採用了高效且具備**封包防混淆**特性的二進位傳輸協議：

#### (1) 指令碼定義
* `SET_CMD = 0xD3`（二進位 `11010011`，最高位 MSB=1，表示指令開始）
* `GET_CMD = 0xC7`（二進位 `11000111`）

#### (2) 7-Bit 數據防碰撞編碼
為了避免傳輸的數值（如 1500 = `0x05DC`）中的字節被錯誤判斷為指令起始符號，韌體強制將每個 16-bit 數值拆解為兩個 **7-bit** 字節（最高位 bit 7 強制為 0）：
$$\text{low\_byte} = \text{val} \ \& \ \text{0x7F}, \quad \text{high\_byte} = (\text{val} \gg 7) \ \& \ \text{0x7F}$$
韌體端還原運算：$\text{val} = \text{low\_byte} \mid (\text{high\_byte} \ll 7)$。

#### (3) 18 軸舵機全域設定封包 (共 39 Bytes)
| Byte 索引 | 欄位名稱 | 數值 / 格式 | 說明 |
| :--- | :--- | :--- | :--- |
| **0** | `Command` | `0xD3` (`SET_CMD`) | 指令起始標頭 |
| **1** | `startIdx` | `0x00` (`SERVO1`) | 起始通道索引（從第 1 顆舵機開始） |
| **2** | `count` | `0x12` (十進位 18) | 欲寫入的連續通道總數 |
| **3 ~ 4** | Servo 1 PWM | Low 7-bit, High 7-bit | 第 1 顆舵機脈寬微秒值 ($1000 \sim 2000\mu s$) |
| **...** | ... | ... | ... |
| **37 ~ 38**| Servo 18 PWM| Low 7-bit, High 7-bit | 第 18 顆舵機脈寬微秒值 |

#### (4) Relay 繼電器開關封包 (共 5 Bytes)
* **開啟電源**：`[ 0xD3, 26, 1, 0x01, 0x00 ]`（startIdx=26 為 RELAY）
* **關閉電源**：`[ 0xD3, 26, 1, 0x00, 0x00 ]`

#### (5) GET 感測器遙測查詢封包 (共 3 Bytes 請求，19 Bytes 回傳)
* **主控發送請求**：`[ 0xC7, startPin, count ]`，讀取 Pin 18 (TS1) 起始的 8 個類比通道：`[ 0xC7, 18, 8 ]`。
* **板端即時回傳**：`[ 0xC7, startPin, count, val0_lo, val0_hi, val1_lo, val1_hi, ... ]`（總長 19 字節）。

### 3.5 實體硬體插針走線與關節重映射 (Joint-to-Pin Mapping) ⭐

> [!IMPORTANT]
> **避免馬達錯位之核心關鍵**：  
> 在 MuJoCo 模擬環境中，六足關節順序為標稱足順序（`L1 -> L2 -> L3 -> R1 -> R2 -> R3`，每個足依序為 `Coxa -> Femur -> Tibia`）。  
> 然而根據 `chica-server` 逆向原廠 `config-2040.txt` 發現，**MakeYourPet 實體走線在 Servo 2040 上的插針分佈並非 0~17 線性排列**！

#### MakeYourPet 原廠標準硬體接線表：

| 六足腿部 (Leg) | 數位孿生關節索引 (Joint Index) | 關節名稱 | Servo 2040 實體插針 (Channel) | 原廠配置標籤 |
| :--- | :---: | :---: | :---: | :---: |
| **L1 (左前)** | 0, 1, 2 | Coxa, Femur, Tibia | **P15, P16, P17** | `L11, L12, L13` |
| **L2 (左中)** | 3, 4, 5 | Coxa, Femur, Tibia | **P09, P10, P11** | `L21, L22, L23` |
| **L3 (左後)** | 6, 7, 8 | Coxa, Femur, Tibia | **P03, P04, P05** | `L31, L32, L33` |
| **R1 (右前)** | 9, 10, 11 | Coxa, Femur, Tibia | **P12, P13, P14** | `R11, R12, R13` |
| **R2 (右中)** | 12, 13, 14 | Coxa, Femur, Tibia | **P06, P07, P08** | `R21, R22, R23` |
| **R3 (右後)** | 15, 16, 17 | Coxa, Femur, Tibia | **P00, P01, P02** | `R31, R32, R33` |

驅動層映射陣列常數定義：
```java
public static final int[] JOINT_TO_PIN_MAP = {
    15, 16, 17,  // L1: Coxa, Femur, Tibia (Joint 0, 1, 2)
    9,  10, 11,  // L2: Coxa, Femur, Tibia (Joint 3, 4, 5)
    3,  4,  5,   // L3: Coxa, Femur, Tibia (Joint 6, 7, 8)
    12, 13, 14,  // R1: Coxa, Femur, Tibia (Joint 9, 10, 11)
    6,  7,  8,   // R2: Coxa, Femur, Tibia (Joint 12, 13, 14)
    0,  1,  2    // R3: Coxa, Femur, Tibia (Joint 15, 16, 17)
};
```

### 3.6 感測回傳 (Telemetry 上行協定) 與 ADC 轉換公式 ⭐

透過發送 `0xC7 18 8` 指令，主控端在控制循環中讀取 8 組 14-bit 類比感測數據：

| 引腳編號 | 實體信號 | 標識 | 原始數值範圍 | 物理轉換公式 | 物理單位 |
| :---: | :--- | :--- | :---: | :--- | :---: |
| **P18** | 右後足觸地 (TS_R3) | `TS6` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P19** | 左後足觸地 (TS_L3) | `TS3` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P20** | 右中足觸地 (TS_R2) | `TS5` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P21** | 左中足觸地 (TS_L2) | `TS2` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P22** | 右前足觸地 (TS_R1) | `TS4` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P23** | 左前足觸地 (TS_L1) | `TS1` | $0 \sim 1024$ | $\text{raw} / 1024.0 > 0.5$ | 數位開關 (0/1) |
| **P24** | 全機總電流感測 | `CURR` | $0 \sim 1024$ | $(\text{raw} - 512) \times 0.0814$ | 安培 (A) |
| **P25** | 電池主供電電壓 | `VOLT` | $0 \sim 4096$ | $\text{raw} / 310.3$ | 伏特 (V) |

### 3.7 欠壓與過流硬體安全保護機制 (Under-voltage & Over-current Cutoff)

為防範 2S 鋰電池（7.4V）過放損毀或舵機堵轉燒板，控制系統具備與 `chica-server` 一致的雙重硬體安全守護：
1. **鋰電池欠壓保護 (Under-voltage Guard)**：
   * **警告閾值**：電壓 $< 6.4\text{ V}$ 持續 $2.0\text{ s}$，觸發聲響警報與儀表板紅字提示。
   * **強制斷電保護**：電壓 $< 6.0\text{ V}$ 持續 $2.0\text{ s}$，**立即向 A0 發送 Relay OFF (`0xD3 26 1 0 0`)**，切斷舵機動力。
2. **舵機過流堵轉保護 (Over-current Guard)**：
   * **警告閾值**：全機電流 $> 8.0\text{ A}$ 持續 $2.0\text{ s}$，發出過載預警。
   * **強制斷電保護**：全機電流 $> 10.0\text{ A}$ 持續 $2.0\text{ s}$，立即強制切斷 Relay 電源。

### 3.8 機械組裝零位偏移與方向校準 (Mechanical Calibration & Attach Angles)
* **基節安裝偏角 (Coxa Attach Angle)**：前足 ($-8.0^\circ$)、中足 ($0.0^\circ$)、後足 ($+8.0^\circ$)。
* **大腿安裝偏角 (Femur Attach Angle)**：$+35.0^\circ$。
* **小腿安裝偏角 (Tibia Attach Angle)**：$+68.0^\circ$。
* **方向極性**：原廠舵機在 $-45^\circ$ 對應 $2000\ \mu s$，在 $+45^\circ$ 對應 $1000\ \mu s$。

---

## 4. 機載手機與機板實體通訊與電源安全

```
┌──────────────────────────────────────────────┐
│  📱 手機 B (載於六足機器人背部)               │
│  - 運行 chica-server + ONNX 殘差神經網路      │
│  - 讀取內部 IMU 感測器姿態                    │
└──────────────────────┬───────────────────────┘
                       │ USB-C OTG 連接線 (CDC-ACM 串列埠)
                       ▼
┌──────────────────────────────────────────────┐
│  ⚡ Pimoroni Servo 2040 主控板               │
│  - 接收 39-byte 角度封包                     │
│  - PIO 產生 18 路 PWM 訊號至舵機             │
│  - 【注意】：割斷背面 "Separate USB" 銅箔    │
└──────────────────────────────────────────────┘
```

### 4.1 USB-OTG 串列直連與 Android 自動授權 (USB_DEVICE_ATTACHED)
* **實體安裝**：使用 MakeYourPet 專用固定架（`phone-bar.stl`）將手機 B 夾持在機器人背部。
* **免點擊自動授權機制**：
  在 `AndroidManifest.xml` 中宣告 `<intent-filter>` 捕捉 `android.hardware.usb.action.USB_DEVICE_ATTACHED`，並搭配 `device_filter.xml` 指定 RP2040 的硬體識別碼（Vendor ID `0x2e8a`，Product ID `0x000a`）。
  * 效果：當 USB 線插入手機時，Android 系統提示一次設為預設應用程式後，**未來每次插線即自動啟動大腦背景服務並直接獲取 USB 讀寫權限**。

### 4.2 硬體電源隔離注意事項 (防止高壓反灌燒機) ⚠️
> [!CAUTION]
> **極度重要之硬體保護操作**：  
> 當使用 2S (7.4V) 鋰電池由接線端子供電給 Servo 2040 時，**必須使用美工刀割斷板子背面標記「Separate USB and Ext. Power」的跳線銅箔**！  
> 若未割斷，7.4V 電池高壓將直接反向灌入手機 B 的 USB 連接埠，造成手機充電晶片或主機板永久燒毀！

---

## 5. 雙手機 + ONNX 系統整體軟體架構

### 5.1 系統拓撲與雙向資料流 (Mermaid 架構圖)

```mermaid
flowchart TD
    subgraph PhoneA ["📱 手機 A：駕駛操控終端 (MakeYourPet-DigitalTwin-Server)"]
        UI["觸控虛擬雙搖桿 & 檔位按鈕 (HTML5 Canvas)"]
        HUD["實時儀表板: 電壓(V)、電流(A)、足端觸地(LEGS)、連線品質"]
        CMD["指令發送器: [vx, vy, yaw_rate, mode]"]
        TX_NET["Wi-Fi TCP / WebSocket 客戶端"]
        RX_NET_A["遙測接收端 (解析 Telemetry)"]
        
        UI --> CMD --> TX_NET
        RX_NET_A --> HUD
    end

    subgraph Comm ["🌐 無線區域網路 (Wi-Fi Hotspot)"]
        PACKET_DOWN["控制指令 / 心跳封包 (50Hz, RTT < 5ms)"]
        PACKET_UP["遙測狀態反饋 (25Hz: V, I, TS1~6)"]
        TX_NET -.-> PACKET_DOWN -.-> RX_NET_B
        TX_NET_B -.-> PACKET_UP -.-> RX_NET_A
    end

    subgraph PhoneB ["🤖 手機 B：機載大腦 (chica-server + ONNX Core)"]
        RX_NET_B["OriginalTcpControlServer (Port 18711, 附 300ms Watchdog)"]
        TX_NET_B["狀態廣播服務 (ready:BPS=...|V=...|I=...|LEGS=...)"]
        IMU["OriginalOrientationSensor (手機 Roll, Pitch, 角速度)"]
        
        subgraph OnnxEngine ["⭐ ONNX 殘差步態引擎 (OnnxLocomotionRunner)"]
            KIN["解析三角步態前饋 (Tripod Kinematics: q_ref)"]
            OBS["67 維觀測狀態建構模組 (Observation Builder)"]
            ORT["ONNX Runtime Mobile (hexapod_policy.onnx)"]
            FUSION["殘差融合: q = q_ref + 0.15 * delta_q"]
        end

        SAFETY["硬體安全守護神: 欠壓 <6.0V / 過流 >10A 自動切斷 Relay"]
        DRIVER["UsbSerialServoBackend (CDC-ACM 7ms + JOINT_TO_PIN_MAP)"]

        RX_NET_B -->|最新運動指令 [vx, vy, yaw]| KIN
        RX_NET_B -->|指令向量| OBS
        IMU -->|姿態與角速度| OBS
        KIN -->|步態時鐘 phi & 前饋基準角 q_ref| OBS
        OBS -->|67-dim 輸入張量| ORT
        ORT -->|18-dim 動作殘差 delta_q| FUSION
        KIN -->|前饋基準角度 q_ref| FUSION
        FUSION -->|18 軸平滑目標角度 (rad)| DRIVER
        
        DRIVER -.->|0xC7 遙測原始值 (V, I, TS1~6)| SAFETY
        SAFETY -->|狀態安全| TX_NET_B
        SAFETY -->|觸發異常保護| DRIVER
    end

    subgraph Hardware ["🦾 六足實體機構 (MakeYourPet Hardware)"]
        S2040["Pimoroni Servo 2040 (RP2040)"]
        RELAY["A0 電源繼電器"]
        SERVOS["18× 金屬數位舵機 (P00~P17)"]
        SENSORS["足端觸地開關 (P18~23) & ADC (P24, P25)"]

        DRIVER ==>|39-byte 角度封包 (115200 Baud)| S2040
        S2040 --> RELAY
        S2040 --> SERVOS
        SENSORS -.->|19-byte 0xC7 回傳幀| S2040
        S2040 -.->|CDC-ACM 上行串列流| DRIVER
    end
```

### 5.2 手機 A：駕駛操控終端與即時狀態儀表板 (HUD)
* **主控形式**：由 `MakeYourPet-DigitalTwin-Server` 託管的現代化 Web 儀表板，手機 A 瀏覽器開啟即可操作。
* **遙控輸入**：
  * 左搖桿：前進/後退速度 $v_x \in [-0.25, 0.45]\text{ m/s}$。
  * 右搖桿：偏航旋轉角速度 $\omega_z \in [-0.75, 0.75]\text{ rad/s}$。
  * **Deadman Switch（放手即停）**：搖桿釋放時即刻送出停止指令，原地收步煞停。
* **即時 Telemetry HUD**：
  * 電池電壓 $V$（低於 6.4V 警報，低於 6.0V 警告切斷）。
  * 負載電流 $I$（即時安培數，防範堵轉）。
  * 六足觸地圖示（LEGS 狀態碼）。
* **Watchdog 守門犬**：手機 B 若超過 300ms 未收指令則原地煞車；超過 2000ms 判定失聯自動關閉 Relay 繼電器。

### 5.3 手機 B：機載大腦 (ChicaController + ONNX Runtime Mobile)
在 Android 原生應用中，以嚴格的 **50Hz (20ms)** 週期執行：
1. 讀取 `OriginalOrientationSensor` 獲得姿態角與角速度。
2. 推進步態時鐘 $\phi$，計算解析前饋角度 $\mathbf{q}_{\text{ref}}$。
3. 組裝 67 維觀測狀態向量 $\mathbf{s}_{67}$。
4. 調用 ONNX Runtime Mobile 推論輸出 18 維動作殘差 $\Delta\mathbf{q} \in [-1, 1]^{18}$。
5. 殘差疊加 $\mathbf{q} = \mathbf{q}_{\text{ref}} + 0.15 \times \Delta\mathbf{q}$，經 EMA 濾波後透過 USB-OTG 發送至機板。

### 5.4 67 維觀測狀態向量對齊與建構規範

為保證實機行為與 MuJoCo 模擬訓練完全一致，觀測向量順序必須嚴格對齊：

| 切片索引 | 維度 | 物理意義 | 實機數據來源 |
| :--- | :---: | :--- | :--- |
| `obs[0:2]` | 2 | 機身姿態 $[\text{roll}, \text{pitch}]$ (rad) | 手機內建 IMU 融合姿態 |
| `obs[2:5]` | 3 | 機身本體角速度 $[\omega_x, \omega_y, \omega_z]$ (rad/s) | 手機內建陀螺儀 |
| `obs[5:8]` | 3 | 機身本體線速度 $[v_x, v_y, v_z]$ (m/s) | 目標前饋指令與卡爾曼估算 |
| `obs[8:26]` | 18 | 關節跟隨誤差 $\mathbf{q}_{\text{current}} - \mathbf{q}_{\text{ref}}$ | 舵機當前預估角度與前饋角之差 |
| `obs[26:44]`| 18 | 關節轉速 $\dot{\mathbf{q}} \times 0.1$ | 差分相鄰週期的目標關節角 |
| `obs[44:62]`| 18 | 上一時刻殘差動作 $\Delta\mathbf{q}_{t-1}$ | 前一週期的網路輸出記憶緩衝區 |
| `obs[62:65]`| 3 | 目標速度指令 $[v_x, v_y, \omega_z]$ | 來自手機 A 的最新遙控目標 |
| `obs[65:67]`| 2 | 步態相位時鐘 $[\sin(\phi), \cos(\phi)]$ | 解析三角步態當前相位函數 |

### 5.5 殘差融合、EMA 濾波與軟體機械限位

$$\mathbf{q}_{\text{raw}} = \mathbf{q}_{\text{ref}} + 0.15 \times \Delta \mathbf{q}_{\text{RL}}$$

1. **軟體機械限位**：
   * Coxa（關節 $0, 3, 6, 9, 12, 15$）：限制在 $[-0.785, 0.785]\text{ rad}$ ($\pm 45^\circ$)
   * Femur（關節 $1, 4, 7, 10, 13, 16$）：限制在 $[-0.785, 0.785]\text{ rad}$ ($\pm 45^\circ$)
   * Tibia（關節 $2, 5, 8, 11, 14, 17$）：限制在 $[-1.047, 1.047]\text{ rad}$ ($\pm 60^\circ$)
2. **指數移動平均 (EMA) 濾波器**：
   $$\mathbf{q}_{\text{send}}(t) = (1 - \beta) \cdot \mathbf{q}_{\text{send}}(t-1) + \beta \cdot \mathbf{q}_{\text{clamped}}(t) \quad (\beta = 0.7)$$

---

## 6. 核心程式碼實作示範

### 6.1 手機 B 原生端：ONNX Runtime 整合骨架 (`OnnxLocomotionRunner.java`)

放置於 `chica-server-main/app/src/main/java/com/makeyourpet/chicaserver/control/OnnxLocomotionRunner.java`：

```java
package com.makeyourpet.chicaserver.control;

import android.content.Context;
import ai.onnxruntime.*;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.Collections;

public final class OnnxLocomotionRunner implements AutoCloseable {
    private static final int OBS_DIM = 67;
    private static final int ACTION_DIM = 18;
    private static final float RESIDUAL_SCALE = 0.15f;
    private static final float EMA_BETA = 0.7f;

    private final OrtEnvironment env;
    private final OrtSession session;
    private final float[] prevAction = new float[ACTION_DIM];
    private final float[] smoothedTarget = new float[ACTION_DIM];
    private double gaitPhase = 0.0;

    public OnnxLocomotionRunner(Context context, String modelAssetPath) throws Exception {
        this.env = OrtEnvironment.getEnvironment();
        try (InputStream is = context.getAssets().open(modelAssetPath)) {
            byte[] modelBytes = new byte[is.available()];
            is.read(modelBytes);
            this.session = env.createSession(modelBytes, new OrtSession.SessionOptions());
        }
    }

    public synchronized float[] step(float vx, float vy, float yawRate, 
                                     float roll, float pitch, float[] omega, 
                                     float[] qRef, float dt) throws OrtException {
        // 1. 步態時鐘推進
        boolean isMoving = Math.abs(vx) > 0.02f || Math.abs(yawRate) > 0.05f;
        if (isMoving) {
            gaitPhase = (gaitPhase + 2.0 * Math.PI * 1.5 * dt) % (2.0 * Math.PI);
        } else {
            gaitPhase = 0.0;
        }

        // 2. 組裝 67 維觀測狀態向量
        float[] obs = new float[OBS_DIM];
        obs[0] = roll;
        obs[1] = pitch;
        System.arraycopy(omega, 0, obs, 2, 3);
        obs[5] = vx; obs[6] = vy; obs[7] = 0.0f;
        // obs[8..25] 關節跟隨誤差, obs[26..43] 關節速度
        System.arraycopy(prevAction, 0, obs, 44, ACTION_DIM);
        obs[62] = vx; obs[63] = vy; obs[64] = yawRate;
        obs[65] = isMoving ? (float) Math.sin(gaitPhase) : 0.0f;
        obs[66] = isMoving ? (float) Math.cos(gaitPhase) : 0.0f;

        // 3. ONNX 推論
        long[] shape = new long[]{1, OBS_DIM};
        try (OnnxTensor inputTensor = OnnxTensor.createTensor(env, FloatBuffer.wrap(obs), shape);
             OrtSession.Result result = session.run(Collections.singletonMap("obs", inputTensor))) {
            
            float[][] rawResidual = (float[][]) result.get(0).getValue();
            float[] residual = rawResidual[0];
            System.arraycopy(residual, 0, prevAction, 0, ACTION_DIM);

            // 4. 殘差疊加、限位與 EMA 濾波
            for (int i = 0; i < ACTION_DIM; i++) {
                float target = qRef[i] + residual[i] * RESIDUAL_SCALE;
                // 限位
                int jointInLeg = i % 3;
                float limit = (jointInLeg == 2) ? 1.047f : 0.785f;
                target = Math.max(-limit, Math.min(limit, target));
                smoothedTarget[i] = (1.0f - EMA_BETA) * smoothedTarget[i] + EMA_BETA * target;
            }
        }
        return smoothedTarget.clone();
    }

    @Override
    public void close() throws Exception {
        if (session != null) session.close();
        if (env != null) env.close();
    }
}
```

### 6.2 手機 B 原生端：Gradle 依賴設定 (`app/build.gradle`)

在 `chica-server-main/app/build.gradle` 的 `dependencies` 區塊加入官方 Android 依賴：

```groovy
dependencies {
    // 既有依賴...
    implementation 'androidx.appcompat:appcompat:1.6.1'

    // ONNX Runtime Android 輕量推論引擎 (支援 arm64-v8a 與 armeabi-v7a)
    implementation 'com.microsoft.onnxruntime:onnxruntime-android:1.17.1'
}
```

### 6.3 手機 A 駕駛端：Web 虛擬雙搖桿與 Telemetry 伺服器 (`pilot_server.py`)

放置於 `MakeYourPet-DigitalTwin-Server/pilot_server.py`，提供手機 A 瀏覽器直接操作的 Web 儀表板，並轉發指令至手機 B 的 TCP 18711 埠：

```python
"""
pilot_server.py - 手機 A 駕駛操控端與 Telemetry HUD 伺服器
運行於 PC 或手機 A 本機，提供瀏覽器電玩級雙搖桿，並橋接至手機 B (ChicaServer TCP 18711)
"""
import socket
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
import json

CHICA_HOST = "192.168.43.1"  # 手機 B 熱點預設 IP
CHICA_PORT = 18711

class ChicaBridge:
    def __init__(self, host=CHICA_HOST, port=CHICA_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.latest_status = "connecting..."
        threading.Thread(target=self._telemetry_listener, daemon=True).start()

    def _telemetry_listener(self):
        buf = ""
        while True:
            try:
                data = self.sock.recv(1024).decode('utf-8', errors='ignore')
                if not data: break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    if line.startswith("ready:") or line.startswith("busy:"):
                        self.latest_status = line.strip()
            except Exception:
                break

    def send_command(self, cmd_str: str):
        try:
            self.sock.sendall((cmd_str + "\n").encode('utf-8'))
        except Exception as e:
            print("Send error:", e)

if __name__ == "__main__":
    print("[駕駛端啟動] 請使用手機 A 瀏覽器開啟 http://<本機IP>:8080 進行操作")
    # 啟動 Web 服務與 TCP 橋接...
```

### 6.4 底層驅動與 PC 模擬測試工具 (`servo2040_driver.py`)

放置於 `MakeYourPet-DigitalTwin-Server/tools/servo2040_driver.py`，用於離線驗證 39-byte 協定與硬體映射：

```python
import time
import serial
import numpy as np

class Servo2040Driver:
    SET_CMD = 0xD3
    GET_CMD = 0xC7
    JOINT_TO_PIN_MAP = [15,16,17, 9,10,11, 3,4,5, 12,13,14, 6,7,8, 0,1,2]

    def __init__(self, port="COM3", baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.03)
        time.sleep(1.2)

    def set_relay(self, enable: bool):
        val = 1 if enable else 0
        self.ser.write(bytearray([self.SET_CMD, 26, 1, val & 0x7F, (val >> 7) & 0x7F]))

    def send_servo_angles(self, radians: np.ndarray):
        degrees = np.rad2deg(radians)
        joint_us = np.clip(1500.0 - degrees * 11.111, 800, 2200).astype(int)
        pulses = [1500] * 18
        for j_idx, pin in enumerate(self.JOINT_TO_PIN_MAP):
            pulses[pin] = joint_us[j_idx]
        pkt = bytearray([self.SET_CMD, 0, 18])
        for us in pulses:
            pkt.extend([us & 0x7F, (us >> 7) & 0x7F])
        self.ser.write(pkt)
```

---

## 7. 實作里程碑與驗證計畫 (Milestones)

專案分六大里程碑推進：

| 里程碑 | 實作目標 | 驗收標準 (Acceptance Criteria) | 預估難度 |
| :--- | :--- | :--- | :---: |
| **M1: chica-server 硬體驗證與舵機標定** | 編譯既有 `chica-server-main` APK 安裝至手機 B，驗證 USB-OTG 自動授權與 18 軸舵機零位 | USB 線插入自動啟動 App，Relay 正常吸合，六足執行 `home` 進入標準零位姿態，電壓/電流回傳正常 | ⭐⭐ |
| **M2: Android ONNX Runtime 邊緣推論** | 在 `chica-server-main` 中加入 `onnxruntime-android`，導入 `hexapod_policy.onnx` 執行推論 | `OnnxLocomotionRunner` 單次推論耗時 $< 2\text{ ms}$，記憶體穩定，可根據虛擬姿態輸出 18 維動作殘差 | ⭐⭐ |
| **M3: 手機 A 駕駛操控與 Telemetry HUD** | 在 `MakeYourPet-DigitalTwin-Server` 建立 Web 雙搖桿介面與 TCP 橋接中繼 | 手機 A 觸控搖桿放手時立即煞車，儀表板 25Hz 即時刷新電壓、電流與觸地狀態，中斷連線 300ms 觸發 Watchdog | ⭐⭐ |
| **M4: 閉環神經步態實機平地行走** | 整合解析三角前饋與 ONNX 殘差神經網路，進行平地直線巡航與旋轉測試 | 機器人平穩前進巡航（速度達 $0.20 \sim 0.25\text{ m/s}$），原地轉向不卡死，欠壓/過流安全保護準確觸發 | ⭐⭐⭐ |
| **M5: 手機 IMU 姿態自平衡與動態適應** | 引入手機 B 內建 IMU 傾角回饋，進行斜坡與碎石路面自適應姿態測試 | 六足於碎石或傾斜地面行走時，能利用殘差網路自適應調整身體水平，防止翻覆 | ⭐⭐⭐⭐ |
| **M6: 正式發行與開機自動化** | 優化 APK 啟動流程與系統自啟動，整合高動態跳躍動作 (`jump_policy.onnx`) | 插上 USB-OTG 自動開機完成閉環，一鍵切換行走/跳躍模式 | ⭐⭐⭐ |
