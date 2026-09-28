# 📱 ChicaServer 完整架構解析與手機安裝執行指南

> **專案位置**：`chica-server-main/`  
> **適用平台**：Android 6.0+ (API 23 ~ 35)  
> **通訊介面**：USB-OTG (CDC-ACM 115200 8N1) + Wi-Fi TCP Socket (Port 18711)  
> **目標機體**：MakeYourPet 六足機器人 (Chica / Chipo 18-DOF 3DOF Hexapod)  
> **底層硬體**：Pimoroni Servo 2040 (RP2040) 或 Pololu Maestro 系列  

---

## 目錄 (Table of Contents)

1. [專案背景與角色定位](#1-專案背景與角色定位)
2. [ChicaServer 核心運作原理 (How it Works)](#2-chicaserver-核心運作原理-how-it-works)
   - 2.1 [系統整體通訊拓撲](#21-系統整體通訊拓撲)
   - 2.2 [子系統一：TCP 網路通訊層 (Port 18711)](#22-子系統一tcp-網路通訊層-port-18711)
   - 2.3 [子系統二：機器人大腦控制中樞與狀態機 (ChicaController)](#23-子系統二機器人大腦控制中樞與狀態機-chicacontroller)
   - 2.4 [子系統三：C++ NDK 高效運動學與步態引擎 (Gait Engine)](#24-子系統三c-ndk-高效運動學與步態引擎-gait-engine)
   - 2.5 [子系統四：硬體 USB CDC 串列驅動與 39-Byte 二進位協定](#25-子系統四硬體-usb-cdc-串列驅動與-39-byte-二進位協定)
   - 2.6 [子系統五：手機內建 IMU 感測器姿態融合 (Orientation Sensor)](#26-子系統五手機內建-imu-感測器姿態融合-orientation-sensor)
3. [手機安裝與編譯部署步驟 (Step-by-step Installation)](#3-手機安裝與編譯部署步驟-step-by-step-installation)
   - 3.1 [事前準備（手機端開發者模式與 USB 偵錯）](#31-事前準備手機端開發者模式與-usb-偵錯)
   - 3.2 [方法一：使用 Android Studio 編譯安裝（最推薦、零配置負擔）](#32-方法一使用-android-studio-編譯安裝最推薦零配置負擔)
   - 3.3 [方法二：使用終端命令列 (CLI) 與 Gradle Wrapper 編譯](#33-方法二使用終端命令列-cli-與-gradle-wrapper-編譯)
   - 3.4 [方法三：直接側載安裝 (Sideloading Prebuilt APK)](#34-方法三直接側載安裝-sideloading-prebuilt-apk)
4. [首次執行與實體機器人連線驗證 (First Run & Hardware Wiring)](#4-首次執行與實體機器人連線驗證-first-run--hardware-wiring)
   - 4.1 [硬體接線與 USB-OTG 自動授權](#41-硬體接線與-usb-otg-自動授權)
   - 4.2 [App 介面判讀與狀態指標](#42-app-介面判讀與狀態指標)
   - 4.3 [使用電腦終端機或 Python 發送控制指令](#43-使用電腦終端機或-python-發送控制指令)
   - 4.4 [常用控制指令速查表](#44-常用控制指令速查表)
5. [硬體參數校準檔 (`config-2040.txt`) 說明](#5-硬體參數校準檔-config-2040txt-說明)
6. [與數位孿生 (RL ONNX) 策略的整合藍圖](#6-與數位孿生-rl-onnx-策略的整合藍圖)

---

## 1. 專案背景與角色定位

在 Make Your Pet 六足機器人體系中，**機器人不需要昂貴笨重的工控機（如樹莓派或 Jetson Nano）**，而是直接將一台常見的 **Android 智慧型手機** 固定在機身背上作為機器人的「運算大腦與感測器中樞」：

* **原版 Chica Server**：MakeYourPet 官方原本發布的專有閉源 Android App，內建人臉追蹤等功能，但未開源其運動學核心。
* **本專案 `chica-server-main`**：由開源社群進行 1:1 精準逆向重構（Clean-room Reconstruction）的完整開源版本（採 GPLv3 授權）。移除了原版捆綁的 Google 專有閉源 ML Kit 人臉模型，完全聚焦於**六足運動學解算、硬體 USB 封包通訊、TCP 控制伺服器、手機 IMU 感測器融合與即時保護機制**。

---

## 2. ChicaServer 核心運作原理 (How it Works)

### 2.1 系統整體通訊拓撲

```
┌─────────────────────────────────┐
│     外部控制終端 (Client)       │
│  - 手機 A 遙控端 App            │
│  - 電腦 Python 腳本 / 終端      │
│  - 強化學習推論中繼節點         │
└────────────────┬────────────────┘
                 │ TCP Socket (Port 18711, Wi-Fi 區域網路)
                 │ 傳輸純文字字串指令 (如 "torque\n", "walk2:0,0.3,0\n")
                 ▼
┌────────────────────────────────────────────────────────┐
│             Android 手機 (ChicaServer)                 │
│                                                        │
│  [1] OriginalTcpControlServer.java                     │
│      - 監聽 0.0.0.0:18711，解析指令並回傳狀態行        │
│                                                        │
│  [2] ChicaController.java (大腦協調中樞)               │
│      - 狀態機 (站立/坐下/步態模式/定高平衡/安全守門)    │
│      - 心跳監控線程 (1~7ms 週期刷新舵機與採集 ADC)     │
│      - 電池低壓 (<6V) / 過流 (>10A) 硬體斷電保護        │
│                                                        │
│  [3] C++ NDK 步態與反向運動學 (chica_gait_jni / apk_model) │
│      - 50Hz 實時三角步態 (Tripod) 與多足協調生成      │
│      - 3DOF 逆運動學 (IK)：幾何坐標 -> 18 個關節角度  │
│                                                        │
│  [4] OriginalOrientationSensor.java                    │
│      - 讀取手機 Gravity + Magnetic 感測器融合姿態      │
│                                                        │
│  [5] UsbSerialServoBackend.java                        │
│      - CDC-ACM 115200 8N1 驅動                         │
│      - ServoPacketEncoder 編碼為 39 位元組二進位封包   │
└────────────────┬───────────────────────────────────────┘
                 │ USB-OTG 串列傳輸線 (115200 Baud, 8N1)
                 │ 發送 0xD3 舵機脈衝 / 接收 0xC7 電壓電流遙測
                 ▼
┌────────────────────────────────────────────────────────┐
│      實體底盤控制板 (Pimoroni Servo 2040)              │
│  - RP2040 雙核心 MCU (跑微秒級 PIO PWM 韌體)           │
│  - Pin 0 (A0)：繼電器開關 (Relay 舵機供電總閥)         │
│  - Pin 1~18：18 組 50Hz PWM 輸出 (1000~2000µs)        │
│  - Pin 19~25：6 足觸地開關 + 電壓分壓 ADC + 電流分流 ADC │
└────────────────┬───────────────────────────────────────┘
                 │ 18 組舵機控制訊號線
                 ▼
       [18 顆金屬齒輪數位伺服舵機]
       (每腿 3 軸：Coxa 髖水平、Femur 大腿俯仰、Tibia 小腿開合)
```

---

### 2.2 子系統一：TCP 網路通訊層 (Port 18711)

檔案：`app/src/main/java/com/makeyourpet/chicaserver/protocol/OriginalTcpControlServer.java`

* **監聽端口**：預設綁定所有網路介面的 `18711` 埠 (`0.0.0.0:18711`)。
* **交握協議**：
  1. 客戶端一旦 TCP 連線成功，伺服器立即主動推送一行動態遙測狀態行：
     ```text
     ready:BPS= 92|V= 7.98|I= 0.25|IP=192.168.1.50|LEGS=------|FLAGS=110000100
     ```
  2. 客戶端每次送出一行文字指令（以換行符 `\n` 結尾），伺服器處理後立即回應狀態行：
     * `ready:...`：指令已接收執行，伺服器處於可用狀態。
     * `busy:...`：機器人當前正在執行不可中斷的過渡動作，提示客戶端稍後重試。
* **遙測狀態各欄位意義**：
  * `BPS`：每秒與硬體通訊成功的心跳次數（板端輪詢率，正常約 80~100 BPS）。
  * `V`：當前鋰電池電壓（伏特，如 7.98V；無硬體時為 `---`）。
  * `I`：全機總電流（安培，如 0.25A）。
  * `IP`：手機當前分配到的 Wi-Fi IP。
  * `LEGS`：6 隻腳的接地開關狀態（`x` 代表踩地接觸，`-` 代表懸空離地）。
  * `FLAGS`：9 碼二進位/狀態旗標，依序代表：
    1. `relay`：舵機總電源繼電器（1=通電，0=斷電）
    2. `standing`：是否處於站立姿態（1=站立，0=趴下坐著）
    3. `keep`：是否鎖定當前姿態
    4. `crab`：橫移步態模式（Crab Mode）
    5. `mode`：當前運動模式編號（0=Standard, 1=Race, 2=Offroad, 3=Custom, 4=Quad）
    6. `level`：主動地貌自動水平維持開關
    7. `autoSit`：閒置超時自動坐下保護開關
    8. `block`：方塊摺疊收納模式
    9. `calibPosition`：九十度零位機械校準姿態

---

### 2.3 子系統二：機器人大腦控制中樞與狀態機 (ChicaController)

檔案：`app/src/main/java/com/makeyourpet/chicaserver/control/ChicaController.java`

這是整個軟體的核心協調器：
1. **多執行緒架構**：
   * **命令執行緒 (`originalCommandExecutor`)**：單執行緒佇列，保證所有步態切換、姿勢調整指令依序安全執行，避免並行衝突。
   * **硬體監控心跳執行緒 (`chica-hardware-monitor`)**：以 1ms 解析度輪詢，每隔 7ms 觸發一次硬體寫入（將計算好的最新 18 關節脈寬刷入 USB 暫存器），且每隔兩次心跳（約 14ms）發起一次類比 ADC（電壓、電流、足端接觸）查詢。
   * **蜂鳴器警報執行緒 (`chica-original-beeper`)**：負責產生節奏性的聲響回饋（開機成功嗶 1 聲、硬體失敗嗶 6 聲、低電壓警報等）。
2. **自動斷電與硬體安全保護 (Safety Guard)**：
   * **低電壓切斷 (Under-voltage Cutoff)**：鋰電池電壓低於 `6.4V` 持續 2 秒觸發蜂鳴器警報；低於 `6.0V` 時立即自動切斷 Relay 繼電器，強制洩力保護鋰電池不被過放損壞。
   * **過電流切斷 (Over-current Cutoff)**：全機總電流超過 `8.0A` 持續 2 秒警報；超過 `10.0A` 立即切斷 Relay，避免大負載舵機堵轉燒毀或造成控制板重啟。
   * **閒置超時自動坐下 (Auto-Sit)**：若客戶端連續送出 60 次 `ack` 輪詢（約數秒無任何控制指令），機器人會優雅地自動平穩坐下並切斷舵機電源，防止舵機長時間通電發燙。

---

### 2.4 子系統三：C++ NDK 高效運動學與步態引擎 (Gait Engine)

檔案：
* JNI 介面：`app/src/main/cpp/chica_gait_jni.cpp`
* 運動學核心：`app/src/main/cpp/apk_model.cpp`、`apk_model.h`
* 脈寬轉換：`app/src/main/cpp/pulse_conversion.cpp`

六足機器人擁有 18 個自由度（每腿 3 個自由度：Coxa 髖水平擺動、Femur 大腿垂直俯仰、Tibia 小腿垂直開合）。

#### 為什麼使用 C++ 原生庫？
在高頻（50Hz / 每 20ms）運動迴圈中，需要同時計算 6 條腿的足端軌跡樣條、空間坐標轉換、反向運動學三角函數解算（Inverse Kinematics）以及 18 路 PWM 微秒值轉換。在 Java 層運算極易因垃圾回收（Garbage Collection, GC）造成微小的毫秒級卡頓，導致實體機器人行走時出現抖動。因此核心數學運算全由 C++17 編寫。

#### 步態算法原理：
* **三角步態 (Tripod Gait)**：將 6 條腿分為兩組交替運動（組一：L1, R2, L3；組二：R1, L2, R3）。當一組處於擺動相（Swing Phase）向前跨步並抬起時，另一組處於支撐相（Stance Phase）著地蹬地向後推動機身。
* **空間幾何逆解 (IK)**：
  輸入目標足端世界坐標 $(X_{foot}, Y_{foot}, Z_{foot})$ 及軀幹姿態 $(Roll, Pitch, Yaw)$，透過餘弦定理與三角投影，解析解出：
  $$\theta_{coxa} = \text{atan2}(Y, X)$$
  $$\theta_{femur}, \theta_{tibia} = \text{IK\_2DOF\_Trig}(\Delta r, \Delta z, L_{femur}, L_{tibia})$$
* **角度轉脈衝 (Angle to Pulse)**：
  `pulse_conversion.cpp` 讀取 `config-2040.txt` 中的伺服零位與斜率，將度數精準映射至 $1000 \sim 2000\ \mu s$ 的整數脈寬。

---

### 2.5 子系統四：硬體 USB CDC 串列驅動與 39-Byte 二進位協定

檔案：
* `app/src/main/java/com/makeyourpet/chicaserver/hardware/UsbSerialServoBackend.java`
* `app/src/main/java/com/makeyourpet/chicaserver/hardware/ServoPacketEncoder.java`

Android 手機透過 USB-OTG 傳輸線連接底層 Pimoroni Servo 2040 板（CDC-ACM 虛擬串列埠，鮑率 115200，8N1）。

#### 二進位指令協定解密：
1. **舵機脈寬控制下行封包 (`0xD3` SET 指令，長度 39 Bytes)**：
   * `Byte 0`：`0xD3`（指令碼，最高位 MSB=1）
   * `Byte 1`：`0`（起始通道號）
   * `Byte 2`：`18`（更新的舵機通道數量）
   * `Byte 3 ~ 38`：18 組 14-bit 脈衝值。
     * **7-bit 防混淆編碼**：為了防止脈衝數值中出現 `0xD3` 等特徵碼導致硬體誤判，每個 14-bit 數值拆解為兩個 7-bit 位元組（最高位強制為 0）：
       $$\text{LowByte} = \text{Pulse} \ \& \ \text{0x7F}$$
       $$\text{HighByte} = (\text{Pulse} \gg 7) \ \& \ \text{0x7F}$$
2. **遙測採集下行請求 (`0xC7` GET 指令)**：
   * 發送 `0xC7 <起始針腳> <數量>`
3. **繼電器電源開關 (`0xD3` 單通道數位輸出)**：
   * 發送 `[0xD3, Pin(26), 1, Value, 0]`，即時導通或切斷總電源。

---

### 2.6 子系統五：手機內建 IMU 感測器姿態融合 (Orientation Sensor)

檔案：`app/src/main/java/com/makeyourpet/chicaserver/OriginalOrientationSensor.java`

* **感測器種類**：註冊 Android 系統的 `Sensor.TYPE_GRAVITY`（重力加速度）與 `Sensor.TYPE_MAGNETIC_FIELD`（地磁感測器）。
* **姿態解算**：
  呼叫 `SensorManager.getRotationMatrix()` 與 `SensorManager.getOrientation()`，以低通濾波器（LERP 遞迴平滑）計算出手機當前相對於大地的俯仰角 (Pitch)、翻滾角 (Roll) 與方位角 (Heading)。
* **應用場景**：
  當發送 `level` 指令時，ChicaServer 會啟動動態地貌水平維持，以反方向傾斜軀幹來抵消地面的坡度，使六足機器人在斜坡上行走時機身仍維持水平！

---

## 3. 手機安裝與編譯部署步驟 (Step-by-step Installation)

安裝 ChicaServer 到機器人專用手機上共有三種途徑。推薦依自身環境選擇：

```
                    ┌────────────────────────┐
                    │   你打算如何安裝？     │
                    └───────────┬────────────┘
                                │
        ┌───────────────────────┼────────────────────────┐
        ▼                       ▼                        ▼
【方法一：Android Studio】  【方法二：CLI 命令行編譯】   【方法三：直接安裝 APK】
適合具備電腦 IDE 的開發者    適合進階開發者 / CI 環境    已有 apk 檔時最快
(自動配置 SDK/NDK/ADB)      (需手動安裝 JDK/SDK)        (只需手機傳輸與點擊)
```

---

### 3.1 事前準備（手機端開發者模式與 USB 偵錯）

無論採取哪種方法，安裝到 Android 手機前請先解鎖權限：

1. **開啟開發者人員選項**：
   * 打開手機「設定」 -> 「關於手機」 (About Phone)。
   * 連續快速點擊「版本號碼」 (Build Number) **7 次**，直到畫面出現「您已成為開發人員！」。
2. **開啟 USB 偵錯**：
   * 回到「設定」 -> 「系統」或「更多設定」 -> 進入「開發人員選項」 (Developer Options)。
   * 開啟 **「USB 偵錯」 (USB Debugging)**。
   * 若為小米/POCO/紅米等 MIUI/HyperOS 機種，需額外開啟 **「USB 安裝」** 與 **「USB 偵錯 (安全設定)」**。
3. **準備 USB-OTG 轉接頭**：
   * 運行時手機需透過 USB-OTG 轉接線接上機板的 USB-C 埠（手機扮演 USB Host 主機角色）。

---

### 3.2 方法一：使用 Android Studio 編譯安裝（最推薦、零配置負擔）

Android Studio 會自動下載適配的 JDK、Android SDK Platform 35、CMake 3.22.1 與 NDK `29.0.14206865`，完全不需手動配置環境變數。

1. **下載並安裝 Android Studio**：
   * 前往 [Android Studio 官方網站](https://developer.android.com/studio) 下載安裝最新版本（Ladybug 或更高版本）。
2. **開啟專案**：
   * 啟動 Android Studio，點擊 **Open**。
   * 選取專案中的子目錄：`Make Your Pet Digital Twin/chica-server-main`。
3. **等待 Gradle 同步完成**：
   * 首次打開時，右下角會顯示 Gradle Sync 進度，自動拉取所需的依賴函式庫（如 `usb-serial-for-android:3.10.0`）。
   * 若提示缺少 NDK `29.0.14206865` 或 SDK Platform 35，直接點擊視窗內的藍色連結 **"Install missing platform(s) and sync project"**，IDE 將全自動下載安裝。
4. **連接手機並執行**：
   * 用傳輸線將 Android 手機連接到電腦。
   * 手機螢幕若彈出「允許 USB 偵錯嗎？」，勾選「一律允許」並點擊「允許」。
   * 在 Android Studio 上方工具列的裝置下拉選單中，選中你的手機名稱。
   * 點擊綠色的執行箭頭按鈕 **▶️ (Run 'app')**。
   * 編譯完成後，App 將自動安裝至手機並在螢幕上以全螢幕橫向啟動！

---

### 3.3 方法二：使用終端命令列 (CLI) 與 Gradle Wrapper 編譯

若希望使用 PowerShell 或 Bash 自動化編譯：

#### 步驟 1：確認電腦安裝 JDK 17
* 本專案要求 JDK 17 ~ 25。可從 [Eclipse Temurin](https://adoptium.net/) 下載安裝 OpenJDK 17 LTS，並確認 `java -version` 輸出正常。

#### 步驟 2：配置 Android SDK 路徑
在 `chica-server-main/` 目錄下建立或檢查 `local.properties` 檔案，寫入你的 Android SDK 本機路徑：
* **Windows 範例**：
  ```properties
  sdk.dir=C\:\\Users\\<你的使用者名稱>\\AppData\\Local\\Android\\Sdk
  ```
* **macOS 範例**：
  ```properties
  sdk.dir=/Users/<username>/Library/Android/sdk
  ```
* **Linux 範例**：
  ```properties
  sdk.dir=/home/<username>/Android/Sdk
  ```

#### 步驟 3：安裝固定版本的 NDK 與 CMake
透過 SDK Manager 下載本專案鎖定的 NDK 與編譯工具：
```bash
sdkmanager "platforms;android-35" "ndk;29.0.14206865" "cmake;3.22.1"
```

#### 步驟 4：執行 Gradle 編譯 Debug APK
進入 `chica-server-main` 資料夾：
```powershell
# Windows PowerShell
cd "chica-server-main"
.\gradlew.bat :app:assembleDebug
```
```bash
# Linux / macOS
cd chica-server-main
chmod +x gradlew
./gradlew :app:assembleDebug
```

編譯成功後，產物 APK 位於：
```
chica-server-main/app/build/outputs/apk/debug/app-debug.apk
```

#### 步驟 5：透過 ADB 安裝至手機
將手機插上電腦並執行：
```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

---

### 3.4 方法三：直接側載安裝 (Sideloading Prebuilt APK)

若已有產出的 `app-debug.apk`，無需連接電腦開發環境：

1. **傳送 APK 檔案至手機**：
   * 透過 USB 傳輸線複製、通訊軟體傳送、或透過手機瀏覽器直接下載 APK 檔案到手機的「下載」 (Download) 資料夾。
2. **安裝未知應用程式**：
   * 在手機檔案管理員中點擊 `app-debug.apk`。
   * 若系統提示「基於安全性考量，您的手機目前不允許安裝來源不明的應用程式」，點擊「設定」，開啟「允許這個來源的應用程式」。
3. **完成安裝**：
   * 點擊「安裝」，完成後點擊「開啟」。

---

## 4. 首次執行與實體機器人連線驗證 (First Run & Hardware Wiring)

### 4.1 硬體接線與 USB-OTG 自動授權

1. **連接 Wi-Fi**：
   * 讓手機連上與你的電腦（或遙控手機 A）**相同的 Wi-Fi 區域網路**。
2. **開啟 ChicaServer App**：
   * 首次開啟時，若尚未連接硬體，介面左上方會顯示硬體狀態為 `virtual`（虛擬模擬模式，不會閃退崩潰）。
3. **插上 Servo 2040 機板**：
   * 使用 **USB-OTG 轉接頭** 將 Pimoroni Servo 2040 的 USB-C 插入手機。
   * **關鍵彈窗**：Android 系統會立即觸發 `USB_DEVICE_ATTACHED` 系統廣播，並彈出視窗：
     > **「是否要開啟 ChicaServer 來處理這個 USB 裝置？」**
   * 請務必勾選 **「一律在連接此裝置時開啟 ChicaServer」**，然後點擊 **確定 (OK)**。
   * 此時 App 獲得 Android 系統底層的 Direct USB 存取權限。
4. **聽取蜂鳴器提示**：
   * 若成功連接 Servo 2040，手機會發出 **1 聲清脆嗶聲**；若連接失敗則會連續嗶 6 聲。

---

### 4.2 App 介面判讀與狀態指標

全螢幕介面提供清晰的 HUD 即時狀態資訊：

```
┌─────────────────────────────────────────────────────────────┐
│ ChicaServer v0.0.5-alpha                   IP: 192.168.1.50 │
│ BPS: 95 | V: 7.95V | I: 0.32A | Status: STANDING           │
│                                                             │
│                                                             │
│                    [ 六足姿態俯視預覽圖 ]                   │
│                                                             │
│                                                             │
│ [BLOCK]      [TORQUE]      [CONFIG]      [CAMERA]    [EXIT] │
└─────────────────────────────────────────────────────────────┘
```

* **IP 顯示**：頂部顯示當前手機的 Wi-Fi IP 位址（例如 `192.168.1.50`），這是外部連線的位址。
* **按鈕控制**：
  * `TORQUE`：手動切換舵機總電源繼電器（點亮時代表舵機已通電鎖死）。
  * `BLOCK`：將所有腿部收攏緊貼機身（方塊收納形態）。
  * `CONFIG`：彈出參數設定編輯器，可直接修改零位微調。
  * `CAMERA`：預留鏡頭畫面切換。
  * `EXIT`：安全關閉伺服器並釋放硬體埠離開。

---

### 4.3 使用電腦終端機或 Python 發送控制指令

手機維持開啟 ChicaServer，在同一個 Wi-Fi 網路下的電腦開啟終端機進行測試：

#### 終端機測試 (PowerShell / Linux Netcat)
```bash
# 透過 nc (Netcat) 連線至手機 (假設手機 IP 為 192.168.1.50)
nc 192.168.1.50 18711

# 連線後手機會立即回傳第一行：
# ready:BPS= 92|V= 7.98|I= 0.25|IP=192.168.1.50|LEGS=------|FLAGS=000000100

# 1. 啟用舵機供電 (Relay 開啟)
torque

# 2. 機器人站立
sit

# 3. 以速度 0.3 向前行走
walk2:0,0.3,0

# 4. 停止行走
walkclear

# 5. 坐下休息
sit

# 6. 離開連線
bye
```

#### 即開即用 Python 控制腳本
專案內可直接以 Python 進行自動化控制，將以下程式碼儲存為 `test_chica.py` 執行：

```python
import socket
import time

CHICA_IP = "192.168.1.50"  # 請替換為你手機畫面上顯示的 IP
CHICA_PORT = 18711

def send_cmd(sock, cmd):
    print(f">> 發送指令: {cmd}")
    sock.sendall((cmd + "\n").encode("utf-8"))
    response = sock.recv(1024).decode("utf-8", errors="ignore")
    print(f"<< 伺服器回傳: {response.strip()}")
    return response

def main():
    print(f"連線至 ChicaServer ({CHICA_IP}:{CHICA_PORT})...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((CHICA_IP, CHICA_PORT))
        # 讀取連線初始狀態行
        init_status = s.recv(1024).decode("utf-8")
        print(f"連線建立成功: {init_status.strip()}")

        time.sleep(1)
        # 1. 上電
        send_cmd(s, "torque")
        time.sleep(1)

        # 2. 站立
        send_cmd(s, "sit")  # sit 為站立/趴下切換開關
        time.sleep(2)

        # 3. 前進 3 秒 (轉向=0, 前進=0.3, 動畫=0)
        send_cmd(s, "walk2:0,0.3,0")
        time.sleep(3)

        # 4. 停步
        send_cmd(s, "walkclear")
        time.sleep(1)

        # 5. 坐下並關閉動力
        send_cmd(s, "sit")
        time.sleep(1)
        send_cmd(s, "torque")

        # 6. 結束連線
        send_cmd(s, "bye")
        print("測試完成！")

if __name__ == "__main__":
    main()
```

---

### 4.4 常用控制指令速查表

| 指令群組 | 指令字串 | 功能說明 |
| :--- | :--- | :--- |
| **電源與姿態** | `torque` | 切換舵機主繼電器供電 (On/Off) |
| | `sit` | 切換站立姿態 / 趴下休息姿態 |
| | `home` | 回到預設零位姿態 |
| | `autosit` | 切換閒置超時自動坐下功能 |
| | `block` | 切換方塊收納姿態 (六腿緊縮) |
| **步態運動** | `walk2:<turn>,<fwd>,<anim>` | 經典三角步態行走（數值範圍 -1.0 ~ 1.0） |
| | `walk1:<turn>,<fwd>,<anim>` | 緩慢穩健步態 |
| | `walk3:<turn>,<fwd>,<anim>` | 快速步態 |
| | `walkclear` | 即刻減速煞停 |
| | `crab` | 切換橫移模式（原本的 turn 轉變為橫向平移） |
| **機身姿態調整** | `setxy:<x>,<y>` | 調整機身水平位移 (X, Y) |
| | `setzu:<z>,<u>` | 調整機身高度與俯仰 |
| | `setclear` | 清除所有姿態偏移量 |
| **環境自適應** | `level` | 切換主動水平維持（利用手機 IMU 抗傾斜） |
| | `bounce` | 展示機身律動彈跳動作 |
| | `beep` | 讓手機發出提示音 |
| **校準模式** | `calibpos` | 進入全舵機 90 度校準校正姿態 |

---

## 5. 硬體參數校準檔 (`config-2040.txt`) 說明

配置檔案位於 `app/src/main/assets/config-2040.txt`，在 App 運行時亦可點擊畫面上的 **CONFIG** 按鈕直接即時編輯並儲存：

1. **舵機插針與微秒校準 (Servo Pins & Calibration)**：
   ```text
   # 格式: [關節名稱] [針腳號] [-45度微秒值] [+45度微秒值]
   L11 P15 2000 1000
   L12 P16 2000 1000
   L13 P17 2000 1000
   ```
   * `L11` 代表左前腿 Coxa，接在 Servo 2040 的 `P15` 插針。
   * 若組裝後某個關節運動方向相反，只需在該行將 `2000 1000` 對調改成 `1000 2000` 即可！
2. **繼電器與感測器接腳**：
   ```text
   RELAY P26 1     # 繼電器訊號接在 P26 (A0)，高電位導通
   WARN_VOL 2 6.4 6 3   # 電壓低於 6.4V 警報，低於 6.0V 強制切斷
   WARN_CUR 2 8 10 3    # 電流超過 8A 警報，超過 10A 強制切斷
   ```
3. **機械幾何尺寸 (mm)**：
   ```text
   COXA_LEN 43     # 髖部關節長度 43mm
   FEMUR_LEN 80    # 大腿長度 80mm
   TIBIA_LEN 134   # 小腿長度 134mm
   ```

---

## 6. 與數位孿生 (RL ONNX) 策略的整合藍圖

在我們的完整專案架構中，`chica-server-main` 具備雙重戰略價值：

1. **現成的實體驗證底座 (Baseline Controller)**：
   你可以直接將此 App 裝上機器人，驗證 18 顆金屬舵機的組裝接線、電源穩定性、足端著地開關與電池續航力，不需重新發明輪子。
2. **AI 強化學習策略 (ONNX) 的推論載體**：
   * **方案 A（TCP 網路中繼推論）**：機身手機運行 ChicaServer，另外一台手機或隨身微型電腦執行 `hexapod_policy.onnx`，推論出運動指令後透過 TCP 18711 傳送 `walk2:...` 控制機體。
   * **方案 B（原生嵌入 ONNX Runtime）**：在 `chica-server-main` 的 C++ Gait Engine 中引入 **ONNX Runtime Mobile (C++ API)**，直接在 App 內部讀取 IMU 與關節歷史向量，以 50Hz 頻率執行我們的強化學習神經網路，實現完全獨立自主的邊緣 AI 仿生行走行態！
