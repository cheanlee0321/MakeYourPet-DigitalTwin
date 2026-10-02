# 📖 Make Your Pet 六足機器人數位孿生與 AI 步態指令手冊
### Complete CLI Command & Workbench Operation Manual

> **專案儲存庫**：`Make Your Pet Digital Twin`  
> **虛擬環境**：`hexapod_rl_env` (Python 3.12, PyTorch CUDA 13.0, MuJoCo 3.14.0)  
> **更新日期**：2026-09-28  

本手冊彙整專案中所有核心執行程式、訓練管線、評估腳本與電玩級即時 3D 遙控之完整指令、參數說明與鍵盤熱鍵速查表。

---

## 目錄 (Table of Contents)
1. [環境準備與前置命令](#1-環境準備與前置命令)
2. [電玩級 3D 遙控工作台 (`demo.py`)](#2-電玩級-3d-遙控工作台-demopy)
3. [行走步態訓練模組 (`train_walking/`)](#3-行走步態訓練模組-train_walking)
4. [立定跳躍殘差訓練模組 (`train_jumping/`)](#4-立定跳躍殘差訓練模組-train_jumping)
5. [模型評估、驗證與導出](#5-模型評估驗證與導出)
6. [TensorBoard 即時監控](#6-tensorboard-即時監控)
7. [數位孿生 XML 重新編譯與影視腳本](#7-數位孿生-xml-重新編譯與影視腳本)

---

## 1. 環境準備與前置命令

本專案支援使用原生 Windows PowerShell 直接調用專用虛擬環境：

```powershell
# 方式 A：直接使用虛擬環境 Python 執行（推薦，最穩定）
.\hexapod_rl_env\Scripts\python.exe <指令>

# 方式 B：先啟用虛擬環境
.\hexapod_rl_env\Scripts\Activate.ps1
python <指令>
```

---

## 2. 電玩級 3D 遙控工作台 (`demo.py`)

主工作台整合了**全自由度行走步態**與**智慧閉環立定跳躍**，提供電玩手感級的鍵盤實體遙控體驗。

```powershell
# 預設啟動 (4 大主題越野公園 + 2 檔標準巡航 + 自動載入最佳行走與跳躍策略)
.\hexapod_rl_env\Scripts\python.exe demo.py

# 平坦地面模式
.\hexapod_rl_env\Scripts\python.exe demo.py --terrain flat

# 崎嶇碎石地貌 (起伏高度 4.5cm)
.\hexapod_rl_env\Scripts\python.exe demo.py --terrain rough --terrain-height 0.045

# 手動指定行走權重與跳躍權重
.\hexapod_rl_env\Scripts\python.exe demo.py --model models/best_model/best_model.zip --jump-model models/jump_best_model/best_model.zip
```

### CLI 參數旗標一覽

| 參數旗標 | 形態 | 預設值 | 說明 |
| :--- | :---: | :---: | :--- |
| `--model` | `str` | 自動搜尋 | 指定行走步態權重檔案路徑 (`.zip`) |
| `--jump-model` | `str` | 自動搜尋 | 指定立定跳躍殘差策略檔案路徑 (`.zip`) |
| `--terrain` | `choice` | `park` | 地表起伏地貌：`flat`, `bumps`, `blocks`, `park`, `rough`, `slope` |
| `--terrain-height` | `float` | `0.045` | 地表最大高低差 (公尺，支援 $0.01 \sim 0.08\text{m}$) |
| `--lift-femur` | `float` | `None` | 自訂大腿抬升幅度 (rad，越野超高抬腿預設約 $0.55\text{ rad} \approx 31.5^\circ$) |
| `--lift-tibia` | `float` | `None` | 自訂小腿屈折幅度 (rad，越野超高抬腿預設約 $0.36\text{ rad} \approx 20.6^\circ$) |
| `--stochastic` | `flag` | `False` | 啟用隨機採樣推論 (預設為確定性最佳策略) |
| `--no-tracking` | `flag` | `False` | 關閉鏡頭自動追隨機器人 (切換為自由相機) |

### 🎮 實體鍵盤熱鍵速查表

> 💡 **按鍵特性**：方向鍵為「按住前進、放開即停」，毫秒級極致響應，無任何視窗失焦或指令延遲！

| 按鍵 | 操作行為 | 說明 |
| :--- | :--- | :--- |
| **`↑` (方向鍵上)** | **按住前進** / 放開站穩 | 大步前進行走，放開即刻煞車收步站穩 |
| **`↓` (方向鍵下)** | **按住倒退** / 放開站穩 | 平穩倒退行走，放開即刻煞車收步站穩 |
| **`←` (方向鍵左)** | **按住左轉** / 放開停止 | 原地向左轉向 |
| **`→` (方向鍵右)** | **按住右轉** / 放開停止 | 原地向右轉向 |
| **`↑` + `←` / `→`** | **弧形前進轉向** | 流暢實現電玩式圓弧推進轉向 |
| **`Shift` + `↑`** | **渦輪加速衝刺** | 突破當前檔位上限，速度增加並額外拉高步頻 10% |
| **`1` / `NumPad 1`** | **ECO 慢步爬坡** | 步頻 1.0 Hz \| 巡航約 0.15 m/s |
| **`2` / `NumPad 2`** | **NORMAL 標準巡航** | 步頻 1.5 Hz \| 巡航約 0.25 m/s (預設標準) |
| **`3` / `NumPad 3`** | **TURBO 極速狂奔** | 步頻 2.5 Hz \| 巡航約 0.45 m/s |
| **`+` / `-`** 或 **`]` / `[`** | 逐級加檔 / 降檔 | 於 1~3 檔間遞增或遞減速度 |
| **`4` / `NumPad 4`** | **越野挺身姿態 (Toggle)** | 單擊切換【OFFROAD 越野挺身姿態】(超高離地避障)，再按切換回【一般基準姿態】 |
| **`5` / `NumPad 5`** | **50cm 爆發大跳** | 激發 Mode 1 跳躍（深蹲蓄力 $\rightarrow$ 爆發蹬地 $\rightarrow$ 空中自適應平衡 $\rightarrow$ 著地柔順吸震） |
| **`6` / `NumPad 6`** | **70cm 火箭超跳** | 激發 Mode 2 極限全功率超跳（飛越人身腰部高度） |
| **`Space` (空白鍵)** | 暫停 / 繼續 | 凍結或恢復物理動態模擬 |
| **`R` / `Backspace`** | 快速重置 | 機器人立即復位回起點中央 |
| **`T`** | 鏡頭追蹤開關 | 切換視角跟隨 (Tracking ON / Free Camera) |
| **滑鼠操作** | 視角平移與施力 | 左鍵旋轉、右鍵平移、滾輪縮放；`Ctrl + 右鍵拖曳` 可對機體施加外力干擾測試平衡 |

---

## 3. 行走步態訓練模組 (`train_walking/`)

負責訓練六足機器人從解析三角步態逆向運動學（Feedforward）出發，學習應對任意向速度指令 $[v_x, v_y, \omega_z]$、複雜地形障礙與外力干擾的 18 自由度殘差網絡。

```powershell
# 1. 標準百萬步行走訓練 (12 個平行環境，平坦地面)
.\hexapod_rl_env\Scripts\python.exe train_walking/train.py --timesteps 1000000 --num-envs 12

# 2. 越野公園地貌複合進階訓練
.\hexapod_rl_env\Scripts\python.exe train_walking/train.py --timesteps 1500000 --num-envs 12 --terrain park --terrain-height 0.035

# 3. 接續先前權重繼續強化訓練
.\hexapod_rl_env\Scripts\python.exe train_walking/train.py --resume models/best_model/best_model.zip --timesteps 500000
```

### `train_walking/train.py` 常用參數

| 參數 | 預設值 | 說明 |
| :--- | :---: | :--- |
| `--timesteps` | `1000000` | 總訓練步數 |
| `--num-envs` | `12` | 平行物理模擬環境進程數 (`SubprocVecEnv`) |
| `--device` | `cpu` | 計算裝置 (`cpu` 或 `cuda`，小尺寸 MLP 多核心 CPU 吞吐量最高) |
| `--lr` | `3e-4` | PPO 學習率 |
| `--n-steps` | `1024` | 每個環境單輪採樣步數 (總緩衝區 = `num-envs` $\times$ `n-steps`) |
| `--batch-size` | `256` | 每次梯度更新之 Mini-batch 大小 |
| `--ent-coef` | `0.008` | 熵懲罰係數 (鼓勵多方向指令與煞車探索) |
| `--terrain` | `flat` | 地形類型 (`flat`, `bumps`, `blocks`, `park`, `rough`, `slope`) |
| `--terrain-height` | `0.025` | 訓練地表起伏最大高度 (m) |
| `--no-domain-rand` | `False` | 關閉領域隨機化 (關閉質量與摩擦力隨機擾動) |

---

## 4. 立定跳躍殘差訓練模組 (`train_jumping/`)

以數字 5 鍵標稱跳躍軌跡為參考前饋，利用強化學習自主學習騰空姿態陀螺儀抑制、對稱發力微調與著地動態柔順阻尼。

```powershell
# 1. 啟動標準立定跳躍殘差訓練 (8 個平行進程，30 萬步約 3.5 分鐘完訓)
.\hexapod_rl_env\Scripts\python.exe train_jumping/train_jump.py --timesteps 300000 --num-envs 8

# 2. 微起伏擾動地形抗摔強化訓練 (提升非平坦地面著地平衡能力)
.\hexapod_rl_env\Scripts\python.exe train_jumping/train_jump.py --terrain uneven --terrain-height 0.020 --timesteps 400000

# 3. 調整基準起跳力度 (預設 1.15 對應 5 鍵 50cm 高跳，可調至 1.35 對應火箭超跳)
.\hexapod_rl_env\Scripts\python.exe train_jumping/train_jump.py --power 1.35 --timesteps 300000
```

### `train_jumping/train_jump.py` 常用參數

| 參數 | 預設值 | 說明 |
| :--- | :---: | :--- |
| `--timesteps` | `300000` | 總採樣步數 (約 3,333 次完整起跳-著地回合) |
| `--num-envs` | `8` | 平行物理模擬環境進程數 |
| `--power` | `1.15` | 基準跳躍推力 (1.15 對應 50cm 大跳, 1.35 對應 70cm 超跳) |
| `--residual-scale`| `0.15` | 騰空與吸震期殘差幅度 (rad，約 $\pm 8.6^\circ$) |
| `--terrain` | `flat` | 訓練場景：`flat`, `uneven`, `bumps`, `slope`, `platform` |
| `--eval-freq` | `10000` | 策略評估與保存最佳權重頻率 (步數) |

---

## 5. 模型評估、驗證與導出

### 1. 立定跳躍「開環 vs 閉環」姿態穩定性對比評估
自動輸出起跳高度、空中最大傾角、觸地傾角、著地腿數與存活率完整表格：

```powershell
# 純終端數據評估 (5 回合)
.\hexapod_rl_env\Scripts\python.exe train_jumping/test_jump_policy.py --episodes 5

# 指定模型權重與地形測試
.\hexapod_rl_env\Scripts\python.exe train_jumping/test_jump_policy.py --model models/jump_best_model/best_model.zip --terrain uneven

# 啟用 3D 視窗即時觀察跳躍著地細節
.\hexapod_rl_env\Scripts\python.exe train_jumping/test_jump_policy.py --render --episodes 3
```

### 2. 行走步態指令跟隨 5 項情境驗證
自動依序測試：①煞車待命 $\rightarrow$ ②前進巡航 $\rightarrow$ ③原地左轉 $\rightarrow$ ④原地右轉 $\rightarrow$ ⑤恢復待命：

```powershell
.\hexapod_rl_env\Scripts\python.exe train_walking/verify_command_tracking.py
```

### 3. 開環跳躍雙檔位物理極限測試
快速檢驗 MuJoCo 物理引擎下的垂直初速、滯空時間與電池艙離地淨空：

```powershell
.\hexapod_rl_env\Scripts\python.exe train_jumping/test_jump.py
```

### 4. 跳躍環境單元測試 (5 大地貌自我檢驗)
```powershell
.\hexapod_rl_env\Scripts\python.exe train_jumping/verify_jump_env.py
```

### 5. 步態策略 ONNX 導出 (供部署至邊緣設備如 Raspberry Pi / 伺服控制器)
```powershell
.\hexapod_rl_env\Scripts\python.exe train_walking/export_onnx.py
```

---

## 6. TensorBoard 即時監控

在訓練過程中或訓練完畢後，開啟瀏覽器查看即時獎勵收斂、姿態誤差與價值損失曲線：

```powershell
# 監看行走步態訓練曲線
tensorboard --logdir=tensorboard_logs

# 監看立定跳躍殘差訓練曲線
tensorboard --logdir=tensorboard_logs/jump_ppo
```
> 開啟瀏覽器訪問：`http://localhost:6006`

---

## 7. 數位孿生 XML 重新編譯與影視腳本

### 1. 重新編譯 MuJoCo 高保真全機 XML (`models/hexapod.xml`)
當修改連桿質量、關節阻尼、STL 視覺網格掛載或 18 軸參考點標記時執行：

```powershell
.\hexapod_rl_env\Scripts\python.exe generate_hexapod_xml.py
```

### 2. 錄製關節運動軌跡 (供 Blender 影視級渲染)
```powershell
.\hexapod_rl_env\Scripts\python.exe train_walking/record_trajectory.py
```

### 3. Blender 影視級動畫合成輸出
```powershell
# 呼叫本機 Blender 執行無頭動畫渲染管線 (需依本機 Blender 路徑)
& "C:\Program Files\Blender Foundation\Blender 4.2\blender.exe" -b blender_assets/hexapod_cinematic.blend -P blender_cinematic.py
```

---

## 8. 一分鐘快速上手指令錦囊 (Quick Cheat Sheet)

```powershell
# 🚀 立即開始 3D 電玩式遙控
.\hexapod_rl_env\Scripts\python.exe demo.py

# 🏋️ 重新訓練立定跳躍殘差策略 (3.5 分鐘)
.\hexapod_rl_env\Scripts\python.exe train_jumping/train_jump.py --timesteps 300000

# 📊 檢視跳躍性能評估報表 (平地 vs 越野)
.\hexapod_rl_env\Scripts\python.exe train_jumping/test_jump_policy.py

# 🚶 驗證行走指令聽從度 (直行、倒車、左右原地旋轉)
.\hexapod_rl_env\Scripts\python.exe train_walking/verify_command_tracking.py
```
