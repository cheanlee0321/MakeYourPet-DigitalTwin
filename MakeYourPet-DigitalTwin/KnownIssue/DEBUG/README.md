# DEBUG 資料夾檔案索引與調試紀錄說明

本資料夾彙整了專案在**模型幾何校正**、**關節坐標轉換**、**姿態調試**與**早實驗證**過程中所使用的調試腳本、產生的對比照片與診斷日誌。

---

## 📂 檔案分類清單

### 1. 3D 列印 STL 幾何投影與連桿特徵調試 (Mesh Projections & Keypoints)
- **腳本**：
  - `plot_raw_femur.py`：讀取原廠 `left-femur.stl` 繪製三軸投影圖
  - `plot_raw_tibia.py`：讀取原廠 `left-tibia.stl` 繪製三軸投影圖
  - `plot_annotated_stls.py`：3D 視覺化標記大腿與小腿的特徵定位點
- **照片**：
  - `raw_femur_projections.png`
  - `raw_tibia_projections.png`
  - `stls_with_annotations.png`
  - `femur_projections.png`
  - `coxa_projections.png`

### 2. 膝關節與小腿裝甲對齊調試 (Knee Joint & Tibia Shield Alignment)
- **腳本**：
  - `inspect_knee.py`：膝關節截面與連桿咬合檢查
  - `inspect_l2_chain.py`：左中足 (L2) 運動鏈局部視角檢查
  - `sweep_fine_knee.py`：膝關節微調參數網格搜尋
  - `sweep_tibia_x.py`：小腿 X 軸偏移量掃描
  - `render_super_knee.py`：膝關節超特寫高解析度算圖
  - `generate_knee_fix_comparison.py`：產出修復前後膝關節比對圖
  - `test_femur_quat.py`：大腿四元數旋轉候選驗證
  - `test_knee_candidates.py`：膝關節 Euler 角度候選驗證
  - `test_knee_pos.py`：膝關節位置對比測試
- **照片**：
  - `femur_euler_candidates.png`
  - `femur_quat_comparison.png`
  - `femur_tibia_connection_fixed.png`
  - `fine_knee_sweep.png`
  - `knee_joint_inspection.png`
  - `knee_pos_comparison.png`
  - `super_closeup_knee.png`
  - `tibia_x_sweep.png`
  - `tibia_shield_fixed_comparison.png`
  - `tibia_shield_lr_comparison.png`
  - `shield_mirror_test.png`
  - `shield_tibia_overlay.png`

### 3. 足端橡膠緩衝球同心校準 (Foot Tip Concentricity Calibration)
- **腳本**：
  - `test_tip_candidates.py`：足尖位置候選參數掃描
  - `generate_tip_comparison.py`：產出足尖校準前後對比圖
  - `verify_all_tips.py`：全機 6 足足尖同心同軸自動驗證
- **照片與模型**：
  - `foot_tip_candidates.png`
  - `foot_tip_calibration_comparison.png`
  - `all_foot_tips_aligned.png`
  - `foot_tip_crop.png`
  - `foot_tip_misalignment.png`
  - `test_tip.xml`

### 4. 全機/單腿原型動態檢視與手動調試 (Interactive Prototyping & Workbench)
- **腳本**：
  - `calibrate_hexapod.py`：18 軸手動滑桿互動調試控制台（含 XYZ 3D 坐標軸指示與推撞測試）
  - `view_one_leg.py`：早期階段 1.1 單腿動態預覽
  - `view_hexapod.py`：早期階段 1.3 全機 Z 字折線姿態展示
  - `render_updated_model.py`：模型更新動態渲染預覽
  - `test_full_calibration.py`：全腿動態物理校準測試
- **照片**：
  - `calibrated_femur_render.png`
  - `current_robot_render.png`
  - `frame_top_view.png`
  - `full_leg_calibration_test.png`
  - `l2_crop_top.png`
  - `r2_crop_top.png`
  - `xyz_compass_preview.png`
  - `all_shields_fixed_top.png`
  - `femur_candidates_comparison.png`
  - `femur_crop.png`

### 5. 早期環境與開環運動學驗證 (Benchmarks & Early Verification)
- **腳本**：
  - `verify_stage0.py`：階段 0 硬體環境檢測（CUDA 13.0, RTX 5070, PyTorch, MuJoCo）
  - `test_env.py`：階段 2 Gymnasium 環境 95x 超音速採樣速率基準測試
  - `test_kinematics_openloop.py`：開環三角步態前饋運動學純物理驗證
  - `diagnose_policy.py`：PPO 策略各足接地狀態與步態高度診斷

### 6. 系統日誌與檔案備份 (Logs & Archives)
- `MJDATA.TXT`：MuJoCo 記憶體配置診斷紀錄
- `MUJOCO_LOG.TXT`：早期數值震盪與極限發散警報日誌
- `KownIssue.md`：早期檔名拼寫備份
