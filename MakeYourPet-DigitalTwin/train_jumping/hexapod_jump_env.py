"""
HexapodJumpEnv: 18 自由度六足機器人立定跳躍專屬殘差強化學習環境
================================================================
本環境專為「立定跳躍 (Jump)」任務量身打造，基於數字 5 行為 (JumpController 標稱跳躍軌跡)
進行殘差強化學習 (Residual RL) 訓練。

核心架構特點：
1. 【殘差前饋耦合 (Residual Feedforward)】：
   - q_target(t) = q_jump_ref(t) + alpha * a_RL(t)
   - q_jump_ref(t): 由 JumpController(power=1.15) 輸出的 18 自由度時序運動學/動力學前饋軌跡 (數字 5 行為)
   - a_RL(t): PPO 策略輸出的關節微調殘差 (縮放 alpha = 0.15 rad 約 8.6 度)
2. 【專屬跳躍回合時序 (Jump Episode Lifecycle)】：
   - 階段 0: 準備穩定期待命 (Pre-jump Settle, Step 0~9, 0.20s)
   - 階段 1: 觸發跳躍爆發 (Trigger Step 10, Crouch -> Thrust -> Flight -> Landing)
   - 階段 2: 著地穩態評估 (Post-landing Settle, Step 70~90, 0.40s)
   - 單回合約 90 步 (1.8 秒)，每回合完整經歷一次起跳-騰空-著地-復原，採樣效率極致
3. 【78 維物理感知觀測空間】：
   - 機身姿態 (Roll, Pitch, 投影重力向量 3D)
   - 機身角速度與線速度 (omega 3D, v_body 3D)
   - 機身相對高度與垂直動態 (rel_height)
   - 關節跟隨誤差 (q - q_ref, 18D)
   - 關節速度 (q_dot * 0.1, 18D)
   - 上一動殘差 (prev_action, 18D)
   - 跳躍狀態機 One-Hot 與時間進度 (5D + 1D)
   - 六足動態接地感測信號 (6D)
4. 【多目標姿態與著地順應獎勵】：
   - 空中姿態平衡 (抑制 Roll/Pitch 翻滾與偏擺)
   - 垂直推力爆發與高度獎勵
   - 六足同步柔順著地與衝擊吸震
   - 著地後標稱站姿快速平穩收斂
   - 腹部零碰撞硬性約束與舵機平滑懲罰
"""

import os
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco

try:
    from .jump_controller import JumpController, JumpState
except (ImportError, ValueError):
    from jump_controller import JumpController, JumpState


class HexapodJumpEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        render_mode=None,
        domain_randomization=True,
        terrain_type="flat",
        terrain_height=0.015,
        jump_power=1.15,
        residual_scale=0.15,
        max_steps=90,
    ):
        super().__init__()
        self.render_mode = render_mode
        self.domain_randomization = domain_randomization
        self.terrain_type = terrain_type
        self.terrain_height = terrain_height
        self.jump_power = float(jump_power)
        self.residual_scale = float(residual_scale)
        self.max_steps = int(max_steps)

        # 載入 MuJoCo 模型 (支援多種路徑層級)
        candidate_paths = [
            os.path.join(os.path.dirname(__file__), "models", "hexapod.xml"),
            os.path.join(os.path.dirname(__file__), "..", "models", "hexapod.xml"),
            os.path.join(os.getcwd(), "models", "hexapod.xml"),
        ]
        model_path = next((p for p in candidate_paths if os.path.exists(p)), None)
        if not model_path:
            raise FileNotFoundError(f"找不到六足機器人模型檔案，嘗試過的路徑: {candidate_paths}")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.trunk_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        self.ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.has_hfield = bool(self.model.nhfield > 0)

        # 碰撞檢測幾何體 ID
        self.frame_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "col_frame")
        self.batt_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "col_batt")
        self.phone_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "col_phone")
        self.belly_geom_ids = {self.frame_geom_id, self.batt_geom_id, self.phone_geom_id}

        # 六足足端與小腿幾何 ID
        self.leg_names = ["L1", "L2", "L3", "R1", "R2", "R3"]
        self.tip_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"tip_{name}")
            for name in self.leg_names
        ]
        self.tibia_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"col_tibia_{name}")
            for name in self.leg_names
        ]

        # 記錄模型原始物理參數
        self.default_trunk_mass = float(self.model.body_mass[self.trunk_id])
        if self.ground_id >= 0:
            self.default_ground_friction = float(self.model.geom_friction[self.ground_id, 0])
        else:
            self.default_ground_friction = 1.2

        # 50Hz 控制週期 (10 個 2ms 物理子步)
        self.substeps = 10
        self.dt = self.model.opt.timestep * self.substeps  # 0.020s

        # 數字 5 跳躍控制器實例 (FSM 基準)
        self.jump_ctrl = JumpController(dt=self.dt, model=self.model, enable_burst=True)

        # 標稱站姿高度與關節角度 (rad)
        self.nominal_height = 0.082
        self.default_joint_angles = np.zeros(18, dtype=np.float32)

        # 動作空間: 18 維連續空間 [-1.0, 1.0]，映射至 [-residual_scale, +residual_scale] rad
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(18,), dtype=np.float32
        )

        # 觀測空間: 78 維連續空間
        # - roll, pitch: 2
        # - projected gravity: 3
        # - omega (body angular vel): 3
        # - v_body (body linear vel): 3
        # - rel_height: 1
        # - joint_error (q - q_ref): 18
        # - joint_vel (q_dot * 0.1): 18
        # - prev_action: 18
        # - fsm_state (one-hot): 5
        # - fsm_phase_ratio: 1
        # - feet_contact: 6
        # 總計 = 2 + 3 + 3 + 3 + 1 + 18 + 18 + 18 + 5 + 1 + 6 = 78
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(78,), dtype=np.float32
        )

        # 回合內部狀態計數
        self.step_count = 0
        self.jump_trigger_step = 10  # 第 10 步 (0.20 秒) 正式觸發起跳
        self.prev_action = np.zeros(18, dtype=np.float32)
        self.current_q_ref = np.zeros(18, dtype=np.float32)
        self.max_height_reached = self.nominal_height
        self.has_touched_ground_after_flight = False

    def _get_terrain_height_at(self, x: float, y: float) -> float:
        """依據 MuJoCo hfield 計算局部地表高度"""
        if not self.has_hfield or self.terrain_type == "flat":
            return 0.0

        nrow = self.model.hfield_nrow[0]
        ncol = self.model.hfield_ncol[0]
        arena_size = float(self.model.hfield_size[0, 0]) * 2.0
        max_allowed_h = float(self.model.hfield_size[0, 2])

        r = int(np.clip((x / arena_size + 0.5) * nrow, 0, nrow - 1))
        c = int(np.clip((y / arena_size + 0.5) * ncol, 0, ncol - 1))
        norm_val = self.model.hfield_data[r * ncol + c]
        return float(norm_val * max_allowed_h)

    def _update_terrain(self):
        """依據地表類型動態更新 MuJoCo 高度場"""
        if not self.has_hfield:
            return
        if self.terrain_type == "flat":
            self.model.hfield_data[:] = 0.0
            return

        nrow = self.model.hfield_nrow[0]
        ncol = self.model.hfield_ncol[0]
        arena_size = float(self.model.hfield_size[0, 0]) * 2.0
        max_allowed_h = float(self.model.hfield_size[0, 2])

        max_h = self.terrain_height
        if self.domain_randomization:
            max_h *= np.random.uniform(0.85, 1.15)

        scale = np.clip(max_h / max_allowed_h, 0.0, 1.0)
        data = np.zeros((nrow, ncol), dtype=np.float32)

        for r in range(nrow):
            for c in range(ncol):
                x = (r / nrow - 0.5) * arena_size
                y = (c / ncol - 0.5) * arena_size
                dist = np.sqrt(x**2 + y**2)

                if self.terrain_type == "uneven":
                    # 中心半徑 0.25m 保留平坦起跳台，外圍著地區隨機微起伏
                    flat_weight = min(1.0, max(0.0, (dist - 0.20) / 0.15))
                    val = 0.5 + 0.5 * (np.sin(x * 8.0) * np.cos(y * 8.0))
                    data[r, c] = val * scale * flat_weight

                elif self.terrain_type == "bumps":
                    # 連續微幅平緩波浪
                    flat_weight = min(1.0, max(0.0, (dist - 0.15) / 0.15))
                    val = 0.5 + 0.4 * np.sin(x * 4.0 + y * 4.0)
                    data[r, c] = val * scale * flat_weight

                elif self.terrain_type == "slope":
                    # 傾斜坡面 (起跳點微平，著地區傾斜)
                    flat_weight = min(1.0, max(0.0, (dist - 0.20) / 0.15))
                    val = np.clip((x + 0.5) / 2.0, 0.0, 1.0)
                    data[r, c] = val * scale * flat_weight

                elif self.terrain_type == "platform":
                    # 離散凸起小圓台 (起跳台高 2cm，跳落著地至 0cm)
                    if dist <= 0.30:
                        data[r, c] = 0.35 * scale
                    else:
                        data[r, c] = 0.0

        self.model.hfield_data[:] = data.flatten()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._update_terrain()
        mujoco.mj_resetData(self.model, self.data)

        # 領域隨機化 (Domain Randomization)
        if self.domain_randomization:
            # 1. 機身質量擾動 +/-12%
            mass_scale = np.random.uniform(0.88, 1.12)
            self.model.body_mass[self.trunk_id] = self.default_trunk_mass * mass_scale

            # 2. 地面摩擦係數隨機化 [0.75, 1.45]
            if self.ground_id >= 0:
                self.model.geom_friction[self.ground_id, 0] = np.random.uniform(0.75, 1.45)
        else:
            self.model.body_mass[self.trunk_id] = self.default_trunk_mass
            if self.ground_id >= 0:
                self.model.geom_friction[self.ground_id, 0] = self.default_ground_friction

        # 重置機器人空間位置
        init_ground_z = self._get_terrain_height_at(0.0, 0.0)
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = init_ground_z + self.nominal_height + (np.random.uniform(-0.002, 0.002) if self.domain_randomization else 0.0)
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])

        # 關節初始角度 (標準標稱零位 + 微量噪聲)
        for i in range(18):
            noise = np.random.uniform(-0.015, 0.015) if self.domain_randomization else 0.0
            self.data.qpos[7 + i] = self.default_joint_angles[i] + noise
            self.data.ctrl[i] = self.data.qpos[7 + i]

        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        # 重置跳躍控制器與內部計數器
        self.jump_ctrl.reset()
        self.step_count = 0
        self.prev_action = np.zeros(18, dtype=np.float32)
        self.current_q_ref = self.default_joint_angles.copy()
        self.max_height_reached = float(self.data.qpos[2])
        self.has_touched_ground_after_flight = False

        obs = self._get_obs()
        info = {
            "height": float(self.data.qpos[2]),
            "fsm_state": "IDLE",
        }
        return obs, info

    def step(self, action):
        self.step_count += 1
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 1. 自動觸發跳躍指令 (在第 jump_trigger_step 步觸發數字 5 行為)
        if self.step_count == self.jump_trigger_step:
            self.jump_ctrl.trigger(power=self.jump_power)

        # 2. 獲取基準運動學前饋目標 q_ref (數字 5 行為)
        if self.jump_ctrl.is_jumping:
            q_jump = self.jump_ctrl.step(data=self.data)
            if q_jump is not None:
                self.current_q_ref = q_jump.copy()
            else:
                self.current_q_ref = self.default_joint_angles.copy()
        else:
            self.current_q_ref = self.default_joint_angles.copy()

        # 3. 殘差強化學習耦合 (階段自適應縮放: 確保起跳爆發維持對稱垂直，騰空與著地充分發揮自適應穩定)
        fsm_state = self.jump_ctrl.state
        if fsm_state == JumpState.THRUST:
            eff_scale = 0.025  # 爆發期收斂至 +/-0.025 rad (~1.4 度)，保證推力對稱，根除單側過載掀翻
        elif fsm_state == JumpState.CROUCH:
            eff_scale = 0.040  # 深蹲期微幅微調
        elif fsm_state == JumpState.FLIGHT:
            eff_scale = self.residual_scale  # 0.15 rad (~8.6 度)，騰空期全力抗翻滾、平展伸腿
        elif fsm_state == JumpState.LANDING:
            eff_scale = self.residual_scale * 1.20  # 0.18 rad (~10.3 度)，著地期全力吸收動能阻尼順應
        else:
            eff_scale = 0.060

        target_angles = self.current_q_ref + action * eff_scale

        # 4. 舵機物理硬體限位保護
        # Coxa & Femur: [-45 deg, +45 deg] = [-0.785, 0.785] rad
        # Tibia: [-60 deg, +60 deg] = [-1.047, 1.047] rad
        for leg_idx in range(6):
            base_j = leg_idx * 3
            target_angles[base_j + 0] = np.clip(target_angles[base_j + 0], -0.785, 0.785)
            target_angles[base_j + 1] = np.clip(target_angles[base_j + 1], -0.785, 0.785)
            target_angles[base_j + 2] = np.clip(target_angles[base_j + 2], -1.047, 1.047)

        for i in range(18):
            self.data.ctrl[i] = target_angles[i]

        # 5. 執行 10 次 2ms 物理子步進 (50Hz 控制頻率)
        for _ in range(self.substeps):
            # 空中隨機微風微擾 (Domain Randomization: 空中 0.8% 機率施加側向微力矩)
            if self.domain_randomization and self.jump_ctrl.state == JumpState.FLIGHT and np.random.rand() < 0.008:
                side_push = np.random.uniform(-0.04, 0.04, size=3)
                self.data.qvel[0:3] += side_push

            mujoco.mj_step(self.model, self.data)

        # 更新最高跳躍高度紀錄
        curr_z = float(self.data.qpos[2])
        if curr_z > self.max_height_reached:
            self.max_height_reached = curr_z

        # 檢查並構建觀測與獎勵
        obs = self._get_obs()
        reward, r_components = self._compute_reward(action)
        terminated = self._is_terminated()
        truncated = self.step_count >= self.max_steps

        self.prev_action = action.copy()

        R = self.data.xmat[self.trunk_id].reshape(3, 3)
        v_body = R.T @ self.data.qvel[0:3]
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))

        info = {
            "height": curr_z,
            "max_height": self.max_height_reached,
            "rel_height": float(curr_z - local_ground_z),
            "vz": float(v_body[2]),
            "fsm_state": self.jump_ctrl.get_phase_name(),
            "reward": reward,
            **r_components,
        }

        return obs, reward, terminated, truncated, info

    def _get_feet_contacts(self) -> np.ndarray:
        """檢測 6 隻腳端是否接觸地面 (回傳 (6,) 0.0 或 1.0 陣列)"""
        contacts = np.zeros(6, dtype=np.float32)
        for c in range(self.data.ncon):
            con = self.data.contact[c]
            g1, g2 = con.geom1, con.geom2
            for leg_i, tip_id in enumerate(self.tip_geom_ids):
                if (g1 == tip_id and g2 == self.ground_id) or (g2 == tip_id and g1 == self.ground_id):
                    contacts[leg_i] = 1.0
        return contacts

    def _check_belly_contact(self) -> bool:
        """檢測機身底盤或電池艙是否撞地"""
        for c in range(self.data.ncon):
            con = self.data.contact[c]
            g1, g2 = con.geom1, con.geom2
            if (g1 in self.belly_geom_ids and g2 == self.ground_id) or (g2 in self.belly_geom_ids and g1 == self.ground_id):
                return True
        return False

    def _get_obs(self) -> np.ndarray:
        # 1. 機身姿態 (四元數轉 Roll, Pitch)
        quat = self.data.qpos[3:7]  # [w, x, y, z]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))

        # 2. 投影重力向量 (Projected Gravity Vector in Body Frame: R^T * [0, 0, -1])
        R = self.data.xmat[self.trunk_id].reshape(3, 3)
        proj_gravity = R.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # 3. 機身角速度 (機身本體座標系)
        omega = self.data.qvel[3:6]

        # 4. 機身線速度 (機身本體座標系)
        v_body = R.T @ self.data.qvel[0:3]

        # 5. 相對地面淨空高度
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        rel_height = float(self.data.qpos[2] - local_ground_z)

        # 6. 關節跟隨誤差 (當前角度 - 基準前饋角度)
        current_joint_angles = self.data.qpos[7:25]
        joint_error = current_joint_angles - self.current_q_ref

        # 7. 關節速度 (適度縮放)
        joint_vel = self.data.qvel[6:24] * 0.1

        # 8. 跳躍狀態機 One-Hot (5 維: [IDLE, CROUCH, THRUST, FLIGHT, LANDING])
        fsm_onehot = np.zeros(5, dtype=np.float32)
        fsm_onehot[self.jump_ctrl.state] = 1.0

        # 9. 當前階段耗費時間比例 (歸一化至 [0, 1])
        t_cur = self.jump_ctrl.state_time
        if self.jump_ctrl.state == JumpState.CROUCH:
            ratio = min(1.0, t_cur / self.jump_ctrl.t_crouch)
        elif self.jump_ctrl.state == JumpState.THRUST:
            ratio = min(1.0, t_cur / self.jump_ctrl.t_thrust)
        elif self.jump_ctrl.state == JumpState.FLIGHT:
            ratio = min(1.0, t_cur / 0.60)
        elif self.jump_ctrl.state == JumpState.LANDING:
            ratio = min(1.0, t_cur / self.jump_ctrl.t_landing)
        else:
            ratio = 0.0

        # 10. 六足接地狀態 (6 維)
        feet_contacts = self._get_feet_contacts()

        obs = np.concatenate([
            np.array([roll, pitch], dtype=np.float32),          # 2
            proj_gravity.astype(np.float32),                    # 3
            omega.astype(np.float32),                           # 3
            v_body.astype(np.float32),                          # 3
            np.array([rel_height], dtype=np.float32),           # 1
            joint_error.astype(np.float32),                     # 18
            joint_vel.astype(np.float32),                       # 18
            self.prev_action.astype(np.float32),                # 18
            fsm_onehot,                                         # 5
            np.array([ratio], dtype=np.float32),                # 1
            feet_contacts                                       # 6
        ]).astype(np.float32)  # 總計 78 維

        return obs

    def _compute_reward(self, action: np.ndarray):
        """專屬多目標跳躍姿態與吸震獎勵函數"""
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        omega = self.data.qvel[3:6]
        vz = float(self.data.qvel[2])
        curr_z = float(self.data.qpos[2])
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        rel_h = curr_z - local_ground_z

        fsm_state = self.jump_ctrl.state
        feet_contacts = self._get_feet_contacts()
        n_touch = np.sum(feet_contacts)

        # 1. 存活獎勵 (保持未翻倒)
        r_alive = 0.30

        # 2. 姿態水平獎勵 (空中與著地全週期高剛性約束)
        ori_err = roll**2 + pitch**2
        tilt_rad = np.sqrt(ori_err)
        r_orient = 2.0 * np.exp(-ori_err / 0.020) - 1.5 * tilt_rad

        # 3. 角速度阻尼懲罰 (強烈抑制空中側翻與旋轉偏擺)
        r_omega = -0.25 * (omega[0]**2 + omega[1]**2 + 0.10 * omega[2]**2)

        # 4. 垂直爆發推力與離地高度獎勵 (高度獎勵與姿態水平嚴密掛鉤，嚴禁以歪斜為代價騙取高度)
        level_factor = float(np.exp(-ori_err / 0.025))
        r_thrust = 0.0
        if fsm_state == JumpState.THRUST:
            r_thrust = 0.60 * max(0.0, vz) * level_factor
        elif fsm_state == JumpState.FLIGHT:
            if rel_h > 0.15:
                r_thrust = 0.80 * (rel_h - 0.15) * level_factor

        # 5. 著地柔順吸震與足端接觸對稱性 (FLIGHT 下落期 & LANDING 階段)
        r_landing = 0.0
        if vz < -0.1 and rel_h < 0.20:
            # 腳掌先觸地且六足接觸越對稱獎勵越高
            r_landing += 0.15 * n_touch

        if fsm_state == JumpState.LANDING:
            # 著地吸震期：獎勵下落動能消散 (|vz| 歸零)
            r_landing += 0.80 * np.exp(-abs(vz) / 0.3)
            # 獎勵機身平穩貼合標稱高度
            h_err = abs(rel_h - self.nominal_height)
            r_landing += 0.50 * np.exp(-h_err / 0.02)

        # 6. 動作正則化與平滑懲罰 (以數字 5 為基準，非必要不大幅扭動關節)
        action_diff = action - self.prev_action
        r_action_mag = -0.015 * np.sum(action**2)
        r_action_smooth = -0.035 * np.sum(action_diff**2)

        # 7. 腹部撞地重大懲罰
        belly_hit = self._check_belly_contact()
        r_belly = -5.0 if belly_hit else 0.0

        total_reward = (
            r_alive
            + r_orient
            + r_omega
            + r_thrust
            + r_landing
            + r_action_mag
            + r_action_smooth
            + r_belly
        )

        components = {
            "r_alive": r_alive,
            "r_orient": r_orient,
            "r_omega": r_omega,
            "r_thrust": r_thrust,
            "r_landing": r_landing,
            "r_action_mag": r_action_mag,
            "r_action_smooth": r_action_smooth,
            "r_belly": r_belly,
        }

        return float(total_reward), components

    def _is_terminated(self) -> bool:
        """判斷是否提前非正常終止回合"""
        curr_z = float(self.data.qpos[2])
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        rel_h = curr_z - local_ground_z

        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        tilt = np.sqrt(roll**2 + pitch**2)

        # 1. 嚴重側翻倒扣：近地面 (rel_h < 0.12m) 且傾角超過 45 度
        if rel_h < 0.12 and tilt > np.deg2rad(45.0):
            return True

        # 2. 腹部碰撞地面 (電池艙或底盤撞地)
        if self._check_belly_contact():
            return True

        # 3. 異常數值發散飛出邊界 (高度超過 1.2m 或水平位移漂出 2.5m)
        if rel_h > 1.20 or abs(self.data.qpos[0]) > 2.5 or abs(self.data.qpos[1]) > 2.5:
            return True

        return False
