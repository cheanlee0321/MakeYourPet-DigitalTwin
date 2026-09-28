import os
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
try:
    from .tripod_kinematics import TripodKinematics
except (ImportError, ValueError):
    from tripod_kinematics import TripodKinematics

class HexapodEnv(gym.Env):
    """
    Make Your Pet 18 自由度六足機器人 Gymnasium 強化學習環境 (殘差強化學習 Residual RL + 解析三角步態運動學前饋)
    - 殘差強化學習 (Residual RL) 架構:
        q_target = q_ref(phase, cmd) + delta_q_RL
        - q_ref: 業界標準閉式解析三角步態逆向運動學前饋軌跡 (速度充沛、步頻穩定、絕對不會黏在地上)
        - delta_q_RL: 強化學習策略輸出的關節殘差補償量 (縮放至 +/-0.15 rad，負責動態平衡、抗滑、地形順應、姿態穩定)
    - 動作空間: 18 維連續空間 [-1.0, 1.0] (殘差控制量，對應 +/-0.15 rad / +/-8.6 度)
    - 觀測空間: 67 維連續空間 (機身姿態、角速度、線速度、關節跟隨誤差、關節轉速、上一動殘差、目標指令、三角步態相位時鐘)
    - 控制頻率: 50Hz (每步進行 10 次 2ms 物理子步進，完美對齊實體舵機 PWM 週期)
    - 指令跟隨: 支援 [vx, vy, yaw_rate] 即時遙控，包含停步待命、前進、倒車、原地旋轉
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, domain_randomization=True, auto_resample_commands=True,
                 terrain_type="flat", terrain_height=0.015):
        super().__init__()
        self.render_mode = render_mode
        self.domain_randomization = domain_randomization
        self.auto_resample_commands = auto_resample_commands
        self.terrain_type = terrain_type
        self.terrain_height = terrain_height

        candidate_paths = [
            os.path.join(os.path.dirname(__file__), "models", "hexapod.xml"),
            os.path.join(os.path.dirname(__file__), "..", "models", "hexapod.xml"),
            os.path.join(os.getcwd(), "models", "hexapod.xml"),
        ]
        model_path = next((p for p in candidate_paths if os.path.exists(p)), None)
        if not model_path:
            raise FileNotFoundError(f"找不到模型檔案，嘗試過的路徑: {candidate_paths}")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.trunk_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        self.ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.has_hfield = bool(self.model.nhfield > 0)

        # 記錄模型原始物理參數 (供領域隨機化重設)
        self.default_trunk_mass = float(self.model.body_mass[self.trunk_id])
        if self.ground_id >= 0:
            self.default_ground_friction = float(self.model.geom_friction[self.ground_id, 0])
        else:
            self.default_ground_friction = 1.2

        # 50Hz 控制週期 (10 個 2ms 物理步)
        self.substeps = 10
        self.dt = self.model.opt.timestep * self.substeps  # 0.020s

        # 步態相位時鐘與運動學前饋產生器 (1.5 Hz 步頻，越野地貌自動切換高抬腿步態避免踢地)
        self.step_frequency = 1.5
        self.phase = 0.0
        self.kinematics = TripodKinematics(step_frequency=self.step_frequency, high_clearance=(self.terrain_type != "flat"))

        # 殘差縮放係數 (+/-0.15 rad，約 8.6 度，專注於動態姿態修正與防滑地表貼合)
        self.residual_scale = 0.15

        # 定義 18 維動作空間 (殘差動作)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(18,), dtype=np.float32
        )

        # 定義 67 維觀測空間 (姿態 2 + 角速度 3 + 本體線速度 3 + 關節誤差 18 + 關節轉速 18 + 上一動殘差 18 + 指令 3 + 相位時鐘 2)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(67,), dtype=np.float32
        )

        # 六足幾何 ID 配置 (足端球與小腿膠囊)
        self.leg_names = ["L1", "L2", "L3", "R1", "R2", "R3"]
        self.tip_geom_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"tip_{name}") for name in self.leg_names]
        self.tibia_geom_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"col_tibia_{name}") for name in self.leg_names]
        self.foot_radius = 0.0048  # 足端橡膠球半徑 (4.8mm)

        # 接觸幾何對應字典 (快速由碰撞 geom 映射至所屬腿索引 0~5)
        self.geom_to_leg = {}
        for i in range(6):
            self.geom_to_leg[self.tip_geom_ids[i]] = i
            self.geom_to_leg[self.tibia_geom_ids[i]] = i

        # 重複使用之數值暫存緩衝區 (避免 step 中重複分配記憶體)
        self._c_force_buf = np.zeros(6, dtype=np.float64)
        self._v_tip_buf = np.zeros(6, dtype=np.float64)

        # 三角步態分組足端 ID: Tripod A (L1, R2, L3), Tripod B (R1, L2, R3)
        self.tripod_A_ids = [self.tip_geom_ids[0], self.tip_geom_ids[4], self.tip_geom_ids[2]]
        self.tripod_B_ids = [self.tip_geom_ids[3], self.tip_geom_ids[1], self.tip_geom_ids[5]]

        # 預設基準站立高度與關節角度 (rad)
        self.default_joint_angles = np.zeros(18, dtype=np.float32)
        self.nominal_height = 0.082

        # 目標運動指令: [vx, vy, yaw_rate]
        self.command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.command_timer = 350
        self.command_step = 0

        self.prev_action = np.zeros(18, dtype=np.float32)
        self.step_count = 0
        self.max_steps = 1000  # 單回合最多 20 秒

        self.viewer = None

    def set_joint_offsets(self, offset_hip: float = 0.0, offset_tibia: float = 0.0):
        """設置關節基準姿態偏移量 (rad) 並同步更新預設關節姿態"""
        self.kinematics.set_joint_offsets(offset_hip, offset_tibia)
        for leg_idx in range(6):
            base_j = leg_idx * 3
            self.default_joint_angles[base_j + 1] = float(offset_hip)
            self.default_joint_angles[base_j + 2] = float(offset_tibia)

    def _sample_command(self):
        """隨機採樣運動目標指令 [vx, vy, yaw_rate]"""
        r = np.random.rand()
        if r < 0.20:
            # 1. 煞車 / 靜止待命 (20% 機率): 要求收步站好
            vx = 0.0
            vy = 0.0
            yaw = 0.0
        elif r < 0.70:
            # 2. 直線 / 緩轉前進 (50% 機率): 速度 0.15 ~ 0.35 m/s
            vx = float(np.random.uniform(0.15, 0.35))
            vy = 0.0
            yaw = float(np.random.uniform(-0.20, 0.20))
        elif r < 0.90:
            # 3. 原地旋轉 / 銳角轉彎 (20% 機率): 旋轉速度 +/-0.35 ~ +/-0.75 rad/s
            vx = float(np.random.uniform(0.0, 0.10))
            vy = 0.0
            yaw_sign = 1.0 if np.random.rand() > 0.5 else -1.0
            yaw = float(yaw_sign * np.random.uniform(0.35, 0.75))
        else:
            # 4. 後退倒車 (10% 機率): 速度 -0.10 ~ -0.25 m/s
            vx = float(np.random.uniform(-0.25, -0.10))
            vy = 0.0
            yaw = float(np.random.uniform(-0.20, 0.20))

        self.command = np.array([vx, vy, yaw], dtype=np.float32)

    def _update_terrain(self):
        """依據地貌設定產生高度場數據，並保持起點半徑 0.45 米平坦無坑"""
        if not self.has_hfield:
            return
        if self.terrain_type == "flat":
            self.model.hfield_data[:] = 0.0
            self.kinematics.set_high_clearance(False)
            return

        # 越野起伏地形：自動啟用高抬腿步態 (抬腿高度 5.5cm，防止踢地穿地)
        self.kinematics.set_high_clearance(True)

        nrow = self.model.hfield_nrow[0]
        ncol = self.model.hfield_ncol[0]
        max_h = self.terrain_height
        if self.domain_randomization:
            max_h *= np.random.uniform(0.8, 1.2)

        max_allowed_h = float(self.model.hfield_size[0, 2])
        scale = np.clip(max_h / max_allowed_h, 0.0, 1.0)
        data = np.zeros((nrow, ncol), dtype=np.float32)

        # 取得場地實體水平長度 (例如 size="5 5" 時，arena_size = 10.0m)
        arena_size = float(self.model.hfield_size[0, 0]) * 2.0

        # 離散區塊網格預計算 (供 blocks 與 park 使用)
        block_size_m = 0.25  # 每個區塊跨距 25cm
        cells_per_block = max(1, int((block_size_m / arena_size) * nrow))
        num_bx = (nrow + cells_per_block - 1) // cells_per_block
        num_by = (ncol + cells_per_block - 1) // cells_per_block
        block_palette = np.array([0.10, 0.28, 0.48, 0.68, 0.88, 1.00], dtype=np.float32)
        rand_block_indices = np.random.randint(0, len(block_palette), size=(num_bx, num_by))
        block_heights = block_palette[rand_block_indices]

        for r in range(nrow):
            bx = min(r // cells_per_block, num_bx - 1)
            for c in range(ncol):
                by = min(c // cells_per_block, num_by - 1)
                x = (r / nrow - 0.5) * arena_size
                y = (c / ncol - 0.5) * arena_size
                dist = np.sqrt(x**2 + y**2)
                if dist < 0.45:
                    data[r, c] = 0.0
                else:
                    flat_weight = min(1.0, (dist - 0.45) / 0.25)
                    if self.terrain_type == "bumps":
                        # 劇烈密集高頻波浪 (波峰波谷更緊湊)
                        val = 0.5 + 0.5 * (np.sin(x * 5.0) * np.cos(y * 5.0))
                    elif self.terrain_type == "blocks":
                        # 離散階梯石塊木樁陣
                        val = block_heights[bx, by]
                    elif self.terrain_type == "park":
                        # 四大象限複合越野挑戰公園 (波浪 / 階梯 / 碎石 / 稜坡)
                        if x >= 0 and y >= 0:
                            # 東北象限: 劇烈高頻波浪
                            val = 0.5 + 0.5 * (np.sin(x * 6.0) * np.cos(y * 6.0))
                        elif x < 0 and y >= 0:
                            # 西北象限: 離散高低階梯木樁
                            val = block_heights[bx, by]
                        elif x < 0 and y < 0:
                            # 西南象限: 嶙峋劇烈碎石
                            n1 = np.sin(x * 8.0 + y * 5.0)
                            n2 = np.cos(x * 12.0 - y * 10.0) * 0.6
                            n3 = np.random.uniform(-0.3, 0.3)
                            val = np.clip(0.5 + 0.45 * (n1 + n2 + n3) / 1.9, 0.0, 1.0)
                        else:
                            # 東南象限: 複合斜坡與金字塔坡道
                            val = 0.5 + 0.45 * np.sin(x * 3.0 + y * 2.0)
                    elif self.terrain_type == "rough":
                        n1 = np.sin(x * 5.0 + y * 3.0)
                        n2 = np.cos(x * 10.0 - y * 8.0) * 0.5
                        n3 = np.random.uniform(-0.25, 0.25)
                        val = 0.5 + 0.4 * (n1 + n2 + n3) / 1.75
                    elif self.terrain_type == "slope":
                        val = np.clip((x + arena_size * 0.2) / (arena_size * 0.7), 0.0, 1.0)
                    else:
                        val = 0.0
                    data[r, c] = np.clip(val * scale * flat_weight, 0.0, 1.0)

        self.model.hfield_data[:] = data.flatten()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._update_terrain()
        mujoco.mj_resetData(self.model, self.data)

        # 領域隨機化 (Domain Randomization)
        if self.domain_randomization:
            # 1. 機身質量微調 +/-10%
            mass_scale = np.random.uniform(0.90, 1.10)
            self.model.body_mass[self.trunk_id] = self.default_trunk_mass * mass_scale

            # 2. 地面摩擦力隨機微調 [0.85, 1.45]
            if self.ground_id >= 0:
                self.model.geom_friction[self.ground_id, 0] = np.random.uniform(0.85, 1.45)
        else:
            self.model.body_mass[self.trunk_id] = self.default_trunk_mass
            if self.ground_id >= 0:
                self.model.geom_friction[self.ground_id, 0] = self.default_ground_friction

        # 重置機身位置 (加入微量噪聲)
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = self.nominal_height + np.random.uniform(-0.003, 0.003)
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])

        # 初始關節角度微量噪聲
        for i in range(18):
            noise = np.random.uniform(-0.02, 0.02)
            self.data.qpos[7 + i] = self.default_joint_angles[i] + noise
            self.data.ctrl[i] = self.data.qpos[7 + i]

        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self.prev_action = np.zeros(18, dtype=np.float32)
        self.step_count = 0
        self.command_step = 0
        self.command_timer = int(np.random.randint(300, 500))
        self.phase = 0.0

        if self.auto_resample_commands:
            self._sample_command()

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action, override_target_angles=None):
        self.step_count += 1
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        if override_target_angles is not None:
            # 外部動作指令覆蓋 (如立定跳躍控制器 FSM 直接注入)
            target_angles = np.array(override_target_angles, dtype=np.float32).copy()
        else:
            # 1. 取得解析三角步態前饋參考角度 q_ref
            q_ref = self.kinematics.get_reference_angles(self.phase, self.command)

            # 2. 疊加 RL 殘差: q_target = q_ref + action * residual_scale
            target_angles = q_ref + action * self.residual_scale

        # 3. 確保控制訊號在舵機物理機械限位之內
        # Coxa & Femur: [-45 deg, +45 deg] = [-0.785, 0.785] rad
        # Tibia: [-60 deg, +60 deg] = [-1.047, 1.047] rad
        for leg_idx in range(6):
            base_j = leg_idx * 3
            target_angles[base_j + 0] = np.clip(target_angles[base_j + 0], -0.785, 0.785)
            target_angles[base_j + 1] = np.clip(target_angles[base_j + 1], -0.785, 0.785)
            target_angles[base_j + 2] = np.clip(target_angles[base_j + 2], -1.047, 1.047)

        for i in range(18):
            self.data.ctrl[i] = target_angles[i]

        # 推進 10 個物理子步進 (20ms)
        for _ in range(self.substeps):
            # 領域隨機化：微小機率施加外力微擾
            if self.domain_randomization and np.random.rand() < 0.005:
                push = np.random.uniform(-0.03, 0.03, size=2)
                self.data.qvel[0:2] += push
            mujoco.mj_step(self.model, self.data)

        # 更新步態相位時鐘 (若處於外部動作覆蓋如跳躍期間，則暫停步態時鐘推進)
        is_moving = (abs(self.command[0]) > 0.02 or abs(self.command[2]) > 0.05) and (override_target_angles is None)
        if is_moving:
            self.phase = (self.phase + 2.0 * np.pi * self.step_frequency * self.dt) % (2.0 * np.pi)
        else:
            self.phase = 0.0

        # 動態重新採樣目標指令
        if self.auto_resample_commands:
            self.command_step += 1
            if self.command_step >= self.command_timer:
                self._sample_command()
                self.command_step = 0
                self.command_timer = int(np.random.randint(300, 500))

        obs = self._get_obs()
        reward = self._compute_reward(action)
        terminated = self._is_terminated(is_override=(override_target_angles is not None))
        truncated = self.step_count >= self.max_steps

        self.prev_action = action.copy()
        R = self.data.xmat[self.trunk_id].reshape(3, 3)
        v_body = R.T @ self.data.qvel[0:3]
        omega_z = float(self.data.qvel[5])
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        info = {
            "vx": float(v_body[0]),
            "vy": float(v_body[1]),
            "yaw_rate": omega_z,
            "cmd_vx": float(self.command[0]),
            "cmd_yaw": float(self.command[2]),
            "height": float(self.data.qpos[2]),
            "rel_height": float(self.data.qpos[2] - local_ground_z),
            "r_early_contact": float(getattr(self, "last_r_early_contact", 0.0)),
            "r_slip": float(getattr(self, "last_r_slip", 0.0)),
            "r_lift_sym": float(getattr(self, "last_r_lift_sym", 0.0)),
        }

        return obs, reward, terminated, truncated, info

    def _get_obs(self):
        # 1. 機身姿態 (四元數轉 Roll, Pitch)
        quat = self.data.qpos[3:7]  # [w, x, y, z]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))

        # 2. 機身角速度 (機身本體坐標系)
        omega = self.data.qvel[3:6]

        # 3. 機身線速度 (轉換為機身本體坐標系: v_body = R^T @ v_world)
        R = self.data.xmat[self.trunk_id].reshape(3, 3)
        v_body = R.T @ self.data.qvel[0:3]

        # 4. 關節跟隨誤差 (相對於當前運動學前饋參考角度 q_ref)
        q_ref = self.kinematics.get_reference_angles(self.phase, self.command)
        current_joint_angles = self.data.qpos[7:25]
        joint_error = current_joint_angles - q_ref

        # 5. 關節轉速 (18 維，適度縮放)
        joint_vel = self.data.qvel[6:24]

        # 6. 步態相位時鐘 (2 維: [sin(phi), cos(phi)])
        is_moving = (abs(self.command[0]) > 0.02 or abs(self.command[2]) > 0.05)
        sin_phase = np.sin(self.phase) if is_moving else 0.0
        cos_phase = np.cos(self.phase) if is_moving else 0.0
        clock = np.array([sin_phase, cos_phase], dtype=np.float32)

        obs = np.concatenate([
            np.array([roll, pitch], dtype=np.float32),          # 2
            omega.astype(np.float32),                           # 3
            v_body.astype(np.float32),                          # 3
            joint_error.astype(np.float32),                     # 18
            joint_vel.astype(np.float32) * 0.1,                 # 18
            self.prev_action.astype(np.float32),                # 18
            self.command.astype(np.float32),                    # 3
            clock                                               # 2
        ]).astype(np.float32)  # 總共 67 維

        return obs

    def _compute_reward(self, action):
        R = self.data.xmat[self.trunk_id].reshape(3, 3)
        v_body = R.T @ self.data.qvel[0:3]
        vx = float(v_body[0])
        vy = float(v_body[1])
        omega_z = float(self.data.qvel[5])

        cmd_vx = float(self.command[0])
        cmd_vy = float(self.command[1])
        cmd_yaw = float(self.command[2])

        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        height = float(self.data.qpos[2])

        action_diff = action - self.prev_action
        is_stop = (abs(cmd_vx) < 0.02 and abs(cmd_yaw) < 0.05)

        if is_stop:
            # === 1. 煞車停步待命模式 ===
            r_stop_v = 3.0 * np.exp(-(vx**2 + vy**2) / 0.005)
            r_stop_yaw = 2.0 * np.exp(-(omega_z**2) / 0.01)
            r_stop_action = -0.5 * float(np.sum(np.square(action)))
            r_motion = r_stop_v + r_stop_yaw + r_stop_action
        else:
            # === 2. 行進與轉向模式 ===
            # A. 前進速度追蹤
            r_track_vx = 3.5 * np.exp(-((vx - cmd_vx)**2) / 0.02)
            # B. 橫移抑制 (防止側滑打滑)
            r_lateral = -2.0 * (vy**2)
            # C. 轉向追蹤
            r_track_yaw = 2.5 * np.exp(-((omega_z - cmd_yaw)**2) / 0.03)

            r_motion = r_track_vx + r_lateral + r_track_yaw

        # === 3. 機身平穩性與姿態平衡獎勵 (Residual RL 核心任務) ===
        # 獎勵水平機身，抑制過度俯仰或側傾 (Roll/Pitch damping)
        r_posture = 2.0 * np.exp(-(roll**2 + pitch**2) / 0.015)

        # 機身相對地面高度維持 (保持在 nominal_height 附近，順應越野起伏地形)
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        rel_height = height - local_ground_z
        r_height = 1.5 * np.exp(-((rel_height - self.nominal_height)**2) / 0.003)

        # 殘差正規化懲罰 (讓運動學前饋承擔主要位移，RL 殘差只做細緻微調與避震)
        r_res_mag = -0.05 * float(np.sum(np.square(action)))
        r_res_smooth = -0.03 * float(np.sum(np.square(action_diff)))

        # === 4. 六足接觸力學與步態對稱性約束 (解決拖地與中腿/角腿抬腿不均) ===
        # A. 提取六足對地法向接觸力 (N)
        foot_normal_force = np.zeros(6, dtype=np.float32)
        for c_idx in range(self.data.ncon):
            c = self.data.contact[c_idx]
            if c.geom1 == self.ground_id or c.geom2 == self.ground_id:
                other = c.geom2 if c.geom1 == self.ground_id else c.geom1
                if other in self.geom_to_leg:
                    l_idx = self.geom_to_leg[other]
                    mujoco.mj_contactForce(self.model, self.data, c_idx, self._c_force_buf)
                    foot_normal_force[l_idx] += float(abs(self._c_force_buf[0]))

        # B. 判斷步態相位 (Swing 擺動相 vs Stance 支撐相)
        # Leg 0: L1 (A), Leg 1: L2 (B), Leg 2: L3 (A), Leg 3: R1 (B), Leg 4: R2 (A), Leg 5: R3 (B)
        phase_A = self.phase % (2.0 * np.pi)
        phase_B = (self.phase + np.pi) % (2.0 * np.pi)

        r_early_contact = 0.0
        r_slip = 0.0
        r_lift_sym = 0.0

        # 計算足端水平打滑懲罰 (針對著地承重腿，切向水平速度必須歸零)
        for i in range(6):
            if foot_normal_force[i] > 1.0:
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_GEOM, self.tip_geom_ids[i], self._v_tip_buf, 0)
                v_horiz_sq = float(self._v_tip_buf[3]**2 + self._v_tip_buf[4]**2)
                # 依承重力道懲罰水平打滑 (切向摩擦磨損)
                f_weight = min(float(foot_normal_force[i]) / 5.0, 2.0)
                r_slip -= 1.5 * f_weight * v_horiz_sq

        if not is_stop:
            swing_tripod_is_A = (phase_A < np.pi)
            swing_p = phase_A if swing_tripod_is_A else phase_B
            # 擺動相強度權重 (中段 sin(p) 最大，兩端平滑過渡)
            swing_weight = float(np.sin(swing_p))

            # 擺動足端清單 (中腿索引, 前角腿索引, 後角腿索引)
            if swing_tripod_is_A:
                swing_mid = 4        # R2
                swing_front = 0      # L1
                swing_rear = 2       # L3
                swing_legs = [0, 4, 2]
            else:
                swing_mid = 1        # L2
                swing_front = 3      # R1
                swing_rear = 5       # R3
                swing_legs = [3, 1, 5]

            # 1. 擺動相觸地 / 拖行重罰 (Early Contact Penalty)
            for leg_i in swing_legs:
                if foot_normal_force[leg_i] > 1.0:
                    f_excess = foot_normal_force[leg_i] - 1.0
                    r_early_contact -= 0.25 * min(f_excess, 12.0) * swing_weight

            # 2. 擺動足端地表垂直淨空 (公尺)
            swing_clearances = np.zeros(3, dtype=np.float32)
            for k, leg_i in enumerate([swing_mid, swing_front, swing_rear]):
                pos = self.data.geom_xpos[self.tip_geom_ids[leg_i]]
                gz = self._get_terrain_height_at(float(pos[0]), float(pos[1]))
                swing_clearances[k] = float(pos[2] - gz - self.foot_radius)

            h_mid, h_front, h_rear = swing_clearances

            # 3. 六足抬腿高度對稱性與淨空方差懲罰 (Lift Symmetry Penalty)
            # 懲罰中腿 (h_mid) 與角腿 (h_front, h_rear) 之間的高度差，消除翹翹板不均
            diff_front = abs(h_mid - h_front)
            diff_rear = abs(h_mid - h_rear)
            var_height = float(np.var(swing_clearances))
            r_lift_sym -= (200.0 * var_height + 5.0 * (diff_front + diff_rear)) * swing_weight

            # 4. 擺動相最小離地淨空引導 (確保角腿不貼地，至少抬離地表 2.5cm)
            target_min_clearance = 0.025 * swing_weight
            for h in swing_clearances:
                if h < target_min_clearance:
                    r_lift_sym -= 20.0 * ((target_min_clearance - h) ** 2)

        self.last_r_early_contact = r_early_contact
        self.last_r_slip = r_slip
        self.last_r_lift_sym = r_lift_sym

        # 存活獎勵
        r_alive = 1.0

        total_reward = (
            r_motion
            + r_posture
            + r_height
            + r_res_mag
            + r_res_smooth
            + r_early_contact
            + r_slip
            + r_lift_sym
            + r_alive
        )

        return float(total_reward)

    def _get_terrain_height_at(self, x: float, y: float) -> float:
        """精準查詢特定 (x, y) 坐標下方之地表高度 (公尺)"""
        if not self.has_hfield or self.terrain_type == "flat":
            return 0.0
        arena_size = float(self.model.hfield_size[0, 0]) * 2.0
        nrow = self.model.hfield_nrow[0]
        ncol = self.model.hfield_ncol[0]
        max_h = float(self.model.hfield_size[0, 2])
        r = int((x / arena_size + 0.5) * nrow)
        c = int((y / arena_size + 0.5) * ncol)
        if 0 <= r < nrow and 0 <= c < ncol:
            return float(self.model.hfield_data[r * ncol + c]) * max_h
        return 0.0

    def _is_terminated(self, is_override: bool = False):
        local_ground_z = self._get_terrain_height_at(float(self.data.qpos[0]), float(self.data.qpos[1]))
        rel_height = float(self.data.qpos[2]) - local_ground_z

        # 1. 姿態嚴重側翻或仰翻 (Roll / Pitch 判定)
        quat = self.data.qpos[3:7]
        w, x, y, z = quat
        roll = abs(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = abs(math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))

        if is_override:
            # 跳躍期間：空中自由騰空允許動態姿態調整；僅在近地面 (<0.08m) 且傾角 >55° 真正倒地時才判定翻倒
            if (roll > math.radians(55.0) or pitch > math.radians(55.0)) and rel_height < 0.08:
                return True
            # 跳躍高度天花板放寬至 1.20m (支援 80cm 超級火箭跳)
            if rel_height > 1.20:
                return True
        else:
            if roll > math.radians(38.0) or pitch > math.radians(38.0):
                return True
            if rel_height > 0.45:
                return True

        # 2. 軀幹嚴重陷入地下或底盤完全受困 (底盤半厚度 1.2cm，rel_height < 1.0cm 代表底盤被強行壓入地底)
        if rel_height < 0.010:
            return True

        return False
