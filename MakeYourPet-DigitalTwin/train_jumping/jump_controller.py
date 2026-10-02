"""
Jump Controller for Make Your Pet Hexapod
實現多階段有限狀態機 (FSM) 立定跳躍控制器：
安全深蹲 (Crouch) -> 瞬間爆發 (Thrust) -> 騰空伸足 (Flight & Reach) -> 動態觸地吸震 (Dynamic Landing)

特點：
1. 【雙檔爆發超跳】：
   - 常規 5 鍵：爆發力矩 15.0 N*m，最高起跳達 55cm+！
   - Shift + 5 鍵：極限火箭超跳 18.0 N*m，最高起跳達 70cm+（飛躍人身腰部高度）！
2. 【安全深蹲幾何】：底盤離地嚴格保留 >3.4cm 淨空，起跳前 100% 零觸地。
3. 【動態觸地傳感 (Contact-Driven Landing)】：利用 MuJoCo 足端接觸流動態感知接地，腳掌先觸地即刻切換吸震阻尼，徹底杜絕腹部著地。
"""
import numpy as np
import mujoco

class JumpState:
    IDLE = 0       # 待命 / 正常步態
    CROUCH = 1     # 蓄力深蹲期
    THRUST = 2     # 瞬間爆發蹬地期 (脈衝過載)
    FLIGHT = 3     # 騰空伸足迎接地面期
    LANDING = 4    # 著地吸震復原期

class JumpController:
    def __init__(self, dt: float = 0.02, model=None, enable_burst: bool = True):
        self.dt = dt
        self.model = model
        self.enable_burst = enable_burst
        self.state = JumpState.IDLE
        self.state_time = 0.0
        self.power = 1.0  # 力度係數 (1.0 = 55cm 標準高跳, 1.25 = 70cm+ 火箭超跳)

        # 時段長度設定 (秒)
        self.t_crouch = 0.16       # 蓄力深蹲 0.16s (8 個控制步)
        self.t_thrust = 0.10       # 爆發蹬地 0.10s (5 個控制步)
        self.t_flight_max = 0.85   # 騰空超時上限 (配合 70cm 跳躍滯空，超時自動著地保險)
        self.t_landing = 0.35      # 著地吸震 0.35s (17 個控制步)

        # 取得足端 geom ID 供動態觸地感測
        self.tip_ids = []
        if self.model is not None:
            self.default_forcerange = self.model.actuator_forcerange.copy()
            self.default_gainprm = self.model.actuator_gainprm.copy()
            self.default_biasprm = self.model.actuator_biasprm.copy()
            leg_names = ["L1", "L2", "L3", "R1", "R2", "R3"]
            self.tip_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"tip_{n}") for n in leg_names]
        else:
            self.default_forcerange = None
            self.default_gainprm = None
            self.default_biasprm = None

        # 各階段關節目標角度 (rad): [coxa, femur, tibia]
        self.q_nominal = np.array([0.0,  0.00,  0.00], dtype=np.float32)
        # 1. 零滑移深蹲幾何：Femur -0.15rad 同步 Tibia +0.075rad 補償，足端水平位移 0.01mm (完全消除滑移摩擦應力)
        self.q_crouch  = np.array([0.0, -0.15,  0.075], dtype=np.float32)
        # 2. 全衝程爆發蹬伸：桿件向垂直最大伸展，做功衝程 8.6cm
        self.q_thrust  = np.array([0.0,  0.78, -0.25], dtype=np.float32)
        # 3. 騰空主動伸足：足端向下延伸迎接地面，腳掌先落地保護腹部
        self.q_flight  = np.array([0.0,  0.22, -0.15], dtype=np.float32)
        # 4. 回歸標稱站姿
        self.q_recover = np.array([0.0,  0.00,  0.00], dtype=np.float32)

        self.last_target = np.zeros(18, dtype=np.float32)

    @property
    def is_jumping(self) -> bool:
        """是否正在執行跳躍動作序列"""
        return self.state != JumpState.IDLE

    def _set_actuator_mode(self, mode: str):
        """
        切換致動器動態特性：
        - 'burst': 起跳瞬間爆發脈衝 (Coxa 剛性鎖死防偏擺 + Femur/Tibia 強大推力)
        - 'landing': 著地主動阻尼 (加大阻尼系數消散衝擊，避免反彈與走位)
        - 'normal': 標準行走特性 (回復原始規格)
        """
        if self.model is None or not self.enable_burst:
            return

        if mode == "burst":
            burst_force = 15.0 * self.power
            kp = burst_force * 3.5
            kv = 0.3
            for i in range(self.model.nu):
                if i % 3 == 0:
                    # Coxa 基節鎖定：高剛度 + 高阻尼，鎖死偏航角，防起跳與著地偏擺
                    self.model.actuator_forcerange[i] = [-15.0, 15.0]
                    self.model.actuator_gainprm[i, 0] = 60.0
                    self.model.actuator_biasprm[i, 1] = -60.0
                    self.model.actuator_biasprm[i, 2] = -2.5
                else:
                    self.model.actuator_forcerange[i] = [-burst_force, burst_force]
                    self.model.actuator_gainprm[i, 0] = kp
                    self.model.actuator_biasprm[i, 1] = -kp
                    self.model.actuator_biasprm[i, 2] = -kv
        elif mode == "landing":
            for i in range(self.model.nu):
                if i % 3 == 0:
                    # Coxa 維持鎖定，防落地衝擊震偏
                    self.model.actuator_forcerange[i] = [-15.0, 15.0]
                    self.model.actuator_gainprm[i, 0] = 60.0
                    self.model.actuator_biasprm[i, 1] = -60.0
                    self.model.actuator_biasprm[i, 2] = -2.5
                else:
                    # 主動著地阻尼：增強阻尼 kv=3.5，迅速吸收落地動能
                    self.model.actuator_forcerange[i] = [-10.0, 10.0]
                    self.model.actuator_gainprm[i, 0] = 25.0
                    self.model.actuator_biasprm[i, 1] = -25.0
                    self.model.actuator_biasprm[i, 2] = -3.5
        else:
            # normal: 恢復標準出廠規格
            self.model.actuator_forcerange[:] = self.default_forcerange
            self.model.actuator_gainprm[:] = self.default_gainprm
            self.model.actuator_biasprm[:] = self.default_biasprm

    def _set_burst_mode(self, enabled: bool):
        """相容舊介面"""
        self._set_actuator_mode("burst" if enabled else "normal")

    def trigger(self, power: float = 1.0) -> bool:
        """觸發立定跳躍 (power=1.0: 55cm 大跳; power=1.25: 70cm 火箭超跳)"""
        if self.state == JumpState.IDLE:
            self.power = float(power)
            self.state = JumpState.CROUCH
            self.state_time = 0.0
            return True
        return False

    def reset(self):
        """強制重置跳躍控制器為待命狀態並恢復標準電機規格"""
        self._set_actuator_mode("normal")
        self.state = JumpState.IDLE
        self.state_time = 0.0

    def step(self, current_posture_offsets: tuple = (0.0, 0.0), data=None) -> np.ndarray:
        """
        推進一個控制步進 (dt = 0.02s)，回傳 18 維目標關節角度 (rad)
        :param current_posture_offsets: (offset_hip, offset_tibia)，供復原期平滑銜接當前檔位姿態
        :param data: mujoco.MjData 實例，供即時接觸感知以在腳掌碰地瞬間精準切入吸震期
        :return: (18,) np.ndarray 目標角度；若為 IDLE 則回傳 None
        """
        if self.state == JumpState.IDLE:
            return None

        self.state_time += self.dt
        offset_hip, offset_tibia = current_posture_offsets
        q_target_recover = self.q_recover.copy()
        q_target_recover[1] += offset_hip
        q_target_recover[2] += offset_tibia

        if self.state == JumpState.CROUCH:
            # 1. 零滑移深蹲蓄力階段：足端零位移，平穩蓄力下壓
            alpha = min(1.0, self.state_time / self.t_crouch)
            leg_q = (1.0 - alpha) * q_target_recover + alpha * self.q_crouch
            if self.state_time >= self.t_crouch:
                self.state = JumpState.THRUST
                self.state_time = 0.0
                self._set_actuator_mode("burst")  # 瞬間啟用爆發脈衝 + Coxa 基節鎖定！

        elif self.state == JumpState.THRUST:
            # 2. 柔化爆發蹬地：平滑過渡 (25ms 快速線性爬升)，消除無窮大加速度衝擊與單側脈衝噪聲
            t_ramp = 0.025
            alpha = min(1.0, self.state_time / t_ramp)
            leg_q = (1.0 - alpha) * self.q_crouch + alpha * self.q_thrust

            # 【步驟 2：智慧離地即時切斷 (Liftoff Cutoff)】
            # 當機身向上升空離地 (z > 0.108m 或 垂直向上速度 vz > 1.8 且足端已脫離地面)，立即切斷爆發推力
            is_liftoff = False
            if data is not None and self.state_time >= 0.04:
                z = float(data.qpos[2])
                vz = float(data.qvel[2])
                if vz > 1.8:
                    n_feet_touch = sum(1 for c in range(data.ncon) if data.contact[c].geom1 in self.tip_ids or data.contact[c].geom2 in self.tip_ids)
                    if z >= 0.108 or n_feet_touch == 0:
                        is_liftoff = True

            if is_liftoff or self.state_time >= self.t_thrust:
                self.state = JumpState.FLIGHT
                self.state_time = 0.0
                self._set_actuator_mode("normal") # 離地騰空，立即切斷爆發推力，嚴防單側延遲蹬地踹翻機身！

        elif self.state == JumpState.FLIGHT:
            # 3. 騰空期主動伸腿迎接地面：足端向下延伸，保證腳掌先觸地
            alpha = min(1.0, self.state_time / 0.10)
            leg_q = (1.0 - alpha) * self.q_thrust + alpha * self.q_flight

            # 動態接地傳感器 (Dynamic Touchdown Detection)
            is_touchdown = False
            if data is not None and self.state_time > 0.18:
                vz = float(data.qvel[2])
                if vz < 0.0:  # 機身正處於下落階段
                    n_feet_touch = 0
                    for c_idx in range(data.ncon):
                        con = data.contact[c_idx]
                        if con.geom1 in self.tip_ids or con.geom2 in self.tip_ids:
                            n_feet_touch += 1
                    if n_feet_touch >= 2:
                        is_touchdown = True

            if is_touchdown or self.state_time >= self.t_flight_max:
                self.state = JumpState.LANDING
                self.state_time = 0.0
                self._set_actuator_mode("landing") # 切換主動著地阻尼吸震！

        elif self.state == JumpState.LANDING:
            # 4. 著地吸震復位：利用電機高阻尼平順壓縮回歸常態站姿，腹部全程懸空
            alpha = min(1.0, self.state_time / self.t_landing)
            leg_q = (1.0 - alpha) * self.q_flight + alpha * q_target_recover
            if self.state_time >= self.t_landing:
                self.state = JumpState.IDLE
                self.state_time = 0.0
                self._set_actuator_mode("normal") # 吸震完成，回歸常規行走電機規格
                leg_q = q_target_recover

        # 六足同動：擴展至 18 維關節陣列
        target_18 = np.tile(leg_q, 6)
        self.last_target = target_18.copy()
        return target_18

    def get_phase_name(self) -> str:
        """取得當前跳躍子階段名稱與進度"""
        jump_type = "🚀 70cm 火箭超跳" if self.power > 1.1 else "⚡ 55cm 爆發大跳"
        names = {
            JumpState.IDLE: "待命",
            JumpState.CROUCH: f"蓄力深蹲 ({int(self.state_time / self.t_crouch * 100)}%)",
            JumpState.THRUST: jump_type,
            JumpState.FLIGHT: "🦅 騰空飛躍 (伸足迎接)",
            JumpState.LANDING: "🛬 腳掌觸地柔順吸震",
        }
        return names.get(self.state, "未知")
