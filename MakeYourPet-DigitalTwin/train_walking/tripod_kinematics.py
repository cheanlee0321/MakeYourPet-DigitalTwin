"""
Tripod Kinematics Reference Generator for Make Your Pet Hexapod
提供業界標準之閉式解析三角步態逆向運動學參考軌跡 (Human Kinematic Prior)
供 Residual RL (殘差強化學習) 作為前饋基準軌跡。
"""
import numpy as np

class TripodKinematics:
    def __init__(self, step_frequency=1.5, high_clearance=False):
        self.step_frequency = step_frequency
        self.high_clearance = high_clearance
        
        # 抬腿振幅參數 (rad): 支援越野超高抬腿 (避免踢地/穿地/克服高階障礙) 與 平地標準步態
        if high_clearance:
            self.lift_femur = 0.55  # 大腿向上大幅挑起 (約 31.5 度，離地 clearance 11.6cm)
            self.lift_tibia = 0.36  # 小腿向內充分屈折 (約 20.6 度)
        else:
            self.lift_femur = 0.32  # 平地標準抬升 (約 18.3 度)
            self.lift_tibia = 0.20  # 平地標準屈曲 (約 11.5 度)
        
        # 關節基準姿態偏移量 (rad): 支援特定檔位 (如越野挺身模式) 調整基礎站姿與行走姿態
        self.offset_hip = 0.0    # Hip 仰角偏移 (正值向下壓深/減小仰角)
        self.offset_tibia = 0.0  # Knee 俯角偏移 (正值向下展平/減小俯角)

        # 腿部幾何槓桿抬腿補償係數 (前後角腿完全對稱 1.25 倍抬升，平衡前後大跨距力臂，杜絕前進與倒退擦地)
        self.scale_middle = 1.0   # 中腿 (L2, R2) 標準離地
        self.scale_front = 1.25   # 前角腿 (L1, R1) 增加抬升 (杜絕倒退拖地)
        self.scale_rear = 1.25    # 後角腿 (L3, R3) 增加抬升 (杜絕前進拖地)

        # 行程增益參數 (高速高能效優化)
        self.stride_gain = 2.4  # vx 與前後擺動行程比例 (放寬至 2.4，步幅提升 20%)
        self.strafe_gain = 2.0  # vy 與橫移切向行程比例
        self.radial_gain = 1.20 # vy 與徑向伸縮行程比例 (配合 -2.36 雅可比平面約束，橫移推力倍增)
        self.yaw_gain = 0.40    # yaw_rate 與左右差速行程比例

        # 六足構型定義: (leg_idx, is_tripod_A, is_right, yaw_mult, mount_angle_psi)
        # Actuator order in XML:
        # 0: L1 (+45°), 1: L2 (+90°), 2: L3 (+135°), 3: R1 (-45°), 4: R2 (-90°), 5: R3 (-135°)
        self.legs_cfg = [
            (0, True,  False, -1.0,  np.pi / 4),      # L1 (Tripod A, Left-Front,  +45°)
            (1, False, False, -1.0,  np.pi / 2),      # L2 (Tripod B, Left-Middle, +90°)
            (2, True,  False, -1.0,  3 * np.pi / 4),  # L3 (Tripod A, Left-Rear,   +135°)
            (3, False, True,   1.0, -np.pi / 4),      # R1 (Tripod B, Right-Front, -45°)
            (4, True,  True,   1.0, -np.pi / 2),      # R2 (Tripod A, Right-Middle,-90°)
            (5, False, True,   1.0, -3 * np.pi / 4),  # R3 (Tripod B, Right-Rear,  -135°)
        ]

    def set_high_clearance(self, enabled: bool = True):
        """動態切換越野超高抬腿模式"""
        self.high_clearance = enabled
        if enabled:
            self.lift_femur = 0.55
            self.lift_tibia = 0.36
        else:
            self.lift_femur = 0.32
            self.lift_tibia = 0.20

    def set_lift_height(self, lift_femur: float, lift_tibia: float):
        """自由客製化抬腿幅度 (rad)"""
        self.lift_femur = float(lift_femur)
        self.lift_tibia = float(lift_tibia)

    def set_strafe_gains(self, strafe_gain: float = 2.0, radial_gain: float = 1.20):
        """動態配置側向橫移切向與徑向增益"""
        self.strafe_gain = float(strafe_gain)
        self.radial_gain = float(radial_gain)

    def set_leg_lift_scales(self, scale_front: float = 1.25, scale_middle: float = 1.0, scale_rear: float = 1.25):
        """動態配置前腿、中腿、後腿之獨立抬升補償係數"""
        self.scale_front = float(scale_front)
        self.scale_middle = float(scale_middle)
        self.scale_rear = float(scale_rear)

    def set_joint_offsets(self, offset_hip: float = 0.0, offset_tibia: float = 0.0):
        """動態配置關節基準姿態偏移量 (rad)"""
        self.offset_hip = float(offset_hip)
        self.offset_tibia = float(offset_tibia)

    def get_reference_angles(self, phase: float, command: np.ndarray) -> np.ndarray:
        """
        計算 18 個關節的參考角度 q_ref (rad)
        支援 [vx, vy, yaw_rate] 全向任意夾角行走、純側向平移與複合轉向
        :param phase: 當前步態相位 phi in [0, 2*pi)
        :param command: 目標指令 [vx, vy, yaw_rate]
        :return: (18,) np.ndarray
        """
        vx = float(command[0])
        vy = float(command[1])
        yaw = float(command[2])
        
        q_ref = np.zeros(18, dtype=np.float32)
        speed = abs(vx) + abs(vy) + abs(yaw)
        
        # 靜止待命模式：回歸基準站立姿態 (疊加關節基準姿態偏移量)
        if speed < 0.02:
            if self.offset_hip != 0.0 or self.offset_tibia != 0.0:
                for leg_idx in range(6):
                    base_j = leg_idx * 3
                    q_ref[base_j + 1] = self.offset_hip
                    q_ref[base_j + 2] = self.offset_tibia
            return q_ref

        # 計算三角步態 A/B 組相位 (相差 180 度 / pi)
        phase_A = phase % (2.0 * np.pi)
        phase_B = (phase + np.pi) % (2.0 * np.pi)

        for leg_idx, is_tripod_A, is_right, yaw_mult, psi in self.legs_cfg:
            p = phase_A if is_tripod_A else phase_B

            # 1. 切向分量 (Coxa 水平擺角): 結合前後直行 (vx)、側向切向投影 (-vy*cos(psi)) 與差速轉向 (yaw)
            # 左腿與右腿之 Coxa 旋轉方向定義相反，透過 is_right 進行符號對稱轉換
            if not is_right:
                stride_tangential = vx * self.stride_gain - vy * np.cos(psi) * self.strafe_gain + yaw * self.yaw_gain * yaw_mult
                leg_stride = np.clip(stride_tangential, -0.65, 0.65)
                q_coxa = np.cos(p) * leg_stride
            else:
                stride_tangential = vx * self.stride_gain + vy * np.cos(psi) * self.strafe_gain + yaw * self.yaw_gain * yaw_mult
                leg_stride = np.clip(stride_tangential, -0.65, 0.65)
                q_coxa = -np.cos(p) * leg_stride

            # 2. 徑向分量 (Femur / Tibia 平面雅可比逆解貼地水平伸縮):
            # 橫移時中腿 (psi=±90°) 正對兩側，Coxa 切向無位移，必須依賴大腿與膝關節反向協同 (dq_tibia = -2.36 * dq_femur) 實現貼地 (dZ=0) 徑向推蹬
            v_radial = -vy * np.sin(psi)
            q_rad = v_radial * self.radial_gain * np.cos(p)
            q_rad = np.clip(q_rad, -0.22, 0.22)

            dq_femur = q_rad
            dq_tibia = -2.36 * q_rad

            # 3. Femur & Tibia 升降角 (Swing 擺動相高抬腿 vs Stance 支撐相貼地蹬踏)
            if p < np.pi:
                # 擺動期 (Swing): 前傾型指數陡峭曲線，邁步前 15% 迅速挑起至最高點，杜絕拖地擦地阻力
                tau = p / np.pi
                h = float(np.sin(np.pi * (tau ** 0.65)))
                if leg_idx in [1, 4]:    # 中腿 (L2, R2)
                    lift_mult = self.scale_middle
                elif leg_idx in [0, 3]:  # 前腿 (L1, R1)
                    lift_mult = self.scale_front
                else:                    # 後腿 (L3, R3)
                    lift_mult = self.scale_rear
                q_femur = -self.lift_femur * lift_mult * h + dq_femur * 0.5
                q_tibia = -self.lift_tibia * lift_mult * h + dq_tibia * 0.5
            else:
                # 支撐期 (Stance): 貼地蹬踏，疊加雅可比平面水平徑向推進力 (保證 dZ=0，足端完全平順滑推)
                q_femur = 0.01 + dq_femur
                q_tibia = 0.005 + dq_tibia

            base_j = leg_idx * 3
            q_ref[base_j + 0] = q_coxa
            q_ref[base_j + 1] = q_femur + self.offset_hip
            q_ref[base_j + 2] = q_tibia + self.offset_tibia

        return q_ref

