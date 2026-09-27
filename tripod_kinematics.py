"""
Tripod Kinematics Reference Generator for Make Your Pet Hexapod
提供業界標準之閉式解析三角步態逆向運動學參考軌跡 (Human Kinematic Prior)
供 Residual RL (殘差強化學習) 作為前饋基準軌跡。
"""
import numpy as np

class TripodKinematics:
    def __init__(self, step_frequency=1.5):
        self.step_frequency = step_frequency
        
        # 抬腿振幅參數 (rad)
        self.lift_femur = 0.32  # 大腿向上抬升 (約 18.3 度)
        self.lift_tibia = 0.20  # 小腿向內屈曲 (約 11.5 度)
        
        # 行程增益參數
        self.stride_gain = 2.0  # vx 與橫擺行程比例 (vx=0.25 -> coxa swing = 0.5 rad)
        self.yaw_gain = 0.40    # yaw_rate 與左右差速行程比例

        # 六足構型定義: (leg_idx, is_tripod_A, is_right, yaw_mult)
        # Actuator order in XML:
        # 0: L1, 1: L2, 2: L3, 3: R1, 4: R2, 5: R3
        self.legs_cfg = [
            (0, True,  False, -1.0), # L1 (Tripod A, Left)
            (1, False, False, -1.0), # L2 (Tripod B, Left)
            (2, True,  False, -1.0), # L3 (Tripod A, Left)
            (3, False, True,   1.0), # R1 (Tripod B, Right)
            (4, True,  True,   1.0), # R2 (Tripod A, Right)
            (5, False, True,   1.0), # R3 (Tripod B, Right)
        ]

    def get_reference_angles(self, phase: float, command: np.ndarray) -> np.ndarray:
        """
        計算 18 個關節的參考角度 q_ref (rad)
        :param phase: 當前步態相位 phi in [0, 2*pi)
        :param command: 目標指令 [vx, vy, yaw_rate]
        :return: (18,) np.ndarray
        """
        vx = float(command[0])
        vy = float(command[1])
        yaw = float(command[2])
        
        q_ref = np.zeros(18, dtype=np.float32)
        speed = abs(vx) + abs(vy) + abs(yaw)
        
        # 靜止待命模式：回歸自然 Z 字形基準站立姿態
        if speed < 0.02:
            return q_ref

        # 計算三角步態 A/B 組相位 (相差 180 度 / pi)
        phase_A = phase % (2.0 * np.pi)
        phase_B = (phase + np.pi) % (2.0 * np.pi)

        stride_base = vx * self.stride_gain

        for leg_idx, is_tripod_A, is_right, yaw_mult in self.legs_cfg:
            p = phase_A if is_tripod_A else phase_B

            # 計算該腿步幅 (結合直行與差速轉向)
            leg_stride = np.clip(stride_base + yaw * self.yaw_gain * yaw_mult, -0.65, 0.65)

            # 1. Coxa 橫擺角 (Left legs: +coxa moves back; Right legs: -coxa moves back)
            if not is_right:
                q_coxa = np.cos(p) * leg_stride
            else:
                q_coxa = -np.cos(p) * leg_stride

            # 2. Femur & Tibia 升降角 (Swing vs Stance)
            if p < np.pi:
                # 擺動期 (Swing): 騰空抬腿向前伸
                h = np.sin(p)
                q_femur = -self.lift_femur * h
                q_tibia = -self.lift_tibia * h
            else:
                # 支撐期 (Stance): 貼地蹬踏向後推
                q_femur = 0.02
                q_tibia = 0.01

            base_j = leg_idx * 3
            q_ref[base_j + 0] = q_coxa
            q_ref[base_j + 1] = q_femur
            q_ref[base_j + 2] = q_tibia

        return q_ref
