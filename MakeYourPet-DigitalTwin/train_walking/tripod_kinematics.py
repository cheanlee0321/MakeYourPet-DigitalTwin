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

        # 腿部幾何槓桿抬腿補償係數 (平衡中腿與前後角腿離地淨空，杜絕拖地與低抬腳)
        self.scale_middle = 1.0   # 中腿 (L2, R2) 標準離地
        self.scale_front = 1.0    # 前腿 (L1, R1) 標準離地
        self.scale_rear = 1.30    # 後腿 (L3, R3) 補償重力與俯仰角，杜絕後腿拖地

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

    def set_leg_lift_scales(self, scale_front: float = 1.0, scale_middle: float = 1.0, scale_rear: float = 1.30):
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
                # 擺動期 (Swing): 騰空大幅高抬腿向前邁步
                # 採用非線性指數波形 (sin(p)^0.8)，在抬足初態即迅猛拔高離地，越障絕不拖泥帶水
                h = float(np.sin(p) ** 0.8)
                # 依腿部構型 (前腿 L1/R1、中腿 L2/R2、後腿 L3/R3) 套用槓桿平衡補償係數
                if leg_idx in [1, 4]:    # 中腿 (L2, R2)
                    lift_mult = self.scale_middle
                elif leg_idx in [0, 3]:  # 前腿 (L1, R1)
                    lift_mult = self.scale_front
                else:                    # 後腿 (L3, R3)
                    lift_mult = self.scale_rear
                q_femur = -self.lift_femur * lift_mult * h
                q_tibia = -self.lift_tibia * lift_mult * h
            else:
                # 支撐期 (Stance): 貼地蹬踏向後推，柔順支撐
                q_femur = 0.01
                q_tibia = 0.005

            base_j = leg_idx * 3
            q_ref[base_j + 0] = q_coxa
            q_ref[base_j + 1] = q_femur + self.offset_hip
            q_ref[base_j + 2] = q_tibia + self.offset_tibia

        return q_ref
