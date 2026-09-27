"""
Test Open-Loop Tripod Kinematics in MuJoCo
驗證開環三角步態運動學是否能驅動六足機器人穩定前進、轉向與煞車
"""
import time
import mujoco
import numpy as np

def get_tripod_reference(phase, cmd):
    """
    計算給定步態相位 phase 與指令 cmd = [vx, vy, yaw_rate] 下的 18 個關節參考角度
    關節順序:
    0..2:   L1 (c, f, t)
    3..5:   L2 (c, f, t)
    6..8:   L3 (c, f, t)
    9..11:  R1 (c, f, t)
    12..14: R2 (c, f, t)
    15..17: R3 (c, f, t)
    """
    vx, vy, yaw = cmd
    q_ref = np.zeros(18, dtype=np.float32)

    # 判斷是否靜止待命
    speed = np.sqrt(vx**2 + vy**2) + abs(yaw)
    if speed < 0.01:
        return q_ref

    # 步幅縮放 (依指令線速度與角速度計算各腿行程)
    # 限制最大行程在 safe actuator range (-45 to 45 deg, 即 -0.785 到 0.785 rad)
    # 取適當步幅：vx = 0.25 m/s 對應 coxa 擺幅約 0.30 rad (17度)
    stride_base = vx * 1.2
    yaw_contrib = yaw * 0.25

    # 腿分組相位: Tripod A (L1, R2, L3), Tripod B (R1, L2, R3)
    # phase in [0, 2*pi)
    phase_A = phase % (2.0 * np.pi)
    phase_B = (phase + np.pi) % (2.0 * np.pi)

    # 6 腿對應的相位與左右側標識
    # (leg_idx, tripod_phase, is_right, yaw_mult)
    legs_cfg = [
        (0, phase_A, False, -1.0), # L1 (Tripod A, Left)
        (1, phase_B, False, -1.0), # L2 (Tripod B, Left)
        (2, phase_A, False, -1.0), # L3 (Tripod A, Left)
        (3, phase_B, True,   1.0), # R1 (Tripod B, Right)
        (4, phase_A, True,   1.0), # R2 (Tripod A, Right)
        (5, phase_B, True,   1.0), # R3 (Tripod B, Right)
    ]

    lift_amp_femur = 0.28 # 抬腿大腿向上角度 (約 16 度)
    lift_amp_tibia = 0.18 # 抬腿小腿屈曲角度 (約 10 度)

    for leg_idx, p, is_right, yaw_mult in legs_cfg:
        # 1. 計算該腿前進行程 (結合前進速度與轉向差速)
        leg_stride = stride_base + yaw_contrib * yaw_mult
        leg_stride = np.clip(leg_stride, -0.40, 0.40)

        # 2. Coxa 擺動角度 (橫擺)
        # Left legs:  +coxa moves backward, so stance (push backward) needs increasing coxa
        # Right legs: -coxa moves backward, so stance (push backward) needs decreasing coxa
        if not is_right:
            q_coxa = np.cos(p) * leg_stride
        else:
            q_coxa = -np.cos(p) * leg_stride

        # 3. 垂直抬腿 (Swing Phase: p in [0, pi))
        if p < np.pi:
            # 擺動期 (Swing): 抬起足端懸空
            # sin(p) 在 [0, pi] 之間為正值 (0 -> 1 -> 0)
            h = np.sin(p)
            q_femur = -lift_amp_femur * h
            q_tibia = -lift_amp_tibia * h
        else:
            # 支撐期 (Stance: p in [pi, 2pi)): 穩固接觸地面蹬踏
            q_femur = 0.02 # 微幅下壓，確保充足地表正壓力
            q_tibia = 0.01

        # 指派關節角度
        base_j = leg_idx * 3
        q_ref[base_j + 0] = q_coxa
        q_ref[base_j + 1] = q_femur
        q_ref[base_j + 2] = q_tibia

    return q_ref

def run_test():
    model = mujoco.MjModel.from_xml_path("models/hexapod.xml")
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.057
    mujoco.mj_forward(model, data)

    trunk_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    dt = 0.02 # 50Hz
    freq = 1.5 # 1.5 Hz 步頻
    cmd = [0.25, 0.0, 0.0] # 前進 0.25 m/s

    print(f"=== 測試開環運動學 (目標指令: vx={cmd[0]} m/s) ===")
    start_x = data.xpos[trunk_id, 0]
    phase = 0.0

    steps = 150 # 3 秒 (150 * 0.02s)
    for s in range(steps):
        phase = (phase + 2.0 * np.pi * freq * dt) % (2.0 * np.pi)
        q_ref = get_tripod_reference(phase, cmd)

        for i in range(18):
            data.ctrl[i] = q_ref[i]

        for _ in range(10): # 10 substeps of 0.002s = 0.02s
            mujoco.mj_step(model, data)

    end_x = data.xpos[trunk_id, 0]
    total_dist = end_x - start_x
    avg_speed = total_dist / (steps * dt)
    print(f"3秒後機身位移: {total_dist:.3f} m, 平均前進速度: {avg_speed:.3f} m/s")
    print(f"機身最終高度: {data.xpos[trunk_id, 2]:.3f} m")

if __name__ == "__main__":
    run_test()
