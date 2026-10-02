import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import math
import argparse
import ctypes
import numpy as np
import mujoco
import mujoco.viewer
from stable_baselines3 import PPO

_root_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_root_dir, "train_walking"))
sys.path.insert(0, os.path.join(_root_dir, "train_jumping"))

from hexapod_env import HexapodEnv
from jump_controller import JumpController, JumpState

# 抑制 Pygame 歡迎字樣，並啟用 Joystick 子系統
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame

# Windows 虛擬按鍵碼 (Virtual Key Codes)
user32 = ctypes.windll.user32
VK_SHIFT = 0x10  # Shift 加速衝刺
VK_UP    = 0x26  # ↑ 方向鍵上 (前進)
VK_DOWN  = 0x28  # ↓ 方向鍵下 (倒退)
VK_LEFT  = 0x25  # ← 方向鍵左 (左轉，或 Shift+← 側向左移)
VK_RIGHT = 0x27  # → 方向鍵右 (右轉，或 Shift+→ 側向右移)
VK_Q     = 0x51  # Q 鍵 (向左側向平移 Strafe Left，完全避開 MuJoCo 視圖衝突)
VK_E     = 0x45  # E 鍵 (向右側向平移 Strafe Right，完全避開 MuJoCo 視圖衝突)
VK_R     = 0x52  # R 鍵 (重置起點)
VK_T     = 0x54  # T 鍵 (鏡頭跟隨切換)
VK_BACK  = 0x08  # Backspace (重置起點)

# 檔位與動作按鍵 (主鍵盤 1~6 與數字小鍵盤 NumPad 1~6)
VK_1 = 0x31
VK_2 = 0x32
VK_3 = 0x33
VK_4 = 0x34
VK_5 = 0x35  # 5 鍵 (50cm 爆發大跳)
VK_6 = 0x36  # 6 鍵 (70cm 火箭超跳)
VK_NUMPAD1 = 0x61
VK_NUMPAD2 = 0x62
VK_NUMPAD3 = 0x63
VK_NUMPAD4 = 0x64
VK_NUMPAD5 = 0x65  # 小鍵盤 5 (50cm 爆發大跳)
VK_NUMPAD6 = 0x66  # 小鍵盤 6 (70cm 火箭超跳)

# 檔位升降鍵 (+/- 或 [/])
VK_OEM_PLUS  = 0xBB  # =/+ 鍵
VK_OEM_MINUS = 0xBD  # -/_ 鍵
VK_ADD       = 0x6B  # 小鍵盤 +
VK_SUBTRACT  = 0x6D  # 小鍵盤 -
VK_LBRACKET  = 0xDB  # [ 鍵 (降檔)
VK_RBRACKET  = 0xDD  # ] 鍵 (升檔)

# 速度檔位定義 (調節腳步移動頻率與目標巡航速度，前後完全對稱)
SPEED_PROFILES = {
    1: {"name": "ECO 慢步微調/爬坡", "freq": 1.0, "vx": 0.15, "back": 0.15, "yaw": 0.35, "shift_mult": 1.35},
    2: {"name": "NORMAL 標準巡航",    "freq": 1.8, "vx": 0.30, "back": 0.30, "yaw": 0.50, "shift_mult": 1.35},
    3: {"name": "TURBO 極速狂飆",      "freq": 2.5, "vx": 0.45, "back": 0.45, "yaw": 0.75, "shift_mult": 1.20},
}

# 姿態模式定義 (透過 4 鍵獨立 Toggle 開關切換，不影響 1~3 檔速度頻率)
POSTURE_PROFILES = {
    False: {"name": "一般基準姿態",         "offset_hip": 0.0, "offset_tibia": 0.0},
    True:  {"name": "OFFROAD 越野挺身姿態", "offset_hip": np.deg2rad(10.0), "offset_tibia": np.deg2rad(10.0)},
}

def is_key_down(vk_code: int) -> bool:
    """實時檢測實體按鍵是否正被按住 (按住為 True，放開為 False)"""
    return bool(user32.GetAsyncKeyState(vk_code) & 0x8000)

def parse_args():
    parser = argparse.ArgumentParser(description="Make Your Pet 六足機器人 AI 步態即時 3D 視覺化與電玩級遙控工作台")
    parser.add_argument("--model", type=str, default=None,
                        help="欲載入的模型路徑 (預設依序嘗試 models/best_model/best_model.zip 或 models/hexapod_final_policy.zip)")
    parser.add_argument("--stochastic", action="store_true",
                        help="啟用隨機採樣推論 (預設為確定性最佳策略)")
    parser.add_argument("--no-tracking", action="store_true",
                        help="關閉鏡頭自動跟隨機器人")
    parser.add_argument("--terrain", type=str, default="park", choices=["flat", "bumps", "blocks", "park", "rough", "slope"],
                        help="3D 地表起伏類型 (flat: 經典平地, blocks: 階梯石柱區塊陣, park: 4大主題複合越野公園, bumps: 密集高頻波浪, rough: 嶙峋碎石, slope: 傾斜坡道，預設: park)")
    parser.add_argument("--terrain-height", type=float, default=0.045,
                        help="地形最大起伏高度 (公尺，預設: 0.045 即 4.5cm，支援至 0.08m / 8.0cm)")
    parser.add_argument("--lift-femur", type=float, default=None,
                        help="自訂大腿抬升幅度 (rad，預設越野超高抬腿: 0.55 rad 約 31.5 度)")
    parser.add_argument("--lift-tibia", type=float, default=None,
                        help="自訂小腿屈折幅度 (rad，預設越野超高抬腿: 0.36 rad 約 20.6 度)")
    parser.add_argument("--jump-model", type=str, default=None,
                        help="欲載入的立定跳躍殘差模型路徑 (若未指定，會自動嘗試載入 models/jump_best_model/best_model.zip)")
    return parser.parse_args()

def print_controls():
    print("=" * 72)
    print("      🎮 MAKE YOUR PET 電玩級直覺遙控工作台（按住行走、放開停步）")
    print("=" * 72)
    print("【遙控指令（即時物理鍵盤響應，完全不衝突任何視圖熱鍵）】：")
    print("  [↑ 方向鍵上]   按住：大步前進行走   | 放開：立刻停步站穩")
    print("  [↓ 方向鍵下]   按住：平穩倒退行走   | 放開：立刻停步站穩")
    print("  [← 方向鍵左]   按住：原地向左轉向   | 放開：立刻停止轉彎 (單獨按)")
    print("  [→ 方向鍵右]   按住：原地向右轉向   | 放開：立刻停止轉彎 (單獨按)")
    print("  ----------------------------------------------------------------------")
    print("  🦀【側向平移 / 蟹步橫移 (Strafe)】：")
    print("    [Q 鍵]       按住：往左側向平移   | 放開：立刻停止橫移 (零視圖衝突)")
    print("    [E 鍵]       按住：往右側向平移   | 放開：立刻停止橫移 (零視圖衝突)")
    print("    [Shift] + [← / →]：側向向左 / 向右平移！")
    print("  🔥【全向走位】：同時按住 [↑] + [Q / E]，可實現流暢的 45° 斜向走位！")
    print("  ⚡【衝刺加速】：按住 [Shift] + [↑]，可在當前檔位激發額外爆發速度！")
    print("  ----------------------------------------------------------------------")
    print("  ⚙️【速度與步頻控制 (1~3 檔獨立切換速度)】：")
    print("    [1] 鍵 / 小鍵盤 1:  ECO 慢步爬坡  (步頻 1.0 Hz | 巡航 0.15 m/s)")
    print("    [2] 鍵 / 小鍵盤 2:  NORMAL 標準  (步頻 1.5 Hz | 巡航 0.25 m/s，預設基準)")
    print("    [3] 鍵 / 小鍵盤 3:  TURBO 極速   (步頻 2.5 Hz | 巡航 0.45 m/s)")
    print("    [+] / [-] 或 [[] / []]：逐級加速 / 減速 (1~3 檔)")
    print("  ----------------------------------------------------------------------")
    print("  🏔️【越野姿態開關 (4 鍵 Toggle 切換)】：")
    print("    [4] 鍵 / 小鍵盤 4：按一下切換為【OFFROAD 越野挺身姿態】(超高底盤避障)")
    print("                      再按一下切換回【一般基準姿態】")
    print("    💡【雙軸完全獨立】：您可按 [4] 挺身，並同時按 [1]~[3] 自由調整快慢！")
    print("  ----------------------------------------------------------------------")
    print("  🚀【立定跳躍雙檔模式 (按鍵 5 與 6)】：")
    print("    [5] 鍵 / 小鍵盤 5：觸發【50cm 爆發大跳 (Standard High Jump)】")
    print("    [6] 鍵 / 小鍵盤 6：激發【70cm 火箭超跳 (Super Rocket Jump)】(飛越人身腰部！)")
    print("                      (安全深蹲蓄力 -> 瞬間全功率爆發 -> 騰空伸足迎接地面 -> 腳掌動態觸地吸震，腹部全程零觸地)")
    print("  ----------------------------------------------------------------------")
    print("  🔄 [R 鍵] 或 [Backspace]：按一下將機器人重置回起點")
    print("  🎥 [T 鍵]：切換鏡頭自動追隨模式 (開/關)")
    print("  ⏸️ [空白鍵 (Space)]：暫停 / 繼續物理模擬")
    print("  ----------------------------------------------------------------------")
    print("  🎮【雙模遙控器 / 遊戲手把配置（自動識別無縫支援）】：")
    print("    • 【BetaFPV 穿越機遙控器 (Mode 2)】：")
    print("      - 右搖桿 (Pitch)：推前 = 前進 | 拉後 = 倒退")
    print("      - 右搖桿 (Roll) ：推左 = 往左側平移 | 推右 = 往右側平移")
    print("      - 左搖桿 (Yaw)  ：推左 = 原地左轉 | 推右 = 原地右轉")
    print("      - SA(左外)=Reset | SB(左內)=三段切速 | SC(右內)=三段姿態(上:基準, 中:無Mapping, 下:越野挺身)")
    print("    • 【標準 PC 遊戲手把 (Xbox / PlayStation / Logitech)】：")
    print("      - 左搖桿 (Left Stick)：360° 全向平移 (前後行走 + 左右橫移，FPS 電玩手感)")
    print("      - 右搖桿 (Right Stick 水平)：原地向左 / 向右轉向")
    print("      - 十字鍵 (D-Pad 左/右)：精確微步橫移")
    print("      - 肩鍵 (LB / RB)：快速側向滑步 (Strafe Dash)")
    print("      - A鍵=循環切速 | B鍵=越野挺身 | X鍵=視角跟隨 | Y鍵=重置起點")
    print("    💡【雙控並行】：鍵盤與遙控手把可同時並行操控，無衝突即時響應！")
    print("-" * 72)
    print("【滑鼠 3D 視角與外力干擾測試】：")
    print("  滑鼠左鍵拖曳: 360° 旋轉視角       滑鼠右鍵拖曳: 平移視角")
    print("  滑鼠滾輪: 縮放視角")
    print("  💥 按住 [Ctrl] + 滑鼠左鍵點擊機器人拖拉：施加推擠外力，測試抗推平衡！")
    print("=" * 72 + "\n")

def main():
    args = parse_args()

    # 1. 尋找模型路徑
    candidate_paths = [
        args.model,
        os.path.join("models", "best_model", "best_model.zip"),
        os.path.join("models", "hexapod_final_policy.zip"),
        "hexapod_best_policy.zip"
    ]
    model_path = None
    for p in candidate_paths:
        if p and os.path.exists(p):
            model_path = p
            break

    if not model_path:
        print(f"[錯誤] 找不到可用的模型檔案！請先執行 train.py 完成訓練。")
        print(f"嘗試過的路徑: {[p for p in candidate_paths if p]}")
        return

    # 2. 初始化乾淨展示環境 (靜止待命，關閉自動切換指令)
    env = HexapodEnv(
        domain_randomization=False,
        auto_resample_commands=False,
        terrain_type=args.terrain,
        terrain_height=args.terrain_height
    )
    env.max_steps = 50000  # 延長至 ~16 分鐘 (原本 1000 步 = 20 秒太短)
    if args.lift_femur is not None and args.lift_tibia is not None:
        env.kinematics.set_lift_height(args.lift_femur, args.lift_tibia)
        print(f"  [運動學] 🦾 套用自訂抬腿幅度: Femur={args.lift_femur:.2f} rad, Tibia={args.lift_tibia:.2f} rad")

    current_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    env.command = current_cmd.copy()

    # 載入策略模型
    model = PPO.load(model_path, device="cpu")
    obs, info = env.reset(seed=42)
    obs[-5:-2] = current_cmd

    # 鏡頭跟隨狀態
    camera_tracking = not args.no_tracking
    reset_requested = False
    r_key_prev = False
    t_key_prev = False

    # 速度檔位與越野姿態控制系統 (雙軸完全獨立)
    current_speed = 2   # 預設 2 檔: 標準巡航 1.5 Hz
    is_offroad = False  # 預設 False: 一般基準姿態

    env.step_frequency = SPEED_PROFILES[current_speed]["freq"]
    env.set_joint_offsets(POSTURE_PROFILES[is_offroad]["offset_hip"], POSTURE_PROFILES[is_offroad]["offset_tibia"])

    speed_keys_prev = {1: False, 2: False, 3: False}
    key_4_prev = False
    key_5_prev = False
    key_6_prev = False
    speed_up_prev = False
    speed_down_prev = False

    # 立定跳躍控制器 (配備瞬間過載脈衝爆發與伸足著地防腹部觸地保護)
    jump_ctrl = JumpController(dt=env.dt, model=env.model, enable_burst=True)

    # 載入立定跳躍殘差強化學習模型 (若有)
    candidate_jump_paths = [
        args.jump_model,
        os.path.join("models", "jump_best_model", "best_model.zip"),
        os.path.join("models", "jump_final_policy.zip"),
    ]
    jump_model_path = None
    for jp in candidate_jump_paths:
        if jp and os.path.exists(jp):
            jump_model_path = jp
            break

    jump_model = None
    if jump_model_path:
        jump_model = PPO.load(jump_model_path, device="cpu")
        jump_status_str = f"智慧閉環殘差策略 ({jump_model_path})"
    else:
        jump_status_str = "基準數字 5 開環姿態 (待訓練完畢自動加載)"
    prev_jump_action = np.zeros(18, dtype=np.float32)

    # 印出說明文字
    print_controls()
    print(f"載入步態權重: {model_path}")
    print(f"立定跳躍模式: 【{jump_status_str}】")
    terrain_desc = f"{args.terrain} (起伏: ±{args.terrain_height*100:.1f} cm)" if args.terrain != 'flat' else 'flat (平坦地面)'
    # 初始化遊戲手把 / BetaFPV 遙控器系統 (若有連接)
    pygame.init()
    pygame.joystick.init()
    joystick = None
    prev_js_buttons = {}
    prev_sa_val = 0.0
    prev_sb_state = 2
    prev_sc_state = 1
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        if hasattr(joystick, "get_init") and not joystick.get_init():
            joystick.init()
        print(f"  🎮 【已偵測並啟用遙控器/手把】: {joystick.get_name()} (類比軸: {joystick.get_numaxes()} | 按鍵: {joystick.get_numbuttons()})")
        print(f"     支援 BetaFPV / 遊戲手把無縫混用操作，鍵盤與遙控器可同時並行使用！")
        is_fpv = any(k in joystick.get_name().lower() for k in ["betafpv", "radio", "edgetx", "opentx", "taranis", "elrs", "frsky"])
        if is_fpv and joystick.get_numaxes() > 4:
            print("     🎚️ 【BetaFPV 撥桿實體對齊】: SA(左外)=Reset | SB(左內)=三段速度 | SC(右內)=三段姿態(上:基準/中:無/下:越野) | SD(右外)=空白\n")
            num_a = joystick.get_numaxes()
            prev_sa_val = joystick.get_axis(7) if num_a > 7 else 0.0
            prev_sb_state = (1 if joystick.get_axis(6) < -0.33 else (3 if joystick.get_axis(6) > 0.33 else 2)) if num_a > 6 else 2
            prev_sc_state = (1 if joystick.get_axis(5) < -0.33 else (3 if joystick.get_axis(5) > 0.33 else 2)) if num_a > 5 else 1
        else:
            print()
        prev_js_buttons = {i: bool(joystick.get_button(i)) for i in range(joystick.get_numbuttons())}
    else:
        print("  ℹ️ 【遊戲手把未連接】: 僅使用鍵盤操控 (隨時插入手把重開即可支援)\n")

    # 3. 啟動 MuJoCo 互動視窗
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        # 設定鏡頭初始追蹤
        if camera_tracking:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = env.trunk_id
            viewer.cam.distance = 0.95
            viewer.cam.elevation = -22
            viewer.cam.azimuth = 90

        step_idx = 0
        episode_reward = 0.0
        last_print_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # ===== 4. 速度切換 (1~3 鍵) 與 越野姿態開關 (4 鍵 Toggle) =====
            def switch_speed(new_s):
                nonlocal current_speed
                if 1 <= new_s <= 3 and new_s != current_speed:
                    current_speed = new_s
                    cfg = SPEED_PROFILES[current_speed]
                    env.step_frequency = cfg["freq"]
                    p_name = POSTURE_PROFILES[is_offroad]["name"]
                    print(f"  [速度切換] ⚡ 已切換至 【{current_speed} 檔: {cfg['name']}】 | 步頻: {cfg['freq']:.1f} Hz | 巡航: {cfg['vx']:.2f} m/s | 姿態: 【{p_name}】")

            def set_offroad(new_offroad: bool):
                nonlocal is_offroad
                if new_offroad != is_offroad:
                    is_offroad = new_offroad
                    p_cfg = POSTURE_PROFILES[is_offroad]
                    env.set_joint_offsets(p_cfg["offset_hip"], p_cfg["offset_tibia"])
                    s_cfg = SPEED_PROFILES[current_speed]
                    if is_offroad:
                        posture_info = f"【{p_cfg['name']}】 (超高離地挺身: Hip +10° / Knee +10°)"
                    else:
                        posture_info = f"【{p_cfg['name']}】 (基準自然拱步姿態)"
                    print(f"  [姿態切換] 🏔️ 已切換為 {posture_info} | 當前速度: 【{current_speed} 檔 {s_cfg['name']}】 ({s_cfg['freq']:.1f} Hz)")

            def toggle_offroad():
                set_offroad(not is_offroad)

            # 檢測 1~3 鍵直選速度檔位 (主鍵盤 1~3 與小鍵盤 1~3)
            for s_num, vks in [(1, [VK_1, VK_NUMPAD1]), (2, [VK_2, VK_NUMPAD2]), (3, [VK_3, VK_NUMPAD3])]:
                k_down = any(is_key_down(vk) for vk in vks)
                if k_down and not speed_keys_prev[s_num]:
                    switch_speed(s_num)
                speed_keys_prev[s_num] = k_down

            # 檢測 4 鍵切換越野姿態 (單擊 Toggle 開關防抖動：按一下越野挺身、再按一下回一般)
            k4_down = is_key_down(VK_4) or is_key_down(VK_NUMPAD4)
            if k4_down and not key_4_prev:
                toggle_offroad()
            key_4_prev = k4_down

            # 檢測 5 鍵觸發 50cm 爆發大跳 (單擊防抖動)
            k5_down = is_key_down(VK_5) or is_key_down(VK_NUMPAD5)
            if k5_down and not key_5_prev:
                if jump_ctrl.trigger(power=1.15):
                    print("\n  [動作] ⚡ 【5 鍵 50cm 爆發大跳】激發！全六足同步蓄力起跳！")
            key_5_prev = k5_down

            # 檢測 6 鍵觸發 70cm 火箭超跳 (單擊防抖動)
            k6_down = is_key_down(VK_6) or is_key_down(VK_NUMPAD6)
            if k6_down and not key_6_prev:
                if jump_ctrl.trigger(power=1.35):
                    print("\n  [動作] 🔥 【6 鍵 70cm 火箭超跳】激發！全六足極限全功率飛躍！")
            key_6_prev = k6_down

            # 檢測升速檔鍵 (+ / ] / NumPad +)
            up_down = is_key_down(VK_OEM_PLUS) or is_key_down(VK_ADD) or is_key_down(VK_RBRACKET)
            if up_down and not speed_up_prev:
                switch_speed(min(3, current_speed + 1))
            speed_up_prev = up_down

            # 檢測降速檔鍵 (- / [ / NumPad -)
            down_down = is_key_down(VK_OEM_MINUS) or is_key_down(VK_SUBTRACT) or is_key_down(VK_LBRACKET)
            if down_down and not speed_down_prev:
                switch_speed(max(1, current_speed - 1))
            speed_down_prev = down_down

            # ===== 5. 實時檢測實體按鍵：按住即走、放開即停 =====
            speed_cfg = SPEED_PROFILES[current_speed]
            shift_pressed = is_key_down(VK_SHIFT)

            # 若按住 Shift 且前進或倒退：激發渦輪加速，速度提升並將步頻額外拉高 10%
            is_fwd_or_back = is_key_down(VK_UP) or is_key_down(VK_DOWN)
            if shift_pressed and is_fwd_or_back:
                run_speed = speed_cfg["vx"] * speed_cfg["shift_mult"]
                env.step_frequency = speed_cfg["freq"] * 1.10
            else:
                run_speed = speed_cfg["vx"]
                env.step_frequency = speed_cfg["freq"]

            target_vx = 0.0
            target_vy = 0.0
            target_wz = 0.0

            strafe_speed = speed_cfg["vx"] * 0.85

            # 1. 鍵盤前後行走 (前後完全對稱)
            if is_key_down(VK_UP):
                target_vx += run_speed
            if is_key_down(VK_DOWN):
                target_vx -= run_speed

            # 2. 鍵盤側向橫移：Q / E 鍵 (完全不衝突 MuJoCo 視圖)
            if is_key_down(VK_Q):
                target_vy += strafe_speed
            if is_key_down(VK_E):
                target_vy -= strafe_speed

            # 3. 鍵盤方向鍵左右：Shift+左右 為側向平移；單獨左右 為原地旋轉
            if shift_pressed:
                if is_key_down(VK_LEFT):
                    target_vy += strafe_speed
                if is_key_down(VK_RIGHT):
                    target_vy -= strafe_speed
            else:
                if is_key_down(VK_LEFT):
                    target_wz += speed_cfg["yaw"]
                if is_key_down(VK_RIGHT):
                    target_wz -= speed_cfg["yaw"]

            # ===== 5.1 讀取 BetaFPV / 遊戲手把類比搖桿與撥桿 (若有連接) =====
            js_vx = 0.0
            js_vy = 0.0
            js_wz = 0.0

            if joystick is not None:
                pygame.event.pump()

                # 讀取按鍵 / 撥桿切換 (上升邊緣單擊觸發，防連續重複觸發)
                for btn_idx in range(joystick.get_numbuttons()):
                    b_now = bool(joystick.get_button(btn_idx))
                    b_prev = prev_js_buttons.get(btn_idx, False)
                    if b_now and not b_prev:
                        # 0 號按鍵 (或手把 A 鍵): 循環切換 1~3 速度檔位 (ECO -> NORMAL -> TURBO)
                        if btn_idx == 0:
                            next_s = 1 if current_speed == 3 else current_speed + 1
                            switch_speed(next_s)
                        # 1 號按鍵 (或手把 B 鍵): 單擊切換 OFFROAD 越野挺身姿態
                        elif btn_idx == 1:
                            toggle_offroad()
                        # 2 號按鍵 (或手把 X 鍵): 切換鏡頭自動追隨模式 (T)
                        elif btn_idx == 2:
                            camera_tracking = not camera_tracking
                            print(f"  [視角] 🎥 遙控器觸發鏡頭自動跟隨: {'開啟 (Tracking ON)' if camera_tracking else '關閉 (Free Camera)'}")
                        # 3 號按鍵 (或手把 Y 鍵): 重置機器人回起點 (R)
                        elif btn_idx == 3:
                            reset_requested = True
                            print(f"  [指令] 🔄 遙控器觸發機器人重置回起點！")
                        # 💡 依指定需求：先不配置跳躍 (跳躍保持為鍵盤 5/6 鍵專屬)
                    prev_js_buttons[btn_idx] = b_now

                is_fpv_radio = any(k in joystick.get_name().lower() for k in ["betafpv", "radio", "edgetx", "opentx", "taranis", "elrs", "frsky"])

                # --- 專門處理 BetaFPV 撥桿 (SA, SB, SC, SD 實體對齊) ---
                if is_fpv_radio and joystick.get_numaxes() > 4:
                    num_a = joystick.get_numaxes()

                    # 1. 【SA 撥桿 (左外 / CH8 / Axis 7)】: 按鍵式 Reset 重置回起點 (撥動即觸發重置)
                    if num_a > 7:
                        val_sa = joystick.get_axis(7)
                        if abs(val_sa - prev_sa_val) > 0.4:
                            prev_sa_val = val_sa
                            reset_requested = True
                            print("  [指令] 🔄 遙控器 【SA 撥桿 (左外)】觸發機器人重置回起點！")

                    # 2. 【SB 撥桿 (左內 / CH7 / Axis 6)】: 三段式切換速度 (上=1檔 ECO, 中=2檔 NORMAL, 下=3檔 TURBO)
                    if num_a > 6:
                        val_sb = joystick.get_axis(6)
                        sb_state = 1 if val_sb < -0.33 else (3 if val_sb > 0.33 else 2)
                        if sb_state != prev_sb_state:
                            prev_sb_state = sb_state
                            switch_speed(sb_state)

                    # 3. 【SC 撥桿 (右內 / CH6 / Axis 5)】: 三段式姿態控制 (上=1 一般基準姿態, 中=2 無Mapping保持原狀, 下=3 OFFROAD越野挺身姿態)
                    if num_a > 5:
                        val_sc = joystick.get_axis(5)
                        sc_state = 1 if val_sc < -0.33 else (3 if val_sc > 0.33 else 2)
                        if sc_state != prev_sc_state:
                            prev_sc_state = sc_state
                            if sc_state == 1:
                                set_offroad(False)
                            elif sc_state == 3:
                                set_offroad(True)
                            # sc_state == 2: 中間檔位無 Mapping (未指派獨立動作，保持原姿態)

                    # 4. 【SD 撥桿 (右外 / CH5 / Axis 4)】: 空白未指派 (閒置保持無動作，完全忽略)

                # 讀取類比軸向 (套用死區過濾 deadzone)
                def get_filtered_axis(ax_idx, deadzone=0.08):
                    if ax_idx < joystick.get_numaxes():
                        v = joystick.get_axis(ax_idx)
                        if abs(v) < deadzone:
                            return 0.0
                        sign = 1.0 if v > 0 else -1.0
                        return sign * ((abs(v) - deadzone) / (1.0 - deadzone))
                    return 0.0

                if is_fpv_radio:
                    # ===== 模式 A: BetaFPV / 航模遙控器 (Mode 2 飛控手感) =====
                    # Axis 1 (Pitch 俯仰)：推前為正，拉後為負 -> 前進/後退速度
                    # Axis 0 (Roll 橫滾)  ：推右為正，推左為負 -> 左右側向平移 (Strafe)
                    # Axis 3 (Yaw 偏航)   ：推左為負 (wz>0 向左轉)，推右為正 (wz<0 向右轉)
                    raw_pitch = get_filtered_axis(1)
                    raw_roll  = get_filtered_axis(0)
                    raw_yaw   = get_filtered_axis(3 if joystick.get_numaxes() >= 4 else 0)

                    pitch_fwd = raw_pitch
                    js_vx = pitch_fwd * run_speed

                    # 機器人坐標系: +Y 為左側，-Y 為右側。搖桿推左(負)輸出 +vy (向左走)
                    js_vy = -raw_roll * (speed_cfg["vx"] * 0.85)
                    js_wz = -raw_yaw * speed_cfg["yaw"]

                else:
                    # ===== 模式 B: 標準 PC 遊戲手把 (Xbox / PlayStation / Logitech / 通用手把) =====
                    # 左搖桿: Axis 0 (左右橫移), Axis 1 (前後直行) -> 360° 全向電玩走位手感
                    raw_ls_x = get_filtered_axis(0)
                    raw_ls_y = get_filtered_axis(1)

                    pitch_fwd = -raw_ls_y  # 標準手把推前為負，因此 -raw_ls_y 為正向前進
                    js_vx = pitch_fwd * run_speed

                    # 左右橫移：左搖桿推左(負)輸出 +vy (向左走)，推右(正)輸出 -vy (向右走)
                    js_vy = -raw_ls_x * (speed_cfg["vx"] * 0.85)

                    # 右搖桿水平轉向 (Yaw): 自動適配不同驅動下的右搖桿 X 軸 (通常為 Axis 2, 3 或 4)
                    raw_rs_x = 0.0
                    for ax_cand in [2, 3, 4]:
                        val_c = get_filtered_axis(ax_cand)
                        if abs(val_c) > abs(raw_rs_x):
                            raw_rs_x = val_c
                    js_wz = -raw_rs_x * speed_cfg["yaw"]

                    # 十字鍵 D-pad (Hat 0) 微調：
                    if joystick.get_numhats() > 0:
                        hat_x, hat_y = joystick.get_hat(0)
                        if hat_x != 0:
                            # hat_x: -1 是向左 (+vy), +1 是向右 (-vy)
                            js_vy = -hat_x * (speed_cfg["vx"] * 0.85)
                        if hat_y != 0 and abs(js_vx) < 0.01:
                            js_vx = hat_y * run_speed

                    # 肩鍵 LB / RB 側滑衝刺 (LB=按鍵 4, RB=按鍵 5):
                    if joystick.get_numbuttons() > 5:
                        if joystick.get_button(4):  # LB: 往左側滑步
                            js_vy = speed_cfg["vx"] * 0.90
                        elif joystick.get_button(5):  # RB: 往右側滑步
                            js_vy = -speed_cfg["vx"] * 0.90

            # 鍵盤與遙控器混合判定 (取絕對值較大者，無縫切換不衝突)
            if abs(js_vx) > abs(target_vx):
                target_vx = js_vx
            if abs(js_vy) > abs(target_vy):
                target_vy = js_vy
            if abs(js_wz) > abs(target_wz):
                target_wz = js_wz

            # 檢測 R 鍵重置 (單擊觸發防抖動)
            r_now = is_key_down(VK_R) or is_key_down(VK_BACK)
            if r_now and not r_key_prev:
                reset_requested = True
                print(f"  [指令] 🔄 [R 鍵] 機器人已重置回起點！")
            r_key_prev = r_now

            # 檢測 T 鍵切換相機跟隨 (單擊觸發防抖動)
            t_now = is_key_down(VK_T)
            if t_now and not t_key_prev:
                camera_tracking = not camera_tracking
                print(f"  [視角] 🎥 鏡頭自動跟隨: {'開啟 (Tracking ON)' if camera_tracking else '關閉 (Free Camera)'}")
            t_key_prev = t_now

            # 更新當前指令
            current_cmd[0] = target_vx
            current_cmd[1] = target_vy
            current_cmd[2] = target_wz
            env.command = current_cmd.copy()

            # 處理手動重置
            if reset_requested:
                jump_ctrl.reset()
                obs, info = env.reset()
                env.step_frequency = SPEED_PROFILES[current_speed]["freq"]
                env.set_joint_offsets(POSTURE_PROFILES[is_offroad]["offset_hip"], POSTURE_PROFILES[is_offroad]["offset_tibia"])
                obs[-5:-2] = current_cmd
                reset_requested = False
                step_idx = 0
                episode_reward = 0.0

            # 同步鏡頭跟隨狀態
            if camera_tracking and viewer.cam.type != mujoco.mjtCamera.mjCAMERA_TRACKING:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = env.trunk_id
            elif not camera_tracking and viewer.cam.type == mujoco.mjtCamera.mjCAMERA_TRACKING:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE

            # 確保觀測中的指令為最新目標 (67 維中 [-5:-2] 為指令，[-2:] 為相位時鐘)
            obs[-5:-2] = current_cmd

            # 策略推論與動作執行 (支援立定跳躍控制器 FSM 優先覆蓋)
            if jump_ctrl.is_jumping:
                current_offsets = (POSTURE_PROFILES[is_offroad]["offset_hip"], POSTURE_PROFILES[is_offroad]["offset_tibia"])
                q_jump = jump_ctrl.step(current_posture_offsets=current_offsets, data=env.data)
                if q_jump is None:
                    q_jump = env.default_joint_angles.copy()

                if jump_model is not None:
                    # 構建 78 維跳躍殘差專屬觀測空間
                    quat = env.data.qpos[3:7]
                    w, x, y, z = quat
                    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                    pitch = math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
                    R = env.data.xmat[env.trunk_id].reshape(3, 3)
                    proj_gravity = R.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
                    omega = env.data.qvel[3:6]
                    v_body = R.T @ env.data.qvel[0:3]
                    local_ground_z = env._get_terrain_height_at(float(env.data.qpos[0]), float(env.data.qpos[1]))
                    rel_height = float(env.data.qpos[2] - local_ground_z)
                    current_joint_angles = env.data.qpos[7:25]
                    joint_error = current_joint_angles - q_jump
                    joint_vel = env.data.qvel[6:24] * 0.1

                    fsm_onehot = np.zeros(5, dtype=np.float32)
                    fsm_onehot[jump_ctrl.state] = 1.0
                    t_cur = jump_ctrl.state_time
                    ratio = min(1.0, t_cur / 0.60)

                    feet_contacts = np.zeros(6, dtype=np.float32)
                    for c in range(env.data.ncon):
                        con = env.data.contact[c]
                        for leg_i, tip_id in enumerate(env.tip_geom_ids):
                            if (con.geom1 == tip_id and con.geom2 == env.ground_id) or (con.geom2 == tip_id and con.geom1 == env.ground_id):
                                feet_contacts[leg_i] = 1.0

                    obs_jump = np.concatenate([
                        np.array([roll, pitch], dtype=np.float32),
                        proj_gravity.astype(np.float32),
                        omega.astype(np.float32),
                        v_body.astype(np.float32),
                        np.array([rel_height], dtype=np.float32),
                        joint_error.astype(np.float32),
                        joint_vel.astype(np.float32),
                        prev_jump_action.astype(np.float32),
                        fsm_onehot,
                        np.array([ratio], dtype=np.float32),
                        feet_contacts
                    ]).astype(np.float32)

                    jump_action, _ = jump_model.predict(obs_jump, deterministic=True)
                    fsm_state = jump_ctrl.state
                    if fsm_state == JumpState.THRUST:
                        eff_scale = 0.025
                    elif fsm_state == JumpState.CROUCH:
                        eff_scale = 0.040
                    elif fsm_state == JumpState.FLIGHT:
                        eff_scale = 0.150
                    elif fsm_state == JumpState.LANDING:
                        eff_scale = 0.180
                    else:
                        eff_scale = 0.060
                    q_override = q_jump + eff_scale * jump_action
                else:
                    q_override = q_jump

                obs, reward, terminated, truncated, info = env.step(np.zeros(18, dtype=np.float32), override_target_angles=q_override)
                if not jump_ctrl.is_jumping:
                    prev_jump_action = np.zeros(18, dtype=np.float32)
                    print("  [動作] 🛬 【立定跳躍完成】已成功平穩著地並恢復常態站姿！\n")
            else:
                action, _ = model.predict(obs, deterministic=not args.stochastic)
                obs, reward, terminated, truncated, info = env.step(action)

            episode_reward += reward
            step_idx += 1

            # 定期在終端印出遙測狀態 (每 0.5 秒一次)
            if time.time() - last_print_time > 0.5:
                if jump_ctrl.is_jumping:
                    status_icon = f"🚀 [{jump_ctrl.get_phase_name()}]"
                else:
                    cmd_x, cmd_y, cmd_w = current_cmd[0], current_cmd[1], current_cmd[2]
                    is_moving = abs(cmd_x) > 0.01 or abs(cmd_y) > 0.01 or abs(cmd_w) > 0.01
                    if not is_moving:
                        status_icon = "🛑 [立定待命中]"
                    elif abs(cmd_y) > 0.03 and abs(cmd_x) > 0.03:
                        status_icon = "↗️ [斜向行駛中]" if cmd_x > 0 else "↘️ [斜向倒退中]"
                    elif abs(cmd_y) > 0.03:
                        status_icon = "🦀 [向左平移中]" if cmd_y > 0 else "🦀 [向右平移中]"
                    elif cmd_x > 0:
                        status_icon = "🏃 [前進行駛中]"
                    elif cmd_x < 0:
                        status_icon = "🔻 [倒退行駛中]"
                    else:
                        status_icon = "🔄 [原地轉彎中]"

                shift_tag = " (⚡TURBO加速)" if (shift_pressed and (is_key_down(VK_UP) or is_key_down(VK_DOWN))) else ""
                posture_tag = "🏔️[OFFROAD越野姿態]" if is_offroad else "🌿[一般姿態]"
                gear_tag = f"⚡[{current_speed}檔 {env.step_frequency:.1f}Hz] {posture_tag}"
                
                rel_h = info.get('rel_height', info['height'])
                actual_vy = info.get('vy', 0.0)
                print(f"{gear_tag} {status_icon}{shift_tag} 目標: [vx={current_cmd[0]:+.2f}, vy={current_cmd[1]:+.2f}, yaw={current_cmd[2]:+.2f}] | "
                      f"實際: [vx={info['vx']:+.3f}, vy={actual_vy:+.3f}, yaw={info['yaw_rate']:+.3f}] | "
                      f"絕對高度: {info['height']:.3f}m (淨空: {rel_h*100:.1f}cm) | 累積獎勵: {episode_reward:+.1f}")
                last_print_time = time.time()

            # 跌倒時自動重置位置；步數上限到達時只靜默重計 (不打斷操控)
            if terminated:
                jump_ctrl.reset()
                print(f"\n[回合結束] ⚠️ 失去平衡翻倒 (存活步數: {step_idx}, 總獎勵: {episode_reward:.2f}) -> 自動重置\n")
                obs, info = env.reset()
                env.step_frequency = SPEED_PROFILES[current_speed]["freq"]
                env.set_joint_offsets(POSTURE_PROFILES[is_offroad]["offset_hip"], POSTURE_PROFILES[is_offroad]["offset_tibia"])
                obs[-5:-2] = current_cmd
                step_idx = 0
                episode_reward = 0.0
            elif truncated:
                # 步數上限到達，靜默重置計數器繼續操控
                env.step_count = 0
                step_idx = 0
                episode_reward = 0.0

            viewer.sync()

            # 維持 50 FPS (20ms 控制週期)
            time_until_next = 0.02 - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

    pygame.quit()

if __name__ == "__main__":
    main()
