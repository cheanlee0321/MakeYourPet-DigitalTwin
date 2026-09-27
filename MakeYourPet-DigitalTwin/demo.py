import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import argparse
import ctypes
import numpy as np
import mujoco
import mujoco.viewer
from stable_baselines3 import PPO
from hexapod_env import HexapodEnv

# Windows 虛擬按鍵碼 (Virtual Key Codes)
user32 = ctypes.windll.user32
VK_SHIFT = 0x10  # Shift 加速衝刺
VK_UP    = 0x26  # ↑ 方向鍵上 (前進)
VK_DOWN  = 0x28  # ↓ 方向鍵下 (倒退)
VK_LEFT  = 0x25  # ← 方向鍵左 (左轉)
VK_RIGHT = 0x27  # → 方向鍵右 (右轉)
VK_R     = 0x52  # R 鍵 (重置起點)
VK_T     = 0x54  # T 鍵 (鏡頭跟隨切換)
VK_BACK  = 0x08  # Backspace (重置起點)

# 檔位按鍵 (主鍵盤 1~4 與數字小鍵盤 NumPad 1~4)
VK_1 = 0x31
VK_2 = 0x32
VK_3 = 0x33
VK_4 = 0x34
VK_NUMPAD1 = 0x61
VK_NUMPAD2 = 0x62
VK_NUMPAD3 = 0x63
VK_NUMPAD4 = 0x64

# 檔位升降鍵 (+/- 或 [/])
VK_OEM_PLUS  = 0xBB  # =/+ 鍵
VK_OEM_MINUS = 0xBD  # -/_ 鍵
VK_ADD       = 0x6B  # 小鍵盤 +
VK_SUBTRACT  = 0x6D  # 小鍵盤 -
VK_LBRACKET  = 0xDB  # [ 鍵 (降檔)
VK_RBRACKET  = 0xDD  # ] 鍵 (升檔)

# 變速檔位定義 (步頻與速度設定檔)
GEAR_PROFILES = {
    1: {"name": "ECO 慢步微調/爬坡",  "freq": 1.0, "vx": 0.15, "back": 0.10, "yaw": 0.35, "shift_mult": 1.35},
    2: {"name": "NORMAL 標準巡航",     "freq": 1.5, "vx": 0.25, "back": 0.18, "yaw": 0.50, "shift_mult": 1.40},
    3: {"name": "SPORT 敏捷快跑",       "freq": 2.0, "vx": 0.36, "back": 0.22, "yaw": 0.65, "shift_mult": 1.25},
    4: {"name": "TURBO 極速狂飆",       "freq": 2.5, "vx": 0.45, "back": 0.25, "yaw": 0.75, "shift_mult": 1.20},
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
    parser.add_argument("--terrain-height", type=float, default=0.035,
                        help="地形最大起伏高度 (公尺，預設: 0.035 即 3.5cm，支援至 0.06m)")
    return parser.parse_args()

def print_controls():
    print("=" * 72)
    print("      🎮 MAKE YOUR PET 電玩級直覺遙控工作台（按住行走、放開停步）")
    print("=" * 72)
    print("【遙控指令（即時物理鍵盤響應，完全不衝突任何視圖熱鍵）】：")
    print("  [↑ 方向鍵上]   按住：大步前進行走   | 放開：立刻停步站穩")
    print("  [↓ 方向鍵下]   按住：平穩倒退行走   | 放開：立刻停步站穩")
    print("  [← 方向鍵左]   按住：原地向左轉向   | 放開：立刻停止轉彎")
    print("  [→ 方向鍵右]   按住：原地向右轉向   | 放開：立刻停止轉彎")
    print("  ----------------------------------------------------------------------")
    print("  🔥【組合操控】：同時按住 [↑] + [←] 或 [→]，可實現流暢的弧形前進轉彎！")
    print("  ⚡【衝刺加速】：按住 [Shift] + [↑]，可在當前檔位激發額外爆發速度！")
    print("  ----------------------------------------------------------------------")
    print("  ⚙️【檔位變速箱 (調節腳步步頻與跑步速度)】：")
    print("    [1] ~ [4] 鍵 / 小鍵盤 1~4：直達 1~4 檔位")
    print("      - [1 檔] ECO 慢步爬坡: 步頻 1.0 Hz | 巡航 0.15 m/s (超穩重高扭力)")
    print("      - [2 檔] NORMAL 標準:  步頻 1.5 Hz | 巡航 0.25 m/s (預設基準步態)")
    print("      - [3 檔] SPORT 快跑:   步頻 2.0 Hz | 巡航 0.36 m/s (敏捷高速奔馳)")
    print("      - [4 檔] TURBO 極速:   步頻 2.5 Hz | 巡航 0.45 m/s (飛速高頻狂飆)")
    print("    [+] / [-] 或 [[] / []]：逐級升檔 / 降檔")
    print("  🔄 [R 鍵] 或 [Backspace]：按一下將機器人重置回起點")
    print("  🎥 [T 鍵]：切換鏡頭自動追隨模式 (開/關)")
    print("  ⏸️ [空白鍵 (Space)]：暫停 / 繼續物理模擬")
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

    # 檔位控制系統 (預設 2 檔: 標準巡航 1.5 Hz)
    current_gear = 2
    env.step_frequency = GEAR_PROFILES[current_gear]["freq"]
    gear_keys_prev = {1: False, 2: False, 3: False, 4: False}
    gear_up_prev = False
    gear_down_prev = False

    # 印出說明文字
    print_controls()
    print(f"載入權重: {model_path}")
    terrain_desc = f"{args.terrain} (起伏: ±{args.terrain_height*100:.1f} cm)" if args.terrain != 'flat' else 'flat (平坦地面)'
    print(f"地貌設定: 【{terrain_desc}】 | 初始檔位: 【2 檔 {GEAR_PROFILES[2]['name']}】 (步頻: 1.5 Hz) | 鏡頭自動跟隨: {'開啟' if camera_tracking else '關閉'}\n")

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

            # ===== 4. 檔位切換檢測 (單擊觸發防抖動) =====
            def switch_gear(new_g):
                nonlocal current_gear
                if 1 <= new_g <= 4 and new_g != current_gear:
                    current_gear = new_g
                    cfg = GEAR_PROFILES[current_gear]
                    env.step_frequency = cfg["freq"]
                    print(f"  [檔位切換] ⚙️ 已切換至 【{current_gear} 檔: {cfg['name']}】 | 步頻: {cfg['freq']:.1f} Hz | 巡航: {cfg['vx']:.2f} m/s")

            # 檢測數字鍵直選檔位 (1~4 鍵與小鍵盤 1~4)
            direct_keys = [
                (1, [VK_1, VK_NUMPAD1]),
                (2, [VK_2, VK_NUMPAD2]),
                (3, [VK_3, VK_NUMPAD3]),
                (4, [VK_4, VK_NUMPAD4]),
            ]
            for g_num, vks in direct_keys:
                k_down = any(is_key_down(vk) for vk in vks)
                if k_down and not gear_keys_prev[g_num]:
                    switch_gear(g_num)
                gear_keys_prev[g_num] = k_down

            # 檢測升檔鍵 (+ / ] / NumPad +)
            up_down = is_key_down(VK_OEM_PLUS) or is_key_down(VK_ADD) or is_key_down(VK_RBRACKET)
            if up_down and not gear_up_prev:
                switch_gear(min(4, current_gear + 1))
            gear_up_prev = up_down

            # 檢測降檔鍵 (- / [ / NumPad -)
            down_down = is_key_down(VK_OEM_MINUS) or is_key_down(VK_SUBTRACT) or is_key_down(VK_LBRACKET)
            if down_down and not gear_down_prev:
                switch_gear(max(1, current_gear - 1))
            gear_down_prev = down_down

            # ===== 5. 實時檢測實體按鍵：按住即走、放開即停 =====
            gear_cfg = GEAR_PROFILES[current_gear]
            shift_pressed = is_key_down(VK_SHIFT)

            # 若按住 Shift 且前進：激發渦輪加速，速度提升並將步頻額外拉高 10%
            if shift_pressed and is_key_down(VK_UP):
                run_speed = gear_cfg["vx"] * gear_cfg["shift_mult"]
                env.step_frequency = gear_cfg["freq"] * 1.10
            else:
                run_speed = gear_cfg["vx"]
                env.step_frequency = gear_cfg["freq"]

            target_vx = 0.0
            target_vy = 0.0
            target_wz = 0.0

            if is_key_down(VK_UP):
                target_vx += run_speed
            if is_key_down(VK_DOWN):
                target_vx -= gear_cfg["back"]
            if is_key_down(VK_LEFT):
                target_wz += gear_cfg["yaw"]
            if is_key_down(VK_RIGHT):
                target_wz -= gear_cfg["yaw"]

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
                obs, info = env.reset()
                env.step_frequency = GEAR_PROFILES[current_gear]["freq"]
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

            # 策略推論預測動作 (18 維)
            action, _ = model.predict(obs, deterministic=not args.stochastic)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step_idx += 1

            # 定期在終端印出遙測狀態 (每 0.5 秒一次)
            if time.time() - last_print_time > 0.5:
                is_moving = abs(current_cmd[0]) > 0.01 or abs(current_cmd[2]) > 0.01
                status_icon = "🏃 [前進行駛中]" if current_cmd[0] > 0 else ("🔻 [倒退行駛中]" if current_cmd[0] < 0 else ("🔄 [原地轉彎中]" if abs(current_cmd[2]) > 0 else "🛑 [立定待命中]"))
                shift_tag = " (⚡TURBO加速)" if (shift_pressed and is_key_down(VK_UP)) else ""
                gear_tag = f"⚙️[{current_gear}檔 {env.step_frequency:.1f}Hz]"
                
                print(f"{gear_tag} {status_icon}{shift_tag} 目標: [vx={current_cmd[0]:+.2f}, yaw={current_cmd[2]:+.2f}] | "
                      f"實際: [vx={info['vx']:+.3f}, yaw={info['yaw_rate']:+.3f}] | "
                      f"高度: {info['height']:.3f} m | 累積獎勵: {episode_reward:+.1f}")
                last_print_time = time.time()

            # 跌倒時自動重置位置；步數上限到達時只靜默重計 (不打斷操控)
            if terminated:
                print(f"\n[回合結束] ⚠️ 失去平衡翻倒 (存活步數: {step_idx}, 總獎勵: {episode_reward:.2f}) -> 自動重置\n")
                obs, info = env.reset()
                env.step_frequency = GEAR_PROFILES[current_gear]["freq"]
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

if __name__ == "__main__":
    main()
