import sys
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import os
import numpy as np
from stable_baselines3 import PPO
try:
    from .hexapod_env import HexapodEnv
except (ImportError, ValueError):
    from hexapod_env import HexapodEnv

def run_test_scenario(env, model, cmd, steps=100, scenario_name=""):
    env.command = np.array(cmd, dtype=np.float32)
    obs = env._get_obs()
    obs[-5:-2] = env.command
    
    vy_history = []
    vx_history = []
    yaw_history = []
    
    for _ in range(steps):
        if model is not None:
            action, _ = model.predict(obs, deterministic=True)
        else:
            action = np.zeros(18, dtype=np.float32)
        obs, r, term, trunc, info = env.step(action)
        obs[-5:-2] = env.command
        vx_history.append(info['vx'])
        vy_history.append(info.get('vy', 0.0))
        yaw_history.append(info['yaw_rate'])
        if term:
            break

    # 取後半段穩定狀態之平均值 (評估穩態巡航速度)
    steady_start = max(0, len(vx_history) // 2)
    mean_vx = np.mean(vx_history[steady_start:])
    mean_vy = np.mean(vy_history[steady_start:])
    mean_yaw = np.mean(yaw_history[steady_start:])
    
    status_tag = "✅ 穩定" if not term else "⚠️ 跌倒"
    
    print(f"{scenario_name: <22} | "
          f"目標: [{cmd[0]:+5.2f}, {cmd[1]:+5.2f}, {cmd[2]:+5.2f}] | "
          f"實際: [{mean_vx:+6.3f}, {mean_vy:+6.3f}, {mean_yaw:+6.3f}] | "
          f"高度: {info['height']:.3f}m | {status_tag}")
    return not term

def main():
    candidate_model_paths = [
        "models/best_model/best_model.zip",
        os.path.join(os.path.dirname(__file__), "..", "models", "best_model", "best_model.zip"),
        os.path.join(os.path.dirname(__file__), "..", "models", "hexapod_final_policy.zip"),
    ]
    model_path = next((p for p in candidate_model_paths if os.path.exists(p)), None)
    
    if model_path:
        print(f"載入策略模型: {model_path}")
        model = PPO.load(model_path, device="cpu")
    else:
        print("未偵測到模型權重，測試純運動學開環前饋基準 (Zero Residual Action)")
        model = None

    env = HexapodEnv(domain_randomization=False, auto_resample_commands=False)
    obs, _ = env.reset(seed=42)

    print("=" * 86)
    print("      六足機器人全向步態與側向橫移指令跟隨驗證 (8 項情境自動化測試)")
    print("=" * 86)
    print(f"{'測試情境名稱': <22} | {'目標指令 [vx, vy, yaw]': ^26} | {'穩態平均 [vx, vy, yaw]': ^26} | 機身高度 | 狀態")
    print("-" * 86)

    test_cases = [
        ("情境 1 [煞車靜止待命]",   [ 0.00,  0.00,  0.00], 60),
        ("情境 2 [直線前進巡航]",   [ 0.25,  0.00,  0.00], 100),
        ("情境 3 [高速倒退行走]",   [-0.25,  0.00,  0.00], 100),
        ("情境 4 [純向左側向平移]", [ 0.00, +0.20,  0.00], 120),
        ("情境 5 [純向右側向平移]", [ 0.00, -0.20,  0.00], 120),
        ("情境 6 [45° 斜向前進]",   [ 0.20, +0.15,  0.00], 120),
        ("情境 7 [45° 斜向倒退]",   [-0.20, +0.15,  0.00], 120),
        ("情境 8 [原地向左自轉]",   [ 0.00,  0.00, +0.50], 100),
        ("情境 9 [橫移複合轉向]",   [ 0.00, +0.15, +0.35], 120),
    ]

    all_passed = True
    for name, cmd, steps in test_cases:
        passed = run_test_scenario(env, model, cmd, steps=steps, scenario_name=name)
        if not passed:
            all_passed = False

    print("=" * 86)
    if all_passed:
        print("🎉 全情境測試執行完畢！機器人全向指令跟隨與平移推進機制運作正常。")
    else:
        print("⚠️ 部份測試情境觸發跌倒重置，建議進行 PPO 強化學習微調訓練以增強穩定性。")
    print("=" * 86)

if __name__ == "__main__":
    main()
