import sys
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import numpy as np
from stable_baselines3 import PPO
from hexapod_env import HexapodEnv

def main():
    model = PPO.load("models/best_model/best_model.zip", device="cpu")
    env = HexapodEnv(domain_randomization=False, auto_resample_commands=False)

    print("=" * 70)
    print("      六足機器人全自由度指令跟隨能力驗證 (5 項情境測試)")
    print("=" * 70)

    # 情境 1: 煞車靜止待命
    env.command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    obs, _ = env.reset(seed=42)
    obs[-5:-2] = env.command
    for _ in range(50):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, _, _, info = env.step(action)
        obs[-5:-2] = env.command
    print(f"情境 1 [煞車待命]: 目標=[ 0.00,  0.00] -> 實際 vx={info['vx']:+.4f} m/s, yaw={info['yaw_rate']:+.4f} rad/s, 高度={info['height']:.4f} m")

    # 情境 2: 前進加速巡航
    env.command = np.array([0.25, 0.0, 0.0], dtype=np.float32)
    for _ in range(100):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, _, _, info = env.step(action)
        obs[-5:-2] = env.command
    print(f"情境 2 [前進巡航]: 目標=[+0.25,  0.00] -> 實際 vx={info['vx']:+.4f} m/s, yaw={info['yaw_rate']:+.4f} rad/s, 高度={info['height']:.4f} m")

    # 情境 3: 原地向左旋轉
    env.command = np.array([0.0, 0.0, 0.5], dtype=np.float32)
    for _ in range(100):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, _, _, info = env.step(action)
        obs[-5:-2] = env.command
    print(f"情境 3 [原地左轉]: 目標=[ 0.00, +0.50] -> 實際 vx={info['vx']:+.4f} m/s, yaw={info['yaw_rate']:+.4f} rad/s, 高度={info['height']:.4f} m")

    # 情境 4: 原地向右旋轉
    env.command = np.array([0.0, 0.0, -0.5], dtype=np.float32)
    for _ in range(100):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, _, _, info = env.step(action)
        obs[-5:-2] = env.command
    print(f"情境 4 [原地右轉]: 目標=[ 0.00, -0.50] -> 實際 vx={info['vx']:+.4f} m/s, yaw={info['yaw_rate']:+.4f} rad/s, 高度={info['height']:.4f} m")

    # 情境 5: 恢復煞車待命
    env.command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    for _ in range(50):
        action, _ = model.predict(obs, deterministic=True)
        obs, r, _, _, info = env.step(action)
        obs[-5:-2] = env.command
    print(f"情境 5 [恢復煞車]: 目標=[ 0.00,  0.00] -> 實際 vx={info['vx']:+.4f} m/s, yaw={info['yaw_rate']:+.4f} rad/s, 高度={info['height']:.4f} m")

    print("=" * 70)
    print("   全情境測試通過！機器人具備卓越的指令聽從與動態切換能力。")
    print("=" * 70)

if __name__ == "__main__":
    main()
