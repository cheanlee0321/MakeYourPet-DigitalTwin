"""
Test Jump Policy & Benchmark Evaluator for Make Your Pet Hexapod
================================================================
對比評估【基準數字 5 行為 (純 FSM 開環)】vs【數字 5 + 殘差強化學習 (智慧閉環)】
在各類地形 (flat, uneven, bumps, slope, platform) 與隨機外力干擾下的跳躍姿態穩定度。

使用範例：
  # 基準開環 vs 閉環對比評估 (終端遙測報表)
  python test_jump_policy.py --model models/jump_best_model/best_model.zip

  # 3D 視覺化單次跳躍展示
  python test_jump_policy.py --model models/jump_best_model/best_model.zip --render --terrain uneven
"""

import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import math
import argparse
import numpy as np
import mujoco
from stable_baselines3 import PPO

try:
    from .hexapod_jump_env import HexapodJumpEnv
except (ImportError, ValueError):
    from hexapod_jump_env import HexapodJumpEnv


def parse_args():
    parser = argparse.ArgumentParser(description="六足機器人立定跳躍殘差策略對比評估工作台")
    parser.add_argument("--model", type=str, default=None,
                        help="欲評估的跳躍策略權重路徑 (.zip)")
    parser.add_argument("--terrain", type=str, default="flat",
                        choices=["flat", "uneven", "bumps", "slope", "platform"],
                        help="評估地貌場景 (預設: flat)")
    parser.add_argument("--terrain-height", type=float, default=0.020,
                        help="地貌最大起伏高度 (m)")
    parser.add_argument("--episodes", type=int, default=5,
                        help="評估回合數 (預設: 5)")
    parser.add_argument("--render", action="store_true",
                        help="開啟 MuJoCo 3D 視窗即時渲染")
    parser.add_argument("--power", type=float, default=1.15,
                        help="基準跳躍爆發力度 (預設: 1.15 對應按鍵 5)")
    return parser.parse_args()


def evaluate_mode(mode_name: str, policy_fn, terrain: str, terrain_h: float, power: float, n_episodes: int, render: bool):
    print(f"\n{'='*75}")
    print(f"  >>> 開始評估：【{mode_name}】 | 地貌: 【{terrain}】 (起伏: ±{terrain_h*100:.1f}cm)")
    print(f"{'='*75}")

    env = HexapodJumpEnv(
        terrain_type=terrain,
        terrain_height=terrain_h,
        jump_power=power,
        domain_randomization=True,
    )

    peak_heights = []
    max_tilts_flight = []
    touchdown_tilts = []
    first_touch_feet_list = []
    belly_hits_list = []
    survival_list = []

    viewer = None
    if render:
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = env.trunk_id
        viewer.cam.distance = 1.10
        viewer.cam.elevation = -18
        viewer.cam.azimuth = 90

    for ep in range(n_episodes):
        obs, info = env.reset(seed=1000 + ep)
        ep_peak_h = 0.0
        ep_max_tilt_flight = 0.0
        ep_touchdown_tilt = None
        ep_first_touch_feet = None
        ep_belly_hits = 0
        flight_started = False

        for step in range(env.max_steps):
            step_start = time.time()
            if policy_fn is not None:
                action = policy_fn(obs)
            else:
                action = np.zeros(18, dtype=np.float32)

            obs, reward, terminated, truncated, info = env.step(action)

            # 遙測數據提取
            curr_h = info["height"]
            if curr_h > ep_peak_h:
                ep_peak_h = curr_h

            quat = env.data.qpos[3:7]
            w, x, y, z = quat
            roll = np.rad2deg(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
            pitch = np.rad2deg(math.asin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
            tilt = np.sqrt(roll**2 + pitch**2)

            fsm_str = info["fsm_state"]
            if "騰空" in fsm_str or "火箭" in fsm_str:
                flight_started = True
                if tilt > ep_max_tilt_flight:
                    ep_max_tilt_flight = tilt

            contacts = env._get_feet_contacts()
            n_contacts = int(np.sum(contacts))

            if flight_started and info["vz"] < 0 and n_contacts >= 1 and ep_touchdown_tilt is None:
                ep_touchdown_tilt = tilt
                ep_first_touch_feet = n_contacts

            if env._check_belly_contact():
                ep_belly_hits += 1

            if render and viewer is not None and viewer.is_running():
                viewer.sync()
                elapsed = time.time() - step_start
                if elapsed < env.dt:
                    time.sleep(env.dt - elapsed)

            if terminated:
                break

        if ep_touchdown_tilt is None:
            ep_touchdown_tilt = tilt
        if ep_first_touch_feet is None:
            ep_first_touch_feet = n_contacts

        peak_heights.append(ep_peak_h)
        max_tilts_flight.append(ep_max_tilt_flight)
        touchdown_tilts.append(ep_touchdown_tilt)
        first_touch_feet_list.append(ep_first_touch_feet)
        belly_hits_list.append(ep_belly_hits)
        survival_list.append(1 if not terminated else 0)

        print(f"  [回合 {ep+1}/{n_episodes}] 最高起跳: {ep_peak_h*100:.1f}cm | 空中最大傾角: {ep_max_tilt_flight:.1f}° | 觸地傾角: {ep_touchdown_tilt:.1f}° | 首拍著地腿數: {ep_first_touch_feet}/6 | 腹部碰撞: {ep_belly_hits}次 | {'✅ 成功穩立' if not terminated else '❌ 側翻跌倒'}")

    if render and viewer is not None:
        viewer.close()

    env.close()

    return {
        "mode": mode_name,
        "mean_peak_h": np.mean(peak_heights) * 100,
        "mean_flight_tilt": np.mean(max_tilts_flight),
        "mean_touchdown_tilt": np.mean(touchdown_tilts),
        "mean_touch_feet": np.mean(first_touch_feet_list),
        "total_belly_hits": np.sum(belly_hits_list),
        "success_rate": np.mean(survival_list) * 100,
    }


def main():
    args = parse_args()

    # 1. 基準開環跳躍 (純數字 5 FSM 控制器，無殘差)
    res_baseline = evaluate_mode(
        mode_name="基準數字 5 行為 (純 FSM 開環)",
        policy_fn=None,
        terrain=args.terrain,
        terrain_h=args.terrain_height,
        power=args.power,
        n_episodes=args.episodes,
        render=args.render,
    )

    # 2. 閉環殘差策略 (若有載入模型權重)
    res_residual = None
    model_path = args.model
    if model_path and not os.path.exists(model_path):
        candidate = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", model_path))
        if os.path.exists(candidate):
            model_path = candidate

    if not model_path:
        for default_cand in [
            os.path.join("models", "jump_best_model", "best_model.zip"),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "jump_best_model", "best_model.zip")),
        ]:
            if os.path.exists(default_cand):
                model_path = default_cand
                break

    if model_path and os.path.exists(model_path):
        print(f"\n正在載入強化學習策略模型: {model_path}")
        model = PPO.load(model_path, device="cpu")

        def residual_policy(obs):
            action, _ = model.predict(obs, deterministic=True)
            return action

        res_residual = evaluate_mode(
            mode_name="數字 5 + 殘差 RL 策略 (智慧閉環)",
            policy_fn=residual_policy,
            terrain=args.terrain,
            terrain_h=args.terrain_height,
            power=args.power,
            n_episodes=args.episodes,
            render=args.render,
        )
    else:
        print(f"\n[提示] 未指定或找不到模型權重檔案 ({args.model})，僅展示基準數字 5 開環評估結果。")

    # 3. 輸出對比綜合報告表
    print("\n" + "=" * 78)
    print("               🏁 六足機器人跳躍姿態穩定性綜合對比報表")
    print("=" * 78)
    print(f"{'指標項目':<22} | {'基準數字 5 開環':<18} | {'數字 5 + 殘差 RL':<18} | {'改善提升':<10}")
    print("-" * 78)

    if res_residual:
        dh = res_residual["mean_peak_h"] - res_baseline["mean_peak_h"]
        dtilt_f = res_baseline["mean_flight_tilt"] - res_residual["mean_flight_tilt"]
        dtilt_td = res_baseline["mean_touchdown_tilt"] - res_residual["mean_touchdown_tilt"]
        dsr = res_residual["success_rate"] - res_baseline["success_rate"]

        print(f"{'平均起跳高度':<22} | {res_baseline['mean_peak_h']:>14.1f} cm | {res_residual['mean_peak_h']:>14.1f} cm | {dh:>+8.1f} cm")
        print(f"{'空中最大傾角 (Roll/Pitch)':<22} | {res_baseline['mean_flight_tilt']:>14.2f}° | {res_residual['mean_flight_tilt']:>14.2f}° | {dtilt_f:>+8.2f}°")
        print(f"{'著地瞬間傾角':<22} | {res_baseline['mean_touchdown_tilt']:>14.2f}° | {res_residual['mean_touchdown_tilt']:>14.2f}° | {dtilt_td:>+8.2f}°")
        print(f"{'首拍著地腿數 (1~6)':<22} | {res_baseline['mean_touch_feet']:>14.1f} 腿 | {res_residual['mean_touch_feet']:>14.1f} 腿 | {res_residual['mean_touch_feet']-res_baseline['mean_touch_feet']:>+8.1f} 腿")
        print(f"{'腹部地面碰撞次數':<22} | {res_baseline['total_belly_hits']:>14d} 次 | {res_residual['total_belly_hits']:>14d} 次 | {res_baseline['total_belly_hits']-res_residual['total_belly_hits']:>+8d} 次")
        print(f"{'跳躍成功存活率':<22} | {res_baseline['success_rate']:>14.1f} % | {res_residual['success_rate']:>14.1f} % | {dsr:>+8.1f} %")
    else:
        print(f"{'平均起跳高度':<22} | {res_baseline['mean_peak_h']:>14.1f} cm | {'--':>18} | {'--':>10}")
        print(f"{'空中最大傾角':<22} | {res_baseline['mean_flight_tilt']:>14.2f}° | {'--':>18} | {'--':>10}")
        print(f"{'著地瞬間傾角':<22} | {res_baseline['mean_touchdown_tilt']:>14.2f}° | {'--':>18} | {'--':>10}")
        print(f"{'首拍著地腿數':<22} | {res_baseline['mean_touch_feet']:>14.1f} 腿 | {'--':>18} | {'--':>10}")
        print(f"{'腹部地面碰撞次數':<22} | {res_baseline['total_belly_hits']:>14d} 次 | {'--':>18} | {'--':>10}")
        print(f"{'跳躍成功存活率':<22} | {res_baseline['success_rate']:>14.1f} % | {'--':>18} | {'--':>10}")

    print("=" * 78 + "\n")


if __name__ == "__main__":
    main()
