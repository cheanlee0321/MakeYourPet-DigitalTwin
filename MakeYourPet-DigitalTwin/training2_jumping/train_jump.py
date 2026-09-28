"""
Train Jump: 18 自由度六足機器人立定跳躍殘差強化學習訓練腳本 (PPO)
            18-DoF Hexapod Standing Jump Residual RL Training Script (PPO)
==========================================================================
基於數字 5 行為 (JumpController 標稱軌跡)，使用 PPO 訓練 18 軸殘差補償神經網路。
Trains an 18-axis residual compensation neural network using PPO on top of Key 5 behavior (JumpController nominal trajectory).

特色 / Features:
- 平行加速：支援多進程 SubprocVecEnv (預設 8~12 平行物理環境) / Parallel acceleration: supports multi-process SubprocVecEnv (default 8-12 parallel environments)
- 專屬評估：定期執行確定性評估，追蹤起跳高度、空中翻滾抑制率與落地平穩度 / Dedicated evaluation: periodic deterministic evaluation tracking jump height, roll suppression, and landing stability
- 自動保存：自動儲存 Best Model 與階段性 Checkpoint / Auto saving: automatically saves Best Model and periodic checkpoints
- 支援地形模式：flat (平地), uneven (起伏), bumps (波浪), slope (微坡), platform (台階) / Supported terrains: flat, uneven, bumps, slope, platform
"""

import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import time
import argparse
import numpy as np
import torch
import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, CallbackList
try:
    from .hexapod_jump_env import HexapodJumpEnv
except (ImportError, ValueError):
    from hexapod_jump_env import HexapodJumpEnv


def make_jump_env(
    rank: int,
    seed: int = 0,
    domain_rand: bool = True,
    terrain_type: str = "flat",
    terrain_height: float = 0.015,
    jump_power: float = 1.15,
    residual_scale: float = 0.15,
):
    """建立包含獨立隨機種子與地貌設定的跳躍環境工廠函數 / Jump environment factory function creating envs with independent seeds and terrain settings"""
    def _init():
        env = HexapodJumpEnv(
            domain_randomization=domain_rand,
            terrain_type=terrain_type,
            terrain_height=terrain_height,
            jump_power=jump_power,
            residual_scale=residual_scale,
        )
        env.reset(seed=seed + rank)
        return env
    return _init


def parse_args():
    parser = argparse.ArgumentParser(description="Make Your Pet 六足機器人 PPO 跳躍殘差訓練主程式")
    parser.add_argument("--timesteps", type=int, default=300_000, help="總訓練步數 (預設: 300,000 步，約 3,300 次完整跳躍回合)")
    parser.add_argument("--num-envs", type=int, default=8, help="平行物理模擬進程數量 (預設: 8)")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda", "auto"],
                        help="運算裝置 (預設: cpu，小尺寸 MLP 於多核心 CPU 上吞吐量最高)")
    parser.add_argument("--lr", type=float, default=3e-4, help="學習率 (預設: 3e-4)")
    parser.add_argument("--n-steps", type=int, default=128, help="每個環境每次採樣步數 (預設: 128，涵蓋 >1 個跳躍回合)")
    parser.add_argument("--batch-size", type=int, default=128, help="梯度更新批次大小 (預設: 128)")
    parser.add_argument("--n-epochs", type=int, default=10, help="每批採樣更新次數 (預設: 10)")
    parser.add_argument("--ent-coef", type=float, default=0.005, help="熵獎勵係數 (預設: 0.005)")
    parser.add_argument("--eval-freq", type=int, default=10_000, help="策略評估頻率 (預設: 每 10,000 步評估一次)")
    parser.add_argument("--save-freq", type=int, default=50_000, help="檢查點保存頻率 (預設: 每 50,000 步保存一次)")
    parser.add_argument("--no-domain-rand", action="store_true", help="關閉領域隨機化 (Domain Randomization)")
    parser.add_argument("--resume", type=str, default=None, help="接續訓練之既有模型權重 (.zip 檔案路徑)")
    parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "uneven", "bumps", "slope", "platform"],
                        help="訓練地貌類型 (flat: 經典平地, uneven: 隨機微高差, bumps: 連續波浪, slope: 傾斜坡, platform: 階梯台，預設: flat)")
    parser.add_argument("--terrain-height", type=float, default=0.015,
                        help="地形最大起伏高度 (公尺，預設: 0.015 即 1.5cm)")
    parser.add_argument("--power", type=float, default=1.15,
                        help="基準跳躍爆發力度 (預設: 1.15 對應數字 5 鍵 50cm 爆發跳躍)")
    parser.add_argument("--residual-scale", type=float, default=0.15,
                        help="殘差關節縮放係數 (rad，預設: 0.15 約 +/-8.6 度)")
    return parser.parse_args()


def main():
    args = parse_args()
    use_domain_rand = not args.no_domain_rand

    print("=" * 72)
    print("      MAKE YOUR PET 數位孿生 —— 立定跳躍殘差強化學習訓練 (PPO)")
    print("=" * 72)
    print(f"[配置] 總訓練步數:      {args.timesteps:,} 步")
    print(f"[配置] 平行環境數量:    {args.num_envs} 個進程 (SubprocVecEnv)")
    print(f"[配置] 運算裝置:        {args.device.upper()}")
    print(f"[配置] 訓練地貌類型:    {args.terrain} (起伏: ±{args.terrain_height*100:.1f} cm)")
    print(f"[配置] 基準跳躍行為:    數字 5 行為 (power = {args.power})")
    print(f"[配置] 殘差調整尺度:    ±{args.residual_scale:.2f} rad (±{np.rad2deg(args.residual_scale):.1f}°)")
    print(f"[配置] 領域隨機化:      {'啟用 (Domain Randomization ON)' if use_domain_rand else '關閉'}")
    print(f"[配置] 學習率 / 熵係數: {args.lr} / {args.ent_coef}")
    print(f"[配置] 採樣緩衝區:      {args.num_envs} envs x {args.n_steps} steps = {args.num_envs * args.n_steps:,} 步/輪")

    # 自動定位專案根目錄 / Automatically locate project root directory
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if not os.path.exists(os.path.join(project_root, "models")):
        project_root = os.getcwd()

    log_dir = os.path.join(project_root, "tensorboard_logs", "jump_ppo")
    chkpt_dir = os.path.join(project_root, "checkpoints", "jump")
    best_model_dir = os.path.join(project_root, "models", "jump_best_model")
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(chkpt_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)

    # 1. 建立並行訓練環境 / 1. Build parallel training environments
    print("\n[1/4] 正在啟動多進程跳躍模擬環境...")
    env = SubprocVecEnv([
        make_jump_env(
            i,
            seed=100,
            domain_rand=use_domain_rand,
            terrain_type=args.terrain,
            terrain_height=args.terrain_height,
            jump_power=args.power,
            residual_scale=args.residual_scale,
        )
        for i in range(args.num_envs)
    ])
    env = VecMonitor(env)

    # 2. 建立獨立評估環境 (關閉領域隨機化，測試策略基準泛化度) / 2. Build independent evaluation environment (domain randomization off for benchmark generalization)
    eval_env = SubprocVecEnv([
        make_jump_env(
            999,
            seed=2026,
            domain_rand=False,
            terrain_type=args.terrain,
            terrain_height=args.terrain_height,
            jump_power=args.power,
            residual_scale=args.residual_scale,
        )
        for _ in range(1)
    ])
    eval_env = VecMonitor(eval_env)

    # 3. 設置回調機制 / 3. Set up callback mechanisms
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=best_model_dir,
        log_path=log_dir,
        eval_freq=max(args.eval_freq // args.num_envs, 1),
        n_eval_episodes=5,
        deterministic=True,
        render=False,
        verbose=1,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.save_freq // args.num_envs, 1),
        save_path=chkpt_dir,
        name_prefix=f"jump_ppo_{args.terrain}",
    )

    callbacks = CallbackList([eval_callback, checkpoint_callback])

    # 4. 初始化神經網路架構 / 4. Initialize neural network architecture
    print("[2/4] 初始化 PPO 神經網路策略架構...")
    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256], vf=[256, 256]),
        activation_fn=torch.nn.Tanh,
    )

    if args.resume and os.path.exists(args.resume):
        print(f"      [載入權重] 正在從 {args.resume} 載入既有跳躍策略...")
        model = PPO.load(
            args.resume,
            env=env,
            device=args.device,
            tensorboard_log=log_dir,
            custom_objects={"learning_rate": args.lr, "ent_coef": args.ent_coef},
        )
    else:
        model = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=args.lr,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=args.ent_coef,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=policy_kwargs,
            verbose=1,
            device=args.device,
            tensorboard_log=log_dir,
        )

    # 5. 開始訓練迴圈 / 5. Start training loop
    print("\n[3/4] 啟動強化學習訓練迴圈！")
    print(f"      可開啟另一個終端機執行：tensorboard --logdir={log_dir}")
    print(f"      瀏覽器開啟 http://localhost:6006 實時查看跳躍姿態穩定度與累積回報曲線。\n")

    start_time = time.time()
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            progress_bar=False,
        )
    except KeyboardInterrupt:
        print("\n[中斷] 偵測到使用者手動中斷訓練 (Ctrl+C)，正在緊急儲存目前權重...")

    elapsed_time = time.time() - start_time
    print(f"\n[4/4] 訓練完成！總耗時: {elapsed_time:.1f} 秒 ({elapsed_time/60:.2f} 分鐘)")

    # 儲存最終模型 / Save final model
    final_model_path = os.path.join(project_root, "models", "jump_final_policy.zip")
    model.save(final_model_path)
    print(f"      [已儲存] 最終跳躍策略權重: {final_model_path}")
    print(f"      [已儲存] 歷史最佳跳躍權重: {os.path.join(best_model_dir, 'best_model.zip')}")
    print("=" * 72)

    env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
