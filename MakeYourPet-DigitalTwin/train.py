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
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, CallbackList
from hexapod_env import HexapodEnv

def make_env(rank: int, seed: int = 0, domain_rand: bool = True, resample_cmd: bool = True,
             terrain_type: str = "flat", terrain_height: float = 0.015):
    """建立包含獨立隨機種子與設定的環境工廠函數"""
    def _init():
        env = HexapodEnv(
            domain_randomization=domain_rand,
            auto_resample_commands=resample_cmd,
            terrain_type=terrain_type,
            terrain_height=terrain_height
        )
        env.reset(seed=seed + rank)
        return env
    return _init

def parse_args():
    parser = argparse.ArgumentParser(description="Make Your Pet 六足機器人 PPO 步態訓練主程式 (階段 3: 全自由度指令追蹤)")
    parser.add_argument("--timesteps", type=int, default=1_000_000, help="總訓練步數 (預設: 1,000,000)")
    parser.add_argument("--num-envs", type=int, default=12, help="平行物理模擬進程數量 (預設: 12)")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda", "auto"],
                        help="運算裝置 (預設: cpu，小尺寸 MLP 於多核心 CPU 上吞吐量最高)")
    parser.add_argument("--lr", type=float, default=3e-4, help="學習率 (預設: 3e-4)")
    parser.add_argument("--n-steps", type=int, default=1024, help="每個環境每次採樣步數 (預設: 1024)")
    parser.add_argument("--batch-size", type=int, default=256, help="GPU/CPU 梯度更新批次大小 (預設: 256)")
    parser.add_argument("--n-epochs", type=int, default=10, help="每批採樣更新次數 (預設: 10)")
    parser.add_argument("--ent-coef", type=float, default=0.008, help="熵獎勵係數 (預設: 0.008，鼓勵探索多向指令與停步)")
    parser.add_argument("--eval-freq", type=int, default=25_000, help="策略評估頻率 (預設: 每 25,000 步評估一次)")
    parser.add_argument("--save-freq", type=int, default=100_000, help="檢查點保存頻率 (預設: 每 100,000 步保存一次)")
    parser.add_argument("--no-domain-rand", action="store_true", help="關閉領域隨機化 (Domain Randomization)")
    parser.add_argument("--resume", type=str, default=None, help="接續訓練之既有模型權重 (.zip 檔案路徑)")
    parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "bumps", "blocks", "park", "rough", "slope"],
                        help="訓練地貌類型 (flat: 經典平地, blocks: 階梯石柱區塊陣, park: 4大主題複合越野公園, bumps: 連續波浪, rough: 粗糙碎石, slope: 傾斜斜坡，預設: flat)")
    parser.add_argument("--terrain-height", type=float, default=0.025,
                        help="地形最大起伏高度 (公尺，預設: 0.025 即 2.5cm，支援至 0.06m)")
    return parser.parse_args()

def main():
    args = parse_args()
    use_domain_rand = not args.no_domain_rand

    print("=" * 70)
    print("      MAKE YOUR PET 數位孿生 —— 階段 3：AI 強化學習步態訓練 (PPO)")
    print("=" * 70)
    print(f"[配置] 總訓練步數:      {args.timesteps:,} 步")
    print(f"[配置] 平行環境數量:    {args.num_envs} 個進程 (SubprocVecEnv)")
    print(f"[配置] 運算裝置:        {args.device.upper()}")
    print(f"[配置] 訓練地貌類型:    {args.terrain} (起伏: ±{args.terrain_height*100:.1f} cm)")
    print(f"[配置] 領域隨機化:      {'啟用 (Domain Randomization ON)' if use_domain_rand else '關閉'}")
    print(f"[配置] 學習率 / 熵係數: {args.lr} / {args.ent_coef}")
    print(f"[配置] 採樣緩衝區:      {args.num_envs} envs x {args.n_steps} steps = {args.num_envs * args.n_steps:,} 步/輪")

    # 建立輸出目錄
    log_dir = os.path.join(".", "tensorboard_logs")
    chkpt_dir = os.path.join(".", "checkpoints")
    best_model_dir = os.path.join(".", "models", "best_model")
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(chkpt_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)

    # 1. 建立並行訓練環境
    print("\n[1/4] 正在啟動多進程物理模擬環境...")
    env = SubprocVecEnv([
        make_env(i, seed=42, domain_rand=use_domain_rand,
                 terrain_type=args.terrain, terrain_height=args.terrain_height)
        for i in range(args.num_envs)
    ])
    env = VecMonitor(env)

    # 2. 建立獨立評估環境 (確定性評估，關閉隨機外力)
    eval_env = SubprocVecEnv([
        make_env(999, seed=1234, domain_rand=False,
                 terrain_type=args.terrain, terrain_height=args.terrain_height)
        for _ in range(1)
    ])
    eval_env = VecMonitor(eval_env)

    # 3. 設置回調機制 (評估回調與定時儲存)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=best_model_dir,
        log_path=log_dir,
        eval_freq=max(args.eval_freq // args.num_envs, 1),
        n_eval_episodes=5,
        deterministic=True,
        render=False,
        verbose=1
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.save_freq // args.num_envs, 1),
        save_path=chkpt_dir,
        name_prefix=f"hexapod_ppo_{args.terrain}_h{int(round(args.terrain_height*100))}cm" if args.terrain != "flat" else "hexapod_ppo"
    )

    callbacks = CallbackList([eval_callback, checkpoint_callback])

    # 4. 初始化或載入 PPO 模型
    print("[2/4] 初始化 PPO 神經網路策略架構...")
    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256], vf=[256, 256]),
        activation_fn=torch.nn.Tanh
    )

    if args.resume and os.path.exists(args.resume):
        print(f"      [載入權重] 正在從 {args.resume} 載入既有策略...")
        model = PPO.load(
            args.resume,
            env=env,
            device=args.device,
            tensorboard_log=log_dir,
            custom_objects={"learning_rate": args.lr, "ent_coef": args.ent_coef}
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
            tensorboard_log=log_dir
        )

    # 5. 開始學習
    print("\n[3/4] 啟動強化學習訓練迴圈！")
    print(f"      請開啟另一個終端機執行：tensorboard --logdir={log_dir}")
    print(f"      瀏覽器開啟 http://localhost:6006 即時查看訓練曲線與步態收斂回報。\n")

    start_time = time.time()
    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n[中斷] 偵測到使用者手動中斷 (Ctrl+C)，正在緊急儲存當前訓練權重...")
    finally:
        elapsed = time.time() - start_time
        sps = args.timesteps / max(elapsed, 1.0)
        print("\n" + "=" * 70)
        print(f"[4/4] 訓練結束！總耗時: {elapsed:.2f} 秒 ({elapsed / 60.0:.2f} 分鐘)")
        print(f"      平均採樣速率: {sps:.1f} Steps/sec (SPS)")

        # 儲存最終模型
        final_model_path = os.path.join(".", "models", "hexapod_final_policy")
        model.save(final_model_path)
        print(f"      最終模型權重已儲存至: {final_model_path}.zip")
        print(f"      最佳評估模型位於:     {os.path.join(best_model_dir, 'best_model.zip')}")

        # 清理並釋放多進程資源
        env.close()
        eval_env.close()
        print("=" * 70)

if __name__ == "__main__":
    main()
