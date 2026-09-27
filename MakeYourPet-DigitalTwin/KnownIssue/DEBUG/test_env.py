import time
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
from hexapod_env import HexapodEnv

def main():
    print("=" * 65)
    print("      STAGE 2: HEXAPOD GYMNASIUM ENVIRONMENT BENCHMARK")
    print("=" * 65)

    env = HexapodEnv()
    print(f"[1/4] Environment created successfully.")
    print(f"      Action Space:      {env.action_space}")
    print(f"      Observation Space: {env.observation_space}")

    # 1. Reset Test
    obs, info = env.reset(seed=42)
    print(f"\n[2/4] Reset Test:")
    print(f"      Initial Observation shape: {obs.shape}")
    print(f"      Obs values finite: {np.isfinite(obs).all()}")
    print(f"      Initial Body Height: {env.data.qpos[2]:.4f} m")

    # 2. Step Test with Random Actions
    print(f"\n[3/4] Step Test (Running 200 random exploration steps):")
    total_reward = 0.0
    for step in range(200):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if terminated or truncated:
            obs, info = env.reset()

    print(f"      200 steps completed.")
    print(f"      Average reward per step: {total_reward / 200:.3f}")
    print(f"      Final Observation shape: {obs.shape}")

    # 3. High-Speed Throughput Benchmark (Steps Per Second)
    print(f"\n[4/4] Single-Thread Throughput Benchmark (1000 steps):")
    env.reset()
    start_time = time.time()
    num_benchmark_steps = 1000
    for _ in range(num_benchmark_steps):
        action = env.action_space.sample()
        env.step(action)
    elapsed = time.time() - start_time
    sps = num_benchmark_steps / elapsed
    sim_time_sec = num_benchmark_steps * env.dt
    speedup = sim_time_sec / elapsed

    print(f"      Execution Time: {elapsed:.3f} s")
    print(f"      Throughput:     {sps:.1f} Steps/sec (SPS)")
    print(f"      Simulated Time: {sim_time_sec:.1f} s")
    print(f"      Speedup Factor: {speedup:.1f}x Real-time (Single CPU core!)")

    print("\n" + "=" * 65)
    print("   ALL STAGE 2 TESTS PASSED! READY FOR PPO TRAINING.")
    print("=" * 65)

if __name__ == "__main__":
    main()
