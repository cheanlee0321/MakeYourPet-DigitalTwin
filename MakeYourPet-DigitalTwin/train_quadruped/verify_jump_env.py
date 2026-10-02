import sys
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")
import numpy as np
try:
    from .hexapod_jump_env import HexapodJumpEnv
except (ImportError, ValueError):
    from hexapod_jump_env import HexapodJumpEnv

def test_jump_env():
    print(">>> 正在初始化 HexapodJumpEnv (平地地形)...")
    env = HexapodJumpEnv(terrain_type="flat", domain_randomization=True)
    obs, info = env.reset()

    print(f"Observation Shape: {obs.shape} (預期 78)")
    assert obs.shape == (78,), f"觀測維度不符: {obs.shape}"

    print(f"Action Space Shape: {env.action_space.shape} (預期 18)")
    assert env.action_space.shape == (18,), f"動作空間維度不符: {env.action_space.shape}"

    print("\n>>> 開始步進測試 (執行 90 個控制步，涵蓋待命、起跳、滯空、著地)...")
    total_reward = 0.0
    states_seen = set()

    for step in range(env.max_steps):
        # 測試微量殘差動作
        action = np.random.uniform(-0.1, 0.1, size=18).astype(np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        states_seen.add(info["fsm_state"])

        if step % 15 == 0 or terminated or truncated:
            print(f"  Step {step:02d} | 階段: {info['fsm_state']:<12} | 高度: {info['height']:.3f}m | vz: {info['vz']:+.2f}m/s | 步獎勵: {reward:+.2f}")

        if terminated:
            print(f"  [提前終止] 第 {step} 步終止！")
            break

    print(f"\n經歷階段: {states_seen}")
    print(f"最高跳躍高度: {info['max_height']:.3f}m")
    print(f"回合總獎勵: {total_reward:.2f}")
    print(">>> HexapodJumpEnv 基本功能驗證 100% 通過！\n")

    for t_type in ["uneven", "bumps", "slope", "platform"]:
        print(f">>> 測試地貌場景: 【{t_type}】...")
        env_t = HexapodJumpEnv(terrain_type=t_type, terrain_height=0.020, domain_randomization=True)
        obs, _ = env_t.reset()
        assert obs.shape == (78,)
        for step in range(35):
            obs, r, term, trunc, info = env_t.step(np.zeros(18, dtype=np.float32))
            if term:
                break
        print(f"  地貌 【{t_type}】 運行正常，起跳頂峰高度: {info['max_height']:.3f}m")

if __name__ == "__main__":
    test_jump_env()
