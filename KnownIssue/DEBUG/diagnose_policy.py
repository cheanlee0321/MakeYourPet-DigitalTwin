import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import numpy as np
from stable_baselines3 import PPO
from hexapod_env import HexapodEnv

model_dir = "models/best_model/best_model.zip" if os.path.exists("models/best_model/best_model.zip") else os.path.join(os.path.dirname(__file__), "..", "models/best_model/best_model.zip")
model = PPO.load(model_dir, device="cpu")
env = HexapodEnv(domain_randomization=False, auto_resample_commands=False)
env.command = np.array([0.25, 0.0, 0.0], dtype=np.float32)
obs, _ = env.reset(seed=42)

tip_ids = [env.model.geom(f"tip_{l}").id for l in ["L1", "L2", "L3", "R1", "R2", "R3"]]

print("Foot tip Z heights over 20 steps:")
for step in range(20):
    action, _ = model.predict(obs, deterministic=True)
    obs, r, term, trunc, info = env.step(action)
    tip_z = [env.data.geom_xpos[tid, 2] for tid in tip_ids]
    n_contacts = sum(1 for z in tip_z if z < 0.012)
    print(f"Step {step:02d}: vx={info['vx']:+.4f}, contacts={n_contacts}, tip_z={[round(z*100, 1) for z in tip_z]}")
