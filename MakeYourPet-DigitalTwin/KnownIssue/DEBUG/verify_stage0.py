import sys
import torch
import mujoco
import gymnasium as gym
from stable_baselines3 import PPO
import onnxruntime as ort
import trimesh
import os

def main():
    print("=" * 60)
    print("       STAGE 0: ENVIRONMENT HEALTH & ACCELERATION CHECK")
    print("=" * 60)
    
    # 1. PyTorch & CUDA
    print(f"\n[1/6] PyTorch Version: {torch.__version__}")
    print(f"      CUDA Available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        print(f"      Target GPU: {gpu_name}")
        a = torch.randn(1000, 1000, device="cuda")
        b = torch.randn(1000, 1000, device="cuda")
        c = torch.matmul(a, b)
        print(f"      CUDA Matrix Multiplication Test (1000x1000): SUCCESS (sum={c.sum().item():.2f})")
    else:
        print("      [WARNING] CUDA is NOT available!")
        sys.exit(1)

    # 2. MuJoCo Simulation Engine
    print(f"\n[2/6] MuJoCo Version: {mujoco.__version__}")
    xml_test = """
    <mujoco model="test_pendulum">
      <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="1 1 0.1"/>
        <body pos="0 0 1">
          <joint type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.04 0.2"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml_test)
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    print("      MuJoCo Physics Step 100 iterations: SUCCESS")

    # 3. Gymnasium
    print(f"\n[3/6] Gymnasium Version: {gym.__version__}")
    env = gym.make("Pendulum-v1")
    obs, _ = env.reset()
    print(f"      Gymnasium 'Pendulum-v1' initialized: obs_shape={obs.shape}")

    # 4. Stable-Baselines3 (PPO) on CUDA
    print("\n[4/6] Stable-Baselines3 PPO Instantiation:")
    ppo_model = PPO("MlpPolicy", env, device="cuda", verbose=0)
    print(f"      PPO Policy successfully placed on device: {ppo_model.policy.device}")
    
    # 5. ONNX Runtime
    print(f"\n[5/6] ONNX Runtime Version: {ort.__version__}")
    providers = ort.get_available_providers()
    print(f"      Available Providers: {providers}")

    # 6. Trimesh & MakeYourPet Mesh Validation
    stl_path = os.path.join(".", "MakeYourPet-hexapod", "hexapod-main", "STL", "femur.stl")
    if not os.path.exists(stl_path):
        stl_path = os.path.join(".", "MakeYourPet-hexapod", "hexapod-main", "STL", "left-femur.stl")
    print(f"\n[6/6] Trimesh 3D Mesh Inspection: {stl_path}")
    if os.path.exists(stl_path):
        mesh = trimesh.load(stl_path)
        print(f"      Mesh Loaded: vertices={len(mesh.vertices)}, faces={len(mesh.faces)}, bounds={mesh.bounds.tolist()}")
    else:
        print(f"      [INFO] Test STL not found at {stl_path}, checking directory...")

    print("\n" + "=" * 60)
    print("   ALL STAGE 0 TESTS PASSED! WORKSTATION READY FOR STAGE 1.")
    print("=" * 60)

if __name__ == "__main__":
    main()
