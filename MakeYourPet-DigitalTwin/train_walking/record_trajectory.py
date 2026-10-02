"""
Make Your Pet - AI 步態軌跡錄製與 3D 資產導出器 (MuJoCo -> Blender 5.2)
==================================================================
職責：
1. 自動從 models/hexapod.xml 提取 11 個標準化 3D 網格，導出至 blender_assets/meshes/
2. 載入訓練完成的 PPO 步態策略，執行無縫六足行走模擬 (50 FPS)
3. 導出每幀 32 個視覺零件的 6DoF 世界坐標與四元數姿態至 gait_trajectory.json
"""

import os
import sys
import json
import argparse
import numpy as np

# 設置編碼支援
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import mujoco
from stable_baselines3 import PPO
from hexapod_env import HexapodEnv


def export_mesh_objs(model: mujoco.MjModel, output_dir: str):
    """從 MuJoCo 記憶體中導出 11 個已校準並縮放的乾淨 OBJ 網格"""
    os.makedirs(output_dir, exist_ok=True)
    mesh_files = {}

    print(f"📦 [1/3] 正在提取並導出 MuJoCo 幾何網格 (至 {output_dir})...")
    for mesh_id in range(model.nmesh):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        vadr = model.mesh_vertadr[mesh_id]
        vnum = model.mesh_vertnum[mesh_id]
        verts = model.mesh_vert[vadr : vadr + vnum]

        fadr = model.mesh_faceadr[mesh_id]
        fnum = model.mesh_facenum[mesh_id]
        faces = model.mesh_face[fadr : fadr + fnum]

        obj_filename = f"{name}.obj"
        obj_path = os.path.join(output_dir, obj_filename)

        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(f"# MakeYourPet Hexapod Export - {name}\n")
            f.write(f"o {name}\n")
            for v in verts:
                f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            for face in faces:
                # Wavefront OBJ 索引從 1 開始
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")

        mesh_files[name] = obj_filename
        print(f"  ✓ 已導出網格: {name:20s} (頂點: {vnum:5d}, 面數: {fnum:5d})")

    return mesh_files


def record_simulation(
    model_path: str,
    duration_sec: float = 5.0,
    motion_type: str = "combo",
    output_json: str = "gait_trajectory.json",
    assets_dir: str = "blender_assets",
):
    """執行神經網絡策略推論並記錄完整步態姿態軌跡"""
    mesh_dir = os.path.join(assets_dir, "meshes")
    env = HexapodEnv(domain_randomization=False, auto_resample_commands=False)
    m = env.model
    d = env.data

    # 1. 導出乾淨 OBJ 幾何網格
    mesh_files = export_mesh_objs(m, mesh_dir)

    # 2. 解析所有需要動畫驅動的視覺幾何體 (vis_*)
    visual_geoms = []
    for i in range(m.ngeom):
        geom_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        if geom_name and geom_name.startswith("vis_") and "axis" not in geom_name:
            mesh_id = m.geom_dataid[i]
            mesh_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mesh_id) if mesh_id >= 0 else None
            mat_id = m.geom_matid[i]
            mat_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, mat_id) if mat_id >= 0 else None
            
            # 分類材質語義 (供 Blender 自動分配頂級 PBR 材質)
            mat_category = "frame"
            if "cover" in geom_name or "shield" in geom_name or "femur" in geom_name:
                mat_category = "armor"  # 電競金/碳纖維外甲
            elif "tip" in geom_name:
                mat_category = "rubber" # 防滑吸震橡膠
            elif "coxa" in geom_name or "tibia" in geom_name:
                mat_category = "leg_dark" # 陽極氧化黑金屬關節

            visual_geoms.append({
                "geom_id": i,
                "geom_name": geom_name,
                "mesh_name": mesh_name,
                "mat_name": mat_name,
                "mat_category": mat_category
            })

    print(f"\n🤖 [2/3] 載入 AI 模型並開始步態推論 (動作模式: {motion_type}, 時長: {duration_sec}s)...")
    policy = PPO.load(model_path, device="cpu")
    obs, _ = env.reset(seed=42)

    fps = 50  # MuJoCo 控制週期 0.02s
    total_steps = int(duration_sec * fps)
    recorded_frames = []

    quat_buffer = np.zeros(4)

    for step in range(total_steps):
        t = step / fps

        # 動作模式軌跡指令設計
        if motion_type == "forward":
            cmd = np.array([0.25, 0.0, 0.0], dtype=np.float32)
        elif motion_type == "sprint":
            cmd = np.array([0.35, 0.0, 0.0], dtype=np.float32)
        elif motion_type == "turn":
            cmd = np.array([0.0, 0.0, 0.50], dtype=np.float32)
        elif motion_type == "combo":
            # 影視級特寫動態組合：
            # 0~1.5s: 昂首前進
            # 1.5~3.5s: 流暢前進弧形轉彎
            # 3.5~5.0s: 衝刺前進並平穩收步
            if t < 1.5:
                cmd = np.array([0.25, 0.0, 0.0], dtype=np.float32)
            elif t < 3.5:
                cmd = np.array([0.22, 0.0, 0.45], dtype=np.float32)
            else:
                cmd = np.array([0.30, 0.0, -0.25], dtype=np.float32)
        else:
            cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        env.command = cmd.copy()
        obs[-5:-2] = cmd

        # AI 策略推論
        action, _ = policy.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)

        # 記錄軀幹核心焦點位置 (供攝影機追蹤)
        trunk_pos = d.xpos[env.trunk_id].tolist()

        # 記錄 32 個幾何體的 6DoF 姿態
        frame_geom_data = {}
        for g in visual_geoms:
            gid = g["geom_id"]
            pos = d.geom_xpos[gid].tolist()
            # MuJoCo 矩陣轉四元數 [w, x, y, z]
            mujoco.mju_mat2Quat(quat_buffer, d.geom_xmat[gid])
            frame_geom_data[g["geom_name"]] = {
                "pos": [round(v, 6) for v in pos],
                "quat": [round(v, 6) for v in quat_buffer.tolist()]  # [w, x, y, z]
            }

        recorded_frames.append({
            "frame": step + 1,
            "time": round(t, 4),
            "trunk_pos": [round(v, 6) for v in trunk_pos],
            "cmd": [round(v, 3) for v in cmd.tolist()],
            "geoms": frame_geom_data
        })

        if terminated:
            print(f"  ⚠️ 機器人於第 {step+1} 幀失去平衡，提前終止錄製。")
            break

    print(f"  ✓ 模擬錄製完成！共收錄 {len(recorded_frames)} 幀 (50 FPS, 前進距離: {d.xpos[env.trunk_id][0]:.3f}m)")

    # 3. 匯總輸出至 JSON
    print(f"\n💾 [3/3] 正在寫入軌跡數據檔: {output_json}...")
    export_payload = {
        "metadata": {
            "fps": fps,
            "total_frames": len(recorded_frames),
            "duration_sec": len(recorded_frames) / fps,
            "motion_type": motion_type,
            "model_path": model_path,
            "robot_name": "MakeYourPet_Hexapod"
        },
        "mesh_assets": mesh_files,
        "visual_geoms": visual_geoms,
        "frames": recorded_frames
    }

    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(export_payload, f, indent=2)

    size_mb = os.path.getsize(output_json) / (1024 * 1024)
    print(f"🎉 軌跡檔案已生成！大小: {size_mb:.2f} MB")
    print(f"👉 下一步：在 Blender 5.2 執行渲染腳本 blender_cinematic.py 即可產出影視級大片！\n")


def main():
    parser = argparse.ArgumentParser(description="Make Your Pet 步態軌跡錄製工具")
    parser.add_argument("--model", type=str, default="models/best_model/best_model.zip",
                        help="RL 模型路徑 (預設: models/best_model/best_model.zip)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="錄製秒數 (預設: 5.0 秒，即 250 幀)")
    parser.add_argument("--motion", type=str, default="combo", choices=["forward", "sprint", "turn", "combo"],
                        help="步態動作展示模式 (combo / forward / sprint / turn)")
    parser.add_argument("--output", type=str, default="gait_trajectory.json",
                        help="輸出 JSON 軌跡檔名稱")
    args = parser.parse_args()

    # 自動檢查模型路徑
    candidate = args.model
    if not os.path.exists(candidate):
        alt = "models/hexapod_final_policy.zip"
        if os.path.exists(alt):
            candidate = alt
        else:
            raise FileNotFoundError(f"找不到可用模型：{args.model} 或 {alt}")

    record_simulation(
        model_path=candidate,
        duration_sec=args.duration,
        motion_type=args.motion,
        output_json=args.output
    )


if __name__ == "__main__":
    main()
