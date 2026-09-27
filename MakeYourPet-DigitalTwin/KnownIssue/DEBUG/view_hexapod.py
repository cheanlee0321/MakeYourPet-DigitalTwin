import time
import math
import mujoco
import mujoco.viewer
import os

def main():
    model_path = os.path.join(os.path.dirname(__file__), "models", "hexapod.xml")
    if not os.path.exists(model_path):
        model_path = os.path.join(os.path.dirname(__file__), "..", "models", "hexapod.xml")
    if not os.path.exists(model_path):
        print(f"找不到模型檔案: {model_path}")
        return

    print("=" * 65)
    print("   Make Your Pet: 18 自由度經典「Z 字高聳膝關節折線」數位孿生")
    print("=" * 65)
    print("幾何結構修正點：")
    print(" • Coxa (基座)：水平向外延伸 43mm。")
    print(" • Femur (大腿)：【向上斜挑 +46mm】，把膝關節頂到半空中 (形成 Z 字折線上半段)！")
    print(" • Tibia (小腿)：從高聳的膝蓋【向下斜插 -86mm 觸地】(形成 Z 字折線下半段)！")
    print(" • 膝關節高度 (約 85mm) 高於機身本體 (約 50mm)，完美重現原作者螳螂式霸氣站姿！")
    print("")
    print("互動操作說明：")
    print(" • 滑鼠左鍵拖曳：360度旋轉視角")
    print(" • 滑鼠右鍵拖曳：平移視角")
    print(" • 滾輪：縮放鏡頭")
    print(" • 按住 Ctrl + 滑鼠右鍵點擊機身拖曳：可以直接施加外力「推撞」機器人！")
    print(" • 空白鍵 (Space)：暫停 / 恢復物理模擬")
    print("=" * 65)

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 18 軸在原生 Z 字形姿態下的預設目標為 0 度 (正中央)
    for i in range(18):
        data.ctrl[i] = 0.0
        data.qpos[7 + i] = 0.0

    # 初始機身高度設為 0.055m，6 隻腳掌穩穩踩地
    data.qpos[2] = 0.055

    fps = 60.0
    frame_time = 1.0 / fps
    steps_per_frame = max(1, int(round(frame_time / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            for _ in range(steps_per_frame):
                t = data.time

                # 呼吸律動展示：讓機身輕微起伏 ±15mm (深蹲與站起)，展示 18 軸大負載動態支撐
                heave_deg = 8.0 * math.sin(2.0 * math.pi * 0.4 * t)

                for leg_idx in range(6):
                    c_idx = leg_idx * 3 + 0
                    f_idx = leg_idx * 3 + 1
                    t_idx = leg_idx * 3 + 2

                    # Coxa 保持中位
                    data.ctrl[c_idx] = 0.0
                    # Femur 與 Tibia 協同伸縮 (大腿上抬 / 小腿下壓)
                    data.ctrl[f_idx] = math.radians(heave_deg)
                    data.ctrl[t_idx] = math.radians(-heave_deg * 1.2)

                # 推進物理步進
                mujoco.mj_step(model, data)

            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    main()
