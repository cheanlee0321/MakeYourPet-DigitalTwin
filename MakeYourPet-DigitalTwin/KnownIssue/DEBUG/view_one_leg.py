import time
import math
import mujoco
import mujoco.viewer
import os

def main():
    model_path = os.path.join(os.path.dirname(__file__), "models", "one_leg.xml")
    if not os.path.exists(model_path):
        model_path = os.path.join(os.path.dirname(__file__), "..", "models", "one_leg.xml")
    if not os.path.exists(model_path):
        print(f"找不到模型檔案: {model_path}")
        return

    print("=" * 60)
    print("      Make Your Pet: 單腿 3 自由度數位孿生平穩動態預覽")
    print("=" * 60)
    print("已徹底根除發瘋震盪核心關鍵：")
    print(" 1. 修正單位陷阱：MuJoCo Python API 的 ctrl 需傳入「弧度 (radians)」。")
    print(" 2. 排除相鄰關節幾何內扣碰撞 (Contact Exclusion)。")
    print(" 3. 填入真實舵機重量 (60g) 與扭矩上限 (2.5 N·m)。")
    print(" 4. 啟用 implicitfast 隱式求解器，徹底告別數值爆炸！")
    print("=" * 60)

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    fps = 60.0
    frame_time = 1.0 / fps
    steps_per_frame = max(1, int(round(frame_time / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # 每個渲染畫面推進對應微步 (2ms x 8 = 16ms)
            for _ in range(steps_per_frame):
                t = data.time

                # 關鍵修正：將期望角度（度）精確轉換為弧度 (radians)
                # 關節 1 (Coxa)：慢速左右水平擺動 ±20 度 (頻率 0.4 Hz)
                coxa_deg = 20.0 * math.sin(2.0 * math.pi * 0.4 * t)
                data.ctrl[0] = math.radians(coxa_deg)

                # 關節 2 (Femur)：慢速抬起與放下 10 ~ 30 度
                femur_deg = 20.0 + 15.0 * math.sin(2.0 * math.pi * 0.4 * t)
                data.ctrl[1] = math.radians(femur_deg)

                # 關節 3 (Tibia)：足部伸展與彎曲 -35 ~ -65 度
                tibia_deg = -50.0 + 15.0 * math.cos(2.0 * math.pi * 0.4 * t)
                data.ctrl[2] = math.radians(tibia_deg)

                # 物理步進
                mujoco.mj_step(model, data)

            # 更新畫面
            viewer.sync()

            # 60 FPS 對齊休眠
            elapsed = time.time() - step_start
            sleep_time = frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    main()
