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

    print("=" * 70)
    print("   Make Your Pet: 18 自由度高擬真數位孿生「全機互動調試控制台」")
    print("=" * 70)
    print("🧭【3D 空間坐標軸 (XYZ Coordinate Axes) 辨識指南】：")
    print("   機身正上方與世界原點已預設開啟 3D 坐標軸指示：")
    print("   🔴 紅色 (Red) 箭頭  = +X 軸：機器人正前方 (Heading / 前進方向，介於 L1 與 R1 之間)")
    print("   🟢 綠色 (Green) 箭頭 = +Y 軸：機器人左側 (Left Side，L1 / L2 / L3 腿側)")
    print("   🔵 藍色 (Blue) 箭頭  = +Z 軸：垂直天頂 (Upward，朝向天空)")
    print("   ─────────────────────────────────────────────────────────")
    print("   負方向參考：")
    print("   • -X 軸 (正後方 Rear)：介於 L3 與 R3 之間")
    print("   • -Y 軸 (右側 Right)：R1 / R2 / R3 腿側")
    print("   • 腿部編號位置：")
    print("     左側 (Green +Y)：L1 (左前), L2 (左中), L3 (左後)")
    print("     右側 (-Y 側)   ：R1 (右前), R2 (右中), R3 (右後)")
    print("")
    print("視覺與坐標軸選單控制：")
    print(" • 視窗左側選單「Rendering」->「Frame」：")
    print("   可切換 [World] (世界原點軸)、[Body] (全機各關節局部軸)、[None] (關閉)")
    print(" • 視窗左側選單「Rendering」->「Label」：")
    print("   可開啟 [Site] (顯示 L1~R3 腿部名稱文字標籤) 或 [Body] (顯示各連桿名稱)")
    print("")
    print("快捷鍵與外力推撞指南：")
    print(" • 空白鍵 (Space)：暫停 / 繼續物理模擬")
    print(" • 鍵盤 'F1'：隨時呼叫 MuJoCo 官方內建鍵盤與滑鼠操作全手冊")
    print(" • 鍵盤 '0'：切換視覺網格顯示")
    print(" • 鍵盤 '1' / '3'：切換物理碰撞膠囊體顯示 (透視內部骨架！)")
    print(" • 鍵盤 'C'：顯示接觸點與法向支撐力 (Contact Points)")
    print(" • 💥【如何用滑鼠推撞 / 施加外力拉扯機器人】：")
    print("   步驟 1：先對著機器人機身或腿部【滑鼠左鍵連點兩下 (Double Click)】進行選取！")
    print("          （選中後該零件會出現選取標記，視窗右上/左側會顯示 Selected Body）")
    print("   步驟 2：按住鍵盤【Ctrl】+【滑鼠右鍵】按住拖曳 ── 即可直接施加推力/拉力 (Force)！")
    print("   步驟 3 (選用)：按住【Ctrl + Shift】+【滑鼠右鍵】拖曳 ── 在水平面 (X-Y) 平移推拉！")
    print("   步驟 4 (選用)：按住【Ctrl】+【滑鼠左鍵】拖曳 ── 施加扭轉力矩 (Torque)！")
    print(" • 一般視角操作：滑鼠左鍵拖曳旋轉視角 | 右鍵平移視角 | 滾輪縮放")
    print("=" * 70)

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 初始所有舵機目標設為 0 (標準 Z 字形站立姿態)
    for i in range(18):
        data.ctrl[i] = 0.0
        data.qpos[7 + i] = 0.0

    # 初始機身高度
    data.qpos[2] = 0.055

    # 預設開啟 3D 坐標軸 (World Frame) 與 站點標籤 (Site Label)
    orig_opt_init = mujoco.MjvOption.__init__
    def custom_opt_init(self, *args, **kwargs):
        orig_opt_init(self, *args, **kwargs)
        self.frame = mujoco.mjtFrame.mjFRAME_WORLD
        self.label = mujoco.mjtLabel.mjLABEL_SITE
    mujoco.MjvOption.__init__ = custom_opt_init

    # 啟動官方全功能互動式調試視窗 (含 18 軸 Slider 控制面版)
    mujoco.viewer.launch(model, data)

if __name__ == "__main__":
    main()
