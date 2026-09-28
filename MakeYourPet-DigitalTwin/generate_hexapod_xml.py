import os

def generate_hexapod_xml():
    legs = [
        {"name": "L1", "side": "left",  "pos": "0.0835  0.0630 -0.010", "yaw":  45.0, "label": "左前足 (Left Front)"},
        {"name": "L2", "side": "left",  "pos": "0.0000  0.0815 -0.010", "yaw":  90.0, "label": "左中足 (Left Middle)"},
        {"name": "L3", "side": "left",  "pos": "-0.0835  0.0630 -0.010", "yaw": 135.0, "label": "左後足 (Left Rear)"},
        {"name": "R1", "side": "right", "pos": "0.0835 -0.0630 -0.010", "yaw": -45.0, "label": "右前足 (Right Front)"},
        {"name": "R2", "side": "right", "pos": "0.0000 -0.0815 -0.010", "yaw": -90.0, "label": "右中足 (Right Middle)"},
        {"name": "R3", "side": "right", "pos": "-0.0835 -0.0630 -0.010", "yaw": -135.0, "label": "右後足 (Right Rear)"},
    ]

    xml = []
    xml.append('<mujoco model="makeyourpet_hexapod_high_fidelity">')
    xml.append('  <compiler angle="degree" coordinate="local"/>')
    xml.append('  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>')
    xml.append('')
    xml.append('  <visual>')
    xml.append('    <scale framelength="0.15" framewidth="0.008"/>')
    xml.append('  </visual>')
    xml.append('')
    xml.append('  <!-- 視覺與網格資源載入 / Visual and mesh assets loading -->')
    xml.append('  <asset>')
    xml.append('    <!-- 3D 列印 STL 網格 (單位 mm -> m 縮放 0.001) / 3D printed STL meshes (scale mm -> m by 0.001) -->')
    xml.append('    <mesh name="mesh_frame" file="../MakeYourPet-hexapod/hexapod-main/STL/frame.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_top_cover" file="../MakeYourPet-hexapod/hexapod-main/STL/top-cover.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_left_coxa" file="../MakeYourPet-hexapod/hexapod-main/STL/left-coxa.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_left_femur" file="../MakeYourPet-hexapod/hexapod-main/STL/left-femur.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_left_tibia" file="../MakeYourPet-hexapod/hexapod-main/STL/left-tibia.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_right_coxa" file="../MakeYourPet-hexapod/hexapod-main/STL/right-coxa.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_right_femur" file="../MakeYourPet-hexapod/hexapod-main/STL/right-femur.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_right_tibia" file="../MakeYourPet-hexapod/hexapod-main/STL/right-tibia.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_right_shield" file="../MakeYourPet-hexapod/hexapod-main/STL/shield.stl" scale="0.001 0.001 0.001"/>')
    xml.append('    <mesh name="mesh_left_shield"  file="../MakeYourPet-hexapod/hexapod-main/STL/shield.stl" scale="0.001 -0.001 0.001"/>')
    xml.append('    <mesh name="mesh_tip" file="../MakeYourPet-hexapod/hexapod-main/STL/tip.stl" scale="0.001 0.001 0.001"/>')
    xml.append('')
    xml.append('    <!-- 外觀材質配色 / Appearance materials and color palette -->')
    xml.append('    <material name="mat_frame" rgba="0.18 0.20 0.24 1.0" specular="0.5" shininess="0.3"/>')
    xml.append('    <material name="mat_armor" rgba="0.95 0.70 0.12 1.0" specular="0.8" shininess="0.5"/>')
    xml.append('    <material name="mat_leg_dark" rgba="0.15 0.15 0.16 1.0" specular="0.4" shininess="0.2"/>')
    xml.append('    <material name="mat_phone" rgba="0.08 0.08 0.10 1.0" specular="0.9" shininess="0.8"/>')
    xml.append('    <material name="mat_rubber" rgba="0.85 0.15 0.15 1.0"/>')
    xml.append('')
    xml.append('    <!-- 地面木紋貼圖與材質 / Ground wood texture and material -->')
    xml.append('    <texture name="tex_wood" type="2d" file="wood.png"/>')
    xml.append('    <material name="mat_wood" texture="tex_wood" texrepeat="14 14" reflectance="0.15" specular="0.3"/>')
    xml.append('')
    xml.append('    <!-- 地形高低起伏高度場資源 (160x160 高精網格，水平跨距 10x10 米，最大高差上限 8.0cm) / Terrain heightfield asset (160x160 grid, 10x10m span, max height diff 8.0cm) -->')
    xml.append('    <hfield name="terrain" nrow="160" ncol="160" size="5 5 0.08 0.1"/>')
    xml.append('  </asset>')
    xml.append('')
    xml.append('  <worldbody>')
    xml.append('    <!-- 1. 原版中央專屬強效聚光燈 (Spotlight，照射機器人與起點圓台，產生清晰足端陰影) / 1. Central high-intensity spotlight (illuminates robot and start pad with crisp foot shadows) -->')
    xml.append('    <light name="spotlight_center" diffuse="0.9 0.9 0.9" specular="0.3 0.3 0.3" pos="0 0 2.5" dir="0 0 -1"/>')
    xml.append('')
    xml.append('    <!-- 2. 四大區域獨立中心頂光 (東北、西北、西南、東南) / 2. Four regional top lights (NE, NW, SW, SE) -->')
    xml.append('    <light name="light_ne" diffuse="0.75 0.75 0.75" specular="0.2 0.2 0.2" pos=" 2.5  2.5 3.0" dir="0 0 -1"/>')
    xml.append('    <light name="light_nw" diffuse="0.75 0.75 0.75" specular="0.2 0.2 0.2" pos="-2.5  2.5 3.0" dir="0 0 -1"/>')
    xml.append('    <light name="light_sw" diffuse="0.75 0.75 0.75" specular="0.2 0.2 0.2" pos="-2.5 -2.5 3.0" dir="0 0 -1"/>')
    xml.append('    <light name="light_se" diffuse="0.75 0.75 0.75" specular="0.2 0.2 0.2" pos=" 2.5 -2.5 3.0" dir="0 0 -1"/>')
    xml.append('    <geom name="ground" type="hfield" hfield="terrain" material="mat_wood" friction="1.2 0.05 0.001" solref="0.002 1" solimp="0.95 0.99 0.0005 0.5 2"/>')
    xml.append('')
    xml.append('    <!-- 六足機體軀幹 (自由關節 freejoint，標稱站姿高度 0.082m) / Hexapod body trunk (freejoint, nominal standing height 0.082m) -->')
    xml.append('    <body name="trunk" pos="0 0 0.082">')
    xml.append('      <freejoint name="root"/>')
    xml.append('      <!-- 核心修正：機身主框架 (frame) 與頂蓋 (top-cover) 沿 Z 軸旋轉 90 度，使長軸與六足朝向對齊！ / Core fix: frame and top-cover rotated 90 deg along Z-axis to align long axis with hexapod heading! -->')
    xml.append('      <geom name="vis_frame" type="mesh" mesh="mesh_frame" pos="0 0 -0.008" euler="0 0 90" material="mat_frame" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <geom name="vis_cover" type="mesh" mesh="mesh_top_cover" pos="0 0 0.008" euler="0 0 90" material="mat_armor" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <!-- 物理碰撞層：內建極簡膠囊/盒體 (Collision Geometry, group=3) / Physics collision layer: minimal capsules/boxes (Collision Geometry, group=3) -->')
    xml.append('      <geom name="col_frame" type="box" size="0.095 0.065 0.012" rgba="0 0 0 0" mass="0.50" group="3"/>')
    xml.append('      <geom name="col_phone" type="box" pos="0 0 0.018" size="0.075 0.038 0.006" rgba="0 0 0 0" mass="0.18" group="3"/>')
    xml.append('      <geom name="col_batt"  type="box" pos="0 0 -0.018" size="0.055 0.028 0.010" rgba="0 0 0 0" mass="0.25" group="3"/>')
    xml.append('      <!-- 3D 空間坐標軸視覺導引 (XYZ 3D 羅盤箭頭, group=1 視覺層無質量無碰撞) / 3D coordinate axis visual guide (XYZ 3D compass arrows, group=1 visual layer no mass no collision) -->')
    xml.append('      <!-- +X (紅 / 前方 Front): 長度 100mm / +X (Red / Front): length 100mm -->')
    xml.append('      <geom name="vis_axis_x" type="cylinder" fromto="0 0 0.04 0.10 0 0.04" size="0.003" rgba="1.0 0.15 0.15 0.95" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <geom name="vis_axis_x_tip" type="sphere" pos="0.10 0 0.04" size="0.007" rgba="1.0 0.15 0.15 1.0" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <!-- +Y (綠 / 左側 Left): 長度 100mm / +Y (Green / Left): length 100mm -->')
    xml.append('      <geom name="vis_axis_y" type="cylinder" fromto="0 0 0.04 0 0.10 0.04" size="0.003" rgba="0.15 0.95 0.15 0.95" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <geom name="vis_axis_y_tip" type="sphere" pos="0 0.10 0.04" size="0.007" rgba="0.15 0.95 0.15 1.0" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <!-- +Z (藍 / 上方 Up): 長度 80mm / +Z (Blue / Up): length 80mm -->')
    xml.append('      <geom name="vis_axis_z" type="cylinder" fromto="0 0 0.04 0 0 0.12" size="0.003" rgba="0.15 0.45 1.0 0.95" contype="0" conaffinity="0" group="1"/>')
    xml.append('      <geom name="vis_axis_z_tip" type="sphere" pos="0 0 0.12" size="0.007" rgba="0.15 0.45 1.0 1.0" contype="0" conaffinity="0" group="1"/>')
    xml.append('')

    for leg in legs:
        lname = leg["name"]
        lside = leg["side"]
        lpos = leg["pos"]
        lyaw = leg["yaw"]
        tibia_mesh_pos = "0.0490 -0.0070 0.0065" if lside == "left" else "0.0490  0.0070 0.0065"
        vis_tip_pos = "0.0477 -0.0025 -0.1077" if lside == "left" else "0.0477  0.0025 -0.1077"
        col_tip_pos = "0.0477 -0.0025 -0.1059" if lside == "left" else "0.0477  0.0025 -0.1059"
        if lside == "left":
            femur_mesh_pos = "-0.0009 -0.0118 -0.0018"
            femur_mesh_euler = "90 0 35.26"
        else:
            femur_mesh_pos = "-0.0020 -0.0118 -0.0003"
            femur_mesh_euler = "-90 0 -35.26"

        xml.append(f'      <!-- ========== {leg["label"]} ({lname}) ========== -->')
        xml.append(f'      <body name="mount_{lname}" pos="{lpos}" euler="0 0 {lyaw}">')
        xml.append(f'        <!-- 關節 1 參考點：Coxa Yaw 舵機軸心 (淺藍色參考球) / Joint 1 reference: Coxa Yaw servo axis (light blue sphere) -->')
        xml.append(f'        <site name="tag_{lname}_coxa" pos="0 0 0.02" size="0.005" rgba="0.2 0.8 1.0 0.8"/>')
        xml.append(f'        <geom name="col_base_{lname}" type="box" size="0.015 0.015 0.01" rgba="0 0 0 0" mass="0.02" group="3"/>')
        xml.append(f'        <!-- 1. Coxa 轉向節 (Yaw): 43mm / 1. Coxa swivel joint (Yaw): 43mm -->')
        xml.append(f'        <body name="coxa_{lname}" pos="0 0 0">')
        xml.append(f'          <joint name="joint_{lname}_coxa" type="hinge" axis="0 0 1" range="-45 45" damping="0.1"/>')
        xml.append(f'          <geom name="vis_coxa_{lname}" type="mesh" mesh="mesh_{lside}_coxa" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>')
        xml.append(f'          <geom name="col_coxa_{lname}" type="capsule" fromto="0 0 0 0.043 0 0" size="0.012" mass="0.06" rgba="0 0 0 0" group="3"/>')
        xml.append(f'          <!-- 2. Femur 大腿升降節 (Pitch): 80mm (Z 字形斜挑頂端 +46mm) / 2. Femur lift joint (Pitch): 80mm (Z-shape upper apex +46mm) -->')
        xml.append(f'          <body name="femur_{lname}" pos="0.043 0 0">')
        xml.append(f'            <joint name="joint_{lname}_femur" type="hinge" axis="0 1 0" range="-45 45" damping="0.1"/>')
        xml.append(f'            <!-- 關節 2 參考點：Femur Pitch 舵機軸心 (淺藍色參考球) / Joint 2 reference: Femur Pitch servo axis (light blue sphere) -->')
        xml.append(f'            <site name="tag_{lname}_femur" pos="0 0 0" size="0.005" rgba="0.2 0.8 1.0 0.8"/>')
        xml.append(f'            <geom name="vis_femur_{lname}" type="mesh" mesh="mesh_{lside}_femur" pos="{femur_mesh_pos}" euler="{femur_mesh_euler}" material="mat_armor" contype="0" conaffinity="0" group="1"/>')
        xml.append(f'            <geom name="col_femur_{lname}" type="capsule" fromto="0 0 0 0.065 0 0.046" size="0.011" mass="0.08" rgba="0 0 0 0" group="3"/>')
        xml.append(f'            <!-- 3. Tibia 小腿屈伸節 (Pitch): 134mm / 3. Tibia extension joint (Pitch): 134mm -->')
        xml.append(f'            <body name="tibia_{lname}" pos="0.065 0 0.046">')
        xml.append(f'              <joint name="joint_{lname}_tibia" type="hinge" axis="0 1 0" range="-60 60" damping="0.1"/>')
        xml.append(f'              <!-- 關節 3 參考點：Tibia Knee Pitch 舵機軸心 (淺藍色參考球) / Joint 3 reference: Tibia Knee Pitch servo axis (light blue sphere) -->')
        xml.append(f'              <site name="tag_{lname}_tibia" pos="0 0 0" size="0.005" rgba="0.2 0.8 1.0 0.8"/>')
        xml.append(f'              <geom name="vis_tibia_{lname}" type="mesh" mesh="mesh_{lside}_tibia" pos="{tibia_mesh_pos}" euler="0 0 180" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>')
        xml.append(f'              <geom name="vis_shield_{lname}" type="mesh" mesh="mesh_{lside}_shield" pos="{tibia_mesh_pos}" euler="0 0 180" material="mat_armor" contype="0" conaffinity="0" group="1"/>')
        xml.append(f'              <geom name="col_tibia_{lname}" type="capsule" fromto="0 0 0 {col_tip_pos}" size="0.0048" mass="0.05" rgba="0 0 0 0" group="3"/>')
        xml.append(f'              <!-- 4. 足端觸地接觸球 (Foot Tip: 視覺 4.2mm 網格 + 校準 4.8mm 接觸球，零懸空、零穿模破圖) / 4. Foot tip contact sphere (Foot Tip: visual 4.2mm mesh + calibrated 4.8mm sphere, zero gap/clipping) -->')
        xml.append(f'              <geom name="vis_tip_{lname}" type="mesh" mesh="mesh_tip" pos="{vis_tip_pos}" material="mat_rubber" contype="0" conaffinity="0" group="1"/>')
        xml.append(f'              <geom name="tip_{lname}" type="sphere" pos="{col_tip_pos}" size="0.0048" mass="0.01" material="mat_rubber" friction="1.2 0.05 0.001" solref="0.002 1" solimp="0.95 0.99 0.0005 0.5 2" group="3"/>')
        xml.append(f'            </body>')
        xml.append(f'          </body>')
        xml.append(f'        </body>')
        xml.append(f'      </body>')
        xml.append('')

    xml.append('    </body>')
    xml.append('  </worldbody>')
    xml.append('')

    # 排除自體干涉碰撞 / Exclude self-collision
    xml.append('  <contact>')
    for leg in legs:
        lname = leg["name"]
        xml.append(f'    <exclude body1="mount_{lname}" body2="coxa_{lname}"/>')
        xml.append(f'    <exclude body1="coxa_{lname}" body2="femur_{lname}"/>')
        xml.append(f'    <exclude body1="femur_{lname}" body2="tibia_{lname}"/>')
    xml.append('  </contact>')
    xml.append('')

    # 18 顆伺服馬達致動器 / 18 servo motor actuators
    xml.append('  <actuator>')
    for leg in legs:
        lname = leg["name"]
        xml.append(f'    <position name="mot_{lname}_c" joint="joint_{lname}_coxa"  kp="12.0" kv="1.2" ctrlrange="-45 45" forcerange="-3.0 3.0"/>')
        xml.append(f'    <position name="mot_{lname}_f" joint="joint_{lname}_femur" kp="12.0" kv="1.2" ctrlrange="-45 45" forcerange="-3.0 3.0"/>')
        xml.append(f'    <position name="mot_{lname}_t" joint="joint_{lname}_tibia" kp="12.0" kv="1.2" ctrlrange="-60 60" forcerange="-3.0 3.0"/>')
    xml.append('  </actuator>')
    xml.append('</mujoco>')

    content = '\n'.join(xml)
    with open('models/hexapod.xml', 'w', encoding='utf-8') as f:
        f.write(content)
    print("models/hexapod.xml successfully generated with calibrated foot tips (4.8mm, zero gap) and 8cm hfield!")

if __name__ == '__main__':
    generate_hexapod_xml()
