r"""
Make Your Pet - Blender 5.2 影視級步態軌跡渲染與動態鏡頭構建腳本
============================================================
執行方式：
& "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" --python blender_cinematic.py

自動化功能：
1. 自動載入 gait_trajectory.json 與 11 個高精度 3D 部件網格
2. 建立 32 個獨立機體零件並載入 50 FPS AI 步態關鍵影格 (Keyframes)
3. 賦予好萊塢級 PBR 材質（蜂黃金屬裝甲、消光鈦黑骨架、高抓地橡膠、賽博發光 LED）
4. 建立影視級三點式動態跟隨燈光組 (Key / Fill / Rim Light)
5. 建立電影級追蹤攝影機（50mm 鏡頭、f/2.8 景深虛化、跟隨運鏡）
6. 搭建微反光展示台與地面接觸陰影
7. 配置 Blender 5.2 EEVEE Next / Cycles 引擎並儲存為 hexapod_cinematic.blend
"""

import os
import sys
import json
import math
import bpy
from mathutils import Vector, Quaternion, Matrix


def create_pbr_material(name: str, base_color, metallic=0.0, roughness=0.5, clearcoat=0.0, is_emission=False, emission_color=(0, 1, 1, 1), emission_strength=10.0):
    """建立或獲取高品質 PBR Principled BSDF 材質"""
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()

    # 輸出節點
    node_out = nodes.new(type="ShaderNodeOutputMaterial")
    node_out.location = (400, 0)

    if is_emission:
        node_emit = nodes.new(type="ShaderNodeEmission")
        node_emit.location = (100, 0)
        node_emit.inputs["Color"].default_value = emission_color
        node_emit.inputs["Strength"].default_value = emission_strength
        mat.node_tree.links.new(node_emit.outputs["Emission"], node_out.inputs["Surface"])
    else:
        node_bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")
        node_bsdf.location = (100, 0)
        node_bsdf.inputs["Base Color"].default_value = base_color
        node_bsdf.inputs["Metallic"].default_value = metallic
        node_bsdf.inputs["Roughness"].default_value = roughness
        if "Coat Weight" in node_bsdf.inputs:
            node_bsdf.inputs["Coat Weight"].default_value = clearcoat
        elif "Coat" in node_bsdf.inputs:
            node_bsdf.inputs["Coat"].default_value = clearcoat
        mat.node_tree.links.new(node_bsdf.outputs["BSDF"], node_out.inputs["Surface"])

    return mat


def build_cinematic_scene(json_path="gait_trajectory.json", mesh_dir="blender_assets/meshes"):
    print("=" * 70)
    print("🎬 [Blender 5.2] 啟動 Make Your Pet 影視級步態場景自動化渲染構建...")
    print("=" * 70)

    if not os.path.exists(json_path):
        raise FileNotFoundError(f"找不到步態軌跡檔案: {json_path}！請先執行 python record_trajectory.py")

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    meta = data["metadata"]
    fps = meta.get("fps", 50)
    frames_data = data["frames"]
    total_frames = len(frames_data)
    visual_geoms = data["visual_geoms"]

    # 1. 清理預設場景
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.fps = fps
    scene.frame_start = 1
    scene.frame_end = total_frames
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100

    # 顏色管理設定 (AgX - Blender 現代好萊塢級高動態範圍)
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"

    # 建立主集合
    col_robot = bpy.data.collections.new("Hexapod_Robot")
    col_stage = bpy.data.collections.new("Studio_Stage")
    col_lights = bpy.data.collections.new("Cinematic_Lights")
    scene.collection.children.link(col_robot)
    scene.collection.children.link(col_stage)
    scene.collection.children.link(col_lights)

    # 2. 構建頂級 PBR 材質庫
    print("🎨 正在編譯電影級 PBR 材質庫...")
    # (a) 賽博黃蜂/碳纖裝甲：高光澤微金屬質感、清漆外塗層
    mat_armor = create_pbr_material(
        "M_CyberArmor",
        base_color=(0.95, 0.68, 0.10, 1.0),
        metallic=0.75,
        roughness=0.25,
        clearcoat=0.6
    )
    # (b) 陽極氧化黑鋁合金骨架
    mat_chassis = create_pbr_material(
        "M_DarkTitanium",
        base_color=(0.045, 0.048, 0.052, 1.0),
        metallic=0.85,
        roughness=0.40
    )
    # (c) 吸震止滑矽膠足墊
    mat_rubber = create_pbr_material(
        "M_SiliconeRubber",
        base_color=(0.02, 0.02, 0.02, 1.0),
        metallic=0.0,
        roughness=0.85
    )
    # (d) 賽博霓虹發光燈條 (Cyan Blue)
    mat_led = create_pbr_material(
        "M_CyberNeonLED",
        base_color=(0, 0, 0, 1),
        is_emission=True,
        emission_color=(0.05, 0.85, 1.0, 1.0),
        emission_strength=18.0
    )

    # 3. 匯入 11 個幾何網格資源
    print(f"📦 正在載入 3D 幾何網格 (來源: {mesh_dir})...")
    loaded_meshes = {}
    for g in visual_geoms:
        mname = g["mesh_name"]
        if mname not in loaded_meshes:
            obj_path = os.path.join(mesh_dir, f"{mname}.obj")
            if not os.path.exists(obj_path):
                print(f"  [警告] 找不到網格檔案: {obj_path}")
                continue
            bpy.ops.wm.obj_import(filepath=obj_path)
            imported_obj = bpy.context.selected_objects[0]
            mesh_data = imported_obj.data
            mesh_data.name = f"Mesh_{mname}"
            # 啟用平滑著色 (Smooth Shading)
            for poly in mesh_data.polygons:
                poly.use_smooth = True
            loaded_meshes[mname] = mesh_data
            # 從當前場景移除臨時導入的物件 (保留 mesh_data 供複製)
            bpy.data.objects.remove(imported_obj, do_unlink=True)

    # 4. 生成 32 個獨立機體物件並分配材質
    print("🤖 正在建立 32 個機體零件並指派物理裝甲材質...")
    robot_objects = {}
    for g in visual_geoms:
        gname = g["geom_name"]
        mname = g["mesh_name"]
        cat = g["mat_category"]

        if mname not in loaded_meshes:
            continue

        obj = bpy.data.objects.new(gname, loaded_meshes[mname])
        obj.rotation_mode = "QUATERNION"
        col_robot.objects.link(obj)

        # 指派材質
        if cat == "armor":
            obj.data.materials.append(mat_armor)
        elif cat == "rubber":
            obj.data.materials.append(mat_rubber)
        else:
            obj.data.materials.append(mat_chassis)

        robot_objects[gname] = obj

    # 建立 LED 裝飾光條（置於機身頂蓋前後）
    mesh_led = bpy.data.meshes.new("LED_Strip_Mesh")
    obj_led = bpy.data.objects.new("Robot_LED_Headlight", mesh_led)
    col_robot.objects.link(obj_led)
    obj_led.data.materials.append(mat_led)
    # 建立簡易立方體光條
    import bmesh
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    # 縮放成條狀 (長 60mm, 寬 6mm, 高 4mm)
    for v in bm.verts:
        v.co.x *= 0.015
        v.co.y *= 0.055
        v.co.z *= 0.005
        v.co.z += 0.012
    bm.to_mesh(mesh_led)
    bm.free()

    # 5. 載入關鍵影格 (AI 步態軌跡驅動)
    print(f"⚡ 正在注入 50 FPS AI 步態關鍵影格 (共 {total_frames} 幀)...")
    
    # 建立一個相機追蹤目標 Empty
    target_empty = bpy.data.objects.new("Hexapod_Camera_Target", None)
    target_empty.empty_display_type = "SPHERE"
    target_empty.empty_display_size = 0.05
    col_robot.objects.link(target_empty)

    for f_idx, frame_info in enumerate(frames_data):
        frame_num = frame_info["frame"]
        tpos = frame_info["trunk_pos"]

        # 設定追蹤空物件位置 (機身幾何中心)
        target_empty.location = (tpos[0], tpos[1], tpos[2] + 0.02)
        target_empty.keyframe_insert(data_path="location", frame=frame_num)

        # LED 燈條隨 trunk 運動
        if "vis_cover" in frame_info["geoms"]:
            cov = frame_info["geoms"]["vis_cover"]
            obj_led.location = (cov["pos"][0], cov["pos"][1], cov["pos"][2] + 0.005)
            obj_led.rotation_mode = "QUATERNION"
            obj_led.rotation_quaternion = tuple(cov["quat"])
            obj_led.keyframe_insert(data_path="location", frame=frame_num)
            obj_led.keyframe_insert(data_path="rotation_quaternion", frame=frame_num)

        # 寫入 32 個幾何零件關鍵影格
        for gname, gdata in frame_info["geoms"].items():
            if gname in robot_objects:
                rob_obj = robot_objects[gname]
                rob_obj.location = tuple(gdata["pos"])
                rob_obj.rotation_quaternion = tuple(gdata["quat"])
                rob_obj.keyframe_insert(data_path="location", frame=frame_num)
                rob_obj.keyframe_insert(data_path="rotation_quaternion", frame=frame_num)

    # 6. 搭建影視級展示地面與舞台
    print("🏛️ 正在搭建微反射金屬展示地面與暗色攝影棚...")
    # 地面 PBR 材質 (深色微粗糙拋光金屬，帶有清晰微倒影)
    mat_floor = create_pbr_material(
        "M_StudioFloor",
        base_color=(0.02, 0.022, 0.025, 1.0),
        metallic=0.92,
        roughness=0.28
    )
    bpy.ops.mesh.primitive_plane_add(size=60.0, location=(0, 0, 0))
    floor_obj = bpy.context.active_object
    floor_obj.name = "Studio_Showroom_Floor"
    floor_obj.data.materials.append(mat_floor)
    col_stage.objects.link(floor_obj)
    scene.collection.objects.unlink(floor_obj)

    # 7. 佈置影視級「三點式」動態跟隨燈光組 (Three-Point Studio Lighting)
    print("💡 正在配置好萊塢級三點式光影 (Key / Fill / Rim / Top)...")
    light_rig_empty = bpy.data.objects.new("Lighting_Rig_Root", None)
    col_lights.objects.link(light_rig_empty)

    # 燈光 Rig 動態跟隨機身軀幹
    for frame_info in frames_data:
        frame_num = frame_info["frame"]
        tpos = frame_info["trunk_pos"]
        light_rig_empty.location = (tpos[0], tpos[1], 0)
        light_rig_empty.keyframe_insert(data_path="location", frame=frame_num)

    # (a) 主光 (Key Light): 暖白大面積柔光箱，45度角照射
    key_light_data = bpy.data.lights.new(name="Key_Light", type="AREA")
    key_light_data.energy = 500.0
    key_light_data.size = 2.5
    key_light_data.color = (1.0, 0.96, 0.90)
    key_light_obj = bpy.data.objects.new("Light_Key", key_light_data)
    key_light_obj.location = (1.2, -1.2, 1.8)
    key_light_obj.rotation_euler = (math.radians(45), math.radians(20), math.radians(45))
    key_light_obj.parent = light_rig_empty
    col_lights.objects.link(key_light_obj)

    # (b) 補光 (Fill Light): 冷藍色中面積柔光箱，填充陰影暗部
    fill_light_data = bpy.data.lights.new(name="Fill_Light", type="AREA")
    fill_light_data.energy = 180.0
    fill_light_data.size = 3.0
    fill_light_data.color = (0.70, 0.82, 1.0)
    fill_light_obj = bpy.data.objects.new("Light_Fill", fill_light_data)
    fill_light_obj.location = (-1.1, -1.4, 1.2)
    fill_light_obj.rotation_euler = (math.radians(50), math.radians(-25), math.radians(-30))
    fill_light_obj.parent = light_rig_empty
    col_lights.objects.link(fill_light_obj)

    # (c) 輪廓光/背光 (Rim Light): 強烈高對比青藍邊緣光，勾勒硬派機械線條
    rim_light_data = bpy.data.lights.new(name="Rim_Light", type="AREA")
    rim_light_data.energy = 750.0
    rim_light_data.size = 2.0
    rim_light_data.color = (0.15, 0.75, 1.0)
    rim_light_obj = bpy.data.objects.new("Light_Rim", rim_light_data)
    rim_light_obj.location = (-1.2, 1.5, 0.9)
    rim_light_obj.rotation_euler = (math.radians(-55), math.radians(35), math.radians(140))
    rim_light_obj.parent = light_rig_empty
    col_lights.objects.link(rim_light_obj)

    # (d) 頂部聚焦射燈 (Top Spot Light): 強調機甲頂蓋結構
    top_light_data = bpy.data.lights.new(name="Top_Spot", type="SPOT")
    top_light_data.energy = 250.0
    top_light_data.spot_size = math.radians(55)
    top_light_data.spot_blend = 0.5
    top_light_data.color = (0.95, 0.98, 1.0)
    top_light_obj = bpy.data.objects.new("Light_Top", top_light_data)
    top_light_obj.location = (0, 0, 2.5)
    top_light_obj.rotation_euler = (0, 0, 0)
    top_light_obj.parent = light_rig_empty
    col_lights.objects.link(top_light_obj)

    # 8. 建立電影級追蹤攝影機 (Cinematic Camera with Tracking & DoF)
    print("🎥 正在建立 50mm 影視級跟隨運鏡攝影機 (含光學景深 f/4.0)...")
    cam_data = bpy.data.cameras.new("Cinematic_Camera")
    cam_data.lens = 45.0 # 45mm 寬廣大氣透視
    cam_data.sensor_width = 36.0 # 全片幅感光元件 (Full Frame 35mm)
    cam_data.dof.use_dof = True
    cam_data.dof.focus_object = target_empty
    cam_data.dof.aperture_fstop = 4.0 # 清晰銳利景深，同時兼顧微柔化背景

    cam_obj = bpy.data.objects.new("Camera_Main", cam_data)
    scene.collection.objects.link(cam_obj)
    scene.camera = cam_obj

    # 增加追蹤約束 (Track To Constraint)，始終牢牢鎖定機器人軀幹焦點
    track_con = cam_obj.constraints.new(type="TRACK_TO")
    track_con.target = target_empty
    track_con.track_axis = "TRACK_NEGATIVE_Z"
    track_con.up_axis = "UP_Y"

    # 電影攝影機運鏡軌跡 (低角度英雄仰角 3/4 側跟拍 + 微距推進)
    for frame_info in frames_data:
        frame_num = frame_info["frame"]
        tpos = frame_info["trunk_pos"]
        t = frame_num / fps

        # 完美取景構圖：距離機器人 1.05m，高度 0.35m，側前方 42 度斜視全視角
        cam_dist = 1.05 - 0.08 * math.sin(t * 0.7)
        cam_angle = math.radians(42) + 0.12 * math.sin(t * 0.5)
        
        cam_x = tpos[0] + cam_dist * math.cos(cam_angle)
        cam_y = tpos[1] - cam_dist * math.sin(cam_angle)
        cam_z = 0.35 + 0.04 * math.cos(t * 0.6)

        cam_obj.location = (cam_x, cam_y, cam_z)
        cam_obj.keyframe_insert(data_path="location", frame=frame_num)

    # 9. 渲染引擎調校 (優選 EEVEE Next，亦完美兼容 Cycles)
    print("⚙️ 正在啟用 Blender 5.2 渲染管線...")
    # Blender 5.x 預設即為 EEVEE Next，支援光線追蹤與即時陰影
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except:
        scene.render.engine = "BLENDER_EEVEE"

    # 輸出配置 (可直接產出 MP4 或高解析 PNG 序列)
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.filepath = "//renders/frame_"

    # 10. 配置 3D 視圖預設模式：自動進入攝影機視角、開啟即時渲染著色、關閉雜亂關係輔助線
    for area in bpy.context.screen.areas:
        if area.type == "VIEW_3D":
            for space in area.spaces:
                if space.type == "VIEW_3D":
                    space.region_3d.view_perspective = "CAMERA"
                    space.shading.type = "RENDERED"
                    space.overlay.show_relationship_lines = False

    # 儲存 .blend 專案檔案
    output_blend = "hexapod_cinematic.blend"
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(output_blend))

    print("=" * 70)
    print(f"🎉 影視級場景構建完成！")
    print(f"📁 專案檔案已儲存至: {os.path.abspath(output_blend)}")
    print(f"📊 動畫時長: {total_frames} 幀 ({total_frames/fps:.1f} 秒, 50 FPS)")
    print("=" * 70)
    print("【兩種觀看與算圖方式】：")
    print("  1. 🖱️【開啟 GUI 預覽】：雙擊 hexapod_cinematic.blend，按 [空白鍵] 播放，按 [Z] 切換 Rendered 視角。")
    print("  2. ⚡【背景算圖一鍵渲染】：")
    print('     & "C:\\Program Files\\Blender Foundation\\Blender 5.2\\blender.exe" -b hexapod_cinematic.blend -f 1')
    print("=" * 70 + "\n")


if __name__ == "__main__":
    build_cinematic_scene()
