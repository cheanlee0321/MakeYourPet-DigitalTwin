import mujoco
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

# Let's inspect the global positions of femur and tibia bodies
body_femur_id = model.body('femur_L2').id
body_tibia_id = model.body('tibia_L2').id
femur_pos = data.xpos[body_femur_id]
tibia_pos = data.xpos[body_tibia_id]

print(f"L2 Femur body pos: {femur_pos}")
print(f"L2 Tibia body pos: {tibia_pos}")

renderer = mujoco.Renderer(model, 480, 640)
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FREE

fig, axs = plt.subplots(2, 2, figsize=(14, 12))

# Focus right on the knee joint of L2 (x ~ 0.043 + 0.065 = 0.108 relative to coxa)
# Global position of L2 knee:
knee_lookat = tibia_pos.copy()

angles = [
    ('L2 Knee - Top View', 90, -89, 0.15),
    ('L2 Knee - Side View (from front of robot)', 0, 0, 0.15),
    ('L2 Knee - Lateral View (looking inwards)', 90, 0, 0.15),
    ('L2 Knee - Perspective 3D', 135, -25, 0.18),
]

for idx, (title, az, el, dist) in enumerate(angles):
    r, c = idx // 2, idx % 2
    camera.lookat = knee_lookat
    camera.distance = dist
    camera.azimuth = az
    camera.elevation = el
    renderer.update_scene(data, camera)
    axs[r, c].imshow(renderer.render())
    axs[r, c].set_title(title, fontsize=12, fontweight='bold')
    axs[r, c].axis('off')

plt.tight_layout()
plt.savefig('knee_joint_inspection.png', dpi=150)
print('knee_joint_inspection.png saved!')
