import mujoco
import matplotlib.pyplot as plt
import numpy as np

model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

bid_t = model.body('tibia_L2').id
knee_pos = data.xpos[bid_t].copy() # [0, 0.1895, 0.101]

renderer = mujoco.Renderer(model, 480, 640)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE

fig, axs = plt.subplots(1, 3, figsize=(18, 6))

views = [
    ('Side View (Looking along Joint Axis X from Right)', 0, 0, 0.08),
    ('Side View (Looking along Joint Axis X from Left)', 180, 0, 0.08),
    ('Top View (Looking down Z)', 90, -89.9, 0.10),
]

for idx, (title, az, el, dist) in enumerate(views):
    cam.lookat = knee_pos
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    renderer.update_scene(data, cam)
    axs[idx].imshow(renderer.render())
    axs[idx].set_title(title, fontsize=11, fontweight='bold')
    axs[idx].axis('off')

plt.tight_layout()
plt.savefig('super_closeup_knee.png', dpi=150)
print('super_closeup_knee.png saved!')
