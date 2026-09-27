import mujoco
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

renderer = mujoco.Renderer(model, 480, 640)
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FREE

fig, axs = plt.subplots(2, 3, figsize=(15, 10))

legs = [
    ('L1', 0, 0, [0.12, 0.20, -0.015], 135),
    ('L2', 0, 1, [0,    0.24, -0.015], 145),
    ('L3', 0, 2, [-0.12, 0.20, -0.015], 155),
    ('R1', 1, 0, [0.12, -0.20, -0.015], -45),
    ('R2', 1, 1, [0,   -0.24, -0.015], -35),
    ('R3', 1, 2, [-0.12, -0.20, -0.015], -25),
]

for lname, r, c, lookat, az in legs:
    camera.lookat = lookat
    camera.distance = 0.14
    camera.elevation = -10
    camera.azimuth = az
    renderer.update_scene(data, camera)
    axs[r, c].imshow(renderer.render())
    pos_val = model.geom(f"vis_tip_{lname}").pos
    axs[r, c].set_title(f"Leg {lname} Foot Tip [{pos_val[0]:.4f}, {pos_val[1]:.4f}, {pos_val[2]:.4f}]", fontsize=11, fontweight='bold')
    axs[r, c].axis('off')

plt.tight_layout()
plt.savefig('all_foot_tips_aligned.png', dpi=150)
print('all_foot_tips_aligned.png saved successfully!')
