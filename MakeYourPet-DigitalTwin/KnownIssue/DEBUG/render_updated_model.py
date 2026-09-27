import mujoco
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

fig, axs = plt.subplots(2, 2, figsize=(14, 11))

views = [
    ('Isometric View', [0, 0, 0.05], 0.55, -30, 45),
    ('Top View (Heading +X is right)', [0, 0, 0.05], 0.55, -89, 90),
    ('Close-up: Left Leg L2', [0, 0.14, 0.04], 0.28, -20, 145),
    ('Close-up: Right Leg R2', [0, -0.14, 0.04], 0.28, -20, -35),
]

for idx, (title, lookat, dist, elev, azim) in enumerate(views):
    r = idx // 2
    c = idx % 2
    renderer = mujoco.Renderer(model, 480, 640)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat = lookat
    camera.distance = dist
    camera.elevation = elev
    camera.azimuth = azim
    renderer.update_scene(data, camera)
    img = renderer.render()
    renderer.close()

    axs[r, c].imshow(img)
    axs[r, c].set_title(title, fontsize=13, fontweight='bold')
    axs[r, c].axis('off')

plt.tight_layout()
plt.savefig('calibrated_femur_render.png', dpi=150)
print('Saved calibrated_femur_render.png successfully!')
