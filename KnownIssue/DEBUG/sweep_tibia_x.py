import mujoco
import matplotlib.pyplot as plt

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Let's test moving vis_tibia along X (which is the longitudinal direction of the leg):
# In tibia body with euler="0 0 180", +X in pos moves the mesh in +X!
# Currently pos="0.0634 -0.0070 -0.0028"
# Let's test X = 0.040, 0.045, 0.050, 0.055, 0.060, 0.0634
candidates_x = [0.040, 0.045, 0.050, 0.055, 0.060, 0.0634]

fig, axs = plt.subplots(2, 3, figsize=(18, 10))

for idx, x_val in enumerate(candidates_x):
    r, c = idx // 3, idx % 3
    test_pos = f"{x_val:.4f} -0.0070 -0.0028"
    test_xml = xml.replace(
        '<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="0.0634 -0.0070 -0.0028" euler="0 0 180"',
        f'<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="{test_pos}" euler="0 0 180"'
    )
    with open('models/test_tibia_temp.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    m = mujoco.MjModel.from_xml_path('models/test_tibia_temp.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    bid = m.body('tibia_L2').id
    knee_lookat = d.xpos[bid].copy()

    renderer = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    cam.lookat = knee_lookat
    cam.distance = 0.14
    cam.azimuth = 90
    cam.elevation = -89
    renderer.update_scene(d, cam)
    axs[r, c].imshow(renderer.render())
    axs[r, c].set_title(f'Tibia pos X = {x_val:.4f}\nTop View', fontsize=11, fontweight='bold')
    axs[r, c].axis('off')
    renderer.close()

plt.tight_layout()
plt.savefig('tibia_x_sweep.png', dpi=150)
print('tibia_x_sweep.png saved!')

import os
if os.path.exists('models/test_tibia_temp.xml'):
    os.remove('models/test_tibia_temp.xml')
