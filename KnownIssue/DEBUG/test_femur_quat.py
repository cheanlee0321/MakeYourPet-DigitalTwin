import mujoco
import matplotlib.pyplot as plt

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Test Option 1 vs Option 2 vs Current:
tests = [
    ('Current', 'pos="-0.0009 -0.0118 -0.0018" euler="90 0 35.26"'),
    ('Option 1 (quat)', 'pos="-0.00303 0.011 -0.00092" quat="-0.67385 0.67385 0.21432 0.21432"'),
    ('Option 2 (quat)', 'pos="-0.00187 -0.011 -0.00255" quat="0.67385 0.67385 -0.21432 0.21432"'),
]

fig, axs = plt.subplots(2, 3, figsize=(18, 10))

for idx, (title, geom_attrs) in enumerate(tests):
    test_xml = xml.replace(
        '<geom name="vis_femur_L2" type="mesh" mesh="mesh_left_femur" pos="-0.0009 -0.0118 -0.0018" euler="90 0 35.26"',
        f'<geom name="vis_femur_L2" type="mesh" mesh="mesh_left_femur" {geom_attrs}'
    )
    with open('models/test_temp.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    m = mujoco.MjModel.from_xml_path('models/test_temp.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    bid = m.body('tibia_L2').id
    knee_lookat = d.xpos[bid].copy()

    renderer = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    # Row 0: Top View
    cam.lookat = knee_lookat
    cam.distance = 0.15
    cam.azimuth = 90
    cam.elevation = -89
    renderer.update_scene(d, cam)
    axs[0, idx].imshow(renderer.render())
    axs[0, idx].set_title(f'{title}\nTop View', fontsize=11, fontweight='bold')
    axs[0, idx].axis('off')

    # Row 1: Perspective 3D
    cam.distance = 0.18
    cam.azimuth = 135
    cam.elevation = -25
    renderer.update_scene(d, cam)
    axs[1, idx].imshow(renderer.render())
    axs[1, idx].set_title(f'{title}\nPerspective', fontsize=11, fontweight='bold')
    axs[1, idx].axis('off')

    renderer.close()

plt.tight_layout()
plt.savefig('femur_quat_comparison.png', dpi=150)
print('femur_quat_comparison.png saved!')

import os
if os.path.exists('models/test_temp.xml'):
    os.remove('models/test_temp.xml')
