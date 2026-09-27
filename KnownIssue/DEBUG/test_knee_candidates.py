import mujoco
import matplotlib.pyplot as plt
import numpy as np

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Test candidate femur configurations for L2:
# In L2:
# Old: euler="90 0 35.26" pos="-0.0009 -0.0118 -0.0018"
# New Case B: euler="90 -35.26 0" pos="-0.0019 -0.0110 -0.0025"
# Or let's test a few variations of signs for euler
candidates = [
    ('Current (Separated)', '90 0 35.26', '-0.0009 -0.0118 -0.0018'),
    ('Euler 90 -35.26 0', '90 -35.26 0', '-0.0019 -0.0110 -0.0025'),
    ('Euler -90 -35.26 0', '-90 -35.26 0', '-0.0030 -0.0110 -0.0009'),
    ('Euler 90 35.26 0', '90 35.26 0', '-0.0019 -0.0110 -0.0025'),
]

fig, axs = plt.subplots(2, 4, figsize=(20, 10))

for idx, (title, euler_str, pos_str) in enumerate(candidates):
    test_xml = xml.replace(
        '<geom name="vis_femur_L2" type="mesh" mesh="mesh_left_femur" pos="-0.0009 -0.0118 -0.0018" euler="90 0 35.26"',
        f'<geom name="vis_femur_L2" type="mesh" mesh="mesh_left_femur" pos="{pos_str}" euler="{euler_str}"'
    )
    with open('models/test_knee_temp.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    m = mujoco.MjModel.from_xml_path('models/test_knee_temp.xml')
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
    cam.distance = 0.16
    cam.azimuth = 135
    cam.elevation = -25
    renderer.update_scene(d, cam)
    axs[1, idx].imshow(renderer.render())
    axs[1, idx].set_title(f'{title}\nPerspective', fontsize=11, fontweight='bold')
    axs[1, idx].axis('off')

    renderer.close()

plt.tight_layout()
plt.savefig('femur_euler_candidates.png', dpi=150)
print('femur_euler_candidates.png saved successfully!')

import os
if os.path.exists('models/test_knee_temp.xml'):
    os.remove('models/test_knee_temp.xml')
