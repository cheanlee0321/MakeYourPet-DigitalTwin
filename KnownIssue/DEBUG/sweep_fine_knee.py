import mujoco
import matplotlib.pyplot as plt
import numpy as np

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Test a fine grid around X=0.047 to 0.051, Z=0.005 to 0.008:
test_cases = [
    ('X=0.0470, Z=0.0065', '0.0470 -0.0070 0.0065'),
    ('X=0.0480, Z=0.0065', '0.0480 -0.0070 0.0065'),
    ('X=0.0490, Z=0.0065', '0.0490 -0.0070 0.0065'),
    ('X=0.0500, Z=0.0065', '0.0500 -0.0070 0.0065'),
]

fig, axs = plt.subplots(2, 4, figsize=(20, 10))

for idx, (title, pos_str) in enumerate(test_cases):
    test_xml = xml.replace(
        '<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="0.0634 -0.0070 -0.0028" euler="0 0 180" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="{pos_str}" euler="0 0 180" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>'
    ).replace(
        '<geom name="vis_shield_L2" type="mesh" mesh="mesh_left_shield" pos="0.0634 -0.0070 -0.0028" euler="0 0 180" material="mat_armor" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="vis_shield_L2" type="mesh" mesh="mesh_left_shield" pos="{pos_str}" euler="0 0 180" material="mat_armor" contype="0" conaffinity="0" group="1"/>'
    )
    with open('models/test_fine.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    m = mujoco.MjModel.from_xml_path('models/test_fine.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    bid = m.body('tibia_L2').id
    knee_lookat = d.xpos[bid].copy()

    renderer = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    # Row 0: Side view along joint axis
    cam.lookat = knee_lookat
    cam.distance = 0.09
    cam.azimuth = 0
    cam.elevation = 0
    renderer.update_scene(d, cam)
    axs[0, idx].imshow(renderer.render())
    axs[0, idx].set_title(f'{title}\nSide View', fontsize=11, fontweight='bold')
    axs[0, idx].axis('off')

    # Row 1: Perspective 3D
    cam.distance = 0.14
    cam.azimuth = 135
    cam.elevation = -20
    renderer.update_scene(d, cam)
    axs[1, idx].imshow(renderer.render())
    axs[1, idx].set_title(f'{title}\nPerspective', fontsize=11, fontweight='bold')
    axs[1, idx].axis('off')

    renderer.close()

plt.tight_layout()
plt.savefig('fine_knee_sweep.png', dpi=150)
print('fine_knee_sweep.png saved!')

import os
if os.path.exists('models/test_fine.xml'):
    os.remove('models/test_fine.xml')
