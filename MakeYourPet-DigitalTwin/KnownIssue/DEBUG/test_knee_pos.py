import mujoco
import matplotlib.pyplot as plt

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Test candidate positions for vis_tibia_L2 and vis_shield_L2:
candidates = [
    ('Current (Separated)', '0.0634 -0.0070 -0.0028'),
    ('X=0.0491, Z=-0.0028', '0.0491 -0.0070 -0.0028'),
    ('X=0.0491, Z=+0.0065', '0.0491 -0.0070  0.0065'),
    ('X=0.0450, Z=+0.0065', '0.0450 -0.0070  0.0065'),
]

fig, axs = plt.subplots(3, 4, figsize=(20, 14))

for idx, (title, pos_str) in enumerate(candidates):
    test_xml = xml.replace(
        '<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="0.0634 -0.0070 -0.0028" euler="0 0 180" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="vis_tibia_L2" type="mesh" mesh="mesh_left_tibia" pos="{pos_str}" euler="0 0 180" material="mat_leg_dark" contype="0" conaffinity="0" group="1"/>'
    ).replace(
        '<geom name="vis_shield_L2" type="mesh" mesh="mesh_left_shield" pos="0.0634 -0.0070 -0.0028" euler="0 0 180" material="mat_armor" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="vis_shield_L2" type="mesh" mesh="mesh_left_shield" pos="{pos_str}" euler="0 0 180" material="mat_armor" contype="0" conaffinity="0" group="1"/>'
    )
    with open('models/test_knee_compare.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    m = mujoco.MjModel.from_xml_path('models/test_knee_compare.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)

    bid = m.body('tibia_L2').id
    knee_lookat = d.xpos[bid].copy()

    renderer = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    # Row 0: Top View
    cam.lookat = knee_lookat
    cam.distance = 0.12
    cam.azimuth = 90
    cam.elevation = -89
    renderer.update_scene(d, cam)
    axs[0, idx].imshow(renderer.render())
    axs[0, idx].set_title(f'{title}\nTop View', fontsize=11, fontweight='bold')
    axs[0, idx].axis('off')

    # Row 1: Side View (from front of robot)
    cam.distance = 0.12
    cam.azimuth = 0
    cam.elevation = 0
    renderer.update_scene(d, cam)
    axs[1, idx].imshow(renderer.render())
    axs[1, idx].set_title(f'{title}\nSide View', fontsize=11, fontweight='bold')
    axs[1, idx].axis('off')

    # Row 2: Perspective 3D
    cam.distance = 0.15
    cam.azimuth = 135
    cam.elevation = -25
    renderer.update_scene(d, cam)
    axs[2, idx].imshow(renderer.render())
    axs[2, idx].set_title(f'{title}\nPerspective', fontsize=11, fontweight='bold')
    axs[2, idx].axis('off')

    renderer.close()

plt.tight_layout()
plt.savefig('knee_pos_comparison.png', dpi=150)
print('knee_pos_comparison.png saved!')

import os
if os.path.exists('models/test_knee_compare.xml'):
    os.remove('models/test_knee_compare.xml')
