import mujoco
import matplotlib.pyplot as plt

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml = f.read()

# Apply calibrated tibia and tip positions to all legs:
# Old tibia: pos="0.0634 -0.0070 -0.0028" (left) / pos="0.0634 0.0070 -0.0028" (right)
# New tibia: pos="0.0490 -0.0070  0.0065" (left) / pos="0.0490 0.0070  0.0065" (right)
# Old tip:   pos="0.0621 -0.0025 -0.1170" (left) / pos="0.0621 0.0025 -0.1170" (right)
# New tip:   pos="0.0477 -0.0025 -0.1077" (left) / pos="0.0477 0.0025 -0.1077" (right)
# (Delta tip: -14.4mm X, +9.3mm Z)

test_xml = xml
for leg in ['L1', 'L2', 'L3']:
    test_xml = test_xml.replace(
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_left_tibia" pos="0.0634 -0.0070 -0.0028"',
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_left_tibia" pos="0.0490 -0.0070 0.0065"'
    ).replace(
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_left_shield" pos="0.0634 -0.0070 -0.0028"',
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_left_shield" pos="0.0490 -0.0070 0.0065"'
    ).replace(
        f'<geom name="col_tibia_{leg}" type="capsule" fromto="0 0 0 0.0621 -0.0025 -0.1170"',
        f'<geom name="col_tibia_{leg}" type="capsule" fromto="0 0 0 0.0477 -0.0025 -0.1077"'
    ).replace(
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0621 -0.0025 -0.1170"',
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0477 -0.0025 -0.1077"'
    ).replace(
        f'<geom name="tip_{leg}" type="sphere" pos="0.0621 -0.0025 -0.1170"',
        f'<geom name="tip_{leg}" type="sphere" pos="0.0477 -0.0025 -0.1077"'
    )

for leg in ['R1', 'R2', 'R3']:
    test_xml = test_xml.replace(
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_right_tibia" pos="0.0634  0.0070 -0.0028"',
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_right_tibia" pos="0.0490  0.0070 0.0065"'
    ).replace(
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_right_shield" pos="0.0634  0.0070 -0.0028"',
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_right_shield" pos="0.0490  0.0070 0.0065"'
    ).replace(
        f'<geom name="col_tibia_{leg}" type="capsule" fromto="0 0 0 0.0621  0.0025 -0.1170"',
        f'<geom name="col_tibia_{leg}" type="capsule" fromto="0 0 0 0.0477  0.0025 -0.1077"'
    ).replace(
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0621  0.0025 -0.1170"',
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0477  0.0025 -0.1077"'
    ).replace(
        f'<geom name="tip_{leg}" type="sphere" pos="0.0621  0.0025 -0.1170"',
        f'<geom name="tip_{leg}" type="sphere" pos="0.0477  0.0025 -0.1077"'
    )

with open('models/test_full_calibration.xml', 'w', encoding='utf-8') as tf:
    tf.write(test_xml)

m = mujoco.MjModel.from_xml_path('models/test_full_calibration.xml')
d = mujoco.MjData(m)
mujoco.mj_step(m, d)

fig, axs = plt.subplots(2, 2, figsize=(16, 12))

renderer = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE

# (0, 0): L2 Knee Joint Closeup
bid_l2 = m.body('tibia_L2').id
cam.lookat = d.xpos[bid_l2].copy()
cam.distance = 0.12
cam.azimuth = 0
cam.elevation = 0
renderer.update_scene(d, cam)
axs[0, 0].imshow(renderer.render())
axs[0, 0].set_title('Left Leg (L2) Knee Joint (Side View)', fontsize=12, fontweight='bold')
axs[0, 0].axis('off')

# (0, 1): R2 Knee Joint Closeup
bid_r2 = m.body('tibia_R2').id
cam.lookat = d.xpos[bid_r2].copy()
cam.distance = 0.12
cam.azimuth = 180
cam.elevation = 0
renderer.update_scene(d, cam)
axs[0, 1].imshow(renderer.render())
axs[0, 1].set_title('Right Leg (R2) Knee Joint (Side View)', fontsize=12, fontweight='bold')
axs[0, 1].axis('off')

# (1, 0): L2 Full Leg Perspective
cam.lookat = d.xpos[bid_l2].copy()
cam.distance = 0.22
cam.azimuth = 135
cam.elevation = -25
renderer.update_scene(d, cam)
axs[1, 0].imshow(renderer.render())
axs[1, 0].set_title('Left Leg (L2) Full Leg Assembly', fontsize=12, fontweight='bold')
axs[1, 0].axis('off')

# (1, 1): R2 Full Leg Perspective
cam.lookat = d.xpos[bid_r2].copy()
cam.distance = 0.22
cam.azimuth = -45
cam.elevation = -25
renderer.update_scene(d, cam)
axs[1, 1].imshow(renderer.render())
axs[1, 1].set_title('Right Leg (R2) Full Leg Assembly', fontsize=12, fontweight='bold')
axs[1, 1].axis('off')

plt.tight_layout()
plt.savefig('full_leg_calibration_test.png', dpi=150)
print('full_leg_calibration_test.png saved!')

import os
if os.path.exists('models/test_full_calibration.xml'):
    os.remove('models/test_full_calibration.xml')
