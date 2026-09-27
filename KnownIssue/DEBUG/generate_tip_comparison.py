import mujoco
import matplotlib.pyplot as plt
import numpy as np

# Load current calibrated model
model_calibrated = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data_calibrated = mujoco.MjData(model_calibrated)
mujoco.mj_step(model_calibrated, data_calibrated)

# Create misaligned model for side-by-side comparison
with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml_str = f.read()

# Replace tip positions back to old misaligned pos="0.0575 0 -0.1137"
xml_misaligned = xml_str
for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
    xml_misaligned = xml_misaligned.replace(
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0621 -0.0025 -0.1170"',
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0575 0 -0.1137"'
    ).replace(
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0621  0.0025 -0.1170"',
        f'<geom name="vis_tip_{leg}" type="mesh" mesh="mesh_tip" pos="0.0575 0 -0.1137"'
    ).replace(
        f'<geom name="tip_{leg}" type="sphere" pos="0.0621 -0.0025 -0.1170"',
        f'<geom name="tip_{leg}" type="sphere" pos="0.0575 0 -0.1137"'
    ).replace(
        f'<geom name="tip_{leg}" type="sphere" pos="0.0621  0.0025 -0.1170"',
        f'<geom name="tip_{leg}" type="sphere" pos="0.0575 0 -0.1137"'
    )

with open('models/temp_misaligned.xml', 'w', encoding='utf-8') as f:
    f.write(xml_misaligned)

model_misaligned = mujoco.MjModel.from_xml_path('models/temp_misaligned.xml')
data_misaligned = mujoco.MjData(model_misaligned)
mujoco.mj_step(model_misaligned, data_misaligned)

fig, axs = plt.subplots(2, 2, figsize=(14, 12))

# Helper to render closeup
def render_tip(model, data, lookat, distance, azimuth, elevation):
    renderer = mujoco.Renderer(model, 480, 640)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat = lookat
    camera.distance = distance
    camera.elevation = elevation
    camera.azimuth = azimuth
    renderer.update_scene(data, camera)
    img = renderer.render()
    renderer.close()
    return img

# (0, 0) Left Leg (L2) - Misaligned
img_l2_before = render_tip(model_misaligned, data_misaligned, [0, 0.24, -0.015], 0.12, 145, 0)
axs[0, 0].imshow(img_l2_before)
axs[0, 0].set_title('Left Leg (L2) - BEFORE (Misaligned)\npos="0.0575 0 -0.1137" (Offset +4.6mm X, 2.5mm Y)', fontsize=12, color='red', fontweight='bold')
axs[0, 0].axis('off')

# (0, 1) Left Leg (L2) - Calibrated
img_l2_after = render_tip(model_calibrated, data_calibrated, [0, 0.24, -0.015], 0.12, 145, 0)
axs[0, 1].imshow(img_l2_after)
axs[0, 1].set_title('Left Leg (L2) - AFTER (Calibrated)\npos="0.0621 -0.0025 -0.1170" (Concentric Aligned)', fontsize=12, color='green', fontweight='bold')
axs[0, 1].axis('off')

# (1, 0) Right Leg (R2) - Misaligned
img_r2_before = render_tip(model_misaligned, data_misaligned, [0, -0.24, -0.015], 0.12, -35, 0)
axs[1, 0].imshow(img_r2_before)
axs[1, 0].set_title('Right Leg (R2) - BEFORE (Misaligned)\npos="0.0575 0 -0.1137" (Detached & Off-axis)', fontsize=12, color='red', fontweight='bold')
axs[1, 0].axis('off')

# (1, 1) Right Leg (R2) - Calibrated
img_r2_after = render_tip(model_calibrated, data_calibrated, [0, -0.24, -0.015], 0.12, -35, 0)
axs[1, 1].imshow(img_r2_after)
axs[1, 1].set_title('Right Leg (R2) - AFTER (Calibrated)\npos="0.0621  0.0025 -0.1170" (Concentric Aligned)', fontsize=12, color='green', fontweight='bold')
axs[1, 1].axis('off')

plt.tight_layout()
plt.savefig('foot_tip_calibration_comparison.png', dpi=150)
print('foot_tip_calibration_comparison.png saved!')

import os
if os.path.exists('models/temp_misaligned.xml'):
    os.remove('models/temp_misaligned.xml')
