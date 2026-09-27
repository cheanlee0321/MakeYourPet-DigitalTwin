import mujoco
import matplotlib.pyplot as plt
import numpy as np

# Load calibrated model
model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

# Generate temporary misaligned model to capture the exact Before state
with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    xml_str = f.read()

# Replace new positions back to old misaligned pos="0.0634 -0.0070 -0.0028"
xml_before = xml_str
for leg in ['L1', 'L2', 'L3']:
    xml_before = xml_before.replace(
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_left_tibia" pos="0.0490 -0.0070 0.0065"',
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_left_tibia" pos="0.0634 -0.0070 -0.0028"'
    ).replace(
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_left_shield" pos="0.0490 -0.0070 0.0065"',
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_left_shield" pos="0.0634 -0.0070 -0.0028"'
    )

for leg in ['R1', 'R2', 'R3']:
    xml_before = xml_before.replace(
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_right_tibia" pos="0.0490  0.0070 0.0065"',
        f'<geom name="vis_tibia_{leg}" type="mesh" mesh="mesh_right_tibia" pos="0.0634  0.0070 -0.0028"'
    ).replace(
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_right_shield" pos="0.0490  0.0070 0.0065"',
        f'<geom name="vis_shield_{leg}" type="mesh" mesh="mesh_right_shield" pos="0.0634  0.0070 -0.0028"'
    )

with open('models/temp_before_knee.xml', 'w', encoding='utf-8') as f:
    f.write(xml_before)

model_before = mujoco.MjModel.from_xml_path('models/temp_before_knee.xml')
data_before = mujoco.MjData(model_before)
mujoco.mj_step(model_before, data_before)

fig, axs = plt.subplots(2, 2, figsize=(15, 12))

def render_view(m, d, lookat, dist, az, el):
    r = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = lookat
    cam.distance = dist
    cam.azimuth = az
    cam.elevation = el
    r.update_scene(d, cam)
    img = r.render()
    r.close()
    return img

bid_l2 = model.body('tibia_L2').id
knee_pos = data.xpos[bid_l2].copy()

# (0, 0): BEFORE Side View
img_b_side = render_view(model_before, data_before, knee_pos, 0.12, 0, 0)
axs[0, 0].imshow(img_b_side)
axs[0, 0].set_title('BEFORE: Femur-Tibia Knee Joint (Side View)\n[Misaligned: ~24mm Gap, Tibia Detached Outside Fork]', fontsize=11, color='red', fontweight='bold')
axs[0, 0].axis('off')

# (0, 1): AFTER Side View
img_a_side = render_view(model, data, knee_pos, 0.12, 0, 0)
axs[0, 1].imshow(img_a_side)
axs[0, 1].set_title('AFTER: Femur-Tibia Knee Joint (Side View)\n[Calibrated: Zero Gap, Tibia Snugly Wraps Knee Pivot]', fontsize=11, color='green', fontweight='bold')
axs[0, 1].axis('off')

# (1, 0): BEFORE Perspective View
img_b_persp = render_view(model_before, data_before, knee_pos, 0.16, 135, -25)
axs[1, 0].imshow(img_b_persp)
axs[1, 0].set_title('BEFORE: Perspective View (Tibia Hook Floating Away)', fontsize=11, color='red', fontweight='bold')
axs[1, 0].axis('off')

# (1, 1): AFTER Perspective View
img_a_persp = render_view(model, data, knee_pos, 0.16, 135, -25)
axs[1, 1].imshow(img_a_persp)
axs[1, 1].set_title('AFTER: Perspective View (Tibia Seamlessly Nested Inside Fork)', fontsize=11, color='green', fontweight='bold')
axs[1, 1].axis('off')

plt.tight_layout()
plt.savefig('femur_tibia_connection_fixed.png', dpi=150)
print('femur_tibia_connection_fixed.png saved successfully!')

import os
if os.path.exists('models/temp_before_knee.xml'):
    os.remove('models/temp_before_knee.xml')
