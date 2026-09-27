import mujoco
import matplotlib.pyplot as plt

with open('models/hexapod.xml', 'r', encoding='utf-8') as f:
    base_xml = f.read()

candidates = [
    ('Current (Misaligned)', '0.0575 0 -0.1137'),
    ('Calibrated Z=-0.114', '0.0621 -0.0025 -0.1140'),
    ('Calibrated Z=-0.117', '0.0621 -0.0025 -0.1170'),
    ('Calibrated Z=-0.120', '0.0621 -0.0025 -0.1200'),
]

fig, axs = plt.subplots(1, 4, figsize=(18, 5))

for idx, (title, pos_str) in enumerate(candidates):
    test_xml = base_xml.replace(
        '<geom name="vis_tip_L2" type="mesh" mesh="mesh_tip" pos="0.0575 0 -0.1137" material="mat_rubber" contype="0" conaffinity="0" group="1"/>',
        f'<geom name="vis_tip_L2" type="mesh" mesh="mesh_tip" pos="{pos_str}" material="mat_rubber" contype="0" conaffinity="0" group="1"/>'
    )
    test_xml = test_xml.replace(
        '<geom name="tip_L2" type="sphere" pos="0.0575 0 -0.1137"',
        f'<geom name="tip_L2" type="sphere" pos="{pos_str}"'
    )

    with open('models/test_tip.xml', 'w', encoding='utf-8') as tf:
        tf.write(test_xml)

    model = mujoco.MjModel.from_xml_path('models/test_tip.xml')
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)

    renderer = mujoco.Renderer(model, 480, 640)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat = [0, 0.24, -0.015]
    camera.distance = 0.12
    camera.elevation = 0
    camera.azimuth = 145
    renderer.update_scene(data, camera)
    axs[idx].imshow(renderer.render())
    axs[idx].set_title(title, fontsize=12, fontweight='bold')
    axs[idx].axis('off')
    renderer.close()

plt.tight_layout()
plt.savefig('foot_tip_candidates.png', dpi=150)
print('Saved foot_tip_candidates.png successfully!')
