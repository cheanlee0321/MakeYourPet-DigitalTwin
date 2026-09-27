import mujoco
import matplotlib.pyplot as plt
import numpy as np

model = mujoco.MjModel.from_xml_path('models/hexapod.xml')
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

# Let's inspect the positions of all geoms in L2 leg
geoms = ['vis_coxa_L2', 'vis_femur_L2', 'vis_tibia_L2', 'vis_shield_L2', 'vis_tip_L2']
for g in geoms:
    gid = model.geom(g).id
    print(f"{g:15s} pos={data.geom_xpos[gid]} mat=\n{data.geom_xmat[gid].reshape(3,3).round(2)}")

# And bodies
bodies = ['mount_L2', 'coxa_L2', 'femur_L2', 'tibia_L2']
for b in bodies:
    bid = model.body(b).id
    print(f"Body {b:12s} pos={data.xpos[bid]}")
