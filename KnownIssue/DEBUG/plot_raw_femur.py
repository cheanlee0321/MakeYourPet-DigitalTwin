import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def load_stl(path):
    with open(path, 'rb') as f:
        f.read(84)
        data = np.fromfile(f, dtype=[('normal', 'f4', (3,)), ('v1', 'f4', (3,)), ('v2', 'f4', (3,)), ('v3', 'f4', (3,)), ('attr', 'u2')])
    return data

data = load_stl('MakeYourPet-hexapod/hexapod-main/STL/left-femur.stl')
# Sample triangles to plot
v1, v2, v3 = data['v1'], data['v2'], data['v3']
v = np.vstack([v1, v2, v3])

fig = plt.figure(figsize=(18, 6))

# Plot projections
ax1 = fig.add_subplot(1, 3, 1)
ax1.scatter(v[::5, 0], v[::5, 1], s=0.5, c='goldenrod', alpha=0.5)
ax1.set_title('Raw Femur STL: X-Y projection (Top View)', fontsize=12, fontweight='bold')
ax1.set_xlabel('X (mm)')
ax1.set_ylabel('Y (mm)')
ax1.axis('equal')
ax1.grid(True)

ax2 = fig.add_subplot(1, 3, 2)
ax2.scatter(v[::5, 0], v[::5, 2], s=0.5, c='royalblue', alpha=0.5)
ax2.set_title('Raw Femur STL: X-Z projection (Side View)', fontsize=12, fontweight='bold')
ax2.set_xlabel('X (mm)')
ax2.set_ylabel('Z (mm)')
ax2.axis('equal')
ax2.grid(True)

ax3 = fig.add_subplot(1, 3, 3)
ax3.scatter(v[::5, 1], v[::5, 2], s=0.5, c='forestgreen', alpha=0.5)
ax3.set_title('Raw Femur STL: Y-Z projection (Front View)', fontsize=12, fontweight='bold')
ax3.set_xlabel('Y (mm)')
ax3.set_ylabel('Z (mm)')
ax3.axis('equal')
ax3.grid(True)

plt.tight_layout()
plt.savefig('raw_femur_projections.png', dpi=150)
print('raw_femur_projections.png saved!')
