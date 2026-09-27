import matplotlib.pyplot as plt
import numpy as np

def load_stl(path):
    with open(path, 'rb') as f:
        f.read(84)
        data = np.fromfile(f, dtype=[('normal', 'f4', (3,)), ('v1', 'f4', (3,)), ('v2', 'f4', (3,)), ('v3', 'f4', (3,)), ('attr', 'u2')])
    return data

data = load_stl('MakeYourPet-hexapod/hexapod-main/STL/left-tibia.stl')
v1, v2, v3 = data['v1'], data['v2'], data['v3']
v = np.vstack([v1, v2, v3])

fig = plt.figure(figsize=(18, 6))

ax1 = fig.add_subplot(1, 3, 1)
ax1.scatter(v[::5, 0], v[::5, 1], s=0.5, c='darkslategray', alpha=0.5)
ax1.set_title('Raw Tibia STL: X-Y projection', fontsize=12, fontweight='bold')
ax1.set_xlabel('X (mm)')
ax1.set_ylabel('Y (mm)')
ax1.axis('equal')
ax1.grid(True)

ax2 = fig.add_subplot(1, 3, 2)
ax2.scatter(v[::5, 0], v[::5, 2], s=0.5, c='crimson', alpha=0.5)
ax2.set_title('Raw Tibia STL: X-Z projection', fontsize=12, fontweight='bold')
ax2.set_xlabel('X (mm)')
ax2.set_ylabel('Z (mm)')
ax2.axis('equal')
ax2.grid(True)

ax3 = fig.add_subplot(1, 3, 3)
ax3.scatter(v[::5, 1], v[::5, 2], s=0.5, c='navy', alpha=0.5)
ax3.set_title('Raw Tibia STL: Y-Z projection', fontsize=12, fontweight='bold')
ax3.set_xlabel('Y (mm)')
ax3.set_ylabel('Z (mm)')
ax3.axis('equal')
ax3.grid(True)

plt.tight_layout()
plt.savefig('raw_tibia_projections.png', dpi=150)
print('raw_tibia_projections.png saved!')
