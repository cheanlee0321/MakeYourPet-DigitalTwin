import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import trimesh

# Load STLs
femur = trimesh.load('MakeYourPet-hexapod/hexapod-main/STL/left-femur.stl')
tibia = trimesh.load('MakeYourPet-hexapod/hexapod-main/STL/left-tibia.stl')

# In femur:
# Proximal hole: [3.0, 1.0, -11.0] mm
# Distal hole:   [83.0, 1.0, -11.0] mm

# In tibia:
# Knee pivot:    [63.2, -6.25, 3.76] mm
# Tip hole:      [1.33, -4.50, -110.95] mm

# Let's plot both in 3D in their respective local frames
fig = plt.figure(figsize=(14, 7))

ax1 = fig.add_subplot(1, 2, 1, projection='3d')
vf = femur.vertices
ax1.scatter(vf[::10, 0], vf[::10, 1], vf[::10, 2], s=0.2, c='goldenrod', alpha=0.3)
ax1.scatter([3.0, 83.0], [1.0, 1.0], [-11.0, -11.0], s=50, c='red', marker='o')
ax1.text(3.0, 1.0, -11.0, ' Proximal (Coxa)', color='red', fontweight='bold')
ax1.text(83.0, 1.0, -11.0, ' Distal (Knee)', color='red', fontweight='bold')
ax1.set_title('Raw Femur STL with Holes', fontsize=12, fontweight='bold')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

ax2 = fig.add_subplot(1, 2, 2, projection='3d')
vt = tibia.vertices
ax2.scatter(vt[::10, 0], vt[::10, 1], vt[::10, 2], s=0.2, c='darkslategray', alpha=0.3)
ax2.scatter([63.2], [-6.25], [3.76], s=50, c='blue', marker='o')
ax2.scatter([1.33], [-4.50], [-110.95], s=50, c='magenta', marker='o')
ax2.text(63.2, -6.25, 3.76, ' Knee Pivot', color='blue', fontweight='bold')
ax2.text(1.33, -4.50, -110.95, ' Foot Tip', color='magenta', fontweight='bold')
ax2.set_title('Raw Tibia STL with Pivot & Tip', fontsize=12, fontweight='bold')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.tight_layout()
plt.savefig('stls_with_annotations.png', dpi=150)
print('stls_with_annotations.png saved!')
