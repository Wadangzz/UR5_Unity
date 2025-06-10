import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

start_point = [0,0,0]
direction = [0,0,55]

rotation_matrix = R.from_euler('xyz',np.deg2rad([0,-59,93])).as_matrix()
end_point = start_point + rotation_matrix @ direction


print(end_point)
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')

# 좌표축 설정
ax.quiver(start_point[0], start_point[1], start_point[2],
          end_point[0], end_point[1], end_point[2],
          color='blue', linewidth=2, arrow_length_ratio=0.05)

# 시작점과 끝점 표시
ax.scatter(*start_point, color='green', s=50, label='Start')
ax.scatter(*end_point, color='red', s=50, label='End')

ax.set_xlim([-60, 60])
ax.set_ylim([-60, 60])
ax.set_zlim([0, 60])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Vector after ZYX Rotation (Yaw=45°, Pitch=45°, Roll=0°)')
ax.legend()
ax.view_init(elev=20, azim=-60)

plt.tight_layout()
plt.show()
