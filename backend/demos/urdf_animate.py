"""
실제 UR5 URDF 메시로 계획 궤적을 따라 움직이는 애니메이션.

경유점(home→A→B→C→home)을 quintic 으로 연결한 관절 궤적을 실제 UR5 메시에
먹여 렌더 → "진짜 로봇이 움직이는" gif. (관절축은 우리 ur5_model 과 같은 순서,
베이스 방향만 URDF 고유)

실행:  python urdf_animate.py   (+ outputs/urdf_motion.gif)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_descriptions.loaders.yourdfpy import load_robot_description
from app.core.robot_math import quintic_time_scaling
from app.core.paths import out

robot = load_robot_description("ur5_description")

# 관절 경유점 + quintic 연결
WP = [np.zeros(6),
      np.array([0.9, -0.6, 0.5, 0.0, 0.3, 0.0]),
      np.array([-0.8, -0.5, 0.8, 0.2, -0.4, 0.0]),
      np.array([0.3, -1.0, 1.4, -0.3, 0.6, 0.0]),
      np.zeros(6)]
PTS = 8                                   # 구간당 프레임
traj = []
for k in range(len(WP) - 1):
    dq = WP[k + 1] - WP[k]
    for j in range(PTS):
        s, _, _ = quintic_time_scaling(j / PTS, 1.0)
        traj.append(WP[k] + s * dq)
traj = np.array(traj)
print(f"총 {len(traj)} 프레임 렌더링...")

fig = plt.figure(figsize=(7, 8))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=15, azim=50)
ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0, 1.0)
ax.set_box_aspect((1, 1, 1))
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title("UR5 (real URDF mesh) following planned trajectory")
holder = [None]


def render(i):
    if holder[0] is not None:
        holder[0].remove()
    robot.update_cfg(traj[i])
    m = robot.scene.to_geometry()
    coll = Poly3DCollection(m.vertices[m.faces], alpha=1.0,
                            facecolor="#8aa9d6", edgecolor="none")
    holder[0] = coll
    ax.add_collection3d(coll)
    return [coll]


anim = FuncAnimation(fig, render, frames=len(traj), interval=70, blit=False)
anim.save(out("urdf_motion.gif"), writer=PillowWriter(fps=14), dpi=70)
print("[저장] outputs/urdf_motion.gif")
plt.show()
