"""
URDF 메시 기반 로봇 시각화 (범용).

robot_descriptions 의 어떤 로봇이든 실제 visual 메시로 그린다 → UR5뿐 아니라
Franka, KUKA, 커스텀 URDF 등 그대로 적용 가능. 메시는 matplotlib(Poly3DCollection)
로 렌더하므로 OpenGL 없이 헤드리스에서도 PNG 저장 가능.

실행:  python urdf_view.py [robot_description_name]   (기본 ur5_description)
예:    python urdf_view.py panda_description
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_descriptions.loaders.yourdfpy import load_robot_description
from paths import out

name = sys.argv[1] if len(sys.argv) > 1 else "ur5_description"
robot = load_robot_description(name)

# 움직이는 관절에 적당한 자세 부여 (없으면 home)
movable = [j for j in robot.robot.joints if j.type != "fixed"]
n = len(movable)
pose = np.array([0.3, -0.9, 1.2, -0.4, 0.6, 0.2])
cfg = pose[:n] if n <= 6 else np.concatenate([pose, np.zeros(n - 6)])
try:
    robot.update_cfg(cfg[:n])
except Exception as e:
    print("[warn] cfg 적용 실패, home 사용:", e)

# 현재 자세의 월드 좌표 메시 (모든 링크 합치기)
mesh = robot.scene.to_geometry()
V, F = mesh.vertices, mesh.faces
print(f"{name}: 링크 {len(robot.link_map)}개, 관절 {n}개, 삼각형 {len(F)}개")

fig = plt.figure(figsize=(7, 8))
ax = fig.add_subplot(111, projection="3d")
ax.add_collection3d(Poly3DCollection(
    V[F], alpha=0.95, facecolor="#8aa9d6", edgecolor="k", linewidths=0.03))

ctr = V.mean(axis=0)
r = (V.max(axis=0) - V.min(axis=0)).max() / 2 * 1.1
ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r)
ax.set_zlim(V[:, 2].min(), V[:, 2].min() + 2 * r)
ax.set_box_aspect((1, 1, 1))
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title(f"URDF mesh: {name}")
ax.view_init(elev=15, azim=50)
plt.tight_layout()
plt.savefig(out(f"urdf_{name}.png"), dpi=95)
print(f"[저장] outputs/urdf_{name}.png")
plt.show()
