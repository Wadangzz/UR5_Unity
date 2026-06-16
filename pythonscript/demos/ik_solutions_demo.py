"""
UR5 다중 IK 해 데모 — 같은 말단 자세를 만드는 여러 관절 자세.

6R 로봇은 한 end-effector pose 에 대해 여러 IK 해(UR5 최대 8개)가 존재한다.
Kinematics.IK_solutions 로 그 해들을 모아, 모두 같은 목표(빨간 별)를 짚지만
팔 모양(elbow up/down, wrist/shoulder flip)이 다른 것을 한 화면에 그린다.

실행:  python ik_solutions_demo.py   (+ ik_solutions.png 저장)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
import matplotlib.pyplot as plt
from robot_math import Kinematics, SE3
import ur5_model as ur5
from paths import out

np.set_printoptions(precision=1, suppress=True)

# --- 목표 말단 자세 (reachable 한 config 의 FK 로 정의) ---
theta_ref = np.array([0.6, -0.4, 0.9, -0.3, 0.5, 0.2])
T_des = ur5.fk(theta_ref)
p_des = T_des[:3, 3]
desired = p_des.tolist() + SE3.orientation(T_des)[0]      # [x,y,z, quaternion]

# --- 여러 IK 해 탐색 ---
sols = Kinematics.IK_solutions(ur5.ik_robot, desired)
print(f"목표 말단 위치 [m] = {p_des}")
print(f"발견된 서로 다른 IK 해: {len(sols)}개\n")
for i, sol in enumerate(sols):
    p = ur5.fk(np.deg2rad(sol))[:3, 3]
    print(f"  해{i+1}: {sol}  | 말단오차 {np.linalg.norm(p - p_des):.1e} m")

# --- 3D 시각화: 모든 해를 한 화면에 ---
arms = [ur5.fk_skeleton(np.deg2rad(sol)) for sol in sols]
allpts = np.concatenate(arms, axis=0)

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=48)
ctr = allpts.mean(axis=0)
r = 0.6
ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r); ax.set_zlim(-0.15, 1.05)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title(f"UR5 IK: {len(sols)} configurations reaching the SAME pose")

colors = plt.cm.tab10(np.linspace(0, 1, max(len(sols), 1)))
for i, (P, col) in enumerate(zip(arms, colors)):
    ax.plot(P[:, 0], P[:, 1], P[:, 2], "-o", color=col, lw=3, ms=5,
            alpha=0.9, label=f"solution {i+1}")

# 공통 목표 (빨간 별)
ax.scatter(*p_des, color="red", marker="*", s=320, depthshade=False,
           edgecolors="k", linewidths=0.5, label="target pose", zorder=10)
ax.legend(loc="upper right")
plt.tight_layout()
plt.savefig(out("ik_solutions.png"), dpi=95)
print("\n[저장] ik_solutions.png")
plt.show()
