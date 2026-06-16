"""
UR5 제어 결과 3D 애니메이션 (matplotlib).

control_demo.py 가 저장한 pd_trajectory.npz (PD+중력보상 궤적)를 읽어,
fk_skeleton 로 각 관절 위치를 계산해 팔을 막대로 그려 움직임을 보여준다.
목표 자세는 회색 고스트로 함께 표시 → 수렴 과정을 눈으로 확인.

실행:  python animate.py          (대화형 창으로 재생)
       ur5_control.gif 도 함께 저장됨.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import ur5_model as ur5
from paths import out

# --- 궤적 로드 ---
data = np.load(out("pd_trajectory.npz"))
t, theta, theta_d = data["t"], data["theta"], data["theta_d"]

# 프레임별 관절 위치 (F, 8, 3) 및 목표 자세
frames = np.array([ur5.fk_skeleton(th) for th in theta])
target = ur5.fk_skeleton(theta_d)

# --- 3D 플롯 셋업 ---
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=45)

center = frames.reshape(-1, 3).mean(axis=0)
r = 0.55
ax.set_xlim(center[0] - r, center[0] + r)
ax.set_ylim(center[1] - r, center[1] + r)
ax.set_zlim(-0.15, 1.05)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title("UR5: PD + Gravity Compensation")

# 목표 자세 고스트
ax.plot(target[:, 0], target[:, 1], target[:, 2],
        "--o", color="lightgray", lw=2, ms=4, label="target")

# 움직이는 팔 + 말단 궤적
arm, = ax.plot([], [], [], "-o", color="tab:blue", lw=4, ms=6, label="UR5")
trace, = ax.plot([], [], [], "-", color="tab:red", lw=1.2, alpha=0.7)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
ax.legend(loc="upper right")

ee_hist = []


def update(i):
    P = frames[i]
    arm.set_data(P[:, 0], P[:, 1]); arm.set_3d_properties(P[:, 2])
    ee_hist.append(P[-1])
    H = np.array(ee_hist)
    trace.set_data(H[:, 0], H[:, 1]); trace.set_3d_properties(H[:, 2])
    txt.set_text(f"t = {t[i]:.2f} s")
    return arm, trace, txt


anim = FuncAnimation(fig, update, frames=len(frames), interval=40, blit=False)

anim.save(out("ur5_control.gif"), writer=PillowWriter(fps=25))
print(f"[저장] ur5_control.gif  ({len(frames)} 프레임)")

plt.show()
