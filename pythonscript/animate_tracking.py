"""
궤적 추종 3D 애니메이션 — Computed Torque vs PD+중력보상.

tracking_demo.py 가 저장한 tracking_trajectory.npz 를 읽어,
  · 회색 점선 = 기준궤적 (움직이는 목표)
  · 파란 = Computed Torque (기준에 딱 붙음)
  · 주황 = PD + 중력보상 (빠른 구간에서 뒤처짐)
을 동시에 그려 추종 성능 차이를 눈으로 보여준다.

실행:  python animate_tracking.py   (+ tracking.gif 저장)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import ur5_model as ur5
from paths import out

d = np.load(out("tracking_trajectory.npz"))
t, ref, ct, pd = d["t"], d["theta_ref"], d["theta_ct"], d["theta_pd"]

P_ref = np.array([ur5.fk_all_joints(q) for q in ref])
P_ct = np.array([ur5.fk_all_joints(q) for q in ct])
P_pd = np.array([ur5.fk_all_joints(q) for q in pd])

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=50)
c = P_ref.reshape(-1, 3).mean(axis=0)
r = 0.6
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(-0.15, 1.05)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title("UR5 Trajectory Tracking:  Computed Torque vs PD")

ref_l, = ax.plot([], [], [], "--o", color="lightgray", lw=2, ms=4, label="reference")
ct_l,  = ax.plot([], [], [], "-o", color="tab:blue", lw=4, ms=6, label="Computed Torque")
pd_l,  = ax.plot([], [], [], "-o", color="tab:orange", lw=3, ms=5, alpha=0.8, label="PD + gravity")
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
ax.legend(loc="upper right")


def update(i):
    for line, P in ((ref_l, P_ref[i]), (ct_l, P_ct[i]), (pd_l, P_pd[i])):
        line.set_data(P[:, 0], P[:, 1]); line.set_3d_properties(P[:, 2])
    txt.set_text(f"t = {t[i]:.2f} s")
    return ref_l, ct_l, pd_l, txt


# 용량 절감을 위해 프레임 솎기(매 3번째) — 재생은 충분히 부드러움
frame_idx = range(0, len(t), 3)
anim = FuncAnimation(fig, update, frames=frame_idx, interval=40, blit=False)
anim.save(out("tracking.gif"), writer=PillowWriter(fps=25), dpi=72)
print(f"[저장] tracking.gif  ({len(list(frame_idx))} 프레임)")
plt.show()
