"""
PID 제어를 실제 UR5 메시로 보여주는 애니메이션 — PD vs PID 나란히.

둘 다 중력보상 없이 같은 목표 자세로:
  왼쪽  PD  (τ=Kp·e+Kd·ė)        → 중력에 처져 목표에 못 미침(steady-state error)
  오른쪽 PID (τ=Kp·e+Ki·∫e+Kd·ė) → I항이 중력 상쇄 → 목표에 정확히 도달

plant=forward_dynamics. 실제 메시 2대를 y로 벌려 렌더(조명 셰이딩).
실행:  python urdf_pid_anim.py   (+ outputs/urdf_pid.gif)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.integrate import solve_ivp
from robot_descriptions.loaders.yourdfpy import load_robot_description
from robot_math import Dynamics
import ur5_model as ur5
from paths import out

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)
robot = load_robot_description("ur5_description")
TGT = np.array([0.0, -np.pi / 3, 0.6, 0.0, 0.4, 0.0])
KP, KI, KD = 120.0, 60.0, 35.0
N, T_END = 30, 6.0


def sim_pd():
    def rhs(t, x):
        th, dth = x[:6], x[6:]
        tau = KP * (TGT - th) - KD * dth                 # PD (I 없음, 중력보상 없음)
        return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, tau, *ARGS)])
    return solve_ivp(rhs, (0, T_END), np.zeros(12), t_eval=np.linspace(0, T_END, N),
                     method="RK45", rtol=1e-6, atol=1e-9).y[:6].T


def sim_pid():
    def rhs(t, x):
        th, dth, I = x[:6], x[6:12], x[12:]
        tau = KP * (TGT - th) + KI * I - KD * dth         # PID
        return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, tau, *ARGS), TGT - th])
    return solve_ivp(rhs, (0, T_END), np.zeros(18), t_eval=np.linspace(0, T_END, N),
                     method="RK45", rtol=1e-6, atol=1e-9).y[:6].T


PD, PID = sim_pd(), sim_pid()
print(f"PD  최종 J2 = {np.rad2deg(PD[-1,1]):.1f}° (목표 {np.rad2deg(TGT[1]):.0f}° → 처짐)")
print(f"PID 최종 J2 = {np.rad2deg(PID[-1,1]):.1f}° (목표 도달)")

OFF_PD = np.array([0.0, -0.45, 0.0])
OFF_PID = np.array([0.0, 0.45, 0.0])
TCP = next(c for c in ("tool0", "ee_link", "wrist_3_link") if c in robot.link_map)
robot.update_cfg(TGT)
star = robot.scene.graph.get(TCP)[0][:3, 3]               # 목표 말단 위치

LIGHT = np.array([0.35, 0.35, 1.0]); LIGHT /= np.linalg.norm(LIGHT)


def shaded(m, base):
    sh = 0.35 + 0.65 * np.clip(m.face_normals @ LIGHT, 0, 1)
    return np.array(mcolors.to_rgb(base))[None, :] * sh[:, None]


fig = plt.figure(figsize=(9, 7), facecolor="white")
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=15, azim=60)
ax.set_xlim(-0.6, 0.6); ax.set_ylim(-1.0, 1.0); ax.set_zlim(-0.2, 1.0)
ax.set_box_aspect((0.6, 1.0, 0.6))
for p in (ax.xaxis, ax.yaxis, ax.zaxis):
    p.pane.set_facecolor((0.97, 0.97, 0.98, 1.0)); p.pane.set_edgecolor((0.9, 0.9, 0.9, 1.0))
ax.grid(False); ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
ax.set_title("PD vs PID  (no gravity comp) — same target", fontsize=13, weight="bold")

# 목표 말단 위치(별) + 라벨
for off in (OFF_PD, OFF_PID):
    ax.scatter(*(star + off), marker="*", s=240, color="#e8743b",
               edgecolors="k", linewidths=0.4, depthshade=False, zorder=12)
ax.text(0.0, -0.45, 0.95, "PD\n(sags)", fontsize=12, weight="bold", color="#c0392b", ha="center")
ax.text(0.0, 0.45, 0.95, "PID\n(reaches)", fontsize=12, weight="bold", color="#1f6fb2", ha="center")
holders = [None, None]


def render(i):
    for h in holders:
        if h is not None:
            h.remove()
    for k, (traj, off, base) in enumerate([(PD, OFF_PD, "#c0504d"), (PID, OFF_PID, "#4f86c6")]):
        robot.update_cfg(traj[i])
        m = robot.scene.to_geometry()
        coll = Poly3DCollection((m.vertices + off)[m.faces], facecolors=shaded(m, base), edgecolor="none")
        holders[k] = coll
        ax.add_collection3d(coll)
    return holders


anim = FuncAnimation(fig, render, frames=N, interval=80, blit=False)
anim.save(out("urdf_pid.gif"), writer=PillowWriter(fps=12), dpi=72)
print("[저장] outputs/urdf_pid.gif")
plt.show()
