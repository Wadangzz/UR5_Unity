"""
경유점 순회 + 동역학 제어 + 실제 URDF 메시 (multi_waypoint_demo 의 메시 버전).

home→A→B→C→home 을 Computed Torque(forward_dynamics plant)로 순회한 θ(t)를
실제 UR5 메시로 렌더. 경유점(별)과 말단 궤적도 함께 표시.

실행:  python urdf_multi_waypoint.py   (+ outputs/urdf_multi_waypoint.gif)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.integrate import solve_ivp
from robot_descriptions.loaders.yourdfpy import load_robot_description
from robot_math import Dynamics, quintic_time_scaling
import ur5_model as ur5
from paths import out

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)
robot = load_robot_description("ur5_description")

# 말단 링크 자동 탐지 (URDF 기준 TCP)
TCP = next(c for c in ("tool0", "ee_link", "wrist_3_link") if c in robot.link_map)


def tcp_pos(q):
    robot.update_cfg(q)
    return robot.scene.graph.get(TCP)[0][:3, 3]


def gravity(th):
    return Dynamics.gravity_forces(th, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


# --- 경유점(관절) : home → A → B → C → home ---
WP = [np.zeros(6),
      np.array([0.9, -0.6, 0.5, 0.0, 0.3, 0.0]),
      np.array([-0.8, -0.5, 0.8, 0.2, -0.4, 0.0]),
      np.array([0.3, -1.0, 1.4, -0.3, 0.6, 0.0]),
      np.zeros(6)]
N_SEG = len(WP) - 1
T_SEG = 1.2
KP, KD = 100.0, 20.0


def reference(t):
    seg = min(int(t // T_SEG), N_SEG - 1)
    q0, q1 = WP[seg], WP[seg + 1]
    s, sd, sdd = quintic_time_scaling(min(t - seg * T_SEG, T_SEG), T_SEG)
    dq = q1 - q0
    return q0 + s * dq, sd * dq, sdd * dq


def ct(t, th, dth):                              # Computed Torque (동역학 제어)
    th_d, dth_d, ddth_d = reference(t)
    aq = ddth_d + KD * (dth_d - dth) + KP * (th_d - th)
    M = Dynamics.mass_matrix(th, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    c = Dynamics.coriolis_forces(th, dth, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    return M @ aq + c + gravity(th)


def rhs(t, x):
    th, dth = x[:6], x[6:]
    return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, ct(t, th, dth), *ARGS)])


T_TOTAL = N_SEG * T_SEG
sol = solve_ivp(rhs, (0, T_TOTAL), np.zeros(12),
                t_eval=np.linspace(0, T_TOTAL, 40), method="RK45", rtol=1e-6, atol=1e-9)
THETA = sol.y[:6].T
STARS = np.array([tcp_pos(q) for q in WP[1:4]])   # A,B,C 의 URDF 말단 위치
print(f"총 {len(THETA)} 프레임 (TCP 링크={TCP})")

# --- 스타일 (조명 셰이딩) ---
LIGHT = np.array([0.35, 0.35, 1.0]); LIGHT /= np.linalg.norm(LIGHT)
BASE = np.array(mcolors.to_rgb("#4f86c6"))


def shaded(m):
    sh = 0.35 + 0.65 * np.clip(m.face_normals @ LIGHT, 0, 1)
    return BASE[None, :] * sh[:, None]


fig = plt.figure(figsize=(7, 8), facecolor="white")
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=16, azim=52)
ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(-0.2, 1.0)
ax.set_box_aspect((1, 1, 0.85))
for p in (ax.xaxis, ax.yaxis, ax.zaxis):
    p.pane.set_facecolor((0.97, 0.97, 0.98, 1.0)); p.pane.set_edgecolor((0.9, 0.9, 0.9, 1.0))
ax.grid(False); ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
ax.set_title("UR5 multi-waypoint  (Computed Torque, real mesh)", fontsize=12, weight="bold")

# 경유점 별 + 라벨
for name, pt in zip("ABC", STARS):
    ax.scatter(*pt, color="#e8743b", marker="*", s=260, depthshade=False,
               edgecolors="k", linewidths=0.4, zorder=12)
    ax.text(pt[0], pt[1], pt[2] + 0.06, name, fontsize=12, weight="bold", color="#c0392b")

trace, = ax.plot([], [], [], "-", color="#e8743b", lw=1.6, alpha=0.8)
holder = [None]
hist = []


def render(i):
    if holder[0] is not None:
        holder[0].remove()
    robot.update_cfg(THETA[i])
    m = robot.scene.to_geometry()
    holder[0] = Poly3DCollection(m.vertices[m.faces], facecolors=shaded(m), edgecolor="none")
    ax.add_collection3d(holder[0])
    hist.append(robot.scene.graph.get(TCP)[0][:3, 3])
    H = np.array(hist)
    trace.set_data(H[:, 0], H[:, 1]); trace.set_3d_properties(H[:, 2])
    return [holder[0], trace]


anim = FuncAnimation(fig, render, frames=len(THETA), interval=80, blit=False)
anim.save(out("urdf_multi_waypoint.gif"), writer=PillowWriter(fps=13), dpi=72)
print("[저장] outputs/urdf_multi_waypoint.gif")
plt.show()
