"""
경유점 순회를 PID 제어 + 실제 URDF 메시로 (multi_waypoint 의 PID·메시 버전).

home→A→B→C→home 을 순수 PID(τ=Kp·e+Ki·∫e+Kd·ė, 중력보상 없음)로 추종한 θ(t)를
실제 UR5 메시로 렌더. 각 경유점에서 잠깐 멈춰(dwell) I항이 중력 처짐을 잡는다.
plant = forward_dynamics, 적분상태 ∫e 포함(18차원).

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
TCP = next(c for c in ("tool0", "ee_link", "wrist_3_link") if c in robot.link_map)


def tcp_pos(q):
    robot.update_cfg(q)
    return robot.scene.graph.get(TCP)[0][:3, 3]


# --- 경유점(관절) ---
WP = [np.zeros(6),
      np.array([0.9, -0.6, 0.5, 0.0, 0.3, 0.0]),
      np.array([-0.8, -0.5, 0.8, 0.2, -0.4, 0.0]),
      np.array([0.3, -1.0, 1.4, -0.3, 0.6, 0.0]),
      np.zeros(6)]
N_SEG = len(WP) - 1
T_MOVE, T_HOLD = 1.0, 0.7            # 이동 + 정지(I항 정착)
SEG = T_MOVE + T_HOLD
T_TOTAL = N_SEG * SEG

KP, KI, KD = 250.0, 120.0, 45.0      # 순수 PID 게인


def reference(t):
    k = min(int(t // SEG), N_SEG - 1)
    local = t - k * SEG
    q0, q1 = WP[k], WP[k + 1]
    if local < T_MOVE:
        s, sd, _ = quintic_time_scaling(local, T_MOVE)
        return q0 + s * (q1 - q0), sd * (q1 - q0)
    return q1.copy(), np.zeros(6)     # dwell: 목표 유지


def rhs(t, x):
    th, dth, I = x[:6], x[6:12], x[12:]
    th_d, dth_d = reference(t)
    e = th_d - th
    tau = KP * e + KI * I + KD * (dth_d - dth)     # 순수 PID
    return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, tau, *ARGS), e])


sol = solve_ivp(rhs, (0, T_TOTAL), np.zeros(18),
                t_eval=np.linspace(0, T_TOTAL, 48), method="RK45", rtol=1e-6, atol=1e-9)
THETA = sol.y[:6].T

# 경유점 추종 정확도(각 dwell 끝에서)
for nm, k in zip("ABC", range(1, 4)):
    idx = int(((k - 1) * SEG + T_MOVE + T_HOLD * 0.9) / T_TOTAL * (len(THETA) - 1))
    err = np.rad2deg(np.abs(THETA[idx] - WP[k])).max()
    print(f"  경유점 {nm} 도달오차 ~{err:.1f}°")

STARS = np.array([tcp_pos(q) for q in WP[1:4]])
print(f"총 {len(THETA)} 프레임 (PID, TCP={TCP})")

# --- 스타일 ---
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
ax.set_title("UR5 multi-waypoint  (PID control, real mesh)", fontsize=12, weight="bold")

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


anim = FuncAnimation(fig, render, frames=len(THETA), interval=70, blit=False)
anim.save(out("urdf_multi_waypoint.gif"), writer=PillowWriter(fps=14), dpi=72)
print("[저장] outputs/urdf_multi_waypoint.gif")
plt.show()
