"""
임피던스 제어를 실제 UR5 메시로 — stiff vs soft (밀었을 때 반응).

말단을 가상 스프링-댐퍼로:  F = -K·(x − x_d) − D·ẋ,   τ = Jᵀ·F + g(θ)
외력(push)은 Jᵀ·F_ext 로 주입. 같은 힘을 줘도:
  왼쪽  stiff(K 큼)  → 거의 안 밀림 (위치제어처럼 뻣뻣)
  오른쪽 soft (K 작음) → 쑥 밀렸다가 힘 빼면 스프링처럼 복귀

J는 fk 의 수치 미분(3x6), plant=forward_dynamics.
실행:  python urdf_impedance_anim.py   (+ outputs/urdf_impedance.gif)
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
from app.core.robot_math import Dynamics
from app.core import ur5_model as ur5
from app.core.paths import out

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)
robot = load_robot_description("ur5_description")
TCP = next(c for c in ("tool0", "ee_link", "wrist_3_link") if c in robot.link_map)

THETA0 = np.array([0.0, -0.6, 1.0, 0.0, 0.5, 0.0])      # 시작=평형 자세
X_D = ur5.fk(THETA0)[:3, 3]                              # 평형 말단 위치
PUSH = np.array([45.0, 0.0, 0.0])                        # 외력 [N] (world +x)
T1, T2 = 0.6, 1.5                                        # 미는 구간 [s]
T_END = 3.5


def gravity(th):
    return Dynamics.gravity_forces(th, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


def jac_p(th, eps=1e-6):                                 # 말단 위치 자코비안 (3x6)
    x0 = ur5.fk(th)[:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        d = th.copy(); d[i] += eps
        J[:, i] = (ur5.fk(d)[:3, 3] - x0) / eps
    return J


def sim(K, D):
    def rhs(t, x):
        th, dth = x[:6], x[6:]
        J = jac_p(th)
        x_cur = ur5.fk(th)[:3, 3]
        F = -K * (x_cur - X_D) - D * (J @ dth)           # 가상 스프링-댐퍼
        tau = J.T @ F + gravity(th)
        if T1 <= t <= T2:
            tau = tau + J.T @ PUSH                        # 외력 주입
        return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, tau, *ARGS)])
    sol = solve_ivp(rhs, (0, T_END), np.concatenate([THETA0, np.zeros(6)]),
                    t_eval=np.linspace(0, T_END, 30), method="RK45", rtol=1e-6, atol=1e-9)
    return sol.y[:6].T


STIFF = sim(1500.0, 130.0)
SOFT = sim(250.0, 50.0)
for nm, TR in [("stiff", STIFF), ("soft", SOFT)]:
    dev = max(np.linalg.norm(ur5.fk(q)[:3, 3] - X_D) for q in TR)
    print(f"  {nm}: 최대 말단 변위 {dev*100:.1f} cm")

# --- 렌더 (메시 2대) ---
OFF = {"stiff": np.array([0, -0.45, 0]), "soft": np.array([0, 0.45, 0])}
robot.update_cfg(THETA0)
star = robot.scene.graph.get(TCP)[0][:3, 3]
LIGHT = np.array([0.35, 0.35, 1.0]); LIGHT /= np.linalg.norm(LIGHT)


def shaded(m, base):
    sh = 0.35 + 0.65 * np.clip(m.face_normals @ LIGHT, 0, 1)
    return np.array(mcolors.to_rgb(base))[None, :] * sh[:, None]


fig = plt.figure(figsize=(9, 7), facecolor="white")
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=-72)
ax.set_xlim(-0.3, 0.9); ax.set_ylim(-1.0, 1.0); ax.set_zlim(-0.1, 1.1)
ax.set_box_aspect((0.6, 1.0, 0.6))
for p in (ax.xaxis, ax.yaxis, ax.zaxis):
    p.pane.set_facecolor((0.97, 0.97, 0.98, 1.0)); p.pane.set_edgecolor((0.9, 0.9, 0.9, 1.0))
ax.grid(False); ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
ax.set_title("Impedance control — push response  (stiff vs soft)", fontsize=12, weight="bold")

for off in OFF.values():                                # 평형점(별)
    ax.scatter(*(star + off), marker="*", s=180, color="#e8743b",
               edgecolors="k", linewidths=0.4, depthshade=False, zorder=12)
ax.text(0.0, -0.45, 1.05, "STIFF\n(K=1500)", fontsize=11, weight="bold", color="#1f6fb2", ha="center")
ax.text(0.0, 0.45, 1.05, "SOFT\n(K=250)", fontsize=11, weight="bold", color="#c0392b", ha="center")
holders = [None, None]


def render(i):
    for h in holders:
        if h is not None:
            h.remove()
    pushing = T1 <= (i / (len(STIFF) - 1) * T_END) <= T2
    for k, (name, TR, base) in enumerate([("stiff", STIFF, "#4f86c6"), ("soft", SOFT, "#c0504d")]):
        robot.update_cfg(TR[i])
        m = robot.scene.to_geometry()
        coll = Poly3DCollection((m.vertices + OFF[name])[m.faces],
                                facecolors=shaded(m, base), edgecolor="none")
        holders[k] = coll
        ax.add_collection3d(coll)
    ax.set_title(("⟶ PUSH!  " if pushing else "") + "Impedance — stiff vs soft",
                 fontsize=12, weight="bold", color=("#c0392b" if pushing else "#333"))
    return holders


anim = FuncAnimation(fig, render, frames=len(STIFF), interval=90, blit=False)
anim.save(out("urdf_impedance.gif"), writer=PillowWriter(fps=11), dpi=72)
print("[저장] outputs/urdf_impedance.gif")
plt.show()
