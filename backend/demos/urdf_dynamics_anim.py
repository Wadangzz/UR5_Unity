"""
동역학을 실제 UR5 URDF 메시에 적용한 애니메이션 (3막 순차).

  1막 중력낙하   : τ=0      → 팔이 중력에 무너짐
  2막 중력보상   : τ=g(θ)   → 무중력처럼 정지
  3막 Computed Torque : 궤적 추종 → 정밀 제어

plant = robot_math.Dynamics.forward_dynamics, 적분 solve_ivp(RK45).
관절각 θ(t)는 동역학 시뮬 결과, URDF 메시는 그걸 그리는 도구(조명 셰이딩).

실행:  python urdf_dynamics_anim.py   (+ outputs/urdf_dynamics.gif)
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
from app.core.robot_math import Dynamics, quintic_time_scaling
from app.core import ur5_model as ur5
from app.core.paths import out

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)
robot = load_robot_description("ur5_description")


def gravity(th):
    return Dynamics.gravity_forces(th, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


def sim(controller, theta0, t_end, n):
    x0 = np.concatenate([np.array(theta0, float), np.zeros(6)])

    def rhs(t, x):
        th, dth = x[:6], x[6:]
        tau = controller(t, th, dth)
        return np.concatenate([dth, Dynamics.forward_dynamics(th, dth, tau, *ARGS)])

    sol = solve_ivp(rhs, (0, t_end), x0, t_eval=np.linspace(0, t_end, n),
                    method="RK45", rtol=1e-6, atol=1e-9)
    return sol.y[:6].T


# --- 1막: 중력 낙하 (무제어) ---
fall = sim(lambda t, th, dth: np.zeros(6), [0, -1.4, 0.5, 0, 0.6, 0], 1.6, 14)

# --- 2막: 중력보상 (정지 유지) ---
hold = sim(lambda t, th, dth: gravity(th), [0, -1.0, 1.2, 0, 0.4, 0], 1.2, 8)

# --- 3막: Computed Torque 궤적 추종 ---
TGT = np.array([0.7, -1.1, 1.4, -0.3, 0.6, 0.2])
KP, KD = 100.0, 20.0
def ref(t):
    s, sd, sdd = quintic_time_scaling(min(t, 1.5), 1.5)
    return s * TGT, sd * TGT, sdd * TGT
def ct(t, th, dth):
    th_d, dth_d, ddth_d = ref(t)
    aq = ddth_d + KD * (dth_d - dth) + KP * (th_d - th)
    M = Dynamics.mass_matrix(th, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    c = Dynamics.coriolis_forces(th, dth, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    return M @ aq + c + gravity(th)
track = sim(ct, np.zeros(6), 2.2, 16)

SEQ = ([(q, "1. No control  (gravity)") for q in fall]
       + [(q, "2. Gravity compensation") for q in hold]
       + [(q, "3. Computed Torque tracking") for q in track])
print(f"총 {len(SEQ)} 프레임")

# --- 세련된 스타일: 조명 셰이딩 + 깔끔한 배경 ---
LIGHT = np.array([0.35, 0.35, 1.0]); LIGHT /= np.linalg.norm(LIGHT)
BASE = np.array(mcolors.to_rgb("#4f86c6"))      # UR 풍 블루


def shaded(mesh):
    inten = np.clip(mesh.face_normals @ LIGHT, 0, 1)
    shade = 0.35 + 0.65 * inten                 # 0.35~1.0 명암
    return BASE[None, :] * shade[:, None]


fig = plt.figure(figsize=(7, 8), facecolor="white")
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=16, azim=52)
ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(-0.2, 1.0)
ax.set_box_aspect((1, 1, 0.85))
for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
    pane.pane.set_facecolor((0.97, 0.97, 0.98, 1.0))
    pane.pane.set_edgecolor((0.9, 0.9, 0.9, 1.0))
ax.grid(False)
ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
title = ax.set_title("", fontsize=13, weight="bold", color="#333")
holder = [None]


def render(i):
    if holder[0] is not None:
        holder[0].remove()
    q, label = SEQ[i]
    robot.update_cfg(q)
    m = robot.scene.to_geometry()
    coll = Poly3DCollection(m.vertices[m.faces], facecolors=shaded(m),
                            edgecolor="none")
    holder[0] = coll
    ax.add_collection3d(coll)
    title.set_text(label)
    return [coll]


anim = FuncAnimation(fig, render, frames=len(SEQ), interval=80, blit=False)
anim.save(out("urdf_dynamics.gif"), writer=PillowWriter(fps=13), dpi=72)
print("[저장] outputs/urdf_dynamics.gif")
plt.show()
