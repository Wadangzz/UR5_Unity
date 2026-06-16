"""
UR5 다중 경유점(waypoint) 순회 데모 — 전 파이프라인을 여러 자세에 연속 적용.

여러 목표 말단 자세를 순서대로:
    각 목표 ──IK──▶ 관절각  ──연결 quintic──▶ 기준궤적  ──Computed Torque──▶ 순회
home → A → B → C → home 을 매끄럽게 오가며 각 별(target)을 짚는다.
(여러분 원래 프로젝트의 "프로그램: 여러 pose 저장 후 실행"과 같은 개념.)

실행:  python multi_waypoint_demo.py   (+ multi_waypoint.gif 저장)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from robot_math import Dynamics, Kinematics, SE3, quintic_time_scaling
import ur5_model as ur5
from paths import out

np.set_printoptions(precision=3, suppress=True)
ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)

# --- 목표 말단 자세들(경유점). reachable 한 config 의 FK 로 정의 ---
WP_CONFIGS = [
    np.array([0.9, -0.6, 0.5, 0.0, 0.3, 0.0]),    # A
    np.array([-0.8, -0.5, 0.8, 0.2, -0.4, 0.0]),  # B
    np.array([0.3, -1.0, 1.4, -0.3, 0.6, 0.0]),   # C
]
WP_POSES = [ur5.fk(c) for c in WP_CONFIGS]
WP_POINTS = [T[:3, 3] for T in WP_POSES]           # 별 위치


# --- 각 경유점을 IK 로 풀어 관절 경유점 리스트 구성 ---
print("경유점별 IK (Kinematics.IK):")
q_home = np.zeros(6)
q_waypoints = [q_home]
for name, T in zip("ABC", WP_POSES):
    desired = T[:3, 3].tolist() + SE3.orientation(T)[0]
    sol_deg, cnt = Kinematics.IK(ur5.ik_robot, [0.0] * 6, desired)
    q = np.deg2rad(sol_deg)
    p = ur5.fk(q)[:3, 3]
    print(f"  {name}: IK {cnt}회, 말단오차 {np.linalg.norm(p - T[:3,3]):.1e} m")
    q_waypoints.append(q)
q_waypoints.append(q_home)                          # 마지막엔 home 복귀
N_SEG = len(q_waypoints) - 1

# --- 구간별 quintic 기준궤적 (home→A→B→C→home) ---
T_SEG = 1.5
T_TOTAL = N_SEG * T_SEG


def reference(t):
    seg = min(int(t // T_SEG), N_SEG - 1)
    tau = t - seg * T_SEG
    q0, q1 = q_waypoints[seg], q_waypoints[seg + 1]
    dq = q1 - q0
    s, sd, sdd = quintic_time_scaling(min(tau, T_SEG), T_SEG)
    return q0 + s * dq, sd * dq, sdd * dq


# --- Computed Torque 제어 + plant ---
KP, KD = 100.0, 20.0


def rhs(t, x):
    theta, dtheta = x[:6], x[6:]
    th_d, dth_d, ddth_d = reference(t)
    aq = ddth_d + KD * (dth_d - dtheta) + KP * (th_d - theta)
    M = Dynamics.mass_matrix(theta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    c = Dynamics.coriolis_forces(theta, dtheta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    g = Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    tau = M @ aq + c + g
    return np.concatenate([dtheta, Dynamics.forward_dynamics(theta, dtheta, tau, *ARGS)])


t_eval = np.linspace(0.0, T_TOTAL, int(T_TOTAL * 30) + 1)
sol = solve_ivp(rhs, (0.0, T_TOTAL), np.concatenate([q_home, np.zeros(6)]),
                t_eval=t_eval, method='RK45', rtol=1e-6, atol=1e-9)
theta_log = sol.y[:6].T
print(f"\n총 {N_SEG}구간, {T_TOTAL:.1f}s 순회 완료. 프레임 {len(t_eval)}개")

# --- 애니메이션 ---
frames = np.array([ur5.fk_skeleton(q) for q in theta_log])
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=48)
allp = np.concatenate([frames.reshape(-1, 3)] + [np.array(WP_POINTS)], axis=0)
ctr = allp.mean(axis=0)
r = 0.6
ax.set_xlim(ctr[0]-r, ctr[0]+r); ax.set_ylim(ctr[1]-r, ctr[1]+r); ax.set_zlim(-0.15, 1.05)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title("UR5 multi-waypoint:  home -> A -> B -> C -> home")

# 경유점 별 + 라벨
for name, pt in zip("ABC", WP_POINTS):
    ax.scatter(*pt, color="red", marker="*", s=220, depthshade=False,
               edgecolors="k", linewidths=0.5, zorder=10)
    ax.text(pt[0], pt[1], pt[2] + 0.05, name, fontsize=12, weight="bold")

arm, = ax.plot([], [], [], "-o", color="tab:blue", lw=4, ms=6)
trace, = ax.plot([], [], [], "-", color="tab:red", lw=1.0, alpha=0.6)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
hist = []


def update(i):
    P = frames[i]
    arm.set_data(P[:, 0], P[:, 1]); arm.set_3d_properties(P[:, 2])
    hist.append(P[-1]); H = np.array(hist)
    trace.set_data(H[:, 0], H[:, 1]); trace.set_3d_properties(H[:, 2])
    txt.set_text(f"t = {sol.t[i]:.2f} s")
    return arm, trace, txt


anim = FuncAnimation(fig, update, frames=len(frames), interval=33, blit=False)
anim.save(out("multi_waypoint.gif"), writer=PillowWriter(fps=25), dpi=72)
print("[저장] multi_waypoint.gif")
plt.show()
