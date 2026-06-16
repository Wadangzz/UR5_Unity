"""
UR5 전체 파이프라인 통합 데모 (순수 Python, Unity 불필요).

  원하는 말단 자세  ──①IK──▶  목표 관절각
                    ──②경로──▶  매끈한 기준궤적 θ_d(t),θ̇_d(t),θ̈_d(t)
                    ──③제어──▶  Computed Torque 토크 τ
                    ──④plant──▶  forward_dynamics 적분 → 실제 움직임
                    ──⑤시각화──▶ matplotlib 3D 애니메이션 (목표에 도달)

"기구학(IK·경로) → 동역학(제어·물리)" 전 과정을 하나로 꿴다.
실행:  python full_pipeline.py   (+ full_pipeline.gif 저장)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from app.core.robot_math import Dynamics, Kinematics, SE3, quintic_time_scaling
from app.core import ur5_model as ur5
from app.core.paths import out

np.set_printoptions(precision=4, suppress=True)
ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)


def banner(s):
    print("\n" + "=" * 64 + "\n" + s + "\n" + "=" * 64)


# ====================================================================
# ① 원하는 말단 자세 (reachable 한 목표를 FK 로 정의)
# ====================================================================
banner("① 원하는 말단 자세")
theta_demo = np.array([0.8, -0.5, 0.6, 0.0, 0.4, 0.0])
T_des = ur5.fk(theta_demo)
p_des = T_des[:3, 3]
print(f"  목표 end-effector 위치 [m] = {p_des}")

# ====================================================================
# ② 역기구학 (IK) — 원본 Kinematics.IK (Newton-Raphson, degree) 사용
#    MR 모델 → ur5.ik_robot 어댑터, 결과 degree → radian 변환
# ====================================================================
banner("② 역기구학 (IK) — Kinematics.IK (원본)")
theta_start = np.zeros(6)
desired = p_des.tolist() + SE3.orientation(T_des)[0]        # [x,y,z, quaternion]
theta_goal_deg, count = Kinematics.IK(ur5.ik_robot, [0.0] * 6, desired)
theta_goal = np.deg2rad(theta_goal_deg)                     # ← degree → radian 변환
p_ik = ur5.fk(theta_goal)[:3, 3]
print(f"  Kinematics.IK 반복 횟수: {count}")
print(f"  목표 관절각 θ_goal [deg] = {np.round(theta_goal_deg, 3)}")
print(f"  목표 관절각 θ_goal [rad] = {theta_goal}")
print(f"  IK 해의 말단 위치 [m]    = {p_ik}   (오차 {np.linalg.norm(p_ik-p_des):.1e} m)")

# ====================================================================
# ③ 경로 계획 — quintic 기준궤적 θ_d(t), θ̇_d(t), θ̈_d(t)
# ====================================================================
banner("③ 경로 계획 (quintic trajectory)")
T_MOVE = 2.0
DTOTAL = theta_goal - theta_start


def reference(t):
    if t >= T_MOVE:
        return theta_goal.copy(), np.zeros(6), np.zeros(6)
    s, sd, sdd = quintic_time_scaling(t, T_MOVE)
    return theta_start + s * DTOTAL, sd * DTOTAL, sdd * DTOTAL


print(f"  home → θ_goal, T={T_MOVE}s, 5차 다항식 시간 스케일링")

# ====================================================================
# ④ Computed Torque 제어 + 순동역학 plant
# ====================================================================
banner("④ 동역학 제어 (Computed Torque) + plant 적분")
KP, KD = 100.0, 20.0       # 임계감쇠 (Kd = 2√Kp)


def computed_torque(t, theta, dtheta):
    th_d, dth_d, ddth_d = reference(t)
    e, ed = th_d - theta, dth_d - dtheta
    aq = ddth_d + KD * ed + KP * e
    M = Dynamics.mass_matrix(theta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    c = Dynamics.coriolis_forces(theta, dtheta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    g = Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    return M @ aq + c + g


T_END = 3.0


def rhs(t, x):
    theta, dtheta = x[:6], x[6:]
    tau = computed_torque(t, theta, dtheta)
    return np.concatenate([dtheta, Dynamics.forward_dynamics(theta, dtheta, tau, *ARGS)])


t_eval = np.linspace(0.0, T_END, 151)
sol = solve_ivp(rhs, (0.0, T_END), np.concatenate([theta_start, np.zeros(6)]),
                t_eval=t_eval, method='RK45', rtol=1e-6, atol=1e-9)
theta_log = sol.y[:6].T

ref_arr = np.array([reference(t)[0] for t in sol.t])
track_err = np.linalg.norm(theta_log - ref_arr, axis=1)
p_final = ur5.fk(theta_log[-1])[:3, 3]
print(f"  운동구간 최대 추종오차 = {np.rad2deg(track_err[sol.t <= T_MOVE].max()):.4f} deg")
print(f"  최종 말단 위치 [m]     = {p_final}")
print(f"  목표 대비 도달 오차    = {np.linalg.norm(p_final - p_des)*1000:.3f} mm")

# ====================================================================
# ⑤ 3D 애니메이션
# ====================================================================
banner("⑤ 시각화 (3D 애니메이션)")
frames = np.array([ur5.fk_skeleton(q) for q in theta_log])
goal_arm = ur5.fk_skeleton(theta_goal)

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=18, azim=48)
c = frames.reshape(-1, 3).mean(axis=0)
r = 0.55
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(-0.15, 1.05)
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
ax.set_title("UR5 full pipeline:  desired pose -> IK -> trajectory -> control")

ax.plot(goal_arm[:, 0], goal_arm[:, 1], goal_arm[:, 2],
        "--o", color="lightgray", lw=2, ms=4, label="IK goal")
ax.scatter(*p_des, color="red", marker="*", s=200, label="desired pose", depthshade=False)
arm, = ax.plot([], [], [], "-o", color="tab:blue", lw=4, ms=6, label="UR5")
trace, = ax.plot([], [], [], "-", color="tab:red", lw=1.2, alpha=0.7)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
ax.legend(loc="upper right")
hist = []


def update(i):
    P = frames[i]
    arm.set_data(P[:, 0], P[:, 1]); arm.set_3d_properties(P[:, 2])
    hist.append(P[-1]); H = np.array(hist)
    trace.set_data(H[:, 0], H[:, 1]); trace.set_3d_properties(H[:, 2])
    txt.set_text(f"t = {sol.t[i]:.2f} s")
    return arm, trace, txt


anim = FuncAnimation(fig, update, frames=len(frames), interval=40, blit=False)
anim.save(out("full_pipeline.gif"), writer=PillowWriter(fps=25), dpi=72)
print("  [저장] full_pipeline.gif")
plt.show()
print("\n[완료] 원하는 자세 → IK → 경로 → 동역학 제어 → 도달, 전 과정 통합.")
