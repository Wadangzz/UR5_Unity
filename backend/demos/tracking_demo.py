"""
UR5 궤적 추종 제어 — Computed Torque(피드백 선형화) vs PD+중력보상.

기준궤적 θ_d(t), θ̇_d(t), θ̈_d(t) 를 quintic time scaling 으로 생성하고,
두 제어기로 따라가게 한 뒤 추종오차를 비교한다.

  Computed Torque :  τ = M(θ)[θ̈_d + Kd·ė + Kp·e] + C(θ,θ̇)θ̇ + g(θ)
                     → 비선형 동역학을 상쇄 → 오차동역학이 ë+Kd·ė+Kp·e=0 (선형)
  PD + 중력보상   :  τ = Kp·e + Kd·ė + g(θ)
                     → 간단하지만 M·C 를 모르므로 빠른 구간에서 뒤처짐

plant = forward_dynamics, 적분 = solve_ivp(RK45).
실행:  python tracking_demo.py   → tracking_trajectory.npz 저장(애니메이션용)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
from scipy.integrate import solve_ivp
from app.core.robot_math import Dynamics, quintic_time_scaling
from app.core import ur5_model as ur5
from app.core.paths import out

np.set_printoptions(precision=4, suppress=True)

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)

# --- 기준궤적: home → goal, T초 동안 quintic (라디안) ---
THETA_S = np.zeros(6)
THETA_GOAL = np.array([np.pi/2, -np.pi/3, np.pi/2, -np.pi/4, np.pi/3, np.pi/6])
T = 2.0
DTOTAL = THETA_GOAL - THETA_S


def reference(t):
    """시각 t 에서 기준 (위치, 속도, 가속도). t>T 면 goal 에서 정지."""
    if t >= T:
        return THETA_GOAL.copy(), np.zeros(6), np.zeros(6)
    s, sd, sdd = quintic_time_scaling(t, T)
    return THETA_S + s * DTOTAL, sd * DTOTAL, sdd * DTOTAL


def gravity(theta):
    return Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


# --- 제어기 ---
KP, KD = 100.0, 20.0          # 오차동역학: 임계감쇠 (Kd = 2√Kp)


def computed_torque(t, theta, dtheta):
    th_d, dth_d, ddth_d = reference(t)
    e, ed = th_d - theta, dth_d - dtheta
    aq = ddth_d + KD * ed + KP * e                       # 가상 가속도 명령
    M = Dynamics.mass_matrix(theta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    c = Dynamics.coriolis_forces(theta, dtheta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
    return M @ aq + c + gravity(theta)


def pd_gravity(t, theta, dtheta):
    th_d, dth_d, _ = reference(t)
    e, ed = th_d - theta, dth_d - dtheta
    return KP * e + KD * ed + gravity(theta)


# --- 시뮬레이터 (controller(t, θ, θ̇) → τ) ---
def simulate(controller, t_end, hz=100):
    x0 = np.concatenate([THETA_S, np.zeros(6)])

    def rhs(t, x):
        theta, dtheta = x[:6], x[6:]
        tau = controller(t, theta, dtheta)
        return np.concatenate([dtheta, Dynamics.forward_dynamics(theta, dtheta, tau, *ARGS)])

    t_eval = np.linspace(0.0, t_end, int(t_end * hz) + 1)
    sol = solve_ivp(rhs, (0.0, t_end), x0, t_eval=t_eval,
                    method='RK45', rtol=1e-6, atol=1e-9)
    return sol.t, sol.y[:6].T


def track_error(t_arr, theta_arr):
    """각 시각의 ||θ − θ_d|| (rad)."""
    ref = np.array([reference(t)[0] for t in t_arr])
    return np.linalg.norm(theta_arr - ref, axis=1), ref


def banner(s):
    print("\n" + "=" * 64 + "\n" + s + "\n" + "=" * 64)


T_END = 3.0
banner("궤적 추종 비교: Computed Torque vs PD + 중력보상")
print(f"  기준궤적: home → goal, quintic, T={T}s   (이후 {T_END-T:.0f}s 정지 유지)")
print(f"  게인 Kp={KP}, Kd={KD}  (임계감쇠)")

t_ct, th_ct = simulate(computed_torque, T_END)
t_pd, th_pd = simulate(pd_gravity, T_END)
err_ct, ref = track_error(t_ct, th_ct)
err_pd, _ = track_error(t_pd, th_pd)

print("\n  추종오차 ||θ−θ_d|| [rad]      Computed-Torque |   PD+중력보상")
print("  " + "-" * 56)
for tc in [0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
    i = int(np.argmin(np.abs(t_ct - tc)))
    print(f"     t={tc:>4.2f}s                 {err_ct[i]:.6f}     |   {err_pd[i]:.6f}")

print("\n  운동 구간(0~T) 최대오차:")
mv = t_ct <= T
print(f"     Computed Torque : {err_ct[mv].max():.6f} rad  ({np.rad2deg(err_ct[mv].max()):.3f}°)")
print(f"     PD + 중력보상   : {err_pd[mv].max():.6f} rad  ({np.rad2deg(err_pd[mv].max()):.3f}°)")
ratio = err_pd[mv].max() / max(err_ct[mv].max(), 1e-12)
print(f"     → Computed Torque 가 약 {ratio:.0f}배 더 정밀하게 추종")

# 애니메이션용 저장 (기준 + 두 제어기 결과)
np.savez(out("tracking_trajectory.npz"), t=t_ct, theta_ref=ref,
         theta_ct=th_ct, theta_pd=th_pd)
print("\n[저장] tracking_trajectory.npz")
print("[완료] 동역학 모델(M·C·g)을 활용한 궤적 추종 제어.")
