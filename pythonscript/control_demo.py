"""
UR5 제어 데모 — Unity 없이 순수 Python으로 닫는 제어 루프.

plant(가상 로봇) = MyRobotMath.Dynamics.forward_dynamics  (순동역학)
제어기            = 토크 τ 를 계산하는 함수

세 시나리오로 동역학+제어를 보여준다:
  1) 토크 0       → 팔이 중력에 무너짐 (제어 없음)
  2) 중력보상     → τ=g(θ) 로 무중력처럼 그 자리에 멈춤
  3) PD+중력보상  → τ=Kp·e + Kd·ė + g(θ) 로 목표 자세로 이동·정지

적분: semi-implicit(symplectic) Euler.
실행:  python control_demo.py   (시나리오3 궤적을 pd_trajectory.npz 로 저장 → 애니메이션용)
"""

import numpy as np
from scipy.integrate import solve_ivp
from MyRobotMath import Dynamics
import ur5_model as ur5

np.set_printoptions(precision=3, suppress=True)

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)


def gravity(theta):
    return Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


def simulate(controller, theta0, t_end=3.0, dtheta0=None, hz=50):
    """
    controller(theta, dtheta) → τ 를 받아 plant(순동역학)를 적분.
    상태 x = [theta(6), dtheta(6)], ẋ = [dtheta, forward_dynamics(...)].
    적분기는 RK45 적응형(scipy.solve_ivp) — 강성/정밀도 자동 처리.
    :return: (t, theta_log, tau_log)
    """
    theta0 = np.array(theta0, dtype=float)
    dtheta0 = np.zeros(6) if dtheta0 is None else np.array(dtheta0, dtype=float)
    x0 = np.concatenate([theta0, dtheta0])

    def rhs(t, x):
        theta, dtheta = x[:6], x[6:]
        tau = controller(theta, dtheta)
        ddtheta = Dynamics.forward_dynamics(theta, dtheta, tau, *ARGS)
        return np.concatenate([dtheta, ddtheta])

    t_eval = np.linspace(0.0, t_end, int(t_end * hz) + 1)
    sol = solve_ivp(rhs, (0.0, t_end), x0, t_eval=t_eval,
                    method='RK45', rtol=1e-6, atol=1e-8)

    theta_log = sol.y[:6].T
    dtheta_log = sol.y[6:].T
    tau_log = np.array([controller(theta_log[i], dtheta_log[i])
                        for i in range(len(sol.t))])
    return sol.t, theta_log, tau_log


def banner(title):
    print("\n" + "=" * 64)
    print(title)
    print("=" * 64)


# ----------------------------------------------------------------------
# 시나리오 1) 토크 0 — 중력에 무너지는 팔
# ----------------------------------------------------------------------
banner("1) 제어 없음 (τ=0): 팔이 중력에 무너짐")
start = np.array([0.0, -0.6, 1.2, 0.0, 0.6, 0.0])   # 살짝 굽힌 자세, 정지에서 시작
t, th, _ = simulate(lambda q, dq: np.zeros(6), start, t_end=2.0)
drift = np.abs(th[-1] - th[0])
print("  시작 θ [rad] =", th[0])
print("  2초 후 θ     =", th[-1])
print(f"  최대 관절 변화 = {np.rad2deg(drift.max()):.1f}°  → 큰 폭으로 흔들림(무너짐)")

# ----------------------------------------------------------------------
# 시나리오 2) 중력보상만 — τ=g(θ) → 무중력처럼 정지
# ----------------------------------------------------------------------
banner("2) 중력보상 (τ=g(θ)): 무중력처럼 그 자리에 멈춤")
t, th, _ = simulate(lambda q, dq: gravity(q), start, t_end=2.0)
drift = np.abs(th[-1] - th[0])
print("  시작 θ [rad] =", th[0])
print("  2초 후 θ     =", th[-1])
print(f"  최대 관절 변화 = {np.rad2deg(drift.max()):.4f}°  → 거의 0 (중력 상쇄)")

# ----------------------------------------------------------------------
# 시나리오 3) PD + 중력보상 — 목표 자세로 이동 후 정지
# ----------------------------------------------------------------------
banner("3) PD + 중력보상: 팔 수평 → 목표(팔 직립)로 이동·정지")
Kp, Kd = 80.0, 22.0
theta_d = np.array([0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0])   # 어깨 -90° (팔 위로)
start3 = np.zeros(6)                                        # home (팔 수평)


def pd_grav(theta, dtheta):
    e = theta_d - theta
    return Kp * e - Kd * dtheta + gravity(theta)   # θ̇_d = 0


t, th, tau = simulate(pd_grav, start3, t_end=3.0)
err = np.linalg.norm(th - theta_d, axis=1)         # 시간별 추종오차 노름
print(f"  게인  Kp={Kp}, Kd={Kd}")
print(f"  목표 θ_d [rad] = {theta_d}")
print("  추종오차 ||θ−θ_d|| [rad]:")
for tc in [0.0, 0.25, 0.5, 1.0, 2.0, 3.0]:
    i = int(np.argmin(np.abs(t - tc)))
    print(f"     t={tc:>4.2f}s : {err[i]:.4f}")
print(f"  최종 오차 = {np.rad2deg(np.abs(th[-1]-theta_d)).max():.3f}°  (수렴)")
print(f"  정상상태 유지 토크 ≈ {tau[-1]}  N·m")

# 시나리오3 궤적 저장 → (B) 애니메이션에서 사용
np.savez("pd_trajectory.npz", t=t, theta=th, theta_d=theta_d)
print("\n[저장] pd_trajectory.npz  (B 단계 matplotlib 애니메이션용)")
print("[완료] Unity 없이 순수 Python으로 제어 루프를 닫음.")
