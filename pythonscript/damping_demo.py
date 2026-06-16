"""
감쇠비(ζ) 효과 데모 — 출렁임을 D(감쇠) 게인으로 어떻게 잡는가.

Computed Torque 로 set-point 제어하면 각 관절 오차가 정확히
    ë + Kd·ė + Kp·e = 0   (2차 선형 시스템)
이 되어, Kd 만 바꾸면 감쇠비 ζ = Kd / (2√Kp) 가 바뀐다.

  ζ < 1  underdamped : 목표를 지나쳐 출렁이며 수렴 (overshoot)
  ζ = 1  critical    : 출렁임 없이 최速 수렴
  ζ > 1  overdamped  : 안 출렁이지만 느림

어깨(2번) 관절을 0 → -60° 로 step 명령하고 응답을 비교한다.
실행:  python damping_demo.py   → damping_response.png 저장
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from MyRobotMath import Dynamics
import ur5_model as ur5

np.set_printoptions(precision=3, suppress=True)

ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)

START = np.zeros(6)
THETA_D = np.array([0.0, -np.pi / 3, 0.0, 0.0, 0.0, 0.0])   # 어깨 -60° step
KP = 100.0
OMEGA_N = np.sqrt(KP)                                        # 고유진동수 ωn = 10 rad/s
T_END = 2.0


def computed_torque_setpoint(Kd):
    """set-point computed torque: τ = M(Kp·e − Kd·θ̇) + Cθ̇ + g  (θ̇_d=θ̈_d=0)."""
    def ctrl(theta, dtheta):
        e = THETA_D - theta
        aq = KP * e - Kd * dtheta
        M = Dynamics.mass_matrix(theta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
        c = Dynamics.coriolis_forces(theta, dtheta, ur5.MLIST, ur5.GLIST, ur5.SLIST)
        g = Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)
        return M @ aq + c + g
    return ctrl


def simulate(Kd):
    ctrl = computed_torque_setpoint(Kd)

    def rhs(t, x):
        theta, dtheta = x[:6], x[6:]
        tau = ctrl(theta, dtheta)
        return np.concatenate([dtheta, Dynamics.forward_dynamics(theta, dtheta, tau, *ARGS)])

    t_eval = np.linspace(0.0, T_END, 400)
    sol = solve_ivp(rhs, (0.0, T_END), np.concatenate([START, np.zeros(6)]),
                    t_eval=t_eval, method='RK45', rtol=1e-7, atol=1e-9)
    return sol.t, sol.y[1]      # 어깨(2번) 관절각


CASES = [(0.2, "underdamped", "tab:red"),
         (1.0, "critical",    "tab:green"),
         (2.0, "overdamped",  "tab:blue")]

target_deg = np.rad2deg(THETA_D[1])
travel = target_deg - np.rad2deg(START[1])      # -60°

plt.figure(figsize=(8, 5))
print(f"목표: 어깨 0° → {target_deg:.0f}°,  Kp={KP:.0f} (ωn={OMEGA_N:.0f} rad/s)\n")
print("  ζ      Kd     오버슈트    정착(±2%)")
print("  " + "-" * 42)

for zeta, name, color in CASES:
    Kd = 2 * zeta * OMEGA_N
    t, q2 = simulate(Kd)
    q2_deg = np.rad2deg(q2)

    # 오버슈트(%): 목표를 지나친 최대량 / 이동량
    overshoot = max(0.0, (q2_deg.min() - target_deg) / travel * 100)
    # 정착시간: 목표의 ±2% 이내로 들어와 유지되는 시각
    band = 0.02 * abs(travel)
    settled = np.where(np.abs(q2_deg - target_deg) > band)[0]
    t_settle = t[settled[-1]] if len(settled) else 0.0

    print(f"  {zeta:.1f}   {Kd:5.0f}   {overshoot:6.1f}%    {t_settle:5.2f}s")
    plt.plot(t, q2_deg, color=color, lw=2, label=f"{name}  ζ={zeta} (Kd={Kd:.0f})")

plt.axhline(target_deg, ls="--", color="gray", lw=1, label="target")
plt.xlabel("time [s]"); plt.ylabel("shoulder angle [deg]")
plt.title("Damping ratio effect  (Computed Torque set-point)")
plt.legend(); plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("damping_response.png", dpi=95)
print("\n[저장] damping_response.png")
print("→ 빨강(ζ=0.2): 목표를 지나쳐 출렁임 / 초록(ζ=1): 깔끔 / 파랑(ζ=2): 느림")
