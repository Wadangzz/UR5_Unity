"""
PID 제어 데모 — 중력보상 없이 PID 만으로 목표 자세 유지.

τ = Kp·e + Ki·∫e dt + Kd·ė   (중력보상 항 없음)
중력을 따로 안 잡아주지만, I(적분)가 누적되며 중력을 대신 상쇄 →
정상상태 오차가 0 으로 수렴. (PD만이면 중력에 처져 오차가 남음)

plant = robot_math.Dynamics.forward_dynamics. 적분상태 ∫e 를 상태에 포함(18차원).
실행:  python pid_demo.py   (+ outputs/pid_response.png)
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from robot_math import Dynamics
import ur5_model as ur5
from paths import out

np.set_printoptions(precision=3, suppress=True)
ARGS = (ur5.GRAVITY, np.zeros(6), ur5.MLIST, ur5.GLIST, ur5.SLIST)

THETA_D = np.array([0.0, -np.pi / 3, 0.6, 0.0, 0.4, 0.0])    # 목표 (중력 부하 큼)
KP, KI, KD = 120.0, 60.0, 35.0
T_END = 6.0


def rhs(t, x):
    th, dth, Iacc = x[:6], x[6:12], x[12:18]
    e = THETA_D - th
    tau = KP * e + KI * Iacc - KD * dth          # PID (중력보상 없음, θ̇_d=0)
    ddth = Dynamics.forward_dynamics(th, dth, tau, *ARGS)
    return np.concatenate([dth, ddth, e])         # İ = e


sol = solve_ivp(rhs, (0, T_END), np.zeros(18),
                t_eval=np.linspace(0, T_END, 500), method="RK45", rtol=1e-7, atol=1e-9)
t = sol.t
TH, DTH, IACC = sol.y[:6].T, sol.y[6:12].T, sol.y[12:18].T
E = THETA_D - TH
err = np.linalg.norm(E, axis=1)

# 어깨(J2) 토크 분해
j = 1
P_t, I_t, D_t = KP * E[:, j], KI * IACC[:, j], -KD * DTH[:, j]
g_target = Dynamics.gravity_forces(THETA_D, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)[j]

print(f"목표 θ_d[deg] = {np.rad2deg(THETA_D)}")
print(f"최종 오차 = {np.rad2deg(np.abs(E[-1])).max():.4f} deg  → 정상상태 오차 ~0 (I 덕분)")
print(f"J2 정상상태 토크: PID 합 {(P_t+I_t+D_t)[-1]:.3f} ≈ 중력 g(θ_d) {g_target:.3f} N·m")
print(f"  → 이 중 I항이 {I_t[-1]:.3f} N·m 담당 (중력을 적분이 대신 상쇄)")

# --- 플롯 ---
fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))

# 1) 관절각 수렴
deg = np.rad2deg
for i, name in [(1, "J2 shoulder"), (2, "J3 elbow"), (4, "J5 wrist2")]:
    ln, = ax[0].plot(t, deg(TH[:, i]), label=name)
    ax[0].axhline(deg(THETA_D[i]), ls="--", lw=1, color=ln.get_color(), alpha=0.5)
ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("joint angle [deg]")
ax[0].set_title("PID set-point response (--- target)"); ax[0].legend(); ax[0].grid(alpha=0.3)

# 2) 추종오차 → 0
ax[1].plot(t, deg(err), color="tab:red")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("||error|| [deg]")
ax[1].set_title("Tracking error -> 0  (no gravity comp!)"); ax[1].grid(alpha=0.3)

# 3) J2 토크 분해 — I가 중력을 인계
ax[2].plot(t, P_t, label="P term")
ax[2].plot(t, I_t, label="I term", lw=2)
ax[2].plot(t, D_t, label="D term")
ax[2].plot(t, P_t + I_t + D_t, "k--", label="total", alpha=0.7)
ax[2].axhline(g_target, color="gray", ls=":", label="gravity g(θ_d)")
ax[2].set_xlabel("time [s]"); ax[2].set_ylabel("J2 torque [N·m]")
ax[2].set_title("Torque split: I term takes over gravity"); ax[2].legend(); ax[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(out("pid_response.png"), dpi=95)
print("\n[저장] outputs/pid_response.png")
