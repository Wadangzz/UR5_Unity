"""시뮬레이션 엔진 — 제어기(PD/PID/CT) + forward_dynamics plant (solve_ivp).

robot_math.Dynamics 는 Mlist/Glist/Slist 를 인자로 받으므로, 현재는 ur5_model
(기본 로봇)을 쓰지만 추후 URDF 로 일반화하기 쉽다.
"""
import numpy as np
from scipy.integrate import solve_ivp
from app.core.robot_math import Dynamics, quintic_time_scaling
from app.core import ur5_model as ur5

N = ur5.N
MODEL = (ur5.MLIST, ur5.GLIST, ur5.SLIST)


def controller_specs():
    def p(k, d, lo, hi):
        return {"key": k, "default": d, "min": lo, "max": hi}
    return [
        {"name": "pd", "label": "PD + 중력보상",
         "params": [p("kp", 100, 0, 400), p("kd", 20, 0, 100)]},
        {"name": "pid", "label": "PID",
         "params": [p("kp", 120, 0, 400), p("ki", 60, 0, 300), p("kd", 35, 0, 100)]},
        {"name": "computed_torque", "label": "Computed Torque",
         "params": [p("kp", 100, 0, 400), p("kd", 20, 0, 100)]},
    ]


def run_simulation(waypoints, controller="computed_torque", gains=None,
                   gravity_comp=True, t_seg=1.2, hold=0.6, hz=30,
                   plant=None, ctrl=None):
    """관절 경유점(rad)을 제어기로 순회. → {t, theta, tcp, error, torque, waypoints_tcp}.

    plant=진짜 로봇 모델, ctrl=컨트롤러가 아는 모델. 둘 다 None 이면 공칭 MODEL
    (plant=ctrl → 이상적). realism.build_models 로 페이로드·모델오차를 주입할 수 있다.
    """
    gains = gains or {}
    Kp = float(gains.get("kp", 100.0))
    Kd = float(gains.get("kd", 20.0))
    Ki = float(gains.get("ki", 60.0))
    plant = plant or MODEL
    ctrl = ctrl or MODEL

    def g_ctrl(th):
        return Dynamics.gravity_forces(th, ur5.GRAVITY, *ctrl)

    WP = [np.asarray(w, dtype=float) for w in waypoints]
    if len(WP) < 2:
        WP = WP + WP
    n_seg = len(WP) - 1
    seg = t_seg + hold
    T = n_seg * seg
    use_I = controller == "pid"

    def ref(t):
        k = min(int(t // seg), n_seg - 1)
        loc = t - k * seg
        q0, q1 = WP[k], WP[k + 1]
        dq = q1 - q0
        if loc < t_seg:
            s, sd, sdd = quintic_time_scaling(loc, t_seg)
            return q0 + s * dq, sd * dq, sdd * dq
        return q1.copy(), np.zeros(N), np.zeros(N)

    def torque(th, dth, th_d, dth_d, ddth_d, I):
        e, ed = th_d - th, dth_d - dth
        if controller == "computed_torque":
            aq = ddth_d + Kd * ed + Kp * e
            M = Dynamics.mass_matrix(th, *ctrl)
            c = Dynamics.coriolis_forces(th, dth, *ctrl)
            return M @ aq + c + g_ctrl(th)
        if controller == "pid":
            tau = Kp * e + Ki * I + Kd * ed
            return tau + g_ctrl(th) if gravity_comp else tau
        tau = Kp * e + Kd * ed                  # pd
        return tau + g_ctrl(th) if gravity_comp else tau

    def rhs(t, x):
        th, dth = x[:N], x[N:2 * N]
        I = x[2 * N:] if use_I else np.zeros(N)
        th_d, dth_d, ddth_d = ref(t)
        tau = torque(th, dth, th_d, dth_d, ddth_d, I)
        ddth = Dynamics.forward_dynamics(th, dth, tau, ur5.GRAVITY, np.zeros(N), *plant)
        parts = [dth, ddth] + ([th_d - th] if use_I else [])
        return np.concatenate(parts)

    x0 = np.concatenate([WP[0], np.zeros(N)] + ([np.zeros(N)] if use_I else []))
    t_eval = np.linspace(0, T, int(T * hz) + 1)
    sol = solve_ivp(rhs, (0, T), x0, t_eval=t_eval, method="RK45", rtol=1e-6, atol=1e-9)

    TH, DTH = sol.y[:N].T, sol.y[N:2 * N].T
    II = sol.y[2 * N:].T if use_I else np.zeros((len(sol.t), N))

    tcp, error, torque_log = [], [], []
    for i, t in enumerate(sol.t):
        th_d, dth_d, ddth_d = ref(t)
        tcp.append(ur5.fk(TH[i])[:3, 3].tolist())
        error.append(float(np.linalg.norm(TH[i] - th_d)))
        torque_log.append([float(v) for v in torque(TH[i], DTH[i], th_d, dth_d, ddth_d, II[i])])

    return {
        "t": sol.t.tolist(),
        "theta": TH.tolist(),
        "tcp": tcp,
        "error": error,
        "torque": torque_log,
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
    }
