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
         "params": [p("kp", 100, 0, 400), p("ki", 0, 0, 200), p("kd", 20, 0, 100)]},
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
    Ki = float(gains.get("ki", 0.0))
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
    use_I = controller in ("pid", "computed_torque")

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
            aq = ddth_d + Kd * ed + Kp * e + Ki * I
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

    # 정착까지: 모션(T) 종료 후 상한(SETTLE_MAX)까지 적분.
    # 발산 안전망: |θ| 가 타당한 오버슛보다 훨씬 큰 THETA_MAX 돌파 시 즉시 종료
    # (inf/NaN 방지용. 발산 '판정'은 아래 정착 실패 + 증가추세로 한다).
    SETTLE_MAX = 2.0                        # 평형 대기 상한(s) — 계산시간 단축
    VEL_TOL = 0.02                          # 정상상태(평형) 판정 속도 임계 rad/s
    VEL_WIN = 0.2                           # 그 속도 이하를 유지해야 하는 윈도우 s
    THETA_MAX = 3 * np.pi                   # 발산 안전망: 위치(rad), 오버슛보다 큼
    OMEGA_MAX = 20.0                        # 발산 안전망: 속도(rad/s), 정상 ~2 보다 큼
    T_cap = T + SETTLE_MAX

    # 위치 또는 속도가 폭주하면 즉시 종료(발산 적분이 길어지는 것 방지)
    def blowup(t, x):
        return min(THETA_MAX - float(np.max(np.abs(x[:N]))),
                   OMEGA_MAX - float(np.max(np.abs(x[N:2 * N]))))
    blowup.terminal = True
    blowup.direction = -1

    t_eval = np.linspace(0, T_cap, int(T_cap * hz) + 1)
    # PD/PID 폐루프는 stiff(고주파) → 명시적 RK45는 스텝 폭발(수십 초).
    # 암시적 Radau 로 ~20-30배 빠름. 허용오차는 시각화용이라 완화(30Hz 샘플엔 충분).
    sol = solve_ivp(rhs, (0, T_cap), x0, t_eval=t_eval, method="Radau",
                    rtol=1e-4, atol=1e-7, events=blowup)

    TH, DTH = sol.y[:N].T, sol.y[N:2 * N].T
    II = sol.y[2 * N:].T if use_I else np.zeros((len(sol.t), N))
    err = np.array([np.linalg.norm(TH[i] - ref(t)[0]) for i, t in enumerate(sol.t)])
    vmax = (np.max(np.abs(DTH), axis=1) if len(sol.t)
            else np.zeros(0))               # 시각별 최대 관절속도

    # 멈춤 = 정상상태(평형) 도달: 모션(T) 이후 속도가 VEL_WIN 동안 VEL_TOL 이하 유지.
    # → 잔류오차가 있어도(PD+페이로드 등) 평형이면 멈춤. 그 시점 오차 = 정상상태 오차.
    diverged = sol.status != 0              # 안전망 이벤트 발동 or 적분 실패
    settle_time = None
    steady_state_error = None
    if not diverged:
        vw = max(1, int(VEL_WIN * hz))
        rest = None
        for i in range(len(sol.t)):
            if sol.t[i] < T:
                continue
            if np.max(vmax[max(0, i - vw):i + 1]) < VEL_TOL:
                rest = i
                break
        if rest is not None:
            settle_time = float(sol.t[rest])
            steady_state_error = float(err[rest])
        else:
            # 평형 미도달: 끝까지 안 멈춤 + 오차 증가추세면 발산
            win = max(1, int(0.5 * hz))
            if len(err) > win and err[-1] > err[-1 - win] * 1.05:
                diverged = True

    # 반환 구간: 평형 도달 시 그 시점+마진, 발산 시 부분 전체, 미도달 시 모션+1s
    if settle_time is not None:
        keep = sol.t <= settle_time + 0.3
    elif diverged:
        keep = np.ones(len(sol.t), dtype=bool)
    else:
        keep = sol.t <= T + 1.0
    idx = np.where(keep)[0]

    tcp, error, torque_log = [], [], []
    for i in idx:
        th_d, dth_d, ddth_d = ref(sol.t[i])
        tcp.append(ur5.fk(TH[i])[:3, 3].tolist())
        error.append(float(err[i]))
        torque_log.append([float(v) for v in torque(TH[i], DTH[i], th_d, dth_d, ddth_d, II[i])])

    return {
        "t": sol.t[idx].tolist(),
        "theta": TH[idx].tolist(),
        "tcp": tcp,
        "error": error,
        "torque": torque_log,
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
        "settle_time": settle_time,
        "steady_state_error": steady_state_error,
        "diverged": bool(diverged),
    }
