"""시뮬레이션 엔진.

두 종류의 plant 를 제공한다 (controller 이름으로 분기):
- 토크 입력 (`_run_torque`): PD / PID / Computed Torque / 임피던스.
  2차계 — forward_dynamics 로 θ̈ 적분 (폐루프가 stiff → Radau).
- 속도 입력 (`_run_velocity`): joint_velocity / resolved_rate (MR §11.3).
  1차계 — θ̇ 가 곧 제어출력, θ 만 적분(기구학 시뮬). stiff 아님 → RK45.

robot_math.Dynamics 는 Mlist/Glist/Slist 를 인자로 받으므로, 현재는 ur5_model
(기본 로봇)을 쓰지만 추후 URDF 로 일반화하기 쉽다.
"""
import numpy as np
from scipy.integrate import solve_ivp
from app.core.robot_math import Dynamics, SE3, SO3, quintic_time_scaling
from app.core import ur5_model as ur5

N = ur5.N
MODEL = (ur5.MLIST, ur5.GLIST, ur5.SLIST)

# --- 공통 적분/정착/발산 상수 ---
SETTLE_MAX = 2.0        # 모션 종료 후 평형 대기 상한(s) — 계산시간 단축
VEL_TOL = 0.02          # 정상상태(평형) 판정 속도 임계 rad/s
VEL_WIN = 0.2           # 그 속도 이하를 유지해야 하는 윈도우 s
THETA_MAX = 3 * np.pi   # 발산 안전망: 위치(rad), 오버슛보다 큼
OMEGA_MAX = 20.0        # 발산 안전망: 속도(rad/s), 정상 ~2 보다 큼

VELOCITY_CONTROLLERS = ("joint_velocity", "resolved_rate")


def run_simulation(waypoints, controller="computed_torque", gains=None,
                   gravity_comp=True, t_seg=1.2, hold=0.6, hz=30,
                   plant=None, ctrl=None, disturbance=None, traj_mode="joint"):
    """관절 경유점(rad)을 제어기로 순회. → {t, theta, tcp, error, torque, qdot, ...}.

    궤적생성(traj_mode)과 제어기는 직교(독립):
    - traj_mode='joint': 관절공간 quintic 보간(현행). 말단 경로는 곡선.
    - traj_mode='task' : 직교 SE(3) 직선보간 → IK 샘플 → θ_d. 말단 직선이지만
      특이점/작업영역밖에서 IK 실패·관절속도 폭발 시 그 지점까지 자르고 traj_error 보고.
    제어기는 모두 θ_d(t) 를 받으므로 어느 궤적이든 동일하게 추종한다.

    controller 가 속도제어/어드미턴스면 기구학 plant, 그 외는 동역학 plant 로 분기.
    plant/ctrl/disturbance 는 동역학 plant·어드미턴스에서만 의미가 있다.
    """
    gains = gains or {}
    WP = _prep_waypoints(waypoints)
    ref, T, traj_error = _build_ref(WP, t_seg, hold, traj_mode)
    if T <= 0:                                  # 궤적 생성 자체 실패(첫 점부터 불가)
        return _empty_result(WP, traj_error)

    if controller == "admittance":
        result = _run_admittance(WP, ref, T, gains, hz, disturbance)
    elif controller in VELOCITY_CONTROLLERS:
        result = _run_velocity(WP, ref, T, controller, gains, hz)
    else:
        result = _run_torque(WP, ref, T, controller, gains, gravity_comp, hz,
                             plant, ctrl, disturbance)
    result["traj_error"] = traj_error
    return result


# ----------------------------------------------------------------------
#  공유 헬퍼 (토크·속도 plant 공용)
# ----------------------------------------------------------------------
def _prep_waypoints(waypoints):
    WP = [np.asarray(w, dtype=float) for w in waypoints]
    if len(WP) < 2:
        WP = WP + WP
    return WP


def _make_ref(WP, t_seg, hold):
    """경유점 → 시간함수 ref(t)=(θ_d, θ̇_d, θ̈_d) (구간별 quintic + hold). (ref, T) 반환."""
    n_seg = len(WP) - 1
    seg = t_seg + hold
    T = n_seg * seg

    def ref(t):
        k = min(int(t // seg), n_seg - 1)
        loc = t - k * seg
        q0, q1 = WP[k], WP[k + 1]
        dq = q1 - q0
        if loc < t_seg:
            s, sd, sdd = quintic_time_scaling(loc, t_seg)
            return q0 + s * dq, sd * dq, sdd * dq
        return q1.copy(), np.zeros(N), np.zeros(N)

    return ref, T


def _se3_straight(X0, X1, s):
    """직교 직선 보간: 위치 선형 + 회전 측지선(SLERP 등가). s∈[0,1]."""
    Td = np.eye(4)
    Td[:3, 3] = (1 - s) * X0[:3, 3] + s * X1[:3, 3]
    Rrel = SO3.log(X0[:3, :3].T @ X1[:3, :3])   # 상대회전(회전벡터)
    Td[:3, :3] = X0[:3, :3] @ SO3.exp(s * Rrel)
    return Td


def _build_ref(WP, t_seg, hold, traj_mode):
    """궤적 생성 → (ref(t)→(θ_d,θ̇_d,θ̈_d), T, traj_error).

    joint: 관절 quintic(해석적, 항상 실행가능).
    task : 직교 SE(3) 직선보간 → IK 샘플(precompute) → θ_d 격자 보간.
           IK 비수렴(작업영역밖) 또는 관절속도 폭발(특이점) 시 그 지점까지 자르고
           traj_error 를 채운다(='실행 불가'를 정직하게 표면화).
    """
    if traj_mode != "task":
        ref, T = _make_ref(WP, t_seg, hold)
        return ref, T, None

    n_seg = len(WP) - 1
    seg = t_seg + hold
    T = n_seg * seg
    poses = [ur5.fk(w) for w in WP]             # 웨이포인트 SE(3)
    ts = np.linspace(0, T, int(T * 200) + 1)    # 조밀 격자(200 Hz)
    TH = np.zeros((len(ts), N))
    seed = WP[0].copy()
    traj_error = None
    n_ok = len(ts)
    for i, t in enumerate(ts):
        k = min(int(t // seg), n_seg - 1)
        loc = t - k * seg
        s = quintic_time_scaling(loc, t_seg)[0] if loc < t_seg else 1.0
        Xd = _se3_straight(poses[k], poses[k + 1], s)
        th, ok = ur5.ik(Xd, seed)
        if not ok:
            traj_error = f"직교 직선이 작업영역/특이점에 막힘 (t≈{t:.2f}s) — 실행 불가"
            n_ok = i
            break
        TH[i], seed = th, th
    ts, TH = ts[:n_ok], TH[:n_ok]
    if len(ts) < 2:
        return (lambda _t: (WP[0].copy(), np.zeros(N), np.zeros(N))), 0.0, traj_error

    dt = ts[1] - ts[0]
    DTH = np.gradient(TH, dt, axis=0)
    # 특이점 부근 관절속도 폭발 감지 → 그 지점까지 절단
    over = np.where(np.max(np.abs(DTH), axis=1) > OMEGA_MAX)[0]
    if traj_error is None and len(over):
        cut = max(over[0], 2)
        traj_error = f"특이점 부근 관절속도 폭발 (t≈{ts[cut - 1]:.2f}s) — 실행 불가"
        ts, TH, DTH = ts[:cut], TH[:cut], DTH[:cut]
    DDTH = np.gradient(DTH, dt, axis=0)
    T_eff = float(ts[-1])

    def ref(t):
        tc = min(max(t, 0.0), T_eff)
        thd = np.array([np.interp(tc, ts, TH[:, j]) for j in range(N)])
        dthd = np.array([np.interp(tc, ts, DTH[:, j]) for j in range(N)])
        ddthd = np.array([np.interp(tc, ts, DDTH[:, j]) for j in range(N)])
        return thd, dthd, ddthd

    return ref, T_eff, traj_error


def _empty_result(WP, traj_error):
    """궤적 생성 실패 시 빈 결과(재생할 것 없음 + 사유)."""
    return {
        "t": [], "theta": [], "tcp": [], "error": [], "torque": [], "qdot": [],
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
        "settle_time": None, "steady_state_error": None,
        "diverged": True, "traj_error": traj_error,
    }


def _blowup_event():
    """위치 또는 속도가 폭주하면 즉시 적분 종료(발산 적분 길어짐 방지)."""
    def blowup(t, x):
        return min(THETA_MAX - float(np.max(np.abs(x[:N]))),
                   OMEGA_MAX - float(np.max(np.abs(x[N:2 * N]))))
    blowup.terminal = True
    blowup.direction = -1
    return blowup


def _detect_settle(sol, T, err, vmax, hz):
    """정상상태(평형) 도달 판정. → (diverged, settle_time, steady_state_error).

    멈춤 = 모션(T) 이후 속도가 VEL_WIN 동안 VEL_TOL 이하 유지. 잔류오차가 있어도
    평형이면 멈춤(그 시점 오차 = 정상상태 오차). 끝까지 안 멈추고 오차 증가추세면 발산.
    """
    diverged = sol.status != 0          # 안전망 이벤트 발동 or 적분 실패
    if diverged:
        return True, None, None

    vw = max(1, int(VEL_WIN * hz))
    rest = None
    for i in range(len(sol.t)):
        if sol.t[i] < T:
            continue
        if np.max(vmax[max(0, i - vw):i + 1]) < VEL_TOL:
            rest = i
            break
    if rest is not None:
        return False, float(sol.t[rest]), float(err[rest])

    win = max(1, int(0.5 * hz))         # 평형 미도달 + 오차 증가추세면 발산
    if len(err) > win and err[-1] > err[-1 - win] * 1.05:
        return True, None, None
    return False, None, None


def _keep_idx(sol, settle_time, diverged, T):
    """반환 구간: 평형 시 그 시점+마진, 발산 시 전체, 미도달 시 모션+1s."""
    if settle_time is not None:
        keep = sol.t <= settle_time + 0.3
    elif diverged:
        keep = np.ones(len(sol.t), dtype=bool)
    else:
        keep = sol.t <= T + 1.0
    return np.where(keep)[0]


# ----------------------------------------------------------------------
#  동역학 plant — 토크 입력 (PD / PID / Computed Torque / 임피던스)
# ----------------------------------------------------------------------
def _run_torque(WP, ref, T, controller, gains, gravity_comp, hz,
                plant, ctrl, disturbance):
    Kp = float(gains.get("kp", 100.0))
    Kd = float(gains.get("kd", 20.0))
    Ki = float(gains.get("ki", 0.0))
    plant = plant or MODEL
    ctrl = ctrl or MODEL

    def g_ctrl(th):
        return Dynamics.gravity_forces(th, ur5.GRAVITY, *ctrl)

    use_I = controller in ("pid", "computed_torque")

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
        if controller == "impedance":
            # 직교(전체 pose) 임피던스: body twist 오차에 스프링-댐퍼.
            # V_err = log(T⁻¹·T_d) (body twist), τ = Jbᵀ(Kp·V_err − Kd·Jb·θ̇) + g
            Verr = SE3.log(np.linalg.inv(ur5.fk(th)) @ ur5.fk(th_d))[0]
            Jb = ur5.jacobian_body(th)
            F = Kp * Verr - Kd * (Jb @ dth)
            return Jb.T @ F + g_ctrl(th)
        tau = Kp * e + Kd * ed                  # pd
        return tau + g_ctrl(th) if gravity_comp else tau

    # 외란: base 프레임 외력(N)을 모션 종료(T) 후 끝단에 인가.
    # 컨트롤러는 모르는 힘(plant 의 Ftip) → PID/CT 는 버티고 임피던스는 밀린다.
    dist = np.asarray(disturbance, dtype=float) if disturbance else None
    has_dist = dist is not None and np.any(dist)

    def Ftip(t, th):
        if not has_dist or t < T:
            return np.zeros(6)
        f_b = ur5.fk(th)[:3, :3].T @ dist     # base→EE 프레임 힘
        return np.concatenate([np.zeros(3), f_b])  # [모멘트; 힘] (EE 프레임)

    def rhs(t, x):
        th, dth = x[:N], x[N:2 * N]
        I = x[2 * N:] if use_I else np.zeros(N)
        th_d, dth_d, ddth_d = ref(t)
        tau = torque(th, dth, th_d, dth_d, ddth_d, I)
        ddth = Dynamics.forward_dynamics(th, dth, tau, ur5.GRAVITY, Ftip(t, th), *plant)
        parts = [dth, ddth] + ([th_d - th] if use_I else [])
        return np.concatenate(parts)

    x0 = np.concatenate([WP[0], np.zeros(N)] + ([np.zeros(N)] if use_I else []))
    T_cap = T + SETTLE_MAX
    t_eval = np.linspace(0, T_cap, int(T_cap * hz) + 1)
    # PD/PID 폐루프는 stiff(고주파) → 명시적 RK45는 스텝 폭발(수십 초).
    # 암시적 Radau 로 ~20-30배 빠름. 허용오차는 시각화용이라 완화(30Hz 샘플엔 충분).
    sol = solve_ivp(rhs, (0, T_cap), x0, t_eval=t_eval, method="Radau",
                    rtol=1e-4, atol=1e-7, events=_blowup_event())

    TH, DTH = sol.y[:N].T, sol.y[N:2 * N].T
    II = sol.y[2 * N:].T if use_I else np.zeros((len(sol.t), N))
    # 추종오차: 임피던스는 task-space(TCP 위치 m), 그 외는 joint-space(rad).
    # (임피던스는 같은 pose 를 다른 관절 config 로 도달할 수 있어 joint 오차가 무의미)
    if controller == "impedance":
        err = np.array([np.linalg.norm(ur5.fk(TH[i])[:3, 3] - ur5.fk(ref(t)[0])[:3, 3])
                        for i, t in enumerate(sol.t)])
    else:
        err = np.array([np.linalg.norm(TH[i] - ref(t)[0]) for i, t in enumerate(sol.t)])
    vmax = (np.max(np.abs(DTH), axis=1) if len(sol.t)
            else np.zeros(0))               # 시각별 최대 관절속도

    diverged, settle_time, steady_state_error = _detect_settle(sol, T, err, vmax, hz)
    idx = _keep_idx(sol, settle_time, diverged, T)

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
        "qdot": [],
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
        "settle_time": settle_time,
        "steady_state_error": steady_state_error,
        "diverged": bool(diverged),
    }


# ----------------------------------------------------------------------
#  기구학 plant — 속도 입력 (resolved-rate, MR §11.3)
# ----------------------------------------------------------------------
def _run_velocity(WP, ref, T, controller, gains, hz):
    """속도제어: θ̇=제어출력, θ만 적분(1차계). 동역학·중력 불필요 → RK45.

    joint_velocity (§11.3.2): θ̇ = θ̇_d + Kp(θ_d−θ) + Ki∫(θ_d−θ)
    resolved_rate   (§11.3.3): Vb = Ad_{X⁻¹X_d}·V_d + Kp·Xe + Ki∫Xe,  θ̇ = J†_b·Vb
                               Xe = log(X⁻¹X_d) (body twist 오차)
    게인은 1차계라 스케일이 작다(≈ 1/시정수). kd 없음.
    """
    Kp = float(gains.get("kp", 2.0))
    Ki = float(gains.get("ki", 0.0))
    use_I = Ki != 0.0
    is_rr = controller == "resolved_rate"
    idim = 6 if is_rr else N             # 적분항 차원 (UR5 는 둘 다 6)

    def law(th, th_d, dth_d, I):
        """제어출력 θ̇ 와 적분항 integrand 반환."""
        if not is_rr:                                   # joint_velocity
            e = th_d - th
            return dth_d + Kp * e + Ki * I, e
        X, Xd = ur5.fk(th), ur5.fk(th_d)
        Tdiff = np.linalg.inv(X) @ Xd
        Xe = SE3.log(Tdiff)[0]                          # body twist 오차
        Vd = ur5.jacobian_body(th_d) @ dth_d           # 피드포워드 트위스트(frame {d})
        Vb = SE3.Adjoint(Tdiff) @ Vd + Kp * Xe + Ki * I
        return ur5.dls_inv(ur5.jacobian_body(th)) @ Vb, Xe

    def rhs(t, x):
        th = x[:N]
        I = x[N:N + idim] if use_I else np.zeros(idim)
        th_d, dth_d, _ = ref(t)
        qd, integrand = law(th, th_d, dth_d, I)
        return np.concatenate([qd] + ([integrand] if use_I else []))

    x0 = np.concatenate([WP[0]] + ([np.zeros(idim)] if use_I else []))
    T_cap = T + SETTLE_MAX
    t_eval = np.linspace(0, T_cap, int(T_cap * hz) + 1)

    # 기구학 1차계는 stiff 아님 → RK45. 위치만 폭주 가드(속도는 제어식이 유계).
    def blowup(t, x):
        return THETA_MAX - float(np.max(np.abs(x[:N])))
    blowup.terminal = True
    blowup.direction = -1
    sol = solve_ivp(rhs, (0, T_cap), x0, t_eval=t_eval, method="RK45",
                    rtol=1e-5, atol=1e-8, events=blowup)

    TH = sol.y[:N].T
    II = sol.y[N:N + idim].T if use_I else np.zeros((len(sol.t), idim))

    # 전 구간 θ̇(=제어출력)·오차 재계산 → 정착판정용 vmax 확보
    QD, err = [], []
    for i, t in enumerate(sol.t):
        th_d, dth_d, _ = ref(t)
        qd, _ = law(TH[i], th_d, dth_d, II[i])
        QD.append(qd)
        if is_rr:                                       # resolved-rate=task 오차(m)
            err.append(float(np.linalg.norm(ur5.fk(TH[i])[:3, 3] - ur5.fk(th_d)[:3, 3])))
        else:                                           # joint 오차(rad)
            err.append(float(np.linalg.norm(TH[i] - th_d)))
    QD = np.array(QD) if len(QD) else np.zeros((0, N))
    err = np.array(err)
    vmax = np.max(np.abs(QD), axis=1) if len(QD) else np.zeros(0)

    diverged, settle_time, steady_state_error = _detect_settle(sol, T, err, vmax, hz)
    idx = _keep_idx(sol, settle_time, diverged, T)

    return {
        "t": sol.t[idx].tolist(),
        "theta": TH[idx].tolist(),
        "tcp": [ur5.fk(TH[i])[:3, 3].tolist() for i in idx],
        "error": [err[i] for i in idx],
        "torque": [],
        "qdot": QD[idx].tolist(),
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
        "settle_time": settle_time,
        "steady_state_error": steady_state_error,
        "diverged": bool(diverged),
    }


# ----------------------------------------------------------------------
#  어드미턴스 (MR §11.7.2) — 외력을 '센싱'해 움직임 생성 (임피던스의 쌍)
# ----------------------------------------------------------------------
def _run_admittance(WP, ref, T, gains, hz, disturbance):
    """어드미턴스: 가상 질량-스프링-댐퍼를 시뮬레이션해 외력→움직임을 만든다.

    M·Δẍ + B·Δẋ + K·Δp = f_ext  (base 프레임 3D 병진, MR 식 11.66).
    그 가상 변위 Δp 를 nominal setpoint 에 더한 목표 pose 를 resolved-rate 로 추종
    (모션 plant — 임피던스가 토크 plant 인 것과 대칭: 어드미턴스 로봇=모션제어).
    외란을 인가하면 로봇이 그 힘에 능동적으로 양보하며 움직인다(정상상태 Δp=f/K).
    힘센서 없는 시뮬에선 외란(이미 아는 값)을 f_ext 로 재사용. M>0 라 2차 응답(오버슛).
    """
    M = max(float(gains.get("m", 2.0)), 0.1)    # 가상 질량 kg (0 방지)
    B = float(gains.get("b", 40.0))             # 가상 감쇠 N·s/m
    K = float(gains.get("k", 600.0))            # 가상 강성 N/m
    KP_TRACK = 20.0                            # 내부 모션 추종(resolved-rate) 게인
    # ↑ 가상 동역학(ωn=√(K/M))보다 빨라야 M-B-K 응답(오버슛 등)이 가려지지 않음

    dist = np.asarray(disturbance, dtype=float) if disturbance else None
    has_dist = dist is not None and np.any(dist)

    def f_ext(t):                               # 외란 = 센싱된 외력 (모션 종료 후 인가)
        return dist if (has_dist and t >= T) else np.zeros(3)

    def rhs(t, x):
        th, dp, dv = x[:N], x[N:N + 3], x[N + 3:N + 6]
        T_set = ur5.fk(ref(t)[0])                      # nominal setpoint pose
        da = (f_ext(t) - B * dv - K * dp) / M          # 가상 어드미턴스 동역학
        T_d = T_set.copy()
        T_d[:3, 3] = T_set[:3, 3] + dp                 # 목표 = setpoint + 가상 변위
        Xe = SE3.log(np.linalg.inv(ur5.fk(th)) @ T_d)[0]
        qd = ur5.dls_inv(ur5.jacobian_body(th)) @ (KP_TRACK * Xe)
        return np.concatenate([qd, dv, da])

    x0 = np.concatenate([WP[0], np.zeros(6)])
    T_cap = T + SETTLE_MAX
    t_eval = np.linspace(0, T_cap, int(T_cap * hz) + 1)

    def blowup(t, x):
        return THETA_MAX - float(np.max(np.abs(x[:N])))
    blowup.terminal = True
    blowup.direction = -1
    sol = solve_ivp(rhs, (0, T_cap), x0, t_eval=t_eval, method="RK45",
                    rtol=1e-5, atol=1e-8, events=blowup)

    TH = sol.y[:N].T
    QD, err = [], []
    for i, t in enumerate(sol.t):
        T_set = ur5.fk(ref(t)[0])
        dp = sol.y[N:N + 3, i]
        T_d = T_set.copy()
        T_d[:3, 3] = T_set[:3, 3] + dp
        Xe = SE3.log(np.linalg.inv(ur5.fk(TH[i])) @ T_d)[0]
        QD.append(ur5.dls_inv(ur5.jacobian_body(TH[i])) @ (KP_TRACK * Xe))
        # 추종오차 = 실제 EE 가 nominal setpoint 에서 벗어난 변위 (= 컴플라이언스)
        err.append(float(np.linalg.norm(ur5.fk(TH[i])[:3, 3] - T_set[:3, 3])))
    QD = np.array(QD) if len(QD) else np.zeros((0, N))
    err = np.array(err)
    vmax = np.max(np.abs(QD), axis=1) if len(QD) else np.zeros(0)

    # 정착: 외란은 모션 종료(T) 후 인가되므로, '한 번 흔들린 뒤(excited)' 평형을 찾는다.
    # (인가 순간 로봇이 정지해 있어 곧바로 정착으로 오판하는 것 방지)
    diverged = sol.status != 0
    settle_time = steady_state_error = None
    if not diverged and len(sol.t):
        vw = max(1, int(VEL_WIN * hz))
        excited = False
        for i in range(len(sol.t)):
            if sol.t[i] < T:
                continue
            if vmax[i] >= VEL_TOL:
                excited = True
            if excited and np.max(vmax[max(0, i - vw):i + 1]) < VEL_TOL:
                settle_time = float(sol.t[i])
                steady_state_error = float(err[i])
                break

    idx = _keep_idx(sol, settle_time, diverged, T)

    return {
        "t": sol.t[idx].tolist(),
        "theta": TH[idx].tolist(),
        "tcp": [ur5.fk(TH[i])[:3, 3].tolist() for i in idx],
        "error": [err[i] for i in idx],
        "torque": [],
        "qdot": QD[idx].tolist(),
        "waypoints_tcp": [ur5.fk(w)[:3, 3].tolist() for w in WP],
        "settle_time": settle_time,
        "steady_state_error": steady_state_error,
        "diverged": bool(diverged),
    }
