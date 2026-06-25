"""실시간 인터랙티브 제어 세션 (grab & push).

기존 sim.py 는 '한 번에 precompute → θ(t) 재생' 인 반면, 여기서는 매 틱마다 외부에서
주는 잡기힘(grab force, EE 에 가하는 base 프레임 외력)에 로봇이 실시간 반응한다.
plant 는 동일한 numba RNE(forward_dynamics) — fd ~0.3ms 라 100Hz 루프가 가능하다.

토크 plant 제어기(PD/PID/CT/임피던스)를 setpoint 유지 모드로 돌린다:
  - 위치제어(PD/PID/CT): 외력에 버틴다(뻣뻣).  임피던스: 양보한다(물렁).
  → 같은 외력에 '싸우는 제어기 vs 양보하는 제어기' 대비가 핵심 데모.

발산 처리 두 모드:
  - 'honest' : 추가 안전층 없음. 폭주하면 감지·정지(실로봇 보호정지) + diverged.
  - 'safe'   : 진짜 안전층(토크 포화·속도/관절 리밋)으로 봉쇄. 폭발 대신 한계서 멈춤.
               개입하면 flags 로 알린다(조용히 '정상'으로 보이지 않게).
  ※ 'safe' 는 '마법으로 성공'이 아니라 '안전하게 실패' — 나쁜 게인은 여전히 작업 실패.
"""
import numpy as np

from app.core import dynamics_fast as df
from app.core.robot_math import SE3
from app.sim import OMEGA_MAX, THETA_MAX

# 물리 적분 dt 상한(s). explicit Euler 는 조건부 안정이라 감쇠항 Kd·Jb·θ̇ 가 stiff
# → 기본 게인(Kp600 Kd60)이 안정하려면 ~2kHz 필요. 스트리밍 dt 를 이 값으로 서브스텝.
# (이보다 게인을 크게 올리면 그래도 발산 = 이산 불안정 시연 — honest 모드가 잡아냄)
PHYS_DT = 0.0005            # 2kHz 물리

# safe 모드 기본 안전 한계 (UR5 기준 보수값)
TAU_MAX_SAFE = 150.0        # 관절 토크 포화 N·m
VEL_MAX_SAFE = 5.0          # 관절 속도 클램프 rad/s


class InteractiveSession:
    """한 번에 한 로봇의 실시간 grab & push 세션. step() 을 루프에서 호출한다."""

    def __init__(self, robot, controller="impedance", gains=None,
                 mode="honest", setpoint=None):
        self.robot = robot
        self.controller = controller
        g = gains or {}
        self.Kp = float(g.get("kp", 600.0))
        self.Kd = float(g.get("kd", 60.0))
        self.Ki = float(g.get("ki", 0.0))
        self.mode = mode
        sp = robot.ready if setpoint is None else np.asarray(setpoint, float)
        self.th_d = sp.copy()                    # 유지 setpoint
        self.T_d = robot.fk(sp)                  # setpoint pose (임피던스용, 상수 → 선계산)
        self.th = sp.copy()
        self.dth = np.zeros(robot.n)
        self.I = np.zeros(robot.n)               # 적분(PID/CT)
        self.dead = False                        # 발산 후 정지(보호정지)
        self.pm = df.stack_model(*robot.dyn_args)   # plant 모델(스택)
        self.cm = self.pm                        # 컨트롤러 모델(=plant, 불확실성 없음)
        if mode == "safe":
            self.tau_max = TAU_MAX_SAFE
            self.vel_max = VEL_MAX_SAFE
            if robot.joint_limits is not None:
                lim = np.asarray(robot.joint_limits, float)
                self.lo, self.hi = lim[:, 0].copy(), lim[:, 1].copy()
            else:                                # 상수빌드 UR5 등 한계 미상 → 넉넉히
                self.lo = np.full(robot.n, -2 * np.pi)
                self.hi = np.full(robot.n, 2 * np.pi)
        else:
            self.tau_max = self.vel_max = self.lo = self.hi = None

    def _g(self, th):
        return df.gravity_forces(th, self.robot.gravity, *self.cm)

    def _control_torque(self, th, dth):
        """setpoint 유지 제어토크 (dth_d=0). _run_torque 의 torque 법칙과 동일 형태."""
        Kp, Kd, Ki = self.Kp, self.Kd, self.Ki
        e, ed = self.th_d - th, -dth
        c = self.controller
        if c == "computed_torque":
            aq = Kd * ed + Kp * e + Ki * self.I
            M = df.mass_matrix(th, *self.cm)
            cor = df.coriolis_forces(th, dth, *self.cm)
            return M @ aq + cor + self._g(th)
        if c == "pid":
            return Kp * e + Ki * self.I + Kd * ed + self._g(th)
        if c == "impedance":
            # 직교 임피던스: τ = Jbᵀ(Kp·Verr − Kd·Jb·θ̇) + g, Verr=log(T⁻¹·T_d)
            Verr = SE3.log(np.linalg.inv(self.robot.fk(th)) @ self.T_d)[0]
            Jb = self.robot.jacobian_body(th)
            return Jb.T @ (Kp * Verr - Kd * (Jb @ dth)) + self._g(th)
        return Kp * e + Kd * ed + self._g(th)    # pd

    def _result(self, flags):
        ee = self.robot.fk(self.th)
        return {
            "theta": self.th.tolist(),
            "ee_pos": ee[:3, 3].tolist(),
            "flags": flags,
            "diverged": self.dead,
        }

    def step(self, grab_force=None, dt=0.01, substeps=None):
        """1틱 전진(스트리밍 dt). 내부에서 물리를 ≥2kHz(PHYS_DT)로 서브스텝 적분 —
        전진오일러는 조건부 안정이라 한 방이면 고게인서 수치 발산(실시간 시뮬 표준 기법).
        grab_force = EE 에 가하는 base 프레임 외력(3-vec, N).
        → {theta, ee_pos, flags, diverged}."""
        if self.dead:                            # 보호정지 후엔 동결
            return self._result(["protective_stop"])
        if substeps is None:
            substeps = max(1, int(np.ceil(dt / PHYS_DT)))   # 물리 ≥ 2kHz
        h = dt / substeps
        flags = set()
        for _ in range(substeps):
            self._substep(grab_force, h, flags)
            if self.dead:
                break
        return self._result(sorted(flags))

    def _substep(self, grab_force, dt, flags):
        """물리 한 스텝(작은 dt). 제어→포화→plant→리밋→발산판정. flags 누적(set)."""
        th, dth = self.th, self.dth
        tau = self._control_torque(th, dth)

        if self.tau_max is not None:             # safe: 토크 포화
            clip = np.clip(tau, -self.tau_max, self.tau_max)
            if not np.array_equal(clip, tau):
                flags.add("saturation")
            tau = clip

        # 잡기힘 = 로봇에 가하는 외력 → Ftip=−R·ᵀgrab (외란 부호 규약과 동일)
        grab = np.zeros(3) if grab_force is None else np.asarray(grab_force, float)
        if np.any(grab):
            f_ee = -self.robot.fk(th)[:3, :3].T @ grab
            Ftip = np.concatenate([np.zeros(3), f_ee])
        else:
            Ftip = np.zeros(6)

        ddth = df.forward_dynamics(th, dth, tau, self.robot.gravity, Ftip, *self.pm)
        dth = dth + ddth * dt                    # semi-implicit Euler
        th = th + dth * dt

        if self.controller in ("pid", "computed_torque"):
            self.I = self.I + (self.th_d - th) * dt

        if self.mode == "safe":                  # 속도·관절 리밋 클램프
            vclip = np.clip(dth, -self.vel_max, self.vel_max)
            if not np.array_equal(vclip, dth):
                flags.add("vel_limit")
            dth = vclip
            tclip = np.clip(th, self.lo, self.hi)
            if not np.array_equal(tclip, th):
                flags.add("joint_limit")
                dth = np.where(tclip != th, 0.0, dth)    # 한계서 정지
            th = tclip

        self.th, self.dth = th, dth
        # 발산 안전망(양 모드): 상태 폭주 → 보호정지
        if (not np.all(np.isfinite(th))
                or np.max(np.abs(th)) > THETA_MAX
                or np.max(np.abs(dth)) > OMEGA_MAX):
            self.dead = True
            flags.add("protective_stop")
