"""
로봇 기구학 수학 라이브러리 (screw theory / Lie group 기반)

구조는 수학적 객체에 따라 3계층으로 분리되어 있습니다:

    SO3        : 회전 군 SO(3) 과 그 Lie 대수 so(3)   (회전만)
    SE3        : 강체변환 군 SE(3) 과 그 Lie 대수 se(3) (회전 + 병진), 내부에서 SO3 사용
    Kinematics : 위 두 객체를 조합한 알고리즘 계층 (Jacobian, IK)

군(group, 대문자)은 "자세/위치", 대수(algebra, 소문자)는 "속도/접선"을 나타내며,
둘을 잇는 다리가 exp / log 입니다.

    so(3) --exp--> SO(3)            se(3) --exp--> SE(3)
       (hat) <--log-- (R)        (twist) <--log-- (T)

표기는 Modern Robotics (Lynch & Park) 컨벤션을 따릅니다.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


# ======================================================================
#  SO3 — 회전 군 SO(3) 과 Lie 대수 so(3)
# ======================================================================
class SO3:
    """3D 회전. 군 원소 = 3x3 회전행렬 R, 대수 원소 = 회전벡터 ω∈ℝ³ (≅ so(3))."""

    @staticmethod
    def hat(omega):
        """hat 사상: ℝ³ → so(3). 벡터를 반대칭(skew-symmetric) 행렬로."""
        return np.array([[0, -omega[2], omega[1]],
                         [omega[2], 0, -omega[0]],
                         [-omega[1], omega[0], 0]])

    @staticmethod
    def vee(omega_hat):
        """vee 사상: so(3) → ℝ³. hat 의 역연산."""
        return np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])

    @staticmethod
    def exp(omega):
        """exp: so(3) → SO(3). 회전벡터(radian) → 회전행렬 (Rodrigues 공식)."""
        theta = np.linalg.norm(omega)
        if theta < 1e-12:
            return np.eye(3)
        k_hat = SO3.hat(np.asarray(omega) / theta)
        return (np.eye(3)
                + np.sin(theta) * k_hat
                + (1 - np.cos(theta)) * (k_hat @ k_hat))

    @staticmethod
    def log(R_mat):
        """log: SO(3) → so(3). 회전행렬 → 회전벡터 (radian)."""
        cos_theta = np.clip((np.trace(R_mat) - 1) / 2, -1.0, 1.0)
        theta = np.arccos(cos_theta)
        if np.abs(theta) < 1e-12:
            return np.zeros(3)
        return theta / (2 * np.sin(theta)) * SO3.vee(R_mat - R_mat.T)

    # --- 표현(representation) 변환 ---
    @staticmethod
    def to_quat(R_mat):
        """SO(3) → quaternion [x, y, z, w]."""
        return R.from_matrix(R_mat).as_quat()

    @staticmethod
    def to_euler(R_mat, degrees=True):
        """SO(3) → Euler 각 [roll, pitch, yaw] ('xyz')."""
        return R.from_matrix(R_mat).as_euler('xyz', degrees=degrees)

    @staticmethod
    def from_quat(quat):
        """quaternion → SO(3)."""
        return R.from_quat(quat).as_matrix()

    @staticmethod
    def from_euler(rpy, degrees=True):
        """Euler 각 [roll, pitch, yaw] → SO(3)."""
        return R.from_euler('xyz', rpy, degrees=degrees).as_matrix()


# ======================================================================
#  SE3 — 강체변환 군 SE(3) 과 Lie 대수 se(3)
# ======================================================================
class SE3:
    """강체변환. 군 원소 = 4x4 변환행렬 T, 대수 원소 = twist V=(ω,v)∈ℝ⁶ (≅ se(3))."""

    @staticmethod
    def hat(V):
        """hat 사상: ℝ⁶ → se(3). twist → 4x4 행렬."""
        omega, v = V[:3], V[3:6]
        T = np.zeros((4, 4))
        T[:3, :3] = SO3.hat(omega)
        T[:3, 3] = v
        return T

    @staticmethod
    def vee(V_hat):
        """vee 사상: se(3) → ℝ⁶."""
        return np.concatenate([SO3.vee(V_hat[:3, :3]), V_hat[:3, 3]])

    @staticmethod
    def exp(degree, screw, joint='R', unit='degree'):
        """
        exp: se(3) → SE(3). 관절 screw 축과 변위 → 4x4 변환행렬.
        :param degree: 회전각(R) 또는 직선변위(P)
        :param screw : 6D screw 축 [ω(3), v(3)]
        :param joint : 'R'(회전) 또는 'P'(직동)
        :param unit  : 'degree' 또는 'radian'
        """
        if unit == 'degree':
            theta = np.deg2rad(degree) if joint == 'R' else degree
        elif unit == 'radian':
            theta = degree if joint == 'R' else np.rad2deg(degree)

        omega = screw[:3]
        v = screw[3:6]

        omega_hat = SO3.hat(omega)
        omega_hat_sq = omega_hat @ omega_hat

        rot = np.eye(3) + np.sin(theta) * omega_hat + (1 - np.cos(theta)) * omega_hat_sq
        G_theta = (np.eye(3) * theta
                   + (1 - np.cos(theta)) * omega_hat
                   + (theta - np.sin(theta)) * omega_hat_sq)
        p = G_theta @ v

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = p
        return T

    @staticmethod
    def exp6(twist):
        """
        exp: se(3) → SE(3) (일반형). 6D twist 전체를 지수사상.
        SE3.exp 과 달리 각도가 인자로 분리돼 있지 않고, twist 의
        회전부 노름 ||ω|| 가 곧 θ 이다 (동역학 RNE 내부에서 사용).
        :param twist: 6D twist [ω(3), v(3)] (= screw·θ)
        """
        omega = np.asarray(twist[:3], dtype=float)
        v = np.asarray(twist[3:6], dtype=float)
        theta = np.linalg.norm(omega)

        T = np.eye(4)
        if theta < 1e-12:          # 순수 병진 (prismatic 등)
            T[:3, 3] = v
            return T

        w_hat = SO3.hat(omega / theta)
        w_hat_sq = w_hat @ w_hat
        T[:3, :3] = np.eye(3) + np.sin(theta) * w_hat + (1 - np.cos(theta)) * w_hat_sq
        G = (np.eye(3) * theta
             + (1 - np.cos(theta)) * w_hat
             + (theta - np.sin(theta)) * w_hat_sq)
        T[:3, 3] = G @ (v / theta)
        return T

    @staticmethod
    def log(T_bd):
        """
        log: SE(3) → se(3). 변환행렬 → twist.
        θ≈0(회전 없음)·θ≈π(sinθ=0) 특이 케이스를 해석적으로 처리하여
        결정적(deterministic)으로 동작한다. (난수 사용 없음)
        :param T_bd: 4x4 변환행렬
        :return: (twist*θ, screw(6D 단위), θ(radian))
        """
        R_bd = T_bd[:3, :3]
        p = T_bd[:3, 3]
        cos_theta = np.clip((np.trace(R_bd) - 1) / 2, -1.0, 1.0)
        theta = np.arccos(cos_theta)

        if theta < 1e-10:
            # 회전 ≈ 0 → 순수 병진. twist = [0, p], θ = ||p||
            d = np.linalg.norm(p)
            if d < 1e-12:
                return np.zeros(6), np.zeros(6), 0.0
            screw = np.concatenate([np.zeros(3), p / d])
            return np.concatenate([np.zeros(3), p]), screw, d

        if np.abs(theta - np.pi) < 1e-6:
            # θ≈π → sinθ≈0. R 대각 성분에서 회전축을 안정적으로 추출
            diag = np.array([R_bd[0, 0], R_bd[1, 1], R_bd[2, 2]])
            k = int(np.argmax(diag))
            omega = (R_bd[:, k] + np.eye(3)[:, k]) / np.sqrt(2 * (1 + diag[k]))
        else:
            omega = SO3.vee(R_bd - R_bd.T) / (2 * np.sin(theta))

        omega_hat = SO3.hat(omega)
        A_inv = (np.eye(3) / theta
                 - 0.5 * omega_hat
                 + (1 / theta - 0.5 / np.tan(theta / 2)) * (omega_hat @ omega_hat))
        v = A_inv @ p
        screw = np.concatenate([omega, v])
        return theta * screw, screw, theta

    @staticmethod
    def Adjoint(T):
        """
        big adjoint Ad_T (6x6). twist 를 다른 좌표계로 옮김 (se(3) 위에 작용).
        :param T: 4x4 변환행렬 ∈ SE(3)
        """
        rot = T[:3, :3]
        p = T[:3, 3]
        adj = np.zeros((6, 6))
        adj[:3, :3] = rot
        adj[3:6, 3:6] = rot
        adj[3:6, :3] = SO3.hat(p) @ rot
        return adj

    @staticmethod
    def ad(V):
        """
        small adjoint ad_V (6x6) = se(3) Lie bracket 연산자.
        ad_V(W) = [V, W]. 동역학의 코리올리/원심 비선형 항 (ω×Iω)을 만든다.
        :param V: 6D twist [ω(3), v(3)]
        """
        omega, v = V[:3], V[3:6]
        Z = np.zeros((3, 3))
        return np.block([[SO3.hat(omega), Z],
                         [SO3.hat(v), SO3.hat(omega)]])

    @staticmethod
    def compose(m, matexps):
        """
        SE(3) 군 곱셈으로 forward kinematics (body axis).
        :param m: 초기 변환행렬 (zero config)
        :param matexps: 관절별 변환행렬(exp 결과) 리스트
        :return: 말단 변환행렬 T_sb
        """
        T_sb = m
        for X in matexps:
            T_sb = T_sb @ X
        return T_sb

    @staticmethod
    def from_pose(pose):
        """
        pose(위치+quaternion) → SE(3).
        :param pose: [x, y, z, qx, qy, qz, qw]
        """
        x, y, z = pose[0], pose[1], pose[2]
        quat = pose[3:]
        T = np.eye(4)
        T[:3, :3] = SO3.from_quat(quat)
        T[:3, 3] = [float(x), float(y), float(z)]
        return T

    @staticmethod
    def orientation(T_sb):
        """
        SE(3) 의 회전부에서 자세를 추출.
        :param T_sb: 4x4 변환행렬
        :return: (quaternion[list], euler[list, degree])
        """
        R_mat = T_sb[:3, :3]
        return SO3.to_quat(R_mat).tolist(), SO3.to_euler(R_mat).tolist()

    @staticmethod
    def pose_euler_to_quat(pose, degree=True):
        """
        pose 의 Euler 자세를 quaternion 으로 변환 (위치는 유지).
        :param pose: [x, y, z, roll, pitch, yaw]
        :return: [x, y, z, qx, qy, qz, qw]
        """
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        if degree:
            roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat().tolist()
        return pose[:3] + quat


# ======================================================================
#  Kinematics — SO3/SE3 연산을 조합한 알고리즘 계층 (Jacobian, IK)
# ======================================================================
class Kinematics:
    """body-frame 기구학 알고리즘 (FK·Jacobian·DLS·IK·조작성).

    모델 데이터(M_home, Blist[, Slist])를 인자로 받아 로봇 무관·radian 으로 동작한다.
    `RobotModel` 이 자신의 데이터를 넘겨 이 메서드들을 호출한다(단일 구현).
    """

    SING_EPS = 0.04      # σ_min < ε → 특이점 근접(감쇠 시작)
    SING_LAM = 0.04      # damped least-squares 감쇠 상한

    @staticmethod
    def fk(M_home, Blist, theta):
        """말단 변환 T_sb (body form): M · ∏ exp([B_i]θ_i)."""
        T = np.array(M_home, dtype=float)
        for i in range(Blist.shape[1]):
            T = T @ SE3.exp6(Blist[:, i] * theta[i])
        return T

    @staticmethod
    def jacobian_body(Blist, theta):
        """Body Jacobian (6 × n)."""
        n = Blist.shape[1]
        Jb = np.zeros((6, n))
        Jb[:, n - 1] = Blist[:, n - 1]
        T = np.eye(4)
        for i in range(n - 2, -1, -1):
            T = T @ SE3.exp6(Blist[:, i + 1] * -theta[i + 1])
            Jb[:, i] = SE3.Adjoint(T) @ Blist[:, i]
        return Jb

    @staticmethod
    def dls_inv(Jb):
        """damped least-squares 역 (특이점 근처 큰 점프 방지). task 차원 6."""
        eps, lam = Kinematics.SING_EPS, Kinematics.SING_LAM
        s_min = np.linalg.svd(Jb, compute_uv=False)[-1]
        if s_min >= eps:
            return np.linalg.pinv(Jb)
        lam2 = lam ** 2 * (1.0 - (s_min / eps) ** 2)
        return Jb.T @ np.linalg.inv(Jb @ Jb.T + lam2 * np.eye(6))

    @staticmethod
    def manipulability(Blist, theta):
        """Yoshikawa 조작성. w=√det(JJᵀ)=∏σ, sigma_min=min σ. →0 이면 특이점."""
        Jb = Kinematics.jacobian_body(Blist, np.asarray(theta, float))
        sv = np.linalg.svd(Jb, compute_uv=False)
        return {"w": float(np.prod(sv)), "sigma_min": float(sv[-1])}

    @staticmethod
    def ik(M_home, Blist, T_target, theta0=None,
           eomg=1e-4, ev=1e-4, max_iter=100):
        """수치 IK (Newton-Raphson, body twist 오차, DLS 감쇠). → (θ, 수렴여부)."""
        n = Blist.shape[1]

        def wrap(t):
            return np.mod(t + np.pi, 2 * np.pi) - np.pi    # 회전관절 가정

        theta = (np.zeros(n) if theta0 is None
                 else np.array(theta0, dtype=float))
        for _ in range(max_iter):
            Tcur = Kinematics.fk(M_home, Blist, theta)
            T_err = np.linalg.inv(Tcur) @ T_target
            ang = np.arccos(np.clip((np.trace(T_err[:3, :3]) - 1) / 2, -1.0, 1.0))
            pos = np.linalg.norm(T_err[:3, 3])
            if ang < eomg and pos < ev:
                return wrap(theta), True
            Vb = SE3.log(T_err)[0]
            theta = theta + Kinematics.dls_inv(
                Kinematics.jacobian_body(Blist, theta)) @ Vb
        return wrap(theta), False

    @staticmethod
    def fk_skeleton(Slist, M_home, theta):
        """관절 screw 축점 기준 스켈레톤 (시각화용). (n+2, 3)."""
        n = Slist.shape[1]
        pts = [np.zeros(3)]
        prod = np.eye(4)
        for i in range(n):
            w, v = Slist[:3, i], Slist[3:, i]
            q = np.cross(w, v)
            pts.append((prod @ np.append(q, 1.0))[:3])
            prod = prod @ SE3.exp6(Slist[:, i] * theta[i])
        pts.append((prod @ M_home)[:3, 3])
        return np.array(pts)


# ======================================================================
#  Dynamics — 동역학 (역동역학 RNE, 질량/코리올리/중력, 순동역학)
# ======================================================================
class Dynamics:
    """
    Recursive Newton-Euler 기반 동역학. Modern Robotics 8장 컨벤션

    모델 데이터(로봇마다 필요):
        Mlist : 링크 좌표계 home 상대변환 [M_{0,1}, M_{1,2}, ..., M_{n,ee}] (n+1 개)
        Glist : 링크별 6x6 공간관성행렬 G_i = diag(I_i, m_i·I₃) (n 개)
        Slist : space frame 기준 screw 축, 6xn 행렬 (열 = 관절축)
        g     : 중력가속도 벡터, 예) [0, 0, -9.81]
        Ftip  : 말단 외력 wrench (6D), 보통 0

    단위 주의: 길이는 m, 질량은 kg, 시간은 s (SI) 로 통일해야
    토크가 N·m 로 일관되게 나온다. (기존 기구학 코드는 mm 사용 → 변환 필요)
    """

    @staticmethod
    def inverse_dynamics(thetalist, dthetalist, ddthetalist,
                         g, Ftip, Mlist, Glist, Slist):
        """
        역동역학 (RNE). (θ, θ̇, θ̈) → 관절 토크 τ.
        forward pass(속도·가속도 전파) + backward pass(렌치 전파) 2-pass.
        """
        n = len(thetalist)
        Slist = np.asarray(Slist, dtype=float)

        Mi = np.eye(4)
        Ai = np.zeros((6, n))
        AdTi = [None] * (n + 1)
        Vi = np.zeros((6, n + 1))
        Vdi = np.zeros((6, n + 1))
        # base 가속도를 -g 로 두는 "중력 트릭": 중력이 자동으로 토크에 반영됨
        Vdi[:, 0] = np.r_[[0, 0, 0], -np.asarray(g, dtype=float)]
        AdTi[n] = SE3.Adjoint(np.linalg.inv(Mlist[n]))
        Fi = np.asarray(Ftip, dtype=float).copy()
        taulist = np.zeros(n)

        # ① Forward pass: base → tip (속도 Vi, 가속도 Vdi 전파)
        for i in range(n):
            Mi = Mi @ Mlist[i]
            Ai[:, i] = SE3.Adjoint(np.linalg.inv(Mi)) @ Slist[:, i]
            AdTi[i] = SE3.Adjoint(SE3.exp6(Ai[:, i] * -thetalist[i])
                                  @ np.linalg.inv(Mlist[i]))
            Vi[:, i + 1] = AdTi[i] @ Vi[:, i] + Ai[:, i] * dthetalist[i]
            Vdi[:, i + 1] = (AdTi[i] @ Vdi[:, i]
                             + Ai[:, i] * ddthetalist[i]
                             + SE3.ad(Vi[:, i + 1]) @ Ai[:, i] * dthetalist[i])

        # ② Backward pass: tip → base (렌치 Fi 전파 → 토크 τ 추출)
        for i in range(n - 1, -1, -1):
            Fi = (AdTi[i + 1].T @ Fi
                  + Glist[i] @ Vdi[:, i + 1]
                  - SE3.ad(Vi[:, i + 1]).T @ (Glist[i] @ Vi[:, i + 1]))
            taulist[i] = Fi @ Ai[:, i]

        return taulist

    @staticmethod
    def mass_matrix(thetalist, Mlist, Glist, Slist):
        """질량행렬 M(θ). RNE 를 단위가속도로 n 번 호출해 열별로 추출."""
        n = len(thetalist)
        M = np.zeros((n, n))
        for i in range(n):
            ddtheta = np.zeros(n)
            ddtheta[i] = 1
            M[:, i] = Dynamics.inverse_dynamics(
                thetalist, np.zeros(n), ddtheta,
                [0, 0, 0], np.zeros(6), Mlist, Glist, Slist)
        return M

    @staticmethod
    def gravity_forces(thetalist, g, Mlist, Glist, Slist):
        """중력항 g(θ). RNE(θ, 0, 0, 중력 ON)."""
        n = len(thetalist)
        return Dynamics.inverse_dynamics(
            thetalist, np.zeros(n), np.zeros(n),
            g, np.zeros(6), Mlist, Glist, Slist)

    @staticmethod
    def coriolis_forces(thetalist, dthetalist, Mlist, Glist, Slist):
        """코리올리/원심 벡터 c(θ,θ̇)θ̇. RNE(θ, θ̇, 0, 중력 OFF)."""
        n = len(thetalist)
        return Dynamics.inverse_dynamics(
            thetalist, dthetalist, np.zeros(n),
            [0, 0, 0], np.zeros(6), Mlist, Glist, Slist)

    @staticmethod
    def end_effector_forces(thetalist, Ftip, Mlist, Glist, Slist):
        """말단 외력이 만드는 관절 토크 Jᵀ(θ)·Ftip."""
        n = len(thetalist)
        return Dynamics.inverse_dynamics(
            thetalist, np.zeros(n), np.zeros(n),
            [0, 0, 0], Ftip, Mlist, Glist, Slist)

    @staticmethod
    def forward_dynamics(thetalist, dthetalist, taulist,
                         g, Ftip, Mlist, Glist, Slist):
        """
        순동역학. (θ, θ̇, τ) → 관절 가속도 θ̈.
        θ̈ = M(θ)⁻¹ (τ − c(θ,θ̇) − g(θ) − Jᵀ·Ftip).  시뮬레이터(plant) 용.
        """
        M = Dynamics.mass_matrix(thetalist, Mlist, Glist, Slist)
        c = Dynamics.coriolis_forces(thetalist, dthetalist, Mlist, Glist, Slist)
        grav = Dynamics.gravity_forces(thetalist, g, Mlist, Glist, Slist)
        tip = Dynamics.end_effector_forces(thetalist, Ftip, Mlist, Glist, Slist)
        return np.linalg.solve(M, np.asarray(taulist, dtype=float) - c - grav - tip)


# ----------------------------------------------------------------------
#  유틸리티 (시간 스케일링)
# ----------------------------------------------------------------------
def quintic_time_scaling(t, T):
    """
    5차 다항식 기반 시간 스케일링 s(t). (t=0, t=T 에서 속도·가속도 0)
    :return: (s, s_dot, s_ddot)
    """
    quintic = np.array([[T ** 3, T ** 4, T ** 5],
                        [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                        [6 * T, 12 * T ** 2, 20 * T ** 3]])
    s_T = np.array([1, 0, 0])
    cofficient = np.linalg.inv(quintic) @ s_T.reshape(3, 1)
    a3, a4, a5 = cofficient.flatten()

    s = a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
    s_dot = 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
    s_ddot = 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
    return s, s_dot, s_ddot

