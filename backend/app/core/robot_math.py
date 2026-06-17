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
from scipy.spatial.transform import Rotation as R, Slerp


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
    """Jacobian, 역기구학 등 Lie Group/Algebra primitive 를 조합한 기구학 알고리즘."""

    @staticmethod
    def space_jacobian(matexps, screws):
        """
        Space Jacobian (6xN). 각 열은 space twist(se(3)).
        :param matexps: 관절별 space 변환행렬 리스트
        :param screws : space twist(6D) 리스트
        """
        n = len(matexps)
        j_s = np.zeros((6, n))
        j_s[:, 0] = np.array(screws[0]).reshape(6,)
        mul = np.eye(4)
        for i in range(1, n):
            mul = mul @ matexps[i - 1]
            j_s[:, i] = SE3.Adjoint(mul) @ np.array(screws[i]).reshape(6,)
        return j_s

    @staticmethod
    def body_jacobian(m, matexps_b, matexps_s, screws):
        """
        Body Jacobian (6xN). 각 열은 body twist(se(3)).
        :param m: 초기 변환행렬
        :param matexps_b: body 변환행렬 리스트
        :param matexps_s: space 변환행렬 리스트
        :param screws   : space twist(6D) 리스트
        """
        T_sb = SE3.compose(m, matexps_b)
        T_bs = np.linalg.inv(T_sb)
        return SE3.Adjoint(T_bs) @ Kinematics.space_jacobian(matexps_s, screws)

    @staticmethod
    def damped_pinv(J, damping=0.005):
        """
        SVD 기반 damped pseudoinverse. 특이점 근처 발산 억제 (λ = damping).
        :param J: Jacobian 행렬
        """
        U, S, Vt = np.linalg.svd(J)
        S_damped = np.diag([s / (s ** 2 + damping ** 2) for s in S])
        return Vt.T @ S_damped @ U.T

    @staticmethod
    def IK(robot, init, desired):
        """
        Numerical IK (Newton-Raphson, body frame). Gimbal lock 회피를 위해
        전체 SE(3) 행렬로 오차를 비교한다.
        :param robot: 로봇 클래스 (joints, B_tw, S_tw, zero)
        :param init: 초기 관절각 (degree)
        :param desired: 목표 pose [x, y, z, quaternion] (size 7)
        :return: (해 관절각(degree), 반복 횟수)
        """
        M = robot.zero
        B = robot.B_tw
        S = robot.S_tw
        L = len(robot.joints)

        T_d = SE3.from_pose(desired)
        threshold = 1e-4
        count = 0

        while True:
            count += 1

            # Forward Kinematics
            matexps_b = [SE3.exp(init[i], B[i], joint=robot.joints[i].type) for i in range(L)]
            matexps_s = [SE3.exp(init[i], S[i], joint=robot.joints[i].type) for i in range(L)]
            T_sb = SE3.compose(M, matexps_b)

            # Error Transformation
            T_bd = np.linalg.inv(T_sb) @ T_d
            V_bd, _, _ = SE3.log(T_bd)

            # Jacobian
            J_b = Kinematics.body_jacobian(M, matexps_b, matexps_s, S)

            # 특이점 감지 및 감쇠 적용
            sigma_min = np.min(np.linalg.svd(J_b, compute_uv=False))
            if sigma_min < 1e-3:
                J_pseudo = Kinematics.damped_pinv(J_b, damping=0.05)
            else:
                J_pseudo = Kinematics.damped_pinv(J_b)

            # Update joint angle (in rad)
            theta = deg2rad(init, robot.joints)
            thetak = theta.reshape(L, 1) + J_pseudo @ V_bd.reshape(L, 1)
            thetak = rad2deg(thetak, robot.joints)
            init = theta_normalize(thetak, robot.joints)

            # Check error norm (position and rotation)
            pos_err = np.linalg.norm(T_bd[:3, 3])
            rot_err = np.linalg.norm(V_bd)

            if pos_err < threshold and rot_err < np.deg2rad(0.1):
                break
            if count >= 50:
                break

        return init, count

    @staticmethod
    def IK_solutions(robot, desired, seeds=None, atol_deg=1.0):
        """
        여러 초기값(seed)에서 NR IK 를 돌려 서로 다른 해를 수집한다.
        6R 로봇은 한 말단 자세에 대해 IK 해가 여러 개(UR5 최대 8개) 존재 →
        seed 를 바꿔 다른 가지(elbow up/down, wrist flip 등)를 찾는다.
        :param robot: 로봇 클래스
        :param desired: 목표 pose [x, y, z, quaternion]
        :param seeds: 초기각(degree) 후보 리스트. None 이면 구조적 기본 seed.
        :param atol_deg: 같은 해로 간주할 각도 허용오차(중복 제거용)
        :return: 서로 다른 해(degree, np.array)들의 리스트
        """
        L = len(robot.joints)
        if seeds is None:
            seeds = [[0] * L]
            for s2 in (-60, 60):              # 어깨
                for s3 in (-90, 90):          # 팔꿈치 (elbow up/down)
                    for s5 in (-90, 90):      # 손목 (wrist flip)
                        seed = [0] * L
                        seed[1], seed[2], seed[4] = s2, s3, s5
                        seeds.append(seed)

        sols = []
        for seed in seeds:
            sol, count = Kinematics.IK(robot, list(seed), desired)
            if count >= 50:                   # 미수렴 폐기
                continue
            sol = np.asarray(sol, dtype=float)
            if not any(np.allclose(sol, s, atol=atol_deg) for s in sols):
                sols.append(sol)
        return sols


# ======================================================================
#  Dynamics — 동역학 (역동역학 RNE, 질량/코리올리/중력, 순동역학)
# ======================================================================
class Dynamics:
    """
    Recursive Newton-Euler 기반 동역학. Modern Robotics 8장 컨벤션.

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
#  유틸리티 (관절각 단위 변환 / 정규화 / 시간 스케일링)
# ----------------------------------------------------------------------
def deg2rad(value, joint):
    """회전 관절(R)만 degree → radian 으로 변환."""
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i].type == 'R':
            theta[i] = np.deg2rad(theta[i])
    return theta


def rad2deg(value, joint):
    """회전 관절(R)만 radian → degree 로 변환."""
    theta = np.array(value)
    for i in range(len(joint)):
        if joint[i].type == 'R':
            theta[i] = np.rad2deg(value[i])
    return theta


def theta_normalize(value, joint):
    """회전 관절(R) 각을 -180 ~ 180 범위로 정규화."""
    theta = value.flatten().tolist()
    for i in range(len(value)):
        if joint[i].type == 'R':
            if value[i] % 360 > 180:
                theta[i] = theta[i] % 360 - 360
            else:
                theta[i] = theta[i] % 360
    return theta


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


# ----------------------------------------------------------------------
#  궤적 생성 (task space / joint space)
# ----------------------------------------------------------------------
def interpolate_SE3_quat(T0, Td, T, N):
    """
    두 SE(3) 자세 사이를 SLERP(회전) + quintic(위치)으로 보간.
    :param T0, Td: 시작/끝 변환행렬
    :param T: 총 운동 시간
    :param N: 보간 점 개수
    :return: 4x4 변환행렬 리스트
    """
    pos0 = T0[:3, 3]
    posd = Td[:3, 3]

    key_times = [0, 1]
    key_rots = R.from_matrix([T0[:3, :3], Td[:3, :3]])
    slerp = Slerp(key_times, key_rots)

    trajectory = []
    for i in range(N):
        t = i / (N - 1) * T
        s, _, _ = quintic_time_scaling(t, T)
        interp_rot = slerp([s])[0]
        interp_pos = (1 - s) * pos0 + s * posd

        T_interp = np.eye(4)
        T_interp[:3, :3] = interp_rot.as_matrix()
        T_interp[:3, 3] = interp_pos
        trajectory.append(T_interp)

    return trajectory


def task_trajectory(robot, start, end, times=1.0, samples=100):
    """
    Task space(SE(3)) 에서 두 pose 사이의 매끄러운 궤적 생성.
    :param start, end: pose [x, y, z, quaternion]
    :return: 4x4 변환행렬 리스트
    """
    theta_start = np.array(start)
    theta_end = np.array(end)

    Td = SE3.from_pose(theta_end)
    T0 = SE3.from_pose(theta_start)

    return interpolate_SE3_quat(T0, Td, times, samples)


def joint_trajectory(start, end, times=1.0, samples=100):
    """
    Joint space 에서 두 관절 구성 사이의 매끄러운 궤적 생성 (quintic).
    :param start, end: 관절각 (degree)
    :return: (d_theta, trajectory)
    """
    theta_start = np.array(start)
    theta_end = np.array(end)

    d_theta = theta_end - theta_start
    for i in range(len(d_theta)):
        if d_theta[i] > 180:
            d_theta[i] -= 360
        elif d_theta[i] < -180:
            d_theta[i] += 360

    T = times
    N = samples

    trajectory = []
    velocity = []
    acceleration = []

    for i in range(N):
        t = i / N * T
        s, s_dot, s_ddot = quintic_time_scaling(t, T)
        trajectory.append((theta_start + s * d_theta).tolist())
        velocity.append((s_dot * d_theta).tolist())
        acceleration.append((s_ddot * d_theta).tolist())

    return d_theta, trajectory


# ----------------------------------------------------------------------
#  하위 호환: 기존 모듈 레벨 IK(...) 호출 유지 → Kinematics.IK 로 위임
# ----------------------------------------------------------------------
def IK(robot, init, desired):
    return Kinematics.IK(robot, init, desired)

