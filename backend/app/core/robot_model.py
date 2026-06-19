"""RobotModel — 로봇 한 대의 기구학·동역학 파라미터 + 메서드 (로봇 무관).

ur5_model 의 하드코딩 함수(fk/jacobian_body/ik/manipulability/dls_inv)를 모델 데이터에
묶어 일반화한 것. (Mlist, Glist, Slist) 는 robot_math.Dynamics / dynamics_fast 가 그대로
받는 형식이라 동역학 엔진은 수정 없이 n-무관하게 동작한다.

생성:
    RobotModel(Mlist, Glist, Slist, M_home, ...)          # 파라미터 직접
    RobotModel.from_urdf(urdf_path, ee_link=...)          # URDF 추출(urdf_loader)
"""
import numpy as np

from app.core.robot_math import SE3

_SING_EPS = 0.04      # σ_min < ε → 특이점 근접(감쇠 시작)
_SING_LAM = 0.04      # damped least-squares 감쇠 상한


class RobotModel:
    def __init__(self, Mlist, Glist, Slist, M_home, *, name="robot",
                 gravity=(0.0, 0.0, -9.81), joint_names=None,
                 joint_limits=None, ready=None):
        self.name = name
        self.Mlist = [np.asarray(M, float) for M in Mlist]
        self.Glist = [np.asarray(G, float) for G in Glist]
        self.Slist = np.asarray(Slist, float)
        self.M_home = np.asarray(M_home, float)
        self.n = self.Slist.shape[1]
        self.Blist = SE3.Adjoint(np.linalg.inv(self.M_home)) @ self.Slist
        self.gravity = np.asarray(gravity, float)
        self.joint_names = joint_names or [f"joint_{i+1}" for i in range(self.n)]
        self.joint_limits = joint_limits
        self.ready = (np.zeros(self.n) if ready is None
                      else np.asarray(ready, float))

    @property
    def dyn_args(self):
        """동역학 엔진 인자 (Mlist, Glist, Slist)."""
        return (self.Mlist, self.Glist, self.Slist)

    # --- 기구학 (MR body frame, radian) ---
    def fk(self, theta):
        """말단 변환행렬 T_sb (body form): M · ∏ exp([B_i]θ_i)."""
        T = self.M_home.copy()
        for i in range(self.n):
            T = T @ SE3.exp6(self.Blist[:, i] * theta[i])
        return T

    def jacobian_body(self, theta):
        """Body Jacobian (6 × n)."""
        Jb = np.zeros((6, self.n))
        Jb[:, self.n - 1] = self.Blist[:, self.n - 1]
        T = np.eye(4)
        for i in range(self.n - 2, -1, -1):
            T = T @ SE3.exp6(self.Blist[:, i + 1] * -theta[i + 1])
            Jb[:, i] = SE3.Adjoint(T) @ self.Blist[:, i]
        return Jb

    def dls_inv(self, Jb):
        """damped least-squares 역 (특이점 근처 큰 점프 방지). task 차원 6."""
        s_min = np.linalg.svd(Jb, compute_uv=False)[-1]
        if s_min >= _SING_EPS:
            return np.linalg.pinv(Jb)
        lam2 = _SING_LAM ** 2 * (1.0 - (s_min / _SING_EPS) ** 2)
        return Jb.T @ np.linalg.inv(Jb @ Jb.T + lam2 * np.eye(6))

    def manipulability(self, theta):
        """Yoshikawa 조작성. w=√det(JJᵀ)=∏σ_i, sigma_min=min σ. →0 이면 특이점."""
        sv = np.linalg.svd(self.jacobian_body(np.asarray(theta, float)),
                           compute_uv=False)
        return {"w": float(np.prod(sv)), "sigma_min": float(sv[-1])}

    def ik(self, T_target, theta0=None, eomg=1e-4, ev=1e-4, max_iter=100):
        """수치 IK (Newton-Raphson, body twist 오차, DLS 감쇠). → (θ, 수렴여부)."""
        def wrap(t):
            return np.mod(t + np.pi, 2 * np.pi) - np.pi    # 회전관절 가정

        theta = (np.zeros(self.n) if theta0 is None
                 else np.array(theta0, dtype=float))
        for _ in range(max_iter):
            T_err = np.linalg.inv(self.fk(theta)) @ T_target
            ang = np.arccos(np.clip((np.trace(T_err[:3, :3]) - 1) / 2, -1.0, 1.0))
            pos = np.linalg.norm(T_err[:3, 3])
            if ang < eomg and pos < ev:
                return wrap(theta), True
            Vb = SE3.log(T_err)[0]
            theta = theta + self.dls_inv(self.jacobian_body(theta)) @ Vb
        return wrap(theta), False

    def fk_skeleton(self, theta):
        """관절 screw 축점 기준 스켈레톤 (시각화용). (n+2, 3)."""
        pts = [np.zeros(3)]
        prod = np.eye(4)
        for i in range(self.n):
            w, v = self.Slist[:3, i], self.Slist[3:, i]
            q = np.cross(w, v)
            pts.append((prod @ np.append(q, 1.0))[:3])
            prod = prod @ SE3.exp6(self.Slist[:, i] * theta[i])
        pts.append((prod @ self.M_home)[:3, 3])
        return np.array(pts)

    @classmethod
    def from_urdf(cls, urdf_path, ee_link=None, **kw):
        from app.core.urdf_loader import load_mr_model
        m = load_mr_model(urdf_path, ee_link)
        return cls(m["Mlist"], m["Glist"], m["Slist"], m["M_home"],
                   name=m["name"], joint_names=m["joint_names"],
                   joint_limits=m["joint_limits"], **kw)
