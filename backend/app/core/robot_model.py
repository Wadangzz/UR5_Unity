"""RobotModel — 로봇 한 대의 기구학·동역학 파라미터 + 메서드 (로봇 무관).

ur5_model 의 하드코딩 함수(fk/jacobian_body/ik/manipulability/dls_inv)를 모델 데이터에
묶어 일반화한 것. (Mlist, Glist, Slist) 는 robot_math.Dynamics / dynamics_fast 가 그대로
받는 형식이라 동역학 엔진은 수정 없이 n-무관하게 동작한다.

생성:
    RobotModel(Mlist, Glist, Slist, M_home, ...)          # 파라미터 직접
    RobotModel.from_urdf(urdf_path, ee_link=...)          # URDF 추출(urdf_loader)
"""
import numpy as np

from app.core.robot_math import SE3, Kinematics


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

    # --- 기구학: robot_math.Kinematics 에 위임 (RobotModel 은 데이터만 보유) ---
    def fk(self, theta):
        return Kinematics.fk(self.M_home, self.Blist, theta)

    def jacobian_body(self, theta):
        return Kinematics.jacobian_body(self.Blist, theta)

    def dls_inv(self, Jb):
        return Kinematics.dls_inv(Jb)

    def manipulability(self, theta):
        return Kinematics.manipulability(self.Blist, theta)

    def ik(self, T_target, theta0=None, eomg=1e-4, ev=1e-4, max_iter=100):
        return Kinematics.ik(self.M_home, self.Blist, T_target,
                             theta0, eomg, ev, max_iter)

    @classmethod
    def from_urdf(cls, urdf_path, ee_link=None, **kw):
        from app.core.urdf_loader import load_mr_model
        m = load_mr_model(urdf_path, ee_link)
        opts = dict(name=m["name"], joint_names=m["joint_names"],
                    joint_limits=m["joint_limits"])
        opts.update(kw)                          # 호출자 인자(name/ready/gravity) 우선
        return cls(m["Mlist"], m["Glist"], m["Slist"], m["M_home"], **opts)
