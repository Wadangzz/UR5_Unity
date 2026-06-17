"""기구학 비즈니스 로직 (IK / FK)."""
import numpy as np
from app.core import ur5_model as ur5
from app.core.robot_math import SE3
from app.schemas import IKRequest, IKResponse, FKRequest, FKResponse


def solve_ik(req: IKRequest) -> IKResponse:
    T = SE3.from_pose(req.pose)
    seed = np.array(req.seed) if req.seed else np.zeros(ur5.N)
    theta, ok = ur5.ik(T, seed)
    return IKResponse(theta=theta.tolist(), converged=bool(ok),
                      tcp=ur5.fk(theta)[:3, 3].tolist())


def solve_fk(req: FKRequest) -> FKResponse:
    T = ur5.fk(np.array(req.theta))
    pos = T[:3, 3].tolist()
    quat, _ = SE3.orientation(T)            # [x,y,z,w]
    m = ur5.manipulability(req.theta)       # 특이점 근접도
    return FKResponse(pose=pos + quat, tcp=pos,
                      w=m["w"], sigma_min=m["sigma_min"])
