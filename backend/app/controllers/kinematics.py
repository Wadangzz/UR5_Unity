"""기구학 비즈니스 로직 (IK)."""
import numpy as np
import ur5_model as ur5
from robot_math import SE3
from app.schemas import IKRequest, IKResponse


def solve_ik(req: IKRequest) -> IKResponse:
    T = SE3.from_pose(req.pose)
    seed = np.array(req.seed) if req.seed else np.zeros(ur5.N)
    theta, ok = ur5.ik(T, seed)
    return IKResponse(theta=theta.tolist(), converged=bool(ok),
                      tcp=ur5.fk(theta)[:3, 3].tolist())
