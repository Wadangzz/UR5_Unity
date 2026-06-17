"""시뮬레이션 실행 비즈니스 로직 — 프로그램 해석(IK) + 엔진 호출."""
import numpy as np
from fastapi import HTTPException
from sqlmodel import Session, select
from app.core import ur5_model as ur5
from app.core.robot_math import SE3
from app import sim, realism
from app.models import Pose
from app.schemas import RunRequest


def run(req: RunRequest, session: Session):
    if req.waypoints is not None:
        waypoints = req.waypoints
    elif req.program_id is not None:
        poses = session.exec(
            select(Pose).where(Pose.program_id == req.program_id).order_by(Pose.id)).all()
        if not poses:
            raise HTTPException(404, f"프로그램 {req.program_id} 에 저장된 포즈가 없습니다")
        # home 에서 출발. 티치된 관절각(theta)이 있으면 그대로(정확 재현),
        # 없으면 Cartesian 을 IK(이식).
        waypoints = [[0.0] * ur5.N]
        for p in poses:
            if p.theta:
                waypoints.append(list(p.theta))
            else:
                T = SE3.from_pose([p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw])
                theta, _ = ur5.ik(T, np.array(waypoints[-1]))
                waypoints.append(theta.tolist())
    else:
        raise HTTPException(400, "program_id 또는 waypoints 중 하나는 필요합니다")

    plant, ctrl = realism.build_models(req.payload, req.model_scale)
    return sim.run_simulation(
        waypoints, controller=req.controller, gains=req.gains,
        gravity_comp=req.gravity_comp, t_seg=req.t_seg, hold=req.hold,
        plant=plant, ctrl=ctrl)
