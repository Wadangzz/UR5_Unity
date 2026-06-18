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
        # 현재 자세에서 출발(실제 로봇처럼 "지금 위치에서 프로그램 실행").
        # start 미지정 시 ready 로 폴백. 티치 관절각(theta)이 있으면 그대로, 없으면 IK.
        waypoints = [list(req.start) if req.start else ur5.READY.tolist()]
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
    result = sim.run_simulation(
        waypoints, controller=req.controller, gains=req.gains,
        gravity_comp=req.gravity_comp, t_seg=req.t_seg, hold=req.hold,
        plant=plant, ctrl=ctrl, disturbance=req.disturbance,
        traj_mode=req.traj_mode)

    # 재생 중 직교좌표·manipulability 를 라이브로 보이게: θ(t) 에서 EE pose·σ_min
    # 시계열을 한 번에 계산해 응답에 실어 보낸다 (프레임당 REST 왕복 없이 인덱싱).
    ee_pose, sigma_min = [], []
    for th in result["theta"]:
        T = ur5.fk(np.array(th))
        quat, _ = SE3.orientation(T)                 # [x,y,z,w]
        ee_pose.append(T[:3, 3].tolist() + quat)
        sigma_min.append(ur5.manipulability(th)["sigma_min"])
    result["ee_pose"] = ee_pose
    result["sigma_min"] = sigma_min
    return result
