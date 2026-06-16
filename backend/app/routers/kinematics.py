"""기구학(IK) 라우터."""
from fastapi import APIRouter, HTTPException
from app.schemas import IKRequest, IKResponse
from app.controllers import kinematics as ctrl

router = APIRouter(prefix="/api", tags=["kinematics"])


@router.post("/ik", response_model=IKResponse)
def ik(req: IKRequest):
    if len(req.pose) != 7:
        raise HTTPException(400, "pose 는 [x,y,z,qx,qy,qz,qw] 7개여야 합니다")
    return ctrl.solve_ik(req)
