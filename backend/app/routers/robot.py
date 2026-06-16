"""로봇 정보 / 제어기 목록 라우터."""
from fastapi import APIRouter
import ur5_model as ur5
from app import sim
from app.schemas import ControllerSpec

router = APIRouter(prefix="/api", tags=["robot"])

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow",
               "wrist_1", "wrist_2", "wrist_3"]


@router.get("/robot")
def get_robot():
    return {"name": "UR5", "n": ur5.N, "joint_names": JOINT_NAMES,
            "home": [0.0] * ur5.N, "home_tcp": ur5.M_HOME[:3, 3].tolist()}


@router.get("/controllers", response_model=list[ControllerSpec])
def get_controllers():
    return sim.controller_specs()
