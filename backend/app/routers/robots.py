"""멀티로봇 라우터 — 사용 가능 로봇 목록 + 로봇별 상세(관절·한계·자세)."""
import numpy as np
from fastapi import APIRouter, HTTPException

from app.core import robot_registry as reg

router = APIRouter(prefix="/api", tags=["robots"])


@router.get("/robots")
def robots():
    """사용 가능 로봇 목록 (로드 없이 메타만)."""
    return reg.list_robots()


@router.get("/robots/{robot_id}")
def robot_detail(robot_id: str):
    """로봇 상세 — 첫 호출 시 URDF 로드(캐시). 관절·한계·home/ready TCP."""
    try:
        rm = reg.get_robot(robot_id)
    except KeyError:
        raise HTTPException(404, f"등록되지 않은 로봇: {robot_id}")
    return {
        "id": robot_id,
        "name": rm.name,
        "n": rm.n,
        "joint_names": rm.joint_names,
        "joint_limits": rm.joint_limits,
        "ready": rm.ready.tolist(),
        "home_tcp": rm.fk(np.zeros(rm.n))[:3, 3].tolist(),
        "ready_tcp": rm.fk(rm.ready)[:3, 3].tolist(),
    }
