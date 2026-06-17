"""프로그램(포즈 저장) 라우터."""
from fastapi import APIRouter, Depends
from sqlmodel import Session
from app.db import get_session
from app.models import Pose
from app.schemas import PoseIn, ProgramSummary
from app.controllers import programs as ctrl

router = APIRouter(prefix="/api/programs", tags=["programs"])


@router.get("", response_model=list[ProgramSummary])
def list_programs(session: Session = Depends(get_session)):
    return ctrl.list_programs(session)


@router.get("/{program_id}", response_model=list[Pose])
def get_program(program_id: str, session: Session = Depends(get_session)):
    return ctrl.get_program(session, program_id)


@router.post("/{program_id}/poses", response_model=Pose)
def add_pose(program_id: str, body: PoseIn, session: Session = Depends(get_session)):
    return ctrl.add_pose(session, program_id, body)


@router.delete("/{program_id}/poses/{pose_id}")
def delete_pose(program_id: str, pose_id: int, session: Session = Depends(get_session)):
    ctrl.delete_pose(session, pose_id)
    return {"ok": True, "pose_id": pose_id}


@router.delete("/{program_id}")
def reset_program(program_id: str, session: Session = Depends(get_session)):
    ctrl.reset_program(session, program_id)
    return {"ok": True, "program_id": program_id}
