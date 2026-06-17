"""프로그램(포즈 저장) 비즈니스 로직 — DB CRUD."""
from sqlmodel import Session, select
from app.models import Pose
from app.schemas import PoseIn


def list_programs(session: Session):
    counts: dict[str, int] = {}
    for pid in session.exec(select(Pose.program_id)).all():
        counts[pid] = counts.get(pid, 0) + 1
    return [{"program_id": k, "n_poses": v} for k, v in sorted(counts.items())]


def get_program(session: Session, program_id: str):
    return session.exec(
        select(Pose).where(Pose.program_id == program_id).order_by(Pose.id)).all()


def add_pose(session: Session, program_id: str, body: PoseIn) -> Pose:
    pose = Pose(program_id=program_id, **body.model_dump())
    session.add(pose)
    session.commit()
    session.refresh(pose)
    return pose


def reset_program(session: Session, program_id: str):
    for p in session.exec(select(Pose).where(Pose.program_id == program_id)).all():
        session.delete(p)
    session.commit()


def delete_pose(session: Session, pose_id: int):
    p = session.get(Pose, pose_id)
    if p:
        session.delete(p)
        session.commit()
