"""시뮬레이션 실행 라우터."""
from fastapi import APIRouter, Depends
from sqlmodel import Session
from app.db import get_session
from app.schemas import RunRequest, RunResponse
from app.controllers import simulate as ctrl

router = APIRouter(prefix="/api", tags=["simulate"])


@router.post("/run", response_model=RunResponse)
def run(req: RunRequest, session: Session = Depends(get_session)):
    return ctrl.run(req, session)
