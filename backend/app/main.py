"""Robot Web Simulator — FastAPI 앱.

실행:  cd backend && uv run main.py
문서:  http://localhost:8500/docs
"""
import os
from pathlib import Path

from dotenv import load_dotenv
from fastapi import Depends, FastAPI, HTTPException, Security
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.security import APIKeyHeader
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from app import sim
from app.db import init_db
from app.meshes_setup import MESHES_DIR, prepare_meshes
from app.routers import (
    interactive_ws,
    kinematics,
    mesh,
    programs,
    robots,
    simulate,
)

load_dotenv(Path(__file__).resolve().parents[1] / ".env")  # backend/.env 로드

app = FastAPI(title="Robot Web Simulator", version="0.1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

# 시뮬 인증: SIM_KEY 설정 시 /api 가 헤더 X-Sim-Key 일치 요구(슬라이드 무관).
# 빈 값이면 비활성 — 로컬 개발은 로그인 없이 그대로.
SIM_KEY = os.environ.get("SIM_KEY", "")
_api_key = APIKeyHeader(name="X-Sim-Key", auto_error=False)


def require_key(key: str | None = Security(_api_key)):
    if SIM_KEY and key != SIM_KEY:
        raise HTTPException(status_code=401, detail="시뮬 인증이 필요합니다")


init_db()                                   # 테이블 생성 (멱등)
prepare_meshes()                            # URDF + 메시를 app/meshes/ 로 복사 (멱등)
sim.warmup()                                # numba RNE JIT 사전 컴파일 (첫 요청 지연 방지)

# 공개(읽기/렌더): robots·kinematics. 슬라이드 3D 로봇도 이걸 씀.
for module in (robots, kinematics):
    app.include_router(module.router)
# 실시간 grab & push WebSocket (/ws/interactive). 스트리밍이라 별도.
app.include_router(interactive_ws.router)
# 보호: 시뮬 실행·DB 쓰기·메시 업로드만 인증(SIM_KEY 미설정이면 no-op).
for module in (programs, simulate, mesh):
    app.include_router(module.router, dependencies=[Depends(require_key)])


class LoginReq(BaseModel):
    key: str


@app.post("/api/login")  # 로그인 폼 키 검증(맞으면 프론트가 저장)
def login(body: LoginReq):
    if SIM_KEY and body.key != SIM_KEY:
        raise HTTPException(status_code=401, detail="키가 틀렸습니다")
    return {"ok": True}


@app.get("/api/auth")  # 현재 키로 시뮬 권한 확인(200/401)
def auth_ok(_=Depends(require_key)):
    return {"ok": True}


# URDF + 메시 정적 서빙 (urdf-loader 가 /meshes/... 로 로드)
app.mount("/meshes", StaticFiles(directory=MESHES_DIR), name="meshes")

# 빌드된 프론트(frontend/dist)를 같은 출처로 서빙 → 단일 포트(프록시 불필요).
# dist 없으면 JSON 루트만(개발 중). 있으면 SPA: 실제 파일이면 그 파일, 없으면
# index.html 폴백(클라이언트 라우팅). /api·/docs·/meshes 는 위에서 먼저 매칭됨.
DIST = Path(__file__).resolve().parents[2] / "frontend" / "dist"

if DIST.is_dir():
    @app.get("/{full_path:path}")
    def spa(full_path: str):
        if full_path.startswith(("api/", "meshes/")):
            raise HTTPException(status_code=404)
        f = DIST / full_path
        if full_path and f.is_file():
            return FileResponse(f)
        return FileResponse(DIST / "index.html")
else:
    @app.get("/")
    def root():
        return {"name": "Robot Web Simulator", "docs": "/docs"}
