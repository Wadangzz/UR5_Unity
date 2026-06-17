"""UR5 Web Simulator — FastAPI 앱.

실행:  cd backend && uv run uvicorn app.main:app --reload
문서:  http://localhost:8000/docs
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from app.db import init_db
from app.routers import robot, kinematics, programs, simulate
from app.meshes_setup import prepare_meshes, MESHES_DIR

app = FastAPI(title="UR5 Web Simulator", version="0.1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

init_db()                                   # 테이블 생성 (멱등)
prepare_meshes()                            # URDF + 메시를 app/meshes/ 로 복사 (멱등)

for module in (robot, kinematics, programs, simulate):
    app.include_router(module.router)

# URDF + 메시 정적 서빙 (urdf-loader 가 /meshes/... 로 로드)
app.mount("/meshes", StaticFiles(directory=MESHES_DIR), name="meshes")


@app.get("/")
def root():
    return {"name": "UR5 Web Simulator", "docs": "/docs"}
