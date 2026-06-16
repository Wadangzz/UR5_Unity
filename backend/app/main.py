"""UR5 Web Simulator — FastAPI 앱.

실행:  cd backend && uv run uvicorn app.main:app --reload
문서:  http://localhost:8000/docs
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.db import init_db
from app.routers import robot, kinematics, programs, simulate

app = FastAPI(title="UR5 Web Simulator", version="0.1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

init_db()                                   # 테이블 생성 (멱등)

for module in (robot, kinematics, programs, simulate):
    app.include_router(module.router)


@app.get("/")
def root():
    return {"name": "UR5 Web Simulator", "docs": "/docs"}
