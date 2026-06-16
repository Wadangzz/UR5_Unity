"""SQLModel DB 엔진/세션 (SQLite)."""
from sqlmodel import SQLModel, create_engine, Session
from app import models  # noqa: F401  (테이블 메타데이터 등록)

engine = create_engine("sqlite:///robot.db",
                       connect_args={"check_same_thread": False})


def init_db():
    SQLModel.metadata.create_all(engine)


def get_session():
    with Session(engine) as session:
        yield session
