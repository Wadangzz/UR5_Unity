"""SQLModel DB 테이블."""
from sqlmodel import SQLModel, Field, Column, JSON


class Pose(SQLModel, table=True):
    """프로그램에 저장된 말단 자세.

    Cartesian(위치+quaternion)은 표시·편집·이식용, theta(관절각)는 정확 재현용.
    둘 다 저장한다 — run-by-program 은 theta 가 있으면 그대로 순회, 없으면 IK.
    """
    id: int | None = Field(default=None, primary_key=True)
    program_id: str = Field(index=True)
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    theta: list[float] = Field(default_factory=list, sa_column=Column(JSON))
