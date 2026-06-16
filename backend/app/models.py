"""SQLModel DB 테이블."""
from sqlmodel import SQLModel, Field


class Pose(SQLModel, table=True):
    """프로그램에 저장된 말단 자세 (위치 + quaternion)."""
    id: int | None = Field(default=None, primary_key=True)
    program_id: str = Field(index=True)
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
