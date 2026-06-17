"""API 요청/응답 스키마 (SQLModel, table=False)."""
from sqlmodel import SQLModel


class PoseIn(SQLModel):
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class ProgramSummary(SQLModel):
    program_id: str
    n_poses: int


class IKRequest(SQLModel):
    pose: list[float]                       # [x,y,z, qx,qy,qz,qw]
    seed: list[float] | None = None         # 초기 관절각(rad)


class IKResponse(SQLModel):
    theta: list[float]
    converged: bool
    tcp: list[float]


class RunRequest(SQLModel):
    program_id: str | None = None
    waypoints: list[list[float]] | None = None   # 관절 경유점(rad)
    controller: str = "computed_torque"          # pd | pid | computed_torque
    gains: dict[str, float] = {}
    gravity_comp: bool = True
    t_seg: float = 1.2
    hold: float = 0.6
    payload: float = 0.0                         # 끝단 미지 질량(kg), plant 에만
    model_scale: float = 1.0                     # 컨트롤러 질량 배율(1.0=정확)


class RunResponse(SQLModel):
    t: list[float]
    theta: list[list[float]]
    tcp: list[list[float]]
    error: list[float]
    torque: list[list[float]]
    waypoints_tcp: list[list[float]]


class ControllerSpec(SQLModel):
    name: str
    label: str
    params: list[dict]
