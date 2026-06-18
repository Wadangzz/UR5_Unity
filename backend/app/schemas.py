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
    theta: list[float] = []                  # 티치 시점 관절각(rad), 정확 재현용


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


class FKRequest(SQLModel):
    theta: list[float]                      # 관절각(rad)


class FKResponse(SQLModel):
    pose: list[float]                       # [x,y,z, qx,qy,qz,qw]
    tcp: list[float]                        # [x,y,z]
    w: float = 0.0                          # 조작성 √det(JJᵀ)=∏σ_i
    sigma_min: float = 0.0                  # 최소 특이값 (→0 이면 특이점)


class RunRequest(SQLModel):
    program_id: str | None = None
    waypoints: list[list[float]] | None = None   # 관절 경유점(rad)
    start: list[float] | None = None             # 프로그램 실행 출발 관절각(현재 자세), 없으면 ready
    controller: str = "computed_torque"          # pd | pid | computed_torque
    gains: dict[str, float] = {}
    traj_mode: str = "joint"                      # joint(관절 quintic) | task(직교 직선)
    gravity_comp: bool = True
    t_seg: float = 1.2
    hold: float = 0.6
    payload: float = 0.0                         # 끝단 미지 질량(kg), plant 에만
    model_scale: float = 1.0                     # 컨트롤러 질량 배율(1.0=정확)
    disturbance: list[float] = [0.0, 0.0, 0.0]   # 끝단 외력(base 프레임, N), 모션 후 인가


class RunResponse(SQLModel):
    t: list[float]
    theta: list[list[float]]
    tcp: list[list[float]]
    error: list[float]
    torque: list[list[float]]                 # 토크제어 출력(N·m). 속도제어 시 빈 배열
    qdot: list[list[float]] = []             # 속도제어 출력 commanded 관절속도(rad/s)
    ee_pose: list[list[float]] = []          # 프레임별 EE pose [x,y,z,qx,qy,qz,qw] (재생 중 직교좌표 표시)
    sigma_min: list[float] = []              # 프레임별 최소특이값 (재생 중 manipulability 표시)
    waypoints_tcp: list[list[float]]
    settle_time: float | None = None         # 평형(정상상태) 도달 시각(s), 미도달 None
    steady_state_error: float | None = None  # 정상상태 오차 ‖θ-θ_d‖(rad)
    diverged: bool = False                   # 발산 여부
    traj_error: str | None = None            # task궤적 생성 실패 사유(특이점/작업영역밖), 없으면 None
