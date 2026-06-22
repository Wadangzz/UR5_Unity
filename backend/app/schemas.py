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
    robot_id: str = "ur5"                   # 대상 로봇 (레지스트리 id)


class IKResponse(SQLModel):
    theta: list[float]
    converged: bool
    tcp: list[float]


class FKRequest(SQLModel):
    theta: list[float]                      # 관절각(rad)
    robot_id: str = "ur5"                   # 대상 로봇 (레지스트리 id)


class FKResponse(SQLModel):
    pose: list[float]                       # [x,y,z, qx,qy,qz,qw]
    tcp: list[float]                        # [x,y,z]
    w: float = 0.0                          # 조작성 √det(JJᵀ)=∏σ_i
    sigma_min: float = 0.0                  # 최소 특이값 (→0 이면 특이점)


class ForceTask(SQLModel):
    """힘/하이브리드 제어의 환경(벽)·목표 사양. 프론트는 fd/gap/move_len 만 보내고
    나머지는 기본값, 벽 point 는 백엔드가 시작자세 FK 로 계산해 채운다."""
    fd: float = 20.0                             # 목표 누름힘(N)
    gap: float = 0.05                            # 시작 시 벽까지 거리(m)
    k_env: float = 4000.0                        # 벽 강성(N/m)
    b_env: float = 40.0                          # 벽 감쇠(N·s/m)
    # 누르는 방향은 EE 도구축(시작자세 기준)으로 잡는다 — 엔드이펙터가 보는 쪽으로 누름.
    tool_axis: list[float] = [0.0, 0.0, 1.0]     # 누름 도구축(EE 로컬, 보는 방향)
    tan_axis: list[float] = [1.0, 0.0, 0.0]      # 접선 도구축(EE 로컬, hybrid 선긋기)
    normal: list[float] = [0.0, 0.0, 1.0]        # 벽 외향 법선(base) — 백엔드가 계산해 채움
    tangent: list[float] = [1.0, 0.0, 0.0]       # 접선 방향(base) — 백엔드가 계산해 채움
    move_len: float = 0.10                       # 접선 이동거리(m, hybrid)
    move_start: float = 0.8                      # 접선 이동 시작(s, hybrid)
    move_time: float = 1.5                       # 접선 이동 소요(s, hybrid)
    t_end: float = 4.0                           # 시뮬 길이(s)


class RunRequest(SQLModel):
    robot_id: str = "ur5"                        # 대상 로봇 (레지스트리 id)
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
    friction: float = 0.0                        # 관절 쿨롱 마찰(N·m, plant) — 토크 제어만
    tau_max: float = 0.0                         # 액추에이터 토크 한계(N·m, 0=무제한)
    control_rate: float = 0.0                    # 0=연속(이상) / >0=이산 ZOH 제어율(Hz)
    noise: float = 0.0                           # 센서 측정 노이즈 std(rad) — 토크 제어만
    noise_seed: int = 0                          # 노이즈 realization seed(고정=재현성)
    force_task: ForceTask | None = None          # 힘/하이브리드 제어 환경(벽)·목표


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
    force: list[float] = []                   # 측정 법선반력(N) — 힘/하이브리드 제어
    force_des: float | None = None            # 목표 힘(N) — 힘/하이브리드
    tan_pos: list[float] = []                 # 접선 실제 변위(m) — 하이브리드
    tan_des: list[float] = []                 # 접선 목표 변위(m) — 하이브리드
    wall: dict[str, list[float]] | None = None  # 벽 {point, normal}(base) — 3D 렌더용
