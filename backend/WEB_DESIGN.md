# UR5 Web Simulator — 구조 설계

브라우저에서 실제 UR5 메시로 **포즈 저장 → 프로그램 실행 → 동역학·제어 시뮬레이션**을
보여주는 웹 앱. (기존 Unity GUI 재현 + 동역학·제어 추가, Unity·ROS 불필요)

---

## 1. 전체 아키텍처

```
┌──────────── Browser (React) ─────────────┐         ┌────────── Python (FastAPI) ──────────┐
│  RobotView  (react-three-fiber + urdf-    │  REST   │  robot_math / ur5_model / Dynamics    │
│             loader)  ← 실제 UR5 메시, GPU │ ◄─────► │  controllers: PD/PID/CT/Impedance     │
│  ControlPanel (제어기·게인 슬라이더)       │         │  sim engine (scipy solve_ivp)         │
│  PoseEditor  (관절/직교 목표, Save)        │  (WS)   │  trajectorysqlite (포즈 DB, 기존)     │
│  Plots  (추종오차·토크)                    │ ◄─────► │  /meshes 정적 서빙 (URDF+메시)         │
└────────────────────────────────────────────┘         └───────────────────────────────────────┘
        ↑ θ(t) 받아서 브라우저가 재생               ↑ 무거운 계산(시뮬)은 여기서 1번
```

**핵심 원칙: 무거운 계산은 백엔드 1번, 애니메이션은 브라우저에서.**
→ 프레임마다 왕복 안 하므로 API 속도가 병목이 아님.

---

## 2. 기술 스택

| | |
|---|---|
| **백엔드** | FastAPI + uvicorn + pydantic. 기존 `robot_math`/`ur5_model`/`trajectorysqlite` 재사용 |
| **프론트** | React (Vite) + `react-three-fiber`(Three.js) + `urdf-loader`(브라우저 URDF 메시) |
| **그래프** | recharts (추종오차·토크) |
| **상태관리** | zustand (가벼움) |
| **통신** | REST(명령·시뮬) + WebSocket(실시간 jog/impedance, 2단계) |

---

## 3. 통신 설계 — "왜 안 느린가"

| 동작 | 방식 | 이유 |
|------|------|------|
| 관절 슬라이더로 포즈 보기 | **프론트에서 FK** (urdf-loader가 처리) | 왕복 0, 즉각 |
| 직교 목표 → 관절각 | `POST /ik` (1회) | IK는 백엔드 (Kinematics.IK) |
| 프로그램 실행 | `POST /run` (1회) → θ(t) 전체 받음 → **브라우저 재생** | 왕복 1번, 계산이 병목(API 아님) |
| 실시간 외력/jog (2단계) | WebSocket | 저지연 스트리밍 |

> ❌ 절대 금지: 프레임마다 REST 호출 (이게 유일하게 느려지는 길)

---

## 4. API 엔드포인트 (REST, prefix `/api`)

| 메서드 | 경로 | 입력 | 출력 |
|--------|------|------|------|
| GET | `/robot` | — | `{joint_names, limits, n, mesh_url}` |
| POST | `/ik` | `{pose:[x,y,z,qx,qy,qz,qw], seed?}` | `{theta:[6], converged}` |
| POST | `/fk` | `{theta:[6]}` | `{tcp:[x,y,z], quat:[4]}` (선택) |
| GET | `/programs` | — | `[{id, n_poses}]` |
| GET | `/programs/{id}` | — | `{id, poses:[[7]]}` |
| POST | `/programs/{id}/poses` | `{pose:[7]}` | `ok` |
| DELETE | `/programs/{id}` | — | reset |
| GET | `/controllers` | — | `[{name, params:[{key,default,min,max}]}]` |
| **POST** | **`/run`** | `{program_id 또는 waypoints, controller, gains, t_seg, hold}` | **`{t, theta[n][6], tcp[n][3], error[n], torque[n][6], waypoints_tcp}`** |

WebSocket (2단계): `WS /ws/jog` — 실시간 관절 jog / impedance 외력
정적: `/meshes/...` — URDF + 메시 파일 (urdf-loader가 로드)

---

## 5. 데이터 모델 (pydantic)

```python
Pose          = list[float]            # [x,y,z, qx,qy,qz,qw]  (DB 기존 스키마와 동일)
RunRequest    = { program_id?: int, waypoints?: [[6]],        # 관절각(rad) 경유점
                  controller: "pid"|"computed_torque"|"impedance"|"pd",
                  gains: {kp,ki,kd, ...}, t_seg: float, hold: float }
RunResponse   = { t:[n], theta:[n][6], tcp:[n][3],
                  error:[n], torque:[n][6], waypoints_tcp:[m][3] }
ControllerSpec= { name, params:[{key, default, min, max}] }
```

---

## 6. 디렉토리 구조

```
pythonscript/
├─ robot_math.py · ur5_model.py · robots.py · trajectorysqlite.py · paths.py   (core, 재사용)
├─ server/                      # FastAPI 백엔드
│   ├─ main.py                  # app + 라우트
│   ├─ schemas.py               # pydantic 모델
│   ├─ sim.py                   # 제어기 + solve_ivp 시뮬 엔진
│   └─ meshes/                  # URDF + 메시 (robot_descriptions에서 복사, 정적 서빙)
├─ demos/ · docs/ · outputs/ ...
└─ web/                         # React 프론트 (Vite)
    └─ src/
        ├─ App.jsx
        ├─ api.js               # fetch 래퍼
        ├─ store.js             # zustand
        └─ components/
            ├─ RobotView.jsx    # urdf-loader + θ(t) 재생
            ├─ ControlPanel.jsx # 제어기 선택 + 게인 슬라이더
            ├─ PoseEditor.jsx   # 관절/직교 목표 + Save
            ├─ ProgramList.jsx  # 저장된 포즈, Run/Reset
            └─ Plots.jsx        # 오차·토크 차트
```

> 백엔드 `server/`는 `demos/`처럼 sys.path shim 한 줄로 core 임포트. `uvicorn server.main:app`.

---

## 7. 데이터 흐름 (시퀀스)

**포즈 저장**
```
사용자 관절 슬라이더 → RobotView FK 표시(브라우저) → [Save]
   → POST /programs/1/poses {pose:[x,y,z,quat]} → DB 저장 → ProgramList 갱신
```

**프로그램 실행**
```
[Run] (controller=pid, gains) → POST /run {program_id:1, controller, gains}
   백엔드: DB 포즈 로드 → 각 포즈 IK → 관절 경유점 → 구간 quintic → 제어 시뮬(solve_ivp)
        → {t, theta[], error[], torque[]} 반환
   프론트: theta[] 를 메시에 재생 + Plots에 오차·토크 표시
```

---

## 8. 구현 단계

| 단계 | 내용 | 테스트 |
|------|------|--------|
| **1** | **FastAPI 백엔드** — `/robot`,`/ik`,`/programs`,`/run`,`/controllers` | curl/파이썬으로 검증 (브라우저 불필요) |
| 2 | URDF + 메시 정적 서빙 (glTF 변환 검토) | 브라우저에서 메시 로드 확인 |
| 3 | React 스캐폴드 + RobotView(메시 렌더) | 정적 포즈 표시 |
| 4 | ControlPanel/PoseEditor/ProgramList ↔ API 연결 | Save/Run 동작 |
| 5 | Plots(오차·토크) + 폴리시 | — |
| 6 | (선택) WebSocket 실시간 jog/impedance | — |

→ **1단계(백엔드 API)부터.** 우리 시뮬을 API로 노출하고 파이썬에서 바로 검증 가능.
```

