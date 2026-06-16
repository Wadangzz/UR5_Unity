# DEVLOG

UR5 로봇 시뮬레이터 개발 로그.

---

## 2026-06-16

### 🎯 오늘 한 것

**동역학·제어 엔진 (main 브랜치)**
- `PyQt5 → PySide6` 마이그레이션
- `robot_math.py`: 단일 `SE3` 클래스를 **SO3 / SE3 / Kinematics / Dynamics** 4계층으로 리팩터링 (Lie 군·대수 구조)
- **RNE 동역학** 추가: `inverse_dynamics`, `mass_matrix`, `gravity_forces`, `coriolis_forces`, `forward_dynamics`
- `ur5_model.py`: Modern Robotics 표준 UR5 파라미터(Mlist/Glist/Slist) + FK/IK/fk_skeleton
- `SE3.log` 결정적 처리(난수 제거), `Kinematics.IK_solutions`(multi-seed)
- 제어 데모: 중력보상 · PD · **PID** · **Computed Torque** · **임피던스**
- 통합 데모: 원하는자세 → IK → 경로 → 제어 → 도달 (`full_pipeline`)

**시각화**
- `fk_skeleton`(실제 관절축) — 막대 스켈레톤 개선
- **실제 UR5 URDF 메시** 시각화 (`yourdfpy` + `robot_descriptions`, Windows OK)
  - `urdf_view`(정적, 범용) / `urdf_dynamics_anim`(3막) / `urdf_multi_waypoint`(PID) /
    `urdf_pid_anim`(PD vs PID) / `urdf_impedance_anim`(stiff vs soft)
- 산출물 `outputs/`(gitignore), 데모 14개 `demos/` 로 정리

**웹 전환 (develop 브랜치)** ← 오늘의 메인
- 🗑 Unity 프로젝트 + Qt GUI 전부 제거 (~177MB 다이어트, main엔 보존)
- `pythonscript/` → `backend/` (uv 패키지)
- **FastAPI + SQLModel 백엔드** (레이어 분리):
  `app/routers` → `app/controllers` → `app/models`·`app/schemas`·`app/sim`·`app/db`
  - 엔드포인트: `/api/robot` `/api/ik` `/api/controllers` `/api/programs`(CRUD) `/api/run`
- 코어 라이브러리 → `app/core/` (robot_math·robots·ur5_model·paths)
- 검증: TestClient — IK 9e-5m, run CT 1.6e-10, program→IK→경유점 순회 정상

### 📂 현재 구조 (develop)

```
UR5_Unity/
├─ backend/
│   ├─ app/
│   │   ├─ core/        robot_math · robots · ur5_model · paths
│   │   ├─ routers/     robot · kinematics · programs · simulate
│   │   ├─ controllers/ kinematics · programs · simulate
│   │   ├─ models.py · schemas.py · sim.py · db.py · main.py
│   ├─ demos/           제어·시각화 데모 14개
│   ├─ docs/            Inverse_Dynamics.md
│   ├─ WEB_DESIGN.md    웹 아키텍처 설계
│   └─ pyproject.toml · uv.lock
├─ DEVLOG.md (이 파일)
└─ README.md  ⚠️ 아직 Unity 설명 (재작성 필요)
```

### ▶ 실행법
```bash
cd backend
uv sync                               # 최초 1회 (환경 생성)
uv run uvicorn app.main:app --reload  # → http://localhost:8000/docs (Swagger)
uv run python demos/gravity_demo.py   # 데모
```

### ✅ 오늘 커밋 (develop)
- `632c6f3` 웹 전환: Unity/Qt 제거 + pythonscript→backend
- `e8e7955` FastAPI 레이어 분리 (router/controller/model/schema)
- `27b0aaf` 코어 → app/core/

---

## 📋 내일 할 것 (TODO)

웹 시뮬레이터 단계 (설계: `backend/WEB_DESIGN.md`)

- [ ] **2. URDF·메시 정적 서빙** — robot_descriptions UR5 메시를 `/meshes` 로 (프론트가 로드)
- [ ] **3. frontend/ 스캐폴드** — Vite React + `react-three-fiber` + `urdf-loader`
      - `RobotView`(메시 렌더 + θ(t) 재생), `ControlPanel`(제어기·게인 슬라이더),
        `PoseEditor`(관절/직교목표 + Save), `ProgramList`(Run/Reset), `Plots`(오차·토크)
- [ ] **4. 컨트롤 ↔ API 연결** — Save/Run → `/api/run` → 메시 재생
- [ ] **5. Plots** (recharts) + 폴리시
- [ ] **6. Docker** — backend Dockerfile + frontend Dockerfile + docker-compose
- [ ] **마무리** — README 재작성(웹 기준), `pythonscript/` 껍데기 삭제(.venv 잠김 풀리면)

### 🚀 확장 아이디어 (킬러 기능)
- [ ] **URDF 업로드 → 범용 로봇 시뮬** — `mr_urdf_loader` 로 URDF→{Mlist,Glist,Slist}
      추출 → `app/sim` 이 그대로 소비 (이미 인자로 받음). UR5 전용 → "뭐든 시뮬"
- [ ] WebSocket 실시간 jog / 임피던스 외력 인터랙션

### ⚠️ 주의/메모
- 브랜치 `develop`. **push는 직접** 할 예정 (오늘 미push)
- `pythonscript/` 폴더 껍데기(.venv만, gitignore) 남음 — IDE/터미널 닫으면 삭제 가능
- 런타임 DB `robot.db`, 산출물 `outputs/`, `*.npz` 는 gitignore
- 데모 실행은 `uv run python demos/xxx.py` (shim 이 backend/ 를 path 에 추가)
