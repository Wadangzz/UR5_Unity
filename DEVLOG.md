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

## 2026-06-17

### 🎯 오늘 한 것 — 웹 시뮬레이터 풀스택 + 제어 인터랙션 (develop, ~30 커밋)

**메시/렌더**
- `/meshes` 정적 서빙 (`app/meshes_setup.py`: robot_descriptions UR5 URDF+메시 복사, gitignore)
- `frontend/` (synex 컨벤션: 소스 `app/`, `main.web.tsx`, `root.tsx`, alias `@`):
  pnpm + Vite8 + React19 + **r3f v9 + drei v10** + three + urdf-loader
- `RobotView`: urdf-loader 로 UR5 메시 렌더. **Z-up 씬**(camera.up=+Z → 월드축=로봇축=직교축 일치),
  `flat`(톤매핑 끔)+다방향 조명, 좌하단 **XYZ 기즈모**
- 백엔드 포트 **8500** (8000=synex), `backend/main.py` 진입점(`uv run main.py`)

**제어 / 시뮬**
- 관절 슬라이더 jog(브라우저 FK, `setJointValues` — 백엔드 왕복 0)
- `POST /api/run` 제어 시뮬 → θ(t) `requestAnimationFrame` 재생 (현재자세/프로그램 순회)
- 제어기 4종: **PD+중력보상 · PID · Computed Torque**(+Ki=PID-CT) · **임피던스**
  (직교 6-DOF, `V=log(T⁻¹T_d)` body twist, `τ=Jbᵀ(K·V−D·Jb·θ̇)+g`)
- 게인 슬라이더 + **ζ·ωn 배지 + 적분 안정여유**(Routh `Ki<Kp·Kd`)
- **자동튜닝(극배치)** 토글: 목표 정착시간 → `Kp=ωn², Kd=2ωn` (관절용, 임피던스 제외)
- **모델 불확실성**(payload·질량배율, plant≠ctrl) + **외란**(3축 외력 Ftip → 임피던스 `Δx=F/K` 컴플라이언스)
- 엔진: **정상상태(속도≈0) 정착까지 가변 적분 + 발산 가드** → `settle_time/steady_state_error/diverged`
- 성능: stiff 솔버 **Radau** (PD/PID 37~106s → ~3s)
- **특이점**: DLS IK 회피(σ_min<0.04 감쇠) + manipulability(`w=√det(JJᵀ)`, σ_min) 라이브 표시

**직교 / 티치**
- `PoseEditor`: 직교 6-DOF(X,Y,Z,RX,RY,RZ) **라이브 IK ↔ 관절 FK 양방향**, σ_min 배지
- `POST /api/fk` 추가. `ProgramList`: 티치(**관절각+Cartesian 둘 다** 저장)·실행·포즈 개별 삭제
- `Plots`(recharts): 추종오차·토크 + 정착/정상상태오차/발산 표시

**툴체인 / 구조**
- ESLint/Prettier **synex 설정 통일**(+tailwind 클래스 정렬), 제어기 스펙을 **프론트 const**로(API 불필요)

**개념 정리(대화):** PD/PID/CT·중력보상·역동역학(RNE)·PID-CT의 I게인·정착 vs 정상상태·
외란 vs 모델불확실성·임피던스(task-space PD, SE3, Jᵀ)·강성/감쇠=가상 게인·
SO(3)/SE(3) 군 vs so(3)/se(3) 대수·manipulability 특이점 판정.

---

## 📋 다음 할 것 (TODO)

- [ ] **멀티로봇** — (b) 백엔드 큐레이션(robot_descriptions ur5/ur10/panda/iiwa) + 프론트 선택,
      `mr_urdf_loader` 로 `{Mlist,Glist,Slist}` 추출(sim 이미 인자로 소비), 메시 서빙 일반화.
      → 그 뒤 (a) **URDF 업로드**(메시 동반 필요)
- [ ] **속도제어**(관절 θ̇_d 적분→추종 / 직교 resolved-rate `J†`) + **순수 토크 입력 모드**
- [ ] **힘제어**(hybrid position/force)·어드미턴스, **마찰·토크포화·센서노이즈** 현실 모델
- [ ] 특이점: resolved-rate 에 DLS `J†`, 조작성 게이지(σ_min 이미 표시)
- [ ] **Docker** 배포 + README 재작성(웹 기준)

### ⚠️ 주의/메모
- 브랜치 `develop`, **push 미실시**(직접)
- 실행: 백엔드 `cd backend && uv run main.py`(:8500), 프론트 `cd frontend && pnpm dev`(:5174)
- `Pose` 등 **DB 스키마 변경 시 `backend/robot.db` 삭제 후 재시작**(create_all 은 ALTER 안 함)
- **UR5 zero(home) 자세가 특이점** → 시작 시 σ_min=0 경고는 정상
- 런타임 `robot.db`·`outputs/`·`*.npz`·`backend/app/meshes/`·`node_modules`·`dist` 는 gitignore
