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

## 2026-06-18

### 🎯 오늘 한 것 — 속도제어 + 어드미턴스 + 리뷰/정리 (develop)

**속도제어 추가 (MR §11.3)** ← 메인
- 토크제어(2차계, `forward_dynamics`)와 **별개 plant** 도입: 속도입력은 **1차계** —
  θ̇ 가 곧 제어출력, θ 만 적분(기구학 시뮬), stiff 아님 → **RK45**(Radau 불필요, 더 빠름)
- `sim.py`: `run_simulation` 을 **디스패처**로 분리
  - `_run_torque` (기존 본체 그대로 — 토크 4종 회귀 검증, 동작 불변)
  - `_run_velocity` (신규): **joint_velocity**(§11.3.2 `θ̇=θ̇_d+Kp·e+Ki∫e`),
    **resolved_rate**(§11.3.3 `Vb=Ad_{X⁻¹X_d}·V_d+Kp·Xe`, `θ̇=J†_b·Vb`, `Xe=log(X⁻¹X_d)`)
  - 공유 헬퍼 추출: `_make_ref` · `_detect_settle` · `_keep_idx` · `_blowup_event`
- `ur5_model.dls_inv` 공개(별칭) — resolved-rate 가 J† 재사용(특이점 자동 감쇠)
- `schemas.RunResponse.qdot` 신설(속도제어 출력 rad/s, 토크모드는 빈 배열)
- 프론트: `CONTROLLERS` 속도제어 2종(kp/ki, **kd 없음**) + **`POLE_PLACE` 상수**로
  자동튜닝(극배치=2차계용) 가시성 중앙화 / `Plots` 는 qdot 있으면 **θ̇** 표시
- 검증: 수렴 sse 1e-6~1e-8, home 특이점서 DLS 로 θ̇ 유계(발산 없음), API 200, tsc/eslint 0

**어드미턴스 추가 (MR §11.7.2)** — 임피던스의 쌍
- `sim.py: _run_admittance` (기구학 **모션 plant**, RK45). 가상 `M·Δẍ+B·Δẋ+K·Δp=f_ext`
  를 시뮬해 변위 Δp 생성 → **resolved-rate 로 추종**. 임피던스가 토크 plant 인 것과
  대칭(**임피던스=움직임보고힘냄 / 어드미턴스=힘보고움직임**). 힘센서 없는 시뮬은 외란을 f_ext 로 재사용
- 내부 추종게인 `KP_TRACK=20` — 가상 동역학(ωn=√(K/M))보다 빨라야 오버슛 등이 안 가려짐
- 정착판정: 외란이 모션종료(T) 후 인가되므로 '한 번 흔들린 뒤(excited) 평형' 으로 판정
- 프론트: 어드미턴스 컨트롤러(게인 **M/B/K**). Plots qdot·POLE_PLACE 기존 처리로 자동 대응
- 검증: 컴플라이언스 `Δp_ss=f/K`(K=600→0.0667m 이론일치), 가상질량 2차응답
  (ζ=0.58/0.22/0.11 → 오버슛 6/35/64%, 정상상태는 M 무관), 회귀 5종 무손상

**리뷰 / 정리**
- `robots.py` **제거**: 앱 미사용·기구학전용·mm단위. `ur5_model`(`_IKAdapter`)이 이미 대체.
  범용화는 추후 **URDF→{Mlist,Glist,Slist} 로더**로(멀티로봇 TODO와 일치) — 방향이 다름
- **PDF 읽기 환경**: `pypdf` dev 추가, Modern Robotics 교재를 `docs/`(gitignore, 로컬)에 비치.
  페이지 오프셋 **+18**(인쇄 p.1 = PDF p.19). `ruff` dev 추가(설정만 있고 미설치였음)
- `PoseEditor` σ_min 라벨 정리(Manipulability 표기)

**개념 정리(대화):** 속도제어 vs 토크제어(1차계 vs 2차계, plant 자체가 다름)·
resolved-rate(`J†`, `Ad_{X⁻¹X_d}` 피드포워드 트위스트)·**임피던스 구현이 MR (11.65)의
M=0+중력보상 단순화형과 정확히 일치**함을 교재 대조로 검증·임피던스↔어드미턴스 쌍대성
(Z=F/X ↔ Y=X/F, stiff환경엔 어드미턴스/soft환경엔 임피던스)·보스턴다이나믹스 푸시리커버리
=컴플라이언스(임피던스)+**역진자/캡처포인트 균형제어**(별개 상위레벨, 우리 고정베이스엔 없음).
- **redundancy/널공간 메모**(멀티로봇 대비): task-space 제어 시 자세 자유도는 **6-DOF UR5
  에선 결정됨(이산 branch)**, **7-DOF(Panda/iiwa)부터 1자유도 자유** → `θ̇=J†V+(I−J†J)θ̇₀`
  널공간으로 자세 능동제어(선호자세·관절한계회피·manipulability↑). 멀티로봇 단계에서 의미.

**라이브 표시 + task-space 궤적 + ready 자세**
- **재생 중 라이브 표시**: 백엔드가 θ(t) 계산 시 EE pose·σ_min 시계열을 응답에 실어
  보내고(프레임당 REST 금지 원칙), 재생 시 PoseEditor 직교좌표·σ_min 배지가 실시간 갱신.
- **task-space 궤적**(MR §9): `traj_mode=joint|task`. 궤적생성⊥제어기 — task 는 직교 SE(3)
  직선보간→IK 샘플(precompute)→θ_d. **제어기 7종 무수정**으로 직선 추종(직선이탈 0.0mm
  vs joint 27mm 곡선). 특이점/작업영역밖서 IK 실패·관절속도 폭발 시 절단+`traj_error`(=옳음).
  · sim.py 궤적생성 중앙화(`_build_ref`), `_run_*` 시그니처 `(WP, ref, T, ...)` 통일.
- **비특이 ready 자세**: kinematic zero(전관절0)=특이점이라 task 출발점으로 부적합 →
  실제 UR 처럼 `READY=[0,-90,90,-90,-90,0]`(σ_min≈0.22) 정의·`/api/robot` 노출, 초기/Run 출발.
  영점 버튼은 특이점 시연용 유지. **프로그램 실행은 현재 자세에서 출발**(start 필드).

**현실 모델 (토크 plant 한정)**
- **마찰**: 쿨롱 `τ_f=Fc·tanh(θ̇/ε)` plant 반력(컨트롤러 모름) → **PD 정상상태오차**(마찰15→sse 0.25),
  **PID 의 I항이 회복**(0.05). "왜 I가 필요한가"가 체감됨.
- **토크 포화**: 명령토크 `[-τ_max, τ_max]` 클립 → 성능저하(τ_max=40 미정착)·부족시 발산(20).
- `_run_torque` 에 `applied()`(포화)+rhs 마찰. 속도/어드미턴스(기구학)는 무관.
- (센서노이즈는 solve_ivp 결정성 처리 필요 → 다음 단계로 분리)

**개념 정리(대화 2):** joint 궤적(θ 보간, 말단 곡선) vs task 궤적(X 보간, 말단 직선)·
제어기 native 공간 분류(관절 4 / task 3, FK·IK 로 상호변환 가능)·**task 궤적의 특이점
IK 실패는 버그가 아니라 물리적으로 옳은 거동**(직교 직선은 특이점서 실행불가)·kinematic
zero(특이점) vs 운용 ready 자세(비특이) 구분.

### ✅ 오늘 커밋 (develop)
- `63f65e2` feat: 속도제어 — resolved-rate + 관절속도 (MR §11.3) + robots.py 제거
- `c588de0` feat: 어드미턴스 제어 (MR §11.7.2) — 임피던스의 쌍
- `ded793b` feat: 재생 중 직교좌표·manipulability 라이브 표시
- `3a7109c` feat: task-space 궤적(직교 직선) + 비특이 ready 자세 + 프로그램 현재자세 출발
- `d643e4f` feat: 현실 모델 — 관절 마찰 + 토크 포화 (토크 plant)

---

## 📋 다음 할 것 (TODO)

- [ ] **멀티로봇** — (b) 백엔드 큐레이션(robot_descriptions ur5/ur10/panda/iiwa) + 프론트 선택,
      `mr_urdf_loader` 로 `{Mlist,Glist,Slist}` 추출(sim 이미 인자로 소비), 메시 서빙 일반화.
      → 그 뒤 (a) **URDF 업로드**(메시 동반 필요)
- [x] **속도제어**(관절 θ̇_d / 직교 resolved-rate `J†`) ✅ 2026-06-18
      └ **순수 토크 입력 모드**(§11.4)는 τ 입력 스키마 필요 → 보류
- [x] **어드미턴스**(§11.7.2) ✅ 2026-06-18
- [x] **task-space 궤적**(직교 직선, §9) + 비특이 ready 자세 ✅ 2026-06-18
- [x] 특이점: resolved-rate DLS `J†` + manipulability(σ_min) 라이브 표시 ✅
- [x] **현실모델 — 마찰·토크포화** ✅ 2026-06-18 (└ 센서노이즈는 보류)
- [ ] **힘제어**(hybrid position/force §11.6) — 렌치 `τ=Jᵀ·F` + 접촉/환경 모델 선행 필요
- [ ] **센서노이즈**(현실모델 나머지) — solve_ivp 결정성(격자 precompute+보간) 처리
- [ ] **redundancy/널공간** 제어(7-DOF, `(I−J†J)θ̇₀`) — 멀티로봇 후
- [ ] **Docker** 배포 + README 재작성(웹 기준)

### ⚠️ 주의/메모
- 브랜치 `develop`, **push 미실시**(직접)
- 실행: 백엔드 `cd backend && uv run main.py`(:8500), 프론트 `cd frontend && pnpm dev`(:5174)
- `Pose` 등 **DB 스키마 변경 시 `backend/robot.db` 삭제 후 재시작**(create_all 은 ALTER 안 함)
- **UR5 zero(home) 자세가 특이점** → 시작 시 σ_min=0 경고는 정상
- 런타임 `robot.db`·`outputs/`·`*.npz`·`backend/app/meshes/`·`node_modules`·`dist` 는 gitignore
