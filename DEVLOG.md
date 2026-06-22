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

**센서노이즈 — 시도 후 보류 (성능 한계 발견, 되돌림)**
- 연속 노이즈 in Radau: **88s**(암시적 솔버가 jitter 분해하려 스텝 잘게 쪼갬).
- 이산 ZOH 루프 스파이크(1kHz, RK4): **~50-60s + PID/임피던스 NaN**. 병목 =
  `forward_dynamics`(순수 NumPy RNE) **~3ms/호출**, 고정스텝이라 ~16000회 호출.
  Radau가 빠른 건 적응적이라 호출 수백번뿐(언어가 아닌 **알고리즘·오버헤드** 차이).
- 결론: 이산 시뮬은 **fd 가 빨라야** 가능. 해법 = **Numba 로 RNE JIT 컴파일**
  (~3ms→~50µs 기대; Numba 도 Python 이라 소스 거의 그대로, 호출 오버헤드만 제거).
  되면 이산 ZOH·노이즈·제어율 노브가 다 열림 → 별도 과제(아래 TODO).

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

### 🚀 numba 고속화 + 이산 ZOH (branch `numba-dynamics`, develop 미머지)

센서노이즈 보류의 근본원인(순수 NumPy RNE ~3.5ms/호출)을 해결하러 시작 → 큰 가속 + 이산 제어로 확장.

- **numba JIT 동역학** (`app/core/dynamics_fast.py`): RNE 전체를 `@njit(cache=True)` 로
  재작성(스택 배열 입력, SE3 헬퍼 hat/exp6/adjoint/ad 포함). robot_math.Dynamics 와
  **비트일치**(max|diff|=0). fd 3.5ms→254µs(**~14x**), 연속 시뮬 2-3.5s→**~0.3s(~7x)**.
  `sim._run_torque` 가 `df.*` 로 호출(스택 1회 변환). `sim.warmup()`+부팅 호출로 첫 요청 컴파일 지연 흡수.
- **하이브리드 제어 모드** (`control_rate`): 0=연속 Radau(이상·빠름·기본) / >0=이산 ZOH 루프
  (rate Hz 샘플·토크 ZOH 유지, plant RK4). 실제 디지털 제어 — **제어율 낮추면 불안정 재현**.
  · 실측: PD 1kHz 안정/500Hz 발산, hot 게인(PID·임피던스)은 2kHz 필요. 자동튜닝 ts=0.2(Kp=400)→2kHz 필요.
- **센서 노이즈**: 샘플 순간 주입(이산 전용, 연속은 비활성). 토크 chatter→D항 증폭. noise>0 자동 이산화.
- `_detect_settle` 발산판정에 절대임계 추가(이산 미세 task오차 오판 수정). `DEMOS.md`(튜닝 레시피+안정성표) 추가.
- **개념(대화 3):** Python이 아니라 *JIT 안 된 NumPy 호출 오버헤드*가 병목(MATLAB도 동일, GPU는 단일로봇
  직렬 RNE 부적합—수천 env 병렬 RL용)·연속(제어율∞ 이상화) vs 이산 ZOH(실제 디지털, sim-to-real)·
  빠른 정착=높은 게인=높은 제어율 필요(절벽).
- **현 상태**: 힘제어는 제어측(Jᵀ·Ftip·빠른동역학) 준비됨, **접촉/환경 모델만 선행하면** 가능.
  멀티로봇은 **동역학 엔진이 이미 generic**(Mlist/Glist/Slist 인자, numba n-무관) → URDF 로더+메시+셀렉터만.

브랜치 커밋: `b7fd734`(numba) · `08eb1fe`(이산 ZOH+노이즈) · `e0ae557`(노이즈 비활성 UX) · `90cc359`(DEMOS) · 외 DEVLOG

---

## 2026-06-19

### 🎯 오늘 한 것 — 리뷰 후속(머지 준비) (branch `numba-dynamics`)

`numba-dynamics` 브랜치 리뷰에서 나온 두 항목을 개선(머지 전 정지).

- **RNE drift 방지 — pytest 도입**: numpy `Dynamics` ↔ numba `dynamics_fast` 두 RNE 가
  같은 알고리즘을 각자 보유 → 한쪽만 고치면 조용히 어긋남. `tests/test_dynamics_parity.py`
  로 5함수(inverse/forward dynamics·mass_matrix·gravity/coriolis) + ready·home 자세를
  200개 랜덤 상태로 비교(`max|diff| < 1e-10`) 박제. **7 passed**. `pyproject` dev 에
  `pytest` 추가(설정만 있고 미설치였던 것 채움).
- **센서 노이즈 seed 노출 + 매 실행 변동**: `_zoh` 의 하드코딩 `default_rng(0)` →
  `RunRequest.noise_seed`(controller→sim 배선). 프론트는 **Run 때마다 seed 자동 +1**
  (실제 센서처럼 매 실행 다른 realization). seed 가 명시 정수라 추후 '고정/재현' 토글 가능.
- 검증: pytest 7 passed, seed 같으면 재현·다르면 다른 패턴 확인, tsc/eslint 0, ruff(신규파일) clean.

### 🤝 힘 제어 Step A — task-space 힘 제어 (MR §11.5, backend+데모)

교재(`docs/Modern_Robotics.pdf` §11.5–11.6) 정독 후, "힘 제어(§11.5) → 하이브리드(§11.6)"
순서로 쌓기로. Step A = 순수 힘 제어부터(backend+데모만, 프론트/스키마는 Step B).

- `sim._run_force` (추가만, 기존 러너 무수정): `τ = g̃ + Jbᵀ(Fd + Kfp·Fe + Kfi∫Fe − Kdamp·V)`
  (식 11.54). 디스패처에 `force` 분기 + `force_task` 파라미터.
- **환경 = 컴플라이언트 평면 벽**(penalty `f = K_env·δ + B_env·δ̇`). 교재 §11.6 은 강체구속
  `A(θ)V=0` 가정이나 수치불안정 → §11.7 이 밝히듯 강성 큰 스프링-댐퍼로 근사. 이 반력이
  곧 힘센서 측정값 F_meas 이자 plant 의 Ftip(폐루프 힘 피드백). 적분은 **접촉 중에만**(windup 방지).
- **`Ftip` 부호 규약 확정**: 이 코드 = 'EE 가 환경에 가하는 렌치'(MR 표준). 환경반력을 부호
  그대로 넣어 정피드백(벽 633mm 관통)→ `Ftip = −F_env` 로 해결. (수치테스트로 규약 확인)
  · ⚠️ 같은 규약상 기존 `_run_torque` 외란 Ftip 은 주석("끝단 외력")과 방향 반대 의심 — 추후 확인 TODO.
- `demos/force_demo.py`: READY 에서 벽을 20N 으로 누름 → 3패널(힘추종/오차/접근·침투).
- **검증**: 정상상태 힘 **20.000N(오차 0%)**, 침투 **5.00mm = 이론 Fd/K_env 정확일치**, 정착 0.73s,
  발산 없음. 회귀(computed_torque sse 1.7e-9·impedance·pytest 7) 무손상.

### 🤝 힘 제어 Step B — 하이브리드 모션/힘 제어 (MR §11.6, backend+데모)

법선=힘, 접선=위치를 **투영행렬로 직교 분리**해 동시 제어 (교재 "칠판 선 긋기").

- `sim._run_hybrid` (추가만): `P = I − Aᵀ(AΛ⁻¹Aᵀ)⁻¹AΛ⁻¹`(식 11.60), `Λ⁻¹=Jb·M⁻¹·Jbᵀ`.
  `τ = Jbᵀ[ P·(Λ·a_motion) + (I−P)·F_force ] + c + g`(식 11.61). A=[0 0 0|nb] 법선 운동 구속.
  · (I−P) = 법선 → PI 힘 제어(+법선 감쇠 Kfd), P = 접선·자세 → **PID** 모션 제어.
  · 디스패처 `hybrid` 분기. Step A 접촉모델을 `_wall_contact` 헬퍼로 추출(공용).
- **삽질 2건(둘 다 교재 정합성 회복으로 해결):**
  · 접선오차가 Kp 무관 7mm 고정 → 동역학일관 투영(P=Λ⁻¹가중=비직교)이라 **모션 적분
    `Ki∫Xe`(식 11.61 항)** 없으면 Λ-커플링으로 정상오차 남음 → Kiv 추가.
  · 그래도 7mm → 모션 목표의 법선좌표를 시작점(벽 위)에 둬 거대 법선오차가 접선으로 누설.
    **목표를 벽면에 투영**(교재 가정 A·Vd=0) → **7mm→0.8mm**.
- `demos/hybrid_demo.py`: 벽 20N 누르며 +x 로 100mm 직선 → 3패널(힘유지/접선추종/force-vs-pos).
- **검증**: 이동 중 법선힘 평균 19.5N(목표 20)·유지, 접선 추종오차 0.8mm, 발산 없음.
  회귀(force StepA F_ss 20.0·computed_torque sse 1.7e-9·pytest 7) 무손상.

### 🤝 힘 제어 Step C — 프론트 UI + 3D 벽 (브라우저 통합)

힘/하이브리드 제어를 웹에서 직접 실행·시각화. backend→REST→프론트 전구간 배선.

- **백엔드**: `schemas.ForceTask`(fd/gap/k_env/b_env/normal/tangent/move_len/...) +
  `RunRequest.force_task`, `RunResponse`에 `force/force_des/tan_pos/tan_des/wall` 추가
  (`response_model`이 strip 하므로 필수). `controllers/simulate`: 힘/하이브리드면 **시작자세
  FK 로 벽 point 계산**(프론트는 fd/gap/move_len 만 보냄) + `wall`(point·normal) 응답.
- **프론트**: `CONTROLLERS`에 force/hybrid(게인 step 소수 지원) + `ForceTask` 타입.
  `ControlPanel` 벽·힘목표 섹션(Fd·간격·접선이동 슬라이더). `Plots`는 force 있으면 **힘 추종
  +(하이브리드)접선 추종** 패널로 전환. **`WallPlane`**(신규): 응답 wall 로 3D 반투명 평면 렌더
  (법선→quaternion). 힘/하이브리드는 **현재 자세에서 출발**(run 분기).
- **검증**: tsc/eslint 0, `pnpm build` OK. TestClient `POST /api/run`(hybrid, 중첩 force_task)
  → 200, F_ss=목표·접선 추종·wall 계산 정상. **라이브 브라우저 확인 완료**(힘 40N 수렴 OK).
- **사용성 보완(피드백 반영):**
  · 벽을 base 고정([0,0,1])이 아니라 **시작자세 EE 도구축(보는 방향)** 에 배치 — `tool_axis`/
    `tan_axis`(EE 로컬) → 컨트롤러가 `R0` 로 base normal/tangent 계산. READY 는 도구 z=아래라
    기존과 동일, EE 를 돌리면 벽이 그 방향으로 따라옴(실제 누름 직관과 일치).
  · Run 버튼 라벨 제어기별 분기(`현재자세에서 벽 누름`/`벽 누르며 선 긋기`) — 힘 제어가 티칭이
    아님을 명확히.

### 🤖 멀티로봇 Phase 1 — URDF → MR 파라미터 추출기 (검증 완료)

엔진은 이미 generic(Mlist/Glist/Slist 인자, numba n-무관) → 빠진 건 임의 URDF에서 그 값 추출.
`mr_urdf_loader`는 urdfpy(py3.12 `collections.Mapping` 제거로 깨짐) 의존이라 폐기 →
유지보수되는 **`yourdfpy`로 직접 추출기 작성**.

- `app/core/urdf_loader.py`: `load_mr_model(urdf_path, ee_link=None)` → `{Mlist, Glist, Slist,
  M_home, joint_names, joint_limits, n}`. 스크류축 `Sᵢ=(ωᵢ, −ωᵢ×qᵢ)`, COM 프레임 Mlist,
  공간관성 `G=blockdiag(I_com, m·I₃)`. 직렬 arm은 자동(문서순), 그리퍼 로봇은 ee_link로
  base→EE **경로만** 추출(분기 제외).
- **검증**: UR5 하드코딩값(ur5_model)과 비교 — Slist `9.8e-12` · Mlist `4.9e-12` · **Glist 정확 0**.
  panda(ee=panda_link8) **n=7**, iiwa14 **n=7** forward_dynamics 동작(엔진 n-무관 확인).
- 남은 통합: RobotModel 추상화(fk/jac/ik 일반화) → 레지스트리/`/api/robots` → sim·controllers
  파라미터화(전역 ur5 49곳) → 프론트 셀렉터. (`yourdfpy`는 dev-dep → 런타임 쓰려면 main으로)

### 🎤 발표자료 (docs/ 로컬, gitignore)

세미나용 슬라이드 제작 — **본편(데모/직관 12장) + 부록(전문 수식 8장)** HTML(단일 파일,
오프라인 MathJax + 인라인 SVG)과 JLT 템플릿 기반 PPTX(python-pptx, 디자인 도형 복제). Lie군·
스크류·exp/log·Rodrigues·SE(3)·Adjoint·RNE·제어법칙 + **ROS2/Isaac Sim API 매핑** 슬라이드.
`docs/`로 이동(gitignore) — 교재 PDF처럼 로컬 보관.

### ✅ 오늘 커밋 (branch)
- `test:` RNE 동치성(pytest 도입) + 노이즈 seed 매 실행 변동
- `feat:` 힘 제어 Step A — task-space 힘 제어(§11.5) + 컴플라이언트 벽 + 데모
- `feat:` 힘 제어 Step B — 하이브리드 모션/힘(§11.6, 투영 P) + 데모
- `feat:` 힘 제어 Step C — 프론트 UI(force/hybrid·Fd·접선) + 3D 벽
- `feat:` 힘 제어 벽=EE 도구축 방향 + Run 라벨 제어기별
- `feat:` 멀티로봇 Phase 1 — URDF→MR 파라미터 추출기(yourdfpy, UR5 검증)
- `feat:` 멀티로봇 Phase 2 — RobotModel(기구학 일반화, ur5_model 재현·URDF IK 검증)
- `feat:` 발표 슬라이드 React 라우트(/slides, Tailwind+shadcn+KaTeX)
- (이 커밋) `feat:` 멀티로봇 Phase 3 — 레지스트리 + /api/robots (ur5/iiwa14/panda)

---

## 2026-06-22

### 🤖 멀티로봇 Phase 4 — sim·controllers RobotModel 파라미터화 (branch `multirobot`)

엔진은 이미 generic 이었고(Mlist/Glist/Slist 인자·numba n-무관), 빠진 '마지막 1마일'인
실제 시뮬 경로의 UR5 하드코딩을 제거했다. 이제 `robot_id` 로 등록된 어떤 로봇이든 같은
엔진으로 시뮬된다.

- **`sim.py`**: 전역 `N`·`MODEL` 제거 → `run_simulation(..., robot=None)` 으로 RobotModel
  주입. 모든 `_run_*`·헬퍼(`_build_ref`/`_make_ref`/`_wall_contact`/`_blowup_event`/`_empty_result`)
  가 `robot.fk/jacobian_body/dls_inv/n/dyn_args/gravity` 를 쓴다. 각 러너 상단에 `N=robot.n`
  로컬 바인딩(클로저가 캡처) → 본문 거의 무수정. `robot=None` 이면 레지스트리 UR5 폴백
  (데모/하위호환). `warmup()` 도 레지스트리 UR5 기준.
- **`controllers/simulate`·`kinematics`**: `registry.get_robot(req.robot_id)` 해석(미등록 404),
  `robot.ready/ik/fk/manipulability` 로 일반화. **`realism.build_models(dyn_args, ...)`** 시그니처
  변경으로 `sim.MODEL` 의존 제거.
- **`robot_registry`**: UR5 는 검증된 `ur5_model` 상수로 즉시 빌드(URDF 추출과 1e-11 일치·
  네트워크 불필요·기존 수치 비트일치), iiwa/panda 는 지연 URDF. **`schemas`** 3종(Run/IK/FK)에
  `robot_id="ur5"` 추가(프론트 미전송 시 UR5 기본 → 완전 하위호환).
- **검증(TestClient)**: UR5 회귀 무손상 — computed_torque sse **2.4e-9** · force **F_ss 20.000N** ·
  σ_min **0.2247**(READY) · resolved_rate 6.1e-7 · impedance 1.2e-3. **panda 7-DOF
  computed_torque sse 1.8e-9 정상 시뮬**(같은 엔진 n-무관 실증). 미등록 robot_id → 404.
  parity pytest **7 passed**.
- 남은 것: **프론트 로봇 셀렉터 + 로봇별 메시 서빙**(현재 프론트는 robot_id 미전송 → ur5 동작),
  이후 URDF 업로드 · 7-DOF 널공간 제어.

---

## 📋 다음 할 것 (TODO)

- [~] **멀티로봇** — ⭐엔진 generic(numba n-무관). [x] **Phase 1: URDF 추출기**(`urdf_loader.py`,
      yourdfpy, UR5 검증·panda/iiwa 7-DOF) ✅ 2026-06-19. mr_urdf_loader는 urdfpy(py3.12 깨짐)라 폐기.
      [x] **Phase 2 RobotModel**(`robot_model.py`, fk/jac/ik/manip/dls 일반화) ✅ 2026-06-19
      — UR5 상수 모델이 ur5_model 함수와 diff 0, URDF 로드 panda/iiwa IK 왕복 OK.
      [x] **Phase 3 레지스트리 + `/api/robots`**(`robot_registry.py`·`routers/robots.py`) ✅ 2026-06-19
      — ur5/iiwa14/panda 지연로딩·캐시, 목록+상세(관절·한계·ready TCP). yourdfpy→main dep.
      [x] **Phase 4 sim·controllers 파라미터화**(`robot_id`·RobotModel 주입, panda 7-DOF
      검증) ✅ 2026-06-22 → [ ] 프론트 셀렉터 + 로봇별 메시
      → [ ] URDF 업로드. (7-DOF 널공간 제어도)
- [x] **속도제어**(관절 θ̇_d / 직교 resolved-rate `J†`) ✅ 2026-06-18
      └ **순수 토크 입력 모드**(§11.4)는 τ 입력 스키마 필요 → 보류
- [x] **어드미턴스**(§11.7.2) ✅ 2026-06-18
- [x] **task-space 궤적**(직교 직선, §9) + 비특이 ready 자세 ✅ 2026-06-18
- [x] 특이점: resolved-rate DLS `J†` + manipulability(σ_min) 라이브 표시 ✅
- [x] **현실모델 — 마찰·토크포화** ✅ 2026-06-18 (└ 센서노이즈는 보류)
- [x] **동역학 고속화 (Numba JIT)** ✅ (branch `numba-dynamics`) — `dynamics_fast.py` @njit RNE,
      numpy 와 비트일치, fd 3.5ms→254µs(~14x), 시뮬 2-3.5s→~0.3s(~7x). 부팅 warmup.
- [x] **이산 ZOH 제어 + 센서노이즈** ✅ (branch) — `control_rate`(0=연속/>0=이산), 하이브리드.
      제어율 낮추면 불안정 재현, 노이즈 chatter. numba 덕에 1kHz ~4s 현실화.
- [~] **힘제어** — [x] Step A: task-space 힘 제어(§11.5, 식 11.54) + 컴플라이언트 벽 + 데모 ✅ 2026-06-19
      [x] Step B: 하이브리드 모션/힘(§11.6, 투영 P 식 11.60/11.61) + 데모 ✅ 2026-06-19
      [x] Step C: 프론트 UI(force/hybrid·Fd·접선 슬라이더, 힘 플롯) + 3D 벽 평면 ✅ 2026-06-19
      → [ ] 라이브 브라우저 스크린샷 검증(선택)
- [ ] **`numba-dynamics` 브랜치 → develop 머지** (parity pytest 통과 + 노이즈 seed 정리 완료 → 머지 가능)
- [ ] (선택) fd 추가 최적화: 해석적 SE(3) 역행렬 → 이산 더 빠르게(2kHz 기본 가능)
- [ ] **redundancy/널공간** 제어(7-DOF, `(I−J†J)θ̇₀`) — 멀티로봇 후
- [ ] **Docker** 배포 + README 재작성(웹 기준)

### ⚠️ 주의/메모
- 브랜치 `develop`, **push 미실시**(직접)
- 실행: 백엔드 `cd backend && uv run main.py`(:8500), 프론트 `cd frontend && pnpm dev`(:5174)
- `Pose` 등 **DB 스키마 변경 시 `backend/robot.db` 삭제 후 재시작**(create_all 은 ALTER 안 함)
- **UR5 zero(home) 자세가 특이점** → 시작 시 σ_min=0 경고는 정상
- 런타임 `robot.db`·`outputs/`·`*.npz`·`backend/app/meshes/`·`node_modules`·`dist` 는 gitignore
