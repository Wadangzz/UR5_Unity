# UR5_Unity
<p align="center">
  <img src="tutorial.gif" width="80%">
  <img src="tutorial_2.gif" width="80%">


</p>  

**Numerical IK로 생성한 UR5 로봇의 Trajectory를  Unity에서 실시간 시각화**하는 프로젝트입니다.  
Python에서 생성한 trajectory를 JSON으로 전송하면, Unity에서 해당 데이터를 수신하여 UR5 3D 모델이 움직입니다.   
[**Youtube link**](https://youtu.be/c5draOBLXvA)

---

## 🛠 주요 기능

- UR5 3D 모델 기반 Unity 시뮬레이션
- Python에서 수치 역기구학으로 생성한 trajectory(JSON) 송신
- TCP Socket 기반 실시간 통신
- 수신된 데이터를 기반으로 Unity 내 로봇 동작
- **Screw theory 기반 동역학 엔진** (RNE 역/순동역학, 질량·코리올리·중력 항)
- **제어 데모** — 중력보상 · PD · Computed Torque 궤적추종 (Unity 없이 순수 Python + matplotlib)

---

## 🛠️ 실행 환경

* Python 3.12+
* `numpy`, `scipy`, `matplotlib`, `pyside6`
* 패키지 관리: [`uv`](https://docs.astral.sh/uv/)

```bash
# pythonscript 폴더에서
uv sync
```
---

## 🔧 시스템 구조

```text
[Python Script] ──> (Socket, JSON) ──> [Unity]
```
---

## 🚀 실행 방법

- UR5_RUN 폴더 내에 Unity Bulid UR5 실행   
- python 실행 전 까지 TCP 수신 대기   
- pythonControl 폴더 내 UR5_QT.py 실행   
- IP, PORT(5000) 입력하여 Unity 연결(localhost = 127.0.0.1)   
- pose, jointangle 슬라이더 조정하여 자세 저장   
- 실행 시 순서대로 Numerical IK 계산 후 trajectory 생성   
- UR5 오브젝트 실시간 동작 확인   

---

## 🧮 동역학 · 제어 데모 (Python, Unity 불필요)

Modern Robotics(screw theory) 기반으로 구현한 동역학 엔진과 제어 데모.
순동역학을 plant 로 삼아 Python 안에서 제어 루프를 닫는다 (`scipy.solve_ivp` 적분).

```bash
# pythonscript 폴더에서
python gravity_demo.py      # 자세별 중력 토크 g(θ) 계산·물리검증
python control_demo.py      # 무제어/중력보상/PD+중력보상 비교 + 궤적 저장
python animate.py           # 위 결과 3D 애니메이션 (ur5_control.gif)
python tracking_demo.py     # Computed Torque vs PD 궤적추종 비교
python animate_tracking.py  # 추종 결과 3D 애니메이션 (tracking.gif)
```

- `MyRobotMath.py` — `SO3`/`SE3`/`Kinematics`/`Dynamics` (Lie 군·대수 구조로 분리)
- `ur5_model.py` — UR5 동역학 파라미터 (Mlist/Glist/Slist, SI 단위)
- 상세 설명: [`docs/Inverse_Dynamics.md`](pythonscript/docs/Inverse_Dynamics.md)

---

## 📌 수정 사항

- SQLite DB 추가해서 Qt GUI 종료해도 이전 저장 값 유지   
- euler angle 해석, 동작하는 부분 quaternion으로 변경( Gimbol Lock 회피 용이 )   
- Reset 버튼 추가해서 프로그램 리셋 기능 구현   
- pyinstaller로 최종 exe 파일 Build 및 Release   

---

## 🔗 License

MIT License. Free to use, modify, and learn from.

---
