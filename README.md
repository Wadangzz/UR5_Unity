⁹# UR5_Unity
<p align="center">
  <img src="UR5.gif" width="30%">
  <img src="UR5_2.gif" width="30%">
  <img src="UR5_3.gif" width="30%">
</p>

**UR5 로봇의 Joint Space Trajectory 데이터를 Unity에서 실시간 시각화**하기 위한 시스템입니다.  
Python에서 생성한 trajectory를 JSON 형식으로 전송하면, Unity가 해당 데이터를 수신하여 UR5 3D 모델을 애니메이션합니다.

---

## 🛠 주요 기능

- UR5 3D 모델 기반 Unity 시뮬레이션
- Python에서 생성된 Joint trajectory(JSON) 수신
- TCP Socket 기반 실시간 통신
- 수신된 데이터를 기반으로 Unity 내 로봇 동작

---

## 🛠️ 실행 환경

* Python 3.9
* `numpy`, `scipy`, `matplotlib`

```bash
pip install numpy scipy matplotlib
```
---

## 🔧 시스템 구조

```text
[Python Script] ──> (Socket, JSON) ──> [Unity (Listener)]
```
---

## 🚀 실행 방법
UR5_RUN 폴더 내에 Unity Bulid UR5 실행   
python 실행 전 까지 TCP listen 대기

pythonControl 폴더 내 ur5_joint.py 실행
x, y, z, Rx, Ry, Rz 입력하여 trajectory 생성 후
UR5 오브젝트 실시간 동작 확인

---

## 📌 주의사항
현재는 Joint Space 기반 trajectory만 지원됩니다.


Task Space trajectory 및 특이점 회피, 충돌 회피 
Joint angle Limit 설정 외 기타 예외 처리는 추후 업데이트 예정입니다.   
QT GUI 또는 Unity 캔버스 GUI 추가 예정

---

## 🔗 License

MIT License. Free to use, modify, and learn from.

---
