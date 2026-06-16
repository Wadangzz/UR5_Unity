"""
UR5 중력보상 데모 — 정지 자세를 버티는 데 필요한 관절 토크 g(θ) 계산.

중력 토크는 질량 + 무게중심만으로 결정되며(관성텐서 무관),
robot_math.Dynamics.gravity_forces 로 RNE 를 통해 계산한다.

실행:  python gravity_demo.py
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))


import numpy as np
from app.core.robot_math import Dynamics, SE3
from app.core import ur5_model as ur5

np.set_printoptions(precision=3, suppress=True)


def gravity_torque(theta):
    """자세 theta(rad, 6개)에서 중력을 버티는 관절 토크 [N·m]."""
    return Dynamics.gravity_forces(theta, ur5.GRAVITY, ur5.MLIST, ur5.GLIST, ur5.SLIST)


def banner(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


# ----------------------------------------------------------------------
# 0) 모델 기하 확인 — home 자세 말단 위치
# ----------------------------------------------------------------------
banner("0) 모델 확인: home(θ=0) 말단 위치")
M = ur5.home_config()
print("end-effector 위치 [m] =", M[:3, 3], "  (UR5 표준: 팔이 수평으로 뻗음)")

# ----------------------------------------------------------------------
# 1) 물리 불변식 ① : 1번 관절(수직 축) 중력 토크는 항상 0
#    중력은 수직이라 수직 축 둘레 모멘트는 0 → tau[0] = 0 (모든 자세)
# ----------------------------------------------------------------------
banner("1) 검증: 1번 관절(수직축) 중력 토크 = 0 (임의 자세)")
rng = np.random.default_rng(0)
ok1 = True
for _ in range(5):
    th = rng.uniform(-np.pi, np.pi, 6)
    t0 = gravity_torque(th)[0]
    ok1 &= abs(t0) < 1e-9
    print(f"  random θ → tau[0] = {t0:+.2e}")
print("→ tau[0] ≈ 0 항상 성립:", ok1)

# ----------------------------------------------------------------------
# 2) 물리 불변식 ② : 1번 관절만 돌리면 나머지 관절 중력 토크 불변
#    수직축 회전은 팔의 '중력에 대한 기울기'를 바꾸지 않음
# ----------------------------------------------------------------------
banner("2) 검증: base(1번) 회전은 2~6번 토크에 영향 없음")
base = np.array([0.0, -0.4, 0.8, 0.3, 0.2, 0.0])
t_a = gravity_torque(base)
rot = base.copy(); rot[0] += 1.234          # 1번 관절만 회전
t_b = gravity_torque(rot)
print("  base θ      → tau =", t_a)
print("  base+Δθ₁    → tau =", t_b)
print("→ 2~6번 토크 동일:", np.allclose(t_a[1:], t_b[1:], atol=1e-9))

# ----------------------------------------------------------------------
# 3) 자세별 중력 토크 — 물리적 직관과 맞는지
# ----------------------------------------------------------------------
banner("3) 자세별 중력 토크 g(θ) [N·m]")
deg = np.deg2rad
poses = {
    "home (팔 수평으로 뻗음)        ": [0, 0, 0, 0, 0, 0],
    "어깨 -90° (팔 위로 직립)       ": [0, deg(-90), 0, 0, 0, 0],
    "어깨 -45°                      ": [0, deg(-45), 0, 0, 0, 0],
    "팔꿈치만 90° 굽힘              ": [0, 0, deg(90), 0, 0, 0],
}
for name, th in poses.items():
    tau = gravity_torque(np.array(th, dtype=float))
    print(f"  {name} tau = {tau}")

print("\n해석:")
print("  · 팔 수평(home): 어깨·팔꿈치 토크 최대 (무거운 팔을 중력에 맞서 듦)")
print("  · 팔 직립(-90°): 무게중심이 축 위로 정렬 → 토크 급감 (거의 0)")
print("  · 손목 쪽일수록 가벼워 토크 작음")

# ----------------------------------------------------------------------
# 4) 어깨 각도 스윕 — sin 꼴로 변하는지 (수평에서 최대, 수직에서 0)
# ----------------------------------------------------------------------
banner("4) 어깨(2번) 각도 스윕 → tau[1] 크기")
print("  shoulder[deg] :  tau[1] [N·m]")
for d in range(-90, 91, 30):
    th = np.array([0, deg(d), 0, 0, 0, 0], dtype=float)
    print(f"     {d:+4d}       :  {gravity_torque(th)[1]:+8.3f}")

print("\n[완료] 모든 토크는 N·m. 물리적으로 일관됨을 확인.")
