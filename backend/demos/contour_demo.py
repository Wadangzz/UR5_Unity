"""
하이브리드 컨투어 추종 데모 (MR §11.6 곡면 확장) — "곡면을 일정 힘으로 쓸기".

평면 한 획(hybrid_demo)을 곡면 위 경로 추종으로 일반화한다. 원기둥 표면을 따라
호(arc)를 그으면서 내내 법선 힘을 Fd 로 유지한다 (폴리싱/디버링/용접의 핵심 동작).

MR 충실성: §11.6 의 구속은 A(θ) (위치종속)라, 곡면은 "접촉점 법선 n(p)가 매 스텝
달라질 뿐" 제어식(투영 P, 식 11.60/11.61)은 그대로다. 표면 모델만 평면 → 원기둥으로
바꾼다 (법선 = 축에서의 반경방향). 공구 자세는 누름축(z)을 −n(p) 에 정렬해 곡면을
따라 기울인다(Level 2 — 평면이면 자동으로 고정).

실행:  uv run python demos/contour_demo.py   (+ outputs/contour_response.png)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from app import sim
from app.core import ur5_model as ur5
from app.core.paths import out

np.set_printoptions(precision=4, suppress=True)

FD = 20.0                                    # 목표 누름힘 (N)
GAP, RAD = 0.03, 0.10                        # 시작 간격 / 원기둥 반경 (m)
SWEEP = np.radians([0, 12, 24, 36, 48])      # 티칭 경로: 표면 위 호 (5점, ~48°)

# --- 시작 자세 기준 원기둥 배치 ---
T0 = ur5.fk(ur5.READY)
R0, p0 = T0[:3, :3], T0[:3, 3]
push = R0 @ np.array([0.0, 0.0, 1.0])        # 공구축(시작자세에서 누르는 방향)
push /= np.linalg.norm(push)
axis = R0 @ np.array([1.0, 0.0, 0.0])        # 원기둥 축 (push 와 직교)
axis /= np.linalg.norm(axis)
center = p0 + push * (GAP + RAD)             # 축 중심선 (EE 앞 gap+R)
radial0 = -push                              # 시작 접촉점의 외향 법선
wdir = np.cross(axis, radial0)               # 표면 위 스윕 방향(접선)
wdir /= np.linalg.norm(wdir)

# --- 티칭 경로 waypoint: 표면 위 호를 IK 로 (자세 R0 유지) ---
WP, seed = [], ur5.READY.copy()
for ang in SWEEP:
    p_t = center + RAD * (np.cos(ang) * radial0 + np.sin(ang) * wdir)
    T = np.eye(4); T[:3, :3] = R0; T[:3, 3] = p_t
    th, ok = ur5.ik(T, seed)
    WP.append(th); seed = th

res = sim.run_simulation(
    waypoints=WP,
    controller="hybrid",
    gains={"kp": 100.0, "kd": 20.0, "kiv": 60.0,
           "kfp": 0.5, "kfi": 4.0, "kfd": 10.0},
    force_task={
        "shape": "cylinder",
        "center": center.tolist(),
        "axis": axis.tolist(),
        "radius": RAD,
        "fd": FD,
        "k_env": 4000.0,
        "b_env": 40.0,
        "move_start": 0.6,
        "move_time": 0.9,                    # 구간당 시간
    },
    hz=60,
)

t = np.array(res["t"])
force = np.array(res["force"])
tcp = np.array(res["tcp"])
des = np.array(res["tcp_des"])

# 표면 단면 좌표(중심 기준 radial0–wdir 평면) 로 투영
def plane2d(P):
    d = P - center
    return d @ radial0, d @ wdir

ax_act, ay_act = plane2d(tcp)
ax_des, ay_des = plane2d(des)
moving = t >= 0.6
print(f"목표 힘 Fd = {FD:.1f} N,  원기둥 R={RAD*100:.0f}cm,  스윕 {np.degrees(SWEEP[-1]):.0f}°")
print(f"발산 = {res['diverged']}")
print(f"쓸기 중 법선 힘: 평균 {force[moving].mean():.2f} N  "
      f"[{force[moving].min():.2f}, {force[moving].max():.2f}]  (목표 {FD})")
tan_err = np.mean([np.linalg.norm((p - d) - ((p - d) @ (n / np.linalg.norm(n))) * (n / np.linalg.norm(n)))
                   for p, d, n in zip(tcp, des, tcp - center)])
print(f"접선 추종오차(평균) = {tan_err*1000:.2f} mm")

# --- 플롯 ---
fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))

# 1) 법선 힘 — 곡면 쓸기 내내 Fd 유지 (법선이 회전하는데도)
ax[0].plot(t, force, color="tab:blue", lw=2, label="normal force")
ax[0].axhline(FD, ls="--", color="gray", label=f"target Fd = {FD} N")
ax[0].axvspan(0.6, t[-1], color="tab:orange", alpha=0.10, label="contour sweep")
ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("force [N]")
ax[0].set_title("Normal force held while sweeping a curve")
ax[0].legend(); ax[0].grid(alpha=0.3)

# 2) 원기둥 단면 위 경로 추종 (표면 = 원, 목표/실제 호)
th_c = np.linspace(0, 2 * np.pi, 200)
ax[1].plot(RAD * np.cos(th_c) * 100, RAD * np.sin(th_c) * 100,
           color="lightgray", lw=1, label="cylinder surface")
ax[1].plot(ax_des * 100, ay_des * 100, ls="--", color="gray", label="target path")
ax[1].plot(ax_act * 100, ay_act * 100, color="tab:green", lw=2, label="actual EE")
ax[1].set_aspect("equal"); ax[1].set_xlabel("radial [cm]"); ax[1].set_ylabel("tangential [cm]")
ax[1].set_title("Contour on cylinder cross-section"); ax[1].legend(); ax[1].grid(alpha=0.3)

# 3) 스윕 위치(접선 호길이) vs 법선 힘 — 일정-힘 컨투어 라인
arc = ay_act * 100
ax[2].plot(arc, force, color="tab:purple")
ax[2].axhline(FD, ls="--", color="gray")
ax[2].set_xlabel("sweep (tangential) [cm]"); ax[2].set_ylabel("normal force [N]")
ax[2].set_title("Force vs sweep  (constant-force contour)"); ax[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(out("contour_response.png"), dpi=95)
print("\n[저장] outputs/contour_response.png")
