"""
하이브리드 모션/힘 제어 데모 (MR §11.6) — "칠판에 선 긋기".

벽을 일정 힘(법선)으로 누르면서, 동시에 벽면을 따라 옆으로 직선(접선) 이동한다.
투영행렬 P (식 11.60) 가 렌치공간을 둘로 쪼갠다:
    τ = Jbᵀ[ P·(Λ·a_motion) + (I−P)·F_force ] + c + g        (식 11.61)
  · (I−P) 부분공간 = 법선 → PI 힘 제어 (목표 Fd 로 누름)
  · P 부분공간     = 접선·자세 → PID 모션 제어 (직선 추종)

핵심: 옆으로 움직이는 내내 법선 힘이 Fd 로 유지된다(두 제어가 직교 분리).

실행:  uv run python demos/hybrid_demo.py   (+ outputs/hybrid_response.png)
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
GAP = 0.05                                   # 시작 시 벽까지 거리 (m)
MOVE_LEN = 0.10                              # 접선 이동거리 (m)
MOVE_START, MOVE_TIME = 0.8, 1.5            # 접선 이동 시작/소요 (s)

p0 = ur5.fk(ur5.READY)[:3, 3]
wall_point = p0 - np.array([0.0, 0.0, GAP])  # 수평 벽(법선 +z), 끝단은 −z 로 누름

res = sim.run_simulation(
    waypoints=[ur5.READY],
    controller="hybrid",
    gains={"kp": 100.0, "kd": 20.0, "kiv": 60.0,
           "kfp": 0.5, "kfi": 4.0, "kfd": 10.0},
    force_task={
        "normal": [0.0, 0.0, 1.0],
        "point": wall_point.tolist(),
        "fd": FD,
        "k_env": 4000.0,
        "b_env": 40.0,
        "tangent": [1.0, 0.0, 0.0],          # +x 로 직선
        "move_len": MOVE_LEN,
        "move_start": MOVE_START,
        "move_time": MOVE_TIME,
        "t_end": 4.0,
    },
)

t = np.array(res["t"])
force = np.array(res["force"])
tan_act = np.array(res["tan_pos"])
tan_des = np.array(res["tan_des"])

# 접선 이동 구간에서의 힘 유지 통계
moving = (t >= MOVE_START) & (t <= MOVE_START + MOVE_TIME)
print(f"목표 힘 Fd = {FD:.1f} N,  접선 이동 {1000*MOVE_LEN:.0f}mm (+x)")
print(f"발산 = {res['diverged']}")
print(f"이동 중 법선 힘: 평균 {force[moving].mean():.2f} N  "
      f"[{force[moving].min():.2f}, {force[moving].max():.2f}]  (목표 {FD})")
print(f"최종: 힘 {force[-1]:.2f} N,  접선 {1000*tan_act[-1]:.2f}mm "
      f"(목표 {1000*tan_des[-1]:.1f}mm, 오차 {1000*abs(tan_act[-1]-tan_des[-1]):.2f}mm)")

# --- 플롯 ---
fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))

# 1) 법선 힘 — 이동 중에도 Fd 유지
ax[0].plot(t, force, color="tab:blue", lw=2, label="normal force")
ax[0].axhline(FD, ls="--", color="gray", label=f"target Fd = {FD} N")
ax[0].axvspan(MOVE_START, MOVE_START + MOVE_TIME, color="tab:orange", alpha=0.12,
              label="tangential move")
ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("force [N]")
ax[0].set_title("Normal force held during motion"); ax[0].legend(); ax[0].grid(alpha=0.3)

# 2) 접선 위치 추종
ax[1].plot(t, 1000 * tan_des, ls="--", color="gray", label="target")
ax[1].plot(t, 1000 * tan_act, color="tab:green", lw=2, label="actual")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("tangential pos [mm]")
ax[1].set_title("Tangential motion tracking"); ax[1].legend(); ax[1].grid(alpha=0.3)

# 3) 평면 궤적(접선 위치 vs 법선 힘) — 누르며 긋는 "선"
ax[2].plot(1000 * tan_act, force, color="tab:purple")
ax[2].axhline(FD, ls="--", color="gray")
ax[2].set_xlabel("tangential pos [mm]"); ax[2].set_ylabel("normal force [N]")
ax[2].set_title("Force vs position  (constant-force line)"); ax[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(out("hybrid_response.png"), dpi=95)
print("\n[저장] outputs/hybrid_response.png")
