"""
힘 제어 데모 (MR §11.5) — 가상 벽을 목표 힘으로 누른다.

τ = g̃(θ) + Jbᵀ(Fd + Kfp·Fe + Kfi∫Fe − Kdamp·V)    (식 11.54)

환경은 컴플라이언트 평면 벽(penalty: f = K_env·δ + B_env·δ̇). READY 자세에서
끝단을 5cm 아래(−z)로 내려 벽에 접촉시킨 뒤, 법선 힘이 목표 Fd 로 수렴하는지 본다.
PI 힘 피드백의 적분항이 정상상태 힘 오차를 0 으로 만든다(접촉 중에만 적분 → windup 방지).

실행:  uv run python demos/force_demo.py   (+ outputs/force_response.png)
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np
import matplotlib.pyplot as plt
from app import sim
from app.core import ur5_model as ur5
from app.core.paths import out

np.set_printoptions(precision=4, suppress=True)

FD = 20.0                                   # 목표 누름힘 (N)
GAP = 0.05                                  # 시작 시 벽까지 거리 (m)

# 벽: READY 끝단 바로 아래 GAP 만큼 떨어진 수평면(외향 법선 +z). 끝단은 −z 로 누른다.
p0 = ur5.fk(ur5.READY)[:3, 3]
wall_point = p0 - np.array([0.0, 0.0, GAP])

res = sim.run_simulation(
    waypoints=[ur5.READY],
    controller="force",
    gains={"kfp": 0.5, "kfi": 4.0, "kdamp": 25.0},
    force_task={
        "normal": [0.0, 0.0, 1.0],
        "point": wall_point.tolist(),
        "fd": FD,
        "k_env": 4000.0,
        "b_env": 40.0,
        "t_end": 4.0,
    },
)

t = np.array(res["t"])
force = np.array(res["force"])
err = np.array(res["error"])
zee = np.array([p[2] for p in res["tcp"]])          # 끝단 z (벽 z = wall_point[2])

print(f"목표 힘 Fd = {FD:.1f} N,  벽 z = {wall_point[2]:.4f} m (시작 z = {p0[2]:.4f})")
print(f"발산 = {res['diverged']},  정착시간 = {res['settle_time']}")
print(f"정상상태 힘 = {force[-1]:.3f} N  (오차 {err[-1]:.3f} N = {100*err[-1]/FD:.2f}%)")
print(f"최종 침투깊이 ~ {1000*(wall_point[2]-zee[-1]):.2f} mm  (이론 Fd/K_env = "
      f"{1000*FD/4000.0:.2f} mm)")

# --- 플롯 ---
fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))

# 1) 측정 힘 → 목표 수렴
ax[0].plot(t, force, color="tab:blue", lw=2, label="measured normal force")
ax[0].axhline(FD, ls="--", color="gray", label=f"target Fd = {FD} N")
if res["settle_time"] is not None:
    ax[0].axvline(res["settle_time"], ls=":", color="tab:green",
                  label=f"settle {res['settle_time']:.2f}s")
ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("force [N]")
ax[0].set_title("Force tracking  F -> Fd"); ax[0].legend(); ax[0].grid(alpha=0.3)

# 2) 힘 오차 → 0
ax[1].plot(t, err, color="tab:red")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("|Fd - F| [N]")
ax[1].set_title("Force error -> 0  (PI integral)"); ax[1].grid(alpha=0.3)

# 3) 끝단 z 와 벽(접촉 기하)
ax[2].plot(t, 1000 * zee, color="tab:purple", label="EE z")
ax[2].axhline(1000 * wall_point[2], ls="--", color="k", label="wall surface")
ax[2].set_xlabel("time [s]"); ax[2].set_ylabel("z [mm]")
ax[2].set_title("Approach & penetration"); ax[2].legend(); ax[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig(out("force_response.png"), dpi=95)
print("\n[저장] outputs/force_response.png")
