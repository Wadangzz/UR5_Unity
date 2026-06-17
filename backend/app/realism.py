"""현실성 모델 빌더 — plant(진짜 로봇) ≠ controller(아는 모델).

이상적 시뮬(sim.py)은 plant=ctrl 이라 CT 가 거의 완벽(오차 1e-10)하다. 실제론
컨트롤러가 로봇을 정확히 모른다 — 이 '틈'을 두 노브로 준다:

  payload      : 끝단(link-6 원점)에 달린 미지 점질량(kg). plant 에만 반영 →
                 컨트롤러는 모르므로 중력보상이 부족 → PD 처짐, PID 적분이 회복.
  model_scale  : 컨트롤러가 아는 링크 질량 배율(1.0=정확, 0.8=20% 과소추정).

→ sim.run_simulation(waypoints, ..., plant=plant, ctrl=ctrl) 로 넘겨 쓴다.
"""
import numpy as np
from app.sim import MODEL


def build_models(payload=0.0, model_scale=1.0):
    """(plant, ctrl) 모델 쌍을 만든다. payload=0·scale=1 이면 둘 다 공칭(이상적)."""
    MLIST, GLIST, SLIST = MODEL

    plant_G = [np.array(g, dtype=float) for g in GLIST]
    if payload:                                   # 점질량 → 마지막 링크 병진관성에 추가
        for k in (3, 4, 5):
            plant_G[-1][k, k] += payload

    ctrl_G = [np.array(g, dtype=float) * model_scale for g in GLIST]

    return (MLIST, plant_G, SLIST), (MLIST, ctrl_G, SLIST)
