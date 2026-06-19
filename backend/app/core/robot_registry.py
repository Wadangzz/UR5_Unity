"""사용 가능한 로봇 레지스트리 — id → RobotModel (URDF 추출, 지연 로딩·캐시).

목록(list_robots)은 메타데이터만(로드 안 함), get_robot(id) 첫 호출 시 robot_descriptions
에서 URDF 를 받아 RobotModel 로 빌드하고 캐시한다. (robot_descriptions 가 첫 사용 시
URDF/메시를 내려받으므로 그 로봇의 첫 요청은 느릴 수 있음)
"""
import importlib
from functools import cache

import numpy as np

from app.core.robot_model import RobotModel

# id: (표시이름, robot_descriptions 모듈, ee_link, dof, ready 자세[rad])
_SPECS = {
    "ur5": ("UR5", "ur5_description", None, 6,
            [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]),
    "iiwa14": ("KUKA iiwa14", "iiwa14_description", None, 7,
               [0.0, 0.5, 0.0, -1.2, 0.0, 1.0, 0.0]),
    "panda": ("Franka Panda", "panda_description", "panda_link8", 7,
              [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),
}


def list_robots():
    """로드 없이 메타데이터 목록."""
    return [{"id": k, "name": v[0], "dof": v[3]} for k, v in _SPECS.items()]


@cache
def get_robot(robot_id: str) -> RobotModel:
    """id → RobotModel (지연 로딩·캐시). 미등록이면 KeyError."""
    name, mod, ee, _dof, ready = _SPECS[robot_id]
    desc = importlib.import_module(f"robot_descriptions.{mod}")
    return RobotModel.from_urdf(desc.URDF_PATH, ee_link=ee, name=name, ready=ready)
