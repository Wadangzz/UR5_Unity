"""사용 가능한 로봇 레지스트리 — id → RobotModel (지연 로딩·캐시).

목록(list_robots)은 메타데이터만(로드 안 함), get_robot(id) 첫 호출 시 RobotModel 을
빌드해 캐시한다. UR5 는 검증된 하드코딩 MR 파라미터(ur5_model)로 즉시 빌드한다
(URDF 추출값과 1e-11 일치·네트워크 불필요·기존 수치와 비트일치). 나머지는 첫 사용 시
robot_descriptions 에서 URDF/메시를 내려받아 추출하므로 그 로봇의 첫 요청은 느릴 수 있음.
"""
import importlib
from functools import cache

from app.core.robot_model import RobotModel

# urdf-loader/메시와 맞춘 UR5 관절 이름 (cfg 적용 순서)
_UR5_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def _build_ur5():
    """검증된 ur5_model 상수로 UR5 빌드 (URDF 추출과 1e-11 일치, 네트워크 불필요)."""
    from app.core import ur5_model as ur5
    return RobotModel(ur5.MLIST, ur5.GLIST, ur5.SLIST, ur5.M_HOME,
                      name="UR5", gravity=ur5.GRAVITY,
                      joint_names=_UR5_JOINTS, ready=ur5.READY)


def _build_urdf(name, mod, ee, ready):
    """robot_descriptions 모듈에서 URDF 를 받아 RobotModel 로 빌드하는 빌더(지연)."""
    def build():
        desc = importlib.import_module(f"robot_descriptions.{mod}")
        return RobotModel.from_urdf(desc.URDF_PATH, ee_link=ee,
                                    name=name, ready=ready)
    return build


# id: (표시이름, dof, 빌더)
_SPECS = {
    "ur5": ("UR5", 6, _build_ur5),
    "iiwa14": ("KUKA iiwa14", 7, _build_urdf(
        "KUKA iiwa14", "iiwa14_description", None,
        [0.0, 0.5, 0.0, -1.2, 0.0, 1.0, 0.0])),
    "panda": ("Franka Panda", 7, _build_urdf(
        "Franka Panda", "panda_description", "panda_link8",
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])),
}


def list_robots():
    """로드 없이 메타데이터 목록."""
    return [{"id": k, "name": v[0], "dof": v[1]} for k, v in _SPECS.items()]


@cache
def get_robot(robot_id: str) -> RobotModel:
    """id → RobotModel (지연 로딩·캐시). 미등록이면 KeyError."""
    return _SPECS[robot_id][2]()
