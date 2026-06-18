"""로봇 정보 라우터."""
from fastapi import APIRouter
from app.core import ur5_model as ur5
from app.meshes_setup import urdf_url, PACKAGE_NAME

router = APIRouter(prefix="/api", tags=["robot"])

# urdf-loader 가 다루는 URDF 관절 이름 (cfg 적용 순서)
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


@router.get("/robot")
def get_robot():
    return {"name": "UR5", "n": ur5.N, "joint_names": JOINT_NAMES,
            "home": [0.0] * ur5.N, "home_tcp": ur5.M_HOME[:3, 3].tolist(),
            "ready": ur5.READY.tolist(),    # 비특이 운용 출발 자세
            "urdf_url": urdf_url(),
            # urdf-loader packages 매핑: package://<name>/... → /meshes/<name>/...
            "packages": {PACKAGE_NAME: f"/meshes/{PACKAGE_NAME}"}}
