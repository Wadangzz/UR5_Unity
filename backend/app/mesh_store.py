"""업로드된 메시(STL/OBJ) 인메모리 레지스트리 — 하이브리드 컨투어용 임의 표면.

프론트가 파일을 올리면 trimesh 로 로드·법선정리해 id 로 보관한다. /api/run 은
mesh_id + 배치 transform(pos/quat/scale)을 보내고, 컨트롤러가 그 transform 을
적용해 표면모델로 쓴다. 인메모리라 재시작 시 사라진다(데모용). 큰 메시는 단순화.
"""
import io
from itertools import count

import trimesh

_MESHES: dict[str, trimesh.Trimesh] = {}
_ids = count(1)
_MAX_FACES = 5000                                # 상한(초과 시 단순화)


def store(data: bytes, filename: str) -> dict:
    """업로드 바이트 → trimesh 로드·저장. → {mesh_id, n_faces, extents}."""
    ext = filename.rsplit(".", 1)[-1].lower()
    if ext not in ("stl", "obj"):
        raise ValueError("STL 또는 OBJ 만 지원합니다")
    mesh = trimesh.load(io.BytesIO(data), file_type=ext, force="mesh")
    if mesh.is_empty or len(mesh.faces) == 0:
        raise ValueError("메시를 읽지 못했습니다")
    if len(mesh.faces) > _MAX_FACES:             # 너무 크면 단순화(쿼리 속도)
        try:                                     # fast_simplification 의존
            mesh = mesh.simplify_quadric_decimation(_MAX_FACES)
        except Exception:                        # 없으면 원본 유지(업로드 성공)
            pass
    mesh.fix_normals()                           # 일관 외향 법선(보간 정확)
    mesh_id = f"m{next(_ids)}"
    _MESHES[mesh_id] = mesh
    return {"mesh_id": mesh_id, "n_faces": int(len(mesh.faces)),
            "extents": mesh.extents.tolist()}


def get(mesh_id: str) -> trimesh.Trimesh:
    """id → trimesh (없으면 KeyError)."""
    return _MESHES[mesh_id]
