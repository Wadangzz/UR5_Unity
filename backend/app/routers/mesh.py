"""메시 업로드 라우터 — 하이브리드 컨투어용 임의 표면(STL/OBJ)."""
from fastapi import APIRouter, HTTPException, UploadFile

from app import mesh_store

router = APIRouter(prefix="/api", tags=["mesh"])


@router.post("/mesh")
async def upload_mesh(file: UploadFile):
    """STL/OBJ 업로드 → trimesh 로드·저장. → {mesh_id, n_faces, extents}."""
    data = await file.read()
    try:
        return mesh_store.store(data, file.filename or "mesh.stl")
    except Exception as e:
        raise HTTPException(400, f"메시 로드 실패: {e}") from None
