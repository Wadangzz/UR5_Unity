"""
URDF + 메시 정적 서빙 준비.

robot_descriptions 캐시(~/.cache/robot_descriptions/...)에 있는 UR5 URDF 와
그것이 참조하는 visual/collision 메시(.dae/.stl)를 `app/meshes/` 아래로 복사한다.
package:// 상대 구조를 그대로 유지하므로 브라우저 urdf-loader 가 package 매핑만으로
메시를 찾을 수 있다.

  package://example-robot-data/robots/ur_description/meshes/ur5/visual/base.dae
        └ PACKAGE(example-robot-data) ─┴──────────── 상대경로 ────────────┘

→ /meshes/example-robot-data/robots/ur_description/meshes/ur5/visual/base.dae

`app/meshes/` 는 .gitignore 되며, 서버 기동 시(import 시) 없으면 자동 생성된다.
Docker 에서도 robot_descriptions 가 첫 import 때 캐시를 내려받으므로 그대로 동작.
"""

import os
import re
import shutil

MESHES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# urdf-loader 에 넘길 package 매핑 (package 이름 → /meshes 하위 경로)
PACKAGE_NAME = "example-robot-data"

# /meshes 기준 URDF 상대 경로 (prepare_meshes 가 채워줌)
_URDF_REL = None

_MESH_RE = re.compile(r'package://([^/]+)/(.+?\.(?:dae|stl|obj|DAE|STL|OBJ))')


def prepare_meshes():
    """캐시의 UR5 URDF + 참조 메시를 app/meshes/ 로 복사. URDF 의 /meshes 상대경로 반환."""
    global _URDF_REL
    from robot_descriptions import ur5_description as desc

    urdf_src = desc.URDF_PATH                       # .../example-robot-data/robots/ur_description/urdf/ur5_robot.urdf
    repo_root = desc.REPOSITORY_PATH                # .../example-robot-data
    pkg_parent = os.path.dirname(repo_root)         # .../robot_descriptions (PACKAGE_NAME 의 부모)

    with open(urdf_src, "r", encoding="utf-8") as f:
        urdf_text = f.read()

    # URDF 자신 복사 (repo_root 부모 기준 상대경로 유지)
    urdf_rel = os.path.relpath(urdf_src, pkg_parent).replace("\\", "/")
    _copy(urdf_src, os.path.join(MESHES_DIR, urdf_rel))

    # URDF 가 참조하는 모든 메시 복사
    copied = 0
    for pkg, rel in _MESH_RE.findall(urdf_text):
        src = os.path.join(pkg_parent, pkg, *rel.split("/"))
        if os.path.exists(src):
            _copy(src, os.path.join(MESHES_DIR, pkg, *rel.split("/")))
            copied += 1

    _URDF_REL = urdf_rel
    return urdf_rel, copied


def _copy(src, dst):
    if os.path.exists(dst):
        return
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def urdf_url():
    """프론트가 로드할 URDF 의 정적 URL (/meshes/...)."""
    if _URDF_REL is None:
        prepare_meshes()
    return "/meshes/" + _URDF_REL
