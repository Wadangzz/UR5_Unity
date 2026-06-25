"""URDF + 메시 정적 서빙 준비 (멀티로봇).

robot_descriptions 캐시(~/.cache/robot_descriptions/...)에 있는 각 로봇의 URDF 와 그것이
참조하는 visual/collision 메시(.dae/.stl/.obj)를 `app/meshes/` 아래로 복사한다.
package:// 상대 구조를 그대로 유지하므로 브라우저 urdf-loader 가 package 매핑만으로
메시를 찾을 수 있다.

  package://example-robot-data/robots/ur_description/meshes/ur5/visual/base.dae
        └ PACKAGE(example-robot-data) ─┴──────────── 상대경로 ────────────┘

→ /meshes/example-robot-data/robots/ur_description/meshes/ur5/visual/base.dae

세 로봇 모두 pkg_parent=dirname(REPOSITORY_PATH) 기준으로 메시가 해석된다
(ur5/panda=example-robot-data·.dae, iiwa14=drake·.obj). 로봇별로 첫 요청 시 지연 준비(복사)
하고 캐시한다. `app/meshes/` 는 .gitignore 되며, Docker 에서도 robot_descriptions 가
첫 import 때 캐시를 내려받으므로 그대로 동작.
"""

import importlib
import os
import re
import shutil

MESHES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# robot_id → robot_descriptions 모듈 이름 (레지스트리와 동일 로봇 집합)
_ROBOT_DESC = {
    "ur5": "ur5_description",
    "iiwa14": "iiwa14_description",
    "panda": "panda_description",
    "omx": "open_manipulator_x_description",
}

_MESH_RE = re.compile(r'package://([^/]+)/(.+?\.(?:dae|stl|obj|DAE|STL|OBJ))')

# robot_id → {"urdf_url", "packages"} (지연 준비·캐시)
_ASSETS: dict[str, dict] = {}


def _copy(src, dst):
    if os.path.exists(dst):
        return
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def _prepare(desc_modname):
    """description 모듈의 URDF + 참조 메시를 app/meshes/ 로 복사. → (urdf_url, packages).

    package 이름은 URDF 내 참조에서 모아 매핑(urdf-loader.packages)으로 만든다.
    """
    desc = importlib.import_module(f"robot_descriptions.{desc_modname}")
    urdf_src = desc.URDF_PATH
    pkg_parent = os.path.dirname(desc.REPOSITORY_PATH)   # package 이름의 부모 디렉터리

    with open(urdf_src, encoding="utf-8") as f:
        urdf_text = f.read()

    # URDF 자신 복사 (pkg_parent 기준 상대경로 유지)
    urdf_rel = os.path.relpath(urdf_src, pkg_parent).replace("\\", "/")
    _copy(urdf_src, os.path.join(MESHES_DIR, *urdf_rel.split("/")))

    # URDF 가 참조하는 모든 메시 복사 + package 이름 수집
    # 메시 패키지 dir 위치는 repo마다 다르다: REPOSITORY_PATH 의 부모에 평평하게 있거나
    # (panda/iiwa14), repo 안에 중첩돼 있다(open_manipulator/open_manipulator_description).
    # 두 후보 부모를 순서대로 시도해 실제 파일이 있는 곳에서 복사한다.
    parents = (pkg_parent, desc.REPOSITORY_PATH)
    pkgs = set()
    for pkg, rel in _MESH_RE.findall(urdf_text):
        for parent in parents:
            src = os.path.join(parent, pkg, *rel.split("/"))
            if os.path.exists(src):
                _copy(src, os.path.join(MESHES_DIR, pkg, *rel.split("/")))
                pkgs.add(pkg)
                break

    packages = {p: f"/meshes/{p}" for p in pkgs}
    return "/meshes/" + urdf_rel, packages


def robot_assets(robot_id):
    """robot_id → {"urdf_url", "packages"} (지연 준비·캐시). 미등록이면 KeyError."""
    if robot_id not in _ASSETS:
        url, packages = _prepare(_ROBOT_DESC[robot_id])   # 미등록 → KeyError
        _ASSETS[robot_id] = {"urdf_url": url, "packages": packages}
    return _ASSETS[robot_id]


def prepare_meshes():
    """서버 부팅: MESHES_DIR 보장 + 기본 로봇(UR5) 준비(멱등)."""
    os.makedirs(MESHES_DIR, exist_ok=True)
    return robot_assets("ur5")
