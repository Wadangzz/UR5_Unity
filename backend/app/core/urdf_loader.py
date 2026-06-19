"""URDF → Modern Robotics 동역학 파라미터 (Mlist, Glist, Slist, M_home).

yourdfpy 로 파싱, 직렬 체인(serial arm) 가정. 회전·직동 관절을 한 스크류축으로 통일.
UR5 하드코딩값(ur5_model)과 비교 검증 완료: Slist/Mlist diff < 1e-11, Glist 정확 일치.

반환 dict 는 robot_math.Dynamics / dynamics_fast 가 그대로 받는 (Mlist, Glist, Slist) 형식.
"""
import numpy as np
from yourdfpy import URDF

_ACT = ("revolute", "continuous", "prismatic")


def load_mr_model(urdf_path, ee_link=None):
    """URDF 경로 → MR 모델 dict.

    ee_link 지정 시 base→ee_link **경로 위의 관절만** 사용(그리퍼 등 분기 제외).
    미지정 시 직렬 arm 가정으로 액추에이터 관절을 문서 순서대로(UR·iiwa 충족).
    """
    r = URDF.load(urdf_path)
    links = {lk.name: lk for lk in r.robot.links}
    if ee_link is not None:
        path = _path_joints(r, ee_link)              # fixed 포함 경로 순서
    else:
        path = [j for j in r.robot.joints if j.type in _ACT]   # 직렬 가정

    Slist, names, limits, Glist = [], [], [], []
    WC = [np.eye(4)]                 # WC[0]=base 프레임, WC[k]=k번째 액추에이터 COM 프레임
    T = np.eye(4)                    # home 에서 현재 링크 프레임의 world 변환
    for j in path:
        T = T @ np.asarray(j.origin, float)
        if j.type not in _ACT:                       # fixed 는 변환만 누적
            continue
        ax = np.asarray(j.axis, float)
        ax = ax / np.linalg.norm(ax)
        w = T[:3, :3] @ ax           # 축 방향(world)
        q = T[:3, 3]                 # 축 위 한 점(world)
        if j.type == "prismatic":
            Slist.append(np.concatenate([np.zeros(3), w]))
        else:
            Slist.append(np.concatenate([w, -np.cross(w, q)]))
        ine = links[j.child].inertial
        WC.append(T @ np.asarray(ine.origin, float))     # COM 프레임(world)
        G = np.zeros((6, 6))
        G[:3, :3] = np.asarray(ine.inertia, float)       # COM 관성텐서
        G[3:, 3:] = float(ine.mass) * np.eye(3)
        Glist.append(G)
        names.append(j.name)
        lim = getattr(j, "limit", None)
        if lim is not None and j.type != "continuous":
            limits.append((float(lim.lower), float(lim.upper)))
        else:
            limits.append((-np.pi, np.pi))

    n = len(names)
    M_home = T.copy()                # EE = ee_link (또는 마지막 액추에이터 링크) 프레임

    Mlist = [WC[1]]                  # M01 = WC1 (base→링크1 COM)
    for i in range(1, n):
        Mlist.append(np.linalg.inv(WC[i]) @ WC[i + 1])
    Mlist.append(np.linalg.inv(WC[n]) @ M_home)          # 링크n COM → EE

    return {
        "name": r.robot.name,
        "n": n,
        "Mlist": Mlist,
        "Glist": Glist,
        "Slist": np.array(Slist).T,  # 6×n (열 = 관절)
        "M_home": M_home,
        "joint_names": names,
        "joint_limits": limits,
    }


def _path_joints(r, ee_link):
    """base(root) → ee_link 경로의 관절 리스트(fixed 포함, 순서대로)."""
    by_child = {j.child: j for j in r.robot.joints}
    seq = []
    cur = ee_link
    while cur in by_child:            # child→parent 로 거슬러 올라감
        seq.append(by_child[cur])
        cur = by_child[cur].parent
    return list(reversed(seq))
