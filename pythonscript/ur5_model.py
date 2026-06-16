"""
UR5 동역학 모델 데이터 (Modern Robotics 표준 파라미터).

단위: 길이 m, 질량 kg, 시간 s (SI) → 토크는 N·m 로 나온다.
출처: Modern Robotics (Lynch & Park), UR5 예제.
MyRobotMath.Dynamics 와 동일한 MR 컨벤션이므로 그대로 사용 가능.

    Mlist : 링크 좌표계 home 상대변환 M_{i-1,i}  (n+1 = 7 개)
    Glist : 링크별 공간관성 G_i = diag(Ixx, Iyy, Izz, m, m, m)  (n = 6 개)
    Slist : space frame 기준 screw 축 (6 x 6)
"""

import numpy as np
from MyRobotMath import SE3

# --- 링크 좌표계 home 상대변환 (M_{i-1,i}) ---
M01 = np.array([[1, 0, 0, 0],     [0, 1, 0, 0],      [0, 0, 1, 0.089159], [0, 0, 0, 1]])
M12 = np.array([[0, 0, 1, 0.28],  [0, 1, 0, 0.13585],[-1, 0, 0, 0],       [0, 0, 0, 1]])
M23 = np.array([[1, 0, 0, 0],     [0, 1, 0, -0.1197],[0, 0, 1, 0.395],    [0, 0, 0, 1]])
M34 = np.array([[0, 0, 1, 0],     [0, 1, 0, 0],      [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
M45 = np.array([[1, 0, 0, 0],     [0, 1, 0, 0.093],  [0, 0, 1, 0],        [0, 0, 0, 1]])
M56 = np.array([[1, 0, 0, 0],     [0, 1, 0, 0],      [0, 0, 1, 0.09465],  [0, 0, 0, 1]])
M67 = np.array([[1, 0, 0, 0],     [0, 0, 1, 0.0823], [0, -1, 0, 0],       [0, 0, 0, 1]])
MLIST = [M01, M12, M23, M34, M45, M56, M67]

# --- 링크별 공간관성 G_i = diag(Ixx, Iyy, Izz, m, m, m) ---
GLIST = [
    np.diag([0.010267495893, 0.010267495893, 0.00666,    3.7,    3.7,    3.7]),
    np.diag([0.22689067591,  0.22689067591,  0.0151074,  8.393,  8.393,  8.393]),
    np.diag([0.049443313556, 0.049443313556, 0.004095,   2.275,  2.275,  2.275]),
    np.diag([0.111172755531, 0.111172755531, 0.21942,    1.219,  1.219,  1.219]),
    np.diag([0.111172755531, 0.111172755531, 0.21942,    1.219,  1.219,  1.219]),
    np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879]),
]

# --- space frame screw 축 (6 x 6, 열 = 관절) ---
SLIST = np.array([
    [0,  0,        0,         0,         0,        0],
    [0,  1,        1,         1,         0,        1],
    [1,  0,        0,         0,        -1,        0],
    [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
    [0,  0,        0,         0,         0.81725,  0],
    [0,  0,        0.425,     0.81725,   0,        0.81725],
])

# 중력가속도 (월드 -z 방향)
GRAVITY = np.array([0.0, 0.0, -9.81])

# 관절 개수
N = 6


def home_config():
    """모든 관절각=0 일 때 말단(end-effector) 변환행렬 M07 = ∏ Mlist."""
    M = np.eye(4)
    for Mi in MLIST:
        M = M @ Mi
    return M


def fk_all_joints(theta):
    """
    각 링크 좌표계 원점의 공간 좌표를 순서대로 반환 (base 포함).
    시각화에서 팔을 막대로 잇는 데 사용.
    space frame PoE:  T_{0,i}(θ) = [∏_{j≤i} exp([S_j]θ_j)] · M_{0,i}
    :param theta: 관절각 (rad, 6개)
    :return: (8,3) 배열 — base, joint1..6 프레임, end-effector 원점
    """
    points = [np.zeros(3)]      # base 원점
    exp_prod = np.eye(4)        # ∏ exp([S_j]θ_j)
    M_cum = np.eye(4)           # M_{0,i}
    for i in range(N):
        exp_prod = exp_prod @ SE3.exp6(SLIST[:, i] * theta[i])
        M_cum = M_cum @ MLIST[i]
        points.append((exp_prod @ M_cum)[:3, 3])
    M_cum = M_cum @ MLIST[N]    # end-effector (M_{0,7})
    points.append((exp_prod @ M_cum)[:3, 3])
    return np.array(points)
