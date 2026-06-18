"""Numba JIT 동역학 (RNE) — robot_math.Dynamics 와 수치 동일, 호출 오버헤드 제거.

순수 NumPy RNE 는 작은 행렬연산 반복이라 호출당 ~ms. @njit(nopython)으로 컴파일하면
~µs 로 떨어져, 이산 ZOH 제어 루프(고정스텝 수천~수만 회)가 현실적이 된다.

robot_math.Dynamics 와 동일한 Modern Robotics 8장 컨벤션. 모델 데이터는 njit 친화적
스택 배열로 받는다:
    Mlist : (n+1, 4, 4)  링크 home 상대변환
    Glist : (n, 6, 6)    링크 공간관성
    Slist : (6, n)       space screw 축
"""
import numpy as np
from numba import njit


@njit(cache=True)
def _hat3(w):
    H = np.zeros((3, 3))
    H[0, 1] = -w[2]; H[0, 2] = w[1]
    H[1, 0] = w[2]; H[1, 2] = -w[0]
    H[2, 0] = -w[1]; H[2, 1] = w[0]
    return H


@njit(cache=True)
def _exp6(tw):
    """se(3) twist(6) → SE(3)(4x4). ||ω|| 가 θ (robot_math.SE3.exp6 와 동일)."""
    omega = tw[:3]
    v = tw[3:6]
    theta = np.sqrt(omega[0] ** 2 + omega[1] ** 2 + omega[2] ** 2)
    T = np.eye(4)
    if theta < 1e-12:                       # 순수 병진
        T[0, 3] = v[0]; T[1, 3] = v[1]; T[2, 3] = v[2]
        return T
    wh = _hat3(omega / theta)
    wh2 = wh @ wh
    R = np.eye(3) + np.sin(theta) * wh + (1.0 - np.cos(theta)) * wh2
    G = (np.eye(3) * theta + (1.0 - np.cos(theta)) * wh
         + (theta - np.sin(theta)) * wh2)
    p = G @ (v / theta)
    T[:3, :3] = R
    T[0, 3] = p[0]; T[1, 3] = p[1]; T[2, 3] = p[2]
    return T


@njit(cache=True)
def _adjoint(T):
    """Ad_T (6x6)."""
    R = T[:3, :3]
    p = T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[3:6, 3:6] = R
    A[3:6, :3] = _hat3(p) @ R
    return A


@njit(cache=True)
def _ad(V):
    """ad_V (6x6) = se(3) Lie bracket 연산자."""
    A = np.zeros((6, 6))
    wh = _hat3(V[:3])
    vh = _hat3(V[3:6])
    A[:3, :3] = wh
    A[3:6, 3:6] = wh
    A[3:6, :3] = vh
    return A


@njit(cache=True)
def inverse_dynamics(theta, dtheta, ddtheta, g, Ftip, Mlist, Glist, Slist):
    """RNE 역동역학 (θ,θ̇,θ̈)→τ. robot_math.Dynamics.inverse_dynamics 와 동일."""
    n = theta.shape[0]
    Ai = np.zeros((6, n))
    AdTi = np.zeros((n + 1, 6, 6))
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Vdi[3, 0] = -g[0]; Vdi[4, 0] = -g[1]; Vdi[5, 0] = -g[2]   # 중력 트릭
    AdTi[n] = _adjoint(np.linalg.inv(Mlist[n]))
    Fi = Ftip.copy()
    tau = np.zeros(n)

    Mi = np.eye(4)
    for i in range(n):
        Mi = Mi @ Mlist[i]
        Ai[:, i] = _adjoint(np.linalg.inv(Mi)) @ Slist[:, i]
        AdTi[i] = _adjoint(_exp6(Ai[:, i] * (-theta[i])) @ np.linalg.inv(Mlist[i]))
        Vi[:, i + 1] = AdTi[i] @ Vi[:, i] + Ai[:, i] * dtheta[i]
        Vdi[:, i + 1] = (AdTi[i] @ Vdi[:, i] + Ai[:, i] * ddtheta[i]
                         + _ad(Vi[:, i + 1]) @ Ai[:, i] * dtheta[i])

    for i in range(n - 1, -1, -1):
        Fi = (AdTi[i + 1].T @ Fi + Glist[i] @ Vdi[:, i + 1]
              - _ad(Vi[:, i + 1]).T @ (Glist[i] @ Vi[:, i + 1]))
        tau[i] = Fi @ Ai[:, i]
    return tau


@njit(cache=True)
def mass_matrix(theta, Mlist, Glist, Slist):
    n = theta.shape[0]
    M = np.zeros((n, n))
    z = np.zeros(n)
    z3 = np.zeros(3)
    z6 = np.zeros(6)
    for i in range(n):
        dd = np.zeros(n)
        dd[i] = 1.0
        M[:, i] = inverse_dynamics(theta, z, dd, z3, z6, Mlist, Glist, Slist)
    return M


@njit(cache=True)
def gravity_forces(theta, g, Mlist, Glist, Slist):
    """중력항 g(θ). RNE(θ, 0, 0, 중력 ON)."""
    n = theta.shape[0]
    z = np.zeros(n); z6 = np.zeros(6)
    return inverse_dynamics(theta, z, z, g, z6, Mlist, Glist, Slist)


@njit(cache=True)
def coriolis_forces(theta, dtheta, Mlist, Glist, Slist):
    """코리올리/원심 c(θ,θ̇)θ̇. RNE(θ, θ̇, 0, 중력 OFF)."""
    n = theta.shape[0]
    z = np.zeros(n); z3 = np.zeros(3); z6 = np.zeros(6)
    return inverse_dynamics(theta, dtheta, z, z3, z6, Mlist, Glist, Slist)


@njit(cache=True)
def forward_dynamics(theta, dtheta, tau, g, Ftip, Mlist, Glist, Slist):
    """순동역학 (θ,θ̇,τ)→θ̈. robot_math.Dynamics.forward_dynamics 와 동일."""
    n = theta.shape[0]
    z = np.zeros(n)
    z3 = np.zeros(3)
    z6 = np.zeros(6)
    M = mass_matrix(theta, Mlist, Glist, Slist)
    c = inverse_dynamics(theta, dtheta, z, z3, z6, Mlist, Glist, Slist)
    grav = inverse_dynamics(theta, z, z, g, z6, Mlist, Glist, Slist)
    tip = inverse_dynamics(theta, z, z, z3, Ftip, Mlist, Glist, Slist)
    return np.linalg.solve(M, tau - c - grav - tip)


def stack_model(Mlist, Glist, Slist):
    """robot_math 모델(list/array) → njit 친화적 연속 배열 튜플로 변환(1회)."""
    M = np.ascontiguousarray(np.stack([np.asarray(m, float) for m in Mlist]))
    G = np.ascontiguousarray(np.stack([np.asarray(g, float) for g in Glist]))
    S = np.ascontiguousarray(np.asarray(Slist, float))
    return M, G, S
