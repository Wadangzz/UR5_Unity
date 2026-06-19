"""numba JIT 동역학(`dynamics_fast`)이 순수 NumPy `robot_math.Dynamics` 와
수치적으로 동일함을 박제한다.

두 RNE 구현은 같은 Modern Robotics §8 알고리즘을 각자 보유한다(고속화를 위해
의도적으로 분리). 한쪽만 수정하면 조용히 어긋날 수 있으므로 동치성을 테스트로
고정. 초기 작성 시 모든 함수가 정확히 비트일치(max|diff| = 0)였다.
"""
import numpy as np
import pytest

from app.core import dynamics_fast as df
from app.core import ur5_model as ur5
from app.core.robot_math import Dynamics

MODEL = (ur5.MLIST, ur5.GLIST, ur5.SLIST)
STACK = df.stack_model(*MODEL)
TOL = 1e-10                          # 비트일치 기대, 부동소수 여유만 둠


def _states(n=200, seed=1):
    """재현 가능한 무작위 (θ, θ̇, θ̈, τ, Ftip) 상태들."""
    rng = np.random.default_rng(seed)
    for _ in range(n):
        yield (rng.standard_normal(ur5.N), rng.standard_normal(ur5.N),
               rng.standard_normal(ur5.N), rng.standard_normal(ur5.N),
               rng.standard_normal(6))


def test_inverse_dynamics_parity():
    md = 0.0
    for th, dth, ddth, _, Ftip in _states():
        a = Dynamics.inverse_dynamics(th, dth, ddth, ur5.GRAVITY, Ftip, *MODEL)
        b = df.inverse_dynamics(th, dth, ddth, ur5.GRAVITY, Ftip, *STACK)
        md = max(md, float(np.max(np.abs(a - b))))
    assert md < TOL, f'inverse_dynamics max|diff| = {md}'


def test_mass_matrix_parity():
    md = 0.0
    for th, *_ in _states():
        a = Dynamics.mass_matrix(th, *MODEL)
        b = df.mass_matrix(th, *STACK)
        md = max(md, float(np.max(np.abs(a - b))))
    assert md < TOL, f'mass_matrix max|diff| = {md}'


def test_gravity_forces_parity():
    md = 0.0
    for th, *_ in _states():
        a = Dynamics.gravity_forces(th, ur5.GRAVITY, *MODEL)
        b = df.gravity_forces(th, ur5.GRAVITY, *STACK)
        md = max(md, float(np.max(np.abs(a - b))))
    assert md < TOL, f'gravity_forces max|diff| = {md}'


def test_coriolis_forces_parity():
    md = 0.0
    for th, dth, *_ in _states():
        a = Dynamics.coriolis_forces(th, dth, *MODEL)
        b = df.coriolis_forces(th, dth, *STACK)
        md = max(md, float(np.max(np.abs(a - b))))
    assert md < TOL, f'coriolis_forces max|diff| = {md}'


def test_forward_dynamics_parity():
    md = 0.0
    for th, dth, _, tau, Ftip in _states():
        a = Dynamics.forward_dynamics(th, dth, tau, ur5.GRAVITY, Ftip, *MODEL)
        b = df.forward_dynamics(th, dth, tau, ur5.GRAVITY, Ftip, *STACK)
        md = max(md, float(np.max(np.abs(a - b))))
    assert md < TOL, f'forward_dynamics max|diff| = {md}'


@pytest.mark.parametrize('th', [ur5.READY, np.zeros(ur5.N)])
def test_named_poses_parity(th):
    """home(특이점) · ready 등 운용 자세에서도 일치."""
    th = np.asarray(th, float)
    a = Dynamics.gravity_forces(th, ur5.GRAVITY, *MODEL)
    b = df.gravity_forces(th, ur5.GRAVITY, *STACK)
    assert np.max(np.abs(a - b)) < TOL
