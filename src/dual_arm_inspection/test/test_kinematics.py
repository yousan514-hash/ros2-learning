"""逆運動学（式で解く）のテスト。"""
import math

import numpy as np
import pytest

from dual_arm_inspection import cell as C
from dual_arm_inspection.geometry import rot_angle


@pytest.fixture(scope="module")
def arms():
    return C.make_arms()


@pytest.mark.parametrize("side", ["l", "r"])
def test_round_trip_random(arms, side):
    """上下限の中の関節角 → 順運動学 → 逆運動学 → 同じ手先の姿勢に戻る（500 回すべて）。"""
    arm = arms[side]
    rng = np.random.default_rng(1)
    for _ in range(500):
        q = rng.uniform(arm.lower, arm.upper)
        R, p = arm.fk(q)
        sol = arm.ik(R, p, seed=rng.uniform(arm.lower, arm.upper))
        assert sol is not None
        assert arm.within_limits(sol)
        dp, dr = arm.pose_error(sol, R, p)
        assert dp < 1e-9 and dr < 1e-9


def test_all_branches_reach_target(arms):
    """上下限を気にしない全部の解（最大 8 通り）が、どれも同じ姿勢を作る。"""
    arm = arms["l"]
    q = np.array([0.4, 0.7, 1.3, 0.5, 0.9, -0.6])
    R, p = arm.fk(q)
    sols = arm.ik_all(R, p)
    assert len(sols) == 8
    for s in sols:
        dp, dr = arm.pose_error(s, R, p)
        assert dp < 1e-9 and dr < 1e-9


def test_wrist_singularity(arms):
    """手首がまっすぐ（j5 = 0）でも解ける（j4 は手がかりの値を使い、j6 で合わせる）。"""
    arm = arms["r"]
    q = np.array([-0.3, 0.6, 1.4, 0.8, 0.0, -0.2])
    R, p = arm.fk(q)
    sol = arm.ik(R, p, seed=q)
    assert sol is not None
    dp, dr = arm.pose_error(sol, R, p)
    assert dp < 1e-9 and dr < 1e-9


def test_unreachable_returns_none(arms):
    arm = arms["l"]
    R = np.eye(3)
    assert arm.ik(R, arm.base + [2.0, 0.0, 0.0]) is None


def test_limits_exclude_elbow_down(arms):
    """ひじの下限が正なので、ひじが下の解は選ばれない。"""
    arm = arms["l"]
    q = np.array([0.2, 0.9, 1.6, 0.0, 0.7, 0.0])
    R, p = arm.fk(q)
    for s in arm.ik_all(R, p):
        if s[2] < 0:
            assert not arm.within_limits(s)


def test_rot_angle_precise_near_zero():
    from dual_arm_inspection.geometry import rot_z
    assert abs(rot_angle(rot_z(1e-10)) - 1e-10) < 1e-15
    assert abs(rot_angle(rot_z(math.pi / 3)) - math.pi / 3) < 1e-12
