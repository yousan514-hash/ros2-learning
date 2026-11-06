"""動きの計画のテスト: 全部の点で解けて・上下限の中・腕どうしと机・胴体から離れている・繰り返せる。"""
import numpy as np
import pytest

from dual_arm_inspection import cell as C
from dual_arm_inspection import planner as P
from dual_arm_inspection.geometry import make_T


@pytest.fixture(scope="module")
def plan():
    return P.plan_cycle()


def test_keyframes_solved_within_limits(plan):
    assert len(plan.keyframes) > 0
    for kf in plan.keyframes:
        arm = plan.arms[kf.arm]
        assert arm.within_limits(kf.q), kf.label
        dp, dr = arm.pose_error(kf.q, kf.R, kf.p)
        assert dp < 1e-9 and dr < 1e-9, kf.label


def test_clearance_along_the_whole_motion(plan):
    for seg in plan.segments:
        for s in np.linspace(0, 1, 21):
            w = P.smoothstep(s)
            ql = seg.start.q["l"] + (seg.end.q["l"] - seg.start.q["l"]) * w
            qr = seg.start.q["r"] + (seg.end.q["r"] - seg.start.q["r"]) * w
            assert plan.arms["l"].within_limits(ql) and plan.arms["r"].within_limits(qr)
            gap, env = plan.clearance(ql, qr, seg.start.held, seg.start.variant)
            assert gap >= C.ARM_MARGIN - 1e-9, seg.label
            assert env >= C.ENV_MARGIN - 1e-9, seg.label


def test_every_face_is_captured_once(plan):
    caps = [(d["part"], d["face"]) for s in plan.segments for k, d in s.events if k == "capture"]
    expected = [(p.name, f.name) for p in plan.parts.values() for f in p.faces]
    assert sorted(caps) == sorted(expected)
    assert len(caps) == 27


def test_attach_detach_pairs(plan):
    held = None
    for seg in plan.segments:
        for k, d in seg.events:
            if k == "attach":
                assert held is None
                held = d["part"]
            elif k == "detach":
                assert held == d["part"]
                held = None
    assert held is None


def test_cycle_is_closed_and_timeline(plan):
    tl = P.Timeline(plan.segments)
    s0, _ = tl.sample(0.0)
    s1, _ = tl.sample(tl.total)
    for k in ("l", "r"):
        assert np.allclose(s0.q[k], s1.q[k])
    # 1 周して戻ったときも、撮る合図を取りこぼさない
    first = tl.events_between(0.0, tl.total)
    wrapped = tl.events_between(tl.total - 30.0, 30.0)
    assert len([e for e in first if e[1] == "capture"]) == 27
    assert all(t <= 30.0 or t > tl.total - 30.0 for t, _, _ in wrapped)


def test_part_follows_left_hand(plan):
    """持っている区間では、部品の位置 = 左手の TCP × 持ち方。"""
    tl = P.Timeline(plan.segments)
    seg_i = next(i for i, s in enumerate(plan.segments) if s.start.held)
    t = (tl.starts[seg_i] + tl.starts[seg_i + 1]) / 2
    state, _ = tl.sample(t)
    part = plan.parts[state.held]
    R, p = plan.arms["l"].fk(state.q["l"])
    world = make_T(R, p) @ part.tool_to_part(state.variant)
    grip = world[:3, :3] @ part.grip_point() + world[:3, 3]
    assert np.linalg.norm(grip - p) < 1e-12
