"""URDF（xacro）と Python の運動学の寸法・上下限が同じかを確かめる。"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from dual_arm_inspection import cell as C
from dual_arm_inspection.geometry import axis_angle, make_T, rot_angle, rot_rpy
from dual_arm_inspection.kinematics import JOINTS, LOWER, UPPER

# xacro が無いときはこのファイルのテストだけを飛ばす。pytest.importorskip を先頭で使うと、
# Humble の launch_testing の pytest プラグイン（集めるときにファイルを import する）が
# 「飛ばす」をフォルダ全体に広げ、ほかのテストまで 0 件になる（LOG.md の 2026-11-07）。
try:
    import xacro
except ImportError:
    xacro = None
pytestmark = pytest.mark.skipif(xacro is None, reason="xacro が無い（Ubuntu は apt で ros-humble-xacro を入れる）")
XACRO = Path(__file__).resolve().parents[1] / "urdf" / "dual_arm_cell.urdf.xacro"


def load(with_cell="true"):
    doc = xacro.process_file(str(XACRO), mappings={"with_cell": with_cell})
    return ET.fromstring(doc.toxml())


def joints(root):
    out = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0]
        lim = j.find("limit")
        out[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
            "T": make_T(rot_rpy(*rpy), xyz), "axis": axis,
            "limit": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None}
    return out


def urdf_fk(js, link, q):
    """URDF の関節をたどって world → link の姿勢を出す。"""
    if link not in js:
        return np.eye(4)
    j = js[link]
    T = urdf_fk(js, j["parent"], q) @ j["T"]
    v = q.get(j["name"], 0.0)
    if j["type"] == "revolute":
        T = T @ make_T(axis_angle(j["axis"], v))
    elif j["type"] == "prismatic":
        T = T @ make_T(None, np.asarray(j["axis"]) * v)
    return T


@pytest.fixture(scope="module")
def js():
    return joints(load())


@pytest.mark.parametrize("side,tip", [("l", "l_tcp"), ("r", "r_camera_optical_frame")])
def test_fk_matches_urdf(js, side, tip):
    arm = C.make_arms()[side]
    rng = np.random.default_rng(5)
    for _ in range(100):
        q = rng.uniform(arm.lower, arm.upper)
        T = urdf_fk(js, tip, dict(zip(arm.geo.joint_names, q)))
        R, p = arm.fk(q)
        assert np.linalg.norm(T[:3, 3] - p) < 1e-9
        assert rot_angle(T[:3, :3].T @ R) < 1e-9


def test_limits_match(js):
    by_name = {v["name"]: v for v in js.values()}
    for side in ("l", "r"):
        for i, j in enumerate(JOINTS):
            lo, hi = by_name[f"{side}_{j}"]["limit"]
            assert (lo, hi) == (LOWER[i], UPPER[i])
    lo, hi = by_name["l_finger_a_joint"]["limit"]
    assert hi == pytest.approx(C.FINGER_MAX)


def test_cell_optional():
    names = {link.get("name") for link in load("false").findall("link")}
    assert "table_link" not in names and "l_tcp" in names
    assert "table_link" in {link.get("name") for link in load("true").findall("link")}
