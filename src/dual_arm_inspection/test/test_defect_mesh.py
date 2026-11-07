"""Isaac Sim で使う欠陥の形（isaac/defect_mesh.py）が、材料のある所に乗るかを確かめる（Isaac Sim は使わない）。"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ISAAC = Path(__file__).resolve().parents[1] / "isaac"
sys.path.insert(0, str(ISAAC))
import defect_mesh as DM  # noqa: E402

PARTS = {p["name"]: p for p in json.loads((Path(__file__).resolve().parents[1] / "config" / "parts.json")
                                        .read_text(encoding="utf-8"))["parts"]}
FACES = {"bolt": ["side_top", "side_bottom", "head_end", "tip_end"], "nut": ["top", "bottom", "side"],
         "washer": ["top", "bottom"]}
LIFT = DM.LIFT_MM


def _cases():
    for name, spec in PARTS.items():
        for face in FACES[spec["kind"]]:
            views = [{}, {"nut": "flat"}] if face == "side" else [{}]
            for view in views:
                for kind in DM.TYPES:
                    yield name, face, kind, view


@pytest.mark.parametrize("name,face,kind,view", list(_cases()))
def test_defect_sits_on_material(name, face, kind, view):
    spec = PARTS[name]
    rng = np.random.default_rng(3)
    for _ in range(40):
        d = DM.make_defect(kind, spec, face, rng, ppm=10.0, view=view)
        P = d["points"] / DM.MM                     # mm
        assert P.shape == (DM.N_POINTS[kind], 3)
        assert np.allclose(np.linalg.norm(d["normals"], axis=1), 1.0)
        k = spec["kind"]
        if k == "bolt":
            T = spec["length"] + spec["head_height"]
            z_tip, z_head, R = -T / 2, -T / 2 + spec["length"], spec["d"] / 2
            if face.startswith("side"):
                r = np.hypot(P[:, 0], P[:, 1])
                assert np.all(r >= R + LIFT - 1e-9) and np.all(r <= R + LIFT + 0.07 * R)   # 円柱に沿う（打痕は少しふくらむ）
                assert np.all(P[:, 2] > z_tip + 0.5) and np.all(P[:, 2] < z_head - 0.5)      # 軸の上（頭・ねじ先にかからない）
                sx = 1.0 if face == "side_top" else -1.0
                assert np.all(sx * P[:, 0] > 0.5 * R)                                          # カメラの側
            elif face == "head_end":
                assert np.all(np.hypot(P[:, 0], P[:, 1]) <= spec["head_across_flats"] / 2)     # 六角の内接円の中
                assert np.all(P[:, 2] >= T / 2 + LIFT - 1e-9)
            else:
                assert np.all(np.hypot(P[:, 0], P[:, 1]) <= R)
                assert np.all(P[:, 2] <= z_tip - LIFT + 1e-9)
        elif face == "side":
            # ナットの横: 平らな面の上（面の中心からの距離が af/2 + 浮かせた分、高さの中）
            ang = math.radians(d["where"]["flat_deg"])
            n = np.array([math.cos(ang), math.sin(ang), 0.0])
            dist = P @ n
            assert np.all(dist >= spec["across_flats"] / 2 + LIFT - 1e-9)
            assert np.all(dist <= spec["across_flats"] / 2 + LIFT + 0.07 * spec["height"])
            assert np.all(np.abs(P[:, 2]) <= spec["height"] / 2)
            e1 = np.array([-math.sin(ang), math.cos(ang), 0.0])
            assert np.all(np.abs(P @ e1) <= spec["across_flats"] / math.sqrt(3) / 2)
            if view.get("nut") == "flat":
                assert d["where"]["flat_deg"] == 60.0
        else:
            rin = (spec["d"] if k == "nut" else spec["d_inner"]) / 2
            rout = (spec["across_flats"] if k == "nut" else spec["d_outer"]) / 2
            r = np.hypot(P[:, 0], P[:, 1])
            assert np.all(r > rin) and np.all(r < rout)                                       # 穴と縁にかからない
            half = (spec["height"] if k == "nut" else spec["thickness"]) / 2
            sz = 1.0 if face == "top" else -1.0
            assert np.all(sz * P[:, 2] >= half + LIFT - 1e-9)


def test_size_follows_part():
    """同じ種類の欠陥は、大きい部品ほど大きい（部品の大きさに合わせる）。"""
    rng = np.random.default_rng(0)
    for kind in ("stain", "dent"):
        sizes = {n: np.mean([DM.make_defect(kind, PARTS[n], "top", rng, 10.0)["size_mm"][0] for _ in range(200)])
                 for n in ("washer_M4", "washer_M8")}
        assert sizes["washer_M8"] > 1.3 * sizes["washer_M4"]


def test_contrast_factor():
    assert DM.contrast_to_factor(0.0, 2.2) == 1.0
    assert math.isclose(DM.contrast_to_factor(0.5, 1.0), 0.5)
    assert DM.contrast_to_factor(0.9, 2.2) < DM.contrast_to_factor(0.1, 2.2)


def test_topology_matches_points():
    for kind in DM.TYPES:
        counts, idx = DM.TOPOLOGY[kind]
        assert sum(counts) == len(idx)
        assert max(idx) == DM.N_POINTS[kind] - 1
    counts, idx = DM.annulus_topology(48)
    assert max(idx) == 2 * 48 - 1 and DM.collar_points(PARTS["bolt_M6x30"], 0.05).shape == (96, 3)
