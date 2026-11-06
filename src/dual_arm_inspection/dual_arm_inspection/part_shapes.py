"""部品の形を三角形の並びにする（RViz の Marker TRIANGLE_LIST 用。ROS を使わない）。

synthetic-testbench の testbench/geometry.py と同じ決め方（中心が原点、軸が z、ボルトの頭が +z、
ナットの平らな面が ±x、ボルトの頭の角が 30 + 60k 度）。ねじ山・面取りは作らない。
"""
from __future__ import annotations

import math

import numpy as np

MM = 0.001


def _hex_radius(across_flats: float, theta: float) -> float:
    """平らな面が 0, 60, 120 度の向きにある正六角形の、角度 theta の向きの半径。"""
    a = (theta + math.pi / 6) % (math.pi / 3) - math.pi / 6
    return across_flats / 2 / math.cos(a)


def _outline(kind: str, size: float, n: int) -> np.ndarray:
    ang = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    if kind == "hex":
        r = np.array([_hex_radius(size, a) for a in ang])
    else:
        r = np.full(n, size / 2)
    return np.column_stack([r * np.cos(ang), r * np.sin(ang)])


def solid_prism(outline: np.ndarray, z0: float, z1: float) -> list:
    """凸の輪郭を z0〜z1 に押し出した立体（ふたは扇形に分ける）。"""
    tris = []
    n = len(outline)
    for i in range(n):
        a, b = outline[i], outline[(i + 1) % n]
        tris += [[(0, 0, z1), (*a, z1), (*b, z1)], [(0, 0, z0), (*b, z0), (*a, z0)]]
        tris += [[(*a, z0), (*b, z0), (*b, z1)], [(*a, z0), (*b, z1), (*a, z1)]]
    return tris


def ring_prism(outer: np.ndarray, inner: np.ndarray, z0: float, z1: float) -> list:
    """穴のあいた立体（外と穴の輪郭は同じ数の点、同じ角度）。"""
    tris = []
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        oa, ob, ia, ib = outer[i], outer[j], inner[i], inner[j]
        for z, flip in ((z1, False), (z0, True)):
            t1, t2 = [(*oa, z), (*ob, z), (*ib, z)], [(*oa, z), (*ib, z), (*ia, z)]
            tris += [t1[::-1], t2[::-1]] if flip else [t1, t2]
        tris += [[(*oa, z0), (*ob, z0), (*ob, z1)], [(*oa, z0), (*ob, z1), (*oa, z1)]]
        tris += [[(*ia, z0), (*ib, z1), (*ib, z0)], [(*ia, z0), (*ia, z1), (*ib, z1)]]
    return tris


def part_triangles(kind: str, dims: dict) -> np.ndarray:
    """部品の三角形 (N, 3, 3)、単位は m。"""
    d = {k: v * MM for k, v in dims.items() if isinstance(v, (int, float))}
    if kind == "bolt":
        total = d["length"] + d["head_height"]
        z_tip, z_head = -total / 2, -total / 2 + d["length"]
        head = np.array([[_hex_radius(d["head_across_flats"], t) * math.cos(t),
                          _hex_radius(d["head_across_flats"], t) * math.sin(t)]
                         for t in (math.radians(30 + 60 * k) for k in range(6))])
        tris = solid_prism(head, z_head, total / 2) + solid_prism(_outline("circle", d["d"], 24), z_tip, z_head)
    elif kind == "nut":
        n = 36
        outer = _outline("hex", d["across_flats"], n)
        inner = _outline("circle", d["d"], n)
        tris = ring_prism(outer, inner, -d["height"] / 2, d["height"] / 2)
    elif kind == "washer":
        n = 36
        tris = ring_prism(_outline("circle", d["d_outer"], n), _outline("circle", d["d_inner"], n),
                          -d["thickness"] / 2, d["thickness"] / 2)
    else:
        raise ValueError(kind)
    return np.asarray(tris, dtype=float)
