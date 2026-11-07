"""濃さと大きさを決めた欠陥（汚れ・打痕・傷）の形を、部品の面に沿って作る（2026-11-08、伊佐美）。

numpy だけで書いてある（Isaac Sim の Python でも pixi の環境でも読める。pixi では形の確かめに使う）。

前の作り方（capture_inspection.py の make_defect_pool）は、暗い球・茶色の円盤・細い箱を面に置いていたので、
打痕・汚れの濃さが 87〜96%（中央値）とほぼ黒になり、見張り（部品の面積）にも当たっていた。ここでは:
  - 濃さ: 部品と同じ金属の材質（亜鉛）で、色だけを k = (1 - 目標の濃さ) ** gamma 倍にする（ねらいは 10〜90%）
  - 大きさ: 面の「材料の幅」s（ボルトの軸の半径・座金の輪の幅など）に比例させる（部品の大きさに合わせる）
  - 形: 面に沿わせる。ボルトの横は円柱に巻きつけ、ナット・座金の輪は輪に沿って曲げる。打痕は浅いふくらみ
  - 置き場所: 材料のある所だけ（穴・縁にかからない窓の中）

座標は部品の座標（中心が原点、軸が z、ボルトの頭が +z、ナットの平らな面が 0, 60, ... 度）。単位は m で返す。
"""
from __future__ import annotations

import math

import numpy as np

MM = 0.001
LIFT_MM = 0.03          # 面から浮かせる高さ（重なってちらつかないように）
N_R, N_T = 6, 28        # 円盤（汚れ・打痕）: 輪の数・周の分け方
N_L = 24                # 傷: 長さ方向の分け方
TYPES = ("dent", "scratch", "stain")
STAIN_TINT = (1.0, 0.95, 0.88)   # 汚れはわずかに茶色（濃さは灰色にしたときの値で測る）


# ---------------------------------------------------------------- つなぎ方（最初に 1 回だけ USD に書く）
def disk_topology() -> tuple[list[int], list[int]]:
    """中心 1 点 + N_R 本の輪 x N_T 点。+法線の側から見て反時計回り。"""
    counts, idx = [], []
    for j in range(N_T):
        counts.append(3)
        idx += [0, 1 + j, 1 + (j + 1) % N_T]
    for i in range(1, N_R):
        for j in range(N_T):
            a, b = 1 + (i - 1) * N_T + j, 1 + (i - 1) * N_T + (j + 1) % N_T
            c, d = 1 + i * N_T + (j + 1) % N_T, 1 + i * N_T + j
            counts.append(4)
            idx += [a, d, c, b]
    return counts, idx


def strip_topology() -> tuple[list[int], list[int]]:
    """長さ方向に N_L 区切り x 幅 2 点。"""
    counts, idx = [], []
    for i in range(N_L):
        counts.append(4)
        idx += [2 * i, 2 * (i + 1), 2 * (i + 1) + 1, 2 * i + 1]
    return counts, idx


def annulus_topology(n: int = 48) -> tuple[list[int], list[int]]:
    """穴あきの板（内側 n 点・外側 n 点）。"""
    counts, idx = [], []
    for j in range(n):
        a, b = j, (j + 1) % n
        counts.append(4)
        idx += [a, n + a, n + b, b]
    return counts, idx


TOPOLOGY = {"dent": disk_topology(), "stain": disk_topology(), "scratch": strip_topology()}
N_POINTS = {"dent": 1 + N_R * N_T, "stain": 1 + N_R * N_T, "scratch": 2 * (N_L + 1)}


# ---------------------------------------------------------------- 足あと（面の上の 2 次元の形、mm）
def disk_footprint(a: float, b: float, psi: float, depth: float):
    """楕円（半径 a・b、向き psi）。depth > 0 なら中心が depth だけふくらむ（打痕）。
    戻り値: x, y, h（面からの高さ）, dh/dx, dh/dy。"""
    rho = np.concatenate([[0.0], np.repeat(np.arange(1, N_R + 1) / N_R, N_T)])
    th = np.concatenate([[0.0], np.tile(np.arange(N_T) * 2 * math.pi / N_T, N_R)])
    lx, ly = a * rho * np.cos(th), b * rho * np.sin(th)
    h = depth * (1.0 - rho ** 2)
    gx, gy = -2 * depth * lx / a ** 2, -2 * depth * ly / b ** 2   # 楕円の座標での傾き
    c, s = math.cos(psi), math.sin(psi)
    return c * lx - s * ly, s * lx + c * ly, h, c * gx - s * gy, s * gx + c * gy


def strip_footprint(length: float, width: float, psi: float):
    """細長い帯（傷）。両端は少し細くする。"""
    t = np.repeat(np.linspace(-1.0, 1.0, N_L + 1), 2)
    side = np.tile([-1.0, 1.0], N_L + 1)
    lx = length / 2 * t
    ly = width / 2 * side * (1.0 - 0.6 * np.abs(t) ** 6)
    c, s = math.cos(psi), math.sin(psi)
    z = np.zeros_like(lx)
    return c * lx - s * ly, s * lx + c * ly, z, z, z


def half_extent(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    return float(np.max(np.abs(x))), float(np.max(np.abs(y))), float(np.max(np.hypot(x, y)))


# ---------------------------------------------------------------- 面の形（足あと → 部品の座標）
def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def map_plane(o, e1, e2, n, x, y, h, gx, gy):
    o, e1, e2, n = (np.asarray(v, float) for v in (o, e1, e2, n))
    hh = h + LIFT_MM
    P = o + np.outer(x, e1) + np.outer(y, e2) + np.outer(hh, n)
    N = np.outer(np.ones_like(x), n) - np.outer(gx, e1) - np.outer(gy, e2)
    return P, N


def map_cylinder(R, phi0, x, y, h, gx, gy, z0):
    """円柱（半径 R、軸が z）に巻きつける。x は周の長さ、y は軸の向き。"""
    phi = phi0 + x / R
    rr = R + h + LIFT_MM
    P = np.column_stack([rr * np.cos(phi), rr * np.sin(phi), z0 + y])
    nr = np.column_stack([np.cos(phi), np.sin(phi), np.zeros_like(phi)])
    ep = np.column_stack([-np.sin(phi), np.cos(phi), np.zeros_like(phi)])
    ez = np.array([0.0, 0.0, 1.0])
    N = nr - gx[:, None] * ep - np.outer(gy, ez)
    return P, N


def map_ring(o, n, e1, e2, rho0, th0, x, y, h, gx, gy):
    """平らな輪（ナット・座金の上下の面）に沿わせる。x は輪に沿った長さ、y は半径の内向き
    （x・y・法線が右手系になるように内向きにする）。"""
    o, n, e1, e2 = (np.asarray(v, float) for v in (o, n, e1, e2))
    rho = rho0 - y
    th = th0 + x / rho0
    rad = np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2)
    tan = -np.outer(np.sin(th), e1) + np.outer(np.cos(th), e2)
    P = o + rho[:, None] * rad + np.outer(h + LIFT_MM, n)
    N = np.outer(np.ones_like(x), n) - gx[:, None] * tan + gy[:, None] * rad
    return P, N


# ---------------------------------------------------------------- 面ごとの決めごと
def face_info(spec: dict, face: str, view: dict | None = None) -> dict:
    """欠陥を置く面の情報: 形（plane / cylinder / ring）、材料の幅 s（mm）、置いてよい窓、傷の向き。"""
    view = view or {}
    k = spec["kind"]
    if k == "bolt":
        T = spec["length"] + spec["head_height"]
        z_tip, z_head = -T / 2, -T / 2 + spec["length"]
        R = spec["d"] / 2
        if face in ("side_top", "side_bottom"):
            # 軸の円柱。周は ±0.6R（約 ±34 度）まで、軸の向きは頭とねじ先から 0.6mm 離す
            return {"shape": "cylinder", "R": R, "phi0": 0.0 if face == "side_top" else math.pi,
                    "z0": (z_tip + z_head) / 2, "s": R, "win": ("rect", 0.6 * R, (z_head - z_tip) / 2 - 0.6),
                    "scratch_psi": (math.pi / 2, math.radians(30)), "scratch_scale": 2.5}
        if face == "head_end":
            af = spec["head_across_flats"]
            return {"shape": "plane", "o": (0, 0, T / 2), "e1": (1, 0, 0), "e2": (0, 1, 0), "n": (0, 0, 1),
                    "s": af / 2, "win": ("disk", 0.85 * af / 2), "scratch_psi": None, "scratch_scale": 1.0}
        if face == "tip_end":
            return {"shape": "plane", "o": (0, 0, z_tip), "e1": (1, 0, 0), "e2": (0, -1, 0), "n": (0, 0, -1),
                    "s": R, "win": ("disk", 0.85 * R), "scratch_psi": None, "scratch_scale": 1.0}
        raise ValueError(face)
    if k == "nut":
        h, af, d = spec["height"], spec["across_flats"], spec["d"]
        if face == "side":
            side = af / math.sqrt(3)
            if view.get("nut") == "flat":
                angs = [60.0]          # 部品を 30 度回して、60 度の平らな面をカメラに向ける
            else:
                angs = [60.0, 120.0]   # 角（90 度）をカメラに向けた前の撮り方: 角の両側の面
            return {"shape": "plane_choice", "angles": angs, "dist": af / 2, "s": min(h, side),
                    "win": ("rect", 0.42 * side, 0.40 * h), "scratch_psi": None, "scratch_scale": 1.0}
        rin, rout, z = d / 2, af / 2, h / 2
    elif k == "washer":
        rin, rout, z = spec["d_inner"] / 2, spec["d_outer"] / 2, spec["thickness"] / 2
    else:
        raise ValueError(k)
    sz = 1.0 if face == "top" else -1.0
    w = rout - rin
    e2 = (0, 1, 0) if sz > 0 else (0, -1, 0)   # 下の面は外から見て反時計回りになるように
    return {"shape": "ring", "o": (0, 0, sz * z), "n": (0, 0, sz), "e1": (1, 0, 0), "e2": e2,
            "rho0": (rin + rout) / 2, "s": w, "win": ("ring", 0.42 * w), "scratch_psi": (0.0, math.radians(8)),
            "scratch_scale": 1.6}


def contrast_to_factor(target: float, gamma: float) -> float:
    """目標の濃さ（画像の明るさが何割下がるか）→ 材質の色に掛ける値。画面の明るさは材質の色に比例しない
    （トーンマッピング）ので、gamma は試し撮りで合わせる。"""
    return float(np.clip(1.0 - target, 0.0, 1.0) ** gamma)


def make_defect(kind: str, spec: dict, face: str, rng, ppm: float, view: dict | None = None) -> dict:
    """欠陥の形を作る。戻り値: points（部品の座標、m）、normals、大きさなどの記録。

    ppm は 1mm が何画素か（傷の幅を 2.2 画素より細くしない。細すぎると画素の中で薄まって濃さが測れない）。
    """
    fi = face_info(spec, face, view)
    s = fi["s"]
    for attempt in range(20):
        shrink = 0.85 ** attempt
        if kind == "stain":
            a = rng.uniform(0.22, 0.38) * s * shrink
            b = a * rng.uniform(0.6, 1.0)
            psi = rng.uniform(0, math.pi)
            fp = disk_footprint(a, b, psi, 0.0)
            size = (2 * a, 2 * b)
        elif kind == "dent":
            a = rng.uniform(0.12, 0.22) * s * shrink
            b = a * rng.uniform(0.8, 1.0)
            psi = rng.uniform(0, math.pi)
            depth = rng.uniform(0.03, 0.06) * a
            fp = disk_footprint(a, b, psi, depth)
            size = (2 * a, 2 * b)
        elif kind == "scratch":
            length = rng.uniform(0.7, 1.4) * s * fi["scratch_scale"] * shrink
            width = max(2.2 / ppm, 0.04 * s)
            if fi["scratch_psi"] is None:
                psi = rng.uniform(0, math.pi)
            else:
                psi = fi["scratch_psi"][0] + rng.uniform(-1, 1) * fi["scratch_psi"][1]
            fp = strip_footprint(length, width, psi)
            size = (length, width)
        else:
            raise ValueError(kind)
        x, y, h, gx, gy = fp
        ex, ey, er = half_extent(x, y)
        win = fi["win"]
        if win[0] == "rect":
            hx, hy = win[1] - ex, win[2] - ey
            if hx < 0 or hy < 0:
                continue
            cx, cy = rng.uniform(-hx, hx), rng.uniform(-hy, hy)
        elif win[0] == "disk":
            rmax = win[1] - er
            if rmax < 0:
                continue
            r, t = rmax * math.sqrt(rng.uniform()), rng.uniform(0, 2 * math.pi)
            cx, cy = r * math.cos(t), r * math.sin(t)
        else:  # ring: 半径の向き（y）だけ窓。輪に沿った向きは角度で決める
            hy = win[1] - ey
            if hy < 0:
                continue
            cx, cy = 0.0, rng.uniform(-hy, hy)
        break
    else:
        raise RuntimeError(f"{spec['name']}/{face}: {kind} を窓に入れられない")

    x, y = x + cx, y + cy
    if fi["shape"] == "cylinder":
        P, N = map_cylinder(fi["R"], fi["phi0"], x, y, h, gx, gy, fi["z0"])
        where = {"phi_deg": math.degrees(fi["phi0"] + cx / fi["R"]), "z_mm": fi["z0"] + cy}
    elif fi["shape"] == "ring":
        th0 = rng.uniform(0, 2 * math.pi)
        P, N = map_ring(fi["o"], fi["n"], fi["e1"], fi["e2"], fi["rho0"], th0, x, y, h, gx, gy)
        where = {"rho_mm": fi["rho0"] - cy, "theta_deg": math.degrees(th0)}
    elif fi["shape"] == "plane_choice":
        ang = math.radians(float(rng.choice(fi["angles"])))
        n = np.array([math.cos(ang), math.sin(ang), 0.0])
        e1 = np.array([-math.sin(ang), math.cos(ang), 0.0])
        e2 = np.array([0.0, 0.0, 1.0])
        P, N = map_plane(n * fi["dist"], e1, e2, n, x, y, h, gx, gy)
        where = {"flat_deg": math.degrees(ang), "u_mm": cx, "z_mm": cy}
    else:
        P, N = map_plane(fi["o"], fi["e1"], fi["e2"], fi["n"], x, y, h, gx, gy)
        where = {"u_mm": cx, "v_mm": cy}
    N = N / np.linalg.norm(N, axis=1, keepdims=True)
    return {"points": P * MM, "normals": N, "size_mm": [round(float(v), 4) for v in size], "s_mm": round(float(s), 4),
            "where": {k: round(float(v), 4) for k, v in where.items()}, "shrink": round(shrink, 4)}


def collar_points(spec: dict, gap_mm: float, behind_mm: float = 1.5, outer_scale: float = 12.0, n: int = 48):
    """ボルトの先端の面を撮るときの、軸に通す黒い穴あき板（部品の座標、m）。
    穴の半径 = 軸の半径 + gap_mm。先端の面から behind_mm だけ頭の側に置く。"""
    T = spec["length"] + spec["head_height"]
    R = spec["d"] / 2
    z = -T / 2 + behind_mm
    th = np.arange(n) * 2 * math.pi / n
    rin, rout = R + gap_mm, outer_scale * R
    inner = np.column_stack([rin * np.cos(th), rin * np.sin(th), np.full(n, z)])
    outer = np.column_stack([rout * np.cos(th), rout * np.sin(th), np.full(n, z)])
    return np.vstack([inner, outer]) * MM


def nut_flat_center_local(spec: dict) -> np.ndarray:
    """「平らな面を正面に」の撮り方で、カメラに向ける平らな面（60 度）の中心（部品の座標、m）。"""
    a = math.radians(60.0)
    return np.array([math.cos(a), math.sin(a), 0.0]) * spec["across_flats"] / 2 * MM


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T
