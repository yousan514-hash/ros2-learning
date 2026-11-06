"""回転と姿勢の小さな道具（numpy だけ）。

- 回転は 3x3 の行列、姿勢は 4x4 の同次変換で持つ
- URDF の rpy は「固定軸の X → Y → Z」なので、行列は Rz(yaw) @ Ry(pitch) @ Rx(roll)
"""
from __future__ import annotations

import math

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def axis_angle(axis, angle: float) -> np.ndarray:
    """軸 axis のまわりに angle だけ回す行列（ロドリゲスの式）。"""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("長さ 0 のベクトルは向きにできない")
    return v / n


def align(a, b) -> np.ndarray:
    """単位ベクトル a を b に重ねる、いちばん小さい回転。"""
    a, b = unit(a), unit(b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:  # 反対向き: a に垂直な軸で半回転
        helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        return axis_angle(np.cross(a, helper), math.pi)
    return axis_angle(np.cross(a, b), math.acos(max(-1.0, min(1.0, c))))


def frame_from_z(z, x_hint=(0.0, 0.0, 1.0)) -> np.ndarray:
    """z 軸の向きを決め、x 軸を x_hint になるべく近づけた回転（x_hint が z と平行なら別の向きを使う）。"""
    z = unit(z)
    h = np.asarray(x_hint, dtype=float)
    x = h - np.dot(h, z) * z
    if np.linalg.norm(x) < 1e-6:
        h = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = h - np.dot(h, z) * z
    x = unit(x)
    return np.column_stack([x, np.cross(z, x), z])


def make_T(R=None, p=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


def rot_angle(R: np.ndarray) -> float:
    """回転の大きさ（ラジアン）。2 つの姿勢の差を測るのに使う。"""
    c = (np.trace(R) - 1.0) / 2.0
    s = 0.5 * math.sqrt((R[2, 1] - R[1, 2]) ** 2 + (R[0, 2] - R[2, 0]) ** 2 + (R[1, 0] - R[0, 1]) ** 2)
    return math.atan2(s, c)  # acos だけだと 0 の近くで誤差が大きい


def quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """回転行列 → 四元数 (x, y, z, w)。ROS のメッセージの順。"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def wrap(a):
    """角度を [-pi, pi) に入れる（配列でもよい）。"""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def segment_distance(p1, q1, p2, q2) -> float:
    """線分 p1-q1 と線分 p2-q2 のいちばん近い距離（Ericson「Real-Time Collision Detection」5.1.9 の方法）。"""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d1 @ d2)
            denom = a * e - b * b
            s = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))
