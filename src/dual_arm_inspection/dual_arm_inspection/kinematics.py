"""6 軸の腕の順運動学と、式で解く逆運動学。

腕の形（自作。urdf/dual_arm_cell.urdf.xacro と同じ寸法）:

    j1 縦の軸（z）で旋回 → 肩の高さ d1
    j2 肩の曲げ（y）     → 上腕 a2
    j3 ひじの曲げ（y）   → 前腕の前半 d4a
    j4 前腕のひねり（z） → 前腕の後半 d4b（ここが手首の中心）
    j5 手首の曲げ（y）
    j6 手先のひねり（z） → 手首の中心から d6 の点が手先（TCP）

全部の関節が 0 のとき、腕はまっすぐ上（+z）を向く。j4・j5・j6 の軸が手首の中心の 1 点で交わる
（球面手首）ので、逆運動学を「手首の中心の位置（j1〜j3）」と「手先の向き（j4〜j6、ZYZ のオイラー角）」
に分けて、式で解ける。解は最大 8 通り（肩の前後 × ひじの上下 × 手首の表裏）。
ひじの下限を 0 より大きくして「ひじが上」の形だけを使う。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import rot_angle, rot_y, rot_z, wrap

JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

# 関節の上限・下限（rad）。URDF にも同じ値を書く（test_urdf.py で照合する）
LOWER = (-2.8, -1.2, 0.05, -2.9, -2.1, -2.9)
UPPER = (2.8, 2.2, 2.7, 2.9, 2.1, 2.9)
VELOCITY = 1.5  # rad/s（URDF の velocity。動きの時間を決めるのにも使う）
# 近い解を選ぶときの重み。肩より手首を動かすほうを好む
WEIGHTS = np.array([1.0, 1.0, 1.0, 0.6, 0.6, 0.4])


@dataclass(frozen=True)
class ArmGeometry:
    prefix: str                      # "l"（左）/ "r"（右）
    base: tuple[float, float, float]  # j1 の位置（world）。向きは world と同じ
    tool: float                      # j6 から TCP までの長さ
    d1: float = 0.10
    a2: float = 0.32
    d4a: float = 0.12
    d4b: float = 0.20
    d6a: float = 0.05                # 手首の中心（j5）から j6 まで

    @property
    def d4(self) -> float:
        return self.d4a + self.d4b

    @property
    def d6(self) -> float:
        return self.d6a + self.tool

    @property
    def joint_names(self) -> list[str]:
        return [f"{self.prefix}_{j}" for j in JOINTS]


@dataclass
class Arm:
    geo: ArmGeometry
    lower: np.ndarray = field(default_factory=lambda: np.array(LOWER))
    upper: np.ndarray = field(default_factory=lambda: np.array(UPPER))

    @property
    def base(self) -> np.ndarray:
        return np.asarray(self.geo.base, dtype=float)

    # ---- 順運動学 ----
    def fk(self, q) -> tuple[np.ndarray, np.ndarray]:
        """関節角 → TCP の (向き R, 位置 p)（world）。"""
        q = np.asarray(q, dtype=float)
        g = self.geo
        r1 = rot_z(q[0])
        r23 = r1 @ rot_y(q[1] + q[2])
        r6 = r23 @ rot_z(q[3]) @ rot_y(q[4]) @ rot_z(q[5])
        p = self.points(q)["wrist"] + r6[:, 2] * g.d6
        return r6, p

    def points(self, q) -> dict[str, np.ndarray]:
        """当たりの確認に使う点（world）: 肩（j2）・ひじ（j3）・手首の中心（j5）・TCP。"""
        q = np.asarray(q, dtype=float)
        g = self.geo
        r1 = rot_z(q[0])
        shoulder = self.base + np.array([0.0, 0.0, g.d1])
        elbow = shoulder + r1 @ rot_y(q[1]) @ np.array([0.0, 0.0, g.a2])
        wrist = elbow + r1 @ rot_y(q[1] + q[2]) @ np.array([0.0, 0.0, g.d4])
        out = {"shoulder": shoulder, "elbow": elbow, "wrist": wrist}
        if len(q) == 6:
            r6 = r1 @ rot_y(q[1] + q[2]) @ rot_z(q[3]) @ rot_y(q[4]) @ rot_z(q[5])
            out["tcp"] = wrist + r6[:, 2] * g.d6
        return out

    # ---- 逆運動学 ----
    def ik_all(self, R, p, q4_hint: float = 0.0) -> list[np.ndarray]:
        """TCP の (向き R, 位置 p) を作る関節角を、上下限を気にせず全部返す（最大 8 通り）。"""
        g = self.geo
        R = np.asarray(R, dtype=float)
        pw = np.asarray(p, dtype=float) - self.base - R[:, 2] * g.d6  # 手首の中心（腕の根元から）
        x, y, z = pw
        s = z - g.d1
        rxy = math.hypot(x, y)
        q1a = math.atan2(y, x) if rxy > 1e-9 else 0.0
        sols = []
        for q1, r in ((q1a, rxy), (q1a + math.pi, -rxy)):
            cos3 = (r * r + s * s - g.a2 ** 2 - g.d4 ** 2) / (2.0 * g.a2 * g.d4)
            if abs(cos3) > 1.0 + 1e-9:
                continue  # 届かない
            cos3 = max(-1.0, min(1.0, cos3))
            for q3 in (math.acos(cos3), -math.acos(cos3)):
                # 縦から測った角度の 2 リンク: 手首 = a2*(sin q2, cos q2) + d4*(sin(q2+q3), cos(q2+q3))
                q2 = math.atan2(r, s) - math.atan2(g.d4 * math.sin(q3), g.a2 + g.d4 * math.cos(q3))
                r36 = (rot_z(q1) @ rot_y(q2 + q3)).T @ R  # = Rz(q4) Ry(q5) Rz(q6)
                s5 = math.hypot(r36[0, 2], r36[1, 2])
                if s5 < 1e-7:  # 手首がまっすぐ（特異点）: j4 と j6 の和しか決まらない
                    if r36[2, 2] > 0:
                        q4 = q4_hint
                        sols.append(np.array([q1, q2, q3, q4, 0.0, math.atan2(r36[1, 0], r36[0, 0]) - q4]))
                    continue  # j5 = pi は上下限の外
                for sign in (1.0, -1.0):
                    q5 = math.atan2(sign * s5, r36[2, 2])
                    q4 = math.atan2(sign * r36[1, 2], sign * r36[0, 2])
                    q6 = math.atan2(sign * r36[2, 1], -sign * r36[2, 0])
                    sols.append(np.array([q1, q2, q3, q4, q5, q6]))
        return [wrap(q) for q in sols]

    def within_limits(self, q, tol: float = 1e-9) -> bool:
        q = np.asarray(q)
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    def ik(self, R, p, seed=None) -> np.ndarray | None:
        """上下限の中の解のうち、seed にいちばん近いものを返す。解がなければ None。"""
        seed = np.zeros(6) if seed is None else np.asarray(seed, dtype=float)
        best, best_cost = None, math.inf
        for q in self.ik_all(R, p, q4_hint=float(seed[3])):
            if not self.within_limits(q):
                continue
            cost = float(np.sum(WEIGHTS * np.abs(q - seed)))
            if cost < best_cost:
                best, best_cost = q, cost
        return best

    def pose_error(self, q, R, p) -> tuple[float, float]:
        """関節角 q の TCP と目標の差（位置 m、向き rad）。逆運動学の確かめに使う。"""
        Rq, pq = self.fk(q)
        return float(np.linalg.norm(pq - np.asarray(p))), rot_angle(Rq.T @ np.asarray(R))
