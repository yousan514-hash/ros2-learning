"""検査の段取りを、関節角の並び（動きの計画）にする。ROS を使わない。

1 個の部品の段取り:
  1. 右腕のカメラがトレイの上へ行き、置いたままの部品の上の面を撮る
  2. 右腕が待つ位置へ戻る → 左腕が部品の真上へ行き、下りて、指を閉じて持ち、上がる
  3. 左腕が部品を「見せる点」へ出し、右腕のカメラが面の正面へ行って撮る。面を変えて繰り返す
  4. 右腕が戻る → 左腕が部品をトレイの同じ場所へ戻し、待つ位置へ戻る

目標の姿勢はすべて逆運動学（kinematics.py の式）で関節角にする。見せる向き・カメラの向きは、
面の法線のまわりの回し方に自由があるので、15 度ずつ 24 通りを試し、上下限の中で、腕どうしが
十分に離れ、今の姿勢から動きが小さいものを選ぶ。
2 つの姿勢のあいだは関節角を滑らかに補間する。途中の点でも腕どうしのすき間を確かめ、足りなければ
片腕ずつ動かす順番に切り替える（それでも足りなければ PlanError）。
"""
from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field, replace

import numpy as np

from . import cell as C
from .geometry import align, axis_angle, frame_from_z, make_T, segment_distance, unit
from .kinematics import VELOCITY, WEIGHTS, Arm

SPINS = [2 * math.pi * k / 24 for k in range(24)]
DWELL = 1.2          # 撮るときに止まる時間（s）
GRIP_TIME = 0.5
SAMPLES = 16         # 補間の途中で当たりを確かめる点の数


class PlanError(RuntimeError):
    pass


@dataclass
class State:
    q: dict[str, np.ndarray]
    grip: float
    held: str | None = None
    variant: int = 0

    def copy(self, **kw) -> "State":
        s = replace(self, q={k: v.copy() for k, v in self.q.items()})
        for k, v in kw.items():
            setattr(s, k, v)
        return s


@dataclass
class Segment:
    label: str
    duration: float
    start: State
    end: State
    events: list[tuple[str, dict]] = field(default_factory=list)  # 着いたときに起こすこと


@dataclass
class Keyframe:
    """逆運動学で解いた目標（確かめの道具が、解けたか・上下限の中か・誤差を調べる）。"""
    label: str
    arm: str
    R: np.ndarray
    p: np.ndarray
    q: np.ndarray


def smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def _box_distance(p: np.ndarray, box) -> float:
    lo, hi = np.asarray(box[0]), np.asarray(box[1])
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi)))


ENV_BOXES = {
    "torso": C.TORSO_BOX,
    "shoulder": C.SHOULDER_BOX,
    "table": ((C.TABLE_CENTER[0] - C.TABLE_SIZE[0] / 2, C.TABLE_CENTER[1] - C.TABLE_SIZE[1] / 2,
               C.TABLE_TOP_Z - C.TABLE_SIZE[2]),
              (C.TABLE_CENTER[0] + C.TABLE_SIZE[0] / 2, C.TABLE_CENTER[1] + C.TABLE_SIZE[1] / 2, C.TABLE_TOP_Z)),
    "tray": ((C.TRAY_CENTER[0] - C.TRAY_SIZE[0] / 2, C.TRAY_CENTER[1] - C.TRAY_SIZE[1] / 2, C.TABLE_TOP_Z),
             (C.TRAY_CENTER[0] + C.TRAY_SIZE[0] / 2, C.TRAY_CENTER[1] + C.TRAY_SIZE[1] / 2, C.TRAY_TOP_Z)),
    **{f"light{i}": box for i, box in enumerate(C.LIGHT_BOXES)},
}


class Planner:
    def __init__(self, arms: dict[str, Arm] | None = None, parts: list[C.Part] | None = None):
        self.arms = arms or C.make_arms()
        self.parts = {p.name: p for p in (parts if parts is not None else C.load_parts()[0])}
        self.segments: list[Segment] = []
        self.keyframes: list[Keyframe] = []
        self.ik_tries = 0     # 試した目標の数
        self.ik_found = 0     # そのうち上下限の中の解があった数
        self.ready = {
            "l": self._best("l", self._candidates_dir([0.28, 0.32, 1.02], unit([0.5, -0.2, -0.85])),
                            seed=np.array([0.6, 0.6, 1.7, 0.0, 0.8, 0.0]), other=None, label="L: ready"),
            "r": None,
        }
        self.ready["r"] = self._best(
            "r", self._candidates_dir([0.30, -0.30, 1.08], unit(C.INSPECT_POINT - np.array([0.30, -0.30, 1.08]))),
            seed=np.array([-0.6, 0.6, 1.7, 0.0, 0.8, 0.0]), other=("l", self.ready["l"], None), label="R: ready")
        self.state = State(q={k: v.copy() for k, v in self.ready.items()}, grip=C.FINGER_MAX)

    # ---- 当たりの確認 ----
    def capsules(self, arm: str, q, held: str | None = None, variant: int = 0) -> list[tuple]:
        pts = self.arms[arm].points(q)
        caps = [(pts["shoulder"], pts["elbow"], C.CAPSULE_R["upper"]),
                (pts["elbow"], pts["wrist"], C.CAPSULE_R["fore"]),
                (pts["wrist"], pts["tcp"], C.CAPSULE_R["hand_" + arm])]
        if arm == "l" and held:
            R, p = self.arms["l"].fk(q)
            c = (make_T(R, p) @ self.parts[held].tool_to_part(variant))[:3, 3]
            caps.append((c, c, C.HELD_PART_R))
        return caps

    def clearance(self, ql, qr, held: str | None = None, variant: int = 0) -> tuple[float, float]:
        """(腕どうしのすき間, 腕と机・トレイ・胴体のすき間)。どちらも m、負ならめり込み。"""
        cl, cr = self.capsules("l", ql, held, variant), self.capsules("r", qr)
        arm_gap = min(segment_distance(a0, a1, b0, b1) - ra - rb for a0, a1, ra in cl for b0, b1, rb in cr)
        return arm_gap, min(self.env_clearance("l", ql), self.env_clearance("r", qr))

    def env_clearance(self, arm: str, q) -> float:
        pts = self.arms[arm].points(q)
        samples = []
        for a, b, r in ((pts["shoulder"], pts["elbow"], C.CAPSULE_R["upper"]),
                        (pts["elbow"], pts["wrist"], C.CAPSULE_R["fore"])):
            for s in np.linspace(0.0, 1.0, 6):
                samples.append((a + (b - a) * s, r))
        return min(_box_distance(p, box) - r for p, r in samples for box in ENV_BOXES.values())

    # ---- 目標の候補 ----
    @staticmethod
    def _candidates_dir(p, z) -> list[tuple[np.ndarray, np.ndarray]]:
        """TCP の位置 p と向き z（指・カメラが向く向き）を決め、z まわりの回し方を 24 通り。"""
        R0 = frame_from_z(z)
        return [(R0 @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]]),
                 np.asarray(p, dtype=float)) for a in SPINS]

    def _best(self, arm: str, candidates, seed, other, label: str, held=None, variant=0):
        """候補の中から、解けて・上下限の中で・腕どうしが離れていて・動きの小さい関節角を選ぶ。

        other: (相手の腕, 相手の関節角, None) または None（相手を気にしない）
        """
        options = []
        for R, p in candidates:
            self.ik_tries += 1
            q = self.arms[arm].ik(R, p, seed)
            if q is None:
                continue
            self.ik_found += 1
            options.append((float(np.sum(WEIGHTS * np.abs(q - seed))), q, (R, p)))
        best = best_target = None
        # 動きの小さい順に、腕どうし・机・胴体とのすき間を確かめて、最初に通ったものを使う
        for _, q, target in sorted(options, key=lambda o: o[0]):
            if other is not None:
                ql, qr = (q, other[1]) if arm == "l" else (other[1], q)
                gap, env = self.clearance(ql, qr, held, variant)
                if gap < C.ARM_MARGIN or env < C.ENV_MARGIN:
                    continue
            best, best_target = q, target
            break
        if best is None:
            raise PlanError(f"{label}: 逆運動学の解がない（候補 {len(candidates)} 個）")
        self.keyframes.append(Keyframe(label, arm, best_target[0], best_target[1], best))
        return best

    # ---- 動き ----
    def _segment_ok(self, s0: State, s1: State) -> bool:
        for s in np.linspace(0.0, 1.0, SAMPLES):
            w = smoothstep(s)
            ql = s0.q["l"] + (s1.q["l"] - s0.q["l"]) * w
            qr = s0.q["r"] + (s1.q["r"] - s0.q["r"]) * w
            gap, env = self.clearance(ql, qr, s0.held, s0.variant)
            if gap < C.ARM_MARGIN or env < C.ENV_MARGIN:
                return False
        return True

    @staticmethod
    def _duration(s0: State, s1: State) -> float:
        dq = max(float(np.max(np.abs(s1.q[k] - s0.q[k]))) for k in ("l", "r"))
        t = dq / (VELOCITY * 0.6)  # 滑らかな補間の最高速度は平均の 1.5 倍なので、上限の 0.9 倍に収まる
        if dq > 1e-9:
            t = max(t, 0.6)
        if abs(s1.grip - s0.grip) > 1e-9:
            t = max(t, GRIP_TIME)
        return t

    def _push(self, label: str, target: State, events=()):
        start = self.state
        dur = self._duration(start, target)
        self.segments.append(Segment(label, dur, start, target, list(events)))
        self.state = target.copy()

    def move(self, label: str, ql=None, qr=None, grip=None, events=()):
        s0 = self.state
        target = s0.copy()
        if ql is not None:
            target.q["l"] = np.asarray(ql, dtype=float)
        if qr is not None:
            target.q["r"] = np.asarray(qr, dtype=float)
        if grip is not None:
            target.grip = grip
        if self._segment_ok(s0, target):
            self._push(label, target, events)
            return
        if ql is not None and qr is not None:
            for first in ("r", "l"):  # 片腕ずつ: 右を先 → 左を先
                mid = s0.copy()
                mid.q[first] = target.q[first]
                if self._segment_ok(s0, mid) and self._segment_ok(mid, target):
                    self._push(f"{label} ({first.upper()} first)", mid)
                    self._push(label, target, events)
                    return
        raise PlanError(f"{label}: 動きの途中で腕どうし（または机・胴体）が近づきすぎる")

    def wait(self, label: str, t: float, events=()):
        self.segments.append(Segment(label, t, self.state, self.state.copy(), list(events)))

    # ---- 段取り ----
    def present(self, part: C.Part, variant: int, face: C.Face) -> tuple[np.ndarray, np.ndarray]:
        """左手で face を VIEW_DIR に向け、右のカメラをその正面に置く関節角の組。"""
        n = np.asarray(face.normal, dtype=float)
        R_gp = part.grasp_variants()[variant]
        T_tp = part.tool_to_part(variant)
        seed_l, seed_r = self.state.q["l"], self.state.q["r"]
        options = []
        for psi in SPINS:
            R_wp = axis_angle(C.VIEW_DIR, psi) @ align(n, C.VIEW_DIR)
            R_wt = R_wp @ R_gp.T
            self.ik_tries += 1
            ql = self.arms["l"].ik(R_wt, C.INSPECT_POINT, seed_l)
            if ql is None:
                continue
            self.ik_found += 1
            T_wp = make_T(R_wt, C.INSPECT_POINT) @ T_tp
            target = (T_wp @ np.append(face.center, 1.0))[:3]
            cam = target + C.VIEW_DIR * C.STANDOFF
            for Rc, pc in self._candidates_dir(cam, -C.VIEW_DIR):
                self.ik_tries += 1
                qr = self.arms["r"].ik(Rc, pc, seed_r)
                if qr is None:
                    continue
                self.ik_found += 1
                cost = float(np.sum(WEIGHTS * np.abs(ql - seed_l))) + float(np.sum(WEIGHTS * np.abs(qr - seed_r)))
                options.append((cost, ql, qr, R_wt, Rc, pc))
        best = None
        for cost, ql, qr, R_wt, Rc, pc in sorted(options, key=lambda o: o[0]):
            gap, env = self.clearance(ql, qr, part.name, variant)
            if gap >= C.ARM_MARGIN and env >= C.ENV_MARGIN:
                best = (ql, qr, R_wt, Rc, pc)
                break
        ql, qr, R_wt, Rc, pc = best
        self.keyframes.append(Keyframe(f"L: present {part.name}/{face.name}", "l", R_wt, C.INSPECT_POINT.copy(), ql))
        self.keyframes.append(Keyframe(f"R: view {part.name}/{face.name}", "r", Rc, pc, qr))
        return ql, qr

    def plan_part(self, part: C.Part, seq_start: int = 0) -> int:
        seq = seq_start
        T_tray = part.tray_pose()

        def capture(face: C.Face) -> tuple[str, dict]:
            return ("capture", {"part": part.name, "kind": part.kind, "face": face.name,
                                "face_type": face.face_type, "where": face.where})

        # 1) トレイの上の面を右のカメラで撮る
        for face in [f for f in part.faces if f.where == "tray"]:
            n_w = T_tray[:3, :3] @ np.asarray(face.normal, dtype=float)
            target = (T_tray @ np.append(face.center, 1.0))[:3]
            qr = self._best("r", self._candidates_dir(target + n_w * C.TRAY_STANDOFF, -n_w), self.state.q["r"],
                            ("l", self.state.q["l"], None), f"R: view {part.name}/{face.name} (tray)")
            self.move(f"R: look at {part.name} in tray", qr=qr)
            self.wait(f"capture {part.name}/{face.name}", DWELL, [capture(face)])
            seq += 1
        self.move("R: back to ready", qr=self.ready["r"])

        # 2) 左手で取る（持ち方の候補から、取る姿勢がいちばん楽なものを選ぶ）
        best = None
        for k, R_gp in enumerate(part.grasp_variants()):
            T_wt = T_tray @ np.linalg.inv(part.tool_to_part(k))
            R_wt, p_wt = T_wt[:3, :3], T_wt[:3, 3]
            self.ik_tries += 2
            q_pre = self.arms["l"].ik(R_wt, p_wt + [0, 0, C.LIFT], self.state.q["l"])
            q_grasp = self.arms["l"].ik(R_wt, p_wt, q_pre if q_pre is not None else self.state.q["l"])
            self.ik_found += int(q_pre is not None) + int(q_grasp is not None)
            if q_pre is None or q_grasp is None:
                continue
            cost = float(np.sum(WEIGHTS * np.abs(q_pre - self.state.q["l"])))
            if best is None or cost < best[0]:
                best = (cost, k, q_pre, q_grasp, R_wt, p_wt)
        if best is None:
            raise PlanError(f"{part.name}: トレイから取る姿勢が見つからない")
        _, k, q_pre, q_grasp, R_wt, p_wt = best
        self.keyframes.append(Keyframe(f"L: above {part.name}", "l", R_wt, p_wt + [0, 0, C.LIFT], q_pre))
        self.keyframes.append(Keyframe(f"L: grasp {part.name}", "l", R_wt, p_wt.copy(), q_grasp))
        self.move(f"L: above {part.name}", ql=q_pre, grip=part.open_half_width)
        self.move(f"L: down to {part.name}", ql=q_grasp)
        self.move(f"L: close gripper on {part.name}", grip=part.grip_half_width, events=[("attach", {"part": part.name})])
        self.state.held, self.state.variant = part.name, k
        self.move(f"L: lift {part.name}", ql=q_pre)

        # 3) 持って見せる面を、順に撮る
        for face in [f for f in part.faces if f.where == "hand"]:
            ql, qr = self.present(part, k, face)
            self.move(f"present {part.name}/{face.name}", ql=ql, qr=qr)
            self.wait(f"capture {part.name}/{face.name}", DWELL, [capture(face)])
            seq += 1

        # 4) 戻す
        self.move("R: back to ready", qr=self.ready["r"])
        self.move(f"L: back above slot of {part.name}", ql=q_pre)
        self.move(f"L: put {part.name} down", ql=q_grasp)
        self.move(f"L: open gripper", grip=part.open_half_width, events=[("detach", {"part": part.name})])
        self.state.held = None
        self.move("L: up", ql=q_pre)
        self.move("L: back to ready", ql=self.ready["l"], grip=C.FINGER_MAX)
        return seq


def plan_cycle(parts: list[C.Part] | None = None, names: list[str] | None = None) -> Planner:
    """全部（または names の）部品を順に検査する計画。終わりの姿勢は始めと同じなので、繰り返せる。"""
    planner = Planner(parts=parts)
    seq = 0
    for p in planner.parts.values():
        if names and p.name not in names:
            continue
        seq = planner.plan_part(p, seq)
    return planner


class Timeline:
    """計画を時刻で引けるようにする（ノードが 30Hz で関節角を出すのに使う）。"""

    def __init__(self, segments: list[Segment]):
        if not segments:
            raise ValueError("動きが空")
        self.segments = segments
        self.starts = np.concatenate([[0.0], np.cumsum([s.duration for s in segments])])
        self.total = float(self.starts[-1])

    def index(self, t: float) -> int:
        return min(max(bisect_right(self.starts, t) - 1, 0), len(self.segments) - 1)

    def sample(self, t: float) -> tuple[State, Segment]:
        """時刻 t（0〜total）の状態。持っている部品は区間の始めの状態に従う。"""
        i = self.index(t)
        seg = self.segments[i]
        s = smoothstep((t - self.starts[i]) / seg.duration) if seg.duration > 0 else 1.0
        q = {k: seg.start.q[k] + (seg.end.q[k] - seg.start.q[k]) * s for k in ("l", "r")}
        grip = seg.start.grip + (seg.end.grip - seg.start.grip) * s
        return State(q=q, grip=grip, held=seg.start.held, variant=seg.start.variant), seg

    def events_between(self, t0: float, t1: float) -> list[tuple[float, str, dict]]:
        """区間の終わりの時刻が (t0, t1] に入る出来事（t0 > t1 なら、1 周して戻ったとみなす）。"""
        out = []
        if t1 < t0:
            return self.events_between(t0, self.total) + self.events_between(-1e-9, t1)
        for i, seg in enumerate(self.segments):
            te = float(self.starts[i + 1])
            if t0 < te <= t1:
                out.extend((te, name, data) for name, data in seg.events)
        return out
