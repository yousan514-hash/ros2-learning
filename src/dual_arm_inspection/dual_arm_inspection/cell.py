"""検査セルの寸法・部品・撮る面・持ち方（ROS を使わない、ふつうの Python）。

座標: world の原点はロボットの足もとの床、x が前（机のほう）、y が左、z が上。長さは m。
URDF（urdf/dual_arm_cell.urdf.xacro）の寸法と合わせてある。腕の寸法と上下限は test_urdf.py で照合する。

部品は synthetic-testbench/config/parts.json（mm）と同じもの（config/parts.json にコピー）。
部品の座標は synthetic-testbench の USD と同じ決め方にした:
  - 中心が原点、軸が z。ボルトは頭が +z。ナットは x の向きの幅が二面幅（平らな面が ±x）
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .geometry import make_T, rot_y, rot_z, unit
from .kinematics import Arm, ArmGeometry

MM = 0.001

# ---- セルの寸法（URDF と同じ） ----
TABLE_TOP_Z = 0.75
TABLE_CENTER = (0.55, 0.0)
TABLE_SIZE = (0.70, 1.00, 0.04)          # x, y, 厚み
TRAY_CENTER = (0.44, 0.0)
TRAY_SIZE = (0.21, 0.21, 0.02)
TRAY_TOP_Z = TABLE_TOP_Z + TRAY_SIZE[2]
SLOT_PITCH = 0.058
TORSO_BOX = ((-0.19, -0.20, 0.0), (0.07, 0.20, 0.95))     # 胴体（最小の角, 最大の角）
SHOULDER_BOX = ((-0.06, -0.28, 0.95), (0.10, 0.28, 1.05))  # 肩の台
# 照明（柱・腕・ランプ）。腕と当たらないかも確かめる
LIGHT_BOXES = (((0.605, 0.455, 0.75), (0.635, 0.485, 1.70)),
               ((0.605, 0.175, 1.70), (0.635, 0.485, 1.73)),
               ((0.54, 0.12, 1.68), (0.70, 0.28, 1.70)))
ARM_BASES = {"l": (0.06, 0.21, 1.05), "r": (0.06, -0.21, 1.05)}
LEFT_TOOL = 0.085    # 左: j6 → 指のあいだの TCP（指先より 2.5cm 内側）
RIGHT_TOOL = 0.05    # 右: j6 → カメラの光学中心（r_camera_optical_frame）
FINGER_MAX = 0.025   # 指 1 本の開き（m）。両側で最大 5cm

# ---- 検査の段取り ----
INSPECT_POINT = np.array([0.40, 0.03, 1.02])   # 左手が部品を見せる点（TCP）
VIEW_DIR = unit([0.30, -1.0, 0.35])            # 見せる面が向く向き（右腕のカメラのほう）
STANDOFF = 0.15                                # 面からカメラまで
TRAY_STANDOFF = 0.15
LIFT = 0.08                                    # 取る・置くときに真上へ逃げる高さ

# ---- 当たりの確認 ----
# 腕を「カプセル（太さのある線分）」で近似する。半径は URDF の円柱より少し大きめ
CAPSULE_R = {"upper": 0.050, "fore": 0.045, "hand_l": 0.045, "hand_r": 0.035}
HELD_PART_R = 0.03         # 持っている部品を球で近似
ARM_MARGIN = 0.03          # 腕どうしのすき間の下限
ENV_MARGIN = 0.02          # 腕と机・トレイ・胴体のすき間の下限


def make_arms() -> dict[str, Arm]:
    return {
        "l": Arm(ArmGeometry("l", ARM_BASES["l"], LEFT_TOOL)),
        "r": Arm(ArmGeometry("r", ARM_BASES["r"], RIGHT_TOOL)),
    }


@dataclass(frozen=True)
class Face:
    name: str
    normal: tuple[float, float, float]   # 部品の座標での外向きの法線
    center: tuple[float, float, float]   # 部品の座標での面の中心（カメラが狙う点）
    face_type: str                       # 疑似カメラの絵の種類（imaging.py）
    where: str                           # "tray"（置いたまま上から）/ "hand"（左手で持って）


@dataclass
class Part:
    name: str
    kind: str
    dims: dict          # mm（parts.json のまま）
    finish: str = "zinc"
    slot: int = 0
    faces: list[Face] = field(default_factory=list)

    # ---- 大きさ ----
    @property
    def grip_half_width(self) -> float:
        """指 1 本の位置（閉じたとき）。ボルトは軸、ナットは二面幅、ワッシャーは外径をはさむ。"""
        d = self.dims
        w = {"bolt": d.get("d", 0), "nut": d.get("across_flats", 0), "washer": d.get("d_outer", 0)}[self.kind]
        return w * MM / 2.0

    @property
    def open_half_width(self) -> float:
        return min(self.grip_half_width + 0.008, FINGER_MAX)

    def extent(self) -> tuple[float, float]:
        """(軸に垂直な方向の最大の半径, 軸の方向の全長)（m）。"""
        d = self.dims
        if self.kind == "bolt":
            return d["head_across_flats"] / math.sqrt(3) * MM, (d["length"] + d["head_height"]) * MM
        if self.kind == "nut":
            return d["across_flats"] / math.sqrt(3) * MM, d["height"] * MM
        return d["d_outer"] / 2 * MM, d["thickness"] * MM

    # ---- トレイに置いたときの姿勢 ----
    def tray_pose(self) -> np.ndarray:
        """world → 部品。ボルトは寝かせて（頭が +y、部品の +x が上）、ナット・ワッシャーは平らに置く。"""
        row, col = divmod(self.slot, 3)
        x = TRAY_CENTER[0] + (row - 1) * SLOT_PITCH
        y = TRAY_CENTER[1] + (1 - col) * SLOT_PITCH
        d = self.dims
        if self.kind == "bolt":
            R = np.column_stack([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # 列 = 部品の x, y, z
            z = TRAY_TOP_Z + d["head_across_flats"] / 2 * MM  # 頭の平らな面で V 溝に乗る
            return make_T(R, [x, y, z])
        h = d["height"] if self.kind == "nut" else d["thickness"]
        return make_T(np.eye(3), [x, y, TRAY_TOP_Z + h / 2 * MM])

    # ---- 持ち方 ----
    def grip_point(self) -> np.ndarray:
        """指ではさむ点（部品の座標）。ボルトは軸の真ん中、ナット・ワッシャーは中心。"""
        if self.kind == "bolt":
            d = self.dims
            total = (d["length"] + d["head_height"]) * MM
            z_head = -total / 2 + d["length"] * MM
            return np.array([0.0, 0.0, z_head - d["length"] * MM / 2])
        return np.zeros(3)

    def grasp_variants(self) -> list[np.ndarray]:
        """持ち方の候補: TCP の座標で見た部品の向き（3x3）。どれでも同じ所をはさめる。

        TCP の座標: z = 指が伸びる向き、y = 指が閉じる向き。手のひらは -z の側。
        - ボルト: 部品の z（軸）が TCP の x、部品の +x（トレイで上を向く側）が手のひら（-z）
        - ナット・ワッシャー: 部品の z が -z（手のひらのほう）、部品の x（ナットの平らな面）を指ではさむ
        """
        if self.kind == "bolt":
            base = rot_y(math.pi / 2)
            return [base, rot_z(math.pi) @ base]
        base = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        steps = 2 if self.kind == "nut" else 4  # ナットは指が平らな面に当たる 2 通りだけ
        return [base @ rot_z(2 * math.pi * k / steps) for k in range(steps)]

    def tool_to_part(self, variant: int) -> np.ndarray:
        """TCP → 部品の同次変換。"""
        R = self.grasp_variants()[variant]
        return make_T(R, -R @ self.grip_point())


def _faces(kind: str, d: dict) -> list[Face]:
    """撮る面。トレイでは上の面を上から、残りは左手で持って見せる（手のひら側の面は持ったままでは見えない）。"""
    if kind == "bolt":
        total = (d["length"] + d["head_height"]) * MM
        half_af = d["head_across_flats"] / 2 * MM
        return [
            Face("side_top", (1, 0, 0), (half_af, 0, 0), "bolt_profile", "tray"),
            Face("head_end", (0, 0, 1), (0, 0, total / 2), "hex_end", "hand"),
            Face("tip_end", (0, 0, -1), (0, 0, -total / 2), "circle_end", "hand"),
            Face("side_bottom", (-1, 0, 0), (-half_af, 0, 0), "bolt_profile", "hand"),
        ]
    if kind == "nut":
        h = d["height"] * MM
        corner = d["across_flats"] / math.sqrt(3) * MM
        return [
            Face("top", (0, 0, 1), (0, 0, h / 2), "hex_ring", "tray"),
            Face("bottom", (0, 0, -1), (0, 0, -h / 2), "hex_ring", "hand"),
            Face("side", (0, 1, 0), (0, corner, 0), "nut_profile", "hand"),
        ]
    t = d["thickness"] * MM
    return [
        Face("top", (0, 0, 1), (0, 0, t / 2), "ring", "tray"),
        Face("bottom", (0, 0, -1), (0, 0, -t / 2), "ring", "hand"),
    ]


def default_parts_file() -> Path:
    """config/parts.json の場所。インストール後は share/、ソースから使うときはパッケージの隣。"""
    here = Path(__file__).resolve().parent
    src = here.parent / "config" / "parts.json"
    if src.exists():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("dual_arm_inspection")) / "config" / "parts.json"
    except Exception:  # noqa: BLE001
        return src


def load_parts(path: str | Path | None = None) -> tuple[list[Part], dict]:
    spec = json.loads(Path(path or default_parts_file()).read_text(encoding="utf-8"))
    parts = []
    for i, p in enumerate(spec["parts"]):
        dims = {k: v for k, v in p.items() if k not in ("name", "kind", "finish")}
        parts.append(Part(p["name"], p["kind"], dims, p.get("finish", "zinc"), slot=i, faces=_faces(p["kind"], dims)))
    return parts, spec
