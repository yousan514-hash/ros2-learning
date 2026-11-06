"""Isaac Sim に渡すファイルを作る（pixi の ROS 2 環境で動かす。Isaac Sim は使わない）。

    pixi run dual_arm_urdf

作るもの（isaac/out/、Git には入れない）:
  dual_arm.urdf          机・トレイ・照明を外したロボットだけの URDF（Isaac Sim の URDF 読み込みは xacro を読めない）
  inspection_plan.json   撮影 27 回ぶんの「関節角・部品の姿勢・面の中心と向き・カメラの姿勢」と、セルと部品の寸法

関節角は ROS 2 の cell_node と同じ計画（planner.py）から取るので、RViz で見た動きと同じ姿勢で撮れる。
姿勢は自作の運動学で計算した world 座標（URDF と同じことを test_urdf.py で確かめてある）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG))

from dual_arm_inspection import cell as C  # noqa: E402
from dual_arm_inspection import planner as P  # noqa: E402
from dual_arm_inspection.geometry import make_T  # noqa: E402


def export_urdf(out: Path) -> Path:
    import xacro

    doc = xacro.process_file(str(PKG / "urdf" / "dual_arm_cell.urdf.xacro"), mappings={"with_cell": "false"})

    def strip_comments(node):  # 日本語のコメントや手元のパスを、読み込む側に渡さない
        for child in list(node.childNodes):
            if child.nodeType == child.COMMENT_NODE:
                node.removeChild(child)
            else:
                strip_comments(child)

    strip_comments(doc)
    path = out / "dual_arm.urdf"
    path.write_text(doc.toprettyxml(indent="  "), encoding="utf-8")
    return path


def export_plan(out: Path) -> Path:
    plan = P.plan_cycle()
    tl = P.Timeline(plan.segments)
    arms = plan.arms
    names = arms["l"].geo.joint_names + arms["r"].geo.joint_names + ["l_finger_a_joint", "l_finger_b_joint"]
    shots = []
    for i, seg in enumerate(plan.segments):
        for kind, data in seg.events:
            if kind != "capture":
                continue
            st, _ = tl.sample(float(tl.starts[i]))  # 止まって撮っている区間の始め
            part = plan.parts[data["part"]]
            if st.held == part.name:
                Rl, pl = arms["l"].fk(st.q["l"])
                T_wp = make_T(Rl, pl) @ part.tool_to_part(st.variant)
            else:
                T_wp = part.tray_pose()
            face = next(f for f in part.faces if f.name == data["face"])
            Rc, pc = arms["r"].fk(st.q["r"])
            shots.append({
                "seq": len(shots) + 1, **data,
                "q": [float(v) for v in np.concatenate([st.q["l"], st.q["r"]])] + [st.grip, st.grip],
                "part_world": T_wp.tolist(),
                "face_center_world": (T_wp @ np.append(face.center, 1.0))[:3].tolist(),
                "face_normal_world": (T_wp[:3, :3] @ np.asarray(face.normal, float)).tolist(),
                "camera_world": make_T(Rc, pc).tolist(),
                "other_parts_world": {p.name: p.tray_pose().tolist() for p in plan.parts.values()
                                      if p.name != part.name},
            })
    spec = json.loads(C.default_parts_file().read_text(encoding="utf-8"))
    doc = {
        "note": "planner.py の計画から作った撮影の一覧。姿勢は world（m）、4x4 の同次変換。q は joint_names の順",
        "joint_names": names,
        "camera": {"frame": "r_camera_optical_frame", "standoff_m": C.STANDOFF, "convention": "ROS optical (z forward, y down)"},
        "cell": {"table_top_z": C.TABLE_TOP_Z, "table_center": C.TABLE_CENTER, "table_size": C.TABLE_SIZE,
                 "tray_center": C.TRAY_CENTER, "tray_size": C.TRAY_SIZE},
        "parts": spec["parts"], "finish": spec["finish"],
        "shots": shots,
    }
    path = out / "inspection_plan.json"
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding="utf-8")
    return path


def main() -> int:
    out = HERE / "out"
    out.mkdir(exist_ok=True)
    print(f"URDF: {export_urdf(out)}")
    path = export_plan(out)
    n = len(json.loads(path.read_text(encoding='utf-8'))["shots"])
    print(f"撮影の一覧: {path}（{n} 回）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
