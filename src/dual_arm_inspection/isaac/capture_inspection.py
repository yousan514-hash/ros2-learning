"""【下書き・まだ動かしていない】Isaac Sim 6.2 で双腕セルを読み込み、右手首の Replicator カメラで検査の画像を撮る。

Isaac Sim の Python で動かす（pixi の環境ではない）。初回は藤田さんが Isaac Sim を一度起動して、
ライセンス（EULA）に自分で同意してから使う。このスクリプトは OMNI_KIT_ACCEPT_EULA を設定しない。

    C:\\isaacsim\\python.bat src\\dual_arm_inspection\\isaac\\capture_inspection.py --max-shots 3
    C:\\isaacsim\\python.bat src\\dual_arm_inspection\\isaac\\capture_inspection.py --good-only --repeat 6 --out ...\\good
    C:\\isaacsim\\python.bat src\\dual_arm_inspection\\isaac\\capture_inspection.py --defects 0.5 --out ...\\test

前もって pixi の環境で `pixi run dual_arm_urdf` を実行し、isaac/out/ に dual_arm.urdf と inspection_plan.json を作っておく。

流れ:
  1. URDF を USD に変換する（isaacsim.asset.importer.urdf の URDFImporter。根元は固定、固定の関節はまとめない
     → r_camera_optical_frame のリンクが USD に残るので、そこにカメラを付けられる）
  2. 新しいステージに、ロボット・机・トレイ・部品（part_shapes.py と同じ三角形）・照明を置く
  3. 右手首の r_camera_optical_frame の下に USD のカメラを作る（ROS の光学座標 z 前・y 下 → USD のカメラ -z 前・y 上
     なので x 軸まわりに 180 度回す）。リング照明として、同じ向きの円盤の光をカメラに付ける
  4. 撮影の一覧（ROS 2 の cell_node と同じ計画）の関節角を順に入れ、部品を置き、Replicator で撮る
  5. 欠陥（打痕・傷・汚れ）は、面の上に小さな形を置いて作る（--defects の確率）。意味のラベルを付けて、
     外接する四角（bounding_box_2d_tight）も正解として保存する
  6. 画像（PNG）と shots.json を書き出す。判定は pixi の環境で judge_folder.py が行う（OpenCV の検査と同じ）

GPU は RTX 4070（12GB）で公式の最低要件（16GB）に届かないので、画面なし（headless）・640x480・
物理の当たり判定なし（関節角を直接入れる）・部品は単純な形、で軽くしている。
API の名前は C:\\isaacsim の 6.2.0 の standalone_examples と skills/ を読んで合わせたが、実行はしていない。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent

parser = argparse.ArgumentParser(description="双腕セルの検査画像を Isaac Sim で撮る（下書き）")
parser.add_argument("--plan", default=str(HERE / "out" / "inspection_plan.json"))
parser.add_argument("--urdf", default=str(HERE / "out" / "dual_arm.urdf"))
parser.add_argument("--usd-dir", default=str(HERE / "out" / "usd"), help="URDF から作った USD を置く場所")
parser.add_argument("--out", default=str(HERE / "out" / "isaac_shots"))
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--max-shots", type=int, default=0, help="0 なら全部（27 回）")
parser.add_argument("--repeat", type=int, default=1, help="撮影の一覧を何周するか（良品を集めるとき用）")
parser.add_argument("--defects", type=float, default=0.3, help="欠陥を置く確率")
parser.add_argument("--good-only", action="store_true", help="欠陥を置かない（良品だけで学ぶ用）")
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--ring-intensity", type=float, default=3000.0, help="リング照明の強さ（明るすぎ・暗すぎなら調整）")
parser.add_argument("--gui", action="store_true", help="画面を出す（重いので最初は出さない）")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})

# ここから下の import は SimulationApp を作ったあとでないと読めない
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
for name in ("omni.scene.optimizer.core", "isaacsim.robot.schema"):  # urdf_import.py の例と同じ
    ext.set_extension_enabled_immediate(name, True)

import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402

sys.path.insert(0, str(PKG))
from dual_arm_inspection.part_shapes import part_triangles  # noqa: E402  numpy だけで書いてある


# ---------------------------------------------------------------- 道具
def matrix_from_list(m) -> Gf.Matrix4d:
    """numpy の並び（列ベクトルの 4x4）→ USD の Matrix4d（行ベクトルなので転置する）。"""
    a = np.asarray(m, dtype=float)
    return Gf.Matrix4d(*a.T.flatten().tolist())


def set_world_matrix(prim, m):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(matrix_from_list(m))


def preview_material(stage, path: str, color, metallic=0.0, roughness=0.5, opacity=1.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def add_box(stage, path, center, size, mat):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(cube.GetPrim(), mat)
    return cube


def add_mesh(stage, path, tris: np.ndarray, mat):
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = tris.reshape(-1, 3)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(len(pts)))))
    mesh.CreateSubdivisionSchemeAttr("none")
    bind(mesh.GetPrim(), mat)
    return mesh


def find_prim(stage, root: str, name: str):
    for prim in stage.Traverse():
        if prim.GetName() == name and str(prim.GetPath()).startswith(root):
            return prim
    return None


def add_semantic(prim, label: str):
    """欠陥に意味のラベルを付ける（外接する四角の正解を Replicator に出させるため）。"""
    try:
        rep.functional.modify.semantics(prim, {"class": label}, mode="add")  # sdg_getting_started_01.py と同じ
    except Exception as exc:  # noqa: BLE001  API の違いはここで気づけるように
        print(f"[warn] semantics を付けられなかった: {exc!r}")


# ---------------------------------------------------------------- 1. URDF → USD
def import_robot() -> str:
    cfg = URDFImporterConfig()
    cfg.urdf_path = os.path.abspath(args.urdf)
    cfg.usd_path = os.path.abspath(args.usd_dir)
    cfg.fix_base = True
    cfg.merge_fixed_joints = False      # l_tcp・r_camera_optical_frame を残す
    cfg.robot_type = "Manipulator"
    cfg.joint_drive_type = "force"
    cfg.joint_target_type = "position"
    cfg.override_joint_stiffness = 5000.0
    cfg.override_joint_damping = 200.0
    cfg.collision_from_visuals = False  # 当たり判定は使わない（関節角を直接入れる）
    usd = URDFImporter(cfg).import_urdf()
    if not usd:
        raise RuntimeError("URDF の変換に失敗した")
    print(f"robot USD: {usd}")
    return usd


# ---------------------------------------------------------------- 2〜3. ステージ
def build_stage(robot_usd: str, plan: dict):
    stage_utils.create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    rep.orchestrator.set_capture_on_play(False)
    UsdGeom.Xform.Define(stage, "/World")
    stage_utils.add_reference_to_stage(usd_path=robot_usd, path="/World/dual_arm")

    cell = plan["cell"]
    wood = preview_material(stage, "/World/Looks/wood", (0.76, 0.66, 0.50), 0.0, 0.7)
    tray = preview_material(stage, "/World/Looks/tray", (0.20, 0.33, 0.45), 0.0, 0.6)
    tc, ts = cell["table_center"], cell["table_size"]
    add_box(stage, "/World/table", (tc[0], tc[1], cell["table_top_z"] - ts[2] / 2), ts, wood)
    rc, rs = cell["tray_center"], cell["tray_size"]
    add_box(stage, "/World/tray", (rc[0], rc[1], cell["table_top_z"] + rs[2] / 2), rs, tray)

    dome = UsdLux.DomeLight.Define(stage, "/World/dome")
    dome.CreateIntensityAttr(300.0)

    fin = plan["finish"]
    parts = {}
    for p in plan["parts"]:
        f = fin[p.get("finish", "zinc")]
        mat = preview_material(stage, f"/World/Looks/{p['name']}", f["color"], f["metallic"], f["roughness"])
        dims = {k: v for k, v in p.items() if k not in ("name", "kind", "finish")}
        xform = UsdGeom.Xform.Define(stage, f"/World/parts/{p['name']}")
        add_mesh(stage, f"/World/parts/{p['name']}/mesh", part_triangles(p["kind"], dims), mat)
        add_semantic(xform.GetPrim(), p["kind"])
        parts[p["name"]] = xform.GetPrim()

    # 右手首のカメラ（ロボットのリンクの下に付ける。見つからなければ計画の姿勢に置くカメラで代わりにする）
    optical = find_prim(stage, "/World/dual_arm", "r_camera_optical_frame")
    attached = optical is not None
    parent = str(optical.GetPath()) if attached else "/World/free_camera"
    if not attached:
        print("[warn] r_camera_optical_frame が USD に見つからない。計画のカメラ姿勢に置くカメラで代わりにする")
        UsdGeom.Xform.Define(stage, parent)
    cam = UsdGeom.Camera.Define(stage, parent + "/WristCamera")
    UsdGeom.Xformable(cam).AddRotateXOp().Set(180.0)  # ROS の光学座標（z 前・y 下）→ USD のカメラ（-z 前・y 上）
    # 焦点距離と撮像素子の幅は「比」だけが画角を決める（単位をそろえれば何でもよい）。水平の画角 約 24 度
    cam.CreateFocalLengthAttr(50.0)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * args.height / args.width)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
    ring = UsdLux.DiskLight.Define(stage, parent + "/RingLight")  # 円盤の光は -z に照らす = カメラと同じ向き
    UsdGeom.Xformable(ring).AddRotateXOp().Set(180.0)
    ring.CreateRadiusAttr(0.03)
    ring.CreateIntensityAttr(args.ring_intensity)
    return stage, parts, str(cam.GetPath()), attached, parent


def find_articulation_root(stage) -> str:
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and str(prim.GetPath()).startswith("/World/dual_arm"):
            return str(prim.GetPath())
    return "/World/dual_arm"


def disable_gravity(stage):
    """関節角を直接入れるので、重さで腕が垂れないように重力を切る（できなければそのまま）。"""
    try:
        from pxr import PhysxSchema
    except ImportError:
        return
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()).startswith("/World/dual_arm"):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)


# ---------------------------------------------------------------- 5. 欠陥
def place_defect(stage, path, kind, center, normal, rng):
    """面の中心の近くに、小さな形を置く（形の z を面の法線に合わせ、面から少し浮かせる）。"""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    helper = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    p = np.asarray(center) + (rng.uniform(-1.5, 1.5) * u + rng.uniform(-1.5, 1.5) * v) * 1e-3 + n * 0.00005
    scale = np.ones(3)
    if kind == "dent":
        g = UsdGeom.Sphere.Define(stage, path)
        r = rng.uniform(0.0006, 0.0012)
        g.CreateRadiusAttr(r)
        p = p - n * r * 0.6  # 半分くらい埋める
        mat = preview_material(stage, path + "_mat", (0.18, 0.18, 0.19), 0.8, 0.6)
    elif kind == "scratch":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        scale = np.array([rng.uniform(0.003, 0.008), 0.0002, 0.00005])  # 長さ 3〜8mm・幅 0.2mm
        mat = preview_material(stage, path + "_mat", (0.30, 0.30, 0.32), 0.9, 0.2)
    else:  # stain
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(rng.uniform(0.0015, 0.003))
        g.CreateHeightAttr(0.00002)
        mat = preview_material(stage, path + "_mat", (0.35, 0.28, 0.20), 0.0, 0.9)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, v, n]) @ np.diag(scale)
    T[:3, 3] = p
    set_world_matrix(g.GetPrim(), T)
    bind(g.GetPrim(), mat)
    add_semantic(g.GetPrim(), kind)
    return str(g.GetPath())


# ---------------------------------------------------------------- 4・6. 撮る
def get_annotator(name: str):
    try:
        return rep.AnnotatorRegistry.get_annotator(name)
    except AttributeError:
        return rep.annotators.get(name)


def save_png(path: Path, rgba):
    img = np.asarray(rgba)[:, :, :3].astype(np.uint8)
    try:
        from PIL import Image

        Image.fromarray(img).save(path)
    except ImportError:
        np.save(path.with_suffix(".npy"), img)


def main():
    plan = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    robot_usd = import_robot()
    stage, parts, cam_path, attached, cam_parent = build_stage(robot_usd, plan)
    disable_gravity(stage)

    rp = rep.create.render_product(cam_path, (args.width, args.height))
    rgb = get_annotator("rgb")
    rgb.attach([rp])
    bbox = get_annotator("bounding_box_2d_tight")
    bbox.attach([rp])

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    robot = Articulation(find_articulation_root(stage))
    names = plan["joint_names"]
    dof_index = [robot.dof_names.index(n) for n in names]
    print(f"articulation: {robot.dof_names}")

    shots = plan["shots"] * args.repeat
    if args.max_shots:
        shots = shots[: args.max_shots]
    records = []
    for k, shot in enumerate(shots):
        q = np.zeros((1, robot.num_dofs), dtype=np.float32)
        q[0, dof_index] = shot["q"]
        robot.set_dof_positions(q)
        set_world_matrix(parts[shot["part"]], shot["part_world"])
        for name, m in shot["other_parts_world"].items():
            set_world_matrix(parts[name], m)
        if not attached:
            set_world_matrix(stage.GetPrimAtPath(cam_parent), shot["camera_world"])

        defects = []
        if not args.good_only and rng.random() < args.defects:
            kind = str(rng.choice(["dent", "scratch", "stain"]))
            path = place_defect(stage, f"/World/defects/d{k}", kind, shot["face_center_world"],
                                shot["face_normal_world"], rng)
            defects.append({"type": kind, "prim": path})
        for _ in range(3):
            simulation_app.update()

        # 自作の運動学（計画）と Isaac Sim のカメラの位置の差。URDF の読み込みが正しいかの確かめ
        cam_err = None
        if attached:
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_parent)).ComputeLocalToWorldTransform(0)
            cam_err = float(np.linalg.norm(np.array(xf.ExtractTranslation()) - np.array(shot["camera_world"])[:3, 3]))

        rep.orchestrator.step(rt_subframes=8)
        name = f"{k:04d}_{shot['part']}_{shot['face']}.png"
        save_png(out / name, rgb.get_data())
        box = bbox.get_data()
        records.append({"file": name, "seq": shot["seq"], "part": shot["part"], "face": shot["face"],
                        "face_type": shot["face_type"], "where": shot["where"], "good": not defects,
                        "defects": defects, "bbox": np.asarray(box["data"]).tolist() if box is not None else None,
                        "bbox_labels": box.get("info", {}).get("idToLabels") if isinstance(box, dict) else None,
                        "camera_position_error_m": cam_err})
        print(f"{name}: defects={[d['type'] for d in defects]} camera_err={cam_err}")
        for d in defects:
            stage.RemovePrim(d["prim"])
            stage.RemovePrim(d["prim"] + "_mat")

    rep.orchestrator.wait_until_complete()
    (out / "shots.json").write_text(json.dumps({"shots": records, "args": vars(args)}, ensure_ascii=False, indent=1),
                                    encoding="utf-8")
    print(f"saved {len(records)} shots to {out}")
    timeline.stop()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
