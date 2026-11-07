r"""Isaac Sim 6.2 で双腕セルを読み込み、右手首の Replicator カメラで検査の画像を撮る（2026-11-07 に動いた）。

Isaac Sim の Python で動かす（pixi の環境ではない）。手順と結果は同じフォルダの README.md。

    C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --no-ros-env --max-shots 3 --good-only --width 480 --height 360 --out src\dual_arm_inspection\isaac\out\try
    C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --no-ros-env --width 480 --height 360 --good-out ...\good --good-repeat 30 --test-out ...\test --test-repeat 6

前もって pixi の環境で `pixi run dual_arm_urdf` を実行し、isaac/out/ に dual_arm.urdf と inspection_plan.json を作っておく。
このスクリプトは OMNI_KIT_ACCEPT_EULA を設定しない。

流れ:
  1. URDF を USD に変換する（URDFImporter。根元は固定、固定の関節はまとめない → r_camera_optical_frame が残る）。
     URDF より新しい USD があれば使い回す。物理は USD の "Physics" の切り替えで "physx" を選ぶ
  2. 新しいステージに、ロボット・机・黒っぽいトレイ・部品（part_shapes.py と同じ三角形）・照明・黒い背景板を置く
  3. 右手首の r_camera_optical_frame の下に USD のカメラを作る（ROS の光学座標 z 前・y 下 → USD のカメラ -z 前・y 上
     なので x 軸まわりに 180 度）。リング照明として、同じ向きの円盤の光を付ける
  4. 撮影の一覧（ROS 2 の cell_node と同じ計画）の関節角を順に入れ、部品を置き、倍率を合わせて Replicator で撮る
  5. 欠陥（打痕・傷・汚れ）は、最初に作っておいた小さな形を、材料のある所に動かして見せる。範囲外の照明もときどき入れる
  6. 画像（PNG）と shots.json を書き出す。判定は pixi の環境で judge_folder.py が行う（OpenCV の検査と同じ）

RTX 4070（12GB）は公式の最低要件（16GB）に届かないが、画面なし・480x360・RaytracedLighting・FXAA で、
GPU のメモリはパソコン全体で約 3.1GB だった。
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

parser = argparse.ArgumentParser(description="双腕セルの検査画像を Isaac Sim で撮る")
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
parser.add_argument("--ring-intensity", type=float, default=1500.0, help="リング照明の強さ（3000 だと平らな面が 220 前後で頭打ちに近かった）")
parser.add_argument("--gui", action="store_true", help="画面を出す（重いので最初は出さない）")
parser.add_argument("--renderer", default="RaytracedLighting", help="RaytracedLighting（軽い）/ RealTimePathTracing")
parser.add_argument("--fill", type=float, default=0.7, help="部品が画面の高さの何割に写るように倍率を合わせるか")
parser.add_argument("--good-out", default="", help="これを渡すと 1 回の起動で良品の組と欠陥ありの組を続けて撮る")
parser.add_argument("--good-repeat", type=int, default=5, help="良品の組: 撮影の一覧を何周するか")
parser.add_argument("--test-out", default="", help="欠陥ありの組を書き出す場所（--good-out なしなら欠陥ありの組だけ撮る）")
parser.add_argument("--test-repeat", type=int, default=3, help="欠陥ありの組: 撮影の一覧を何周するか")
parser.add_argument("--p-ood", type=float, default=0.05, help="欠陥ありの組で、範囲外の照明（暗すぎ・明るすぎ）にする確率")
parser.add_argument("--no-jitter", action="store_true", help="照明の強さと部品の置き方のばらつきを入れない")
parser.add_argument("--reimport", action="store_true", help="URDF から USD を作り直す（既定では、URDF より新しい USD があれば使い回す）")
# 2026-11-08: 組の一覧（JSON）で、学ぶ用・しきい値を決める用・確かめる用などを 1 回の起動でまとめて撮る（scope_sets.json）
parser.add_argument("--sets", default="", help="組の一覧の JSON（これを渡すと下の組の撮り方になる。欠陥は defect_mesh.py の濃さつき）")
parser.add_argument("--sets-root", default=str(HERE / "out" / "scope"), help="組ごとのフォルダを作る場所")
parser.add_argument("--only", default="", help="撮る組の名前（カンマ区切り。空なら全部）")
parser.add_argument("--contrast-gamma", type=float, default=0.0, help="濃さ → 材質の色の換算の gamma（0 なら JSON の値）")
args, _ = parser.parse_known_args()

import time  # noqa: E402
import traceback  # noqa: E402

T0 = time.monotonic()
from isaacsim import SimulationApp  # noqa: E402

# 既定の DLSS は描画を半分の解像度にしてから引き伸ばす（480x360 → 240x180 で描いていた）。
# 検査の画像なので、引き伸ばさない FXAA（anti_aliasing=2）にし、描き方も軽い RaytracedLighting にする
simulation_app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720,
                                "renderer": args.renderer, "anti_aliasing": 2})
print(f"[time] SimulationApp ready: {time.monotonic() - T0:.1f} s", flush=True)

# ここから下の import は SimulationApp を作ったあとでないと読めない
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
for name in ("omni.scene.optimizer.core", "isaacsim.robot.schema"):  # urdf_import.py の例と同じ
    ext.set_extension_enabled_immediate(name, True)

import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402

sys.path.insert(0, str(PKG))
sys.path.insert(0, str(HERE))
import defect_mesh as DM  # noqa: E402  numpy だけで書いてある（濃さと大きさを決めた欠陥の形）
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
    # 毎回変換すると dual_arm_1, dual_arm_2 ... と増えるので、URDF より新しい USD があれば使い回す
    existing = Path(args.usd_dir) / "dual_arm" / "dual_arm.usda"
    if not args.reimport and existing.exists() and existing.stat().st_mtime > Path(args.urdf).stat().st_mtime:
        print(f"robot USD (reuse): {existing}", flush=True)
        return str(existing)
    if existing.exists():
        import shutil

        shutil.rmtree(existing.parent, ignore_errors=True)  # 自分で作った out/usd/dual_arm だけを消す
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
    # 変換した USD は物理を "Physics" の切り替え（variant）で持っていて、何も選ばないと物理なし（関節も動かせない）。
    # PhysX の版を選ぶ（Franka の例で variants=[...] を渡しているのと同じやり方）
    stage_utils.add_reference_to_stage(usd_path=robot_usd, path="/World/dual_arm", variants=[("Physics", "physx")])

    cell = plan["cell"]
    wood = preview_material(stage, "/World/Looks/wood", (0.76, 0.66, 0.50), 0.0, 0.7)
    # トレイと背景板は黒っぽくする（撮像の標準化: 背景を暗くそろえると、部品を背景から切り出しやすい）
    tray = preview_material(stage, "/World/Looks/tray", (0.04, 0.045, 0.05), 0.0, 0.9)
    black = preview_material(stage, "/World/Looks/backdrop", (0.02, 0.02, 0.02), 0.0, 1.0)
    backdrop = add_box(stage, "/World/backdrop", (0.0, 0.0, -5.0), (0.16, 0.16, 0.002), black)
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
    return stage, parts, cam, attached, parent, backdrop.GetPrim()


def visible_extent_mm(part: dict, face_type: str) -> float:
    """写る部品のいちばん長い所（mm）。倍率をレシピのように部品ごとに合わせるのに使う（疑似カメラと同じ考え）。"""
    if part["kind"] == "bolt":
        return {"bolt_profile": part["length"] + part["head_height"], "hex_end": 2 * part["head_across_flats"] / 3 ** 0.5,
                "circle_end": part["d"]}[face_type]
    if part["kind"] == "nut":
        return {"hex_ring": 2 * part["across_flats"] / 3 ** 0.5,
                "nut_profile": max(part["height"], part["across_flats"])}[face_type]
    return part["d_outer"]


def zoom(cam, part: dict, face_type: str, distance_m: float, extent_mm: float | None = None):
    """部品が画面の高さの fill 割に写る焦点距離にする（ピンホール: 写る高さ = 撮像素子の高さ x 距離 / 焦点距離）。"""
    field_mm = (extent_mm or visible_extent_mm(part, face_type)) / args.fill
    vertical_aperture = 20.955 * args.height / args.width
    cam.GetFocalLengthAttr().Set(float(vertical_aperture * distance_m * 1000.0 / field_mm))


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
def make_defect_pool(stage) -> dict:
    """欠陥の形を、物理を動かす前に 1 種類ずつ作っておく（見えない状態）。

    動いている途中で prim を足したり消したりすると、関節を動かす仕組み（Articulation）が効かなくなり、
    腕が止まったままになった。そこで、作るのは最初の 1 回だけにし、撮るときは位置・大きさ・見える/見えないだけを変える。
    """
    pool = {}
    shapes = {"dent": (UsdGeom.Sphere, (0.18, 0.18, 0.19), 0.8, 0.6),
              "scratch": (UsdGeom.Cube, (0.30, 0.30, 0.32), 0.9, 0.2),
              "stain": (UsdGeom.Cylinder, (0.35, 0.28, 0.20), 0.0, 0.9)}
    UsdGeom.Xform.Define(stage, "/World/defects")
    for kind, (cls, color, metallic, rough) in shapes.items():
        g = cls.Define(stage, f"/World/defects/{kind}")
        if kind == "dent":
            g.CreateRadiusAttr(1.0)
        elif kind == "scratch":
            g.CreateSizeAttr(1.0)
        else:
            g.CreateRadiusAttr(1.0)
            g.CreateHeightAttr(1.0)
        bind(g.GetPrim(), preview_material(stage, f"/World/Looks/defect_{kind}", color, metallic, rough))
        add_semantic(g.GetPrim(), kind)
        UsdGeom.Imageable(g.GetPrim()).MakeInvisible()
        pool[kind] = g.GetPrim()
    return pool


def defect_point(spec: dict, face: str, rng) -> tuple[np.ndarray, np.ndarray]:
    """欠陥を置く点と法線（部品の座標、m）。面の中心ではなく、材料のある所に置く。

    はじめは面の中心に置いていたので、ナット・座金では穴の真ん中に浮かび、ボルトの横では軸から浮いていた。
    """
    mm = 0.001
    k = spec["kind"]
    a = rng.uniform(0, 2 * np.pi)
    if k == "bolt":
        T = (spec["length"] + spec["head_height"]) * mm
        z_head = -T / 2 + spec["length"] * mm
        r = spec["d"] / 2 * mm
        if face in ("side_top", "side_bottom"):
            sx = 1.0 if face == "side_top" else -1.0
            return np.array([sx * r, 0.0, rng.uniform(-T / 2 + 1 * mm, z_head - 1 * mm)]), np.array([sx, 0.0, 0.0])
        if face == "head_end":
            rr = rng.uniform(0, 0.55) * spec["head_across_flats"] / 2 * mm
            return np.array([rr * np.cos(a), rr * np.sin(a), T / 2]), np.array([0.0, 0.0, 1.0])
        rr = rng.uniform(0, 0.55) * r
        return np.array([rr * np.cos(a), rr * np.sin(a), -T / 2]), np.array([0.0, 0.0, -1.0])
    if k == "nut":
        h, af = spec["height"] * mm, spec["across_flats"] * mm
        if face == "side":  # 角（+y）の両側の平らな面の上
            x = rng.uniform(-0.35, 0.35) * af
            return np.array([x, af / 3 ** 0.5 - abs(x) / 3 ** 0.5, rng.uniform(-0.3, 0.3) * h]), np.array([0.0, 1.0, 0.0])
        rin, rout = spec["d"] / 2 * mm, af / 2  # af はもう m
    else:
        h = spec["thickness"] * mm
        rin, rout = spec["d_inner"] / 2 * mm, spec["d_outer"] / 2 * mm
    rr = rng.uniform(rin + 0.25 * (rout - rin), rout - 0.25 * (rout - rin))  # 穴と外の縁から離す
    sz = 1.0 if face == "top" else -1.0
    return np.array([rr * np.cos(a), rr * np.sin(a), sz * h / 2]), np.array([0.0, 0.0, sz])


def place_defect(pool, kind, center, normal, rng):
    """面の中心の近くに欠陥の形を出す（形の z を面の法線に合わせ、面から少し浮かせる）。大きさは拡大率で変える。"""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    helper = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    p = np.asarray(center) + n * 0.00005
    if kind == "dent":  # 半径 0.6〜1.2mm の暗い球を、半分くらい埋める
        r = rng.uniform(0.0006, 0.0012)
        scale = np.array([r, r, r])
        p = p - n * r * 0.6
    elif kind == "scratch":  # 長さ 3〜8mm・幅 0.2mm
        scale = np.array([rng.uniform(0.003, 0.008), 0.0002, 0.00005])
    else:  # 汚れ: 半径 1.5〜3mm の薄い円盤
        r = rng.uniform(0.0015, 0.003)
        scale = np.array([r, r, 0.00002])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, v, n]) @ np.diag(scale)
    T[:3, 3] = p
    prim = pool[kind]
    set_world_matrix(prim, T)
    UsdGeom.Imageable(prim).MakeVisible()
    return prim


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


def place_backdrop(prim, shot):
    """持って見せる面の後ろ 6cm に黒い板を置く（トレイの面を撮るときは遠くへどける）。"""
    if shot["where"] != "hand":
        set_world_matrix(prim, np.diag([0.16, 0.16, 0.002, 1.0]) + np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, -5.0],
                                                                            [0, 0, 0, 0]]))
        return
    n = np.asarray(shot["face_normal_world"], float)
    n /= np.linalg.norm(n)
    helper = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, np.cross(n, u), n]) @ np.diag([0.16, 0.16, 0.002])
    T[:3, 3] = np.asarray(shot["face_center_world"]) - n * 0.06
    set_world_matrix(prim, T)


def world_position(path: str, stage):
    """prim の world の位置。物理で動くリンクは Fabric にしか書かれないことがあるので、XformPrim で読む。"""
    try:
        from isaacsim.core.experimental.prims import XformPrim

        pos, _ = XformPrim(path).get_world_poses()
        return np.asarray(pos.numpy() if hasattr(pos, "numpy") else pos, float).reshape(-1, 3)[0]
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] XformPrim で読めなかった（USD から読む）: {exc!r}", flush=True)
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(xf.ExtractTranslation(), float)


def jitter_pose(m, rng):
    """部品の置き方のばらつき: 軸のまわりに ±3 度、位置を ±0.5mm（疑似カメラの位置・向きのずれに当たる）。"""
    T = np.asarray(m, float).copy()
    a = np.radians(rng.uniform(-3, 3))
    rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    T[:3, :3] = T[:3, :3] @ rz
    T[:3, 3] += rng.uniform(-0.0005, 0.0005, 3)
    return T


def capture_set(ctx, shots, out: Path, good_only: bool, defect_p: float, p_ood: float, rng):
    """撮影の一覧を順に撮り、PNG と shots.json を out に書く。"""
    out.mkdir(parents=True, exist_ok=True)
    stage, robot, cam, ring = ctx["stage"], ctx["robot"], ctx["cam"], ctx["ring"]
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():  # 物理が止まっていると、関節角を入れても腕が動かない
        timeline.play()
        simulation_app.update()
    records = []
    for k, shot in enumerate(shots):
        q = np.zeros((1, robot.num_dofs), dtype=np.float32)
        q[0, ctx["dof_index"]] = shot["q"]
        robot.set_dof_positions(q)
        pose = shot["part_world"] if args.no_jitter else jitter_pose(shot["part_world"], rng)
        set_world_matrix(ctx["parts"][shot["part"]], pose)
        for name, m in shot["other_parts_world"].items():
            set_world_matrix(ctx["parts"][name], m)
        if not ctx["attached"]:
            set_world_matrix(stage.GetPrimAtPath(ctx["cam_parent"]), shot["camera_world"])
        place_backdrop(ctx["backdrop"], shot)
        distance = float(np.linalg.norm(np.asarray(shot["camera_world"])[:3, 3] - np.asarray(shot["face_center_world"])))
        zoom(cam, ctx["part_spec"][shot["part"]], shot["face_type"], distance)

        gain, ood = 1.0, None
        if not args.no_jitter:
            gain = float(rng.uniform(0.9, 1.1))
        if rng.random() < p_ood:  # 照明の劣化・外からの光の代わり
            ood = str(rng.choice(["dim", "bright"]))
            gain = float(rng.uniform(0.40, 0.55) if ood == "dim" else rng.uniform(1.6, 1.9))
        ring.GetIntensityAttr().Set(args.ring_intensity * gain)

        defects = []
        if not good_only and rng.random() < defect_p:
            kind = str(rng.choice(["dent", "scratch", "stain"]))
            pt, nrm = defect_point(ctx["part_spec"][shot["part"]], shot["face"], rng)
            pose_a = np.asarray(pose, float)
            prim = place_defect(ctx["defect_pool"], kind, pose_a[:3, :3] @ pt + pose_a[:3, 3], pose_a[:3, :3] @ nrm, rng)
            defects.append({"type": kind, "prim": prim})
        for _ in range(6):  # 3 回では、関節角を入れたあとの位置の読み出しがまだ前の姿勢のことがあった
            simulation_app.update()

        # 自作の運動学（計画）と Isaac Sim に読み込んだロボットのカメラの位置の差（URDF の読み込みの確かめ）。
        # 撮る前と撮ったあとの 2 回読む（読み出しが遅れていないかも見る）
        cam_err = cam_err_after = None
        target = np.asarray(shot["camera_world"])[:3, 3]
        if ctx["attached"]:
            cam_err = float(np.linalg.norm(world_position(ctx["cam_parent"], stage) - target))
        rep.orchestrator.step(rt_subframes=8)
        if ctx["attached"]:
            cam_err_after = float(np.linalg.norm(world_position(ctx["cam_parent"], stage) - target))
        name = f"{k:04d}_{shot['part']}_{shot['face']}.png"
        save_png(out / name, ctx["rgb"].get_data())
        box = ctx["bbox"].get_data()
        records.append({"file": name, "seq": shot["seq"], "part": shot["part"], "face": shot["face"],
                        "face_type": shot["face_type"], "where": shot["where"], "good": not defects and ood is None,
                        "defects": [{"type": d["type"]} for d in defects], "ood": ood, "light_gain": gain,
                        "bbox": np.asarray(box["data"]).tolist() if isinstance(box, dict) and "data" in box else None,
                        "bbox_labels": box.get("info", {}).get("idToLabels") if isinstance(box, dict) else None,
                        "camera_position_error_m": cam_err, "camera_position_error_after_m": cam_err_after})
        print(f"{out.name}/{name}: defects={[d['type'] for d in defects]} ood={ood} camera_err={cam_err} "
              f"after={cam_err_after} [time] {time.monotonic() - T0:.1f} s", flush=True)
        for d in defects:
            UsdGeom.Imageable(d["prim"]).MakeInvisible()
    # ここで rep.orchestrator.wait_until_complete() を呼ぶと Replicator がタイムラインを止め、物理が進まなくなって
    # 次の組で腕が動かなかった（画像は注釈から直接保存しているので、待つのは最後に 1 回だけにする）
    (out / "shots.json").write_text(json.dumps({"shots": records, "args": vars(args)}, ensure_ascii=False, indent=1),
                                    encoding="utf-8")
    print(f"saved {len(records)} shots to {out} [time] {time.monotonic() - T0:.1f} s", flush=True)


# ---------------------------------------------------------------- 7. 組の一覧で撮る（2026-11-08）
def _vec3_array(a):
    a = np.asarray(a, np.float32)
    try:
        return Vt.Vec3fArray.FromNumpy(a)
    except Exception:  # noqa: BLE001  古い USD には FromNumpy がない
        return Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in a])


def _define_dynamic_mesh(stage, path: str, topology, mat, label: str | None):
    """点の位置だけを撮るたびに書き換えるメッシュ（つなぎ方は最初に 1 回だけ書く）。"""
    counts, idx = topology
    mesh = UsdGeom.Mesh.Define(stage, path)
    n = max(idx) + 1
    mesh.CreatePointsAttr(_vec3_array(np.zeros((n, 3))))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateNormalsAttr(_vec3_array(np.tile([0.0, 0.0, 1.0], (n, 1))))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr(_vec3_array([[-1e-3] * 3, [1e-3] * 3]))
    bind(mesh.GetPrim(), mat)
    if label:
        add_semantic(mesh.GetPrim(), label)
    UsdGeom.Imageable(mesh.GetPrim()).MakeInvisible()
    return mesh


def _set_mesh(mesh, pts, normals=None):
    pts = np.asarray(pts, float)
    mesh.GetPointsAttr().Set(_vec3_array(pts))
    if normals is not None:
        mesh.GetNormalsAttr().Set(_vec3_array(normals))
    mesh.GetExtentAttr().Set(_vec3_array([pts.min(axis=0), pts.max(axis=0)]))


def make_graded_pool(stage, finish: dict) -> dict:
    """濃さつきの欠陥（部品と同じ亜鉛の材質で、色だけを暗くする）と、ボルトの先端用の黒い穴あき板を作る。
    前の欠陥と同じく、物理を動かす前に作っておき、撮るときは点の位置・色・見える/見えないだけを変える。"""
    z = finish["zinc"]
    UsdGeom.Xform.Define(stage, "/World/defects2")
    pool = {}
    for kind in DM.TYPES:
        mpath = f"/World/Looks/graded_{kind}"
        mat = preview_material(stage, mpath, z["color"], z["metallic"], z["roughness"])
        mesh = _define_dynamic_mesh(stage, f"/World/defects2/{kind}", DM.TOPOLOGY[kind], mat, kind)
        pool[kind] = {"mesh": mesh, "shader": UsdShade.Shader.Get(stage, mpath + "/Shader")}
    black = preview_material(stage, "/World/Looks/collar", (0.02, 0.02, 0.02), 0.0, 1.0)
    collar = _define_dynamic_mesh(stage, "/World/collar", DM.annulus_topology(48), black, None)
    return {"defects": pool, "collar": collar, "zinc": z}


def place_backdrop_at(prim, center, normal, size: float):
    """面の後ろ 6cm に、size 四方の黒い板を置く。"""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    helper = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, np.cross(n, u), n]) @ np.diag([size, size, 0.002])
    T[:3, 3] = np.asarray(center, float) - n * 0.06
    set_world_matrix(prim, T)


def hide_backdrop(prim):
    set_world_matrix(prim, np.diag([0.16, 0.16, 0.002, 1.0]) + np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, -5.0],
                                                                        [0, 0, 0, 0]]))


def _match(shot: dict, g: dict) -> bool:
    if "where" in g and shot["where"] != g["where"]:
        return False
    if "kind" in g and shot["kind"] not in g["kind"]:
        return False
    return not ("face" in g and shot["face"] not in g["face"])


def build_entries(cfg: dict, sdef: dict, plan_shots: list, rng) -> list[dict]:
    """1 つの組の撮影の一覧を作る。良品はレシピ（部品 x 面）ごとに同じ枚数、不良は面のまとまりごとに
    defects_per_group 枚（種類 3 つを同じ数、濃さは 10〜90% を種類ごとに等間隔の帯から 1 つずつ）。順番は混ぜる。"""
    dcfg = cfg["defect"]
    lo, hi = dcfg["contrast"]
    entries = []
    for g in sdef["groups"]:
        gs = [s for s in plan_shots if _match(s, cfg["groups"][g])]
        if not gs:
            raise ValueError(f"{sdef['name']}: まとまり {g} に当たる撮影がない")
        for s in gs:
            entries += [{"shot": s, "group": g, "defect": None, "ood": None}
                        for _ in range(int(sdef.get("good_per_recipe", 0)))]
        n = int(sdef.get("defects_per_group", 0))
        if n:
            specs = []
            for ti, typ in enumerate(dcfg["types"]):
                m = n // len(dcfg["types"]) + (1 if ti < n % len(dcfg["types"]) else 0)
                ts = lo + (hi - lo) * (np.arange(m) + rng.uniform(0, 1, m)) / m
                specs += [(typ, float(t)) for t in ts]
            order = rng.permutation(len(specs))
            for i, j in enumerate(order):
                typ, t = specs[j]
                entries.append({"shot": gs[i % len(gs)], "group": g, "defect": {"type": typ, "target": t}, "ood": None})
        k_ood = int(sdef.get("ood_per_recipe", 0))
        for s in gs:
            for j in range(k_ood):
                d = None
                if rng.random() < float(sdef.get("ood_defect_p", 0.0)):
                    d = {"type": str(rng.choice(dcfg["types"])), "target": float(rng.uniform(lo, hi))}
                entries.append({"shot": s, "group": g, "defect": d, "ood": "dim" if j % 2 == 0 else "bright"})
    return [entries[i] for i in rng.permutation(len(entries))]


def view_geometry(shot: dict, spec: dict, view: dict) -> tuple[np.ndarray, np.ndarray, float | None]:
    """撮り方に合わせた (部品の座標で足す回転, 面の中心の world, 倍率の基準の大きさ mm)。"""
    extra = np.eye(4)
    center = np.asarray(shot["face_center_world"], float)
    extent = None
    if spec["kind"] == "nut" and shot["face"] == "side" and view.get("nut") == "flat":
        # 角ではなく平らな面をカメラに向ける（部品を軸のまわりに 30 度回す）。写る幅は角から角まで
        extra = DM.rot_z(30.0)
        P = np.asarray(shot["part_world"], float) @ extra
        center = P[:3, :3] @ DM.nut_flat_center_local(spec) + P[:3, 3]
        extent = max(spec["height"], 2 * spec["across_flats"] / 3 ** 0.5)
    if spec["kind"] == "bolt" and shot["face"] == "tip_end" and view.get("tip") == "wide":
        extent = 2 * spec["head_across_flats"] / 3 ** 0.5   # 後ろの頭が見切れないように、頭の角から角まで写す
    return extra, center, extent


def capture_entries(ctx, entries: list, out: Path, view: dict, gamma, rng, meta: dict):
    """組の一覧を順に撮り、PNG と shots.json を out に書く（capture_set の新しい版。欠陥は濃さつき）。"""
    out.mkdir(parents=True, exist_ok=True)
    stage, robot, cam, ring = ctx["stage"], ctx["robot"], ctx["cam"], ctx["ring"]
    gp = ctx["graded"]
    zinc = np.asarray(gp["zinc"]["color"], float)
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        simulation_app.update()
    board = float(view.get("board", 0.16))
    records = []
    for k, e in enumerate(entries):
        shot = e["shot"]
        spec = ctx["part_spec"][shot["part"]]
        q = np.zeros((1, robot.num_dofs), dtype=np.float32)
        q[0, ctx["dof_index"]] = shot["q"]
        robot.set_dof_positions(q)
        extra, center, extent = view_geometry(shot, spec, view)
        pose = jitter_pose(shot["part_world"], rng) @ extra
        set_world_matrix(ctx["parts"][shot["part"]], pose)
        for name, m in shot["other_parts_world"].items():
            set_world_matrix(ctx["parts"][name], m)
        if shot["where"] == "hand":
            place_backdrop_at(ctx["backdrop"], center, shot["face_normal_world"], board)
        else:
            hide_backdrop(ctx["backdrop"])
        distance = float(np.linalg.norm(np.asarray(shot["camera_world"])[:3, 3] - center))
        zoom(cam, spec, shot["face_type"], distance, extent)
        ppm = args.fill * args.height / (extent or visible_extent_mm(spec, shot["face_type"]))

        gain = float(rng.uniform(0.9, 1.1))
        if e["ood"] == "dim":
            gain = float(rng.uniform(0.40, 0.55))
        elif e["ood"] == "bright":
            gain = float(rng.uniform(1.6, 1.9))
        ring.GetIntensityAttr().Set(args.ring_intensity * gain)

        collar_on = spec["kind"] == "bolt" and shot["face"] == "tip_end" and view.get("tip") == "collar"
        if collar_on:
            _set_mesh(gp["collar"], DM.collar_points(spec, float(view.get("tip_gap_mm", 0.05))))
            set_world_matrix(gp["collar"].GetPrim(), pose)
            UsdGeom.Imageable(gp["collar"].GetPrim()).MakeVisible()

        defects, shown = [], None
        if e["defect"]:
            typ, target = e["defect"]["type"], float(e["defect"]["target"])
            geo = DM.make_defect(typ, spec, shot["face"], rng, ppm, view)
            g_typ = float(gamma[typ]) if isinstance(gamma, dict) else float(gamma)   # 種類ごとに試し撮りで合わせた値
            factor = DM.contrast_to_factor(target, g_typ)
            color = zinc * factor * (np.asarray(DM.STAIN_TINT) if typ == "stain" else 1.0)
            item = gp["defects"][typ]
            item["shader"].GetInput("diffuseColor").Set(Gf.Vec3f(*map(float, color)))
            _set_mesh(item["mesh"], geo["points"], geo["normals"])
            set_world_matrix(item["mesh"].GetPrim(), pose)
            UsdGeom.Imageable(item["mesh"].GetPrim()).MakeVisible()
            shown = item["mesh"].GetPrim()
            defects.append({"type": typ, "target_contrast": round(target, 4), "gamma": g_typ, "albedo_factor": round(factor, 5),
                            "size_mm": geo["size_mm"], "s_mm": geo["s_mm"], "where": geo["where"],
                            "shrink": geo["shrink"]})
        for _ in range(6):
            simulation_app.update()
        cam_err = cam_err_after = None
        target_cam = np.asarray(shot["camera_world"])[:3, 3]
        if ctx["attached"]:
            cam_err = float(np.linalg.norm(world_position(ctx["cam_parent"], stage) - target_cam))
        rep.orchestrator.step(rt_subframes=8)
        if ctx["attached"]:
            cam_err_after = float(np.linalg.norm(world_position(ctx["cam_parent"], stage) - target_cam))
        name = f"{k:04d}_{shot['part']}_{shot['face']}.png"
        save_png(out / name, ctx["rgb"].get_data())
        box = ctx["bbox"].get_data()
        records.append({"file": name, "seq": shot["seq"], "part": shot["part"], "face": shot["face"],
                        "face_type": shot["face_type"], "where": shot["where"], "group": e["group"],
                        "good": not defects and e["ood"] is None, "defects": defects, "ood": e["ood"],
                        "light_gain": gain, "view": view, "ppm": round(ppm, 4),
                        "bbox": np.asarray(box["data"]).tolist() if isinstance(box, dict) and "data" in box else None,
                        "bbox_labels": box.get("info", {}).get("idToLabels") if isinstance(box, dict) else None,
                        "camera_position_error_m": cam_err, "camera_position_error_after_m": cam_err_after})
        if k % 50 == 0 or k == len(entries) - 1:
            print(f"{out.name}/{name}: defects={[(d['type'], d['target_contrast']) for d in defects]} ood={e['ood']} "
                  f"camera_err_after={cam_err_after} [time] {time.monotonic() - T0:.1f} s", flush=True)
        if shown is not None:
            UsdGeom.Imageable(shown).MakeInvisible()
        if collar_on:
            UsdGeom.Imageable(gp["collar"].GetPrim()).MakeInvisible()
    (out / "shots.json").write_text(json.dumps({"shots": records, "args": vars(args), **meta}, ensure_ascii=False,
                                               indent=1), encoding="utf-8")
    print(f"saved {len(records)} shots to {out} [time] {time.monotonic() - T0:.1f} s", flush=True)


def run_sets(ctx, plan: dict, rng):
    cfg = json.loads(Path(args.sets).read_text(encoding="utf-8"))
    gamma = args.contrast_gamma or cfg["defect"]["gamma"]   # 数 1 つか、種類ごとの {"dent": ..., ...}
    only = {s for s in args.only.split(",") if s}
    root = Path(args.sets_root)
    for sdef in cfg["sets"]:
        if only and sdef["name"] not in only:
            continue
        entries = build_entries(cfg, sdef, plan["shots"], rng)
        view = sdef.get("view", {})
        print(f"[set] {sdef['name']}: {len(entries)} shots view={view}", flush=True)
        capture_entries(ctx, entries, root / sdef["name"], view, gamma, rng,
                        {"set": sdef, "defect_cfg": {**cfg["defect"], "gamma": gamma}, "groups": cfg["groups"],
                         "seed": args.seed})


def main():
    plan = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    part_spec = {p["name"]: p for p in plan["parts"]}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    robot_usd = import_robot()
    print(f"[time] URDF -> USD: {time.monotonic() - T0:.1f} s", flush=True)
    stage, parts, cam, attached, cam_parent, backdrop = build_stage(robot_usd, plan)
    defect_pool = make_defect_pool(stage)  # 物理を動かす前に作る
    graded = make_graded_pool(stage, plan["finish"]) if args.sets else None
    disable_gravity(stage)

    rp = rep.create.render_product(str(cam.GetPath()), (args.width, args.height))
    rgb = get_annotator("rgb")
    rgb.attach([rp])
    bbox = get_annotator("bounding_box_2d_tight")
    bbox.attach([rp])

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    root = find_articulation_root(stage)
    print(f"articulation root: {root}", flush=True)
    robot = Articulation(root)
    print(f"articulation dofs: {robot.dof_names}", flush=True)
    names = plan["joint_names"]
    dof_index = [robot.dof_names.index(n) for n in names]
    print(f"[time] stage ready: {time.monotonic() - T0:.1f} s", flush=True)

    ctx = {"stage": stage, "robot": robot, "dof_index": dof_index, "parts": parts, "part_spec": part_spec,
           "cam": cam, "attached": attached, "cam_parent": cam_parent, "backdrop": backdrop,
           "ring": UsdLux.DiskLight(stage.GetPrimAtPath(cam_parent + "/RingLight")), "rgb": rgb, "bbox": bbox,
           "defect_pool": defect_pool, "graded": graded}
    if args.sets:
        run_sets(ctx, plan, rng)
    elif args.good_out or args.test_out:
        if args.good_out:
            capture_set(ctx, plan["shots"] * args.good_repeat, Path(args.good_out), True, 0.0, 0.0, rng)
        if args.test_out:
            capture_set(ctx, plan["shots"] * args.test_repeat, Path(args.test_out), False, args.defects, args.p_ood, rng)
    else:
        shots = plan["shots"] * args.repeat
        if args.max_shots:
            shots = shots[: args.max_shots]
        capture_set(ctx, shots, out, args.good_only, args.defects, 0.0, rng)
    rep.orchestrator.wait_until_complete()
    timeline.stop()


if __name__ == "__main__":
    try:
        main()
    except BaseException:  # noqa: BLE001
        # close() のあとは fastShutdown ですぐにプロセスが終わり、トレースバックが出ないので、先に書き出す
        tb = traceback.format_exc()
        print(tb, flush=True)
        try:
            Path(args.out).mkdir(parents=True, exist_ok=True)
            (Path(args.out) / "error.txt").write_text(tb, encoding="utf-8")
        except OSError:
            pass
    finally:
        simulation_app.close()
