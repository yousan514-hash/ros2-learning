"""疑似カメラと検査（3 段判定・範囲外の見張り）と、画像メッセージの詰め方のテスト。"""
import json
from pathlib import Path

import numpy as np
import pytest

from dual_arm_inspection import cell as C
from dual_arm_inspection import imaging
from dual_arm_inspection import inspection as I
from dual_arm_inspection import part_shapes


@pytest.fixture(scope="module")
def parts():
    return C.load_parts()[0]


@pytest.fixture(scope="module")
def inspector(parts):
    ins = I.Inspector([parts[1], parts[4], parts[7]])  # bolt_M6x30, nut_M6, washer_M6（時間を短くする）
    ins.calibrate(n_fit=16, n_val=16)
    return ins


def test_render_shape_and_seed(parts):
    a, ta = imaging.render(parts[1], "bolt_profile", np.random.default_rng(3), [{"type": "dent", "severity": 0.8}])
    b, tb = imaging.render(parts[1], "bolt_profile", np.random.default_rng(3), [{"type": "dent", "severity": 0.8}])
    assert a.shape == (imaging.H, imaging.W, 3) and a.dtype == np.uint8
    assert np.array_equal(a, b) and ta == tb
    json.dumps(ta)  # 正解は JSON にできる（/inspection/shot で送る）


def test_every_face_type_renders(parts):
    rng = np.random.default_rng(0)
    for p in parts:
        for f in p.faces:
            img, _ = imaging.render(p, f.face_type, rng)
            gray = img.mean(axis=2)
            frac = (gray > 90).mean()
            assert 0.05 < frac < 0.85, (p.name, f.face_type, frac)


def test_strong_dent_is_darker(parts):
    rng = np.random.default_rng(4)
    img, truth = imaging.render(parts[7], "ring", rng, [{"type": "dent", "severity": 1.0}], jitter=False)
    x, y, w, h = truth["defects"][0]["bbox"]
    gray = img.mean(axis=2)
    part_level = np.median(gray[gray > 90])
    assert gray[y:y + h, x:x + w].min() < part_level - 40  # 打痕の芯は、部品のふつうの明るさよりはっきり暗い


def test_three_way_decisions(inspector, parts):
    rng = np.random.default_rng(11)
    bolt = parts[1]
    clean, _ = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting(1.0, 0.05, 0.3))
    assert inspector.inspect(clean, bolt.name, "hex_end").verdict == "OK"
    bad, _ = imaging.render(bolt, "hex_end", rng, [{"type": "scratch", "severity": 1.0}], imaging.Lighting())
    res = inspector.inspect(bad, bolt.name, "hex_end")
    assert res.verdict == "NG" and res.boxes
    dim, _ = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting(gain=0.5, ood=True))
    assert inspector.inspect(dim, bolt.name, "hex_end").reason == "out_of_range:gain"
    stray, _ = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting(ambient=70.0, ood=True))
    assert inspector.inspect(stray, bolt.name, "hex_end").verdict == "HOLD"
    empty = np.full((imaging.H, imaging.W, 3), 36, np.uint8)
    assert inspector.inspect(empty, bolt.name, "hex_end").reason == "no_part"


def test_thresholds_from_good_only(inspector):
    cal = inspector.calibration
    assert cal["t_ok"] < cal["t_ng"]
    assert cal["good_images"] == 16 * len(inspector.templates)


def test_evaluate_table(inspector):
    rows = I.evaluate(inspector, n=60, seed=2)
    table = I.confusion(rows)
    assert sum(sum(v.values()) for v in table.values()) == 60
    s = I.summary(table)
    assert s["overkill_rate"] <= 0.05
    assert s["ood_caught"] == 1.0


def test_part_mesh_size(parts):
    tris = part_shapes.part_triangles("bolt", parts[2].dims)  # M8x40
    z = tris[:, :, 2]
    assert z.max() - z.min() == pytest.approx((40 + 5.3) / 1000)
    nut = part_shapes.part_triangles("nut", parts[5].dims)
    assert np.abs(nut[:, :, 0]).max() == pytest.approx(13 / 2000, rel=1e-6)  # 二面幅が x


def test_parts_json_is_same_as_testbench():
    src = Path(r"C:\Users\yousa\dev\synthetic-testbench\config\parts.json")
    if not src.exists():
        pytest.skip("synthetic-testbench がない")
    here = Path(__file__).resolve().parents[1] / "config" / "parts.json"
    assert json.loads(here.read_text(encoding="utf-8")) == json.loads(src.read_text(encoding="utf-8"))


def test_image_msg_round_trip():
    ru = pytest.importorskip("dual_arm_inspection.ros_utils")
    from builtin_interfaces.msg import Time
    img = np.random.default_rng(0).integers(0, 256, (imaging.H, imaging.W, 3), dtype=np.uint8)
    msg = ru.image_to_msg(img, Time(sec=1, nanosec=2), "r_camera_optical_frame")
    assert msg.encoding == "rgb8" and msg.step == imaging.W * 3 and len(msg.data) == img.size
    assert np.array_equal(ru.msg_to_image(msg), img)


def test_fit_from_given_images_and_judge_folder(tmp_path, parts, capsys):
    """Isaac Sim の画像を判定する道具（isaac/judge_folder.py）を、疑似カメラの画像で通しで確かめる。"""
    import importlib.util

    import cv2

    rng = np.random.default_rng(21)
    bolt = parts[1]

    def write(folder, n, defects):
        folder.mkdir()
        recs = []
        for k in range(n):
            d = [{"type": "scratch", "severity": 1.0}] if defects and k % 2 else []
            img, _ = imaging.render(bolt, "hex_end", rng, d, imaging.sample_lighting(rng))
            name = f"{k:03d}.png"
            cv2.imwrite(str(folder / name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            recs.append({"file": name, "part": bolt.name, "face_type": "hex_end", "good": not d})
        (folder / "shots.json").write_text(json.dumps({"shots": recs}), encoding="utf-8")

    write(tmp_path / "good", 16, False)
    write(tmp_path / "test", 8, True)
    path = Path(__file__).resolve().parents[1] / "isaac" / "judge_folder.py"
    spec = importlib.util.spec_from_file_location("judge_folder", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main(["--good", str(tmp_path / "good"), "--test", str(tmp_path / "test")]) == 0
    out = capsys.readouterr().out
    assert "'NG': 4" in out  # 強い傷 4 枚はすべて NG
    assert "'good': {'OK': 4" in out or "good          {'OK': 4" in out


def test_fit_needs_enough_images(parts):
    ins = I.Inspector([parts[1]])
    img, _ = imaging.render(parts[1], "hex_end", np.random.default_rng(0))
    with pytest.raises(ValueError):
        ins.fit({(parts[1].name, "hex_end"): [img] * 3}, n_fit=3)


# ---- 形の見張り（2026-11-08）: 濃い欠陥で部品が「欠けて」見えても範囲外にせず、点数に回す ----
def _paint_dark_blob(img, cx, cy, r):
    """部品の縁にかかる濃い汚れのつもりで、暗い円を描く（大津の二値化で部品から外れる大きさ）。"""
    out = img.copy()
    import cv2
    cv2.circle(out, (int(cx), int(cy)), int(r), (30, 28, 26), -1, cv2.LINE_AA)
    return out


def test_dark_blob_at_edge_goes_to_score_not_shape(inspector, parts):
    """大きく濃い汚れが縁にかかって部品の面積が 15% 以上減っても、形の見張りで保留にせず NG にする。"""
    rng = np.random.default_rng(31)
    bolt = parts[1]
    img, truth = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting())
    cx, cy = truth["pose"]["cx"], truth["pose"]["cy"]
    bad = _paint_dark_blob(img, cx + 55, cy, 34)  # 六角の右の縁にかかる暗い円
    res = inspector.inspect(bad, bolt.name, "hex_end")
    assert res.features["missing"] > 0.10          # 学んだ形の 1 割以上が暗く欠けている
    assert res.features["area_ratio"] < 0.85       # 前の見張り（面積 15%）なら「shape」で保留になっていた
    assert res.verdict == "NG" and res.reason == "defect", (res.verdict, res.reason, res.features)
    assert res.defect_type == "stain" and res.boxes


def test_dark_blob_inside_goes_to_score_not_shape(inspector, parts):
    """部品の中の大きな暗い所（穴として切り出しから外れる）も、形の見張りではなく点数で NG。"""
    rng = np.random.default_rng(32)
    bolt = parts[1]
    img, truth = imaging.render(bolt, "circle_end", rng, (), imaging.Lighting())  # ボルトの先端の面
    cx, cy = truth["pose"]["cx"], truth["pose"]["cy"]
    bad = _paint_dark_blob(img, cx + 20, cy - 8, 22)
    res = inspector.inspect(bad, bolt.name, "circle_end")
    assert res.features["missing"] > 0.05
    assert res.verdict == "NG" and res.reason == "defect", (res.verdict, res.reason, res.features)


def test_lighting_guard_still_works_with_dark_blob(inspector, parts):
    """暗く欠けた所があっても、照明の見張り（明るさの倍率）は残っている。"""
    rng = np.random.default_rng(33)
    bolt = parts[1]
    dim, truth = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting(gain=0.5, ood=True))
    cx, cy = truth["pose"]["cx"], truth["pose"]["cy"]
    res = inspector.inspect(_paint_dark_blob(dim, cx + 55, cy, 34), bolt.name, "hex_end")
    assert res.verdict == "HOLD" and res.reason == "out_of_range:gain"


def test_extra_bright_object_is_out_of_range(inspector, parts):
    """学んだ形からはみ出る明るい物（指・隣の部品）が部品にくっついて写ったら、範囲外として保留。"""
    import cv2

    rng = np.random.default_rng(34)
    nut = parts[4]
    img, truth = imaging.render(nut, "hex_ring", rng, (), imaging.Lighting())
    cx, cy = int(truth["pose"]["cx"]), int(truth["pose"]["cy"])
    out = img.copy()
    cv2.rectangle(out, (cx + 40, cy - 12), (cx + 150, cy + 12), (190, 190, 195), -1)  # 右へ伸びる明るい棒
    res = inspector.inspect(out, nut.name, "hex_ring")
    assert res.features["extra"] > 0.15
    assert res.verdict == "HOLD" and res.reason == "out_of_range:extra", (res.reason, res.features)


def test_mostly_missing_part_is_shape(inspector, parts):
    """学んだ形の大部分が欠けている（別の部品・見切れ）ときは、欠陥ではなく形の範囲外。"""
    rng = np.random.default_rng(35)
    bolt = parts[1]
    img, truth = imaging.render(bolt, "hex_end", rng, (), imaging.Lighting())
    cx = int(truth["pose"]["cx"])
    out = img.copy()
    out[:, : cx + 50] = (36, 36, 36)  # 右の端の 2 割ほどだけ残して、背景で塗りつぶす（見切れ）
    res = inspector.inspect(out, bolt.name, "hex_end")
    assert res.verdict == "HOLD" and res.reason == "shape"
    assert res.features["missing"] > I.MISSING_MAX


def test_shape_diff_and_refine_align():
    """shape_diff は欠け・はみ出しを分け、refine_align はずれた位置を学んだ形に戻す。"""
    import cv2

    roi = np.zeros((120, 160), bool)
    cv2.rectangle(roi.view(np.uint8), (40, 30), (120, 90), 1, -1)
    m = roi.astype(np.float32).copy()
    m[40:60, 50:70] = 0.0            # 中に欠け
    m[50:70, 125:140] = 1.0          # 右にはみ出し
    missing, extra = I.shape_diff(m, roi)
    assert 350 <= missing.sum() <= 400 and 250 <= extra.sum() <= 300
    assert not missing[:, 0:40].any() and not extra[:, 40:121].any()
    # 位置合わせ: 右下が欠けたマスクを (+7, -5) ずらして渡すと、refine_align が (-7, +5) 戻す
    mask = roi.astype(np.uint8)
    mask[70:91, 90:121] = 0
    M = np.array([[1.0, 0.0, 7.0], [0.0, 1.0, -5.0]])
    M2, shift = I.refine_align(mask, M, roi, radius=40.0)
    assert shift == (-7, 5)
    assert np.allclose(M2[:, 2], (0.0, 0.0))
