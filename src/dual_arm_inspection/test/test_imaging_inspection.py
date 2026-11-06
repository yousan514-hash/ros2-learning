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
