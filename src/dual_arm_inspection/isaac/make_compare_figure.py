"""疑似画像と Isaac Sim の画像で、検査の当たり外れを比べる絵を作る（pixi の環境で動かす）。

    pixi run python src/dual_arm_inspection/isaac/make_compare_figure.py

読むもの: isaac/out/good/（良品）、isaac/out/test/（欠陥あり）、isaac/out/judge.json（judge_folder.py の --json）
作るもの: docs/isaac_shots.png（27 面を 1 枚ずつ）、docs/isaac_vs_synthetic.png（表と見本の画像）
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(HERE.parent))
import check_dual_arm as CK  # noqa: E402  OpenCV の描き方（日本語の字・色）と、疑似画像の評価を使い回す
from dual_arm_inspection import inspection as I  # noqa: E402

OUT = HERE / "out"
DOCS = ROOT / "docs"


def load(folder: Path, name: str) -> np.ndarray:
    return cv2.imdecode(np.fromfile(str(folder / name), np.uint8), cv2.IMREAD_COLOR)


def shots_sheet(T) -> Path:
    recs = json.loads((OUT / "good" / "shots.json").read_text(encoding="utf-8"))["shots"][:27]
    tw, th, cols = 240, 180, 7
    rows = (len(recs) + cols - 1) // cols
    img = np.full((40 + rows * (th + 26), 12 + cols * (tw + 8), 3), CK.SURFACE, np.uint8)
    T.put(img, "Isaac Sim 6.2 の右手首カメラ（Replicator）で撮った 27 面（良品、480x360 を縮小）", 12, 28, 18, CK.INK)
    for k, r in enumerate(recs):
        x, y = 12 + (k % cols) * (tw + 8), 40 + (k // cols) * (th + 26)
        img[y:y + th, x:x + tw] = cv2.resize(load(OUT / "good", r["file"]), (tw, th), interpolation=cv2.INTER_AREA)
        T.put(img, f"{r['part']} / {r['face']}（{'トレイ' if r['where'] == 'tray' else '手'}）", x, y + th + 18, 13, CK.INK2)
    path = DOCS / "isaac_shots.png"
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


def draw_table(img, T, x0, y0, title, table, summary, note):
    T.put(img, title, x0, y0, 17, CK.INK)
    cols, cw, rh = ["OK", "NG", "HOLD"], 84, 34
    gx, gy = x0 + 150, y0 + 14
    for j, c in enumerate(cols):
        x = gx + j * cw
        cv2.rectangle(img, (x + 4, gy), (x + cw - 4, gy + 26), CK.STATUS[c], -1)
        T.put(img, "保留" if c == "HOLD" else c, x + cw // 2, gy + 19, 14, CK.STATUS_INK[c], "center")
    for i, (key, name) in enumerate((("good", "良品"), ("defect", "不良"), ("out_of_range", "範囲外"))):
        y = gy + 32 + i * rh
        T.put(img, f"{name}（{sum(table[key].values())}）", gx - 10, y + 23, 15, CK.INK, "right")
        for j, c in enumerate(cols):
            x = gx + j * cw
            cv2.rectangle(img, (x + 4, y + 3), (x + cw - 4, y + rh - 3), CK.GRID, 1)
            T.put(img, str(table[key][c]), x + cw // 2, y + 24, 17, CK.INK, "center")
    y = gy + 32 + 3 * rh + 24
    T.put(img, f"見逃し {summary['miss_rate']:.1%} ／ 過検出 {summary['overkill_rate']:.1%} ／ 人に回る {summary['hold_rate']:.1%}"
               f" ／ 範囲外を保留 {summary['ood_caught']:.0%}", x0, y, 14, CK.INK)
    T.put(img, note, x0, y + 22, 13, CK.MUTED)


def compare_figure(T, synth) -> Path:
    judge = json.loads((OUT / "judge.json").read_text(encoding="utf-8"))
    img = np.full((900, 1640, 3), CK.SURFACE, np.uint8)
    T.put(img, "同じ検査（良品だけで学ぶ → OK／NG／保留）を、疑似画像と Isaac Sim の画像で比べた", 24, 36, 21, CK.INK)
    s_tab, s_sum = synth["table"], synth["summary"]
    draw_table(img, T, 24, 80, f"疑似画像（OpenCV で作図、{s_sum['total']} 枚）", s_tab, s_sum,
               f"学び: 18 種類 x 良品 40 枚。欠陥は面の模様に直接描く（強さ 0〜1）")
    draw_table(img, T, 840, 80, f"Isaac Sim 6.2（Replicator、{judge['summary']['total']} 枚）", judge["table"],
               judge["summary"], "学び: 27 面 x 良品 30 枚（18 枚で学び 12 枚でしきい値）。欠陥は面の上の小さな形")
    T.put(img, "Isaac Sim の画像の判定の例（帯の色 = 判定、四角 = 欠陥の候補）", 24, 380, 18, CK.INK)
    rows = judge["rows"]
    want = [lambda r: r["truth"] == "good" and r["verdict"] == "OK",
            lambda r: r["truth"] == "defect" and r["verdict"] == "NG" and "dent" in r["true_types"],
            lambda r: r["truth"] == "defect" and r["verdict"] == "NG" and "stain" in r["true_types"],
            lambda r: r["truth"] == "defect" and r["verdict"] == "HOLD",
            lambda r: r["truth"] == "out_of_range" and r["verdict"] == "HOLD",
            lambda r: r["truth"] == "defect" and r["verdict"] == "OK" and "scratch" in r["true_types"],
            lambda r: r["truth"] == "good" and r["verdict"] == "NG"]
    picks, used = [], set()
    for f in want:
        r = next((r for r in rows if f(r) and r["file"] not in used), None)
        if r:
            picks.append(r)
            used.add(r["file"])
    tw, th = 224, 168
    truth_ja = {"good": "良品", "defect": "不良", "out_of_range": "範囲外（照明）"}
    type_ja = {"scratch": "傷", "dent": "打痕", "stain": "汚れ"}
    for k, r in enumerate(picks):
        x, y = 24 + k * (tw + 8), 400
        raw = cv2.cvtColor(load(OUT / "test", r["file"]), cv2.COLOR_BGR2RGB)
        res = I.Result(r["verdict"], r["score"], r["reason"], r["pred_type"], r["boxes"])
        tile = cv2.resize(I.annotate(raw, res), (tw, th), interpolation=cv2.INTER_AREA)
        img[y:y + th, x:x + tw] = cv2.cvtColor(tile, cv2.COLOR_RGB2BGR)
        t = truth_ja[r["truth"]] + ("（" + "・".join(type_ja[v] for v in r["true_types"]) + "）" if r["true_types"] else "")
        T.put(img, f"{r['part']} / {r['face']}", x, y + th + 18, 13, CK.INK2)
        T.put(img, f"正解: {t}", x, y + th + 36, 13, CK.INK)
    notes = [
        "わかったこと",
        "・Isaac Sim では見逃しが 3 枚（どれも持って撮る横の面の細い傷。指の近く・軸の丸みで写りにくい）。保留（人に回る）が疑似画像より多い",
        "・背景が暗くそろうトレイの上の面と頭の面は、良品の点数がほぼ 0 で安定。先端の面（後ろの頭が写り込む）とナットの横の面は、良品でもばらつきが大きい（過検出 3 枚のうち 2 枚）",
        "・しきい値はレシピ（部品 x 面）ごとにした。全体で 1 つだと、ばらつきの大きい面にしきい値が引っ張られ、ほかの面の感度が落ちた",
        "・どちらの数値も自作の仮定（欠陥の大きさ・照明）での比べ。実物では確かめていない",
    ]
    for i, line in enumerate(notes):
        T.put(img, line, 24, 650 + i * 28, 16 if i == 0 else 14, CK.INK if i == 0 else CK.INK2)
    path = DOCS / "isaac_vs_synthetic.png"
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


def main() -> int:
    T = CK.Text()
    print(f"絵: {shots_sheet(T)}")
    synth = CK.check_inspection()
    print(f"絵: {compare_figure(T, synth)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
