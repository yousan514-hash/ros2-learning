"""見逃しの曲線と「しきい値 ↔ 人に回る割合」を、疑似画像と Isaac Sim の画像で同じ物差しで比べる（pixi の環境で動かす）。

    pixi run python src/dual_arm_inspection/isaac/analyze_miss_curve.py

読むもの: isaac/out/good/・isaac/out/test/（Isaac Sim の画像と shots.json）。疑似画像はその場で作る（imaging.py）
作るもの: docs/isaac_miss_curve.png（欠陥の大きさ・濃さごとの検出）
          docs/isaac_hold_tradeoff.png（しきい値を動かしたときの見逃しと保留、面ごとの保留）
          isaac/out/miss_curve.json（1 枚ずつの点数と、表の数字）

物差し（どちらの画像にも同じ計算をする。検査のしきい値には関係しない）:
  - 大きさ: 正解の欠陥の外接する四角の長いほうの辺を mm に直したもの（倍率は撮るときのレシピの値）
  - 濃さ: 正解の四角の中で、良品の平均の見え方からの明るさのずれ |D| のいちばん大きい値（%）

数字はすべて自作の仮定（欠陥の作り方・照明・部品の形）での結果。Isaac Sim の欠陥ありは 56 枚と少ないので、
割合には 95% の幅（Wilson の信頼区間）を付ける。
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
ROOT = HERE.parents[2]
for p in (PKG, HERE, ROOT / "tools"):
    sys.path.insert(0, str(p))
import check_dual_arm as CK  # noqa: E402  日本語の字と色を使い回す
import judge_folder as JF  # noqa: E402  Isaac Sim の画像の読み方と、学ぶ単位（部品 x 面）を使い回す
from dual_arm_inspection import cell as C  # noqa: E402
from dual_arm_inspection import imaging as IM  # noqa: E402
from dual_arm_inspection import inspection as I  # noqa: E402

OUT = HERE / "out"
DOCS = ROOT / "docs"
DEFECTS = ("scratch", "dent", "stain")
TYPE_JA = {"scratch": "傷", "dent": "打痕", "stain": "汚れ"}
FACE_JA = {"side_top": "横（トレイ）", "head_end": "頭の面", "tip_end": "先端の面", "side_bottom": "横（手）",
           "top": "上（トレイ）", "bottom": "下（手）", "side": "横（手）"}
KIND_JA = {"bolt": "ボルト", "nut": "ナット", "washer": "座金"}
# 写り込みのある面: ボルトの先端の面（後ろの頭が見える）、ナットの横の面（穴の奥・指が見える）
REFLECT = {("bolt", "tip_end"), ("nut", "side")}
HUMAN_MISS = 0.15   # 人が保留を見たときの見逃し（ナレッジの「目視の見逃し 5〜30%」の真ん中くらい。仮定）
LINE_MIX = {"good": 0.97, "defect": 0.02, "out_of_range": 0.01}  # ラインに近い割合（仮定）


# ---------------------------------------------------------------- 数え方
def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float, float]:
    """割合と 95% の幅（Wilson）。枚数が少ないときも 0〜1 をはみ出さない。"""
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, c - h), min(1.0, c + h)


def pct(k: int, n: int) -> str:
    p, lo, hi = wilson(k, n)
    return f"{100 * p:.1f}%（{k}/{n}、95% の幅 {100 * lo:.1f}〜{100 * hi:.1f}%）" if n else "—"


# ---------------------------------------------------------------- 物差し
def isaac_ppm(part: C.Part, face_type: str, fill: float, height: int) -> float:
    """Isaac Sim の 1mm が何画素か（capture_inspection.py の visible_extent_mm / zoom と同じ式）。"""
    d = part.dims
    if part.kind == "bolt":
        ext = {"bolt_profile": d["length"] + d["head_height"], "hex_end": 2 * d["head_across_flats"] / 3 ** 0.5,
               "circle_end": d["d"]}[face_type]
    elif part.kind == "nut":
        ext = {"hex_ring": 2 * d["across_flats"] / 3 ** 0.5, "nut_profile": max(d["height"], d["across_flats"])}[face_type]
    else:
        ext = d["d_outer"]
    return fill * height / ext


def peak_contrast(ins: I.Inspector, img: np.ndarray, part: str, key: str, box_xyxy) -> float:
    """正解の四角の中で、良品の平均の見え方からの明るさのずれ |D| のいちばん大きい値（3x3 でならしてから）。"""
    tpl = ins.templates[(part, key)]
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    mask = I.segment(gray)
    if mask is None:
        return float("nan")
    a, _, M = I.align(gray, mask, I.base_type(key) in IM.PROFILE_TYPES)
    D, _ = I.measure(a, tpl.mean, tpl.roi)
    x0, y0, x1, y1 = box_xyxy
    pts = np.array([[x0, y0, 1], [x1, y0, 1], [x1, y1, 1], [x0, y1, 1]], np.float64) @ M.T
    sel = np.zeros(D.shape, np.uint8)
    cv2.fillConvexPoly(sel, np.round(pts).astype(np.int32), 1)
    sel = (cv2.dilate(sel, np.ones((3, 3), np.uint8)) > 0) & tpl.roi
    if sel.sum() < 3:
        return float("nan")  # 欠陥が学んだ範囲（ROI）の外 = 縁の上など
    return float(np.max(np.abs(cv2.GaussianBlur(D, (3, 3), 0))[sel]))


def sigma_stats(ins: I.Inspector) -> dict:
    """良品のばらつき σ（ROI の中の中央値と 95 パーセンタイル）を、学ぶ単位ごとに。"""
    return {f"{p}|{k}": [float(np.median(t.sigma[t.roi])), float(np.percentile(t.sigma[t.roi], 95))]
            for (p, k), t in ins.templates.items()}


def raw_score(ins: I.Inspector, img: np.ndarray, part: str, key: str) -> float:
    """範囲外・形の見張りを飛ばして、点数だけ出す（見張りで保留にしたものが、点数ならどうだったかを見る）。"""
    tpl = ins.templates[(part, key)]
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    mask = I.segment(gray)
    if mask is None:
        return float("nan")
    a, _, _ = I.align(gray, mask, I.base_type(key) in IM.PROFILE_TYPES)
    D, _ = I.measure(a, tpl.mean, tpl.roi)
    comps = I.components(D, tpl)
    return float(comps[0]["energy"]) if comps else 0.0


def row_of(ins, res: I.Result, img: np.ndarray, **kw) -> dict:
    tpl = ins.templates[(kw["part"], kw["key"])]
    pre = res.reason not in ("clean", "uncertain", "defect")   # 範囲外・形の違いで、点数の前に保留にしたもの
    return {**kw, "verdict": res.verdict, "reason": res.reason, "score": float(res.score), "pre": pre,
            "raw_score": raw_score(ins, img, kw["part"], kw["key"]) if pre else float(res.score),
            "t_ok": float(tpl.t_ok), "t_ng": float(tpl.t_ng)}


# ---------------------------------------------------------------- 集める
def collect_isaac() -> tuple[list[dict], dict, dict]:
    parts = {p.name: p for p in C.load_parts()[0]}
    ins = I.Inspector(list(parts.values()))
    samples = defaultdict(list)
    for rec, img in JF.load(OUT / "good"):
        samples[(rec["part"], JF.key(rec))].append(img)
    n_fit = max(2, int(min(len(v) for v in samples.values()) * 0.6))   # judge_folder.py と同じ（30 枚なら 18 枚）
    cal = ins.fit(dict(samples), n_fit)
    doc = json.loads((OUT / "test" / "shots.json").read_text(encoding="utf-8"))
    args = ast.literal_eval(doc["args"]) if isinstance(doc["args"], str) else doc["args"]
    fill = float(args.get("fill", 0.7))
    rows = []
    for rec, img in JF.load(OUT / "test"):
        part, key = parts[rec["part"]], JF.key(rec)
        res = ins.inspect(img, part.name, key)
        truth = "out_of_range" if rec.get("ood") else ("defect" if rec.get("defects") else "good")
        size_mm = size_px = contrast = float("nan")
        if rec.get("defects"):
            labels = {int(k): v["class"] for k, v in rec["bbox_labels"].items()}
            boxes = [b for b in rec["bbox"] if labels.get(int(b[0])) in DEFECTS]
            if boxes:
                _, x0, y0, x1, y1, _ = boxes[0]
                size_px = float(max(x1 - x0 + 1, y1 - y0 + 1))
                size_mm = size_px / isaac_ppm(part, rec["face_type"], fill, img.shape[0])
                contrast = peak_contrast(ins, img, part.name, key, (x0, y0, x1, y1))
        rows.append(row_of(ins, res, img, ds="isaac", file=rec["file"], part=part.name, kind=part.kind, face=rec["face"],
                           key=key, where=rec["where"], truth=truth, types=[d["type"] for d in rec.get("defects", [])],
                           size_px=size_px, size_mm=size_mm, contrast=contrast, gain=float(rec.get("light_gain", 1.0))))
    return rows, cal, sigma_stats(ins)


def collect_synthetic(n: int, seed: int, p_defect: float, p_ood: float, n_fit: int = 20, n_val: int = 20,
                      ins: I.Inspector | None = None) -> tuple[list[dict], I.Inspector]:
    """inspection.evaluate と同じ順に乱数を使う（seed=7・n=200 なら README の 200 枚と同じ画像になる）。"""
    if ins is None:
        ins = I.Inspector(C.load_parts()[0])
        ins.calibrate(n_fit, n_val)
    rng = np.random.default_rng(seed)
    plist = list(ins.parts.values())
    rows = []
    for _ in range(n):
        part = plist[int(rng.integers(len(plist)))]
        face = part.faces[int(rng.integers(len(part.faces)))]
        img, truth = IM.random_shot(rng, part, face.face_type, p_defect, p_ood)
        res = ins.inspect(img, part.name, face.face_type)
        size_mm = size_px = contrast = float("nan")
        if truth["defects"]:
            best = max(truth["defects"], key=lambda d: max(d["bbox"][2], d["bbox"][3]))
            x, y, w, h = best["bbox"]
            size_px = float(max(w, h))
            size_mm = size_px / IM.scale_ppm(part, face.face_type)
            contrast = max(peak_contrast(ins, img, part.name, face.face_type, (bx, by, bx + bw - 1, by + bh - 1))
                           for bx, by, bw, bh in (d["bbox"] for d in truth["defects"]))
        rows.append(row_of(ins, res, img, ds="synthetic", part=part.name, kind=part.kind, face=face.name,
                           key=face.face_type, where=face.where, truth=IM.truth_class(truth),
                           types=[d["type"] for d in truth["defects"]],
                           severity=max([d["severity"] for d in truth["defects"]], default=0.0),
                           size_px=size_px, size_mm=size_mm, contrast=contrast, gain=float(truth["lighting"]["gain"])))
    return rows, ins


# ---------------------------------------------------------------- しきい値を動かす
def verdicts(rows, k_ok: float = 1.0, band: float | None = None) -> list[str]:
    """OK と保留の境を k_ok 倍にする（NG の境はそのまま、ただし OK の境より下にはしない）。
    band を与えると、NG の境を「OK の境 x band」にする（保留の幅をそろえる）。"""
    out = []
    for r in rows:
        if r["pre"]:
            out.append("HOLD")
            continue
        t_ok = r["t_ok"] * k_ok
        t_ng = max(t_ok * band if band else r["t_ng"], t_ok)
        out.append("OK" if r["score"] < t_ok else ("NG" if r["score"] >= t_ng else "HOLD"))
    return out


def rates(rows, vs, mix: dict) -> dict:
    n = defaultdict(int)
    c = defaultdict(int)
    for r, v in zip(rows, vs):
        n[r["truth"]] += 1
        c[(r["truth"], v)] += 1
    per = {t: {v: c[(t, v)] / n[t] if n[t] else 0.0 for v in ("OK", "NG", "HOLD")} for t in ("good", "defect", "out_of_range")}
    wsum = sum(mix.values())
    hold = sum(mix[t] / wsum * per[t]["HOLD"] for t in per)
    held_def = per["defect"]["HOLD"]
    miss = per["defect"]["OK"]
    return {"miss": miss, "overkill": per["good"]["NG"], "hold": hold, "hold_good": per["good"]["HOLD"],
            "hold_defect": held_def, "auto_ng": per["defect"]["NG"],
            "total_miss": miss + HUMAN_MISS * held_def,   # 保留に回った不良も、人が HUMAN_MISS の割合で見逃す
            "n": dict(n), "k": {f"{t}>{v}": c[(t, v)] for t in per for v in ("OK", "NG", "HOLD")}}


def sweep(rows, mix, ks) -> list[dict]:
    out = []
    for k in ks:
        vs = verdicts(rows, k)
        out.append({"k_ok": float(k), **rates(rows, vs, mix), "line": rates(rows, vs, LINE_MIX)})
    return out


def targets(rows, mix, levels=(0.10, 0.05, 0.02, 0.0)) -> list[dict]:
    """AI の見逃しの目標（不良を OK にする割合）ごとに、それを満たすいちばんゆるい OK の境と、そのときの保留。
    点数が 0 の不良（何も見つからない）があると、見逃し 0 には「全部人に回す」（倍率 0）しかない。"""
    df = [r for r in rows if r["truth"] == "defect"]
    ratios = sorted(r["score"] / r["t_ok"] for r in df if not r["pre"])
    out = []
    for target in levels:
        m = int(math.floor(target * len(df) + 1e-9))       # 許される見逃しの枚数
        # m+1 枚目の不良がちょうど OK にならない所（掛け算の丸めで境の上に出ないよう、ほんの少し下げる）
        k = ratios[m] * (1 - 1e-9) if m < len(ratios) else 10 * ratios[-1]
        vs = verdicts(rows, k)
        out.append({"target": target, "k_ok": float(k), "test": rates(rows, vs, mix), "line": rates(rows, vs, LINE_MIX)})
    return out


# ---------------------------------------------------------------- 表にする
SIZE_EDGES = [0.0, 1.0, 2.0, 4.0, 6.0, float("inf")]
SIZE_LABELS = ["1mm 未満", "1〜2mm", "2〜4mm", "4〜6mm", "6mm 以上"]
CON_EDGES = [0.0, 0.1, 0.2, 0.4, 0.7, float("inf")]
CON_LABELS = ["10% 未満", "10〜20%", "20〜40%", "40〜70%", "70% 以上"]
GROUPS = ["トレイの上（上から）", "ボルトの頭の面（手）", "ナット・座金の下（手）", "ボルトの横（手）",
          "ボルトの先端の面（手）", "ナットの横（手）"]
CAUSES = ["範囲外の照明（ねらいどおり）", "不良が形・明るさの見張りに当たった", "不良の点数があいまい", "良品があいまい・形"]


def group_of(r: dict) -> str:
    if r["where"] == "tray":
        return GROUPS[0]
    if r["kind"] == "bolt":
        return {"head_end": GROUPS[1], "side_bottom": GROUPS[3], "tip_end": GROUPS[4]}[r["face"]]
    return GROUPS[5] if (r["kind"], r["face"]) in REFLECT else GROUPS[2]


def cause_of(r: dict) -> str | None:
    if r["verdict"] != "HOLD":
        return None
    if r["truth"] == "out_of_range":
        return CAUSES[0]
    if r["truth"] == "defect":
        return CAUSES[1] if r["pre"] else CAUSES[2]
    return CAUSES[3]


def binned(rows, field, edges) -> list[dict]:
    df = [r for r in rows if r["truth"] == "defect" and not math.isnan(r[field])]
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sub = [r for r in df if lo <= r[field] < hi]
        out.append({"lo": lo, "hi": hi, "n": len(sub), **{v: sum(r["verdict"] == v for r in sub) for v in ("OK", "HOLD", "NG")}})
    return out


def group_table(rows) -> list[dict]:
    out = []
    for g in GROUPS:
        sub = [r for r in rows if group_of(r) == g]
        good = [r for r in sub if r["truth"] == "good"]
        df = [r for r in sub if r["truth"] == "defect"]
        tok = [r["t_ok"] for r in sub]
        out.append({"group": g, "n": len(sub), "n_good": len(good), "n_defect": len(df),
                    "miss": sum(r["verdict"] == "OK" for r in df), "overkill": sum(r["verdict"] == "NG" for r in good),
                    "auto_ng": sum(r["verdict"] == "NG" for r in df), "hold": sum(r["verdict"] == "HOLD" for r in sub),
                    "causes": {c: sum(cause_of(r) == c for r in sub) for c in CAUSES},
                    "t_ok_min": min(tok) if tok else 0.0, "t_ok_max": max(tok) if tok else 0.0})
    return out


def gain_table(rows) -> list[dict]:
    out = []
    for lo, hi in ((0.85, 0.95), (0.95, 1.05), (1.05, 1.15)):
        sub = [r for r in rows if r["truth"] != "out_of_range" and lo <= r["gain"] < hi]
        out.append({"lo": lo, "hi": hi, "n": len(sub), "hold": sum(r["verdict"] == "HOLD" for r in sub),
                    "n_defect": sum(r["truth"] == "defect" for r in sub),
                    "miss": sum(r["truth"] == "defect" and r["verdict"] == "OK" for r in sub),
                    "n_good": sum(r["truth"] == "good" for r in sub),
                    "overkill": sum(r["truth"] == "good" and r["verdict"] == "NG" for r in sub)})
    return out


def what_if(iso, mix) -> list[dict]:
    """Isaac Sim の結果で「ここを直したら」を試す（同じ画像・同じ点数で、決め方だけ変える）。"""
    def fixed_monitor(rows):
        # 見張り（形・明るさ）が不良に反応した分を、点数で判定し直す。正解を見て選んでいるので「直せたときの上限」
        out = []
        for r in rows:
            if r["pre"] and r["truth"] == "defect":
                r = {**r, "pre": False, "score": r["raw_score"]}
            out.append(r)
        return out

    def drop_reflect(rows):
        # 写り込みの 2 面は自動で決めない（全部人へ）= 使用範囲から外す
        return [{**r, "pre": True} if (r["kind"], r["face"]) in REFLECT else r for r in rows]

    cases = [("今のまま", iso, None), ("保留の幅を 3 倍 → 2 倍", iso, 2.0),
             ("見張りが不良に反応しないよう直せたら（上限）", fixed_monitor(iso), None),
             ("↑ + 保留の幅 2 倍", fixed_monitor(iso), 2.0),
             ("写り込みの 2 面を自動の対象から外す", drop_reflect(iso), None),
             ("見張りを直す + 写り込みの 2 面を外す", drop_reflect(fixed_monitor(iso)), None)]
    out = []
    for name, rows, band in cases:
        vs = verdicts(rows, 1.0, band)
        out.append({"case": name, "test": rates(rows, vs, mix), "line": rates(rows, vs, LINE_MIX)})
    return out


SCOPE = GROUPS[:3]   # 使用範囲の案: 自動で決めてよい面（トレイの上・ボルトの頭の面・ナットと座金の下）


def scope_table(rows) -> dict:
    """使用範囲の案の中だけで数える（範囲の外の面は、全部人に回す前提）。照明が範囲外のものは見張りが保留にする。"""
    inside = [r for r in rows if group_of(r) in SCOPE]
    good = [r for r in inside if r["truth"] == "good"]
    df = [r for r in inside if r["truth"] == "defect"]
    ood = [r for r in rows if r["truth"] == "out_of_range"]
    strong = [r for r in rows if r["truth"] == "defect" and not math.isnan(r["contrast"]) and r["contrast"] >= 0.2]
    auto = [r for r in inside if r["verdict"] != "HOLD"]
    return {"n": len(rows), "inside": len(inside), "auto": len(auto),
            "miss": [sum(r["verdict"] == "OK" for r in df), len(df)],
            "overkill": [sum(r["verdict"] == "NG" for r in good), len(good)],
            "ood_held": [sum(r["verdict"] == "HOLD" for r in ood), len(ood)],
            "strong_miss": [sum(r["verdict"] == "OK" for r in strong), len(strong)]}


# ---------------------------------------------------------------- 絵（OpenCV。matplotlib は使わない）
SURF, INK, INK2, MUTED, GRID, AXIS = CK.SURFACE, CK.INK, CK.INK2, CK.MUTED, CK.GRID, CK.AXIS
DS_COLOR = {"synthetic": CK.bgr("#2a78d6"), "isaac": CK.bgr("#eb6834")}   # 区別の色の 1 番・2 番
DS_JA = {"synthetic": "疑似画像", "isaac": "Isaac Sim"}
CAUSE_COLOR = [CK.bgr(h) for h in ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")]
VERDICT_JA = {"OK": "OK（= 見逃し）", "HOLD": "保留（人へ）", "NG": "NG（自動で弾く）"}


class Axes:
    def __init__(self, img, T, x, y, w, h, xr, yr, xlog=False):
        self.img, self.T, self.x, self.y, self.w, self.h = img, T, x, y, w, h
        self.xr, self.yr, self.xlog = xr, yr, xlog

    def X(self, v: float) -> int:
        a, b = self.xr
        if self.xlog:
            v, a, b = math.log10(v), math.log10(a), math.log10(b)
        return int(round(self.x + (min(max(v, a), b) - a) / (b - a) * self.w))

    def Y(self, v: float) -> int:
        a, b = self.yr
        return int(round(self.y + self.h - (min(max(v, a), b) - a) / (b - a) * self.h))

    def frame(self, xticks, yticks, xfmt, yfmt, xlabel="", ylabel="", vgrid=True):
        for v in yticks:
            yy = self.Y(v)
            cv2.line(self.img, (self.x, yy), (self.x + self.w, yy), GRID, 1, cv2.LINE_AA)
            self.T.put(self.img, yfmt(v), self.x - 8, yy + 5, 13, INK2, "right")
        for v in xticks:
            xx = self.X(v)
            if vgrid:
                cv2.line(self.img, (xx, self.y), (xx, self.y + self.h), GRID, 1, cv2.LINE_AA)
            self.T.put(self.img, xfmt(v), xx, self.y + self.h + 20, 13, INK2, "center")
        cv2.line(self.img, (self.x, self.y + self.h), (self.x + self.w, self.y + self.h), AXIS, 1, cv2.LINE_AA)
        if xlabel:
            self.T.put(self.img, xlabel, self.x + self.w // 2, self.y + self.h + 44, 14, INK2, "center")
        if ylabel:
            self.T.put(self.img, ylabel, self.x - 8, self.y - 14, 14, INK2, "left")

    def line(self, xs, ys, color, thick=2):
        pts = np.array([[self.X(a), self.Y(b)] for a, b in zip(xs, ys)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(self.img, [pts], False, color, thick, cv2.LINE_AA)

    def dot(self, x, y, color, r=6):
        c = (self.X(x), self.Y(y))
        cv2.circle(self.img, c, r + 2, SURF, -1, cv2.LINE_AA)   # 重なっても見分けられるよう、まわりに 2px の地の色
        cv2.circle(self.img, c, r, color, -1, cv2.LINE_AA)
        return c


def legend_status(img, T, x, y) -> int:
    for v in ("OK", "HOLD", "NG"):
        cv2.rectangle(img, (x, y - 11), (x + 14, y + 3), CK.STATUS[v], -1)
        x += 20 + T.put(img, VERDICT_JA[v], x + 20, y + 2, 14, INK2) + 18
    return x


def legend_ds(img, T, x, y, line=False) -> int:
    for ds in ("synthetic", "isaac"):
        if line:
            cv2.line(img, (x, y - 4), (x + 22, y - 4), DS_COLOR[ds], 2, cv2.LINE_AA)
            cv2.circle(img, (x + 11, y - 4), 5, DS_COLOR[ds], -1, cv2.LINE_AA)
            x += 30
        else:
            cv2.rectangle(img, (x, y - 8), (x + 14, y - 3), DS_COLOR[ds], -1)
            x += 20
        x += T.put(img, DS_JA[ds], x, y + 2, 14, INK2) + 22
    return x


def bars_panel(img, T, x, y, w, h, data, labels, title, xlabel):
    ax = Axes(img, T, x, y, w, h, (0, len(labels)), (0, 1))
    ax.frame([], [0, 0.25, 0.5, 0.75, 1.0], str, lambda v: f"{100 * v:.0f}%", "", "", vgrid=False)
    T.put(img, title, x - 50, y - 40, 17, INK)
    T.put(img, "（不良のうちの割合）", x + 40, y - 40, 13, INK2)
    gw = w / len(labels)
    bw = int(min(40, gw * 0.3))
    for i, lab in enumerate(labels):
        cx = x + gw * (i + 0.5)
        for j, ds in enumerate(("synthetic", "isaac")):
            b = data[ds][i]
            x0 = int(cx - bw - 3) if j == 0 else int(cx + 3)
            x1 = x0 + bw
            T.put(img, "疑似" if ds == "synthetic" else "Isaac", (x0 + x1) // 2, y + h + 20, 13, INK2, "center")
            if b["n"] == 0:
                T.put(img, "なし", (x0 + x1) // 2, y + h - 8, 12, MUTED, "center")
                continue
            base = 0.0
            for v in ("OK", "HOLD", "NG"):   # 下から 見逃し・保留・NG（見逃しを基準の線から読めるように）
                f = b[v] / b["n"]
                if f > 0:
                    ya, yb = ax.Y(base), ax.Y(base + f)
                    cv2.rectangle(img, (x0, yb + (1 if base + f < 1 else 0)), (x1, ya - (1 if base > 0 else 0)), CK.STATUS[v], -1)
                base += f
            cv2.rectangle(img, (x0, ax.Y(1) - 7), (x1, ax.Y(1) - 3), DS_COLOR[ds], -1)   # どちらの画像かの印
            T.put(img, f"{b['n']}", (x0 + x1) // 2, ax.Y(1) - 12, 12, INK2, "center")
            p, lo, hi = wilson(b["OK"], b["n"])   # 見逃しの 95% の幅
            if b["OK"] > 0 or ds == "isaac":
                xm = (x0 + x1) // 2
                cv2.line(img, (xm, ax.Y(lo)), (xm, ax.Y(hi)), INK, 1, cv2.LINE_AA)
                for yy in (ax.Y(lo), ax.Y(hi)):
                    cv2.line(img, (xm - 4, yy), (xm + 4, yy), INK, 1, cv2.LINE_AA)
        T.put(img, lab, int(cx), y + h + 40, 14, INK, "center")
    T.put(img, xlabel, x + w // 2, y + h + 64, 14, INK2, "center")


def figure_curve(curves: dict, totals: dict, path: Path) -> Path:
    W_, H_ = 1440, 660
    img = np.full((H_, W_, 3), SURF, np.uint8)
    T = CK.Text()
    T.put(img, "欠陥の大きさ・濃さごとの検出（同じ物差しで、疑似画像と Isaac Sim を比べる）", 28, 38, 22, INK)
    T.put(img, f"不良: 疑似画像 {totals['synthetic']} 枚・Isaac Sim {totals['isaac']} 枚（範囲外の照明は除く）。"
               "黒い縦線 = 見逃しの 95% の幅（Wilson）。棒の上の数字 = 枚数。自作の仮定（欠陥の作り方・照明）での結果",
          28, 66, 14, INK2)
    T.put(img, "Isaac Sim の濃い打痕・汚れ（濃さ 70% 以上）の保留は、点数を出す前の見張り（部品の形・明るさ）に当たったもの。"
               "見逃しは細くて薄い傷（濃さ 20% 未満）", 28, 90, 14, INK2)
    lx = legend_status(img, T, 28, 122)
    legend_ds(img, T, lx + 20, 122)
    bars_panel(img, T, 90, 210, 600, 340, curves["size"], SIZE_LABELS, "大きさ別", "大きさ = 正解の欠陥の外接する四角の長いほうの辺（mm）")
    bars_panel(img, T, 800, 210, 600, 340, curves["contrast"], CON_LABELS, "濃さ別",
               "濃さ = 正解の四角の中で、良品の平均の見え方からの明るさのずれの最大（%）")
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


def figure_tradeoff(sw: dict, tg: dict, wi: list[dict], path: Path) -> Path:
    W_, H_ = 1440, 690
    img = np.full((H_, W_, 3), SURF, np.uint8)
    T = CK.Text()
    T.put(img, "OK の境（しきい値）を動かしたとき: 見逃しと「人に回る割合（保留）」", 28, 38, 22, INK)
    T.put(img, "どちらも学び方は同じ（良品だけ・部品 x 面ごと）。OK の境を全部の面で同じ倍率にして動かし、NG の境はそのまま。"
               "◎ = 今のしきい値（x1）", 28, 66, 14, INK2)
    legend_ds(img, T, 28, 98, line=True)
    # 左: 試験の組の割合（Isaac Sim の 162 枚と同じ: 良品 100・不良 56・範囲外 6）
    a = Axes(img, T, 100, 170, 560, 380, (0, 1), (0, 0.4))
    a.frame([0, 0.2, 0.4, 0.6, 0.8, 1.0], [0, 0.1, 0.2, 0.3, 0.4], lambda v: f"{100 * v:.0f}%", lambda v: f"{100 * v:.0f}%",
            "人に回る割合（保留）: 試験の組の割合（良品 62%・不良 35%・範囲外 4%）", "AI の見逃し（不良を OK にした割合）")
    T.put(img, "AI の見逃しと保留", 50, 134, 17, INK)
    # 右: ラインに近い割合（良品 97%・不良 2%・範囲外 1%）で、人の見逃しも入れた全体の見逃し
    b = Axes(img, T, 800, 170, 560, 380, (0, 1), (0, 0.4))
    b.frame([0, 0.2, 0.4, 0.6, 0.8, 1.0], [0, 0.1, 0.2, 0.3, 0.4], lambda v: f"{100 * v:.0f}%", lambda v: f"{100 * v:.0f}%",
            "人の時間 = 人に回る割合: ラインに近い割合（良品 97%・不良 2%・範囲外 1%、仮定）",
            f"全体の見逃し（AI の見逃し + 保留の不良を人が {100 * HUMAN_MISS:.0f}% 見逃す、仮定）")
    T.put(img, "目標を厳しくしたとき（ナレッジの確かめ）", 750, 134, 17, INK)
    for ds in ("synthetic", "isaac"):
        rows = sw[ds]
        a.line([r["hold"] for r in rows], [r["miss"] for r in rows], DS_COLOR[ds])
        b.line([r["line"]["hold"] for r in rows], [r["line"]["total_miss"] for r in rows], DS_COLOR[ds])
        now = min(rows, key=lambda r: abs(math.log(r["k_ok"])))
        c = a.dot(now["hold"], now["miss"], DS_COLOR[ds], 7)
        cv2.circle(img, c, 12, INK, 1, cv2.LINE_AA)
        T.put(img, f"今: 見逃し {100 * now['miss']:.1f}%・保留 {100 * now['hold']:.1f}%", c[0] + 16, c[1] - 8, 13, INK)
        c = b.dot(now["line"]["hold"], now["line"]["total_miss"], DS_COLOR[ds], 7)
        cv2.circle(img, c, 12, INK, 1, cv2.LINE_AA)
        if ds == "isaac":
            T.put(img, f"今: 人の時間 {100 * now['line']['hold']:.1f}%・全体 {100 * now['line']['total_miss']:.1f}%",
                  c[0] + 16, c[1] - 10, 12, INK)
        for t in tg[ds]:
            L = t["line"]
            if ds == "isaac" and t["target"] in (0.02, 0.0):
                p = b.dot(L["hold"], L["total_miss"], DS_COLOR[ds], 6)
                T.put(img, f"AI の見逃し {'0' if t['target'] == 0 else '≦2'}%: 人の時間 {100 * L['hold']:.0f}%・全体 "
                           f"{100 * L['total_miss']:.1f}%", p[0] + 4, p[1] - 12, 12, INK)
            if ds == "synthetic" and t["target"] == 0.0:   # 点数 0 の不良があるので、見逃し 0 は「全部人へ」
                p = b.dot(L["hold"], L["total_miss"], DS_COLOR[ds], 6)
                T.put(img, f"AI の見逃し 0%: 全部人へ（{100 * L['hold']:.0f}%）・全体 {100 * L['total_miss']:.1f}%",
                      p[0] - 12, p[1] - 12, 12, INK, "right")
    fix = next(w for w in wi if w["case"].startswith("↑"))["line"]   # 見張りを直す + 保留の幅 2 倍（上限）
    cx, cy = b.X(fix["hold"]), b.Y(fix["total_miss"])
    cv2.rectangle(img, (cx - 8, cy - 8), (cx + 8, cy + 8), SURF, -1)
    cv2.rectangle(img, (cx - 6, cy - 6), (cx + 6, cy + 6), DS_COLOR["isaac"], -1)
    T.put(img, f"■ しきい値はそのまま、見張りと保留の幅を直す（上限）: 人の時間 {100 * fix['hold']:.1f}%・全体 "
               f"{100 * fix['total_miss']:.1f}%", 28, 640, 14, INK)
    T.put(img, "見逃し 0 を狙って OK の境を締めるより、保留の原因を直すほうが、人の時間を増やさずに全体の見逃しが減る",
          28, 666, 14, INK2)
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


def figure_factors(groups: list[dict], total: int, path: Path) -> Path:
    W_, H_ = 1440, 560
    img = np.full((H_, W_, 3), SURF, np.uint8)
    T = CK.Text()
    T.put(img, f"Isaac Sim で保留が増えた所（{total} 枚のうち保留 {sum(g['hold'] for g in groups)} 枚の内訳、面のまとまりごと）",
          28, 38, 22, INK)
    x = 28
    for c, col in zip(CAUSES, CAUSE_COLOR):
        cv2.rectangle(img, (x, 66), (x + 14, 80), col, -1)
        x += 20 + T.put(img, c, x + 20, 79, 14, INK2) + 22
    x0, y0, unit, rh = 330, 120, 26, 58
    T.put(img, "保留の枚数", x0, y0 - 8, 13, INK2)
    T.put(img, "枚数・見逃し・過検出・OK の境（良品だけで決めた値）", 760, y0 - 8, 13, INK2)
    for i, g in enumerate(groups):
        y = y0 + 10 + i * rh
        star = "（写り込み）" if g["group"] in (GROUPS[4], GROUPS[5]) else ""
        T.put(img, g["group"], x0 - 14, y + 24, 15, INK, "right")
        if star:
            T.put(img, star, x0 - 14, y + 42, 12, MUTED, "right")
        x = x0
        for c, col in zip(CAUSES, CAUSE_COLOR):
            k = g["causes"][c]
            if k:
                cv2.rectangle(img, (x, y + 8), (x + k * unit - 2, y + 34), col, -1)   # 2px の地の色ですき間
                x += k * unit
        T.put(img, f"{g['hold']}/{g['n']}（{100 * g['hold'] / g['n']:.0f}%）", x + 8, y + 27, 14, INK)
        T.put(img, f"{g['n']} 枚（良品 {g['n_good']}・不良 {g['n_defect']}）  見逃し {g['miss']}・過検出 {g['overkill']}"
                   f"  OK の境 {g['t_ok_min']:,.0f}〜{g['t_ok_max']:,.0f}", 760, y + 27, 14, INK2)
    T.put(img, "OK の境の数字は、良品のばらつきから決めた「点数」。写り込みのある先端の面は良品でも点数が大きく、境が頭の面（3〜4）の"
               "50〜5,000 倍になる。自作の仮定（欠陥の作り方・照明）での結果", 28, H_ - 24, 13, INK2)
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


def _md(rows_md: list[str]) -> None:
    print("\n".join(rows_md))


def report(data: dict) -> dict:
    """表の数字をまとめて、標準出力に Markdown の表で出す（README と LOG.md に貼る）。"""
    iso, big, ref, ref18 = data["isaac"], data["synthetic_big"], data["synthetic_ref"], data["synthetic_ref_18_12"]
    mix = {t: sum(r["truth"] == t for r in iso) for t in ("good", "defect", "out_of_range")}
    for rows in (iso, big, ref, ref18):   # 決め方を作り直しても、検査の判定と同じになることを確かめる
        assert verdicts(rows) == [r["verdict"] for r in rows]
    ks = np.logspace(-3, 1.5, 181)
    out = {"mix": mix,
           "curves": {"size": {"synthetic": binned(big, "size_mm", SIZE_EDGES), "isaac": binned(iso, "size_mm", SIZE_EDGES)},
                      "contrast": {"synthetic": binned(big, "contrast", CON_EDGES), "isaac": binned(iso, "contrast", CON_EDGES)}},
           "sweep": {"synthetic": sweep(big, mix, ks), "isaac": sweep(iso, mix, ks)},
           "targets": {"synthetic": targets(big, mix), "isaac": targets(iso, mix)},
           "groups": {"isaac": group_table(iso), "synthetic": group_table(big)},
           "gain": gain_table(iso), "what_if": what_if(iso, mix),
           "train_count": {"20/20": rates(ref, verdicts(ref), mix), "18/12": rates(ref18, verdicts(ref18), mix)}}

    def by_type(rows):
        df = [r for r in rows if r["truth"] == "defect"]
        res = {}
        for t in DEFECTS:
            sub = [r for r in df if r["types"][0] == t]
            res[t] = {**{v: sum(r["verdict"] == v for r in sub) for v in ("OK", "HOLD", "NG")},
                      "n": len(sub), "pre": sum(r["pre"] for r in sub),
                      "size": float(np.nanmedian([r["size_mm"] for r in sub])),
                      "contrast": float(np.nanmedian([r["contrast"] for r in sub]))}
        return res
    out["types"] = {"isaac": by_type(iso), "synthetic": by_type(big)}

    def cell(b):
        return f"{b['n']}・{pct(b['OK'], b['n'])}・{100 * b['HOLD'] / b['n']:.0f}%" if b["n"] else "なし"

    for name, key, labels in (("大きさ", "size", SIZE_LABELS), ("濃さ", "contrast", CON_LABELS)):
        md = [f"\n## 見逃しの曲線: {name}", "", f"| {name} | 疑似画像: 枚数・見逃し・保留 | Isaac Sim: 枚数・見逃し・保留 |", "|---|---|---|"]
        md += [f"| {lab} | {cell(s_)} | {cell(i_)} |"
               for lab, s_, i_ in zip(labels, out["curves"][key]["synthetic"], out["curves"][key]["isaac"])]
        _md(md)

    def tcell(d):
        return (f"{d['n']} 枚: NG {d['NG']}・保留 {d['HOLD']}（うち見張り {d['pre']}）・OK {d['OK']}、"
                f"{d['size']:.1f}mm・{100 * d['contrast']:.0f}%")
    _md(["\n## 欠陥の種類ごと（大きさ・濃さは中央値）", "", "| 種類 | 疑似画像 | Isaac Sim |", "|---|---|---|"]
        + [f"| {TYPE_JA[t]} | {tcell(out['types']['synthetic'][t])} | {tcell(out['types']['isaac'][t])} |" for t in DEFECTS])

    def near(rows, k):
        return min(rows, key=lambda q: abs(math.log(q["k_ok"] / k)))

    def scell(q):
        return f"{100 * q['miss']:.1f}%・{100 * q['hold']:.1f}%・{100 * q['overkill']:.1f}%"
    _md(["\n## OK の境の倍率と、見逃し・保留・過検出（試験の組の割合）", "",
         "| OK の境の倍率 | 疑似画像: 見逃し・保留・過検出 | Isaac Sim: 見逃し・保留・過検出 |", "|---|---|---|"]
        + [f"| x{k:g} | {scell(near(out['sweep']['synthetic'], k))} | {scell(near(out['sweep']['isaac'], k))} |"
           for k in (0.01, 0.1, 0.3, 1.0, 3.0, 10.0)])

    def gcell(t):
        L = t["line"]
        return f"x{t['k_ok']:.3g}・{100 * L['miss']:.1f}%・{100 * L['hold']:.1f}%・{100 * L['total_miss']:.1f}%"
    _md([f"\n## AI の見逃しの目標を厳しくしたとき（ラインに近い割合、人の見逃し {100 * HUMAN_MISS:.0f}% の仮定）", "",
         "| 目標 | 疑似画像: 倍率・AI の見逃し・人の時間・全体の見逃し | Isaac Sim: 倍率・AI の見逃し・人の時間・全体の見逃し |",
         "|---|---|---|"]
        + [f"| ≦{100 * s_['target']:.0f}% | {gcell(s_)} | {gcell(i_)} |"
           for s_, i_ in zip(out["targets"]["synthetic"], out["targets"]["isaac"])])
    _md(["\n## 面のまとまりごと（Isaac Sim）", "",
         "| 面 | 枚数 | 保留 | 内訳（照明・見張り・不良あいまい・良品） | 見逃し | 過検出 | 不良を自動で NG | OK の境 |",
         "|---|---|---|---|---|---|---|---|"]
        + [f"| {g['group']} | {g['n']} | {g['hold']} | {'・'.join(str(g['causes'][c]) for c in CAUSES)} | "
           f"{g['miss']}/{g['n_defect']} | {g['overkill']}/{g['n_good']} | {g['auto_ng']}/{g['n_defect']} | "
           f"{g['t_ok_min']:,.0f}〜{g['t_ok_max']:,.0f} |" for g in out["groups"]["isaac"]])
    _md(["\n## 面のまとまりごと（疑似画像 1200 枚、比べるため）", "", "| 面 | 枚数 | 保留 | 見逃し | 過検出 |", "|---|---|---|---|---|"]
        + [f"| {g['group']} | {g['n']} | {g['hold']} | {g['miss']}/{g['n_defect']} | {g['overkill']}/{g['n_good']} |"
           for g in out["groups"]["synthetic"]])
    _md(["\n## 照明の明るさ（範囲の中）ごと（Isaac Sim）", "", "| 明るさの倍率 | 枚数 | 保留 | 見逃し | 過検出 |", "|---|---|---|---|---|"]
        + [f"| {g['lo']:.2f}〜{g['hi']:.2f} | {g['n']} | {pct(g['hold'], g['n'])} | {g['miss']}/{g['n_defect']} | "
           f"{g['overkill']}/{g['n_good']} |" for g in out["gain"]])
    _md(["\n## ここを直したら（Isaac Sim、同じ画像・同じ点数で決め方だけ変える）", "",
         "| 場合 | 見逃し | 過検出 | 保留（試験の組） | 人の時間（ライン） | 全体の見逃し |", "|---|---|---|---|---|---|"]
        + [f"| {w['case']} | {100 * w['test']['miss']:.1f}% | {100 * w['test']['overkill']:.1f}% | {100 * w['test']['hold']:.1f}% | "
           f"{100 * w['line']['hold']:.1f}% | {100 * w['test']['total_miss']:.1f}% |" for w in out["what_if"]])
    out["scope"] = {"isaac": scope_table(iso), "synthetic": scope_table(big)}
    md = ["\n## 使用範囲の案（自動: " + "・".join(SCOPE) + "。ほかの面と範囲外の照明は人へ）", "",
          "| | 疑似画像 | Isaac Sim |", "|---|---|---|"]
    for key, name in (("miss", "範囲の中の見逃し"), ("overkill", "範囲の中の過検出"), ("ood_held", "範囲外の照明を保留にできた"),
                      ("strong_miss", "濃さ 20% 以上の不良の見逃し（全部の面）")):
        md.append(f"| {name} | {pct(*out['scope']['synthetic'][key])} | {pct(*out['scope']['isaac'][key])} |")
    md.append("| AI が自分で決めた（請求の対象になる）割合 | "
              + " | ".join(pct(out["scope"][ds]["auto"], out["scope"][ds]["n"]) for ds in ("synthetic", "isaac")) + " |")
    _md(md)
    tc = out["train_count"]
    print(f"\n疑似画像を Isaac Sim と同じ枚数（学び 18・しきい値 12）で学ぶと: 見逃し {100 * tc['20/20']['miss']:.1f}% → "
          f"{100 * tc['18/12']['miss']:.1f}%、保留 {100 * tc['20/20']['hold']:.1f}% → {100 * tc['18/12']['hold']:.1f}%"
          "（Isaac Sim の割合で重みづけ）")
    for ds in ("synthetic", "isaac"):
        v = np.array(list(data["sigma"][ds].values()))
        print(f"良品のばらつき σ（{DS_JA[ds]}）: ROI の中央値の中央値 {np.median(v[:, 0]):.3f}、95 パーセンタイルの最大 {v[:, 1].max():.3f}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n-synthetic", type=int, default=1200, help="曲線用の疑似画像の枚数")
    ap.add_argument("--reuse", action="store_true", help="out/miss_curve.json を読み直すだけ（画像を判定し直さない）")
    ap.add_argument("--no-figure", action="store_true")
    args = ap.parse_args(argv)
    path = OUT / "miss_curve.json"
    if args.reuse and path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        t0 = time.monotonic()
        iso, cal, sig_i = collect_isaac()
        print(f"Isaac Sim: {len(iso)} 枚（{time.monotonic() - t0:.0f} 秒）")
        ref, ins_s = collect_synthetic(200, 7, 0.3, 0.05)
        big, _ = collect_synthetic(args.n_synthetic, 11, 0.35, 0.04, ins=ins_s)
        # 同じ枚数で学んだら？（Isaac Sim と同じ: 学び 18 枚・しきい値 12 枚）
        ref18, _ = collect_synthetic(200, 7, 0.3, 0.05, n_fit=18, n_val=12)
        print(f"疑似画像: 200 + {len(big)} + 200 枚（{time.monotonic() - t0:.0f} 秒）")
        data = {"isaac": iso, "synthetic_ref": ref, "synthetic_big": big, "synthetic_ref_18_12": ref18,
                "isaac_calibration": cal, "sigma": {"isaac": sig_i, "synthetic": sigma_stats(ins_s)}}
    out = report(data)
    data["tables"] = out
    path.write_text(json.dumps(data, ensure_ascii=False, indent=0, default=float), encoding="utf-8")
    if not args.no_figure:
        totals = {ds: sum(b["n"] for b in out["curves"]["size"][ds]) for ds in ("synthetic", "isaac")}
        for p in (figure_curve(out["curves"], totals, DOCS / "isaac_miss_curve.png"),
                  figure_tradeoff(out["sweep"], out["targets"], out["what_if"], DOCS / "isaac_hold_tradeoff.png"),
                  figure_factors(out["groups"]["isaac"], len(data["isaac"]), DOCS / "isaac_hold_factors.png")):
            print(f"絵: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
