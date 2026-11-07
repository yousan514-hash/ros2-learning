"""保証つきのしきい値: 「使用範囲の中なら、見逃しは α 以下（確からしさ 1−δ）」と言える OK の境を、
今の Isaac Sim の画像と疑似画像で作り、その限界を数字で出す（pixi の環境で動かす）。

    pixi run isaac_guarantee     # = python src/dual_arm_inspection/isaac/guarantee_threshold.py（約 2.5 分）

決め方は dual_arm_inspection/guarantee.py（Learn then Test・Conformal Risk Control の、見逃しが単調な場合）。
動かすのは OK の境の倍率 k だけ（各レシピの t_ok x k。k = 1 が今の境）。NG の境はそのまま（ただし OK の境より下にしない）。

やること
  1. Isaac Sim（今の out/good・out/test。撮り直さない）: 使用範囲 v2 の中の不良は何枚で、今のデータで約束できる見逃しの
     上限はいくつか。2% を約束するには何枚要るか
  2. 疑似画像（その場で作る）: 不良全部を母集団とみなし、校正の不良 n 枚で決めた k が、母集団での見逃しの率で約束（α）を
     破った回の割合を出す（300 回）。今の境・保証のない決め方・CRC と比べる。約束を「すべての濃さ」と「濃さ 20% 以上」の 2 通りで
     （すべての濃さ: 点数 0 の見えない不良があるので α = 5%。濃さ 20% 以上: α = 2%）
  3. 限界: 条件がずれると保証は破れるか。疑似画像で決めた k を (a) 同じ条件の別の画像 (b) 薄い欠陥が多い疑似画像
     (c) Isaac Sim にそのまま使う
  4. 使用範囲書 v3 の案（isaac/scope_v3_draft.json）と、絵（docs/isaac_guarantee.png）
数字はすべて自作の仮定（欠陥の作り方・照明・部品の形）での結果。実物では確かめていない。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
ROOT = HERE.parents[2]
for p in (PKG, HERE, ROOT / "tools"):
    sys.path.insert(0, str(p))
import analyze_miss_curve as A  # noqa: E402  Isaac Sim と疑似画像の集め方・物差し・使用範囲 v2 の面を使い回す
import check_dual_arm as CK  # noqa: E402
from dual_arm_inspection import guarantee as G  # noqa: E402
from dual_arm_inspection import imaging as IM  # noqa: E402

OUT = HERE / "out"
DOCS = ROOT / "docs"
DELTA = 0.10          # 約束が外れてよい確率（確からしさ 90%）
DELTAS = (0.10, 0.05)
STRONG = 0.20         # 使用範囲書 v2 の「検出を約束する濃さ」
REPS = 300
LINE = {"good": 0.97, "defect": 0.02, "out_of_range": 0.01}   # ラインに近い割合（analyze_miss_curve と同じ仮定）
# 約束の 2 通り: (名前, 約束する見逃しの率 α, 濃さの下限, 確かめの校正の枚数, ずれの確かめの校正の枚数)
SETTINGS = (("all", 0.05, None, 300, 1000), ("strong", 0.02, STRONG, 300, 500))
SETTING_JA = {"all": "すべての濃さ・α 5%", "strong": "濃さ 20% 以上・α 2%"}


# ---------------------------------------------------------------- 行をまとめて判定する
def in_scope(rows, all_faces: bool = False) -> list[dict]:
    """使用範囲の中: 範囲外の照明を除き、v2 の 3 つの面だけ（all_faces なら v1 = 全部の面）。"""
    return [r for r in rows if r["truth"] != "out_of_range" and (all_faces or A.group_of(r) in A.SCOPE)]


def arrays(rows) -> dict:
    f = lambda k: np.array([r[k] for r in rows], float)  # noqa: E731
    a = {"pre": np.array([bool(r["pre"]) for r in rows]), "score": f("score"), "t_ok": f("t_ok"),
         "t_ng": f("t_ng"), "contrast": np.array([r.get("contrast", float("nan")) for r in rows], float),
         "defect": np.array([r["truth"] == "defect" for r in rows]), "good": np.array([r["truth"] == "good" for r in rows])}
    a["ratio"] = np.where(a["pre"], np.inf, a["score"] / a["t_ok"])   # 不良なら「r < k で見逃し」
    a["strong"] = a["defect"] & (np.nan_to_num(a["contrast"], nan=-1.0) >= STRONG)
    return a


def subset(a: dict, min_contrast: float | None) -> np.ndarray:
    """約束の対象の不良（濃さの下限があれば、その濃さ以上）の番号。"""
    sel = a["defect"] if min_contrast is None else a["defect"] & (np.nan_to_num(a["contrast"], nan=-1.0) >= min_contrast)
    return np.flatnonzero(sel)


def judge(a: dict, k: float) -> tuple[np.ndarray, np.ndarray]:
    """analyze_miss_curve.verdicts と同じ決め方（OK の境 x k、NG の境はそのまま・OK の境より下にしない）。"""
    t_ok = a["t_ok"] * k
    t_ng = np.maximum(a["t_ng"], t_ok)
    ok = ~a["pre"] & (a["score"] < t_ok)
    ng = ~a["pre"] & ~ok & (a["score"] >= t_ng)
    return ok, ng


def stats(a: dict, k: float, idx=None) -> dict:
    ok, ng = judge(a, k)
    sel = np.ones(len(ok), bool) if idx is None else np.isin(np.arange(len(ok)), idx)
    d, g, s = sel & a["defect"], sel & a["good"], sel & a["strong"]
    hold = sel & ~ok & ~ng
    hg = float(hold[g].mean()) if g.any() else float("nan")
    hd = float(hold[d].mean()) if d.any() else float("nan")
    return {"k": float(k), "n_defect": int(d.sum()), "miss": int((ok & d).sum()),
            "n_strong": int(s.sum()), "miss_strong": int((ok & s).sum()),
            "n_good": int(g.sum()), "overkill": int((ng & g).sum()), "hold_good": hg, "hold_defect": hd,
            "hold_scope": float(hold[sel].mean()), "auto_scope": float(1 - hold[sel].mean()),
            "hold_line": LINE["good"] * hg + LINE["defect"] * (0.0 if math.isnan(hd) else hd) + LINE["out_of_range"]}


def risk(st: dict, min_contrast: float | None) -> tuple[int, int]:
    """約束の対象での (見逃し, 不良の枚数)。"""
    return (st["miss"], st["n_defect"]) if min_contrast is None else (st["miss_strong"], st["n_strong"])


# ---------------------------------------------------------------- 1. Isaac Sim（今の画像）
def isaac_part(iso: list[dict]) -> dict:
    out = {"needed": {}}
    for alpha in (0.02, 0.05):
        for delta in DELTAS:
            for m in (0, 1):
                out["needed"][f"a{alpha}_d{delta}_m{m}"] = G.n_needed(alpha, delta, m)
    cases = {"v2": (in_scope(iso), None), "v2_strong": (in_scope(iso), STRONG), "v1": (in_scope(iso, True), None)}
    for name, (rows, mc) in cases.items():
        a = arrays(rows)
        r = a["ratio"][subset(a, mc)]
        n, m_now = len(r), G.misses_at(r, 1.0)
        row = {"n": n, "misses_now": m_now, "now": stats(a, 1.0), "zero_score": int(np.sum(r == 0)),
               "bound_now": {str(d): G.upper_bound(m_now, n, d) for d in DELTAS},
               "alpha_min": {str(d): G.upper_bound(0, n, d) for d in DELTAS}, "ltt": {}}
        for alpha in (0.02, 0.05, 0.10, 0.15):
            g = G.guaranteed_threshold(r, alpha, DELTA)
            row["ltt"][str(alpha)] = {**g, "stats": stats(a, g["k"])}
        row["auto_of_all_now"] = row["now"]["auto_scope"] * len(rows) / len(iso)   # 範囲の外の面は人へ、と数える
        out[name] = row
    return out


# ---------------------------------------------------------------- 2. 疑似画像で確かめる
def collect_faint(n: int, seed: int, ins, power: float = 3.0) -> list[dict]:
    """条件のずれ（欠陥の分布）: 欠陥の強さを rng.random()**power にした疑似画像（ふつうは **1.6）。全部に欠陥を入れる。"""
    rng = np.random.default_rng(seed)
    plist = list(ins.parts.values())
    rows = []
    for _ in range(n):
        part = plist[int(rng.integers(len(plist)))]
        face = part.faces[int(rng.integers(len(part.faces)))]
        light = IM.sample_lighting(rng, 0.0)
        defects = [{"type": str(rng.choice(IM.DEFECT_TYPES)), "severity": float(rng.random() ** power)}]
        img, truth = IM.render(part, face.face_type, rng, defects, light)
        res = ins.inspect(img, part.name, face.face_type)
        x, y, w, h = truth["defects"][0]["bbox"]
        contrast = A.peak_contrast(ins, img, part.name, face.face_type, (x, y, x + w - 1, y + h - 1))
        rows.append(A.row_of(ins, res, img, ds="faint", part=part.name, kind=part.kind, face=face.name,
                             key=face.face_type, where=face.where, truth="defect", contrast=contrast,
                             severity=truth["defects"][0]["severity"]))
    return rows


def validate(a: dict, alpha: float, min_contrast: float | None, n_cal: int, seed: int = 5) -> dict:
    """疑似画像の不良全部を「母集団」とみなし、そこから校正の不良 n_cal 枚を選んで k を決め、母集団での見逃しの率
    （= 本当の見逃しの率）が α を超えたかを数える。REPS 回くり返す。LTT なら、超えた回は δ（10%）以下になるはず。"""
    rng = np.random.default_rng(seed)
    D = subset(a, min_contrast)
    cache: dict[float, float] = {}

    def run(method, n):
        res = []
        for _ in range(REPS):
            cal = rng.choice(D, size=n, replace=True)   # 母集団（疑似画像の不良全部）から、入れ替えありで選ぶ
            k = 1.0 if method == "now" else G.guaranteed_threshold(a["ratio"][cal], alpha, DELTA, method)["k"]
            if k not in cache:
                cache[k] = float(judge(a, k)[0][D].mean())   # 本当の見逃しの率 = 母集団での率（測りの揺れなし）
            res.append((k, cache[k]))
        return np.array(res).T

    out = {"alpha": alpha, "min_contrast": min_contrast, "n_cal": n_cal, "n_defect": int(len(D)),
           "n_good": int(a["good"].sum()), "zero_share": float(np.mean(a["ratio"][D] == 0)), "rules": {}, "by_n": {}}
    for method in ("now", "naive", "crc", "ltt"):
        ks, ms = run(method, n_cal)
        st = stats(a, float(np.median(ks)))
        out["rules"][method] = {"violation": float(np.mean(ms > alpha)), "miss_mean": float(ms.mean()),
                                "miss_p90": float(np.percentile(ms, 90)), "k_median": float(np.median(ks)),
                                "all_hold": float(np.mean(ks == 0.0)), "hold_good": st["hold_good"],
                                "hold_line": st["hold_line"]}
    for n in (150, 300, 1000):
        ks, ms = run("ltt", n)
        st = stats(a, float(np.median(ks)))
        out["by_n"][str(n)] = {"violation": float(np.mean(ms > alpha)), "miss_mean": float(ms.mean()),
                               "k_median": float(np.median(ks)), "all_hold": float(np.mean(ks == 0.0)),
                               "hold_good": st["hold_good"], "hold_line": st["hold_line"]}
    return out


# ---------------------------------------------------------------- 3. 条件のずれ
def shift_part(a_pool: dict, a_faint: dict, a_iso: dict, alpha: float, min_contrast: float | None, n_cal: int,
               seed: int = 9) -> dict:
    rng = np.random.default_rng(seed)
    perm = rng.permutation(subset(a_pool, min_contrast))
    cal, test = perm[:n_cal], perm[n_cal:]
    g = G.guaranteed_threshold(a_pool["ratio"][cal], alpha, DELTA)
    out = {"alpha": alpha, "min_contrast": min_contrast, "n_cal": n_cal, "k": g["k"], "cal": g, "cases": {}}
    test_idx = np.concatenate([test, np.flatnonzero(a_pool["good"])])
    for name, a, idx in (("same", a_pool, test_idx), ("faint", a_faint, None), ("isaac", a_iso, None)):
        st = stats(a, g["k"], idx)
        m, n = risk(st, min_contrast)
        out["cases"][name] = {**st, "risk_miss": m, "risk_n": n, "bound": G.upper_bound(m, n, DELTA),
                              "violated": n > 0 and m / n > alpha}
    return out


# ---------------------------------------------------------------- 4. 使用範囲書 v3 の案
def scope_v3(res: dict) -> dict:
    iso = res["isaac"]
    s = iso["v2_strong"]
    val = res["validate"]["strong"]
    sh = res["shift"]["strong"]
    return {
        "draft": True,
        "note": "瞳の案（2026-11-08）。今の台帳の UsageScope の欄（version・auto_faces・human_faces・min_contrast・basis）は "
                "v2 と同じ形。guarantee_proposal は新しい欄の提案で、台帳に入れるか・名前は千恵が決める。"
                "数字は自作の Isaac Sim と疑似画像だけ（実物では未確認）",
        "version": "v3",
        "name": "瞳の案 v3（v2 の面と濃さ + 見逃しの上限の約束）",
        "auto_faces": ["bolt_*/side_top", "nut_*/top", "washer_*/top", "bolt_*/head_end", "nut_*/bottom", "washer_*/bottom"],
        "human_faces": ["bolt_*/tip_end", "nut_*/side", "bolt_*/side_bottom"],
        "min_contrast": STRONG,
        "basis": f"guarantee_threshold.py（2026-11-08）。Isaac Sim 162 枚のうち範囲の中で濃さ 20% 以上の不良 {s['n']} 枚、"
                 f"疑似画像の濃さ 20% 以上の不良 {val['n_defect']} 枚で確かめた",
        "guarantee_proposal": {
            "risk": "使用範囲の中（auto_faces・照明は範囲内）で、濃さ min_contrast 以上の不良のうち、AI が OK にした割合",
            "method": "Learn then Test（二項分布の片側の上限。OK の境の倍率 k について見逃しは単調）",
            "confidence": 1 - DELTA,
            "now_with_isaac_images": {
                "k_ok": 1.0, "n_defects": s["n"], "misses": s["misses_now"],
                "miss_rate_max": round(s["bound_now"][str(DELTA)], 4),
                "comment": "今の Isaac Sim の画像で約束できるのはここまで（不良の枚数が少ないため）。2% には足りない"},
            "target_after_more_defects": {
                "miss_rate_max": 0.02,
                "n_defects_needed_if_0_miss": iso["needed"]["a0.02_d0.1_m0"],
                "n_defects_needed_if_1_miss": iso["needed"]["a0.02_d0.1_m1"],
                "comment": "伊佐美の撮り直し（範囲の中の不良を増やす）のあと、校正の画像で k を決め直して書き込む",
                "synthetic_check": f"疑似画像では、校正 {val['n_cal']} 枚で決めた k が約束（2%）を破った回は "
                                   f"{100 * val['rules']['ltt']['violation']:.1f}%（{REPS} 回、許す割合 10%）。"
                                   f"ただし疑似画像の校正 {sh['n_cal']} 枚で決めた k = {sh['k']:.3f} を Isaac Sim に使うと、"
                                   f"見逃し {sh['cases']['isaac']['risk_miss']}/{sh['cases']['isaac']['risk_n']} で約束を破った。"
                                   "k は、判定するのと同じ撮り方の画像で校正する"},
            "valid_while": ["しきい値の版（thresholds）が校正のときと同じ", "照明・背景・部品の形が範囲外の見張りの中",
                            "撮り方（カメラ・照明・倍率）と欠陥の測り方（濃さ）の版が校正のときと同じ。変えたら校正し直して版を上げる",
                            "実物では未確認（合成の画像だけの約束）。実物の答え合わせのあとで、実物の校正に置き換える"],
            "audit": {"rule": "工場がわざと流す不良の見本（濃さ min_contrast 以上）n 枚のうち、AI が OK にした数 m について "
                              "P(Bin(n, miss_rate_max) ≧ m) ≦ 0.05 なら版を止める（guarantee.audit_stop）。止めたあとの判定は数えない",
                      "example": "上限 2% のとき、見本 50 枚で 4 枚以上見逃したら止める"},
        },
    }


# ---------------------------------------------------------------- 絵（OpenCV。matplotlib は使わない）
SURF, INK, INK2, MUTED = CK.SURFACE, CK.INK, CK.INK2, CK.MUTED
BLUE, ORANGE, GREEN, YELLOW = (CK.bgr(h) for h in ("#2a78d6", "#eb6834", "#1baf7a", "#eda100"))


def dashed_h(img, x0, x1, y, color):
    for x in range(x0, x1, 12):
        cv2.line(img, (x, y), (min(x + 6, x1), y), color, 1, cv2.LINE_AA)


def whisker(img, ax, xm, lo, hi):
    cv2.line(img, (xm, ax.Y(lo)), (xm, ax.Y(hi)), INK, 1, cv2.LINE_AA)
    for yy in (ax.Y(lo), ax.Y(hi)):
        cv2.line(img, (xm - 4, yy), (xm + 4, yy), INK, 1, cv2.LINE_AA)


def figure(res: dict, path: Path) -> Path:
    W_, H_ = 1500, 870
    img = np.full((H_, W_, 3), SURF, np.uint8)
    T = CK.Text()
    iso, val, sh = res["isaac"], res["validate"], res["shift"]
    T.put(img, "保証つきのしきい値:「使用範囲の中なら見逃しは α 以下（確からしさ 90%）」と言えるか", 28, 38, 22, INK)
    T.put(img, "動かすのは OK の境の倍率だけ（各面の t_ok x k）。決め方は Learn then Test（二項分布の片側の上限）。"
               "数字はすべて自作の Isaac Sim と疑似画像（実物では未確認）", 28, 66, 14, INK2)

    # 1. 約束できる上限と不良の枚数
    ax = A.Axes(img, T, 90, 170, 380, 400, (10, 1000), (0, 0.25), xlog=True)
    ax.frame([10, 30, 100, 300, 1000], [0, 0.05, 0.10, 0.15, 0.20, 0.25], lambda v: f"{v:g}", lambda v: f"{100 * v:.0f}%",
             "範囲の中の不良の枚数（校正に使う）", "見逃し 0 枚のとき約束できる上限")
    T.put(img, "1. 不良の枚数で、約束できる上限が決まる", 40, 122, 17, INK)
    ns = np.geomspace(10, 1000, 80)
    for d, col in ((0.10, BLUE), (0.05, GREEN)):
        ax.line(ns, [G.upper_bound(0, int(n), d) for n in ns], col)
    dashed_h(img, ax.X(10), ax.X(1000), ax.Y(0.02), INK2)
    T.put(img, "目標 2%", ax.X(1000) - 4, ax.Y(0.02) - 6, 12, INK2, "right")
    T.put(img, "確からしさ 90%", ax.X(160), ax.Y(0.235), 13, BLUE)
    T.put(img, "確からしさ 95%", ax.X(160), ax.Y(0.215), 13, GREEN)
    n2 = iso["needed"]["a0.02_d0.1_m0"]
    c = ax.dot(n2, 0.02, BLUE, 5)
    T.put(img, f"{n2} 枚で 2%", c[0] - 4, c[1] - 12, 12, INK, "right")
    for key, lab, dy in (("v1", "Isaac 全部の面", -12), ("v2", "Isaac v2 の面", 22), ("v2_strong", "Isaac v2・濃さ20%以上", -14)):
        q = iso[key]
        c = ax.dot(q["n"], q["bound_now"][str(DELTA)], ORANGE, 6)
        T.put(img, f"{lab}: 見逃し {q['misses_now']}/{q['n']} → {100 * q['bound_now'][str(DELTA)]:.1f}%",
              c[0] + 10, c[1] + dy, 12, INK)

    # 2. 疑似画像で確かめる
    bx, by, bw, bh = 590, 170, 400, 400
    bxx = A.Axes(img, T, bx, by, bw, bh, (0, 8), (0, 1))
    bxx.frame([], [0, 0.25, 0.5, 0.75, 1.0], str, lambda v: f"{100 * v:.0f}%", "", "約束を破った回の割合", vgrid=False)
    T.put(img, f"2. 同じ条件で確かめる（疑似画像、校正 300 枚 x {REPS} 回）", 540, 122, 17, INK)
    dashed_h(img, bx, bx + bw, bxx.Y(DELTA), INK2)
    T.put(img, "許す割合 10%", bx + 4, bxx.Y(DELTA) - 6, 12, INK2)
    labels = {"now": "今", "naive": "保証なし", "crc": "CRC", "ltt": "LTT"}
    cols = {"now": MUTED, "naive": YELLOW, "crc": GREEN, "ltt": BLUE}
    v = val["all"]   # 濃さ 20% 以上は、疑似画像ではどの k でも見逃しがなく、どの決め方も破らない（下に文で書く）
    for i, mth in enumerate(("now", "naive", "crc", "ltt")):
        q = v["rules"][mth]
        cx = bx + int(bw * (i + 0.5) / 4)
        yv = bxx.Y(q["violation"])
        cv2.rectangle(img, (cx - 30, yv), (cx + 30, bxx.Y(0)), cols[mth], -1)
        T.put(img, f"{100 * q['violation']:.0f}%", cx, yv - 6, 13, INK, "center")
        T.put(img, labels[mth], cx, by + bh + 20, 13, INK, "center")
        T.put(img, f"見逃し 平均 {100 * q['miss_mean']:.1f}%", cx, by + bh + 40, 11, INK2, "center")
        T.put(img, f"良品の保留 {100 * q['hold_good']:.1f}%", cx, by + bh + 56, 11, INK2, "center")
    T.put(img, f"約束 = すべての濃さで見逃し 5% 以下。母集団 = 範囲の中の不良 {v['n_defect']:,} 枚", 540, by + bh + 80, 12, INK2)

    # 3. 条件のずれ
    cx0, cy0, cw, ch = 1110, 170, 360, 400
    tops = []
    for name, *_r in SETTINGS:
        for q in sh[name]["cases"].values():
            if q["risk_n"]:
                tops.append(A.wilson(q["risk_miss"], q["risk_n"])[2])
    ytop = min(1.0, max(0.10, math.ceil(max(tops) * 1.1 * 20) / 20))
    cxx = A.Axes(img, T, cx0, cy0, cw, ch, (0, 3), (0, ytop))
    step = 0.05 if ytop <= 0.3 else 0.1
    cxx.frame([], list(np.arange(0, ytop + 1e-9, step)), str, lambda v: f"{100 * v:.0f}%", "", "見逃しの率（95% の幅）", vgrid=False)
    T.put(img, "3. 条件がずれると破れる", 1070, 122, 17, INK)
    for name, col in (("all", BLUE), ("strong", GREEN)):
        dashed_h(img, cx0, cx0 + cw, cxx.Y(sh[name]["alpha"]), col)
    names = {"same": "同じ条件（疑似）", "faint": "薄い欠陥が多い", "isaac": "Isaac Sim（v2）"}
    for i, case in enumerate(("same", "faint", "isaac")):
        cx = cx0 + int(cw * (i + 0.5) / 3)
        for j, (name, col) in enumerate((("all", BLUE), ("strong", GREEN))):
            q = sh[name]["cases"][case]
            x0 = cx - 36 if j == 0 else cx + 4
            x1 = x0 + 32
            if q["risk_n"] == 0:
                continue
            p, lo, hi = A.wilson(q["risk_miss"], q["risk_n"])
            cv2.rectangle(img, (x0, cxx.Y(p)), (x1, cxx.Y(0)), col, -1)
            whisker(img, cxx, (x0 + x1) // 2, lo, hi)
            T.put(img, f"{q['risk_miss']}/{q['risk_n']}", (x0 + x1) // 2, cxx.Y(hi) - 6, 11, INK2, "center")
        T.put(img, names[case], cx, cy0 + ch + 20, 13, INK, "center")
    for j, (name, col) in enumerate((("all", BLUE), ("strong", GREEN))):
        yy = cy0 + ch + 46 + 22 * j
        cv2.rectangle(img, (cx0, yy - 11), (cx0 + 14, yy + 1), col, -1)
        T.put(img, f"{SETTING_JA[name]}（校正 {sh[name]['n_cal']} 枚で k = {sh[name]['k']:.2f}）", cx0 + 20, yy, 13, INK2)
    T.put(img, "破線 = 約束（青 5%・緑 2%）。k は疑似画像だけで決めた", cx0, cy0 + ch + 90, 12, INK2)

    v2 = iso["v2_strong"]
    v, s = val["all"], val["strong"]
    T.put(img, f"今の Isaac Sim で約束できるのは「範囲の中・濃さ 20% 以上の見逃しは {100 * v2['bound_now'][str(DELTA)]:.1f}% 以下"
               f"（確からしさ 90%）」まで（見逃し {v2['misses_now']}/{v2['n']}）。2% には、見逃し 0 枚で不良 {n2} 枚が要る", 28, 724, 15, INK)
    T.put(img, f"同じ条件なら LTT は約束を守る（破った回 {100 * v['rules']['ltt']['violation']:.0f}% ≦ 10%）。保証のない決め方は "
               f"{100 * v['rules']['naive']['violation']:.0f}%、CRC（期待値の保証）は {100 * v['rules']['crc']['violation']:.0f}% の回で破る。"
               "校正の不良を増やすと良品の保留が減る（LTT: " + "・".join(f"{n} 枚 {100 * q['hold_good']:.1f}%" for n, q in v["by_n"].items())
          + "）", 28, 752, 14, INK2)
    T.put(img, f"すべての濃さでは点数 0 の見えない不良が {100 * v['zero_share']:.1f}% あり、2% は約束しにくい。濃さ 20% 以上（疑似画像の不良 "
               f"{s['n_defect']:,} 枚）は k = 3 でも見逃し 0 で、どの決め方も破らない", 28, 778, 14, INK2)
    fa, fi = sh["all"]["cases"]["faint"], sh["strong"]["cases"]["isaac"]
    T.put(img, f"ただし疑似画像で決めた k は、薄い欠陥が多い画像で 5% の約束を破り（{100 * fa['risk_miss'] / max(fa['risk_n'], 1):.1f}%）、"
               f"Isaac Sim では濃さ 20% 以上の 2% の約束を破った（{fi['risk_miss']}/{fi['risk_n']}）。", 28, 810, 15, INK)
    T.put(img, "合成で決めた約束は、欠陥の分布や画像の作り方が変わると守られない → 実物での答え合わせと、実物での校正が要る",
          28, 838, 15, INK)
    path.write_bytes(cv2.imencode(".png", img)[1].tobytes())
    return path


# ---------------------------------------------------------------- 通し
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n-pool", type=int, default=8000, help="確かめ用の疑似画像の枚数（不良は 6 割）")
    ap.add_argument("--n-faint", type=int, default=3000, help="薄い欠陥が多い疑似画像の枚数")
    ap.add_argument("--no-figure", action="store_true")
    args = ap.parse_args(argv)
    t0 = time.monotonic()
    iso, _, _ = A.collect_isaac()
    print(f"Isaac Sim: {len(iso)} 枚（{time.monotonic() - t0:.0f} 秒）")
    _, ins = A.collect_synthetic(10, 1, 0.5, 0.0)            # 疑似カメラの良品で学ぶ（analyze_miss_curve と同じ）
    pool, _ = A.collect_synthetic(args.n_pool, 21, 0.6, 0.0, ins=ins)
    faint = collect_faint(args.n_faint, 31, ins)
    print(f"疑似画像: {len(pool)} + 薄い欠陥 {len(faint)} 枚（{time.monotonic() - t0:.0f} 秒）")

    # 決め直した判定が、検査の判定と同じか（k = 1 のとき）
    for rows in (iso, pool, faint):
        ok, ng = judge(arrays(rows), 1.0)
        v = np.where(ok, "OK", np.where(ng, "NG", "HOLD"))
        assert list(v) == [r["verdict"] for r in rows], "k=1 の判定が検査の判定と合わない"

    a_pool, a_faint, a_iso = arrays(in_scope(pool)), arrays(in_scope(faint)), arrays(in_scope(iso))
    res = {"delta": DELTA, "isaac": isaac_part(iso), "validate": {}, "shift": {},
           "pool_now": stats(a_pool, 1.0), "faint_now": stats(a_faint, 1.0),
           "contrast_median": {"pool": float(np.nanmedian(a_pool["contrast"][a_pool["defect"]])),
                               "faint": float(np.nanmedian(a_faint["contrast"][a_faint["defect"]])),
                               "isaac": float(np.nanmedian(a_iso["contrast"][a_iso["defect"]]))}}
    for name, alpha, mc, n_val, n_shift in SETTINGS:
        res["validate"][name] = validate(a_pool, alpha, mc, n_val)
        res["shift"][name] = shift_part(a_pool, a_faint, a_iso, alpha, mc, n_shift)
    res["scope_v3"] = scope_v3(res)
    OUT.mkdir(exist_ok=True)
    (OUT / "guarantee.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    (HERE / "scope_v3_draft.json").write_text(json.dumps(res["scope_v3"], ensure_ascii=False, indent=1) + "\n",
                                              encoding="utf-8")
    report(res)
    if not args.no_figure:
        print(f"絵: {figure(res, DOCS / 'isaac_guarantee.png')}")
    print(f"（{time.monotonic() - t0:.0f} 秒）")
    return 0


def report(res: dict) -> None:
    iso = res["isaac"]
    pc = lambda v: f"{100 * v:.1f}%"  # noqa: E731
    print("\n## 1. Isaac Sim（今の画像）で約束できる上限")
    print("| 範囲 | 不良 | 今の見逃し | 上限 90% | 上限 95% | 見逃し 0 でも言える下限 90% | 162 枚のうち AI が決めた |")
    print("|---|---|---|---|---|---|---|")
    for key, name in (("v1", "v1 全部の面"), ("v2", "v2 の 3 つの面"), ("v2_strong", "v2・濃さ 20% 以上")):
        q = iso[key]
        print(f"| {name} | {q['n']} | {q['misses_now']} | {pc(q['bound_now']['0.1'])} | {pc(q['bound_now']['0.05'])} | "
              f"{pc(q['alpha_min']['0.1'])} | {pc(q['auto_of_all_now'])} |")
    print("要る不良の枚数（α・δ・見逃し）:", iso["needed"])
    for key in ("v1", "v2", "v2_strong"):
        for al, g in iso[key]["ltt"].items():
            s = g["stats"]
            print(f"  {key} α={al}: できる={g['feasible']} k={g['k']:.3f} 見逃し {g['misses']}/{g['n']} 範囲の中の保留 "
                  f"{pc(s['hold_scope'])}（今 {pc(iso[key]['now']['hold_scope'])}）良品の保留 {pc(s['hold_good'])}")
    for name, v in res["validate"].items():
        print(f"\n## 2. 疑似画像で確かめる: {SETTING_JA[name]}（不良 {v['n_defect']} 枚・良品 {v['n_good']} 枚、"
              f"点数 0 の不良 {pc(v['zero_share'])}、校正 {v['n_cal']} 枚、{REPS} 回）")
        print("| 決め方 | 約束を破った回 | 見逃し（平均） | 見逃し（上位 10%） | k（中央値） | 全部人へ | 良品の保留 | ラインの保留 |")
        print("|---|---|---|---|---|---|---|---|")
        for m, q in v["rules"].items():
            print(f"| {m} | {pc(q['violation'])} | {pc(q['miss_mean'])} | {pc(q['miss_p90'])} | {q['k_median']:.3f} | "
                  f"{pc(q['all_hold'])} | {pc(q['hold_good'])} | {pc(q['hold_line'])} |")
        for n, q in v["by_n"].items():
            print(f"  LTT 校正 {n} 枚: 破った回 {pc(q['violation'])}・見逃し {pc(q['miss_mean'])}・k {q['k_median']:.3f}・"
                  f"全部人へ {pc(q['all_hold'])}・良品の保留 {pc(q['hold_good'])}・ラインの保留 {pc(q['hold_line'])}")
    for name, s in res["shift"].items():
        print(f"\n## 3. 条件のずれ: {SETTING_JA[name]}（疑似画像の校正 {s['n_cal']} 枚で k = {s['k']:.3f}）")
        for case, q in s["cases"].items():
            print(f"  {case}: 見逃し {q['risk_miss']}/{q['risk_n']}（{pc(q['risk_miss'] / max(q['risk_n'], 1))}、"
                  f"上限 90% {pc(q['bound'])}、破った={q['violated']}）良品の保留 {pc(q['hold_good'])}")
    print("\n濃さの中央値:", {k: pc(v) for k, v in res["contrast_median"].items()})
    print("今の境（k=1）: 疑似画像", res["pool_now"], "\n薄い欠陥", res["faint_now"])


if __name__ == "__main__":
    raise SystemExit(main())
