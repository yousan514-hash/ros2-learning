"""伊佐美の撮り直し（使用範囲の中の不良を増やす・写り込みの 2 面の撮り方を直す）を数える（pixi の環境で動かす）。

    pixi run python src/dual_arm_inspection/isaac/summarize_scope.py                    # scope_sets.json の組を全部
    pixi run python src/dual_arm_inspection/isaac/summarize_scope.py --fit DIR --test DIR [DIR ...]   # フォルダを指定

数えるもの（保証つきのしきい値は瞳が作る。ここでは作らない）:
  - 組ごと・面のまとまりごとの良品・不良・範囲外の照明の枚数、不良の種類
  - 不良の濃さ（瞳の物差し analyze_miss_curve.peak_contrast: 正解の四角の中で、良品の平均の見え方からの明るさの
    ずれの最大）と、ねらいの濃さ（撮るときに決めた値）との関係
  - 今の検査（judge_folder.py と同じ学び方: 学ぶ用の良品の 6 割で学び、残りで OK・NG の境）での OK／NG／保留。
    写り込みの 2 面は、前の撮り方と直した撮り方を同じ欠陥の作り方で撮り、保留の割合で比べる
書き出すもの: <root>/summary.json（表の数字と 1 枚ずつの判定）
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
ROOT = HERE.parents[2]
for p in (PKG, HERE, ROOT / "tools"):
    sys.path.insert(0, str(p))
import analyze_miss_curve as A  # noqa: E402  濃さの物差し（peak_contrast）と 95% の幅（wilson）を使い回す
import judge_folder as JF  # noqa: E402  画像の読み方と学ぶ単位（部品 x 面）を使い回す
from dual_arm_inspection import cell as C  # noqa: E402
from dual_arm_inspection import inspection as I  # noqa: E402

DEFECTS = ("scratch", "dent", "stain")
TYPE_JA = {"scratch": "傷", "dent": "打痕", "stain": "汚れ"}
GROUP_JA = {"tray": "トレイの上（上から）", "head": "ボルトの頭の面（手）", "under": "ナット・座金の下（手）",
            "tip": "ボルトの先端の面（手）", "nut_side": "ナットの横（手）", "side_hand": "ボルトの横（手）"}
BINS = [(0.0, 0.1), (0.1, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.9), (0.9, 9.0)]


def truth_of(rec: dict) -> str:
    return "out_of_range" if rec.get("ood") else ("defect" if rec.get("defects") else "good")


def fit_inspector(folder: Path, parts: dict) -> tuple[I.Inspector, dict, int]:
    """学ぶ用のフォルダの良品だけで学ぶ（judge_folder.py と同じ: 6 割で学び、残りでしきい値）。"""
    ins = I.Inspector(list(parts.values()))
    samples = defaultdict(list)
    for rec, img in JF.load(folder):
        if truth_of(rec) == "good":
            samples[(rec["part"], JF.key(rec))].append(img)
    n_min = min(len(v) for v in samples.values())
    n_fit = max(2, int(n_min * 0.6))
    cal = ins.fit(dict(samples), n_fit)
    return ins, cal, n_fit


def defect_box(rec: dict):
    """Replicator の外接する四角（意味のラベルが欠陥のもの）。なければ None。"""
    if not rec.get("bbox") or not rec.get("bbox_labels"):
        return None
    labels = {int(k): v["class"] for k, v in rec["bbox_labels"].items()}
    boxes = [b for b in rec["bbox"] if labels.get(int(b[0])) in DEFECTS]
    return boxes[0][1:5] if boxes else None


def judge(ins: I.Inspector, folder: Path, parts: dict, set_name: str) -> list[dict]:
    rows = []
    for rec, img in JF.load(folder):
        part, key = parts[rec["part"]], JF.key(rec)
        if (part.name, key) not in ins.templates:
            continue
        res = ins.inspect(img, part.name, key)
        row = {"set": set_name, "file": rec["file"], "part": part.name, "kind": part.kind, "face": rec["face"],
               "group": rec.get("group", ""), "truth": truth_of(rec), "verdict": res.verdict, "reason": res.reason,
               "score": round(float(res.score), 3), "t_ok": round(float(ins.templates[(part.name, key)].t_ok), 3),
               "type": None, "target": None, "contrast": None, "size_px": None, "size_mm": None}
        if rec.get("defects"):
            d = rec["defects"][0]
            row.update(type=d["type"], target=d.get("target_contrast"), size_mm=d.get("size_mm"))
            box = defect_box(rec)
            if box is not None:
                x0, y0, x1, y1 = box
                row["size_px"] = float(max(x1 - x0 + 1, y1 - y0 + 1))
                c = A.peak_contrast(ins, img, part.name, key, (x0, y0, x1, y1))
                row["contrast"] = None if math.isnan(c) else round(float(c), 4)
        rows.append(row)
    return rows


# ---------------------------------------------------------------- まとめ
def pct(k: int, n: int) -> str:
    if not n:
        return "—"
    p, lo, hi = A.wilson(k, n)
    return f"{100 * p:.1f}%（{k}/{n}、{100 * lo:.0f}〜{100 * hi:.0f}%）"


def group_table(rows: list[dict]) -> dict:
    out = {}
    for g in sorted({r["group"] for r in rows}, key=lambda x: list(GROUP_JA).index(x) if x in GROUP_JA else 99):
        rs = [r for r in rows if r["group"] == g]
        good = [r for r in rs if r["truth"] == "good"]
        df = [r for r in rs if r["truth"] == "defect"]
        ood = [r for r in rs if r["truth"] == "out_of_range"]
        inr = good + df
        cs = np.array([r["contrast"] for r in df if r["contrast"] is not None], float)
        out[g] = {
            "n_good": len(good), "n_defect": len(df), "n_ood": len(ood),
            "types": dict(Counter(r["type"] for r in df)),
            "contrast_n": int(len(cs)),
            "contrast_q": [round(float(v), 3) for v in np.percentile(cs, [0, 25, 50, 75, 100])] if len(cs) else None,
            "contrast_bins": [int(((cs >= lo) & (cs < hi)).sum()) for lo, hi in BINS] if len(cs) else None,
            "good": dict(Counter(r["verdict"] for r in good)), "defect": dict(Counter(r["verdict"] for r in df)),
            "ood_hold": sum(r["verdict"] == "HOLD" for r in ood),
            "hold": sum(r["verdict"] == "HOLD" for r in inr), "n_in_range": len(inr),
            "miss": sum(r["verdict"] == "OK" for r in df), "overkill": sum(r["verdict"] == "NG" for r in good),
            "hold_reasons": dict(Counter(r["reason"].split(":")[0] for r in inr if r["verdict"] == "HOLD")),
            "t_ok": sorted({r["t_ok"] for r in rs}),
        }
    return out


def contrast_calibration(rows: list[dict]) -> dict:
    """ねらいの濃さ（撮るときに決めた値）と、測った濃さ（瞳の物差し）の関係を種類ごとに。"""
    out = {}
    for t in DEFECTS:
        rs = [r for r in rows if r["type"] == t and r["contrast"] is not None and r["target"] is not None]
        if not rs:
            continue
        tg = np.array([r["target"] for r in rs])
        ms = np.array([r["contrast"] for r in rs])
        bins = []
        for lo in np.arange(0.1, 0.9, 0.2):
            sel = (tg >= lo) & (tg < lo + 0.2)
            if sel.any():
                bins.append({"target": [round(float(lo), 1), round(float(lo + 0.2), 1)], "n": int(sel.sum()),
                             "measured_median": round(float(np.median(ms[sel])), 3),
                             "measured_range": [round(float(ms[sel].min()), 3), round(float(ms[sel].max()), 3)]})
        out[t] = {"n": len(rs), "corr": round(float(np.corrcoef(tg, ms)[0, 1]), 3) if len(rs) > 2 else None,
                  "bins": bins}
    return out


def print_groups(title: str, gt: dict):
    print(f"\n### {title}")
    print("| 面 | 良品 | 不良（打痕・傷・汚れ） | 範囲外 | 濃さ（最小・中央・最大） | 濃さ 10〜90% | 良品 OK/NG/保留 | 不良 OK/NG/保留 | 保留（範囲の中） |")
    print("|---|---|---|---|---|---|---|---|---|")
    for g, s in gt.items():
        ty = s["types"]
        q = s["contrast_q"]
        inb = sum(s["contrast_bins"][1:5]) if s["contrast_bins"] else 0
        gv = "/".join(str(s["good"].get(v, 0)) for v in ("OK", "NG", "HOLD"))
        dv = "/".join(str(s["defect"].get(v, 0)) for v in ("OK", "NG", "HOLD"))
        cq = f"{100 * q[0]:.0f}・{100 * q[2]:.0f}・{100 * q[4]:.0f}%" if q else "—"
        print(f"| {GROUP_JA.get(g, g)} | {s['n_good']} | {s['n_defect']}（{ty.get('dent', 0)}・{ty.get('scratch', 0)}・"
              f"{ty.get('stain', 0)}） | {s['n_ood']} | {cq} | {inb}/{s['contrast_n']} | {gv} | {dv} | "
              f"{pct(s['hold'], s['n_in_range'])} |")


def _tile(folder: Path, rec: dict, label: str, T, w: int = 240, h: int = 180) -> np.ndarray:
    """1 枚を縮めて、上に見出しを付ける（OpenCV の BGR のまま）。"""
    img = cv2.imdecode(np.fromfile(str(folder / rec["file"]), np.uint8), cv2.IMREAD_COLOR)
    img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    pad = np.full((26, w, 3), A.CK.SURFACE, np.uint8)
    T.put(pad, label, w // 2, 19, 14, A.CK.INK, "center")
    tile = np.vstack([pad, img])
    cv2.rectangle(tile, (0, 0), (w - 1, h + 25), A.CK.SURFACE, 2)
    return tile


def make_figure(root: Path, rows: list[dict], out: Path):
    """見本の絵: 上 3 段 = しきい値を決める用の不良（測った濃さが 10〜90% の帯から 1 枚ずつ、種類ごと）、
    下 2 段 = 写り込みの 2 面の前の撮り方と直した撮り方（良品）。"""
    T = A.CK.Text()
    tiles_rows = []
    df = [r for r in rows if r["set"] == "calib" and r["truth"] == "defect" and r["contrast"] is not None]
    for typ in ("stain", "dent", "scratch"):
        row = []
        for lo in (0.1, 0.3, 0.5, 0.7):
            cand = [r for r in df if r["type"] == typ and lo <= r["contrast"] < lo + 0.2]
            if cand:
                r = sorted(cand, key=lambda r: abs(r["contrast"] - (lo + 0.1)))[0]
                rec = {"file": r["file"]}
                row.append(_tile(root / "calib", rec, f"{TYPE_JA[typ]} 濃さ {100 * r['contrast']:.0f}%", T))
        if row:
            tiles_rows.append(row)
    for names in (("ab/tip_legacy/test", "ab/tip_collar/test", "ab/tip_collar03/test", "ab/tip_wide/test"),
                  ("ab/nut_legacy/test", "ab/nut_flat/test", "ab/nut_flat_big/test")):
        row = []
        for n in names:
            folder = root / n
            if not (folder / "shots.json").exists():
                continue
            recs = json.loads((folder / "shots.json").read_text(encoding="utf-8"))["shots"]
            rec = next(r for r in recs if not r["defects"] and r["part"].endswith(("M6x30", "M6")))
            label = {"tip_legacy": "先端: 前", "tip_collar": "先端: 穴あき板 0.05mm", "tip_collar03": "先端: 穴あき板 0.3mm",
                     "tip_wide": "先端: 画角を広げる", "nut_legacy": "ナットの横: 前（角が正面）",
                     "nut_flat": "ナットの横: 平らな面を正面", "nut_flat_big": "平らな面を正面 + 背景板 40cm"}[n.split("/")[1]]
            row.append(_tile(folder, rec, label, T))
        if row:
            tiles_rows.append(row)
    width = max(len(r) for r in tiles_rows)
    blank = np.full_like(tiles_rows[0][0], 255)
    img = np.vstack([np.hstack(r + [blank] * (width - len(r))) for r in tiles_rows])
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))[1].tofile(str(out))
    print(f"絵: {out}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(HERE / "scope_sets.json"))
    ap.add_argument("--root", default=str(HERE / "out" / "scope"), help="組のフォルダがある場所")
    ap.add_argument("--fit", default="", help="学ぶ用のフォルダ（--test と一緒に）")
    ap.add_argument("--test", nargs="*", default=[], help="判定するフォルダ")
    ap.add_argument("--json", default="", help="書き出す JSON（既定は <root>/summary.json）")
    ap.add_argument("--figure", default="", help="見本の絵を書き出す PNG（例: docs/isaac_scope_shots.png）")
    args = ap.parse_args(argv)

    parts = {p.name: p for p in C.load_parts()[0]}
    root = Path(args.root)
    if args.fit:
        pairs = [(Path(args.fit), [Path(t) for t in args.test], Path(args.fit).name)]
    else:
        cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
        by_fit = defaultdict(list)
        for s in cfg["sets"]:
            if s.get("fit_with"):
                by_fit[s["fit_with"]].append(root / s["name"])
        pairs = [(root / f, tests, f) for f, tests in by_fit.items()]

    import hashlib

    src = Path(I.__file__).read_bytes()
    # 判定に使った inspection.py の指紋（作業中の版と、コミットした版のどちらで数えたかを後で確かめられるように）
    result = {"inspection_py": {"path": str(Path(I.__file__)), "sha256": hashlib.sha256(src).hexdigest()}, "pairs": {}}
    print(f"inspection.py: {Path(I.__file__)} sha256 {result['inspection_py']['sha256'][:12]}")
    all_rows = []
    for fit_dir, tests, label in pairs:
        ins, cal, n_fit = fit_inspector(fit_dir, parts)
        sig = A.sigma_stats(ins)
        entry = {"fit": str(fit_dir), "n_fit": n_fit, "calibration": cal, "sigma95_max": round(max(v[1] for v in sig.values()), 4),
                 "t_ok": {k: round(float(t.t_ok), 2) for k, t in ((f"{p}|{k}", t) for (p, k), t in ins.templates.items())},
                 "tests": {}}
        n_good_fit = sum(1 for rec, _ in JF.load(fit_dir) if truth_of(rec) == "good")
        print(f"\n## 学ぶ用: {fit_dir.name}（良品 {n_good_fit} 枚、レシピ {len(ins.templates)} 個、1 レシピ {n_fit} 枚で学び残りでしきい値）"
              f" OK の境 {cal['t_ok_range'][0]:.1f}〜{cal['t_ok_range'][1]:.1f}、良品のばらつき σ の 95 パーセンタイルの最大 {entry['sigma95_max']:.3f}")
        for t in tests:
            rows = judge(ins, t, parts, t.relative_to(root).as_posix() if t.is_relative_to(root) else t.name)
            all_rows += rows
            gt = group_table(rows)
            entry["tests"][str(t)] = {"groups": gt, "contrast_calibration": contrast_calibration(rows)}
            print_groups(f"{t.relative_to(root) if t.is_relative_to(root) else t}（{len(rows)} 枚）", gt)
            cc = entry["tests"][str(t)]["contrast_calibration"]
            if cc:
                print("ねらいの濃さ → 測った濃さ（中央値）: " + "; ".join(
                    f"{TYPE_JA[k]} " + ", ".join(f"{int(100 * b['target'][0])}-{int(100 * b['target'][1])}%→{100 * b['measured_median']:.0f}%"
                                                 for b in v["bins"]) for k, v in cc.items()))
        result["pairs"][label] = entry
    result["rows"] = all_rows
    out = Path(args.json) if args.json else root / "summary.json"
    out.write_text(json.dumps(result, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(f"\n書き出した: {out}")
    if args.figure:
        make_figure(root, all_rows, Path(args.figure))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
