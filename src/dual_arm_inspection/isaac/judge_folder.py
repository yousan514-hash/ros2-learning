"""Isaac Sim で撮った画像を、ROS 2 の検査ノードと同じやり方（inspection.py）で判定する（pixi の環境で動かす）。

    pixi run python src/dual_arm_inspection/isaac/judge_folder.py --good out/good --test out/test

  --good  欠陥なしで撮ったフォルダ（capture_inspection.py --good-only --repeat 6 など）。ここだけで学ぶ
  --test  欠陥ありで撮ったフォルダ（capture_inspection.py --defects 0.5 など）。OK／NG／保留の表を出す

疑似カメラの画像で学んだしきい値は Isaac Sim の画像には合わないので、必ず Isaac Sim の良品で学び直す。
（撮り方が変わったら学び直す = 範囲外の見張りの考え方と同じ）
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from dual_arm_inspection import cell as C  # noqa: E402
from dual_arm_inspection import inspection as I  # noqa: E402


def load(folder: Path):
    doc = json.loads((folder / "shots.json").read_text(encoding="utf-8"))
    for rec in doc["shots"]:
        img = cv2.imdecode(np.fromfile(str(folder / rec["file"]), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(folder / rec["file"])
        yield rec, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--good", required=True)
    ap.add_argument("--test", required=True)
    ap.add_argument("--n-fit", type=int, default=0, help="学びに使う枚数（0 なら良品の 6 割）")
    args = ap.parse_args(argv)

    parts, _ = C.load_parts()
    ins = I.Inspector(parts)
    samples = defaultdict(list)
    for rec, img in load(Path(args.good)):
        samples[(rec["part"], rec["face_type"])].append(img)
    n_min = min(len(v) for v in samples.values())
    n_fit = args.n_fit or max(2, int(n_min * 0.6))
    cal = ins.fit(dict(samples), n_fit)
    print(f"学び: 型 {cal['templates']} 個、しきい値 OK < {cal['t_ok']:.1f} ≦ 保留 < {cal['t_ng']:.1f} ≦ NG")

    table = {t: {v: 0 for v in ("OK", "NG", "HOLD")} for t in ("good", "defect", "out_of_range")}
    for rec, img in load(Path(args.test)):
        res = ins.inspect(img, rec["part"], rec["face_type"])
        truth = "good" if rec["good"] else "defect"
        table[truth][res.verdict] += 1
        print(f"{rec['file']}: {res.verdict} ({res.defect_type or res.reason}, score {res.score:.1f}) [truth: {truth}]")
    for k, row in table.items():
        print(f"{k:<14}{row}")
    print({k: round(v, 3) for k, v in I.summary(table).items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
