"""双腕の外観検査セルを、RViz も ROS も起動せずに確かめる。

    pixi run check_dual_arm

確かめること:
  1. 動きの計画: 全部の目標（キーフレーム）で逆運動学が解けたか・上下限の中か・解いた関節角で手先が目標に合うか
  2. 動きの途中（1 周を 50Hz で刻んだ全部の点）: 上下限・腕どうしのいちばん近い距離・机と胴体とのすき間・関節の速さ
  3. 逆運動学そのもの: 上下限の中の関節角をでたらめに 2000 通り作り、手先の姿勢から解き直して戻るか
  4. 検査: 良品だけで学び、ラインと同じ確率（欠陥 30%・範囲外 5%）で 200 枚撮って、OK／NG／保留の表を作る
最後に 1 枚の絵（docs/dual_arm_inspection.png）にまとめる。matplotlib は使わず OpenCV で描く。
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "dual_arm_inspection"))
from dual_arm_inspection import cell as C  # noqa: E402
from dual_arm_inspection import inspection as I  # noqa: E402
from dual_arm_inspection import planner as P  # noqa: E402
from dual_arm_inspection.kinematics import VELOCITY  # noqa: E402

OUT = ROOT / "docs" / "dual_arm_inspection.png"
RATE = 50.0
N_SHOTS = 200


# ---------------------------------------------------------------- 1〜3: 動き
def check_motion():
    t0 = time.monotonic()
    plan = P.plan_cycle()
    plan_time = time.monotonic() - t0
    tl = P.Timeline(plan.segments)

    kf_ok, max_dp, max_dr = 0, 0.0, 0.0
    for kf in plan.keyframes:
        arm = plan.arms[kf.arm]
        dp, dr = arm.pose_error(kf.q, kf.R, kf.p)
        max_dp, max_dr = max(max_dp, dp), max(max_dr, dr)
        kf_ok += int(arm.within_limits(kf.q) and dp < 1e-9 and dr < 1e-9)

    ts = np.arange(0.0, tl.total + 1e-9, 1.0 / RATE)
    gaps, envs, labels = [], [], []
    limit_ng, prev, max_vel, min_q5 = 0, None, 0.0, math.inf
    for t in ts:
        st, seg = tl.sample(float(t))
        gap, env = plan.clearance(st.q["l"], st.q["r"], st.held, st.variant)
        gaps.append(gap)
        envs.append(env)
        labels.append(seg.label)
        for k in ("l", "r"):
            limit_ng += int(not plan.arms[k].within_limits(st.q[k]))
            min_q5 = min(min_q5, abs(float(st.q[k][4])))
        q = np.concatenate([st.q["l"], st.q["r"]])
        if prev is not None:
            max_vel = max(max_vel, float(np.max(np.abs(q - prev))) * RATE)
        prev = q
    gaps, envs = np.array(gaps), np.array(envs)
    caps = [(float(tl.starts[i + 1]) - plan.segments[i].duration, d)
            for i, s in enumerate(plan.segments) for k, d in s.events if k == "capture"]

    arms = plan.arms
    rng = np.random.default_rng(20261106)
    rt = {}
    for k, arm in arms.items():
        ok, worst = 0, 0.0
        for _ in range(2000):
            q = rng.uniform(arm.lower, arm.upper)
            R, p = arm.fk(q)
            sol = arm.ik(R, p, seed=rng.uniform(arm.lower, arm.upper))
            if sol is not None and arm.within_limits(sol):
                dp, dr = arm.pose_error(sol, R, p)
                worst = max(worst, dp, dr)
                ok += int(dp < 1e-9 and dr < 1e-9)
        rt[k] = (ok, worst)

    i_gap, i_env = int(np.argmin(gaps)), int(np.argmin(envs))
    return {
        "plan": plan, "timeline": tl, "plan_time": plan_time, "ts": ts, "gaps": gaps, "envs": envs,
        "labels": labels, "captures": caps, "kf_ok": kf_ok, "kf_n": len(plan.keyframes),
        "max_dp": max_dp, "max_dr": max_dr, "limit_ng": limit_ng, "max_vel": max_vel, "min_q5": min_q5,
        "min_gap": (float(gaps[i_gap]), float(ts[i_gap]), labels[i_gap]),
        "min_env": (float(envs[i_env]), float(ts[i_env]), labels[i_env]), "roundtrip": rt,
    }


# ---------------------------------------------------------------- 4: 検査
def check_inspection():
    parts, _ = C.load_parts()
    ins = I.Inspector(parts)
    t0 = time.monotonic()
    cal = ins.calibrate()
    cal_time = time.monotonic() - t0
    rows = I.evaluate(ins, n=N_SHOTS, seed=7, p_defect=0.3, p_ood=0.05)
    table = I.confusion(rows)
    return {"inspector": ins, "cal": cal, "cal_time": cal_time, "rows": rows, "table": table,
            "summary": I.summary(table)}


# ---------------------------------------------------------------- 絵
def bgr(hexstr: str) -> tuple[int, int, int]:
    h = hexstr.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))


SURFACE, INK, INK2, MUTED = bgr("#fcfcfb"), bgr("#0b0b0b"), bgr("#52514e"), bgr("#8a897f")
GRID, AXIS, SERIES = bgr("#e1e0d9"), bgr("#c3c2b7"), bgr("#2a78d6")
STATUS = {"OK": bgr("#0ca30c"), "NG": bgr("#d03b3b"), "HOLD": bgr("#fab219")}
STATUS_INK = {"OK": (255, 255, 255), "NG": (255, 255, 255), "HOLD": INK}
JA = {"OK": "OK", "NG": "NG", "HOLD": "保留"}


class Text:
    """日本語の字（OpenCV の freetype + Windows の BIZ UDゴシック）。なければ ASCII だけで描く。"""

    def __init__(self):
        self.ft = None
        for name in ("BIZ-UDGothicR.ttc", "YuGothM.ttc", "meiryo.ttc", "msgothic.ttc"):
            path = Path(r"C:\Windows\Fonts") / name
            if path.exists() and hasattr(cv2, "freetype"):
                self.ft = cv2.freetype.createFreeType2()
                self.ft.loadFontData(str(path), 0)
                break

    def put(self, img, text, x, y, size=18, color=INK, anchor="left"):
        """(x, y) は字の下の線（ベースライン）。anchor = left / center / right。"""
        if self.ft is None:
            text = text.encode("ascii", "replace").decode()
            w = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size / 30, 1)[0][0]
        else:
            w = self.ft.getTextSize(text, size, -1)[0][0]
        if anchor == "center":
            x -= w // 2
        elif anchor == "right":
            x -= w
        if self.ft is None:
            cv2.putText(img, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size / 30, color, 1, cv2.LINE_AA)
        else:
            self.ft.putText(img, text, (int(x), int(y)), size, color, -1, cv2.LINE_AA, True)
        return w


def project(eye_yaw=-0.55, eye_pitch=0.42):
    e = np.array([math.cos(eye_pitch) * math.cos(eye_yaw), math.cos(eye_pitch) * math.sin(eye_yaw),
                  math.sin(eye_pitch)])
    d = -e
    right = np.cross(d, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, d)
    return right, up, d, e


def draw_cell(img, T, box, motion, insp_face=("bolt_M6x30", "head_end")):
    """セルの見取り図（右前の上から見た正射影）。見せて撮っている瞬間の姿勢。パネルの外にははみ出さない。"""
    x0, y0, w, h = box
    canvas = np.full((h, w, 3), SURFACE, np.uint8)
    plan, tl = motion["plan"], motion["timeline"]
    t_cap = next(t for t, d in motion["captures"] if (d["part"], d["face"]) == insp_face)
    st, _ = tl.sample(t_cap)
    right, up, d, e = project()
    scale, cx, cy = 430.0, w * 0.50, h * 0.56
    center = np.array([0.36, 0.0, 1.0])

    def px(p):
        q = np.asarray(p) - center
        return (int(cx + scale * float(q @ right)), int(cy - scale * float(q @ up)))

    def depth(p):
        return float((np.asarray(p) - center) @ d)

    boxes = []
    tc, ts = C.TABLE_CENTER, C.TABLE_SIZE
    trc, trs = C.TRAY_CENTER, C.TRAY_SIZE
    cut = 0.62  # 胴体は机より下を切って描く
    boxes.append(((tc[0] - ts[0] / 2, tc[1] - ts[1] / 2, C.TABLE_TOP_Z - ts[2]),
                  (tc[0] + ts[0] / 2, tc[1] + ts[1] / 2, C.TABLE_TOP_Z), bgr("#c2a880")))
    boxes.append(((trc[0] - trs[0] / 2, trc[1] - trs[1] / 2, C.TABLE_TOP_Z),
                  (trc[0] + trs[0] / 2, trc[1] + trs[1] / 2, C.TRAY_TOP_Z), bgr("#34546f")))
    boxes.append(((C.TORSO_BOX[0][0], C.TORSO_BOX[0][1], cut), C.TORSO_BOX[1], bgr("#5c6670")))
    boxes.append((C.SHOULDER_BOX[0], C.SHOULDER_BOX[1], bgr("#5c6670")))
    light = np.array([0.3, -0.2, 0.93])

    def draw_box(lo, hi, color):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        corners = np.array([[(lo, hi)[(i >> a) & 1][a] for a in range(3)] for i in range(8)])
        for axis in range(3):
            for side in (0, 1):
                n = np.zeros(3)
                n[axis] = 1.0 if side else -1.0
                if n @ e <= 0:
                    continue
                idx = [i for i in range(8) if ((i >> axis) & 1) == side]
                quad = corners[[idx[0], idx[1], idx[3], idx[2]]]
                col = tuple(int(c * (0.78 + 0.22 * float(n @ light))) for c in color)
                pts = np.array([px(p) for p in quad], np.int32)
                cv2.fillPoly(canvas, [pts], col, cv2.LINE_AA)
                cv2.polylines(canvas, [pts], True, tuple(int(c * 0.65) for c in col), 1, cv2.LINE_AA)

    # 胴体は机の奥なので先に描き、机 → その上のトレイの順に重ねる
    for lo, hi, col in (boxes[2], boxes[3], boxes[0], boxes[1]):
        draw_box(lo, hi, col)
    for p in plan.parts.values():  # トレイの部品（小さい点）
        pp = p.tray_pose()[:3, 3]
        cv2.circle(canvas, px(pp), 3, bgr("#c9ccd1"), -1, cv2.LINE_AA)

    arms = []
    for k in ("l", "r"):
        caps = plan.capsules(k, st.q[k], st.held if k == "l" else None, st.variant)
        arms.append((depth(plan.arms[k].points(st.q[k])["elbow"]), k, caps))
    for _, k, caps in sorted(arms, key=lambda a: -a[0]):
        for a, b, r in caps[:3]:
            th = max(3, int(2 * r * scale * 0.8))
            cv2.line(canvas, px(a), px(b), bgr("#3b3f45"), th + 3, cv2.LINE_AA)
            cv2.line(canvas, px(a), px(b), bgr("#d4d7db") if k == "l" else bgr("#dfe3e8"), th, cv2.LINE_AA)
        for p in (caps[0][0], caps[0][1], caps[1][1]):
            cv2.circle(canvas, px(p), max(4, int(0.045 * scale * 0.8)), bgr("#40454c"), -1, cv2.LINE_AA)

    Rr, pr = plan.arms["r"].fk(st.q["r"])
    part = plan.parts[insp_face[0]]
    Rl, pl = plan.arms["l"].fk(st.q["l"])
    T_wp = C.make_T(Rl, pl) @ part.tool_to_part(st.variant)
    face = next(f for f in part.faces if f.name == insp_face[1])
    target = (T_wp @ np.append(face.center, 1.0))[:3]
    cv2.line(canvas, px(pr), px(target), SERIES, 2, cv2.LINE_AA)
    cv2.rectangle(canvas, (px(pr)[0] - 6, px(pr)[1] - 6), (px(pr)[0] + 6, px(pr)[1] + 6), bgr("#26406f"), -1)
    cv2.circle(canvas, px(T_wp[:3, 3]), 6, bgr("#b0b3b8"), -1, cv2.LINE_AA)
    cv2.circle(canvas, px(T_wp[:3, 3]), 6, INK2, 1, cv2.LINE_AA)

    def label(text, p, dx, dy, size=16, color=INK, anchor="left"):
        x = min(max(p[0] + dx, 8), w - 8)
        y = min(max(p[1] + dy, 40), h - 40)
        T.put(canvas, text, x, y, size, color, anchor)

    T.put(canvas, "セルの見取り図（右前の上から。左腕が部品を見せ、右腕のカメラが撮っている瞬間）", 12, 26, 17, INK)
    # 正面から見るので、ロボットの左腕は絵の右側に来る
    for k, text, dx, anchor in (("l", "左腕: 持って見せる（グリッパ）", 120, "left"),
                                ("r", "右腕: 手首カメラで撮る", -120, "right")):
        el = px(plan.arms[k].points(st.q[k])["elbow"])
        tx, ty = min(max(el[0] + dx, 8), w - 8), max(el[1] - 30, 60)
        cv2.line(canvas, (tx + (6 if anchor == "right" else -6), ty - 6), el, INK2, 1, cv2.LINE_AA)
        cv2.circle(canvas, el, 3, INK2, -1, cv2.LINE_AA)
        T.put(canvas, text, tx, ty, 16, INK, anchor)
    label(f"視線（面から {C.STANDOFF:.2f} m）", px(target), 14, 34, 14, SERIES)
    label("部品トレイ（9 個）", px((C.TRAY_CENTER[0] + 0.12, C.TRAY_CENTER[1] - 0.10, C.TRAY_TOP_Z)), 0, 24, 14, INK2)
    T.put(canvas, f"撮っている面: {insp_face[0]} / {insp_face[1]}", 12, h - 14, 14, INK2)
    img[y0:y0 + h, x0:x0 + w] = canvas


def draw_gap_chart(img, T, box, motion):
    """腕どうしのいちばん近い距離（1 本の線）と下限。"""
    x0, y0, w, h = box
    ts, gaps = motion["ts"], motion["gaps"]
    left, right, top, bottom = x0 + 64, x0 + w - 20, y0 + 52, y0 + h - 46
    ymax = max(0.35, float(np.ceil(gaps.max() * 10) / 10))
    xmax = float(ts[-1])

    def P_(t, g):
        return (int(left + (right - left) * t / xmax), int(bottom - (bottom - top) * g / ymax))

    T.put(img, f"腕どうしのいちばん近い距離（m）— 1 周 {xmax:.0f} 秒、{RATE:.0f}Hz で刻んだ全部の点", x0 + 12, y0 + 26,
          18, INK)
    for gv in np.arange(0, ymax + 1e-9, 0.1):
        y = P_(0, gv)[1]
        cv2.line(img, (left, y), (right, y), GRID, 1)
        T.put(img, f"{gv:.1f}", left - 8, y + 5, 13, MUTED, "right")
    cv2.line(img, (left, bottom), (right, bottom), AXIS, 1)
    for tv in range(0, int(xmax) + 1, 30):
        x = P_(tv, 0)[0]
        cv2.line(img, (x, bottom), (x, bottom + 4), AXIS, 1)
        T.put(img, f"{tv}", x, bottom + 20, 13, MUTED, "center")
    T.put(img, "秒", right, bottom + 38, 13, MUTED, "right")
    for t, _ in motion["captures"]:  # 撮った時刻
        x = P_(t, 0)[0]
        cv2.line(img, (x, bottom - 6), (x, bottom), MUTED, 1)
    ym = P_(0, C.ARM_MARGIN)[1]
    for xs in range(left, right, 12):
        cv2.line(img, (xs, ym), (min(xs + 6, right), ym), INK2, 1)
    T.put(img, f"下限 {C.ARM_MARGIN:.2f} m", right - 4, ym - 6, 13, INK2, "right")
    pts = np.array([P_(t, g) for t, g in zip(ts, gaps)], np.int32)
    cv2.polylines(img, [pts], False, SERIES, 2, cv2.LINE_AA)
    g, t, label = motion["min_gap"]
    p = P_(t, g)
    cv2.circle(img, p, 5, SERIES, -1, cv2.LINE_AA)
    cv2.circle(img, p, 7, SURFACE, 2, cv2.LINE_AA)
    tx, ty = left + 12, top + 16
    cv2.line(img, (min(p[0], right), ty + 6), p, INK2, 1, cv2.LINE_AA)
    T.put(img, f"最小 {g:.3f} m（{t:.0f} 秒: {label}）", tx, ty, 14, INK)
    T.put(img, "x 軸の上の短い線 = 撮った時刻", left + 4, bottom + 38, 13, MUTED)


def draw_table(img, T, box, insp):
    x0, y0, w, h = box
    tab, s, cal = insp["table"], insp["summary"], insp["cal"]
    T.put(img, f"検査の当たり外れ（疑似画像 {N_SHOTS} 枚、欠陥 30%・範囲外 5% の割合で撮影）", x0 + 12, y0 + 26, 18, INK)
    rows = [("good", "良品"), ("defect", "不良"), ("out_of_range", "範囲外（照明）")]
    cols = ["OK", "NG", "HOLD"]
    gx, gy, cw, rh = x0 + 150, y0 + 56, 96, 40
    for j, c in enumerate(cols):
        x = gx + j * cw
        cv2.rectangle(img, (x + 6, gy), (x + cw - 6, gy + 30), STATUS[c], -1, cv2.LINE_AA)
        T.put(img, JA[c] if c != "HOLD" else "保留（人へ）", x + cw // 2, gy + 21, 15, STATUS_INK[c], "center")
    T.put(img, "正解 ＼ 判定", gx - 12, gy + 21, 14, INK2, "right")
    good_cells = {("good", "OK"), ("defect", "NG"), ("out_of_range", "HOLD")}
    for i, (key, name) in enumerate(rows):
        y = gy + 38 + i * rh
        T.put(img, name, gx - 12, y + 26, 16, INK, "right")
        for j, c in enumerate(cols):
            x = gx + j * cw
            fill = bgr("#eef4ea") if (key, c) in good_cells else SURFACE
            cv2.rectangle(img, (x + 6, y + 4), (x + cw - 6, y + rh - 2), fill, -1)
            cv2.rectangle(img, (x + 6, y + 4), (x + cw - 6, y + rh - 2), GRID, 1)
            T.put(img, str(tab[key][c]), x + cw // 2, y + 28, 19, INK, "center")
        T.put(img, f"計 {sum(tab[key].values())}", gx + 3 * cw + 12, y + 27, 14, MUTED)
    mx = x0 + 12
    my = gy + 38 + 3 * rh + 30
    lines = [
        f"見逃し（不良を OK）: {tab['defect']['OK']} / {sum(tab['defect'].values())} 枚 = {s['miss_rate']:.1%}",
        f"過検出（良品を NG）: {tab['good']['NG']} / {sum(tab['good'].values())} 枚 = {s['overkill_rate']:.1%}",
        f"人に回る（保留）: {s['hold_rate']:.1%}　範囲外を保留にできた: {s['ood_caught']:.0%}",
        f"自動で決めた分の正解率: {s['auto_accuracy']:.1%}",
        f"しきい値は良品だけで決めた: OK < {cal['t_ok']:.1f} ≦ 保留 < {cal['t_ng']:.1f} ≦ NG（点数）",
    ]
    for k, line in enumerate(lines):
        T.put(img, line, mx, my + k * 24, 15, INK if k < 4 else INK2)
    T.put(img, "数値は自作の疑似画像での結果。欠陥の強さや照明の仮定で変わる（実物では未検証）", mx, my + len(lines) * 24 + 4, 13,
          MUTED)


def pick_examples(rows):
    want = [
        lambda r: r["truth"] == "good" and r["verdict"] == "OK",
        lambda r: r["verdict"] == "NG" and r["pred_type"] == "scratch",
        lambda r: r["verdict"] == "NG" and r["pred_type"] == "dent",
        lambda r: r["verdict"] == "NG" and r["pred_type"] == "stain",
        lambda r: r["verdict"] == "HOLD" and r["reason"] == "uncertain",
        lambda r: r["verdict"] == "HOLD" and r["reason"].startswith("out_of_range"),
        lambda r: r["verdict"] == "HOLD" and r["reason"] == "shape",
        lambda r: r["truth"] == "defect" and r["verdict"] == "OK",
    ]
    out, used = [], set()
    for f in want:
        r = next((r for r in rows if f(r) and r["i"] not in used), None)
        if r is not None:
            out.append(r)
            used.add(r["i"])
    return out[:8]


TRUTH_JA = {"good": "良品", "defect": "不良", "out_of_range": "範囲外"}
TYPE_JA = {"scratch": "傷", "dent": "打痕", "stain": "汚れ"}


def draw_examples(img, T, box, insp):
    x0, y0, w, h = box
    T.put(img, "疑似カメラの画像と判定（帯の色 = 判定、四角 = 欠陥の候補）", x0 + 12, y0 + 26, 18, INK)
    ex = pick_examples(insp["rows"])
    tw, th = 184, 138
    for k, r in enumerate(ex):
        col, row = k % 4, k // 4
        x, y = x0 + 12 + col * (tw + 10), y0 + 44 + row * (th + 52)
        tile = cv2.resize(I.annotate(r["img"], r["result"]), (tw, th), interpolation=cv2.INTER_AREA)
        img[y:y + th, x:x + tw] = cv2.cvtColor(tile, cv2.COLOR_RGB2BGR)
        cv2.rectangle(img, (x - 1, y - 1), (x + tw, y + th), AXIS, 1)
        truth = TRUTH_JA[r["truth"]]
        if r["true_types"]:
            truth += "（" + "・".join(TYPE_JA[t] for t in r["true_types"]) + f" 強さ {r['severity']:.2f}）"
        elif r["ood_kind"]:
            truth += "（" + {"dim": "暗い", "bright": "明るすぎ", "stray": "外の光"}[r["ood_kind"]] + "）"
        T.put(img, f"{r['part']} / {r['face']}", x, y + th + 18, 13, INK2)
        T.put(img, f"正解: {truth}", x, y + th + 36, 13, INK)


def render_png(motion, insp, path: Path):
    T = Text()
    W, H = 1640, 1020
    img = np.full((H, W, 3), SURFACE, np.uint8)
    T.put(img, "双腕の外観検査セル（自作の簡単な形）: 左腕が持って見せ、右腕の手首カメラで撮り、3 段（OK／NG／保留）で判定",
          24, 40, 22, INK)
    m = motion
    sub = (f"逆運動学: 目標 {m['kf_n']} 個すべて解けて上下限の中（手先の誤差 最大 {max(m['max_dp'], m['max_dr']):.0e}）"
           f"｜ 腕どうしの最短 {m['min_gap'][0]:.3f} m ｜ 机・胴体との最短 {m['min_env'][0]:.3f} m ｜ "
           f"1 周 {m['timeline'].total:.0f} 秒・撮影 {len(m['captures'])} 回")
    T.put(img, sub, 24, 70, 15, INK2)
    panels = {"cell": (16, 88, 800, 560), "chart": (16, 660, 800, 344), "table": (832, 88, 792, 404),
              "ex": (832, 504, 792, 500)}
    for x, y, w, h in panels.values():
        cv2.rectangle(img, (x, y), (x + w, y + h), GRID, 1)
    draw_cell(img, T, panels["cell"], motion)
    draw_gap_chart(img, T, panels["chart"], motion)
    draw_table(img, T, panels["table"], insp)
    draw_examples(img, T, panels["ex"], insp)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", img)  # 日本語を含むパスでも書けるように
    if not ok:
        raise RuntimeError("PNG にできなかった")
    path.write_bytes(buf.tobytes())


def main() -> int:
    m = check_motion()
    print("== 1. 動きの計画（逆運動学で解いた目標）")
    print(f"  目標の数: {m['kf_n']}、解けて上下限の中で手先が合った: {m['kf_ok']} / {m['kf_n']}"
          f"（{m['kf_ok'] / m['kf_n']:.1%}）")
    print(f"  手先の誤差の最大: 位置 {m['max_dp']:.1e} m、向き {m['max_dr']:.1e} rad")
    print(f"  探した候補: {m['plan'].ik_tries} 個のうち、上下限の中の解があった {m['plan'].ik_found} 個"
          f"（{m['plan'].ik_found / m['plan'].ik_tries:.1%}）。計画にかかった時間 {m['plan_time']:.1f} 秒")
    print(f"  1 周: {m['timeline'].total:.1f} 秒、区間 {len(m['plan'].segments)}、撮影 {len(m['captures'])} 回")
    print(f"== 2. 動きの途中（{len(m['ts'])} 点 = {RATE:.0f}Hz）")
    print(f"  上下限の外に出た点: {m['limit_ng']}")
    g, t, lab = m["min_gap"]
    print(f"  腕どうしのいちばん近い距離: {g:.3f} m（下限 {C.ARM_MARGIN} m）… {t:.1f} 秒「{lab}」")
    g, t, lab = m["min_env"]
    print(f"  腕と机・トレイ・胴体のいちばん近い距離: {g:.3f} m（下限 {C.ENV_MARGIN} m）… {t:.1f} 秒「{lab}」")
    print(f"  関節の速さの最大: {m['max_vel']:.2f} rad/s（URDF の上限 {VELOCITY} rad/s）")
    print(f"  手首の曲げ（j5）の絶対値の最小: {m['min_q5']:.2f} rad（0 に近いと手首の特異点）")
    print("== 3. 逆運動学の解き直し（でたらめな関節角 2000 通り）")
    for k, (ok, worst) in m["roundtrip"].items():
        print(f"  {'左' if k == 'l' else '右'}腕: {ok} / 2000 で元の手先の姿勢に戻った（誤差の最大 {worst:.1e}）")

    insp = check_inspection()
    cal, tab, s = insp["cal"], insp["table"], insp["summary"]
    print("== 4. 検査（良品だけで学ぶ → 3 段判定）")
    print(f"  学び: 型 {cal['templates']} 個、しきい値を決めた良品 {cal['good_images']} 枚（{insp['cal_time']:.1f} 秒）"
          f" → OK < {cal['t_ok']:.1f} ≦ 保留 < {cal['t_ng']:.1f} ≦ NG")
    print(f"  {'正解＼判定':<12}{'OK':>6}{'NG':>6}{'保留':>6}{'計':>6}")
    for key, name in (("good", "良品"), ("defect", "不良"), ("out_of_range", "範囲外")):
        r = tab[key]
        print(f"  {name:<12}{r['OK']:>6}{r['NG']:>6}{r['HOLD']:>6}{sum(r.values()):>6}")
    print(f"  見逃し {s['miss_rate']:.1%}、過検出 {s['overkill_rate']:.1%}、人に回る {s['hold_rate']:.1%}、"
          f"範囲外を保留にできた {s['ood_caught']:.0%}、自動で決めた分の正解率 {s['auto_accuracy']:.1%}")
    misses = [r for r in insp["rows"] if r["truth"] == "defect" and r["verdict"] == "OK"]
    if misses:
        print("  見逃した欠陥: " + ", ".join(f"{r['part']}/{r['face']} {','.join(r['true_types'])} 強さ {r['severity']:.2f}"
                                       for r in misses))
    typed = [r for r in insp["rows"] if r["truth"] == "defect" and r["verdict"] == "NG"]
    hit = sum(1 for r in typed if r["pred_type"] in r["true_types"])
    print(f"  NG にした不良のうち、欠陥の種類（傷・打痕・汚れ）も当たった: {hit} / {len(typed)}")

    render_png(m, insp, OUT)
    print(f"== 絵: {OUT}")
    ok = (m["kf_ok"] == m["kf_n"] and m["limit_ng"] == 0 and m["min_gap"][0] >= C.ARM_MARGIN
          and m["min_env"][0] >= C.ENV_MARGIN and all(v[0] == 2000 for v in m["roundtrip"].values()))
    print("== 動きの確認: すべて OK" if ok else "== 動きの確認: NG があります")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
