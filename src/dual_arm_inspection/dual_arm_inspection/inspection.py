"""検査: 良品だけで学んだ「ふつうの見え方」と比べて、3 段（OK／NG／保留）で判定する（ROS を使わない）。

藤田さんのナレッジ「精度100%を目指さない外観検査AI」の考えに合わせた:
  - 良品だけで学ぶ: 部品と面の種類ごとに、良品の画像から「平均の見え方」と「ばらつき」を作る
  - 3 段判定: 確実に良品 → OK（自動で通す）、確実に不良 → NG（自動で弾く）、あいまい → 保留（人に回す）
  - 範囲外の見張り: 明るさ・背景・明るさの傾き・部品の形（学んだ形からのはみ出し）が、学んだときの範囲
    （使用範囲）から外れたら、欠陥の点数に関係なく保留にして人に回す（照明の劣化や外からの光に気づく）。
    部品の一部が暗く「欠けて」見えるのは濃い汚れ・打痕の可能性が高いので、範囲外にせず点数に回す

やり方:
  1. 部品を背景から切り出す（大津の二値化 → いちばん大きい輪郭。小さい穴は欠陥なので埋め、大きい穴は残す）
  2. 重心と軸の向きで、画像を決まった位置・向きに合わせる。部品が暗く欠けて（濃い汚れ・打痕）重心がずれたときは、
     残った明るい所が学んだ形にいちばん収まる位置・向きへ合わせ直す
  3. 平均の見え方で割り、明るさの平面（倍率と傾き）を外れ値に強いやり方で当てはめて取り除く（欠けた所は使わない）
  4. 残ったずれを、良品のばらつきで割った値（z）が大きい所をまとめ、いちばん大きいまとまりの強さを点数にする
  5. 点数の 2 つのしきい値（良品だけで決める）で 3 段に分ける
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from . import imaging

Z_PIX = 3.0            # 1 画素が「ふつうでない」とみなす z（良品のばらつきの何倍か）
MIN_AREA = 3
SIGMA_FLOOR = 0.012
ALIGN_TOL_PX = 0.35   # 位置合わせのずれの見込み（画素）
# 見張る量と、学んだ範囲に足す余裕。extra は学んだ形からの「はみ出し」の割合（指・隣の部品・別の部品に気づく。
# 手で持つ面は良品でも指が写るので、良品で学んだ範囲で見る）
ENV_MARGIN = {"gain": 0.04, "background": 5.0, "grad": 0.03, "extra": 0.03}
MISSING_MAX = 0.60         # 学んだ形のこの割合より多く欠けていたら、欠陥ではなく部品が違う・見切れているとみなす
REFINE_MIN = 0.02          # 欠けがこの割合を超えたら、重心がずれているので学んだ形に合わせ直す
SHAPE_BAND_PX = 2          # 形を比べるとき、縁のこの幅は位置合わせのずれとして数えない
FIT_ERODE_PX = 3           # 明るさの平面を当てはめるとき、欠けた所の縁からこの幅も使わない（ぼけた縁に引っ張られない）
COLORS = {"OK": (12, 163, 12), "NG": (208, 59, 59), "HOLD": (250, 178, 25)}  # RGB（状態の色）
LABEL_JA = {"OK": "OK", "NG": "NG", "HOLD": "保留"}
_GRIDS: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}


def _grid(shape) -> tuple[np.ndarray, np.ndarray]:
    """画像の中心を 0、100 画素を 1 とした座標（画像の大きさごとに 1 回だけ作る）。"""
    key = (int(shape[0]), int(shape[1]))
    if key not in _GRIDS:
        ys, xs = np.mgrid[0:key[0], 0:key[1]].astype(np.float32)
        _GRIDS[key] = ((xs - key[1] / 2) / 100.0, (ys - key[0] / 2) / 100.0)
    return _GRIDS[key]


@dataclass
class Template:
    mean: np.ndarray
    sigma: np.ndarray
    roi: np.ndarray
    area: float
    ref_level: float
    radius: float
    env_lo: dict = field(default_factory=dict)
    env_hi: dict = field(default_factory=dict)
    t_ok: float = 3.0
    t_ng: float = 9.0


@dataclass
class Result:
    verdict: str                       # "OK" / "NG" / "HOLD"
    score: float
    reason: str                        # clean / uncertain / defect / out_of_range:<何が> / no_part / shape
    defect_type: str | None = None
    boxes: list = field(default_factory=list)    # 元の画像での [x, y, w, h]
    features: dict = field(default_factory=dict)


# ---- 部品を切り出す・合わせる ----
def segment(gray: np.ndarray) -> np.ndarray | None:
    blur = cv2.GaussianBlur(gray.astype(np.uint8), (5, 5), 0)
    thr, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    fg, bg = blur[bw > 0], blur[bw == 0]
    if len(fg) == 0 or len(bg) == 0 or float(fg.mean()) - float(bg.mean()) < 25.0:
        return None  # 明るさの差がない = 部品が写っていない
    contours, hier = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    outer = [i for i in range(len(contours)) if hier[0][i][3] < 0]
    k = max(outer, key=lambda i: cv2.contourArea(contours[i]))
    area = cv2.contourArea(contours[k])
    if area < 400:
        return None
    mask = np.zeros(gray.shape, np.uint8)
    cv2.drawContours(mask, contours, k, 1, -1)
    for i in range(len(contours)):  # 大きい穴（ナット・ワッシャーの穴）だけ残す
        if hier[0][i][3] == k and cv2.contourArea(contours[i]) > 0.06 * area:
            cv2.drawContours(mask, contours, i, 0, -1)
    return mask


def align(gray: np.ndarray, mask: np.ndarray, profile: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """重心を画像の中心へ、（横から見た形なら）軸を水平に。戻り値は (合わせた画像, 合わせたマスク, 2x3 の変換)。"""
    mom = cv2.moments(mask, binaryImage=True)
    cx, cy = mom["m10"] / mom["m00"], mom["m01"] / mom["m00"]
    ang = 0.0
    if profile:
        ang = 0.5 * math.degrees(math.atan2(2 * mom["mu11"], mom["mu20"] - mom["mu02"]))
        ang = (ang + 45.0) % 90.0 - 45.0  # 縦長・横長のどちらでも、水平・垂直に近い向きへ
    M = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
    h, w = gray.shape[:2]  # 疑似カメラ（320x240）でも Isaac Sim（480x360 など）でも同じように使う
    M[:, 2] += (w / 2 - cx, h / 2 - cy)
    a = cv2.warpAffine(gray, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    m = cv2.warpAffine(mask.astype(np.float32), M, (w, h), flags=cv2.INTER_LINEAR)
    return a, m, M


def warp(gray: np.ndarray, mask: np.ndarray, M: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """2x3 の変換で画像とマスクを合わせる（align と同じ補間）。"""
    h, w = gray.shape[:2]
    a = cv2.warpAffine(gray, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    m = cv2.warpAffine(mask.astype(np.float32), M, (w, h), flags=cv2.INTER_LINEAR)
    return a, m


def shape_diff(m: np.ndarray, roi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """合わせたマスクと学んだ形の差: (欠けた所, はみ出た所)。縁の SHAPE_BAND_PX 画素は位置合わせのずれとして数えない。

    欠けた所 = 学んだ形の中なのに暗い（大津の二値化で部品から外れた）所。濃い汚れ・打痕・大きな穴
    はみ出た所 = 学んだ形の外なのに明るい所。指・隣の部品・別の部品・照明の大きな変化
    """
    k = np.ones((2 * SHAPE_BAND_PX + 1,) * 2, np.uint8)
    roi8 = roi.astype(np.uint8)
    inner = cv2.erode(roi8, k) > 0
    outer = cv2.dilate(roi8, k) > 0
    bright = m > 0.5
    return inner & ~bright, bright & ~outer


def refine_align(mask: np.ndarray, M: np.ndarray, roi: np.ndarray, radius: float) -> tuple[np.ndarray, tuple[int, int]]:
    """部品の一部が暗く欠けて重心がずれたとき、残った明るい所が学んだ形にいちばん収まる位置へずらし直す。

    学んだ形の中を +1、外を -1 の重みにして、マスクを半径の半分までずらしながら重みの合計（相互相関）が
    いちばん大きい所を探す。欠けた所は 0 なので効かず、はみ出すと減点される。向きは変えない（手で持つ面は
    指も写るので、向きまで合わせ直すとかえってずれた）。戻り値は (直した 2x3 の変換, ずらした量 px)。
    """
    h, w = roi.shape
    R = int(max(8, round(0.5 * radius)))
    weight = np.where(roi, 1.0, -1.0).astype(np.float32)
    padded = cv2.copyMakeBorder(weight, R, R, R, R, cv2.BORDER_CONSTANT, value=-1.0)
    m = (cv2.warpAffine(mask.astype(np.float32), M, (w, h), flags=cv2.INTER_NEAREST) > 0.5).astype(np.float32)
    res = cv2.matchTemplate(padded, m, cv2.TM_CCORR)
    _, _, _, loc = cv2.minMaxLoc(res)
    dx, dy = loc[0] - R, loc[1] - R
    Mt = M.copy()
    Mt[:, 2] += (dx, dy)
    return Mt, (int(dx), int(dy))


def fit_region(m: np.ndarray, roi: np.ndarray) -> np.ndarray:
    """明るさの平面を当てはめる所: 学んだ形の中から、暗く欠けた所とそのぼけた縁（FIT_ERODE_PX）を除いた所。

    欠けがなければ学んだ形の全部（＝前と同じ）。良品の判定は変わらない。
    """
    dark = (roi & ~(m > 0.5)).astype(np.uint8)
    if not dark.any():
        return roi
    near_dark = cv2.dilate(dark, np.ones((2 * FIT_ERODE_PX + 1,) * 2, np.uint8)) > 0
    return roi & ~near_dark


def _plane_fit(R: np.ndarray, roi: np.ndarray, iters: int = 3) -> np.ndarray:
    """R ≈ a + b*x + c*y を、外れ値（欠陥）を除きながら当てはめる。"""
    ys, xs = np.nonzero(roi[::2, ::2])
    ys, xs = ys * 2, xs * 2
    vals = R[ys, xs]
    h, w = roi.shape
    X = np.column_stack([np.ones_like(xs, dtype=np.float64), (xs - w / 2) / 100.0, (ys - h / 2) / 100.0])
    keep = np.ones(len(vals), bool)
    coef = np.array([np.median(vals), 0.0, 0.0])
    for _ in range(iters):
        if keep.sum() < 20:
            break
        coef, *_ = np.linalg.lstsq(X[keep], vals[keep], rcond=None)
        rel = vals / (X @ coef) - 1.0
        mad = np.median(np.abs(rel[keep])) + 1e-6
        keep = np.abs(rel) < max(4.0 * 1.4826 * mad, 0.02)
    return coef


def measure(A: np.ndarray, mean: np.ndarray, roi: np.ndarray,
            fit_roi: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(ずれの地図 D, 明るさの平面の係数)。D = 画像 / (平均の見え方 × 平面) - 1。

    fit_roi を渡すと、明るさの平面はその中だけで当てはめる（欠けた所＝濃い欠陥に引っ張られて
    「暗くなった」と誤解しないため）。D は roi の全部で出す。
    """
    R = np.where(roi, A / np.maximum(mean, 1e-3), 0.0)
    fit_on = roi if fit_roi is None or fit_roi[::2, ::2].sum() < 20 else fit_roi
    coef = _plane_fit(R, fit_on)
    xn, yn = _grid(roi.shape)
    plane = coef[0] + coef[1] * xn + coef[2] * yn
    D = np.where(roi, R / np.maximum(plane, 1e-3) - 1.0, 0.0).astype(np.float32)
    return D, coef


def _zmap(D: np.ndarray, tpl: Template) -> np.ndarray:
    z = np.abs(cv2.GaussianBlur(D, (3, 3), 0)) / tpl.sigma
    z[~tpl.roi] = 0.0
    return z


def components(D: np.ndarray, tpl: Template) -> list[dict]:
    """z が Z_PIX を超える画素のまとまり（大きい順）。"""
    z = _zmap(D, tpl)
    n, labels, stats, _ = cv2.connectedComponentsWithStats((z > Z_PIX).astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < MIN_AREA:
            continue
        sel = labels == i
        energy = float(np.sum(np.minimum(z[sel], 25.0) - Z_PIX))
        pts = np.column_stack(np.nonzero(sel))[:, ::-1].astype(np.float32)
        (_, _), (w, h), _ = cv2.minAreaRect(pts)
        aspect = (max(w, h) + 1.0) / (min(w, h) + 1.0)
        out.append({"energy": energy, "area": area, "aspect": float(aspect),
                    "mean_d": float(np.mean(D[sel])), "peak_d": float(np.max(np.abs(D[sel]))),
                    "bbox": [int(v) for v in stats[i, :4]]})
    return sorted(out, key=lambda c: -c["energy"])


def defect_type(c: dict) -> str:
    """いちばん大きいまとまりの形から、欠陥の種類を当てる（細長い → 傷、広くて暗い → 汚れ、小さい → 打痕）。"""
    if c["aspect"] >= 3.0:
        return "scratch"
    if c["area"] >= 110 or (c["mean_d"] < 0 and c["area"] >= 60 and c["peak_d"] < 0.25):
        return "stain"
    return "dent"


def background_level(gray: np.ndarray, mask: np.ndarray) -> float:
    far = cv2.dilate(mask, np.ones((15, 15), np.uint8)) == 0
    return float(np.median(gray[far])) if far.any() else 0.0


def base_type(face_type: str) -> str:
    """"bolt_profile@side_top" のように面の名前を足した鍵から、面の種類だけを取り出す。

    Isaac Sim では、同じ種類でもトレイの上から撮る面と手で持って撮る面で背景や照明が違うので、
    面の名前ごとに別の「ふつうの見え方」を学ぶ。
    """
    return face_type.split("@")[0]


class Inspector:
    def __init__(self, parts):
        self.parts = {p.name: p for p in parts}
        self.templates: dict[tuple[str, str], Template] = {}
        self.t_ok = 0.0
        self.t_ng = 0.0
        self.calibration: dict = {}

    def face_types(self) -> list[tuple[str, str]]:
        out = []
        for p in self.parts.values():
            for f in p.faces:
                if (p.name, f.face_type) not in out:
                    out.append((p.name, f.face_type))
        return out

    # ---- 良品だけで学ぶ ----
    def _features(self, gray, mask, tpl: Template, coef, m=None) -> dict:
        center = float(coef[0])
        feats = {"gain": center / tpl.ref_level,
                 "background": background_level(gray, mask),
                 "grad": float(math.hypot(coef[1], coef[2]) / max(center, 1e-6) * tpl.radius / 100.0),
                 "area_ratio": float(mask.sum() / tpl.area)}
        if m is not None:
            missing, extra = shape_diff(m, tpl.roi)
            feats["missing"] = float(missing.sum() / tpl.area)   # 学んだ形のうち暗く欠けた割合（欠陥の候補）
            feats["extra"] = float(extra.sum() / tpl.area)       # 学んだ形からはみ出た割合（見張りの対象）
        return feats

    def calibrate(self, n_fit: int = 20, n_val: int = 20, seed: int = 20261106) -> dict:
        """疑似カメラで良品（欠陥なし・ふつうの照明）を撮り、それだけで学ぶ。"""
        rng = np.random.default_rng(seed)
        samples = {}
        for name, ft in self.face_types():
            part = self.parts[name]
            samples[(name, ft)] = [imaging.render(part, ft, rng, (), imaging.sample_lighting(rng, 0.0))[0]
                                   for _ in range(n_fit + n_val)]
        cal = self.fit(samples, n_fit)
        cal["seed"] = seed
        return cal

    def fit(self, samples: dict[tuple[str, str], list[np.ndarray]], n_fit: int) -> dict:
        """良品の画像だけで学ぶ。samples は {(部品名, 面の種類): [RGB 画像, ...]}。

        それぞれの最初の n_fit 枚で「平均の見え方・ばらつき・使用範囲」を作り、残りの枚数で
        しきい値を決める（学びに使った画像でしきい値を決めると甘くなるので分ける）。
        疑似カメラの画像でも、Isaac Sim や実物のカメラの画像でも同じように使える。
        """
        good_scores = []
        for (name, ft), images in samples.items():
            if len(images) <= n_fit:
                raise ValueError(f"{name}/{ft}: 良品の画像が足りない（{len(images)} 枚、学びに {n_fit} 枚 + しきい値用）")
            profile = base_type(ft) in imaging.PROFILE_TYPES
            shots = []
            for img in images:
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
                mask = segment(gray)
                if mask is None:
                    raise ValueError(f"{name}/{ft}: 良品の画像で部品が見つからない")
                a, m, _ = align(gray, mask, profile)
                shots.append((gray, mask, a, m))
            fit, val = shots[:n_fit], shots[n_fit:]
            inner = [cv2.erode((m > 0.5).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0 for *_, m in fit]
            norm = [a / np.median(a[i]) for (_, _, a, _), i in zip(fit, inner)]
            mean = np.median(np.stack(norm), axis=0).astype(np.float32)
            roi = np.mean(np.stack([m for *_, m in fit]), axis=0) > 0.98
            roi = cv2.erode(roi.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            area = float(np.mean([mk.sum() for _, mk, _, _ in fit]))
            radius = math.sqrt(area / math.pi)  # 明るさの傾きを「部品の端から端で何割」に直すための大きさ
            tpl = Template(mean, np.ones_like(mean), roi, area, 1.0, radius)
            measured = [measure(a, mean, roi, fit_roi=fit_region(m, roi)) for _, _, a, m in shots]  # 判定と同じ当てはめ方
            devs = [cv2.GaussianBlur(D, (3, 3), 0) for D, _ in measured[:n_fit]]
            centers = [coef[0] for _, coef in measured[:n_fit]]
            tpl.ref_level = float(np.mean(centers))
            sigma = np.sqrt(np.mean(np.stack(devs) ** 2, axis=0))
            # 明るさが急に変わる所（面の境・頭と軸の境）は、位置合わせが 0.35 画素ずれるだけで大きく違って見える。
            # 少ない良品ではそのばらつきを学びきれないので、模様の傾きから見込んだ分を足す
            gy, gx = np.gradient(cv2.GaussianBlur(mean, (0, 0), 0.7))
            edge = ALIGN_TOL_PX * np.hypot(gx, gy) / np.maximum(mean, 1e-3)
            sigma = np.sqrt(cv2.GaussianBlur(sigma, (0, 0), 1.0) ** 2 + edge ** 2)
            tpl.sigma = np.maximum(sigma, SIGMA_FLOOR).astype(np.float32)
            feats = [self._features(gray, mk, tpl, coef, m) for (gray, mk, _, m), (_, coef) in zip(shots, measured)]
            for k, margin in ENV_MARGIN.items():
                vals = np.array([f[k] for f in feats])
                spread = float(vals.max() - vals.min())
                tpl.env_lo[k] = float(vals.min() - 0.25 * spread - margin)
                tpl.env_hi[k] = float(vals.max() + 0.25 * spread + margin)
            self.templates[(name, ft)] = tpl
            val = []
            for D, _ in measured[n_fit:]:  # しきい値は、学びに使っていない良品で決める
                comps = components(D, tpl)
                val.append(comps[0]["energy"] if comps else 0.0)
            # しきい値はレシピ（部品 x 面）ごと。1 つの面のばらつきが大きくても、ほかの面の感度を落とさない
            # OK と保留の境: 良品の 98% が OK になる点数。NG と保留の境: 良品で出た最大の点数の 2 倍
            tpl.t_ok = max(float(np.percentile(val, 98)), 3.0)
            tpl.t_ng = max(2.0 * float(np.max(val)), 3.0 * tpl.t_ok)
            good_scores += val
        g = np.array(good_scores)
        t_ok = [t.t_ok for t in self.templates.values()]
        t_ng = [t.t_ng for t in self.templates.values()]
        self.t_ok, self.t_ng = float(np.median(t_ok)), float(np.median(t_ng))  # 表示用（代表値）
        self.calibration = {"templates": len(self.templates), "good_images": int(len(g)),
                            "good_score_p50": float(np.median(g)), "good_score_p98": float(np.percentile(g, 98)),
                            "good_score_max": float(g.max()), "t_ok": self.t_ok, "t_ng": self.t_ng,
                            "t_ok_range": [float(min(t_ok)), float(max(t_ok))],
                            "t_ng_range": [float(min(t_ng)), float(max(t_ng))]}
        return self.calibration

    # ---- 判定 ----
    def inspect(self, img: np.ndarray, part_name: str, face_type: str) -> Result:
        tpl = self.templates[(part_name, face_type)]
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
        mask = segment(gray)
        if mask is None:
            return Result("HOLD", 0.0, "no_part")
        profile = base_type(face_type) in imaging.PROFILE_TYPES
        a, m, M = align(gray, mask, profile)
        missing, _ = shape_diff(m, tpl.roi)
        shift = (0, 0)
        if missing.sum() > REFINE_MIN * tpl.area:  # 暗く欠けて重心がずれたかもしれない → 学んだ形に合わせ直す
            M, shift = refine_align(mask, M, tpl.roi, tpl.radius)
            a, m = warp(gray, mask, M)
        D, coef = measure(a, tpl.mean, tpl.roi, fit_roi=fit_region(m, tpl.roi))
        feats = self._features(gray, mask, tpl, coef, m)
        feats["shift"] = shift
        # 形の見張り: 欠けが大きすぎる（部品が違う・見切れ）ときだけ範囲外。はみ出し（指・別の物）は
        # 良品で学んだ範囲（env の extra）で見る。部品の中や縁が暗く欠けているのは濃い汚れ・打痕の候補なので点数へ
        if feats["missing"] > MISSING_MAX:
            return Result("HOLD", 0.0, "shape", features=feats)
        for k in ENV_MARGIN:
            if not tpl.env_lo[k] <= feats[k] <= tpl.env_hi[k]:
                return Result("HOLD", 0.0, f"out_of_range:{k}", features=feats)
        comps = components(D, tpl)
        score = comps[0]["energy"] if comps else 0.0
        inv = cv2.invertAffineTransform(M)
        boxes = []
        for c in comps[:5]:
            x, y, w, h = c["bbox"]
            corners = np.array([[x, y, 1], [x + w, y, 1], [x, y + h, 1], [x + w, y + h, 1]], np.float64)
            pts = corners @ inv.T
            x0, y0 = pts.min(axis=0)
            x1, y1 = pts.max(axis=0)
            boxes.append([int(x0), int(y0), int(math.ceil(x1 - x0)), int(math.ceil(y1 - y0))])
        if score < tpl.t_ok:
            return Result("OK", score, "clean", None, boxes, feats)
        dtype = defect_type(comps[0])
        if score >= tpl.t_ng:
            return Result("NG", score, "defect", dtype, boxes, feats)
        return Result("HOLD", score, "uncertain", dtype, boxes, feats)


# ---- 絵と評価 ----
def annotate(img: np.ndarray, res: Result, title: str = "") -> np.ndarray:
    """判定の結果を画像に描く（上に色の帯、欠陥の候補に四角）。RViz に出すので ASCII の字だけ。"""
    out = img.copy()
    color = COLORS[res.verdict]
    for i, (x, y, w, h) in enumerate(res.boxes):
        cv2.rectangle(out, (x - 3, y - 3), (x + w + 3, y + h + 3), color if i == 0 else (200, 200, 200), 1, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (out.shape[1], 22), color, -1)
    label = {"OK": "OK", "NG": "NG", "HOLD": "HOLD -> human"}[res.verdict]
    detail = res.defect_type or (res.reason if res.verdict == "HOLD" else "")
    text = f"{label}  {detail}  s={res.score:.0f}".strip()
    ink = (11, 11, 11) if res.verdict == "HOLD" else (255, 255, 255)
    cv2.putText(out, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.48, ink, 1, cv2.LINE_AA)
    if title:
        cv2.putText(out, title, (6, out.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
    return out


def evaluate(inspector: Inspector, n: int = 200, seed: int = 7, p_defect: float = 0.3,
             p_ood: float = 0.05) -> list[dict]:
    """ラインと同じ確率で n 枚撮って判定し、正解と並べた表の行を返す。"""
    rng = np.random.default_rng(seed)
    parts = list(inspector.parts.values())
    rows = []
    for i in range(n):
        part = parts[int(rng.integers(len(parts)))]
        face = part.faces[int(rng.integers(len(part.faces)))]
        img, truth = imaging.random_shot(rng, part, face.face_type, p_defect, p_ood)
        res = inspector.inspect(img, part.name, face.face_type)
        rows.append({"i": i, "part": part.name, "face": face.name, "face_type": face.face_type,
                     "truth": imaging.truth_class(truth), "verdict": res.verdict, "score": res.score,
                     "reason": res.reason, "pred_type": res.defect_type,
                     "true_types": [d["type"] for d in truth["defects"]],
                     "severity": max([d["severity"] for d in truth["defects"]], default=0.0),
                     "ood_kind": truth.get("ood_kind"), "img": img, "result": res})
    return rows


def confusion(rows: list[dict]) -> dict[str, dict[str, int]]:
    table = {t: {v: 0 for v in ("OK", "NG", "HOLD")} for t in ("good", "defect", "out_of_range")}
    for r in rows:
        table[r["truth"]][r["verdict"]] += 1
    return table


def summary(table: dict[str, dict[str, int]]) -> dict[str, float]:
    good, bad, ood = (sum(table[k].values()) for k in ("good", "defect", "out_of_range"))
    total = good + bad + ood
    auto = table["good"]["OK"] + table["good"]["NG"] + table["defect"]["OK"] + table["defect"]["NG"]
    correct = table["good"]["OK"] + table["defect"]["NG"]
    return {
        "total": total,
        "miss_rate": table["defect"]["OK"] / bad if bad else 0.0,          # 見逃し: 不良を OK にした
        "overkill_rate": table["good"]["NG"] / good if good else 0.0,     # 過検出: 良品を NG にした
        "hold_rate": sum(table[k]["HOLD"] for k in table) / total if total else 0.0,   # 人に回る割合
        "ood_caught": table["out_of_range"]["HOLD"] / ood if ood else 1.0,
        "auto_accuracy": correct / auto if auto else 0.0,
    }
