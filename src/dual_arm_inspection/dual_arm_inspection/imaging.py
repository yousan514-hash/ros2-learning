"""疑似カメラ: 部品の面の画像を numpy / OpenCV で作る（ROS を使わない）。

- 面の種類ごとに、形（六角・円・輪・横から見た形）と、その面の明るさの模様（面取り・円柱の光り方）を描く
- 照明はカメラのまわりのリング照明を想定（撮像をそろえる）。それでも残る明るさのばらつきと、
  光の向きによる明るさの傾きを少し入れる
- 傷（scratch）・打痕（dent）・汚れ（stain）を、強さ（severity 0〜1）を変えてランダムに入れる。
  弱いものは目でもほとんど見えない（だから「保留」や「見逃し」が出る）
- 範囲外（照明の劣化・外からの光）の画像も少しの確率で作る。検査の「範囲外の見張り」を試すため

座標の決め方: u が部品の軸の向き（横から見た形）、v がそれに垂直。画像の中心のまわりに、
位置を数ピクセル・向きを数度ずらして置く（持ち方・置き方のばらつき）。
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import cv2
import numpy as np

W, H = 320, 240
BASE_LEVEL = 200.0     # 反射率 1.0・明るさの倍率 1.0 のときの明るさ（0〜255）
BG_LEVEL = 36.0        # 背景の明るさ
DEFECT_TYPES = ("scratch", "dent", "stain")
FACE_TYPES = ("bolt_profile", "hex_end", "circle_end", "hex_ring", "nut_profile", "ring")
PROFILE_TYPES = ("bolt_profile", "nut_profile")
TINT = np.array([0.97, 0.99, 1.03])  # 亜鉛めっきの少し青い銀色
STAIN_TINT = np.array([1.0, 0.95, 0.84])
_YS, _XS = np.mgrid[0:H, 0:W].astype(np.float32)


@dataclass
class Lighting:
    gain: float = 1.0      # 明るさの倍率（照明の強さ・劣化）
    grad: float = 0.0      # 明るさの傾き（部品の端から端で ±grad）
    angle: float = 0.0     # 傾きの向き（rad）
    ambient: float = 0.0   # 背景に足される明るさ（外からの光）
    ood: bool = False
    ood_kind: str | None = None


def sample_lighting(rng: np.random.Generator, p_ood: float = 0.0) -> Lighting:
    """ふだんの照明（使用範囲の中）か、ときどき範囲外の照明。"""
    if rng.random() < p_ood:
        kind = str(rng.choice(["dim", "bright", "stray"]))
        if kind == "dim":
            return Lighting(float(rng.uniform(0.45, 0.65)), float(rng.uniform(0, 0.08)),
                            float(rng.uniform(0, 2 * math.pi)), 0.0, True, kind)
        if kind == "bright":
            return Lighting(float(rng.uniform(1.40, 1.70)), float(rng.uniform(0, 0.08)),
                            float(rng.uniform(0, 2 * math.pi)), 0.0, True, kind)
        return Lighting(float(rng.uniform(0.95, 1.05)), float(rng.uniform(0.25, 0.40)),
                        float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(45, 80)), True, kind)
    return Lighting(float(rng.uniform(0.90, 1.10)), float(rng.uniform(0.0, 0.10)),
                    float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(-3, 3)))


# ---- 形 ----
def _dims_mm(part, face_type: str) -> dict:
    d = part.dims
    if part.kind == "bolt":
        af = d["head_across_flats"]
        return {"d": d["d"], "L": d["length"], "Hh": d["head_height"], "af": af,
                "c": 2 * af / math.sqrt(3), "T": d["length"] + d["head_height"]}
    if part.kind == "nut":
        af = d["across_flats"]
        return {"af": af, "c": 2 * af / math.sqrt(3), "h": d["height"], "hole": d["d"]}
    return {"D": d["d_outer"], "din": d["d_inner"], "t": d["thickness"]}


def scale_ppm(part, face_type: str) -> float:
    """1mm が何ピクセルか。レシピで部品ごとに倍率を合わせ、部品が画面の 6〜8 割に入るようにする想定。"""
    m = _dims_mm(part, face_type)
    if face_type == "bolt_profile":
        return min(0.80 * W / m["T"], 0.80 * H / m["c"])
    if face_type in ("hex_end", "hex_ring"):
        return 0.72 * H / m["c"]
    if face_type == "circle_end":
        return 0.60 * H / m["d"]
    if face_type == "nut_profile":
        return min(0.80 * W / m["h"], 0.80 * H / m["af"])
    if face_type == "ring":
        return 0.72 * H / m["D"]
    raise ValueError(f"知らない面の種類: {face_type}")


def _cov(sd):
    """符号つき距離（内側が正、ピクセル）→ 覆う割合（0〜1）。縁がなめらかになる。"""
    return np.clip(sd + 0.5, 0.0, 1.0)


def _sd_hex(u, v, af):
    """平らな面が ±u の向きにある正六角形（二面幅 af）。"""
    c60, s60 = 0.5, math.sqrt(3) / 2
    return af / 2 - np.maximum(np.abs(u), np.maximum(np.abs(u * c60 + v * s60), np.abs(-u * c60 + v * s60)))


def _sd_rect(u, v, u0, u1, v0, v1):
    return np.minimum(np.minimum(u - u0, u1 - u), np.minimum(v - v0, v1 - v))


def pattern(part, face_type: str, cx: float, cy: float, ang: float) -> tuple[np.ndarray, np.ndarray, float]:
    """(反射率の地図, 部品が覆う割合, 部品の半径の目安 px)。部品の外は反射率 0。"""
    ppm = scale_ppm(part, face_type)
    m = {k: v * ppm for k, v in _dims_mm(part, face_type).items()}
    dx, dy = _XS - cx, _YS - cy
    ca, sa = math.cos(ang), math.sin(ang)
    u, v = dx * ca + dy * sa, -dx * sa + dy * ca
    rho = np.hypot(u, v)

    if face_type == "bolt_profile":
        T, Hh, c, d = m["T"], m["Hh"], m["c"], m["d"]
        u_head = -T / 2 + Hh
        head = _cov(_sd_rect(u, v, -T / 2, u_head, -c / 2, c / 2))
        shank = _cov(_sd_rect(u, v, u_head - 1.0, T / 2, -d / 2, d / 2))
        w_mid = np.clip(c / 4 - np.abs(v) + 0.5, 0, 1)
        alb_head = 0.80 * w_mid + np.where(v < 0, 0.60, 0.56) * (1 - w_mid)
        t = np.clip(v / (d / 2), -1, 1)
        alb_shank = 0.42 + 0.36 * np.sqrt(1 - t * t) + 0.14 * np.exp(-(t / 0.35) ** 2)
        mask = np.maximum(head, shank)
        alb = head * alb_head + (1 - head) * shank * alb_shank
        alb = np.where(mask > 0, alb / np.maximum(mask, 1e-6), 0.0)
        radius = max(T, c) / 2
    elif face_type == "hex_end":
        af = m["af"]
        mask = _cov(_sd_hex(u, v, af))
        alb = 0.78 - 0.16 * np.clip(rho - 0.47 * af + 0.5, 0, 1)
        radius = m["c"] / 2
    elif face_type == "circle_end":
        r = m["d"] / 2
        mask = _cov(r - rho)
        alb = 0.74 - 0.16 * np.clip(rho - 0.84 * r + 0.5, 0, 1)
        radius = r
    elif face_type == "hex_ring":
        af, rh = m["af"], m["hole"] / 2
        mask = _cov(np.minimum(_sd_hex(u, v, af), rho - rh))
        alb = 0.78 - 0.18 * np.clip(rh * 1.25 - rho + 0.5, 0, 1) - 0.14 * np.clip(rho - 0.47 * af + 0.5, 0, 1)
        radius = m["c"] / 2
    elif face_type == "nut_profile":
        h, af = m["h"], m["af"]
        mask = _cov(_sd_rect(u, v, -h / 2, h / 2, -af / 2, af / 2))
        alb = np.where(v < 0, 0.74, 0.64) + 0.10 * np.exp(-(v / 1.2) ** 2)
        alb = alb - 0.14 * np.clip(np.abs(u) - (h / 2 - 0.08 * h) + 0.5, 0, 1)
        radius = max(h, af) / 2
    elif face_type == "ring":
        ro, ri = m["D"] / 2, m["din"] / 2
        mask = _cov(np.minimum(ro - rho, rho - ri))
        alb = 0.76 - 0.05 * np.clip((rho - ri) / max(ro - ri, 1e-6), 0, 1)
        radius = ro
    else:
        raise ValueError(f"知らない面の種類: {face_type}")
    alb = np.where(mask > 0, alb, 0.0).astype(np.float32)
    return alb, mask.astype(np.float32), float(radius)


# ---- 欠陥 ----
def _pick_point(core: np.ndarray, rng) -> tuple[float, float]:
    ys, xs = np.nonzero(core)
    i = int(rng.integers(len(xs)))
    return float(xs[i]) + rng.uniform(-0.5, 0.5), float(ys[i]) + rng.uniform(-0.5, 0.5)


def _bbox(footprint: np.ndarray) -> list[int]:
    ys, xs = np.nonzero(footprint > 0.05)
    if len(xs) == 0:
        return [0, 0, 0, 0]
    return [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)]


def add_defect(alb: np.ndarray, stain: np.ndarray, core: np.ndarray, radius: float, kind: str, severity: float,
               light_angle: float, rng) -> dict:
    """反射率の地図に欠陥を 1 つ描く（その場で書き換える）。正解（種類・強さ・外接する四角）を返す。"""
    x0, y0 = _pick_point(core, rng)
    if kind == "scratch":
        length = (0.25 + 0.35 * rng.random()) * radius
        a = rng.uniform(0, math.pi)
        bend = rng.uniform(-0.35, 0.35)
        p0 = np.array([x0 - math.cos(a) * length / 2, y0 - math.sin(a) * length / 2])
        p1 = np.array([x0, y0])
        p2 = np.array([x0 + math.cos(a + bend) * length / 2, y0 + math.sin(a + bend) * length / 2])
        canvas = np.zeros(alb.shape, np.uint8)
        pts = (np.stack([p0, p1, p2]) * 16).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], False, 255, thickness=int(rng.integers(1, 3)), lineType=cv2.LINE_AA, shift=4)
        fp = canvas.astype(np.float32) / 255.0 * core
        sign = 1.0 if rng.random() < 0.4 else -1.0   # 光って明るく見える傷と、暗く見える傷
        alb += sign * (0.03 + 0.25 * severity) * fp
    elif kind == "dent":
        r = 2.5 + 3.0 * rng.random()
        xs, ys = _XS, _YS
        d2 = (xs - x0) ** 2 + (ys - y0) ** 2
        depth = 0.05 + 0.35 * severity
        dark = np.exp(-d2 / (2 * (r / 1.5) ** 2))
        lx, ly = x0 + math.cos(light_angle) * r * 0.7, y0 + math.sin(light_angle) * r * 0.7
        rim = np.exp(-((xs - lx) ** 2 + (ys - ly) ** 2) / (2 * (r / 2.2) ** 2))
        fp = np.maximum(dark, rim) * core
        alb += (-depth * dark + 0.45 * depth * rim) * core
    elif kind == "stain":
        canvas = np.zeros(alb.shape, np.uint8)
        for _ in range(int(rng.integers(3, 6))):
            c = (int((x0 + rng.uniform(-8, 8)) * 16), int((y0 + rng.uniform(-8, 8)) * 16))
            cv2.circle(canvas, c, int(rng.uniform(4, 12) * 16), 255, -1, cv2.LINE_AA, shift=4)
        blob = cv2.GaussianBlur(canvas.astype(np.float32) / 255.0, (0, 0), 2.5) * core
        fp = blob
        alb *= 1.0 - (0.05 + 0.30 * severity) * blob
        stain += 0.7 * blob  # 茶色っぽさ（色の変わり方は render で付ける）
    else:
        raise ValueError(kind)
    return {"type": kind, "severity": round(float(severity), 3), "bbox": _bbox(fp)}


# ---- 撮る ----
def render(part, face_type: str, rng: np.random.Generator, defects=(), lighting: Lighting | None = None,
           jitter: bool = True) -> tuple[np.ndarray, dict]:
    """1 枚撮る。defects は [{"type": ..., "severity": ...}]。戻り値は (RGB の uint8 画像, 正解)。"""
    light = lighting or Lighting()
    cx, cy, ang = W / 2, H / 2, 0.0
    if jitter:
        cx += rng.uniform(-6, 6)
        cy += rng.uniform(-5, 5)
        ang = math.radians(rng.uniform(-4, 4))
    alb, mask, radius = pattern(part, face_type, cx, cy, ang)
    stain = np.zeros((H, W), np.float32)
    core = cv2.erode((mask > 0.99).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
    truth_defects = [add_defect(alb, stain, core, radius, d["type"], d["severity"], light.angle, rng) for d in defects]

    ca, sa = math.cos(light.angle), math.sin(light.angle)
    shade = 1.0 + (light.grad / radius) * ((_XS - cx) * ca + (_YS - cy) * sa)
    part_val = (BASE_LEVEL * light.gain) * alb * shade
    xn, yn = (_XS - W / 2) / W, (_YS - H / 2) / H
    bg = (BG_LEVEL + light.ambient) + 4.0 * xn - 6.0 * (xn * xn + yn * yn)
    texture = cv2.GaussianBlur(rng.standard_normal((H, W), dtype=np.float32) * 4.0, (0, 0), 1.0)
    gray = bg + mask * (part_val - bg + texture) + rng.standard_normal((H, W), dtype=np.float32) * 2.2
    color = TINT.astype(np.float32)[None, None, :] * (1.0 - (1.0 - STAIN_TINT.astype(np.float32))[None, None, :]
                                                      * stain[:, :, None])
    rgb = gray[:, :, None] * (1.0 + mask[:, :, None] * (color - 1.0))
    rgb += rng.standard_normal((H, W, 3), dtype=np.float32)
    img = np.clip(rgb, 0, 255).astype(np.uint8)
    truth = {"defects": truth_defects, "ood": light.ood, "ood_kind": light.ood_kind,
             "lighting": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in asdict(light).items()},
             "pose": {"cx": round(cx, 2), "cy": round(cy, 2), "angle_deg": round(math.degrees(ang), 2)}}
    return img, truth


def random_shot(rng: np.random.Generator, part, face_type: str, p_defect: float = 0.3,
                p_ood: float = 0.05) -> tuple[np.ndarray, dict]:
    """ラインで撮る 1 枚: p_defect の確率で欠陥（1〜2 個）、p_ood の確率で範囲外の照明。"""
    light = sample_lighting(rng, p_ood)
    defects = []
    if rng.random() < p_defect:
        for _ in range(1 if rng.random() < 0.85 else 2):
            # 強さは弱いほうに寄せる（目でやっと見える程度のものが多い）
            defects.append({"type": str(rng.choice(DEFECT_TYPES)), "severity": float(rng.random() ** 1.6)})
    return render(part, face_type, rng, defects, light)


def truth_class(truth: dict) -> str:
    """評価のための正解の分類: out_of_range（範囲外）/ defect（不良）/ good（良品）。"""
    if truth.get("ood"):
        return "out_of_range"
    return "defect" if truth.get("defects") else "good"
