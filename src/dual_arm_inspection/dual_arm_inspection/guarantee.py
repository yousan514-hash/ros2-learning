"""保証つきのしきい値: 「使用範囲の中なら、見逃しは α 以下（確からしさ 1−δ）」と言える OK の境を決める（ROS を使わない）。

Learn then Test（Angelopoulos ほか 2021, arXiv:2110.01052）と Conformal Risk Control（同 2022, arXiv:2208.02814）の、
見逃し（0 か 1 の損失）が単調に増える場合のいちばん簡単な形。numpy もいらない（math だけ）。

- 動かすのは OK の境の倍率 k（各レシピの t_ok x k。k = 1 が今の境）。不良 i は score_i < t_ok_i x k のとき OK（= 見逃し）
  比 r_i = score_i / t_ok_i とすると「r_i < k なら見逃し」。範囲外・形の見張りで保留になった不良は、どの k でも
  見逃しにならない（r = ∞）
- 校正の不良 n 枚での見逃しの枚数 m(k) は、k について単調に増える。だから「m(k) ≦ m*」を満たす一番ゆるい k は、
  比を小さい順に並べた (m*+1) 番目のすぐ下。単調なので、k の格子を前もって決めなくても LTT の保証は同じ
- 確からしさの保証（LTT）: m* =「P(Bin(n, α) ≦ m) ≦ δ」を満たす一番大きい m
  → P（本当の見逃しの率 ≦ α）≧ 1 − δ
- 期待値の保証（CRC）: (m + 1) / (n + 1) ≦ α を満たす一番大きい m → E［見逃しの率］≦ α
- 前提: 校正の不良と、これから判定する不良が同じ条件から来る（入れ替えても分布が同じ）。条件が変わったら保証はない
  → 範囲外の見張りと使用範囲書の面の振り分けで「同じ条件」を守り、外れたものは数えない
"""
from __future__ import annotations

import math

K_MAX = 3.0   # 不良が 1 枚も制約にならないときの上限（今の境の 3 倍まで。NG の境はふつうその手前）


def binom_cdf(m: int, n: int, p: float) -> float:
    """P(Bin(n, p) ≦ m)。n が数千でも桁あふれしないよう、対数で足す。"""
    if m < 0:
        return 0.0
    if m >= n:
        return 1.0
    if p <= 0.0:
        return 1.0
    if p >= 1.0:
        return 0.0
    lp, lq = math.log(p), math.log1p(-p)
    terms = [math.lgamma(n + 1) - math.lgamma(i + 1) - math.lgamma(n - i + 1) + i * lp + (n - i) * lq
             for i in range(m + 1)]
    top = max(terms)
    return min(1.0, math.exp(top) * sum(math.exp(t - top) for t in terms))


def binom_sf(m: int, n: int, p: float) -> float:
    """P(Bin(n, p) ≧ m)。"""
    return 1.0 - binom_cdf(m - 1, n, p)


def upper_bound(m: int, n: int, delta: float) -> float:
    """見逃し m / n から言える、本当の見逃しの率の上限（確からしさ 1−δ、片側の Clopper-Pearson）。

    P(Bin(n, p) ≦ m) = δ となる p。m = 0 なら 1 − δ^(1/n)。
    """
    if n <= 0:
        return 1.0
    if m >= n:
        return 1.0
    if m == 0:
        return 1.0 - delta ** (1.0 / n)
    lo, hi = m / n, 1.0
    for _ in range(80):   # 二分法（P(Bin ≦ m) は p について単調に減る）
        mid = 0.5 * (lo + hi)
        if binom_cdf(m, n, mid) > delta:
            lo = mid
        else:
            hi = mid
    return hi


def ltt_max_misses(n: int, alpha: float, delta: float) -> int:
    """LTT: 校正の不良 n 枚で、見逃しが何枚までなら「率 ≦ α（確からしさ 1−δ）」と言えるか。言えなければ −1。"""
    m = -1
    while m + 1 <= n and binom_cdf(m + 1, n, alpha) <= delta:
        m += 1
    return m


def crc_max_misses(n: int, alpha: float) -> int:
    """CRC: (m + 1) / (n + 1) ≦ α を満たす一番大きい m（期待値の保証）。言えなければ −1。"""
    return int(math.floor(alpha * (n + 1) - 1 + 1e-12))


def naive_max_misses(n: int, alpha: float) -> int:
    """保証なし（比べる用）: 校正の見逃しの率が α 以下になる一番大きい m。"""
    return int(math.floor(alpha * n + 1e-12))


def n_needed(alpha: float, delta: float, misses: int = 0, n_max: int = 1_000_000) -> int:
    """見逃し misses 枚で「率 ≦ α（確からしさ 1−δ）」と言うのに要る、不良の最少の枚数。"""
    n = max(misses + 1, 1)
    while n <= n_max:
        if binom_cdf(misses, n, alpha) <= delta:
            return n
        n += 1
    raise ValueError("n_max までに届かない")


def threshold_for(ratios, max_misses: int, k_max: float = K_MAX) -> float:
    """校正の不良の比 r（score / t_ok、見張りで保留なら inf）から、見逃しが max_misses 枚以下になる一番ゆるい倍率 k。

    max_misses < 0 なら 0（どれも OK にしない = 全部人へ）。比が 0 の不良（何も見つからない）が制約になると k = 0。
    """
    if max_misses < 0:
        return 0.0
    finite = sorted(r for r in ratios if math.isfinite(r))
    if max_misses >= len(finite):
        return k_max
    # (max_misses+1) 番目に小さい比のちょうど手前。掛け算の丸めで境の上に出ないよう、ほんの少し下げる
    return min(k_max, finite[max_misses] * (1.0 - 1e-9))


def misses_at(ratios, k: float) -> int:
    return sum(1 for r in ratios if r < k)


def guaranteed_threshold(ratios, alpha: float, delta: float = 0.1, method: str = "ltt",
                         k_max: float = K_MAX) -> dict:
    """校正の不良の比から、約束を守る一番ゆるい倍率 k と、その中身を返す。

    method: "ltt"（確からしさ 1−δ で率 ≦ α）/ "crc"（期待値で ≦ α）/ "naive"（保証なし。比べる用）
    """
    ratios = list(ratios)
    n = len(ratios)
    if method == "ltt":
        mm = ltt_max_misses(n, alpha, delta)
    elif method == "crc":
        mm = crc_max_misses(n, alpha)
    elif method == "naive":
        mm = naive_max_misses(n, alpha)
    else:
        raise ValueError(method)
    k = threshold_for(ratios, mm, k_max)
    m = misses_at(ratios, k)
    return {"method": method, "alpha": alpha, "delta": delta, "n": n, "max_misses": mm, "k": k, "misses": m,
            "feasible": mm >= 0, "bound": upper_bound(m, n, delta)}


def audit_stop(misses: int, n: int, alpha: float, level: float = 0.05) -> bool:
    """抜き取りの監査: わざと流した不良の見本 n 枚のうち misses 枚を AI が OK にした。

    約束（率 ≦ α）が本当なら、これ以上の見逃しが出る確率 P(Bin(n, α) ≧ misses) は小さい。それが level 以下なら、
    約束が破れていると判断して使用範囲書の版を止める（請求では、止めたあとの判定を数えない）。
    """
    return n > 0 and binom_sf(misses, n, alpha) <= level
