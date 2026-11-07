"""保証つきのしきい値（guarantee.py）の計算を、手で確かめられる値と比べる。"""
import math
import random

from dual_arm_inspection import guarantee as G


def test_binom_cdf_small():
    # Bin(3, 0.5): P(≦1) = (1 + 3) / 8
    assert math.isclose(G.binom_cdf(1, 3, 0.5), 0.5)
    assert G.binom_cdf(-1, 10, 0.3) == 0.0
    assert G.binom_cdf(10, 10, 0.3) == 1.0
    assert math.isclose(G.binom_sf(1, 3, 0.5), 7 / 8)


def test_upper_bound_zero_misses_matches_formula():
    # 見逃し 0/31 なら 1 − δ^(1/n)（確からしさ 90% で約 7.2%）
    assert math.isclose(G.upper_bound(0, 31, 0.10), 1 - 0.10 ** (1 / 31))
    assert 0.071 < G.upper_bound(0, 31, 0.10) < 0.073
    # m > 0 は二分法。定義 P(Bin(n, p) ≦ m) = δ を満たす
    p = G.upper_bound(3, 56, 0.10)
    assert math.isclose(G.binom_cdf(3, 56, p), 0.10, abs_tol=1e-9)
    assert p > 3 / 56


def test_n_needed():
    # 見逃し 0 で 2% を言うには: 0.98^n ≦ δ
    assert G.n_needed(0.02, 0.10) == math.ceil(math.log(0.10) / math.log(0.98))
    assert G.n_needed(0.02, 0.05) == math.ceil(math.log(0.05) / math.log(0.98))
    assert G.n_needed(0.02, 0.10, misses=1) > G.n_needed(0.02, 0.10)


def test_ltt_and_crc_max_misses():
    assert G.ltt_max_misses(31, 0.02, 0.10) == -1      # 31 枚では 2% は言えない
    assert G.ltt_max_misses(114, 0.02, 0.10) == 0
    assert G.ltt_max_misses(300, 0.02, 0.10) == 2      # P(Bin(300, .02) ≦ 2) ≈ 0.06、≦ 3 は ≈ 0.15
    assert G.crc_max_misses(300, 0.02) == 5             # (m + 1) / 301 ≦ 0.02
    assert G.crc_max_misses(31, 0.02) == -1


def test_threshold_for_keeps_misses():
    ratios = [0.2, 0.5, 0.5, 0.9, math.inf, 1.5]
    assert G.threshold_for(ratios, -1) == 0.0
    k = G.threshold_for(ratios, 1)
    assert G.misses_at(ratios, k) <= 1 and k < 0.5
    k = G.threshold_for(ratios, 2)                     # 同じ比の 2 枚は一緒に OK になるので、1 枚しか許せない
    assert G.misses_at(ratios, k) <= 2
    assert G.threshold_for(ratios, 10) == G.K_MAX     # 見張りで保留の不良（inf）は制約にならない


def test_ltt_guarantee_holds_in_simulation():
    """比が一様乱数の不良で、LTT で決めた k の本当の見逃しの率（= k）が α を超える回は δ 以下になる。"""
    rng = random.Random(3)
    alpha, delta, n, reps = 0.05, 0.10, 200, 400
    bad = 0
    for _ in range(reps):
        ratios = [rng.random() for _ in range(n)]
        k = G.guaranteed_threshold(ratios, alpha, delta)["k"]
        bad += k > alpha          # 一様分布なので、本当の見逃しの率 = P(r < k) = k
    assert bad / reps <= delta + 0.03


def test_audit_stop():
    # 上限 2% で見本 50 枚: 3 枚までは続ける、4 枚で止める（P(Bin(50, .02) ≧ 4) ≈ 0.018）
    assert not G.audit_stop(3, 50, 0.02)
    assert G.audit_stop(4, 50, 0.02)
    assert not G.audit_stop(0, 0, 0.02)
