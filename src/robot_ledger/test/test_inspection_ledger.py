"""検査の台帳のテスト: 使用範囲書で数える・工場の受け取りの署名・見逃しの申し立て・改ざんの見抜き。"""
import copy

import pytest

from robot_ledger.billing import LedgerInvalid
from robot_ledger.inspection_ledger import (
    Factory, ReceiptMismatch, TrustedKeys, UsageScope, inspection_record, is_in_range,
    make_inspection_bill, record_inspection, start_cell_ledger, thresholds_version,
)
from robot_ledger.ledger import Ledger, RobotIdentity, canonical, sha256_hex

IMG = "ab" * 32
TH = thresholds_version({"t_ok": 3.0, "t_ng": 9.0})
# (部品, 判定, reason): 範囲内の OK 2・NG 1・保留 1、範囲外 1
SHOTS = [("bolt_M6x30", "OK", "clean"), ("nut_M6", "NG", "defect"), ("bolt_M6x30", "HOLD", "uncertain"),
         ("washer_M6", "HOLD", "out_of_range:gain"), ("nut_M6", "OK", "clean")]


@pytest.fixture
def world():
    cell, factory_id = RobotIdentity.generate(), RobotIdentity.generate()
    keys = TrustedKeys(cell.public_key_hex, factory_id.public_key_hex)
    scope = UsageScope(cell.robot_id, TH, price_per_inspection=100, refund_per_miss=1000,
                       refund_per_overkill=100, claim_window_s=3600.0)
    led = start_cell_ledger(cell, scope, t=0.0)
    for i, (part, verdict, reason) in enumerate(SHOTS, start=1):
        record_inspection(led, cell, inspection_record(part, "face", verdict, 1.0, reason, TH, IMG), t=float(i))
    factory = Factory(factory_id, keys.cell)
    return cell, factory, keys, scope, led


def test_record_marks_range_and_refuses_auto_verdict_out_of_range():
    assert is_in_range("clean") and is_in_range("uncertain")
    assert not is_in_range("out_of_range:background") and not is_in_range("shape") and not is_in_range("no_part")
    rec = inspection_record("nut_M6", "top", "HOLD", 0.0, "out_of_range:gain", TH, IMG)
    assert rec["in_range"] is False and "truth" not in rec
    with pytest.raises(ValueError):  # 範囲外なのに AI が自分で決めた（OK）ことにはできない
        inspection_record("nut_M6", "top", "OK", 0.0, "out_of_range:gain", TH, IMG)
    with pytest.raises(ValueError):
        inspection_record("nut_M6", "top", "OK", 0.0, "clean", TH, "not-a-hash")


def test_counts_only_auto_decisions_in_range_with_receipt(world):
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=len(led.entries) - 1, t=10.0)
    bill = make_inspection_bill(led, factory.ledger, scope, keys)
    s = bill.summary()
    assert (s["counted_ok"], s["counted_ng"], s["hold"], s["out_of_range"]) == (2, 1, 1, 1)
    assert bill.amount == 300


def test_batch_without_receipt_is_not_counted(world):
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=2, t=10.0)  # seq 1〜2 だけ受け取った（3〜5 はまだ）
    bill = make_inspection_bill(led, factory.ledger, scope, keys)
    assert bill.count("counted") == 2 and bill.count("not_received") == 1  # seq 5 の OK は受け取り待ち
    factory.receive(led, scope, upto_seq=5, t=11.0)  # 次のまとまり（seq 3〜5）
    assert factory.ledger.entries[-1].data["from_seq"] == 3
    assert make_inspection_bill(led, factory.ledger, scope, keys).count("counted") == 3


def test_edited_line_without_key_is_refused(world):
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=5, t=10.0)
    led.entries[3].data["verdict"] = "OK"  # 保留を OK に書き換えて、請求を増やそうとする
    with pytest.raises(LedgerInvalid, match="書き換え"):
        make_inspection_bill(led, factory.ledger, scope, keys)


def test_cell_cannot_rewrite_history_after_factory_receipt(world):
    """セルが自分の鍵で台帳を丸ごと作り直す（保留 → OK）と、鍵の確認は通るが、工場の受け取りと合わない。"""
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=5, t=10.0)
    forged = start_cell_ledger(cell, scope, t=0.0)
    for e in led.entries[1:]:
        data = dict(e.data, verdict="OK", reason="clean") if e.data["verdict"] == "HOLD" and e.data["in_range"] else e.data
        record_inspection(forged, cell, data, t=e.time)
    assert forged.verify(trusted_public_key=keys.cell) == []  # 1者の署名だけなら「正しい」台帳に見える
    with pytest.raises(ReceiptMismatch):
        make_inspection_bill(forged, factory.ledger, scope, keys)


def test_receipt_signed_by_untrusted_key_is_refused(world):
    cell, factory, keys, scope, led = world
    fake = Factory(RobotIdentity.generate(), keys.cell)  # 工場のふりをする別の鍵
    fake.receive(led, scope, upto_seq=5, t=10.0)
    with pytest.raises(LedgerInvalid, match="工場の台帳: .*登録された鍵と違う"):
        make_inspection_bill(led, fake.ledger, scope, keys)


def test_scope_conditions_thresholds_version_and_parts(world):
    cell, factory, keys, scope, led = world
    record_inspection(led, cell, inspection_record("nut_M6", "f", "OK", 1.0, "clean", "th:old", IMG), t=6.0)
    factory.receive(led, scope, upto_seq=6, t=10.0)
    bill = make_inspection_bill(led, factory.ledger, scope, keys)
    assert bill.count("thresholds") == 1 and bill.count("counted") == 3
    narrow = UsageScope(**{**scope.__dict__, "parts": ["nut_M6"]})  # 契約がナットだけなら、ボルトは数えない
    led2 = start_cell_ledger(cell, narrow, t=0.0)
    for e in led.entries[1:6]:
        record_inspection(led2, cell, e.data, t=e.time)
    f2 = Factory(factory.identity, keys.cell)
    f2.receive(led2, narrow, upto_seq=5, t=10.0)
    bill2 = make_inspection_bill(led2, f2.ledger, narrow, keys)
    assert bill2.count("part") == 1 and bill2.count("counted") == 2


def test_claims_refund_only_valid_ones(world):
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=5, t=10.0)
    factory.claim(led, scope, 1, "miss", "後の工程で傷が見つかった", t=100.0)       # OK の行 → 認める
    factory.claim(led, scope, 1, "miss", "同じ行にもう一度", t=101.0)             # 2回目 → 認めない
    factory.claim(led, scope, 3, "miss", "保留の行", t=102.0)                    # 人に回した行 → 認めない
    factory.claim(led, scope, 2, "miss", "NG の行に見逃し", t=103.0)              # 種類が合わない → 認めない
    factory.claim(led, scope, 5, "overkill", "期限切れ", t=10.0 + 3600.0 + 1.0)  # OK の行に過検出・期限切れ
    factory.claim(led, scope, 2, "overkill", "良品だった", t=104.0)              # NG の行 → 認める
    bill = make_inspection_bill(led, factory.ledger, scope, keys)
    assert len(bill.claims_accepted) == 2 and len(bill.claims_rejected) == 4
    assert bill.refund_total == 1000 + 100
    assert bill.amount == 300 - 1100  # 減額が上回ればマイナス（返す額）


def test_ledger_started_under_another_scope_is_refused(world):
    cell, factory, keys, scope, led = world
    other = UsageScope(**{**scope.__dict__, "price_per_inspection": 999})
    with pytest.raises(LedgerInvalid, match="使用範囲書"):
        factory.receive(led, other, upto_seq=5)
    factory.receive(led, scope, upto_seq=5, t=10.0)
    with pytest.raises(LedgerInvalid, match="使用範囲書"):
        make_inspection_bill(led, factory.ledger, other, keys)


def test_trusted_keys_file_has_public_keys_only(world, tmp_path):
    cell, factory, keys, scope, led = world
    path = tmp_path / "trusted_keys.json"
    keys.save(path)
    assert TrustedKeys.load(path) == keys
    text = path.read_text(encoding="utf-8")
    assert "PRIVATE" not in text and keys.cell in text and keys.cell_id in text


def test_saved_ledgers_round_trip_and_still_bill(world, tmp_path):
    cell, factory, keys, scope, led = world
    factory.receive(led, scope, upto_seq=5, t=10.0)
    led.save(tmp_path / "cell.jsonl")
    factory.ledger.save(tmp_path / "factory.jsonl")
    bill = make_inspection_bill(Ledger.load(tmp_path / "cell.jsonl"), Ledger.load(tmp_path / "factory.jsonl"),
                                scope, keys)
    assert bill.amount == 300
    assert scope.fingerprint() == sha256_hex(canonical(copy.deepcopy(scope).__dict__))


def test_shot_matcher_pairs_topics_in_any_order():
    from robot_ledger.shot_matcher import ShotMatcher

    m = ShotMatcher()
    assert m.add_result({"seq": 1, "verdict": "OK"}) == []  # 結果が先に届いても待つ
    assert m.add_image((10, 5), b"pixels") == []
    ready = m.add_shot({"seq": 1, "stamp": [10, 5], "part": "nut_M6"})
    assert ready == [({"seq": 1, "verdict": "OK"}, sha256_hex(b"pixels"))]
    m.add_shot({"seq": 2, "stamp": [11, 0]})
    assert m.add_result({"seq": 2}) == []  # 画像がまだ
    assert m.add_image((11, 0), b"other")[0][1] == sha256_hex(b"other")
