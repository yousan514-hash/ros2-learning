"""台帳と請求のテスト。改ざん・削除・偽の台帳を見抜けるかを確かめる。"""
import copy

import pytest

from robot_ledger.billing import Contract, LedgerInvalid, make_bill
from robot_ledger.ledger import Ledger, RobotIdentity, canonical, sha256_hex


def work_ledger(ident, errors=(0.049, 0.050, 0.049)):
    led = Ledger()
    led.append(ident, "session_start", {"points": len(errors)}, t=0.0)
    for i, err in enumerate(errors, start=1):
        led.append(ident, "task_done", {"point": i, "error": err}, t=float(i))
    led.append(ident, "session_end", {"tasks": len(errors)}, t=10.0)
    return led


@pytest.fixture
def robot():
    return RobotIdentity.generate()


def test_correct_ledger_has_no_problems(robot):
    led = work_ledger(robot)
    assert led.verify(trusted_public_key=robot.public_key_hex) == []
    assert led.entries[0].prev_hash == "0" * 64
    assert all(b.prev_hash == a.hash for a, b in zip(led.entries, led.entries[1:]))


def test_edited_data_is_detected(robot):
    led = work_ledger(robot)
    led.entries[2].data["error"] = 0.001  # 誤差を小さく書き換える
    assert any("書き換え" in p for p in led.verify())


def test_rehash_without_key_fails_signature(robot):
    """ハッシュを計算し直しても、秘密鍵がないので署名が合わない。"""
    led = work_ledger(robot)
    e = led.entries[2]
    e.data["error"] = 0.001
    e.hash = sha256_hex(canonical(e.body()))
    problems = led.verify()
    assert any("署名" in p for p in problems)
    assert any("つながっていない" in p for p in problems)  # 次の行の prev_hash とも合わない


def test_deleted_entry_is_detected(robot):
    led = work_ledger(robot)
    del led.entries[2]
    problems = led.verify()
    assert any("番号が飛んでいる" in p for p in problems)
    assert any("つながっていない" in p for p in problems)


def test_forged_ledger_needs_trusted_key(robot):
    """別の鍵で丸ごと作った偽の台帳は、中身だけ見ると正しい。登録された鍵と比べて初めて見抜ける。"""
    attacker = RobotIdentity.generate()
    fake = work_ledger(attacker, errors=(0.01,) * 10)
    assert fake.verify() == []
    assert any("登録された鍵" in p for p in fake.verify(trusted_public_key=robot.public_key_hex))


def test_save_and_load(robot, tmp_path):
    led = work_ledger(robot)
    path = tmp_path / "ledger.jsonl"
    led.save(path)
    loaded = Ledger.load(path)
    assert loaded.verify(trusted_public_key=robot.public_key_hex) == []
    assert loaded.head == led.head


def test_append_line_continues_chain(robot, tmp_path):
    path = tmp_path / "ledger.jsonl"
    led = Ledger()
    for i in range(3):
        Ledger.append_line(path, led.append(robot, "task_done", {"point": i, "error": 0.0}))
    assert Ledger.load(path).verify(trusted_public_key=robot.public_key_hex) == []


def test_key_file_is_reused(tmp_path):
    a = RobotIdentity.load_or_create(tmp_path / "robot.key")
    b = RobotIdentity.load_or_create(tmp_path / "robot.key")
    assert a.robot_id == b.robot_id
    assert (tmp_path / "robot.pub").read_text().strip() == a.public_key_hex


# ---- 請求 ----

def test_bill_counts_only_tasks_inside_the_range(robot):
    led = work_ledger(robot, errors=(0.04, 0.05, 0.20))
    bill = make_bill(led, Contract(robot.public_key_hex, price_per_task=100, max_error=0.05))
    assert (bill.counted, bill.excluded, bill.amount) == (2, 1, 200)


def test_bill_refuses_tampered_ledger(robot):
    led = work_ledger(robot)
    led.entries[1].data["error"] = 0.0
    with pytest.raises(LedgerInvalid):
        make_bill(led, Contract(robot.public_key_hex, price_per_task=100, max_error=0.05))


def test_bill_refuses_other_robot(robot):
    other = RobotIdentity.generate()
    with pytest.raises(LedgerInvalid):
        make_bill(work_ledger(other), Contract(robot.public_key_hex, price_per_task=100, max_error=0.05))


def test_original_untouched_by_deepcopy(robot):
    """テストの書き換えが元の台帳に影響しないことの確認（台帳の値はコピーして扱える）。"""
    led = work_ledger(robot)
    tampered = copy.deepcopy(led)
    tampered.entries[1].data["error"] = 9.9
    assert led.verify() == [] and tampered.verify() != []
