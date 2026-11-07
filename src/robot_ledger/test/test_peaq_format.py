"""peaq の記録の形に書き出すテスト。公式の資料に書かれた確かめ方・欄の決まりに沿って確かめる。"""
import hashlib
import os

import pytest

from robot_ledger import peaq_format as P
from robot_ledger.inspection_ledger import inspection_record, thresholds_version
from robot_ledger.ledger import Ledger, RobotIdentity, sha256_hex, verify_signature


@pytest.fixture
def robot():
    return RobotIdentity.generate()


def test_keccak256_known_values_and_same_permutation_as_sha3():
    assert P.keccak256(b"").hex() == "c5d2460186f7233c927e7db2dcc703c0e500b653ca82273b7bfad8045d85a470"
    assert P.keccak256(b"abc").hex() == "4e03657aea45a94fc7d47ba826c8d667c0d1e6e33a64a036ec44f58fa12d6c45"
    # 詰め物の1バイトだけ変えた SHA3-256 が hashlib と一致する = 置換が正しい（長さはブロックの境目を含む）
    for n in (0, 1, 135, 136, 137, 271, 272, 1000):
        m = os.urandom(n)
        assert P.sha3_256_pure(m) == hashlib.sha3_256(m).digest()
    assert P.keccak256(b"abc") != hashlib.sha3_256(b"abc").digest()


def test_cid_v1_raw_matches_known_empty_block():
    assert P.cid_v1_raw(b"") == "bafkreihdwdcefgh4dqkjv67uzcmw7ojee6xedzdetojuzjevtenxquvyku"


def test_envelope_follows_official_verification_steps(robot):
    data = b'{"verdict":"OK"}'
    env = P.make_envelope(data, robot, "2026-11-07T00:00:00.000Z")
    assert env["schema"] == "peaq-ipfs-envelope@v1"
    assert env["payload"]["dataSha256"] == hashlib.sha256(data).hexdigest()
    # ed25519 は challenge "<dataSha256>|<createdAt>|<robotId>"（UTF-8）のバイト列そのものに署名する
    msg = f"{env['payload']['dataSha256']}|2026-11-07T00:00:00.000Z|{robot.robot_id}".encode("utf-8")
    assert verify_signature(env["proof"]["publicKeyHex"], msg, env["proof"]["signatureHex"])
    assert P.verify_envelope(env, data, trusted_public_key=robot.public_key_hex) == []


def test_envelope_detects_tampering(robot):
    data = b'{"verdict":"HOLD"}'
    env = P.make_envelope(data, robot, "2026-11-07T00:00:00.000Z")
    assert any("SHA-256" in p for p in P.verify_envelope(env, b'{"verdict":"OK"}'))
    moved = {**env, "header": {**env["header"], "createdAt": "2026-11-08T00:00:00.000Z"}}
    assert any("署名" in p for p in P.verify_envelope(moved, data))
    other = RobotIdentity.generate()
    assert any("登録された鍵" in p for p in P.verify_envelope(env, data, trusted_public_key=other.public_key_hex))
    sr = {**env, "proof": {**env["proof"], "algorithm": "sr25519"}}
    assert any("sr25519" in p for p in P.verify_envelope(sr, data))


def test_event_rules_from_official_docs():
    good = P.make_event(P.EVENT_ACTIVITY, 1, 1770000000, b"{}", b"{}")
    problems, data_hash = P.validate_event(good, now=1770000001)
    assert problems == [] and data_hash == "0x" + P.keccak256(b"{}").hex()
    rev = P.make_event(P.EVENT_REVENUE, 1200, 1770000000, None, b"{}", currency="JPY")
    assert P.validate_event(rev) == ([], None)
    bad = {
        "event_type": {**good, "event_type": 2},
        "value": {**good, "value": -1},
        "currency_activity": {**good, "currency": "JPY"},
        "currency_revenue": {**rev, "currency": "yen"},
        "raw_0x": {**good, "raw_data_hex": "0x"},
        "trust_1_without_tx": {**good, "trust_level": 1},
        "chain": {**good, "source_chain_id": 1},
        "metadata_size": {**good, "metadata_hex": "0x" + "00" * 4097},
        "future": good,
    }
    for name, ev in bad.items():
        problems, _ = P.validate_event(ev, now=1769999999 if name == "future" else None)
        assert problems, name


def test_ledger_entry_to_envelope_and_event(robot):
    led = Ledger()
    th = thresholds_version({"t_ok": 1})
    e = led.append(robot, "inspection",
                   inspection_record("nut_M6", "top", "OK", 2.5, "clean", th, sha256_hex(b"png")), t=1770000000.25)
    env = P.entry_envelope(e, robot)
    assert env["header"]["createdAt"] == "2026-02-02T02:40:00.250Z"
    assert P.verify_envelope(env, P.entry_bytes(e), robot.public_key_hex) == []
    ev = P.inspection_event(e, counted=True)
    problems, data_hash = P.validate_event(ev)
    assert problems == [] and ev["timestamp"] == 1770000000
    assert data_hash == "0x" + P.keccak256(P.entry_bytes(e)).hex()
    e.data["verdict"] = "NG"  # 封筒を作ったあとで台帳の行を書き換える → 照らし合わせで見抜ける
    assert P.verify_envelope(env, P.entry_bytes(e), robot.public_key_hex)
