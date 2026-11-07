"""台帳の行を、peaq の記録の形に書き出す（手元だけ。IPFS にもチェーンにも送らない）。

形は peaq の公式の資料（2026-11-07 に読み直した）に合わせた:
- Verifiable Storage の封筒（peaq-ipfs-envelope@v1）
  docs.peaq.xyz/peaqchain/sdk-reference/robotics-sdk/ros2/messaging/verifiable-storage.md
  欄: payload.dataCid・payload.dataSha256・header.createdAt・header.robotId・proof.algorithm・
      proof.signatureHex・proof.publicKeyHex。ロボットの鍵が challenge "<dataSha256>|<createdAt>|<robotId>" に署名する。
  公式の確かめ方: (3) dataCid のデータの SHA-256 を dataSha256 と照らす (4) challenge を UTF-8 で作り直し、
  sr25519 なら challenge の SHA-256（32バイト）に、ed25519 なら challenge のバイト列そのものに対して、
  publicKeyHex で署名を確かめる。そのあと identity/read で DID の持ち主を確かめる
- peaqOS のイベント（events/validate・submit・batch_submit の欄）
  docs.peaq.xyz/peaqos/sdk-reference/ros2/services.md と docs.peaq.xyz/peaqos/concepts/events.md
  欄: machine_id・event_type（0 売上／1 活動）・value（通貨の最小単位の整数。円は 1円単位）・timestamp（Unix 秒）・
      raw_data_hex・trust_level（0 自己申告／1 チェーンで確かめられる／2 ハードウェアの署名）・source_chain_id
      （0 はチェーンの外）・source_tx_hash・metadata_hex。チェーンに載る dataHash は keccak256(raw_data)

公式の資料に書かれていない所（要確認。ここでは仮に決めた）:
- 封筒の中で "schema" をどこに置くか（ここでは一番外）、createdAt の形式（ここでは UTC の ISO 8601、ミリ秒まで）
- robotId（本当は did:peaq:...。DID を登録していないので、台帳のロボットの ID を仮に入れる）
- dataCid（IPFS に入れていないので仮。CIDv1・raw・sha2-256 を手元で計算する。1ブロックに収まる小さいデータなら
  IPFS の `--cid-version 1 --raw-leaves` と同じになるはずだが、IPFS では試していない）
- イベントの currency（概念のページにはあるが、ROS 2 のサービスの例には出てこない）
"""
from __future__ import annotations

import base64
import json
import re
from dataclasses import asdict
from datetime import datetime, timezone

from robot_ledger.ledger import Entry, RobotIdentity, canonical, sha256_hex, verify_signature

ENVELOPE_SCHEMA = "peaq-ipfs-envelope@v1"
EVENT_REVENUE, EVENT_ACTIVITY = 0, 1
TRUST_SELF_REPORTED, TRUST_ON_CHAIN, TRUST_HARDWARE = 0, 1, 2
CHAIN_OFF, CHAIN_PEAQ, CHAIN_BASE = 0, 3338, 8453
SUPPORTED_CHAIN_IDS = (CHAIN_OFF, CHAIN_PEAQ, CHAIN_BASE)
METADATA_MAX_BYTES = 4096  # peaq の EventRegistry の上限（これを超えると MetadataTooLarge）
MACHINE_ID_UNREGISTERED = 0  # 機械をまだ登録していない（仮）
_CURRENCY = re.compile(r"^[A-Z0-9]{3,10}$")
_HEX = re.compile(r"^0x([0-9a-fA-F]{2})*$")


# ---- keccak256（Ethereum・peaq の EVM が使うハッシュ）を標準ライブラリだけで ----
# hashlib の sha3_256 とは「最後の詰め物の1バイト」だけが違う（SHA-3 は 0x06、keccak は 0x01）。
# 置換（Keccak-f[1600]）は同じなので、テストでは同じ関数の SHA-3 の側を hashlib と突き合わせて確かめる。
_MASK = (1 << 64) - 1


def _rc_bit(t: int) -> int:
    r = 1
    for _ in range(t % 255):
        r <<= 1
        if r & 0x100:
            r ^= 0x171
    return r & 1


_RC = [sum(_rc_bit(j + 7 * i) << ((1 << j) - 1) for j in range(7)) for i in range(24)]
_ROT = [[0] * 5 for _ in range(5)]
_x, _y = 1, 0
for _t in range(24):
    _ROT[_x][_y] = ((_t + 1) * (_t + 2) // 2) % 64
    _x, _y = _y, (2 * _x + 3 * _y) % 5


def _rotl(v: int, n: int) -> int:
    return ((v << n) | (v >> (64 - n))) & _MASK if n else v


def _keccak_f(a: list[int]) -> list[int]:
    for rc in _RC:
        c = [a[x] ^ a[x + 5] ^ a[x + 10] ^ a[x + 15] ^ a[x + 20] for x in range(5)]
        d = [c[(x - 1) % 5] ^ _rotl(c[(x + 1) % 5], 1) for x in range(5)]
        a = [a[i] ^ d[i % 5] for i in range(25)]
        b = [0] * 25
        for x in range(5):
            for y in range(5):
                b[y + 5 * ((2 * x + 3 * y) % 5)] = _rotl(a[x + 5 * y], _ROT[x][y])
        a = [b[i] ^ (~b[(i % 5 + 1) % 5 + 5 * (i // 5)] & b[(i % 5 + 2) % 5 + 5 * (i // 5)]) for i in range(25)]
        a[0] ^= rc
    return a


def _sponge(data: bytes, pad: int, rate: int = 136, out_len: int = 32) -> bytes:
    msg = bytearray(data)
    msg.append(pad)
    msg.extend(b"\x00" * (-len(msg) % rate))
    msg[-1] |= 0x80
    state = [0] * 25
    for off in range(0, len(msg), rate):
        block = msg[off:off + rate]
        for i in range(rate // 8):
            state[i] ^= int.from_bytes(block[8 * i:8 * i + 8], "little")
        state = _keccak_f(state)
    return b"".join(state[i].to_bytes(8, "little") for i in range(rate // 8))[:out_len]


def keccak256(data: bytes) -> bytes:
    return _sponge(data, 0x01)


def sha3_256_pure(data: bytes) -> bytes:
    """テスト用: 同じ置換で SHA3-256 を作る（hashlib.sha3_256 と一致すれば、置換が正しい）。"""
    return _sponge(data, 0x06)


# ---- CID（仮）----
def cid_v1_raw(data: bytes) -> str:
    """CIDv1（raw 0x55・sha2-256 0x12・32バイト）を base32 の小文字で（先頭 b）。IPFS には入れない。"""
    import hashlib

    raw = bytes([0x01, 0x55, 0x12, 0x20]) + hashlib.sha256(data).digest()
    return "b" + base64.b32encode(raw).decode("ascii").lower().rstrip("=")


# ---- Verifiable Storage の封筒 ----
def created_at_iso(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def challenge(data_sha256: str, created_at: str, robot_id: str) -> bytes:
    return f"{data_sha256}|{created_at}|{robot_id}".encode("utf-8")


def make_envelope(data: bytes, identity: RobotIdentity, created_at: str, robot_id: str | None = None,
                  with_cid: bool = True) -> dict:
    """data（台帳の1行など）の封筒を作る。署名は ed25519（台帳と同じ鍵）。"""
    rid = robot_id or identity.robot_id
    sha = sha256_hex(data)
    return {
        "schema": ENVELOPE_SCHEMA,
        "header": {"createdAt": created_at, "robotId": rid},
        "payload": {"dataCid": cid_v1_raw(data) if with_cid else "", "dataSha256": sha},
        "proof": {"algorithm": "ed25519", "publicKeyHex": identity.public_key_hex,
                  "signatureHex": identity.sign(challenge(sha, created_at, rid))},
    }


def verify_envelope(envelope: dict, data: bytes, trusted_public_key: str | None = None) -> list[str]:
    """公式の確かめ方の手順 3・4 に沿って確かめる。問題の一覧を返す（空なら正しい）。

    手順 4 の最後の「identity/read で DID の持ち主を確かめる」は、DID を登録していないので、
    信頼する鍵の一覧（trusted_public_key）と比べることで代える。
    """
    problems = []
    try:
        header, payload, proof = envelope["header"], envelope["payload"], envelope["proof"]
        sha, cid = payload["dataSha256"], payload["dataCid"]
        created_at, robot_id = header["createdAt"], header["robotId"]
        alg, sig, pub = proof["algorithm"], proof["signatureHex"], proof["publicKeyHex"]
    except (KeyError, TypeError):
        return ["封筒の欄が足りない"]
    if envelope.get("schema") != ENVELOPE_SCHEMA:
        problems.append(f"封筒の形式が {ENVELOPE_SCHEMA} でない")
    # 手順 3: データの SHA-256 を dataSha256 と照らす（CID が入っていれば、それも計算し直して照らす）
    if sha256_hex(data) != sha:
        problems.append("データの SHA-256 が dataSha256 と合わない（データが書き換えられている）")
    if cid and cid_v1_raw(data) != cid:
        problems.append("データの CID が dataCid と合わない")
    # 手順 4: challenge を作り直して署名を確かめる
    msg = challenge(sha, created_at, robot_id)
    if alg == "ed25519":
        if not verify_signature(pub, msg, sig):
            problems.append("署名が正しくない（challenge の作り直しで合わない）")
    elif alg == "sr25519":
        problems.append("sr25519 の署名は、この手元の実装では確かめられない")
    else:
        problems.append(f"知らない署名の方式: {alg}")
    if trusted_public_key and pub != trusted_public_key:
        problems.append("登録された鍵と違う鍵で署名されている")
    return problems


def entry_bytes(entry: Entry) -> bytes:
    """台帳の1行（hash と signature も含む全部）を、封筒・イベントに入れるデータにする。"""
    return canonical(asdict(entry))


def entry_envelope(entry: Entry, identity: RobotIdentity) -> dict:
    return make_envelope(entry_bytes(entry), identity, created_at_iso(entry.time))


# ---- peaqOS のイベント ----
def make_event(event_type: int, value: int, timestamp: int, raw_data: bytes | None, metadata: bytes = b"",
               *, machine_id: int = MACHINE_ID_UNREGISTERED, trust_level: int = TRUST_SELF_REPORTED,
               source_chain_id: int = CHAIN_OFF, source_tx_hash: str = "", currency: str = "") -> dict:
    return {"machine_id": machine_id, "event_type": event_type, "value": value, "currency": currency,
            "timestamp": timestamp, "raw_data_hex": "0x" + raw_data.hex() if raw_data else "",
            "trust_level": trust_level, "source_chain_id": source_chain_id, "source_tx_hash": source_tx_hash,
            "metadata_hex": "0x" + metadata.hex()}


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def validate_event(ev: dict, now: int | None = None) -> tuple[list[str], str | None]:
    """公式の資料に書かれた欄の決まりで確かめ、チェーンに載る dataHash（keccak256(raw_data)）を手元で計算する。

    events/validate と同じく、ガス代を使う前の確認にあたる。戻り値は (問題の一覧, dataHash)。
    """
    p = []
    if ev.get("event_type") not in (EVENT_REVENUE, EVENT_ACTIVITY):
        p.append("event_type は 0（売上）か 1（活動）")
    value = ev.get("value")
    if not _is_int(value) or value < 0:
        p.append("value は 0 以上の整数（通貨の最小単位）")
    cur = ev.get("currency", "")
    if ev.get("event_type") == EVENT_REVENUE and not _CURRENCY.match(cur or ""):
        p.append("売上のイベントの currency は大文字の英数字 3〜10 字")
    if ev.get("event_type") == EVENT_ACTIVITY and cur != "":
        p.append("活動のイベントの currency は空にする")
    ts = ev.get("timestamp")
    if not _is_int(ts) or ts <= 0:
        p.append("timestamp は正の整数（Unix 秒）")
    elif now is not None and ts > now:
        p.append("timestamp が未来（チェーンの EventRegistry は FutureTimestamp で拒む）")
    trust = ev.get("trust_level")
    if trust not in (TRUST_SELF_REPORTED, TRUST_ON_CHAIN, TRUST_HARDWARE):
        p.append("trust_level は 0・1・2 のどれか")
    if trust == TRUST_ON_CHAIN and not ev.get("source_tx_hash"):
        p.append("trust_level 1（チェーンで確かめられる）には source_tx_hash が要る")
    if ev.get("source_chain_id") not in SUPPORTED_CHAIN_IDS:
        p.append(f"source_chain_id は {SUPPORTED_CHAIN_IDS} のどれか")
    if not _is_int(ev.get("machine_id")) or ev.get("machine_id") < 0:
        p.append("machine_id は 0 以上の整数")
    raw_hex = ev.get("raw_data_hex", "")
    data_hash = None
    if raw_hex != "":
        if raw_hex == "0x" or not _HEX.match(raw_hex or ""):
            p.append("raw_data_hex は、入れるなら空でない 16進（'0x' だけは拒まれる）。入れないなら空文字")
        else:
            data_hash = "0x" + keccak256(bytes.fromhex(raw_hex[2:])).hex()
    meta = ev.get("metadata_hex", "0x")
    if not _HEX.match(meta or ""):
        p.append("metadata_hex は 16進")
    elif len(meta[2:]) // 2 > METADATA_MAX_BYTES:
        p.append(f"metadata は {METADATA_MAX_BYTES} バイトまで（MetadataTooLarge）")
    return p, data_hash


def inspection_event(entry: Entry, counted: bool) -> dict:
    """検査1件 → 活動のイベント（value は 1件）。raw_data は台帳の1行そのもの、metadata は要点だけ。"""
    d = entry.data
    meta = {"schema": "inspection-ledger@0", "seq": entry.seq, "verdict": d.get("verdict"),
            "in_range": d.get("in_range"), "billable": counted, "entry_hash": entry.hash}
    return make_event(EVENT_ACTIVITY, 1, int(entry.time), entry_bytes(entry), canonical(meta))


def bill_event(summary: dict, scope_fp: str, cell_head: str, factory_head: str, t: float) -> dict:
    """請求額 → 売上のイベントの形（JPY、1円単位）。支払いが済んだら刻む想定の見本（お金は動かさない）。"""
    amount = max(int(summary["amount"]), 0)
    raw = canonical({"schema": "inspection-bill@0", "scope": scope_fp, "cell_head": cell_head,
                     "factory_head": factory_head, "summary": summary})
    meta = canonical({"schema": "inspection-bill@0", "scope": scope_fp, "cell_head": cell_head})
    return make_event(EVENT_REVENUE, amount, int(t), raw, meta, currency="JPY")


def dumps(obj) -> str:
    return json.dumps(obj, ensure_ascii=False)
