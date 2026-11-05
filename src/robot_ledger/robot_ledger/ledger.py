"""ロボットの稼働台帳: 署名つきで、ハッシュでつないだ追記だけの記録。

ブロックチェーンの芯にあたる3つを、小さく自分で作る。
- ID: ロボットごとの鍵（Ed25519）。公開鍵のハッシュがロボットの ID になる
- 署名: 記録の1行ずつに、ロボットの秘密鍵で署名する。鍵を持たない人は偽の記録を作れない
- ハッシュの鎖: 各行に1つ前の行のハッシュを入れる。途中の1行を書き換えると、そこから先がつながらなくなる

何人もで同じ台帳を持ち合う仕組み（合意の取り方）は、まだ入れていない。
"""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey

GENESIS = "0" * 64  # 最初の行の「1つ前のハッシュ」


def canonical(obj) -> bytes:
    """同じ中身なら必ず同じバイト列になる JSON（ハッシュと署名の対象）。"""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def robot_id_from_public_key(public_key_hex: str) -> str:
    return "robot:" + sha256_hex(bytes.fromhex(public_key_hex))[:16]


class RobotIdentity:
    """ロボットの鍵と ID。秘密鍵はロボットの中だけに置き、Git には入れない。"""

    def __init__(self, private_key: Ed25519PrivateKey):
        self._key = private_key
        raw = private_key.public_key().public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)
        self.public_key_hex = raw.hex()
        self.robot_id = robot_id_from_public_key(self.public_key_hex)

    @classmethod
    def generate(cls) -> RobotIdentity:
        return cls(Ed25519PrivateKey.generate())

    @classmethod
    def load_or_create(cls, path: Path) -> RobotIdentity:
        """鍵のファイルがあれば読み、なければ作って保存する。公開鍵は隣の .pub に書く（登録用）。"""
        path = Path(path)
        if path.exists():
            return cls(serialization.load_pem_private_key(path.read_bytes(), password=None))
        ident = cls.generate()
        path.parent.mkdir(parents=True, exist_ok=True)
        pem = ident._key.private_bytes(
            serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()
        )
        path.write_bytes(pem)
        path.with_suffix(".pub").write_text(ident.public_key_hex + "\n", encoding="utf-8")
        return ident

    def sign(self, data: bytes) -> str:
        return self._key.sign(data).hex()


def verify_signature(public_key_hex: str, data: bytes, signature_hex: str) -> bool:
    try:
        key = Ed25519PublicKey.from_public_bytes(bytes.fromhex(public_key_hex))
        key.verify(bytes.fromhex(signature_hex), data)
        return True
    except (InvalidSignature, ValueError):
        return False


@dataclass
class Entry:
    seq: int
    robot_id: str
    public_key: str
    time: float
    event: str
    data: dict
    prev_hash: str
    hash: str = ""
    signature: str = ""

    def body(self) -> dict:
        """ハッシュと署名の対象（hash と signature 以外のすべて）。"""
        d = asdict(self)
        del d["hash"], d["signature"]
        return d


class Ledger:
    def __init__(self, entries: list[Entry] | None = None):
        self.entries: list[Entry] = list(entries or [])

    @property
    def head(self) -> str:
        """最後の行のハッシュ。台帳全体の「指紋」になる。"""
        return self.entries[-1].hash if self.entries else GENESIS

    def append(self, identity: RobotIdentity, event: str, data: dict | None = None, t: float | None = None) -> Entry:
        e = Entry(
            seq=len(self.entries),
            robot_id=identity.robot_id,
            public_key=identity.public_key_hex,
            time=time.time() if t is None else t,
            event=event,
            data=dict(data or {}),
            prev_hash=self.head,
        )
        e.hash = sha256_hex(canonical(e.body()))
        e.signature = identity.sign(bytes.fromhex(e.hash))
        self.entries.append(e)
        return e

    def verify(self, trusted_public_key: str | None = None) -> list[str]:
        """問題の一覧を返す。空なら台帳は正しい。

        trusted_public_key を渡さないと「中身が自分自身と矛盾していないか」しか確かめられない。
        別の鍵で丸ごと作り直した偽の台帳も通ってしまうので、請求などでは登録された鍵を必ず渡す。
        """
        problems = []
        prev = GENESIS
        for i, e in enumerate(self.entries):
            where = f"seq {e.seq}"
            if e.seq != i:
                problems.append(f"{where}: 番号が飛んでいる（{i} のはず）")
            if e.prev_hash != prev:
                problems.append(f"{where}: 1つ前の行とつながっていない")
            if sha256_hex(canonical(e.body())) != e.hash:
                problems.append(f"{where}: 中身が書き換えられている（ハッシュが合わない）")
            try:
                id_ok = robot_id_from_public_key(e.public_key) == e.robot_id
                signed = bytes.fromhex(e.hash)
            except ValueError:
                id_ok, signed = False, b""
            if not id_ok:
                problems.append(f"{where}: ID と公開鍵が合わない")
            if trusted_public_key and e.public_key != trusted_public_key:
                problems.append(f"{where}: 登録された鍵と違う鍵で署名されている")
            if not verify_signature(e.public_key, signed, e.signature):
                problems.append(f"{where}: 署名が正しくない")
            prev = e.hash
        return problems

    # ---- ファイル（1行に1つの記録、JSON Lines） ----

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            for e in self.entries:
                f.write(json.dumps(asdict(e), ensure_ascii=False) + "\n")

    @staticmethod
    def append_line(path: Path, entry: Entry) -> None:
        """1行だけ書き足す（記録しながら少しずつ保存する）。"""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(asdict(entry), ensure_ascii=False) + "\n")

    @classmethod
    def load(cls, path: Path) -> Ledger:
        path = Path(path)
        if not path.exists():
            return cls()
        with open(path, encoding="utf-8") as f:
            return cls([Entry(**json.loads(line)) for line in f if line.strip()])
