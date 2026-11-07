"""検査の台帳: 外観検査の1件を台帳の1行にし、使用範囲書と工場の受け取りの署名で請求額を出す。

ロボット（双腕の検査セル）× AI（OK／NG／保留の3段判定）× ブロックチェーンの芯（署名・ハッシュの鎖・鍵の登録）を
1本につなぐ。お金は動かさない（円の金額を出すだけ）。

- 検査セルの台帳: 先頭の行に「どの使用範囲書で働くか」（指紋）を書き、そのあと 1行 = 検査1件。
  セルの鍵で署名し、前の行のハッシュでつなぐ（ledger.Ledger をそのまま使う）。画像そのものは入れず SHA-256 だけ
- 使用範囲書（UsageScope）: 何を数えるかを前もって決める。範囲内で AI が自分で決めた分（OK／NG）だけを数え、
  保留（人に回した分）と範囲外は数えない。後から見逃し・過検出がわかったときの減額と、申し立ての期限も書く
- 工場の台帳: 工場が自分の鍵で「セルの台帳のどこまでを受け取ったか」（受け取りの署名）と、
  「後から見つかった見逃し・過検出の申し立て」を書く
- 請求（make_inspection_bill）: 2つの台帳を、信頼する鍵の一覧（TrustedKeys）で確かめてから数える。
  受け取りの署名がない分は数えない。受け取ったあとでセルが自分の鍵で台帳を作り直しても、
  受け取りの署名に入れたハッシュと合わなくなるので見抜ける（1者の署名では防げない所を、2者の署名で防ぐ）
- 使用範囲書の版（version・auto_faces・human_faces・min_contrast・basis）: 面ごとに「AI に任せる／人に回す」と、
  検出を約束する欠陥の濃さを書ける。人に回す面は、セルが AI の判定を影（ai_verdict）に残して保留にし（route_by_scope）、
  請求では数えない。約束の濃さより薄い欠陥の見逃しは、申し立てても減額しない。版の欄を書かなければ前と同じ指紋になる
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from fnmatch import fnmatchcase
from pathlib import Path

from robot_ledger.billing import LedgerInvalid
from robot_ledger.ledger import Entry, Ledger, RobotIdentity, canonical, robot_id_from_public_key, sha256_hex

VERDICTS = ("OK", "NG", "HOLD")
EV_SCOPE = "scope"
EV_INSPECTION = "inspection"
EV_RECEIPT = "receipt"
EV_CLAIM = "claim"
CLAIM_KINDS = {"miss": "OK", "overkill": "NG"}  # 申し立ての種類 → その申し立てができる判定
OUT_OF_RANGE_REASONS = ("shape", "no_part")    # 部品の面積が範囲の外・部品が写っていない（検査の reason）


def is_in_range(reason: str) -> bool:
    """検査の reason から「使用範囲の中で撮れたか」を決める（範囲外の見張りに引っかかったら False）。"""
    return not (reason.startswith("out_of_range") or reason in OUT_OF_RANGE_REASONS)


def thresholds_version(calibration: dict) -> str:
    """しきい値の版: 良品で学んだ結果（しきい値と使用範囲）の指紋。学び直すと版が変わる。"""
    return "th:" + sha256_hex(canonical(calibration))[:16]


def inspection_record(part: str, face: str, verdict: str, score: float, reason: str, thresholds: str,
                      image_sha256: str, *, defect_type: str | None = None, shot: str | int | None = None) -> dict:
    """台帳の1行に入れる中身（検査1件）。正解（本当に不良か）はセルにはわからないので入れない。"""
    if verdict not in VERDICTS:
        raise ValueError(f"判定は {VERDICTS} のどれか: {verdict}")
    if len(image_sha256) != 64 or any(c not in "0123456789abcdef" for c in image_sha256):
        raise ValueError("画像の SHA-256 は 16進 64桁（小文字）")
    in_range = is_in_range(reason)
    if not in_range and verdict != "HOLD":
        raise ValueError(f"範囲外（{reason}）なのに {verdict}: 範囲外は必ず保留にして人に回す")
    return {"part": part, "face": face, "verdict": verdict, "score": round(float(score), 2), "reason": reason,
            "defect_type": defect_type, "in_range": in_range, "thresholds": thresholds,
            "image_sha256": image_sha256, "shot": shot}


@dataclass
class TrustedKeys:
    """信頼する鍵の一覧（公開鍵だけ。秘密鍵は入れない）。今は手で渡す。いずれ DID の登録先から読む（案）。"""
    cell: str      # 検査セルの公開鍵（16進）
    factory: str   # 受け取る側（工場）の公開鍵（16進）

    @property
    def cell_id(self) -> str:
        return robot_id_from_public_key(self.cell)

    @property
    def factory_id(self) -> str:
        return robot_id_from_public_key(self.factory)

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        doc = {"cell": {"id": self.cell_id, "public_key": self.cell},
               "factory": {"id": self.factory_id, "public_key": self.factory}}
        path.write_text(json.dumps(doc, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> TrustedKeys:
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(doc["cell"]["public_key"], doc["factory"]["public_key"])


@dataclass
class UsageScope:
    """使用範囲書: 検査セルと工場が前もって決める「数える条件」。指紋を両方の台帳に書いて、同じ契約だと示す。"""
    cell_id: str                    # どのセルの仕事か
    thresholds: str                 # しきい値の版（この版で決めた判定だけを数える）
    price_per_inspection: int       # 範囲内で AI が自分で決めた1件（OK／NG）の料金（円）
    refund_per_miss: int            # 見逃し（OK と出したが不良）が後でわかったときの1件の減額（円）
    refund_per_overkill: int        # 過検出（NG と出したが良品）が後でわかったときの1件の減額（円）
    claim_window_s: float = 30 * 24 * 3600.0  # 受け取りから何秒までの申し立てを認めるか
    parts: list[str] = field(default_factory=list)  # 対象の部品（空ならすべて）
    name: str = ""
    # ---- 使用範囲書の版（2026-11-07 に足した欄）。書かなければ指紋は前と同じ ----
    version: str = ""                                     # 版の名前（例 "v2"）
    auto_faces: list[str] = field(default_factory=list)   # AI に任せる面「部品/面」の型（例 "bolt_*/head_end"）。空ならすべての面
    human_faces: list[str] = field(default_factory=list)  # 人に回す面（auto_faces より強い）
    min_contrast: float | None = None                     # 検出を約束する欠陥の濃さ（0〜1）。これより薄い見逃しは減額しない
    basis: str = ""                                       # 根拠（どの評価から決めたか）

    def __post_init__(self):
        for pat in self.auto_faces + self.human_faces:
            if "/" not in pat:
                raise ValueError(f"面は「部品/面」の型で書く（例 bolt_*/head_end）: {pat}")
        if self.min_contrast is not None and not 0.0 < self.min_contrast <= 1.0:
            raise ValueError(f"約束する濃さは 0 より大きく 1 以下: {self.min_contrast}")
        # 並べ方で指紋が変わらないように、面の型は並べ替えて重なりを消す
        self.auto_faces = sorted(set(self.auto_faces))
        self.human_faces = sorted(set(self.human_faces))

    def fingerprint(self) -> str:
        """指紋 = 中身の正規化 JSON の SHA-256。版の欄は、書いたときだけ入れる（前の版の台帳がそのまま読める）。"""
        d = asdict(self)
        for k, default in VERSION_DEFAULTS.items():
            if d[k] == default:
                del d[k]
        return sha256_hex(canonical(d))

    def to_dict(self) -> dict:
        return asdict(self) | {"fingerprint": self.fingerprint()}

    @classmethod
    def from_dict(cls, d: dict) -> UsageScope:
        """保存した使用範囲書を読む。指紋が書いてあれば、中身から計算し直した指紋と合うか確かめる。"""
        d = dict(d)
        fp = d.pop("fingerprint", None)
        scope = cls(**d)
        if fp is not None and fp != scope.fingerprint():
            raise LedgerInvalid(["使用範囲書のファイルの中身と、書いてある指紋が合わない（書き換えられた）"])
        return scope

    def face_mode(self, part: str, face: str) -> str:
        """この面を AI に任せるか（"auto"）、人に回すか（"human"）。人に回すが優先。書いていない面は人へ。"""
        key = f"{part}/{face}"
        if any(fnmatchcase(key, p) for p in self.human_faces):
            return "human"
        if self.auto_faces and not any(fnmatchcase(key, p) for p in self.auto_faces):
            return "human"
        return "auto"

    def promises(self, contrast: float | None) -> bool:
        """この濃さの欠陥を見つけることを約束しているか（約束の濃さがなければ、すべて約束する）。"""
        if self.min_contrast is None:
            return True
        return contrast is not None and contrast >= self.min_contrast


VERSION_DEFAULTS = {"version": "", "auto_faces": [], "human_faces": [], "min_contrast": None, "basis": ""}


def route_by_scope(record: dict, scope: UsageScope) -> dict:
    """セルが台帳に書く前に通す: 使用範囲書で人に回す面なら保留にし、AI の判定は影（ai_verdict）として残す。

    範囲外（照明など）はもともと保留なのでそのまま。影の判定は請求には使わないが、範囲を広げる根拠を貯めるのに使う。
    """
    if not record["in_range"] or scope.face_mode(record["part"], record["face"]) == "auto":
        return record
    return {**record, "verdict": "HOLD", "reason": "scope:face",
            "ai_verdict": record["verdict"], "ai_reason": record["reason"]}


# ---- 検査セルの側 ----
def start_cell_ledger(identity: RobotIdentity, scope: UsageScope, t: float | None = None) -> Ledger:
    """先頭の行（seq 0）に、どの使用範囲書で働くかを書いた台帳を作る。"""
    if identity.robot_id != scope.cell_id:
        raise ValueError("使用範囲書のセルの ID と、鍵の ID が違う")
    led = Ledger()
    head = {"scope": scope.fingerprint(), "thresholds": scope.thresholds}
    if scope.version:
        head["version"] = scope.version  # 読む人のための目印。約束を結びつけるのは指紋
    led.append(identity, EV_SCOPE, head, t=t)
    return led


def record_inspection(ledger: Ledger, identity: RobotIdentity, record: dict, t: float | None = None) -> Entry:
    return ledger.append(identity, EV_INSPECTION, record, t=t)


# ---- 工場（受け取る側）----
class Factory:
    """工場の台帳を書く側。受け取る前に、信頼するセルの鍵でセルの台帳を確かめる。"""

    def __init__(self, identity: RobotIdentity, trusted_cell_key: str, ledger: Ledger | None = None):
        self.identity = identity
        self.trusted_cell_key = trusted_cell_key
        self.ledger = ledger or Ledger()

    def received_upto(self, cell_id: str, scope_fp: str) -> int:
        """このセル・この使用範囲書で、どの seq まで受け取ったか（まだなら 0）。"""
        upto = 0
        for e in self.ledger.entries:
            if e.event == EV_RECEIPT and e.data["cell_id"] == cell_id and e.data["scope"] == scope_fp:
                upto = max(upto, e.data["to_seq"])
        return upto

    def receive(self, cell_ledger: Ledger, scope: UsageScope, upto_seq: int, t: float | None = None) -> Entry:
        """セルの台帳の、前に受け取った次の行から upto_seq までを1つのまとまりとして受け取り、署名する。

        署名する中身に upto_seq の行のハッシュ（head）を入れる。ハッシュの鎖なので、head はそこまでの全部の行を表す。
        """
        problems = cell_ledger.verify(trusted_public_key=self.trusted_cell_key)
        if problems:
            raise LedgerInvalid(["検査セルの台帳: " + p for p in problems])
        first = cell_ledger.entries[0] if cell_ledger.entries else None
        if first is None or first.event != EV_SCOPE or first.data.get("scope") != scope.fingerprint():
            raise LedgerInvalid(["検査セルの台帳の先頭に、この使用範囲書の指紋がない"])
        from_seq = self.received_upto(scope.cell_id, scope.fingerprint()) + 1
        if not from_seq <= upto_seq < len(cell_ledger.entries):
            raise ValueError(f"受け取る範囲がおかしい: seq {from_seq}〜{upto_seq}（台帳は {len(cell_ledger.entries)} 行）")
        batch = cell_ledger.entries[from_seq:upto_seq + 1]
        data = {"cell_id": scope.cell_id, "scope": scope.fingerprint(), "from_seq": from_seq, "to_seq": upto_seq,
                "head": cell_ledger.entries[upto_seq].hash,
                "inspections": sum(1 for e in batch if e.event == EV_INSPECTION)}
        return self.ledger.append(self.identity, EV_RECEIPT, data, t=t)

    def claim(self, cell_ledger: Ledger, scope: UsageScope, seq: int, kind: str, note: str = "",
              t: float | None = None, contrast: float | None = None) -> Entry:
        """後から見つかった見逃し（miss）・過検出（overkill）を、セルの台帳の行を指して申し立てる。

        contrast: 工場が測った欠陥の濃さ（0〜1、良品の平均からの明るさのずれの最大）。使用範囲書に約束の濃さがあれば要る。
        """
        if kind not in CLAIM_KINDS:
            raise ValueError(f"申し立ての種類は {tuple(CLAIM_KINDS)} のどれか: {kind}")
        e = cell_ledger.entries[seq]
        data = {"cell_id": scope.cell_id, "scope": scope.fingerprint(), "seq": seq, "entry_hash": e.hash,
                "kind": kind, "note": note}
        if contrast is not None:
            data["contrast"] = round(float(contrast), 4)
        return self.ledger.append(self.identity, EV_CLAIM, data, t=t)


# ---- 請求 ----
STATUS_JA = {
    "counted": "数える（範囲内で AI が自分で決めた）",
    "hold": "数えない: 保留（人に回した）",
    "out_of_range": "数えない: 範囲外（人に回した）",
    "face": "数えない: 使用範囲書で人に回す面",
    "thresholds": "数えない: しきい値の版が使用範囲書と違う",
    "part": "数えない: 使用範囲書にない部品",
    "not_received": "数えない: 工場の受け取りの署名がない",
}


@dataclass
class BillLine:
    seq: int
    part: str
    face: str
    verdict: str
    status: str          # STATUS_JA の鍵
    refund: int = 0
    note: str = ""


@dataclass
class InspectionBill:
    scope: str                                   # 使用範囲書の指紋
    lines: list[BillLine] = field(default_factory=list)
    gross: int = 0                               # 数えた件数 × 料金
    refund_total: int = 0                        # 申し立てで減らした額
    claims_accepted: list[str] = field(default_factory=list)
    claims_rejected: list[str] = field(default_factory=list)
    rejected_by: dict[str, int] = field(default_factory=dict)  # 認めなかった理由 → 件数（REJECT_JA の鍵）

    @property
    def amount(self) -> int:
        """差し引きの請求額（円）。減額が上回ればマイナス（返す額）。"""
        return self.gross - self.refund_total

    def count(self, status: str, verdict: str | None = None) -> int:
        return sum(1 for ln in self.lines if ln.status == status and (verdict is None or ln.verdict == verdict))

    def summary(self) -> dict:
        return {"inspections": len(self.lines), "counted": self.count("counted"),
                "counted_ok": self.count("counted", "OK"), "counted_ng": self.count("counted", "NG"),
                **{k: self.count(k) for k in STATUS_JA if k != "counted"},
                "gross": self.gross, "refund_total": self.refund_total, "amount": self.amount,
                "claims_accepted": len(self.claims_accepted), "claims_rejected": len(self.claims_rejected),
                "refund_miss": sum(1 for ln in self.lines if ln.note == "miss"),
                "refund_overkill": sum(1 for ln in self.lines if ln.note == "overkill"),
                "rejected_by": dict(self.rejected_by)}


REJECT_JA = {"not_found": "指している行がない", "not_counted": "数えていない行（人が見た）", "kind": "判定と種類が合わない",
             "promise": "約束の濃さより薄い", "window": "期限切れ", "duplicate": "2回目"}


class ReceiptMismatch(LedgerInvalid):
    """工場が受け取りの署名をした台帳と、いま出された台帳が違う（受け取ったあとで作り直された）。"""


def make_inspection_bill(cell_ledger: Ledger, factory_ledger: Ledger, scope: UsageScope,
                         keys: TrustedKeys) -> InspectionBill:
    """2つの台帳を信頼する鍵で確かめ、使用範囲書の条件で数えて、申し立ての分を引く。"""
    problems = ["検査セルの台帳: " + p for p in cell_ledger.verify(trusted_public_key=keys.cell)]
    problems += ["工場の台帳: " + p for p in factory_ledger.verify(trusted_public_key=keys.factory)]
    if problems:
        raise LedgerInvalid(problems)
    if keys.cell_id != scope.cell_id:
        raise LedgerInvalid(["使用範囲書のセルと、信頼する鍵の一覧のセルが違う"])
    fp = scope.fingerprint()
    first = cell_ledger.entries[0] if cell_ledger.entries else None
    if first is None or first.event != EV_SCOPE or first.data.get("scope") != fp:
        raise LedgerInvalid(["検査セルの台帳の先頭に、この使用範囲書の指紋がない"])

    # 受け取りの署名: 署名に入れた head が、いまの台帳のその行のハッシュと合うか
    received: dict[int, float] = {}  # seq → 受け取った時刻
    mismatch = []
    for r in factory_ledger.entries:
        if r.event != EV_RECEIPT or r.data.get("cell_id") != scope.cell_id or r.data.get("scope") != fp:
            continue
        a, b = r.data["from_seq"], r.data["to_seq"]
        if b >= len(cell_ledger.entries) or cell_ledger.entries[b].hash != r.data["head"]:
            mismatch.append(f"工場の受け取り（seq {a}〜{b}、工場の台帳の seq {r.seq}）と、いまの検査セルの台帳が合わない")
            continue
        for s in range(a, b + 1):
            received.setdefault(s, r.time)
    if mismatch:
        raise ReceiptMismatch(mismatch)

    bill = InspectionBill(scope=fp)
    by_seq: dict[int, BillLine] = {}
    for e in cell_ledger.entries:
        if e.event != EV_INSPECTION:
            continue
        d = e.data
        if not d.get("in_range", False):
            status = "out_of_range"
        elif scope.face_mode(d["part"], d["face"]) == "human":
            status = "face"  # セルが保留にしていなくても数えない（人に回す約束の面）
        elif d["verdict"] == "HOLD":
            status = "hold"
        elif d.get("thresholds") != scope.thresholds:
            status = "thresholds"
        elif scope.parts and d["part"] not in scope.parts:
            status = "part"
        elif e.seq not in received:
            status = "not_received"
        else:
            status = "counted"
        line = BillLine(e.seq, d["part"], d["face"], d["verdict"], status)
        bill.lines.append(line)
        by_seq[e.seq] = line
    bill.gross = bill.count("counted") * scope.price_per_inspection

    # 申し立て: 数えた行だけ。判定と種類が合い、期限の中で、1行に1回まで
    done: set[int] = set()
    refund = {"miss": scope.refund_per_miss, "overkill": scope.refund_per_overkill}
    for c in factory_ledger.entries:
        if c.event != EV_CLAIM or c.data.get("cell_id") != scope.cell_id or c.data.get("scope") != fp:
            continue
        seq, kind = c.data["seq"], c.data["kind"]
        tag = f"工場の seq {c.seq}: 検査の seq {seq} を {kind}"
        line = by_seq.get(seq)
        con = c.data.get("contrast")
        why = None
        if line is None or seq >= len(cell_ledger.entries) or cell_ledger.entries[seq].hash != c.data["entry_hash"]:
            why = "not_found", "指している検査の行が見つからない・中身が違う"
        elif line.status != "counted":
            why = "not_counted", f"数えていない行（{STATUS_JA[line.status]}）。人が見た分は AI の責任にしない"
        elif CLAIM_KINDS.get(kind) != line.verdict:
            why = "kind", f"判定が {line.verdict} の行には {kind} を申し立てられない"
        elif kind == "miss" and not scope.promises(con):
            why = "promise", ((f"濃さ {con:.0%}" if con is not None else "濃さの測りがなく")
                              + f"、約束（濃さ {scope.min_contrast:.0%} 以上）の外。薄い欠陥の見逃しは減額しない")
        elif c.time - received[seq] > scope.claim_window_s:
            why = "window", "申し立ての期限を過ぎている"
        elif seq in done:
            why = "duplicate", "同じ行への2回目の申し立て"
        if why is not None:
            bill.claims_rejected.append(tag + " → " + why[1])
            bill.rejected_by[why[0]] = bill.rejected_by.get(why[0], 0) + 1
        else:
            done.add(seq)
            line.refund = refund[kind]
            line.note = kind
            bill.refund_total += refund[kind]
            bill.claims_accepted.append(tag + f" → {refund[kind]} 円を減らす")
    return bill

