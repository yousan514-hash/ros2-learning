"""台帳から「働いた分だけ」の請求額を出す。

数えるのは、使用範囲（契約で決めた条件）の中で終わった仕事だけ。台帳が正しくなければ請求しない。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robot_ledger.ledger import Ledger


@dataclass
class Contract:
    robot_public_key: str  # 登録されたロボットの公開鍵。これ以外の署名は認めない
    price_per_task: int  # 1件あたりの料金（円）
    max_error: float  # 使用範囲: この誤差以内で終わった仕事だけを数える


@dataclass
class Bill:
    counted: int
    excluded: int
    amount: int
    lines: list[str] = field(default_factory=list)


class LedgerInvalid(Exception):
    def __init__(self, problems: list[str]):
        super().__init__("台帳が正しくない: " + "; ".join(problems))
        self.problems = problems


def make_bill(ledger: Ledger, contract: Contract) -> Bill:
    problems = ledger.verify(trusted_public_key=contract.robot_public_key)
    if problems:
        raise LedgerInvalid(problems)
    counted = excluded = 0
    lines = []
    for e in ledger.entries:
        if e.event != "task_done":
            continue
        err = float(e.data.get("error", math.inf))
        name = f"seq {e.seq} 点{e.data.get('point', '?')} 誤差 {err:.3f}"
        if err <= contract.max_error:
            counted += 1
            lines.append(f"{name} → 数える")
        else:
            excluded += 1
            lines.append(f"{name} → 使用範囲の外（数えない）")
    return Bill(counted, excluded, counted * contract.price_per_task, lines)
