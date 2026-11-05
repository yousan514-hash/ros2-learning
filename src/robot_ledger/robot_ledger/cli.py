"""台帳を確かめるコマンドと、請求額を出すコマンド。

    ros2 run robot_ledger ledger_verify ledger/turtle1.jsonl --pubkey-file ledger/turtle1.pub
    ros2 run robot_ledger ledger_bill ledger/turtle1.jsonl --pubkey-file ledger/turtle1.pub --price 100 --max-error 0.05
"""
import argparse
import sys
from pathlib import Path

from robot_ledger.billing import Contract, LedgerInvalid, make_bill
from robot_ledger.ledger import Ledger


def _pubkey(args) -> str | None:
    if args.pubkey:
        return args.pubkey
    if args.pubkey_file:
        return Path(args.pubkey_file).read_text(encoding="utf-8").strip()
    return None


def _common(ap):
    ap.add_argument("ledger", help="台帳のファイル（.jsonl）")
    ap.add_argument("--pubkey", help="登録されたロボットの公開鍵（16進）")
    ap.add_argument("--pubkey-file", help="公開鍵のファイル（.pub）")


def verify_main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="台帳が改ざんされていないかを確かめる")
    _common(ap)
    args = ap.parse_args(argv)
    led = Ledger.load(args.ledger)
    key = _pubkey(args)
    if not led.entries:
        print("台帳が空です")
        return 1
    print(f"{len(led.entries)} 行 / ロボット {led.entries[0].robot_id} / 最後のハッシュ {led.head[:16]}…")
    if key is None:
        print("注意: 公開鍵を渡していないので、別の鍵で作り直した偽の台帳は見抜けません")
    problems = led.verify(trusted_public_key=key)
    if problems:
        print("NG: 台帳が正しくありません")
        for p in problems:
            print("  - " + p)
        return 1
    print("OK: 署名もハッシュの鎖も正しい")
    return 0


def bill_main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="使用範囲の中で終わった仕事だけを数えて、請求額を出す")
    _common(ap)
    ap.add_argument("--price", type=int, default=100, help="1件あたりの料金（円）")
    ap.add_argument("--max-error", type=float, default=0.05, help="使用範囲: この誤差以内の仕事だけ数える")
    args = ap.parse_args(argv)
    key = _pubkey(args)
    if key is None:
        print("請求には、登録されたロボットの公開鍵が必要です（--pubkey か --pubkey-file）")
        return 2
    try:
        bill = make_bill(Ledger.load(args.ledger), Contract(key, args.price, args.max_error))
    except LedgerInvalid as e:
        print("請求できません: " + str(e))
        return 1
    for line in bill.lines:
        print("  " + line)
    print(f"数えた {bill.counted} 件 / 範囲の外 {bill.excluded} 件 / 請求 {bill.amount} 円（1件 {args.price} 円）")
    return 0


if __name__ == "__main__":
    sys.exit(verify_main())
