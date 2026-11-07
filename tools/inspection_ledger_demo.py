"""検査の台帳の通しの見本: 双腕の検査セル（ロボット × AI）の判定を、署名つきの台帳にして請求額を出す。

    pixi run inspection_ledger_demo

やること（お金・登録・ネットワークは使わない。出力は ledger/inspection_demo/ で、Git には入れない）:
  1. 検査1件 = 台帳の1行。Isaac Sim で撮った 162 枚の判定（isaac/out/judge.json）と、疑似画像 200 枚の判定を、
     検査セルの鍵で署名して台帳にする（画像そのものは入れず SHA-256 だけ）
  2. 工場が 50 件ずつのまとまりに受け取りの署名をする。最後のまとまりは「まだ届いていない」として署名しない
  3. 使用範囲書で数える: 範囲内で AI が自分で決めた分（OK／NG）だけ。保留・範囲外・受け取り待ちは数えない
  4. 後から見逃し・過検出がわかった分を、工場が申し立てて減らす（見本では、正解の付いたシミュレーションの答えで
     「後から見つかった」ことにする。正解はセルの台帳には入れない）
  5. 改ざんを見抜けるか: 鍵なしで1行を書き換える／セルが自分の鍵で台帳を丸ごと作り直す
  6. peaq の記録の形（Verifiable Storage の封筒・peaqOS のイベント）に書き出し、公式の確かめ方で確かめる
"""
from __future__ import annotations

import copy
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot_ledger"))
sys.path.insert(0, str(ROOT / "src" / "dual_arm_inspection"))
from robot_ledger import peaq_format as P  # noqa: E402
from robot_ledger.billing import LedgerInvalid  # noqa: E402
from robot_ledger.inspection_ledger import (  # noqa: E402
    STATUS_JA, Factory, ReceiptMismatch, TrustedKeys, UsageScope, inspection_record, make_inspection_bill,
    record_inspection, start_cell_ledger, thresholds_version,
)
from robot_ledger.ledger import Ledger, RobotIdentity  # noqa: E402

OUT = ROOT / "ledger" / "inspection_demo"
ISAAC = ROOT / "src" / "dual_arm_inspection" / "isaac" / "out"
BATCH = 50          # 工場が1回に受け取る件数
CYCLE_S = 6.0       # 1件の検査にかかる時間（見本の時刻を作るため）
PRICE, REFUND_MISS, REFUND_OVERKILL = 100, 1000, 100  # 円（見本の値。お金は動かさない）


def isaac_rows():
    doc = json.loads((ISAAC / "judge.json").read_text(encoding="utf-8"))
    rows = []
    for r in doc["rows"]:
        img = ISAAC / "test" / r["file"]
        rows.append({"part": r["part"], "face": r["face"], "verdict": r["verdict"], "score": r["score"],
                     "reason": r["reason"], "defect_type": r["pred_type"], "shot": r["file"],
                     "image_sha256": hashlib.sha256(img.read_bytes()).hexdigest(), "truth": r["truth"]})
    return thresholds_version(doc["calibration"]), rows


def synthetic_rows(n=200):
    import cv2

    from dual_arm_inspection import cell as C
    from dual_arm_inspection import inspection as I

    parts, _ = C.load_parts()
    ins = I.Inspector(parts)
    cal = ins.calibrate()
    rows = []
    for r in I.evaluate(ins, n=n):
        ok, png = cv2.imencode(".png", cv2.cvtColor(r["img"], cv2.COLOR_RGB2BGR))
        rows.append({"part": r["part"], "face": r["face"], "verdict": r["verdict"], "score": r["score"],
                     "reason": r["reason"], "defect_type": r["pred_type"], "shot": r["i"],
                     "image_sha256": hashlib.sha256(png.tobytes()).hexdigest(), "truth": r["truth"]})
    return thresholds_version(cal), rows


def run_source(name, label, thresholds, rows, cell, factory_id, keys):
    """1つの撮り方（Isaac Sim／疑似画像）について、台帳 → 受け取り → 申し立て → 請求 → peaq の形、まで。"""
    scope = UsageScope(cell.robot_id, thresholds, PRICE, REFUND_MISS, REFUND_OVERKILL,
                       claim_window_s=30 * 24 * 3600.0, name=f"双腕の外観検査セル / {label}")
    t0 = time.time() - (len(rows) + 60) * CYCLE_S
    led = start_cell_ledger(cell, scope, t=t0)
    truth = {}
    for i, r in enumerate(rows, start=1):
        rec = inspection_record(r["part"], r["face"], r["verdict"], r["score"], r["reason"], thresholds,
                                r["image_sha256"], defect_type=r["defect_type"], shot=r["shot"])
        e = record_inspection(led, cell, rec, t=t0 + i * CYCLE_S)
        truth[e.seq] = r["truth"]  # 正解は台帳に入れない（後から見つかる見逃しを作るためだけに使う）

    factory = Factory(factory_id, keys.cell)
    last = len(led.entries) - 1
    ends = list(range(BATCH, last + 1, BATCH))
    if not ends or ends[-1] != last:
        ends.append(last)
    for end in ends[:-1]:  # 最後のまとまりは、まだ届いていない
        factory.receive(led, scope, end, t=led.entries[end].time + 30.0)
    waiting = (ends[-2] + 1 if len(ends) > 1 else 1, last)

    pre = make_inspection_bill(led, factory.ledger, scope, keys)
    t_claim = led.entries[-1].time + 10.0
    for ln in pre.lines:  # 後の工程で見つかった、ということにする
        if ln.status != "counted":
            continue
        if ln.verdict == "OK" and truth[ln.seq] == "defect":
            factory.claim(led, scope, ln.seq, "miss", "後の工程で不良が見つかった", t=t_claim)
        elif ln.verdict == "NG" and truth[ln.seq] == "good":
            factory.claim(led, scope, ln.seq, "overkill", "弾いた部品を人が見たら良品だった", t=t_claim)
    bill = make_inspection_bill(led, factory.ledger, scope, keys)

    led.save(OUT / f"{name}_cell.jsonl")
    factory.ledger.save(OUT / f"{name}_factory.jsonl")
    (OUT / f"{name}_scope.json").write_text(json.dumps(scope.to_dict(), ensure_ascii=False, indent=1), encoding="utf-8")
    (OUT / f"{name}_bill.json").write_text(json.dumps(
        {"summary": bill.summary(), "claims_accepted": bill.claims_accepted, "claims_rejected": bill.claims_rejected,
         "lines": [ln.__dict__ for ln in bill.lines]}, ensure_ascii=False, indent=1), encoding="utf-8")

    # 保存したファイルから読み直しても同じ請求になるか
    again = make_inspection_bill(Ledger.load(OUT / f"{name}_cell.jsonl"), Ledger.load(OUT / f"{name}_factory.jsonl"),
                                 scope, keys)
    assert again.summary() == bill.summary()

    # 最後のまとまりも受け取ったら、いくら増えるか（受け取りの署名がない分は数えていないことの確かめ）
    f2 = Factory(factory_id, keys.cell, Ledger(copy.deepcopy(factory.ledger.entries)))
    f2.receive(led, scope, last, t=t_claim + 1.0)
    after = make_inspection_bill(led, f2.ledger, scope, keys)

    s = bill.summary()
    print(f"\n== {label}: {len(rows)} 件（しきい値の版 {thresholds}、使用範囲書の指紋 {scope.fingerprint()[:16]}…）")
    for k in STATUS_JA:
        print(f"  {STATUS_JA[k]:<36} {bill.count(k):>4} 件")
    print(f"  受け取りの署名: {len(ends) - 1} まとまり（seq 1〜{ends[-2] if len(ends) > 1 else 0}）。"
          f"seq {waiting[0]}〜{waiting[1]} は受け取り待ち")
    print(f"  数えた {s['counted']} 件（OK {s['counted_ok']}・NG {s['counted_ng']}）× {PRICE} 円 = {s['gross']} 円")
    for c in bill.claims_accepted:
        print("  申し立て（認めた）: " + c)
    for c in bill.claims_rejected:
        print("  申し立て（認めない）: " + c)
    print(f"  減額 {s['refund_total']} 円 → 差し引き {s['amount']} 円")
    print(f"  最後のまとまりも受け取ると: 数える {after.count('counted')} 件（+{after.count('counted') - s['counted']}）")
    return {"name": name, "label": label, "scope": scope, "led": led, "factory": factory, "bill": bill,
            "summary": s, "waiting": waiting, "received_batches": len(ends) - 1,
            "after_last_receipt": after.count("counted")}


def tamper_checks(res, cell, keys):
    """改ざんを見抜けるか。見抜けたら True。"""
    name, scope, factory = res["name"], res["scope"], res["factory"]
    led_path = OUT / f"{name}_cell.jsonl"
    led = Ledger.load(led_path)
    target = next(e for e in led.entries if e.event == "inspection" and e.data["verdict"] == "HOLD" and e.data["in_range"])
    print(f"\n== 改ざんの見抜き（{res['label']} の台帳、seq {target.seq} の保留を OK にして請求を増やそうとする）")
    ok = True

    # A. 鍵を持たない人が、ファイルの1行だけを書き換える
    lines = led_path.read_text(encoding="utf-8").splitlines()
    row = json.loads(lines[target.seq])
    row["data"]["verdict"], row["data"]["reason"] = "OK", "clean"
    lines[target.seq] = json.dumps(row, ensure_ascii=False)
    forged_path = OUT / f"{name}_cell_tampered.jsonl"
    forged_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    try:
        make_inspection_bill(Ledger.load(forged_path), factory.ledger, scope, keys)
        print("  A. 1行の書き換え → 見抜けなかった")
        ok = False
    except LedgerInvalid as e:
        print(f"  A. 1行の書き換え → 請求を拒否: {e.problems[0]}")

    # B. 検査セル自身が、自分の鍵で台帳を丸ごと作り直す（署名もハッシュの鎖も「正しい」）
    rebuilt = start_cell_ledger(cell, scope, t=led.entries[0].time)
    for e in led.entries[1:]:
        data = dict(e.data, verdict="OK", reason="clean") if e.seq == target.seq else e.data
        record_inspection(rebuilt, cell, data, t=e.time)
    self_ok = rebuilt.verify(trusted_public_key=keys.cell) == []
    try:
        make_inspection_bill(rebuilt, factory.ledger, scope, keys)
        print("  B. セルの鍵で作り直し → 見抜けなかった")
        ok = False
    except ReceiptMismatch as e:
        print(f"  B. セルの鍵で作り直し → セルの鍵の確認は{'通る' if self_ok else '通らない'}が、"
              f"工場の受け取りと合わず請求を拒否: {e.problems[0]}")
    return ok


def peaq_export(res, cell, keys):
    """peaq の記録の形に書き出して、公式の確かめ方で確かめる。全部通れば True。"""
    name, led, bill = res["name"], res["led"], res["bill"]
    out = OUT / "peaq"
    out.mkdir(parents=True, exist_ok=True)
    envs = [P.entry_envelope(e, cell) for e in led.entries]
    with open(out / f"{name}_envelopes.jsonl", "w", encoding="utf-8") as f:
        for env in envs:
            f.write(P.dumps(env) + "\n")
    # 台帳のファイル全体の封筒（チェーンに刻むなら、この封筒の CID を {"envelopeCid": ...} で置く）
    ledger_bytes = (OUT / f"{name}_cell.jsonl").read_bytes()
    whole = P.make_envelope(ledger_bytes, cell, P.created_at_iso(led.entries[-1].time))
    onchain = {"envelopeCid": P.cid_v1_raw(P.canonical(whole)), "note": "仮。IPFS にもチェーンにも送っていない"}
    (out / f"{name}_ledger_envelope.json").write_text(
        json.dumps({"envelope": whole, "onchain_record": onchain}, ensure_ascii=False, indent=1), encoding="utf-8")

    counted = {ln.seq for ln in bill.lines if ln.status == "counted"}
    events = [P.inspection_event(e, e.seq in counted) for e in led.entries if e.event == "inspection"]
    events.append(P.bill_event(res["summary"], res["scope"].fingerprint(), led.head, res["factory"].ledger.head,
                               t=time.time() - 1))
    (out / f"{name}_events.json").write_text(json.dumps(events, ensure_ascii=False), encoding="utf-8")

    # 確かめる: 保存したファイルから読み直して、公式の手順どおりに
    saved = Ledger.load(OUT / f"{name}_cell.jsonl")
    with open(out / f"{name}_envelopes.jsonl", encoding="utf-8") as f:
        loaded = [json.loads(line) for line in f]
    env_bad = sum(1 for e, env in zip(saved.entries, loaded) if P.verify_envelope(env, P.entry_bytes(e), keys.cell))
    whole_bad = P.verify_envelope(whole, ledger_bytes, keys.cell)
    now = int(time.time())
    ev_bad = sum(1 for ev in events if P.validate_event(ev, now=now)[0])
    meta_max = max(len(ev["metadata_hex"]) // 2 - 1 for ev in events)
    # 書き換えた1行は、封筒との照らし合わせでも見抜ける
    tampered = Ledger.load(OUT / f"{name}_cell_tampered.jsonl")
    caught = sum(1 for e, env in zip(tampered.entries, loaded) if P.verify_envelope(env, P.entry_bytes(e), keys.cell))
    print(f"\n== peaq の形（{res['label']}）: 封筒 {len(envs)} 個（確かめて合わない {env_bad}）、"
          f"台帳全体の封筒 {'OK' if not whole_bad else whole_bad}、"
          f"イベント {len(events)} 個（活動 {len(events) - 1}・売上 1、決まりに合わない {ev_bad}、metadata 最大 {meta_max} バイト）")
    print(f"   書き換えた台帳を封筒と照らすと、合わない行 {caught} 行")
    return env_bad == 0 and not whole_bad and ev_bad == 0 and caught == 1, {
        "envelopes": len(envs), "envelopes_bad": env_bad, "events": len(events), "events_bad": ev_bad,
        "tampered_lines_caught_by_envelope": caught}


def main() -> int:
    if OUT.exists():
        for p in OUT.glob("*.jsonl"):
            p.unlink()
        shutil.rmtree(OUT / "peaq", ignore_errors=True)
    OUT.mkdir(parents=True, exist_ok=True)
    cell = RobotIdentity.load_or_create(OUT / "keys" / "cell.key")         # 秘密鍵: Git に入れない（ledger/ と *.key）
    factory_id = RobotIdentity.load_or_create(OUT / "keys" / "factory.key")
    keys = TrustedKeys(cell.public_key_hex, factory_id.public_key_hex)
    keys.save(OUT / "trusted_keys.json")
    print(f"検査セル {keys.cell_id} / 工場 {keys.factory_id}（信頼する鍵の一覧: {OUT / 'trusted_keys.json'}）")

    sources = []
    if (ISAAC / "judge.json").exists():
        sources.append(("isaac", "Isaac Sim の画像", *isaac_rows()))
    else:
        print("Isaac Sim の判定（isaac/out/judge.json）がないので飛ばす（pixi run isaac_judge で作る）")
    print("疑似画像を撮って判定しています（良品で学ぶ → 200 枚）…")
    sources.append(("synthetic", "疑似画像", *synthetic_rows()))

    all_ok = True
    report = []
    for name, label, th, rows in sources:
        res = run_source(name, label, th, rows, cell, factory_id, keys)
        tamper_ok = tamper_checks(res, cell, keys)
        peaq_ok, peaq_stats = peaq_export(res, cell, keys)
        all_ok &= tamper_ok and peaq_ok
        report.append({"name": name, "label": label, "summary": res["summary"], "waiting": res["waiting"],
                       "received_batches": res["received_batches"], "after_last_receipt": res["after_last_receipt"],
                       "tamper_detected": tamper_ok, "peaq": peaq_stats,
                       "cell_head": res["led"].head, "factory_head": res["factory"].ledger.head})
    (OUT / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

    print("\n| 撮り方 | 全件 | 範囲内で AI が決めた（OK／NG） | 保留（人へ） | 範囲外（人へ） | 受け取り待ち | 請求した件数 | 小計 | 見逃し・過検出の減額 | 差し引き |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in report:
        s = r["summary"]
        auto = s["counted"] + s["not_received"] + s["thresholds"] + s["part"]
        print(f"| {r['label']} | {s['inspections']} | {auto} | {s['hold']} | {s['out_of_range']} | {s['not_received']} | "
              f"{s['counted']}（OK {s['counted_ok']}・NG {s['counted_ng']}） | {s['gross']:,} 円 | "
              f"{s['refund_total']:,} 円（{s['claims_accepted']} 件） | {s['amount']:,} 円 |")
    print(f"\n{'OK' if all_ok else 'NG'}: 改ざんの見抜きと peaq の形の確かめ（出力 {OUT}）")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
