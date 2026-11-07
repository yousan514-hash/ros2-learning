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
  7. 1〜6 を、使用範囲書の 2 つの版（v1: 今の使用範囲書／v2: 瞳の案）で回し、全部受け取ったときの請求・減額・
     人に回った件数を表にして比べる。見逃しの濃さは瞳の物差し（isaac/out/miss_curve.json）を「工場が測った値」とする
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
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
    REJECT_JA, STATUS_JA, Factory, ReceiptMismatch, TrustedKeys, UsageScope, inspection_record,
    make_inspection_bill, record_inspection, route_by_scope, start_cell_ledger, thresholds_version,
)
from robot_ledger.ledger import Ledger, RobotIdentity  # noqa: E402

OUT = ROOT / "ledger" / "inspection_demo"
ISAAC = ROOT / "src" / "dual_arm_inspection" / "isaac" / "out"
BATCH = 50          # 工場が1回に受け取る件数
CYCLE_S = 6.0       # 1件の検査にかかる時間（見本の時刻を作るため）
PRICE, REFUND_MISS, REFUND_OVERKILL = 100, 1000, 100  # 円（見本の値。お金は動かさない）
HUMAN = ("hold", "out_of_range", "face")  # 人に回った行の status

# 使用範囲書の版。v1 は版の欄を書かない（前回のデモと同じ指紋になる）。v2 は瞳の案
SCOPE_VERSIONS = [
    ("v1", "今の使用範囲書（すべての面・濃さの約束なし）", {}),
    ("v2", "瞳の案（3 つの面だけ AI・濃さ 20% 以上を約束）", {
        "version": "v2",
        # トレイの上から撮る面・ボルトの頭の面・ナットと座金の下の面は AI に任せる
        "auto_faces": ["bolt_*/side_top", "nut_*/top", "washer_*/top", "bolt_*/head_end", "nut_*/bottom",
                       "washer_*/bottom"],
        # 写り込みのあるボルトの先端の面・ナットの横の面と、手で持って撮るボルトの横の面は人に回す
        "human_faces": ["bolt_*/tip_end", "nut_*/side", "bolt_*/side_bottom"],
        "min_contrast": 0.2,
        "basis": "瞳の使用範囲書の案（src/dual_arm_inspection/isaac/README.md の「使用範囲の案」、コミット 245c91d）。"
                 "Isaac Sim 162 枚で、範囲の中の見逃し 0/31・過検出 1/72、濃さ 20% 以上の見逃し 0/45。自作の仮定で、枚数が少ない"}),
]


def _num(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else float(x)


def load_contrast():
    """瞳の物差しの「濃さ」（後から工場が測った値の代わりに使う）。isaac/out/miss_curve.json がなければ空。"""
    path = ISAAC / "miss_curve.json"
    if not path.exists():
        print("見逃しの濃さ（isaac/out/miss_curve.json）がない: 濃さを約束する版の見逃しの申し立ては、測りがないので認めない"
              "（pixi run python src/dual_arm_inspection/isaac/analyze_miss_curve.py で作る）")
        return {}, []
    doc = json.loads(path.read_text(encoding="utf-8"))
    return {r["file"]: _num(r["contrast"]) for r in doc.get("isaac", [])}, doc.get("synthetic_ref", [])


def isaac_rows(contrast):
    doc = json.loads((ISAAC / "judge.json").read_text(encoding="utf-8"))
    rows = []
    for r in doc["rows"]:
        img = ISAAC / "test" / r["file"]
        rows.append({"part": r["part"], "face": r["face"], "verdict": r["verdict"], "score": r["score"],
                     "reason": r["reason"], "defect_type": r["pred_type"], "shot": r["file"],
                     "image_sha256": hashlib.sha256(img.read_bytes()).hexdigest(), "truth": r["truth"],
                     "contrast": contrast.get(r["file"])})
    return thresholds_version(doc["calibration"]), rows


def synthetic_rows(ref, n=200):
    import cv2

    from dual_arm_inspection import cell as C
    from dual_arm_inspection import inspection as I

    parts, _ = C.load_parts()
    ins = I.Inspector(parts)
    cal = ins.calibrate()
    rows, mismatch = [], 0
    for r in I.evaluate(ins, n=n):
        ok, png = cv2.imencode(".png", cv2.cvtColor(r["img"], cv2.COLOR_RGB2BGR))
        # miss_curve.json の synthetic_ref は同じ乱数（seed=7・n=200）の同じ画像。念のため部品・面・判定で照らす
        m = ref[r["i"]] if r["i"] < len(ref) else None
        same = m is not None and (m["part"], m["face"], m["verdict"]) == (r["part"], r["face"], r["verdict"])
        mismatch += m is not None and not same
        rows.append({"part": r["part"], "face": r["face"], "verdict": r["verdict"], "score": r["score"],
                     "reason": r["reason"], "defect_type": r["pred_type"], "shot": r["i"],
                     "image_sha256": hashlib.sha256(png.tobytes()).hexdigest(), "truth": r["truth"],
                     "contrast": _num(m["contrast"]) if same else None})
    if mismatch:
        print(f"  疑似画像の濃さ: {mismatch} 枚が miss_curve.json と合わないので、濃さなしにした")
    return thresholds_version(cal), rows


def file_claims(factory, led, scope, bill, truth, contrast, t, skip=()):
    """後の工程で見つかった、ということにして申し立てる（正解は台帳に入れない。濃さは工場が測った値の代わり）。"""
    for ln in bill.lines:
        if ln.status != "counted" or ln.seq in skip:
            continue
        if ln.verdict == "OK" and truth[ln.seq] == "defect":
            factory.claim(led, scope, ln.seq, "miss", "後の工程で不良が見つかった", t=t, contrast=contrast[ln.seq])
        elif ln.verdict == "NG" and truth[ln.seq] == "good":
            factory.claim(led, scope, ln.seq, "overkill", "弾いた部品を人が見たら良品だった", t=t)


def run_source(name, label, thresholds, rows, cell, factory_id, keys, tag="v1", vlabel="", extra=None):
    """1つの撮り方（Isaac Sim／疑似画像）× 使用範囲書の1つの版について、台帳 → 受け取り → 申し立て → 請求、まで。"""
    scope = UsageScope(cell.robot_id, thresholds, PRICE, REFUND_MISS, REFUND_OVERKILL,
                       claim_window_s=30 * 24 * 3600.0, name=f"双腕の外観検査セル / {label}", **(extra or {}))
    key = f"{name}_{tag}"
    t0 = time.time() - (len(rows) + 60) * CYCLE_S
    led = start_cell_ledger(cell, scope, t=t0)
    truth, contrast = {}, {}
    for i, r in enumerate(rows, start=1):
        rec = inspection_record(r["part"], r["face"], r["verdict"], r["score"], r["reason"], thresholds,
                                r["image_sha256"], defect_type=r["defect_type"], shot=r["shot"])
        e = record_inspection(led, cell, route_by_scope(rec, scope), t=t0 + i * CYCLE_S)
        truth[e.seq] = r["truth"]  # 正解は台帳に入れない（後から見つかる見逃しを作るためだけに使う）
        contrast[e.seq] = r.get("contrast")

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
    file_claims(factory, led, scope, pre, truth, contrast, t_claim)
    bill = make_inspection_bill(led, factory.ledger, scope, keys)

    led.save(OUT / f"{key}_cell.jsonl")
    factory.ledger.save(OUT / f"{key}_factory.jsonl")
    (OUT / f"{key}_scope.json").write_text(json.dumps(scope.to_dict(), ensure_ascii=False, indent=1), encoding="utf-8")
    (OUT / f"{key}_bill.json").write_text(json.dumps(
        {"summary": bill.summary(), "claims_accepted": bill.claims_accepted, "claims_rejected": bill.claims_rejected,
         "lines": [ln.__dict__ for ln in bill.lines]}, ensure_ascii=False, indent=1), encoding="utf-8")

    # 保存したファイルから読み直しても同じ請求になるか（使用範囲書もファイルから読み、指紋を確かめる）
    scope_again = UsageScope.from_dict(json.loads((OUT / f"{key}_scope.json").read_text(encoding="utf-8")))
    again = make_inspection_bill(Ledger.load(OUT / f"{key}_cell.jsonl"), Ledger.load(OUT / f"{key}_factory.jsonl"),
                                 scope_again, keys)
    assert again.summary() == bill.summary()

    # 最後のまとまりも受け取ったら（受け取りの署名がない分は数えていないことの確かめ）。版どうしの比べはこちらで行う
    f2 = Factory(factory_id, keys.cell, Ledger(copy.deepcopy(factory.ledger.entries)))
    f2.receive(led, scope, last, t=t_claim + 1.0)
    mid = make_inspection_bill(led, f2.ledger, scope, keys)
    claimed = {c.data["seq"] for c in f2.ledger.entries if c.event == "claim"}
    file_claims(f2, led, scope, mid, truth, contrast, t_claim + 2.0, skip=claimed)
    full = make_inspection_bill(led, f2.ledger, scope, keys)

    # 人に回った分の中身（正解で数える）と、人に回した面での AI の影の判定
    human_lines = [ln for ln in full.lines if ln.status in HUMAN]
    shadow = [(led.entries[ln.seq].data.get("ai_verdict"), truth[ln.seq]) for ln in full.lines if ln.status == "face"]
    stats = {"human": len(human_lines), "human_defects": sum(truth[ln.seq] == "defect" for ln in human_lines),
             "shadow_ok": sum(v == "OK" for v, _ in shadow), "shadow_ng": sum(v == "NG" for v, _ in shadow),
             "shadow_miss": sum(v == "OK" and t == "defect" for v, t in shadow)}

    s = bill.summary()
    print(f"\n== {label} × {tag} {vlabel}: {len(rows)} 件（しきい値の版 {thresholds}、使用範囲書の指紋 {scope.fingerprint()[:16]}…）")
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
    print(f"  最後のまとまりも受け取ると: 数える {full.count('counted')} 件（+{full.count('counted') - s['counted']}）、"
          f"差し引き {full.amount} 円")
    if stats["shadow_ok"] + stats["shadow_ng"]:
        print(f"  人に回した面の、AI の影の判定（請求には使わない）: OK {stats['shadow_ok']}・NG {stats['shadow_ng']}。"
              f"AI に任せていたら見逃しになった不良 {stats['shadow_miss']} 件")
    return {"name": name, "key": key, "label": label, "tag": tag, "vlabel": vlabel, "scope": scope, "led": led,
            "factory": factory, "bill": bill, "summary": s, "waiting": waiting, "received_batches": len(ends) - 1,
            "after_last_receipt": full.count("counted"), "full": full.summary(), "stats": stats}


def tamper_checks(res, cell, keys):
    """改ざんを見抜けるか。見抜けたら True。"""
    name, scope, factory = res["key"], res["scope"], res["factory"]
    led_path = OUT / f"{name}_cell.jsonl"
    led = Ledger.load(led_path)
    target = next(e for e in led.entries if e.event == "inspection" and e.data["verdict"] == "HOLD" and e.data["in_range"])
    print(f"\n== 改ざんの見抜き（{res['label']} × {res['tag']} の台帳、seq {target.seq} の保留を OK にして請求を増やそうとする）")
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
    name, led, bill = res["key"], res["led"], res["bill"]
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
    print(f"\n== peaq の形（{res['label']} × {res['tag']}）: 封筒 {len(envs)} 個（確かめて合わない {env_bad}）、"
          f"台帳全体の封筒 {'OK' if not whole_bad else whole_bad}、"
          f"イベント {len(events)} 個（活動 {len(events) - 1}・売上 1、決まりに合わない {ev_bad}、metadata 最大 {meta_max} バイト）")
    print(f"   書き換えた台帳を封筒と照らすと、合わない行 {caught} 行")
    return env_bad == 0 and not whole_bad and ev_bad == 0 and caught == 1, {
        "envelopes": len(envs), "envelopes_bad": env_bad, "events": len(events), "events_bad": ev_bad,
        "tampered_lines_caught_by_envelope": caught}


def main() -> int:
    if OUT.exists():
        for pat in ("*.jsonl", "*_scope.json", "*_bill.json"):
            for p in OUT.glob(pat):
                p.unlink()
        shutil.rmtree(OUT / "peaq", ignore_errors=True)
    OUT.mkdir(parents=True, exist_ok=True)
    cell = RobotIdentity.load_or_create(OUT / "keys" / "cell.key")         # 秘密鍵: Git に入れない（ledger/ と *.key）
    factory_id = RobotIdentity.load_or_create(OUT / "keys" / "factory.key")
    keys = TrustedKeys(cell.public_key_hex, factory_id.public_key_hex)
    keys.save(OUT / "trusted_keys.json")
    print(f"検査セル {keys.cell_id} / 工場 {keys.factory_id}（信頼する鍵の一覧: {OUT / 'trusted_keys.json'}）")

    c_isaac, c_synth = load_contrast()
    sources = []
    if (ISAAC / "judge.json").exists():
        sources.append(("isaac", "Isaac Sim の画像", *isaac_rows(c_isaac)))
    else:
        print("Isaac Sim の判定（isaac/out/judge.json）がないので飛ばす（pixi run isaac_judge で作る）")
    print("疑似画像を撮って判定しています（良品で学ぶ → 200 枚）…")
    sources.append(("synthetic", "疑似画像", *synthetic_rows(c_synth)))

    all_ok = True
    report = []
    for name, label, th, rows in sources:
        for tag, vlabel, extra in SCOPE_VERSIONS:
            res = run_source(name, label, th, rows, cell, factory_id, keys, tag, vlabel, extra)
            tamper_ok = tamper_checks(res, cell, keys)
            peaq_ok, peaq_stats = peaq_export(res, cell, keys)
            all_ok &= tamper_ok and peaq_ok
            report.append({"name": name, "label": label, "version": tag, "version_label": vlabel,
                           "scope_fingerprint": res["scope"].fingerprint(), "summary": res["summary"],
                           "waiting": res["waiting"], "received_batches": res["received_batches"],
                           "after_last_receipt": res["after_last_receipt"], "full_receipt": res["full"],
                           "human": res["stats"], "tamper_detected": tamper_ok, "peaq": peaq_stats,
                           "cell_head": res["led"].head, "factory_head": res["factory"].ledger.head})
    (OUT / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

    print("\n最後のまとまりがまだ受け取り待ちのとき:")
    print("| 撮り方 | 使用範囲書 | 全件 | 範囲内で AI が決めた（OK／NG） | 保留（人へ） | 範囲外（人へ） | 人に回す面 | 受け取り待ち | 請求した件数 | 小計 | 見逃し・過検出の減額 | 差し引き |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in report:
        s = r["summary"]
        auto = s["counted"] + s["not_received"] + s["thresholds"] + s["part"]
        print(f"| {r['label']} | {r['version']} | {s['inspections']} | {auto} | {s['hold']} | {s['out_of_range']} | "
              f"{s['face']} | {s['not_received']} | {s['counted']}（OK {s['counted_ok']}・NG {s['counted_ng']}） | "
              f"{s['gross']:,} 円 | {s['refund_total']:,} 円（{s['claims_accepted']} 件） | {s['amount']:,} 円 |")

    print("\n使用範囲書の版の比べ（全部のまとまりを受け取り、後から見つかった見逃し・過検出をすべて申し立てたとき）:")
    print("| 撮り方 | 使用範囲書 | 請求の件数（AI が決めた） | 人に回った件数（保留・範囲外・面） | うち不良（人が見る） | 小計 | 減額（見逃し・過検出） | 認めない申し立て | 差し引き | 1 件あたり |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in report:
        f, h = r["full_receipt"], r["human"]
        rej = "・".join(f"{REJECT_JA[k]} {v}" for k, v in f["rejected_by"].items()) or "0"
        per = f["amount"] / f["counted"] if f["counted"] else 0.0
        print(f"| {r['label']} | {r['version']} | {f['counted']} / {f['inspections']} | "
              f"{h['human']}（{f['hold']}・{f['out_of_range']}・{f['face']}） | {h['human_defects']} | {f['gross']:,} 円 | "
              f"{f['refund_total']:,} 円（{f['refund_miss']}・{f['refund_overkill']}） | {rej} | {f['amount']:,} 円 | "
              f"{per:.1f} 円 |")
    print(f"\n{'OK' if all_ok else 'NG'}: 改ざんの見抜きと peaq の形の確かめ（出力 {OUT}）")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
