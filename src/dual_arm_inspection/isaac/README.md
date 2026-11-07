# 2 段目: 双腕セルを Isaac Sim 6.2 で撮って、同じ検査で判定する

> **状態: 2026-11-07 に動いた。** Windows 11・RTX 4070（12GB）・Isaac Sim 6.2.0 で、画面なし（headless）・480x360。
> 良品 810 枚と欠陥あり 162 枚を撮り、ROS 2 側と同じ検査（`inspection.py`、良品だけで学ぶ → OK／NG／保留）で判定した。

RViz で動かしているのと**同じ計画・同じ関節角**で、Isaac Sim の右手首カメラ（Replicator）から撮ります。

```
pixi（ROS 2 の環境）                      Isaac Sim 6.2 の Python（C:\isaacsim\python.bat）
──────────────────────                  ────────────────────────────────────────
export_urdf.py                           capture_inspection.py
  xacro → dual_arm.urdf（机などなし）  →    URDF → USD（初回だけ。2 回目からは使い回す）
  planner → inspection_plan.json      →    関節角を入れる・部品を置く・黒い背景板・欠陥を出す
                                           右手首のカメラ + リング照明 → Replicator で撮る
judge_folder.py / make_compare_figure.py ← out\good\ と out\test\（PNG + shots.json）
  Isaac の良品だけで学び直す → 表と絵（docs\isaac_vs_synthetic.png）
```

## 藤田さんが自分で動かすコマンド

```powershell
cd C:\Users\yousa\dev\ros2-learning
pixi run dual_arm_urdf
C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --no-ros-env --width 480 --height 360 --good-out src\dual_arm_inspection\isaac\out\good --good-repeat 30 --test-out src\dual_arm_inspection\isaac\out\test --test-repeat 6 --defects 0.4 --p-ood 0.05 --seed 2
pixi run isaac_figure
```

- 2 行目（撮る）: 良品 27 面 x 30 周 = 810 枚と、欠陥あり 27 面 x 6 周 = 162 枚を、1 回の起動で撮る
- 3 行目: Isaac Sim の良品だけで学び直して判定し（`isaac_judge`）、比べの絵を `docs\isaac_vs_synthetic.png` と `docs\isaac_shots.png` に作る
- まず試すだけなら: `... capture_inspection.py --no-ros-env --max-shots 3 --good-only --width 480 --height 360 --out src\dual_arm_inspection\isaac\out\try`
- `out\` は Git に入れない（`.gitignore` 済み）

## かかった時間と GPU のメモリ（2026-11-07）

| 場面 | 時間 | GPU のメモリの最大 |
|---|---|---|
| はじめての起動（シェーダーの準備、藤田さんの実行） | 約 135 秒で起動 | — |
| 2 回目からの起動 | 約 8 秒で起動 | — |
| 3 枚 | 13 秒（起動から終わりまで） | 3,020 MiB |
| 良品 810 枚 + 欠陥あり 162 枚 | 136 秒 | 3,077 MiB |
| 欠陥あり 162 枚だけ | 31 秒 | 3,038 MiB |

GPU のメモリはパソコン全体の値です（Isaac Sim を動かす前に、ほかのアプリで約 1.2GB 使っていた）。12GB の 4 分の 1 くらいで足りました。

## 結果（疑似画像との比べ）

| | 疑似画像（200 枚） | Isaac Sim（162 枚） |
|---|---|---|
| 良品 → OK／NG／保留 | 142 ／ 0 ／ 1 | 94 ／ 3 ／ 3 |
| 不良 → OK／NG／保留 | 1 ／ 49 ／ 0 | 3 ／ 32 ／ 21 |
| 範囲外 → 保留 | 7 ／ 7 | 6 ／ 6 |
| 見逃し | 2.0% | 5.4% |
| 過検出 | 0.0% | 3.0% |
| 人に回る（保留） | 4.0% | 18.5% |
| 自動で決めた分の正解率 | 99.5% | 95.5% |

- 自作の運動学で計算したカメラの位置と、Isaac Sim に読み込んだロボットのカメラの位置の差: 972 枚すべて 1mm 以下（最大 0.005mm）
- Isaac Sim の見逃し 3 枚は、どれも持って撮る横の面の細い傷。過検出 3 枚のうち 2 枚は、後ろの頭が写り込む先端の面と、ナットの横の面
- どちらも自作の仮定（欠陥の大きさ・照明）での比べです。実物では確かめていません

## 中身

| ファイル | すること |
|---|---|
| `export_urdf.py` | xacro → `out\dual_arm.urdf`（机などなし・コメントなし）と、撮影 27 回ぶんの関節角・姿勢（`out\inspection_plan.json`） |
| `capture_inspection.py` | Isaac Sim の Python で撮る。主なオプション: `--good-out/--good-repeat`、`--test-out/--test-repeat`、`--defects`（欠陥の確率）、`--p-ood`（範囲外の照明の確率）、`--fill`（部品が画面の高さの何割に写るか）、`--ring-intensity`、`--reimport` |
| `judge_folder.py` | Isaac Sim の良品だけで学び直して判定（`pixi run isaac_judge`） |
| `make_compare_figure.py` | 疑似画像と Isaac Sim の表と見本の絵（`pixi run isaac_figure`） |

撮り方の決めごと:
- 照明: カメラのまわりのリング照明（円盤の光）+ 弱いドームライト。強さは毎回 ±10% ばらつかせる。範囲外は x0.40〜0.55（暗い）か x1.6〜1.9（明るい）
- 背景: 持って撮る面の 6cm 後ろに黒い板、トレイも黒っぽくした（撮像の標準化。指も黒いので、部品を切り出しやすい）
- 倍率: 部品が画面の高さの 7 割に写るよう、撮るたびに焦点距離を変える（疑似カメラの「レシピごとの倍率」と同じ）
- 置き方のばらつき: 部品を軸のまわりに ±3 度、位置を ±0.5mm
- 欠陥: 打痕（暗い球、半径 0.6〜1.2mm）・傷（長さ 3〜8mm、幅 0.2mm）・汚れ（茶色の薄い円盤、半径 1.5〜3mm）を、材料のある所に置く
- 学ぶ単位は「部品 x 面」（トレイの上から撮る面と、持って撮る面は見え方が違うので分ける）。良品 30 枚のうち 18 枚で学び、12 枚でしきい値を決める

## つまずいたことと直し方

| 症状 | 原因 | 直し方 |
|---|---|---|
| 起動して 137 秒で何も出さずに終わった（Traceback もなし） | 例外のあと `finally: simulation_app.close()` が先に動き、`fastShutdown` でトレースバックを出す前にプロセスが終わった | 例外を `close()` の前に表示し、`error.txt` にも書く |
| 関節を動かせない | 変換した USD は物理を `Physics` の切り替え（variant）で持っていて、何も選ばないと物理なし | `add_reference_to_stage(..., variants=[("Physics", "physx")])` |
| 描画が 240x180 だった | 既定の DLSS が半分の解像度で描いて引き伸ばす | `SimulationApp({"renderer": "RaytracedLighting", "anti_aliasing": 2})`（FXAA） |
| 画像が灰色一色 | URDF のレンズの円柱が光学中心より 2cm 前に出ていて、カメラがレンズの中から写していた | xacro のレンズの位置を直した（RViz では気づかなかった） |
| 欠陥ありの組で、腕が動かなくなった | `rep.orchestrator.wait_until_complete()` がタイムラインを止め、物理が進まなくなった | 待つのは最後に 1 回だけ。念のため欠陥の形は最初に作っておき、位置と見える/見えないだけを変える |
| カメラの位置の差が 0.23m と出た | 関節角を入れた直後に読むと、まだ前の姿勢の値 | 描いたあとにも読む（こちらは全部 1mm 以下） |
| 見逃しが 37% になった | 欠陥を面の中心に置いていたので、ナット・座金では穴の中に浮き、ボルトの横では軸から浮いていた | 材料のある所に置くように直したら 5.4% |
| しきい値が数千になった | 全部の面で 1 つのしきい値だと、ばらつきの大きい面（先端の面）に引っ張られる | しきい値をレシピ（部品 x 面）ごとにした |
| 毎回 USD ができて `dual_arm_1` `dual_arm_2` … と増える | 変換のたびに別の名前で書く | URDF より新しい USD があれば使い回す（`--reimport` で作り直し） |

## このあと

- 先端の面・ナットの横の面のばらつきを減らす（後ろの頭を写さない画角、背景板の置き方）
- 部品の形を synthetic-testbench の `scripts/make_parts.py` の USD（面法線・材質つき）に置き換える
- 欠陥を「小さな形」ではなく、テクスチャや形の変形（打痕のくぼみ）にする
- ROS 2 ブリッジで、`cell_node` の `/joint_states` を Isaac Sim がそのまま受けて動く形にする
