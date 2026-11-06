# 2 段目（下書き）: 双腕セルを Isaac Sim 6.2 で撮る

> **状態: 下書き。まだ一度も動かしていません。** API の名前は `C:\isaacsim`（6.2.0）の `standalone_examples` と `skills/` を読んで合わせましたが、実行して確かめたのは pixi 側の `export_urdf.py` と `judge_folder.py`（疑似画像での pytest）だけです。

RViz で動かしているのと**同じ計画・同じ姿勢**で、Isaac Sim の右手首カメラ（Replicator）から検査の画像を撮り、ROS 2 側と同じ検査（`inspection.py`、良品だけで学ぶ → 3 段判定）で判定します。

```
pixi（ROS 2 の環境）                      Isaac Sim 6.2 の Python（C:\isaacsim\python.bat）
──────────────────────                  ────────────────────────────────────────
export_urdf.py                           capture_inspection.py
  xacro → dual_arm.urdf（机などなし）  →    URDF → USD（URDFImporter、根元固定）
  planner → inspection_plan.json      →    関節角を入れる・部品を置く・欠陥を置く
                                           右手首のカメラ + リング照明 → Replicator で撮る
judge_folder.py                      ←    out\good\ と out\test\（PNG + shots.json）
  Isaac の良品で学び直す → OK／NG／保留の表
```

## 0. 始める前に（藤田さんが自分でやること）

1. **ライセンス（EULA）への同意**: `C:\isaacsim\isaac-sim.bat` を一度起動し、表示されるライセンスを読んで、自分で同意してください。このスクリプトは `OMNI_KIT_ACCEPT_EULA` を設定しません（同意を代わりにしないため）。
2. **GPU のメモリ**: RTX 4070（12GB）は公式の最低要件（16GB）に届きません。このスクリプトは軽くしてあります（画面なし・640x480・当たり判定なし・単純な形）。ほかの GPU を使うアプリ（ブラウザの動画、ゲームなど）は閉じておくと安心です。
3. 初回は、シェーダーの準備で数分〜十数分かかることがあります（止まって見えても待つ）。

## 1. Isaac Sim に渡すファイルを作る（pixi の環境）

```powershell
cd C:\Users\yousa\dev\ros2-learning
pixi run dual_arm_urdf
# → src\dual_arm_inspection\isaac\out\dual_arm.urdf          （机・トレイ・照明を外したロボットだけ、コメントなし）
# → src\dual_arm_inspection\isaac\out\inspection_plan.json   （撮影 27 回ぶんの関節角・部品とカメラの姿勢・面の中心と向き）
```

`out\` は Git に入れません（`.gitignore` 済み）。

## 2. まず 3 枚だけ撮ってみる（Isaac Sim の Python）

```powershell
cd C:\Users\yousa\dev\ros2-learning
C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --max-shots 3 --good-only --out src\dual_arm_inspection\isaac\out\try
```

確かめること:
- `src\dual_arm_inspection\isaac\out\try\` に PNG が 3 枚できる。部品が画面の真ん中あたりに写っている
- 画面に出る `camera_err=` が 0.001（1mm）くらいより小さい。これは「自作の運動学で計算したカメラの位置」と「Isaac Sim に読み込んだロボットのカメラの位置」の差で、大きければ関節の名前の対応や URDF の読み込みがずれています
- 真っ黒なら照明が弱い（`--ring-intensity 10000` など）、真っ白なら強すぎ（`--ring-intensity 1000`）

画面を見ながら確かめたいときは `--gui` を足します（重くなります）。

## 3. 良品を集めて、欠陥ありを撮る

```powershell
# 良品（欠陥なし）: 撮影の一覧を 6 周 = 各面 6 枚
C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --good-only --repeat 6 --out src\dual_arm_inspection\isaac\out\good
# 欠陥あり: 半分の確率で打痕・傷・汚れを面の上に置く（2 周）
C:\isaacsim\python.bat src\dual_arm_inspection\isaac\capture_inspection.py --defects 0.5 --repeat 2 --seed 2 --out src\dual_arm_inspection\isaac\out\test
```

`shots.json` に、部品・面・欠陥の種類・Replicator の外接する四角（`bounding_box_2d_tight`）が入ります。

## 4. 判定する（pixi の環境）

```powershell
pixi run python src\dual_arm_inspection\isaac\judge_folder.py --good src\dual_arm_inspection\isaac\out\good --test src\dual_arm_inspection\isaac\out\test
```

疑似カメラの画像で学んだしきい値は Isaac Sim の画像に合わないので、**Isaac Sim の良品だけで学び直します**（撮り方が変わったら学び直す = 範囲外の見張りと同じ考え）。良品の 6 割で「ふつうの見え方」を作り、残りの 4 割でしきい値を決めます。

## うまくいかないとき

| 症状 | 考えられること | 試すこと |
|---|---|---|
| `ImportError: isaacsim.asset.importer.urdf.impl` | 拡張機能の名前・場所が違う | `C:\isaacsim\standalone_examples\api\isaacsim.asset.importer.urdf\urdf_import.py --test` が動くか確かめ、その import に合わせる |
| `r_camera_optical_frame が USD に見つからない` | 固定の関節がまとめられた | `cfg.merge_fixed_joints = False` を確かめる。だめでも、計画のカメラ姿勢に置くカメラで代わりに撮れる（その場合 `camera_err` は出ない） |
| `robot.dof_names.index(...)` で ValueError | 関節の名前が USD で変わった | 表示される `articulation: [...]` を見て、名前の対応を直す |
| 腕が垂れる・震える | 重力・駆動の強さ | `disable_gravity()` が効いているか。`override_joint_stiffness` を上げる |
| メモリ不足で落ちる | 12GB では足りない場面 | `--width 480 --height 360`、`--gui` を外す、ほかのアプリを閉じる。それでもだめならクラウドの GPU（**課金は承認が要る**） |
| 画像が暗い・白い | 照明の強さ（露出）の単位の違い | `--ring-intensity` と、`build_stage()` のドームライト（300）を調整 |

## このあと

- 部品の形を、synthetic-testbench の `scripts/make_parts.py` が作る USD（面法線つき・材質つき）に置き換える
- 欠陥を「小さな形」ではなく、テクスチャ（傷・汚れの絵）や形の変形（打痕のくぼみ）にする
- ROS 2 ブリッジで、`cell_node` の `/joint_states` を Isaac Sim がそのまま受けて動く形にする（いまは計画をファイルで渡している）
- Isaac Sim の画像と疑似画像で、検査の当たり外れがどれだけ違うかを比べる（合成と実物の差を詰める練習）
