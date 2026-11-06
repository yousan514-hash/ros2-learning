# 学習記録

## 2026-11-02 — Windows に ROS 2 Jazzy を入れて、最初のノードを動かす

### やったこと
1. **環境選び**: この PC には WSL も Docker もなく、CPU の仮想化機能もオフだった。WSL2 を入れるには BIOS の設定変更と再起動が要る。そこで、管理者権限も再起動もいらない **pixi + RoboStack** で入れることにした。
   - 調べたら、Isaac Sim 6 の公式手順も Windows では pixi の ROS 2 Jazzy を使っていて、WSL2 は非推奨だった。後で Isaac Sim とつなぐときにも同じ方向で進められる。
2. **インストール**: `winget install prefix-dev.pixi` → `pixi install`。ROS 2 本体（ros-base）、turtlesim、デモのノードが入った。
   - 日本語の入ったパスは ROS のツールで問題が出やすいので、作業フォルダは `C:\Users\yousa\dev\ros2-learning` にした。
3. **デモを動かす**: `ros2 run demo_nodes_py talker` で `/chatter` に送り、`ros2 topic echo /chatter --once` で受信できた（`Hello World: 7`）。
4. **自分でノードを書く**: 公式チュートリアルの Python 版パブリッシャーとサブスクライバー（`src/py_pubsub`）を作り、`colcon build` でビルドした。
   - `ros2 node list` → `/minimal_publisher` と `/minimal_subscriber`
   - `ros2 topic info /topic` → Publisher 1 / Subscription 1
   - `ros2 topic hz /topic` → 平均 1.999Hz（0.5秒ごとに送る設定どおり）

### 詰まったこと
- **C++ のデモ（`demo_nodes_cpp talker`）が起動直後に落ちる。** 終了コードは `3221225477`（`0xC0000005`、メモリアクセス違反）。通信ミドルウェアを Fast DDS から Cyclone DDS に替えても同じだった。Python 版は問題なく動く。
  - **当面の対応**: 学習は Python で進める。
  - **次に試すこと**: Isaac Sim の公式ワークスペース（`IsaacSim-ros_workspaces` の `jazzy_ws`）に移るときに、MSVC Build Tools を入れて C++ をビルドし直す。公式の手順は Zenoh（`rmw_zenoh`）を使う。
- **colcon の警告**: 環境変数に日本語のパス（OneDrive のドキュメント）があるため、`cp932` で読めないという警告が出る。ビルド自体は成功するので無視してよい。

### 学んだこと
- ノードは「1つの仕事をするプログラム」、トピックは「ノード同士がメッセージを流す名前つきの通り道」。
- `ros2 node list` / `ros2 topic list` / `ros2 topic info` / `ros2 topic echo` / `ros2 topic hz` で、動いているシステムの中身を外から確かめられる。
- Python のノードは `rclpy.init()` → `Node` を作る → `rclpy.spin()` で動かし続ける、という形が基本。

## 2026-11-02（続き）— サービス、パラメータ、launch

### やったこと
1. **サービス**: `example_interfaces/srv/AddTwoInts` を使い、2つの整数を足して返す受け手（`add_server`）と頼み手（`add_client`）を作った。`add_client 2 3` → `5`。
2. **パラメータ**: 送る言葉（`greeting`）と間隔（`period`）を外から変えられる `param_talker` を作った。
   - `ros2 param get /param_talker greeting` → `Konnichiwa`
   - `ros2 param set /param_talker greeting Changed` で、動いている途中に言葉が変わった（`Changed: 24`）
   - `period` に `-1.0` を入れると、コールバックで拒否される（`Setting parameter failed`）
3. **launch**: `pubsub.launch.py` で `param_talker` と `listener` を1回のコマンドで起動し、引数をパラメータとして渡した。

### 詰まったこと
- **colcon のビルドで `UnicodeDecodeError: 'cp932'`。** `setup.py` の日本語を、Windows の標準の文字コード（cp932）で読もうとして失敗し、新しいノードがビルドされていなかった。その結果、頼み手がサービスを待ち続けて止まった。
  - **解決**: `pixi.toml` の `[activation.env]` に `PYTHONUTF8 = "1"` を入れて、Python を UTF-8 モードにした。警告も消えた。
  - **再発防止**: 頼み手が無限に待たないよう、10秒で諦めてエラーにした。
- **launch の引数は文字列。** `period:=1.0` をそのまま渡すと文字列になり、小数で宣言したパラメータと型が合わない。`ParameterValue(period, value_type=float)` で小数として渡した。
- パラメータの拒否理由を日本語にすると、Windows の画面で文字化けした。英語にした。

### 学んだこと
- **トピック**は「流しっぱなし」、**サービス**は「頼んで、答えを1回もらう」。用途で使い分ける。
- **パラメータ**はノードの設定値。外から読み書きでき、変更を受け入れるか拒否するかをノード側で決められる。
- **launch** は複数のノードとその設定を1つのファイルにまとめる仕組み。ロボットのシステムはこれで起動する。

## 2026-11-02（続き）— C++ のノードが落ちる原因がわかった

### 調べ方
1. **Windows のイベントログ**（アプリケーション、イベント ID 1000）で、落ちたときの「障害が発生したモジュール」を探した。talker の記録はなかったが、**別のアプリ（GitHub Copilot）が同じ `0xC0000005` で `C:\WINDOWS\SYSTEM32\MSVCP140.dll` の中で落ちていた**。
2. バージョンを比べた。
   - Windows 本体の `MSVCP140.dll`: **14.24**（入っている Visual C++ 再頒布パッケージは 2019 年の 14.22）
   - pixi の環境（conda-forge）の `msvcp140.dll`: **14.51**
3. **確かめた**: 環境の新しい DLL を一時的に `talker.exe` の隣に置くと、**落ちずに動いた**。確認後に元に戻した。

### 原因
新しいコンパイラで作られた C++ のプログラムが、**古い Visual C++ ランタイム（`MSVCP140.dll`）を読み込むと、`std::mutex` などで落ちる**。

Windows は DLL を「exe と同じフォルダ → `System32` → `PATH`」の順に探す。
- `python.exe` は環境の一番上のフォルダにあり、隣に新しい DLL があるので問題ない（だから Python のノードは動いた）
- `talker.exe` は `Library\lib\demo_nodes_cpp\` にあり、隣に DLL がないので、**先に `System32` の古い DLL** が読まれて落ちる

### 直し方
- **根本の対策**: Visual C++ 2015-2022 再頒布パッケージ（x64）を最新にする（`winget install Microsoft.VCRedist.2015+.x64`、管理者の許可が必要）
- 管理者になれないときの回避策: 環境の新しい DLL を、使う exe の隣に置く（exe ごとに必要なのでおすすめしない）

### 学んだこと
- Windows で C++ のプログラムが `0xC0000005` ですぐ落ちるときは、**イベントログの「障害が発生したモジュール」**を見る。同じ PC のほかのアプリの記録も手がかりになる。
- 「Python は動くのに C++ だけ落ちる」は、**DLL の探し方の違い**（exe の置き場所）で説明できることがある。

## 2026-11-02（続き）— tf2（座標変換）と rosbag2（記録と再生）

C++ のノードは Visual C++ ランタイムの更新待ちなので、Python のプロセスから使える2つを先に進めた。中身は C++ だが、Python のプロセスでは python.exe の隣の新しいランタイムが先に読まれているので落ちない。

### tf2
- `frame_broadcaster`: world → robot（半径 1m の円を 10 秒で 1 周、進む向きを向く）と、robot → camera（前 0.2m・上 0.1m に固定。静的な変換）を送る
- `frame_listener`: world → camera をたどって、カメラが世界のどこにあるかを求める
- **確かめ方**: ロボットが円の接線を向き、カメラが前 0.2m にあるなら、カメラの原点からの距離は常に √(1² + 0.2²) ≒ 1.0198m になる
- **結果**: `camera in world: x=+1.019 y=-0.037 z=+0.100 dist=1.020`。距離も高さも合っている。`/tf` と `/tf_static` が流れている

### rosbag2
- `bag_recorder`: `/topic` を決まった数だけ記録し、ファイルを閉じて終わる（記録のコマンドを強制終了するとファイルが壊れることがあるので、公式チュートリアルの「ノードから記録する」方式にした）
- **結果**: 0.2 秒ごとに送る talker から 10 件を記録。`ros2 bag info` → `Storage id: mcap`、`Messages: 10`、`Duration: 1.80s`
- `ros2 bag play bags/talk` で再生すると、記録したメッセージ（`Recorded: 60`）がもう一度 `/topic` に流れた

### 学んだこと
- **tf2** は「どの部品が、どの部品から見てどこにあるか」の木構造。動く関係（tf）と、動かない関係（tf_static）を分けて送る。たどる道筋は tf2 が自動で計算する
- 向き（回転）はクォータニオンで表す。z 軸まわりだけなら `(0, 0, sin(θ/2), cos(θ/2))`
- **rosbag2** は流れたデータをそのまま保存して、あとで同じように流し直せる。Isaac Sim のセンサーのデータを記録して、あとで画像処理を試すときに使える

## 2026-11-03 — C++ のノードを動かす、アクション、スマート アプリ コントロール

### C++ のノード（Visual C++ ランタイム）
- Visual C++ 再頒布可能パッケージの更新は、管理者の許可の画面が取り消されて終わった（`winget` の終了コード 1602）。Windows のランタイムは 14.24 のまま
- 管理者権限なしで直す `tools/vcrt_local.py` を作った。環境に入っている新しいランタイム（14.51）を、C++ の exe があるフォルダに置く（Microsoft が認めている「ローカル配置」）
  - System32 のほうが新しければ何もしない（あとで再頒布パッケージを更新すれば、自然に不要になる）
  - 同じドライブならハードリンクで置くので、容量をほとんど使わない
  - `--undo` で、置いたもの（環境のランタイムと中身が同じもの）だけを消す
- `talker_cpp` と `turtlesim` のタスクは、先に `vcrt_local` を実行するようにした
- **結果**: C++ の talker が落ちずに動いた（`Publishing: 'Hello World: 1'` … 1 秒ごと）

### turtlesim が止められた（スマート アプリ コントロール）
- turtlesim を起動すると `[WinError 4551] アプリケーション制御ポリシーによってこのファイルがブロックされました`
- **調べ方**: イベントログ `Microsoft-Windows-CodeIntegrity/Operational` の ID 3077 に、止められたファイルが出ていた（`turtlesim_node.exe` と `turtlesim__rosidl_generator_py.dll`）。レジストリの `VerifiedAndReputablePolicyState` は 1（オン）
- **原因**: Windows 11 の「スマート アプリ コントロール」は、読み込むすべての exe・DLL について、署名か、Microsoft のクラウドでの評判を確かめ、どちらもなければ止める（[解説](https://textslashplain.com/2026/04/28/smart-app-control/)）。conda-forge / RoboStack のファイルには署名がない。rclpy や demo_nodes_cpp は通り、turtlesim は止められた。評判の違いとみられる
- **対処**: スマート アプリ コントロールをオフにすれば動くはず。セキュリティの設定なので、オフにするかは PC の持ち主が決める
  - 2026 年 4 月の更新から、オフにしたあとでもオンに戻せるようになった（[topedia](https://blog.topedia.com/2026/04/smart-app-control-in-windows-11-can-now-be-re-enabled-without-reinstalling/)）
  - ただし 10 月の更新 KB5127648（この PC に入っている）で、オフにした PC でオンに戻せなくなる不具合を Microsoft が認めている（[IT-Connect](https://www.it-connect.tech/windows-11-kb5127648-breaks-smart-app-control-in-october-2026-update/)）
- 自分で C++ をビルドした exe・DLL も署名がないので、同じように止められるとみられる。Isaac Sim × ROS 2（`IsaacSim-ros_workspaces` を自分でビルドする）に進むときにも関係する

### アクション（Fibonacci）
- `fibonacci_server` / `fibonacci_client`: `example_interfaces/action/Fibonacci` を使う公式チュートリアル。turtlesim のアクション（`rotate_absolute`）は上の理由で使えないので、こちらにした
- 計算の途中でも取り消しを受け付けるよう、`MultiThreadedExecutor` と `ReentrantCallbackGroup` を使った（スレッドが 1 本だと、計算している間は取り消しの依頼が処理されない）
- **結果**:
  - `order:=10` → 途中経過が 9 回届き、結果 `[0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]`（succeeded）
  - `order:=10 cancel_after:=1.2` → `[0, 1, 1, 2, 3]` で canceled
  - `order:=100` → サーバーが断った（goal rejected）

### 亀に字を書かせる（turtle_draw）
- サービス（`teleport_absolute`、`clear`）で始点に移って画面を消し、トピック（`pose` を受け取り、`cmd_vel` を送る）で点を順にたどる。距離と向きのずれに比例して速さを決める（P 制御）。向きが合うまで進まないので、角がきれいに曲がる
- turtlesim を動かせないので、turtlesim と同じ動きの式（16ms ごとに向き → 位置の順に更新し、最後の速度の指令を使い続ける）で確かめる `tools/check_turtle_draw.py` を作った
- **結果**: 「F」の 5 点すべてに誤差 0.05 以内で着き、線分からのずれは最大 0.048（15.9 秒）。turtlesim の本体での確認は、スマート アプリ コントロールの判断待ち

### 詰まったこと
- アクションの頼み手で、ゴールのハンドルを `self.handle` に入れたら `AttributeError: handle cannot be modified after node creation`。`handle` は rclpy の Node がすでに使っている名前だった。`self.goal_handle` にした

### 学んだこと
- **アクション**は「頼む → 引き受ける／断る → 途中経過 → 結果」。途中で取り消せる。移動ロボットの「ここまで行って」（Nav2 の NavigateToPose）もこの仕組み
- Windows で「ブロックされました」と出たら、ウイルス対策ではなく**スマート アプリ コントロール**のことがある。イベントログの CodeIntegrity で、止められたファイルがわかる
- 動かせないものがあっても、**同じ式で動く小さなシミュレーション**を作れば、制御の考え方は先に確かめられる

## 2026-11-03（続き）— 2 つの設定を変えて、turtlesim が動いた

### Visual C++ ランタイムを更新した
- Visual C++ 再頒布可能パッケージを更新した（System32 の `msvcp140.dll`: 14.24 → **14.51**）
- `pixi run vcrt_local --undo` で、exe の隣に置いた DLL（110 個）を消してから、C++ の talker を直接起動した → **落ちずに動いた**（`Publishing: 'Hello World: 1'` …）
- これで「落ちる原因は古いランタイム」が確定した。`vcrt_local` は今は何もしない（`System32 のほうが新しいか同じなので、何もしません`）。管理者の許可がもらえない PC のために残す

### スマート アプリ コントロールをオフにした
- Windows セキュリティ → アプリとブラウザー コントロール → スマート アプリ コントロールの設定 → オフ。`VerifiedAndReputablePolicyState` が 1 → 0 になった（再起動は要らなかった）
- turtlesim が起動し、`turtle_draw` が「F」を描いた

| 点 | 目標 | 誤差 |
|---|---|---|
| 1 | (3.0, 9.0) | 0.049 |
| 2 | (7.5, 9.0) | 0.050 |
| 3 | (3.0, 9.0) | 0.049 |
| 4 | (3.0, 5.5) | 0.050 |
| 5 | (6.5, 5.5) | 0.049 |

- 画像: [`docs/turtle_draw_F.png`](docs/turtle_draw_F.png)（turtlesim の窓を `PrintWindow` で保存し、青い画面の部分だけ切り出した）

### 学んだこと
- **小さなシミュレーションで先に確かめた結果（誤差 0.048〜0.050）と、本物の turtlesim の結果（0.049〜0.050）が一致した。** 動きの式を同じにしておけば、本物がなくても制御の考え方は確かめられる
- 環境の問題は「回避策で先に進む → 根本の対策が済んだら回避策を外して確かめる」の順にすると、原因の切り分けがはっきりする

## 2026-11-05 — ロボット × ブロックチェーン: 稼働台帳（robot_ledger）

### ねらい
「ロボットの働きを検証できる記録にして、働いた分だけ払う」を、小さく動かす。ブロックチェーンの芯（ID・署名・追記だけの鎖）を自分で作って中身を理解する。新しいものは入れず、ROS 2 の環境にもともと入っていた `cryptography`（Ed25519）と `pytest` を使った。

### 作ったもの
- `ledger.py`: ロボットの鍵と ID（公開鍵のハッシュ）、1行ずつ署名してハッシュでつなぐ台帳、確かめる処理、JSON Lines での保存
- `billing.py`: 登録された公開鍵で台帳を確かめてから、契約の「使用範囲」（誤差の上限）の中の仕事だけを数えて請求額を出す
- `recorder_node.py`: turtlesim の亀を「働くロボット」に見立て、決めた点で止まるたびに `task_done` と誤差を署名つきで書き足す。亀を動かす turtle_draw とは別のノードにして、メーターのように外から測る
- `cli.py`: `ledger_verify`（確かめる）と `ledger_bill`（請求額）

### 結果
- テスト12件すべて合格（書き換え・削除・ハッシュの計算し直し・別の鍵で作った偽の台帳・請求）
- turtlesim + turtle_draw + ledger_recorder: 台帳は7行（開始・仕事5件・終了）。誤差は 0.045〜0.048、走った距離 22.73
- `ledger_verify` → OK
- `ledger_bill`（1件100円）: 誤差0.05以内なら **5件・500円**、0.046以内なら **3件・300円**
- 点2の誤差を 0.048 → 0.001 に書き換えたコピー → `seq 2: 中身が書き換えられている` で NG、請求も拒否

### 学んだこと
- **ハッシュの鎖だけでは足りない。** 中身を書き換えてハッシュを計算し直すと鎖はつながってしまうが、**秘密鍵がないので署名が合わない**。改ざんを止めているのは署名
- **署名だけでも足りない。** 別の鍵で台帳を丸ごと作り直すと、中身は矛盾なく「正しく」見える。**どの鍵が本物のロボットかを登録しておく（ID の登録）**ことで初めて見抜ける。ブロックチェーンで ID を管理する理由はここ
- 「使用範囲」を契約に入れると、同じ台帳でも請求額が変わる。**何を仕事として数えるかを、記録の前に決めておく**ことが大事
- 亀の瞬間移動（teleport）まで距離に数えてしまったので、1回で1m以上の移動は飛ばした

### 次
- Isaac Sim の Franka が物を置くたびに、同じ台帳に記録する（Isaac Sim の Python 3.12 にも cryptography が入っていた）
- 受け取る側（工場）の署名を足して「両者が認めた仕事」にする
- 台帳の最後のハッシュを公開のブロックチェーン（テストネット）に刻む

## 2026-11-06 — 双腕の外観検査セル（dual_arm_inspection）を RViz で動かす

### ねらい
「双腕ロボットと画像検査のシミュレーション」を、まず ROS 2 + RViz で動いて見える形にする（1 段目）。Isaac Sim で撮るのは下書きまで（2 段目）。題材は自作の簡単な形と、synthetic-testbench の部品（ボルト・ナット・座金）だけ。新しいものは入れていない（rviz2・xacro・joint_state_publisher・opencv 5.0 は統括が先に入れた）。

### 作ったもの
- `urdf/dual_arm_cell.urdf.xacro`: 胴体（箱）+ 6 軸の腕 2 本。左手はグリッパ（指 2 本・直動の関節）、右手は手首カメラ（リング照明つき）。机・トレイ・照明（`with_cell:=false` で外せる）。関節の上限・下限あり
- `kinematics.py`: 腕は「肩の旋回 → 肩・ひじの曲げ → 前腕のひねり → 手首の曲げ → 手先のひねり」で、j4〜j6 の軸が 1 点で交わる（球面手首）。逆運動学を「手首の中心の位置」と「手先の向き（ZYZ のオイラー角）」に分けて式で解く（最大 8 通り → 上下限の中で今の姿勢に近いもの）
- `planner.py`: 1 個の部品ごとに「右のカメラがトレイの上の面を撮る → 左手が取る → カメラの前に出して面を変えながら撮る → 戻す」。見せる向き・カメラの向きは法線まわりに 15 度ずつ 24 通り試し、腕どうし（太さのある線分の距離）と机・胴体・照明とのすき間を確かめて選ぶ。途中の点で近づきすぎるときは片腕ずつ動かす
- `imaging.py`（疑似カメラ）: 面の種類ごとの形と明るさの模様、傷・打痕・汚れ（強さ 0〜1）、照明のばらつき、ときどき範囲外の照明。`inspection.py`（検査）: 良品だけで「平均の見え方とばらつき」を学び、位置を合わせて比べ、OK／NG／保留の 3 段。明るさ・背景・明るさの傾き・面積が学んだ範囲の外なら保留（範囲外の見張り）
- ノード 3 つ（`cell_node`・`camera_node`・`inspector_node`）、launch（robot_state_publisher + 3 ノード + rviz2）、RViz の設定（RobotModel・TF・Marker・Image）
- `tools/check_dual_arm.py`（GUI なしで確かめる → `docs/dual_arm_inspection.png`）、pytest 28 件、pixi のタスク `dual_arm`・`check_dual_arm`・`test_dual_arm`・`dual_arm_urdf`
- `isaac/`: Isaac Sim 6.2 で同じ姿勢を撮る下書き（`capture_inspection.py`、未実行）と手順書、`export_urdf.py`・`judge_folder.py`（pixi 側、実行・テスト済み）

### 結果
- pytest: 28 件すべて合格（逆運動学・URDF と自作の運動学の一致・計画・疑似カメラ・3 段判定・画像メッセージ・judge_folder）
- `pixi run check_dual_arm`:
  - 動きの目標 65 個: すべて解けて上下限の中（手先の誤差 最大 位置 4e-16 m・向き 1e-15 rad）。探した候補 9,024 個のうち解があったのは 8,920 個（98.8%）
  - 1 周 194.9 秒を 50Hz で刻んだ 9,746 点: 上下限の外 0、腕どうしの最短 0.052 m（下限 0.03 m、「nut_M4 の横の面を見せる」とき）、机・トレイ・胴体・照明との最短 0.050 m、関節の速さの最大 1.35 rad/s（上限 1.5）
  - でたらめな関節角 2000 通り × 2 腕を解き直し: すべて元に戻った（誤差 最大 5e-14）
  - 検査（疑似画像 200 枚、欠陥 30%・範囲外 5%）:

| 正解＼判定 | OK | NG | 保留 |
|---|---|---|---|
| 良品 143 | 142 | 0 | 1 |
| 不良 50 | 1 | 49 | 0 |
| 範囲外 7 | 0 | 0 | 7 |

  見逃し 2.0%（強さ 0.00 のごく薄い傷 1 枚）、過検出 0%、人に回る 4.0%。しきい値は良品だけで決めた（OK < 3.0 ≦ 保留 < 9.0 ≦ NG）。数値は自作の疑似画像での結果で、仮定（欠陥の強さの分布・照明）で変わる
- launch で動かした: `ros2 topic list` に `/joint_states`・`/tf`・`/robot_description`・`/wrist_camera/image_raw`・`/inspection/result` など 13 本。`ros2 topic echo --once` で `/inspection/result`（JSON）・`/joint_states`（14 個の値）・`/inspection/image_annotated`（rgb8、step 960）を確かめた
- RViz の画面: [`docs/dual_arm_rviz.png`](docs/dual_arm_rviz.png)（見せて撮っている所）、[`docs/dual_arm_rviz_hold.png`](docs/dual_arm_rviz_hold.png)（保留: 打痕の疑い・点数 5 → 人へ）、[`docs/dual_arm_rviz_ng.png`](docs/dual_arm_rviz_ng.png)（NG: 傷）

### 詰まったこと
- **トレイのいちばん遠い場所に、左手が届かなかった**（逆運動学の解なし）。腕を少し長く（上腕・前腕 0.30 → 0.32 m）、肩を低く（1.12 → 1.05 m）、トレイを手前（中心 x 0.46 → 0.44 m）にした
- **計画に 6.7 秒かかった。** 見せる向き × カメラの向きの 576 組すべてで当たりを確かめていた。動きの小さい順に並べ、最初に通ったものを使うようにして 1.9 秒
- **良品なのに NG になる画像があった**（ボルトの頭と軸の境、六角の面の境）。位置合わせが 0.3 画素ずれるだけで明るさが大きく違う所で、良品 20 枚ではばらつきを学びきれなかった。模様の傾き × 0.35 画素の分をばらつきに足して直した
- **`ros2 topic list` が 60 秒たっても返らなかった。** `--no-daemon` だけだと `/rosout` と `/parameter_events` しか出ない。`--no-daemon --spin-time 5` で全部出た（見つけ合うのに数秒かかる）
- **Python のノードのログが launch の画面に出なかった**（出力をファイルに向けたとき）。`emulate_tty=True` を付けても同じ。ログは `C:\Users\yousa\.ros\log\python_*.log` に出ているので、そこで確かめた
- **ノードが動いている間に `pixi run build` すると `Permission denied`**（`camera_node.exe` を上書きできない）。止めてからビルドする
- **RViz の文字の空白がとても広く描かれ、語がばらばらになった**（Windows）。空白を `_` に替え、行は改行で分けた
- **裏で起動した RViz が最小化で始まり、Image の表示が切れていた。** 普通の窓で起動すると Image も出た。ターミナルから `pixi run dual_arm` で起動すれば普通の窓になるはず（藤田さんの手元では未確認）
- 画面の撮影で窓が小さく写った: PowerShell が DPI に対応していないと、窓の大きさが 1/1.5 で返る。`SetProcessDPIAware()` を呼んでから `PrintWindow` した

### 学んだこと
- **逆運動学は、腕の形を「手首の 3 軸が 1 点で交わる」ように決めると、位置と向きに分けて式で解ける。** 解が 8 通りあるので、上下限と「今の姿勢に近い」で 1 つに決める。ひじの下限を正にすると、ひじが下に回る形を最初から外せる
- **双腕は、止まった姿勢だけでなく「動いている途中」の距離も確かめる。** 今の計画では両腕が同時に動く区間が 18 あり、途中の点まで含めてすべて下限を超えていた（近づきすぎたときに片腕ずつに切り替える仕組みは入れたが、今の計画では一度も使われていない）
- **3 段判定のしきい値は、学びに使っていない良品で決める。** 学びに使った画像で決めると、本番より甘く見える
- 撮り方（照明・カメラ）が変わったら、学び直しが要る。Isaac Sim の画像を判定するときも、Isaac Sim の良品で学び直す（`isaac/judge_folder.py`）
