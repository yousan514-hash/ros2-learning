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
