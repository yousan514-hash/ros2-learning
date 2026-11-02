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
