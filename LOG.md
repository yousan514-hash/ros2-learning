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
