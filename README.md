# ros2-learning

ROS 2 Jazzy を **Windows 11 にネイティブで** 入れて学ぶ記録です。WSL2 も Docker も使わず、[pixi](https://pixi.sh/) と [RoboStack](https://robostack.github.io/) で環境を作っています。

ゴールは、ROS 2 の基礎 → Isaac Sim との連携 → Isaac Lab での強化学習、まで進むことです。

> この方法を選んだ理由: Isaac Sim 6 の公式手順でも、Windows では pixi で入れる ROS 2 Jazzy が使われていて、WSL2 は非推奨になっている（[Isaac Sim: ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)）。

## 環境

| 項目 | 内容 |
|---|---|
| OS | Windows 11 Home |
| ROS 2 | Jazzy（RoboStack） |
| パッケージ管理 | pixi 0.81.0 |
| Python | 3.12 |
| GPU | RTX 4070（12GB） |

## 使い方

```powershell
# pixi を入れる（初回のみ）
winget install prefix-dev.pixi

# 環境を作る（初回は数分かかる）
pixi install

# 自作パッケージをビルド
pixi run build

# 2つのターミナルで実行
pixi run talker     # 0.5秒ごとに "Hello World: N" を送る
pixi run listener   # 受け取って表示する
```

確認用のコマンド:

```powershell
pixi run ros2 node list         # /minimal_publisher, /minimal_subscriber
pixi run ros2 topic info /topic # Publisher 1 / Subscription 1
pixi run ros2 topic hz /topic   # 約 2Hz
```

## 中身

| パス | 内容 |
|---|---|
| [`src/py_pubsub`](src/py_pubsub) | Python のパブリッシャーとサブスクライバー（公式チュートリアル） |
| [`pixi.toml`](pixi.toml) | 環境と実行コマンドの定義 |
| [`LOG.md`](LOG.md) | 日ごとの学習記録（やったこと・詰まったこと・解決策） |

## 進み具合

- [x] Windows に ROS 2 Jazzy を入れる（pixi / RoboStack）
- [x] デモのノードを動かし、ノードとトピックを確認する
- [x] Python でパブリッシャーとサブスクライバーを書く
- [ ] サービスとパラメータ
- [ ] launch ファイル
- [ ] turtlesim で移動ロボットを動かす
- [ ] Isaac Sim × ROS 2（公式の `IsaacSim-ros_workspaces` に移る）
- [ ] Isaac Lab で強化学習

## ライセンス

Apache-2.0
