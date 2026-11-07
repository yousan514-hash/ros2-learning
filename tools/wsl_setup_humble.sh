#!/usr/bin/env bash
# WSL2 の Ubuntu 22.04 に ROS 2 Humble を入れる（公式の手順どおり:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html ）。
#
# 現場の案件や OSS（OM1-sim）・peaq の SDK は「Ubuntu 22.04 ＋ Humble」が前提なので、
# Windows の Jazzy（pixi）とは別に、Linux の環境をもう1つ持つ。
# Windows から root で実行するので、sudo のパスワードは要らない:
#
#   wsl -d Ubuntu-22.04 -u root bash /mnt/c/Users/yousa/dev/ros2-learning/tools/wsl_setup_humble.sh fujita
#
# 何度実行してもよい（入っているものは飛ばす）。
set -euo pipefail

USER_NAME="${1:?使い方: wsl_setup_humble.sh <Ubuntu のユーザー名>}"
id "$USER_NAME" >/dev/null
export DEBIAN_FRONTEND=noninteractive

echo "== 1. ロケールを UTF-8 にする"
apt-get update
apt-get install -y locales software-properties-common curl
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "== 2. ROS 2 の apt リポジトリを足す（鍵と一覧は ros2-apt-source パッケージで入れる）"
add-apt-repository -y universe
ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
codename=$(. /etc/os-release && echo "$VERSION_CODENAME")
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb"
apt-get install -y /tmp/ros2-apt-source.deb

echo "== 3. 本体（desktop: rviz2・turtlesim・デモを含む）と開発の道具（colcon・rosdep など）"
apt-get update
apt-get upgrade -y
apt-get install -y ros-humble-desktop ros-dev-tools

echo "== 4. rosdep（依存パッケージを調べる道具）の初期化"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
sudo -u "$USER_NAME" -H rosdep update

echo "== 5. ログインしたら ROS 2 を使えるようにする"
line='source /opt/ros/humble/setup.bash'
bashrc="/home/${USER_NAME}/.bashrc"
grep -qxF "$line" "$bashrc" || echo "$line" >> "$bashrc"

set +u
# shellcheck disable=SC1091
. /opt/ros/humble/setup.bash
echo "ROS 2 を入れました: ROS_DISTRO=${ROS_DISTRO}"
