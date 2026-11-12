#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mkdir -p "$BASE/setup_logs"
exec > >(tee "$BASE/setup_logs/install_s06.log") 2>&1
trap 'rc=$?; printf "%s\n" "$rc" > "$BASE/setup_logs/install_s06.exit_code"; echo "安装退出码：$rc"' EXIT
echo 'S06：仅安装 RTAB-Map SLAM 0.23.13 与必要消息依赖。请在本终端输入 sudo 密码。'
PACKAGES=(ros-jazzy-rtabmap-slam=0.23.13-1noble.20261003.101914)
apt-get -s install --no-install-recommends --no-remove "${PACKAGES[@]}" > "$BASE/setup_logs/install_s06.simulation.txt"
cat "$BASE/setup_logs/install_s06.simulation.txt"
if grep -q '^Remv ' "$BASE/setup_logs/install_s06.simulation.txt"; then exit 2; fi
sudo -v
sudo apt-get -o Acquire::http::Proxy::packages.ros.org=http://127.0.0.1:10808 -o Acquire::https::Proxy::packages.ros.org=http://127.0.0.1:10808 -o Acquire::http::Timeout=30 -o Acquire::Retries=2 install -y --no-install-recommends --no-remove "${PACKAGES[@]}"
dpkg-query -W -f='${Package}\t${Version}\t${Status}\n' ros-jazzy-robot-localization ros-jazzy-rtabmap ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-msgs ros-jazzy-rtabmap-slam
echo S06_DEPENDENCIES_INSTALLED
