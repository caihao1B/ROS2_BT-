#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mkdir -p "$BASE/setup_logs"
exec > >(tee "$BASE/setup_logs/install_s01.log") 2>&1
trap 'rc=$?; printf "%s\n" "$rc" > "$BASE/setup_logs/install_s01.exit_code"; echo "安装退出码：$rc"' EXIT
echo '第 1 步：安装 Fields2Cover 2.1.1、METIS 5.1.0 和必要依赖，用于编译兼容性样例。'
echo '请只在本终端的 sudo 提示处输入本机密码。'
PACKAGES=(ros-jazzy-fields2cover=2.1.1-1noble.20260922.035009 libmetis-dev=5.1.0.dfsg-7build3)
apt-get -s install --no-install-recommends --no-remove "${PACKAGES[@]}" > "$BASE/setup_logs/install_s01.simulation.txt"
cat "$BASE/setup_logs/install_s01.simulation.txt"
if grep -q '^Remv ' "$BASE/setup_logs/install_s01.simulation.txt"; then exit 2; fi
sudo -v
sudo apt-get -o Acquire::http::Proxy::packages.ros.org=http://127.0.0.1:10808 \
  -o Acquire::https::Proxy::packages.ros.org=http://127.0.0.1:10808 \
  -o Acquire::http::Timeout=30 -o Acquire::Retries=2 \
  install -y --no-install-recommends --no-remove "${PACKAGES[@]}"
dpkg-query -W -f='${Package}\t${Version}\t${Status}\n' ros-jazzy-fields2cover libmetis-dev
echo 'S01_DEPENDENCIES_INSTALLED'
