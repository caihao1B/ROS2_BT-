#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --packages-select gnss_supervisor --symlink-install
