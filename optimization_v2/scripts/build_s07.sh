#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --packages-select global_pose_manager --symlink-install
