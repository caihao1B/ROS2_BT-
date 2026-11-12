#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --packages-select chassis_bridge robot_description --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
