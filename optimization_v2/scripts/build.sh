#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --executor sequential --packages-select project_baseline --event-handlers desktop_notification- --cmake-args -DCMAKE_BUILD_TYPE=Release
