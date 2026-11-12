#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --packages-select motion_prediction prediction_risk_guard --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
