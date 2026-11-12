#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$ROOT/optimization_v2/env.sh"
cd "$ROOT/optimization_v2/ws"
colcon build --packages-select sensor_sim_adapter --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
