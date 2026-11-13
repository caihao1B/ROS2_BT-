#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mkdir -p "$BASE/build_s17"
c++ -std=c++20 -O2 -Wall -Wextra -Werror -I"$BASE/ws/src/acceptance_runner/include" "$BASE/ws/src/acceptance_runner/src/monitor.cpp" -o "$BASE/build_s17/monitor"
echo 'S17 independent monitor built'
source "$BASE/env.sh"
cd "$BASE/ws"
colcon build --packages-select acceptance_runner --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
