#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
exec "$BASE/venvs/scene/bin/python" "$BASE/ws/src/sensor_sim_adapter/scripts/stage_runner.py" "$@"
