#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
exec "$BASE/venvs/scene/bin/python" "$BASE/ws/src/coverage_planner/scripts/stage12.py" "$@"
