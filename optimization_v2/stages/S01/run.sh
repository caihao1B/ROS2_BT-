#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
exec /usr/bin/python3 "$BASE/ws/src/project_baseline/scripts/stage_runner.py" "$@"
