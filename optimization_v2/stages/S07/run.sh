#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
exec "$BASE/venvs/scene/bin/python" "$BASE/ws/src/global_pose_manager/scripts/localization_stage.py" "$@"
