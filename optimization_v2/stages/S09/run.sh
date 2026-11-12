#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
exec "$BASE/venvs/scene/bin/python" "$BASE/ws/src/terrain_perception/scripts/stage.py" "$@"
