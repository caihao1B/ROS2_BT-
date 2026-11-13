#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$BASE/env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$BASE/venvs/scene/bin/python" "$BASE/ws/src/experience_memory/scripts/stage16.py" "$@"
