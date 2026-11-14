#!/usr/bin/env bash
set -Eeuo pipefail
r01_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$r01_base/env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$r01_base/venvs/scene/bin/python" "$r01_base/ws/src/task_runtime/scripts/taskctl.py" "$@"
