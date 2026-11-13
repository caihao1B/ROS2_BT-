#!/usr/bin/env bash
set -Eeuo pipefail
release_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$release_base/env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$release_base/venvs/scene/bin/python" "$release_base/ws/src/release_tools/scripts/release18.py" "$@"
