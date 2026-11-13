#!/usr/bin/env bash
set -Eeuo pipefail
s18_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$s18_base/env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$s18_base/venvs/scene/bin/python" "$s18_base/ws/src/release_tools/scripts/stage18.py" "$@"
