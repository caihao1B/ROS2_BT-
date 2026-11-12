#!/usr/bin/env bash
set -Eeuo pipefail
STEPCTL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 "$STEPCTL_DIR/stepctl.py" "$@"
