#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# Linux x86_64 / Python 3.12 wheels; exact versions and SHA256, no sudo pip.
flock -n "$BASE/../native_20261108/runs/session.lock" bash -c '
  set -Eeuo pipefail
  base="$1"
  /usr/bin/python3 -m venv "$base/venvs/prediction"
  "$base/venvs/prediction/bin/pip" install --require-hashes -r "$base/config/prediction_wheels.lock"
  "$base/venvs/prediction/bin/pip" check
' bash "$BASE"
