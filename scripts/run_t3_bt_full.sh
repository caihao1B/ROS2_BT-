#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$WS/scripts/e1_run_t3_method.sh" --method bt_full "$@"
