#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-static}"
case "$PROFILE" in
  static)
    cp "$WS/profiles/executor_e1_static.yaml" "$WS/src/coverage_manager/params/executor_params.yaml"
    cp "$WS/profiles/supervisor_e1_static.yaml" "$WS/src/coverage_bt_supervisor/params/supervisor.yaml"
    ;;
  dynamic)
    cp "$WS/profiles/executor_dynamic.yaml" "$WS/src/coverage_manager/params/executor_params.yaml"
    cp "$WS/profiles/supervisor_dynamic_full.yaml" "$WS/src/coverage_bt_supervisor/params/supervisor.yaml"
    ;;
  norepair)
    cp "$WS/profiles/executor_dynamic.yaml" "$WS/src/coverage_manager/params/executor_params.yaml"
    cp "$WS/profiles/supervisor_norepair.yaml" "$WS/src/coverage_bt_supervisor/params/supervisor.yaml"
    ;;
  closure_off)
    cp "$WS/profiles/executor_dynamic.yaml" "$WS/src/coverage_manager/params/executor_params.yaml"
    cp "$WS/profiles/supervisor_closure_off.yaml" "$WS/src/coverage_bt_supervisor/params/supervisor.yaml"
    ;;
  *)
    echo "Usage: bash scripts/apply_profile.sh {static|dynamic|norepair|closure_off}" >&2
    exit 2
    ;;
esac

echo "[OK] Applied profile: $PROFILE"
echo "  executor   -> src/coverage_manager/params/executor_params.yaml"
echo "  supervisor -> src/coverage_bt_supervisor/params/supervisor.yaml"
