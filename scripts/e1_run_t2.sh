#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_ID=""
PLANNER=""
PLAN_FILE=""
ASSET_INDEX=""
SPAWN_SEG_IDX="${SPAWN_SEG_IDX:-0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map-id) MAP_ID="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --plan-file) PLAN_FILE="$2"; shift 2;;
    --asset-index) ASSET_INDEX="$2"; shift 2;;
    --spawn-seg) SPAWN_SEG_IDX="$2"; shift 2;;
    *) echo "[EXP][T2] unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$PLAN_FILE" ]]; then
  RESOLVE_ARGS=()
  [[ -n "$ASSET_INDEX" ]] && RESOLVE_ARGS+=(--asset-index "$ASSET_INDEX")
  [[ -n "$MAP_ID" ]] && RESOLVE_ARGS+=(--map-id "$MAP_ID")
  [[ -n "$PLANNER" ]] && RESOLVE_ARGS+=(--planner "$PLANNER")
  PLAN_FILE=$(python3 "$WS/scripts/e1_common.py" resolve "${RESOLVE_ARGS[@]}" --field plan_json)
fi

if [[ ! -f "$PLAN_FILE" ]]; then echo "[EXP][T2][ERROR] plan not found: $PLAN_FILE" >&2; exit 3; fi

echo "[EXP][T2] plan=$PLAN_FILE"
bash "$WS/scripts/t2_freeze_map_odom_manual.sh" "$PLAN_FILE" "$SPAWN_SEG_IDX"
