#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_ID=""
PLANNER=""
PLAN_FILE=""
MAP_YAML=""
WORLD_FILE=""
ASSET_INDEX=""
WITH_RVIZ="${WITH_RVIZ:-1}"
SPAWN_SEG_IDX="${SPAWN_SEG_IDX:-0}"
NAV2_PARAMS="${NAV2_PARAMS_OVERRIDE:-}"
MODEL_ROOT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map-id) MAP_ID="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --plan-file) PLAN_FILE="$2"; shift 2;;
    --map-yaml) MAP_YAML="$2"; shift 2;;
    --world-file) WORLD_FILE="$2"; shift 2;;
    --asset-index) ASSET_INDEX="$2"; shift 2;;
    --rviz) WITH_RVIZ="$2"; shift 2;;
    --spawn-seg) SPAWN_SEG_IDX="$2"; shift 2;;
    --nav2-params) NAV2_PARAMS="$2"; shift 2;;
    *) echo "[EXP][T1] unknown arg: $1" >&2; exit 2;;
  esac
done

RESOLVE_ARGS=()
[[ -n "$ASSET_INDEX" ]] && RESOLVE_ARGS+=(--asset-index "$ASSET_INDEX")
[[ -n "$MAP_ID" ]] && RESOLVE_ARGS+=(--map-id "$MAP_ID")
[[ -n "$PLANNER" ]] && RESOLVE_ARGS+=(--planner "$PLANNER")
[[ -n "$PLAN_FILE" ]] && RESOLVE_ARGS+=(--plan-file "$PLAN_FILE")
[[ -n "$MAP_YAML" ]] && RESOLVE_ARGS+=(--map-yaml "$MAP_YAML")
[[ -n "$WORLD_FILE" ]] && RESOLVE_ARGS+=(--world-file "$WORLD_FILE")
ASSET_JSON=$(python3 "$WS/scripts/e1_common.py" resolve "${RESOLVE_ARGS[@]}")

readarray -t RESOLVED < <(python3 - <<'PY' "$ASSET_JSON"
import json,sys
x=json.loads(sys.argv[1])
for key in ('map_id','planner','plan_json','map_yaml','world_file','model_root'):
    print(x.get(key,''))
PY
)
MAP_ID="${RESOLVED[0]}"
PLANNER="${RESOLVED[1]}"
PLAN_FILE="${RESOLVED[2]}"
MAP_YAML="${RESOLVED[3]}"
WORLD_FILE="${RESOLVED[4]}"
MODEL_ROOT="${RESOLVED[5]}"

if [[ ! -f "$PLAN_FILE" ]]; then echo "[EXP][T1][ERROR] plan not found: $PLAN_FILE" >&2; exit 3; fi
if [[ ! -f "$MAP_YAML" ]]; then echo "[EXP][T1][ERROR] map yaml not found: $MAP_YAML" >&2; exit 4; fi
if [[ ! -f "$WORLD_FILE" ]]; then echo "[EXP][T1][ERROR] world not found: $WORLD_FILE" >&2; exit 5; fi
if [[ -n "$MODEL_ROOT" && ! -d "$MODEL_ROOT" ]]; then echo "[EXP][T1][WARN] model_root not found: $MODEL_ROOT" >&2; MODEL_ROOT=""; fi

[[ -n "$MODEL_ROOT" ]] && export GAZEBO_MODEL_PATH="$MODEL_ROOT:${GAZEBO_MODEL_PATH:-}"
export MAP_FILE_OVERRIDE="$MAP_YAML"
export WORLD_OVERRIDE="$WORLD_FILE"
[[ -n "$NAV2_PARAMS" ]] && export NAV2_PARAMS_OVERRIDE="$NAV2_PARAMS"

echo "[EXP][T1] map_id=$MAP_ID planner=$PLANNER"
echo "[EXP][T1] plan =$PLAN_FILE"
echo "[EXP][T1] map  =$MAP_YAML"
echo "[EXP][T1] world=$WORLD_FILE"
[[ -n "$MODEL_ROOT" ]] && echo "[EXP][T1] models=$MODEL_ROOT"
[[ -n "$NAV2_PARAMS" ]] && echo "[EXP][T1] nav2_params=$NAV2_PARAMS"

bash "$WS/scripts/start_stack_static_tf_fixed.sh" "$PLAN_FILE" "$WITH_RVIZ" "$SPAWN_SEG_IDX"
