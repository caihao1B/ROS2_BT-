#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="crossing"
MAP_ID=""
PLANNER=""
PLAN_FILE=""
MAP_YAML=""
ASSET_INDEX=""
BOUNDS_SOURCE="plan"
BBOX_PAD_M="0.6"
MANUAL_BBOX=""
SEED=""
SEGMENTS=""
STARTUP_WAIT=""
PARAMS_FILE=""
EXTRA_PARAMS=()
STILL_FRAC="0.5"
STILL_SIZE_PROFILE="human"
STILL_NAME_PREFIX="still"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2;;
    --map-id) MAP_ID="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --plan-file) PLAN_FILE="$2"; shift 2;;
    --map-yaml) MAP_YAML="$2"; shift 2;;
    --asset-index) ASSET_INDEX="$2"; shift 2;;
    --bounds) BOUNDS_SOURCE="$2"; shift 2;;
    --bbox-pad) BBOX_PAD_M="$2"; shift 2;;
    --bbox) MANUAL_BBOX="$2"; shift 2;;
    --seed) SEED="$2"; shift 2;;
    --segments) SEGMENTS="$2"; shift 2;;
    --startup-wait) STARTUP_WAIT="$2"; shift 2;;
    --params-file) PARAMS_FILE="$2"; shift 2;;
    --obst-param) EXTRA_PARAMS+=("$2"); shift 2;;
    --still-frac) STILL_FRAC="$2"; shift 2;;
    --still-size-profile) STILL_SIZE_PROFILE="$2"; shift 2;;
    --still-name-prefix) STILL_NAME_PREFIX="$2"; shift 2;;
    *) echo "[EXP][OBS] unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$PARAMS_FILE" ]]; then
  case "$SCENARIO" in
    crossing) PARAMS_FILE="$WS/profiles/dynamic_obstacles_small_s1_crossing.yaml" ;;
    blocking) PARAMS_FILE="$WS/profiles/dynamic_obstacles_small_s2_blocking.yaml" ;;
    mixed) PARAMS_FILE="$WS/profiles/dynamic_obstacles_small_s3_mixed.yaml" ;;
    still) PARAMS_FILE="$WS/profiles/dynamic_obstacles_small_s4_still.yaml" ;;
    *) echo "[EXP][OBS] use --scenario {crossing|blocking|mixed|still} or provide --params-file" >&2; exit 2 ;;
  esac
fi

RESOLVE_ARGS=()
[[ -n "$ASSET_INDEX" ]] && RESOLVE_ARGS+=(--asset-index "$ASSET_INDEX")
[[ -n "$MAP_ID" ]] && RESOLVE_ARGS+=(--map-id "$MAP_ID")
[[ -n "$PLANNER" ]] && RESOLVE_ARGS+=(--planner "$PLANNER")
[[ -n "$PLAN_FILE" ]] && RESOLVE_ARGS+=(--plan-file "$PLAN_FILE")
[[ -n "$MAP_YAML" ]] && RESOLVE_ARGS+=(--map-yaml "$MAP_YAML")
ASSET_JSON=$(python3 "$WS/scripts/e1_common.py" resolve "${RESOLVE_ARGS[@]}")
readarray -t RESOLVED < <(python3 - <<'PY' "$ASSET_JSON"
import json,sys
x=json.loads(sys.argv[1])
for key in ('map_id','planner','plan_json','map_yaml'):
    print(x.get(key,''))
PY
)
MAP_ID="${RESOLVED[0]}"
PLANNER="${RESOLVED[1]}"
PLAN_FILE="${RESOLVED[2]}"
MAP_YAML="${RESOLVED[3]}"

if [[ ! -f "$PLAN_FILE" ]]; then echo "[EXP][OBS][ERROR] plan not found: $PLAN_FILE" >&2; exit 3; fi
if [[ ! -f "$PARAMS_FILE" ]]; then echo "[EXP][OBS][ERROR] params file not found: $PARAMS_FILE" >&2; exit 4; fi

AUTO_SCENARIO_FILE=""
if [[ "$SCENARIO" == "still" ]]; then
  if [[ -z "$SEGMENTS" ]]; then
    SEGMENTS='[0]'
  fi
  AUTO_SCENARIO_DIR="$WS/log/auto_scenarios"
  mkdir -p "$AUTO_SCENARIO_DIR"
  SAFE_TAG="$(echo "${MAP_ID}_${PLANNER}_${SEGMENTS}" | tr -cd '[:alnum:]_-' | cut -c1-80)"
  AUTO_SCENARIO_FILE="$AUTO_SCENARIO_DIR/${SAFE_TAG}_still.yaml"
  python3 "$WS/scripts/gen_still_obstacles_yaml.py" \
    --plan "$PLAN_FILE" \
    --segments "$SEGMENTS" \
    --frac "$STILL_FRAC" \
    --size-profile "$STILL_SIZE_PROFILE" \
    --name-prefix "$STILL_NAME_PREFIX" \
    --out "$AUTO_SCENARIO_FILE" >/dev/null
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
set -u

ARGS=(
  --ros-args
  --params-file "$PARAMS_FILE"
  -p plan_path:="$PLAN_FILE"
  -p bounds_source:="$BOUNDS_SOURCE"
  -p bbox_pad_m:="$BBOX_PAD_M"
  -p map_yaml_path:="$MAP_YAML"
  -p use_sim_time:=True
)
[[ -n "$SEED" ]] && ARGS+=( -p seed:="$SEED" )
[[ -n "$SEGMENTS" ]] && ARGS+=( -p suite_segment_indices:="$SEGMENTS" )
[[ -n "$STARTUP_WAIT" ]] && ARGS+=( -p startup_wait_sec:="$STARTUP_WAIT" )
[[ -n "$AUTO_SCENARIO_FILE" ]] && ARGS+=( -p scenario_file:="$AUTO_SCENARIO_FILE" )
if [[ -n "$MANUAL_BBOX" ]]; then
  ARGS+=( -p bounds_source:=manual )
  ARGS+=( -p bbox:="$MANUAL_BBOX" )
fi
for kv in "${EXTRA_PARAMS[@]}"; do
  ARGS+=( -p "$kv" )
done

echo "[EXP][OBS] map_id=$MAP_ID planner=$PLANNER scenario=$SCENARIO"
echo "[EXP][OBS] plan=$PLAN_FILE"
echo "[EXP][OBS] map_yaml=$MAP_YAML"
echo "[EXP][OBS] params=$PARAMS_FILE"
[[ -n "$AUTO_SCENARIO_FILE" ]] && echo "[EXP][OBS] scenario_file=$AUTO_SCENARIO_FILE"
ros2 run coverage_sim dynamic_obstacles "${ARGS[@]}"
