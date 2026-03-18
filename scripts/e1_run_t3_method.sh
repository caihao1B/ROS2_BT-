#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
METHOD="${METHOD:-}"
MAP_ID=""
PLANNER=""
PLAN_FILE=""
MAP_YAML=""
WORLD_FILE=""
ASSET_INDEX=""
OUT_DIR=""
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-0}"
TOOL_RADIUS_M="${TOOL_RADIUS_M:-0.20}"
BASE_FRAME="${BASE_FRAME:-base_link}"
RECORD_BAG="${E1_RECORD_BAG:-0}"
LOGGER_DELAY_SEC="${LOGGER_DELAY_SEC:-1.5}"
SCENARIO_TAG="${SCENARIO_TAG:-nominal}"
NOTES="${RUN_NOTES:-}"
ENABLE_RESOURCE_MONITOR="${E1_ENABLE_RESOURCE_MONITOR:-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) METHOD="$2"; shift 2;;
    --map-id) MAP_ID="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --plan-file) PLAN_FILE="$2"; shift 2;;
    --map-yaml) MAP_YAML="$2"; shift 2;;
    --world-file) WORLD_FILE="$2"; shift 2;;
    --asset-index) ASSET_INDEX="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --timeout-sec) RUN_TIMEOUT_SEC="$2"; shift 2;;
    --tool-radius) TOOL_RADIUS_M="$2"; shift 2;;
    --base-frame) BASE_FRAME="$2"; shift 2;;
    --record-bag) RECORD_BAG=1; shift 1;;
    --scenario-tag) SCENARIO_TAG="$2"; shift 2;;
    --notes) NOTES="$2"; shift 2;;
    --enable-resource-monitor) ENABLE_RESOURCE_MONITOR=1; shift 1;;
    --disable-resource-monitor) ENABLE_RESOURCE_MONITOR=0; shift 1;;
    *) echo "[EXP][T3] unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$METHOD" ]]; then echo "[EXP][T3][ERROR] --method required" >&2; exit 2; fi

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
for key in ('map_id','planner','plan_json','map_yaml','world_file'):
    print(x.get(key,''))
PY
)
MAP_ID="${RESOLVED[0]}"
PLANNER="${RESOLVED[1]}"
PLAN_FILE="${RESOLVED[2]}"
MAP_YAML="${RESOLVED[3]}"
WORLD_FILE="${RESOLVED[4]}"

if [[ ! -f "$PLAN_FILE" ]]; then echo "[EXP][T3][ERROR] plan not found: $PLAN_FILE" >&2; exit 3; fi

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="$WS/runs/e1/$METHOD/$PLANNER/$MAP_ID/$STAMP"
fi
mkdir -p "$OUT_DIR"
PIPE_LOG="$OUT_DIR/pipeline.log"
LOGGER_LOG="$OUT_DIR/logger.log"
BAG_LOG="$OUT_DIR/bag.log"
BAG_PID=""
PIPE_PID=""

cleanup() {
  set +e
  if [[ -n "$BAG_PID" ]]; then
    kill "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$PIPE_PID" ]]; then
    kill -INT "$PIPE_PID" 2>/dev/null || true
    sleep 2
    pkill -P "$PIPE_PID" 2>/dev/null || true
    kill "$PIPE_PID" 2>/dev/null || true
    wait "$PIPE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

__restore_u=0
if [[ $- == *u* ]]; then
  __restore_u=1
  set +u
fi
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}"
export COLCON_TRACE="${COLCON_TRACE-}"
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
if [[ "$__restore_u" == "1" ]]; then
  set -u
fi
unset __restore_u

if [[ "$METHOD" == "nav2_base" ]]; then
  EXECUTABLE="${E1_BASELINE_EXECUTABLE:-coverage_executor_nobt_rejoin}"
  ros2 run coverage_manager "$EXECUTABLE" --ros-args \
    --params-file "$WS/src/coverage_manager/params/executor_params.yaml" \
    -p plan_path:="$PLAN_FILE" \
    -p use_sim_time:=True >"$PIPE_LOG" 2>&1 &
  PIPE_PID=$!
elif [[ "$METHOD" == "bt_full" || "$METHOD" == "bt_none" ]]; then
  BT_MODE="FULL"
  if [[ "$METHOD" == "bt_none" ]]; then
    BT_MODE="NONE"
  fi
  bash "$WS/scripts/run_dynamic_bt_suite_mppi.sh" \
    --mode "$BT_MODE" \
    --scenario agri_suite \
    --plan "$PLAN_FILE" \
    --no-obstacles >"$PIPE_LOG" 2>&1 &
  PIPE_PID=$!
else
  echo "[EXP][T3][ERROR] unsupported method: $METHOD" >&2
  exit 4
fi

sleep "$LOGGER_DELAY_SEC"

if [[ "$RECORD_BAG" == "1" ]]; then
  ros2 bag record -o "$OUT_DIR/bag" \
    /tf /tf_static /odom /cmd_vel /map \
    /global_costmap/costmap_raw /local_costmap/costmap_raw \
    /coverage/events /coverage/recovery_event /coverage/strategy_decision /coverage/current_path \
    >"$BAG_LOG" 2>&1 &
  BAG_PID=$!
fi

LOGGER_CMD=(python3 "$WS/scripts/e1_runtime_logger.py"
  --output-dir "$OUT_DIR"
  --method "$METHOD"
  --map-id "$MAP_ID"
  --planner "$PLANNER"
  --scenario-tag "$SCENARIO_TAG"
  --notes "$NOTES"
  --plan-json "$PLAN_FILE"
  --map-yaml "$MAP_YAML"
  --world-file "$WORLD_FILE"
  --tool-radius "$TOOL_RADIUS_M"
  --base-frame "$BASE_FRAME"
  --auto-exit-on-complete)

if [[ "$ENABLE_RESOURCE_MONITOR" == "1" ]]; then
  LOGGER_CMD+=(--enable-resource-monitor)
fi

set +e
if [[ "$RUN_TIMEOUT_SEC" != "0" ]]; then
  timeout "${RUN_TIMEOUT_SEC}s" "${LOGGER_CMD[@]}" >"$LOGGER_LOG" 2>&1
  LOGGER_RC=$?
else
  "${LOGGER_CMD[@]}" >"$LOGGER_LOG" 2>&1
  LOGGER_RC=$?
fi
set -e

echo "[EXP][T3] method=$METHOD map_id=$MAP_ID planner=$PLANNER scenario=$SCENARIO_TAG out=$OUT_DIR logger_rc=$LOGGER_RC"
exit 0
