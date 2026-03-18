#!/usr/bin/env bash
set -eo pipefail

# Run the dynamic-BT pipeline components (NOT Gazebo/Nav2).
# Assumes Gazebo + Nav2 already running (e.g. via start_stack_dyn_costmap_mppi.sh).
#
# This script is designed for the C/D/E trigger tests: keep Gazebo+Nav2 and static TF in
# separate terminals, everything else can be started here.
#
# Usage examples:
#   # BT pipeline only (no obstacles):
#   bash scripts/run_dynamic_bt_suite_mppi.sh --mode FULL --scenario agri_suite --plan plan_gate_exec.json --record --no-obstacles
#
#   # Obstacles only (run in a separate terminal; you can start it later to avoid "spawn撞飞"):
#   bash scripts/run_dynamic_bt_suite_mppi.sh --only-obstacles --scenario agri_suite --plan plan_gate_exec.json \
#     --bounds manual --bbox "[0.0, 0.0, 40.0, 30.0]" \
#     --obst-param startup_wait_sec:=15.0 --obst-param agri_initial_delay_max_sec:=20.0
#
# Modes: NONE, A, B, C, D, E, FULL
# Scenarios (params file base name without ".yaml"):
#   case_A_crossing, case_B_parallel, case_C_blocking, agri_suite

MODE="C"
SCENARIO="case_C_blocking"
PLAN_ARG="plan_micro_gate_bt_4seg.json"
PRED_CFG="config/cv_predictor_h12.yaml"
RECORD=0

# Dynamic obstacle spawn bounds (important for quick / stable triggers)
BOUNDS_SOURCE="plan"      # plan | manual | map_yaml
BBOX_PAD_M="0.6"

# Split control:
#   --no-obstacles     : do NOT start dynamic_obstacles in this script (run it in another terminal)
#   --only-obstacles   : start ONLY dynamic_obstacles (no BT pipeline)
RUN_OBSTACLES=1
ONLY_OBSTACLES=0

# Manual bbox override for dynamic_obstacles (forces bounds_source=manual)
# Example: --bbox "[0.0, 0.0, 40.0, 30.0]"
BBOX_MANUAL=""

# Extra obstacle params passthrough (repeatable)
# Example: --obst-param startup_wait_sec:=15.0 --obst-param agri_initial_delay_max_sec:=20.0
OBST_EXTRA_PARAMS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2;;
    --scenario) SCENARIO="$2"; shift 2;;
    --plan) PLAN_ARG="$2"; shift 2;;
    --pred) PRED_CFG="$2"; shift 2;;
    --record) RECORD=1; shift 1;;

    --bounds) BOUNDS_SOURCE="$2"; shift 2;;
    --bbox-pad) BBOX_PAD_M="$2"; shift 2;;

    --no-obstacles) RUN_OBSTACLES=0; shift 1;;
    --only-obstacles) ONLY_OBSTACLES=1; shift 1;;
    --bbox) BBOX_MANUAL="$2"; shift 2;;
    --obst-param) OBST_EXTRA_PARAMS+=("$2"); shift 2;;

    *) echo "[ERROR] Unknown arg: $1"; exit 2;;
  esac
done

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- ROS / colcon setup may reference unset vars; avoid nounset crash ----
set +u
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "[WARN] $WS/install/setup.bash not found. Did you colcon build?"
fi
set -u

# Resolve plan path (prefer explicit path, then src/, then install/share/)
BRINGUP_SHARE="$(ros2 pkg prefix farm_gate_bringup)/share/farm_gate_bringup"
PLAN_SRC="$WS/src/farm_gate_bringup/plans"
PLAN_SHARE="$BRINGUP_SHARE/plans"

if [ -f "$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_ARG"
elif [ -f "$PLAN_SRC/$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_SRC/$PLAN_ARG"
elif [ -f "$PLAN_SHARE/$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_SHARE/$PLAN_ARG"
else
  echo "[ERROR] Plan file not found: $PLAN_ARG"
  echo "  - src  : $PLAN_SRC/$PLAN_ARG"
  echo "  - share: $PLAN_SHARE/$PLAN_ARG"
  exit 1
fi

# Selector profile mapping
PROFILE=""
case "$MODE" in
  NONE) PROFILE="src/coverage_decision/params/profiles/selector_none.yaml";;
  A)    PROFILE="src/coverage_decision/params/profiles/selector_A.yaml";;
  B)    PROFILE="src/coverage_decision/params/profiles/selector_B.yaml";;
  C)    PROFILE="src/coverage_decision/params/profiles/selector_C.yaml";;
  D)    PROFILE="src/coverage_decision/params/profiles/selector_D_test.yaml";;
  E)    PROFILE="src/coverage_decision/params/profiles/selector_E_test.yaml";;
  FULL) PROFILE="src/coverage_decision/params/profiles/selector_FULL.yaml";;
  *) echo "[ERROR] Unknown MODE=$MODE (use NONE/A/B/C/D/E/FULL)"; exit 2;;
esac

SCENARIO_FILE="src/coverage_sim/params/dynamic_obstacles_${SCENARIO}.yaml"
if [ ! -f "$WS/$SCENARIO_FILE" ]; then
  echo "[ERROR] Scenario params not found: $WS/$SCENARIO_FILE"
  exit 1
fi

PRED_FILE="$WS/$PRED_CFG"
if [ ! -f "$PRED_FILE" ]; then
  echo "[ERROR] Predictor config not found: $PRED_FILE"
  exit 1
fi

echo "[INFO] Running dynamic BT suite:"
echo "  WS      : $WS"
echo "  MODE    : $MODE"
echo "  PROFILE : $PROFILE"
echo "  SCENARIO: $SCENARIO_FILE"
echo "  PLAN    : $PLAN_FILE"
echo "  PRED    : $PRED_FILE"
echo "  RECORD  : $RECORD"
echo "  OBST_BOUNDS: $BOUNDS_SOURCE (bbox_pad_m=$BBOX_PAD_M)"
echo "  RUN_OBSTACLES: $RUN_OBSTACLES"
echo "  ONLY_OBSTACLES: $ONLY_OBSTACLES"
if [ -n "$BBOX_MANUAL" ]; then
  echo "  BBOX_MANUAL: $BBOX_MANUAL"
fi
if [ "${#OBST_EXTRA_PARAMS[@]}" -gt 0 ]; then
  echo "  OBST_EXTRA_PARAMS: ${OBST_EXTRA_PARAMS[*]}"
fi

cleanup() {
  echo ""
  echo "[INFO] Cleaning up nodes..."
  kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$ONLY_OBSTACLES" = "0" ]; then
  # 1) scan_tracker
  ros2 run coverage_perception scan_tracker --ros-args \
      --params-file "$(ros2 pkg prefix coverage_perception)/share/coverage_perception/params/scan_tracker.yaml" \
      -p use_sim_time:=True &
  sleep 0.3

  # 2) cv_predictor (long horizon by default)
  ros2 run coverage_prediction cv_predictor --ros-args \
      --params-file "$PRED_FILE" \
      -p use_sim_time:=True &
  sleep 0.3

  # 3) region state layer (persistent evidence for D1/D2)
  ros2 run coverage_replanner region_state --ros-args \
      --params-file "$WS/src/coverage_replanner/params/region_state.yaml" \
      -p plan_path:="$PLAN_FILE" \
      -p use_sim_time:=True &
  sleep 0.3

  # 4) D1 advisor
  ros2 run coverage_replanner d1_advisor --ros-args \
      --params-file "$WS/src/coverage_replanner/params/d1_advisor.yaml" \
      -p use_sim_time:=True &
  sleep 0.3

  # 5) D2 advisor
  ros2 run coverage_replanner d2_advisor --ros-args \
      --params-file "$WS/src/coverage_replanner/params/d2_advisor.yaml" \
      -p use_sim_time:=True &
  sleep 0.3

  # 6) coverage_supervisor (BT-like executive)
  ros2 run coverage_bt_supervisor coverage_supervisor --ros-args \
      --params-file "$WS/src/coverage_bt_supervisor/params/supervisor.yaml" \
      -p plan_path:="$PLAN_FILE" \
      -p use_sim_time:=True &
  sleep 0.3

  # 7) strategy_selector
  ros2 run coverage_decision strategy_selector --ros-args \
      --params-file "$WS/$PROFILE" \
      -p use_sim_time:=True &
  sleep 0.3
fi

if [ "$RUN_OBSTACLES" = "1" ] || [ "$ONLY_OBSTACLES" = "1" ]; then
  # 5) dynamic obstacles (run directly so we can override bounds from CLI)
  OBST_ARGS=(
    --ros-args
    --params-file "$WS/$SCENARIO_FILE"
    -p plan_path:="$PLAN_FILE"
    -p bounds_source:="$BOUNDS_SOURCE"
    -p bbox_pad_m:="$BBOX_PAD_M"
    -p map_yaml_path:="$WS/src/farm_gate_world/maps/map.yaml"
    -p use_sim_time:=True
  )

  # If manual bbox is provided, force bounds_source=manual and pass bbox.
  if [ -n "$BBOX_MANUAL" ]; then
    OBST_ARGS+=( -p bounds_source:=manual )
    OBST_ARGS+=( -p bbox:="$BBOX_MANUAL" )
  fi

  # Passthrough extra obstacle params (repeatable)
  if [ "${#OBST_EXTRA_PARAMS[@]}" -gt 0 ]; then
    for kv in "${OBST_EXTRA_PARAMS[@]}"; do
      OBST_ARGS+=( -p "$kv" )
    done
  fi

  ros2 run coverage_sim dynamic_obstacles "${OBST_ARGS[@]}" &
  sleep 0.3
else
  echo "[INFO] Skipping dynamic obstacles (run in another terminal)."
fi

# 6) optional recorder (events + short bag clips)
if [ "$RECORD" = "1" ] && [ "$ONLY_OBSTACLES" = "0" ]; then
  TAG="${MODE}_${SCENARIO}_mppi_h12"
  python3 "$WS/scripts/ae_trigger_recorder.py" \
      --out-dir "$HOME/.ros/ae_events" \
      --tag "$TAG" \
      --bag \
      --bag-seconds 8 \
      --ros-args -p use_sim_time:=True &
fi

echo "[INFO] All nodes launched. Ctrl+C to stop."
wait
