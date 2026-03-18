#!/usr/bin/env bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -e
set +u

PLAN_ARG="${1:-plan_micro_gate_bt_4seg.json}"
WITH_RVIZ="${2:-1}"
SPAWN_SEG_IDX="${3:-0}"

# ---- sanitize seg idx (fix '"0"' cases) ----
SPAWN_SEG_IDX="$(echo "$SPAWN_SEG_IDX" | tr -d '"' | tr -d "'" )"
if ! [[ "$SPAWN_SEG_IDX" =~ ^[0-9]+$ ]]; then
  echo "[WARN] spawn_seg_idx invalid: '$SPAWN_SEG_IDX', fallback to 0"
  SPAWN_SEG_IDX="0"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "[INFO] Killing old Gazebo/Nav2/RViz processes..."
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 gazebo || true
pkill -9 nav2_ || true
pkill -9 component_container || true
pkill -9 robot_state_publisher || true
pkill -9 spawn_entity.py || true
pkill -9 rviz2 || true
sleep 1

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

export TURTLEBOT3_MODEL=waffle
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_DATABASE_URI=""

FARM="$(ros2 pkg prefix farm_gate_world)/share/farm_gate_world"
TB3="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo"
BRINGUP_SHARE="$(ros2 pkg prefix farm_gate_bringup)/share/farm_gate_bringup"

WORLD="${WORLD_OVERRIDE:-$FARM/worlds/farm_field.world}"
MAP_FILE="${MAP_FILE_OVERRIDE:-$FARM/maps/map.yaml}"

# ---- allow override nav2 params ----
DEFAULT_PARAMS="$WS/config/nav2_params_farm_gate_sim_dyn_costmap_mppi_shadowfix.yaml"
NAV2_PARAMS="${NAV2_PARAMS_OVERRIDE:-$DEFAULT_PARAMS}"
if [ ! -f "$NAV2_PARAMS" ]; then
  echo "[ERROR] NAV2 params not found: $NAV2_PARAMS"
  exit 2
fi
echo "[INFO] World: $WORLD"
echo "[INFO] Map:   $MAP_FILE"
echo "[INFO] Nav2 params: $NAV2_PARAMS"

RVIZ_CFG="$BRINGUP_SHARE/rviz/farm_gate_coverage.rviz"

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
  exit 1
fi
echo "[INFO] Using plan: $PLAN_FILE"

# Gazebo paths
GAZEBO_MODELS="$FARM/models:$TB3/models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODELS:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"

# ---- parse spawn pose robustly ----
echo "[INFO] Parsing spawn pose (seg_idx=$SPAWN_SEG_IDX)"
read -r SPAWN_X SPAWN_Y SPAWN_YAW <<<"$(PLAN="$PLAN_FILE" SEG_IDX="$SPAWN_SEG_IDX" python3 - <<'PY'
import os, json, math, sys
plan_path=os.environ["PLAN"]
seg_raw=os.environ.get("SEG_IDX","0")
seg_raw=seg_raw.strip().strip('"').strip("'")
try:
    seg_idx=int(seg_raw)
except Exception:
    seg_idx=0
plan=json.load(open(plan_path,"r",encoding="utf-8"))
segs=plan.get("segments",[])
seg_idx=max(0, min(seg_idx, len(segs)-1)) if segs else 0
try:
    p=segs[seg_idx]["poses"][0]
    x=float(p.get("x",0.0)); y=float(p.get("y",0.0))
    if "qz" in p and "qw" in p:
        qz=float(p.get("qz",0.0)); qw=float(p.get("qw",1.0))
        yaw=math.atan2(2.0*(qw*qz), 1.0-2.0*(qz*qz))
    else:
        yaw=float(p.get("yaw",0.0))
except Exception:
    x=y=yaw=0.0
print(f"{x} {y} {yaw}")
PY
)"
# fallback if empty
SPAWN_X="${SPAWN_X:-0.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"
echo "[INFO] Spawn: x=$SPAWN_X y=$SPAWN_Y yaw=$SPAWN_YAW"

cleanup() {
  echo ""
  echo "[INFO] Cleanup..."
  kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---- Launch Gazebo ----
echo "[INFO] Launching gzserver... logs: /tmp/gzserver_farm_gate.log"
gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  "$WORLD" >/tmp/gzserver_farm_gate.log 2>&1 &
sleep 1

echo "[INFO] Launching gzclient... logs: /tmp/gzclient_farm_gate.log"
gzclient >/tmp/gzclient_farm_gate.log 2>&1 &

# ---- wait /clock topic exists (no QoS hang) ----
echo "[INFO] Waiting /clock topic..."
for i in $(seq 1 80); do
  if ros2 topic list 2>/dev/null | grep -qx "/clock"; then
    echo "[INFO] /clock topic exists."
    break
  fi
  sleep 0.25
done
if ! ros2 topic list 2>/dev/null | grep -qx "/clock"; then
  echo "[ERROR] /clock topic not found. gzserver likely failed."
  tail -n 80 /tmp/gzserver_farm_gate.log || true
  exit 10
fi

echo "[INFO] Waiting /spawn_entity service..."
for i in $(seq 1 80); do
  if ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
    echo "[INFO] /spawn_entity OK"
    break
  fi
  sleep 0.25
done
if ! ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
  echo "[ERROR] /spawn_entity not found."
  tail -n 80 /tmp/gzserver_farm_gate.log || true
  exit 11
fi

echo "[INFO] Launching robot_state_publisher..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True >/tmp/rsp_farm_gate.log 2>&1 &
sleep 1

echo "[INFO] Spawning TurtleBot3..."
timeout 60 ros2 run gazebo_ros spawn_entity.py \
  -entity waffle \
  -file "$TB3/models/turtlebot3_waffle/model.sdf" \
  -x "$SPAWN_X" -y "$SPAWN_Y" -z 0.05 -Y "$SPAWN_YAW" \
  -timeout 30 || {
    echo "[ERROR] spawn_entity failed."
    tail -n 80 /tmp/gzserver_farm_gate.log || true
    exit 12
  }
echo "[INFO] spawn_entity OK"

echo ""
echo "[NEXT] Start Terminal-2 (static TF map->odom):"
echo "  source $WS/scripts/cov_source.bash"
echo "  covcd"
echo "  bash scripts/t2_freeze_map_odom_manual.sh \"$PLAN_FILE\" $SPAWN_SEG_IDX"
echo ""

echo "[INFO] Waiting for TF map->odom (timeout ~120s)..."
for i in $(seq 1 240); do
  if python3 "$WS/scripts/check_tf_ready.py" --target map --source odom --timeout 0.8 >/dev/null 2>&1; then
    echo "[INFO] map->odom TF ready."
    break
  fi
  sleep 0.5
done
if ! python3 "$WS/scripts/check_tf_ready.py" --target map --source odom --timeout 0.8 >/dev/null 2>&1; then
  echo "[ERROR] TF map->odom not available. Run Terminal-2 first."
  exit 15
fi
echo "[INFO] Launching map_server..."
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=True \
  -p yaml_filename:="$MAP_FILE" >/tmp/map_server_farm_gate.log 2>&1 &

echo "[INFO] Launching lifecycle_manager for map_server..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -r __node:=lifecycle_manager_map \
  -p use_sim_time:=True \
  -p autostart:=True \
  -p node_names:="[map_server]" >/tmp/lifecycle_map_farm_gate.log 2>&1 &

echo "[INFO] Launching Nav2 navigation (NO AMCL)..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:="$NAV2_PARAMS" \
  autostart:=True >/tmp/nav2_nav_farm_gate.log 2>&1 &

if [ "$WITH_RVIZ" = "1" ]; then
  echo "[INFO] Launching RViz..."
  rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=True &
fi

echo ""
echo "[INFO] Stack is up (STATIC TF, no AMCL)."
echo "  - Plan:   $PLAN_FILE"
echo "  - Params: $NAV2_PARAMS"
echo "  - Logs:   /tmp/gzserver_farm_gate.log  /tmp/nav2_nav_farm_gate.log"
echo ""
wait
