#!/usr/bin/env bash
# Robust T2: publish static TF map->odom using tf2 lookup
# Hotfixes:
#   - avoid stdout/stderr pollution when capturing tx ty yaw
#   - avoid TransformListener background-thread shutdown exception
#   - tolerate shells where nounset was previously enabled

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -eo pipefail
set +u

PLAN_ARG="${1:-plan_micro_gate_bt_4seg.json}"
SEG_IDX="${2:-0}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"

source /opt/ros/humble/setup.bash
export COLCON_TRACE="${COLCON_TRACE:-}"
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "[T2][WARN] $WS/install/setup.bash not found. Did you colcon build?"
fi

PLAN_SRC="$WS/src/farm_gate_bringup/plans"
BRINGUP_SHARE="$(ros2 pkg prefix farm_gate_bringup)/share/farm_gate_bringup"
PLAN_SHARE="$BRINGUP_SHARE/plans"

if [ -f "$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_ARG"
elif [ -f "$PLAN_SRC/$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_SRC/$PLAN_ARG"
elif [ -f "$PLAN_SHARE/$PLAN_ARG" ]; then
  PLAN_FILE="$PLAN_SHARE/$PLAN_ARG"
else
  echo "[T2][ERROR] Plan file not found: $PLAN_ARG"
  echo "  - src  : $PLAN_SRC/$PLAN_ARG"
  echo "  - share: $PLAN_SHARE/$PLAN_ARG"
  exit 1
fi

echo "[T2] PLAN_FILE=$PLAN_FILE seg_idx=$SEG_IDX base_frame=$BASE_FRAME"
echo "[T2] Waiting TF odom->$BASE_FRAME ..."

TMP_RESULT="$(mktemp)"
TMP_LOG="$(mktemp)"
cleanup() {
  rm -f "$TMP_RESULT" "$TMP_LOG"
}
trap cleanup EXIT

PLAN_FILE="$PLAN_FILE" SEG_IDX="$SEG_IDX" BASE_FRAME="$BASE_FRAME" RESULT_FILE="$TMP_RESULT" \
python3 - <<'PY' >"$TMP_LOG" 2>&1
import os
import json
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.time import Time
import tf2_ros


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


plan_file = os.environ["PLAN_FILE"]
seg_raw = os.environ.get("SEG_IDX", "0").strip().strip('"').strip("'")
base = os.environ.get("BASE_FRAME", "base_footprint")
result_file = os.environ["RESULT_FILE"]

with open(plan_file, "r", encoding="utf-8") as f:
    plan = json.load(f)

segs = plan.get("segments", [])
if not segs:
    print("[T2][PY][ERROR] No segments in plan", file=sys.stderr)
    raise SystemExit(2)

seg_idx = int(seg_raw) if seg_raw else 0
seg_idx = max(0, min(seg_idx, len(segs) - 1))
poses = segs[seg_idx].get("poses", [])
if not poses:
    print(f"[T2][PY][ERROR] Segment {seg_idx} has no poses", file=sys.stderr)
    raise SystemExit(2)

p = poses[0]
xm = float(p.get("x", 0.0))
ym = float(p.get("y", 0.0))

if "qz" in p and "qw" in p:
    qz = float(p["qz"])
    qw = float(p["qw"])
    yawm = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
else:
    yawm = float(p.get("yaw", 0.0))

rclpy.init(args=None)
node = Node("t2_tf_lookup")

try:
    node.declare_parameter("use_sim_time", True)
except ParameterAlreadyDeclaredException:
    pass
except Exception:
    pass

try:
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
except Exception:
    pass

buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, node, spin_thread=False)

tf = None
deadline = time.time() + 12.0
while time.time() < deadline:
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except Exception:
        pass
    try:
        tf = buf.lookup_transform("odom", base, Time())
        break
    except Exception:
        time.sleep(0.05)

if tf is None:
    print(f"[T2][PY][ERROR] TF lookup failed: odom -> {base}", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

xo = float(tf.transform.translation.x)
yo = float(tf.transform.translation.y)
q = tf.transform.rotation
yawo = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

yaw = (yawm - yawo + math.pi) % (2.0 * math.pi) - math.pi
c = math.cos(yaw)
s = math.sin(yaw)
tx = xm - (c * xo - s * yo)
ty = ym - (s * xo + c * yo)

with open(result_file, "w", encoding="utf-8") as f:
    f.write(f"{tx} {ty} {yaw}\n")

node.destroy_node()
rclpy.shutdown()
PY

if [ ! -s "$TMP_RESULT" ]; then
  echo "[T2][ERROR] Failed to compute static map->odom."
  echo "[T2][ERROR] Python log follows:"
  cat "$TMP_LOG"
  exit 3
fi

read -r MX MY MYAW < "$TMP_RESULT"

if [ -z "${MX:-}" ] || [ -z "${MY:-}" ] || [ -z "${MYAW:-}" ]; then
  echo "[T2][ERROR] Empty numeric result."
  echo "[T2][ERROR] Python log follows:"
  cat "$TMP_LOG"
  exit 4
fi

echo "[T2] static map->odom: x=$MX y=$MY yaw=$MYAW"
echo "[T2] IMPORTANT: keep this terminal running (Ctrl+C to stop)."

ros2 run tf2_ros static_transform_publisher \
  --x "$MX" --y "$MY" --z 0 \
  --yaw "$MYAW" --pitch 0 --roll 0 \
  --frame-id map --child-frame-id odom \
  --ros-args -p use_sim_time:=True
