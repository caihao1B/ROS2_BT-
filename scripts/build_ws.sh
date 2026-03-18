#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
rm -rf build install log
colcon build --symlink-install --packages-up-to coverage_msgs coverage_metrics coverage_perception coverage_prediction coverage_replanner coverage_bt_supervisor coverage_decision coverage_manager coverage_sim farm_gate_world farm_gate_bringup
