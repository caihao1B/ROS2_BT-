#!/usr/bin/env bash
# Source ROS 2 + this workspace overlay + common env vars for the farm_gate coverage tests.
#
# Usage:
#   source <WS>/scripts/cov_source.bash
#
# Tip (auto-source on every new terminal):
#   echo 'source <WS>/scripts/cov_source.bash' >> ~/.bashrc

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 / overlay setup (temporarily relax nounset for ROS setup scripts)
__restore_u=0
if [[ $- == *u* ]]; then
  __restore_u=1
  set +u
fi

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}"
export COLCON_TRACE="${COLCON_TRACE-}"

source /opt/ros/humble/setup.bash

# Overlay (if built)
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "[cov_source] WARN: $WS/install/setup.bash not found. Did you run: colcon build --symlink-install ?"
fi

if [[ "$__restore_u" == "1" ]]; then
  set -u
fi
unset __restore_u

# Common env
export COV_WS="$WS"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

# Networking defaults (avoid "can't see /clock" or /odom between terminals)
# You can override these before sourcing this script.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Convenience aliases
alias covcd='cd "$COV_WS"'
alias covbuild='(cd "$COV_WS" && colcon build --symlink-install)'

# Experiment helpers
alias covt1='bash "$COV_WS/scripts/run_t1.sh"'
alias covt2='bash "$COV_WS/scripts/run_t2.sh"'
alias covt3full='bash "$COV_WS/scripts/run_t3_bt_full.sh"'
alias covt3none='bash "$COV_WS/scripts/run_t3_bt_none.sh"'
alias covt3base='bash "$COV_WS/scripts/run_t3_nav2_base.sh"'
alias covobs='bash "$COV_WS/scripts/run_obstacles.sh"'
alias covprofile='bash "$COV_WS/scripts/apply_profile.sh"'
