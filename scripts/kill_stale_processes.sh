#!/usr/bin/env bash
set -e
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f nav2_container || true
pkill -f component_container || true
pkill -f static_transform_publisher || true
pkill -f coverage_supervisor || true
pkill -f coverage_executor || true
pkill -f coverage_executor_nobt_rejoin || true
pkill -f coverage_executor_guarded || true
pkill -f e1_runtime_logger.py || true
pkill -f "ros2 bag record" || true
pkill -f strategy_selector || true
pkill -f cv_predictor || true
pkill -f scan_tracker || true
pkill -f dynamic_obstacles || true
pkill -f ae_trigger_recorder.py || true
echo "[OK] stale Gazebo / Nav2 / BT processes cleaned."
