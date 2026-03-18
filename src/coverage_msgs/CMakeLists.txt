cmake_minimum_required(VERSION 3.8)
project(coverage_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleTrack.msg"
  "msg/ObstacleTrackArray.msg"
  "msg/ObstaclePrediction.msg"
  "msg/ObstaclePredictionArray.msg"
  "msg/StrategyDecision.msg"
  "msg/StrategyDecisionStamped.msg"
  "srv/ComputeDetour.srv"
  "srv/D1Evaluate.srv"
  "srv/D2Replan.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
