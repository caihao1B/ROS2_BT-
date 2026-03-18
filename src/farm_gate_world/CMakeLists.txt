cmake_minimum_required(VERSION 3.8)
project(farm_gate_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    maps
    worlds
    models
    config
    data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
