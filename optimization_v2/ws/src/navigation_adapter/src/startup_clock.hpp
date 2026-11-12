// Hot-restart barrier: use actual node clocks, never manufacture a /clock tick.
#pragma once
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <memory>
#include <stdexcept>
#include <string>

namespace s13 {
inline void waitForRestartClock(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
{
  const std::string parameter = "s13_startup_sim_time_floor";
  if (!node->has_parameter(parameter)) {node->declare_parameter(parameter, 0.0);}
  const double floor = node->get_parameter(parameter).as_double();
  if (!std::isfinite(floor) || floor < 0.0) {
    throw std::runtime_error("INVALID_STARTUP_CLOCK_FLOOR");
  }
  if (floor == 0.0) {return;}  // Original cold-start path remains unchanged.
  RCLCPP_INFO(node->get_logger(), "STARTUP_CLOCK_WAIT floor=%.6f node=%.6f costmap=%.6f",
    floor, node->now().seconds(), costmap->now().seconds());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok()) {
    const double node_time = node->now().seconds();
    const double costmap_time = costmap->now().seconds();
    if (node_time >= floor && costmap_time >= floor && std::abs(node_time-costmap_time) < 0.1) {
      RCLCPP_INFO(node->get_logger(), "STARTUP_CLOCK_READY floor=%.6f node=%.6f costmap=%.6f",
        floor, node_time, costmap_time);
      return;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("STARTUP_CLOCK_NOT_RECEIVED");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("STARTUP_CLOCK_INTERRUPTED");
}
}
