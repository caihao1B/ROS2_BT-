#pragma once
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <chrono>
#include <thread>

namespace r08 {
// A signal requests an executor stop; it never destroys the ROS context while
// a PCL callback is publishing. The main thread finishes the callback first.
inline volatile std::sig_atomic_t stop_requested = 0;
inline void request_stop(int) { stop_requested = 1; }
inline void initialize(int argc, char **argv) {
  stop_requested = 0;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
}
inline void spin(const std::shared_ptr<rclcpp::Node> &node) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.remove_node(node);
}
}  // namespace r08
