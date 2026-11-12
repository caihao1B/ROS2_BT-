#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("s01_isolation_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("/s01/isolation_probe", 10);
  auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
    std_msgs::msg::String msg;
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    msg.data = std::string("S01_DOMAIN_") + (domain ? domain : "UNSET");
    publisher->publish(msg);
  });
  auto finish = node->create_wall_timer(std::chrono::seconds(10), []() { rclcpp::shutdown(); });
  rclcpp::spin(node);
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
