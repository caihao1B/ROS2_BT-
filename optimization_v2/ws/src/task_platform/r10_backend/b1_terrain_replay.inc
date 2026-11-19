// Offline-only harness, compiled separately for both production terrain sources.
// No Gazebo process, navigation node, actuator, or live topic provides input.
#include <chrono>
#include <fstream>
#include <iostream>
#include <rcl/time.h>

int main(int argc, char** argv) {
  r08::initialize(argc, argv);
  auto node = std::make_shared<Terrain>();
  const auto manifest = J::parse(std::ifstream(std::getenv("B1_REPLAY_INPUT")));
  std::ofstream bytes(std::getenv("B1_REPLAY_OUTPUT"), std::ios::binary);
  if (!bytes) return 2;
  auto * clock = node->get_clock()->get_clock_handle();
  if (rcl_enable_ros_time_override(clock) != RCL_RET_OK) return 3;
  for (const auto& row : manifest) {
    auto ns = row.at("ns").get<int64_t>();
    if (rcl_set_ros_time_override(clock, ns) != RCL_RET_OK) return 4;
    const rclcpp::Time time(ns, RCL_ROS_TIME);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = time;
    const auto & p = row.at("odom");
    odom.pose.pose.position.x = p.at(0).get<double>();
    odom.pose.pose.position.y = p.at(1).get<double>();
    double yaw = p.at(2).get<double>();
    odom.pose.pose.orientation.z = std::sin(yaw*.5);
    odom.pose.pose.orientation.w = std::cos(yaw*.5);
    node->odom_.push_back(odom);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = time;
    cloud.header.frame_id = "lidar_link";
    cloud.height = 1;
    cloud.width = row.at("points").get<uint32_t>();
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step*cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);
    size_t offset = 0;
    for (std::string name : {"x", "y", "z"}) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      field.offset = offset;
      offset += sizeof(float);
      cloud.fields.push_back(field);
    }
    std::ifstream input(row.at("path").get<std::string>(), std::ios::binary);
    if (!input.read(reinterpret_cast<char*>(cloud.data.data()), cloud.data.size())) return 5;
    node->replay_bytes.clear();
    node->replay_messages.clear();
    auto before = std::chrono::steady_clock::now();
    node->cloud(cloud);
    auto elapsed = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count();
    if (node->replay_bytes.size() != 4*node->cells_.size() || !node->replay_messages.contains("health")) return 6;
    bytes.write(reinterpret_cast<const char*>(node->replay_bytes.data()), node->replay_bytes.size());
    bytes << node->replay_messages.dump() << '\n';
    std::cout << J({{"ns",ns},{"points",cloud.width},{"ms",elapsed},{"published_grid_bytes",node->replay_bytes.size()},
                    {"valid",node->replay_messages["health"].at("valid")}}).dump() << '\n';
  }
  node.reset();
  rclcpp::shutdown();
}
