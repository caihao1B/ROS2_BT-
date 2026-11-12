// Standalone teaching example, not a ROS node or production supervisor.
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

struct Pose2 { double x, y, yaw; };

Pose2 compose(Pose2 a, Pose2 b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y, a.yaw+b.yaw};
}
Pose2 inverse(Pose2 a) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-c*a.x-s*a.y, s*a.x-c*a.y, -a.yaw};
}
struct Gate { double distance, nis; bool accepted; };
Gate gate(double dx, double dy, double sigma) {
  const double distance = std::hypot(dx,dy);
  const double nis = (dx*dx+dy*dy)/(sigma*sigma+0.0025);
  return {distance, nis, distance <= 0.15 && nis <= 9.21};
}
int main() {
  const Pose2 odom_base{2,1,0.1}, map_odom{0.4,-0.2,0.17453292519943295};
  const auto map_base=compose(map_odom, odom_base);
  const auto recovered=compose(map_base, inverse(odom_base));
  assert(std::abs(recovered.x-map_odom.x)<1e-12);
  assert(std::abs(recovered.y-map_odom.y)<1e-12);
  assert(std::abs(recovered.yaw-map_odom.yaw)<1e-12);
  const auto rejected=gate(0.16,0,0.04);
  assert(rejected.nis<9.21 && !rejected.accepted);
  assert(gate(0.10,0,0.04).accepted);
  std::cout << std::fixed << std::setprecision(6)
            << "SYNTHETIC_TEACHING_ONLY\n"
            << "map->base: " << map_base.x << ", " << map_base.y << ", " << map_base.yaw << '\n'
            << "recovered map->odom: " << recovered.x << ", " << recovered.y << ", " << recovered.yaw << '\n'
            << "16cm GNSS residual NIS=" << rejected.nis << "; distance gate rejects\n"
            << "PASS: transform composition and innovation gates\n";
}
