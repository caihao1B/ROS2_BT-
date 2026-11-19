#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace r10 {
using Pursuit = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

class DirectedPathCurvature : public Pursuit {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    Pursuit::configure(parent, name, tf, map);
    auto node = parent.lock();
    const auto parameter = [&](const std::string & suffix, double value) {
      auto key = name + "." + suffix;
      if (!node->has_parameter(key)) node->declare_parameter(key, value);
      return node->get_parameter(key).as_double();
    };
    max_k_ = parameter("vehicle_max_curvature", 0.0);
    lateral_gain_ = parameter("lateral_gain_per_m2", 1.0);
    heading_gain_ = parameter("heading_gain_per_m", 2.0);
    if (!(max_k_ > 0 && lateral_gain_ > 0 && heading_gain_ > 0))
      throw std::invalid_argument("Missing finite positive Ackermann tracking envelope");
  }

  void setPlan(const nav_msgs::msg::Path & path) override {
    plan_ = path;
    Pursuit::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override {
    // RPP still owns speed selection, progress, and online costmap/collision rejection.
    auto command = Pursuit::computeVelocityCommands(pose, velocity, checker);
    const double v = command.twist.linear.x;
    if (std::abs(v) < 0.001) return command;
    geometry_msgs::msg::PoseStamped robot;
    if (!path_handler_->transformPose(plan_.header.frame_id, pose, robot))
      throw nav2_core::ControllerTFError("Vehicle pose unavailable in connector frame");

    double nearest = std::numeric_limits<double>::infinity(), k = 0;
    int chosen_gear = 0;
    for (size_t i = 1; i < plan_.poses.size(); ++i) {
      const auto & a = plan_.poses[i-1].pose;
      const auto & b = plan_.poses[i].pose;
      const double x = b.position.x-a.position.x, y = b.position.y-a.position.y;
      const double length = std::hypot(x,y);
      if (length < 1.e-6) continue;
      const double dx = robot.pose.position.x-a.position.x, dy = robot.pose.position.y-a.position.y;
      const double fraction = std::clamp((x*dx+y*dy)/(length*length), 0., 1.);
      const double ex = dx-fraction*x, ey = dy-fraction*y;
      const double squared_distance = ex*ex+ey*ey;
      if (squared_distance >= nearest) continue;
      const double first = tf2::getYaw(a.orientation);
      const double angle = std::remainder(tf2::getYaw(b.orientation)-first, 2*M_PI);
      const double reference = first+fraction*angle;
      chosen_gear = x*std::cos(reference)+y*std::sin(reference)>0 ? 1 : -1;
      const double lateral = -std::sin(reference)*ex + std::cos(reference)*ey;
      const double heading = std::remainder(tf2::getYaw(robot.pose.orientation)-reference, 2*M_PI);
      // Signed curvature is d(vehicle heading)/d(signed rear-axle travel).
      // In reverse the heading feedback changes sign; the lateral term is unchanged.
      k = angle/(chosen_gear*length) - lateral_gain_*lateral - chosen_gear*heading_gain_*heading;
      nearest = squared_distance;
    }
    if (!std::isfinite(nearest) || chosen_gear*v <= 0)
      throw nav2_core::NoValidControl("Requested speed disagrees with directed connector");
    command.twist.angular.z = v*std::clamp(k,-max_k_,max_k_);
    // The safety decision must describe the velocity that leaves this controller, including
    // adjusted curvature. RPP's original costmap footprint and time horizon remain active.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    if (collision_checker_->isCollisionImminent(pose,v,command.twist.angular.z,1.2))
      throw nav2_core::NoValidControl("Projected corrected Ackermann footprint collides");
    return command;
  }

private:
  nav_msgs::msg::Path plan_;
  double max_k_ = 0., lateral_gain_ = 1., heading_gain_ = 2.;
};
}
PLUGINLIB_EXPORT_CLASS(r10::DirectedPathCurvature, nav2_core::Controller)
