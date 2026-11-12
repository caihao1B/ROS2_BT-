// Smac computes geometry from the live observed map. Geometry is fixed in map;
// a measurement timestamp must not freeze its later map->odom conversion.
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include "startup_clock.hpp"
#include "exact_goal.hpp"
namespace s13 {
class StaticMapPlanner : public nav2_core::GlobalPlanner {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{
    "nav2_core", "nav2_core::GlobalPlanner"};
  nav2_core::GlobalPlanner::Ptr delegate_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    waitForRestartClock(parent.lock(), costmap);
    if (costmap->getGlobalFrameID() != "map") {
      throw std::runtime_error("STATIC_PLAN_REQUIRES_MAP_FRAME");
    }
    delegate_ = loader_.createSharedInstance("nav2_smac_planner::SmacPlanner2D");
    costmap_ = costmap;
    delegate_->configure(parent, name, tf, costmap);
  }
  void activate() override {delegate_->activate();}
  void deactivate() override {delegate_->deactivate();}
  void cleanup() override {delegate_->cleanup();delegate_.reset();costmap_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel) override {
    auto path = delegate_->createPlan(start, goal, cancel);
    if (path.header.frame_id != "map") {
      throw std::runtime_error("UNEXPECTED_PLAN_FRAME");
    }
    // Smac 2D returns grid centers. Keep the requested service position when
    // the short final correction is safe in the actual current costmap.
    if (!path.poses.empty()) {
      const auto point = [](const geometry_msgs::msg::Pose & p) {
        const auto & q=p.orientation;
        return PlanPoint{p.position.x,p.position.y,
          std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
      };
      auto * map=costmap_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
      const auto footprint=costmap_->getRobotFootprint();
      const auto safe=[&](double x,double y,double yaw) {
        unsigned int mx,my;
        if (!map->worldToMap(x,y,mx,my) || map->getCost(mx,my)>=253) {return false;}
        const double cost=checker.footprintCostAtPose(x,y,yaw,footprint);
        return cost>=0. && cost<253.;
      };
      const auto & previous=path.poses[path.poses.size()>1?path.poses.size()-2:0];
      if (canUseExactGoal(point(previous.pose),point(path.poses.back().pose),
        point(goal.pose),map->getResolution(),safe)) {path.poses.back().pose=goal.pose;}
    }
    path.header.stamp = builtin_interfaces::msg::Time();
    for (auto & pose : path.poses) {pose.header = path.header;}
    return path;
  }
};
}
PLUGINLIB_EXPORT_CLASS(s13::StaticMapPlanner, nav2_core::GlobalPlanner)
