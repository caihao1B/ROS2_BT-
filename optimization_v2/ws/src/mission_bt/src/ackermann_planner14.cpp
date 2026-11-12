// Keep a current collision-checked path stable; otherwise delegate to Dubins
// Hybrid-A*. Map geometry uses latest-TF stamps as required by the S13 adapter.
#include <nav2_core/global_planner.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "startup_clock.hpp"
#include "path_retention.hpp"
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
namespace s14 {
class AckermannPlanner : public nav2_core::GlobalPlanner {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{
    "nav2_core", "nav2_core::GlobalPlanner"};
  nav2_core::GlobalPlanner::Ptr delegate_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr decisions_;
  nav_msgs::msg::Path previous_;
  geometry_msgs::msg::Pose previous_goal_;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    s13::waitForRestartClock(parent.lock(), costmap);
    if (costmap->getGlobalFrameID() != "map") {
      throw std::runtime_error("ACKERMANN_PLAN_REQUIRES_MAP_FRAME");
    }
    delegate_=loader_.createSharedInstance("nav2_smac_planner::SmacPlannerHybrid");
    delegate_->configure(parent,name,tf,costmap);
    costmap_=costmap;parent_=parent;
    decisions_=parent.lock()->create_publisher<std_msgs::msg::String>("/s14/navigation_decision",10);
  }
  void activate() override {delegate_->activate();decisions_->on_activate();}
  void deactivate() override {delegate_->deactivate();decisions_->on_deactivate();previous_.poses.clear();}
  void cleanup() override {delegate_->cleanup();delegate_.reset();costmap_.reset();decisions_.reset();previous_.poses.clear();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,std::function<bool()> cancel) override {
    if(cancel())throw nav2_core::PlannerCancelled("S14_CANCELLED_BEFORE_PATH_CHECK");
    Retention check;
    const bool same_goal=std::hypot(previous_goal_.position.x-goal.pose.position.x,previous_goal_.position.y-goal.pose.position.y)<1e-6&&std::abs(angle(heading(previous_goal_)-heading(goal.pose)))<1e-6;
    if(!previous_.poses.empty()&&same_goal){
      auto *map=costmap_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      check=retain(previous_,start.pose,goal.pose,*map,costmap_->getRobotFootprint(),costmap_->isCurrent());
    }else if(!previous_.poses.empty())check.reason="GOAL_CHANGED";
    if(cancel())throw nav2_core::PlannerCancelled("S14_CANCELLED_AFTER_PATH_CHECK");
    std_msgs::msg::String event;event.data=nlohmann::json({{"t",parent_.lock()->now().seconds()},{"reason_code",check.reason},{"action",check.valid?"RETAIN":"REPLAN"},{"checked_footprints",check.checked},{"path_distance_m",check.distance},{"path_yaw_error_rad",check.yaw_error},{"distance_limit_m",.25},{"yaw_limit_rad",.1},{"unknown_rejected",true}}).dump();decisions_->publish(event);
    nav_msgs::msg::Path path;
    if(check.valid){path=previous_;path.poses.erase(path.poses.begin(),path.poses.begin()+check.first);}
    else path=delegate_->createPlan(start,goal,cancel);
    if(path.header.frame_id!="map")throw std::runtime_error("UNEXPECTED_PLAN_FRAME");
    path.header.stamp=builtin_interfaces::msg::Time();
    for(auto & pose:path.poses)pose.header=path.header;
    previous_=path;previous_goal_=goal.pose;
    return path;
  }
};
}
PLUGINLIB_EXPORT_CLASS(s14::AckermannPlanner,nav2_core::GlobalPlanner)
