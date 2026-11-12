#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include "startup_clock.hpp"
#include <nav2_core/controller_exceptions.hpp>
namespace s13 {
class MeasuredController:public nav2_core::Controller {
 pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core","nav2_core::Controller"};
 nav2_core::Controller::Ptr implementation_;
 rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
 rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr metrics_;
 std::string name_,type_;
 std::string path_frame_,base_frame_;
 std::shared_ptr<tf2_ros::Buffer> tf_;
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&p,std::string name,std::shared_ptr<tf2_ros::Buffer>tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>cost)override{
  parent_=p;name_=name;tf_=tf;base_frame_=cost->getBaseFrameID();auto n=p.lock();waitForRestartClock(n,cost);if(!n->has_parameter(name+".implementation"))n->declare_parameter(name+".implementation",std::string(""));n->get_parameter(name+".implementation",type_);
  if(type_.empty()||type_=="s13::MeasuredController"){throw std::runtime_error("INVALID_DELEGATE");}
  implementation_=loader_.createSharedInstance(type_);implementation_->configure(p,name,tf,cost);metrics_=n->create_publisher<std_msgs::msg::String>("/s13/controller_metrics",100);
 }
 void activate()override{implementation_->activate();metrics_->on_activate();}
 void deactivate()override{implementation_->deactivate();metrics_->on_deactivate();}
 void cleanup()override{implementation_->cleanup();implementation_.reset();metrics_.reset();}
 void setPlan(const nav_msgs::msg::Path&p)override{path_frame_=p.header.frame_id;implementation_->setPlan(p);}
 void setSpeedLimit(const double&v,const bool&p)override{implementation_->setSpeedLimit(v,p);}
 void reset()override{implementation_->reset();}
 bool cancel()override{return implementation_->cancel();}
 geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped&p,const geometry_msgs::msg::Twist&v,nav2_core::GoalChecker*g)override{
  auto begin=std::chrono::steady_clock::now();bool ok=false;double pose_age=-1.;geometry_msgs::msg::TwistStamped result;
  auto publish=[&](){double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();auto n=parent_.lock();std_msgs::msg::String m;m.data=nlohmann::json({{"t",n->now().seconds()},{"controller",name_},{"plugin",type_},{"compute_ms",ms},{"pose_age_s",pose_age},{"success",ok},{"v",result.twist.linear.x},{"w",result.twist.angular.z}}).dump();metrics_->publish(m);};
  try{
   auto coherent=p;
   if(path_frame_!=p.header.frame_id){
    // Use the newest common observed time, not a future extrapolation or a
    // second TF publisher. All lookups are nonblocking; stale data rejects.
    const auto common=tf_->lookupTransform(path_frame_,base_frame_,tf2::TimePointZero);
    const rclcpp::Time stamp(common.header.stamp);
    pose_age=(parent_.lock()->now()-stamp).seconds();
    if(pose_age<-.05||pose_age>.15)throw nav2_core::ControllerTFError("COMMON_TF_STALE");
    const auto local=tf_->lookupTransform(p.header.frame_id,base_frame_,stamp);
    coherent.header=local.header;
    coherent.pose.position.x=local.transform.translation.x;
    coherent.pose.position.y=local.transform.translation.y;
    coherent.pose.position.z=local.transform.translation.z;
    coherent.pose.orientation=local.transform.rotation;
   }
   result=implementation_->computeVelocityCommands(coherent,v,g);ok=true;publish();return result;
  }catch(const tf2::TransformException&e){publish();throw nav2_core::ControllerTFError(e.what());}catch(...){publish();throw;}
 }
};}
PLUGINLIB_EXPORT_CLASS(s13::MeasuredController,nav2_core::Controller)
