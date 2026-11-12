#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include "risk15.hpp"
using J=nlohmann::json;using Wall=std::chrono::steady_clock;
class Guard:public rclcpp::Node{
 J config_,packet_=J::object(),robot_=J::object();double last_t_=-1,odom_t_=-100,intent_t_=-100;
 Wall::time_point packet_wall_=Wall::now();
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forecasts_,gate_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;std::ofstream log_;
 public:
 Guard():Node("s15_prediction_risk_guard"){
  auto file=declare_parameter<std::string>("config_file","");std::ifstream f(file);f>>config_;
  log_.open(declare_parameter<std::string>("log_file",""));pub_=create_publisher<std_msgs::msg::String>("/s15/risk",5);
  forecasts_=create_subscription<std_msgs::msg::String>("/s15/forecasts",5,[this](std_msgs::msg::String::ConstSharedPtr m){try{packet_=J::parse(m->data);packet_wall_=Wall::now();}catch(...){packet_=J::object();}});
  odom_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){auto&p=m->pose.pose;auto&q=p.orientation;odom_t_=rclcpp::Time(m->header.stamp).seconds();robot_["frame"]=m->header.frame_id;robot_["x"]=p.position.x;robot_["y"]=p.position.y;robot_["yaw"]=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));});
  gate_=create_subscription<std_msgs::msg::String>("/s13/gate",10,[this](std_msgs::msg::String::ConstSharedPtr m){try{J j=J::parse(m->data);if(j.value("raw_v",0.)>.01){robot_["intent_v"]=std::min(j["raw_v"].get<double>(),.12);robot_["intent_w"]=j.value("raw_w",0.);intent_t_=now().seconds();}}catch(...){}});
  timer_=create_wall_timer(std::chrono::milliseconds(40),[this]{tick();});
 }
 void tick(){
  double t=now().seconds();s15::Risk r;
  if(t-intent_t_>3.2){robot_["intent_v"]=0.;robot_["intent_w"]=0.;}
  try{
   if(t<last_t_||t-odom_t_<-.05||t-odom_t_>.2)r.reason="PREDICTION_ODOM_FALLBACK";
   else if(std::chrono::duration<double>(Wall::now()-packet_wall_).count()>1.5)r.reason="PREDICTION_TRANSPORT_FALLBACK";
   else r=s15::assess(packet_,robot_,t,config_);
  }catch(const std::exception&e){r.reason="PREDICTION_INVALID_FALLBACK";}
  last_t_=t;
  J result={{"t",t},{"frame","odom"},{"usable",r.usable},{"hold",r.hold},{"max_speed",r.cap},{"mode_envelope_mass",r.mass},{"reason",r.reason},{"method",packet_.value("method",std::string("missing"))},{"conflicts",r.conflicts},{"proxy_not_calibrated_probability",true}};
  std_msgs::msg::String msg;msg.data=result.dump();pub_->publish(msg);log_<<msg.data<<'\n';log_.flush();
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Guard>());}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}rclcpp::shutdown();}
