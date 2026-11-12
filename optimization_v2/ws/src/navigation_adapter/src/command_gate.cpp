// Single S13 navigation request owner. S03 remains the sole physical actuator publisher.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <map>
#include "stop_latch.hpp"
using J=nlohmann::json;using Wall=std::chrono::steady_clock;
class Gate:public rclcpp::Node{
 J c_,state_;std::map<std::string,J> observations_;std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_,status_,steer_;rclcpp::TimerBase::SharedPtr timer_;
 double raw_v_=0,raw_w_=0,raw_t_=-100,last_t_=0,out_v_=0,out_w_=0,v_=0,w_=0,odom_t_=-100;uint64_t sequence_=0,epoch_=0;
 Wall::time_point clock_wall_=Wall::now(),lease_wall_=Wall::now();std::ofstream log_;
 StopLatch brake_;std::string brake_reason_="STARTUP_STANDSTILL";
 bool fresh(const std::string&k,double age=.3){auto&j=observations_[k];return j.is_object()&&now().seconds()-j.value("t",-100.)>=-.05&&now().seconds()-j.value("t",-100.)<age;}
 bool owner(const std::string&topic,const std::string&name){auto p=get_publishers_info_by_topic(topic);return p.size()==1&&p[0].node_name()==name;}
 double clamp(double x,double b){return std::clamp(x,-b,b);}
 void tick(){
  double t=now().seconds();double dt=std::clamp(t-last_t_,0.,.1);if(t!=last_t_)clock_wall_=Wall::now();bool rewind=t<last_t_;last_t_=t;
  auto&s=observations_["global"];auto&h=observations_["health"];auto&p=observations_["perception"];auto&tool=observations_["tool"];auto&lease=observations_["lease"];auto&drive=observations_["drive"];
  std::string reason="READY",mode="STOP";double v=0,w=0;bool aligned=false;
  if(rewind||std::chrono::duration<double>(Wall::now()-clock_wall_).count()>.2)reason="CLOCK_STOP";
  else if(!owner("/s13/lease","s13_action_client")||!owner("/s03/request","s03_test_driver")||!owner("/s13/nav_cmd","controller_server"))reason="OWNER_CONFLICT";
  else if(!fresh("lease",.2)||std::chrono::duration<double>(Wall::now()-lease_wall_).count()>.35||!lease.value("active",false)||lease.value("plan_revision",std::string())!=c_.at("plan_revision"))reason="LEASE_OR_PLAN_STOP";
  else if(!fresh("global")||!s.value("navigation_allowed",false)||s.value("plan_revalidation_required",false))reason="LOCALIZATION_STOP";
  else if(!fresh("health")||h.value("max_speed",0.)<=0)reason="LOCAL_ODOM_STOP";
  else if(!fresh("perception",.45)||!p.value("geometry_fresh",false)||!p.value("boundary_valid",false)||p.value("max_speed",0.)<=0)reason="PERCEPTION_STOP";
  else if(!fresh("tool",.25)||!tool.value("stopped_confirmed",false)||tool.value("tool_on_requested",true)||tool.value("active",true))reason="TOOL_NOT_STOPPED";
  else if(t-odom_t_>.2||!fresh("drive"))reason="FEEDBACK_STOP";
  else if(t>=c_.at("sim_deadline").get<double>()-4)reason="SIM_DEADLINE";
  else if(lease.value("operation",std::string())=="ALIGN"){
   double k=lease.value("curvature",1e9);
   if(c_.at("vehicle")!="ackermann"||!std::isfinite(k)||std::abs(k)>1./2.11)reason="INVALID_STEERING_TARGET";
   else if(std::abs(v_)>=.01||std::abs(w_)>=.02)reason="STEERING_WAIT_STOPPED";
   else {mode=drive.value("armed",false)?"DRIVE":"ARM";aligned=mode=="DRIVE";reason="STOPPED_STEERING";}
  }else if(t-raw_t_>.2||!std::isfinite(raw_v_)||!std::isfinite(raw_w_))reason="NAV_COMMAND_STALE";
  else if(!drive.value("armed",false)){mode="ARM";reason="ARM_AFTER_CONJUNCTIVE_ADMISSION";}
  else{
   mode="DRIVE";double cap=std::min({.12,s.value("max_speed",0.),h.value("max_speed",0.),p.value("max_speed",0.)});v=std::clamp(raw_v_,0.,cap);w=clamp(raw_w_,.35);
   if(c_.at("vehicle")=="ackermann")w=clamp(w,v/2.11);
   double acc=v<out_v_?.8:.5;v=std::clamp(v,out_v_-acc*dt,out_v_+acc*dt);w=std::clamp(w,out_w_-.8*dt,out_w_+.8*dt);
   if(c_.at("vehicle")=="ackermann")w=clamp(w,v/2.11);
  }
  auto& enc=observations_["encoders"];
  bool stopped=std::isfinite(v_)&&std::isfinite(w_)&&std::abs(v_)<.01&&std::abs(w_)<.02&&t-odom_t_>=-.05&&t-odom_t_<.15&&fresh("encoders",.15)&&enc.value("valid",false);
  if(stopped){const auto& wheels=enc.at("velocity");stopped=wheels.is_array()&&wheels.size()>=2;
   if(stopped)for(int i=0;i<2;++i){double speed=wheels[i].get<double>()*c_.at("wheel_radius_m").get<double>();stopped=stopped&&std::isfinite(speed)&&std::abs(speed)<.01;}}
  bool requested_stop=mode=="STOP";
  if(requested_stop)brake_reason_=reason;
  bool braking=brake_.update(requested_stop,stopped&&!rewind,t);
  if(braking){mode="STOP";v=w=0;aligned=false;if(!requested_stop)reason="BRAKING_TO_CONFIRMED_STOP";}
  if(aligned){std_msgs::msg::String steering;steering.data=J({{"t",t},{"curvature",lease.at("curvature")}}).dump();steer_->publish(steering);}
  out_v_=v;out_w_=w;std_msgs::msg::String m;m.data=J({{"sequence",++sequence_},{"epoch",epoch_},{"stamp",t},{"mode",mode},{"v",v},{"w",w}}).dump();output_->publish(m);
  state_=J({{"t",t},{"mode",mode},{"v",v},{"w",w},{"raw_v",raw_v_},{"raw_w",raw_w_},{"measured_v",v_},{"measured_w",w_},{"reason",reason},{"brake_latched",braking},{"brake_cause",brake_reason_},{"measured_standstill",stopped},{"steering",aligned},{"action_id",(lease.is_object()?lease.value("action_id",std::string()):std::string())},{"plan_revision",c_.at("plan_revision")}});m.data=state_.dump();status_->publish(m);log_<<m.data<<"\n";log_.flush();
 }
public:
 Gate():Node("s03_test_driver"){
  declare_parameter("config_file",std::string());std::ifstream f(get_parameter("config_file").as_string());f>>c_;log_.open(c_.at("log_file").get<std::string>());output_=create_publisher<std_msgs::msg::String>("/s03/request",1);status_=create_publisher<std_msgs::msg::String>("/s13/gate",20);steer_=create_publisher<std_msgs::msg::String>("/s13/steering_request",1);
  for(auto p:std::vector<std::pair<std::string,std::string>>{{"/s08/status","global"},{"/s05/safety_limit","health"},{"/s09/safety","perception"},{"/s10/action/state","tool"},{"/s13/lease","lease"},{"/s03/drive/status","drive"},{"/s03/encoders","encoders"}})subs_.push_back(create_subscription<std_msgs::msg::String>(p.first,20,[this,key=p.second](std_msgs::msg::String::SharedPtr m){try{observations_[key]=J::parse(m->data);if(key=="lease")lease_wall_=Wall::now();if(key=="encoders")epoch_=observations_[key].at("epoch");}catch(...){observations_[key]=J::object();}}));
  cmd_=create_subscription<geometry_msgs::msg::Twist>("/s13/nav_cmd",1,[this](geometry_msgs::msg::Twist::SharedPtr m){raw_v_=m->linear.x;raw_w_=m->angular.z;raw_t_=now().seconds();});
  odom_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",20,[this](nav_msgs::msg::Odometry::SharedPtr m){v_=m->twist.twist.linear.x;w_=m->twist.twist.angular.z;odom_t_=rclcpp::Time(m->header.stamp).seconds();});timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
 }
 ~Gate(){log_.close();}
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Gate>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}rclcpp::shutdown();}
