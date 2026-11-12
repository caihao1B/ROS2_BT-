// Observation-only safety request. The existing chassis arbiter remains the actuator owner.
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
using nlohmann::json;
class HealthGuard:public rclcpp::Node {
 double imu_time=-100,icp_time=-100,odom_time=-100,bad_since=-1,epoch_time=0;
 double imu_age,icp_age,stop_age,startup,slow,normal,odom_age;
 bool icp_good=false;int epoch=0;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr icp,epochs;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr limit;
 rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
 rclcpp::TimerBase::SharedPtr timer;
 static double stamp(const builtin_interfaces::msg::Time &t){return t.sec+t.nanosec*1e-9;}
 public:HealthGuard():Node("s05_health_guard"){
  imu_age=declare_parameter("imu_age_s",.25);icp_age=declare_parameter("icp_age_s",.6);stop_age=declare_parameter("icp_stop_s",2.);startup=declare_parameter("startup_s",8.);slow=declare_parameter("degraded_speed_mps",.2);normal=declare_parameter("normal_speed_mps",.45);odom_age=declare_parameter("fused_age_s",.25);
  imu=create_subscription<sensor_msgs::msg::Imu>("/s05/inputs/imu",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){imu_time=stamp(m->header.stamp);});
  icp=create_subscription<std_msgs::msg::String>("/s05/icp/quality",10,[this](std_msgs::msg::String::ConstSharedPtr m){try{auto d=json::parse(m->data);icp_time=d.at("t");icp_good=d.at("valid");}catch(const json::exception&){icp_good=false;}});
  odom=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odom_time=stamp(m->header.stamp);});
  epochs=create_subscription<std_msgs::msg::String>("/s05/epoch",10,[this](std_msgs::msg::String::ConstSharedPtr m){auto d=json::parse(m->data);epoch=d.at("epoch");epoch_time=d.at("t");imu_time=icp_time=odom_time=-100;icp_good=false;bad_since=-1;});
  limit=create_publisher<std_msgs::msg::String>("/s05/safety_limit",10);diagnostics=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/s05/diagnostics",10);
  timer=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
 }
 void tick(){
  double t=now().seconds();std::string state="OK",reason="OBSERVATIONS_CURRENT";double cap=normal;
  bool bad=!icp_good||t-icp_time>icp_age||t-icp_time<-.1;
  if(bad&&bad_since<0)bad_since=t;
  if(!bad)bad_since=-1;
  if(t-epoch_time<startup){state="STOP";reason="INITIALIZING_NEW_EPOCH";cap=0;}
  else if(t-imu_time>imu_age||t-imu_time<-.1){state="STOP";reason="IMU_STALE";cap=0;}
  else if(t-odom_time>odom_age||t-odom_time<-.1){state="STOP";reason="FUSED_ODOM_STALE";cap=0;}
  else if(bad){state=t-bad_since>=stop_age?"STOP":"DEGRADED";reason="ICP_DEGENERATE_OR_STALE";cap=state=="STOP"?0:slow;}
  std_msgs::msg::String m;m.data=json{{"t",t},{"epoch",epoch},{"state",state},{"reason",reason},{"max_speed",cap},{"imu_age",t-imu_time},{"icp_age",t-icp_time},{"odom_age",t-odom_time}}.dump();limit->publish(m);
  diagnostic_msgs::msg::DiagnosticArray d;d.header.stamp=now();diagnostic_msgs::msg::DiagnosticStatus s;s.name="s05/localization_integrity";s.hardware_id="observed_sensor_rig";s.level=state=="OK"?s.OK:state=="DEGRADED"?s.WARN:s.ERROR;s.message=reason;d.status.push_back(s);diagnostics->publish(d);
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<HealthGuard>());rclcpp::shutdown();}
