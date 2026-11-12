// Operational mapping lifecycle. It never owns the actuator or local odometry TF.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
using Empty=std_srvs::srv::Empty;
using Trigger=std_srvs::srv::Trigger;
using Steady=std::chrono::steady_clock;
class Manager:public rclcpp::Node {
  std::string state_="COLLECTING",target_,error_;
  double v_=1.,w_=1.,stopped_since_=-1.,last_odom_=-100.,last_clock_=0.;
  bool busy_=false; Steady::time_point deadline_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Client<Empty>::SharedPtr pause_,resume_,backup_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
  void request(const std::string &action,Trigger::Response &res) {
    if(busy_||state_=="WAIT_STOP"||state_=="FAULT"){res.success=false;res.message="BUSY_OR_FAULT";return;}
    if(action=="resume") {
      if(state_!="PAUSED"&&state_!="SAVED"){res.success=false;res.message="NOT_PAUSED";return;}
      state_="RESUMING";call(resume_,"COLLECTING");
    } else {target_=action;state_="WAIT_STOP";stopped_since_=-1.;deadline_=Steady::now()+std::chrono::seconds(12);}
    res.success=true;res.message="ACCEPTED: wait for /s06/manager/status completion";
  }
  void call(rclcpp::Client<Empty>::SharedPtr client,const std::string &next) {
    if(!client->service_is_ready()){state_="FAULT";error_="RTAB_SERVICE_UNAVAILABLE";return;}
    busy_=true;deadline_=Steady::now()+std::chrono::seconds(15);
    client->async_send_request(std::make_shared<Empty::Request>(),[this,next](rclcpp::Client<Empty>::SharedFuture f){
      try{f.get();if(state_=="FAULT")return;busy_=false;state_=next;}
      catch(const std::exception &e){busy_=false;state_="FAULT";error_=e.what();}
    });
  }
  void tick() {
    const double t=now().seconds();
    if(t<last_clock_){state_="FAULT";error_="CLOCK_REWIND_NEW_MAPPING_SESSION_REQUIRED";busy_=false;}
    last_clock_=t;
    if((busy_||state_=="WAIT_STOP")&&Steady::now()>deadline_){state_="FAULT";error_="LIFECYCLE_WALL_TIMEOUT";busy_=false;}
    if(state_=="WAIT_STOP"&&!busy_){
      if(t-last_odom_<.25&&std::abs(v_)<.01&&std::abs(w_)<.02){
        if(stopped_since_<0)stopped_since_=t;
        if(t-stopped_since_>.5)call(pause_,target_=="save"?"BACKUP_PENDING":"PAUSED");
      }else stopped_since_=-1.;
    }
    if(state_=="BACKUP_PENDING"&&!busy_)call(backup_,"SAVED");
    nlohmann::json j={{"t",t},{"mode","MAPPING"},{"state",state_},{"motion_allowed",state_=="COLLECTING"&&!busy_},{"busy",busy_},{"v",v_},{"w",w_},{"error",error_}};
    std_msgs::msg::String m;m.data=j.dump();pub_->publish(m);
  }
public:
  Manager():Node("s06_map_manager") {
    pub_=create_publisher<std_msgs::msg::String>("/s06/manager/status",10);
    odom_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",20,[this](nav_msgs::msg::Odometry::SharedPtr m){v_=m->twist.twist.linear.x;w_=m->twist.twist.angular.z;last_odom_=rclcpp::Time(m->header.stamp).seconds();});
    const std::string base="/s06/rtabmap/s06_slam/";
    pause_=create_client<Empty>(base+"pause");resume_=create_client<Empty>(base+"resume");backup_=create_client<Empty>(base+"backup");
    for(const std::string action:{"pause","resume","save"})services_.push_back(create_service<Trigger>("/s06/manager/"+action,[this,action](std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){request(action,*r);}));
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Manager>());rclcpp::shutdown();return 0;}
