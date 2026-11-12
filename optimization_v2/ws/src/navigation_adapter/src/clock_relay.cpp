// Retain genuine Gazebo time; publish every 10 ms simulation time and every rewind.
// Physics and sensor timestamps are unchanged. This avoids 1 kHz /clock fanout
// waking every Python node 1000 times per simulated second.
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
class Relay:public rclcpp::Node {
 rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
 rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
 int64_t last_=-1;
public:Relay():Node("s13_clock_relay") {
  pub_=create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS());
  sub_=create_subscription<rosgraph_msgs::msg::Clock>("/s13/clock_input",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr m){
   int64_t t=int64_t(m->clock.sec)*1000000000LL+m->clock.nanosec;
   if(last_<0||t<last_||t-last_>=10000000LL){last_=t;pub_->publish(*m);}
  });
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Relay>());rclcpp::shutdown();}
