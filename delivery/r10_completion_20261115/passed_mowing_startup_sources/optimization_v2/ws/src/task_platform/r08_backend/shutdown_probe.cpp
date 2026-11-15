#include "cooperative_shutdown.hpp"
#include <iostream>
int main(int argc,char **argv) {
 r08::initialize(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("r08_shutdown_callback_probe");
 bool intact=false;
 auto timer=node->create_wall_timer(std::chrono::milliseconds(50),[&]{
  std::cout<<"CALLBACK_ACTIVE"<<std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  intact=rclcpp::ok();
  std::cout<<(intact?"CALLBACK_CONTEXT_INTACT":"CONTEXT_SHUT_DOWN_DURING_CALLBACK")<<std::endl;
 });
 r08::spin(node);timer.reset();node.reset();rclcpp::shutdown();
 return intact && r08::stop_requested ? 0 : 2;
}
