// Use the installed ros_gz converters/QoS. Only lifecycle ownership is changed.
#include <ros_gz_bridge/ros_gz_bridge.hpp>
#include "../r08_backend/cooperative_shutdown.hpp"
#include <iostream>
class ToolBridge:public ros_gz_bridge::RosGzBridge {
public:
 void configure(){
  for(const std::string topic:{"/s10/tool/command","/s10/tool/fault","/s10/tool/raw_feedback"}){
   ros_gz_bridge::BridgeConfig c;c.ros_topic_name=c.gz_topic_name=topic;
   c.ros_type_name="std_msgs/msg/String";c.gz_type_name="gz.msgs.StringMsg";
   c.direction=topic=="/s10/tool/raw_feedback"?ros_gz_bridge::BridgeDirection::GZ_TO_ROS:ros_gz_bridge::BridgeDirection::ROS_TO_GZ;
   add_bridge(c);
  }
 }
 void close(){
  // The executor has stopped; unsubscribe while publishers and ROS context live.
  if(heartbeat_timer_)heartbeat_timer_->cancel();
  if(gz_node_)gz_node_->Unsubscribe("/s10/tool/raw_feedback");
  handles_.clear();services_.clear();heartbeat_timer_.reset();gz_node_.reset();
 }
};
int main(int argc,char**argv){
 r08::initialize(argc,argv);
 try{auto node=std::make_shared<ToolBridge>();node->configure();r08::spin(node);node->close();node.reset();}
 catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 2;}
 rclcpp::shutdown();return 0;
}
