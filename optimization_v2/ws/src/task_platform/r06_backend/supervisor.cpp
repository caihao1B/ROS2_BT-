#include "policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <chrono>
using r06::J;
int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("r06_common_supervisor");auto config=n->declare_parameter<std::string>("config_file","");auto xml=n->declare_parameter<std::string>("tree_file","");auto logfile=n->declare_parameter<std::string>("log_file","");J cfg;std::ifstream input(config);input>>cfg;r06::Policy p(cfg);BT::BehaviorTreeFactory f;p.register_nodes(f);auto tree=f.createTreeFromFile(xml);std::ofstream log(logfile);J latest=J::object();double receipt=-1,latest_stamp=-1,cancel_since=-1,last_log=-1;uint64_t sequence=0,dispatch=0;std::string previous,wire_action,active_action;J evidence=J::object();
 auto wall=[](){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();};
 auto pub=n->create_publisher<std_msgs::msg::String>("/r06/directive",5);auto treepub=n->create_publisher<std_msgs::msg::String>("/r06/tree",5);
 auto sub=n->create_subscription<std_msgs::msg::String>("/r06/context",5,[&](std_msgs::msg::String::ConstSharedPtr m){try{
  auto e=J::parse(m->data);double t=e.at("stamp");auto seq=e.at("sequence").get<uint64_t>();
  if(!std::isfinite(t)||seq<=sequence||t<latest_stamp||t>n->now().seconds()+.10)throw std::runtime_error("CLOCK_OR_REPLAY");
  sequence=seq;latest_stamp=t;receipt=wall();latest=e;
 }catch(...){latest=J::object();receipt=-1;}});
 auto reset=n->create_service<std_srvs::srv::Trigger>("/r06/acknowledge_manual_hold",[&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=p.reset();res->message=res->success?"Stopped, fresh and valid; explicit acknowledgement accepted":"Cannot reset while moving, stale, unsafe or unlocalized";});
 auto timer=n->create_wall_timer(std::chrono::milliseconds(30),[&]{
  double t=n->now().seconds();auto e=latest;bool bound=true;
  for(const auto& k:{"task_version","vehicle_hash","rig_revision","session"})bound &= e.contains(k)&&e.at(k)==cfg.at(k);
  e["bindings_valid"]=bound;e["stale"]=receipt<0||wall()-receipt>cfg.at("context_wall_s").get<double>()||t-latest_stamp>cfg.at("context_age_s").get<double>()||t<latest_stamp;
  p.prepare(e,t);tree.tickOnce();auto d=p.d;std::string desired=d.value("action",std::string("STOP"));std::string emitted=desired;
  bool nonmotion=desired=="WAIT"||desired=="REPLAN"||desired=="DEFER"||desired=="HUMAN"||desired=="COMPLETE";
  bool confirmed=p.yes("nav_idle")&&p.yes("tool_stopped")&&p.yes("stopped");
  bool speed_mode_only=(desired=="FOLLOW"&&active_action=="EXECUTE")||(desired=="EXECUTE"&&active_action=="FOLLOW");
  bool transition=desired!=active_action&&!speed_mode_only;
  if(desired!="STOP"&&!confirmed&&(nonmotion||transition||cancel_since>=0)){
   if(cancel_since<0)cancel_since=t;emitted="CANCEL";d["pending_action"]=desired;d["cancel_request"]=true;
   if(t-cancel_since>cfg.at("cancel_confirm_s").get<double>()){p.manual=true;p.manual_reason="CANCEL_OR_STOP_CONFIRMATION_TIMEOUT";emitted="STOP";d["reason_code"]=p.manual_reason;}
  }else {cancel_since=-1;active_action=desired;}
  if(nonmotion||emitted=="CANCEL"){d["tool_stop_request"]=true;}
  if(desired=="STOP"){d["cancel_request"]=true;d["tool_stop_request"]=true;}
  if(emitted!=wire_action){wire_action=emitted;++dispatch;}
  d["selected_action"]=desired;d["action"]=emitted;d["dispatch_id"]=dispatch;d["context_sequence"]=sequence;d["stopped_confirmation"]=confirmed;d["task_version"]=cfg["task_version"];d["session"]=cfg["session"];
  std_msgs::msg::String msg;msg.data=d.dump();pub->publish(msg);std::string key=emitted+":"+d.value("reason_code",std::string());
  if(key!=previous||t-last_log>=.2){log<<msg.data<<'\n';log.flush();previous=key;last_log=t;}
  J nodes=J::array();for(const auto& subtree:tree.subtrees)for(const auto& node:subtree->nodes)nodes.push_back({{"name",node->name()},{"status",BT::toStr(node->status())}});msg.data=J({{"t",t},{"nodes",nodes},{"action",emitted}}).dump();treepub->publish(msg);
 });
 rclcpp::spin(n);tree.haltTree();rclcpp::shutdown();return 0;
}
