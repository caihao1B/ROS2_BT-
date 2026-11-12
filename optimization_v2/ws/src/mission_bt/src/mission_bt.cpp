#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include "decision.hpp"
#include "context_clock.hpp"
using J=nlohmann::json;
struct Shared {
 std::shared_ptr<rclcpp::Node> node;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr directive;
 J context=J::object(), decision=J::object(), cfg;
 s14::ContextBuffer contexts;
 uint64_t seq=0;
 std::ofstream log;
 void publish(const J& j){std_msgs::msg::String m;m.data=j.dump();directive->publish(m);}
};
class Dispatch:public BT::StatefulActionNode {
 Shared& s_;std::string action_;uint64_t id_=0;bool cancel_=false;
 BT::NodeStatus update(){
  const auto next=s_.decision.value("action",std::string("STOP"));
  if(next!=action_){
   if(!s_.context.value("idle",false)){
    cancel_=true;s_.publish({{"t",s_.node->now().seconds()},{"id",id_},{"action","CANCEL"},{"reason_code",s_.decision.value("reason_code",std::string())}});
    return BT::NodeStatus::RUNNING;
   }
   action_=next;id_=++s_.seq;cancel_=false;
  }
  auto d=s_.decision;d["id"]=id_;d["cancel_pending"]=cancel_;s_.publish(d);
  return BT::NodeStatus::RUNNING;
 }
 public:
 Dispatch(const std::string&name,const BT::NodeConfig&cfg,Shared&s):BT::StatefulActionNode(name,cfg),s_(s){}
 static BT::PortsList providedPorts(){return {};}
 BT::NodeStatus onStart()override{return update();}
 BT::NodeStatus onRunning()override{return update();}
 void onHalted()override{s_.publish({{"t",s_.node->now().seconds()},{"id",id_},{"action","CANCEL"},{"reason_code","TREE_HALTED"}});}
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);Shared s;s.node=std::make_shared<rclcpp::Node>("s14_mission_bt");
 auto cfg=s.node->declare_parameter<std::string>("config_file","");auto xml=s.node->declare_parameter<std::string>("tree_file","");auto log=s.node->declare_parameter<std::string>("log_file","");
 std::ifstream f(cfg);f>>s.cfg;s.log.open(log);s.directive=s.node->create_publisher<std_msgs::msg::String>("/s14/directive",5);
 const auto wall=[](){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();};
 auto sub=s.node->create_subscription<std_msgs::msg::String>("/s14/context",5,[&](std_msgs::msg::String::ConstSharedPtr m){try{s.contexts.push(J::parse(m->data),wall());}catch(...){s.contexts.clear();s.context=J::object();}});
 s14::Policy policy(s.cfg);BT::BehaviorTreeFactory factory;double last=-100;std::string previous;
 factory.registerSimpleAction("Decide",[&](BT::TreeNode& node){
  double t=s.node->now().seconds();s.context=s.contexts.select(t,wall(),s.cfg.at("context_age_s"),s.cfg.at("context_wall_s"));
  auto e=s.context;e["t"]=t;
  s.decision=policy.choose(e);static_cast<const BT::TreeNode&>(node).config().blackboard->set("decision",s.decision.dump());
  auto reason=s.decision.at("reason_code").get<std::string>();if(reason!=previous||t-last>=.2){s.log<<s.decision.dump()<<'\n';s.log.flush();last=t;previous=reason;}
  return BT::NodeStatus::SUCCESS;
 });
 factory.registerBuilder<Dispatch>("Dispatch",[&](const std::string&n,const BT::NodeConfig&c){return std::make_unique<Dispatch>(n,c,s);});
 auto tree=factory.createTreeFromFile(xml);auto pub=s.node->create_publisher<std_msgs::msg::String>("/s14/tree",2);
 auto timer=s.node->create_wall_timer(std::chrono::milliseconds(30),[&]{tree.tickOnce();J states=J::array();for(const auto&subtree:tree.subtrees)for(const auto&n:subtree->nodes)states.push_back({{"node",n->name()},{"status",BT::toStr(n->status())}});std_msgs::msg::String m;m.data=J({{"t",s.node->now().seconds()},{"nodes",states},{"decision",s.decision.value("action",std::string())}}).dump();pub->publish(m);});
 rclcpp::spin(s.node);tree.haltTree();rclcpp::shutdown();
}
