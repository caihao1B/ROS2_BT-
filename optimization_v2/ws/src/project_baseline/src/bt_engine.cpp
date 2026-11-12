#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <memory>
#include <string>
using J=nlohmann::json;
using S=BT::NodeStatus;
struct Context {
  J in, commands=J::array(), transitions=J::array();
  std::string mode="STARTUP", reason="STARTUP";
  void request(const std::string& verb,const std::string& why) {
    commands.push_back({{"verb",verb},{"reason",why},{"goal_id",in.value("goal_id",std::string{})}});
  }
};
class Condition:public BT::ConditionNode {
  std::shared_ptr<Context> c;std::string kind;
public:
  Condition(const std::string& n,const BT::NodeConfig& cfg,std::shared_ptr<Context> ctx,std::string k):ConditionNode(n,cfg),c(ctx),kind(k){}
  static BT::PortsList providedPorts(){return {};}
  S tick() override {
    const auto& b=c->in;bool result=false;
    if(kind=="HasFault")result=b.at("fault").get<bool>();
    if(kind=="HoldRequired")result=b.at("hold").get<bool>();
    if(kind=="IsFinished")result=b.at("finished").get<bool>();
    if(kind=="WaitNeeded")result=b.at("wait_pending").get<bool>();
    if(kind=="NeedsRejoin")result=b.at("rejoin").get<bool>();
    if(kind=="MayExecute")result=b.at("may_execute").get<bool>() && b.value("repair",std::string{}).empty();
    if(kind=="IsYield")result=b.at("yielding").get<bool>();
    if(kind=="IsTransfer")result=b.at("transfer").get<bool>();
    if(kind=="DetourRequested")result=b.value("repair",std::string{})=="DETOUR";
    if(kind=="D1Requested")result=b.value("repair",std::string{})=="D1";
    if(kind=="D2Requested")result=b.value("repair",std::string{})=="D2";
    return result?S::SUCCESS:S::FAILURE;
  }
};
class SetMode:public BT::SyncActionNode {
  std::shared_ptr<Context> c;
public:
  SetMode(const std::string& n,const BT::NodeConfig& cfg,std::shared_ptr<Context> ctx):SyncActionNode(n,cfg),c(ctx){}
  static BT::PortsList providedPorts(){return {BT::InputPort<std::string>("value")};}
  S tick() override {c->mode=getInput<std::string>("value").value();c->reason=c->in.value("reason",std::string{});return S::SUCCESS;}
};
class Action:public BT::StatefulActionNode {
  std::shared_ptr<Context> c;std::string kind, segment;double wall_start=0,sim_start=0;
  S run() {
    const auto& b=c->in;
    const bool stopped=b.at("stop_confirmed").get<bool>();
    double elapsed=b.at("wall").get<double>()-wall_start;
    const double timeout=getInput<double>("timeout_s").value();
    if(kind=="WaitClear") elapsed=b.at("sim").get<double>()-sim_start;
    if(!((kind=="FailurePause" || kind=="HoldPause") && stopped) && (elapsed>timeout || (kind=="WaitClear" && b.at("wall").get<double>()-wall_start>120.))) {
      c->request("LATCH_FAILURE",kind+"_TIMEOUT");c->request("PAUSE",kind+"_TIMEOUT");return S::FAILURE;
    }
    if(kind=="EnsureStopped" || kind=="FailurePause" || kind=="HoldPause") {
      c->mode=kind=="FailurePause"?"FAILURE_PAUSE":kind=="HoldPause"?"HELD":"WAIT";
      c->reason=b.value("reason",kind);
      if(!stopped)c->request("PAUSE",kind);
      if(kind=="EnsureStopped" && stopped)return S::SUCCESS;
      return S::RUNNING; // A stopped fault/hold never signals mission completion.
    }
    if(kind=="WaitClear") {
      c->mode="WAIT";c->reason="WAIT_FOR_CONTINUOUS_VALID_CLEARANCE";
      if(!stopped){c->request("PAUSE","WAIT_STOP_BARRIER_LOST");return S::RUNNING;}
      if(b.at("ready").get<bool>()) {c->request("WAIT_FINISHED","CLEAR_HYSTERESIS_CONFIRMED");return S::SUCCESS;}
      return S::RUNNING;
    }
    if(kind=="RecoverSameSegment") {
      c->mode="REJOIN_SAME_SEGMENT";c->reason="CURRENT_SEGMENT_BOUNDED_CURSOR";
      if((b.at("phase")=="FAILED" && !b.value("retry_pending",false)) || b.at("phase")=="RECOVERY_REQUIRED")return S::FAILURE;
      if(b.at("phase")=="PAUSED" || (b.at("phase")=="FAILED" && b.value("retry_pending",false))) {if(stopped && b.at("ready").get<bool>())c->request("RESUME","BOUNDED_REJOIN");}
      else c->request("ALLOW","REJOIN_PROGRESS");
      if(b.at("phase")=="TRACKING" && b.at("role")=="FOLLOW_SEGMENT") {c->request("REJOIN_FINISHED","FOLLOW_SEGMENT_ACCEPTED");return S::SUCCESS;}
      return S::RUNNING;
    }
    if(kind=="ExecuteSegment") {
      if(b.at("segment")!=segment || b.at("finished").get<bool>())return S::SUCCESS;
      if(b.at("phase")=="FAILED" || b.at("phase")=="RECOVERY_REQUIRED")return S::FAILURE;
      c->request("ALLOW",c->mode);return S::RUNNING;
    }
    if(kind=="RepairDetour" || kind=="RepairD1" || kind=="RepairD2") {
      c->mode=kind;
      if(!b.value("repair_available",false)) {c->request("REPAIR_UNAVAILABLE",kind+"_NOT_IMPLEMENTED");return S::FAILURE;}
      c->reason=b.value("repair_reason",std::string("REPAIR_STOP_BARRIER"));
      if(!stopped) {c->request("PAUSE","REPAIR_STOP_BARRIER");return S::RUNNING;}
      const auto state=b.value("repair_state",std::string("IDLE"));
      if(state=="OK") {c->request("REPAIR_FINISHED",c->reason);return S::SUCCESS;}
      if(state=="FAILED") {c->request("REPAIR_FAILED",c->reason);return S::FAILURE;}
      if(state=="IDLE")c->request("START_REPAIR",kind);
      return S::RUNNING;
    }
    if(kind=="ReportTerminal") {c->mode="TASK_TERMINAL";c->reason="COVERAGE_VERDICT_REMAINS_INDEPENDENT";return S::SUCCESS;}
    throw std::runtime_error("Unknown action "+kind);
  }
public:
  Action(const std::string& n,const BT::NodeConfig& cfg,std::shared_ptr<Context> ctx,std::string k):StatefulActionNode(n,cfg),c(ctx),kind(k){}
  static BT::PortsList providedPorts(){return {BT::InputPort<double>("timeout_s",120.,"wall seconds; WaitClear sim seconds plus wall bound")};}
  S onStart() override {wall_start=c->in.at("wall");sim_start=c->in.at("sim");segment=c->in.at("segment");return run();}
  S onRunning() override{return run();}
  void onHalted() override {
    c->transitions.push_back({{"node",name()},{"event","HALT"},{"kind",kind}});
    if(kind=="ExecuteSegment" || kind=="RecoverSameSegment" || kind=="EnsureStopped")c->request("PAUSE",kind+"_HALTED");
    if(kind=="RepairDetour" || kind=="RepairD1" || kind=="RepairD2")c->request("CANCEL_REPAIR",kind+"_HALTED");
  }
};
class Trace:public BT::StatusChangeLogger {
  std::shared_ptr<Context> c;
public:
  Trace(BT::Tree& tree,std::shared_ptr<Context> ctx):StatusChangeLogger(tree.rootNode()),c(ctx){enableTransitionToIdle(true);}
  ~Trace() override {unsubscribeFromTreeChanges();}
  void callback(BT::Duration,const BT::TreeNode& node,S before,S after)override {
    c->transitions.push_back({{"node",node.name()},{"uid",node.UID()},{"from",BT::toStr(before)},{"to",BT::toStr(after)}});
  }
  void flush()override{}
};
int main(int argc,char**argv) {
  try {
    if(argc!=2)throw std::runtime_error("Usage: dino_task_bt TREE.xml");
    auto c=std::make_shared<Context>();BT::BehaviorTreeFactory f;
    for(const auto* kind:{"HasFault","HoldRequired","IsFinished","WaitNeeded","NeedsRejoin","MayExecute","IsYield","IsTransfer","DetourRequested","D1Requested","D2Requested"})f.registerNodeType<Condition>(kind,c,std::string(kind));
    for(const auto* kind:{"EnsureStopped","WaitClear","RecoverSameSegment","ExecuteSegment","FailurePause","HoldPause","RepairDetour","RepairD1","RepairD2","ReportTerminal"})f.registerNodeType<Action>(kind,c,std::string(kind));
    f.registerNodeType<SetMode>("SetMode",c);
    auto tree=f.createTreeFromFile(argv[1]);Trace trace(tree,c);
    std::string line;long last=-1;
    while(std::getline(std::cin,line)) {
      auto input=J::parse(line);if(input.at("schema")!="DINO-TASK-BT-TICK/1.0")throw std::runtime_error("SCHEMA");
      long seq=input.at("sequence");if(seq<=last)throw std::runtime_error("NON_MONOTONIC_TICK");last=seq;
      c->in=input.at("snapshot");c->commands=J::array();c->transitions=J::array();
      S state=S::IDLE;
      if(input.value("op",std::string("tick"))=="halt")tree.haltTree();else state=tree.tickOnce();
      std::cout<<J({{"schema","DINO-TASK-BT-RESULT/1.0"},{"sequence",seq},{"context",input.at("context")},{"status",BT::toStr(state)},{"mode",c->mode},{"reason",c->reason},{"commands",c->commands},{"transitions",c->transitions}}).dump()<<std::endl;
    }
    c->transitions=J::array();tree.haltTree();
    std::cerr<<J({{"event","EOF_HALT"},{"transitions",c->transitions}}).dump()<<std::endl;return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}
}
