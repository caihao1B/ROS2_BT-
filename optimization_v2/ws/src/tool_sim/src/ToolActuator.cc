// Simulator boundary: only this plugin can read physical pose or drive the virtual joint.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
namespace s10 {
using J=nlohmann::json;
class ToolActuator:public gz::sim::System,public gz::sim::ISystemConfigure,
 public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate {
 using Wall=std::chrono::steady_clock;
 J cfg_,request_,fault_;std::mutex mutex_;gz::transport::Node node_;
 gz::transport::Node::Publisher feedback_,truth_;gz::sim::Entity joint_=0,tool_=0,body_=0;
 std::string boot_,key_,attempt_;uint64_t seq_=0,sensor_seq_=0,received_=0,seen_=0;
 double last_command_=-100,last_t_=-1,last_pub_=-1,target_=0,command_=0,measured_=0,flow_=0,total_=0;
 bool on_=false,healthy_=true;Wall::time_point arrival_=Wall::now();std::deque<std::pair<double,J>>queue_;
 gz::math::Pose3d previous_;double previous_t_=-1;
 void receive(const gz::msgs::StringMsg&m){std::lock_guard<std::mutex> l(mutex_);try{request_=J::parse(m.data());}catch(...){request_=J();}arrival_=Wall::now();received_++;}
 void fault(const gz::msgs::StringMsg&m){std::lock_guard<std::mutex> l(mutex_);try{fault_=J::parse(m.data());}catch(...){fault_=J();}}
public:
 void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element>&s,
 gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
  cfg_=J::parse(s->Get<std::string>("config"));gz::sim::Model model(e);joint_=model.JointByName(ecm,"s10_motor_joint");tool_=model.LinkByName(ecm,"tool_link");body_=model.LinkByName(ecm,"base_link");
  if(!joint_||!tool_||!body_)throw std::runtime_error("S03 tool geometry or S10 joint missing");
  ecm.CreateComponent(joint_,gz::sim::components::JointVelocity());
  node_.Subscribe("/s10/tool/command",&ToolActuator::receive,this);node_.Subscribe("/s10/tool/fault",&ToolActuator::fault,this);
  feedback_=node_.Advertise<gz::msgs::StringMsg>("/s10/tool/raw_feedback");truth_=node_.Advertise<gz::msgs::StringMsg>("/s10/evaluation/effect");
 }
 void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm)override{
  double t=std::chrono::duration<double>(i.simTime).count(),dt=std::chrono::duration<double>(i.dt).count();J request,fault;uint64_t received;double wall_age;
  {std::lock_guard<std::mutex>l(mutex_);request=request_;fault=fault_;received=received_;wall_age=std::chrono::duration<double>(Wall::now()-arrival_).count();}
  if(t<last_t_){on_=false;seq_=0;boot_.clear();queue_.clear();}last_t_=t;
  healthy_=!(fault.is_object()&&fault.value("mode","")=="JAM"&&t<fault.value("until",0.));
  if(received!=seen_){seen_=received;
   try{
    bool off=request.at("mode")=="OFF";
    if(off)on_=false;
    if(request.at("session")!=cfg_.at("session")||request.at("tool_revision")!=cfg_.at("tool_revision"))throw std::runtime_error("VERSION");
    std::string boot=request.at("boot");uint64_t seq=request.at("sequence");double stamp=request.at("t");
    if(!std::isfinite(stamp)||t-stamp<-.03||t-stamp>cfg_["command_sim_s"].get<double>())throw std::runtime_error("TIME");
    if(boot!=boot_){if(!off)throw std::runtime_error("NEW_OWNER_REQUIRES_OFF");boot_=boot;seq_=0;}
    if(seq<=seq_)throw std::runtime_error("REPLAY");
    seq_=seq;last_command_=t;key_=request.value("key","");attempt_=request.value("attempt","");on_=!off&&request.at("mode")=="ON";
   }catch(...){on_=false;}
  }
  if(i.paused||wall_age>cfg_["command_wall_s"].get<double>()||t-last_command_>cfg_["command_sim_s"].get<double>()||!healthy_)on_=false;
  target_=on_?cfg_["rotor_target_rad_s"].get<double>():0.;
  if(!i.paused&&dt>0&&dt<.1)command_+=std::clamp(target_-command_,-cfg_["rotor_accel_rad_s2"].get<double>()*dt,cfg_["rotor_accel_rad_s2"].get<double>()*dt);
  else command_=0.;
  ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_,{command_});
  auto velocity=ecm.Component<gz::sim::components::JointVelocity>(joint_);measured_=velocity&&!velocity->Data().empty()?velocity->Data()[0]:0.;
  // Spray valve closes immediately on OFF. Mower rotor coast-down remains physical.
  flow_=on_&&healthy_&&std::abs(measured_)>=8.?std::clamp(std::abs(measured_)/12.,0.,1.):0.;
  if(!i.paused&&dt>0&&dt<.1)total_+=flow_*dt;
  if(!i.paused&&t-last_pub_>=.049){last_pub_=t;J m={{"schema","S10-TOOL-OBS/1"},{"t",t},{"seq",++sensor_seq_},{"sensor_epoch",cfg_["session"]},{"tool_revision",cfg_["tool_revision"]},{"boot",boot_},{"key",key_},{"attempt",attempt_},{"request_on",on_},{"rpm_rad_s",measured_},{"flow",flow_},{"meter",total_},{"healthy",healthy_}};queue_.push_back({t+cfg_["sensor_delay_s"].get<double>(),m});}
  while(!queue_.empty()&&queue_.front().first<=t){gz::msgs::StringMsg msg;msg.set_data(queue_.front().second.dump());feedback_.Publish(msg);queue_.pop_front();}
 }
 void PostUpdate(const gz::sim::UpdateInfo&i,const gz::sim::EntityComponentManager&ecm)override{
  double t=std::chrono::duration<double>(i.simTime).count();if(i.paused||t-previous_t_<.049)return;
  auto body=gz::sim::worldPose(body_,ecm),tool=gz::sim::worldPose(tool_,ecm);double v=previous_t_>=0?(body.Pos()-previous_.Pos()).Length()/(t-previous_t_):0.;previous_=body;previous_t_=t;
  J m={{"t",t},{"key",key_},{"attempt",attempt_},{"on",on_},{"healthy",healthy_},{"rpm_rad_s",measured_},{"flow",flow_},{"meter",total_},{"tool_xyz",{tool.X(),tool.Y(),tool.Z()}},{"tool_yaw",tool.Rot().Yaw()},{"body_v",v},{"body_xyz",{body.X(),body.Y(),body.Z()}}};gz::msgs::StringMsg msg;msg.set_data(m.dump());truth_.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(s10::ToolActuator,gz::sim::System,s10::ToolActuator::ISystemConfigure,s10::ToolActuator::ISystemPreUpdate,s10::ToolActuator::ISystemPostUpdate)
