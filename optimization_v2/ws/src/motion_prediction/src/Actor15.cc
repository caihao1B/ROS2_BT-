// Simulation-only piecewise motion. Future waypoints never enter online ROS.
#include <algorithm>
#include <cmath>
#include <vector>
#include <sstream>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
namespace s15sim {
class ScriptedMotion final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
 struct Point{double t;gz::math::Vector3d p;};struct Actor{std::string name;std::vector<Point> path;gz::sim::Entity model=0,link=0;};
 std::vector<Actor> actors_;gz::transport::Node node_;gz::transport::Node::Publisher pub_;double last_=-1;
 public:
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&sdf,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
  auto cfg=sdf->Clone();for(auto e=cfg->GetElement("actor");e;e=e->GetNextElement("actor")){
   Actor a;a.name=e->Get<std::string>("name");for(auto p=e->GetElement("waypoint");p;p=p->GetNextElement("waypoint")){double t,x,y,z;std::istringstream s(p->Get<std::string>());if(!(s>>t>>x>>y>>z))throw std::runtime_error("waypoint parse");a.path.push_back({t,{x,y,z}});}
   if(a.path.size()<2)throw std::runtime_error("waypoint count");for(size_t i=1;i<a.path.size();++i)if(a.path[i].t<=a.path[i-1].t)throw std::runtime_error("waypoint clock");actors_.push_back(a);
  }pub_=node_.Advertise<gz::msgs::Pose_V>("/s09/evaluation/dynamic_poses");
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
  if(info.paused)return;double t=std::chrono::duration<double>(info.simTime).count();
  for(auto&a:actors_){if(!a.model){a.model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(a.name));if(a.model)a.link=gz::sim::Model(a.model).LinkByName(ecm,"body");}if(!a.link)continue;
   size_t i=1;while(i+1<a.path.size()&&a.path[i].t<t)++i;auto&lo=a.path[i-1];auto&hi=a.path[i];double u=std::clamp((t-lo.t)/(hi.t-lo.t),0.,1.);auto pose=gz::sim::worldPose(a.link,ecm);auto target=lo.p+(hi.p-lo.p)*u;auto feed=(t>=lo.t&&t<hi.t)?(hi.p-lo.p)/(hi.t-lo.t):gz::math::Vector3d::Zero;auto v=feed+2*(target-pose.Pos());if(v.Length()>2)v=v.Normalized()*2;
   gz::sim::Link link(a.link);link.SetLinearVelocity(ecm,pose.Rot().RotateVectorReverse(v));auto euler=pose.Rot().Euler();link.SetAngularVelocity(ecm,{-4*euler.X(),-4*euler.Y(),0});
  }
 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
  double t=std::chrono::duration<double>(info.simTime).count();if(info.paused||t-last_<.099)return;last_=t;gz::msgs::Pose_V msg;
  auto s=msg.mutable_header()->mutable_stamp();s->set_sec(static_cast<int64_t>(t));s->set_nsec(static_cast<int>((t-floor(t))*1e9));
  for(const auto&a:actors_){if(!a.link)continue;auto p=gz::sim::worldPose(a.link,ecm);auto*m=msg.add_pose();m->set_name(a.name);m->set_id(a.link);*m->mutable_header()->mutable_stamp()=*s;auto*h=m->mutable_header()->add_data();h->set_key("frame_id");h->add_value("map");h=m->mutable_header()->add_data();h->set_key("child_frame_id");h->add_value(a.name);m->mutable_position()->set_x(p.Pos().X());m->mutable_position()->set_y(p.Pos().Y());m->mutable_position()->set_z(p.Pos().Z());m->mutable_orientation()->set_w(p.Rot().W());m->mutable_orientation()->set_z(p.Rot().Z());}
  pub_.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(s15sim::ScriptedMotion,gz::sim::System,s15sim::ScriptedMotion::ISystemConfigure,s15sim::ScriptedMotion::ISystemPreUpdate,s15sim::ScriptedMotion::ISystemPostUpdate)
