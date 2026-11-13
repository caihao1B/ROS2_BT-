// Demonstration scene only: periodic intent with bounded local yielding.
// Simulator state drives actor dynamics, never the robot perception or planner.
// This is not ORCA or a validated commercial pedestrian model.
#include <algorithm>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
namespace showcase {
class ReactiveActors final : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
 public gz::sim::ISystemPostUpdate {
 struct Actor {std::string name; gz::math::Vector3d center,amp; double period,phase,elapsed=0,radius=.45,clear_since=-1; bool yielding=false;
   gz::sim::Entity model=0,link=0;};
 std::vector<Actor> actors; gz::transport::Node node;
 gz::transport::Node::Publisher pub; double last=-1; gz::sim::Entity vehicle=0,vehicleLink=0;
 double rear=-.5,front=1.3,halfWidth=.52;std::ofstream events;
 public:
 void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
   gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
   auto cfg=sdf->Clone();
   rear=cfg->Get<double>("vehicle_rear");front=cfg->Get<double>("vehicle_front");halfWidth=cfg->Get<double>("vehicle_half_width");
   events.open(cfg->Get<std::string>("events"));if(!events)throw std::runtime_error("actor event log unavailable");
   for(auto e=cfg->GetElement("actor");e;e=e->GetNextElement("actor")) {
     Actor a; a.name=e->Get<std::string>("name");
     a.center=e->Get<gz::math::Vector3d>("center");
     a.amp=e->Get<gz::math::Vector3d>("amplitude");
     if(e->HasElement("radius"))a.radius=e->Get<double>("radius");
     a.period=e->Get<double>("period"); a.phase=e->Get<double>("phase");
     if(a.period<=0) throw std::runtime_error("period must be positive");
     actors.push_back(a);
   }
   pub=node.Advertise<gz::msgs::Pose_V>("/s02/evaluation/dynamic_poses");
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
   if(info.paused) return;
   const double t=std::chrono::duration<double>(info.simTime).count();
   const double dt=std::max(0.,std::min(.05,std::chrono::duration<double>(info.dt).count()));
   if(!vehicle){vehicle=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("vehicle"));if(vehicle)vehicleLink=gz::sim::Model(vehicle).LinkByName(ecm,"base_link");}
   for(auto &a:actors) {
     if(!a.model) {a.model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(a.name));
       if(a.model) a.link=gz::sim::Model(a.model).LinkByName(ecm,"body");}
     if(!a.link) continue;
     const auto pose=gz::sim::worldPose(a.link,ecm);
     const double w=2*M_PI/a.period,q=w*a.elapsed+a.phase;
     const gz::math::Vector3d target(a.center.X()+a.amp.X()*sin(q),
       a.center.Y()+a.amp.Y()*sin(2*q),a.center.Z());
     auto velocity=gz::math::Vector3d(a.amp.X()*w*cos(q),2*a.amp.Y()*w*cos(2*q),0)
       + 2.0*(target-pose.Pos());
     if(velocity.Length()>2.0) velocity=velocity.Normalized()*2.0;
     bool yield=false;
     if(vehicleLink){
       const auto vp=gz::sim::worldPose(vehicleLink,ecm);const auto local=vp.Rot().RotateVectorReverse(pose.Pos()-vp.Pos());
       const auto lv=vp.Rot().RotateVectorReverse(velocity);
       // Predict only this actor's near approach to the current full vehicle
       // rectangle. Freeze intent time while yielding: no later catch-up sprint.
       for(double horizon=0.;horizon<=1.5;horizon+=.1){
         const double x=local.X()+lv.X()*horizon,y=local.Y()+lv.Y()*horizon;
         const double dx=std::max({rear-x,0.,x-front}),dy=std::max(0.,std::abs(y)-halfWidth);
         if(std::hypot(dx,dy)<a.radius+(a.yielding?.50:.35)){yield=true;break;}
       }
     }
     // Require a stable half-second clearance before resuming. This avoids
     // millisecond stop/go chatter at the geometric threshold.
     if(a.yielding){
       if(yield)a.clear_since=-1;
       else {if(a.clear_since<0)a.clear_since=t;if(t-a.clear_since<.5)yield=true;}
     }else a.clear_since=-1;
     if(yield){velocity.X(0.);velocity.Y(0.);}else a.elapsed+=dt;
     if(yield!=a.yielding){a.yielding=yield;events<<"{\"t\":"<<t<<",\"actor\":\""<<a.name<<"\",\"yielding\":"<<(yield?"true":"false")<<"}\n";events.flush();}
     gz::sim::Link link(a.link);
     link.SetLinearVelocity(ecm,pose.Rot().RotateVectorReverse(velocity));
     // Keep the primitive person/cart upright; heading is deliberately fixed.
     link.SetAngularVelocity(ecm,-4.0*pose.Rot().Euler());
   }
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
   const double t=std::chrono::duration<double>(info.simTime).count();
   if(info.paused || t-last<.099) return;
   last=t;
   gz::msgs::Pose_V msg; auto stamp=msg.mutable_header()->mutable_stamp();
   stamp->set_sec(static_cast<int64_t>(t)); stamp->set_nsec(static_cast<int>((t-floor(t))*1e9));
   auto add=[&](gz::sim::Entity entity,const std::string &name) {
     if(!entity) return;
     auto p=gz::sim::worldPose(entity,ecm); auto m=msg.add_pose(); m->set_name(name); m->set_id(entity);
     *m->mutable_header()->mutable_stamp()=*stamp;
     auto h=m->mutable_header()->add_data();h->set_key("frame_id");h->add_value("map");
     h=m->mutable_header()->add_data();h->set_key("child_frame_id");h->add_value(name);
     m->mutable_position()->set_x(p.X());m->mutable_position()->set_y(p.Y());m->mutable_position()->set_z(p.Z());
     m->mutable_orientation()->set_w(p.Rot().W());m->mutable_orientation()->set_x(p.Rot().X());
     m->mutable_orientation()->set_y(p.Rot().Y());m->mutable_orientation()->set_z(p.Rot().Z());
   };
   for(const auto &a:actors) add(a.link,a.name);
   auto fixture=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("contact_fixture"));
   if(fixture) add(gz::sim::Model(fixture).LinkByName(ecm,"body"),"contact_fixture");
   pub.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(showcase::ReactiveActors,gz::sim::System,showcase::ReactiveActors::ISystemConfigure,showcase::ReactiveActors::ISystemPreUpdate,showcase::ReactiveActors::ISystemPostUpdate)
