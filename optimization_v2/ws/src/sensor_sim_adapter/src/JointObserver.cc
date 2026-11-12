// Read-only shaft measurement. No body pose, actuation or TF publication.
#include <chrono>
#include <vector>
#include <nlohmann/json.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
namespace s04 {
class JointObserver final : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
 std::vector<gz::sim::Entity> joints;
 gz::transport::Node node;
 gz::transport::Node::Publisher publisher;
 double last=-1, period=.01;
 public:
 void Configure(const gz::sim::Entity &entity,
     const std::shared_ptr<const sdf::Element> &sdf,
     gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
   gz::sim::Model model(entity);
   period=1./sdf->Get<double>("rate");
   for (const auto &name : {"rear_left_joint", "rear_right_joint", "steer_left_joint", "steer_right_joint"}) {
     const auto joint=model.JointByName(ecm,name);
     if (!joint) continue;
     joints.push_back(joint);
     if (!ecm.Component<gz::sim::components::JointPosition>(joint))
       ecm.CreateComponent(joint,gz::sim::components::JointPosition());
     if (!ecm.Component<gz::sim::components::JointVelocity>(joint))
       ecm.CreateComponent(joint,gz::sim::components::JointVelocity());
   }
   publisher=node.Advertise<gz::msgs::StringMsg>("/s04/raw/joints");
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,
     const gz::sim::EntityComponentManager &ecm) override {
   double t=std::chrono::duration<double>(info.simTime).count();
   if (t<last) last=-1;
   if (info.paused || t-last<period-1e-7) return;
   std::vector<double> q,v;
   for (auto joint:joints) {
     auto p=ecm.Component<gz::sim::components::JointPosition>(joint);
     auto w=ecm.Component<gz::sim::components::JointVelocity>(joint);
     if (!p || !w || p->Data().empty() || w->Data().empty()) return;
     q.push_back(p->Data()[0]);v.push_back(w->Data()[0]);
   }
   last=t;
   gz::msgs::StringMsg msg;
   msg.set_data(nlohmann::json{{"t",t},{"q",q},{"velocity",v}}.dump());
   publisher.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(s04::JointObserver,gz::sim::System,
 s04::JointObserver::ISystemConfigure,s04::JointObserver::ISystemPostUpdate)
