// Sole map->odom owner. A transform alone never authorizes task motion.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <array>
#include <deque>
#include <chrono>
using json=nlohmann::json;using T=Eigen::Isometry3d;using Clock=std::chrono::steady_clock;
T fromPose(const geometry_msgs::msg::Pose &p){T t=T::Identity();t.translation()<<p.position.x,p.position.y,p.position.z;t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized().toRotationMatrix();return t;}
double angle(const T&a,const T&b){return Eigen::AngleAxisd(a.linear().transpose()*b.linear()).angle();}
json values(const T&t){Eigen::Quaterniond q(t.linear());return {t.translation().x(),t.translation().y(),t.translation().z(),q.x(),q.y(),q.z(),q.w()};}
class Manager:public rclcpp::Node{
  std::array<double,36> covariance_{};
  json cfg_;std::string state_="INITIALIZING",reason_="WAITING_FOR_VERIFIED_OBSERVATIONS";
  std::deque<std::pair<double,T>> odom_;T correction_=T::Identity(),candidate_=T::Identity();
  bool have_=false,hold_=false,contract_=false,fault_=false;int count_=0,accepted_=0,jumps_=0;
  double last_good_=-100.,first_candidate_=0.,previous_evidence_=-100.,last_clock_=0.,odom_time_=-100.,v_=0.,w_=0.,contract_t_=-100.;
  Clock::time_point odom_wall_=Clock::now(),contract_wall_=Clock::now(),clock_wall_=Clock::now();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr evidence_sub_,contract_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr acknowledge_;
  void evidence(const std_msgs::msg::String &msg){
    try{
      auto e=json::parse(msg.data);double t=e.at("t");
      if(fault_||!contract_||e.at("map_id")!=cfg_.at("map_id"))return;
      if(t<=previous_evidence_||now().seconds()-t>cfg_["max_observation_age_s"].get<double>()||t>now().seconds()+.05)return;
      previous_evidence_=t;
      if(!e.at("valid").get<bool>()){count_=0;state_=have_?"DEGRADED":"RELOCALIZING";reason_=e.value("reason","QUALITY_REJECTED");return;}
      if(odom_.empty())return;
      auto nearest=std::min_element(odom_.begin(),odom_.end(),[t](const auto&a,const auto&b){return std::abs(a.first-t)<std::abs(b.first-t);});
      if(std::abs(nearest->first-t)>.05){reason_="ODOMETRY_TIME_MISMATCH";return;}
      auto p=e.at("pose").get<std::vector<double>>();T global=T::Identity();global.translation()<<p.at(0),p.at(1),p.at(2);global.linear()=Eigen::Quaterniond(p.at(6),p.at(3),p.at(4),p.at(5)).normalized().toRotationMatrix();T next=global*nearest->second.inverse();
      if(!next.matrix().allFinite())return;
      const bool jump=have_&&((next.translation()-correction_.translation()).norm()>cfg_["map_jump_translation_m"].get<double>()||angle(next,correction_)>cfg_["map_jump_yaw_rad"].get<double>());
      if(state_=="LOCALIZED"&&!jump){correction_=next;covariance_=e.at("covariance").get<std::array<double,36>>();last_good_=t;++accepted_;reason_="MATCH_CONFIRMED";return;}
      if(jump&&!hold_){hold_=true;++jumps_;count_=0;reason_="MAP_JUMP_REVALIDATION_REQUIRED";}
      if(count_==0||(next.translation()-candidate_.translation()).norm()>cfg_["stable_translation_m"].get<double>()||angle(next,candidate_)>cfg_["stable_yaw_rad"].get<double>()){count_=1;first_candidate_=t;}else ++count_;
      candidate_=next;state_="RELOCALIZING";reason_=hold_?"MAP_JUMP_HOLD":"CONFIRMING_MULTIPLE_OBSERVATIONS";
      if(count_>=cfg_["confirmation_count"].get<int>()&&t-first_candidate_>=cfg_["confirmation_span_s"].get<double>()&&std::abs(v_)<.01&&std::abs(w_)<.02){correction_=next;covariance_=e.at("covariance").get<std::array<double,36>>();have_=true;last_good_=t;state_="LOCALIZED";reason_=hold_?"LOCALIZED_WAITING_FOR_DIAGNOSTIC_ACK":"MATCH_CONFIRMED";++accepted_;}
    }catch(const std::exception &e){state_="DEGRADED";reason_=std::string("INVALID_EVIDENCE: ")+e.what();count_=0;}
  }
  void tick(){
    double t=now().seconds();auto wall=Clock::now();
    if(t<last_clock_){fault_=true;reason_="CLOCK_REWIND_NEW_SESSION_REQUIRED";}
    if(t>last_clock_){clock_wall_=wall;}
    last_clock_=t;
    bool fresh_odom=t-odom_time_<.25&&std::chrono::duration<double>(wall-odom_wall_).count()<.5;
    bool fresh_contract=contract_&&t-contract_t_<.6&&std::chrono::duration<double>(wall-contract_wall_).count()<.8;
    if(fault_||!fresh_contract){state_=(!fault_&&!have_&&t<5)?"INITIALIZING":"LOST";if(!fault_)reason_="MAP_CONTRACT_UNAVAILABLE_OR_REJECTED";count_=0;}
    else if(!fresh_odom||std::chrono::duration<double>(wall-clock_wall_).count()>.6){state_="DEGRADED";reason_="LOCAL_ODOMETRY_OR_CLOCK_STALE";count_=0;}
    else if(state_=="RELOCALIZING"&&t-previous_evidence_>cfg_["lost_after_s"].get<double>()){state_="LOST";reason_="RELOCALIZATION_OBSERVATIONS_LOST";count_=0;}
    else if(have_&&t-last_good_>cfg_["lost_after_s"].get<double>()&&state_!="RELOCALIZING"){state_="LOST";reason_="GLOBAL_OBSERVATIONS_LOST";count_=0;}
    else if(state_=="LOCALIZED"&&t-last_good_>cfg_["max_observation_age_s"].get<double>()){state_="DEGRADED";reason_="GLOBAL_OBSERVATION_STALE";count_=0;}
    const bool allow=state_=="LOCALIZED"&&!hold_&&fresh_contract&&fresh_odom&&!fault_;
    if(have_&&!odom_.empty()&&fresh_odom){
      Eigen::Quaterniond q(correction_.linear());geometry_msgs::msg::TransformStamped tr;tr.header.stamp=rclcpp::Time(static_cast<int64_t>(odom_time_*1e9));tr.header.frame_id="map";tr.child_frame_id="odom";tr.transform.translation.x=correction_.translation().x();tr.transform.translation.y=correction_.translation().y();tr.transform.translation.z=correction_.translation().z();tr.transform.rotation.x=q.x();tr.transform.rotation.y=q.y();tr.transform.rotation.z=q.z();tr.transform.rotation.w=q.w();tf_->sendTransform(tr);
      T global=correction_*odom_.back().second;Eigen::Quaterniond gq(global.linear());geometry_msgs::msg::PoseWithCovarianceStamped p;p.header=tr.header;p.pose.pose.position.x=global.translation().x();p.pose.pose.position.y=global.translation().y();p.pose.pose.position.z=global.translation().z();p.pose.pose.orientation.x=gq.x();p.pose.pose.orientation.y=gq.y();p.pose.pose.orientation.z=gq.z();p.pose.pose.orientation.w=gq.w();p.pose.covariance=covariance_;for(int k:{0,7,14,21,28,35})p.pose.covariance[k]=allow?covariance_[k]+std::max(0.,t-last_good_)*(k<21?.01:.005):9999.;pose_->publish(p);
    }
    json s={{"t",t},{"mode","LOCALIZATION"},{"state",state_},{"reason",reason_},{"navigation_allowed",allow},{"max_speed",allow?.12:0.},{"confirmation_count",count_},{"accepted_observations",accepted_},{"last_good_t",last_good_},{"observation_age_s",t-last_good_},{"map_id",cfg_["map_id"]},{"plan_revalidation_required",hold_},{"jump_count",jumps_},{"has_transform",have_},{"correction",values(correction_)},{"v",v_},{"w",w_}};
    std_msgs::msg::String msg;msg.data=s.dump();status_->publish(msg);
  }
public:
  Manager():Node("s07_global_pose_manager"){
    std::ifstream file(declare_parameter<std::string>("policy_file",""));file>>cfg_;
    status_=create_publisher<std_msgs::msg::String>("/s07/status",10);pose_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/s07/pose",10);tf_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",50,[this](nav_msgs::msg::Odometry::SharedPtr m){odom_time_=rclcpp::Time(m->header.stamp).seconds();odom_wall_=Clock::now();v_=m->twist.twist.linear.x;w_=m->twist.twist.angular.z;odom_.emplace_back(odom_time_,fromPose(m->pose.pose));while(odom_.size()>250)odom_.pop_front();});
    contract_sub_=create_subscription<std_msgs::msg::String>("/s07/map_contract",10,[this](std_msgs::msg::String::SharedPtr m){try{auto j=json::parse(m->data);contract_=j.at("valid").get<bool>()&&j.at("map_id")==cfg_.at("map_id");contract_t_=j.at("t");contract_wall_=Clock::now();}catch(...){contract_=false;}});
    evidence_sub_=create_subscription<std_msgs::msg::String>("/s07/localization/evidence",10,[this](std_msgs::msg::String::SharedPtr m){evidence(*m);});
    acknowledge_=create_service<std_srvs::srv::Trigger>("/s07/acknowledge_diagnostic_hold",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){r->success=state_=="LOCALIZED"&&std::abs(v_)<.01&&std::abs(w_)<.02&&cfg_.value("diagnostic_fixture",false);if(r->success){hold_=false;reason_="DIAGNOSTIC_ACK_NO_ACTIVE_WORK_PLAN";}r->message=r->success?"Released diagnostic hold; no work plan active":"REJECTED: not stopped/localized diagnostic fixture";});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Manager>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}rclcpp::shutdown();return 0;}
