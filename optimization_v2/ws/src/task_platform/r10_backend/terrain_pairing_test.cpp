// Exercise the production queue methods with deterministic clock and message stamps.
// Pass the generated session include using -DPAIRING_INCLUDE='".../file.inc"'.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace rclcpp {
struct Time {
  double value;
  explicit Time(double v): value(v) {}
  double seconds() const {return value;}
};
struct TimerBase {using SharedPtr=std::shared_ptr<TimerBase>;};
}
struct Header {double stamp; std::string frame_id;};
namespace sensor_msgs::msg {
struct PointCloud2 {using SharedPtr=std::shared_ptr<PointCloud2>; Header header;};
}
struct Odom {Header header;};

struct Harness {
  double t=0.,last_=-1.;
  bool epoch_bad_=false;
  int accepted=0;
  std::string reason;
  std::deque<Odom> odom_;
  rclcpp::Time now() const {return rclcpp::Time(t);}
  double val(const std::string &key) const {return key=="max_odom_delta_s" ? .05 : .45;}
  void fail(double,const std::string &why) {reason=why;}
  void cloud(const sensor_msgs::msg::PointCloud2 &m) {
    const double stamp=m.header.stamp;
    if(stamp<=last_ || epoch_bad_ || m.header.frame_id!="lidar_link" || t-stamp>.45 || stamp>t+.05) {
      fail(stamp,"SOURCE_INVALID");return;
    }
    last_=stamp;
    for(const auto &o:odom_) if(std::abs(o.header.stamp-stamp)<=.05) {++accepted;return;}
    fail(stamp,"ODOMETRY_EPOCH_MISMATCH");
  }
#include PAIRING_INCLUDE
  void receive(double stamp) {receive_cloud(std::make_shared<Cloud>(Cloud{{stamp,"lidar_link"}}));}
};

int main() {
  Harness delayed;
  delayed.t=10.;delayed.odom_.push_back({{9.8,"odom"}});delayed.receive(10.);
  assert(delayed.pending_cloud_ && delayed.pending_deferred_ && delayed.accepted==0);
  delayed.t=10.03;delayed.odom_.push_back({{9.96,"odom"}});delayed.try_pair();
  assert(!delayed.pending_cloud_ && delayed.accepted==1 && delayed.pair_wait_>.029);
  Harness timeout;
  timeout.t=20.;timeout.receive(20.);timeout.t=20.051;timeout.try_pair();
  assert(!timeout.pending_cloud_ && !timeout.accepted && timeout.reason=="ODOMETRY_EPOCH_MISMATCH");
  Harness old;
  old.t=1.;old.odom_.push_back({{.1,"odom"}});old.receive(.1);
  assert(!old.pending_cloud_ && !old.accepted && old.reason=="SOURCE_INVALID");
  Harness replaced;
  replaced.t=2.;replaced.receive(2.);replaced.t=2.025;replaced.receive(2.025);
  assert(replaced.pending_cloud_ && replaced.pending_cloud_->header.stamp==2.025 && !replaced.accepted && replaced.reason=="ODOMETRY_EPOCH_MISMATCH");
  Harness reset;
  reset.t=5.;reset.receive(5.);reset.t=4.99;reset.odom_.push_back({{5.,"odom"}});reset.try_pair();
  assert(!reset.pending_cloud_ && !reset.accepted && reset.epoch_bad_ && reset.reason=="NONMONOTONIC_SENSOR_TIME");
  std::cout << "matched delayed odometry, missing odometry timeout, stale cloud, bounded replacement and clock rollback: PASS\n";
}
