// The executor is single-threaded. Keep one cloud while odometry from another
// subscription catches up. All original frame, freshness and matching gates
// run again in cloud(), including on expiry and on replacement.
using Cloud = sensor_msgs::msg::PointCloud2;
Cloud::SharedPtr pending_cloud_;
double wait_start_ = 0.;
double pair_wait_ = 0.;
rclcpp::TimerBase::SharedPtr pairing_timer_;

void process_pending() {
  auto message = std::move(pending_cloud_);
  if (!message) return;
  pair_wait_ = std::max(0., now().seconds() - wait_start_);
  try {
    cloud(*message);
  } catch (const std::exception &error) {
    fail(now().seconds(), std::string("PROCESSING_EXCEPTION: ") + error.what());
  }
}

void try_pair() {
  if (!pending_cloud_) return;
  const double now_s = now().seconds();
  const double stamp = rclcpp::Time(pending_cloud_->header.stamp).seconds();
  const double delta = val("max_odom_delta_s");
  const bool match = std::any_of(odom_.rbegin(), odom_.rend(), [stamp,delta](const auto &odometry) {
    return std::abs(rclcpp::Time(odometry.header.stamp).seconds() - stamp) <= delta;
  });
  if (match || now_s < wait_start_ || now_s - wait_start_ >= delta ||
      now_s - stamp >= val("max_cloud_age_s") || stamp > now_s + .05 ||
      pending_cloud_->header.frame_id != "lidar_link" || epoch_bad_ || stamp <= last_) {
    process_pending();
  }
}

void receive_cloud(Cloud::SharedPtr message) {
  if (pending_cloud_) process_pending();
  pending_cloud_ = std::move(message);
  wait_start_ = now().seconds();
  try_pair();
}
