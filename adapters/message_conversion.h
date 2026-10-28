#pragma once
#include <algorithm>
#include <fast_livo2/types.h>

namespace fast_livo2::adapter {
template <class Message> ImuSample imuSample(const Message &message, int64_t stamp_ns) {
  ImuSample sample;
  sample.stamp_ns = stamp_ns;
  const auto &a = message.linear_acceleration;
  const auto &w = message.angular_velocity;
  sample.linear_acceleration = Eigen::Vector3d(a.x, a.y, a.z);
  sample.angular_velocity = Eigen::Vector3d(w.x, w.y, w.z);
  return sample;
}
template <class Message> bool livoxScan(const Message &message, LivoxScan &scan) {
  if (message.point_num != message.points.size())
    return false;
  scan.points.resize(message.points.size());
  for (size_t i = 0; i < scan.points.size(); ++i) {
    const auto &p = message.points[i];
    scan.points[i] = {p.offset_time, p.x, p.y, p.z, p.reflectivity, p.tag, p.line};
  }
  return true;
}
template <class Pose> void pose(const Odometry &state, Pose &message) {
  auto &p = message.position;
  auto &q = message.orientation;
  p.x = state.position.x();
  p.y = state.position.y();
  p.z = state.position.z();
  q.x = state.orientation.x();
  q.y = state.orientation.y();
  q.z = state.orientation.z();
  q.w = state.orientation.w();
}
template <class Message> void odometry(const Odometry &state, Message &message) {
  pose(state, message.pose.pose);
  auto &v = message.twist.twist.linear;
  auto &w = message.twist.twist.angular;
  v.x = state.linear_velocity_body.x();
  v.y = state.linear_velocity_body.y();
  v.z = state.linear_velocity_body.z();
  w.x = state.angular_velocity_body.x();
  w.y = state.angular_velocity_body.y();
  w.z = state.angular_velocity_body.z();
  std::copy(state.covariance.begin(), state.covariance.end(), message.pose.covariance.begin());
}
} // namespace fast_livo2::adapter
