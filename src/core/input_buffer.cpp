#include "mapper_impl.h"
#include "IMU_Processing.h"
#include "preprocess.h"
#include <cmath>

namespace fast_livo2 {
bool Mapper::pushLivox(int64_t stamp_ns, const LivoxScan &scan) {
  auto &p = *impl_;
  if (!p.config.lidar_en) return false;
  const double stamp = seconds(stamp_ns) + p.config.lidar_time_offset;
  Cloud::Ptr cloud(new Cloud);
  p.p_pre->process(scan, cloud);
  return p.enqueueCloud(stamp, std::move(cloud));
}

bool Mapper::pushPointCloud(int64_t stamp_ns, const pcl::PCLPointCloud2 &scan) {
  auto &p = *impl_;
  if (!p.config.lidar_en) return false;
  const double stamp = seconds(stamp_ns) + p.config.lidar_time_offset;
  Cloud::Ptr cloud(new Cloud);
  p.p_pre->process(scan, cloud);
  return p.enqueueCloud(stamp, std::move(cloud));
}

bool Mapper::Impl::enqueueCloud(double stamp, Cloud::Ptr cloud) {
  if (cloud->empty()) {
    runtime_log.write("event=scan_discard reason=no_valid_points");
    return false;
  }
  if (stamp < last_lidar) {
    lidar_buffer.clear();
    lidar_pushed = false;
    runtime_log.write("event=lidar_time_reset");
  }
  lidar_buffer.push_back({stamp, std::move(cloud)});
  last_lidar = stamp;
  return true;
}

bool Mapper::pushImu(const ImuSample &input) {
  auto &p = *impl_;
  if (!p.config.imu_en || p.last_lidar < 0.) return false;
  if (!input.linear_acceleration.allFinite() || !input.angular_velocity.allFinite()) {
    p.runtime_log.write("event=imu_discard reason=nonfinite");
    return false;
  }
  auto sample = std::make_shared<ImuSample>(input);
  double stamp = input.seconds() - p.config.imu_time_offset;
  if (p.config.ros_driver_fix_en) stamp += std::round(p.last_lidar - stamp);
  sample->stamp_ns = std::llround(stamp * 1e9);
  if (p.last_imu > 0. && stamp < p.last_imu) {
    p.runtime_log.write("event=imu_discard reason=backwards_time");
    return false;
  }
  p.last_imu = stamp;
  p.imu_buffer.push_back(sample);
  if (p.config.imu_prop_enable && p.observer && !p.p_imu->imu_need_init) {
    p.propagation_buffer.push_back(*sample);
    p.propagateImu(*sample);
  }
  return true;
}

bool Mapper::pushImage(int64_t stamp_ns, cv::Mat image) {
  auto &p = *impl_;
  if (!p.config.img_en) return false;
  if (p.config.hilti_en && ++p.image_count % 4 != 0) return false;
  const double stamp = seconds(stamp_ns) + p.config.img_time_offset;
  if (p.last_lidar < 0. || std::abs(stamp - p.last_image) < .001) return false;
  if (stamp < p.last_image || stamp - p.last_image < .02) {
    p.runtime_log.write("event=image_discard reason=timestamp_spacing");
    return false;
  }
  if (image.empty() || image.type() != CV_8UC3) {
    p.runtime_log.write("event=image_discard reason=expected_bgr8");
    return false;
  }
  p.image_buffer.push_back({stamp, std::move(image)});
  p.last_image = stamp;
  return true;
}

void Mapper::Impl::propagateImu(const ImuSample &sample) {
  if (rebase_propagation) {
    propagated = state;
    propagation_time = measures.last_lio_update_time;
    while (!propagation_buffer.empty() && propagation_buffer.front().seconds() < propagation_time)
      propagation_buffer.pop_front();
    rebase_propagation = false;
  }
  if (propagation_time < 0.) return;
  while (!propagation_buffer.empty()) {
    const auto current = propagation_buffer.front();
    propagation_buffer.pop_front();
    const double dt = current.seconds() - propagation_time;
    if (dt <= 0.) continue;
    const V3D acceleration = current.linear_acceleration * G_m_s2 / p_imu->IMU_mean_acc_norm - propagated.bias_a;
    const V3D angular = current.angular_velocity - propagated.bias_g;
    propagated.rot_end = propagated.rot_end * Exp(angular, dt);
    const V3D world_acceleration = propagated.rot_end * acceleration + propagated.gravity;
    propagated.pos_end += propagated.vel_end * dt + .5 * world_acceleration * dt * dt;
    propagated.vel_end += world_acceleration * dt;
    propagation_time = current.seconds();
  }
  const auto out = odometry(propagation_time, propagated, sample.angular_velocity);
  observer(out);
}
} // namespace fast_livo2
