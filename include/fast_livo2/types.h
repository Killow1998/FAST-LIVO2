#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace fast_livo2 {
inline double seconds(int64_t stamp_ns) {
  return static_cast<double>(stamp_ns / 1000000000) +
         static_cast<double>(stamp_ns % 1000000000) * 1e-9;
}
using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGB>;
struct ImuSample {
  using ConstPtr = std::shared_ptr<const ImuSample>;
  int64_t stamp_ns = 0;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  double seconds() const { return fast_livo2::seconds(stamp_ns); }
};
struct LivoxPoint {
  uint32_t offset_time = 0;
  float x = 0, y = 0, z = 0;
  uint8_t reflectivity = 0, tag = 0, line = 0;
};
struct LivoxScan { std::vector<LivoxPoint> points; };
struct Odometry {
  int64_t stamp_ns = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_body = Eigen::Vector3d::Zero();
  std::array<double, 36> covariance{};
};
struct PlaneVisual {
  int id = 0;
  Eigen::Vector3d position, scale, color;
  Eigen::Quaterniond orientation;
  double alpha = 0.;
};
enum class FrameStatus { Waiting, Skipped, Initializing, LidarUpdated, Mapped };
struct FrameOutput {
  FrameStatus status = FrameStatus::Waiting;
  Odometry odometry;
  Cloud::ConstPtr body_cloud, world_cloud, effect_cloud;
  ColorCloud::ConstPtr color_cloud;
  cv::Mat image;
  std::vector<PlaneVisual> planes;
  std::string diagnostic;
  bool append_path = false;
  bool consumed() const { return status != FrameStatus::Waiting; }
};
using OdometryObserver = std::function<void(const Odometry &)>;
} // namespace fast_livo2
