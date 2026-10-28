#pragma once
#include <fast_livo2/config.h>
#include <fast_livo2/types.h>
#include <memory>
#include <pcl/PCLPointCloud2.h>

namespace fast_livo2 {
// One serial caller owns an instance. Outputs own their sensor data.
class Mapper {
public:
  explicit Mapper(const Config &config, OdometryObserver imu_observer = {});
  ~Mapper();
  Mapper(const Mapper &) = delete;
  Mapper &operator=(const Mapper &) = delete;
  bool pushImu(const ImuSample &sample);
  bool pushLivox(int64_t stamp_ns, const LivoxScan &scan);
  bool pushPointCloud(int64_t stamp_ns, const pcl::PCLPointCloud2 &scan);
  bool pushImage(int64_t stamp_ns, cv::Mat image);
  FrameOutput process();
  void finish();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo2
