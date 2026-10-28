#pragma once
#include <fast_livo2/mapper.h>
#include "common_lib.h"
#include <pcl/filters/voxel_grid.h>
class ImuProcess;
class Preprocess;
class VoxelMapManager;
class VIOManager;
namespace vk { class AbstractCamera; }
#include "runtime_log.h"
#include <fstream>

namespace fast_livo2 {
struct Mapper::Impl {
  Impl(const Config &value, OdometryObserver callback);
  ~Impl();
  bool enqueueCloud(double stamp, Cloud::Ptr cloud);
  bool sync(LidarMeasureGroup &measures);
  FrameOutput process();
  void lidarUpdate();
  void visualUpdate();
  void makeOutput(FrameOutput &out);
  void saveArtifacts(const FrameOutput &out);
  void finish();
  Odometry odometry(double stamp, const StatesGroup &state) const;
  void propagateImu(const ImuSample &sample);

  const Config config;
  RuntimeLog runtime_log;
  OdometryObserver observer;
  std::unique_ptr<vk::AbstractCamera> camera;
  std::unique_ptr<Preprocess> p_pre;
  std::unique_ptr<ImuProcess> p_imu;
  std::unique_ptr<VoxelMapManager> voxelmap_manager;
  std::unique_ptr<VIOManager> vio_manager;
  Eigen::Matrix3d extR;
  Eigen::Vector3d extT;
  SLAM_MODE mode;
  StatesGroup state, predicted, propagated;
  LidarMeasureGroup measures;
  double first_lidar_time = 0., last_lidar = -1., last_imu = -1., last_image = -1.;
  double propagation_time = -1., last_metrics = -1., last_lio_seconds = 0.;
  bool first_frame = true, gravity_aligned = false, map_initialized = false;
  bool lidar_pushed = false, discarded_input = false;
  bool rebase_propagation = false;
  size_t frame_count = 0, image_count = 0;
  std::string diagnostic, root_directory;
  std::deque<Cloud::Ptr> lidar_buffer;
  std::deque<double> lidar_times;
  std::deque<ImuSample::ConstPtr> imu_buffer;
  std::deque<ImuSample> propagation_buffer;
  std::deque<cv::Mat> image_buffer;
  std::deque<double> image_times;
  std::vector<pointWithVar> points_with_variance;
  Cloud::Ptr undistorted, down_body, down_world, world_for_visual;
  Cloud::Ptr pending_body, pending_color_points, saved_intensity;
  ColorCloud::Ptr saved_color;
  std::vector<PlaneVisual> pending_planes;
  Cloud::Ptr pending_effect;
  pcl::VoxelGrid<PointType> downsample;
  int color_publish_count = 1, save_count = 0, image_save_count = 0;
  std::ofstream trajectory_file, lidar_pose_file, image_pose_file, colmap_points;
};
} // namespace fast_livo2
