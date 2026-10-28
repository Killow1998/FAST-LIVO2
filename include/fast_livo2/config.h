#pragma once
#include <string>
#include <vector>
namespace fast_livo2 {
struct GeometryConfig {
  bool is_pub_plane_map_ = false;
  int max_layer_ = 1;
  double max_voxel_size_ = 0.5;
  double planner_threshold_ = 0.01;
  double sigma_num_ = 3;
  double beam_err_ = 0.02;
  double dept_err_ = 0.05;
  std::vector<int> layer_init_num_ = std::vector<int>{5,5,5,5,5};
  int max_points_num_ = 50;
  int max_iterations_ = 5;
  bool map_sliding_en = false;
  int half_map_size = 100;
  double sliding_thresh = 8;
};
struct CameraConfig {
  std::string model = "Pinhole", calibration_file;
  int width = 0, height = 0;
  double scale = 1., fx = 0., fy = 0., cx = 0., cy = 0., skew = 0.;
  double d0 = 0., d1 = 0., d2 = 0., d3 = 0.;
  double k1 = 0., k2 = 0., k3 = 0., k4 = 0., k5 = 0., k6 = 0., k7 = 0.;
};
struct Config {
  bool ros_driver_fix_en = false;
  int img_en = 1;
  int lidar_en = 1;
  bool normal_en = true;
  bool inverse_composition_en = false;
  int max_iterations = 5;
  double IMG_POINT_COV = 100;
  bool raycast_en = false;
  bool exposure_estimate_en = true;
  double inv_expo_cov = 0.2;
  int grid_size = 5;
  int grid_n_height = 17;
  int patch_pyrimid_level = 3;
  int patch_size = 8;
  double outlier_threshold = 1000;
  double exposure_time_init = 0.0;
  double img_time_offset = 0.0;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;
  bool imu_prop_enable = false;
  bool gravity_align_en = false;
  std::string seq_name = "01";
  bool pose_output_en = false;
  double gyr_cov = 1.0;
  double acc_cov = 1.0;
  int imu_int_frame = 3;
  bool imu_en = false;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  double blind = 0.01;
  double filter_size_surf_min = 0.5;
  bool hilti_en = false;
  int lidar_type = 1;
  int N_SCANS = 6;
  int point_filter_num = 3;
  bool feature_enabled = false;
  int pcd_save_interval = -1;
  bool pcd_save_en = false;
  int pcd_save_type = 0;
  bool img_save_en = false;
  int img_save_interval = 1;
  bool colmap_output_en = false;
  double filter_size_pcd = 0.5;
  std::vector<double> extrinT = {0., 0., 0.};
  std::vector<double> extrinR = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::vector<double> cameraextrinT = {0., 0., 0.};
  std::vector<double> cameraextrinR = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double plot_time = -10;
  int frame_cnt = 6;
  double blind_rgb_points = 0.01;
  int pub_scan_num = 1;
  bool pub_effect_point_en = false;
  bool dense_map_en = false;
  GeometryConfig geometry;
  CameraConfig camera;
  int num_threads = 4;
  bool runtime_log_en = true;
  std::string output_directory;
};
} // namespace fast_livo2
