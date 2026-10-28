#include "mapper_impl.h"
#include "IMU_Processing.h"
#include "preprocess.h"
#include "vio.h"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/polynomial_camera.h>

namespace fast_livo2 {
Mapper::Mapper(const Config &c, OdometryObserver callback) {
  if (c.extrinT.size() != 3 || c.extrinR.size() != 9 ||
      c.cameraextrinT.size() != 3 || c.cameraextrinR.size() != 9 ||
      c.N_SCANS < 1 || c.N_SCANS > 128 || c.point_filter_num < 1 ||
      c.lidar_type < 1 || c.lidar_type > 7 || c.num_threads < 1 ||
      c.filter_size_surf_min <= 0. || c.geometry.max_voxel_size_ <= 0. ||
      c.geometry.max_layer_ < 0 ||
      c.geometry.layer_init_num_.size() <= static_cast<size_t>(c.geometry.max_layer_) ||
      c.geometry.max_iterations_ < 1 || c.max_iterations < 1 ||
      c.patch_size < 2 || c.patch_size % 2 || c.patch_pyrimid_level < 1 ||
      c.patch_pyrimid_level > 8 || c.grid_n_height < 1 || c.pub_scan_num < 1)
    throw std::invalid_argument("Invalid FAST-LIVO2 calibration, grid, or solver configuration");
  if (c.img_en && c.camera.model != "Ocam" &&
      (c.camera.width < 1 || c.camera.height < 1 || c.camera.scale <= 0. ||
       c.camera.fx <= 0. || c.camera.fy <= 0.))
    throw std::invalid_argument("FAST-LIVO2 requires valid camera dimensions and intrinsics");
  impl_ = std::make_unique<Impl>(c, std::move(callback));
}
Mapper::~Mapper() = default;
Mapper::Impl::~Impl() = default;
FrameOutput Mapper::process() { return impl_->process(); }
void Mapper::finish() { impl_->finish(); }

Mapper::Impl::Impl(const Config &value, OdometryObserver callback)
    : config(value), runtime_log(value.runtime_log_en), observer(std::move(callback)),
      p_pre(std::make_unique<Preprocess>()), p_imu(std::make_unique<ImuProcess>()),
      voxelmap_manager(std::make_unique<VoxelMapManager>(value.geometry)) {
  extT << VEC_FROM_ARRAY(config.extrinT);
  extR << MAT_FROM_ARRAY(config.extrinR);
  p_pre->set(config.feature_enabled, config.lidar_type, config.blind, config.point_filter_num);
  p_pre->blind_sqr = config.blind * config.blind;
  p_pre->N_SCANS = config.N_SCANS;
  p_imu->lidar_type = config.lidar_type;
  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D::Constant(config.gyr_cov));
  p_imu->set_acc_cov_scale(V3D::Constant(config.acc_cov));
  p_imu->set_inv_expo_cov(config.inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D::Constant(0.0001));
  p_imu->set_acc_bias_cov(V3D::Constant(0.0001));
  p_imu->set_imu_init_frame_num(config.imu_int_frame);
  if (!config.imu_en) p_imu->disable_imu();
  if (!config.gravity_est_en) p_imu->disable_gravity_est();
  if (!config.ba_bg_est_en) p_imu->disable_bias_est();
  if (!config.exposure_estimate_en) p_imu->disable_exposure_est();
  voxelmap_manager->extT_ = extT;
  voxelmap_manager->extR_ = extR;
  voxelmap_manager->num_threads = config.num_threads;
  downsample.setLeafSize(config.filter_size_surf_min, config.filter_size_surf_min,
                         config.filter_size_surf_min);
  mode = config.img_en && config.lidar_en ? LIVO : config.imu_en ? ONLY_LIO : ONLY_LO;
  for (auto *cloud : {&undistorted, &down_body, &down_world, &world_for_visual,
                     &pending_color_points, &saved_intensity})
    cloud->reset(new Cloud);
  saved_color.reset(new ColorCloud);
  root_directory = config.output_directory.empty() ? ROOT_DIR : config.output_directory;
  if (root_directory.back() != '/') root_directory += '/';
  if (config.pcd_save_en || config.img_save_en || config.pose_output_en || config.colmap_output_en) {
    for (const char *directory : {"Log/pcd", "Log/image", "Log/result", "Log/Colmap/sparse/0", "Log/Colmap/images"})
      std::filesystem::create_directories(root_directory + directory);
    if (config.pcd_save_en) lidar_pose_file.open(root_directory + "Log/pcd/lidar_poses.txt");
    if (config.img_save_en) image_pose_file.open(root_directory + "Log/image/image_poses.txt");
    if (config.pose_output_en) trajectory_file.open(root_directory + "Log/result/" + config.seq_name + ".txt");
    if (config.colmap_output_en) colmap_points.open(root_directory + "Log/Colmap/sparse/0/points3D.txt");
  }
  if (config.img_en) {
    const auto &c = config.camera;
    if (c.model == "Pinhole")
      camera = std::make_unique<vk::PinholeCamera>(c.width, c.height, c.scale, c.fx, c.fy,
                                                 c.cx, c.cy, c.d0, c.d1, c.d2, c.d3);
    else if (c.model == "ATAN")
      camera = std::make_unique<vk::ATANCamera>(c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.d0);
    else if (c.model == "Ocam")
      camera = std::make_unique<vk::OmniCamera>(c.calibration_file);
    else if (c.model == "EquidistantCamera")
      camera = std::make_unique<vk::EquidistantCamera>(c.width, c.height, c.scale, c.fx, c.fy,
                                                     c.cx, c.cy, c.k1, c.k2, c.k3, c.k4);
    else if (c.model == "PolynomialCamera")
      camera = std::make_unique<vk::PolynomialCamera>(c.width, c.height, c.fx, c.fy, c.cx, c.cy,
                                                     c.skew, c.k2, c.k3, c.k4, c.k5, c.k6, c.k7);
    else throw std::invalid_argument("Unsupported FAST-LIVO2 camera model: " + c.model);
    vio_manager = std::make_unique<VIOManager>();
    auto &v = *vio_manager;
    v.cam = camera.get();
    v.grid_size = config.grid_size;
    v.voxel_size = config.geometry.max_voxel_size_;
    v.patch_size = config.patch_size;
    v.outlier_threshold = config.outlier_threshold;
    v.setImuToLidarExtrinsic(extT, extR);
    auto rotation = config.cameraextrinR, translation = config.cameraextrinT;
    v.setLidarToCameraExtrinsic(rotation, translation);
    v.state = &state;
    v.state_propagat = &predicted;
    v.max_iterations = config.max_iterations;
    v.img_point_cov = config.IMG_POINT_COV;
    v.normal_en = config.normal_en;
    v.inverse_composition_en = config.inverse_composition_en;
    v.raycast_en = config.raycast_en;
    v.grid_n_height = config.grid_n_height;
    v.patch_pyrimid_level = config.patch_pyrimid_level;
    v.exposure_estimate_en = config.exposure_estimate_en;
    v.colmap_output_en = config.colmap_output_en;
    v.num_threads = config.num_threads;
    v.output_directory = root_directory;
    v.initializeVIO();
  }
  runtime_log.write("event=start threads=" + std::to_string(config.num_threads) +
                    " camera=" + config.camera.model + " mode=" + std::to_string(mode));
}

Odometry Mapper::Impl::odometry(double stamp, const StatesGroup &s) const {
  Odometry out;
  out.stamp_ns = std::llround(stamp * 1e9);
  out.position = s.pos_end;
  out.orientation = Eigen::Quaterniond(s.rot_end);
  out.linear_velocity_body = s.rot_end.transpose() * s.vel_end;
  out.angular_velocity_body = p_imu->angularVelocity();
  Eigen::Matrix<double, 6, 6> covariance;
  covariance.topLeftCorner<3, 3>() = s.cov.block<3, 3>(3, 3);
  covariance.topRightCorner<3, 3>() = s.cov.block<3, 3>(3, 0) * s.rot_end.transpose();
  covariance.bottomLeftCorner<3, 3>() = s.rot_end * s.cov.block<3, 3>(0, 3);
  covariance.bottomRightCorner<3, 3>() = s.rot_end * s.cov.block<3, 3>(0, 0) * s.rot_end.transpose();
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col) out.covariance[row * 6 + col] = covariance(row, col);
  return out;
}

FrameOutput Mapper::Impl::process() {
  FrameOutput out;
  discarded_input = false;
  if (!sync(measures)) {
    if (discarded_input) out.status = FrameStatus::Skipped;
    out.diagnostic.swap(diagnostic);
    return out;
  }
  if (first_frame) {
    first_lidar_time = measures.last_lio_update_time;
    p_imu->first_lidar_time = first_lidar_time;
    first_frame = false;
  }
  const double started = omp_get_wtime();
  p_imu->Process2(measures, state, undistorted);
  if (!p_imu->report.empty()) {
    diagnostic.swap(p_imu->report);
    p_imu->report.clear();
    runtime_log.write(diagnostic);
  }
  if (config.gravity_align_en && !gravity_aligned && !p_imu->imu_need_init) {
    const Eigen::Matrix3d rotation = Eigen::Quaterniond::FromTwoVectors(state.gravity, V3D(0, 0, -1)).toRotationMatrix();
    state.pos_end = rotation * state.pos_end;
    state.rot_end = rotation * state.rot_end;
    state.vel_end = rotation * state.vel_end;
    state.gravity = rotation * state.gravity;
    gravity_aligned = true;
  }
  predicted = state;
  voxelmap_manager->state_ = state;
  voxelmap_manager->feats_undistort_ = undistorted;
  if (p_imu->imu_need_init) out.status = FrameStatus::Initializing;
  else if (measures.lio_vio_flg == VIO) {
    if (pending_body) {
      visualUpdate();
      out.status = FrameStatus::Mapped;
      makeOutput(out);
    } else out.status = FrameStatus::Skipped;
  } else if (!undistorted->empty()) {
    lidarUpdate();
    last_lio_seconds = omp_get_wtime() - started;
    out.status = mode == LIVO ? FrameStatus::LidarUpdated : FrameStatus::Mapped;
    if (out.status == FrameStatus::Mapped) makeOutput(out);
  } else out.status = FrameStatus::Skipped;
  out.diagnostic.swap(diagnostic);
  if (out.status == FrameStatus::Mapped) {
    rebase_propagation = true;
    if (measures.last_lio_update_time - last_metrics >= 1.) {
      std::ostringstream row;
      row << std::setprecision(17) << "event=frame stamp=" << measures.last_lio_update_time
          << " lio_process_seconds=" << last_lio_seconds
          << " vio_process_seconds=" << (mode == LIVO ? omp_get_wtime() - started : 0.) << " points=" << out.body_cloud->size()
          << " geometric_matches=" << voxelmap_manager->effct_feat_num_
          << " visual_matches=" << (vio_manager ? vio_manager->total_points : 0);
      runtime_log.write(row.str());
      last_metrics = measures.last_lio_update_time;
    }
    saveArtifacts(out);
  }
  return out;
}
} // namespace fast_livo2
