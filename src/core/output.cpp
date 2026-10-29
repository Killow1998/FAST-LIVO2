#include "mapper_impl.h"
#include "vio.h"
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <sstream>

namespace fast_livo2 {
void Mapper::Impl::makeOutput(FrameOutput &out, const V3D &angular_measurement) {
  out.odometry = odometry(measures.last_lio_update_time, state, angular_measurement);
  out.body_cloud = std::move(pending_body);
  out.planes.swap(pending_planes);
  out.effect_cloud = std::move(pending_effect);
  out.append_path = ++frame_count % 10 == 0;
  if (mode != LIVO) {
    out.world_cloud.reset(new Cloud(*world_for_visual));
    return;
  }
  out.image = vio_manager->img_cp;
  *pending_color_points += *world_for_visual;
  color_publish_count += 2; // The original batching counter advanced at LIO and VIO.
  if (color_publish_count < config.pub_scan_num) return;
  color_publish_count = 1;
  ColorCloud::Ptr colored(new ColorCloud);
  colored->reserve(pending_color_points->size());
  for (const auto &point : pending_color_points->points) {
    const V3D world(point.x, point.y, point.z);
    const V3D camera_point = vio_manager->new_frame_->w2f(world);
    if (camera_point.z() < 0.) continue;
    const V2D pixel = vio_manager->new_frame_->w2c(world);
    if (!camera->isInFrame(pixel.cast<int>(), 3) || camera_point.norm() <= config.blind_rgb_points) continue;
    const V3F color = vio_manager->getInterpolatedPixel(vio_manager->img_rgb, pixel);
    pcl::PointXYZRGB output;
    output.x = point.x; output.y = point.y; output.z = point.z;
    output.r = color[2]; output.g = color[1]; output.b = color[0];
    colored->push_back(output);
  }
  out.color_cloud = colored;
  // A completed color batch must not accumulate indefinitely when no point is visible.
  pending_color_points->clear();
  world_for_visual->clear();
}

void Mapper::Impl::saveArtifacts(const FrameOutput &out) {
  const auto &pose = out.odometry;
  const double stamp = seconds(pose.stamp_ns);
  std::ostringstream pose_row;
  if (config.pose_output_en || config.pcd_save_en || config.img_save_en) {
    pose_row << std::fixed << std::setprecision(6) << stamp << ' ' << pose.position.transpose() << ' '
             << pose.orientation.x() << ' ' << pose.orientation.y() << ' ' << pose.orientation.z()
             << ' ' << pose.orientation.w() << '\n';
    if (config.pose_output_en) trajectory_file << pose_row.str();
  }
  std::ostringstream name;
  name << std::fixed << std::setprecision(6) << stamp;
  if (config.pcd_save_en) {
    if (config.pcd_save_type == 1) *saved_intensity += *out.body_cloud;
    else if (out.color_cloud) *saved_color += *out.color_cloud;
    else if (out.world_cloud) *saved_intensity += *out.world_cloud;
    lidar_pose_file << pose_row.str();
    const int interval = config.pcd_save_type == 1 ? 1 : config.pcd_save_interval;
    if (++save_count >= interval && interval > 0) {
      const std::string file = root_directory + "Log/pcd/" + name.str() + ".pcd";
      if (!saved_color->empty()) pcl::io::savePCDFileBinary(file, *saved_color);
      if (!saved_intensity->empty()) pcl::io::savePCDFileBinary(file, *saved_intensity);
      saved_color->clear(); saved_intensity->clear(); save_count = 0;
    }
  }
  if (config.img_save_en && mode == LIVO && ++image_save_count >= config.img_save_interval && config.img_save_interval > 0) {
    cv::imwrite(root_directory + "Log/image/" + name.str() + ".png", vio_manager->img_rgb);
    image_pose_file << pose_row.str();
    image_save_count = 0;
  }
}

void Mapper::Impl::finish() {
  if (config.pcd_save_en && (!saved_color->empty() || !saved_intensity->empty())) {
    const std::string file = root_directory + "Log/pcd/all_raw_points.pcd";
    if (!saved_color->empty()) {
      pcl::io::savePCDFileBinary(file, *saved_color);
      pcl::VoxelGrid<pcl::PointXYZRGB> filter;
      filter.setLeafSize(config.filter_size_pcd, config.filter_size_pcd, config.filter_size_pcd);
      filter.setInputCloud(saved_color);
      ColorCloud filtered;
      filter.filter(filtered);
      pcl::io::savePCDFileBinary(root_directory + "Log/pcd/all_downsampled_points.pcd", filtered);
      if (config.colmap_output_en) {
        colmap_points << "# POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < filtered.size(); ++i) {
          const auto &point = filtered.points[i];
          colmap_points << i << ' ' << point.x << ' ' << point.y << ' ' << point.z << ' '
                        << static_cast<int>(point.r) << ' ' << static_cast<int>(point.g) << ' '
                        << static_cast<int>(point.b) << " 0\n";
        }
      }
    } else pcl::io::savePCDFileBinary(file, *saved_intensity);
    saved_color->clear(); saved_intensity->clear();
  }
  runtime_log.write("event=finish frames=" + std::to_string(frame_count));
}
} // namespace fast_livo2
