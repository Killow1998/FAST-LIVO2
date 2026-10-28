#include "mapper_impl.h"
#include "vio.h"

namespace fast_livo2 {
void Mapper::Impl::lidarUpdate() {
  downsample.setInputCloud(undistorted);
  downsample.filter(*down_body);
  down_world->resize(down_body->size());
  const M3D rotation = state.rot_end * extR;
  const V3D translation = state.rot_end * extT + state.pos_end;
  for (size_t i = 0; i < down_body->size(); ++i) {
    const auto &point = down_body->points[i];
    const V3D position = rotation * V3D(point.x, point.y, point.z) + translation;
    auto &world = down_world->points[i];
    world.x = position.x(); world.y = position.y(); world.z = position.z();
    world.intensity = point.intensity;
  }
  auto &map = *voxelmap_manager;
  map.feats_down_body_ = down_body;
  map.feats_down_world_ = down_world;
  map.feats_down_size_ = down_body->size();
  if (!map_initialized) {
    map.BuildVoxelMap();
    map_initialized = true;
  }
  map.StateEstimation(predicted);
  state = map.state_;
  const M3D updated_rotation = state.rot_end * extR;
  const V3D updated_translation = state.rot_end * extT + state.pos_end;
  for (size_t i = 0; i < down_body->size(); ++i) {
    const auto &point = down_body->points[i];
    // Preserve the original float world-cloud conversion used by map updates.
    const V3D position = updated_rotation * V3D(point.x, point.y, point.z) + updated_translation;
    map.pv_list_[i].point_w = position.cast<float>().cast<double>();
    const M3D cross = map.cross_mat_list_[i];
    map.pv_list_[i].var = updated_rotation * map.body_cov_list_[i] * updated_rotation.transpose() +
        (-cross) * state.cov.block<3, 3>(0, 0) * (-cross).transpose() + state.cov.block<3, 3>(3, 3);
  }
  map.UpdateVoxelMap(map.pv_list_);
  points_with_variance = map.pv_list_;
  if (config.geometry.map_sliding_en) map.mapSliding();
  if (config.geometry.is_pub_plane_map_) pending_planes = map.planeVisuals();
  if (config.pub_effect_point_en) {
    pending_effect.reset(new Cloud);
    for (const auto &plane : map.ptpl_list_) {
      const V3D position = state.rot_end * (extR * plane.point_b_ + extT) + state.pos_end;
      PointType point;
      point.x = position.x(); point.y = position.y(); point.z = position.z();
      point.intensity = 255.f;
      pending_effect->push_back(point);
    }
  }
  pending_body.reset(new Cloud(*undistorted));
  for (auto &point : pending_body->points) {
    const V3D position = extR * V3D(point.x, point.y, point.z) + extT;
    point.x = position.x(); point.y = position.y(); point.z = position.z();
  }
  const auto &cloud = config.dense_map_en ? undistorted : down_body;
  world_for_visual->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &point = cloud->points[i];
    const V3D position = updated_rotation * V3D(point.x, point.y, point.z) + updated_translation;
    auto &world = world_for_visual->points[i];
    world.x = position.x(); world.y = position.y(); world.z = position.z();
    world.intensity = point.intensity;
  }
}

void Mapper::Impl::visualUpdate() {
  if (world_for_visual->empty()) return;
  vio_manager->plot_flag = std::abs(measures.last_lio_update_time - first_lidar_time - config.plot_time) <
                           config.frame_cnt / 2 * .1;
  vio_manager->processFrame(measures.measures.back().img, points_with_variance,
                           voxelmap_manager->voxel_map_, measures.last_lio_update_time - first_lidar_time);
}
} // namespace fast_livo2
