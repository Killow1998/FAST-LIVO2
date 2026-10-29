/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "preprocess.h"

void Preprocess::l515_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<pcl::PointXYZRGB> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind_sqr) continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.normal_x = pl_orig.points[i].r;
    added_pt.normal_y = pl_orig.points[i].g;
    added_pt.normal_z = pl_orig.points[i].b;

    added_pt.curvature = 0.0;
    pl_surf.points.push_back(added_pt);
  }

}

void Preprocess::oust64_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind_sqr) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if (pl_orig.points[i].ring < N_SCANS) { pl_buff[pl_orig.points[i].ring].push_back(added_pt); }
    }

    extractFeatures();

  }
  else
  {
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < blind_sqr) continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;

      pl_surf.points.push_back(added_pt);
    }
    std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
      return a.curvature < b.curvature;
    });
  }
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].time > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    extractFeatures();

  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

void Preprocess::Pandar128_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();

  pcl::PointCloud<Pandar128_ros::Point> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = static_cast<float>(pl_orig.points[i].intensity) / 255.0f;
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

    if (i % point_filter_num == 0)
    {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
      {
        pl_surf.points.push_back(added_pt);
      }
    }
  }

  auto comparePoints = [](const PointType& a, const PointType& b) -> bool
  {
    return a.curvature < b.curvature;
  };

  std::sort(pl_surf.points.begin(), pl_surf.points.end(), comparePoints);

}

void Preprocess::xt32_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<xt32_ros::Point> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].timestamp > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  double time_head = pl_orig.points[0].timestamp;

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].timestamp / 1000.0; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    extractFeatures();

  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

void Preprocess::robosense_handler(const pcl::PCLPointCloud2 &msg)
{
  pl_surf.clear();

  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromPCLPointCloud2(msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; ++i)
  {
    if (i % point_filter_num != 0) continue;

    const auto& pt = pl_orig.points[i];
    const double x = pt.x, y = pt.y, z = pt.z;
    const double dist_sqr = x * x + y * y + z * z;
    const bool is_valid = (dist_sqr >= blind_sqr) && !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    if (!is_valid) continue;

    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = (pt.timestamp - time_head) * 1000.0;
    pl_surf.points.push_back(added_pt);
  }
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
    return a.curvature < b.curvature;
  });
}
