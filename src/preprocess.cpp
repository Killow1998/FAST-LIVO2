/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;
  group_size = 8;
  disA = 0.01;
  disB = 0.1;
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const fast_livo2::LivoxScan &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const pcl::PCLPointCloud2 &msg, PointCloudXYZI::Ptr &pcl_out)
{
  if (msg.width == 0 || msg.height == 0) { pcl_out->clear(); return; }
  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case L515:
    l515_handler(msg);
    break;

  case XT32:
    xt32_handler(msg);
    break;

  case PANDAR128:
    Pandar128_handler(msg);
    break;

  case ROBOSENSE:
    robosense_handler(msg);
    break;

  default:
    printf("Error LiDAR Type: %d \n", lidar_type);
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const fast_livo2::LivoxScan &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg.points.size();
  // printf("point_filter_num: %d\n", point_filter_num);

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg.points[i].line < N_SCANS) && ((msg.points[i].tag & 0x30) == 0x10))
      {
        pl_full[i].x = msg.points[i].x;
        pl_full[i].y = msg.points[i].y;
        pl_full[i].z = msg.points[i].z;
        pl_full[i].intensity = msg.points[i].reflectivity;
        pl_full[i].curvature = msg.points[i].offset_time / float(1000000); // use curvature as time of each laser points

        bool is_new = false;
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg.points[i].line].push_back(pl_full[i]);
        }
      }
    }
    extractFeatures();
  }
  else
  {
    for (uint i = 0; i < plsize; i++)
    {
      if ((msg.points[i].line < N_SCANS)) // && ((msg.points[i].tag & 0x30) == 0x10))
      {
        valid_num++;

        pl_full[i].x = msg.points[i].x;
        pl_full[i].y = msg.points[i].y;
        pl_full[i].z = msg.points[i].z;
        pl_full[i].intensity = msg.points[i].reflectivity;
        pl_full[i].curvature = msg.points[i].offset_time / float(1000000); // use curvature as time of each laser points

        if (i == 0)
          pl_full[i].curvature = fabs(pl_full[i].curvature) < 1.0 ? pl_full[i].curvature : 0.0;
        else
        {
          pl_full[i].curvature = fabs(pl_full[i].curvature - pl_full[i - 1].curvature) < 1.0
                                     ? pl_full[i].curvature
                                     : pl_full[i - 1].curvature + 0.004166667f; // float(100/24000)
        }

        if (valid_num % point_filter_num == 0)
        {
          if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z >= blind_sqr)
          {
            pl_surf.push_back(pl_full[i]);
            // if (i % 100 == 0 || i == 0) printf("pl_full[i].curvature: %f \n",
            // pl_full[i].curvature);
          }
        }
      }
    }
  }
}

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

  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
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
      // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
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

      // cout<<added_pt.curvature<<endl;

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
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
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
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

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
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // added_pt.curvature = pl_orig.points[i].t;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
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
        // printf("time mode: %d time: %d \n", given_offset_time,
        // pl_orig.points[i].t);
      }
    }
  }

  // define a lambda function for the comparison
  auto comparePoints = [](const PointType& a, const PointType& b) -> bool
  {
    return a.curvature < b.curvature;
  };
  
  // sort the points using the comparison function
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), comparePoints);
  
  // cout << GREEN << "pl_surf.points[0].timestamp: " << pl_surf.points[0].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[1000].timestamp: " << pl_surf.points[1000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[5000].timestamp: " << pl_surf.points[5000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[10000].timestamp: " << pl_surf.points[10000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[20000].timestamp: " << pl_surf.points[20000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[30000].timestamp: " << pl_surf.points[30000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[31000].timestamp: " << pl_surf.points[31000].curvature << RESET << endl;
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
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
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
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

      // printf("added_pt.curvature: %lf %lf \n", added_pt.curvature,
      // pl_orig.points[i].timestamp);

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
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
