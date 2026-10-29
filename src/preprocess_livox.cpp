/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "preprocess.h"

void Preprocess::avia_handler(const fast_livo2::LivoxScan &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg.points.size();

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
      if ((msg.points[i].line < N_SCANS))
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
          }
        }
      }
    }
  }
}
