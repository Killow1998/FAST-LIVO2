#include "mapper_impl.h"
namespace fast_livo2 {
bool Mapper::Impl::sync(LidarMeasureGroup &meas)
{
  const bool visual_pending = mode == LIVO && meas.lio_vio_flg == LIO;
  if (!visual_pending && lidar_buffer.empty() && config.lidar_en) return false;
  if (image_buffer.empty() && config.img_en) return false;
  if (!visual_pending && imu_buffer.empty() && config.imu_en) return false;

  switch (mode)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lidar_times.front();
    if (!lidar_pushed)
    {
      meas.lidar = lidar_buffer.front();
      if (meas.lidar->points.size() <= 1) {
        lidar_buffer.pop_front(); lidar_times.pop_front();
        discarded_input = true;
        runtime_log.write("event=scan_discard reason=insufficient_points");
        return false;
      }

      meas.lidar_frame_beg_time = lidar_times.front();
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;
    }

    if (config.imu_en && last_imu < meas.lidar_frame_end_time)
    {
      return false;
    }

    struct MeasureGroup m;

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->seconds() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    lidar_times.pop_front();

    meas.lio_vio_flg = LIO;
    meas.measures.clear();
    meas.measures.push_back(m);
    lidar_pushed = false;
    return true;
  }

  case LIVO:
  {

    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    switch (last_lio_vio_flg)
    {
    case WAIT:
    case VIO:
    {
      double img_capture_time = image_times.front() + config.exposure_time_init;

      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lidar_times.front();

      double lid_newest_time = lidar_times.back() + lidar_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back()->seconds();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        image_buffer.pop_front();
        image_times.pop_front();
        runtime_log.write("event=image_discard reason=older_than_lidar_update");
        discarded_input = true;
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        return false;
      }

      struct MeasureGroup m;

      m.imu.clear();
      m.lio_time = img_capture_time;
      while (!imu_buffer.empty())
      {
        if (imu_buffer.front()->seconds() > m.lio_time) break;

        if (imu_buffer.front()->seconds() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
      }

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lidar_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);

      while (!lidar_buffer.empty())
      {
        if (lidar_times.front() > img_capture_time) break;
        const auto &pcl = lidar_buffer.front()->points;
        double frame_header_time(lidar_times.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lidar_buffer.pop_front();
        lidar_times.pop_front();
      }

      meas.measures.clear();
      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      return true;
    }

    case LIO:
    {
      double img_capture_time = image_times.front() + config.exposure_time_init;
      meas.lio_vio_flg = VIO;
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = image_buffer.front();
      image_buffer.pop_front();
      image_times.pop_front();
      meas.measures.push_back(m);
      lidar_pushed = false;
      return true;
    }

    default:
    {
      return false;
    }
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed)
    {
      if (lidar_buffer.empty())  return false;
      meas.lidar = lidar_buffer.front();
      meas.lidar_frame_beg_time = lidar_times.front();
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_pushed = true;
    }
    struct MeasureGroup m;
    m.lio_time = meas.lidar_frame_end_time;
    lidar_buffer.pop_front();
    lidar_times.pop_front();
    lidar_pushed = false;
    meas.lio_vio_flg = LO;
    meas.measures.clear();
    meas.measures.push_back(m);
    return true;
  }

  default:
  {
    runtime_log.write("event=invalid_mode");
    return false;
  }
  }
  return false;
}
}
