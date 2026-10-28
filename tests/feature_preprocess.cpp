#include "preprocess.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <new>

namespace {
fast_livo2::LivoxScan planarLine(float range, int count, float step = .02f) {
  fast_livo2::LivoxScan scan;
  for (int i = 0; i < count; ++i) {
    fast_livo2::LivoxPoint p;
    p.x = range;
    p.y = i * step;
    p.z = .5f;
    p.tag = 0x10;
    p.reflectivity = 12;
    p.offset_time = i * 1000;
    scan.points.push_back(p);
  }
  return scan;
}
PointCloudXYZI extract(const fast_livo2::LivoxScan &scan, int lidar_type,
                      unsigned char fill, int lines = 1, double blind = .01) {
  alignas(Preprocess) unsigned char storage[sizeof(Preprocess)];
  std::memset(storage, fill, sizeof(storage));
  auto &preprocess = *new (storage) Preprocess;
  preprocess.feature_enabled = true;
  preprocess.lidar_type = lidar_type;
  preprocess.N_SCANS = lines;
  preprocess.point_filter_num = 1;
  preprocess.blind = blind;
  preprocess.blind_sqr = blind * blind;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  if (lidar_type == AVIA) {
    preprocess.process(scan, output);
  } else {
    pcl::PointCloud<ouster_ros::Point> points;
    for (const auto &input : scan.points) {
      ouster_ros::Point p{};
      p.x = input.x; p.y = input.y; p.z = input.z;
      p.intensity = input.reflectivity; p.ring = input.line;
      p.t = input.offset_time;
      points.push_back(p);
    }
    pcl::PCLPointCloud2 message;
    pcl::toPCLPointCloud2(points, message);
    preprocess.process(message, output);
  }
  PointCloudXYZI result = *output;
  preprocess.~Preprocess();
  return result;
}
}

class FeaturePreprocess : public ::testing::TestWithParam<int> {};

TEST_P(FeaturePreprocess, PlanarReturnsDoNotDependOnObjectStorage) {
  const int lidar = GetParam();
  for (float range : {2.f, 10.f, 30.f}) {
    SCOPED_TRACE(::testing::Message() << "lidar=" << lidar << " range=" << range);
    const auto input = planarLine(range, 80);
    const auto reference = extract(input, lidar, 0);
    ASSERT_GT(reference.size(), 20u);
    for (unsigned char fill : {0x55, 0xff}) {
      const auto actual = extract(input, lidar, fill);
      ASSERT_EQ(reference.size(), actual.size());
      for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_FLOAT_EQ(actual[i].x, reference[i].x);
        EXPECT_FLOAT_EQ(actual[i].y, reference[i].y);
        EXPECT_FLOAT_EQ(actual[i].z, reference[i].z);
        EXPECT_FLOAT_EQ(actual[i].curvature, reference[i].curvature);
        EXPECT_TRUE(std::isfinite(actual[i].intensity));
      }
    }
  }
}

TEST_P(FeaturePreprocess, SparseAndBlindRingsReturnNoFeatures) {
  const int lidar = GetParam();
  for (int count : {0, 1, 2}) {
    EXPECT_TRUE(extract(planarLine(2.f, count), lidar, 0xff, 2).empty());
  }
  EXPECT_TRUE(extract(planarLine(.01f, 40, .001f), lidar, 0xff, 2, .2).empty());
}

TEST_P(FeaturePreprocess, DenseScanTailDoesNotReadPastTheRing) {
  const int lidar = GetParam();
  const auto result = extract(planarLine(30.f, 16, .001f), lidar, 0xff);
  for (const auto &point : result)
    EXPECT_TRUE(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z));
}

INSTANTIATE_TEST_SUITE_P(Lidar, FeaturePreprocess, ::testing::Values(AVIA, OUST64));
