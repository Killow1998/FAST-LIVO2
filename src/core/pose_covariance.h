#pragma once
#include "common_lib.h"
#include <array>

namespace fast_livo2 {
// Right-attitude errors use body axes; ROS pose covariance uses parent axes.
inline std::array<double, 36> poseCovariance(const StatesGroup &s) {
  std::array<double, 36> output;
  Eigen::Matrix<double, 6, 6> covariance;
  covariance.topLeftCorner<3, 3>() = s.cov.block<3, 3>(3, 3);
  covariance.topRightCorner<3, 3>() = s.cov.block<3, 3>(3, 0) * s.rot_end.transpose();
  covariance.bottomLeftCorner<3, 3>() = s.rot_end * s.cov.block<3, 3>(0, 3);
  covariance.bottomRightCorner<3, 3>() = s.rot_end * s.cov.block<3, 3>(0, 0) * s.rot_end.transpose();
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col) output[row * 6 + col] = covariance(row, col);
  return output;
}
} // namespace fast_livo2
