#include "IMU_Processing.h"
#include "pose_covariance.h"
#include "message_conversion.h"
#include <gtest/gtest.h>
#if defined(TEST_ROS2)
#include <nav_msgs/msg/odometry.hpp>
#elif defined(TEST_ROS1)
#include <nav_msgs/Odometry.h>
#endif

TEST(ImuCovariance, ParentAxesAndCrossTermsReachRosInPositionRotationOrder) {
  StatesGroup state;
  state.rot_end = Eigen::AngleAxisd(std::acos(-1.) / 2., V3D::UnitZ()).toRotationMatrix();
  state.cov.setZero();
  state.cov.diagonal().head<6>() << .1, .2, .3, 1., 2., 3.;
  state.cov(3, 0) = state.cov(0, 3) = .04;
  state.cov(4, 1) = state.cov(1, 4) = .05;
  fast_livo2::Odometry output;
  output.orientation = Eigen::Quaterniond(state.rot_end);
  output.covariance = fast_livo2::poseCovariance(state);
  Eigen::Matrix<double, 6, 6> expected = Eigen::Matrix<double, 6, 6>::Zero();
  expected.diagonal() << 1., 2., 3., .2, .1, .3;
  expected(0, 4) = expected(4, 0) = .04;
  expected(1, 3) = expected(3, 1) = -.05;
#if defined(TEST_ROS2) || defined(TEST_ROS1)
#if defined(TEST_ROS2)
  nav_msgs::msg::Odometry message;
#else
  nav_msgs::Odometry message;
#endif
  fast_livo2::adapter::odometry(output, message);
  const auto &actual = message.pose.covariance;
#else
  const auto &actual = output.covariance;
#endif
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col)
      EXPECT_NEAR(actual[row * 6 + col], expected(row, col), 1e-15);
}

TEST(ImuCovariance, ConstantVelocityUncertaintyGrowsWithElapsedTime) {
  ImuProcess process;
  process.cov_acc.setZero();
  process.cov_gyr.setZero();
  process.cov_bias_acc.setZero();
  process.cov_bias_gyr.setZero();
  process.cov_inv_expo = 0.;
  StatesGroup state;
  state.cov.setZero();
  state.cov.block<3, 3>(3, 3).diagonal().setConstant(2.);
  state.cov.block<3, 3>(7, 7).diagonal().setConstant(3.);
  for (int i = 1; i <= 200; ++i) {
    process.propagateCovariance(state, M3D::Identity(), V3D::Zero(), V3D::Zero(), .005);
    const double time = i * .005;
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(state.cov(3 + axis, 3 + axis), 2. + 3. * time * time, 1e-12);
      EXPECT_NEAR(state.cov(3 + axis, 7 + axis), 3. * time, 1e-12);
    }
  }
  EXPECT_TRUE(state.cov.isApprox(state.cov.transpose(), 1e-12));
}

TEST(ImuCovariance, NoiseAndDisabledStateEstimationUseExistingModel) {
  ImuProcess process;
  process.disable_bias_est();
  process.disable_gravity_est();
  process.disable_exposure_est();
  process.cov_gyr.setConstant(.2);
  process.cov_acc = V3D(.1, .3, .5);
  process.cov_bias_gyr.setConstant(.04);
  process.cov_bias_acc.setConstant(.06);
  StatesGroup state;
  state.cov.setZero();
  const M3D rotation = Eigen::AngleAxisd(std::acos(-1.) / 2., V3D::UnitZ()).toRotationMatrix();
  state.cov.block<3, 3>(10, 10).diagonal().setConstant(4.);
  state.cov.block<3, 3>(13, 13).diagonal().setConstant(5.);
  state.cov.block<3, 3>(16, 16).diagonal().setConstant(6.);
  state.cov(6, 6) = .9;
  process.propagateCovariance(state, rotation, V3D::Zero(), V3D::Zero(), .1);
  EXPECT_NEAR(state.cov(0, 0), .002, 1e-15);
  EXPECT_NEAR(state.cov(7, 7), .003, 1e-15);
  EXPECT_NEAR(state.cov(8, 8), .001, 1e-15);
  EXPECT_NEAR(state.cov(9, 9), .005, 1e-15);
  EXPECT_NEAR(state.cov(10, 10), 4.0004, 1e-15);
  EXPECT_NEAR(state.cov(13, 13), 5.0006, 1e-15);
  EXPECT_EQ(state.cov(6, 6), .9);
}
