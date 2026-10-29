#include "IMU_Processing.h"

void ImuProcess::propagateCovariance(StatesGroup &state, const M3D &rotation,
                                    const V3D &acceleration, const V3D &angular_rate,
                                    double dt) const {
  M3D acceleration_skew;
  acceleration_skew << SKEW_SYM_MATRX(acceleration);
  MD(DIM_STATE, DIM_STATE) transition = MD(DIM_STATE, DIM_STATE)::Identity();
  MD(DIM_STATE, DIM_STATE) noise = MD(DIM_STATE, DIM_STATE)::Zero();
  transition.block<3, 3>(0, 0) = Exp(angular_rate, -dt);
  if (ba_bg_est_en) transition.block<3, 3>(0, 10) = -Eye3d * dt;
  transition.block<3, 3>(3, 7) = Eye3d * dt;
  transition.block<3, 3>(7, 0) = -rotation * acceleration_skew * dt;
  if (ba_bg_est_en) transition.block<3, 3>(7, 13) = -rotation * dt;
  if (gravity_est_en) transition.block<3, 3>(7, 16) = Eye3d * dt;

  if (exposure_estimate_en) noise(6, 6) = cov_inv_expo * dt * dt;
  noise.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
  noise.block<3, 3>(7, 7) = rotation * cov_acc.asDiagonal() * rotation.transpose() * dt * dt;
  noise.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt;
  noise.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt;
  state.cov = transition * state.cov * transition.transpose() + noise;
}
