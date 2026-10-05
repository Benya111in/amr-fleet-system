// IMU 바이어스 추정기 구현.

#include "amr_localization/imu_bias_estimator.hpp"

#include <cmath>

namespace amr_localization
{

ImuBiasEstimator::ImuBiasEstimator(double gravity)
: gravity_(gravity)
{
}

void ImuBiasEstimator::reset()
{
  count_ = 0;
  gyro_mean_.setZero();
  gyro_m2_.setZero();
  accel_mean_.setZero();
  accel_m2_.setZero();
}

void ImuBiasEstimator::addSample(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel)
{
  ++count_;
  const double n = static_cast<double>(count_);
  const Eigen::Vector3d dg = gyro - gyro_mean_;
  gyro_mean_ += dg / n;
  gyro_m2_ += dg.cwiseProduct(gyro - gyro_mean_);
  const Eigen::Vector3d da = accel - accel_mean_;
  accel_mean_ += da / n;
  accel_m2_ += da.cwiseProduct(accel - accel_mean_);
}

ImuBiasResult ImuBiasEstimator::result() const
{
  ImuBiasResult r;
  r.samples = count_;
  if (count_ == 0) {
    return r;
  }
  r.gyro_bias = gyro_mean_;
  r.accel_bias = accel_mean_ - Eigen::Vector3d(0.0, 0.0, gravity_);
  if (count_ >= 2) {
    const double denom = static_cast<double>(count_ - 1);
    r.gyro_std = (gyro_m2_ / denom).cwiseSqrt();
    r.accel_std = (accel_m2_ / denom).cwiseSqrt();
    const double sqrt_n = std::sqrt(static_cast<double>(count_));
    r.gyro_se = r.gyro_std / sqrt_n;
    r.accel_se = r.accel_std / sqrt_n;
  }
  return r;
}

bool ImuBiasEstimator::isStationary(
  double gyro_std_max, double accel_std_max, double accel_norm_tolerance) const
{
  if (count_ < 2) {
    return false;
  }
  const ImuBiasResult r = result();
  if (r.gyro_std.maxCoeff() > gyro_std_max || r.accel_std.maxCoeff() > accel_std_max) {
    return false;
  }
  // 수평 정지면 가속도 평균 크기 ≈ g (바이어스 0.1 m/s² 수준은 허용 오차 안)
  return std::abs(accel_mean_.norm() - gravity_) <= accel_norm_tolerance;
}

}  // namespace amr_localization
