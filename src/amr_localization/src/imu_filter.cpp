// IMU 전처리 코어 구현.

#include "amr_localization/imu_filter.hpp"

#include <cstddef>
#include <string>

namespace amr_localization
{

ImuFilter::ImuFilter(const ImuFilterParams & params)
: params_(params),
  gyro_bias_(params.gyro_bias),
  accel_bias_(params.accel_bias),
  startup_estimator_(params.gravity),
  calibration_estimator_(params.gravity)
{
  for (auto & f : lpf_) {
    f.setCutoff(params_.lpf_cutoff_hz);
  }
  startup_active_ = params_.startup_time > 0.0;
}

void ImuFilter::applyEstimate(const ImuBiasResult & r)
{
  gyro_bias_ = r.gyro_bias;
  accel_bias_ = r.accel_bias;
  gyro_bias_var_ = r.gyro_se.cwiseProduct(r.gyro_se);
  accel_bias_var_ = r.accel_se.cwiseProduct(r.accel_se);
}

void ImuFilter::finishStartup(double stamp, bool force_fail, const std::string & reason)
{
  startup_active_ = false;
  BiasEstimateOutcome o;
  o.startup = true;
  o.duration = stamp - startup_begin_;
  if (!force_fail) {
    o.result = startup_estimator_.result();
    applyEstimate(o.result);
    o.success = true;
    o.message = "startup bias estimated from " + std::to_string(o.result.samples) + " samples";
  } else if (last_stationary_.samples >= params_.min_startup_samples) {
    o.result = last_stationary_;
    applyEstimate(o.result);
    o.success = true;
    o.message = reason + "; using " + std::to_string(o.result.samples) +
      " samples collected before motion";
  } else {
    o.result = startup_estimator_.result();
    gyro_bias_ = params_.gyro_bias;
    accel_bias_ = params_.accel_bias;
    gyro_bias_var_.setZero();
    accel_bias_var_.setZero();
    o.success = false;
    o.message = reason + "; falling back to parameter bias";
  }
  outcomes_.push_back(o);
}

bool ImuFilter::startCalibration(std::size_t samples)
{
  if (calibration_active_ || samples < 2) {
    return false;
  }
  calibration_estimator_.reset();
  calibration_target_ = samples;
  calibration_active_ = true;
  return true;
}

bool ImuFilter::takeOutcome(BiasEstimateOutcome & outcome)
{
  if (outcomes_.empty()) {
    return false;
  }
  outcome = outcomes_.front();
  outcomes_.pop_front();
  return true;
}

bool ImuFilter::process(
  double stamp, const Eigen::Vector3d & gyro_raw, const Eigen::Vector3d & accel_raw,
  Eigen::Vector3d & gyro_out, Eigen::Vector3d & accel_out)
{
  double dt = 0.0;
  if (has_stamp_) {
    dt = stamp - last_stamp_;
    if (dt < 0.0 || dt > params_.max_gap) {
      for (auto & f : lpf_) {
        f.reset();
      }
    }
  }
  has_stamp_ = true;
  last_stamp_ = stamp;

  // calibrate 요청 (기동 추정보다 우선)
  if (calibration_active_) {
    if (calibration_estimator_.count() == 0) {
      calibration_begin_ = stamp;
    }
    calibration_estimator_.addSample(gyro_raw, accel_raw);
    if (calibration_estimator_.count() >= calibration_target_) {
      BiasEstimateOutcome o;
      o.startup = false;
      o.duration = stamp - calibration_begin_;
      o.result = calibration_estimator_.result();
      if (calibration_estimator_.isStationary(
          params_.stationary_gyro_std_max, params_.stationary_accel_std_max,
          params_.accel_norm_tolerance))
      {
        applyEstimate(o.result);
        o.success = true;
        o.message = "bias calibrated from " + std::to_string(o.result.samples) + " samples";
        startup_active_ = false;  // 요청 결과가 기동 추정을 대체
      } else {
        o.success = false;
        o.message = "robot was not stationary during calibration; bias unchanged";
      }
      calibration_active_ = false;
      outcomes_.push_back(o);
    }
  }

  if (startup_active_) {
    if (!startup_started_) {
      startup_started_ = true;
      startup_begin_ = stamp;
    }
    startup_estimator_.addSample(gyro_raw, accel_raw);
    if (startup_estimator_.count() < params_.min_startup_samples) {
      return false;  // 바이어스가 정해지기 전에는 발행하지 않는다 (0.01 rad/s 드리프트 방지)
    }
    if (!startup_estimator_.isStationary(
        params_.stationary_gyro_std_max, params_.stationary_accel_std_max,
        params_.accel_norm_tolerance))
    {
      finishStartup(stamp, true, "motion detected during startup bias estimation");
    } else {
      last_stationary_ = startup_estimator_.result();
      gyro_bias_ = last_stationary_.gyro_bias;  // 누적 평균을 잠정 바이어스로
      accel_bias_ = last_stationary_.accel_bias;
      if (stamp - startup_begin_ >= params_.startup_time) {
        finishStartup(stamp, false, "");
      }
    }
  }

  const Eigen::Vector3d g = gyro_raw - gyro_bias_;
  const Eigen::Vector3d a = accel_raw - accel_bias_;
  for (int i = 0; i < 3; ++i) {
    gyro_out[i] = lpf_[i].filter(g[i], dt);
    accel_out[i] = lpf_[3 + i].filter(a[i], dt);
  }
  return true;
}

}  // namespace amr_localization
