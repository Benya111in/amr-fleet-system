// 휠 오도메트리 코어 구현.

#include "amr_localization/wheel_odometry.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace amr_localization
{

namespace
{
// 발행 스케줄 허용 오차: 주기의 25 %. 50 Hz 입력 / 50 Hz 발행에서 ±2 ms 지터가 있어도
// 매 입력 발행하고, 100 Hz 입력 / 50 Hz 발행에서는 정확히 한 개 걸러 발행한다.
constexpr double kScheduleTolerance = 0.25;

/// 바퀴별 인코더 파라미터: 슬립 기준 회전각 φ_ref = ℓ_ref / r_i
/// (거리당 슬립 분산이 공분산 모델과 일치하도록).
EncoderParams wheelEncoderParams(const WheelOdometryParams & p, double radius)
{
  EncoderParams e = p.encoder;
  if (radius > 0.0 && p.noise.slip_reference_distance > 0.0) {
    e.slip_reference_angle = p.noise.slip_reference_distance / radius;
  }
  return e;
}

Eigen::Matrix2d parameterCovariance(const DiffDriveGeometry & g, double rel_stddev)
{
  Eigen::Matrix2d s = Eigen::Matrix2d::Zero();
  const double psi_r = g.right_wheel_radius / g.wheel_separation;
  const double psi_l = g.left_wheel_radius / g.wheel_separation;
  s(0, 0) = std::pow(rel_stddev * psi_r, 2);
  s(1, 1) = std::pow(rel_stddev * psi_l, 2);
  return s;
}
}  // namespace

WheelOdometry::WheelOdometry(const WheelOdometryParams & params)
: params_(params),
  left_encoder_(wheelEncoderParams(params, params.geometry.left_wheel_radius), params.seed),
  right_encoder_(wheelEncoderParams(params, params.geometry.right_wheel_radius), params.seed + 1),
  covariance_(params.geometry.wheel_separation)
{
  if (!params_.geometry.valid()) {
    throw std::invalid_argument("wheel radii and separation must be positive");
  }
  if (params_.publish_period < 0.0) {
    throw std::invalid_argument("publish_period must be non-negative");
  }
  covariance_.setParameterCovariance(
    parameterCovariance(params_.geometry, params_.wheel_param_rel_stddev));
}

void WheelOdometry::resetInterval(double stamp)
{
  interval_start_ = stamp;
  acc_ds_left_ = acc_ds_right_ = 0.0;
  acc_dphi_left_ = acc_dphi_right_ = 0.0;
  acc_slip_var_left_ = acc_slip_var_right_ = 0.0;
  interval_ticks_left_ = interval_ticks_right_ = 0;
  interval_ambiguous_ = 0;
}

void WheelOdometry::resetPose(const Pose2D & pose)
{
  pose_ = pose;
  covariance_.reset();
}

void WheelOdometry::resetAll()
{
  left_encoder_.reset();
  right_encoder_.reset();
  initialized_ = false;
  resetPose();
}

void WheelOdometry::setWheelRadii(double left_radius, double right_radius)
{
  DiffDriveGeometry g = params_.geometry;
  g.left_wheel_radius = left_radius;
  g.right_wheel_radius = right_radius;
  if (!g.valid()) {
    throw std::invalid_argument("wheel radii must be positive");
  }
  params_.geometry = g;
}

void WheelOdometry::setParameterCovariance(const Eigen::Matrix2d & sigma_psi)
{
  covariance_.setParameterCovariance(sigma_psi);
}

bool WheelOdometry::update(
  double stamp, double left_angle, double right_angle, WheelOdometryOutput & out,
  double left_velocity, double right_velocity)
{
  if (initialized_ && stamp < last_stamp_ - 1e-9) {
    // 시뮬레이션 리셋 등 시간 역행: 모든 상태를 버리고 새로 시작한다
    resetAll();
  }
  // 인코더 샘플링 = 발행 주기 (sensors.yaml wheel_encoder.update_rate). Gazebo joint_states 는
  // 물리 스텝마다(최대 1 kHz) 오므로 그 사이 샘플은 건너뛴다. 누적 각 기반이라 건너뛴 이동량은
  // 다음 샘플에 그대로 담기고, 슬립 잡음이 "주기당" 한 번 적용되어 잡음 통계가 입력 주기와
  // 무관해진다.
  if (initialized_ && params_.publish_period > 0.0 &&
    stamp < next_publish_ - kScheduleTolerance * params_.publish_period)
  {
    return false;
  }
  const double gap = initialized_ ? stamp - last_stamp_ : 0.0;
  const EncoderReading left = left_encoder_.update(left_angle, left_velocity, gap);
  const EncoderReading right = right_encoder_.update(right_angle, right_velocity, gap);
  if (!initialized_ || !left.valid || !right.valid) {
    initialized_ = true;
    last_stamp_ = stamp;
    next_publish_ = stamp + params_.publish_period;
    resetInterval(stamp);
    return false;
  }

  const DiffDriveGeometry & g = params_.geometry;
  const double ds_left = g.left_wheel_radius * left.delta_angle;
  const double ds_right = g.right_wheel_radius * right.delta_angle;
  const BodyIncrement inc = wheelDisplacementsToIncrement(ds_left, ds_right, g.wheel_separation);

  // 분기 모호·불가능 점프: 그 증분을 믿지 않도록 한 바퀴 둘레² 를 더한다
  const double rev_left = 2.0 * M_PI * g.left_wheel_radius;
  const double rev_right = 2.0 * M_PI * g.right_wheel_radius;
  const double extra_left = left.ambiguous ? rev_left * rev_left : 0.0;
  const double extra_right = right.ambiguous ? rev_right * rev_right : 0.0;
  if (left.ambiguous || right.ambiguous) {
    ++interval_ambiguous_;
    ++ambiguous_total_;
  }
  const double var_left = stepDisplacementVariance(params_.noise, ds_left) + extra_left;
  const double var_right = stepDisplacementVariance(params_.noise, ds_right) + extra_right;
  covariance_.propagate(
    pose_.theta, inc, left.delta_angle, right.delta_angle, var_left, var_right);
  pose_ = integrate(pose_, inc, params_.integration);

  acc_ds_left_ += ds_left;
  acc_ds_right_ += ds_right;
  acc_dphi_left_ += left.delta_angle;
  acc_dphi_right_ += right.delta_angle;
  // 슬립 분산 (물리 슬립 k 는 구간 합 |ΣΔs| 로 따로): σ_s² ℓ_ref |Δs| + 모호 구간 가산
  const double slip_per_m = params_.noise.slip_noise_stddev * params_.noise.slip_noise_stddev *
    params_.noise.slip_reference_distance;
  acc_slip_var_left_ += slip_per_m * std::abs(ds_left) + extra_left;
  acc_slip_var_right_ += slip_per_m * std::abs(ds_right) + extra_right;
  interval_ticks_left_ += left.delta_ticks;
  interval_ticks_right_ += right.delta_ticks;
  last_stamp_ = stamp;

  const double interval = stamp - interval_start_;
  if (params_.publish_period > 0.0) {
    next_publish_ += params_.publish_period;
    if (stamp > next_publish_) {
      next_publish_ = stamp + params_.publish_period;  // 입력이 끊겼다 재개된 경우 위상 재설정
    }
  }
  if (!(interval > 0.0)) {
    return false;
  }

  // 구간 변위 분산 σ_i²(T) = Σ σ_s²Δs² + k|ΣΔs| + δ²/6 (양자화: 구간 양 끝 카운트 오차)
  const double tick = left_encoder_.tickAngle();
  const double q_left = params_.encoder.quantize ?
    quantizationVariance(g.left_wheel_radius * tick) : 0.0;
  const double q_right = params_.encoder.quantize ?
    quantizationVariance(g.right_wheel_radius * tick) : 0.0;

  TwistCovarianceInput tin;
  tin.var_ds_left = acc_slip_var_left_ +
    params_.noise.slip_distance_coeff * std::abs(acc_ds_left_) + q_left;
  tin.var_ds_right = acc_slip_var_right_ +
    params_.noise.slip_distance_coeff * std::abs(acc_ds_right_) + q_right;
  tin.dt = interval;
  tin.separation = g.wheel_separation;
  tin.phidot_left = acc_dphi_left_ / interval;
  tin.phidot_right = acc_dphi_right_ / interval;
  tin.sigma_psi = covariance_.parameterCovariance();
  tin.lateral_stddev = params_.lateral_stddev;
  tin.lateral_skid_coeff = params_.lateral_skid_coeff;

  out.stamp = stamp;
  out.interval = interval;
  out.pose = pose_;
  out.twist.v = 0.5 * (acc_ds_right_ + acc_ds_left_) / interval;
  out.twist.w = (acc_ds_right_ - acc_ds_left_) / (g.wheel_separation * interval);
  tin.v = out.twist.v;
  tin.w = out.twist.w;
  out.twist_covariance = computeTwistCovariance(tin);
  out.pose_covariance = covariance_.publishedPoseCovariance(pose_.theta, q_left, q_right);
  out.ticks_left = left.ticks;
  out.ticks_right = right.ticks;
  out.tick_rate_left = static_cast<double>(interval_ticks_left_) / interval;
  out.tick_rate_right = static_cast<double>(interval_ticks_right_) / interval;
  out.ambiguous_steps = interval_ambiguous_;
  resetInterval(stamp);
  return true;
}

}  // namespace amr_localization
