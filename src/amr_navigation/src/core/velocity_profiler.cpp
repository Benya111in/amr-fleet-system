#include "amr_navigation/core/velocity_profiler.hpp"

#include <algorithm>
#include <cmath>

namespace amr_navigation
{
namespace core
{

VelocityProfiler::VelocityProfiler(const VelocityProfilerConfig & config)
: config_(config), pid_v_(config.pid_linear), pid_w_(config.pid_angular),
  model_v_(1.0, config.model_time_constant, config.model_delay, config.nominal_period),
  model_w_(1.0, config.model_time_constant, config.model_delay, config.nominal_period)
{
  applyLimits();
}

void VelocityProfiler::resetModels(double v, double w)
{
  model_v_.reset(v);
  model_w_.reset(w);
}

void VelocityProfiler::setConfig(const VelocityProfilerConfig & config)
{
  config_ = config;
  pid_v_.setConfig(config_.pid_linear);
  pid_w_.setConfig(config_.pid_angular);
  model_v_ = FirstOrderPlant(
    1.0, config_.model_time_constant, config_.model_delay, config_.nominal_period,
    model_v_.output());
  model_w_ = FirstOrderPlant(
    1.0, config_.model_time_constant, config_.model_delay, config_.nominal_period,
    model_w_.output());
  applyLimits();
}

void VelocityProfiler::setPayloadScale(double s)
{
  payload_scale_ = std::clamp(s, 0.05, 1.0);
  applyLimits();
}

void VelocityProfiler::applyLimits()
{
  JerkLimits l = config_.linear;
  l.a_max *= payload_scale_;
  if (l.j_max > 0.0) {
    l.j_max *= payload_scale_;
  }
  lin_.setLimits(l);
  JerkLimits sl = l;   // 정지 착지: 같은 가속·저크 한계, 속도 범위는 보정 폭까지
  sl.v_min = std::min(l.v_min, -config_.max_linear_correction);
  sl.v_max = std::max(l.v_max, config_.max_linear_correction);
  stop_.setLimits(sl);
  JerkLimits a = config_.angular;
  a.a_max *= payload_scale_;
  ang_.setLimits(a);
}

void VelocityProfiler::reset(double v, double w)
{
  stopping_ = false;
  out_v1_ = out_v0_ = v;
  lin_.reset(v, 0.0);
  ang_.reset(w, 0.0);
  w_ref_ = w;
  pid_v_.reset();
  pid_w_.reset();
  resetModels(v, w);
  curvature_mode_ = false;
}

ProfilerOutput VelocityProfiler::update(
  double v_target, double w_target, bool has_meas, double v_meas, double w_meas, double dt)
{
  ProfilerOutput out;
  if (dt <= 0.0) {
    out.v_ref = lin_.velocity();
    out.w_ref = w_ref_;
    out.v = out.v_ref;
    out.w = out.w_ref;
    return out;
  }
  // 재동기화 (기준이 실제와 크게 벌어짐)
  if (has_meas && (std::abs(lin_.velocity() - v_meas) > config_.resync_threshold_v ||
    std::abs(w_ref_ - w_meas) > config_.resync_threshold_w))
  {
    lin_.reset(v_meas, 0.0);
    ang_.reset(w_meas, 0.0);
    w_ref_ = w_meas;
    pid_v_.reset();
    pid_w_.reset();
    resetModels(v_meas, w_meas);
    out.resynced = true;
  }

  const auto & L = lin_.limits();
  const auto & A = ang_.limits();
  v_target = std::clamp(v_target, L.v_min, L.v_max);
  w_target = std::clamp(w_target, A.v_min, A.v_max);

  // 선속도: 저크 제한 필터
  const double v_ref = lin_.step(v_target, dt);

  // 각속도: 곡률 보존 또는 저크 제한 필터
  const bool curvature = config_.preserve_curvature &&
    std::abs(v_target) > config_.curvature_min_speed;
  if (curvature) {
    const double kappa = w_target / v_target;
    double w_des = std::clamp(kappa * v_ref, A.v_min, A.v_max);
    const double dw = A.a_max * dt;
    w_ref_ = std::clamp(w_des, w_ref_ - dw, w_ref_ + dw);
    curvature_mode_ = true;
  } else {
    if (curvature_mode_) {
      ang_.reset(w_ref_, 0.0);   // 모드 전환 시 필터 상태를 현재 기준으로
      curvature_mode_ = false;
    }
    w_ref_ = ang_.step(w_target, dt);
  }
  if (curvature) {
    ang_.reset(w_ref_, 0.0);
  }

  out.v_ref = v_ref;
  out.w_ref = w_ref_;
  out.a_ref = lin_.acceleration();

  // 기준 모델: 플랜트가 모델대로면 측정이 따라갈 예상 응답.
  //   이번 주기의 측정은 직전 주기까지의 명령이
  // 만든 응답이므로 이번 기준을 넣기 전의 모델 출력과 비교한다 (한 주기 정렬).
  double y_mv = v_ref;
  double y_mw = w_ref_;
  if (config_.use_reference_model) {
    y_mv = model_v_.output();
    y_mw = model_w_.output();
    model_v_.step(v_ref, dt);
    model_w_.step(w_ref_, dt);
  }

  // 정지 데드밴드: 목표·기준 모두 0 이면 정확히 0 (적분기 잔류로 기어가는 것 방지)
  const double eps = config_.stop_deadband;
  if (std::abs(v_target) <= eps && std::abs(w_target) <= eps &&
    std::abs(v_ref) <= eps && std::abs(w_ref_) <= eps)
  {
    pid_v_.reset();
    pid_w_.reset();
    resetModels(0.0, 0.0);
    if (!stopping_) {
      // 직전 출력 상태에서 0 으로 저크 제한 착지 (PI 잔여 보정이 없으면 이미 0 → 즉시 끝)
      const double a_max = stop_.limits().a_max;
      stop_.reset(out_v1_, std::clamp((out_v1_ - out_v0_) / dt, -a_max, a_max));
      stopping_ = true;
    }
    out.v = std::abs(stop_.velocity()) > 0.0 ||
      stop_.acceleration() != 0.0 ? stop_.step(0.0, dt) : 0.0;
    out_v0_ = out_v1_;
    out_v1_ = out.v;
    return out;   // w = 0
  }
  stopping_ = false;

  if (config_.use_pid && has_meas) {
    const double cv = config_.max_linear_correction;
    const double cw = config_.max_angular_correction;
    out.v = pid_v_.update(
      y_mv, v_meas, dt, v_ref, std::max(L.v_min, v_ref - cv), std::min(L.v_max, v_ref + cv));
    out.w = pid_w_.update(
      y_mw, w_meas, dt, w_ref_, std::max(A.v_min, w_ref_ - cw), std::min(A.v_max, w_ref_ + cw));
  } else {
    out.v = v_ref;
    out.w = w_ref_;
  }
  out_v0_ = out_v1_;
  out_v1_ = out.v;
  return out;
}

}  // namespace core
}  // namespace amr_navigation
