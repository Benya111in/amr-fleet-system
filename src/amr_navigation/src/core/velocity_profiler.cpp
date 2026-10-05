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
  JerkLimits a = config_.angular;
  a.a_max *= payload_scale_;
  ang_.setLimits(a);
}

void VelocityProfiler::reset(double v, double w)
{
  lin_.reset(v, 0.0);
  ang_.reset(w, 0.0);
  w_ref_ = w;
  pid_v_.reset();
  pid_w_.reset();
  resetModels(v, w);
  curvature_mode_ = false;
  out_v_ = v;
  out_a_ = 0.0;
  out_a_prev_ = 0.0;
  has_prev_a_ = false;
  corr_v_ = 0.0;
  corr_w_ = 0.0;
  filt_init_ = false;
  hold_elapsed_ = 0.0;
}

void VelocityProfiler::resync(double v, double w)
{
  reset(v, w);
  ++resync_count_;
}

void VelocityProfiler::shapeOutput(double target, double a_target, double dt)
{
  const JerkLimits & L = lin_.limits();
  const double a_max = std::max(1e-9, L.a_max);
  const double j = L.j_max;
  if (!(j > 0.0) || !std::isfinite(j)) {
    // 사다리꼴 모드: 가속 한계만
    const double dv = std::clamp(target - out_v_, -a_max * dt, a_max * dt);
    out_v_ += dv;
    out_a_prev_ = out_a_;
    out_a_ = dv / dt;
    return;
  }
  // 상대 착지: b1 = a_u(k) − a_target 를 고르면 이번 스텝 뒤 오차 e = E0 − b1·dt/2.
  // 이후 최대 저크로 b → 0 할 때 e 가 정확히 0 이 되는 b1 (JerkLimitedFilter 와 같은 폐형식).
  const double e0 = target - out_v_ - 0.5 * (out_a_ + a_target) * dt;
  const double b_star = (e0 >= 0.0 ? 1.0 : -1.0) * j *
    (-0.5 * dt + std::sqrt(0.25 * dt * dt + 2.0 * std::abs(e0) / j));
  double a1 = std::clamp(a_target + b_star, out_a_ - j * dt, out_a_ + j * dt);
  // 주기가 들쭉날쭉할 때: 명령 열의 수치 가속도 (u_k − u_{k−1})/dt_k = (a_{k−1} + a_k)/2 의
  // 차분이 (a_k − a_{k−2})/(2 dt_k) 이므로 |a_k − a_{k−2}| ≤ 2 j dt_k 도 지킨다
  // (등간격이면 위 조건에 포함)
  if (has_prev_a_) {
    a1 = std::clamp(a1, out_a_prev_ - 2.0 * j * dt, out_a_prev_ + 2.0 * j * dt);
  }
  a1 = std::clamp(a1, -a_max, a_max);
  out_v_ += 0.5 * (out_a_ + a1) * dt;
  out_a_prev_ = out_a_;
  has_prev_a_ = true;
  out_a_ = a1;
}

ProfilerOutput VelocityProfiler::update(
  double v_target, double w_target, bool has_meas, double v_meas, double w_meas, double dt,
  bool fresh)
{
  ProfilerOutput out;
  if (dt <= 0.0) {
    out.v_ref = lin_.velocity();
    out.w_ref = w_ref_;
    out.v = out_v_;
    out.w = std::clamp(w_ref_ + corr_w_, ang_.limits().v_min, ang_.limits().v_max);
    out.a_out = out_a_;
    return out;
  }
  const bool meas_new = has_meas && fresh;
  // --- 재동기화: 명령이 실행되지 않음 ---
  if (has_meas) {
    bool now = std::abs(lin_.velocity() - v_meas) > config_.resync_threshold_v ||
      std::abs(w_ref_ - w_meas) > config_.resync_threshold_w;
    if (!now && meas_new) {
      // 모델 응답(기준이 명목 플랜트면 낼 속도)과 측정의 지속 괴리
      // = 하류(safety_node)가 붙잡거나 깎음.
      // 느린·슬립 플랜트(측정/모델 ≥ 0.6)는 붙잡힘이 아니다 — PI 가 보정할 몫
      const double y_m = model_v_.output();
      const bool held =
        (std::abs(y_m - v_meas) > config_.hold_error_v && std::abs(v_meas) < 0.6 * std::abs(y_m)) ||
        (std::abs(v_meas) < 0.01 && std::abs(y_m) > 0.04);
      hold_elapsed_ = held ? hold_elapsed_ + dt : 0.0;
      now = hold_elapsed_ >= config_.hold_time;
    }
    if (now) {
      resync(v_meas, w_meas);
      out.resynced = true;
    }
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

  // 정지: 목표·기준 모두 0 → PI 끔, 남은 보정은 성형이 저크 한계로 0 에 착지
  const double eps = config_.stop_deadband;
  const bool stopping = std::abs(v_target) <= eps && std::abs(w_target) <= eps &&
    std::abs(v_ref) <= eps && std::abs(w_ref_) <= eps;
  bool pi_v_updated = false;
  if (stopping || !config_.use_pid || !has_meas) {
    pid_v_.reset();
    pid_w_.reset();
    corr_v_ = 0.0;
    corr_w_ = 0.0;
    filt_init_ = false;
    if (stopping) {
      resetModels(0.0, 0.0);
    }
  } else if (meas_new) {
    // 측정·모델 응답 공통 저역 (잡음이 kp 로 명령에 새지 않게; 같은 필터라 추종 지연은 상쇄)
    const double tf = std::max(0.0, config_.measurement_filter_time);
    const double alpha = dt / (tf + dt);
    if (!filt_init_) {
      filt_y_ = v_meas;
      filt_ym_ = y_mv;
      filt_yw_ = w_meas;
      filt_ymw_ = y_mw;
      filt_init_ = true;
    } else {
      filt_y_ += alpha * (v_meas - filt_y_);
      filt_ym_ += alpha * (y_mv - filt_ym_);
      filt_yw_ += alpha * (w_meas - filt_yw_);
      filt_ymw_ += alpha * (y_mw - filt_ymw_);
    }
    const double cv = config_.max_linear_correction;
    const double cw = config_.max_angular_correction;
    corr_v_ = pid_v_.update(filt_ym_, filt_y_, dt, 0.0, -cv, cv);
    corr_w_ = pid_w_.update(filt_ymw_, filt_yw_, dt, 0.0, -cw, cw);
    pi_v_updated = true;
  }
  // (신선하지 않은 측정: corr 유지, 적분 없음)

  // 선속도 출력 성형: 목표 v_ref + corr, 피드포워드 가속도 a_ref
  const double target = std::clamp(v_ref + corr_v_, L.v_min, L.v_max);
  shapeOutput(target, lin_.acceleration(), dt);
  if (pi_v_updated) {
    // 역계산: 성형이 실제로 더한 보정 (u − v_ref) 과 PI 출력의 차만큼 적분기를 그 자리에서 되감는다
    // (추종 시정수 0: 가속 예산을 기준 램프가 다 쓰는 동안 적분이 쌓였다가 램프 끝에
    // 튀어나오지 않게)
    pid_v_.addToIntegral((out_v_ - v_ref) - corr_v_);
  }
  // 정지 착지 끝: 남은 값이 한 스텝 저크 여유(j·dt²/2, j·dt/2) 안이면 정확히 0 (수치 저크 ≤ j 유지)
  const double jd = L.j_max > 0.0 ? L.j_max : L.a_max / dt;
  if (stopping && std::abs(out_v_) <= 0.5 * jd * dt * dt && std::abs(out_a_) <= 0.5 * jd * dt) {
    out_v_ = 0.0;
    out_a_prev_ = out_a_;
    out_a_ = 0.0;
  }
  out.v = out_v_;
  out.a_out = out_a_;
  out.correction = out_v_ - v_ref;
  out.w = stopping ? 0.0 : std::clamp(w_ref_ + corr_w_, A.v_min, A.v_max);
  return out;
}

}  // namespace core
}  // namespace amr_navigation
