#include "amr_navigation/core/pid.hpp"

#include <algorithm>
#include <cmath>

namespace amr_navigation
{
namespace core
{

Pid::Pid(const PidConfig & config)
: config_(config)
{
}

void Pid::reset(double integral)
{
  integral_ = integral;
  d_filt_ = 0.0;
  has_prev_ = false;
  saturated_ = false;
  u_raw_ = 0.0;
}

double Pid::update(double r, double y, double dt, double ff)
{
  return update(r, y, dt, ff, config_.output_min, config_.output_max);
}

double Pid::update(double r, double y, double dt, double ff, double out_min, double out_max)
{
  if (out_max < out_min) {
    out_max = out_min;
  }
  if (dt <= 0.0) {
    return std::clamp(ff + integral_, out_min, out_max);
  }
  const double e = r - y;
  const double p = config_.kp * (config_.setpoint_weight * r - y);
  double d = 0.0;
  if (config_.kd != 0.0) {
    if (has_prev_) {
      const double dy = (y - prev_y_) / dt;
      const double alpha = dt / (std::max(0.0, config_.derivative_tau) + dt);
      d_filt_ += alpha * (dy - d_filt_);
    }
    d = -config_.kd * d_filt_;   // 측정 미분 (기준 미분 킥 없음)
  }
  prev_y_ = y;
  has_prev_ = true;

  u_raw_ = ff + p + integral_ + d;
  const double u = std::clamp(u_raw_, out_min, out_max);
  saturated_ = (u != u_raw_);
  if (config_.tracking_time > 0.0) {
    // 역계산: 포화량만큼 적분기를 되감는다
    integral_ += (config_.ki * e + (u - u_raw_) / config_.tracking_time) * dt;
  } else {
    // 조건부 적분: 포화된 방향으로 더 밀어내는 적분은 멈춘다
    const bool pushing = saturated_ && ((u_raw_ > u && e > 0.0) || (u_raw_ < u && e < 0.0));
    if (!pushing) {
      integral_ += config_.ki * e * dt;
    }
  }
  integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);
  return u;
}

void Pid::backCalculate(double du, double dt)
{
  if (config_.tracking_time > 0.0 && dt > 0.0) {
    integral_ += du / config_.tracking_time * dt;
    integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);
  }
}

void Pid::addToIntegral(double du)
{
  integral_ = std::clamp(integral_ + du, -config_.integral_limit, config_.integral_limit);
}

FirstOrderPlant::FirstOrderPlant(
  double gain, double time_constant, double delay, double dt, double y0)
: gain_(gain), tau_(time_constant), dt_(dt),
  delay_steps_(static_cast<std::size_t>(std::max(0.0, std::round(delay / dt)))), y_(y0)
{
  reset(y0);
}

void FirstOrderPlant::reset(double y0)
{
  y_ = y0;
  const double u0 = std::abs(gain_) > 1e-12 ? y0 / gain_ : 0.0;
  buffer_.assign(delay_steps_, u0);
}

double FirstOrderPlant::step(double u)
{
  return step(u, dt_);
}

double FirstOrderPlant::step(double u, double dt)
{
  double ud = u;
  if (delay_steps_ > 0) {
    buffer_.push_back(u);
    ud = buffer_.front();
    buffer_.pop_front();
  }
  const double alpha = tau_ > 0.0 ? 1.0 - std::exp(-std::max(0.0, dt) / tau_) : 1.0;
  y_ += alpha * (gain_ * ud - y_);
  return y_;
}

}  // namespace core
}  // namespace amr_navigation
