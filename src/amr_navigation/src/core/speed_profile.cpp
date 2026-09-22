#include "amr_navigation/core/speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{

double SpeedProfile::curvatureCap(double kappa, const SpeedProfileConfig & c)
{
  double v = c.desired_speed;
  const double k = std::abs(kappa);
  if (k > 1e-6) {
    if (c.max_angular_vel > 0.0) {
      v = std::min(v, c.max_angular_vel / k);
    }
    if (c.max_lateral_accel > 0.0) {
      v = std::min(v, std::sqrt(c.max_lateral_accel / k));
    }
  }
  return v;
}

double SpeedProfile::stoppingDistance(double v, double a, double j, double t_c)
{
  v = std::abs(v);
  if (v < 1e-9) {
    return 0.0;
  }
  if (a <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  double d = v * std::max(0.0, t_c);
  if (j <= 0.0 || !std::isfinite(j)) {
    return d + v * v / (2.0 * a);   // 저크 무시 (사다리꼴)
  }
  const double t_r = a / j;                      // 감속도 0 → −a 램프 시간
  const double dv_ramp = 0.5 * j * t_r * t_r;    // 램프 동안 속도 감소 = a²/(2j)
  if (v <= dv_ramp) {
    // 램프 도중 정지: v − j t²/2 = 0
    const double t = std::sqrt(2.0 * v / j);
    return d + v * t - j * t * t * t / 6.0;
  }
  d += v * t_r - j * t_r * t_r * t_r / 6.0;
  const double v1 = v - dv_ramp;
  return d + v1 * v1 / (2.0 * a);
}

double SpeedProfile::maxSpeedForStop(double d, double a, double j, double t_c)
{
  if (d <= 0.0 || a <= 0.0) {
    return 0.0;
  }
  // 상한: 사다리꼴 속도 √(2ad) 는 저크·지연을 넣은 정지거리로는 항상 d 이상 → 해는 [0, √(2ad)]
  double lo = 0.0;
  double hi = std::sqrt(2.0 * a * d);
  for (int i = 0; i < 40; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (stoppingDistance(mid, a, j, t_c) <= d) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

void SpeedProfile::build(
  const std::vector<Pose2D> & path, const SpeedProfileConfig & config, bool end_is_goal)
{
  config_ = config;
  end_is_goal_ = end_is_goal;
  s_ = cumulativeLength(path);
  kappa_ = discreteCurvature(path, config.curvature_window);
  cap_.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    cap_[i] = curvatureCap(kappa_[i], config);
  }
  if (end_is_goal && !cap_.empty()) {
    cap_.back() = 0.0;
  }
}

double SpeedProfile::allowedSpeed(double s, double horizon) const
{
  if (s_.empty()) {
    return 0.0;
  }
  double v = config_.desired_speed;
  const double d = std::max(1e-6, config_.max_decel);
  const auto it = std::lower_bound(s_.begin(), s_.end(), s);
  for (auto k = static_cast<std::size_t>(std::distance(s_.begin(), it)); k < s_.size(); ++k) {
    const double ds = s_[k] - s;
    if (horizon > 0.0 && ds > horizon) {
      break;
    }
    if (end_is_goal_ && k + 1 == s_.size()) {
      // 목표 정지: 저크·지연을 넣은 정지거리 역함수 (하류 필터가 늦게 감속해도 목표 앞에서 선다)
      v = std::min(
        v, maxSpeedForStop(std::max(0.0, ds), d, config_.max_jerk, config_.stop_latency));
    } else {
      v = std::min(v, std::sqrt(cap_[k] * cap_[k] + 2.0 * d * std::max(0.0, ds)));
    }
  }
  return v;
}

std::vector<double> SpeedProfile::velocityProfile(double v0) const
{
  const std::size_t n = s_.size();
  std::vector<double> v(cap_);
  if (n == 0) {
    return v;
  }
  v[0] = std::min(v[0], std::max(0.0, v0));
  const double a = std::max(1e-6, config_.max_accel);
  const double d = std::max(1e-6, config_.max_decel);
  for (std::size_t i = 1; i < n; ++i) {
    const double ds = s_[i] - s_[i - 1];
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * a * ds));
  }
  for (std::size_t i = n - 1; i-- > 0; ) {
    const double ds = s_[i + 1] - s_[i];
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * d * ds));
  }
  return v;
}

double SpeedProfile::rotationTime(double angle, double w_max, double alpha_max)
{
  angle = std::abs(angle);
  if (angle < 1e-9 || w_max <= 0.0 || alpha_max <= 0.0) {
    return 0.0;
  }
  if (angle >= w_max * w_max / alpha_max) {
    return angle / w_max + w_max / alpha_max;
  }
  return 2.0 * std::sqrt(angle / alpha_max);
}

double SpeedProfile::predictTravelTime(
  double v0, double start_heading_error, double goal_heading_error) const
{
  if (s_.size() < 2) {
    return rotationTime(start_heading_error, config_.max_angular_vel, config_.max_angular_accel) +
           rotationTime(goal_heading_error, config_.max_angular_vel, config_.max_angular_accel);
  }
  const std::vector<double> v = velocityProfile(v0);
  constexpr double kEps = 1e-3;
  double t = 0.0;
  for (std::size_t i = 1; i < s_.size(); ++i) {
    const double ds = s_[i] - s_[i - 1];
    if (ds <= 0.0) {
      continue;
    }
    t += 2.0 * ds / std::max(v[i - 1] + v[i], kEps);
  }
  if (config_.max_jerk > 0.0) {
    t += config_.max_accel / config_.max_jerk;   // 정지-출발 1쌍의 S-curve 시간 증가
  }
  t += rotationTime(start_heading_error, config_.max_angular_vel, config_.max_angular_accel);
  t += rotationTime(goal_heading_error, config_.max_angular_vel, config_.max_angular_accel);
  return t;
}

}  // namespace core
}  // namespace amr_navigation
