#include "amr_navigation/core/jerk_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace amr_navigation
{
namespace core
{

JerkLimitedFilter::JerkLimitedFilter(const JerkLimits & limits)
: limits_(limits)
{
}

void JerkLimitedFilter::reset(double v, double a)
{
  v_ = v;
  a_ = a;
  last_jerk_ = 0.0;
}

double JerkLimitedFilter::step(double v_ref, double dt)
{
  if (dt <= 0.0) {
    return v_;
  }
  v_ref = std::clamp(v_ref, limits_.v_min, limits_.v_max);
  const double a_max = std::max(1e-9, limits_.a_max);
  const double j = limits_.j_max;
  const double a_prev = a_;

  if (!(j > 0.0) || !std::isfinite(j)) {
    // 사다리꼴: 가속도 한계만
    const double dv = std::clamp(v_ref - v_, -a_max * dt, a_max * dt);
    v_ += dv;
    a_ = dv / dt;
    last_jerk_ = (a_ - a_prev) / dt;
    return v_;
  }

  // 종단 스냅
  if (std::abs(v_ref - v_) <= j * dt * dt && std::abs(a_) <= j * dt) {
    v_ = v_ref;
    a_ = 0.0;
    last_jerk_ = (a_ - a_prev) / dt;
    return v_;
  }
  const double c = v_ref - v_ - 0.5 * a_ * dt;
  const double a_star = (c >= 0.0 ? 1.0 : -1.0) * j *
    (-0.5 * dt + std::sqrt(0.25 * dt * dt + 2.0 * std::abs(c) / j));
  const double jerk = std::clamp((a_star - a_) / dt, -j, j);
  double a1 = std::clamp(a_ + jerk * dt, -a_max, a_max);
  // 한계가 줄어든 직후(|a| > a_max) 에도 저크 한계를 지키며 줄인다
  a1 = std::clamp(a1, a_ - j * dt, a_ + j * dt);
  v_ += 0.5 * (a_ + a1) * dt;
  a_ = a1;
  last_jerk_ = (a_ - a_prev) / dt;
  return v_;
}

}  // namespace core
}  // namespace amr_navigation
