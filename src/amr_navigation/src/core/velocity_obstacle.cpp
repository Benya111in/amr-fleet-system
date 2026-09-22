#include "amr_navigation/core/velocity_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{

double DynamicObstacle::speed() const
{
  return std::hypot(vx, vy);
}

Point2D chordVelocity(double theta, double v, double w, double tau)
{
  const double half = 0.5 * w * tau;
  double mag = v;
  if (std::abs(half) > 1e-9) {
    mag = v * std::sin(half) / half;
  }
  const double dir = theta + half;
  return {mag * std::cos(dir), mag * std::sin(dir)};
}

bool inVelocityObstacle(const Point2D & p, const Point2D & w_rel, double R, double tau)
{
  const double pn2 = p.x * p.x + p.y * p.y;
  const double closing = p.x * w_rel.x + p.y * w_rel.y;   // > 0 이면 접근
  if (pn2 < R * R) {
    return closing > 0.0;
  }
  const double wn2 = w_rel.x * w_rel.x + w_rel.y * w_rel.y;
  if (wn2 < 1e-12 || closing <= 0.0) {
    return false;   // 정지 상대 운동이거나 멀어지는 중
  }
  const double t_star = std::clamp(closing / wn2, 0.0, tau);
  const double dx = p.x - t_star * w_rel.x;
  const double dy = p.y - t_star * w_rel.y;
  return dx * dx + dy * dy < R * R;
}

double velocityObstacleTime(const Point2D & p, const Point2D & w_rel, double R, double tau)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const double pn2 = p.x * p.x + p.y * p.y;
  const double closing = p.x * w_rel.x + p.y * w_rel.y;
  if (pn2 < R * R) {
    return closing > 0.0 ? 0.0 : kInf;
  }
  const double wn2 = w_rel.x * w_rel.x + w_rel.y * w_rel.y;
  if (wn2 < 1e-12 || closing <= 0.0) {
    return kInf;
  }
  // ‖p − t·w‖² = R² 의 작은 근
  const double disc = closing * closing - wn2 * (pn2 - R * R);
  if (disc <= 0.0) {   // 접선(최근접 = R)은 inVelocityObstacle 과 같이 밖으로 본다
    return kInf;
  }
  const double t = (closing - std::sqrt(disc)) / wn2;
  return t <= tau ? std::max(0.0, t) : kInf;
}

double firstContactTime(
  const std::vector<Pose2D> & traj, double dt, const DynamicObstacle & obs, double R)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  if (traj.empty()) {
    return kInf;
  }
  // 상대 위치 r(t) = robot(t) − obs(t). 구간 [t_k, t_k+1] 에서 선형: r = r0 + s·dr, s∈[0,1]
  auto rel = [&](std::size_t k) {
      const Point2D o = obs.predict(static_cast<double>(k) * dt);
      return Point2D{traj[k].x - o.x, traj[k].y - o.y};
    };
  Point2D r0 = rel(0);
  if (r0.x * r0.x + r0.y * r0.y < R * R) {
    return 0.0;
  }
  for (std::size_t k = 0; k + 1 < traj.size(); ++k) {
    const Point2D r1 = rel(k + 1);
    const double dx = r1.x - r0.x;
    const double dy = r1.y - r0.y;
    const double a = dx * dx + dy * dy;
    const double b = 2.0 * (r0.x * dx + r0.y * dy);
    const double c = r0.x * r0.x + r0.y * r0.y - R * R;
    if (a > 1e-12) {
      const double disc = b * b - 4.0 * a * c;
      if (disc >= 0.0) {
        const double s = (-b - std::sqrt(disc)) / (2.0 * a);   // 먼저 들어가는 근
        if (s >= 0.0 && s <= 1.0) {
          return (static_cast<double>(k) + s) * dt;
        }
      }
    }
    r0 = r1;
  }
  return kInf;
}

}  // namespace core
}  // namespace amr_navigation
