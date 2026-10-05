#include "amr_navigation/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{

double wrapAngle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pathLength(const std::vector<Pose2D> & path)
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += distance(path[i - 1], path[i]);
  }
  return len;
}

std::vector<double> cumulativeLength(const std::vector<Pose2D> & path)
{
  std::vector<double> s(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    s[i] = s[i - 1] + distance(path[i - 1], path[i]);
  }
  return s;
}

Projection projectOntoPath(
  const std::vector<Pose2D> & path, const Point2D & p, std::size_t begin, std::size_t end,
  const std::vector<double> * cum_s)
{
  Projection best;
  if (path.empty()) {
    return best;
  }
  if (path.size() == 1) {
    best.point = {path[0].x, path[0].y};
    best.heading = path[0].theta;
    best.cte = 0.0;
    return best;
  }
  const std::size_t n = path.size();
  if (end == 0 || end > n) {
    end = n;
  }
  begin = std::min(begin, n - 2);
  // 선분 시작 정점의 배타적 상한 (최소 1개 선분은 검사)
  const std::size_t seg_end = std::min(n - 1, std::max(begin + 1, end - 1));
  double best_d2 = std::numeric_limits<double>::infinity();
  double s_acc = 0.0;
  for (std::size_t i = begin; i < seg_end; ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double bx = path[i + 1].x;
    const double by = path[i + 1].y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = std::clamp(((p.x - ax) * dx + (p.y - ay) * dy) / len2, 0.0, 1.0);
    }
    const double qx = ax + t * dx;
    const double qy = ay + t * dy;
    const double d2 = (p.x - qx) * (p.x - qx) + (p.y - qy) * (p.y - qy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best.segment = i;
      best.t = t;
      best.point = {qx, qy};
      const double seg_len = std::sqrt(len2);
      best.s = (cum_s != nullptr ? (*cum_s)[i] : s_acc) + t * seg_len;
      if (len2 > 1e-12) {
        best.heading = std::atan2(dy, dx);
        // 좌측 법선 (−dy, dx) 방향 성분
        best.cte = ((p.x - qx) * (-dy) + (p.y - qy) * dx) / seg_len;
      } else {
        best.heading = path[i].theta;
        best.cte = 0.0;
      }
    }
    s_acc += std::sqrt(len2);
  }
  return best;
}

Pose2D interpolateAt(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, double s)
{
  if (path.empty()) {
    return Pose2D{};
  }
  if (path.size() == 1 || s <= 0.0) {
    Pose2D out = path.front();
    if (path.size() > 1) {
      out.theta = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);
    }
    return out;
  }
  if (s >= cum_s.back()) {
    Pose2D out = path.back();
    const std::size_t n = path.size();
    const double dx = path[n - 1].x - path[n - 2].x;
    const double dy = path[n - 1].y - path[n - 2].y;
    if (std::hypot(dx, dy) > 1e-9) {
      out.theta = std::atan2(dy, dx);
    }
    return out;
  }
  // 이분 탐색으로 선분 찾기
  const auto it = std::upper_bound(cum_s.begin(), cum_s.end(), s);
  const std::size_t j = static_cast<std::size_t>(std::distance(cum_s.begin(), it));
  const std::size_t i = j - 1;
  const double seg = cum_s[j] - cum_s[i];
  const double t = seg > 1e-12 ? (s - cum_s[i]) / seg : 0.0;
  Pose2D out;
  out.x = path[i].x + t * (path[j].x - path[i].x);
  out.y = path[i].y + t * (path[j].y - path[i].y);
  out.theta = std::atan2(path[j].y - path[i].y, path[j].x - path[i].x);
  return out;
}

std::vector<double> discreteCurvature(const std::vector<Pose2D> & path, double window_m)
{
  const std::size_t n = path.size();
  std::vector<double> kappa(n, 0.0);
  if (n < 3) {
    return kappa;
  }
  if (window_m <= 0.0) {
    for (std::size_t i = 1; i + 1 < n; ++i) {
      const double h0 = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
      const double h1 = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
      const double ds = 0.5 * (distance(path[i - 1], path[i]) + distance(path[i], path[i + 1]));
      kappa[i] = ds > 1e-9 ? wrapAngle(h1 - h0) / ds : 0.0;
    }
    return kappa;
  }
  const std::vector<double> s = cumulativeLength(path);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double s0 = std::max(0.0, s[i] - window_m);
    const double s1 = std::min(s.back(), s[i] + window_m);
    const Pose2D a = interpolateAt(path, s, s0);
    const Pose2D c = interpolateAt(path, s, s1);
    const Pose2D b = path[i];
    const double h0 = std::atan2(b.y - a.y, b.x - a.x);
    const double h1 = std::atan2(c.y - b.y, c.x - b.x);
    const double ds = 0.5 * (s1 - s0);
    if (ds > 1e-9 && distance(a, b) > 1e-9 && distance(b, c) > 1e-9) {
      kappa[i] = wrapAngle(h1 - h0) / ds;
    }
  }
  return kappa;
}

Pose2D integrateArc(const Pose2D & p, double v, double w, double dt)
{
  Pose2D out;
  if (std::abs(w) < 1e-9) {
    out.x = p.x + v * std::cos(p.theta) * dt;
    out.y = p.y + v * std::sin(p.theta) * dt;
    out.theta = p.theta;
    return out;
  }
  const double th1 = p.theta + w * dt;
  const double r = v / w;
  out.x = p.x + r * (std::sin(th1) - std::sin(p.theta));
  out.y = p.y - r * (std::cos(th1) - std::cos(p.theta));
  out.theta = wrapAngle(th1);
  return out;
}

Point2D toRobotFrame(const Pose2D & robot, const Point2D & world)
{
  const double dx = world.x - robot.x;
  const double dy = world.y - robot.y;
  const double c = std::cos(robot.theta);
  const double s = std::sin(robot.theta);
  return {c * dx + s * dy, -s * dx + c * dy};
}

Pose2D compose(const Pose2D & T, const Pose2D & p)
{
  const double c = std::cos(T.theta);
  const double s = std::sin(T.theta);
  return {T.x + c * p.x - s * p.y, T.y + s * p.x + c * p.y, wrapAngle(T.theta + p.theta)};
}

Pose2D inverse(const Pose2D & T)
{
  const double c = std::cos(T.theta);
  const double s = std::sin(T.theta);
  return {-(c * T.x + s * T.y), -(-s * T.x + c * T.y), wrapAngle(-T.theta)};
}

}  // namespace core
}  // namespace amr_navigation
