// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 2D 기하 도우미 구현.

#include "amr_perception/geometry2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_perception
{

Mat2 Pose2D::rotation() const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Mat2 r;
  r << c, -s, s, c;
  return r;
}

Vec2 Pose2D::rotate(const Vec2 & v) const
{
  return rotation() * v;
}

Vec2 Pose2D::apply(const Vec2 & p) const
{
  return rotate(p) + translation();
}

Pose2D Pose2D::compose(const Pose2D & other) const
{
  const Vec2 t = apply(other.translation());
  return Pose2D{t.x(), t.y(), normalizeAngle(yaw + other.yaw)};
}

Pose2D Pose2D::inverse() const
{
  const Mat2 rt = rotation().transpose();
  const Vec2 t = -(rt * translation());
  return Pose2D{t.x(), t.y(), normalizeAngle(-yaw)};
}

double normalizeAngle(double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

double distanceToRectangle(const Vec2 & p, double length, double width)
{
  // 사각형 밖으로 튀어나온 성분만 남기면 가장 가까운 변/꼭짓점까지의 벡터가 된다.
  const double dx = std::max(std::abs(p.x()) - 0.5 * length, 0.0);
  const double dy = std::max(std::abs(p.y()) - 0.5 * width, 0.0);
  return std::hypot(dx, dy);
}

double distanceToSegment(const Vec2 & p, const Vec2 & a, const Vec2 & b)
{
  const Vec2 ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 <= std::numeric_limits<double>::epsilon()) {
    return (p - a).norm();
  }
  const double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

bool pointInPolygon(const Vec2 & p, const std::vector<Vec2> & polygon)
{
  bool inside = false;
  const std::size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Vec2 & a = polygon[i];
    const Vec2 & b = polygon[j];
    const bool crosses = (a.y() > p.y()) != (b.y() > p.y());
    if (crosses) {
      const double x_int = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (p.x() < x_int) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double distanceToPolygon(const Vec2 & p, const std::vector<Vec2> & polygon)
{
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polygon.size() == 1) {
    return (p - polygon.front()).norm();
  }
  if (pointInPolygon(p, polygon)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Vec2 & a = polygon[i];
    const Vec2 & b = polygon[(i + 1) % polygon.size()];
    best = std::min(best, distanceToSegment(p, a, b));
  }
  return best;
}

}  // namespace amr_perception
