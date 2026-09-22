#include "amr_navigation/core/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace amr_navigation
{
namespace core
{

FootprintChecker::FootprintChecker(
  const CostGrid & grid, std::vector<Point2D> footprint, uint8_t circumscribed_cost,
  bool allow_unknown)
: grid_(grid), footprint_(std::move(footprint)), circumscribed_cost_(circumscribed_cost),
  allow_unknown_(allow_unknown)
{
}

uint8_t FootprintChecker::centerCost(double x, double y) const
{
  return grid_.costAtWorld(x, y);
}

double FootprintChecker::cost(const Pose2D & pose) const
{
  const uint8_t c0 = centerCost(pose.x, pose.y);
  if (c0 == kNoInformation) {
    if (!allow_unknown_) {
      return -1.0;
    }
  } else if (c0 >= kInscribedInflated) {
    return -1.0;
  } else if (c0 < circumscribed_cost_) {
    return static_cast<double>(c0);
  }
  if (footprint_.size() < 2) {
    return c0 == kNoInformation ? 0.0 : static_cast<double>(c0);
  }
  // 외곽선 래스터화
  const double cs = std::cos(pose.theta);
  const double sn = std::sin(pose.theta);
  std::vector<Cell> verts;
  verts.reserve(footprint_.size());
  for (const auto & p : footprint_) {
    const double wx = pose.x + cs * p.x - sn * p.y;
    const double wy = pose.y + sn * p.x + cs * p.y;
    Cell c;
    grid_.worldToMap(wx, wy, c.x, c.y);   // 범위 밖이어도 좌표는 채워진다 → 아래 atOrLethal
    verts.push_back(c);
  }
  uint8_t max_cost = c0 == kNoInformation ? 0 : c0;
  bool hit = false;
  for (std::size_t i = 0; i < verts.size() && !hit; ++i) {
    const Cell & a = verts[i];
    const Cell & b = verts[(i + 1) % verts.size()];
    traceLine(
      a.x, a.y, b.x, b.y, [&](int x, int y) {
        const uint8_t c = grid_.atOrLethal(x, y);
        if (c == kLethalObstacle || (c == kNoInformation && !allow_unknown_)) {
          hit = true;
          return false;
        }
        if (c != kNoInformation) {
          max_cost = std::max(max_cost, c);
        }
        return true;
      });
  }
  if (hit) {
    return -1.0;
  }
  return static_cast<double>(max_cost);
}

double FootprintChecker::inscribedRadius(const std::vector<Point2D> & fp)
{
  // 원점에서 각 변(선분)까지의 최소 거리
  double r = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < fp.size(); ++i) {
    const Point2D & a = fp[i];
    const Point2D & b = fp[(i + 1) % fp.size()];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = std::clamp(-(a.x * dx + a.y * dy) / len2, 0.0, 1.0);
    }
    r = std::min(r, std::hypot(a.x + t * dx, a.y + t * dy));
  }
  return fp.empty() ? 0.0 : r;
}

double FootprintChecker::circumscribedRadius(const std::vector<Point2D> & fp)
{
  double r = 0.0;
  for (const auto & p : fp) {
    r = std::max(r, std::hypot(p.x, p.y));
  }
  return r;
}

std::vector<Point2D> FootprintChecker::rectangle(double length, double width)
{
  const double hx = 0.5 * length;
  const double hy = 0.5 * width;
  return {{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}};
}

}  // namespace core
}  // namespace amr_navigation
