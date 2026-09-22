// 시험용 좁은 통로 장면: 순폭 0.60 m 통로(x ∈ [3, 7], 벽 y = ±0.30)를
// LiDAR(σ 0.03, 0.5°, lidar_link 0.15 앞)로 10 Hz 관측해 코스트맵을 매 스캔 새로 만든다
// (영속 0, 끝점 = 치명 셀, Nav2 식 팽창 0.2 / 0.8 / 3.0). denoise = true 면 끝점을
// core::denoiseRanges (costmap_scan_filter_node 와 같은 기본값) 뒤에 찍는다.
// 참값 판정: 로봇 사각형(0.60 x 0.40) ↔ 랙 사각형 최소 거리.
#ifndef AISLE_SCENE_HPP_
#define AISLE_SCENE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/scan_denoise.hpp"

namespace aisle
{
using amr_navigation::core::FootprintChecker;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::Pose2D;

struct Box
{
  double x0, x1, y0, y1;
};

inline const std::vector<Box> & racks()
{
  static const std::vector<Box> r{{3.0, 7.0, 0.30, 1.30}, {3.0, 7.0, -1.30, -0.30}};
  return r;
}

// 광선-사각형 교차 (slab). 없으면 +inf
inline double rayBox(double ox, double oy, double dx, double dy, const Box & b)
{
  double t0 = 0.0;
  double t1 = std::numeric_limits<double>::infinity();
  const double o[2] = {ox, oy};
  const double d[2] = {dx, dy};
  const double lo[2] = {b.x0, b.y0};
  const double hi[2] = {b.x1, b.y1};
  for (int k = 0; k < 2; ++k) {
    if (std::abs(d[k]) < 1e-12) {
      if (o[k] < lo[k] || o[k] > hi[k]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double a = (lo[k] - o[k]) / d[k];
    double c = (hi[k] - o[k]) / d[k];
    if (a > c) {
      std::swap(a, c);
    }
    t0 = std::max(t0, a);
    t1 = std::min(t1, c);
    if (t0 > t1) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return t0 > 0.0 ? t0 : std::numeric_limits<double>::infinity();
}

struct Scene
{
  double resolution{0.025};
  bool denoise{true};
  double sigma{0.03};
  std::mt19937 rng{11};
  // resolution 에 맞춰 build() 가 다시 만든다
  OwnedGrid grid{360 * 2, 120 * 2, 0.025, 0, -0.5, -1.5};
  std::vector<amr_navigation::core::Point2D> fp = FootprintChecker::rectangle(0.60, 0.40);

  // 자세 pose 에서 한 번 관측해 코스트맵을 새로 만든다
  void observe(const Pose2D & pose)
  {
    const int w = static_cast<int>(std::lround(9.0 / resolution));
    const int h = static_cast<int>(std::lround(3.0 / resolution));
    grid = OwnedGrid(w, h, resolution, 0, -0.5, -1.5);
    const double lx = pose.x + 0.15 * std::cos(pose.theta);
    const double ly = pose.y + 0.15 * std::sin(pose.theta);
    std::normal_distribution<double> nd(0.0, sigma);
    std::vector<float> ranges(720, std::numeric_limits<float>::infinity());
    const double inc = 2.0 * M_PI / 720.0;
    for (int i = 0; i < 720; ++i) {
      const double a = pose.theta - M_PI + i * inc;
      double r = std::numeric_limits<double>::infinity();
      for (const auto & b : racks()) {
        r = std::min(r, rayBox(lx, ly, std::cos(a), std::sin(a), b));
      }
      if (std::isfinite(r) && r < 25.0) {
        ranges[static_cast<std::size_t>(i)] = static_cast<float>(r + nd(rng));
      }
    }
    if (denoise) {
      ranges = amr_navigation::core::denoiseRanges(
        ranges,
        amr_navigation::core::ScanDenoiseConfig(), true);
    }
    for (int i = 0; i < 720; ++i) {
      const float r = ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(r)) {
        continue;
      }
      const double a = pose.theta - M_PI + i * inc;
      int mx = 0;
      int my = 0;
      if (grid.view().worldToMap(lx + r * std::cos(a), ly + r * std::sin(a), mx, my)) {
        grid.at(mx, my) = amr_navigation::core::kLethalObstacle;
      }
    }
    amr_navigation::core::inflate(grid, 0.2, 0.8, 3.0);
  }

  FootprintChecker checker() const
  {
    return FootprintChecker(
      grid.view(), fp,
      amr_navigation::core::inflationCost(FootprintChecker::circumscribedRadius(fp), 0.2, 3.0));
  }
};

// 로봇 사각형 ↔ 랙 사각형 참값 최소 거리 (겹치면 0)
inline double trueClearance(const Pose2D & p)
{
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  double best = std::numeric_limits<double>::infinity();
  for (const auto & b : racks()) {
    // 로봇 꼭짓점 → 랙
    for (const auto & lc :
      {std::pair<double, double>{0.3, 0.2}, {0.3, -0.2}, {-0.3, -0.2}, {-0.3, 0.2}})
    {
      const double x = p.x + c * lc.first - s * lc.second;
      const double y = p.y + s * lc.first + c * lc.second;
      best =
        std::min(
        best,
        std::hypot(std::max({b.x0 - x, 0.0, x - b.x1}), std::max({b.y0 - y, 0.0, y - b.y1})));
    }
    // 랙 꼭짓점 → 로봇
    for (const auto & bc :
      {std::pair<double, double>{b.x0, b.y0}, {b.x0, b.y1}, {b.x1, b.y0}, {b.x1, b.y1}})
    {
      const double dx = bc.first - p.x;
      const double dy = bc.second - p.y;
      const double lx = c * dx + s * dy;
      const double ly = -s * dx + c * dy;
      best =
        std::min(
        best,
        std::hypot(std::max(0.0, std::abs(lx) - 0.3), std::max(0.0, std::abs(ly) - 0.2)));
    }
  }
  return best;
}
}  // namespace aisle

#endif  // AISLE_SCENE_HPP_
