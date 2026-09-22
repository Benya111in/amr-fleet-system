#include "amr_navigation/core/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{
namespace
{
double polylineLength(const std::vector<Point2D> & pts)
{
  double len = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    len += distance(pts[i - 1], pts[i]);
  }
  return len;
}
}  // namespace

PathSmoother::PathSmoother(const SmootherConfig & config)
: config_(config)
{
}

bool PathSmoother::cellAllowed(uint8_t c) const
{
  if (c == kNoInformation) {
    return config_.allow_unknown;
  }
  return c <= config_.max_allowed_cost;
}

int PathSmoother::cellValue(uint8_t c) const
{
  if (c == kNoInformation) {
    return config_.allow_unknown ? 0 : static_cast<int>(kNoInformation);
  }
  return static_cast<int>(c);
}

int PathSmoother::segmentMaxCost(const CostGrid & grid, const Point2D & a, const Point2D & b) const
{
  int mx = 0;
  traceSupercover(
    grid, a.x, a.y, b.x, b.y, [&](int x, int y) {
      mx = std::max(mx, cellValue(grid.atOrLethal(x, y)));
      return true;
    });
  return mx;
}

std::vector<Point2D> PathSmoother::cellsToWorld(
  const CostGrid & grid, const std::vector<Cell> & cells)
{
  std::vector<Point2D> out;
  out.reserve(cells.size());
  for (const auto & c : cells) {
    Point2D p;
    grid.mapToWorld(c.x, c.y, p.x, p.y);
    out.push_back(p);
  }
  return out;
}

bool PathSmoother::segmentFree(const CostGrid & grid, const Point2D & a, const Point2D & b) const
{
  return traceSupercover(
    grid, a.x, a.y, b.x, b.y, [&](int x, int y) {
      return grid.inBounds(x, y) && cellAllowed(grid.at(x, y));
    });
}

double PathSmoother::segmentCost(
  const CostGrid & grid, const Point2D & a, const Point2D & b,
  const std::array<float, 256> & multiplier)
{
  const double len_cells = distance(a, b) / grid.resolution;
  if (len_cells < 1e-9) {
    return 0.0;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(len_cells / 0.5)));
  const double ds = len_cells / steps;
  double cost = 0.0;
  for (int k = 0; k < steps; ++k) {
    const double t = (k + 0.5) / steps;   // 중점 규칙
    const double wx = a.x + t * (b.x - a.x);
    const double wy = a.y + t * (b.y - a.y);
    const float m = multiplier[grid.costAtWorld(wx, wy)];
    if (!(m < std::numeric_limits<float>::infinity())) {
      return std::numeric_limits<double>::infinity();
    }
    cost += ds * static_cast<double>(m);
  }
  return cost;
}

std::vector<std::size_t> PathSmoother::shortcut(
  const CostGrid & grid, const std::vector<Point2D> & pts,
  const std::array<float, 256> & multiplier) const
{
  const std::size_t n = pts.size();
  std::vector<std::size_t> keep;
  if (n == 0) {
    return keep;
  }
  // 원래 경로의 누적 비용 G_k (A* 와 같은 가중: 도착 셀 비용 × 길이)
  std::vector<double> G(n, 0.0);
  for (std::size_t k = 1; k < n; ++k) {
    const float m = multiplier[grid.costAtWorld(pts[k].x, pts[k].y)];
    const double mm = m < std::numeric_limits<float>::infinity() ? static_cast<double>(m) :
      static_cast<double>(multiplier[kMaxNonObstacle]);
    G[k] = G[k - 1] + distance(pts[k - 1], pts[k]) / grid.resolution * mm;
  }
  const double ratio = 1.0 + std::max(0.0, config_.shortcut_cost_ratio);
  std::size_t i = 0;
  keep.push_back(0);
  while (i + 1 < n) {
    std::size_t best = i + 1;
    // 원래 경로 구간 [i, j] 의 최대 셀 비용 (여유거리 천장)
    int local_max = std::max(
      cellValue(grid.costAtWorld(pts[i].x, pts[i].y)),
      cellValue(grid.costAtWorld(pts[i + 1].x, pts[i + 1].y)));
    for (std::size_t j = i + 2; j < n; ++j) {
      local_max = std::max(local_max, cellValue(grid.costAtWorld(pts[j].x, pts[j].y)));
      if (distance(pts[i], pts[j]) > config_.shortcut_max_length) {
        break;
      }
      if (!segmentFree(grid, pts[i], pts[j])) {
        break;
      }
      if (segmentMaxCost(grid, pts[i], pts[j]) > local_max + config_.clearance_cost_margin) {
        break;
      }
      const double chord = segmentCost(grid, pts[i], pts[j], multiplier);
      if (chord > ratio * (G[j] - G[i]) + 1e-9) {
        break;
      }
      best = j;
    }
    keep.push_back(best);
    i = best;
  }
  return keep;
}

std::vector<Point2D> PathSmoother::resample(const std::vector<Point2D> & pts, double spacing)
{
  if (pts.size() < 2 || spacing <= 0.0) {
    return pts;
  }
  std::vector<Point2D> out;
  out.push_back(pts.front());
  double carry = 0.0;   // 마지막 출력점 이후 누적 거리
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const Point2D a = pts[i - 1];
    const Point2D b = pts[i];
    const double seg = distance(a, b);
    if (seg < 1e-12) {
      continue;
    }
    double s = spacing - carry;   // 이 선분에서 다음 표본까지의 거리
    while (s < seg - 1e-9) {
      const double t = s / seg;
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
      s += spacing;
    }
    carry = seg - (s - spacing);
  }
  if (distance(out.back(), pts.back()) > 1e-9) {
    // 끝점이 직전 표본과 너무 가까우면 직전 표본을 끝점으로 대체 (간격 < spacing/2 방지)
    if (out.size() > 1 && distance(out.back(), pts.back()) < 0.5 * spacing) {
      out.back() = pts.back();
    } else {
      out.push_back(pts.back());
    }
  }
  return out;
}

int PathSmoother::smooth(
  const CostGrid & grid, std::vector<Point2D> & pts,
  bool & rolled_back) const
{
  rolled_back = false;
  const std::size_t n = pts.size();
  if (n < 3) {
    return 0;
  }
  const std::vector<Point2D> x = pts;   // 데이터 항 기준
  std::vector<Point2D> prev = pts;
  // 입력에서 이미 막힌 선분(시작점이 INSCRIBED 구역 안인 경우 등)은 가드에서 제외한다
  std::vector<char> exempt(n - 1, 0);
  std::vector<int> seg_max(n - 1, 0);
  for (std::size_t i = 1; i < n; ++i) {
    exempt[i - 1] = segmentFree(grid, pts[i - 1], pts[i]) ? 0 : 1;
    seg_max[i - 1] = segmentMaxCost(grid, pts[i - 1], pts[i]);
  }
  // 점별 여유거리 천장: 입력에서 점 i 에 닿는 두 선분의 최대 셀 비용
  std::vector<int> ceiling(n, 0);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    ceiling[i] = std::max(seg_max[i - 1], seg_max[i]);
  }
  const double wd = config_.w_data;
  const double ws = config_.w_smooth;
  const double wc = config_.w_clearance;
  const double res = grid.resolution;
  const double thr = static_cast<double>(config_.clearance_cost_threshold);
  auto cost_value = [&](int mx, int my) -> double {
      const uint8_t c = grid.atOrLethal(mx, my);
      return c == kNoInformation ? 0.0 : static_cast<double>(c);
    };
  int it = 0;
  for (; it < config_.max_iterations; ++it) {
    double change = 0.0;
    for (std::size_t i = 1; i + 1 < n; ++i) {
      Point2D y = pts[i];
      double nx = y.x + wd * (x[i].x - y.x) + ws * (pts[i - 1].x + pts[i + 1].x - 2.0 * y.x);
      double ny = y.y + wd * (x[i].y - y.y) + ws * (pts[i - 1].y + pts[i + 1].y - 2.0 * y.y);
      if (wc > 0.0) {
        int mx = 0;
        int my = 0;
        grid.worldToMap(y.x, y.y, mx, my);
        const double c = cost_value(mx, my);
        if (c > thr && thr < 252.0) {
          const double gx = cost_value(mx + 1, my) - cost_value(mx - 1, my);
          const double gy = cost_value(mx, my + 1) - cost_value(mx, my - 1);
          const double gn = std::hypot(gx, gy);
          if (gn > 1e-9) {
            const double scale = std::min(1.0, (c - thr) / (252.0 - thr));
            nx += wc * res * scale * (-gx / gn);
            ny += wc * res * scale * (-gy / gn);
          }
        }
      }
      // 점별 사영: 새 위치와 이웃 선분이 천장을 넘으면 이 점은 이번 스윕에 움직이지 않는다
      const Point2D cand{nx, ny};
      if (cellValue(grid.costAtWorld(nx, ny)) > ceiling[i] ||
        segmentMaxCost(grid, pts[i - 1], cand) > ceiling[i] ||
        segmentMaxCost(grid, cand, pts[i + 1]) > ceiling[i])
      {
        continue;
      }
      change += std::hypot(nx - y.x, ny - y.y);
      pts[i] = cand;
    }
    // 롤백 가드: 모든 점·선분이 통행 가능해야 한다
    bool ok = true;
    for (std::size_t i = 1; i < n && ok; ++i) {
      ok = exempt[i - 1] != 0 || segmentFree(grid, pts[i - 1], pts[i]);
    }
    if (!ok) {
      pts = prev;
      rolled_back = true;
      return it;
    }
    prev = pts;
    if (change < config_.tolerance) {
      return it + 1;
    }
  }
  return it;
}

std::vector<Pose2D> PathSmoother::assignOrientations(
  const std::vector<Point2D> & pts, const double * goal_yaw)
{
  std::vector<Pose2D> out(pts.size());
  const std::size_t n = pts.size();
  for (std::size_t i = 0; i < n; ++i) {
    out[i].x = pts[i].x;
    out[i].y = pts[i].y;
  }
  if (n == 1) {
    out[0].theta = goal_yaw != nullptr ? *goal_yaw : 0.0;
    return out;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t a = i == 0 ? 0 : i - 1;
    const std::size_t b = i + 1 < n ? i + 1 : n - 1;
    out[i].theta = std::atan2(pts[b].y - pts[a].y, pts[b].x - pts[a].x);
  }
  if (goal_yaw != nullptr) {
    out[n - 1].theta = *goal_yaw;
  }
  return out;
}

std::vector<Pose2D> PathSmoother::process(
  const CostGrid & grid, const std::vector<Cell> & cells,
  const std::array<float, 256> & multiplier, const Point2D * start, const Point2D * goal,
  const double * goal_yaw, SmootherStats * stats) const
{
  std::vector<Point2D> pts = cellsToWorld(grid, cells);
  if (pts.empty()) {
    return {};
  }
  if (start != nullptr) {
    pts.front() = *start;
  }
  if (goal != nullptr) {
    if (pts.size() == 1 && start != nullptr) {
      pts.push_back(*goal);
    } else {
      pts.back() = *goal;
    }
  }
  SmootherStats st;
  st.raw_points = pts.size();
  st.raw_length = polylineLength(pts);

  std::vector<Point2D> work = pts;
  if (config_.enable_shortcut && pts.size() > 2) {
    const auto keep = shortcut(grid, pts, multiplier);
    work.clear();
    for (std::size_t k : keep) {
      work.push_back(pts[k]);
    }
  }
  st.shortcut_points = work.size();
  st.shortcut_length = polylineLength(work);

  const double spacing = config_.output_spacing > 0.0 ? config_.output_spacing : grid.resolution;
  work = resample(work, spacing);
  if (config_.enable_smoothing) {
    bool rb = false;
    st.iterations = smooth(grid, work, rb);
    st.rolled_back = rb;
  }
  st.smoothed_length = polylineLength(work);
  st.output_points = work.size();
  if (stats != nullptr) {
    *stats = st;
  }
  return assignOrientations(work, goal_yaw);
}

}  // namespace core
}  // namespace amr_navigation
