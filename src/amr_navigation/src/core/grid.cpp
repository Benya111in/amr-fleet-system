#include "amr_navigation/core/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{

bool CostGrid::worldToMap(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  my = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return inBounds(mx, my);
}

void CostGrid::mapToWorld(int mx, int my, double & wx, double & wy) const
{
  wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
  wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
}

uint8_t CostGrid::costAtWorld(double wx, double wy) const
{
  int mx = 0;
  int my = 0;
  if (!worldToMap(wx, wy, mx, my)) {
    return kLethalObstacle;
  }
  return at(mx, my);
}

OwnedGrid::OwnedGrid(int w, int h, double res, uint8_t fill, double ox, double oy)
: cells(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), fill),
  width(w), height(h), resolution(res), origin_x(ox), origin_y(oy)
{
}

CostGrid OwnedGrid::view() const
{
  CostGrid g;
  g.data = cells.data();
  g.width = width;
  g.height = height;
  g.resolution = resolution;
  g.origin_x = origin_x;
  g.origin_y = origin_y;
  return g;
}

void OwnedGrid::fillRect(double x0, double y0, double x1, double y1, uint8_t value)
{
  // 셀 중심이 경계 위에 있을 때 부동소수 오차로 빠지지 않도록 1e-9 m 여유
  constexpr double kEps = 1e-9;
  const double lo_x = std::min(x0, x1) - kEps;
  const double hi_x = std::max(x0, x1) + kEps;
  const double lo_y = std::min(y0, y1) - kEps;
  const double hi_y = std::max(y0, y1) + kEps;
  for (int my = 0; my < height; ++my) {
    const double cy = origin_y + (my + 0.5) * resolution;
    if (cy < lo_y || cy > hi_y) {
      continue;
    }
    for (int mx = 0; mx < width; ++mx) {
      const double cx = origin_x + (mx + 0.5) * resolution;
      if (cx >= lo_x && cx <= hi_x) {
        at(mx, my) = value;
      }
    }
  }
}

uint8_t inflationCost(double distance_m, double inscribed_radius, double cost_scaling_factor)
{
  if (distance_m <= 0.0) {
    return kLethalObstacle;
  }
  if (distance_m <= inscribed_radius) {
    return kInscribedInflated;
  }
  const double factor = std::exp(-cost_scaling_factor * (distance_m - inscribed_radius));
  // Nav2 와 같이 절삭(floor)
  return static_cast<uint8_t>(static_cast<double>(kInscribedInflated - 1) * factor);
}

void inflate(
  OwnedGrid & grid, double inscribed_radius, double inflation_radius, double cost_scaling_factor)
{
  const int w = grid.width;
  const int h = grid.height;
  const double res = grid.resolution;
  const int r_cells = static_cast<int>(std::ceil(inflation_radius / res));
  // 커널: (dx, dy, cost) — 반경 안의 오프셋과 그 거리의 비용을 미리 계산
  struct K
  {
    int dx;
    int dy;
    uint8_t cost;
  };
  std::vector<K> kernel;
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const double d = std::hypot(dx, dy) * res;
      if (d > inflation_radius || (dx == 0 && dy == 0)) {
        continue;
      }
      const uint8_t c = inflationCost(d, inscribed_radius, cost_scaling_factor);
      if (c > 0) {
        kernel.push_back({dx, dy, c});
      }
    }
  }
  const std::vector<uint8_t> src = grid.cells;
  auto lethal = [&](int x, int y) {
      return src[static_cast<std::size_t>(y) * w + x] == kLethalObstacle;
    };
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!lethal(x, y)) {
        continue;
      }
      // 내부 치명 셀(4-이웃이 모두 치명)은 경계 셀의 커널이 이미 덮는다 → 건너뜀
      const bool interior =
        x > 0 && y > 0 && x < w - 1 && y < h - 1 &&
        lethal(x - 1, y) && lethal(x + 1, y) && lethal(x, y - 1) && lethal(x, y + 1);
      if (interior) {
        continue;
      }
      for (const auto & k : kernel) {
        const int nx = x + k.dx;
        const int ny = y + k.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        uint8_t & cell = grid.at(nx, ny);
        if (cell != kNoInformation && cell < k.cost) {
          cell = k.cost;
        }
      }
    }
  }
}

bool traceLine(int x0, int y0, int x1, int y1, const std::function<bool(int, int)> & visit)
{
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    if (!visit(x, y)) {
      return false;
    }
    if (x == x1 && y == y1) {
      return true;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

bool traceSupercover(
  const CostGrid & grid, double x0, double y0, double x1, double y1,
  const std::function<bool(int, int)> & visit)
{
  // Amanatides–Woo 격자 순회. 선분이 셀 모서리를 정확히 지나면 양옆 셀도 방문한다(보수적).
  const double res = grid.resolution;
  const double gx0 = (x0 - grid.origin_x) / res;
  const double gy0 = (y0 - grid.origin_y) / res;
  const double gx1 = (x1 - grid.origin_x) / res;
  const double gy1 = (y1 - grid.origin_y) / res;
  int cx = static_cast<int>(std::floor(gx0));
  int cy = static_cast<int>(std::floor(gy0));
  const int ex = static_cast<int>(std::floor(gx1));
  const int ey = static_cast<int>(std::floor(gy1));
  const double dx = gx1 - gx0;
  const double dy = gy1 - gy0;
  const int step_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
  const int step_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const double t_delta_x = step_x != 0 ? std::abs(1.0 / dx) : kInf;
  const double t_delta_y = step_y != 0 ? std::abs(1.0 / dy) : kInf;
  double t_max_x = kInf;
  double t_max_y = kInf;
  if (step_x > 0) {
    t_max_x = (std::floor(gx0) + 1.0 - gx0) * t_delta_x;
  } else if (step_x < 0) {
    t_max_x = (gx0 - std::floor(gx0)) * t_delta_x;
  }
  if (step_y > 0) {
    t_max_y = (std::floor(gy0) + 1.0 - gy0) * t_delta_y;
  } else if (step_y < 0) {
    t_max_y = (gy0 - std::floor(gy0)) * t_delta_y;
  }
  if (!visit(cx, cy)) {
    return false;
  }
  // 안전장치: 최대 방문 수 = 맨해튼 거리 + 2
  const int max_steps = std::abs(ex - cx) + std::abs(ey - cy) + 2;
  constexpr double kEps = 1e-12;
  for (int i = 0; i < max_steps && !(cx == ex && cy == ey); ++i) {
    if (std::min(t_max_x, t_max_y) > 1.0 + 1e-9) {
      break;  // 수치 오차로 끝점을 지나침 → 아래에서 끝 셀만 방문
    }
    if (std::abs(t_max_x - t_max_y) < kEps) {
      // 모서리 통과: 두 옆 셀을 모두 방문
      if (!visit(cx + step_x, cy) || !visit(cx, cy + step_y)) {
        return false;
      }
      cx += step_x;
      cy += step_y;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (t_max_x < t_max_y) {
      cx += step_x;
      t_max_x += t_delta_x;
    } else {
      cy += step_y;
      t_max_y += t_delta_y;
    }
    if (!visit(cx, cy)) {
      return false;
    }
  }
  if (!(cx == ex && cy == ey)) {
    return visit(ex, ey);
  }
  return true;
}

}  // namespace core
}  // namespace amr_navigation
