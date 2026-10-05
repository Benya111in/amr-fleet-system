#include "amr_navigation/core/scan_denoise.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_navigation
{
namespace core
{

std::vector<float> denoiseRanges(
  const std::vector<float> & ranges, const ScanDenoiseConfig & config, bool wrap)
{
  const int n = static_cast<int>(ranges.size());
  std::vector<float> out(ranges);
  const int k = std::max(0, config.half_window);
  if (n == 0 || k == 0) {
    return out;
  }
  std::vector<float> support;
  support.reserve(static_cast<std::size_t>(2 * k + 1));
  for (int i = 0; i < n; ++i) {
    const float ri = ranges[static_cast<std::size_t>(i)];
    if (!std::isfinite(ri)) {
      continue;
    }
    support.clear();
    for (int d = -k; d <= k; ++d) {
      int j = i + d;
      if (j < 0 || j >= n) {
        if (!wrap) {
          continue;
        }
        j = (j + n) % n;
      }
      const float rj = ranges[static_cast<std::size_t>(j)];
      if (std::isfinite(rj) && std::abs(rj - ri) <= config.range_gate) {
        support.push_back(rj);
      }
    }
    if (static_cast<int>(support.size()) < std::max(1, config.min_support)) {
      continue;
    }
    const std::size_t mid = support.size() / 2;
    std::nth_element(
      support.begin(),
      support.begin() + static_cast<std::ptrdiff_t>(mid), support.end());
    float med = support[mid];
    if (support.size() % 2 == 0) {
      // 짝수 개: 가운데 두 값의 평균 (아래쪽 값 = mid 앞 구간의 최댓값)
      const float lo =
        *std::max_element(support.begin(), support.begin() + static_cast<std::ptrdiff_t>(mid));
      med = 0.5f * (med + lo);
    }
    out[static_cast<std::size_t>(i)] = med;
  }
  return out;
}

std::size_t excludeDiscsFromScan(
  std::vector<float> & ranges, double angle_min, double angle_increment,
  const std::vector<Point2D> & centers, double radius)
{
  std::size_t removed = 0;
  if (centers.empty() || radius <= 0.0) {
    return removed;
  }
  const double r2 = radius * radius;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r)) {
      continue;
    }
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    const double x = r * std::cos(a);
    const double y = r * std::sin(a);
    for (const auto & c : centers) {
      if ((x - c.x) * (x - c.x) + (y - c.y) * (y - c.y) <= r2) {
        ranges[i] = std::numeric_limits<float>::infinity();
        ++removed;
        break;
      }
    }
  }
  return removed;
}

}  // namespace core
}  // namespace amr_navigation
