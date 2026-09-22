// 2D LiDAR 스캔 필터 구현. 단계 설명은 헤더 주석.

#include "amr_localization/scan_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace amr_localization
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

/// from → to 의 반시계 각거리 [0, 2π).
double ccwDistance(double from, double to)
{
  double d = std::fmod(to - from, kTwoPi);
  if (d < 0.0) {
    d += kTwoPi;
  }
  return d;
}
}  // namespace

ScanFilter::ScanFilter(const ScanFilterParams & params)
: params_(params)
{
}

double ScanFilter::effectiveRangeMin(double sensor_range_min) const
{
  return std::max(sensor_range_min, params_.range_min);
}

double ScanFilter::effectiveRangeMax(double sensor_range_max) const
{
  return std::min(sensor_range_max, params_.range_max);
}

bool ScanFilter::inArc(double angle, double start, double end)
{
  if (end - start >= kTwoPi - 1e-9) {
    return true;
  }
  return ccwDistance(start, angle) <= ccwDistance(start, end) + 1e-9;
}

bool ScanFilter::isAngleRemoved(double angle) const
{
  if (!inArc(angle, params_.angle_window_min, params_.angle_window_max)) {
    return true;
  }
  for (const auto & mask : params_.angle_mask) {
    if (inArc(angle, mask.first, mask.second)) {
      return true;
    }
  }
  return false;
}

ScanFilterStats ScanFilter::apply(
  const std::vector<float> & in, double angle_min, double angle_increment,
  double sensor_range_min, double sensor_range_max, std::vector<float> & out) const
{
  ScanFilterStats stats;
  const std::size_t n = in.size();
  stats.input = n;
  out.assign(in.begin(), in.end());
  if (n == 0) {
    return stats;
  }

  const double r_min = effectiveRangeMin(sensor_range_min);
  const double r_max = effectiveRangeMax(sensor_range_max);
  const double inc = std::abs(angle_increment);
  // 360° 스캔: n 번째 빔이 0 번째와 겹친다 → 이웃 탐색을 감는다
  const bool full_circle = inc > 0.0 &&
    std::abs(static_cast<double>(n) * inc - kTwoPi) < 0.5 * inc;

  // 1. 거리 필터 (REP-117)
  for (std::size_t i = 0; i < n; ++i) {
    const float r = in[i];
    if (std::isnan(r)) {
      ++stats.nan_input;
      out[i] = kNaN;
    } else if (r < r_min) {
      ++stats.too_close;
      out[i] = -kInf;
    } else if (r > r_max) {
      ++stats.too_far;
      out[i] = kInf;
    }
  }

  // 2. 각도 필터
  for (std::size_t i = 0; i < n; ++i) {
    const double angle = angle_min + static_cast<double>(i) * angle_increment;
    if (isAngleRemoved(angle)) {
      if (!std::isnan(out[i])) {
        ++stats.angle_removed;
      }
      out[i] = kNaN;
    }
  }

  auto neighbor = [&](std::size_t i, std::ptrdiff_t offset, std::size_t & j) -> bool {
      const std::ptrdiff_t idx = static_cast<std::ptrdiff_t>(i) + offset;
      const std::ptrdiff_t size = static_cast<std::ptrdiff_t>(n);
      if (idx >= 0 && idx < size) {
        j = static_cast<std::size_t>(idx);
        return true;
      }
      if (!full_circle) {
        return false;
      }
      j = static_cast<std::size_t>((idx % size + size) % size);
      return j != i;
    };

  // 3. 아웃라이어(고립점) 제거 — 이웃 일관성
  if (params_.outlier_window > 0 && params_.outlier_min_neighbors > 0) {
    const std::vector<float> snapshot = out;
    std::vector<bool> outlier(n, false);
    for (std::size_t i = 0; i < n; ++i) {
      const double ri = snapshot[i];
      if (!std::isfinite(ri)) {
        continue;
      }
      int support = 0;
      for (int k = 1; k <= params_.outlier_window && support < params_.outlier_min_neighbors;
        ++k)
      {
        const double spacing = static_cast<double>(k) * inc;
        const double thr = params_.outlier_thresh + params_.outlier_range_gain * ri * spacing;
        const double cos_k = std::cos(spacing);
        for (const std::ptrdiff_t sign : {-1, 1}) {
          std::size_t j = 0;
          if (!neighbor(i, sign * static_cast<std::ptrdiff_t>(k), j)) {
            continue;
          }
          const double rj = snapshot[j];
          if (!std::isfinite(rj)) {
            continue;
          }
          const double d2 = ri * ri + rj * rj - 2.0 * ri * rj * cos_k;
          if (d2 <= thr * thr) {
            ++support;
          }
        }
      }
      if (support < params_.outlier_min_neighbors) {
        outlier[i] = true;
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (outlier[i]) {
        out[i] = kNaN;
        ++stats.outliers;
      }
    }
  }

  // 4. 섀도우(베일) 제거 — 인접 빔 쌍의 시선각
  if (params_.shadow_filter_enabled && inc > 0.0) {
    const std::vector<float> snapshot = out;
    std::vector<bool> shadow(n, false);
    const double lo = params_.shadow_min_angle;
    const double hi = M_PI - params_.shadow_min_angle;
    const double sin_inc = std::sin(inc);
    const double cos_inc = std::cos(inc);
    // 같은 면의 두 빔 거리 차는 잡음만으로 N(0, 2σ_r²) → k·√2·σ_r 이하면 베일로 보지 않는다
    const double sigma_r = std::max(params_.shadow_range_noise_stddev, 0.0);
    const double noise_jump = params_.shadow_noise_factor * std::sqrt(2.0) * sigma_r;
    const std::size_t pairs = full_circle ? n : n - 1;
    for (std::size_t i = 0; i < pairs; ++i) {
      const std::size_t j = (i + 1) % n;
      const double ri = snapshot[i];
      const double rj = snapshot[j];
      if (!std::isfinite(ri) || !std::isfinite(rj) || std::abs(ri - rj) <= noise_jump) {
        continue;
      }
      const double beta = std::atan2(rj * sin_inc, ri - rj * cos_inc);
      if (beta < lo || beta > hi) {
        shadow[ri > rj ? i : j] = true;
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (shadow[i]) {
        out[i] = kNaN;
        ++stats.shadows;
      }
    }
  }

  stats.valid_output = static_cast<std::size_t>(
    std::count_if(out.begin(), out.end(), [](float r) {return std::isfinite(r);}));
  return stats;
}

}  // namespace amr_localization
