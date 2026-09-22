// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// LiDAR 스캔 클러스터링 구현.

#include "amr_perception/scan_clustering.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace amr_perception
{

namespace
{
constexpr double kBigCost = 1e20;

int findRoot(std::vector<int> & parent, int i)
{
  while (parent[i] != i) {
    parent[i] = parent[parent[i]];
    i = parent[i];
  }
  return i;
}

// PCA 장축/단축 방향 길이 (점 투영의 최대-최소)
void principalExtents(const std::vector<Vec2> & pts, double & length, double & width, Vec2 & axis)
{
  axis = Vec2(1.0, 0.0);
  length = 0.0;
  width = 0.0;
  if (pts.size() < 2) {
    return;
  }
  Vec2 mean = Vec2::Zero();
  for (const auto & p : pts) {
    mean += p;
  }
  mean /= static_cast<double>(pts.size());
  Mat2 cov = Mat2::Zero();
  for (const auto & p : pts) {
    const Vec2 d = p - mean;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Mat2> es(cov);
  axis = es.eigenvectors().col(1);  // 최대 고유값 방향
  const Vec2 minor(-axis.y(), axis.x());
  double amin = std::numeric_limits<double>::infinity();
  double amax = -amin;
  double bmin = amin;
  double bmax = -amin;
  for (const auto & p : pts) {
    const double a = (p - mean).dot(axis);
    const double b = (p - mean).dot(minor);
    amin = std::min(amin, a);
    amax = std::max(amax, a);
    bmin = std::min(bmin, b);
    bmax = std::max(bmax, b);
  }
  length = amax - amin;
  width = bmax - bmin;
}
}  // namespace

// ---------------------------------------------------------------- 거리 변환

void distanceTransform1d(const std::vector<double> & f, std::vector<double> & d)
{
  const int n = static_cast<int>(f.size());
  d.assign(n, 0.0);
  if (n == 0) {
    return;
  }
  std::vector<int> v(n, 0);        // 하한 포락선을 이루는 포물선의 꼭짓점
  std::vector<double> z(n + 1);    // 포물선 사이 경계
  int k = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  auto intersect = [&f](int q, int p) {
      return ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) /
             (2.0 * q - 2.0 * p);
    };
  for (int q = 1; q < n; ++q) {
    double s = intersect(q, v[k]);
    // z[0] = -inf 이므로 k = 0 에서 반드시 멈춘다
    while (s <= z[k]) {
      --k;
      s = intersect(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const double dq = static_cast<double>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

void StaticMapDistance::build(
  int width, int height, double resolution, const Pose2D & origin,
  const std::vector<int8_t> & data, int occupied_threshold)
{
  width_ = 0;
  height_ = 0;
  dist_.clear();
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    data.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    return;
  }
  resolution_ = resolution;
  origin_inv_ = origin.inverse();

  const std::size_t n = data.size();
  std::vector<double> grid(n);
  bool any_occupied = false;
  for (std::size_t i = 0; i < n; ++i) {
    const bool occ = data[i] >= occupied_threshold;
    grid[i] = occ ? 0.0 : kBigCost;
    any_occupied = any_occupied || occ;
  }
  dist_.assign(n, std::numeric_limits<float>::infinity());
  width_ = width;
  height_ = height;
  if (!any_occupied) {
    return;
  }
  // 열 방향 → 행 방향 (분리형)
  std::vector<double> f;
  std::vector<double> d;
  f.resize(height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      f[y] = grid[static_cast<std::size_t>(y) * width + x];
    }
    distanceTransform1d(f, d);
    for (int y = 0; y < height; ++y) {
      grid[static_cast<std::size_t>(y) * width + x] = d[y];
    }
  }
  f.resize(width);
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; ++x) {
      f[x] = grid[row + x];
    }
    distanceTransform1d(f, d);
    for (int x = 0; x < width; ++x) {
      const double sq = d[x];
      dist_[row + x] = sq >= 0.5 * kBigCost ? std::numeric_limits<float>::infinity() :
        static_cast<float>(std::sqrt(sq) * resolution_);
    }
  }
}

double StaticMapDistance::distance(const Vec2 & p_map) const
{
  if (!valid()) {
    return std::numeric_limits<double>::infinity();
  }
  const Vec2 q = origin_inv_.apply(p_map);
  const int ix = static_cast<int>(std::floor(q.x() / resolution_));
  const int iy = static_cast<int>(std::floor(q.y() / resolution_));
  if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(dist_[static_cast<std::size_t>(iy) * width_ + ix]);
}

// ---------------------------------------------------------------- 투영/라벨

std::vector<ScanPoint> projectScan(
  const LaserScanData & scan, const Pose2D & sensor_pose, double max_range)
{
  std::vector<ScanPoint> out;
  out.reserve(scan.ranges.size());
  const double upper = std::min(scan.range_max, max_range);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(r) || r < scan.range_min || r > upper) {
      continue;
    }
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    ScanPoint sp;
    sp.range = r;
    sp.bearing = a;
    sp.beam = static_cast<int>(i);
    sp.position = sensor_pose.apply(Vec2(r * std::cos(a), r * std::sin(a)));
    out.push_back(sp);
  }
  return out;
}

void labelBackground(
  std::vector<ScanPoint> & points, const StaticMapDistance & map,
  const Pose2D & map_from_tracking, double background_radius)
{
  if (!map.valid()) {
    return;
  }
  for (auto & p : points) {
    p.map_distance = map.distance(map_from_tracking.apply(p.position));
    p.background = p.map_distance <= background_radius;
  }
}

// ---------------------------------------------------------------- 분할

double abdThreshold(double range, double dphi, double lambda, double sigma_r)
{
  if (dphi >= lambda) {
    return -1.0;
  }
  return range * std::sin(dphi) / std::sin(lambda - dphi) + 3.0 * sigma_r;
}

std::vector<std::vector<int>> segmentScan(
  const std::vector<ScanPoint> & points, double angle_increment,
  const SegmentationParams & params, bool full_circle, int total_beams)
{
  std::vector<std::vector<int>> segments;
  if (points.empty()) {
    return segments;
  }
  const double dphi0 = std::abs(angle_increment);
  auto breaks = [&](const ScanPoint & a, const ScanPoint & b, int beam_gap) {
      if (a.background != b.background) {
        return true;
      }
      const double th = abdThreshold(
        a.range, dphi0 * beam_gap, params.lambda, params.sigma_r);
      if (th < 0.0) {
        return true;
      }
      return (b.position - a.position).norm() > th;
    };

  segments.push_back({0});
  for (std::size_t i = 1; i < points.size(); ++i) {
    const int gap = points[i].beam - points[i - 1].beam;
    if (breaks(points[i - 1], points[i], gap)) {
      segments.push_back({});
    }
    segments.back().push_back(static_cast<int>(i));
  }
  // 360° 스캔: 마지막 빔과 첫 빔이 이웃이면 끝 세그먼트를 첫 세그먼트 앞에 붙인다
  if (full_circle && segments.size() > 1) {
    const ScanPoint & last = points.back();
    const ScanPoint & first = points.front();
    const int gap = first.beam + (total_beams - last.beam);
    if (!breaks(last, first, gap)) {
      std::vector<int> merged = segments.back();
      merged.insert(merged.end(), segments.front().begin(), segments.front().end());
      segments.front() = merged;
      segments.pop_back();
    }
  }
  return segments;
}

std::vector<std::vector<int>> mergeSegments(
  const std::vector<ScanPoint> & points, const std::vector<std::vector<int>> & segments,
  double merge_min_gap)
{
  const int k = static_cast<int>(segments.size());
  std::vector<int> parent(k);
  std::iota(parent.begin(), parent.end(), 0);
  const double gap2 = merge_min_gap * merge_min_gap;
  for (int a = 0; a < k; ++a) {
    for (int b = a + 1; b < k; ++b) {
      if (segments[a].empty() || segments[b].empty()) {
        continue;
      }
      if (points[segments[a].front()].background != points[segments[b].front()].background) {
        continue;
      }
      bool close = false;
      for (int ia : segments[a]) {
        for (int ib : segments[b]) {
          if ((points[ia].position - points[ib].position).squaredNorm() < gap2) {
            close = true;
            break;
          }
        }
        if (close) {
          break;
        }
      }
      if (close) {
        parent[findRoot(parent, b)] = findRoot(parent, a);
      }
    }
  }
  std::vector<std::vector<int>> out;
  std::vector<int> slot(k, -1);
  for (int a = 0; a < k; ++a) {
    const int r = findRoot(parent, a);
    if (slot[r] < 0) {
      slot[r] = static_cast<int>(out.size());
      out.emplace_back();
    }
    auto & dst = out[slot[r]];
    dst.insert(dst.end(), segments[a].begin(), segments[a].end());
  }
  return out;
}

std::vector<std::vector<int>> splitOversized(
  const std::vector<ScanPoint> & points, const std::vector<std::vector<int>> & segments,
  double max_extent)
{
  std::vector<std::vector<int>> out;
  std::vector<std::vector<int>> stack(segments.rbegin(), segments.rend());
  while (!stack.empty()) {
    std::vector<int> seg = std::move(stack.back());
    stack.pop_back();
    std::vector<Vec2> pts;
    pts.reserve(seg.size());
    for (int i : seg) {
      pts.push_back(points[i].position);
    }
    double length = 0.0;
    double width = 0.0;
    Vec2 axis;
    principalExtents(pts, length, width, axis);
    if (length <= max_extent || seg.size() < 4) {
      out.push_back(std::move(seg));
      continue;
    }
    // 가장 큰 연속 점 간격에서 둘로 나눈다
    std::size_t cut = 1;
    double best = -1.0;
    for (std::size_t i = 1; i < seg.size(); ++i) {
      const double g = (points[seg[i]].position - points[seg[i - 1]].position).norm();
      if (g > best) {
        best = g;
        cut = i;
      }
    }
    std::vector<int> left(seg.begin(), seg.begin() + static_cast<std::ptrdiff_t>(cut));
    std::vector<int> right(seg.begin() + static_cast<std::ptrdiff_t>(cut), seg.end());
    stack.push_back(std::move(right));
    stack.push_back(std::move(left));
  }
  return out;
}

// ---------------------------------------------------------------- 측정 모델

Cluster buildCluster(
  const std::vector<ScanPoint> & points, const std::vector<int> & indices,
  const Vec2 & sensor_origin, double angle_increment, const ClusterModelParams & params)
{
  Cluster c;
  const int n = static_cast<int>(indices.size());
  c.num_points = n;
  if (n == 0) {
    return c;
  }
  c.points.reserve(n);
  double range_sum = 0.0;
  double bmin = std::numeric_limits<double>::infinity();
  double bmax = -bmin;
  // 센서 좌표계 각도: 빔 각을 기준으로 연속(unwrap)되게 누적
  const double ref = points[indices.front()].bearing;
  for (int i : indices) {
    const ScanPoint & sp = points[i];
    c.points.push_back(sp.position);
    c.centroid += sp.position;
    range_sum += sp.range;
    const double b = ref + normalizeAngle(sp.bearing - ref);
    bmin = std::min(bmin, b);
    bmax = std::max(bmax, b);
  }
  c.centroid /= static_cast<double>(n);
  c.mean_range = range_sum / static_cast<double>(n);
  c.bearing_min = bmin;
  c.bearing_max = bmax;

  // 시선(LOS) 단위벡터: 센서 → 점 중심. 편향 보정은 센서에서 멀어지는 방향으로 μ_δ.
  Vec2 los = c.centroid - sensor_origin;
  const double los_norm = los.norm();
  los = los_norm > 1e-9 ? Vec2(los / los_norm) : Vec2(1.0, 0.0);
  c.measurement = c.centroid + params.bias_mu * los;

  // 공분산 (시선 좌표계 → 추적 프레임)
  const double s = c.mean_range * std::abs(angle_increment);  // 빔 간 가로 간격
  const double var_par = params.sigma_r * params.sigma_r / n +
    params.sigma_delta * params.sigma_delta;
  const double var_perp = s * s / 24.0 + params.sigma_lat * params.sigma_lat;
  const double floor2 = params.sigma_floor * params.sigma_floor;
  Mat2 rot;
  rot << los.x(), -los.y(), los.y(), los.x();
  Mat2 d = Mat2::Zero();
  d(0, 0) = std::max(var_par, floor2);
  d(1, 1) = std::max(var_perp, floor2);
  c.R = rot * d * rot.transpose();
  Mat2 dj = Mat2::Zero();
  dj(0, 0) = params.sigma_r * params.sigma_r / n + params.sigma_seg * params.sigma_seg;
  dj(1, 1) = s * s / 24.0 + params.sigma_seg * params.sigma_seg;
  c.R_jitter = rot * dj * rot.transpose();

  Vec2 axis;
  principalExtents(c.points, c.length, c.width, axis);
  double rad = 0.0;
  for (const auto & p : c.points) {
    rad = std::max(rad, (p - c.measurement).norm());
  }
  c.radius = std::max(rad, params.min_radius);
  return c;
}

bool isOccludedAtEdge(
  const LaserScanData & scan, const std::vector<ScanPoint> & points,
  const std::vector<int> & indices, double mean_range, bool full_circle, double margin)
{
  const int total = static_cast<int>(scan.ranges.size());
  if (indices.empty() || total == 0) {
    return false;
  }
  // 첫 빔 기준 부호 있는 오프셋으로 각도 극값 빔을 찾는다 (360° 이음매를 넘는 세그먼트 대응)
  const int b0 = points[indices.front()].beam;
  int lo = 0;
  int hi = 0;
  for (int i : indices) {
    int d = points[i].beam - b0;
    if (full_circle) {
      d = ((d + total / 2) % total + total) % total - total / 2;
    }
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  for (const int nb : {b0 + lo - 1, b0 + hi + 1}) {
    int b = nb;
    if (full_circle) {
      b = (b % total + total) % total;
    } else if (b < 0 || b >= total) {
      continue;
    }
    const double r = static_cast<double>(scan.ranges[b]);
    if (std::isfinite(r) && r >= scan.range_min && r < mean_range - margin) {
      return true;
    }
  }
  return false;
}

std::vector<Cluster> extractClusters(
  const LaserScanData & scan, const Pose2D & sensor_pose, const StaticMapDistance * map,
  const Pose2D & map_from_tracking, const SegmentationParams & seg,
  const ClusterModelParams & model)
{
  std::vector<ScanPoint> pts = projectScan(scan, sensor_pose, seg.max_range);
  if (map != nullptr && map->valid()) {
    labelBackground(pts, *map, map_from_tracking, seg.background_radius);
  }
  const int total_beams = static_cast<int>(scan.ranges.size());
  const double coverage = std::abs(scan.angle_increment) * total_beams;
  const bool full_circle = coverage >= 2.0 * M_PI - 2.0 * std::abs(scan.angle_increment);
  auto segments = segmentScan(pts, scan.angle_increment, seg, full_circle, total_beams);
  // 배경 세그먼트는 추적 대상이 아니다 (정적 지도). 전경만 병합/재분할.
  std::vector<std::vector<int>> foreground;
  for (auto & s : segments) {
    if (!s.empty() && !pts[s.front()].background) {
      foreground.push_back(std::move(s));
    }
  }
  foreground = mergeSegments(pts, foreground, seg.merge_min_gap);
  foreground = splitOversized(pts, foreground, seg.max_extent);

  std::vector<Cluster> clusters;
  for (const auto & s : foreground) {
    double mean_r = 0.0;
    for (int i : s) {
      mean_r += pts[i].range;
    }
    mean_r /= std::max<std::size_t>(s.size(), 1);
    const int min_pts = mean_r <= seg.far_range ? seg.min_points_near : seg.min_points_far;
    if (static_cast<int>(s.size()) < min_pts) {
      continue;
    }
    Cluster c = buildCluster(pts, s, sensor_pose.translation(), scan.angle_increment, model);
    if (isOccludedAtEdge(scan, pts, s, c.mean_range, full_circle, model.occlusion_margin)) {
      // 보이는 조각의 중심은 가려진 쪽 반대로 치우친다 → 가로(시선 수직) 분산 팽창
      const Vec2 los = (c.centroid - sensor_pose.translation()).normalized();
      const Vec2 perp(-los.y(), los.x());
      c.R += model.sigma_occluded * model.sigma_occluded * perp * perp.transpose();
      c.occluded = true;
    }
    int overlap = 0;
    for (int i : s) {
      if (pts[i].map_distance >= 0.0 && pts[i].map_distance <= seg.overlap_radius) {
        ++overlap;
      }
    }
    c.map_overlap = static_cast<double>(overlap) / static_cast<double>(s.size());
    clusters.push_back(std::move(c));
  }
  return clusters;
}

}  // namespace amr_perception
