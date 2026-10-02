// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// LiDAR 스캔 클러스터링 구현.

#include "amr_perception/scan_clustering.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

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
  origin_ = origin;
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

double StaticMapDistance::interpolate(const Vec2 & p_map, Vec2 * gradient) const
{
  const double inf = std::numeric_limits<double>::infinity();
  if (!valid()) {
    return inf;
  }
  // 셀 중심 격자 좌표 (셀 (i, j) 중심 = ((i + 0.5)·res, (j + 0.5)·res))
  const Vec2 q = origin_inv_.apply(p_map);
  const double u = q.x() / resolution_ - 0.5;
  const double v = q.y() / resolution_ - 0.5;
  const int i0 = static_cast<int>(std::floor(u));
  const int j0 = static_cast<int>(std::floor(v));
  if (i0 < 0 || j0 < 0 || i0 + 1 >= width_ || j0 + 1 >= height_) {
    return inf;
  }
  const double fu = u - i0;
  const double fv = v - j0;
  auto at = [this](int i, int j) {
      return static_cast<double>(dist_[static_cast<std::size_t>(j) * width_ + i]);
    };
  const double d00 = at(i0, j0);
  const double d10 = at(i0 + 1, j0);
  const double d01 = at(i0, j0 + 1);
  const double d11 = at(i0 + 1, j0 + 1);
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    return inf;
  }
  if (gradient != nullptr) {
    const Vec2 g_grid(
      ((d10 - d00) * (1.0 - fv) + (d11 - d01) * fv) / resolution_,
      ((d01 - d00) * (1.0 - fu) + (d11 - d10) * fu) / resolution_);
    *gradient = origin_.rotate(g_grid);
  }
  return (d00 * (1.0 - fu) + d10 * fu) * (1.0 - fv) + (d01 * (1.0 - fu) + d11 * fu) * fv;
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

namespace
{
/// 센서 중심 회전 보정 (tx, ty, θ): p' = R(θ)(p - c) + c + t
Pose2D pivotCorrection(const Eigen::Vector3d & delta, const Vec2 & c)
{
  const Pose2D rot{0.0, 0.0, delta(2)};
  const Vec2 t = c + Vec2(delta(0), delta(1)) - rot.rotate(c);
  return Pose2D{t.x(), t.y(), delta(2)};
}

double medianOf(std::vector<double> v)
{
  if (v.empty()) {
    return 0.0;
  }
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  return v[mid];
}
}  // namespace

MapAlignment alignScanToMap(
  const std::vector<ScanPoint> & points, const Vec2 & sensor, const StaticMapDistance & map,
  const Pose2D & map_from_tracking, const PosePrior & prior, const SegmentationParams & params)
{
  MapAlignment out;
  out.map_from_tracking = map_from_tracking;
  out.pivot = map_from_tracking.apply(sensor);
  const double sxy = std::max(prior.sigma_xy, 1e-3);
  const double syaw = std::max(prior.sigma_yaw, 1e-4);
  Eigen::Matrix3d prior_info = Eigen::Matrix3d::Zero();
  prior_info(0, 0) = prior_info(1, 1) = 1.0 / (sxy * sxy);
  prior_info(2, 2) = 1.0 / (syaw * syaw);
  out.covariance = prior_info.inverse();
  if (!map.valid()) {
    return out;
  }
  // 대응 후보: 초기 거리장 값이 작은 점 (지도 구조물에 맞은 빔)
  std::vector<Vec2> pts;
  pts.reserve(points.size());
  for (const auto & sp : points) {
    const Vec2 pm = map_from_tracking.apply(sp.position);
    if (map.distance(pm) <= params.align_max_correspondence) {
      pts.push_back(pm);
    }
  }
  out.correspondences = static_cast<int>(pts.size());
  if (out.correspondences < params.align_min_points) {
    return out;
  }
  const Vec2 c = out.pivot;
  const double surface = 0.5 * map.resolution();  // 점유 셀 중심 ↔ 표면
  // 측정 분산 = LiDAR 거리 σ² + 격자 양자화 res²/12 (사전분포와 같은 단위로 맞춘다)
  const double meas_var = params.sigma_r * params.sigma_r +
    map.resolution() * map.resolution() / 12.0;
  // 강건 비용 (대응점 고정: 멀어진 점은 상한 잔차로 센다 → 점이 빠져 비용이 줄어드는 일이 없다)
  const double cap = params.align_max_correspondence;
  const double k = params.align_huber;
  auto huber = [k](double r) {
      const double a = std::abs(r);
      return a <= k ? 0.5 * a * a : k * (a - 0.5 * k);
    };
  auto evaluate = [&](const Eigen::Vector3d & dl, Eigen::Matrix3d * Hd, Eigen::Vector3d * gd,
      std::vector<double> * res) {
      double cost = dl.dot(prior_info * dl);
      if (Hd != nullptr) {
        Hd->setZero();
        gd->setZero();
        res->clear();
      }
      const Pose2D T = pivotCorrection(dl, c);
      const Pose2D R{0.0, 0.0, dl(2)};
      for (const auto & p0 : pts) {
        Vec2 grad;
        const double d = map.interpolate(T.apply(p0), &grad);
        const double r = std::isfinite(d) ? d - surface : cap;
        if (!std::isfinite(d) || std::abs(r) > cap) {
          cost += huber(cap) / meas_var;
          continue;
        }
        cost += huber(r) / meas_var;
        if (Hd != nullptr) {
          res->push_back(r);
          const double w = std::abs(r) <= k ? 1.0 : k / std::abs(r);
          const Vec2 v = R.rotate(p0 - c);
          const Eigen::Vector3d J(grad.x(), grad.y(), -grad.x() * v.y() + grad.y() * v.x());
          *Hd += w * J * J.transpose();
          *gd += w * J * r;
        }
      }
      return cost;
    };
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  Eigen::Matrix3d H_data;
  Eigen::Vector3d g_data;
  std::vector<double> residuals;
  double cost = evaluate(delta, &H_data, &g_data, &residuals);
  bool converged = false;
  for (int it = 0; it < params.align_iterations; ++it) {
    const Eigen::Matrix3d H = prior_info + H_data / meas_var;
    const Eigen::Vector3d g = prior_info * delta + g_data / meas_var;
    Eigen::Vector3d step = -H.ldlt().solve(g);
    // 한 번에 10 cm·1.1° 까지만 (거리장 기울기는 셀 단위로 끊기므로 큰 걸음은 넘어간다)
    const double scale = std::min(
      {1.0, 0.10 / std::max(step.head<2>().norm(), 1e-12),
        0.02 / std::max(std::abs(step(2)), 1e-12)});
    step *= scale;
    bool accepted = false;
    for (int half = 0; half < 6 && !accepted; ++half, step *= 0.5) {
      Eigen::Matrix3d Hc;
      Eigen::Vector3d gc;
      std::vector<double> rc;
      const double c2 = evaluate(delta + step, &Hc, &gc, &rc);
      if (c2 <= cost) {
        delta += step;
        cost = c2;
        H_data = Hc;
        g_data = gc;
        residuals.swap(rc);
        accepted = true;
      }
    }
    if (!accepted || (step.head<2>().norm() < 2e-4 && std::abs(step(2)) < 2e-5)) {
      converged = true;  // 더 내려갈 곳이 없다 (지역 최소)
      break;
    }
  }
  out.attempted = pivotCorrection(delta, c);
  if (static_cast<int>(residuals.size()) < params.align_min_points ||
    delta.head<2>().norm() > params.align_max_translation ||
    std::abs(delta(2)) > params.align_max_rotation)
  {
    return out;  // 실패: 보정 없이 사전분포 공분산
  }
  // 잔차 강건 σ 와 공분산 Σ = σ²(H_data + σ²Λ_prior)⁻¹ 근사: 정보 = H_data/σ² + Λ_prior
  const double med = medianOf(residuals);
  std::vector<double> dev;
  dev.reserve(residuals.size());
  for (const double r : residuals) {
    dev.push_back(std::abs(r - med));
  }
  const double sigma = std::max(1.4826 * medianOf(dev), 0.01);
  if (sigma > params.align_max_residual) {
    return out;  // 잔차가 큰 정합 = 구조물이 맞지 않는 지역 최소 → 보정 없이 사전분포
  }
  const Eigen::Matrix3d info = H_data / (sigma * sigma) + prior_info;
  out.covariance = info.inverse();
  out.residual_sigma = sigma;
  out.correction = pivotCorrection(delta, c);
  out.map_from_tracking = out.correction.compose(map_from_tracking);
  out.valid = true;
  out.converged = converged;
  return out;
}

void labelBackground(
  std::vector<ScanPoint> & points, const StaticMapDistance & map, const MapAlignment & alignment,
  const SegmentationParams & params)
{
  if (!map.valid()) {
    return;
  }
  const double k = params.background_k_sigma;
  const double r0 = std::max(params.background_radius, k * alignment.residual_sigma);
  const Eigen::Matrix3d & S = alignment.covariance;
  for (auto & sp : points) {
    const Vec2 pm = alignment.map_from_tracking.apply(sp.position);
    sp.map_distance = map.distance(pm);
    if (!std::isfinite(sp.map_distance) || sp.map_distance > params.background_max_radius) {
      sp.background = false;
      continue;
    }
    // 점 변위 δp = [I, J(p - c)] δ 의 거리장 법선 성분 분산
    Vec2 n;
    double sigma_n2 = S(0, 0) + S(1, 1);  // 법선을 모르면 두 축 합 (보수적)
    if (std::isfinite(map.interpolate(pm, &n)) && n.norm() > 1e-6) {
      n.normalize();
      const Vec2 v = pm - alignment.pivot;
      const Eigen::Vector3d a(n.x(), n.y(), -n.x() * v.y() + n.y() * v.x());
      sigma_n2 = a.dot(S * a);
    }
    const double r_bg = std::min(
      params.background_max_radius, std::sqrt(r0 * r0 + k * k * std::max(sigma_n2, 0.0)));
    sp.background = sp.map_distance <= r_bg;
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
  const ClusterModelParams & model, const PosePrior & prior, MapAlignment * alignment_out)
{
  std::vector<ScanPoint> pts = projectScan(scan, sensor_pose, seg.max_range);
  if (map != nullptr && map->valid()) {
    if (seg.align_to_map) {
      const MapAlignment al = alignScanToMap(
        pts, sensor_pose.translation(), *map, map_from_tracking, prior, seg);
      labelBackground(pts, *map, al, seg);
      if (alignment_out != nullptr) {
        *alignment_out = al;
      }
    } else {
      labelBackground(pts, *map, map_from_tracking, seg.background_radius);
    }
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
