// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 다중 장애물 추적기 구현.

#include "amr_perception/obstacle_tracker.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "amr_perception/hungarian.hpp"

namespace amr_perception
{

namespace
{
double sigmoid(double x)
{
  return 1.0 / (1.0 + std::exp(-x));
}

// 추적 프레임에서 센서 기준 클러스터 각도 범위 (중심각, 반폭)
struct AngularSpan
{
  double center{0.0};
  double half_width{0.0};
  double range{0.0};
};

AngularSpan clusterSpan(const Cluster & c, const Vec2 & sensor)
{
  AngularSpan span;
  const Vec2 d = c.centroid - sensor;
  span.center = std::atan2(d.y(), d.x());
  span.range = c.mean_range;
  for (const auto & p : c.points) {
    const Vec2 dp = p - sensor;
    const double a = normalizeAngle(std::atan2(dp.y(), dp.x()) - span.center);
    span.half_width = std::max(span.half_width, std::abs(a));
  }
  return span;
}
}  // namespace

// ---------------------------------------------------------------- 연관

Eigen::MatrixXd buildAugmentedCostMatrix(
  const std::vector<std::vector<Innovation>> & innovations,
  const std::vector<std::vector<char>> & feasible, const std::vector<double> & pd,
  double lambda_birth, double big_cost)
{
  const int n = static_cast<int>(innovations.size());
  const int k = n > 0 ? static_cast<int>(innovations.front().size()) : 0;
  Eigen::MatrixXd c = Eigen::MatrixXd::Constant(n + k, k + n, big_cost);
  const double two_pi = 2.0 * M_PI;
  for (int i = 0; i < n; ++i) {
    const double p = std::clamp(pd[i], 1e-6, 1.0 - 1e-6);
    for (int j = 0; j < k; ++j) {
      if (!feasible[i][j]) {
        continue;
      }
      const Innovation & inn = innovations[i][j];
      const double logdet = std::log((two_pi * inn.S).determinant());
      c(i, j) = inn.mahalanobis2() + logdet - 2.0 * std::log(p);
    }
    c(i, k + i) = -2.0 * std::log(1.0 - p);  // 미탐
  }
  const double birth = -2.0 * std::log(lambda_birth);
  for (int j = 0; j < k; ++j) {
    c(n + j, j) = birth;  // 새 트랙 탄생 (또는 오경보)
    for (int i = 0; i < n; ++i) {
      c(n + j, k + i) = 0.0;  // 더미 × 더미
    }
  }
  return c;
}

AssociationResult solveGnnAssociation(
  const std::vector<std::vector<Innovation>> & innovations,
  const std::vector<std::vector<char>> & feasible, const std::vector<double> & pd,
  double lambda_birth, double big_cost)
{
  AssociationResult result;
  const int n = static_cast<int>(innovations.size());
  const int k = n > 0 ? static_cast<int>(innovations.front().size()) : 0;
  result.track_to_cluster.assign(n, -1);
  if (n == 0) {
    return result;
  }
  result.cluster_to_track.assign(k, -1);
  if (k == 0) {
    return result;
  }
  const Eigen::MatrixXd c = buildAugmentedCostMatrix(
    innovations, feasible, pd, lambda_birth, big_cost);
  const std::vector<int> assign = solveAssignment(c);
  for (int i = 0; i < n; ++i) {
    const int j = assign[i];
    if (j >= 0 && j < k && feasible[i][j]) {
      result.track_to_cluster[i] = j;
      result.cluster_to_track[j] = i;
    }
  }
  return result;
}

// ---------------------------------------------------------------- LS 속도 검정

VelocityTestResult leastSquaresVelocityTest(
  const std::vector<double> & t, const std::vector<Vec2> & z, const Mat2 & R_jitter,
  int min_samples)
{
  VelocityTestResult r;
  const std::size_t n = std::min(t.size(), z.size());
  if (static_cast<int>(n) < min_samples || n < 2) {
    return r;
  }
  double tbar = 0.0;
  Vec2 zbar = Vec2::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    tbar += t[i];
    zbar += z[i];
  }
  tbar /= static_cast<double>(n);
  zbar /= static_cast<double>(n);
  double stt = 0.0;
  Vec2 stz = Vec2::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const double dt = t[i] - tbar;
    stt += dt * dt;
    stz += dt * (z[i] - zbar);
  }
  if (stt < 1e-9) {
    return r;
  }
  r.velocity = stz / stt;
  // Cov(v̂) = R / S_tt → T = S_tt · v̂^T R^-1 v̂
  r.chi2 = stt * r.velocity.dot(R_jitter.ldlt().solve(r.velocity));
  r.valid = true;
  return r;
}

// ---------------------------------------------------------------- 추적기

ObstacleTracker::ObstacleTracker(const TrackerParams & params, FilterFactory factory)
: params_(params), factory_(std::move(factory))
{
  if (!factory_) {
    const double q = params_.q;
    const double v0 = params_.init_velocity_std;
    factory_ = [q, v0](const Vec2 & z, const Mat2 & R) {
        return std::make_unique<ConstantVelocityKalmanFilter>(q, z, R, v0);
      };
  }
}

void ObstacleTracker::reset()
{
  tracks_.clear();
  initialized_ = false;
}

int ObstacleTracker::countHits(const std::deque<bool> & history, int window)
{
  int hits = 0;
  const int n = static_cast<int>(history.size());
  for (int i = std::max(0, n - window); i < n; ++i) {
    hits += history[i] ? 1 : 0;
  }
  return hits;
}

double ObstacleTracker::detectionProbability(
  const Track & track, const std::vector<Cluster> & clusters, const EgoState & ego,
  bool * occluded) const
{
  if (occluded != nullptr) {
    *occluded = false;
  }
  const Vec2 p = track.filter->state().head<2>();
  const Vec2 d = p - ego.sensor_position;
  const double r = d.norm();
  if (r > params_.far_detection_range) {
    return params_.pd_occluded;
  }
  const double bearing = std::atan2(d.y(), d.x());
  const double track_half = std::atan2(track.radius, std::max(r, 1e-3));
  for (const auto & c : clusters) {
    if (c.mean_range >= r - params_.occlusion_margin) {
      continue;
    }
    const AngularSpan span = clusterSpan(c, ego.sensor_position);
    if (std::abs(normalizeAngle(bearing - span.center)) <= span.half_width + track_half) {
      if (occluded != nullptr) {
        *occluded = true;
      }
      return params_.pd_occluded;
    }
  }
  return params_.pd_visible;
}

void ObstacleTracker::updateDynamicState(Track & track, const EgoState & ego)
{
  std::vector<double> ts;
  std::vector<Vec2> zs;
  ts.reserve(track.samples.size());
  zs.reserve(track.samples.size());
  for (const auto & s : track.samples) {
    ts.push_back(s.t);
    zs.push_back(s.z);
  }
  const VelocityTestResult res = leastSquaresVelocityTest(
    ts, zs, track.R_jitter, params_.velocity_min_samples);
  if (!res.valid) {
    return;
  }
  track.ls_speed = res.velocity.norm();
  track.ls_chi2 = res.chi2;
  // 자차 운동에 의한 시선 회전율 |ω_LOS| = |v_sensor ⊥ LOS| / r
  const Vec2 los = track.last_measurement - ego.sensor_position;
  const double r = std::max(los.norm(), 0.1);
  const Vec2 u = los / r;
  const double v_perp = std::abs(u.x() * ego.sensor_velocity.y() - u.y() * ego.sensor_velocity.x());
  const double omega_los = v_perp / r;
  const double v_min_eff = params_.v_min + 2.0 * params_.sigma_delta * omega_los;
  const bool fire = res.chi2 > params_.velocity_chi2 && track.ls_speed > v_min_eff;
  if (fire) {
    ++track.fire_count;
    track.quiet_count = 0;
    if (track.fire_count >= params_.dynamic_consecutive) {
      track.is_dynamic = true;
    }
  } else {
    ++track.quiet_count;
    track.fire_count = 0;
    if (track.quiet_count >= params_.dynamic_release) {
      track.is_dynamic = false;
    }
  }
}

void ObstacleTracker::update(
  double stamp, const std::vector<Cluster> & clusters, const EgoState & ego)
{
  double dt = 0.0;
  if (initialized_) {
    dt = stamp - last_stamp_;
    if (dt < 0.0 || dt > params_.max_dt) {
      // 시간 역행(시뮬레이션 리셋) 또는 긴 공백: 예측이 무의미하므로 트랙을 버린다
      tracks_.clear();
      dt = 0.0;
    }
  }
  initialized_ = true;
  last_stamp_ = stamp;

  for (auto & tr : tracks_) {
    tr.filter->predict(dt);
  }

  const int n = static_cast<int>(tracks_.size());
  const int k = static_cast<int>(clusters.size());
  std::vector<std::vector<Innovation>> inn(n, std::vector<Innovation>(k));
  std::vector<std::vector<char>> feasible(n, std::vector<char>(k, 0));
  std::vector<double> pd(n, params_.pd_visible);
  for (int i = 0; i < n; ++i) {
    Track & tr = tracks_[i];
    pd[i] = detectionProbability(tr, clusters, ego, &tr.occluded);
    const double since = std::max(stamp - tr.last_update_stamp, 0.0);
    for (int j = 0; j < k; ++j) {
      inn[i][j] = tr.filter->innovation(clusters[j].measurement, clusters[j].R);
      const double d2 = inn[i][j].mahalanobis2();
      if (!(d2 <= params_.gate_chi2)) {
        continue;
      }
      Eigen::SelfAdjointEigenSolver<Mat2> es(inn[i][j].S);
      const double lmax = std::max(es.eigenvalues().maxCoeff(), 0.0);
      const double jump = (clusters[j].measurement - tr.last_measurement).norm();
      if (jump > params_.v_phys * since + 3.0 * std::sqrt(lmax)) {
        continue;
      }
      feasible[i][j] = 1;
    }
  }

  AssociationResult assoc;
  if (n > 0 && k > 0) {
    assoc = solveGnnAssociation(inn, feasible, pd, params_.lambda_birth, params_.big_cost);
  } else {
    assoc.track_to_cluster.assign(n, -1);
    assoc.cluster_to_track.assign(k, -1);
  }

  const std::size_t history_len = static_cast<std::size_t>(
    std::max(params_.confirm_window, params_.quality_window));
  for (int i = 0; i < n; ++i) {
    Track & tr = tracks_[i];
    const int j = assoc.track_to_cluster[i];
    if (j >= 0) {
      const Cluster & c = clusters[j];
      tr.filter->update(c.measurement, c.R);
      tr.history.push_back(true);
      ++tr.hits;
      tr.consecutive_misses = 0;
      tr.last_measurement = c.measurement;
      tr.last_update_stamp = stamp;
      tr.R_jitter = c.R_jitter;
      tr.radius = (1.0 - params_.radius_smoothing) * tr.radius +
        params_.radius_smoothing * c.radius;
      if (!c.occluded) {
        // 부분 가림 측정은 중심이 치우쳐 LS 속도 기울기를 오염시키므로 창에 넣지 않는다
        tr.samples.push_back(Sample{stamp, c.measurement});
        while (static_cast<int>(tr.samples.size()) > params_.velocity_window) {
          tr.samples.pop_front();
        }
        updateDynamicState(tr, ego);
      }
    } else {
      tr.history.push_back(false);
      ++tr.misses;
      ++tr.consecutive_misses;
    }
    while (tr.history.size() > history_len) {
      tr.history.pop_front();
    }
  }

  // 탄생
  for (int j = 0; j < k; ++j) {
    if (assoc.cluster_to_track[j] >= 0) {
      continue;
    }
    const Cluster & c = clusters[j];
    Track tr;
    tr.id = next_id_++;
    tr.filter = factory_(c.measurement, c.R);
    tr.first_stamp = stamp;
    tr.last_update_stamp = stamp;
    tr.last_measurement = c.measurement;
    tr.history.push_back(true);
    tr.hits = 1;
    tr.R_jitter = c.R_jitter;
    tr.radius = c.radius;
    tr.samples.push_back(Sample{stamp, c.measurement});
    tracks_.push_back(std::move(tr));
  }

  // 생명주기
  const int max_tentative_misses = params_.confirm_window - params_.confirm_hits + 1;
  std::vector<Track> kept;
  kept.reserve(tracks_.size());
  for (auto & tr : tracks_) {
    const int window_hits = countHits(tr.history, params_.confirm_window);
    if (!tr.confirmed) {
      const int window_len = std::min<int>(
        static_cast<int>(tr.history.size()), params_.confirm_window);
      const int window_misses = window_len - window_hits;
      if (window_hits >= params_.confirm_hits &&
        stamp - tr.first_stamp >= params_.min_confirm_age - 1e-6)
      {
        tr.confirmed = true;
      } else if (window_misses >= max_tentative_misses) {
        continue;  // 확정 불가 → 삭제
      }
    } else {
      // 가려진 동안은 더 오래 예측만으로 유지한다
      const int allowed = tr.occluded ? params_.max_misses_occluded : params_.max_misses;
      if (tr.consecutive_misses > allowed) {
        continue;
      }
    }
    const Vec4 x = tr.filter->state();
    const double speed = x.tail<2>().norm();
    if (speed >= params_.heading_min_speed) {
      tr.last_heading = std::atan2(x(3), x(2));
      tr.heading_valid = true;
    }
    kept.push_back(std::move(tr));
  }
  tracks_ = std::move(kept);
}

TrackOutput ObstacleTracker::makeOutput(const Track & tr) const
{
  TrackOutput o;
  const Vec4 x = tr.filter->state();
  const Mat4 p = tr.filter->covariance();
  o.id = tr.id;
  o.position = x.head<2>();
  o.velocity = x.tail<2>();
  o.covariance = p;
  o.speed = o.velocity.norm();
  o.heading = tr.last_heading;
  if (o.speed >= params_.heading_min_speed) {
    const Vec2 nrm(-o.velocity.y() / o.speed, o.velocity.x() / o.speed);
    const double var = nrm.dot(p.bottomRightCorner<2, 2>() * nrm);
    o.heading_std = std::min(std::sqrt(std::max(var, 0.0)) / o.speed, M_PI);
  } else {
    o.heading_std = M_PI;
  }
  const int window_len = std::min<int>(static_cast<int>(tr.history.size()), params_.quality_window);
  const double hit_ratio = window_len > 0 ?
    static_cast<double>(countHits(tr.history, params_.quality_window)) / window_len : 0.0;
  const double tr_ppp = std::max(p(0, 0) + p(1, 1), 1e-12);
  const auto & b = params_.confidence_beta;
  const double sref2 = params_.confidence_sigma_ref * params_.confidence_sigma_ref;
  o.class_confidence = 0.0;
  o.confidence = sigmoid(
    b[0] + b[1] * hit_ratio + b[2] * std::log(tr_ppp / sref2) + b[3] * o.class_confidence);
  o.is_dynamic = tr.is_dynamic;
  o.confirmed = tr.confirmed;
  o.radius = tr.radius;
  o.hits = tr.hits;
  o.misses = tr.misses;
  o.consecutive_misses = tr.consecutive_misses;
  o.age = last_stamp_ - tr.first_stamp;
  o.ls_speed = tr.ls_speed;
  o.ls_chi2 = tr.ls_chi2;
  return o;
}

std::vector<TrackOutput> ObstacleTracker::outputs(bool confirmed_only) const
{
  std::vector<TrackOutput> out;
  for (const auto & tr : tracks_) {
    if (confirmed_only && !tr.confirmed) {
      continue;
    }
    out.push_back(makeOutput(tr));
  }
  return out;
}

}  // namespace amr_perception
