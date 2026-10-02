// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 충돌 예측과 TTC 구현.

#include "amr_perception/ttc.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace amr_perception
{

RobotMotionModel RobotMotionModel::stationary(const Vec2 & position)
{
  RobotMotionModel m;
  m.kind_ = Kind::Line;
  m.origin_ = position;
  m.speed_ = 0.0;
  return m;
}

RobotMotionModel RobotMotionModel::straightLine(const Vec2 & position, double yaw, double speed)
{
  RobotMotionModel m;
  m.kind_ = Kind::Line;
  m.origin_ = position;
  m.direction_ = Vec2(std::cos(yaw), std::sin(yaw));
  m.speed_ = speed;
  return m;
}

bool RobotMotionModel::fromPath(
  const std::vector<Vec2> & path, const Vec2 & position, double speed, double max_deviation,
  RobotMotionModel & out)
{
  if (path.size() < 2) {
    return false;
  }
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] + (path[i] - path[i - 1]).norm();
  }
  // 로봇 위치의 경로 사영 (가장 가까운 선분)
  double best = std::numeric_limits<double>::infinity();
  double s0 = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Vec2 a = path[i - 1];
    const Vec2 ab = path[i] - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 1e-12 ? std::clamp((position - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const double d = (position - (a + t * ab)).norm();
    if (d < best) {
      best = d;
      s0 = arc[i - 1] + t * std::sqrt(len2);
    }
  }
  if (best > max_deviation) {
    return false;
  }
  out = RobotMotionModel();
  out.kind_ = Kind::Path;
  out.path_ = path;
  out.arc_ = std::move(arc);
  out.s0_ = s0;
  out.speed_ = std::abs(speed);
  out.origin_ = position;
  return true;
}

RobotMotionModel RobotMotionModel::select(
  const std::vector<Vec2> & path, const Vec2 & position, double yaw, double speed,
  const TtcParams & params)
{
  RobotMotionModel m;
  const double path_speed = std::max(std::abs(speed), params.min_robot_speed);
  if (fromPath(path, position, path_speed, params.max_path_deviation, m)) {
    return m;
  }
  if (std::abs(speed) < 1e-3) {
    return stationary(position);
  }
  return straightLine(position, yaw, speed);
}

Vec2 RobotMotionModel::positionAt(double tau) const
{
  if (kind_ == Kind::Line) {
    return origin_ + tau * speed_ * direction_;
  }
  const double s = std::min(s0_ + speed_ * tau, arc_.back());
  // 이분 탐색으로 선분 찾기
  const auto it = std::upper_bound(arc_.begin(), arc_.end(), s);
  std::size_t i = static_cast<std::size_t>(std::distance(arc_.begin(), it));
  if (i == 0) {
    return path_.front();
  }
  if (i >= arc_.size()) {
    return path_.back();
  }
  const double seg = arc_[i] - arc_[i - 1];
  const double t = seg > 1e-12 ? (s - arc_[i - 1]) / seg : 0.0;
  return path_[i - 1] + t * (path_[i] - path_[i - 1]);
}

Mat2 ObstacleMotion::covarianceAt(double tau) const
{
  return P_pp + tau * (P_pv + P_pv.transpose()) + tau * tau * P_vv +
         (q * tau * tau * tau / 3.0) * Mat2::Identity();
}

double collisionClearance(
  const ObstacleMotion & obstacle, const RobotMotionModel & robot, const TtcParams & params,
  double tau)
{
  const Vec2 d = robot.positionAt(tau) - obstacle.positionAt(tau);
  const double dist = d.norm();
  const Vec2 u = dist > 1e-9 ? Vec2(d / dist) : Vec2(1.0, 0.0);
  const Mat2 cov = obstacle.covarianceAt(tau);
  const double sigma = std::sqrt(std::max(u.dot(cov * u), 0.0));
  const double inflate = std::min(params.k_sigma * sigma, params.sigma_cap);
  return dist - (params.robot_radius + obstacle.radius + inflate);
}

TtcResult computeTimeToCollision(
  const ObstacleMotion & obstacle, const RobotMotionModel & robot, const TtcParams & params)
{
  TtcResult result;
  const double step = params.step > 1e-6 ? params.step : 0.1;
  const int steps = static_cast<int>(std::ceil(params.horizon / step));
  double prev_tau = 0.0;
  const double g0 = collisionClearance(obstacle, robot, params, 0.0);
  result.cpa_distance = (robot.positionAt(0.0) - obstacle.positionAt(0.0)).norm();
  result.cpa_time = 0.0;
  if (g0 <= 0.0) {
    result.ttc = 0.0;
    return result;
  }
  for (int i = 1; i <= steps; ++i) {
    const double tau = std::min(i * step, params.horizon);
    const double dist = (robot.positionAt(tau) - obstacle.positionAt(tau)).norm();
    if (dist < result.cpa_distance) {
      result.cpa_distance = dist;
      result.cpa_time = tau;
    }
    const double g = collisionClearance(obstacle, robot, params, tau);
    if (g <= 0.0) {
      // [prev_tau, tau] 에서 부호가 바뀐다 → 이분법
      double lo = prev_tau;
      double hi = tau;
      for (int it = 0; it < params.refine_iterations; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (collisionClearance(obstacle, robot, params, mid) <= 0.0) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      result.ttc = hi;
      return result;
    }
    prev_tau = tau;
  }
  return result;
}

}  // namespace amr_perception
