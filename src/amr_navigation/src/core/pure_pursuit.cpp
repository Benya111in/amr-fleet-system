#include "amr_navigation/core/pure_pursuit.hpp"

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
SpeedProfileConfig toProfileConfig(const PurePursuitConfig & c)
{
  SpeedProfileConfig p;
  p.desired_speed = c.desired_linear_vel;
  p.max_accel = c.max_linear_accel;
  p.max_decel = c.max_linear_decel;
  p.max_angular_vel = c.max_angular_vel;
  p.max_angular_accel = c.max_angular_accel;
  p.max_lateral_accel = c.max_lateral_accel;
  p.max_jerk = c.max_linear_jerk;
  p.min_speed = c.min_approach_vel;
  p.stop_latency = c.approach_latency;
  p.curvature_window = c.curvature_window;
  return p;
}
}  // namespace

PurePursuit::PurePursuit(const PurePursuitConfig & config)
: config_(config)
{
}

void PurePursuit::setConfig(const PurePursuitConfig & config)
{
  config_ = config;
  if (!path_.empty()) {
    profile_.build(path_, toProfileConfig(config_), true);
  }
}

void PurePursuit::setPath(const std::vector<Pose2D> & path)
{
  path_ = path;
  cum_s_ = cumulativeLength(path_);
  profile_.build(path_, toProfileConfig(config_), true);
  hint_ = 0;
  fresh_ = true;
}

double PurePursuit::lookaheadDistance(double v, const PurePursuitConfig & c)
{
  return std::clamp(c.lookahead_time * std::abs(v), c.min_lookahead, c.max_lookahead);
}

double PurePursuit::curvatureToPoint(double x, double y)
{
  const double l2 = x * x + y * y;
  if (l2 < 1e-9) {
    return 0.0;
  }
  return 2.0 * y / l2;
}

double PurePursuit::rotationCommand(double angle, double w_max, double alpha_max)
{
  const double mag = std::min(w_max, std::sqrt(2.0 * alpha_max * std::abs(angle)));
  return angle >= 0.0 ? mag : -mag;
}

Point2D PurePursuit::findLookahead(
  const std::vector<Pose2D> & path, const Point2D & c, double L, std::size_t start_segment)
{
  if (path.empty()) {
    return c;
  }
  for (std::size_t i = start_segment; i + 1 < path.size(); ++i) {
    const Point2D a{path[i].x, path[i].y};
    const Point2D b{path[i + 1].x, path[i + 1].y};
    if (distance(b, c) < L) {
      continue;   // 이 선분 끝이 원 안 → 다음 선분
    }
    // |a + t(b−a) − c|² = L², 원 밖으로 나가는 근 (큰 근)
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double fx = a.x - c.x;
    const double fy = a.y - c.y;
    const double A = dx * dx + dy * dy;
    const double B = 2.0 * (fx * dx + fy * dy);
    const double C = fx * fx + fy * fy - L * L;
    if (A < 1e-12) {
      return b;
    }
    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) {
      return b;
    }
    const double t = std::clamp((-B + std::sqrt(disc)) / (2.0 * A), 0.0, 1.0);
    return {a.x + t * dx, a.y + t * dy};
  }
  return {path.back().x, path.back().y};
}

PurePursuitOutput PurePursuit::compute(const Pose2D & pose, double v_now, double speed_limit)
{
  PurePursuitOutput out;
  if (path_.empty()) {
    out.mode = PurePursuitMode::kNoPath;
    return out;
  }
  const Point2D p{pose.x, pose.y};
  // 투영 (처음에는 전역 탐색, 이후 직전 인덱스 부근 창)
  Projection proj;
  if (fresh_ || path_.size() < 3) {
    proj = projectOntoPath(path_, p, 0, 0, &cum_s_);
    fresh_ = false;
  } else {
    const std::size_t begin = hint_ > 20 ? hint_ - 20 : 0;
    const std::size_t end = std::min(path_.size(), hint_ + 200);
    proj = projectOntoPath(path_, p, begin, end, &cum_s_);
  }
  hint_ = proj.segment;
  out.s = proj.s;
  out.cte = proj.cte;
  out.d_goal = std::max(
    std::max(0.0, cum_s_.back() - proj.s), distance(p, Point2D{path_.back().x, path_.back().y}));

  // 목표 방향 정렬
  if (out.d_goal <= config_.goal_align_distance) {
    const double dth = wrapAngle(path_.back().theta - pose.theta);
    if (std::abs(dth) <= config_.goal_yaw_tolerance) {
      out.mode = PurePursuitMode::kGoalReached;
      return out;
    }
    out.mode = PurePursuitMode::kRotateToGoal;
    out.w = rotationCommand(dth, config_.max_angular_vel, config_.max_angular_accel);
    return out;
  }

  // look-ahead 점과 곡률
  const double L = lookaheadDistance(v_now, config_);
  const Point2D carrot = findLookahead(path_, p, L, proj.segment);
  const Point2D local = toRobotFrame(pose, carrot);
  out.lookahead = std::hypot(local.x, local.y);
  out.carrot = carrot;
  out.carrot_angle = std::atan2(local.y, local.x);
  double kappa = curvatureToPoint(local.x, local.y);

  if (config_.use_chord_correction && out.lookahead > 1e-3) {
    // CC-PP: 기준 현 G⁰ (투영점에서 경로를 따라 L 앞, Frenet
    //   좌표) 의 횡성분을 빼고 경로 곡률을 피드포워드
    const Pose2D g0 = interpolateAt(path_, cum_s_, proj.s + out.lookahead);
    const double ch = std::cos(proj.heading);
    const double sh = std::sin(proj.heading);
    const double dx0 = g0.x - proj.point.x;
    const double dy0 = g0.y - proj.point.y;
    const double y_g0 = -sh * dx0 + ch * dy0;
    const auto & kv = profile_.curvature();
    const std::size_t idx = std::min(kv.size() - 1, proj.segment + (proj.t > 0.5 ? 1 : 0));
    const double l2 = out.lookahead * out.lookahead;
    kappa = kv[idx] + config_.chord_gain * 2.0 * (local.y - y_g0) / l2;
  }
  out.curvature = kappa;

  // 제자리 회전 (경로 방향 오차가 크고 거의 정지)
  if (config_.use_rotate_to_heading &&
    std::abs(out.carrot_angle) > config_.rotate_to_heading_min_angle &&
    std::abs(v_now) <= config_.rotate_to_heading_max_v)
  {
    out.mode = PurePursuitMode::kRotateToPath;
    out.w = rotationCommand(out.carrot_angle, config_.max_angular_vel, config_.max_angular_accel);
    return out;
  }

  // 속도 조절
  const double horizon = config_.desired_linear_vel * config_.desired_linear_vel /
    (2.0 * std::max(1e-6, config_.max_linear_decel)) + out.lookahead + 0.5;
  double v = std::min(config_.desired_linear_vel, speed_limit);
  v = std::min(v, profile_.allowedSpeed(proj.s, horizon));
  v = std::min(v, SpeedProfile::curvatureCap(kappa, toProfileConfig(config_)));
  v *= std::max(0.0, std::cos(out.carrot_angle));
  v = std::max(v, std::min(config_.min_approach_vel, speed_limit));
  double w = v * kappa;
  if (std::abs(w) > config_.max_angular_vel) {
    w = std::copysign(config_.max_angular_vel, w);
    v = config_.max_angular_vel / std::abs(kappa);   // 곡률 보존
  }
  out.v = v;
  out.w = w;
  out.mode = PurePursuitMode::kTrack;
  return out;
}

}  // namespace core
}  // namespace amr_navigation
