#include "amr_rl_controller/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "amr_navigation/core/velocity_obstacle.hpp"

namespace amr_rl_controller
{

namespace C = contract;

double wrapAngle(double a)
{
  // 파이썬 (a + π) % 2π − π 와 같은 결과 (% 는 나눗수 부호를 따른다)
  constexpr double kPi = amr_navigation::core::kPi;
  const double two_pi = 2.0 * kPi;
  double r = std::fmod(a + kPi, two_pi);
  if (r < 0.0) {
    r += two_pi;
  }
  return r - kPi;
}

ContractPath::ContractPath(const std::vector<Point2D> & pts)
: pts_(pts)
{
  if (pts_.empty()) {
    throw std::invalid_argument("빈 경로");
  }
  cum_.resize(pts_.size(), 0.0);
  for (std::size_t i = 1; i < pts_.size(); ++i) {
    cum_[i] = cum_[i - 1] + std::hypot(pts_[i].x - pts_[i - 1].x, pts_[i].y - pts_[i - 1].y);
  }
}

ContractPath::Projection ContractPath::project(const Point2D & p) const
{
  Projection out;
  if (pts_.size() == 1) {
    const double d = std::hypot(p.x - pts_[0].x, p.y - pts_[0].y);
    out.s = 0.0;
    out.dist = d;
    out.signed_e = d;
    out.seg_yaw = 0.0;
    return out;
  }
  bool found = false;
  for (std::size_t i = 0; i + 1 < pts_.size(); ++i) {
    const double ax = pts_[i].x;
    const double ay = pts_[i].y;
    const double sx = pts_[i + 1].x - ax;
    const double sy = pts_[i + 1].y - ay;
    const double ln2 = sx * sx + sy * sy;
    if (ln2 < 1e-18) {
      continue;
    }
    double u = ((p.x - ax) * sx + (p.y - ay) * sy) / ln2;
    u = std::min(1.0, std::max(0.0, u));
    const double qx = ax + u * sx;
    const double qy = ay + u * sy;
    const double d = std::hypot(p.x - qx, p.y - qy);
    if (!found || d < out.dist - 1e-12) {
      const double cross = sx * (p.y - ay) - sy * (p.x - ax);
      const double sign = cross >= 0.0 ? 1.0 : -1.0;
      out.s = cum_[i] + u * std::sqrt(ln2);
      out.dist = d;
      out.signed_e = sign * d;
      out.seg_yaw = std::atan2(sy, sx);
      found = true;
    }
  }
  if (!found) {   // 모든 구간 길이 0
    const double d = std::hypot(p.x - pts_[0].x, p.y - pts_[0].y);
    out = Projection{0.0, d, d, 0.0};
  }
  return out;
}

Point2D ContractPath::interpolate(double s) const
{
  if (s <= 0.0) {
    return pts_.front();
  }
  if (s >= cum_.back()) {
    return pts_.back();
  }
  // np.searchsorted(cum, s, side='right') - 1
  const auto it = std::upper_bound(cum_.begin(), cum_.end(), s);
  long i = static_cast<long>(it - cum_.begin()) - 1;   // NOLINT(runtime/int)
  i = std::min(std::max(i, 0L), static_cast<long>(pts_.size()) - 2);   // NOLINT(runtime/int)
  const std::size_t k = static_cast<std::size_t>(i);
  const double ln = cum_[k + 1] - cum_[k];
  const double u = ln < 1e-18 ? 0.0 : (s - cum_[k]) / ln;
  return {pts_[k].x + u * (pts_[k + 1].x - pts_[k].x), pts_[k].y + u * (pts_[k + 1].y - pts_[k].y)};
}

Point2D toRobot(const Pose2D & pose, const Point2D & p)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const double dx = p.x - pose.x;
  const double dy = p.y - pose.y;
  return {c * dx + s * dy, -s * dx + c * dy};
}

Point2D rotateToRobot(double theta, const Point2D & v)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  return {c * v.x + s * v.y, -s * v.x + c * v.y};
}

double footprintClearance(const Point2D & p)
{
  const double dx = std::max(std::abs(p.x) - C::kHalfL, 0.0);
  const double dy = std::max(std::abs(p.y) - C::kHalfW, 0.0);
  return std::hypot(dx, dy);
}

double beamAngle(int i)
{
  return C::kLidarAngleMin + static_cast<double>(i) * C::kLidarAngleInc;
}

bool beamValid(double r)
{
  return std::isfinite(r) && r >= C::kLidarRangeMin && r <= C::kLidarRangeMax;
}

std::vector<Point2D> scanPoints(const std::vector<double> & ranges)
{
  if (static_cast<int>(ranges.size()) != C::kLidarSamples) {
    throw std::invalid_argument(
            "빔 수 " + std::to_string(ranges.size()) + " != " + std::to_string(C::kLidarSamples));
  }
  std::vector<Point2D> pts;
  pts.reserve(ranges.size());
  for (int i = 0; i < C::kLidarSamples; ++i) {
    const double r = ranges[static_cast<std::size_t>(i)];
    if (!beamValid(r)) {
      continue;
    }
    const double a = beamAngle(i);
    pts.push_back({C::kLidarX + r * std::cos(a), C::kLidarY + r * std::sin(a)});
  }
  return pts;
}

bool pointsInFootprint(const std::vector<Point2D> & pts, const Pose2D & pose)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const double hl = C::kHalfL + C::kFpPad;
  const double hw = C::kHalfW + C::kFpPad;
  for (const auto & q : pts) {
    const double dx = q.x - pose.x;
    const double dy = q.y - pose.y;
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;
    if (std::abs(lx) <= hl && std::abs(ly) <= hw) {
      return true;
    }
  }
  return false;
}

std::vector<SelectedTrack> selectTracks(const ContractState & st, double max_range)
{
  std::vector<SelectedTrack> out;
  const double age = std::max(0.0, st.track_age);
  for (const auto & t : st.tracks) {
    const double speed = std::hypot(t.vx, t.vy);
    if (!t.is_dynamic && speed < C::kDynFastSpeed) {
      continue;
    }
    const Point2D pm{t.x + t.vx * age, t.y + t.vy * age};
    SelectedTrack r;
    r.p = toRobot(st.pose, pm);
    r.v = rotateToRobot(st.pose.theta, {t.vx, t.vy});
    r.dist = std::hypot(r.p.x, r.p.y);
    if (r.dist > max_range) {
      continue;
    }
    r.id = t.id;
    r.is_dynamic = t.is_dynamic;
    r.speed = speed;
    out.push_back(r);
  }
  std::sort(
    out.begin(), out.end(), [](const SelectedTrack & a, const SelectedTrack & b) {
      return std::tie(a.dist, a.id) < std::tie(b.dist, b.id);
    });
  return out;
}

double trackTtc(const Point2D & p, const Point2D & v_obs_robot, double v_meas)
{
  const Point2D w_rel{v_meas - v_obs_robot.x, -v_obs_robot.y};
  return amr_navigation::core::velocityObstacleTime(
    p, w_rel, C::robotRadius() + C::kObstacleRadius, C::kTtcHorizon);
}

}  // namespace amr_rl_controller
