#include "amr_rl_controller/core/mask.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "amr_navigation/core/velocity_obstacle.hpp"
#include "amr_rl_controller/core/geometry.hpp"

namespace amr_rl_controller
{

namespace C = contract;
namespace core = amr_navigation::core;

namespace
{
core::DwaConfig contractDwaConfig()
{
  core::DwaConfig c;
  c.limits.acc_lim_x = C::kAMax;      // simTime 의 a
  c.sim_dt = C::kSimDt;
  c.sim_time_min = C::kSimTimeMin;
  c.sim_time_max = C::kSimTimeMax;
  c.vo_time_horizon = C::kVoTimeHorizon;
  c.vo_margin = C::kVoMargin;
  c.vo_max_range = C::kVoMaxRange;
  c.dynamic_speed_threshold = C::kDynSpeedThreshold;
  c.vo_time_tie = C::kVoTimeTie;
  c.goal_overshoot_margin = C::kGoalOvershootMargin;
  return c;
}
}  // namespace

MaskBuilder::MaskBuilder()
: dwa_(contractDwaConfig())
{
}

double MaskBuilder::dGoalForRollout(const ContractState & s)
{
  const ContractPath path(s.path);
  const auto proj = path.project({s.pose.x, s.pose.y});
  const double rem = std::max(0.0, path.length() - proj.s);
  const Point2D & g = s.path.back();
  return std::max(rem, std::hypot(g.x - s.pose.x, g.y - s.pose.y));
}

bool MaskBuilder::footprintFree(
  const std::vector<Point2D> & pts, double v, double w, double d_goal) const
{
  // DwaPlanner::rollout: poses[0] = 시작, poses[k] = 시작에서 k·sim_dt 정확 적분, k = 1..⌈T/dt⌉
  const auto poses = dwa_.rollout(core::Pose2D{}, v, w, dwa_.simTime(v));
  for (std::size_t k = 1; k < poses.size(); ++k) {
    const double s_k = std::abs(v) * static_cast<double>(k) * C::kSimDt;
    if (std::isfinite(d_goal) && s_k > d_goal + C::kGoalOvershootMargin) {
      break;
    }
    // integrateArc 는 θ 를 감지만 (geometry.cpp:191) 여기서는 cos·sin 으로만 쓴다
    if (pointsInFootprint(pts, poses[k])) {
      return false;
    }
  }
  return true;
}

MaskResult MaskBuilder::compute(const ContractState & s) const
{
  MaskResult r;
  r.vo_time.fill(kInf);
  const Accel a_prev = s.prev_actions[0];
  const double v_hi = std::min(s.v_cap, C::kVMaxHw);
  const std::vector<Point2D> pts = scanPoints(s.scan_ranges);
  const double d_goal = dGoalForRollout(s);

  // VO 대상: select_tracks(6 m) ∩ 속력 ≥ 0.2
  std::vector<SelectedTrack> obstacles;
  for (const auto & t : selectTracks(s, C::kVoMaxRange)) {
    if (t.speed >= C::kDynSpeedThreshold) {
      obstacles.push_back(t);
    }
  }
  const double R = C::robotRadius() + C::kObstacleRadius + C::kVoMargin;
  const double tol = C::kLatticeTol;

  for (int i = 0; i < C::kNCells; ++i) {
    const std::size_t k = static_cast<std::size_t>(i);
    const Accel a = cellAccel(i);
    const double v1 = s.v_cmd + a.first * C::kDt;
    const double w1 = s.w_cmd + a.second * C::kDt;
    r.v_next[k] = v1;
    r.w_next[k] = w1;
    r.speed_ok[k] = (C::kVMin - tol <= v1 && v1 <= v_hi + tol) && std::abs(w1) <= C::kWMax + tol;
    r.jerk_ok[k] = std::abs(a.first - a_prev.first) <= C::kJerkStepV + tol &&
      std::abs(a.second - a_prev.second) <= C::kJerkStepW + tol;
    if (!(r.speed_ok[k] && r.jerk_ok[k])) {
      continue;
    }
    r.fp_ok[k] = footprintFree(pts, v1, w1, d_goal);
    if (r.fp_ok[k] && !obstacles.empty()) {
      const core::Point2D cv = core::chordVelocity(0.0, v1, w1, C::kVoTimeHorizon);
      double t = kInf;
      for (const auto & o : obstacles) {
        t = std::min(
          t, core::velocityObstacleTime(
            o.p, {cv.x - o.v.x, cv.y - o.v.y}, R, C::kVoTimeHorizon));
      }
      r.vo_time[k] = t;
    }
  }

  bool any_sel = false;
  bool any_cell = false;
  double t_max = -kInf;
  for (std::size_t k = 0; k < static_cast<std::size_t>(C::kNCells); ++k) {
    const bool sel = r.speed_ok[k] && r.jerk_ok[k] && r.fp_ok[k];
    if (!sel) {
      continue;
    }
    any_sel = true;
    t_max = std::max(t_max, r.vo_time[k]);
    if (!std::isfinite(r.vo_time[k])) {
      r.mask[k] = true;
      any_cell = true;
    }
  }
  if (!any_cell && any_sel) {
    r.saturated = true;
    for (std::size_t k = 0; k < static_cast<std::size_t>(C::kNCells); ++k) {
      const bool sel = r.speed_ok[k] && r.jerk_ok[k] && r.fp_ok[k];
      r.mask[k] = sel && r.vo_time[k] >= t_max - C::kVoTimeTie;
    }
  }
  r.mask[C::kFallbackIndex] = true;
  r.n_valid = static_cast<int>(std::count(r.mask.begin(), r.mask.end() - 1, true));
  r.fallback = fallbackCommand(s.v_cmd, s.w_cmd);
  return r;
}

}  // namespace amr_rl_controller
