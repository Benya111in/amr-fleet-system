#include "amr_rl_controller/core/observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_rl_controller/core/geometry.hpp"

namespace amr_rl_controller
{

namespace C = contract;

namespace
{
double clip(double x, double lo, double hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}
}  // namespace

std::array<double, C::kNSectors> lidarSectors(const std::vector<double> & ranges)
{
  if (static_cast<int>(ranges.size()) != C::kLidarSamples) {
    throw std::invalid_argument(
            "빔 수 " + std::to_string(ranges.size()) + " != " + std::to_string(C::kLidarSamples));
  }
  std::array<double, C::kNSectors> sec{};
  for (int k = 0; k < C::kNSectors; ++k) {
    double m = C::kSectorClip;
    for (int j = 0; j < C::kBeamsPerSector; ++j) {
      const int i = k * C::kBeamsPerSector + j;
      const double r = ranges[static_cast<std::size_t>(i)];
      double clr = C::kSectorClip;
      if (beamValid(r)) {
        const double a = beamAngle(i);
        clr = std::min(
          footprintClearance({C::kLidarX + r * std::cos(a), C::kLidarY + r * std::sin(a)}),
          C::kSectorClip);
      }
      m = std::min(m, clr);
    }
    sec[static_cast<std::size_t>(k)] = m / C::kSectorClip;
  }
  return sec;
}

Observation buildObservation(const ContractState & s)
{
  std::array<double, C::kObsDim> o{};
  // lidar_sectors
  const auto sec = lidarSectors(s.scan_ranges);
  std::copy(sec.begin(), sec.end(), o.begin() + C::kObsLidar);

  // ego
  {
    std::array<double, 6 + 2 * C::kNPrevActions> ego{
      s.v_meas / C::kVMaxHw, s.w_meas / C::kWMax, s.a_v_meas / C::kAMax,
      s.a_w_meas / C::kAlphaMax, s.v_cmd / C::kVMaxHw, s.w_cmd / C::kWMax};
    for (int k = 0; k < C::kNPrevActions; ++k) {
      const auto & a = s.prev_actions[static_cast<std::size_t>(k)];
      ego[static_cast<std::size_t>(6 + 2 * k)] = a.first / C::kAMax;
      ego[static_cast<std::size_t>(7 + 2 * k)] = a.second / C::kAlphaMax;
    }
    for (std::size_t k = 0; k < ego.size(); ++k) {
      o[static_cast<std::size_t>(C::kObsEgo) + k] = clip(ego[k], -3.0, 3.0);
    }
  }

  // timing
  o[C::kObsTiming + 0] = clip(s.scan_age / C::kScanAgeNorm, 0.0, 1.0);
  o[C::kObsTiming + 1] = clip(s.track_age / C::kTrackAgeNorm, 0.0, 1.0);
  o[C::kObsTiming + 2] = clip(s.v_cap / C::kVMaxHw, 0.0, 1.0);

  // path
  const ContractPath path(s.path);
  const auto proj = path.project({s.pose.x, s.pose.y});
  {
    std::size_t i = C::kObsPath;
    for (int k = 1; k <= C::kNSubgoals; ++k) {
      const Point2D g = path.interpolate(proj.s + C::kSubgoalSpacing * k);
      const Point2D q = toRobot(s.pose, g);
      const double d = std::hypot(q.x, q.y);
      const double b = d > 1e-9 ? std::atan2(q.y, q.x) : 0.0;
      o[i++] = clip(d / C::kSubgoalDistNorm, 0.0, 2.0);
      o[i++] = std::sin(b);
      o[i++] = std::cos(b);
    }
    const double e_psi = wrapAngle(s.pose.theta - proj.seg_yaw);
    o[i++] = clip(proj.signed_e / C::kDevPeakNorm, -2.0, 2.0);
    o[i++] = std::sin(e_psi);
    o[i++] = std::cos(e_psi);
  }

  // goal
  {
    const double d_arc = std::max(0.0, path.length() - proj.s);
    const Point2D g = toRobot(s.pose, s.path.back());
    const double dg = std::hypot(g.x, g.y);
    const double bg = dg > 1e-9 ? std::atan2(g.y, g.x) : 0.0;
    o[C::kObsGoal + 0] = std::min(d_arc, C::kGoalClip) / C::kGoalClip;
    o[C::kObsGoal + 1] = std::min(dg, C::kGoalClip) / C::kGoalClip;
    o[C::kObsGoal + 2] = std::sin(bg);
    o[C::kObsGoal + 3] = std::cos(bg);
  }

  // safety
  {
    const int z = s.safety.zone;
    if (z < 0 || z > 3) {
      throw std::invalid_argument("safety zone 범위 밖: " + std::to_string(z));
    }
    o[static_cast<std::size_t>(C::kObsSafety + z)] = 1.0;
    o[C::kObsSafety + 4] = std::min(s.safety.speed_limit, C::kVMaxHw) / C::kVMaxHw;
    o[C::kObsSafety + 5] = s.safety.proximity_stop ? 1.0 : 0.0;
    o[C::kObsSafety + 6] = std::min(s.safety.min_ttc, C::kTtcHorizon) / C::kTtcHorizon;
  }

  // dev
  if (s.dev.in_dev) {
    o[C::kObsDev + 0] = 1.0;
    o[C::kObsDev + 1] = clip(s.dev.peak / C::kDevPeakNorm, 0.0, 2.0);
    o[C::kObsDev + 2] = clip(s.dev.t_since_peak / C::kDevTimerNorm, 0.0, 2.0);
  }

  // tracks
  {
    const auto sel = selectTracks(s, C::kTrackRange);
    const std::size_t n = std::min<std::size_t>(sel.size(), C::kKTracks);
    for (std::size_t k = 0; k < n; ++k) {
      const auto & t = sel[k];
      const double ttc = trackTtc(t.p, t.v, s.v_meas);
      const std::size_t b = static_cast<std::size_t>(C::kObsTracks) + k * C::kTrackFeats;
      o[b + 0] = t.p.x / C::kTrackPosNorm;
      o[b + 1] = t.p.y / C::kTrackPosNorm;
      o[b + 2] = t.v.x / C::kTrackVelNorm;
      o[b + 3] = t.v.y / C::kTrackVelNorm;
      o[b + 4] = t.dist / C::kTrackPosNorm;
      o[b + 5] = std::min(ttc, C::kTtcHorizon) / C::kTtcHorizon;
      o[b + 6] = t.is_dynamic ? 1.0 : 0.0;
      o[b + 7] = 1.0;
    }
  }

  Observation out;
  for (std::size_t k = 0; k < out.size(); ++k) {
    out[k] = static_cast<float>(o[k]);
  }
  return out;
}

}  // namespace amr_rl_controller
