#include "amr_navigation/core/crossing_gate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "amr_navigation/core/crossing_yield.hpp"
#include "amr_navigation/core/speed_profile.hpp"

namespace amr_navigation
{
namespace core
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kStep = 0.05;   // [m] 정지점 탐색 간격 (경로 정점 간격)

double sumRadius(const GateTrack & t, const GateConfig & cfg)
{
  return cfg.robot_radius + t.radius + cfg.pos_err;
}

bool isActive(const GateTrack & t, const GateConfig & cfg)
{
  return t.heading_known || t.stationary_s <= cfg.static_timeout;
}

/// 점 q 가 방향을 아는 트랙의 **앞쪽** 진행선 통로 안인가 (되돌아옴은 보지 않는다 — 지나간 보행자
/// 뒤에서까지 물러나면 지나갈 때마다 후진한다)
bool inForwardCorridor(const GateTrack & t, const Point2D & q, const GateConfig & cfg)
{
  if (!t.heading_known) {
    return false;
  }
  const double R = cfg.robot_radius + t.radius + cfg.pos_err;
  const double dx = q.x - t.x;
  const double dy = q.y - t.y;
  const double along = dx * t.ux + dy * t.uy;
  const double lateral = std::abs(-dx * t.uy + dy * t.ux);
  return along >= -R && lateral <= R + std::abs(along) * std::sin(cfg.heading_unc);
}

/// 트랙 진행선까지의 수직 거리 (방향을 모르면 트랙 중심까지의 거리)
double lineDistance(const GateTrack & t, const Point2D & q)
{
  const double dx = q.x - t.x;
  const double dy = q.y - t.y;
  if (!t.heading_known) {
    return std::hypot(dx, dy);
  }
  return std::abs(-dx * t.uy + dy * t.ux);
}

struct Group
{
  std::size_t first{0};
  std::size_t last{0};
  GateReason reason{GateReason::kNone};
};
}  // namespace

bool waitingSpotThreatened(const GateTrack & t, const Point2D & q, const GateConfig & cfg)
{
  if (!t.heading_known) {
    // 방향을 모르는(막 서 있는) 보행자는 어느 쪽으로든 걸을 수 있다 — 반환점에서 도는 작업자가
    // 걷기 시작하면 그 진행선이 로봇 자리를 지날 수 있으므로, 기다리는 동안(hold_exclusion_horizon)
    // 닿을 수 있는 자리에는 서지 않는다. 3 s 원판만 보면 로봇이 차선 축 0.3 m 까지 들어간 뒤에야
    // 통로가 생긴다 (포팅 시뮬레이션 B→A: 반환점 정지 4 s 뒤 출발한 작업자에 12/610 접촉)
    return canReach(t, q, cfg.hold_exclusion_horizon, cfg);
  }
  const double R = cfg.robot_radius + t.radius + cfg.pos_err;
  const double dx = q.x - t.x;
  const double dy = q.y - t.y;
  const double along = dx * t.ux + dy * t.uy;
  const double lateral = std::abs(-dx * t.uy + dy * t.ux);
  if (lateral > R + std::abs(along) * std::sin(cfg.heading_unc)) {
    return false;   // 진행선 통로 밖
  }
  if (along >= -R) {
    return true;    // 앞·옆: 언제든 걸어온다
  }
  return reversalReturnTime(-along - R, t.speed, cfg) <= cfg.hold_exclusion_horizon;
}

double reversalReturnTime(double behind, double speed, const GateConfig & cfg)
{
  const double a = std::max(1e-6, cfg.walk_accel);
  const double v = std::clamp(speed, 0.0, cfg.v_max);
  const double t_stop = v / a;
  const double d_stop = v * v / (2.0 * a);
  const double d_back = std::max(0.0, behind) + d_stop;   // 선 자리에서 되돌아갈 거리
  const double d_acc = cfg.v_max * cfg.v_max / (2.0 * a);
  const double t_back = d_back <= d_acc ?
    std::sqrt(2.0 * d_back / a) : cfg.v_max / a + (d_back - d_acc) / cfg.v_max;
  return t_stop + cfg.turn_time + t_back;
}

bool canReach(
  const GateTrack & t, const Point2D & q, double tau, const GateConfig & cfg, GateReason * why)
{
  auto say = [why](GateReason r) {
      if (why != nullptr) {
        *why = r;
      }
    };
  const double R = sumRadius(t, cfg);
  const double dx = q.x - t.x;
  const double dy = q.y - t.y;
  const double d = std::hypot(dx, dy);
  if (d <= R) {
    say(GateReason::kTouching);
    return true;
  }
  tau = std::max(0.0, tau);
  if (!t.heading_known) {
    // 방향을 모른다 — 어느 쪽으로든 v_max 로 갈 수 있다 (반환점에서 도는 보행자)
    if (cfg.v_max * tau >= d - R) {
      say(GateReason::kUnknownHeading);
      return true;
    }
    return false;
  }
  const double along = dx * t.ux + dy * t.uy;
  const double lateral = std::abs(-dx * t.uy + dy * t.ux);
  if (lateral > R + std::abs(along) * std::sin(cfg.heading_unc)) {
    return false;   // 진행선 통로 밖
  }
  if (along >= -R) {
    // 앞 또는 옆: 속도 크기는 믿지 않는다 — [0, v_max] 어느 속력이든 가능
    if (cfg.v_max * tau >= std::max(0.0, along - R)) {
      say(GateReason::kForward);
      return true;
    }
    return false;
  }
  // 뒤: 멈추고 돌아서 되돌아오는 최단 시간
  if (reversalReturnTime(-along - R, t.speed, cfg) <= tau) {
    say(GateReason::kReversal);
    return true;
  }
  return false;
}

GateResult evaluateGate(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<GateTrack> & tracks, const GateConfig & cfg,
  const GateState & prev)
{
  GateResult out;
  out.next = prev;
  if (!cfg.enable || path.size() < 2 || cum_s.size() != path.size()) {
    out.next = GateState{};
    return out;
  }
  v_robot = std::max(0.0, v_robot);
  const Point2D robot_pt{robot.x, robot.y};
  const double s_end = std::min(cum_s.back(), s_robot + cfg.lookahead);
  auto arrival = [&](double s) {
      return travelTime(std::max(0.0, s - s_robot), v_robot, cfg.accel, cfg.robot_v_max);
    };

  // --- 트랙별 도달가능 구간 (로봇이 전속으로 달릴 때 그 시각에 닿을 수 있는 경로점) ---
  std::vector<GateTrack> active;
  active.reserve(tracks.size());
  for (const GateTrack & t : tracks) {
    if (isActive(t, cfg)) {
      active.push_back(t);
    }
  }
  std::size_t i0 = 0;
  while (i0 + 1 < path.size() && cum_s[i0] < s_robot - 1e-9) {
    ++i0;
  }
  for (std::size_t k = 0; k < active.size(); ++k) {
    GateConflict c;
    c.track = k;
    bool found = false;
    for (std::size_t i = i0; i < path.size() && cum_s[i] <= s_end + 1e-9; ++i) {
      GateReason why = GateReason::kNone;
      if (canReach(active[k], {path[i].x, path[i].y}, arrival(cum_s[i]) + cfg.delay, cfg, &why)) {
        if (!found) {
          found = true;
          c.first = i;
          c.reason = why;
        }
        c.last = i;
      }
    }
    if (found) {
      out.conflicts.push_back(c);
    }
  }
  out.n_threats = static_cast<int>(out.conflicts.size());
  std::sort(
    out.conflicts.begin(), out.conflicts.end(),
    [](const GateConflict & a, const GateConflict & b) {return a.first < b.first;});
  // 앞선 커밋이 판정한 트랙(면제)은 구간을 만들지 않는다 — 커밋 뒤에 **나타난** 트랙만 새로 본다.
  // 커밋을 "출구까지 아무것도 안 본다" 로 두면 판정에 없던 보행자(B 출발 때 좁은 랙 뒤에서 나온
  // worker_crossing)가 걸어 들어와도 눈을 감는다 — 포팅 시뮬레이션 B→A + worker_random 22/300 접촉,
  // 전부 커밋 0.5 s 뒤에 나타난 트랙.
  const std::vector<int> carried = prev.exempt_ids;
  auto exempted = [&](int id) {
      return id >= 0 && std::find(carried.begin(), carried.end(), id) != carried.end();
    };
  std::vector<Group> groups;
  for (const GateConflict & c : out.conflicts) {
    if (exempted(active[c.track].id)) {
      continue;
    }
    if (!groups.empty() && cum_s[c.first] - cum_s[groups.back().last] < cfg.group_gap) {
      groups.back().last = std::max(groups.back().last, c.last);
    } else {
      groups.push_back(Group{c.first, c.last, c.reason});
    }
  }

  // --- 커밋 중: 판정한 트랙에 대해서는 출구를 지날 때까지 개입하지 않는다 ---
  if (prev.phase == GatePhase::kCommitted) {
    double s_exit = kInf;
    if (prev.has_exit) {
      s_exit = projectOntoPath(path, prev.exit_point, 0, 0, &cum_s).s;
    }
    if (!prev.has_exit || s_robot > s_exit - cfg.exit_tol) {
      out.phase = GatePhase::kOpen;
      out.reason = GateReason::kClear;
      out.next = GateState{};
      return out;
    }
    if (groups.empty()) {
      out.phase = GatePhase::kCommitted;
      out.reason = prev.forced ? GateReason::kForcedCommit : GateReason::kDecided;
      out.exempt_ids = carried;
      out.s_out = s_exit;
      out.window_s = arrival(s_exit);
      return out;
    }
    // 새 트랙의 위협 — 판정한 트랙의 면제는 유지한 채 아래 정지점 논리로 내려간다
  }
  out.exempt_ids = carried;
  out.next.exempt_ids = carried;

  // --- 위협 없음: 판정 중이었다면 커밋, 아니면 열림 ---
  if (groups.empty()) {
    const bool deciding = prev.phase == GatePhase::kApproach || prev.phase == GatePhase::kHold ||
      prev.phase == GatePhase::kRetreat;
    if (!deciding) {
      out.phase = GatePhase::kOpen;
      out.reason = GateReason::kClear;
      out.next = GateState{};
      return out;
    }
    GateState nx;
    nx.phase = GatePhase::kCommitted;
    nx.forced = false;
    nx.has_exit = prev.has_exit;
    nx.exit_point = prev.exit_point;
    nx.exempt_ids = carried;
    for (const GateTrack & t : tracks) {   // 판정에 들어간 트랙 전부 (정적 판정 포함)
      if (t.id >= 0 && !exempted(t.id)) {
        nx.exempt_ids.push_back(t.id);
      }
    }
    out.phase = GatePhase::kCommitted;
    out.reason = GateReason::kDecided;
    out.exempt_ids = nx.exempt_ids;
    if (prev.has_exit) {
      out.s_out = projectOntoPath(path, prev.exit_point, 0, 0, &cum_s).s;
      out.window_s = arrival(out.s_out);
    }
    out.next = nx;
    return out;
  }

  // --- 첫 노출 구간과 정지점 ---
  const Group & g = groups.front();
  out.s_in = cum_s[g.first];
  out.s_out = cum_s[g.last];
  out.reason = g.reason;
  // 게이트가 맡은(닿을 수 있는) 트랙은 접근·정지 중에도 VO·TTC₀ 에서 뺀다 — 노출 구간 밖으로 서러
  // 가는 로봇에 TTC₀ 조향 보조항이 붙으면 경로를 벗어난다 (폐루프 계약: 접근 중 이탈 0.19 m)
  for (const GateConflict & c : out.conflicts) {
    if (active[c.track].id >= 0 && !exempted(active[c.track].id)) {
      out.exempt_ids.push_back(active[c.track].id);
    }
  }
  out.window_s = arrival(out.s_out);
  out.next.has_exit = true;
  {
    const Pose2D e = interpolateAt(path, cum_s, out.s_out);
    out.next.exit_point = {e.x, e.y};
  }
  const double s_hold_nominal = out.s_in - cfg.hold_back;
  double s_hold = s_hold_nominal;
  // 정지점은 **기하로** 고른다: 방향을 아는 어떤 트랙의 진행선 통로 안(앞 전부 +
  // hold_exclusion_horizon 안에 되돌아올 수 있는 뒤)에도, 방향 모르는 트랙이 그 시간 안에 닿는
  // 자리에도 서지 않는다. 시간이 남는다고(지금 지나가면 로봇이 먼저) 통로 안에 서면 — 노출 구간은
  // 도착 시각에 따라 통로 깊숙이 시작할 수 있다 — 기다리는 동안 보행자가 걸어 들어온다 (포팅
  // 시뮬레이션: B→A 62/610 접촉, 전부 차선 축 위 정지 중). 뒤따라오는 보행자도 예외가 아니다 —
  // 경로가 그 진행선으로 모여드는 굽이에 서면 따라잡힌다 (예외를 두었을 때 A→B 여유 +0.086 m).
  while (s_hold > 0.0) {
    const Pose2D h = interpolateAt(path, cum_s, s_hold);
    bool blocked = false;
    for (const GateTrack & t : active) {
      if (waitingSpotThreatened(t, {h.x, h.y}, cfg)) {
        blocked = true;
        break;
      }
    }
    if (!blocked) {
      break;
    }
    s_hold -= kStep;
  }
  s_hold = std::max(0.0, s_hold);
  const double d_stop = SpeedProfile::stoppingDistance(v_robot, cfg.decel, cfg.jerk, cfg.latency);
  const double s_stop_now = s_robot + d_stop;   // 지금 최대 감속으로 서는 자리
  bool brake_now = false;
  if (v_robot > cfg.stop_v && s_stop_now > s_hold + cfg.hold_tol) {
    // 안전한 정지점에는 설 수 없다 (늦게 발견, 또는 뒤에서 따라오는 보행자에 밀려 노출 구간이
    // 뒤로 자란 경우). 원칙은 **지금 선다** — 서서 물러나는 쪽이 낫다. 강제 커밋(전속 통과)은
    // 지금 서는 자리가 노출 구간 안이고 **몸으로도** 어떤 보행자의 진행선 위이며, 출구까지 남은
    // 길이 commit_max_run 안일 때만이다: 거기 서면 접촉이 확정이고 빠져나가는 편이 짧다. 나란한
    // 구간(5 m)에서는 전속으로 달려도 마주 오는 보행자를 못 벗어난다 — 서서 물러나는 것이 맞다
    // (포팅 시뮬레이션 B→A: 랙 뒤에서 나온 보행자에 강제 커밋 → 나란한 구간 안 정면 접촉).
    // 뒤따르는 보행자의 시간 노출 구간은 정지 자리 뒤까지 자라지만 진행선은 옆(1 m)이라 서는 것이
    // 안전하다 (A→B 위상 0.40: 강제 커밋했더니 내리막 굽이에서 따라잡혀 여유 +0.015 m).
    bool stop_spot_exposed = false;
    if (s_stop_now >= out.s_in && out.s_out - s_stop_now <= cfg.commit_max_run) {
      const Pose2D sp = interpolateAt(path, cum_s, s_stop_now);
      for (const GateTrack & t : active) {
        if (!t.heading_known) {
          continue;
        }
        const double r_body = cfg.robot_radius + t.radius;
        const double dx = sp.x - t.x;
        const double dy = sp.y - t.y;
        const double along = dx * t.ux + dy * t.uy;
        const double lateral = std::abs(-dx * t.uy + dy * t.ux);
        if (lateral <= r_body && along >= -r_body) {
          stop_spot_exposed = true;
          break;
        }
      }
    }
    if (stop_spot_exposed) {
      GateState nx;
      nx.phase = GatePhase::kCommitted;
      nx.forced = true;
      nx.has_exit = true;
      nx.exit_point = out.next.exit_point;
      nx.exempt_ids = carried;   // 새 트랙은 빼지 않는다 — VO/TTC 는 예전대로
      out.phase = GatePhase::kCommitted;
      out.reason = GateReason::kForcedCommit;
      out.exempt_ids = carried;
      out.next = nx;
      return out;
    }
    brake_now = true;
    s_hold = std::max(0.0, std::min(s_hold_nominal, s_stop_now));
  } else if (s_hold < s_robot - cfg.hold_tol) {
    s_hold = std::max(0.0, std::min(s_hold_nominal, s_stop_now));   // 서 있거나 거의 섰다
  }
  out.s_hold = s_hold;
  {
    const Pose2D h = interpolateAt(path, cum_s, s_hold);
    out.next.has_hold = true;
    out.next.hold_point = {h.x, h.y};
  }

  // --- 정지 자리 위협: 어떤 트랙이 hold_threat_horizon 안에 지금 자리에 닿거나, 지금 자리가 방향을
  // 아는 트랙의 앞쪽 진행선 통로 안인가. 고를 때 피했을 자리에 서 있지 않는다 — B 출발 때 좁은 랙
  // 뒤에서 나타난 보행자의 통로가 정지 자리를 덮으면 3 s 안에 닿기 전에 물러난다 (포팅 시뮬레이션
  // B→A + worker_random: 닿을 때까지 기다렸더니 12/300 접촉, 전부 정지 중). 되돌아옴 가지는 여기
  // 쓰지 않는다 — 지나간 보행자 뒤에서까지 물러나면 매 조우마다 후진했다 (610/610 위상) ---
  std::vector<std::size_t> threats_now;
  for (std::size_t k = 0; k < active.size(); ++k) {
    if (canReach(active[k], robot_pt, cfg.hold_threat_horizon, cfg) ||
      inForwardCorridor(active[k], robot_pt, cfg))
    {
      threats_now.push_back(k);
    }
  }
  const bool threatened_now = !threats_now.empty();
  const bool stopped = v_robot <= cfg.stop_v;
  const bool retreating = prev.phase == GatePhase::kRetreat;
  if (stopped && (threatened_now || retreating || s_robot >= s_hold - cfg.hold_tol)) {
    // 정지: 물러날 수 있으면 물러난다 (경로를 따라 뒤로 — 이탈 0)
    // 창 시작(경로 시작)보다 뒤로는 물러나지 않는다 — 첫 계획의 시작점 뒤는 하네스가 이탈로 잰다
    bool retreat = threatened_now && cfg.retreat_enable && s_robot > kStep;
    const double retreated =
      prev.has_retreat_origin ? distance(robot_pt, prev.retreat_origin) : 0.0;
    if (retreat && retreated >= cfg.retreat_max) {
      retreat = false;
    }
    if (retreat) {
      // 남은 후진 예산만큼 경로를 따라 물러난 자리에서 위협 트랙의 진행선으로부터 멀어져야 한다
      // (0.5 m 앞만 보면 축 위에 선 로봇은 축을 건너는 동안 거리가 줄어 물러나지 못한다 — 포팅
      // 시뮬레이션 B→A: 늦게 발견해 차선 축 위에 선 채 접촉). 창 시작보다 뒤는 접선으로 잇는다
      const double budget = std::max(0.0, cfg.retreat_max - retreated);
      Point2D back;
      if (s_robot - budget >= 0.0) {
        const Pose2D b = interpolateAt(path, cum_s, s_robot - budget);
        back = {b.x, b.y};
      } else {
        const Pose2D b0 = interpolateAt(path, cum_s, 0.0);
        const double extra = budget - s_robot;
        back = {b0.x - extra * std::cos(b0.theta), b0.y - extra * std::sin(b0.theta)};
      }
      for (const std::size_t k : threats_now) {
        if (lineDistance(active[k], back) - lineDistance(active[k], robot_pt) < cfg.retreat_gain) {
          retreat = false;   // 물러나도 그 트랙에서 멀어지지 않는다
          break;
        }
      }
    }
    if (retreat) {
      out.phase = GatePhase::kRetreat;
      out.reason = GateReason::kHoldThreatened;
      out.retreat = true;
      out.retreat_speed = cfg.retreat_speed;
      out.hard_stop = false;
      out.next.phase = GatePhase::kRetreat;
      if (!out.next.has_retreat_origin) {
        out.next.has_retreat_origin = true;
        out.next.retreat_origin = robot_pt;
      }
      return out;
    }
    out.phase = GatePhase::kHold;
    out.hard_stop = true;
    out.speed_limit = 0.0;
    if (threatened_now) {
      out.reason = GateReason::kHoldThreatened;
    }
    out.next.phase = GatePhase::kHold;
    return out;
  }

  // --- 접근: 정지점까지의 속도 상한 (안전한 정지점에 못 미치면 지금 최대 감속) ---
  out.phase = GatePhase::kApproach;
  out.speed_limit = brake_now ? 0.0 : SpeedProfile::maxSpeedForStop(
    std::max(0.0, s_hold - s_robot), cfg.decel, cfg.jerk, cfg.latency);
  out.next.phase = GatePhase::kApproach;
  return out;
}

}  // namespace core
}  // namespace amr_navigation
