#include "amr_navigation/core/dwa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "amr_navigation/core/speed_profile.hpp"

namespace amr_navigation
{
namespace core
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

std::vector<double> linspace(double lo, double hi, int n)
{
  std::vector<double> out;
  if (n <= 1 || hi - lo < 1e-9) {
    out.push_back(n <= 1 ? hi : lo);
    if (n > 1 && hi - lo >= 1e-9) {
      out.push_back(hi);
    }
    return out;
  }
  out.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    out.push_back(lo + (hi - lo) * i / (n - 1));
  }
  return out;
}

// 경로의 창 투영: 직전 인덱스에서 앞쪽 창만 본다 (호길이 단조 증가 가정)
Projection windowProjection(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Point2D & p,
  std::size_t & hint)
{
  constexpr std::size_t kWindow = 60;   // 경로점 간격 0.05 m 기준 약 3 m
  const std::size_t end = std::min(path.size(), hint + kWindow);
  Projection pr = projectOntoPath(path, p, hint, end, &cum_s);
  hint = pr.segment;
  return pr;
}
}  // namespace

DwaPlanner::DwaPlanner(const DwaConfig & config)
: config_(config)
{
}

double DwaPlanner::stoppingDistance(double v, double a, double j, double t_c)
{
  return SpeedProfile::stoppingDistance(v, a, j, t_c);
}

double DwaPlanner::simTime(double v) const
{
  const double a = std::max(1e-6, config_.limits.acc_lim_x);
  return std::clamp(std::abs(v) / a + 0.5, config_.sim_time_min, config_.sim_time_max);
}

YieldConfig DwaPlanner::yieldConfig() const
{
  YieldConfig y;
  y.enable = config_.yield_crossing && config_.use_dynamic_obstacles;
  y.robot_radius = config_.robot_radius;
  y.corridor_margin = config_.yield_corridor_margin;
  y.stop_margin = config_.yield_stop_margin;
  y.clear_margin = config_.yield_clear_margin;
  y.horizon = config_.yield_horizon;
  y.nominal_speed = config_.yield_nominal_speed;
  y.hold_standoff = config_.yield_hold_standoff;
  y.approach_radius = config_.yield_approach_radius;
  y.approach_horizon = config_.yield_approach_horizon;
  y.lookahead = config_.yield_lookahead;
  y.max_zone = config_.yield_max_zone;
  y.min_speed = config_.dynamic_speed_threshold;
  y.accel = config_.limits.acc_lim_x;
  y.decel = config_.limits.decel_lim_x;
  y.jerk = config_.limits.jerk_lim_x;
  y.latency = config_.approach_latency;
  y.v_max = config_.limits.max_vel_x;
  y.decision_enable = config_.yield_decision;
  y.body_margin = config_.yield_body_margin;
  y.go_margin = config_.yield_go_margin;
  y.go_clearance = config_.yield_go_clearance;
  y.go_hold_clearance = config_.yield_go_hold_clearance;
  y.go_speed_floor = config_.yield_go_speed_floor;
  y.hold_slack = config_.yield_hold_slack;
  y.decision_min_sin = config_.yield_decision_min_sin;
  return y;
}

std::pair<double, double> DwaPlanner::windowCenter(const DwaInput & in) const
{
  if (in.has_last && std::abs(in.v_last - in.v_meas) <= config_.window_reset_v &&
    std::abs(in.w_last - in.w_meas) <= config_.window_reset_w)
  {
    return {in.v_last, in.w_last};
  }
  return {in.v_meas, in.w_meas};
}

DwaWindow DwaPlanner::dynamicWindow(double v_c, double w_c, double v_cap) const
{
  const auto & L = config_.limits;
  const double dt = config_.control_period;
  DwaWindow win;
  win.v_cap = std::min(L.max_vel_x, v_cap);
  win.v_lo = std::max(L.min_vel_x, v_c - L.decel_lim_x * dt);
  win.v_hi = std::min(win.v_cap, v_c + L.acc_lim_x * dt);
  if (v_c + L.acc_lim_x * dt < L.min_vel_x) {
    // 허용 범위 한참 아래(후진 복구 직후 등): 한 주기에 닿을 수 있는 만큼만 범위 쪽으로
    win.v_lo = win.v_hi = v_c + L.acc_lim_x * dt;
  }
  // (리뷰 후 수정) 이전에는 v_c < min_vel_x 이면 상한을 min_vel_x 로 막았다 — 정지 뒤 측정 속도가
  // −1e-4 처럼 조금만 음수여도 창이 [v_c, 0] 으로 붙어 0 만 고르고, 그 명령이 다시 창 중심이 되어
  // 영영 출발하지 못했다 (Gazebo 좁은 통로 왕복 lap 8: 18 s 정지 → Failed to make progress).
  // 시험: DwaWindowRecoversFromTinyNegativeSpeed
  if (win.v_hi < win.v_lo) {
    win.v_hi = win.v_lo;   // 상한이 창 아래로 내려감(속도 제한 급감) → 최대 감속만
  }
  win.w_lo = std::max(-L.max_vel_theta, w_c - L.acc_lim_theta * dt);
  win.w_hi = std::min(L.max_vel_theta, w_c + L.acc_lim_theta * dt);
  if (win.w_hi < win.w_lo) {
    const double w = std::clamp(w_c, -L.max_vel_theta, L.max_vel_theta);
    win.w_lo = w;
    win.w_hi = w;
  }
  return win;
}

std::vector<std::pair<double, double>> DwaPlanner::sampleVelocities(
  const DwaWindow & win, double v_c, double w_c) const
{
  // R2: 창을 "표본 필터" 가 아니라 "방출 명령 클램프" 로 재정의한다.
  // 표본은 **지속 목표 명령**이고 롤아웃(rolloutRamp)이 가속·저크로 거기까지 램프한다.
  // 방출은 t = control_period 의 램프 상태라 여전히 창 안이다 — 하류 계약 불변.
  // 창 격자로 뽑으면 v_meas=0 에서 후보가 [0, 0.05] 뿐이고 342 개가 물리적으로 같아진다.
  const double lo = config_.sustained_sampling ? config_.limits.min_vel_x : win.v_lo;
  const double hi = config_.sustained_sampling ? win.v_cap : win.v_hi;
  const double wlo = config_.sustained_sampling ? -config_.limits.max_vel_theta : win.w_lo;
  const double whi = config_.sustained_sampling ? config_.limits.max_vel_theta : win.w_hi;
  std::vector<double> vs = linspace(lo, hi, std::max(1, config_.vx_samples));
  std::vector<double> ws = linspace(wlo, whi, std::max(1, config_.vth_samples));
  // 직진(ω = 0) 열을 정확히 포함
  if (wlo <= 0.0 && whi >= 0.0 &&
    std::none_of(ws.begin(), ws.end(), [](double w) {return std::abs(w) < 1e-9;}))
  {
    ws.push_back(0.0);
  }
  std::vector<std::pair<double, double>> out;
  out.reserve(vs.size() * ws.size() + 1);
  for (double v : vs) {
    for (double w : ws) {
      out.emplace_back(v, w);
    }
  }
  // 제동 후보: 최대 감속, ω 를 0 쪽으로 (항상 창 안)
  const auto & L = config_.limits;
  const double dt = config_.control_period;
  double vb = v_c > 0.0 ? std::max(0.0, v_c - L.decel_lim_x * dt) :
    std::min(0.0, v_c + L.decel_lim_x * dt);
  vb = std::clamp(vb, lo, hi);
  double wb = w_c - (w_c > 0.0 ? 1.0 : -1.0) * std::min(std::abs(w_c), L.acc_lim_theta * dt);
  wb = std::clamp(wb, wlo, whi);
  out.emplace_back(vb, wb);
  return out;
}

std::vector<Pose2D> DwaPlanner::rollout(const Pose2D & start, double v, double w, double T) const
{
  const double dt = config_.sim_dt;
  const int n = std::max(1, static_cast<int>(std::ceil(T / dt - 1e-9)));
  std::vector<Pose2D> poses;
  poses.reserve(static_cast<std::size_t>(n) + 1);
  poses.push_back(start);
  // 매 스텝을 시작 자세에서 정확 적분 (오차 누적 없음)
  for (int k = 1; k <= n; ++k) {
    poses.push_back(integrateArc(start, v, w, k * dt));
  }
  return poses;
}

void DwaPlanner::rampStep(
  double * v, double * a, double target, double a_max, double j_max, double dt)
{
  const double j = std::max(j_max, 1e-9);
  const double dv = target - *v;
  // 목표에 도달: 속도는 붙잡고 가속도만 저크 안에서 0 으로 내린다.
  // (예전에는 여기서 a 를 0 으로 **급변**시켜 저크 한계를 깼다
  //  — 계약 RampStepRespectsAccelAndJerk 가 잡았다.)
  if (std::abs(dv) < 1e-9) {
    *a += std::clamp(-*a, -j * dt, j * dt);
    *v = target;
    return;
  }
  // 지금 가속도를 저크로 0 까지 줄이는 동안 더 벌어지는 속도 — 이만큼 남았으면 가속을 접는다
  const double a_settle = 0.5 * (*a) * std::abs(*a) / j;
  double a_des = (dv - a_settle) > 0.0 ? a_max : -a_max;
  // 이 스텝에 목표에 정확히 닿을 수 있으면 그 가속도를 쓴다 (한계 안일 때만)
  const double a_land = dv / dt;
  if (std::abs(a_land) <= a_max && std::abs(a_land - *a) <= j * dt) {
    a_des = a_land;
  }
  double a_new = *a + std::clamp(a_des - *a, -j * dt, j * dt);
  a_new = std::clamp(a_new, -a_max, a_max);
  double v_new = *v + a_new * dt;
  // 저크 한계 안에서 막을 수 없는 오버슛이면 속도만 목표에 붙인다. 가속도는 연속으로 두고
  // 다음 스텝의 위 분기가 0 으로 내린다 — 저크 한계를 깨지 않는 유일한 방법이다.
  if ((dv > 0.0 && v_new > target) || (dv < 0.0 && v_new < target)) {
    v_new = target;
  }
  *a = a_new;
  *v = v_new;
}

std::vector<Pose2D> DwaPlanner::rolloutRamp(
  const Pose2D & start, double v0, double w0, double v_t, double w_t, double T,
  std::vector<double> * arc, std::vector<double> * speeds) const
{
  const double dt = config_.sim_dt;
  const int n = std::max(1, static_cast<int>(std::ceil(T / dt - 1e-9)));
  std::vector<Pose2D> poses;
  poses.reserve(static_cast<std::size_t>(n) + 1);
  poses.push_back(start);
  if (arc != nullptr) {
    arc->assign(1, 0.0);
    arc->reserve(static_cast<std::size_t>(n) + 1);
  }
  if (speeds != nullptr) {
    speeds->assign(1, v0);
    speeds->reserve(static_cast<std::size_t>(n) + 1);
  }
  const auto & L = config_.limits;
  Pose2D p = start;
  double v = v0, av = 0.0, w = w0, aw = 0.0, s_cum = 0.0;
  for (int k = 1; k <= n; ++k) {
    rampStep(
      &v, &av, v_t, v_t > v ? L.acc_lim_x : L.decel_lim_x,
      std::max(1e-6, config_.limits.jerk_lim_x), dt);
    rampStep(&w, &aw, w_t, L.acc_lim_theta, std::max(1e-6, config_.jerk_lim_theta), dt);
    p = integrateArc(p, v, w, dt);
    s_cum += std::abs(v) * dt;
    poses.push_back(p);
    if (arc != nullptr) {
      arc->push_back(s_cum);
    }
    if (speeds != nullptr) {
      speeds->push_back(v);
    }
  }
  return poses;
}

std::vector<Pose2D> DwaPlanner::recenterPath(
  const std::vector<Pose2D> & path, const PointCostFn & point_cost, bool end_is_goal,
  std::size_t * shifted) const
{
  const std::size_t n = path.size();
  std::vector<double> off(n, 0.0);
  std::size_t count = 0;
  if (n < 3 || config_.recenter_max_shift <= 0.0 || config_.recenter_step <= 0.0) {
    if (shifted != nullptr) {
      *shifted = 0;
    }
    return path;
  }
  const std::vector<double> cum = cumulativeLength(path);
  const int K =
    std::max(
    1,
    static_cast<int>(std::floor(config_.recenter_max_shift / config_.recenter_step + 1e-9)));
  std::vector<double> costs(static_cast<std::size_t>(2 * K + 1));
  for (std::size_t i = 0; i < n; ++i) {
    if (end_is_goal && cum.back() - cum[i] < config_.recenter_goal_keep) {
      continue;
    }
    const Pose2D & p = path[i];
    if (point_cost(p.x, p.y) < config_.recenter_min_cost) {
      continue;
    }
    // 법선은 이웃 점 방향으로 (경로 자세 θ 가 없거나 거친 경우 대비)
    const Pose2D & a = path[i > 0 ? i - 1 : i];
    const Pose2D & b = path[i + 1 < n ? i + 1 : i];
    const double th = std::atan2(b.y - a.y, b.x - a.x);
    const double nx = -std::sin(th);
    const double ny = std::cos(th);
    double best = std::numeric_limits<double>::infinity();
    for (int k = -K; k <= K; ++k) {
      const double o = k * config_.recenter_step;
      const double c = point_cost(p.x + o * nx, p.y + o * ny);
      costs[static_cast<std::size_t>(k + K)] = c;
      best = std::min(best, c);
    }
    // 골: 양쪽 끝이 바닥보다 높다 (한쪽만 막힌 곳이면 끝이 바닥 → 옮기지 않는다)
    if (!(costs.front() > best && costs.back() > best)) {
      continue;
    }
    // 바닥이 평평하면 그 구간의 가운데
    double sum = 0.0;
    int cnt = 0;
    for (int k = -K; k <= K; ++k) {
      if (costs[static_cast<std::size_t>(k + K)] <= best) {
        sum += k;
        ++cnt;
      }
    }
    off[i] = sum / cnt * config_.recenter_step;
    ++count;
  }
  if (shifted != nullptr) {
    *shifted = count;
  }
  if (count == 0) {
    return path;
  }
  // 이동평균 (옮기지 않은 점은 0 으로 참여 → 좁은 곳 입구에서 완만히 들어간다)
  const int w = std::max(0, config_.recenter_smooth);
  std::vector<Pose2D> out(path);
  for (std::size_t i = 0; i < n; ++i) {
    double sum = 0.0;
    int cnt = 0;
    for (int d = -w; d <= w; ++d) {
      const std::ptrdiff_t j = static_cast<std::ptrdiff_t>(i) + d;
      if (j < 0 || j >= static_cast<std::ptrdiff_t>(n)) {
        continue;
      }
      sum += off[static_cast<std::size_t>(j)];
      ++cnt;
    }
    const double o = std::clamp(
      sum / std::max(
        1,
        cnt), -config_.recenter_max_shift,
      config_.recenter_max_shift);
    if (o == 0.0) {
      continue;
    }
    const Pose2D & a = path[i > 0 ? i - 1 : i];
    const Pose2D & b = path[i + 1 < n ? i + 1 : i];
    const double th = std::atan2(b.y - a.y, b.x - a.x);
    out[i].x += -std::sin(th) * o;
    out[i].y += std::cos(th) * o;
  }
  return out;
}

DwaResult DwaPlanner::compute(
  const DwaInput & in, const FootprintCostFn & footprint_cost,
  const PointCostFn & point_cost) const
{
  DwaResult res;
  const auto & L = config_.limits;
  const auto & W = config_.weights;

  // --- 경로 기준량 (0 단계: 좁은 곳 재중심) ---
  static const std::vector<Pose2D> kEmpty;
  const std::vector<Pose2D> & path_in = in.path != nullptr ? *in.path : kEmpty;
  const std::vector<Pose2D> path = config_.recenter_narrow ?
    recenterPath(path_in, point_cost, in.path_end_is_goal, &res.n_recentered) : path_in;
  const std::vector<double> cum_s = cumulativeLength(path);
  const Point2D robot_pt{in.pose.x, in.pose.y};
  Projection robot_proj;
  double d_goal = kInf;
  double goal_yaw = in.pose.theta;
  if (!path.empty()) {
    robot_proj = projectOntoPath(path, robot_pt, 0, 0, &cum_s);
    if (in.path_end_is_goal) {
      // 남은 호길이와 목표까지 직선거리 중 큰 값 (경로 끝 옆으로 비껴 있는 경우 보정)
      d_goal = std::max(
        std::max(0.0, cum_s.back() - robot_proj.s),
        distance(robot_pt, Point2D{path.back().x, path.back().y}));
    }
    goal_yaw = path.back().theta;
  }
  res.d_goal = d_goal;
  const bool aligning = std::isfinite(d_goal) && d_goal <= config_.goal_align_distance;

  // 동적 장애물 선별 (VO·TTC·양보 공용)
  std::vector<DynamicObstacle> dyn;
  if (config_.use_dynamic_obstacles) {
    for (const auto & o : in.obstacles) {
      if (o.speed() >= config_.dynamic_speed_threshold) {
        dyn.push_back(o);
      }
    }
  }

  // --- 1) 동적 창 (횡단 양보 정지선이 외부 속도 제한과 같은 자리에 들어간다) ---
  const auto [v_c, w_c] = windowCenter(in);
  // 횡단 게이트(crossing_gate.hpp)가 켜지면 통로 정지선·결정층(evaluateYield)을 **대신한다**.
  // 노출 구간 밖 정지점에서 진입 전에 판정하고, 커밋 뒤에는 어떤 양보·hold·retreat·escape 도
  // 개입하지 않는다 — 진입 뒤 정지가 곧 접촉이라는 것이 08 의 1,500 시행이 확정한 물리다.
  const bool gate_on = config_.gate.enable;
  if (gate_on) {
    GateConfig gc = config_.gate;
    gc.robot_radius = config_.robot_radius;
    gc.accel = L.acc_lim_x;
    gc.decel = L.decel_lim_x;
    gc.jerk = L.jerk_lim_x;
    gc.latency = config_.approach_latency;
    gc.robot_v_max = L.max_vel_x;
    res.gate = evaluateGate(
      path, cum_s, in.pose, robot_proj.s, in.v_meas, in.gate_tracks, gc, in.gate_state);
    // 기록용 투영: dwa/stats 의 yield 열이 뜻을 잃지 않게 (정지점 = 정지선, 커밋 = 통로 안).
    // 아래 escaping 은 gate_on 이면 켜지지 않으므로 이 투영이 거동을 바꾸지는 않는다.
    switch (res.gate.phase) {
      case GatePhase::kApproach:
      case GatePhase::kHold:
      case GatePhase::kRetreat:
        res.yield.state = YieldState::kYield;
        break;
      case GatePhase::kCommitted:
        res.yield.state = YieldState::kCommitted;
        break;
      default:
        res.yield.state = YieldState::kClear;
        break;
    }
    res.yield.speed_limit = res.gate.speed_limit;
    res.yield.stop_distance = std::isfinite(res.gate.s_hold) ?
      std::max(0.0, res.gate.s_hold - robot_proj.s) : kInf;
    res.yield.zone_entry = std::isfinite(res.gate.s_in) ? res.gate.s_in - robot_proj.s : kInf;
    res.yield.zone_exit = std::isfinite(res.gate.s_out) ? res.gate.s_out - robot_proj.s : kInf;
  } else {
    res.yield = evaluateYield(
      path, cum_s, in.pose, robot_proj.s, std::max(0.0, in.v_meas), dyn, yieldConfig(),
      static_cast<YieldDecision>(in.yield_decision_last));
  }
  const YieldDecision dec = res.yield.decision;
  // 양보 상한은 로봇 가속 한계보다 빨리 오를 수 없다 (통로 예측 흔들림이 브레이크를 놓지 못하게).
  // 다만 go / retreat 결정은 의도적인 해제다 — 무너진 정지선의 잔여 상한(08 j8c t6: committed 직후
  // 0.6 m/s)을 이어받으면 지나가거나 비켜설 속도부터 잃는다. hold 는 유한 상한이라 그대로 묶인다.
  // 게이트는 접근·정지 중(유한 상한)에만 묶고, 커밋·열림(∞)은 그대로 둔다 — 커밋 뒤 불간섭.
  const bool limit_rise = gate_on ?
    (res.gate.phase == GatePhase::kApproach || res.gate.phase == GatePhase::kHold ||
    res.gate.phase == GatePhase::kRetreat) :
    (dec != YieldDecision::kGo && dec != YieldDecision::kRetreat);
  if (std::isfinite(in.yield_limit_last) && limit_rise) {
    res.yield.speed_limit = std::min(
      res.yield.speed_limit, in.yield_limit_last + L.acc_lim_x * config_.control_period);
  }
  const double v_cap = std::min(std::min(L.max_vel_x, in.speed_limit), res.yield.speed_limit);
  res.v_cap = v_cap;
  res.window = dynamicWindow(v_c, w_c, v_cap);
  // 게이트가 커밋 시점에 판정한 트랙은 VO·TTC 에서도 뺀다 — 게이트가 책임진다. 판정 밖 트랙
  // (커밋 뒤에 나타난 것, 강제 커밋)은 예전대로 VO·TTC 가 맡는다.
  auto gate_exempt = [&](const DynamicObstacle & o) {
      return gate_on && o.id >= 0 &&
             std::find(res.gate.exempt_ids.begin(), res.gate.exempt_ids.end(), o.id) !=
             res.gate.exempt_ids.end();
    };
  // 이미 통로 안(kCommitted): 앞은 VO 가 막으므로 뒤로 빠질 수 있게 창 아래쪽을 연다.
  // 한 주기에 닿을 수 있는 범위(감속 한계) 안에서만 내린다 — 명령이 튀지 않는다.
  // 횡단일 때만 의미가 있다: 정면 접근(장애물 진행 방향이 로봇 헤딩과 나란)은 통로 축이 우리
  // 진행선과 같아 "옆으로 빠진다" 가 성립하지 않는다 — 그 상황은 VO/TTC 의 감속이 맡는다.
  // 이미 서 있거나 뒤로 빠지는 중일 때만 연다. 정상 주행 중에는 건드리지 않는다 — 연속 운용(14)
  // 실측: 주행 중에도 열어 두었더니 후진↔전진이 되풀이되어 "Failed to make progress" 58 건과
  // 작업 시간 초과 1 건이 났다 (같은 구성의 앞 실행은 0 건).
  // 비켜서야 하는가: 장애물이 지나갈 원통(반경 합) 안에 로봇이 있으면, 그대로 있으면 반드시
  // 스친다. 08 실측: 접촉 51/52 가 β < 0.611 m (로봇 0.361 + 사람 0.25) 였고, 접촉 직전 0.8 s
  // 동안 로봇은 제자리 회전만 했다 (v 0.000, w 0.5~0.74) — 차동 구동은 회전으로 위치가 변하지
  // 않으므로 β 가 그대로다. 예전에는 "이미 멈춤(v_meas < 0.05)" 일 때만 탈출을 켰는데, 그때는
  // 동적 창이 한 주기에 −0.05 m/s 밖에 못 내 계획 지평 안에서 만들 수 있는 가로 이동이 사실상
  // 0 이다. 즉 비켜야 할 때는 이미 비킬 수 없었다. 속도가 남아 있을 때 켠다.
  bool leave_lane_diag = false;   // R0 계측용 사본 (선택 로직은 블록 안의 leave_lane 을 쓴다)
  bool escaping = !gate_on && res.yield.state == YieldState::kCommitted &&
    config_.yield_escape_speed > 0.0 && res.yield.obstacle >= 0 &&
    static_cast<std::size_t>(res.yield.obstacle) < dyn.size();
  // "지금 누군가의 몸 앞에 서 있는가" 는 **모든 동적 트랙**을 봐야 한다. 예전에는 양보 장애물
  // 하나의 축으로만 쟀는데, 근거리에서 사람 하나가 두 트랙으로 갈라지면(다리 분리) 양보 선택
  // 규칙(통로 출구가 가장 먼 트랙, crossing_yield.cpp)이 체계적으로 비낀 쪽을 고른다. 그러면
  // 로봇 β 가 비낀 축 기준으로 계산돼 실제보다 커지고, 원통 안에서 켜져야 할 두 규칙(가속
  // 허용·제자리 회전 배제)이 하필 마지막 순간에 꺼진다.
  // 08 실측: 접촉 4건 모두 0.4~1.0 s 전에 진짜 축에서 0.35~0.39 m 비낀 중복 트랙이 생겼다.
  // j8c 시행 6 은 주기 단위로 확인된다 — 142.108~142.199 진짜 트랙(β≈0.30)일 때 cmd_v 0.057
  // (배제 규칙이 살아 0.05 이상 유지), 142.244 중복 트랙으로 바뀌자 0.007 → 142.296 부터
  // 0.000, 접촉 0.25 s 뒤 중복 트랙 축이 붙자 다시 0.050.
  bool in_body = false;
  if (escaping) {
    for (const DynamicObstacle & o : dyn) {
      const double su = std::max(1e-6, o.speed());
      const double b = std::abs(
        -(in.pose.x - o.x) * (o.vy / su) + (in.pose.y - o.y) * (o.vx / su));
      if (b < config_.robot_radius + o.radius) {
        in_body = true;
        break;
      }
    }
    const DynamicObstacle & o0 = dyn[static_cast<std::size_t>(res.yield.obstacle)];
    const double su0 = std::max(1e-6, o0.speed());
    const double b0 = std::abs(
      -(in.pose.x - o0.x) * (o0.vy / su0) + (in.pose.y - o0.y) * (o0.vx / su0));
    // 진입 시점의 결정(crossing_yield.hpp YieldDecision)이 있으면 그것이 우선한다.
    //   go      — 감속 없이 지나간다: 비켜서기 이득도 후진 창도 없다 (아래 TTC₀ 벌점도 끈다).
    //   hold / retreat — 몸 원통 **밖**에서도 바깥으로 조향해야 하므로 비켜서기를 켠다. 예전에는
    //             원통 + 5 cm 안이거나 서 있을 때만 켰다 — 그래서 R_c 통로와 몸 원통 사이(08 기하:
    //             경로로 1.2~1.5 m)에서는 순수 경로 추종 + VO/TTC 감속뿐이었고, 로봇은 그 띠를
    //             지나 원통 안까지 들어간 뒤에야 비켜서기를 시작했다 (접촉 4건 공통).
    if (dec == YieldDecision::kGo) {
      escaping = false;
    } else if (dec == YieldDecision::kHold || dec == YieldDecision::kRetreat) {
      escaping = true;
    } else {
      escaping = b0 < config_.robot_radius + o0.radius + config_.yield_vacate_margin ||
        in.v_meas < config_.yield_escape_v_meas;
    }
  }
  if (escaping) {
    res.window.v_lo = std::max(
      -config_.yield_escape_speed, v_c - L.decel_lim_x * config_.control_period);
    res.window.v_lo = std::min(res.window.v_lo, res.window.v_hi);
    // 비켜서되 **가속하지는 않는다**: 마주 오는 장애물을 향해 빨라지는 것은 위험을 키운다.
    // 조향으로 원통에서 벗어나는 것이 목적이지 먼저 지나가려는 것이 아니다.
    // 다만 **몸 원통 안**(β < 반경 합)에서는 다르다. 거기서는 그대로 있는 것이 곧 접촉이고,
    // 저속에서는 계획 지평 안에 만들 수 있는 가로 이동이 사실상 0 이라 조향할 수단조차 없다 —
    // 08 실측 g8b/g8c 접촉 2건이 모두 원통 안에서 0.11 / 0.23 m/s 로 기어가던 중이었고, 둘 다
    // 작업자가 정확히 180° 정면이라 후보 간 β 차이가 없어 탈출 순위도 발동하지 못했다.
    // 원통 안에서는 창을 열어 조향 권한을 되찾게 한다. 들이받는 후보는 VO/TTC 가 그대로 막는다.
    if (!in_body) {
      res.window.v_hi = std::min(res.window.v_hi, std::max(in.v_meas, 0.0));
    }
    res.window.v_hi = std::max(res.window.v_hi, res.window.v_lo);
  }

  // 게이트 정지·후진: 샘플링을 거치지 않는다 — 정지 중에는 v = 0, w = 0 이 계약이고(VO 포화·탈출
  // 이득이 선 로봇을 옆으로 밀지 못하게), 후진은 경로를 따라 곧게(이탈 0) 롤아웃이 비어 있을 때만.
  if (gate_on && (res.gate.hard_stop || res.gate.retreat)) {
    double v_out = 0.0;
    if (res.gate.retreat) {
      const std::vector<Pose2D> back = rollout(in.pose, -res.gate.retreat_speed, 0.0, 1.0);
      bool blocked = false;
      for (std::size_t k = 1; k < back.size(); ++k) {
        if (footprint_cost(back[k]) < 0.0) {
          blocked = true;
          break;
        }
      }
      if (blocked) {
        res.gate.retreat = false;
        res.gate.hard_stop = true;
        res.gate.reason = GateReason::kHoldThreatened;
        res.gate.phase = GatePhase::kHold;
        res.gate.next.phase = GatePhase::kHold;
      } else {
        v_out = -res.gate.retreat_speed;
      }
    }
    res.found = true;
    res.v = v_out;
    res.w = 0.0;
    res.best = DwaCandidate{};
    res.best.v = v_out;
    res.best.is_brake = true;
    res.best.poses = {in.pose};
    return res;
  }

  // 목표 속도: 접근 감속 + 현재 헤딩 오차가 크면 감속 (경로가 뒤에 있으면 제자리 회전 유도)
  double v_des = v_cap;
  if (std::isfinite(d_goal)) {
    // 목표 접근: 저크·지연을 넣은 정지거리의 역함수
    // (사다리꼴 √(2ad) 는 하류 저크 필터 때문에 지나친다)
    v_des = std::min(
      v_des,
      SpeedProfile::maxSpeedForStop(d_goal, L.decel_lim_x, L.jerk_lim_x, config_.approach_latency));
  }
  if (!path.empty()) {
    const double ell0 = config_.heading_lookahead_min;
    const Pose2D tgt0 = interpolateAt(path, cum_s, robot_proj.s + ell0);
    const double a0 = wrapAngle(std::atan2(tgt0.y - in.pose.y, tgt0.x - in.pose.x) - in.pose.theta);
    if (distance(Point2D{tgt0.x, tgt0.y}, robot_pt) > 1e-3) {
      v_des *= std::max(0.0, std::cos(a0));
    }
  }
  if (aligning) {
    v_des = 0.0;
  }

  // --- 2) 샘플링 ---
  const auto samples = sampleVelocities(res.window, v_c, w_c);
  res.n_samples = samples.size();

  const double v_span = std::max(1e-6, L.max_vel_x - L.min_vel_x);
  // 명령 → 감속 시작 지연: 명령 유지 T_c + 저크 램프의 평균 지연 a/(2j)
  const double t_lag = config_.commit_time +
    (L.jerk_lim_x > 0.0 ? L.decel_lim_x / (2.0 * L.jerk_lim_x) : 0.0);
  std::vector<DwaCandidate> cands;
  cands.reserve(samples.size());
  for (std::size_t si = 0; si < samples.size(); ++si) {
    DwaCandidate c;
    c.v = samples[si].first;
    c.w = samples[si].second;
    c.is_brake = (si + 1 == samples.size());
    const double T = simTime(c.v);
    if (config_.sustained_sampling) {
      // R1: (c.v, c.w) 는 **목표 명령**이고 롤아웃이 가속·저크로 거기까지 램프한다.
      // 호길이는 |c.v|·k·dt 가 아니라 실제 프로파일의 적분이다 (등속 가정이 깨졌으므로).
      std::vector<double> speeds;
      c.poses = rolloutRamp(in.pose, in.v_meas, in.w_meas, c.v, c.w, T, &c.arc, &speeds);
      c.v_peak = 0.0;
      for (double sp : speeds) {
        c.v_peak = std::max(c.v_peak, std::abs(sp));
      }
      // 방출값 = 한 제어 주기 뒤의 램프 상태. 정의상 1주기 도달집합 안이다.
      double ve = in.v_meas, ae = 0.0, we = in.w_meas, aw = 0.0;
      rampStep(
        &ve, &ae, c.v, c.v > in.v_meas ? L.acc_lim_x : L.decel_lim_x,
        std::max(1e-6, L.jerk_lim_x), config_.control_period);
      rampStep(
        &we, &aw, c.w, L.acc_lim_theta, std::max(1e-6, config_.jerk_lim_theta),
        config_.control_period);
      c.v_emit = std::clamp(ve, res.window.v_lo, res.window.v_hi);
      c.w_emit = std::clamp(we, res.window.w_lo, res.window.w_hi);
    } else {
      c.poses = rollout(in.pose, c.v, c.w, T);
      c.v_emit = c.v;
      c.w_emit = c.w;
      c.v_peak = std::abs(c.v);
      c.arc.resize(c.poses.size());
      for (std::size_t k = 0; k < c.poses.size(); ++k) {
        c.arc[k] = std::abs(c.v) * static_cast<double>(k) * config_.sim_dt;
      }
    }

    // --- 4) 충돌 검사 (목표 너머 제외) ---
    std::size_t n_check = c.poses.size();
    for (std::size_t k = 1; k < c.poses.size(); ++k) {
      const double s_k = c.arc[k];
      if (std::isfinite(d_goal) && s_k > d_goal + config_.goal_overshoot_margin) {
        n_check = k;
        break;
      }
      if (footprint_cost(c.poses[k]) < 0.0) {
        c.collision = true;
        break;
      }
    }
    if (c.collision) {
      ++res.n_collision;
      cands.push_back(std::move(c));
      continue;
    }
    c.poses.resize(n_check);

    // --- 5) VO 원뿔 판정 (진입 시각도 기록: 포화 시 선택 기준) ---
    if (config_.use_velocity_obstacles && !dyn.empty()) {
      // R1: 램프에서는 등속 현 근사가 틀린다 — 게다가 chordVelocity 는 v=0 에서 ω 와 무관하게
      // 정확히 (0,0) 이라 회전 후보가 VO 에 전혀 보이지 않았다 (연구 브리프 §3.2).
      // 실제 롤아웃의 시작→τ 변위를 τ 로 나눈 것이 정의상 그 구간의 유효 속도다.
      Point2D v_eff = chordVelocity(in.pose.theta, c.v, c.w, config_.vo_time_horizon);
      if (config_.sustained_sampling) {
        const double tau = std::max(1e-6, config_.vo_time_horizon);
        const std::size_t kt = std::min(
          c.poses.size() - 1,
          static_cast<std::size_t>(std::llround(tau / config_.sim_dt)));
        const double used = static_cast<double>(kt) * config_.sim_dt;
        if (used > 1e-9) {
          v_eff = Point2D{(c.poses[kt].x - in.pose.x) / used,
            (c.poses[kt].y - in.pose.y) / used};
        }
      }
      for (const auto & o : dyn) {
        if (gate_exempt(o)) {
          continue;
        }
        const Point2D p{o.x - in.pose.x, o.y - in.pose.y};
        if (std::hypot(p.x, p.y) > config_.vo_max_range) {
          continue;
        }
        const double R = config_.robot_radius + o.radius + config_.vo_margin;
        const Point2D w_rel{v_eff.x - o.vx, v_eff.y - o.vy};
        c.vo_time = std::min(c.vo_time, velocityObstacleTime(p, w_rel, R, config_.vo_time_horizon));
      }
      c.vo_rejected = std::isfinite(c.vo_time);
    }

    // --- 6) 비용 ---
    // 추종 항(헤딩·경로)은 앞쪽 path_eval_time 만 본다 (충돌·여유는 정지 지평 전체):
    // 긴 등곡률 원호가 곡률이 바뀌는 경로(S 자)의 평균 곡률을 골라 안쪽을 가로지르는 것을
    // 막는다 (dwa.md §1.5).
    std::size_t k_eval = c.poses.size() - 1;
    if (config_.path_eval_time > 0.0) {
      k_eval = std::min(
        k_eval,
        static_cast<std::size_t>(std::ceil(config_.path_eval_time / config_.sim_dt - 1e-9)));
    }
    const Pose2D & end = c.poses[k_eval];
    // 헤딩
    if (!path.empty()) {
      if (aligning) {
        c.terms.heading = std::abs(wrapAngle(goal_yaw - end.theta)) / kPi;
      } else {
        std::size_t hint = robot_proj.segment;
        const Projection pe = windowProjection(path, cum_s, {end.x, end.y}, hint);
        const double ell = std::clamp(
          config_.heading_lookahead_gain * std::abs(c.v) + config_.heading_lookahead_offset,
          config_.heading_lookahead_min, config_.heading_lookahead_max);
        const Pose2D tgt = interpolateAt(path, cum_s, pe.s + ell);
        const double dx = tgt.x - end.x;
        const double dy = tgt.y - end.y;
        if (std::hypot(dx, dy) > 1e-3) {
          c.terms.heading = std::abs(wrapAngle(std::atan2(dy, dx) - end.theta)) / kPi;
        } else {
          c.terms.heading = std::abs(wrapAngle(goal_yaw - end.theta)) / kPi;
        }
      }
    }
    // 여유(기준 경로 대비 초과 비용)와 경로 이탈
    double clear = 0.0;
    double path_sum = 0.0;
    double cte_max = 0.0;
    std::size_t hint = robot_proj.segment;
    for (std::size_t k = 1; k < c.poses.size(); ++k) {
      const Pose2D & p = c.poses[k];
      const double cp = point_cost(p.x, p.y);
      double cg = 0.0;
      if (!path.empty()) {
        const double s_ref = robot_proj.s + c.arc[k];
        const Pose2D g = interpolateAt(path, cum_s, s_ref);
        cg = point_cost(g.x, g.y);
        if (k <= k_eval) {
          const Projection pk = windowProjection(path, cum_s, {p.x, p.y}, hint);
          path_sum += std::min(std::abs(pk.cte) / config_.path_band, 1.0);
          cte_max = std::max(cte_max, std::abs(pk.cte));
        }
      }
      clear = std::max(clear, std::max(0.0, cp - cg));
    }
    const std::size_t nk = std::max<std::size_t>(1, k_eval);
    c.terms.clearance = std::min(1.0, clear / 252.0);
    c.terms.path = path.empty() ? 0.0 : path_sum / static_cast<double>(nk);
    c.max_cte = cte_max;
    // 이탈 한계 초과 벌점: 한계는 max(d_off, 지금 로봇의 이탈) — 이미 밖이면 그 거리까지는 벌하지
    // 않아 복귀 후보가 살아남고, 한계 밖으로 더 나가는 후보만 강하게 벌한다 (단조 감소 포락선).
    // 그 래칫에는 반드시 상한이 있어야 한다. 한계가 로봇을 끝없이 따라 올라가면 이탈을 막는
    // 유일한 항이 영영 켜지지 않아, 남은 탈출 이득(w_e/R_c = 1.91 /m)이 통로 반폭까지 로봇을
    // 밀어낸다 — 08 실측 e8b 의 이탈 1.39 m 가 반폭 1.31 m 와 같은 값이다. 상한에 닿으면 벌점
    // 기울기(w_f/off_path_band = 3.0 /m)가 이득(1.91 /m)을 이겨 거기서 멈춘다. 벌하는 대상은
    // 로봇의 지금 이탈이 아니라 후보 궤적이 예측하는 최대 이탈이라, 실제 이탈은 상한 앞에서 선다.
    if (!path.empty() && config_.max_path_offset > 0.0 && config_.off_path_band > 0.0) {
      const double limit = std::min(
        config_.max_path_offset_hard,
        std::max(config_.max_path_offset, std::abs(robot_proj.cte)));
      // 포화(min(1.0, ·))를 두지 않는다. 포화하면 한계 밖에서 기울기가 사라져 **되돌아올 유인이
      // 없어진다** — 08 실측 u8c t6 의 복귀 10.60 s 가 그 모습이다 (peak 1.058 m 에서 0.15 m 로
      // 돌아오는 데 10.6 s). 비용이라 위로 열려 있어도 무방하고, 모든 후보가 한계 밖인 상황에서는
      // 가장 덜 벗어난 후보가 이겨 단조 복귀가 된다.
      c.terms.off_path = std::max(0.0, cte_max - limit) / config_.off_path_band;
    }
    c.terms.velocity = std::abs(v_des - c.v) / v_span;
    // 진동
    if (in.has_last) {
      const double eps = config_.oscillation_w_eps;
      const bool dir_flip = (in.v_last > 0.05 && c.v < -0.05) || (in.v_last<-0.05 && c.v>0.05);
      const bool spin_flip = std::abs(c.v) < 0.05 && std::abs(in.w_last) > eps &&
        std::abs(c.w) > eps && c.w * in.w_last < 0.0;
      c.terms.oscillation = (dir_flip || spin_flip) ? 1.0 : 0.0;
    }
    // 동적 장애물 TTC₀ (평균 예측, 원호 연장). J_dyn 은 "예측 접촉 전에 설 수 있는 속도" 초과분이
    // 주항이다: v_safe(TTC₀) = a·max(0, TTC₀ − t_lag). 초과 1 m/s 당 w_d/v_span 이 속도 항 기울기
    // w_v/v_span 보다 커야 감속이 이긴다 (w_d 1.5 > w_v 0.4). 1 − TTC₀/T_pred 는 같은 속도에서
    // TTC 가 긴 쪽(회피 방향)을 고르는 보조항 (dwa.md §2.2).
    if (!dyn.empty()) {
      const std::vector<Pose2D> pred = config_.sustained_sampling ?
        rolloutRamp(in.pose, in.v_meas, in.w_meas, c.v, c.w, config_.prediction_time) :
        rollout(in.pose, c.v, c.w, config_.prediction_time);
      double ttc = kInf;
      for (std::size_t oi = 0; oi < dyn.size(); ++oi) {
        // 결정 층(go/hold/retreat)이 맡은 장애물은 TTC₀ 벌점에서 뺀다. go 는 몸 원통 기준으로 여유
        // 있게 지나간다고 계산한 장애물을 "예측 접촉 전에 설 수 있는 속도" 로 감속시키는 것이 08
        // 접촉의 첫 단계였기 때문이고 (j8c t6: committed 직후 0.8 → 0.6, 이어서 우선회), hold 와
        // retreat 는 TTC₀ 의 조향 보조항(1 − TTC₀/T_pred, 5 s 원호)이 **앞을 가로지르는** 후보에
        // 0.2~0.3 의 이득을 줘 바깥으로 비켜서는 이득(0.1~0.25/0.1 rad/s)을 뒤집기 때문이다
        // (재현 j8c t6 근접판: 작업자 4.7 m 에서 retreat 인데 오른쪽으로 돌아 축을 가로질렀다).
        // 다만 hold/retreat 는 **몸 원통 안**일 때만 뺀다 — 밖에서는 TTC₀ 의 감속이 이롭고
        // (재현 t12/t21: 밖에서까지 빼면 스칠 때 여유 0.50 → 0.30), 안에서는 가로 권한이 먼저다.
        // VO 는 그대로 둔다 — 마지막 방어선. 결정 없는 장애물(다른 작업자)은 예전 그대로.
        if (oi < res.yield.decided.size() && res.yield.decided[oi] &&
          (dec == YieldDecision::kGo || in_body))
        {
          continue;
        }
        const DynamicObstacle & o = dyn[oi];
        if (gate_exempt(o)) {
          continue;
        }
        const double R = config_.robot_radius + o.radius + config_.dynamic_margin;
        ttc = std::min(ttc, firstContactTime(pred, config_.sim_dt, o, R));
      }
      c.ttc = ttc;
      if (std::isfinite(ttc)) {
        const double v_safe = L.decel_lim_x * std::max(0.0, ttc - t_lag);
        const double excess = std::max(0.0, c.v_peak - v_safe) / v_span;
        const double urgency = std::max(0.0, 1.0 - ttc / std::max(1e-6, config_.prediction_time));
        c.terms.dynamic = std::min(1.0, excess + config_.dynamic_steer_gain * urgency);
      }
    }
    if (escaping) {
      // 통로 축(장애물 진행선)까지의 거리 |β| — 클수록 좋다. 반폭 R_c 밖이면 0 (이미 나갔다).
      const DynamicObstacle & o = dyn[static_cast<std::size_t>(res.yield.obstacle)];
      const double su = std::max(1e-6, o.speed());
      const double ux = o.vx / su, uy = o.vy / su;
      const Pose2D & end = c.poses.back();
      const double beta_end = -(end.x - o.x) * uy + (end.y - o.y) * ux;      // 부호 있음 (좌 +)
      const double beta = std::abs(beta_end);
      const double rc = config_.robot_radius + o.radius + config_.yield_corridor_margin;
      // hold / retreat 에서는 **축에서 멀어지는 쪽으로만** 이득을 준다. 예전에는 |β| 만 봐서 축을
      // 가로질러 반대편으로 빠지는 후보(= 작업자 앞을 가로지르기)도 같은 이득을 받았다. 08 실측에서
      // 그 방향이 두 번 죽었다: A→B 에서 남측은 랙 포켓(축~랙 0.99 m, 몸 원통 0.661 밖에 설 자리가
      // 랙과 나란히 붙어야 0.13 m)이고, 앞을 가로지르는 명령은 안전 게이트가 0.5/0.2/0 으로 깎는다
      // (j8c t21: cmd 0.368 → gate_out 0.000). 지금 있는 쪽에서 더 멀어지는 후보만 이득을 받는다.
      // 축 기준 "지금 있는 쪽" 의 부호로 잰 끝점 거리 β_dir. 클수록 이득이고, 음수(축을 가로질러
      // 반대편)는 1 을 넘는 벌점으로 이어진다 — 기울기를 끊지 않는 것이 요점이다. 재현 시험에서
      // 두 번 확인했다: "안쪽이면 전부 1.0" 이면 초기 선회가 안쪽일 때 후보가 전부 같아지고,
      // "가로지르면 1.0" 이면 축 가까이에서 안쪽을 향할 때 롤아웃 1.5 s 안에 전부 가로질러 또
      // 전부 같아진다. 두 경우 다 경로 항이 그대로 축을 가로지르게 만들었다.
      const double beta_now = -(in.pose.x - o.x) * uy + (in.pose.y - o.y) * ux;
      const bool directed = (dec == YieldDecision::kHold || dec == YieldDecision::kRetreat) &&
        std::abs(beta_now) > 0.05;
      // 결정이 있는 비켜서기의 목적은 몸 원통을 여유 있게 벗어나는 것이지 통로 반폭까지 나가는
      // 것이 아니다. 이득은 β = 몸 원통 + 2·body_margin (여유 0.30 m = e-stop 거리) 에서 포화한다
      // — 그 너머는 이탈 예산만 쓴다 (재현 j8c t6: R_c 까지 끌면 이탈 1.25 m, 명세 1.0 초과).
      // 반증됐던 "기준 반경 0.661(겨우 벗어나는 반경)" 과 다르다: 그쪽은 여유 0 에서 포화해 스쳤고,
      // 여기는 여유 0.30 이며 결정 없는 원통 안 탈출(legacy)은 그대로 R_c 다.
      const double beta_target =
        config_.robot_radius + o.radius + 2.0 * config_.yield_body_margin;
      const double beta_dir = directed ?
        std::min(beta_end * (beta_now > 0.0 ? 1.0 : -1.0), beta_target) : beta;
      // 경로에서 더 벗어나며 빠지는 후보에는 이득을 주지 않는다: 통로는 경로를 가로지르므로
      // 빠져나가는 방향은 경로를 따라(대개 뒤로)다. 옆으로 휘면 이탈 예산(명세 1 m)만 쓴다.
      // 통로가 경로와 나란하면(마주 오거나 뒤따라오는 기하) 경로를 따라 움직여도 통로 축까지의
      // 거리가 변하지 않는다 — 빠져나갈 길은 옆으로 비키는 것뿐이라 이탈 억제를 풀어야 한다.
      // 08 실측: 작업자가 경로와 154° 로 마주 오고 로봇은 축에서 β ≈ 0.5 m 떨어져 선다. 반경 합
      // 0.611 m 안이라 서 있으면 반드시 스친다 (접촉 51/52 가 이 모양). 필요한 것은 0.1~0.6 m 의
      // 가로 이동이고 이탈 예산(1 m) 안이다.
      const double pyaw = interpolateAt(path, cum_s, robot_proj.s).theta;
      const double align = std::abs(std::cos(pyaw) * ux + std::sin(pyaw) * uy);
      const double drift = c.max_cte - std::abs(robot_proj.cte) - config_.yield_escape_drift;
      const bool block_drift = drift > 0.0 && align < config_.yield_parallel_cos;
      c.terms.escape = block_drift ? 1.0 :
        std::clamp(1.0 - beta_dir / std::max(1e-6, rc), 0.0, directed ? 2.0 : 1.0);
    }
    c.cost = W.heading * c.terms.heading + W.clearance * c.terms.clearance +
      W.velocity * c.terms.velocity + W.path * c.terms.path +
      W.oscillation * c.terms.oscillation + W.dynamic * c.terms.dynamic +
      W.off_path * c.terms.off_path + W.escape * c.terms.escape;
    ++res.n_valid;
    if (c.vo_rejected) {
      ++res.n_vo_rejected;
    }
    cands.push_back(std::move(c));
  }

  // --- 7) 선택 ---
  const DwaCandidate * best = nullptr;
  for (const auto & c : cands) {
    if (!c.collision && !c.vo_rejected && (best == nullptr || c.cost < best->cost)) {
      best = &c;
    }
  }
  if (best == nullptr && res.n_valid > 0) {
    // VO 포화: 한 주기 동적 창(±a·Δt)이 통째로 VO 안이다. VO 를 끄지 않고 VO 진입이 가장 늦은
    // 샘플(= 등속을 유지해도 가장 오래 안전)을 고른다 — 정면 접근이면 제동, 횡단이면 감속·회피 쪽이
    // 뽑히고, 매 주기 한 창씩 VO 밖으로 옮겨 간다. 진입 시각이 vo_time_tie 안으로 같으면 비용 최소.
    res.vo_saturated = true;
    // 통로 안(kCommitted)에서 포화하면 "가장 늦게 VO 에 들어가는 후보" 는 사실상 **정지**이고,
    // 통로 안에서 멈추는 것이 바로 접촉 메커니즘이다 — 통합 08 실측: 접촉 52 건이 전부
    // yield_state=committed 였고 62 %가 속도 0, 침투 중앙 5.8 mm 로 "비키지 않는 보행자가
    // 서 있는 로봇에 걸어 들어온" 모양이다. 갇혔을 때 정지는 가장 나쁜 선택이므로, 이때는
    // 통로에서 가장 빨리 벗어나는 후보(escape 항이 작은 쪽)를 고른다. 앞뒤 어느 쪽이든 좋다.
    // 다만 "빠져나갈 여지" 가 실제로 있을 때만 그 기준을 쓴다. 정면 접근이면 통로 축이 우리
    // 진행선과 같아 어느 후보를 골라도 통로 축까지의 거리가 같다 — 그때는 정지가 옳고,
    // 예전처럼 VO 진입이 가장 늦은 후보를 고른다 (단위 시험 ClosedLoopHeadOnYields).
    double esc_lo = 1e9, esc_hi = -1e9;
    if (escaping) {
      for (const auto & c : cands) {
        if (!c.collision) {
          esc_lo = std::min(esc_lo, c.terms.escape);
          esc_hi = std::max(esc_hi, c.terms.escape);
        }
      }
    }
    // 게다가 **이미 멈춰 있을 때만** 쓴다. 실측상 접촉의 62 %가 속도 0 이고, 그 상태에서
    // 서 있기가 최악의 선택이라는 것이 확정된 부분이다. 주행 중에는 예전 기준(정면 접근에
    // 대한 제동)을 그대로 둔다 — 단위 시험 ClosedLoopHeadOnYields.
    const bool leave_lane = escaping && esc_hi - esc_lo > config_.escape_spread_min;
    leave_lane_diag = leave_lane;
    // 다만 escape 항만으로 고르면 비용에 실린 이탈 벌점이 통째로 빠진다 — 통로 반폭(1.31 m)에
    // 닿을 때까지 아무것도 말리지 않아 명세 4.7 의 이탈 상한(1.0 m)을 넘긴다. 08 실측 e8b 가
    // 그 모양이었다: 그 구간만 VO 기각이 주기당 17.7 건이었고 이탈이 1.39 m 까지 갔다 (복귀
    // 9.66 s). 예산 안에 드는 후보가 하나라도 있으면 그 안에서만 고른다. 이미 예산 밖이면
    // 지금 이탈까지는 허용해 "더 나가지만 않는" 후보가 남는다.
    // 예산은 **단단한 상한** 이다. 예전에는 max(예산, 지금 이탈) 이라, 예산 밖 후보밖에 없으면
    // 지금 이탈까지 자격을 넓혀 주었고 그만큼 또 자랐다 — 매 주기 조금씩 밀려 u8a 24번 시행이
    // 1.363 m 까지 갔다 (그 시행은 작업자와 최소 거리 0.881 m, 재계획 없음, 양보율 0.46 —
    // 정당한 회피가 그냥 너무 넓었다). 이제 예산 밖이면 **가장 덜 벗어나는 후보** 를 고른다.
    const double dev_cap = config_.max_path_offset_hard;
    // 몸 원통 안에서는 **순수 제자리 회전을 고르지 않는다.** 차동 구동은 회전으로 위치가 변하지
    // 않아 장애물 진행선까지의 거리 β 가 그대로다 — 서 있는 것과 똑같고, 통로 안에서 서 있는
    // 것이 바로 접촉 기제다. 그런데 VO 포화의 기본 기준("VO 에 가장 늦게 들어가는 후보")은
    // 정확히 그 정지·회전 후보를 뽑는다. 정면 기하에서는 후보 간 β 차이가 없어 탈출 순위
    // (leave_lane)도 발동하지 못하므로 이 경로로 빠진다.
    // 08 실측 reg1: 접촉 2건이 모두 VO 포화 중 v = 0.000 · w = +0.4~1.0 의 순수 제자리 회전이었다
    // (시행 12 234.4~235.4 s, 시행 13 252.3~252.9 s, 둘 다 ttc 0.000 · committed · 원통 안).
    // 과거 접촉 51/52 도 같은 모양이었다. 움직이는 비충돌 후보가 하나라도 있으면 그 안에서 고른다
    // — 충돌 후보는 어차피 제외되므로 장애물로 들어가는 선택이 되지는 않는다.
    bool has_moving = false;
    if (in_body) {
      for (const auto & c : cands) {
        if (!c.collision && std::abs(c.v) >= config_.escape_min_speed) {
          has_moving = true;
          break;
        }
      }
    }
    const bool drop_spin = in_body && has_moving;
    for (const auto & c : cands) {
      if (drop_spin && std::abs(c.v) < config_.escape_min_speed) {
        continue;
      }
      if (c.collision) {
        continue;
      }
      if (best == nullptr) {
        best = &c;
        continue;
      }
      const bool c_in = c.max_cte <= dev_cap, b_in = best->max_cte <= dev_cap;
      const bool better = leave_lane ?
        (c_in != b_in ? c_in :
        (!c_in ? c.max_cte < best->max_cte - 1e-9 :   // 둘 다 예산 밖이면 덜 벗어나는 쪽
        (c.terms.escape < best->terms.escape - 1e-9 ||
        (std::abs(c.terms.escape - best->terms.escape) <= 1e-9 && c.cost < best->cost)))) :
        (c.vo_time > best->vo_time + config_.vo_time_tie ||
        (std::abs(c.vo_time - best->vo_time) <= config_.vo_time_tie && c.cost < best->cost));
      if (better) {
        best = &c;
      }
    }
  }
  if (best != nullptr) {
    res.found = true;
    res.best = *best;
    // R2: 후보의 v·w 는 지속 목표이고 실제로 내보내는 것은 한 주기 뒤 램프 상태다
    // (sustained_sampling 이 꺼져 있으면 v_emit == v 이므로 종전과 같다).
    res.v = best->v_emit;
    res.w = best->w_emit;
  } else {
    // 유효 샘플 없음 → 제동 후보 (마지막 샘플)
    res.found = false;
    res.best = cands.back();
    res.v = cands.back().v_emit;
    res.w = cands.back().w_emit;
  }

  // --- R0 계측 (거동에 쓰이지 않는다. 위에서 이미 계산된 값만 읽는다) ---
  // 목적: 접촉률(30 시행당 1 건) 대신 **제어 주기당** 기제 지표를 남긴다.
  // docs/research/dynamic-avoidance-root-cause §6 R0.
  {
    DwaDiag & g = res.diag;
    g.escaping = escaping;
    g.leave_lane = leave_lane_diag;
    double cmin = kInf, cmax = -kInf;
    double vlo = kInf, vhi = -kInf, wlo = kInf, whi = -kInf;
    double dmin = kInf, dmax = -kInf, tmin = kInf;
    for (const auto & c : cands) {
      vlo = std::min(vlo, c.v);
      vhi = std::max(vhi, c.v);
      wlo = std::min(wlo, c.w);
      whi = std::max(whi, c.w);
      if (std::isfinite(c.ttc)) {
        tmin = std::min(tmin, c.ttc);
      }
      if (!c.collision) {
        ++g.n_collision_free;
        // 롤아웃 종점 변위 — "후보들이 서로 구별되는가" 의 직접 관측 (§3.1)
        if (!c.poses.empty()) {
          const Pose2D & e = c.poses.back();
          const double d = std::hypot(e.x - in.pose.x, e.y - in.pose.y);
          dmin = std::min(dmin, d);
          dmax = std::max(dmax, d);
        }
        if (!c.vo_rejected) {
          ++g.n_selectable;
          cmin = std::min(cmin, c.cost);
          cmax = std::max(cmax, c.cost);
        }
      }
    }
    if (g.n_selectable > 0) {
      g.cost_min = cmin;
      g.cost_max = cmax;
    }
    if (std::isfinite(vlo)) {
      g.v_lo = vlo;
      g.v_hi = vhi;
      g.w_lo = wlo;
      g.w_hi = whi;
    }
    if (std::isfinite(dmin) && std::isfinite(dmax)) {
      g.disp_span = dmax - dmin;
    }
    g.ttc_min = std::isfinite(tmin) ? tmin : -1.0;
    // 최근접 동적 장애물까지의 거리 — §3.3 의 조기 반환(‖p‖ < R_vo) 지배 여부를 본다
    double pmin = kInf;
    for (const auto & o : in.obstacles) {
      pmin = std::min(pmin, std::hypot(o.x - in.pose.x, o.y - in.pose.y));
    }
    g.nearest_obs = std::isfinite(pmin) ? pmin : -1.0;
  }
  return res;
}

}  // namespace core
}  // namespace amr_navigation
