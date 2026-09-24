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
  y.lookahead = config_.yield_lookahead;
  y.max_zone = config_.yield_max_zone;
  y.min_speed = config_.dynamic_speed_threshold;
  y.accel = config_.limits.acc_lim_x;
  y.decel = config_.limits.decel_lim_x;
  y.jerk = config_.limits.jerk_lim_x;
  y.latency = config_.approach_latency;
  y.v_max = config_.limits.max_vel_x;
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
  win.v_lo = std::max(L.min_vel_x, v_c - L.decel_lim_x * dt);
  win.v_hi = std::min(std::min(L.max_vel_x, v_cap), v_c + L.acc_lim_x * dt);
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
  std::vector<double> vs = linspace(win.v_lo, win.v_hi, std::max(1, config_.vx_samples));
  std::vector<double> ws = linspace(win.w_lo, win.w_hi, std::max(1, config_.vth_samples));
  // 직진(ω = 0) 열을 정확히 포함
  if (win.w_lo <= 0.0 && win.w_hi >= 0.0 &&
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
  vb = std::clamp(vb, win.v_lo, win.v_hi);
  double wb = w_c - (w_c > 0.0 ? 1.0 : -1.0) * std::min(std::abs(w_c), L.acc_lim_theta * dt);
  wb = std::clamp(wb, win.w_lo, win.w_hi);
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
  res.yield = evaluateYield(
    path, cum_s, in.pose, robot_proj.s, std::max(0.0, in.v_meas), dyn, yieldConfig());
  const double v_cap = std::min(std::min(L.max_vel_x, in.speed_limit), res.yield.speed_limit);
  res.v_cap = v_cap;
  res.window = dynamicWindow(v_c, w_c, v_cap);
  // 이미 통로 안(kCommitted): 앞은 VO 가 막으므로 뒤로 빠질 수 있게 창 아래쪽을 연다.
  // 한 주기에 닿을 수 있는 범위(감속 한계) 안에서만 내린다 — 명령이 튀지 않는다.
  // 횡단일 때만 의미가 있다: 정면 접근(장애물 진행 방향이 로봇 헤딩과 나란)은 통로 축이 우리
  // 진행선과 같아 "옆으로 빠진다" 가 성립하지 않는다 — 그 상황은 VO/TTC 의 감속이 맡는다.
  // 이미 서 있거나 뒤로 빠지는 중일 때만 연다. 정상 주행 중에는 건드리지 않는다 — 연속 운용(14)
  // 실측: 주행 중에도 열어 두었더니 후진↔전진이 되풀이되어 "Failed to make progress" 58 건과
  // 작업 시간 초과 1 건이 났다 (같은 구성의 앞 실행은 0 건).
  bool escaping = res.yield.state == YieldState::kCommitted &&
    config_.yield_escape_speed > 0.0 && res.yield.obstacle >= 0 &&
    static_cast<std::size_t>(res.yield.obstacle) < dyn.size() &&
    in.v_meas < config_.yield_escape_v_meas;
  // (각도 조건은 두지 않는다: 08 경로는 작업자 진행 방향과 26° 라 |cos| 0.9 로 걸러졌고, 실제로
  //  정지한 채 통로 안에서 접촉이 났다. 정면 접근은 "정지·후진 중" 조건과 VO 가 함께 막는다.)
  if (escaping) {
    res.window.v_lo = std::max(
      -config_.yield_escape_speed, v_c - L.decel_lim_x * config_.control_period);
    res.window.v_lo = std::min(res.window.v_lo, res.window.v_hi);
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
    c.poses = rollout(in.pose, c.v, c.w, T);

    // --- 4) 충돌 검사 (목표 너머 제외) ---
    std::size_t n_check = c.poses.size();
    for (std::size_t k = 1; k < c.poses.size(); ++k) {
      const double s_k = std::abs(c.v) * k * config_.sim_dt;
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
      const Point2D v_eff = chordVelocity(in.pose.theta, c.v, c.w, config_.vo_time_horizon);
      for (const auto & o : dyn) {
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
        const double s_ref = robot_proj.s + std::abs(c.v) * k * config_.sim_dt;
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
    if (!path.empty() && config_.max_path_offset > 0.0 && config_.off_path_band > 0.0) {
      const double limit = std::max(config_.max_path_offset, std::abs(robot_proj.cte));
      c.terms.off_path = std::min(1.0, std::max(0.0, cte_max - limit) / config_.off_path_band);
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
      const std::vector<Pose2D> pred = rollout(in.pose, c.v, c.w, config_.prediction_time);
      double ttc = kInf;
      for (const auto & o : dyn) {
        const double R = config_.robot_radius + o.radius + config_.dynamic_margin;
        ttc = std::min(ttc, firstContactTime(pred, config_.sim_dt, o, R));
      }
      c.ttc = ttc;
      if (std::isfinite(ttc)) {
        const double v_safe = L.decel_lim_x * std::max(0.0, ttc - t_lag);
        const double excess = std::max(0.0, std::abs(c.v) - v_safe) / v_span;
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
      const double beta = std::abs(-(end.x - o.x) * uy + (end.y - o.y) * ux);
      const double rc = config_.robot_radius + o.radius + config_.yield_corridor_margin;
      // 경로에서 더 벗어나며 빠지는 후보에는 이득을 주지 않는다: 통로는 경로를 가로지르므로
      // 빠져나가는 방향은 경로를 따라(대개 뒤로)다. 옆으로 휘면 이탈 예산(명세 1 m)만 쓴다.
      const double drift = c.max_cte - std::abs(robot_proj.cte) - config_.yield_escape_drift;
      c.terms.escape = drift > 0.0 ? 1.0 :
        std::clamp(1.0 - beta / std::max(1e-6, rc), 0.0, 1.0);
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
    for (const auto & c : cands) {
      if (c.collision) {
        continue;
      }
      if (best == nullptr || c.vo_time > best->vo_time + config_.vo_time_tie ||
        (std::abs(c.vo_time - best->vo_time) <= config_.vo_time_tie && c.cost < best->cost))
      {
        best = &c;
      }
    }
  }
  if (best != nullptr) {
    res.found = true;
    res.best = *best;
    res.v = best->v;
    res.w = best->w;
  } else {
    // 유효 샘플 없음 → 제동 후보 (마지막 샘플)
    res.found = false;
    res.best = cands.back();
    res.v = cands.back().v;
    res.w = cands.back().w;
  }
  return res;
}

}  // namespace core
}  // namespace amr_navigation
