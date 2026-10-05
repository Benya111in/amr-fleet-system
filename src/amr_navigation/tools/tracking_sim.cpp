// 폐루프 경로 추종 시뮬레이터 — ROS 없이 제어기 튜닝을 반복하는 도구
// (docs/algorithms/dwa.md §4.1, path_tracking.md §1.5).
//
//   ros2 run amr_navigation tracking_sim [dwa|pp] [key=value ...]
//
// 체인 (kinematic_sim.py + velocity_profiler_node 와 같은 구조, 빈 코스트맵):
//   제어기 20 Hz (DwaPlanner 또는 PurePursuit) → VelocityProfiler 50 Hz (저크 필터 + 모델 추종 PI)
//   → DiffDrive 속도 서보 100 Hz (순수 지연 0.04 s + 1차 지연 0.08 s + 가속 제한) → 정확 원호 적분.
// 목표 판정은 controller_server SimpleGoalChecker(stateful, xy 0.10 m, yaw 0.05 rad)처럼
// 한 뒤 명령 0, 정지할 때까지 더 적분해 목표 통과량을 잰다.
// 기준 경로: closed_loop_eval.py 와 같은 straight 20 m / uturn (R 2 m 180°) /
// scurve (R 3 m ±30°·60° 물결).
// 출력: 경로별 CTE 평균·최대 (직선/곡선: |κ| > 0.1 1/m 정점 ± 0.5 m 를 곡선), 주행 시간 대 예측
//   (SpeedProfile::predictTravelTime), 목표 통과량, 제어기 1 주기 계산 시간.
// 조정 가능한 키 (DWA): heading path velocity clearance la_gain la_offset la_min la_max path_band
//   path_eval_time vx vth approach_latency
// 조정 가능한 키 (PP): lookahead_time min_la max_la chord approach_latency
// 공통: offset (출발 자세 횡 오프셋 [m])
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/pure_pursuit.hpp"
#include "amr_navigation/core/speed_profile.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

namespace core = amr_navigation::core;

namespace
{
constexpr double kStep = 0.05;
constexpr double kDt = 0.01;

void appendStraight(std::vector<core::Pose2D> & p, double len)
{
  const core::Pose2D o = p.back();
  const int n = static_cast<int>(std::lround(len / kStep));
  for (int i = 1; i <= n; ++i) {
    p.push_back(core::integrateArc(o, 1.0, 0.0, i * kStep));
  }
}

void appendArc(std::vector<core::Pose2D> & p, double radius, double angle)
{
  const core::Pose2D o = p.back();
  const int n = static_cast<int>(std::lround(std::abs(angle) * radius / kStep));
  const double sgn = angle >= 0.0 ? 1.0 : -1.0;
  for (int i = 1; i <= n; ++i) {
    p.push_back(core::integrateArc(o, 1.0, sgn / radius, i * kStep));
  }
}

std::map<std::string, std::vector<core::Pose2D>> referencePaths()
{
  std::map<std::string, std::vector<core::Pose2D>> out;
  std::vector<core::Pose2D> s{{0.0, 0.0, 0.0}};
  appendStraight(s, 20.0);
  out["1_straight"] = s;
  std::vector<core::Pose2D> u{{0.0, 0.0, core::kPi / 2.0}};
  appendStraight(u, 3.0);
  appendArc(u, 2.0, -core::kPi);
  appendStraight(u, 3.0);
  out["2_uturn"] = u;
  std::vector<core::Pose2D> c{{0.0, 0.0, 0.0}};
  for (int k = 0; k < 2; ++k) {
    appendArc(c, 3.0, core::kPi / 6.0);
    appendArc(c, 3.0, -core::kPi / 3.0);
    appendArc(c, 3.0, core::kPi / 6.0);
  }
  appendStraight(c, 1.0);
  out["3_scurve"] = c;
  return out;
}

// 서보 한 축: 순수 지연 → 1차 지연 → 가속 제한 (kinematic_sim.py ServoAxis 와 같다)
struct Servo
{
  Servo(double tau, double delay, double a_max)
  : tau_(tau), a_max_(a_max), buf_(static_cast<std::size_t>(std::lround(delay / kDt)), 0.0) {}
  double step(double cmd)
  {
    if (!buf_.empty()) {
      buf_.push_back(cmd);
      cmd = buf_.front();
      buf_.erase(buf_.begin());
    }
    const double alpha = 1.0 - std::exp(-kDt / tau_);
    const double lim = a_max_ * kDt;
    value_ += std::clamp(alpha * (cmd - value_), -lim, lim);
    return value_;
  }
  double value() const {return value_;}
  double tau_;
  double a_max_;
  std::vector<double> buf_;
  double value_{0.0};
};

struct Result
{
  double cte_sum[2]{0.0, 0.0};
  double cte_max[2]{0.0, 0.0};
  int n[2]{0, 0};
  double time{0.0};
  double t_pred{0.0};
  double overshoot{0.0};
  double cycle_us_mean{0.0};
  double cycle_us_max{0.0};
  bool reached{false};
};

double get(const std::map<std::string, double> & kv, const std::string & key, double def)
{
  const auto it = kv.find(key);
  return it == kv.end() ? def : it->second;
}

Result run(
  const std::string & ctrl, const std::vector<core::Pose2D> & path,
  const std::map<std::string, double> & kv)
{
  Result r;
  const std::vector<double> cum = core::cumulativeLength(path);
  const std::vector<double> kappa = core::discreteCurvature(path, 0.25);
  std::vector<int> curve(path.size(), 0);
  for (std::size_t i = 0; i < path.size(); ++i) {
    for (std::size_t j = 0; j < path.size(); ++j) {
      if (std::abs(kappa[j]) > 0.1 && std::abs(cum[j] - cum[i]) <= 0.5) {
        curve[i] = 1;
        break;
      }
    }
  }
  core::SpeedProfile pred;
  pred.build(path, core::SpeedProfileConfig());
  r.t_pred = pred.predictTravelTime();

  // 제어기
  core::DwaConfig dc;
  dc.weights.heading = get(kv, "heading", dc.weights.heading);
  dc.weights.path = get(kv, "path", dc.weights.path);
  dc.weights.velocity = get(kv, "velocity", dc.weights.velocity);
  dc.weights.clearance = get(kv, "clearance", dc.weights.clearance);
  dc.weights.oscillation = 0.5;
  dc.vx_samples = static_cast<int>(get(kv, "vx", 11));
  dc.vth_samples = static_cast<int>(get(kv, "vth", 31));
  dc.heading_lookahead_gain = get(kv, "la_gain", dc.heading_lookahead_gain);
  dc.heading_lookahead_offset = get(kv, "la_offset", dc.heading_lookahead_offset);
  dc.heading_lookahead_min = get(kv, "la_min", dc.heading_lookahead_min);
  dc.heading_lookahead_max = get(kv, "la_max", dc.heading_lookahead_max);
  dc.path_band = get(kv, "path_band", dc.path_band);
  dc.path_eval_time = get(kv, "path_eval_time", dc.path_eval_time);
  dc.approach_latency = get(kv, "approach_latency", dc.approach_latency);
  dc.goal_align_distance = 0.08;
  const core::DwaPlanner dwa(dc);
  core::PurePursuitConfig pc;
  pc.lookahead_time = get(kv, "lookahead_time", pc.lookahead_time);
  pc.min_lookahead = get(kv, "min_la", pc.min_lookahead);
  pc.max_lookahead = get(kv, "max_la", pc.max_lookahead);
  pc.use_chord_correction = get(kv, "chord", 0.0) > 0.5;
  pc.approach_latency = get(kv, "approach_latency", pc.approach_latency);
  core::PurePursuit pp(pc);
  pp.setPath(path);

  core::VelocityProfiler prof;   // 기본 = velocity_profiler.yaml (저크 2, PI 0.4/2.0)
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  core::Pose2D pose = path.front();
  // offset=<m>: 경로 왼쪽으로 옮긴 자세에서 출발 (수렴·강건성 확인)
  const double off = get(kv, "offset", 0.0);
  pose.x -= off * std::sin(pose.theta);
  pose.y += off * std::cos(pose.theta);
  double v_cmd = 0.0;
  double w_cmd = 0.0;
  double v_out = 0.0;
  double w_out = 0.0;
  bool has_last = false;
  bool xy_latched = false;
  bool done = false;
  std::size_t hint = 0;
  int n_cycle = 0;
  const core::Pose2D goal = path.back();
  for (int k = 0; k < 12000; ++k) {
    if (k % 5 == 0 && !done) {
      const double dg = core::distance(pose, goal);
      xy_latched = xy_latched || dg <= 0.10;
      if (xy_latched && std::abs(core::wrapAngle(goal.theta - pose.theta)) <= 0.05) {
        done = true;
        r.reached = true;
        v_cmd = w_cmd = 0.0;
      } else {
        const auto t0 = std::chrono::steady_clock::now();
        if (ctrl == "pp") {
          const core::PurePursuitOutput o = pp.compute(pose, v_out);
          v_cmd = o.v;
          w_cmd = o.w;
        } else {
          // 플러그인 PlanWindow 와 같이 최근접점부터 6 m 창
          double best = 1e18;
          const std::size_t end = std::min(path.size(), hint + 200);
          for (std::size_t i = hint; i < end; ++i) {
            const double d = core::distance(path[i], pose);
            if (d < best) {
              best = d;
              hint = i;
            }
          }
          std::vector<core::Pose2D> win;
          const std::size_t s0 = hint > 0 ? hint - 1 : 0;
          for (std::size_t i = s0; i < path.size() && cum[i] - cum[s0] <= 6.05; ++i) {
            win.push_back(path[i]);
          }
          core::DwaInput in;
          in.pose = pose;
          in.v_meas = sv.value();
          in.w_meas = sw.value();
          in.has_last = has_last;
          in.v_last = v_cmd;
          in.w_last = w_cmd;
          in.path = &win;
          in.path_end_is_goal = win.back().x == goal.x && win.back().y == goal.y;
          const core::DwaResult res = dwa.compute(
            in, [](const core::Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
          v_cmd = res.v;
          w_cmd = res.w;
          has_last = true;
        }
        const double us = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - t0).count();
        r.cycle_us_mean += us;
        r.cycle_us_max = std::max(r.cycle_us_max, us);
        ++n_cycle;
      }
    }
    if (k % 2 == 0) {
      const core::ProfilerOutput o = prof.update(v_cmd, w_cmd, true, sv.value(), sw.value(), 0.02);
      v_out = o.v;
      w_out = o.w;
    }
    const double v = sv.step(v_out);
    const double w = sw.step(w_out);
    pose = core::integrateArc(pose, v, w, kDt);
    if (!done) {
      r.time += kDt;
      const core::Projection pr = core::projectOntoPath(path, {pose.x, pose.y}, 0, 0, &cum);
      const int seg = curve[std::min(path.size() - 1, pr.segment + (pr.t > 0.5 ? 1 : 0))];
      r.cte_sum[seg] += std::abs(pr.cte);
      r.cte_max[seg] = std::max(r.cte_max[seg], std::abs(pr.cte));
      ++r.n[seg];
    } else if (std::abs(v) < 1e-4 && std::abs(w) < 1e-4 && std::abs(v_out) < 1e-4) {
      break;
    }
  }
  r.overshoot = (pose.x - goal.x) * std::cos(goal.theta) + (pose.y - goal.y) * std::sin(goal.theta);
  r.cycle_us_mean /= std::max(1, n_cycle);
  return r;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string ctrl = argc > 1 ? argv[1] : "dwa";
  std::map<std::string, double> kv;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const auto eq = a.find('=');
    if (eq != std::string::npos) {
      kv[a.substr(0, eq)] = std::atof(a.substr(eq + 1).c_str());
    }
  }
  std::printf(
    "| %s | path | ok | CTE straight mean / max [cm] | CTE curve mean / max [cm] "
    "| T act / pred [s] | err [%%] | overshoot [cm] | cycle mean / max [ms] |\n",
    ctrl.c_str());
  std::printf("| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n");
  for (const auto & [name, path] : referencePaths()) {
    const Result r = run(ctrl, path, kv);
    auto mean = [&r](int s) {return r.n[s] > 0 ? 100.0 * r.cte_sum[s] / r.n[s] : 0.0;};
    std::printf(
      "| %s | %s | %d | %.1f / %.1f | %.1f / %.1f | %.2f / %.2f | %.1f | %.1f | %.3f / %.3f |\n",
      ctrl.c_str(), name.c_str() + 2, r.reached ? 1 : 0, mean(0), 100.0 * r.cte_max[0], mean(1),
      100.0 * r.cte_max[1], r.time, r.t_pred, 100.0 * std::abs(r.time - r.t_pred) / r.time,
      100.0 * r.overshoot, 1e-3 * r.cycle_us_mean, 1e-3 * r.cycle_us_max);
  }
  return 0;
}
