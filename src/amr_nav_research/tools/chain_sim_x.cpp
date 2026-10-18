// chain_sim_x — chain_sim (src/amr_navigation/tools/chain_sim.cpp) 의 연구 사본
// (docs/research/paper-research/prereg.md §3.1, contract.md §1).
//
//   chain_sim_x <scenes.txt> <out.jsonl> [key=value ...] [cfg=<file>]
//
// 원본과 다른 점 (모두 기본값이면 원본과 비트 단위로 같은 JSONL 지표를 낸다):
//   - 제어기 코어가 amr_nav_research::DwaPlannerX 다. 연구 스위치 키 (cfg 파일·인자 모두):
//     reset_persist_cycles, reset_hard_v, jerk_consistent, jerk_lim, pp_candidate
//     (플러그인 파라미터 <plugin>.research.* 에서 "research." 를 뺀 이름).
//   - stateful=0|1: SimpleGoalChecker stateful 흉내 (research_io.hpp GoalCheckEmu). 기본 0.
//   - reset_log=<path> (또는 cfg 키 log_resets): 창 재설정 기록 CSV (research_io.hpp).
//     열 = contract §1 의 11 열 + scene, arm, seed. 덧붙이기로 연다.
//   - JSONL 끝에 필드를 더한다: stateful, research 설정, n_resets(+hard·persist),
//     n_r1_suppressed, n_pp_best, n_jerk_applied, t_xy_latch. 원본 필드와 순서는 그대로다.
//   - 2 라운드 (prereg_r2.md): 키 reset_model, reset_model_theta, reset_model_theta_w,
//     model_delay, model_tau (F1), goal_extension_dist (F2), blocked_detour_shift,
//     blocked_lookahead (F3). JSONL 끝에 2 라운드 설정과 n_detour, n_detour_blocked,
//     n_goal_ext, t_entry·yaw_err_entry (참값 자세가 목표 0.10 m 안에 처음 든 시각과 그때
//     |yaw 오차|, 100 Hz 플랜트 스텝 기준, 없으면 -1) 를 더한다.
//   - 3 라운드 (prereg_r3.md): 키 accel_hold (G1). JSONL 끝에 accel_hold, n_g1_held (G1 이
//     선속도 재설정 후보를 유예한 주기 수), 재설정 기록 끝 열 g1_held (장면 안 누적).
//
// 아래는 원본 머리 주석이다.
//
// tracking_sim 과 같은 사슬 (제어기 rate Hz → VelocityProfiler 50 Hz → 서보 100 Hz: 순수 지연
// 0.04 s + 1차 0.08 s + 가속 제한 → 정확 원호 적분)에 다음을 더했다:
//   - 장면 파일의 임의 경로·축정렬 장애물 (정적 코스트맵 0.025 m, Nav2 식 팽창 0.2 / 0.8 / 3.0 =
//     지역 코스트맵 값) 또는 lidar=1: 10 Hz σ 0.03 LiDAR(720 빔, 앞 0.15 m)로 매 스캔 새로 만든
//     8 × 8 m 창 코스트맵 (게이트 중앙값 denoise — costmap_scan_filter_node 와 같은 기본값)
//   - 플러그인과 같은 설정 의미: 플러그인 파라미터 이름(key=value, cfg 파일 "key value" 줄)과
//     플러그인 기본값, PlanWindow(경로 창 path_horizon), 직전 명령 되먹임, 유효 샘플 없음 예외
//     (no_valid_patience)와 controller_server failure_tolerance 1.0 s
//   - controller_server 의 SimpleGoalChecker(xy 0.10, yaw 0.05, stateful false)와
//     SimpleProgressChecker(0.3 m / 20 s)
//   - plant=ideal: 하류 사슬 없음 (제어 주기 동안 명령을 그대로 적분 — M3 기전 확인용)
//   - 잡음: odom_noise_v/w (백색, 제어기·프로파일러 되먹임 각각),
//     pose_noise_xy/yaw (가우스-마르코프 τ pose_noise_tau, 제어기·목표 판정 입력), seed
// 출력: 장면마다 JSON 한 줄 (성공·실패 원인, CTE 직선/곡선 평균·최대, 시간, 목표 정지 오차(부호),
//   정착 시간, 명령 가속·저크, 최소 참값 여유, 주기 계산 시간, M4·M5 계측).
// 장면 파일: "scene <id> <family>" / "path <n>" + n 줄 "x y θ" / "boxes <m>" + m 줄
//   "x0 y0 x1 y1" / "end".
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/pure_pursuit.hpp"
#include "amr_navigation/core/scan_denoise.hpp"
#include "amr_navigation/core/speed_profile.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"
#include "amr_nav_research/dwa_x.hpp"
#include "amr_nav_research/research_io.hpp"

namespace core = amr_navigation::core;
namespace rx = amr_nav_research;

namespace
{
constexpr double kDt = 0.01;          // 플랜트 적분 [s]
constexpr double kRes = 0.025;        // 코스트맵 해상도 [m] (지역 코스트맵)
constexpr double kHalfL = 0.30;       // 풋프린트 0.60 × 0.40
constexpr double kHalfW = 0.20;

struct Box
{
  double x0, y0, x1, y1;
};

struct Scene
{
  std::string id;
  std::string family;
  std::vector<core::Pose2D> path;
  std::vector<Box> boxes;
};

std::vector<Scene> readScenes(const std::string & file)
{
  std::ifstream in(file);
  std::vector<Scene> out;
  std::string tok;
  while (in >> tok) {
    if (tok != "scene") {
      continue;
    }
    Scene s;
    in >> s.id >> s.family;
    std::size_t n = 0;
    in >> tok >> n;
    s.path.resize(n);
    for (auto & p : s.path) {
      in >> p.x >> p.y >> p.theta;
    }
    std::size_t m = 0;
    in >> tok >> m;
    s.boxes.resize(m);
    for (auto & b : s.boxes) {
      in >> b.x0 >> b.y0 >> b.x1 >> b.y1;
    }
    in >> tok;   // end
    out.push_back(std::move(s));
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// 설정: 플러그인 이름·기본값 (dwa_controller.cpp / pure_pursuit_controller.cpp configure 와 같다)

struct DwaExtra
{
  double path_horizon{6.0};
  int no_valid_patience{10};
};

core::DwaConfig dwaPluginDefaults(double rate)
{
  core::DwaConfig c;
  c.limits.max_vel_x = 1.0;
  c.limits.min_vel_x = 0.0;
  c.limits.max_vel_theta = 1.5;
  c.limits.acc_lim_x = 1.0;
  c.limits.decel_lim_x = 1.0;
  c.limits.acc_lim_theta = 2.0;
  c.limits.jerk_lim_x = 2.0;
  c.weights = core::DwaWeights();
  c.weights.oscillation = 0.5;
  c.control_period = 1.0 / rate;
  c.vth_samples = 31;
  c.jerk_lim_theta = 6.0;
  c.goal_align_distance = 0.08;
  c.recenter_step = kRes;
  c.robot_radius = std::hypot(kHalfL, kHalfW);
  return c;
}

bool toBool(const std::string & v)
{
  return v == "1" || v == "true" || v == "True" || v == "TRUE";
}

bool applyDwa(core::DwaConfig & c, DwaExtra & e, const std::string & k, const std::string & v)
{
  const double d = std::atof(v.c_str());
  auto & L = c.limits;
  auto & W = c.weights;
  const std::map<std::string, double *> dbl{
    {"max_vel_x", &L.max_vel_x}, {"min_vel_x", &L.min_vel_x}, {"max_vel_theta", &L.max_vel_theta},
    {"acc_lim_x", &L.acc_lim_x}, {"decel_lim_x", &L.decel_lim_x},
    {"acc_lim_theta", &L.acc_lim_theta}, {"jerk_lim_x", &L.jerk_lim_x},
    {"heading_weight", &W.heading}, {"clearance_weight", &W.clearance},
    {"velocity_weight", &W.velocity}, {"path_weight", &W.path},
    {"oscillation_weight", &W.oscillation}, {"dynamic_weight", &W.dynamic},
    {"off_path_weight", &W.off_path}, {"escape_weight", &W.escape},
    {"jerk_lim_theta", &c.jerk_lim_theta}, {"sim_dt", &c.sim_dt},
    {"sim_time_min", &c.sim_time_min}, {"sim_time_max", &c.sim_time_max},
    {"commit_time", &c.commit_time}, {"approach_latency", &c.approach_latency},
    {"window_reset_v", &c.window_reset_v}, {"window_reset_w", &c.window_reset_w},
    {"heading_lookahead_gain", &c.heading_lookahead_gain},
    {"heading_lookahead_offset", &c.heading_lookahead_offset},
    {"heading_lookahead_min", &c.heading_lookahead_min},
    {"heading_lookahead_max", &c.heading_lookahead_max}, {"path_band", &c.path_band},
    {"max_path_offset", &c.max_path_offset}, {"max_path_offset_hard", &c.max_path_offset_hard},
    {"off_path_band", &c.off_path_band}, {"path_eval_time", &c.path_eval_time},
    {"goal_align_distance", &c.goal_align_distance},
    {"goal_overshoot_margin", &c.goal_overshoot_margin},
    {"oscillation_w_eps", &c.oscillation_w_eps}, {"prediction_time", &c.prediction_time},
    {"dynamic_margin", &c.dynamic_margin}, {"vo_time_horizon", &c.vo_time_horizon},
    {"vo_margin", &c.vo_margin}, {"vo_max_range", &c.vo_max_range},
    {"dynamic_speed_threshold", &c.dynamic_speed_threshold},
    {"dynamic_steer_gain", &c.dynamic_steer_gain}, {"vo_time_tie", &c.vo_time_tie},
    {"recenter_min_cost", &c.recenter_min_cost}, {"recenter_max_shift", &c.recenter_max_shift},
    {"recenter_goal_keep", &c.recenter_goal_keep},
    {"textbook_lookahead", &c.textbook_lookahead}, {"path_horizon", &e.path_horizon},
    {"escape_min_speed", &c.escape_min_speed}};
  const std::map<std::string, bool *> flag{
    {"sustained_sampling", &c.sustained_sampling},
    {"use_dynamic_obstacles", &c.use_dynamic_obstacles},
    {"use_velocity_obstacles", &c.use_velocity_obstacles},
    {"yield_crossing", &c.yield_crossing}, {"yield_decision", &c.yield_decision},
    {"recenter_narrow", &c.recenter_narrow}, {"textbook_cost", &c.textbook_cost},
    {"relative_clearance", &c.relative_clearance},
    {"window_center_measured", &c.window_center_measured},
    {"brake_candidate", &c.brake_candidate}};
  if (auto it = dbl.find(k); it != dbl.end()) {
    *it->second = d;
  } else if (auto jt = flag.find(k); jt != flag.end()) {
    *jt->second = toBool(v);
  } else if (k == "vx_samples") {
    c.vx_samples = static_cast<int>(d);
  } else if (k == "vth_samples") {
    c.vth_samples = static_cast<int>(d);
  } else if (k == "recenter_smooth") {
    c.recenter_smooth = static_cast<int>(d);
  } else if (k == "no_valid_patience") {
    e.no_valid_patience = static_cast<int>(d);
  } else if (k == "goal_approach") {
    c.goal_approach = v == "none" ? 1 : (v == "trapezoid" ? 2 : 0);
    if (v != "none" && v != "trapezoid" && v != "jerk_latency") {
      std::fprintf(stderr, "chain_sim: goal_approach '%s' 는 알 수 없다\n", v.c_str());
      std::exit(2);
    }
  } else {
    return false;
  }
  return true;
}

struct PpExtra
{
  double velocity_reset_threshold{0.3};
  bool use_collision_detection{true};
  double collision_check_time{1.0};
};

bool applyPp(
  core::PurePursuitConfig & c, PpExtra & e, const std::string & k,
  const std::string & v)
{
  const double d = std::atof(v.c_str());
  const std::map<std::string, double *> dbl{
    {"desired_linear_vel", &c.desired_linear_vel}, {"lookahead_time", &c.lookahead_time},
    {"min_lookahead_dist", &c.min_lookahead}, {"max_lookahead_dist", &c.max_lookahead},
    {"max_angular_vel", &c.max_angular_vel}, {"max_angular_accel", &c.max_angular_accel},
    {"max_linear_accel", &c.max_linear_accel}, {"max_linear_decel", &c.max_linear_decel},
    {"max_lateral_accel", &c.max_lateral_accel}, {"max_linear_jerk", &c.max_linear_jerk},
    {"min_approach_linear_velocity", &c.min_approach_vel},
    {"approach_latency", &c.approach_latency}, {"curvature_window", &c.curvature_window},
    {"rotate_to_heading_min_angle", &c.rotate_to_heading_min_angle},
    {"rotate_to_heading_max_v", &c.rotate_to_heading_max_v},
    {"goal_align_distance", &c.goal_align_distance},
    {"goal_yaw_tolerance", &c.goal_yaw_tolerance}, {"chord_gain", &c.chord_gain},
    {"velocity_reset_threshold", &e.velocity_reset_threshold},
    {"collision_check_time", &e.collision_check_time}};
  if (auto it = dbl.find(k); it != dbl.end()) {
    *it->second = d;
  } else if (k == "use_rotate_to_heading") {
    c.use_rotate_to_heading = toBool(v);
  } else if (k == "use_chord_correction") {
    c.use_chord_correction = toBool(v);
  } else if (k == "use_collision_detection") {
    e.use_collision_detection = toBool(v);
  } else {
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// 세계: 참값 장애물(축정렬 사각형)과 제어기가 보는 코스트맵

double rayBox(double ox, double oy, double dx, double dy, const Box & b)
{
  double t0 = 0.0;
  double t1 = 1e18;
  const double o[2] = {ox, oy};
  const double dd[2] = {dx, dy};
  const double lo[2] = {b.x0, b.y0};
  const double hi[2] = {b.x1, b.y1};
  for (int k = 0; k < 2; ++k) {
    if (std::abs(dd[k]) < 1e-12) {
      if (o[k] < lo[k] || o[k] > hi[k]) {
        return 1e18;
      }
      continue;
    }
    double a = (lo[k] - o[k]) / dd[k];
    double c = (hi[k] - o[k]) / dd[k];
    if (a > c) {
      std::swap(a, c);
    }
    t0 = std::max(t0, a);
    t1 = std::min(t1, c);
    if (t0 > t1) {
      return 1e18;
    }
  }
  return t0 > 0.0 ? t0 : 1e18;
}

std::array<core::Point2D, 4> robotCorners(const core::Pose2D & p)
{
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  std::array<core::Point2D, 4> out;
  const double lx[4] = {kHalfL, kHalfL, -kHalfL, -kHalfL};
  const double ly[4] = {kHalfW, -kHalfW, -kHalfW, kHalfW};
  for (int i = 0; i < 4; ++i) {
    out[static_cast<std::size_t>(i)] = {p.x + c * lx[i] - s * ly[i], p.y + s * lx[i] + c * ly[i]};
  }
  return out;
}

// 로봇 사각형 ↔ 사각형 참값 최소 거리 (겹치면 0, *overlap = true). 볼록 다각형 둘의 거리는
// 겹치지 않으면 한쪽 꼭짓점과 다른 쪽 다각형 사이에서 나온다.
double trueClearance(const core::Pose2D & p, const std::vector<Box> & boxes, bool * overlap)
{
  const auto rc = robotCorners(p);
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  double best = 1e9;
  for (const auto & b : boxes) {
    // 분리축 판정 (상자 축 2 + 로봇 축 2)
    double rx0 = 1e18, rx1 = -1e18, ry0 = 1e18, ry1 = -1e18;
    for (const auto & q : rc) {
      rx0 = std::min(rx0, q.x);
      rx1 = std::max(rx1, q.x);
      ry0 = std::min(ry0, q.y);
      ry1 = std::max(ry1, q.y);
    }
    bool sep = (rx1 < b.x0) || (rx0 > b.x1) || (ry1 < b.y0) || (ry0 > b.y1);
    const std::array<core::Point2D, 4> bc{{{b.x0, b.y0}, {b.x1, b.y0}, {b.x1, b.y1},
      {b.x0, b.y1}}};
    if (!sep) {
      double u0 = 1e18, u1 = -1e18, w0 = 1e18, w1 = -1e18;
      for (const auto & q : bc) {
        const double dx = q.x - p.x;
        const double dy = q.y - p.y;
        const double u = c * dx + s * dy;
        const double w = -s * dx + c * dy;
        u0 = std::min(u0, u);
        u1 = std::max(u1, u);
        w0 = std::min(w0, w);
        w1 = std::max(w1, w);
      }
      sep = (u1 < -kHalfL) || (u0 > kHalfL) || (w1 < -kHalfW) || (w0 > kHalfW);
    }
    if (!sep) {
      if (overlap != nullptr) {
        *overlap = true;
      }
      return 0.0;
    }
    for (const auto & q : rc) {
      best = std::min(
        best, std::hypot(
          std::max({b.x0 - q.x, 0.0, q.x - b.x1}), std::max({b.y0 - q.y, 0.0, q.y - b.y1})));
    }
    for (const auto & q : bc) {
      const double dx = q.x - p.x;
      const double dy = q.y - p.y;
      const double u = c * dx + s * dy;
      const double w = -s * dx + c * dy;
      best = std::min(
        best, std::hypot(std::max(0.0, std::abs(u) - kHalfL), std::max(0.0, std::abs(w) - kHalfW)));
    }
  }
  return best;
}

struct World
{
  std::vector<Box> boxes;
  core::OwnedGrid grid;
  bool lidar{false};
  std::vector<core::Point2D> fp = core::FootprintChecker::rectangle(2 * kHalfL, 2 * kHalfW);

  void markBoxesAndInflate()
  {
    for (const auto & b : boxes) {
      for (int my = 0; my < grid.height; ++my) {
        const double wy = grid.origin_y + (my + 0.5) * kRes;
        if (wy < b.y0 - 1e-9 || wy > b.y1 + 1e-9) {
          continue;
        }
        for (int mx = 0; mx < grid.width; ++mx) {
          const double wx = grid.origin_x + (mx + 0.5) * kRes;
          if (wx >= b.x0 - 1e-9 && wx <= b.x1 + 1e-9) {
            grid.at(mx, my) = core::kLethalObstacle;
          }
        }
      }
    }
    core::inflate(grid, 0.2, 0.8, 3.0);
  }

  // 정적 코스트맵: 경로 외접 사각형 + 5 m
  void buildStatic(const std::vector<core::Pose2D> & path)
  {
    double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
    for (const auto & p : path) {
      x0 = std::min(x0, p.x);
      x1 = std::max(x1, p.x);
      y0 = std::min(y0, p.y);
      y1 = std::max(y1, p.y);
    }
    x0 -= 5.0;
    y0 -= 5.0;
    const int w = static_cast<int>(std::ceil((x1 + 5.0 - x0) / kRes));
    const int h = static_cast<int>(std::ceil((y1 + 5.0 - y0) / kRes));
    grid = core::OwnedGrid(w, h, kRes, 0, x0, y0);
    markBoxesAndInflate();
  }

  // LiDAR 한 스캔으로 로봇 중심 8 × 8 m 창 코스트맵을 새로 만든다 (영속 0 = 최신 스캔만)
  void observe(const core::Pose2D & pose, std::mt19937 & rng)
  {
    const int n = static_cast<int>(std::lround(8.0 / kRes));
    grid = core::OwnedGrid(
      n, n, kRes, 0, std::floor((pose.x - 4.0) / kRes) * kRes,
      std::floor((pose.y - 4.0) / kRes) * kRes);
    const double lx = pose.x + 0.15 * std::cos(pose.theta);
    const double ly = pose.y + 0.15 * std::sin(pose.theta);
    std::normal_distribution<double> nd(0.0, 0.03);
    std::vector<float> ranges(720, std::numeric_limits<float>::infinity());
    const double inc = 2.0 * core::kPi / 720.0;
    std::vector<const Box *> near;
    for (const auto & b : boxes) {
      const double dx = std::max({b.x0 - lx, 0.0, lx - b.x1});
      const double dy = std::max({b.y0 - ly, 0.0, ly - b.y1});
      if (std::hypot(dx, dy) < 6.0) {
        near.push_back(&b);
      }
    }
    for (int i = 0; i < 720; ++i) {
      const double a = pose.theta - core::kPi + i * inc;
      double r = 1e18;
      for (const Box * b : near) {
        r = std::min(r, rayBox(lx, ly, std::cos(a), std::sin(a), *b));
      }
      if (r < 12.0) {
        ranges[static_cast<std::size_t>(i)] = static_cast<float>(r + nd(rng));
      }
    }
    ranges = core::denoiseRanges(ranges, core::ScanDenoiseConfig(), true);
    for (int i = 0; i < 720; ++i) {
      const float r = ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(r) || r > 5.5f || r < 0.1f) {   // obstacle_max_range 5.5
        continue;
      }
      const double a = pose.theta - core::kPi + i * inc;
      int mx = 0;
      int my = 0;
      if (grid.view().worldToMap(lx + r * std::cos(a), ly + r * std::sin(a), mx, my)) {
        grid.at(mx, my) = core::kLethalObstacle;
      }
    }
    core::inflate(grid, 0.2, 0.8, 3.0);
  }

  core::FootprintChecker checker() const
  {
    return core::FootprintChecker(
      grid.view(), fp,
      core::inflationCost(core::FootprintChecker::circumscribedRadius(fp), 0.2, 3.0), true);
  }
};

// 서보 한 축: 순수 지연 → 1차 지연 → 가속 제한 (kinematic_sim.py ServoAxis, tracking_sim 과 같다)
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
  double tau_;
  double a_max_;
  std::vector<double> buf_;
  double value_{0.0};
};

// 플러그인 PlanWindow 와 같다 (새 경로는 앞 400 점, 이후 직전 인덱스부터 200 점에서 최근접)
struct PlanWindow
{
  const std::vector<core::Pose2D> * poses{nullptr};
  std::size_t hint{0};
  bool fresh{true};
  std::vector<core::Pose2D> window(const core::Pose2D & r, double horizon, bool & end_is_goal)
  {
    const auto & P = *poses;
    const std::size_t begin = fresh ? 0 : hint;
    const std::size_t end = std::min(P.size(), begin + (fresh ? 400 : 200));
    double best = 1e18;
    std::size_t bi = begin;
    for (std::size_t i = begin; i < end; ++i) {
      const double d = core::distance(P[i], r);
      if (d < best) {
        best = d;
        bi = i;
      }
    }
    hint = bi;
    fresh = false;
    const std::size_t s = bi > 0 ? bi - 1 : 0;
    std::vector<core::Pose2D> out{P[s]};
    double len = 0.0;
    std::size_t last = s;
    for (std::size_t i = s + 1; i < P.size(); ++i) {
      len += core::distance(P[i - 1], P[i]);
      out.push_back(P[i]);
      last = i;
      if (len > horizon) {
        break;
      }
    }
    end_is_goal = last + 1 == P.size();
    return out;
  }
};

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return -1.0;
  }
  std::sort(v.begin(), v.end());
  const double idx = q * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = std::min(v.size() - 1, lo + 1);
  return v[lo] + (idx - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

struct Options
{
  std::string ctrl{"dwa"};
  std::string arm{"dwa"};
  bool ideal{false};
  bool lidar{false};
  double rate{20.0};
  unsigned seed{1};
  double odom_noise_v{0.0};
  double odom_noise_w{0.0};
  double pose_noise_xy{0.0};
  double pose_noise_yaw{0.0};
  double pose_noise_tau{1.0};
  std::string trace_dir;
  core::DwaConfig dwa;
  DwaExtra dwa_extra;
  core::PurePursuitConfig pp;
  PpExtra pp_extra;
  rx::ResearchConfig research;   // 연구 스위치 (기본 = 끔)
  bool stateful{false};          // 목표 판정 stateful (기본 = 원본 판정)
  std::string reset_log;         // 재설정 기록 CSV (빈 문자열 = 끔)
};

// 연구 키 (contract §1: 플러그인 이름에서 "research." 를 뺀 것) + stateful·reset_log
bool applyResearch(Options & o, const std::string & k, const std::string & v)
{
  const double d = std::atof(v.c_str());
  if (k == "reset_persist_cycles") {
    o.research.reset_persist_cycles = static_cast<int>(d);
  } else if (k == "reset_hard_v") {
    o.research.reset_hard_v = d;
  } else if (k == "jerk_consistent") {
    o.research.jerk_consistent = toBool(v);
  } else if (k == "jerk_lim") {
    o.research.jerk_lim = d;
  } else if (k == "pp_candidate") {
    o.research.pp_candidate = toBool(v);
  } else if (k == "log_resets" || k == "reset_log") {
    o.reset_log = v;
  } else if (k == "stateful") {
    o.stateful = toBool(v);
  } else if (k == "reset_model") {
    o.research.reset_model = toBool(v);
  } else if (k == "reset_model_theta") {
    o.research.reset_model_theta = d;
  } else if (k == "reset_model_theta_w") {
    o.research.reset_model_theta_w = d;
  } else if (k == "model_delay") {
    o.research.model_delay = d;
  } else if (k == "model_tau") {
    o.research.model_tau = d;
  } else if (k == "goal_extension_dist") {
    o.research.goal_extension_dist = d;
  } else if (k == "blocked_detour_shift") {
    o.research.blocked_detour_shift = d;
  } else if (k == "blocked_lookahead") {
    o.research.blocked_lookahead = d;
  } else if (k == "accel_hold") {
    o.research.accel_hold = toBool(v);
  } else {
    return false;
  }
  return true;
}

struct Metrics
{
  bool reached{false};
  std::string fail{"none"};
  double t_reach{-1.0};
  double t_end{0.0};
  double length{0.0};
  double t_pred{0.0};
  double cte_sum[2]{0.0, 0.0};
  double cte_max[2]{0.0, 0.0};
  int cte_n[2]{0, 0};
  double stop_along{0.0};
  double stop_lat{0.0};
  double stop_err{0.0};
  double settle{-1.0};
  double min_clear{1e9};
  bool collision{false};
  double t_v90{-1.0};
  std::vector<double> cmd_acc;
  std::vector<double> cmd_jerk;
  std::vector<double> prof_jerk;
  std::vector<double> cycle_us;
  int n_cycles{0};
  int n_not_found{0};
  int n_exceptions{0};
  int n_horizon_varied{0};
  int n_recentered{0};
  // 연구 계측 (원본 지표에는 영향 없음)
  int n_resets{0};
  int n_resets_hard{0};
  int n_resets_persist{0};
  int n_r1_suppressed{0};
  int n_pp_best{0};
  int n_jerk_applied{0};
  int n_detour{0};
  int n_detour_blocked{0};
  int n_goal_ext{0};
  int n_g1_held{0};
  double t_entry{-1.0};
  double yaw_err_entry{-1.0};
  double t_xy_latch{-1.0};
};

// 곡선 구간 표시: |κ| > 0.1 1/m 정점 ± 0.5 m (closed_loop_eval.py·tracking_sim 과 같다)
std::vector<int> curveLabels(
  const std::vector<core::Pose2D> & path,
  const std::vector<double> & cum)
{
  const std::vector<double> kappa = core::discreteCurvature(path, 0.25);
  std::vector<int> curve(path.size(), 0);
  std::size_t j0 = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    while (j0 < path.size() && cum[j0] < cum[i] - 0.5) {
      ++j0;
    }
    for (std::size_t j = j0; j < path.size() && cum[j] <= cum[i] + 0.5; ++j) {
      if (std::abs(kappa[j]) > 0.1) {
        curve[i] = 1;
        break;
      }
    }
  }
  return curve;
}

struct Rng
{
  std::mt19937 gen;
  std::normal_distribution<double> nd{0.0, 1.0};
  double n(double sigma) {return sigma > 0.0 ? sigma * nd(gen) : 0.0;}
};

// 가우스-마르코프 위치추정 오차 (정상 표준편차 sigma, 상관 시간 tau)
struct GaussMarkov
{
  double x{0.0};
  void step(Rng & r, double sigma, double tau, double dt)
  {
    if (sigma <= 0.0) {
      return;
    }
    const double a = std::exp(-dt / std::max(1e-6, tau));
    x = a * x + std::sqrt(1.0 - a * a) * r.n(sigma);
  }
};

Metrics runScene(const Scene & sc, const Options & o, FILE * trace, rx::ResetLogger * rlog)
{
  Metrics m;
  const auto & path = sc.path;
  const std::vector<double> cum = core::cumulativeLength(path);
  const std::vector<int> curve = curveLabels(path, cum);
  m.length = cum.back();
  core::SpeedProfile pred;
  pred.build(path, core::SpeedProfileConfig());
  m.t_pred = pred.predictTravelTime();

  World world;
  world.boxes = sc.boxes;
  world.lidar = o.lidar && !sc.boxes.empty();
  Rng rng{std::mt19937(o.seed), {}};
  std::mt19937 scan_rng(o.seed * 7919U + 17U);
  if (!world.lidar) {
    world.buildStatic(path);
  }
  const rx::DwaPlannerX dwa(o.dwa, o.research);
  rx::ResearchState rstate;   // 주기 사이 연구 상태 (DwaResultX::next 를 되먹인다)
  rx::GoalCheckEmu goal_check(0.10, 0.05, o.stateful);
  const std::string rlog_tag = sc.id + "," + o.arm + "," + std::to_string(o.seed);
  core::PurePursuit pp(o.pp);
  pp.setPath(path);
  PlanWindow pw;
  pw.poses = &path;

  core::VelocityProfiler prof;
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  core::Pose2D pose = path.front();
  GaussMarkov ex, ey, eth;
  const double dt_c = 1.0 / o.rate;
  const int steps_per_cycle = std::max(1, static_cast<int>(std::lround(dt_c / kDt)));
  double v_cmd = 0.0, w_cmd = 0.0, v_out = 0.0, w_out = 0.0;
  double v_last = 0.0, w_last = 0.0;
  bool has_last = false;
  bool done = false;
  int no_valid = 0;
  double last_valid_t = 0.0;
  core::Pose2D prog_pose = pose;
  double prog_t = 0.0;
  std::size_t cte_hint = 0;
  const double t_max = std::max(60.0, 3.0 * m.t_pred + 30.0);
  double prev_cmd_v = 0.0, prev_cmd_a = 0.0;
  bool have_prev_v = false, have_prev_a = false;
  double prev_prof_v = 0.0, prev_prof_a = 0.0;
  int n_prof = 0;
  double t = 0.0;
  double t_stop_since = -1.0;
  const core::Pose2D goal = path.back();
  for (int k = 0; t < t_max + 10.0; ++k, t += kDt) {
    // 위치추정 오차 (제어기·목표 판정 입력)
    ex.step(rng, o.pose_noise_xy, o.pose_noise_tau, kDt);
    ey.step(rng, o.pose_noise_xy, o.pose_noise_tau, kDt);
    eth.step(rng, o.pose_noise_yaw, o.pose_noise_tau, kDt);
    if (world.lidar && k % 10 == 0) {
      world.observe(pose, scan_rng);
    }
    if (k % steps_per_cycle == 0 && !done) {
      const core::Pose2D est{pose.x + ex.x, pose.y + ey.x, core::wrapAngle(pose.theta + eth.x)};
      const double v_meas = (o.ideal ? v_cmd : sv.value_) + rng.n(o.odom_noise_v);
      const double w_meas = (o.ideal ? w_cmd : sw.value_) + rng.n(o.odom_noise_w);
      const double dg = core::distance(est, goal);
      // SimpleGoalChecker (xy 0.10, yaw 0.05). stateful=0 이면 원본 조건 그대로다.
      const bool latched_before = goal_check.xyLatched();
      const bool goal_ok = goal_check.reached(dg, core::wrapAngle(goal.theta - est.theta));
      if (!latched_before && goal_check.xyLatched()) {
        m.t_xy_latch = t;
      }
      if (goal_ok) {
        done = true;
        m.reached = true;
        m.t_reach = t;
        v_cmd = w_cmd = 0.0;
      } else if (t > t_max) {
        done = true;
        m.fail = "timeout";
        v_cmd = w_cmd = 0.0;
      } else {
        // 진행 검사 (SimpleProgressChecker 0.3 m / 20 s)
        if (core::distance(est, prog_pose) > 0.3) {
          prog_pose = est;
          prog_t = t;
        } else if (t - prog_t > 20.0) {
          done = true;
          m.fail = "no_progress";
          v_cmd = w_cmd = 0.0;
        }
      }
      if (!done) {
        const auto t0 = std::chrono::steady_clock::now();
        bool exception = false;
        if (o.ctrl == "pp") {
          const double v_now = (has_last && std::abs(v_last - v_meas) <=
            o.pp_extra.velocity_reset_threshold) ? v_last : v_meas;
          const core::PurePursuitOutput out = pp.compute(est, v_now);
          bool hit = false;
          if (o.pp_extra.use_collision_detection && out.mode == core::PurePursuitMode::kTrack) {
            const auto chk = world.checker();
            const double T = std::max(
              o.pp_extra.collision_check_time,
              std::abs(out.v) / std::max(1e-3, o.pp.max_linear_decel));
            for (int j = 1; j <= static_cast<int>(std::ceil(T / 0.1)); ++j) {
              if (chk.collides(core::integrateArc(est, out.v, out.w, j * 0.1))) {
                hit = true;
                break;
              }
            }
          }
          if (hit || out.mode == core::PurePursuitMode::kNoPath) {
            exception = true;
            has_last = false;
          } else {
            v_cmd = out.v;
            w_cmd = out.w;
            v_last = out.v;
            has_last = true;
          }
        } else {
          bool end_is_goal = false;
          const std::vector<core::Pose2D> win =
            pw.window(est, o.dwa_extra.path_horizon, end_is_goal);
          rx::DwaInputX in;
          in.pose = est;
          in.v_meas = v_meas;
          in.w_meas = w_meas;
          in.has_last = has_last;
          in.v_last = v_last;
          in.w_last = w_last;
          in.path = &win;
          in.path_end_is_goal = end_is_goal;
          in.research = rstate;
          const auto chk = world.checker();
          const auto view = world.grid.view();
          const rx::DwaResultX r = dwa.compute(
            in, [&chk](const core::Pose2D & p) {return chk.cost(p);},
            [view](double x, double y) {
              const uint8_t c = view.costAtWorld(x, y);
              return c == core::kNoInformation ? 0.0 : static_cast<double>(c);
            });
          m.n_horizon_varied += dwa.simTime(r.window.v_hi) > o.dwa.sim_time_min + 1e-12;
          m.n_recentered += r.n_recentered > 0;
          rstate = r.next;
          m.n_g1_held += r.center.accel_held;
          if (r.center.reason != rx::ResetReason::kNone) {
            ++m.n_resets;
            m.n_resets_hard += r.center.reason == rx::ResetReason::kHard;
            m.n_resets_persist += r.center.reason == rx::ResetReason::kPersist;
            if (rlog != nullptr) {
              rlog->write(
                t, in, r.center, r.d_goal, o.dwa.window_reset_v, o.dwa.window_reset_w,
                m.n_g1_held, rlog_tag);
            }
          }
          m.n_r1_suppressed += r.center.suppressed;
          m.n_pp_best += r.pp_selected;
          m.n_jerk_applied += r.jerk_applied;
          m.n_detour += r.detour.active;
          m.n_detour_blocked += r.detour.blocked;
          m.n_goal_ext += r.goal_ext;
          if (!r.found) {
            ++m.n_not_found;
            ++no_valid;
          } else {
            no_valid = 0;
          }
          if (!r.found && no_valid >= o.dwa_extra.no_valid_patience && std::abs(v_meas) < 0.05) {
            no_valid = 0;
            has_last = false;
            exception = true;
          } else {
            has_last = true;
            v_last = r.v;
            w_last = r.w;
            v_cmd = r.v;
            w_cmd = r.w;
          }
        }
        m.cycle_us.push_back(
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
        ++m.n_cycles;
        if (exception) {
          ++m.n_exceptions;
          if (t - last_valid_t > 1.0) {   // controller_server failure_tolerance
            done = true;
            m.fail = "controller_abort";
            v_cmd = w_cmd = 0.0;
          }
        } else {
          last_valid_t = t;
        }
        if (!done) {
          if (have_prev_v) {
            const double a = (v_cmd - prev_cmd_v) / dt_c;
            m.cmd_acc.push_back(std::abs(a));
            if (have_prev_a) {
              m.cmd_jerk.push_back(std::abs((a - prev_cmd_a) / dt_c));
            }
            prev_cmd_a = a;
            have_prev_a = true;
          }
          prev_cmd_v = v_cmd;
          have_prev_v = true;
        }
      }
    }
    double v = 0.0;
    double w = 0.0;
    if (o.ideal) {
      v = v_cmd;
      w = w_cmd;
    } else {
      if (k % 2 == 0) {
        const double fb_v = sv.value_ + rng.n(o.odom_noise_v);
        const double fb_w = sw.value_ + rng.n(o.odom_noise_w);
        const core::ProfilerOutput po = prof.update(v_cmd, w_cmd, true, fb_v, fb_w, 0.02);
        v_out = po.v;
        w_out = po.w;
        if (n_prof >= 1) {
          const double a = (v_out - prev_prof_v) / 0.02;
          if (n_prof >= 2) {
            m.prof_jerk.push_back(std::abs((a - prev_prof_a) / 0.02));
          }
          prev_prof_a = a;
        }
        prev_prof_v = v_out;
        ++n_prof;
      }
      v = sv.step(v_out);
      w = sw.step(w_out);
    }
    pose = core::integrateArc(pose, v, w, kDt);
    bool ov = false;
    const double clr = world.boxes.empty() ? 1e9 : trueClearance(pose, world.boxes, &ov);
    m.min_clear = std::min(m.min_clear, clr);
    m.collision = m.collision || ov;
    if (m.t_v90 < 0.0 && v >= 0.9 * o.dwa.limits.max_vel_x) {
      m.t_v90 = t + kDt;
    }
    if (m.t_entry < 0.0 && core::distance(pose, goal) <= 0.10) {   // 첫 진입 (참값)
      m.t_entry = t + kDt;
      m.yaw_err_entry = std::abs(core::wrapAngle(goal.theta - pose.theta));
    }
    if (!done) {
      // 참값 CTE: 직전 투영 근처 창에서만 찾는다 (되돌아오는 경로의 다른 다리로 튀지 않게)
      const std::size_t b = cte_hint > 10 ? cte_hint - 10 : 0;
      const std::size_t e = std::min(path.size(), cte_hint + 80);
      const core::Projection pr = core::projectOntoPath(path, {pose.x, pose.y}, b, e, &cum);
      cte_hint = pr.segment;
      const int seg = curve[std::min(path.size() - 1, pr.segment + (pr.t > 0.5 ? 1 : 0))];
      m.cte_sum[seg] += std::abs(pr.cte);
      m.cte_max[seg] = std::max(m.cte_max[seg], std::abs(pr.cte));
      ++m.cte_n[seg];
    }
    if (trace != nullptr && k % 2 == 0) {
      std::fprintf(
        trace, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", t + kDt, pose.x, pose.y,
        pose.theta, v, w, v_cmd, w_cmd, v_out, done ? 1 : 0);
    }
    if (done) {
      const bool still = std::abs(v) < 1e-3 && std::abs(w) < 1e-3 &&
        (o.ideal || (std::abs(v_out) < 1e-3 && std::abs(w_out) < 1e-3));
      if (still && t_stop_since < 0.0) {
        t_stop_since = t + kDt;
      }
      if (still || t > (m.reached ? m.t_reach : t) + 15.0) {
        break;
      }
    }
  }
  m.t_end = t;
  const double c = std::cos(goal.theta);
  const double s = std::sin(goal.theta);
  m.stop_along = (pose.x - goal.x) * c + (pose.y - goal.y) * s;
  m.stop_lat = -(pose.x - goal.x) * s + (pose.y - goal.y) * c;
  m.stop_err = std::hypot(pose.x - goal.x, pose.y - goal.y);
  if (m.reached && t_stop_since >= 0.0) {
    m.settle = t_stop_since - m.t_reach;
  }
  return m;
}

void writeJson(FILE * f, const Scene & sc, const Options & o, const Metrics & m)
{
  auto mean = [&m](int s) {return m.cte_n[s] > 0 ? m.cte_sum[s] / m.cte_n[s] : -1.0;};
  auto mx = [&m](int s) {return m.cte_n[s] > 0 ? m.cte_max[s] : -1.0;};
  const int n_all = m.cte_n[0] + m.cte_n[1];
  const double mean_all = n_all > 0 ? (m.cte_sum[0] + m.cte_sum[1]) / n_all : -1.0;
  double us_mean = 0.0;
  for (double u : m.cycle_us) {
    us_mean += u;
  }
  us_mean /= std::max<std::size_t>(1, m.cycle_us.size());
  const bool success = m.reached && !m.collision;
  std::fprintf(
    f,
    "{\"scene\": \"%s\", \"family\": \"%s\", \"arm\": \"%s\", \"ctrl\": \"%s\", "
    "\"plant\": \"%s\", \"rate\": %.1f, \"lidar\": %d, \"seed\": %u, "
    "\"odom_noise_v\": %.4f, \"odom_noise_w\": %.4f, \"pose_noise_xy\": %.4f, "
    "\"pose_noise_yaw\": %.4f, "
    "\"success\": %d, \"reached\": %d, \"fail\": \"%s\", \"collision\": %d, "
    "\"t_reach\": %.3f, \"t_end\": %.3f, \"length\": %.3f, \"t_pred\": %.3f, "
    "\"cte_straight_mean\": %.5f, \"cte_straight_max\": %.5f, "
    "\"cte_curve_mean\": %.5f, \"cte_curve_max\": %.5f, \"cte_mean\": %.5f, \"cte_max\": %.5f, "
    "\"stop_along\": %.4f, \"stop_lat\": %.4f, \"stop_err\": %.4f, \"settle\": %.3f, "
    "\"min_clear\": %.4f, \"t_v90\": %.3f, "
    "\"cmd_acc_p99\": %.4f, \"cmd_acc_max\": %.4f, \"cmd_jerk_p99\": %.4f, \"cmd_jerk_max\": %.4f, "
    "\"prof_jerk_p99\": %.4f, \"prof_jerk_max\": %.4f, "
    "\"cycle_us_mean\": %.2f, \"cycle_us_p95\": %.2f, \"cycle_us_max\": %.2f, "
    "\"n_cycles\": %d, \"n_not_found\": %d, \"n_exceptions\": %d, \"n_horizon_varied\": %d, "
    "\"n_recentered\": %d, "
    "\"stateful\": %d, \"reset_persist_cycles\": %d, \"reset_hard_v\": %.3f, "
    "\"jerk_consistent\": %d, \"jerk_lim\": %.3f, \"pp_candidate\": %d, "
    "\"n_resets\": %d, \"n_resets_hard\": %d, \"n_resets_persist\": %d, "
    "\"n_r1_suppressed\": %d, \"n_pp_best\": %d, \"n_jerk_applied\": %d, "
    "\"t_xy_latch\": %.3f, "
    "\"reset_model\": %d, \"reset_model_theta\": %.3f, \"goal_extension_dist\": %.3f, "
    "\"blocked_detour_shift\": %.3f, \"n_detour\": %d, \"n_detour_blocked\": %d, "
    "\"n_goal_ext\": %d, \"t_entry\": %.3f, \"yaw_err_entry\": %.5f, "
    "\"accel_hold\": %d, \"n_g1_held\": %d}\n",
    sc.id.c_str(), sc.family.c_str(), o.arm.c_str(), o.ctrl.c_str(), o.ideal ? "ideal" : "chain",
    o.rate, o.lidar ? 1 : 0, o.seed, o.odom_noise_v, o.odom_noise_w, o.pose_noise_xy,
    o.pose_noise_yaw, success ? 1 : 0, m.reached ? 1 : 0, m.fail.c_str(), m.collision ? 1 : 0,
    m.t_reach, m.t_end, m.length, m.t_pred, mean(0), mx(0), mean(1), mx(1), mean_all,
    std::max(mx(0), mx(1)), m.stop_along, m.stop_lat, m.stop_err, m.settle,
    m.min_clear > 1e8 ? -1.0 : m.min_clear, m.t_v90,
    percentile(m.cmd_acc, 0.99), percentile(m.cmd_acc, 1.0), percentile(m.cmd_jerk, 0.99),
    percentile(m.cmd_jerk, 1.0), percentile(m.prof_jerk, 0.99), percentile(m.prof_jerk, 1.0),
    us_mean, percentile(m.cycle_us, 0.95), percentile(m.cycle_us, 1.0), m.n_cycles,
    m.n_not_found, m.n_exceptions, m.n_horizon_varied, m.n_recentered,
    o.stateful ? 1 : 0, o.research.reset_persist_cycles, o.research.reset_hard_v,
    o.research.jerk_consistent ? 1 : 0, o.research.jerk_lim, o.research.pp_candidate ? 1 : 0,
    m.n_resets, m.n_resets_hard, m.n_resets_persist, m.n_r1_suppressed, m.n_pp_best,
    m.n_jerk_applied, m.t_xy_latch,
    o.research.reset_model ? 1 : 0, o.research.reset_model_theta, o.research.goal_extension_dist,
    o.research.blocked_detour_shift, m.n_detour, m.n_detour_blocked, m.n_goal_ext, m.t_entry,
    m.yaw_err_entry, o.research.accel_hold ? 1 : 0, m.n_g1_held);
}

void applyKey(Options & o, const std::string & k, const std::string & v)
{
  const double d = std::atof(v.c_str());
  if (k == "ctrl") {
    o.ctrl = v;
  } else if (k == "arm") {
    o.arm = v;
  } else if (k == "plant") {
    o.ideal = v == "ideal";
  } else if (k == "lidar") {
    o.lidar = toBool(v);
  } else if (k == "rate") {
    o.rate = d;
  } else if (k == "seed") {
    o.seed = static_cast<unsigned>(d);
  } else if (k == "odom_noise_v") {
    o.odom_noise_v = d;
  } else if (k == "odom_noise_w") {
    o.odom_noise_w = d;
  } else if (k == "pose_noise_xy") {
    o.pose_noise_xy = d;
  } else if (k == "pose_noise_yaw") {
    o.pose_noise_yaw = d;
  } else if (k == "pose_noise_tau") {
    o.pose_noise_tau = d;
  } else if (k == "trace_dir") {
    o.trace_dir = v;
  } else if (!applyDwa(o.dwa, o.dwa_extra, k, v) && !applyPp(o.pp, o.pp_extra, k, v)) {
    if (applyResearch(o, k, v)) {
      return;
    }
    std::fprintf(stderr, "chain_sim_x: 알 수 없는 키 '%s' (무시하지 않는다)\n", k.c_str());
    std::exit(2);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(
      stderr, "사용: chain_sim_x <scenes.txt> <out.jsonl> [key=value ...] [cfg=<file>]\n");
    return 2;
  }
  // rate 는 control_period 를 정하므로 먼저 읽는다
  double rate = 20.0;
  std::vector<std::pair<std::string, std::string>> kvs;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    const auto eq = a.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string k = a.substr(0, eq);
    const std::string v = a.substr(eq + 1);
    if (k == "cfg") {   // "key value" 또는 "key: value" 줄, # 주석
      std::ifstream cf(v);
      std::string line;
      while (std::getline(cf, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) {
          line = line.substr(0, hash);
        }
        std::replace(line.begin(), line.end(), ':', ' ');
        std::istringstream ls(line);
        std::string ck, cv;
        if (ls >> ck >> cv) {
          kvs.emplace_back(ck, cv);
        }
      }
    } else {
      kvs.emplace_back(k, v);
    }
  }
  for (const auto & [k, v] : kvs) {
    if (k == "rate") {
      rate = std::atof(v.c_str());
    }
  }
  Options o;
  o.rate = rate;
  o.dwa = dwaPluginDefaults(rate);
  for (const auto & [k, v] : kvs) {
    applyKey(o, k, v);
  }
  const std::vector<Scene> scenes = readScenes(argv[1]);
  FILE * out = std::fopen(argv[2], "a");
  if (out == nullptr) {
    std::fprintf(stderr, "chain_sim_x: %s 를 열 수 없다\n", argv[2]);
    return 2;
  }
  rx::ResetLogger rlog;
  if (!o.reset_log.empty() && !rlog.open(o.reset_log, "scene,arm,seed")) {
    std::fprintf(stderr, "chain_sim_x: 재설정 기록 %s 를 열 수 없다\n", o.reset_log.c_str());
    std::fclose(out);
    return 2;
  }
  for (const Scene & sc : scenes) {
    FILE * trace = nullptr;
    if (!o.trace_dir.empty()) {
      const std::string tp = o.trace_dir + "/" + o.arm + "__" + sc.id + "__s" +
        std::to_string(o.seed) + ".csv";
      trace = std::fopen(tp.c_str(), "w");
      if (trace != nullptr) {
        std::fprintf(trace, "t,x,y,theta,v,w,v_cmd,w_cmd,v_prof,done\n");
      }
    }
    const Metrics m = runScene(sc, o, trace, rlog.isOpen() ? &rlog : nullptr);
    if (trace != nullptr) {
      std::fclose(trace);
    }
    writeJson(out, sc, o, m);
    std::fflush(out);
  }
  std::fclose(out);
  return 0;
}
