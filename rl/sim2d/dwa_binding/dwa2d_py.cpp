// 팀 DWA 코어(src/amr_navigation/src/core/dwa.cpp)를 2D 시뮬레이터에서 부르는 pybind11 모듈 amr_dwa2d_py.
//
// 원본 코어는 읽기 전용으로 그대로 컴파일한다 (build_dwa2d.sh). 여기에는 ROS 래퍼
// (src/amr_navigation/src/ros/dwa_controller.cpp) 중 2D 에 필요한 부분의 이식만 있다:
//   - PlanWindow::window (dwa_controller.cpp:31-70): 경로 창 (최근접 정점 탐색 + path_horizon)
//   - DWAController::computeVelocityCommands (dwa_controller.cpp:455-534): 입력 조립, 직전 명령 기억,
//     no_valid_patience 예외 조건
//   - 지역 코스트맵 (nav2_params.yaml local_costmap): rolling window 8 × 8 m · 0.025 m, ObstacleLayer
//     (scan_costmap = core::denoiseRanges 게이트 중앙값, 레이트레이스 지우기 6.0 m, 마킹 5.5 m, 풋프린트 지우기),
//     InflationLayer (core::inflate, 내접 0.20 · 반경 0.8 · s 3.0), FootprintChecker (외접 비용)
// 기존 모듈 amr_core_py 와 따로 빌드한다 (그 모듈의 바이트·동작은 바뀌지 않는다).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/scan_denoise.hpp"

namespace py = pybind11;
namespace core = amr_navigation::core;

namespace
{
using U8Array = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;
using F64Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using F32Array = py::array_t<float, py::array::c_style | py::array::forcecast>;
constexpr double kInf = std::numeric_limits<double>::infinity();

// ------------------------------------------------------------------------------------------
// DwaConfig <-> dict (필드 이름 = dwa.hpp 의 멤버 이름, limits.* / weights.* / gate.* 는 접두어)

struct FieldRef
{
  double * d{nullptr};
  int * i{nullptr};
  bool * b{nullptr};
};

std::map<std::string, FieldRef> fieldMap(core::DwaConfig & c)
{
  std::map<std::string, FieldRef> m;
  auto D = [&m](const char * k, double * p) {m[k] = FieldRef{p, nullptr, nullptr};};
  auto I = [&m](const char * k, int * p) {m[k] = FieldRef{nullptr, p, nullptr};};
  auto B = [&m](const char * k, bool * p) {m[k] = FieldRef{nullptr, nullptr, p};};
  D("limits.max_vel_x", &c.limits.max_vel_x);
  D("limits.min_vel_x", &c.limits.min_vel_x);
  D("limits.max_vel_theta", &c.limits.max_vel_theta);
  D("limits.acc_lim_x", &c.limits.acc_lim_x);
  D("limits.decel_lim_x", &c.limits.decel_lim_x);
  D("limits.acc_lim_theta", &c.limits.acc_lim_theta);
  D("limits.jerk_lim_x", &c.limits.jerk_lim_x);
  D("weights.heading", &c.weights.heading);
  D("weights.clearance", &c.weights.clearance);
  D("weights.velocity", &c.weights.velocity);
  D("weights.path", &c.weights.path);
  D("weights.oscillation", &c.weights.oscillation);
  D("weights.dynamic", &c.weights.dynamic);
  D("weights.off_path", &c.weights.off_path);
  D("weights.escape", &c.weights.escape);
  D("control_period", &c.control_period);
  I("vx_samples", &c.vx_samples);
  I("vth_samples", &c.vth_samples);
  D("sim_dt", &c.sim_dt);
  B("sustained_sampling", &c.sustained_sampling);
  D("jerk_lim_theta", &c.jerk_lim_theta);
  D("sim_time_min", &c.sim_time_min);
  D("sim_time_max", &c.sim_time_max);
  D("commit_time", &c.commit_time);
  D("approach_latency", &c.approach_latency);
  D("window_reset_v", &c.window_reset_v);
  D("window_reset_w", &c.window_reset_w);
  D("heading_lookahead_gain", &c.heading_lookahead_gain);
  D("heading_lookahead_offset", &c.heading_lookahead_offset);
  D("heading_lookahead_min", &c.heading_lookahead_min);
  D("heading_lookahead_max", &c.heading_lookahead_max);
  D("path_band", &c.path_band);
  D("max_path_offset", &c.max_path_offset);
  D("max_path_offset_hard", &c.max_path_offset_hard);
  D("off_path_band", &c.off_path_band);
  D("path_eval_time", &c.path_eval_time);
  D("goal_align_distance", &c.goal_align_distance);
  D("goal_overshoot_margin", &c.goal_overshoot_margin);
  D("oscillation_w_eps", &c.oscillation_w_eps);
  B("use_dynamic_obstacles", &c.use_dynamic_obstacles);
  B("use_velocity_obstacles", &c.use_velocity_obstacles);
  D("prediction_time", &c.prediction_time);
  D("robot_radius", &c.robot_radius);
  D("dynamic_margin", &c.dynamic_margin);
  D("vo_time_horizon", &c.vo_time_horizon);
  D("vo_margin", &c.vo_margin);
  D("vo_max_range", &c.vo_max_range);
  D("dynamic_speed_threshold", &c.dynamic_speed_threshold);
  D("dynamic_steer_gain", &c.dynamic_steer_gain);
  D("vo_time_tie", &c.vo_time_tie);
  B("yield_crossing", &c.yield_crossing);
  D("yield_corridor_margin", &c.yield_corridor_margin);
  D("yield_stop_margin", &c.yield_stop_margin);
  D("yield_clear_margin", &c.yield_clear_margin);
  D("yield_horizon", &c.yield_horizon);
  D("yield_nominal_speed", &c.yield_nominal_speed);
  D("yield_hold_standoff", &c.yield_hold_standoff);
  D("yield_approach_radius", &c.yield_approach_radius);
  D("yield_approach_horizon", &c.yield_approach_horizon);
  D("yield_lookahead", &c.yield_lookahead);
  D("yield_max_zone", &c.yield_max_zone);
  D("yield_escape_speed", &c.yield_escape_speed);
  D("yield_escape_drift", &c.yield_escape_drift);
  D("yield_escape_v_meas", &c.yield_escape_v_meas);
  D("escape_min_speed", &c.escape_min_speed);
  D("escape_spread_min", &c.escape_spread_min);
  D("yield_parallel_cos", &c.yield_parallel_cos);
  D("yield_vacate_margin", &c.yield_vacate_margin);
  B("yield_decision", &c.yield_decision);
  D("yield_body_margin", &c.yield_body_margin);
  D("yield_go_margin", &c.yield_go_margin);
  D("yield_go_clearance", &c.yield_go_clearance);
  D("yield_go_hold_clearance", &c.yield_go_hold_clearance);
  D("yield_go_speed_floor", &c.yield_go_speed_floor);
  D("yield_hold_slack", &c.yield_hold_slack);
  D("yield_decision_min_sin", &c.yield_decision_min_sin);
  B("gate.enable", &c.gate.enable);
  D("gate.pos_err", &c.gate.pos_err);
  D("gate.heading_unc", &c.gate.heading_unc);
  D("gate.v_max", &c.gate.v_max);
  D("gate.delay", &c.gate.delay);
  D("gate.hold_back", &c.gate.hold_back);
  D("gate.turn_time", &c.gate.turn_time);
  D("gate.walk_accel", &c.gate.walk_accel);
  D("gate.vknown", &c.gate.vknown);
  D("gate.static_timeout", &c.gate.static_timeout);
  D("gate.lookahead", &c.gate.lookahead);
  D("gate.group_gap", &c.gate.group_gap);
  D("gate.hold_tol", &c.gate.hold_tol);
  D("gate.hold_threat_horizon", &c.gate.hold_threat_horizon);
  D("gate.hold_exclusion_horizon", &c.gate.hold_exclusion_horizon);
  D("gate.commit_max_run", &c.gate.commit_max_run);
  B("gate.retreat_enable", &c.gate.retreat_enable);
  D("gate.retreat_speed", &c.gate.retreat_speed);
  D("gate.retreat_max", &c.gate.retreat_max);
  B("recenter_narrow", &c.recenter_narrow);
  D("recenter_min_cost", &c.recenter_min_cost);
  D("recenter_max_shift", &c.recenter_max_shift);
  D("recenter_step", &c.recenter_step);
  I("recenter_smooth", &c.recenter_smooth);
  D("recenter_goal_keep", &c.recenter_goal_keep);
  return m;
}

core::DwaConfig configFromDict(const py::dict & d)
{
  core::DwaConfig c;
  auto m = fieldMap(c);
  for (auto item : d) {
    const std::string k = py::str(item.first);
    auto it = m.find(k);
    if (it == m.end()) {
      throw std::runtime_error("DwaConfig 에 없는 키: " + k);
    }
    if (it->second.d != nullptr) {
      *it->second.d = item.second.cast<double>();
    } else if (it->second.i != nullptr) {
      *it->second.i = item.second.cast<int>();
    } else {
      *it->second.b = item.second.cast<bool>();
    }
  }
  return c;
}

py::dict configToDict(core::DwaConfig c)
{
  py::dict d;
  for (const auto & kv : fieldMap(c)) {
    if (kv.second.d != nullptr) {
      d[py::str(kv.first)] = *kv.second.d;
    } else if (kv.second.i != nullptr) {
      d[py::str(kv.first)] = *kv.second.i;
    } else {
      d[py::str(kv.first)] = *kv.second.b;
    }
  }
  return d;
}

std::vector<core::Point2D> toFootprint(const F64Array & fp)
{
  if (fp.ndim() != 2 || fp.shape(1) != 2) {
    throw std::runtime_error("footprint 는 [K, 2] 이어야 한다");
  }
  const auto a = fp.unchecked<2>();
  std::vector<core::Point2D> out;
  for (py::ssize_t i = 0; i < a.shape(0); ++i) {
    out.push_back({a(i, 0), a(i, 1)});
  }
  return out;
}

std::vector<core::Pose2D> toPoses(const F64Array & p)
{
  if (p.ndim() != 2 || p.shape(1) != 3) {
    throw std::runtime_error("poses 는 [K, 3] (x, y, yaw) 이어야 한다");
  }
  const auto a = p.unchecked<2>();
  std::vector<core::Pose2D> out;
  out.reserve(static_cast<std::size_t>(a.shape(0)));
  for (py::ssize_t i = 0; i < a.shape(0); ++i) {
    out.push_back({a(i, 0), a(i, 1), a(i, 2)});
  }
  return out;
}

// 한 걸음 결과 (파이썬으로 돌려줄 값)
struct StepOut
{
  double v{0.0};
  double w{0.0};
  bool found{false};
  bool exception{false};       // no_valid_patience 조건 (ROS 래퍼라면 PlannerException)
  std::size_t n_samples{0};
  std::size_t n_valid{0};
  std::size_t n_collision{0};
  std::size_t n_recentered{0};
  double best_cost{0.0};
  double d_goal{0.0};
  bool end_is_goal{false};
  std::size_t window_len{0};
};

py::dict toDict(const StepOut & s)
{
  py::dict d;
  d["v"] = s.v;
  d["w"] = s.w;
  d["found"] = s.found;
  d["exception"] = s.exception;
  d["n_samples"] = s.n_samples;
  d["n_valid"] = s.n_valid;
  d["n_collision"] = s.n_collision;
  d["n_recentered"] = s.n_recentered;
  d["best_cost"] = s.best_cost;
  d["d_goal"] = std::isfinite(s.d_goal) ? s.d_goal : -1.0;
  d["end_is_goal"] = s.end_is_goal;
  d["window_len"] = s.window_len;
  return d;
}

/// core::DwaPlanner::compute 를 dwa_controller.cpp:487-497 와 같은 람다로 부른다.
core::DwaResult runCore(
  const core::DwaPlanner & planner, const core::DwaInput & in, const core::CostGrid & grid,
  const std::vector<core::Point2D> & footprint, uint8_t circ_cost, bool allow_unknown)
{
  const core::FootprintChecker checker(grid, footprint, circ_cost, allow_unknown);
  return planner.compute(
    in, [&checker](const core::Pose2D & p) {return checker.cost(p);},
    [&grid](double x, double y) {
      const uint8_t c = grid.costAtWorld(x, y);
      return c == core::kNoInformation ? 0.0 : static_cast<double>(c);
    });
}

// ------------------------------------------------------------------------------------------
// 지역 코스트맵 (rolling window)

struct CostmapParams
{
  double size_m{8.0};
  double resolution{0.025};
  double obstacle_max_range{5.5};
  double raytrace_max_range{6.0};
  double inscribed_radius{0.20};
  double inflation_radius{0.8};
  double cost_scaling_factor{3.0};
  bool footprint_clearing{true};
  bool denoise{true};
  int denoise_half_window{5};
  double denoise_range_gate{0.15};
  int denoise_min_support{3};
};

class RollingCostmap
{
public:
  explicit RollingCostmap(const CostmapParams & p)
  : p_(p)
  {
    n_ = static_cast<int>(std::lround(p.size_m / p.resolution));
    obstacle_.assign(static_cast<std::size_t>(n_) * n_, core::kFreeSpace);
    master_ = core::OwnedGrid(n_, n_, p.resolution, core::kFreeSpace, 0.0, 0.0);
  }

  void reset()
  {
    std::fill(obstacle_.begin(), obstacle_.end(), core::kFreeSpace);
    std::fill(master_.cells.begin(), master_.cells.end(), core::kFreeSpace);
    have_origin_ = false;
    ox_ = oy_ = 0.0;
    master_.origin_x = master_.origin_y = 0.0;
  }

  /// Nav2 Costmap2D::updateOrigin (rolling window: 새 원점 = 로봇 − 크기/2, 셀 격자에 맞춘 이동, 겹친 셀 보존)
  void updateOrigin(double new_ox, double new_oy)
  {
    if (!have_origin_) {
      ox_ = new_ox;
      oy_ = new_oy;
      have_origin_ = true;
      return;
    }
    const int cox = static_cast<int>((new_ox - ox_) / p_.resolution);
    const int coy = static_cast<int>((new_oy - oy_) / p_.resolution);
    if (cox == 0 && coy == 0) {
      return;
    }
    std::vector<uint8_t> next(obstacle_.size(), core::kFreeSpace);
    for (int y = 0; y < n_; ++y) {
      const int sy = y + coy;
      if (sy < 0 || sy >= n_) {
        continue;
      }
      for (int x = 0; x < n_; ++x) {
        const int sx = x + cox;
        if (sx < 0 || sx >= n_) {
          continue;
        }
        next[static_cast<std::size_t>(y) * n_ + x] = obstacle_[static_cast<std::size_t>(sy) * n_ + sx];
      }
    }
    obstacle_.swap(next);
    ox_ += cox * p_.resolution;
    oy_ += coy * p_.resolution;
  }

  bool worldToMap(double wx, double wy, int & mx, int & my) const
  {
    if (wx < ox_ || wy < oy_) {
      return false;
    }
    mx = static_cast<int>((wx - ox_) / p_.resolution);
    my = static_cast<int>((wy - oy_) / p_.resolution);
    return mx < n_ && my < n_;
  }

  /// Nav2 Costmap2D::raytraceLine + bresenham2D (끝 셀 포함, max_length 셀 수로 자름)
  void raytraceClear(int x0, int y0, int x1, int y1, unsigned int max_length)
  {
    const int dx = x1 - x0;
    const int dy = y1 - y0;
    const unsigned int adx = static_cast<unsigned int>(std::abs(dx));
    const unsigned int ady = static_cast<unsigned int>(std::abs(dy));
    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
    const double scale = dist == 0.0 ? 1.0 : std::min(1.0, max_length / dist);
    int x = x0;
    int y = y0;
    auto clear = [&](int cx, int cy) {
        if (cx >= 0 && cy >= 0 && cx < n_ && cy < n_) {
          obstacle_[static_cast<std::size_t>(cy) * n_ + cx] = core::kFreeSpace;
        }
      };
    if (adx >= ady) {
      int err = static_cast<int>(adx / 2);
      const unsigned int end = std::min(static_cast<unsigned int>(scale * adx), adx);
      for (unsigned int i = 0; i < end; ++i) {
        clear(x, y);
        x += sx;
        err += static_cast<int>(ady);
        if (static_cast<unsigned int>(err) >= adx) {
          y += sy;
          err -= static_cast<int>(adx);
        }
      }
      clear(x, y);
    } else {
      int err = static_cast<int>(ady / 2);
      const unsigned int end = std::min(static_cast<unsigned int>(scale * ady), ady);
      for (unsigned int i = 0; i < end; ++i) {
        clear(x, y);
        y += sy;
        err += static_cast<int>(adx);
        if (static_cast<unsigned int>(err) >= ady) {
          x += sx;
          err -= static_cast<int>(ady);
        }
      }
      clear(x, y);
    }
  }

  /// 스캔 한 장으로 갱신: (1) 원점 이동 (2) 레이트레이스 지우기 (3) 마킹 (4) 풋프린트 지우기 (5) 팽창.
  /// sensor (sx, sy, syaw) = LiDAR 원점 자세, robot = base_link 자세 (rolling 원점·풋프린트 지우기).
  void update(
    double sx, double sy, double syaw, const core::Pose2D & robot,
    std::vector<float> ranges, double angle_min, double angle_inc, double range_min,
    double range_max, const std::vector<core::Point2D> & footprint)
  {
    const double half = 0.5 * n_ * p_.resolution;
    updateOrigin(robot.x - half, robot.y - half);
    if (p_.denoise) {
      core::ScanDenoiseConfig dc;
      dc.half_window = p_.denoise_half_window;
      dc.range_gate = p_.denoise_range_gate;
      dc.min_support = p_.denoise_min_support;
      ranges = core::denoiseRanges(ranges, dc, true);
    }
    // 점 목록 (laser_geometry: 유효 범위 밖·비유한 빔은 버린다 — inf_is_valid False)
    std::vector<std::pair<double, double>> pts;
    pts.reserve(ranges.size());
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double r = ranges[i];
      if (!std::isfinite(r) || r < range_min || r >= range_max) {
        continue;
      }
      const double a = syaw + angle_min + static_cast<double>(i) * angle_inc;
      pts.emplace_back(sx + r * std::cos(a), sy + r * std::sin(a));
    }
    // (2) ObstacleLayer::raytraceFreespace — 원점 셀에서 끝점까지, 지도 밖 끝점은 경계로 자른다
    int x0 = 0;
    int y0 = 0;
    const double map_end_x = ox_ + n_ * p_.resolution;
    const double map_end_y = oy_ + n_ * p_.resolution;
    if (worldToMap(sx, sy, x0, y0)) {
      // Costmap2D::cellDistance = ceil(거리 / 해상도)
      const unsigned int cell_max =
        static_cast<unsigned int>(std::max(0.0, std::ceil(p_.raytrace_max_range / p_.resolution)));
      for (const auto & q : pts) {
        double wx = q.first;
        double wy = q.second;
        const double a = wx - sx;
        const double b = wy - sy;
        if (wx < ox_) {
          const double t = (ox_ - sx) / a;
          wx = ox_;
          wy = sy + b * t;
        }
        if (wy < oy_) {
          const double t = (oy_ - sy) / b;
          wx = sx + a * t;
          wy = oy_;
        }
        if (wx > map_end_x) {
          const double t = (map_end_x - sx) / a;
          wx = map_end_x - 0.001;
          wy = sy + b * t;
        }
        if (wy > map_end_y) {
          const double t = (map_end_y - sy) / b;
          wx = sx + a * t;
          wy = map_end_y - 0.001;
        }
        int x1 = 0;
        int y1 = 0;
        if (!worldToMap(wx, wy, x1, y1)) {
          continue;
        }
        raytraceClear(x0, y0, x1, y1, cell_max);
      }
    }
    // (3) 마킹 (센서 원점에서 obstacle_max_range 안)
    const double mr2 = p_.obstacle_max_range * p_.obstacle_max_range;
    for (const auto & q : pts) {
      const double dx = q.first - sx;
      const double dy = q.second - sy;
      if (dx * dx + dy * dy >= mr2) {
        continue;
      }
      int mx = 0;
      int my = 0;
      if (worldToMap(q.first, q.second, mx, my)) {
        obstacle_[static_cast<std::size_t>(my) * n_ + mx] = core::kLethalObstacle;
      }
    }
    // (4) 풋프린트 지우기 (ObstacleLayer footprint_clearing_enabled 기본 true) — 셀 중심이 다각형 안
    if (p_.footprint_clearing && footprint.size() >= 3) {
      clearFootprint(robot, footprint);
    }
    // (5) master = 장애물 층 → 팽창 (InflationLayer)
    master_.origin_x = ox_;
    master_.origin_y = oy_;
    master_.cells = obstacle_;
    core::inflate(master_, p_.inscribed_radius, p_.inflation_radius, p_.cost_scaling_factor);
  }

  void clearFootprint(const core::Pose2D & robot, const std::vector<core::Point2D> & fp)
  {
    const double c = std::cos(robot.theta);
    const double s = std::sin(robot.theta);
    std::vector<core::Point2D> w;
    double xmin = kInf, xmax = -kInf, ymin = kInf, ymax = -kInf;
    for (const auto & p : fp) {
      const core::Point2D q{robot.x + c * p.x - s * p.y, robot.y + s * p.x + c * p.y};
      xmin = std::min(xmin, q.x);
      xmax = std::max(xmax, q.x);
      ymin = std::min(ymin, q.y);
      ymax = std::max(ymax, q.y);
      w.push_back(q);
    }
    int ix0 = 0, iy0 = 0, ix1 = 0, iy1 = 0;
    ix0 = static_cast<int>(std::floor((xmin - ox_) / p_.resolution));
    iy0 = static_cast<int>(std::floor((ymin - oy_) / p_.resolution));
    ix1 = static_cast<int>(std::floor((xmax - ox_) / p_.resolution));
    iy1 = static_cast<int>(std::floor((ymax - oy_) / p_.resolution));
    for (int y = std::max(0, iy0); y <= std::min(n_ - 1, iy1); ++y) {
      for (int x = std::max(0, ix0); x <= std::min(n_ - 1, ix1); ++x) {
        const double px = ox_ + (x + 0.5) * p_.resolution;
        const double py = oy_ + (y + 0.5) * p_.resolution;
        bool neg = true;
        bool pos = true;
        for (std::size_t i = 0; i < w.size(); ++i) {
          const core::Point2D & a = w[i];
          const core::Point2D & b = w[(i + 1) % w.size()];
          const double cr = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
          neg = neg && cr <= 0.0;
          pos = pos && cr >= 0.0;
        }
        if (neg || pos) {   // 볼록 다각형 안 (꼭짓점 순서 무관)
          obstacle_[static_cast<std::size_t>(y) * n_ + x] = core::kFreeSpace;
        }
      }
    }
  }

  core::CostGrid view() const {return master_.view();}
  const core::OwnedGrid & master() const {return master_;}
  void setMaster(const core::OwnedGrid & g)
  {
    master_ = g;
    ox_ = g.origin_x;
    oy_ = g.origin_y;
    have_origin_ = true;
  }
  const std::vector<uint8_t> & obstacle() const {return obstacle_;}
  int size() const {return n_;}
  double originX() const {return ox_;}
  double originY() const {return oy_;}

private:
  CostmapParams p_;
  int n_{0};
  std::vector<uint8_t> obstacle_;
  core::OwnedGrid master_;
  bool have_origin_{false};
  double ox_{0.0};
  double oy_{0.0};
};

// ------------------------------------------------------------------------------------------
// env 하나의 DWA (ROS 래퍼의 상태 + 지역 코스트맵)

class Dwa2D
{
public:
  Dwa2D(
    const py::dict & cfg, const py::dict & cm, const F64Array & footprint, double path_horizon,
    int no_valid_patience, bool allow_unknown)
  : planner_(configFromDict(cfg)), footprint_(toFootprint(footprint)), path_horizon_(path_horizon),
    no_valid_limit_(no_valid_patience), allow_unknown_(allow_unknown), costmap_(cmParams(cm))
  {
    // dwa_controller.cpp ros_utils circumscribedCost: InflationLayer::computeCost(r_circ / res)
    const double r = core::FootprintChecker::circumscribedRadius(footprint_);
    circ_cost_ = core::inflationCost(r, cmp_.inscribed_radius, cmp_.cost_scaling_factor);
  }

  static CostmapParams cmParamsStatic(const py::dict & cm)
  {
    CostmapParams p;
    for (auto item : cm) {
      const std::string k = py::str(item.first);
      if (k == "size_m") {p.size_m = item.second.cast<double>();
      } else if (k == "resolution") {p.resolution = item.second.cast<double>();
      } else if (k == "obstacle_max_range") {p.obstacle_max_range = item.second.cast<double>();
      } else if (k == "raytrace_max_range") {p.raytrace_max_range = item.second.cast<double>();
      } else if (k == "inscribed_radius") {p.inscribed_radius = item.second.cast<double>();
      } else if (k == "inflation_radius") {p.inflation_radius = item.second.cast<double>();
      } else if (k == "cost_scaling_factor") {p.cost_scaling_factor = item.second.cast<double>();
      } else if (k == "footprint_clearing") {p.footprint_clearing = item.second.cast<bool>();
      } else if (k == "denoise") {p.denoise = item.second.cast<bool>();
      } else if (k == "denoise_half_window") {p.denoise_half_window = item.second.cast<int>();
      } else if (k == "denoise_range_gate") {p.denoise_range_gate = item.second.cast<double>();
      } else if (k == "denoise_min_support") {p.denoise_min_support = item.second.cast<int>();
      } else {
        throw std::runtime_error("코스트맵 설정에 없는 키: " + k);
      }
    }
    return p;
  }

  /// 새 경로 (PlanWindow::setPlan) + activate() 와 같은 기억 지우기. 코스트맵도 비운다 (새 에피소드).
  void reset(const F64Array & poses)
  {
    plan_ = toPoses(poses);
    hint_ = 0;
    fresh_ = true;
    has_last_ = false;
    v_last_ = w_last_ = 0.0;
    no_valid_count_ = 0;
    yield_limit_last_ = kInf;
    yield_decision_last_ = 0;
    costmap_.reset();
  }

  /// PlanWindow::window (dwa_controller.cpp:31-70) — T = 항등 (2D 는 map = odom), back_search 0 (게이트 꺼짐)
  std::vector<core::Pose2D> window(const core::Pose2D & robot, bool & end_is_goal)
  {
    std::vector<core::Pose2D> out;
    end_is_goal = false;
    if (plan_.empty()) {
      return out;
    }
    constexpr std::size_t kFreshWindow = 400;
    constexpr std::size_t kTrackWindow = 200;
    const std::size_t begin = fresh_ ? 0 : hint_;
    const std::size_t end = std::min(plan_.size(), begin + (fresh_ ? kFreshWindow : kTrackWindow));
    double best = kInf;
    std::size_t best_i = begin;
    for (std::size_t i = begin; i < end; ++i) {
      const double d = core::distance(plan_[i], robot);
      if (d < best) {
        best = d;
        best_i = i;
      }
    }
    hint_ = best_i;
    fresh_ = false;
    const std::size_t s = best_i > 0 ? best_i - 1 : 0;
    out.push_back(plan_[s]);
    double len = 0.0;
    std::size_t last = s;
    for (std::size_t i = s + 1; i < plan_.size(); ++i) {
      len += core::distance(plan_[i - 1], plan_[i]);
      out.push_back(plan_[i]);
      last = i;
      if (len > path_horizon_) {
        break;
      }
    }
    end_is_goal = last + 1 == plan_.size();
    return out;
  }

  /// computeVelocityCommands (동적 장애물·게이트·적재 없음): 창 → 입력 → 코어 → 직전 명령 기억 · 예외 조건
  StepOut step(double x, double y, double th, double v_meas, double w_meas)
  {
    StepOut o;
    core::DwaInput in;
    in.pose = {x, y, th};
    bool end_is_goal = false;
    const std::vector<core::Pose2D> path = window(in.pose, end_is_goal);
    in.path = &path;
    in.path_end_is_goal = end_is_goal;
    in.v_meas = v_meas;
    in.w_meas = w_meas;
    in.has_last = has_last_;
    in.v_last = v_last_;
    in.w_last = w_last_;
    in.speed_limit = 1e9;   // setSpeedLimit 미사용 (speed_limit_ 초기값)
    in.yield_limit_last = yield_limit_last_;
    in.yield_decision_last = yield_decision_last_;
    const core::DwaResult res =
      runCore(planner_, in, costmap_.view(), footprint_, circ_cost_, allow_unknown_);
    yield_limit_last_ = res.yield.speed_limit;
    yield_decision_last_ = static_cast<int>(res.yield.decision);
    o.v = res.v;
    o.w = res.w;
    o.found = res.found;
    if (!res.found) {
      ++no_valid_count_;
      if (no_valid_count_ >= no_valid_limit_ && std::abs(in.v_meas) < 0.05) {
        no_valid_count_ = 0;
        has_last_ = false;
        o.exception = true;   // 래퍼는 여기서 PlannerException 을 던진다 (명령을 내지 않는다)
        o.v = 0.0;            // controller_server 는 실패 허용 시간 동안 0 속도를 낸다 (publishZeroVelocity)
        o.w = 0.0;
      }
    } else {
      no_valid_count_ = 0;
    }
    if (!o.exception) {
      has_last_ = true;
      v_last_ = res.v;
      w_last_ = res.w;
    }
    o.n_samples = res.n_samples;
    o.n_valid = res.n_valid;
    o.n_collision = res.n_collision;
    o.n_recentered = res.n_recentered;
    o.best_cost = res.best.cost;
    o.d_goal = res.d_goal;
    o.end_is_goal = end_is_goal;
    o.window_len = path.size();
    return o;
  }

  void updateCostmap(
    double sx, double sy, double syaw, double rx, double ry, double rth,
    std::vector<float> ranges, double angle_min, double angle_inc, double range_min,
    double range_max)
  {
    costmap_.update(
      sx, sy, syaw, core::Pose2D{rx, ry, rth}, std::move(ranges), angle_min, angle_inc,
      range_min, range_max, footprint_);
  }

  RollingCostmap & costmap() {return costmap_;}
  py::dict config() const {return configToDict(planner_.config());}
  int circCost() const {return circ_cost_;}

private:
  CostmapParams cmParams(const py::dict & cm)
  {
    cmp_ = cmParamsStatic(cm);
    return cmp_;
  }

  CostmapParams cmp_;
  core::DwaPlanner planner_;
  std::vector<core::Point2D> footprint_;
  double path_horizon_;
  int no_valid_limit_;
  bool allow_unknown_;
  RollingCostmap costmap_;
  uint8_t circ_cost_{0};
  std::vector<core::Pose2D> plan_;
  std::size_t hint_{0};
  bool fresh_{true};
  bool has_last_{false};
  double v_last_{0.0};
  double w_last_{0.0};
  int no_valid_count_{0};
  double yield_limit_last_{kInf};
  int yield_decision_last_{0};
};

template<class F>
void parallelFor(std::size_t n, int threads, F f)
{
  const int nt = std::max(1, std::min<int>(threads, static_cast<int>(n)));
  if (nt <= 1) {
    for (std::size_t i = 0; i < n; ++i) {
      f(i);
    }
    return;
  }
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> pool;
  pool.reserve(static_cast<std::size_t>(nt));
  for (int t = 0; t < nt; ++t) {
    pool.emplace_back(
      [&]() {
        for (std::size_t i = next.fetch_add(1); i < n; i = next.fetch_add(1)) {
          f(i);
        }
      });
  }
  for (auto & th : pool) {
    th.join();
  }
}

/// 여러 env 의 한 걸음 (GIL 해제, 스레드 풀). idx = 계산할 env 번호, poses [n,3], v/w [n] 은 idx 와 같은 순서.
py::dict stepBatch(
  std::vector<Dwa2D *> dwas, const py::array_t<int64_t> & idx, const F64Array & poses,
  const F64Array & v, const F64Array & w, int threads)
{
  const auto I = idx.unchecked<1>();
  const auto P = poses.unchecked<2>();
  const auto V = v.unchecked<1>();
  const auto W = w.unchecked<1>();
  const std::size_t n = static_cast<std::size_t>(I.shape(0));
  std::vector<StepOut> out(n);
  std::vector<std::size_t> ids(n);
  std::vector<double> px(n), py_(n), pt(n), vv(n), ww(n);
  for (std::size_t k = 0; k < n; ++k) {
    ids[k] = static_cast<std::size_t>(I(static_cast<py::ssize_t>(k)));
    if (ids[k] >= dwas.size()) {
      throw std::runtime_error("idx 범위 밖");
    }
    px[k] = P(static_cast<py::ssize_t>(k), 0);
    py_[k] = P(static_cast<py::ssize_t>(k), 1);
    pt[k] = P(static_cast<py::ssize_t>(k), 2);
    vv[k] = V(static_cast<py::ssize_t>(k));
    ww[k] = W(static_cast<py::ssize_t>(k));
  }
  {
    py::gil_scoped_release release;
    parallelFor(
      n, threads, [&](std::size_t k) {
        out[k] = dwas[ids[k]]->step(px[k], py_[k], pt[k], vv[k], ww[k]);
      });
  }
  py::array_t<double> ov(static_cast<py::ssize_t>(n)), ow(static_cast<py::ssize_t>(n));
  py::array_t<bool> of(static_cast<py::ssize_t>(n)), oe(static_cast<py::ssize_t>(n));
  py::array_t<int64_t> onv(static_cast<py::ssize_t>(n)), onc(static_cast<py::ssize_t>(n));
  py::array_t<double> odg(static_cast<py::ssize_t>(n));
  auto a = ov.mutable_unchecked<1>();
  auto b = ow.mutable_unchecked<1>();
  auto c = of.mutable_unchecked<1>();
  auto e = oe.mutable_unchecked<1>();
  auto f = onv.mutable_unchecked<1>();
  auto g = onc.mutable_unchecked<1>();
  auto h = odg.mutable_unchecked<1>();
  for (std::size_t k = 0; k < n; ++k) {
    const py::ssize_t s = static_cast<py::ssize_t>(k);
    a(s) = out[k].v;
    b(s) = out[k].w;
    c(s) = out[k].found;
    e(s) = out[k].exception;
    f(s) = static_cast<int64_t>(out[k].n_valid);
    g(s) = static_cast<int64_t>(out[k].n_collision);
    h(s) = std::isfinite(out[k].d_goal) ? out[k].d_goal : -1.0;
  }
  py::dict d;
  d["v"] = ov;
  d["w"] = ow;
  d["found"] = of;
  d["exception"] = oe;
  d["n_valid"] = onv;
  d["n_collision"] = onc;
  d["d_goal"] = odg;
  return d;
}

/// 여러 env 의 지역 코스트맵 갱신. sensor/robot [n,3], ranges [n, B] float32.
void costmapBatch(
  std::vector<Dwa2D *> dwas, const py::array_t<int64_t> & idx, const F64Array & sensor,
  const F64Array & robot, const F32Array & ranges, double angle_min, double angle_inc,
  double range_min, double range_max, int threads)
{
  const auto I = idx.unchecked<1>();
  const auto S = sensor.unchecked<2>();
  const auto R = robot.unchecked<2>();
  const auto G = ranges.unchecked<2>();
  const std::size_t n = static_cast<std::size_t>(I.shape(0));
  const std::size_t nb = static_cast<std::size_t>(G.shape(1));
  std::vector<std::size_t> ids(n);
  std::vector<std::array<double, 6>> st(n);
  std::vector<std::vector<float>> rr(n, std::vector<float>(nb));
  for (std::size_t k = 0; k < n; ++k) {
    const py::ssize_t s = static_cast<py::ssize_t>(k);
    ids[k] = static_cast<std::size_t>(I(s));
    if (ids[k] >= dwas.size()) {
      throw std::runtime_error("idx 범위 밖");
    }
    st[k] = {S(s, 0), S(s, 1), S(s, 2), R(s, 0), R(s, 1), R(s, 2)};
    for (std::size_t j = 0; j < nb; ++j) {
      rr[k][j] = G(s, static_cast<py::ssize_t>(j));
    }
  }
  py::gil_scoped_release release;
  parallelFor(
    n, threads, [&](std::size_t k) {
      dwas[ids[k]]->updateCostmap(
        st[k][0], st[k][1], st[k][2], st[k][3], st[k][4], st[k][5], std::move(rr[k]), angle_min,
        angle_inc, range_min, range_max);
    });
}

/// 골든 대조용: 주어진 격자·경로·상태로 코어를 한 번 부른다 (창·기억·코스트맵 갱신 없음).
py::dict computeRaw(
  const py::dict & cfg, const U8Array & cells, double res, double ox, double oy,
  const F64Array & footprint, int circ_cost, bool allow_unknown, double x, double y, double th,
  double v_meas, double w_meas, bool has_last, double v_last, double w_last,
  const F64Array & path, bool end_is_goal)
{
  if (cells.ndim() != 2) {
    throw std::runtime_error("cells 는 [H, W] uint8");
  }
  core::OwnedGrid g(
    static_cast<int>(cells.shape(1)), static_cast<int>(cells.shape(0)), res, core::kFreeSpace, ox,
    oy);
  std::memcpy(g.cells.data(), cells.data(), g.cells.size());
  const core::DwaPlanner planner(configFromDict(cfg));
  const std::vector<core::Pose2D> p = toPoses(path);
  core::DwaInput in;
  in.pose = {x, y, th};
  in.v_meas = v_meas;
  in.w_meas = w_meas;
  in.has_last = has_last;
  in.v_last = v_last;
  in.w_last = w_last;
  in.path = &p;
  in.path_end_is_goal = end_is_goal;
  in.speed_limit = 1e9;
  const core::DwaResult r = runCore(
    planner, in, g.view(), toFootprint(footprint), static_cast<uint8_t>(circ_cost), allow_unknown);
  py::dict d;
  d["v"] = r.v;
  d["w"] = r.w;
  d["found"] = r.found;
  d["n_samples"] = r.n_samples;
  d["n_valid"] = r.n_valid;
  d["n_collision"] = r.n_collision;
  d["n_recentered"] = r.n_recentered;
  d["best_cost"] = r.best.cost;
  d["d_goal"] = std::isfinite(r.d_goal) ? r.d_goal : -1.0;
  return d;
}

py::array_t<uint8_t> gridOut(const core::OwnedGrid & g)
{
  py::array_t<uint8_t> out({static_cast<py::ssize_t>(g.height), static_cast<py::ssize_t>(g.width)});
  std::memcpy(out.mutable_data(), g.cells.data(), g.cells.size());
  return out;
}

py::array_t<uint8_t> inflatePy(
  const U8Array & cells, double res, double inscribed, double radius, double scaling)
{
  core::OwnedGrid g(
    static_cast<int>(cells.shape(1)), static_cast<int>(cells.shape(0)), res, core::kFreeSpace);
  std::memcpy(g.cells.data(), cells.data(), g.cells.size());
  core::inflate(g, inscribed, radius, scaling);
  return gridOut(g);
}
}  // namespace

PYBIND11_MODULE(amr_dwa2d_py, m)
{
  m.doc() = "amr_navigation DWA 코어 + 지역 코스트맵 이식 — rl/sim2d 2D-DWA 팔 (B5)";
  m.def("default_config", []() {return configToDict(core::DwaConfig{});});
  m.def(
    "config_roundtrip", [](const py::dict & d) {return configToDict(configFromDict(d));},
    py::arg("cfg"));
  m.def(
    "inflation_cost", [](double d, double ins, double s) {
      return static_cast<int>(core::inflationCost(d, ins, s));
    }, py::arg("distance_m"), py::arg("inscribed_radius"), py::arg("cost_scaling_factor"));
  m.def(
    "inflate", &inflatePy, py::arg("cells"), py::arg("resolution"), py::arg("inscribed_radius"),
    py::arg("inflation_radius"), py::arg("cost_scaling_factor"));
  m.def(
    "denoise_ranges", [](const F32Array & r, int k, double gate, int sup) {
      const auto a = r.unchecked<1>();
      std::vector<float> v(static_cast<std::size_t>(a.shape(0)));
      for (py::ssize_t i = 0; i < a.shape(0); ++i) {
        v[static_cast<std::size_t>(i)] = a(i);
      }
      core::ScanDenoiseConfig c;
      c.half_window = k;
      c.range_gate = gate;
      c.min_support = sup;
      return core::denoiseRanges(v, c, true);
    }, py::arg("ranges"), py::arg("half_window"), py::arg("range_gate"), py::arg("min_support"));
  m.def(
    "compute_raw", &computeRaw, py::arg("cfg"), py::arg("cells"), py::arg("resolution"),
    py::arg("origin_x"), py::arg("origin_y"), py::arg("footprint"), py::arg("circumscribed_cost"),
    py::arg("allow_unknown"), py::arg("x"), py::arg("y"), py::arg("theta"), py::arg("v_meas"),
    py::arg("w_meas"), py::arg("has_last"), py::arg("v_last"), py::arg("w_last"), py::arg("path"),
    py::arg("end_is_goal"));
  py::class_<Dwa2D>(m, "Dwa2D")
  .def(
    py::init<const py::dict &, const py::dict &, const F64Array &, double, int, bool>(),
    py::arg("cfg"), py::arg("costmap"), py::arg("footprint"), py::arg("path_horizon"),
    py::arg("no_valid_patience"), py::arg("allow_unknown"))
  .def("reset", &Dwa2D::reset, py::arg("poses"))
  .def(
    "step", [](Dwa2D & d, double x, double y, double th, double v, double w) {
      return toDict(d.step(x, y, th, v, w));
    }, py::arg("x"), py::arg("y"), py::arg("theta"), py::arg("v_meas"), py::arg("w_meas"))
  .def(
    "update_costmap", [](Dwa2D & d, double sx, double sy, double syaw, double rx, double ry,
    double rth, const F32Array & ranges, double amin, double ainc, double rmin, double rmax) {
      const auto a = ranges.unchecked<1>();
      std::vector<float> v(static_cast<std::size_t>(a.shape(0)));
      for (py::ssize_t i = 0; i < a.shape(0); ++i) {
        v[static_cast<std::size_t>(i)] = a(i);
      }
      d.updateCostmap(sx, sy, syaw, rx, ry, rth, std::move(v), amin, ainc, rmin, rmax);
    })
  .def(
    "master_grid", [](Dwa2D & d) {return gridOut(d.costmap().master());})
  .def(
    "obstacle_grid", [](Dwa2D & d) {
      const int n = d.costmap().size();
      py::array_t<uint8_t> out({static_cast<py::ssize_t>(n), static_cast<py::ssize_t>(n)});
      std::memcpy(out.mutable_data(), d.costmap().obstacle().data(), d.costmap().obstacle().size());
      return out;
    })
  .def(
    "costmap_origin", [](Dwa2D & d) {
      return std::make_pair(d.costmap().originX(), d.costmap().originY());
    })
  .def(
    "set_master_grid", [](Dwa2D & d, const U8Array & cells, double res, double ox, double oy) {
      core::OwnedGrid g(
        static_cast<int>(cells.shape(1)), static_cast<int>(cells.shape(0)), res,
        core::kFreeSpace, ox, oy);
      std::memcpy(g.cells.data(), cells.data(), g.cells.size());
      d.costmap().setMaster(g);
    })
  .def("config", &Dwa2D::config)
  .def("circumscribed_cost", &Dwa2D::circCost);
  m.def(
    "step_batch", &stepBatch, py::arg("dwas"), py::arg("idx"), py::arg("poses"), py::arg("v"),
    py::arg("w"), py::arg("threads") = 1);
  m.def(
    "costmap_batch", &costmapBatch, py::arg("dwas"), py::arg("idx"), py::arg("sensor"),
    py::arg("robot"), py::arg("ranges"), py::arg("angle_min"), py::arg("angle_inc"),
    py::arg("range_min"), py::arg("range_max"), py::arg("threads") = 1);
  m.attr("LETHAL") = static_cast<int>(core::kLethalObstacle);
  m.attr("INSCRIBED") = static_cast<int>(core::kInscribedInflated);
}
