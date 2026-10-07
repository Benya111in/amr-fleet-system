// 골든 대조용: 바인딩(amr_dwa2d_py)을 거치지 않고 core::DwaPlanner::compute 를 직접 부른다.
//
// 설정은 배포 값을 **이 파일에 손으로 옮긴 것**이다 (src/amr_navigation/config/nav2_params.yaml
// controller_server.DWA 100–359 행, 없는 키는 dwa_controller.cpp configure() 95–224 행의 기본값,
// 한계는 config/robot_params.yaml limits 57–62 행). 파이썬 쪽(amr_sim2d/dwa2d.py)은 같은 값을 YAML 에서
// 자동으로 읽는다 — 두 경로가 같은 출력을 내면 설정 옮김과 입력 변환이 모두 맞다는 근거가 된다.
//
// 입력 (stdin, 공백 구분 텍스트):
//   grid W H res ox oy  /  W*H 개 셀 값  /  circ C  /  fp K x y …  /  allow_unknown 0|1
//   state x y th v_meas w_meas has_last v_last w_last end_is_goal  /  path K  /  K × (x y th)
// 출력 (한 줄): v w found n_samples n_valid n_collision n_recentered best_cost d_goal (%.17g)
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/grid.hpp"

namespace core = amr_navigation::core;

static core::DwaConfig deployConfig()
{
  core::DwaConfig c;
  c.limits.max_vel_x = 1.0;          // N:104
  c.limits.min_vel_x = 0.0;          // N:105
  c.limits.max_vel_theta = 1.5;      // R:60
  c.limits.acc_lim_x = 1.0;          // R:59
  c.limits.decel_lim_x = 1.0;        // R:59
  c.limits.acc_lim_theta = 2.0;      // R:61
  c.limits.jerk_lim_x = 2.0;         // R:62
  c.weights.heading = 0.6;           // N:146
  c.weights.clearance = 1.0;         // N:147
  c.weights.velocity = 0.4;          // N:149
  c.weights.path = 2.0;              // N:150
  c.weights.oscillation = 0.5;       // N:151
  c.weights.dynamic = 1.5;           // N:152
  c.weights.off_path = 3.0;          // N:162
  c.weights.escape = 2.5;            // N:244
  c.control_period = 1.0 / 20.0;     // N:75
  c.vx_samples = 11;                 // N:109
  c.vth_samples = 31;                // N:110
  c.sustained_sampling = false;      // N:135
  c.jerk_lim_theta = 6.0;            // 래퍼 기본값 (limits.max_angular_jerk 없음)
  c.sim_dt = 0.1;                    // N:136
  c.sim_time_min = 1.5;              // N:137
  c.sim_time_max = 2.5;              // N:138
  c.commit_time = 0.2;               // N:139
  c.approach_latency = 0.3;          // N:140
  c.window_reset_v = 0.3;            // N:144
  c.window_reset_w = 0.5;            // N:145
  c.heading_lookahead_gain = 0.4;    // N:155
  c.heading_lookahead_offset = 0.3;  // N:156
  c.heading_lookahead_min = 0.4;     // N:157
  c.heading_lookahead_max = 1.2;     // N:158
  c.path_band = 0.8;                 // N:159
  c.max_path_offset = 0.9;           // N:163
  c.max_path_offset_hard = 0.95;     // N:164
  c.off_path_band = 1.0;             // N:169
  c.path_eval_time = 0.8;            // N:153
  c.goal_align_distance = 0.08;      // N:177
  c.goal_overshoot_margin = 0.1;     // N:178
  c.oscillation_w_eps = 0.05;        // 래퍼 기본값
  c.use_dynamic_obstacles = true;    // N:181
  c.use_velocity_obstacles = true;   // N:182
  c.prediction_time = 5.0;           // N:183
  c.dynamic_margin = 0.5;            // N:184
  c.vo_time_horizon = 2.0;           // N:185
  c.vo_margin = 0.3;                 // N:186
  c.vo_max_range = 6.0;              // N:187
  c.dynamic_speed_threshold = 0.2;   // N:188
  c.dynamic_steer_gain = 0.3;        // N:190
  c.vo_time_tie = 0.02;              // N:191
  c.yield_crossing = true;           // N:192
  c.yield_corridor_margin = 0.7;     // N:194
  c.yield_stop_margin = 0.8;         // N:200
  c.yield_clear_margin = 2.0;        // N:206
  c.yield_horizon = 8.0;             // N:210
  c.yield_nominal_speed = 1.0;       // N:211
  c.yield_hold_standoff = 0.35;      // N:212
  c.yield_approach_radius = 3.0;     // N:213
  c.yield_approach_horizon = 3.0;    // N:219
  c.yield_max_zone = 6.0;            // N:232
  c.yield_escape_speed = 0.5;        // N:241
  c.yield_escape_drift = 0.15;       // N:245
  c.yield_escape_v_meas = 0.05;      // N:331
  c.escape_min_speed = 0.05;         // N:246
  c.yield_decision = false;          // N:264
  c.yield_body_margin = 0.15;        // N:265
  c.yield_go_margin = 0.0;           // N:269
  c.yield_go_clearance = 0.40;       // N:271
  c.yield_go_hold_clearance = 0.30;  // N:279
  c.yield_go_speed_floor = 1.0;      // N:280
  c.yield_hold_slack = 0.3;          // N:284
  c.yield_decision_min_sin = 0.34;   // N:286
  c.gate.enable = false;             // N:305
  c.gate.hold_back = 1.2;            // N:306
  c.gate.delay = 0.3;                // N:308
  c.gate.pos_err = 0.25;             // N:309
  c.gate.heading_unc = 0.1;          // N:310
  c.gate.v_max = 1.0;                // N:311
  c.gate.vknown = 0.3;               // N:312
  c.gate.turn_time = 1.57;           // N:313
  c.gate.walk_accel = 0.6;           // N:314
  c.gate.static_timeout = 5.0;       // N:315
  c.gate.lookahead = 8.0;            // N:316
  c.gate.group_gap = 1.5;            // N:317
  c.gate.hold_tol = 0.3;             // N:318
  c.gate.hold_threat_horizon = 3.0;  // N:319
  c.gate.hold_exclusion_horizon = 10.0;   // N:320
  c.gate.commit_max_run = 2.0;       // N:323
  c.gate.retreat_enable = true;      // N:325
  c.gate.retreat_speed = 0.5;        // N:326
  c.gate.retreat_max = 4.0;          // N:327
  c.recenter_narrow = true;          // N:336
  c.recenter_min_cost = 100.0;       // N:337
  c.recenter_max_shift = 0.10;       // N:338
  c.recenter_smooth = 4;             // N:339
  c.recenter_goal_keep = 0.5;        // N:340
  c.recenter_step = 0.025;           // N:521 (지역 코스트맵 해상도, dwa_controller.cpp:210)
  c.robot_radius = std::hypot(0.30, 0.20);   // N:356 −1 → 외접 반경 (N:522 풋프린트)
  c.yield_lookahead = 4.0;           // min(N:179 path_horizon 4.0, N:231 4.0)
  return c;
}

int main()
{
  std::string tag;
  int W = 0, H = 0;
  double res = 0.0, ox = 0.0, oy = 0.0;
  std::cin >> tag >> W >> H >> res >> ox >> oy;
  core::OwnedGrid g(W, H, res, core::kFreeSpace, ox, oy);
  for (int i = 0; i < W * H; ++i) {
    int v = 0;
    std::cin >> v;
    g.cells[static_cast<std::size_t>(i)] = static_cast<uint8_t>(v);
  }
  int circ = 0;
  std::cin >> tag >> circ;
  int K = 0;
  std::cin >> tag >> K;
  std::vector<core::Point2D> fp(static_cast<std::size_t>(K));
  for (auto & p : fp) {
    std::cin >> p.x >> p.y;
  }
  int allow = 0;
  std::cin >> tag >> allow;
  core::DwaInput in;
  int has_last = 0, end_is_goal = 0;
  std::cin >> tag >> in.pose.x >> in.pose.y >> in.pose.theta >> in.v_meas >> in.w_meas >> has_last >>
  in.v_last >> in.w_last >> end_is_goal;
  in.has_last = has_last != 0;
  in.path_end_is_goal = end_is_goal != 0;
  in.speed_limit = 1e9;
  int P = 0;
  std::cin >> tag >> P;
  std::vector<core::Pose2D> path(static_cast<std::size_t>(P));
  for (auto & p : path) {
    std::cin >> p.x >> p.y >> p.theta;
  }
  in.path = &path;
  if (!std::cin) {
    std::cerr << "입력 형식 오류\n";
    return 2;
  }
  const core::DwaPlanner planner(deployConfig());
  const core::CostGrid grid = g.view();
  const core::FootprintChecker checker(grid, fp, static_cast<uint8_t>(circ), allow != 0);
  const core::DwaResult r = planner.compute(
    in, [&checker](const core::Pose2D & p) {return checker.cost(p);},
    [&grid](double x, double y) {
      const uint8_t c = grid.costAtWorld(x, y);
      return c == core::kNoInformation ? 0.0 : static_cast<double>(c);
    });
  std::printf(
    "%.17g %.17g %d %zu %zu %zu %zu %.17g %.17g\n", r.v, r.w, r.found ? 1 : 0, r.n_samples,
    r.n_valid, r.n_collision, r.n_recentered, r.best.cost,
    std::isfinite(r.d_goal) ? r.d_goal : -1.0);
  return 0;
}
