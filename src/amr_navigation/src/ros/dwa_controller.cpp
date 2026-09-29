#include "amr_navigation/dwa_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/ros_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{

// ---------------------------------------------------------------------------------------------
// PlanWindow

void PlanWindow::setPlan(const nav_msgs::msg::Path & plan)
{
  plan_ = plan;
  poses_ = toPoses2D(plan);
  hint_ = 0;
  fresh_ = true;
}

std::vector<core::Pose2D> PlanWindow::window(
  const core::Pose2D & T, const core::Pose2D & robot, double horizon, bool & end_is_goal)
{
  std::vector<core::Pose2D> out;
  end_is_goal = false;
  if (poses_.empty()) {
    return out;
  }
  // 로봇을 경로 프레임으로 옮겨 최근접 정점 탐색. 새 경로는 로봇 위치에서 시작하므로 앞부분만,
  // 이후에는 직전 인덱스부터 앞쪽 창만 본다 (되돌아오는 경로에서 뒤쪽 구간으로 튀지 않도록).
  const core::Pose2D r = core::compose(core::inverse(T), robot);
  constexpr std::size_t kFreshWindow = 400;   // 0.05 m 간격 기준 20 m
  constexpr std::size_t kTrackWindow = 200;   // 10 m
  const std::size_t begin = fresh_ ? 0 : (hint_ > back_search_ ? hint_ - back_search_ : 0);
  const std::size_t end = std::min(poses_.size(), begin + (fresh_ ? kFreshWindow : kTrackWindow));
  double best = std::numeric_limits<double>::infinity();
  std::size_t best_i = begin;
  for (std::size_t i = begin; i < end; ++i) {
    const double d = core::distance(poses_[i], r);
    if (d < best) {
      best = d;
      best_i = i;
    }
  }
  hint_ = best_i;
  fresh_ = false;
  const std::size_t s = best_i > 0 ? best_i - 1 : 0;
  out.push_back(core::compose(T, poses_[s]));
  double len = 0.0;
  std::size_t last = s;
  for (std::size_t i = s + 1; i < poses_.size(); ++i) {
    len += core::distance(poses_[i - 1], poses_[i]);
    out.push_back(core::compose(T, poses_[i]));
    last = i;
    if (len > horizon) {
      break;
    }
  }
  end_is_goal = last + 1 == poses_.size();
  return out;
}

// ---------------------------------------------------------------------------------------------
// DWAController

void DWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("DWAController: parent node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  const std::string p = name_ + ".";

  core::DwaConfig c;
  auto & L = c.limits;
  // 한계 기본값은 robot_params.yaml limits.* (controller_server
  //   에 함께 로드), 플러그인 파라미터가 있으면 우선
  const double a_lin = param(node, "limits.max_linear_acceleration", 1.0);
  L.max_vel_x =
    param(node, p + "max_vel_x", std::min(1.0, param(node, "limits.max_linear_velocity", 2.0)));
  L.min_vel_x = param(node, p + "min_vel_x", 0.0);
  L.max_vel_theta =
    param(node, p + "max_vel_theta", param(node, "limits.max_angular_velocity", 1.5));
  L.acc_lim_x = param(node, p + "acc_lim_x", a_lin);
  L.decel_lim_x = param(node, p + "decel_lim_x", a_lin);
  L.acc_lim_theta = param(
    node, p + "acc_lim_theta", param(node, "limits.max_angular_acceleration", 2.0));
  L.jerk_lim_x = param(node, p + "jerk_lim_x", param(node, "limits.max_linear_jerk", 2.0));
  max_vel_x_ = L.max_vel_x;
  auto & W = c.weights;
  W.heading = param(node, p + "heading_weight", 0.6);
  W.clearance = param(node, p + "clearance_weight", 1.0);
  W.velocity = param(node, p + "velocity_weight", 0.4);
  W.path = param(node, p + "path_weight", 2.0);
  W.oscillation = param(node, p + "oscillation_weight", 0.5);
  W.dynamic = param(node, p + "dynamic_weight", 1.5);
  W.off_path = param(node, p + "off_path_weight", 3.0);
  W.escape = param(node, p + "escape_weight", 2.5);
  double controller_frequency = 20.0;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", controller_frequency);
  }
  c.control_period = 1.0 / std::max(1.0, controller_frequency);
  c.vx_samples = param(node, p + "vx_samples", 11);
  c.vth_samples = param(node, p + "vth_samples", 31);
  // R1+R2: 표본을 지속 목표 명령으로, 롤아웃을 가속·저크 램프로 (연구 브리프 §6).
  // 각저크 기본값은 velocity_profiler 의 max_angular_jerk 와 같은 출처를 쓴다.
  c.sustained_sampling = param(node, p + "sustained_sampling", false);
  c.jerk_lim_theta = param(
    node, p + "jerk_lim_theta", param(node, "limits.max_angular_jerk", 6.0));
  c.sim_dt = param(node, p + "sim_dt", 0.1);
  c.sim_time_min = param(node, p + "sim_time_min", 1.5);
  c.sim_time_max = param(node, p + "sim_time_max", 2.5);
  c.commit_time = param(node, p + "commit_time", 0.2);
  c.approach_latency = param(node, p + "approach_latency", 0.3);
  c.window_reset_v = param(node, p + "window_reset_v", 0.3);
  c.window_reset_w = param(node, p + "window_reset_w", 0.5);
  c.heading_lookahead_gain = param(node, p + "heading_lookahead_gain", 0.4);
  c.heading_lookahead_offset = param(node, p + "heading_lookahead_offset", 0.3);
  c.heading_lookahead_min = param(node, p + "heading_lookahead_min", 0.4);
  c.heading_lookahead_max = param(node, p + "heading_lookahead_max", 1.2);
  c.path_band = param(node, p + "path_band", 0.8);
  c.max_path_offset = param(node, p + "max_path_offset", 0.9);
  c.max_path_offset_hard = param(node, p + "max_path_offset_hard", 0.95);
  c.off_path_band = param(node, p + "off_path_band", 0.05);
  c.path_eval_time = param(node, p + "path_eval_time", 0.8);
  c.goal_align_distance = param(node, p + "goal_align_distance", 0.08);
  c.goal_overshoot_margin = param(node, p + "goal_overshoot_margin", 0.1);
  c.oscillation_w_eps = param(node, p + "oscillation_w_eps", 0.05);
  c.use_dynamic_obstacles = param(node, p + "use_dynamic_obstacles", true);
  c.use_velocity_obstacles = param(node, p + "use_velocity_obstacles", true);
  c.prediction_time = param(node, p + "prediction_time", 5.0);
  c.dynamic_margin = param(node, p + "dynamic_margin", 0.5);
  c.vo_time_horizon = param(node, p + "vo_time_horizon", 2.0);
  c.vo_margin = param(node, p + "vo_margin", 0.3);
  c.vo_max_range = param(node, p + "vo_max_range", 6.0);
  c.dynamic_speed_threshold = param(node, p + "dynamic_speed_threshold", 0.2);
  c.dynamic_steer_gain = param(node, p + "dynamic_steer_gain", 0.3);
  c.vo_time_tie = param(node, p + "vo_time_tie", 0.02);
  c.yield_crossing = param(node, p + "yield_crossing", true);
  c.yield_corridor_margin = param(node, p + "yield_corridor_margin", 0.50);
  c.yield_stop_margin = param(node, p + "yield_stop_margin", 0.25);
  c.yield_clear_margin = param(node, p + "yield_clear_margin", 1.0);
  c.yield_horizon = param(node, p + "yield_horizon", 8.0);
  c.yield_nominal_speed = param(node, p + "yield_nominal_speed", 1.0);
  c.yield_hold_standoff = param(node, p + "yield_hold_standoff", 0.35);
  c.yield_approach_radius = param(node, p + "yield_approach_radius", 3.0);
  c.yield_approach_horizon = param(node, p + "yield_approach_horizon", 3.0);
  yield_decision_dwell_ = param(node, p + "yield_decision_dwell", 0.8);
  c.yield_max_zone = param(node, p + "yield_max_zone", 6.0);
  c.yield_escape_speed = param(node, p + "yield_escape_speed", 0.25);
  c.yield_escape_drift = param(node, p + "yield_escape_drift", 0.15);
  c.yield_escape_v_meas = param(node, p + "yield_escape_v_meas", 0.05);
  c.escape_min_speed = param(node, p + "escape_min_speed", 0.05);
  c.yield_decision = param(node, p + "yield_decision", true);
  c.yield_body_margin = param(node, p + "yield_body_margin", 0.15);
  c.yield_go_margin = param(node, p + "yield_go_margin", 0.0);
  c.yield_go_clearance = param(node, p + "yield_go_clearance", 0.40);
  c.yield_go_hold_clearance = param(node, p + "yield_go_hold_clearance", 0.30);
  c.yield_go_speed_floor = param(node, p + "yield_go_speed_floor", 1.0);
  c.yield_hold_slack = param(node, p + "yield_hold_slack", 0.3);
  c.yield_decision_min_sin = param(node, p + "yield_decision_min_sin", 0.34);
  // 횡단 게이트 (T안, crossing_gate.hpp). 기본값은 시뮬레이션에서 접촉 0/610 을 낸 값 그대로다.
  c.gate.enable = param(node, p + "yield_gate", false);
  c.gate.pos_err = param(node, p + "gate_pos_err", 0.25);
  c.gate.heading_unc = param(node, p + "gate_heading_unc", 0.1);
  c.gate.v_max = param(node, p + "gate_v_max", 1.0);
  c.gate.delay = param(node, p + "gate_delay", 0.3);
  c.gate.hold_back = param(node, p + "gate_hold_back", 1.2);
  c.gate.turn_time = param(node, p + "gate_turn_time", 1.57);
  c.gate.walk_accel = param(node, p + "gate_walk_accel", 0.6);
  c.gate.vknown = param(node, p + "gate_vknown", 0.3);
  c.gate.static_timeout = param(node, p + "gate_static_timeout", 5.0);
  c.gate.lookahead = param(node, p + "gate_lookahead", 8.0);
  c.gate.group_gap = param(node, p + "gate_group_gap", 1.5);
  c.gate.hold_tol = param(node, p + "gate_hold_tol", 0.3);
  c.gate.hold_threat_horizon = param(node, p + "gate_hold_threat_horizon", 3.0);
  c.gate.hold_exclusion_horizon = param(node, p + "gate_hold_exclusion_horizon", 10.0);
  c.gate.commit_max_run = param(node, p + "gate_commit_max_run", 2.0);
  c.gate.retreat_enable = param(node, p + "gate_retreat", true);
  c.gate.retreat_speed = param(node, p + "gate_retreat_speed", 0.5);
  c.gate.retreat_max = param(node, p + "gate_retreat_max", 4.0);
  gate_enable_ = c.gate.enable;
  gate_lookahead_ = c.gate.lookahead;
  gate_vknown_ = c.gate.vknown;
  gate_line_window_ = param(node, p + "gate_line_window", 1.5);
  gate_track_memory_ = param(node, p + "gate_track_memory", 2.0);
  c.recenter_narrow = param(node, p + "recenter_narrow", true);
  c.recenter_min_cost = param(node, p + "recenter_min_cost", 100.0);
  c.recenter_max_shift = param(node, p + "recenter_max_shift", 0.10);
  c.recenter_smooth = param(node, p + "recenter_smooth", 4);
  c.recenter_goal_keep = param(node, p + "recenter_goal_keep", 0.5);
  c.recenter_step = costmap_ros_->getCostmap()->getResolution();   // 탐색 간격 = 지역 코스트맵 셀
  const double robot_radius = param(node, p + "robot_radius", -1.0);
  c.robot_radius = robot_radius > 0.0 ? robot_radius :
    core::FootprintChecker::circumscribedRadius(footprintOf(*costmap_ros_));
  obstacle_radius_ = param(node, p + "obstacle_radius", 0.25);
  dynamic_fast_speed_ = param(node, p + "dynamic_fast_speed", 0.5);
  track_timeout_ = param(node, p + "track_timeout", 0.5);
  yield_velocity_tau_ = param(node, p + "yield_velocity_tau", 0.5);
  robot_mass_ = param(node, p + "robot_mass", 47.6);
  path_horizon_ = param(node, p + "path_horizon", 6.0);
  // 정지선 탐색 거리는 코스트맵 프레임으로 옮긴 경로 창을 넘을 수 없다
  c.yield_lookahead = std::min(path_horizon_, param(node, p + "yield_lookahead", 4.0));
  no_valid_limit_ = param(node, p + "no_valid_patience", 10);
  transform_tolerance_ = param(node, p + "transform_tolerance", 0.2);
  allow_unknown_ = param(node, p + "allow_unknown", true);
  const std::string obs_topic =
    param(node, p + "tracked_obstacles_topic", std::string("perception/tracked_obstacles"));
  const std::string payload_topic = param(node, p + "payload_topic", std::string("payload/mass"));

  base_config_ = c;
  planner_.setConfig(c);

  obs_sub_ = node->create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    obs_topic, rclcpp::QoS(5),
    [this](const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg) {onObstacles(msg);});
  payload_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    payload_topic, rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {onPayload(msg);});
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("dwa/stats", 10);
  RCLCPP_INFO(
    logger_,
    "DWAController '%s': v [%.2f, %.2f] m/s, w %.2f rad/s, %d x %d samples, VO %s, robot radius "
    "%.3f m", name_.c_str(), L.min_vel_x, L.max_vel_x, L.max_vel_theta, c.vx_samples,
    c.vth_samples, c.use_velocity_obstacles ? "on" : "off", c.robot_radius);
  RCLCPP_INFO(
    node->get_logger(), "  표본: %s (지속 목표 명령 + 램프 롤아웃 = %s)",
    c.sustained_sampling ? "sustained" : "1주기 창",
    c.sustained_sampling ? "on" : "off");
}

void DWAController::cleanup()
{
  obs_sub_.reset();
  payload_sub_.reset();
  local_plan_pub_.reset();
  stats_pub_.reset();
}

void DWAController::activate()
{
  local_plan_pub_->on_activate();
  stats_pub_->on_activate();
  has_last_ = false;
  gate_state_ = core::GateState{};
  gate_hist_.clear();
  gate_hist_stamp_ = -1.0;
}

void DWAController::deactivate()
{
  local_plan_pub_->on_deactivate();
  stats_pub_->on_deactivate();
}

void DWAController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_.setPlan(path);
}

void DWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit <= 0.0 || speed_limit >= 100.0 - 1e-9) {
    speed_limit_ = 1e9;   // nav2_costmap_2d::NO_SPEED_LIMIT (0) → 제한 없음
  } else {
    speed_limit_ = percentage ? max_vel_x_ * speed_limit / 100.0 : speed_limit;
  }
}

void DWAController::onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obs_mutex_);
  obstacles_ = msg;
}

void DWAController::onPayload(const std_msgs::msg::Float32::SharedPtr msg)
{
  // 적재 질량 → 가속 한계 비례 축소 s = m / (m + m_payload) (같은 구동 토크 가정)
  const double m = std::max(0.0, static_cast<double>(msg->data));
  const double s = robot_mass_ / (robot_mass_ + m);
  std::lock_guard<std::mutex> lock(mutex_);
  core::DwaConfig c = base_config_;
  c.limits.acc_lim_x *= s;
  c.limits.decel_lim_x *= s;
  c.limits.acc_lim_theta *= s;
  planner_.setConfig(c);
  RCLCPP_INFO(logger_, "DWAController: payload %.1f kg -> accel scale %.3f", m, s);
}

std::vector<core::DynamicObstacle> DWAController::obstaclesInFrame(const std::string & frame)
{
  amr_msgs::msg::TrackedObstacleArray::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    msg = obstacles_;
  }
  std::vector<core::DynamicObstacle> out;
  if (!msg || msg->obstacles.empty()) {
    return out;
  }
  const rclcpp::Time stamp(msg->header.stamp, clock_->get_clock_type());
  const double age = (clock_->now() - stamp).seconds();
  last_track_age_ = age;
  if (age > track_timeout_ || age < -1.0) {
    // 트랙이 오래됐다고 "장애물 없음" 으로 보는 것은 열린 실패다 — 양보가 풀린다.
    // 얼마나 자주 일어나는지 dwa/stats 로 드러낸다 (age 가 남으므로 로그에서 구별된다).
    return out;
  }
  core::Pose2D T;
  if (!lookupTransform2D(*tf_, frame, msg->header.frame_id, 0.0, T)) {
    return out;
  }
  const double c = std::cos(T.theta);
  const double s = std::sin(T.theta);
  const double dt = std::max(0.0, age);
  std::unordered_set<int> seen;
  for (const auto & o : msg->obstacles) {
    // 추적기 분류(is_dynamic) 이거나 빠른 트랙만
    // (정지 물체 트랙의 추정 속도 잡음 0.2–0.3 m/s 가 VO·TTC 를 만들지 않게)
    if (!o.is_dynamic && std::hypot(o.velocity.x, o.velocity.y) < dynamic_fast_speed_) {
      continue;
    }
    core::DynamicObstacle d;
    // 등속 모델로 현재 시각까지 전진시킨 뒤 코스트맵 프레임으로 회전·이동
    const double px = o.position.x + o.velocity.x * dt;
    const double py = o.position.y + o.velocity.y * dt;
    d.x = T.x + c * px - s * py;
    d.y = T.y + s * px + c * py;
    d.vx = c * o.velocity.x - s * o.velocity.y;
    d.vy = s * o.velocity.x + c * o.velocity.y;
    d.radius = obstacle_radius_;
    d.id = static_cast<int>(o.track_id);
    // 통로 예측용 평활 속도 (트랙별 지수 필터). VO·TTC 는 위의 원시 속도를 그대로 쓴다.
    if (yield_velocity_tau_ > 0.0) {
      // 시간은 **제어 틱이 아니라 트랙 메시지 스탬프**로 센다. 이 함수는 매 제어 주기(20 Hz)
      // 호출되면서 캐시된 최신 메시지를 쓰므로, 추적기가 더 느리면 같은 스탬프를 여러 번 본다.
      // 예전에는 그것을 `gap <= 0.0` 으로 불연속과 함께 묶어 매번 원시값으로 리셋했고, 그래서
      // τ 평활이 사실상 꺼져 있었다 (실측 재현: 20 Hz × 10 Hz 에서 0.5 s 뒤 잔량 0.000,
      // 고친 뒤 exp(-1)=0.368 — test_track_velocity_filter.cpp).
      core::TrackVelocityFilter & f = vel_filter_[o.track_id];
      f.update(d.vx, d.vy, stamp.seconds(), yield_velocity_tau_, track_timeout_);
      d.vx_pred = f.vx;
      d.vy_pred = f.vy;
      d.has_pred = true;
    }
    seen.insert(o.track_id);
    out.push_back(d);
  }
  for (auto it = vel_filter_.begin(); it != vel_filter_.end(); ) {   // 사라진 트랙은 버린다
    it = seen.count(it->first) ? std::next(it) : vel_filter_.erase(it);
  }
  return out;
}

std::vector<core::GateTrack> DWAController::gateTracksInFrame(const std::string & frame)
{
  amr_msgs::msg::TrackedObstacleArray::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    msg = obstacles_;
  }
  const double now = clock_->now().seconds();
  for (auto & kv : gate_hist_) {
    kv.second.in_latest = false;
  }
  if (msg && !msg->obstacles.empty()) {
    const rclcpp::Time stamp(msg->header.stamp, clock_->get_clock_type());
    const double ts = stamp.seconds();
    const double age = now - ts;
    core::Pose2D T;
    if (age <= track_timeout_ && age >= -1.0 && ts > gate_hist_stamp_ &&
      lookupTransform2D(*tf_, frame, msg->header.frame_id, 0.0, T))
    {
      // 등속 외삽 없이 관측 위치 그대로 — 지연은 게이트의 delay 가 도달 시간에 더한다.
      // 기록은 코스트맵(odom) 프레임: 연속이라 위치추정 보정 점프가 가짜 변위가 되지 않는다.
      gate_hist_stamp_ = ts;
      const double c = std::cos(T.theta);
      const double s = std::sin(T.theta);
      for (const auto & o : msg->obstacles) {
        const double px = T.x + c * o.position.x - s * o.position.y;
        const double py = T.y + s * o.position.x + c * o.position.y;
        TrackHistory & h = gate_hist_[static_cast<int>(o.track_id)];
        if (h.first_seen < 0.0) {
          h.first_seen = ts;
        }
        h.last_seen = ts;
        h.x = px;
        h.y = py;
        h.in_latest = true;
        h.samples.push_back({ts, px, py});
        while (!h.samples.empty() && ts - h.samples.front()[0] > gate_line_window_) {
          h.samples.pop_front();
        }
      }
    } else if (age <= track_timeout_ && age >= -1.0) {
      // 같은 메시지를 다시 봤다 (제어 20 Hz > 추적 10 Hz): 최신 목록의 트랙은 여전히 "보인다"
      for (const auto & o : msg->obstacles) {
        auto it = gate_hist_.find(static_cast<int>(o.track_id));
        if (it != gate_hist_.end()) {
          it->second.in_latest = true;
        }
      }
    }
  }
  std::vector<core::GateTrack> out;
  for (auto it = gate_hist_.begin(); it != gate_hist_.end(); ) {
    TrackHistory & h = it->second;
    if (now - h.last_seen > gate_track_memory_) {
      it = gate_hist_.erase(it);   // 기억에서도 지운다
      continue;
    }
    core::GateTrack g;
    g.id = it->first;
    g.x = h.x;
    g.y = h.y;
    g.radius = obstacle_radius_;
    // 방향: 창 안 첫 표본 → 마지막 표본의 변위. 사라진(기억 속) 트랙은 방향 모름으로 본다 —
    // 어느 쪽으로든 v_max 로 갈 수 있는 원판. 속도 크기는 되돌아옴 모델의 정지 시간에만 쓴다.
    if (h.in_latest && h.samples.size() >= 2) {
      const auto & a = h.samples.front();
      const auto & b = h.samples.back();
      const double dt = b[0] - a[0];
      const double dx = b[1] - a[1];
      const double dy = b[2] - a[2];
      const double disp = std::hypot(dx, dy);
      if (dt >= 0.5 && disp / dt >= gate_vknown_) {
        g.heading_known = true;
        g.ux = dx / disp;
        g.uy = dy / disp;
        g.speed = disp / dt;
        h.last_moving = h.last_seen;
      }
    }
    g.stationary_s = g.heading_known ? 0.0 :
      now - (h.last_moving >= 0.0 ? h.last_moving : h.first_seen);
    out.push_back(g);
    ++it;
  }
  return out;
}

geometry_msgs::msg::TwistStamped DWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (plan_.empty()) {
    throw nav2_core::PlannerException("DWAController: no plan");
  }
  const std::string frame = costmap_ros_->getGlobalFrameID();
  core::Pose2D T;
  if (!lookupTransform2D(*tf_, frame, plan_.frame(), transform_tolerance_, T)) {
    throw nav2_core::PlannerException(
            "DWAController: cannot transform plan from '" + plan_.frame() + "' to '" + frame + "'");
  }
  core::DwaInput in;
  in.pose = toPose2D(pose.pose);
  bool end_is_goal = false;
  // 게이트는 노출 구간 출구까지 봐야 하므로 창을 늘리고, 경로 후진 뒤 창이 따라오도록 뒤쪽 탐색을
  // 연다
  plan_.setBackSearch(gate_enable_ ? 60 : 0);
  const std::vector<core::Pose2D> path = plan_.window(
    T, in.pose, gate_enable_ ? std::max(path_horizon_, gate_lookahead_) : path_horizon_,
    end_is_goal);
  in.path = &path;
  in.path_end_is_goal = end_is_goal;
  in.v_meas = velocity.linear.x;
  in.w_meas = velocity.angular.z;
  in.has_last = has_last_;
  in.v_last = v_last_;
  in.w_last = w_last_;
  in.speed_limit = speed_limit_;
  in.yield_limit_last = yield_limit_last_;
  in.yield_decision_last = yield_decision_last_;
  in.obstacles = obstaclesInFrame(frame);
  if (gate_enable_) {
    in.gate_tracks = gateTracksInFrame(frame);
    in.gate_state = gate_state_;
  }

  core::DwaResult res;
  {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> cl(*costmap->getMutex());
    const core::CostGrid grid = costmapView(*costmap);
    const core::FootprintChecker checker(
      grid, footprintOf(*costmap_ros_), circumscribedCost(*costmap_ros_), allow_unknown_);
    res = planner_.compute(
      in, [&checker](const core::Pose2D & p) {return checker.cost(p);},
      [&grid](double x, double y) {
        const uint8_t c = grid.costAtWorld(x, y);
        return c == core::kNoInformation ? 0.0 : static_cast<double>(c);
      });
  }
  yield_limit_last_ = res.yield.speed_limit;
  if (gate_enable_) {
    gate_state_ = res.gate.next;
  }
  // 결정 디바운스. 08 실측: 접촉이 난 시행들이 전부 go ↔ hold ↔ retreat 진동이었다
  // (p8d 5회, p8e 8회, r8d 9회, s8a **31회**/410주기). 매 주기 여유 추정이 조금만 흔들려도
  // 분기가 바뀌고, 그러면 지나가지도 물러나지도 못한 채 통로 안에서 시간을 쓴다.
  // "보수적인 쪽으로만" 도 시도했으나(q 계열) retreat 에 영구 고착돼 더 나빴다 — 접촉 5.1 %.
  // 그래서 방향은 막지 않고 **최소 유지 시간**만 둔다. 바뀔 만한 이유가 계속 있으면 바뀐다.
  const int dec_now = static_cast<int>(res.yield.decision);
  const rclcpp::Time now_t = clock_->now();
  if (dec_now != yield_decision_last_) {
    if (yield_decision_last_ != 0 && dec_now != 0 &&
      (now_t - yield_decision_since_).seconds() < yield_decision_dwell_)
    {
      res.yield.decision = static_cast<core::YieldDecision>(yield_decision_last_);
    } else {
      yield_decision_last_ = dec_now;
      yield_decision_since_ = now_t;
    }
  }
  const double cycle_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  if (!res.found) {
    ++no_valid_count_;
    if (no_valid_count_ >= no_valid_limit_ && std::abs(in.v_meas) < 0.05) {
      no_valid_count_ = 0;
      has_last_ = false;
      throw nav2_core::PlannerException("DWAController: no valid trajectory (all samples collide)");
    }
  } else {
    no_valid_count_ = 0;
  }
  has_last_ = true;
  v_last_ = res.v;
  w_last_ = res.w;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = res.v;
  cmd.twist.angular.z = res.w;

  if (local_plan_pub_->is_activated() && local_plan_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Header h;
    h.frame_id = frame;
    h.stamp = cmd.header.stamp;
    local_plan_pub_->publish(toPathMsg(res.best.poses, h));
  }
  if (stats_pub_->is_activated()) {
    std_msgs::msg::Float64MultiArray st;
    st.data = {cycle_ms, static_cast<double>(res.n_samples), static_cast<double>(res.n_valid),
      static_cast<double>(res.n_collision), static_cast<double>(res.n_vo_rejected),
      res.vo_saturated ? 1.0 : 0.0,
      std::isfinite(res.best.ttc) ? res.best.ttc : -1.0, res.v, res.w,
      std::isfinite(res.d_goal) ? res.d_goal : -1.0, static_cast<double>(res.n_recentered),
      static_cast<double>(static_cast<int>(res.yield.state)),
      std::isfinite(res.yield.stop_distance) ? res.yield.stop_distance : -1.0,
      std::isfinite(res.yield.zone_entry) ? res.yield.zone_entry : -1e9,
      static_cast<double>(res.yield.obstacle),
      static_cast<double>(in.obstacles.size()), last_track_age_,
      // [17] kCommitted 결정 (0 none · 1 go · 2 hold · 3 retreat), [18] go 여유 [s] (없으면 −1)
      static_cast<double>(static_cast<int>(res.yield.decision)),
      std::isfinite(res.yield.margin) ? res.yield.margin : -1.0,
      // [19]~[27] 횡단 게이트 (헤더 주석 참고). 게이트가 꺼져 있으면 0 · 0 · −1 · −1e9 · 0 · 0 ·
      // −1e9 · −1e9 · 0
      static_cast<double>(static_cast<int>(res.gate.phase)),
      static_cast<double>(static_cast<int>(res.gate.reason)),
      res.gate.window_s,
      std::isfinite(res.gate.s_hold) ? res.yield.stop_distance : -1e9,
      static_cast<double>(res.gate.n_threats),
      static_cast<double>(in.gate_tracks.size()),
      std::isfinite(res.gate.s_in) ? res.yield.zone_entry : -1e9,
      std::isfinite(res.gate.s_out) ? res.yield.zone_exit : -1e9,
      static_cast<double>(res.gate.exempt_ids.size()),
      // [28]~[39] R0 계측 (docs/research/dynamic-avoidance-root-cause §6 R0).
      // 거동에 쓰이지 않는다 — 접촉률 대신 **주기당** 기제 지표를 남겨 검정력을 확보한다.
      // [28] 선택 가능 후보 수, [29] 충돌 없는 후보 수, [30] 비용 max-min (평탄도),
      // [31] 후보 롤아웃 종점 변위 스팬 [m], [32][33] 후보 v 범위, [34][35] 후보 ω 범위,
      // [36] min_k TTC₀, [37] 최근접 동적 장애물 ‖p‖, [38] escaping, [39] leave_lane,
      // [40] 게이트 **전** 계획기 출력 v (res.v 는 상한 적용 후라 따로 남긴다),
      // [41] 이 주기에 쓴 속도 상한 v_cap
      static_cast<double>(res.diag.n_selectable),
      static_cast<double>(res.diag.n_collision_free),
      res.diag.n_selectable > 0 ? res.diag.cost_max - res.diag.cost_min : -1.0,
      res.diag.disp_span,
      res.diag.v_lo, res.diag.v_hi, res.diag.w_lo, res.diag.w_hi,
      res.diag.ttc_min, res.diag.nearest_obs,
      res.diag.escaping ? 1.0 : 0.0, res.diag.leave_lane ? 1.0 : 0.0,
      res.best.v, res.v_cap};
    stats_pub_->publish(st);
  }
  return cmd;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::DWAController, nav2_core::Controller)
