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
  const std::size_t begin = fresh_ ? 0 : hint_;
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
  double controller_frequency = 20.0;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", controller_frequency);
  }
  c.control_period = 1.0 / std::max(1.0, controller_frequency);
  c.vx_samples = param(node, p + "vx_samples", 11);
  c.vth_samples = param(node, p + "vth_samples", 31);
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
  robot_mass_ = param(node, p + "robot_mass", 47.6);
  path_horizon_ = param(node, p + "path_horizon", 6.0);
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
  if (age > track_timeout_ || age < -1.0) {
    return out;
  }
  core::Pose2D T;
  if (!lookupTransform2D(*tf_, frame, msg->header.frame_id, 0.0, T)) {
    return out;
  }
  const double c = std::cos(T.theta);
  const double s = std::sin(T.theta);
  const double dt = std::max(0.0, age);
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
    out.push_back(d);
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
  const std::vector<core::Pose2D> path = plan_.window(T, in.pose, path_horizon_, end_is_goal);
  in.path = &path;
  in.path_end_is_goal = end_is_goal;
  in.v_meas = velocity.linear.x;
  in.w_meas = velocity.angular.z;
  in.has_last = has_last_;
  in.v_last = v_last_;
  in.w_last = w_last_;
  in.speed_limit = speed_limit_;
  in.obstacles = obstaclesInFrame(frame);

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
      std::isfinite(res.d_goal) ? res.d_goal : -1.0, static_cast<double>(res.n_recentered)};
    stats_pub_->publish(st);
  }
  return cmd;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::DWAController, nav2_core::Controller)
