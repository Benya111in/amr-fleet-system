#include "amr_navigation/pure_pursuit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/ros_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PurePursuitController: parent node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  const std::string p = name_ + ".";
  core::PurePursuitConfig c;
  c.desired_linear_vel = param(node, p + "desired_linear_vel", 1.0);
  c.lookahead_time = param(node, p + "lookahead_time", 0.8);
  c.min_lookahead = param(node, p + "min_lookahead_dist", 0.4);
  c.max_lookahead = param(node, p + "max_lookahead_dist", 1.8);
  // 한계 기본값은 robot_params.yaml limits.* (단일 출처), 플러그인 파라미터가 있으면 우선
  const double a_lin = param(node, "limits.max_linear_acceleration", 1.0);
  c.max_angular_vel = param(
    node, p + "max_angular_vel", param(node, "limits.max_angular_velocity", 1.5));
  c.max_angular_accel = param(
    node, p + "max_angular_accel", param(node, "limits.max_angular_acceleration", 2.0));
  c.max_linear_accel = param(node, p + "max_linear_accel", a_lin);
  c.max_linear_decel = param(node, p + "max_linear_decel", a_lin);
  c.max_lateral_accel = param(node, p + "max_lateral_accel", 0.8);
  c.max_linear_jerk =
    param(node, p + "max_linear_jerk", param(node, "limits.max_linear_jerk", 2.0));
  c.min_approach_vel = param(node, p + "min_approach_linear_velocity", 0.05);
  c.approach_latency = param(node, p + "approach_latency", 0.3);
  c.curvature_window = param(node, p + "curvature_window", 0.25);
  c.use_rotate_to_heading = param(node, p + "use_rotate_to_heading", true);
  c.rotate_to_heading_min_angle = param(node, p + "rotate_to_heading_min_angle", 0.785);
  c.rotate_to_heading_max_v = param(node, p + "rotate_to_heading_max_v", 0.1);
  c.goal_align_distance = param(node, p + "goal_align_distance", 0.08);
  c.goal_yaw_tolerance = param(node, p + "goal_yaw_tolerance", 0.02);
  c.use_chord_correction = param(node, p + "use_chord_correction", false);
  c.chord_gain = param(node, p + "chord_gain", 1.0);
  window_reset_v_ = param(node, p + "velocity_reset_threshold", 0.3);
  use_collision_detection_ = param(node, p + "use_collision_detection", true);
  collision_check_time_ = param(node, p + "collision_check_time", 1.0);
  transform_tolerance_ = param(node, p + "transform_tolerance", 0.2);
  robot_mass_ = param(node, p + "robot_mass", 47.6);
  const std::string payload_topic = param(node, p + "payload_topic", std::string("payload/mass"));
  base_config_ = c;
  pp_.setConfig(c);

  payload_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    payload_topic, rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {onPayload(msg);});
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("pure_pursuit/stats", 10);
  RCLCPP_INFO(
    logger_,
    "PurePursuitController '%s': v %.2f m/s, L = clamp(%.2f v, %.2f, %.2f) m, chord correction %s",
    name_.c_str(), c.desired_linear_vel, c.lookahead_time, c.min_lookahead, c.max_lookahead,
    c.use_chord_correction ? "on" : "off");
}

void PurePursuitController::cleanup()
{
  payload_sub_.reset();
  local_plan_pub_.reset();
  carrot_pub_.reset();
  stats_pub_.reset();
}

void PurePursuitController::activate()
{
  local_plan_pub_->on_activate();
  carrot_pub_->on_activate();
  stats_pub_->on_activate();
  has_last_ = false;
}

void PurePursuitController::deactivate()
{
  local_plan_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  stats_pub_->on_deactivate();
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_frame_ = path.header.frame_id;
  pp_.setPath(toPoses2D(path));
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit <= 0.0 || speed_limit >= 100.0 - 1e-9) {
    speed_limit_ = 1e9;
  } else {
    speed_limit_ = percentage ? base_config_.desired_linear_vel * speed_limit / 100.0 : speed_limit;
  }
}

void PurePursuitController::onPayload(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double m = std::max(0.0, static_cast<double>(msg->data));
  const double s = robot_mass_ / (robot_mass_ + m);
  std::lock_guard<std::mutex> lock(mutex_);
  core::PurePursuitConfig c = base_config_;
  c.max_linear_accel *= s;
  c.max_linear_decel *= s;
  c.max_angular_accel *= s;
  c.max_lateral_accel *= s;
  pp_.setConfig(c);
}

bool PurePursuitController::arcCollides(const core::Pose2D & start, double v, double w) const
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  const core::CostGrid grid = costmapView(*costmap);
  const core::FootprintChecker checker(
    grid, footprintOf(*costmap_ros_), circumscribedCost(*costmap_ros_), true);
  // 제동 거리만큼은 반드시 본다: T = max(검사 시간, v / a_dec)
  const double T = std::max(
    collision_check_time_, std::abs(v) / std::max(1e-3, base_config_.max_linear_decel));
  const double dt = 0.1;
  const int n = static_cast<int>(std::ceil(T / dt));
  for (int k = 1; k <= n; ++k) {
    if (checker.collides(core::integrateArc(start, v, w, k * dt))) {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (pp_.path().empty()) {
    throw nav2_core::PlannerException("PurePursuitController: no plan");
  }
  // 로봇 자세를 경로 프레임으로
  core::Pose2D T;
  if (!lookupTransform2D(*tf_, plan_frame_, pose.header.frame_id, transform_tolerance_, T)) {
    throw nav2_core::PlannerException(
            "PurePursuitController: cannot transform '" + pose.header.frame_id + "' to '" +
            plan_frame_ + "'");
  }
  const core::Pose2D robot_local = toPose2D(pose.pose);
  const core::Pose2D robot = core::compose(T, robot_local);
  // look-ahead 기준 속도: 직전 명령 (측정과 크게 다르면 측정)
  double v_now = velocity.linear.x;
  if (has_last_ && std::abs(v_last_ - velocity.linear.x) <= window_reset_v_) {
    v_now = v_last_;
  }
  const core::PurePursuitOutput out = pp_.compute(robot, v_now, speed_limit_);
  if (out.mode == core::PurePursuitMode::kNoPath) {
    throw nav2_core::PlannerException("PurePursuitController: empty path");
  }
  if (use_collision_detection_ && out.mode == core::PurePursuitMode::kTrack &&
    arcCollides(robot_local, out.v, out.w))
  {
    has_last_ = false;
    throw nav2_core::PlannerException("PurePursuitController: collision ahead on commanded arc");
  }
  has_last_ = true;
  v_last_ = out.v;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = out.v;
  cmd.twist.angular.z = out.w;

  const double cycle_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (carrot_pub_->is_activated() && carrot_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PointStamped pt;
    pt.header.frame_id = plan_frame_;
    pt.header.stamp = cmd.header.stamp;
    pt.point.x = out.carrot.x;
    pt.point.y = out.carrot.y;
    carrot_pub_->publish(pt);
  }
  if (local_plan_pub_->is_activated() && local_plan_pub_->get_subscription_count() > 0) {
    std::vector<core::Pose2D> arc;
    for (int k = 0; k <= 15; ++k) {
      arc.push_back(core::integrateArc(robot_local, out.v, out.w, 0.1 * k));
    }
    std_msgs::msg::Header h;
    h.frame_id = pose.header.frame_id;
    h.stamp = cmd.header.stamp;
    local_plan_pub_->publish(toPathMsg(arc, h));
  }
  if (stats_pub_->is_activated()) {
    std_msgs::msg::Float64MultiArray st;
    st.data = {cycle_ms, out.cte, out.lookahead, out.curvature, out.v, out.w,
      static_cast<double>(static_cast<int>(out.mode)), out.d_goal};
    stats_pub_->publish(st);
  }
  return cmd;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::PurePursuitController, nav2_core::Controller)
