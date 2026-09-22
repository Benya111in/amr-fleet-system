// docking_server_node 구현 (docking_server_node.hpp 설명 참고).
#include "amr_behavior/docking/docking_server_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_behavior
{
namespace docking
{

std::optional<MarkerObservation> markerToObservation(
  const geometry_msgs::msg::Pose & pose, const std::string & normal_axis)
{
  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-6 || !std::isfinite(pose.position.x) ||
    !std::isfinite(pose.position.y))
  {
    return std::nullopt;
  }
  const tf2::Matrix3x3 rot(tf2::Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm));
  // 마커 축 (열 벡터) 을 base_link 에서 표현
  const bool negative = !normal_axis.empty() && normal_axis[0] == '-';
  const char axis = normal_axis.empty() ? 'z' : normal_axis.back();
  const int col = axis == 'x' ? 0 : (axis == 'y' ? 1 : 2);
  double nx = rot[0][col];
  double ny = rot[1][col];
  if (negative) {
    nx = -nx;
    ny = -ny;
  }
  if (std::hypot(nx, ny) < 0.2) {
    return std::nullopt;   // 법선이 거의 수직 (평면 투영 불가) → 잘못된 관측
  }
  MarkerObservation obs;
  obs.x = pose.position.x;
  obs.y = pose.position.y;
  obs.normal_yaw = std::atan2(ny, nx);
  return obs;
}

DockingServerNode::DockingServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_server_node", options)
{
  Params & p = base_params_;
  const std::string law = declare_parameter<std::string>("control_law", "graceful");
  if (const auto parsed = parseControlLaw(law)) {
    p.law = *parsed;
  } else {
    RCLCPP_WARN(get_logger(), "control_law '%s' 모름 → graceful", law.c_str());
  }
  p.standoff = declare_parameter<double>("standoff", p.standoff);
  p.position_tolerance = declare_parameter<double>("position_tolerance", p.position_tolerance);
  p.angle_tolerance = declare_parameter<double>("angle_tolerance", p.angle_tolerance);
  p.settle_frames = declare_parameter<int>("settle_frames", p.settle_frames);
  p.final_distance = declare_parameter<double>("final_distance", p.final_distance);
  p.stop_distance = declare_parameter<double>("stop_distance", p.stop_distance);
  p.align_threshold = declare_parameter<double>("align_threshold", p.align_threshold);
  p.max_linear_speed = declare_parameter<double>("max_linear_speed", p.max_linear_speed);
  p.final_linear_speed = declare_parameter<double>("final_linear_speed", p.final_linear_speed);
  p.min_linear_speed = declare_parameter<double>("min_linear_speed", p.min_linear_speed);
  p.max_angular_speed = declare_parameter<double>("max_angular_speed", p.max_angular_speed);
  p.search_angular_speed = declare_parameter<double>(
    "search_angular_speed", p.search_angular_speed);
  p.search_sweep = declare_parameter<double>("search_sweep", p.search_sweep);
  p.k_distance = declare_parameter<double>("k_distance", p.k_distance);
  p.k_heading = declare_parameter<double>("k_heading", p.k_heading);
  p.lookahead = declare_parameter<double>("lookahead", p.lookahead);
  p.k_phi = declare_parameter<double>("k_phi", p.k_phi);
  p.k_delta = declare_parameter<double>("k_delta", p.k_delta);
  p.beta = declare_parameter<double>("beta", p.beta);
  p.lambda = declare_parameter<double>("lambda", p.lambda);
  p.slowdown_radius = declare_parameter<double>("slowdown_radius", p.slowdown_radius);
  p.linear_deadband = declare_parameter<double>("linear_deadband", p.linear_deadband);
  p.angular_deadband = declare_parameter<double>("angular_deadband", p.angular_deadband);
  p.marker_timeout = declare_parameter<double>("marker_timeout", p.marker_timeout);
  p.search_timeout = declare_parameter<double>("search_timeout", p.search_timeout);
  p.attempt_timeout = declare_parameter<double>("attempt_timeout", p.attempt_timeout);
  p.backup_distance = declare_parameter<double>("backup_distance", p.backup_distance);
  p.backup_speed = declare_parameter<double>("backup_speed", p.backup_speed);
  p.max_overshoot = declare_parameter<double>("max_overshoot", p.max_overshoot);
  p.filter_coef = declare_parameter<double>("filter_coef", p.filter_coef);
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  p.control_period = 1.0 / std::max(1.0, control_rate_hz_);
  default_max_attempts_ = declare_parameter<int>("max_attempts", 3);
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  normal_axis_ = declare_parameter<std::string>("marker_normal_axis", "z");
  detector_node_ = declare_parameter<std::string>("detector_node", "");

  const auto ids = declare_parameter<std::vector<std::string>>(
    "docks.ids", std::vector<std::string>{});
  for (const auto & id : ids) {
    standoffs_[id] = declare_parameter<double>("docks." + id + ".standoff", p.standoff);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::QoS(1));
  marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "perception/dock_marker_pose", rclcpp::SensorDataQoS(),
    std::bind(&DockingServerNode::onMarker, this, std::placeholders::_1));
  if (!detector_node_.empty()) {
    detector_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_);
  }
  server_ = rclcpp_action::create_server<Dock>(
    this, "dock",
    std::bind(&DockingServerNode::onGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DockingServerNode::onCancel, this, std::placeholders::_1),
    std::bind(&DockingServerNode::onAccepted, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(), "dock 서버 준비 (법칙 %s, %.0f Hz, 허용 %.3f m / %.2f°, 최대 %d 회)", law.c_str(),
    control_rate_hz_, p.position_tolerance, p.angle_tolerance * 180.0 / M_PI,
    default_max_attempts_);
}

double DockingServerNode::standoffFor(const std::string & dock_id) const
{
  const auto it = standoffs_.find(dock_id);
  return it == standoffs_.end() ? base_params_.standoff : it->second;
}

rclcpp_action::GoalResponse DockingServerNode::onGoal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Dock::Goal> goal)
{
  if (active_) {
    RCLCPP_WARN(get_logger(), "도킹 실행 중 → 새 goal(%s) 거절", goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingServerNode::onCancel(
  const std::shared_ptr<GoalHandle>/*handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingServerNode::onAccepted(const std::shared_ptr<GoalHandle> handle)
{
  const auto goal = handle->get_goal();
  Params params = base_params_;
  params.standoff = standoffFor(goal->dock_id);
  const int attempts = goal->max_retries > 0 ? goal->max_retries : default_max_attempts_;
  controller_ = std::make_unique<DockingController>(params);
  controller_->start(now().seconds(), attempts);
  last_phase_ = controller_->phase();
  active_ = handle;
  pending_obs_.reset();
  setDetectorEnabled(true);
  RCLCPP_INFO(
    get_logger(), "도킹 시작: %s (standoff %.3f m, 최대 %d 회)", goal->dock_id.c_str(),
    params.standoff, attempts);
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {controlStep();});
}

void DockingServerNode::onMarker(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!active_) {
    return;
  }
  geometry_msgs::msg::Pose pose = msg->pose;
  const std::string & frame = msg->header.frame_id;
  if (!frame.empty() && frame != base_frame_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, frame, tf2::TimePointZero);
      tf2::doTransform(msg->pose, pose, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "마커 프레임 %s → %s 변환 실패: %s", frame.c_str(),
        base_frame_.c_str(), e.what());
      return;
    }
  }
  if (auto obs = markerToObservation(pose, normal_axis_)) {
    pending_obs_ = obs;
  }
}

void DockingServerNode::controlStep()
{
  if (!active_ || !controller_) {
    return;
  }
  if (active_->is_canceling()) {
    controller_->cancel();
    finish(true);
    return;
  }
  const Command cmd = controller_->update(now().seconds(), pending_obs_);
  pending_obs_.reset();
  publishCommand(cmd);
  const Phase phase = controller_->phase();
  if (phase == Phase::kBackup && last_phase_ != Phase::kBackup) {
    RCLCPP_WARN(
      get_logger(), "도킹 시도 %d 실패 (%s) → 후진 후 재시도", controller_->attempt(),
      controller_->failureReason().c_str());
  }
  last_phase_ = phase;

  auto feedback = std::make_shared<Dock::Feedback>();
  feedback->current_phase = phaseName(controller_->phase());
  const double remaining = controller_->distanceRemaining();
  feedback->distance_remaining = static_cast<float>(std::isfinite(remaining) ? remaining : -1.0);
  feedback->attempt = static_cast<uint8_t>(controller_->attempt());
  active_->publish_feedback(feedback);

  if (controller_->finished()) {
    finish(false);
  }
}

void DockingServerNode::finish(bool canceled)
{
  publishCommand(Command{});
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  setDetectorEnabled(false);
  auto result = std::make_shared<Dock::Result>();
  const DockErrors & e = controller_->errors();
  result->success = controller_->succeeded();
  result->final_position_error = static_cast<float>(e.valid() ? e.position() : -1.0);
  result->final_angle_error = static_cast<float>(e.valid() ? std::fabs(e.heading) : -1.0);
  result->attempts_used = static_cast<uint8_t>(controller_->attempt());
  const double angle_deg = e.valid() ? std::fabs(e.heading) * 180.0 / M_PI : -1.0;
  const unsigned attempts = result->attempts_used;
  if (canceled) {
    active_->canceled(result);
    RCLCPP_INFO(get_logger(), "도킹 취소");
  } else if (result->success) {
    active_->succeed(result);
    RCLCPP_INFO(
      get_logger(), "도킹 성공: 위치 %.4f m, 각도 %.3f°, 시도 %u",
      result->final_position_error, angle_deg, attempts);
  } else {
    active_->abort(result);
    RCLCPP_WARN(
      get_logger(), "도킹 실패 (%s): 위치 %.4f m, 각도 %.3f°, 시도 %u",
      controller_->failureReason().c_str(), result->final_position_error, angle_deg, attempts);
  }
  active_.reset();
}

void DockingServerNode::publishCommand(const Command & cmd)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.linear;
  twist.angular.z = cmd.angular;
  cmd_pub_->publish(twist);
}

void DockingServerNode::setDetectorEnabled(bool enabled)
{
  if (!detector_client_) {
    return;
  }
  if (!detector_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%s 파라미터 서비스 없음 (enabled 전환 생략)",
      detector_node_.c_str());
    return;
  }
  detector_client_->set_parameters({rclcpp::Parameter("enabled", enabled)});
}

}  // namespace docking
}  // namespace amr_behavior
