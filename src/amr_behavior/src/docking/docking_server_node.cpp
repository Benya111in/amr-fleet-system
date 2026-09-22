// docking_server_node 구현 (docking_server_node.hpp 설명 참고).
#include "amr_behavior/docking/docking_server_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
  const char axis = normal_axis.empty() ? 'x' : normal_axis.back();
  const int col = axis == 'x' ? 0 : (axis == 'y' ? 1 : 2);
  double nx = rot[0][col];
  double ny = rot[1][col];
  if (negative) {
    nx = -nx;
    ny = -ny;
  }
  if (std::hypot(nx, ny) < 0.2) {
    return std::nullopt;   // 법선이 거의 수직 (평면 투영 불가) → 잘못된 관측 또는 축 규약 불일치
  }
  MarkerObservation obs;
  obs.x = pose.position.x;
  obs.y = pose.position.y;
  obs.normal_yaw = std::atan2(ny, nx);
  return obs;
}

std::vector<std::pair<double, double>> exclusionPolygon(
  const MarkerObservation & m, const ExclusionBox & box)
{
  // 마커 프레임 (x = 바깥 법선, y = 판 가로) 꼭짓점 → base_link: p = m + R(θn)·(px, py)
  const double c = std::cos(m.normal_yaw);
  const double s = std::sin(m.normal_yaw);
  const std::pair<double, double> local[4] = {
    {box.front_margin, -box.half_width}, {box.front_margin, box.half_width},
    {-box.depth, box.half_width}, {-box.depth, -box.half_width}};
  std::vector<std::pair<double, double>> out;
  for (const auto & p : local) {
    out.emplace_back(m.x + c * p.first - s * p.second, m.y + s * p.first + c * p.second);
  }
  return out;
}

double distanceFromPlate(const MarkerObservation & m)
{
  // 로봇(원점) − 마커 중심 을 바깥 법선에 사영
  return -(m.x * std::cos(m.normal_yaw) + m.y * std::sin(m.normal_yaw));
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
  p.heading_stop_tolerance = declare_parameter<double>(
    "heading_stop_tolerance", p.heading_stop_tolerance);
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
  // 계약 C3: aruco_detector_node 는 마커 모델 프레임 (+x = 판 바깥 법선, +z = 위) 으로 낸다
  normal_axis_ = declare_parameter<std::string>("marker_normal_axis", "x");
  detector_service_ = declare_parameter<std::string>("detector_enable_service", "");
  exclusion_enabled_ = declare_parameter<bool>("exclusion.enabled", true);
  exclusion_box_.half_width = declare_parameter<double>(
    "exclusion.half_width", exclusion_box_.half_width);
  exclusion_box_.depth = declare_parameter<double>("exclusion.depth", exclusion_box_.depth);
  exclusion_box_.front_margin = declare_parameter<double>(
    "exclusion.front_margin", exclusion_box_.front_margin);
  exclusion_release_distance_ = declare_parameter<double>(
    "exclusion.release_distance", exclusion_release_distance_);
  exclusion_marker_timeout_ = declare_parameter<double>(
    "exclusion.marker_timeout", exclusion_marker_timeout_);

  const auto ids = declare_parameter<std::vector<std::string>>(
    "docks.ids", std::vector<std::string>{});
  for (const auto & id : ids) {
    standoffs_[id] = declare_parameter<double>("docks." + id + ".standoff", p.standoff);
    marker_ids_[id] = static_cast<int>(declare_parameter<int>("docks." + id + ".marker_id", -1));
  }
  marker_id_max_age_ = declare_parameter<double>("marker_id_max_age", marker_id_max_age_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::QoS(1));
  exclusion_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "safety/dock_exclusion", rclcpp::QoS(10));
  marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "perception/dock_marker_pose", rclcpp::SensorDataQoS(),
    std::bind(&DockingServerNode::onMarker, this, std::placeholders::_1));
  // 검출기는 자세 다음에 id 를 낸다 → 자세가 올 때의 최근 id 는 직전 프레임 것
  // (한 마커만 보이면 같다)
  marker_id_sub_ = create_subscription<std_msgs::msg::Int32>(
    "perception/dock_marker_id", rclcpp::QoS(10),
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      last_marker_id_ = msg->data;
      last_marker_id_time_ = now().seconds();
    });
  if (!detector_service_.empty()) {
    detector_client_ = create_client<std_srvs::srv::SetBool>(detector_service_);
  }
  server_ = rclcpp_action::create_server<Dock>(
    this, "dock",
    std::bind(&DockingServerNode::onGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DockingServerNode::onCancel, this, std::placeholders::_1),
    std::bind(&DockingServerNode::onAccepted, this, std::placeholders::_1));
  // 제어·예외 발행 주기 (goal 이 없을 때는 세션 확인만 한다)
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {controlStep();});
  RCLCPP_INFO(
    get_logger(),
    "dock 서버 준비 (법칙 %s, %.0f Hz, 허용 %.3f m / %.2f°, 최대 %d 회, 마커 법선 축 %s, 예외 %s)",
    law.c_str(), control_rate_hz_, p.position_tolerance, p.angle_tolerance * 180.0 / M_PI,
    default_max_attempts_, normal_axis_.c_str(), exclusion_enabled_ ? "켬" : "끔");
}

double DockingServerNode::standoffFor(const std::string & dock_id) const
{
  const auto it = standoffs_.find(dock_id);
  return it == standoffs_.end() ? base_params_.standoff : it->second;
}

int DockingServerNode::markerIdFor(const std::string & dock_id) const
{
  const auto it = marker_ids_.find(dock_id);
  return it == marker_ids_.end() ? -1 : it->second;
}

bool DockingServerNode::wrongMarker(double t) const
{
  if (expected_marker_id_ < 0 || last_marker_id_time_ < 0.0) {
    return false;   // 도크 id 를 모르거나 검출기가 id 를 내지 않는다 (이전 규약과 호환)
  }
  // id 를 내는 검출기인데 최근 id 가 없거나 다르면 버린다 (fail-safe)
  return t - last_marker_id_time_ > marker_id_max_age_ || last_marker_id_ != expected_marker_id_;
}

rclcpp_action::GoalResponse DockingServerNode::onGoal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Dock::Goal> goal)
{
  if (active_ && active_->is_canceling()) {
    // 취소 요청을 받은 goal 은 다음 제어 주기에 끝난다 → 새 goal 이 선점
    // (BT 가 halt 직후 다시 보낸다)
    controller_->cancel();
    finish(true);
  }
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
  session_standoff_ = params.standoff;
  expected_marker_id_ = markerIdFor(goal->dock_id);
  if (!session_active_) {
    session_active_ = true;
    setDetectorEnabled(true);
  }
  RCLCPP_INFO(
    get_logger(), "도킹 시작: %s (standoff %.3f m, 최대 %d 회)", goal->dock_id.c_str(),
    params.standoff, attempts);
}

void DockingServerNode::onMarker(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!active_ && !session_active_) {
    return;
  }
  if (wrongMarker(now().seconds())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "마커 id %d ≠ 도크 마커 %d → 관측 버림", last_marker_id_,
      expected_marker_id_);
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
    if (active_) {
      pending_obs_ = obs;
    }
    last_obs_ = obs;
    last_obs_time_ = now().seconds();
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "마커 자세의 %s 축이 수평이 아니다 → 관측 버림 (marker_normal_axis 규약 확인)",
      normal_axis_.c_str());
  }
}

void DockingServerNode::controlStep()
{
  const double t = now().seconds();
  if (active_ && controller_) {
    goalStep(t);
  }
  exclusionStep(t);
}

void DockingServerNode::goalStep(double t)
{
  if (active_->is_canceling()) {
    controller_->cancel();
    finish(true);
    return;
  }
  const Command cmd = controller_->update(t, pending_obs_);
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

void DockingServerNode::exclusionStep(double t)
{
  if (!session_active_) {
    return;
  }
  // 마커 추정: goal 중에는 추적기(관측 사이 예측 포함), 끝난 뒤에는 신선한 관측
  std::optional<MarkerObservation> marker;
  if (active_ && controller_) {
    marker = controller_->markerEstimate();
  }
  const bool fresh = last_obs_ && t - last_obs_time_ <= exclusion_marker_timeout_;
  if (!marker && fresh) {
    marker = last_obs_;
  }
  if (!active_) {
    // goal 이 끝난 뒤: 판에서 standoff + release 밖으로 물러났거나 마커가 안 보이면 세션 종료
    if (!fresh ||
      distanceFromPlate(*last_obs_) > session_standoff_ + exclusion_release_distance_)
    {
      endSession();
      return;
    }
  }
  if (!exclusion_enabled_ || !marker) {
    return;
  }
  geometry_msgs::msg::PolygonStamped msg;
  msg.header.frame_id = base_frame_;
  msg.header.stamp = now();
  for (const auto & p : exclusionPolygon(*marker, exclusion_box_)) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(p.first);
    pt.y = static_cast<float>(p.second);
    msg.polygon.points.push_back(pt);
  }
  exclusion_pub_->publish(msg);
}

void DockingServerNode::endSession()
{
  session_active_ = false;
  last_obs_.reset();
  setDetectorEnabled(false);
  RCLCPP_INFO(get_logger(), "도킹 세션 종료 (예외 사각형 발행 중단)");
}

void DockingServerNode::finish(bool canceled)
{
  publishCommand(Command{});
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
      get_logger(), *get_clock(), 5000, "%s 서비스 없음 (검출기 켜기/끄기 생략)",
      detector_service_.c_str());
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  detector_client_->async_send_request(request);
}

}  // namespace docking
}  // namespace amr_behavior
