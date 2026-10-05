#include "amr_navigation/velocity_profiler_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace amr_navigation
{

VelocityProfilerNode::VelocityProfilerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("velocity_profiler_node", options)
{
  // robot_params.yaml 공통 한계 (같은 파일을 여러 노드가 읽는다)
  const double v_max = declare_parameter("limits.max_linear_velocity", 2.0);
  const double v_min = declare_parameter("limits.min_linear_velocity", -0.5);
  const double a_max = declare_parameter("limits.max_linear_acceleration", 1.0);
  const double w_max = declare_parameter("limits.max_angular_velocity", 1.5);
  const double alpha_max = declare_parameter("limits.max_angular_acceleration", 2.0);
  const double j_max = declare_parameter("limits.max_linear_jerk", 2.0);
  const double base_mass = declare_parameter("robot.base_mass", 45.0);
  const double wheel_mass = declare_parameter("robot.wheel_mass", 1.0);
  const double caster_mass = declare_parameter("robot.caster_mass", 0.3);
  robot_mass_ = base_mass + 2.0 * wheel_mass + 2.0 * caster_mass;

  // 노드 고유 (config/velocity_profiler.yaml)
  const double rate = declare_parameter("rate", 50.0);
  period_ = 1.0 / std::max(1.0, rate);
  cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
  odom_timeout_ = declare_parameter("odom_timeout", 0.2);
  idle_publish_time_ = declare_parameter("idle_publish_time", 1.0);

  core::VelocityProfilerConfig c;
  c.linear = {v_min, v_max, a_max, declare_parameter("jerk_limit_enabled", true) ? j_max : 0.0};
  c.angular = {-w_max, w_max, alpha_max, declare_parameter("max_angular_jerk", 6.0)};
  c.preserve_curvature = declare_parameter("preserve_curvature", true);
  c.curvature_min_speed = declare_parameter("curvature_min_speed", 0.05);
  c.use_pid = declare_parameter("use_pid", true);
  c.use_reference_model = declare_parameter("use_reference_model", true);
  c.model_time_constant = declare_parameter("model_time_constant", 0.08);
  c.model_delay = declare_parameter("model_delay", 0.04);
  c.nominal_period = period_;
  auto pid = [this](const std::string & ax, double kp, double ki) {
      core::PidConfig p;
      p.kp = declare_parameter("pid." + ax + ".kp", kp);
      p.ki = declare_parameter("pid." + ax + ".ki", ki);
      p.kd = declare_parameter("pid." + ax + ".kd", 0.0);
      p.setpoint_weight = declare_parameter("pid." + ax + ".setpoint_weight", 1.0);
      const double tt_default = p.kp > 0.0 && p.ki > 0.0 ? std::sqrt(p.kp / p.ki) : 0.1;
      p.tracking_time = declare_parameter("pid." + ax + ".tracking_time", tt_default);
      p.derivative_tau = declare_parameter("pid." + ax + ".derivative_tau", 0.02);
      return p;
    };
  c.pid_linear = pid("linear", 0.4, 2.0);
  c.pid_angular = pid("angular", 0.4, 2.0);
  c.max_linear_correction = declare_parameter("max_linear_correction", 0.2);
  c.max_angular_correction = declare_parameter("max_angular_correction", 0.3);
  c.resync_threshold_v = declare_parameter("resync_threshold_v", 0.4);
  c.resync_threshold_w = declare_parameter("resync_threshold_w", 0.6);
  c.hold_error_v = declare_parameter("hold_error_v", 0.15);
  c.hold_time = declare_parameter("hold_time", 0.3);
  c.measurement_filter_time = declare_parameter("measurement_filter_time", 0.04);
  c.stop_deadband = declare_parameter("stop_deadband", 1e-3);
  profiler_.setConfig(c);

  const auto cmd_qos = rclcpp::QoS(1).reliable();
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_nav", cmd_qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {onCmd(msg);});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", rclcpp::QoS(5),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(msg);});
  payload_sub_ = create_subscription<std_msgs::msg::Float32>(
    "payload/mass", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {onPayload(msg);});
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", rclcpp::QoS(1));
  state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "velocity_profiler/state", rclcpp::QoS(10));
  // 노드 시계(use_sim_time 이면 /clock) 기반 타이머: 시뮬레이션
  //   실시간 계수와 무관하게 sim 시각 50 Hz
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(period_), [this]() {onTimer();});
  RCLCPP_INFO(
    get_logger(),
    "velocity_profiler_node: %.0f Hz, v [%.2f, %.2f] a %.2f j %.2f, w %.2f alpha %.2f, "
    "PI kp %.2f ki %.2f (model T %.3f s, delay %.3f s)", rate, v_min, v_max, a_max, j_max, w_max,
    alpha_max, c.pid_linear.kp, c.pid_linear.ki, c.model_time_constant, c.model_delay);
}

void VelocityProfilerNode::onCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool idle = !has_cmd_ || idle_time_ > idle_publish_time_;
  v_target_ = msg->linear.x;
  w_target_ = msg->angular.z;
  last_cmd_time_ = now();
  has_cmd_ = true;
  if (idle && (v_target_ != 0.0 || w_target_ != 0.0)) {
    // 유휴에서 첫 명령: 타이머 위상(최대 1 주기)을 기다리지 않고 바로 한 주기
    // → 이후 타이머는 여기서 다시 센다
    idle_time_ = 0.0;
    has_tick_ = false;
    tickLocked();
    timer_->reset();
  }
}

void VelocityProfilerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  v_meas_ = msg->twist.twist.linear.x;
  w_meas_ = msg->twist.twist.angular.z;
  last_odom_time_ = now();
  odom_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
  has_odom_ = true;
}

void VelocityProfilerNode::onPayload(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double m = std::max(0.0, static_cast<double>(msg->data));
  std::lock_guard<std::mutex> lock(mutex_);
  profiler_.setPayloadScale(robot_mass_ / (robot_mass_ + m));
  RCLCPP_INFO(
    get_logger(), "payload %.1f kg -> accel/jerk scale %.3f", m, profiler_.payloadScale());
}

void VelocityProfilerNode::onTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tickLocked();
}

void VelocityProfilerNode::tickLocked()
{
  const rclcpp::Time t = now();
  double dt = period_;
  if (has_tick_) {
    dt = (t - last_tick_).seconds();
    if (dt <= 1e-6) {
      return;   // 시각이 흐르지 않음 (시뮬레이션 정지) → 필터를 전진시키지 않는다
    }
    if (dt > 5.0 * period_) {
      dt = period_;   // 시각 점프(재시작) 방어
    }
    // 늦게 온 주기는 실제 경과로 적분한다 (프로파일이 시뮬레이션 시각을 따라간다).
    // 주기가 들쭉날쭉해도 명령의 수치 저크가 한계를 넘지 않는 것은 코어 출력 성형이 맡는다
    // (VelocityProfiler::shapeOutput)
  }
  last_tick_ = t;
  has_tick_ = true;

  // 명령 타임아웃 → 목표 0 (감속 프로파일로 정지)
  if (has_cmd_ && (t - last_cmd_time_).seconds() > cmd_timeout_) {
    v_target_ = 0.0;
    w_target_ = 0.0;
  }
  const bool meas_ok = has_odom_ && (t - last_odom_time_).seconds() <= odom_timeout_;
  const bool fresh = odom_stamp_ != used_odom_stamp_;
  const core::ProfilerOutput out =
    profiler_.update(v_target_, w_target_, meas_ok, v_meas_, w_meas_, dt, fresh);
  used_odom_stamp_ = odom_stamp_;

  // 유휴: 목표·출력이 0 인 상태가 idle_publish_time 이상 이어지면
  //   발행을 멈춘다(다른 발행자 방해 금지)
  const bool zero = out.v == 0.0 && out.w == 0.0 && v_target_ == 0.0 && w_target_ == 0.0;
  idle_time_ = zero ? idle_time_ + dt : 0.0;
  if (!has_cmd_ || idle_time_ > idle_publish_time_) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = out.v;
  cmd.angular.z = out.w;
  cmd_pub_->publish(cmd);

  const double a_out = (out.v - last_v_out_) / dt;
  if (state_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float64MultiArray st;
    const double jerk = (out.a_ref - last_a_ref_) / dt;
    const double jerk_out = (a_out - last_a_out_) / dt;
    st.data = {v_target_, w_target_, out.v_ref, out.w_ref, out.a_ref, out.v, out.w, v_meas_,
      w_meas_, jerk, out.resynced ? 1.0 : 0.0, jerk_out, out.a_out, out.correction,
      static_cast<double>(profiler_.resyncCount()), fresh ? 1.0 : 0.0};
    state_pub_->publish(st);
  }
  last_a_ref_ = out.a_ref;
  last_v_out_ = out.v;
  last_a_out_ = a_out;
}

}  // namespace amr_navigation
