// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// safety_node — cmd_vel 최종 게이트 (components.md §3.4 / §4.1 / §5.4, sequences.md §2).
//
// Sub  cmd_vel_smoothed (Twist)                 → 제한 후 cmd_vel 로 (유일한 cmd_vel 발행자)
//      scan_filtered (LaserScan, sensor)        → 풋프린트 모서리 거리 존 판정 + LiDAR 생존
//      perception/tracked_obstacles             → 최소 TTC 기반 연속 감속
//      estop, /fleet/estop (Bool, transient_local) → E-stop 래치
//      imu/data, wheel_odom, camera/camera_info, camera/depth/camera_info → 센서 생존 감시
//      safety/dock_exclusion (PolygonStamped, 선택) → 도킹 판 근접 예외 (docking 브리프 제안)
// Pub  cmd_vel (50 Hz), safety/estop_active (Bool, latched),
//      safety/zone (UInt8 0 CLEAR · 1 WARNING · 2 CRITICAL · 3 STOP, latched, 변화 시 —
//        components.md §5.4 계약, fleet_adapter_node 가 구독),
//      safety/zone_name (String "CLEAR|WARNING|CRITICAL|STOP", latched, 변화 시 — 표시용),
//      diagnostics (DiagnosticArray, 1 Hz + 변화 시)
// Srv  safety/reset_estop (std_srvs/Trigger)
//
// 판정 로직은 amr_perception/safety_gate.hpp (ROS 비의존, gtest) 에 있다. 이 노드는 메시지 변환,
// TF(lidar → base_link), 타이밍(50 Hz + 이벤트 즉시 발행)만 담당한다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_perception/nodes.hpp"
#include "amr_perception/ros_conversions.hpp"
#include "amr_perception/safety_gate.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_perception
{

namespace
{
struct SensorDefault
{
  const char * name;
  const char * topic;
  double timeout;
  const char * action;
};

// robot_params.yaml safety.sensor_timeouts 기본값과 components.md §5.4 구독 토픽
const SensorDefault kSensorDefaults[] = {
  {"lidar", "scan_filtered", 0.3, "stop"},
  {"wheel_encoder", "wheel_odom", 0.06, "stop"},
  {"imu", "imu/data", 0.05, "degraded"},
  {"rgb_camera", "camera/camera_info", 0.1, "degraded"},
  {"depth_camera", "camera/depth/camera_info", 0.2, "degraded"},
};
}  // namespace

class SafetyNode : public rclcpp::Node
{
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("safety_node", options)
  {
    SafetyParams p;
    // config/robot_params.yaml
    p.footprint_length = declare_parameter<double>("robot.footprint_length", 0.60);
    p.footprint_width = declare_parameter<double>("robot.footprint_width", 0.40);
    p.max_linear_velocity = declare_parameter<double>("limits.max_linear_velocity", 2.0);
    p.min_linear_velocity = declare_parameter<double>("limits.min_linear_velocity", -0.5);
    p.max_angular_velocity = declare_parameter<double>("limits.max_angular_velocity", 1.5);
    p.max_deceleration = declare_parameter<double>("limits.max_linear_acceleration", 1.0);
    const std::string reference =
      declare_parameter<std::string>("safety.distance_reference", "footprint_edge");
    if (reference != "footprint_edge") {
      RCLCPP_WARN(
        get_logger(), "safety.distance_reference=%s 미지원 — footprint_edge 로 판정한다",
        reference.c_str());
    }
    p.emergency_stop_distance = declare_parameter<double>("safety.emergency_stop_distance", 0.30);
    p.critical_zone_distance = declare_parameter<double>("safety.critical_zone_distance", 0.50);
    p.warning_zone_distance = declare_parameter<double>("safety.warning_zone_distance", 1.00);
    p.warning_zone_max_speed = declare_parameter<double>("safety.warning_zone_max_speed", 0.5);
    p.critical_zone_max_speed = declare_parameter<double>("safety.critical_zone_max_speed", 0.2);
    p.reaction_latency = declare_parameter<double>("safety.reaction_latency", 0.15);
    p.clearance_speed_limit_enabled =
      declare_parameter<bool>("safety.clearance_speed_limit_enabled", true);
    p.degraded_mode_max_speed = declare_parameter<double>("safety.degraded_mode_max_speed", 0.2);
    // src/amr_perception/config/perception.yaml (safety_node)
    p.stop_release_distance = declare_parameter<double>("stop_release_distance", 0.50);
    p.zone_hysteresis = declare_parameter<double>("zone_hysteresis", 0.05);
    p.ttc_limit_enabled = declare_parameter<bool>("ttc.enabled", true);
    p.ttc_critical = declare_parameter<double>("ttc.critical", 2.15);
    p.ttc_max_age = declare_parameter<double>("ttc.max_age", 0.5);
    p.command_timeout = declare_parameter<double>("command_timeout", 0.5);
    p.estop_release_requires_reset = declare_parameter<bool>("estop_release_requires_reset", true);
    p.reset_grace = declare_parameter<double>("reset_grace", 1.0);
    const std::string region = declare_parameter<std::string>("zone_region", "omni");
    p.zone_region = region == "motion" ? ZoneRegion::kMotion : ZoneRegion::kOmni;
    p.lateral_stop_distance = declare_parameter<double>("lateral_stop_distance", 0.05);
    p.self_filter_margin = declare_parameter<double>("self_filter_margin", 0.02);
    p.allow_escape = declare_parameter<bool>("allow_escape", true);
    p.escape_horizon = declare_parameter<double>("escape_horizon", 0.5);
    p.exclusion_stop_distance = declare_parameter<double>("exclusion_stop_distance", 0.10);
    p.exclusion_timeout = declare_parameter<double>("exclusion_timeout", 0.3);
    rate_hz_ = declare_parameter<double>("rate", 50.0);
    base_frame_ = prefixedFrame(
      declare_parameter<std::string>("frame_prefix", ""),
      declare_parameter<std::string>("base_frame", "base_link"));

    // 센서 감시: timeout <= 0 이면 감시하지 않는다
    std::vector<SensorWatch> watches;
    std::map<std::string, std::string> topics;
    for (const auto & d : kSensorDefaults) {
      const std::string name = d.name;
      SensorWatch w;
      w.name = name;
      w.timeout = declare_parameter<double>("safety.sensor_timeouts." + name, d.timeout);
      const std::string action = declare_parameter<std::string>(
        "sensor_actions." + name, d.action);
      w.action = action == "degraded" ? SensorFailureAction::kDegraded : SensorFailureAction::kStop;
      topics[name] = declare_parameter<std::string>("sensor_topics." + name, d.topic);
      if (w.timeout > 0.0) {
        watches.push_back(w);
      }
    }
    gate_ = std::make_unique<SafetyGate>(p, watches, now().seconds());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("topics.cmd_out", "cmd_vel"), rclcpp::QoS(10).reliable());
    zone_pub_ = create_publisher<std_msgs::msg::UInt8>(
      declare_parameter<std::string>("topics.zone", "safety/zone"), latched);
    zone_name_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("topics.zone_name", "safety/zone_name"), latched);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("topics.estop_active", "safety/estop_active"), latched);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      declare_parameter<std::string>("topics.diagnostics", "diagnostics"), 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("topics.cmd_in", "cmd_vel_smoothed"),
      rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        gate_->setCommand(msg->linear.x, msg->angular.z, now().seconds());
        step(Trigger::kOnChange);
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topics["lidar"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(*msg);});
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topics["wheel_encoder"], rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr) {heartbeat("wheel_encoder");});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      topics["imu"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {heartbeat("imu");});
    rgb_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      topics["rgb_camera"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {heartbeat("rgb_camera");});
    depth_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      topics["depth_camera"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {heartbeat("depth_camera");});
    tracked_sub_ = create_subscription<amr_msgs::msg::TrackedObstacleArray>(
      declare_parameter<std::string>("topics.tracked_obstacles", "perception/tracked_obstacles"),
      rclcpp::QoS(10).reliable(),
      [this](amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg) {
        double ttc = std::numeric_limits<double>::infinity();
        for (const auto & o : msg->obstacles) {
          if (!std::isnan(o.time_to_collision)) {
            ttc = std::min(ttc, static_cast<double>(o.time_to_collision));
          }
        }
        gate_->setMinTtc(ttc, now().seconds());
      });
    // E-stop: 대시보드가 transient_local 로 발행 → 늦게 떠도 마지막 값을 받는다
    for (const auto & [source, topic] : std::vector<std::pair<std::string, std::string>>{
        {"estop", declare_parameter<std::string>("topics.estop", "estop")},
        {"fleet_estop", declare_parameter<std::string>("topics.fleet_estop", "/fleet/estop")}})
    {
      const std::string src = source;
      estop_subs_.push_back(
        create_subscription<std_msgs::msg::Bool>(
          topic, latched, [this, src](std_msgs::msg::Bool::ConstSharedPtr msg) {
            gate_->setEstopSource(src, msg->data, now().seconds());
            RCLCPP_WARN(
              get_logger(), "E-stop 입력 %s = %s", src.c_str(), msg->data ? "true" : "false");
            step(Trigger::kOnChange);
          }));
    }
    exclusion_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      declare_parameter<std::string>("topics.dock_exclusion", "safety/dock_exclusion"), 10,
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {onExclusion(*msg);});
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("topics.reset_estop", "safety/reset_estop"),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const ResetResult r = gate_->requestReset(now().seconds());
        res->success = r.success;
        res->message = r.message;
        RCLCPP_WARN(
          get_logger(), "reset_estop: success=%d %s", r.success, r.message.c_str());
        step(Trigger::kOnChange);
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {step(Trigger::kTimer);});
    RCLCPP_INFO(
      get_logger(),
      "안전 게이트 %.0f Hz: 존 %.2f/%.2f/%.2f m, 상한 %.2f/%.2f m/s, τ_crit %.2f s, 영역 %s, "
      "감시 센서 %zu 개", rate_hz_, p.emergency_stop_distance, p.critical_zone_distance,
      p.warning_zone_distance, p.warning_zone_max_speed, p.critical_zone_max_speed, p.ttc_critical,
      region.c_str(), watches.size());
  }

private:
  /// 발행 시점: kTimer 매 주기, kOnChange 존/정지 상태가 바뀐 경우만 즉시
  enum class Trigger { kOnChange, kTimer };

  void heartbeat(const std::string & name)
  {
    gate_->sensorHeartbeat(name, now().seconds());
  }

  bool baseFromFrame(const std::string & frame, Pose2D & out)
  {
    if (frame.empty() || frame == base_frame_) {
      out = Pose2D{};
      return true;
    }
    auto it = static_tf_cache_.find(frame);
    if (it != static_tf_cache_.end()) {
      out = it->second;
      return true;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      out = pose2DFromTransform(tf);
      static_tf_cache_[frame] = out;  // 센서 장착은 고정 조인트 → 한 번만 조회
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s <- %s 실패: %s", base_frame_.c_str(),
        frame.c_str(), e.what());
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    Pose2D base_from_lidar;
    if (!baseFromFrame(scan.header.frame_id, base_from_lidar)) {
      return;  // TF 가 없으면 생존 신호도 주지 않는다 → 타임아웃 시 정지 (보수적)
    }
    std::vector<Vec2> pts;
    pts.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      pts.push_back(base_from_lidar.apply(Vec2(r * std::cos(a), r * std::sin(a))));
    }
    const double t = now().seconds();
    gate_->setScanPoints(pts, t);
    heartbeat("lidar");
    step(Trigger::kOnChange);  // 존/정지 상태가 바뀌면 다음 주기를 기다리지 않고 즉시 발행
  }

  void onExclusion(const geometry_msgs::msg::PolygonStamped & msg)
  {
    Pose2D tf;
    if (!baseFromFrame(msg.header.frame_id, tf)) {
      return;
    }
    std::vector<Vec2> poly;
    for (const auto & pt : msg.polygon.points) {
      poly.push_back(tf.apply(Vec2(pt.x, pt.y)));
    }
    gate_->setExclusionPolygon(poly, now().seconds());
  }

  /// 판정 + 발행. 타이머가 정확히 rate(50 Hz)로 발행하고, 이벤트(스캔·명령·E-stop·reset)는
  /// 존/정지 상태가 바뀐 경우에만 다음 주기를 기다리지 않고 즉시 한 번 더 발행한다.
  void step(Trigger trigger)
  {
    const rclcpp::Time t = now();
    const SafetyStatus st = gate_->evaluate(t.seconds());
    if (trigger == Trigger::kOnChange && published_once_ && st.zone == last_zone_ &&
      st.estop_active == last_estop_)
    {
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = st.command.linear;
    cmd.angular.z = st.command.angular;
    cmd_pub_->publish(cmd);

    const bool changed = !published_once_ || st.zone != last_zone_ ||
      st.estop_active != last_estop_ || st.degraded != last_degraded_ ||
      st.failed_sensors.size() != last_failed_;
    if (!published_once_ || st.zone != last_zone_) {
      std_msgs::msg::UInt8 level;
      level.data = static_cast<uint8_t>(st.zone);
      zone_pub_->publish(level);
      std_msgs::msg::String name;
      name.data = zoneName(st.zone);
      zone_name_pub_->publish(name);
    }
    if (!published_once_ || st.estop_active != last_estop_) {
      std_msgs::msg::Bool b;
      b.data = st.estop_active;
      estop_pub_->publish(b);
      if (st.estop_active) {
        RCLCPP_WARN(get_logger(), "정지 활성: %s", joinReasons(st).c_str());
      } else {
        RCLCPP_INFO(get_logger(), "정지 해제");
      }
    }
    if (changed || (t - last_diag_).seconds() >= 1.0) {
      publishDiagnostics(st, t);
      last_diag_ = t;
    }
    published_once_ = true;
    last_zone_ = st.zone;
    last_estop_ = st.estop_active;
    last_degraded_ = st.degraded;
    last_failed_ = st.failed_sensors.size();
  }

  static std::string joinReasons(const SafetyStatus & st)
  {
    std::string s;
    for (const auto & r : st.reasons) {
      s += (s.empty() ? "" : ",") + r;
    }
    return s.empty() ? "-" : s;
  }

  void publishDiagnostics(const SafetyStatus & st, const rclcpp::Time & t)
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = t;
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = std::string(get_namespace()) + "/safety_node";
    ds.hardware_id = get_namespace();
    if (st.estop_active) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      ds.message = "정지: " + joinReasons(st);
    } else if (st.degraded || st.zone != SafetyZone::kClear) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      ds.message = std::string(zoneName(st.zone)) + (st.degraded ? " / 저속(센서 고장)" : "");
    } else {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      ds.message = "CLEAR";
    }
    auto kv = [&ds](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        ds.values.push_back(e);
      };
    std::string failed;
    for (const auto & f : st.failed_sensors) {
      failed += (failed.empty() ? "" : ",") + f;
    }
    kv("zone", zoneName(st.zone));
    kv("min_distance_m", std::to_string(st.min_distance));
    kv("min_ttc_s", std::to_string(st.min_ttc));
    kv("speed_limit_mps", std::to_string(st.speed_limit));
    kv("estop_latched", st.estop_latched ? "true" : "false");
    kv("proximity_stop", st.proximity_stop ? "true" : "false");
    kv("failed_sensors", failed.empty() ? "-" : failed);
    kv("reasons", joinReasons(st));
    arr.status.push_back(ds);
    diag_pub_->publish(arr);
  }

  double rate_hz_{50.0};
  std::string base_frame_;
  std::unique_ptr<SafetyGate> gate_;
  std::map<std::string, Pose2D> static_tf_cache_;
  rclcpp::Time last_diag_{0, 0, RCL_ROS_TIME};
  bool published_once_{false};
  SafetyZone last_zone_{SafetyZone::kClear};
  bool last_estop_{false};
  bool last_degraded_{false};
  std::size_t last_failed_{0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr zone_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_sub_;
  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr tracked_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> estop_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr exclusion_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::Node::SharedPtr createSafetyNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<SafetyNode>(options);
}

}  // namespace amr_perception
