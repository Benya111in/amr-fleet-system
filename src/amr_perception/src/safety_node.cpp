// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// safety_node — cmd_vel 최종 게이트 (components.md §3.4 / §4.1 / §5.4, sequences.md §2).
//
// Sub  cmd_vel_smoothed (Twist)              → 제한 후 cmd_vel 로 (유일한 cmd_vel 발행자)
//      scan_filtered (LaserScan, sensor)     → 접근 영역·접촉 가드 판정 + LiDAR 생존
//      camera/depth/points_filtered (PointCloud2, sensor)
//                                            → 전방 LiDAR 평면 아래 물체 (지면 높이로 자름)
//      wheel_odom (Odometry)                 → 측정 속도(접근 영역 가설) + 엔코더 생존
//      perception/tracked_obstacles          → 최소 TTC 기반 연속 감속
//      estop, /fleet/estop (Bool)            → E-stop 래치. transient_local·volatile 발행자 모두
//                                              (구독 2 개, 발행자 GID + 원천 시각으로 중복 제거)
//      imu/data, camera/camera_info, camera/depth/camera_info → 센서 생존 감시
//      safety/dock_exclusion (PolygonStamped, TF 로 풀리는 아무 프레임) → 도킹 예외 (계약 C2)
// Pub  cmd_vel (50 Hz),
//      safety/estop_active (Bool, latched — E-stop 래치 또는 정지형 센서 고장, 계약 C1),
//      safety/zone (UInt8 0 CLEAR · 1 WARNING · 2 CRITICAL · 3 STOP, latched, 변화 시 —
//        components.md §5.4 계약, fleet_adapter_node 가 구독),
//      safety/zone_name (String "CLEAR|WARNING|CRITICAL|STOP", latched, 변화 시 — 표시용),
//      diagnostics (DiagnosticArray, 1 Hz + 변화 시)
// Srv  safety/reset_estop (std_srvs/Trigger)
//
// 판정 로직은 amr_perception/safety_gate.hpp (ROS 비의존, gtest) 에 있다. 이 노드는 메시지 변환,
// TF(센서 → base_link, 예외 다각형 → odom → base_link), 타이밍(50 Hz + 이벤트 즉시 발행)만 한다.

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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
  double rate;           ///< sensors.yaml <name>.update_rate 기본값 [Hz]
  double late_timeout;   ///< robot_params.yaml safety.sensor_timeouts 기본값 [s] → 지연 경고
  int fault_periods;     ///< 이 주기 수 연속 결측 → 고장 (perception.yaml sensor_fault_periods)
  const char * action;
};

// 고장 주기 수 근거: tracking.md §6.8 (부하 시 수신 간격 실측 → 오탐 없는 최소 배수)
const SensorDefault kSensorDefaults[] = {
  {"lidar", "scan_filtered", 10.0, 0.3, 3, "stop"},
  {"wheel_encoder", "wheel_odom", 50.0, 0.06, 10, "stop"},
  {"imu", "imu/data", 100.0, 0.05, 20, "degraded"},
  {"rgb_camera", "camera/camera_info", 30.0, 0.1, 9, "degraded"},
  {"depth_camera", "camera/depth/camera_info", 15.0, 0.2, 6, "degraded"},
};

/// 3D 강체 변환 (깊이 점군: optical → base_link)
struct Rigid3
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
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
    base_link_height_ = declare_parameter<double>("robot.base_link_height", 0.18);
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
    ttc_only_dynamic_ = declare_parameter<bool>("ttc.only_dynamic", true);
    p.command_timeout = declare_parameter<double>("command_timeout", 0.5);
    p.estop_release_requires_reset = declare_parameter<bool>("estop_release_requires_reset", true);
    p.swept_margin = declare_parameter<double>("approach.swept_margin", 0.0);
    p.region_margin = declare_parameter<double>("approach.region_margin", 0.10);
    p.hold_motion_intent = declare_parameter<bool>("approach.hold_motion_intent", true);
    p.measured_velocity_timeout = declare_parameter<double>("approach.measured_timeout", 0.2);
    p.measured_velocity_time_constant =
      declare_parameter<double>("approach.measured_time_constant", 0.1);
    p.self_filter_margin = declare_parameter<double>("self_filter_margin", 0.02);
    p.beam_window_width = declare_parameter<double>("robust.beam_window_width", 0.04);
    p.beam_window = declare_parameter<int>("robust.beam_window", 5);
    p.beam_support = declare_parameter<double>("robust.beam_support", 0.6);
    p.guard_window_width = declare_parameter<double>("robust.guard_window_width", 0.06);
    p.guard_window = declare_parameter<int>("robust.guard_window", 9);
    p.contact_guard_distance = declare_parameter<double>("robust.contact_guard_distance", 0.02);
    p.stop_confirm_frames = declare_parameter<int>("robust.stop_confirm_frames", 2);
    p.immediate_stop_margin = declare_parameter<double>("robust.immediate_stop_margin", 0.05);
    p.cloud_min_points = declare_parameter<int>("depth_cloud.min_points", 3);
    p.cloud_timeout = declare_parameter<double>("depth_cloud.timeout", 0.3);
    p.cloud_max_age = declare_parameter<double>("depth_cloud.max_age", 0.4);
    p.latency_compensation = declare_parameter<bool>("latency_compensation", true);
    p.max_compensation_age = declare_parameter<double>("max_compensation_age", 0.5);
    p.allow_escape = declare_parameter<bool>("allow_escape", true);
    p.escape_horizon = declare_parameter<double>("escape_horizon", 0.3);
    p.escape_max_speed = declare_parameter<double>("escape_max_speed", p.critical_zone_max_speed);
    p.exclusion_stop_distance = declare_parameter<double>("exclusion_stop_distance", 0.10);
    p.exclusion_timeout = declare_parameter<double>("exclusion_timeout", 0.3);
    cloud_enabled_ = declare_parameter<bool>("depth_cloud.enabled", true);
    p.cloud_required = cloud_enabled_;
    cloud_min_height_ = declare_parameter<double>("depth_cloud.min_height", 0.03);
    cloud_max_height_ = declare_parameter<double>("depth_cloud.max_height", 0.40);
    cloud_max_range_ = declare_parameter<double>("depth_cloud.max_range", 3.5);
    rate_hz_ = declare_parameter<double>("rate", 50.0);
    const std::string prefix = declare_parameter<std::string>("frame_prefix", "");
    base_frame_ = prefixedFrame(prefix, declare_parameter<std::string>("base_frame", "base_link"));
    odom_frame_ = prefixedFrame(prefix, declare_parameter<std::string>("odom_frame", "odom"));

    // 센서 감시: 지연 = safety.sensor_timeouts (≤ 0 이면 감시 안 함),
    // 고장 = max(지연, sensor_fault_periods / <sensor>.update_rate) — 주기에서 유도한 연속 결측
    std::vector<SensorWatch> watches;
    std::map<std::string, std::string> topics;
    for (const auto & d : kSensorDefaults) {
      const std::string name = d.name;
      SensorWatch w;
      w.name = name;
      w.late_timeout = declare_parameter<double>("safety.sensor_timeouts." + name, d.late_timeout);
      const double rate = declare_parameter<double>(name + ".update_rate", d.rate);
      const int periods = declare_parameter<int>("sensor_fault_periods." + name, d.fault_periods);
      w.fault_timeout = std::max(w.late_timeout, rate > 0.0 ? periods / rate : 0.0);
      const std::string action = declare_parameter<std::string>(
        "sensor_actions." + name, d.action);
      w.action = action == "degraded" ? SensorFailureAction::kDegraded : SensorFailureAction::kStop;
      topics[name] = declare_parameter<std::string>("sensor_topics." + name, d.topic);
      if (w.late_timeout > 0.0) {
        watches.push_back(w);
        RCLCPP_INFO(
          get_logger(), "센서 %s (%s): 지연 %.3f s, 고장 %.3f s → %s", name.c_str(),
          topics[name].c_str(), w.late_timeout, w.fault_timeout, action.c_str());
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
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double t = now().seconds();
        gate_->sensorHeartbeat("wheel_encoder", t);
        gate_->setMeasuredVelocity(msg->twist.twist.linear.x, msg->twist.twist.angular.z, t);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      topics["imu"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {heartbeat("imu");});
    rgb_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      topics["rgb_camera"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {heartbeat("rgb_camera");});
    depth_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      topics["depth_camera"], rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {heartbeat("depth_camera");});
    if (cloud_enabled_) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("depth_cloud.topic", "camera/depth/points_filtered"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});
    }
    tracked_sub_ = create_subscription<amr_msgs::msg::TrackedObstacleArray>(
      declare_parameter<std::string>("topics.tracked_obstacles", "perception/tracked_obstacles"),
      rclcpp::QoS(10).reliable(),
      [this](amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg) {
        std::vector<TrackTtc> tracks;
        tracks.reserve(msg->obstacles.size());
        for (const auto & o : msg->obstacles) {
          tracks.push_back({static_cast<double>(o.time_to_collision), o.is_dynamic});
        }
        gate_->setMinTtc(minTrackTtc(tracks, ttc_only_dynamic_), now().seconds());
      });
    // E-stop: transient_local 구독(늦게 떠도 latched 마지막 값) + volatile 구독(volatile 발행자).
    // volatile 구독은 두 종류 발행자와 모두 맞으므로 같은 표본이 두 번 올 수 있다
    // → 발행자별 원천 시각으로 거른다.
    for (const auto & [source, topic] : std::vector<std::pair<std::string, std::string>>{
        {"estop", declare_parameter<std::string>("topics.estop", "estop")},
        {"fleet_estop", declare_parameter<std::string>("topics.fleet_estop", "/fleet/estop")}})
    {
      subscribeEstop(source, topic, latched, true);
      subscribeEstop(source, topic, rclcpp::QoS(10).reliable().durability_volatile(), false);
    }
    exclusion_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      declare_parameter<std::string>("topics.dock_exclusion", "safety/dock_exclusion"), 10,
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {onExclusion(*msg);});
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("topics.reset_estop", "safety/reset_estop"),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const ResetResult r = gate_->requestReset();
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
      "안전 게이트 %.0f Hz: 존 %.2f/%.2f/%.2f m (접근 영역, 좌우 여유 %.2f m), 상한 %.2f/%.2f m/s, "
      "τ_crit %.2f s, 접촉 가드 %.2f m, 깊이 점군 %s (지면 %.2f~%.2f m), 감시 센서 %zu 개",
      rate_hz_, p.emergency_stop_distance, p.critical_zone_distance, p.warning_zone_distance,
      p.swept_margin, p.warning_zone_max_speed, p.critical_zone_max_speed, p.ttc_critical,
      p.contact_guard_distance, cloud_enabled_ ? "사용" : "끔", cloud_min_height_,
      cloud_max_height_, watches.size());
  }

private:
  /// 발행 시점: kTimer 매 주기, kOnChange 존/정지 상태가 바뀌거나 속도가 줄어든 경우만 즉시
  enum class Trigger { kOnChange, kTimer };

  void heartbeat(const std::string & name)
  {
    gate_->sensorHeartbeat(name, now().seconds());
  }

  void subscribeEstop(
    const std::string & source, const std::string & topic, const rclcpp::QoS & qos,
    bool transient_local)
  {
    rclcpp::SubscriptionOptions opts;
    opts.event_callbacks.incompatible_qos_callback =
      [this, topic, transient_local](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
        if (transient_local) {
          // volatile 발행자는 volatile 구독이 받는다 (정상)
          RCLCPP_DEBUG(
            get_logger(), "E-stop %s: volatile 발행자 — volatile 구독으로 수신", topic.c_str());
          return;
        }
        ++estop_incompatible_;
        RCLCPP_ERROR(
          get_logger(), "E-stop %s: QoS 가 맞지 않는 발행자 (정책 %d) — reliable 로 발행해야 한다",
          topic.c_str(), static_cast<int>(info.last_policy_kind));
      };
    estop_subs_.push_back(
      create_subscription<std_msgs::msg::Bool>(
        topic, qos,
        [this, source](std_msgs::msg::Bool::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
          onEstop(source, msg->data, info);
        }, opts));
  }

  void onEstop(const std::string & source, bool active, const rclcpp::MessageInfo & info)
  {
    const auto & mi = info.get_rmw_message_info();
    std::string key = source + "/";
    key.append(reinterpret_cast<const char *>(mi.publisher_gid.data), RMW_GID_STORAGE_SIZE);
    const int64_t ts = mi.source_timestamp;
    auto it = estop_seen_.find(key);
    if (ts != 0 && it != estop_seen_.end() && ts <= it->second) {
      return;  // 다른 구독으로 이미 받은 표본(중복) 또는 더 오래된 표본
    }
    estop_seen_[key] = ts;
    gate_->setEstopSource(source, active);
    RCLCPP_WARN(get_logger(), "E-stop 입력 %s = %s", source.c_str(), active ? "true" : "false");
    step(Trigger::kOnChange);
  }

  bool lookup2D(const std::string & target, const std::string & source, Pose2D & out)
  {
    try {
      out = pose2DFromTransform(tf_buffer_->lookupTransform(target, source, tf2::TimePointZero));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s <- %s 실패: %s", target.c_str(), source.c_str(),
        e.what());
      return false;
    }
  }

  /// 센서 장착(고정 조인트) → base_link. 성공한 조회만 캐시한다.
  bool sensorTransform(const std::string & frame, Rigid3 & out)
  {
    if (frame.empty() || frame == base_frame_) {
      out = Rigid3{};
      return true;
    }
    auto it = sensor_tf_cache_.find(frame);
    if (it != sensor_tf_cache_.end()) {
      out = it->second;
      return true;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      out.R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      out.t = Eigen::Vector3d(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      sensor_tf_cache_[frame] = out;
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
    Rigid3 tf;
    if (!sensorTransform(scan.header.frame_id, tf)) {
      return;  // TF 가 없으면 생존 신호도 주지 않는다 → 고장 시 정지 (보수적)
    }
    const Pose2D base_from_lidar{tf.t.x(), tf.t.y(), std::atan2(tf.R(1, 0), tf.R(0, 0))};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<Vec2> beams(scan.ranges.size(), Vec2(nan, nan));
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      beams[i] = base_from_lidar.apply(Vec2(r * std::cos(a), r * std::sin(a)));
    }
    const double inc = std::abs(static_cast<double>(scan.angle_increment));
    const bool circular = inc * static_cast<double>(scan.ranges.size()) >= 2.0 * M_PI - 2.0 * inc;
    const double t = now().seconds();
    // 촬영 시각 (지연 보정). 스탬프가 없거나 미래면 수신 시각
    const double stamp = rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type()).seconds();
    refreshExclusion();
    gate_->setScan(
      beams, base_from_lidar.translation(), inc, circular, stamp > 0.0 && stamp <= t ? stamp : t);
    gate_->sensorHeartbeat("lidar", t);
    step(Trigger::kOnChange);  // 상태가 바뀌면 다음 주기를 기다리지 않고 즉시 발행
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    Rigid3 tf;
    if (!sensorTransform(msg.header.frame_id, tf)) {
      return;
    }
    std::vector<Vec2> pts;
    if (msg.width * msg.height > 0) {
      pts.reserve(static_cast<std::size_t>(msg.width) * msg.height);
      sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
      const double r2 = cloud_max_range_ * cloud_max_range_;
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        const Eigen::Vector3d q(*ix, *iy, *iz);
        if (!q.allFinite()) {
          continue;
        }
        const Eigen::Vector3d b = tf.R * q + tf.t;
        const double z_ground = b.z() + base_link_height_;
        if (z_ground < cloud_min_height_ || z_ground > cloud_max_height_ ||
          b.x() * b.x() + b.y() * b.y() > r2)
        {
          continue;  // 바닥, 차체보다 높은 점, 먼 점은 안전 판정에 쓰지 않는다
        }
        pts.emplace_back(b.x(), b.y());
      }
    }
    refreshExclusion();
    const double t = now().seconds();
    const double stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()).seconds();
    gate_->setCloud(pts, stamp > 0.0 && stamp <= t ? stamp : t, t);
    step(Trigger::kOnChange);
  }

  void onExclusion(const geometry_msgs::msg::PolygonStamped & msg)
  {
    const std::string frame = msg.header.frame_id.empty() ? base_frame_ : msg.header.frame_id;
    std::vector<Vec2> poly;
    poly.reserve(msg.polygon.points.size());
    for (const auto & pt : msg.polygon.points) {
      poly.emplace_back(pt.x, pt.y);
    }
    exclusion_received_ = now().seconds();
    // odom 에 고정해 두고 스캔·점군마다 현재 base_link 로 다시 옮긴다 (수신 사이 로봇 이동 반영)
    Pose2D odom_from_frame;
    if (frame != base_frame_ && lookup2D(odom_frame_, frame, odom_from_frame)) {
      exclusion_odom_.clear();
      for (const auto & q : poly) {
        exclusion_odom_.push_back(odom_from_frame.apply(q));
      }
      have_exclusion_odom_ = true;
      refreshExclusion();
      return;
    }
    // odom 이 없으면(단독 시험 등) 수신 시점의 base_link 로 바로 옮긴다
    Pose2D base_from_frame;
    if (frame != base_frame_ && !lookup2D(base_frame_, frame, base_from_frame)) {
      return;
    }
    have_exclusion_odom_ = false;
    for (auto & q : poly) {
      q = base_from_frame.apply(q);
    }
    gate_->setExclusionPolygon(poly, exclusion_received_);
  }

  void refreshExclusion()
  {
    if (!have_exclusion_odom_ || now().seconds() - exclusion_received_ > 1.0) {
      return;
    }
    Pose2D base_from_odom;
    if (!lookup2D(base_frame_, odom_frame_, base_from_odom)) {
      return;
    }
    std::vector<Vec2> poly;
    poly.reserve(exclusion_odom_.size());
    for (const auto & q : exclusion_odom_) {
      poly.push_back(base_from_odom.apply(q));
    }
    gate_->setExclusionPolygon(poly, exclusion_received_);
  }

  /// 판정 + 발행. 타이머가 정확히 rate(50 Hz)로 발행하고, 이벤트(스캔·점군·명령·E-stop·reset)는
  /// 존/정지 상태가 바뀌었거나 출력 속도가 줄어든 경우에만 즉시 한 번 더 발행한다.
  void step(Trigger trigger)
  {
    const rclcpp::Time t = now();
    const SafetyStatus st = gate_->evaluate(t.seconds());
    const bool slowed = std::abs(st.command.linear) < std::abs(last_linear_) - 0.05 ||
      std::abs(st.command.angular) < std::abs(last_angular_) - 0.1;
    if (trigger == Trigger::kOnChange && published_once_ && st.zone == last_zone_ &&
      st.estop_active == last_estop_ && !slowed)
    {
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = st.command.linear;
    cmd.angular.z = st.command.angular;
    cmd_pub_->publish(cmd);
    last_linear_ = st.command.linear;
    last_angular_ = st.command.angular;

    const bool changed = !published_once_ || st.zone != last_zone_ ||
      st.estop_active != last_estop_ || st.degraded != last_degraded_ ||
      st.failed_sensors.size() != last_failed_ || st.late_sensors.size() != last_late_ ||
      st.cloud_stale != last_cloud_stale_;
    if (!published_once_ || st.zone != last_zone_) {
      std_msgs::msg::UInt8 level;
      level.data = static_cast<uint8_t>(st.zone);
      zone_pub_->publish(level);
      std_msgs::msg::String name;
      name.data = zoneName(st.zone);
      zone_name_pub_->publish(name);
      if (st.zone == SafetyZone::kStop) {
        RCLCPP_WARN(
          get_logger(), "근접 STOP: %s (D %.3f, 가드 %.3f, 예외 %.3f m)", joinReasons(st).c_str(),
          st.min_distance, st.contact_distance, st.exclusion_distance);
      }
    }
    if (!published_once_ || st.estop_active != last_estop_) {
      std_msgs::msg::Bool b;
      b.data = st.estop_active;
      estop_pub_->publish(b);
      if (st.estop_active) {
        RCLCPP_WARN(get_logger(), "E-stop 활성: %s", joinReasons(st).c_str());
      } else {
        RCLCPP_INFO(get_logger(), "E-stop 해제");
      }
    }
    if (st.late_sensors.size() != last_late_ && !st.late_sensors.empty()) {
      RCLCPP_WARN(get_logger(), "센서 지연: %s", join(st.late_sensors).c_str());
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
    last_late_ = st.late_sensors.size();
    last_cloud_stale_ = st.cloud_stale;
  }

  static std::string join(const std::vector<std::string> & items)
  {
    std::string s;
    for (const auto & r : items) {
      s += (s.empty() ? "" : ",") + r;
    }
    return s.empty() ? "-" : s;
  }

  static std::string joinReasons(const SafetyStatus & st) {return join(st.reasons);}

  void publishDiagnostics(const SafetyStatus & st, const rclcpp::Time & t)
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = t;
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = std::string(get_namespace()) + "/safety_node";
    ds.hardware_id = get_namespace();
    const bool warn = st.degraded || st.zone != SafetyZone::kClear || !st.late_sensors.empty() ||
      estop_incompatible_ > 0;
    if (st.estop_active) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      ds.message = "E-stop: " + joinReasons(st);
    } else if (st.zone == SafetyZone::kStop) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      ds.message = "근접 STOP: " + join(st.stop_causes);
    } else if (warn || st.cloud_stale) {
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      ds.message = std::string(zoneName(st.zone)) + (st.degraded ? " / 저속(센서 고장)" : "") +
        (st.late_sensors.empty() ? "" : " / 센서 지연") +
        (st.cloud_stale ? " / 깊이 점군 없음(전진 저속)" : "") +
        (estop_incompatible_ > 0 ? " / E-stop QoS 불일치 발행자" : "");
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
    kv("zone", zoneName(st.zone));
    kv("min_distance_m", std::to_string(st.min_distance));
    kv("lidar_distance_m", std::to_string(st.lidar_distance));
    kv("depth_distance_m", std::to_string(st.cloud_distance));
    kv("contact_distance_m", std::to_string(st.contact_distance));
    kv("exclusion_distance_m", std::to_string(st.exclusion_distance));
    kv("min_ttc_s", std::to_string(st.min_ttc));
    kv("speed_limit_mps", std::to_string(st.speed_limit));
    kv("estop_latched", st.estop_latched ? "true" : "false");
    kv("proximity_stop", st.proximity_stop ? "true" : "false");
    kv("stop_causes", join(st.stop_causes));
    kv("failed_sensors", join(st.failed_sensors));
    kv("late_sensors", join(st.late_sensors));
    kv("depth_cloud_stale", st.cloud_stale ? "true" : "false");
    kv("estop_incompatible_publishers", std::to_string(estop_incompatible_));
    kv("reasons", joinReasons(st));
    arr.status.push_back(ds);
    diag_pub_->publish(arr);
  }

  double rate_hz_{50.0};
  double base_link_height_{0.18};
  bool cloud_enabled_{true};
  double cloud_min_height_{0.03};
  double cloud_max_height_{0.40};
  double cloud_max_range_{3.5};
  std::string base_frame_;
  std::string odom_frame_;
  std::unique_ptr<SafetyGate> gate_;
  std::map<std::string, Rigid3> sensor_tf_cache_;
  std::map<std::string, int64_t> estop_seen_;
  int estop_incompatible_{0};
  std::vector<Vec2> exclusion_odom_;
  bool have_exclusion_odom_{false};
  double exclusion_received_{-1.0};
  rclcpp::Time last_diag_{0, 0, RCL_ROS_TIME};
  bool published_once_{false};
  bool ttc_only_dynamic_{true};   ///< TTC 제한에 동적 트랙만 (minTrackTtc)
  SafetyZone last_zone_{SafetyZone::kClear};
  bool last_estop_{false};
  bool last_degraded_{false};
  std::size_t last_failed_{0};
  std::size_t last_late_{0};
  bool last_cloud_stale_{false};
  double last_linear_{0.0};
  double last_angular_{0.0};

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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
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
