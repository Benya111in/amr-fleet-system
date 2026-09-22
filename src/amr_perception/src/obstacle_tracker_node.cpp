// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// obstacle_tracker_node — LiDAR 동적 장애물 추적 (components.md §3.4 / §5.4).
//
// Sub  scan_filtered (sensor_msgs/LaserScan, sensor QoS)
//      /map (nav_msgs/OccupancyGrid, transient_local) — 정적 배경 분리
//      odometry/filtered_map (nav_msgs/Odometry) — 자기 속도 (TTC, 시선 회전 보정)
//      plan (nav_msgs/Path) — TTC 로봇 궤적
// Pub  perception/tracked_obstacles (amr_msgs/TrackedObstacleArray, 스캔마다 = 10 Hz, reliable)
//      perception/tracked_markers (visualization_msgs/MarkerArray)
// TF   tracking_frame(odom) ← lidar, tracking_frame ← base, map ← tracking_frame (스캔 스탬프)
//
// 추적은 odom 프레임에서 한다 (AMCL 보정 점프가 가짜 속도가 되지 않도록). 출력만 map 으로 바꾼다.
// 알고리즘은 amr_perception/{scan_clustering, obstacle_tracker, ttc}.hpp (ROS 비의존) 에 있다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_perception/obstacle_tracker.hpp"
#include "amr_perception/nodes.hpp"
#include "amr_perception/ros_conversions.hpp"
#include "amr_perception/scan_clustering.hpp"
#include "amr_perception/ttc.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_perception
{

class ObstacleTrackerNode : public rclcpp::Node
{
public:
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_tracker_node", options)
  {
    const std::string prefix = declare_parameter<std::string>("frame_prefix", "");
    tracking_frame_ =
      prefixedFrame(prefix, declare_parameter<std::string>("tracking_frame", "odom"));
    base_frame_ = prefixedFrame(prefix, declare_parameter<std::string>("base_frame", "base_link"));
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 5.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    occupied_threshold_ = declare_parameter<int>("map_occupied_threshold", 65);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    seg_.lambda = declare_parameter<double>("segmentation.lambda_deg", 10.0) * M_PI / 180.0;
    seg_.sigma_r = declare_parameter<double>("segmentation.sigma_r", 0.03);
    // config/sensors.yaml 을 함께 넘기면 LiDAR 거리 잡음 σ 를 거기서 가져온다 (단일 출처)
    const double lidar_sigma = declare_parameter<double>("lidar.noise_stddev", -1.0);
    if (lidar_sigma > 0.0) {
      seg_.sigma_r = lidar_sigma;
    }
    seg_.merge_min_gap = declare_parameter<double>("segmentation.merge_min_gap", 0.10);
    seg_.min_points_near = declare_parameter<int>("segmentation.min_points_near", 3);
    seg_.min_points_far = declare_parameter<int>("segmentation.min_points_far", 2);
    seg_.far_range = declare_parameter<double>("segmentation.far_range", 6.0);
    seg_.max_extent = declare_parameter<double>("segmentation.max_extent", 1.5);
    seg_.max_range = declare_parameter<double>("segmentation.max_range", 12.0);
    seg_.background_radius = declare_parameter<double>("segmentation.background_radius", 0.10);
    seg_.overlap_radius = declare_parameter<double>("segmentation.overlap_radius", 0.25);
    seg_.align_to_map = declare_parameter<bool>("background.align_to_map", true);
    seg_.align_max_correspondence =
      declare_parameter<double>("background.max_correspondence", 0.30);
    seg_.align_huber = declare_parameter<double>("background.huber", 0.05);
    seg_.align_iterations = declare_parameter<int>("background.iterations", 8);
    seg_.align_min_points = declare_parameter<int>("background.min_points", 40);
    seg_.align_max_translation = declare_parameter<double>("background.max_translation", 0.30);
    seg_.align_max_rotation = declare_parameter<double>("background.max_rotation", 0.06);
    seg_.align_max_residual = declare_parameter<double>("background.max_residual", 0.05);
    seg_.background_k_sigma = declare_parameter<double>("background.k_sigma", 3.0);
    seg_.background_max_radius = declare_parameter<double>("background.max_radius", 0.35);
    prior_xy_min_ = declare_parameter<double>("background.prior_sigma_xy_min", 0.03);
    prior_xy_max_ = declare_parameter<double>("background.prior_sigma_xy_max", 0.20);
    prior_yaw_min_ = declare_parameter<double>("background.prior_sigma_yaw_min", 0.005);
    prior_yaw_max_ = declare_parameter<double>("background.prior_sigma_yaw_max", 0.05);

    model_.bias_mu = declare_parameter<double>("cluster_model.bias_mu", 0.15);
    model_.sigma_delta = declare_parameter<double>("cluster_model.sigma_delta", 0.10);
    model_.sigma_lat = declare_parameter<double>("cluster_model.sigma_lat", 0.03);
    model_.sigma_r = seg_.sigma_r;
    model_.sigma_floor = declare_parameter<double>("cluster_model.sigma_floor", 0.02);
    model_.sigma_seg = declare_parameter<double>("cluster_model.sigma_seg", 0.03);
    model_.min_radius = declare_parameter<double>("cluster_model.min_radius", 0.10);
    model_.sigma_occluded = declare_parameter<double>("cluster_model.sigma_occluded", 0.20);
    model_.occlusion_margin = declare_parameter<double>("cluster_model.occlusion_margin", 0.30);

    TrackerParams tp;
    tp.q = declare_parameter<double>("kf.q", 0.25);
    tp.init_velocity_std = declare_parameter<double>("kf.init_velocity_std", 1.5);
    tp.gate_chi2 = declare_parameter<double>("association.gate_chi2", 9.21);
    tp.pd_visible = declare_parameter<double>("association.pd_visible", 0.9);
    tp.pd_occluded = declare_parameter<double>("association.pd_occluded", 0.5);
    tp.far_detection_range = declare_parameter<double>("association.far_detection_range", 8.0);
    tp.lambda_birth = declare_parameter<double>("association.lambda_birth", 0.01);
    tp.v_phys = declare_parameter<double>("association.v_phys", 3.0);
    tp.confirm_hits = declare_parameter<int>("lifecycle.confirm_hits", 3);
    tp.confirm_window = declare_parameter<int>("lifecycle.confirm_window", 5);
    tp.min_confirm_age = declare_parameter<double>("lifecycle.min_confirm_age", 0.2);
    tp.max_misses = declare_parameter<int>("lifecycle.max_misses", 5);
    tp.max_misses_occluded = declare_parameter<int>("lifecycle.max_misses_occluded", 15);
    tp.velocity_window = declare_parameter<int>("dynamic.window", 10);
    tp.velocity_min_samples = declare_parameter<int>("dynamic.min_samples", 4);
    tp.velocity_chi2 = declare_parameter<double>("dynamic.chi2", 13.82);
    tp.v_min = declare_parameter<double>("dynamic.v_min", 0.15);
    tp.sigma_delta = model_.sigma_delta;
    tp.dynamic_consecutive = declare_parameter<int>("dynamic.consecutive", 2);
    tp.dynamic_release = declare_parameter<int>("dynamic.release", 5);
    tp.heading_min_speed = declare_parameter<double>("output.heading_min_speed", 0.1);
    const auto beta = declare_parameter<std::vector<double>>(
      "output.confidence_beta", std::vector<double>{-2.0, 4.0, -0.5, 1.0});
    for (std::size_t i = 0; i < 4 && i < beta.size(); ++i) {
      tp.confidence_beta[i] = beta[i];
    }
    tp.confidence_sigma_ref = declare_parameter<double>("output.confidence_sigma_ref", 0.3);
    publish_tentative_ = declare_parameter<bool>("output.publish_tentative", false);
    tracker_ = std::make_unique<ObstacleTracker>(tp);

    ttc_.horizon = declare_parameter<double>("ttc.horizon", 5.0);
    ttc_.step = declare_parameter<double>("ttc.step", 0.1);
    ttc_.k_sigma = declare_parameter<double>("ttc.k_sigma", 1.0);
    ttc_.sigma_cap = declare_parameter<double>("ttc.sigma_cap", 0.5);
    ttc_.robot_radius = declare_parameter<double>("ttc.robot_radius", 0.361);
    ttc_.min_robot_speed = declare_parameter<double>("ttc.min_robot_speed", 0.2);
    ttc_.max_path_deviation = declare_parameter<double>("ttc.max_path_deviation", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<amr_msgs::msg::TrackedObstacleArray>(
      declare_parameter<std::string>("topics.output", "perception/tracked_obstacles"),
      rclcpp::QoS(10).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("topics.markers", "perception/tracked_markers"), 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter<std::string>("topics.scan", "scan_filtered"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(*msg);});
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter<std::string>("topics.map", "/map"), latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {onMap(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("topics.odometry", "odometry/filtered_map"), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = *msg;
        odom_received_ = now();
      });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      declare_parameter<std::string>("topics.plan", "plan"), 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        plan_ = *msg;
        plan_received_ = now();
      });

    RCLCPP_INFO(
      get_logger(), "추적 프레임 %s → 출력 %s, base %s (q=%.2f, gate=%.2f, v_min=%.2f)",
      tracking_frame_.c_str(), output_frame_.c_str(), base_frame_.c_str(), tp.q, tp.gate_chi2,
      tp.v_min);
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    Pose2D origin{
      msg.info.origin.position.x, msg.info.origin.position.y,
      yawFromQuaternion(msg.info.origin.orientation)};
    if (!msg.header.frame_id.empty() && msg.header.frame_id != output_frame_) {
      RCLCPP_WARN(
        get_logger(), "/map 프레임 %s 가 output_frame %s 와 다르다 — output_frame 을 맞출 것",
        msg.header.frame_id.c_str(), output_frame_.c_str());
    }
    map_.build(
      static_cast<int>(msg.info.width), static_cast<int>(msg.info.height),
      msg.info.resolution, origin, msg.data, occupied_threshold_);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(
      get_logger(), "정적 지도 거리 LUT 생성 %ux%u @ %.3f m (%.1f ms)", msg.info.width,
      msg.info.height, msg.info.resolution, ms);
  }

  bool lookup(
    const std::string & target, const std::string & source, const rclcpp::Time & t,
    Pose2D & out)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target, source, t, rclcpp::Duration::from_seconds(tf_timeout_));
      out = pose2DFromTransform(tf);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "TF %s <- %s 실패: %s", target.c_str(),
        source.c_str(), e.what());
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(scan.header.stamp, get_clock()->get_clock_type());
    Pose2D tracking_from_lidar;
    Pose2D tracking_from_base;
    if (!lookup(tracking_frame_, scan.header.frame_id, stamp, tracking_from_lidar) ||
      !lookup(tracking_frame_, base_frame_, stamp, tracking_from_base))
    {
      return;
    }
    // map ← tracking: 배경 분리와 출력 변환.
    // 실패하면 직전 값(없으면 배경 분리 생략, 추적 프레임 출력)
    Pose2D map_from_tracking;
    const bool have_map_tf = lookup(output_frame_, tracking_frame_, stamp, map_from_tracking);
    if (have_map_tf) {
      last_map_from_tracking_ = map_from_tracking;
      have_last_map_tf_ = true;
    } else if (have_last_map_tf_) {
      map_from_tracking = last_map_from_tracking_;
    }
    const bool to_map = have_map_tf || have_last_map_tf_;
    const bool map_usable = map_.valid() && to_map;

    LaserScanData data;
    data.angle_min = scan.angle_min;
    data.angle_increment = scan.angle_increment;
    data.range_min = scan.range_min;
    data.range_max = scan.range_max;
    data.ranges = scan.ranges;
    // 위치 추정 공분산 (odometry/filtered_map) → 국소 정합 사전분포 (하·상한으로 자른다)
    PosePrior prior;
    prior.sigma_xy = prior_xy_max_;
    prior.sigma_yaw = prior_yaw_max_;
    if (odom_received_.nanoseconds() > 0 && (now() - odom_received_).seconds() <= odom_timeout_) {
      const auto & c = odom_.pose.covariance;
      prior.sigma_xy = std::clamp(
        std::sqrt(std::max(c[0], c[7])), prior_xy_min_, prior_xy_max_);
      prior.sigma_yaw = std::clamp(std::sqrt(std::max(c[35], 0.0)), prior_yaw_min_, prior_yaw_max_);
    }
    MapAlignment alignment;
    const auto clusters = extractClusters(
      data, tracking_from_lidar, map_usable ? &map_ : nullptr, map_from_tracking, seg_, model_,
      prior, &alignment);
    if (map_usable && seg_.align_to_map) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "지도 정합: 보정 (%.3f, %.3f) m, %.2f°, 대응 %d, 잔차 σ %.3f m, 성공 %d "
        "(시도 (%.3f, %.3f) m, %.2f°, 사전 σ %.3f m, %.2f°)",
        alignment.correction.x, alignment.correction.y, alignment.correction.yaw * 180.0 / M_PI,
        alignment.correspondences, alignment.residual_sigma, alignment.valid,
        alignment.attempted.x, alignment.attempted.y, alignment.attempted.yaw * 180.0 / M_PI,
        prior.sigma_xy, prior.sigma_yaw * 180.0 / M_PI);
    }

    // 자기 운동: base 속도(odom twist, base 프레임) → LiDAR 속도 (추적 프레임)
    double v_lin = 0.0;
    double w_ang = 0.0;
    if (odom_received_.nanoseconds() > 0 && (now() - odom_received_).seconds() <= odom_timeout_) {
      v_lin = odom_.twist.twist.linear.x;
      w_ang = odom_.twist.twist.angular.z;
    }
    const Pose2D base_from_lidar = tracking_from_base.inverse().compose(tracking_from_lidar);
    const Vec2 r = base_from_lidar.translation();
    const Vec2 v_sensor_base(v_lin - w_ang * r.y(), w_ang * r.x());
    EgoState ego;
    ego.sensor_position = tracking_from_lidar.translation();
    ego.sensor_velocity = tracking_from_base.rotate(v_sensor_base);

    tracker_->update(stamp.seconds(), clusters, ego);
    const auto tracks = tracker_->outputs(!publish_tentative_);

    // TTC: 추적 프레임에서 경로와 로봇 자세로
    std::vector<Vec2> path;
    if (plan_received_.nanoseconds() > 0 && (now() - plan_received_).seconds() <= plan_timeout_ &&
      !plan_.poses.empty())
    {
      Pose2D tracking_from_plan;
      const std::string plan_frame = plan_.header.frame_id.empty() ? output_frame_ :
        plan_.header.frame_id;
      if (lookup(
          tracking_frame_, plan_frame, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
          tracking_from_plan))
      {
        path.reserve(plan_.poses.size());
        for (const auto & ps : plan_.poses) {
          path.push_back(
            tracking_from_plan.apply(Vec2(ps.pose.position.x, ps.pose.position.y)));
        }
      }
    }
    const RobotMotionModel robot = RobotMotionModel::select(
      path, tracking_from_base.translation(), tracking_from_base.yaw, v_lin, ttc_);

    const Pose2D out_tf = to_map ? map_from_tracking : Pose2D{};
    amr_msgs::msg::TrackedObstacleArray out;
    out.header.stamp = scan.header.stamp;
    out.header.frame_id = to_map ? output_frame_ : tracking_frame_;
    out.obstacles.reserve(tracks.size());
    std::vector<double> ttcs;
    for (const auto & t : tracks) {
      ObstacleMotion om;
      om.position = t.position;
      om.velocity = t.velocity;
      om.P_pp = t.covariance.topLeftCorner<2, 2>();
      om.P_pv = t.covariance.topRightCorner<2, 2>();
      om.P_vv = t.covariance.bottomRightCorner<2, 2>();
      om.q = tracker_->params().q;
      om.radius = t.radius;
      const TtcResult ttc = computeTimeToCollision(om, robot, ttc_);
      ttcs.push_back(ttc.ttc);

      amr_msgs::msg::TrackedObstacle o;
      o.header = out.header;
      o.track_id = t.id;
      const Vec2 p = out_tf.apply(t.position);
      const Vec2 v = out_tf.rotate(t.velocity);
      o.position.x = p.x();
      o.position.y = p.y();
      o.position.z = 0.0;
      o.velocity.x = v.x();
      o.velocity.y = v.y();
      o.velocity.z = 0.0;
      o.heading = static_cast<float>(normalizeAngle(t.heading + out_tf.yaw));
      o.confidence = static_cast<float>(t.confidence);
      o.is_dynamic = t.is_dynamic;
      o.time_to_collision = static_cast<float>(ttc.ttc);
      out.obstacles.push_back(o);
    }
    pub_->publish(out);
    if (publish_markers_) {
      publishMarkers(out, tracks);
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(
      get_logger(), "스캔 처리 %.2f ms, 클러스터 %zu, 트랙 %zu (출력 %zu)", ms, clusters.size(),
      tracker_->size(), tracks.size());
  }

  void publishMarkers(
    const amr_msgs::msg::TrackedObstacleArray & out, const std::vector<TrackOutput> & tracks)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = out.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    for (std::size_t i = 0; i < out.obstacles.size(); ++i) {
      const auto & o = out.obstacles[i];
      const double radius = tracks[i].radius;
      visualization_msgs::msg::Marker body;
      body.header = out.header;
      body.ns = "tracks";
      body.id = static_cast<int>(o.track_id);
      body.type = visualization_msgs::msg::Marker::CYLINDER;
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position = o.position;
      body.pose.position.z = 0.5;
      body.pose.orientation.w = 1.0;
      body.scale.x = body.scale.y = 2.0 * radius;
      body.scale.z = 1.0;
      body.color.a = 0.6F;
      body.color.r = o.is_dynamic ? 1.0F : 0.5F;
      body.color.g = o.is_dynamic ? 0.2F : 0.5F;
      body.color.b = o.is_dynamic ? 0.1F : 0.5F;
      body.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(body);

      visualization_msgs::msg::Marker arrow = body;
      arrow.ns = "velocity";
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.pose = geometry_msgs::msg::Pose();
      arrow.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point a;
      a.x = o.position.x;
      a.y = o.position.y;
      a.z = 1.0;
      geometry_msgs::msg::Point b = a;
      b.x += o.velocity.x;  // 1 s 뒤 위치
      b.y += o.velocity.y;
      arrow.points = {a, b};
      arrow.scale.x = 0.05;
      arrow.scale.y = 0.12;
      arrow.scale.z = 0.12;
      arrow.color.a = 1.0F;
      arr.markers.push_back(arrow);

      visualization_msgs::msg::Marker text = body;
      text.ns = "label";
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z = 1.3;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = 0.25;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
      char buf[96];
      const double speed = std::hypot(o.velocity.x, o.velocity.y);
      if (std::isfinite(o.time_to_collision)) {
        std::snprintf(
          buf, sizeof(buf), "#%u %.2f m/s TTC %.1f s", o.track_id, speed, o.time_to_collision);
      } else {
        std::snprintf(buf, sizeof(buf), "#%u %.2f m/s", o.track_id, speed);
      }
      text.text = buf;
      arr.markers.push_back(text);
    }
    marker_pub_->publish(arr);
  }

  std::string tracking_frame_;
  std::string base_frame_;
  std::string output_frame_;
  double tf_timeout_{0.05};
  double plan_timeout_{5.0};
  double odom_timeout_{0.5};
  int occupied_threshold_{65};
  bool publish_markers_{true};
  bool publish_tentative_{false};

  SegmentationParams seg_;
  ClusterModelParams model_;
  double prior_xy_min_{0.03};
  double prior_xy_max_{0.20};
  double prior_yaw_min_{0.005};
  double prior_yaw_max_{0.05};
  TtcParams ttc_;
  std::unique_ptr<ObstacleTracker> tracker_;
  StaticMapDistance map_;
  Pose2D last_map_from_tracking_;
  bool have_last_map_tf_{false};

  nav_msgs::msg::Odometry odom_;
  rclcpp::Time odom_received_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path plan_;
  rclcpp::Time plan_received_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<amr_msgs::msg::TrackedObstacleArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
};

rclcpp::Node::SharedPtr createObstacleTrackerNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ObstacleTrackerNode>(options);
}

}  // namespace amr_perception
