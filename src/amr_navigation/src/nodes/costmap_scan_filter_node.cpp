#include "amr_navigation/costmap_scan_filter_node.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer_interface.h"

namespace amr_navigation
{
namespace
{
tf2::Transform toTf(const geometry_msgs::msg::TransformStamped & t)
{
  const auto & q = t.transform.rotation;
  const auto & p = t.transform.translation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(p.x, p.y, p.z));
}
}  // namespace

CostmapScanFilterNode::CostmapScanFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("costmap_scan_filter_node", options)
{
  const std::string in = declare_parameter("input_topic", std::string("scan_filtered"));
  const std::string out = declare_parameter("output_topic", std::string("scan_costmap"));
  const std::string static_out =
    declare_parameter("static_output_topic", std::string("scan_costmap_static"));
  const std::string cloud_in =
    declare_parameter("cloud_input_topic", std::string("camera/depth/points_filtered"));
  const std::string cloud_out =
    declare_parameter("cloud_output_topic", std::string("camera/depth/points_static"));
  const std::string tracks =
    declare_parameter("tracks_topic", std::string("perception/tracked_obstacles"));
  config_.half_window = static_cast<int>(declare_parameter("half_window", 5));
  config_.range_gate = declare_parameter("range_gate", 0.15);
  config_.min_support = static_cast<int>(declare_parameter("min_support", 3));
  exclude_dynamic_ = declare_parameter("exclude_dynamic", true);
  dynamic_min_speed_ = declare_parameter("dynamic_min_speed", 0.2);
  dynamic_fast_speed_ = declare_parameter("dynamic_fast_speed", 0.5);
  dynamic_radius_ = declare_parameter("dynamic_radius", 0.55);
  track_timeout_ = declare_parameter("track_timeout", 0.5);

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out, rclcpp::SensorDataQoS());
  static_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(static_out, rclcpp::SensorDataQoS());
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    in, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(msg);});
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_in, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(msg);});
  tracks_sub_ = create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    tracks, rclcpp::QoS(5),
    [this](const amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr msg) {onTracks(msg);});
  RCLCPP_INFO(
    get_logger(),
    "costmap_scan_filter_node: %s -> %s (±%d beams, gate %.3f m, support %d), dynamic exclusion %s "
    "(r %.2f m, v ≥ %.2f m/s) -> %s, %s", sub_->get_topic_name(), pub_->get_topic_name(),
    config_.half_window, config_.range_gate, config_.min_support, exclude_dynamic_ ? "on" : "off",
    dynamic_radius_, dynamic_min_speed_, static_pub_->get_topic_name(),
    cloud_pub_->get_topic_name());
}

void CostmapScanFilterNode::onTracks(
  const amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tracks_ = msg;
}

std::vector<core::Point2D> CostmapScanFilterNode::dynamicCenters(
  const rclcpp::Time & stamp, std::string & frame)
{
  std::vector<core::Point2D> out;
  amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr tracks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tracks = tracks_;
  }
  if (!exclude_dynamic_ || !tracks || tracks->obstacles.empty()) {
    return out;
  }
  const rclcpp::Time t_tracks(tracks->header.stamp, stamp.get_clock_type());
  const double dt = (stamp - t_tracks).seconds();
  if (dt > track_timeout_ || dt < -track_timeout_) {
    return out;
  }
  frame = tracks->header.frame_id;
  for (const auto & o : tracks->obstacles) {
    // 동적: 추적기 분류(is_dynamic) 이거나 빠른 트랙 (분류가 늦는 첫 0.5–1 s·ID 교체 직후 대비)
    const double sp = std::hypot(o.velocity.x, o.velocity.y);
    if (sp < dynamic_min_speed_ || (!o.is_dynamic && sp < dynamic_fast_speed_)) {
      continue;
    }
    out.push_back({o.position.x + o.velocity.x * dt, o.position.y + o.velocity.y * dt});
  }
  return out;
}

bool CostmapScanFilterNode::lookup(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  tf2::Transform & out) const
{
  // 센서 시각의 변환 (로봇이 움직이는 동안 최신 변환과 v·지연 만큼 다르다). 아직 없으면
  // (map→odom 이 센서보다 늦게 옴) 최신 변환, 그것도 없으면 false.
  for (const auto & t : {tf2_ros::fromRclcpp(stamp), tf2::TimePointZero}) {
    try {
      out = toTf(tf_->lookupTransform(target, source, t, tf2::durationFromSec(0.0)));
      return true;
    } catch (const tf2::TransformException &) {
    }
  }
  return false;
}

void CostmapScanFilterNode::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  // 360° 스캔(마지막 빔 다음이 첫 빔)이면 양 끝을 이웃으로 본다
  const double span = std::abs(msg->angle_increment) * static_cast<double>(msg->ranges.size());
  const bool wrap = span > 2.0 * M_PI - 1.5 * std::abs(msg->angle_increment);
  auto out = std::make_unique<sensor_msgs::msg::LaserScan>(*msg);
  out->ranges = core::denoiseRanges(msg->ranges, config_, wrap);
  auto stat = std::make_unique<sensor_msgs::msg::LaserScan>(*out);
  pub_->publish(std::move(out));

  std::string frame;
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const auto centers = dynamicCenters(stamp, frame);
  tf2::Transform T;
  // 트랙 프레임(map) → 스캔 프레임 (lidar_link, 수평). 변환 없음 → 빼지 않는다 (전역도 그대로 본다)
  if (!centers.empty() && lookup(msg->header.frame_id, frame, stamp, T)) {
    std::vector<core::Point2D> local;
    local.reserve(centers.size());
    for (const auto & c : centers) {
      const tf2::Vector3 v = T * tf2::Vector3(c.x, c.y, 0.0);
      local.push_back({v.x(), v.y()});
    }
    removed_beams_ += core::excludeDiscsFromScan(
      stat->ranges, stat->angle_min, stat->angle_increment, local, dynamic_radius_);
  }
  static_pub_->publish(std::move(stat));
}

void CostmapScanFilterNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::string frame;
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const auto centers = dynamicCenters(stamp, frame);
  if (centers.empty()) {
    cloud_pub_->publish(*msg);
    return;
  }
  tf2::Transform T;
  // 점군 프레임(깊이 광학) → 트랙 프레임(map)
  if (!lookup(frame, msg->header.frame_id, stamp, T)) {
    cloud_pub_->publish(*msg);
    return;
  }
  const double r2 = dynamic_radius_ * dynamic_radius_;
  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out->header = msg->header;
  out->fields = msg->fields;
  out->is_bigendian = msg->is_bigendian;
  out->point_step = msg->point_step;
  out->height = 1;
  out->is_dense = msg->is_dense;
  const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
  out->data.reserve(msg->data.size());
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
  std::size_t kept = 0;
  for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z) {
    const tf2::Vector3 p = T * tf2::Vector3(*x, *y, *z);
    bool drop = false;
    for (const auto & c : centers) {
      if ((p.x() - c.x) * (p.x() - c.x) + (p.y() - c.y) * (p.y() - c.y) <= r2) {
        drop = true;
        break;
      }
    }
    if (drop) {
      ++removed_points_;
      continue;
    }
    const auto * src = &msg->data[i * msg->point_step];
    out->data.insert(out->data.end(), src, src + msg->point_step);
    ++kept;
  }
  out->width = static_cast<uint32_t>(kept);
  out->row_step = out->point_step * out->width;
  cloud_pub_->publish(std::move(out));
}

}  // namespace amr_navigation
