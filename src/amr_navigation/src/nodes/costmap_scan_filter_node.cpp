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
  // 명세 4.7 "재계획하여 회피". 임계 유도는 nav2_params.yaml 주석 참고.
  admit_ttc_ = declare_parameter("admit_ttc", 0.0);
  admit_release_s_ = declare_parameter("admit_release_s", 1.0);
  predict_horizon_ = declare_parameter("predict_horizon", 2.0);
  predict_dt_ = declare_parameter("predict_dt", 0.25);
  predict_z_ = declare_parameter("predict_z", 0.30);
  if (admit_ttc_ > 0.0) {
    predicted_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter("predicted_topic", std::string("perception/predicted_obstacles")),
      rclcpp::SensorDataQoS());
  }
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
  const rclcpp::Time & stamp, std::string & frame, std::vector<PredictedTrack> * predicted)
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
  // 명세 4장 7절 "경로를 재계획(Replanning)하여 회피" 를 위한 **선별 재진입**.
  //
  // 종전: 빠른 동적 트랙은 전부 전역 코스트맵에서 지웠다("지나가는 사람은 지역 계획기가").
  // 그 결과 A* 는 자기를 부른 장애물을 볼 수 없었고, TTC 재계획 가지를 켜도 같은 경로가
  // 다시 나왔다 — 그래서 nav_ttc_bt 기본값이 false 다.
  //
  // 지금: 지우는 대상을 "모든 빠른 동적 트랙" 에서 "**내 경로와 만나지 않을** 빠른 동적
  // 트랙" 으로 좁힌다. 추적기의 time_to_collision 은 로봇의 계획 경로를 따라가는 예측으로
  // 계산되므로(obstacle_tracker_node.cpp:288-322, ttc.cpp), 유한하고 작으면 "경로 위에서
  // 실제로 만난다" 는 뜻이다. 지나가는 사람은 TTC = inf 라 종전대로 계속 빠진다
  // = 전역 경로 요동은 그대로 막힌다.
  //
  // **ttc_path_based 가 false 면 재진입하지 않는다.** 그때의 TTC 는 직선 외삽 또는 정지
  // 모형이라 "내 경로 위에서 만나는가" 가 아닌 다른 양이고, 특히 양보로 멈춘 로봇 옆을
  // 지나가는 보행자가 유한 TTC 를 받아 전역에 찍히면 경로가 이유 없이 흔들린다.
  const bool may_admit = admit_ttc_ > 0.0 && predicted != nullptr && tracks->ttc_path_based;
  const double t_now = stamp.seconds();
  for (const auto & o : tracks->obstacles) {
    // 동적: 추적기 분류(is_dynamic) 이거나 빠른 트랙 (분류가 늦는 첫 0.5–1 s·ID 교체 직후 대비)
    const double sp = std::hypot(o.velocity.x, o.velocity.y);
    if (sp < dynamic_min_speed_ || (!o.is_dynamic && sp < dynamic_fast_speed_)) {
      continue;
    }
    if (may_admit) {
      const double ttc = o.time_to_collision;
      const bool conflict = std::isfinite(ttc) && ttc >= 0.0 && ttc <= admit_ttc_;
      const auto it = admitted_.find(o.track_id);
      // 유지 창: 한 번 재진입한 트랙은 TTC 가 잠깐 튀어도 admit_release_s 동안 유지한다.
      // 없으면 마킹과 소거가 번갈아 일어나 전역 경로가 떨린다.
      const bool hold = it != admitted_.end() && (t_now - it->second) <= admit_release_s_;
      if (conflict) {
        admitted_[o.track_id] = t_now;
      } else if (!hold && it != admitted_.end()) {
        admitted_.erase(it);
      }
      if (conflict || hold) {
        ++admitted_count_;
        // 현재 위치만 남기면 늦다. 기하 유도: 마킹이 경로를 실제로 막는 것은 보행자가
        // 경로에서 (robot_radius 0.361 + r_obs 0.25) = 0.611 m 안일 때이고 v = 1.0 이면
        // TTC ≈ 0.61 s 다. 그런데 재계획에는 코스트맵 갱신 0.2 s + BT 주기 1.0 s +
        // A* 0.013 s = 1.21 s 가 든다. **0.61 < 1.21 이므로 현재 위치 마킹으로는
        // 원리적으로 늦는다.** 그래서 명세 9장이 요구하는 대로 **예측** 점유를 넣는다.
        // TrackedObstacle 에 반경이 없다 — 이미 스캔 제외에 쓰는 dynamic_radius_ 를 그대로
        // 쓴다 (사람 발자국 반경 이상으로 설정돼 있고 test_costmap_config 가 고정한다).
        predicted->push_back(
          PredictedTrack{o.position.x, o.position.y, o.velocity.x, o.velocity.y, dynamic_radius_});
        continue;               // 현재 끝점도 지우지 않는다
      }
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
  std::vector<PredictedTrack> predicted;
  const auto centers = dynamicCenters(stamp, frame, &predicted);
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
  publishPredicted(predicted, frame, stamp);
}

void CostmapScanFilterNode::publishPredicted(
  const std::vector<PredictedTrack> & predicted, const std::string & frame,
  const rclcpp::Time & stamp)
{
  // 명세 9장 "동적 장애물을 추적하고 **예측 기반** 회피가 수행되는가".
  // 재진입시킨 트랙이 앞으로 predict_horizon 동안 쓸고 지나갈 영역을 점구름으로 낸다.
  // 전역 코스트맵의 별도 관측 소스로만 들어간다 — **지역 코스트맵은 구독하지 않는다**
  // (지역 회피는 종전대로 DWA 의 VO/TTC 가 한다. 불변식 A/B).
  // 프레임을 모르면(트랙 없음·만료) 아무것도 내지 않는다 — 빈 frame_id 로 발행하면
  // 코스트맵이 변환에 실패해 경고만 쌓인다.
  if (!predicted_pub_ || frame.empty() || predict_horizon_ <= 0.0) {
    return;
  }
  if (predicted.empty()) {
    {
      sensor_msgs::msg::PointCloud2 empty;   // 비면 빈 구름 — 옛 마킹이 남지 않게
      empty.header.frame_id = frame;
      empty.header.stamp = stamp;
      empty.height = 1;
      empty.width = 0;
      sensor_msgs::PointCloud2Modifier m(empty);
      m.setPointCloud2FieldsByString(1, "xyz");
      m.resize(0);
      predicted_pub_->publish(empty);
    }
    return;
  }
  // 시간 표본마다 원판 둘레를 찍는다 (속을 채우면 점이 과하다 — 코스트맵은 셀 단위라
  // 둘레만 찍어도 팽창이 안쪽을 메운다).
  const int n_t = std::max(1, static_cast<int>(std::lround(predict_horizon_ / predict_dt_)));
  constexpr int kRing = 8;
  std::vector<std::array<float, 3>> pts;
  pts.reserve(predicted.size() * static_cast<std::size_t>(n_t) * kRing);
  for (const auto & p : predicted) {
    for (int k = 1; k <= n_t; ++k) {
      const double t = static_cast<double>(k) * predict_dt_;
      const double cx = p.x + p.vx * t, cy = p.y + p.vy * t;
      for (int a = 0; a < kRing; ++a) {
        const double th = 2.0 * M_PI * a / kRing;
        pts.push_back(
          {static_cast<float>(cx + p.r * std::cos(th)),
            static_cast<float>(cy + p.r * std::sin(th)),
            static_cast<float>(predict_z_)});
      }
    }
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (const auto & q : pts) {
    *ix = q[0];
    *iy = q[1];
    *iz = q[2];
    ++ix;
    ++iy;
    ++iz;
  }
  predicted_pub_->publish(cloud);
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
