// costmap_scan_filter_node — 코스트맵 센서 입력 전처리 (docs/algorithms/costmap.md §3, §5.2).
//   scan_filtered ─► 게이트 중앙값(core::denoiseRanges) ─► scan_costmap
//                                        (지역 코스트맵: 모든 장애물)
//                 └► 동적 트랙 제외 ─► scan_costmap_static (전역 코스트맵)
//   camera/depth/points_filtered ─► 동적 트랙 제외 ─► camera/depth/points_static
//                                        (전역 코스트맵 깊이 레이어)
// 동적 트랙 = perception/tracked_obstacles 중 속도 ≥ dynamic_min_speed 이면서
// (is_dynamic 이거나 속도 ≥ dynamic_fast_speed), 메시지 나이 ≤ track_timeout.
// 센서 시각으로 등속 예측한 중심에서 dynamic_radius 안의 끝점·점을 뺀다 — 지나가는 사람·차량은
// 지역 계획기(DWA VO/TTC + 지역 코스트맵)가 피하고, 전역 경로가 1 Hz·TTC 재계획마다 그 순간
// 위치를 돌아가며 뒤집히지 않게 한다. 서 있는(비동적) 사람·물체는 그대로 전역에 들어가 재계획
// 대상이다. 트랙 프레임 → 센서 프레임 TF 는 센서 시각 기준(없으면 최신)이고, TF 가 없으면 빼지
// 않고 그대로 낸다 (안전 쪽).
//
// Sub  scan_filtered, camera/depth/points_filtered (sensor QoS), perception/tracked_obstacles
// Pub  scan_costmap, scan_costmap_static, camera/depth/points_static (sensor QoS)
#ifndef AMR_NAVIGATION__COSTMAP_SCAN_FILTER_NODE_HPP_
#define AMR_NAVIGATION__COSTMAP_SCAN_FILTER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_navigation/core/scan_denoise.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_navigation
{

class CostmapScanFilterNode : public rclcpp::Node
{
public:
  explicit CostmapScanFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const core::ScanDenoiseConfig & config() const {return config_;}
  std::size_t removedBeams() const {return removed_beams_;}
  std::size_t removedPoints() const {return removed_points_;}

private:
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void onTracks(const amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr & msg);
  /// 시각 stamp 로 예측한 동적 트랙 중심 (트랙 프레임). 없거나 오래되면 빈 목록.
  std::vector<core::Point2D> dynamicCenters(const rclcpp::Time & stamp, std::string & frame);
  /// target ← source 변환: 시각 stamp, 없으면 최신. 둘 다 없으면 false.
  bool lookup(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    tf2::Transform & out) const;

  core::ScanDenoiseConfig config_;
  bool exclude_dynamic_{true};
  double dynamic_min_speed_{0.2};
  double dynamic_fast_speed_{0.5};
  double dynamic_radius_{0.55};
  double track_timeout_{0.5};
  std::size_t removed_beams_{0};
  std::size_t removed_points_{0};

  std::mutex mutex_;
  amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr tracks_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr tracks_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__COSTMAP_SCAN_FILTER_NODE_HPP_
