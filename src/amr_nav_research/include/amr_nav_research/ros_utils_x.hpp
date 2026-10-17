// ROS ↔ 코어 변환 공용 함수 — amr_navigation/ros_utils.hpp 의 사본 (이름공간만 다르다).
// 원본은 amr_navigation_plugins 라이브러리 안에 있다. 그 라이브러리를 링크하면 이 플러그인을
// pluginlib 으로 올릴 때 원본 플러그인들의 등록이 이 라이브러리 이름으로 함께 실행된다 — 같은
// controller_server 에 원본 팔과 연구 팔을 함께 올리는 시험대에서 등록이 엉킬 수 있어 옮겼다.
#ifndef AMR_NAV_RESEARCH__ROS_UTILS_X_HPP_
#define AMR_NAV_RESEARCH__ROS_UTILS_X_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_nav_research
{

namespace core = amr_navigation::core;

/// Costmap2D 의 비소유 뷰 (같은 메모리 배치, 복사 없음).
///   호출자가 코스트맵 뮤텍스를 잡고 있어야 한다.
core::CostGrid costmapView(const nav2_costmap_2d::Costmap2D & costmap);

double yawOf(const geometry_msgs::msg::Pose & pose);
core::Pose2D toPose2D(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose toPoseMsg(const core::Pose2D & pose);

std::vector<core::Pose2D> toPoses2D(const nav_msgs::msg::Path & path);
nav_msgs::msg::Path toPathMsg(
  const std::vector<core::Pose2D> & poses, const std_msgs::msg::Header & header);

/// 코스트맵 풋프린트(패딩 포함) → 코어 다각형. 반경 모드면 원을 16각형으로 근사.
std::vector<core::Point2D> footprintOf(nav2_costmap_2d::Costmap2DROS & costmap_ros);

/// 외접 반경에서의 inflation 비용 (InflationLayer 가 없으면 0 → 항상 외곽선 검사).
uint8_t circumscribedCost(nav2_costmap_2d::Costmap2DROS & costmap_ros);

/// frame 간 2D 강체 변환 T(target ← source). 실패하면 false.
bool lookupTransform2D(
  const tf2_ros::Buffer & tf, const std::string & target, const std::string & source,
  double timeout_s, core::Pose2D & out);

/// 파라미터 선언(없을 때만) 후 값 읽기.
template<typename T>
T param(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  const T & default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(default_value));
  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

}  // namespace amr_nav_research

#endif  // AMR_NAV_RESEARCH__ROS_UTILS_X_HPP_
