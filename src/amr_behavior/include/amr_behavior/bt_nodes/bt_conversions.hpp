// XML 리터럴 → ROS 메시지 변환 (BT.CPP v3 convertFromString 특수화) 과 평면 자세 도우미.
//
// PoseStamped 리터럴 형식 (세미콜론 구분):
//   "x;y;yaw"          → frame_id = "map"
//   "frame;x;y;yaw"    → frame_id 지정
// 서브트리에 고정 자세(예: 시험용 대기 위치)를 XML 로 넘길 때 쓴다. 보통은 실행기가 파라미터에서
// 읽은 자세를 블랙보드에 PoseStamped 로 넣으므로 문자열 변환을 거치지 않는다.
#ifndef AMR_BEHAVIOR__BT_NODES__BT_CONVERSIONS_HPP_
#define AMR_BEHAVIOR__BT_NODES__BT_CONVERSIONS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace amr_behavior
{

/// 평면 자세 → PoseStamped (z = 0, roll = pitch = 0).
inline geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw * 0.5);
  p.pose.orientation.w = std::cos(yaw * 0.5);
  return p;
}

/// 쿼터니언의 z 축 회전각 [rad].
inline double yawOf(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace amr_behavior

namespace BT
{

template<>
inline geometry_msgs::msg::PoseStamped convertFromString(StringView str)
{
  const std::vector<StringView> parts = splitString(str, ';');
  if (parts.size() == 3) {
    return amr_behavior::makePose(
      "map", convertFromString<double>(parts[0]), convertFromString<double>(parts[1]),
      convertFromString<double>(parts[2]));
  }
  if (parts.size() == 4) {
    return amr_behavior::makePose(
      std::string(parts[0].data(), parts[0].size()), convertFromString<double>(parts[1]),
      convertFromString<double>(parts[2]), convertFromString<double>(parts[3]));
  }
  throw RuntimeError("PoseStamped 리터럴은 \"x;y;yaw\" 또는 \"frame;x;y;yaw\" 형식이어야 한다");
}

}  // namespace BT

#endif  // AMR_BEHAVIOR__BT_NODES__BT_CONVERSIONS_HPP_
