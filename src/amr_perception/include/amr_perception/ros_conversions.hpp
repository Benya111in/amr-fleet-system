// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// ROS 메시지 ↔ 순수 라이브러리 타입 변환 (노드 래퍼 전용, 헤더 전용).

#ifndef AMR_PERCEPTION__ROS_CONVERSIONS_HPP_
#define AMR_PERCEPTION__ROS_CONVERSIONS_HPP_

#include <cmath>
#include <string>

#include "amr_perception/geometry2d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace amr_perception
{

/// 쿼터니언 → yaw (ZYX 오일러의 z)
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

/// yaw → 쿼터니언 (roll = pitch = 0)
inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  q.x = 0.0;
  q.y = 0.0;
  return q;
}

/// 3D 변환의 평면 성분 (x, y, yaw). 2D LiDAR 는 본체 규약 rpy 0 장착이라 평면 변환으로 충분하다.
inline Pose2D pose2DFromTransform(const geometry_msgs::msg::TransformStamped & tf)
{
  return Pose2D{
    tf.transform.translation.x, tf.transform.translation.y,
    yawFromQuaternion(tf.transform.rotation)};
}

/// 프레임 접두사 적용 ("amr_01/" + "odom"). 이미 '/' 를 포함하거나 비어 있으면 그대로.
inline std::string prefixedFrame(const std::string & prefix, const std::string & frame)
{
  if (prefix.empty() || frame.empty() || frame.find('/') != std::string::npos) {
    return frame;
  }
  return prefix + frame;
}

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__ROS_CONVERSIONS_HPP_
