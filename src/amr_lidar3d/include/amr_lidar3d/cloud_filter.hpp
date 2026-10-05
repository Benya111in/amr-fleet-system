// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 3D LiDAR 점군 전처리 순수 로직 (rclcpp 비의존 — sensor_msgs 메시지 구조체만 쓴다, gtest 대상).
//
// Fortress(ign-sensors 6) gpu_lidar 점군의 특성
// (보고서 「3D LiDAR SLAM 도입 방안」, sensor.md 실측):
//   - 필드 x·y·z·intensity(FLOAT32), ring(UINT16), 점별 time 없음. 정렬 패딩으로 point_step 32 B.
//   - 범위 밖 빔은 ±inf 좌표로 남고 하나라도 있으면 is_dense=false.
// filterNonFinite 는 x·y·z 가 모두 유한한 점만 원래 순서대로 남긴 비조직(height 1) 점군을 만든다.
// 필드 배치(오프셋·point_step·ring 등 다른 필드)는 그대로 복사하므로 ring 이 보존된다.
// 출력 is_dense = true.

#ifndef AMR_LIDAR3D__CLOUD_FILTER_HPP_
#define AMR_LIDAR3D__CLOUD_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace amr_lidar3d
{

/// 한 번 거른 결과의 점 수.
struct FilterStats
{
  std::size_t input_points{0};   ///< width × height
  std::size_t kept_points{0};    ///< x·y·z 가 모두 유한한 점
  std::size_t dropped_points{0};  ///< 비유한(±inf, NaN) 점
};

/// 입력을 거를 수 없는 이유 (kOk 가 아니면 출력은 손대지 않는다).
enum class FilterError
{
  kOk,
  kMissingXyz,        ///< x / y / z 필드 중 하나가 없다
  kUnsupportedType,   ///< x·y·z 가 FLOAT32·FLOAT64 가 아니거나 서로 형이 다르다, count ≠ 1
  kBadLayout,         ///< 필드가 point_step 밖, row_step < width·point_step, data 가 짧다
  kBigEndian,         ///< 빅엔디언 데이터 (Gazebo·x86 은 리틀엔디언 — 지원하지 않는다)
};

/// 오류 이름 (로그용).
const char * toString(FilterError error);

/// x·y·z 가 유한한 점만 남긴다. frame_id_override 가 비어 있지 않으면 header.frame_id 를 바꾼다.
FilterError filterNonFinite(
  const sensor_msgs::msg::PointCloud2 & in, const std::string & frame_id_override,
  sensor_msgs::msg::PointCloud2 & out, FilterStats & stats);

}  // namespace amr_lidar3d

#endif  // AMR_LIDAR3D__CLOUD_FILTER_HPP_
