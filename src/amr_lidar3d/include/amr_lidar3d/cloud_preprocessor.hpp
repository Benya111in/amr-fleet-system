// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// cloud_preprocessor_node 팩토리 — 실행 파일(컴포넌트 등록)과 rclcpp 통합 시험이
// 같은 노드를 만든다.
// 노드 설명: src/cloud_preprocessor.cpp 머리말.

#ifndef AMR_LIDAR3D__CLOUD_PREPROCESSOR_HPP_
#define AMR_LIDAR3D__CLOUD_PREPROCESSOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace amr_lidar3d
{

/// amr_lidar3d::CloudPreprocessor 노드를 만든다 (이름 lidar3d_preprocessor).
std::shared_ptr<rclcpp::Node> createCloudPreprocessor(const rclcpp::NodeOptions & options);

}  // namespace amr_lidar3d

#endif  // AMR_LIDAR3D__CLOUD_PREPROCESSOR_HPP_
