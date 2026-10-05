// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// C++ 노드 팩토리 (amr_perception_nodes 라이브러리). 실행 파일(src/main_*.cpp)과 rclcpp 통합
// 테스트(test/test_ros_nodes.cpp)가 같은 노드 구현을 쓴다. 인터페이스는 각 구현 파일 머리 주석과
// docs/architecture/components.md §5.4.

#ifndef AMR_PERCEPTION__NODES_HPP_
#define AMR_PERCEPTION__NODES_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace amr_perception
{

/// obstacle_tracker_node: scan_filtered → perception/tracked_obstacles (KF + GNN + TTC)
rclcpp::Node::SharedPtr createObstacleTrackerNode(const rclcpp::NodeOptions & options);

/// safety_node: cmd_vel_smoothed → cmd_vel (존·TTC·E-stop·센서 타임아웃 게이트)
rclcpp::Node::SharedPtr createSafetyNode(const rclcpp::NodeOptions & options);

/// pointcloud_filter_node: camera/depth/image_raw → camera/depth/points_filtered
rclcpp::Node::SharedPtr createPointcloudFilterNode(const rclcpp::NodeOptions & options);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__NODES_HPP_
