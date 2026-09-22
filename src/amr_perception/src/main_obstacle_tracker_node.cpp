// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// obstacle_tracker_node 실행 파일.
// 노드 구현: src/obstacle_tracker_node.cpp (amr_perception_nodes 라이브러리).

#include "amr_perception/nodes.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(amr_perception::createObstacleTrackerNode(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
