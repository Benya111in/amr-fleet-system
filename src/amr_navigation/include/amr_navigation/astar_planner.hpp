// amr_navigation::AStarPlanner — nav2_core::GlobalPlanner 플러그인 (명세 4.4 "A* 직접 구현").
//
// 얇은 래퍼: 코스트맵 → core::AStar (8-연결 비용 인지 A*)
//   → core::PathSmoother (숏컷·경사 하강 평활화) →
// nav_msgs/Path. 알고리즘은 include/amr_navigation/core/ 에 있고 ROS 의존이 없다.
// 파라미터 (planner_server.<name>.*, docs/algorithms/astar.md 표):
//   cost_weight, allow_unknown, unknown_cost, tolerance, max_iterations, max_planning_time,
//   allow_corner_cutting, allow_start_in_inscribed, use_final_approach_orientation,
//   smoother.{enable_shortcut, shortcut_max_length, shortcut_cost_ratio, clearance_cost_margin,
//             output_spacing, enable_smoothing, w_data, w_smooth, w_clearance,
//             clearance_cost_threshold, max_iterations, tolerance}
#ifndef AMR_NAVIGATION__ASTAR_PLANNER_HPP_
#define AMR_NAVIGATION__ASTAR_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_navigation/core/astar.hpp"
#include "amr_navigation/core/path_smoother.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_navigation
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;

  std::mutex mutex_;   // 파라미터 갱신 ↔ 계획 동시 실행 보호
  core::AStar astar_;
  core::PathSmoother smoother_;
  double tolerance_{0.25};
  double max_planning_time_{0.4};
  bool use_final_approach_orientation_{false};
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__ASTAR_PLANNER_HPP_
