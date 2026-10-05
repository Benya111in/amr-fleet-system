// amr_navigation::PurePursuitController — nav2_core::Controller 플러그인
//   (명세 4.5 "Pure Pursuit 직접 구현").
//
// 얇은 래퍼: 로봇 자세를 경로 프레임(map)으로 옮겨 core::PurePursuit
//   (속도 적응 look-ahead L = clamp(k·v),
// κ = 2y/L², 곡률·목표 감속, 제자리 회전 모드) 호출. 명령
//   원호를 로컬 코스트맵에서 풋프린트로 검사해
// 충돌이 예상되면 PlannerException (BT 가 재계획/복구). 알고리즘은
//   include/amr_navigation/core/pure_pursuit.hpp.
// 발행: local_plan (명령 원호), lookahead_point (geometry_msgs/PointStamped),
//       pure_pursuit/stats (std_msgs/Float64MultiArray [cycle_ms, cte,
//       lookahead, curvature, v, w, mode, d_goal])
#ifndef AMR_NAVIGATION__PURE_PURSUIT_CONTROLLER_HPP_
#define AMR_NAVIGATION__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "amr_navigation/core/pure_pursuit.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"

namespace amr_navigation
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void onPayload(const std_msgs::msg::Float32::SharedPtr msg);
  /// 명령 (v, ω) 원호를 코스트맵 프레임에서 풋프린트 검사. 충돌 시 true.
  bool arcCollides(const core::Pose2D & start, double v, double w) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  std::mutex mutex_;
  core::PurePursuitConfig base_config_;
  core::PurePursuit pp_;
  std::string plan_frame_;
  double transform_tolerance_{0.2};
  double speed_limit_{1e9};
  double v_last_{0.0};
  bool has_last_{false};
  double window_reset_v_{0.3};
  bool use_collision_detection_{true};
  double collision_check_time_{1.0};
  double robot_mass_{47.6};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr payload_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__PURE_PURSUIT_CONTROLLER_HPP_
