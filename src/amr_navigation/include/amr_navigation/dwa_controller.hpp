// amr_navigation::DWAController — nav2_core::Controller 플러그인
//   (명세 4.4 "DWA 핵심 로직 직접 구현").
//
// 얇은 래퍼: 전역 경로(map) → 로컬 코스트맵 프레임(odom) 창 변환, 추적
//   장애물(perception/tracked_obstacles)
// → 코스트맵 프레임 등속 예측, core::DwaPlanner (동적 창 →
//   샘플링 → 원호 롤아웃 → 풋프린트 충돌 → VO 제외 →
// 비용 → 선택) 호출. 알고리즘은 include/amr_navigation/core/dwa.hpp.
// 구독: perception/tracked_obstacles (amr_msgs/TrackedObstacleArray), payload/mass
//   (std_msgs/Float32, latched)
// 발행: local_plan (nav_msgs/Path, 선택 궤적), dwa/stats (std_msgs/Float64MultiArray, 아래 순서)
//   [cycle_ms, n_samples, n_valid, n_collision, n_vo_rejected, vo_saturated, best_ttc, v, w,
//    d_goal, n_recentered, yield_state (0 clear / 1 yield / 2 committed), yield_stop_distance,
//    yield_zone_entry (로봇 → 교차 구간 입구, 없으면 −1e9 — 음수면 이미 구간 안), yield_obstacle
//    (상한을 정한 장애물 색인, 없으면 −1)]
//   (best_ttc·yield_stop_distance 는 없으면 −1: 유한하지 않은 값)
#ifndef AMR_NAVIGATION__DWA_CONTROLLER_HPP_
#define AMR_NAVIGATION__DWA_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <string>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_navigation/core/dwa.hpp"
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

/// 전역 경로를 코스트맵 프레임으로 옮겨 로봇 근처부터 horizon 까지 자르는 공용 도우미.
class PlanWindow
{
public:
  void setPlan(const nav_msgs::msg::Path & plan);
  bool empty() const {return plan_.poses.empty();}
  const std::string & frame() const {return plan_.header.frame_id;}
  /// T(costmap ← plan) 로 변환해 로봇 최근접점부터 horizon
  ///   [m] 까지. end_is_goal: 마지막 점 포함 여부.
  std::vector<core::Pose2D> window(
    const core::Pose2D & T, const core::Pose2D & robot, double horizon, bool & end_is_goal);

private:
  nav_msgs::msg::Path plan_;
  std::vector<core::Pose2D> poses_;   // plan 프레임
  std::size_t hint_{0};
  bool fresh_{true};
};

class DWAController : public nav2_core::Controller
{
public:
  DWAController() = default;
  ~DWAController() override = default;

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
  void onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg);
  void onPayload(const std_msgs::msg::Float32::SharedPtr msg);
  std::vector<core::DynamicObstacle> obstaclesInFrame(const std::string & frame);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DWAController")};
  rclcpp::Clock::SharedPtr clock_;

  std::mutex mutex_;
  core::DwaConfig base_config_;
  core::DwaPlanner planner_;
  PlanWindow plan_;
  double path_horizon_{6.0};
  double transform_tolerance_{0.2};
  bool allow_unknown_{true};
  double speed_limit_{1e9};
  double max_vel_x_{1.0};
  bool has_last_{false};
  double v_last_{0.0};
  double w_last_{0.0};
  int no_valid_count_{0};
  int no_valid_limit_{10};

  // 동적 장애물
  std::mutex obs_mutex_;
  amr_msgs::msg::TrackedObstacleArray::SharedPtr obstacles_;
  double obstacle_radius_{0.25};
  double dynamic_fast_speed_{0.5};
  double track_timeout_{0.5};
  /// [s] 통로 예측용 속도의 지수 평활 시간 상수 (0 = 끔 — 원시 속도를 그대로 쓴다)
  double yield_velocity_tau_{0.5};
  struct VelocityFilter
  {
    double vx{0.0};
    double vy{0.0};
    double stamp{-1.0};
  };
  std::unordered_map<int, VelocityFilter> vel_filter_;
  /// 직전 주기의 횡단 양보 속도 상한 (오르는 속도를 가속 한계로 묶는다 — core::DwaInput 참고)
  double yield_limit_last_{std::numeric_limits<double>::infinity()};
  double robot_mass_{47.6};

  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr payload_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__DWA_CONTROLLER_HPP_
