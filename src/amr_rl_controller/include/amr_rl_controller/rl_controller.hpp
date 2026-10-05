// amr_rl_controller::RLController — nav2_core::Controller 플러그인 (명세 5장 보너스 1).
//
// 얇은 ROS 계층: 구독한 스캔·트랙·safety 상태·프로파일러 상태와 전역 경로로 계약 상태를 만들고
// (core/state.hpp), 결정 엔진(core/decision_engine.hpp)이 10 Hz 로 ONNX 정책을 부른다.
// 정책을 쓸 수 없는 결정 주기(입력 무효·추론 실패·시간 초과·비유한 출력)는 같은 이름의
// 파라미터를 읽는 내부 amr_navigation::DWAController 가 명령을 낸다 (rl.dwa_fallback, 기본 참).
//
// 파라미터 (<name>.rl.*): model_path, mode (B|C), intra_op_threads, max_inference_ms,
//   hysteresis_beta, require_metadata, dwa_fallback, scan_topic, scan_timeout, track_timeout,
//   profiler_timeout, goal_reset_distance, transform_tolerance, stats_topic.
//   v_cap 은 DWA 와 같은 <name>.max_vel_x (계약 v_cap = 1.0) 와 setSpeedLimit 의 최솟값.
// 구독 (controller_server 네임스페이스 상대): scan_filtered (LaserScan),
//   perception/tracked_obstacles, safety/zone (UInt8, latched), diagnostics (safety_node 항목),
//   velocity_profiler/state (Float64MultiArray, [3] ω_ref, [4] a_ref).
// 발행: rl_controller/stats (Float64MultiArray, 순서는 rl_controller.cpp 의 kStatsFields).
#ifndef AMR_RL_CONTROLLER__RL_CONTROLLER_HPP_
#define AMR_RL_CONTROLLER__RL_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_navigation/dwa_controller.hpp"
#include "amr_rl_controller/core/decision_engine.hpp"
#include "amr_rl_controller/policy/onnx_policy.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"

namespace amr_rl_controller
{

class RLController : public nav2_core::Controller
{
public:
  RLController() = default;
  ~RLController() override = default;

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

  /// 누적 계수 (시험·진단): 결정, DWA 폴백(사유별 5), 계약 폴백 셀, 마스크 밖 행동.
  struct Counters
  {
    std::size_t ticks{0};
    std::size_t decisions{0};
    std::array<std::size_t, 5> dwa{{0, 0, 0, 0, 0}};
    std::size_t fallback_cell{0};
    std::size_t invalid_action{0};
    std::size_t align{0};
  };
  Counters counters() const;
  /// 마지막 틱의 엔진 결과 (시험·진단).
  TickResult lastResult() const;

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onTracks(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg);
  void onZone(const std_msgs::msg::UInt8::SharedPtr msg);
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void onProfilerState(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  /// 계약 상태를 채운다. 실패 이유는 in.state_ok / in.state_error 로.
  void buildInput(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    TickInput & in);
  void publishStats(const TickResult & r, double cycle_ms, const TickInput & in);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RLController")};
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<OnnxPolicy> policy_;
  std::unique_ptr<DecisionEngine> engine_;
  std::shared_ptr<amr_navigation::DWAController> dwa_;
  bool dwa_fallback_{true};

  mutable std::mutex mutex_;          // 경로·엔진·계수
  nav_msgs::msg::Path plan_;
  bool have_goal_{false};
  double goal_x_{0.0};
  double goal_y_{0.0};
  std::string goal_frame_;
  double goal_reset_distance_{0.25};
  double transform_tolerance_{0.2};
  double v_cap_op_{contract::kVMaxOp};
  double speed_limit_{kInf};
  Counters counters_;
  TickResult last_;

  // 센서 상태 (콜백 스레드)
  mutable std::mutex sensor_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  amr_msgs::msg::TrackedObstacleArray::SharedPtr tracks_;
  SafetyState safety_;
  double profiler_a_v_{0.0};
  double profiler_a_w_{0.0};
  double profiler_w_ref_{0.0};
  bool profiler_have_w_{false};
  rclcpp::Time profiler_stamp_{0, 0, RCL_ROS_TIME};
  double profiler_dt_{0.02};          // velocity_profiler rate 50 Hz
  double scan_timeout_{0.3};
  double track_timeout_{0.5};
  double profiler_timeout_{0.2};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr zone_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr profiler_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
  rclcpp::Time last_summary_{0, 0, RCL_ROS_TIME};
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__RL_CONTROLLER_HPP_
