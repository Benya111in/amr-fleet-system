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
//    (상한을 정한 장애물 색인, 없으면 −1), n_obstacles (DWA 가 이번 주기에 본 동적 장애물 수),
//    track_age_s (그 트랙 메시지의 나이 — track_timeout 을 넘으면 목록이 비워진다),
//    [17] yield_decision (0 없음 · 1 go · 2 hold · 3 retreat), [18] go 여유 [s] (없으면 −1),
//    --- 횡단 게이트 (yield_gate, core::crossing_gate) ---
//    [19] gate_phase (0 open · 1 approach · 2 hold · 3 committed · 4 retreat),
//    [20] gate_reason (core::GateReason: 0 없음 · 1 clear · 2 앞으로 옴 · 3 방향 모름 ·
//         4 되돌아옴 · 5 강제 커밋 · 6 정지 자리 위협 · 7 판정 커밋 · 8 이미 반경 안),
//    [21] gate_window_s (지금 상태에서 노출 구간 출구까지 주행 시간 [s], 없으면 −1),
//    [22] gate_hold_m (정지점까지 경로 거리 [m], 없으면 −1e9),
//    [23] gate_threats (닿을 수 있는 트랙 수),
//    [24] gate_tracks (게이트가 본 트랙 수 — 기억 속 트랙 포함),
//    [25] gate_zone_in_m (구간 시작까지 [m], 없으면 −1e9),
//    [26] gate_zone_out_m (구간 끝까지 [m], 없으면 −1e9),
//    [27] gate_exempt (VO·TTC 면제 트랙 수)]
//   (best_ttc·yield_stop_distance 는 없으면 −1: 유한하지 않은 값)
#ifndef AMR_NAVIGATION__DWA_CONTROLLER_HPP_
#define AMR_NAVIGATION__DWA_CONTROLLER_HPP_

#include <array>
#include <deque>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <string>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/track_velocity_filter.hpp"
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
  /// 로봇이 창 시작보다 뒤로 갈 수 있을 때(횡단 게이트의 경로 후진) 직전 색인에서 이만큼 뒤부터
  /// 최근접을 찾는다. 0 이면 예전처럼 앞쪽만 본다.
  void setBackSearch(std::size_t points) {back_search_ = points;}

private:
  nav_msgs::msg::Path plan_;
  std::vector<core::Pose2D> poses_;   // plan 프레임
  std::size_t hint_{0};
  bool fresh_{true};
  std::size_t back_search_{0};
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
  /// 횡단 게이트용 트랙: 확정 트랙 전부(속도·is_dynamic 으로 거르지 않음) + 방금 사라진 트랙의
  /// 기억. 진행 방향은 최근 gate_line_window 동안의 **변위**로 정한다 (평균 속력 ≥ vknown 일
  /// 때만 부호를 믿는다).
  std::vector<core::GateTrack> gateTracksInFrame(const std::string & frame);

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
  double last_track_age_{-1.0};   ///< [s] 마지막으로 본 트랙 메시지의 나이 (dwa/stats 진단)
  // 트랙 속도 평활은 core/track_velocity_filter.hpp 의 순수 구조로 옮겼다 — 갱신 규칙이 여기
  // 인라인으로 있는 동안 단위 계약을 걸 수 없었고, "같은 메시지 재관측" 을 불연속으로 오인하는
  // 결함이 숨어 있었다 (test_track_velocity_filter.cpp 참조).
  std::unordered_map<int, core::TrackVelocityFilter> vel_filter_;
  /// 직전 주기의 횡단 양보 속도 상한 (오르는 속도를 가속 한계로 묶는다 — core::DwaInput 참고)
  double yield_limit_last_{std::numeric_limits<double>::infinity()};
  int yield_decision_last_{0};    ///< 직전 주기의 kCommitted 결정 (core::YieldDecision)
  rclcpp::Time yield_decision_since_{0, 0, RCL_ROS_TIME};   ///< 그 결정이 시작된 시각
  double yield_decision_dwell_{0.8};   ///< [s] 결정 최소 유지 시간 (진동 억제)
  double robot_mass_{47.6};

  // 횡단 게이트 (core::crossing_gate) — yield_gate 가 true 일 때만
  bool gate_enable_{false};
  double gate_lookahead_{8.0};        ///< [m] 게이트가 볼 경로 길이 (창 = max(path_horizon, 이 값))
  double gate_line_window_{1.5};      ///< [s] 진행 방향을 정하는 변위 창
  double gate_track_memory_{2.0};     ///< [s] 사라진 트랙을 방향 모름 위협으로 기억하는 시간
  double gate_vknown_{0.3};           ///< [m/s] 변위 평균 속력이 이 이상이면 방향 부호를 믿는다
  core::GateState gate_state_;
  struct TrackHistory
  {
    std::deque<std::array<double, 3>> samples;   // (stamp, x, y) 코스트맵 프레임
    double first_seen{-1.0};
    double last_seen{-1.0};
    double last_moving{-1.0};
    double x{0.0};
    double y{0.0};
    bool in_latest{false};
  };
  std::unordered_map<int, TrackHistory> gate_hist_;
  double gate_hist_stamp_{-1.0};      ///< 마지막으로 기록에 넣은 트랙 메시지 스탬프

  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr payload_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_pub_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__DWA_CONTROLLER_HPP_
